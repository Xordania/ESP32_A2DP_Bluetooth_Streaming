#include "sdkconfig.h"

#if CONFIG_BTDM_CONTROLLER_MODE_BLE_ONLY || CONFIG_BTDM_CONTROLLER_MODE_BTDM

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "bluetooth_ble.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_log.h"

static const char *TAG = "BLE_DISCOVERY";

// Queue for discovered BLE devices
static QueueHandle_t discovered_ble_device_queue = NULL;

// Cache for discovered BLE devices to prevent duplicate processing
static discovered_ble_device_t discovered_ble_devices_cache[CONFIG_BLE_MAX_DISCOVERED_DEVICES_CACHE];
static int discovered_ble_devices_count = 0;
static int discovered_ble_devices_fifo_pointer = 0;

// Scanning state
static bool is_scanning = false;

/**
 * @brief Log detailed information about a discovered BLE device
 *
 * Prints device information including address, signal strength, name,
 * services, and manufacturer data. Separated from processing logic
 * for cleaner code organization and easier debugging control.
 *
 * @param device Pointer to discovered BLE device information
 */
static void log_ble_device_info(const discovered_ble_device_t *device)
{
    ESP_LOGI(TAG, "-----------------------------------\nBLE Device Info:");
    ESP_LOGI(TAG, "  Address: %02x:%02x:%02x:%02x:%02x:%02x",
             device->bda[0], device->bda[1], device->bda[2],
             device->bda[3], device->bda[4], device->bda[5]);

    // Log address type
    const char *addr_type_str;
    switch (device->addr_type)
    {
    case BLE_ADDR_TYPE_PUBLIC:
        addr_type_str = "Public";
        break;
    case BLE_ADDR_TYPE_RANDOM:
        addr_type_str = "Random";
        break;
    default:
        addr_type_str = "Unknown";
        break;
    }
    ESP_LOGI(TAG, "  Address type: %s", addr_type_str);

    // Log signal strength
    ESP_LOGI(TAG, "  RSSI: %d dBm", device->rssi);

    // Log parsed device information
    if (device->has_name)
    {
        ESP_LOGI(TAG, "  Device name: %s", device->name);
    }

    if (device->has_services)
    {
        ESP_LOGI(TAG, "  Services found: %d", device->service_count);
        for (int i = 0; i < device->service_count; i++)
        {
            ESP_LOGI(TAG, "    Service UUID: 0x%04X", device->service_uuids[i]);
        }
    }

    if (device->manufacturer_data_len > 0)
    {
        ESP_LOGI(TAG, "  Manufacturer data: %d bytes", device->manufacturer_data_len);
    }
}

/**
 * @brief Check if BLE device has already been discovered and processed
 *
 * Searches the discovered BLE devices cache to see if we've already processed
 * this device. Prevents duplicate processing during discovery phase.
 * BLE devices can send multiple advertisement packets, so this is essential.
 *
 * @param bda Bluetooth device address to check
 * @param addr_type Address type (public, random, etc.)
 * @return true if device already discovered, false if new device
 */
static bool is_ble_device_already_discovered(esp_bd_addr_t bda, esp_ble_addr_type_t addr_type)
{
    for (int i = 0; i < discovered_ble_devices_count; i++)
    {
        // memcmp == 0 when two blocks of memory are equivalent
        if (memcmp(discovered_ble_devices_cache[i].bda, bda, ESP_BD_ADDR_LEN) == 0 &&
            discovered_ble_devices_cache[i].addr_type == addr_type)
        {
            return true;
        }
    }
    return false;
}

/**
 * @brief Add BLE device to discovered devices cache
 *
 * Adds a new BLE device to the cache using FIFO replacement when full.
 * Cache tracks both address and address type since BLE devices can have
 * multiple address types.
 *
 * @param device Complete discovered BLE device information to add to cache
 */
static void add_ble_device_to_cache(discovered_ble_device_t *device)
{
    if (discovered_ble_devices_count < CONFIG_BLE_MAX_DISCOVERED_DEVICES_CACHE)
    {
        // Cache not full yet, add to next available slot
        memcpy(&discovered_ble_devices_cache[discovered_ble_devices_count], device, sizeof(discovered_ble_device_t));
        discovered_ble_devices_count++;
    }
    else
    {
        // Cache is full, overwrite oldest entry using FIFO pointer
        memcpy(&discovered_ble_devices_cache[discovered_ble_devices_fifo_pointer], device, sizeof(discovered_ble_device_t));

        ESP_LOGD(TAG, "BLE cache full, overwrote device at index %d",
                 discovered_ble_devices_fifo_pointer);

        // Advance FIFO pointer with wrap-around
        discovered_ble_devices_fifo_pointer = (discovered_ble_devices_fifo_pointer + 1) % CONFIG_BLE_MAX_DISCOVERED_DEVICES_CACHE;
    }
}

/**
 * @brief Parse BLE advertisement data
 *
 * Parses structured advertisement data to extract device information.
 * BLE advertisement data is formatted as length-type-value structures.
 * Extracts device name, service UUIDs, manufacturer data, and other info.
 *
 * @param adv_data Pointer to advertisement data
 * @param adv_data_len Length of advertisement data in bytes
 * @param device_info Pointer to device info structure to populate
 */
static void parse_advertisement_data(uint8_t *adv_data, uint16_t adv_data_len, discovered_ble_device_t *device_info)
{
    uint8_t *p = adv_data;
    uint8_t *end = adv_data + adv_data_len;

    // Initialize device info
    device_info->has_name       = false;
    device_info->has_services   = false;
    device_info->name[0]        = '\0';
    device_info->service_count  = 0;


    /*
     * Iterate through the entire adv_data buffer
     * For information on the structure of the adv_data buffer refer to the "ADVERTISING AND SCAN RESPONSE DATA
     * FORMAT" section of the Bluetooth Core Specification
     */ 
    while(p < end){
        uint8_t length = *p++; // Length stored in index 0

        if(length == 0 || p + length > end){
            // If reaches here the data is invalid or incomplete
            break;
        }

        uint8_t type = *p++; // Type stored in index 1
        uint8_t *data = p;
        uint8_t data_len = length - 1;

        switch(type){
            case ESP_BLE_AD_TYPE_NAME_SHORT:
                // Just a shortened name, handle it the same as a complete name
            case ESP_BLE_AD_TYPE_NAME_CMPL:
                {
                    // Check name is not bigger than the maximum we allow for. If so just cut it off at the max
                    int copy_len = (data_len < (CONFIG_BLE_MAX_DEVICE_NAME_LEN - 1) ? data_len : (CONFIG_BLE_MAX_DEVICE_NAME_LEN - 1));
                    memcpy(device_info->name, data, copy_len);
                    device_info->name[copy_len] = '\0';
                    device_info->has_name = true;
                    ESP_LOGD(TAG, "Device name: %s", device_info->name);
                }
                break;
        
            case ESP_BLE_AD_TYPE_16SRV_CMPL:
                // Handle incomplete and complete 16-bit service class UUIDs the same
            case ESP_BLE_AD_TYPE_16SRV_PART:
                    device_info->has_services = true;
                    device_info->service_count = data_len / 2; // 2 bytes per UUID
                    
                    // Limit service count to maximum
                if (device_info->service_count > CONFIG_BLE_MAX_SERVICES_PER_DEVICE) {
                    device_info->service_count = CONFIG_BLE_MAX_SERVICES_PER_DEVICE;
                }

                // Copy service UUIDs
                for (int i = 0; i < device_info->service_count; i++) {
                    // data[i*2]        = First byte of UUID
                    // data[i*2 + 1]    = Second byte of UUID
                    device_info->service_uuids[i] = (data[i*2 + 1] << 8) | data[i*2];
                }
            
                ESP_LOGD(TAG, "Found %d 16-bit service UUIDs", device_info->service_count);
                break;
            
            case ESP_BLE_AD_MANUFACTURER_SPECIFIC_TYPE:
                // Check for manufacterer specific data
                {
                int copy_len = (data_len < CONFIG_BLE_MAX_MANUFACTURER_DATA_LEN) ? 
                                data_len : CONFIG_BLE_MAX_MANUFACTURER_DATA_LEN;
                memcpy(device_info->manufacturer_data, data, copy_len);
                device_info->manufacturer_data_len = copy_len;
                ESP_LOGD(TAG, "  Manufacturer data: %d bytes", data_len);
                }
                break;
            
            case ESP_BLE_AD_TYPE_TX_PWR:
                // Get the power level
                if(data_len >= 1){
                    device_info->tx_power = (int8_t)data[0];
                    ESP_LOGD(TAG, "TX Power: %d dBm", device_info->tx_power);
                }
                break;

            default:
                ESP_LOGD(TAG, "Unhandled AD type: 0x%02x, length %d", type, data_len);
        }
        p += data_len;
    }
}

/**
 * @brief Background task to process discovered BLE devices
 *
 * A FreeRTOS task that processes discovered BLE devices from the discovery queue.
 * Performs detailed analysis of advertisement data including device names,
 * service UUIDs, manufacturer data, and signal strength. Follows the same
 * architectural pattern as the Classic Bluetooth implementation.
 *
 * The task handles heavy processing outside of BLE stack callbacks to maintain
 * stack responsiveness during device discovery.
 *
 * @param pvParameters FreeRTOS task parameter (unused)
 */
static void ble_device_processing_task(void *pvParameters)
{
    discovered_ble_device_t device;

    while (1)
    {
        if (xQueueReceive(discovered_ble_device_queue, &device, portMAX_DELAY))
        {
            // Ensure the device hasn't already been processed
            if (is_ble_device_already_discovered(device.bda, device.addr_type))
            {
                // Print devug message and move onto the next device
                ESP_LOGD(TAG, "BLE device %02x:%02x:%02x:%02x:%02x:%02x already processed, skipping",
                         device.bda[0], device.bda[1], device.bda[2],
                         device.bda[3], device.bda[4], device.bda[5]);
                continue;
            }

            log_ble_device_info(&device);
            add_ble_device_to_cache(&device);
        }
    }
}

/**
 * @brief Handle individual BLE scan results (discovered devices)
 *
 * Processes scan result events and extracts device information.
 * Creates device info structure and queues it for background processing.
 * Handles both advertisement data and scan response data.
 *
 * @param scan_result Pointer to scan result data from BLE stack
 */
static void handle_ble_scan_result(esp_ble_gap_cb_param_t *param){
    switch(param->scan_rst.search_evt){

        // This is the event that fire every time an advertising device is found
        case ESP_GAP_SEARCH_INQ_RES_EVT:
        // New device discovered
        {
            // Device info structure
            discovered_ble_device_t device_info;

            // Copy device info
            mempcpy(device_info.bda, param->scan_rst.bda, ESP_BD_ADDR_LEN);
            device_info.addr_type = param->scan_rst.ble_addr_type;
            device_info.rssi = param->scan_rst.rssi;

            // Parse advertisement data if present
            if(param->scan_rst.adv_data_len > 0){
                parse_advertisement_data(param->scan_rst.ble_adv, param->scan_rst.adv_data_len, &device_info);
            }

            // Parse scan response data if present (active scanning)
            // According to ESP-IDF docs: scan response data follows advertising data in ble_adv array
            if (param->scan_rst.scan_rsp_len > 0) {
                ESP_LOGD(TAG, "Scan response data available (%d bytes)", param->scan_rst.scan_rsp_len);
                uint8_t *scan_rsp_data = param->scan_rst.ble_adv + param->scan_rst.adv_data_len;
                parse_advertisement_data(scan_rsp_data, param->scan_rst.scan_rsp_len, &device_info);
            }
            
            // Queue device for background processing
            if(discovered_ble_device_queue != NULL){
                if(xQueueSend(discovered_ble_device_queue, &device_info, 0) != pdTRUE){
                    ESP_LOGW(TAG, "BLE device queue fill, dropping discovered device");
                }
            }
        }
        break;

        case ESP_GAP_SEARCH_INQ_CMPL_EVT:
            // Scan duration completed naturally
                    ESP_LOGI(TAG, "BLE device discovery completed (scan duration expired)");
        is_scanning = false;
        break;
        
    default:
        ESP_LOGI(TAG, "Unhandled BLE scan result event: %d", param->scan_rst.search_evt);
        break;
    }
}

/**
 * @brief BLE GAP event callback handler
 *
 * Handles all BLE GAP events including scan parameter setting, scan start/stop,
 * and device discovery results. Maintains minimal processing in callback to
 * preserve stack responsiveness.
 *
 * @param event BLE GAP event type
 * @param param Event-specific parameter data
 */
static void ble_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param){
    switch(event){
        case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
            // The scan parameters have been set successfully
            ESP_LOGI(TAG, "BLE scan parameters set complete, starting scan");

            // Start scanning
            esp_err_t ret = esp_ble_gap_start_scanning(30); // 30 seconds scan. TODO: Make this configurable
            if(ret != ESP_OK){
                ESP_LOGE(TAG, "BLE start scanning failed: %s", esp_err_to_name(ret));
            }
            break;
        
        case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
            // The attempt to start the scanning has completed

            // Scanning started fine
            if(param->scan_start_cmpl.status == ESP_BT_STATUS_SUCCESS){
                ESP_LOGI(TAG, "BLE scanning started successfully");
                is_scanning = true;
            }else{
                ESP_LOGE(TAG, "BLE scanning start failed, status: %d", param->scan_start_cmpl.status);
            }
            break;
        
        case ESP_GAP_BLE_SCAN_RESULT_EVT:
            // Device discovered. Called the scan result function
            handle_ble_scan_result(param);
            break;
        
        case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
            // Scanning is finished as esp_ble_gap_stop_scanning() has been called
            if(param->scan_stop_cmpl.status == ESP_BT_STATUS_SUCCESS){
                ESP_LOGI(TAG, "BLE scanning stopped successfully");
                is_scanning = false;
            }else{
                ESP_LOGE(TAG, "BLE scanning stop failed, status: %d", param->scan_stop_cmpl.status);
            }
            break;
        default:
            ESP_LOGI(TAG, "BLE GAP event: %d", event);
            break;
    }
}

/**
 * @brief Stop BLE device discovery
 *
 * Stops ongoing BLE scanning operation if active.
 *
 * @return esp_err_t ESP_OK on success, error code on failure
 */
esp_err_t ble_stop_device_discovery(void)
{
    if (!is_scanning) {
        ESP_LOGW(TAG, "BLE scanning is not active");
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret = esp_ble_gap_stop_scanning();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BLE stop scanning failed: %s", esp_err_to_name(ret));
        return ret;
    }

    return ESP_OK;
}

/**
 * @brief Start BLE device discovery (scanning)
 *
 * Configures scan parameters and initiates BLE device discovery.
 * Uses active scanning to get maximum device information including
 * scan response data. Scan parameters are optimized for discovery
 * rather than power consumption.
 *
 * @param scan_duration Duration to scan in seconds (0 = scan indefinitely)
 * @return esp_err_t ESP_OK on success, error code on failure
 */

esp_err_t ble_start_device_discovery(uint32_t scan_duration){
    esp_err_t ret;

    if(is_scanning){
        ESP_LOGW(TAG, "BLE scanning already in progress");
        return ESP_ERR_INVALID_STATE;
    }

    // Configure scan parameters for device discovery
    static esp_ble_scan_params_t ble_scan_params ={
        .scan_type          = BLE_SCAN_TYPE_ACTIVE,         // Active scanning gets scan response data
        .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,         // Use public address
        .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,    // Accept all advertisements
        .scan_interval      = 0x50,                         // 50ms scan interval (0x50 * 0.625ms = 50ms)
        .scan_window        = 0x30,                         // 30ms scan window (0x30 * 0.625ms = 30ms)
        .scan_duplicate     = BLE_SCAN_DUPLICATE_DISABLE    // Report all packets (including duplicates)
    };

    // Register GAP (Gerneric Access Profile) callback for BLE events
    ret = esp_ble_gap_register_callback(ble_gap_cb);
    if(ret != ESP_OK){
        ESP_LOGE(TAG, "BLE GAP callback register failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Set scan parameters - this will trigger ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT
    ret = esp_ble_gap_set_scan_params(&ble_scan_params);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BLE scan params set failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "BLE scan parameters configured, scan will start automatically");
    return ESP_OK;
}

/**
 * @brief Initialize BLE device discovery functionality
 *
 * Sets up the BLE discovery infrastructure including device queue,
 * background processing task, and initial configuration. Follows the
 * same initialization pattern as Classic Bluetooth implementation.
 *
 * @return esp_err_t ESP_OK on success, error code on failure
 */

esp_err_t ble_discovery_init(void)
{
    ESP_LOGI(TAG, "Initializing BLE Device Discovery");

    // Create device discovery queue
    discovered_ble_device_queue = xQueueCreate(CONFIG_BLE_DISCOVERED_DEVICE_QUEUE_SIZE,
                                               sizeof(discovered_ble_device_t));

    if (discovered_ble_device_queue == NULL)
    {
        ESP_LOGE(TAG, "Failed to create BLE device queue");
        return ESP_ERR_NO_MEM;
    }

    // Configure task core affinity. Defaults to the CORE_APP cpu
    BaseType_t core_id = CONFIG_BLE_DEVICE_PROCESSING_CORE_ID;
    if (core_id == -1)
    {
        core_id = tskNO_AFFINITY;
    }

    // Create device processing FreeRTOS task
    BaseType_t task_result = xTaskCreatePinnedToCore(
        ble_device_processing_task,                   // Task function
        "ble_device_proc",                            // Task name
        CONFIG_BLE_DEVICE_PROCESSING_TASK_STACK_SIZE, // Stack size
        NULL,                                         // Parameters
        CONFIG_BLE_DEVICE_PROCESSING_TASK_PRIORITY,   // Priority
        NULL,                                         // Task handle
        core_id                                       // Core affinity
    );

    if (task_result != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create BLE device processing task");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "BLE device discovery initialized successfully");
    return ESP_OK;
}

#endif // CONFIG_BTDM_CONTROLLER_MODE_BLE_ONLY || CONFIG_BTDM_CONTROLLER_MODE_BTDM
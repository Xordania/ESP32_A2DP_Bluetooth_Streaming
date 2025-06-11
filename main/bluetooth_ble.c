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
    const char* addr_type_str;
    switch (device->addr_type) {
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
    if (device->has_name) {
        ESP_LOGI(TAG, "  Device name: %s", device->name);
    }
    
    if (device->has_services) {
        ESP_LOGI(TAG, "  Services found: %d", device->service_count);
        for (int i = 0; i < device->service_count; i++) {
            ESP_LOGI(TAG, "    Service UUID: 0x%04X", device->service_uuids[i]);
        }
    }
    
    if (device->manufacturer_data_len > 0) {
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
                 discovered_ble_devices_fifo_pointer);;

        // Advance FIFO pointer with wrap-around
        discovered_ble_devices_fifo_pointer = (discovered_ble_devices_fifo_pointer + 1) % CONFIG_BLE_MAX_DISCOVERED_DEVICES_CACHE;
        

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
 static void ble_device_processing_task(void *pvParameters){
    discovered_ble_device_t device;

    while(1)
    {
        if(xQueueReceive(discovered_ble_device_queue, &device, portMAX_DELAY)){
            // Ensure the device hasn't already been processed
            if(is_ble_device_already_discovered(device.bda, device.addr_type))
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
    esp_err_t ret;

    ESP_LOGI(TAG, "Initializing BLE Device Discovery");

    // Create device discovery queue
    discovered_ble_device_queue = xQueueCreate(CONFIG_BLE_DISCOVERED_DEVICE_QUEUE_SIZE,
                                                sizeof(discovered_ble_device_t));

    if(discovered_ble_device_queue == NULL){
        ESP_LOGE(TAG, "Failed to create BLE device queue");
        return ESP_ERR_NO_MEM;
    }

    // Configure task core affinity. Defaults to the CORE_APP cpu
    BaseType_t core_id = CONFIG_BLE_DEVICE_PROCESSING_CORE_ID;
    if(core_id == -1){
        core_id = tskNO_AFFINITY;
    }

    // Create device processing FreeRTOS task
    BaseType_t task_result = xTaskCreatePinnedToCore(
        ble_device_processing_task,                     // Task function
        "ble_device_proc",                              // Task name
        CONFIG_BLE_DEVICE_PROCESSING_TASK_STACK_SIZE,   // Stack size
        NULL,                                           // Parameters
        CONFIG_BLE_DEVICE_PROCESSING_TASK_PRIORITY,     // Priority
        NULL,                                           // Task handle
        core_id                                         // Core affinity
    );

if(task_result != pdPASS){
    ESP_LOGE(TAG, "Failed to create BLE device processing task");
    return ESP_ERR_NO_MEM;
}

ESP_LOGI(TAG, "BLE device discovery initialized successfully");
return ESP_OK;

}

#endif // CONFIG_BTDM_CONTROLLER_MODE_BLE_ONLY || CONFIG_BTDM_CONTROLLER_MODE_BTDM
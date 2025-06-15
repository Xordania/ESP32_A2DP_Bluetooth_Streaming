#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "bluetooth_classic.h"
#include "bluetooth_common.h"
#include "bluetooth_ble.h"
#include "bluetooth_uuid_definitions.h"
#include "bluetooth_device_analyzer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "esp_err.h"
#include "esp_bt.h"
#include "esp_log.h"

static const char *TAG = "MAIN";

// Target device information
static bool jbl_device_found = false;
static discovered_ble_device_t target_jbl_device;
/**
 * @brief Check if discovered device is likely a JBL FLIP 5
 * 
 * Uses multiple criteria to identify JBL devices:
 * - Device name contains "JBL" or "FLIP"
 * - Manufacturer data indicates Harman International (JBL's parent company)
 * - Signal strength is reasonable for connection
 */
static bool is_jbl_flip5_device(const discovered_ble_device_t *device)
{
    // Check device name for JBL indicators
    if (device->has_name) {
        if (strstr(device->name, "JBL") != NULL || 
            strstr(device->name, "FLIP") != NULL ||
            strstr(device->name, "Flip") != NULL) {
            ESP_LOGI(TAG, "Found device with JBL name: %s", device->name);
            return true;
        }
    }
    
    // Check manufacturer data for Harman International (JBL's parent company)
    if (device->manufacturer_data_len >= 2) {
        uint16_t company_id = (device->manufacturer_data[1] << 8) | device->manufacturer_data[0];
        if (company_id == BT_COMPANY_ID_HARMAN) {
            ESP_LOGI(TAG, "Found Harman International device (JBL parent company)");
            return true;
        }
    }
    
    return false;
}

/**
 * @brief Callback for BLE device analysis completion
 * 
 * This is called when the device analyzer finishes analyzing a potential
 * JBL device to determine its audio streaming capabilities.
 */
static void jbl_analysis_complete_cb(ble_device_analysis_result_t *result)
{
    ESP_LOGI(TAG, "=== JBL Device Analysis Complete ===");
    ESP_LOGI(TAG, "Device: %02x:%02x:%02x:%02x:%02x:%02x",
             result->bda[0], result->bda[1], result->bda[2],
             result->bda[3], result->bda[4], result->bda[5]);
    
    if (result->connection_failed) {
        ESP_LOGE(TAG, "Failed to connect to JBL device for analysis");
        return;
    }
    
    // Log discovered services
    ESP_LOGI(TAG, "Total services discovered: %d", result->service_count);
    ESP_LOGI(TAG, "Audio services found: %d", result->audio_service_count);
    
    // Check for BLE Audio capabilities
    if (ble_device_has_le_audio(result)) {
        ESP_LOGI(TAG, "✓ JBL device supports LE Audio!");
        ESP_LOGI(TAG, "Capabilities: %s", ble_audio_capabilities_to_string(result->audio_capabilities));
        
        // Log specific audio services found
        for (int i = 0; i < result->audio_service_count; i++) {
            uint16_t uuid = result->audio_services[i];
            ESP_LOGI(TAG, "  Audio Service: 0x%04X (%s)", 
                     uuid, get_ble_audio_service_name(uuid));
        }
        
        // Get primary audio service for streaming
        uint16_t primary_service = ble_get_primary_audio_service(result);
        if (primary_service != 0) {
            ESP_LOGI(TAG, "Primary audio service: 0x%04X (%s)", 
                     primary_service, get_ble_audio_service_name(primary_service));
        }
        
        // Log connection parameters (important for audio streaming)
        ESP_LOGI(TAG, "Connection parameters:");
        ESP_LOGI(TAG, "  Interval: %d ms", result->conn_interval);
        ESP_LOGI(TAG, "  Latency: %d", result->conn_latency);
        ESP_LOGI(TAG, "  Timeout: %d ms", result->supervision_timeout);
        
        // TODO: Proceed with audio streaming setup
        ESP_LOGI(TAG, "Ready for BLE Audio streaming to JBL FLIP 5!");
        
    } else if (ble_device_supports_audio(result)) {
        ESP_LOGI(TAG, "✓ JBL device supports some audio (not LE Audio)");
        ESP_LOGI(TAG, "Capabilities: %s", ble_audio_capabilities_to_string(result->audio_capabilities));
        
        // Might be using proprietary audio protocol
        ESP_LOGW(TAG, "Device may use proprietary audio streaming");
        
    } else {
        ESP_LOGW(TAG, "✗ JBL device doesn't appear to support audio streaming");
        ESP_LOGW(TAG, "This might not be a FLIP 5 or analysis incomplete");
    }
}

/**
 * @brief Process discovered BLE devices looking for JBL FLIP 5
 * 
 * This function would be called from your main BLE discovery loop
 * when new devices are found.
 */
void process_discovered_device_for_jbl(const discovered_ble_device_t *device)
{
    // Skip if we already found our target device
    if (jbl_device_found) {
        return;
    }
    
    // Check signal strength - need reasonable signal for audio streaming
    if (device->rssi < -80) {
        ESP_LOGD(TAG, "Device signal too weak (RSSI: %d dBm), skipping", device->rssi);
        return;
    }
    
    // Check if this looks like a JBL device
    if (is_jbl_flip5_device(device)) {
        ESP_LOGI(TAG, "🎵 Potential JBL FLIP 5 found!");
        ESP_LOGI(TAG, "Address: %02x:%02x:%02x:%02x:%02x:%02x",
                 device->bda[0], device->bda[1], device->bda[2],
                 device->bda[3], device->bda[4], device->bda[5]);
        ESP_LOGI(TAG, "Name: %s", device->has_name ? device->name : "Unknown");
        ESP_LOGI(TAG, "RSSI: %d dBm", device->rssi);
        
        // Save device info
        memcpy(&target_jbl_device, device, sizeof(discovered_ble_device_t));
        jbl_device_found = true;
        
        // Stop discovery to save power and reduce interference
        ESP_LOGI(TAG, "Stopping BLE discovery to analyze JBL device...");
        ble_stop_device_discovery();
        
        // Start detailed analysis of the JBL device
        ESP_LOGI(TAG, "Starting detailed analysis of JBL device...");
        esp_err_t ret = ble_analyze_device(device->bda, device->addr_type, jbl_analysis_complete_cb);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to start JBL device analysis: %s", esp_err_to_name(ret));
            jbl_device_found = false; // Reset so we can try again
        }
    }
}

/**
 * @brief Initialize JBL FLIP 5 discovery system
 * 
 * Call this after basic BLE stack initialization to set up
 * the discovery and analysis system for finding JBL speakers.
 */
esp_err_t init_jbl_flip5_discovery(void)
{
    esp_err_t ret;
    
    ESP_LOGI(TAG, "Initializing JBL FLIP 5 discovery system...");
    
    // Initialize BLE device analyzer
    ret = ble_device_analyzer_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize BLE device analyzer: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "JBL FLIP 5 discovery system ready!");
    ESP_LOGI(TAG, "Start BLE device discovery to begin searching...");
    
    return ESP_OK;
}

/**
 * @brief Enhanced device filtering for BLE discovery
 * 
 * This shows how you could modify your existing BLE discovery
 * to be more targeted for audio devices.
 */
static bool should_analyze_device(const discovered_ble_device_t *device)
{
    // Always analyze if it looks like JBL
    if (is_jbl_flip5_device(device)) {
        return true;
    }
    
    // Check for devices advertising audio services
    if (device->has_services) {
        for (int i = 0; i < device->service_count; i++) {
            if (is_ble_audio_service(device->service_uuids[i])) {
                ESP_LOGI(TAG, "Found device with BLE Audio service: 0x%04X", 
                         device->service_uuids[i]);
                return true;
            }
        }
    }
    
    // Check manufacturer data for known audio companies
    if (device->manufacturer_data_len >= 2) {
        uint16_t company_id = (device->manufacturer_data[1] << 8) | device->manufacturer_data[0];
        
        switch (company_id) {
            case BT_COMPANY_ID_HARMAN:    // JBL, AKG
            case BT_COMPANY_ID_SONY:      // Sony audio products
            case BT_COMPANY_ID_BOSE:      // Bose speakers
            case BT_COMPANY_ID_BEATS:     // Beats headphones
                ESP_LOGI(TAG, "Found audio company device: %s", 
                         get_company_name(company_id));
                return true;
        }
    }
    
    return false;
}

/**
 * @brief Integration point with existing BLE discovery
 * 
 * Add this call to your existing ble_device_processing_task()
 * in bluetooth_ble.c after logging device info.
 */
void integrate_with_existing_discovery(const discovered_ble_device_t *device)
{
    // Your existing device logging code here...
    
    // Add JBL-specific processing
    if (should_analyze_device(device)) {
        process_discovered_device_for_jbl(device);
    }
}


static void ble_config(void)
{
    esp_err_t ret;

    // Release unused Classic BT memory back to the heap
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    bt_nvs_init();

    // Initialize BLE stack first
    ret = bt_controller_stack_init(ESP_BT_MODE_BLE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BLE stack initialization failed: %s", esp_err_to_name(ret));
        return;
    }
    
    // Initialize BLE device discovery
    ret = ble_discovery_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BLE discovery initialization failed: %s", esp_err_to_name(ret));
        return;
    }
    
    // Search for the JBL FLIP 5
    ret = init_jbl_flip5_discovery();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "JBL discovery init failed: %s", esp_err_to_name(ret));
        return;
    }

    // Start BLE device discovery
    ret = ble_start_device_discovery(30); // Scan for 30 seconds
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BLE device discovery start failed: %s", esp_err_to_name(ret));
        return;
    }
}

void app_main(void)
{

    esp_err_t ret;

    // If bluetooth class is initialize it
    #if CONFIG_BTDM_CONTROLLER_MODE_BR_EDR_ONLY
        // Release unused memory back to the heap if using bluetooth classic only
        // Has to be done before initialization
        ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));
        
        
        bt_a2dp_source_init();

    // Both classic and BLE active
    #elif CONFIG_BTDM_CONTROLLER_MODE_BTDM
        bt_a2dp_source_init();

        //TODO: Activate BLE

    // Only BLE active
    #elif CONFIG_BTDM_CONTROLLER_MODE_BLE_ONLY
        
        ble_config();

    // Neither versions of bluetooth are active. Clear the memory
    #else
        ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_IDLE));
    #endif

    // Initialize A2DP source
    while (1)
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
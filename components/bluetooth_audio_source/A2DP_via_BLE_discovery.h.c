#include <string.h>
#include "a2dp_via_ble_discovery.h"
#include "esp_a2dp_api.h"
#include "esp_log.h"
#include "bluetooth_ble.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

static const char *TAG = "DUAL_MODE_DETECT";

/**
 * @brief Step 1: Check BLE advertisement for BR/EDR capability
 * 
 * Check the ble advertisement data for BR/ER support
 */
bool parse_ble_adv_for_dual_mode(discovered_ble_device_t *device_info)
{   
    // Device supports dual mode if the 2nd bit in not set
    bool supports_classic_mode = !(device_info->supported_modes & 0x04);

    ESP_LOGI(TAG, "%s BR/ER support: %s. Supported_modes: 0x%0x", device_info->name, supports_classic_mode ? "TRUE" : "FALSE", supports_classic_mode);
    return supports_classic_mode;  // Conservative: assume no BR/EDR if not advertised
}


bool probe_device_a2dp_support(esp_bd_addr_t bda, uint32_t timeout_ms){

    ESP_LOGI(TAG, "Attempting A2DP connection");


     // Initiate A2DP connection
    esp_err_t ret = esp_a2d_source_connect(bda);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "A2DP connection initiation failed: %s", esp_err_to_name(ret));
        return false;
    }

    return true;
}
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "bluetooth_classic.h"
#include "bluetooth_common.h"
#include "bluetooth_ble.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "esp_err.h"
#include "esp_bt.h"
#include "esp_log.h"

static const char *TAG = "MAIN";

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
        
        // Start BLE device discovery
        ret = ble_start_device_discovery(30); // Scan for 30 seconds
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "BLE device discovery start failed: %s", esp_err_to_name(ret));
            return;
        }
        


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
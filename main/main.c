#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "bluetooth_classic.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "esp_err.h"
#include "esp_bt.h"

void app_main(void)
{
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
        // Release unused memory back to the heap if using bluetooth low energy only
        ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

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
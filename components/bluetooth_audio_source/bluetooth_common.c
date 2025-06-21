#include "bluetooth_common.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_avrc_api.h"
#include "esp_bt_defs.h"
#include "esp_log.h"


static const char* TAG = "BT_COMMON";

/**
 * @brief Initialize NVS (Non-Volatile Storage) for Bluetooth
 *
 * According to ESP-IDF documentation: "NVS is designed to store key-value 
 * pairs in flash. This is used by the Bluetooth stack to store pairing 
 * information and other persistent data."
 *
 * @return esp_err_t ESP_OK on success, error code on failure
 */
esp_err_t bt_nvs_init(void)
{
    esp_err_t ret = nvs_flash_init();

    // Handle case where NVS partition is full or version mismatch
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS partition issues, erasing and reinitializing");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "NVS initialization failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "NVS initialized successfully");
    return ESP_OK;
}

/**
 * @brief Initialize Bluetooth controller and Bluedroid stack
 *
 * From ESP-IDF documentation: "The Bluetooth controller and Bluedroid stack 
 * must be initialized before any Bluetooth functionality can be used."
 *
 * Also: "The controller can be configured to operate in different modes: 
 * Classic Bluetooth only, BLE only, or dual mode."
 *
 * @param mode Bluetooth mode (ESP_BT_MODE_CLASSIC_BT, ESP_BT_MODE_BLE, or ESP_BT_MODE_BTDM)
 * @return esp_err_t ESP_OK on success, error code on failure
 */
esp_err_t bt_controller_stack_init(esp_bt_mode_t mode)
{
    esp_err_t ret;

    // Validate mode parameter
    if (mode != ESP_BT_MODE_CLASSIC_BT && mode != ESP_BT_MODE_BLE && mode != ESP_BT_MODE_BTDM) {
        ESP_LOGE(TAG, "Invalid Bluetooth mode: %d", mode);
        return ESP_ERR_INVALID_ARG;
    }

    // Configure Bluetooth controller
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    bt_cfg.mode = mode;

    // Initialize Bluetooth controller
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Bluetooth controller init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Enable Bluetooth controller with specified mode
    ret = esp_bt_controller_enable(mode);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Bluetooth controller enable failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Initialize Bluedroid protocol stack
    ret = esp_bluedroid_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Bluedroid stack init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Enable Bluedroid protocol stack
    ret = esp_bluedroid_enable();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Bluedroid stack enable failed: %s", esp_err_to_name(ret));
        return ret;
    }

    const char* mode_str = (mode == ESP_BT_MODE_CLASSIC_BT) ? "Classic BT" :
                          (mode == ESP_BT_MODE_BLE) ? "BLE" : "Dual Mode";
    ESP_LOGI(TAG, "Bluetooth controller and stack initialized successfully (%s)", mode_str);
    
    return ESP_OK;
}

/**
 * @brief Compare two bdas
=
 * @param a the first bda
 * @param b the second vbda
 * @return true if they're the same. False if they're not 
 */
bool equal_bda(esp_bd_addr_t a, esp_bd_addr_t b){
    for(uint8_t i = 0; i < 6; i++){
        if(a[i] != b[i]){
            return false;
        }
    }

    return true;
}
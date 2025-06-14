#ifndef BLUETOOTH_BLE_H
#define BLUETOOTH_BLE_H

#include <stdint.h>
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_bt_defs.h"
#include "sdkconfig.h"

// Structure to hold discovered BLE device information
typedef struct {
    esp_bd_addr_t bda;                                                  // Bluetooth device address
    esp_ble_addr_type_t addr_type;                                      // Address type (public/random)
    int8_t rssi;                                                        // Signal strength

    // Parsed advertisement data        
    bool has_name;                                                      // Device name present
    char name[CONFIG_BLE_MAX_DEVICE_NAME_LEN];                          // Device name

    bool has_services;                                                  // Service UUIDs present
    uint8_t service_count;                                              // Number of service UUIDs
    uint16_t service_uuids[CONFIG_BLE_MAX_SERVICES_PER_DEVICE];         // 16-bit service UUIDs
    
    uint8_t manufacturer_data[CONFIG_BLE_MAX_MANUFACTURER_DATA_LEN];    // Manufacturer specific data
    uint8_t manufacturer_data_len;                                      // Length of manufacturer data
    
    int8_t tx_power;                                                    // Advertised TX power level

    // Raw advertisement data (for advanced processing)     
    uint8_t adv_data_len;                                               // Advertisement data length
    uint8_t scan_rsp_len;                                               // Scan response data length
} discovered_ble_device_t;

// Function declarations
#if CONFIG_BTDM_CONTROLLER_MODE_BLE_ONLY || CONFIG_BTDM_CONTROLLER_MODE_BTDM
    esp_err_t ble_discovery_init(void);
    esp_err_t ble_start_device_discovery(uint32_t scan_duration);
    esp_err_t ble_stop_device_discovery(void);
#endif // CONFIG_BTDM_CONTROLLER_MODE_BLE_ONLY || CONFIG_BTDM_CONTROLLER_MODE_BTDM

#endif // BLUETOOTH_BLE_H
#ifndef BLUETOOTH_BLE_H
#define BLUETOOTH_BLE_H

#include <stdint.h>
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_bt_defs.h"
#include "sdkconfig.h"
#include "esp_event.h" 

// Structure to hold discovered BLE device information
typedef struct {
    esp_bd_addr_t bda;                                                  // Bluetooth device address
    esp_ble_addr_type_t addr_type;                                      // Address type (public/random)
    esp_bt_dev_type_t supported_modes;                                  // Bluetooth device type
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

/**
 * @brief BLE Discovery Event IDs
 * 
 * These are the specific events that the BLE discovery system can post.
 */
typedef enum {
    BLE_DISCOVERY_STARTED,              // Discovery scanning started
    BLE_DISCOVERY_STOPPED,              // Discovery scanning stopped
    BLE_DISCOVERY_DEVICE_FOUND,         // New BLE device discovered
    BLE_DISCOVERY_SCAN_TIMEOUT,         // Scan duration timeout reached
    BLE_DISCOVERY_ERROR,                // Error occurred during discovery
    BLE_SCAN_RESPONSE,                  // Reponse to a scan
} ble_discovery_event_id_t;

/**
 * @brief Event data for BLE_DISCOVERY_DEVICE_FOUND
 * 
 * Contains all information about a newly discovered device.
 */
typedef struct {
    discovered_ble_device_t device;     // Complete device information
    bool is_duplicate;                  // Was this device seen before?
    uint32_t discovery_count;           // Total devices found in this session
    uint32_t session_id;                // Unique ID for this discovery session
} ble_device_found_event_data_t;

/**
 * @brief BLE Discovery Event Base
 * 
 * This creates a unique identifier for our BLE discovery events.
 */
ESP_EVENT_DECLARE_BASE(BLE_DISCOVERY_EVENTS);

/**
 * @brief Event data for BLE_DISCOVERY_STARTED
 */
typedef struct {
    uint32_t scan_duration_sec;         // Configured scan duration
    uint32_t session_id;                // Unique session identifier
    esp_ble_scan_params_t scan_params;  // Scan parameters being used
} ble_discovery_started_event_data_t;

/**
 * @brief Event data for BLE_DISCOVERY_STOPPED
 */
typedef struct {
    uint32_t total_devices_found;       // Total devices discovered
    uint32_t scan_duration_ms;          // Actual scan duration
    uint32_t session_id;                // Session that ended
    esp_err_t stop_reason;              // Why discovery stopped (ESP_OK = normal, ESP_ERR_TIMEOUT = timeout)
} ble_discovery_stopped_event_data_t;

/**
 * @brief Event data for BLE_DISCOVERY_ERROR
 */
typedef struct {
    esp_err_t error_code;               // Error code
    const char* error_description;      // Human-readable description
    ble_discovery_event_id_t failed_operation;  // What operation failed
} ble_discovery_error_event_data_t;


// Function declarations
#if CONFIG_BTDM_CONTROLLER_MODE_BLE_ONLY || CONFIG_BTDM_CONTROLLER_MODE_BTDM
    esp_err_t ble_discovery_init(void);
    esp_err_t ble_start_device_discovery(uint32_t scan_duration);
    esp_err_t ble_stop_device_discovery(void);
    void ble_get_discovery_stats(uint32_t *devices_found, uint32_t *scan_sessions);
    void ble_get_discovery_stats(uint32_t *devices_found, uint32_t *scan_sessions);
    bool ble_is_discovery_active(void);
#endif // CONFIG_BTDM_CONTROLLER_MODE_BLE_ONLY || CONFIG_BTDM_CONTROLLER_MODE_BTDM

#endif // BLUETOOTH_BLE_H
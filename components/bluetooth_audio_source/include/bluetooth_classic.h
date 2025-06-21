#include "sdkconfig.h"

#if CONFIG_BTDM_CONTROLLER_MODE_BR_EDR_ONLY || CONFIG_BTDM_CONTROLLER_MODE_BTDM

#ifndef BLUETOOTH_CLASSIC_H
#define BLUETOOTH_CLASSIC_H

#include "esp_bt.h"
#include "esp_a2dp_api.h"
#include "esp_gap_bt_api.h"
#include "esp_event.h" 

#define BT_DEVICE_NAME "ESP32"

// Structure to hold discovered device info
typedef struct {
    esp_bd_addr_t bda;
    int num_prop;
    esp_bt_gap_dev_prop_t props[CONFIG_A2DP_MAX_DEVICE_PROPERTIES];
} discovered_device_t;


/**
 * @brief BLE Discovery Event IDs
 * 
 * These are the specific events that the BLE discovery system can post.
 */
typedef enum {
    BT_DISCOVERY_STARTED,              // Discovery scanning started
    BT_DISCOVERY_STOPPED,              // Discovery scanning stopped
    BT_DISCOVERY_DEVICE_FOUND,         // New device discovered
    BT_DISCOVERY_SCAN_TIMEOUT,         // Scan duration timeout reached
    BT_DISCOVERY_ERROR,                // Error occurred during discovery
    BT_SCAN_RESPONSE,                  // Reponse to a scan
} bt_discovery_event_id_t;

/**
 * @brief Event data for BT_DISCOVERY_DEVICE_FOUND
 * 
 * Contains all information about a newly discovered device.
 */
typedef struct {
    discovered_device_t device;         // Complete device information
    bool is_duplicate;                  // Was this device seen before?
    uint32_t discovery_count;           // Total devices found in this session
    uint32_t session_id;                // Unique ID for this discovery session
} bt_device_found_event_data_t;


/**
 * @brief Bluetooth Discovery Event Base
 * 
 * This creates a unique identifier for our Bluetooth discovery events.
 */
ESP_EVENT_DECLARE_BASE(BT_DISCOVERY_EVENTS);

// Function declarations
esp_err_t bt_gap_init(QueueHandle_t *discovered_devices);
void add_device_to_bt_cache(discovered_device_t *device);
bool is_bt_device_already_discovered(esp_bd_addr_t bda);
const char* get_bt_device_class_name(uint32_t major_class);
bool is_bt_device_already_discovered(esp_bd_addr_t bda);

#endif // BLUETOOTH_CLASSIC_H
#endif // CONFIG_BTDM_CONTROLLER_MODE_BR_EDR_ONLY || CONFIG_BTDM_CONTROLLER_MODE_BTDM
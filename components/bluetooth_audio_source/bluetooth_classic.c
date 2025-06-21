/**
 * @file main.c
 * @brief ESP32 Bluetooth A2DP Audio Source Implementation
 *
 * This file implements a complete Bluetooth A2DP (Advanced Audio Distribution Profile)
 * source application for ESP32. The application can discover nearby Bluetooth audio
 * devices (speakers, headphones) and establish A2DP connections for audio streaming.
 *
 * Key Features:
 * - Automatic Bluetooth device discovery and connection
 * - A2DP source profile implementation
 * - Codec negotiation support (SBC, AAC, MPEG Audio)
 * - Efficient device processing with background task architecture
 * - Configurable parameters via Kconfig
 * - Professional error handling and logging
 *
 * Architecture:
 * - Bluetooth callbacks handle events with minimal processing
 * - Heavy device analysis is performed in dedicated FreeRTOS task
 * - Device discovery results are queued to maintain stack responsiveness
 * - Memory-safe device information copying prevents data corruption
 *
 * Configuration:
 * All major parameters are configurable via `idf.py menuconfig` under
 * "A2DP Source Configuration" including queue sizes, task priorities,
 * and memory allocation limits.
 *
 * @author Jordan Fearn
 * @date 08/06/2025
 * @version 1.0
 */

#include "sdkconfig.h"


// Don't bother flashing this if not in either bluetooth classic mode or dual bluetooth mode
#if CONFIG_BTDM_CONTROLLER_MODE_BR_EDR_ONLY || CONFIG_BTDM_CONTROLLER_MODE_BTDM


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "bluetooth_classic.h"
#include "bluetooth_common.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_avrc_api.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_defs.h"
#include "esp_event.h" 

static const char *TAG = "A2DP_SOURCE";

static QueueHandle_t discovered_device_queue = NULL;

// Cache for discovered devices to prevent duplicate processing and for retrival later on when selecting devices
static discovered_device_t discovered_devices_cache[CONFIG_A2DP_MAX_DISCOVERED_DEVICES_CACHE];
static int discovered_devices_count = 0;
static int discovered_devices_fifo_pointer = 0;


static struct {
    bool events_enabled;                // Are events initialized?
    uint32_t current_session_id;        // Current discovery session ID
    uint32_t devices_in_session;        // Devices found in current session
    uint32_t total_devices_found;       // Total devices found across all sessions
    uint32_t total_scan_sessions;       // Total number of scan sessions
    TickType_t session_start_time;      // When current session started
    uint32_t configured_scan_duration;  // Configured scan duration for current session
} discovery_stats = {
    .events_enabled = false,
    .current_session_id = 0,
    .devices_in_session = 0,
    .total_devices_found = 0,
    .total_scan_sessions = 0
};

/**
 * @brief Define the event base for BT discovery
 */
ESP_EVENT_DEFINE_BASE(BT_DISCOVERY_EVENTS);

// ============================================================================
// EVENT POSTING FUNCTIONS
// ============================================================================
/**
 * @brief Post device discovered event
 * 
 * Posts BT_DISCOVERY_DEVICE_FOUND event with complete device information.
 */
static void post_device_discovered_event(discovered_device_t *device)
{
    if (!discovery_stats.events_enabled) {
        ESP_LOGD(TAG, "Events not enabled, skipping device discovered event");
        return;
    }

    bool is_duplicate = is_bt_device_already_discovered(device->bda);
    
    // Increment counters
    if (!is_duplicate) {
        discovery_stats.devices_in_session++;
        discovery_stats.total_devices_found++;
    }

    // Create event data
    bt_device_found_event_data_t event_data = {
        .device = *device,
        .is_duplicate = is_duplicate,
        .discovery_count = discovery_stats.devices_in_session,
        .session_id = discovery_stats.current_session_id
    };

    // Post event
    esp_err_t ret = esp_event_post(BT_DISCOVERY_EVENTS,
                                  BT_DISCOVERY_DEVICE_FOUND,
                                  &event_data,
                                  sizeof(event_data),
                                  100 / portTICK_PERIOD_MS);  // 100ms timeout
    
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to post device discovered event: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGD(TAG, "Posted device discovered event for session %lu", discovery_stats.current_session_id);
    }
}

/**
 * @brief Check if device has already been discovered and processed
 *
 * Searches the discovered devices cache to see if we've already processed
 * this device. Prevents duplicate processing of the same device during
 * discovery phase.
 *
 * @param bda Bluetooth device address to check
 * @return true if device already discovered, false if new device
 */
bool is_bt_device_already_discovered(esp_bd_addr_t bda)
{
    for (int i = 0; i < discovered_devices_count; i++)
    {
        if (memcmp(discovered_devices_cache[i].bda, bda, ESP_BD_ADDR_LEN) == 0)
        {
            return true;
        }
    }
    return false;
}

/**
 * @brief Add device to discovered devices cache
 *
 * Adds a new device to the cache of already-discovered devices.
 * The cache is built as a fifo with a rotating pointer to the 'end'
 * of the fifo keeping track of where we are.
 * @param device Complete discovered device information to add to cache
 */
void add_device_to_bt_cache(discovered_device_t *device)
{

    if (discovered_devices_count < CONFIG_A2DP_MAX_DISCOVERED_DEVICES_CACHE)
    {
        // Cache not full yet, add to next available slot
        memcpy(&discovered_devices_cache[discovered_devices_count], device, sizeof(discovered_device_t));
        discovered_devices_count++;
    }
    else
    {
        // Cache is full, overwrite oldest entry using FIFO pointer
        memcpy(&discovered_devices_cache[discovered_devices_fifo_pointer], device, sizeof(discovered_device_t));
        
        ESP_LOGD(TAG, "Cache full, overwrote device at index %d", 
                discovered_devices_fifo_pointer);

        // Advance FIFO pointer with wrap-around
        discovered_devices_fifo_pointer = (discovered_devices_fifo_pointer + 1) % CONFIG_A2DP_MAX_DISCOVERED_DEVICES_CACHE;
        
    }
}

/**
 * @brief Get cached discovered devices for selection
 *
 * Returns pointer to the discovered devices cache and count.
 * Useful for implementing device selection UI.
 *
 * @param count Pointer to store the number of cached devices
 * @return discovered_device_t* Pointer to cache array
 */
static discovered_device_t* get_discovered_devices(int *count)
{
    *count = discovered_devices_count;
    return discovered_devices_cache;
}

/**
 * @brief Get human-readable device class description
 *
 * Converts Bluetooth major device class code to descriptive string.
 * Useful for debugging and understanding what types of devices are discovered.
 *
 * @param major_class Major device class code from Class of Device
 * @return const char* Human-readable description
 */
const char* get_bt_device_class_name(uint32_t major_class)
{
    switch (major_class)
    {
        case 0x00: return "Miscellaneous";
        case 0x01: return "Computer";
        case 0x02: return "Phone";
        case 0x03: return "LAN/Network Access Point";
        case 0x04: return "Audio/Video";
        case 0x05: return "Peripheral";
        case 0x06: return "Imaging";
        case 0x07: return "Wearable";
        case 0x08: return "Toy";
        case 0x09: return "Health";
        case 0x0a: return "Uncategorized";
        case 0x0b: return "Reserved";
        case 0x0c: return "Reserved";
        case 0x0d: return "Computer";
        case 0x0e: return "Reserved";
        case 0x0f: return "Reserved";
        case 0x1F: return "Uncategorized";
        default: return "Unknown";
    }
}

/**
 * @brief GAP (Generic Access Profile) event callback handler
 *
 * Handles fundamental Bluetooth GAP events including device discovery,
 * authentication, and connection management. This callback manages the
 * low-level Bluetooth operations that enable device finding and pairing.
 *
 * Key events handled:
 * - Device discovery results (discovered devices are queued for processing)
 * - Authentication completion (pairing success/failure)
 * - Discovery state changes (start/stop notifications)
 *
 * Device discovery results are processed efficiently by queueing device
 * information for background processing, maintaining Bluetooth stack
 * responsiveness during busy discovery periods.
 *
 * @param event Type of GAP event that occurred
 * @param param Event-specific parameter data containing device information,
 *              authentication results, or discovery state information
 *
 * @note This callback executes in Bluetooth stack context - heavy processing
 *       is deferred to background task via queue
 * @note Discovery results are rate-limited by queue size to prevent memory overflow
 */
static void bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    switch (event)
    {
    case ESP_BT_GAP_AUTH_CMPL_EVT:
        if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS)
        {
            ESP_LOGI(TAG, "Authentication success: %s", param->auth_cmpl.device_name);
        }
        else
        {
            ESP_LOGE(TAG, "Authentication failed, status: %d", param->auth_cmpl.stat);
        }
        break;

    case ESP_BT_GAP_DISC_RES_EVT:
    {

        // Copy from the callback parameter into the custom discovered_device_t struct
        // This is so that when the param pointer is destroyed once the callback is exited
        // there is still a copy of data saved in the custom struct
        discovered_device_t device;
        memcpy(device.bda, param->disc_res.bda, ESP_BD_ADDR_LEN);
        device.num_prop = param->disc_res.num_prop;

        // Copy properties with overflow protection
        // Limit copying to our configured maximum to prevent buffer overflow
        int props_to_copy;
        if (device.num_prop > CONFIG_A2DP_MAX_DEVICE_PROPERTIES)
        {
            props_to_copy = CONFIG_A2DP_MAX_DEVICE_PROPERTIES;
        }
        else
        {
            props_to_copy = device.num_prop;
        }

        for (int i = 0; i < props_to_copy; i++)
        {
            device.props[i] = param->disc_res.prop[i];
        }

        // Send discovery event
        post_device_discovered_event(&device);

        // Queue for processing in separate task
        // Doing this outside of the interrupt will free up the bluetooth stack so if another event triggers
        // the callback will have finsihed quickly enough to handle it
        if (discovered_device_queue != NULL)
        {
            if (xQueueSend(discovered_device_queue, &device, 0) != pdTRUE)
            {
                ESP_LOGW(TAG, "Device queue full, dropping discovered device");
            }
        }
        break;
    }

    case ESP_BT_GAP_DISC_STATE_CHANGED_EVT:
        if (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STOPPED)
        {
            ESP_LOGI(TAG, "Device discovery stopped");
        }
        else if (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STARTED)
        {
            ESP_LOGI(TAG, "Device discovery started");
        }
        break;

    default:
        ESP_LOGI(TAG, "GAP event: %d", event);
        break;
    }
}

/**
 * @brief Initialize GAP (Generic Access Profile) functionality
 *
 * Sets up the Bluetooth Generic Access Profile including callback registration,
 * device name configuration, scan mode setup, and device discovery initiation.
 *
 * GAP is responsible for device discovery, connection establishment, 
 * security procedures, and device management.
 *
 * GAP setup sequence:
 * 1. Register GAP event callback for handling discovery and authentication
 * 2. Set device name for identification by other devices
 * 3. Configure scan mode (connectable and discoverable)
 * 4. Start device discovery to find nearby Bluetooth devices
 *
 * @return esp_err_t ESP_OK on success, error code on failure
 *
 * @note Device discovery runs for 10 seconds by default
 * @note Device will be both connectable and discoverable to other devices
 */
esp_err_t bt_gap_init(QueueHandle_t *discovered_devices)
{
    esp_err_t ret;

    discovered_device_queue = *discovered_devices;

    // Register GAP callback for device discovery and authentication events
    ret = esp_bt_gap_register_callback(bt_gap_cb);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "GAP callback register failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Set device name that will be visible to other Bluetooth devices
    ret = esp_bt_gap_set_device_name(BT_DEVICE_NAME);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Set device name failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Set scan mode to be both connectable and discoverable
    ret = esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Set scan mode failed: %s", esp_err_to_name(ret));
        return ret;
    }

    discovery_stats.events_enabled = true;
    ESP_LOGI(TAG, "Bluetooth discovery event initialized");

    ESP_LOGI(TAG, "GAP initialized, run esp_bt_gap_start_discovery() to begin discovery");
    return ESP_OK;
}


#endif

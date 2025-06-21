/**
 * @file a2dp_via_ble_discovery.h
 * @brief Detection of Classic Bluetooth A2DP capability via BLE discovery
 * 
 * This module analyzes BLE advertisement data and GATT services to determine
 * if a device likely supports Classic Bluetooth A2DP audio streaming.
 */

#ifndef A2DP_VIA_BLE_DISCOVERY_H
#define A2DP_VIA_BLE_DISCOVERY_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_gap_ble_api.h"
#include "bluetooth_ble.h"

/**
 * @brief Known audio device entry
 */
typedef struct {
    const char* name_pattern;           // Device name pattern to match
    uint16_t manufacturer_id;           // BLE manufacturer ID
    bool supports_a2dp;                 // Known to support A2DP
    bool supports_le_audio;             // Known to support LE Audio
    const char* category;               // Device category
} known_audio_device_t;

/**
 * @brief Check falgs in set from advertised ble data for dual mode-capabiltiy
 * 
 * Parses data set from advertisement data to see if it has been decided that the device in
 * questions has dual mode capaibltiy or not
 * 
 * @param device_info device information set when advertisement data in read
 * 
 * @return true if device is capabile of dual-mode bluetooth. False if it is not
 */
bool parse_ble_adv_for_dual_mode(discovered_ble_device_t *device_info);

/**
 * @brief Probe device for A2DP support by attempting connection
 * 
 * Send out a sounding signal and see if a the device at the parameter address responds 
 * @param bda Device address to probe
 * @param timeout_ms How long to wait for connection attempt
 * @return true if A2DP connection succeeds, false otherwise
 */
bool probe_device_a2dp_support(esp_bd_addr_t bda, uint32_t timeout_ms);
#endif // A2DP_VIA_BLE_DISCOVERY_H
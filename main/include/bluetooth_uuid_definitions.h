#ifndef BLUETOOTH_UUID_DEFINITIONS_H
#define BLUETOOTH_UUID_DEFINITIONS_H

#include <stdint.h>

/**
 * @brief Classic Bluetooth Audio Service UUIDs
 * 
 * Standard 16-bit service UUIDs for Classic Bluetooth audio profiles.
 * These are defined in the Bluetooth SIG specifications.
 */
typedef enum{
    // A2DP (Advanced Audio Distribution Profile) Services
    BT_UUID_A2DP_SOURCE             = 0x110A,  // Audio source (sends audio)
    BT_UUID_A2DP_SINK               = 0x110B,  // Audio sink (receives audio)
    BT_UUID_AUDIO_SOURCE            = 0x110A,  // Alias for A2DP source
    BT_UUID_AUDIO_SINK              = 0x110B,  // Alias for A2DP sink
    
    // AVRCP (Audio/Video Remote Control Profile) Services  
    BT_UUID_AVRCP_CONTROLLER        = 0x110E,  // Remote control controller
    BT_UUID_AVRCP_TARGET            = 0x110C,  // Remote control target
    
    // Voice/Call Services
    BT_UUID_HEADSET                 = 0x1108,  // Headset profile
    BT_UUID_HEADSET_AUDIO_GATEWAY   = 0x1112,  // Headset audio gateway
    BT_UUID_HANDSFREE               = 0x111E,  // Hands-free profile
    BT_UUID_HANDSFREE_AUDIO_GATEWAY = 0x111F,  // Hands-free audio gateway
    
    // Other Audio-Related Services
    BT_UUID_GENERIC_AUDIO           = 0x1203,  // Generic audio service
} bt_classic_audio_uuid_t;

/**
 * @brief BLE Audio Service UUIDs
 * 
 * Standard 16-bit service UUIDs for BLE Audio (LE Audio) specification.
 * These services support the newer LC3 codec over Bluetooth Low Energy.
 */
typedef enum {
    // Core BLE Audio Services
    BLE_UUID_AUDIO_STREAM_CONTROL   = 0x184E,  // Audio Stream Control Service (ASCS)
    BLE_UUID_PUBLISHED_AUDIO_CAP    = 0x1850,  // Published Audio Capabilities Service (PACS)
    BLE_UUID_BROADCAST_AUDIO_SCAN   = 0x184F,  // Broadcast Audio Scan Service (BASS)
    
    // Audio Content Control Services
    BLE_UUID_MEDIA_CONTROL          = 0x1848,  // Media Control Service (MCS)
    BLE_UUID_GENERIC_MEDIA_CONTROL  = 0x1849,  // Generic Media Control Service (GMCS)
    
    // Volume and Rendering Services
    BLE_UUID_VOLUME_CONTROL         = 0x1844,  // Volume Control Service (VCS)
    BLE_UUID_VOLUME_OFFSET_CONTROL  = 0x1845,  // Volume Offset Control Service (VOCS)
    BLE_UUID_AUDIO_INPUT_CONTROL    = 0x1843,  // Audio Input Control Service (AICS)
    
    // Coordinated Set Services
    BLE_UUID_COORDINATED_SET_ID     = 0x1846,  // Coordinated Set Identification Service (CSIS)
} ble_audio_uuid_t;

/**
 * @brief Common BLE Service UUIDs (Non-Audio)
 * 
 * Standard BLE services that are commonly found in audio devices
 * for battery reporting, device information, etc.
 */
typedef enum {
    // Device Information Services
    BLE_UUID_DEVICE_INFORMATION     = 0x180A,  // Device Information Service
    BLE_UUID_BATTERY_SERVICE        = 0x180F,  // Battery Service
    BLE_UUID_GENERIC_ACCESS         = 0x1800,  // Generic Access Profile
    BLE_UUID_GENERIC_ATTRIBUTE      = 0x1801,  // Generic Attribute Profile
    
    // Human Interface Services
    BLE_UUID_HUMAN_INTERFACE_DEVICE = 0x1812,  // Human Interface Device Service
    BLE_UUID_SCAN_PARAMETERS        = 0x1813,  // Scan Parameters Service
} ble_common_uuid_t;

/**
 * @brief Bluetooth Manufacturer Company IDs
 * 
 * Standard company identifiers assigned by the Bluetooth SIG
 */
typedef enum {
    BT_COMPANY_ID_APPLE             = 0x004C,  // Apple Inc.
    BT_COMPANY_ID_SAMSUNG           = 0x0075,  // Samsung Electronics
    BT_COMPANY_ID_SONY              = 0x012D,  // Sony Corporation  
    BT_COMPANY_ID_HARMAN            = 0x0117,  // Harman International (JBL, AKG)
    BT_COMPANY_ID_BOSE              = 0x009E,  // Bose Corporation
    BT_COMPANY_ID_BEATS             = 0x01D7,  // Beats Electronics
    BT_COMPANY_ID_SENNHEISER        = 0x047F,  // Sennheiser Communications
    BT_COMPANY_ID_GOOGLE            = 0x00E0,  // Google Inc.
    BT_COMPANY_ID_MICROSOFT         = 0x0006,  // Microsoft Corporation
} bt_company_id_t;

#endif
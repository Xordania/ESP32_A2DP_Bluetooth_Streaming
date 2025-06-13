#ifndef BLUETOOTH_UUID_DEFINITIONS_H
#define BLUETOOTH_UUID_DEFINITIONS_H

#include <stdlib.h>
#include <stdbool.h>

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

// ========================================= Lookup Tables =========================================

// Array of valid Classic Bluetooth audio service UUIDs
// Sorted in ascending order for potential binary search optimization.
static const uint16_t classic_audio_uuids[] = {
    BT_UUID_HEADSET,                 // 0x1108
    BT_UUID_A2DP_SOURCE,             // 0x110A
    BT_UUID_A2DP_SINK,               // 0x110B
    BT_UUID_AVRCP_TARGET,            // 0x110C
    BT_UUID_AVRCP_CONTROLLER,        // 0x110E
    BT_UUID_HEADSET_AUDIO_GATEWAY,   // 0x1112
    BT_UUID_HANDSFREE,               // 0x111E
    BT_UUID_HANDSFREE_AUDIO_GATEWAY, // 0x111F
    BT_UUID_GENERIC_AUDIO,           // 0x1203
};


#define CLASSIC_AUDIO_UUID_COUNT (sizeof(classic_audio_uuids) / sizeof(classic_audio_uuids[0]))

/**
 * @brief Array of valid BLE Audio service UUIDs
 * 
 * Covers the main BLE Audio specification services.
 */
static const uint16_t ble_audio_uuids[] = {
    BLE_UUID_AUDIO_INPUT_CONTROL,    // 0x1843
    BLE_UUID_VOLUME_CONTROL,         // 0x1844
    BLE_UUID_VOLUME_OFFSET_CONTROL,  // 0x1845
    BLE_UUID_COORDINATED_SET_ID,     // 0x1846
    BLE_UUID_MEDIA_CONTROL,          // 0x1848
    BLE_UUID_GENERIC_MEDIA_CONTROL,  // 0x1849
    BLE_UUID_AUDIO_STREAM_CONTROL,   // 0x184E
    BLE_UUID_BROADCAST_AUDIO_SCAN,   // 0x184F
    BLE_UUID_PUBLISHED_AUDIO_CAP,    // 0x1850
};

#define BLE_AUDIO_UUID_COUNT (sizeof(ble_audio_uuids) / sizeof(ble_audio_uuids[0]))


/**
 * @brief Structure for UUID to name mapping
 */
typedef struct {
    uint16_t uuid;
    const char* name;
} uuid_name_map_t;

/**
 * @brief Classic Bluetooth audio service name lookup table
 * 
 * Cleaner than switch statements - just add new entries here.
 */
static const uuid_name_map_t classic_audio_names[] = {
    {BT_UUID_HEADSET,                 "Headset"},
    {BT_UUID_A2DP_SOURCE,             "A2DP Source"},
    {BT_UUID_A2DP_SINK,               "A2DP Sink"},
    {BT_UUID_AVRCP_TARGET,            "AVRCP Target"},
    {BT_UUID_AVRCP_CONTROLLER,        "AVRCP Controller"},
    {BT_UUID_HEADSET_AUDIO_GATEWAY,   "Headset Audio Gateway"},
    {BT_UUID_HANDSFREE,               "Hands-Free"},
    {BT_UUID_HANDSFREE_AUDIO_GATEWAY, "Hands-Free Audio Gateway"},
    {BT_UUID_GENERIC_AUDIO,           "Generic Audio"},
};

#define CLASSIC_AUDIO_NAMES_COUNT (sizeof(classic_audio_names) / sizeof(classic_audio_names[0]))


/**
 * @brief BLE Audio service name lookup table
 */
static const uuid_name_map_t ble_audio_names[] = {
    {BLE_UUID_AUDIO_INPUT_CONTROL,    "Audio Input Control Service"},
    {BLE_UUID_VOLUME_CONTROL,         "Volume Control Service"},
    {BLE_UUID_VOLUME_OFFSET_CONTROL,  "Volume Offset Control Service"},
    {BLE_UUID_COORDINATED_SET_ID,     "Coordinated Set Identification Service"},
    {BLE_UUID_MEDIA_CONTROL,          "Media Control Service"},
    {BLE_UUID_GENERIC_MEDIA_CONTROL,  "Generic Media Control Service"},
    {BLE_UUID_AUDIO_STREAM_CONTROL,   "Audio Stream Control Service"},
    {BLE_UUID_BROADCAST_AUDIO_SCAN,   "Broadcast Audio Scan Service"},
    {BLE_UUID_PUBLISHED_AUDIO_CAP,    "Published Audio Capabilities Service"},
};

#define BLE_AUDIO_NAMES_COUNT (sizeof(ble_audio_names) / sizeof(ble_audio_names[0]))

/**
 * @brief Company ID to name lookup table
 */
static const struct {
    bt_company_id_t company_id;
    const char* name;
} 

company_names[] = {
    {BT_COMPANY_ID_APPLE,       "Apple Inc."},
    {BT_COMPANY_ID_SAMSUNG,     "Samsung Electronics"},
    {BT_COMPANY_ID_SONY,        "Sony Corporation"},
    {BT_COMPANY_ID_HARMAN,      "Harman International (JBL/AKG)"},
    {BT_COMPANY_ID_BOSE,        "Bose Corporation"},
    {BT_COMPANY_ID_BEATS,       "Beats Electronics"},
    {BT_COMPANY_ID_SENNHEISER,  "Sennheiser Communications"},
    {BT_COMPANY_ID_GOOGLE,      "Google Inc."},
    {BT_COMPANY_ID_MICROSOFT,   "Microsoft Corporation"},
};

#define COMPANY_NAMES_COUNT (sizeof(company_names) / sizeof(company_names[0]))


// ========================================= Function Definnitions =========================================


/**
 * @brief Check if UUID represents a Classic Bluetooth audio service
 * 
 * @param uuid 16-bit service UUID to check
 * @return true if UUID is a Classic Bluetooth audio service
 */
bool is_classic_audio_service(uint16_t uuid);

/**
 * @brief Check if UUID represents a BLE Audio service
 * 
 * @param uuid 16-bit service UUID to check
 * @return true if UUID is a BLE Audio service
 */
bool is_ble_audio_service(uint16_t uuid);

/**
 * @brief Get human-readable name for Classic Bluetooth audio service UUID
 * 
 * @param uuid Service UUID (cast to bt_classic_audio_uuid_t for type safety)
 * @return String description of the service, valid for lifetime of program
 */
const char* get_classic_audio_service_name(bt_classic_audio_uuid_t uuid);

/**
 * @brief Get human-readable name for BLE Audio service UUID
 * 
 * @param uuid Service UUID (cast to ble_audio_uuid_t for type safety)
 * @return String description of the service, valid for lifetime of program
 */
const char* get_ble_audio_service_name(ble_audio_uuid_t uuid);

/**
 * @brief Get human-readable name for company ID
 * 
 * @param company_id Bluetooth SIG assigned company identifier
 * @return String name of the company, valid for lifetime of program
 */
const char* get_company_name(bt_company_id_t company_id);

/**
 * @brief Get count of known Classic Bluetooth audio services
 * 
 * Useful for iterating through all known services or validation.
 * 
 * @return Number of Classic Bluetooth audio services in lookup table
 */
size_t get_classic_audio_service_count(void);

/**
 * @brief Get count of known BLE Audio services
 * 
 * @return Number of BLE Audio services in lookup table
 */
size_t get_ble_audio_service_count(void);

#endif // BLUETOOTH_UUID_DEFINITIONS_H
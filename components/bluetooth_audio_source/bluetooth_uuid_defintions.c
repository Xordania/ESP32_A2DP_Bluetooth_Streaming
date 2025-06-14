#include "bluetooth_uuid_definitions.h"
#include <stdint.h>

/**
 * @brief Check if UUID is a Classic Bluetooth audio service (Linear Search)
 * 
 * Lienar search designed for small arrays
 *
 * @param uuid 16-bit service UUID to check
 * @return true if UUID is a Classic Bluetooth audio service
 */
bool is_classic_audio_service(uint16_t uuid)
{
    for (size_t i = 0; i < CLASSIC_AUDIO_UUID_COUNT; i++) {
        if (classic_audio_uuids[i] == uuid) {
            return true;
        }
    }
    return false;
}

/**
 * @brief Check if UUID represents a BLE Audio service
 * 
 * Clean array-based lookup for BLE Audio services.
 * 
 * @param uuid 16-bit service UUID to check
 * @return true if UUID is a BLE Audio service
 */
bool is_ble_audio_service(uint16_t uuid)
{
    for (size_t i = 0; i < BLE_AUDIO_UUID_COUNT; i++) {
        if (ble_audio_uuids[i] == uuid) {
            return true;
        }
    }
    return false;
}

/**
 * @brief Get human-readable name for Classic Bluetooth audio service UUID
 * 
 * Clean array-based lookup instead of switch statement.
 * 
 * @param uuid Service UUID (cast to bt_classic_audio_uuid_t for type safety)
 * @return String description of the service
 */
const char* get_classic_audio_service_name(bt_classic_audio_uuid_t uuid)
{
    for (size_t i = 0; i < CLASSIC_AUDIO_NAMES_COUNT; i++) {
        if (classic_audio_names[i].uuid == (uint16_t)uuid) {
            return classic_audio_names[i].name;
        }
    }
    return "Unknown Classic Audio Service";
}

/**
 * @brief Get human-readable name for BLE Audio service UUID
 * 
 * Clean array-based lookup instead of switch statement.
 * 
 * @param uuid Service UUID (cast to ble_audio_uuid_t for type safety)
 * @return String description of the service
 */
const char* get_ble_audio_service_name(ble_audio_uuid_t uuid)
{
    for (size_t i = 0; i < BLE_AUDIO_NAMES_COUNT; i++) {
        if (ble_audio_names[i].uuid == (uint16_t)uuid) {
            return ble_audio_names[i].name;
        }
    }
    return "Unknown BLE Audio Service";
}

/**
 * @brief Get human-readable name for company ID
 * 
 * Clean array-based lookup instead of switch statement.
 * 
 * @param company_id Bluetooth SIG assigned company identifier
 * @return String name of the company
 */
const char* get_company_name(bt_company_id_t company_id)
{
    for (size_t i = 0; i < COMPANY_NAMES_COUNT; i++) {
        if (company_names[i].company_id == company_id) {
            return company_names[i].name;
        }
    }
    return "Unknown Company";
}
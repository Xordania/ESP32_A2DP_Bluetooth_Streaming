#ifndef BLUETOOTH_DEVICE_ANALYZER_H
#define BLUETOOTH_DEVICE_ANALYZER_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_bt_defs.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "bluetooth_uuid_definitions.h"

/**
 * @brief BLE Audio streaming capabilities of a device
 * 
 * Bitmask representing different audio capabilities that can be discovered
 * through GATT service discovery. Multiple capabilities can be combined.
 */
typedef enum {
    BLE_AUDIO_CAP_NONE              = 0x00,     // No audio capabilities
    BLE_AUDIO_CAP_AUDIO_STREAM_CTL  = 0x01,     // Audio Stream Control Service
    BLE_AUDIO_CAP_PUBLISHED_AUDIO   = 0x02,     // Published Audio Capabilities
    BLE_AUDIO_CAP_VOLUME_CONTROL    = 0x04,     // Volume Control Service
    BLE_AUDIO_CAP_MEDIA_CONTROL     = 0x08,     // Media Control Service
    BLE_AUDIO_CAP_BROADCAST_AUDIO   = 0x10,     // Broadcast Audio
    BLE_AUDIO_CAP_COORDINATED_SET   = 0x20,     // Coordinated Set
    BLE_AUDIO_CAP_LEGACY_AUDIO      = 0x40,     // Has legacy A2DP-like services
    BLE_AUDIO_CAP_UNKNOWN           = 0x80      // Unknown/not determined
} ble_audio_capability_t;

/**
 * @brief BLE device analysis result structure
 * 
 * Contains comprehensive information about a BLE device's services and
 * audio capabilities discovered through GATT service discovery. This
 * structure is populated asynchronously and delivered via callback.
 */
typedef struct {
    esp_bd_addr_t bda;                      // Bluetooth device address
    esp_ble_addr_type_t addr_type;          // BLE address type (public/random)
    char name[64];                          // Device name
    int8_t rssi;                            // Signal strength from discovery
    
    // BLE service information
    uint16_t services[32];                  // All service UUIDs found
    uint8_t service_count;                  // Total number of services
    
    // Audio-specific services
    uint16_t audio_services[16];            // Audio-related service UUIDs
    uint8_t audio_service_count;            // Number of audio services
    uint32_t audio_capabilities;            // Bitmask of ble_audio_capability_t
    
    // Device information
    uint16_t appearance;                    // BLE appearance value
    uint16_t manufacturer_id;               // Manufacturer ID if available
    
    // Connection parameters (useful for audio streaming)
    uint16_t conn_interval;                 // Connection interval in ms
    uint16_t conn_latency;                  // Connection latency
    uint16_t supervision_timeout;           // Supervision timeout in ms
    
    // Analysis status
    bool analysis_complete;                 // Analysis finished
    bool connection_failed;                 // Failed to connect
    bool services_discovered;               // GATT discovery completed
} ble_device_analysis_result_t;


/**
 * @brief Callback for BLE device analysis completion
 * 
 * Called when device analysis is complete (either successfully or with failure).
 * The result structure contains all discovered information about the device.
 * This callback is invoked from the BLE stack context, so heavy processing
 * should be deferred to another task.
 * 
 * @param result Pointer to analysis results (valid only during callback)
 * 
 * @note The result pointer is only valid during the callback execution.
 *       Copy any needed data before returning from the callback.
 */
typedef void (*ble_device_analysis_cb_t)(ble_device_analysis_result_t *result);

/**
 * @brief Initialize BLE device analyzer module
 * 
 * Sets up the analyzer module by registering GATT client callbacks and
 * creating necessary resources for device analysis. Must be called after
 * BLE stack initialization (esp_bluedroid_enable) but before starting
 * any device analysis.
 * 
 * The module supports concurrent analysis of up to MAX_CONCURRENT_ANALYSES
 * devices. Each analysis involves:
 * - Connecting to the target device
 * - Performing GATT service discovery
 * - Categorizing audio capabilities based on discovered services
 * - Disconnecting and cleaning up resources
 * 
 * @return esp_err_t 
 *         - ESP_OK: Module initialized successfully
 *         - ESP_ERR_NO_MEM: Failed to allocate resources
 *         - ESP_ERR_INVALID_STATE: BLE stack not initialized
 */
esp_err_t ble_device_analyzer_init(void);

/**
 * @brief Analyze a discovered BLE device for audio capabilities
 * 
 * Initiates an asynchronous analysis of a BLE device to determine its
 * audio streaming capabilities. The analysis process involves:
 * 
 * 1. Establishing a BLE connection to the target device
 * 2. Performing complete GATT service discovery
 * 3. Identifying and categorizing audio-related services
 * 4. Determining device capabilities (LE Audio, legacy audio, etc.)
 * 5. Disconnecting from the device
 * 6. Invoking the callback with results
 * 
 * The analysis typically takes 2-5 seconds depending on the device and
 * connection parameters. The callback is guaranteed to be called even
 * if the analysis fails (with appropriate error flags set).
 * 
 * @param bda Bluetooth device address to analyze (6 bytes)
 * @param addr_type BLE address type (BLE_ADDR_TYPE_PUBLIC or BLE_ADDR_TYPE_RANDOM)
 * @param callback Function to call when analysis completes (required)
 * 
 * @return esp_err_t 
 *         - ESP_OK: Analysis started successfully
 *         - ESP_ERR_INVALID_ARG: Invalid parameters (null callback)
 *         - ESP_ERR_INVALID_STATE: Module not initialized or device already being analyzed
 *         - ESP_ERR_NO_MEM: No free analysis slots available
 *         - Other: BLE stack errors from esp_ble_gattc_open
 * 
 * @note Only one analysis per device address is allowed at a time
 * @note The device should be advertising and within range for successful analysis
 */
esp_err_t ble_analyze_device(esp_bd_addr_t bda, esp_ble_addr_type_t addr_type, 
                             ble_device_analysis_cb_t callback);

/**
 * @brief Analyze a BLE device using an existing connection
 * 
 * Performs service discovery and capability analysis on a device that
 * is already connected. This is useful when you've established a connection
 * for other purposes and want to analyze the device without reconnecting.
 * 
 * The analysis process is identical to ble_analyze_device except:
 * - No new connection is established
 * - The existing connection is NOT closed after analysis
 * - Analysis may be faster due to skipping connection establishment
 * 
 * @param conn_id Existing BLE connection ID from connection event
 * @param bda Bluetooth device address (for identification)
 * @param callback Function to call when analysis completes
 * 
 * @return esp_err_t 
 *         - ESP_OK: Analysis started successfully
 *         - ESP_ERR_INVALID_STATE: Module not initialized or device already being analyzed
 *         - ESP_ERR_NO_MEM: No free analysis slots available
 *         - Other: GATT errors from service discovery
 * 
 * @note The connection must be active and stable for successful analysis
 * @note Caller remains responsible for managing the connection lifecycle
 */
esp_err_t ble_analyze_connected_device(uint16_t conn_id, esp_bd_addr_t bda,
                                      ble_device_analysis_cb_t callback);

/**
 * @brief Check if device supports modern BLE Audio (LE Audio)
 * 
 * Examines the analysis results to determine if the device implements
 * the LE Audio specification. LE Audio devices typically support:
 * - Audio Stream Control Service (ASCS)
 * - Published Audio Capabilities Service (PACS)
 * - Optionally: Volume Control, Media Control, Broadcast Audio
 * 
 * This is the preferred audio standard for new BLE audio devices as it
 * offers better quality, lower latency, and more features than legacy
 * BLE audio solutions.
 * 
 * @param result Device analysis result from callback
 * 
 * @return true if device supports LE Audio core services, false otherwise
 * 
 * @note Returns false if analysis is incomplete or result is NULL
 */
bool ble_device_has_le_audio(const ble_device_analysis_result_t *result);

/**
 * @brief Check if device has any audio streaming capability
 * 
 * Determines if the device can stream or receive audio via BLE using
 * any supported method (LE Audio, proprietary services, etc.). This is
 * a broader check than ble_device_has_le_audio and includes legacy or
 * proprietary audio services.
 * 
 * @param result Device analysis result from callback
 * 
 * @return true if device has any audio capability flags set, false otherwise
 * 
 * @note Use ble_device_has_le_audio for checking modern LE Audio specifically
 */
bool ble_device_supports_audio(const ble_device_analysis_result_t *result);

/**
 * @brief Convert audio capabilities bitmask to human-readable string
 * 
 * Creates a space-separated string listing all audio capabilities found
 * in the provided bitmask. Useful for logging and debugging.
 * 
 * Example output: "AudioStreamCtl PublishedAudio VolumeCtl"
 * 
 * @param capabilities Bitmask of ble_audio_capability_t values
 * 
 * @return Static string buffer containing capability names
 * 
 * @warning The returned string uses a static buffer and will be overwritten
 *          by subsequent calls. Copy if you need persistent storage.
 */
const char* ble_audio_capabilities_to_string(uint32_t capabilities);

/**
 * @brief Get the primary (most important) audio service UUID
 * 
 * Analyzes the discovered audio services and returns the UUID of the
 * most important service for audio streaming. Priority order:
 * 1. Audio Stream Control Service (0x184E) - Core LE Audio
 * 2. Published Audio Capabilities (0x1850) - Core LE Audio
 * 3. First audio service found - Fallback for proprietary services
 * 
 * This helps determine which service to use when establishing an audio
 * stream with the device.
 * 
 * @param result Device analysis result containing service information
 * 
 * @return Primary audio service UUID, or 0 if no audio services found
 * 
 * @note Check audio_service_count > 0 before calling this function
 */
uint16_t ble_get_primary_audio_service(const ble_device_analysis_result_t *result);

/**
 * @brief Cancel an ongoing BLE device analysis
 * 
 * Stops an in-progress analysis and frees associated resources. If the
 * analyzer created a connection to the device, it will be closed. The
 * analysis callback will NOT be invoked after cancellation.
 * 
 * Use cases:
 * - User cancels device selection
 * - Application is shutting down
 * - Analysis is taking too long
 * 
 * @param bda Device address of the analysis to cancel
 * 
 * @return esp_err_t 
 *         - ESP_OK: Analysis cancelled successfully
 *         - ESP_ERR_NOT_FOUND: No ongoing analysis for this device
 * 
 * @note Safe to call even if analysis has already completed
 */
esp_err_t ble_cancel_device_analysis(esp_bd_addr_t bda);

/**
 * @brief Get number of devices currently being analyzed
 * 
 * Returns the count of active analysis operations. Useful for:
 * - Checking if analyzer is busy
 * - Implementing UI feedback (progress indicators)
 * - Resource management decisions
 * 
 * @return Number of active analyses (0 to MAX_CONCURRENT_ANALYSES)
 */
int ble_get_active_analysis_count(void);

/**
 * @brief Deinitialize BLE device analyzer module
 * 
 * Performs complete cleanup of the analyzer module:
 * - Cancels all ongoing analyses
 * - Closes any connections created by the analyzer
 * - Unregisters GATT client
 * - Frees all allocated resources
 * 
 * Should be called before shutting down the BLE stack or when the
 * analyzer is no longer needed. After calling this, ble_device_analyzer_init
 * must be called again before using any analyzer functions.
 * 
 * @note Safe to call multiple times
 * @note Ongoing analysis callbacks will NOT be invoked
 */
void ble_device_analyzer_deinit(void);

#endif // BLUETOOTH_DEVICE_ANALYZER_H
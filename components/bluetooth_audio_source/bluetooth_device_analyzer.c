/**
 * @file bluetooth_device_analyzer.c
 * @brief BLE device audio capability analyzer implementation
 *
 * Analyzes BLE devices to determine their audio streaming capabilities by
 * performing GATT service discovery. Focuses on identifying LE Audio services
 * as well as proprietary audio streaming services.
 * 
 * It has the ability to find Classic Bluetooth services even if they were found over 
 * BLE. This is useful for many streaming devices that act in this manner.
 *
 * Architecture:
 * - Maintains analysis state for multiple devices concurrently
 * - Uses ESP-IDF GATT client API for service discovery
 * - Connects temporarily to devices for analysis
 * - Delivers results asynchronously via callbacks
 */

#include "bluetooth_device_analyzer.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_gap_ble_api.h"
#include "esp_gatt_defs.h"

static const char *TAG = "BLE_ANALYZER";

// Maximum concurrent device analyses
#define MAX_CONCURRENT_ANALYSES 5

// Analysis timeout in seconds
#define ANALYSIS_TIMEOUT_SEC 10

/**
 * @brief Define the event base for Analyzer discovery
 */
ESP_EVENT_DEFINE_BASE(BLE_ANALYZER_EVENTS);

/**
 * @brief State for ongoing BLE device analysis
 */
typedef struct {
    bool in_use;                            // Slot is being used
    esp_bd_addr_t bda;                      // Device being analyzed
    ble_device_analysis_result_t result;    // Analysis results
    ble_device_analysis_cb_t callback;      // Completion callback
    
    // Connection state
    esp_gatt_if_t gattc_if;                 // GATT client interface
    uint16_t conn_id;                       // BLE connection ID
    bool connected;                         // Connection established
    bool own_connection;                    // We created the connection
    
    // Discovery state
    bool discovery_in_progress;             // Service discovery ongoing
    uint16_t current_service_handle;        // Current service being explored
    
    // Timing
    TickType_t start_time;                  // Analysis start time
} ble_analysis_state_t;

// Module state
static struct {
    bool initialized;
    ble_analysis_state_t analyses[MAX_CONCURRENT_ANALYSES];
    SemaphoreHandle_t mutex;
    esp_gatt_if_t gattc_if;                // Shared GATT client interface
    uint8_t app_id;                        // GATT client app ID
} analyzer_state = {0};


// ============================================================================
// EVENT POSTING FUNCTIONS
// ============================================================================
/**
 * @brief Post device discovered event
 * 
 * Posts BLE_DISCOVERY_DEVICE_FOUND event with complete device information.
 */
static void post_analysis_complete_event(ble_device_analysis_result_t *analyzer)
{
    
    // Create event data
    ble_analysis_complete_event_data_t event_data = {
       .device = *analyzer,
    };

    // Post event
    esp_err_t ret = esp_event_post(BLE_ANALYZER_EVENTS,
                                  BLE_ANALYZER_COMPLETE,
                                  &event_data,
                                  sizeof(event_data),
                                  100 / portTICK_PERIOD_MS);  // 100ms timeout
    
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to post device discovered event: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGD(TAG, "For device: %s", analyzer->name);
    }
}

// ============================================================================
// FUNCTIONS
// ============================================================================

/**
 * @brief Find analysis state by device address
 */
static ble_analysis_state_t* find_analysis_by_address(esp_bd_addr_t bda)
{
    for (int i = 0; i < MAX_CONCURRENT_ANALYSES; i++) {
        if (analyzer_state.analyses[i].in_use &&
            memcmp(analyzer_state.analyses[i].bda, bda, ESP_BD_ADDR_LEN) == 0) {
            return &analyzer_state.analyses[i];
        }
    }
    return NULL;
}

/**
 * @brief Find analysis state by connection ID
 */
static ble_analysis_state_t* find_analysis_by_conn_id(uint16_t conn_id)
{
    for (int i = 0; i < MAX_CONCURRENT_ANALYSES; i++) {
        if (analyzer_state.analyses[i].in_use &&
            analyzer_state.analyses[i].conn_id == conn_id) {
            return &analyzer_state.analyses[i];
        }
    }
    return NULL;
}

/**
 * @brief Allocate analysis state for new device
 */
static ble_analysis_state_t* allocate_analysis_state(void)
{
    for (int i = 0; i < MAX_CONCURRENT_ANALYSES; i++) {
        if (!analyzer_state.analyses[i].in_use) {
            memset(&analyzer_state.analyses[i], 0, sizeof(ble_analysis_state_t));
            analyzer_state.analyses[i].in_use = true;
            analyzer_state.analyses[i].start_time = xTaskGetTickCount();
            return &analyzer_state.analyses[i];
        }
    }
    return NULL;
}

/**
 * @brief Free analysis state
 */
static void free_analysis_state(ble_analysis_state_t *state)
{
    if(!state || !state->in_use){
        ESP_LOGW(TAG, "Attempted to free invalid or unsused analysis state");
        return;
    }

        ESP_LOGI(TAG, "Freeing analysis state for device %02x:%02x:%02x:%02x:%02x:%02x",
            state->bda[0], state->bda[1], state->bda[2],
            state->bda[3], state->bda[4], state->bda[5]);

    // Mark as not in use and clear the data. The disconnect event will handle the clean up
    memset(state, 0, sizeof(ble_analysis_state_t));
    state->in_use = false;
    state->gattc_if = ESP_GATT_IF_NONE;
    state->conn_id = 0xFFFF;
}

/**
 * @brief Analyze discovered services and categorize audio capabilities
 */
static void analyze_services(ble_analysis_state_t *state)
{
    state->result.audio_capabilities = BLE_AUDIO_CAP_NONE;
    state->result.audio_service_count = 0;
    
    // Check each discovered service
    for (int i = 0; i < state->result.service_count; i++) {
        uint16_t uuid = state->result.services[i];
        
        // Check if it's an audio service
        if (is_ble_audio_service(uuid)) {
            // Add to audio services list
            if (state->result.audio_service_count < 16) {
                state->result.audio_services[state->result.audio_service_count++] = uuid;
            }
            
            // Set capability flags based on service type
            switch (uuid) {
                case BLE_UUID_AUDIO_STREAM_CONTROL:
                    state->result.audio_capabilities |= BLE_AUDIO_CAP_AUDIO_STREAM_CTL;
                    break;
                case BLE_UUID_PUBLISHED_AUDIO_CAP:
                    state->result.audio_capabilities |= BLE_AUDIO_CAP_PUBLISHED_AUDIO;
                    break;
                case BLE_UUID_VOLUME_CONTROL:
                    state->result.audio_capabilities |= BLE_AUDIO_CAP_VOLUME_CONTROL;
                    break;
                case BLE_UUID_MEDIA_CONTROL:
                case BLE_UUID_GENERIC_MEDIA_CONTROL:
                    state->result.audio_capabilities |= BLE_AUDIO_CAP_MEDIA_CONTROL;
                    break;
                case BLE_UUID_BROADCAST_AUDIO_SCAN:
                    state->result.audio_capabilities |= BLE_AUDIO_CAP_BROADCAST_AUDIO;
                    break;
                case BLE_UUID_COORDINATED_SET_ID:
                    state->result.audio_capabilities |= BLE_AUDIO_CAP_COORDINATED_SET;
                    break;
            }
            
            ESP_LOGI(TAG, "Found audio service: 0x%04X (%s)", 
                     uuid, get_ble_audio_service_name(uuid));
        }
    }
    
    // Log summary
    if (state->result.audio_service_count > 0) {
        ESP_LOGI(TAG, "Device has %d audio services", state->result.audio_service_count);
    } else {
        ESP_LOGI(TAG, "No audio services found on device");
    }
}

/**
 * @brief Complete device analysis and invoke callback
 */
static void complete_analysis(ble_analysis_state_t *state)
{
    // Check there is analysis happening on the passed state
    if (!state || !state->in_use) {
        ESP_LOGW(TAG, "Attempted to complete invalid analysis");
        return;
    }

    // Prevents completion firing twice
    if (state->result.analysis_complete) {
        ESP_LOGW(TAG, "Analysis already completed for this device");
        return;
    }
    
    ESP_LOGI(TAG, "Completing analysis for device %02x:%02x:%02x:%02x:%02x:%02x",
             state->bda[0], state->bda[1], state->bda[2],
             state->bda[3], state->bda[4], state->bda[5]);
    
    state->result.analysis_complete = true;
    
    // Analyze the services found
    analyze_services(state);
    
    // Log analysis duration
    TickType_t duration = xTaskGetTickCount() - state->start_time;
    ESP_LOGI(TAG, "BLE device analysis completed in %lu ms", 
             duration * portTICK_PERIOD_MS);
    
    // Make a copy of the callback and result before requesting disconnect
    ble_device_analysis_cb_t callback = state->callback;
    ble_device_analysis_result_t result_copy = state->result;


    // Request disconnect if we own the connection
    if (state->connected && state->own_connection && 
        state->gattc_if != ESP_GATT_IF_NONE && 
        state->conn_id != 0xFFFF) {
        
        ESP_LOGI(TAG, "Requesting disconnect from device (conn_id: %d)", state->conn_id);
        esp_err_t ret = esp_ble_gattc_close(state->gattc_if, state->conn_id);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to request disconnect: %s", esp_err_to_name(ret));
            // If disconnect fails, free state immediately and call callback
            free_analysis_state(state);
            if (callback) {
                callback(&result_copy);
            }
        }

        // Post the event
        post_analysis_complete_event(&result_copy);

        // If disconnect succeeds, the disconnect event will handle cleanup and callback
    } else {
        // No connection to close, free state immediately and call callback
        free_analysis_state(state);
        if (callback) {
            callback(&result_copy);
        }
    }
}

/**
 * @brief GATT client callback for BLE operations
 */
static void gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, 
                               esp_ble_gattc_cb_param_t *param)
{
    ble_analysis_state_t *state;
    
    switch (event) {
    case ESP_GATTC_REG_EVT:
        ESP_LOGI(TAG, "GATT client registered, gattc_if %d, status %d", 
                 gattc_if, param->reg.status);

        // Double check the registration has happened peroperly then set the gatt connection interface
        if (param->reg.status == ESP_GATT_OK && param->reg.app_id == analyzer_state.app_id) {
            analyzer_state.gattc_if = gattc_if;
        }
        break;
        
    // Physical conenction to the device has been made
    case ESP_GATTC_CONNECT_EVT:
        ESP_LOGI(TAG, "Connected to device, conn_id %d", 
                 param->connect.conn_id);
        
        // Ensure that nothing else can make changes this analyzer while checking connections
        xSemaphoreTake(analyzer_state.mutex, portMAX_DELAY);
        state = find_analysis_by_address(param->connect.remote_bda);

        state->conn_id = param->connect.conn_id;
        state->gattc_if = gattc_if;
        state->connected = true;
        
        // Store connection parameters
        state->result.conn_interval = param->connect.conn_params.interval;
        state->result.conn_latency = param->connect.conn_params.latency;
        state->result.supervision_timeout = param->connect.conn_params.timeout;
        
        ESP_LOGI(TAG, "Connection params - interval: %d, latency: %d, timeout: %d",
                    state->result.conn_interval,
                    state->result.conn_latency,
                    state->result.supervision_timeout);
        
        // Start service discovery
        ESP_LOGI(TAG, "Starting service discovery...");
        esp_err_t ret = esp_ble_gattc_search_service(gattc_if, param->connect.conn_id, NULL);

        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Service discovery start failed: %s", esp_err_to_name(ret));
            complete_analysis(state);
        } else {
            state->discovery_in_progress = true;
        }

        xSemaphoreGive(analyzer_state.mutex);
        break;
        
    // A new service/characteristic has been found
    case ESP_GATTC_SEARCH_RES_EVT: {
        uint16_t uuid = 0;
        bool is_standard_service = false;

        // Extract UUID based on length
        if (param->search_res.srvc_id.uuid.len == ESP_UUID_LEN_16) {
            uuid = param->search_res.srvc_id.uuid.uuid.uuid16;
            is_standard_service = true;
        } else if (param->search_res.srvc_id.uuid.len == ESP_UUID_LEN_32) {
            // For 32-bit UUIDs, we only store if they fit in 16 bits
            uuid = param->search_res.srvc_id.uuid.uuid.uuid32 & 0xFFFF;
            is_standard_service = true;

        }else if(param->search_res.srvc_id.uuid.len == ESP_UUID_LEN_128){
            // 128-bit UUIDs are often proprietary services
            uint8_t *uuid128 = param->search_res.srvc_id.uuid.uuid.uuid128;
        
             // Log the full 128-bit UUID for debugging
            ESP_LOGI(TAG, "Service found: 128-bit UUID: %02x%02x%02x%02x-%02x%02x-%02x%02x-%02x%02x-%02x%02x%02x%02x%02x%02x",
                 uuid128[15], uuid128[14], uuid128[13], uuid128[12],
                 uuid128[11], uuid128[10], uuid128[9], uuid128[8],
                 uuid128[7], uuid128[6], uuid128[5], uuid128[4],
                 uuid128[3], uuid128[2], uuid128[1], uuid128[0]);

            // Check if it's a Bluetooth SIG standard service with 128-bit representation
            // Standard services have the form: 0000XXXX-0000-1000-8000-00805F9B34FB
            if (uuid128[15] == 0x00 && uuid128[14] == 0x00 && 
                uuid128[11] == 0x00 && uuid128[10] == 0x00 &&
                uuid128[9] == 0x10 && uuid128[8] == 0x00 &&
                uuid128[7] == 0x80 && uuid128[6] == 0x00 &&
                uuid128[5] == 0x00 && uuid128[4] == 0x80 &&
                uuid128[3] == 0x5F && uuid128[2] == 0x9B &&
                uuid128[1] == 0x34 && uuid128[0] == 0xFB) {

                // Extract the 16-bit service UUID from bytes 12-13
                uuid = (uuid128[13] << 8) | uuid128[12];
                is_standard_service = true;
                ESP_LOGI(TAG, "128-bit UUID is standard service: 0x%04X", uuid);
            }else{
                // uuid is for propritary service
                ESP_LOGI(TAG, "Found proprietary 128-bit service UUID");

                // For JBL devices, we should investigate if this is their audio service
                // For now, we'll skip storing it in our 16-bit array
                is_standard_service = false;
            }
        }else{
            ESP_LOGW(TAG, "Unknown UUID length: %d", param->search_res.srvc_id.uuid.len);       
        }
        
        ESP_LOGI(TAG, "Service found: UUID length %d, UUID16: 0x%04X", 
                 param->search_res.srvc_id.uuid.len, uuid);
        
        // Ensure that nothing else can make changes this analyzer while checking services
        xSemaphoreTake(analyzer_state.mutex, portMAX_DELAY);
        state = find_analysis_by_conn_id(param->search_res.conn_id);
        
        if (state->result.service_count < 32) {
            state->result.services[state->result.service_count++] = uuid;
        }
        
        xSemaphoreGive(analyzer_state.mutex);
        break;
    }
    
    // Completed search of peripherals services
    case ESP_GATTC_SEARCH_CMPL_EVT:
        ESP_LOGI(TAG, "Service discovery complete, status: %d", param->search_cmpl.status);
        
        xSemaphoreTake(analyzer_state.mutex, portMAX_DELAY);
        state = find_analysis_by_conn_id(param->search_cmpl.conn_id);
        
        // Possible that this fails and the state never closes?
        state->discovery_in_progress = false;
        state->result.services_discovered = true;
        
        // Analysis is complete
        complete_analysis(state);
        
        
        xSemaphoreGive(analyzer_state.mutex);
        break;
        
    // Physical connection in terminated
    case ESP_GATTC_DISCONNECT_EVT:
        ESP_LOGI(TAG, "Disconnected from device, reason: 0x%02x", param->disconnect.reason);
        
        xSemaphoreTake(analyzer_state.mutex, portMAX_DELAY);
        state = find_analysis_by_address(param->disconnect.remote_bda);
        
        if(state && state->in_use){
            ESP_LOGI(TAG, "Hanlding disconnect for analysis state");

            // Make copies before freeign state
            ble_device_analysis_cb_t callback = state->callback;
            ble_device_analysis_result_t result_copy = state->result;
            bool analysis_was_complete = state->result.analysis_complete;

            // Mark as disconnected
            state->connected = false;

            // If analysis wasn't completed, mark as failed
            if(!analysis_was_complete){
                ESP_LOGW(TAG, "Disconnected before analysis complete - marking as failed");
                result_copy.analysis_complete = true;
                result_copy.connection_failed = true;
            }

            // Free the state
            free_analysis_state(state);

            // Call callback with results (only if analysis was complete or we're reporting failure)
            if(callback && (analysis_was_complete || result_copy.connection_failed)){
                callback(&result_copy);
            }
        }else{
            ESP_LOGD(TAG, "Disconenct event for unknown or already free device");
        }
        
        xSemaphoreGive(analyzer_state.mutex);
        break;

    default:
        break;
    }
}


/**
 * @brief Initialize BLE device analyzer module
 */
esp_err_t ble_device_analyzer_init(void)
{
    if (analyzer_state.initialized) {
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Initializing BLE device analyzer");
    
    // Create mutex
    analyzer_state.mutex = xSemaphoreCreateMutex();
    if (!analyzer_state.mutex) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }
    
    // Register GATT client callback
    esp_err_t ret = esp_ble_gattc_register_callback(gattc_event_handler);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GATTC callback registration failed: %s", esp_err_to_name(ret));
        vSemaphoreDelete(analyzer_state.mutex);
        return ret;
    }
    
    // Register GATT client app
    analyzer_state.app_id = 0;  // Use app_id 0 for analyzer
    ret = esp_ble_gattc_app_register(analyzer_state.app_id);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GATTC app registration failed: %s", esp_err_to_name(ret));
        vSemaphoreDelete(analyzer_state.mutex);
        return ret;
    }
    
    analyzer_state.initialized = true;
    ESP_LOGI(TAG, "BLE device analyzer initialized successfully");   

    return ESP_OK;
}

/**
 * @brief Analyze a discovered BLE device
 */
esp_err_t ble_analyze_device(esp_bd_addr_t bda, esp_ble_addr_type_t addr_type, 
                             ble_device_analysis_cb_t callback)
{
    if (!analyzer_state.initialized) {
        ESP_LOGE(TAG, "Analyzer not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (!callback) {
        ESP_LOGE(TAG, "Callback is required");
        return ESP_ERR_INVALID_ARG;
    }
    
    xSemaphoreTake(analyzer_state.mutex, portMAX_DELAY);
    
    // Check if already analyzing this device
    if (find_analysis_by_address(bda)) {
        xSemaphoreGive(analyzer_state.mutex);
        ESP_LOGW(TAG, "Already analyzing this device");
        return ESP_ERR_INVALID_STATE;
    }
    
    // Allocate analysis state
    ble_analysis_state_t *state = allocate_analysis_state();
    if (!state) {
        xSemaphoreGive(analyzer_state.mutex);
        ESP_LOGE(TAG, "No free analysis slots");
        return ESP_ERR_NO_MEM;
    }
    
    // Initialize state
    memcpy(state->bda, bda, ESP_BD_ADDR_LEN);
    memcpy(state->result.bda, bda, ESP_BD_ADDR_LEN);
    state->result.addr_type = addr_type;
    state->callback = callback;
    state->own_connection = true;  // We're creating the connection
    
    xSemaphoreGive(analyzer_state.mutex);
    
    ESP_LOGI(TAG, "Starting BLE device analysis for %02x:%02x:%02x:%02x:%02x:%02x",
             bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);
    
    // Connect to BLE device
    esp_err_t ret = esp_ble_gattc_open(analyzer_state.gattc_if, bda, addr_type, true);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to connect to BLE device: %s", esp_err_to_name(ret));
        free_analysis_state(state);
        return ret;
    }
    
    return ESP_OK;
}

/*
 * @brief Analyze a device with existing connection
 */
esp_err_t ble_analyze_connected_device(uint16_t conn_id, esp_bd_addr_t bda,
                                      ble_device_analysis_cb_t callback)
{
    if (!analyzer_state.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    xSemaphoreTake(analyzer_state.mutex, portMAX_DELAY);
    
    // Check if already analyzing
    if (find_analysis_by_address(bda)) {
        xSemaphoreGive(analyzer_state.mutex);
        return ESP_ERR_INVALID_STATE;
    }
    
    // Allocate state
    ble_analysis_state_t *state = allocate_analysis_state();
    if (!state) {
        xSemaphoreGive(analyzer_state.mutex);
        return ESP_ERR_NO_MEM;
    }
    
    // Initialize for existing connection
    memcpy(state->bda, bda, ESP_BD_ADDR_LEN);
    memcpy(state->result.bda, bda, ESP_BD_ADDR_LEN);
    state->callback = callback;
    state->conn_id = conn_id;
    state->connected = true;
    state->own_connection = false;  // We didn't create this connection
    state->gattc_if = analyzer_state.gattc_if;
    
    xSemaphoreGive(analyzer_state.mutex);
    
    // Start service discovery on existing connection
    esp_err_t ret = esp_ble_gattc_search_service(analyzer_state.gattc_if, conn_id, NULL);
    if (ret != ESP_OK) {
        free_analysis_state(state);
        return ret;
    }
    
    state->discovery_in_progress = true;
    return ESP_OK;
}

/**
 * @brief Check if device supports any audio
 */
bool ble_device_supports_audio(const ble_device_analysis_result_t *result)
{
    if (!result || !result->analysis_complete) {
        return false;
    }
    
    return result->audio_capabilities != BLE_AUDIO_CAP_NONE;
}

/**
 * @brief Check if device supports modern BLE Audio (LE Audio)
 */
bool ble_device_has_le_audio(const ble_device_analysis_result_t *result)
{
    if (!result || !result->analysis_complete) {
        return false;
    }
    
    // Check for core LE Audio services
    // According to LE Audio spec, both ASCS and PACS are mandatory
    uint32_t required_caps = BLE_AUDIO_CAP_AUDIO_STREAM_CTL | BLE_AUDIO_CAP_PUBLISHED_AUDIO;
    
    // Check if device has both required capabilities
    if ((result->audio_capabilities & required_caps) == required_caps) {
        return true;
    }
    
    return false;
}

/*
 * @brief Get human-readable string for capabilities
 */
const char* ble_audio_capabilities_to_string(uint32_t capabilities)
{
    static char buffer[256];
    buffer[0] = '\0';
    
    if (capabilities == BLE_AUDIO_CAP_NONE) {
        return "None";
    }
    
    if (capabilities & BLE_AUDIO_CAP_AUDIO_STREAM_CTL) {
        strcat(buffer, "AudioStreamCtl ");
    }
    if (capabilities & BLE_AUDIO_CAP_PUBLISHED_AUDIO) {
        strcat(buffer, "PublishedAudio ");
    }
    if (capabilities & BLE_AUDIO_CAP_VOLUME_CONTROL) {
        strcat(buffer, "VolumeCtl ");
    }
    if (capabilities & BLE_AUDIO_CAP_MEDIA_CONTROL) {
        strcat(buffer, "MediaCtl ");
    }
    if (capabilities & BLE_AUDIO_CAP_BROADCAST_AUDIO) {
        strcat(buffer, "Broadcast ");
    }
    if (capabilities & BLE_AUDIO_CAP_COORDINATED_SET) {
        strcat(buffer, "CoordinatedSet ");
    }
    
    // Remove trailing space
    size_t len = strlen(buffer);
    if (len > 0 && buffer[len-1] == ' ') {
        buffer[len-1] = '\0';
    }
    
    return buffer;
}

/**
 * @brief Get primary audio service UUID
 */
uint16_t ble_get_primary_audio_service(const ble_device_analysis_result_t *result)
{
    if (!result || result->audio_service_count == 0) {
        return 0;
    }
    
    // Prioritize core LE Audio services
    for (int i = 0; i < result->audio_service_count; i++) {
        uint16_t uuid = result->audio_services[i];
        if (uuid == BLE_UUID_AUDIO_STREAM_CONTROL ||
            uuid == BLE_UUID_PUBLISHED_AUDIO_CAP) {
            return uuid;
        }
    }
    
    // Return first audio service if no core services found
    return result->audio_services[0];
}

/**
 * @brief Cancel ongoing analysis
 */
esp_err_t ble_cancel_device_analysis(esp_bd_addr_t bda)
{
    xSemaphoreTake(analyzer_state.mutex, portMAX_DELAY);
    
    ble_analysis_state_t *state = find_analysis_by_address(bda);
    if (state) {
        ESP_LOGI(TAG, "Cancelling analysis for device");
        free_analysis_state(state);
        xSemaphoreGive(analyzer_state.mutex);
        return ESP_OK;
    }
    
    xSemaphoreGive(analyzer_state.mutex);
    return ESP_ERR_NOT_FOUND;
}

/**
 * @brief Get active analysis count
 */
int ble_get_active_analysis_count(void)
{
    int count = 0;
    
    xSemaphoreTake(analyzer_state.mutex, portMAX_DELAY);
    
    for (int i = 0; i < MAX_CONCURRENT_ANALYSES; i++) {
        if (analyzer_state.analyses[i].in_use) {
            count++;
        }
    }
    
    xSemaphoreGive(analyzer_state.mutex);
    
    return count;
}

/**
 * @brief Deinitialize analyzer module
 */
void ble_device_analyzer_deinit(void)
{
    if (!analyzer_state.initialized) {
        return;
    }
    
    ESP_LOGI(TAG, "Deinitializing BLE device analyzer");
    
    // Cancel all ongoing analyses
    for (int i = 0; i < MAX_CONCURRENT_ANALYSES; i++) {
        if (analyzer_state.analyses[i].in_use) {
            free_analysis_state(&analyzer_state.analyses[i]);
        }
    }
    
    // Unregister GATT client
    if (analyzer_state.gattc_if != ESP_GATT_IF_NONE) {
        esp_ble_gattc_app_unregister(analyzer_state.gattc_if);
    }
    
    // Delete mutex
    if (analyzer_state.mutex) {
        vSemaphoreDelete(analyzer_state.mutex);
    }
    
    analyzer_state.initialized = false;
    ESP_LOGI(TAG, "BLE device analyzer deinitialized");
}
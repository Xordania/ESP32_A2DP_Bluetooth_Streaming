
#include <string.h>
#include <math.h>
#include "main.h"
#include "bluetooth_classic.h"
#include "bluetooth_common.h"
#include "bluetooth_ble.h"
#include "bluetooth_uuid_definitions.h"
#include "bluetooth_device_analyzer.h"
#include "a2dp_via_ble_discovery.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "esp_err.h"
#include "esp_bt.h"
#include "esp_log.h"
#include "bluetooth_classic.h"
#include "bluetooth_a2dp.h"


#define SINE_FREQ 440
#define SAMPLE_RATE 44100

static const char *TAG = "MAIN";

// Target device information
static bool jbl_device_found = false;
static discovered_ble_device_t target_jbl_device;

static sine_wave_config_t wave;


/**
 * @brief Generate sine wave for 16-bit mono audio
 * 
 * Simple example audio generator for 16-bit mono audio.
 * 
 * @param buffer Output buffer for audio samples (int16_t mono pairs)
 * @param samples Number of samples per channel to generate
 * @param sample_offset Starting sample index for phase continuity  
 * @param context Generator context (sine_wave_config_t*)
 */
static void generate_sine_wave_16bit(int16_t *buffer, size_t samples, void *context)
{
    if (!context) {
        ESP_LOGE(TAG, "No context provided to sine_wave generator");
        return;
    }
    
    sine_wave_config_t *config = (sine_wave_config_t*)context;
    float frequency = config->frequency;
    float amplitude = config->amplitude;  
    float current_phase = config->phase;
    
    // Calculate phase increment per sample
    float phase_increment = 2.0f * M_PI * frequency / config->sample_rate;
   
    for (size_t i = 0; i < samples; i++) {
        float sine_value = sinf(current_phase + (i * phase_increment));
        int16_t sample = (int16_t)(sine_value * amplitude * 32767);
       
        // Fill mono sample
        buffer[i] = sample;  

        // Advance phase for next sample
        config->phase += phase_increment;
    
        // Wrap phase to prevent overflow
        if (config->phase >= 2.0f * M_PI) {
            config->phase -= 2.0f * M_PI;
        }
    }
}

/**
 * @brief A2DP source data callback for audio streaming
 *
 * This callback is invoked by the ESP-IDF Bluetooth stack to request audio data
 * for transmission over A2DP. The function generates real-time audio data and
 * fills the provided buffer. Called from the A2DP source task context.
 * 
 * @param buf Pointer to audio data buffer to fill with PCM samples
 * @param len Number of bytes requested by the stack:
 *            - Positive value: Number of bytes to write to buffer
 *            - Negative value (typically -1): Flush request, clear any buffered data
 *
 * @return int32_t Number of bytes actually written to buffer:
 *                 - Should equal 'len' for successful data provision
 *                 - Return 0 for flush requests or when no data available
 *                 - Stack expects this to match requested length to avoid underflows
 *
 * @note Buffer format: 16-bit signed PCM samples, stereo interleaved
 * @note Function must execute quickly to maintain real-time audio streaming
 * @note Underflow warnings occur if insufficient data is provided
 */
static int32_t a2dp_data_callback(uint8_t *buf, int32_t len) {
    // Handle flush request (when len is -1)
    if (len < 0) {
        ESP_LOGI(TAG, "Data buffer flush requested");
        return 0;
    }
    else{
        generate_sine_wave_16bit((int16_t *)buf, len, &wave);
        return len;
    }
}


// The set of functions below here (setup_data_request and start_playback_connection_complete) may be 
// better served as finite state machine checks inside of a FreeRTOS task that has their values changed
// by event handle callbacks (maybe)
static void start_playback_connection_complete(void* event_handler_arg, 
                                       esp_event_base_t event_base,
                                       int32_t event_id, 
                                       void* event_data){
    
    switch(event_id){
        case A2DP_CONNECTION_COMPLETE:{    
            a2dp_connection_complete_event_data_t *data = (a2dp_connection_complete_event_data_t*)event_data;
            if(equal_bda(target_jbl_device.bda, data->conn.conn_stat.remote_bda)){

                // init wave with starter variables
                wave.amplitude = 0.05;
                wave.frequency = 440;
                wave.sample_rate = 44100;

                // Start audio callback so the A2DP stack start to request data
                esp_a2d_media_ctrl(ESP_A2D_MEDIA_CTRL_START);
            }
        }
    }
}

static void setup_data_request(void* event_handler_arg, 
                                       esp_event_base_t event_base,
                                       int32_t event_id, 
                                       void* event_data){
    
    switch(event_id){
        case A2DP_INITIALIZATION_COMPLETE:{
            esp_err_t ret = esp_a2d_source_register_data_callback(a2dp_data_callback);
            if(ret != ESP_OK){
                ESP_LOGE(TAG, "Data callback registration failed: %s", esp_err_to_name(ret));
            }
                


            // Send out sounding on a2dp to jbl device to check if it will connect
            if(probe_device_a2dp_support(target_jbl_device.bda, 1000) == true){
                ESP_LOGI(TAG, "Detected A2DP support");

                ret = esp_event_handler_register(A2DP_CONNECTION_EVENTS,    // Event base
                                    ESP_EVENT_ANY_ID,                           // Any event ID from this base
                                    start_playback_connection_complete,         // Handler function
                                    NULL);                                      // User data (none needed)
            }else{
                ESP_LOGE(TAG, "No A2DP support detected");
            }
        }
    }
}

/**
 * @brief Check if discovered device is likely a JBL FLIP 5
 * 
 * Uses multiple criteria to identify JBL devices:
 * - Device name contains "JBL" or "FLIP"
 * - Manufacturer data indicates Harman International (JBL's parent company)
 * - Signal strength is reasonable for connection
 */
static bool is_jbl_flip5_device(const discovered_ble_device_t *device)
{
    // Check device name for JBL indicators
    if (device->has_name) {
        if (strstr(device->name, "JBL") != NULL || 
            strstr(device->name, "FLIP") != NULL ||
            strstr(device->name, "Flip") != NULL) {
            ESP_LOGI(TAG, "Found device with JBL name: %s", device->name);
            return true;
        }
    }
    
    // Check manufacturer data for Harman International (JBL's parent company)
    if (device->manufacturer_data_len >= 2) {
        uint16_t company_id = (device->manufacturer_data[1] << 8) | device->manufacturer_data[0];
        if (company_id == BT_COMPANY_ID_HARMAN) {
            ESP_LOGI(TAG, "Found Harman International device (JBL parent company)");
            return true;
        }
    }
    
    return false;
}

/**
 * @brief Callback for BLE device analysis completion
 * 
 * This is called when the device analyzer finishes analyzing a potential
 * JBL device to determine its audio streaming capabilities.
 */
static void jbl_analysis_complete_cb(ble_device_analysis_result_t *result)
{
    ESP_LOGI(TAG, "=== JBL Device Analysis Complete ===");
    ESP_LOGI(TAG, "Device: %02x:%02x:%02x:%02x:%02x:%02x",
             result->bda[0], result->bda[1], result->bda[2],
             result->bda[3], result->bda[4], result->bda[5]);
    
    if (result->connection_failed) {
        ESP_LOGE(TAG, "Failed to connect to JBL device for analysis");
        return;
    }
    
    // Log discovered services
    ESP_LOGI(TAG, "Total services discovered: %d", result->service_count);
    ESP_LOGI(TAG, "Audio services found: %d", result->audio_service_count);
    
    // Check for BLE Audio capabilities
    if (ble_device_has_le_audio(result)) {
        ESP_LOGI(TAG, "JBL device supports LE Audio!");
        ESP_LOGI(TAG, "Capabilities: %s", ble_audio_capabilities_to_string(result->audio_capabilities));
        
        // Log specific audio services found
        for (int i = 0; i < result->audio_service_count; i++) {
            uint16_t uuid = result->audio_services[i];
            ESP_LOGI(TAG, "  Audio Service: 0x%04X (%s)", 
                     uuid, get_ble_audio_service_name(uuid));
        }
        
        // Get primary audio service for streaming
        uint16_t primary_service = ble_get_primary_audio_service(result);
        if (primary_service != 0) {
            ESP_LOGI(TAG, "Primary audio service: 0x%04X (%s)", 
                     primary_service, get_ble_audio_service_name(primary_service));
        }
        
        // Log connection parameters (important for audio streaming)
        ESP_LOGI(TAG, "Connection parameters:");
        ESP_LOGI(TAG, "  Interval: %d ms", result->conn_interval);
        ESP_LOGI(TAG, "  Latency: %d", result->conn_latency);
        ESP_LOGI(TAG, "  Timeout: %d ms", result->supervision_timeout);
        
        // TODO: Proceed with audio streaming setup
        ESP_LOGI(TAG, "Ready for BLE Audio streaming to JBL FLIP 5!");
        
    }
}

static void ble_analysis_event_handler(void* event_handler_arg, 
                                       esp_event_base_t event_base,
                                       int32_t event_id, 
                                       void* event_data)
{

    switch(event_id){
        case BLE_ANALYZER_COMPLETE:{
            ble_analysis_complete_event_data_t *device_data = (ble_analysis_complete_event_data_t*)event_data;
            ble_device_analysis_result_t *device = &device_data->device;

            ESP_LOGI(TAG, "Analyzer event called. Device bda %02x:%02x:%02x:%02x:%02x:%02x. Stored bda: %02x:%02x:%02x:%02x:%02x:%02x",
                    device->bda[0], device->bda[1], device->bda[2],
                    device->bda[3], device->bda[4], device->bda[5],
                    target_jbl_device.bda[0], target_jbl_device.bda[1], target_jbl_device.bda[2],
                    target_jbl_device.bda[3], target_jbl_device.bda[4], target_jbl_device.bda[5]);

            // Check we're dealing with the wanted JBL device here
            if(equal_bda(device->bda, target_jbl_device.bda)){

                // Check if it has BLE audio capabilities (it doesn't, but this will form a template for future options)
                if(ble_device_has_le_audio(device)){
                    // Connect via LE audio

                }else{

                    // Check if the device has Bluetooth classic capabilities capabilities
                    if(parse_ble_adv_for_dual_mode(&target_jbl_device)){
                        
                        esp_err_t ret;

                        // Deinitialize the bluetooth controller before we start it up in a different mode
                        ret = bt_controller_stack_deinit();

                        if(ret == ESP_OK){
                            // Start up the bluetooth classic
                            bt_a2dp_source_init(true);
                            
                            // Set up the event handler that will call when a2dp properly initialses
                            ret = esp_event_handler_register(A2DP_INITIALIZATION_EVENTS,     // Event base
                                                        ESP_EVENT_ANY_ID,                // Any event ID from this base
                                                        setup_data_request,              // Handler function
                                                        NULL);                           // User data (none needed)

                                                        
                            if (ret != ESP_OK) {
                                ESP_LOGE(TAG, "Failed to register A2DP initialization event handler: %s", esp_err_to_name(ret));
                            }

                        }else{
                            ESP_LOGE(TAG, "Controller deinit failed: %s", esp_err_to_name(ret));
                        }
                    }
                }
            }
        }
    }
}


/**
 * @brief Event handler for BLE discovery events
 * 
 * Handles all BLE discovery related events. This is where we process
 * discovered devices and look for JBL FLIP 5 speakers.
 * 
 * @param event_handler_arg User data passed to the event handler
 * @param event_base Event base (BLE_DISCOVERY_EVENTS in our case)
 * @param event_id Specific event ID (DEVICE_FOUND, STARTED, etc.)
 * @param event_data Event-specific data
 */
static void ble_discovery_event_handler(void* event_handler_arg, 
                                       esp_event_base_t event_base,
                                       int32_t event_id, 
                                       void* event_data)
{
    // Verify this is our event base
    if (event_base != BLE_DISCOVERY_EVENTS) {
        return;
    }

    switch (event_id) {
        case BLE_DISCOVERY_STARTED:
        {
            ble_discovery_started_event_data_t *started_data = (ble_discovery_started_event_data_t*)event_data;
            ESP_LOGI(TAG, "BLE Discovery Started - Session %lu, Duration: %lu seconds", 
                     started_data->session_id, started_data->scan_duration_sec);
            break;
        }

        case BLE_DISCOVERY_DEVICE_FOUND:
        {
            ble_device_found_event_data_t *device_data = (ble_device_found_event_data_t*)event_data;
            
            ESP_LOGI(TAG, "BLE Device Found (#%lu in session %lu)%s", 
                     device_data->discovery_count,
                     device_data->session_id,
                     device_data->is_duplicate ? " [DUPLICATE]" : "");

            // Skip if we already found our target device
            if (jbl_device_found) {
                return;
            }

            // Skip duplicates for analysis
            if (device_data->is_duplicate) {
                return;
            }

            discovered_ble_device_t *device = &device_data->device;

            // Check signal strength - need reasonable signal for audio streaming
            if (device->rssi < -80) {
                return;
            }

            // Check if this looks like a JBL device
            if (is_jbl_flip5_device(device)) {

                // Save device info
                memcpy(&target_jbl_device, device, sizeof(discovered_ble_device_t));
                jbl_device_found = true;

                // Stop discovery to save power and reduce interference
                ble_stop_device_discovery();

                // Start detailed analysis of the JBL device
                ESP_LOGI(TAG, "Starting detailed analysis of JBL device...");
                esp_err_t ret = ble_analyze_device(device->bda, device->addr_type, jbl_analysis_complete_cb);
                if (ret != ESP_OK) {
                    ESP_LOGE(TAG, "Failed to start JBL device analysis: %s", esp_err_to_name(ret));
                    jbl_device_found = false; // Reset so we can try again
                }
            } else {
                ESP_LOGD(TAG, "Device doesn't match JBL criteria");
            }
            break;
        }

        case BLE_DISCOVERY_STOPPED:
        {
            ble_discovery_stopped_event_data_t *stopped_data = (ble_discovery_stopped_event_data_t*)event_data;
            ESP_LOGI(TAG, "BLE Discovery Stopped - Session %lu", stopped_data->session_id);
            ESP_LOGI(TAG, "   Found %lu devices in %lu ms", 
                     stopped_data->total_devices_found, stopped_data->scan_duration_ms);
            
            if (stopped_data->stop_reason == ESP_ERR_TIMEOUT) {
                ESP_LOGI(TAG, "   Reason: Scan timeout");
            } else if (stopped_data->stop_reason == ESP_OK) {
                ESP_LOGI(TAG, "   Reason: Manually stopped");
            } else {
                ESP_LOGW(TAG, "   Reason: Error - %s", esp_err_to_name(stopped_data->stop_reason));
            }
            break;
        }

        case BLE_DISCOVERY_ERROR:
        {
            ble_discovery_error_event_data_t *error_data = (ble_discovery_error_event_data_t*)event_data;
            ESP_LOGE(TAG, "BLE Discovery Error: %s (%s)", 
                     error_data->error_description, 
                     esp_err_to_name(error_data->error_code));
            break;
        }

        default:
            ESP_LOGW(TAG, "Unknown BLE discovery event: %ld", event_id);
            break;
    }
}

static void bt_discovery_event_handler(void* event_handler_arg, 
                                       esp_event_base_t event_base,
                                       int32_t event_id, 
                                       void* event_data)
{
    ESP_LOGI(TAG, "Bluetooth Classic event id found: %ld", event_id);
}

/**
 * @brief Initialize JBL FLIP 5 discovery system
 * 
 * Call this after basic BLE stack initialization to set up
 * the discovery and analysis system for finding JBL speakers.
 */
esp_err_t init_jbl_flip5_discovery(void)
{
    esp_err_t ret;
    
    ESP_LOGI(TAG, "Initializing JBL FLIP 5 discovery system...");
    
    // Initialize BLE device analyzer
    ret = ble_device_analyzer_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize BLE device analyzer: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "JBL FLIP 5 discovery system ready!");
    ESP_LOGI(TAG, "Start BLE device discovery to begin searching...");
    
    return ESP_OK;
}

/**
 * @brief Initialize event handling for BLE discovery
 * 
 * Sets up the ESP event system and registers handlers for BLE discovery events.
 */
static esp_err_t init_event_handling(void)
{
    esp_err_t ret;

    // Create the default event loop if it doesn't exist
    ret = esp_event_loop_create_default();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        // ESP_ERR_INVALID_STATE means the default loop already exists, which is fine
        ESP_LOGE(TAG, "Failed to create default event loop: %s", esp_err_to_name(ret));
        return ret;
    }

    // Register our BLE discovery event handler
    // According to ESP-IDF docs: "esp_event_handler_register() registers an event handler
    // to the default event loop for a specific event"
    ret = esp_event_handler_register(BLE_DISCOVERY_EVENTS,          // Event base
                                   ESP_EVENT_ANY_ID,                // Any event ID from this base
                                   ble_discovery_event_handler,     // Handler function
                                   NULL);                           // User data (none needed)
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register BLE discovery event handler: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_event_handler_register(BT_DISCOVERY_EVENTS,           // Event base
                                   ESP_EVENT_ANY_ID,                // Any event ID from this base
                                   bt_discovery_event_handler,      // Handler function
                                   NULL);                           // User data (none needed)

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register Bluetooth Classic discovery event handler: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_event_handler_register(BLE_ANALYZER_EVENTS,           // Event base
                                   ESP_EVENT_ANY_ID,                // Any event ID from this base
                                   ble_analysis_event_handler,      // Handler function
                                   NULL);                           // User data (none needed)

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register Bluetooth Classic discovery event handler: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Event handling initialized successfully");
    return ESP_OK;
}



static void ble_config(void)
{
    esp_err_t ret;

    bt_nvs_init();

    // Initialize BLE stack first
    ret = bt_controller_stack_init(ESP_BT_MODE_BLE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BLE stack initialization failed: %s", esp_err_to_name(ret));
        return;
    }
    
    // Initialize BLE device discovery
    ret = ble_discovery_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BLE discovery initialization failed: %s", esp_err_to_name(ret));
        return;
    }
    
    // Search for the JBL FLIP 5
    ret = init_jbl_flip5_discovery();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "JBL discovery init failed: %s", esp_err_to_name(ret));
        return;
    }

    // Start BLE device discovery
    ret = ble_start_device_discovery(30); // Scan for 30 seconds
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BLE device discovery start failed: %s", esp_err_to_name(ret));
        return;
    }
}

void app_main(void)
{

    esp_err_t ret;

    // Initialize event handling first - this must come before any event posting
    ret = init_event_handling();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Event handling initialization failed: %s", esp_err_to_name(ret));
        return;
    }
    
    // If bluetooth class is initialize it
    #if CONFIG_BTDM_CONTROLLER_MODE_BR_EDR_ONLY
        // Release unused memory back to the heap if using bluetooth classic only
        // Has to be done before initialization
        ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));
        
        
        bt_a2dp_source_init();

    // Both classic and BLE active
    #elif CONFIG_BTDM_CONTROLLER_MODE_BTDM
        ble_config();

    // Only BLE active
    #elif CONFIG_BTDM_CONTROLLER_MODE_BLE_ONLY
        ble_config();

        // Release unused Classic BT memory back to the heap
        ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    // Neither versions of bluetooth are active. Clear the memory
    #else
        ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_IDLE));
    #endif

    // Initialize A2DP source
    while (1)
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
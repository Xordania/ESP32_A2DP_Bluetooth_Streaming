/*
 *   @brief handles the connection an a2dp compliant sink
 *
 *   This file handles the a2dp connection between this device and 
 *   a conencted sink device. It defines the callbacks and handles the
 *   processing required.
 */


#include "sdkconfig.h"

// Don't bother flashing this if not in either bluetooth classic mode or dual bluetooth mode
#if CONFIG_BTDM_CONTROLLER_MODE_BR_EDR_ONLY || CONFIG_BTDM_CONTROLLER_MODE_BTDM

#include "bluetooth_a2dp.h"
#include "esp_log.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_bt.h"
#include "bluetooth_common.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "bluetooth_classic.h"
#include "string.h"

static const char* TAG = "BT_A2DP";


// Queue for discoverd devices
QueueHandle_t discovered_device_queue = NULL;

/**
 * @brief Define the event base for A2DP initialization 
 */
ESP_EVENT_DEFINE_BASE(A2DP_INITIALIZATION_EVENTS);

/**
 * @brief Define the event base for A2DP connection
 */
ESP_EVENT_DEFINE_BASE(A2DP_CONNECTION_EVENTS);

// ============================================================================
// EVENT POSTING FUNCTIONS
// ============================================================================
/**
 * 
 * @brief Post a2dp initialized event
 * 
 * Posts A2DP_INITIALIZATION_COMPLETE
 */
static void post_initialization_complete_event()
{
    // Post event
    esp_err_t ret = esp_event_post(A2DP_INITIALIZATION_EVENTS,
                                  A2DP_INITIALIZATION_COMPLETE,
                                  NULL,
                                  sizeof(NULL),
                                  100 / portTICK_PERIOD_MS);  // 100ms timeout

    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to post initialization complete event: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGD(TAG, "Posted initialization complete event");
    }
}

/**
 * 
 * @brief Post device discovered event
 * 
 * Posts A2DP_CONNECTION_COMPLETE
 * 
 * @param param the structure containing information about the connection that has been made
 * 
 */
static void post_connection_complete_event(esp_a2d_cb_param_t *param)
{

    a2dp_connection_complete_event_data_t event_data;
    event_data.conn = *param;

    // Post event
    esp_err_t ret = esp_event_post(A2DP_CONNECTION_EVENTS,
                                  A2DP_CONNECTION_COMPLETE,
                                  &event_data,
                                  sizeof(event_data),
                                  100 / portTICK_PERIOD_MS);  // 100ms timeout

    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to initialization complete: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGD(TAG, "Posted initialization complete event");
    }
}

// ============================================================================
// FUNCTIONALITY
// ============================================================================

/**
 * @brief Parse audio codec configuration from A2DP negotiation
 *
 * Extracts and interprets codec information from the A2DP audio configuration
 * event. Supports SBC, MPEG Audio, AAC, and ATRAC codecs. For SBC codec,
 * performs detailed parsing of sample rate and channel configuration.
 *
 * @param param Pointer to A2DP callback parameter containing codec configuration
 * @return audio_config_t Parsed audio configuration with codec type, sample rate,
 *                        channels, codec name, and validity flag
 *
 * @note For non-SBC codecs, sample rate defaults to 44.1kHz as this is most common
 * @note If codec type is unknown, is_valid will be set to false
 */
static audio_config_t parse_audio_codec_cfg(esp_a2d_cb_param_t *param)
{
    audio_config_t config = {0};
    esp_a2d_mcc_t *mcc = &param->audio_cfg.mcc;

    config.codec_type = mcc->type;
    config.channels = 2; // Default to stereo
    config.is_valid = true;

    // Checks what kind of codec the sink is requesting
    // SBC is the default kind for speakers, though this should handle it if another
    // codec is asked for
    switch (mcc->type)
    {
    case ESP_A2D_MCT_SBC:
    {
        uint8_t *sbc = mcc->cie.sbc;
        config.codec_name = "SBC";

        // Parse sample frequency from SBC capability info element (CIE)
        // SBC CIE byte 0 format: [sampling_freq][channel_mode][block_length][subbands][allocation_method]
        //                        bits 4-7       bits 0-3     ...
        if (sbc[0] & 0x20)
            config.sample_rate = 48000;
        else if (sbc[0] & 0x10)
            config.sample_rate = 44100;
        else if (sbc[0] & 0x08)
            config.sample_rate = 32000;
        else if (sbc[0] & 0x04)
            config.sample_rate = 16000;
        else
            config.sample_rate = 44100; // Default fallback

        // Parse channel mode from bits 0-3 of byte 0
        // 0x08 = Mono, others = Stereo variants (Joint Stereo, Stereo, Dual Channel)
        if (sbc[0] & 0x08)
            config.channels = 1; // Mono
        else
            config.channels = 2; // Stereo modes
        break;
    }

    case ESP_A2D_MCT_M12:
        config.codec_name = "MPEG Audio";
        config.sample_rate = 44100; // Common default for MP3
        break;

    case ESP_A2D_MCT_M24:
        config.codec_name = "AAC";
        config.sample_rate = 44100; // Common default for AAC
        break;

    case ESP_A2D_MCT_ATRAC:
        config.codec_name = "ATRAC";
        config.sample_rate = 44100;
        break;

    default:
        config.codec_name = "Unknown";
        config.sample_rate = 44100;
        config.is_valid = false;
        break;
    }

    return config;
}

/**
 * @brief A2DP event callback handler
 *
 * Handles all A2DP (Advanced Audio Distribution Profile) events during audio
 * streaming operations. Processes connection state changes, audio streaming
 * state changes, codec configuration events, and media control acknowledgments.
 *
 * Key events handled:
 * - Connection establishment/termination
 * - Audio streaming start/stop
 * - Codec negotiation and configuration
 * - Media control command acknowledgments
 *
 * @param event Type of A2DP event that occurred
 * @param param Event-specific parameter data containing relevant information
 *
 * @note This callback executes in Bluetooth stack context - keep processing minimal
 * @note Connection and audio state changes are logged for debugging
 * @note Codec configuration is parsed and logged with detailed information
 */
static void bt_a2d_cb(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param)
{
    switch (event)
    {
    case ESP_A2D_CONNECTION_STATE_EVT:
        // State of on the connection with a sink
        if (param->conn_stat.state == ESP_A2D_CONNECTION_STATE_CONNECTED)
        {
            ESP_LOGI(TAG, "A2DP connected to: %02x:%02x:%02x:%02x:%02x:%02x",
                         param->conn_stat.remote_bda[0], param->conn_stat.remote_bda[1],
                         param->conn_stat.remote_bda[2], param->conn_stat.remote_bda[3], 
                         param->conn_stat.remote_bda[4], param->conn_stat.remote_bda[5]);
            
            post_connection_complete_event(param);
        }
        else if (param->conn_stat.state == ESP_A2D_CONNECTION_STATE_CONNECTING)
        {
            ESP_LOGI(TAG, "A2DP connecting");
        }
        else if (param->conn_stat.state == ESP_A2D_CONNECTION_STATE_DISCONNECTING)
        {
            ESP_LOGI(TAG, "A2DP disconnecting");
        }
        else if (param->conn_stat.state == ESP_A2D_CONNECTION_STATE_DISCONNECTED)
        {
            ESP_LOGI(TAG, "A2DP disconnected");

            // Stop audio stream
        }
        break;
    case ESP_A2D_AUDIO_STATE_EVT:
        // Change in audio streaming state (i.e. Puase)
        if (param->audio_stat.state == ESP_A2D_AUDIO_STATE_STARTED)
        {
            ESP_LOGI(TAG, "Audio streaming started");
        }
        else if (param->audio_stat.state == ESP_A2D_AUDIO_STATE_STOPPED)
        {
            ESP_LOGI(TAG, "Audio stream stopped");
        }
        else if (param->audio_stat.state == ESP_A2D_AUDIO_STATE_REMOTE_SUSPEND)
        {
            ESP_LOGI(TAG, "Audio streaming remote suspended");
        }

        break;

    case ESP_A2D_AUDIO_CFG_EVT:
        // Configuration of the bluetooth link i.e. sample rate, channel number, etc. etc.
        audio_config_t config = parse_audio_codec_cfg(param);
        if (config.is_valid)
        {
            ESP_LOGI(TAG, "Audio codec configured:");
            ESP_LOGI(TAG, "  Codec: %s", config.codec_name);
            ESP_LOGI(TAG, "  Sample rate: %lu Hz", config.sample_rate);
            ESP_LOGI(TAG, "  Channels: %d", config.channels);

            // TODO: Store config globally for audio pipeline use
            // global_audio_config = config;
        }
        else
        {
            ESP_LOGW(TAG, "Unsupported codec negotiated: %s", config.codec_name);
        }
        break;

    case ESP_A2D_MEDIA_CTRL_ACK_EVT:
        // Response to media control commands like play and pasue
        ESP_LOGI(TAG, "Media control acknowledged");
        break;

    case ESP_A2D_PROF_STATE_EVT:
        if(param->a2d_prof_stat.init_state == ESP_A2D_INIT_SUCCESS){
            ESP_LOGI(TAG, "A2DP initialization successful");

            post_initialization_complete_event();
        }else{
            ESP_LOGI(TAG, "A2DP deinitialization successful");
        }
        break;
    default:
        ESP_LOGE(TAG, "Unhandled A2DP event %d", event);
        break;
    }
}

/**
 * @brief Log current A2DP configuration settings
 *
 * Displays the current configuration values that were set via Kconfig.
 * Useful for debugging and verifying configuration during initialization.
 */
static void log_a2dp_config(){
    // Log configuration values
    ESP_LOGI(TAG, "A2DP Source Configuration:");
    ESP_LOGI(TAG, "Device queue size: %d", CONFIG_A2DP_DISCOVERED_DEVICE_QUEUE_SIZE);
    ESP_LOGI(TAG, "Task stack size: %d bytes", CONFIG_A2DP_DEVICE_PROCESSING_TASK_STACK_SIZE);
    ESP_LOGI(TAG, "Task priority: %d", CONFIG_A2DP_DEVICE_PROCESSING_TASK_PRIORITY);
    ESP_LOGI(TAG, "Max device properties: %d", CONFIG_A2DP_MAX_DEVICE_PROPERTIES);
}
/**
 * @brief Background task to process discovered Bluetooth devices
 *
 * A FreeRTOS task that processes discovered Bluetooth devices from the
 * discovery queue. It performs detailed analysis of device properties
 * including device name, Class of Device, and RSSI. When an audio/video device
 * is found (CoD major class 0x04), it attempts an A2DP connection and stops
 * device discovery.
 *
 * The task is designed to handle heavy processing outside of Bluetooth stack
 * callbacks to maintain stack responsiveness during device discovery.
 *
 * @param pvParameters FreeRTOS task parameter (unused in this implementation)
 *
 * @note This task runs indefinitely until an audio device is found and connected
 * @note Task will block on queue receive when no devices are pending processing
 * @note Device discovery is cancelled when first audio device connection is attempted
 */

static void a2dp_device_processing_task(void *pvParameters)
{
    discovered_device_t device;

    while (1)
    {
       if (xQueueReceive(discovered_device_queue, &device, portMAX_DELAY))
        {
            // Check if we've already processed this device
            if (is_bt_device_already_discovered(device.bda))
            {
                ESP_LOGD(TAG, "Device %02x:%02x:%02x:%02x:%02x:%02x already processed, skipping",
                         device.bda[0], device.bda[1], device.bda[2],
                         device.bda[3], device.bda[4], device.bda[5]);
                continue;
            }

            ESP_LOGI(TAG, "Processing new device: %02x:%02x:%02x:%02x:%02x:%02x",
                     device.bda[0], device.bda[1], device.bda[2],
                     device.bda[3], device.bda[4], device.bda[5]);

            char device_name[64] = "Unknown";

            // Process device properties
            for (int i = 0; i < device.num_prop; i++)
            {
                esp_bt_gap_dev_prop_t *prop = &device.props[i];

                switch (prop->type)
                {
                case ESP_BT_GAP_DEV_PROP_BDNAME:
                    strncpy(device_name, (char *)prop->val, sizeof(device_name) - 1);
                    device_name[sizeof(device_name) - 1] = '\0';
                    ESP_LOGI(TAG, "Device name: %s", device_name);                    break;

                case ESP_BT_GAP_DEV_PROP_COD:
                {
                    uint32_t cod = *(uint32_t *)prop->val;

                    // Extract classes from received information
                    // CoD format: [Service Classes][Major Device Class][Minor Device Class][Format]
                    //             bits 13-23        bits 8-12          bits 2-7         bits 0-1
                    uint32_t major_class = (cod & 0x1F00) >> 8;
                    uint32_t minor_class = (cod & 0x00FC) >> 2;      
                    uint32_t service_class = (cod & 0xFFE000) >> 13; 
                    
                    ESP_LOGI(TAG, "Class of Device: 0x%06lx", cod);
                    ESP_LOGI(TAG, "  Major class: 0x%02lx (%s)", major_class, get_bt_device_class_name(major_class));
                    ESP_LOGI(TAG, "  Minor class: 0x%02lx", minor_class);    
                    ESP_LOGI(TAG, "  Service class: 0x%03lx", service_class);

                    // Extract major device class from bits 8-12 of Class of Device
                    // CoD format: [Service Classes][Major Device Class][Minor Device Class][Format]
                    //             bits 13-23        bits 8-12          bits 2-7         bits 0-1
                    if (major_class == 0x04)
                    {
                        ESP_LOGI(TAG, "Found audio/video device!");
                    }
                    break;
                }

                case ESP_BT_GAP_DEV_PROP_RSSI:
                    ESP_LOGI(TAG, "RSSI: %d dBm", *(int8_t *)prop->val);
                    break;

                default:
                    ESP_LOGI(TAG, "Property type %d, length %d", prop->type, prop->len);
                    break;
                }
            }

            // Add to cache for future reference (all devices, not just audio)
            add_device_to_bt_cache(&device);

        }
    }
}



/**
 * @brief Initialize Bluetooth A2DP source functionality
 *
 * Performs complete initialization of Bluetooth Classic stack and A2DP source
 * profile. This includes controller initialization, Bluedroid stack setup,
 * callback registration, and device discovery initiation.
 *
 * A2DP Initialization sequence:
 * 1. Create device discovery queue
 * 2. Configure task core affinity
 * 3. Create device processing background task
 * 4. Register A2DP event callback
 * 5. Initialize A2DP source profile
 * 6. Initialize GAP functionality (device discovery, naming, scan mode)
 *
 * Configuration values are read from Kconfig settings for queue size,
 * task parameters, and device limits.
 *
 * @note This function must be called after basic system initialization
 * @note Requires sufficient heap memory for Bluetooth stack (~150KB)
 * @note Will start device discovery automatically upon successful initialization
 * @note Function will return early if any initialization step fails
 */
void bt_a2dp_source_init(bool dual_bluetooth)
{
    esp_err_t ret;

    // 1. Create device discovery queue
    discovered_device_queue = xQueueCreate(CONFIG_A2DP_DISCOVERED_DEVICE_QUEUE_SIZE, 
                                          sizeof(discovered_device_t));
    if (discovered_device_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create discovered device queue");
        return;
    }

    log_a2dp_config();

    // Initialize NVS
    ret = bt_nvs_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "NVS initialization failed");
        return;
    }

    if(dual_bluetooth){
        bt_controller_stack_init(ESP_BT_MODE_CLASSIC_BT);
    }else{
        bt_controller_stack_init(ESP_BT_MODE_BTDM);
    }

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Bluetooth stack initialization failed");
        return;
    }


    // 2. Configure task core affinity
    // Handle Kconfig core selection: -1 means "any core"
    BaseType_t core_id = CONFIG_A2DP_DEVICE_PROCESSING_CORE_ID;
    if (core_id == -1) {
        core_id = tskNO_AFFINITY;
    }

    // 3. Create device processing background task
    BaseType_t task_result = xTaskCreatePinnedToCore(
        a2dp_device_processing_task,                        // Task function
        "bt_device_proc",                                   // Task name
        CONFIG_A2DP_DEVICE_PROCESSING_TASK_STACK_SIZE,      // Stack size
        NULL,                                               // Parameters
        CONFIG_A2DP_DEVICE_PROCESSING_TASK_PRIORITY,        // Priority
        NULL,                                               // Task handle
        core_id                                             // Core affinity
    );

    if (task_result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create device processing task");
        return;
    }

    // 4. Register A2DP event callback
    ret = esp_a2d_register_callback(bt_a2d_cb);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "A2DP callback register failed: %s", esp_err_to_name(ret));
        return;
    }

    // 5. Initialize A2DP source profile
    ret = esp_a2d_source_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "A2DP source initialize failed: %s", esp_err_to_name(ret));
        return;
    }

    // 6. Initialize GAP functionality (device discovery, naming, scan mode)
    ret = bt_gap_init(&discovered_device_queue);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GAP initialization failed: %s", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(TAG, "A2DP source initialized successfully");
}
#endif
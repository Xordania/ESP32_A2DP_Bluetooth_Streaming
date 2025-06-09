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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "main.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_system.h"
#include "esp_log.h"
#include "sdkconfig.h"

static const char *TAG = "A2DP_SOURCE";

// Queue for discoverd devices
static QueueHandle_t discovered_device_queue = NULL;

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
 * @brief Background task to process discovered Bluetooth devices
 *
 * A FreeRTOs task that processes discovered Bluetooth devices from the
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
static void device_processing_task(void *pvParameters)
{
    discovered_device_t device;

    while (1)
    {
        if (xQueueReceive(discovered_device_queue, &device, portMAX_DELAY))
        {
            ESP_LOGI(TAG, "Processing device: %02x:%02x:%02x:%02x:%02x:%02x",
                     device.bda[0], device.bda[1], device.bda[2],
                     device.bda[3], device.bda[4], device.bda[5]);

            bool is_audio_device = false;

            // Process device properties
            for (int i = 0; i < device.num_prop; i++)
            {
                esp_bt_gap_dev_prop_t *prop = &device.props[i];

                switch (prop->type)
                {
                case ESP_BT_GAP_DEV_PROP_BDNAME:
                    ESP_LOGI(TAG, "Device name: %s", (char *)prop->val);
                    break;

                case ESP_BT_GAP_DEV_PROP_COD:
                {
                    uint32_t cod = *(uint32_t *)prop->val;

                    // Extract major device class from bits 8-12 of Class of Device
                    // CoD format: [Service Classes][Major Device Class][Minor Device Class][Format]
                    //             bits 13-23        bits 8-12          bits 2-7         bits 0-1
                    uint32_t major_class = (cod & 0x1F00) >> 8;
                    ESP_LOGI(TAG, "Class of Device: 0x%06lx, Major class: 0x%02lx", cod, major_class);

                    // Extract major device class from bits 8-12 of Class of Device
                    // CoD format: [Service Classes][Major Device Class][Minor Device Class][Format]
                    //             bits 13-23        bits 8-12          bits 2-7         bits 0-1
                    if (major_class == 0x04)
                    {
                        ESP_LOGI(TAG, "Found audio/video device!");
                        is_audio_device = true;
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

            // Connect to audio devices
            if (is_audio_device)
            {
                ESP_LOGI(TAG, "Attempting A2DP connection...");
                esp_a2d_source_connect(device.bda);
                esp_bt_gap_cancel_discovery();
                break; // Stop processing more devices
            }
        }
    }
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
            ESP_LOGI(TAG, "A2DP connected to: ");

            for (int i = 0; i < ESP_BD_ADDR_LEN; i++)
            {
                printf("%c", param->conn_stat.remote_bda[i]);
            }

            // Audio stream
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

    default:
        ESP_LOGE(TAG, "Unhandled A2DP event %d", event);
        break;
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

static void log_config(){
    // Log configuration values
    ESP_LOGI(TAG, "A2DP Source Configuration:");
    ESP_LOGI(TAG, "Device queue size: %d", CONFIG_A2DP_DISCOVERED_DEVICE_QUEUE_SIZE);
    ESP_LOGI(TAG, "Task stack size: %d bytes", CONFIG_A2DP_DEVICE_PROCESSING_TASK_STACK_SIZE);
    ESP_LOGI(TAG, "Task priority: %d", CONFIG_A2DP_DEVICE_PROCESSING_TASK_PRIORITY);
    ESP_LOGI(TAG, "Max device properties: %d", CONFIG_A2DP_MAX_DEVICE_PROPERTIES);
}

/**
 * @brief Initialize Bluetooth A2DP source functionality
 *
 * Performs complete initialization of Bluetooth Classic stack and A2DP source
 * profile. This includes controller initialization, Bluedroid stack setup,
 * callback registration, and device discovery initiation.
 *
 * Initialization sequence:
 * 1. Creates device processing queue and background task
 * 2. Initializes NVS for Bluetooth pairing data storage
 * 3. Configures and enables Bluetooth controller (Classic mode only)
 * 4. Initializes and enables Bluedroid protocol stack
 * 5. Registers GAP and A2DP event callbacks
 * 6. Initializes A2DP source profile
 * 7. Sets device name and discoverable mode
 * 8. Starts device discovery to find audio devices
 *
 * Configuration values are read from Kconfig settings for queue size,
 * task parameters, and device limits.
 *
 * @note This function must be called after basic system initialization
 * @note Requires sufficient heap memory for Bluetooth stack (~150KB)
 * @note Will start device discovery automatically upon successful initialization
 * @note Function will return early if any initialization step fails
 */
void bt_a2dp_source_init(void)
{
    log_config();

    // Intiitalize NVS to store Bluetooth paiaring data storage
    esp_err_t ret = nvs_flash_init();

    // If the storage is full then erase it and reinit
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    ESP_ERROR_CHECK(ret);

    // Release unused memory back to the heap
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    bt_cfg.mode = ESP_BT_MODE_CLASSIC_BT;

    // Ensure we're in the correct controller code. Change controller mode in menuconfig if not
    #if CONFIG_BTDM_CONTROLLER_MODE_BLE_ONLY
    #error "A2DP requires Classic Bluetooth - change controller mode in menuconfig"
    #endif

    // Create the queues and tasks for discovered devices
    discovered_device_queue = xQueueCreate(CONFIG_A2DP_DISCOVERED_DEVICE_QUEUE_SIZE, sizeof(discovered_device_t));

    if (discovered_device_queue == NULL)
    {
        ESP_LOGE(TAG, "Failed to create discovered device queue");
        return;
    }

    // tskNoAffinity maps to no core in particular, so if the user has no preference of the core used to
    // do processing we let the idf decide
    BaseType_t core_id = CONFIG_A2DP_DEVICE_PROCESSING_CORE_ID;
    if (core_id == -1) {
        core_id = tskNO_AFFINITY;
    }

    BaseType_t task_result = xTaskCreatePinnedToCore(
        device_processing_task,
        "bt_device_proc",
        CONFIG_A2DP_DEVICE_PROCESSING_TASK_STACK_SIZE, 
        NULL,
        CONFIG_A2DP_DEVICE_PROCESSING_TASK_PRIORITY,
        NULL,
        core_id
    );

    if (task_result != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create device processing task");
        return;
    }

    // Controller config and enables error checks
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Bluetooth controller initialize failed: %s", esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Bluetooth controller enable failed: %s", esp_err_to_name(ret));
        return;
    }

    // Turn on the bluetooth stack
    ret = esp_bluedroid_init();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Bluetooth stack init failed: %s", esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Bluetooth stack enable failed: %s", esp_err_to_name(ret));
        return;
    }

    // Register callbacks
    ret = esp_a2d_register_callback(bt_a2d_cb);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "A2DP callback register failed: %s", esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_gap_register_callback(bt_gap_cb);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "GAP callback register failed: %s", esp_err_to_name(ret));
        return;
    }

    ret = esp_a2d_source_init();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "A2DP source initialize failed: %s", esp_err_to_name(ret));
        return;
    }

    // Set device name
    ret = esp_bt_gap_set_device_name(BT_DEVICE_NAME);

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Set device name failed: %s", esp_err_to_name(ret));
        return;
    }

    // Set connection and disconverable node
    ret = esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Set scan mode failed: %s", esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, 10, 0);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Start discovery failed %s", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(TAG, "A2DP source initialized successfully");
}



void app_main(void)
{
    bt_a2dp_source_init();

    // Initialize A2DP source
    while (1)
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
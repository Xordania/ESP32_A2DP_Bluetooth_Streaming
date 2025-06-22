#include "sdkconfig.h"

// Don't bother flashing this if not in either bluetooth classic mode or dual bluetooth mode
#if CONFIG_BTDM_CONTROLLER_MODE_BR_EDR_ONLY || CONFIG_BTDM_CONTROLLER_MODE_BTDM

#include "esp_a2dp_api.h"
#include "esp_event.h" 

// Structure to hold parsed audio configuration
typedef struct {
    esp_a2d_mct_t codec_type;
    uint32_t sample_rate;
    uint8_t channels;
    const char* codec_name;
    bool is_valid;
} audio_config_t;

/**
 * @brief A2DP initialization Event IDs
 * 
 * These are the specific events that the A2DP initialization system can post.
 */
typedef enum {
    A2DP_INITIALIZATION_COMPLETE,               // A2DP initialized
} a2dp_initialization_event_id_t;


/**
 * @brief A2DP conenction Event IDs
 * 
 * These are the specific events that the A2DP connection system can post.
 */
typedef enum {
    A2DP_CONNECTION_COMPLETE,                   // A2DP connection successful
} a2dp_connection_event_id_t;

/**
 * @brief Event data for A2DP_CONNECTION_COMPLETE
 * 
 * Contains all information about a newly discovered device.
 */
typedef struct {
    esp_a2d_cb_param_t conn;
} a2dp_connection_complete_event_data_t;


/**
 * @brief A2DP Initialization Event Base
 * 
 * This creates a unique identifier for our A2DP initialization events.
 */
ESP_EVENT_DECLARE_BASE(A2DP_INITIALIZATION_EVENTS);

/**
 * @brief A2DP Connection Event Base
 * 
 * This creates a unique identifier for our A2DP connection events.
 */
ESP_EVENT_DECLARE_BASE(A2DP_CONNECTION_EVENTS);


void bt_a2dp_source_init(bool dual_bluetooth);

#endif

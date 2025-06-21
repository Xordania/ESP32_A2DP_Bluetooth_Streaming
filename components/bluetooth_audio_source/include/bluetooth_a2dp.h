#include "sdkconfig.h"

// Don't bother flashing this if not in either bluetooth classic mode or dual bluetooth mode
#if CONFIG_BTDM_CONTROLLER_MODE_BR_EDR_ONLY || CONFIG_BTDM_CONTROLLER_MODE_BTDM

#include "esp_a2dp_api.h"

// Structure to hold parsed audio configuration
typedef struct {
    esp_a2d_mct_t codec_type;
    uint32_t sample_rate;
    uint8_t channels;
    const char* codec_name;
    bool is_valid;
} audio_config_t;

void bt_a2dp_source_init(bool dual_bluetooth);

#endif

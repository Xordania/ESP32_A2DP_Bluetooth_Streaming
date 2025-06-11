#ifndef BLUETOOTH_CLASSIC_H
#define BLUETOOTH_CLASSIC_H


#include "esp_bt.h"
#include "esp_a2dp_api.h"
#include "esp_gap_bt_api.h"


#define BT_DEVICE_NAME "ESP32"

// Structure to hold discovered device info
typedef struct {
    esp_bd_addr_t bda;
    int num_prop;
    esp_bt_gap_dev_prop_t props[CONFIG_A2DP_MAX_DEVICE_PROPERTIES];
} discovered_device_t;

// Structure to hold parsed audio configuration
typedef struct {
    esp_a2d_mct_t codec_type;
    uint32_t sample_rate;
    uint8_t channels;
    const char* codec_name;
    bool is_valid;
} audio_config_t;

// Only define functions if bluetooth classic is active
#if CONFIG_BTDM_CONTROLLER_MODE_BR_EDR_ONLY
    // Function declarations
    void bt_a2dp_source_init(void);
#endif


#endif // BLUETOOTH_CLASSIC_H
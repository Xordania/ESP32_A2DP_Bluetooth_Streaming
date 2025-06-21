#include "esp_err.h"
#include "esp_bt.h"
#include "esp_bt_defs.h"

esp_err_t bt_nvs_init(void);
esp_err_t bt_controller_stack_init(esp_bt_mode_t mode);
bool equal_bda(esp_bd_addr_t a, esp_bd_addr_t b);
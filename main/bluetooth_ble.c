#include "sdkconfig.h"

#if CONFIG_BTDM_CONTROLLER_MODE_BLE_ONLY || CONFIG_BTDM_CONTROLLER_MODE_BTDM

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "bluetooth_ble.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_log.h"

static const char *TAG = "BLE_DISCOVERY";

// Queue for discovered BLE devices
static QueueHandle_t discovered_ble_device_queue = NULL;

// Cache for discovered BLE devices to prevent duplicate processing
static discovered_ble_device_t discovered_ble_devices_cache[CONFIG_BLE_MAX_DISCOVERED_DEVICES_CACHE];
static int discovered_ble_devices_count = 0;
static int discovered_ble_devices_fifo_pointer = 0;

// Scanning state
static bool is_scanning = false;

#endif // CONFIG_BTDM_CONTROLLER_MODE_BLE_ONLY || CONFIG_BTDM_CONTROLLER_MODE_BTDM
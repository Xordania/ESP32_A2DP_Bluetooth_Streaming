idf_component_register(SRCS "bluetooth_device_analyzer.c" "bluetooth_uuid_defintions.c" "bluetooth_ble.c" "bluetooth_common.c" "bluetooth_classic.c"
                    REQUIRES freertos nvs_flash bt
                    INCLUDE_DIRS "" "include")

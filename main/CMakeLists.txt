idf_component_register(SRCS "bluetooth_common.c" "main.c" "bluetooth_classic.c"
                    REQUIRES freertos nvs_flash bt
                    INCLUDE_DIRS "" "include")

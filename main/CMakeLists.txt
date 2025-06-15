idf_component_register(SRCS "main.c"
                    REQUIRES freertos 
                    nvs_flash 
                    bt 
                    bluetooth_audio_source 
                    esp_event
                    INCLUDE_DIRS "" "include")

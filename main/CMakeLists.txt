idf_component_register(SRCS "led_indicator.c" "ble_gattc.c" "main.c"
                           "wifi_manager.c"
                           "ntp_time.c"
                           "gpio_button.c"
                    PRIV_REQUIRES nvs_flash
                                  esp_wifi
                                  esp_netif
                                  esp_event
                                  driver
                                  lwip
                                  bt
                    INCLUDE_DIRS ".")

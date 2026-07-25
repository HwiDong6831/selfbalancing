idf_component_register(SRCS "main.c" "telemetry.c"
                       PRIV_REQUIRES spi_flash driver esp_timer mpu6050 foc encoder balance
                                     esp_wifi esp_netif esp_event nvs_flash
                       INCLUDE_DIRS "")

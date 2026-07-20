idf_component_register(SRCS "main.c"
                       PRIV_REQUIRES spi_flash driver esp_timer mpu6050 foc encoder balance
                       INCLUDE_DIRS "")

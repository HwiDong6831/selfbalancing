idf_component_register(SRCS "main.c"
                       PRIV_REQUIRES spi_flash esp_driver_i2c esp_timer mpu6050 foc encoder balance
                       INCLUDE_DIRS "")

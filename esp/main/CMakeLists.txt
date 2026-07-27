idf_component_register(SRCS "main.c"
                       PRIV_REQUIRES driver esp_timer foc encoder mpu6050 balance
                       INCLUDE_DIRS "")

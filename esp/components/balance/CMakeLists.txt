# balance 컴포넌트
idf_component_register(SRCS "balance.c"
                       INCLUDE_DIRS "."
                       REQUIRES esp_driver_i2c)

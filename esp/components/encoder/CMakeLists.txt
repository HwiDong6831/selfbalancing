# encoder 컴포넌트 — MT6701 자기 인코더 (I2C)
idf_component_register(SRCS "encoder.c"
                       INCLUDE_DIRS "."
                       REQUIRES esp_driver_i2c)

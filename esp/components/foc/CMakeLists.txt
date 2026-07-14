# foc 컴포넌트 — BLDC 전압모드 FOC (LEDC 3-PWM + EN GPIO)
idf_component_register(SRCS "foc.c"
                       INCLUDE_DIRS "."
                       REQUIRES esp_driver_ledc esp_driver_gpio)

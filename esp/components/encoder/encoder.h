#pragma once

#include "driver/i2c.h"
#include "esp_err.h"
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

// 레거시 I2C 포트 저장 (MT6701 0x06 은 포트+주소로 직접 접근)
esp_err_t encoder_init(i2c_port_t port);

// 14bit raw 각도 읽기 (0 ~ 16383)
esp_err_t encoder_read_raw(uint16_t *raw);

// 각도를 라디안으로 (0 ~ 2π)
esp_err_t encoder_read_angle(float *angle_rad);

#ifdef __cplusplus
}
#endif

/*
 * encoder.h — MT6701 자기 인코더 드라이버 (I2C)
 *
 * 배선: I2C 주소 0x06, 메인 버스 직결 (mux 안 거침), SDA16/SCL17.
 *       IIC 솔더 점퍼 납땜 필수 (안 하면 SSI 모드로 버스 hang).
 * 스펙: 14bit 절대 각도, 레지스터 0x03(상위)/0x04(하위).
 */
#pragma once

#include "driver/i2c_master.h"
#include "esp_err.h"
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

// 이미 만들어진 I2C 버스에 MT6701(0x06) 디바이스 등록
esp_err_t encoder_init(i2c_master_bus_handle_t bus);

// 14bit raw 각도 읽기 (0 ~ 16383)
esp_err_t encoder_read_raw(uint16_t *raw);

// 각도를 라디안으로 (0 ~ 2π)
esp_err_t encoder_read_angle(float *angle_rad);

#ifdef __cplusplus
}
#endif

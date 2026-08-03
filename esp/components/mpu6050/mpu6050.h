#pragma once

#include "driver/i2c.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// mux 채널을 순회하며 MPU 를 깨우고 WHO_AM_I 를 확인한다.
esp_err_t mpu6050_init(i2c_port_t port, const int *channels, int num_channels);

// 가속도 + 자이로 raw 값 동시 읽기 (14바이트 버스트). 실패 시 출력 미변경.
esp_err_t mpu6050_read_accel_gyro(int channel,
                                  int16_t *ax, int16_t *ay, int16_t *az,
                                  int16_t *gx, int16_t *gy, int16_t *gz);

#ifdef __cplusplus
}
#endif

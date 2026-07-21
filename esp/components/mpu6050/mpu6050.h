#pragma once

#include "driver/i2c.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * I2C 버스에 mux/MPU 디바이스를 등록하고, channels를 순회하며 MPU wake + WHO_AM_I 확인함.
 *
 * bus          : I2C 버스 핸들
 * channels     : 사용할 mux 채널 번호 배열
 * num_channels : channels 배열 길이 (함수 안에선 배열 길이를 알 수 없어 넘겨줘야 함)
 *
 * 반환: ESP_OK. 등록 실패 시 ESP_ERROR_CHECK 로 abort.
 */
esp_err_t mpu6050_init(i2c_port_t port, const int *channels, int num_channels);

/*
 * 지정한 mux 채널의 MPU 가속도 raw 값(16bit) 읽기.
 *
 * channel   : 읽을 mux 채널 번호
 * ax, ay, az: 결과 저장 포인터 (raw int16, 스케일 변환 전)
 *
 * 반환: 성공 ESP_OK, 실패 시 I2C 에러 코드.
 */
esp_err_t mpu6050_read_accel(int channel, int16_t *ax, int16_t *ay, int16_t *az);

/*
 * 가속도 + 자이로 3축 동시 읽기 (14바이트 버스트).
 *
 * ax,ay,az : 가속도 raw (기본 ±2g = 16384 LSB/g)
 * gx,gy,gz : 자이로 raw (기본 ±250°/s = 131 LSB/(°/s))
 *
 * 반환: 성공 ESP_OK, 실패 시 I2C 에러 코드 (실패 시 출력 미변경 → 이전값 유지).
 */
esp_err_t mpu6050_read_accel_gyro(int channel,
                                  int16_t *ax, int16_t *ay, int16_t *az,
                                  int16_t *gx, int16_t *gy, int16_t *gz);

#ifdef __cplusplus
}
#endif

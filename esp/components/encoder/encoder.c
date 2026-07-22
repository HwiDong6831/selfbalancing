#include "encoder.h"
#include "freertos/FreeRTOS.h"  // pdMS_TO_TICKS

#define MT6701_ADDR     0x06
#define REG_ANGLE_H     0x03    // 각도 상위 바이트
#define REG_ANGLE_L     0x04    // 각도 하위 바이트

#define I2C_TIMEOUT_MS  10      // stuck 시 빨리 ESP_ERR_TIMEOUT 반환

// 레거시 i2c 드라이버. 버스 stuck 시 CPU 행 없이 타임아웃 반환.
static i2c_port_t s_port;

esp_err_t encoder_init(i2c_port_t port)
{
    s_port = port;
    return ESP_OK;
}

esp_err_t encoder_read_raw(uint16_t *raw)
{
    // 0x03(H), 0x04(L)를 한 트랜잭션(2바이트 버스트, 주소 자동증가)으로 읽음.
    // 별도 2회 읽으면 회전 중 두 값이 다른 시점이 되어 각도가 깨짐.
    uint8_t buf[2];
    uint8_t reg = (uint8_t)REG_ANGLE_H;
    esp_err_t err = i2c_master_write_read_device(s_port, MT6701_ADDR, &reg, 1, buf, 2,
                                                 pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    if (err != ESP_OK) return err;      // 실패 전파 → 호출부가 이전 값 유지

    *raw = (buf[0] << 6) | (buf[1] >> 2);
    return ESP_OK;
}

esp_err_t encoder_read_angle(float *angle_rad)
{
    uint16_t raw = 0;
    esp_err_t err = encoder_read_raw(&raw);
    if (err != ESP_OK) return err;      // 실패 전파 (예전엔 항상 ESP_OK 였음)

    *angle_rad = 360.0f - (float)raw * ((2.0f * M_PI) / 16384); // 모터 전기각 방향이 반대여서 360에서 빼줌
    return ESP_OK;
}

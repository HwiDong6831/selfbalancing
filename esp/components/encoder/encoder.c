#include "encoder.h"

#define MT6701_ADDR     0x06
#define REG_ANGLE_H     0x03    // 각도 상위 바이트
#define REG_ANGLE_L     0x04    // 각도 하위 바이트

static i2c_master_dev_handle_t encoder_dev;

esp_err_t encoder_init(i2c_master_bus_handle_t bus)
{
    i2c_device_config_t encoder_cfg = {
        .device_address = MT6701_ADDR,
        .scl_speed_hz = 400000,
        .dev_addr_length = I2C_ADDR_BIT_LEN_7
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &encoder_cfg, &encoder_dev));
    return ESP_OK;
}

esp_err_t encoder_read_raw(uint16_t *raw)
{
    uint8_t high, low;
    uint8_t reg_h = (uint8_t)REG_ANGLE_H;
    uint8_t reg_l = (uint8_t)REG_ANGLE_L;
    i2c_master_transmit_receive(encoder_dev, &reg_h, 1, &high, 1, 100);
    i2c_master_transmit_receive(encoder_dev, &reg_l, 1, &low, 1, 100);  // 이러면 같은 순간의 high와 low가 아니게 되는 것 아닌가?

    *raw = (high << 6) | (low >> 2);
    return ESP_OK;
}

esp_err_t encoder_read_angle(float *angle_rad)
{
    uint16_t raw = 0;
    encoder_read_raw(&raw);
    *angle_rad = 360.0f - (float)raw * ((2.0f * M_PI) / 16384); // 모터 전기각 방향이 반대여서 360에서 빼줌

    return ESP_OK;
}

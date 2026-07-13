#include "mpu6050.h"
#include "esp_log.h"

#define MUX_ADDR        0x70
#define MPU_ADDR        0x68

#define REG_PWR_MGMT_1  0x6B
#define REG_WHOAMI      0x75
#define REG_ACCEL_XOUT  0x3B

#define I2C_TIMEOUT_MS  100

static i2c_master_dev_handle_t s_mux_dev;
static i2c_master_dev_handle_t s_mpu_dev;

static esp_err_t mux_select(int channel)
{
    uint8_t mask = (uint8_t)(1 << channel);
    return i2c_master_transmit(s_mux_dev, &mask, 1, I2C_TIMEOUT_MS);
}

esp_err_t mpu6050_init(i2c_master_bus_handle_t bus, const int *channels, int num_channels)
{
    esp_err_t err;

    // mux 등록
    i2c_device_config_t mux_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MUX_ADDR,
        .scl_speed_hz = 400000,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &mux_cfg, &s_mux_dev));

    // MPU 등록
    i2c_device_config_t mpu_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MPU_ADDR,
        .scl_speed_hz = 400000,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &mpu_cfg, &s_mpu_dev));

    // 채널마다 MPU wake + 센서 확인
    for (int i = 0; i < num_channels; i++) {
        int ch = channels[i];
        err = mux_select(ch);
        if (err != ESP_OK) {
            ESP_LOGE("MPU", "채널 %d : mux 선택 실패", ch);
            continue;
        }

        uint8_t wake[2] = {REG_PWR_MGMT_1, 0x00};
        err = i2c_master_transmit(s_mpu_dev, wake, 2, I2C_TIMEOUT_MS);
        if (err != ESP_OK) {
            ESP_LOGE("MPU", "채널 %d : wake 실패", ch);
            continue;
        }

        // 센서 확인
        uint8_t who = 0;
        uint8_t reg = REG_WHOAMI;
        err = i2c_master_transmit_receive(s_mpu_dev, &reg, 1, &who, 1, I2C_TIMEOUT_MS);
        if (err == ESP_OK) {
            ESP_LOGI("MPU", "채널 %d : WHO_AM_I = 0x%02x", ch, who);
        } else {
            ESP_LOGE("MPU", "채널 %d : 읽기 실패", ch);
        }
    }

    return ESP_OK;
}

esp_err_t mpu6050_read_accel(int channel, int16_t *ax, int16_t *ay, int16_t *az)
{
    esp_err_t err = mux_select(channel);
    if (err != ESP_OK) return err;

    // ACCEL_XOUT_H 부터 6바이트(x,y,z 각각 2바이트)
    uint8_t raw[6] = {0};
    uint8_t reg = REG_ACCEL_XOUT;
    err = i2c_master_transmit_receive(s_mpu_dev, &reg, 1, raw, 6, I2C_TIMEOUT_MS);
    if (err != ESP_OK) return err;

    // 상위/하위 바이트 결합
    *ax = (int16_t)(raw[0] << 8 | raw[1]);
    *ay = (int16_t)(raw[2] << 8 | raw[3]);
    *az = (int16_t)(raw[4] << 8 | raw[5]);
    return ESP_OK;
}

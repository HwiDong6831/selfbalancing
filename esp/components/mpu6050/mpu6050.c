#include "mpu6050.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"

#define MUX_ADDR        0x70
#define MPU_ADDR        0x68

#define REG_PWR_MGMT_1  0x6B
#define REG_PWR_MGMT_2  0x6C
#define REG_WHOAMI      0x75
#define REG_ACCEL_XOUT  0x3B

#define I2C_TIMEOUT_MS  10      // I2C 트랜잭션 타임아웃 [ms]
#define MUX_SETTLE_US   500     // mux 채널 전환 후 버스 안정 대기 [µs]

static i2c_port_t s_port;

static esp_err_t mux_select(int channel)
{
    uint8_t mask = (uint8_t)(1 << channel);
    return i2c_master_write_to_device(s_port, MUX_ADDR, &mask, 1,
                                      pdMS_TO_TICKS(I2C_TIMEOUT_MS));
}

esp_err_t mpu6050_init(i2c_port_t port, const int *channels, int num_channels)
{
    esp_err_t err;

    s_port = port;

    for (int i = 0; i < num_channels; i++) {
        int ch = channels[i];
        err = mux_select(ch);
        if (err != ESP_OK) {
            ESP_LOGE("MPU", "채널 %d : mux 선택 실패", ch);
            continue;
        }

        uint8_t wake[2] = {REG_PWR_MGMT_1, 0x00};
        err = i2c_master_write_to_device(s_port, MPU_ADDR, wake, 2,
                                         pdMS_TO_TICKS(I2C_TIMEOUT_MS));
        if (err != ESP_OK) {
            ESP_LOGE("MPU", "채널 %d : wake 실패", ch);
            continue;
        }

        // 이 비트가 남아 있으면 가속도계는 멀쩡한데 자이로만 0 이 온다.
        uint8_t gyro_on[2] = {REG_PWR_MGMT_2, 0x00};
        err = i2c_master_write_to_device(s_port, MPU_ADDR, gyro_on, 2,
                                         pdMS_TO_TICKS(I2C_TIMEOUT_MS));
        if (err != ESP_OK) ESP_LOGE("MPU", "채널 %d : 자이로 대기 해제 실패", ch);

        uint8_t who = 0;
        uint8_t reg = REG_WHOAMI;
        err = i2c_master_write_read_device(s_port, MPU_ADDR, &reg, 1, &who, 1,
                                           pdMS_TO_TICKS(I2C_TIMEOUT_MS));
        if (err == ESP_OK) {
            ESP_LOGI("MPU", "채널 %d : WHO_AM_I = 0x%02x", ch, who);
        } else {
            ESP_LOGE("MPU", "채널 %d : 읽기 실패", ch);
        }
    }

    return ESP_OK;
}

esp_err_t mpu6050_read_accel_gyro(int channel,
                                  int16_t *ax, int16_t *ay, int16_t *az,
                                  int16_t *gx, int16_t *gy, int16_t *gz)
{
    esp_err_t err = mux_select(channel);
    if (err != ESP_OK) return err;

    esp_rom_delay_us(MUX_SETTLE_US);

    // accel(6) + temp(2) + gyro(6) 을 한 트랜잭션으로 읽어 같은 시점의 값을 확보한다.
    uint8_t raw[14] = {0};
    uint8_t reg = REG_ACCEL_XOUT;
    err = i2c_master_write_read_device(s_port, MPU_ADDR, &reg, 1, raw, 14,
                                       pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    if (err != ESP_OK) return err;

    *ax = (int16_t)(raw[0]  << 8 | raw[1]);
    *ay = (int16_t)(raw[2]  << 8 | raw[3]);
    *az = (int16_t)(raw[4]  << 8 | raw[5]);
    *gx = (int16_t)(raw[8]  << 8 | raw[9]);
    *gy = (int16_t)(raw[10] << 8 | raw[11]);
    *gz = (int16_t)(raw[12] << 8 | raw[13]);
    return ESP_OK;
}

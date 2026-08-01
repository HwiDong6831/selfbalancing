#include "mpu6050.h"
#include "esp_log.h"
#include "esp_rom_sys.h"        // esp_rom_delay_us
#include "freertos/FreeRTOS.h"  // pdMS_TO_TICKS

#define MUX_ADDR        0x70
#define MPU_ADDR        0x68

#define REG_PWR_MGMT_1  0x6B
#define REG_PWR_MGMT_2  0x6C
#define REG_WHOAMI      0x75
#define REG_ACCEL_XOUT  0x3B

#define I2C_TIMEOUT_MS  10      // stuck 시 빨리 ESP_ERR_TIMEOUT 반환 (레거시는 CPU 안 멈춤)
#define MUX_SETTLE_US   500     // mux 채널 전환 후 다운스트림 버스 안정 대기 (µs)

// 레거시 i2c 드라이버 사용. 버스 stuck 시 새 i2c_master 처럼 CPU가 인터럽트에서
// 뻗지 않고, 타임아웃 후 ESP_ERR_TIMEOUT 을 반환 → 이전 값 유지로 넘어감.
// (ESP32 Arduino Wire 도 내부적으로 이 레거시 드라이버 기반)
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

    // 채널마다 MPU wake + 센서 확인
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

        // 자이로 3축 대기 해제. 이 비트가 남아 있으면 가속도계는 멀쩡한데 자이로만 0 이 온다.
        uint8_t gyro_on[2] = {REG_PWR_MGMT_2, 0x00};
        err = i2c_master_write_to_device(s_port, MPU_ADDR, gyro_on, 2,
                                         pdMS_TO_TICKS(I2C_TIMEOUT_MS));
        if (err != ESP_OK) ESP_LOGE("MPU", "채널 %d : 자이로 대기 해제 실패", ch);

        // 센서 확인
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

esp_err_t mpu6050_read_accel(int channel, int16_t *ax, int16_t *ay, int16_t *az)
{
    // mux 채널 선택
    esp_err_t err = mux_select(channel);
    if (err != ESP_OK) return err;         // 실패 시 출력 안 건드림 → 호출부가 이전 값 유지

    // 채널 전환 직후 다운스트림 버스가 안정될 때까지 대기
    esp_rom_delay_us(MUX_SETTLE_US);

    // ACCEL_XOUT_H 부터 6바이트(x,y,z 각각 2바이트)
    uint8_t raw[6] = {0};
    uint8_t reg = REG_ACCEL_XOUT;
    err = i2c_master_write_read_device(s_port, MPU_ADDR, &reg, 1, raw, 6,
                                       pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    if (err != ESP_OK) return err;         // 실패 시 출력 안 건드림 → 호출부가 이전 값 유지

    // 상위/하위 바이트 결합
    *ax = (int16_t)(raw[0] << 8 | raw[1]);
    *ay = (int16_t)(raw[2] << 8 | raw[3]);
    *az = (int16_t)(raw[4] << 8 | raw[5]);
    return ESP_OK;
}

esp_err_t mpu6050_read_accel_gyro(int channel,
                                  int16_t *ax, int16_t *ay, int16_t *az,
                                  int16_t *gx, int16_t *gy, int16_t *gz)
{
    // mux 채널 선택
    esp_err_t err = mux_select(channel);
    if (err != ESP_OK) return err;         // 실패 시 출력 안 건드림 → 호출부가 이전 값 유지

    // 채널 전환 직후 다운스트림 버스가 안정될 때까지 대기
    esp_rom_delay_us(MUX_SETTLE_US);

    // ACCEL_XOUT_H 부터 14바이트 버스트: accel(6) + temp(2) + gyro(6).
    // 한 트랜잭션으로 읽어 같은 시점의 accel/gyro 확보.
    //
    // 실패는 거의 전부 ESP_ERR_TIMEOUT 이고, PWM 스위칭 잡음이 원인이다.
    // 통신 속도·mux 전환·센서 무응답은 전부 배제됐다(2026-07-27 일지).
    uint8_t raw[14] = {0};
    uint8_t reg = REG_ACCEL_XOUT;
    err = i2c_master_write_read_device(s_port, MPU_ADDR, &reg, 1, raw, 14,
                                       pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    if (err != ESP_OK) return err;         // 실패 시 출력 안 건드림 → 호출부가 이전 값 유지

    *ax = (int16_t)(raw[0]  << 8 | raw[1]);
    *ay = (int16_t)(raw[2]  << 8 | raw[3]);
    *az = (int16_t)(raw[4]  << 8 | raw[5]);
    // raw[6..7] = 온도 (사용 안 함)
    *gx = (int16_t)(raw[8]  << 8 | raw[9]);
    *gy = (int16_t)(raw[10] << 8 | raw[11]);
    *gz = (int16_t)(raw[12] << 8 | raw[13]);
    return ESP_OK;
}

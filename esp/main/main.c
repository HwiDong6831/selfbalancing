#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c_master.h"

#include "mpu6050.h"

#define I2C_PORT            I2C_NUM_0
#define PIN_SDA             16
#define PIN_SCL             17
#define SENSOR_PERIOD_MS    10

// 사용할 mux 채널 (이 로봇 전용 배선 → main 에 둠)
static const int channels[3] = {0, 1, 6};

void app_main(void)
{
    // I2C 버스 생성
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_PORT,
        .sda_io_num = PIN_SDA,
        .scl_io_num = PIN_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus));

    // MPU 초기화
    ESP_ERROR_CHECK(mpu6050_init(bus, channels, 3));

    // 각 채널 순회하며 가속도 읽고, 200ms 마다 평균 출력
    int16_t ax[3] = {0}, ay[3] = {0}, az[3] = {0};
    TickType_t last = xTaskGetTickCount();

    while (1) {
        for (int i = 0; i < 3; i++) {
            esp_err_t err = mpu6050_read_accel(channels[i], &ax[i], &ay[i], &az[i]);
            if (err != ESP_OK) {
                ESP_LOGE("MPU", "[채널 %d] 값 읽기 실패", channels[i]);
            }
        }

        if (xTaskGetTickCount() - last > pdMS_TO_TICKS(200)) {
            ESP_LOGI("MPU", "X: %6d  Y: %6d  Z: %6d",
                     (ax[0] + ax[1] + ax[2]) / 3,
                     (ay[0] + ay[1] + ay[2]) / 3,
                     (az[0] + az[1] + az[2]) / 3);
            last = xTaskGetTickCount();
        }

        vTaskDelay(pdMS_TO_TICKS(SENSOR_PERIOD_MS));      // 센싱 주기
    }
}

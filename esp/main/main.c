/*
 * main.c — Step 2: MT6701 인코더 각도 읽기 검증
 *
 * 모터 끄고, 축을 손으로 돌리면서 각도가 0~360° 매끄럽게 변하는지 확인.
 * (FOC 오픈루프 테스트는 components/foc + git 이력에 보존.
 *  MPU 센싱은 components/mpu6050 에 보존.)
 */
#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "esp_timer.h"

#include "encoder.h"
#include "mpu6050.h"
#include "foc.h"
#include "balance.h"

#define I2C_PORT    I2C_NUM_0
#define PIN_SDA     16
#define PIN_SCL     17
#define I2C_FREQ_HZ 100000

void app_main(void)
{
    // I2C 버스 생성 (레거시 드라이버: 버스 stuck 시 CPU 행 없이 타임아웃 반환)
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = PIN_SDA,
        .scl_io_num = PIN_SCL,
        .sda_pullup_en = true,
        .scl_pullup_en = true,
        .master.clk_speed = I2C_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_PORT, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_PORT, I2C_MODE_MASTER, 0, 0, 0));

    // MT6701 초기화
    ESP_ERROR_CHECK(encoder_init(I2C_PORT));

    // MPU6050 초기화
    const int channels[3] = {0, 1, 6};
    const int num_channels = 3;
    ESP_ERROR_CHECK(mpu6050_init(I2C_PORT, channels, num_channels));

    // FOC 초기화
    foc_init();
    foc_enable(true);

    // 정렬
    foc_set_phase_voltage(0.0f, 2.0f, 0.0f);
    vTaskDelay(pdMS_TO_TICKS(1000));

    float align_angle = 0.0f;
    encoder_read_angle(&align_angle);
    float angle_offset = foc_align(align_angle);


    // 영점설정
    vTaskDelay(pdMS_TO_TICKS(1000));
    int16_t ax_init, ay_init, az_init;  // 영점
    mpu6050_read_accel(channels[0], &ax_init, &ay_init, &az_init); // 우선 센서 1개로 동작 시험
    ESP_LOGI("MAIN", "영점 설정 완료(%d)", ay_init);

    // 루프
    float prev_angle = 0.0f;
    encoder_read_angle(&prev_angle);
    int64_t prev_time = esp_timer_get_time();

    int16_t ax, ay, az;
    int cnt = 0;
    vTaskDelay(pdMS_TO_TICKS(10));
    while (1) {
        int64_t now_time = esp_timer_get_time();
        float dt = (now_time-prev_time)*1e-6f;
        // TODO: 루프 도는동안 값이 바뀔 가능성 고려해봐야함
        mpu6050_read_accel(channels[0], &ax, &ay, &az); // 우선 센서 1개로 동작 시험
        float target_vel = balance_control(ay, ay_init, dt);

        float now_angle;
        if (encoder_read_angle(&now_angle) == ESP_OK) {
            foc_closeloop_velocity(target_vel, dt, prev_angle, now_angle, angle_offset);
            prev_time = now_time;
            prev_angle = now_angle;
        } else {
            ESP_LOGE("ENC", "읽기 실패");
        }

        if(cnt>20){
            ESP_LOGI("MAIN", "init: %6d   tilt: %6d   target_vel: %6.1f", ay_init, ay, target_vel);
            // ESP_LOGI("MAIN", "now_angle: %6.1f", now_angle);
            // ESP_LOGI("MAIN", "ay: %6d", ay);
            cnt = 0;
        } else {
            cnt++;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

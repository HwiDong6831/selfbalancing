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

    // 정렬 (d축에 전압 인가 → 회전자를 전기각 0°에 고정)
    foc_set_phase_voltage(2.0f, 0.0f, 0.0f);
    vTaskDelay(pdMS_TO_TICKS(1000));

    float align_angle = 0.0f;
    encoder_read_angle(&align_angle);
    float angle_offset = foc_align(align_angle);


    // 영점설정
    vTaskDelay(pdMS_TO_TICKS(1000));
    int16_t ax_init, ay_init, az_init;  // 영점
    mpu6050_read_accel(channels[0], &ax_init, &ay_init, &az_init); // 우선 센서 1개로 동작 시험
    ESP_LOGI("MAIN", "영점 설정 완료(%d)", ay_init);

    // 자이로 gx bias 측정
    float gx_bias = 0.0f;
    {
        int32_t sum = 0;
        int n_ok = 0;
        int16_t t_ax, t_ay, t_az, t_gx, t_gy, t_gz;
        for (int i = 0; i < 200; i++) {
            if (mpu6050_read_accel_gyro(channels[0], &t_ax, &t_ay, &t_az,
                                        &t_gx, &t_gy, &t_gz) == ESP_OK) {
                sum += t_gx;
                n_ok++;
            }
            vTaskDelay(pdMS_TO_TICKS(3));
        }
        gx_bias = n_ok ? (float)sum / n_ok : 0.0f;
        ESP_LOGI("MAIN", "gx_bias = %.1f (n=%d)", gx_bias, n_ok);
    }

    // 루프
    float prev_angle = 0.0f;
    encoder_read_angle(&prev_angle);
    int64_t prev_time = esp_timer_get_time();

    int16_t ax, ay, az;
    int16_t gx, gy, gz;   // [임시] 자이로 축 확인용
    int cnt = 0;
    vTaskDelay(pdMS_TO_TICKS(10));
    while (1) {
        int64_t now_time = esp_timer_get_time();
        float dt = (now_time-prev_time)*1e-6f;
        // TODO: 루프 도는동안 값이 바뀔 가능성 고려해봐야함
        mpu6050_read_accel_gyro(channels[0], &ax, &ay, &az, &gx, &gy, &gz);
        // float target_vel = balance_control(ay, ay_init, dt);   // [보존] 속도모드

        // 상보필터 각도 추정 → 토크모드 제어
        float rate;
        float angle = balance_estimate_angle(ay, az, gx, gx_bias, dt, &rate);
        float uq = balance_torque(angle, rate);

        float now_angle;
        if (encoder_read_angle(&now_angle) == ESP_OK) {
            // foc_closeloop_velocity(target_vel, dt, prev_angle, now_angle, angle_offset); // [보존]
            foc_apply_torque(uq, now_angle, angle_offset);
            prev_time = now_time;
            prev_angle = now_angle;
        } else {
            ESP_LOGE("ENC", "읽기 실패");
        }

        if(cnt>200){
            ESP_LOGI("MAIN", "angle: %7.2f  rate: %8.2f  uq: %5.2f", angle, rate, uq);
            cnt = 0;
        } else {
            cnt++;
        }

        vTaskDelay(pdMS_TO_TICKS(1));   // 빠른 커뮤테이션 (진동 개선)
    }
}

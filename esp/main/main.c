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

    // 밸런스 setpoint 캘리브레이션:
    // 로봇을 밸런스 자세로 잡은 채 ~2초간 상보필터를 수렴시켜, 그 각도를 기준(0)으로.
    float balance_setpoint = 0.0f;
    {
        int16_t c_ax, c_ay, c_az, c_gx, c_gy, c_gz;
        int64_t t_prev  = esp_timer_get_time();
        int64_t t_start = t_prev;
        float ang = 0.0f, r;
        while (esp_timer_get_time() - t_start < 2000000) {   // 2초
            int64_t t_now = esp_timer_get_time();
            float cdt = (t_now - t_prev) * 1e-6f;
            t_prev = t_now;
            if (mpu6050_read_accel_gyro(channels[0], &c_ax, &c_ay, &c_az,
                                        &c_gx, &c_gy, &c_gz) == ESP_OK) {
                ang = balance_estimate_angle(c_ay, c_az, c_gx, gx_bias, cdt, &r);
            }
            vTaskDelay(pdMS_TO_TICKS(3));
        }
        balance_setpoint = ang;
        ESP_LOGI("MAIN", "balance_setpoint = %.2f deg", balance_setpoint);
    }

    // 루프
    int64_t prev_time = esp_timer_get_time();

    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    int cnt = 0;
    vTaskDelay(pdMS_TO_TICKS(10));
    while (1) {
        int64_t now_time = esp_timer_get_time();
        float dt = (now_time - prev_time) * 1e-6f;
        prev_time = now_time;

        mpu6050_read_accel_gyro(channels[0], &ax, &ay, &az, &gx, &gy, &gz);

        float rate;
        float angle = balance_estimate_angle(ay, az, gx, gx_bias, dt, &rate);
        float uq = balance_torque(angle - balance_setpoint, rate);

        float now_angle;
        if (encoder_read_angle(&now_angle) == ESP_OK) {
            foc_apply_torque(uq, now_angle, angle_offset);
        } else {
            ESP_LOGE("ENC", "읽기 실패");
        }

        if(cnt>200){
            ESP_LOGI("MAIN", "err: %7.2f  rate: %8.2f  uq: %5.2f",
                     angle - balance_setpoint, rate, uq);
            cnt = 0;
        } else {
            cnt++;
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

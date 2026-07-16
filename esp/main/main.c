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
#include "driver/i2c_master.h"
#include "esp_timer.h"

#include "encoder.h"
#include "foc.h"

#define I2C_PORT    I2C_NUM_0
#define PIN_SDA     16
#define PIN_SCL     17

void app_main(void)
{
    // I2C 버스 생성 (MT6701 은 메인 버스 직결)
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

    ESP_ERROR_CHECK(encoder_init(bus));

    foc_init();
    foc_enable(true);

    // while(1){
    //     foc_openloop_velocity(5, 0.01f);
    //     vTaskDelay(pdMS_TO_TICKS(10));
    // }



    ESP_LOGI("FOC", "foc 초기화 작업 완료");

    // 정렬
    foc_set_phase_voltage(0.0f, 2.0f, 0.0f);

    vTaskDelay(pdMS_TO_TICKS(3000));

    float align_angle = 0.0f;
    encoder_read_angle(&align_angle);
    float angle_offset = foc_align(align_angle);

    ESP_LOGI("FOC", "전기각 정렬 완료(align_angle: %f, offset: %f)", align_angle, angle_offset);

    static float prev_angle = 0.0f;
    float prev_time = esp_timer_get_time();

    while (1) {
        float now_angle;
        if (encoder_read_angle(&now_angle) == ESP_OK) {
            ESP_LOGI("ENC", "각도: %f deg", now_angle * (180.0f / M_PI));
            float now_time = esp_timer_get_time();
            foc_closeloop_velocity(10, (now_time-prev_time)*1e-6f, prev_angle, now_angle, angle_offset);
            prev_time = now_time;
            prev_angle = now_angle;
        } else {
            ESP_LOGE("ENC", "읽기 실패");
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

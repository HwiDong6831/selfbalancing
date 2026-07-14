/*
 * main.c — Step 1: BLDC 오픈루프 회전 테스트 (배선/드라이버 검증용)
 *
 * 주의: 오픈루프는 발열 큼 (회전자 위치 무시하고 전압 강제 인가).
 * 짧게만 전원 넣고 확인할 것. 검증 끝나면 클로즈드루프 FOC 로.
 *
 * (MPU 센싱 로직은 components/mpu6050 에 그대로 있음. 복원하려면
 *  이전 app_main 의 i2c 버스 생성 + mpu6050_init/read 루프 사용.)
 */
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_log.h"

#include "foc.h"

void app_main(void)
{
    foc_init();
    foc_enable(true);

    const float target_vel = 10.0f;    // 최종 목표 축 속도 [rad/s] (~1.6 rev/s)
    const float accel = 5.0f;          // 가속도 [rad/s^2] → 0에서 목표까지 2초
    float vel = 0.0f;                  // 현재 속도 (0에서 램프업)
    int64_t last = esp_timer_get_time();

    ESP_LOGI("FOC", "오픈루프 회전 시작 (발열 주의, 짧게)");

    while (1) {
        // 실제 경과 시간으로 dt 계산 (루프 주기와 무관하게 전기각 정확히 적분)
        int64_t now = esp_timer_get_time();
        float dt = (now - last) / 1000000.0f;   // us → s
        last = now;

        // 목표 속도까지 서서히 램프업 (오픈루프 락 유지)
        vel += accel * dt;
        if (vel > target_vel) vel = target_vel;

        foc_openloop_velocity(vel, dt);

        vTaskDelay(1);
    }
}

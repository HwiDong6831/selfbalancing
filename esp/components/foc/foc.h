#pragma once

#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// LEDC 3채널 + EN 핀 초기화.
void foc_init(void);

// 드라이버 enable/disable.
void foc_enable(bool on);

/*
 * PWM 주파수 변경 (진단용).
 *
 * 스위칭 가장자리가 I2C 를 방해하는지 보려고 주파수만 바꿔 비교할 때 쓴다.
 * 11비트 해상도를 유지해야 하므로 80MHz/hz 가 2048 이상이어야 한다(= 39kHz 이하).
 */
esp_err_t foc_set_pwm_freq(uint32_t hz);

float foc_align(float align_angle);

// 전압 벡터 → 3상 PWM duty
// ud       : d축 전압 (0)
// uq       : q축 전압 (토크)
// angle_el : 전기각 [rad]
void foc_set_phase_voltage(float ud, float uq, float angle_el);

// 오픈루프 속도 제어.
// target_vel : 목표 회전 속도 [축 기준 rad/s]
// dt         : 지난 호출 이후 경과 시간 [s]
// 반환: 현재 전기각 [rad]
float foc_openloop_velocity(float target_vel, float dt);

// [보존] 클로즈루프 속도 제어. 토크모드로 대체되어 미사용.
// float foc_closeloop_velocity(float target_vel, float dt, float prev_angle, float now_angle, float angle_offset);

// 토크 인가 (속도 PID 우회). 밸런싱 상태피드백용.
// uq          : q축 전압 (토크). 내부에서 ±V_SUPPLY 클램프.
// now_angle   : 현재 기계각 [rad] (인코더)
// angle_offset: 정렬 오프셋
void foc_apply_torque(float uq, float now_angle, float angle_offset);

#ifdef __cplusplus
}
#endif

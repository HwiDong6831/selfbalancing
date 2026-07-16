#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// LEDC 3채널 + EN 핀 초기화.
void foc_init(void);

// 드라이버 enable/disable.
void foc_enable(bool on);

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

// 클로즈루프 속도 제어.
// target_vel : 목표 회전 속도 [축 기준 rad/s]
// dt         : 지난 호출 이후 경과 시간 [s]
// angle_rad  : 현재 기계각 [rad]
float foc_closeloop_velocity(float target_vel, float dt, float prev_angle, float now_angle, float angle_offset);

#ifdef __cplusplus
}
#endif

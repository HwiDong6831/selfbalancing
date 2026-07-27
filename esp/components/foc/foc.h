#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// LEDC 3채널 + EN 핀 초기화.
void foc_init(void);

// 드라이버 enable/disable.
void foc_enable(bool on);

// 정렬 오프셋 계산. 정·역 스윕 끝에서 각각 잰 기계각을 넘기면 평균내 편향을 상쇄한다.
float foc_align(float angle_fwd, float angle_rev);

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

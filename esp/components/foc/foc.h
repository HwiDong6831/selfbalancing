#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// LEDC 3채널 + EN 핀 초기화.
void foc_init(void);

// 드라이버 enable/disable.
void foc_enable(bool on);

// 정·역 스윕 끝에서 잰 기계각으로 정렬 오프셋을 구한다.
float foc_align(float angle_fwd, float angle_rev);

// 전압 벡터(ud, uq)를 전기각 [rad] 에 실어 3상 PWM duty 로 낸다.
void foc_set_phase_voltage(float ud, float uq, float angle_el);

// 토크 인가. uq [V] 를 현재 기계각에 맞춘 전기각에 직접 싣는다.
void foc_apply_torque(float uq, float now_angle, float angle_offset);

#ifdef __cplusplus
}
#endif

/*
 * foc.h — BLDC 전압 모드 FOC 드라이버 (SimpleFOC Mini, 3-PWM)
 *
 * SimpleFOC Mini = DRV8313 기반 3-PWM 게이트 드라이버 → 전류 센서 없음.
 * 따라서 전류 루프 없는 "전압 모드 FOC" 만 구현.
 * 핵심: 전압 벡터(Ud, Uq)를 전기각(angle_el)에 맞춰 3상 PWM duty 로 변환.
 *
 * 배선: EN=GPIO5, IN1=GPIO18, IN2=GPIO19, IN3=GPIO23, VM=3S Lipo(~11.3V)
 */
#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// LEDC 3채널 + EN 핀 초기화. 초기 상태는 disable.
void foc_init(void);

// 드라이버 enable/disable. disable 시 3상 duty 0 + EN 핀 low.
void foc_enable(bool on);

/*
 * 전압 벡터 → 3상 PWM duty (FOC 핵심).
 *  ud       : d축 전압 (보통 0)
 *  uq       : q축 전압 (토크 성분)
 *  angle_el : 전기각 [rad]
 * inverse Park → inverse Clarke → 각 상 duty(0.5 중심) 로 변환.
 */
void foc_set_phase_voltage(float ud, float uq, float angle_el);

/*
 * 오픈루프 속도 제어 (인코더 없이 강제 회전, 배선/드라이버 검증용).
 *  target_vel : 목표 회전 속도 [축 기준 rad/s]
 *  dt         : 지난 호출 이후 경과 시간 [s]
 * 반환: 현재 전기각 [rad]
 * 주의: 발열 큼. 전압 낮게, 짧게만.
 */
float foc_openloop_velocity(float target_vel, float dt);

#ifdef __cplusplus
}
#endif

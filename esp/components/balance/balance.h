#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// [보존] 속도모드 밸런싱 제어. 토크모드(balance_torque)로 대체되어 미사용.
// float balance_control(int16_t angle, int16_t balance_zero_angle, float dt);

/*
 * 토크모드 밸런싱 제어 (상태피드백).
 *
 * angle     : 상보필터 추정 각도 [deg] (0 = 똑바로)
 * rate      : 각속도 [deg/s]
 * wheel_vel : 휠 속도 [rad/s]. 한쪽으로 계속 감기면 기준 각도가 틀렸다는 신호다
 *
 * 반환: q축 전압 uq [V]. |angle| > 컷오프면 0 (넘어짐 → 정지).
 */
float balance_torque(float angle, float rate, float wheel_vel);

/*
 * 상보필터로 tilt 각도 추정 (자이로 gz + 가속도 ax/ay 융합).
 *
 * ax, ay   : 가속도 raw (tilt 축은 Z → XY 평면에서 각도)
 * gz       : 자이로 raw (tilt 각속도 축)
 * gz_bias  : 정지 시 측정한 gz 평균 (offset 제거용)
 * dt       : 루프 주기 (초)
 * rate_out : (출력) 각속도 [deg/s], NULL 허용
 *
 * 반환: 융합 각도 [deg]
 */
float balance_estimate_angle(int16_t ax, int16_t ay, int16_t gz,
                             float gz_bias, float dt, float *rate_out);

#ifdef __cplusplus
}
#endif

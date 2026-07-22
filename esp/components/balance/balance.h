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
 * angle : 상보필터 추정 각도 [deg] (0 = 똑바로)
 * rate  : 각속도 [deg/s]
 *
 * 반환: q축 전압 uq [V]. |angle| > 컷오프면 0 (넘어짐 → 정지).
 */
float balance_torque(float angle, float rate);

/*
 * 상보필터로 tilt 각도 추정 (자이로 gx + 가속도 ay/az 융합).
 *
 * ay, az   : 가속도 raw (tilt 축은 X → YZ 평면에서 각도)
 * gx       : 자이로 raw (tilt 각속도 축)
 * gx_bias  : 정지 시 측정한 gx 평균 (offset 제거용)
 * dt       : 루프 주기 (초)
 * rate_out : (출력) 각속도 [deg/s], NULL 허용
 *
 * 반환: 융합 각도 [deg]
 */
float balance_estimate_angle(int16_t ay, int16_t az, int16_t gx,
                             float gx_bias, float dt, float *rate_out);

#ifdef __cplusplus
}
#endif

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

float balance_control(int16_t angle, int16_t balance_zero_angle, float dt);

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

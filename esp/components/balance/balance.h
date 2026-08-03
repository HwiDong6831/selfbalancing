#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// 토크모드 밸런싱 제어. 상태피드백으로 q축 전압 [V] 을 낸다.
float balance_torque(float angle, float rate, float wheel_vel);

// 상보필터로 tilt 각도 [deg] 를 추정한다. rate_out 은 각속도 [deg/s], NULL 허용.
float balance_estimate_angle(int16_t ax, int16_t ay, int16_t gz,
                             float gz_bias, float dt, float *rate_out);

#ifdef __cplusplus
}
#endif

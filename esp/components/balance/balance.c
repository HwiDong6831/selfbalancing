#include "balance.h"
#include <math.h>
#include <stdbool.h>

#define GYRO_LSB_PER_DPS  131.0f // 자이로 감도 (±250°/s 기준)
#define COMP_ALPHA        0.98f  // 상보필터의 자이로 비중

#define BAL_K1          -4.0f    // angle 게인 [V/deg]
#define BAL_K2          -1.0f    // rate 게인 [V/(deg/s)]
#define BAL_K3           0.3f    // 휠속도 게인 [V/(rad/s)]

#define UQ_LIMIT        6.0f     // uq 클램프 [V]
#define TILT_CUTOFF     35.0f    // |angle| 초과 시 정지 [deg]
#define DEADBAND        0.0f     // 각도항을 무시할 |error| [deg]. 0 = 끔

static float clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

// angle=0 이 목표(똑바로).
float balance_torque(float angle, float rate, float wheel_vel)
{
    if (fabsf(angle) > TILT_CUTOFF) return 0.0f;

    float pos = (fabsf(angle) < DEADBAND) ? 0.0f : angle;

    float uq = BAL_K1 * pos + BAL_K2 * rate + BAL_K3 * wheel_vel;
    return clampf(uq, -UQ_LIMIT, UQ_LIMIT);
}

float balance_estimate_angle(int16_t ax, int16_t ay, int16_t gz,
                             float gz_bias, float dt, float *rate_out)
{
    static float angle = 0.0f;
    static bool  init  = false;

    float accel_angle = atan2f((float)ax, (float)ay) * (180.0f / (float)M_PI);
    float rate = ((float)gz - gz_bias) / GYRO_LSB_PER_DPS;

    if (!init) {
        angle = accel_angle;
        init  = true;
    }
    angle = COMP_ALPHA * (angle + rate * dt) + (1.0f - COMP_ALPHA) * accel_angle;

    if (rate_out) *rate_out = rate;
    return angle;
}

#include "balance.h"
#include <math.h>
#include <stdbool.h>

// PID
#define PID_KP          0.2f                    // 오차 반영 계수
#define PID_KD          0.0f                    //

// 상보필터
#define GYRO_LSB_PER_DPS  131.0f                // 자이로 기본 ±250°/s
#define COMP_ALPHA        0.98f                 // 자이로 비중 (나머지는 가속도로 드리프트 보정)

float balance_estimate_angle(int16_t ay, int16_t az, int16_t gx,
                             float gx_bias, float dt, float *rate_out)
{
    static float angle = 0.0f;
    static bool  init  = false;

    // 가속도로 절대 각도 (tilt 축 X → YZ 평면), deg.
    // -ay,-az 로 180° 이동 → 똑바로 선 자세가 ±180 경계 아닌 0 근처가 되게 함.
    float accel_angle = atan2f(-(float)ay, -(float)az) * (180.0f / (float)M_PI);
    // 자이로 각속도 (bias 제거), deg/s
    float rate = ((float)gx - gx_bias) / GYRO_LSB_PER_DPS;

    if (!init) {            // 첫 샘플은 가속도각으로 초기화 (수렴 빠르게)
        angle = accel_angle;
        init  = true;
    }
    // 자이로 적분(빠름) + 가속도(느린 드리프트 보정)
    angle = COMP_ALPHA * (angle + rate * dt) + (1.0f - COMP_ALPHA) * accel_angle;

    if (rate_out) *rate_out = rate;
    return angle;
}


float balance_control(int16_t angle, int16_t balance_zero_angle, float dt){
    float error = (float)(balance_zero_angle - angle);

    static float prev_error = 0.0f;
    float d = (error - prev_error)/dt;
    prev_error = error;

    float target_vel = (PID_KP*error + PID_KD*d) / 150 ;
    return target_vel;
}

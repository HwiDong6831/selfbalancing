#include "balance.h"
#include <math.h>
#include <stdbool.h>

// PID (구 속도모드용, 보존)
#define PID_KP          0.2f                    // 오차 반영 계수
#define PID_KD          0.0f                    //

// 상보필터
#define GYRO_LSB_PER_DPS  131.0f                // 자이로 기본 ±250°/s
#define COMP_ALPHA        0.98f                 // 자이로 비중 (나머지는 가속도로 드리프트 보정)

// 토크모드 상태피드백
#define BAL_K1          -2.0f                   // angle 게인 [V/deg] (부호: 실기 확인해 반대라 음수)
#define BAL_K2          -0.4f                    // rate 게인 [V/(deg/s)] (부호 확인 후 추가, K1과 같은 부호계열)
#define UQ_LIMIT        11.0f                    // uq 클램프 [V]
#define TILT_CUTOFF     35.0f                   // |angle| 초과 시 정지 [deg]
#define DEADBAND        1.0f                    // |error| < 이 값이면 각도항 무시 [deg]

static float clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

// uq = K1·angle + K2·rate. angle=0 이 목표(똑바로).
// 부호가 반대면 즉시 넘어짐 → BAL_K1/K2 부호 뒤집기.
float balance_torque(float angle, float rate)
{
    if (fabsf(angle) > TILT_CUTOFF) return 0.0f;   // 넘어짐 → 모터 정지

    // ±DEADBAND 안에서는 각도항 무시(미세 진동 억제). 밖이면 full 인가(경계 넘으면 확).
    // 각속도항은 항상 유지 → 초기 낙하 감지·댐핑.
    float pos = (fabsf(angle) < DEADBAND) ? 0.0f : angle;
    float uq = BAL_K1 * pos + BAL_K2 * rate;
    return clampf(uq, -UQ_LIMIT, UQ_LIMIT);
}

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


// [보존] 속도모드 밸런싱 제어. 토크모드로 대체되어 미사용.
#if 0
float balance_control(int16_t angle, int16_t balance_zero_angle, float dt){
    float error = (float)(balance_zero_angle - angle);

    static float prev_error = 0.0f;
    float d = (error - prev_error)/dt;
    prev_error = error;

    float target_vel = (PID_KP*error + PID_KD*d) / 150 ;
    return target_vel;
}
#endif

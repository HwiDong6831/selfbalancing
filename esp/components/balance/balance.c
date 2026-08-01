#include "balance.h"
#include <math.h>
#include <stdbool.h>

// PID (구 속도모드용, 보존)
#define PID_KP          0.2f                    // 오차 반영 계수
#define PID_KD          0.0f                    //

// 상보필터
#define GYRO_LSB_PER_DPS  131.0f                // 자이로 기본 ±250°/s
#define COMP_ALPHA        0.98f                 // 자이로 비중 (나머지는 가속도로 드리프트 보정)

/*
 * 토크모드 상태피드백. 세 항이 서로 다른 시간 규모를 본다.
 *
 *   K1 넘어져 있다          → 되돌린다
 *   K2 넘어지려 한다        → 미리 막는다 (가장 빠름)
 *   K3 계속 이쪽으로 밀린다 → 기준 자세 자체를 옮긴다 (가장 느림)
 *
 * 부호는 전부 실기에서 쟀다. 계산으로 유도해 두 번 틀렸다(2026-07-28 일지).
 */
#define BAL_K1          -4.0f    // angle 게인 [V/deg]
#define BAL_K2          -1.0f    // rate 게인 [V/(deg/s)]. 잡음이 들어오는 유일한 창구다
#define BAL_K3           0.3f    // 휠속도 게인 [V/(rad/s)]. K1/K2 와 부호가 반대다

#define UQ_LIMIT        6.0f     // uq 클램프 [V]. foc.c 의 선형 한계(V_SUPPLY/2)를 3% 넘는다
#define TILT_CUTOFF     35.0f    // |angle| 초과 시 정지 [deg]
#define DEADBAND        0.0f     // |error| < 이 값이면 각도항 무시 [deg]. 0 = 끔

static float clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

// angle=0 이 목표(똑바로). 부호가 반대면 기울수록 넘어지는 쪽으로 밀어 즉시 넘어진다.
float balance_torque(float angle, float rate, float wheel_vel)
{
    if (fabsf(angle) > TILT_CUTOFF) return 0.0f;   // 넘어짐 → 모터 정지

    // 데드밴드는 복원력이 0 인 구간을 만든다. 미세 진동을 막는 값보다 그 대가가 커서 껐다.
    float pos = (fabsf(angle) < DEADBAND) ? 0.0f : angle;

    // 휠이 한쪽으로 계속 감긴다 = 지금 0 도라고 믿는 자세가 사실은 기울어 있다.
    // 그 신호로 목표 자세를 옮겨, 기준 각도가 정확하지 않아도 진짜 균형점을 찾아가게 한다.
    float uq = BAL_K1 * pos + BAL_K2 * rate + BAL_K3 * wheel_vel;
    return clampf(uq, -UQ_LIMIT, UQ_LIMIT);
}

float balance_estimate_angle(int16_t ax, int16_t ay, int16_t gz,
                             float gz_bias, float dt, float *rate_out)
{
    static float angle = 0.0f;
    static bool  init  = false;

    // 가속도로 절대 각도 (tilt 축 Z → XY 평면), deg.
    // 똑바로 선 자세에서 중력이 +Y 로 실려 ay≈1g, ax≈0 이므로 그대로 0 도가 나온다.
    float accel_angle = atan2f((float)ax, (float)ay) * (180.0f / (float)M_PI);
    // 자이로 각속도 (bias 제거), deg/s
    float rate = ((float)gz - gz_bias) / GYRO_LSB_PER_DPS;

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

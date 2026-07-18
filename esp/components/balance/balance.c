#include "balance.h"

// PID
#define PID_KP          0.2f                    // 오차 반영 계수
#define PID_KD          0.0f                    // 


float balance_control(int16_t angle, int16_t balance_zero_angle, float dt){
    float error = (float)(balance_zero_angle - angle);

    static float prev_error = 0.0f;
    float d = (error - prev_error)/dt;
    prev_error = error;

    float target_vel = (PID_KP*error + PID_KD*d) / 150 ;
    return target_vel;
}

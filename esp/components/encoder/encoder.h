#pragma once

#include "esp_err.h"
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

// PCNT 유닛과 Z 인터럽트 초기화. 카운트 0 에서 시작한다.
esp_err_t encoder_init(void);

// 현재 카운트. 한 바퀴 안에서의 위치이며 -CPR < c < CPR 이다.
int encoder_get_count(void);

// 각도를 라디안으로 (0 ~ 2π)
esp_err_t encoder_read_angle(float *angle_rad);

/*
 * [진단] Z 통과 횟수와 마지막 Z 시점의 카운트.
 *
 * Z 는 한 바퀴에 한 번 나오므로 매번 같은 위치에서 걸린다. 따라서 ENC_CPR 이 맞으면
 * last 가 한 자리에 머물고, 틀리면 회전할 때마다 (실제 CPR - ENC_CPR) 만큼 일정하게
 * 밀린다. 이 값으로 CPR 을 확정하고 누적 오차 크기도 같이 본다.
 */
void encoder_get_z(uint32_t *count, int *last);

#ifdef __cplusplus
}
#endif

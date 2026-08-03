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

#ifdef __cplusplus
}
#endif

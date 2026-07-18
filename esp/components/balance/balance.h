#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

float balance_control(int16_t angle, int16_t balance_zero_angle, float dt);

#ifdef __cplusplus
}
#endif

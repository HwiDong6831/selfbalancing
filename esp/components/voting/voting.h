#pragma once

#include <stdbool.h>
#include <stdint.h>

#define VOTING_N         3    // 센서 수
#define VOTING_AXIS_MAX  2    // 한 번에 비교하는 축 수의 최대값

#define VOTING_TOL_ACC   1500 // 가속도 이상치 판정 임계값 [LSB]
#define VOTING_TOL_GYRO   300 // 자이로 이상치 판정 임계값 [LSB]

typedef enum {
    VOTING_OK = 0,      // 3개 전부 채택
    VOTING_DEGRADED,    // 1개 배제하고 나머지 둘로 계속
    VOTING_SINGLE,      // 남은 하나로 계속하되 검증은 못 한다
    VOTING_FAIL,        // 다수결 불가. 값을 내지 않는다
} voting_result_t;

typedef struct {
    voting_result_t result;
    int16_t val[VOTING_AXIS_MAX];   // 채택된 센서들의 평균. FAIL 이면 0
    bool    used[VOTING_N];         // 센서별 채택 여부
} voting_out_t;

// 같은 신호를 센서 3개끼리 비교해 채택 집합과 평균을 낸다. valid=false 는 읽기 실패.
void voting_fuse(const int16_t v[VOTING_N][VOTING_AXIS_MAX], const bool valid[VOTING_N],
                 int axes, int16_t tol, voting_out_t *out);

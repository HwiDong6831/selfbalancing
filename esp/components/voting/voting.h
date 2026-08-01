#pragma once

#include <stdbool.h>
#include <stdint.h>

// ESP-IDF 에 의존하지 않는 순수 C. 호스트에서 그대로 컴파일해 테스트한다 (test/ 참조).

#define VOTING_N         3    // 센서 수
#define VOTING_AXIS_MAX  2    // 한 번에 비교하는 축 수의 최대값 (가속도쌍 2, 자이로 1)

// 이상치 판정 임계값 [LSB]. 실측 편차에 여유를 얹었다. 좁히면 안 된다 (2026-08-02 일지).
#define VOTING_TOL_ACC   1500
#define VOTING_TOL_GYRO   300

typedef enum {
    VOTING_OK = 0,      // 3개 전부 채택
    VOTING_DEGRADED,    // 1개 배제하고 나머지 둘로 계속
    VOTING_FAIL,        // 다수결 불가. 값을 내지 않는다
} voting_result_t;

typedef struct {
    voting_result_t result;
    int16_t val[VOTING_AXIS_MAX];   // 채택된 센서들의 평균. FAIL 이면 0
    bool    used[VOTING_N];         // 센서별 채택 여부
} voting_out_t;

/*
 * 같은 신호를 센서 3개끼리 비교해 채택 집합과 평균을 낸다.
 * 가속도쌍(axes 2)과 자이로(axes 1)를 따로 부른다 — 이유는 2026-08-02 일지.
 *
 * valid=false 는 읽기 실패. FAIL 이면 호출자가 직전 값을 유지해야 한다.
 */
void voting_fuse(const int16_t v[VOTING_N][VOTING_AXIS_MAX], const bool valid[VOTING_N],
                 int axes, int16_t tol, voting_out_t *out);

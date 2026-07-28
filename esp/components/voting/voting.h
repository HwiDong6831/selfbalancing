#pragma once

#include <stdbool.h>
#include <stdint.h>

// ESP-IDF 에 의존하지 않는 순수 C. 호스트에서 그대로 컴파일해 테스트한다.

#define VOTING_N 3

/*
 * 이상치 판정 임계값 [LSB]. 중앙값에서 이만큼 넘게 벗어나면 그 센서를 버린다.
 * 채널별 영점 보정을 적용한 뒤 실측한 정상 편차에 여유를 얹어 정했다 (2026-07-29 일지).
 *
 *            정상 편차   임계값
 *   가속도    109~490     1500   3배 여유. 2048 짜리 비트 뒤집힘도 잡힌다
 *   자이로     29~150      300   2배 여유
 *
 * 함부로 좁히면 안 된다. 세 센서를 mux 로 1ms 씩 어긋나게 읽으므로 급변 구간에서는
 * 정상이어도 값이 벌어진다. 그 폭을 재보지 않고 조이면, 외란에 반응해야 할 바로 그
 * 순간에 FAIL 로 멈춘다.
 */
#define VOTING_TOL_ACC   1500
#define VOTING_TOL_GYRO   300

/*
 * FAIL 은 "값이 없다" 가 아니라 "검증되지 않은 값" 이라는 뜻이다.
 * 안전 필수 분야라면 여기서 안전 상태로 전환하지만, 이 로봇은 제어를 끊으면 넘어진다.
 */
typedef enum {
    VOTING_OK = 0,      // 3개 전부 채택
    VOTING_DEGRADED,    // 1개 배제하고 나머지로 계속. 여전히 상호 검증됨
    VOTING_FAIL,        // 다수결 불가. 값은 내지만 맞는지 확인할 수 없음
} voting_result_t;

/*
 * 제어에 실제로 쓰는 세 축만 본다 (balance_estimate_angle 의 입력).
 * 단위는 센서 raw LSB 그대로다. 변환하면 오차만 끼어든다.
 *
 * valid = false 는 읽기 실패(dropout). 값을 보지 않고 배제한다.
 */
typedef struct {
    int16_t ay, az, gx;
    bool    valid;
} voting_sample_t;

typedef struct {
    voting_result_t result;
    int16_t ay, az, gx;          // 채택된 센서들의 융합값
    bool    used[VOTING_N];      // 채널별 채택 여부
} voting_out_t;

/*
 * 센서 3개를 융합한다.
 *
 * FAIL 이면 out 의 값은 0 이다. 틀린 줄 아는 값을 내미느니 비워 둔다.
 * 대신 호출자가 직전 값을 유지해야 한다 — 밸런싱 중에 제어를 끊으면 그대로 넘어진다.
 */
void voting_fuse(const voting_sample_t in[VOTING_N], voting_out_t *out);

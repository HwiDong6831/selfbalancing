/*
 * voting_fuse 호스트 테스트. ESP32 에 올리지 않고 PC 에서 바로 돌린다.
 *
 *   gcc -std=c11 -Wall -I.. ../voting.c test_voting.c -o test_voting && ./test_voting
 */
#include <stdio.h>
#include "voting.h"

static int fails = 0;

#define CHECK(cond, ...)                                        \
    do {                                                        \
        if (!(cond)) {                                          \
            printf("  FAIL %s:%d  ", __func__, __LINE__);       \
            printf(__VA_ARGS__);                                \
            printf("\n");                                       \
            fails++;                                            \
        }                                                       \
    } while (0)

static const bool ALL_OK[VOTING_N] = { true, true, true };

// --- 가속도쌍 (축 2개) ---

static void test_all_agree(void)
{
    const int16_t v[VOTING_N][VOTING_AXIS_MAX] = { {100, 16000}, {200, 16100}, {300, 16200} };
    voting_out_t o;

    voting_fuse(v, ALL_OK, 2, VOTING_TOL_ACC, &o);

    CHECK(o.result == VOTING_OK, "result=%d", o.result);
    CHECK(o.used[0] && o.used[1] && o.used[2], "셋 다 채택되어야 한다");
    CHECK(o.val[0] == 200 && o.val[1] == 16100, "평균 %d,%d", o.val[0], o.val[1]);
}

// 비트 하나가 뒤집혀 2048 만큼 튄 경우. 임계값 1500 을 넘으므로 걸린다.
static void test_one_outlier(void)
{
    const int16_t v[VOTING_N][VOTING_AXIS_MAX] = { {100, 16000}, {200, 16100}, {150, 18148} };
    voting_out_t o;

    voting_fuse(v, ALL_OK, 2, VOTING_TOL_ACC, &o);

    CHECK(o.result == VOTING_DEGRADED, "result=%d", o.result);
    CHECK(o.used[0] && o.used[1] && !o.used[2], "2번만 배제되어야 한다");
    CHECK(o.val[0] == 150 && o.val[1] == 16050, "평균 %d,%d", o.val[0], o.val[1]);
}

// 0~1, 1~2 는 일치하지만 0~2 는 불일치인 사슬. 누가 맞는지 못 가린다.
static void test_chain_is_fail(void)
{
    const int16_t v[VOTING_N][VOTING_AXIS_MAX] = { {0, 0}, {1400, 0}, {2800, 0} };
    voting_out_t o;

    voting_fuse(v, ALL_OK, 2, VOTING_TOL_ACC, &o);

    CHECK(o.result == VOTING_FAIL, "result=%d", o.result);
    CHECK(o.val[0] == 0 && o.val[1] == 0, "FAIL 이면 값을 내지 않는다");
    CHECK(!o.used[0] && !o.used[1] && !o.used[2], "채택이 없어야 한다");
}

static void test_all_disagree(void)
{
    const int16_t v[VOTING_N][VOTING_AXIS_MAX] = { {0, 0}, {5000, 0}, {10000, 0} };
    voting_out_t o;

    voting_fuse(v, ALL_OK, 2, VOTING_TOL_ACC, &o);

    CHECK(o.result == VOTING_FAIL, "result=%d", o.result);
}

// 임계값은 포함이다. 딱 tol 이면 일치, 하나 더 벌어지면 불일치.
static void test_tolerance_edge(void)
{
    voting_out_t o;

    const int16_t on[VOTING_N][VOTING_AXIS_MAX] = { {0, 0}, {VOTING_TOL_ACC, 0}, {0, 0} };
    voting_fuse(on, ALL_OK, 2, VOTING_TOL_ACC, &o);
    CHECK(o.result == VOTING_OK, "딱 임계값이면 일치여야 한다 (result=%d)", o.result);

    const int16_t over[VOTING_N][VOTING_AXIS_MAX] = { {0, 0}, {VOTING_TOL_ACC + 1, 0}, {0, 0} };
    voting_fuse(over, ALL_OK, 2, VOTING_TOL_ACC, &o);
    CHECK(o.result == VOTING_DEGRADED, "하나 넘으면 배제여야 한다 (result=%d)", o.result);
    CHECK(!o.used[1], "1번이 배제되어야 한다");
}

// --- 읽기 실패 ---

static void test_one_dropout(void)
{
    const bool    valid[VOTING_N] = { true, true, false };
    const int16_t v[VOTING_N][VOTING_AXIS_MAX] = { {100, 16000}, {200, 16100}, {0, 0} };
    voting_out_t  o;

    voting_fuse(v, valid, 2, VOTING_TOL_ACC, &o);

    CHECK(o.result == VOTING_DEGRADED, "result=%d", o.result);
    CHECK(o.used[0] && o.used[1] && !o.used[2], "실패한 센서는 값을 보지 않고 배제");
    CHECK(o.val[0] == 150 && o.val[1] == 16050, "평균 %d,%d", o.val[0], o.val[1]);
}

// 둘이 읽기 실패면 남은 하나가 유일한 데이터다. 검증은 못 해도 그 값으로 간다.
static void test_two_dropout_is_single(void)
{
    const bool    valid[VOTING_N] = { true, false, false };
    const int16_t v[VOTING_N][VOTING_AXIS_MAX] = { {100, 16000}, {0, 0}, {0, 0} };
    voting_out_t  o;

    voting_fuse(v, valid, 2, VOTING_TOL_ACC, &o);

    CHECK(o.result == VOTING_SINGLE, "result=%d", o.result);
    CHECK(o.used[0] && !o.used[1] && !o.used[2], "0번만 채택되어야 한다");
    CHECK(o.val[0] == 100 && o.val[1] == 16000, "값 %d,%d", o.val[0], o.val[1]);
}

// 셋 다 읽기 실패. 쓸 값이 없다.
static void test_all_dropout(void)
{
    const bool    valid[VOTING_N] = { false, false, false };
    const int16_t v[VOTING_N][VOTING_AXIS_MAX] = { {0, 0}, {0, 0}, {0, 0} };
    voting_out_t  o;

    voting_fuse(v, valid, 2, VOTING_TOL_ACC, &o);

    CHECK(o.result == VOTING_FAIL, "result=%d", o.result);
}

/*
 * 읽히긴 하는데 값이 어긋나는 경우는 SINGLE 이 아니라 FAIL 이다.
 * 둘 다 살아 있으니 "유일한 데이터" 가 아니고, 누가 맞는지 가릴 근거도 없다.
 */
static void test_two_valid_disagree_is_fail(void)
{
    const bool    valid[VOTING_N] = { true, true, false };
    const int16_t v[VOTING_N][VOTING_AXIS_MAX] = { {0, 0}, {9000, 0}, {0, 0} };
    voting_out_t  o;

    voting_fuse(v, valid, 2, VOTING_TOL_ACC, &o);

    CHECK(o.result == VOTING_FAIL, "result=%d", o.result);
}

// --- 자이로 (축 1개) ---

static void test_gyro_ignores_second_axis(void)
{
    // [1] 은 안 보는 자리다. 쓰레기를 넣어도 판정이 흔들리면 안 된다.
    const int16_t v[VOTING_N][VOTING_AXIS_MAX] = { {10, 30000}, {50, -30000}, {90, 0} };
    voting_out_t  o;

    voting_fuse(v, ALL_OK, 1, VOTING_TOL_GYRO, &o);

    CHECK(o.result == VOTING_OK, "result=%d", o.result);
    CHECK(o.val[0] == 50, "평균 %d", o.val[0]);
    CHECK(o.val[1] == 0, "안 보는 축은 0 이어야 한다 (%d)", o.val[1]);
}

// 2번 자이로만 고장난 상황. 자이로만 배제되고 그 센서의 가속도는 살아야 한다.
static void test_broken_gyro_keeps_its_accel(void)
{
    const int16_t acc[VOTING_N][VOTING_AXIS_MAX] = { {100, 16000}, {200, 16100}, {150, 16050} };
    const int16_t gyr[VOTING_N][VOTING_AXIS_MAX] = { {800, 0}, {850, 0}, {0, 0} };
    voting_out_t  oa, og;

    voting_fuse(acc, ALL_OK, 2, VOTING_TOL_ACC,  &oa);
    voting_fuse(gyr, ALL_OK, 1, VOTING_TOL_GYRO, &og);

    CHECK(oa.result == VOTING_OK, "가속도는 셋 다 채택 (result=%d)", oa.result);
    CHECK(oa.used[2], "고장난 건 자이로뿐이므로 2번 가속도는 살아야 한다");

    CHECK(og.result == VOTING_DEGRADED, "자이로는 하나 배제 (result=%d)", og.result);
    CHECK(!og.used[2], "2번 자이로가 배제되어야 한다");
    CHECK(og.val[0] == 825, "남은 둘 평균 %d", og.val[0]);
}

int main(void)
{
    test_all_agree();
    test_one_outlier();
    test_chain_is_fail();
    test_all_disagree();
    test_tolerance_edge();
    test_one_dropout();
    test_two_dropout_is_single();
    test_all_dropout();
    test_two_valid_disagree_is_fail();
    test_gyro_ignores_second_axis();
    test_broken_gyro_keeps_its_accel();

    printf(fails ? "실패 %d건\n" : "전부 통과\n", fails);
    return fails ? 1 : 0;
}

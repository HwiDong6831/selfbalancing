/*
 * voting 로직 테스트. 하드웨어 없이 호스트에서 돈다.
 *
 *   gcc -Wall -Wextra -I.. test_voting.c ../voting.c -o test_voting && ./test_voting
 */
#include <stdio.h>
#include "voting.h"

// 정상 상태 기준값 (똑바로 선 자세: az 가 중력 1g, 나머지는 0 근처)
#define AY0      0
#define AZ0  16384
#define GX0      0

#define BAD  (VOTING_TOL_ACC * 4)   // 임계값을 확실히 넘는 이상치

static int failed = 0;

static const char *rname(voting_result_t r)
{
    switch (r) {
    case VOTING_OK:       return "OK";
    case VOTING_DEGRADED: return "DEGRADED";
    default:              return "FAIL";
    }
}

static void check(const char *name, int cond)
{
    printf(cond ? "  ok    %s\n" : "  FAIL  %s\n", name);
    if (!cond) failed++;
}

static void expect(const char *name, const voting_sample_t in[3],
                   voting_result_t want, int u0, int u1, int u2)
{
    voting_out_t o;
    voting_fuse(in, &o);

    int ok = (o.result == want) &&
             (o.used[0] == (u0 != 0)) &&
             (o.used[1] == (u1 != 0)) &&
             (o.used[2] == (u2 != 0));

    if (!ok) {
        printf("  FAIL  %s\n", name);
        printf("        기대 %s used[%d %d %d]\n", rname(want), u0, u1, u2);
        printf("        실제 %s used[%d %d %d]  ay=%d az=%d gx=%d\n",
               rname(o.result), o.used[0], o.used[1], o.used[2], o.ay, o.az, o.gx);
        failed++;
    } else {
        printf("  ok    %s\n", name);
    }
}

int main(void)
{
    printf("voting 테스트\n");

    // 1. 전부 정상 — 약간의 편차는 허용 범위 안
    {
        voting_sample_t in[3] = {
            {AY0 + 10, AZ0 - 30, GX0 + 5,  true},
            {AY0 - 20, AZ0 + 15, GX0 - 8,  true},
            {AY0 + 5,  AZ0 + 40, GX0 + 2,  true},
        };
        expect("정상 3개 → OK", in, VOTING_OK, 1, 1, 1);
    }

    // 2. 읽기 실패 1개 — 나머지 둘이 일치하면 그 둘로 간다
    {
        voting_sample_t in[3] = {
            {AY0, AZ0, GX0, true},
            {0,   0,   0,   false},
            {AY0, AZ0, GX0, true},
        };
        expect("dropout 1개 → DEGRADED", in, VOTING_DEGRADED, 1, 0, 1);
    }

    // 3. 값 이상 1개 — dropout 과 결과가 같다(구분은 호출자의 fault 몫)
    {
        voting_sample_t in[3] = {
            {AY0, AZ0, GX0, true},
            {AY0, AZ0 - BAD, GX0, true},
            {AY0, AZ0, GX0, true},
        };
        expect("가속도 이상치 1개 → DEGRADED", in, VOTING_DEGRADED, 1, 0, 1);
    }

    // 4. 가속도는 멀쩡한데 자이로만 틀림 — 축 하나만 어긋나도 그 센서는 못 믿는다
    {
        voting_sample_t in[3] = {
            {AY0, AZ0, GX0, true},
            {AY0, AZ0, GX0, true},
            {AY0, AZ0, GX0 + VOTING_TOL_GYRO * 4, true},
        };
        expect("자이로만 이상치 → DEGRADED", in, VOTING_DEGRADED, 1, 1, 0);
    }

    // 5. 셋이 제각각 — 일치하는 짝이 없다
    {
        voting_sample_t in[3] = {
            {AY0,       AZ0, GX0, true},
            {AY0 + BAD, AZ0, GX0, true},
            {AY0 - BAD, AZ0, GX0, true},
        };
        expect("셋 다 어긋남 → FAIL", in, VOTING_FAIL, 0, 0, 0);
    }

    // 6. 사슬 — a~b, b~c 인데 a~c 는 아니다. 어느 짝이 맞는지 가릴 수 없다
    {
        const int16_t step = VOTING_TOL_ACC - 100;
        voting_sample_t in[3] = {
            {AY0,            AZ0, GX0, true},
            {AY0 + step,     AZ0, GX0, true},
            {AY0 + step * 2, AZ0, GX0, true},
        };
        expect("사슬(짝 2개) → FAIL", in, VOTING_FAIL, 0, 0, 0);
    }

    // 7. 유효 1개 — 대조할 상대가 없다
    {
        voting_sample_t in[3] = {
            {AY0, AZ0, GX0, true},
            {0, 0, 0, false},
            {0, 0, 0, false},
        };
        expect("유효 1개 → FAIL", in, VOTING_FAIL, 0, 0, 0);
    }

    // 8. 유효 2개, 서로 일치
    {
        voting_sample_t in[3] = {
            {AY0 + 50, AZ0, GX0, true},
            {AY0 - 50, AZ0, GX0, true},
            {0, 0, 0, false},
        };
        expect("유효 2개 일치 → DEGRADED", in, VOTING_DEGRADED, 1, 1, 0);
    }

    // 9. 유효 2개, 불일치 — 누가 틀렸는지 못 가린다
    {
        voting_sample_t in[3] = {
            {AY0, AZ0, GX0, true},
            {AY0, AZ0 - BAD, GX0, true},
            {0, 0, 0, false},
        };
        expect("유효 2개 불일치 → FAIL", in, VOTING_FAIL, 0, 0, 0);
    }

    // 10. 전부 실패
    {
        voting_sample_t in[3] = {
            {0, 0, 0, false}, {0, 0, 0, false}, {0, 0, 0, false},
        };
        expect("전부 dropout → FAIL", in, VOTING_FAIL, 0, 0, 0);
    }

    // 11. 임계값 경계 — 딱 임계값이면 일치, 1 넘으면 불일치
    {
        voting_sample_t in[3] = {
            {AY0, AZ0, GX0, true},
            {AY0, AZ0, GX0, true},
            {AY0, AZ0 + VOTING_TOL_ACC, GX0, true},
        };
        expect("경계값(=임계값) → OK", in, VOTING_OK, 1, 1, 1);

        in[2].az = AZ0 + VOTING_TOL_ACC + 1;
        expect("경계값+1 → 배제", in, VOTING_DEGRADED, 1, 1, 0);
    }

    // 12. FAIL 이면 값을 채우지 않는다 — 호출자가 직전 값을 유지해야 한다
    {
        voting_sample_t in[3] = {
            {0, 0, 0, false}, {0, 0, 0, false}, {0, 0, 0, false},
        };
        voting_out_t o;
        voting_fuse(in, &o);
        check("FAIL 이면 값 0", o.ay == 0 && o.az == 0 && o.gx == 0);
    }

    // 13. 융합값이 채택된 것들의 평균인가
    {
        voting_sample_t in[3] = {
            {100, AZ0, GX0, true},
            {200, AZ0, GX0, true},
            {300, AZ0, GX0, true},
        };
        voting_out_t o;
        voting_fuse(in, &o);
        check("정상 3개 평균", o.result == VOTING_OK && o.ay == 200);

        in[2].valid = false;
        voting_fuse(in, &o);
        check("배제 후 나머지 평균", o.result == VOTING_DEGRADED && o.ay == 150);
    }

    printf(failed ? "\n실패 %d 건\n" : "\n전부 통과\n", failed);
    return failed ? 1 : 0;
}

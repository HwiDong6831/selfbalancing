#include "voting.h"

#define TOL_ACC   VOTING_TOL_ACC
#define TOL_GYRO  VOTING_TOL_GYRO

static int16_t absdiff(int16_t a, int16_t b)
{
    int32_t d = (int32_t)a - b;
    return (int16_t)(d < 0 ? -d : d);
}

// 두 센서가 같은 것을 보고 있는가. 읽기 실패한 센서는 누구와도 일치하지 않는다.
static bool agree(const voting_sample_t *a, const voting_sample_t *b)
{
    if (!a->valid || !b->valid) return false;

    return absdiff(a->ay, b->ay) <= TOL_ACC &&
           absdiff(a->az, b->az) <= TOL_ACC &&
           absdiff(a->gx, b->gx) <= TOL_GYRO;
}

static void mean_of_used(const voting_sample_t in[VOTING_N], voting_out_t *out)
{
    int32_t ay = 0, az = 0, gx = 0;
    int     n  = 0;

    for (int i = 0; i < VOTING_N; i++) {
        if (!out->used[i]) continue;
        ay += in[i].ay;
        az += in[i].az;
        gx += in[i].gx;
        n++;
    }
    if (n == 0) return;

    out->ay = (int16_t)(ay / n);
    out->az = (int16_t)(az / n);
    out->gx = (int16_t)(gx / n);
}

void voting_fuse(const voting_sample_t in[VOTING_N], voting_out_t *out)
{
    *out = (voting_out_t){0};

    bool ab = agree(&in[0], &in[1]);
    bool ac = agree(&in[0], &in[2]);
    bool bc = agree(&in[1], &in[2]);
    int  pairs = (int)ab + (int)ac + (int)bc;

    if (pairs == 3) {
        out->used[0] = out->used[1] = out->used[2] = true;
        mean_of_used(in, out);
        out->result = VOTING_OK;
        return;
    }

    // 짝이 둘이면 a~b, b~c 인데 a~c 는 아닌 사슬이다. 어느 짝이 맞는지 가릴 근거가 없다.
    if (pairs == 1) {
        if      (ab) out->used[0] = out->used[1] = true;
        else if (ac) out->used[0] = out->used[2] = true;
        else         out->used[1] = out->used[2] = true;
        mean_of_used(in, out);
        out->result = VOTING_DEGRADED;
        return;
    }

    out->result = VOTING_FAIL;   // 값은 0. 호출자가 직전 값을 유지한다
}

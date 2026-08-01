#include "voting.h"

// 두 센서가 같은 것을 보고 있는가. 축 하나라도 임계값을 넘으면 불일치다.
static bool agree(const int16_t a[VOTING_AXIS_MAX], const int16_t b[VOTING_AXIS_MAX],
                  bool valid_a, bool valid_b, int axes, int16_t tol)
{
    if (!valid_a || !valid_b) return false;

    for (int k = 0; k < axes; k++) {
        int32_t d = (int32_t)a[k] - b[k];
        if (d < 0) d = -d;
        if (d > tol) return false;
    }
    return true;
}

void voting_fuse(const int16_t v[VOTING_N][VOTING_AXIS_MAX], const bool valid[VOTING_N],
                 int axes, int16_t tol, voting_out_t *out)
{
    *out = (voting_out_t){0};

    bool ab = agree(v[0], v[1], valid[0], valid[1], axes, tol);
    bool ac = agree(v[0], v[2], valid[0], valid[2], axes, tol);
    bool bc = agree(v[1], v[2], valid[1], valid[2], axes, tol);
    int  pairs = (int)ab + (int)ac + (int)bc;

    if (pairs == 3) {
        out->used[0] = out->used[1] = out->used[2] = true;
        out->result  = VOTING_OK;
    } else if (pairs == 1) {
        if      (ab) out->used[0] = out->used[1] = true;
        else if (ac) out->used[0] = out->used[2] = true;
        else         out->used[1] = out->used[2] = true;
        out->result = VOTING_DEGRADED;
    } else {
        out->result = VOTING_FAIL;   // 짝이 0 또는 2개. 누가 맞는지 가릴 수 없다
        return;
    }

    int32_t sum[VOTING_AXIS_MAX] = {0};
    int     n = 0;
    for (int i = 0; i < VOTING_N; i++) {
        if (!out->used[i]) continue;
        for (int k = 0; k < axes; k++) sum[k] += v[i][k];
        n++;
    }
    for (int k = 0; k < axes; k++) out->val[k] = (int16_t)(sum[k] / n);
}

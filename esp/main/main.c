/*
 * 3단계 — 밸런싱.
 *
 * MPU 3개를 voting(서로 비교해 이상한 값을 골라내는 다수결)으로 융합해 기울기를 추정하고
 * q축 전압을 만들어, 엔코더 폐루프 커뮤테이션으로 모터에 인가한다.
 * 제어는 별도 태스크에서 돌고 app_main 은 로그만 찍는다.
 */
#include <math.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/i2c.h"

#include "encoder.h"
#include "mpu6050.h"
#include "foc.h"
#include "balance.h"
#include "voting.h"
#include "telemetry.h"
#include "secrets.h"

#define I2C_PORT       I2C_NUM_0
#define PIN_SDA        16
#define PIN_SCL        17
// 100kHz 는 드라이버 이관(2026-07-21) 때 딸려온 값이고 속도는 버스 멈춤과 무관했다.
// 센서 3개를 읽는 지금은 버스에 신호가 떠 있는 시간을 줄이는 편이 낫다. 부품 3종 모두 지원.
#define I2C_FREQ_HZ    400000

// 고정 주기 8ms = 125Hz. 400kHz 로 올린 뒤 그냥 두면 247Hz 까지 올라가는데, mux 전환
// 횟수가 초당 2배가 되고 게인도 115Hz 기준으로 맞춘 값이라 주기를 고정한다.
#define LOOP_PERIOD_MS 8
#define VEL_LPF        0.2f

/*
 * 0 = 계측만. 드라이버 EN 을 끄고 정렬도 건너뛴다. 제어 루프는 그대로 돌며 uq 를 찍는다.
 * 별도 계측 루프를 두지 않는 이유는, 그러면 실제로 돌릴 코드와 다른 코드를 검증하게 되기
 * 때문이다. 게인·부호를 손대고 모터를 물리기 전에 0 으로 두고 로그부터 확인한다.
 */
#define DRIVE_MOTOR    1

// 0 = WiFi/텔레메트리를 아예 시작하지 않는다. 시리얼 로그만 남는다.
#define USE_TELEMETRY  1

// 대시보드 송신 간격 (50Hz). 루프마다 보내면 WiFi 송신 전류가 센서 전원을 흔든다.
#define TELEMETRY_US   20000

#define GYRO_CAL_N     200         // gz bias 평균 샘플 수 (5ms 간격 = 1초)
#define SETPOINT_MS    2000        // 똑바로 세운 채 상보필터를 수렴시키는 시간

#define LAG_COMP       1.0f        // 커뮤테이션 지연 보상. 1.0 = 한 루프치, 0.0 = 보상 없음

#define ALIGN_UD_V     2.0f
#define ALIGN_HOLD_MS  1000        // 회전자를 전기각 0 으로 끌어오는 대기
#define ALIGN_SWEEP_MS 1000        // 한 방향 스윕 시간
#define ALIGN_STEPS    200         // 스텝당 전기각 1.8도
#define ALIGN_SAMPLES  16          // 끝점 각도 평균 샘플 수

/*
 * 엔코더 손상 판정 상한 = ENC_MAX_VEL × 경과시간 + ENC_BAD_FLOOR [rad].
 *
 * 휠 속도 추정치를 쓰면 안 된다. 버린 동안 추정치가 얼어붙는데, 그 얼어붙은 값이 다시
 * 상한을 좁혀 정상값까지 버리게 된다. 한번 잠기면 안 풀린다 (2026-07-29 실측: 정상값을
 * 13루프 연속 버리고 커뮤테이션이 멈춰 급가속). 고정값이어야 시간이 갈수록 상한이 넓어져
 * 스스로 풀린다.
 * 실측 최고가 55 rad/s 이므로 80 을 상한으로 두면 정상값은 걸리지 않는다.
 */
#define ENC_MAX_VEL    80.0f
#define ENC_BAD_FLOOR  0.30f

/*
 * 휠 각가속도 상한 [rad/s^2]. 손상값이 상한을 통과해도 휠 속도가 한 번에 튀지 못하게 막는다.
 * 버리는 게 아니라 깎는 것이라 커뮤테이션이 멈추지 않는다.
 * 실측 최대 약 120 이므로 300 이면 정상 가속은 걸리지 않는다.
 */
#define WHEEL_MAX_ACC  300.0f

#define STAT_US        100000      // 통계 창. 0.7초짜리 넘어짐을 보려면 10Hz 는 되어야 한다

// 로그 출력(190자 @115200 = 16.5ms)이 커뮤테이션을 멈추지 않도록 제어를 별도 태스크로
// 뺀다. 우선순위가 높으면 로그 태스크가 UART 를 기다리는 중이어도 즉시 뺏어온다.
#define CTRL_PRIO      10
#define CTRL_STACK     4096
#define CTRL_CORE      0

static const char *TAG = "MAIN";

// 셋을 전부 제어에 쓴다. voting 이 서로를 검증해 이상한 값을 골라낸다.
#define NUM_SENSORS    VOTING_N
static const int MUX_CHANNELS[NUM_SENSORS] = {5, 6, 7};

// ACCEL_CONFIG / GYRO_CONFIG 를 안 건드리므로 리셋 기본값 ±2g, ±250도/s.
#define ACC_LSB_PER_G     16384.0f
#define GYRO_LSB_PER_DPS  131.0f

// 1초치 계측. 루프 주기가 바뀌면 "초당 몇 회" 는 비교가 안 되므로 비율로 본다.
typedef struct {
    uint32_t loops;
    uint32_t mpu_fail;
    uint32_t enc_bad;       // 물리적으로 불가능해 버린 각도 변화
    float    d_max;         // 최대 각도 변화 [rad]
    float    d_max_span;    // 그때 직전 채택값 이후 경과 시간 [s]
} stats_t;

typedef struct {
    stats_t st;
    int64_t win_us;     // 이 창의 실제 길이. loop Hz 를 정확히 내려면 필요하다
    float   wheel_vel;
    float   angle;      // 기울기 [deg], 0 = 똑바로
    float   rate;       // 기울기 각속도 [deg/s]
    float   uq;         // balance_torque 가 낸 q축 전압 [V]
} stats_msg_t;

static QueueHandle_t s_stats_q;   // 길이 1. 제어 루프는 덮어쓰기만 하고 대기하지 않는다

// 채널별 영점 [LSB]. 부팅 때 한 번 재고 안 바뀐다.
static float s_gz0[NUM_SENSORS];
static float s_a0[NUM_SENSORS][2];

static void i2c_bus_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = PIN_SDA,
        .scl_io_num = PIN_SCL,
        .sda_pullup_en = true,
        .scl_pullup_en = true,
        .master.clk_speed = I2C_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_PORT, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_PORT, I2C_MODE_MASTER, 0, 0, 0));
}

// 센서 값은 ±2g·±250도/s 끝에서 int16 한계에 닿는다. 영점을 빼거나 drift 를 더하면
// 그 범위를 넘어가 부호가 뒤집히므로 잘라 준다.
static int16_t clamp16(float v)
{
    if (v >  32767.0f) return  32767;
    if (v < -32768.0f) return -32768;
    return (int16_t)v;
}

// 엔코더 각도 차이. 0/2π 경계를 넘어도 되도록 최단 방향으로 접는다.
static float angle_delta(float now, float prev)
{
    float d = now - prev;
    if (d >  (float)M_PI) d -= 2.0f * (float)M_PI;
    if (d < -(float)M_PI) d += 2.0f * (float)M_PI;
    return d;
}

static void stats_add(stats_t *st, float d, float span)
{
    if (fabsf(d) > st->d_max) {
        st->d_max      = fabsf(d);
        st->d_max_span = span;
    }
}

/*
 * 물리적으로 불가능한 각도 변화인가. span 은 마지막으로 채택한 값 이후 경과 시간이다.
 *
 * 루프 주기가 아니라 경과 시간을 쓰는 이유: 버린 뒤에는 다음 값이 두 루프치를 건너뛰므로,
 * 한 루프 예산으로 재면 정상값도 걸려 연속 배제로 이어진다.
 */
static bool enc_implausible(float d, float span)
{
    return fabsf(d) > ENC_MAX_VEL * span + ENC_BAD_FLOOR;
}

/*
 * 최대 각도 변화가 그 순간 물리적으로 가능한 값(dt × 휠속도)의 몇 배인지 같이 찍는다.
 * 1.0 근처면 정상이고, 크게 넘으면 엔코더가 틀린 값을 준 것이다.
 */
static void stats_print(const stats_msg_t *m)
{
    const stats_t *st = &m->st;
    float d_max_deg = st->d_max * 180.0f / (float)M_PI;
    float explain   = fabsf(m->wheel_vel) * st->d_max_span * 180.0f / (float)M_PI;

    unsigned long hz = m->win_us ? (unsigned long)(st->loops * 1000000LL / m->win_us) : 0;

    // MPU 는 한 루프에 NUM_SENSORS 번 읽으므로 실패율 분모도 그만큼이다.
    uint32_t reads = st->loops * NUM_SENSORS;

    // Z 카운트가 한 자리에 머물러야 한 바퀴 카운트가 맞는 것이다 (encoder.h 참조).
    uint32_t z_n;
    int      z_last;
    encoder_get_z(&z_n, &z_last);

    ESP_LOGI(TAG, "loop %luHz  각도 %6.1f도  각속도 %7.1f도/s  uq %6.2fV  "
                  "휠 %6.1f  MPU실패 %lu(%.2f%%)  손상버림 %lu  최대변화 %.1f도(%.1f배)  "
                  "카운트 %5d  Z %lu회(카운트 %d)",
             hz, m->angle, m->rate, m->uq, m->wheel_vel,
             (unsigned long)st->mpu_fail,
             reads ? 100.0f * st->mpu_fail / reads : 0.0f,
             (unsigned long)st->enc_bad,
             d_max_deg, (explain > 0.01f) ? d_max_deg / explain : 0.0f,
             encoder_get_count(), (unsigned long)z_n, z_last);
}

#if DRIVE_MOTOR
// 엔코더 각도 평균. 0/2π 경계를 넘어도 되도록 단위벡터로 더한다.
static float read_angle_avg(void)
{
    float sum_s = 0.0f, sum_c = 0.0f;

    for (int i = 0; i < ALIGN_SAMPLES; i++) {
        float a;
        if (encoder_read_angle(&a) == ESP_OK) {
            sum_s += sinf(a);
            sum_c += cosf(a);
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    return atan2f(sum_s, sum_c);
}

/*
 * 전기각을 0 → 2π → 0 으로 왕복시키며 양 끝에서 기계각을 잰다.
 * 가만히 끌어당기기만 하면 정지마찰 때문에 회전자가 끝까지 안 간다. 끌고 지나가야 한다.
 */
static float align_rotor(void)
{
    const float step     = 2.0f * (float)M_PI / ALIGN_STEPS;
    const int   dwell_ms = ALIGN_SWEEP_MS / ALIGN_STEPS;

    foc_set_phase_voltage(ALIGN_UD_V, 0.0f, 0.0f);
    vTaskDelay(pdMS_TO_TICKS(ALIGN_HOLD_MS));

    for (int i = 1; i <= ALIGN_STEPS; i++) {
        foc_set_phase_voltage(ALIGN_UD_V, 0.0f, i * step);
        vTaskDelay(pdMS_TO_TICKS(dwell_ms));
    }
    float angle_fwd = read_angle_avg();

    for (int i = ALIGN_STEPS - 1; i >= 0; i--) {
        foc_set_phase_voltage(ALIGN_UD_V, 0.0f, i * step);
        vTaskDelay(pdMS_TO_TICKS(dwell_ms));
    }
    float angle_rev = read_angle_avg();

    foc_set_phase_voltage(0.0f, 0.0f, 0.0f);   // 물고 있으면 DC 가 계속 흐른다

    return foc_align(angle_fwd, angle_rev);
}
#endif  // DRIVE_MOTOR

// 채널별 영점. 자이로는 평균을 통째로, 가속도는 중앙값 대비 차이만 뺀다 (2026-08-02 일지).
static void measure_cal(void)
{
    int32_t sg[NUM_SENSORS] = {0}, sa[NUM_SENSORS][2] = {{0}};
    int     n[NUM_SENSORS]  = {0};
    float   mean_a[NUM_SENSORS][2];
    int16_t ax, ay, az, gx, gy, gz;

    for (int i = 0; i < GYRO_CAL_N; i++) {
        for (int c = 0; c < NUM_SENSORS; c++) {
            if (mpu6050_read_accel_gyro(MUX_CHANNELS[c], &ax, &ay, &az,
                                        &gx, &gy, &gz) == ESP_OK) {
                sg[c]    += gz;
                sa[c][0] += ax;
                sa[c][1] += ay;
                n[c]++;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    for (int c = 0; c < NUM_SENSORS; c++) {
        s_gz0[c]     = n[c] ? (float)sg[c]    / n[c] : 0.0f;
        mean_a[c][0] = n[c] ? (float)sa[c][0] / n[c] : 0.0f;
        mean_a[c][1] = n[c] ? (float)sa[c][1] / n[c] : 0.0f;
    }

    // 셋뿐이라 정렬 없이 비교로 중앙값을 고른다.
    for (int k = 0; k < 2; k++) {
        float x = mean_a[0][k], y = mean_a[1][k], z = mean_a[2][k];
        float med = (x < y) ? ((y < z) ? y : ((x < z) ? z : x))
                            : ((x < z) ? x : ((y < z) ? z : y));
        for (int c = 0; c < NUM_SENSORS; c++) s_a0[c][k] = mean_a[c][k] - med;
    }
}

// 조립 오차 때문에 "진짜 똑바로" 가 센서상 0 이 아니다. 세워 둔 채 수렴시켜 기준을 잡는다.
// 제어 루프와 같은 기준을 쓰도록 여기서도 영점을 뺀다.
static float measure_setpoint(void)
{
    int64_t t0 = esp_timer_get_time(), prev = t0;
    float   angle = 0.0f, rate;
    int16_t ax, ay, az, gx, gy, gz;

    while (esp_timer_get_time() - t0 < (int64_t)SETPOINT_MS * 1000) {
        int64_t now = esp_timer_get_time();
        float   dt  = (now - prev) * 1e-6f;
        prev = now;

        if (mpu6050_read_accel_gyro(MUX_CHANNELS[0], &ax, &ay, &az,
                                    &gx, &gy, &gz) == ESP_OK) {
            angle = balance_estimate_angle((int16_t)(ax - s_a0[0][0]), (int16_t)(ay - s_a0[0][1]),
                                           gz, s_gz0[0], dt, &rate);
        }
        vTaskDelay(pdMS_TO_TICKS(LOOP_PERIOD_MS));
    }
    return angle;
}

// 대시보드로 한 프레임 보낸다. telemetry_publish 는 논블로킹이라 제어 루프에서 불러도 된다.
static void publish_frame(const int16_t a[][3], const int16_t g[][3],
                          const telemetry_fault_t *fault,
                          const voting_out_t *va, const voting_out_t *vg,
                          float tilt, float rate, float setpoint,
                          float uq, float enc_rad)
{
    telemetry_frame_t f = {
        .angle     = tilt,
        .rate      = rate,
        .setpoint  = setpoint,
        .uq        = uq,
        .enc_angle = enc_rad * (180.0f / (float)M_PI),
        .tol_accel = VOTING_TOL_ACC / ACC_LSB_PER_G,
        .tol_gyro  = VOTING_TOL_GYRO / GYRO_LSB_PER_DPS,
    };

    // voting_result_t 와 telemetry_vote_t 는 같은 순서라 그대로 옮긴다.
    f.accel.result = (telemetry_vote_t)va->result;
    f.gyro.result  = (telemetry_vote_t)vg->result;
    f.accel.val[0] = va->val[0] / ACC_LSB_PER_G;
    f.accel.val[1] = va->val[1] / ACC_LSB_PER_G;
    f.gyro.val[0]  = vg->val[0] / GYRO_LSB_PER_DPS;
    for (int i = 0; i < NUM_SENSORS; i++) {
        f.accel.used[i] = va->used[i];
        f.gyro.used[i]  = vg->used[i];
    }

    // ax/ay/gz 는 voting 이 실제로 비교한 값(영점 뺀 값)을 보낸다. 화면과 판정 근거를
    // 같게 두려는 것이다. az/gx/gy 는 제어·voting 에 안 쓰여 원값 그대로다.
    for (int i = 0; i < NUM_SENSORS; i++) {
        f.sensors[i].ch    = MUX_CHANNELS[i];
        f.sensors[i].fault = fault[i];
        f.sensors[i].ax = (a[i][0] - s_a0[i][0]) / ACC_LSB_PER_G;
        f.sensors[i].ay = (a[i][1] - s_a0[i][1]) / ACC_LSB_PER_G;
        f.sensors[i].az =  a[i][2] / ACC_LSB_PER_G;
        f.sensors[i].gx =  g[i][0] / GYRO_LSB_PER_DPS;
        f.sensors[i].gy =  g[i][1] / GYRO_LSB_PER_DPS;
        f.sensors[i].gz = (g[i][2] - s_gz0[i]) / GYRO_LSB_PER_DPS;
        f.sensors[i].ax0 = s_a0[i][0] / ACC_LSB_PER_G;
        f.sensors[i].ay0 = s_a0[i][1] / ACC_LSB_PER_G;
        f.sensors[i].gz0 = s_gz0[i]   / GYRO_LSB_PER_DPS;
    }
    telemetry_publish(&f);
}

static void balance_loop(float offset, float setpoint)
{
    int64_t now       = esp_timer_get_time();
    int64_t prev_time = now, last_stat = now, last_pub = now;

    stats_t st = {0};
    float   wheel_vel = 0.0f, prev_angle = 0.0f;
    float   since_ok  = 0.0f;   // 마지막으로 채택한 엔코더 값 이후 경과 시간 [s]
    float   tilt = 0.0f, rate = 0.0f, uq = 0.0f;
    int16_t a[NUM_SENSORS][3], g[NUM_SENSORS][3];   // [채널][x,y,z]
    telemetry_fault_t fault[NUM_SENSORS] = {0};

    // 영점을 뺀 비교용 값. 제어에 쓰는 축만 담는다 (가속도 ax,ay / 자이로 gz).
    int16_t av[NUM_SENSORS][VOTING_AXIS_MAX], gv[NUM_SENSORS][VOTING_AXIS_MAX] = {{0}};
    bool    valid[NUM_SENSORS];
    voting_out_t va = {0}, vg = {0};

    // 결함 주입용. held 는 freeze 가 되돌려 줄 마지막 정상값, drift 는 누적 벌어짐이다.
    int16_t held_a[NUM_SENSORS][3] = {{0}}, held_g[NUM_SENSORS][3] = {{0}};
    float   drift[NUM_SENSORS] = {0};

    encoder_read_angle(&prev_angle);

    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
        now = esp_timer_get_time();
        int64_t dt_us = now - prev_time;
        float   dt    = dt_us * 1e-6f;
        prev_time = now;
        st.loops++;

        // 셋을 다 읽는다. 한 번에 약 1ms 씩 든다.
        // 실패 1회는 I2C 타임아웃 10ms 라 반드시 세어야 한다.
        for (int i = 0; i < NUM_SENSORS; i++) {
            // 결함 주입. 실제 결함과 같은 경로를 타도록 원본 값을 덮는다.
            float inj_rate;
            telemetry_fault_t inj = telemetry_get_inject(MUX_CHANNELS[i], &inj_rate);
            drift[i] = (inj == TELEMETRY_FAULT_DRIFT) ? drift[i] + inj_rate * dt : 0.0f;

            if (mpu6050_read_accel_gyro(MUX_CHANNELS[i],
                                        &a[i][0], &a[i][1], &a[i][2],
                                        &g[i][0], &g[i][1], &g[i][2]) != ESP_OK) {
                st.mpu_fail++;
                fault[i] = TELEMETRY_FAULT_DROPOUT;
                a[i][0] = a[i][1] = a[i][2] = 0;
                g[i][0] = g[i][1] = g[i][2] = 0;
            } else {
                fault[i] = TELEMETRY_FAULT_NONE;
                // 주입 중에 갱신하면 freeze 가 매 루프 현재값을 되돌려 줘 아무 효과가 없다.
                if (inj == TELEMETRY_FAULT_NONE) {
                    memcpy(held_a[i], a[i], sizeof a[i]);
                    memcpy(held_g[i], g[i], sizeof g[i]);
                }
            }

            if (inj == TELEMETRY_FAULT_DROPOUT) {
                a[i][0] = a[i][1] = a[i][2] = 0;
                g[i][0] = g[i][1] = g[i][2] = 0;
            } else if (inj == TELEMETRY_FAULT_FREEZE) {
                memcpy(a[i], held_a[i], sizeof a[i]);
                memcpy(g[i], held_g[i], sizeof g[i]);
            } else if (inj == TELEMETRY_FAULT_DRIFT) {
                // 임계값 배수로 받아 신호마다 제 임계값에 맞춰 벌린다. 둘이 같이 걸린다.
                a[i][0] = clamp16(a[i][0] + drift[i] * VOTING_TOL_ACC);
                a[i][1] = clamp16(a[i][1] + drift[i] * VOTING_TOL_ACC);
                g[i][2] = clamp16(g[i][2] + drift[i] * VOTING_TOL_GYRO);
            }
            if (inj != TELEMETRY_FAULT_NONE) fault[i] = inj;

            valid[i] = (fault[i] != TELEMETRY_FAULT_DROPOUT);

            av[i][0] = clamp16(a[i][0] - s_a0[i][0]);
            av[i][1] = clamp16(a[i][1] - s_a0[i][1]);
            gv[i][0] = clamp16(g[i][2] - s_gz0[i]);
        }

        // 가속도쌍과 자이로를 따로 투표한다 (이유는 voting.h).
        voting_fuse(av, valid, 2, VOTING_TOL_ACC,  &va);
        voting_fuse(gv, valid, 1, VOTING_TOL_GYRO, &vg);

        // FAIL 이면 갱신하지 않고 직전 명령을 잇는다. 값 0 을 넣으면 "똑바로" 로 읽혀 넘어진다.
        if (va.result != VOTING_FAIL && vg.result != VOTING_FAIL) {
            // 영점은 채널별로 이미 뺐으므로 bias 는 0 이다.
            tilt = balance_estimate_angle(va.val[0], va.val[1], vg.val[0],
                                          0.0f, dt, &rate) - setpoint;
            uq   = balance_torque(tilt, rate, wheel_vel);
        }

        // PCNT 는 하드웨어가 세므로 읽기 실패가 없다 (I2C 시절의 실패 경로는 사라졌다).
        float angle;
        encoder_read_angle(&angle);
        since_ok += dt;

        float d = angle_delta(angle, prev_angle);
        stats_add(&st, d, since_ok);

        /*
         * ABZ 로 옮긴 뒤로는 걸리지 않는다. 남겨두는 것은 이 값이 계속 0 이라는 것 자체가
         * 전환이 성공했다는 증거이기 때문이다 (2026-08-02 일지).
         *
         * 걸리면 버리고 직전 전압 벡터를 유지한다. prev_angle 을 안 옮기므로 다음
         * 정상값이 그대로 이어진다. 로그는 찍지 않는다 — UART 블로킹이 커뮤테이션을
         * 더 멈춘다.
         */
        if (enc_implausible(d, since_ok)) {
            st.enc_bad++;
        } else {
            // 물리적으로 가능한 가속 범위로 측정값을 깎은 뒤 필터에 넣는다.
            float meas = d / since_ok;
            float step = WHEEL_MAX_ACC * since_ok;
            if (meas > wheel_vel + step) meas = wheel_vel + step;
            if (meas < wheel_vel - step) meas = wheel_vel - step;

            wheel_vel  = (1.0f - VEL_LPF) * wheel_vel + VEL_LPF * meas;
            prev_angle = angle;
            since_ok   = 0.0f;

            // 읽은 위치가 아니라 다음 갱신 시점의 예상 위치에 자계를 세운다.
            // 상한은 wheel_vel 이 튀어도 벡터가 날아가지 않게 하려는 것이다.
            float adv     = LAG_COMP * wheel_vel * dt;
            float adv_max = ENC_MAX_VEL * dt;
            if (adv >  adv_max) adv =  adv_max;
            if (adv < -adv_max) adv = -adv_max;

            // DRIVE_MOTOR 0 이면 EN 이 꺼져 있어 duty 를 써도 출력이 나가지 않는다.
            foc_apply_torque(uq, angle + adv, offset);
        }

        if (now - last_pub >= TELEMETRY_US) {
            last_pub = now;
            publish_frame(a, g, fault, &va, &vg, tilt, rate, setpoint, uq, prev_angle);
        }

        if (now - last_stat >= STAT_US) {
            stats_msg_t msg = { .st = st, .win_us = now - last_stat,
                                .wheel_vel = wheel_vel,
                                .angle = tilt, .rate = rate, .uq = uq };
            last_stat = now;
            xQueueOverwrite(s_stats_q, &msg);
            st = (stats_t){0};
        }

        // 일한 시간과 무관하게 주기를 일정하게 유지한다 (vTaskDelay 면 일감만큼 밀린다)
        xTaskDelayUntil(&last_wake, pdMS_TO_TICKS(LOOP_PERIOD_MS));
    }
}

static void control_task(void *arg)
{
    ESP_LOGI(TAG, "센서 영점 측정 중 — 가만히 두세요");
    measure_cal();

    for (int c = 0; c < NUM_SENSORS; c++) {
        ESP_LOGI(TAG, "ch%d 영점  자이로 %7.1f  가속도 %7.1f / %7.1f LSB",
                 MUX_CHANNELS[c], s_gz0[c], s_a0[c][0], s_a0[c][1]);
    }

    ESP_LOGI(TAG, "똑바로 세워 잡으세요 (%d ms)", SETPOINT_MS);
    float setpoint = measure_setpoint();

#if DRIVE_MOTOR
    float offset = align_rotor();
#else
    float offset = 0.0f;   // 모터를 안 돌리므로 커뮤테이션 정렬이 필요 없다
#endif

    ESP_LOGI(TAG, "기준 각도 %.1f도, 구동 %s", setpoint, DRIVE_MOTOR ? "ON" : "OFF(계측만)");
    balance_loop(offset, setpoint);
}

// app_main(우선순위 1)이 그대로 로그 태스크가 된다. 제어보다 낮으므로 UART 를
// 기다리는 중에도 제어 루프가 언제든 뺏어간다.
void app_main(void)
{
#if USE_TELEMETRY
    // 가장 먼저. 이후의 초기화 로그(자이로 영점, 정렬 오프셋)까지 웹으로 실린다.
    // WiFi 연결까지 블록하므로 공유기가 없으면 여기서 멈춘다.
    static const telemetry_config_t tele_cfg = {
        .wifi_ssid     = WIFI_SSID,
        .wifi_password = WIFI_PASSWORD,
        .server_uri    = SERVER_WS_URI,
    };
    ESP_ERROR_CHECK(telemetry_start(&tele_cfg));
#endif

    i2c_bus_init();
    ESP_ERROR_CHECK(encoder_init());
    ESP_ERROR_CHECK(mpu6050_init(I2C_PORT, MUX_CHANNELS, NUM_SENSORS));

    foc_init();
    foc_enable(DRIVE_MOTOR);

    s_stats_q = xQueueCreate(1, sizeof(stats_msg_t));
    ESP_ERROR_CHECK(s_stats_q ? ESP_OK : ESP_ERR_NO_MEM);

    xTaskCreatePinnedToCore(control_task, "control",
                            CTRL_STACK, NULL, CTRL_PRIO, NULL, CTRL_CORE);

    stats_msg_t msg;
    while (1) {
        if (xQueueReceive(s_stats_q, &msg, portMAX_DELAY) == pdTRUE) {
            stats_print(&msg);
        }
    }
}

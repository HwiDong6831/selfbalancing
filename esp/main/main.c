/*
 * 3단계 — 밸런싱.
 *
 * MPU 3개를 voting 으로 융합해 기울기를 추정하고, 그 q축 전압을 엔코더 폐루프
 * 커뮤테이션으로 모터에 인가한다. 제어는 별도 태스크에서 돌고 app_main 은 로그만 찍는다.
 */
#include <math.h>
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
#include "telemetry.h"
#include "voting.h"
#include "secrets.h"

#define I2C_PORT       I2C_NUM_0
#define PIN_SDA        16
#define PIN_SCL        17
#define I2C_FREQ_HZ    100000

#define LOOP_PERIOD_MS 1           // 여기에 MPU 3개 읽기가 더해져 실제로는 약 165Hz
#define VEL_LPF        0.2f

/*
 * 0 = 계측만. 드라이버 EN 을 끄고 정렬도 건너뛴다. 제어 루프는 그대로 돌며 uq 를 찍는다.
 * 별도 계측 루프를 두지 않는 이유는, 그러면 실제로 돌릴 코드와 다른 코드를 검증하게 되기
 * 때문이다. 게인·부호를 손대고 모터를 물리기 전에 0 으로 두고 로그부터 확인한다.
 */
#define DRIVE_MOTOR    1

/*
 * 1 = 주기 로그를 센서 3개 편차 측정용으로 바꾼다. voting 임계값(TOL_ACC/TOL_GYRO)의
 * 근거를 만들 때 쓴다. 임계값을 다시 조일 일이 있으면 켜서 재면 된다.
 */
#define SENSOR_SURVEY  0

#define CAL_N          200         // 채널별 영점 평균 샘플 수 (5ms 간격 = 1초)
#define SETPOINT_MS    2000        // 똑바로 세운 채 상보필터를 수렴시키는 시간

#define LAG_COMP       1.0f        // 커뮤테이션 지연 보상. 1.0 = 한 루프치, 0.0 = 보상 없음

#define ALIGN_UD_V     2.0f
#define ALIGN_HOLD_MS  1000        // 회전자를 전기각 0 으로 끌어오는 대기
#define ALIGN_SWEEP_MS 1000        // 한 방향 스윕 시간
#define ALIGN_STEPS    200         // 스텝당 전기각 1.8도
#define ALIGN_SAMPLES  16          // 끝점 각도 평균 샘플 수

/*
 * 엔코더 손상 판정 상한 = ENC_MAX_VEL × dt + ENC_BAD_FLOOR [rad].
 *
 * 휠 속도 추정치를 쓰면 안 된다. 걸러내는 로직이 없어 손상값이 그대로 추정치에
 * 들어가고, 오염된 추정치가 다시 상한을 무너뜨린다.
 * 실측 최고가 50 rad/s 이므로 80 을 상한으로 두면 정상값은 걸리지 않는다.
 */
#define ENC_MAX_VEL    80.0f
#define ENC_BAD_FLOOR  0.30f

#define STAT_US        100000      // 통계 창. 0.7초짜리 넘어짐을 보려면 10Hz 는 되어야 한다

// 로그 출력(190자 @115200 = 16.5ms)이 커뮤테이션을 멈추지 않도록 제어를 별도 태스크로
// 뺀다. 우선순위가 높으면 로그 태스크가 UART 를 기다리는 중이어도 즉시 뺏어온다.
#define CTRL_PRIO      10
#define CTRL_STACK     4096
#define CTRL_CORE      0

static const char *TAG = "MAIN";

// 셋을 매 루프 읽어 voting 으로 융합한다. 대시보드에는 셋 다 따로 보낸다.
#define NUM_SENSORS    3
static const int MUX_CHANNELS[NUM_SENSORS] = {0, 1, 6};

// ACCEL_CONFIG / GYRO_CONFIG 를 안 건드리므로 리셋 기본값 ±2g, ±250도/s.
#define ACC_LSB_PER_G     16384.0f
#define GYRO_LSB_PER_DPS  131.0f

// 1초치 계측. 루프 주기가 바뀌면 "초당 몇 회" 는 비교가 안 되므로 비율로 본다.
typedef struct {
    uint32_t loops;
    uint32_t enc_fail;
    uint32_t mpu_fail;
    uint32_t enc_bad;       // 물리적으로 불가능한 각도 변화
    uint32_t vote_fail;     // voting 실패 → 직전 값 유지한 횟수
    float    d_max;         // 최대 각도 변화 [rad]
    int64_t  d_max_dt_us;   // 그때의 루프 주기
#if SENSOR_SURVEY
    int16_t  dev_acc;       // 창 안의 최대 편차 (중앙값 대비) [LSB]
    int16_t  dev_gyro;
#endif
} stats_t;

typedef struct {
    stats_t st;
    int64_t win_us;     // 이 창의 실제 길이. loop Hz 를 정확히 내려면 필요하다
    float   wheel_vel;
    float   angle;      // 기울기 [deg], 0 = 똑바로
    float   rate;       // 기울기 각속도 [deg/s]
    float   uq;         // balance_torque 가 낸 q축 전압 [V]
#if SENSOR_SURVEY
    int16_t s_ay[NUM_SENSORS], s_az[NUM_SENSORS], s_gx[NUM_SENSORS];   // 찍는 순간의 raw
#endif
} stats_msg_t;

static QueueHandle_t s_stats_q;   // 길이 1. 제어 루프는 덮어쓰기만 하고 대기하지 않는다

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

static int16_t median3(int16_t a, int16_t b, int16_t c)
{
    if ((a >= b) == (a <= c)) return a;
    if ((b >= a) == (b <= c)) return b;
    return c;
}

#if SENSOR_SURVEY
static int16_t absdiff16(int16_t a, int16_t b)
{
    int32_t d = (int32_t)a - b;
    return (int16_t)(d < 0 ? -d : d);
}

// 세 센서가 중앙값에서 얼마나 벌어지는지. 창 안의 최대치를 남긴다.
static void survey_dev(stats_t *st, const int16_t a[][3], const int16_t g[][3],
                       const telemetry_fault_t *fault)
{
    int16_t m_ay = median3(a[0][1], a[1][1], a[2][1]);
    int16_t m_az = median3(a[0][2], a[1][2], a[2][2]);
    int16_t m_gx = median3(g[0][0], g[1][0], g[2][0]);

    for (int i = 0; i < NUM_SENSORS; i++) {
        if (fault[i] != TELEMETRY_FAULT_NONE) continue;   // 읽기 실패는 편차가 아니다
        int16_t d;
        d = absdiff16(a[i][1], m_ay); if (d > st->dev_acc)  st->dev_acc  = d;
        d = absdiff16(a[i][2], m_az); if (d > st->dev_acc)  st->dev_acc  = d;
        d = absdiff16(g[i][0], m_gx); if (d > st->dev_gyro) st->dev_gyro = d;
    }
}
#endif

// 엔코더 각도 차이. 0/2π 경계를 넘어도 되도록 최단 방향으로 접는다.
static float angle_delta(float now, float prev)
{
    float d = now - prev;
    if (d >  (float)M_PI) d -= 2.0f * (float)M_PI;
    if (d < -(float)M_PI) d += 2.0f * (float)M_PI;
    return d;
}

static void stats_add(stats_t *st, float d, float dt, int64_t dt_us)
{
    if (fabsf(d) > st->d_max) {
        st->d_max       = fabsf(d);
        st->d_max_dt_us = dt_us;
    }
    if (fabsf(d) > ENC_MAX_VEL * dt + ENC_BAD_FLOOR) st->enc_bad++;
}

#if SENSOR_SURVEY
// 편차가 임계값의 하한이다. 정지·기울임·밸런싱 중 최대치를 보고 여유를 얹어 정한다.
static void stats_print(const stats_msg_t *m)
{
    const stats_t *st = &m->st;
    unsigned long hz = m->win_us ? (unsigned long)(st->loops * 1000000LL / m->win_us) : 0;

    ESP_LOGI(TAG, "%luHz  ch0 %6d %6d %5d | ch1 %6d %6d %5d | ch6 %6d %6d %5d | "
                  "최대편차 acc %d gyro %d",
             hz,
             m->s_ay[0], m->s_az[0], m->s_gx[0],
             m->s_ay[1], m->s_az[1], m->s_gx[1],
             m->s_ay[2], m->s_az[2], m->s_gx[2],
             st->dev_acc, st->dev_gyro);
}
#else
/*
 * 최대 각도 변화가 그 순간 물리적으로 가능한 값(dt × 휠속도)의 몇 배인지 같이 찍는다.
 * 1.0 근처면 정상이고, 크게 넘으면 엔코더가 틀린 값을 준 것이다.
 */
static void stats_print(const stats_msg_t *m)
{
    const stats_t *st = &m->st;
    float d_max_deg = st->d_max * 180.0f / (float)M_PI;
    float explain   = fabsf(m->wheel_vel) * (st->d_max_dt_us * 1e-6f) * 180.0f / (float)M_PI;

    unsigned long hz = m->win_us ? (unsigned long)(st->loops * 1000000LL / m->win_us) : 0;

    // MPU 는 한 루프에 NUM_SENSORS 번 읽으므로 실패율 분모도 그만큼이다.
    uint32_t reads = st->loops * NUM_SENSORS;

    ESP_LOGI(TAG, "loop %luHz  각도 %6.1f도  각속도 %7.1f도/s  uq %6.2fV  "
                  "휠 %6.1f  MPU실패 %lu(%.2f%%)  voting실패 %lu  손상 %lu  최대변화 %.1f도(%.1f배)",
             hz, m->angle, m->rate, m->uq, m->wheel_vel,
             (unsigned long)st->mpu_fail,
             reads ? 100.0f * st->mpu_fail / reads : 0.0f,
             (unsigned long)st->vote_fail,
             (unsigned long)st->enc_bad,
             d_max_deg, (explain > 0.01f) ? d_max_deg / explain : 0.0f);
}
#endif  // SENSOR_SURVEY

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

/*
 * 채널별 영점. 읽은 값에서 이만큼 빼야 세 센서를 서로 비교할 수 있다.
 *
 * 자이로는 정지 상태에서 0 이어야 하므로 측정한 영점을 통째로 뺀다.
 * 가속도는 중력이 걸려 있어 통째로 빼면 각도 정보까지 사라진다. 세 센서 평균의 중앙값을
 * 기준 삼아 거기서 벗어난 만큼만 뺀다 — 물리값은 살리고 센서 간 편차만 없앤다.
 */
typedef struct {
    int16_t ay[NUM_SENSORS], az[NUM_SENSORS], gx[NUM_SENSORS];
} sensor_cal_t;

static void apply_cal(const sensor_cal_t *c, int i,
                      int16_t *ay, int16_t *az, int16_t *gx)
{
    *ay -= c->ay[i];
    *az -= c->az[i];
    *gx -= c->gx[i];
}

static void measure_cal(sensor_cal_t *cal)
{
    int32_t sum_ay[NUM_SENSORS] = {0}, sum_az[NUM_SENSORS] = {0}, sum_gx[NUM_SENSORS] = {0};
    int     n[NUM_SENSORS] = {0};
    int16_t ax, ay, az, gx, gy, gz;

    for (int k = 0; k < CAL_N; k++) {
        for (int i = 0; i < NUM_SENSORS; i++) {
            if (mpu6050_read_accel_gyro(MUX_CHANNELS[i], &ax, &ay, &az,
                                        &gx, &gy, &gz) == ESP_OK) {
                sum_ay[i] += ay;
                sum_az[i] += az;
                sum_gx[i] += gx;
                n[i]++;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    int16_t mean_ay[NUM_SENSORS], mean_az[NUM_SENSORS];
    for (int i = 0; i < NUM_SENSORS; i++) {
        mean_ay[i]  = n[i] ? (int16_t)(sum_ay[i] / n[i]) : 0;
        mean_az[i]  = n[i] ? (int16_t)(sum_az[i] / n[i]) : 0;
        cal->gx[i]  = n[i] ? (int16_t)(sum_gx[i] / n[i]) : 0;   // 자이로는 통째로
    }

    int16_t ref_ay = median3(mean_ay[0], mean_ay[1], mean_ay[2]);
    int16_t ref_az = median3(mean_az[0], mean_az[1], mean_az[2]);
    for (int i = 0; i < NUM_SENSORS; i++) {
        cal->ay[i] = mean_ay[i] - ref_ay;                       // 가속도는 중앙값 대비만
        cal->az[i] = mean_az[i] - ref_az;
    }

    for (int i = 0; i < NUM_SENSORS; i++) {
        ESP_LOGI(TAG, "ch%d 영점 ay %d az %d gx %d",
                 MUX_CHANNELS[i], cal->ay[i], cal->az[i], cal->gx[i]);
    }
}

// 조립 오차 때문에 "진짜 똑바로" 가 센서상 0 이 아니다. 세워 둔 채 수렴시켜 기준을 잡는다.
static float measure_setpoint(const sensor_cal_t *cal)
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
            apply_cal(cal, 0, &ay, &az, &gx);
            angle = balance_estimate_angle(ay, az, gx, 0.0f, dt, &rate);
        }
        vTaskDelay(pdMS_TO_TICKS(LOOP_PERIOD_MS));
    }
    return angle;
}

static telemetry_vote_t vote_to_telemetry(voting_result_t r)
{
    switch (r) {
    case VOTING_OK:       return TELEMETRY_VOTE_OK;
    case VOTING_DEGRADED: return TELEMETRY_VOTE_DEGRADED;
    default:              return TELEMETRY_VOTE_FAIL;
    }
}

// 대시보드로 한 프레임 보낸다. telemetry_publish 는 논블로킹이라 제어 루프에서 불러도 된다.
static void publish_frame(const int16_t a[][3], const int16_t g[][3],
                          const telemetry_fault_t *fault, const voting_out_t *vo,
                          float tilt, float rate, float setpoint,
                          float uq, float enc_rad)
{
    telemetry_frame_t f = {
        .vote      = vote_to_telemetry(vo->result),
        .angle     = tilt,
        .rate      = rate,
        .setpoint  = setpoint,
        .uq        = uq,
        .enc_angle = enc_rad * (180.0f / (float)M_PI),
    };

    for (int i = 0; i < NUM_SENSORS; i++) {
        f.vote_used[i]     = vo->used[i];
        f.sensors[i].ch    = MUX_CHANNELS[i];
        f.sensors[i].fault = fault[i];
        f.sensors[i].ax = a[i][0] / ACC_LSB_PER_G;
        f.sensors[i].ay = a[i][1] / ACC_LSB_PER_G;
        f.sensors[i].az = a[i][2] / ACC_LSB_PER_G;
        f.sensors[i].gx = g[i][0] / GYRO_LSB_PER_DPS;
        f.sensors[i].gy = g[i][1] / GYRO_LSB_PER_DPS;
        f.sensors[i].gz = g[i][2] / GYRO_LSB_PER_DPS;
    }
    telemetry_publish(&f);
}

static void balance_loop(float offset, const sensor_cal_t *cal, float setpoint)
{
    int64_t now       = esp_timer_get_time();
    int64_t prev_time = now, last_stat = now;

    stats_t st = {0};
    float   wheel_vel = 0.0f, prev_angle = 0.0f;
    float   tilt = 0.0f, rate = 0.0f, uq = 0.0f;
    int16_t a[NUM_SENSORS][3], g[NUM_SENSORS][3];   // [채널][x,y,z]
    telemetry_fault_t fault[NUM_SENSORS] = {0};

    encoder_read_angle(&prev_angle);

    while (1) {
        now = esp_timer_get_time();
        int64_t dt_us = now - prev_time;
        float   dt    = dt_us * 1e-6f;
        prev_time = now;
        st.loops++;

        // 한 번에 약 1ms 씩 더 든다. 실패 1회는 I2C 타임아웃 10ms 라 반드시 세어야 한다.
        for (int i = 0; i < NUM_SENSORS; i++) {
            if (mpu6050_read_accel_gyro(MUX_CHANNELS[i],
                                        &a[i][0], &a[i][1], &a[i][2],
                                        &g[i][0], &g[i][1], &g[i][2]) != ESP_OK) {
                st.mpu_fail++;
                fault[i] = TELEMETRY_FAULT_DROPOUT;
                a[i][0] = a[i][1] = a[i][2] = 0;
                g[i][0] = g[i][1] = g[i][2] = 0;
            } else {
                fault[i] = TELEMETRY_FAULT_NONE;
                apply_cal(cal, i, &a[i][1], &a[i][2], &g[i][0]);
            }
        }
#if SENSOR_SURVEY
        survey_dev(&st, a, g, fault);
#endif

        // 읽기가 ESP_OK 여도 값만 깨져 오는 경우가 있다(비트 뒤집힘). fault 로는 안 잡힌다.
        voting_sample_t vs[VOTING_N];
        for (int i = 0; i < NUM_SENSORS; i++) {
            vs[i] = (voting_sample_t){ .ay = a[i][1], .az = a[i][2], .gx = g[i][0],
                                       .valid = (fault[i] == TELEMETRY_FAULT_NONE) };
        }
        voting_out_t vo;
        voting_fuse(vs, &vo);

        /*
         * FAIL 이면 tilt/rate/uq 를 그대로 둬 직전 명령을 이어간다. 특히 전부 dropout 이면
         * 값이 0 인데, atan2(0,0) 이 0 이라 "똑바로 서 있다" 로 읽혀 복원 토크가 사라진다.
         */
        if (vo.result != VOTING_FAIL) {
            // 영점은 apply_cal 에서 이미 뺐으므로 bias 인자는 0 이다.
            tilt = balance_estimate_angle(vo.ay, vo.az, vo.gx, 0.0f, dt, &rate) - setpoint;
            uq   = balance_torque(tilt, rate, wheel_vel);
        } else {
            st.vote_fail++;
        }

        float angle;
        if (encoder_read_angle(&angle) != ESP_OK) {
            // 로그 금지: UART 블로킹이 커뮤테이션을 더 멈춘다.
            // 직전 전압 벡터를 그대로 두고 보정하지 않는다.
            st.enc_fail++;
        } else {
            float d = angle_delta(angle, prev_angle);
            stats_add(&st, d, dt, dt_us);

            wheel_vel  = (1.0f - VEL_LPF) * wheel_vel + VEL_LPF * (d / dt);
            prev_angle = angle;

            // 읽은 위치가 아니라 다음 갱신 시점의 예상 위치에 자계를 세운다.
            // 상한은 wheel_vel 이 튀어도 벡터가 날아가지 않게 하려는 것이다.
            float adv     = LAG_COMP * wheel_vel * dt;
            float adv_max = ENC_MAX_VEL * dt;
            if (adv >  adv_max) adv =  adv_max;
            if (adv < -adv_max) adv = -adv_max;

            // DRIVE_MOTOR 0 이면 EN 이 꺼져 있어 duty 를 써도 출력이 나가지 않는다.
            foc_apply_torque(uq, angle + adv, offset);
        }

        publish_frame(a, g, fault, &vo, tilt, rate, setpoint, uq, prev_angle);

        if (now - last_stat >= STAT_US) {
            stats_msg_t msg = { .st = st, .win_us = now - last_stat,
                                .wheel_vel = wheel_vel,
                                .angle = tilt, .rate = rate, .uq = uq };
#if SENSOR_SURVEY
            for (int i = 0; i < NUM_SENSORS; i++) {
                msg.s_ay[i] = a[i][1];
                msg.s_az[i] = a[i][2];
                msg.s_gx[i] = g[i][0];
            }
#endif
            last_stat = now;
            xQueueOverwrite(s_stats_q, &msg);
            st = (stats_t){0};
        }

        vTaskDelay(pdMS_TO_TICKS(LOOP_PERIOD_MS));
    }
}

static void control_task(void *arg)
{
    ESP_LOGI(TAG, "센서 영점 측정 중 — 똑바로 세워 가만히 잡으세요");
    sensor_cal_t cal;
    measure_cal(&cal);

    ESP_LOGI(TAG, "기준 각도 측정 중 (%d ms)", SETPOINT_MS);
    float setpoint = measure_setpoint(&cal);

#if DRIVE_MOTOR
    float offset = align_rotor();
#else
    float offset = 0.0f;   // 모터를 안 돌리므로 커뮤테이션 정렬이 필요 없다
#endif

    ESP_LOGI(TAG, "기준 각도 %.1f도, 구동 %s", setpoint, DRIVE_MOTOR ? "ON" : "OFF(계측만)");
    balance_loop(offset, &cal, setpoint);
}

// app_main(우선순위 1)이 그대로 로그 태스크가 된다. 제어보다 낮으므로 UART 를
// 기다리는 중에도 제어 루프가 언제든 뺏어간다.
void app_main(void)
{
    // 가장 먼저. 이후의 초기화 로그(자이로 영점, 정렬 오프셋)까지 웹으로 실린다.
    // WiFi 연결까지 블록하므로 공유기가 없으면 여기서 멈춘다.
    static const telemetry_config_t tele_cfg = {
        .wifi_ssid     = WIFI_SSID,
        .wifi_password = WIFI_PASSWORD,
        .server_uri    = SERVER_WS_URI,
    };
    ESP_ERROR_CHECK(telemetry_start(&tele_cfg));

    i2c_bus_init();
    ESP_ERROR_CHECK(encoder_init(I2C_PORT));
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

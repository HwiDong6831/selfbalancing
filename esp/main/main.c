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
#define I2C_FREQ_HZ    400000

#define LOOP_PERIOD_MS 8           // 제어 루프 고정 주기 [ms]
#define VEL_LPF        0.2f        // 휠 속도 LPF 계수

#define TELEMETRY_US   20000       // 대시보드 송신 간격 [µs]

#define GYRO_CAL_N     200         // gz 영점 평균 샘플 수
#define SETPOINT_MS    2000        // 상보필터를 수렴시키는 시간 [ms]

#define LAG_COMP       1.0f        // 커뮤테이션 지연 보상. 1.0 = 한 루프치

#define ALIGN_UD_V     2.0f        // 정렬에 쓰는 d축 전압 [V]
#define ALIGN_HOLD_MS  1000        // 회전자를 전기각 0 으로 끌어오는 대기 [ms]
#define ALIGN_SWEEP_MS 1000        // 한 방향 스윕 시간 [ms]
#define ALIGN_STEPS    200         // 스윕 스텝 수
#define ALIGN_SAMPLES  16          // 끝점 각도 평균 샘플 수

// 엔코더 손상 판정 상한 = ENC_MAX_VEL × 경과시간 + ENC_BAD_FLOOR [rad]
#define ENC_MAX_VEL    80.0f
#define ENC_BAD_FLOOR  0.30f

#define WHEEL_MAX_ACC  300.0f      // 휠 각가속도 상한 [rad/s^2]
#define STAT_US        100000      // 통계 창 길이 [µs]

#define CTRL_PRIO      10
#define CTRL_STACK     4096
#define CTRL_CORE      0

static const char *TAG = "MAIN";

#define NUM_SENSORS    VOTING_N
static const int MUX_CHANNELS[NUM_SENSORS] = {5, 6, 7};

#define ACC_LSB_PER_G     16384.0f  // 가속도 감도 (±2g 기준)
#define GYRO_LSB_PER_DPS  131.0f    // 자이로 감도 (±250°/s 기준)

typedef struct {
    uint32_t loops;
    uint32_t mpu_fail;
    uint32_t enc_bad;       // 잘못된 값(갑자기 튀는 값)
} stats_t;

typedef struct {
    stats_t st;
    int64_t win_us;
    float   wheel_vel;
    float   angle;      // 기울기 [deg]
    float   rate;       // 기울기 각속도 [deg/s]
    float   uq;         // q축 전압 [V]
} stats_msg_t;

static QueueHandle_t s_stats_q;

static float s_gz0[NUM_SENSORS];      // 자이로 영점
static float s_a0[NUM_SENSORS][2];    // 가속도 영점

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

static int16_t clamp16(float v)
{
    if (v >  32767.0f) return  32767;
    if (v < -32768.0f) return -32768;
    return (int16_t)v;
}

// 엔코더 각도 차 계산
static float angle_delta(float now, float prev)
{
    float d = now - prev;
    if (d >  (float)M_PI) d -= 2.0f * (float)M_PI;
    if (d < -(float)M_PI) d += 2.0f * (float)M_PI;
    return d;
}

// 인코더 값이 잘못되었는지 판단(갑자기 튄 값)
static bool enc_implausible(float d, float span)
{
    return fabsf(d) > ENC_MAX_VEL * span + ENC_BAD_FLOOR;
}

static void stats_print(const stats_msg_t *m)
{
    const stats_t *st = &m->st;
    unsigned long hz = m->win_us ? (unsigned long)(st->loops * 1000000LL / m->win_us) : 0;
    uint32_t reads = st->loops * NUM_SENSORS;

    ESP_LOGI(TAG, "loop %luHz  각도 %6.1f도  각속도 %7.1f도/s  uq %6.2fV  "
                  "휠 %6.1f  MPU실패 %lu(%.2f%%)  손상버림 %lu  카운트 %5d",
             hz, m->angle, m->rate, m->uq, m->wheel_vel,
             (unsigned long)st->mpu_fail,
             reads ? 100.0f * st->mpu_fail / reads : 0.0f,
             (unsigned long)st->enc_bad, encoder_get_count());
}

// 엔코더 각도 평균 구하기
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

// 기계각과 전기각 오프셋 구하기
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

    foc_set_phase_voltage(0.0f, 0.0f, 0.0f);

    return foc_align(angle_fwd, angle_rev);
}

// 센서 채널별 영점
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

    for (int k = 0; k < 2; k++) {
        float x = mean_a[0][k], y = mean_a[1][k], z = mean_a[2][k];
        float med = (x < y) ? ((y < z) ? y : ((x < z) ? z : x))
                            : ((x < z) ? x : ((y < z) ? z : y));
        for (int c = 0; c < NUM_SENSORS; c++) s_a0[c][k] = mean_a[c][k] - med;
    }
}

// 기준 각도(세웠을 때) 도출
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

// 대시보드로 메트리 전송
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

    f.accel.result = (telemetry_vote_t)va->result;
    f.gyro.result  = (telemetry_vote_t)vg->result;
    f.accel.val[0] = va->val[0] / ACC_LSB_PER_G;
    f.accel.val[1] = va->val[1] / ACC_LSB_PER_G;
    f.gyro.val[0]  = vg->val[0] / GYRO_LSB_PER_DPS;
    for (int i = 0; i < NUM_SENSORS; i++) {
        f.accel.used[i] = va->used[i];
        f.gyro.used[i]  = vg->used[i];
    }

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

    // 영점을 뺀 투표용 값 (가속도 ax,ay / 자이로 gz)
    int16_t av[NUM_SENSORS][VOTING_AXIS_MAX], gv[NUM_SENSORS][VOTING_AXIS_MAX] = {{0}};
    bool    valid[NUM_SENSORS];
    voting_out_t va = {0}, vg = {0};

    int16_t held_a[NUM_SENSORS][3] = {{0}}, held_g[NUM_SENSORS][3] = {{0}};  // freeze 가 되돌려 줄 값
    float   drift[NUM_SENSORS] = {0};                                        // drift 누적 벌어짐

    encoder_read_angle(&prev_angle);

    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
        now = esp_timer_get_time();
        int64_t dt_us = now - prev_time;
        float   dt    = dt_us * 1e-6f;
        prev_time = now;
        st.loops++;

        for (int i = 0; i < NUM_SENSORS; i++) {
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

        voting_fuse(av, valid, 2, VOTING_TOL_ACC,  &va);
        voting_fuse(gv, valid, 1, VOTING_TOL_GYRO, &vg);

        // FAIL 이면 갱신하지 않고 직전 명령을 잇는다.
        if (va.result != VOTING_FAIL && vg.result != VOTING_FAIL) {
            tilt = balance_estimate_angle(va.val[0], va.val[1], vg.val[0],
                                          0.0f, dt, &rate) - setpoint;
            uq   = balance_torque(tilt, rate, wheel_vel);
        }

        float angle;
        encoder_read_angle(&angle);
        since_ok += dt;

        float d = angle_delta(angle, prev_angle);

        if (enc_implausible(d, since_ok)) {
            st.enc_bad++;
        } else {
            float meas = d / since_ok;
            float step = WHEEL_MAX_ACC * since_ok;
            if (meas > wheel_vel + step) meas = wheel_vel + step;
            if (meas < wheel_vel - step) meas = wheel_vel - step;

            wheel_vel  = (1.0f - VEL_LPF) * wheel_vel + VEL_LPF * meas;
            prev_angle = angle;
            since_ok   = 0.0f;

            // 읽은 위치가 아니라 다음 갱신 시점의 예상 위치에 자계를 세움.
            float adv     = LAG_COMP * wheel_vel * dt;
            float adv_max = ENC_MAX_VEL * dt;
            if (adv >  adv_max) adv =  adv_max;
            if (adv < -adv_max) adv = -adv_max;

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

        xTaskDelayUntil(&last_wake, pdMS_TO_TICKS(LOOP_PERIOD_MS));
    }
}

static void control_task(void *arg)
{
    ESP_LOGI(TAG, "센서 영점 측정 중");
    measure_cal();

    for (int c = 0; c < NUM_SENSORS; c++) {
        ESP_LOGI(TAG, "ch%d 영점  자이로 %7.1f  가속도 %7.1f / %7.1f LSB",
                 MUX_CHANNELS[c], s_gz0[c], s_a0[c][0], s_a0[c][1]);
    }

    ESP_LOGI(TAG, "기준 각도 측정 중 (%d ms)", SETPOINT_MS);
    float setpoint = measure_setpoint();

    float offset = align_rotor();

    ESP_LOGI(TAG, "기준 각도 %.1f도", setpoint);
    balance_loop(offset, setpoint);
}

void app_main(void)
{
    static const telemetry_config_t tele_cfg = {
        .wifi_ssid     = WIFI_SSID,
        .wifi_password = WIFI_PASSWORD,
        .server_uri    = SERVER_WS_URI,
    };
    ESP_ERROR_CHECK(telemetry_start(&tele_cfg));

    i2c_bus_init();
    ESP_ERROR_CHECK(encoder_init());
    ESP_ERROR_CHECK(mpu6050_init(I2C_PORT, MUX_CHANNELS, NUM_SENSORS));

    foc_init();
    foc_enable(true);

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

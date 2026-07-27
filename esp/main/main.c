/*
 * 2단계 — 엔코더 폐루프 커뮤테이션 + 고정 토크.
 *
 * MPU 는 읽기만 하고 제어에는 쓰지 않는다. 통신선을 공유하는 상태를 만드는 것이
 * 목적이다. 3단계에서 기울기 제어를 붙인다.
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

#define I2C_PORT       I2C_NUM_0
#define PIN_SDA        16
#define PIN_SCL        17
#define I2C_FREQ_HZ    100000

#define LOOP_PERIOD_MS 1           // 여기에 MPU 읽기 시간이 더해져 실제로는 약 250Hz
#define VEL_LPF        0.2f

/*
 * 0 = 계측만. 드라이버 EN 을 끄고 정렬도 건너뛴다. uq 는 계산해서 찍기만 한다.
 * 부호가 반대면 기울수록 넘어지는 쪽으로 세게 밀어붙이므로, 모터를 물리기 전에
 * 손으로 기울여 로그로 확인한다.
 */
#define DRIVE_MOTOR    1

#define GYRO_CAL_N     200         // gx bias 평균 샘플 수 (5ms 간격 = 1초)
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

#define STAT_US        1000000

// 로그 출력(190자 @115200 = 16.5ms)이 커뮤테이션을 멈추지 않도록 제어를 별도 태스크로
// 뺀다. 우선순위가 높으면 로그 태스크가 UART 를 기다리는 중이어도 즉시 뺏어온다.
#define CTRL_PRIO      10
#define CTRL_STACK     4096
#define CTRL_CORE      0

static const char *TAG = "MAIN";

// 제어에 쓰는 건 0 번뿐이다. 나머지 둘은 표시용으로 나중에 붙인다.
static const int MUX_CHANNELS[3] = {0, 1, 6};
static const int NUM_SENSORS     = 3;

// 1초치 계측. 루프 주기가 바뀌면 "초당 몇 회" 는 비교가 안 되므로 비율로 본다.
typedef struct {
    uint32_t loops;
    uint32_t enc_fail;
    uint32_t mpu_fail;
    uint32_t enc_bad;       // 물리적으로 불가능한 각도 변화
    float    d_max;         // 최대 각도 변화 [rad]
    int64_t  d_max_dt_us;   // 그때의 루프 주기
} stats_t;

typedef struct {
    stats_t st;
    float   wheel_vel;
    float   angle;      // 기울기 [deg], 0 = 똑바로
    float   rate;       // 기울기 각속도 [deg/s]
    float   uq;         // balance_torque 가 낸 q축 전압 [V]
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

/*
 * 최대 각도 변화가 그 순간 물리적으로 가능한 값(dt × 휠속도)의 몇 배인지 같이 찍는다.
 * 1.0 근처면 정상이고, 크게 넘으면 엔코더가 틀린 값을 준 것이다.
 */
static void stats_print(const stats_msg_t *m)
{
    const stats_t *st = &m->st;
    float d_max_deg = st->d_max * 180.0f / (float)M_PI;
    float explain   = fabsf(m->wheel_vel) * (st->d_max_dt_us * 1e-6f) * 180.0f / (float)M_PI;

    ESP_LOGI(TAG, "loop %luHz  각도 %6.1f도  각속도 %7.1f도/s  uq %6.2fV  "
                  "휠 %6.1f  MPU실패 %lu(%.2f%%)  손상 %lu  최대변화 %.1f도(%.1f배)",
             (unsigned long)st->loops, m->angle, m->rate, m->uq, m->wheel_vel,
             (unsigned long)st->mpu_fail,
             st->loops ? 100.0f * st->mpu_fail / st->loops : 0.0f,
             (unsigned long)st->enc_bad,
             d_max_deg, (explain > 0.01f) ? d_max_deg / explain : 0.0f);
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

// 정지 상태에서 재는 자이로 영점. 이 값을 빼야 적분이 드리프트하지 않는다.
static float measure_gyro_bias(void)
{
    int32_t sum = 0;
    int     n   = 0;
    int16_t ax, ay, az, gx, gy, gz;

    for (int i = 0; i < GYRO_CAL_N; i++) {
        if (mpu6050_read_accel_gyro(MUX_CHANNELS[0], &ax, &ay, &az,
                                    &gx, &gy, &gz) == ESP_OK) {
            sum += gx;
            n++;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    return n ? (float)sum / n : 0.0f;
}

// 조립 오차 때문에 "진짜 똑바로" 가 센서상 0 이 아니다. 세워 둔 채 수렴시켜 기준을 잡는다.
static float measure_setpoint(float gx_bias)
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
            angle = balance_estimate_angle(ay, az, gx, gx_bias, dt, &rate);
        }
        vTaskDelay(pdMS_TO_TICKS(LOOP_PERIOD_MS));
    }
    return angle;
}

static void balance_loop(float offset, float gx_bias, float setpoint)
{
    int64_t now       = esp_timer_get_time();
    int64_t prev_time = now, last_stat = now;

    stats_t st = {0};
    float   wheel_vel = 0.0f, prev_angle = 0.0f;
    float   tilt = 0.0f, rate = 0.0f, uq = 0.0f;
    int16_t ax, ay, az, gx, gy, gz;

    encoder_read_angle(&prev_angle);

    while (1) {
        now = esp_timer_get_time();
        int64_t dt_us = now - prev_time;
        float   dt    = dt_us * 1e-6f;
        prev_time = now;
        st.loops++;

        // 실패 1회는 I2C 타임아웃 10ms 라 반드시 세어야 한다.
        if (mpu6050_read_accel_gyro(MUX_CHANNELS[0], &ax, &ay, &az,
                                    &gx, &gy, &gz) != ESP_OK) {
            st.mpu_fail++;
        } else {
            tilt = balance_estimate_angle(ay, az, gx, gx_bias, dt, &rate) - setpoint;
            uq   = balance_torque(tilt, rate);
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

        if (now - last_stat >= STAT_US) {
            last_stat = now;
            stats_msg_t msg = { .st = st, .wheel_vel = wheel_vel,
                                .angle = tilt, .rate = rate, .uq = uq };
            xQueueOverwrite(s_stats_q, &msg);
            st = (stats_t){0};
        }

        vTaskDelay(pdMS_TO_TICKS(LOOP_PERIOD_MS));
    }
}

static void control_task(void *arg)
{
    ESP_LOGI(TAG, "자이로 영점 측정 중 — 가만히 두세요");
    float gx_bias = measure_gyro_bias();

    ESP_LOGI(TAG, "영점 %.1f LSB. 똑바로 세워 잡으세요 (%d ms)", gx_bias, SETPOINT_MS);
    float setpoint = measure_setpoint(gx_bias);

#if DRIVE_MOTOR
    float offset = align_rotor();
#else
    float offset = 0.0f;   // 모터를 안 돌리므로 커뮤테이션 정렬이 필요 없다
#endif

    ESP_LOGI(TAG, "기준 각도 %.1f도, 구동 %s", setpoint, DRIVE_MOTOR ? "ON" : "OFF(계측만)");
    balance_loop(offset, gx_bias, setpoint);
}

// app_main(우선순위 1)이 그대로 로그 태스크가 된다. 제어보다 낮으므로 UART 를
// 기다리는 중에도 제어 루프가 언제든 뺏어간다.
void app_main(void)
{
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

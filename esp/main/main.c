/*
 * 1단계 — 최소 폐루프.
 *
 * 엔코더 각도로 전기각을 만들어 고정 전압을 인가하는 것만 한다.
 * MPU·텔레메트리·이상값 검사·예측·지연보상 전부 없다.
 *
 * 여기서 소리가 나면 폐루프 커뮤테이션 자체가 원인이다.
 * 조용하면 이 위에 하나씩 얹어가며 소리가 나기 시작하는 지점을 찾는다.
 */
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/i2c.h"

#include "encoder.h"
#include "mpu6050.h"
#include "foc.h"

#define I2C_PORT    I2C_NUM_0
#define PIN_SDA     16
#define PIN_SCL     17
// 50k / 100k / 400k 를 비교한 결과 MPU 읽기 실패율이 루프당 1.00% / 1.09% / 0.87% 로
// 사실상 평평했다. 통신 속도는 원인이 아니므로 기준값으로 되돌린다.
#define I2C_FREQ_HZ 100000

#define TEST_UQ        3.0f        // 인가할 q축 전압 [V]
#define FLIP_US        5000000     // 회전 방향 유지 시간. 정/역을 번갈아 들어본다
#define ALIGN_UD_V     2.0f        // 정렬 시 d축 전압
#define ALIGN_HOLD_MS  1000        // 회전자가 전기각 0 으로 끌려올 때까지 대기
#define STAT_US        1000000
#define VEL_LPF        0.2f

/*
 * 손상 판정 상한 = ENC_MAX_VEL × dt + ENC_BAD_FLOOR [rad].
 *
 * 휠 속도 "추정치" 를 쓰지 않는 것이 핵심이다. 손상값이 그대로 추정치에 들어가
 * 추정치를 오염시키고, 그 오염된 추정치로 판정하면 기준 자체가 무너진다.
 * 실측 최고 50 rad/s 이므로 80 을 상한으로 잡으면 정상값은 절대 걸리지 않는다.
 *
 * 세기만 하고 걸러내지는 않는다. 걸러내면 무엇이 들어오는지 볼 수 없다.
 */
#define ENC_MAX_VEL    80.0f       // 물리적 최고 휠 속도 [rad/s]
#define ENC_BAD_FLOOR  0.30f       // 고정 여유 [rad] (17도)

/*
 * 건너뛰기는 뺐다.
 *
 * MPU 읽기가 실패한 루프의 엔코더 값을 버리는 우회책을 넣어 봤으나,
 * prev_angle 을 갱신하지 않는 바람에 다음 비교가 여러 루프치 움직임을 담게 되고
 * 그게 손상으로 세어져 계측 자체를 오염시켰다(손상 48 → 60 → 98).
 *
 * 41초 전 구간에서 "MPU실패 0 인 초는 손상도 0" 이 19/19 예외 없이 성립했다.
 * 우회하지 말고 MPU 읽기 실패를 없애는 쪽으로 간다.
 */

/*
 * 루프 주기 = 커뮤테이션 갱신 주기.
 *
 * 1 = 999Hz(1단계, 무음) / 4 = 250Hz(2-A단계, 무음).
 * 주기만으로는 소리가 나지 않는다는 것이 확인됐으므로 1 로 되돌린다.
 * 여기에 MPU 읽기가 붙어 자연히 300Hz 근처가 된다.
 */
#define LOOP_PERIOD_MS 1

/*
 * 1 이면 모터를 켜지 않고 통신선만 시험한다.
 *
 * MPU 읽기 실패가 100% 시간 초과였다(무응답 0). 거래를 시작했는데 못 끝낸다는
 * 뜻이므로 무언가가 통신선을 붙잡고 있다. 이 선에 붙은 것 중 아직 배제하지 않은
 * 것은 엔코더(MT6701, 0x06) 하나뿐이다. 0x06 은 I2C 규격 예약 대역(0x04~0x07)이며,
 * 루프에서 매번 MPU 읽기 사이에 끼어 있다.
 *
 * 엔코더 읽기를 넣고 뺀 두 구간의 시간초과율을 비교한다.
 */
#define BUS_TEST 0
#define BUS_TEST_UD_V 1.5f         // C 구간 d축 전압. DC 라 오래 물리지 않는다

/*
 * PWM 주파수를 foc.c 기본값(22000) 대신 이 값으로 바꾼다. 0 이면 그대로 둔다.
 *
 * 실측 시간초과율: 정지 0.00% / 22kHz 0.97% / 10kHz 0.35% / 5kHz 0.04%.
 * 22kHz 는 가청 대역 위라 조용하지만 통신을 깨뜨린다. 5kHz 는 통신이 무결한 대신
 * 모터에서 삐 소리가 난다. 맞바꿈이다.
 *
 * 먼저 5kHz 로 타격음이 사라지는지 확인하고, 사라지면 타협점을 찾는다.
 */
#define PWM_FREQ_OVERRIDE 5000

static const char *TAG = "MAIN";

// 제어에 쓰는 건 0 번 하나뿐이다. 나머지 둘은 나중에 표시용으로 붙인다.
static const int MUX_CHANNELS[3] = {0, 1, 6};
static const int NUM_SENSORS     = 3;

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

/*
 * d축에 전압을 걸어 회전자를 전기각 0 으로 끌어온 뒤 그때의 기계각을 잰다.
 * 극쌍이 7 이라 안정 위치가 7 군데지만 전기적으로는 같은 자리라 어디에 서든 된다.
 */
static float align_rotor(void)
{
    foc_set_phase_voltage(ALIGN_UD_V, 0.0f, 0.0f);
    vTaskDelay(pdMS_TO_TICKS(ALIGN_HOLD_MS));

    float a = 0.0f;
    ESP_ERROR_CHECK(encoder_read_angle(&a));

    foc_set_phase_voltage(0.0f, 0.0f, 0.0f);   // 정렬 전압 해제. 물고 있으면 DC 가 계속 흐른다

    float offset = foc_align(a);
    ESP_LOGI(TAG, "정렬: 기계각 %.1f도 → 오프셋 %.1f도",
             a * 180.0f / (float)M_PI, offset * 180.0f / (float)M_PI);
    return offset;
}

#if BUS_TEST
/*
 * 한 구간을 seconds 만큼 돌리며 MPU 읽기 실패 내역을 센다.
 * with_encoder 만 다른 두 구간을 비교하는 것이 목적이다.
 */
static void bus_test_phase(const char *name, bool with_encoder, int seconds)
{
    int16_t  ax, ay, az, gx, gy, gz;
    float    a;
    uint32_t n = 0, enc_fail = 0;
    int64_t  end = esp_timer_get_time() + (int64_t)seconds * 1000000;

    mpu6050_take_fail_counts(NULL, NULL, NULL);      // 카운터 초기화

    while (esp_timer_get_time() < end) {
        n++;
        mpu6050_read_accel_gyro(MUX_CHANNELS[0], &ax, &ay, &az, &gx, &gy, &gz);
        if (with_encoder && encoder_read_angle(&a) != ESP_OK) enc_fail++;
        vTaskDelay(pdMS_TO_TICKS(LOOP_PERIOD_MS));
    }

    uint32_t mux_f = 0, nack = 0, tout = 0;
    mpu6050_take_fail_counts(&mux_f, &nack, &tout);

    ESP_LOGI(TAG, "%s : %lu회 중  mux %lu  무응답 %lu  시간초과 %lu (%.2f%%)  엔코더실패 %lu",
             name, (unsigned long)n, (unsigned long)mux_f, (unsigned long)nack,
             (unsigned long)tout, n ? (100.0f * tout / n) : 0.0f,
             (unsigned long)enc_fail);
}

/*
 * 스위칭만으로 시간 초과가 발생함이 확인됐다(정지 0.00% → 스위칭 0.77%,
 * 전류를 흘려도 0.78% 로 변화 없음).
 *
 * 원인이 스위칭 가장자리라면 주파수를 낮출수록 실패율도 같이 떨어져야 한다.
 * 비례하지 않으면 다른 메커니즘이다.
 *
 * 조건은 전부 동일하다 — 세 상 듀티 0.5(전압차 0, 전류 없음), 읽기는 MPU + 엔코더.
 * 주파수만 바꾼다. 낮은 주파수는 가청 대역이라 모터에서 소리가 나는 것이 정상이다.
 */
static void bus_test(void)
{
    static const uint32_t FREQS[] = { 22000, 10000, 5000 };
    char name[32];

    ESP_LOGI(TAG, "통신선 시험 시작. 읽기는 전부 MPU + 엔코더.");

    // 기준: 드라이버 꺼짐. 스위칭도 전류도 없다
    foc_enable(false);
    bus_test_phase("정지 (기준)   ", true, 10);

    // 세 상 듀티가 전부 0.5 로 같아 코일 양단 전압차가 0.
    // 스위칭은 하지만 전류는 흐르지 않는다
    foc_set_phase_voltage(0.0f, 0.0f, 0.0f);
    foc_enable(true);

    for (int i = 0; i < (int)(sizeof(FREQS) / sizeof(FREQS[0])); i++) {
        ESP_ERROR_CHECK(foc_set_pwm_freq(FREQS[i]));
        foc_set_phase_voltage(0.0f, 0.0f, 0.0f);     // 주파수 변경 후 듀티 재적용
        snprintf(name, sizeof(name), "스위칭 %5lukHz", (unsigned long)(FREQS[i] / 1000));
        bus_test_phase(name, true, 10);
    }

    foc_enable(false);
    ESP_ERROR_CHECK(foc_set_pwm_freq(22000));
    ESP_LOGI(TAG, "통신선 시험 끝");
}
#endif

static void torque_loop(float offset)
{
    int64_t prev_time = esp_timer_get_time();
    int64_t last_stat = prev_time;
    int64_t flip_time = prev_time;
    int     dir       = 1;

    uint32_t loop_n = 0, enc_fail = 0, mpu_fail = 0, enc_bad = 0;
    uint32_t bad_after_fail = 0;      // MPU 읽기가 실패한 그 루프에서 발생한 손상
    float    d_max = 0.0f, wheel_vel = 0.0f;
    int64_t  d_max_dt_us = 0;
    int16_t  ax, ay, az, gx, gy, gz;

    float prev_angle = 0.0f;
    encoder_read_angle(&prev_angle);

    prev_time = last_stat = flip_time = esp_timer_get_time();

    while (1) {
        int64_t now   = esp_timer_get_time();
        int64_t dt_us = now - prev_time;
        float   dt    = dt_us * 1e-6f;
        prev_time = now;
        loop_n++;

        if (now - flip_time >= FLIP_US) {
            flip_time = now;
            dir = -dir;
        }

        // 2-B: 값은 쓰지 않는다. 같은 통신선에 mux + MPU 트랜잭션을 끼워넣는 것만이 목적이다.
        // 실패 1 회 = I2C 타임아웃 10ms 이므로 반드시 세어야 한다.
        bool mpu_failed = false;
        if (mpu6050_read_accel_gyro(MUX_CHANNELS[0], &ax, &ay, &az, &gx, &gy, &gz) != ESP_OK) {
            mpu_fail++;
            mpu_failed = true;
        }

        float angle;
        if (encoder_read_angle(&angle) != ESP_OK) {
            // 로그 금지: UART 블로킹이 커뮤테이션을 더 멈춘다.
            // 읽기에 실패하면 직전 전압 벡터를 그대로 둔다. 보정하지 않는다.
            enc_fail++;
        } else {
            float d = angle_delta(angle, prev_angle);
            if (fabsf(d) > d_max) {
                d_max       = fabsf(d);
                d_max_dt_us = dt_us;
            }
            // 손상이 "MPU 읽기가 실패한 그 루프" 에 몰려 있는지 본다.
            // 타임아웃은 트랜잭션이 중간에 끊긴 상태라, 슬레이브가 선을 붙잡은 채로
            // 남으면 직후의 엔코더 읽기가 지저분한 상태에서 시작한다.
            if (fabsf(d) > ENC_MAX_VEL * dt + ENC_BAD_FLOOR) {
                enc_bad++;
                if (mpu_failed) bad_after_fail++;
            }

            wheel_vel  = (1.0f - VEL_LPF) * wheel_vel + VEL_LPF * (d / dt);
            prev_angle = angle;

            foc_apply_torque(dir * TEST_UQ, angle, offset);
        }

        // 초당 1회. 이 출력 자체도 UART 블로킹이라 루프 하나를 늦춘다.
        if (now - last_stat >= STAT_US) {
            last_stat = now;

            // 최대변화가 그 순간 물리적으로 가능한 값(dt × 휠속도)인지 본다.
            // 1.0배 근처면 정상, 크게 넘으면 엔코더 값이 틀린 것이다.
            float d_max_deg  = d_max * 180.0f / (float)M_PI;
            float explain_deg = fabsf(wheel_vel) * (d_max_dt_us * 1e-6f) * 180.0f / (float)M_PI;

            // 루프 주기가 바뀌면 초당 횟수는 비교가 안 된다. 루프당 비율로 같이 찍는다.
            float bad_pct = loop_n ? (100.0f * enc_bad  / loop_n) : 0.0f;
            float mpu_pct = loop_n ? (100.0f * mpu_fail / loop_n) : 0.0f;

            // MPU 읽기 실패가 mux 채널 쓰기에서 났는지 데이터 읽기에서 났는지.
            // 어느 쪽이냐에 따라 대책이 다르다.
            uint32_t mux_f = 0, nack = 0, tout = 0;
            mpu6050_take_fail_counts(&mux_f, &nack, &tout);

            ESP_LOGI(TAG, "loop %luHz  휠 %6.1f rad/s  엔코더실패 %lu  "
                          "MPU실패 %lu(%.2f%%: mux %lu, 무응답 %lu, 시간초과 %lu)  "
                          "손상 %lu(%.2f%%, MPU실패동시 %lu)  "
                          "최대변화 %.1f도 (dt %.1fms 설명가능 %.1f도 = %.1f배)",
                     (unsigned long)loop_n, wheel_vel, (unsigned long)enc_fail,
                     (unsigned long)mpu_fail, mpu_pct,
                     (unsigned long)mux_f, (unsigned long)nack, (unsigned long)tout,
                     (unsigned long)enc_bad, bad_pct, (unsigned long)bad_after_fail,
                     d_max_deg, d_max_dt_us * 1e-3f, explain_deg,
                     (explain_deg > 0.01f) ? d_max_deg / explain_deg : 0.0f);

            loop_n = enc_fail = mpu_fail = enc_bad = bad_after_fail = 0;
            d_max = 0.0f;
            d_max_dt_us = 0;
        }

        vTaskDelay(pdMS_TO_TICKS(LOOP_PERIOD_MS));
    }
}

void app_main(void)
{
    i2c_bus_init();
    ESP_ERROR_CHECK(encoder_init(I2C_PORT));
    ESP_ERROR_CHECK(mpu6050_init(I2C_PORT, MUX_CHANNELS, NUM_SENSORS));

    foc_init();

#if PWM_FREQ_OVERRIDE
    ESP_ERROR_CHECK(foc_set_pwm_freq(PWM_FREQ_OVERRIDE));
    ESP_LOGI(TAG, "PWM %d Hz", PWM_FREQ_OVERRIDE);
#endif

#if BUS_TEST
    bus_test();          // 모터를 켜지 않는다. 통신선만 본다
    return;
#endif

    foc_enable(true);

    float offset = align_rotor();
    ESP_LOGI(TAG, "토크 인가 시작 (uq ±%.1fV, %d초마다 방향 전환)",
             TEST_UQ, (int)(FLIP_US / 1000000));

    torque_loop(offset);
}

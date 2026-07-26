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
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/i2c.h"

#include "encoder.h"
#include "foc.h"

#define I2C_PORT    I2C_NUM_0
#define PIN_SDA     16
#define PIN_SCL     17
#define I2C_FREQ_HZ 100000

#define TEST_UQ        3.0f        // 인가할 q축 전압 [V]
#define FLIP_US        5000000     // 회전 방향 유지 시간. 정/역을 번갈아 들어본다
#define ALIGN_UD_V     2.0f        // 정렬 시 d축 전압
#define ALIGN_HOLD_MS  1000        // 회전자가 전기각 0 으로 끌려올 때까지 대기
#define STAT_US        1000000
#define VEL_LPF        0.2f

static const char *TAG = "MAIN";

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

static void torque_loop(float offset)
{
    int64_t prev_time = esp_timer_get_time();
    int64_t last_stat = prev_time;
    int64_t flip_time = prev_time;
    int     dir       = 1;

    uint32_t loop_n = 0, enc_fail = 0;
    float    d_max = 0.0f, wheel_vel = 0.0f;
    int64_t  d_max_dt_us = 0;

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

            ESP_LOGI(TAG, "loop %luHz  휠 %6.1f rad/s  실패 %lu  "
                          "최대변화 %.1f도 (dt %.1fms 설명가능 %.1f도 = %.1f배)",
                     (unsigned long)loop_n, wheel_vel, (unsigned long)enc_fail,
                     d_max_deg, d_max_dt_us * 1e-3f, explain_deg,
                     (explain_deg > 0.01f) ? d_max_deg / explain_deg : 0.0f);

            loop_n = enc_fail = 0;
            d_max = 0.0f;
            d_max_dt_us = 0;
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void app_main(void)
{
    i2c_bus_init();
    ESP_ERROR_CHECK(encoder_init(I2C_PORT));

    foc_init();
    foc_enable(true);

    float offset = align_rotor();
    ESP_LOGI(TAG, "토크 인가 시작 (uq ±%.1fV, %d초마다 방향 전환)",
             TEST_UQ, (int)(FLIP_US / 1000000));

    torque_loop(offset);
}

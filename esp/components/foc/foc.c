#include "foc.h"
#include <math.h>
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_log.h"


#define PIN_EN          5
#define PIN_IN1         18
#define PIN_IN2         19
#define PIN_IN3         23

#define PWM_FREQ_HZ     10000                   // 스위칭 주파수 [Hz]
#define PWM_RES         LEDC_TIMER_11_BIT
#define DUTY_MAX        2047                    // 11비트
#define PWM_MODE        LEDC_LOW_SPEED_MODE
#define PWM_TIMER       LEDC_TIMER_0

#define V_SUPPLY        11.6f                   // 공급 전압 [V]
#define POLE_PAIRS      7                       // 극쌍 수

static const int ch_gpio[3] = {PIN_IN1, PIN_IN2, PIN_IN3};


static float clampf(float x, float lo, float hi)
{
    if( x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

// 각도를 0 ~ 2π 로 접는다.
static float normalize_angle(float a)
{
    a = fmodf(a, 2.0f * (float)M_PI);
    return (a < 0.0f) ? a + 2.0f * (float)M_PI : a;
}


void foc_init(void)
{
    gpio_config_t foc_cfg = {
        .pin_bit_mask = 1ULL << PIN_EN,
        .mode = GPIO_MODE_OUTPUT
    };
    ESP_ERROR_CHECK(gpio_config(&foc_cfg));
    ESP_ERROR_CHECK(gpio_set_level(PIN_EN, 0));

    ESP_LOGI("FOC", "EN핀 설정 완료");

    ledc_timer_config_t ledc_cfg = {
        .speed_mode = PWM_MODE,
        .duty_resolution = PWM_RES,
        .freq_hz = PWM_FREQ_HZ,
        .timer_num = PWM_TIMER,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_cfg));
    ESP_LOGI("FOC", "LEDC 타이머 설정 완료");


    for(int i = 0; i<3; i++){
        ledc_channel_config_t ch_cfg = {
            .gpio_num = ch_gpio[i],
            .speed_mode = PWM_MODE,
            .timer_sel = PWM_TIMER,
            .channel = LEDC_CHANNEL_0 + i,
            .duty = 0,
            .hpoint = 0
        };
        ESP_ERROR_CHECK(ledc_channel_config(&ch_cfg));
        ESP_LOGI("FOC", "3상 %d번 채널 등록 완료", i);
    }
    ESP_LOGI("FOC", "3상 채널 등록 완료");
}


void foc_enable(bool on)
{
    if(!on){
        for(int i = 0; i<3; i++){
            ledc_set_duty(PWM_MODE, LEDC_CHANNEL_0 + i, 0);
            ledc_update_duty(PWM_MODE, LEDC_CHANNEL_0 + i);
        }
        gpio_set_level(PIN_EN, 0);
    } else {
        gpio_set_level(PIN_EN, 1);
    }
}


float foc_align(float angle_fwd, float angle_rev)
{
    float a = -angle_fwd * POLE_PAIRS;
    float b = -angle_rev * POLE_PAIRS;

    float off  = atan2f(sinf(a) + sinf(b), cosf(a) + cosf(b));
    float diff = atan2f(sinf(a - b), cosf(a - b));

    ESP_LOGI("FOC", "정렬 오프셋 %.1f도 (정방향/역방향 편차 %.1f도)",
             off * 180.0f / (float)M_PI, diff * 180.0f / (float)M_PI);

    return off;
}

void foc_set_phase_voltage(float ud, float uq, float angle_el)
{
    float c = cosf(normalize_angle(angle_el));
    float s = sinf(normalize_angle(angle_el));

    // 역 Park: (d,q) → (α,β)
    float u_alpha = c * ud - s * uq;
    float u_beta  = s * ud + c * uq;

    // 역 Clarke: (α,β) → 3상
    float u[3];
    u[0] = u_alpha;
    u[1] = -0.5f * u_alpha + 0.866025f * u_beta;
    u[2] = -0.5f * u_alpha - 0.866025f * u_beta;

    for (int i = 0; i < 3; i++) {
        float dc = clampf(u[i] / V_SUPPLY + 0.5f, 0.0f, 1.0f);
        ledc_set_duty(PWM_MODE, LEDC_CHANNEL_0 + i, (uint32_t)(dc * DUTY_MAX));
        ledc_update_duty(PWM_MODE, LEDC_CHANNEL_0 + i);
    }
}

// 토크 인가: uq 를 현재 전기각에 직접 인가.
void foc_apply_torque(float uq, float now_angle, float angle_offset)
{
    uq = clampf(uq, -V_SUPPLY, V_SUPPLY);
    float angle_el = normalize_angle(now_angle * POLE_PAIRS + angle_offset);
    foc_set_phase_voltage(0.0f, uq, angle_el);
}

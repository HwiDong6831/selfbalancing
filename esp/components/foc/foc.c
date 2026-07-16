#include "foc.h"
#include <math.h>
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_log.h"


#define PIN_EN          5
#define PIN_IN1         18
#define PIN_IN2         19
#define PIN_IN3         23

// PWM 설정
#define PWM_FREQ_HZ     22000                   // 가청주파수 위로 설정해서 모터에서 소리 안나게 함
#define PWM_RES         LEDC_TIMER_11_BIT       // pwm 주파수를 뽑으려면 11비트가 최대
#define DUTY_MAX        2047                    // 11비트
#define PWM_MODE        LEDC_LOW_SPEED_MODE
#define PWM_TIMER       LEDC_TIMER_0

// 전압
#define V_SUPPLY        11.3f                   // VM 실측
#define POLE_PAIRS      7                       // 2804 모터 (임시)


// PID
#define PID_KP          0.1f                    // 오차 반영 계수
#define PID_KI          0.0f                    // 
#define PID_KD          0.0f                    // 

static const int ch_gpio[3] = {PIN_IN1, PIN_IN2, PIN_IN3};


static float clampf(float x, float lo, float hi)
{
    if( x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

// 전기각을 라디안으로 변환
static float normalize_angle(float a)
{
    a = fmodf(a, 2.0f * (float)M_PI);
    return (a < 0.0f) ? a + 2.0f * (float)M_PI : a;
}


void foc_init(void)
{
    // esp32의 EN 핀(IO5) 설정
    gpio_config_t foc_cfg = {
        .pin_bit_mask = 1ULL << PIN_EN,
        .mode = GPIO_MODE_OUTPUT
    };
    ESP_ERROR_CHECK(gpio_config(&foc_cfg));
    ESP_ERROR_CHECK(gpio_set_level(PIN_EN, 0)); // PWM 설정이 완료되기 전까지 꺼둠(모터 오작동 방지)
    
    ESP_LOGI("FOC", "EN핀 설정 완료");

    // LEDC 타이머 설정
    ledc_timer_config_t ledc_cfg = {
        .speed_mode = PWM_MODE,
        .duty_resolution = PWM_RES,
        .freq_hz = PWM_FREQ_HZ,
        .timer_num = PWM_TIMER,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_cfg));
    ESP_LOGI("FOC", "LEDC 타이머 설정 완료");


    // 3상 채널 등록
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


float foc_align(float align_angle){
    return align_angle * POLE_PAIRS;
}

// 
void foc_set_phase_voltage(float ud, float uq, float angle_el)
{
    float c = cosf(normalize_angle(angle_el));
    float s = sinf(normalize_angle(angle_el));

    // 역 Park: (d,q) → (α,β)
    float u_alpha = c * ud - s * uq;
    float u_beta  = s * ud + c * uq;

    // 역 Clarke: (α,β) → 3상 (0.866025 = √3/2)
    float u[3];
    u[0] = u_alpha;
    u[1] = -0.5f * u_alpha + 0.866025f * u_beta;
    u[2] = -0.5f * u_alpha - 0.866025f * u_beta;

    // 전압 → duty (V_SUPPLY로 정규화 + 0.5 중심 시프트 + 클램프) 후 출력
    for (int i = 0; i < 3; i++) {
        float dc = clampf(u[i] / V_SUPPLY + 0.5f, 0.0f, 1.0f);
        ledc_set_duty(PWM_MODE, LEDC_CHANNEL_0 + i, (uint32_t)(dc * DUTY_MAX));
        ledc_update_duty(PWM_MODE, LEDC_CHANNEL_0 + i);
    }
}

float foc_openloop_velocity(float target_vel, float dt)
{
    // 전기각을 함수 밖에서도 유지해야 하므로 static 변수 사용
    static float angle_el = 0.0f;

    // 축속도 → 전기각 적분 (축 1바퀴 = 전기각 POLE_PAIRS 바퀴)
    angle_el = normalize_angle(angle_el + target_vel * POLE_PAIRS * dt);

    // ud=0, uq=2V
    foc_set_phase_voltage(0.0f, 2.0f, angle_el);

    return angle_el;
}

float foc_closeloop_velocity(float target_vel, float dt, float prev_angle, float now_angle, float angle_offset){
    // P
    // 각도 차이 계산(가까운쪽으로)
    float diff = fmodf(now_angle-prev_angle + (float)M_PI, 2.0f * M_PI);
    if(diff < 0.0f) diff += 2.0f * M_PI;
    diff -= (float)M_PI;

    float angle_vel =  diff / dt; 
    float error = target_vel - angle_vel;

    // I
    static float integral = 0.0f;
    integral += error * dt;

    // D
    static float prev_error = 0.0f;
    float derivative = (error - prev_error) / dt;
    prev_error = error;

    // 각 상황에 의도한 전압이 산출되도록 매크로 상수 조정 필요
    float uq = (PID_KP * error) + (PID_KI * integral) + (PID_KD * derivative);

    // 현재 전기각 계산
    float angle_el = normalize_angle(now_angle * POLE_PAIRS + angle_offset);
    // 현재 전기각에 uq 전압 인가
    foc_set_phase_voltage(0.0f, uq, angle_el);

    return angle_vel;
}
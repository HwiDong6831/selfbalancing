#include "encoder.h"
#include "driver/pulse_cnt.h"
#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_log.h"

#define PIN_A       21
#define PIN_B       22
#define PIN_Z       26

/*
 * 한 바퀴 카운트. MT6701 기본 1024PPR 을 4체배한 값이다.
 * 실측 전 가정값이므로 encoder_get_z 의 로그로 검증해야 한다.
 */
#define ENC_CPR     4096

// A/B 를 바꿔 달았으면 회전 방향이 반대로 세어진다. 실측 후 부호를 정한다.
#define ENC_DIR     (+1)

/*
 * 이보다 짧은 펄스는 잡음으로 보고 버린다.
 * 최고속 55rad/s 에서도 펄스 간격이 28us 라 1us 로는 정상 신호가 잘리지 않는다.
 * PM 이 꺼져 있어 APB 80MHz 고정이므로 필터 시간이 흔들리지 않는다.
 */
#define GLITCH_NS   1000

static const char *TAG = "ENC";

static pcnt_unit_handle_t s_unit;
static volatile uint32_t  s_z_count;
static volatile int       s_z_last;

/*
 * Z 상승엣지. 판단은 밖에서 하고 여기서는 카운트만 떠 놓는다.
 * IRAM_ATTR 을 안 붙인다 — pcnt_unit_get_count 가 flash 에 있어 IRAM ISR 에서는
 * 부를 수 없다. ISR 서비스도 IRAM 플래그 없이 설치하므로 이대로 안전하다.
 */
static void z_isr(void *arg)
{
    int c = 0;
    pcnt_unit_get_count(s_unit, &c);
    s_z_last = c;
    s_z_count++;
}

static esp_err_t channel_init(int edge_gpio, int level_gpio, bool flip)
{
    // 한 채널은 자기 엣지에서 세고 상대 레벨로 방향을 판단한다.
    // A/B 두 채널을 함께 쓰면 한 칸에 엣지가 4번 잡혀 분해능이 PPR 의 4배가 된다.
    pcnt_channel_handle_t ch;
    pcnt_chan_config_t cfg = {
        .edge_gpio_num  = edge_gpio,
        .level_gpio_num = level_gpio,
    };
    ESP_RETURN_ON_ERROR(pcnt_new_channel(s_unit, &cfg, &ch), TAG, "채널 생성");

    /*
     * 두 채널은 엣지 방향을 서로 반대로 걸어야 한다.
     * 같은 방향으로 걸면 A 가 센 만큼 B 가 되돌려 카운트가 0 근처에 묶인다.
     */
    ESP_RETURN_ON_ERROR(pcnt_channel_set_edge_action(ch,
            flip ? PCNT_CHANNEL_EDGE_ACTION_INCREASE     // 상승엣지
                 : PCNT_CHANNEL_EDGE_ACTION_DECREASE,
            flip ? PCNT_CHANNEL_EDGE_ACTION_DECREASE     // 하강엣지
                 : PCNT_CHANNEL_EDGE_ACTION_INCREASE),
        TAG, "엣지 동작");

    ESP_RETURN_ON_ERROR(pcnt_channel_set_level_action(ch,
            PCNT_CHANNEL_LEVEL_ACTION_KEEP,         // 상대가 high 면 위 방향 그대로
            PCNT_CHANNEL_LEVEL_ACTION_INVERSE),     // low 면 뒤집는다
        TAG, "레벨 동작");

    return ESP_OK;
}

esp_err_t encoder_init(void)
{
    /*
     * 상·하한을 ±CPR 로 걸면 한 바퀴마다 카운터가 스스로 0 으로 돌아간다.
     * 그래서 카운트가 곧 한 바퀴 안에서의 위치가 되고, 회전수를 따로 누적할 필요가 없다.
     */
    pcnt_unit_config_t unit_cfg = {
        .high_limit =  ENC_CPR,
        .low_limit  = -ENC_CPR,
    };
    ESP_RETURN_ON_ERROR(pcnt_new_unit(&unit_cfg, &s_unit), TAG, "유닛 생성");

    pcnt_glitch_filter_config_t filter_cfg = { .max_glitch_ns = GLITCH_NS };
    ESP_RETURN_ON_ERROR(pcnt_unit_set_glitch_filter(s_unit, &filter_cfg), TAG, "필터");

    ESP_RETURN_ON_ERROR(channel_init(PIN_A, PIN_B, false), TAG, "A 채널");
    ESP_RETURN_ON_ERROR(channel_init(PIN_B, PIN_A, true),  TAG, "B 채널");

    /*
     * 풀다운을 켜는 이유: 선이 안 붙어 핀이 떠 있으면 주변 잡음이 인터럽트를 일으킨다.
     * MT6701 의 ABZ 는 push-pull 출력이라 풀다운이 있어도 진짜 신호는 그대로 뜬다.
     */
    gpio_config_t z_cfg = {
        .pin_bit_mask = 1ULL << PIN_Z,
        .mode         = GPIO_MODE_INPUT,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type    = GPIO_INTR_POSEDGE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&z_cfg), TAG, "Z 핀");

    esp_err_t err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) return err;   // 이미 설치됐으면 통과
    ESP_RETURN_ON_ERROR(gpio_isr_handler_add(PIN_Z, z_isr, NULL), TAG, "Z ISR");

    ESP_RETURN_ON_ERROR(pcnt_unit_enable(s_unit), TAG, "enable");
    ESP_RETURN_ON_ERROR(pcnt_unit_clear_count(s_unit), TAG, "clear");
    ESP_RETURN_ON_ERROR(pcnt_unit_start(s_unit), TAG, "start");

    ESP_LOGI(TAG, "ABZ 엔코더 시작 (A%d B%d Z%d, %d카운트/회전)",
             PIN_A, PIN_B, PIN_Z, ENC_CPR);
    return ESP_OK;
}

int encoder_get_count(void)
{
    int c = 0;
    pcnt_unit_get_count(s_unit, &c);
    return ENC_DIR * c;
}

esp_err_t encoder_read_angle(float *angle_rad)
{
    float a = encoder_get_count() * (2.0f * (float)M_PI / ENC_CPR);
    *angle_rad = (a < 0.0f) ? a + 2.0f * (float)M_PI : a;   // 카운트는 음수일 수 있다

    // PCNT 는 하드웨어가 세므로 읽기 실패가 없다. 반환값은 호출부 호환용이다.
    return ESP_OK;
}

void encoder_get_z(uint32_t *count, int *last)
{
    *count = s_z_count;
    *last  = ENC_DIR * s_z_last;
}

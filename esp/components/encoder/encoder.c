#include "encoder.h"
#include "driver/pulse_cnt.h"
#include "esp_check.h"
#include "esp_log.h"

#define PIN_A       21
#define PIN_B       22

#define ENC_CPR     4096        // 한 바퀴 카운트
#define ENC_DIR     (+1)        // 회전 방향 부호
#define GLITCH_NS   1000        // 이것보다 짧은 펄스는 잡음으로 처리(버림)

static const char *TAG = "ENC";

static pcnt_unit_handle_t s_unit;

static esp_err_t channel_init(int edge_gpio, int level_gpio, bool flip)
{
    pcnt_channel_handle_t ch;
    pcnt_chan_config_t cfg = {
        .edge_gpio_num  = edge_gpio,
        .level_gpio_num = level_gpio,
    };
    ESP_RETURN_ON_ERROR(pcnt_new_channel(s_unit, &cfg, &ch), TAG, "채널 생성");

    ESP_RETURN_ON_ERROR(pcnt_channel_set_edge_action(ch,
            flip ? PCNT_CHANNEL_EDGE_ACTION_INCREASE
                 : PCNT_CHANNEL_EDGE_ACTION_DECREASE,
            flip ? PCNT_CHANNEL_EDGE_ACTION_DECREASE
                 : PCNT_CHANNEL_EDGE_ACTION_INCREASE),
        TAG, "엣지 동작");

    ESP_RETURN_ON_ERROR(pcnt_channel_set_level_action(ch,
            PCNT_CHANNEL_LEVEL_ACTION_KEEP,
            PCNT_CHANNEL_LEVEL_ACTION_INVERSE),
        TAG, "레벨 동작");

    return ESP_OK;
}

esp_err_t encoder_init(void)
{
    pcnt_unit_config_t unit_cfg = {
        .high_limit =  ENC_CPR,
        .low_limit  = -ENC_CPR,
    };
    ESP_RETURN_ON_ERROR(pcnt_new_unit(&unit_cfg, &s_unit), TAG, "유닛 생성");

    pcnt_glitch_filter_config_t filter_cfg = { .max_glitch_ns = GLITCH_NS };
    ESP_RETURN_ON_ERROR(pcnt_unit_set_glitch_filter(s_unit, &filter_cfg), TAG, "필터");

    ESP_RETURN_ON_ERROR(channel_init(PIN_A, PIN_B, false), TAG, "A 채널");
    ESP_RETURN_ON_ERROR(channel_init(PIN_B, PIN_A, true),  TAG, "B 채널");

    ESP_RETURN_ON_ERROR(pcnt_unit_enable(s_unit), TAG, "enable");
    ESP_RETURN_ON_ERROR(pcnt_unit_clear_count(s_unit), TAG, "clear");
    ESP_RETURN_ON_ERROR(pcnt_unit_start(s_unit), TAG, "start");

    ESP_LOGI(TAG, "AB 엔코더 시작 (A%d B%d, %d카운트/회전)", PIN_A, PIN_B, ENC_CPR);
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
    *angle_rad = (a < 0.0f) ? a + 2.0f * (float)M_PI : a;

    return ESP_OK;
}

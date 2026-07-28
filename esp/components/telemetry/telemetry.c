#include "telemetry.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "esp_websocket_client.h"

static const char *TAG = "TELE";

#define COMM_CORE       1       // 제어 루프는 core 0, 통신 루프는 core 1
#define TASK_STACK      4096
#define TASK_PRIO       3
#define JSON_BUF_SIZE   768

#define LOG_LINE_MAX    160
#define LOG_Q_LEN       48      // WS 연결 전 부팅 로그 저장 길이
#define LOG_DRAIN_MAX   8

#define WIFI_GOT_IP_BIT BIT0

static telemetry_config_t       s_cfg;
static EventGroupHandle_t       s_wifi_events;
static QueueHandle_t            s_frame_q;
static QueueHandle_t            s_log_q;
static esp_websocket_client_handle_t s_ws;

static char s_json[JSON_BUF_SIZE];   // 태스크 스택 절약용 정적 버퍼

static void wifi_event_handler(void *arg, esp_event_base_t base,
                               int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        xEventGroupClearBits(s_wifi_events, WIFI_GOT_IP_BIT);
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *e = (ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "WiFi 연결됨. IP = " IPSTR, IP2STR(&e->ip_info.ip));
        xEventGroupSetBits(s_wifi_events, WIFI_GOT_IP_BIT);
    }
}

static esp_err_t wifi_start_sta(void)
{
    // WiFi 드라이버가 보정값을 NVS 에 저장하므로 NVS 초기화가 선행됨
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    s_wifi_events = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
            WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
            IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL, NULL));

    wifi_config_t wc = {0};
    strlcpy((char *)wc.sta.ssid,     s_cfg.wifi_ssid,     sizeof(wc.sta.ssid));
    strlcpy((char *)wc.sta.password, s_cfg.wifi_password, sizeof(wc.sta.password));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wc));

    // wifi 절전 모드 off(절전모드 들어가면 느려짐)
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi 연결 대기: %s", s_cfg.wifi_ssid);
    xEventGroupWaitBits(s_wifi_events, WIFI_GOT_IP_BIT,
                        pdFALSE, pdTRUE, portMAX_DELAY);
    return ESP_OK;
}

static void ws_event_handler(void *arg, esp_event_base_t base,
                             int32_t id, void *data)
{
    switch (id) {
    case WEBSOCKET_EVENT_CONNECTED:
        ESP_LOGI(TAG, "서버 연결됨: %s", s_cfg.server_uri);
        break;
    case WEBSOCKET_EVENT_DISCONNECTED:
        ESP_LOGW(TAG, "서버 연결 끊김 (자동 재시도)");
        break;
    default:
        break;
    }
}

// nan 이면 snprintf 가 "nan" 을 뱉어 JSON 이 깨지고 서버가 프레임을 통째로 버린다.
static float fin(float v)
{
    return isfinite(v) ? v : 0.0f;
}

static const char *fault_name(telemetry_fault_t f)
{
    switch (f) {
    case TELEMETRY_FAULT_DROPOUT: return "dropout";
    default:                      return "none";
    }
}

static int build_json(const telemetry_frame_t *f, char *buf, size_t n)
{
    int len = snprintf(buf, n,
        "{\"ts\":%lld,\"sensors\":[",
        (long long)(esp_timer_get_time() / 1000));

    for (int i = 0; i < 3; i++) {
        const telemetry_sensor_t *s = &f->sensors[i];
        len += snprintf(buf + len, (len < (int)n) ? n - len : 0,
            "%s{\"ch\":%d,\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f,"
            "\"gx\":%.2f,\"gy\":%.2f,\"gz\":%.2f,\"fault\":\"%s\"}",
            (i ? "," : ""), s->ch,
            fin(s->ax), fin(s->ay), fin(s->az),
            fin(s->gx), fin(s->gy), fin(s->gz), fault_name(s->fault));
    }

    // voting 미구현. "ok" 로 두면 대시보드에 정상으로 보여 오해를 부른다.
    len += snprintf(buf + len, (len < (int)n) ? n - len : 0,
        "],\"voting\":{\"result\":\"n/a\",\"used\":[%d,%d,%d],\"rejected\":[]},"
        "\"balance\":{\"angle\":%.2f,\"rate\":%.2f,\"setpoint\":%.2f,\"uq\":%.3f},"
        "\"encoder\":{\"angle\":%.2f}}",
        f->sensors[0].ch, f->sensors[1].ch, f->sensors[2].ch,
        fin(f->angle), fin(f->rate), fin(f->setpoint), fin(f->uq),
        fin(f->enc_angle));

    return len;
}

static vprintf_like_t s_prev_vprintf;

/*
 * ESP_LOGx 후킹. 시리얼 출력은 유지하고 큐에 복사만 한다.
 *
 * WiFi/lwIP 내부를 포함한 아무 태스크에서나 불리므로 블록 금지.
 * 여기서 직접 WS 전송을 하면 전송이 찍는 에러 로그로 무한 재귀에 빠진다.
 * CONFIG_LOG_VERSION=1 기준 (한 줄 = 호출 1회).
 */
static int log_vprintf(const char *fmt, va_list args)
{
    if (s_log_q) {
        char line[LOG_LINE_MAX];
        va_list cp;
        va_copy(cp, args);
        int n = vsnprintf(line, sizeof(line), fmt, cp);
        va_end(cp);

        if (n > 0) {
            for (char *p = line; *p; p++) {
                if (*p == '\n' || *p == '\r') { *p = '\0'; break; }
            }
            if (line[0]) {
                xQueueSend(s_log_q, line, 0);   // 넘치면 버린다
            }
        }
    }
    return s_prev_vprintf(fmt, args);
}

static void json_escape(const char *src, char *dst, size_t dstn)
{
    size_t o = 0;
    for (const char *p = src; *p && o + 7 < dstn; p++) {
        unsigned char c = (unsigned char)*p;
        if (c == '"' || c == '\\') {
            dst[o++] = '\\';
            dst[o++] = (char)c;
        } else if (c < 0x20 || c == 0x7f) {
            o += snprintf(dst + o, dstn - o, "\\u%04x", c);
        } else {
            dst[o++] = (char)c;
        }
    }
    dst[o] = '\0';
}

static void send_pending_logs(void)
{
    char line[LOG_LINE_MAX];

    // 상한을 두는 이유: 전송이 에러 로그를 유발하면 큐가 계속 다시 채워진다.
    for (int i = 0; i < LOG_DRAIN_MAX; i++) {
        if (xQueueReceive(s_log_q, line, 0) != pdTRUE) {
            return;
        }
        int len = snprintf(s_json, sizeof(s_json), "{\"log\":\"");
        json_escape(line, s_json + len, sizeof(s_json) - len - 3);
        len += (int)strlen(s_json + len);
        len += snprintf(s_json + len, sizeof(s_json) - len, "\"}");

        esp_websocket_client_send_text(s_ws, s_json, len, pdMS_TO_TICKS(100));
    }
}

static void telemetry_task(void *arg)
{
    telemetry_frame_t frame;

    while (1) {
        // 로그도 흘려보내야 하므로 무한 대기 대신 타임아웃
        bool has_frame = (xQueueReceive(s_frame_q, &frame, pdMS_TO_TICKS(20)) == pdTRUE);

        // 미연결이어도 로그 큐는 비우지 않는다. WS 연결 전 부팅 로그가
        // 살아남아 연결 직후 한꺼번에 올라간다.
        if (!esp_websocket_client_is_connected(s_ws)) {
            continue;
        }

        if (has_frame) {
            int len = build_json(&frame, s_json, sizeof(s_json));
            if (len > 0 && len < (int)sizeof(s_json)) {
                esp_websocket_client_send_text(s_ws, s_json, len, pdMS_TO_TICKS(100));
            } else {
                ESP_LOGW(TAG, "JSON 버퍼 부족 (len=%d)", len);
            }
        }

        send_pending_logs();
    }
}

esp_err_t telemetry_start(const telemetry_config_t *cfg)
{
    if (!cfg || !cfg->wifi_ssid || !cfg->wifi_password || !cfg->server_uri) {
        return ESP_ERR_INVALID_ARG;
    }
    s_cfg = *cfg;

    ESP_ERROR_CHECK(wifi_start_sta());

    s_frame_q = xQueueCreate(1, sizeof(telemetry_frame_t));
    s_log_q   = xQueueCreate(LOG_Q_LEN, LOG_LINE_MAX);
    if (!s_frame_q || !s_log_q) {
        return ESP_ERR_NO_MEM;
    }

    esp_websocket_client_config_t ws_cfg = {
        .uri                  = s_cfg.server_uri,
        .reconnect_timeout_ms = 2000,
        .network_timeout_ms   = 3000,
        .task_stack           = TASK_STACK,
        .task_prio            = TASK_PRIO,
    };
    s_ws = esp_websocket_client_init(&ws_cfg);
    if (!s_ws) {
        return ESP_FAIL;
    }
    esp_websocket_register_events(s_ws, WEBSOCKET_EVENT_ANY, ws_event_handler, NULL);
    ESP_ERROR_CHECK(esp_websocket_client_start(s_ws));

    // 통신 코어에 고정 → 제어 루프(core 0) 타이밍에 영향 없음
    xTaskCreatePinnedToCore(telemetry_task, "telemetry",
                            TASK_STACK, NULL, TASK_PRIO, NULL, COMM_CORE);

    s_prev_vprintf = esp_log_set_vprintf(log_vprintf);
    return ESP_OK;
}

void telemetry_publish(const telemetry_frame_t *frame)
{
    if (s_frame_q) {
        xQueueOverwrite(s_frame_q, frame);   // 차 있어도 대기하지 않음
    }
}

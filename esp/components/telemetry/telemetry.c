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
#include "cJSON.h"

static const char *TAG = "TELE";

#define COMM_CORE       1       // 제어 루프는 core 0, 통신 루프는 core 1
#define TASK_STACK      4096
#define TASK_PRIO       3
#define JSON_BUF_SIZE   1024

#define LOG_LINE_MAX    160
#define LOG_Q_LEN       48      // WS 연결 전 부팅 로그 저장 길이
#define LOG_DRAIN_MAX   8

#define WIFI_GOT_IP_BIT BIT0
#define WIFI_TX_POWER   44      // 0.25dBm 단위 = 11dBm

/*
 * 이 시간 넘게 연결이 안 돌아오면 클라이언트를 통째로 재시작한다.
 * 자동 재접속(reconnect_timeout_ms 2초)은 클라이언트가 살아 있을 때만 돈다. 전송 중
 * 소켓이 사라지면 "already stop" 으로 굳어 재접속이 시작조차 안 된다.
 */
#define WS_DEAD_MS      5000

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

    /*
     * 송신 순간 전류가 센서 전원을 끌어내려 엔코더가 0 을 뱉고 I2C 가 깨졌다 (2026-07-29).
     * 단위 0.25dBm, 범위 8~84. 기본 80(20dBm) → 44(11dBm).
     * 공유기가 가까워(rssi -52) 이 출력으로도 충분하다. 접속이 불안하면 올린다.
     */
    ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(WIFI_TX_POWER));

    ESP_LOGI(TAG, "WiFi 연결 대기: %s", s_cfg.wifi_ssid);
    xEventGroupWaitBits(s_wifi_events, WIFI_GOT_IP_BIT,
                        pdFALSE, pdTRUE, portMAX_DELAY);
    return ESP_OK;
}

/*
 * 대시보드가 건 결함 주입. 채널 번호로 찾는다.
 * mode/rate 는 WS 태스크가 쓰고 제어 태스크가 읽는다 (telemetry.h 참조).
 */
static struct {
    int   ch;
    volatile telemetry_fault_t mode;
    volatile float rate;
} s_inject[3] = { { .ch = -1 }, { .ch = -1 }, { .ch = -1 } };

telemetry_fault_t telemetry_get_inject(int ch, float *rate)
{
    for (int i = 0; i < 3; i++) {
        if (s_inject[i].ch != ch) continue;
        if (rate) *rate = s_inject[i].rate;
        return s_inject[i].mode;
    }
    if (rate) *rate = 0.0f;
    return TELEMETRY_FAULT_NONE;
}

// {"cmd":"fault","ch":6,"mode":"drift","rate":0.5}
static void apply_command(const char *json, int len)
{
    cJSON *root = cJSON_ParseWithLength(json, len);
    if (!root) {
        ESP_LOGW(TAG, "명령 파싱 실패");
        return;
    }

    const cJSON *ch   = cJSON_GetObjectItem(root, "ch");
    const cJSON *mode = cJSON_GetObjectItem(root, "mode");
    if (cJSON_IsNumber(ch) && cJSON_IsString(mode)) {
        telemetry_fault_t m = TELEMETRY_FAULT_NONE;
        if      (!strcmp(mode->valuestring, "dropout")) m = TELEMETRY_FAULT_DROPOUT;
        else if (!strcmp(mode->valuestring, "freeze"))  m = TELEMETRY_FAULT_FREEZE;
        else if (!strcmp(mode->valuestring, "drift"))   m = TELEMETRY_FAULT_DRIFT;

        const cJSON *rate = cJSON_GetObjectItem(root, "rate");
        // 처음 보는 채널이면 빈 칸에 자리를 잡는다
        for (int i = 0; i < 3; i++) {
            if (s_inject[i].ch != ch->valueint && s_inject[i].ch != -1) continue;
            s_inject[i].ch   = ch->valueint;
            s_inject[i].rate = cJSON_IsNumber(rate) ? (float)rate->valuedouble : 0.0f;
            s_inject[i].mode = m;
            ESP_LOGW(TAG, "결함 주입 ch%d = %s", ch->valueint, mode->valuestring);
            break;
        }
    }
    cJSON_Delete(root);
}

static void ws_event_handler(void *arg, esp_event_base_t base,
                             int32_t id, void *data)
{
    switch (id) {
    case WEBSOCKET_EVENT_CONNECTED:
        ESP_LOGI(TAG, "서버 연결됨: %s", s_cfg.server_uri);
        break;
    case WEBSOCKET_EVENT_DATA: {
        // 텍스트 프레임만 본다. ping/pong/close 는 op_code 가 다르다.
        esp_websocket_event_data_t *d = data;
        if (d->op_code == 0x01 && d->data_len > 0) {
            apply_command(d->data_ptr, d->data_len);
        }
        break;
    }
    case WEBSOCKET_EVENT_DISCONNECTED:
        ESP_LOGW(TAG, "서버 연결 끊김 (자동 재시도)");
        break;
    case WEBSOCKET_EVENT_ERROR:
        // 재시작은 telemetry_task 가 한다. 여기는 WS 자신의 태스크라 스스로를 못 멈춘다.
        ESP_LOGW(TAG, "WS 오류");
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
    case TELEMETRY_FAULT_FREEZE:  return "freeze";
    case TELEMETRY_FAULT_DRIFT:   return "drift";
    default:                      return "none";
    }
}

// 판정 한 덩이. 가속도(axes 2)·자이로(axes 1)에 각각 쓴다.
static int vote_json(const telemetry_voting_t *v, const telemetry_sensor_t *s,
                     int axes, char *buf, size_t n)
{
    static const char *name[] = { "ok", "degraded", "single", "fail" };

    int len = snprintf(buf, n, "{\"result\":\"%s\",\"used\":[", name[v->result]);

    for (int i = 0, k = 0; i < 3; i++) {
        if (!v->used[i]) continue;
        len += snprintf(buf + len, (len < (int)n) ? n - len : 0,
                        "%s%d", (k++ ? "," : ""), s[i].ch);
    }
    len += snprintf(buf + len, (len < (int)n) ? n - len : 0, "],\"rejected\":[");

    for (int i = 0, k = 0; i < 3; i++) {
        if (v->used[i]) continue;
        len += snprintf(buf + len, (len < (int)n) ? n - len : 0,
                        "%s%d", (k++ ? "," : ""), s[i].ch);
    }
    len += snprintf(buf + len, (len < (int)n) ? n - len : 0, "],\"val\":[");

    for (int k = 0; k < axes; k++) {
        len += snprintf(buf + len, (len < (int)n) ? n - len : 0,
                        "%s%.4f", (k ? "," : ""), fin(v->val[k]));
    }
    len += snprintf(buf + len, (len < (int)n) ? n - len : 0, "]}");

    return len;
}

static int build_json(const telemetry_frame_t *f, char *buf, size_t n)
{
    int len = snprintf(buf, n,
        "{\"ts\":%lld,\"sensors\":[",
        (long long)(esp_timer_get_time() / 1000));

    for (int i = 0; i < 3; i++) {
        const telemetry_sensor_t *s = &f->sensors[i];
        len += snprintf(buf + len, (len < (int)n) ? n - len : 0,
            "%s{\"ch\":%d,\"ax\":%.4f,\"ay\":%.4f,\"az\":%.3f,"
            "\"gx\":%.2f,\"gy\":%.2f,\"gz\":%.2f,"
            "\"ax0\":%.4f,\"ay0\":%.4f,\"gz0\":%.2f,\"fault\":\"%s\"}",
            (i ? "," : ""), s->ch,
            fin(s->ax), fin(s->ay), fin(s->az),
            fin(s->gx), fin(s->gy), fin(s->gz),
            fin(s->ax0), fin(s->ay0), fin(s->gz0), fault_name(s->fault));
    }

    len += snprintf(buf + len, (len < (int)n) ? n - len : 0,
        "],\"voting\":{\"tol\":{\"accel\":%.4f,\"gyro\":%.2f},\"accel\":",
        fin(f->tol_accel), fin(f->tol_gyro));
    len += vote_json(&f->accel, f->sensors, 2, buf + len, (len < (int)n) ? n - len : 0);

    len += snprintf(buf + len, (len < (int)n) ? n - len : 0, ",\"gyro\":");
    len += vote_json(&f->gyro, f->sensors, 1, buf + len, (len < (int)n) ? n - len : 0);

    len += snprintf(buf + len, (len < (int)n) ? n - len : 0,
        "},\"balance\":{\"angle\":%.2f,\"rate\":%.2f,\"setpoint\":%.2f,\"uq\":%.3f},"
        "\"encoder\":{\"angle\":%.2f}}",
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

        // 실패하면 남은 것도 실패한다. 계속 밀어넣으면 그 에러 로그가 큐를 다시 채운다.
        if (esp_websocket_client_send_text(s_ws, s_json, len, pdMS_TO_TICKS(100)) < 0) {
            return;
        }
    }
}

static void telemetry_task(void *arg)
{
    telemetry_frame_t frame;
    int64_t last_conn = esp_timer_get_time();

    while (1) {
        // 로그도 흘려보내야 하므로 무한 대기 대신 타임아웃
        bool has_frame = (xQueueReceive(s_frame_q, &frame, pdMS_TO_TICKS(20)) == pdTRUE);

        // 미연결이어도 로그 큐는 비우지 않는다. WS 연결 전 부팅 로그가
        // 살아남아 연결 직후 한꺼번에 올라간다.
        if (!esp_websocket_client_is_connected(s_ws)) {
            if (esp_timer_get_time() - last_conn > WS_DEAD_MS * 1000LL) {
                ESP_LOGW(TAG, "WS 응답 없음 %d초 — 재시작", WS_DEAD_MS / 1000);
                esp_websocket_client_stop(s_ws);
                esp_websocket_client_start(s_ws);
                last_conn = esp_timer_get_time();
            }
            continue;
        }
        last_conn = esp_timer_get_time();

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

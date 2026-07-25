#include "telemetry.h"
#include "secrets.h"

#include <math.h>
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

#define COMM_CORE       1       // 제어 루프(app_main)는 core 0
#define TASK_STACK      4096
#define TASK_PRIO       3
#define JSON_BUF_SIZE   768

static EventGroupHandle_t       s_wifi_events;
#define WIFI_GOT_IP_BIT         BIT0

static QueueHandle_t            s_frame_q;   // 길이 1. 제어 코어 → 통신 코어
static esp_websocket_client_handle_t s_ws;

static char s_json[JSON_BUF_SIZE];   // 태스크 스택 절약용 정적 버퍼

/* ---------------------------------------------------------------- WiFi */

static void wifi_event_handler(void *arg, esp_event_base_t base,
                               int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        xEventGroupClearBits(s_wifi_events, WIFI_GOT_IP_BIT);   // 무한 재시도
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *e = (ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "WiFi 연결됨. IP = " IPSTR, IP2STR(&e->ip_info.ip));
        xEventGroupSetBits(s_wifi_events, WIFI_GOT_IP_BIT);
    }
}

static esp_err_t wifi_start_sta(void)
{
    // WiFi 드라이버는 보정값을 NVS 에 저장하므로 NVS 초기화가 선행돼야 한다.
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

    wifi_config_t wc = {
        .sta = {
            .ssid     = WIFI_SSID,
            .password = WIFI_PASSWORD,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wc));

    // 절전 모드를 끈다. 켜져 있으면 송신 지연이 수십~수백 ms 까지 튄다.
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi 연결 대기: %s", WIFI_SSID);
    xEventGroupWaitBits(s_wifi_events, WIFI_GOT_IP_BIT,
                        pdFALSE, pdTRUE, portMAX_DELAY);
    return ESP_OK;
}

/* ----------------------------------------------------------- WebSocket */

static void ws_event_handler(void *arg, esp_event_base_t base,
                             int32_t id, void *data)
{
    switch (id) {
    case WEBSOCKET_EVENT_CONNECTED:
        ESP_LOGI(TAG, "서버 연결됨: %s", SERVER_WS_URI);
        break;
    case WEBSOCKET_EVENT_DISCONNECTED:
        ESP_LOGW(TAG, "서버 연결 끊김 (자동 재시도)");
        break;
    default:
        break;
    }
}

/* ------------------------------------------------------------ 직렬화 */

/* nan 이면 snprintf 가 "nan" 을 뱉어 JSON 이 깨지고 프레임이 통째로 버려진다. */
static float fin(float v)
{
    return isfinite(v) ? v : 0.0f;
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
            "\"gx\":%.2f,\"gy\":%.2f,\"gz\":%.2f,\"fault\":\"none\"}",
            (i ? "," : ""), s->ch,
            fin(s->ax), fin(s->ay), fin(s->az),
            fin(s->gx), fin(s->gy), fin(s->gz));
    }

    // voting 은 FR-2 구현 전이라 고정값
    len += snprintf(buf + len, (len < (int)n) ? n - len : 0,
        "],\"voting\":{\"result\":\"ok\",\"used\":[%d,%d,%d],\"rejected\":[]},"
        "\"balance\":{\"angle\":%.2f,\"rate\":%.2f,\"setpoint\":%.2f,\"uq\":%.3f},"
        "\"encoder\":{\"angle\":%.2f}}",
        f->sensors[0].ch, f->sensors[1].ch, f->sensors[2].ch,
        fin(f->angle), fin(f->rate), fin(f->setpoint), fin(f->uq),
        fin(f->enc_angle));

    return len;
}

/* -------------------------------------------------------- 전송 태스크 */

static void telemetry_task(void *arg)
{
    telemetry_frame_t frame;

    while (1) {
        if (xQueueReceive(s_frame_q, &frame, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        if (!esp_websocket_client_is_connected(s_ws)) {
            continue;   // 미연결이면 버린다. 재연결은 클라이언트가 알아서.
        }

        int len = build_json(&frame, s_json, sizeof(s_json));
        if (len <= 0 || len >= (int)sizeof(s_json)) {
            ESP_LOGW(TAG, "JSON 버퍼 부족 (len=%d)", len);
            continue;
        }
        esp_websocket_client_send_text(s_ws, s_json, len, pdMS_TO_TICKS(100));
    }
}

/* ------------------------------------------------------------ 공개 API */

esp_err_t telemetry_start(void)
{
    ESP_ERROR_CHECK(wifi_start_sta());

    s_frame_q = xQueueCreate(1, sizeof(telemetry_frame_t));
    if (!s_frame_q) {
        return ESP_ERR_NO_MEM;
    }

    esp_websocket_client_config_t ws_cfg = {
        .uri                  = SERVER_WS_URI,
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
    return ESP_OK;
}

void telemetry_publish(const telemetry_frame_t *frame)
{
    if (s_frame_q) {
        xQueueOverwrite(s_frame_q, frame);   // 차 있어도 대기하지 않음
    }
}

/*
 * telemetry.h — 센서/제어 상태와 시리얼 로그를 Spring 서버로 WebSocket 송신
 *
 * core 0 = 제어 루프, core 1 = 통신. 둘 사이는 길이 1 큐로 잇고,
 * 제어 루프는 xQueueOverwrite 로 최신 프레임만 덮어써 대기 시간이 0이다.
 */
#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int   ch;               // TCA9548A mux 채널
    float ax, ay, az;       // [g]
    float gx, gy, gz;       // [deg/s]
} telemetry_sensor_t;

/* 서버의 SensorFrame record 와 1:1 대응. */
typedef struct {
    telemetry_sensor_t sensors[3];
    float angle;            // [deg]
    float rate;             // [deg/s]
    float setpoint;         // [deg]
    float uq;               // [V]
    float enc_angle;        // [deg]
} telemetry_frame_t;

/* WiFi 연결 → WS 시작 → core 1 전송 태스크 생성 → 로그 후킹.
 * WiFi 연결까지 블록되므로 제어 루프 시작 전에 호출할 것. */
esp_err_t telemetry_start(void);

/* 최신 프레임 등록 (논블로킹). 제어 루프에서 호출. */
void telemetry_publish(const telemetry_frame_t *frame);

#ifdef __cplusplus
}
#endif

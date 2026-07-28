#pragma once

#include <stdbool.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    const char *wifi_ssid;
    const char *wifi_password;
    const char *server_uri;     // ws://<서버 LAN IP>:8080/ws/esp
} telemetry_config_t;

// 서버 계약은 none|dropout|freeze|drift 네 가지지만, 판정 로직이 있는 건 dropout 뿐이다.
// freeze(값 고정)·drift(서서히 틀어짐)는 결함 주입 검증에서 붙인다.
typedef enum {
    TELEMETRY_FAULT_NONE = 0,
    TELEMETRY_FAULT_DROPOUT,    // 읽기 실패
} telemetry_fault_t;

typedef struct {
    int   ch;                   // TCA9548A mux 채널
    float ax, ay, az;           // [g]
    float gx, gy, gz;           // [deg/s]
    telemetry_fault_t fault;
} telemetry_sensor_t;

// voting 컴포넌트의 결과를 전송용으로 옮겨 담는다. telemetry 가 voting 에 의존하지 않도록.
typedef enum {
    TELEMETRY_VOTE_NA = 0,      // 아직 판정하지 않은 프레임
    TELEMETRY_VOTE_OK,
    TELEMETRY_VOTE_DEGRADED,
    TELEMETRY_VOTE_FAIL,
} telemetry_vote_t;

// 서버의 SensorFrame record 와 1:1 대응.
typedef struct {
    telemetry_sensor_t sensors[3];
    telemetry_vote_t   vote;
    bool               vote_used[3];
    float angle;                // [deg]
    float rate;                 // [deg/s]
    float setpoint;             // [deg]
    float uq;                   // [V]
    float enc_angle;            // [deg]
} telemetry_frame_t;

// WiFi 연결까지 블록하므로 제어 루프 시작 전에 호출.
// cfg 의 문자열은 호출자가 계속 유지해야 한다.
esp_err_t telemetry_start(const telemetry_config_t *cfg);

// 최신 프레임 등록. 논블로킹이므로 제어 루프에서 직접 호출해도 된다.
void telemetry_publish(const telemetry_frame_t *frame);

#ifdef __cplusplus
}
#endif

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

// dropout 만 실제 판정이 있고, freeze·drift 는 대시보드가 주입할 때만 나온다.
typedef enum {
    TELEMETRY_FAULT_NONE = 0,
    TELEMETRY_FAULT_DROPOUT,    // 읽기 실패 (실제 또는 주입)
    TELEMETRY_FAULT_FREEZE,     // 값 고정 (주입)
    TELEMETRY_FAULT_DRIFT,      // 서서히 틀어짐 (주입)
} telemetry_fault_t;

typedef struct {
    int   ch;                   // PCA9548A mux 채널
    float ax, ay, az;           // [g]      ax/ay 는 영점 뺀 비교값
    float gx, gy, gz;           // [deg/s]  gz 는 영점 뺀 비교값
    float ax0, ay0, gz0;        // 채널별 영점. 가속도는 중앙값 대비 차이다
    telemetry_fault_t fault;
} telemetry_sensor_t;

// voting_result_t 와 같은 순서. voting 컴포넌트에 의존하지 않으려고 따로 둔다.
typedef enum {
    TELEMETRY_VOTE_OK = 0,
    TELEMETRY_VOTE_DEGRADED,
    TELEMETRY_VOTE_SINGLE,
    TELEMETRY_VOTE_FAIL,
} telemetry_vote_t;

typedef struct {
    telemetry_vote_t result;
    bool  used[3];              // 채널별 채택 여부. 나머지가 rejected 로 나간다
    float val[2];               // 채택 평균. 자이로는 [0] 만 쓴다
} telemetry_voting_t;

// 서버의 SensorFrame record 와 1:1 대응.
typedef struct {
    telemetry_sensor_t sensors[3];
    telemetry_voting_t accel;   // 가속도쌍(ax,ay) 판정
    telemetry_voting_t gyro;    // 자이로(gz) 판정
    float angle;                // [deg]
    float rate;                 // [deg/s]
    float setpoint;             // [deg]
    float uq;                   // [V]
    float enc_angle;            // [deg]
    float tol_accel;            // 이상치 임계값 [g]
    float tol_gyro;             // 이상치 임계값 [deg/s]
} telemetry_frame_t;

// WiFi 연결까지 블록하므로 제어 루프 시작 전에 호출.
// cfg 의 문자열은 호출자가 계속 유지해야 한다.
esp_err_t telemetry_start(const telemetry_config_t *cfg);

// 최신 프레임 등록. 논블로킹이므로 제어 루프에서 직접 호출해도 된다.
void telemetry_publish(const telemetry_frame_t *frame);

/*
 * 대시보드가 그 채널에 걸어 둔 결함. 없으면 NONE.
 * rate 는 drift 전용으로 "초당 임계값의 몇 배로 벌어지는가" 다.
 *
 * WS 태스크가 쓰고 제어 태스크가 읽는다. 필드가 각각 정렬된 워드라 잠그지 않는다 —
 * 최악이라야 한 루프가 낡은 rate 를 쓴다.
 */
telemetry_fault_t telemetry_get_inject(int ch, float *rate);

#ifdef __cplusplus
}
#endif

# 센서 웹 서버 구조와 선택 근거

로봇의 센서 정보를 웹 페이지에서 실시간으로 보기 위한 별도 서버(`server/`)의 설계 결정을 정리한다. **무엇을 했는지보다 왜 그렇게 했는지**에 초점을 둔다.

## 1. 목적

- ESP32가 측정한 센서/제어 상태를 브라우저에 실시간 스트리밍한다.
- 결함 시뮬레이션(FR-2)·디지털 트윈(FR-8)의 공통 표시 기반으로 쓴다.
- 현재 단계: **ESP32 연동 전에 서버·웹을 먼저 완성**하고, 가짜 데이터로 검증한다.

## 2. 스택

| 항목 | 선택 |
|------|------|
| 언어/런타임 | Java 21 (Temurin) |
| 프레임워크 | Spring Boot 4.1.0 (webmvc + websocket) |
| 빌드 | Gradle |
| 직렬화 | Jackson 3 (`tools.jackson`, Spring Boot 4 기본) |
| 프론트 | 순수 HTML/CSS/JS, 정적 호스팅 (라이브러리 무의존) |

## 3. 아키텍처 선택 — Spring 허브

```
ESP32 ──WS──► Spring 서버 ──WS──► 브라우저
                    (중계 + 정적 호스팅)
```

두 가지 후보가 있었다.

- **(A) Spring 허브 (채택):** ESP32가 Spring으로 데이터를 보내고, Spring이 브라우저로 중계 + 웹 자산 호스팅.
- **(B) ESP32 직결:** ESP32가 직접 WebSocket 서버가 되고 브라우저가 붙음 (초기 `digital-twin.md` 안).

**(A)를 택한 이유:**

- ESP32 부담을 줄인다. 온보드 flash에 웹 자산을 안 올려도 되고, 동시 접속 수 제한(임베디드 WS 서버)에서 자유롭다.
- 다중 브라우저(PC + 모바일 동시)를 서버가 fan-out으로 감당한다.
- 로깅·기록·결함 주입 제어 등 확장 지점을 서버에 둘 수 있다.
- Spring Boot 스캐폴드에 이미 websocket starter가 포함돼 있어 자연스럽다.

## 4. 핵심 설계 — `TelemetrySink` 경계(seam)

데이터를 **만드는 쪽**과 **뿌리는 쪽**을 인터페이스 하나로 갈라 놓았다.

```java
public interface TelemetrySink {
    void publish(SensorFrame frame);
}
```

- **생산자**(현재 `DummyTelemetrySource`)는 이 인터페이스에만 의존한다. 뒤에서 WebSocket으로 어떻게 전송되는지 전혀 모른다.
- **구현체**(`DashboardWebSocketHandler`)가 받은 프레임을 접속한 모든 브라우저로 broadcast 한다.

**이 벽을 세운 이유 — 생산자 교체 비용을 0으로 만들기 위해서.** 지금은 더미가 `publish`를 부르고, ESP32 연동 시엔 ingest 핸들러가 같은 `publish`를 부른다. 그 사이 **전송·스키마·프론트 코드는 한 줄도 바뀌지 않는다**.

## 5. 데이터 계약 — `SensorFrame`

서버↔브라우저가 합의하는 JSON 한 프레임의 모양을 `record` 하나로 고정했다.

- `ts`, `sensors[3]`(MPU 6축 + fault), `voting`(used/rejected), `balance`(angle/rate/setpoint/uq), `encoder`.
- `balance` 필드는 펌웨어 `main.c` 변수와 1:1 대응.
- **계약을 한 곳에 못박은 이유:** 생산자·소비자·프론트가 각자 이 record 하나만 보고 맞추면 되므로, 필드 변경 시 파급 지점이 명확하다.

## 6. 더미 우선(mock-first) 전략

- `DummyTelemetrySource`가 `@Scheduled(fixedRate=33)`로 ≈30Hz 프레임을 생성해 `publish`.
- 값은 −10~10 **삼각파** 하나(`v`)를 모든 필드에 채우는 단순 형태. *실제처럼 보일 필요가 없으므로* 사인파·노이즈·센서 스케일 흉내를 걷어내고 최소화했다.
- `@ConditionalOnProperty(telemetry.mock.enabled)`로 켜고 끈다. **ESP32 연동 시 `false`로 두면 더미 빈이 아예 생성되지 않는다** — 코드 삭제 불필요.

**mock-first 이유:** 무선 펌웨어가 나오기 전에 서버·웹 전체 경로를 먼저 눈으로 검증하기 위해서.

## 7. 통신 — WebSocket + 순수 fan-out

- 프로토콜: **WebSocket** (저지연 연속 스트림, HTTP 폴링 대비 적합).
- 엔드포인트: `/ws/telemetry`.
- STOMP 같은 pub/sub 계층 없이 **순수 `TextMessage` fan-out**을 택했다. 채널이 하나(텔레메트리 단방향 브로드캐스트)뿐이라 STOMP는 오버킬.

## 8. 프론트엔드 — 무의존 정적 웹

- Spring이 `static/`을 자동 서빙. 앱 설치 불필요, 모바일 접속 가능.
- 차트는 외부 라이브러리 없이 `<canvas>`에 직접 그린다(최근 240프레임 링버퍼). CSP·오프라인·파일 수 측면에서 유리.
- 데이터 수신(30Hz)과 화면 렌더(`requestAnimationFrame`)를 분리해 부드럽게 그린다.
- 숫자 표시는 고정폭 + 부호 자리 확보(음수 `−`, 양수 figure space)로 값이 바뀌어도 흔들리지 않게 했다.

## 9. 동시성(스레드) 결정

서버는 멀티스레드다. 스케줄러 스레드(더미 생산)와 톰캣 연결 스레드(브라우저 접속/해제)가 공유 상태를 건드린다. 판단 기준은 **"이 데이터를 건드리는 스레드가 2명 이상인가"**.

| 대상 | 스레드 수 | 처리 | 이유 |
|------|-----------|------|------|
| `sessions` 집합 | 스케줄러 + 톰캣 다수 | `CopyOnWriteArraySet` | 순회(30Hz)가 잦고 변경(접속)이 드문 비대칭. 순회 중 변경돼도 예외·손상 없음 |
| 한 세션 전송 | 현재 1, 미래 다수 | `synchronized(session)` | `WebSocketSession.sendMessage`는 동시 호출에 안전하지 않음. 세션 단위로만 직렬화 |
| `v`, `dir` (더미) | 실질 1 (단일 스케줄러 풀) | 없음 | 겹쳐 실행되지 않아 경쟁 상태 없음. 불필요한 동기화 배제 |

`synchronized(session)`은 현재 생산자가 하나뿐이라 당장은 불필요하지만, ESP32 소스가 별도 스레드로 붙는 미래를 대비해 미리 건다(비용이 거의 없고, 누락 시 재현이 어려운 버그).

## 10. ESP32 연동 시 변경점 (설계의 결실)

1. `EspIngestHandler`(WebSocket ingest) 신설 → 받은 JSON을 `SensorFrame`으로 파싱 후 **동일한 `sink.publish(frame)`** 호출.
2. `application.properties`에서 `telemetry.mock.enabled=false`.

→ `DashboardWebSocketHandler`, `SensorFrame`, 프론트는 **불변**. 생산자만 교체.

## 11. 실행

```
JAVA_HOME = <Temurin 21 경로>
./gradlew.bat bootRun   →   http://localhost:8080
```


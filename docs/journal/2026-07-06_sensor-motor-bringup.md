# 작업 일지 - 2026-07-06 : 센서·엔코더·모터 브링업

삼각형 셀프밸런싱 로봇의 핵심 부품(IMU 3개, 마그네틱 엔코더, BLDC 모터+드라이버)을
ESP32(wemos_d1_mini32)에 하나씩 연결하며 인식·동작을 검증한 기록.

## 목표

- TCA9548A 멀티플렉서 경유 MPU6050 3개 가속도 읽기
- MT6701 마그네틱 엔코더(I2C) 각도 읽기
- SimpleFOC Mini 드라이버 + BLDC 모터 회전 검증

## 하드웨어 구성 / 배선 (검증 완료)

### I2C 버스 (공유)
- ESP32 **SDA=GPIO16, SCL=GPIO17** (`Wire.begin(16, 17)`) — 기본 21/22 아님, 주의
- 버스 속도 400kHz

| 장치 | 주소 | 연결 |
|------|------|------|
| TCA9548A 멀티플렉서 | 0x70 | A0/A1/A2 = GND (현재 미연결, floating 상태로도 0x70 잡힘) |
| MPU6050 ×3 | 0x68 | mux 채널 **0, 1, 6** 의 SDn/SCn. 각 AD0=GND |
| MT6701 엔코더 | 0x06 | 메인 버스 직결 (mux 안 거침). **IIC 솔더 점퍼 납땜 필수** |

### SimpleFOC Mini (3-PWM 드라이버)
- EN=GPIO5, IN1=GPIO18, IN2=GPIO19, IN3=GPIO23
- 모터 3상 → M1/M2/M3
- 드라이버 전원(VM) → 3S Lipo (~11.3V 실측), GND 공통 필수

## 시행착오 기록 (중요)

### 1. I2C 핀이 기본값 아님 (21/22 → 16/17)
- 증상: `mux NO ACK`, 메인 버스 스캔 `(none)`, 모든 장치 미검출
- 원인: 실제 배선은 GPIO16/17인데 `Wire.begin()` 기본값(21/22) 사용
- 해결: `Wire.begin(16, 17)` 명시

### 2. MPU6050 클론칩 WHO_AM_I = 0x70
- 증상: 채널 스캔에 0x68 뜨는데(장치 존재) `mpuInit()` FAIL
- 원인: 정품 MPU6050 WHO_AM_I=0x68, 이 클론칩은 **0x70** 반환. 코드가 0x68/0x69만 허용
- 해결: WHO_AM_I 체크 완화 — 통신 실패(0x00/0xFF)만 거르고 나머지 응답 통과
- 데이터 자체는 정상 (정지 시 az ≈ +1.00g 확인)

### 3. MT6701 기본 SSI 모드 → 버스 hang
- 증상: MT6701을 I2C 버스에 추가한 뒤 **전체 버스 마비**
  - SDA/SCL idle 1.7V (정상 3.3V), 모든 주소 `I2C hardware timeout`
  - VCC 3.3V 정상, SDA-SCL 단락 아님
- 원인: 이 MT6701 모듈은 **기본 SSI 모드**. 보드에 `IIC ->` 표기 + 솔더 점퍼 패드 2개.
  점퍼 미납땜 상태로 SDA/SCL에 물리면 칩이 SSI로 동작하며 라인을 당겨 버스 hang
- 해결: **`IIC ->` 솔더 점퍼 2개 납땜(브릿지)** → I2C 모드 활성화 → 정상
- 판별법: 문제 장치만 떼고 스캔해서 버스 살아나는지 확인

### 4. USB 연결 끊김 (코드 43 / Write timeout)
- 증상: 업로드 시 `A serial exception error: Write timeout`, 장치관리자 "알 수 없는 USB 장치(장치 설명자 요청 실패) 코드 43"
- 관찰: esptool이 COM5로 자동인식했으나 `pio device list` 상 COM5는 **Sony 장치(VID 054C)** — ESP32 아님. 진짜 ESP32 포트가 목록에 없었음
- 원인: USB 데이터 연결 불안정 (케이블/커넥터/노이즈). 전원 LED는 켜져도 데이터선 불량이면 이 증상
- 참고: ESP32 USB-시리얼 칩 VID = `10C4`(CP210x) 또는 `1A86`(CH340)
- 해결: 재연결 후 정상화 (케이블/포트 확인)

### 5. SimpleFOC BLDCMotor 오픈루프가 PWM 0 출력 (미해결, 우회)
- 증상: `motor.controller = velocity_openloop` + `motor.move()` 호출 시 IN1/2/3 전부 0V, 모터 무반응
  - `driver.init()` 성공, `MOT:Enable driver` 로그 정상, `motor.enable()` 추가해도 동일
- 격리 검증:
  - **`driver.setPwm(8,4,1)` 직접 호출은 정상** — IN1/IN2/IN3 = 2.2V/1.1V/0.27V (8:4:1 비율) → 드라이버·모터·PWM 하드웨어 전부 정상
  - 즉 SimpleFOC `BLDCMotor` 오픈루프 레이어만 0 출력
- 우회: motor 객체 버리고 **오픈루프 직접 구현** (전기각 sweep + SPWM을 `driver.setPwm`으로) → 모터 정상 회전
- TODO: 원인 규명 필요. FOC 클로즈드루프(`initFOC()`)는 경로가 다르므로 될 가능성 있음. 안 되면 재조사

## 최종 검증 결과

- ✅ MPU6050 ×3 (채널 0/1/6) 가속도 정상 (정지 az ≈ +1.00g)
- ✅ MT6701 각도 정상 (축 회전 시 0~360° 매끄럽게 변화)
- ✅ 드라이버 + 모터 회전 정상 (오픈루프 직접 구현으로 확인)

## 특이사항 / 주의

- **오픈루프는 발열 큼** — 회전자 위치 무시하고 전압 강제 인가 → 무효전류. 저속·정지 시 심함.
  검증용으로만 짧게. 전압 낮게(1~2V). FOC 가면 발열 급감
- MT6701 `IIC ->` 점퍼 납땜 안 하면 I2C 버스 전체가 죽으니 재조립 시 반드시 확인

## 다음 작업 (TODO)

1. FOC 클로즈드루프: MT6701 + 모터 정렬(`initFOC`), 위치/속도 제어
   - SimpleFOC `MagneticSensorI2C`로 MT6701(0x06, 14bit, reg 0x03/0x04) 설정
2. BLDCMotor 오픈루프 0V 원인 규명 (시행착오 #5)
3. 모터 pole pairs 확정 (현재 코드 임시값 7)
4. mux A0/A1/A2 → GND 확정 연결 (현재 floating)
5. 3개 IMU 융합 → 기울기 추정 → 밸런싱 제어 루프

## 코드 상태

- `src/main.cpp` : 현재 오픈루프 직접 구현(모터 검증용). 이전 단계 코드(MPU/MT6701 읽기)는
  순차적으로 덮어써짐 — 필요 시 이 일지의 배선/설정 참고해 복원
- `src/pins.h`, `src/mpu6050.h` : env 분리 시도 때 만든 헬퍼. 현재 main.cpp 미참조(orphan).
  핀·주소·MPU 헬퍼 참고용으로 남겨둠

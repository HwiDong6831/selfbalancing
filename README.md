# 삼각형 셀프밸런싱 로봇

1축 밸런싱 휠을 통해 스스로 중심을 잡는 삼각형 로봇.

### 특징
외부 충격에 실시간으로 대응하여 균형을 유지.

가속도 센서를 3개 탑재하여, 센싱된 정보를 다수결로 결정하여 판단하도록 설계.

일부 센서가 고장나거나 오작동하더라도 로봇 동작이 정상적으로 이뤄지도록 하는 것이 목적.

## 언어 / 빌드 환경

| 항목 | 내용 |
|------|------|
| 언어 | C++ |
| 플랫폼 | PlatformIO (VSCode) |
| 프레임워크 | Arduino framework |
| 타겟 보드 | ESP32 D1 Mini Live |

## 라이브러리

| 라이브러리 | 용도 |
|-----------|------|
| SimpleFOC (Arduino-FOC) | FOC 모터제어 |
| SimpleFOC MagneticSensorI2C / SPI | MT6701 자기인코더 |
| Adafruit_MPU6050 | MPU6050 |

## 주요 부품
메인보드: ESP32 D1 Mini Live

FOC 드라이버: SimpleFOC Mini

멀티플렉서: PCA9548A

가속도센서: MPU6050

자기인코더: MT6701

모터: 2804-100KV 브러시리스

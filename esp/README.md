# esp — 밸런싱 로봇 펌웨어 (ESP-IDF v5.5)

ESP32 D1 Mini 에서 도는 제어 펌웨어. core 0 이 제어 루프, core 1 이 통신을 맡는다.

## 구조

```
main/           app_main. 초기화 → 캘리브레이션 → 1kHz 제어 루프
components/
  mpu6050/      MPU6500 ×3 (PCA9548A mux 0x70)
  encoder/      MT6701 자기 인코더 (0x06)
  foc/          전압모드 FOC (LEDC 3-PWM + EN)
  balance/      상보필터 각도 추정 + 토크모드 상태피드백
  telemetry/    WiFi STA + WebSocket 송신 + ESP_LOGx 미러링
```

## 빌드

`export.ps1` 이 py3.14 가상환경을 찾는데 실제 설치된 건 py3.11 이라 경로를 먼저 지정해야 한다.

```powershell
$env:IDF_PYTHON_ENV_PATH="C:\Espressif\python_env\idf5.5_py3.11_env"
. C:\Espressif\frameworks\esp-idf-v5.5.4\export.ps1

idf.py build
idf.py -p COM<n> flash monitor
```

WiFi/서버 주소는 커밋되지 않는다.
아래 명령어로 secrets.h 파일을 생성한 후, 내부에 로봇을 연결할 WiFi 및 서버 정보를 기입한다.
```
cp main/secrets.h.example main/secrets.h
```

## 주의

- 튜닝 중 통신을 완전히 배제하려면 `main.c` 의 `TELEMETRY_ENABLED` 를 0 으로 변경한다.

# 2026-07-16 : FOC 클로즈루프 구현

### 문제 1. foc 정렬 과정
- 시도: 클로즈루프 로직 구현 후 동작 테스트
- 증상/원인: 전압은 인가되나, 제자리에서 떨림 문제
- 이유: encoder 값이 증가하는 방향과 모터 전기각이 반대
- 해결: encoder_read_angle이 계산하는 방향을 반대 방향으로 구현


### 문제 2. i2c 버스 충돌 문제 
- 증상: 
```
E (441401) i2c.master: I2C software timeout
W (441401) i2c.common: GPIO 16 is not usable, maybe conflict with others
W (441401) i2c.common: GPIO 17 is not usable, maybe conflict with others
```
- 원인: 
- 해결: 
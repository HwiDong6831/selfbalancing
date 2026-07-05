/**
 * 검증 2단계(대안): 오픈루프 직접 구현 (motor 객체 우회)
 *
 * driver.setPwm 직접은 정상 확인됨. SimpleFOC BLDCMotor 오픈루프가
 * PWM 0 을 내는 문제 우회 -> 전기각을 직접 sweep 해서 SPWM 생성.
 *
 * ⚠️ 모터 고정 후. 회전으로 판정.
 * 배선: EN=5, IN1=18, IN2=19, IN3=23, 모터->M1/M2/M3, VM->배터리
 *
 * 시리얼: T<속도rad/s>  V<전압>  S(정지)
 */
#include <Arduino.h>
#include <SimpleFOC.h>

constexpr int PIN_EN  = 5;
constexpr int PIN_IN1 = 18;
constexpr int PIN_IN2 = 19;
constexpr int PIN_IN3 = 23;

constexpr float SUPPLY_VOLTAGE = 12.0f;
constexpr int   POLE_PAIRS     = 7;

BLDCDriver3PWM driver = BLDCDriver3PWM(PIN_IN1, PIN_IN2, PIN_IN3, PIN_EN);

float target_velocity = 3.0f;   // 기계 rad/s
float motor_voltage = 3.0f;     // 인가 전압
float shaft_angle = 0;          // 기계각 누적
long  last_us = 0;

// SPWM: Uq(=voltage), 전기각으로 3상 전압 생성 후 setPwm
void setPhaseVoltage(float Uq, float angle_el) {
  float Ualpha = -Uq * sin(angle_el);
  float Ubeta  =  Uq * cos(angle_el);
  float center = SUPPLY_VOLTAGE / 2.0f;
  // Clarke 역변환 + 중심 오프셋
  float Ua = Ualpha + center;
  float Ub = -0.5f * Ualpha + (_SQRT3_2) * Ubeta + center;
  float Uc = -0.5f * Ualpha - (_SQRT3_2) * Ubeta + center;
  driver.setPwm(Ua, Ub, Uc);
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println(F("\n=== 검증2(대안): 오픈루프 직접 ==="));

  SimpleFOCDebug::enable(&Serial);
  driver.voltage_power_supply = SUPPLY_VOLTAGE;
  driver.voltage_limit = SUPPLY_VOLTAGE;
  if (!driver.init()) {
    Serial.println(F("[FAIL] driver.init()"));
    return;
  }
  driver.enable();
  last_us = micros();
  Serial.println(F("[OK] driver init+enable. T<속도>/V<전압>/S"));
  Serial.printf("start: target=%.1f rad/s, V=%.1f\n", target_velocity, motor_voltage);
}

void handleSerial() {
  if (!Serial.available()) return;
  char c = Serial.read();
  if (c == 'T') { target_velocity = Serial.parseFloat();
                  Serial.printf("target=%.2f rad/s\n", target_velocity); }
  else if (c == 'V') { motor_voltage = Serial.parseFloat();
                  Serial.printf("V=%.2f\n", motor_voltage); }
  else if (c == 'S') { target_velocity = 0; Serial.println("STOP"); }
}

void loop() {
  long now = micros();
  float dt = (now - last_us) * 1e-6f;
  if (dt <= 0 || dt > 0.5f) dt = 1e-3f;
  last_us = now;

  shaft_angle += target_velocity * dt;               // 기계각 적분
  float angle_el = shaft_angle * POLE_PAIRS;          // 전기각
  setPhaseVoltage(motor_voltage, angle_el);

  handleSerial();
}

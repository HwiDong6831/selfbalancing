/**
 * MPU6050 헬퍼 (TCA9548A 멀티플렉서 경유) - 헤더 온리
 *
 * Wire.begin() 은 호출측에서 먼저 실행. 채널 선택 -> init/read.
 */
#pragma once
#include <Arduino.h>
#include <Wire.h>
#include "pins.h"

// MPU6050 레지스터
constexpr uint8_t MPU_REG_PWR_MGMT_1   = 0x6B;
constexpr uint8_t MPU_REG_ACCEL_CONFIG = 0x1C;
constexpr uint8_t MPU_REG_ACCEL_XOUT_H = 0x3B;
constexpr uint8_t MPU_REG_WHO_AM_I     = 0x75;

constexpr float MPU_ACCEL_SENS = 16384.0f;  // ±2g: 16384 LSB/g

// 멀티플렉서 채널 선택 (해당 채널만 활성)
inline void tcaSelect(uint8_t ch) {
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << ch);
  Wire.endTransmission();
}

inline void mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

inline uint8_t mpuRead(uint8_t reg) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (uint8_t)1);
  return Wire.available() ? Wire.read() : 0;
}

// 현재 선택된 채널의 MPU6050 초기화. 성공=true
inline bool mpuInit() {
  uint8_t who = mpuRead(MPU_REG_WHO_AM_I);
  if (who != 0x68 && who != 0x69) return false;
  mpuWrite(MPU_REG_PWR_MGMT_1, 0x00);    // sleep 해제
  delay(10);
  mpuWrite(MPU_REG_ACCEL_CONFIG, 0x00);  // ±2g
  delay(10);
  return true;
}

// 가속도 3축 raw 읽기
inline void mpuReadAccel(int16_t &ax, int16_t &ay, int16_t &az) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_REG_ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (uint8_t)6);
  ax = (Wire.read() << 8) | Wire.read();
  ay = (Wire.read() << 8) | Wire.read();
  az = (Wire.read() << 8) | Wire.read();
}

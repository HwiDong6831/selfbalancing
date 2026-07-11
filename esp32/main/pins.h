/**
 * 공통 핀/주소 정의 - 삼각형 셀프밸런싱 로봇
 */
#pragma once

#include <stdint.h>

// --- I2C (TCA9548A + MPU6050) ---
constexpr int PIN_SDA = 16;
constexpr int PIN_SCL = 17;

constexpr uint8_t TCA_ADDR = 0x70;  // TCA9548A (A0=A1=A2=GND)
constexpr uint8_t MPU_ADDR = 0x68;  // MPU6050 (AD0=GND)

// MPU6050 가 연결된 멀티플렉서 채널
constexpr uint8_t MUX_CHANNELS[] = {0, 1, 6};
constexpr uint8_t NUM_SENSORS = sizeof(MUX_CHANNELS) / sizeof(MUX_CHANNELS[0]);

// --- SimpleFOC Mini (3-PWM 드라이버) ---
constexpr int PIN_EN  = 5;
constexpr int PIN_IN1 = 18;
constexpr int PIN_IN2 = 19;
constexpr int PIN_IN3 = 23;

// --- 전원 ---
constexpr float SUPPLY_VOLTAGE = 12.0f;  // 3S Lipo ~12V
constexpr float TEST_VOLTAGE   = 3.0f;   // 테스트 기준 전압(낮게)

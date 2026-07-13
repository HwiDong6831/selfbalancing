/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/i2c_master.h"


#define I2C_PORT        I2C_NUM_0
#define PIN_SDA         16
#define PIN_SCL         17
#define MUX_ADDR        0x70
#define MPU_ADDR        0x68

#define REG_PWR_MGMT_1  0x6B
#define REG_WHOAMI      0x75
#define REG_ACCEL_XOUT  0x3B

static const int channels[3] = {0, 1, 6};

static i2c_master_bus_handle_t bus;
static i2c_master_dev_handle_t mux_dev;
static i2c_master_dev_handle_t mpu_dev;

void app_main(void)
{
    // I2C 버스 생성 (핀 16, 17)
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_PORT,
        .sda_io_num = PIN_SDA,
        .scl_io_num = PIN_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus));

    // 멀티플렉서 디바이스 등록
    i2c_device_config_t mux_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MUX_ADDR,
        .scl_speed_hz = 400000
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus,&mux_cfg,&mux_dev));

    // MPU 디바이스 등록
    i2c_device_config_t mpu_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_7,
        .device_address = MPU_ADDR,
        .scl_speed_hz = 400000
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &mpu_cfg, &mpu_dev));

    // 채널마다 MPU wake, 신원확인
    for(int i = 0; i<3; i++){
        int ch = channels[i];

        // MUX 채널 선택
        uint8_t ch_mask = (uint8_t)(1 << ch);
        ESP_ERROR_CHECK(i2c_master_transmit(mux_dev, &ch_mask, 1, 100));
        

        // MPU wake
        uint8_t buf[2] = {REG_PWR_MGMT_1, 0x00};
        ESP_ERROR_CHECK(i2c_master_transmit(mpu_dev, buf, 2, 100));

        uint8_t who = 0;
        uint8_t reg_who_am_i = (uint8_t)REG_WHOAMI;
        esp_err_t err = i2c_master_transmit_receive(mpu_dev, &reg_who_am_i, 1, &who, 1, 100);
        if(err == ESP_OK){
            ESP_LOGI("MPU", "채널 %d : %x", ch, who);
        } else {
            ESP_LOGE("MPU", "채널 %d : 읽기 실패");
        }
    }


    // 루프 돌면서 각 채널 순회, 값 출력

    int16_t ax[3] = {0}, ay[3] = {0}, az[3] = {0};
    TickType_t last = xTaskGetTickCount();

    while(1){
        for(int i = 0; i<3; i++){
            int ch = channels[i];

            // MUX 채널 선택
            uint8_t ch_mask = (uint8_t)(1 << ch);
            i2c_master_transmit(mux_dev, &ch_mask, 1, 100);

            uint8_t raw[6] = {0};
            uint8_t reg_accel_out = (uint8_t)REG_ACCEL_XOUT;
            esp_err_t err = i2c_master_transmit_receive(mpu_dev, &reg_accel_out, 1, raw, 6, 100);
            if(err==ESP_OK){
                ax[i] = (int16_t)(raw[0]<<8 | raw[1]);
                ay[i] = (int16_t)(raw[2]<<8 | raw[3]);
                az[i] = (int16_t)(raw[4]<<8 | raw[5]);
            } else {
                ESP_LOGE("MPU", "[채널 %d] 값 읽기 실패");
            }
        }
        
        if(xTaskGetTickCount() - last > pdMS_TO_TICKS(200)) {
            ESP_LOGI("MPU", "X: %6d  Y: %6d  Z: %6d", (ax[0]+ax[1]+ax[2])/3, (ay[0]+ay[1]+ay[2])/3, (az[0]+az[1]+az[2])/3);

            // for(int i = 0; i<3; i++){
            //     ESP_LOGI("MPU", "[채널 %d] X: %d  Y: %d  Z: %d", i, ax[i], ay[i], az[i]);
            // }

            last = xTaskGetTickCount();
        }

        vTaskDelay(pdMS_TO_TICKS(10));      // 센싱 주기
    }

}

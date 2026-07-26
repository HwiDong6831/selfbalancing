#include "esp_log.h"
#include "driver/i2c.h"

#include "encoder.h"
#include "mpu6050.h"
#include "foc.h"

#define I2C_PORT    I2C_NUM_0
#define PIN_SDA     16
#define PIN_SCL     17
#define I2C_FREQ_HZ 100000

static const char *TAG = "MAIN";

// mux 채널 번호. 0 번이 제어용, 나머지 둘은 표시용이다.
static const int MUX_CHANNELS[3] = {0, 1, 6};
static const int NUM_SENSORS     = 3;

static void i2c_bus_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = PIN_SDA,
        .scl_io_num = PIN_SCL,
        .sda_pullup_en = true,
        .scl_pullup_en = true,
        .master.clk_speed = I2C_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_PORT, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_PORT, I2C_MODE_MASTER, 0, 0, 0));
}

void app_main(void)
{
    i2c_bus_init();
    ESP_ERROR_CHECK(encoder_init(I2C_PORT));
    ESP_ERROR_CHECK(mpu6050_init(I2C_PORT, MUX_CHANNELS, NUM_SENSORS));

    // foc_init 은 EN 을 내린 상태로 끝난다. 제어 루프가 생기기 전까지 켜지 않는다.
    foc_init();

    ESP_LOGI(TAG, "초기화 완료");
}

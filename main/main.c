#include <stdio.h>
#include <stdint.h>
#include "driver/i2c.h"
#include "driver/adc.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ADC------------------------------------------
#define ADC_CHANNEL ADC_CHANNEL_0

const static char *TAG1 = "adc";
// ---------------------------------------------
// DIGITAL--------------------------------------
#define IR_SENSOR_GPIO GPIO_NUM_4

const static char *TAG2 = "IR";
// ---------------------------------------------
// I2C------------------------------------------
// I2C and BMP280 Configuration
#define BMP280_I2C_ADDR        0x76
#define BMP280_REG_PRESS_MSB   0xF7
#define BMP280_REG_CALIB_START 0x88
#define BMP280_REG_CTRL_MEAS   0xF4
#define I2C_MASTER_NUM         I2C_NUM_0
#define I2C_MASTER_SCL_IO      9
#define I2C_MASTER_SDA_IO      8
#define I2C_MASTER_FREQ_HZ     100000

static const char *TAG3 = "BMP280";
typedef struct { uint16_t T1; int16_t T2, T3} bmp280_calib_t;
static bmp280_calib_t calib;
// ---------------------------------------------

// ADC: Water Sensor------------------------------------------------------------------------------------------------------
void ADC_Water_Sensor(void*param){

 adc1_config_width(ADC_WIDTH_BIT_12);
 adc1_config_channel_atten(ADC_CHANNEL, ADC_ATTEN_DB_11);

 while (1)
 {
    int adc_reading = adc1_get_raw(ADC_CHANNEL);

    ESP_LOGI(TAG1, "ADC Value: %d\n", adc_reading);
    vTaskDelay(pdMS_TO_TICKS(1000));
 }
 vTaskDelete(NULL);
}
// -----------------------------------------------------------------------------------------------------------------------
// DIGITAL: IR Sensor-----------------------------------------------------------------------------------------------------
void DIGITAL_IR_Sensor(void*param){
    // Configure GPIO pin as input    
    gpio_config_t ir_config = {0};
    ir_config.mode = GPIO_MODE_INPUT;
    ir_config.pin_bit_mask = 1ULL << IR_SENSOR_GPIO;
    ir_config.pull_up_en = 0x0;
    ir_config.pull_down_en = 0x0;
    
    gpio_config(&ir_config);

    while (1) {
        int ir_state = gpio_get_level(IR_SENSOR_GPIO); // Read IR sensor state
        ESP_LOGI(TAG2, "IR Sensor State: %s\n", ir_state ? "HIGH" : "LOW");
        vTaskDelay(pdMS_TO_TICKS(2000)); // 500ms delay
    }
    vTaskDelete(NULL);
}
// -----------------------------------------------------------------------------------------------------------------------
// I2C: BMP280 (Temperature)----------------------------------------------------------------------------------------------
void i2c_master_init() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER, .sda_io_num = I2C_MASTER_SDA_IO, .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE, .scl_pullup_en = GPIO_PULLUP_ENABLE, .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

esp_err_t i2c_read(uint8_t reg, uint8_t *data, size_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BMP280_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BMP280_I2C_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, len, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t i2c_write(uint8_t reg, uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BMP280_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return ret;
}

void bmp280_read_calibration() {
    uint8_t calib_raw[24];
    i2c_read(BMP280_REG_CALIB_START, calib_raw, 24);
    calib.T1 = (calib_raw[1] << 8) | calib_raw[0];
    calib.T2 = (calib_raw[3] << 8) | calib_raw[2];
    calib.T3 = (calib_raw[5] << 8) | calib_raw[4];
}

int32_t bmp280_compensate_temperature(int32_t adc_T, int32_t *t_fine) {
    int32_t var1 = ((((adc_T >> 3) - ((int32_t)calib.T1 << 1))) * ((int32_t)calib.T2)) >> 11;
    int32_t var2 = (((((adc_T >> 4) - ((int32_t)calib.T1)) * ((adc_T >> 4) - ((int32_t)calib.T1))) >> 12) * ((int32_t)calib.T3)) >> 14;
    *t_fine = var1 + var2;
    return (*t_fine * 5 + 128) >> 8;
}

void I2C_BPM280(void*param){
    ESP_LOGI(TAG3, "Initializing BMP280...");
    i2c_master_init();
    bmp280_read_calibration();
    i2c_write(BMP280_REG_CTRL_MEAS, 0x27); // Normal mode, temp+press oversampling x1

    while (1) {
        uint8_t data[6];
        if (i2c_read(BMP280_REG_PRESS_MSB, data, 6) == ESP_OK) {
            int32_t adc_T = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4);
            int32_t t_fine;
            int32_t temp = bmp280_compensate_temperature(adc_T, &t_fine);
            ESP_LOGI(TAG3, "Temperature: %.2f °C", temp / 100.0);
        } else {
            ESP_LOGE(TAG3, "Failed to read BMP280 data");
        }
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
    vTaskDelete(NULL);
}
// -----------------------------------------------------------------------------------------------------------------------
void app_main(){
 xTaskCreate(I2C_BPM280, "BPM280", 2048, NULL, 5, NULL);
 xTaskCreate(DIGITAL_IR_Sensor, "IR_Sensor", 2048, NULL, 5, NULL);
 xTaskCreate(ADC_Water_Sensor, "Water_Sensor", 2048, NULL, 5, NULL);
}

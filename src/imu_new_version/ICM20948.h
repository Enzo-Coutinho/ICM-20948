#ifndef ICM20948_h
#define ICM20948_h

#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "registers_icm20948.h"

gpio_num_t i2c_sda_port = GPIO_NUM_21;
gpio_num_t i2c_scl_port = GPIO_NUM_22;
uint32_t timeout = 1000 / portTICK_PERIOD_MS;

i2c_master_bus_config_t i2c_mst_config = {
    .i2c_port = -1,
    .sda_io_num = i2c_sda_port,
    .scl_io_num = i2c_scl_port,
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .glitch_ignore_cnt = 7,
    .flags.enable_internal_pullup = true,
};

i2c_master_bus_handle_t i2c_bus_handle;

i2c_device_config_t icm20948_dev_cfg = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = 0x69,
    .scl_speed_hz = 100000,
};

i2c_master_dev_handle_t icm20948_dev_handle;

void initialize(); //

esp_err_t write_register(uint8_t reg, uint8_t data); //
esp_err_t read_register(uint8_t reg, uint8_t len, uint8_t *data); //


#endif
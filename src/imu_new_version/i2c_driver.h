#ifndef i2c_driver_h
#define i2c_driver_h

#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "registers_icm20948.h"
#include "esp32-hal-log.h"

extern gpio_num_t i2c_sda_port;
extern gpio_num_t i2c_scl_port;
extern uint32_t timeout;

extern i2c_master_bus_config_t i2c_mst_config;

extern i2c_master_bus_handle_t i2c_bus_handle;

void i2c_initialize(const i2c_device_config_t *dev_config, i2c_master_dev_handle_t *ret_handle); //

void i2c_write_register(uint8_t reg, uint8_t data, i2c_master_dev_handle_t *ret_handle); //
void i2c_read_register(uint8_t reg, uint8_t *data, i2c_master_dev_handle_t *ret_handle); //
esp_err_t i2c_device_is_connected(uint16_t address);

#endif
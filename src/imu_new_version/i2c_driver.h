#ifndef i2c_driver_h
#define i2c_driver_h

#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "registers_icm20948.h"
#include "ak09916/ak09916.h"

extern gpio_num_t i2c_sda_port;
extern gpio_num_t i2c_scl_port;
extern uint32_t timeout;

extern i2c_master_bus_config_t i2c_mst_config;

extern i2c_master_bus_handle_t i2c_bus_handle;

void initialize(); //

void write_register(uint8_t reg, uint8_t data); //
void read_register(uint8_t reg, uint8_t len, uint8_t *data); //

#endif
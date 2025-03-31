#ifndef ICM20948_h
#define ICM20948_h

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

extern i2c_device_config_t icm20948_dev_cfg;

extern i2c_master_dev_handle_t icm20948_dev_handle;

void initialize(); //

void write_register(uint8_t reg, uint8_t data); //
void read_register(uint8_t reg, uint8_t len, uint8_t *data); //

uint8_t getWhoIAm();
void setBank(uint8_t bank);
uint8_t getBank();

void setPWR_MGMT_1(uint8_t pwr_mgmt_1_value);
void setINT_PIN_CFG(uint8_t cfg);
void setMST_CTRL(uint8_t ctrl);

esp_err_t deviceIsConnected();

void default_init();

float magX(void); // micro teslas
float magY(void); // micro teslas
float magZ(void); // micro teslas

float accX(void); // milli g's
float accY(void); // milli g's
float accZ(void); // milli g's

float gyrX(void); // degrees per second
float gyrY(void); // degrees per second
float gyrZ(void); // degrees per second

float temp(void); // degrees celsius

uint8_t getWhoIAm_Mag();

#endif
#ifndef ak09916_h
#define ak09916_h


#include "registers_ak09916.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2c_driver.h"

uint8_t getWIA();
uint8_t getST_1(); 
uint8_t getHXL();
uint8_t getHXH();
uint8_t getHYL();
uint8_t getHYH();
uint8_t getHZL();
uint8_t getHZH();
uint8_t getST_2();

uint8_t getCNTL_2();
uint8_t getCNTL_3();

void setCNTL_2();
void setCNTL_3();

void reset_ak09916();

void set_dev_handle(i2c_master_dev_handle_t* dev);
void write_register_ak09916(uint8_t reg, uint8_t data);
uint8_t read_register_ak09916(uint8_t reg);

#endif
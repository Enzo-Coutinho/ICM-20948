#ifndef IMU_h
#define IMU_h

#include "ICM20948.h"

void start();
void restart_icm20948();
void default_init();
void enable_DMP();
bool isConnected();
bool isConnected_Mag();
void i2cConfigurePeripheral(uint8_t slvNumber, uint8_t address, uint8_t reg, uint8_t len, bool rw=true, bool enable=true, bool data_only=false, bool grp=false, bool swap=false, uint8_t dataOut=(uint8_t)0U);
#endif
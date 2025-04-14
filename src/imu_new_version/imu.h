#ifndef IMU_h
#define IMU_h

#include "ICM20948.h"

void start();
void restart_icm20948();
void default_init();
void enableDMP(bool enable);
void configureDMP();
void enableFIFO(bool enable);
void resetDMP();
void enableDMPSensor(enum inv_icm20948_sensor sensor, bool enable);
void resetFIFO();
void sleep(bool sleep);
uint8_t sensor_type_2_android_sensor(enum inv_icm20948_sensor sensor);
void lowPowerMode(bool enable);
bool isConnected();
void inv_icm20948_read_dmp_data(icm_20948_DMP_data_t *data);
bool isConnected_Mag();
void i2cConfigurePeripheral(uint8_t slvNumber, uint8_t address, uint8_t reg, uint8_t len, bool rw=true, bool enable=true, bool data_only=false, bool grp=false, bool swap=false, uint8_t dataOut=(uint8_t)0U);
#endif
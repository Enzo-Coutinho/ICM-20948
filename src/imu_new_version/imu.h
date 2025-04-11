#ifndef IMU_h
#define IMU_h

#include "ICM20948.h"

void start();
void restart_icm20948();
void default_init();
void enable_DMP();
bool isConnected();
bool isConnected_Mag();
#endif
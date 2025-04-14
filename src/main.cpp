#include <Arduino.h>

#include "imu_new_version/imu.h"

void setup() {
    Serial.begin(115200);
    start();
    //restart_icm20948();
    while(!isConnected()){
        Serial.printf("Trying...\n");
    }
    default_init();
    enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR, true);
    Serial.printf("ICM20948 Connected: %d\n", getWhoIAm());
    Serial.printf("Mag is connected: %d\n", getWhoIAm_Mag());
}

void loop() {
    icm_20948_DMP_data_t data;
    uint8_t fifoBytes[icm_20948_DMP_Maximum_Bytes]; // Interim storage for the FIFO data

}
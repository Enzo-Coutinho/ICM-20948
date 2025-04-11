#include <Arduino.h>

#include "imu_new_version/imu.h"

void setup() {
    Serial.begin(115200);
    start();
    while(!isConnected()){
        Serial.printf("Trying...\n");
    }
    default_init();
    Serial.printf("ICM20948 Connected: %d\n", isConnected());
    Serial.printf("Mag is connected: %d\n", isConnected_Mag());
}

void loop() {

}
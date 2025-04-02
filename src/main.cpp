#include <Arduino.h>

#include "imu_new_version/ICM20948.h"

void setup() {
    Serial.begin(115200);
    start();
    while(deviceIsConnected() != ESP_OK)
    {
        Serial.printf("Starting %s\n", esp_err_to_name(deviceIsConnected()));
        restart_icm20948();
        delay(500);
    }
    default_init();
}

void loop() {
    Serial.printf("Who i am: %d\n", getWhoIAm());
    Serial.printf("Who i am mag: %d\n", getWhoIAm_Mag());
    //Serial.printf("Status 1: %d\n", getST_1());
    delay(500);
}
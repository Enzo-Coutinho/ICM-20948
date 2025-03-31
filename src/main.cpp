#include <Arduino.h>

#include "imu_new_version/ICM20948.h"

uint8_t addr = 0;

void setup() {
    Serial.begin(115200);
    initialize();
    default_init();
}

void loop() {
    esp_err_t ok = deviceIsConnected();
    read_register(__USER_BANK_3::REGISTERS::__I2C_SLV0_ADDR, 1, &addr);
    Serial.printf("Device is Connected: %d\n", ok);
    Serial.printf("Magnetometer addres: %d\n", addr);
    delay(100);
}
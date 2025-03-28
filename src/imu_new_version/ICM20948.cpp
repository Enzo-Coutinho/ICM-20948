#include "ICM20948.h"

void initialize()
{
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &i2c_bus_handle));

    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus_handle, &icm20948_dev_cfg, &icm20948_dev_handle));
}

esp_err_t write_register(uint8_t reg, uint8_t data)
{
    uint8_t buff[2] = {reg, data};
    return i2c_master_transmit(icm20948_dev_handle, buff, sizeof(buff), timeout);
}

esp_err_t read_register(uint8_t reg, uint8_t len, uint8_t *data)
{
    return i2c_master_transmit_receive(icm20948_dev_handle, &reg, sizeof(reg), data, len, timeout);
}

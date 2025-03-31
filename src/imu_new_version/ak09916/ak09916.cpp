#include "ak09916.h"

i2c_master_dev_handle_t dev_handle;
uint32_t timeout_ak09916 = 1000 / portTICK_PERIOD_MS;

uint8_t getWIA()
{
    uint8_t whoIAm = 0;
    read_register_ak09916(_AK_09916::REGISTERS::__WIA_2, sizeof(whoIAm), &whoIAm);
    return whoIAm;
}

uint8_t getST_1()
{
    uint8_t status_1 = -1;
    read_register_ak09916(_AK_09916::REGISTERS::__ST_1, sizeof(status_1), &status_1);
    return status_1;
}

uint8_t getHXL()
{
    uint8_t HXL = 0;
    read_register_ak09916(_AK_09916::REGISTERS::__HXL, 1, &HXL);
    return HXL;
}

void set_dev_handle(i2c_master_dev_handle_t dev)
{
    dev_handle = dev;
}

void write_register_ak09916(uint8_t reg, uint8_t data)
{
    uint8_t addr_send[2] = {__I2C_SLV4_ADDR, AK_09916_DEVICE_ADDRESS};
    ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, addr_send, sizeof(addr_send), timeout_ak09916));
    uint8_t val_send[2] = {__I2C_SLV4_DO, data};
    ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, val_send, sizeof(val_send), timeout_ak09916));
    uint8_t reg_send[2] = {__I2C_SLV4_REG, reg};
    ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, reg_send, sizeof(reg_send), timeout_ak09916));
    uint8_t ctrl_send[2] = {__I2C_SLV4_CTRL, 0x80};
    ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, ctrl_send, sizeof(ctrl_send), timeout_ak09916));
}

void read_register_ak09916_8(uint8_t reg, uint8_t len, uint8_t *data)
{
    uint8_t reg_send[2] = {AK_09916_DEVICE_ADDRESS, reg};
    ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, reg_send, sizeof(reg_send), data, len, timeout_ak09916));
}
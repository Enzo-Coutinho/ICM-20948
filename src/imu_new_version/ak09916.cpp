#include "ak09916.h"

i2c_master_dev_handle_t * dev_handle;
uint32_t timeout_ak09916 = 1000 / portTICK_PERIOD_MS;

void set_dev_handle(i2c_master_dev_handle_t* dev)
{
    dev_handle = dev;
}

uint8_t getWIA()
{
    uint8_t whoIAm = read_register_ak09916(_AK_09916::REGISTERS::__WIA_2);
    return whoIAm;
}

uint8_t getST_1()
{
    uint8_t status_1 = read_register_ak09916(_AK_09916::REGISTERS::__ST_1);
    return status_1;
}

uint8_t getHXL()
{
    uint8_t HXL = read_register_ak09916(_AK_09916::REGISTERS::__HXL);
    return HXL;
}

void reset_ak09916()
{
    write_register_ak09916(_AK_09916::REGISTERS::__CNTL_3, _AK_09916::CNTL_3::SRST);
}

void write_register_ak09916(uint8_t reg, uint8_t data)
{

    i2c_write_register(__USER_BANK_3::REGISTERS::__I2C_SLV4_ADDR, AK_09916_DEVICE_ADDRESS, dev_handle);
    i2c_write_register(__USER_BANK_3::REGISTERS::__I2C_SLV4_DO, data, dev_handle);
    i2c_write_register(__USER_BANK_3::REGISTERS::__I2C_SLV4_REG, reg, dev_handle);
    i2c_write_register(__USER_BANK_3::REGISTERS::__I2C_SLV4_CTRL, 0x80, dev_handle);
    uint8_t ready = 0;
    while(ready & 0x80)
    {
        i2c_read_register(__USER_BANK_3::REGISTERS::__I2C_SLV4_CTRL, &ready, dev_handle);
    }
}

uint8_t read_register_ak09916(uint8_t reg)
{
    i2c_write_register(__USER_BANK_3::REGISTERS::__I2C_SLV4_ADDR, AK_09916_DEVICE_ADDRESS | 0x80, dev_handle);
    i2c_write_register(__USER_BANK_3::REGISTERS::__I2C_SLV4_REG, reg, dev_handle);
    i2c_write_register(__USER_BANK_3::REGISTERS::__I2C_SLV4_CTRL, 0x80, dev_handle);
    uint8_t ready = 0;
    i2c_read_register(__USER_BANK_3::REGISTERS::__I2C_SLV4_CTRL, &ready, dev_handle);
    uint8_t data = 0;
    i2c_read_register(__USER_BANK_3::REGISTERS::__I2C_SLV4_DI, &data, dev_handle);
    return data;
}
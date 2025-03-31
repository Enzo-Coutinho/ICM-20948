#include "ICM20948.h"

gpio_num_t i2c_sda_port = GPIO_NUM_21;
gpio_num_t i2c_scl_port = GPIO_NUM_22;
uint32_t timeout = 1000 / portTICK_PERIOD_MS;

i2c_master_bus_config_t i2c_mst_config = {
    .i2c_port = -1,
    .sda_io_num = i2c_sda_port,
    .scl_io_num = i2c_scl_port,
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .glitch_ignore_cnt = 7,
};

i2c_master_bus_handle_t i2c_bus_handle;

i2c_device_config_t icm20948_dev_cfg = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = ICM_20948_DEVICE_ADDRESS,
    .scl_speed_hz = 100000,
};

i2c_master_dev_handle_t icm20948_dev_handle;


void initialize()
{
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &i2c_bus_handle));

    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus_handle, &icm20948_dev_cfg, &icm20948_dev_handle));

    set_dev_handle(icm20948_dev_handle);
}

void default_init()
{
    setBank(__USER_BANK_0_ADDR);

    setPWR_MGMT_1(__USER_BANK_0::PWR_MGMT_1::__DEVICE_RESET);
    
    setBank(__USER_BANK_3_ADDR);

    setMST_CTRL(__USER_BANK_3::I2C_MST_CTRL::I2C_MST_CLK_7 | __USER_BANK_3::I2C_MST_CTRL::I2C_MST_P_NSR);

    setBank(__USER_BANK_0_ADDR);

    write_register(__USER_BANK_0::REGISTERS::__USER_CTRL, __USER_BANK_0::USER_CTRL::__I2C_MST_EN);
}

void write_register(uint8_t reg, uint8_t data)
{
    uint8_t buff[2] = {reg, data};
    ESP_ERROR_CHECK(i2c_master_transmit(icm20948_dev_handle, buff, sizeof(buff), timeout));
}

void read_register(uint8_t reg, uint8_t len, uint8_t *data)
{
    ESP_ERROR_CHECK(i2c_master_transmit_receive(icm20948_dev_handle, &reg, sizeof(reg), data, len, timeout));
}

esp_err_t deviceIsConnected()
{
    return i2c_master_probe(i2c_bus_handle, ICM_20948_DEVICE_ADDRESS, timeout);
}

uint8_t getWhoIAm()
{
    uint8_t who_i_am = 0;
    read_register(__USER_BANK_0::REGISTERS::__WHO_AM_I, 1, &who_i_am);
    return who_i_am;
}

void setBank(uint8_t bank)
{
    uint8_t _bank = (bank << 4) & 0x30;
    write_register(__REG_BANK_SEL, _bank);
}

uint8_t getBank()
{
    uint8_t bank_value = 0;
    read_register(__REG_BANK_SEL, 1, &bank_value);
    return bank_value;
}

void setPWR_MGMT_1(uint8_t pwr_mgmt_1_value)
{
    write_register(__USER_BANK_0::REGISTERS::__PWR_MGMT_1, pwr_mgmt_1_value);
}

void setINT_PIN_CFG(uint8_t cfg)
{
    write_register(__USER_BANK_0::REGISTERS::__INT_PIN_CFG, cfg);
}

void setMST_CTRL(uint8_t ctrl)
{
    write_register(__USER_BANK_3::REGISTERS::__I2C_MST_CTRL, ctrl);
}

uint8_t getWhoIAm_Mag()
{
    return getWIA();
}
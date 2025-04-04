#include "ICM20948.h"

i2c_device_config_t icm20948_dev_cfg = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = ICM_20948_DEVICE_ADDRESS,
    .scl_speed_hz = 100000,
};

i2c_master_dev_handle_t icm20948_dev_handle = NULL;

uint8_t lastBank = -1;

void start()
{
    i2c_initialize(&icm20948_dev_cfg, &icm20948_dev_handle);
}

void restart_icm20948()
{
    __USER_BANK_0::pwr_mgmt_1.pwr_mgmt_1_bitmap.__DEVICE_RESET = 1;
    setPWR_MGMT_1(__USER_BANK_0::pwr_mgmt_1.pwr_mgmt_1_bitmap);
}

void default_init()
{
    __USER_BANK_3::i2c_mst_ctrl.i2c_mst_ctrl_bitmap.__I2C_MST_CLK = 7;
    setMST_CTRL(__USER_BANK_3::i2c_mst_ctrl.i2c_mst_ctrl_bitmap);

    __USER_BANK_0::user_ctrl.user_ctrl_bitmap.__I2C_MST_EN = 1;
    setUSER_CTRL(__USER_BANK_0::user_ctrl.user_ctrl_bitmap);
    set_dev_handle(&icm20948_dev_handle);

    reset_ak09916();

    __USER_BANK_2::odr_align_en.odr_align_en_bitmap.__ODR_ALIGN_EN = 1;
    setODR_ALIGN_EN(__USER_BANK_2::odr_align_en.odr_align_en_bitmap);

    uint8_t pwr = getPWR_MGMT_1() & ~0x40;
    __USER_BANK_0::pwr_mgmt_1.pwr_mgmt_1_u8 = pwr;
    setPWR_MGMT_1(__USER_BANK_0::pwr_mgmt_1.pwr_mgmt_1_bitmap);
}

esp_err_t deviceIsConnected()
{
    return i2c_device_is_connected(ICM_20948_DEVICE_ADDRESS);
}

uint8_t getWhoIAm()
{
    setBank(__USER_BANK_0_ADDR);
    uint8_t who_i_am = 0;
    i2c_read_register(__USER_BANK_0::REGISTERS::__WHO_AM_I, &who_i_am, &icm20948_dev_handle);
    return who_i_am;
}

void setBank(uint8_t bank)
{
    if((bank > 3) || lastBank == bank) return;
    uint8_t _bank = (bank << 4) & 0x30;
    i2c_write_register(__REG_BANK_SEL, _bank, &icm20948_dev_handle);
    lastBank = bank;
}

uint8_t getBank()
{
    uint8_t bank_value = 0;
    i2c_read_register(__REG_BANK_SEL, &bank_value, &icm20948_dev_handle);
    return bank_value;
}

void setPWR_MGMT_1(__USER_BANK_0::pwr_mgmt_1_bitmap_t pwr_mgmt_1_bitmap)
{
    setBank(__USER_BANK_0_ADDR);
    __USER_BANK_0::pwr_mgmt_1.pwr_mgmt_1_bitmap = pwr_mgmt_1_bitmap;
    i2c_write_register(__USER_BANK_0::REGISTERS::__PWR_MGMT_1, __USER_BANK_0::pwr_mgmt_1.pwr_mgmt_1_u8, &icm20948_dev_handle);
}

uint8_t getPWR_MGMT_1()
{
    setBank(__USER_BANK_0_ADDR);
    uint8_t pwr_mgmt = 0;
    i2c_read_register(__USER_BANK_0::REGISTERS::__PWR_MGMT_1, &pwr_mgmt, &icm20948_dev_handle);
    return pwr_mgmt;
}

void setINT_PIN_CFG(__USER_BANK_0::int_pin_cgf_bitmap_t int_pin_cfg_bitmap)
{
    setBank(__USER_BANK_0_ADDR);
    __USER_BANK_0::int_pin_cfg.int_pin_cfg_bitmap = int_pin_cfg_bitmap;
    i2c_write_register(__USER_BANK_0::REGISTERS::__INT_PIN_CFG, __USER_BANK_0::int_pin_cfg.int_pin_cfg_u8, &icm20948_dev_handle);
}

void setMST_CTRL(__USER_BANK_3::i2c_mst_ctrl_bitmap_t i2c_mst_ctrl_bitmap)
{
    setBank(__USER_BANK_3_ADDR);
    __USER_BANK_3::i2c_mst_ctrl.i2c_mst_ctrl_bitmap = i2c_mst_ctrl_bitmap;
    i2c_write_register(__USER_BANK_3::REGISTERS::__I2C_MST_CTRL,  __USER_BANK_3::i2c_mst_ctrl.i2c_mst_ctrl_u8, &icm20948_dev_handle);
}

void setODR_ALIGN_EN(__USER_BANK_2::odr_align_en_bitmap_t odr_align_en_bitmap)
{
    setBank(__USER_BANK_2_ADDR);
    __USER_BANK_2::odr_align_en.odr_align_en_bitmap = odr_align_en_bitmap;
    i2c_write_register(__USER_BANK_2::REGISTERS::__ODR_ALIGN_EN,  __USER_BANK_2::odr_align_en.odr_align_en_u8, &icm20948_dev_handle);
}

uint8_t getWhoIAm_Mag()
{
    setBank(__USER_BANK_3_ADDR);
    return getWIA();
}
#include "imu.h"

__USER_BANK_0::user_ctrl_t user_ctrl = {.user_ctrl_u8 = 0x00};
__USER_BANK_0::lp_config_t lp_config = {.lp_config_u8 = 0x40};
__USER_BANK_0::pwr_mgmt_1_t pwr_mgmt_1 = {.pwr_mgmt_1_u8 = 0x41};
__USER_BANK_0::pwr_mgmt_2_t pwr_mgmt_2 = {.pwr_mgmt_2_u8 = 0x40};
__USER_BANK_0::int_pin_cfg_t int_pin_cfg = {.int_pin_cfg_u8 = 0x00};
__USER_BANK_0::int_enable_t int_enable = {.int_enable_u8 = 0x00};
__USER_BANK_0::int_enable_1_t int_enable_1 = {.int_enable_1_u8 = 0x00};

__USER_BANK_2::odr_align_en_t odr_align_en = {.odr_align_en_u8 = 0x00};

__USER_BANK_3::i2c_mst_ctrl_t i2c_mst_ctrl = {.i2c_mst_ctrl_u8 = 0x00};

void start()
{
    i2c_initialize(&icm20948_dev_cfg, &icm20948_dev_handle);
}


void restart_icm20948()
{
    pwr_mgmt_1.pwr_mgmt_1_bitmap.__DEVICE_RESET = 1;
    setPWR_MGMT_1(pwr_mgmt_1);
}

bool isConnected()
{
    return getWhoIAm() == DEFAULT_VALUE_WIA_ICM;
}

bool isConnected_Mag()
{
    return getWhoIAm_Mag() == DEFAULT_VALUE_WIA_MAG;
}

void sleep(bool sleep)
{
    __USER_BANK_0::pwr_mgmt_1_t pwr = getPWR_MGMT_1();
    pwr.pwr_mgmt_1_bitmap.__SLEEP = (uint8_t)sleep;
    setPWR_MGMT_1(pwr);
}


void default_init()
{
    i2c_mst_ctrl.i2c_mst_ctrl_bitmap.__I2C_MST_CLK = 7;
    setMST_CTRL(i2c_mst_ctrl);

    user_ctrl.user_ctrl_bitmap.__I2C_MST_EN = 1;
    setUSER_CTRL(user_ctrl);

    set_dev_handle(&icm20948_dev_handle);

    reset_Mag();

    odr_align_en.odr_align_en_bitmap.__ODR_ALIGN_EN = 1;
    setODR_ALIGN_EN(odr_align_en);

    sleep(false);
}
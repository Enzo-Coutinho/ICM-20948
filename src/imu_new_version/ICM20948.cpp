#include "ICM20948.h"

i2c_device_config_t icm20948_dev_cfg = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = ICM_20948_DEVICE_ADDRESS,
    .scl_speed_hz = 100000,
};

i2c_master_dev_handle_t icm20948_dev_handle = NULL;

uint8_t lastBank = -1;


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
    if((bank > 3) || (lastBank == bank)) return;
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

// ==================================================================
//                          USER BANK 0
// ==================================================================

void setUSER_CTRL(__USER_BANK_0::user_ctrl_t user_ctrl)
{
    setBank(__USER_BANK_0_ADDR);
    i2c_write_register(__USER_BANK_0::REGISTERS::__USER_CTRL, user_ctrl.user_ctrl_u8, &icm20948_dev_handle);
}

void setLP_CONFIG(__USER_BANK_0::lp_config_t lp_config)
{
    setBank(__USER_BANK_0_ADDR);
    i2c_write_register(__USER_BANK_0::REGISTERS::__LP_CONFIG_, lp_config.lp_config_u8, &icm20948_dev_handle);
}

void setPWR_MGMT_1(__USER_BANK_0::pwr_mgmt_1_t pwr_mgmt_1)
{
    setBank(__USER_BANK_0_ADDR);
    i2c_write_register(__USER_BANK_0::REGISTERS::__PWR_MGMT_1, pwr_mgmt_1.pwr_mgmt_1_u8, &icm20948_dev_handle);
}

__USER_BANK_0::pwr_mgmt_1_t getPWR_MGMT_1()
{
    setBank(__USER_BANK_0_ADDR);
    __USER_BANK_0::pwr_mgmt_1_t pwr_mgmt = {.pwr_mgmt_1_u8 = 0};
    i2c_read_register(__USER_BANK_0::REGISTERS::__PWR_MGMT_1, &pwr_mgmt.pwr_mgmt_1_u8, &icm20948_dev_handle);
    return pwr_mgmt;
}

void setPWR_MGMT_2(__USER_BANK_0::pwr_mgmt_2_t pwr_mgmt_2)
{
    setBank(__USER_BANK_0_ADDR);
    i2c_write_register(__USER_BANK_0::REGISTERS::__PWR_MGMT_2, pwr_mgmt_2.pwr_mgmt_2_u8, &icm20948_dev_handle);
}

void setINT_PIN_CFG(__USER_BANK_0::int_pin_cfg_t int_pin_cfg)
{
    setBank(__USER_BANK_0_ADDR);
    i2c_write_register(__USER_BANK_0::REGISTERS::__INT_PIN_CFG, int_pin_cfg.int_pin_cfg_u8, &icm20948_dev_handle);
}

void setFIFO_EN_1(__USER_BANK_0::fifo_en_1_t fifo_en_1)
{
    setBank(__USER_BANK_0_ADDR);
    i2c_write_register(__USER_BANK_0::REGISTERS::__FIFO_EN_1, fifo_en_1.fifo_en_1_u8, &icm20948_dev_handle);
}

void setFIFO_EN_2(__USER_BANK_0::fifo_en_2_t fifo_en_2)
{
    setBank(__USER_BANK_0_ADDR);
    i2c_write_register(__USER_BANK_0::REGISTERS::__FIFO_EN_1, fifo_en_2.fifo_en_2_u8, &icm20948_dev_handle);
}

void setFIFO_RST(__USER_BANK_0::fifo_rst_t fifo_rst)
{
    setBank(__USER_BANK_0_ADDR);
    i2c_write_register(__USER_BANK_0::REGISTERS::__FIFO_RST, fifo_rst.fifo_rst_u8, &icm20948_dev_handle);
}

void setINT_ENABLE_1(__USER_BANK_0::int_enable_1_t int_enable_1)
{
    setBank(__USER_BANK_0_ADDR);
    i2c_write_register(__USER_BANK_0::REGISTERS::__INT_ENABLE_1, int_enable_1.int_enable_1_u8, &icm20948_dev_handle);
}

void setHARDWARE_FIX(uint8_t fix)
{
    setBank(__USER_BANK_0_ADDR);
    i2c_write_register(117, fix, &icm20948_dev_handle);
}

void setFIFO_PRIORITY_SEL(uint8_t fifoPrior)
{
    setBank(__USER_BANK_0_ADDR);
    i2c_write_register(38, fifoPrior, &icm20948_dev_handle);
}

__USER_BANK_1::timebase_correction_pll_t getTIMEBASE_CORRECTION_PLL()
{
    setBank(__USER_BANK_1_ADDR);
    __USER_BANK_1::timebase_correction_pll_t pll = {.time_base_correction_pll_u8=0x00};
    i2c_read_register(__USER_BANK_1::REGISTERS::__TIMEBASE_CORRECTION_PLL, &pll.time_base_correction_pll_u8, &icm20948_dev_handle);
    return pll;
}

void setGYRO_SMPLRT_DIV(__USER_BANK_2::gyro_smplrt_div_t gyro_smplrt_div)
{
    setBank(__USER_BANK_2_ADDR);
    i2c_write_register(__USER_BANK_2::REGISTERS::__GYRO_SMPLRT_DIV, gyro_smplrt_div.gyro_smplrt_div_u8, &icm20948_dev_handle);
}

void setODR_ALIGN_EN(__USER_BANK_2::odr_align_en_t odr_align_en)
{
    setBank(__USER_BANK_2_ADDR);
    i2c_write_register(__USER_BANK_2::REGISTERS::__ODR_ALIGN_EN,  odr_align_en.odr_align_en_u8, &icm20948_dev_handle);
}

void setGYRO_CONFIG_1(__USER_BANK_2::gyro_config_1_t gyro_config_1)
{
    setBank(__USER_BANK_2_ADDR);
    i2c_write_register(__USER_BANK_2::REGISTERS::__GYRO_CONFIG_1, gyro_config_1.gyro_config_1_u8, &icm20948_dev_handle);
}

void setACCEL_CONFIG(__USER_BANK_2::accel_config_t accel_config)
{
    setBank(__USER_BANK_2_ADDR);
    i2c_write_register(__USER_BANK_2::REGISTERS::__ACCEL_CONFIG, accel_config.accel_config_u8, &icm20948_dev_handle);
}

void setACCEL_SMPLRT_DIV_2(__USER_BANK_2::accel_smplrt_div_2_t accel_smplrt_div_2)
{
    setBank(__USER_BANK_2_ADDR);
    i2c_write_register(__USER_BANK_2::REGISTERS::__ACCEL_SMPLRT_DIV_2, accel_smplrt_div_2.accel_smplrt_div_2_u8, &icm20948_dev_handle);
}

void setDMPStartAddress(uint16_t address)
{
    setBank(__USER_BANK_2_ADDR);
    uint8_t start_address[2] = {(uint8_t)(address >> 8), (uint8_t)(address & 0xff)};
    i2c_write_register_with_diff_buff(0x50, start_address, &icm20948_dev_handle);
}



// ==================================================================
//                          USER BANK 3
// ==================================================================

void setMST_ODR_CONFIG(__USER_BANK_3::i2c_mst_odr_config_t i2c_mst_odr)
{
    setBank(__USER_BANK_3_ADDR);
    i2c_write_register(__USER_BANK_3::REGISTERS::__I2C_MST_ODR_CONFIG, i2c_mst_odr.i2c_mst_odr_config_u8, &icm20948_dev_handle);
}

void setMST_CTRL(__USER_BANK_3::i2c_mst_ctrl_t i2c_mst_ctrl)
{
    setBank(__USER_BANK_3_ADDR);
    i2c_write_register(__USER_BANK_3::REGISTERS::__I2C_MST_CTRL,  i2c_mst_ctrl.i2c_mst_ctrl_u8, &icm20948_dev_handle);
}

void setI2C_SLVX_ADDR(uint8_t slvNumber, __USER_BANK_3::i2c_slvx_addr_t slvx_addr_t)
{
    setBank(__USER_BANK_3_ADDR);
    i2c_write_register(slvNumber, slvx_addr_t.i2c_slvx_addr_u8, &icm20948_dev_handle);
}

__USER_BANK_3::i2c_slvx_addr_t getI2C_SLVX_ADDR(uint8_t slvNumber)
{
    setBank(__USER_BANK_3_ADDR);
    __USER_BANK_3::i2c_slvx_addr_t slvx_addr = {.i2c_slvx_addr_u8 = 0};
    i2c_read_register(slvNumber, &slvx_addr.i2c_slvx_addr_u8, &icm20948_dev_handle);
    return slvx_addr;
}

void setI2C_SLVX_REG(uint8_t slvNumber, uint8_t I2C_SLVX_REG)
{
    setBank(__USER_BANK_3_ADDR);
    i2c_write_register(slvNumber, I2C_SLVX_REG, &icm20948_dev_handle);
}

uint8_t getI2C_SLVX_REG(uint8_t slvNumber)
{
    setBank(__USER_BANK_3_ADDR);
    uint8_t data = 0;
    i2c_read_register(slvNumber, &data, &icm20948_dev_handle);
    return data;
}

void setI2C_SLVX_CTRL(uint8_t slvNumber, __USER_BANK_3::i2c_slvx_ctrl_t i2c_slvx_ctrl)
{
    setBank(__USER_BANK_3_ADDR);
    i2c_write_register(slvNumber, i2c_slvx_ctrl.i2c_slvx_ctrl_u8, &icm20948_dev_handle);
}

__USER_BANK_3::i2c_slvx_ctrl_t getI2C_SLVX_CTRL(uint8_t slvNumber)
{
    setBank(__USER_BANK_3_ADDR);
    __USER_BANK_3::i2c_slvx_ctrl_t i2c_slvx_ctrl = {.i2c_slvx_ctrl_u8 = 0};
    i2c_read_register(slvNumber,  &i2c_slvx_ctrl.i2c_slvx_ctrl_u8, &icm20948_dev_handle);
    return i2c_slvx_ctrl;
}

void setI2C_SLVX_DO(uint8_t slvNumber, uint8_t data)
{
    setBank(__USER_BANK_3_ADDR);
    i2c_write_register(slvNumber, data, &icm20948_dev_handle);
}

uint8_t getI2C_SLVX_DO(uint8_t slvNumber)
{
    setBank(__USER_BANK_3_ADDR);
    uint8_t data = 0;
    i2c_read_register(slvNumber, &data, &icm20948_dev_handle);
    return data;
}

void setDMPmems(uint16_t reg, uint32_t lenght, const uint8_t * data)
{
    setBank(__USER_BANK_0_ADDR);

    uint8_t lBankSelected = (reg >> 8);
    uint32_t bytesWritten = 0;
    uint32_t thisLen;
    uint8_t lStartAddrSelected;
    i2c_write_register(126, lBankSelected, &icm20948_dev_handle);

    while(bytesWritten < lenght)
    {
        lStartAddrSelected = (reg & 0xff);

        i2c_write_register(124, lStartAddrSelected, &icm20948_dev_handle);

        if(lenght - bytesWritten <= 16)
            thisLen = lenght - bytesWritten;
        else
            thisLen = 16;
        i2c_write_register_with_specific_size(125, data[bytesWritten], &icm20948_dev_handle, thisLen);

        bytesWritten += thisLen;
        reg += thisLen;
    }
}

uint8_t getWhoIAm_Mag()
{
    setBank(__USER_BANK_3_ADDR);
    return getWIA();
}

void reset_Mag()
{
    reset_ak09916();
}
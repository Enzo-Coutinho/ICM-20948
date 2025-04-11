#include "imu.h"

__USER_BANK_0::user_ctrl_t user_ctrl = {.user_ctrl_u8 = 0x00};
__USER_BANK_0::lp_config_t lp_config = {.lp_config_u8 = 0x40};
__USER_BANK_0::pwr_mgmt_1_t pwr_mgmt_1 = {.pwr_mgmt_1_u8 = 0x41};
__USER_BANK_0::pwr_mgmt_2_t pwr_mgmt_2 = {.pwr_mgmt_2_u8 = 0x40};
__USER_BANK_0::int_pin_cfg_t int_pin_cfg = {.int_pin_cfg_u8 = 0x00};
__USER_BANK_0::int_enable_t int_enable = {.int_enable_u8 = 0x00};
__USER_BANK_0::int_enable_1_t int_enable_1 = {.int_enable_1_u8 = 0x00};
__USER_BANK_0::fifo_en_1_t fifo_en_1 = {.fifo_en_1_u8 = 0x00};
__USER_BANK_0::fifo_en_2_t fifo_en_2 = {.fifo_en_2_u8 = 0x00};
__USER_BANK_0::fifo_rst_t fifo_rst = {.fifo_rst_u8 = 0x00};


__USER_BANK_2::odr_align_en_t odr_align_en = {.odr_align_en_u8 = 0x00};
__USER_BANK_2::gyro_config_1_t gyro_config_1 = {.gyro_config_1_u8 = 0x01};
__USER_BANK_2::accel_config_t accel_config = {.accel_config_u8 = 0x01};
__USER_BANK_2::gyro_smplrt_div_t gyro_smplrt_div = {.gyro_smplrt_div_u8 = 0x00};
__USER_BANK_2::accel_smplrt_div_2_t accel_smplrt_div_2 = {.accel_smplrt_div_2_u8 = 0x00};


__USER_BANK_3::i2c_mst_ctrl_t i2c_mst_ctrl = {.i2c_mst_ctrl_u8 = 0x00};
__USER_BANK_3::i2c_mst_odr_config_t i2c_mst_odr_config = {.i2c_mst_odr_config_u8 = 0x00};



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

    #ifdef DMP_ENABLED
        enable_DMP();
    #endif
}

void enable_DMP()
{
    i2cConfigurePeripheral(__USER_BANK_3::REGISTERS::__I2C_SLV0_ADDR, AK_09916_DEVICE_ADDRESS, _AK_09916::REGISTERS::__RSV_2, 10, true, true, false, true, true, 0);

    i2cConfigurePeripheral(__USER_BANK_3::REGISTERS::__I2C_SLV0_ADDR, AK_09916_DEVICE_ADDRESS, _AK_09916::REGISTERS::__CNTL_2, 1, true, true, false, true, true, 1);

    i2c_mst_odr_config.i2c_mst_odr_config_bitmap.__I2C_MST_ODR_CONFIG = 4;
    setMST_ODR_CONFIG(i2c_mst_odr_config);

    pwr_mgmt_1.pwr_mgmt_1_bitmap.__CLKSEL = 1;
    setPWR_MGMT_1(pwr_mgmt_1);

    pwr_mgmt_2.pwr_mgmt_2_u8 = 0x40; // pressure sensor?
    setPWR_MGMT_2(pwr_mgmt_2);

    lp_config.lp_config_bitmap.__I2C_MST_CYCLE = 1;
    setLP_CONFIG(lp_config);

    user_ctrl.user_ctrl_bitmap.__FIFO_EN = 0;
    user_ctrl.user_ctrl_bitmap.__DMP_EN = 1;
    setUSER_CTRL(user_ctrl);

    gyro_config_1.gyro_config_1_bitmap.__GYRO_FS_SEL = 3;
    gyro_config_1.gyro_config_1_bitmap.__GYRO_DLPFCFG = 1;

    setGYRO_CONFIG_1(gyro_config_1);

    accel_config.accel_config_bitmap.__ACCEL_FS_SEL = 1;

    setACCEL_CONFIG(accel_config);

    fifo_en_1.fifo_en_1_u8 = 0x00;
    setFIFO_EN_1(fifo_en_1);

    fifo_en_2.fifo_en_2_u8 = 0x00;
    setFIFO_EN_2(fifo_en_2); 
    
    int_enable_1.int_enable_1_bitmap.__RAW_DATA_0_RDY_EN = 0;
    setINT_ENABLE_1(int_enable_1);
    
    fifo_rst.fifo_rst_bitmap.__FIFO_RESET = 1;
    setFIFO_RST(fifo_rst);

    gyro_smplrt_div.gyro_smplrt_div_bitmap.__GYRO_SMPLRT_DIV = 19;
    setGYRO_SMPLRT_DIV(gyro_smplrt_div);

    accel_smplrt_div_2.accel_smplrt_div_2_bitmap.__ACCEL_SMPLRT_DIV = 19;
    setACCEL_SMPLRT_DIV_2(accel_smplrt_div_2);

    setDMPStartAddress();

    setHARDWARE_FIX(0x48);

    setFIFO_PRIORITY_SEL(0xE4);

    const uint8_t accScale[4] = {0x04, 0x00, 0x00, 0x00};
    setDMPmems(ACC_SCALE, 4, &accScale[0]);

    const uint8_t accScale2[4] = {0x00, 0x04, 0x00, 0x00};
    setDMPmems(ACC_SCALE, 4, &accScale2[0]);

    const uint8_t mountMultiplierZero[4] = {0x00, 0x00, 0x00, 0x00};
    const uint8_t mountMultiplierPlus[4] = {0x09, 0x99, 0x99, 0x99}; 
    const uint8_t mountMultiplierMinus[4] = {0xF6, 0x66, 0x66, 0x67};

    setDMPmems(CPASS_MTX_00, 4, &mountMultiplierPlus[0]); 
    setDMPmems(CPASS_MTX_01, 4, &mountMultiplierZero[0]); 
    setDMPmems(CPASS_MTX_02, 4, &mountMultiplierZero[0]); 
    setDMPmems(CPASS_MTX_10, 4, &mountMultiplierZero[0]); 
    setDMPmems(CPASS_MTX_11, 4, &mountMultiplierMinus[0]); 
    setDMPmems(CPASS_MTX_12, 4, &mountMultiplierZero[0]); 
    setDMPmems(CPASS_MTX_20, 4, &mountMultiplierZero[0]); 
    setDMPmems(CPASS_MTX_21, 4, &mountMultiplierZero[0]);
    setDMPmems(CPASS_MTX_22, 4, &mountMultiplierMinus[0]); 

    const unsigned char b2sMountMultiplierZero[4] = {0x00, 0x00, 0x00, 0x00};
    const unsigned char b2sMountMultiplierPlus[4] = {0x40, 0x00, 0x00, 0x00};

    setDMPmems(B2S_MTX_00, 4, &b2sMountMultiplierPlus[0]); 
    setDMPmems(B2S_MTX_01, 4, &b2sMountMultiplierZero[0]); 
    setDMPmems(B2S_MTX_02, 4, &b2sMountMultiplierZero[0]);
    setDMPmems(B2S_MTX_10, 4, &b2sMountMultiplierZero[0]); 
    setDMPmems(B2S_MTX_11, 4, &b2sMountMultiplierPlus[0]);
    setDMPmems(B2S_MTX_12, 4, &b2sMountMultiplierZero[0]); 
    setDMPmems(B2S_MTX_20, 4, &b2sMountMultiplierZero[0]); 
    setDMPmems(B2S_MTX_21, 4, &b2sMountMultiplierZero[0]); 
    setDMPmems(B2S_MTX_22, 4, &b2sMountMultiplierPlus[0]); 
}

void i2cConfigurePeripheral(uint8_t slvNumber, uint8_t address, uint8_t reg, uint8_t len, bool rw, bool enable, bool data_only, bool grp, bool swap, uint8_t dataOut)
{
    __USER_BANK_3::i2c_slvx_addr_t slvx_addr = {.i2c_slvx_addr_bitmap = {.__I2C_ID_X = address, .__I2C_SLVX_RNW=rw}};

    setI2C_SLVX_ADDR(slvNumber, slvx_addr);

    if(!rw)
        setI2C_SLVX_DO(slvNumber, dataOut);

    setI2C_SLVX_REG(slvNumber, reg);

    __USER_BANK_3::i2c_slvx_ctrl_t slvx_ctrl = {.i2c_slvx_ctrl_bitmap = {.__I2C_SLVX_LENG = len, .__I2C_SLVX_GRP=grp, .__I2C_SLVX_REG_DIS=data_only, .__I2C_SLVX_BYTE_SW=swap, .__I2C_SLVX_EN=enable}};

    setI2C_SLVX_CTRL(slvNumber, slvx_ctrl);
}
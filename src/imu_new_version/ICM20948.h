#ifndef ICM20948_h
#define ICM20948_h

#include "Arduino.h"
#include "i2c_driver.h"
#include "ak09916.h"
#include "ICM_20948_DMP.h"

extern i2c_device_config_t icm20948_dev_cfg;

extern i2c_master_dev_handle_t icm20948_dev_handle;

#define DMP_ENABLED

// ==================================================================
//                          GENERAL
// ==================================================================

void setBank(uint8_t bank);
uint8_t getBank();
esp_err_t deviceIsConnected();

// ==================================================================
//                          USER BANK 0
// ==================================================================

uint8_t getWhoIAm();

void setUSER_CTRL(__USER_BANK_0::user_ctrl_t user_ctrl);
__USER_BANK_0::user_ctrl_t getUSER_CTRL();

void setLP_CONFIG(__USER_BANK_0::lp_config_t lp_config);
__USER_BANK_0::lp_config_t getLP_CONFIG();

void setPWR_MGMT_1(__USER_BANK_0::pwr_mgmt_1_t pwr_mgmt_1);
__USER_BANK_0::pwr_mgmt_1_t getPWR_MGMT_1();

void setPWR_MGMT_2(__USER_BANK_0::pwr_mgmt_2_t pwr_mgmt_2);
__USER_BANK_0::pwr_mgmt_2_t getPWR_MGMT_2();

void setINT_PIN_CFG(__USER_BANK_0::int_pin_cfg_t int_pin_cfg);
__USER_BANK_0::int_pin_cfg_t getINT_PINCFG();

void setINT_ENABLE(__USER_BANK_0::int_enable_t int_enable);
__USER_BANK_0::int_enable_t getINT_ENABLE();

void setINT_ENABLE_1(__USER_BANK_0::int_enable_1_t  int_enable_1);
__USER_BANK_0::int_enable_1_t getINT_ENABLE_1();

void setINT_ENABLE_2(__USER_BANK_0::int_enable_2_t int_enable_2);
__USER_BANK_0::int_enable_2_t getINT_ENABLE_2();

void setINT_ENABLE_3(__USER_BANK_0::int_enable_3_t int_enable_3);
__USER_BANK_0::int_enable_3_t getINT_ENABLE_3();

__USER_BANK_0::i2c_mst_status_t getI2C_MST_STATUS();

__USER_BANK_0::int_status_t getINT_STATUS();
__USER_BANK_0::int_status_1_t getINT_STATUS_1();
__USER_BANK_0::int_status_2_t getINT_STATUS_2();
__USER_BANK_0::int_status_3_t getINT_STATUS_3();

__USER_BANK_0::delay_timeh_t getDELAY_TIMEH();
__USER_BANK_0::delay_timel_t getDELAY_TIMEL();

__USER_BANK_0::accel_xout_h_t getACCEL_XOUT_H();
__USER_BANK_0::accel_xout_l_t getACCEL_XOUT_L();
__USER_BANK_0::accel_yout_h_t getACCEL_YOUT_H();
__USER_BANK_0::accel_yout_l_t getACCEL_YOUT_L();
__USER_BANK_0::accel_zout_h_t getACCEL_ZOUT_H();
__USER_BANK_0::accel_zout_l_t getACCEL_ZOUT_L();

__USER_BANK_0::gyro_xout_h_t getGYRO_XOUT_H();
__USER_BANK_0::gyro_xout_l_t getGYRO_XOUT_L();
__USER_BANK_0::gyro_yout_h_t getGYRO_YOUT_H();
__USER_BANK_0::gyro_yout_l_t getGYRO_YOUT_L();
__USER_BANK_0::gyro_zout_h_t getGYRO_ZOUT_H();
__USER_BANK_0::gyro_zout_l_t getGYRO_ZOUT_L();

__USER_BANK_0::temp_out_h_t getTEMP_OUT_H();
__USER_BANK_0::temp_out_l_t getTEMP_OUT_L();

__USER_BANK_0::ext_slv_sens_data_xx_t getEXT_SLV_SENS_DATA_XX(uint8_t slvNumber);

void setFIFO_EN_1(__USER_BANK_0::fifo_en_1_t fifo_en_1);
__USER_BANK_0::fifo_en_1_t getFIFO_EN_1();

void setFIFO_EN_2(__USER_BANK_0::fifo_en_2_t fifo_en_2);
__USER_BANK_0::fifo_en_2_t getFIFO_EN_2();

void setFIFO_RST(__USER_BANK_0::fifo_rst_t fifo_rst);
__USER_BANK_0::fifo_rst_t getFIFO_RST();

void setFIFO_MODE();
__USER_BANK_0::fifo_mode_t getFIFO_MODE();

__USER_BANK_0::fifo_count_h_t getFIFO_COUNTH();
__USER_BANK_0::fifo_count_l_t getFIFO_COUNTL();

void setFIFO_R_W();
__USER_BANK_0::fifo_r_w_t getFIFO_R_W();

__USER_BANK_0::data_rdy_status_t getDATA_RDY_STATUS();
__USER_BANK_0::fifo_cfg_t getFIFO_CFG();

// ==================================================================
//                          USER BANK 1
// ==================================================================

void setSELF_TEST_X_GYRO(uint8_t self_test_x_gyro);
uint8_t getSELF_TEST_X_GYRO();
void setSELF_TEST_Y_GYRO(uint8_t self_test_y_gyro);
uint8_t getSELF_TEST_Y_GYRO();
void setSELF_TEST_Z_GYRO(uint8_t self_test_z_gyro);
uint8_t getSELF_TEST_Z_GYRO();

void setSELF_TEST_X_ACCEL(uint8_t self_test_x_accel);
uint8_t getSELF_TEST_X_ACCEL();
void setSELF_TEST_Y_ACCEL(uint8_t self_test_y_accel);
uint8_t getSELF_TEST_Y_ACCEL();
void setSELF_TEST_Z_ACCEL(uint8_t self_test_z_accel);
uint8_t getSELF_TEST_Z_ACCEL();


// ==================================================================
//                          USER BANK 2
// ==================================================================

void setGYRO_SMPLRT_DIV(__USER_BANK_2::gyro_smplrt_div_t gyro_smplrt_div);
void setODR_ALIGN_EN(__USER_BANK_2::odr_align_en_t odr_align_en);
void setGYRO_CONFIG_1(__USER_BANK_2::gyro_config_1_t gyro_config_1);
void setACCEL_CONFIG(__USER_BANK_2::accel_config_t accel_config);
void setACCEL_SMPLRT_DIV_2(__USER_BANK_2::accel_smplrt_div_2_t accel_smplrt_div_2);

void setDMPStartAddress(uint16_t address=(uint16_t)4096U);

// ==================================================================
//                          USER BANK 3
// ==================================================================

void setMST_ODR_CONFIG(__USER_BANK_3::i2c_mst_odr_config_t i2c_mst_odr);

void setMST_CTRL(__USER_BANK_3::i2c_mst_ctrl_t i2c_mst_ctrl);

void setI2C_SLVX_ADDR(uint8_t slvNumber, __USER_BANK_3::i2c_slvx_addr_t);
__USER_BANK_3::i2c_slvx_addr_t getI2C_SLVX_ADDR(uint8_t slvNumber);

void setI2C_SLVX_REG(uint8_t slvNumber, uint8_t I2C_SLVX_REG);
uint8_t getI2C_SLVX_REG(uint8_t slvNumber);

void setI2C_SLVX_CTRL(uint8_t slvNumber, __USER_BANK_3::i2c_slvx_ctrl_t);
__USER_BANK_3::i2c_slvx_ctrl_t getI2C_SLVX_CTRL(uint8_t slvNumber);

void setI2C_SLVX_DO(uint8_t slvNumber, uint8_t data);
uint8_t getI2C_SLVX_DO(uint8_t slvNumber);

// ==================================================================
//                          MAG (sub of bank 3)
// ==================================================================

uint8_t getWhoIAm_Mag();
void reset_Mag();

// ==================================================================
//                          DMP (All banks)
// ==================================================================

void setHARDWARE_FIX(uint8_t fix);
void setFIFO_PRIORITY_SEL(uint8_t fifoPrio);
void setDMPmems(uint16_t reg, uint32_t len, const uint8_t * data);

#endif
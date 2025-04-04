#ifndef ICM20948_h
#define ICM20948_h

#include "Arduino.h"
#include "i2c_driver.h"
#include "ak09916.h"

extern i2c_device_config_t icm20948_dev_cfg;

extern i2c_master_dev_handle_t icm20948_dev_handle;

// ==================================================================
//                          GENERAL
// ==================================================================

void start();
void restart_icm20948();

void setBank(uint8_t bank);
uint8_t getBank();

// ==================================================================
//                          USER BANK 0
// ==================================================================

uint8_t getWhoIAm();

void setUSER_CTRL(__USER_BANK_0::user_ctrl_bitmap_t user_ctrl_bitmap);
uint8_t getUSER_CTRL();

void setLP_CONFIG(__USER_BANK_0::lp_config_bitmap_t lp_config_bitmap);
uint8_t getLP_CONFIG();

void setPWR_MGMT_1(__USER_BANK_0::pwr_mgmt_1_bitmap_t pwr_mgmt_1_bitmap);
uint8_t getPWR_MGMT_1();

void setPWR_MGMT_2(__USER_BANK_0::pwr_mgmt_2_bitmap_t pwr_mgmt_2_bitmap);
uint8_t getPWR_MGMT_2();

void setINT_PIN_CFG(__USER_BANK_0::int_pin_cgf_bitmap_t int_pin_cfg_bitmap);
uint8_t getINT_PINCFG();

void setINT_ENABLE(__USER_BANK_0::int_enable_bitmap_t int_enable_bitmap);
uint8_t getINT_ENABLE();

void setINT_ENABLE_1(__USER_BANK_0::int_enable_1_bitmap_t  int_enable_1_bitmap);
uint8_t getINT_ENABLE_1();

void setINT_ENABLE_2(__USER_BANK_0::int_enable_2_bitmap_t int_enable_2_bitmap);
uint8_t getINT_ENABLE_2();

void setINT_ENABLE_3(__USER_BANK_0::int_enable_3_bitmap_t int_enable_3_bitmap);
uint8_t getINT_ENABLE_3();

uint8_t getI2C_MST_STATUS();

uint8_t getINT_STATUS();
uint8_t getINT_STATUS_1();
uint8_t getINT_STATUS_2();
uint8_t getINT_STATUS_3();

uint8_t getDELAY_TIMEH();
uint8_t getDELAY_TIMEL();

uint8_t getACCEL_XOUT_H();
uint8_t getACCEL_XOUT_L();
uint8_t getACCEL_YOUT_H();
uint8_t getACCEL_YOUT_L();
uint8_t getACCEL_ZOUT_H();
uint8_t getACCEL_ZOUT_L();

uint8_t getGYRO_XOUT_H();
uint8_t getGYRO_XOUT_L();
uint8_t getGYRO_YOUT_H();
uint8_t getGYRO_YOUT_L();
uint8_t getGYRO_ZOUT_H();
uint8_t getGYRO_ZOUT_L();

uint8_t getTEMP_OUT_H();
uint8_t getTEMP_OUT_L();

uint8_t getEXT_SLV_SENS_DATA_00();
uint8_t getEXT_SLV_SENS_DATA_01();
uint8_t getEXT_SLV_SENS_DATA_02();
uint8_t getEXT_SLV_SENS_DATA_03();
uint8_t getEXT_SLV_SENS_DATA_04();
uint8_t getEXT_SLV_SENS_DATA_05();
uint8_t getEXT_SLV_SENS_DATA_06();
uint8_t getEXT_SLV_SENS_DATA_07();
uint8_t getEXT_SLV_SENS_DATA_08();
uint8_t getEXT_SLV_SENS_DATA_09();
uint8_t getEXT_SLV_SENS_DATA_10();
uint8_t getEXT_SLV_SENS_DATA_11();
uint8_t getEXT_SLV_SENS_DATA_12();
uint8_t getEXT_SLV_SENS_DATA_13();
uint8_t getEXT_SLV_SENS_DATA_14();
uint8_t getEXT_SLV_SENS_DATA_15();
uint8_t getEXT_SLV_SENS_DATA_16();
uint8_t getEXT_SLV_SENS_DATA_17();
uint8_t getEXT_SLV_SENS_DATA_18();
uint8_t getEXT_SLV_SENS_DATA_19();
uint8_t getEXT_SLV_SENS_DATA_20();
uint8_t getEXT_SLV_SENS_DATA_21();
uint8_t getEXT_SLV_SENS_DATA_22();
uint8_t getEXT_SLV_SENS_DATA_23();

void setFIFO_EN_1(uint8_t fifo_en_1);
uint8_t getFIFO_EN_1();

void setFIFO_EN_2(uint8_t fifo_en_2);
uint8_t getFIFO_EN_2();

void setFIFO_RST(uint8_t fifo_rst);
uint8_t getFIFO_RST();

void setFIFO_MODE();
uint8_t getFIFO_MODE();

uint8_t getFIFO_COUNTH();
uint8_t getFIFO_COUNTL();

void setFIFO_R_W();
uint8_t getFIFO_R_W();

uint8_t DATA_RDY_STATUS();
uint8_t FIFO_CFG();

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

void setODR_ALIGN_EN(__USER_BANK_2::odr_align_en_bitmap_t odr_align_en_bitmap);


// ==================================================================
//                          USER BANK 3
// ==================================================================

void setMST_CTRL(__USER_BANK_3::i2c_mst_ctrl_bitmap_t i2c_mst_ctrl_bitmap);

esp_err_t deviceIsConnected();

void default_init();

uint8_t getWhoIAm_Mag();

#endif
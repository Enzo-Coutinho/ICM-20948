#pragma once
// Use this function to desloc BIT_POSEs
#define BIT_POSE(x) (1U << x)

#define ICM_20948_DEVICE_ADDRESS 0x69

#define __REG_BANK_SEL 0x7F
#define __USER_BANK_0_ADDR 0
#define __USER_BANK_1_ADDR 1
#define __USER_BANK_2_ADDR 2
#define __USER_BANK_3_ADDR 3

typedef uint8_t u8;

namespace __USER_BANK_0
{

    enum REGISTERS
    {
        __WHO_AM_I = 0x00,
        __USER_CTRL = 0x03,
        __LP_CONFIG_ = 0x05,
        __PWR_MGMT_1 = 0x06,
        __PWR_MGMT_2 = 0x07,
        __INT_PIN_CFG = 0x0F,
        __INT_ENABLE = 0x10,
        __INT_ENABLE_1 = 0x11,
        __INT_ENABLE_2 = 0x12,
        __INT_ENABLE_3 = 0x13,
        __I2C_MST_STATUS = 0x17,
        __INT_STATUS = 0x19,
        __INT_STATUS_1 = 0x1A,
        __INT_STATUS_2 = 0x1B,
        __INT_STATUS_3 = 0x1C,
        __DELAY_TIMEH = 0x28,
        __DELAY_TIMEL = 0x29,
        __ACCEL_XOUT_H = 0x2D,
        __ACCEL_XOUT_L = 0x2E,    
        __ACCEL_YOUT_H = 0x2F,
        __ACCEL_YOUT_L = 0x30,   
        __ACCEL_ZOUT_H = 0x31,
        __ACCEL_ZOUT_L = 0x32,
        __GYRO_XOUT_H = 0x33,
        __GYRO_XOUT_L = 0x34,
        __GYRO_YOUT_H = 0x35,
        __GYRO_YOUT_L = 0x36,
        __GYRO_ZOUT_H = 0x37,
        __GYRO_ZOUT_L = 0x38,
        __TEMP_OUT_H = 0x39,
        __TEMP_OUT_L = 0x3A,
        __EXT_SLV_SENS_DATA_00 = 0x3B,
        __EXT_SLV_SENS_DATA_01 = 0x3C,
        __EXT_SLV_SENS_DATA_02 = 0x3D,
        __EXT_SLV_SENS_DATA_03 = 0x3E,
        __EXT_SLV_SENS_DATA_04 = 0x3F,
        __EXT_SLV_SENS_DATA_05 = 0x40,
        __EXT_SLV_SENS_DATA_06 = 0x41,
        __EXT_SLV_SENS_DATA_07 = 0x42,
        __EXT_SLV_SENS_DATA_08 = 0x43,     
        __EXT_SLV_SENS_DATA_09 = 0x44,    
        __EXT_SLV_SENS_DATA_10 = 0x45,    
        __EXT_SLV_SENS_DATA_11 = 0x46,    
        __EXT_SLV_SENS_DATA_12 = 0x47,
        __EXT_SLV_SENS_DATA_13 = 0x48,      
        __EXT_SLV_SENS_DATA_14 = 0x49,      
        __EXT_SLV_SENS_DATA_15 = 0x4A,      
        __EXT_SLV_SENS_DATA_16 = 0x4B,
        __EXT_SLV_SENS_DATA_17 = 0x4C,  
        __EXT_SLV_SENS_DATA_18 = 0x4D,  
        __EXT_SLV_SENS_DATA_19 = 0x4E,
        __EXT_SLV_SENS_DATA_20 = 0x4F,      
        __EXT_SLV_SENS_DATA_21 = 0x50,      
        __EXT_SLV_SENS_DATA_22 = 0x51,      
        __EXT_SLV_SENS_DATA_23 = 0x52,   
        __FIFO_EN_1 = 0x66,
        __FIFO_EN_2 = 0x67,
        __FIFO_RST = 0x68,
        __FIFO_MODE = 0x69,
        __FIFO_COUNTH = 0x70,
        __FIFO_COUNT_L = 0x71,
        __FIFO_R_W = 0x72,
        __DATA_RDY_STATUS = 0x74,
        __FIFO_CFG = 0x76,
    };

    typedef struct
    {
        u8 : 1;
        u8 __I2C_MST_RST : 1;
        u8 __SRAM_RST : 1;
        u8 __DMP_RST : 1;
        u8 __I2C_IF_DIS : 1;
        u8 __I2C_MST_EN : 1;
        u8 __FIFO_EN : 1;
        u8 __DMP_EN : 1;
    } user_ctrl_bitmap_t;

    typedef union
    {
        u8 user_ctrl_u8;
        user_ctrl_bitmap_t user_ctrl_bitmap;
    } user_ctrl_t;
    
    typedef struct
    {
        u8 : 4;
        u8 __GYRO_CICLE : 1;
        u8 __ACCEL_CYCLE : 1;
        u8 __I2C_MST_CYCLE : 1;
        u8 : 1;
    } lp_config_bitmap_t;

    typedef union
    {
        u8 lp_config_u8;
        lp_config_bitmap_t lp_config_bitmap;

    } lp_config_t;

    typedef struct
    {
        u8 __CLKSEL : 3;
        u8 __TEMP_DIS : 1;
        u8 : 1;
        u8 __LP_EN : 1;
        u8 __SLEEP : 1;
        u8 __DEVICE_RESET : 1;
    } pwr_mgmt_1_bitmap_t;

    typedef union
    {
        u8 pwr_mgmt_1_u8;
        pwr_mgmt_1_bitmap_t pwr_mgmt_1_bitmap;
    } pwr_mgmt_1_t;

    typedef struct
    {
        u8 __DISABLE_GYRO : 3;
        u8 __DISABLE_ACCEL : 3;
        u8 : 2;
    } pwr_mgmt_2_bitmap_t;

    typedef union
    {
        u8 pwr_mgmt_2_u8;
        pwr_mgmt_2_bitmap_t pwr_mgmt_2_bitmap;
    } pwr_mgmt_2_t;

    typedef struct
    {
        u8 : 1;
        u8 __BY_PASS : 1;
        u8 __FSYNC_INT_MODE_EN : 1;
        u8 __ACTL_FSYNC : 1;
        u8 __INT_ANYRD_2CLEAR : 1;
        u8 __INT1_LATCH_EN : 1;
        u8 __INT1_OPEN : 1;
        u8 __INT1_ACTL : 1;
    } int_pin_cgf_bitmap_t;

    typedef union
    {
        u8 int_pin_cfg_u8;
        int_pin_cgf_bitmap_t int_pin_cfg_bitmap;
    } int_pin_cfg_t;

    typedef struct
    {
        u8 __I2C_MST_INT_EN : 1;
        u8 __DMP_INT1_EN : 1;
        u8 __PLL_RDY_EN : 1;
        u8 __WOM_INT_EN : 1;
        u8 : 3;
        u8 __REG_WOF_EN : 1;
    } int_enable_bitmap_t;

    typedef union
    {
        u8 int_enable_u8;
        int_enable_bitmap_t int_enable_bitmap;
    } int_enable_t;

    typedef struct
    {
        u8 __RAW_DATA_0_RDY_EN : 1;
        u8 : 7;
    } int_enable_1_bitmap_t;

    typedef union
    {
        u8 int_enable_1_u8;
        int_enable_1_bitmap_t int_enable_1_bitmap;
    } int_enable_1_t;

    typedef struct
    {
        u8 __FIFO_OVERFLOW_EN : 5;
        u8 : 3;
    } int_enable_2_bitmap_t;

    typedef union
    {
        u8 int_enable_2_u8;
        int_enable_2_bitmap_t int_enable_2_bitmap;
    } int_enable_2_t;

    typedef struct
    {
        u8 __FIFO_WM_EN : 5;
        u8 : 3;
    } int_enable_3_bitmap_t;

    typedef union
    {
        u8 int_enable_3_u8;
        int_enable_3_bitmap_t int_enable_3_bitmap;
    } int_enable_3_t;

    typedef struct
    {
        u8 __I2C_SLV0_NACK : 1;
        u8 __I2C_SLV1_NACK : 1;
        u8 __I2C_SLV2_NACK : 1;
        u8 __I2C_SLV3_NACK : 1;
        u8 __I2C_SLV4_NACK : 1;
        u8 __I2C_LOST_ARB : 1;
        u8 __I2C_SLV4_DONE : 1;
        u8 __PASS_THROUGH : 1;
    } i2c_mst_status_bitmap_t;

    typedef union
    {
        u8 i2c_mst_status_u8;
        i2c_mst_status_bitmap_t i2c_mst_status_bitmap;
    } i2c_mst_status_t;

    typedef struct 
    {
        u8 __I2C_MST_INT : 1;
        u8 __DMP_INT1 : 1;
        u8 __PLL_RDY_INT : 1;
        u8 __WOM_INT : 1;
        u8 : 4;
    } int_status_bitmap_t;

    typedef union
    {
        u8 int_status_u8;
        int_status_bitmap_t int_status_bitmap;
    } int_status_t;

    /*
    enum INT_STATUS_1
    {
        __RAW_DATA_0_RDY_INT = BIT_POSE(0),
    };

    enum INT_STATUS_2
    {
        __FIFO_OVERFLOW_INT = BIT_POSE(0),
    };

    enum INT_STATUS_3
    {
        __FIFO_WM_INT = BIT_POSE(0),
    };
    */

    typedef struct
    {
        u8 SLV_0_FIFO_EN : 1;
        u8 SLV_1_FIFO_EN : 1;
        u8 SLV_2_FIFO_EN : 1;
        u8 SLV_3_FIFO_EN : 1;
        u8 : 4;
    } fifo_en_1_bitmap_t;

    typedef union
    {
        u8 fifo_en_1_u8;
        fifo_en_1_bitmap_t fifo_en_1_bitmap;
    } fifo_en_1_t;

    typedef struct
    {
        u8 __TEMP_FIFO_EN : 1;
        u8 __GYRO_X_FIFO_EN : 1;
        u8 __GYRO_Y_FIFO_EN : 1;
        u8 __GYRO_Z_FIFO_EN : 1;
        u8 __ACCEL_FIFO_EN : 1;
        u8 : 3;
    } fifo_en_2_bitmap_t;

    typedef union
    {
        u8 fifo_en_2_u8;
        fifo_en_2_bitmap_t fifo_en_2_bitmap;
    } fifo_en_2_t;

    typedef struct
    {
        u8 __FIFO_RESET : 5;
        u8 : 3;
    } fifo_rst_bitmap_t;

    typedef union
    {
        u8 fifo_rst_u8;
        fifo_rst_bitmap_t fifo_rst_bitmap;
    } fifo_rst_t;

    typedef struct
    {
        u8 __FIFO_MODE : 5;
        u8 : 3;
    } fifo_mode_bitmap_t;

    typedef union
    {
        u8 fifo_mode_u8;
        fifo_mode_bitmap_t fifo_mode_bitmap;
    } fifo_mode_t;

    typedef struct
    {
        u8 FIFO_R_W : 8;
    } fifo_r_w_bitmap_t;

    typedef union
    {
        fifo_r_w_bitmap_t fifo_r_w_bitmap;
        u8 fifo_r_w_u8;

    } fifo_r_w_t;

    typedef struct
    {
        u8 __FIFO_CFG : 8;
    } fifo_cfg_bitmap_t;

    typedef union
    {
        fifo_cfg_bitmap_t fifo_cfg_bitmap;
        u8 fifo_cfg_u8;
    } fifo_cfg_t;
}

namespace __USER_BANK_1
{
    enum REGISTERS
    {
        __SELF_TEST_X_GYRO = 0x02,
        __SELF_TEST_Y_GYRO = 0x03,
        __SELF_TEST_Z_GYRO = 0x04,
        __SELF_TEST_X_ACCEL = 0x0E,
        __SELF_TEST_Y_ACCEL = 0x0F,
        __SELF_TEST_Z_ACCEL = 0x10,
        __XA_OFFS_H = 0x14,
        __XA_OFFS_L = 0x15,
        __YA_OFFS_H = 0x17,
        __YA_OFFS_L = 0x18,
        __ZA_OFFS_H = 0x1A,
        __ZA_OFFS_L = 0x1B,
        __TIMEBASE_CORRECTION_PLL = 0x28,
    };
}

namespace __USER_BANK_2
{
    enum REGISTERS
    {
        __GYRO_SMPLRT_DIV = 0x00,
        __GYRO_CONFIG_1 = 0x01,
        __GYRO_CONFIG_2 = 0x02,
        __XG_OFFS_USRH = 0x03,
        __XG_OFFS_USRL = 0x04,
        __YG_OFFS_USRH = 0x05,
        __YG_OFFS_USRL = 0x06,
        __ZG_OFFS_USRH = 0x07,
        __ZG_OFFS_USRL = 0x08,
        __ODR_ALIGN_EN = 0x09,
        __ACCEL_SMPLRT_DIV_1 = 0x10,
        __ACCEL_SMPLRT_DIV_2 = 0x11,
        __ACCEL_INTEL_CTRL = 0x12,
        __ACCEL_WOM_THR = 0x13,
        __ACCEL_CONFIG = 0x14,
        __ACCEL_CONFIG_2 = 0x15,
        __FSYNC_CONFIG = 0x52,
        __TEMP_CONFIG = 0x53,
        __MOD_CTRL_USR = 0x54,
    };

    typedef struct
    {
        u8 __GYRO_SMPLRT_DIV : 8;
    } gyro_smplrt_div_bitmap_t;

    typedef union
    {
        u8 gyro_smplrt_div_u8;
        gyro_smplrt_div_bitmap_t gyro_smplrt_div_bitmap;
    } gyro_smplrt_div_t;

    typedef struct
    {
        u8 __GYRO_FCHOICE : 1;
        u8 __GYRO_FS_SEL : 2;
        u8 __GYRO_DLPFCFG : 3;
        u8 : 2;
    } gyro_config_1_bitmap_t;

    typedef union
    {
        u8 gyro_config_1_u8;
        gyro_config_1_bitmap_t gyro_config_1_bitmap;
    } gyro_config_1_t;

    typedef struct
    {
        u8 __GYRO_AVGCFG : 3;
        u8 __ZGYRO_CTEN : 1;
        u8 __YGYRO_CTEN : 1;
        u8 __XGYRO_CTEN : 1;
        u8 : 2;
    } gyro_config_2_bitmap_t;

    enum ACCEL_INTEL_CTRL
    {
        __ACCEL_INTEL_MODE_INT_INITA_SAMPLE_COMPARE = 0x00,
        __ACCEL_INTEL_MODE_INT_PREVIOUS_SAMPLE = 0x01,
        __ACCEL_INTEL_EN = 0x02
    };

    enum ACCEL_CONFIG
    {
        __ACCEL_FCHOICE_BYPASS_DLPF = 0x00,
        __ACCEL_FCHOICE_EN_ACCEL_DLPF = 0x01,
        __ACCEL_FS_SEL_2G = 0x00,
        __ACCEL_FS_SEL_4G = 0x02,
        __ACCEL_FS_SEL_8G = 0x04,
        __ACCEL_FS_SEL_16G = 0x06,
    };

    enum ACCEL_CONFIG_2
    {
        __DEC3_CFG__AVG_1 = 0x00,
        __DEC3_CFG_AVG_8 = 0x01,
        __DEC3_CFG_AVG_16 = 0x02,
        __DEC3_CFG_AVG_32 = 0x03,
        __AZ_ST_EN_REG = BIT_POSE(2),
        __AY_ST_EN_REG = BIT_POSE(3),
        __AX_ST_EN_REG = BIT_POSE(4),
    };

    enum TEMP_CONFIG
    {
        __TEMP_DLPFCFG_0 = 0x00,
        __TEMP_DLPFCFG_1 = 0x01,
        __TEMP_DLPFCFG_2 = 0x02,
        __TEMP_DLPFCFG_3 = 0x03,
        __TEMP_DLPFCFG_4 = 0x04,
        __TEMP_DLPFCFG_5 = 0x05,
        __TEMP_DLPFCFG_6 = 0x06,
        __TEMP_DLPFCFG_7 = 0x07,
    };

    enum MOD_CTRL_USR
    {
        __REG_LP_DMP_EN = BIT_POSE(0),
    };

    typedef struct
    {
        u8 __ODR_ALIGN_EN : 1;
        u8 : 7;
    } odr_align_en_bitmap_t;

    typedef union
    {
        u8 odr_align_en_u8;
        odr_align_en_bitmap_t odr_align_en_bitmap;
    } odr_align_en_t;
}

namespace __USER_BANK_3
{
    enum REGISTERS
    {
        __I2C_MST_ODR_CONFIG = 0x00,
        __I2C_MST_CTRL = 0x01,
        __I2C_MST_DELAY_CTRL = 0x02,
        __I2C_SLV0_ADDR = 0x03,
        __I2C_SLV0_REG = 0x04,
        __I2C_SLV0_CTRL = 0x05,
        __I2C_SLV0_DO = 0x06,
        __I2C_SLV1_ADDR = 0x07,
        __I2C_SLV1_REG = 0x08,
        __I2C_SLV1_CTRL = 0x09,
        __I2C_SLV1_DO = 0x0A,
        __I2C_SLV2_ADDR = 0x0B,
        __I2C_SLV2_REG = 0x0C,
        __I2C_SLV2_CTRL = 0x0D,
        __I2C_SLV2_DO = 0x0E,
        __I2C_SLV3_ADDR = 0x0F,
        __I2C_SLV3_REG = 0x10,
        __I2C_SLV3_CTRL = 0x11,
        __I2C_SLV3_DO = 0x12,
        __I2C_SLV4_ADDR = 0x13,
        __I2C_SLV4_REG = 0x14,
        __I2C_SLV4_CTRL = 0x15,
        __I2C_SLV4_DO = 0x16,
        __I2C_SLV4_DI = 0x17,
    };

    typedef struct
    {
        u8 __I2C_MST_CLK : 4;
        u8 __I2C_MST_P_NSR : 1;
        u8 : 2;
        u8 __MULT_MST_EN : 1;
    } i2c_mst_ctrl_bitmap_t;

    typedef union
    {
        u8 i2c_mst_ctrl_u8;
        i2c_mst_ctrl_bitmap_t i2c_mst_ctrl_bitmap;
    } i2c_mst_ctrl_t;
}
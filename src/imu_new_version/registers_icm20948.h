#ifndef registers_icmp20948_h
#define registers_icmp20948_h

// Use this function to desloc BIT_POSEs
#define BIT_POSE(x) (1UL << x)

#define ICM_20948_DEVICE_ADDRESS 0x68

#define __REG_BANK_SEL 0x7F
#define __USER_BANK_0_ADDR 0
#define __USER_BANK_1_ADDR 1
#define __USER_BANK_2_ADDR 2
#define __USER_BANK_3_ADDR 3

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

    enum USER_CTRL
    {
        __I2C_MST_RST = BIT_POSE(1),
        __SRAM_RST = BIT_POSE(2),
        __DMP_RST = BIT_POSE(3),
        __I2C_IF_DIS = BIT_POSE(4),
        __I2C_MST_EN = BIT_POSE(5),
        __FIFO_EN = BIT_POSE(6),
        __DMP_EN = BIT_POSE(7),
    };

    enum LP_CONFIG
    {
        __GYRO_CICLE = BIT_POSE(4),
        __ACCEL_CYCLE = BIT_POSE(5),
        __I2C_MST_CYCLE = BIT_POSE(6),
    };

    enum PWR_MGMT_1
    {
        __CLKSEL_INTERNAL_OSCI = 0x00,
        __CLKSEL_CLK_1 = 0x01,
        __CLKSEL_CLK_2 = 0x02,
        __CLKSEL_CLK_3 = 0x03,
        __CLKSEL_CLK_4 = 0x04,
        __CLKSEL_CLK_5 = 0x05,
        __CLKSEL_INTERNAL_OSCI_1 = 0x06,
        __CLKSEL_STOP_CLK = 0x07,
        __TEMP_DIS = BIT_POSE(3),
        __LP_EN = BIT_POSE(5),
        __SLEEP = BIT_POSE(6),
        __DEVICE_RESET = BIT_POSE(7)
    };

    enum PWR_MGMT_2
    {
        __DISABLE_GYRO_ALL_AXES = 0x07,
        __ENABLE_GYRO_ALL_AXES = 0x00,
        __DISABLE_ACCEL_ALL_AXES = 0x38,
        __ENABLE_ACCEL_ALL_AXES = 0x00,
    };

    enum INT_PIN_CFG
    {
        __BYPASS_EN = BIT_POSE(1),
        __FSYNC_INT_MODE_EN = BIT_POSE(2),
        __ACTL_FSYNC = BIT_POSE(3),
        __INT_ANYRD_2CLEAR = BIT_POSE(4),
        __INT1_LATCH_EN = BIT_POSE(5),
        __INT1_OPEN = BIT_POSE(6),
        __INT1_ACTL = BIT_POSE(7),
    };

    enum INT_ENABLE
    {
        __I2C_MST_INT_EN = BIT_POSE(0),
        __DMP_INT1_EN = BIT_POSE(1),
        __PLL_RDY_EN = BIT_POSE(2),
        __WOM_INT_EN = BIT_POSE(3),
        __REG_WOF_EN = BIT_POSE(7),
    };

    enum INT_ENABLE_1
    {
        __RAW_DATA_0_RDY_EN = BIT_POSE(0),
    };

    enum INT_ENABLE_2
    {
        __FIFO_OVERFLOW_DIS = 0x00,
        __FIFO_OVERFLOW_EN = BIT_POSE(0),
    };

    enum INT_ENABLE_3
    {
        __FIFO_WM_DIS = 0x00,
        __FIFO_WM_EN = BIT_POSE(0),
    };

    enum I2C_MST_STATUS
    {
        __I2C_SLV0_NACK = BIT_POSE(0),
        __I2C_SLV1_NACK = BIT_POSE(1),
        __I2C_SLV2_NACK = BIT_POSE(2),
        __I2C_SLV3_NACK = BIT_POSE(3),
        __I2C_SLV4_NACK = BIT_POSE(4),
        __I2C_LOST_ARB = BIT_POSE(5),
        __I2C_SLV4_DONE = BIT_POSE(6),
        __PASS_THROUGH = BIT_POSE(7),
    };

    enum INT_STATUS
    {
        __I2C_MST_INT = BIT_POSE(0),
        __DMP_INT1 = BIT_POSE(1),
        __PLL_RDY_INT = BIT_POSE(2),
        __WOM_INT = BIT_POSE(3),
    };

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

    enum FIFO_EN_1
    {
        __SLV_0_FIFO_EN = BIT_POSE(0),
        __SLV_1_FIFO_EN = BIT_POSE(1),
        __SLV_2_FIFO_EN = BIT_POSE(2),
        __SLV_3_FIFO_EN = BIT_POSE(3),
    };

    enum FIFO_EN_2
    {
        __TEMP_FIFO_EN = BIT_POSE(0),
        __GYRO_X_FIFO_EN = BIT_POSE(1),
        __GYRO_Y_FIFO_EN = BIT_POSE(2),
        __GYRO_Z_FIFO_EN = BIT_POSE(3),
        __ACCEL_FIFO_EN = BIT_POSE(4)
    };

    enum FIFO_RST
    {
        __FIFO_RESET = BIT_POSE(0), 
    };

    enum FIFO_MODE
    {
        __STREAM = 0x00,
        __SNAPSHOT = 0x01,
    };

    enum FIFO_R_W
    {
        // create all write bytes late
    };

    enum FIFO_CFG
    {
        __FIFO_CFG_INTERRUPT = 0x01,
    };
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

    enum GYRO_CONFIG_1
    {
        __GYRO_FCHOICE_DISABLE = 0x00,
        __GYRO_FCHOICE_ENABLE = 0x01,
        __GYRO_FS_SEL_250DPS = 0x00,
        __GYRO_FS_SEL_500DPS = 0x02,
        __GYRO_FS_SEL_1000DPS = 0x04,
        __GYRO_FS_SEL_2000DPS = 0x06,
        __GYRO_DLPCFG_0 = 0x00,
        __GYRO_DLPCFG_1 = 0x08,
        __GYRO_DLPCFG_2 = 0x10,
        __GYRO_DLPCFG_3 = 0x18,
        __GYRO_DLPCFG_4 = 0x20,
        __GYRO_DLPCFG_5 = 0x28,
        __GYRO_DLPCFG_6 = 0x30,
        __GYRO_DLPCFG_7 = 0x38,
    };

    enum GYRO_CONFIG_2
    {
        __GYRO_AVGCFG_1X = 0x00,
        __GYRO_AVGCFG_2X = 0x01,
        __GYRO_AVGCFG_4X = 0x02,
        __GYRO_AVGCFG_8X = 0x03,
        __GYRO_AVGCFG_16X = 0x04,
        __GYRO_AVGCFG_32X = 0x05,
        __GYRO_AVGCFG_64X = 0x06,
        __GYRO_AVGCFG_128X = 0x07,
    };

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
        uint8_t I2C_MST_CLK : 4;
        uint8_t I2C_MST_P_NSR : 1;
        uint8_t : 2;
        uint8_t MULT_MST_EN : 1;
    } _i2c_mst_ctrl;

    enum I2C_MST_CTRL
    {
        //
        //
        I2C_MST_CLK_0 = 0x00,
        I2C_MST_CLK_1 = 0x01,
        I2C_MST_CLK_2 = 0x02,
        I2C_MST_CLK_3 = 0x03,
        I2C_MST_CLK_4 = 0x04,
        I2C_MST_CLK_5 = 0x05,
        I2C_MST_CLK_6 = 0x06,
        I2C_MST_CLK_7 = 0x07,
        I2C_MST_CLK_8 = 0x08,
        I2C_MST_CLK_9 = 0x09,
        I2C_MST_CLK_10 = 0x0A,
        I2C_MST_CLK_11 = 0x0B,
        I2C_MST_CLK_12 = 0x0C,
        I2C_MST_CLK_13 = 0x0D,
        I2C_MST_CLK_14 = 0x0E,
        I2C_MST_CLK_15 = 0x0F,
        I2C_MST_P_NSR = 0x10,
        MULT_MST_EN = 0x80
    };
}

#endif
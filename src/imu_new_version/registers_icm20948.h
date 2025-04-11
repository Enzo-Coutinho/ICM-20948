#pragma once
// Use this function to desloc BIT_POSEs
#define BIT_POSE(x) (1U << x)

#define ICM_20948_DEVICE_ADDRESS 0x69

#define __REG_BANK_SEL 0x7F
#define __USER_BANK_0_ADDR 0
#define __USER_BANK_1_ADDR 1
#define __USER_BANK_2_ADDR 2
#define __USER_BANK_3_ADDR 3

#define DEFAULT_VALUE_WIA_ICM 0xEA
#define DEFAULT_VALUE_WIA_MAG 0x09

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

    typedef union
    {
        u8 user_ctrl_u8;
        struct user_ctrl_bitmap_t
        {
            u8 : 1;
            u8 __I2C_MST_RST : 1;
            u8 __SRAM_RST : 1;
            u8 __DMP_RST : 1;
            u8 __I2C_IF_DIS : 1;
            u8 __I2C_MST_EN : 1;
            u8 __FIFO_EN : 1;
            u8 __DMP_EN : 1;
        } user_ctrl_bitmap;
    } user_ctrl_t;

    typedef union
    {
        u8 lp_config_u8;
        struct lp_config_bitmap_t
        {
            u8 : 4;
            u8 __GYRO_CICLE : 1;
            u8 __ACCEL_CYCLE : 1;
            u8 __I2C_MST_CYCLE : 1;
            u8 : 1;
        } lp_config_bitmap;
    } lp_config_t;

    typedef union
    {
        u8 pwr_mgmt_1_u8;
        struct pwr_mgmt_1_bitmap_t
        {
            u8 __CLKSEL : 3;
            u8 __TEMP_DIS : 1;
            u8 : 1;
            u8 __LP_EN : 1;
            u8 __SLEEP : 1;
            u8 __DEVICE_RESET : 1;
        } pwr_mgmt_1_bitmap;
    } pwr_mgmt_1_t;

    typedef union
    {
        u8 pwr_mgmt_2_u8;
        struct pwr_mgmt_2_bitmap_t
        {
            u8 __DISABLE_GYRO : 3;
            u8 __DISABLE_ACCEL : 3;
            u8 : 2;
        } pwr_mgmt_2_bitmap;
    } pwr_mgmt_2_t;

    typedef union
    {
        u8 int_pin_cfg_u8;
        struct int_pin_cgf_bitmap_t
        {
            u8 : 1;
            u8 __BY_PASS : 1;
            u8 __FSYNC_INT_MODE_EN : 1;
            u8 __ACTL_FSYNC : 1;
            u8 __INT_ANYRD_2CLEAR : 1;
            u8 __INT1_LATCH_EN : 1;
            u8 __INT1_OPEN : 1;
            u8 __INT1_ACTL : 1;
        } int_pin_cgf_bitmap;
    } int_pin_cfg_t;

    typedef union
    {
        u8 int_enable_u8;
        struct int_enable_bitmap_t
        {
            u8 __I2C_MST_INT_EN : 1;
            u8 __DMP_INT1_EN : 1;
            u8 __PLL_RDY_EN : 1;
            u8 __WOM_INT_EN : 1;
            u8 : 3;
            u8 __REG_WOF_EN : 1;
        } int_enable_bitmap;
    } int_enable_t;

    typedef union
    {
        u8 int_enable_1_u8;
        struct int_enable_1_bitmap_t
        {
            u8 __RAW_DATA_0_RDY_EN : 1;
            u8 : 7;
        } int_enable_1_bitmap;
    } int_enable_1_t;

    typedef union
    {
        u8 int_enable_2_u8;
        struct int_enable_2_bitmap_t
        {
            u8 __FIFO_OVERFLOW_EN : 5;
            u8 : 3;
        } int_enable_2_bitmap;
    } int_enable_2_t;

    typedef union
    {
        u8 int_enable_3_u8;
        struct int_enable_3_bitmap_t
        {
            u8 __FIFO_WM_EN : 5;
            u8 : 3;
        } int_enable_3_bitmap;
    } int_enable_3_t;

    typedef union
    {
        u8 i2c_mst_status_u8;
        struct i2c_mst_status_bitmap_t
        {
            u8 __I2C_SLV0_NACK : 1;
            u8 __I2C_SLV1_NACK : 1;
            u8 __I2C_SLV2_NACK : 1;
            u8 __I2C_SLV3_NACK : 1;
            u8 __I2C_SLV4_NACK : 1;
            u8 __I2C_LOST_ARB : 1;
            u8 __I2C_SLV4_DONE : 1;
            u8 __PASS_THROUGH : 1;
        } i2c_mst_status_bitmap;
    } i2c_mst_status_t;

    typedef union
    {
        u8 int_status_u8;
        struct int_status_bitmap_t 
        {
            u8 __I2C_MST_INT : 1;
            u8 __DMP_INT1 : 1;
            u8 __PLL_RDY_INT : 1;
            u8 __WOM_INT : 1;
            u8 : 4;
        } int_status_bitmap;;
    } int_status_t;

    typedef union 
    {
        u8 int_status_1_u8;
        struct int_status_1_bitmap_t
        {
            u8 __RAW_DATA_0_RDY_INT : 1;
            u8 : 7;
        };
    } int_status_1_t;

    typedef union 
    {
        u8 int_status_2_u8;
        struct int_status_2_bitmap_t
        {
            u8 __FIFO_OVERFLOW_INT : 5;
            u8 : 3;
        };
    } int_status_2_t;

    typedef union 
    {
        u8 int_status_3_u8;
        struct int_status_3_bitmap_t
        {
            u8 __FIFO_WM_INT : 5;
            u8 : 3;
        };
    } int_status_3_t;
    
    typedef union 
    {
        u8 delay_timeh_u8;
        struct delay_timeh_bitmap_t
        {
            u8 __DELAY_TIMEH : 8;
        } delay_timeh_bitmap;
    } delay_timeh_t;

    typedef union 
    {
        u8 delay_timel_u8;
        struct delay_timeh_bitmap_t
        {
            u8 __DELAY_TIMEL : 8;
        } delay_timel_bitmap;
    } delay_timel_t;

    typedef union 
    {
        u8 accel_xout_h_u8;
        struct accel_xout_h_bitmap_t
        {
            u8 __ACCEL_XOUT_H : 8;
        } accel_xout_h_bitmap;
    } accel_xout_h_t;

    typedef union 
    {
        u8 accel_xout_l_u8;
        struct accel_xout_l_bitmap_t
        {
            u8 __ACCEL_XOUT_L : 8;
        } accel_xout_l_bitmap;
    } accel_xout_l_t;

    typedef union 
    {
        u8 accel_yout_h_u8;
        struct accel_xout_h_bitmap_t
        {
            u8 __ACCEL_YOUT_H : 8;
        } accel_yout_h_bitmap;
    } accel_yout_h_t;

    typedef union 
    {
        u8 accel_yout_l_u8;
        struct accel_yout_l_bitmap_t
        {
            u8 __ACCEL_YOUT_L : 8;
        } accel_yout_l_bitmap;
    } accel_yout_l_t;

    typedef union 
    {
        u8 accel_zout_h_u8;
        struct accel_zout_h_bitmap_t
        {
            u8 __ACCEL_ZOUT_H : 8;
        } accel_zout_h_bitmap;
    } accel_zout_h_t;

    typedef union 
    {
        u8 accel_zout_l_u8;
        struct accel_zout_l_bitmap_t
        {
            u8 __ACCEL_ZOUT_L : 8;
        } accel_zout_l_bitmap;
    } accel_zout_l_t;

    typedef union 
    {
        u8 gyro_xout_h_u8;
        struct gyro_xout_h_bitmap_t
        {
            u8 __GYRO_XOUT_H : 8;
        } gyro_xout_h_bitmap;
    } gyro_xout_h_t;

    typedef union 
    {
        u8 gyro_xout_l_u8;
        struct gyro_xout_l_bitmap_t
        {
            u8 __GYRO_XOUT_L : 8;
        } gyro_xout_l_bitmap;
    } gyro_xout_l_t;

    typedef union 
    {
        u8 gyro_yout_h_u8;
        struct gyro_yout_h_bitmap_t
        {
            u8 __GYRO_YOUT_H : 8;
        } gyro_yout_h_bitmap;
    } gyro_yout_h_t;

    typedef union 
    {
        u8 gyro_yout_l_u8;
        struct gyro_yout_l_bitmap_t
        {
            u8 __GYRO_YOUT_L : 8;
        } gyro_yout_l_bitmap;
    } gyro_yout_l_t;

    typedef union 
    {
        u8 gyro_zout_h_u8;
        struct gyro_zout_h_bitmap_t
        {
            u8 __GYRO_ZOUT_H : 8;
        } gyro_zout_h_bitmap;
    } gyro_zout_h_t;

    typedef union 
    {
        u8 gyro_zout_l_u8;
        struct gyro_zout_l_bitmap_t
        {
            u8 __GYRO_YOUT_L : 8;
        } gyro_zout_l_bitmap;
    } gyro_zout_l_t;

    typedef union 
    {
        u8 temp_out_h_u8;
        struct temp_out_h_bitmap_t
        {
            u8 __TEMP_OUT_H : 8;
        } temp_out_h_bitmap;
    } temp_out_h_t;

    typedef union 
    {
        u8 temp_out_l_u8;
        struct temp_out_l_bitmap_t
        {
            u8 __TEMP_OUT_L : 8;
        } temp_out_l_bitmap;
    } temp_out_l_t;


    typedef union
    {
        u8 ext_slv_sens_data_xx_u8;
        struct ext_slv_sens_data_xx_bitmap_t
        {
            u8 __EXT_SLV_SENS_DATA_XX : 8;
        } ext_slv_sens_data_xx_bitmap;
    } ext_slv_sens_data_xx_t;
    
    
    typedef union
    {
        u8 fifo_en_1_u8;
        struct fifo_en_1_bitmap_t
        {
            u8 SLV_0_FIFO_EN : 1;
            u8 SLV_1_FIFO_EN : 1;
            u8 SLV_2_FIFO_EN : 1;
            u8 SLV_3_FIFO_EN : 1;
            u8 : 4;
        } fifo_en_1_bitmap;
    } fifo_en_1_t;
    

    typedef union
    {
        u8 fifo_en_2_u8;
        struct fifo_en_2_bitmap_t
        {
            u8 __TEMP_FIFO_EN : 1;
            u8 __GYRO_X_FIFO_EN : 1;
            u8 __GYRO_Y_FIFO_EN : 1;
            u8 __GYRO_Z_FIFO_EN : 1;
            u8 __ACCEL_FIFO_EN : 1;
            u8 : 3;
        } fifo_en_2_bitmap;
    } fifo_en_2_t;

    typedef union
    {
        u8 fifo_rst_u8;
        struct fifo_rst_bitmap_t
        {
            u8 __FIFO_RESET : 5;
            u8 : 3;
        } fifo_rst_bitmap;
    } fifo_rst_t;

    typedef union
    {
        u8 fifo_mode_u8;
        struct fifo_mode_bitmap_t
        {
            u8 __FIFO_MODE : 5;
            u8 : 3;
        } fifo_mode_bitmap;
    } fifo_mode_t;

    typedef union
    {
        u8 fifo_count_h_u8;
        struct fifo_count_h_bitmap_t
        {
            u8 __FIFO_CNT : 5;
            u8 : 3;
        } fifo_count_h_bitmap;
    } fifo_count_h_t;
    
    typedef union
    {
        u8 fifo_count_l_u8;
        struct fifo_count_l_bitmap_t
        {
            u8 __FIFO_CNT : 5;
            u8 : 3;
        } fifo_count_l_bitmap;
    } fifo_count_l_t;

    typedef union
    {
        u8 fifo_r_w_u8;
        struct fifo_r_w_bitmap_t
        {
            u8 FIFO_R_W : 8;
        } fifo_r_w_bitmap;
    } fifo_r_w_t;

    typedef union
    {
        u8 data_rdy_status_u8;
        struct data_rdy_status_bitmap_t
        {
            u8 __RAW_DATA_RDY : 4;
            u8 : 3;
            u8 __WOF_STATUS : 1;
        };
    } data_rdy_status_t;
    

    typedef union
    {
        u8 fifo_cfg_u8;
        struct fifo_cfg_bitmap_t
        {
            u8 __FIFO_CFG : 8;
        } fifo_cfg_bitmap;
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

    typedef union 
    {
        u8 self_test_x_gyro_u8;
        struct self_test_x_gyro_bitmap_t
        {
            u8 __XG_ST_DATA : 8;
        } self_test_x_gyro_bitmap;
    } self_test_x_gyro_t;

    typedef union 
    {
        u8 self_test_y_gyro_u8;
        struct self_test_y_gyro_bitmap_t
        {
            u8 __YG_ST_DATA : 8;
        } self_test_y_gyro_bitmap;
    } self_test_y_gyro_t;

    typedef union 
    {
        u8 self_test_z_gyro_u8;
        struct self_test_z_gyro_bitmap_t
        {
            u8 __ZG_ST_DATA : 8;
        } self_test_z_gyro_bitmap;
    } self_test_z_gyro_t;

    typedef union 
    {
        u8 self_test_x_accel_u8;
        struct self_test_x_accel_bitmap_t
        {
            u8 __XA_ST_DATA : 8;
        } self_test_x_accel_bitmap;
    } self_test_x_accel_t;

    typedef union 
    {
        u8 self_test_y_accel_u8;
        struct self_test_y_accel_bitmap_t
        {
            u8 __YA_ST_DATA : 8;
        } self_test_y_accel_bitmap;
    } self_test_y_accel_t;

    typedef union 
    {
        u8 self_test_z_accel_u8;
        struct self_test_z_accel_bitmap_t
        {
            u8 __ZA_ST_DATA : 8;
        } self_test_z_accel_bitmap;
    } self_test_z_accel_t;

    typedef union
    {
        u8 xa_offs_h_u8;
        struct xa_offs_h_bitmap_t
        {
            u8 __XA_OFFS_H : 8;
        } xa_offs_h_bitmap;
    } xa_offs_h_t;

    typedef union
    {
        u8 xa_offs_l_u8;
        struct xa_offs_l_bitmap_t
        {
            u8 : 1;
            u8 __XA_OFFS_L : 7;
        } xa_offs_l_bitmap;
    } xa_offs_l_t;

    typedef union
    {
        u8 ya_offs_h_u8;
        struct ya_offs_h_bitmap_t
        {
            u8 __YA_OFFS_H : 8;
        } ya_offs_h_bitmap;
    } ya_offs_h_t;

    typedef union
    {
        u8 ya_offs_l_u8;
        struct ya_offs_l_bitmap_t
        {
            u8 : 1;
            u8 __YA_OFFS_L : 7;
        } ya_offs_l_bitmap;
    } ya_offs_l_t;

    typedef union
    {
        u8 za_offs_h_u8;
        struct za_offs_h_bitmap_t
        {
            u8 __ZA_OFFS_H : 8;
        } za_offs_h_bitmap;
    } za_offs_h_t;

    typedef union
    {
        u8 za_offs_l_u8;
        struct za_offs_l_bitmap_t
        {
            u8 : 1;
            u8 __ZA_OFFS_L : 7;
        } za_offs_l_bitmap;
    } za_offs_l_t;

    typedef union
    {
        u8 time_base_correction_pll_u8;
        struct time_base_correction_pll_bitmap_t
        {
            u8 __TIME_BASE_CORRECTION_PLL : 8;
        } time_base_correction_pll_bitmap;
    } timebase_correction_pll_t;
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

    typedef union
    {
        u8 gyro_smplrt_div_u8;
        struct gyro_smplrt_div_bitmap_t
        {
            u8 __GYRO_SMPLRT_DIV : 8;
        } gyro_smplrt_div_bitmap;
    } gyro_smplrt_div_t;


    typedef union
    {
        u8 gyro_config_1_u8;
        struct gyro_config_1_bitmap_t
        {
            u8 __GYRO_FCHOICE : 1;
            u8 __GYRO_FS_SEL : 2;
            u8 __GYRO_DLPFCFG : 3;
            u8 : 2;
        } gyro_config_1_bitmap;
    } gyro_config_1_t;

    typedef union
    {
        u8 gyro_config_2_u8;
        struct gyro_config_2_bitmap_t
        {
            u8 __GYRO_AVGCFG : 3;
            u8 __ZGYRO_CTEN : 1;
            u8 __YGYRO_CTEN : 1;
            u8 __XGYRO_CTEN : 1;
            u8 : 2;
        } gyro_config_2_bitmap;
    } gyro_config_2_t;

    typedef union
    {
        u8 xg_offs_usrh_u8;
        struct xg_offs_usrh_bitmap_t
        {
            u8 __XG_OFFS_USRH : 8;
        } xg_offs_usrh_bitmap;
    } xg_offs_usrh_t;

    typedef union
    {
        u8 xg_offs_usrl_u8;
        struct xg_offs_usrl_bitmap_t
        {
            u8 __XG_OFFS_USRL : 8;
        } xg_offs_usrl_bitmap;
    } xg_offs_usrl_t;

    typedef union
    {
        u8 yg_offs_usrh_u8;
        struct yg_offs_usrh_bitmap_t
        {
            u8 __YG_OFFS_USRH : 8;
        } yg_offs_usrh_bitmap;
    } yg_offs_usrh_t;

    typedef union
    {
        u8 yg_offs_usrl_u8;
        struct yg_offs_usrl_bitmap_t
        {
            u8 __YG_OFFS_USRL : 8;
        } yg_offs_usrl_bitmap;
    } yg_offs_usrl_t;

    typedef union
    {
        u8 zg_offs_usrh_u8;
        struct zg_offs_usrh_bitmap_t
        {
            u8 __ZG_OFFS_USRH : 8;
        } zg_offs_usrh_bitmap;
    } zg_offs_usrh_t;

    typedef union
    {
        u8 zg_offs_usrl_u8;
        struct zg_offs_usrl_bitmap_t
        {
            u8 __ZG_OFFS_USRL : 8;
        } zg_offs_usrl_bitmap;
    } zg_offs_usrl_t;

    typedef union
    {
        u8 odr_align_en_u8;
        struct odr_align_en_bitmap_t
        {
            u8 __ODR_ALIGN_EN : 1;
            u8 : 7;
        } odr_align_en_bitmap;
    } odr_align_en_t;

    typedef union 
    {
        u8 accel_smplrt_div_1_u8;
        struct accel_smplrt_div_1_bitmap_t
        {
            u8 __ACCEL_SMPLRT_DIV : 4;
            u8 : 4;
        } accel_smplrt_div_1_bitmap;
    } accel_smplrt_div_1_t;

    typedef union 
    {
        u8 accel_smplrt_div_2_u8;
        struct accel_smplrt_div_2_bitmap_t
        {
            u8 __ACCEL_SMPLRT_DIV : 8;
        } accel_smplrt_div_2_bitmap;
    } accel_smplrt_div_2_t;
    

    typedef union 
    {
        u8 accel_intel_ctrl_u8;
        struct accel_intel_ctrl_bitmap_t
        {
            u8 __ACCEL_INTEL_MODE_INT : 1;
            u8 __ACCEL_INTEL_EN : 1;
            u8 : 6;
        } accel_intel_ctrl_bitmap;
    } accel_intel_ctrl_t;

    typedef union 
    {
        u8 accel_wom_thr_u8;
        struct accel_wom_thr_bitmap_t
        {
            u8 __WOM_THRESHOLD : 8;
        } accel_wom_thr_bitmap;
    } accel_wom_thr_t;
    
    typedef union 
    {
        u8 accel_config_u8;
        struct accel_config_bitmap_t
        {
            u8 __ACCEL_FCHOICE : 1;
            u8 __ACCEL_FS_SEL : 2;
            u8 __ACCEL_DLPFCFG : 3;
            u8 : 2;
        } accel_config_bitmap;
    } accel_config_t;
    
    typedef union 
    {
        u8 accel_config_2_u8;
        struct accel_config_2_bitmap_t
        {
            u8 __DEC3_CFG : 2;
            u8 __AZ_ST_EN_REG : 1;
            u8 __AY_ST_EN_REG : 1;
            u8 __AX_ST_EN_REG : 1;
            u8 : 3;
        } accel_config_2_bitmap;
    } accel_config_2_t;

    typedef union 
    {
        u8 fsync_config_u8;
        struct fysync_config_bitmap_t
        {
            u8 __EXT_SYNC_SET : 4;
            u8 __WOF_EDGE_INT : 1;
            u8 __WOF_DEGLITCH_EN : 1;
            u8 : 1;
            u8 __DELAY_TIME_EN : 1;
        } fysync_config_bitmap;
    } fysync_config_t;
    

    typedef union 
    {
        u8 temp_config_u8;
        struct temp_config_bitmap_t
        {
            u8 __TEMP_DLPFCFG : 3;
        } temp_config_bitmap;
    } temp_config_t;
    

    typedef union 
    {
        u8 mod_ctrl_usr_u8;
        struct mod_ctrl_usr_bitmap_t
        {
            u8 __REG_LP_DMP_EN : 1;
            u8 : 7;
        } mod_ctrl_usr_bitmap;
    } mod_ctrl_usr_t;
    
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

    typedef union 
    {
        u8 i2c_mst_odr_config_u8;
        struct i2c_mst_odr_config_bitmap_t
        {
            u8 i2c_mst_odr_config : 4;
            u8 : 4;
        } i2c_mst_odr_config_bitmap;
    } i2c_mst_odr_config_t;
    
    typedef union
    {
        u8 i2c_mst_ctrl_u8;
        struct i2c_mst_ctrl_bitmap_t
        {
            u8 __I2C_MST_CLK : 4;
            u8 __I2C_MST_P_NSR : 1;
            u8 : 2;
            u8 __MULT_MST_EN : 1;
        } i2c_mst_ctrl_bitmap;
    } i2c_mst_ctrl_t;

    typedef union 
    {
        u8 i2c_mst_delay_ctrl_u8;
        struct i2c_mst_delay_ctrl_bitmap_t
        {
            u8 __I2C_SLV0_DELAY_EN : 1;
            u8 __I2C_SLV1_DELAY_EN : 1;
            u8 __I2C_SLV2_DELAY_EN : 1;
            u8 __I2C_SLV3_DELAY_EN : 1;
            u8 __I2C_SLV4_DELAY_EN : 1;
            u8 : 2;
            u8 __DELAY_ES_SHADOW : 1;
        } i2c_mst_delay_ctrl_bitmap;
    } i2c_mst_delay_ctrl_t;
    
}
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

uint32_t _enabled_Android_0 = 0;
uint32_t _enabled_Android_1 = 0;
uint16_t _dataOutCtl1 = 0;
uint16_t _dataOutCtl2 = 0;
const uint16_t inv_androidSensor_to_control_bits[ANDROID_SENSOR_NUM_MAX] =
    {
        // Data output control 1 register bit definition
        // 16-bit accel                                0x8000
        // 16-bit gyro                                 0x4000
        // 16-bit compass                              0x2000
        // 16-bit ALS                                  0x1000
        // 32-bit 6-axis quaternion                    0x0800
        // 32-bit 9-axis quaternion + heading accuracy 0x0400
        // 16-bit pedometer quaternion                 0x0200
        // 32-bit Geomag rv + heading accuracy         0x0100
        // 16-bit Pressure                             0x0080
        // 32-bit calibrated gyro                      0x0040
        // 32-bit calibrated compass                   0x0020
        // Pedometer Step Detector                     0x0010
        // Header 2                                    0x0008
        // Pedometer Step Indicator Bit 2              0x0004
        // Pedometer Step Indicator Bit 1              0x0002
        // Pedometer Step Indicator Bit 0              0x0001
        // Unsupported Sensors are 0xFFFF

        0xFFFF, // 0  Meta Data
        0x8008, // 1  Accelerometer
        0x0028, // 2  Magnetic Field
        0x0408, // 3  Orientation
        0x4048, // 4  Gyroscope
        0x1008, // 5  Light
        0x0088, // 6  Pressure
        0xFFFF, // 7  Temperature
        0xFFFF, // 8  Proximity <----------- fixme
        0x0808, // 9  Gravity
        0x8808, // 10 Linear Acceleration
        0x0408, // 11 Rotation Vector
        0xFFFF, // 12 Humidity
        0xFFFF, // 13 Ambient Temperature
        0x2008, // 14 Magnetic Field Uncalibrated
        0x0808, // 15 Game Rotation Vector
        0x4008, // 16 Gyroscope Uncalibrated
        0x0000, // 17 Significant Motion
        0x0018, // 18 Step Detector
        0x0010, // 19 Step Counter <----------- fixme
        0x0108, // 20 Geomagnetic Rotation Vector
        0xFFFF, // 21 ANDROID_SENSOR_HEART_RATE,
        0xFFFF, // 22 ANDROID_SENSOR_PROXIMITY,

        0x8008, // 23 ANDROID_SENSOR_WAKEUP_ACCELEROMETER,
        0x0028, // 24 ANDROID_SENSOR_WAKEUP_MAGNETIC_FIELD,
        0x0408, // 25 ANDROID_SENSOR_WAKEUP_ORIENTATION,
        0x4048, // 26 ANDROID_SENSOR_WAKEUP_GYROSCOPE,
        0x1008, // 27 ANDROID_SENSOR_WAKEUP_LIGHT,
        0x0088, // 28 ANDROID_SENSOR_WAKEUP_PRESSURE,
        0x0808, // 29 ANDROID_SENSOR_WAKEUP_GRAVITY,
        0x8808, // 30 ANDROID_SENSOR_WAKEUP_LINEAR_ACCELERATION,
        0x0408, // 31 ANDROID_SENSOR_WAKEUP_ROTATION_VECTOR,
        0xFFFF, // 32 ANDROID_SENSOR_WAKEUP_RELATIVE_HUMIDITY,
        0xFFFF, // 33 ANDROID_SENSOR_WAKEUP_AMBIENT_TEMPERATURE,
        0x2008, // 34 ANDROID_SENSOR_WAKEUP_MAGNETIC_FIELD_UNCALIBRATED,
        0x0808, // 35 ANDROID_SENSOR_WAKEUP_GAME_ROTATION_VECTOR,
        0x4008, // 36 ANDROID_SENSOR_WAKEUP_GYROSCOPE_UNCALIBRATED,
        0x0018, // 37 ANDROID_SENSOR_WAKEUP_STEP_DETECTOR,
        0x0010, // 38 ANDROID_SENSOR_WAKEUP_STEP_COUNTER,
        0x0108, // 39 ANDROID_SENSOR_WAKEUP_GEOMAGNETIC_ROTATION_VECTOR
        0xFFFF, // 40 ANDROID_SENSOR_WAKEUP_HEART_RATE,
        0x0000, // 41 ANDROID_SENSOR_WAKEUP_TILT_DETECTOR,
        0x8008, // 42 Raw Acc
        0x4048, // 43 Raw Gyr
};

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

    enableFIFO(false);

    enableDMP(false);

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
    setDMPmems(ACC_SCALE, 4, accScale);

    const uint8_t accScale2[4] = {0x00, 0x04, 0x00, 0x00};
    setDMPmems(ACC_SCALE, 4, accScale2);

    const uint8_t mountMultiplierZero[4] = {0x00, 0x00, 0x00, 0x00};
    const uint8_t mountMultiplierPlus[4] = {0x09, 0x99, 0x99, 0x99}; 
    const uint8_t mountMultiplierMinus[4] = {0xF6, 0x66, 0x66, 0x67};

    setDMPmems(CPASS_MTX_00, 4, mountMultiplierPlus); 
    setDMPmems(CPASS_MTX_01, 4, mountMultiplierZero); 
    setDMPmems(CPASS_MTX_02, 4, mountMultiplierZero); 
    setDMPmems(CPASS_MTX_10, 4, mountMultiplierZero); 
    setDMPmems(CPASS_MTX_11, 4, mountMultiplierMinus); 
    setDMPmems(CPASS_MTX_12, 4, mountMultiplierZero); 
    setDMPmems(CPASS_MTX_20, 4, mountMultiplierZero); 
    setDMPmems(CPASS_MTX_21, 4, mountMultiplierZero);
    setDMPmems(CPASS_MTX_22, 4, mountMultiplierMinus); 

    const unsigned char b2sMountMultiplierZero[4] = {0x00, 0x00, 0x00, 0x00};
    const unsigned char b2sMountMultiplierPlus[4] = {0x40, 0x00, 0x00, 0x00};

    setDMPmems(B2S_MTX_00, 4, b2sMountMultiplierPlus); 
    setDMPmems(B2S_MTX_01, 4, b2sMountMultiplierZero); 
    setDMPmems(B2S_MTX_02, 4, b2sMountMultiplierZero);
    setDMPmems(B2S_MTX_10, 4, b2sMountMultiplierZero); 
    setDMPmems(B2S_MTX_11, 4, b2sMountMultiplierPlus);
    setDMPmems(B2S_MTX_12, 4, b2sMountMultiplierZero); 
    setDMPmems(B2S_MTX_20, 4, b2sMountMultiplierZero); 
    setDMPmems(B2S_MTX_21, 4, b2sMountMultiplierZero); 
    setDMPmems(B2S_MTX_22, 4, b2sMountMultiplierPlus); 

    __USER_BANK_1::timebase_correction_pll_t tbc_pll = getTIMEBASE_CORRECTION_PLL();

    long gyro_sf;
    int gyro_level = 4;
    unsigned long long const MagicConstant = 264446880937391LL;
    unsigned long long const MagicConstantScale = 100000LL;
    unsigned long long ResultLL;

    if (tbc_pll.time_base_correction_pll_u8 & 0x80)
    {
      ResultLL = (MagicConstant * (long long)(1ULL << gyro_level) * (1 + 19) / (1270 - (tbc_pll.time_base_correction_pll_u8 & 0x7F)) / MagicConstantScale);
    }
    else
    {
      ResultLL = (MagicConstant * (long long)(1ULL << gyro_level) * (1 + 19) / (1270 + tbc_pll.time_base_correction_pll_u8) / MagicConstantScale);
    }

    if (ResultLL > 0x7FFFFFFF)
        gyro_sf = 0x7FFFFFFF;
    else
        gyro_sf = (long)ResultLL;

    unsigned char gyro_sf_reg[4];
    gyro_sf_reg[0] = (unsigned char)(gyro_sf >> 24);
    gyro_sf_reg[1] = (unsigned char)(gyro_sf >> 16);
    gyro_sf_reg[2] = (unsigned char)(gyro_sf >> 8);
    gyro_sf_reg[3] = (unsigned char)(gyro_sf & 0xff);

    setDMPmems(GYRO_SF, 4, gyro_sf_reg);

    const uint8_t gyroFullScale[4] = {0x10, 0x00, 0x00, 0x00}; // 2000dps : 2^28
    setDMPmems(GYRO_FULLSCALE, 4, gyroFullScale); 

    // Configure the Accel Only Gain: 15252014 (225Hz) 30504029 (112Hz) 61117001 (56Hz)
    const uint8_t accelOnlyGain[4] = {0x03, 0xA4, 0x92, 0x49}; // 56Hz
    //const unsigned char accelOnlyGain[4] = {0x00, 0xE8, 0xBA, 0x2E}; // 225Hz
    //const unsigned char accelOnlyGain[4] = {0x01, 0xD1, 0x74, 0x5D}; // 112Hz
    setDMPmems(ACCEL_ONLY_GAIN, 4, accelOnlyGain);

    // Configure the Accel Alpha Var: 1026019965 (225Hz) 977872018 (112Hz) 882002213 (56Hz)
    const uint8_t accelAlphaVar[4] = {0x34, 0x92, 0x49, 0x25}; // 56Hz
    //const unsigned char accelAlphaVar[4] = {0x3D, 0x27, 0xD2, 0x7D}; // 225Hz
    //const unsigned char accelAlphaVar[4] = {0x3A, 0x49, 0x24, 0x92}; // 112Hz
    setDMPmems(ACCEL_ALPHA_VAR, 4, accelAlphaVar); 

    // Configure the Accel A Var: 47721859 (225Hz) 95869806 (112Hz) 191739611 (56Hz)
    const uint8_t accelAVar[4] = {0x0B, 0x6D, 0xB6, 0xDB}; // 56Hz
    //const unsigned char accelAVar[4] = {0x02, 0xD8, 0x2D, 0x83}; // 225Hz
    //const unsigned char accelAVar[4] = {0x05, 0xB6, 0xDB, 0x6E}; // 112Hz
    setDMPmems(ACCEL_A_VAR, 4, accelAVar); 

    // Configure the Accel Cal Rate
    const uint8_t accelCalRate[4] = {0x00, 0x00}; // Value taken from InvenSense Nucleo example
    setDMPmems(ACCEL_CAL_RATE, 2, accelCalRate); 

    // Configure the Compass Time Buffer. The I2C Master ODR Configuration (see above) sets the magnetometer read rate to 68.75Hz.
    // Let's set the Compass Time Buffer to 69 (Hz).
    const uint8_t compassRate[2] = {0x00, 0x45}; // 69Hz
    setDMPmems(CPASS_TIME_BUFFER, 2, compassRate);
}

void enableDMPSensor(enum inv_icm20948_sensor sensor, bool enable)
{
    uint16_t inv_event_control = 0; // Use this to store the value for MOTION_EVENT_CTL
    uint16_t data_rdy_status = 0;   // Use this to store the value for DATA_RDY_STATUS

    uint8_t androidsensor = sensor_type_2_android_sensor(sensor);

    uint16_t delta = inv_androidSensor_to_control_bits[androidsensor];
        if(delta = 0xFFFF)
            return;

    uint64_t androidSensorAsBitMask;
    if(androidsensor < 32)
    {
        androidSensorAsBitMask = (1L << androidsensor);
        if(enable)
        {
            _enabled_Android_0 &- ~androidSensorAsBitMask;
        } else
        {
            _enabled_Android_0 |= androidSensorAsBitMask;
        }
    } else
    {
        androidSensorAsBitMask = (1L << androidsensor);
        if(enable)
        {
            _enabled_Android_1 &- ~androidSensorAsBitMask;
        } else
        {
            _enabled_Android_1 |= androidSensorAsBitMask;
        }
    }

    delta = 0;

    for(int i=0; i<32; i++)
    {
        androidSensorAsBitMask = 1L << i;
        if((_enabled_Android_0 & androidSensorAsBitMask) > 0)
        {
            delta |= inv_androidSensor_to_control_bits[i];
        }
        if((_enabled_Android_1 & androidSensorAsBitMask) > 0)
        {
            delta |= inv_androidSensor_to_control_bits[32 + i];
        }

        if (((_enabled_Android_0 & androidSensorAsBitMask & INV_NEEDS_ACCEL_MASK) > 0)
        || ((_enabled_Android_1 & androidSensorAsBitMask & INV_NEEDS_ACCEL_MASK1) > 0))
        {
            data_rdy_status |= DMP_Data_ready_Accel;
            inv_event_control |= DMP_Motion_Event_Control_Accel_Calibr;
        }
        if (((_enabled_Android_0 & androidSensorAsBitMask & INV_NEEDS_GYRO_MASK) > 0)
        || ((_enabled_Android_1 & androidSensorAsBitMask & INV_NEEDS_GYRO_MASK1) > 0))
        {
            data_rdy_status |= DMP_Data_ready_Gyro;
            inv_event_control |= DMP_Motion_Event_Control_Gyro_Calibr;
        }
        if (((_enabled_Android_0 & androidSensorAsBitMask & INV_NEEDS_COMPASS_MASK) > 0)
        || ((_enabled_Android_1 & androidSensorAsBitMask & INV_NEEDS_COMPASS_MASK1) > 0))
        {
            data_rdy_status |= DMP_Data_ready_Secondary_Compass;
            inv_event_control |= DMP_Motion_Event_Control_Compass_Calibr;
    }
    }

    sleep(false);

    lowPowerMode(false);

    uint16_t delta2 = 0;
    if ((delta & DMP_Data_Output_Control_1_Accel) > 0)
    {
        delta2 |= DMP_Data_Output_Control_2_Accel_Accuracy;
    }
    if (((delta & DMP_Data_Output_Control_1_Gyro_Calibr) > 0) || ((delta & DMP_Data_Output_Control_1_Gyro) > 0))
    {
        delta2 |= DMP_Data_Output_Control_2_Gyro_Accuracy;
    }
    if (((delta & DMP_Data_Output_Control_1_Compass_Calibr) > 0) || ((delta & DMP_Data_Output_Control_1_Compass) > 0) || ((delta & DMP_Data_Output_Control_1_Quat9) > 0) || ((delta & DMP_Data_Output_Control_1_Geomag) > 0))
    {
        delta2 |= DMP_Data_Output_Control_2_Compass_Accuracy;
    }

    unsigned char data_output_control_reg[2];
    data_output_control_reg[0] = (unsigned char)(delta >> 8);
    data_output_control_reg[1] = (unsigned char)(delta & 0xff);
    _dataOutCtl1 = delta; // Diagnostics
    setDMPmems(DATA_OUT_CTL1, 2, data_output_control_reg);

    data_output_control_reg[0] = (unsigned char)(delta2 >> 8);
    data_output_control_reg[1] = (unsigned char)(delta2 & 0xff);
    _dataOutCtl2 = delta2;
    setDMPmems(DATA_OUT_CTL2, 2, data_output_control_reg);

    data_output_control_reg[0] = (unsigned char)(data_rdy_status >> 8);
    data_output_control_reg[1] = (unsigned char)(data_rdy_status & 0xff);
    setDMPmems(DATA_RDY_STATUS, 2, data_output_control_reg);

    if ((delta & DMP_Data_Output_Control_1_Quat9) > 0)
    {
        inv_event_control |= DMP_Motion_Event_Control_9axis;
    }
    if (((delta & DMP_Data_Output_Control_1_Step_Detector) > 0) || ((delta & DMP_Data_Output_Control_1_Step_Ind_0) > 0) || ((delta & DMP_Data_Output_Control_1_Step_Ind_1) > 0) || ((delta & DMP_Data_Output_Control_1_Step_Ind_2) > 0))
    {
        inv_event_control |= DMP_Motion_Event_Control_Pedometer_Interrupt;
    }
    if ((delta & DMP_Data_Output_Control_1_Geomag) > 0)
    {
        inv_event_control |= DMP_Motion_Event_Control_Geomag;
    }

    data_output_control_reg[0] = (unsigned char)(inv_event_control >> 8);
    data_output_control_reg[1] = (unsigned char)(inv_event_control & 0xff);

    setDMPmems(MOTION_EVENT_CTL, 2, data_output_control_reg);
}

void sleep(bool sleep)
{
    pwr_mgmt_1 = getPWR_MGMT_1();
    pwr_mgmt_1.pwr_mgmt_1_bitmap.__SLEEP = (uint8_t)sleep;
    setPWR_MGMT_1(pwr_mgmt_1);
}

void lowPowerMode(bool enable)
{
    pwr_mgmt_1 = getPWR_MGMT_1();
    pwr_mgmt_1.pwr_mgmt_1_bitmap.__LP_EN = (uint8_t)enable;
    setPWR_MGMT_1(pwr_mgmt_1);
}

void enableDMP(bool enable)
{
    user_ctrl = getUSER_CTRL();
    user_ctrl.user_ctrl_bitmap.__DMP_EN = enable;
    setUSER_CTRL(user_ctrl);
}

void enableFIFO(bool enable)
{
    user_ctrl = getUSER_CTRL();
    user_ctrl.user_ctrl_bitmap.__FIFO_EN = enable;
    setUSER_CTRL(user_ctrl);
}

void i2cConfigurePeripheral(uint8_t slvNumber, uint8_t address, uint8_t reg, uint8_t len, bool rw, bool enable, bool data_only, bool grp, bool swap, uint8_t dataOut)
{
    // PRECISA ADICIONAR UM SWITCH COM OS ENDERECOS CERTOS AQUI 
    __USER_BANK_3::i2c_slvx_addr_t slvx_addr = {.i2c_slvx_addr_bitmap = {.__I2C_ID_X = address, .__I2C_SLVX_RNW=rw}};

    setI2C_SLVX_ADDR(slvNumber, slvx_addr);

    if(!rw)
        setI2C_SLVX_DO(slvNumber, dataOut);

    setI2C_SLVX_REG(slvNumber, reg);

    __USER_BANK_3::i2c_slvx_ctrl_t slvx_ctrl = {.i2c_slvx_ctrl_bitmap = {.__I2C_SLVX_LENG = len, .__I2C_SLVX_GRP=grp, .__I2C_SLVX_REG_DIS=data_only, .__I2C_SLVX_BYTE_SW=swap, .__I2C_SLVX_EN=enable}};

    setI2C_SLVX_CTRL(slvNumber, slvx_ctrl);
}


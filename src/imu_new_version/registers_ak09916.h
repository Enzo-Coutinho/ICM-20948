#ifndef registers_ak09916_h
#define registers_ak09916_h

#include <stdint.h>
#include "registers_icm20948.h"

#define AK_09916_DEVICE_ADDRESS 0x0C

namespace _AK_09916
{
    enum REGISTERS
    {
        __WIA_2 = 0x01,
        __ST_1 = 0x10,
        __HXL = 0x11,
        __HXH = 0x12,
        __HYL = 0x13,
        __HYH = 0x14,
        __HZL = 0x15,
        __HZH = 0x16,
        __ST_2 = 0x18,
        __CNTL_2 = 0x31,
        __CNTL_3 = 0x32,
        __TS_1 = 0x33, // DO NOT ACESS
        __TS_2 = 0x34, // DO NOT ACESS
    };

    enum CNTL_2
    {
        PD_MODE = 0x00,
        SING_MEAS_MODE = 0x01,
        CONT_MEAS_MODE_1 = 0x02,
        CONT_MEAS_MODE_2 = 0x04,
        CONT_MEAS_MODE_3 = 0x06,
        CONT_MEAS_MODE_4 = 0x08,
        SELF_TEST_MODE = 0x10
    };

    enum CNTL_3
    {
        SRST = 0x01,
    };
}

#endif
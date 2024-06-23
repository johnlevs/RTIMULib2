  ////////////////////////////////////////////////////////////////////////////
  //
  //  This file is part of RTIMULib
  //
  //  Copyright (c) 2014-2015, richards-tech, LLC
  //
  //  Permission is hereby granted, free of charge, to any person obtaining a copy of
  //  this software and associated documentation files (the "Software"), to deal in
  //  the Software without restriction, including without limitation the rights to use,
  //  copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
  //  Software, and to permit persons to whom the Software is furnished to do so,
  //  subject to the following conditions:
  //
  //  The above copyright notice and this permission notice shall be included in all
  //  copies or substantial portions of the Software.
  //
  //  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
  //  INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
  //  PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
  //  HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
  //  OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
  //  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.



#ifndef RTIMULSM6DSLMAP_H
#define RTIMULSM6DSLMAP_H

#include "RTIMU.h"

namespace LSM6DSL {
    
    // Register definitions
    typedef struct _CTRL1_XL_t {
        union {
            struct {
                uint8_t bw0_XL     : 1;
                uint8_t lpf1_bw_sel: 1;
                uint8_t fs_XL      : 2;
                uint8_t odr_XL     : 4;
            };
            uint8_t value;
        };
    }CTRL1_XL_t;

    typedef struct _CTRL2_G_t {
        union {
            struct {
                uint8_t reserved: 1;
                uint8_t fs_125  : 1;
                uint8_t fs_G    : 2;
                uint8_t odr_G   : 4;
            };
            uint8_t value;
        };
    }CTRL2_G_t;

    typedef struct _CTRL3_C_t {
        union {
            struct {
                uint8_t swReset  : 1;
                uint8_t ble      : 1;
                uint8_t ifInc    : 1;
                uint8_t sim      : 1;
                uint8_t pp_od    : 1;
                uint8_t h_lactive: 1;
                uint8_t bdu      : 1;
                uint8_t boot     : 1;
            };
            uint8_t value;
        };
    }CTRL3_C_t;

    typedef struct _CTRL5_C_t{
        union{
            struct{
                uint8_t st_xl   : 2;
                uint8_t st_g    : 2;
                uint8_t den_lh  : 1;
                uint8_t rounding: 3;
            };
            uint8_t value;
        };
    }CTRL5_C_t;

    typedef struct _CTRL10_C_t{
        union{
            struct{
                uint8_t sign_motion_en: 1;
                uint8_t ped_rst_step  : 1;
                uint8_t func_en       : 1;
                uint8_t tilt_en       : 1;
                uint8_t pedo_en       : 1;
                uint8_t timer_en      : 1;
                uint8_t wrist_tilt_en : 1;
            };
            uint8_t value;
        };
    }CTRL10_C_t;

    typedef struct _WAKE_UP_DUR_t{
        union{
            struct{
                uint8_t sleep_dur: 4;
                uint8_t timer_hr : 1;
                uint8_t wake_dur : 2;
                uint8_t ff_dur   : 1;
            };
            uint8_t value;
        };
    }WAKE_UP_DUR_t;



    // nice to have constants
    constexpr RTFLOAT GYRO_FSR[5] = { .004375, .00875 , .01750, .035, .070 };  // dps/lsb 
    constexpr RTFLOAT ACCEL_FSR[4] = { 6.1e-5, 4.88e-4, 2.44e-4, 1.22e-4 };   //G's/lsb
    constexpr RTFLOAT ODR_SCALE = 13.0;                                     // Hz



    // register map
    enum {
        FUNC_CFG_ACCESS          = 0x1,    // r/w
        SENSOR_SYNC_TIME_FRAME   = 0x4,    // r/w
        SENSOR_SYNC_RES_RATIO    = 0x5,    // r/w
        FIFO_CTRL1               = 0x6,    // r/w
        FIFO_CTRL2               = 0x7,    // r/w
        FIFO_CTRL3               = 0x8,    // r/w
        FIFO_CTRL4               = 0x9,    // r/w
        FIFO_CTRL5               = 0x0A,   // r/w
        DRDY_PULSE_CFG_G         = 0x0B,   // r/w
        INT1_CTRL                = 0x0D,   // r/w
        INT2_CTRL                = 0x0E,   // r/w
        WHO_AM_I                 = 0x0F,   // r
        CTRL1_XL                 = 0x10,   // r/w
        CTRL2_G                  = 0x11,   // r/w
        CTRL3_C                  = 0x12,   // r/w
        CTRL4_C                  = 0x13,   // r/w
        CTRL5_C                  = 0x14,   // r/w
        CTRL6_C                  = 0x15,   // r/w
        CTRL7_G                  = 0x16,   // r/w
        CTRL8_XL                 = 0x17,   // r/w
        CTRL9_XL                 = 0x18,   // r/w
        CTRL10_C                 = 0x19,   // r/w
        MASTER_CONFIG            = 0x1A,   // r/w
        WAKE_UP_SRC              = 0x1B,   // r
        TAP_SRC                  = 0x1C,   // r
        D6D_SRC                  = 0x1D,   // r
        STATUS_REG               = 0x1E,   // r
        OUT_TEMP_L               = 0x20,   // r
        OUT_TEMP_H               = 0x21,   // r
        OUTX_L_G                 = 0x22,   // r
        OUTX_H_G                 = 0x23,   // r
        OUTY_L_G                 = 0x24,   // r
        OUTY_H_G                 = 0x25,   // r
        OUTZ_L_G                 = 0x26,   // r
        OUTZ_H_G                 = 0x27,   // r
        OUTX_L_XL                = 0x28,   // r
        OUTX_H_XL                = 0x29,   // r
        OUTY_L_XL                = 0x2A,   // r
        OUTY_H_XL                = 0x2B,   // r
        OUTZ_L_XL                = 0x2C,   // r
        OUTZ_H_XL                = 0x2D,   // r
        SENSORHUB1_REG           = 0x2E,   // r
        SENSORHUB2_REG           = 0x2F,   // r
        SENSORHUB3_REG           = 0x30,   // r
        SENSORHUB4_REG           = 0x31,   // r
        SENSORHUB5_REG           = 0x32,   // r
        SENSORHUB6_REG           = 0x33,   // r
        SENSORHUB7_REG           = 0x34,   // r
        SENSORHUB8_REG           = 0x35,   // r
        SENSORHUB9_REG           = 0x36,   // r
        SENSORHUB10_REG          = 0x37,   // r
        SENSORHUB11_REG          = 0x38,   // r
        SENSORHUB12_REG          = 0x39,   // r
        FIFO_STATUS1             = 0x3A,   // r
        FIFO_STATUS2             = 0x3B,   // r
        FIFO_STATUS3             = 0x3C,   // r
        FIFO_STATUS4             = 0x3D,   // r
        FIFO_DATA_OUT_L          = 0x3E,   // r
        FIFO_DATA_OUT_H          = 0x3F,   // r
        TIMESTAMP0_REG           = 0x40,   // r
        TIMESTAMP1_REG           = 0x41,   // r
        TIMESTAMP2_REG           = 0x42,   // r/w
        STEP_TIMESTAMP_L         = 0x49,   // r
        STEP_TIMESTAMP_H         = 0x4A,   // r
        STEP_COUNTER_L           = 0x4B,   // r
        STEP_COUNTER_H           = 0x4C,   // r
        SENSORHUB13_REG          = 0x4D,   // r
        SENSORHUB14_REG          = 0x4E,   // r
        SENSORHUB15_REG          = 0x4F,   // r
        SENSORHUB16_REG          = 0x50,   // r
        SENSORHUB17_REG          = 0x51,   // r
        SENSORHUB18_REG          = 0x52,   // r
        FUNC_SRC1                = 0x53,   // r
        FUNC_SRC2                = 0x54,   // r
        WRIST_TILT_IA            = 0x55,   // r
        TAP_CFG                  = 0x58,   // r/w
        TAP_THS_6D               = 0x59,   // r/w
        INT_DUR2                 = 0x5A,   // r/w
        WAKE_UP_THS              = 0x5B,   // r/w
        WAKE_UP_DUR              = 0x5C,   // r/w
        FREE_FALL                = 0x5D,   // r/w
        MD1_CFG                  = 0x5E,   // r/w
        MD2_CFG                  = 0x5F,   // r/w
        MASTER_CMD_CODE          = 0x60,   // r/w
        SENS_SYNC_SPI_ERROR_CODE = 0x61,   // r/w
        OUT_MAG_RAW_X_L          = 0x66,   // r
        OUT_MAG_RAW_X_H          = 0x67,   // r
        OUT_MAG_RAW_Y_L          = 0x68,   // r
        OUT_MAG_RAW_Y_H          = 0x69,   // r
        OUT_MAG_RAW_Z_L          = 0x6A,   // r
        OUT_MAG_RAW_Z_H          = 0x6B,   // r
        X_OFS_USR                = 0x73,   // r/w
        Y_OFS_USR                = 0x74,   // r/w
        Z_OFS_USR                = 0x75,   // r/w
    };


        // Selectable odr values
    enum {
        ODR_OFF    = 0,
        ODR_0012_5 = 1,
        ODR_0026   = 2,
        ODR_0052   = 3,
        ODR_0104   = 4,
        ODR_0208   = 5,
        ODR_0416   = 6,
        ODR_0833   = 7,
        ODR_1660   = 8,
        ODR_3330   = 9,
        ODR_6660   = 10
    };

        // Selectable accel fs values
    enum {
        FS_02G = 0,
        FS_16G = 1,
        FS_04G = 2,
        FS_08G = 3,
    };

        // Selectable gyro fs values
    enum {
        FS_0125DPS = 0,
        FS_0250DPS = 1,
        FS_0500DPS = 2,
        FS_1000DPS = 3,
        FS_2000DPS = 4,
    };

        // accel filter config options
    enum {
        ACCEL_FILTER_SEL_BYPASS = 0,
        ACCEL_FILTER_SEL_LPF2   = 1,
        ACCEL_FILTER_SEL_HPF    = 2,
    };

    enum {
        ACCEL_AAF_CUTOFF_1500 = 0,
        ACCEL_AAF_CUTTOF_0400 = 1,
    };
};




#endif  // RTIMULSM6DSLMAP_H
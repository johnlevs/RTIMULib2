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


#ifndef _RTIMULSM9DS1_H
#define	_RTIMULSM9DS1_H

#include "RTIMU.h"

//  Define this symbol to use cache mode

class RTIMULSM9DS1 : public RTIMU
{
public:
    RTIMULSM9DS1(RTIMUSettings *settings);
    ~RTIMULSM9DS1();

    virtual const char *IMUName() { return "LSM9DS1"; }
    virtual int IMUType() { return RTIMU_TYPE_LSM9DS1; }
    virtual bool IMUInit();
    virtual int IMUGetPollInterval();
    virtual bool IMURead();

private:
    // register setters
    
    bool setGyroSampleRate();
    bool setGyroCTRL2();
    bool setGyroCTRL3();
    bool setGyroCTRL4();
    bool setAccelCTRL5();
    bool setAccelCTRL6();
    bool setAccelCTRL7();
    bool setCompassCTRL1();
    bool setCompassCTRL2();
    bool setCompassCTRL3();
    bool setCompassCTRL4();
    bool setCompassCTRL5();
    bool setCtrl9();
    bool setFifoMode();
    bool initializeFifoBuffer();

    // helpers
    inline int checkSettingsValue(int value, int min, int max, int failsafe, const char *registerName) {
        if (value < min || value > max) {
            HAL_ERROR3("Invalid value %d for %s. Setting to default: %d\n", value, registerName, failsafe);
            return failsafe;
        }
        return value;
    }

    void processIMUData();
    bool processCache(bool readOnce);

    static constexpr int M_SAMPLERATE_SCALAR = 952; // 952 Hz
    static constexpr int M_MAG_SAMPLERATE_SCALAR = 80; // 80 Hz
    // one would think that the sensor scale factors are a function of the fsr, but they are not for some wild reason
    const RTFLOAT M_GYRO_SCALE_LOOKUP[4] = {.00875, .0175 , 0, .070 }; // 245, 500, 2000 dps
    const RTFLOAT M_ACCEL_SCALE_LOOKUP[4] = {.000061, .000732, .000122, .000244 }; // 2, 16, 4, 8 g
    const RTFLOAT M_COMPASS_SCALE_LOOKUP[4] = {.00014, .00029, .00043, .00058 }; // 4, 8, 12, 16 gauss  
    
    static constexpr unsigned int M_SENSOR_3_AXIS_BYTE_SIZE = 6;    // size of 3 axis data in bytes (2 bytes per axis, applies for accel, gyro, and compass)
    static constexpr unsigned int M_IMU_FIFO_SIZE_MAX = 3;         // max number of fifo slots per sensor axis

    unsigned char m_imuSampleSize;
    unsigned char m_imuCacheSize;
    unsigned char m_imuReadPtr;

    unsigned char m_accelGyroAddress;
    unsigned char m_accelOnlyAdress;

    unsigned char m_accelGyroSlaveAddr;                     // I2C address of accel andgyro
    unsigned char m_magSlaveAddr;                           // I2C address of mag

    RTFLOAT m_gyroScale;
    RTFLOAT m_accelScale;
    RTFLOAT m_compassScale;

    uint64_t m_magODRInterval;                              //  1e6/ODR for magnetometer
    uint64_t m_lastMagRead;                                 // last time magnetometer was read

    // fifo buffer vars
    RTIMU::RTIMUFifoBuffer m_fifoBuff;                      // fifo buffer
    static constexpr int M_FIFO_CHUNK_SIZE = 32;  
    static constexpr int M_FIFO_SAMPLE_SIZE = 12;


};

#endif // _RTIMULSM9DS1_H

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


#ifndef _RTIMULSM6DSLLIS3MDL_H
#define	_RTIMULSM6DSLLIS3MDL_H
#include "RTIMU.h"
#include "RTIMULSM6DSLMap.h"
class RTIMULSM6DSLLIS3MDL : public RTIMU {
public:
    RTIMULSM6DSLLIS3MDL(RTIMUSettings *settings);
    ~RTIMULSM6DSLLIS3MDL();

    virtual const char *IMUName() { return "LSM6DSL & LIS3MDL"; }
    virtual int IMUType() { return RTIMU_TYPE_LSM6DSLLIS3MDL; }
    virtual bool IMUInit();
    virtual int IMUGetPollInterval();
    virtual bool IMURead();


private:


    // register setters
    bool setCtrl1XL();
    bool setCtrl2G();
    bool setCtrl3C();

    bool setCtrl5c();

    bool setCtrl10c();

    bool setWakeUpDur();

    // data read helpers
    bool fifoRead();
    bool dataRegRead();

    void setUpFusionParameters();


    // read/write helpers
    inline bool _writeReg(uint8_t reg, const uint8_t value, const char *errorMsg) { return m_settings->HALWrite(m_gyroAccelSlaveAddr, reg, value, errorMsg); }
    inline bool _readReg(uint8_t reg, uint8_t *buffer, unsigned short length, const char *errorMsg) { return m_settings->HALRead(m_gyroAccelSlaveAddr, reg, length, buffer, errorMsg); }

    inline int checkSettingsValue(int value, int min, int max, int failsafe, const char *registerName)
    {
        if (value < min || value > max) {
            HAL_ERROR3("Invalid value %d for %s. Setting to default: %d\n", value, registerName, failsafe);
            return failsafe;
        }
        return value;
    }


    uint8_t m_gyroAccelSlaveAddr;

    RTFLOAT m_accelScale;
    RTFLOAT m_gyroScale;

    // Structs to store register values
    LSM6DSL::CTRL1_XL_t m_ctrl1xl;
    LSM6DSL::CTRL2_G_t m_ctrl2g;
    LSM6DSL::CTRL3_C_t m_ctrl3c;
    LSM6DSL::CTRL5_C_t m_ctrl5c;
    LSM6DSL::CTRL10_C_t m_ctrl10c;
    LSM6DSL::WAKE_UP_DUR_t m_wakeUpDur;

    RTIMUFifoBuffer m_fifoCache;

    uint64_t m_timeTagRolloverCount;
    uint64_t m_timeTagBaseOffset;


};


#endif  // _RTIMULSM6DSLLIS3MDL_H
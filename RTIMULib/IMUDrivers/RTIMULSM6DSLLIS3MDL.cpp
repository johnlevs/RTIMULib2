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



#include "RTIMULSM6DSLLIS3MDL.h"

using namespace LSM6DSL;

RTIMULSM6DSLLIS3MDL::RTIMULSM6DSLLIS3MDL(RTIMUSettings *settings) : RTIMU(settings)
{}

RTIMULSM6DSLLIS3MDL::~RTIMULSM6DSLLIS3MDL()
{}

bool RTIMULSM6DSLLIS3MDL::IMUInit()
{

    m_gyroAccelSlaveAddr = m_settings->m_I2CSlaveAddress;

    setCalibrationData();

    if (!m_settings->HALOpen())
        return false;

    // boot imu
    if (!setCtrl3C())
        return false;
    // wait for boot to finish
    usleep(100);

    // set up the gyro/accel
    if (!setCtrl1XL())
        return false;

    if (!setCtrl2G())
        return false;

    if (!setCtrl5c())
        return false;
    gyroBiasInit();

    m_timeTagRolloverCount = 0;
    m_timeTagBaseOffset = 0;
    m_imuData.timestamp = 0;
    dataRegRead();

    // sometimes this imu needs  a bit to get situated
    usleep(50000);

    HAL_INFO("LSM6DSL init complete\n");

    return true;
}

int RTIMULSM6DSLLIS3MDL::IMUGetPollInterval()
{
    return 0;
}

bool RTIMULSM6DSLLIS3MDL::setCtrl1XL()
{
    if (!_readReg(CTRL1_XL, &m_ctrl1xl.value, 1, "Failed to read CTRL1_XL register"))
        return false;

    // Set the output data rate for the accelerometer
    m_ctrl1xl.odr_XL = checkSettingsValue(
        m_settings->m_LSM6DSLAccelSampleRate,
        ODR_OFF,
        ODR_6660,
        LSM6DSL_ACCEL_SAMPLERATE_208,
        RTIMULIB_LSM6DSL_ACCEL_SAMPLERATE
    );

    m_fusion->setAccelEnable(m_ctrl1xl.odr_XL > ODR_OFF);

    // Set the full scale range for the accelerometer
    m_ctrl1xl.fs_XL = checkSettingsValue(
        m_settings->m_LSM6DSLAccelFSR,
        FS_02G,
        FS_08G,
        LSM6DSL_ACCEL_FSR_2,
        RTIMULIB_LSM6DSL_ACCEL_FSR
    );

    m_accelScale = ACCEL_FSR[m_ctrl1xl.fs_XL];

    // Set the filter selection for the accelerometer
    m_ctrl1xl.lpf1_bw_sel = checkSettingsValue(
        m_settings->m_LSM6DSLAccelFilter2Select,
        ACCEL_FILTER_SEL_BYPASS,
        ACCEL_FILTER_SEL_HPF,
        LSM6DSL_ACCEL_FILTER_BYPASS,
        RTIMULIB_LSM6DSL_ACCEL_FILTER
    ) > ACCEL_FILTER_SEL_BYPASS;

    // Set the low pass filter for the accelerometer
    m_ctrl1xl.bw0_XL = checkSettingsValue(
        m_settings->m_LSM6DSLAccelLPFAn,
        ACCEL_AAF_CUTOFF_1500,
        ACCEL_AAF_CUTTOF_0400,
        LSM6DSL_ACCEL_AN_LPF_1500,
        RTIMULIB_LSM6DSL_ACCLE_LPFAn
    );

    return _writeReg(CTRL1_XL, m_ctrl1xl.value, "Failed to write CTRL1_XL register");
}

bool RTIMULSM6DSLLIS3MDL::setCtrl2G()
{
    if (!_readReg(CTRL2_G, &m_ctrl2g.value, 1, "Failed to read CTRL2_G register"))
        return false;

    // Set the output data rate for the gyroscope
    m_ctrl2g.odr_G = checkSettingsValue(
        m_settings->m_LSM6DSLGyroSampleRate,
        ODR_OFF,
        ODR_6660,
        LSM6DSL_GYRO_SAMPLERATE_208,
        RTIMULIB_LSM6DSL_GYRO_SAMPLERATE
    );

    m_fusion->setGyroEnable(m_ctrl2g.odr_G > ODR_OFF);
    m_sampleRate = (1 << (m_ctrl2g.odr_G - 1)) * ODR_SCALE;

    // Set the full scale range for the gyroscope
    uint8_t tempFSR = checkSettingsValue(
        m_settings->m_LSM6DSLGyroFSR,
        FS_0125DPS,
        FS_2000DPS,
        LSM6DSL_GYRO_FSR_250,
        RTIMULIB_LSM6DSL_GYRO_FSR
    );
    if (tempFSR - 1 < 0)
        m_ctrl2g.fs_125 = 1;
    else
        m_ctrl2g.fs_G = tempFSR - 1;
    m_gyroScale = GYRO_FSR[tempFSR] * RTMATH_DEGREE_TO_RAD;



    return _writeReg(CTRL2_G, m_ctrl2g.value, "Failed to write CTRL2_G register");
}

bool RTIMULSM6DSLLIS3MDL::setCtrl3C()
{
    // read in chip defaults
    if (!_readReg(CTRL3_C, &m_ctrl3c.value, 1, "Failed to read CTRL3_C register"))
        return false;

    m_ctrl3c.boot = 1;
    // perform boot
    if (!_writeReg(CTRL3_C, m_ctrl3c.value, "Failed to write CTRL3_C register for boot"))
        return false;
    usleep(2000); // 15 mS boot time

    if (!_readReg(CTRL3_C, &m_ctrl3c.value, 1, "Failed to read CTRL3_C register after boot"))
        return false;

    m_ctrl3c.swReset = 1;
    // perform reset
    if (!_writeReg(CTRL3_C, m_ctrl3c.value, "Failed to write CTRL3_C register for reset"))
        return false;
    usleep(60); // 50 uS reset time

    if (!_readReg(CTRL3_C, &m_ctrl3c.value, 1, "Failed to read CTRL3_C register after reset"))
        return false;

    // set up defaults
    m_ctrl3c.bdu = 1;
    m_ctrl3c.ifInc = 1;

    return _writeReg(CTRL3_C, m_ctrl3c.value, "Failed to write CTRL3_C register");
}

bool RTIMULSM6DSLLIS3MDL::setCtrl5c()
{
    if (!_readReg(CTRL5_C, &m_ctrl5c.value, 1, "Failed to read CTRL5_C register"))
        return false;
    m_ctrl5c.rounding = 0x3;    // enable rounding on gyro & accel data

    return _writeReg(CTRL5_C, m_ctrl5c.value, "Failed to write CTRL5_C register");
}

bool RTIMULSM6DSLLIS3MDL::setCtrl10c()
{
    if (!_readReg(CTRL10_C, &m_ctrl10c.value, 1, "Failed to read CTRL10_C register"))
        return false;

    m_ctrl10c.timer_en = 1;

    return _writeReg(CTRL10_C, m_ctrl10c.value, "Failed to write CTRL10_C register");
}

bool RTIMULSM6DSLLIS3MDL::setWakeUpDur()
{
    if (!_readReg(WAKE_UP_DUR, &m_wakeUpDur.value, 1, "Failed to read WAKE_UP_DUR register"))
        return false;
    m_wakeUpDur.timer_hr = 1; // enable 25 uS lsb
    return _writeReg(WAKE_UP_DUR, m_wakeUpDur.value, "Failed to write WAKE_UP_DUR register");
}

bool RTIMULSM6DSLLIS3MDL::dataRegRead()
{
    uint8_t rawData[OUTZ_H_XL - STATUS_REG + 1];

    if (!_readReg(STATUS_REG, rawData, 1, "Failed to read data registers"))
        return false;
    
    // associate timestamp with data validity read
    m_imuData.timestamp = RTMath::currentUSecsSinceEpoch();
    m_imuData.accelValid = rawData[0] & 1;
    m_imuData.gyroValid = rawData[0] & 2 >> 1;

    if (!(rawData[0] & 3))
        return false;


    if (rawData[0] & 2) {
        if (_readReg(OUTX_L_G, &rawData[4], OUTZ_H_G - OUTX_L_G + 1, "Failed to read gyro data"))    
            RTMath::convertToVector(&rawData[4], m_imuData.gyro, m_gyroScale, false);
    }
    if (rawData[0] & 1){
        if (_readReg(OUTX_L_XL, &rawData[10], OUTZ_H_XL - OUTX_L_XL + 1, "Failed to read accel data"))
            RTMath::convertToVector(&rawData[10], m_imuData.accel, m_accelScale, false);
    }

    
    //processTimeStamp(((uint64_t)rawTime[2] << 16 | (uint64_t)rawTime[1] << 8 | rawTime[0]) * 25);

    return true;
}





bool RTIMULSM6DSLLIS3MDL::IMURead()
{
    if (!dataRegRead())
        return false;


    // sort out gyro axes and correct for bias
    RTFLOAT temp = m_imuData.gyro.x();
    m_imuData.gyro.setX(-m_imuData.gyro.y());
    m_imuData.gyro.setY(-temp);
    m_imuData.gyro.setZ(-m_imuData.gyro.z());

    //  sort out accel data;


    m_imuData.accel.setX(-m_imuData.accel.y());
    m_imuData.accel.setY(-m_imuData.accel.x());

    //  now do standard processing

    handleGyroBias();

    calibrateAccel();

    //  now update the filter

    updateFusion();


    return true;
}
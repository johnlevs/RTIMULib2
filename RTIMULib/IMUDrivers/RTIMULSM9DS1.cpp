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


#include "RTIMULSM9DS1.h"
#include "RTIMUSettings.h"

//  this sets the learning rate for compass running average calculation

#define COMPASS_ALPHA 0.2f

RTIMULSM9DS1::RTIMULSM9DS1(RTIMUSettings *settings) : RTIMU(settings)
{
    m_imuData.timestamp = 0;
    m_lastMagRead = 0;
}

RTIMULSM9DS1::~RTIMULSM9DS1() {}

bool RTIMULSM9DS1::IMUInit()
{
    unsigned char result;

    //  configure IMU

    m_accelGyroSlaveAddr = m_settings->m_I2CSlaveAddress;

    // work outmag address

    if (m_settings->HALRead(LSM9DS1_MAG_ADDRESS0, LSM9DS1_MAG_WHO_AM_I, 1, &result, "")) {
        if (result == LSM9DS1_MAG_ID) {
            m_magSlaveAddr = LSM9DS1_MAG_ADDRESS0;
        }
    } else if (m_settings->HALRead(LSM9DS1_MAG_ADDRESS1, LSM9DS1_MAG_WHO_AM_I, 1, &result, "")) {
        if (result == LSM9DS1_MAG_ID) {
            m_magSlaveAddr = LSM9DS1_MAG_ADDRESS1;
        }
    } else if (m_settings->HALRead(LSM9DS1_MAG_ADDRESS2, LSM9DS1_MAG_WHO_AM_I, 1, &result, "")) {
        if (result == LSM9DS1_MAG_ID) {
            m_magSlaveAddr = LSM9DS1_MAG_ADDRESS2;
        }
    } else if (m_settings->HALRead(LSM9DS1_MAG_ADDRESS3, LSM9DS1_MAG_WHO_AM_I, 1, &result, "")) {
        if (result == LSM9DS1_MAG_ID) {
            m_magSlaveAddr = LSM9DS1_MAG_ADDRESS3;
        }
    }

    setCalibrationData();

    //  enable the I2C bus

    if (!m_settings->HALOpen())
        return false;

    //  Set up the gyro/accel
    //  bit 7 boots imu, bit 6 enables BDU, bit 2 enables auto-increment
    if (!m_settings->HALWrite(m_accelGyroSlaveAddr, LSM9DS1_CTRL8, (1 << 7) | (1 << 6) | (1 << 2), "Failed to boot LSM9DS1"))
        return false;

    m_settings->delayMs(100);

    if (!m_settings->HALRead(m_accelGyroSlaveAddr, LSM9DS1_WHO_AM_I, 1, &result, "Failed to read LSM9DS1 accel/gyro id"))
        return false;

    if (result != LSM9DS1_ID) {
        HAL_ERROR1("Incorrect LSM9DS1 gyro id %d\n", result);
        return false;
    }

    if (!setGyroSampleRate())
        return false;

    if (!setGyroCTRL2())
        return false;

    if (!setGyroCTRL3())
        return false;

    if (!setGyroCTRL4())
        return false;

    if (!setAccelCTRL6())
        return false;

    if (!setAccelCTRL7())
        return false;

    if (!setCtrl9())
        return false;

    if (!setFifoMode())
        return false;

    //  Set up the mag

    if (!m_settings->HALRead(m_magSlaveAddr, LSM9DS1_MAG_WHO_AM_I, 1, &result, "Failed to read LSM9DS1 accel/mag id"))
        return false;

    if (result != LSM9DS1_MAG_ID) {
        HAL_ERROR1("Incorrect LSM9DS1 accel/mag id %d\n", result);
        return false;
    }

    if (!setCompassCTRL1())
        return false;

    if (!setCompassCTRL2())
        return false;

    if (!setCompassCTRL3())
        return false;

    if (!setCompassCTRL4())
        return false;

    if (!setCompassCTRL5())
        return false;

    gyroBiasInit();

    initializeFifoBuffer();


    // set validity flags

    m_imuData.fusionPoseValid  = false;
    m_imuData.fusionQPoseValid = false;
    m_imuData.gyroValid        = m_settings->m_LSM9DS1GyroSampleRate > LSM9DS1_GYRO_SAMPLERATE_OFF;
    m_imuData.accelValid       = m_settings->m_LSM9DS1AccelSampleRate > LSM9DS1_ACCEL_SAMPLERATE_OFF;
    m_imuData.compassValid     = m_settings->m_LSM9DS1CompassSampleRate > LSM9DS1_COMPASS_SAMPLERATE_OFF;
    m_imuData.pressureValid    = false;
    m_imuData.temperatureValid = false;
    m_imuData.humidityValid    = false;
    m_imuData.pressure         = 0;
    m_imuData.temperature      = 0;



    HAL_INFO("LSM9DS1 init complete\n");
    return true;
}


bool RTIMULSM9DS1::setGyroSampleRate()
{

    m_settings->m_LSM9DS1GyroSampleRate = checkSettingsValue(
        m_settings->m_LSM9DS1GyroSampleRate,
        LSM9DS1_GYRO_SAMPLERATE_OFF,
        LSM9DS1_GYRO_SAMPLERATE_952,
        LSM9DS1_GYRO_SAMPLERATE_119,
        "LSM9DS1 gyro sample rate"
    );

    // set sample rate based on gyro setting unless gyro is disabled
    if (m_settings->m_LSM9DS1GyroSampleRate > LSM9DS1_GYRO_SAMPLERATE_OFF) {
        // otherwise, samplerate is set by accel    
        if (m_settings->m_LSM9DS1GyroSampleRate > LSM9DS1_GYRO_SAMPLERATE_14_9)
            m_sampleInterval = 1e6 / M_SAMPLERATE_SCALAR * (1 << (LSM9DS1_GYRO_SAMPLERATE_952 - m_settings->m_LSM9DS1GyroSampleRate));
        else
            m_sampleInterval = 2e6 / M_SAMPLERATE_SCALAR * (1 << (LSM9DS1_GYRO_SAMPLERATE_952 - m_settings->m_LSM9DS1GyroSampleRate));

        m_sampleRate = 1e6 / m_sampleInterval;
    }

    m_fusion->setGyroEnable(m_settings->m_LSM9DS1GyroSampleRate != LSM9DS1_GYRO_SAMPLERATE_OFF);


    m_settings->m_LSM9DS1GyroBW = checkSettingsValue(
        m_settings->m_LSM9DS1GyroBW,
        LSM9DS1_GYRO_BANDWIDTH_0,
        LSM9DS1_GYRO_BANDWIDTH_3,
        LSM9DS1_GYRO_BANDWIDTH_1,
        "LSM9DS1 gyro bandwidth"
    );

    m_settings->m_LSM9DS1GyroFsr = checkSettingsValue(
        m_settings->m_LSM9DS1GyroFsr,
        LSM9DS1_GYRO_FSR_245,
        LSM9DS1_GYRO_FSR_2000,
        LSM9DS1_GYRO_FSR_245,
        "LSM9DS1 gyro FSR"
    );
    // extra check for in between invalid value
    m_settings->m_LSM9DS1GyroFsr = (m_settings->m_LSM9DS1GyroFsr == 2) ? LSM9DS1_GYRO_FSR_245 : m_settings->m_LSM9DS1GyroFsr;
    m_gyroScale = M_GYRO_SCALE_LOOKUP[m_settings->m_LSM9DS1GyroFsr] * RTMATH_DEGREE_TO_RAD;

    unsigned char ctrl1 = (m_settings->m_LSM9DS1GyroSampleRate << 5) | (m_settings->m_LSM9DS1GyroFsr << 3) | m_settings->m_LSM9DS1GyroBW;

    return (m_settings->HALWrite(m_accelGyroSlaveAddr, LSM9DS1_CTRL1, ctrl1, "Failed to set LSM9DS1 gyro CTRL1"));
}

bool RTIMULSM9DS1::setGyroCTRL2()
{
    m_settings->m_LSM9DS1GyroBW = checkSettingsValue(
        m_settings->m_LSM9DS1GyroBW,
        LSM9DS1_GYRO_BANDWIDTH_OFF,
        LSM9DS1_GYRO_BANDWIDTH_3,
        LSM9DS1_GYRO_BANDWIDTH_1,
        "LSM9DS1 gyro bandwidth"
    );

    m_settings->m_LSM9DS1GyroHpf = checkSettingsValue(
        m_settings->m_LSM9DS1GyroHpf,
        LSM9DS1_GYRO_HPF_OFF,
        LSM9DS1_GYRO_HPF_9,
        LSM9DS1_GYRO_HPF_0,
        "LSM9DS1 gyro high pass filter"
    );
    // configure filter selection for gyro
    // 00 - LPF 1
    // 01 - LPF 1 + HPF
    // 11 - LPF 1 + HPF + LPF2

    unsigned char ctrl2 = 0;

    ctrl2 |= (m_settings->m_LSM9DS1GyroBW > LSM9DS1_GYRO_BANDWIDTH_OFF) ? 2 : 0;
    ctrl2 |= (m_settings->m_LSM9DS1GyroHpf > LSM9DS1_GYRO_HPF_OFF) ? 1 : 0;

    return (m_settings->HALWrite(m_accelGyroSlaveAddr, LSM9DS1_CTRL2, ctrl2, "Failed to set LSM9DS1 gyro CTRL2"));;
}

bool RTIMULSM9DS1::setGyroCTRL3()
{
    unsigned char ctrl3 = 0;

    m_settings->m_LSM9DS1GyroHpf = checkSettingsValue(
        m_settings->m_LSM9DS1GyroHpf,
        LSM9DS1_GYRO_HPF_OFF,
        LSM9DS1_GYRO_HPF_9,
        LSM9DS1_GYRO_HPF_0,
        "LSM9DS1 gyro high pass filter"
    );
    if (m_settings->m_LSM9DS1GyroHpf > LSM9DS1_GYRO_HPF_OFF)
        ctrl3 = (1 << 6) | m_settings->m_LSM9DS1GyroHpf;

    // set gyro to low power mode if sample rate is low enough per the manual
    if (m_settings->m_LSM9DS1GyroSampleRate <= LSM9DS1_GYRO_SAMPLERATE_119 && m_settings->m_LSM9DS1GyroBW <= LSM9DS1_GYRO_SAMPLERATE_14_9)
        ctrl3 |= (1 << 7);

    return m_settings->HALWrite(m_accelGyroSlaveAddr, LSM9DS1_CTRL3, ctrl3, "Failed to set LSM9DS1 gyro CTRL3");
}

bool RTIMULSM9DS1::setGyroCTRL4()
{
    unsigned char ctrl4 = 0;
    m_settings->m_LSM9DS1GyroSampleRate = checkSettingsValue(
        m_settings->m_LSM9DS1GyroSampleRate,
        LSM9DS1_GYRO_SAMPLERATE_OFF,
        LSM9DS1_GYRO_SAMPLERATE_952,
        LSM9DS1_GYRO_SAMPLERATE_119,
        "LSM9DS1 gyro sample rate"
    );
    if (m_settings->m_LSM9DS1GyroSampleRate == LSM9DS1_GYRO_SAMPLERATE_OFF)
        ctrl4 = 0;
    else
        // disable gyros output if configured off ([X|Y|Z]en_G = 1)
        ctrl4 = (7 << 3);

    return m_settings->HALWrite(m_accelGyroSlaveAddr, LSM9DS1_CTRL4, ctrl4, "Failed to set LSM9DS1 gyro CTRL4");
}

bool RTIMULSM9DS1::setAccelCTRL5()
{
    unsigned char ctrl5 = 0;

    m_settings->m_LSM9DS1AccelSampleRate = checkSettingsValue(
        m_settings->m_LSM9DS1AccelSampleRate,
        LSM9DS1_ACCEL_SAMPLERATE_OFF,
        LSM9DS1_ACCEL_SAMPLERATE_952,
        LSM9DS1_ACCEL_SAMPLERATE_119,
        "LSM9DS1 accel sample rate"
    );
    if (m_settings->m_LSM9DS1AccelSampleRate == LSM9DS1_ACCEL_SAMPLERATE_OFF)
        ctrl5 = 0;
    else
        // disable accels output if configured off ([X|Y|Z]en_XL = 1)
        ctrl5 = 0x1C;
    return m_settings->HALWrite(m_accelGyroSlaveAddr, LSM9DS1_CTRL5, ctrl5, "Failed to set LSM9DS1 accel CTRL5");
}

bool RTIMULSM9DS1::setAccelCTRL6()
{
    unsigned char ctrl6 = 0;

    m_settings->m_LSM9DS1AccelSampleRate = checkSettingsValue(
        m_settings->m_LSM9DS1AccelSampleRate,
        LSM9DS1_ACCEL_SAMPLERATE_OFF,
        LSM9DS1_ACCEL_SAMPLERATE_952,
        LSM9DS1_ACCEL_SAMPLERATE_119,
        "LSM9DS1 accel sample rate"
    );
    // if the gyro is off, set the sample rate via accel setting
    if (m_settings->m_LSM9DS1GyroSampleRate == LSM9DS1_GYRO_SAMPLERATE_OFF) {
        if (m_settings->m_LSM9DS1AccelSampleRate > LSM9DS1_ACCEL_SAMPLERATE_50)
            m_sampleInterval = 1e6 / (M_SAMPLERATE_SCALAR / (1 << (LSM9DS1_ACCEL_SAMPLERATE_952 - m_settings->m_LSM9DS1AccelSampleRate)));
        else if (m_settings->m_LSM9DS1AccelSampleRate == LSM9DS1_ACCEL_SAMPLERATE_50)
            m_sampleInterval = 1e6 / 50;
        else
            m_sampleInterval = 1e6 / 10;

        m_settings->m_LSM9DS1AccelSampleRate = m_settings->m_LSM9DS1GyroSampleRate;
        m_sampleRate = 1e6 / m_sampleInterval;
    }
    // enable accel in fusion calcs
    m_fusion->setAccelEnable(m_settings->m_LSM9DS1AccelSampleRate != LSM9DS1_ACCEL_SAMPLERATE_OFF);


    // having this HPF on seems to remove gravity(?) from the accel data, although it's not well documented.
    // Not a good idea to have it on since the attitude is heavily based on accels sensing gravity.
    // note, this is enabled via ctrl7
    m_settings->m_LSM9DS1AccelLpf = checkSettingsValue(
        m_settings->m_LSM9DS1AccelLpf,
        LSM9DS1_ACCEL_LPF_OFF,
        LSM9DS1_ACCEL_LPF_50,
        LSM9DS1_ACCEL_LPF_ODRBASED,
        "LSM9DS1 accel low pass filter"
    );
    if (m_settings->m_LSM9DS1AccelLpf == LSM9DS1_ACCEL_LPF_ODRBASED)
        ctrl6 = 4;
    else if (m_settings->m_LSM9DS1AccelLpf != LSM9DS1_ACCEL_LPF_OFF)
        ctrl6 = m_settings->m_LSM9DS1AccelLpf;

    m_settings->m_LSM9DS1AccelFsr = checkSettingsValue(
        m_settings->m_LSM9DS1AccelFsr,
        LSM9DS1_ACCEL_FSR_2,
        LSM9DS1_ACCEL_FSR_8,
        LSM9DS1_ACCEL_FSR_2,
        "LSM9DS1 accel FSR"
    );

    m_accelScale = M_ACCEL_SCALE_LOOKUP[m_settings->m_LSM9DS1AccelFsr];

    ctrl6 |= (m_settings->m_LSM9DS1AccelSampleRate << 5) | (m_settings->m_LSM9DS1AccelFsr << 3);

    return m_settings->HALWrite(m_accelGyroSlaveAddr, LSM9DS1_CTRL6, ctrl6, "Failed to set LSM9DS1 accel CTRL6");
}

bool RTIMULSM9DS1::setAccelCTRL7()
{
    m_settings->m_LSM9DS1AccelLpf2 = checkSettingsValue(
        m_settings->m_LSM9DS1AccelLpf2,
        LSM9DS1_ACCEL_HR_LPF2_OFF,
        LSM9DS1_ACCEL_HR_LPF2_400,
        LSM9DS1_ACCEL_HR_LPF2_OFF,
        "LSM9DS1 accel high pass filter"
    );

    unsigned char ctrl7 = 0;

    // see the lsm9ds1 manual for specifics on how the filtering works
    if (m_settings->m_LSM9DS1AccelLpf2 > LSM9DS1_ACCEL_HR_LPF2_OFF)
        ctrl7 |= (1 << 7) | (m_settings->m_LSM9DS1AccelLpf2 << 5);
    else if (m_settings->m_LSM9DS1AccelLpf > LSM9DS1_ACCEL_LPF_OFF)
        ctrl7 |= (1 << 2);

    return m_settings->HALWrite(m_accelGyroSlaveAddr, LSM9DS1_CTRL7, ctrl7, "Failed to set LSM9DS1 accel CTRL7");
}


bool RTIMULSM9DS1::setCtrl9()
{
    m_settings->m_LSM9DS1FifoMode = checkSettingsValue(
        m_settings->m_LSM9DS1FifoMode,
        LSM9DS1_FIFO_BYPASS_MODE,
        LSM9DS1_FIFO_CONTINUOUS_MODE,
        LSM9DS1_FIFO_CONTINUOUS_MODE,
        "LSM9DS1 FIFO mode"
    );

    if (m_settings->m_LSM9DS1FifoMode == LSM9DS1_FIFO_BYPASS_MODE)
        // if in bypass mode, set to 0
        return m_settings->HALWrite(m_accelGyroSlaveAddr, LSM9DS1_CTRL9, 0x00, "Failed to set LSM9DS1 CTRL9");
    else
        // fifo_en = 1 and STOP_ON_FTH = 1 (enables fifo and stops on threshold set in fifo_ctrl)
        return m_settings->HALWrite(m_accelGyroSlaveAddr, LSM9DS1_CTRL9, 0x03, "Failed to set LSM9DS1 CTRL9");
}


bool RTIMULSM9DS1::setFifoMode()
{
    m_settings->m_LSM9DS1FifoMode = checkSettingsValue(
        m_settings->m_LSM9DS1FifoMode,
        LSM9DS1_FIFO_BYPASS_MODE,
        LSM9DS1_FIFO_CONTINUOUS_MODE,
        LSM9DS1_FIFO_CONTINUOUS_MODE,
        "LSM9DS1 FIFO mode"
    );

    m_settings->m_LSM9DS1PollMode = checkSettingsValue(
        m_settings->m_LSM9DS1PollMode,
        LSM9DS1_FAST_POLL_MODE_OFF,
        LSM9DS1_FIFO_CACHE_MODE,
        LSM9DS1_FAST_POLL_MODE_OFF,
        "LSM9DS1 fast poll mode"
    );

    // sets FMODE bits
    unsigned char fifoCtrl = (m_settings->m_LSM9DS1FifoMode << 5);
    if (m_settings->m_LSM9DS1FifoMode > LSM9DS1_FIFO_BYPASS_MODE)
        // sets FTH bits
        fifoCtrl |= (M_IMU_FIFO_SIZE_MAX - 1);                      //FIFO depth is limited to FIFO_CTRL (2Eh)(FTH [4:0]) + 1 data.

    return m_settings->HALWrite(m_accelGyroSlaveAddr, LSM9DS1_FIFO_CTRL, fifoCtrl, "Failed to set LSM9DS1 FIFO mode");
}


bool RTIMULSM9DS1::setCompassCTRL1()
{
    unsigned char ctrl1;

    m_settings->m_LSM9DS1CompassSampleRate = checkSettingsValue(
        m_settings->m_LSM9DS1CompassSampleRate,
        LSM9DS1_COMPASS_SAMPLERATE_OFF,
        LSM9DS1_COMPASS_SAMPLERATE_FAST_ODR,
        LSM9DS1_COMPASS_SAMPLERATE_80,
        "LSM9DS1 compass sample rate"
    );
    m_settings->m_LSM9DS1CompassOpMode = checkSettingsValue(
        m_settings->m_LSM9DS1CompassOpMode,
        LSM9DS1_COMPASS_LOW_POWER_MODE,
        LSM9DS1_COMPASS_ULTRA_HIGH_PERFORMANCE_MODE,
        LSM9DS1_COMPASS_MEDIUM_PERFORMANCE_MODE,
        "LSM9DS1 compass operation mode"
    );
    // set up fast odr if need be
    if (m_settings->m_LSM9DS1CompassSampleRate == LSM9DS1_COMPASS_SAMPLERATE_FAST_ODR) {
        m_settings->m_LSM9DS1CompassSampleRate = LSM9DS1_COMPASS_SAMPLERATE_80;
        ctrl1 = 0x03;
    }
    m_fusion->setCompassEnable(m_settings->m_LSM9DS1CompassSampleRate != LSM9DS1_COMPASS_SAMPLERATE_OFF);

    ctrl1 = (m_settings->m_LSM9DS1CompassSampleRate << 2) | (m_settings->m_LSM9DS1CompassOpMode << 5) | (1 << 7);
    m_magODRInterval = 1e6 / M_MAG_SAMPLERATE_SCALAR * (1 << (LSM9DS1_COMPASS_SAMPLERATE_80 - m_settings->m_LSM9DS1CompassSampleRate));

    return m_settings->HALWrite(m_magSlaveAddr, LSM9DS1_MAG_CTRL1, ctrl1, "Failed to set LSM9DS1 compass CTRL5");
}

bool RTIMULSM9DS1::setCompassCTRL2()
{
    unsigned char ctrl2;

    //  convert FSR to uT
    m_settings->m_LSM9DS1CompassFsr = checkSettingsValue(
        m_settings->m_LSM9DS1CompassFsr,
        LSM9DS1_COMPASS_FSR_4,
        LSM9DS1_COMPASS_FSR_16,
        LSM9DS1_COMPASS_FSR_4,
        "LSM9DS1 compass FSR"
    );

    m_compassScale = M_COMPASS_SCALE_LOOKUP[m_settings->m_LSM9DS1CompassFsr] * RTMATH_GUASS_TO_MICROTESLA;

    ctrl2 = (m_settings->m_LSM9DS1CompassFsr << 5);
    return m_settings->HALWrite(m_magSlaveAddr, LSM9DS1_MAG_CTRL2, ctrl2, "Failed to set LSM9DS1 compass CTRL6");
}

bool RTIMULSM9DS1::setCompassCTRL3()
{
    return m_settings->HALWrite(m_magSlaveAddr, LSM9DS1_MAG_CTRL3, 0x00, "Failed to set LSM9DS1 compass CTRL3");
}

bool RTIMULSM9DS1::setCompassCTRL4()
{
    m_settings->m_LSM9DS1CompassOpMode = checkSettingsValue(
        m_settings->m_LSM9DS1CompassOpMode,
        LSM9DS1_COMPASS_LOW_POWER_MODE,
        LSM9DS1_COMPASS_ULTRA_HIGH_PERFORMANCE_MODE,
        LSM9DS1_COMPASS_MEDIUM_PERFORMANCE_MODE,
        "LSM9DS1 compass operation mode"
    );
    unsigned char ctrl4 = (m_settings->m_LSM9DS1CompassOpMode << 2);
    return m_settings->HALWrite(m_magSlaveAddr, LSM9DS1_MAG_CTRL4, ctrl4, "Failed to set LSM9DS1 compass CTRL4");
}



bool RTIMULSM9DS1::setCompassCTRL5()
{
// setup BDU
    return m_settings->HALWrite(m_magSlaveAddr, LSM9DS1_MAG_CTRL5, (1 << 6), "Failed to set LSM9DS1 compass CTRL5");
}

bool RTIMULSM9DS1::initializeFifoBuffer()
{
    m_imuSampleSize = M_SENSOR_3_AXIS_BYTE_SIZE;
    m_imuCacheSize = 1;    // default to 1
    m_imuReadPtr = 0;

    if (m_settings->m_LSM9DS1GyroSampleRate > LSM9DS1_GYRO_SAMPLERATE_OFF)
        m_imuSampleSize *= 2;

    if (m_settings->m_LSM9DS1PollMode == LSM9DS1_FIFO_CACHE_MODE)
        m_imuCacheSize = M_IMU_FIFO_SIZE_MAX;

    else if (m_settings->m_LSM9DS1PollMode == LSM9DS1_FAST_POLL_MODE_OFF) {
        m_imuSampleSize += 1;
        m_imuReadPtr = 1;
    }

    m_fifoBuff.buildCache(m_imuSampleSize, m_imuCacheSize, 16);  // initialize fifo cache

    return true;
}

int RTIMULSM9DS1::IMUGetPollInterval()
{
    return  (RTMath::currentUSecsSinceEpoch() > m_sampleInterval + m_imuData.timestamp) ? 0 : (m_sampleInterval + m_imuData.timestamp - RTMath::currentUSecsSinceEpoch());
}

bool RTIMULSM9DS1::IMURead()
{
    unsigned char compassData[7];   //1 byte for status 6, bytes for mag
    unsigned char sampleCount = 1;
    unsigned char readLocation = 0;

    uint64_t now = RTMath::currentUSecsSinceEpoch();

    bool isCompassSampleRateOn = m_settings->m_LSM9DS1CompassSampleRate != LSM9DS1_COMPASS_SAMPLERATE_OFF;
    bool isTimeForMagRead = now >= m_magODRInterval + m_lastMagRead;
    bool isFifoSampleSizeSufficient = m_settings->m_LSM9DS1PollMode != LSM9DS1_FIFO_CACHE_MODE || m_fifoBuff.getSampleSize() > 1;
    bool isNotFifoCacheMode = m_settings->m_LSM9DS1PollMode != LSM9DS1_FIFO_CACHE_MODE;
    bool isTimeForCacheRead = m_fifoBuff.frontTimeStampBase() + m_sampleInterval * M_IMU_FIFO_SIZE_MAX <= now;


    bool magRead = isCompassSampleRateOn && isTimeForMagRead && isFifoSampleSizeSufficient;
    bool cacheRead = isNotFifoCacheMode || (!magRead && isTimeForCacheRead);
        // boot out if no read available in timed poll mode
    if (m_settings->m_LSM9DS1PollMode == LSM9DS1_TIMED_POLL_MODE && m_imuData.timestamp + m_sampleInterval > now)
        return false;

    if (cacheRead) {

        // read fifo src if doing any sort of caching
        if (m_settings->m_LSM9DS1PollMode == LSM9DS1_FIFO_CACHE_MODE) {
            if (!m_settings->HALRead(m_accelGyroSlaveAddr, LSM9DS1_FIFO_SRC, 1, &sampleCount, "Failed to read LSM9DS1 FIFO status"))
                return false;

            if ((sampleCount &= 0x3f) == 0)
                return false;
        }

        unsigned short readsize = sampleCount * m_imuSampleSize;
        unsigned char *buffer = m_fifoBuff.readIn(readsize);

        if (m_settings->m_LSM9DS1GyroSampleRate > LSM9DS1_GYRO_SAMPLERATE_OFF)
            // accel only read
            readLocation = LSM9DS1_STATUS + !m_imuReadPtr;
        else
            // accel & gyro read
            readLocation = LSM9DS1_STATUS2 + !m_imuReadPtr;

        if (!m_settings->HALRead(m_accelGyroSlaveAddr, readLocation, readsize, buffer, "Failed to read LSM9DS1 FIFO data"))
            return false;
    }

    if (magRead) {
        readLocation = LSM9DS1_MAG_STATUS + !m_imuReadPtr;
        unsigned short readsize = sizeof(compassData) - !m_imuReadPtr;
        if (m_settings->HALRead(m_magSlaveAddr, readLocation, readsize, &compassData[!m_imuReadPtr], "Failed to read LSM9DS1 compass data")) {

            if (!m_settings->m_LSM9DS1PollMode)
                m_imuData.compassValid = compassData[0] & 0x08;

            if (m_imuData.compassValid){
                RTMath::convertToVector(&compassData[1], m_imuData.compass, m_compassScale, false);

                //  sort out compass axes
                m_imuData.compass.setX(-m_imuData.compass.x());
                m_imuData.compass.setZ(-m_imuData.compass.z());

                if (!m_lastMagRead)
                    m_compassAverage = m_imuData.compass;
                m_lastMagRead = now;

                calibrateAverageCompass();
            }
        }
    }

    // false means there's no imu data in the cache :(
    if (!processCache(magRead))
        return false;
    processIMUData();

    return true;
}



void RTIMULSM9DS1::processIMUData()
{

//  sort out gyro axes and correct for bias
    m_imuData.gyro.setZ(-m_imuData.gyro.z());

    //  sort out accel data;

    m_imuData.accel.setX(-m_imuData.accel.x());
    m_imuData.accel.setY(-m_imuData.accel.y());

    //  now do standard processing

    handleGyroBias();

    calibrateAccel();

    //  now update the filter

    updateFusion();
}

bool RTIMULSM9DS1::processCache(bool magRead)
{
    if (m_fifoBuff.isempty())
        return false;
        
    RTVector3 accelTemp(0, 0, 0);
    RTVector3 gyroTemp(0, 0, 0);
    RTVector3 accelSum(0, 0, 0);
    RTVector3 gyroSum(0, 0, 0);
    RTFLOAT DTsum = 0;
    RTFLOAT dt;
    bool readOnce = true;
    // take weighted average of new data    
    while (m_fifoBuff.getSampleSize() > 1 || readOnce) {

        if (m_imuReadPtr) {
            m_imuData.gyroValid = *(m_fifoBuff.front()) & 0x2;
            m_imuData.accelValid = *(m_fifoBuff.front()) & 0x1;
        }

        dt = (m_fifoBuff.frontTimeStamp(m_sampleInterval) - m_imuData.timestamp)/1e6;
        if (m_settings->m_LSM9DS1GyroSampleRate > LSM9DS1_GYRO_SAMPLERATE_OFF) {
            RTMath::convertToVector(m_fifoBuff.front() + m_imuReadPtr, gyroTemp, m_gyroScale, false);
            RTMath::convertToVector(m_fifoBuff.front() + m_imuReadPtr + M_SENSOR_3_AXIS_BYTE_SIZE, accelTemp, m_accelScale, false);
        } else
            RTMath::convertToVector(m_fifoBuff.front() + m_imuReadPtr, accelTemp, m_accelScale, false);

        DTsum += dt;
        accelSum += (accelTemp * dt);
        gyroSum += (gyroTemp * dt);

        m_fifoBuff.pop();
        
        if (!(m_imuData.gyroValid || m_imuData.accelValid))
            return false;

        readOnce = false;
    }

    
    accelSum /= DTsum;
    gyroSum /= DTsum;

    m_imuData.accel = accelSum;
    m_imuData.gyro = gyroSum;

    m_imuData.timestamp = m_fifoBuff.frontTimeStamp(m_sampleInterval);

    return true;

}


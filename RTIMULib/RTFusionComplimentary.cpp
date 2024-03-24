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



#include "RTFusionComplimentary.h"



RTFusionComplimentary::RTFusionComplimentary()
{
    m_gyroAlphaThreshold = M_GYRO_FILTER_THRESHOLD;
    m_accelAlphaThreshold = M_ACCEL_FILTER_THRESHOLD;
}


void RTFusionComplimentary::reset()
{}

void RTFusionComplimentary::predict()
{
    if (!m_enableGyro)
        return;

    RTFLOAT x2, y2, z2;
    RTFLOAT qs, qx, qy, qz;

    qs = m_stateQ.scalar();
    qx = m_stateQ.x();
    qy = m_stateQ.y();
    qz = m_stateQ.z();

    x2 = m_gyro.x() / (RTFLOAT)2.0;
    y2 = m_gyro.y() / (RTFLOAT)2.0;
    z2 = m_gyro.z() / (RTFLOAT)2.0;

    // Predict new state

    m_stateQ.setScalar(qs + (-x2 * qx - y2 * qy - z2 * qz) * m_timeDelta);
    m_stateQ.setX(qx + (x2 * qs + z2 * qy - y2 * qz) * m_timeDelta);
    m_stateQ.setY(qy + (y2 * qs - z2 * qx + x2 * qz) * m_timeDelta);
    m_stateQ.setZ(qz + (z2 * qs + y2 * qx - x2 * qy) * m_timeDelta);
    m_stateQ.normalize();
}

void RTFusionComplimentary::update()
{
    if (m_enableAccel) {
        m_filterAlpha = (m_accel.length() - 1.);
        m_filterAlpha = m_filterAlpha < 0 ? -1.0 * m_filterAlpha : m_filterAlpha;

        if (m_filterAlpha < m_accelAlphaThreshold)
            m_filterAlpha = 0.;
        else if (m_filterAlpha > m_gyroAlphaThreshold)
            m_filterAlpha = 1.;
        else
            m_filterAlpha = 1 / (m_gyroAlphaThreshold - m_accelAlphaThreshold) * (m_filterAlpha - m_accelAlphaThreshold);

        // copy predicted state to fusion pose
        m_stateQ.toEuler(m_fusionPose);

        // interpolate the euler angles between fusion pose & measured pose based on the adaptive gain.
        for (int i = 0; i < 3; i++)
            m_fusionPose.setData(i, m_measuredPose.data(i) * (1. - m_filterAlpha) + m_fusionPose.data(i) * m_filterAlpha);

        m_fusionQPose.fromEuler(m_fusionPose);
    }
}

void RTFusionComplimentary::newIMUData(RTIMU_DATA &data, const RTIMUSettings *settings)
{
    if (m_debug) {
        HAL_INFO("\n------\n");
        HAL_INFO2("IMU update delta time: %f, sample %d\n", m_timeDelta, m_sampleNumber++);
    }
    m_sampleNumber++;

    if (m_enableGyro)
        m_gyro = data.gyro;
    else
        m_gyro = RTVector3();
    m_accel = data.accel;
    m_compass = data.compass;
    m_compassValid = data.compassValid;

    if (m_firstTime) {
        m_lastFusionTime = data.timestamp;
        calculatePose(m_accel, m_compass, settings->m_compassAdjDeclination);

        //  initialize the poses

        m_stateQ.fromEuler(m_measuredPose);
        m_fusionQPose = m_stateQ;
        m_fusionPose = m_measuredPose;
        m_firstTime = false;

    } else {
        m_timeDelta = (RTFLOAT)(data.timestamp - m_lastFusionTime) / (RTFLOAT)1000000;
        m_lastFusionTime = data.timestamp;
        if (m_timeDelta <= 0)
            return;

        predict();
        // measure
        calculatePose(m_accel, m_compass, settings->m_compassAdjDeclination);

        update();

        data.fusionPose = m_fusionPose;
        data.fusionQPose = m_fusionQPose;

    }

}

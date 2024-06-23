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



RTFusionComplimentary::RTFusionComplimentary() :
    m_accelFIRFilter(M_ACCEL_FIR_ORDER),
    m_alphaIIRFilter(M_ALPHA_IIR_ORDER)
{
    // default value for now
    m_slerpPower = M_ACCEL_FILTER_THRESHOLD;

    const RTFLOAT polesFIR[M_ACCEL_FIR_ORDER] = { 1,1,1,1,1,1,1,1,1,1 };
    const RTFLOAT polesIIR[M_ALPHA_IIR_ORDER] = { 1,2,3,4,5 };
    
    m_accelFIRFilter.setPoles(polesFIR);
    m_alphaIIRFilter.setPoles(polesIIR);
}


void RTFusionComplimentary::reset()
{
    m_accelFIRFilter.zeros();
    m_alphaIIRFilter.zeros();
}

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
        
        // deterine alpha value used in slerp
        RTFLOAT accelNorm = fabs(m_accel.length() - 1);
        RTFLOAT lastOutput = m_alphaIIRFilter.outputFIR(false);
        m_accelFIRFilter.add(accelNorm);

        bool bypassIR = accelNorm > lastOutput;
        RTFLOAT accelMag = m_accelFIRFilter.outputFIR(bypassIR);
        RTFLOAT alpha = accelMag / m_slerpPower;                   // slerp power can be set in imu drivers
        
        alpha = alpha > 1 ? 1 : alpha;
        alpha = m_alphaIIRFilter.outputIIR(alpha, bypassIR);
        alpha = alpha > 1 ? 1 : alpha;
        
        // slerp gyro solution with accel solution
        m_fusionQPose = RTQuaternion::slerp(m_stateQ, m_measuredQPose, alpha);
    }
    else {
        m_fusionQPose = m_stateQ;
    }
    m_fusionQPose.toEuler(m_fusionPose);

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

RTFusionComplimentary::IRFilter::IRFilter(uint8_t order) :
    m_head(order - 1),
    m_order(order)
{
    m_data = new RTFLOAT[m_order];
    m_poles = new RTFLOAT[m_order];
    zeros();
}

RTFusionComplimentary::IRFilter::~IRFilter()
{
    delete[] m_data;
    delete[] m_poles;
}

void RTFusionComplimentary::IRFilter::add(RTFLOAT data)
{
    m_head = (m_head + 1) % m_order;
    m_data[m_head] = data;

    m_sampleCount = m_sampleCount < m_order ? m_sampleCount + 1 : m_order;
}

void RTFusionComplimentary::IRFilter::setPoles(const RTFLOAT *poles)
{
    RTFLOAT poleSum = 0;
    for (int i = 0; i < m_order; i++)
        poleSum += poles[i];

    for (int i = 0; i < m_order; i++)
        m_poles[i] = poles[i] / poleSum;
}

RTFLOAT RTFusionComplimentary::IRFilter::outputFIR(bool bypass)
{
    if (m_sampleCount < m_order || bypass)
        return m_data[m_head];

    RTFLOAT sum = 0;
    for (int i = 0; i < m_order; i++)
        sum += m_data[(m_head + i + 1) % m_order] * m_poles[m_order - 1 - i];
    
    return sum;
}

RTFLOAT RTFusionComplimentary::IRFilter::outputIIR(RTFLOAT data, bool bypass)
{
    if (m_sampleCount < m_order || bypass){
        add(data);
        return m_data[m_head];
    }
    RTFLOAT sum = 0;
    for (int i = 0; i < m_order - 1; i++)
        sum += m_data[(m_head + i + 2) % m_order] * m_poles[m_order - 1 - i];

    add(sum + data * m_poles[0]);
    return m_data[m_head];
}

void RTFusionComplimentary::IRFilter::zeros()
{
    m_sampleCount = 0;
    for (int i = 0; i < m_order; i++)
        m_data[i] = 0;
}
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



#ifndef _RTCOMPLIMENTARY_H_
#define _RTCOMPLIMENTARY_H_

#include "RTFusion.h"
#include "RTIMUSettings.h"

class RTFusionComplimentary : public RTFusion{
public:
    RTFusionComplimentary();
    // ~RTFusionComplimentary();

    int fusionType() { return RTFUSION_TYPE_COMPLIMENTARY; }
    void reset();
    void newIMUData(RTIMU_DATA& data, const RTIMUSettings *settings);

    inline void setGyroAlphaThreshold(RTFLOAT t1) { m_gyroAlphaThreshold = t1; }
    inline void setAccelAlphaThreshold(RTFLOAT t2) { m_accelAlphaThreshold = t2; }
    
private:
    static constexpr RTFLOAT M_ACCEL_FILTER_THRESHOLD = 0.2;
    static constexpr RTFLOAT M_GYRO_FILTER_THRESHOLD = 0.3;

    void predict();
    void update();

    RTFLOAT m_filterAlpha;
    RTFLOAT m_gyroAlphaThreshold;
    RTFLOAT m_accelAlphaThreshold;

    RTVector3 m_gyro;										// unbiased gyro data
    RTFLOAT m_timeDelta;                                    // time between predictions

    RTQuaternion m_stateQ;									// quaternion state vector

    int m_sampleNumber;                                     // number of samples accumulated

};


#endif // _RTCOMPLIMENTARY_H_
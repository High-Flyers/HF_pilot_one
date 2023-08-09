//=============================================================================================
// MadgwickAHRS.h
//=============================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
//
// From the x-io website "Open-source resources available on this website are
// provided under the GNU General Public Licence unless an alternative licence
// is provided in source."
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=============================================================================================
#ifndef MadgwickAHRS_h
#define MadgwickAHRS_h
#include <math.h>

//--------------------------------------------------------------------------------------------
// Variable declaration
class Madgwick
{
private:
    inline float invSqrt(float x);
    inline void quatMult(float q10, float q11, float q12, float q13, float q20, float q21, float q22, float q23, float &qr0, float &qr1, float &qr2, float &qr3);
    float beta; // algorithm gain
    float q0;
    float q1;
    float q2; // quaternion is in w x y z format
    float q3; // quaternion of sensor frame relative to auxiliary frame
    float invSampleFreq;
    float roll;
    float pitch;
    float yaw;
    char anglesComputed;
    void computeAngles();

    //-------------------------------------------------------------------------------------------
    // Function declarations
public:
    Madgwick(void);
    void begin(float sampleFrequency) { invSampleFreq = 1.0f / sampleFrequency; }
    // gyro should be in rad/s
    void update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
    void updateIMU(float gx, float gy, float gz, float ax, float ay, float az);
    // float getPitch(){return atan2f(2.0f * q2 * q3 - 2.0f * q0 * q1, 2.0f * q0 * q0 + 2.0f * q3 * q3 - 1.0f);};
    // float getRoll(){return -1.0f * asinf(2.0f * q1 * q3 + 2.0f * q0 * q2);};
    // float getYaw(){return atan2f(2.0f * q1 * q2 - 2.0f * q0 * q3, 2.0f * q0 * q0 + 2.0f * q1 * q1 - 1.0f);};
    void computeAnglesFrame(float qf0, float qf1, float qf2, float qf3);
    float getRoll()
    {
        if (!anglesComputed)
            computeAngles();
        return roll * 57.29578f;
    }
    float getPitch()
    {
        if (!anglesComputed)
            computeAngles();
        return pitch * 57.29578f;
    }
    float getYaw()
    {
        if (!anglesComputed)
            computeAngles();
        return yaw * 57.29578f + 180.0f;
    }
    float getRollRadians()
    {
        if (!anglesComputed)
            computeAngles();
        return roll;
    }
    float getPitchRadians()
    {
        if (!anglesComputed)
            computeAngles();
        return pitch;
    }
    float getYawRadians()
    {
        if (!anglesComputed)
            computeAngles();
        return yaw;
    }
};
#endif

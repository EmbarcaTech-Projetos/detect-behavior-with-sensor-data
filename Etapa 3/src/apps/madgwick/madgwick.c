#include <math.h>
#include "madgwick.h"

// System constants
#define deltat 0.2f 

// Madgwick filter parameters
#define beta 0.1f // algorithm gain

/**
 * @brief Updates the orientation quaternion using accelerometer and gyroscope data.
 * @param gx Gyroscope x-axis measurement in radians/second.
 * @param gy Gyroscope y-axis measurement in radians/second.
 * @param gz Gyroscope z-axis measurement in radians/second.
 * @param ax Accelerometer x-axis measurement in any unit (e.g., g's, m/s^2).
 * @param ay Accelerometer y-axis measurement.
 * @param az Accelerometer z-axis measurement.
 * @param q The orientation quaternion {w, x, y, z}. Must be initialized to {1, 0, 0, 0} before the first call.
 */
void madgwick_ahrs_update_imu(float gx, float gy, float gz, float ax, float ay, float az, float q[4])
{
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q1, _4q2;
    float _2q0q2, _2q2q3, q0q0, q1q1, q2q2, q3q3;

    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-q[1] * gx - q[2] * gy - q[3] * gz);
    qDot2 = 0.5f * (q[0] * gx + q[2] * gz - q[3] * gy);
    qDot3 = 0.5f * (q[0] * gy - q[1] * gz + q[3] * gx);
    qDot4 = 0.5f * (q[0] * gz + q[1] * gy - q[2] * gx);

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {
        // Normalise accelerometer measurement
        recipNorm = 1.0f / sqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        _2q0 = 2.0f * q[0];
        _2q1 = 2.0f * q[1];
        _2q2 = 2.0f * q[2];
        _2q3 = 2.0f * q[3];
        _4q1 = 4.0f * q[1];
        _4q2 = 4.0f * q[2];
        _2q0q2 = 2.0f * q[0] * q[2];
        _2q2q3 = 2.0f * q[2] * q[3];
        q0q0 = q[0] * q[0];
        q1q1 = q[1] * q[1];
        q2q2 = q[2] * q[2];
        q3q3 = q[3] * q[3];

        // Gradient descent algorithm corrective step
        s0 = _2q2 * (2.0f * (q[1] * q[3] - q[0] * q[2]) - ax) - _2q1 * (2.0f * (q[0] * q[1] + q[2] * q[3]) - ay);
        s1 = _2q3 * (2.0f * (q[1] * q[3] - q[0] * q[2]) - ax) + _2q0 * (2.0f * (q[0] * q[1] + q[2] * q[3]) - ay) - _4q1 * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - az);
        s2 = -_2q0 * (2.0f * (q[1] * q[3] - q[0] * q[2]) - ax) + _2q3 * (2.0f * (q[0] * q[1] + q[2] * q[3]) - ay) - _4q2 * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - az);
        s3 = _2q1 * (2.0f * (q[1] * q[3] - q[0] * q[2]) - ax) + _2q2 * (2.0f * (q[0] * q[1] + q[2] * q[3]) - ay);
        
        // Normalise the gradient
        recipNorm = 1.0f / sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        // Apply feedback step
        qDot1 -= beta * s0;
        qDot2 -= beta * s1;
        qDot3 -= beta * s2;
        qDot4 -= beta * s3;
    }

    // Integrate rate of change of quaternion to yield quaternion
    q[0] += qDot1 * deltat;
    q[1] += qDot2 * deltat;
    q[2] += qDot3 * deltat;
    q[3] += qDot4 * deltat;

    // Normalise quaternion
    recipNorm = 1.0f / sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    q[0] *= recipNorm;
    q[1] *= recipNorm;
    q[2] *= recipNorm;
    q[3] *= recipNorm;
}
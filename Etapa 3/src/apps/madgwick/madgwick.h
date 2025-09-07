#ifndef _MADGWICK_H_
#define _MADGWICK_H_

void madgwick_ahrs_update_imu(float gx, float gy, float gz, float ax, float ay, float az, float q[4]);

#endif 
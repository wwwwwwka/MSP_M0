#ifndef _QUA_H
#define _QUA_H

#include "ti_msp_dl_config.h"

extern float pitch, roll, yaw;

float invSqrt(float x);
void IMU_Update(float gyro_x, float gyro_y, float gyro_z, float ax, float ay, float az);



#endif

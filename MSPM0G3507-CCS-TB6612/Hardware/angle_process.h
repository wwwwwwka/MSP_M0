#ifndef _ANGLE_PROCESS_H
#define _ANGLE_PROCESS_H

#include "ti_msp_dl_config.h"

#define Pi 3.14159265

extern uint8_t Way_Angle;                             //获取角度的算法，1：四元数  2：卡尔曼  3：互补滤波
extern float Angle_Pitch,Angle_Roll,Angle_Yaw;
// extern float Angle_Balance,Gyro_Balance,Gyro_Turn; //平衡倾角 平衡陀螺仪 转向陀螺仪
// extern float Acceleration_Z;

extern float gyro_x,gyro_y,gyro_z,accel_x,accel_y,accel_z;
extern float Accel_Y,Accel_Z,Accel_X,Accel_Angle_x,Accel_Angle_y,Accel_Angle_z,Gyro_X,Gyro_Z,Gyro_Y;


void Get_Angle(uint8_t way);

#endif
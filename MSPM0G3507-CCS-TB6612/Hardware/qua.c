#include "qua.h"
#include <math.h>

//-------------------------------------------
//  @brief      开方倒数函数（取至雷神3）
//  @param      x       要运算的数据
//  @return     y       开放后的倒数
//--------------------------------------------
float invSqrt(float x) 
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f375a86 - (i>>1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}



/*
 * @brief       四元数解算
 * @param
 * @return      void
 *
 * */
float IMU_Kp = 2.50f;//四元数初始值 IMU_Kp = 0.0008f;加速度计的收敛速率比例增益
float IMU_Ki = 0.0005f;//0.001f;//加速度补偿角速度积分值初始值 0.00005f;陀螺仪收敛速率积分增益
float qua_dt = 0.005f;
const float halfT = 0.0005f;
float qn0 = 1, qn1 = 0, qn2 = 0, qn3 = 1;
float exInteg = 0, eyInteg = 0, ezInteg = 0;
const float RtA = 57.2957795f; //RtA = 180.0f/pi

float pitch, roll, yaw;

void IMU_Update(float gyro_x, float gyro_y, float gyro_z, float ax, float ay, float az)
{
       float norm;
       float vx, vy, vz;//当前的机体坐标系上的重力单位向量
       float ex, ey, ez;//四元数计算值与加速度计测量值的误差

       gyro_x=gyro_x*0.0174f;//  pi/180
       gyro_y=gyro_y*0.0174f;
       gyro_z=gyro_z*0.0174f;

       float q0=qn0;
       float q1=qn1;
       float q2=qn2;
       float q3=qn3;

       //测量正常化
       norm = invSqrt(ax*ax + ay*ay + az*az); //把加速度计的三维向量转成单维向量
       ax *= norm ;
       ay *= norm ;
       az *= norm ;

       //估计方向的重力
       vx = 2*(q1*q3 - q0*q2);
       vy = 2*(q0*q1 + q2*q3);
       vz = 1-2*q1*q1-2*q2*q2 ;

       //经叉乘积并求出误差
       ex = (ay*vz - az*vy);
       ey = (az*vx - ax*vz);
       ez = (ax*vy - ay*vx);

       //滤波

       //积分误差比例积分增益
       exInteg = exInteg + ex * IMU_Ki * qua_dt;//此处ex要滤波
       eyInteg = eyInteg + ey * IMU_Ki * qua_dt;
       ezInteg = ezInteg + ez * IMU_Ki * qua_dt;

       //调整后的陀螺仪测量   互补滤波
       gyro_x = gyro_x + IMU_Kp*ex + exInteg;
       gyro_y = gyro_y + IMU_Kp*ey + eyInteg;
       gyro_z = gyro_z + IMU_Kp*ez + ezInteg;

   //    Gyro_x=gx/0.0174f;
   //    Gyro_y=gy/0.0174f;
   //    Gyro_z=gz/0.0174f;

       gyro_x=gyro_x*(0.5f*qua_dt);
       gyro_y=gyro_y*(0.5f*qua_dt);
       gyro_z=gyro_z*(0.5f*qua_dt);

       //四元数更新
       q0 = q0 + (-q1*gyro_x - q2*gyro_y - q3*gyro_z);//*halfT;
       q1 = q1 + ( q0*gyro_x + q2*gyro_z - q3*gyro_y);//*halfT;
       q2 = q2 + ( q0*gyro_y - q1*gyro_z + q3*gyro_x);//*halfT;
       q3 = q3 + ( q0*gyro_z + q1*gyro_y - q2*gyro_x);//*halfT;

     //四元数规范化
       norm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
       qn0 = q0 * norm;
       qn1 = q1 * norm;
       qn2 = q2 * norm;
       qn3 = q3 * norm;

       pitch =-asinf(2.0f*(-q0*q2 +q1*q3)) * RtA; // 俯仰   换算成度
       roll = atanf(2.0f*(q0*q1 + q2*q3 )/(q0*q0-q1*q1-q2*q2+q3*q3)) * RtA; // 横滚
       yaw  = atan(2.0f*(q0*q3 + q2*q1 )/(1-2*(q2*q2+q3*q3))) * RtA;
    //    yaw = atan2(2*(q1*q2 + q0*q3), 1-2*q2*q2-4*q2*q3-2*q3*q3)* RtA;
       pitch += gyro_y * 0.005f; // 俯仰   换算成度
       roll += gyro_x * 0.005f;
       yaw += gyro_z * 0.005f;
//       pitch = asin(2*(q0*q2 - q1*q3))* RtA;
//       roll  = asin(2*(q0*q1 + q2*q3))* RtA;
   //    eulerAngle.yaw   = atan2(2*(q1*q2 + q0*q3),-2*q2*q2 -2*.q3*Q_info.q3 + 1)*57.3f;

}

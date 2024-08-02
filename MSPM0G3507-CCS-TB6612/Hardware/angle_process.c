#include "angle_process.h"
#include "ti_msp_dl_config.h"
#include "board.h"
#include "angle_process.h"
#include "IOI2C.h"
#include "mpu6050.h"
#include "filter.h"

uint8_t Way_Angle=2;                             //获取角度的算法，1：四元数  2：卡尔曼  3：互补滤波
float Angle_Pitch=0,Angle_Roll=0,Angle_Yaw=0;
// float Angle_Balance=0,Gyro_Balance=0,Gyro_Turn=0; //平衡倾角 平衡陀螺仪 转向陀螺仪
// float Acceleration_Z=0;

float gyro_x,gyro_y,gyro_z,accel_x,accel_y,accel_z;
float Accel_Y,Accel_Z,Accel_X,Accel_Angle_x,Accel_Angle_y,Accel_Angle_z,Gyro_X,Gyro_Z,Gyro_Y;

void Get_Angle(uint8_t way)
{
    if(way==1)                           //DMP的读取在数据采集中断读取，严格遵循时序要求
    {
        Read_DMP();                          //读取加速度、角速度、倾角
        Angle_Pitch=Pitch;
        Angle_Roll=Roll;
        Angle_Yaw=Yaw;

        // Angle_Balance=Pitch;                 //更新平衡倾角,前倾为正，后倾为负
        // Gyro_Balance=gyro[0];              //更新平衡角速度,前倾为正，后倾为负
        // Gyro_Turn=gyro[2];                 //更新转向角速度
        // Acceleration_Z=accel[2];           //更新Z轴加速度计
    }
    else
    {
        Gyro_X=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_XOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_XOUT_L);    //读取X轴陀螺仪
        Gyro_Y=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_YOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_YOUT_L);    //读取Y轴陀螺仪
        Gyro_Z=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_ZOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_ZOUT_L);    //读取Z轴陀螺仪
        Accel_X=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_XOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_XOUT_L); //读取X轴加速度计
        Accel_Y=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_YOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_YOUT_L); //读取X轴加速度计
        Accel_Z=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_ZOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_ZOUT_L); //读取Z轴加速度计
        if(Gyro_X>32768)  Gyro_X-=65536;                 //数据类型转换  也可通过short强制类型转换
        if(Gyro_Y>32768)  Gyro_Y-=65536;                 //数据类型转换  也可通过short强制类型转换
        if(Gyro_Z>32768)  Gyro_Z-=65536;                 //数据类型转换
        if(Accel_X>32768) Accel_X-=65536;                //数据类型转换
        if(Accel_Y>32768) Accel_Y-=65536;                //数据类型转换
        if(Accel_Z>32768) Accel_Z-=65536;                //数据类型转换
        //Gyro_Balance=-Gyro_X;                            //更新平衡角速度
        Accel_Angle_x=atan2(Accel_Y,Accel_Z)*180/Pi;     //计算倾角，转换单位为度
        Accel_Angle_y=atan2(Accel_X,Accel_Z)*180/Pi;     //计算倾角，转换单位为度
        accel_x=Accel_X/1671.84;
        accel_y=Accel_Y/1671.84;
        accel_z=Accel_Z/1671.84;
        gyro_x=Gyro_X/16.4;                              //陀螺仪量程转换
        gyro_y=Gyro_Y/16.4;                              //陀螺仪量程转换
        gyro_z=Gyro_Z/16.4;                              //陀螺仪量程转换
        if(Way_Angle==2)
        {
             Pitch= -Kalman_Filter_x(Accel_Angle_x,gyro_x);//卡尔曼滤波
             Roll = -Kalman_Filter_y(Accel_Angle_y,gyro_y);             
        }
        else if(Way_Angle==3)
        {
             Pitch = -Complementary_Filter_x(Accel_Angle_x,gyro_x);//互补滤波
             Roll = -Complementary_Filter_y(Accel_Angle_y,gyro_y);             
        }
        // Angle_Balance=Pitch;                              //更新平衡倾角
        // Gyro_Turn=Gyro_Z;                                 //更新转向角速度
        // Acceleration_Z=Accel_Z;                           //更新Z轴加速度计

    }

}
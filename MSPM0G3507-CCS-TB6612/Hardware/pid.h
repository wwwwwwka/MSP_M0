#ifndef _PID_H
#define _PID_H

#include "ti_msp_dl_config.h"
#include "line_following.h"

extern int Get_Encoder_countA,encoderA_cnt,Get_Encoder_countB,encoderB_cnt,PWMA,PWMB;
extern float Yaw_err;
extern float Yaw_Stand;
extern float Vec_err1, Vec_err2;
extern float Vec_Stand;


typedef struct 
{
    float kp;
    float ki;
    float kd;
    float i_add;    //积分累计
    float i_th;     //积分分离
    float error;
    float err_1;
    float output;  
    float standard;  
}Angle_Register;

void Angle_PID_init(Angle_Register* m_Angle_Register);
float Angle_PID(Angle_Register* m_Angle_Register,float input, float standard);


#endif 

#include "pid.h"

Angle_Register *m_Angle_Register;

void Angle_PID_init(Angle_Register* m_Angle_Register)
{
    m_Angle_Register->err_1 = 0;
    m_Angle_Register->error = 0;
    m_Angle_Register->i_add = 0;
    m_Angle_Register->i_th = 0;
    m_Angle_Register->kd = 30;
    m_Angle_Register->ki = 0;
    m_Angle_Register->kp = 20;
    m_Angle_Register->output = 0;
    m_Angle_Register->standard = 0;
}

//函数用来控制打角
//参数：当前角度，期望角度
float Angle_PID(Angle_Register* m_Angle_Register,float input, float standard)
{
    m_Angle_Register->standard = standard;
    m_Angle_Register->error = input - m_Angle_Register->standard;
    m_Angle_Register->output = m_Angle_Register->kp*m_Angle_Register->error + m_Angle_Register->kd*(m_Angle_Register->error - m_Angle_Register->err_1);
    m_Angle_Register->err_1 = m_Angle_Register->error;
    return m_Angle_Register->output;
}


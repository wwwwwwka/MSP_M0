#include "line_following.h"
#include "angle_process.h"
#include "board.h"
#include "motor.h"
#include "pid.h"
#include "mpu6050.h"

Go_staus go_flag=0;
ABCD abcd=A1;

float yaw_now;
int temp = 0;
uint32_t ENCODER=0;

uint32_t line1, line2, line3, line4, line5, line6, line7, line8;
int pwma=PWM_Com, pwmb=PWM_Com;
int pwm_stand;
Angle_Register Angle_Register_type;

void Line_Flag_Read()
{
    line1 = DL_GPIO_readPins(LINE4_PORT, LINE4_PIN_41_PIN);
    line2 = DL_GPIO_readPins(LINE4_PORT, LINE4_PIN_42_PIN);
    line3 = DL_GPIO_readPins(LINE4_PORT, LINE4_PIN_43_PIN);
    line4 = DL_GPIO_readPins(LINE4_PORT, LINE4_PIN_44_PIN);
    line5 = DL_GPIO_readPins(LINE4_PORT, LINE4_PIN_45_PIN);
    line6 = DL_GPIO_readPins(LINE4_PORT, LINE4_PIN_46_PIN);
    line7 = DL_GPIO_readPins(LINE4_PORT, LINE4_PIN_47_PIN);
    line8 = DL_GPIO_readPins(LINE4_PORT, LINE4_PIN_48_PIN);
}

void Derection_Go()
{
    
    if(!line4 && !line5 && line1 && line2 && line3 && line6 && line7 && line8)
        go_flag = Go_Straight;
    else
        go_flag = Go_Trun;
    
    if(line1 && line2 && line3 && line4 && line5 && line6 && line7 && line8)
    {
        static uint16_t temp=0;
        temp++;
        if(temp == 100)
        {
            go_flag = No_line;
            DL_GPIO_setPins(LED1_PORT,LED1_PIN_0_PIN);
            temp = 0;
        }
    }

    if(go_flag < No_line) {temp=0;DL_GPIO_clearPins(LED1_PORT,LED1_PIN_0_PIN);}
    
}



void Line_Control()
{
    Line_Flag_Read();
    Derection_Go();
    
    switch (go_flag) 
    {
        case Go_Straight:
            Set_PWM(PWM_Com, PWM_Com);
            break;
        case Go_Trun:
            if(!line4 || !line3 || !line2 || !line1)
            {   
                if(!line4)  Set_PWM(PWM_Com-50, PWM_Com+50); //左转小xiao
                if(!line3)  Set_PWM(PWM_Com-100, PWM_Com+100); //左转小
                if(!line2)  Set_PWM(PWM_Com-150, PWM_Com+150); //左转中
                if(!line1)  Set_PWM(0, 400); //左转大
            }
            else if(!line8 || !line7 || !line6 || !line5)
            {
                if(!line5)  Set_PWM(PWM_Com+50, PWM_Com-50); //右转小xiao
                if(!line6)  Set_PWM(PWM_Com+100, PWM_Com-100); //右转小
                if(!line7)  Set_PWM(PWM_Com+150, PWM_Com-150); //右转中
                if(!line8)  Set_PWM(400, 0); //右转大
            }
            break;
        case No_line:
        
            if(abcd==D4) abcd=A1;
            else if(abcd==B2) abcd=C3;

            if(abcd==A1)
            {
               for(;;)
               {    
                    
                    Get_Angle(1);
        
                    Yaw_err  = Angle_PID(&Angle_Register_type, Angle_Yaw, 0);

                    pwma=PWM_Com+(int)Yaw_err;
                    pwmb=PWM_Com-(int)Yaw_err;
                    // pwma = Velocity_A(PWM_Com-(int)Yaw_err,encoderA_cnt);//130
                    // pwmb = Velocity_B(PWM_Com+(int)Yaw_err,encoderB_cnt);//170
                    
                    if(pwma>=1200) pwma=1200;
                    else if(pwma<=0) pwma=0;
                    if(pwmb>=1200) pwmb=1200;
                    else if(pwmb<=0) pwmb=0;
                    
                    ENCODER=ENCODER+(encoderA_cnt+encoderB_cnt)/10;
 
                    printf("%d, %d, %d \n\r", ENCODER, encoderA_cnt, encoderB_cnt);
                    Set_PWM(pwma, pwmb);

                    delay_ms(50);

                    Line_Flag_Read();
                    
                    if((!line1 || !line2 || !line3 || !line4 || !line5 || !line6 || !line7 || !line8)&&ENCODER>=700)
                    {
                        abcd=B2;
                        ENCODER=0;
                        break;
                    }
                    
                }
            }
            else if(abcd==C3)
            {
                Set_PWM(0, 0);

                for(;;)
                {
                    Get_Angle(1);
                    if(Angle_Yaw>=0) Angle_Yaw=Angle_Yaw-180;
                    else if(Angle_Yaw<0) Angle_Yaw=180+Angle_Yaw;

                    if(Angle_Yaw<5 && Angle_Yaw>-5)
                        break;
                    else
                    {
                        Yaw_err  = Angle_PID(&Angle_Register_type, Angle_Yaw, 0);
                        Set_PWM(-Yaw_err, Yaw_err);
                    }
                }

                for(;;)
                {
                    Get_Angle(1);
                    if(Angle_Yaw>=0) Angle_Yaw=Angle_Yaw-180;
                    else if(Angle_Yaw<0) Angle_Yaw=180+Angle_Yaw;

                    Yaw_err  = Angle_PID(&Angle_Register_type, Angle_Yaw, 0);
                    pwma=PWM_Com+(int)Yaw_err;
                    pwmb=PWM_Com-(int)Yaw_err;
                    // pwma = Velocity_A(PWM_Com-(int)Yaw_err,encoderA_cnt);//130
                    // pwmb = Velocity_B(PWM_Com+(int)Yaw_err,encoderB_cnt);//170
                    
                    if(pwma>=1200) pwma=1200;
                    else if(pwma<=0) pwma=0;
                    if(pwmb>=1200) pwmb=1200;
                    else if(pwmb<=0) pwmb=0;

                    //printf("%f, %d, %d \n\r", Angle_Yaw, pwma, pwmb);
                    ENCODER=ENCODER+(encoderA_cnt+encoderB_cnt)/10;
 
                    printf("%d, %d, %d \n\r", ENCODER, encoderA_cnt, encoderB_cnt);
                    Set_PWM(pwma, pwmb);

                    delay_ms(50);

                    Line_Flag_Read();

                    if((!line1 || !line2 || !line3 || !line4 || !line5 || !line6 || !line7 || !line8)&&ENCODER>=750)
                    {
                        abcd=D4;
                        ENCODER=0;
                        break;
                    }
                }
            }
            break;

    }
}

void Angle_init(void)
{
   Angle_PID_init(&Angle_Register_type);
}
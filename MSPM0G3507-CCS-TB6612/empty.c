#include "ti_msp_dl_config.h"
#include "led.h"
#include "board.h"
#include "motor.h"
#include "IOI2C.h"
#include "mpu6050.h"
#include "filter.h"
#include "angle_process.h"
//#include "ADC.h"
#include "pid.h"
#include "line_following.h"
#include "qua.h"

//-75 102

typedef enum
{
    MODE_1,
    MODE_2,
    MODE_3,
    MODE_4
}MODE_Status;

int Cur_mode = MODE_2;

int Get_Encoder_countA,encoderA_cnt,Get_Encoder_countB,encoderB_cnt,PWMA,PWMB;
float Yaw_err;
float Yaw_Stand;
float Vec_err1, Vec_err2;
float Vec_Stand;
//Angle_Register Angle_Register_type;

bool flag_A=false, flag_B=false, flag_C=false, flag_D=false;

Angle_Register Angle_Register_type;

int main(void)
{
    SYSCFG_DL_init();
    DL_Timer_startCounter(PWM_0_INST);
    
    NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);
    NVIC_ClearPendingIRQ(GPIO_MULTIPLE_GPIOA_INT_IRQN);
    NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);

    MPU6050_initialize();
    DMP_Init();
    Angle_init();
    
    DL_GPIO_clearPins(BEER_PORT,BEER_PIN_1_PIN);
    DL_GPIO_setPins(LED1_PORT,LED1_PIN_0_PIN);
    delay_ms(1000);
    DL_GPIO_setPins(BEER_PORT,BEER_PIN_1_PIN);
    DL_GPIO_clearPins(LED1_PORT,LED1_PIN_0_PIN);

    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);
    NVIC_EnableIRQ(GPIO_MULTIPLE_GPIOA_INT_IRQN);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);

    while (1) {
        if(DL_GPIO_readPins(KEY1_PORT, KEY1_PIN_18_PIN))
            break;
    }

    while (1) {
        
        Get_Angle(1);
        
        // Yaw_err  = Angle_PID(&Angle_Register_type, Angle_Yaw, 90);

        // pwma=PWM_Com+(int)Yaw_err;
        // pwmb=PWM_Com-(int)Yaw_err;
        // // pwma = Velocity_A(PWM_Com-(int)Yaw_err,encoderA_cnt);//130
        // // pwmb = Velocity_B(PWM_Com+(int)Yaw_err,encoderB_cnt);//170
        
        // if(pwma>=1200) pwma=1200;
	    // else if(pwma<=0) pwma=0;
        // if(pwmb>=1200) pwmb=1200;
	    // else if(pwmb<=0) pwmb=0;

        // printf("%f, %d, %d \n\r", Angle_Yaw, pwma, pwmb);
        // Set_PWM(pwma, pwmb);

        // delay_ms(50);

        // printf("%n, %n, %n, %n, %n, %n, %n, %n \n\r", line1, line2, line3, line4, line5, line6, line7, line8);
        Line_Control();
        //if(Cur_mode == MODE_2)
        // {
        //     if((Angle_Yaw<=180 && Angle_Yaw>=145) && (Angle_Yaw<=-145 && Angle_Yaw>=-180))
        //     {
        //         if(Angle_Yaw<0)
        //             Angle_Yaw = Angle_Yaw+180;
        //         else
        //             Angle_Yaw = Angle_Yaw-180;
        //     }
        //         Yaw_err  = Angle_PID(&Angle_Register_type, Angle_Yaw, 0);
        // }

        // Yaw_err  = Angle_PID(&Angle_Register_type, Angle_Yaw, 90);
        // Set_PWM((int)-Yaw_err, (int)Yaw_err);

        
        //printf("%f\n\r",Angle_Yaw);
        
        // delay_ms(10);
        //LED_Flash(10);

    }
}

void TIMER_0_INST_IRQHandler(void)
{
    if(DL_TimerA_getPendingInterrupt(TIMER_0_INST))
    {
        if(DL_TIMER_IIDX_ZERO)
        {
            encoderA_cnt = Get_Encoder_countA;
            encoderB_cnt = Get_Encoder_countB;
            
            Get_Encoder_countA = 0;
            Get_Encoder_countB = 0;
        }
    }
}




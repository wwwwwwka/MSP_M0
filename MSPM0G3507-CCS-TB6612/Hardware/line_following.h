#ifndef _LINE_FOLLOWING_H
#define _LINE_FOLLOWING_H

#include "pid.h"
#include "ti/driverlib/dl_i2c.h"
#include "ti_msp_dl_config.h"

// #define Go_Straight 11
// #define GO_Left     12
// #define Go_Right    13
// #define Go_Angle    14

#define PWM_CHANGE 200
#define PWM_Com 600
#define PWM_Max 800
#define PWM_Min 0

typedef enum
{
   Go_Straight,
   Go_Trun,
   No_line
   
}Go_staus;

typedef enum
{
   A1,
   B2,
   C3,
   D4
}ABCD;

extern uint32_t line1, line2, line3, line4, line5, line6, line7, line8;
//extern int go_flag;
extern int pwma, pwmb;
extern int pwm_stand;


void Line_Flag_Read();
void Derection_Go();
void Line_Control();
void Angle_init();


#endif
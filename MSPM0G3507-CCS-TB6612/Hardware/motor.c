#include "motor.h"

float Velcity_Kp=1,  Velcity_Ki=0.5,  Velcity_Kd; 

int Velocity_A(int TargetVelocity, int CurrentVelocity)
{  
    int Bias;  
		static int ControlVelocityA, Last_biasA; 
		
		Bias=TargetVelocity-CurrentVelocity; 
		
		ControlVelocityA+=Velcity_Ki*(Bias-Last_biasA)+Velcity_Kp*Bias;  
                                                                   //Velcity_Kp*(Bias-Last_bias) 
	                                                                 //Velcity_Ki*Bias             
		Last_biasA=Bias;	
	    if(ControlVelocityA>=1200) ControlVelocityA=1200;
	    else if(ControlVelocityA<=0) ControlVelocityA=0;
		return ControlVelocityA; 
}

int Velocity_B(int TargetVelocity, int CurrentVelocity)
{  
    int Bias;  
		static int ControlVelocityB, Last_biasB; 
		
		Bias=TargetVelocity-CurrentVelocity; 
		
		ControlVelocityB+=Velcity_Ki*(Bias-Last_biasB)+Velcity_Kp*Bias;  
                                                                   //Velcity_Kp*(Bias-Last_bias) 
	                                                                 //Velcity_Ki*Bias             
		Last_biasB=Bias;	
	    if(ControlVelocityB>=1200) ControlVelocityB=1200;
	    else if(ControlVelocityB<=0) ControlVelocityB=0;
		return ControlVelocityB; 
}

void Set_PWM(int pwma,int pwmb)
{
	
	if(pwma>=0)
	{
		DL_GPIO_setPins(AIN1_PORT,AIN1_PIN_12_PIN);
		DL_GPIO_clearPins(AIN2_PORT,AIN2_PIN_13_PIN);
		DL_Timer_setCaptureCompareValue(PWM_0_INST,ABS(pwma),GPIO_PWM_0_C0_IDX);
	}
	else
	{
		DL_GPIO_setPins(AIN2_PORT,AIN2_PIN_13_PIN);
		DL_GPIO_clearPins(AIN1_PORT,AIN1_PIN_12_PIN);
		DL_Timer_setCaptureCompareValue(PWM_0_INST,ABS(pwma),GPIO_PWM_0_C0_IDX);
	}
	if(pwmb>=0)
	{
		DL_GPIO_setPins(BIN1_PORT,BIN1_Pin_Bin1_PIN);
		DL_GPIO_clearPins(BIN2_PORT,BIN2_Pin_Bin2_PIN);
		DL_Timer_setCaptureCompareValue(PWM_0_INST,ABS(pwmb),GPIO_PWM_0_C1_IDX);
	}
    else
	{
		DL_GPIO_setPins(BIN2_PORT,BIN2_Pin_Bin2_PIN);
		DL_GPIO_clearPins(BIN1_PORT,BIN1_Pin_Bin1_PIN);
		DL_Timer_setCaptureCompareValue(PWM_0_INST,ABS(pwmb),GPIO_PWM_0_C1_IDX);
	}		

}




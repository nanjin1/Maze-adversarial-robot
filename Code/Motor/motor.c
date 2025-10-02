#include "motor.h"



void motor_init(void){
		HAL_TIM_Base_Start(&htim1);
		HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
		HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
		HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
		
		HAL_TIM_Base_Start(&htim2);
		HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
		HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);
		HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4);
	
		pid_init();
}



/*
��������motor_set_pwm   L0-1-�� R0-2-�� L1-3-�� R1-4-��
L0 ---TIM1.12 		R0  ---TIM1.3.4
L1 ---TIM2.12		  R1  ---TIM2.3.4
*/
void motor_set_pwm(uint8_t motor, int32_t pid_out)
{
	int32_t ccr = 0;
	
	if (pid_out >= 0)
	{
		if (pid_out > MOTOR_PWM_MAX)
			ccr = MOTOR_PWM_MAX;
		else
			ccr = pid_out;
		
		switch (motor)
		{
			case 1: TIM1->CCR1 = 0; TIM1->CCR3 = ccr;	break;  //��ǰ
			case 2: TIM2->CCR3 = 0; TIM2->CCR1 = ccr;	break;  //���
			case 3: TIM1->CCR2 = 0; TIM1->CCR4 = ccr;	break;  //��ǰ
			case 4: TIM2->CCR4 = 0; TIM2->CCR2 = ccr;	break;  //�Һ�
			default: ; //TODO
		}
	}
	
	else if (pid_out < 0)
	{
		if (pid_out < -MOTOR_PWM_MAX)
			ccr = MOTOR_PWM_MAX;
		else
			ccr = -pid_out;
		
		switch (motor)
		{
			case 1: TIM1->CCR3 = 0; TIM1->CCR1 = ccr;	break;  //��
			case 2: TIM2->CCR1 = 0; TIM2->CCR3 = ccr;   break; 	//��
			case 3: TIM1->CCR4 = 0; TIM1->CCR2 = ccr;	break;  //��
			case 4: TIM2->CCR2 = 0; TIM2->CCR4 = ccr;	break;  //��
			default: ; //TODO
		}
	}
}

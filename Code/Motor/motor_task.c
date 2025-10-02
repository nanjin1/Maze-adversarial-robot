#include "motor_task.h"
#include  "encoder.h"
#include "motor.h"
#include "uart.h"
#include "speed_ctrl.h"
#include "pid.h"
#include "turn.h"
#include "scaner.h"
#include "bsp_linefollower.h"
#include "sin_generate.h"
#include "bsp_buzzer.h"
#include "openmv.h"
#include "map.h"
#include "QR.h"
TaskHandle_t motor_handler;
volatile int dirct[4] = {1,1,1,1};  
volatile uint8_t PIDMode;
uint8_t line_gyro_switch = 0;
#define PI  3.1415926535

void motor_task(void *pvParameters){
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();   //��ȡϵͳ����
	static uint8_t mouse = 0;    //С����
	static uint8_t scaner_time = 0;
	while(1){
		scaner_time++;
		if (PIDMode == is_Line)
		{
			getline_error();
		}
		if(scaner_time == 5)
		{
				get_motor_speed();
			
				motor_all.encoder_avg = (float)((float)(pulse_num[0]*dirct[0] + pulse_out[0]*MAX_pulse*dirct[0]) + (float)(pulse_num[1]*dirct[1] + pulse_out[1]*MAX_pulse*dirct[1]) + (float)(pulse_num[2]*dirct[2] + pulse_out[2]*MAX_pulse*dirct[2]) + (float)(pulse_num[3]*dirct[3] + pulse_out[3]*MAX_pulse*dirct[3])) / 4.0f;
				//�־�6.8cm
				motor_all.Distance =((motor_all.encoder_avg * 6.8f * PI)/(572.0f));//ת��Ϊ1��
				//��������ƽ��->ѭ��
				if (line_gyro_switch == 1)    //�����line_gyro_switch����PIDMODE�л�������������ı�־λ
				{
					line_pid_obj = gyroG_pid;
					TC_speed = TG_speed;
					gyroG_pid = (struct P_pid_obj){0,0,0,0,0,0,0};
					TG_speed = (struct Gradual){0,0,0};
					line_gyro_switch = 0;
				}
				//ѭ��->��������ƽ��
				else if (line_gyro_switch == 2)
				{
					gyroG_pid = line_pid_obj;
					TG_speed = TC_speed;
					line_pid_obj = (struct P_pid_obj){0,0,0,0,0,0,0};
					TC_speed = (struct Gradual){0,0,0};
					line_gyro_switch = 0;
				}
				else
				{
					if (PIDMode == is_Line)
					{
						//getline_error();
						gradual_cal(&TC_speed,motor_all.Cspeed,motor_all.Cincrement);
						Go_Line(TC_speed.Now);
					}
					else
						motor_all.Cspeed = 0;
					
					//ת��PID����
					if (PIDMode == is_Turn)	
					{
						if(Turn_Angle(angle.AngleT))
						{
							gyroT_pid = (struct P_pid_obj){0,0,0,0,0,0};
						}
					}
					if (PIDMode == is_SmallTurn)	
					{
						if(Small_Turn_Angle(angle.AngleT))
						{
							small_gyroT_pid = (struct P_pid_obj){0,0,0,0,0,0};
						}
					}
					//��ƽ��PID����
					if (PIDMode == is_Gyro)
					{ 
	//					gradual_cal(&TG_speed,motor_all.Gspeed,motor_all.Gincrement);
						runWithAngle(angle.AngleG, motor_all.Gspeed);
					}
					else
						motor_all.Gspeed = 0;
				}
				
				mouse++;
				if(mouse>100){
					mouse =0;
					HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_1);
				}

				if(PIDMode == is_Free)
				{
					
				}
				else
				{
					if(ScanMode == is_Front)
					{
						motor_set_pwm(1, (int32_t)motor_all.Lspeed);
						motor_set_pwm(2, (int32_t)motor_all.Lspeed);
						motor_set_pwm(3, (int32_t)motor_all.Rspeed);
						motor_set_pwm(4, (int32_t)motor_all.Rspeed);
					}
					else if(ScanMode == is_Back)
					{
						motor_set_pwm(1, -(int32_t)motor_all.Rspeed);
						motor_set_pwm(2, -(int32_t)motor_all.Rspeed);
						motor_set_pwm(3, -(int32_t)motor_all.Lspeed);
						motor_set_pwm(4, -(int32_t)motor_all.Lspeed);
					}
				}
				scaner_time = 0;
			}
		vTaskDelayUntil(&xLastWakeTime, (1/portTICK_RATE_MS));//��������5ms // INCLUDE_vTaskDelayUntil 1
	
   }
}

//PID���񴴽�
void motor_task_create(void){
	  xTaskCreate((TaskFunction_t ) motor_task,//������
	               (const char *)"motor_task",	  //��������
								 (uint32_t) motor_size,    //�����ջ��С
								 (void* )NULL,                  //���ݸ����������ָ�����
								 (UBaseType_t) motor_task_priority, //��������ȼ�
								(TaskHandle_t *)&motor_handler ); //������
							 }
void pid_mode_switch(uint8_t target_mode)
{
	switch (target_mode)
	{
		case is_Turn: {
			gyroG_pid = (struct P_pid_obj){0,0,0,0,0,0,0};
			TG_speed = (struct Gradual){0,0,0};
			line_pid_obj = (struct P_pid_obj){0,0,0,0,0,0,0};
			TC_speed = (struct Gradual){0,0,0};
			gyroT_pid = (struct P_pid_obj){0,0,0,0,0,0,0};
			small_gyroT_pid = (struct P_pid_obj){0,0,0,0,0,0,0};
			break;
		}
		
		case is_SmallTurn: {
			gyroG_pid = (struct P_pid_obj){0,0,0,0,0,0,0};
			TG_speed = (struct Gradual){0,0,0};
			line_pid_obj = (struct P_pid_obj){0,0,0,0,0,0,0};
			TC_speed = (struct Gradual){0,0,0};
			gyroT_pid = (struct P_pid_obj){0,0,0,0,0,0,0};
			small_gyroT_pid = (struct P_pid_obj){0,0,0,0,0,0,0};
			break;
		}
		
		case is_Line: {
			if (PIDMode == is_Gyro)  //����ƽ���л���ѭ��
			{
				line_gyro_switch = 1;
			}
			else
			{
				gyroT_pid = (struct P_pid_obj){0,0,0,0,0,0,0};
				small_gyroT_pid = (struct P_pid_obj){0,0,0,0,0,0,0};
			}
			break;
		}
		
		case is_Gyro: {
			if (PIDMode == is_Line)  //��ѭ���л�����ƽ��
			{
				line_gyro_switch = 2;
			}
			else
			{
				gyroT_pid = (struct P_pid_obj){0,0,0,0,0,0,0};
				small_gyroT_pid = (struct P_pid_obj){0,0,0,0,0,0,0};
			}
			break;
		}
		
		case is_Free: {
			break;
		}
		
		case is_No: {
			line_pid_obj = (struct P_pid_obj){0,0,0,0,0,0,0};
			TC_speed = (struct Gradual){0,0,0};
			gyroG_pid = (struct P_pid_obj){0,0,0,0,0,0,0};
			TG_speed = (struct Gradual){0,0,0};
			gyroT_pid = (struct P_pid_obj){0,0,0,0,0,0,0};
			small_gyroT_pid = (struct P_pid_obj){0,0,0,0,0,0,0};
			break;
		}
		case is_sp :
		{
			break;
		}
	}
	PIDMode = target_mode;
}

void get_motor_speed()
{
	if(ScanMode == is_Front)
	{
		dirct[0] = dirct[1] = -1;
		dirct[2] = dirct[3] = 1;
	}
	else if(ScanMode == is_Back)
	{
		dirct[0] = dirct[1] = 1;
		dirct[2] = dirct[3] = -1;
	}
	
	if(high_time[0]==0)
		motor_L0.measure=0;
	else
		motor_L0.measure = (float)(100000/high_time[0]) * dirct[0] * direction[0] ;
	if(high_time[1]==0)
		motor_L1.measure=0;
	else
		motor_L1.measure = (float)(100000/high_time[1]) * dirct[1] * direction[1] ;
	if(high_time[2]==0)
		motor_R0.measure=0;
	else
		motor_R0.measure = (float)(100000/high_time[2]) * dirct[2] * direction[2] ;
	if(high_time[3]==0)
		motor_R1.measure=0;
	else
		motor_R1.measure = (float)(100000/high_time[3]) * dirct[2] * direction[3] ;
}

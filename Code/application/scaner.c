#include "scaner.h"
#include "map.h"
#include "math.h"
#include "turn.h"
#include "stdio.h"
#include "pid.h"
#include "speed_ctrl.h"
#include "bsp_linefollower.h"
#include "motor.h"

#define LINE_SPEED_MAX 50
#define Speed_Compensate   5
#define BLACK 0								//ѭ����
#define WHLITE 1							//ѭ����

const float line_weight[8] = {-0.9,-0.5,-0.3,-0.1,0.1,0.3,0.5,0.9};//0.3
//���ҵ���
volatile struct Scaner_Set scaner_set = {0,0};

uint8_t ScanMode;
//float CatchsensorNum = 0;	//Ѱ��ʱ��Ŀ�괫����λ�ã�һ�������Ϊ0��΢΢���õ�
volatile SCANER Scaner;

uint32_t data_head = 0;
uint32_t data_tail = 0;

#define Line_color BLACK
//ѭ���� 1234578 87654321

void Go_Line(float speed){
	float Fspeed;				//����PID�����Ľ��
					
	line_pid_obj.measure = Scaner.error;  							//��ǰѭ�������ڵ�λ�ã�������-7��0��0��7
	line_pid_obj.target = scaner_set.CatchsensorNum;		//Ŀ��
//	printf("error=%f\r\n",line_pid_obj.measure);
	Fspeed = positional_PID(&line_pid_obj, &line_pid_param); //����λ��PID����
	
	if (Fspeed>=LINE_SPEED_MAX) 
		Fspeed = LINE_SPEED_MAX;
	else if (Fspeed<=-LINE_SPEED_MAX) 
		Fspeed = -LINE_SPEED_MAX;
	
	Fspeed *= fabsf(speed)/50;
//	printf("%f\r\n",Fspeed);
	
	
	motor_all.Lspeed = speed-Fspeed;
	motor_all.Rspeed = speed+Fspeed;
	
}
uint8_t getline_error()//��ø��²�ͬѲ��ģʽ�µ����ֵ
{
	get_detail();//��ȡѲ��ֵ
	if(Line_Scan(&Scaner, Lamp_Max, scaner_set.EdgeIgnore))
	{
		return 1;
	}
	else
	{
		return 0;
	}
//	printf("%d   %d   %d   %d\r\n",motor_L0.measure,motor_L1.measure,motor_R0.measure,motor_R1.measure);
}
void get_detail()
{
	//�������Ʒ���0
		//��ߵĵ�
	//��������   1 2 3 4 5 6 7 8
	if(Line_color)
	{
		data_head = 0xff;//����
		data_tail = 0xff;
		Scaner.detail = 0xff;
		if(ScanMode == is_Front)
		{
			Scaner.detail^=((uint8_t)HAL_GPIO_ReadPin(GPIO_Left1,GPIO_Left1_P)<<0);
			Scaner.detail^=((uint8_t)HAL_GPIO_ReadPin(GPIO_Left2,GPIO_Left2_P)<<1);
			Scaner.detail^=((uint8_t)HAL_GPIO_ReadPin(GPIO_Left3,GPIO_Left3_P)<<2);
			Scaner.detail^=((uint8_t)HAL_GPIO_ReadPin(GPIO_Left4,GPIO_Left4_P)<<3);
			Scaner.detail^=((uint8_t)HAL_GPIO_ReadPin(GPIO_Left5,GPIO_Left5_P)<<4);
			Scaner.detail^=((uint8_t)HAL_GPIO_ReadPin(GPIO_Left6,GPIO_Left6_P)<<5);
			Scaner.detail^=((uint8_t)HAL_GPIO_ReadPin(GPIO_Left7,GPIO_Left7_P)<<6);
			Scaner.detail^=((uint8_t)HAL_GPIO_ReadPin(GPIO_Left8,GPIO_Left8_P)<<7);
		}
		else
		{
			Scaner.detail^=((uint8_t)HAL_GPIO_ReadPin(GPIO_Right1,GPIO_Right1_P)<<7); //��ͬ���1.��ͬ���0
			Scaner.detail^=((uint8_t)HAL_GPIO_ReadPin(GPIO_Right2,GPIO_Right2_P)<<6);
			Scaner.detail^=((uint8_t)HAL_GPIO_ReadPin(GPIO_Right3,GPIO_Right3_P)<<5);
			Scaner.detail^=((uint8_t)HAL_GPIO_ReadPin(GPIO_Right4,GPIO_Right4_P)<<4);
			Scaner.detail^=((uint8_t)HAL_GPIO_ReadPin(GPIO_Right5,GPIO_Right5_P)<<3);
			Scaner.detail^=((uint8_t)HAL_GPIO_ReadPin(GPIO_Right6,GPIO_Right6_P)<<2);
			Scaner.detail^=((uint8_t)HAL_GPIO_ReadPin(GPIO_Right7,GPIO_Right7_P)<<1);
			Scaner.detail^=((uint8_t)HAL_GPIO_ReadPin(GPIO_Right8,GPIO_Right8_P)<<0);
		}
	}
	else
	{
		Scaner.detail = 0x0;
		if(ScanMode == is_Front)
		{
			Scaner.detail|=((uint8_t)HAL_GPIO_ReadPin(GPIO_Left1,GPIO_Left1_P)<<0);
			Scaner.detail|=((uint8_t)HAL_GPIO_ReadPin(GPIO_Left2,GPIO_Left2_P)<<1);
			Scaner.detail|=((uint8_t)HAL_GPIO_ReadPin(GPIO_Left3,GPIO_Left3_P)<<2);
			Scaner.detail|=((uint8_t)HAL_GPIO_ReadPin(GPIO_Left4,GPIO_Left4_P)<<3);
			Scaner.detail|=((uint8_t)HAL_GPIO_ReadPin(GPIO_Left5,GPIO_Left5_P)<<4);
			Scaner.detail|=((uint8_t)HAL_GPIO_ReadPin(GPIO_Left6,GPIO_Left6_P)<<5);
			Scaner.detail|=((uint8_t)HAL_GPIO_ReadPin(GPIO_Left7,GPIO_Left7_P)<<6);
			Scaner.detail|=((uint8_t)HAL_GPIO_ReadPin(GPIO_Left8,GPIO_Left8_P)<<7);
		}	
		else if(ScanMode == is_Back)
		{
			Scaner.detail|=((uint8_t)HAL_GPIO_ReadPin(GPIO_Right1,GPIO_Right1_P)<<7);
			Scaner.detail|=((uint8_t)HAL_GPIO_ReadPin(GPIO_Right2,GPIO_Right2_P)<<6);
			Scaner.detail|=((uint8_t)HAL_GPIO_ReadPin(GPIO_Right3,GPIO_Right3_P)<<5);
			Scaner.detail|=((uint8_t)HAL_GPIO_ReadPin(GPIO_Right4,GPIO_Right4_P)<<4);
			Scaner.detail|=((uint8_t)HAL_GPIO_ReadPin(GPIO_Right5,GPIO_Right5_P)<<3);
			Scaner.detail|=((uint8_t)HAL_GPIO_ReadPin(GPIO_Right6,GPIO_Right6_P)<<2);
			Scaner.detail|=((uint8_t)HAL_GPIO_ReadPin(GPIO_Right7,GPIO_Right7_P)<<1);
			Scaner.detail|=((uint8_t)HAL_GPIO_ReadPin(GPIO_Right8,GPIO_Right8_P)<<0);
		}
	}
	
//	if(ScanMode == is_Front)
//		Scaner.detail = data_head;
//	else if(ScanMode == is_Back)
//		Scaner.detail = data_tail;
}
//ѭ��ɨ��
uint8_t Line_Scan(SCANER *scaner, unsigned char sensorNum, int8_t edge_ignore)
{
	float error = 0;
	u8 linenum=0;//��¼�ߵ���Ŀ
	u8 lednum=0;
	int8_t lednum_tmp = 0;
							//��ö�����Ѳ��ֵ
	for(uint8_t i=0;i<sensorNum;i++) 		//��С�������������������������������
	{								//linenum������¼�ж������ߣ�line������¼�ڼ����ߡ�
		if((scaner->detail&(0x1<<i))) 
		{
			lednum++;
			if(!(scaner->detail&(1<<(i+1)))) 
				++linenum;			//�ȶ�ȡ��������������������⵽��1��Ϊ0��Ϊһ����
		}
	}
	
	scaner->lineNum = linenum;			
	scaner->ledNum = lednum;
	
	for(uint8_t i= edge_ignore; i<sensorNum - edge_ignore; i++) 
	{
			lednum_tmp += (scaner->detail>>(sensorNum-1-i))&0X01;
			error += ((scaner->detail>>(sensorNum-1-i))&0X01) * line_weight[i];
	}
	
	if(lednum_tmp>=3)
	{
		Scaner.error=0;
		return 0;
	}   
	if(lednum==0|lednum_tmp==0)
	{
		error=0;
	}
	else
	{
		error/=(float)lednum_tmp;		//ȡƽ��
	}
	Scaner.error = error;
	return 0;
}


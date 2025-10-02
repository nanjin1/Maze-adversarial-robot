#include "map.h"
#include "barrier.h"
#include "sys.h"
#include "usart.h"
#include "delay.h"
#include "scaner.h"
#include "imu_task.h"
#include "turn.h"
#include "speed_ctrl.h"
#include "motor_task.h"
#include "bsp_linefollower.h"
#include "math.h"
#include "usmart.h"
#include "bsp_buzzer.h"
#include "motor.h"
#include "motor_task.h"
#include "encoder.h"
#include "uart.h"
#include "openmv.h"
#include "dijkstra.h"

struct Map_State map = {0,0};

#define SlowSpeed 2500
#define FastSpeed 3500

int route[800] = {0};

NODESR nodesr;	//�����м����
					

//��ͼ��ʼ��
void mapInit()
{
	u8 i=0;
	
	ScanMode = is_Front;
	
	for(i=0;i<204;i++)			//ȫ��ͼ�Ƕȵ���
	{
		Node[i].flag &=(~STOPTURN);
		Node[i].speed /=2;
	}
	
	nodesr.flag=0;
}

u8 getNextConnectNode(u8 nownode,u8 nextnode) 
{//�õ����ڵ㵽���ڽ��ĵ�ַ
	unsigned char rest = ConnectionNum[nownode];//���������ڵĽ����
	unsigned char addr = Address[nownode];//�õ�����addr
	int i = 0;
	for (i = 0; i < rest; i++) 
	{
		if(Node[addr].nodenum == nextnode)//���ؽ���ַ
		{			
			return addr;
		}
		addr++;
	}
	return 0;
}

u8 deal_arrive()
{				
	register uint8_t lnum = 0, i = 0;
	register uint16_t seed = 0;
	getline_error();
	static uint8_t times=0;
	if ((nodesr.nowNode.flag & DLEFT) == DLEFT)  //����
	{
		//���3��������1��������
		if (Scaner.ledNum>=3)
		{
			seed = 0X80;
			for (i = 0; i<4; i++)
			{
				if (Scaner.detail & seed)
					++lnum;
				if (lnum >= 3)
					times++;
				seed >>= 1;
			}
			lnum = 0;
		}
		else if((Scaner.detail & 0xE0) == 0xE0)
		{	
			times++;
		}
		else{
			times=0;
		}
		if(times>=4)
		{
			times=0;
			return 1;
		}
	}
	if ((nodesr.nowNode.flag & DRIGHT) == DRIGHT)//�Ұ��
	{    	
		if (Scaner.ledNum >= 3)
		{
			seed = 0X01;
			for (i = 0; i<4; i++)
			{
				if (Scaner.detail & seed)
					++lnum;
				if (lnum >= 3)
					times++;
				seed <<= 1;
			}
			lnum = 0;
		}
		else if((Scaner.detail & 0x07) == 0x07)
		{
			times++;
		}
		else 
		{
			times=0;
		}
		if(times>=4)
		{
			times=0;
			return 1;
		}
	}
	if((nodesr.nowNode.flag & In_Road) == In_Road)
	{
		return 1;
	}
    return 0;
	
}

static uint8_t time = 0;
void Cross()
{			
	float num = 0;
	static u8 _flag=1;			//��㶯����־λ
	if(map.point == 0)
	{
		nodesr.nextNode	= Node[getNextConnectNode(nodesr.nowNode.nodenum,route[map.point++])];//��ȡ��һ���
	}
	if(_flag==1)//ѭ��
	{
		pid_mode_switch(is_Line);
		if(fabsf(motor_all.Distance) >= 0.7f*nodesr.nowNode.step) //�������0.7�Ͳ�ѭ����
		{
			_flag=0;
			if((nodesr.nowNode.flag&RESTMPUZ)==RESTMPUZ || ((data_head == 0x18) && (data_tail == 0x18)))//������У��
			{
				mpuZreset(imu.yaw, nodesr.nowNode.angle);     //��ȡ������Z
			}
			if(nodesr.nowNode.angle == nodesr.nextNode.angle)
			{
				motor_all.Cspeed = FastSpeed;
			}
			else
			{
				motor_all.Cspeed = SlowSpeed;// ԭ�� 40 
			}
		}
		else//δ�ﵽ0.7�ľ���
		{
			if((nodesr.nowNode.flag&RESTMPUZ)==RESTMPUZ || ((data_head == 0x18) && (data_tail == 0x18)))//������У��
			{
				mpuZreset(imu.yaw, nodesr.nowNode.angle);     //��ȡ������Z
			}
			motor_all.Cspeed = nodesr.nowNode.speed;// ԭ�� 40 
		}
	}
	else if(_flag==0)//·������0.7  �ж�·��
	{
		map_function(nodesr.nowNode.function);
		if((nodesr.flag&0x04)!=0x04&&deal_arrive()&&(nodesr.flag&0x80)!=0x80&&(nodesr.flag)!=0x20)	//�ж��Ƿ񵽴�·��
		{
	//		buzzer_on();
			scaner_set.CatchsensorNum = 0;
			nodesr.flag |= 0x04;	//����·��		
		}	
	}
	if((nodesr.flag&0x04)==0x04)//ת�䣨�Ѿ�����·�ڣ�
	{	
		nodesr.flag&=~0x04;		//	�������·�ڱ�־
		if(route[map.point-1] != 255)	 //route[i]==255����һ����·������
		{		
		    //�����һ���Ƕ��뵱ǰ���Ƕ���ͬ������Ҫת������Ҫ����	    
			if ((fabs(need2turn(getAngleZ(),nodesr.nextNode.angle))<10) 
				||(fabs(need2turn(nodesr.nowNode.angle,nodesr.nextNode.angle))<10))
			{														 		
				_flag=1;
			}
			else //��Ҫת
			{
				int break_time = 0;
				if(nodesr.nowNode.flag&STOPTURN)    //STOPTURN��־λ����ԭ��ת
				{	    
					motor_all.Cspeed = 1500;
					num = motor_all.Distance;
//					if(nodesr.nowNode.angle - nodesr.nextNode.angle == -90 || nodesr.nowNode.angle - nodesr.nextNode.angle == 270)
//					{
						while(fabsf(motor_all.Distance-num) < 12)//12
						{
							vTaskDelay(2);
						}
//					}
//					else if(nodesr.nowNode.angle - nodesr.nextNode.angle == -270 || nodesr.nowNode.angle - nodesr.nextNode.angle == 90)
//					{
//						while(fabsf(motor_all.Distance-num) < 14)//12
//						{
//							vTaskDelay(2);
//						}
//					}
					float relativeAngle;
					angle.AngleT=getAngleZ()+need2turn(nodesr.nowNode.angle,nodesr.nextNode.angle);
					if(angle.AngleT>180)	angle.AngleT -= 360;
					else if(angle.AngleT<-180)	angle.AngleT += 360;
					relativeAngle = angle.AngleT;
//					angle.AngleT = nodesr.nextNode.angle;
					pid_mode_switch(is_Turn);	
					while(fabsf(need2turn(nodesr.nextNode.angle,getAngleZ())) > 5)
					{
						break_time++;
						float now_angle = getAngleZ();						
						getline_error();
						if(Scaner.lineNum==1 && (fabsf(need2turn(relativeAngle,getAngleZ()))<27/*fabsf(need2turn(now_angle,angle.AngleT))*0.3f*/) && break_time >= 500)
						{
							if(Scaner.detail&0x03)
							{
								pid_mode_switch(is_Free);
								if(ScanMode == is_Front)
								{
									motor_set_pwm(1, 1500);
									motor_set_pwm(2, 1500);
									motor_set_pwm(3, -1500);
									motor_set_pwm(4, -1500);
								}
								if(ScanMode == is_Back)//
								{
									motor_set_pwm(1, 1500);
									motor_set_pwm(2, 1500);
									motor_set_pwm(3, -1500);
									motor_set_pwm(4, -1500);
								}
							}
							vTaskDelay(1);
					    }
						if(Scaner.lineNum==1 && (fabsf(need2turn(relativeAngle,getAngleZ()))<27/*fabsf(need2turn(now_angle,angle.AngleT))*0.3f*/) && break_time >= 500)
						{
							if(Scaner.detail&0xC0)
							{
								pid_mode_switch(is_Free);
								if(ScanMode == is_Front)
								{
									motor_set_pwm(1, -1500);
									motor_set_pwm(2, -1500);
									motor_set_pwm(3, 1500);
									motor_set_pwm(4, 1500);
								}
								if(ScanMode == is_Back)//
								{
									motor_set_pwm(1, -1500);
									motor_set_pwm(2, -1500);
									motor_set_pwm(3, 1500);
									motor_set_pwm(4, 1500);
								}
							}
							vTaskDelay(1);
						}
						if(Scaner.lineNum==1&&(Scaner.detail&0x18)&&(fabsf(need2turn(angle.AngleT,getAngleZ()))</*fabsf(need2turn(now_angle,angle.AngleT))*0.3f*/27))
						{
							break;
						}
						vTaskDelay(2);
						if(break_time >= 1500)
							break;
						if(Scaner.ledNum == 0 && motor_L0.measure < 10 && motor_R0.measure < 10 && motor_L1.measure < 10 && motor_R1.measure < 10)
							break;
					}	
					break_time = 0;
					getline_error();
					if(Scaner.ledNum == 0)
					{
						struct PID_param origin_parm = gyroT_pid_param;
						gyroT_pid_param.ki = 0.5;
						if(nodesr.nowNode.angle - nodesr.nextNode.angle == -90 || nodesr.nowNode.angle - nodesr.nextNode.angle == 270)
							angle.AngleT = getAngleZ()+30;
						else if(nodesr.nowNode.angle - nodesr.nextNode.angle == -270 || nodesr.nowNode.angle - nodesr.nextNode.angle == 90)
							angle.AngleT = getAngleZ()-30;
						
						pid_mode_switch(is_Turn);	
						while(fabsf(need2turn(angle.AngleT,getAngleZ())) > 5)
						{
							break_time++;
							char now_angle = getAngleZ();						
							getline_error();
							static int seed;
							if(Scaner.lineNum ==1 && (fabsf(need2turn(angle.AngleT,getAngleZ()))<27/*fabsf(need2turn(now_angle,angle.AngleT))*0.3f*/) && break_time >= 500)
							{
								if(Scaner.detail&0x03)
								{
									pid_mode_switch(is_Free);
									if(ScanMode == is_Front)//��ת
									{
										motor_set_pwm(1, 1500);
										motor_set_pwm(2, 1500);
										motor_set_pwm(3, -1500);
										motor_set_pwm(4, -1500);
									}
									if(ScanMode == is_Back)//
									{
										motor_set_pwm(1, 1500);
										motor_set_pwm(2, 1500);
										motor_set_pwm(3, -1500);
										motor_set_pwm(4, -1500);
									}
								}
								vTaskDelay(1);
							}
							if(Scaner.lineNum==1 && (fabsf(need2turn(angle.AngleT,getAngleZ()))<27/*fabsf(need2turn(now_angle,angle.AngleT))*0.3f*/) && break_time >= 500)
							{
								if(Scaner.detail&0xC0)
								{
									pid_mode_switch(is_Free);
									if(ScanMode == is_Front)
									{
										motor_set_pwm(1, -1500);
										motor_set_pwm(2, -1500);
										motor_set_pwm(3, 1500);
										motor_set_pwm(4, 1500);
									}
									if(ScanMode == is_Back)//
									{
										motor_set_pwm(1, -1500);
										motor_set_pwm(2, -1500);
										motor_set_pwm(3, 1500);
										motor_set_pwm(4, 1500);
									}
								}
								vTaskDelay(1);
							}
							if(Scaner.lineNum==1
							&&(Scaner.detail&0x18)
							&&(fabsf(need2turn(angle.AngleT,getAngleZ()))<27/*fabsf(need2turn(now_angle,angle.AngleT))*0.3f*/))
							{
								break;
							}
							vTaskDelay(2);
							if(break_time >= 1500)
								break;
						}
						break_time = 0;
						gyroT_pid_param = origin_parm;
					}
				}
				else		//����ת
				{	
					pid_mode_switch(is_Gyro);
					struct PID_param origin_parm = gyroG_pid_param;
					float origin_speedMax = motor_all.GyroG_speedMax;
					float relativeAngle;
					
					gyroG_pid_param.kp = 1;/*1.1*/
					gyroG_pid_param.ki = 0;
					gyroG_pid_param.kd = 0;
					
					motor_all.Gspeed=motor_all.Cspeed;
					motor_all.GyroG_speedMax=0.03f*motor_all.Gspeed;
					
					angle.AngleG=getAngleZ()+need2turn(nodesr.nowNode.angle,nodesr.nextNode.angle);
					if(angle.AngleG>180)	angle.AngleG -= 360;
					else if(angle.AngleG<-180)	angle.AngleG += 360;
					
					relativeAngle = angle.AngleG;
					while(fabsf(need2turn(relativeAngle,getAngleZ())) > 5)
					{
						break_time++;
						getline_error();
						if(Scaner.lineNum==1&&(Scaner.detail&0x3C)&&(fabsf(need2turn(angle.AngleG,getAngleZ()))<fabsf(need2turn(angle.AngleG,nodesr.nowNode.angle))*0.2f))
						{
							break;
						}
						if(break_time>=750 && Scaner.ledNum!=0)
						{
							break;
						}
						vTaskDelay(2);
					}
					break_time = 0;	
					gyroG_pid_param = origin_parm;
					motor_all.GyroG_speedMax = origin_speedMax;
					buzzer_off();
				}
			}
			//ת������ѭ��ģʽ�����½�㣬��ձ�����ֵ
			_flag=1;
			buzzer_off();
			pid_mode_switch(is_Line);	
			nodesr.lastNode=nodesr.nowNode; //���½��
			nodesr.nowNode=nodesr.nextNode;	//���½��
			motor_all.Cspeed=nodesr.nowNode.speed;
			nodesr.nextNode	= Node[getNextConnectNode(nodesr.nowNode.nodenum,route[map.point++])];//��ȡ��һ���	
			if(nodesr.nextNode.nodenum == B3  || nodesr.nextNode.nodenum == B5 
			|| nodesr.nextNode.nodenum == B6  || nodesr.nextNode.nodenum == B11
			|| nodesr.nextNode.nodenum == B12 || nodesr.nextNode.nodenum == B14)
			{
				nodesr.nextNode.step = 5;
			}
			else if(nodesr.nowNode.angle != nodesr.nextNode.angle 
				&& (nodesr.nextNode.function & Treasure) == Treasure)
			{
				nodesr.nextNode.step = 0;
			}
			else if(nodesr.nowNode.angle == nodesr.nextNode.angle 
				&& (nodesr.nextNode.function & Treasure) == Treasure)
			{
				nodesr.nextNode.step = 14;
			}
			if((nodesr.nextNode.function&Treasure) == Treasure)
			{
				nodesr.nowNode.flag |= STOPTURN;
			}
			if((nodesr.nextNode.function&Platform) == Platform)
			{
				nodesr.nowNode.flag &=(~STOPTURN);
			}
			if((nodesr.lastNode.angle == nodesr.nowNode.angle)
			&&	(nodesr.nowNode.flag & In_Road) == In_Road 
			&& (nodesr.nextNode.flag & Corner) == Corner
			&& (nodesr.nextNode.function & Treasure) == Treasure)
			{
				nodesr.nowNode.step = 40;
			}
			scaner_set.EdgeIgnore=0;
			scaner_set.CatchsensorNum=0;
		    encoder_clear();
			motor_all.Distance = 0;
			nodesr.flag&=~0x04;		//	�������·�ڱ�־
		} 
		else//���·������
		{		
			motor_all.Cspeed=0;
			CarBrake();
			_flag=1;
			map.routetime+=1;
		}	
	}
	if(nodesr.flag&0x80)
	{
		_flag=1;
		nodesr.nextNode	= Node[getNextConnectNode(nodesr.nowNode.nodenum,route[map.point++])];		//����һ�����и���
		nodesr.flag&=~0x80;
		pid_mode_switch(is_Line);
		motor_all.Cspeed=nodesr.nowNode.speed;
	}
}
	

void map_function(u8 fun)
{
	switch(fun)
	{
		case 0:break;
		case 1:break;		//Ѱ��
		case Treasure      :TREASURE();break;//ƽ̨
		case Platform  	  :PLATFORM();break;//���� ���ȣ��ٶ�
		default:break;		
	}
}



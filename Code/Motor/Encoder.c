#include "encoder.h"
#include  <stdlib.h>
#include <string.h>
#include "uart.h"
#include "speed_ctrl.h"
wheel wheel_1,wheel_2,wheel_3,wheel_4;
uint32_t capture_Buf_before[4] = {0},capture_Buf_latter[4] = {0};   //��ż���ʱ��
uint8_t  capture_Cnt[4] = {0};    			//״̬��־λ�������л�ģʽ��
uint32_t high_time[4] = {0};        			//�ߵ�ƽʱ��
uint32_t real_time[4] = {0};    					//CCR��ʵ��ʱ
int  direction[4] = {0,0,0,0};                //����תʶ��
uint8_t  level_before[4]={0},level_latter[4] = {0};         //ë���ж�
int  pulse_num[4] = {0};                  //�������
int  pulse_out[4] = {0};                     //��¼�������
uint8_t frist_flag[4]={0};								//��һ���ռ��˸����ݵı�־
uint8_t temp_i[4]={0};
uint32_t temp_time[4][8] = {0};        			//�ߵ�ƽ�ݴ�ʱ��

uint8_t dog[4] = {0};   //�����

//����С�����������벶�������ĸ���ʱ��
void Encoder_init(void){
	//��ǰ
	wheel_1.TIM = htim3;  //A�ඨʱ��
	wheel_1.GPIO_2 = GPIOB;        //ͨ��2--B��
	wheel_1.GPIO_PORT_2 = GPIO_PIN_8;
	wheel_1.channelA=TIM_CHANNEL_4;//A��ͨ��
	HAL_TIM_IC_Start_IT(&(wheel_1.TIM),wheel_1.channelA);	  //�������벶��
	__HAL_TIM_SET_CAPTUREPOLARITY(&wheel_1.TIM,wheel_1.channelA, TIM_INPUTCHANNELPOLARITY_RISING);  //���������ز���
		//���
	wheel_2.TIM = htim3;
	wheel_2.GPIO_2 = GPIOB;        //ͨ��2--B��
	wheel_2.GPIO_PORT_2 = GPIO_PIN_9;
	wheel_2.channelA=TIM_CHANNEL_3;//A��ͨ��
	HAL_TIM_IC_Start_IT(&(wheel_2.TIM), wheel_2.channelA);	  //�������벶��
	__HAL_TIM_SET_CAPTUREPOLARITY(&wheel_2.TIM, wheel_2.channelA, TIM_INPUTCHANNELPOLARITY_RISING);  //���������ز���
	//��ǰ
	wheel_3.TIM = htim3;
	wheel_3.GPIO_2 = GPIOE;        //ͨ��2--B��
	wheel_3.GPIO_PORT_2 = GPIO_PIN_1;
	wheel_3.channelA=TIM_CHANNEL_1;//A��ͨ��
	HAL_TIM_IC_Start_IT(&(wheel_3.TIM), wheel_3.channelA);	  //�������벶��
	__HAL_TIM_SET_CAPTUREPOLARITY(&wheel_3.TIM, wheel_3.channelA, TIM_INPUTCHANNELPOLARITY_RISING);  //���������ز���
	//�Һ�
	wheel_4.TIM = htim3;
	wheel_4.GPIO_2 = GPIOE;        //ͨ��1--B��
	wheel_4.GPIO_PORT_2 = GPIO_PIN_0;
	wheel_4.channelA=TIM_CHANNEL_2;//A��ͨ��
	HAL_TIM_IC_Start_IT(&(wheel_4.TIM), wheel_4.channelA);	  //�������벶��
	__HAL_TIM_SET_CAPTUREPOLARITY(&wheel_4.TIM, wheel_4.channelA, TIM_INPUTCHANNELPOLARITY_RISING);  //���������ز���
	
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{

	//������
	if( htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4 )
	{
		dog[0]=0;
		switch(capture_Cnt[0]){
			
			case 0:   //ģʽ1   --------   ���������� ��ʼ��ʱ
				capture_Buf_before[0] = (wheel_1.TIM.Instance) ->CCR4;//��ȡ��ǰ�Ĳ���ֵ
				__HAL_TIM_SET_CAPTUREPOLARITY(&wheel_1.TIM,wheel_1.channelA,TIM_ICPOLARITY_FALLING);  //����Ϊ�½��ز���
				capture_Cnt[0]++; //��ģʽһ�л���ģʽ��
			  if(( level_before[0]= HAL_GPIO_ReadPin(wheel_1.GPIO_2,wheel_1.GPIO_PORT_2) )==RESET){   //�ж�����ת	           
					           direction[0] = BACKWARD;   //��ת
				 // printf("��ת!\r\n");
				}
				else{direction[0] = FORWARD;   //��ת
				//	printf("ZHENG----------------------------------------ת!\r\n");
				}
				break;
			
			  case 1:   //  ģʽ��  --------   �����½��� �رռ�ʱ 
				capture_Buf_latter[0] = (wheel_1.TIM.Instance) ->CCR4;//��ȡ��ǰ�Ĳ���ֵ
				HAL_TIM_IC_Stop_IT(&wheel_1.TIM,wheel_1.channelA); //ֹͣ����  ����: __HAL_TIM_DISABLE(&htim5);
			  level_latter[0]=HAL_GPIO_ReadPin(wheel_1.GPIO_2,wheel_1.GPIO_PORT_2);//�ٴζ�ȡB��
			  if(level_before[0] != level_latter[0]){   //�ж��Ƿ���ë��
					if(capture_Buf_latter[0]<capture_Buf_before[0]){       //�������
				    real_time[0] = (65535-capture_Buf_before[0])+capture_Buf_latter[0] +1;
			      }
					else{
						real_time[0] =  capture_Buf_latter[0]- capture_Buf_before[0] ;  //û�����
			      }
					if(!frist_flag[0]){    //ǰ�˴�Ҫ���ռ�����
						temp_time[0][temp_i[0]] = real_time[0];
						temp_i[0]++;
						if(temp_i[0]==8){
							for(uint8_t i =0; i<8;i++){
								real_time[0]+= temp_time[0][i];
							}
							high_time[0] = real_time[0]/8;
							frist_flag[0]=1;
						}
					}
					else{
						high_time[0] = 0;
						memmove(&temp_time[0], &temp_time[0][1], sizeof(uint32_t) * 7);
						temp_time[0][7] = real_time[0];
						for(uint8_t i =0; i<8;i++){
								real_time[0]+= temp_time[0][i];	
							}
						high_time[0] = real_time[0]/8;
					}
					pulse_num[0]+= direction[0];     //��������������ڼ���λ��
							if(pulse_num[0]>=MAX_pulse){
						pulse_out[0]++;
						pulse_num[0]=0;
					}
					else if(pulse_num[0]<=-MAX_pulse){
						pulse_out[0]--;
						pulse_num[0]=0;
					}
				}
			  //printf("high_time = %5d us   direction = %d\r\n",high_time[0],direction[0]);     //���ߵ�ƽʱ��
			  capture_Cnt[0] = 0;  //��ձ�־
					HAL_TIM_IC_Start_IT(&(wheel_1.TIM), wheel_1.channelA);	  //�������벶��
				__HAL_TIM_SET_CAPTUREPOLARITY(&wheel_1.TIM, wheel_1.channelA, TIM_INPUTCHANNELPOLARITY_RISING);  //���������ز���
			  break; 
		 }
	}
	
	//���
	if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
	{
		dog[1]=0;
		switch(capture_Cnt[1]){
			
			case 0:   //ģʽ1   --------   ���������� ��ʼ��ʱ
				capture_Buf_before[1] = (wheel_2.TIM.Instance) ->CCR3;//��ȡ��ǰ�Ĳ���ֵ
				__HAL_TIM_SET_CAPTUREPOLARITY(&wheel_2.TIM,wheel_2.channelA,TIM_ICPOLARITY_FALLING);  //����Ϊ�½��ز���
				capture_Cnt[1]++; //��ģʽһ�л���ģʽ��
			  if(( level_before[1]=HAL_GPIO_ReadPin(wheel_2.GPIO_2,wheel_2.GPIO_PORT_2) )==RESET){   //�ж�����ת	           
					direction[1] = BACKWARD;   //��ת
				//   printf("��ת!\r\n");
				}
				else{direction[1] = FORWARD;   //��ת
				//printf("-------------------------------------------------\r\n");
				}
				break;
			
			  case 1:   //  ģʽ��  --------   �����½��� �رռ�ʱ 
				capture_Buf_latter[1] = (wheel_2.TIM.Instance) ->CCR3;//��ȡ��ǰ�Ĳ���ֵ
				HAL_TIM_IC_Stop_IT(&wheel_2.TIM,wheel_2.channelA); //ֹͣ����  ����: __HAL_TIM_DISABLE(&htim5);
			  level_latter[1]=HAL_GPIO_ReadPin(wheel_2.GPIO_2,wheel_2.GPIO_PORT_2);//�ٴζ�ȡB��
			  if(level_before[1] != level_latter[1]){   //�ж��Ƿ���ë��
					if(capture_Buf_latter[1]<capture_Buf_before[1]){       //�������
				    real_time[1] = (65535-capture_Buf_before[1])+capture_Buf_latter[1] + 1;
			      }
					else{
						real_time[1] =  capture_Buf_latter[1]- capture_Buf_before[1] ;  //û�����
			      }
					if(!frist_flag[1]){    //ǰ�˴�Ҫ���ռ�����
						temp_time[1][temp_i[1]] = real_time[1];
						temp_i[1]++;
						if(temp_i[1]==8){
							for(uint8_t i =0; i<8;i++){
								real_time[1]+= temp_time[1][i];
							}
							high_time[1] = real_time[1]/8;
							frist_flag[1]=1;
						}
					}
					else{
						high_time[1] = 0;
						memmove(&temp_time[1], &temp_time[1][1], sizeof(uint32_t) * 7);
						temp_time[1][7] = real_time[1];
						for(uint8_t i =0; i<8;i++){
								real_time[1]+= temp_time[1][i];	
							}
						high_time[1] = real_time[1]/8;
					}
					pulse_num[1]+= direction[1] ;     //��������������ڼ���λ��
							if(pulse_num[1]>=MAX_pulse){
						pulse_out[1]++;
						pulse_num[1]=0;
					}
					else if(pulse_num[1]<=-MAX_pulse){
						pulse_out[1]--;
						pulse_num[1]=0;
					}
				}
			  //printf("high_time = %5d us   direction = %d\r\n",high_time[0],direction[0]);     //���ߵ�ƽʱ��
			  capture_Cnt[1] = 0;  //��ձ�־
					HAL_TIM_IC_Start_IT(&(wheel_2.TIM), wheel_2.channelA);	  //�������벶��
				__HAL_TIM_SET_CAPTUREPOLARITY(&wheel_2.TIM, wheel_2.channelA, TIM_INPUTCHANNELPOLARITY_RISING);  //���������ز���
			  break; 
		 }
	}
	//printf("%d\r\n",(int)(TIM3->SR));
	//��ǰ
	if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
	{
		dog[2]=0;
		switch(capture_Cnt[2]){
			
			case 0:   //ģʽ1   --------   ���������� ��ʼ��ʱ
				capture_Buf_before[2] = (wheel_3.TIM.Instance) ->CCR1;//��ȡ��ǰ�Ĳ���ֵ
				__HAL_TIM_SET_CAPTUREPOLARITY(&wheel_3.TIM,wheel_3.channelA,TIM_ICPOLARITY_FALLING);  //����Ϊ�½��ز���
				capture_Cnt[2]++; //��ģʽһ�л���ģʽ��
			  if(( level_before[2]=HAL_GPIO_ReadPin(wheel_3.GPIO_2,wheel_3.GPIO_PORT_2) )==RESET){   //�ж�����ת	           
					           direction[2] = BACKWARD;   //��ת
//								printf("��ת!\r\n");
				}
				else{direction[2] = FORWARD;   //��ת
//					printf("��ת!\r\n");
				}
				break;
			
			  case 1:   //  ģʽ��  --------   �����½��� �رռ�ʱ 
				capture_Buf_latter[2] = (wheel_3.TIM.Instance) ->CCR1;//��ȡ��ǰ�Ĳ���ֵ
				HAL_TIM_IC_Stop_IT(&wheel_3.TIM,wheel_3.channelA); //ֹͣ����  ����: __HAL_TIM_DISABLE(&htim5);
			  level_latter[2]=HAL_GPIO_ReadPin(wheel_3.GPIO_2,wheel_3.GPIO_PORT_2);//�ٴζ�ȡB��
			  if(level_before[2] != level_latter[2]){   //�ж��Ƿ���ë��
					if(capture_Buf_latter[2]<capture_Buf_before[2]){       //�������
				    real_time[2] = (65535-capture_Buf_before[2])+capture_Buf_latter[2] + 1;
			      }
					else{
						real_time[2] =  capture_Buf_latter[2]- capture_Buf_before[2] ;  //û�����
			      }
					if(!frist_flag[2]){    //ǰ�˴�Ҫ���ռ�����
						temp_time[2][temp_i[2]] = real_time[2];
						temp_i[2]++;
						if(temp_i[2]==8){
							for(uint8_t i =0; i<8;i++){
								real_time[2]+= temp_time[2][i];
							}
							high_time[2] = real_time[2]/8;
							frist_flag[2]=1;
						}
					}
					else{
						high_time[2] = 0;
						memmove(&temp_time[2], &temp_time[2][1], sizeof(uint32_t) * 7);
						temp_time[2][7] = real_time[2];
						for(uint8_t i =0; i<8;i++){
								real_time[2]+= temp_time[2][i];	
							}
						high_time[2] = real_time[2]/8;
					}
					pulse_num[2]+= direction[2] ;     //��������������ڼ���λ��
							if(pulse_num[2]>=MAX_pulse){
						pulse_out[2]++;
						pulse_num[2]=0;
					}
					else if(pulse_num[0]<=-MAX_pulse){
						pulse_out[2]--;
						pulse_num[2]=0;
					}
				}
			  //printf("high_time = %5d us   direction = %d\r\n",high_time[0],direction[0]);     //���ߵ�ƽʱ��
			  capture_Cnt[2] = 0;  //��ձ�־
					HAL_TIM_IC_Start_IT(&(wheel_3.TIM), wheel_3.channelA);	  //�������벶��
				__HAL_TIM_SET_CAPTUREPOLARITY(&wheel_3.TIM,wheel_3.channelA, TIM_INPUTCHANNELPOLARITY_RISING);  //���������ز���
			  break; 
		 }
	}
	
	//�Һ�
	if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2 )
	{
		dog[3]=0;
		switch(capture_Cnt[3]){
			
			case 0:   //ģʽ1   --------   ���������� ��ʼ��ʱ
				capture_Buf_before[3] = (wheel_4.TIM.Instance) ->CCR2;//��ȡ��ǰ�Ĳ���ֵ
				__HAL_TIM_SET_CAPTUREPOLARITY(&wheel_4.TIM,wheel_4.channelA,TIM_ICPOLARITY_FALLING);  //����Ϊ�½��ز���
				capture_Cnt[3]++; //��ģʽһ�л���ģʽ��
			  if(( level_before[3]=HAL_GPIO_ReadPin(wheel_4.GPIO_2,wheel_4.GPIO_PORT_2) )==RESET){   //�ж�����ת	           
					           direction[3] = BACKWARD;   //��ת
				  				//	printf("GHHJWSDHDCHJHJת!\r\n");
				}
				else{direction[3] = FORWARD;   //��ת
									//	printf("��ת!\r\n");
				}
				break;
			
			  case 1:   //  ģʽ��  --------   �����½��� �رռ�ʱ 
				capture_Buf_latter[3] = (wheel_4.TIM.Instance) ->CCR2;//��ȡ��ǰ�Ĳ���ֵ
				HAL_TIM_IC_Stop_IT(&wheel_4.TIM,wheel_4.channelA); //ֹͣ����  ����: __HAL_TIM_DISABLE(&htim5);
			  level_latter[3]=HAL_GPIO_ReadPin(wheel_4.GPIO_2,wheel_4.GPIO_PORT_2);//�ٴζ�ȡB��
			  if(level_before[3] != level_latter[3]){   //�ж��Ƿ���ë��
					if(capture_Buf_latter[3]<capture_Buf_before[3]){       //�������
				    real_time[3] = (65535-capture_Buf_before[3])+capture_Buf_latter[3] + 1 ;
			      }
					else{
						real_time[3] =  capture_Buf_latter[3]- capture_Buf_before[3] ;  //û�����
			      }
					if(!frist_flag[3]){    //ǰ�˴�Ҫ���ռ�����
						temp_time[3][temp_i[3]] = real_time[3];
						temp_i[3]++;
						if(temp_i[3]==8){
							for(uint8_t i =0; i<8;i++){
								real_time[3]+= temp_time[3][i];
							}
							high_time[3] = real_time[3]/8;
							frist_flag[3]=1;
						}
					}
					else{
						high_time[3] = 0;
						memmove(&temp_time[3], &temp_time[3][1], sizeof(uint32_t) * 7);
						temp_time[3][7] = real_time[3];
						for(uint8_t i =0; i<8;i++){
								real_time[3]+= temp_time[3][i];	
							}
						high_time[3] = real_time[3]/8;
					}
					pulse_num[3]+= direction[3] ;     //��������������ڼ���λ��
							if(pulse_num[3]>=MAX_pulse){
						pulse_out[3]++;
						pulse_num[3]=0;
					}
					else if(pulse_num[3]<=-MAX_pulse){
						pulse_out[3]--;
						pulse_num[3]=0;
					}
				}
			  //printf("high_time = %5d us   direction = %d\r\n",high_time[0],direction[0]);     //���ߵ�ƽʱ��
			  capture_Cnt[3] = 0;  //��ձ�־
					HAL_TIM_IC_Start_IT(&(wheel_4.TIM), wheel_4.channelA);	  //�������벶��
				__HAL_TIM_SET_CAPTUREPOLARITY(&wheel_4.TIM,wheel_4.channelA, TIM_INPUTCHANNELPOLARITY_RISING);  //���������ز���
			  break; 
		 }
	}
}
//���¼�������
void encoder_clear(void)
{
	HAL_TIM_IC_Stop_IT(&wheel_1.TIM,wheel_1.channelA);
	HAL_TIM_IC_Stop_IT(&wheel_2.TIM,wheel_2.channelA);
	HAL_TIM_IC_Stop_IT(&wheel_3.TIM,wheel_3.channelA);
	HAL_TIM_IC_Stop_IT(&wheel_4.TIM,wheel_4.channelA);
	memset((void * )pulse_num,(int)0,(unsigned int )sizeof(pulse_num));//����
	memset((void * )pulse_out,(int)0,(unsigned int )sizeof(pulse_out));//����
	motor_all.Distance = 0;
	HAL_TIM_IC_Start_IT(&(wheel_1.TIM), wheel_1.channelA);	  //�������벶��
	HAL_TIM_IC_Start_IT(&(wheel_2.TIM), wheel_2.channelA);	  //�������벶��
	HAL_TIM_IC_Start_IT(&(wheel_3.TIM), wheel_3.channelA);	  //�������벶��
	HAL_TIM_IC_Start_IT(&(wheel_4.TIM), wheel_4.channelA);	  //�������벶��
}

#include "imu_task.h"
#include "usart.h"
#include "main.h"
#include "uart.h"
#include "stdio.h"
#include "string.h"
struct Imu imu;
UART_HandleTypeDef gyro;//UART���


void gyro_init(uint32_t bound)
{	
	//UART ��ʼ������
	gyro.Instance=UART4;					    //USART2
	gyro.Init.BaudRate=bound;				    //������
	gyro.Init.WordLength=UART_WORDLENGTH_8B;   //�ֳ�Ϊ8λ���ݸ�ʽ
	gyro.Init.StopBits=UART_STOPBITS_1;	    //һ��ֹͣλ
	gyro.Init.Parity=UART_PARITY_NONE;		    //����żУ��λ
	gyro.Init.HwFlowCtl=UART_HWCONTROL_NONE;   //��Ӳ������
	gyro.Init.Mode=UART_MODE_TX_RX;		    //�շ�ģʽ
	HAL_UART_Init(&gyro);					    //HAL_UART_Init()��ʹ��UART3
	__HAL_UART_ENABLE_IT(&gyro, UART_IT_RXNE);
}

//////�����ǽ����ж�
//void UART4_IRQHandler()
//{
//	static uint8_t i,flag=0;
//	static uint8_t data[11];
//	if(__HAL_UART_GET_FLAG(&gyro,UART_FLAG_RXNE)!=0){
//		uint8_t temp = gyro.Instance->RDR;
//		//printf("temp=%d\r\n",temp);
//		if(temp==0x55&&flag==0){
//			flag=1;
//		}
//		if(flag==1){
//			data[i] = temp;
//			i++;
//			if(i==2&&temp!=0x55){
//					flag=0;
//					i=0;
//			}
//			if(i==11){
//					uint8_t sum = 0;
//					for (int j=0; j<10; j++){
//						sum += data[j];
//					}
//					if(sum==data[10]){
//						imu.roll   = 180.0 * (short) ((data[5]<<8)|data[4])/32768.0;  
//						imu.pitch  = 180.0 * (short) ((data[7]<<8)|data[6])/32768.0;//����(��Ϊ��)
//						imu.yaw    = 180.0 * (short) ((data[9]<<8)|data[8])/32768.0;
//					//	printf("roll=%f,pitch=%f,yaw=%f \r\n",imu.roll,imu.pitch,imu.yaw);
//					}	
//					flag=0;
//					i=0;
//			}
//		}
//	}
//	gyro.Instance->ISR = 0;   //���SR��־λ
//}
#define BUFFER_SIZE 22
uint8_t imu_rx_buf[BUFFER_SIZE] = {0};
uint8_t imu_rx_len = 0;
float roll,pitch,yaw;
void imu_receive_init(void)
{
	__HAL_UART_ENABLE_IT(&huart4, UART_IT_IDLE);
	HAL_UART_Receive_DMA(&huart4,imu_rx_buf,BUFFER_SIZE);
}
void UART4_IRQHandler(void)
{
	uint32_t flag_idle = 0;
	
	flag_idle = __HAL_UART_GET_FLAG(&huart4,UART_FLAG_IDLE); 
	if((flag_idle != RESET))
	{ 
		__HAL_UART_CLEAR_IDLEFLAG(&huart4);

		HAL_UART_DMAStop(&huart4); 
		uint32_t temp = __HAL_DMA_GET_COUNTER(&hdma_uart4_rx);   
		imu_rx_len = BUFFER_SIZE - temp; 
	
		if(imu_rx_buf[11] == 0x55)
		{
			if (imu_rx_buf[12] == 0X53)
			{
				imu.yaw    = 180.0 * (short) ((imu_rx_buf[18]<<8)|imu_rx_buf[17])/32768.0;
			}
		}
		memset(imu_rx_buf,0,imu_rx_len);
		imu_rx_len = 0;
		
	}
	HAL_UART_Receive_DMA(&huart4,imu_rx_buf,BUFFER_SIZE);
	HAL_UART_IRQHandler(&huart4);
}

#include "openmv.h"
#include "usart.h"
#include "math.h"
#include "barrier.h"
#include "stdio.h"
#include "Scaner.h"
#include "QR.h"
#include "string.h"
//color
//1 �����汦��
//2 ����汦��
//3 ����α����
//4 ���α����
//5 ������ɫ��
//6 ������ɫ��

UART_HandleTypeDef mv;//UART���
UART_HandleTypeDef mv_R;//UART���
uint8_t color;

void mv_init(uint32_t bound)
{	
	//UART ��ʼ������
	mv.Instance=UART7;					  
	mv.Init.BaudRate=bound;				    //������
	mv.Init.WordLength=UART_WORDLENGTH_8B;   //�ֳ�Ϊ8λ���ݸ�ʽ
	mv.Init.StopBits=UART_STOPBITS_1;	    //һ��ֹͣλ
	mv.Init.Parity=UART_PARITY_NONE;		    //����żУ��λ
	mv.Init.HwFlowCtl=UART_HWCONTROL_NONE;   //��Ӳ������
	mv.Init.Mode=UART_MODE_TX_RX;		    //�շ�ģʽ
	HAL_UART_Init(&mv);					    //HAL_UART_Init()��ʹ��UART3
	__HAL_UART_ENABLE_IT(&mv, UART_IT_RXNE);
	close_mv();
}

void mvR_init(uint32_t bound)
{	
	//UART ��ʼ������
	mv_R.Instance=USART6;					  
	mv_R.Init.BaudRate=bound;				    //������
	mv_R.Init.WordLength=UART_WORDLENGTH_8B;   //�ֳ�Ϊ8λ���ݸ�ʽ
	mv_R.Init.StopBits=UART_STOPBITS_1;	    //һ��ֹͣλ
	mv_R.Init.Parity=UART_PARITY_NONE;		    //����żУ��λ
	mv_R.Init.HwFlowCtl=UART_HWCONTROL_NONE;   //��Ӳ������
	mv_R.Init.Mode=UART_MODE_TX_RX;		    //�շ�ģʽ
	HAL_UART_Init(&mv_R);					    //HAL_UART_Init()��ʹ��UART3
	__HAL_UART_ENABLE_IT(&mv_R, UART_IT_RXNE);
	close_mvR();
}

void open_mv()
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);
}

void close_mv()
{
	color = 0;
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);
}

void open_mvR()
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);
}

void close_mvR()
{
	color = 0;
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);
}

void UART7_IRQHandler(void)
{
	static uint8_t temp=0;
	static uint8_t flag=0;
//	if(__HAL_UART_GET_FLAG(&mv,UART_FLAG_RXNE)!=0){
		temp=mv.Instance->RDR;
		
		if(flag==1)
		{
			if(temp == 1)
				color = 1;
			else if(temp == 2)
				color = 2;
			else if(temp == 3)
				color = 3;
			else if(temp == 4)
				color = 4;
			else
				color = 0;
			flag=0;
		}
		if(temp==0xFF && flag==0)
		{
			flag=1;
		}
//	}

	mv.Instance->ISR = 0;   //���SR��־λ
}

void USART6_IRQHandler(void)
{
	static uint8_t temp=0;
//	static uint8_t flag=0;
//	uint8_t temp_color=0;
	
	if(__HAL_UART_GET_FLAG(&mv_R,UART_FLAG_RXNE)!=0){
		temp = mv_R.Instance->RDR;
		
//		if(temp == 0xff && flag == 0)
//		{
//			flag = 1;
//		}
//		if(flag ==1 && temp != 0)
//		{
//			TreaPoint[0] = temp;
//			flag = 2;
//		}if(flag ==2 && temp != 0)
//		{
//			TreaPoint[1] = temp;
//			flag = 3;
//		}if(flag ==3 && temp != 0)
//		{
//			TreaPoint[2] = temp;
//			flag = 4;
//		}if(flag ==4 && temp != 0)
//		{
//			TreaPoint[3] = temp;
//			flag = 5;
//		}if(flag ==5 && temp != 0)
//		{
//			TreaPoint[4] = temp;
//			flag = 6;
//		}if(flag ==6 && temp != 0)
//		{
//			TreaPoint[5] = temp;
//			flag = 7;
//		}if(flag ==7 && temp != 0)
//		{
//			TreaPoint[6] = temp;
//			flag = 8;
//		}if(flag ==8 && temp != 0)
//		{
//			TreaPoint[7] = temp;
//		}
//		if(temp == 0)
//		{
//			for(uint8_t i = 0;i < 8;i++)
//				TreaPoint[i] = 0;
//			
//			flag = 0;
//		}
	}
	mv_R.Instance->ISR = 0;   //���SR��־λ
}
/*****************************************************************************
�������� bofang()
�������ܣ����Ÿ���
�βΣ���
*******************************************************************************/
void bofang()
{
	  uint8_t data[5]={0x7e,0x03,0x01,0x02,0xef};
	  HAL_UART_Transmit(&huart6,data,5,0xFFFF);
}
/*****************************************************************************
�������� stop_bofang()
�������ܣ�ֹͣ���Ÿ���
�βΣ���
*******************************************************************************/
void stop_bofang()
{
	uint8_t data[5]={0x7e,0x03,0x02,0x01,0xef};
	HAL_UART_Transmit(&huart6,data,5,0xFFFF);
}
//����ָ������
void bofang_zhiding(int shou)
{
	//7E 05 41 00(������λ) 01(������λ) 45(У���) EF
	  uint8_t data[7]={0x7e,0x05,0x41,0x00,0x00,0x00,0xef};
	  data[4]=shou;
	  uint8_t sum=data[1]^data[2]^data[3]^data[4];
	  data[5]=sum;
//	   HAL_UART_Transmit(&huart3,data,7,0xFFFF);
//	   while(__HAL_UART_GET_FLAG(&huart3,USART_ISR_TXE)==0)
//	   {
//		   vTaskDelay(2);
//	   }
	  for(uint8_t i=0;i<7;i++)
	  {
		  HAL_UART_Transmit(&huart6,&data[i],1,0xFFFF);
	  }
	  
}

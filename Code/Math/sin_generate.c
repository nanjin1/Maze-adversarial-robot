#include "sin_generate.h"
#include "math.h"

#define PI 3.1415926
TIM_HandleTypeDef TIM5_Handler;      //��ʱ����� 
struct sin_param sin1={0,0,1000,0.15};
float sin_generator(struct sin_param *param)
{
	float output;
	
	param->actual_t = param->time * param->angular_velocity;
	
	output = param->gain * sin(param->actual_t * PI/180);
	
	++param->time;
	
	if (param->actual_t >= 360)
		param->time = 0;
	
	return output;
}
void Timer5_Init()
{	//��ʱ��5 1ms
   __HAL_RCC_TIM5_CLK_ENABLE();
     
    TIM5_Handler.Instance=TIM5;                          //ͨ�ö�ʱ��4
    TIM5_Handler.Init.Prescaler=215;                     //��Ƶ
    TIM5_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;    //���ϼ�����
    TIM5_Handler.Init.Period=999;                        //�Զ�װ��ֵ
    TIM5_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_Base_Init(&TIM5_Handler);
		HAL_NVIC_SetPriority(TIM5_IRQn,0,0);    //�����ж����ȼ�����ռ���ȼ�3�������ȼ�3
    HAL_NVIC_EnableIRQ(TIM5_IRQn);          //����ITM4�ж�  
    HAL_TIM_Base_Start_IT(&TIM5_Handler); //ʹ�ܶ�ʱ��4�Ͷ�ʱ��4�ж� 
}

void TIM5_IRQHandler(void)
{ 	
    if(__HAL_TIM_GET_IT_SOURCE(&TIM5_Handler,TIM_IT_UPDATE)==SET)//����ж�
    {
		sin1.time++;
    }
    __HAL_TIM_CLEAR_IT(&TIM5_Handler, TIM_IT_UPDATE);//����жϱ�־λ
}

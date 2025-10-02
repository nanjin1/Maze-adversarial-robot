#ifndef __SCANER_H
#define __SCANER_H
#include "sys.h"
#include "pid.h"
#define Lamp_Max 8   //ѭ���������
#define is_Front 0
#define is_Back  1
typedef struct scaner	
{
	uint32_t detail;  //�����Ƶ�����
	float error;			//���
	u8 ledNum;				//�Ƶ�����
	u8 lineNum;      //linenum������¼�ж�����������
}SCANER;

#define LFB_SENSOR_NUM 8   //ѭ���崫��������
struct Scaner_Set {
	float CatchsensorNum;   //Ŀ��λ��
	int8_t EdgeIgnore;			//���Ե�
};

extern uint32_t data_head;
extern uint32_t data_tail;

extern volatile struct Scaner_Set scaner_set;
extern uint8_t ScanMode;
extern volatile SCANER Scaner;
extern const float line_weight[8];
void Go_Line(float speed);
void get_detail(void);
uint8_t Line_Scan(SCANER *scaner, unsigned char sensorNum, int8_t edge_ignore);
void actions(uint8_t action);
uint8_t getline_error(void);


#define GPIO_Left1 GPIOD
#define GPIO_Left1_P GPIO_PIN_0
#define GPIO_Left2 GPIOD
#define GPIO_Left2_P GPIO_PIN_1
#define GPIO_Left3 GPIOD
#define GPIO_Left3_P GPIO_PIN_2
#define GPIO_Left4 GPIOD
#define GPIO_Left4_P GPIO_PIN_3
#define GPIO_Left5 GPIOD
#define GPIO_Left5_P GPIO_PIN_4
#define GPIO_Left6 GPIOD
#define GPIO_Left6_P GPIO_PIN_7
#define GPIO_Left7 GPIOB
#define GPIO_Left7_P GPIO_PIN_3
#define GPIO_Left8 GPIOB
#define GPIO_Left8_P GPIO_PIN_4


#define GPIO_Right1 GPIOE
#define GPIO_Right1_P GPIO_PIN_6
#define GPIO_Right2 GPIOE
#define GPIO_Right2_P GPIO_PIN_5
#define GPIO_Right3 GPIOE
#define GPIO_Right3_P GPIO_PIN_4
#define GPIO_Right4 GPIOE
#define GPIO_Right4_P GPIO_PIN_3
#define GPIO_Right5 GPIOD
#define GPIO_Right5_P GPIO_PIN_10
#define GPIO_Right6 GPIOB
#define GPIO_Right6_P GPIO_PIN_7
#define GPIO_Right7 GPIOB
#define GPIO_Right7_P GPIO_PIN_6
#define GPIO_Right8 GPIOB
#define GPIO_Right8_P GPIO_PIN_5

//#define GPIO_Right1 GPIOB
//#define GPIO_Right1_P GPIO_PIN_5
//#define GPIO_Right2 GPIOB
//#define GPIO_Right2_P GPIO_PIN_6
//#define GPIO_Right3 GPIOB
//#define GPIO_Right3_P GPIO_PIN_7
//#define GPIO_Right4 GPIOE
//#define GPIO_Right4_P GPIO_PIN_2
//#define GPIO_Right5 GPIOE
//#define GPIO_Right5_P GPIO_PIN_3
//#define GPIO_Right6 GPIOE
//#define GPIO_Right6_P GPIO_PIN_4
//#define GPIO_Right7 GPIOE
//#define GPIO_Right7_P GPIO_PIN_5
//#define GPIO_Right8 GPIOE
//#define GPIO_Right8_P GPIO_PIN_6
#endif

#ifndef __MAP_H
#define __MAP_H

#include "map_message.h"

#define NO      	(1<<0) 
#define DLEFT 		(1<<1)				//��ߺ���
#define DRIGHT 		(1<<2)				//�ұߺ���   �Ұ����
#define RESTMPUZ	(1<<9)			    //������У׼
#define STOPTURN 	(1<<10)				//ͣ����ת��
#define SLOWDOWN	(1<<11)    			//����
#define Corner      (1<<12)             //�սǱ��ص�
#define In_Road     (1<<13)             //·�м䱦�ص�

enum barriers {
	Treasure = 2,
	Platform,
};

enum MapNode {
	A1,A2,L1,A3,A4,A5,L2,A6,A7,P1,          		//9
	L3,B1,A8,A9,A10,A11,A61,L4,A12,A13,     		//19
	B3,A14,A15,A16,B4,L5,A17,B5,A18,A19, 		//29
	A20,A21,L6,A22,L7,B6,A23,A24,A25,B7,   		 //39
	L8,B8,A26,A27,A28,A29,L9,A30,           		//47
	A31,L10,A32,A33,A34,A35,B9,L11,          		 //55
	B10,A36,A37,A38,B11,L12,A39,L13,A40,A41, 	   //65
	A42,A43,B12,A44,L14,B13,A45,A46,A47,B14,  //75
	A48,A49,L15,A62,A50,A51,A52,A53,B16,L16,      //85
	P2,A54,A55,L17,A56,A57,A58,L18,A59,A60,        //95
	A63
};

/**************************************/
//�����Ϣ
//flag 0λѰ�߷�ʽ��0��Ѱ�ߣ�1��Ѱ��
//flag 123λ����·�ڱ�־��	000����ߴ򵽣�001���ұߴ򵽣�010������ߣ�011�ұ����ߣ�100�����ɶ���һ��	
//flag 45λ��������Ŀ	
//flag 6λ��Ѱ�߷�ʽ�Ƿ�Ҫ�л���1��Ҫ�л���0����Ҫ�л�
//flag 7λ	��Ҫ������У��
//flag 8~11	
typedef struct _node{
	u8 nodenum;     //�������
	u32  flag;	    //����־λ
	float angle;	//�Ƕ�	
	u16	step;		//�߳�
	float speed;	//Ѱ���ٶ�
	u8 function;    //��㺯��
}NODE;

extern NODE Node[205];
/*************************/
//flag 0λ��1��������������0�������
//flag 1λ������·���ж�
//flag 2λ���Ƿ񵽴�·��
//flag 3λ��arrive��temp����
//flag 4λ��Z������
//flag 5λ��·�ߴ���λ ����
//flag 6λ��û����
//flag 7λ�����
typedef struct _nodesr{
	u8 flag;
	NODE nowNode;	//��ǰ���
	NODE nextNode;	//��һ���
	NODE lastNode;  //��һ�����
}NODESR;

extern NODESR nodesr;

struct Map_State {
	u16 point;
	u8 routetime;//�ڼ����ܵ�ͼ
};
extern struct Map_State map;
extern int route[800];
u8 getNextConnectNode(u8 nownode,u8 nextnode);
void mapInit(void);
void Cross(void);
void map_function(u8 fun);
u8 deal_arrive(void);


#endif








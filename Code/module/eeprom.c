//#include "iic.h"
//#include "eeprom.h"
//#include "stdio.h"
//uint8_t Send_Byte(uint16_t address,uint8_t pBuffer)
//{
//	IIC_Start();
//	IIC_Send_Byte(Address_Write);
//	if(IIC_Wait_Ack())
//	{
//		IIC_Stop();
//		return 0;
//	}
//	
//	IIC_Send_Byte(address>>8);
//	if(IIC_Wait_Ack())
//	{
//		IIC_Stop();
//		return 0;
//	}
//	
//	IIC_Send_Byte((uint8_t)(address&0xFF));
//	if(IIC_Wait_Ack())
//	{
//		IIC_Stop();
//		return 0;
//	}
//	
//	IIC_Send_Byte(pBuffer);
//	if(IIC_Wait_Ack())
//	{
//		IIC_Stop();
//		return 0;
//	}
//	
//	IIC_Stop();
//	delay_ms(50);
//	return 1;
//}	

//uint8_t Read_Byte(uint16_t address,uint8_t *data)
//{
//	EEPROM_wait();
//	IIC_Start();
//	IIC_Send_Byte(Address_Write);
//	if(IIC_Wait_Ack())
//	{
//		IIC_Stop();
//		return 0;
//	}
//	
//	IIC_Send_Byte(address>>8);
//	if(IIC_Wait_Ack())
//	{
//		IIC_Stop();
//		return 0;
//	}
//	
//	IIC_Send_Byte((uint8_t)(address&0xFF));
//	if(IIC_Wait_Ack())
//	{
//		IIC_Stop();
//		return 0;
//	}
//	IIC_Start();
//	IIC_Send_Byte(Address_Read);
//	if(IIC_Wait_Ack())
//	{
//		IIC_Stop();
//		return 0;
//	}
//	
//	*data = IIC_Read_Byte(0);
//	
//	IIC_Stop();
//	return 1;
//}	

////����ֵΪ1���ȴ���ʱ������ֵΪ0������
//uint8_t EEPROM_wait(void)//�ȴ�EEPROM�ڲ�ʱ�����
//{
//	uint16_t cycle = 0;
//	
//	while(EEPROM_check_device())//EEPROM��Ӧ��
//	{
//		cycle++;
//		if(cycle>=10000) return 1;//�ȴ���ʱ
//	}
//	
//	return 0;//��ɵȴ�
//}

//uint8_t EEPROM_check_device(void)//���EEPROM�Ƿ����
//{
//	uint8_t result;
//	
//	IIC_Start();//������ʼ�ź�
//	IIC_Send_Byte(Address_Write);
//	
//	if(IIC_Wait_Ack())//�����Ӧ�ź�
//	{
//		result = 1; 
//	}		//��ʾEEPROM�����ڣ���Ӧ��
//	else
//	{
//		result = 0;
//	}		//��ʾEEPROM����
//	
//	IIC_NAck();//����Ƕ�ģʽ�£���Ҫ�����Լ���EEPROM����һ����Ӧ���źŲ���ֹͣ
//	IIC_Stop();
//	
//	return result;
//}

///************************************************
//eeprom һ�������д��˸��ֽڣ�ÿ�˸��ֽ�Ϊһҳ
//size:Ҫд������ֽ�
//************************************************/
////����ֵΪ1��д������������ֵΪ0��д�����
//uint8_t EEPROM_write_bytes(uint16_t address,uint8_t *data,uint16_t size)
//{
//	uint16_t i;
// 
//	for(i = 0;i < size;i++)
//	{
//		if(i == 0 || address%8 == 0)//��һ�λ���ÿ�˴Σ�i�Ǵ�0-7��
//		//���õ�ַ����ķ�����ÿ8���ֽڣ�0-7��һҳ����ÿһҳ�Ŀ�ͷ������ʼ�ź�
//		{
//			/*ÿд��һҳ�Ľ������ȷ���һ��stop�źţ�����ǰ���д�����
//			����ǵ�һ��д��Ҳ��Ӱ�죬��Ϊ�����еȴ�EEPROM�ڲ�ʱ����ɵĳ���*/
//			IIC_Stop();
//			if(IIC_Wait_Ack() == 1)
//			{
//				IIC_Stop();
//				return 0;
//			}
//			
//			IIC_Start();//������ʼ�ź�
//			IIC_Send_Byte(Address_Write);
//			if(IIC_Wait_Ack())
//			{
//				IIC_Stop();
//				return 0;
//			}
//			IIC_Send_Byte(address>>8);
//			if(IIC_Wait_Ack())
//			{
//				IIC_Stop();
//				return 0;
//			}
//			
//			IIC_Send_Byte((uint8_t)(address&0xFF));
//			if(IIC_Wait_Ack())
//			{
//				IIC_Stop();
//				return 0;
//			}
//		}//�ڵ�һҳ����ÿд��8���ֽڶ�Ҫ��һ������ĳ���
// 
//		IIC_Send_Byte(*data);//�� EEPROMд�����ݣ�һ���ֽ�һ���ֽڵ�����
//		if(IIC_Wait_Ack() == 1)
//		{
//			IIC_Stop();
//			return 0;
//		}
//		data++;
//		address++;
//	}
// 
//	/*ȫ���������֮��*/
//	IIC_Stop();
//	if(IIC_Wait_Ack() == 1)
//	{
//		IIC_Stop();
//		return 0;
//	}
//	return 1;
//}


////����ֵΪ1����ȡ����������ֵΪ0����ȡ����
//uint8_t EEPROM_read_bytes(uint16_t address,uint8_t *data,uint16_t size)//��EEPROM�����ȡn���ֽ�
//{
//	IIC_Start(); //���͵�һ����ʼ�ź�
//	IIC_Send_Byte(Address_Write);
//	if(IIC_Wait_Ack())
//	{
//		IIC_Stop();
//		return 0;
//	}
//	
//	IIC_Send_Byte(address>>8);
//	if(IIC_Wait_Ack())
//	{
//		IIC_Stop();
//		return 0;
//	}
//	
//	IIC_Send_Byte((uint8_t)(address&0xFF));
//	if(IIC_Wait_Ack())
//	{
//		IIC_Stop();
//		return 0;
//	}
//	
//	IIC_Start(); //���͵ڶ�����ʼ�ź�
//		
//	IIC_Send_Byte(Address_Read);
//	if(IIC_Wait_Ack())
//	{
//		IIC_Stop();
//		return 0;
//	}
//	
//	uint16_t i;
//	for(i = 0;i < size;i++)
//	{
//		if(i == size - 1)
//		{
//			*data = IIC_Read_Byte(0);
//		}//���ݽ��չ���
//		else
//		{
//			*data = IIC_Read_Byte(1);
//		}
//		data++;//ָ��ָ����һ������
//	}
//			
//	IIC_Stop();	
//	return 1;
//}

#include "iic.h"
#include "eeprom.h"

uint8_t Send_Byte(uint16_t address,uint8_t pBuffer)
{
	IIC_Start();
	IIC_Send_Byte(Address_Write);
	if(IIC_Wait_Ack())
	{
		IIC_Stop();
		return 0;
	}
	
	IIC_Send_Byte(address>>8);
	if(IIC_Wait_Ack())
	{
		IIC_Stop();
		return 0;
	}
	
	IIC_Send_Byte((uint8_t)(address&0xFF));
	if(IIC_Wait_Ack())
	{
		IIC_Stop();
		return 0;
	}
	
	IIC_Send_Byte(pBuffer);
	if(IIC_Wait_Ack())
	{
		IIC_Stop();
		return 0;
	}
	
	IIC_Stop();
	delay_ms(50);
	return 1;
}	

uint8_t Read_Byte(uint16_t address,uint8_t *data)
{
	EEPROM_wait();
	
	IIC_Start();
	IIC_Send_Byte(Address_Write);
	if(IIC_Wait_Ack())
	{
		IIC_Stop();
		return 5;
	}
	
	IIC_Send_Byte(address>>8);
	if(IIC_Wait_Ack())
	{
		IIC_Stop();
		return 0;
	}
	
	IIC_Send_Byte((uint8_t)(address&0xFF));
	if(IIC_Wait_Ack())
	{
		IIC_Stop();
		return 0;
	}
	IIC_Start();
	IIC_Send_Byte(Address_Read);
	if(IIC_Wait_Ack())
	{
		IIC_Stop();
		return 0;
	}
	
	*data = IIC_Read_Byte(0);
	
	IIC_Stop();
	return 1;
}	

uint8_t EEPROM_check_device(void)//���EEPROM�Ƿ����
{
	uint8_t result;
	
	IIC_Start();//������ʼ�ź�
	IIC_Send_Byte(Address_Write);
	
	if(IIC_Wait_Ack())//�����Ӧ�ź�
	{
		result = 1; 
	}		//��ʾEEPROM�����ڣ���Ӧ��
	else
	{
		result = 0;
	}		//��ʾEEPROM����
	
	IIC_NAck();//����Ƕ�ģʽ�£���Ҫ�����Լ���EEPROM����һ����Ӧ���źŲ���ֹͣ
	IIC_Stop();
	
	return result;
}

//����ֵΪ1���ȴ���ʱ������ֵΪ0������
uint8_t EEPROM_wait(void)//�ȴ�EEPROM�ڲ�ʱ�����
{
	uint16_t cycle = 0;
	
	while(EEPROM_check_device())//EEPROM��Ӧ��
	{
		cycle++;
		if(cycle>=10000) return 1;//�ȴ���ʱ
	}
	
	return 0;//��ɵȴ�
}

/************************************************
eeprom һ�������д��˸��ֽڣ�ÿ�˸��ֽ�Ϊһҳ
size:Ҫд������ֽ�
************************************************/
//����ֵΪ1��д������������ֵΪ0��д�����
uint8_t EEPROM_write_bytes(uint16_t address,uint8_t *data,uint16_t size)
{
	uint16_t i;
 
	for(i = 0;i < size;i++)
	{
		if(i == 0 || address%8 == 0)//��һ�λ���ÿ�˴Σ�i�Ǵ�0-7��
		//���õ�ַ����ķ�����ÿ8���ֽڣ�0-7��һҳ����ÿһҳ�Ŀ�ͷ������ʼ�ź�
		{
			/*ÿд��һҳ�Ľ������ȷ���һ��stop�źţ�����ǰ���д�����
			����ǵ�һ��д��Ҳ��Ӱ�죬��Ϊ�����еȴ�EEPROM�ڲ�ʱ����ɵĳ���*/
			IIC_Stop();
			if(IIC_Wait_Ack() == 1)
			{
				IIC_Stop();
				return 0;
			}
			
			IIC_Start();//������ʼ�ź�
			IIC_Send_Byte(Address_Write);
			if(IIC_Wait_Ack())
			{
				IIC_Stop();
				return 0;
			}
			IIC_Send_Byte(address>>8);
			if(IIC_Wait_Ack())
			{
				IIC_Stop();
				return 0;
			}
			
			IIC_Send_Byte((uint8_t)(address&0xFF));
			if(IIC_Wait_Ack())
			{
				IIC_Stop();
				return 0;
			}
		}//�ڵ�һҳ����ÿд��8���ֽڶ�Ҫ��һ������ĳ���
 
		IIC_Send_Byte(*data);//�� EEPROMд�����ݣ�һ���ֽ�һ���ֽڵ�����
		if(IIC_Wait_Ack() == 1)
		{
			IIC_Stop();
			return 0;
		}
		data++;
		address++;
	}
 
	/*ȫ���������֮��*/
	IIC_Stop();
	if(IIC_Wait_Ack() == 1)
	{
		IIC_Stop();
		return 0;
	}
	return 1;
}


//����ֵΪ1����ȡ����������ֵΪ0����ȡ����
uint8_t EEPROM_read_bytes(uint16_t address,uint8_t *data,uint16_t size)//��EEPROM�����ȡn���ֽ�
{
	IIC_Start(); //���͵�һ����ʼ�ź�
	IIC_Send_Byte(Address_Write);
	if(IIC_Wait_Ack())
	{
		IIC_Stop();
		return 0;
	}
	
	IIC_Send_Byte(address>>8);
	if(IIC_Wait_Ack())
	{
		IIC_Stop();
		return 0;
	}
	
	IIC_Send_Byte((uint8_t)(address&0xFF));
	if(IIC_Wait_Ack())
	{
		IIC_Stop();
		return 0;
	}
	
	IIC_Start(); //���͵ڶ�����ʼ�ź�
		
	IIC_Send_Byte(Address_Read);
	if(IIC_Wait_Ack())
	{
		IIC_Stop();
		return 0;
	}
	
	uint16_t i;
	for(i = 0;i < size;i++)
	{
		if(i == size - 1)
		{
			*data = IIC_Read_Byte(0);
		}//���ݽ��չ���
		else
		{
			*data = IIC_Read_Byte(1);
		}
		data++;//ָ��ָ����һ������
	}
			
	IIC_Stop();	
	return 1;
}

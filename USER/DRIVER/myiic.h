#ifndef __MYIIC_H
#define __MYIIC_H
#include "includes.h" 
 
//���Ŷ���
#define MS5837_PORT                GPIOB
#define MS5837_SCL_PIN             MS5837_SCL_PIN_Pin
#define MS5837_SDA_PIN             MS5837_SDA_PIN_Pin
//#define delay_us(x)                Tim6_DelaynUS(x) 
//IO��������
void SDA_In(void);
void SDA_Out(void);
 
//IO��������	
#define SCL(x)        (x?HAL_GPIO_WritePin(MS5837_PORT, MS5837_SCL_PIN, GPIO_PIN_SET):HAL_GPIO_WritePin(MS5837_PORT, MS5837_SCL_PIN, GPIO_PIN_RESET))
#define SDA(x)        (x?HAL_GPIO_WritePin(MS5837_PORT, MS5837_SDA_PIN, GPIO_PIN_SET):HAL_GPIO_WritePin(MS5837_PORT, MS5837_SDA_PIN, GPIO_PIN_RESET))
#define READ_SDA()     HAL_GPIO_ReadPin(MS5837_PORT,MS5837_SDA_PIN)
 
//IIC���в�������
void IIC_Init(void);                //��ʼ��IIC��IO��				 
void IIC_Start(void);				        //����IIC��ʼ�ź�
void IIC_Stop(void);	  			      //����IICֹͣ�ź�
void IIC_Send_Byte(u8 txd);			    //IIC����һ���ֽ�
u8 IIC_Read_Byte(unsigned char ack);//IIC��ȡһ���ֽ�
u8 IIC_Wait_Ack(void); 				      //IIC�ȴ�ACK�ź�
void IIC_Ack(void);					        //IIC����ACK�ź�
void IIC_NAck(void);				        //IIC������ACK�ź�

void delay_sys_us(uint32_t Delay);
void IIC_Write_One_Byte(u8 daddr,u8 addr,u8 data);
u8 IIC_Read_One_Byte(u8 daddr,u8 addr);	  
#endif

















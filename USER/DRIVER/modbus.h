#ifndef __PRESSURE_H
#define __PRESSURE_H

#include "usart.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>

#define AMMONIA_RX_LEN 256
#define RE(val) val ? HAL_GPIO_WritePin(USART3_RE_GPIO_Port, USART3_RE_Pin, GPIO_PIN_SET) : HAL_GPIO_WritePin(USART3_RE_GPIO_Port, USART3_RE_Pin, GPIO_PIN_RESET)

typedef unsigned char u8;
typedef unsigned short u16;
typedef unsigned int u32;

#define AMMONIA_USART huart3

typedef struct
{
	u8 RX_flag : 1;				 //IDLE receive flag
	u16 RX_Size;				 //receive length
	u8 RX_pData[AMMONIA_RX_LEN]; //DMA receive buffer
	u8 RealBuff[AMMONIA_RX_LEN];
} Ammonia_UsartRec_t;

typedef struct
{
	float ammonia_val;
	float ammonia_t; //设备1温度
	u8 valid_1;		 //设备1数据有效标志

} AmmoniaMsg_t;
void ammonia_Init(void);
void uart_send(u8 *buf, u16 len);
void Pressure_GetData(AmmoniaMsg_t *pressure); //imu 数据读取函数
void HAL_USART3_Receive_IDLE(void);			   //空闲接收中断
void Pressure_Test_Demo(void);				   //imu 测试demo

extern AmmoniaMsg_t ammonia_msg;

extern Ammonia_UsartRec_t ammonia_rec;

#endif

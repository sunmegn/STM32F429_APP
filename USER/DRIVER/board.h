#ifndef __BOARD_H
#define __BOARD_H

#include "includes.h"

 
#ifdef FREERTOS_SYS
	#define GET_TICK_COUNT()    xTaskGetTickCount()     
#else
	#define GET_TICK_COUNT()    HAL_GetTick()
#endif

#define  MS5837_IIC   	 	hi2c1
#define  SHT35_IIC     		hi2c3
#define  MCP3221_IIC   		hi2c3
#define  PCA9685_IIC   		hi2c2
#define  R6093U_USART  
#define  BAT_ADC       		hadc1
#define  W5500_SPI     		hspi1
#define  YunTai_TIM    		htim8
#define  YunTai_TIM_CH    TIM_CHANNEL_3
 
#define  LED_TIM       		htim4
#define  LED_PWM_CH1      TIM_CHANNEL_1
#define  LED_PWM_CH2      TIM_CHANNEL_2

extern u32 use_time;
void delay_ms(int nms);
void Task_Queue_Semaphore_Timers_Create(void);//消息队列创建
void BSP_Init(void);
u32 Reflash(void);//执行周期 

void USER_UART_IRQHandler(UART_HandleTypeDef *huart);

extern u32 run_time[6];
u32 getRunTime(u8 n);
#endif

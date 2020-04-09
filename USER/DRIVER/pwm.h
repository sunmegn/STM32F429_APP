#ifndef __PWM_H
#define __PWM_H

#include "includes.h"
#define   SERVO_INIT        1450 // 1050(��� ������) :  1750(��� ������)          1587 = X  
#define 	YunTai_MID    		           1500        //��̨��ʼֵ
#define 	LED_MID                      0           //LED̽�յƳ�ʼֵ 

#define	  YunTai_SetPwm(x)               __HAL_TIM_SET_COMPARE(&YunTai_TIM,YunTai_TIM_CH,x); 
#define	  LED_SetPwm(x)            do{__HAL_TIM_SET_COMPARE(&LED_TIM,LED_PWM_CH1,x);__HAL_TIM_SET_COMPARE(&LED_TIM,LED_PWM_CH2,x);}while(0);

void PWM_Init(void);
void PWM_Test_Demo(void);
#endif

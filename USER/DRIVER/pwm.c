#include "pwm.h"
//����ͷʹ��
void PWM_ServoInit(void)
{
//	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
//	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,SERVO_INIT);
}
int GetAngle = 0;
void SetCameraAngle(int value)
{
//	GetAngle = value;
//	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,(GetAngle-50)*6+SERVO_INIT);
}
void PWM_Init(void)
{
	HAL_TIM_PWM_Start(&YunTai_TIM,YunTai_TIM_CH);
	__HAL_TIM_SET_COMPARE(&YunTai_TIM,YunTai_TIM_CH,1200);//�����̨
	
	HAL_TIM_PWM_Start(&LED_TIM,LED_PWM_CH1);         //����
	__HAL_TIM_SET_COMPARE(&LED_TIM,LED_PWM_CH1,0); 
	
	HAL_TIM_PWM_Start(&LED_TIM,LED_PWM_CH2);         //�Ҵ��
	__HAL_TIM_SET_COMPARE(&LED_TIM,LED_PWM_CH2,0);
	
}



void PWM_Test_Demo(void)
{
	PWM_Init();
	
	while(1)
	{

		delay_ms(500);
	}
	
}
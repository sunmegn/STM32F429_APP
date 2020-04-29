#ifndef __SONAR852TASK_H
#define __SONAR852TASK_H

#include "includes.h"
#include "stm32f4xx_hal.h"
typedef struct{
	uint8_t flag;
	uint16_t Rx_Size;
	uint8_t Sonar852_Body[600];
	uint8_t Sonar852_Header[30];
}sonar852_Typedef;
	
void Sonar852Task_Function(void const * argument);
void sonar852_sendHeader(uint8_t *buf, uint16_t len); //��852���ŷ�����λ����������
int sonar852_send(uint8_t *combuf, uint16_t len);         //����852��������

#endif

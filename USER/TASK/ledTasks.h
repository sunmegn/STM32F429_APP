#ifndef __LEDTASKS_H
#define __LEDTASKS_H

#include "includes.h"
#define BUZZER(x)		(x?HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET):HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET))


void ledTask_Function(void const * argument);

#endif


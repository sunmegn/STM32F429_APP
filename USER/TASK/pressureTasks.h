#ifndef __PRESSURETASKS_H
#define __PRESSURETASKS_H

#include "includes.h"
extern QueueHandle_t Pressure_Message_Queue;

void       pressureTask_Function(void const *argument);
extern u32 Reflash(void);
extern u32 delta;

#endif

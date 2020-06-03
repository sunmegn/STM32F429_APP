#ifndef __HOLDERTASK_H
#define __HOLDERTASK_H

#include "main.h"
#include "global.h"
#include "iwdg.h"

#include "ZEROHolder.h"

typedef struct
{
    short    setpos;
    float    getpos;
    uint32_t nowcnt;
    uint8_t  SW;
    uint8_t  CW;

    uint8_t vaild;

    char TimeTestF;
} HolderParam_t;

typedef struct
{
    uint32_t midpos;
    int16_t  maxang;
    int16_t  minang;
} HolderPosParam_t;

void    HolderTask_Function(void const *argument);
void    HolderE2CheckInit(void);
uint8_t HoldePos_Init(void);
uint8_t HolderPosERRCheak(float setpos, float readpos, float val, int times);
float   ValToAngle(int16_t val, int16_t maxang, int16_t minang);

extern QueueHandle_t HolderBackQueue;

#endif

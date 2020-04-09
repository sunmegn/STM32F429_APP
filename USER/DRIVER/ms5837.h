#ifndef __MS5837_H
#define __MS5837_H

#include "includes.h"
#include "myiic.h"
/* 函数申明 -----------------------------------------------*/

#define MS583703BA_ADC_RD          0x00
#define	MS583703BA_PROM_RD 	       0xA0
#define MS583703BA_PROM_CRC        0xAE

#define MS583703BA_SlaveAddress    0xEE  //MS5387的地址
#define MS583703BA_RST             0x1E  //cmd 复位

#define	MS583703BA_D2_OSR_4096   	 0x5A	// 9.04 mSec conversion time ( 110.62 Hz)
#define	MS583703BA_D1_OSR_4096   	 0x4A

#define MS5837_OSR256					 		 0x40
#define MS5837_OSR512					 		 0x42
#define MS5837_OSR1024					   0x44
#define MS5837_OSR2048					   0x46
#define MS5837_OSR4096					   0x48
#define FILTER_num 20
//压传
typedef struct
{
	float Temperature;
	int32_t Pressure;
	int32_t init_value;
	float depth;
}PressureMsg_t;
float Get_High(void);
void Filter_Hight(unsigned int set_hight);
void Hight_PwmOut(void);
void MS583703BA_getPressure(void);
void MS583703BA_getTemperature(void);
void MS583703BA_RESET(void);

void MS5837_TasksFunction(void);

unsigned long MS583703BA_getConversion(uint8_t command);
void MS583703BA_GetTemperature(void);
void MS8537_Test_Demo(void);
void MS5837_Init(void);
void MS5837_GetData(PressureMsg_t *pressure);
#endif

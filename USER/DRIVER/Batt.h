#ifndef __BATT_H
#define __BATT_H

#include "includes.h"
//µÁ¡ø
typedef struct 
{
    float current;
    float voltage;
}PowerMsg_t;
 
#define BAT_V_FILTER_N     ( 10 )
#define BAT_C_FILTER_N     ( 100 )
#define BAT_V_CHANNEL      (  0  )
#define BAT_C_CHANNEL      (  1  )

void Batt_Init(void);
void Batt_Test_Demo(void);
void Get_BattData(PowerMsg_t *BattStruct); 


#endif

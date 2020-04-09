#ifndef __W5500_NET_H
#define __W5500_NET_H

#include "includes.h"
 
void W5500_Init(void);
void W5500_Test_Demo(void);
void SendToPC(uint8_t id,uint8_t* data,uint8_t len);

#endif

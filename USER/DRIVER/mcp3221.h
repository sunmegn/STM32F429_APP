#ifndef __MCP3221_H
#define __MCP3221_H

#include "includes.h"

//#define  MCP3221_IIC   hi2c3  
float MCP3221_GetVoltage(void);
void MCP3221_Test_Demo(void);
#define MCP3221_WRITE_ADDR     ( 0x9A  )
#define MCP3221_READ_ADDR      ( 0x9A+1 )
#endif

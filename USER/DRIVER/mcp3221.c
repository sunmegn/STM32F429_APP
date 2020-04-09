#include "mcp3221.h"
/**
  ******************************************************************************
  * @file    mcp3221.c
  * @author  LiuYang
  * @brief   IIC ∂¡»°µÁ—π
  *
  *
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */

void MCP3221_StartConverVoltage(void)
{
	
	
}
u8 MCP3221_COMM_Buffer[2]={0};
u8 MCP3221_Data_Buffer[2]={0};
float valtale = 0;
u16 voltage_raw_data=0;
float MCP3221_GetVoltage(void)
{
	float temp = 0;
	HAL_I2C_Master_Receive(&MCP3221_IIC,MCP3221_READ_ADDR,MCP3221_Data_Buffer,2,0x10); 
	voltage_raw_data = ((u16)MCP3221_Data_Buffer[0]<<8)+MCP3221_Data_Buffer[1];
//	temp = 0.0012210012210012 * voltage_raw_data*5.4;
	temp =  voltage_raw_data*5.0f/4096*5.4f;
	return temp;
}

void MCP3221_Test_Demo(void)
{
	
	while(1)
	{
		valtale = MCP3221_GetVoltage();
		delay_ms(300);
	}
	
}

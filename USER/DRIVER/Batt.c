#include "Batt.h"

#define BATT_BUFFER_SIZE 50
uint32_t ADC_Value[10];
uint32_t BatCurrentBuf[BAT_C_FILTER_N + 1] = {0};
float    BatVoltageBuf[BAT_V_FILTER_N + 1] = {0};
/**********************************************************************************************************
*函 数 名: GetBattInit
*功能说明: 电压电流值采样
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void Batt_Init(void)
{
    int i = 0;
    HAL_ADC_Start_DMA(&BAT_ADC, (uint32_t *)&ADC_Value, 10); //电流值采样

    for (i = 0; i < BAT_V_FILTER_N + 1; i++) //电压值采样
    {
        BatVoltageBuf[i] = MCP3221_GetVoltage();
        delay_ms(1);
    }

    //	for(i = 0; i < BAT_C_FILTER_N + 1; i++)
    //	{
    //			BatCurrentBuf[i] = ADC_Value[BAT_C_CHANNEL];
    //			delay_ms(1);
    //	}
}
/**********************************************************************************************************
*函 数 名: Voltage_RecursionFilter
*功能说明: 电压ADC采集原始数据均值滤波
*形    参: uint32_t *filterbuf      uint32_t ADValue 
*              电压总数组               新采集的数据  
*              
*返 回 值: 均值滤波后的电压原始值
**********************************************************************************************************/
static float Voltage_RecursionFilter(float *filterbuf, float input_vol)
{
    float filtersum           = 0;
    int   i                   = 0;
    filterbuf[BAT_V_FILTER_N] = input_vol;
    for (i = 0; i < BAT_V_FILTER_N; i++)
    {
        filterbuf[i] = filterbuf[i + 1];
        filtersum += filterbuf[i];
    }
    return (filtersum / BAT_V_FILTER_N);
}
/**********************************************************************************************************
*函 数 名: Current_RecursionFilter
*功能说明: 电流ADC采集原始数据均值滤波
*形    参: uint32_t *filterbuf      uint32_t ADValue 
*             电流总数组               新采集的数据  
*              
*返 回 值: 均值滤波后的电流原始值
**********************************************************************************************************/
static uint32_t Current_RecursionFilter(uint32_t *filterbuf, uint32_t ADValue)
{
    uint32_t filtersum        = 0;
    int      i                = 0;
    filterbuf[BAT_C_FILTER_N] = ADValue;
    for (i = 0; i < BAT_C_FILTER_N; i++)
    {
        filterbuf[i] = filterbuf[i + 1]; //
        filtersum += filterbuf[i];
    }
    return (int)(filtersum / BAT_C_FILTER_N);
}
float adcVoltage = 0;

/**********************************************************************************************************
*函 数 名: GetVoltage
*功能说明: 获取电池电压  (  110k   
*形    参: 无                10K       11:1   
*返 回 值: 电压值        )
**********************************************************************************************************/
float GetVoltage(void)
{
    adcVoltage = MCP3221_GetVoltage();
    return Voltage_RecursionFilter(BatVoltageBuf, adcVoltage);
}
/**********************************************************************************************************
*函 数 名: GetCurrent
*功能说明: 对电流值进行均值滤波   ( ACS758KCB-150B-PFF-T   ±150A     13.3mV/A    5V供电 )
*形    参: 无
*返 回 值: 电流值
**********************************************************************************************************/
uint32_t adc_sum    = 0;
float    adc_offset = 0.5f;
float    GetCurrent(void) //电流值均值滤波
{
    float temp;
    adc_sum = 0;
    for (int i = 0; i < 10; i++)
    {
        adc_sum += ADC_Value[i];
    }
    adc_sum = adc_sum / 10;
    temp    = (2.5 - (adc_sum * 3.3f / 4096)) * 1000 / 13.3 + adc_offset;
    return temp;
}
/**********************************************************************************************************
*函 数 名: Get_BattData
*功能说明: 传递电压电流结构体
*形    参: PowerMsg_t 
*返 回 值: 无 
**********************************************************************************************************/
void Get_BattData(PowerMsg_t *BattStruct)
{
    BattStruct->voltage = GetVoltage();
    BattStruct->current = GetCurrent();
}

float      vol = 0.0, cur = 0.0;
PowerMsg_t test_power;
void       Batt_Test_Demo(void)
{
    Batt_Init();
    while (1)
    {
        Get_BattData(&test_power);
        delay_ms(500);
    }
}
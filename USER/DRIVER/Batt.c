#include "Batt.h"

#define BATT_BUFFER_SIZE 50
uint32_t ADC_Value[10];
uint32_t BatCurrentBuf[BAT_C_FILTER_N + 1] = {0};
float    BatVoltageBuf[BAT_V_FILTER_N + 1] = {0};
/**********************************************************************************************************
*�� �� ��: GetBattInit
*����˵��: ��ѹ����ֵ����
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void Batt_Init(void)
{
    int i = 0;
    HAL_ADC_Start_DMA(&BAT_ADC, (uint32_t *)&ADC_Value, 10); //����ֵ����

    for (i = 0; i < BAT_V_FILTER_N + 1; i++) //��ѹֵ����
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
*�� �� ��: Voltage_RecursionFilter
*����˵��: ��ѹADC�ɼ�ԭʼ���ݾ�ֵ�˲�
*��    ��: uint32_t *filterbuf      uint32_t ADValue 
*              ��ѹ������               �²ɼ�������  
*              
*�� �� ֵ: ��ֵ�˲���ĵ�ѹԭʼֵ
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
*�� �� ��: Current_RecursionFilter
*����˵��: ����ADC�ɼ�ԭʼ���ݾ�ֵ�˲�
*��    ��: uint32_t *filterbuf      uint32_t ADValue 
*             ����������               �²ɼ�������  
*              
*�� �� ֵ: ��ֵ�˲���ĵ���ԭʼֵ
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
*�� �� ��: GetVoltage
*����˵��: ��ȡ��ص�ѹ  (  110k   
*��    ��: ��                10K       11:1   
*�� �� ֵ: ��ѹֵ        )
**********************************************************************************************************/
float GetVoltage(void)
{
    adcVoltage = MCP3221_GetVoltage();
    return Voltage_RecursionFilter(BatVoltageBuf, adcVoltage);
}
/**********************************************************************************************************
*�� �� ��: GetCurrent
*����˵��: �Ե���ֵ���о�ֵ�˲�   ( ACS758KCB-150B-PFF-T   ��150A     13.3mV/A    5V���� )
*��    ��: ��
*�� �� ֵ: ����ֵ
**********************************************************************************************************/
uint32_t adc_sum    = 0;
float    adc_offset = 0.5f;
float    GetCurrent(void) //����ֵ��ֵ�˲�
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
*�� �� ��: Get_BattData
*����˵��: ���ݵ�ѹ�����ṹ��
*��    ��: PowerMsg_t 
*�� �� ֵ: �� 
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
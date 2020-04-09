/***
 * @author        : robosea
 * @version       : v1.0.0
 * @Date          : 2019-12-16 11:15:48
 * @LastEditors   : Robosea
 * @LastEditTime  : 2020-02-21 00:42:10
 * @FilePath      : \ROV_F429_APP-10-10\USER\TASK\sensorTasks.c
 * @brief         : 
 */
#include "sensorTasks.h"
#include "mcp3221.h"
TempHumiMsg_t TempHum_raw;
PowerMsg_t    power_data_raw;
extern float  valtale;

/***
 * @function_name: 
 * @brief: 
 * @param {type} 
 * @return: None
 */
void SensorTask_Function(void const *argument)
{

    portTickType tick = xTaskGetTickCount();
    while (1)
    {
        run_time[3] = getRunTime(3);
#ifdef DEBUG

#else
        SHT35_ReadData(&TempHum_raw);
        Get_BattData(&power_data_raw);

        xQueueOverwrite(TempHum_Message_Queue, &TempHum_raw);
        xQueueOverwrite(Battery_Message_Queue, &power_data_raw);
#endif
        osDelay(1000);
    }
}

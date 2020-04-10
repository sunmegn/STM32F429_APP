/**
 * @author        :robosea
 * @version       :v1.0.0
 * @Date          :2019-12-16 11:15:48
 * @LastEditors:Robosea
 * @LastEditTime:2020-04-09 20:05:07
 * @FilePath      :\ROV_F429_APP-10-10\USER\TASK\pressureTasks.c
 * @brief         :压力控制任务函数
 */
#include "pressureTasks.h"

PressureMsg_t pressure_raw;

/**
 * @function_name: pressureTask_Function
 * @brief:压力控制任务函数
 * @param: 任务句柄
 * @return:None
 */
void pressureTask_Function(void const *argument)
{
    portTickType tick = xTaskGetTickCount();
    while (1)
    {
        run_time[2] = getRunTime(2);
#ifdef DEBUG

#else
        MS5837_GetData(&pressure_raw);
        xQueueOverwrite(Pressure_Message_Queue, &pressure_raw);
#endif
        //		osDelay(50);
        vTaskDelayUntil(&tick, 20);
    }
}

/***
 * @function_name:
 * @brief:
 * @param {type}
 * @return: None
 */
PressureMsg_t get_PressureData() { return pressure_raw; }
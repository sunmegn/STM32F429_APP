/***
 * @author        : robosea
 * @version       : v1.0.0
 * @Date          : 2019-12-16 11:15:47
 * @LastEditors   : Robosea
 * @LastEditTime  : 2020-02-21 00:35:19
 * @FilePath      : \ROV_F429_APP-10-10\USER\TASK\ledTasks.c
 * @brief         :
 */
#include "ledTasks.h"
#include "iwdg.h"
#include "global.h"
#include "MTLink.h"
#include "stmflash.h"
#include "IAP.h"
void ledTask_Function(void const *argument)
{
    portTickType tick     = xTaskGetTickCount();
    u8           my_id[1] = {2};
    MTLinkSetMY_ID(&MTLink_UDP, my_id, sizeof(my_id));
    u8 jump_flag = STMFLASH_Read_Byte(UPDATA_FLAG_ADDR);
    if (jump_flag != 2)
    {
        Jump_To_Bootloader();
    }
    while (1)
    {
#ifdef DEBUG
#else
        if (PowerKeyState() == 0)
        {
            POWER_KEY(1);
        }
        else
        {
            POWER_KEY(0);
        }
        if (g_HeartBeatCnt > 1) //如果超过两秒接收不到心跳包，则转换为手动模式
        {
            g_LostPC = 1;
            //关闭LED大灯
            LED_SetPwm(CONSTRAIN(0, 5000, 0));
            //摄像头云台恢复中位
            YunTai_SetPwm(CONSTRAIN(1500, 2000, 1000)); //控制摄像头舵机，发送中值
        }
        else
        {
            g_HeartBeatCnt++;
            g_LostPC = 0;
        }
        //BUZZER(1);
        osDelay(50);
        //BUZZER(0);
        HAL_IWDG_Refresh(&hiwdg); //超过两秒不喂狗，复位重启32k，32, 2000
#endif
        //vTaskDelayUntil(&tick,1000);	//1000/ portTICK_RATE_MS
        MTLinkPrint(&MTLink_UDP, 0x02, 0x01, "Run In APP...", sizeof("Run In APP..."), 300);
        osDelay(1000);
    }
}

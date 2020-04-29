/**
 * @author        :robosea
 * @version       :v1.0.0
 * @Date          :2019-12-16 11:15:47
 * @LastEditors:smake
 * @LastEditTime:2020-04-24 17:15:57
 * @brief         : 惯导任务函数
 */

#include "imuTasks.h"

IMUMsg_t                   IMU_raw_data;
Data_GYRO_HandleType       imurpy;
extern u8                  WakeUpHostAck[8];
extern u8                  SwitchToMeasureMode[8];
extern u8                  SwitchToConfigMode[8];
extern u8                  GetAllStatus[8];
extern CtrlPara_t          ctrlpara_data;
extern u8                  is_imustatus;
float                imu_yaw_real        = 0;
change_imu_yaw_val_typedef changed_imu_yaw_val = {0};
float                      two_Imu_yaw_error   = 0;

void imuTask_Function(void const *argument)
{
    osDelay(10);
    portTickType tick = xTaskGetTickCount();
    u8           cnt  = 0;

    while (1)
    {
        run_time[5] = getRunTime(5);
#ifdef DEBUG

#else

        IMU_GetData(&IMU_raw_data);
        imu_yaw_real                    = IMU_raw_data.yaw;
        changed_imu_yaw_val.imu_yaw_now = IMU_raw_data.yaw;
        two_Imu_yaw_error               = (changed_imu_yaw_val.imu_yaw_now - changed_imu_yaw_val.imu_yaw_last);
        if (fabs(two_Imu_yaw_error) >= 180)
        {
            if (changed_imu_yaw_val.imu_yaw_last < 0)
            {
                two_Imu_yaw_error -= 360;
            }
            else
            {
                two_Imu_yaw_error += 360;
            }
        }
        changed_imu_yaw_val.imu_yaw_totalval += two_Imu_yaw_error;
        changed_imu_yaw_val.imu_yaw_last = changed_imu_yaw_val.imu_yaw_now;
        IMU_raw_data.yaw                 = changed_imu_yaw_val.imu_yaw_totalval;

        IMU_Getrpy(&imurpy);

        ctrlpara_data.gyroX = imurpy.gyroX;
        ctrlpara_data.gyroY = imurpy.gyroY;
        ctrlpara_data.gyroZ = imurpy.gyroZ;
        cnt++;
        //		if(is_imustatus == 1)
        //		{
        //			HAL_UART_Transmit(&IMU_USART, WakeUpHostAck, sizeof(WakeUpHostAck), 0x1);
        //			is_imustatus = 2;
        //			cnt = 0;
        //		}
        //		else
        if (is_imustatus == 2) //&&cnt >= 100
        {
            HAL_UART_Transmit(&IMU_USART, SwitchToConfigMode, sizeof(SwitchToConfigMode), 0x10);
            cnt = 0;
        }
        else if (is_imustatus == 3 && cnt >= 20)
        {
            HAL_UART_Transmit(&IMU_USART, GetAllStatus, sizeof(GetAllStatus), 0x10);
            cnt = 0;
        }
        else if (is_imustatus == 4 && cnt >= 20) //
        {
            HAL_UART_Transmit(&IMU_USART, SwitchToMeasureMode, sizeof(SwitchToMeasureMode), 0x10);
            is_imustatus = 0;
        }

        xQueueOverwrite(IMU_Message_Queue, &IMU_raw_data);
#endif
        vTaskDelayUntil(&tick, 20);

        //osDelay(20);
    }
}

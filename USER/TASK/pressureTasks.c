/**
 * @author        :robosea
 * @version       :v1.0.0
 * @Date          :2019-12-16 11:15:48
 * @LastEditors   :smake
 * @LastEditTime  :2020-06-07 00:20:53
 * @FilePath      :\STM32F429_APP\USER\TASK\pressureTasks.c
 * @brief         :压力控制任务函数
 */
#include "pressureTasks.h"
#include "math.h"

PressureMsg_t            pressure_raw;

extern AllInfoToPC_Msg_t RovInfo_msg;//调试使用，使用了其中的reserve[1]曲线用于观察压传卡尔曼滤波前值；
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
        RovInfo_msg.Reserve.reserve[1] = pressure_raw.depth; //显示滤波前原始值
        //压传数据滤波处理,kalman滤波
        pressure_raw.depth = kalman_filter(pressure_raw.depth);
        xQueueOverwrite(Pressure_Message_Queue, &pressure_raw);
#endif
        vTaskDelayUntil(&tick, 20);
    }
}

/***
 * @function_name:get_PressureData
 * @brief:获得压传数值
 * @param {None}
 * @return: None
 */
PressureMsg_t get_PressureData() { return pressure_raw; }

//    卡尔曼滤波函数 （只适用于近似线性变化的量）
//     参数 ： 需要进行滤波的变量
float x_last = 0;           // 表示上一次的最优值
float p_last = 0.02;        // 表示上一次的 最优协方差
float Q      = 0.018;       // Q （偏差）为高斯白噪声 不随时间变化
float R      = 0.542;       // R  (偏差) 为高斯白噪声 不随时间变化
float kg;                   // kg 为 kalman filter 用于计算 最优值
float x_mid;                // 当前的预测值
float x_now;                // 当前的最优值
float p_mid;                // 当前的协方差
float p_now;                // 当前的 最优 协方差
float sumerror_kalman  = 0; // 估计值的 累计 误差
float sumerror_measure = 0; // 测量值的 累计误差

/**
 * @function_name:kalman_filter
 * @brief:一阶卡尔曼滤波
 * @param kalman_val:需要进行滤波的变量
 * @return:滤波输出值
 */
float kalman_filter(int kalman_val)
{
    x_mid = x_last;
    p_mid = p_last + Q;
    kg    = p_mid / (p_mid + R);
    x_now = x_mid + kg * (kalman_val - x_mid);
    p_now = (1 - kg) * p_mid;

    sumerror_kalman  = sumerror_kalman + fabs(x_mid - x_now);
    sumerror_measure = sumerror_measure + fabs(kalman_val - x_now);

    p_last = p_now; //更新covariance值
    x_last = x_now; //更新系统状态值

    return p_now;
}

#ifndef __PRESSURETASKS_H
#define __PRESSURETASKS_H

#include "includes.h"
extern QueueHandle_t Pressure_Message_Queue;

void       pressureTask_Function(void const *argument);
extern u32 Reflash(void);
extern u32 delta;

extern float x_last;           // 表示上一次的最优值
extern float p_last;           // 表示上一次的 最优协方差
extern float Q;                // Q （偏差）为高斯白噪声 不随时间变化
extern float R;                // R  (偏差) 为高斯白噪声 不随时间变化
extern float kg;               // kg 为 kalman filter 用于计算 最优值
extern float x_mid;            // 当前的预测值
extern float x_now;            // 当前的最优值
extern float p_mid;            // 当前的协方差
extern float p_now;            // 当前的 最优 协方差
extern float sumerror_kalman;  // 卡尔曼估计值的 累计 误差
extern float sumerror_measure; // 真值 和 估计值的 累计误差



float kalman_filter(int kalman_val);

#endif

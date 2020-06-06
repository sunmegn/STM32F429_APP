#ifndef __PRESSURETASKS_H
#define __PRESSURETASKS_H

#include "includes.h"
extern QueueHandle_t Pressure_Message_Queue;

void       pressureTask_Function(void const *argument);
extern u32 Reflash(void);
extern u32 delta;

extern float x_last;           // ��ʾ��һ�ε�����ֵ
extern float p_last;           // ��ʾ��һ�ε� ����Э����
extern float Q;                // Q ��ƫ�Ϊ��˹������ ����ʱ��仯
extern float R;                // R  (ƫ��) Ϊ��˹������ ����ʱ��仯
extern float kg;               // kg Ϊ kalman filter ���ڼ��� ����ֵ
extern float x_mid;            // ��ǰ��Ԥ��ֵ
extern float x_now;            // ��ǰ������ֵ
extern float p_mid;            // ��ǰ��Э����
extern float p_now;            // ��ǰ�� ���� Э����
extern float sumerror_kalman;  // ����������ֵ�� �ۼ� ���
extern float sumerror_measure; // ��ֵ �� ����ֵ�� �ۼ����



float kalman_filter(int kalman_val);

#endif

/**
 * @author        :robosea
 * @version       :v1.0.0
 * @Date          :2019-12-16 11:15:48
 * @LastEditors   :smake
 * @LastEditTime  :2020-06-07 00:20:53
 * @FilePath      :\STM32F429_APP\USER\TASK\pressureTasks.c
 * @brief         :ѹ������������
 */
#include "pressureTasks.h"
#include "math.h"

PressureMsg_t            pressure_raw;

extern AllInfoToPC_Msg_t RovInfo_msg;//����ʹ�ã�ʹ�������е�reserve[1]�������ڹ۲�ѹ���������˲�ǰֵ��
/**
 * @function_name: pressureTask_Function
 * @brief:ѹ������������
 * @param: ������
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
        RovInfo_msg.Reserve.reserve[1] = pressure_raw.depth; //��ʾ�˲�ǰԭʼֵ
        //ѹ�������˲�����,kalman�˲�
        pressure_raw.depth = kalman_filter(pressure_raw.depth);
        xQueueOverwrite(Pressure_Message_Queue, &pressure_raw);
#endif
        vTaskDelayUntil(&tick, 20);
    }
}

/***
 * @function_name:get_PressureData
 * @brief:���ѹ����ֵ
 * @param {None}
 * @return: None
 */
PressureMsg_t get_PressureData() { return pressure_raw; }

//    �������˲����� ��ֻ�����ڽ������Ա仯������
//     ���� �� ��Ҫ�����˲��ı���
float x_last = 0;           // ��ʾ��һ�ε�����ֵ
float p_last = 0.02;        // ��ʾ��һ�ε� ����Э����
float Q      = 0.018;       // Q ��ƫ�Ϊ��˹������ ����ʱ��仯
float R      = 0.542;       // R  (ƫ��) Ϊ��˹������ ����ʱ��仯
float kg;                   // kg Ϊ kalman filter ���ڼ��� ����ֵ
float x_mid;                // ��ǰ��Ԥ��ֵ
float x_now;                // ��ǰ������ֵ
float p_mid;                // ��ǰ��Э����
float p_now;                // ��ǰ�� ���� Э����
float sumerror_kalman  = 0; // ����ֵ�� �ۼ� ���
float sumerror_measure = 0; // ����ֵ�� �ۼ����

/**
 * @function_name:kalman_filter
 * @brief:һ�׿������˲�
 * @param kalman_val:��Ҫ�����˲��ı���
 * @return:�˲����ֵ
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

    p_last = p_now; //����covarianceֵ
    x_last = x_now; //����ϵͳ״ֵ̬

    return p_now;
}

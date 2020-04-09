#ifndef __KALMAN_FILTERING_H
#define __KALMAN_FILTERING_H
#include "main.h"
#include "includes.h"

#define PI 3.1415926f

#define TIME 0.02
	
extern float k_speed;
extern float ACC_speed;
extern float distance_speed;
extern float acc_middle;
extern float acc;

typedef struct
{
    float angle_dot; 	
    float Q_angle;// 过程噪声的协方差
    float Q_gyro;//0.03 过程噪声的协方差 过程噪声的协方差为一个一行两列矩阵
    float R_angle;// 测量噪声的协方差 既测量偏差
    float dt;//                 
    char  C_0;
    float Q_bias, Angle_err;
    float PCt_0, PCt_1, E;
    float K_0, K_1, t_0, t_1;
    float Pdot[4];
    float PP[2][2];
    float angle;
    float speed;
}kalman_type;


typedef struct
{
    float angle_dot; 	
    float Q_speed;// 过程噪声的协方差
    float Q_gyro;//0.03 过程噪声的协方差 过程噪声的协方差为一个一行两列矩阵
    float R_speed;// 测量噪声的协方差 既测量偏差
    float dt;//                 
    char  C_0;
    float Q_bias, speed_err;
    float PCt_0, PCt_1, E;
    float K_0, K_1, t_0, t_1;
    float Pdot[4];
    float PP[2][2];
    float speed;
}kalman_speed_type;

typedef struct
{
    float speed_x;
    float speed_y;
    float speed_z;
    float acc_x;
    float acc_y;
    float acc_z;
}speed_type;

typedef struct{
    float TuneA[100];
    float TunePu[100];
    int i;
    int j;
    float average_TuneA;
    float average_TunePu;
    float Ku;
}setting_type;

extern kalman_type kalman_x;

extern kalman_type kalman_y;

extern kalman_type kalman_z;

extern speed_type speed_data;

extern kalman_speed_type kalman_speed;

float Kalman_Filter(kalman_type *parameter,float Accel,float Gyro);


float Kalman_Filter_speed(kalman_speed_type *parameter,float distance,IMUAccPkg acc_speed, IMUMsg_t angle);


void speed_task(IMUAccPkg acc_speed, IMUMsg_t angle);

extern float speed_acc;

#endif

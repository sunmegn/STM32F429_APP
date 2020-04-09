#include "Kalman_filtering.h"
#include "math.h"
#include "includes.h"
#include "board.h"

extern IMUMsg_t IMU_raw_data;

kalman_type kalman_x = {.Q_angle = 0.25,.Q_gyro = 0.003,.dt = 0.005,.C_0 = 1,
                        .PP[0][0] = 1,.PP[0][1] = 0,.PP[1][0] = 0,.PP[1][1] = 1,.R_angle = 0.03};

kalman_type kalman_y = {.Q_angle = 0.25,.Q_gyro = 0.003,.dt = 0.005,.C_0 = 1,
                        .PP[0][0] = 1,.PP[0][1] = 0,.PP[1][0] = 0,.PP[1][1] = 1,.R_angle = 0.03};
         
kalman_type kalman_z = {.Q_angle = 0.25,.Q_gyro = 0.003,.dt = 0.005,.C_0 = 1,.R_angle = 0.11,
                        .PP[0][0] = 1,.PP[0][1] = 0,.PP[1][0] = 0,.PP[1][1] = 1};
    
kalman_speed_type kalman_speed = {.Q_speed = 0.0002,.dt = 0.02,.C_0 = 1,.R_speed = 0.9f,  
                        .PP[0][0] = 1,.PP[0][1] = 0,.PP[1][0] = 0,.PP[1][1] = 1};
    
speed_type speed_data;
    
float k_speed;
float ACC_speed;
float acc_middle;

/**
  * @funNm   Kalman_Filter
  * @brief   卡尔曼滤波
  * @param	 parameter  参数
  * @param	 Accel  电子罗盘得到的角度
  * @param	 Gyro  陀螺仪得到的角速度
  * @retval  滤波之后的角度
**/
float Kalman_Filter(kalman_type *parameter,float Accel,float Gyro)		
{
	parameter->angle += (Gyro - parameter->Q_bias) * parameter->dt; //先验估计
	parameter->Pdot[0] = parameter->Q_angle - parameter->PP[0][1] - parameter->PP[1][0]; // Pk-先验估计误差协方差的微分

	parameter->Pdot[1] = -parameter->PP[1][1];
	parameter->Pdot[2] = -parameter->PP[1][1];
	parameter->Pdot[3] = parameter->Q_gyro;
	parameter->PP[0][0] += parameter->Pdot[0] * parameter->dt;   // Pk-先验估计误差协方差微分的积分
	parameter->PP[0][1] += parameter->Pdot[1] * parameter->dt;   // =先验估计误差协方差
	parameter->PP[1][0] += parameter->Pdot[2] * parameter->dt;
	parameter->PP[1][1] += parameter->Pdot[3] * parameter->dt;
		
	parameter->Angle_err = Accel - parameter->angle;	//zk-先验估计
	
	parameter->PCt_0 = parameter->C_0 * parameter->PP[0][0]; 
	parameter->PCt_1 = parameter->C_0 * parameter->PP[1][0];
 
	parameter->E = parameter->R_angle + parameter->C_0 * parameter->PCt_0;
	
	parameter->K_0 = parameter->PCt_0 / parameter->E;
	parameter->K_1 = parameter->PCt_1 /parameter->E;
	
	parameter->t_0 = parameter->PCt_0;
	parameter->t_1 = parameter->C_0 * parameter->PP[0][1];

	parameter->PP[0][0] -= parameter->K_0 * parameter->t_0;		 //后验估计误差协方差
	parameter->PP[0][1] -= parameter->K_0 * parameter->t_1;
	parameter->PP[1][0] -= parameter->K_1 * parameter->t_0;
	parameter->PP[1][1] -= parameter->K_1 * parameter->t_1;
		
	parameter->angle += parameter->K_0 * parameter->Angle_err;	 //后验估计
	parameter->Q_bias += parameter->K_1 * parameter->Angle_err;	 //后验估计 
	parameter->angle_dot = Gyro - parameter->Q_bias;	 //输出值(后验估计)的微分=角速度
	
	return parameter->angle;
}

/**
  * @funNm   Kalman_Filter
  * @brief   卡尔曼滤波求速度
  * @param	 parameter  参数
  * @param	 distance  压传
  * @param	 加速度以及角度
  * @retval  滤波之后的角度
**/
float acc = 0;
float speed_acc = 0;
float Kalman_Filter_speed(kalman_speed_type *parameter,float distance,IMUAccPkg acc_speed, IMUMsg_t angle)		
{
    float acc_temp = 0;
    float distance_speed = 0;
    distance = distance - 17.5 * sinf(angle.pitch * PI / 180.f);
    distance = distance / 100.f;
    static float last_distance = 0;

    static float last_acc = 0;
    
    if(acc_speed.Accz == 0)
    {
        return 0;
    }
    
    distance_speed = (distance - last_distance) / parameter->dt;

    last_distance = distance;
    
    acc_temp = -sinf(angle.pitch * PI / 180.f) * acc_speed.Accx + 
                cosf(angle.pitch  * PI / 180.f) * sinf(angle.roll * PI / 180.f) * -acc_speed.Accy + 
                cosf(angle.pitch * PI / 180.f) * cosf(angle.roll * PI / 180.f) * -acc_speed.Accz;
    
    acc = acc_temp * 9.8 + 9.8 + 0.15;
    
    if(acc < 0.015f && acc > -0.015f)
    {  
        acc = 0;
        last_acc = 0;
    }
    
    speed_acc += (acc + last_acc) / 2.f * parameter->dt;
    parameter->speed += (acc + last_acc) / 2.f * parameter->dt;
    
    ACC_speed = parameter->speed;
    
    last_acc = acc;
    
	parameter->Pdot[0] = parameter->Q_speed - parameter->PP[0][1] - parameter->PP[1][0]; // Pk-先验估计误差协方差的微分

	parameter->Pdot[1] = -parameter->PP[1][1];
	parameter->Pdot[2] = -parameter->PP[1][1];
	parameter->Pdot[3] = parameter->Q_gyro; 
	parameter->PP[0][0] += parameter->Pdot[0] * parameter->dt;   // Pk-先验估计误差协方差微分的积分
	parameter->PP[0][1] += parameter->Pdot[1] * parameter->dt;   // =先验估计误差协方差
	parameter->PP[1][0] += parameter->Pdot[2] * parameter->dt;
	parameter->PP[1][1] += parameter->Pdot[3] * parameter->dt; 
		
	parameter->speed_err = distance_speed - parameter->speed;	//zk-先验估计
	
	parameter->PCt_0 = parameter->C_0 * parameter->PP[0][0]; 
	parameter->PCt_1 = parameter->C_0 * parameter->PP[1][0];
 
	parameter->E = parameter->R_speed + parameter->C_0 * parameter->PCt_0;
        
	parameter->K_0 = parameter->PCt_0 / parameter->E;
	parameter->K_1 = parameter->PCt_1 /parameter->E;
	
	parameter->t_0 = parameter->PCt_0;
	parameter->t_1 = parameter->C_0 * parameter->PP[0][1];

	parameter->PP[0][0] -= parameter->K_0 * parameter->t_0;		 //后验估计误差协方差
	parameter->PP[0][1] -= parameter->K_0 * parameter->t_1;
	parameter->PP[1][0] -= parameter->K_1 * parameter->t_0;
	parameter->PP[1][1] -= parameter->K_1 * parameter->t_1;
		
	parameter->speed += parameter->K_0 * parameter->speed_err;	 //后验估计
	parameter->Q_bias += parameter->K_1 * parameter->speed_err;	 //后验估计 
 
	return parameter->speed;
}

float middle = 0;
u8 get_middle_acc(float time)
{
    static int number = 0;
    static float value_z = 0;
    
    if(number >= time)
    {
        middle = value_z / time;
        value_z = 0;
        number = 0;
        return 1;
    }
    
    else
    {
        if(acc < 1.f && acc > -1.f && acc != 0.f)
        {
            value_z += acc;
            number++;
        }
        return 0;
    }
}

void speed_task(IMUAccPkg acc_speed, IMUMsg_t angle)
{
    angle.yaw = 0;
    static float last_acc_x = 0;
    static float last_acc_y = 0;
    static float last_acc_z = 0;
    
    float acc_z = -sinf(angle.pitch* PI / 180.f) * acc_speed.Accx + 
            cosf(angle.pitch * PI / 180.f) * sinf(angle.roll * PI / 180.f) * -acc_speed.Accy + 
            cosf(angle.pitch * PI / 180.f) * cosf(angle.roll * PI / 180.f) * -acc_speed.Accz;
    
    float acc_x = cosf(angle.yaw * PI / 180.f) * cosf(angle.pitch * PI / 180.f) * acc_speed.Accx + 
            (cosf(angle.yaw * PI / 180.f) * sinf(angle.pitch * PI / 180.f) * sinf(angle.roll * PI / 180.f) - sinf(angle.yaw * PI / 180.f) * cosf(angle.roll * PI / 180.f)) * -acc_speed.Accy+ 
            (cosf(angle.yaw * PI / 180.f) * sinf(angle.pitch * PI / 180.f) * cosf(angle.roll * PI / 180.f) + sinf(angle.yaw * PI / 180.f) * sinf(angle.roll * PI / 180.f)) * -acc_speed.Accz; 

    float acc_y = sinf(angle.yaw * PI / 180.f) * cosf(angle.pitch * PI / 180.f) * acc_speed.Accx + 
            (sinf(angle.yaw * PI / 180.f) * sinf(angle.pitch * PI / 180.f) * sinf(angle.roll * PI / 180.f) + cosf(angle.yaw * PI / 180.f) * cosf(angle.roll * PI / 180.f)) * -acc_speed.Accy +
            (sinf(angle.yaw * PI / 180.f) * sinf(angle.pitch * PI / 180.f) * cosf(angle.roll * PI / 180.f) - cosf(angle.yaw * PI / 180.f) * sinf(angle.roll * PI / 180.f)) * -acc_speed.Accz;

    speed_data.acc_x = acc_x * 9.8;
    speed_data.acc_y = acc_y * 9.8;  
    speed_data.acc_z = acc_z * 9.8 + 9.8;
    
    speed_data.speed_x += (speed_data.acc_x + last_acc_x) * TIME / 2.f;
    speed_data.speed_y += (speed_data.acc_y + last_acc_y) * TIME / 2.f;
    speed_data.speed_z += (speed_data.acc_z + last_acc_z) * TIME / 2.f;
    
    last_acc_x = speed_data.acc_x;  
    last_acc_y = speed_data.acc_y;
    last_acc_z = speed_data.acc_z;   
}


setting_type setting;
float shock(setting_type *parameter, float now_vaule,float set_vaule,float Tottle_vaule,float dead_zone)
{
    static float crest;     //波峰
    static float trough;    //波谷
    float output = 0;
    static float last_vaule;
    static float last_last_vaule;
    
    if(last_vaule > last_last_vaule && last_vaule > now_vaule)
    {
        crest = last_vaule;
    }
    
    if(last_vaule < last_last_vaule && last_vaule < now_vaule)
    {
        trough = last_vaule;
        parameter->TuneA[parameter->i++] = crest - trough;
        parameter->TunePu[parameter->j++] = getRunTime(0);   
    }
    
    if(now_vaule < set_vaule + dead_zone)
    {
        output = Tottle_vaule; 
    } 
    
    if(now_vaule > set_vaule - dead_zone)
    {
        output = -Tottle_vaule;
    }
    
    last_vaule = now_vaule;
    last_last_vaule = last_vaule;
    
    return output;
}




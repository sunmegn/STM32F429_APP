#include "PID.h"
 
PID_t Deep_pid;
PID_t Heading_pid;
//增量式PID
//	-15.8    -0.076   电压小于15.6V
//  -13.8    -0.05    电压大于15.6V

//位置式PID
//   -12     -0.35

                              //kp,ki,kd,    p_max,     p_min,    i_max,     i_min,    d_max,     d_min,  out_max,  out_min   
float Heading_pid_param[11]={  -4.7,-0.03,-0.01,MAX_SPEED,-MAX_SPEED,MAX_SPEED,-MAX_SPEED,MAX_SPEED,-MAX_SPEED,MAX_SPEED,-MAX_SPEED };
float    Deep_pid_param[11]={-15.8,-0.076,0.0,MAX_SPEED,-MAX_SPEED,MAX_SPEED,-MAX_SPEED,MAX_SPEED,-MAX_SPEED,MAX_SPEED,-MAX_SPEED};

//float Heading_pid_param[11]={  0,0,0,MAX_SPEED,-MAX_SPEED,MAX_SPEED,-MAX_SPEED,MAX_SPEED,-MAX_SPEED,MAX_SPEED,-MAX_SPEED };
//float    Deep_pid_param[11]={-15.8,-0.076,0.0,MAX_SPEED,-MAX_SPEED,MAX_SPEED,-MAX_SPEED,MAX_SPEED,-MAX_SPEED,MAX_SPEED,-MAX_SPEED};

void PID_Init(void)                                                 //PID  - 初始化
{
	PID_Param_Init(&Deep_pid,Deep_pid_param);       //定身 - 推进器
	PID_Param_Init(&Heading_pid,Heading_pid_param); //定航 - 尾巴推进 PID
}
 
void PID_Param_Init(PID_t *pid, float *pid_param) //PID 参数设置函数
{
	pid->Kp = pid_param[0];
	pid->Ki = pid_param[1];;
	pid->Kd = pid_param[2];;
	pid->p_OutMAX = pid_param[3];
	pid->p_OutMIN = pid_param[4];
	pid->i_OutMAX = pid_param[5];
	pid->i_OutMIN = pid_param[6];
	pid->d_OutMAX = pid_param[7];
	pid->d_OutMIN = pid_param[8];
	pid->OutPutMAX = pid_param[9];
	pid->OutPutMIN = pid_param[10];
}
void PID_Clear(PID_t *pid)                   //PID 累计量清空
{
  pid->Bias = 0;
	pid->Last_Bias = 0;
	pid->Last_Last_Bias = 0;
	pid->p_Out = 0;
	pid->i_Out = 0;
	pid->d_Out = 0;
	pid->OutPut = 0;
} 
int Position_PID_Cal(float tar,float input,PID_t *pid)
{
	pid->Bias = tar -input;
	
	pid->p_Out = pid->Kp*pid->Bias;
	pid->p_Out = CONSTRAIN(pid->p_Out, pid->p_OutMAX, pid->p_OutMIN); 
	
	pid->i_Out+= pid->Ki*pid->Bias;                                                          
	pid->i_Out = CONSTRAIN(pid->i_Out, pid->i_OutMAX, pid->i_OutMIN);
	
	pid->d_Out = pid->Kd*(pid->Bias - pid->Last_Bias);
  pid->d_Out = CONSTRAIN(pid->d_Out, pid->d_OutMAX, pid->d_OutMIN);
	
	pid->OutPut  = pid->p_Out+pid->i_Out+pid->d_Out;
	pid->OutPut  = CONSTRAIN(pid->OutPut,pid->OutPutMAX,pid->OutPutMIN);
	
	pid->Last_Bias = pid->Bias;						
  return pid->OutPut;
}	
int Incremental_PID_Cal(float tar,float input,PID_t *pid)         //增量式PID计算 
{
	pid->Bias = tar - input ;
	
	pid->p_Out = pid->Kp*(pid->Bias-pid->Last_Bias);                                      
	pid->p_Out = CONSTRAIN(pid->p_Out, pid->p_OutMAX, pid->p_OutMIN);           
	
	pid->i_Out = pid->Ki*pid->Bias;                                                          
	pid->i_Out = CONSTRAIN(pid->i_Out, pid->i_OutMAX, pid->i_OutMIN);
	
	pid->d_Out = pid->Kd*(pid->Bias - 2*pid->Last_Bias + pid->Last_Last_Bias);
  pid->d_Out = CONSTRAIN(pid->d_Out, pid->d_OutMAX, pid->d_OutMIN);
	
	pid->OutPut += pid->p_Out+pid->i_Out+pid->d_Out;
	pid->OutPut  = CONSTRAIN(pid->OutPut,pid->OutPutMAX,pid->OutPutMIN);
	
	pid->Last_Last_Bias = pid->Last_Bias;
	pid->Last_Bias = pid->Bias;						
  return pid->OutPut;
}

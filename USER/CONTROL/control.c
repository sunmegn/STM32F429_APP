/**
 * @author        :robosea
 * @version       :v1.0.0
 * @Date          :2019-12-16 11:15:42
 * @LastEditors   :smake
 * @LastEditTime  :2020-04-30 15:36:53
 * @brief         :电机控制函数组
 */
#include "control.h"
#include "stmflash.h"
#define DogMax_WaitTimes 100
//static ControlMsg_t control_data;
int               RoboHand_Pwm = MANIPULATOR_MID;
int               DogBeatCount = DogMax_WaitTimes;
ROV_FValuetypedef ROV_Val;
u8                motorcalflag = 0; //是否进行电机PWM标定
CtrlPara_t        ctrlpara_data;
extern uint16_t   MotorPWMMidVal[6];

/**
 * @function_name:CtrlMidPwmInit
 * @brief:初始化推进器中值
 * @param :None
 * @return:None
 */
void CtrlMidPwmInit(void)
{
    STMFLASH_Read(PARAMADDR, MotorPWMMidVal, 6);
    if (MotorPWMMidVal[0] < MINMIDPWMVAL || MotorPWMMidVal[0] > MAXMIDPWMVAL) //判断中值是否在合理区间
    {
        MotorPWMMidVal[0] = 1500;
    }
    if (MotorPWMMidVal[1] < MINMIDPWMVAL || MotorPWMMidVal[1] > MAXMIDPWMVAL)
    {
        MotorPWMMidVal[1] = 1500;
    }
    if (MotorPWMMidVal[2] < MINMIDPWMVAL || MotorPWMMidVal[2] > MAXMIDPWMVAL)
    {
        MotorPWMMidVal[2] = 1500;
    }
    if (MotorPWMMidVal[3] < MINMIDPWMVAL || MotorPWMMidVal[3] > MAXMIDPWMVAL)
    {
        MotorPWMMidVal[3] = 1500;
    }
    if (MotorPWMMidVal[4] < MINMIDPWMVAL || MotorPWMMidVal[4] > MAXMIDPWMVAL)
    {
        MotorPWMMidVal[4] = 1500;
    }
    if (MotorPWMMidVal[5] < MINMIDPWMVAL || MotorPWMMidVal[5] > MAXMIDPWMVAL)
    {
        MotorPWMMidVal[5] = 1500;
    }
}

void FeedDogHeardBeat(void)
{
    DogBeatCount = DogMax_WaitTimes;
}

/**
 * @function_name:DogHeardBeat
 * @brief:心跳
 * @param :None
 * @return:u8
 */
u8 DogHeardBeat(void)
{
    DogBeatCount--;
    if (DogBeatCount <= 0)
    {
        DogBeatCount = 0;
        return 0;
    }
    else
    {
        return 1;
    }
}

////////以下推力转换成pwm控制
/**
  * @funNm	sign符号函数
  * @brief  输入值取符号 返回±1
  * @param	x:输入float型数据
  * @retval x > 0 return 1 x < 0 return -1 x = 0 return 0
  */
float sign(float x)
{
    float output = 0;
    if (x > 0)
    {
        output = 1;
    }
    else if (x < 0)
    {
        output = -1;
    }
    return output;
}

/**
  * @funNm  SetValf32
  * @brief  输入参数限幅输出
  * @param	x:输入参数
  * @param	Lval:最小阈值
  * @param	Hval:最大阈值
  * @retval float:返回限幅后的数
  */
float SetValf32(float x, float Lval, float Hval)
{
    if (x > Hval)
    {
        return Hval;
    }
    else if (x < Lval)
    {
        return Lval;
    }
    return x;
}

/**
  * @funNm  OutputMax
  * @brief  比较输出两个参数最大值
  * @param	a:输入参数1
  * @param	b:输入参数2
  * @retval float:返回两个参数最大值
  */
float OutputMax(float a, float b)
{
    if (a > b)
    {
        return a;
    }
    return b;
}
/**
  * @funNm  ThrottleAutoAdapt
  * @brief  -1000 ~ +1000的油门自适应到对应区间上
  * @param	in:输入的油门
  * @param	min:区间最小值
  * @param	max:区间最大值
  * @retval float返回对应区间的值
  */
float ThrottleAutoAdapt(float in, float min, float max)
{
    float pmax = OutputMax(fabs(min), fabs(max));
    float out  = in * pmax / 1000;
    out        = SetValf32(out, min, max);
    return out;
}

/**
  * @funNm  InputFxyDec
  * @brief  xy平面力分解到对应斜率的直线上
  * @param	ang:分解到对应直线上，直线与x轴的夹角
  * @param	x0:x方向实际推力
  * @param	y0:y方向实际推力
  * @retval float:返回计算后的分解到直线上的推力大小
  */
float InputFxyDec(float ang, float x0, float y0)
{
    //FIXME 这里力分解有问题，应该往电机轴线所在直线上分解,修改后如下
    // return (x0 / sinf(ang) + y0 / cosf(ang));

    float tsita = tan(ang * pi / 180);
    float x     = (x0 + tsita * y0) / (2 * tsita + 0.0000001);
    float y     = tsita * x;
    return (sqrt(x * x + y * y)) * sign(x);
}

/**
  * @funNm  OwnForceX
  * @brief  根据每个推进器实际推力计算自身XO方向产生的推力
  * @param	Tang:推进器装配角度
  * @param	f1:T1推力
  * @param	f2:T2推力
  * @param	f3:T3推力
  * @param	f4:T4推力
  * @retval float:返回计算后推力大小
  */
float OwnForceX(float Tang, float f1, float f2, float f3, float f4)
{
    float sita = Tang * pi / 180;
    return (f1 * sin(sita) - f2 * sin(sita) + f3 * sin(sita) - f4 * sin(sita));
}

/**
  * @funNm  OwnForceY
  * @brief  根据每个推进器实际推力计算自身YO方向产生的推力
  * @param	Tang:推进器装配角度
  * @param	f1:T1推力
  * @param	f2:T2推力
  * @param	f3:T3推力
  * @param	f4:T4推力
  * @retval float:返回计算后推力大小
  */
float OwnForceY(float Tang, float f1, float f2, float f3, float f4)
{
    float sita = Tang * pi / 180;
    return (f1 * cos(sita) + f2 * cos(sita) - f3 * cos(sita) - f4 * cos(sita));
}

/**
  * @funNm  OwnForceZ
  * @brief  根据每个推进器实际推力计算自身ZO方向产生的推力
  * @param	f5:T5推力
  * @param	f6:T6推力
* @retval float:返回计算后推力大小
  */
float OwnForceZ(float f5, float f6)
{
    return (f5 + f6);
}

/**
  * @funNm  ActiveForOxy
  * @brief  xy方向推力分配
  * @param	Tang:推进器与x轴夹角
  * @param	fx:OX方向推力
  * @param	fy:OY方向推力
  * @param	*F:F[7]对应6个推进器的推力F[0]无效
  * @retval 0
  */
float ActiveForOxy(float Tang, float fx, float fy, float *F)
{
    float F14, F23, Fmax = fabs(PROPFMin * 2);
    float kx = 1;
    F14      = InputFxyDec(Tang, fx, fy); //
    F23      = InputFxyDec(-Tang, fx, fy);
    Fmax     = OutputMax(fabs(F14), fabs(F23));
    if (fabs(fx) <= 1e-6 || fabs(fy) <= 1e-6)
    {
        kx = 1; //直游和侧移
    }
    else
    {
        kx = fabs(PROPFMin * 2) / (Fmax + 0.000001); //缩小
    }
    F14  = F14 * kx;
    F23  = F23 * kx;
    F[1] = F14 / 2;
    F[2] = F23 / 2;
    F[3] = -1 * F23 / 2;
    F[4] = -1 * F14 / 2;
    F[5] = 0;
    F[6] = 0;
    return 0;
}

/**
  * @funNm  ActiveForZO
  * @brief  Z方向推力分配
  * @param	fz:ZO方向推力
  * @param	*F:F[7]对?6个推进器推力F[0]无效
  * @retval 0
  */
float ActiveForZO(float fz, float *F)
{
    float F56 = fz / 2;
    F[1]      = 0;
    F[2]      = 0;
    F[3]      = 0;
    F[4]      = 0;
    F[5]      = F56;
    F[6]      = F56;
    return 0;
}

/**
  * @funNm  ActiveForYaw
  * @brief  yaw自由度推力分配
  * @param	fyaw:yaw自由度推力
  * @param	*F:F[7]对应6个推进器的推力F[0]无效
  * @retval 0
  */
float ActiveForYaw(float fyaw, float *F)
{
    float DF14 = fyaw / 4;
    float DF23 = fyaw / 4;
    F[1]       = -1 * DF14;
    F[2]       = DF23;
    F[3]       = DF23;
    F[4]       = -1 * DF14;
    F[5]       = 0;
    F[6]       = 0;
    return 0;
}

/**
  * @funNm  ActiveForPitch
  * @brief  pitch自由度推力分配
  * @param	fpitch:pitch自由度推力
  * @param	*F:F[7]对应6个推进器的推力F[0]无效
  * @retval 0
  */
float ActiveForPitch(float fpitch, float *F)
{
    F[1] = 0;
    F[2] = 0;
    F[3] = 0;
    F[4] = 0;
    F[5] = 0;
    F[6] = 0;
    return 0;
}

/**
  * @funNm  ActiveForRoll
  * @brief  roll自由度推力分配
  * @param	froll:roll自由度推力
  * @param	*F:F[7]对应6个推进器的推力F[0]无效
  * @retval 0
  */
float ActiveForRoll(float froll, float *F)
{
    float DF56 = froll / 2;
    F[1]       = 0;
    F[2]       = 0;
    F[3]       = 0;
    F[4]       = 0;
    F[5]       = -1 * DF56;
    F[6]       = DF56;
    return 0;
}

/**
  * @funNm  ForceToESC
  * @brief  根据推力映射成双向电调PWM值
  * @param	f:推力大小
  * @param	PorN:正桨还是反桨 1：正桨 -1：反桨
  * @param	offset:基准0值
  * @retval uint16:PWM值(限幅1500-2500)
  */
uint16_t ForceToESC(float f, float PorN, float offset)
{
    float A[4] = {0.0027, -0.2944, 15.1133, 35.4478};
    float x, y;

    if (f > 0)
    {
        x = f * 63.59 / PROPFMax;
        y = A[0] * x * x * x + A[1] * x * x + A[2] * x + A[3];
        y = SetValf32(y, 0, 500.1);
    }
    else if (f < 0)
    {
        x = f * 63.55 / PROPFMin;
        y = A[0] * x * x * x + A[1] * x * x + A[2] * x + A[3];
        y = -1 * y;
        y = SetValf32(y, -500.1, 0);
    }
    else
    {
        y = 0;
    }

    return (y * PorN + offset);
}

uint16_t PROP1, PROP2, PROP3, PROP4, PROP5, PROP6;

/**
  * @funNm  Cal6FreedomForceValue
  * @brief  根据每个推进器计算六自由度的推力范围 并写到ROV_VAL结构体中
  * @param	Tang:推进器相对x轴角度
  * @param	Tmin:最小推力(Tmin < 0 负推力)
  * @param	Tmax:最大推力Tmax > 0 正推力
  * @retval void
  */
void Cal6FreedomForceValue(float Tang, float Tmin, float Tmax)
{
    memset(&ROV_Val, 0, sizeof(ROV_Val));
    ROV_Val.Fxmax     = OwnForceX(Tang, Tmax, Tmin, Tmax, Tmin);
    ROV_Val.Fxmin     = -1 * ROV_Val.Fxmax;
    ROV_Val.Fymax     = OwnForceY(Tang, Tmax, Tmax, Tmin, Tmin);
    ROV_Val.Fymin     = -1 * ROV_Val.Fymax;
    ROV_Val.Fzmax     = OwnForceZ(Tmax, Tmax);
    ROV_Val.Fzmin     = OwnForceZ(Tmin, Tmin);
    ROV_Val.Fyawmax   = fabs(Tmin) * 4;
    ROV_Val.Fyawmin   = -1 * fabs(Tmin) * 4;
    ROV_Val.Fpitchmax = 1;
    ROV_Val.Fpitchmin = -1;
    ROV_Val.Frollmax  = fabs(Tmin) * 2;
    ROV_Val.Frollmin  = -1 * ROV_Val.Frollmax;
}

/**
  * @funNm  GetThrottleKpower
  * @brief  计算政府油门占单侧油门的比例
  * @param	x:输入油门
  * @param	min:幅最大油门min < 0
  * @param	max:正最大油门 max > 0
  * @retval 返回油门比例
  */
float GetThrottleKpower(float x, float min, float max)
{
    if (x > 0)
    {
        return (x / max);
    }
    else if (x < 0)
    {
        return (x / min);
    }
    return 0;
}

/**
  * @funNm  SixFreedomForceControl
  * @brief  每个自由度应受力对应到具体推进器的力 以及是否需要姿态补偿
  * @param	Closestate: true:开启三维力补偿  false:不进行力补偿
  * @param	Fx:x方向力
  * @param	Fy:y方向力
  * @param	Fz:z方向力
  * @param	Fyaw:yaw转动力(忽略角度系数)
  * @param	Fpitch:pitch转动力
  * @param	Froll:roll转动力
  * @param	yaw:当前yaw角
  * @param	pitch:当前pitch角
  * @param	roll:当前roll角
  * @retval void
  * @以下为控制量输出与推进器对应关系
  *        ∧     
  *        |      
  *     1  |  4   
  *   2---------5 
  *     3  |  6   
  *        |      
  */
SOTF_typeDef FLpfPWM1;
SOTF_typeDef FLpfPWM2;
SOTF_typeDef FLpfPWM3;
SOTF_typeDef FLpfPWM4;
SOTF_typeDef FLpfPWM5;
SOTF_typeDef FLpfPWM6;
AccLimit_t   ACCPWM1;
AccLimit_t   ACCPWM2;
AccLimit_t   ACCPWM3;
AccLimit_t   ACCPWM4;
AccLimit_t   ACCPWM5;
AccLimit_t   ACCPWM6;

float KxCompToYaw  = 0;
float KxCompToRoll = 0;
float KxCopmToZo   = 0;

/**
 * @function_name:SixFreedomForceControl
 * @brief:六轴力转每个电机转速
 * @param :None
 * @return:None
 */
void SixFreedomForceControl(bool Closestate, float Fx, float Fy, float Fz, float Fyaw, float Fpitch, float Froll, float yaw, float pitch, float roll)
{
    static float CInfx = 0, CInfy = 0, CInfz = 0, CInfyaw = 0, CInfpitch = 0, CInfroll = 0;
    static float BufFoxy[7], BufFzo[7], BufFyaw[7], BufFpitch[7], BufFroll[7];
    float        fx = 0, fy = 0, fz = 0, Ry, Rp, Rr;

    CInfx     = SetValf32(Fx, ROV_Val.Fxmin, ROV_Val.Fxmax); //限制x方向力大小
    CInfy     = SetValf32(Fy, ROV_Val.Fymin, ROV_Val.Fymax);
    CInfz     = SetValf32(Fz, ROV_Val.Fzmin, ROV_Val.Fzmax);
    CInfyaw   = SetValf32(Fyaw, ROV_Val.Fyawmin, ROV_Val.Fyawmax);
    CInfpitch = SetValf32(Fpitch, ROV_Val.Fpitchmin, ROV_Val.Fpitchmax);
    CInfroll  = SetValf32(Froll, ROV_Val.Frollmin, ROV_Val.Frollmax);

    //三方向被动力补偿
    if (Closestate == true)
    {
        Ry    = yaw * pi / 180; //角度转弧度
        Rp    = pitch * pi / 180;
        Rr    = roll * pi / 180;
        CInfx = SetValf32((Fx - fx) / (cos(Rr) + 0.000001), ROV_Val.Fxmin, ROV_Val.Fxmax);
        CInfy = SetValf32((Fy - fy) / (cos(Rp) + 0.000001), ROV_Val.Fymin, ROV_Val.Fymax);
    }

    float kx, ky, Addkxy, kyaw, kz, kroll, Addkrz;
    //油门比例计算
    kx     = GetThrottleKpower(CInfx, ROV_Val.Fxmin, ROV_Val.Fxmax);
    ky     = GetThrottleKpower(CInfy, ROV_Val.Fymin, ROV_Val.Fymax);
    kyaw   = GetThrottleKpower(CInfyaw, ROV_Val.Fyawmin, ROV_Val.Fyawmax);
    Addkxy = (OutputMax(kx, ky) + kyaw) - 1; //似皆硕?溢出
    if (Addkxy > 0)
    {
        CInfx = CInfx * (kx - Addkxy) / (kx + 0.000001);
        CInfy = CInfy * (ky - Addkxy) / (ky + 0.000001);
    }
    //重新补偿
    CInfyaw = SetValf32(Fyaw + KxCompToYaw * CInfx, ROV_Val.Fyawmin, ROV_Val.Fyawmax);
    //z轴补偿
    fz    = CInfx * sin(Rr) - CInfy * sin(Rp);
    CInfz = SetValf32((Fz - fz) / (cos(Rp) * cos(Rr) + 0.000001), ROV_Val.Fzmin, ROV_Val.Fzmax);
    //	if(CInfz < 0){
    //			CInfz=CInfz*KxCopmToZo;
    //	}
    //	else{
    //		CInfz=1*CInfz;
    //	}
    CInfroll = SetValf32(Froll + KxCompToRoll * CInfx, ROV_Val.Frollmin, ROV_Val.Frollmax);
    //竖直方向定深优先
    kz     = GetThrottleKpower(CInfz, ROV_Val.Fzmin, ROV_Val.Fzmax);
    kroll  = GetThrottleKpower(CInfroll, ROV_Val.Frollmin, ROV_Val.Frollmax);
    Addkrz = kz + kroll - 1;
    //竖直方向定深优先
    if (Addkrz > 0)
    { //缩小侧倾油门
        CInfroll = CInfroll * (kroll - Addkrz) / (kroll + 0.000001);
    }
    //对应到每个推进器
    //水平
    ActiveForOxy(30, CInfx, CInfy, BufFoxy);
    ActiveForYaw(CInfyaw, BufFyaw);
    //滚转
    ActiveForZO(CInfz, BufFzo);
    ActiveForPitch(CInfpitch, BufFpitch);
    ActiveForRoll(CInfroll, BufFroll);

    if (motorcalflag)
    {
        //进行版本转换，不同电机编号对应
        PROP1 = MotorPWMMidVal[0];
        PROP2 = MotorPWMMidVal[3];
        PROP3 = MotorPWMMidVal[2];
        PROP4 = MotorPWMMidVal[5];
        PROP5 = MotorPWMMidVal[1];
        PROP6 = MotorPWMMidVal[4];
    }
    else
    {
        PROP1 = ForceToESC(BufFoxy[1] + BufFzo[1] + BufFyaw[1] + BufFpitch[1] + BufFroll[1], -1, MotorPWMMidVal[0]); //左前
        PROP2 = ForceToESC(BufFoxy[2] + BufFzo[2] + BufFyaw[2] + BufFpitch[2] + BufFroll[2], -1, MotorPWMMidVal[3]); //右前
        PROP3 = ForceToESC(BufFoxy[3] + BufFzo[3] + BufFyaw[3] + BufFpitch[3] + BufFroll[3], -1, MotorPWMMidVal[2]); //左后
        PROP4 = ForceToESC(BufFoxy[4] + BufFzo[4] + BufFyaw[4] + BufFpitch[4] + BufFroll[4], -1, MotorPWMMidVal[5]); //右后
        PROP5 = ForceToESC(BufFoxy[5] + BufFzo[5] + BufFyaw[5] + BufFpitch[5] + BufFroll[5], -1, MotorPWMMidVal[1]); //左中
        PROP6 = ForceToESC(BufFoxy[6] + BufFzo[6] + BufFyaw[6] + BufFpitch[6] + BufFroll[6], -1, MotorPWMMidVal[4]); //右中
    }

    IIR_2OrderLpf_Init(&FLpfPWM5, 50, 4);
    IIR_2OrderLpf_Init(&FLpfPWM6, 50, 4);

    pca_setpwm(FL_MOTOR, 0, PROP1);                        //左前
    pca_setpwm(FR_MOTOR, 0, PROP2);                        //右前
    pca_setpwm(BL_MOTOR, 0, PROP3);                        //左后
    pca_setpwm(BR_MOTOR, 0, PROP4);                        //右后
                                                           //	pca_setpwm(UL_MOTOR,0,PROP5);//ZUO
                                                           //	pca_setpwm(UR_MOTOR,0,PROP6);//YOU
    pca_setpwm(UL_MOTOR, 0, SOTFOutput(&FLpfPWM5, PROP5)); //ZUO //FIXME SOTFOutput返回值为float,pca_setpwm输入参数为u32
    pca_setpwm(UR_MOTOR, 0, SOTFOutput(&FLpfPWM6, PROP6)); //YOU
    MANIPULATOR_VAL(RoboHand_Pwm);                         //机械手
    ctrlpara_data.FR_val = PROP2;
    ctrlpara_data.FL_val = PROP1;
    ctrlpara_data.BR_val = PROP4;
    ctrlpara_data.BL_val = PROP3;
    ctrlpara_data.UL_val = FLpfPWM5.yk;
    ctrlpara_data.UR_val = FLpfPWM6.yk;
}

/**
  * @funNm  ThrottleToForceControl
  * @brief  油门到力的转换 以及是否需要姿态补偿
  * @param	Closestate: true:开启三维补偿  false:不进行补偿
  * @param	Tx:x方向油门
  * @param	Ty:y方向油门
  * @param	Tz:z方向油门
  * @param	Tyaw:yaw转动油门
  * @param	Tpitch:pitch转动油门
  * @param	Troll:roll转动油门
  * @param	yaw:当前yaw角
  * @param	pitch:当前pitch角
  * @param	roll:当前roll角
  * @retval void
  */
void ThrottleToForceControl(bool Closestate, float Tx, float Ty, float Tz, float Tyaw, float Tpitch, float Troll, float yaw, float pitch, float roll)
{
    float Fx, Fy;
    if (ROV_Val.Fxmax > ROV_Val.Fymax)
    { //水平力统一标定 保证方向性 按最大的标定
        Fx = ThrottleAutoAdapt(Tx, ROV_Val.Fxmin, ROV_Val.Fxmax);
        Fy = ThrottleAutoAdapt(Ty, ROV_Val.Fxmin, ROV_Val.Fxmax);
    }
    else
    {
        Fx = ThrottleAutoAdapt(Tx, ROV_Val.Fymin, ROV_Val.Fymax);
        Fy = ThrottleAutoAdapt(Ty, ROV_Val.Fymin, ROV_Val.Fymax);
    }
    float Fz     = ThrottleAutoAdapt(Tz, ROV_Val.Fzmin, ROV_Val.Fzmax);
    float Fyaw   = ThrottleAutoAdapt(Tyaw, ROV_Val.Fyawmin, ROV_Val.Fyawmax);
    float Fpitch = ThrottleAutoAdapt(Tpitch, ROV_Val.Fpitchmin, ROV_Val.Fpitchmax);
    float Froll  = ThrottleAutoAdapt(Troll, ROV_Val.Frollmin, ROV_Val.Frollmax);
    KxCompToRoll = -0.85 * fabs(NLfal(Tx, 0.9, 1000) / 1000);
    SixFreedomForceControl(Closestate, Fx, Fy, Fz, Fyaw, Fpitch, Froll, yaw, pitch, roll);
}

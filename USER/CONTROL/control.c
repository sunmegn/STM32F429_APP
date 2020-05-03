/**
 * @author        :robosea
 * @version       :v1.0.0
 * @Date          :2019-12-16 11:15:42
 * @LastEditors   :smake
 * @LastEditTime  :2020-04-30 15:36:53
 * @brief         :������ƺ�����
 */
#include "control.h"
#include "stmflash.h"
#define DogMax_WaitTimes 100
//static ControlMsg_t control_data;
int               RoboHand_Pwm = MANIPULATOR_MID;
int               DogBeatCount = DogMax_WaitTimes;
ROV_FValuetypedef ROV_Val;
u8                motorcalflag = 0; //�Ƿ���е��PWM�궨
CtrlPara_t        ctrlpara_data;
extern uint16_t   MotorPWMMidVal[6];

/**
 * @function_name:CtrlMidPwmInit
 * @brief:��ʼ���ƽ�����ֵ
 * @param :None
 * @return:None
 */
void CtrlMidPwmInit(void)
{
    STMFLASH_Read(PARAMADDR, MotorPWMMidVal, 6);
    if (MotorPWMMidVal[0] < MINMIDPWMVAL || MotorPWMMidVal[0] > MAXMIDPWMVAL) //�ж���ֵ�Ƿ��ں�������
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
 * @brief:����
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

////////��������ת����pwm����
/**
  * @funNm	sign���ź���
  * @brief  ����ֵȡ���� ���ء�1
  * @param	x:����float������
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
  * @brief  ��������޷����
  * @param	x:�������
  * @param	Lval:��С��ֵ
  * @param	Hval:�����ֵ
  * @retval float:�����޷������
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
  * @brief  �Ƚ���������������ֵ
  * @param	a:�������1
  * @param	b:�������2
  * @retval float:���������������ֵ
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
  * @brief  -1000 ~ +1000����������Ӧ����Ӧ������
  * @param	in:���������
  * @param	min:������Сֵ
  * @param	max:�������ֵ
  * @retval float���ض�Ӧ�����ֵ
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
  * @brief  xyƽ�����ֽ⵽��Ӧб�ʵ�ֱ����
  * @param	ang:�ֽ⵽��Ӧֱ���ϣ�ֱ����x��ļн�
  * @param	x0:x����ʵ������
  * @param	y0:y����ʵ������
  * @retval float:���ؼ����ķֽ⵽ֱ���ϵ�������С
  */
float InputFxyDec(float ang, float x0, float y0)
{
    //FIXME �������ֽ������⣬Ӧ���������������ֱ���Ϸֽ�,�޸ĺ�����
    // return (x0 / sinf(ang) + y0 / cosf(ang));

    float tsita = tan(ang * pi / 180);
    float x     = (x0 + tsita * y0) / (2 * tsita + 0.0000001);
    float y     = tsita * x;
    return (sqrt(x * x + y * y)) * sign(x);
}

/**
  * @funNm  OwnForceX
  * @brief  ����ÿ���ƽ���ʵ��������������XO�������������
  * @param	Tang:�ƽ���װ��Ƕ�
  * @param	f1:T1����
  * @param	f2:T2����
  * @param	f3:T3����
  * @param	f4:T4����
  * @retval float:���ؼ����������С
  */
float OwnForceX(float Tang, float f1, float f2, float f3, float f4)
{
    float sita = Tang * pi / 180;
    return (f1 * sin(sita) - f2 * sin(sita) + f3 * sin(sita) - f4 * sin(sita));
}

/**
  * @funNm  OwnForceY
  * @brief  ����ÿ���ƽ���ʵ��������������YO�������������
  * @param	Tang:�ƽ���װ��Ƕ�
  * @param	f1:T1����
  * @param	f2:T2����
  * @param	f3:T3����
  * @param	f4:T4����
  * @retval float:���ؼ����������С
  */
float OwnForceY(float Tang, float f1, float f2, float f3, float f4)
{
    float sita = Tang * pi / 180;
    return (f1 * cos(sita) + f2 * cos(sita) - f3 * cos(sita) - f4 * cos(sita));
}

/**
  * @funNm  OwnForceZ
  * @brief  ����ÿ���ƽ���ʵ��������������ZO�������������
  * @param	f5:T5����
  * @param	f6:T6����
* @retval float:���ؼ����������С
  */
float OwnForceZ(float f5, float f6)
{
    return (f5 + f6);
}

/**
  * @funNm  ActiveForOxy
  * @brief  xy������������
  * @param	Tang:�ƽ�����x��н�
  * @param	fx:OX��������
  * @param	fy:OY��������
  * @param	*F:F[7]��Ӧ6���ƽ���������F[0]��Ч
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
        kx = 1; //ֱ�κͲ���
    }
    else
    {
        kx = fabs(PROPFMin * 2) / (Fmax + 0.000001); //��С
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
  * @brief  Z������������
  * @param	fz:ZO��������
  * @param	*F:F[7]��?6���ƽ�������F[0]��Ч
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
  * @brief  yaw���ɶ���������
  * @param	fyaw:yaw���ɶ�����
  * @param	*F:F[7]��Ӧ6���ƽ���������F[0]��Ч
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
  * @brief  pitch���ɶ���������
  * @param	fpitch:pitch���ɶ�����
  * @param	*F:F[7]��Ӧ6���ƽ���������F[0]��Ч
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
  * @brief  roll���ɶ���������
  * @param	froll:roll���ɶ�����
  * @param	*F:F[7]��Ӧ6���ƽ���������F[0]��Ч
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
  * @brief  ��������ӳ���˫����PWMֵ
  * @param	f:������С
  * @param	PorN:�������Ƿ��� 1������ -1������
  * @param	offset:��׼0ֵ
  * @retval uint16:PWMֵ(�޷�1500-2500)
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
  * @brief  ����ÿ���ƽ������������ɶȵ�������Χ ��д��ROV_VAL�ṹ����
  * @param	Tang:�ƽ������x��Ƕ�
  * @param	Tmin:��С����(Tmin < 0 ������)
  * @param	Tmax:�������Tmax > 0 ������
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
  * @brief  ������������ռ�������ŵı���
  * @param	x:��������
  * @param	min:���������min < 0
  * @param	max:��������� max > 0
  * @retval �������ű���
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
  * @brief  ÿ�����ɶ�Ӧ������Ӧ�������ƽ������� �Լ��Ƿ���Ҫ��̬����
  * @param	Closestate: true:������ά������  false:������������
  * @param	Fx:x������
  * @param	Fy:y������
  * @param	Fz:z������
  * @param	Fyaw:yawת����(���ԽǶ�ϵ��)
  * @param	Fpitch:pitchת����
  * @param	Froll:rollת����
  * @param	yaw:��ǰyaw��
  * @param	pitch:��ǰpitch��
  * @param	roll:��ǰroll��
  * @retval void
  * @����Ϊ������������ƽ�����Ӧ��ϵ
  *        ��     
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
 * @brief:������תÿ�����ת��
 * @param :None
 * @return:None
 */
void SixFreedomForceControl(bool Closestate, float Fx, float Fy, float Fz, float Fyaw, float Fpitch, float Froll, float yaw, float pitch, float roll)
{
    static float CInfx = 0, CInfy = 0, CInfz = 0, CInfyaw = 0, CInfpitch = 0, CInfroll = 0;
    static float BufFoxy[7], BufFzo[7], BufFyaw[7], BufFpitch[7], BufFroll[7];
    float        fx = 0, fy = 0, fz = 0, Ry, Rp, Rr;

    CInfx     = SetValf32(Fx, ROV_Val.Fxmin, ROV_Val.Fxmax); //����x��������С
    CInfy     = SetValf32(Fy, ROV_Val.Fymin, ROV_Val.Fymax);
    CInfz     = SetValf32(Fz, ROV_Val.Fzmin, ROV_Val.Fzmax);
    CInfyaw   = SetValf32(Fyaw, ROV_Val.Fyawmin, ROV_Val.Fyawmax);
    CInfpitch = SetValf32(Fpitch, ROV_Val.Fpitchmin, ROV_Val.Fpitchmax);
    CInfroll  = SetValf32(Froll, ROV_Val.Frollmin, ROV_Val.Frollmax);

    //�����򱻶�������
    if (Closestate == true)
    {
        Ry    = yaw * pi / 180; //�Ƕ�ת����
        Rp    = pitch * pi / 180;
        Rr    = roll * pi / 180;
        CInfx = SetValf32((Fx - fx) / (cos(Rr) + 0.000001), ROV_Val.Fxmin, ROV_Val.Fxmax);
        CInfy = SetValf32((Fy - fy) / (cos(Rp) + 0.000001), ROV_Val.Fymin, ROV_Val.Fymax);
    }

    float kx, ky, Addkxy, kyaw, kz, kroll, Addkrz;
    //���ű�������
    kx     = GetThrottleKpower(CInfx, ROV_Val.Fxmin, ROV_Val.Fxmax);
    ky     = GetThrottleKpower(CInfy, ROV_Val.Fymin, ROV_Val.Fymax);
    kyaw   = GetThrottleKpower(CInfyaw, ROV_Val.Fyawmin, ROV_Val.Fyawmax);
    Addkxy = (OutputMax(kx, ky) + kyaw) - 1; //�ƽ�˶?���
    if (Addkxy > 0)
    {
        CInfx = CInfx * (kx - Addkxy) / (kx + 0.000001);
        CInfy = CInfy * (ky - Addkxy) / (ky + 0.000001);
    }
    //���²���
    CInfyaw = SetValf32(Fyaw + KxCompToYaw * CInfx, ROV_Val.Fyawmin, ROV_Val.Fyawmax);
    //z�Ჹ��
    fz    = CInfx * sin(Rr) - CInfy * sin(Rp);
    CInfz = SetValf32((Fz - fz) / (cos(Rp) * cos(Rr) + 0.000001), ROV_Val.Fzmin, ROV_Val.Fzmax);
    //	if(CInfz < 0){
    //			CInfz=CInfz*KxCopmToZo;
    //	}
    //	else{
    //		CInfz=1*CInfz;
    //	}
    CInfroll = SetValf32(Froll + KxCompToRoll * CInfx, ROV_Val.Frollmin, ROV_Val.Frollmax);
    //��ֱ����������
    kz     = GetThrottleKpower(CInfz, ROV_Val.Fzmin, ROV_Val.Fzmax);
    kroll  = GetThrottleKpower(CInfroll, ROV_Val.Frollmin, ROV_Val.Frollmax);
    Addkrz = kz + kroll - 1;
    //��ֱ����������
    if (Addkrz > 0)
    { //��С��������
        CInfroll = CInfroll * (kroll - Addkrz) / (kroll + 0.000001);
    }
    //��Ӧ��ÿ���ƽ���
    //ˮƽ
    ActiveForOxy(30, CInfx, CInfy, BufFoxy);
    ActiveForYaw(CInfyaw, BufFyaw);
    //��ת
    ActiveForZO(CInfz, BufFzo);
    ActiveForPitch(CInfpitch, BufFpitch);
    ActiveForRoll(CInfroll, BufFroll);

    if (motorcalflag)
    {
        //���а汾ת������ͬ�����Ŷ�Ӧ
        PROP1 = MotorPWMMidVal[0];
        PROP2 = MotorPWMMidVal[3];
        PROP3 = MotorPWMMidVal[2];
        PROP4 = MotorPWMMidVal[5];
        PROP5 = MotorPWMMidVal[1];
        PROP6 = MotorPWMMidVal[4];
    }
    else
    {
        PROP1 = ForceToESC(BufFoxy[1] + BufFzo[1] + BufFyaw[1] + BufFpitch[1] + BufFroll[1], -1, MotorPWMMidVal[0]); //��ǰ
        PROP2 = ForceToESC(BufFoxy[2] + BufFzo[2] + BufFyaw[2] + BufFpitch[2] + BufFroll[2], -1, MotorPWMMidVal[3]); //��ǰ
        PROP3 = ForceToESC(BufFoxy[3] + BufFzo[3] + BufFyaw[3] + BufFpitch[3] + BufFroll[3], -1, MotorPWMMidVal[2]); //���
        PROP4 = ForceToESC(BufFoxy[4] + BufFzo[4] + BufFyaw[4] + BufFpitch[4] + BufFroll[4], -1, MotorPWMMidVal[5]); //�Һ�
        PROP5 = ForceToESC(BufFoxy[5] + BufFzo[5] + BufFyaw[5] + BufFpitch[5] + BufFroll[5], -1, MotorPWMMidVal[1]); //����
        PROP6 = ForceToESC(BufFoxy[6] + BufFzo[6] + BufFyaw[6] + BufFpitch[6] + BufFroll[6], -1, MotorPWMMidVal[4]); //����
    }

    IIR_2OrderLpf_Init(&FLpfPWM5, 50, 4);
    IIR_2OrderLpf_Init(&FLpfPWM6, 50, 4);

    pca_setpwm(FL_MOTOR, 0, PROP1);                        //��ǰ
    pca_setpwm(FR_MOTOR, 0, PROP2);                        //��ǰ
    pca_setpwm(BL_MOTOR, 0, PROP3);                        //���
    pca_setpwm(BR_MOTOR, 0, PROP4);                        //�Һ�
                                                           //	pca_setpwm(UL_MOTOR,0,PROP5);//ZUO
                                                           //	pca_setpwm(UR_MOTOR,0,PROP6);//YOU
    pca_setpwm(UL_MOTOR, 0, SOTFOutput(&FLpfPWM5, PROP5)); //ZUO //FIXME SOTFOutput����ֵΪfloat,pca_setpwm�������Ϊu32
    pca_setpwm(UR_MOTOR, 0, SOTFOutput(&FLpfPWM6, PROP6)); //YOU
    MANIPULATOR_VAL(RoboHand_Pwm);                         //��е��
    ctrlpara_data.FR_val = PROP2;
    ctrlpara_data.FL_val = PROP1;
    ctrlpara_data.BR_val = PROP4;
    ctrlpara_data.BL_val = PROP3;
    ctrlpara_data.UL_val = FLpfPWM5.yk;
    ctrlpara_data.UR_val = FLpfPWM6.yk;
}

/**
  * @funNm  ThrottleToForceControl
  * @brief  ���ŵ�����ת�� �Լ��Ƿ���Ҫ��̬����
  * @param	Closestate: true:������ά����  false:�����в���
  * @param	Tx:x��������
  * @param	Ty:y��������
  * @param	Tz:z��������
  * @param	Tyaw:yawת������
  * @param	Tpitch:pitchת������
  * @param	Troll:rollת������
  * @param	yaw:��ǰyaw��
  * @param	pitch:��ǰpitch��
  * @param	roll:��ǰroll��
  * @retval void
  */
void ThrottleToForceControl(bool Closestate, float Tx, float Ty, float Tz, float Tyaw, float Tpitch, float Troll, float yaw, float pitch, float roll)
{
    float Fx, Fy;
    if (ROV_Val.Fxmax > ROV_Val.Fymax)
    { //ˮƽ��ͳһ�궨 ��֤������ �����ı궨
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

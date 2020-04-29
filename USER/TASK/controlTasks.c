/**
 * @author        :robosea
 * @version       :v1.0.0
 * @Date          :2019-12-16 11:15:47
 * @LastEditors:smake
 * @LastEditTime:2020-04-24 11:03:22
 * @brief         :
 */

#include "controlTasks.h"
#include "Kalman_filtering.h"
#include "math.h"
#include "Object.h"
#include "stmflash.h"

extern float   KxCompToRoll;
extern uint8_t g_nowWokeMode;

NLPID_Typedef       YawNLPID;
NLPID_Typedef       DeepNLPID;
NLPID_Typedef       RollNLPID;
CloseLoopPID_t      yawPID;
CloseLoopPID_t      rollPID;
CloseLoopPID_t      depthPID;
ControlParam_t      myctrlparam;
CtrlFeedBackParam_t ctrlreback;
float               SetTmpPressure = 0;
int8_t              flag           = 0;
AllPIDArg_typedef   AllPIDArgument;
Tottle_Crotypedef   TottleCro;
float               heading_ref = 0, depth_ref = 0, roll_ref = 0;
extern CtrlPara_t   ctrlpara_data;
bool                Closestate    = true;
float               depthspeed    = 0;
float               DeepOffset    = 38;
RockerLimit_t       RockerLimOY   = {0};
RockerLimit_t       RockerLimOX   = {0};
RockerLimit_t       RockerLimOYAW = {0};
RockerLimit_t       RockerLimZO   = {0};

/**
 * @function_name:doLostHeartBeat
 * @brief:没有收到控制消息，设备停机
 * @param :None
 * @return:None
 */
void doLostHeartBeat(void)
{
    g_runMode               = 0x01;
    myctrlparam.isRunMode   = 0x01;
    myctrlparam.FB_rocker   = 0;
    myctrlparam.LR_rocker   = 0;
    myctrlparam.TURN_rocker = 0;
    myctrlparam.UD_rocker   = 0;
}
SOTF_typeDef CloseLoopUD;

/**
 * @name:  ControlTask_Function
 * @msg: 控制任务函数
 * @param *argument
 * @return: None
 */
void ControlTask_Function(void const *argument)
{
    portTickType tick = xTaskGetTickCount();
    // AllPIDArgument = ;//TODO 修正从flash读写PID值
    NLPID_Init();
    Cal6FreedomForceValue(30, PROPFMin, PROPFMax);
    AllRocker_Init(500, 0.02, 100, -1000, 1000); //摇杆值转化成力函数参数初始化,限制死区，极值。
    IIR_2OrderLpf_Init(&CloseLoopUD, 50, 2);
    //计算周期10ms以内，8ms
    while (1)
    {
        run_time[4] = getRunTime(4);
        if (g_nowWokeMode == NOMALMODE)
        {
            if (xQueueReceive(Control_Message_Queue, &myctrlparam, 1))
            {
                // 摇杆通道赋值
                ctrlpara_data.fb_rocker   = myctrlparam.FB_rocker;   //前后，前正后负
                ctrlpara_data.lr_rocker   = myctrlparam.LR_rocker;   //左拐右拐，左负右正
                ctrlpara_data.ud_rocker   = myctrlparam.UD_rocker;   //上浮下潜，上正下负
                ctrlpara_data.turn_rocker = myctrlparam.TURN_rocker; //左右平移，左负右正
                g_runMode                 = myctrlparam.isRunMode;
            }
            if (g_LostPC) //若通信丢失则将控制模式调整为手动模式，且将摇杆值均设为0
            {
                doLostHeartBeat();
            }
            //		AllRocker_Init(yawPID.OutP,0.02,200,-1000,1000);
            TottleCro.Ty    = RockerValDispose(&RockerLimOY, myctrlparam.FB_rocker * 10.0, 5, 100);
            TottleCro.Tx    = RockerValDispose(&RockerLimOX, myctrlparam.LR_rocker * 10.0, 5, 100);
            TottleCro.Tz    = RockerValDispose(&RockerLimZO, myctrlparam.UD_rocker * 10.0, 5, 100);
            TottleCro.Tyaw  = RockerValDispose(&RockerLimOYAW, -1 * myctrlparam.TURN_rocker * 10.0, 5, 100);
            TottleCro.Tpith = 0;
            //            TottleCro.Tx    = TottleCro.Tx * 0.5; //转向限速
            //            roll_ref        = -4.6;
            if ((g_runMode & 0x04) || (g_runMode & 0x02)) //入水判断myctrlparam.depth > 5
            {
                TottleCro.Troll = NLPID_Control(&RollNLPID, myctrlparam.roll, myctrlparam.grayX, roll_ref);
            }
            else
            {
                NLPID_Clear(&RollNLPID);
                TottleCro.Troll = 0;
            }
            /* 定航 */
            if (g_runMode & 0x04)
            {
                if (fabs(myctrlparam.TURN_rocker) > 0)
                {
                    heading_ref = myctrlparam.yaw + YAWCODE2ANG(RockerLimit(-1 * myctrlparam.TURN_rocker, 10, -100, 100)); //将输入摇杆值转换成角度，并计算当前需要的航向值
                }
                ctrlpara_data.pid_head = heading_ref;
                TottleCro.Tyaw         = YAWPID_Control(&YawNLPID, 1 * myctrlparam.yaw, myctrlparam.grayZ, heading_ref); //heading_ref
            }
            else
            {
                NLPID_Clear(&YawNLPID);
                heading_ref = myctrlparam.yaw;
            }

            //深度控制
            if (g_runMode & 0x02) //&&(fabs(up_down_rocker)<40)
            {
                if (fabs(myctrlparam.UD_rocker) > 0)
                {
                    depth_ref = SOTFOutput(&CloseLoopUD, myctrlparam.depth + DEEPCODE2SPEED(RockerLimit(myctrlparam.UD_rocker * 1.0, 10, -100, 100)));
                }

                if (depth_ref < 0)
                {
                    depth_ref = 0;
                }
                if (flag == 1)
                {
                    depth_ref = SetTmpPressure;
                }
                ctrlpara_data.pid_updown = depth_ref * 0.1;
                //计算输出力
                TottleCro.Tz = NLPID_Control(&DeepNLPID, myctrlparam.depth, k_speed, depth_ref); //myctrlparam.UD_rocker*0.005 depth_ref,depthspeed深度为厘米单位depth_ref
                // TottleCro.Tz = NLPID_Control(&DeepNLPID, myctrlparam.depth - 17.5 * sinf(myctrlparam.pitch * PI / 180), k_speed, depth_ref); //myctrlparam.UD_rocker*0.005 depth_ref,depthspeed深度为厘米单位depth_ref
            }
            else
            {
                NLPID_Clear(&DeepNLPID);
                depth_ref = SOTFOutput(&CloseLoopUD, myctrlparam.depth);
            }

            //控制输出
            ThrottleToForceControl(Closestate, TottleCro.Tx, TottleCro.Ty, TottleCro.Tz, TottleCro.Tyaw, TottleCro.Tpith, TottleCro.Troll, myctrlparam.yaw, myctrlparam.pitch, myctrlparam.roll);

            vTaskDelayUntil(&tick, 20);
        }
        else
        {
            osDelay(100);
        }
    }
}

float YawErrorDispose(float now, float obj)
{
    float err = obj - now;
    if (err > 180)
    {
        return (180 - fabs(now) + (180 - obj) + obj);
    }
    else if (err < -180)
    {
        return (-1 * ((180 - fabs(now)) + (180 + obj)) + obj);
    }
    return now;
}

////////////闭环控制，添加
float NLSat(float x, float val)
{
    float output = 0;
    if (fabs(x) < val)
    {
        output = x / (val + 0.00001);
    }
    else
    {
        output = sign(x);
    }
    return output;
}

float NLfal(float x, float a, float val)
{
    float out;
    float Absx  = fabs(x);
    float signx = sign(x);
    if (Absx > 10000)
    {
        out = x;
    }
    else
    {
        out = signx * val * pow(Absx / val, 1.0 / a);
    }
    return out;
}

/**
 * @function_name:NLPID_Control
 * @brief:使用PID计算输出
 * @param *nl:PID值
 * @param Onow:外环反馈
 * @param Inow:内环反馈
 * @param obj:外环输入，即目标值
 * @return:None
 */
float NLPID_Control(NLPID_Typedef *nl, float Onow, float Inow, float obj)
{
    SOTFOutput(&nl->OIntf, obj);
    SOTFOutput(&nl->Ofbtf, Onow);
    SOTFOutput(&nl->Ifbtf, Inow);
    if (nl->OuterLoopEN == true) //如果使用外环控制
    {
        nl->Oe = nl->OIntf.yk - nl->Ofbtf.yk;
        if (fabs(nl->Oe) < nl->ZVal)
            return 0;                                                                          //死区控制
        nl->Oe1 = ((nl->OIntf.yk - nl->OIntf.yk_2) - (nl->Ofbtf.yk - nl->Ofbtf.yk_2)) / nl->T; //微分
        nl->OeI = nl->OeI + nl->Oe * nl->T;                                                    //积分
        nl->OeI = SetValf32(nl->OeI, -1 * nl->OeImax, nl->OeImax);                             //限幅
        if (fabs(nl->Oe) > nl->NVal)
        {
            nl->Ooutput = nl->OP * nl->NKx * nl->Oe + nl->OD * nl->Oe1;
            nl->OeI     = 0;
        }
        else
        {
            nl->Ooutput = nl->OP * nl->Oe +
                          nl->OI * nl->OeI +
                          nl->OD * nl->Oe1;
        }
        nl->Ooutput = SetValf32(nl->Ooutput, nl->OoutMin, nl->OoutMax); //限幅
        //内环
        SOTFOutput(&nl->Oouttf, nl->Ooutput);
        nl->Ie  = nl->Oouttf.yk - nl->Ifbtf.yk;
        nl->Ie1 = ((nl->Oouttf.yk - nl->Oouttf.yk_2) - (nl->Ifbtf.yk - nl->Ifbtf.yk_2)) / nl->T;
        nl->IeI = nl->IeI + nl->Ie * nl->T;                        //积分
        nl->IeI = SetValf32(nl->IeI, -1 * nl->IeImax, nl->IeImax); //限幅
        if (fabs(nl->Oe) > nl->NVal)
        {
            nl->IeI     = 0;
            nl->Ioutput = nl->IP * nl->NKx * NLfal(nl->Ie, nl->falnum, nl->INVal) +
                          nl->ID * NLfal(nl->Ie1, nl->falnum / 2, nl->INVal);
        }
        else
        {
            nl->Ioutput = nl->IP * NLfal(nl->Ie, nl->falnum, nl->INVal) +
                          nl->II * nl->IeI +
                          nl->ID * NLfal(nl->Ie1, nl->falnum / 2, nl->INVal);
        }
    }
    else
    //仅使用内环
    {
        SOTFOutput(&nl->Oouttf, obj);
        nl->Ie      = nl->Oouttf.yk - nl->Ifbtf.yk;
        nl->Ie1     = ((nl->Oouttf.yk - nl->Oouttf.yk_2) - (nl->Ifbtf.yk - nl->Ifbtf.yk_2)) / nl->T;
        nl->IeI     = nl->IeI + nl->Ie * nl->T;                        //积分
        nl->IeI     = SetValf32(nl->IeI, -1 * nl->IeImax, nl->IeImax); //限幅
        nl->Ioutput = nl->IP * NLfal(nl->Ie, nl->falnum, nl->INVal) +
                      nl->II * nl->IeI +
                      nl->ID * NLfal(nl->Ie1, nl->falnum / 2, nl->INVal);
    }
    return nl->Ioutput;
}

/**
 * @function_name:ProNLPID_Control
 * @brief:
 * @param *nl:
 * @param Onow:
 * @param Inow:
 * @param obj:
 * @return:None
 */
float ProNLPID_Control(NLPID_Typedef *nl, float Onow, float Inow, float obj)
{
    SOTFOutput(&nl->OIntf, obj);
    SOTFOutput(&nl->Ofbtf, Onow);
    SOTFOutput(&nl->Ifbtf, Inow);
    if (nl->OuterLoopEN == true) //如果使用外环控制
    {
        nl->Oe = nl->OIntf.yk - nl->Ofbtf.yk;
        if (fabs(nl->Oe) < nl->ZVal)
            return 0; //死区控制
        nl->Oe1 = ((nl->OIntf.yk - nl->OIntf.yk_2) - (nl->Ofbtf.yk - nl->Ofbtf.yk_2)) / nl->T;
        nl->OeI = nl->OeI + nl->Oe * nl->T;                        //积分
        nl->OeI = SetValf32(nl->OeI, -1 * nl->OeImax, nl->OeImax); //限幅
                                                                   //		nl->Ooutput=nl->OP*nl->Oe+nl->OI*nl->OeI+nl->OD*nl->Oe1;
        nl->Ooutput += (nl->T * (nl->OP * NLfal(nl->Oe1, nl->falnum, nl->INVal) + nl->OI * NLfal(nl->Oe, nl->falnum, nl->INVal)));
        nl->Ooutput = SetValf32(nl->Ooutput, nl->OoutMin, nl->OoutMax); //限幅
        //内环
        SOTFOutput(&nl->Oouttf, nl->Ooutput);
        nl->Ie      = nl->Oouttf.yk - nl->Ifbtf.yk;
        nl->Ie1     = ((nl->Oouttf.yk - nl->Oouttf.yk_2) - (nl->Ifbtf.yk - nl->Ifbtf.yk_2)) / nl->T;
        nl->IeI     = nl->IeI + nl->Ie * nl->T;                        //积分
        nl->IeI     = SetValf32(nl->IeI, -1 * nl->IeImax, nl->IeImax); //限幅
        nl->Ioutput = nl->IP * nl->Ie + nl->II * nl->IeI + nl->ID * nl->Ie1;
    }
    else
    //仅使用内环
    {
        SOTFOutput(&nl->Oouttf, obj);
        nl->Ie      = nl->Oouttf.yk - nl->Ifbtf.yk;
        nl->Ie1     = ((nl->Oouttf.yk - nl->Oouttf.yk_2) - (nl->Ifbtf.yk - nl->Ifbtf.yk_2)) / nl->T;
        nl->IeI     = nl->IeI + nl->Ie * nl->T;                        //积分
        nl->IeI     = SetValf32(nl->IeI, -1 * nl->IeImax, nl->IeImax); //限幅
        nl->Ioutput = nl->IP * nl->Ie + nl->II * nl->IeI + nl->ID * nl->Ie1;
    }
    return nl->Ioutput;
}

/**
 * @function_name:YAWPID_Control
 * @brief:
 * @param *nl:
 * @param Onow:
 * @param Inow:
 * @param obj:
 * @return:None
 */
float YAWPID_Control(NLPID_Typedef *nl, float Onow, float Inow, float obj)
{
    SOTFOutput(&nl->OIntf, obj);
    SOTFOutput(&nl->Ofbtf, Onow);
    SOTFOutput(&nl->Ifbtf, Inow);
    float CLHeading = YawErrorDispose(nl->Ofbtf.yk, nl->OIntf.yk);
    if (nl->OuterLoopEN == true) //如果开启外环控制
    {
        nl->Oe = nl->OIntf.yk - CLHeading;
        if (fabs(nl->Oe) < nl->ZVal)
            return 0; //死区控制
        nl->Oe1 = ((nl->OIntf.yk - nl->OIntf.yk_2) - (nl->Ofbtf.yk - nl->Ofbtf.yk_2)) / nl->T;
        nl->OeI = nl->OeI + nl->Oe * nl->T;                        //积分
        nl->OeI = SetValf32(nl->OeI, -1 * nl->OeImax, nl->OeImax); //限幅
        if (fabs(nl->Oe) > nl->NVal)
        {
            nl->Ooutput = nl->OP * nl->NKx * nl->Oe + nl->OD * nl->Oe1;
            nl->OeI     = 0;
        }
        else
        {
            nl->Ooutput = nl->OP * nl->Oe + nl->OI * nl->OeI + nl->OD * nl->Oe1;
        }
        nl->Ooutput = SetValf32(nl->Ooutput, nl->OoutMin, nl->OoutMax); //限幅
        //内环
        SOTFOutput(&nl->Oouttf, nl->Ooutput);
        nl->Ie  = nl->Oouttf.yk - nl->Ifbtf.yk;
        nl->Ie1 = ((nl->Oouttf.yk - nl->Oouttf.yk_2) - (nl->Ifbtf.yk - nl->Ifbtf.yk_2)) / nl->T;
        nl->IeI = nl->IeI + nl->Ie * nl->T;                        //积分
        nl->IeI = SetValf32(nl->IeI, -1 * nl->IeImax, nl->IeImax); //限幅
        if (fabs(nl->Oe) > nl->NVal)
        {
            nl->IeI     = 0;
            nl->Ioutput = nl->IP * nl->NKx * NLfal(nl->Ie, nl->falnum, nl->INVal) + nl->ID * NLfal(nl->Ie1, nl->falnum / 2, nl->INVal);
        }
        else
        {
            nl->Ioutput = nl->IP * NLfal(nl->Ie, nl->falnum, nl->INVal) + nl->II * nl->IeI + nl->ID * NLfal(nl->Ie1, nl->falnum / 2, nl->INVal);
            ;
        }
    }
    else
    //仅使用内环
    {
        SOTFOutput(&nl->Oouttf, obj);
        nl->Ie  = nl->Oouttf.yk - nl->Ifbtf.yk;
        nl->Ie1 = ((nl->Oouttf.yk - nl->Oouttf.yk_2) - (nl->Ifbtf.yk - nl->Ifbtf.yk_2)) / nl->T;
        nl->IeI = nl->IeI + nl->Ie * nl->T;                        //积分
        nl->IeI = SetValf32(nl->IeI, -1 * nl->IeImax, nl->IeImax); //限幅
                                                                   //		nl->Ioutput=nl->IP*nl->Ie+nl->II*nl->IeI+nl->ID*nl->Ie1;
        nl->Ioutput = nl->IP * NLfal(nl->Ie, nl->falnum, nl->INVal) + nl->II * nl->IeI + nl->ID * NLfal(nl->Ie1, nl->falnum / 2, nl->INVal);
    }
    return nl->Ioutput;
}

/**
 * @function_name:NLPID_Init
 * @brief:PID初始化赋值
 * @param None
 * @return:None
 */
void NLPID_Init(void)
{
    YawNLPID.OuterLoopEN = true;
    YawNLPID.T           = 0.02;
    YawNLPID.ZVal        = 0;
    YawNLPID.NVal        = 15;
    YawNLPID.NKx         = 1.5;
    /***外环输入***/

    YawNLPID.OIntf.num[0] = 1;
    YawNLPID.OIntf.num[1] = 0;
    YawNLPID.OIntf.num[2] = 0;
    YawNLPID.OIntf.den[0] = 1;
    YawNLPID.OIntf.den[1] = 0;
    YawNLPID.OIntf.den[2] = 0;

    /*********/
    /***外环反馈*15Hz***/
    IIR_2OrderLpf_Init(&YawNLPID.Ofbtf, 50, 10);
    /****航向外环PID*****/
    YawNLPID.OP      = 6;    //7.4
    YawNLPID.OI      = 0.02; //0.02;//
    YawNLPID.OD      = 0;
    YawNLPID.OeImax  = 100;
    YawNLPID.OoutMax = 2000;
    YawNLPID.OoutMin = -2000;
    /****航向内环PID******/
    YawNLPID.falnum = 1;
    YawNLPID.INVal  = 25;
    YawNLPID.IP     = 1.2;  //1.7;
    YawNLPID.II     = 0.06; //0.05;
    YawNLPID.ID     = 0;

    YawNLPID.IeImax = 300;
    /***内环输入****/
    YawNLPID.Oouttf.num[0] = 1;
    YawNLPID.Oouttf.num[1] = 0;
    YawNLPID.Oouttf.num[2] = 0;
    YawNLPID.Oouttf.den[0] = 1;
    YawNLPID.Oouttf.den[1] = 0;
    YawNLPID.Oouttf.den[2] = 0;
    /*********/
    /***内环反馈15Hz****/
    IIR_2OrderLpf_Init(&YawNLPID.Ifbtf, 50, 14);
    /***************************************************/
    DeepNLPID.OuterLoopEN = true;
    DeepNLPID.T           = 0.02;
    DeepNLPID.ZVal        = 0;
    DeepNLPID.NVal        = 20;
    DeepNLPID.NKx         = 1.5;
    /***外环输入*5Hz***/
    /*********/
    /***外环反馈*15Hz***/
    IIR_2OrderLpf_Init(&DeepNLPID.Ofbtf, 50, 10);

    DeepNLPID.OIntf.num[0] = 1;
    DeepNLPID.OIntf.num[1] = 0;
    DeepNLPID.OIntf.num[2] = 0;
    DeepNLPID.OIntf.den[0] = 1;
    DeepNLPID.OIntf.den[1] = 0;
    DeepNLPID.OIntf.den[2] = 0;
    /*********/
    DeepNLPID.OP      = 2;     //0.01;//0.02;//4;//0.215;//0.6;0.15,0.0043
    DeepNLPID.OI      = 0.003; //0.0003;//0.05;//0.25;//0.25;//0.1;0.005
    DeepNLPID.OD      = 0.0;
    DeepNLPID.OeImax  = 500; //100
    DeepNLPID.OoutMax = 1500;
    DeepNLPID.OoutMin = -1500;

    DeepNLPID.falnum = 1.1; //1.2
    DeepNLPID.INVal  = 50;
    DeepNLPID.IP     = 2;   //690;//270;//8;//9;8，0.1
    DeepNLPID.II     = 0.5; //0.45;//3.00;//0.1;//0.08;0.1
    DeepNLPID.ID     = 0;

    DeepNLPID.IeImax = 300;
    /***内环输入****/
    DeepNLPID.Oouttf.num[0] = 1;
    DeepNLPID.Oouttf.num[1] = 0;
    DeepNLPID.Oouttf.num[2] = 0;
    DeepNLPID.Oouttf.den[0] = 1;
    DeepNLPID.Oouttf.den[1] = 0;
    DeepNLPID.Oouttf.den[2] = 0;
    /*********/
    /***内环反馈15Hz****/
    IIR_2OrderLpf_Init(&DeepNLPID.Ifbtf, 50, 11);
    /*********/
    /********************************************/
    RollNLPID.OuterLoopEN = true;
    RollNLPID.T           = 0.02;
    RollNLPID.ZVal        = 0;
    RollNLPID.NVal        = 15;
    RollNLPID.NKx         = 1.5;
    /***外环输入*5Hz***/
    RollNLPID.OIntf.num[0] = 1;
    RollNLPID.OIntf.num[1] = 0;
    RollNLPID.OIntf.num[2] = 0;
    RollNLPID.OIntf.den[0] = 1;
    RollNLPID.OIntf.den[1] = 0;
    RollNLPID.OIntf.den[2] = 0;
    /*********/
    /***外环反馈*15Hz***/
    IIR_2OrderLpf_Init(&RollNLPID.Ofbtf, 50, 14);

    /*********/
    RollNLPID.OP      = 0;      //1.2;0.0025
    RollNLPID.OI      = 0.0000; //0.068;0.0001
    RollNLPID.OD      = 0;
    RollNLPID.OeImax  = 100;
    RollNLPID.OoutMax = 1500;
    RollNLPID.OoutMin = -1500;

    RollNLPID.falnum = 1;
    RollNLPID.INVal  = 25;
    RollNLPID.IP     = 2.4;
    RollNLPID.II     = 0.332;
    RollNLPID.ID     = 0.004;

    RollNLPID.IeImax = 300;
    /***内环输入****/
    RollNLPID.Oouttf.num[0] = 1;
    RollNLPID.Oouttf.num[1] = 0;
    RollNLPID.Oouttf.num[2] = 0;
    RollNLPID.Oouttf.den[0] = 1;
    RollNLPID.Oouttf.den[1] = 0;
    RollNLPID.Oouttf.den[2] = 0;
    /*********/
    /***内环反馈15Hz****/
    IIR_2OrderLpf_Init(&RollNLPID.Ifbtf, 50, 14);
    /*********/
}
/**
 * @function_name:NLPID_Clear
 * @brief:PID中间过程参数清零
 * @param :NLPID_Typedef *nl
 * @return:None
 */
void NLPID_Clear(NLPID_Typedef *nl)
{
    nl->OIntf.uk   = 0;
    nl->OIntf.uk_1 = 0;
    nl->OIntf.uk_2 = 0;
    nl->OIntf.yk   = 0;
    nl->OIntf.yk_1 = 0;
    nl->OIntf.yk_2 = 0;
    nl->Ofbtf.uk   = 0;
    nl->Ofbtf.uk_1 = 0;
    nl->Ofbtf.uk_2 = 0;
    nl->Ofbtf.yk   = 0;
    nl->Ofbtf.yk_1 = 0;
    nl->Ofbtf.yk_2 = 0;

    nl->Ifbtf.uk    = 0;
    nl->Ifbtf.uk_1  = 0;
    nl->Ifbtf.uk_2  = 0;
    nl->Ifbtf.yk    = 0;
    nl->Ifbtf.yk_1  = 0;
    nl->Ifbtf.yk_2  = 0;
    nl->Oouttf.uk   = 0;
    nl->Oouttf.uk_1 = 0;
    nl->Oouttf.uk_2 = 0;
    nl->Oouttf.yk   = 0;
    nl->Oouttf.yk_1 = 0;
    nl->Oouttf.yk_2 = 0;
    nl->IeI         = 0;
    nl->Ie1         = 0;
    nl->Ie          = 0;
    nl->OeI         = 0;
    nl->Oe1         = 0;
    nl->Oe          = 0;
    nl->Ooutput     = 0;
    nl->Ioutput     = 0;
}

void AllRocker_Init(float AccMax, float ts, float ZeroVal, float Rockermin, float Rockermax)
{
    RockerLimit_Init(&RockerLimOY, AccMax, ts, ZeroVal, Rockermin, Rockermax);
    RockerLimit_Init(&RockerLimOX, AccMax, ts, ZeroVal, Rockermin, Rockermax);
    RockerLimit_Init(&RockerLimOYAW, AccMax, ts, ZeroVal, Rockermin, Rockermax);
    RockerLimit_Init(&RockerLimZO, AccMax, ts, ZeroVal, Rockermin, Rockermax);
}

void RockerLimit_Init(RockerLimit_t *lim, float AccMax, float ts, float ZeroVal, float Rockermin, float Rockermax)
{
    lim->ts        = ts;
    lim->amax      = AccMax;
    lim->Rockermin = Rockermin;
    lim->Rockermax = Rockermax;
    lim->ZVal      = ZeroVal;
}

float RockerLimit(float RockerVal, float Zval, float min, float max)
{
    float Val, accKx = 1.0;
    if (fabs(RockerVal) <= (Zval + 0.00001))
    {
        Val = 0;
    }
    else if (RockerVal > 0)
    {
        Val = (RockerVal - Zval) * (max - min) / ((max - min) - 2 * Zval);
    }
    else if (RockerVal < 0)
    {
        Val = (RockerVal + Zval) * (max - min) / ((max - min) - 2 * Zval);
    }
    else
    {
        Val = 0;
    }
    return Val;
}

/**
 * @function_name:RockerValDispose()
 * @brief:摇杆值处理转化成力函数
 * @param *lim          ：限制结构体指针
 * @param RockerVal     :摇杆值
 * @param RZeroAccKx    :摇杆拨动加速度系数
 * @param RockerKx      :摇杆值系数
 * @return:None
 */
float RockerValDispose(RockerLimit_t *lim, float RockerVal, float RZeroAccKx, float RockerKx)
{
    float Val = 0, accKx = 1.0;
    if (fabs(RockerVal) <= (lim->ZVal + 0.00001))
    {
        Val   = 0;
        accKx = RZeroAccKx;
    }
    else if (RockerVal > 0)
    {
        Val = (RockerVal - lim->ZVal) * (lim->Rockermax - lim->Rockermin) / ((lim->Rockermax - lim->Rockermin) - 2 * lim->ZVal);
    }
    else if (RockerVal < 0)
    {
        Val = (RockerVal + lim->ZVal) * (lim->Rockermax - lim->Rockermin) / ((lim->Rockermax - lim->Rockermin) - 2 * lim->ZVal);
    }
    else
    {
        Val = 0;
    }
    /**??***/
    lim->a = (Val - lim->lastvel) / lim->ts;
    if (lim->a > (lim->amax * accKx))
        lim->a = (lim->amax * accKx);
    else if (lim->a < -(lim->amax * accKx))
        lim->a = -(lim->amax * accKx);
    if (fabs(lim->fa - Val) < 0.00001)
    {
        lim->a = 0;
    }
    lim->fa      = lim->fa + lim->a * lim->ts;
    lim->nowv    = lim->fa;
    lim->lastvel = lim->fa;
    return RockerKx * lim->nowv / 100;
}
///////////////////////////

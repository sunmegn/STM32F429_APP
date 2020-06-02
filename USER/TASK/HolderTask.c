#include "HolderTask.h"

#include "controlTasks.h"
#include "Object.h"
//#include "system.h"
//#include "AT24CXX.h"
#include "messageTasks.h"
#include "MTLink.h"


HolderParam_t    HolderParam;
QueueHandle_t    HolderBackQueue = NULL;
Holder_t         Holder;
HolderPosParam_t PosParam = {.midpos = 286000, .maxang = 60, .minang = -60};

extern QueueHandle_t CtrlToHolderQueue;
extern QueueHandle_t ObjectToHolderQueue; //->HolderTask
extern SystemParam_t SystemParam;

void HolderBackQueue_Init(void)
{
    do
    {
        HolderBackQueue = xQueueCreate(1, sizeof(HolderParam));
    } while (HolderBackQueue == NULL);
}

void HolderTask_func(void const *argument)
{
    HolderCAN1Queues_Init();
    HolderBackQueue_Init();
    //ObjectToHolderQueue_Init();
    float codeval;
    char  HolderInitSate = 0;

    while (1)
    {
        /*任务时间获取*/
        //SystemState.TaskRunTime.taskruntime[HolderTask] = getRunTime(HolderTask);

        HolderInitSate = 0;
        //if (g_DeckState == 1)
			 if ( 1)
        {
            //MTLinkMsgPrintf(MY_ID, HOST_ID, "Begin HolderInit!");

            /**云台初始化**/
            if (ZERO_Init(8000) == 0x0f)
            {
                HolderInitSate = 1;
                //MTLinkMsgPrintf(MY_ID, HOST_ID, "HolderInit Finish!");
            }
            else
            {
                HolderInitSate    = 0;
                uint8_t SW        = 0x00;
                SW                = ZERO_GetStat(1, 50, 4);
                HolderParam.vaild = SW;
                //MTLinkMsgPrintf(MY_ID, HOST_ID, "HolderInit Fail!  SW:0x%02x", SW);
            }

            /**初始云台位置**/
            if (HoldePos_Init() && HolderInitSate == 1)
            {
                //MTLinkMsgPrintf(MY_ID, HOST_ID, "HolderPosInit Finish!");
                //MTLinkMsgPrintf(MY_ID, HOST_ID, "HolderReady!");
                HolderInitSate = 1;
            }
            else
            {
                //MTLinkMsgPrintf(MY_ID, HOST_ID, "HolderPosInit Fail!");
                HolderParam.vaild = 0x41;
                HolderInitSate    = 0;
            }

            if (gc_initzero_BUTTON == 1 && HolderInitSate == 0)
            {
                gc_initzero_BUTTON = 0;
                //MTLinkMsgPrintf(MY_ID, HOST_ID, "HolderSet ERR!");
            }

            HolderInitSate = 1;
            while (HolderInitSate)
            {
                /*任务时间获取*/
                //SystemState.TaskRunTime.taskruntime[HolderTask] = getRunTime(HolderTask);

                /*云台电源判断*/
//                if (g_DeckState == 0)
//                    break;

                if (g_nowWokeMode == DEBUGMODE)
                {
                    CAN1_Test_Demo();
                }

                else if (g_nowWokeMode == NOMALMODE || g_nowWokeMode == TESTMODE)
                {
                    /**从数据管理线程 获取到控制数据**/
                    if (CtrlToHolderQueue)
                    {
                        if (xQueuePeek(CtrlToHolderQueue, &HolderParam, 1) == pdTRUE)
                        {
                            /**控制云台运动**/
                            ZERO_Set_Angle(ValToAngle(HolderParam.setpos, PosParam.maxang, PosParam.minang), PosParam.midpos, PosParam.maxang, PosParam.minang);
                        }
                    }

                    HolderParam.CW = CWSOandENOperation;                      //设定控制字
                    HolderParam.SW = ZERO_GetDriverState(CWSOandENOperation); //状态机切换状态  返回当前状态;

                    /**获取当前位置**/
                    HolderParam.getpos = -ZERO_Get_Angle(PosParam.midpos, PosParam.maxang, PosParam.minang);
                    HolderParam.nowcnt = ZERO_Get_Pos();

                    /**返回云台位置 状态 到数据管理**/
                    if (HolderBackQueue)
                        xQueueOverwrite(HolderBackQueue, &HolderParam);

                    /*云台中值校准*/
                    if (gc_initzero_BUTTON == 1 && HolderInitSate == 1)
                    {
                        /**从上位机(OBJ) 获取校准数据**/
                        if (ObjectToHolderQueue)
                        {
                            if (xQueuePeek(ObjectToHolderQueue, &Holder, 1) == pdTRUE)
                            {
                                /**获取当前位置单位:cnt   赋值校准数据**/
                                codeval         = ZERO_Get_Pos();
                                PosParam.midpos = codeval;
                                PosParam.maxang = Holder.maxaddpos;
                                PosParam.minang = Holder.minaddpos;

                                /**根据校准数据 设定云台电机的最大最小安全位置 并保存到电机驱动器中**/
                                ZERO_SaveSaftyPos(codeval, Holder.maxaddpos, Holder.minaddpos);

                                /**将云台可运动的最大最小角度 及中值位置cnt  保存到EEPROM中**/
                                SystemParam.holder_mid.midholder = PosParam.midpos;
                                SystemParam.holder_mid.maxang    = PosParam.maxang;
                                SystemParam.holder_mid.minang    = PosParam.minang;
                                //EEPROM_Save((u8 *)&SystemParam.holder_mid, sizeof(SystemParam.holder_mid), (u8 *)&SystemParam, EEPROM_OFFEST, 3);

                                gc_initzero_BUTTON = 0;
                                //MTLinkMsgPrintf(MY_ID, HOST_ID, "HolderSet OK!  MID:%d MAX:%d MIN:%d", PosParam.midpos, PosParam.maxang, PosParam.minang);
                            }
                        }
                    }
                    else if (gc_initzero_BUTTON == 1 && HolderInitSate == 0)
                    {
                        gc_initzero_BUTTON = 0;
                        //MTLinkMsgPrintf(MY_ID, HOST_ID, "HolderSet ERR!");
                    }

                    /*云台控制延迟测试*/
//                    if (HolderParam.TimeTestF == 2)
//                    {
//                        MTLink_Encode(&MTLink_UDP, MY_ID, HOST_ID, 0 /*不需要应答*/, CMD_TIMETEST_ID, (uint8_t *)&SysDateTime, sizeof(DateTime_t), 10); //上传PC显示
//                        //g_TimeTestF           = 0;
//                        HolderParam.TimeTestF = 0;
//                    }

                    /*云台动作异常判断*/
                    if (HolderPosERRCheak(ValToAngle(HolderParam.setpos, PosParam.maxang, PosParam.minang), -HolderParam.getpos, 5, 500))
                    { //云台未正常动作 重新初始
                        //MTLinkMsgPrintf(MY_ID, HOST_ID, "Holder ERR!! SET:%0.3f GET:%0.3f", ValToAngle(HolderParam.setpos, PosParam.maxang, PosParam.minang), -HolderParam.getpos);
                        HolderParam.vaild = 0x41;
                        break;
                    }
                }
                //GetSensorState(HOLDER_ID, HolderParam.vaild, 0x0f, 1000, &SystemState);

                HAL_IWDG_Refresh(&hiwdg); //超过两秒不喂狗，复位重启32k，32, 2000
                vTaskDelay(100);
            }
        }
        //GetSensorState(HOLDER_ID, HolderParam.vaild, 0x0f, 1000, &SystemState);

        HAL_IWDG_Refresh(&hiwdg); //超过两秒不喂狗，复位重启32k，32, 2000
        vTaskDelay(100);
    }
}

//void HolderE2CheckInit(void)
//{
//    PosParam.midpos = SystemParam.holder_mid.midholder;
//    PosParam.maxang = SystemParam.holder_mid.maxang;
//    PosParam.minang = SystemParam.holder_mid.minang;
//}

uint8_t HoldePos_Init(void)
{
    uint32_t codeval  = 0;
    int16_t  angle    = 0;
    uint32_t Basetime = HAL_GetTick();
    uint32_t nowtime  = Basetime;

    if (CAN1RxQueue)
    {
        //初始云台角度
        ZERO_Set_Angle(0, PosParam.midpos, PosParam.maxang, PosParam.minang);
    }
    do
    {
        angle = -ZERO_Get_Angle(PosParam.midpos, PosParam.maxang, PosParam.minang);
    } while (fabs((float)angle) >= 2 && (nowtime - Basetime) < 10000);
    if (nowtime - Basetime > 10000)
        return 0;
    return 1;
}

uint8_t HolderPosERRCheak(float setpos, float readpos, float val, int times)
{
    static int cnt;

    if (fabs(setpos - readpos) >= val)
        cnt++;
    else
        cnt = 0;

    if (cnt >= times)
    {
        cnt = 0;
        return 1;
    }

    return 0;
}

float ValToAngle(int16_t val, int16_t maxang, int16_t minang)
{
    if (val > 0)
        return val / 100.0 * fabs((float)maxang);
    else if (val < 0)
        return val / 100.0 * fabs((float)minang);
    else
        return val;
}

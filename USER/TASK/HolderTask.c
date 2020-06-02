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
        /*����ʱ���ȡ*/
        //SystemState.TaskRunTime.taskruntime[HolderTask] = getRunTime(HolderTask);

        HolderInitSate = 0;
        //if (g_DeckState == 1)
			 if ( 1)
        {
            //MTLinkMsgPrintf(MY_ID, HOST_ID, "Begin HolderInit!");

            /**��̨��ʼ��**/
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

            /**��ʼ��̨λ��**/
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
                /*����ʱ���ȡ*/
                //SystemState.TaskRunTime.taskruntime[HolderTask] = getRunTime(HolderTask);

                /*��̨��Դ�ж�*/
//                if (g_DeckState == 0)
//                    break;

                if (g_nowWokeMode == DEBUGMODE)
                {
                    CAN1_Test_Demo();
                }

                else if (g_nowWokeMode == NOMALMODE || g_nowWokeMode == TESTMODE)
                {
                    /**�����ݹ����߳� ��ȡ����������**/
                    if (CtrlToHolderQueue)
                    {
                        if (xQueuePeek(CtrlToHolderQueue, &HolderParam, 1) == pdTRUE)
                        {
                            /**������̨�˶�**/
                            ZERO_Set_Angle(ValToAngle(HolderParam.setpos, PosParam.maxang, PosParam.minang), PosParam.midpos, PosParam.maxang, PosParam.minang);
                        }
                    }

                    HolderParam.CW = CWSOandENOperation;                      //�趨������
                    HolderParam.SW = ZERO_GetDriverState(CWSOandENOperation); //״̬���л�״̬  ���ص�ǰ״̬;

                    /**��ȡ��ǰλ��**/
                    HolderParam.getpos = -ZERO_Get_Angle(PosParam.midpos, PosParam.maxang, PosParam.minang);
                    HolderParam.nowcnt = ZERO_Get_Pos();

                    /**������̨λ�� ״̬ �����ݹ���**/
                    if (HolderBackQueue)
                        xQueueOverwrite(HolderBackQueue, &HolderParam);

                    /*��̨��ֵУ׼*/
                    if (gc_initzero_BUTTON == 1 && HolderInitSate == 1)
                    {
                        /**����λ��(OBJ) ��ȡУ׼����**/
                        if (ObjectToHolderQueue)
                        {
                            if (xQueuePeek(ObjectToHolderQueue, &Holder, 1) == pdTRUE)
                            {
                                /**��ȡ��ǰλ�õ�λ:cnt   ��ֵУ׼����**/
                                codeval         = ZERO_Get_Pos();
                                PosParam.midpos = codeval;
                                PosParam.maxang = Holder.maxaddpos;
                                PosParam.minang = Holder.minaddpos;

                                /**����У׼���� �趨��̨����������С��ȫλ�� �����浽�����������**/
                                ZERO_SaveSaftyPos(codeval, Holder.maxaddpos, Holder.minaddpos);

                                /**����̨���˶��������С�Ƕ� ����ֵλ��cnt  ���浽EEPROM��**/
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

                    /*��̨�����ӳٲ���*/
//                    if (HolderParam.TimeTestF == 2)
//                    {
//                        MTLink_Encode(&MTLink_UDP, MY_ID, HOST_ID, 0 /*����ҪӦ��*/, CMD_TIMETEST_ID, (uint8_t *)&SysDateTime, sizeof(DateTime_t), 10); //�ϴ�PC��ʾ
//                        //g_TimeTestF           = 0;
//                        HolderParam.TimeTestF = 0;
//                    }

                    /*��̨�����쳣�ж�*/
                    if (HolderPosERRCheak(ValToAngle(HolderParam.setpos, PosParam.maxang, PosParam.minang), -HolderParam.getpos, 5, 500))
                    { //��̨δ�������� ���³�ʼ
                        //MTLinkMsgPrintf(MY_ID, HOST_ID, "Holder ERR!! SET:%0.3f GET:%0.3f", ValToAngle(HolderParam.setpos, PosParam.maxang, PosParam.minang), -HolderParam.getpos);
                        HolderParam.vaild = 0x41;
                        break;
                    }
                }
                //GetSensorState(HOLDER_ID, HolderParam.vaild, 0x0f, 1000, &SystemState);

                HAL_IWDG_Refresh(&hiwdg); //�������벻ι������λ����32k��32, 2000
                vTaskDelay(100);
            }
        }
        //GetSensorState(HOLDER_ID, HolderParam.vaild, 0x0f, 1000, &SystemState);

        HAL_IWDG_Refresh(&hiwdg); //�������벻ι������λ����32k��32, 2000
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
        //��ʼ��̨�Ƕ�
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

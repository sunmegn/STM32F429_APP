/**
 * @author        :robosea
 * @version       :v1.0.0
 * @Date          :2020-02-19 14:33:17
 * @LastEditors:smake
 * @LastEditTime:2020-04-20 20:48:44
 * @brief         :
 */
#include "Object.h"
#include "IAP.h"
#include "MTLink.h"
#include "Version.h"
#include "global.h"
#include "messageTasks.h"
#include "stmflash.h"
#include "manipulaterD2.h"
/*******************��������**********************/
static mtlink_all_CMD_t  ctrl_cmd;
extern AllPIDArg_typedef AllPIDArgument;    //���ڴ�����е�PID����ֵ
CloseLoopPID_t           PIDFromPC;         //���ڽ�����λ���������ĵ�����ֵ
extern NLPID_Typedef     YawNLPID;          //controlTask�ж����ʹ�õ�YawPIDֵ
extern NLPID_Typedef     DeepNLPID;         //controlTask�ж����ʹ�õ�DeepPIDֵ
extern NLPID_Typedef     RollNLPID;         //controlTask�ж����ʹ�õ�RollPIDֵ
extern QueueHandle_t     DataManageQueue;   //������λ������ָ��
extern int               RoboHand_Pwm;      //���û�е��PWMֵ
extern u8                motorcalflag;      //����궨��־
extern u16               MotorPWMMidVal[6]; //��������ʹ�õ�PWM��ֵ
MotorMidVal_t            PWMMidFromPC;      //���ڽ�����λ�����������ƽ�����ֵ
int                      manipulater_period = 2;
int                      manipulater_times  = 0;
union {
float float_val[2];
	uint8_t char_val [8]
}manipulater_position;
extern float RsCmd_position_Limit_P[2];//�ƶ���
extern float RsCmd_position_Limit_R[2];//ת����

/**
 * @function_name:MTLinkDispose
 * @brief:�ж������Ƿ�Ϊ��λ������
 * @param SID:Դ��ַ
 * @param DID:Ŀ���ַ
 * @param obj:�������
 * @param *buf:���ݵ�ַ
 * @param len:���ݳ���
 * @return:None
 */
bool MTLinkDispose(uint8_t SID, uint8_t DID, uint16_t obj, uint8_t *buf, int len)
{
    bool state = true;
    switch (DID)
    {
    case MY_ID: //�����Լ���
        switch (SID)
        {
        case HOST_ID: //������PC����λ������������
            PC_MasterDispose(false, SID, obj, buf, len);
            break;
        default:
            break;
        }
        break;
    case CAN_IMU_ID: //ת�ӵ�����CAN����ID��
        MTLinkToCANConnect(DID, obj, buf, len);
    default:
        break;
    }
    return state;
}
/**
 * @function_name:MTLinkToCANConnect
 * @brief:MTLinkЭ��תCANЭ��
 * @param :None
 * @return:None
 */
void MTLinkToCANConnect(uint8_t DID, uint16_t obj, uint8_t *buf, uint32_t len)
{
    SampleSendBufToCANBus(DID, obj, buf, len, 300);
}
// CAN_OBJ
bool CANObject_Dispose(int Master_ID, uint16_t object, uint8_t Sub_ID, char RorW, uint8_t *buf, uint32_t *len)
{
    uint16_t RxObj = object;
    uint32_t SumID = Master_ID * 1000 + Sub_ID;
    switch (SumID) //ת��Object
    {
    case MY_ID * 1000:
        PC_MasterDispose(true, HOST_ID, RxObj, buf, (int)len[0]);
        break;
    default:
        break;
    }
    return true;
}
/**
 * @function_name:PC_MasterDispose
 * @brief:��MTLink���ݽ�����ִ��
 * @param obj:�������
 * @param *buf:���ݵ�ַ
 * @param len:���ݳ���
 * @return:None
 */
bool PC_MasterDispose(bool IsCAN, uint8_t SID, uint16_t obj, uint8_t *buf, int len)
{
    u8 rxbuf[100];
    if (len > 100)
        return false;
    memcpy(rxbuf, buf, len);
    g_HeartBeatCnt = 0;
    switch (obj)
    {
    case CMD_REBOOT_ID: //        		0x2000
        //�����λ
        NVIC_SystemReset();
        break;
    case CMD_DEBUG_ID: //        		0x2001
        //�������ģʽ
        g_nowWokeMode = DEBUGMODE;
        break;
    case CMD_VERSION_ID: //      		0x2002
        //��������汾��
        LoadVersion(&MyVersion, 1.0, "SunMeng", 2020, 4, 10, 12, 23);
        MTLink_Encode(&MTLink_UDP, MY_ID, HOST_ID, 0 /*����ҪӦ��*/, CMD_VERSION_ID, (uint8_t *)&MyVersion, sizeof(AUV_Version_Typedef), 10);
        break;
    case CMD_GOTOBL_ID: //        	0x2004
                        //��ת��BootLoader
        Jump_To_Bootloader();
        break;
    case CMD_ALL_ID: //              0x3006
        memcpy(&ctrl_cmd, rxbuf, sizeof(mtlink_all_CMD_t));
        ctrl_cmd.UpDown = (-1) * ctrl_cmd.UpDown;
        LED_SetPwm(CONSTRAIN(ctrl_cmd.light * 50, 5000, 0));
        static int last_ptz = 0;
        //        ctrl_cmd.ptz        = CONSTRAIN(ctrl_cmd.ptz, last_ptz + 5, last_ptz - 5);
        YunTai_SetPwm(CONSTRAIN(ctrl_cmd.ptz * 10 + 1100, 1900, 1100)); //��������ͷ���
        if (ctrl_cmd.manipulator[0] | ctrl_cmd.manipulator[1])
        {
            if (manipulater_times >= manipulater_period)
            {
				manipulater_position.float_val[0] += ((float)ctrl_cmd.manipulator[0]/5.f);
				manipulater_position.float_val[0]=ConstrainFloat(manipulater_position.float_val[0],RsCmd_position_Limit_P[0],RsCmd_position_Limit_P[1]);
				manipulater_D2_send(0x01, RS_CMD_POSITION_SET, 0, &manipulater_position.char_val[0], 4);
				osDelay(5);
				manipulater_position.float_val[1] += ((float)ctrl_cmd.manipulator[1]/5.f);
				manipulater_position.float_val[1]=ConstrainFloat(manipulater_position.float_val[1],RsCmd_position_Limit_R[0],RsCmd_position_Limit_R[1]);
				manipulater_D2_send(0x02, RS_CMD_POSITION_SET, 0, &manipulater_position.char_val[4], 4);
		
                manipulater_times       = 0;
            }
            manipulater_times++;
        }
        if (DataManageQueue)
            xQueueOverwrite(DataManageQueue, &ctrl_cmd);
        break;
    case CMD_CALIBRATION_ID: //      0x6000
    {
        //�궨�ƽ�����ֵ���Ƚ�����ģʽ��ΪDebugģʽ��Ȼ��ֱ�������ƽ���PWMֵ
        memcpy(&PWMMidFromPC, rxbuf, sizeof(PWMMidFromPC));
        switch (PWMMidFromPC.saveflag) //�ж϶����ģ�д
        {
        case 3:
            taskENTER_CRITICAL(); //�����ٽ�������ֹ���flash���
            STMFLASH_Read(PARAMADDR, MotorPWMMidVal, 6);
            taskEXIT_CRITICAL(); //�˳��ٽ���
            motorcalflag              = 0;
            PWMMidFromPC.motormidpwm1 = MotorPWMMidVal[0];
            PWMMidFromPC.motormidpwm2 = MotorPWMMidVal[1];
            PWMMidFromPC.motormidpwm3 = MotorPWMMidVal[2];
            PWMMidFromPC.motormidpwm4 = MotorPWMMidVal[3];
            PWMMidFromPC.motormidpwm5 = MotorPWMMidVal[4];
            PWMMidFromPC.motormidpwm6 = MotorPWMMidVal[5];
            MTLink_Encode(&MTLink_UDP, MY_ID, HOST_ID, 0 /*����ҪӦ��*/, CMD_CALIBRATION_ID, (uint8_t *)&PWMMidFromPC, sizeof(MotorMidVal_t), 10); //�ϴ�PC��ʾ
            break;
        case 2:
            MotorPWMMidVal[0] = PWMMidFromPC.motormidpwm1;
            MotorPWMMidVal[1] = PWMMidFromPC.motormidpwm2;
            MotorPWMMidVal[2] = PWMMidFromPC.motormidpwm3;
            MotorPWMMidVal[3] = PWMMidFromPC.motormidpwm4;
            MotorPWMMidVal[4] = PWMMidFromPC.motormidpwm5;
            MotorPWMMidVal[5] = PWMMidFromPC.motormidpwm6;
            motorcalflag      = 1;
            break;
        case 1:
            MotorPWMMidVal[0] = PWMMidFromPC.motormidpwm1;
            MotorPWMMidVal[1] = PWMMidFromPC.motormidpwm2;
            MotorPWMMidVal[2] = PWMMidFromPC.motormidpwm3;
            MotorPWMMidVal[3] = PWMMidFromPC.motormidpwm4;
            MotorPWMMidVal[4] = PWMMidFromPC.motormidpwm5;
            MotorPWMMidVal[5] = PWMMidFromPC.motormidpwm6;
            taskENTER_CRITICAL();    //�����ٽ�������ֹ���flash���
            STMFLASH_Erase_DATA(23); //�������ݴ洢��
            osDelay(20);
            STMFLASH_Write(PARAMADDR, MotorPWMMidVal, 6);
            taskEXIT_CRITICAL();
            motorcalflag = 0;
            break;
        default:
            break;
        }
    }
    case CMD_SAVECAL_ID: //        	0x6001
    {
        switch (rxbuf[0]) //�ж϶����ģ�д
        {
        case 3:
            break;
        case 2:
            break;
        case 1:
            break;
        default:
            break;
        }
    }
    case CMD_YAWPID_ID: //0x6002
    {
        memcpy(&PIDFromPC, rxbuf, sizeof(PIDFromPC));
        switch (PIDFromPC.saveflag) //�ж϶����ģ�д
        {
        case 3:
            PIDFromPC.OutP = YawNLPID.OP;
            PIDFromPC.OutI = YawNLPID.OI;
            PIDFromPC.OutD = YawNLPID.OD;
            PIDFromPC.InP  = YawNLPID.IP;
            PIDFromPC.InI  = YawNLPID.II;
            PIDFromPC.InD  = YawNLPID.ID;
            MTLink_Encode(&MTLink_UDP, MY_ID, HOST_ID, 0 /*����ҪӦ��*/, CMD_YAWPID_ID, (uint8_t *)&PIDFromPC, sizeof(PIDFromPC), 10); //�ϴ�PC��ʾ
            break;
        case 2:
            YawNLPID.OP = PIDFromPC.OutP;
            YawNLPID.OI = PIDFromPC.OutI;
            YawNLPID.OD = PIDFromPC.OutD;
            YawNLPID.IP = PIDFromPC.InP;
            YawNLPID.II = PIDFromPC.InI;
            YawNLPID.ID = PIDFromPC.InD;
            break;
        case 1:
            YawNLPID.OP = PIDFromPC.OutP;
            YawNLPID.OI = PIDFromPC.OutI;
            YawNLPID.OD = PIDFromPC.OutD;
            YawNLPID.IP = PIDFromPC.InP;
            YawNLPID.II = PIDFromPC.InI;
            YawNLPID.ID = PIDFromPC.InD;
            //TODO ��ӱ��湦��
            break;
        default:
            break;
        }
    }
    case CMD_DEPTHPID_ID: //0x6003
    {
        memcpy(&PIDFromPC, rxbuf, sizeof(PIDFromPC));
        switch (PIDFromPC.saveflag) //�ж϶����ģ�д
        {
        case 3:
            PIDFromPC.OutP = DeepNLPID.OP;
            PIDFromPC.OutI = DeepNLPID.OI;
            PIDFromPC.OutD = DeepNLPID.OD;
            PIDFromPC.InP  = DeepNLPID.IP;
            PIDFromPC.InI  = DeepNLPID.II;
            PIDFromPC.InD  = DeepNLPID.ID;
            MTLink_Encode(&MTLink_UDP, MY_ID, HOST_ID, 0 /*����ҪӦ��*/, CMD_DEPTHPID_ID, (uint8_t *)&PIDFromPC, sizeof(PIDFromPC), 10); //�ϴ�PC��ʾ
            break;
        case 2:
            DeepNLPID.OP = PIDFromPC.OutP;
            DeepNLPID.OI = PIDFromPC.OutI;
            DeepNLPID.OD = PIDFromPC.OutD;
            DeepNLPID.IP = PIDFromPC.InP;
            DeepNLPID.II = PIDFromPC.InI;
            DeepNLPID.ID = PIDFromPC.InD;
            break;
        case 1:
            DeepNLPID.OP = PIDFromPC.OutP;
            DeepNLPID.OI = PIDFromPC.OutI;
            DeepNLPID.OD = PIDFromPC.OutD;
            DeepNLPID.IP = PIDFromPC.InP;
            DeepNLPID.II = PIDFromPC.InI;
            DeepNLPID.ID = PIDFromPC.InD;
            //TODO ��ӱ��湦��
            break;
        default:
            break;
        }
    }
    case CMD_ROLLPID_ID: //0x6004
    {
        memcpy(&PIDFromPC, rxbuf, sizeof(PIDFromPC));
        switch (PIDFromPC.saveflag) //�ж϶����ģ�д
        {
        case 3:
            PIDFromPC.OutP = RollNLPID.OP;
            PIDFromPC.OutI = RollNLPID.OI;
            PIDFromPC.OutD = RollNLPID.OD;
            PIDFromPC.InP  = RollNLPID.IP;
            PIDFromPC.InI  = RollNLPID.II;
            PIDFromPC.InD  = RollNLPID.ID;
            MTLink_Encode(&MTLink_UDP, MY_ID, HOST_ID, 0 /*����ҪӦ��*/, CMD_ROLLPID_ID, (uint8_t *)&PIDFromPC, sizeof(PIDFromPC), 10); //�ϴ�PC��ʾ
            break;
        case 2:
            RollNLPID.OP = PIDFromPC.OutP;
            RollNLPID.OI = PIDFromPC.OutI;
            RollNLPID.OD = PIDFromPC.OutD;
            RollNLPID.IP = PIDFromPC.InP;
            RollNLPID.II = PIDFromPC.InI;
            RollNLPID.ID = PIDFromPC.InD;
            break;
        case 1:
            RollNLPID.OP = PIDFromPC.OutP;
            RollNLPID.OI = PIDFromPC.OutI;
            RollNLPID.OD = PIDFromPC.OutD;
            RollNLPID.IP = PIDFromPC.InP;
            RollNLPID.II = PIDFromPC.InI;
            RollNLPID.ID = PIDFromPC.InD;
            //TODO ��ӱ��湦��
            break;
        default:
            break;
        }
    }
    case CMD_HOSTHEARTBEAT_ID: //        0xF000
        g_HeartBeatCnt = 0;
        break;
    default:
        break;
    }
    return true;
}

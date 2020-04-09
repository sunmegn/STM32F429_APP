/**
 * @author        :robosea
 * @version       :v1.0.0
 * @Date          :2020-02-19 14:33:17
 * @LastEditors:Robosea
 * @LastEditTime:2020-03-02 16:06:25
 * @brief         :
 */
#include "Object.h"
#include "IAP.h"
#include "MTLink.h"
#include "Version.h"
#include "global.h"
#include "messageTasks.h"

extern CloseLoopPID_t yawPID;
extern CloseLoopPID_t rollPID;
extern CloseLoopPID_t depthPID;

extern NLPID_Typedef YawNLPID;
extern NLPID_Typedef DeepNLPID;
extern NLPID_Typedef RollNLPID;
extern QueueHandle_t DataManageQueue;
//static ControlParam_t ctrlparam;
extern QueueHandle_t    ControlQueue;
static mtlink_all_CMD_t ctrl_cmd;

extern u8 g_runMode; //控制模式，01 手动模式  04 定航模式  02 定深模式  06 定深定航模式

/**
 * @function_name:MTLinkDispose
 * @brief:判断数据是否为上位机传输
 * @param SID:源地址
 * @param obj:命令代号
 * @param *buf:数据地址
 * @param len:数据长度
 * @return:None
 */
bool MTLinkDispose(uint8_t SID, uint16_t obj, uint8_t *buf, int len)
{
    bool state = false;
    switch (SID)
    {
    case HOST_ID: //可能是PC端上位机传来的数据
        PC_MasterDispose(obj, buf, len);
        break;
    default:
        break;
    }
    return state;
}

/**
 * @function_name:PC_MasterDispose
 * @brief:对MTLink数据解析并执行
 * @param obj:命令代号
 * @param *buf:数据地址
 * @param len:数据长度
 * @return:None
 */
bool PC_MasterDispose(uint16_t obj, uint8_t *buf, int len)
{
    u8  rxbuf[100];
    u16 motorpwmset[6] = {1470, 1470, 1470, 1470, 1470, 1470};
    if (len > 100)
        return false;
    memcpy(rxbuf, buf, len);

    g_HeartBeatCnt = 0;
    switch (obj)
    {
    case CMD_REBOOT_ID: //        		0x2000
        //软件复位
        NVIC_SystemReset();
        break;
    case CMD_DEBUG_ID: //        		0x2001
        //进入调试模式
        g_nowWokeMode = DEBUGMODE;
        break;
    case CMD_VERSION_ID: //      		0x2002
        //返回软件版本号
        LoadVersion(&MyVersion, 1.0, "shaoyg", 2019, 7, 22, 12, 23);
        MTLink_Encode(&MTLink_UDP, My_ID, HOST_ID, 0 /*不需要应答*/, SOFTVERSION_ID, (uint8_t *)&MyVersion, sizeof(AUV_Version_Typedef), 10);
        break;
    case CMD_BOOTLOAD_ID: //        	0x2003
        //跳转到BootLoader
        Jump_To_Bootloader();
        break;
    case CMD_ALL_ID: //              0x3006
        memcpy(&ctrl_cmd, rxbuf, sizeof(mtlink_all_CMD_t));
        //FIXME 这种方式修改手动模式下可以使用，定深和定航时会导致失控【传递函数发散】
        // ctrl_cmd.LeftRightTurn = (-1) * ctrl_cmd.LeftRightTurn;
        ctrl_cmd.UpDown = (-1) * ctrl_cmd.UpDown;
        // ctrl_cmd.FrontBack     = (-1) * ctrl_cmd.FrontBack;
        // ctrl_cmd.LeftRightMove = (-1) * ctrl_cmd.LeftRightMove;

        LED_SetPwm(CONSTRAIN(ctrl_cmd.light * 50, 5000, 0));
        YunTai_SetPwm(CONSTRAIN(-1 * ctrl_cmd.ptz * 10 + 2000, 2000, 1000)); //控制摄像头舵机
        if (DataManageQueue)
            xQueueOverwrite(DataManageQueue, &ctrl_cmd);
        break;
    case CMD_CALIBRATION_ID: //      0x6000
        //标定推进器中值，先将工作模式改为Debug模式，然后直接设置推进器PWM值

        g_nowWokeMode = TESTMODE;
        //			memcpy((u16 *)motorpwmset, rxbuf, 12);
        motorpwmset[0] = (rxbuf[1] << 8) + rxbuf[0];
        motorpwmset[1] = (rxbuf[3] << 8) + rxbuf[2];
        motorpwmset[2] = (rxbuf[5] << 8) + rxbuf[4];
        motorpwmset[3] = (rxbuf[7] << 8) + rxbuf[6];
        motorpwmset[4] = (rxbuf[9] << 8) + rxbuf[8];
        motorpwmset[5] = (rxbuf[11] << 8) + rxbuf[10];

        pca_setpwm(FL_MOTOR, 0, motorpwmset[0]); //左前
        pca_setpwm(FR_MOTOR, 0, motorpwmset[3]); //右前

        pca_setpwm(BR_MOTOR, 0, motorpwmset[5]); //右后
        pca_setpwm(BL_MOTOR, 0, motorpwmset[2]); //左后

        pca_setpwm(UL_MOTOR, 0, motorpwmset[1]); //ZUO
        pca_setpwm(UR_MOTOR, 0, motorpwmset[4]); //YOU

        break;
    case CMD_SAVECAL_ID: //        	0x6001
        //保存推进器中值,保存成功进入正常工作模式
        motorpwmset[0] = (rxbuf[1] << 8) + rxbuf[0];
        motorpwmset[1] = (rxbuf[3] << 8) + rxbuf[2];
        motorpwmset[2] = (rxbuf[5] << 8) + rxbuf[4];
        motorpwmset[3] = (rxbuf[7] << 8) + rxbuf[6];
        motorpwmset[4] = (rxbuf[9] << 8) + rxbuf[8];
        motorpwmset[5] = (rxbuf[11] << 8) + rxbuf[10];

        // STMFLASH_Write(PARAMADDR, motorpwmset, 1);
        // SaveToFlash();  //FIXME 可能会出问题，调试此功能时需要在检查一下
        g_nowWokeMode = NOMALMODE; //保存完成，将工作模式设置成正常模式
        break;
    case CMD_YAWPID_ID:                         //0x6002
        memcpy((uint8_t *)&yawPID, rxbuf, len); //航向PID调参
        YawNLPID.IP = yawPID.InP;
        YawNLPID.II = yawPID.InI;
        YawNLPID.ID = yawPID.InD;
        YawNLPID.OP = yawPID.OutP;
        YawNLPID.OI = yawPID.OutI;
        YawNLPID.OD = yawPID.OutD;
        break;
    case CMD_DEPTHPID_ID:                         //0x6003
        memcpy((uint8_t *)&depthPID, rxbuf, len); //深度PID调参
        DeepNLPID.IP = depthPID.InP;
        DeepNLPID.II = depthPID.InI;
        DeepNLPID.ID = depthPID.InD;
        DeepNLPID.OP = depthPID.OutP;
        DeepNLPID.OI = depthPID.OutI;
        DeepNLPID.OD = depthPID.OutD;
        break;
    case CMD_ROLLPID_ID:                         //0x6004
        memcpy((uint8_t *)&rollPID, rxbuf, len); //横滚PID调参
        RollNLPID.IP = rollPID.InP;
        RollNLPID.II = rollPID.InI;
        RollNLPID.ID = rollPID.InD;
        RollNLPID.OP = rollPID.OutP;
        RollNLPID.OI = rollPID.OutI;
        RollNLPID.OD = rollPID.OutD;
        break;
    case CMD_HOSTHEARTBEAT_ID: //        0xF000
        g_HeartBeatCnt = 0;
        break;
    default:
        break;
    }
    return true;
}

/**
 * @author        :robosea
 * @version       :v1.0.0
 * @Date          :2019-12-16 11:15:47
 * @LastEditors:smake
 * @LastEditTime:2020-04-19 00:28:44
 * @brief         :
 */
#ifndef __OBJECT_H_
#define __OBJECT_H_
//#include "global.h"
#include "MTLink.h"
#include "controlTasks.h"
#include "tim.h"
#include "board.h"
#include "CanNetWork.h"
#include "stmflash.h"
#define PID_ARGUEMENT_SECTOR 13;
#define PID_ARGUEMENT_ADDR   ADDR_FLSH_SECTOR_13;
enum MTLINK_OBJECT
{
    CMD_PRINT_ID         = 0X0000,
    CMD_REBOOT_ID        = 0x2000,
    CMD_DEBUG_ID         = 0x2001,
    CMD_VERSION_ID       = 0x2003,
    CMD_GOTOBL_ID        = 0x2004,
    CMD_GOTOBLSUCC_ID    = 0x2005,
    CMD_TOAPPSUCC_ID     = 0x2006,
    CMD_TOAPPERR_ID      = 0x2007,
    CMD_GOTOAPP          = 0x2008,
    CMD_SAVEPARAM_ID     = 0x2009,
    CMD_CONTROLMODE_ID   = 0x3000,
    CMD_CONTROL_ID       = 0x3001,
    CMD_LIGHT_ID         = 0x3002,
    CMD_HAND_ID          = 0x3003,
    CMD_CAMERA_ID        = 0x3004,
    CMD_TARGETVAL_ID     = 0x3005,
    CMD_ALL_ID           = 0x3006,
    CMD_CALIBRATION_ID   = 0x6000,
    CMD_SAVECAL_ID       = 0x6001,
    CMD_YAWPID_ID        = 0x6002, //航向PID调参
    CMD_DEPTHPID_ID      = 0x6003, //深度PID调参
    CMD_ROLLPID_ID       = 0x6004, //横滚PID调参
    CMD_HOSTHEARTBEAT_ID = 0xF000,
    CMD_APPCODE_ID       = 0xF002,
};
/*
*推进器中值结构体
*/
typedef struct
{
    u8  saveflag;
    u16 motormidpwm1;
    u16 motormidpwm2;
    u16 motormidpwm3;
    u16 motormidpwm4;
    u16 motormidpwm5;
    u16 motormidpwm6;
    u16 motormidpwm7;
    u16 motormidpwm8;
} MotorMidVal_t;
#define LIGHT_PWM(val)                                       \
    do                                                       \
    {                                                        \
        __HAL_TIM_SET_COMPARE(&LIGHT_TIM1, LIGHT1_PWM, val); \
        __HAL_TIM_SET_COMPARE(&LIGHT_TIM1, LIGHT2_PWM, val); \
        __HAL_TIM_SET_COMPARE(&LIGHT_TIM1, LIGHT3_PWM, val); \
    } while (0);
#define HAND_PWM(val)        __HAL_TIM_SET_COMPARE(&ARM_TIM9, ARM_PWM, val);       //机械臂
#define HAND_PWM_ROTATE(val) __HAL_TIM_SET_COMPARE(&ARM_TIM9, ROTATE_PWM, val);    //机械臂
#define HAND_PWM_CALMPS(val) __HAL_TIM_SET_COMPARE(&ARM_TIM9, ARM_PWM, val);       //机械臂
#define CANMRAMOTOR_PWM(val) __HAL_TIM_SET_COMPARE(&CAMERA_TIM9, CAMERA_PWM, val); //相机舵机
#define CAMRAMOTOR_POS(val)  set_pos(val);                                         //相机舵机
bool PC_MasterDispose(bool IsCAN, uint8_t SID, uint16_t obj, uint8_t *buf, int len);
bool CANObject_Dispose(int Master_ID, uint16_t object, uint8_t Sub_ID, char RorW, uint8_t *buf, uint32_t *len);
void MTLinkToCANConnect(uint8_t DID, uint16_t obj, uint8_t *buf, uint32_t len);
#endif

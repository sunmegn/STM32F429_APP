/**
 * @author        :robosea
 * @version       :v1.0.0
 * @Date          :2019-12-16 11:15:47
 * @LastEditors   :Robosea
 * @LastEditTime  :2020-03-02 14:32:37
 * @brief         :
 */
#ifndef __OBJECT_H_
#define __OBJECT_H_
//#include "global.h"

#include "MTLink.h"
#include "controlTasks.h"
#include "tim.h"
#include "board.h"

#define CMD_REBOOT_ID    0x2000
#define CMD_DEBUG_ID     0x2001
#define CMD_VERSION_ID   0x2003
#define CMD_BOOTLOAD_ID  0x2004
#define CMD_BOOTAPP_ID   0x2008
#define CMD_SAVEPARAM_ID 0x2009

#define CMD_CONTROLMODE_ID 0x3000
#define CMD_CONTROL_ID     0x3001
#define CMD_LIGHT_ID       0x3002
#define CMD_HAND_ID        0x3003
#define CMD_CAMERA_ID      0x3004
#define CMD_TARGETVAL_ID   0x3005
#define CMD_ALL_ID         0x3006

#define CMD_CALIBRATION_ID 0x6000
#define CMD_SAVECAL_ID     0x6001
#define CMD_YAWPID_ID      0x6002
#define CMD_DEPTHPID_ID    0x6003
#define CMD_ROLLPID_ID     0x6004

//const u16 CMD_CONTROLMODE_ID = 0x3000;
//const u16 CMD_CONTROL_ID = 0x3001;
//const u16 CMD_LIGHT_ID = 0x3002;
//const u16 CMD_HAND_ID = 0x3003;
//const u16 CMD_CAMERA_ID = 0x3004;
//const u16 CMD_TARGETVAL_ID = 0x3005;

//const u16 CMD_CALIBRATION_ID = 0x6000;
//const u16 CMD_SAVECAL_ID = 0x6001;
//const u16 CMD_YAWPID_ID = 0x6002;
//const u16 CMD_DEPTHPID_ID = 0x6003;
//const u16 CMD_ROLLPID_ID = 0x6004;

#define CMD_HOSTHEARTBEAT_ID 0xF000
#define CMD_BOOTBINDATA_ID   0xF002

#define LIGHT_PWM(val)                                       \
    do                                                       \
    {                                                        \
        __HAL_TIM_SET_COMPARE(&LIGHT_TIM1, LIGHT1_PWM, val); \
        __HAL_TIM_SET_COMPARE(&LIGHT_TIM1, LIGHT2_PWM, val); \
        __HAL_TIM_SET_COMPARE(&LIGHT_TIM1, LIGHT3_PWM, val); \
    } while (0);
#define HAND_PWM(val)        __HAL_TIM_SET_COMPARE(&ARM_TIM9, ARM_PWM, val);    //机械臂
#define HAND_PWM_ROTATE(val) __HAL_TIM_SET_COMPARE(&ARM_TIM9, ROTATE_PWM, val); //机械臂
#define HAND_PWM_CALMPS(val) __HAL_TIM_SET_COMPARE(&ARM_TIM9, ARM_PWM, val);    //机械臂

#define CANMRAMOTOR_PWM(val) __HAL_TIM_SET_COMPARE(&CAMERA_TIM9, CAMERA_PWM, val); //相机舵机
#define CAMRAMOTOR_POS(val)  set_pos(val);                                         //相机舵机
#define GETPWMVAL0_100(val)  ((val - 50) * 10 + 1500)                              //计算camera舵机中值

bool PC_MasterDispose(uint16_t obj, uint8_t *buf, int len);

#endif

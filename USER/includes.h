#ifndef __INCLUDES_H
#define __INCLUDES_H

//CubeMX 生成
#include "main.h"
#include "cmsis_os.h"
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

//C 语言标准库
#include "stdint.h"
#include "string.h"
#include "stdlib.h"
#include "stdio.h"
//自定义
typedef uint32_t u32;
typedef uint16_t u16;
typedef uint8_t  u8;
#include "global.h"
//通信
//#include "message.h"
#include "data_parse.h"
#include "mathTool.h"

//Driver
#include "board.h"
#include "SHT35.h"
#include "ms5837.h"
#include "Batt.h"
#include "imu.h"
#include "led.h"
#include "pca9685.h"
#include "switch.h"
#include "pwm.h"
#include "mcp3221.h"

//W5500
#include "types.h"
// #include "w5500.h"//引用此头文件会使CanOpen协议中RTR引用错误，引起变量重定义
// #include "socket.h"

//Control
#include "control.h"
#include "PID.h"

//任务
#include "ledTasks.h"
#include "imuTasks.h"
#include "pressureTasks.h"
#include "sensorTasks.h"
#include "controlTasks.h"
#include "messageTasks.h"

//#define DEBUG
#endif

/**
 * @author        :robosea
 * @version       :v1.0.0
 * @Date          :2019-12-16 11:15:42
 * @LastEditors:smake
 * @LastEditTime:2020-04-25 13:49:49
 * @brief         :
 */
#include "board.h"
#include "udp.h"
#include "usart.h"
#include "manipulaterD2.h"

QueueHandle_t IMU_Message_Queue      = NULL;
QueueHandle_t Control_Message_Queue  = NULL;
QueueHandle_t Command_Message_Queue  = NULL;
QueueHandle_t Pressure_Message_Queue = NULL;
QueueHandle_t TempHum_Message_Queue  = NULL;
QueueHandle_t Battery_Message_Queue  = NULL;
QueueHandle_t ModbusRxQueue          = NULL;
QueueHandle_t FeedBackQueue          = NULL;

/**
  * @brief  
  * @param  
  * @note  
  */
void delay_ms(int nms)
{
    if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED)
    {
        HAL_Delay(nms);
    }
    else
    {
        osDelay(nms);
    }
}
u32 use_time = 0;
u32 Reflash(void) //执行周期
{
    static u32 start_time = 0;
    static u32 end_time   = 0;

    end_time   = HAL_GetTick();
    use_time   = end_time - start_time;
    start_time = end_time;
    return use_time;
}
/**
  * @brief  任务消息队列、信号量、定时器创建 
  * @param  
  * @note   1. 在各个任务执行前创建 2. 注意防止中断打断创建
  */
void Task_Queue_Semaphore_Timers_Create(void)
{
    do
    {
        Pressure_Message_Queue = xQueueCreate(1, sizeof(PressureMsg_t)); //Pressure_Message_Queue
    } while (Pressure_Message_Queue == NULL);

    do
    {
        IMU_Message_Queue = xQueueCreate(1, sizeof(IMUMsg_t));
    } while (IMU_Message_Queue == NULL);

    do
    {
        Control_Message_Queue = xQueueCreate(1, sizeof(ControlParam_t));
    } while (Control_Message_Queue == NULL);

    do
    {
        TempHum_Message_Queue = xQueueCreate(1, sizeof(TempHumiMsg_t));
    } while (TempHum_Message_Queue == NULL);

    do
    {
        Battery_Message_Queue = xQueueCreate(1, sizeof(PowerMsg_t));
    } while (Battery_Message_Queue == NULL);

    //    do{
    //		FeedBackQueue = xQueueCreate(1, sizeof(PowerMsg_t));
    //	}while(FeedBackQueue == NULL);
}
/**
  * @brief  
  * @param  
  * @note  
  */
void USER_UART_IRQHandler(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) //
    {
    }
    if (huart->Instance == USART2) //
    {
    }
    if (huart->Instance == USART3) //
    {
    }
    if (huart->Instance == UART4) //
    {
    }
    if (huart->Instance == UART5) //
    {
    }
    if (huart->Instance == USART6) //
    {
    }
    if (huart->Instance == UART7) //
    {
    }
    if (huart->Instance == UART8) //Saber IMU
    {
        HAL_UART8_Receive_IDLE(&huart8);
    }
}

u32 last_time[6] = {0};
u32 run_time[6]  = {0};
u32 getRunTime(u8 n)
{
    u32 run_time_vaule = 0;
    u32 cur_time       = HAL_GetTick();

    run_time_vaule = cur_time - last_time[n];

    last_time[n] = cur_time;

    return run_time_vaule;
}

/**
  * @brief  
  * @param  
  * @note  
  */
void BSP_Init(void)
{
    PowerON_SwitchSelfHold();   //电源开关锁存
    PCA9685_Init();             //IIC 16路PWM输出
    MS5837_Init();              //压传
    SHT35_Init();               //舱内温湿度
    Batt_Init();                //电源 电压、电流
    Udp_Init();                 //W5500 UDP通信
    IMU_Init();                 //惯导模块
    RS485_Init();               //初始化852声呐
    manipulater_D2_RS485Init(); //初始化机械臂相关参数
    PWM_Init();                 //LED大灯PWM
    MessageInit();
    PID_Init();
    CtrlMidPwmInit();
    //  测试demo
    //  PWM_Test_Demo();
    //	PCA9685_Test_Demo();
    //	LED_Test_Demo();
    //	MessageInit();
    //  SHT35_Test_Demo();
    //	Batt_Test_Demo();
    //  IMU_Test_Demo();
    //	MS8537_Test_Demo();
    //  W5500_Test_Demo();       				//W5500 UDP通信
    //	MCP3221_Test_Demo();
}

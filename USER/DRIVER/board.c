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
u32 Reflash(void) //ִ������
{
    static u32 start_time = 0;
    static u32 end_time   = 0;

    end_time   = HAL_GetTick();
    use_time   = end_time - start_time;
    start_time = end_time;
    return use_time;
}
/**
  * @brief  ������Ϣ���С��ź�������ʱ������ 
  * @param  
  * @note   1. �ڸ�������ִ��ǰ���� 2. ע���ֹ�жϴ�ϴ���
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
    PowerON_SwitchSelfHold();   //��Դ��������
    PCA9685_Init();             //IIC 16·PWM���
    MS5837_Init();              //ѹ��
    SHT35_Init();               //������ʪ��
    Batt_Init();                //��Դ ��ѹ������
    Udp_Init();                 //W5500 UDPͨ��
    IMU_Init();                 //�ߵ�ģ��
    RS485_Init();               //��ʼ��852����
    manipulater_D2_RS485Init(); //��ʼ����е����ز���
    PWM_Init();                 //LED���PWM
    MessageInit();
    PID_Init();
    CtrlMidPwmInit();
    //  ����demo
    //  PWM_Test_Demo();
    //	PCA9685_Test_Demo();
    //	LED_Test_Demo();
    //	MessageInit();
    //  SHT35_Test_Demo();
    //	Batt_Test_Demo();
    //  IMU_Test_Demo();
    //	MS8537_Test_Demo();
    //  W5500_Test_Demo();       				//W5500 UDPͨ��
    //	MCP3221_Test_Demo();
}

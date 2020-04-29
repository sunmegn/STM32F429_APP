/**
 * @author        :smake
 * @version       :v1.0.0
 * @Date          :2020-04-21 15:58:30
 * @LastEditors:smake
 * @LastEditTime:2020-04-25 14:14:54
 * @brief         :
 */
#include "Sonar852Task.h"
#include "usart.h"
#include "MTLink.h"
#include "Object.h"

sonar852_Typedef          Sonar852_Data;
int                       SONAR852GETDATA_FLAG = 0;
extern UART_HandleTypeDef huart3;
extern MTLink_typedef     MTLink_UDP;
int                       sonar_send_tims    = 0;
int                       ctrl_cmd_SonarFlag = 0;
//向852声呐发送上位机请求数据
void sonar852_sendHeader(uint8_t *buf, uint16_t len)
{
    RS485_RE(1);
    HAL_UART_Transmit(&huart3, buf, len, 100); //声呐头信息长度为27
    RS485_RE(0);
}

void Sonar852Task_Function(void const *argument)
{
    TickType_t tick = xTaskGetTickCount();
    while (1)
    {
        if (SONAR852GETDATA_FLAG)
        {
            HAL_GPIO_WritePin(GPIOD, USART3_RE_Pin, GPIO_PIN_RESET);
            MTLink_Encode(&MTLink_UDP, MY_ID, HOST_ID, 0 /*不需要应答*/, CMD_SONAR852_ID, Sonar852_Data.Sonar852_Body, Sonar852_Data.Rx_Size, 10);
            SONAR852GETDATA_FLAG = 0;
            sonar_send_tims++;
        }

        vTaskDelayUntil(&tick, 20);
    }
}

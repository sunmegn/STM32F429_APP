/**
  ******************************************************************************
  * File Name          : USART.h
  * Description        : This file provides code for the configuration
  *                      of the USART instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __usart_H
#define __usart_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern UART_HandleTypeDef huart8;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;

/* USER CODE BEGIN Private defines */
    #define RS485_RE(val) val ? HAL_GPIO_WritePin(GPIOD, USART3_RE_Pin, GPIO_PIN_SET) : HAL_GPIO_WritePin(GPIOD, USART3_RE_Pin, GPIO_PIN_RESET)
    #define USART3_RX_LEN 600

    typedef struct
    {
        uint8_t  RXFlag : 1;
        uint16_t RxSize;
        uint8_t  RX_pData[USART3_RX_LEN];
        uint8_t  RealBuf[USART3_RX_LEN];
    } uart3_Receive_typedef;
/* USER CODE END Private defines */

void MX_UART8_Init(void);
void MX_USART3_UART_Init(void);
void MX_USART6_UART_Init(void);

/* USER CODE BEGIN Prototypes */
    int  usart3_Decode(uint8_t *buf, uint16_t len);
    void HAL_USART3_Receive_IDLE(void);
	int RS485_Init(void);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ usart_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

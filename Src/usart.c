/**
  ******************************************************************************
  * File Name          : USART.c
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

/* Includes ------------------------------------------------------------------*/
#include "usart.h"

/* USER CODE BEGIN 0 */
#include "manipulaterD2.h"
extern uint8_t manipulater_D2_receiveBuf[32];

/* USER CODE END 0 */

UART_HandleTypeDef huart8;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef  hdma_uart8_rx;
DMA_HandleTypeDef  hdma_usart3_rx;
DMA_HandleTypeDef  hdma_usart6_rx;

/* UART8 init function */
void MX_UART8_Init(void)
{

    huart8.Instance          = UART8;
    huart8.Init.BaudRate     = 115200;
    huart8.Init.WordLength   = UART_WORDLENGTH_8B;
    huart8.Init.StopBits     = UART_STOPBITS_1;
    huart8.Init.Parity       = UART_PARITY_NONE;
    huart8.Init.Mode         = UART_MODE_TX_RX;
    huart8.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart8.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart8) != HAL_OK)
    {
        Error_Handler();
    }
}
/* USART3 init function */

void MX_USART3_UART_Init(void)
{

    huart3.Instance          = USART3;
    huart3.Init.BaudRate     = 115200;
    huart3.Init.WordLength   = UART_WORDLENGTH_8B;
    huart3.Init.StopBits     = UART_STOPBITS_1;
    huart3.Init.Parity       = UART_PARITY_NONE;
    huart3.Init.Mode         = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart3) != HAL_OK)
    {
        Error_Handler();
    }
}
/* USART6 init function */

void MX_USART6_UART_Init(void)
{

    huart6.Instance          = USART6;
    huart6.Init.BaudRate     = 115200;
    huart6.Init.WordLength   = UART_WORDLENGTH_8B;
    huart6.Init.StopBits     = UART_STOPBITS_1;
    huart6.Init.Parity       = UART_PARITY_NONE;
    huart6.Init.Mode         = UART_MODE_TX_RX;
    huart6.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart6.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart6) != HAL_OK)
    {
        Error_Handler();
    }
}

void HAL_UART_MspInit(UART_HandleTypeDef *uartHandle)
{

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if (uartHandle->Instance == UART8)
    {
        /* USER CODE BEGIN UART8_MspInit 0 */

        /* USER CODE END UART8_MspInit 0 */
        /* UART8 clock enable */
        __HAL_RCC_UART8_CLK_ENABLE();

        __HAL_RCC_GPIOE_CLK_ENABLE();
        /**UART8 GPIO Configuration    
    PE0     ------> UART8_RX
    PE1     ------> UART8_TX 
    */
        GPIO_InitStruct.Pin       = GPIO_PIN_0 | GPIO_PIN_1;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_PULLUP;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF8_UART8;
        HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

        /* UART8 DMA Init */
        /* UART8_RX Init */
        hdma_uart8_rx.Instance                 = DMA1_Stream6;
        hdma_uart8_rx.Init.Channel             = DMA_CHANNEL_5;
        hdma_uart8_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
        hdma_uart8_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
        hdma_uart8_rx.Init.MemInc              = DMA_MINC_ENABLE;
        hdma_uart8_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_uart8_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
        hdma_uart8_rx.Init.Mode                = DMA_CIRCULAR;
        hdma_uart8_rx.Init.Priority            = DMA_PRIORITY_HIGH;
        hdma_uart8_rx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
        if (HAL_DMA_Init(&hdma_uart8_rx) != HAL_OK)
        {
            Error_Handler();
        }

        __HAL_LINKDMA(uartHandle, hdmarx, hdma_uart8_rx);

        /* UART8 interrupt Init */
        HAL_NVIC_SetPriority(UART8_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(UART8_IRQn);
        /* USER CODE BEGIN UART8_MspInit 1 */

        /* USER CODE END UART8_MspInit 1 */
    }
    else if (uartHandle->Instance == USART3)
    {
        /* USER CODE BEGIN USART3_MspInit 0 */

        /* USER CODE END USART3_MspInit 0 */
        /* USART3 clock enable */
        __HAL_RCC_USART3_CLK_ENABLE();

        __HAL_RCC_GPIOD_CLK_ENABLE();
        /**USART3 GPIO Configuration    
    PD8     ------> USART3_TX
    PD9     ------> USART3_RX 
    */
        GPIO_InitStruct.Pin       = GPIO_PIN_8 | GPIO_PIN_9;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
        HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

        /* USART3 DMA Init */
        /* USART3_RX Init */
        hdma_usart3_rx.Instance                 = DMA1_Stream1;
        hdma_usart3_rx.Init.Channel             = DMA_CHANNEL_4;
        hdma_usart3_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
        hdma_usart3_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
        hdma_usart3_rx.Init.MemInc              = DMA_MINC_ENABLE;
        hdma_usart3_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_usart3_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
        hdma_usart3_rx.Init.Mode                = DMA_NORMAL;
        hdma_usart3_rx.Init.Priority            = DMA_PRIORITY_MEDIUM;
        hdma_usart3_rx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
        if (HAL_DMA_Init(&hdma_usart3_rx) != HAL_OK)
        {
            Error_Handler();
        }

        __HAL_LINKDMA(uartHandle, hdmarx, hdma_usart3_rx);

        /* USART3 interrupt Init */
        HAL_NVIC_SetPriority(USART3_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(USART3_IRQn);
        /* USER CODE BEGIN USART3_MspInit 1 */

        /* USER CODE END USART3_MspInit 1 */
    }
    else if (uartHandle->Instance == USART6)
    {
        /* USER CODE BEGIN USART6_MspInit 0 */

        /* USER CODE END USART6_MspInit 0 */
        /* USART6 clock enable */
        __HAL_RCC_USART6_CLK_ENABLE();

        __HAL_RCC_GPIOC_CLK_ENABLE();
        /**USART6 GPIO Configuration    
    PC6     ------> USART6_TX
    PC7     ------> USART6_RX 
    */
        GPIO_InitStruct.Pin       = GPIO_PIN_6 | GPIO_PIN_7;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

        /* USART6 DMA Init */
        /* USART6_RX Init */
        hdma_usart6_rx.Instance                 = DMA2_Stream1;
        hdma_usart6_rx.Init.Channel             = DMA_CHANNEL_5;
        hdma_usart6_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
        hdma_usart6_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
        hdma_usart6_rx.Init.MemInc              = DMA_MINC_ENABLE;
        hdma_usart6_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_usart6_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
        hdma_usart6_rx.Init.Mode                = DMA_CIRCULAR;
        hdma_usart6_rx.Init.Priority            = DMA_PRIORITY_LOW;
        hdma_usart6_rx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
        if (HAL_DMA_Init(&hdma_usart6_rx) != HAL_OK)
        {
            Error_Handler();
        }

        __HAL_LINKDMA(uartHandle, hdmarx, hdma_usart6_rx);

        /* USER CODE BEGIN USART6_MspInit 1 */

        /* USER CODE END USART6_MspInit 1 */
    }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef *uartHandle)
{

    if (uartHandle->Instance == UART8)
    {
        /* USER CODE BEGIN UART8_MspDeInit 0 */

        /* USER CODE END UART8_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_UART8_CLK_DISABLE();

        /**UART8 GPIO Configuration    
    PE0     ------> UART8_RX
    PE1     ------> UART8_TX 
    */
        HAL_GPIO_DeInit(GPIOE, GPIO_PIN_0 | GPIO_PIN_1);

        /* UART8 DMA DeInit */
        HAL_DMA_DeInit(uartHandle->hdmarx);

        /* UART8 interrupt Deinit */
        HAL_NVIC_DisableIRQ(UART8_IRQn);
        /* USER CODE BEGIN UART8_MspDeInit 1 */

        /* USER CODE END UART8_MspDeInit 1 */
    }
    else if (uartHandle->Instance == USART3)
    {
        /* USER CODE BEGIN USART3_MspDeInit 0 */

        /* USER CODE END USART3_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_USART3_CLK_DISABLE();

        /**USART3 GPIO Configuration    
    PD8     ------> USART3_TX
    PD9     ------> USART3_RX 
    */
        HAL_GPIO_DeInit(GPIOD, GPIO_PIN_8 | GPIO_PIN_9);

        /* USART3 DMA DeInit */
        HAL_DMA_DeInit(uartHandle->hdmarx);

        /* USART3 interrupt Deinit */
        HAL_NVIC_DisableIRQ(USART3_IRQn);
        /* USER CODE BEGIN USART3_MspDeInit 1 */

        /* USER CODE END USART3_MspDeInit 1 */
    }
    else if (uartHandle->Instance == USART6)
    {
        /* USER CODE BEGIN USART6_MspDeInit 0 */

        /* USER CODE END USART6_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_USART6_CLK_DISABLE();

        /**USART6 GPIO Configuration    
    PC6     ------> USART6_TX
    PC7     ------> USART6_RX 
    */
        HAL_GPIO_DeInit(GPIOC, GPIO_PIN_6 | GPIO_PIN_7);

        /* USART6 DMA DeInit */
        HAL_DMA_DeInit(uartHandle->hdmarx);
        /* USER CODE BEGIN USART6_MspDeInit 1 */

        /* USER CODE END USART6_MspDeInit 1 */
    }
}

/* USER CODE BEGIN 1 */
uart3_Receive_typedef usart3_rxbuf;

int RS485_Init(void)
{
    RS485_RE(0);
    HAL_UART_Receive_DMA(&huart3, usart3_rxbuf.RX_pData, sizeof(usart3_rxbuf.RX_pData));
    __HAL_UART_CLEAR_IDLEFLAG(&huart3);
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
    return 0;
}

/**
  * @funNm   HAL_USART3_Receive_IDLE
  * @brief   USART3空闲中断接受回调函数 接受数据
  * @param	 无
  * @retval  无
  */
void HAL_USART3_Receive_IDLE(void)
{
    __HAL_UART_CLEAR_IDLEFLAG(&huart3);
    HAL_UART_DMAStop(&huart3); //DMAmuxChannel

    usart3_rxbuf.RxSize = USART3_RX_LEN - __HAL_DMA_GET_COUNTER(&hdma_usart3_rx);
    usart3_Decode(usart3_rxbuf.RX_pData, usart3_rxbuf.RxSize);
    memset(usart3_rxbuf.RX_pData, '\0', usart3_rxbuf.RxSize);

    HAL_UART_Receive_DMA(&huart3, usart3_rxbuf.RX_pData, USART3_RX_LEN);
}

//usart3数据解析
int usart3_Decode(uint8_t *buf, uint16_t len)
{
    if ((len <= 32) & (buf[0] == 0xA5)) //有可能是机械臂返回值
    {
        memcpy(manipulater_D2_receiveBuf, buf, buf[len - 2]);
        manipulater_D2_receive(manipulater_D2_receiveBuf);
    }
    return 0;
}
/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

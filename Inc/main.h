/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

    /* Private includes ----------------------------------------------------------*/
    /* USER CODE BEGIN Includes */

    /* USER CODE END Includes */

    /* Exported types ------------------------------------------------------------*/
    /* USER CODE BEGIN ET */

    /* USER CODE END ET */

    /* Exported constants --------------------------------------------------------*/
    /* USER CODE BEGIN EC */

    /* USER CODE END EC */

    /* Exported macro ------------------------------------------------------------*/
    /* USER CODE BEGIN EM */

    /* USER CODE END EM */

    /* Exported functions prototypes ---------------------------------------------*/
    void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BLUE_LED_Pin              GPIO_PIN_13
#define BLUE_LED_GPIO_Port        GPIOC
#define GREEN_LED_Pin             GPIO_PIN_14
#define GREEN_LED_GPIO_Port       GPIOC
#define RED_LED_Pin               GPIO_PIN_15
#define RED_LED_GPIO_Port         GPIOC
#define BATT_CURRENT_Pin          GPIO_PIN_1
#define BATT_CURRENT_GPIO_Port    GPIOC
#define BUZZER_Pin                GPIO_PIN_1
#define BUZZER_GPIO_Port          GPIOA
#define W5500_RESET_Pin           GPIO_PIN_2
#define W5500_RESET_GPIO_Port     GPIOA
#define W5500_INIT_Pin            GPIO_PIN_3
#define W5500_INIT_GPIO_Port      GPIOA
#define W5500_SPI_CS_Pin          GPIO_PIN_4
#define W5500_SPI_CS_GPIO_Port    GPIOA
#define W5500_SPI_SCK_Pin         GPIO_PIN_5
#define W5500_SPI_SCK_GPIO_Port   GPIOA
#define W5500_SPI_MISO_Pin        GPIO_PIN_6
#define W5500_SPI_MISO_GPIO_Port  GPIOA
#define W5500_SPI_MOSI_Pin        GPIO_PIN_7
#define W5500_SPI_MOSI_GPIO_Port  GPIOA
#define PCA9685_SCL_Pin           GPIO_PIN_10
#define PCA9685_SCL_GPIO_Port     GPIOB
#define PCA9685_SDA_Pin           GPIO_PIN_11
#define PCA9685_SDA_GPIO_Port     GPIOB
#define USART3_RE_Pin             GPIO_PIN_10
#define USART3_RE_GPIO_Port       GPIOD
#define LORA_AUX_Pin              GPIO_PIN_11
#define LORA_AUX_GPIO_Port        GPIOD
#define LORA_M1_Pin               GPIO_PIN_9
#define LORA_M1_GPIO_Port         GPIOA
#define LORA_M0_Pin               GPIO_PIN_10
#define LORA_M0_GPIO_Port         GPIOA
#define SPI3_CS_Pin               GPIO_PIN_15
#define SPI3_CS_GPIO_Port         GPIOA
#define POWER_KEY_STATE_Pin       GPIO_PIN_1
#define POWER_KEY_STATE_GPIO_Port GPIOD
#define POWER_KEY_Pin             GPIO_PIN_2
#define POWER_KEY_GPIO_Port       GPIOD
#define MS5837_SCL_PIN_Pin        GPIO_PIN_6
#define MS5837_SCL_PIN_GPIO_Port  GPIOB
#define MS5837_SDA_PIN_Pin        GPIO_PIN_7
#define MS5837_SDA_PIN_GPIO_Port  GPIOB
#define SABER_RST_Pin             GPIO_PIN_9
#define SABER_RST_GPIO_Port       GPIOB
    /* USER CODE BEGIN Private defines */

    /* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

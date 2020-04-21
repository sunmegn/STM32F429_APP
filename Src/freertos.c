/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */     
#include "includes.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId messageTaskHandle;
osThreadId SensorTaskHandle;
osThreadId controlTaskHandle;
osThreadId pressureTaskHandle;
osThreadId imuTaskHandle;
osThreadId ledTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void MessageTask_Function(void const * argument);
void SensorTask_Function(void const * argument);
void ControlTask_Function(void const * argument);
void pressureTask_Function(void const * argument);
void imuTask_Function(void const * argument);
void ledTask_Function(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t  xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize)
{
    *ppxIdleTaskTCBBuffer   = &xIdleTaskTCBBuffer;
    *ppxIdleTaskStackBuffer = &xIdleStack[0];
    *pulIdleTaskStackSize   = configMINIMAL_STACK_SIZE;
    /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
    /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
    /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
    Task_Queue_Semaphore_Timers_Create();
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of messageTask */
  osThreadDef(messageTask, MessageTask_Function, osPriorityHigh, 0, 2048);
  messageTaskHandle = osThreadCreate(osThread(messageTask), NULL);

  /* definition and creation of SensorTask */
  osThreadDef(SensorTask, SensorTask_Function, osPriorityLow, 0, 512);
  SensorTaskHandle = osThreadCreate(osThread(SensorTask), NULL);

  /* definition and creation of controlTask */
  osThreadDef(controlTask, ControlTask_Function, osPriorityRealtime, 0, 2048);
  controlTaskHandle = osThreadCreate(osThread(controlTask), NULL);

  /* definition and creation of pressureTask */
  osThreadDef(pressureTask, pressureTask_Function, osPriorityIdle, 0, 512);
  pressureTaskHandle = osThreadCreate(osThread(pressureTask), NULL);

  /* definition and creation of imuTask */
  osThreadDef(imuTask, imuTask_Function, osPriorityAboveNormal, 0, 2048);
  imuTaskHandle = osThreadCreate(osThread(imuTask), NULL);

  /* definition and creation of ledTask */
  osThreadDef(ledTask, ledTask_Function, osPriorityIdle, 0, 128);
  ledTaskHandle = osThreadCreate(osThread(ledTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_MessageTask_Function */
/**
  * @brief  Function implementing the messageTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_MessageTask_Function */
__weak void MessageTask_Function(void const * argument)
{
  /* USER CODE BEGIN MessageTask_Function */
    /* Infinite loop */
    for (;;)
    {
        osDelay(1);
    }
  /* USER CODE END MessageTask_Function */
}

/* USER CODE BEGIN Header_SensorTask_Function */
/**
* @brief Function implementing the SensorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SensorTask_Function */
__weak void SensorTask_Function(void const * argument)
{
  /* USER CODE BEGIN SensorTask_Function */
    /* Infinite loop */
    for (;;)
    {
        osDelay(1);
    }
  /* USER CODE END SensorTask_Function */
}

/* USER CODE BEGIN Header_ControlTask_Function */
/**
* @brief Function implementing the controlTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ControlTask_Function */
__weak void ControlTask_Function(void const * argument)
{
  /* USER CODE BEGIN ControlTask_Function */
    /* Infinite loop */
    for (;;)
    {
        osDelay(1);
    }
  /* USER CODE END ControlTask_Function */
}

/* USER CODE BEGIN Header_pressureTask_Function */
/**
* @brief Function implementing the pressureTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_pressureTask_Function */
__weak void pressureTask_Function(void const * argument)
{
  /* USER CODE BEGIN pressureTask_Function */
    /* Infinite loop */
    for (;;)
    {
        osDelay(1);
    }
  /* USER CODE END pressureTask_Function */
}

/* USER CODE BEGIN Header_imuTask_Function */
/**
* @brief Function implementing the imuTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_imuTask_Function */
__weak void imuTask_Function(void const * argument)
{
  /* USER CODE BEGIN imuTask_Function */
    /* Infinite loop */
    for (;;)
    {
        osDelay(1);
    }
  /* USER CODE END imuTask_Function */
}

/* USER CODE BEGIN Header_ledTask_Function */
/**
* @brief Function implementing the ledTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ledTask_Function */
__weak void ledTask_Function(void const * argument)
{
  /* USER CODE BEGIN ledTask_Function */
    /* Infinite loop */
    for (;;)
    {
        osDelay(1);
    }
  /* USER CODE END ledTask_Function */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

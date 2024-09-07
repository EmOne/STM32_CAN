/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "can.h"
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
evState_t ev =
{ .comm = POST, .sensor = POST, .output = POST, .tracking = POST, .event =
		EV_NONE };

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for canDefaultTask */
osThreadId_t canDefaultTaskHandle;
const osThreadAttr_t canDefaultTask_attributes = {
  .name = "canDefaultTask",
  .stack_size = 1024 * 4,
  .priority =
		(osPriority_t) osPriorityLow,
};
/* Definitions for canRouterTask */
osThreadId_t canRouterTaskHandle;
const osThreadAttr_t canRouterTask_attributes = {
  .name = "canRouterTask",
  .stack_size = 256 * 4,
  .priority =
		(osPriority_t) osPriorityLow,
};
/* Definitions for canServerTask */
osThreadId_t canServerTaskHandle;
const osThreadAttr_t canServerTask_attributes = {
  .name = "canServerTask",
  .stack_size = 1024 * 4,
  .priority =
		(osPriority_t) osPriorityLow,
};
/* Definitions for canClientTask */
osThreadId_t canClientTaskHandle;
const osThreadAttr_t canClientTask_attributes = {
  .name = "canClientTask",
  .stack_size = 1024 * 4,
  .priority =
		(osPriority_t) osPriorityLow,
};
/* Definitions for canRxMsgQueue */
osMessageQueueId_t canRxMsgQueueHandle;
const osMessageQueueAttr_t canRxMsgQueue_attributes = {
  .name = "canRxMsgQueue"
};
/* Definitions for coreBinarySem */
osSemaphoreId_t coreBinarySemHandle;
const osSemaphoreAttr_t coreBinarySem_attributes = {
  .name = "coreBinarySem"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void canStartDefaultTask(void *argument);
void canStartRouterTask(void *argument);
void canStartServerTask(void *argument);
void canStartClientTask(void *argument);

extern void MX_USB_HOST_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

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

  /* Create the semaphores(s) */
  /* creation of coreBinarySem */
  coreBinarySemHandle = osSemaphoreNew(1, 1, &coreBinarySem_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of canRxMsgQueue */
  canRxMsgQueueHandle = osMessageQueueNew (16, sizeof(uint16_t), &canRxMsgQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, (void*) &ev, &defaultTask_attributes);

  /* creation of canDefaultTask */
  canDefaultTaskHandle = osThreadNew(canStartDefaultTask, (void*) &ev, &canDefaultTask_attributes);

  /* creation of canRouterTask */
  canRouterTaskHandle = osThreadNew(canStartRouterTask, (void*) &ev, &canRouterTask_attributes);

  /* creation of canServerTask */
  canServerTaskHandle = osThreadNew(canStartServerTask, (void*) &ev, &canServerTask_attributes);

  /* creation of canClientTask */
  canClientTaskHandle = osThreadNew(canStartClientTask, (void*) &ev, &canClientTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
//	osThreadSuspend(canRouterTaskHandle);
//	osThreadSuspend(canServerTaskHandle);
//	osThreadSuspend(canClientTaskHandle);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for USB_HOST */
  MX_USB_HOST_Init();
  /* USER CODE BEGIN StartDefaultTask */
	/* Init CSP */
	csp_init();
  /* Infinite loop */
  for(;;)
  {
		//Event cur_event = event_check();
		//		switch(currentMode) {
		//			case POST:
		//				currentMode = POST_function(cur_event);
		//				break;
		//			case IDLE:
		//				currentMode = IDLE_function(cur_event);
		//				break;
		//			case SETTING:
		//				currentMode = SETTING_function(cur_event);
		//				break;
		//			case RUNNING:
		//				currentMode = RUNNING_function(cur_event);
		//				break;
		//			case ALARM:
		//				currentMode = ALARM_function(cur_event);
		//				break;
		//			case FAILSAFE:
		//				currentMode = FAILSAFE_function(cur_event);
		//				break;
		//			default:
		//				currentMode = currentMode;
		// 				break;
		//		}

    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_canStartDefaultTask */
/**
* @brief Function implementing the canDefaultTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_canStartDefaultTask */
__weak void canStartDefaultTask(void *argument)
{
  /* USER CODE BEGIN canStartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END canStartDefaultTask */
}

/* USER CODE BEGIN Header_canStartRouterTask */
/**
* @brief Function implementing the canRouterTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_canStartRouterTask */
__weak void canStartRouterTask(void *argument)
{
  /* USER CODE BEGIN canStartRouterTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END canStartRouterTask */
}

/* USER CODE BEGIN Header_canStartServerTask */
/**
* @brief Function implementing the canServerTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_canStartServerTask */
__weak void canStartServerTask(void *argument)
{
  /* USER CODE BEGIN canStartServerTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END canStartServerTask */
}

/* USER CODE BEGIN Header_canStartClientTask */
/**
* @brief Function implementing the canClientTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_canStartClientTask */
__weak void canStartClientTask(void *argument)
{
  /* USER CODE BEGIN canStartClientTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END canStartClientTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */


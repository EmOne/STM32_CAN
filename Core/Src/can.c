/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
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
#include "can.h"

/* USER CODE BEGIN 0 */
CAN_RxHeaderTypeDef rxHeader; //CAN Bus Transmit Header
CAN_TxHeaderTypeDef txHeader; //CAN Bus Receive Header
uint8_t canRX[8] =
{ 0, 0, 0, 0, 0, 0, 0, 0 };  //CAN Bus Receive Buffer
CAN_FilterTypeDef canfil; //CAN Bus Filter
uint32_t canMailbox; //CAN Bus Mail box variable
uint8_t flagRecv = 0;
Mode *currentMode;
/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;

/* CAN1 init function */
void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 42;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_2TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_4TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_3TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
	canfil.FilterBank = 0;
	canfil.FilterMode = CAN_FILTERMODE_IDMASK;
	canfil.FilterFIFOAssignment = CAN_RX_FIFO0;
	canfil.FilterIdHigh = 0xFFFFFFFF;
	canfil.FilterIdLow = 0xFFFFFFFF;
	canfil.FilterMaskIdHigh = 0x00000000;
	canfil.FilterMaskIdLow = 0x00000000;
	canfil.FilterScale = CAN_FILTERSCALE_32BIT;
	canfil.FilterActivation = ENABLE;
	canfil.SlaveStartFilterBank = 14;
	HAL_CAN_ConfigFilter(&hcan1, &canfil);

	txHeader.DLC = 8;
	txHeader.IDE = CAN_ID_STD;
	txHeader.RTR = CAN_RTR_DATA;
	txHeader.StdId = 0x300;
	txHeader.ExtId = 0x100;
	txHeader.TransmitGlobalTime = ENABLE;
	HAL_CAN_Start(&hcan1);

	HAL_CAN_ActivateNotification(&hcan1,
	CAN_IT_RX_FIFO0_MSG_PENDING);
  /* USER CODE END CAN1_Init 2 */

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**CAN1 GPIO Configuration
    PD0     ------> CAN1_RX
    PD1     ------> CAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(CAN1_TX_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN1_TX_IRQn);
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
    HAL_NVIC_SetPriority(CAN1_RX1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX1_IRQn);
    HAL_NVIC_SetPriority(CAN1_SCE_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN1_SCE_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN1 GPIO Configuration
    PD0     ------> CAN1_RX
    PD1     ------> CAN1_TX
    */
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_0|GPIO_PIN_1);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN1_TX_IRQn);
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
    HAL_NVIC_DisableIRQ(CAN1_RX1_IRQn);
    HAL_NVIC_DisableIRQ(CAN1_SCE_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

void HAL_CAN_RxFifo0FullCallback(CAN_HandleTypeDef *hcan)
{
	if (hcan != NULL)
	{
		if (hcan->Instance == CAN1)
		{
		}
	}
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if (hcan != NULL)
	{
		if (hcan->Instance == CAN1)
		{
			HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, canRX);
			HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
//			portENTER_CRITICAL();
			flagRecv = 1;
//			portEXIT_CRITICAL();
		}
	}

}

void canStartDefaultTask(void *argument)
{
	uint8_t csend[] =
	{ 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08 };
	/* Infinite loop */
	static uint16_t seq = 0;
	uint8_t fR = 0;
	currentMode = &((evState_t*) argument)->comm;

	for (;;)
	{
//		Event cur_event = event_check();
		switch (*currentMode)
		{
		case POST:
		//				currentMode = POST_function(cur_event);
			*currentMode = IDLE;
			break;
		case IDLE:
		//				currentMode = IDLE_function(cur_event);
			*currentMode = SETTING;
			break;
		case SETTING:
		//				currentMode = SETTING_function(cur_event);
			seq++;
			csend[0] = (uint8_t) ((seq >> 8) & 0xFF);
			csend[1] = (uint8_t) ((seq >> 0) & 0xFF);

			*currentMode = RUNNING;
			break;
		case RUNNING:
		//				currentMode = RUNNING_function(cur_event);
//			portENTER_CRITICAL();
			fR = flagRecv;
//			portEXIT_CRITICAL();
			if (!fR)
				{
				csend[2] = 0x03;
				csend[3] = 0x04;
				csend[4] = 0x05;
				csend[5] = 0x06;
				csend[6] = 0x07;
				csend[7] = 0x08;
				txHeader.StdId = 0x300;
				if (HAL_CAN_AddTxMessage(&hcan1, &txHeader, csend, &canMailbox)
						!= HAL_OK)
				{
					*currentMode = ALARM;
					break;
				}
				}
			else
				{
//				portENTER_CRITICAL();
				flagRecv = 0;
//				portEXIT_CRITICAL();
				txHeader.StdId = rxHeader.StdId + 1;
				csend[2] = canRX[2];
				csend[3] = canRX[3];
				csend[4] = canRX[4];
				csend[5] = canRX[5];
				csend[6] = canRX[6];
				csend[7] = canRX[7];
				if (HAL_CAN_AddTxMessage(&hcan1, &txHeader, csend, &canMailbox)
						!= HAL_OK)
				{
					*currentMode = ALARM;
					break;
				}

				}
			*currentMode = WAITING;
			break;
		case WAITING:
			osDelay(1000);
			*currentMode = IDLE;
			break;
		case ALARM:
			//				currentMode = ALARM_function(cur_event);
			seq--;
			*currentMode = FAILSAFE;
			break;
		case FAILSAFE:
			//				currentMode = FAILSAFE_function(cur_event)
			*currentMode = POST;
			break;
		default:
			currentMode = currentMode;
			break;
		}
		osDelay(1);
	}
}
/* USER CODE END 1 */

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
uint8_t canRX[32] =
{ 0, 0, 0, 0, 0, 0, 0, 0 };  //CAN Bus Receive Buffer
uint8_t csend[32] =
{ 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08 };
CAN_FilterTypeDef canfil; //CAN Bus Filter
uint32_t canMailbox; //CAN Bus Mail box variable
uint8_t flagRecv = 0;

/* Server port, the port the server listens on for incoming connections from the client. */
#define MY_SERVER_PORT		10
#define SERVER_TC_PORT		13
#define SERVER_ACK_PORT		14
#define SERVER_STATUS_PORT	15


/* Commandline options */
static uint8_t server_address = 11;

/* test mode, used for verifying that host & client can exchange packets over the loopback interface */
static bool test_mode = false;
static unsigned int server_received = 0;
extern osThreadId_t canDefaultTaskHandle;
extern osThreadAttr_t defaultTask_attributes;
extern osThreadId_t canRouterTaskHandle;
extern osThreadAttr_t canRouterTask_attributes;
extern osThreadId_t canServerTaskHandle;
extern osThreadAttr_t canServerTask_attributes;
extern osThreadId_t canClientTaskHandle;
extern osThreadAttr_t canClientTask_attributes;
extern osMutexId_t canSendMutexHandle;
//Extend CSP_DBG_ERR

#define CSP_DBG_ERR_INVALID_CAN_CONFIG 13
#define CONFIG_CSP_CAN_RX_MSGQ_DEPTH 10

/** Driver configration */


can_context_t mcan[1] =
{
		{ .mode = 0, .lock = NULL, .interface =
		{ .name = "can0", .interface_data = &mcan[0].ifdata, .driver_data =
				&mcan[0], }, } };
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
  hcan1.Init.SyncJumpWidth = CAN_SJW_4TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_3TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = ENABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = ENABLE;
  hcan1.Init.TransmitFifoPriority = ENABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

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
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
  if (hcan != NULL) {
    if (hcan->Instance == CAN1) {
      can_async_disable(hcan);
      mcan[0].interface.tx_error++;
      can_async_enable(hcan);
      return;
    }
  }
}


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
	BaseType_t xTaskWoken = pdFALSE;
#if CAN_DEFER_TASK
	vTaskNotifyGiveFromISR(can_task_handle, &xTaskWoken);
#else
//	HAL_GPIO_WritePin(LD6_GPIO_Port, LD6_Pin, GPIO_PIN_SET);
	if (hcan != NULL)
	{

		if (hcan->Instance == CAN1)
		{
			HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, canRX);

//			portENTER_CRITICAL();
			flagRecv = 1;
//			portEXIT_CRITICAL();

//			csp_hex_dump("rx", canRX, rxHeader.DLC);

			if (rxHeader.IDE != CAN_ID_STD && rxHeader.RTR == 0)
			{
			/* Process frame within ISR
			 * (This can also be deferred to a task with: csp_can_process_frame_deferred) */
				csp_can_rx(&mcan[0].interface, rxHeader.ExtId, canRX,
						rxHeader.DLC,
					(int*) &xTaskWoken);
			}

		}
	}
//	HAL_GPIO_WritePin(LD6_GPIO_Port, LD6_Pin, GPIO_PIN_RESET);
	HAL_GPIO_TogglePin(LD6_GPIO_Port, LD6_Pin);
#endif
	portYIELD_FROM_ISR(xTaskWoken);

}

void canStartDefaultTask(void *argument)
{
	/* Infinite loop */
	static uint16_t seq = 0;
	uint8_t fR = 0;
	Mode * currentMode = &((evState_t*) argument)->comm;

//	int ret;
	uint8_t address = 0;
//	const char *kiss_device = NULL;
	const char *rtable = NULL;
	csp_iface_t *can_iface = NULL;

	osThreadSuspend(canRouterTaskHandle);
	osThreadSuspend(canServerTaskHandle);
	osThreadSuspend(canClientTaskHandle);

	csp_print("Initializing CSP\n");

	/* Start router */
	csp_dbg_packet_print = 1;
	/* Init CSP */
	csp_init();

//	router_start();
	osThreadResume(canRouterTaskHandle);
//	canRouterTaskHandle = osThreadNew(canStartRouterTask, argument,
//			&canRouterTask_attributes);

	csp_iface_t *default_iface = NULL;
	const char *ifname = "can0";
	address = 11;
	server_address = 10;
	const CAN_HandleTypeDef *device = &hcan1;
	uint32_t bitrate = 125000;

	uint16_t filter_addr = server_address;
	uint16_t filter_mask = 0; //x3FFF;
        int error;

	for (;;)
	{
//		Event cur_event = event_check();
		switch ((Mode)(*currentMode))
		{
		case POST:
			//				currentMode = POST_function(cur_event)
			error = csp_can_open_and_add_interface(device, ifname, address,
					bitrate, filter_addr, filter_mask, &can_iface);
			if (error != CSP_ERR_NONE)
			{
				csp_print("failed to add CAN interface [%s], error: %d\n",
						ifname,
						error);
				*currentMode = FAILSAFE;
				break;
			}
			can_iface->is_default = 1;
			default_iface = can_iface;

			if (rtable)
			{
				int error = csp_rtable_load(rtable);
				if (error < 1)
				{
					csp_print("csp_rtable_load(%s) failed, error: %d", rtable,
							error);
					*currentMode = FAILSAFE;
					break;
				}
			}
			else if (default_iface)
			{
				csp_rtable_set(0, 0, default_iface, CSP_NO_VIA_ADDRESS);
			}

			if (!default_iface)
			{
				/* no interfaces configured - run server and client in process, using loopback interface */
				server_address = address;
				/* run as test mode only use loopback interface */
				test_mode = true;
			}

			/*
			 * In the Zephyr port, we have disabled stdio usage and unified logging with the Zephyr
			 * logging API. As a result, the following functions currently do not print anything.
			 */
			csp_print("Connection table\n")
			;
			csp_conn_print_table();

			csp_print("Interfaces\n")
			;
			csp_iflist_print();

			csp_print("Route table\n")
			;
			csp_rtable_print();

			/* Start server thread */
//			if ((server_address == 255) || /* no server address specified, I must be server */
//			(default_iface == NULL))
//			{ /* no interfaces specified -> run server & client via loopback */
//				server_start();
			osThreadResume(canServerTaskHandle);
//			canServerTaskHandle = osThreadNew(canStartServerTask,
//					argument,
//					&canServerTask_attributes);
//			}

			/* Start client thread */
//			if ((server_address != 255) || /* server address specified, I must be client */
//			(default_iface == NULL))
//			{ /* no interfaces specified -> run server & client via loopback */
//				client_start();
			osThreadResume(canClientTaskHandle);
//			canClientTaskHandle = osThreadNew(canStartClientTask, argument,
//					&canClientTask_attributes);
//			}

			*currentMode = IDLE;
			break;
		case IDLE:
		//				currentMode = IDLE_function(cur_event);
			osDelay(3);
			if (test_mode)
			{
				/* Test mode is intended for checking that host & client can exchange packets over loopback */
				if (server_received < 5)
				{
					csp_print("Server received %u packets", server_received);
					*currentMode = FAILSAFE;
					break;
				}
				csp_print("Server received %u packets", server_received);
				*currentMode = FAILSAFE;
				break;
			}
			else
			{
				osThreadSuspend(canDefaultTaskHandle);
			}
			
//			*currentMode = SETTING;
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
				else
				{
					HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);
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
				else
				{
					HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);
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
			csp_can_stop(can_iface);
			*currentMode = POST;
			break;
		default:
			*currentMode = *currentMode;
			break;
		}

	}
}

void canStartRouterTask(void *argument)
{
//	Mode * currentMode = &((evState_t*) argument)->comm;

	for (;;)
	{
//		//Event cur_event = event_check();
//		switch (*currentMode)
//		{
//		case POST:
////			currentMode = POST_function(cur_event);
//			*currentMode = RUNNING;
//			break;
//		case IDLE:
////			currentMode = IDLE_function(cur_event);
//			break;
//		case SETTING:
////			currentMode = SETTING_function(cur_event);
//			break;
//		case RUNNING:
//			currentMode = RUNNING_function(cur_event);
			csp_route_work();
//			break;
//		case ALARM:
////			currentMode = ALARM_function(cur_event);
//			break;
//		case FAILSAFE:
////			currentMode = FAILSAFE_function(cur_event);
//			break;
//		default:
//			*currentMode = *currentMode;
//			break;
//		}
//		osDelay(1);
	}
}

void TC_RECV_Handler(csp_packet_t *packet)
{
	/* TODO:  ACK*/
	csp_packet_t *ack_packet = csp_buffer_get(0);
	ack_packet->length = sprintf((char*) ack_packet->data, "%s", packet->data);
	memset(ack_packet->data + 2, '1', 1);
	memset(ack_packet->data + ack_packet->length, 0, 1);
	ack_packet->length++;
	csp_sendto_reply(packet, ack_packet, CSP_O_SAME);
	//				csp_can_send_ack(ack_packet, server_address, 1000, CSP_O_NONE);
	csp_print("ACK Packet send on PORT (%d): %s (%ld)\n", SERVER_ACK_PORT,
			(char* ) ack_packet->data, ack_packet->length);
	csp_buffer_free(ack_packet);
	csp_buffer_free(packet);
}

void canStartServerTask(void *argument)
{
//	Mode * currentMode = &((evState_t*) argument)->comm;

	csp_socket_t sock =
	{ 0 };
	int dp = 0;

	csp_print("Server task started\n");

	csp_bind_callback(TC_RECV_Handler, SERVER_TC_PORT);
	/* Bind socket to all ports, e.g. all incoming connections will be handled here */
	csp_bind(&sock, CSP_ANY);


	/* Create a backlog of 10 connections, i.e. up to 10 new connections can be queued */
	csp_listen(&sock, 64);

	for (;;)
	{
		ulTaskNotifyTake(true, 1 * configTICK_RATE_HZ);
//		/* Wait for a new connection, 10000 mS timeout */
		csp_conn_t *conn;
		if ((conn = csp_accept(&sock, 10000)) == NULL)
		{
			/* timeout */
			continue;
		}

		/* Read packets on connection, timout is 10 mS */
		csp_packet_t *packet;
		while ((packet = csp_read(conn, 50)) != NULL)
		{
			dp = csp_conn_dport(conn);
			switch (dp)
			{
			case SERVER_TC_PORT:
				/* Process packet here */
				csp_print("TC Packet received on PORT (%d): %s (%ld)\n", dp,
						(char* ) packet->data, packet->length)
				;
				TC_RECV_Handler(packet);
				server_received++;
				break;

			default:
				/* Call the default CSP service handler, handle pings, buffer use, etc. */
				csp_service_handler(packet);
				break;
			}
		}

		/* Close current connection */
		csp_close(conn);
	}
}

/* Client task sending requests to server task */
void canStartClientTask(void *argument)
{
//	Mode * currentMode = &((evState_t*) argument)->comm;

	uint16_t count = 0;
	uint16_t server_addr = 10;
	int dport = SERVER_STATUS_PORT;
	csp_print("Client task started\n");

	for (;;)
	{
		ulTaskNotifyTake(true, 1 * configTICK_RATE_HZ);
		//		k_sleep(test_mode ? K_USEC(200000) : K_USEC(1000000));
		osDelay(test_mode ? 200 : 1000);
		HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);
		/* Send ping to server, timeout 1000 mS, ping size 10 bytes */
		int result = csp_ping(server_addr, 1000, 10, CSP_O_NONE);
		csp_print("Ping address: %u, result %d [mS]\n", server_addr, result);

		if (result == -1)
		{
			/* Send reboot request to server, the server has no actual implementation of csp_sys_reboot() and fails to reboot */
//			csp_reboot(server_addr);
//			csp_print("reboot system request sent to address: %u\n", server_addr);
			continue;
		}

		/* Send data packet (string) to server */

		/* 1. Get packet buffer for message/data */
		csp_packet_t *packet = csp_buffer_get(0);
		if (packet == NULL)
		{
			/* Could not get buffer element */
			csp_print("Failed to get CSP buffer\n");
			continue;
		}

		/* 2. Copy data to packet */
		csp_print("Send status on Port (%d): 0x%03X [%d] ", SERVER_STATUS_PORT,
				0x300 & 0xfff, 8);
		unsigned int alen = sprintf((char*) packet->data, "%03X#",
				0x300 & 0xfff);

		csend[0] = ((uint8_t) (count >> 8)) & 0xFF;
		csend[1] = ((uint8_t) (count >> 0)) & 0xFF;
		for (int i = 0; i < 8; i++)
		{
			csp_print("%02X ", csend[i]);
			alen += sprintf((char*) packet->data + alen, "%02X", csend[i])
			;
		}

		memset(packet->data + alen, 0, 1);
		alen++;
		csp_print("(%d)\n", alen);
		count++;

		/* 3. Set packet length */
		packet->length = alen; //(strlen((char*) packet->data) + 1); /* include the 0 termination */

		/* 4. Connect to host on 'server_address', port MY_SERVER_PORT with regular UDP-like protocol and 1000 ms timeout */
		csp_conn_t *conn = csp_connect(CSP_PRIO_NORM, server_addr, dport, 1000,
		CSP_O_NONE);
		if (conn == NULL)
		{
			/* Connect failed */
			csp_print("Connection failed\n");
			csp_buffer_free(packet);
			continue;
		}

		/* 5. Send packet */
		csp_send(conn, packet);

		/* 6. Close connection */
		csp_buffer_free(packet);
		csp_close(conn);
	}
}

/**
 * Open CAN and add CSP interface.
 */
int csp_can_open_and_add_interface(const CAN_HandleTypeDef *device,
		const char *ifname, uint16_t address, uint32_t bitrate,
		uint16_t filter_addr, uint16_t filter_mask, csp_iface_t **return_iface)
{
	/* Create a mutex type semaphore. */
	mcan[0].lock = xSemaphoreCreateMutexStatic(&mcan[0].lock_buf);

	int ret;
	unsigned int netmask = filter_mask;
//	osThreadId_t rx_tid;
	can_context_t *ctx = &mcan[0];
	const char *name = ifname ? ifname : mcan[0].name;

	if (device == NULL)
	{
		ret = CSP_ERR_INVAL;
		goto end;
	}

	const csp_conf_t *csp_conf = csp_get_conf();
	if (csp_conf->version == 2)
	{

//		canfil.FilterMaskIdLow = CFP2_DST_MASK << CFP2_DST_OFFSET;
		canfil.FilterMaskIdHigh = netmask << CFP2_DST_OFFSET;

		mcan[0].id = address << CFP2_DST_OFFSET;
		mcan[0].id_l3bc = ((1 << (csp_id_get_host_bits() - netmask)) - 1)
				<< CFP2_DST_OFFSET;
		mcan[0].id_l2bc = 0x3FFF << CFP2_DST_OFFSET;
	}
	else
	{

		canfil.FilterMaskIdLow = CFP_MAKE_DST((1 << CFP_HOST_SIZE) - 1);

		mcan[0].id = CFP_MAKE_DST(address);
		mcan[0].id_l2bc = 0; //! Not supported on CSP1
		mcan[0].id_l3bc = 0; //! Not supported on CSP1
	}

//	if (rx_thread_idx >= CONFIG_CSP_CAN_RX_THREAD_NUM)
//	{
//		LOG_ERR(
//				"[%s] No more RX thread can be created. (MAX: %d) Please check CONFIG_CSP_CAN_RX_THREAD_NUM.",
//				name, CONFIG_CSP_CAN_RX_THREAD_NUM);
//		ret = CSP_ERR_DRIVER;
//		goto end;
//	}

	/*
	 * TODO:
	 * In the current implementation, we use k_alloc() to allocate the memory
	 * for CAN context like socketcan implementation.
	 * However, we plan to remove dynamic memory allocations as described in
	 * the issue below.
	 * - https://github.com/libcsp/libcsp/issues/460
	 * And in Zephyr, the default heap memory size is 0, so we need to set a
	 * value using CONFIG_HEAP_MEM_POOL_SIZE.
	 */
//	ctx = calloc(1, sizeof(can_context_t));
//	if (ctx == NULL)
//	{
//		LOG_ERR("[%s] Failed to allocate %zu bytes from the system heap", name,
//				sizeof(can_context_t));
//		ret = CSP_ERR_NOMEM;
//		goto end;
//	}

	/* Set the each parameter to CAN context. */
	strncpy(ctx->name, name, sizeof(ctx->name) - 1);
	ctx->interface.name = ctx->name;
	ctx->interface.addr = address;
	ctx->interface.netmask = netmask;
	ctx->interface.interface_data = &ctx->ifdata;
	ctx->interface.driver_data = ctx;
	ctx->ifdata.tx_func = csp_can_tx_frame;
	ctx->ifdata.pbufs = NULL;
	ctx->device = (CAN_HandleTypeDef*) device;
	ctx->filter_id = -1;
	ctx->stop_can_event = osEventFlagsNew((osEventFlagsAttr_t*
			)
			{ 0 });

	csp_print(
			"INIT %s: device, local address: %d, bitrate: %ld: filter add: %d, filter mask: 0x%04x\n",
			ctx->name, address, bitrate, filter_addr,
			filter_mask);

	/* Initialize the RX message queue */
//	k_msgq_init(&ctx->rx_msgq, ctx->rx_msgq_buf, sizeof(struct can_frame),
//			CONFIG_CSP_CAN_RX_MSGQ_DEPTH);

	/* Set Bit rate */
//	ret = can_set_bitrate(device, bitrate);
//	if (ret < 0)
//	{
//		LOG_ERR("[%s] can_set_bitrate() failed, error: %d", ctx->name, ret);
//		goto cleanup_heap;
//	}
	MX_CAN1_Init();

	/* Set RX filter */
	ret = csp_can_set_rx_filter(&ctx->interface, filter_addr, filter_mask);
	if (ret < 0)
	{
		csp_print("[%s] csp_can_add_rx_filter() failed, error: %d", ctx->name,
				ret);
//		goto cleanup_heap;
		return ret;
	}

	/* Add interface to CSP */
	ret = csp_can_add_interface(&ctx->interface);
	if (ret != CSP_ERR_NONE)
	{
		csp_print("[%s] csp_can_add_interface() failed, error: %d", ctx->name,
				ret);
		return ret; //goto cleanup_filter;
	}

	/* Create receive thread */
//	rx_tid = k_thread_create(&ctx->rx_thread, rx_stack[rx_thread_idx],
//			K_THREAD_STACK_SIZEOF(rx_stack[rx_thread_idx]),
//			(k_thread_entry_t) csp_can_rx_thread, &ctx->rx_msgq, &ctx->iface,
//			&ctx->stop_can_event, CONFIG_CSP_CAN_RX_THREAD_PRIORITY, 0,
//			K_NO_WAIT);
//	if (!rx_tid)
//	{
//		LOG_ERR("[%s] k_thread_create() failed", ctx->name);
//		ret = CSP_ERR_DRIVER;
//		goto cleanup_iface;
//	}
//	rx_thread_idx++;

	/* Enable CAN */
	ret = HAL_CAN_Start((CAN_HandleTypeDef*) device);
	if (ret < 0)
	{
		csp_print("[%s] can_start() failed, error: %d", ctx->name, ret);
		return ret; //goto cleanup_thread;
	}
	HAL_CAN_ActivateNotification(&hcan1,
	CAN_IT_RX_FIFO0_MSG_PENDING);

	if (return_iface)
	{
		*return_iface = &ctx->interface;
	}

	/*
	 * The following section is for restoring acquired resources when
	 * something fails. Unfortunately, we can't take any action if the
	 * restoration process fails, so we proceed with the remaining
	 * cleanup. In addtion to this, we've chosen not to restore the
	 * CAN bit rate. If this causes any issues, please open an issue
	 * on GitHub.
	 */
//	cleanup_thread: //(void) csp_can_finish_rx_thread(ctx);

//	cleanup_iface: (void) csp_can_remove_interface(&ctx->interface);

//	cleanup_filter: //csp_can_remove_rx_filter(ctx);

//	cleanup_heap: free(ctx);

	end: return ret;
}

/**
 * Set CAN RX filter based on destination address and bit mask.
 */
int csp_can_set_rx_filter(csp_iface_t *iface, uint16_t filter_addr,
		uint16_t filter_mask)
{
	int ret = 0;
	CAN_FilterTypeDef *filter = &canfil;
	can_context_t *ctx;

	if ((iface == NULL) || (iface->driver_data == NULL))
	{
		ret = CSP_ERR_INVAL;
		goto end;
	}

	ctx = iface->driver_data;

	/* If a filter is already set, delete it. */
	if (ctx->filter_id >= 0)
	{
//		can_remove_rx_filter(ctx->device, ctx->filter_id);
	}

	if (csp_conf.version == 1)
	{
		filter->FilterIdHigh = CFP_MAKE_DST(filter_addr);
		filter->FilterMaskIdHigh = CFP_MAKE_DST(filter_mask);

		//filter->FilterIdLow = CFP_MAKE_DST(filter_addr);
		//filter->FilterMaskIdLow = CFP_MAKE_DST(filter_mask);
	}
	else
	{
		filter->FilterIdHigh = filter_addr << CFP2_DST_OFFSET;
		filter->FilterMaskIdHigh = filter_mask << CFP2_DST_OFFSET;

		//filter->FilterIdLow = filter_addr << CFP2_DST_OFFSET;
		//filter->FilterMaskIdLow = filter_mask << CFP2_DST_OFFSET;
	}

	filter->FilterBank = 0;
	filter->FilterMode = CAN_FILTERMODE_IDMASK;
	filter->FilterFIFOAssignment = CAN_RX_FIFO0;
	filter->FilterScale = CAN_FILTERSCALE_32BIT;
	filter->FilterActivation = ENABLE;
	filter->SlaveStartFilterBank = 14;

//	ret = can_add_rx_filter_msgq(ctx->device, &ctx->rx_msgq, &filter);
	ret = HAL_CAN_ConfigFilter(ctx->device, filter);
	if (ret < 0)
	{
		csp_print("[%s] can_add_rx_filter_msgq() failed, error: %d\n",
				iface->name,
				ctx->filter_id);
		goto end;
	}

	ctx->filter_id = ret;

	end: return ret;
}

/**
 * Stop the CAN and RX thread
 */
int csp_can_stop(csp_iface_t *iface)
{
	int ret;
	can_context_t *ctx;

	if ((iface == NULL) || (iface->driver_data == NULL))
	{
		ret = CSP_ERR_INVAL;
		goto end;
	}

	ctx = iface->driver_data;

	ret = HAL_CAN_Stop(ctx->device);
	if (ret < 0)
	{
		csp_print(
				"[%s] can_stop() failed, but will continue the cleannp. error: %d",
				iface->name, ret);
	}

//	(void) csp_can_finish_rx_thread(ctx);

	(void) csp_can_remove_interface(&ctx->interface);

//	csp_can_remove_rx_filter(ctx);

	csp_print("Stop CAN interface: %s. ret: %d", iface->name, ret);

//	free(ctx);

	end: return ret;
}

void can_async_disable(void *dev)
{
	HAL_CAN_Stop(&hcan1);
}
void can_async_enable(void *dev)
{
	HAL_CAN_Start(&hcan1);
}

int can_async_write(void * dev, struct can_message * msg)
{
        txHeader.StdId = msg->id;
	txHeader.ExtId = msg->extid;
	txHeader.RTR = msg->type;
	txHeader.DLC = msg->len;
	txHeader.IDE = msg->fmt;
	txHeader.TransmitGlobalTime = ENABLE;
#if defined (STM32F407xx)
        return HAL_CAN_AddTxMessage((CAN_HandleTypeDef * ) dev, &txHeader, msg->data, &canMailbox);
#else
//Other platform CAN message transmit
#endif
        
}

void can_async_set_filter(void * dev, int filter_block, int can_ide, void * filter)
{
}

int csp_can_tx_frame(void *driver_data, uint32_t id, const uint8_t *data,
		uint8_t dlc) {
	int ret = CSP_ERR_NONE;
	
	can_context_t * ctx = driver_data;
	
        if ((ctx == NULL) || (ctx->lock == NULL)) {
		return 0;
	}
        
        if (dlc > CAN_MAX_DLC)
	{
		ret = CSP_ERR_INVAL;
		goto end;
	}

	struct can_message msg;
	msg.extid = id;
	msg.type = CAN_TYPE_DATA;
	msg.data = (uint8_t * ) data;
	msg.len = dlc;
	msg.fmt  = CAN_FMT_EXTID;
        
        /* Task locking */
	while(xSemaphoreTake(ctx->lock, 1) == pdFALSE);
#if 0
//	ret = can_send(ctx->device, &frame, CSP_CAN_TX_TIME_OUT, NULL, NULL);
	ret = HAL_CAN_AddTxMessage(ctx->device, &txHeader, buf, &canMailbox);
	if (ret < 0)
	{
		csp_print("[%s] can_send() failed, errno %d", ctx->name, ret);
	}
#else
        /**
	 * Blocking IO:
	 *
	 * With CAN blocking IO is needed because we only have room for 32 frames or 256 bytes.
	 * This is barely enough to hold a large CSP packet
	 * We could go for async transmission with software queues, this will allow for larger amounts
	 * of data to be queued. However in this case, simplicity is chosen over performance.
	 * When the TX FIFO is full, we ask the task to sleep a tick.
	 * Usually data heavy fuctions are running in their separate tasks anyways.
	 * A CAN frame is 96 bits and is transmitted within 96 us.
	 * A tick period can vary from 1 to 10 ms (typically)
	 * 3 retries have been chosen because a futher dealy than this would almost certainly be
	 * due to an error.
	 */
	int attempts = 3;
	while((attempts > 0) && (can_async_write((void * ) (ctx->device), &msg) != ERR_NONE)) {
		vTaskDelay(1);
		attempts--;
	}
  #endif
        HAL_GPIO_TogglePin(LD5_GPIO_Port, LD5_Pin);
        /* Unlock */
	xSemaphoreGive(ctx->lock);
	
	end:
	return ret;
}

int csp_can_send_ack(csp_packet_t *packet, uint16_t node, uint32_t timeout,
		uint8_t conn_options) {
	uint32_t start, time;

	/* Counter */
	start = csp_get_ms();

	/* Open connection */
	csp_conn_t *conn = csp_connect(CSP_PRIO_NORM, node, SERVER_ACK_PORT,
			timeout,
			conn_options);
	if (conn == NULL)
		return -1;

	/* Try to send frame */
	csp_send(conn, packet);

	/* Clean up */
	csp_close(conn);

	/* We have a reply */
	time = (csp_get_ms() - start);

	return time;
}



#ifndef CAN_DEFER_TASK
#define CAN_DEFER_TASK 0
#endif

#if CAN_DEFER_TASK
static StaticTask_t can_task_tcb;
static StackType_t can_task_stack[500];
static TaskHandle_t can_task_handle;
#endif


#if CAN_DEFER_TASK
void CAN_0_rx_callback_task(struct can_async_descriptor *const descr) {
	BaseType_t xTaskWoken = pdFALSE;
	vTaskNotifyGiveFromISR(can_task_handle, &xTaskWoken);
	portYIELD_FROM_ISR(xTaskWoken);
}

static void can_task(void * param) {
	
	while(1) {

		ulTaskNotifyTake(true, 1 * configTICK_RATE_HZ);

		int xTaskWoken = pdFALSE;
		struct can_message msg;
		uint8_t data[8];
		msg.data = data;

		while(can_async_read(&CAN_0, &msg) == ERR_NONE) {

			//csp_hex_dump("rx", msg.data, msg.len);

			/* Process frame within ISR
			* (This can also be deferred to a task with: csp_can_process_frame_deferred) */
			csp_can_rx(&mcan[0].interface, msg.id, msg.data, msg.len, &xTaskWoken);

		}

		if (xTaskWoken) {
			portYIELD();
		}

	}

}
#endif

csp_iface_t * csp_driver_can_init(int addr, int netmask, int id, can_mode_e mode, uint32_t bitrate) {

	/* Create a mutex type semaphore. */
	mcan[id].lock = xSemaphoreCreateMutexStatic(&mcan[id].lock_buf);

	const csp_conf_t *csp_conf = csp_get_conf();

	/* Generate ID */
    uint32_t can_mask = 0;
    if (csp_conf->version == 2) {
        
        can_mask = CFP2_DST_MASK << CFP2_DST_OFFSET;

        mcan[id].id = addr << CFP2_DST_OFFSET;
        mcan[id].id_l3bc = ((1 << (csp_id_get_host_bits() - netmask)) - 1) << CFP2_DST_OFFSET;
        mcan[id].id_l2bc = 0x3FFF << CFP2_DST_OFFSET;
    } else {

        can_mask = CFP_MAKE_DST((1 << CFP_HOST_SIZE) - 1);

        mcan[id].id = CFP_MAKE_DST(addr);
        mcan[id].id_l2bc = 0; //! Not supported on CSP1
        mcan[id].id_l3bc = 0; //! Not supported on CSP1
    }

	if (mode == CSP_CAN_MASKED) {
		mcan[id].mask = can_mask;
	} else if (mode == CSP_CAN_PROMISC) {
		mcan[id].mask = 0;
	} else {
		csp_dbg_errno = CSP_DBG_ERR_INVALID_CAN_CONFIG;
		return NULL;
	}

	mcan[id].ifdata.tx_func = csp_can_tx_frame;
	mcan[id].ifdata.pbufs = NULL;
	mcan[id].interface.interface_data = &mcan[id].ifdata;

	mcan[id].interface.addr = addr;
	mcan[id].interface.netmask = netmask;

	/* Regsiter interface */
	csp_can_add_interface(&mcan[id].interface);

#if CAN_DEFER_TASK
	/* CAN task */
	can_async_register_callback(&CAN_0, CAN_ASYNC_RX_CB, (FUNC_PTR)CAN_0_rx_callback_task);
	can_task_handle = xTaskCreateStatic(can_task, "CAN", 500, NULL, 1, can_task_stack, &can_task_tcb);

#else

	/* CAN ISR */
	can_async_register_callback(&hcan1, CAN_ASYNC_RX_CB, (FUNC_PTR)HAL_CAN_RxFifo0MsgPendingCallback);
#endif

	can_async_register_callback(&hcan1, CAN_ASYNC_IRQ_CB, (FUNC_PTR)HAL_CAN_ErrorCallback);
	can_async_enable(&hcan1);

    /* Filter for own address */
    struct can_filter filter;
    filter.id   = mcan[id].id;
    filter.mask = mcan[id].mask;
    //csp_print("H filter %x %x\n", filter.id, filter.mask);
    can_async_set_filter(&hcan1, 0, CAN_FMT_EXTID, &filter);

    /* Filter for subnet broadcast address */
    filter.id   = mcan[id].id_l3bc;
    filter.mask = mcan[id].mask;
    //csp_print("L3 filter %x %x\n", filter.id, filter.mask);
    can_async_set_filter(&hcan1, 1, CAN_FMT_EXTID, &filter);

    /* Filter for layer-2 broadcast */
    filter.id   = mcan[id].id_l2bc;
    filter.mask = mcan[id].mask;
    //csp_print("L2 filter %x %x\n", filter.id, filter.mask);
    can_async_set_filter(&hcan1, 2, CAN_FMT_EXTID, &filter);

	return &mcan[id].interface;

}
/* USER CODE END 1 */

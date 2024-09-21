/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.h
  * @brief   This file contains all the function prototypes for
  *          the can.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CAN_H__
#define __CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include <semphr.h>
#include <task.h>

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdbool.h>

//CubeSat Space Protocol
#include "csp/csp_debug.h"
#include "csp/csp.h"
#include "csp/csp_id.h"
#include "csp/arch/csp_time.h"
#include "csp/interfaces/csp_if_can.h"

/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan1;

/* USER CODE BEGIN Private defines */
#define CSP_DBG_ERR_INVALID_CAN_CONFIG  13
#define CONFIG_CSP_CAN_RX_MSGQ_DEPTH    10
#define ERR_NONE                        CSP_ERR_NONE
#define CAN_TYPE_DATA                   CAN_RTR_DATA
#define CAN_TYPE_REMOTE                 CAN_RTR_REMOTE
#define CAN_FMT_STDID                   CAN_ID_STD
#define CAN_FMT_EXTID                   CAN_ID_EXT
#define CAN_ASYNC_RX_CB                 HAL_CAN_RX_FIFO0_MSG_PENDING_CB_ID
#define CAN_ASYNC_IRQ_CB                HAL_CAN_ERROR_CB_ID
typedef void (* FUNC_PTR)(CAN_HandleTypeDef *_hcan);
#define CAN_MAX_DLC 8

typedef enum eCAN_MODE
{
	CSP_CAN_MASKED, 
        CSP_CAN_PROMISC,
} can_mode_e;

typedef struct can_message
{
        union {
          CAN_RxHeaderTypeDef header;
          struct{
            uint32_t id;
            uint32_t extid;
            uint32_t fmt;
            uint32_t type;
            uint32_t len;
            uint32_t timestamp;
            uint32_t filter_id;
          };
        };
	uint8_t * data;
} can_message;

typedef struct can_filter
{
  uint32_t id;
  uint32_t mask;
} can_filter;

typedef struct mcan_s
{
	can_mode_e mode;
	uint32_t id;
	uint32_t id_l3bc;
	uint32_t id_l2bc;
	uint32_t mask;
	csp_can_interface_data_t ifdata;
	SemaphoreHandle_t lock;
	StaticSemaphore_t lock_buf;
	csp_iface_t interface;
#if defined (STM32F407xx)
        char name[CSP_IFLIST_NAME_MAX + 1];
        CAN_HandleTypeDef *device;
	osMessageQId rx_msgq;
	osEventFlagsId_t stop_can_event;
	osThreadId_t rx_thread;
	uint8_t rx_msgq_buf[sizeof(CAN_FIFOMailBox_TypeDef)
			* CONFIG_CSP_CAN_RX_MSGQ_DEPTH];
	int filter_id;
#endif
} can_context_t;
/* USER CODE END Private defines */

void MX_CAN1_Init(void);

/* USER CODE BEGIN Prototypes */
extern void canStartDefaultTask(void *argument);
extern void canStartRouterTask(void *argument);
extern void canStartServerTask(void *argument);
extern void canStartClientTask(void *argument);

/**
 * Open CAN and add CSP interface.
 *
 *  @param[in] device CAN device structure.
 *  @param[in] ifname CSP interface name.
 *  @param[in] address CSP address of the interface.
 *  @param[in] bitrate CAN bitrate.
 *  @param[in] filter_addr Destination address you want to set in the RX filter.
 *  @param[in] filter_mask Bit mask you want to set in the RX filter.
 *  @param[out] return_iface Added interface
 *  @return #CSP_ERR_NONE on success, otherwise an error code.
 */
int csp_can_open_and_add_interface(const CAN_HandleTypeDef *device,
		const char *ifname, uint16_t address, uint32_t bitrate,
		uint16_t filter_addr, uint16_t filter_mask, csp_iface_t **return_iface);

/**
 * Set CAN RX filter based on destination address and bit mask.
 *
 *  @param[in] iface Interface to set the RX filter.
 *  @param[in] filter_addr Destination address you want to set in the RX filter.
 *  @param[in] filter_mask Bit mask you want to set in the RX filter.
 *  @return Filter ID, a negative value on failure.
 */
int csp_can_set_rx_filter(csp_iface_t *iface, uint16_t filter_addr,
		uint16_t filter_mask);

/**
 * Stop the CAN and RX thread
 *
 *  @param[in] iface Interface to stop.
 *  @return #CSP_ERR_NONE on success, otherwise an error code.
 */
int csp_can_stop(csp_iface_t *iface);

int csp_can_tx_frame(void *driver_data, uint32_t id, const uint8_t *data,
		uint8_t dlc);

int csp_can_send_ack(csp_packet_t *packet, uint16_t node, uint32_t timeout,
		uint8_t conn_options);

struct can_async_descriptor {

};
struct _can_async_device {

};
enum can_async_interrupt_type{
  SIGNAL,
  EVENT
};

#if CAN_DEFER_TASK
void CAN_0_rx_callback_task(struct can_async_descriptor *const descr);
static void can_task(void * param);
#endif

int csp_can_tx_frame(void *driver_data, uint32_t id, const uint8_t * data, uint8_t dlc);
csp_iface_t * csp_driver_can_init(int addr, int netmask, int id, can_mode_e mode, uint32_t bitrate);
void can_async_disable(void * dev);
void can_async_enable(void * dev);
#if 0
void CAN_0_rx_callback(struct can_async_descriptor *const descr);
void CAN_0_irq_callback(struct _can_async_device *dev, enum can_async_interrupt_type type);
void can_async_register_callback(void * dev, int FUNC_ID_CB, void * func_callback);
#else
#define CAN_0_rx_callback HAL_CAN_RxFifo0MsgPendingCallback
#define CAN_0_irq_callback HAL_CAN_ErrorCallback
#define can_async_register_callback HAL_CAN_RegisterCallback
#endif
int can_async_write(void * dev, struct can_message * msg);
void can_async_set_filter(void * dev, int filter_block, int can_ide, void * filter);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */


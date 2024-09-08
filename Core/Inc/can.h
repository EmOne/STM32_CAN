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
#include "csp/csp.h"
#include "csp/interfaces/csp_if_can.h"

/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan1;

/* USER CODE BEGIN Private defines */
typedef enum eCAN_MODE
{
	CSP_CAN_MASKED, CSP_CAN_PROMISC,
} can_mode_e;
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
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */


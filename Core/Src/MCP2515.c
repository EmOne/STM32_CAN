/*
    (c) 2016 Microchip Technology Inc. and its subsidiaries. You may use this
    software and any derivatives exclusively with Microchip products.

    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
    WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
    PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION
    WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION.

    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
    BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
    FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
    ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
    THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.

    MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE
    TERMS.
*/

#include "MCP2515.h"
#include "gpio.h"

uint8_t mcpMode;
uint8_t nReservedTx = 0; // Count of tx buffers for reserved send
/* SPI related variables */
extern SPI_HandleTypeDef        hspi1;
#define SPI_CAN                 &hspi1
#define SPI_TIMEOUT             10
#define MCP2515_CS_HIGH()   HAL_GPIO_WritePin(CAN_CS_GPIO_Port, CAN_CS_Pin, GPIO_PIN_SET)
#define MCP2515_CS_LOW()    HAL_GPIO_WritePin(CAN_CS_GPIO_Port, CAN_CS_Pin, GPIO_PIN_RESET)
#ifdef MCP2515_SELECT
#undef MCP2515_SELECT
#define MCP2515_SELECT		MCP2515_CS_LOW()
#endif
#ifdef MCP2515_UNSELECT
#undef MCP2515_UNSELECT
#define MCP2515_UNSELECT	MCP2515_CS_HIGH()
#endif

/* Prototypes */
static void SPI_Tx(uint8_t data);
static void SPI_TxBuffer(uint8_t *buffer, uint8_t length);
static uint8_t SPI_Rx(void);
static void SPI_RxBuffer(uint8_t *buffer, uint8_t length);

/*********************************************************************************************************
 ** Function name:           txCtrlReg
 ** Descriptions:            return tx ctrl reg according to tx buffer index.
 **                          According to my tests this is faster and saves memory compared using vector
 *********************************************************************************************************/
uint8_t txCtrlReg(uint8_t i)
{
	switch (i)
	{
	case 0:
		return MCP_TXB0CTRL;
	case 1:
		return MCP_TXB1CTRL;
	case 2:
		return MCP_TXB2CTRL;
	}
	return MCP_TXB2CTRL;
}

/*********************************************************************************************************
 ** Function name:           statusToBuffer
 ** Descriptions:            converts CANINTF status to tx buffer index
 *********************************************************************************************************/
uint8_t statusToTxBuffer(uint8_t status)
{
	switch (status)
	{
	case MCP_TX0IF:
		return 0;
	case MCP_TX1IF:
		return 1;
	case MCP_TX2IF:
		return 2;
	}

	return 0xff;
}

/*********************************************************************************************************
 ** Function name:           statusToBuffer
 ** Descriptions:            converts CANINTF status to tx buffer sidh
 *********************************************************************************************************/
uint8_t statusToTxSidh(uint8_t status)
{
	switch (status)
	{
	case MCP_TX0IF:
		return MCP_TXB0SIDH;
	case MCP_TX1IF:
		return MCP_TXB1SIDH;
	case MCP_TX2IF:
		return MCP_TXB2SIDH;
	}

	return 0;
}

/*********************************************************************************************************
 ** Function name:           txSidhToTxLoad
 ** Descriptions:            return tx load command according to tx buffer sidh register
 *********************************************************************************************************/
uint8_t txSidhToRTS(uint8_t sidh)
{
	switch (sidh)
	{
	case MCP_TXB0SIDH:
		return MCP_RTS_TX0;
	case MCP_TXB1SIDH:
		return MCP_RTS_TX1;
	case MCP_TXB2SIDH:
		return MCP_RTS_TX2;
	}
	return 0;
}

/*********************************************************************************************************
 ** Function name:           txSidhToTxLoad
 ** Descriptions:            return tx load command according to tx buffer sidh register
 *********************************************************************************************************/
uint8_t txSidhToTxLoad(uint8_t sidh)
{
	switch (sidh)
	{
	case MCP_TXB0SIDH:
		return MCP_LOAD_TX0;
	case MCP_TXB1SIDH:
		return MCP_LOAD_TX1;
	case MCP_TXB2SIDH:
		return MCP_LOAD_TX2;
	}
	return 0;
}

/*********************************************************************************************************
 ** Function name:           txIfFlag
 ** Descriptions:            return tx interrupt flag
 *********************************************************************************************************/
uint8_t txIfFlag(uint8_t i)
{
	switch (i)
	{
	case 0:
		return MCP_TX0IF;
	case 1:
		return MCP_TX1IF;
	case 2:
		return MCP_TX2IF;
	}
	return 0;
}

/*********************************************************************************************************
 ** Function name:           txStatusPendingFlag
 ** Descriptions:            return buffer tx pending flag on status
 *********************************************************************************************************/
uint8_t txStatusPendingFlag(uint8_t i)
{
	switch (i)
	{
	case 0:
		return MCP_STAT_TX0_PENDING;
	case 1:
		return MCP_STAT_TX1_PENDING;
	case 2:
		return MCP_STAT_TX2_PENDING;
	}
	return 0xff;
}

/* initialize MCP2515 */
bool MCP2515_Initialize(void)
{
	MCP2515_UNSELECT;

  uint8_t loop = 10;

  do {
    /* check SPI Ready */
    if(HAL_SPI_GetState(SPI_CAN) == HAL_SPI_STATE_READY)
      return true;
    
    loop--;
  } while(loop > 0); 
      
  return false;
}

/* change mode as configuration mode */
bool MCP2515_SetConfigMode(void)
{
  /* configure CANCTRL Register */
  MCP2515_WriteByte(MCP2515_CANCTRL, 0x80);
  
  uint8_t loop = 10;
  
  do {    
    /* confirm mode configuration */
    if((MCP2515_ReadByte(MCP2515_CANSTAT) & 0xE0) == 0x80)
      return true;
    
    loop--;
  } while(loop > 0); 
  
  return false;
}

/* change mode as normal mode */
bool MCP2515_SetNormalMode(void)
{
  /* configure CANCTRL Register */
  MCP2515_WriteByte(MCP2515_CANCTRL, 0x00);
  
  uint8_t loop = 10;
  
  do {    
    /* confirm mode configuration */
    if((MCP2515_ReadByte(MCP2515_CANSTAT) & 0xE0) == 0x00)
      return true;
    
    loop--;
  } while(loop > 0);
  
  return false;
}

/* Entering sleep mode */
bool MCP2515_SetSleepMode(void)
{
  /* configure CANCTRL Register */
  MCP2515_WriteByte(MCP2515_CANCTRL, 0x20);
  
  uint8_t loop = 10;
  
  do {    
    /* confirm mode configuration */
    if((MCP2515_ReadByte(MCP2515_CANSTAT) & 0xE0) == 0x20)
      return true;
    
    loop--;
  } while(loop > 0);
  
  return false;
}

/*********************************************************************************************************
 ** Function name:           mcp2515_setCANCTRL_Mode
 ** Descriptions:            set control mode
 *********************************************************************************************************/
uint8_t mcp2515_setCANCTRL_Mode(const uint8_t newmode) // set mode
{
	// If the chip is asleep and we want to change mode then a manual wake needs to be done
	// This is done by setting the wake up interrupt flag
	// This undocumented trick was found at https://github.com/mkleemann/can/blob/master/can_sleep_mcp2515.c
	if ((mcp2515_getMode()) == MODE_SLEEP && newmode != MODE_SLEEP)
	{
		// Make sure wake interrupt is enabled
		uint8_t wakeIntEnabled = (MCP2515_ReadByte(MCP2515_CANINTE) & MCP_WAKIF);
		if (!wakeIntEnabled)
		{
			MCP2515_BitModify(MCP2515_CANINTE, MCP_WAKIF, MCP_WAKIF);
		}

		// Set wake flag (this does the actual waking up)
		MCP2515_BitModify(MCP2515_CANINTE, MCP_WAKIF, MCP_WAKIF);

		// Wait for the chip to exit SLEEP and enter LISTENONLY mode.

		// If the chip is not connected to a CAN bus (or the bus has no other powered nodes) it will sometimes trigger the wake interrupt as soon
		// as it's put to sleep, but it will stay in SLEEP mode instead of automatically switching to LISTENONLY mode.
		// In this situation the mode needs to be manually set to LISTENONLY.

		if (mcp2515_requestNewMode(MODE_LISTENONLY) != MCP2515_OK)
		{
			return MCP2515_FAIL;
		}

		// Turn wake interrupt back off if it was originally off
		if (!wakeIntEnabled)
		{
			MCP2515_BitModify(MCP2515_CANINTE, MCP_WAKIF, 0);
		}
	}

	// Clear wake flag
	MCP2515_BitModify(MCP2515_CANINTE, MCP_WAKIF, 0);

	return mcp2515_requestNewMode(newmode);
}

/*********************************************************************************************************
 ** Function name:           mcp2515_requestNewMode
 ** Descriptions:            Set control mode
 *********************************************************************************************************/
uint8_t mcp2515_requestNewMode(const uint8_t newmode) // Set mode
{
	unsigned long startTime = HAL_GetTick();

	// Spam new mode request and wait for the operation  to complete
	while (1)
	{
		// Request new mode
		// This is inside the loop as sometimes requesting the new mode once doesn't work (usually when attempting to sleep)
		MCP2515_BitModify(MCP_CANCTRL, MODE_MASK, newmode);

		uint8_t statReg = MCP2515_ReadByte(MCP_CANSTAT);
		if ((statReg & MODE_MASK) == newmode)
		{ // We're now in the new mode
			return MCP2515_OK;
		}

		if ((HAL_GetTick() - startTime) > 200)
		{ // Wait no more than 200ms for the operation to complete
			return MCP2515_FAIL;
		}
	}
}

/*********************************************************************************************************
 ** Function name:           mcp2515_configRate
 ** Descriptions:            set baudrate
 *********************************************************************************************************/
uint8_t mcp2515_configRate(const uint8_t canSpeed, const uint8_t clock) // set baudrate
{
	uint8_t set, cfg1, cfg2, cfg3;
	set = 1;
	switch (clock)
	{
	case (MCP_16MHz):
		switch (canSpeed)
		{
		case (CAN_5KBPS):
			cfg1 = MCP_16MHz_5kBPS_CFG1;
			cfg2 = MCP_16MHz_5kBPS_CFG2;
			cfg3 = MCP_16MHz_5kBPS_CFG3;
			break;

		case (CAN_10KBPS):
			cfg1 = MCP_16MHz_10kBPS_CFG1;
			cfg2 = MCP_16MHz_10kBPS_CFG2;
			cfg3 = MCP_16MHz_10kBPS_CFG3;
			break;

		case (CAN_20KBPS):
			cfg1 = MCP_16MHz_20kBPS_CFG1;
			cfg2 = MCP_16MHz_20kBPS_CFG2;
			cfg3 = MCP_16MHz_20kBPS_CFG3;
			break;

		case (CAN_25KBPS):
			cfg1 = MCP_16MHz_25kBPS_CFG1;
			cfg2 = MCP_16MHz_25kBPS_CFG2;
			cfg3 = MCP_16MHz_25kBPS_CFG3;
			break;

		case (CAN_31K25BPS):
			cfg1 = MCP_16MHz_31k25BPS_CFG1;
			cfg2 = MCP_16MHz_31k25BPS_CFG2;
			cfg3 = MCP_16MHz_31k25BPS_CFG3;
			break;

		case (CAN_33KBPS):
			cfg1 = MCP_16MHz_33kBPS_CFG1;
			cfg2 = MCP_16MHz_33kBPS_CFG2;
			cfg3 = MCP_16MHz_33kBPS_CFG3;
			break;

		case (CAN_40KBPS):
			cfg1 = MCP_16MHz_40kBPS_CFG1;
			cfg2 = MCP_16MHz_40kBPS_CFG2;
			cfg3 = MCP_16MHz_40kBPS_CFG3;
			break;

		case (CAN_50KBPS):
			cfg1 = MCP_16MHz_50kBPS_CFG1;
			cfg2 = MCP_16MHz_50kBPS_CFG2;
			cfg3 = MCP_16MHz_50kBPS_CFG3;
			break;

		case (CAN_80KBPS):
			cfg1 = MCP_16MHz_80kBPS_CFG1;
			cfg2 = MCP_16MHz_80kBPS_CFG2;
			cfg3 = MCP_16MHz_80kBPS_CFG3;
			break;

		case (CAN_83K3BPS):
			cfg1 = MCP_16MHz_83k3BPS_CFG1;
			cfg2 = MCP_16MHz_83k3BPS_CFG2;
			cfg3 = MCP_16MHz_83k3BPS_CFG3;
			break;

		case (CAN_95KBPS):
			cfg1 = MCP_16MHz_95kBPS_CFG1;
			cfg2 = MCP_16MHz_95kBPS_CFG2;
			cfg3 = MCP_16MHz_95kBPS_CFG3;
			break;

		case (CAN_100KBPS):
			cfg1 = MCP_16MHz_100kBPS_CFG1;
			cfg2 = MCP_16MHz_100kBPS_CFG2;
			cfg3 = MCP_16MHz_100kBPS_CFG3;
			break;

		case (CAN_125KBPS):
			cfg1 = MCP_16MHz_125kBPS_CFG1;
			cfg2 = MCP_16MHz_125kBPS_CFG2;
			cfg3 = MCP_16MHz_125kBPS_CFG3;
			break;

		case (CAN_200KBPS):
			cfg1 = MCP_16MHz_200kBPS_CFG1;
			cfg2 = MCP_16MHz_200kBPS_CFG2;
			cfg3 = MCP_16MHz_200kBPS_CFG3;
			break;

		case (CAN_250KBPS):
			cfg1 = MCP_16MHz_250kBPS_CFG1;
			cfg2 = MCP_16MHz_250kBPS_CFG2;
			cfg3 = MCP_16MHz_250kBPS_CFG3;
			break;

		case (CAN_500KBPS):
			cfg1 = MCP_16MHz_500kBPS_CFG1;
			cfg2 = MCP_16MHz_500kBPS_CFG2;
			cfg3 = MCP_16MHz_500kBPS_CFG3;
			break;

		case (CAN_666KBPS):
			cfg1 = MCP_16MHz_666kBPS_CFG1;
			cfg2 = MCP_16MHz_666kBPS_CFG2;
			cfg3 = MCP_16MHz_666kBPS_CFG3;
			break;

		case (CAN_800KBPS):
			cfg1 = MCP_16MHz_800kBPS_CFG1;
			cfg2 = MCP_16MHz_800kBPS_CFG2;
			cfg3 = MCP_16MHz_800kBPS_CFG3;
			break;

		case (CAN_1000KBPS):
			cfg1 = MCP_16MHz_1000kBPS_CFG1;
			cfg2 = MCP_16MHz_1000kBPS_CFG2;
			cfg3 = MCP_16MHz_1000kBPS_CFG3;
			break;

		default:
			set = 0;
			break;
		}
		break;
	case (MCP_12MHz):
		switch (canSpeed)
		{
		case (CAN_20KBPS):
			cfg1 = MCP_12MHz_20kBPS_CFG1;
			cfg2 = MCP_12MHz_20kBPS_CFG2;
			cfg3 = MCP_12MHz_20kBPS_CFG3;
			break;

		case (CAN_25KBPS):
			cfg1 = MCP_12MHz_25kBPS_CFG1;
			cfg2 = MCP_12MHz_25kBPS_CFG2;
			cfg3 = MCP_12MHz_25kBPS_CFG3;
			break;

		case (CAN_31K25BPS):
			cfg1 = MCP_12MHz_31k25BPS_CFG1;
			cfg2 = MCP_12MHz_31k25BPS_CFG2;
			cfg3 = MCP_12MHz_31k25BPS_CFG3;
			break;

		case (CAN_33KBPS):
			cfg1 = MCP_12MHz_33kBPS_CFG1;
			cfg2 = MCP_12MHz_33kBPS_CFG2;
			cfg3 = MCP_12MHz_33kBPS_CFG3;
			break;

		case (CAN_40KBPS):
			cfg1 = MCP_12MHz_40kBPS_CFG1;
			cfg2 = MCP_12MHz_40kBPS_CFG2;
			cfg3 = MCP_12MHz_40kBPS_CFG3;
			break;

		case (CAN_50KBPS):
			cfg1 = MCP_12MHz_50kBPS_CFG1;
			cfg2 = MCP_12MHz_50kBPS_CFG2;
			cfg3 = MCP_12MHz_50kBPS_CFG3;
			break;

		case (CAN_80KBPS):
			cfg1 = MCP_12MHz_80kBPS_CFG1;
			cfg2 = MCP_12MHz_80kBPS_CFG2;
			cfg3 = MCP_12MHz_80kBPS_CFG3;
			break;

		case (CAN_83K3BPS):
			cfg1 = MCP_12MHz_83k3BPS_CFG1;
			cfg2 = MCP_12MHz_83k3BPS_CFG2;
			cfg3 = MCP_12MHz_83k3BPS_CFG3;
			break;

		case (CAN_95KBPS):
			cfg1 = MCP_12MHz_95kBPS_CFG1;
			cfg2 = MCP_12MHz_95kBPS_CFG2;
			cfg3 = MCP_12MHz_95kBPS_CFG3;
			break;

		case (CAN_100KBPS):
			cfg1 = MCP_12MHz_100kBPS_CFG1;
			cfg2 = MCP_12MHz_100kBPS_CFG2;
			cfg3 = MCP_12MHz_100kBPS_CFG3;
			break;

		case (CAN_125KBPS):
			cfg1 = MCP_12MHz_125kBPS_CFG1;
			cfg2 = MCP_12MHz_125kBPS_CFG2;
			cfg3 = MCP_12MHz_125kBPS_CFG3;
			break;

		case (CAN_200KBPS):
			cfg1 = MCP_12MHz_200kBPS_CFG1;
			cfg2 = MCP_12MHz_200kBPS_CFG2;
			cfg3 = MCP_12MHz_200kBPS_CFG3;
			break;

		case (CAN_250KBPS):
			cfg1 = MCP_12MHz_250kBPS_CFG1;
			cfg2 = MCP_12MHz_250kBPS_CFG2;
			cfg3 = MCP_12MHz_250kBPS_CFG3;
			break;

		case (CAN_500KBPS):
			cfg1 = MCP_12MHz_500kBPS_CFG1;
			cfg2 = MCP_12MHz_500kBPS_CFG2;
			cfg3 = MCP_12MHz_500kBPS_CFG3;
			break;

		case (CAN_666KBPS):
			cfg1 = MCP_12MHz_666kBPS_CFG1;
			cfg2 = MCP_12MHz_666kBPS_CFG2;
			cfg3 = MCP_12MHz_666kBPS_CFG3;
			break;

		case (CAN_1000KBPS):
			cfg1 = MCP_12MHz_1000kBPS_CFG1;
			cfg2 = MCP_12MHz_1000kBPS_CFG2;
			cfg3 = MCP_12MHz_1000kBPS_CFG3;
			break;

		default:
			set = 0;
			break;
		}
		break;
	case (MCP_8MHz):
		switch (canSpeed)
		{
		case (CAN_5KBPS):
			cfg1 = MCP_8MHz_5kBPS_CFG1;
			cfg2 = MCP_8MHz_5kBPS_CFG2;
			cfg3 = MCP_8MHz_5kBPS_CFG3;
			break;

		case (CAN_10KBPS):
			cfg1 = MCP_8MHz_10kBPS_CFG1;
			cfg2 = MCP_8MHz_10kBPS_CFG2;
			cfg3 = MCP_8MHz_10kBPS_CFG3;
			break;

		case (CAN_20KBPS):
			cfg1 = MCP_8MHz_20kBPS_CFG1;
			cfg2 = MCP_8MHz_20kBPS_CFG2;
			cfg3 = MCP_8MHz_20kBPS_CFG3;
			break;

		case (CAN_31K25BPS):
			cfg1 = MCP_8MHz_31k25BPS_CFG1;
			cfg2 = MCP_8MHz_31k25BPS_CFG2;
			cfg3 = MCP_8MHz_31k25BPS_CFG3;
			break;

		case (CAN_40KBPS):
			cfg1 = MCP_8MHz_40kBPS_CFG1;
			cfg2 = MCP_8MHz_40kBPS_CFG2;
			cfg3 = MCP_8MHz_40kBPS_CFG3;
			break;

		case (CAN_50KBPS):
			cfg1 = MCP_8MHz_50kBPS_CFG1;
			cfg2 = MCP_8MHz_50kBPS_CFG2;
			cfg3 = MCP_8MHz_50kBPS_CFG3;
			break;

		case (CAN_80KBPS):
			cfg1 = MCP_8MHz_80kBPS_CFG1;
			cfg2 = MCP_8MHz_80kBPS_CFG2;
			cfg3 = MCP_8MHz_80kBPS_CFG3;
			break;

		case (CAN_95K2BPS):
			cfg1 = MCP_8MHz_95k2BPS_CFG1;
			cfg2 = MCP_8MHz_95k2BPS_CFG2;
			cfg3 = MCP_8MHz_95k2BPS_CFG3;
			break;

		case (CAN_100KBPS):
			cfg1 = MCP_8MHz_100kBPS_CFG1;
			cfg2 = MCP_8MHz_100kBPS_CFG2;
			cfg3 = MCP_8MHz_100kBPS_CFG3;
			break;

		case (CAN_125KBPS):
			cfg1 = MCP_8MHz_125kBPS_CFG1;
			cfg2 = MCP_8MHz_125kBPS_CFG2;
			cfg3 = MCP_8MHz_125kBPS_CFG3;
			break;

		case (CAN_200KBPS):
			cfg1 = MCP_8MHz_200kBPS_CFG1;
			cfg2 = MCP_8MHz_200kBPS_CFG2;
			cfg3 = MCP_8MHz_200kBPS_CFG3;
			break;

		case (CAN_250KBPS):
			cfg1 = MCP_8MHz_250kBPS_CFG1;
			cfg2 = MCP_8MHz_250kBPS_CFG2;
			cfg3 = MCP_8MHz_250kBPS_CFG3;
			break;

		case (CAN_500KBPS):
			cfg1 = MCP_8MHz_500kBPS_CFG1;
			cfg2 = MCP_8MHz_500kBPS_CFG2;
			cfg3 = MCP_8MHz_500kBPS_CFG3;
			break;

		case (CAN_800KBPS):
			cfg1 = MCP_8MHz_800kBPS_CFG1;
			cfg2 = MCP_8MHz_800kBPS_CFG2;
			cfg3 = MCP_8MHz_800kBPS_CFG3;
			break;

		case (CAN_1000KBPS):
			cfg1 = MCP_8MHz_1000kBPS_CFG1;
			cfg2 = MCP_8MHz_1000kBPS_CFG2;
			cfg3 = MCP_8MHz_1000kBPS_CFG3;
			break;

		default:
			set = 0;
			break;
		}
		break;
	default:
		set = 0;
		break;
	}

	if (set)
	{
		MCP2515_WriteByte(MCP_CNF1, cfg1);
		MCP2515_WriteByte(MCP_CNF2, cfg2);
		MCP2515_WriteByte(MCP_CNF3, cfg3);
		return MCP2515_OK;
	}
	else
	{
		return MCP2515_FAIL;
	}
}

/*********************************************************************************************************
 ** Function name:           mcp2515_initCANBuffers
 ** Descriptions:            init canbuffers
 *********************************************************************************************************/
void mcp2515_initCANBuffers(void)
{
	uint8_t i, a1, a2, a3;

	a1 = MCP2515_TXB0CTRL;
	a2 = MCP2515_TXB1CTRL;
	a3 = MCP2515_TXB2CTRL;
	for (i = 0; i < 14; i++)
	{                       // in-buffer loop
		MCP2515_WriteByte(a1, 0);
		MCP2515_WriteByte(a2, 0);
		MCP2515_WriteByte(a3, 0);
		a1++;
		a2++;
		a3++;
	}
	MCP2515_WriteByte(MCP2515_RXB0CTRL, 0);
	MCP2515_WriteByte(MCP2515_RXB1CTRL, 0);
}

/*********************************************************************************************************
 ** Function name:           setSleepWakeup
 ** Descriptions:            Enable or disable the wake up interrupt (If disabled the MCP2515 will not be woken up by CAN bus activity)
 *********************************************************************************************************/
void mcp2515_setSleepWakeup(uint8_t enable)
{
	MCP2515_BitModify(MCP2515_CANINTE, MCP_WAKIF, enable ? MCP_WAKIF : 0);
}

/*********************************************************************************************************
 ** Function name:           sleep
 ** Descriptions:            Put mcp2515 in sleep mode to save power
 *********************************************************************************************************/
uint8_t mcp2515_sleep(void)
{
	if (mcp2515_getMode() != MODE_SLEEP)
	{
		return mcp2515_setCANCTRL_Mode(MODE_SLEEP);
	}
	else
	{
		return CAN_OK;
	}
}

/*********************************************************************************************************
 ** Function name:           wake
 ** Descriptions:            wake MCP2515 manually from sleep. It will come back in the mode it was before sleeping.
 *********************************************************************************************************/
uint8_t mcp2515_wake()
{
	uint8_t currMode = mcp2515_getMode();
	if (currMode != mcpMode)
	{
		return mcp2515_setCANCTRL_Mode(mcpMode);
	}
	else
	{
		return CAN_OK;
	}
}


/*********************************************************************************************************
 ** Function name:           setMode
 ** Descriptions:            Sets control mode
 *********************************************************************************************************/
uint8_t mcp2515_setMode(const uint8_t opMode)
{
	if (opMode !=
	MODE_SLEEP)
	{ // if going to sleep, the value stored in opMode is not changed so that we can return to it later
		mcpMode = opMode;
	}
	return mcp2515_setCANCTRL_Mode(opMode);
}

/*********************************************************************************************************
 ** Function name:           getMode
 ** Descriptions:            Returns current control mode
 *********************************************************************************************************/
uint8_t mcp2515_getMode(void)
{
	return MCP2515_ReadByte(MCP2515_CANSTAT) & MODE_MASK;
}


/*********************************************************************************************************
 ** Function name:           mcp2515_reset
 ** Descriptions:            reset the device
 *********************************************************************************************************/
void MCP2515_Reset(void)
{
  MCP2515_SELECT;

  SPI_Tx(MCP2515_RESET);

  MCP2515_UNSELECT;

	HAL_Delay(10);
}

/*********************************************************************************************************
 ** Function name:           mcp2515_readRegister
 ** Descriptions:            read register
 *********************************************************************************************************/
uint8_t MCP2515_ReadByte (uint8_t address)
{
  uint8_t retVal;

  MCP2515_SELECT;

  SPI_Tx(MCP2515_READ);
  SPI_Tx(address);
  retVal = SPI_Rx();

  MCP2515_UNSELECT;

  return retVal;
}

/*********************************************************************************************************
 ** Function name:           mcp2515_readRegisterS
 ** Descriptions:            read registerS
 *********************************************************************************************************/
void MCP2515_ReadRxSequence(uint8_t instruction, uint8_t *data, uint8_t length)
{
	uint8_t tbufdata[4];
	uint8_t i, id, ext, pMsgSize, len, rtrBit;
	MCP2515_SELECT;

//	SPI_Tx(MCP2515_READ);
	SPI_RxBuffer(&instruction, 1);
//	SPI_RxBuffer(data,
//			length <= CAN_MAX_CHAR_IN_MESSAGE ?
//					length : CAN_MAX_CHAR_IN_MESSAGE);
	// mcp2515 has auto-increment of address-pointer
	for (i = 0; i < 4; i++)
	{
		data[i] = tbufdata[i] = SPI_Rx();

	}
	data[4] = pMsgSize = SPI_Rx();
	len = pMsgSize & MCP_DLC_MASK;
	for (i = 0; i < len && i < CAN_MAX_CHAR_IN_MESSAGE; i++)
	{
		data[5 + i] = SPI_Rx();
	}

	MCP2515_UNSELECT;

	rtrBit = (pMsgSize & MCP_RTR_MASK) ? 1 : 0;
	id = (tbufdata[MCP_SIDH] << 3) + (tbufdata[MCP_SIDL] >> 5);
	ext = 0;
	if ((tbufdata[MCP_SIDL] & MCP_TXB_EXIDE_M) == MCP_TXB_EXIDE_M)
	{
		/* extended id                  */
		id = (id << 2) + (tbufdata[MCP_SIDL] & 0x03);
		id = (id << 8) + tbufdata[MCP_EID8];
		id = (id << 8) + tbufdata[MCP_EID0];
		ext = 1;
	}


}

/*********************************************************************************************************
 ** Function name:           mcp2515_setRegister
 ** Descriptions:            set register
 *********************************************************************************************************/
void MCP2515_WriteByte(uint8_t address, uint8_t data)
{    
  MCP2515_SELECT;
  
  SPI_Tx(MCP2515_WRITE);
  SPI_Tx(address);
  SPI_Tx(data);  
    
  MCP2515_UNSELECT;
}

/*********************************************************************************************************
 ** Function name:           mcp2515_setRegisterS
 ** Descriptions:            set registerS
 *********************************************************************************************************/
void MCP2515_WriteByteSequence(uint8_t startAddress, uint8_t endAddress, uint8_t *data)
{    
  MCP2515_SELECT;
  
  SPI_Tx(MCP2515_WRITE);
  SPI_Tx(startAddress);
  SPI_TxBuffer(data, (endAddress - startAddress + 1));
  
  MCP2515_UNSELECT;
}

/* write to TxBuffer */
void MCP2515_LoadTxSequence(uint8_t instruction, uint8_t *idReg, uint8_t dlc, uint8_t *data)
{    
  MCP2515_SELECT;
  
  SPI_Tx(instruction);
  SPI_TxBuffer(idReg, 4);
  SPI_Tx(dlc);
  SPI_TxBuffer(data, dlc);
       
  MCP2515_UNSELECT;
}

/* write to TxBuffer(1 byte) */
void MCP2515_LoadTxBuffer(uint8_t instruction, uint8_t data)
{
  MCP2515_CS_LOW();
  
  SPI_Tx(instruction);
  SPI_Tx(data);
        
  MCP2515_CS_HIGH();
}

/* request to send */
void MCP2515_RequestToSend(uint8_t instruction)
{
  MCP2515_CS_LOW();
  
  SPI_Tx(instruction);
      
  MCP2515_CS_HIGH();
}

/*********************************************************************************************************
 ** Function name:           mcp2515_readStatus
 ** Descriptions:            read mcp2515's Status
 *********************************************************************************************************/
uint8_t MCP2515_ReadStatus(void)
{
  uint8_t retVal;
  
  MCP2515_SELECT;
  
  SPI_Tx(MCP2515_READ_STATUS);
  retVal = SPI_Rx();
        
  MCP2515_UNSELECT;
  
  return retVal;
}

/*********************************************************************************************************
 ** Function name:           readRxTxStatus
 ** Descriptions:            Read RX and TX interrupt bits. Function uses status reading, but translates.
 **                          result to MCP_CANINTF. With this you can check status e.g. on interrupt sr
 **                          with one single call to save SPI calls. Then use checkClearRxStatus and
 **                          checkClearTxStatus for testing.
 *********************************************************************************************************/
uint8_t mcp2515_readRxTxStatus(void)
{
	uint8_t ret = (MCP2515_ReadStatus()
			& (MCP_STAT_TXIF_MASK | MCP_STAT_RXIF_MASK));
	ret = (ret & MCP_STAT_TX0IF ? MCP_TX0IF : 0) |
	(ret & MCP_STAT_TX1IF ? MCP_TX1IF : 0) |
	(ret & MCP_STAT_TX2IF ? MCP_TX2IF : 0) |
	(ret & MCP_STAT_RXIF_MASK); // Rx bits happend to be same on status and MCP_CANINTF
	return ret;
}

/*********************************************************************************************************
 ** Function name:           checkClearRxStatus
 ** Descriptions:            Return first found rx CANINTF status and clears it from parameter.
 **                          Note that this does not affect to chip CANINTF at all. You can use this
 **                          with one single readRxTxStatus call.
 *********************************************************************************************************/
uint8_t mcp2515_checkClearRxStatus(uint8_t *status)
{
	uint8_t ret;

	ret = *status & MCP_RX0IF; *status &= ~MCP_RX0IF;

	if (ret == 0)
	{
		ret = *status & MCP_RX1IF;
		*status &= ~MCP_RX1IF;
	}

	return ret;
}

/*********************************************************************************************************
 ** Function name:           checkClearTxStatus
 ** Descriptions:            Return specified buffer of first found tx CANINTF status and clears it from parameter.
 **                          Note that this does not affect to chip CANINTF at all. You can use this
 **                          with one single readRxTxStatus call.
 *********************************************************************************************************/
uint8_t mcp2515_checkClearTxStatus(uint8_t *status, uint8_t iTxBuf)
{
	uint8_t ret;

	if (iTxBuf < MCP_N_TXBUFFERS)
	{ // Clear specific buffer flag
		ret = *status & txIfFlag(iTxBuf); *status &= ~txIfFlag(iTxBuf);
	}
	else
	{
		ret = 0;
		for (uint8_t i = 0; i < MCP_N_TXBUFFERS - nReservedTx; i++)
		{
			ret = *status & txIfFlag(i);
			if (ret != 0)
			{
				*status &= ~txIfFlag(i);
				return ret;
			}
		};
	}

	return ret;
}

/*********************************************************************************************************
 ** Function name:           clearBufferTransmitIfFlags
 ** Descriptions:            Clear transmit interrupt flags for specific buffer or for all unreserved buffers.
 **                          If interrupt will be used, it is important to clear all flags, when there is no
 **                          more data to be sent. Otherwise IRQ will newer change state.
 *********************************************************************************************************/
void mcp2515_clearBufferTransmitIfFlags(uint8_t flags)
{
	flags &= MCP_TX_INT;
	if (flags == 0)
	{
		return;
	}
	MCP2515_BitModify(MCP_CANINTF, flags, 0);
}

/*********************************************************************************************************
 ** Function name:           checkReceive
 ** Descriptions:            check if got something
 *********************************************************************************************************/
uint8_t mcp2515_checkReceive(void)
{
	uint8_t res;
	res = MCP2515_ReadStatus();                          // RXnIF in Bit 1 and 0
	return ((res & MCP_STAT_RXIF_MASK) ? CAN_MSGAVAIL : CAN_NOMSG);
}

/*********************************************************************************************************
 ** Function name:           checkError
 ** Descriptions:            if something error
 *********************************************************************************************************/
uint8_t mcp2515_checkError(uint8_t *err_ptr)
{
	uint8_t eflg = MCP2515_ReadByte(MCP_EFLG);
	MCP2515_BitModify(MCP_EFLG, 0xFF, 0);
	if (err_ptr)
	{
		*err_ptr = eflg;
	}
	return ((eflg & MCP_EFLG_ERRORMASK) ? CAN_CTRLERROR : CAN_OK);
}

void mcp2515_reserveTxBuffers(uint8_t nTxBuf)
{
	nReservedTx = (nTxBuf < MCP_N_TXBUFFERS ? nTxBuf : MCP_N_TXBUFFERS - 1);
}
/*********************************************************************************************************
 ** Function name:           mcp2515_readStatus
 ** Descriptions:            read mcp2515's Status
 *********************************************************************************************************/
uint8_t MCP2515_GetRxStatus(void)
{
  uint8_t retVal;
  
  MCP2515_SELECT;
  
  SPI_Tx(MCP2515_RX_STATUS);
  retVal = SPI_Rx();
        
  MCP2515_UNSELECT;
  
  return retVal;
}

/*********************************************************************************************************
 ** Function name:           mcp2515_modifyRegister
 ** Descriptions:            set bit of one register
 *********************************************************************************************************/
void MCP2515_BitModify(uint8_t address, uint8_t mask, uint8_t data)
{    
  MCP2515_SELECT;
  
  SPI_Tx(MCP2515_BIT_MOD);
  SPI_Tx(address);
  SPI_Tx(mask);
  SPI_Tx(data);
        
  MCP2515_UNSELECT;
}

/* SPI Tx wrapper function  */
static void SPI_Tx(uint8_t data)
{
  HAL_SPI_Transmit(SPI_CAN, &data, 1, SPI_TIMEOUT);    
}

/* SPI Tx wrapper function */
static void SPI_TxBuffer(uint8_t *buffer, uint8_t length)
{
  HAL_SPI_Transmit(SPI_CAN, buffer, length, SPI_TIMEOUT);    
}

/* SPI Rx wrapper function */
static uint8_t SPI_Rx(void)
{
  uint8_t retVal;
  HAL_SPI_Receive(SPI_CAN, &retVal, 1, SPI_TIMEOUT);
  return retVal;
}

/* SPI Rx wrapper function */
static void SPI_RxBuffer(uint8_t *buffer, uint8_t length)
{
  HAL_SPI_Receive(SPI_CAN, buffer, length, SPI_TIMEOUT);
}

/*********************************************************************************************************
 ** Function name:           mcp2515_start_transmit
 ** Descriptions:            Start message transmit on mcp2515
 *********************************************************************************************************/
void mcp2515_start_transmit(const uint8_t mcp_addr)
{            // start transmit

	MCP2515_SELECT;
	SPI_Tx(txSidhToRTS(mcp_addr));
	MCP2515_UNSELECT;

}

/*********************************************************************************************************
 ** Function name:           mcp2515_isTXBufFree
 ** Descriptions:            Test is tx buffer free for transmitting
 *********************************************************************************************************/
uint8_t mcp2515_isTXBufFree(uint8_t *txbuf_n, uint8_t iBuf)
{ /* get Next free txbuf          */
	*txbuf_n = 0x00;

	if (iBuf >= MCP_N_TXBUFFERS
			|| (MCP2515_ReadStatus() & txStatusPendingFlag(iBuf)) != 0)
	{
		return MCP_ALLTXBUSY;
	}

	*txbuf_n = txCtrlReg(iBuf) + 1; /* return SIDH-address of Buffer */
	MCP2515_BitModify(MCP_CANINTF, txIfFlag(iBuf), 0);

	return MCP2515_OK;
}

/*********************************************************************************************************
 ** Function name:           mcp2515_getNextFreeTXBuf
 ** Descriptions:            finds next free tx buffer for sending. Return MCP_ALLTXBUSY, if there is none.
 *********************************************************************************************************/
uint8_t mcp2515_getNextFreeTXBuf(uint8_t *txbuf_n)
{               // get Next free txbuf
	uint8_t status = MCP2515_ReadStatus() & MCP_STAT_TX_PENDING_MASK;
	uint8_t i;

	*txbuf_n = 0x00;

	if (status == MCP_STAT_TX_PENDING_MASK)
	{
		return MCP_ALLTXBUSY;    // All buffers are pending
	}

	// check all 3 TX-Buffers except reserved
	for (i = 0; i < MCP_N_TXBUFFERS - nReservedTx; i++)
	{
		if ((status & txStatusPendingFlag(i)) == 0)
		{
			*txbuf_n = txCtrlReg(i) + 1;        // return SIDH-address of Buffer
			MCP2515_BitModify(MCP_CANINTF, txIfFlag(i), 0);
			return MCP2515_OK;                                // ! function exit
		}
	}

	return MCP_ALLTXBUSY;
}

/*********************************************************************************************************
 ** Function name:           mcp2515_write_canMsg
 ** Descriptions:            write msg
 **                          Note! There is no check for right address!
 *********************************************************************************************************/
void mcp2515_write_canMsg(const uint8_t buffer_sidh_addr, unsigned long id,
		uint8_t ext, uint8_t rtrBit, uint8_t len, uint8_t *buf)
{
	uint8_t load_addr = txSidhToTxLoad(buffer_sidh_addr);

	uint8_t tbufdata[4];
	uint8_t dlc = len | (rtrBit ? MCP_RTR_MASK : 0);
	uint8_t i;

	mcp2515_id_to_buf(ext, id, tbufdata);

#ifdef SPI_HAS_TRANSACTION
	SPI_BEGIN();
#endif
	MCP2515_SELECT;
	SPI_RxBuffer(&load_addr, 1);
	for (i = 0; i < 4; i++)
	{
		SPI_Tx(tbufdata[i]);
	}
	SPI_Tx(dlc);
	for (i = 0; i < len && i < CAN_MAX_CHAR_IN_MESSAGE; i++)
	{
		SPI_Tx(buf[i]);
	}

	MCP2515_UNSELECT;
#ifdef SPI_HAS_TRANSACTION
    SPI_END();
    #endif

	mcp2515_start_transmit(buffer_sidh_addr);

}

/*********************************************************************************************************
 ** Function name:           mcp2515_read_canMsg
 ** Descriptions:            read message
 *********************************************************************************************************/
void mcp2515_read_canMsg(const uint8_t buffer_load_addr,
		volatile unsigned long *id, volatile uint8_t *ext,
		volatile uint8_t *rtrBit, volatile uint8_t *len, volatile uint8_t *buf)
{ /* read can msg                 */
	uint8_t tbufdata[4];
	uint8_t i;

	MCP2515_SELECT;
	SPI_RxBuffer(&buffer_load_addr, 1);
	// mcp2515 has auto-increment of address-pointer
	for (i = 0; i < 4; i++)
	{
		tbufdata[i] = SPI_Rx();
	}

	*id = (tbufdata[MCP_SIDH] << 3) + (tbufdata[MCP_SIDL] >> 5);
	*ext = 0;
	if ((tbufdata[MCP_SIDL] & MCP_TXB_EXIDE_M) == MCP_TXB_EXIDE_M)
	{
		/* extended id                  */
		*id = (*id << 2) + (tbufdata[MCP_SIDL] & 0x03);
		*id = (*id << 8) + tbufdata[MCP_EID8];
		*id = (*id << 8) + tbufdata[MCP_EID0];
		*ext = 1;
	}

	uint8_t pMsgSize = SPI_Rx();
	*len = pMsgSize & MCP_DLC_MASK;
	*rtrBit = (pMsgSize & MCP_RTR_MASK) ? 1 : 0;
	for (i = 0; i < *len && i < CAN_MAX_CHAR_IN_MESSAGE; i++)
	{
		buf[i] = SPI_Rx();
	}

	MCP2515_UNSELECT;
}

/*********************************************************************************************************
 ** Function name:           mcp2515_id_to_buf
 ** Descriptions:            configure tbufdata[4] from id and ext
 *********************************************************************************************************/
void mcp2515_id_to_buf(const uint8_t ext, const unsigned long id,
		uint8_t *tbufdata)
{
	uint16_t canid;

	canid = (uint16_t) (id & 0x0FFFF);

	if (ext == 1)
	{
		tbufdata[MCP_EID0] = (uint8_t) (canid & 0xFF);
		tbufdata[MCP_EID8] = (uint8_t) (canid >> 8);
		canid = (uint16_t) (id >> 16);
		tbufdata[MCP_SIDL] = (uint8_t) (canid & 0x03);
		tbufdata[MCP_SIDL] += (uint8_t) ((canid & 0x1C) << 3);
		tbufdata[MCP_SIDL] |= MCP_TXB_EXIDE_M;
		tbufdata[MCP_SIDH] = (uint8_t) (canid >> 5);
	}
	else
	{
		tbufdata[MCP_SIDH] = (uint8_t) (canid >> 3);
		tbufdata[MCP_SIDL] = (uint8_t) ((canid & 0x07) << 5);
		tbufdata[MCP_EID0] = 0;
		tbufdata[MCP_EID8] = 0;
	}
}

/*********************************************************************************************************
 ** Function name:           enableTxInterrupt
 ** Descriptions:            enable interrupt for all tx buffers
 *********************************************************************************************************/
void mcp2515_enableTxInterrupt(bool enable)
{
	uint8_t interruptStatus = MCP2515_ReadByte(MCP_CANINTE);

	if (enable)
	{
		interruptStatus |= MCP_TX_INT;
	}
	else
	{
		interruptStatus &= ~MCP_TX_INT;
	}

	MCP2515_WriteByte(MCP_CANINTE, interruptStatus);
}

void mcp2515_enableRxInterrupt(bool enable)
{
	uint8_t interruptStatus = MCP2515_ReadByte(MCP_CANINTE);

	if (enable)
	{
		interruptStatus |= MCP_RX_INT;
	}
	else
	{
		interruptStatus &= ~MCP_RX_INT;
	}

	MCP2515_WriteByte(MCP_CANINTE, interruptStatus);
}
void mcp2515_enableErrInterrupt(bool enable)
{

}
void mcp2515_enableWkupInterrupt(bool enable)
{

}

/*********************************************************************************************************
 ** Function name:           mcp2515_write_id
 ** Descriptions:            write can id
 *********************************************************************************************************/
void mcp2515_write_id(const uint8_t mcp_addr, const uint8_t ext,
		const unsigned long id)
{
	uint8_t tbufdata[4];

	mcp2515_id_to_buf(ext, id, tbufdata);
	MCP2515_WriteByteSequence(mcp_addr, mcp_addr + 4, tbufdata);
}

/*********************************************************************************************************
 ** Function name:           mcp2515_read_id
 ** Descriptions:            read can id
 *********************************************************************************************************/
void mcp2515_read_id(const uint8_t mcp_addr, uint8_t *ext, unsigned long *id)
{
	uint8_t tbufdata[4];

	*ext = 0;
	*id = 0;

	MCP2515_ReadRxSequence(mcp_addr, tbufdata, 4);

	*id = (tbufdata[MCP_SIDH] << 3) + (tbufdata[MCP_SIDL] >> 5);

	if ((tbufdata[MCP_SIDL] & MCP_TXB_EXIDE_M) == MCP_TXB_EXIDE_M)
	{
		// extended id
		*id = (*id << 2) + (tbufdata[MCP_SIDL] & 0x03);
		*id = (*id << 8) + tbufdata[MCP_EID8];
		*id = (*id << 8) + tbufdata[MCP_EID0];
		*ext = 1;
	}
}

/*********************************************************************************************************
 ** Function name:           init_Mask
 ** Descriptions:            init canid Masks
 *********************************************************************************************************/
uint8_t mcp2515_init_Mask(uint8_t num, uint8_t ext, unsigned long ulData)
{
	uint8_t res = MCP2515_OK;

	res = mcp2515_setCANCTRL_Mode(MODE_CONFIG);
	if (res > 0)
	{
		return res;
	}

	if (num == 0)
	{
		mcp2515_write_id(MCP_RXM0SIDH, ext, ulData);

	}
	else if (num == 1)
	{
		mcp2515_write_id(MCP_RXM1SIDH, ext, ulData);
	}
	else
	{
		res = MCP2515_FAIL;
	}

	res = mcp2515_setCANCTRL_Mode(mcpMode);
	if (res > 0)
	{
		return res;
	}
	return res;
}

/*********************************************************************************************************
 ** Function name:           init_Filt
 ** Descriptions:            init canid filters
 *********************************************************************************************************/
uint8_t mcp2515_init_Filt(uint8_t num, uint8_t ext, unsigned long ulData)
{
	uint8_t res = MCP2515_OK;
	res = mcp2515_setCANCTRL_Mode(MODE_CONFIG);
	if (res > 0)
	{
		return res;
	}

	switch (num)
	{
	case 0:
		mcp2515_write_id(MCP_RXF0SIDH, ext, ulData);
		break;

	case 1:
		mcp2515_write_id(MCP_RXF1SIDH, ext, ulData);
		break;

	case 2:
		mcp2515_write_id(MCP_RXF2SIDH, ext, ulData);
		break;

	case 3:
		mcp2515_write_id(MCP_RXF3SIDH, ext, ulData);
		break;

	case 4:
		mcp2515_write_id(MCP_RXF4SIDH, ext, ulData);
		break;

	case 5:
		mcp2515_write_id(MCP_RXF5SIDH, ext, ulData);
		break;

	default:
		res = MCP2515_FAIL;
	}

	res = mcp2515_setCANCTRL_Mode(mcpMode);
	if (res > 0)
	{
		return res;
	}
	return res;
}

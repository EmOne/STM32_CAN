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

#ifndef __MCP2515_H
#define	__MCP2515_H

#include <stdbool.h>
#include <inttypes.h>
#include "CANSPI.h"
#include "mcp2515_can_dfs.h"

#define CAN_OK              (0)
#define CAN_FAILINIT        (1)
#define CAN_FAILTX          (2)
#define CAN_MSGAVAIL        (3)
#define CAN_NOMSG           (4)
#define CAN_CTRLERROR       (5)
#define CAN_GETTXBFTIMEOUT  (6)
#define CAN_SENDMSGTIMEOUT  (7)
#define CAN_FAIL            (0xff)

// clock
typedef enum
{
	MCP_NO_MHz,
	/* apply to MCP2515 */
	MCP_16MHz,
	MCP_12MHz,
	MCP_8MHz,
	/* apply to MCP2518FD */
	MCP2518FD_40MHz = MCP_16MHz /* To compatible MCP2515 shield */,
	MCP2518FD_20MHz,
	MCP2518FD_10MHz,
} MCP_CLOCK_T;

typedef enum
{
	CAN_NOBPS,
	CAN_5KBPS,
	CAN_10KBPS,
	CAN_20KBPS,
	CAN_25KBPS,
	CAN_31K25BPS,
	CAN_33KBPS,
	CAN_40KBPS,
	CAN_50KBPS,
	CAN_80KBPS,
	CAN_83K3BPS,
	CAN_95KBPS,
	CAN_95K2BPS,
	CAN_100KBPS,
	CAN_125KBPS,
	CAN_200KBPS,
	CAN_250KBPS,
	CAN_500KBPS,
	CAN_666KBPS,
	CAN_800KBPS,
	CAN_1000KBPS
} MCP_BITTIME_SETUP;


/* MCP2515 SPI Instruction Set */
#define MCP2515_RESET           MCP_RESET

#define MCP2515_READ            MCP_READ
#define MCP2515_READ_RXB0SIDH   0x90
#define MCP2515_READ_RXB0D0     0x92
#define MCP2515_READ_RXB1SIDH   0x94
#define MCP2515_READ_RXB1D0     0x96

#define MCP2515_WRITE           MCP_WRITE
#define MCP2515_LOAD_TXB0SIDH   0x40    /* TX0 ID location */
#define MCP2515_LOAD_TXB0D0     0x41    /* TX0 Data location */
#define MCP2515_LOAD_TXB1SIDH   0x42    /* TX1 ID location */
#define MCP2515_LOAD_TXB1D0     0x43    /* TX1 Data location */
#define MCP2515_LOAD_TXB2SIDH   0x44    /* TX2 ID location */
#define MCP2515_LOAD_TXB2D0     0x45    /* TX2 Data location */

#define MCP2515_RTS_TX0         0x81
#define MCP2515_RTS_TX1         0x82
#define MCP2515_RTS_TX2         0x84
#define MCP2515_RTS_ALL         0x87
#define MCP2515_READ_STATUS     0xA0
#define MCP2515_RX_STATUS       MCP_READ_STATUS
#define MCP2515_BIT_MOD         MCP_BITMOD

/* MCP25152515 Register Adresses */
#define MCP2515_RXF0SIDH	0x00
#define MCP2515_RXF0SIDL	0x01
#define MCP2515_RXF0EID8	0x02
#define MCP2515_RXF0EID0	0x03
#define MCP2515_RXF1SIDH	0x04
#define MCP2515_RXF1SIDL	0x05
#define MCP2515_RXF1EID8	0x06
#define MCP2515_RXF1EID0	0x07
#define MCP2515_RXF2SIDH	0x08
#define MCP2515_RXF2SIDL	0x09
#define MCP2515_RXF2EID8	0x0A
#define MCP2515_RXF2EID0	0x0B
#define MCP2515_CANSTAT		MCP_CANSTAT
#define MCP2515_CANCTRL		0x0F

#define MCP2515_RXF3SIDH	0x10
#define MCP2515_RXF3SIDL	0x11
#define MCP2515_RXF3EID8	0x12
#define MCP2515_RXF3EID0	0x13
#define MCP2515_RXF4SIDH	0x14
#define MCP2515_RXF4SIDL	0x15
#define MCP2515_RXF4EID8	0x16
#define MCP2515_RXF4EID0	0x17
#define MCP2515_RXF5SIDH	0x18
#define MCP2515_RXF5SIDL	0x19
#define MCP2515_RXF5EID8	0x1A
#define MCP2515_RXF5EID0	0x1B
#define MCP2515_TEC		0x1C
#define MCP2515_REC		0x1D

#define MCP2515_RXM0SIDH	MCP_RXM0SIDH
#define MCP2515_RXM0SIDL	MCP_RXM0SIDL
#define MCP2515_RXM0EID8	MCP_RXM0EID8
#define MCP2515_RXM0EID0	MCP_RXM0EID0
#define MCP2515_RXM1SIDH	MCP_RXM1SIDH
#define MCP2515_RXM1SIDL	MCP_RXM1SIDL
#define MCP2515_RXM1EID8	MCP_RXM1EID8
#define MCP2515_RXM1EID0	MCP_RXM1EID0
#define MCP2515_CNF3		MCP_CNF3
#define MCP2515_CNF2		MCP_CNF2
#define MCP2515_CNF1		MCP_CNF1
#define MCP2515_CANINTE		MCP_CANINTE
#define MCP2515_CANINTF		MCP_CANINTF
#define MCP2515_EFLG		MCP_EFLG

#define MCP2515_TXB0CTRL	MCP_TXB0CTRL
#define MCP2515_TXB1CTRL	MCP_TXB1CTRL
#define MCP2515_TXB2CTRL	MCP_TXB2CTRL
#define MCP2515_RXB0CTRL	MCP_RXB0CTRL
#define MCP2515_RXB0SIDH	MCP_RXB0SIDH
#define MCP2515_RXB1CTRL	MCP_RXB1CTRL
#define MCP2515_RXB1SIDH	MCP_RXB1SIDH

/* Defines for Rx Status */
#define MSG_IN_RXB0             0x01
#define MSG_IN_RXB1             0x02
#define MSG_IN_BOTH_BUFFERS     0x03

typedef union{
  struct{
    unsigned RX0IF      : 1;
    unsigned RX1IF      : 1;
    unsigned TXB0REQ    : 1;
    unsigned TX0IF      : 1;
    unsigned TXB1REQ    : 1;
    unsigned TX1IF      : 1;
    unsigned TXB2REQ    : 1;
    unsigned TX2IF      : 1;
  };
  uint8_t ctrl_status;  
}ctrl_status_t;

typedef union{
  struct{
    unsigned filter     : 3;
    unsigned msgType    : 2;
    unsigned unusedBit  : 1;
    unsigned rxBuffer   : 2;
  };
  uint8_t ctrl_rx_status;
}ctrl_rx_status_t;

typedef union{
  struct{
    unsigned EWARN      :1;
    unsigned RXWAR      :1;
    unsigned TXWAR      :1;
    unsigned RXEP       :1;
    unsigned TXEP       :1;
    unsigned TXBO       :1;
    unsigned RX0OVR     :1;
    unsigned RX1OVR     :1;  
  };
  uint8_t error_flag_reg;
}ctrl_error_status_t;

typedef union{
  struct{
    uint8_t RXBnSIDH;
    uint8_t RXBnSIDL;
    uint8_t RXBnEID8;
    uint8_t RXBnEID0;
    uint8_t RXBnDLC;
    uint8_t RXBnD0;
    uint8_t RXBnD1;
    uint8_t RXBnD2;
    uint8_t RXBnD3;
    uint8_t RXBnD4;
    uint8_t RXBnD5;
    uint8_t RXBnD6;
    uint8_t RXBnD7;
  };
  uint8_t rx_reg_array[13];
}rx_reg_t;

/* MCP2515 Registers */
typedef struct{
  uint8_t RXF0SIDH;
  uint8_t RXF0SIDL;
  uint8_t RXF0EID8;
  uint8_t RXF0EID0;
}RXF0;

typedef struct{
  uint8_t RXF1SIDH;
  uint8_t RXF1SIDL;
  uint8_t RXF1EID8;
  uint8_t RXF1EID0;
}RXF1;

typedef struct{
  uint8_t RXF2SIDH;
  uint8_t RXF2SIDL;
  uint8_t RXF2EID8;
  uint8_t RXF2EID0;
}RXF2;

typedef struct{
  uint8_t RXF3SIDH;
  uint8_t RXF3SIDL;
  uint8_t RXF3EID8;
  uint8_t RXF3EID0;
}RXF3;

typedef struct{
  uint8_t RXF4SIDH;
  uint8_t RXF4SIDL;
  uint8_t RXF4EID8;
  uint8_t RXF4EID0;
}RXF4;

typedef struct{
  uint8_t RXF5SIDH;
  uint8_t RXF5SIDL;
  uint8_t RXF5EID8;
  uint8_t RXF5EID0;
}RXF5;

typedef struct{
  uint8_t RXM0SIDH;
  uint8_t RXM0SIDL;
  uint8_t RXM0EID8;
  uint8_t RXM0EID0;
}RXM0;

typedef struct{
  uint8_t RXM1SIDH;
  uint8_t RXM1SIDL;
  uint8_t RXM1EID8;
  uint8_t RXM1EID0;
}RXM1;

typedef struct{
  uint8_t tempSIDH;
  uint8_t tempSIDL;
  uint8_t tempEID8;
  uint8_t tempEID0;
}id_reg_t;

/* Functions */
bool MCP2515_Initialize(void);
bool MCP2515_SetConfigMode(void);
bool MCP2515_SetNormalMode(void);
bool MCP2515_SetSleepMode(void);
void MCP2515_Reset(void);
uint8_t MCP2515_ReadByte (uint8_t address);
void MCP2515_ReadRxSequence(uint8_t instruction, uint8_t *data, uint8_t length);
void MCP2515_WriteByte(uint8_t address, uint8_t data);
void MCP2515_WriteByteSequence(uint8_t startAddress, uint8_t endAddress, uint8_t *data);
void MCP2515_LoadTxSequence(uint8_t instruction, uint8_t *idReg, uint8_t dlc, uint8_t *data);
void MCP2515_LoadTxBuffer(uint8_t instruction, uint8_t data);
void MCP2515_RequestToSend(uint8_t instruction);
uint8_t MCP2515_ReadStatus(void);
uint8_t MCP2515_GetRxStatus(void);
void MCP2515_BitModify(uint8_t address, uint8_t mask, uint8_t data);

uint8_t mcp2515_setCANCTRL_Mode(const uint8_t newmode);              // set mode
uint8_t mcp2515_requestNewMode(const uint8_t newmode);               // Set mode
uint8_t mcp2515_configRate(const uint8_t canSpeed, const uint8_t clock); // set baudrate
void mcp2515_setSleepWakeup(uint8_t enable);
uint8_t mcp2515_sleep(void);
uint8_t mcp2515_setMode(const uint8_t opMode);
uint8_t mcp2515_getMode(void);
void mcp2515_initCANBuffers(void);
void mcp2515_reserveTxBuffers(uint8_t nTxBuf);
uint8_t mcp2515_checkReceive(void);
void mcp2515_write_canMsg(const uint8_t buffer_sidh_addr, unsigned long id,
		uint8_t ext, uint8_t rtrBit, uint8_t len, uint8_t *buf);
void mcp2515_id_to_buf(const uint8_t ext, const unsigned long id,
		uint8_t *tbufdata);
void mcp2515_read_canMsg(const uint8_t buffer_load_addr,
		volatile unsigned long *id, volatile uint8_t *ext,
		volatile uint8_t *rtrBit, volatile uint8_t *len, volatile uint8_t *buf);
extern uint8_t mcpMode;

#endif

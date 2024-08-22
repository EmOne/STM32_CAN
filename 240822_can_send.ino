//Arduino Uno R3 /w CAN-Shield for testing with STM32F767 CAN connectivity
//Type "manual" to manually input CAN ID and payload data (in HEX form)
//Type 'auto' to randomly send CAN ID and payload data.
//All data will be shown in Decimal form just like in .ino file that provided from K'Anol 

#include <SPI.h>
#include "mcp2515_can.h"  // Include the necessary CAN library

const int SPI_CS_PIN = 9;  // CS pin for MCP2515
const int CAN_INT_PIN = 2; // Interrupt pin for MCP2515

mcp2515_can CAN(SPI_CS_PIN); // Set CS pin

#define MAX_DATA_SIZE 8
uint32_t id;
uint8_t  type; // bit0: ext, bit1: rtr

byte cdata[MAX_DATA_SIZE] = {0};
unsigned char stmp[8] = {0, 0, 0, 0, 0, 0, 0, 0};

enum Mode { MANUAL, AUTO };
Mode currentMode = MANUAL; // Start in MANUAL mode

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  if (CAN.begin(CAN_125KBPS) != 0) { // Initialize CAN at 125kbps
    Serial.println("CAN initialization failed!");
    while (1);
  }

  Serial.println("CAN initialized successfully!");
  Serial.println("Enter 'manual' to switch to manual mode or 'auto' to switch to auto mode.");
}

void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');  // Read input string
    input.trim();  // Remove any leading/trailing whitespace

    if (input.equalsIgnoreCase("manual")) {
      currentMode = MANUAL;
      Serial.println("Switched to MANUAL mode.");
      Serial.println("Enter the CAN ID, followed by 8 payload data bytes (HEX)");
      Serial.println("(e.g., '100 AA 02 03 FF 05 06 07 DE'):");
    } else if (input.equalsIgnoreCase("auto")) {
      currentMode = AUTO;
      Serial.println("Switched to AUTO mode. Payload will be shown as DECIMAL");
    } else if (currentMode == MANUAL) {
      uint32_t canId = 0;
      uint8_t data[8] = {0};
      int dataLength = 0;

      // Parse the input string
      int parsedElements = sscanf(input.c_str(), "%lx %hhx %hhx %hhx %hhx %hhx %hhx %hhx %hhx", 
                                  &canId, 
                                  &data[0], &data[1], &data[2], &data[3], 
                                  &data[4], &data[5], &data[6], &data[7]);

      if (parsedElements < 2) {
        Serial.println("Invalid input format. Please try again.");
        return;
      }

      dataLength = parsedElements - 1;  // Number of data bytes entered

      // Send CAN message
      if (CAN.sendMsgBuf(canId, 0, dataLength, data) == MCP2515_FAIL) {
        Serial.println("Failed to send manual message!");
      } else {
        Serial.println();
        Serial.print("Sent CAN ID: 0x");
        Serial.print(canId, HEX);  // Print CAN ID in hexadecimal format
        Serial.print(" with data: ");
        for (int i = 0; i < dataLength; i++) {
          Serial.print(data[i], DEC);  // Print data in decimal format
          Serial.print(" ");
        }
        Serial.println();
      }

      // Check for incoming CAN messages
      receiveCANMessage();
    }
  }

  if (currentMode == AUTO) {
    // Randomly send CAN message
    randomSendCANMessage();
    delay(1000);  // Delay to avoid flooding the bus

    // Check for incoming CAN messages
    receiveCANMessage();
  }
}

void randomSendCANMessage() {
  static unsigned long lastSendTime = 0;
  unsigned long currentTime = millis();

  // Send a random CAN message every 5 seconds
  if (currentTime - lastSendTime >= 5000) {
    unsigned long randomCanId = random(0x000, 0x7FF); // Random CAN ID (11-bit)
    unsigned char randomBuf[8];
    unsigned char randomLen = 0;

    // Generate random data
    randomLen = random(1, 9); // Random length from 1 to 8 bytes
    for (int i = 0; i < randomLen; i++) {
      randomBuf[i] = random(0x00, 0xFF);
    }

    // Send the random CAN message
    if (CAN.sendMsgBuf(randomCanId, 0, randomLen, randomBuf) == MCP2515_FAIL) {
      Serial.println("Failed to send random message!");
    } else {
      Serial.print("Sent random CAN ID: 0x");
      Serial.print(randomCanId, HEX);
      Serial.print(" with data: ");
      for (int i = 0; i < randomLen; i++) {
        Serial.print(randomBuf[i], DEC);
        Serial.print(" ");
      }
      Serial.println();
    }

    lastSendTime = currentTime;
  }
}

void receiveCANMessage() {
  unsigned char len = 0;
  unsigned char buf[8];
  unsigned long canId = 0;

  // Check for incoming CAN message
    delay(1000);

#ifdef CAN_2518FD        
    if (CAN_MSGAVAIL == CAN_RECEIVE.checkReceive()) {
    // read data,  len: data length, buf: data buf
      SERIAL_PORT_MONITOR.println("checkReceive");
      CAN_RECEIVE.readMsgBuf(&len, buf);

       // print the data
      for (int i = 0; i < len; i++) {
        SERIAL_PORT_MONITOR.print(buf[i]); SERIAL_PORT_MONITOR.print(" ");
      }
#else 
    unsigned long t = millis();
    if (CAN_MSGAVAIL != CAN.checkReceive()) {
          SERIAL_PORT_MONITOR.println();
          SERIAL_PORT_MONITOR.println("else Loop nothig rcvd");
    }
    if (CAN_MSGAVAIL == CAN.checkReceive()) {
    // read data,  len: data length, buf: data buf
      SERIAL_PORT_MONITOR.println("checkReceive");
      CAN.readMsgBuf(&len, buf);

       // print the data
      for (int i = 0; i < len; i++) {
        SERIAL_PORT_MONITOR.print(buf[i]); SERIAL_PORT_MONITOR.print(" ");
      }
        return;
    } else {
    
    char prbuf[32 + MAX_DATA_SIZE * 3];
    int i, n;

    unsigned long t = millis();
    // read data, len: data length, buf: data buf
    CAN.readMsgBuf(&len, cdata);
  
    id = CAN.getCanId();
    type = (CAN.isExtendedFrame() << 0) |
          (CAN.isRemoteRequest() << 1);
    /*
    * MCP2515(or this driver) could not handle properly
    * the data carried by remote frame
    */
    n = sprintf(prbuf, "%04lu.%03d ", t / 1000, int(t % 1000));
    /* Displayed type:
    *
    * 0x00: standard data frame
    * 0x02: extended data frame
    * 0x30: standard remote frame
    * 0x32: extended remote frame
    */
    static const byte type2[] = {0x00, 0x02, 0x30, 0x32};
    n += sprintf(prbuf + n, "RX: [%08lX](%02X) ", (unsigned long)id, type2[type]);
    // n += sprintf(prbuf, "RX: [%08lX](%02X) ", id, type);

    for (int i = 0; i < len; i++) {
        n += sprintf(prbuf + n, "%02X ", buf[i]);
    }
    SERIAL_PORT_MONITOR.println();
    SERIAL_PORT_MONITOR.println(prbuf);
    CAN.sendMsgBuf(id, 0, len, cdata);
#endif
   
    SERIAL_PORT_MONITOR.println();
    SERIAL_PORT_MONITOR.println("---LOOP END---"); 
    SERIAL_PORT_MONITOR.println();
}
}

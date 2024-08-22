#include <SPI.h>
#include "mcp2515_can.h"  // Include the necessary CAN library

const int SPI_CS_PIN = 9;  // CS pin for MCP2515
const int CAN_INT_PIN = 2; // Interrupt pin for MCP2515

mcp2515_can CAN(SPI_CS_PIN); // Set CS pin

enum Mode { MANUAL, RANDOM };
Mode currentMode = MANUAL; // Start in MANUAL mode

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  if (CAN.begin(CAN_500KBPS) != 0) { // Initialize CAN at 500kbps
    Serial.println("CAN initialization failed!");
    while (1);
  }
  
  Serial.println("CAN initialized successfully!");
  Serial.println("Enter 'manual' to switch to manual mode or 'random' to switch to random mode.");
  Serial.println("In manual mode, enter CAN ID and data bytes (e.g., '100 01 02 03 04 05 06 07 08'):");
}

void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');  // Read input string
    input.trim();  // Remove any leading/trailing whitespace

    if (input.equalsIgnoreCase("manual")) {
      currentMode = MANUAL;
      Serial.println("Switched to MANUAL mode.");
      Serial.println("Enter CAN ID and data bytes (e.g., '100 01 02 03 04 05 06 07 08'):");
    } else if (input.equalsIgnoreCase("random")) {
      currentMode = RANDOM;
      Serial.println("Switched to RANDOM mode.");
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
        Serial.print("Sent CAN ID: 0x");
        Serial.print(canId, HEX);
        Serial.print(" with data: ");
        for (int i = 0; i < dataLength; i++) {
          Serial.print(data[i], HEX);
          Serial.print(" ");
        }
        Serial.println();
      }
    }
  }

  if (currentMode == RANDOM) {
    // Randomly send CAN message
    randomSendCANMessage();
    delay(1000);  // Delay to avoid flooding the bus
  }

  unsigned char len = 0;
  unsigned char buf[8];
  unsigned long canId = 0;

  // Check for incoming CAN message
  if (CAN_MSGAVAIL == CAN.checkReceive()) {
    CAN.readMsgBuf(&len, buf);  // Read data: len = data length, buf = data buffer
    canId = CAN.getCanId();     // Get the CAN ID of the received message

    // Print the received message
    Serial.print("Received CAN ID: 0x");
    Serial.print(canId, HEX);
    Serial.print(" with data: ");
    for (int i = 0; i < len; i++) {
      Serial.print(buf[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
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
    for (int i = 0; i < 8; i++) {
      randomBuf[i] = random(0x00, 0xFF);
      randomLen++;
    }

    // Send the random CAN message
    if (CAN.sendMsgBuf(randomCanId, 0, randomLen, randomBuf) == MCP2515_FAIL) {
      Serial.println("Failed to send random message!");
    } else {
      Serial.print("Sent random CAN ID: 0x");
      Serial.print(randomCanId, HEX);
      Serial.print(" with data: ");
      for (int i = 0; i < randomLen; i++) {
        Serial.print(randomBuf[i], HEX);
        Serial.print(" ");
      }
      Serial.println();
    }

    lastSendTime = currentTime;
  }
}

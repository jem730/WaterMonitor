// ---------------------------- Module Description ----------------------------
//
// Module Name: I2C Peripheral Handler
// Description: This module configures the ESP32 as an I2C slave device. It
//              allows the ESP32 to receive data from an I2C master device
//              and optionally respond with data when requested.
//
// ---------------------------- Pin Definitions -------------------------------
// SDA_PIN: GPIO19 - I2C data line
// SCL_PIN: GPIO20 - I2C clock line
//
// ----------------------------- I2C Parameters -------------------------------
// I2C_DEV_ADDR: 0x51 - The 7-bit I2C peripheral address for the ESP32
//
// ----------------------------- Buffer Details -------------------------------
// rxBuffer: A volatile buffer (size 32 bytes) that stores received data.
// dataLength: Tracks the number of bytes stored in the rxBuffer.
//
// ----------------------------- Functionality ---------------------------------
// 1. onReceive(int len):
//    - Triggered when data is received from the I2C master.
//    - Clears the rxBuffer, processes incoming bytes, and stores them.
//
// 2. onRequest():
//    - Triggered when the master requests data from the slave.
//    - Sends the contents of the rxBuffer back to the master.
//
// 3. setup():
//    - Initializes I2C as a peripheral with a specified address.
//    - Configures interrupt handlers for data receive and request events.
//
// 4. loop():
//    - Currently used for debugging or as a placeholder for further tasks.
//
// ----------------------------------------------------------------------------

#include <Wire.h>

#define SDA_PIN 19
#define SCL_PIN 20
#define I2C_DEV_ADDR 0x51

volatile uint8_t rxBuffer[32];  // Buffer for received data
volatile size_t dataLength = 0;

void onReceive(int len) {
    // Clear the buffer before receiving new data
    memset((void*)rxBuffer, 0, sizeof(rxBuffer));
    dataLength = 0;

    Serial.printf("Data received [%d]: ", len);
    while (Wire.available()) {
        size_t length = dataLength;  // Temporary non-volatile copy
        if (length < sizeof(rxBuffer)) {
            rxBuffer[length] = Wire.read();
            Serial.printf("%X ", rxBuffer[length]);  // Print in HEX
            dataLength = length + 1;  // Assign incremented value back
        }
    }
    Serial.println();
}

void onRequest() {
    // Send the received data back to the master
    Wire.write((const uint8_t*)rxBuffer, dataLength);
    Serial.println("Data sent to master.");

    // Clear the buffer after sending data
    memset((void*)rxBuffer, 0, sizeof(rxBuffer));
    dataLength = 0;
}

void setup() {
    Serial.begin(115200);

    if (!Wire.begin(I2C_DEV_ADDR, SDA_PIN, SCL_PIN, 100000)) {
        Serial.println("Failed to initialize I2C slave!");
        while (1);
    }
    Serial.println("I2C slave initialized successfully.");

    Wire.onReceive(onReceive);
    Wire.onRequest(onRequest);
}

void loop() {
    delay(500);
}


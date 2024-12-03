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


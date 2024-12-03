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

#include <Wire.h>               // I2C communication library
#include "HT_SSD1306Wire.h"     // OLED display library

#define SDA_PIN 19
#define SCL_PIN 20
#define I2C_DEV_ADDR 0x51

static SSD1306Wire display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED);

volatile uint8_t rxBuffer[32];  // Buffer for received data
volatile size_t dataLength = 0;
volatile uint8_t receivedValue = 0;  // Store the single received byte
volatile bool dataReady = false;    // Flag to indicate data availability

/**
 * @brief Enables the external voltage pin (Vext) for peripherals like the OLED.
 */
void VextON(void) {
  pinMode(Vext, OUTPUT);
  digitalWrite(Vext, LOW); // Enable Vext
}

/**
 * @brief Disables the external voltage pin (Vext) for peripherals.
 */
void VextOFF(void) {
  pinMode(Vext, OUTPUT);
  digitalWrite(Vext, HIGH); // Disable Vext
}
/**
 * @brief Interrupt handler for I2C slave receive event.
 * @param len Number of bytes received (should always be 1).
 */
void onReceive(int len) {
    if (len > 0 && Wire.available()) {
        receivedValue = Wire.read();  // Store the received byte
        dataReady = true;             // Indicate data is ready
    }
}

/**
 * @brief Blocking function to wait for a single byte of data.
 * @return The received byte value.
 */
uint8_t waitForData() {
    // Wait until the data is available
    while (!dataReady) {
        // Optional: You can implement a timeout mechanism here
    }

    dataReady = false;  // Reset the flag for future use
    return receivedValue;  // Return the value received
}

void onRequest() {
    // Send the received data back to the master
    Wire.write((const uint8_t*)rxBuffer, dataLength);
    Serial.println("Data sent to master.");

    // Clear the buffer after sending data
    memset((void*)rxBuffer, 0, sizeof(rxBuffer));
    dataLength = 0;
}

void dispFiveDigits () {
  uint8_t value = waitForData();  // Wait for the data to arrive
  display.drawString(0, 0, String(value));
  display.display(); // Write to the OLED screen
  value = waitForData();  // Wait for the data to arrive
  display.drawString(0, 1, String(value));
  display.display(); // Write to the OLED screen
  value = waitForData();  // Wait for the data to arrive
  display.drawString(0, 2, String(value));
  display.display(); // Write to the OLED screen
  value = waitForData();  // Wait for the data to arrive
  display.drawString(0, 3, String(value));
  display.display(); // Write to the OLED screen
  value = waitForData();  // Wait for the data to arrive
  display.drawString(0, 1, String(value));
  display.display(); // Write to the OLED screen
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
  
  // Enable external power for the OLED display
  VextON();
  delay(100);

  // Initialize the OLED display
  display.init();
  display.setFont(ArialMT_Plain_10);

}

void loop() {
  dispFiveDigits();
//    uint8_t value = waitForData();  // Wait for the data to arrive

//    Serial.print("Received Value: ");
//    Serial.println(value, HEX);  // Print the value in HEX format
}


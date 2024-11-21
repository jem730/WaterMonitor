/**
 * Ultrasonic Distance Measurement with OLED Display
 *
 * This program reads the distance from an ultrasonic rangefinder 
 * (e.g., HC-SR04 or equivalent) and displays the measured distance 
 * on an OLED screen using the Heltec ESP32 development board. 
 * It also prints the distance to the Serial Monitor for debugging.
 *
 * Pins Used:
 * - `trigPin (GPIO47)`: Trigger pin for the ultrasonic rangefinder.
 * - `echoPin (GPIO48)`: Echo pin for the ultrasonic rangefinder.
 * - I2C for OLED:
 *    - SDA: Default board-specific pin (Heltec ESP32-specific).
 *    - SCL: Default board-specific pin (Heltec ESP32-specific).
 *
 * The program uses the `HT_SSD1306Wire` library to manage the OLED
 * and measures distance based on the speed of sound (0.034 cm/us).
 *
 * Functions:
 * - `getSDist()`: Sends a pulse to the ultrasonic sensor and calculates the distance.
 * - `displayDist(int dist)`: Displays the measured distance on the OLED screen.
 * - `VextON() / VextOFF()`: Enables or disables the external voltage pin for the OLED.
 * - `loop()`: Continuously measures and displays the distance.
 */

#include <Wire.h>               // I2C communication library
#include "HT_SSD1306Wire.h"     // OLED display library

#define trigPin 47 // GPIO pin for triggering the ultrasonic rangefinder
#define echoPin 48 // GPIO pin for receiving the echo signal

// Initialize the OLED display with its I2C address and configuration
static SSD1306Wire display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED);

void setup() {
  // Initialize serial communication for debugging
  Serial.begin(115200);
  Serial.println();
  Serial.println();

  // Enable external power for the OLED display
  VextON();
  delay(100);

  // Initialize the OLED display
  display.init();
  display.setFont(ArialMT_Plain_10);

  // Set up ultrasonic sensor pins
  pinMode(trigPin, OUTPUT); // Set trigger pin as output
  pinMode(echoPin, INPUT);  // Set echo pin as input
}

/**
 * @brief Sends a pulse to the ultrasonic rangefinder and calculates the distance.
 * 
 * @return int Distance in centimeters.
 */
int getSDist() {
  long duration = 0;  // Variable to store pulse duration
  int distance = 0;   // Variable to store calculated distance

  // Clear the trigPin by setting it LOW
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);

  // Send a 10-microsecond pulse to trigPin
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Measure the duration of the echo pulse
  duration = pulseIn(echoPin, HIGH);

  // Calculate the distance in cm (speed of sound: 0.034 cm/us)
  distance = duration * 0.034 / 2;

  // Print the distance to the Serial Monitor
  Serial.print("Distance = ");
  Serial.print(distance);
  Serial.println(" cm");

  delay(50); // Short delay for sensor stability
  return distance;
}

/**
 * @brief Displays the measured distance on the OLED display.
 * 
 * @param dist The distance to display in centimeters.
 */
void displayDist(int dist) {
  display.clear(); // Clear the OLED screen
  // Display the distance on the OLED
  display.drawString(0, 0, "Distance: " + String(dist) + " cm");
  display.display(); // Write to the OLED screen

  // Print the distance to the Serial Monitor for debugging
  Serial.print("Distance = ");
  Serial.print(dist);
  Serial.println(" cm");
}

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

void loop() {
  VextON(); // Ensure Vext is enabled for OLED display
  int cm_dist = 0;

  // Measure the distance using the ultrasonic sensor
  cm_dist = getSDist();
  delay(10); // Short delay before updating display

  // Display the distance on the OLED
  displayDist(cm_dist);
  delay(50); // Short delay before the next measurement
}

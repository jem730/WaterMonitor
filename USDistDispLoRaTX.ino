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
#include "LoRaWan_APP.h"
#include "Arduino.h"

#define trigPin 47 // GPIO pin for triggering the ultrasonic rangefinder
#define echoPin 48 // GPIO pin for receiving the echo signal
#define RF_FREQUENCY                                915000000 // Hz
#define TX_OUTPUT_POWER                             5        // dBm
#define LORA_BANDWIDTH                              0         // [0: 125 kHz,
                                                              //  1: 250 kHz,
                                                              //  2: 500 kHz,
                                                              //  3: Reserved]
#define LORA_SPREADING_FACTOR                       7         // [SF7..SF12]
#define LORA_CODINGRATE                             1         // [1: 4/5,
                                                              //  2: 4/6,
                                                              //  3: 4/7,
                                                              //  4: 4/8]
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         0         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false
#define RX_TIMEOUT_VALUE                            1000
#define BUFFER_SIZE                                 30 // Define the payload size here

char txpacket[BUFFER_SIZE];
char rxpacket[BUFFER_SIZE];

double txNumber;

bool lora_idle=true;

static RadioEvents_t RadioEvents;

void LoRaInitTX ( void );
void serialInit ( void );
void dispInit ( void );
void VextON ( void );
void VextOFF ( void ); 
void OnTxDone( void );
void OnTxTimeout( void );
void displayDist(int dist);
int getSDist( void ); 
void LoRaTX_Dist(int RDist);

// Initialize the OLED display with its I2C address and configuration
static SSD1306Wire display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED);

void setup() {

  // Enable external power for the OLED display
  VextON();
  delay(100);

  // Initialize the OLED display
  display.init();
  display.setFont(ArialMT_Plain_10);

  // Set up ultrasonic sensor pins
  pinMode(trigPin, OUTPUT); // Set trigger pin as output
  pinMode(echoPin, INPUT);  // Set echo pin as input

  serialInit();
  LoRaInitTX();
}

void loop() {
    int cm_dist = getSDist(); // Measure distance
    displayDist(cm_dist);     // Display distance on OLED
    LoRaTX_Dist(cm_dist);     // Transmit distance over LoRa

    Radio.IrqProcess();       // Check for LoRa events (transmission complete, timeout, etc.)
}


void LoRaInitTX ( void )  {
  Mcu.begin(HELTEC_BOARD,SLOW_CLK_TPYE);
  txNumber=0;

    RadioEvents.TxDone = OnTxDone;
    RadioEvents.TxTimeout = OnTxTimeout;
    
    Radio.Init( &RadioEvents );
    Radio.SetChannel( RF_FREQUENCY );
    Radio.SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                                   LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                                   LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   true, 0, 0, LORA_IQ_INVERSION_ON, 3000 ); 
}

void OnTxDone( void )  {
	Serial.println("TX done......");
	lora_idle = true;
}

void OnTxTimeout( void )  {
    Radio.Sleep( );
    Serial.println("TX Timeout......");
    lora_idle = true;
}
/**
 * @brief Sends a pulse to the ultrasonic rangefinder and calculates the distance.
 * 
 * @return int Distance in centimeters.
 */
int getSDist( void ) {
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
  Serial.print(" cm ");
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
void serialInit ( void ) {
  // Initialize serial communication for debugging
  Serial.begin(115200);
  Serial.println();
  Serial.println();
}

void LoRaTX_Dist(int RDist) {
    if (lora_idle == true) {
        sprintf(txpacket, "Distance: %d cm", RDist);  // Format the distance packet

        Serial.printf("\r\nSending packet \"%s\", length %d\r\n", txpacket, strlen(txpacket));

        // Send the packet over LoRa
        Radio.Send((uint8_t *)txpacket, strlen(txpacket));
        lora_idle = false;
    }
}






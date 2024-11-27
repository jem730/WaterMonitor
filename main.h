#ifndef ESP_H
#define ESP_H

#include "stm32l4xx_hal.h" // HAL drivers for STM32L4 series
#include <stdint.h>
#include <string.h> // For string manipulation (if needed)

// Define the I2C address of the ESP32 (7-bit format)
#define ESP32_I2C_ADDRESS 0x30

// Function prototypes
/**
 * @brief Initialize GPIOs and I2C settings for communication with ESP32
 * @retval None
 */
void ESP32_Init(void);

/**
 * @brief Send data to ESP32 over I2C
 * @param data: Pointer to the buffer containing the data to be sent
 * @param size: Number of bytes to send
 * @retval None
 */
void ESP32_Write(uint8_t *data, uint16_t size);

/**
 * @brief Read data from ESP32 over I2C
 * @param data: Pointer to the buffer to store the received data
 * @param size: Number of bytes to read
 * @retval None
 */
void ESP32_Read(uint8_t *data, uint16_t size);

#endif /* ESP_H */

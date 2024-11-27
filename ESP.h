#ifndef ESP_H
#define ESP_H

#include "stm32l4xx_hal.h" // HAL drivers
#include "stm32l4xx_hal_i2c.h" // I2C-specific HAL driver
#include <stdint.h>
#include <string.h>

#define ESP32_I2C_ADDRESS 0x30 // I2C address of ESP32

void ESP32_Init(void);
void ESP32_Write(uint8_t *data, uint16_t size);
void ESP32_Read(uint8_t *data, uint16_t size);

#endif /* ESP_H */

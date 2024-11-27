#include "ESP.h"

// Use the I2C handle from the main program
extern I2C_HandleTypeDef hi2c1;


/**
  * @brief Initialize GPIOs for I2C communication with ESP32
  * @retval None
  */
void ESP32_Init(void) {
    RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOBEN); // Enable GPIOB clock

    // Alternate function setup
    GPIOB->AFR[1] &= ~((0x000F << GPIO_AFRH_AFSEL8_Pos)); // SCL
    GPIOB->AFR[1] |= ((0x0004 << GPIO_AFRH_AFSEL8_Pos));
    GPIOB->AFR[1] &= ~((0x000F << GPIO_AFRH_AFSEL9_Pos)); // SDA
    GPIOB->AFR[1] |= ((0x0004 << GPIO_AFRH_AFSEL9_Pos));

    // Open drain output
    GPIOB->OTYPER &= ~(GPIO_OTYPER_OT8 | GPIO_OTYPER_OT9);
    GPIOB->OTYPER |= (GPIO_OTYPER_OT8 | GPIO_OTYPER_OT9);

    // Pull-up resistors
    GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD8 | GPIO_PUPDR_PUPD9);
    GPIOB->PUPDR |= (GPIO_PUPDR_PUPD8_0 | GPIO_PUPDR_PUPD9_0);

    // Very high speed
    GPIOB->OSPEEDR |= ((3 << GPIO_OSPEEDR_OSPEED8_Pos) |
                       (3 << GPIO_OSPEEDR_OSPEED9_Pos));

    // Alternate function mode
    GPIOB->MODER &= ~(GPIO_MODER_MODE8 | GPIO_MODER_MODE9);
    GPIOB->MODER |= (GPIO_MODER_MODE8_1 | GPIO_MODER_MODE9_1);

    // Enable I2C clock
    RCC->APB1ENR1 |= RCC_APB1ENR1_I2C1EN;
    I2C1->CR1 &= ~(I2C_CR1_PE);          // Reset I2C
    I2C1->CR1 &= ~(I2C_CR1_ANFOFF);     // Enable analog filter
    I2C1->CR1 &= ~(I2C_CR1_DNF);        // Disable digital filter
    I2C1->TIMINGR = 0x00503D58;         // Timing for 16 MHz SYSCLK
    I2C1->CR2 &= ~(I2C_CR2_ADD10);      // 7-bit address mode
    I2C1->CR1 |= (I2C_CR1_PE);          // Enable I2C
}

/**
  * @brief Send data to ESP32 over I2C
  * @param data: Pointer to data buffer
  * @param size: Number of bytes to send
  * @retval None
  */
void ESP32_Write(uint8_t *data, uint16_t size) {
    I2C1->CR2 &= ~(I2C_CR2_RD_WRN);          // Set WRITE mode
    I2C1->CR2 &= ~(I2C_CR2_NBYTES);          // Clear byte count
    I2C1->CR2 |= (size << I2C_CR2_NBYTES_Pos); // Set number of bytes to write
    I2C1->CR2 &= ~(I2C_CR2_SADD);            // Clear device address
    I2C1->CR2 |= (ESP32_I2C_ADDRESS << (I2C_CR2_SADD_Pos + 1)); // Set device address
    I2C1->CR2 |= I2C_CR2_START;              // Start I2C WRITE op

    for (uint16_t i = 0; i < size; i++) {
        while (!(I2C1->ISR & I2C_ISR_TXIS)); // Wait for TX buffer
        I2C1->TXDR = data[i];               // Transmit data
    }

    while (!(I2C1->ISR & I2C_ISR_TC));       // Wait for transfer complete
    I2C1->CR2 |= I2C_CR2_STOP;              // Generate STOP condition
}

/**
  * @brief Read data from ESP32 over I2C
  * @param data: Pointer to data buffer
  * @param size: Number of bytes to read
  * @retval None
  */
void ESP32_Read(uint8_t *data, uint16_t size) {
    I2C1->CR2 |= I2C_CR2_RD_WRN;              // Set READ mode
    I2C1->CR2 &= ~(I2C_CR2_NBYTES);           // Clear byte count
    I2C1->CR2 |= (size << I2C_CR2_NBYTES_Pos); // Set number of bytes to read
    I2C1->CR2 &= ~(I2C_CR2_SADD);             // Clear device address
    I2C1->CR2 |= (ESP32_I2C_ADDRESS << (I2C_CR2_SADD_Pos + 1)); // Set device address
    I2C1->CR2 |= I2C_CR2_START;               // Start I2C READ op

    for (uint16_t i = 0; i < size; i++) {
        while (!(I2C1->ISR & I2C_ISR_RXNE)); // Wait for RX buffer
        data[i] = I2C1->RXDR;                // Receive data
    }

    while (!(I2C1->ISR & I2C_ISR_TC));        // Wait for transfer complete
    I2C1->CR2 |= I2C_CR2_STOP;               // Generate STOP condition
}

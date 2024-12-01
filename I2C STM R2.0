/*******************************************************************************
* EE 329 A9 I2C EEPROM
*******************************************************************************
* @file           : main.c
* @brief          : Data written to EEPROM using I2C protocol
* project         : EE 329 F'24 Assignment 9
* authors         : John Mort - jmort@calpoly.edu
* citations 	  : Keanu Lam - klam93@calpoly.edu
*
* date            : 11/11/24
* compiler        : STM32CubeIDE v.1.16.1 Build:  22882_20240916_0822 (UTC)
* target          : NUCLEO-L4A6ZG
*******************************************************************************
*Description: This program writes a single byte of data to the EEPROM using I2C
*			  then reads it back. If the read data matches what was originally
*			  written, the blue LED on the NUCLEO board (PB-7) lights up. The
*			  EEPROM address is hard wired to 0x51 via the three chip address
*			  pins (A0-A2), and the WP pin is grounded to enable write
*			  operations. Both I2C lines require a 2 kOhm pullup resistor to 5V
*			  to operate at 400 kHz.
*
*******************************************************************************
*Pinout: PB-7: Blue indicator LED on the Nucleo board
*		 PB-8: SCL (I2C, requires pull-up)
*		 PB-9: SDA (I2C, requires pull-up)
******************************************************************************/
#include "main.h"
#include "stm32l4xx.h" // Make sure to include appropriate STM32 headers

int main(void) {
    // Initialize GPIO for I2C and on-board LED (Assume LED on PA5 for Nucleo)
    GPIO_I2C1_Init();   // Initialize GPIO for I2C
    I2C1_Init();        // Initialize I2C1 peripheral

//  Configure the blue LED pin
    GPIOB->MODER &= ~(GPIO_MODER_MODE7);
    GPIOB->MODER |= (1 << GPIO_MODER_MODE7_Pos);

    // Write random byte to EEPROM at the specified address
    EEPROM_WriteByte(RANDOM_ADDR, RANDOM_BYTE);
    // Wait for 5 ms to allow data to be written
    Delay_ms(5000);
    // Read the byte back from EEPROM
    uint8_t read_data = EEPROM_ReadByte(RANDOM_ADDR);

    // Check if read data matches written data
    if (read_data == RANDOM_BYTE) {
        GPIOB->ODR |= (1 << 7);  // Turn ON LED if data matches
    } else {
        GPIOB->ODR &= ~(1 << 7); // Keep LED OFF if data doesn't match
    }

    // Infinite loop
    while (1);
}



/* -----------------------------------------------------------------------------
* function : Delay_ms()
* action   : Defines a ms delay.
* authors  : -John Mort - jmort@calpoly.edu
* date     : 11/10/24
* -------------------------------------------------------------------------- */
// Simple millisecond delay function (Assumes 16 MHz SysTick)
void Delay_ms(uint32_t ms) {
    SysTick->LOAD = 16000 - 1;  // 1 ms delay at 16 MHz
    SysTick->VAL = 0;           // Clear current value register
    SysTick->CTRL = 5;          // Enable SysTick with processor clock

    for (uint32_t i = 0; i < ms; i++) {
        while (!(SysTick->CTRL & 0x10000));  // Wait for the COUNTFLAG
    }

    SysTick->CTRL = 0;          // Disable SysTick
}







/* -----------------------------------------------------------------------------
* function : GPIO_I2C1_Init()
* action   : Configures PB8 and PB9 for I2C1 SCL and SDA operation.
*            Sets GPIO mode to alternate function with AF4, configures as
*            open-drain, medium speed, and no pull-up/pull-down.
* authors  : -John Mort - jmort@calpoly.edu
* date     : 11/10/24
* -------------------------------------------------------------------------- */

void GPIO_I2C1_Init(void) {
	// Configure PB8 and PB9 for SCL and SDA respectively
	// Enable GPIOB clock
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
	// Clear PB8 and PB9 mode bits
	GPIOB->MODER &= ~(GPIO_MODER_MODE8 | GPIO_MODER_MODE9);
    // Set PB8 and PB9 to AF mode
    GPIOB->MODER |= (GPIO_MODER_MODE8_1 | GPIO_MODER_MODE9_1);
    // Set PB8 and PB9 as no pull-up/pull-down
    GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD8 | GPIO_PUPDR_PUPD9);
    // Set PB8 and PB 9 in open-drain configuration
    GPIOB->OTYPER |= (GPIO_OTYPER_OT8 | GPIO_OTYPER_OT9);
    // Set medium speed for PG7 and PG8
    GPIOB->OSPEEDR |= (GPIO_OSPEEDR_OSPEED8_0 | GPIO_OSPEEDR_OSPEED9_0);
    // Set Alternate Function 4 (AF4) for I2C1 on PB8 and PB9
    GPIOB->AFR[1] |= (4 << GPIO_AFRH_AFSEL8_Pos) | (4 << GPIO_AFRH_AFSEL9_Pos);
}

/* -----------------------------------------------------------------------------
* function : I2C1_Init()
* action   : Configures I2C1 peripheral, sets timing, enables I2C, and
*            configures filter settings. Uses CubeMX timing for 16MHz SYSCLK.
* authors  : -John Mort - jmort@calpoly.edu
* 			 -John Penvenne - jpenvenn@calpoly.edu
* date     : 11/10/24
* -------------------------------------------------------------------------- */

void I2C1_Init(void) {
	// Configure I2C
	RCC->APB1ENR1 |= RCC_APB1ENR1_I2C1EN;  // enable I2C bus clock
	I2C1->CR1   &= ~( I2C_CR1_PE );        // put I2C into reset (release SDA, SCL)
	I2C1->CR1   &= ~( I2C_CR1_ANFOFF );    // filters: enable analog
	I2C1->CR1   &= ~( I2C_CR1_DNF );       // filters: disable digital
	I2C1->TIMINGR = 0x00303D5B;            // 16 MHz SYSCLK timing from CubeMX
	I2C1->CR2   &= ~( I2C_CR2_ADD10 );     // 7-bit address mode
	I2C1->CR1   |=  ( I2C_CR1_PE );        // enable I2C
}

/* -----------------------------------------------------------------------------
* function : EEPROM_WriteByte(uint16_t mem_address, uint8_t data)
* INs      : mem_address - 16-bit memory address in EEPROM
*            data        - 8-bit data to write to EEPROM
* OUTs     : none
* action   : Writes a single byte to the specified EEPROM address. Sends
*            control byte, address, and data byte, generating STOP after last byte.
* authors  : -John Mort - jmort@calpoly.edu
* resources: Referenced code from John Penvenne - jpenvenn@calpoly.edu
* date     : 11/10/24
* -------------------------------------------------------------------------- */

void EEPROM_WriteByte(uint16_t mem_address, uint8_t data) {

    while (I2C1->ISR & I2C_ISR_BUSY);       // Wait until I2C is ready
    /* Set up the control byte:
       1. Shift address left 1 space as LSB is used for R/W
       2. Set number of bytes per transaction to three
       3. Enable AUTOEND, which sends a stop bit after the last byte */
    I2C1->CR2 = (EEPROM_ADDRESS << 1) | (3 << I2C_CR2_NBYTES_Pos)
    		     | I2C_CR2_AUTOEND;
    I2C1->CR2 &= ~I2C_CR2_RD_WRN;           // Write mode
    I2C1->CR2 |= I2C_CR2_START;             // Generate START condition
    										// transmit interrupt status (TXIS)
    while (!(I2C1->ISR & I2C_ISR_TXIS));    // Wait for TXIS flag
    I2C1->TXDR = (mem_address >> 8) & 0xFF; // Send high byte of memory address
    while (!(I2C1->ISR & I2C_ISR_TXIS));    // Wait for TXIS flag
    I2C1->TXDR = mem_address & 0xFF;        // Send low byte of memory address
    while (!(I2C1->ISR & I2C_ISR_TXIS));    // Wait for TXIS flag
    I2C1->TXDR = data;                      // Send data byte
    while (!(I2C1->ISR & I2C_ISR_STOPF));   // Wait until STOP condition is detected
    I2C1->ICR = I2C_ICR_STOPCF;             // Clear the STOPF flag by writing to STOPCF

}

/* -----------------------------------------------------------------------------
* function : EEPROM_ReadByte(uint16_t mem_address)
* INs      : mem_address - 16-bit memory address in EEPROM
* OUTs     : 8-bit data read from EEPROM
* action   : Reads a single byte from the specified EEPROM address. Sends control
*            byte and address in write mode, followed by repeated START to read data.
* authors  : -John Mort - jmort@calpoly.edu
* resources: Referenced code from John Penvenne, with guidance from ChatGPT by OpenAI
* date     : 11/10/24
* -------------------------------------------------------------------------- */

uint8_t EEPROM_ReadByte(uint16_t mem_address) {
    uint8_t data = 0;

    while (I2C1->ISR & I2C_ISR_BUSY);       // Wait until I2C is ready
    // Send control byte
    I2C1->CR2 = (EEPROM_ADDRESS << 1) | (2 << I2C_CR2_NBYTES_Pos);
    I2C1->CR2 &= ~I2C_CR2_RD_WRN;           // Write mode
    I2C1->CR2 |= I2C_CR2_START;             // Generate START condition

    // Send address
    while (!(I2C1->ISR & I2C_ISR_TXIS));    // Wait for TXIS flag
    I2C1->TXDR = (mem_address >> 8) & 0xFF; // Send high byte of memory address
    while (!(I2C1->ISR & I2C_ISR_TXIS));    // Wait for TXIS flag
    I2C1->TXDR = mem_address & 0xFF;        // Send low byte of memory address

    // Read data, no ACK so no condition checks required
    I2C1->CR2 = (EEPROM_ADDRESS << 1) | (1 << I2C_CR2_NBYTES_Pos)
    			 | I2C_CR2_RD_WRN | I2C_CR2_START | I2C_CR2_AUTOEND;
    while (!(I2C1->ISR & I2C_ISR_RXNE));    // Wait for RXNE flag
    data = I2C1->RXDR;                      // Read data byte

    return data;
}


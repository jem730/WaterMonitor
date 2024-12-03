#include "main.h"
#include "stm32l4xx.h"

/* Private function prototypes ----------------------------------------------- */
void SystemClock_Config(void);
void GPIO_I2C1_Init(void);
void I2C1_Init(void);
void Delay_ms(uint32_t ms);
void I2C_SendNumber(uint8_t number);

/* Main ---------------------------------------------------------------------- */
int main(void) {
    // Initialize HAL, clock, and GPIO
    HAL_Init();
    SystemClock_Config();
    GPIO_I2C1_Init();
    I2C1_Init();

    // Configure LED on PB7
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
    GPIOB->MODER &= ~(GPIO_MODER_MODE7); // Clear mode bits
    GPIOB->MODER |= (1 << GPIO_MODER_MODE7_Pos); // Set as output

    uint8_t number_to_send = 6;

    while (1) {
        GPIOB->ODR |= (1 << 7); // Turn ON LED
        Delay_ms(10); // Delay for 1 second
        I2C_SendNumber(number_to_send);
        number_to_send++;
//        GPIOB->ODR &= ~(1 << 7); // Turn OFF LED
        Delay_ms(10); // Delay for 1 second
    }
}

/* -----------------------------------------------------------------------------
 * Function : Delay_ms
 * Action   : Simple millisecond delay using SysTick
 ----------------------------------------------------------------------------- */
void Delay_ms(uint32_t ms) {
    SysTick->LOAD = 16000 - 1; // 1 ms delay at 16 MHz
    SysTick->VAL = 0;          // Clear current value
    SysTick->CTRL = 5;         // Enable SysTick with processor clock

    for (uint32_t i = 0; i < ms; i++) {
        while (!(SysTick->CTRL & 0x10000)); // Wait for COUNTFLAG
    }

    SysTick->CTRL = 0; // Disable SysTick
}

/* -----------------------------------------------------------------------------
 * Function : GPIO_I2C1_Init
 * Action   : Configures PB8 and PB9 for I2C SCL and SDA operation
 ----------------------------------------------------------------------------- */
void GPIO_I2C1_Init(void) {
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN; // Enable GPIOB clock

    // Configure PB8 (SCL) and PB9 (SDA) as alternate function (AF4)
    GPIOB->MODER &= ~(GPIO_MODER_MODE8 | GPIO_MODER_MODE9);
    GPIOB->MODER |= (GPIO_MODER_MODE8_1 | GPIO_MODER_MODE9_1); // AF mode
    GPIOB->OTYPER |= (GPIO_OTYPER_OT8 | GPIO_OTYPER_OT9);       // Open-drain
    GPIOB->OSPEEDR |= (GPIO_OSPEEDR_OSPEED8 | GPIO_OSPEEDR_OSPEED9); // Medium speed
    GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD8 | GPIO_PUPDR_PUPD9);     // No pull-up/pull-down
    GPIOB->AFR[1] |= (4 << GPIO_AFRH_AFSEL8_Pos) | (4 << GPIO_AFRH_AFSEL9_Pos); // AF4
}

/* -----------------------------------------------------------------------------
 * Function : I2C1_Init
 * Action   : Initializes the I2C1 peripheral with 400 kHz settings
 ----------------------------------------------------------------------------- */
void I2C1_Init(void) {
    RCC->APB1ENR1 |= RCC_APB1ENR1_I2C1EN; // Enable I2C1 clock
    I2C1->CR1 &= ~I2C_CR1_PE;             // Disable I2C peripheral
    I2C1->CR1 &= ~I2C_CR1_ANFOFF;         // Enable analog filter
    I2C1->CR1 &= ~I2C_CR1_DNF;            // Disable digital filter
    I2C1->TIMINGR = 0x00303D5B;           // Timing for 16 MHz SYSCLK
    I2C1->CR2 &= ~I2C_CR2_ADD10;          // 7-bit addressing
    I2C1->CR1 |= I2C_CR1_PE;              // Enable I2C peripheral
}

/* -----------------------------------------------------------------------------
 * Function : I2C_SendNumber
 * Action   : Sends a single byte (number) to the ESP32 over I2C
 ----------------------------------------------------------------------------- */
#define I2C_TIMEOUT_MS 10  // Maximum wait time for bus idle in milliseconds

void I2C_SendNumber(uint8_t number) {
    uint8_t ESP32_ADDRESS = 0x51;
    uint32_t timeout;

    // Wait until the I2C bus is free or timeout
    timeout = I2C_TIMEOUT_MS;
    while ((I2C1->ISR & I2C_ISR_BUSY) && timeout > 0) {
        Delay_ms(1);
        timeout--;
    }

    if (timeout == 0) {
        return;  // Exit if the bus is busy
    }

    // Configure the I2C transaction
    I2C1->CR2 = (ESP32_ADDRESS << 1)        // Set slave address (7-bit) for write
                | (1 << I2C_CR2_NBYTES_Pos) // 1 byte to send
                | I2C_CR2_AUTOEND;          // Generate STOP after transmission
    I2C1->CR2 &= ~I2C_CR2_RD_WRN;           // Write mode
    I2C1->CR2 |= I2C_CR2_START;             // Generate START condition

    // Wait for TXIS flag
    timeout = I2C_TIMEOUT_MS;
    while (!(I2C1->ISR & I2C_ISR_TXIS) && timeout > 0) {
        Delay_ms(1);
        timeout--;
    }

    if (timeout == 0) {
        return;  // Exit if TXIS flag is not set
    }

    // Send data byte
    I2C1->TXDR = number;

    // Wait for TC flag
    timeout = I2C_TIMEOUT_MS;
    while (!(I2C1->ISR & I2C_ISR_TC) && timeout > 0) {
        Delay_ms(1);
        timeout--;
    }

    if (timeout == 0) {
        return;  // Exit if TC flag is not set
    }

    // Clear the STOP condition flag
    I2C1->ICR = I2C_ICR_STOPCF;
}



void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_8;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}


void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

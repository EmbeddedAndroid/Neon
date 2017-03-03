/**
  ******************************************************************************
  * @file    GPIO/GPIO_IOToggle/Src/main.c
  * @author  MCD Application Team
  * @version V1.1.5
  * @date    06-May-2016
  * @brief   This example describes how to configure and use GPIOs through
  *          the STM32F4xx HAL API.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/** @addtogroup STM32F4xx_HAL_Examples
  * @{
  */

/** @addtogroup GPIO_IOToggle
  * @{
  */

int __io_getchar(void);
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static GPIO_InitTypeDef  GPIO_InitStruct;
UART_HandleTypeDef UartHandle;
I2C_HandleTypeDef I2cxHandle, I2cyHandle;

//#define DEBUG 1

#define LED_SLEEP_DELAY 100
#define LED_OFF_SLEEP_DELAY 50

#define I2C_LED_ADDRESS 0x60
#define I2C_TEMP_ADDRESS 0x70

/* Private function prototypes -----------------------------------------------*/
#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
static void SystemClock_Config(void);
static void Error_Handler(void);

/* Private functions ---------------------------------------------------------*/
static void write_I2C_register(I2C_HandleTypeDef *h, uint16_t addr_bus, uint8_t register_pointer, uint16_t register_value)
{
    uint8_t data[3];
    data[0] = register_pointer;
    data[1] = register_value>>8;    // MSB byte of 16bit data
    data[2] = register_value;       // LSB byte of 16bit data

    HAL_I2C_Master_Transmit(h, addr_bus, data, 3, 100);  // data is the start pointer of our array
}

#if DEBUG
static void i2cdetect(I2C_HandleTypeDef *h, const char *s, int addr_min, int addr_max)
{
  int a;

  if (addr_min < 0x03)
    addr_min = 0x03;
  if (addr_max > 0x77)
    addr_max = 0x77;

  if (s)
    printf("\r\ni2cdetect on bus %s", s);
  printf("\r\n     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f");
  for (a = 0; a < 128; a++) {
    if ((a & 0xf) == 0)
      printf("\r\n%02x:", a);
    if (a < addr_min || a > addr_max) {
      printf("   ");
      continue;
    }
    if (HAL_I2C_Master_Transmit(h, (uint16_t)(2*a), NULL, 0, 25)== HAL_OK) {
      printf(" %02x", a);
      continue;
    }
    if (HAL_I2C_GetError(h) == HAL_I2C_ERROR_AF) {
      printf(" --");
      continue;
    }
    printf(" ??");
  }
  printf("\r\n");
}
#endif

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  /* STM32F4xx HAL library initialization:
       - Configure the Flash prefetch, instruction and Data caches
       - Configure the Systick to generate an interrupt each 1 msec
       - Set NVIC Group Priority to 4
       - Global MSP (MCU Support Package) initialization
     */
  HAL_Init();

  /* Configure the system clock to 100 MHz */
  SystemClock_Config();

  /*##-1- Enable GPIOB Clock (to be able to program the configuration registers) */
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*##-1- Configure the UART peripheral ######################################*/
  /* Put the USART peripheral in the Asynchronous mode (UART Mode) */
  /* UART1 configured as follow:
      - Word Length = 8 Bits
      - Stop Bit = One Stop bit
      - Parity = NO parity
      - BaudRate = 115200 baud
      - Hardware flow control disabled (RTS and CTS signals) */
  UartHandle.Instance          = USARTx;

  UartHandle.Init.BaudRate     = 115200;
  UartHandle.Init.WordLength   = UART_WORDLENGTH_8B;
  UartHandle.Init.StopBits     = UART_STOPBITS_1;
  UartHandle.Init.Parity       = UART_PARITY_NONE;
  UartHandle.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
  UartHandle.Init.Mode         = UART_MODE_TX_RX;
  UartHandle.Init.OverSampling = UART_OVERSAMPLING_16;

  if(HAL_UART_Init(&UartHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  /*##-1- Configure the I2C peripheral ######################################*/
  I2cxHandle.Instance             = I2Cx;

  I2cxHandle.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
  I2cxHandle.Init.ClockSpeed      = 100000;
  I2cxHandle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  I2cxHandle.Init.DutyCycle       = I2C_DUTYCYCLE_16_9;
  I2cxHandle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  I2cxHandle.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
  I2cxHandle.Init.OwnAddress1     = 0x00;
  I2cxHandle.Init.OwnAddress2     = 0x00;

  if(HAL_I2C_Init(&I2cxHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  I2cyHandle.Instance             = I2Cy;

  I2cyHandle.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
  I2cyHandle.Init.ClockSpeed      = 100000;
  I2cyHandle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  I2cyHandle.Init.DutyCycle       = I2C_DUTYCYCLE_16_9;
  I2cyHandle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  I2cyHandle.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
  I2cyHandle.Init.OwnAddress1     = 0x00;
  I2cyHandle.Init.OwnAddress2     = 0x00;

  if(HAL_I2C_Init(&I2cyHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  /*##-2- Configure PB12~15 IO in output push-pull mode to drive external LED ###*/
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_13;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_14;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_15;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* Initial delay, on purpose  */
  HAL_Delay(1000);

#if DEBUG
  printf("Hello world !\r\n");

  i2cdetect(&I2cxHandle, "I2C2", 0, 127);
  i2cdetect(&I2cyHandle, "I2C3", 0, 127);
#endif

  write_I2C_register(&I2cyHandle, 2 * I2C_LED_ADDRESS, 0x03, 0xFF); // Set PWM0 duty cycle to 100%
  write_I2C_register(&I2cyHandle, 2 * I2C_LED_ADDRESS, 0x05, 0xFF); // Set PWM1 duty cycle to 100%

  uint8_t ret = 0;

  /* Init/Reset Si7034 */
  uint8_t si_buf[6]; /* 6 is the max read bytes */
  /* Command: measure temperature and relative humidity, normal hold mode */
  si_buf[0] = 0xFE;
  if ((ret = HAL_I2C_Master_Transmit(&I2cxHandle, 2 * I2C_TEMP_ADDRESS, si_buf, 1, 100)))
    printf("Si7034: I2C HAL ERROR RESET: %d\r\n", ret);

  HAL_Delay(100);

  if ((ret = HAL_I2C_IsDeviceReady(&I2cxHandle, 2 * I2C_TEMP_ADDRESS, 3, 100))) {
    printf("Si7034: Device not ready, aborting! - Error: %d\r\n", ret);
    write_I2C_register(&I2cyHandle, 2 * I2C_LED_ADDRESS, 0x09, 0x0C); // RED LED
    return -1;
  }

  /* Get Si7034 device ID */
  si_buf[0] = 0xFC;
  si_buf[1] = 0xC9;
  if ((ret = HAL_I2C_Master_Transmit(&I2cxHandle, 2 * I2C_TEMP_ADDRESS, si_buf, 2, 100))) {
    printf("Si7034: I2C HAL ERROR DEVICE ID: %d\r\n", ret);
    write_I2C_register(&I2cyHandle, 2 * I2C_LED_ADDRESS, 0x09, 0x0C); // RED LED
    return -1;
  }
  HAL_I2C_Master_Receive(&I2cxHandle, 2 * I2C_TEMP_ADDRESS, si_buf, 6, 100);
  if (si_buf[0] != 0x22) {
    printf("Si7034: INVALID DEVICE ID: %d\r\n", si_buf[0]);
    write_I2C_register(&I2cyHandle, 2 * I2C_LED_ADDRESS, 0x09, 0x0C); // RED LED
    return -1;
  }

  /*##-3- Toggle PB12~15 IO in an infinite loop #################################*/
  while (1)
  {
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
    HAL_Delay(LED_SLEEP_DELAY);
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);

    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_15);
    HAL_Delay(LED_SLEEP_DELAY);
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_15);

    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);
    HAL_Delay(LED_SLEEP_DELAY);
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);

    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_13);
    HAL_Delay(LED_SLEEP_DELAY);
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_13);

  /* -4 flashing I2C bus circular LEDs + RGB LED

  When used only as GPIO expander registers 0x00 to 0x01
  (0x00 for LP3944) and 0x06 to 0x09 (0x06 to 0x07 for
  LP3944) are of interest. Registers 0x00 and 0x01 are used to
  read the status of the I/Os. Registers 0x06 to 0x09 are used
  to control the status of the outputs.

  */

    // RGB LEDs
    write_I2C_register(&I2cyHandle, 2 * I2C_LED_ADDRESS, 0x09, 0x0C); // RGB
    HAL_Delay(LED_SLEEP_DELAY);

    // incremental
    write_I2C_register(&I2cyHandle, 2 * I2C_LED_ADDRESS, 0x06, 0x03); // LED 0
    HAL_Delay(LED_SLEEP_DELAY);
    write_I2C_register(&I2cyHandle, 2 * I2C_LED_ADDRESS, 0x06, 0x0F); // LED 1
    HAL_Delay(LED_SLEEP_DELAY);
    write_I2C_register(&I2cyHandle, 2 * I2C_LED_ADDRESS, 0x06, 0x3F); // LED 2
    HAL_Delay(LED_SLEEP_DELAY);
    write_I2C_register(&I2cyHandle, 2 * I2C_LED_ADDRESS, 0x06, 0xFF); // LED 3
    HAL_Delay(LED_SLEEP_DELAY);
    write_I2C_register(&I2cyHandle, 2 * I2C_LED_ADDRESS, 0x07, 0x03); // LED 4
    HAL_Delay(LED_SLEEP_DELAY);
    write_I2C_register(&I2cyHandle, 2 * I2C_LED_ADDRESS, 0x07, 0x0F); // LED 5
    HAL_Delay(LED_SLEEP_DELAY);
    write_I2C_register(&I2cyHandle, 2 * I2C_LED_ADDRESS, 0x07, 0x3F); // LED 6
    HAL_Delay(LED_SLEEP_DELAY);
    write_I2C_register(&I2cyHandle, 2 * I2C_LED_ADDRESS, 0x07, 0xFF); // LED 7
    HAL_Delay(LED_SLEEP_DELAY);
    write_I2C_register(&I2cyHandle, 2 * I2C_LED_ADDRESS, 0x08, 0x03); // LED 8
    HAL_Delay(LED_SLEEP_DELAY);
    write_I2C_register(&I2cyHandle, 2 * I2C_LED_ADDRESS, 0x08, 0x0F); // LED 9
    HAL_Delay(LED_SLEEP_DELAY);
    write_I2C_register(&I2cyHandle, 2 * I2C_LED_ADDRESS, 0x08, 0x3F); // LED 10
    HAL_Delay(LED_SLEEP_DELAY);
    write_I2C_register(&I2cyHandle, 2 * I2C_LED_ADDRESS, 0x08, 0xFF); // LED 11
    HAL_Delay(LED_SLEEP_DELAY);

    write_I2C_register(&I2cyHandle, 2 * I2C_LED_ADDRESS, 0x09, 0x00); // RGB
    HAL_Delay(LED_OFF_SLEEP_DELAY);

    write_I2C_register(&I2cyHandle, 2 * I2C_LED_ADDRESS, 0x06, 0xFC); // LED 0
    HAL_Delay(LED_OFF_SLEEP_DELAY);
    write_I2C_register(&I2cyHandle, 2 * I2C_LED_ADDRESS, 0x06, 0xF0); // LED 1
    HAL_Delay(LED_OFF_SLEEP_DELAY);
    write_I2C_register(&I2cyHandle, 2 * I2C_LED_ADDRESS, 0x06, 0xC0); // LED 2
    HAL_Delay(LED_OFF_SLEEP_DELAY);
    write_I2C_register(&I2cyHandle, 2 * I2C_LED_ADDRESS, 0x06, 0x00); // LED 3
    HAL_Delay(LED_OFF_SLEEP_DELAY);
    write_I2C_register(&I2cyHandle, 2 * I2C_LED_ADDRESS, 0x07, 0xFC); // LED 4
    HAL_Delay(LED_OFF_SLEEP_DELAY);
    write_I2C_register(&I2cyHandle, 2 * I2C_LED_ADDRESS, 0x07, 0xF0); // LED 5
    HAL_Delay(LED_OFF_SLEEP_DELAY);
    write_I2C_register(&I2cyHandle, 2 * I2C_LED_ADDRESS, 0x07, 0xC0); // LED 6
    HAL_Delay(LED_OFF_SLEEP_DELAY);
    write_I2C_register(&I2cyHandle, 2 * I2C_LED_ADDRESS, 0x07, 0x00); // LED 7
    HAL_Delay(LED_OFF_SLEEP_DELAY);
    write_I2C_register(&I2cyHandle, 2 * I2C_LED_ADDRESS, 0x08, 0xFC); // LED 8
    HAL_Delay(LED_OFF_SLEEP_DELAY);
    write_I2C_register(&I2cyHandle, 2 * I2C_LED_ADDRESS, 0x08, 0xF0); // LED 9
    HAL_Delay(LED_OFF_SLEEP_DELAY);
    write_I2C_register(&I2cyHandle, 2 * I2C_LED_ADDRESS, 0x08, 0xC0); // LED 10
    HAL_Delay(LED_OFF_SLEEP_DELAY);
    write_I2C_register(&I2cyHandle, 2 * I2C_LED_ADDRESS, 0x08, 0x00); // LED 11

#if DEBUG
    if (HAL_I2C_GetError(&I2cyHandle) == HAL_I2C_ERROR_AF) {
      printf("ERROR: LP3944 I2C FAILURE\r\n");
    }
#endif

    /* Read and report temperature and humidity data */
    int si_temp = 0, si_humidity = 0;
    int temp = 0, humidity = 0;
    uint8_t si_buf[6]; /* 6 is the max read bytes */
    /* Command: measure temperature and relative humidity, normal hold mode */
    si_buf[0] = 0x7C;
    si_buf[1] = 0xA2;

    HAL_I2C_Master_Transmit(&I2cxHandle, 2 * I2C_TEMP_ADDRESS, si_buf, 2, 100);
    HAL_I2C_Master_Receive(&I2cxHandle, 2 * I2C_TEMP_ADDRESS, si_buf, 6, 100);

    si_temp = (((int) si_buf[0]) << 8) + si_buf[1];
    si_humidity = (((int) si_buf[3]) << 8) + si_buf[4];
    /* Convert temperature to celsius */
    temp = (((si_temp * 21875) >> 13) - 45000) / 1000;
    /* Convert the value to percent relative humidity */
    humidity = ((si_humidity * 12500) >> 13) / 1000;
    if (humidity < 0) {
      humidity = 0;
    } else if (humidity > 100) {
      humidity = 100;
    }

    printf("SHUB: T %d H %d\r\n", temp, humidity);
  }
}


/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the EVAL_COM1 and Loop until the end of transmission */
  HAL_UART_Transmit(&UartHandle, (uint8_t *)&ch, 1, HAL_MAX_DELAY);

  return ch;
}

/**
 * @brief  Retargets the C library scanf function to the USART (GNU)
 * @param  None
 * @retval None
 */
int __io_getchar(void)
{
  /* Place your implementation of fgetc here */
  /* e.g. read a character from the USART */
  uint8_t ch;

  HAL_UART_Receive(&UartHandle, &ch, 1, HAL_MAX_DELAY);

  return ch;
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (HSI)
  *            SYSCLK(Hz)                     = 100000000
  *            HCLK(Hz)                       = 100000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 2
  *            APB2 Prescaler                 = 1
  *            HSI Frequency(Hz)              = 16000000
  *            PLL_M                          = 16
  *            PLL_N                          = 400
  *            PLL_P                          = 4
  *            PLL_Q                          = 7
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale2 mode
  *            Flash Latency(WS)              = 3
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();

  /* The voltage scaling allows optimizing the power consumption when the device is
     clocked below the maximum system frequency, to update the voltage scaling value
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /* Enable HSI Oscillator and activate PLL with HSI as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 0x10;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 400;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  while(1)
  {
  }
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

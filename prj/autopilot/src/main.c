/**
 * @file    main.c
 * @author  Raymond Oung
 * @date    2014.10.24
 * @brief   Main program
 *
 * MIT License
 *
 * Copyright (c) 2022 Raymond Oung
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h> // debug

#include "main.h"
#include "nvic_list.h"
#include "stm32f4xx.h"

#include "can.h"
#include "clock.h"
#include "i2c.h"
#include "iwdg.h"
#include "led.h"
#include "sdio.h"
#include "uart.h"

#include "24xx64.h"
#include "mpu9250.h"
#include "ms5607.h"
#include "sdcard.h"

/** @addtogroup Main
 * @{
 */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
void RCC_Configuration(void);
void GPIO_Configuration(void);
void NVIC_Configuration(void);

/**
 * @brief  Main program
 * @param  None
 * @retval None
 */
int main(void) {
  // At this stage the microcontroller clock setting is already configured,
  // this is done through SystemInit() function which is called from startup
  // file (startup_stm32f4xx.s) before to branch to application main.
  // To reconfigure the default setting of SystemInit() function, refer to
  // system_stm32f4xx.c file

  //========================================================================
  // INITIALISE PERIPHERALS
  //========================================================================
  RCC_Configuration();
  GPIO_Configuration();
  NVIC_Configuration();

  CLOCK_Init();

  I2C_ResetInit(I2C_RESET_CLK, I2C_RESET_PORT, I2C_RESET_PIN);
  Delay(100);
  I2C_On();

  LED_Init(LED1_CLK, LED1_PORT, LED1_PIN); // blue
  LED_Init(LED2_CLK, LED2_PORT, LED2_PIN); // red
  LED_Init(LED3_CLK, LED3_PORT, LED3_PIN); // green
  LED_Init(LED4_CLK, LED4_PORT, LED4_PIN); // yellow
  LED_OnAll();
  Delay(500);
  LED_OffAll();

  UART_PeriphInit(UART1, UART1_BAUD, UART1_DMA_BUF_SIZE, UART1_TX_CLK,
                  UART1_TX_PORT, UART1_TX_PIN, UART1_TX_PIN_SRC, UART1_RX_CLK,
                  UART1_RX_PORT, UART1_RX_PIN, UART1_RX_PIN_SRC,
                  UART1_DMA_RX_NVIC_PREEMPTION_PRIORITY,
                  UART1_DMA_TX_NVIC_PREEMPTION_PRIORITY,
                  UART1_IT_RX_NVIC_PREEMPTION_PRIORITY,
                  UART1_IT_TX_NVIC_PREEMPTION_PRIORITY);

  UART_PeriphInit(UART4, UART4_BAUD, UART4_DMA_BUF_SIZE, UART4_TX_CLK,
                  UART4_TX_PORT, UART4_TX_PIN, UART4_TX_PIN_SRC, UART4_RX_CLK,
                  UART4_RX_PORT, UART4_RX_PIN, UART4_RX_PIN_SRC,
                  UART4_DMA_RX_NVIC_PREEMPTION_PRIORITY,
                  UART4_DMA_TX_NVIC_PREEMPTION_PRIORITY,
                  UART4_IT_RX_NVIC_PREEMPTION_PRIORITY,
                  UART4_IT_TX_NVIC_PREEMPTION_PRIORITY);

  I2C_PeriphInit(I2C1, I2C1_SPEED, I2C1_SDA_CLK, I2C1_SDA_PORT, I2C1_SDA_PIN,
                 I2C1_SDA_PIN_SOURCE, I2C1_SCL_CLK, I2C1_SCL_PORT, I2C1_SCL_PIN,
                 I2C1_SCL_PIN_SOURCE, I2C1_NVIC_PREEMPTION_PRIORITY);

  I2C_PeriphInit(I2C2, I2C2_SPEED, I2C2_SDA_CLK, I2C2_SDA_PORT, I2C2_SDA_PIN,
                 I2C2_SDA_PIN_SOURCE, I2C2_SCL_CLK, I2C2_SCL_PORT, I2C2_SCL_PIN,
                 I2C2_SCL_PIN_SOURCE, I2C2_NVIC_PREEMPTION_PRIORITY);

  SDIO_PeriphInit(SDIO_D0_CLK, SDIO_D0_PORT, SDIO_D0_PIN, SDIO_D0_PIN_SRC,
                  SDIO_D1_CLK, SDIO_D1_PORT, SDIO_D1_PIN, SDIO_D1_PIN_SRC,
                  SDIO_D2_CLK, SDIO_D2_PORT, SDIO_D2_PIN, SDIO_D2_PIN_SRC,
                  SDIO_D3_CLK, SDIO_D3_PORT, SDIO_D3_PIN, SDIO_D3_PIN_SRC,
                  SDIO_CMD_CLK, SDIO_CMD_PORT, SDIO_CMD_PIN, SDIO_CMD_PIN_SRC,
                  SDIO_CLK_CLK, SDIO_CLK_PORT, SDIO_CLK_PIN, SDIO_CLK_PIN_SRC,
                  SDIO_IT_NVIC_PREEMPTION_PRIORITY,
                  SDIO_DMA_NVIC_PREEMPTION_PRIORITY);

  CAN_PeriphInit(CAN1, CAN1_FILT_NUM, CAN1_GPIO_CLK, CAN1_CLK, CAN1_TX_PORT,
                 CAN1_TX_PIN, CAN1_TX_PIN_SOURCE, CAN1_RX_PORT, CAN1_RX_PIN,
                 CAN1_RX_PIN_SOURCE, CAN1_RX_IRQn,
                 CAN1_RX_NVIC_PREEMPTION_PRIORITY, CAN1_TX_IRQn,
                 CAN1_TX_NVIC_PREEMPTION_PRIORITY);

  CAN_PeriphInit(CAN2, CAN2_FILT_NUM, CAN2_GPIO_CLK, CAN2_CLK, CAN2_TX_PORT,
                 CAN2_TX_PIN, CAN2_TX_PIN_SOURCE, CAN2_RX_PORT, CAN2_RX_PIN,
                 CAN2_RX_PIN_SOURCE, CAN2_RX_IRQn,
                 CAN2_RX_NVIC_PREEMPTION_PRIORITY, CAN2_TX_IRQn,
                 CAN2_TX_NVIC_PREEMPTION_PRIORITY);

  //========================================================================
  // INTERNAL WATCHDOG RESET
  //========================================================================
#if 0
	if (IWDG_Resume()) 
	{
		LED_OnAll();

		// perform some action here
		UART_Print(1,"ERROR: Internal watchdog reset\r\n");
		while(1);
	}
#endif

  //========================================================================
  // INITIALISE MEMORY
  //========================================================================
  EEPROM_Init(EEPROM_I2C, EEPROM_ADDRESS);
  SD_Card_Init();
  SD_Card_OpenFile("log.dat");

  uint8_t data[4] = {1u, 2u, 3u, 4u};
  uint8_t read[4] = {0u, 0u, 0u, 0u};
  EEPROM_Write(0x00, data, 4u);

  //========================================================================
  // INITIALISE SENSORS
  //========================================================================
  MS5607_Init(MS5607_I2C, MS5607_PRES_OSR_4096, MS5607_TEMP_OSR_4096);
  MPU9250_Init(MPU9250_I2C, MPU9250_GYRO_BW_184, MPU9250_GYRO_RANGE_250,
               MPU9250_ACCL_BW_184, MPU9250_ACCL_RANGE_2,
               MPU9250_MAGN_RATE_100);

  //========================================================================
  // INITIALISE INTERNAL WATCHDOG
  //========================================================================
#if 0
	IWDG_Init(100u);
#endif

  //========================================================================
  // MAIN LOOP
  //========================================================================
  while (1) {
    LED_Toggle(2);

    MPU9250_ReadSensor();
    MS5607_ReadSensor();
    EEPROM_Read(0x00, read, 4);

    char str[100];
    sprintf(str,
            "%.3f %.3f %.3f | %.3f %.3f %.3f | %.3f %.3f | %u %u %u %u\r\n",
            MPU9250_GetGyroX(), MPU9250_GetGyroY(), MPU9250_GetGyroZ(),
            MPU9250_GetAcclX(), MPU9250_GetAcclY(), MPU9250_GetAcclZ(),
            MS5607_GetPressure(), MS5607_GetTemperature(), read[0], read[1],
            read[2], read[3]);
    UART_Print(1, str);

    LED_On(1);
    SD_Card_Write(str, strlen(str)); // this is a blocking function; avoid
                                     // using this in an ISR()
    SD_Card_Sync();
    LED_Off(1);

    uint8_t out[8] = {1, 2, 3, 4, 5, 6, 7, 8};
    CAN_Write(1, 0x00, &out, 8);
    CAN_Write(2, 0x00, &out, 8);

    // IWDG_ReloadCounter();
    Delay(100);
  }

  UART_Print(1, "ERROR: Failed to initialise\r\n");
  Delay(100);

  //========================================================================
  // CLEAN-UP
  //========================================================================
  RCC_DeInit();
  GPIO_DeInit(GPIOA);
  GPIO_DeInit(GPIOB);
  GPIO_DeInit(GPIOC);
  GPIO_DeInit(GPIOD);
  SYSCFG_DeInit();
  return 0;
}

/**
 * @brief  Configures the different system clocks.
 * @param  None
 * @retval None
 */
void RCC_Configuration(void) {
  // Setup the microcontroller system. Initialize the Embedded Flash Interface,
  // initialize the PLL and update the System Frequency variable.
  SystemInit();

  // Peripheral Clock 1 (PCLK1 = APB1) (168 MHz Max.):
  RCC_PCLK1Config(RCC_HCLK_Div4); // set to 42 MHz

  // Peripheral Clock 2 (PCLK2 = APB2) (168 MHz Max.):
  RCC_PCLK2Config(RCC_HCLK_Div4); // set to 42 MHz
}

/**
 * @brief  Configures the different GPIO ports.
 * @param  None
 * @retval None
 */
void GPIO_Configuration(void) {
  GPIO_InitTypeDef GPIO_InitStructure;

  // Configure all unused GPIO port pins in Analog Input mode (floating input
  // trigger OFF), this will reduce the power consumption and increase the
  // device immunity against EMI/EMC
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;

  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  GPIO_Init(GPIOD, &GPIO_InitStructure);
}

/**
 * @brief  Configures the nested vectored interrupt controller.
 * @param  None
 * @retval None
 */
void NVIC_Configuration(void) {
  // Set Priority Group
  NVIC_PriorityGroupConfig(NVIC_PRIORITY_GROUP);
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line) {
  // User can add his own implementation to report the file name and line
  // number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
  // line)

  // Infinite loop
  while (1)
    ;
}
#endif
/**
 * @}
 */

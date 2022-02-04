/**
 * @file    main.c
 * @author  Raymond Oung
 * @date    2014.05.06
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
#include <string.h>

#include "main.h"
#include "nvic_list.h"
#include "stm32f4xx.h"

#include "can.h"
#include "clock.h"
#include "led.h"
#include "sdio.h"
#include "uart.h"

#include "circular_buffer.h"
#include "sdcard.h"

/** @addtogroup Main
 * @{
 */

/* Public typedef ------------------------------------------------------------*/
CircularBuffer_t canWriteBuf;
CircularBuffer_t uartWriteBuf;

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define BUFFER_SIZE 65532u // [bytes]
#define SYNC_PERIOD 2000u  // [msec]

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
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
  // BUFFER INITIALIZATION
  //========================================================================
  CB_Init(&canWriteBuf, BUFFER_SIZE);  // CAN-bus SD write buffer
  CB_Init(&uartWriteBuf, BUFFER_SIZE); // UART SD write buffer

  //========================================================================
  // PERIPHERAL INITIALIZATION
  //========================================================================
  RCC_Configuration();
  GPIO_Configuration();
  NVIC_Configuration();

  CLOCK_Init();
  LED_Init(LED1_CLK, LED1_PORT, LED1_PIN);
  LED_Init(LED2_CLK, LED2_PORT, LED2_PIN);
  LED_OnAll();
  Delay(3000u);
  LED_OffAll();

  UART_PeriphInit(UART1, UART1_BAUD, UART1_DMA_BUF_SIZE, UART1_TX_CLK,
                  UART1_TX_PORT, UART1_TX_PIN, UART1_TX_PIN_SRC, UART1_RX_CLK,
                  UART1_RX_PORT, UART1_RX_PIN, UART1_RX_PIN_SRC,
                  UART1_DMA_RX_NVIC_PREEMPTION_PRIORITY,
                  UART1_DMA_TX_NVIC_PREEMPTION_PRIORITY,
                  UART1_IT_RX_NVIC_PREEMPTION_PRIORITY,
                  UART1_IT_TX_NVIC_PREEMPTION_PRIORITY);

  UART_PeriphInit(UART2, UART2_BAUD, UART2_DMA_BUF_SIZE, UART2_TX_CLK,
                  UART2_TX_PORT, UART2_TX_PIN, UART2_TX_PIN_SRC, UART2_RX_CLK,
                  UART2_RX_PORT, UART2_RX_PIN, UART2_RX_PIN_SRC,
                  UART2_DMA_RX_NVIC_PREEMPTION_PRIORITY,
                  UART2_DMA_TX_NVIC_PREEMPTION_PRIORITY,
                  UART2_IT_RX_NVIC_PREEMPTION_PRIORITY,
                  UART2_IT_TX_NVIC_PREEMPTION_PRIORITY);

  UART_PeriphInit(UART4, UART4_BAUD, UART4_DMA_BUF_SIZE, UART4_TX_CLK,
                  UART4_TX_PORT, UART4_TX_PIN, UART4_TX_PIN_SRC, UART4_RX_CLK,
                  UART4_RX_PORT, UART4_RX_PIN, UART4_RX_PIN_SRC,
                  UART4_DMA_RX_NVIC_PREEMPTION_PRIORITY,
                  UART4_DMA_TX_NVIC_PREEMPTION_PRIORITY,
                  UART4_IT_RX_NVIC_PREEMPTION_PRIORITY,
                  UART4_IT_TX_NVIC_PREEMPTION_PRIORITY);

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
  // SD CARD INITIALIZATION
  //========================================================================
  SDIO_PeriphInit(SDIO_D0_CLK, SDIO_D0_PORT, SDIO_D0_PIN, SDIO_D0_PIN_SRC,
                  SDIO_D1_CLK, SDIO_D1_PORT, SDIO_D1_PIN, SDIO_D1_PIN_SRC,
                  SDIO_D2_CLK, SDIO_D2_PORT, SDIO_D2_PIN, SDIO_D2_PIN_SRC,
                  SDIO_D3_CLK, SDIO_D3_PORT, SDIO_D3_PIN, SDIO_D3_PIN_SRC,
                  SDIO_CMD_CLK, SDIO_CMD_PORT, SDIO_CMD_PIN, SDIO_CMD_PIN_SRC,
                  SDIO_CLK_CLK, SDIO_CLK_PORT, SDIO_CLK_PIN, SDIO_CLK_PIN_SRC,
                  SDIO_IT_NVIC_PREEMPTION_PRIORITY,
                  SDIO_DMA_NVIC_PREEMPTION_PRIORITY);

  if (!SD_Card_Init())
    return 0;
  SD_Card_PrintInfo();
  if (!SD_Card_OpenFile("log.bin"))
    return 0;
  if (!SD_Card_Sync())
    return 0;

  //========================================================================
  // MAIN PROGRAM -- WRITE DATA TO SDCARD AND SYNCHRONIZE
  //========================================================================
  char out[100u];
  sprintf(out, "Logging Started...\r\n");
  UART_Print(VERBOSE_UART_ID, out);

  uint32_t t0 = GetTime();
  while (1) {
    LED_Off(LED_RED);

    // write data from CAN buffer to SD card
    if (!CB_IsEmpty(&canWriteBuf)) {
      uint8_t byte = 0x00;
      LED_Blink(LED_GREEN, 100u);
      CB_Read(&canWriteBuf, &byte);
      SD_Card_Write(&byte, 1u);
    }

    // write data from UART buffer to SD card
    if (!CB_IsEmpty(&uartWriteBuf)) {
      uint8_t byte = 0x00;
      LED_Blink(LED_GREEN, 100u);
      CB_Read(&uartWriteBuf, &byte);
      char str[10];
      sprintf(str, "%c", byte);
      SD_Card_Write((uint8_t *)&str[0], 1u);
      // UART_Print(VERBOSE_UART_ID, str); // debug
      // SD_Card_Write(&byte, 1u);
    }

    // sync SD card
    if (GetTime() - t0 > SYNC_PERIOD) {
      t0 = GetTime();
      LED_On(LED_RED);
      SD_Card_Sync();
    }
  }
}

/**
 * @brief  Configures the different system clocks.
 * @param  None
 * @retval None
 */
void RCC_Configuration(void) {
  /* Setup the microcontroller system. Initialize the Embedded Flash Interface,
     initialize the PLL and update the System Frequency variable. */
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

  /* Configure all unused GPIO port pins in Analog Input mode (floating input
     trigger OFF), this will reduce the power consumption and increase the
     device immunity against EMI/EMC */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;

  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  GPIO_Init(GPIOD, &GPIO_InitStructure);
  GPIO_Init(GPIOE, &GPIO_InitStructure);
}

/**
 * @brief  Configures the nested vectored interrupt controller.
 * @param  None
 * @retval None
 */
void NVIC_Configuration(void) {
  /* Set Priority Group */
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
  /* User can add his own implementation to report the file name and line
     number,
     e.g. printf("Wrong parameters value: file %s on line %d\r\n", file, line)
   */

  /* Infinite loop */
  while (1)
    ;
}
#endif
/**
 * @}
 */

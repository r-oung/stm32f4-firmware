/**
 * @file    main.c
 * @author  Raymond Oung
 * @date    2014.03.19
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

#include "clock.h"
#include "led.h"
#include "sdio.h"
#include "uart.h"

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
  /*!< At this stage the microcontroller clock setting is already configured,
          this is done through SystemInit() function which is called from
     startup file (startup_stm32f4xx.s) before to branch to application main. To
     reconfigure the default setting of SystemInit() function, refer to
          system_stm32f4xx.c file
          */
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
  Delay(1000);
  LED_OffAll();

  UART_PeriphInit(UART1, UART1_BAUD, UART1_DMA_BUF_SIZE, UART1_TX_CLK,
                  UART1_TX_PORT, UART1_TX_PIN, UART1_TX_PIN_SRC, UART1_RX_CLK,
                  UART1_RX_PORT, UART1_RX_PIN, UART1_RX_PIN_SRC,
                  UART1_DMA_RX_NVIC_PREEMPTION_PRIORITY,
                  UART1_DMA_TX_NVIC_PREEMPTION_PRIORITY,
                  UART1_IT_RX_NVIC_PREEMPTION_PRIORITY,
                  UART1_IT_TX_NVIC_PREEMPTION_PRIORITY);

  SDIO_PeriphInit(SDIO_D0_CLK, SDIO_D0_PORT, SDIO_D0_PIN, SDIO_D0_PIN_SRC,
                  SDIO_D1_CLK, SDIO_D1_PORT, SDIO_D1_PIN, SDIO_D1_PIN_SRC,
                  SDIO_D2_CLK, SDIO_D2_PORT, SDIO_D2_PIN, SDIO_D2_PIN_SRC,
                  SDIO_D3_CLK, SDIO_D3_PORT, SDIO_D3_PIN, SDIO_D3_PIN_SRC,
                  SDIO_CMD_CLK, SDIO_CMD_PORT, SDIO_CMD_PIN, SDIO_CMD_PIN_SRC,
                  SDIO_CLK_CLK, SDIO_CLK_PORT, SDIO_CLK_PIN, SDIO_CLK_PIN_SRC,
                  SDIO_IT_NVIC_PREEMPTION_PRIORITY,
                  SDIO_DMA_NVIC_PREEMPTION_PRIORITY);

  //========================================================================
  // SD CARD
  //========================================================================
  SD_Card_Init();
  SD_Card_PrintInfo();
  SD_Card_OpenFile("test.dat");
  SD_Card_Sync();
  Delay(3000);

  uint32_t id = 0u;
  uint16_t cnt = 0u;
  while (1) {
    LED_Blink(1, 100);

    char str[100];
    sprintf(str, "%.3f %u\r\n", GetTimeU(), id++);

    float t0 = GetTimeU();
    SD_Card_Write(
        str,
        strlen(
            str)); // this is a blocking function; avoid adding this to an ISR()
    cnt += strlen(str);

    if (cnt > 8096) {
      cnt = 0u;
      SD_Card_Sync();
      sprintf(str, "%.3f: sync-delta: %.3f\r\n", GetTimeU(), GetTimeU() - t0);
      UART_Print(1, str);
    } else {
      sprintf(str, "%.3f:\r\n", GetTimeU());
      UART_Print(1, str);
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
 number, e.g. printf("Wrong parameters value: file %s on line %d\r\n", file,
 line) */

  /* Infinite loop */
  while (1)
    ;
}
#endif
/**
 * @}
 */

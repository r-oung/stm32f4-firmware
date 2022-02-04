/**
 * @file    iwdg.c
 * @author  Raymond Oung
 * @date    2009.11.03
 * @brief   Internal watchdog functions
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
#include "iwdg.h"

#ifdef _VERBOSE
#include "uart.h"
#include <stdio.h>
#define VERBOSE_UART_ID 1u
#endif

/** @addtogroup Source
 * @{
 */

/** @addtogroup Low_Level
 * @{
 */

/** @defgroup IWDG_Private_Variables
 * @{
 */
__IO uint32_t uwTimingDelay = 0;
__IO uint32_t uwCaptureNumber = 0;
__IO uint32_t uwPeriodValue = 0;
uint16_t tmpCC4[2] = {0, 0};
/**
 * @}
 */

/** @defgroup IWDG_Private_Function_Prototypes
 * @{
 */
static uint32_t IWDG_GetLSIFrequency(void);
/**
 * @}
 */

/**
 * @brief  Initialises internal watchdog
 * @param  timeout Timeout [msec]
 * @retval None
 */
void IWDG_Init(uint16_t timeout) {
  // Get the LSI frequency:  TIM5 is used to measure the LSI frequency
  uint32_t lsiFreq = IWDG_GetLSIFrequency();

#ifdef _VERBOSE
  char str[100];
  sprintf(str, "LSI Frequency: %lu Hz\r\n", lsiFreq);
  UART_Print(VERBOSE_UART_ID, str);
#endif

  // IWDG timeout equal to 250 ms (the timeout may vary depending on LSI
  // frequency precision) Enable write access to IWDG_PR and IWDG_RLR registers
  IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);

  // IWDG counter clock: LSI/32
  IWDG_SetPrescaler(IWDG_Prescaler_32);

  // Set counter reload value to obtain IWDG TimeOut
  // IWDG counter clock Frequency = lsiFreq/32
  // Counter Reload Value = timeout/IWDG counter clock Period
  //                      = (timeout/1000.0f) / (32/lsiFreq)
  IWDG_SetReload((uint16_t)((timeout / 1000.0f) / (32.0f / lsiFreq)));

  // Reload IWDG counter
  IWDG_ReloadCounter();

  // Enable IWDG (the LSI oscillator will be enabled by hardware)
  IWDG_Enable();
}

/**
 * @brief  Check if the system has resumed from IWDG reset
 * @param  None
 * @retval 1=Yes, 0=No
 */
uint8_t IWDG_Resume(void) {
  if (RCC_GetFlagStatus(RCC_FLAG_IWDGRST) == SET) {
    RCC_ClearFlag();
    return 1u;
  }

  return 0u;
}

/**
 * @brief  Configures TIM5 to measure the LSI oscillator frequency.
 * @param  None
 * @retval LSI Frequency
 */
static uint32_t IWDG_GetLSIFrequency(void) {
  NVIC_InitTypeDef NVIC_InitStructure;
  TIM_ICInitTypeDef TIM_ICInitStructure;
  RCC_ClocksTypeDef RCC_ClockFreq;

  // Enable the LSI oscillator
  RCC_LSICmd(ENABLE);
  while (RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET)
    ;

  // TIM5 configuration
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
  TIM_RemapConfig(TIM5, TIM5_LSI);
  TIM_PrescalerConfig(TIM5, 0u, TIM_PSCReloadMode_Immediate);

  // TIM5 configuration: Input Capture mode
  // The LSI oscillator is connected to TIM5 CH4
  // The Rising edge is used as active edge,
  // The TIM5 CCR4 is used to compute the frequency value
  TIM_ICInitStructure.TIM_Channel = TIM_Channel_4;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV8;
  TIM_ICInitStructure.TIM_ICFilter = 0u;
  TIM_ICInit(TIM5, &TIM_ICInitStructure);

  // Enable TIM5 Interrupt channel
  NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  // Enable TIM5 counter
  TIM_Cmd(TIM5, ENABLE);

  // Reset the flags
  TIM5->SR = 0u;

  // Enable the CC4 Interrupt Request
  TIM_ITConfig(TIM5, TIM_IT_CC4, ENABLE);

  // Wait until the TIM5 get 2 LSI edges (refer to TIM5_IRQHandler() in
  // stm32f4xx_it.c file)
  while (uwCaptureNumber != 2u)
    ;

  // Deinitialize the TIM5 peripheral registers to their default reset values
  TIM_DeInit(TIM5);

  // Compute the LSI frequency, depending on TIM5 input clock frequency (PCLK1)
  // Get SYSCLK, HCLK and PCLKx frequency
  RCC_GetClocksFreq(&RCC_ClockFreq);

  // Get PCLK1 prescaler
  if ((RCC->CFGR & RCC_CFGR_PPRE1) == 0u) {
    // PCLK1 prescaler equal to 1 => TIMCLK = PCLK1
    return ((RCC_ClockFreq.PCLK1_Frequency / uwPeriodValue) * 8u);
  } else {
    // PCLK1 prescaler different from 1 => TIMCLK = 2 * PCLK1
    return (((2u * RCC_ClockFreq.PCLK1_Frequency) / uwPeriodValue) * 8u);
  }
}

/**
 * @brief  ISR for capturing LSI value
 * @param  None
 * @retval None
 */
void IWDG_TIM5_ISR(void) {
  if (TIM_GetITStatus(TIM5, TIM_IT_CC4) != RESET) {
    /* Get the Input Capture value */
    tmpCC4[uwCaptureNumber++] = TIM_GetCapture4(TIM5);

    /* Clear CC4 Interrupt pending bit */
    TIM_ClearITPendingBit(TIM5, TIM_IT_CC4);

    if (uwCaptureNumber >= 2) {
      /* Compute the period length */
      uwPeriodValue = (uint16_t)(0xFFFF - tmpCC4[0] + tmpCC4[1] + 1);
    }
  }
}
/**
 * @}
 */

/**
 * @}
 */

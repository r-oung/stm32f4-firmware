/**
 * @file    stm32f4xx_it.c
 * @author  Raymond Oung
 * @date    2014.05.18
 * @brief   Interrupt service routines
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
#include "stm32f4xx_it.h"

#include "clock.h"
#include "neox.h"
#include "uart.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
 * @brief   This function handles NMI exception.
 * @param  None
 * @retval None
 */
void NMI_Handler(void) {}

/**
 * @brief  This function handles Hard Fault exception.
 * @param  None
 * @retval None
 */
void HardFault_Handler(void) {
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
    ;
}

/**
 * @brief  This function handles Memory Manage exception.
 * @param  None
 * @retval None
 */
void MemManage_Handler(void) {
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
    ;
}

/**
 * @brief  This function handles Bus Fault exception.
 * @param  None
 * @retval None
 */
void BusFault_Handler(void) {
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
    ;
}

/**
 * @brief  This function handles Usage Fault exception.
 * @param  None
 * @retval None
 */
void UsageFault_Handler(void) {
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
    ;
}

/**
 * @brief  This function handles SVCall exception.
 * @param  None
 * @retval None
 */
void SVC_Handler(void) {}

/**
 * @brief  This function handles Debug Monitor exception.
 * @param  None
 * @retval None
 */
void DebugMon_Handler(void) {}

/**
 * @brief  This function handles PendSVC exception.
 * @param  None
 * @retval None
 */
void PendSV_Handler(void) {}

/**
 * @brief  This function handles SysTick Handler.
 * @param  None
 * @retval None
 */
void SysTick_Handler(void) { SysTick_TimerValInc(); }

/******************************************************************************/
/*                 STM32F40x UART Interrupt Handlers                          */
/******************************************************************************/
/**
 * @brief  This function handles UART1 IT transmitter interrupt requests
 * @param  None
 * @retval None
 */
void UART1_IT_IRQHandler(void) { UART_TxISR_IT(1); }

/**
 * @brief  This function handles UART1 DMA transmitter interrupt requests
 * @param  None
 * @retval None
 */
void UART1_DMA_TxIRQHandler(void) { UART_TxISR_DMA(1); }

//------------------------------------------------------------------------

/**
 * @brief  This function handles UART2 IT transmitter interrupt requests
 * @param  None
 * @retval None
 */
void UART2_IT_IRQHandler(void) {
  if (UART_IT_GetRxITStatus(2) == SET) {
    NEOx_ISR_IT(USART_ReceiveData(USART2));
  }

  UART_TxISR_IT(2);
}

/**
 * @brief  This function handles UART2 DMA receiver interrupt requests
 * @param  None
 * @retval None
 */
void UART2_DMA_RxIRQHandler(void) {
  if (UART_DMA_GetRxITStatus(2) == SET) {
    NEOx_ISR_DMA(UART_GetDMABuffer(2));
    UART_DMA_ClearRxITPendingBit(2);
  }
}

/**
 * @brief  This function handles UART2 DMA transmitter interrupt requests
 * @param  None
 * @retval None
 */
void UART2_DMA_TxIRQHandler(void) { UART_TxISR_DMA(2); }

/**
 * @file    stm32f4xx_it.c
 * @author  Raymond Oung
 * @date    2014.10.24
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
#include <stdio.h> // debug

#include "can.h"
#include "clock.h"
#include "i2c.h"
#include "iwdg.h"
#include "led.h"
#include "sdio.h"
#include "uart.h"

#include "24xx64.h"
#include "sdsdio.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define VERBOSE_UART_ID 1u
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
void NMI_Handler(void) {
  // Go to infinite loop when NMI exception occurs
  UART_Print(VERBOSE_UART_ID, "ERROR: NMI Exeception\r\n");
  while (1)
    ;
}

/**
 * @brief  This function handles Hard Fault exception.
 * @param  None
 * @retval None
 */
void HardFault_Handler(void) {
  // Go to infinite loop when Hard Fault exception occurs
  UART_Print(VERBOSE_UART_ID, "ERROR: Hard Fault Exeception\r\n");
  while (1)
    ;
}

/**
 * @brief  This function handles Memory Manage exception.
 * @param  None
 * @retval None
 */
void MemManage_Handler(void) {
  // Go to infinite loop when Memory Manage exception occurs
  UART_Print(VERBOSE_UART_ID, "ERROR: Memory Manage Exeception\r\n");
  while (1)
    ;
}

/**
 * @brief  This function handles Bus Fault exception.
 * @param  None
 * @retval None
 */
void BusFault_Handler(void) {
  // Go to infinite loop when Bus Fault exception occurs
  UART_Print(VERBOSE_UART_ID, "ERROR: Bus Fault Exeception\r\n");
  while (1)
    ;
}

/**
 * @brief  This function handles Usage Fault exception.
 * @param  None
 * @retval None
 */
void UsageFault_Handler(void) {
  // Go to infinite loop when Usage Fault exception occurs
  UART_Print(VERBOSE_UART_ID, "ERROR: Usage Fault Exeception\r\n");
  while (1)
    ;
}

/**
 * @brief  This function handles SV Call exception.
 * @param  None
 * @retval None
 */
void SVC_Handler(void) {
  // Go to infinite loop when SV Call exception occurs
  UART_Print(VERBOSE_UART_ID, "ERROR: SV Call Exeception\r\n");
  while (1)
    ;
}

/**
 * @brief  This function handles Debug Monitor exception.
 * @param  None
 * @retval None
 */
void DebugMon_Handler(void) {
  // Go to infinite loop when Debug Monitor exception occurs
  UART_Print(VERBOSE_UART_ID, "ERROR: Debug Monitor Exeception\r\n");
  while (1)
    ;
}

/**
 * @brief  This function handles PendSVC exception.
 * @param  None
 * @retval None
 */
void PendSV_Handler(void) {
  // Go to infinite loop when PendSVC exception occurs
  UART_Print(VERBOSE_UART_ID, "ERROR: PendSVC Exeception\r\n");
  while (1)
    ;
}

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
 * @brief  This function handles UART4 IT transmitter interrupt requests
 * @param  None
 * @retval None
 */
void UART4_IT_IRQHandler(void) { UART_TxISR_IT(4); }

/**
 * @brief  This function handles UART4 DMA receiver interrupt requests
 * @param  None
 * @retval None
 */
void UART4_DMA_RxIRQHandler(void) {
  if (UART_DMA_GetRxITStatus(4) == SET) {
    UART_DMA_ClearRxITPendingBit(4);
  }
}

/**
 * @brief  This function handles UART4 DMA transmitter interrupt requests
 * @param  None
 * @retval None
 */
void UART4_DMA_TxIRQHandler(void) { UART_TxISR_DMA(4); }

/******************************************************************************/
/*                 STM32F40x I2C Interrupt Handlers                           */
/******************************************************************************/
/**
 * @brief  This function handles I2C1 interrupt requests
 * @param  None
 * @retval None
 */
void I2C1_IRQHandler(void) { EEPROM_WriteIT_ISR(); }

/**
 * @brief  This function handles I2C2 interrupt requests
 * @param  None
 * @retval None
 */
void I2C2_IRQHandler(void) {}

/******************************************************************************/
/*                 STM32F40x IWDG Interrupt Handlers                          */
/******************************************************************************/
/**
 * @brief  This function handles IWDG interrupt requests
 * @param  None
 * @retval None
 */
void IWDG_IRQHandler(void) { IWDG_TIM5_ISR(); }

/******************************************************************************/
/*                  STM32F40x CAN Interrupt Handlers                          */
/******************************************************************************/
/**
 * @brief  This function handles CAN1 RX global interrupt requests.
 * @param  None
 * @retval None
 */
void CAN1_RX_IRQHandler(void) {
  LED_Toggle(0);
  CanRxMsg RxMessage;
  CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);
  CAN_FIFORelease(CAN1, CAN_Filter_FIFO0);
}

/**
 * @brief  This function handles CAN1 TX global interrupt requests.
 * @param  None
 * @retval None
 */
void CAN1_TX_IRQHandler(void) {
  CAN_ClearITPendingBit(CAN1, CAN_IT_TME);
  CAN_TxISR(1);
}

/**
 * @brief  This function handles CAN2 RX global interrupt requests.
 * @param  None
 * @retval None
 */
void CAN2_RX_IRQHandler(void) {
  LED_Toggle(1);
  CanRxMsg RxMessage;
  CAN_Receive(CAN2, CAN_FIFO0, &RxMessage);
  CAN_FIFORelease(CAN2, CAN_Filter_FIFO0);
}

/**
 * @brief  This function handles CAN2 TX global interrupt requests.
 * @param  None
 * @retval None
 */
void CAN2_TX_IRQHandler(void) {
  CAN_ClearITPendingBit(CAN2, CAN_IT_TME);
  CAN_TxISR(2);
}

/******************************************************************************/
/*                 STM32F10x SDIO Interrupt Handlers                          */
/******************************************************************************/
/**
 * @brief  This function handles SDIO IT interrupt requests
 * @param  None
 * @retval None
 */
void SDIO_IT_IRQHandler(void) { SD_ProcessIRQSrc(); }

/**
 * @brief  This function handles SDIO DMA interrupt requests
 *         requests.
 * @param  None
 * @retval None
 */
void SDIO_DMA_IRQHandler(void) { SD_ProcessDMAIRQ(); }

/**
 * @file    stm32f4xx_it.c
 * @author  Raymond Oung
 * @date    2014.08.11
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
#include "main.h"

#include "can.h"
#include "clock.h"
#include "led.h"

#include "dw1000.h"
#include "range.h"

#ifdef _VERBOSE
#include "main.h"
#include "uart.h"
#include <stdio.h>
#include <string.h>
#endif

/* Private typedef -----------------------------------------------------------*/
#ifdef _VERBOSE
#ifdef _LNS_V03
#define VERBOSE_UART_ID 4u // LNS-v03
#else
#define VERBOSE_UART_ID 2u // DWM1000-v02
#endif

#endif

#define WATCHDOG_THRESHOLD 10u
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
void NMI_Handler(void) {
  while (1) {
#ifdef _VERBOSE
    UART_Print(VERBOSE_UART_ID, "ERROR: NMI Exception\r\n");
#endif
  }
}

/**
 * @brief  This function handles Hard Fault exception.
 * @param  None
 * @retval None
 */
void HardFault_Handler(void) {
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1) {
#ifdef _VERBOSE
    UART_Print(VERBOSE_UART_ID, "ERROR: Hard Fault\r\n");
#endif
  }
}

/**
 * @brief  This function handles Memory Manage exception.
 * @param  None
 * @retval None
 */
void MemManage_Handler(void) {
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1) {
#ifdef _VERBOSE
    UART_Print(VERBOSE_UART_ID, "ERROR: Memory Manage Expection\r\n");
#endif
  }
}

/**
 * @brief  This function handles Bus Fault exception.
 * @param  None
 * @retval None
 */
void BusFault_Handler(void) {
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1) {
#ifdef _VERBOSE
    UART_Print(VERBOSE_UART_ID, "ERROR: Bus Fault Exception\r\n");
#endif
  }
}

/**
 * @brief  This function handles Usage Fault exception.
 * @param  None
 * @retval None
 */
void UsageFault_Handler(void) {
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1) {
#ifdef _VERBOSE
    UART_Print(VERBOSE_UART_ID, "ERROR: Usage Fault Exception\r\n");
#endif
  }
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
void PendSV_Handler(void) {
  while (1) {
#ifdef _VERBOSE
    UART_Print(VERBOSE_UART_ID, "ERROR: PendSVC Exception\r\n");
#endif
  }
}

/**
 * @brief  This function handles SysTick Handler.
 * @param  None
 * @retval None
 */
void SysTick_Handler(void) { SysTick_TimerValInc(); }

/******************************************************************************/
/*                 STM32F40x EXTI Interrupt Handlers                          */
/******************************************************************************/
/**
 * @brief  This function handles External line 3 interrupt request.
 * @param  None
 * @retval None
 */
void EXTI3_IRQHandler(void) {
  if (EXTI_GetITStatus(EXTI_Line3) == SET) {
    EXTI_ClearITPendingBit(EXTI_Line3);

    uint8_t data[100];
    if (DW1000_Receive(data) > 0u) {
      // if a valid frame was received, process the data
      RANGE_Arbiter(data,
                    DW1000_GetRxTimestamp()); // send message data and timestamp
                                              // to be post-processed
    }
  }
}

/**
 * @brief  This function handles External line 5 to 9 interrupt request.
 * @param  None
 * @retval None
 */
void EXTI9_5_IRQHandler(void) {
  LED_Toggle(0);
  if (EXTI_GetITStatus(EXTI_Line5) == SET) {
    EXTI_ClearITPendingBit(EXTI_Line5);
  }

  if (EXTI_GetITStatus(EXTI_Line6) == SET) {
    EXTI_ClearITPendingBit(EXTI_Line6);
  }

  if (EXTI_GetITStatus(EXTI_Line7) == SET) {
    EXTI_ClearITPendingBit(EXTI_Line7);
  }

  if (EXTI_GetITStatus(EXTI_Line8) == SET) {
    EXTI_ClearITPendingBit(EXTI_Line8);
  }

  if (EXTI_GetITStatus(EXTI_Line9) == SET) {
    EXTI_ClearITPendingBit(EXTI_Line9);
  }
}

/**
 * @brief  This function handles External line 10 to 15 interrupt request.
 * @param  None
 * @retval None
 */
void EXTI15_10_IRQHandler(void) {
  LED_Toggle(0);
  while (1)
    ;
  if (EXTI_GetITStatus(EXTI_Line10) == SET) {
    EXTI_ClearITPendingBit(EXTI_Line10);
  }

  if (EXTI_GetITStatus(EXTI_Line11) == SET) {
    EXTI_ClearITPendingBit(EXTI_Line11);
  }

  if (EXTI_GetITStatus(EXTI_Line12) == SET) {
    EXTI_ClearITPendingBit(EXTI_Line12);

    uint8_t data[100];
    if (DW1000_Receive(data) > 0u) {
      // if a valid frame was received, process the data
      RANGE_Arbiter(data,
                    DW1000_GetRxTimestamp()); // send message data and timestamp
                                              // to be post-processed
    }
  }

  if (EXTI_GetITStatus(EXTI_Line13) == SET) {
    EXTI_ClearITPendingBit(EXTI_Line13);
  }

  if (EXTI_GetITStatus(EXTI_Line14) == SET) {
    EXTI_ClearITPendingBit(EXTI_Line14);
  }

  if (EXTI_GetITStatus(EXTI_Line15) == SET) {
    EXTI_ClearITPendingBit(EXTI_Line15);
  }
}

/******************************************************************************/
/*                 STM32F40x UART Interrupt Handlers                          */
/******************************************************************************/
/**
 * @brief  This function handles UART2 IT transmitter interrupt requests
 * @param  None
 * @retval None
 */
void UART2_IT_IRQHandler(void) { UART_TxISR_IT(2); }

/**
 * @brief  This function handles UART2 DMA transmitter interrupt requests
 * @param  None
 * @retval None
 */
void UART2_DMA_TxIRQHandler(void) { UART_TxISR_DMA(2); }

/**
 * @brief  This function handles UART2 DMA receiver interrupt requests
 * @param  None
 * @retval None
 */
void UART2_DMA_RxIRQHandler(void) {
  if (UART_DMA_GetRxITStatus(2) == SET) {
    UART_DMA_ClearRxITPendingBit(2);
  }
}

/**
 * @brief  This function handles UART4 IT transmitter interrupt requests
 * @param  None
 * @retval None
 */
void UART4_IT_IRQHandler(void) { UART_TxISR_IT(4); }

/**
 * @brief  This function handles UART4 DMA transmitter interrupt requests
 * @param  None
 * @retval None
 */
void UART4_DMA_TxIRQHandler(void) { UART_TxISR_DMA(4); }

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

/******************************************************************************/
/*                  STM32F40x CAN Interrupt Handlers                          */
/******************************************************************************/
/**
 * @brief  This function handles CAN1 RX global interrupt requests.
 * @param  None
 * @retval None
 */
void CAN1_RX_IRQHandler(void) {
  CanRxMsg RxMessage;

  LED_Toggle(0);
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

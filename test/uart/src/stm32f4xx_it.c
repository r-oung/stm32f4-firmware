/**
 * @file    stm32f4xx_it.c
 * @author  Raymond Oung
 * @date    2013.06.15
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
#include <stdio.h>

#include "clock.h"
#include "led.h"
#include "main.h"
#include "uart.h"

/* Private typedef -----------------------------------------------------------*/
typedef struct _pkt_t {
  uint8_t start;
  uint8_t dst;
  uint8_t type;
  uint8_t flags;
  uint8_t seq;
  float p;
  float q;
  float r;
  float t;
  uint32_t chk;
} __attribute__((__packed__)) pkt_t; // 25 bytes

typedef struct _pkt2_t {
  uint8_t start;
  uint8_t data[16];
} __attribute__((__packed__)) pkt2_t; // 17 bytes

/* Private define ------------------------------------------------------------*/
#define START_BYTE 'g'

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

/**
 * @brief  Parser
 * @param  byte
 * @retval None
 */
void parser(uint8_t byte) {
  static uint8_t first = 1u;
  static float ts0 = 0.0f;
  static float max = 0.0f;

  static uint8_t cnt = 0u;
  static uint8_t buffer[0xff];
  static uint32_t err = 0u;

  // copy byte to buffer
  buffer[cnt++] = byte;

  if (buffer[0] == START_BYTE) {
    if (cnt == sizeof(pkt_t)) {
      cnt = 0u; // reset counter

      pkt_t *pkt;
      pkt = (pkt_t *)buffer;

      LED_Toggle(3);

      if (first) {
        first = 0u;
        ts0 = GetTimeU();
      } else {
        float dt = GetTimeU() - ts0;
        ts0 = GetTimeU();
        if (max < dt) {
          max = dt;
        }
        char str[100];
        sprintf(str, "%f %f %f\r\n", GetTimeU(), dt, max);
        UART_Print(6, str);
      }

      // char str[100]; sprintf(str, "%f %f %u\r\n", GetTimeU(), pkt->t,
      // (unsigned int)err); UART_Print(6,str); char str[100]; sprintf(str, "%f
      // %u %u %u %u %u %f %f %f %f %u\r\n",
      //   GetTimeU(),
      //   pkt->start, pkt->dst, pkt->type, pkt->flags, pkt->seq,
      //   pkt->p, pkt->q, pkt->r, pkt->t,
      //   pkt->chk); UART_Print(6,str);
    }
  } else {
    cnt = 0u;
    err++;
  }
}

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
 * @brief  This function handles UART1 DMA receiver interrupt requests
 * @param  None
 * @retval None
 */
void UART1_DMA_RxIRQHandler(void) {
  if (UART_DMA_GetRxITStatus(1) == SET) {
    char str[100];
    sprintf(str, "%d\r\n", UART_GetDMABuffer(1)[0]);
    UART_Print(1, str);
    UART_DMA_ClearRxITPendingBit(1);
  }
}

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
    uint8_t i;
    for (i = 0; i < UART4_DMA_BUF_SIZE; i++) {
      parser(UART_GetDMABuffer(4)[i]);
    }

    UART_DMA_ClearRxITPendingBit(4);
  }
}

/**
 * @brief  This function handles UART4 DMA transmitter interrupt requests
 * @param  None
 * @retval None
 */
void UART4_DMA_TxIRQHandler(void) { UART_TxISR_DMA(4); }

//------------------------------------------------------------------------

/**
 * @brief  This function handles UART6 IT transmitter interrupt requests
 * @param  None
 * @retval None
 */
void UART6_IT_IRQHandler(void) { UART_TxISR_IT(6); }

/**
 * @brief  This function handles UART6 DMA receiver interrupt requests
 * @param  None
 * @retval None
 */
void UART6_DMA_RxIRQHandler(void) {
  if (UART_DMA_GetRxITStatus(6) == SET) {
    char str[100];
    sprintf(str, "%d\r\n", UART_GetDMABuffer(6)[0]);
    UART_Print(6, str);
    UART_DMA_ClearRxITPendingBit(6);
  }
}

/**
 * @brief  This function handles UART6 DMA transmitter interrupt requests
 * @param  None
 * @retval None
 */
void UART6_DMA_TxIRQHandler(void) { UART_TxISR_DMA(6); }

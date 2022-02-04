/**
 * @file    stm32f4xx_it.h
 * @author  Raymond Oung
 * @date    2014.10.30
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
#include <stdio.h>  // debug
#include <string.h> // debug

#include "stm32f4xx_it.h"

#include "can.h"
#include "clock.h"
#include "main.h"
#include "sdio.h"
#include "uart.h"

#include "circular_buffer.h"
#include "sdsdio.h"

extern CircularBuffer_t canWriteBuf;
extern CircularBuffer_t uartWriteBuf;

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define ECHO_CONSOLE

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
 * @brief  This function handles UART1 DMA receiver interrupt requests
 * @param  None
 * @retval None
 */
void UART1_DMA_RxIRQHandler(void) {
  if (UART_DMA_GetRxITStatus(1) == SET) {
    char str[10];
    sprintf(str, "%c", UART_GetDMABuffer(1)[0]);
    UART_Print(1, str);
    UART_DMA_ClearRxITPendingBit(1);
  }
}

/**
 * @brief  This function handles UART2 IT transmitter interrupt requests
 * @param  None
 * @retval None
 */
void UART2_IT_IRQHandler(void) { UART_TxISR_IT(2); }

/**
 * @brief  This function handles UART2 DMA receiver interrupt requests
 * @param  None
 * @retval None
 */
void UART2_DMA_RxIRQHandler(void) {
  if (UART_DMA_GetRxITStatus(2) == SET) {
#if 0
    char str[10]; sprintf(str, "%c", UART_GetDMABuffer(2)[0]); UART_Print(2,str);
#else
    // copy data to the buffer if there's sufficient space
    if (CB_FreeElems(&uartWriteBuf) >= UART2_DMA_BUF_SIZE) {
      uint8_t i;
      for (i = 0u; i < UART2_DMA_BUF_SIZE; i++) {
        char str[UART2_DMA_BUF_SIZE];
        sprintf(str, "%c", UART_GetDMABuffer(2)[i]);
        CB_Write(&uartWriteBuf, (uint8_t *)&str[i]);
      }
    } else {
      // ERROR: insufficient space
      char out[100u];
      sprintf(out, "[%lu] %u [free bytes]\r\n", GetTime(),
              (unsigned int)CB_FreeElems(&uartWriteBuf));
      UART_Print(VERBOSE_UART_ID, out);
    }
#endif
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
 * @brief  This function handles UART4 DMA receiver interrupt requests
 * @param  None
 * @retval None
 */
void UART4_DMA_RxIRQHandler(void) {
  if (UART_DMA_GetRxITStatus(4) == SET) {
#if 0
    char str[10]; sprintf(str, "%c", UART_GetDMABuffer(4)[0]); UART_Print(4,str);
#else
    // copy data to the buffer if there's sufficient space
    if (CB_FreeElems(&uartWriteBuf) >= UART4_DMA_BUF_SIZE) {
      uint8_t i;
      for (i = 0u; i < UART4_DMA_BUF_SIZE; i++) {
        // char str[10]; sprintf(str, "%c", UART_GetDMABuffer(4)[i]);
        // UART_Print(VERBOSE_UART_ID, str); // debug
        char str[UART4_DMA_BUF_SIZE];
        sprintf(str, "%c", UART_GetDMABuffer(4)[i]);
        CB_Write(&uartWriteBuf, (uint8_t *)&str[i]);
      }
    } else {
      // ERROR: insufficient space
      char out[100u];
      sprintf(out, "[%lu] %u [free bytes]\r\n", GetTime(),
              (unsigned int)CB_FreeElems(&uartWriteBuf));
      UART_Print(VERBOSE_UART_ID, out);
    }
#endif
    UART_DMA_ClearRxITPendingBit(4);
  }
}

/******************************************************************************/
/*                  STM32F10x CAN Interrupt Handlers                          */
/******************************************************************************/
// uint32_t StdId; // Specifies the standard identifier.
//                 // This parameter can be a value between 0 to 0x7FF.

// uint32_t ExtId; // Specifies the extended identifier.
//                 // This parameter can be a value between 0 to 0x1FFFFFFF.

// uint8_t IDE;     // Specifies the type of identifier for the message that
//                  // will be received. This parameter can be a value of
//                  // @ref CAN_identifier_type

// uint8_t RTR;     // Specifies the type of frame for the received message.
//                  // This parameter can be a value of
//                  // @ref CAN_remote_transmission_request

// uint8_t DLC;     // Specifies the length of the frame that will be received.
//                  // This parameter can be a value between 0 to 8

// uint8_t Data[8]; // Contains the data to be received. It ranges from 0 to
//                  // 0xFF.

// uint8_t FMI;     // Specifies the index of the filter the message stored in
//                  // the mailbox passes through. This parameter can be a
//                  // value between 0 to 0xFF

void write_uint8(const uint8_t val, uint8_t *const buf) { buf[0] = val; }

void write_uint32(const uint32_t val, uint8_t *const buf) {
  buf[0] = (val >> 0u) & 0xFF;
  buf[1] = (val >> 8u) & 0xFF;
  buf[2] = (val >> 16u) & 0xFF;
  buf[3] = (val >> 24u) & 0xFF;
}

void write2buf(CAN_TypeDef *CANx, CanRxMsg *msg) {
  uint8_t i;
  uint8_t buf[19u];

  // Timestamp (4 bytes)
  write_uint32(GetTime(), &buf[0]);

  // CAN bus number (1 byte)
  write_uint8(CANx == CAN1 ? 1 : (CANx == CAN2 ? 2 : 0), &buf[4]);

  // CAN ID type (1 byte) -- 0: Extended ID | 1: Standard ID
  write_uint8(msg->IDE == CAN_Id_Standard ? 1 : 0, &buf[5]);

  // CAN ID (4 bytes)
  write_uint32(msg->IDE == CAN_Id_Standard ? msg->StdId : msg->ExtId, &buf[6]);

  // CAN message data length (1 byte)
  write_uint8(msg->DLC, &buf[10]);

  // CAN data (8 bytes)
  for (i = 0u; i < 8u; i++)
    write_uint8(msg->Data[i], &buf[11 + i]);

  // copy data to the buffer if there's sufficient space
  if (CB_FreeElems(&canWriteBuf) >= 19u) {
    for (i = 0u; i < 19u; i++) {
      CB_Write(&canWriteBuf, &buf[i]);
    }
  } else {
    // ERROR: insufficient space
    char out[100u];
    sprintf(out, "[%lu] %u [free bytes]\r\n", GetTime(),
            (unsigned int)CB_FreeElems(&canWriteBuf));
    UART_Print(VERBOSE_UART_ID, out);
  }
}

/**
 * @brief  This function handles CAN1 RX global interrupt requests
 * @param  None
 * @retval None
 */
void CAN1_RX0_IRQHandler(void) {
  CanRxMsg RxMessage;
  CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);
  write2buf(CAN1, &RxMessage);
}

/**
 * @brief  This function handles CAN2 RX global interrupt requests
 * @param  None
 * @retval None
 */
void CAN2_RX0_IRQHandler(void) {
  CanRxMsg RxMessage;
  CAN_Receive(CAN2, CAN_FIFO0, &RxMessage);
  write2buf(CAN2, &RxMessage);
}

/**
 * @brief  This function handles CAN1 TX global interrupt requests
 * @param  None
 * @retval None
 */
void CAN1_TX_IRQHandler(void) {
  CAN_ClearITPendingBit(CAN1, CAN_IT_TME);
  CAN_TxISR(1);
}

/**
 * @brief  This function handles CAN2 TX global interrupt requests
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

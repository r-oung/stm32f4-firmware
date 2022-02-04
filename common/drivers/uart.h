/**
 * @file    uart.h
 * @author  Raymond Oung
 * @date    2009.10.29
 * @brief   UART interface functions
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __UART_H
#define __UART_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"

/** @addtogroup Source
 * @{
 */

/** @addtogroup Low_Level
 * @{
 */

/** @addtogroup UART
 * @{
 */

/** @defgroup UART_Defines
 * @{
 */
#define UART1 USART1
#define UART2 USART2
#define UART3 USART3
#define UART6 USART6

#define UART1_IT_IRQHandler USART1_IRQHandler
#define UART2_IT_IRQHandler USART2_IRQHandler
#define UART3_IT_IRQHandler USART3_IRQHandler
#define UART4_IT_IRQHandler UART4_IRQHandler
#define UART5_IT_IRQHandler UART5_IRQHandler
#define UART6_IT_IRQHandler USART6_IRQHandler

#define UART1_DMA_RxIRQHandler DMA2_Stream5_IRQHandler
#define UART2_DMA_RxIRQHandler DMA1_Stream5_IRQHandler
#define UART3_DMA_RxIRQHandler DMA1_Stream1_IRQHandler
#define UART4_DMA_RxIRQHandler DMA1_Stream2_IRQHandler
#define UART5_DMA_RxIRQHandler DMA1_Stream0_IRQHandler
#define UART6_DMA_RxIRQHandler DMA2_Stream2_IRQHandler

#define UART1_DMA_TxIRQHandler DMA2_Stream7_IRQHandler
#define UART2_DMA_TxIRQHandler DMA1_Stream6_IRQHandler
#define UART3_DMA_TxIRQHandler DMA1_Stream3_IRQHandler
#define UART4_DMA_TxIRQHandler DMA1_Stream4_IRQHandler
#define UART5_DMA_TxIRQHandler DMA1_Stream7_IRQHandler
#define UART6_DMA_TxIRQHandler DMA2_Stream6_IRQHandler
/**
 * @}
 */

/** @defgroup UART_Exported_Functions
 * @{
 */
void UART_PeriphInit(USART_TypeDef *UARTx, uint32_t baud,
                     uint16_t DMA_bufferSize, uint32_t tx_clk,
                     GPIO_TypeDef *tx_port, uint16_t tx_pin, uint8_t tx_pin_src,
                     uint32_t rx_clk, GPIO_TypeDef *rx_port, uint16_t rx_pin,
                     uint8_t rx_pin_src, uint8_t rxDMA_PremptPriority,
                     uint8_t txDMA_PremptPriority, uint8_t rxIT_PremptPriority,
                     uint8_t txIT_PremptPriority);

void UART_SetBaud(uint8_t p, uint32_t baud);
FlagStatus UART_GetFlagStatus(uint8_t p, uint16_t UART_FLAG);
uint16_t UART_ReceiveData(uint8_t p);
void UART_SendData(uint8_t p, uint16_t data);

void UART_IT_RxIRQ(uint8_t p, FunctionalState NewState);
void UART_IT_TxIRQ(uint8_t p, FunctionalState NewState);
FlagStatus UART_IT_GetRxITStatus(uint8_t p);
FlagStatus UART_IT_GetTxITStatus(uint8_t p);
FlagStatus UART_DMA_GetRxITStatus(uint8_t p);
FlagStatus UART_DMA_GetTxITStatus(uint8_t p);
void UART_DMA_ClearRxITPendingBit(uint8_t p);
void UART_DMA_ClearTxITPendingBit(uint8_t p);

void UART_DMA_RxIRQ(uint8_t p, FunctionalState NewState);
void UART_DMA_TxIRQ(uint8_t p, FunctionalState NewState);
void UART_DMA_RxModeConfig(uint8_t p, uint32_t mode);
void UART_DMA_TxModeConfig(uint8_t p, uint32_t mode);
void UART_DMA_RxBufferSizeConfig(uint8_t p, uint16_t size);
void UART_DMA_TxBufferSizeConfig(uint8_t p, uint16_t size);

void UART_TxISR_IT(uint8_t p);
void UART_TxISR_DMA(uint8_t p);

uint8_t *UART_GetDMABuffer(uint8_t p);
uint8_t UART_Write_IT(uint8_t p, void *data, uint16_t size);
uint8_t UART_Write_DMA(uint8_t p, void *data, uint16_t size);

void UART_Print(uint8_t p, void *str);
/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* __UART_H */

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

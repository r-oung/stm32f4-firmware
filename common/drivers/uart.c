/**
 * @file    uart.c
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

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "circular_buffer.h"
#include "double_buffer.h"
#include "uart.h"

/** @addtogroup Source
 * @{
 */

/** @addtogroup Low_Level
 * @{
 */

/** @addtogroup UART
 * @{
 */

/** @defgroup UART
 * @brief UART setup
 * @{
 */

/** @defgroup UART_Private_Variables
 * @{
 */
volatile static uint8_t UART_DMA_txBuf0_flag[6];
volatile static uint8_t UART_DMA_txBuf1_flag[6];

static struct {
  USART_InitTypeDef init;
  DMA_InitTypeDef dmaInitTx;
  DMA_InitTypeDef dmaInitRx;
  DMA_Stream_TypeDef *streamRx;
  DMA_Stream_TypeDef *streamTx;
  uint32_t DMA_RxIT;
  uint32_t DMA_TxIT;
} UART_config[6];

static CircularBuffer_t UART_txBufIT[6]; // IT Transmit: circular buffer
static DoubleBuffer_t UART_txBufDMA[6];  // DMA Transmit: double buffer
static DoubleBuffer_t UART_rxBufDMA[6];  // DMA Receive: double buffer
/**
 * @}
 */

/** @defgroup UART_Private_Functions
 * @{
 */
static USART_TypeDef *UART_id2type(uint8_t p);
static uint8_t UART_id2buf(uint8_t id);
static uint8_t UART_type2buf(USART_TypeDef *t);
/**
 * @}
 */

/**
 * @brief  Initialises UART
 * 		   By default this will use a fixed-size DMA buffer for
 * reception; Transmission can be accomplished by IT and/or DMA
 * @modes  1 - Receive DMA+IT
 * 		   2 - Receive DMA
 * 		   3 - Transmit IT
 * 		   4 - Transmit DMA
 *
 * @param  UARTx         		UART peripheral type [USART1..UART6]
 * @param  baud 		 		Baud rate
 * @param  DMA_bufferSize 		Buffer size in bytes
 * @param  tx_clk        		Tx GPIO clock
 * @param  tx_port       		Tx GPIO port
 * @param  tx_pin        		Tx GPIO pin
 * @param  tx_pin_src    		Tx GPIO pin source
 * @param  rx_clk        		Rx GPIO clock
 * @param  rx_port       		Rx GPIO port
 * @param  rx_pin        		Rx GPIO pin
 * @param  rx_pin_src    		Rx GPIO pin source
 * @param  rxDMA_PremptPriority DMA Rx interrupt priority
 * @param  txDMA_PremptPriority DMA Tx interrupt priority
 * @param  rxIT_PremptPriority  IT Rx interrupt priority
 * @param  txIT_PremptPriority 	IT Rx interrupt priority
 * @retval None
 */
void UART_PeriphInit(USART_TypeDef *UARTx, uint32_t baud,
                     uint16_t DMA_bufferSize, uint32_t tx_clk,
                     GPIO_TypeDef *tx_port, uint16_t tx_pin, uint8_t tx_pin_src,
                     uint32_t rx_clk, GPIO_TypeDef *rx_port, uint16_t rx_pin,
                     uint8_t rx_pin_src, uint8_t rxDMA_PremptPriority,
                     uint8_t txDMA_PremptPriority, uint8_t rxIT_PremptPriority,
                     uint8_t txIT_PremptPriority) {
  uint8_t n = UART_type2buf(UARTx);

  // allocate memory
  CB_Init(&UART_txBufIT[n], 0xFF);            // IT transmit buffer
  DB_Init(&UART_txBufDMA[n], DMA_bufferSize); // DMA transmit buffer
  DB_Init(&UART_rxBufDMA[n], DMA_bufferSize); // DMA receive buffer

  RCC_AHB1PeriphClockCmd(tx_clk, ENABLE);
  RCC_AHB1PeriphClockCmd(rx_clk, ENABLE);

  if (UARTx == USART1)
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
  if (UARTx == USART2)
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
  if (UARTx == USART3)
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
  if (UARTx == UART4)
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
  if (UARTx == UART5)
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);
  if (UARTx == USART6)
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);

  if (UARTx == USART1)
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
  if (UARTx == USART2)
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
  if (UARTx == USART3)
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
  if (UARTx == UART4)
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
  if (UARTx == UART5)
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
  if (UARTx == USART6)
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

  if (UARTx == USART1) {
    GPIO_PinAFConfig(tx_port, tx_pin_src, GPIO_AF_USART1);
    GPIO_PinAFConfig(rx_port, rx_pin_src, GPIO_AF_USART1);
  }

  if (UARTx == USART2) {
    GPIO_PinAFConfig(tx_port, tx_pin_src, GPIO_AF_USART2);
    GPIO_PinAFConfig(rx_port, rx_pin_src, GPIO_AF_USART2);
  }

  if (UARTx == USART3) {
    GPIO_PinAFConfig(tx_port, tx_pin_src, GPIO_AF_USART3);
    GPIO_PinAFConfig(rx_port, rx_pin_src, GPIO_AF_USART3);
  }

  if (UARTx == UART4) {
    GPIO_PinAFConfig(tx_port, tx_pin_src, GPIO_AF_UART4);
    GPIO_PinAFConfig(rx_port, rx_pin_src, GPIO_AF_UART4);
  }

  if (UARTx == UART5) {
    GPIO_PinAFConfig(tx_port, tx_pin_src, GPIO_AF_UART5);
    GPIO_PinAFConfig(rx_port, rx_pin_src, GPIO_AF_UART5);
  }

  if (UARTx == USART6) {
    GPIO_PinAFConfig(tx_port, tx_pin_src, GPIO_AF_USART6);
    GPIO_PinAFConfig(rx_port, rx_pin_src, GPIO_AF_USART6);
  }

  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;

  GPIO_InitStructure.GPIO_Pin = tx_pin;
  GPIO_Init(tx_port, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = rx_pin;
  GPIO_Init(rx_port, &GPIO_InitStructure);

  NVIC_InitTypeDef NVIC_InitStructure;
  if (UARTx == USART1)
    NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream5_IRQn;
  if (UARTx == USART2)
    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream5_IRQn;
  if (UARTx == USART3)
    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream1_IRQn;
  if (UARTx == UART4)
    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream2_IRQn;
  if (UARTx == UART5)
    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream0_IRQn;
  if (UARTx == USART6)
    NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = rxDMA_PremptPriority;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  if (UARTx == USART1)
    NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream7_IRQn;
  if (UARTx == USART2)
    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream6_IRQn;
  if (UARTx == USART3)
    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream3_IRQn;
  if (UARTx == UART4)
    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream4_IRQn;
  if (UARTx == UART5)
    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream7_IRQn;
  if (UARTx == USART6)
    NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream6_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = txDMA_PremptPriority;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  if (UARTx == USART1)
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  if (UARTx == USART2)
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
  if (UARTx == USART3)
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
  if (UARTx == UART4)
    NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
  if (UARTx == UART5)
    NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;
  if (UARTx == USART6)
    NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = rxIT_PremptPriority;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = txIT_PremptPriority;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  uint32_t channel;
  if (UARTx == USART1)
    channel = DMA_Channel_4;
  if (UARTx == USART2)
    channel = DMA_Channel_4;
  if (UARTx == USART3)
    channel = DMA_Channel_4;
  if (UARTx == UART4)
    channel = DMA_Channel_4;
  if (UARTx == UART5)
    channel = DMA_Channel_4;
  if (UARTx == USART6)
    channel = DMA_Channel_5;

  // configure DMA for receiving UART data
  DMA_StructInit(&UART_config[n].dmaInitRx);
  UART_config[n].dmaInitRx.DMA_DIR = DMA_DIR_PeripheralToMemory;
  UART_config[n].dmaInitRx.DMA_Channel = channel;
  UART_config[n].dmaInitRx.DMA_Mode = DMA_Mode_Circular;
  UART_config[n].dmaInitRx.DMA_Priority = DMA_Priority_VeryHigh;
  UART_config[n].dmaInitRx.DMA_PeripheralBaseAddr = (uint32_t) & (UARTx->DR);
  UART_config[n].dmaInitRx.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  UART_config[n].dmaInitRx.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  UART_config[n].dmaInitRx.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  UART_config[n].dmaInitRx.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  UART_config[n].dmaInitRx.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  UART_config[n].dmaInitRx.DMA_MemoryInc = DMA_MemoryInc_Enable;
  UART_config[n].dmaInitRx.DMA_FIFOMode = DMA_FIFOMode_Disable;
  UART_config[n].dmaInitRx.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
  UART_config[n].dmaInitRx.DMA_BufferSize = (uint32_t)DMA_bufferSize;
  UART_config[n].dmaInitRx.DMA_Memory0BaseAddr =
      (uint32_t)UART_rxBufDMA[n].elems0;

  if (UARTx == USART1) {
    UART_config[n].streamRx = DMA2_Stream5;
    UART_config[n].DMA_RxIT = DMA_IT_TCIF5;
  }
  if (UARTx == USART2) {
    UART_config[n].streamRx = DMA1_Stream5;
    UART_config[n].DMA_RxIT = DMA_IT_TCIF5;
  }
  if (UARTx == USART3) {
    UART_config[n].streamRx = DMA1_Stream1;
    UART_config[n].DMA_RxIT = DMA_IT_TCIF1;
  }
  if (UARTx == UART4) {
    UART_config[n].streamRx = DMA1_Stream2;
    UART_config[n].DMA_RxIT = DMA_IT_TCIF2;
  }
  if (UARTx == UART5) {
    UART_config[n].streamRx = DMA1_Stream0;
    UART_config[n].DMA_RxIT = DMA_IT_TCIF0;
  }
  if (UARTx == USART6) {
    UART_config[n].streamRx = DMA2_Stream2;
    UART_config[n].DMA_RxIT = DMA_IT_TCIF2;
  }

  DMA_Init(UART_config[n].streamRx, &UART_config[n].dmaInitRx);

  DMA_DoubleBufferModeConfig(UART_config[n].streamRx,
                             (uint32_t)UART_rxBufDMA[n].elems1, DMA_Memory_0);
  DMA_DoubleBufferModeCmd(UART_config[n].streamRx, ENABLE);

  DMA_ITConfig(UART_config[n].streamRx, DMA_IT_TC, ENABLE);
  USART_DMACmd(UARTx, USART_DMAReq_Rx, ENABLE);
  DMA_Cmd(UART_config[n].streamRx, ENABLE);

  // configure DMA for transmitting UART data
  DMA_StructInit(&UART_config[n].dmaInitTx);
  UART_config[n].dmaInitTx.DMA_DIR = DMA_DIR_MemoryToPeripheral;
  UART_config[n].dmaInitTx.DMA_Channel = channel;
  UART_config[n].dmaInitTx.DMA_Mode = DMA_Mode_Circular;
  UART_config[n].dmaInitTx.DMA_Priority = DMA_Priority_Low;
  UART_config[n].dmaInitTx.DMA_PeripheralBaseAddr = (uint32_t) & (UARTx->DR);
  UART_config[n].dmaInitTx.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  UART_config[n].dmaInitTx.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  UART_config[n].dmaInitTx.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  UART_config[n].dmaInitTx.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  UART_config[n].dmaInitTx.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  UART_config[n].dmaInitTx.DMA_MemoryInc = DMA_MemoryInc_Enable;
  UART_config[n].dmaInitTx.DMA_FIFOMode = DMA_FIFOMode_Disable;
  UART_config[n].dmaInitTx.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
  UART_config[n].dmaInitTx.DMA_BufferSize = (uint32_t)DMA_bufferSize;
  UART_config[n].dmaInitTx.DMA_Memory0BaseAddr =
      (uint32_t)UART_txBufDMA[n].elems0;

  if (UARTx == USART1) {
    UART_config[n].streamTx = DMA2_Stream7;
    UART_config[n].DMA_TxIT = DMA_IT_TCIF7;
  }
  if (UARTx == USART2) {
    UART_config[n].streamTx = DMA1_Stream6;
    UART_config[n].DMA_TxIT = DMA_IT_TCIF6;
  }
  if (UARTx == USART3) {
    UART_config[n].streamTx = DMA1_Stream3;
    UART_config[n].DMA_TxIT = DMA_IT_TCIF3;
  }
  if (UARTx == UART4) {
    UART_config[n].streamTx = DMA1_Stream4;
    UART_config[n].DMA_TxIT = DMA_IT_TCIF4;
  }
  if (UARTx == UART5) {
    UART_config[n].streamTx = DMA1_Stream7;
    UART_config[n].DMA_TxIT = DMA_IT_TCIF7;
  }
  if (UARTx == USART6) {
    UART_config[n].streamTx = DMA2_Stream6;
    UART_config[n].DMA_TxIT = DMA_IT_TCIF6;
  }

  DMA_Init(UART_config[n].streamTx, &UART_config[n].dmaInitTx);

  DMA_DoubleBufferModeConfig(UART_config[n].streamTx,
                             (uint32_t)UART_txBufDMA[n].elems1, DMA_Memory_0);
  DMA_DoubleBufferModeCmd(UART_config[n].streamTx, ENABLE);

  DMA_ITConfig(UART_config[n].streamTx, DMA_IT_TC, ENABLE);
  USART_DMACmd(UARTx, USART_DMAReq_Tx, ENABLE);
  DMA_Cmd(UART_config[n].streamTx, DISABLE);

  // enable oversampling by 8
  // Note: Maximum BaudRate that can be achieved when using oversampling-by-8
  // is: (USART APB Clock / 8) 		 e.g. (42 MHz / 8) = 5250000 baud
  USART_OverSampling8Cmd(UARTx, ENABLE);

  // UART configurations
  UART_config[n].init.USART_WordLength = USART_WordLength_8b;
  UART_config[n].init.USART_StopBits = USART_StopBits_1;
  UART_config[n].init.USART_Parity = USART_Parity_No;
  UART_config[n].init.USART_HardwareFlowControl =
      USART_HardwareFlowControl_None;
  UART_config[n].init.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  UART_config[n].init.USART_BaudRate = baud;
  USART_Init(UARTx, &UART_config[n].init);
  USART_Cmd(UARTx, ENABLE);
}

//------------------------------------------------------------------------

/**
 * @brief  UART peripheral ID to type
 * @param  p Peripheral ID
 * @retval UART_TypeDef * [USART1..USART6]
 */
static USART_TypeDef *UART_id2type(uint8_t p) {
  USART_TypeDef *UARTx;
  UARTx = USART1;

  switch (p) {
  case 1:
    UARTx = USART1;
    break;
  case 2:
    UARTx = USART2;
    break;
  case 3:
    UARTx = USART3;
    break;
  case 4:
    UARTx = UART4;
    break;
  case 5:
    UARTx = UART5;
    break;
  case 6:
    UARTx = USART6;
    break;
  default:
    break;
  }

  return UARTx;
}

/**
 * @brief  UART identifier to buffer index
 * @param  id UART identifier [1..6]
 * @retval Buffer index [0..5]
 */
static uint8_t UART_id2buf(uint8_t id) {
  uint8_t b = 0u;

  switch (id) {
  case 1:
    b = 0u;
    break;
  case 2:
    b = 1u;
    break;
  case 3:
    b = 2u;
    break;
  case 4:
    b = 3u;
    break;
  case 5:
    b = 4u;
    break;
  case 6:
    b = 5u;
    break;
  default:
    break;
  }

  return b;
}

/**
 * @brief  UART type to buffer index
 * @param  UARTx UART type [USART1..USART6]
 * @retval Buffer index [0..5]
 */
static uint8_t UART_type2buf(USART_TypeDef *UARTx) {
  uint8_t b = 0u;

  if (UARTx == USART1)
    b = 0u;
  if (UARTx == USART2)
    b = 1u;
  if (UARTx == USART3)
    b = 2u;
  if (UARTx == UART4)
    b = 3u;
  if (UARTx == UART5)
    b = 4u;
  if (UARTx == USART6)
    b = 5u;

  return b;
}

//------------------------------------------------------------------------

/**
 * @brief Sets UART baud rate after initialization
 * @param  p Peripheral ID
 * @param  baud Baud rate desried
 * @retval None
 */
void UART_SetBaud(uint8_t p, uint32_t baud) {
  uint8_t n = UART_id2buf(p);
  UART_config[n].init.USART_BaudRate = baud;
  USART_Init(UART_id2type(p), &UART_config[n].init);
}

/**
 * @brief  Get flag status
 * @param  p Peripheral ID
 * @param  UART_FLAG Status flag
 * @retval None
 */
FlagStatus UART_GetFlagStatus(uint8_t p, uint16_t UART_FLAG) {
  return USART_GetFlagStatus(UART_id2type(p), UART_FLAG);
}

/**
 * @brief  Receive data
 * @param  p Peripheral ID
 * @retval None
 */
uint16_t UART_ReceiveData(uint8_t p) {
  return USART_ReceiveData(UART_id2type(p));
}

/**
 * @brief  Send data
 * @param  p Peripheral ID
 * @param  data Data to be sent
 * @retval None
 */
void UART_SendData(uint8_t p, uint16_t data) {
  return USART_SendData(UART_id2type(p), data);
}

//------------------------------------------------------------------------

/**
 * @brief  Configure IT receiver interrupt
 * @param  p Peripheral ID
 * @param  NewState Enable/Disable
 * @retval None
 */
void UART_IT_RxIRQ(uint8_t p, FunctionalState NewState) {
  USART_ITConfig(UART_id2type(p), USART_IT_RXNE, NewState);
}

/**
 * @brief  Configure IT transmitter interrupt
 * @param  p Peripheral ID
 * @param  NewState Enable/Disable
 * @retval None
 */
void UART_IT_TxIRQ(uint8_t p, FunctionalState NewState) {
  USART_ITConfig(UART_id2type(p), USART_IT_TXE, NewState);
}

/**
 * @brief  Get IT receiver interrupt status
 * @param  p Peripheral ID
 * @retval None
 */
FlagStatus UART_IT_GetRxITStatus(uint8_t p) {
  return USART_GetITStatus(UART_id2type(p), USART_IT_RXNE);
}

/**
 * @brief  Get IT transmitter interrupt status
 * @param  p Peripheral ID
 * @retval None
 */
FlagStatus UART_IT_GetTxITStatus(uint8_t p) {
  return USART_GetITStatus(UART_id2type(p), USART_IT_TXE);
}

/**
 * @brief  Get DMA receiver interrupt status
 * @param  p Peripheral ID
 * @retval SET/RESET
 */
FlagStatus UART_DMA_GetRxITStatus(uint8_t p) {
  uint8_t n = UART_id2buf(p);
  return DMA_GetITStatus(UART_config[n].streamRx, UART_config[n].DMA_RxIT);
}

/**
 * @brief  Get DMA transmitter interrupt status
 * @param  p Peripheral ID
 * @retval SET/RESET
 */
FlagStatus UART_DMA_GetTxITStatus(uint8_t p) {
  uint8_t n = UART_id2buf(p);
  return DMA_GetITStatus(UART_config[n].streamTx, UART_config[n].DMA_TxIT);
}

/**
 * @brief  Clear DMA receiver interrupt pending bit
 * @param  p Peripheral ID
 * @retval None
 */
void UART_DMA_ClearRxITPendingBit(uint8_t p) {
  uint8_t n = UART_id2buf(p);
  DMA_ClearITPendingBit(UART_config[n].streamRx, UART_config[n].DMA_RxIT);
}

/**
 * @brief  Clear DMA transmitter interrupt pending bit
 * @param  p Peripheral ID
 * @retval None
 */
void UART_DMA_ClearTxITPendingBit(uint8_t p) {
  uint8_t n = UART_id2buf(p);
  DMA_ClearITPendingBit(UART_config[n].streamTx, UART_config[n].DMA_TxIT);
}

//------------------------------------------------------------------------

/**
 * @brief  Configure DMA receive interrupt
 * @param  p Peripheral ID
 * @param  NewState Enable/Disable
 * @retval None
 */
void UART_DMA_RxIRQ(uint8_t p, FunctionalState NewState) {
  uint8_t n = UART_id2buf(p);
  DMA_Cmd(UART_config[n].streamRx, NewState);
  USART_DMACmd(UART_id2type(p), USART_DMAReq_Rx, NewState);
}

/**
 * @brief  Configure DMA transmit interrupt
 * @param  p Peripheral ID
 * @param  NewState Enable/Disable
 * @retval None
 */
void UART_DMA_TxIRQ(uint8_t p, FunctionalState NewState) {
  uint8_t n = UART_id2buf(p);
  DMA_Cmd(UART_config[n].streamTx, NewState);
  USART_DMACmd(UART_id2type(p), USART_DMAReq_Tx, NewState);
}

/**
 * @brief  Configure DMA receive mode
 * @param  p Peripheral ID
 * @param  mode DMA_Mode_Normal or DMA_Mode_Circular
 * @retval None
 */
void UART_DMA_RxModeConfig(uint8_t p, uint32_t mode) {
  uint8_t n = UART_id2buf(p);

  switch (mode) {
  case DMA_Mode_Normal:
    DMA_DoubleBufferModeCmd(UART_config[n].streamRx, DISABLE);
    break;
  case DMA_Mode_Circular:
    DMA_DoubleBufferModeCmd(UART_config[n].streamRx, ENABLE);
    break;
  default:
    break;
  }

  UART_config[n].dmaInitRx.DMA_Mode = mode;
  DMA_Init(UART_config[n].streamRx, &UART_config[n].dmaInitRx);
}

/**
 * @brief  Configure DMA transmit mode
 * @param  p Peripheral ID
 * @param  mode DMA_Mode_Normal or DMA_Mode_Circular
 * @retval None
 */
void UART_DMA_TxModeConfig(uint8_t p, uint32_t mode) {
  uint8_t n = UART_id2buf(p);

  switch (mode) {
  case DMA_Mode_Normal:
    DMA_DoubleBufferModeCmd(UART_config[n].streamTx, DISABLE);
    break;
  case DMA_Mode_Circular:
    DMA_DoubleBufferModeCmd(UART_config[n].streamTx, ENABLE);
    break;
  default:
    break;
  }

  UART_config[n].dmaInitTx.DMA_Mode = mode;
  DMA_Init(UART_config[n].streamTx, &UART_config[n].dmaInitTx);
}

/**
 * @brief  Configure DMA receive buffer size
 * @param  p Peripheral ID
 * @param  size Buffer size
 * @retval None
 */
void UART_DMA_RxBufferSizeConfig(uint8_t p, uint16_t size) {
  uint8_t n = UART_id2buf(p);
  UART_config[n].dmaInitRx.DMA_BufferSize = size;
  DMA_SetCurrDataCounter(UART_config[n].streamRx, size);
}

/**
 * @brief  Configure DMA transmit buffer size
 * @param  p Peripheral ID
 * @param  size Buffer size
 * @retval None
 */
void UART_DMA_TxBufferSizeConfig(uint8_t p, uint16_t size) {
  uint8_t n = UART_id2buf(p);
  UART_config[n].dmaInitTx.DMA_BufferSize = size;
  DMA_SetCurrDataCounter(UART_config[n].streamTx, size);
}

//------------------------------------------------------------------------

/**
 * @brief  UART transmit interrupt service routine (via IT)
 * @param  p UART peripheral ID
 * @retval None
 */
void UART_TxISR_IT(uint8_t p) {
  if (USART_GetITStatus(UART_id2type(p), USART_IT_TXE) == RESET)
    return;

  uint8_t n = UART_id2buf(p);

  uint8_t byte;
  CB_Read(&UART_txBufIT[n], &byte);
  USART_SendData(UART_id2type(p), byte); // send byte to UART

  // when everything has been transmitted, pause transmission
  if (CB_IsEmpty(&UART_txBufIT[n])) {
    USART_ITConfig(UART_id2type(p), USART_IT_TXE, DISABLE);
  }
}

/**
 * @brief UART transmit interrupt service routine (via DMA)
 * @param p UART peripheral ID
 * @retval None
 */
void UART_TxISR_DMA(uint8_t p) {
  uint8_t n = UART_id2buf(p);

  if (DMA_GetITStatus(UART_config[n].streamTx, UART_config[n].DMA_TxIT) ==
      RESET)
    return;

  // now that DMA transfer is complete and there's no data to be transmitted,
  // pause DMA stream
  if (UART_DMA_txBuf0_flag[n]) {
    UART_DMA_txBuf0_flag[n] = 0u;
    DB_Clear(&UART_txBufDMA[n], 0u);
    if (!DB_IsFull(&UART_txBufDMA[n], 2u)) {
      DMA_Cmd(UART_config[n].streamTx, DISABLE);
    }
  }

  if (UART_DMA_txBuf1_flag[n]) {
    UART_DMA_txBuf1_flag[n] = 0u;
    DB_Clear(&UART_txBufDMA[n], 1u);
    if (!DB_IsFull(&UART_txBufDMA[n], 1u)) {
      DMA_Cmd(UART_config[n].streamTx, DISABLE);
    }
  }

  DMA_ClearITPendingBit(UART_config[n].streamTx, UART_config[n].DMA_TxIT);
}

//------------------------------------------------------------------------

/**
 * @brief  Get pointer to UART DMA-receive buffer
 * @param  p USART peripheral ID
 * @retval Pointer to UART DMA-receiver buffer
 */
uint8_t *UART_GetDMABuffer(uint8_t p) {
  uint8_t n = UART_id2buf(p);

  switch (UART_config[n].dmaInitRx.DMA_Mode) {
  case DMA_Mode_Normal:
    return UART_rxBufDMA[n].elems0;
    break;

  case DMA_Mode_Circular:
    switch (DMA_GetCurrentMemoryTarget(UART_config[n].streamRx)) {
    case 0: // DMA_Memory_0 in use
      return DB_GetBuf(&UART_rxBufDMA[n], 1u);
      break;

    case 1: // DMA_Memory_1 in use
      return DB_GetBuf(&UART_rxBufDMA[n], 0u);
      break;

    default:
      break;
    }
    break;

  default:
    break;
  }

  return NULL;
}

/**
 * @brief  Write data to UART IT-transmit buffer
 * @param  p USART peripheral ID
 * @param  data Pointer to buffer
 * @param  size Size of buffer
 * @retval size Number of bytes written to UART transmit IT buffer
 */
uint8_t UART_Write_IT(uint8_t p, void *data, uint16_t size) {
  uint8_t n = UART_id2buf(p);

  // copy data to the buffer if there's sufficient space
  if (size <= CB_FreeElems(&UART_txBufIT[n])) {
    uint16_t i;
    for (i = 0; i < size; i++) {
      CB_Write(&UART_txBufIT[n], data + i);
    }

    // enable UART transmit interrupt
    USART_ITConfig(UART_id2type(p), USART_IT_TXE, ENABLE);
    return size;
  }

  return 0u;
}

/**
 * @brief  Write data to UART DMA-transmit buffer
 * @param  p USART peripheral ID
 * @param  data Pointer to data buffer
 * @param  size Size of buffer
 * @retval size Number of bytes written to UART transmit DMA buffer
 */
uint8_t UART_Write_DMA(uint8_t p, void *data, uint16_t size) {
  uint8_t n = UART_id2buf(p);

  // copy data to the buffer if there's sufficient space
  if (size <= DB_FreeElems(&UART_txBufDMA[n])) {
    uint16_t i;
    for (i = 0; i < size; i++) {
      DB_Write(&UART_txBufDMA[n], data + i);
    }

    // when half the buffer is filled, enable UART transmit via DMA
    if (DB_IsFull(&UART_txBufDMA[n], 0u)) {
      UART_DMA_txBuf0_flag[n] = 1u;
      DMA_Cmd(UART_config[n].streamTx, ENABLE);
    }

    if (DB_IsFull(&UART_txBufDMA[n], 1u)) {
      UART_DMA_txBuf1_flag[n] = 1u;
      DMA_Cmd(UART_config[n].streamTx, ENABLE);
    }

    return size;
  }

  return 0u;
}

//------------------------------------------------------------------------

/**
 * @brief  Write text to UART transmit buffer
 * @param  p USART peripheral ID
 * @param  str Pointer to text buffer
 * @retval None
 */
void UART_Print(uint8_t p, void *str) {
  UART_Write_IT(p, str, strlen((char *)str));
}

/**
 * @}
 */

/**
 * @}
 */

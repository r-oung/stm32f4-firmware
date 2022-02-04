/**
 * @file    sdio.c
 * @author  Raymond Oung
 * @date    2014.10.30
 * @brief   SDIO interface functions
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
#include "sdio.h"

static DMA_InitTypeDef SDIO_DMA_InitStructure_Tx;
static DMA_InitTypeDef SDIO_DMA_InitStructure_Rx;

/**
 * @brief  Initialise SDIO interface.
 * @param  -
 * @retval None
 */
void SDIO_PeriphInit(uint32_t d0_clk, GPIO_TypeDef *d0_port, uint16_t d0_pin,
                     uint8_t d0_pin_src, uint32_t d1_clk, GPIO_TypeDef *d1_port,
                     uint16_t d1_pin, uint8_t d1_pin_src, uint32_t d2_clk,
                     GPIO_TypeDef *d2_port, uint16_t d2_pin, uint8_t d2_pin_src,
                     uint32_t d3_clk, GPIO_TypeDef *d3_port, uint16_t d3_pin,
                     uint8_t d3_pin_src, uint32_t cmd_clk,
                     GPIO_TypeDef *cmd_port, uint16_t cmd_pin,
                     uint8_t cmd_pin_src, uint32_t clk_clk,
                     GPIO_TypeDef *clk_port, uint16_t clk_pin,
                     uint8_t clk_pin_src, uint8_t IT_PremptPriority,
                     uint8_t DMA_PremptPriority) {
  RCC_AHB1PeriphClockCmd(d0_clk | d1_clk | d2_clk | d3_clk | cmd_clk | clk_clk,
                         ENABLE);
  RCC_AHB1PeriphClockCmd(SDIO_DMA_CLK, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SDIO, ENABLE);

  GPIO_PinAFConfig(d0_port, d0_pin_src, GPIO_AF_SDIO);
  GPIO_PinAFConfig(d1_port, d1_pin_src, GPIO_AF_SDIO);
  GPIO_PinAFConfig(d2_port, d2_pin_src, GPIO_AF_SDIO);
  GPIO_PinAFConfig(d3_port, d3_pin_src, GPIO_AF_SDIO);
  GPIO_PinAFConfig(cmd_port, cmd_pin_src, GPIO_AF_SDIO);
  GPIO_PinAFConfig(clk_port, clk_pin_src, GPIO_AF_SDIO);

  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Pin = d0_pin;
  GPIO_Init(d0_port, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = d1_pin;
  GPIO_Init(d1_port, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = d2_pin;
  GPIO_Init(d2_port, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = d3_pin;
  GPIO_Init(d3_port, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = cmd_pin;
  GPIO_Init(cmd_port, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Pin = clk_pin;
  GPIO_Init(clk_port, &GPIO_InitStructure);

  NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannel = SDIO_IT_IRQ;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = IT_PremptPriority;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = SDIO_DMA_IRQ;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = DMA_PremptPriority;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
  NVIC_Init(&NVIC_InitStructure);

  SDIO_DMA_InitStructure_Tx.DMA_Channel = SDIO_DMA_CHANNEL;
  SDIO_DMA_InitStructure_Tx.DMA_PeripheralBaseAddr = (uint32_t) & (SDIO->FIFO);
  SDIO_DMA_InitStructure_Tx.DMA_DIR = DMA_DIR_MemoryToPeripheral;
  SDIO_DMA_InitStructure_Tx.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  SDIO_DMA_InitStructure_Tx.DMA_MemoryInc = DMA_MemoryInc_Enable;
  SDIO_DMA_InitStructure_Tx.DMA_PeripheralDataSize =
      DMA_PeripheralDataSize_Word;
  SDIO_DMA_InitStructure_Tx.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
  SDIO_DMA_InitStructure_Tx.DMA_Mode = DMA_Mode_Normal;
  SDIO_DMA_InitStructure_Tx.DMA_Priority = DMA_Priority_VeryHigh;
  SDIO_DMA_InitStructure_Tx.DMA_FIFOMode = DMA_FIFOMode_Enable;
  SDIO_DMA_InitStructure_Tx.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
  SDIO_DMA_InitStructure_Tx.DMA_MemoryBurst = DMA_MemoryBurst_INC4;
  SDIO_DMA_InitStructure_Tx.DMA_PeripheralBurst = DMA_PeripheralBurst_INC4;

  SDIO_DMA_InitStructure_Rx.DMA_Channel = SDIO_DMA_CHANNEL;
  SDIO_DMA_InitStructure_Rx.DMA_PeripheralBaseAddr = (uint32_t) & (SDIO->FIFO);
  SDIO_DMA_InitStructure_Rx.DMA_DIR = DMA_DIR_PeripheralToMemory;
  SDIO_DMA_InitStructure_Rx.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  SDIO_DMA_InitStructure_Rx.DMA_MemoryInc = DMA_MemoryInc_Enable;
  SDIO_DMA_InitStructure_Rx.DMA_PeripheralDataSize =
      DMA_PeripheralDataSize_Word;
  SDIO_DMA_InitStructure_Rx.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
  SDIO_DMA_InitStructure_Rx.DMA_Mode = DMA_Mode_Normal;
  SDIO_DMA_InitStructure_Rx.DMA_Priority = DMA_Priority_VeryHigh;
  SDIO_DMA_InitStructure_Rx.DMA_FIFOMode = DMA_FIFOMode_Enable;
  SDIO_DMA_InitStructure_Rx.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
  SDIO_DMA_InitStructure_Rx.DMA_MemoryBurst = DMA_MemoryBurst_INC4;
  SDIO_DMA_InitStructure_Rx.DMA_PeripheralBurst = DMA_PeripheralBurst_INC4;
}

/**
 * @brief  Configures the DMA2 Channel4 for SDIO Tx request.
 * @param  buf Pointer to the source buffer
 * @param  size Buffer size
 * @retval None
 */
void SDIO_DMA_TxConfig(uint32_t *buf, uint32_t size) {
  DMA_ClearFlag(SDIO_DMA_STREAM, SDIO_DMA_FLAG_FEIF | SDIO_DMA_FLAG_DMEIF |
                                     SDIO_DMA_FLAG_TEIF | SDIO_DMA_FLAG_HTIF |
                                     SDIO_DMA_FLAG_TCIF);
  DMA_Cmd(SDIO_DMA_STREAM, DISABLE);

  SDIO_DMA_InitStructure_Tx.DMA_Memory0BaseAddr = (uint32_t)buf;
  SDIO_DMA_InitStructure_Tx.DMA_BufferSize = size;
  DMA_Init(SDIO_DMA_STREAM, &SDIO_DMA_InitStructure_Tx);

  DMA_ITConfig(SDIO_DMA_STREAM, DMA_IT_TC, ENABLE);
  DMA_FlowControllerConfig(SDIO_DMA_STREAM, DMA_FlowCtrl_Peripheral);
  DMA_Cmd(SDIO_DMA_STREAM, ENABLE);
}

/**
 * @brief  Configures the DMA2 Channel4 for SDIO Rx request.
 * @param  buf Pointer to the destination buffer
 * @param  size Buffer size
 * @retval None
 */
void SDIO_DMA_RxConfig(uint32_t *buf, uint32_t size) {
  DMA_ClearFlag(SDIO_DMA_STREAM, SDIO_DMA_FLAG_FEIF | SDIO_DMA_FLAG_DMEIF |
                                     SDIO_DMA_FLAG_TEIF | SDIO_DMA_FLAG_HTIF |
                                     SDIO_DMA_FLAG_TCIF);
  DMA_Cmd(SDIO_DMA_STREAM, DISABLE);

  SDIO_DMA_InitStructure_Rx.DMA_Memory0BaseAddr = (uint32_t)buf;
  SDIO_DMA_InitStructure_Rx.DMA_BufferSize = size;
  DMA_Init(SDIO_DMA_STREAM, &SDIO_DMA_InitStructure_Rx);

  DMA_ITConfig(SDIO_DMA_STREAM, DMA_IT_TC, ENABLE);
  DMA_FlowControllerConfig(SDIO_DMA_STREAM, DMA_FlowCtrl_Peripheral);
  DMA_Cmd(SDIO_DMA_STREAM, ENABLE);
}

/**
 * @brief  Clear DMA flag.
 * @param  None
 * @retval None
 */
void SDIO_DMA_ClearFlag(void) {
  DMA_ClearFlag(SDIO_DMA_STREAM, SDIO_DMA_FLAG_TCIF | SDIO_DMA_FLAG_FEIF);
}

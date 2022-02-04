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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SDIO_H
#define __SDIO_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"

#define SDIO_IT_IRQHandler SDIO_IRQHandler
#define SDIO_DMA_IRQHandler DMA2_Stream3_IRQHandler

#define SDIO_IT_IRQ SDIO_IRQn
#define SDIO_DMA_IRQ DMA2_Stream3_IRQn

#define SDIO_DMA_CLK RCC_AHB1Periph_DMA2
#define SDIO_DMA_STREAM DMA2_Stream3
#define SDIO_DMA_CHANNEL DMA_Channel_4
#define SDIO_DMA_IT DMA_IT_TCIF3

#define SDIO_DMA_FLAG_FEIF DMA_FLAG_FEIF3
#define SDIO_DMA_FLAG_DMEIF DMA_FLAG_DMEIF3
#define SDIO_DMA_FLAG_TEIF DMA_FLAG_TEIF3
#define SDIO_DMA_FLAG_HTIF DMA_FLAG_HTIF3
#define SDIO_DMA_FLAG_TCIF DMA_FLAG_TCIF3

/** @defgroup SDIO_Exported_Functions
 * @{
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
                     uint8_t DMA_PremptPriority);
void SDIO_DMA_TxConfig(uint32_t *buf, uint32_t size);
void SDIO_DMA_RxConfig(uint32_t *buf, uint32_t size);
void SDIO_DMA_ClearFlag(void);
/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* __SDIO_H */

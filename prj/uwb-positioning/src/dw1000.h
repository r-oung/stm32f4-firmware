/**
 * @file    dw1000.h
 * @author  Raymond Oung
 * @date    2014.08.11
 * @brief   DW1000 driver
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
#ifndef __DW1000_H
#define __DW1000_H

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

/** @addtogroup DW1000
 * @{
 */

/** @defgroup DW1000_Exported_Types
 * @{
 */
typedef enum _DW1000_channel_t {
  CH1 = 1u,
  CH2 = 2u,
  CH3 = 3u,
  CH4 = 4u,
  CH5 = 5u,
  CH7 = 7u
} DW1000_channel_t;

typedef enum _DW1000_data_rate_t {
  DATA_RATE_110 = 1u, // 110 Kbps, recommended preamble length: 2048, 4096
  DATA_RATE_850 = 2u, // 850 Kbps, recommended preamble length: 256, 512, 1024
  DATA_RATE_6800 = 3u // 6.8 Mbps, recommended preamble length: 64, 128, 256
} DW1000_data_rate_t;

typedef enum _DW1000_prf_t {
  PRF_4MHZ = 0,
  PRF_16MHZ = 1,
  PRF_64MHZ = 2
} DW1000_prf_t;

typedef enum _DW1000_preamble_code_t {
  PREAMBLE_CODE_1 = 1u,
  PREAMBLE_CODE_2 = 2u,
  PREAMBLE_CODE_3 = 3u,
  PREAMBLE_CODE_4 = 4u,
  PREAMBLE_CODE_5 = 5u,
  PREAMBLE_CODE_6 = 6u,
  PREAMBLE_CODE_7 = 7u,
  PREAMBLE_CODE_8 = 8u,
  PREAMBLE_CODE_9 = 9u,
  PREAMBLE_CODE_10 = 10u,
  PREAMBLE_CODE_11 = 11u,
  PREAMBLE_CODE_12 = 12u,
  PREAMBLE_CODE_17 = 17u,
  PREAMBLE_CODE_18 = 18u,
  PREAMBLE_CODE_19 = 19u,
  PREAMBLE_CODE_20 = 20u,
  PREAMBLE_CODE_21 = 21u,
  PREAMBLE_CODE_22 = 22u,
  PREAMBLE_CODE_23 = 23u,
  PREAMBLE_CODE_24 = 24u
} DW1000_preamble_code_t;

typedef enum _DW1000_preamble_length_t {
  PREAMBLE_LENGTH_64 = 64u,
  PREAMBLE_LENGTH_128 = 128u,
  PREAMBLE_LENGTH_256 = 256u,
  PREAMBLE_LENGTH_512 = 512u,
  PREAMBLE_LENGTH_1024 = 1024u,
  PREAMBLE_LENGTH_2048 = 2048u,
  PREAMBLE_LENGTH_4096 = 4096u
} DW1000_preamble_length_t;
/**
 * @}
 */

/** @defgroup DW1000_Exported_Functions
 * @{
 */
#define DW1000_TIME2SEC 1.0 / 499.2e6 / 128.0 // ~15.65e-12 sec
#define DW1000_MAX_TIMESTAMP 0xFFFFFFFFFF

#define DW1000_STATIC 0x00
#define DW1000_DYNAMIC 0xFF

void DW1000_Init(SPI_TypeDef *SPIx, uint32_t en_clk, GPIO_TypeDef *en_port,
                 uint16_t en_pin, uint32_t cs_clk, GPIO_TypeDef *cs_port,
                 uint16_t cs_pin, uint32_t wake_clk, GPIO_TypeDef *wake_port,
                 uint16_t wake_pin, uint32_t nrst_clk, GPIO_TypeDef *nrst_port,
                 uint16_t nrst_pin, uint32_t irq_clk, GPIO_TypeDef *irq_port,
                 uint16_t irq_pin, uint8_t irq_port_src, uint8_t irq_pin_src,
                 uint32_t irq_line, uint8_t irq_IRQn,
                 uint8_t irq_premptPriority);

uint8_t DW1000_Config(DW1000_channel_t ch, DW1000_data_rate_t rate,
                      DW1000_prf_t prf, DW1000_preamble_code_t preamble_code,
                      DW1000_preamble_length_t preamble_length);

void DW1000_RxTimeout(FunctionalState NewState);

void DW1000_Reset(void);

uint8_t DW1000_Receive(uint8_t *buf);
void DW1000_Transmit(uint8_t *buf, uint8_t size);

uint64_t DW1000_GetSysTimestamp(void);
uint64_t DW1000_GetRxTimestamp(void);
uint64_t DW1000_GetTxTimestamp(void);

float DW1000_GetSNR(void);
float DW1000_GetFPPower(void);
float DW1000_GetRxPower(void);

void DW1000_TransceiverOff(void);
void DW1000_ReceiverOn(void);

float DW1000_ReadBackgroundEnergy(DW1000_channel_t ch);
float DW1000_ReadTemperature(void);
float DW1000_GetVoltage(void);
float DW1000_GetTemperature(void);
void DW1000_ReadAccumulatorCIR(uint16_t *real, uint16_t *img);

void DW1000_PrintSysStatus(void);

void DW1000_ConfigDefault(void);
void DW1000_ReadAllRegisters(void);
/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* __DW1000_H */

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

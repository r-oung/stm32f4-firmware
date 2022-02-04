/**
 * @file    can.h
 * @author  Raymond Oung
 * @date    2013.09.25
 * @brief   CAN bus interface functions
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
#ifndef __CAN_H
#define __CAN_H

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

/** @addtogroup CAN
 * @{
 */

/** @defgroup CAN_Exported_Defines
 * @{
 */
#define CAN1_RX_IRQn CAN1_RX0_IRQn
#define CAN2_RX_IRQn CAN2_RX0_IRQn
#define CAN1_RX_IRQHandler CAN1_RX0_IRQHandler
#define CAN2_RX_IRQHandler CAN2_RX0_IRQHandler
/**
 * @}
 */

/** @defgroup CAN_Private_Variables
 * @{
 */
/**
 * @}
 */

/** @defgroup CAN_Exported_Functions
 * @{
 */
void CAN_PeriphInit(CAN_TypeDef *CANx, uint8_t filt_num, uint32_t clk,
                    uint32_t pclk, GPIO_TypeDef *tx_port, uint16_t tx_pin,
                    uint8_t tx_pin_src, GPIO_TypeDef *rx_port, uint16_t rx_pin,
                    uint8_t rx_pin_src, uint8_t rxIRQn,
                    uint8_t rxPremptPriority, uint8_t txIRQn,
                    uint8_t txPremptPriority);
uint8_t CAN_Write(uint8_t p, uint32_t id, void *data, uint8_t size);
void CAN_TxISR(uint8_t p);
/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H */

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

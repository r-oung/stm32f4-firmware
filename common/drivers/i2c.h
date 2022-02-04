/**
 * @file    i2c.h
 * @author  Raymond Oung
 * @date    2009.11.02
 * @brief   I2C interface functions
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
#ifndef __I2C_H
#define __I2C_H

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

/** @addtogroup I2C
 * @{
 */

/** @defgroup I2C_Defines
 * @{
 */
#define I2C1_IRQHandler I2C1_EV_IRQHandler
#define I2C2_IRQHandler I2C2_EV_IRQHandler
/**
 * @}
 */

/** @defgroup I2C_Exported_Functions
 * @{
 */
void I2C_PeriphInit(I2C_TypeDef *I2Cx, uint32_t speed, uint32_t sda_clk,
                    GPIO_TypeDef *sda_port, uint16_t sda_pin,
                    uint8_t sda_pin_src, uint32_t scl_clk,
                    GPIO_TypeDef *scl_port, uint16_t scl_pin,
                    uint8_t scl_pin_src, uint8_t premptPriority);
void I2C_ResetInit(uint32_t clk, GPIO_TypeDef *port, uint16_t pin);
void I2C_On(void);
void I2C_Off(void);

void I2C_SetSpeed(I2C_TypeDef *I2Cx, uint32_t speed);
uint32_t I2C_GetSpeed(I2C_TypeDef *I2Cx);

void I2C_SetIT(I2C_TypeDef *I2Cx, FunctionalState NewState);
uint8_t I2C_ITStop(I2C_TypeDef *I2Cx);
uint8_t I2C_GetBusyStatus(I2C_TypeDef *I2Cx);

void I2C_IRQn(I2C_TypeDef *I2Cx, FunctionalState NewState);
/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* __I2C_H */

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

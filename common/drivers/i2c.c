/**
 * @file    i2c.c
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

/* Includes ------------------------------------------------------------------*/
#include "i2c.h"

/** @addtogroup Source
 * @{
 */

/** @addtogroup Peripherals
 * @{
 */

/** @defgroup I2C
 * @brief I2C setup
 * @{
 */

/** @defgroup I2C_Private_Functions
 * @{
 */
static uint8_t I2C_type2buf(I2C_TypeDef *I2Cx);
/**
 * @}
 */

/** @defgroup I2C_Private_Variables
 * @{
 */
struct _LED_state_t {
  uint32_t clk;
  GPIO_TypeDef *port;
  uint16_t pin;
} I2C_Reset;

static struct _I2C_status_t {
  uint8_t stopReq;
  uint8_t busy;
} I2C_status[2];
static I2C_InitTypeDef I2C_config[2];
/**
 * @}
 */

/**
 * @brief  Initializes the I2C peripheral
 * @param  I2Cx        I2C Peripheral
 * @param  speed       Clock speed [Hz]
 * @param  sda_clk     SDA GPIO clock
 * @param  sda_port    SDA GPIO port
 * @param  sda_pin     SDA GPIO pin
 * @param  sda_pin_src SDA GPIO pin source
 * @param  scl_clk     SCL GPIO clock
 * @param  scl_port    SCL GPIO port
 * @param  scl_pin     SCL GPIO pin
 * @param  scl_pin_src SCL GPIO pin source
 * @retval None
 */
void I2C_PeriphInit(I2C_TypeDef *I2Cx, uint32_t speed, uint32_t sda_clk,
                    GPIO_TypeDef *sda_port, uint16_t sda_pin,
                    uint8_t sda_pin_src, uint32_t scl_clk,
                    GPIO_TypeDef *scl_port, uint16_t scl_pin,
                    uint8_t scl_pin_src, uint8_t premptPriority) {
  uint8_t n = I2C_type2buf(I2Cx);

  RCC_AHB1PeriphClockCmd(sda_clk | scl_clk, ENABLE); // GPIO clock

  if (I2Cx == I2C1)
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
  if (I2Cx == I2C2)
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);

  if (I2Cx == I2C1) {
    GPIO_PinAFConfig(sda_port, sda_pin_src, GPIO_AF_I2C1);
    GPIO_PinAFConfig(scl_port, scl_pin_src, GPIO_AF_I2C1);
  }

  if (I2Cx == I2C2) {
    GPIO_PinAFConfig(sda_port, sda_pin_src, GPIO_AF_I2C2);
    GPIO_PinAFConfig(scl_port, scl_pin_src, GPIO_AF_I2C2);
  }

  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

  GPIO_InitStructure.GPIO_Pin = scl_pin;
  GPIO_Init(scl_port, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = sda_pin;
  GPIO_Init(sda_port, &GPIO_InitStructure);

  NVIC_InitTypeDef NVIC_InitStructure;
  if (I2Cx == I2C1)
    NVIC_InitStructure.NVIC_IRQChannel = I2C1_EV_IRQn;
  if (I2Cx == I2C2)
    NVIC_InitStructure.NVIC_IRQChannel = I2C2_EV_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = premptPriority;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  I2C_config[n].I2C_Mode = I2C_Mode_I2C;
  I2C_config[n].I2C_DutyCycle = I2C_DutyCycle_2;
  I2C_config[n].I2C_OwnAddress1 = 0x00;
  I2C_config[n].I2C_Ack = I2C_Ack_Enable;
  I2C_config[n].I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;

  I2C_config[n].I2C_ClockSpeed = speed;
  I2C_Cmd(I2Cx, ENABLE);          // enable peripheral
  I2C_Init(I2Cx, &I2C_config[n]); // apply configurations after enabling it
  I2C_ITConfig(I2Cx, I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR, DISABLE);
}

/**
 * @brief  Initializes the I2C reset pin
 * @param  clk  Peripheral clock
 * @param  port GPIO port
 * @param  pin  GPIO pin
 * @retval None
 */
void I2C_ResetInit(uint32_t clk, GPIO_TypeDef *port, uint16_t pin) {
  RCC_AHB1PeriphClockCmd(clk, ENABLE);

  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = pin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(port, &GPIO_InitStructure);

  // initialise state
  I2C_Reset.clk = clk;
  I2C_Reset.port = port;
  I2C_Reset.pin = pin;
  I2C_Off();
}

/**
 * @brief  Switches selected I2C On
 * @param  None
 * @retval None
 */
void I2C_On(void) { GPIO_ResetBits(I2C_Reset.port, I2C_Reset.pin); }

/**
 * @brief  Switches selected I2C Off
 * @param  None
 * @retval None
 */
void I2C_Off(void) { GPIO_SetBits(I2C_Reset.port, I2C_Reset.pin); }

/**
 * @brief  I2C type to buffer index
 * @param  I2Cx I2C type [I2C1,I2C2]
 * @retval Buffer index [0,1]
 */
static uint8_t I2C_type2buf(I2C_TypeDef *I2Cx) {
  uint8_t b = 0u;

  if (I2Cx == I2C1)
    b = 0u;
  if (I2Cx == I2C2)
    b = 1u;

  return b;
}

/**
 * @brief  Set I2C speed
 * @param  I2Cx I2C Peripheral
 * @param  speed I2C speed
 * @retval None
 */
void I2C_SetSpeed(I2C_TypeDef *I2Cx, uint32_t speed) {
  uint8_t n = I2C_type2buf(I2Cx);
  I2C_Cmd(I2Cx, DISABLE);
  I2C_config[n].I2C_ClockSpeed = speed;
  I2C_Init(I2Cx, &I2C_config[n]);
  I2C_Cmd(I2Cx, ENABLE);
}

/**
 * @brief  Get I2C speed
 * @param  I2Cx I2C Peripheral
 * @retval None
 */
uint32_t I2C_GetSpeed(I2C_TypeDef *I2Cx) {
  uint8_t n = I2C_type2buf(I2Cx);
  return I2C_config[n].I2C_ClockSpeed;
}

/**
 * @brief  Enable/Disable I2C interrupts
 * @param  I2Cx I2C Peripheral
 * @param  NewState DISABLE or ENABLE
 * @retval None
 */
void I2C_SetIT(I2C_TypeDef *I2Cx, FunctionalState NewState) {
  uint8_t n = I2C_type2buf(I2Cx);

  if (NewState == ENABLE) {
    I2C_ITConfig(I2Cx, I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR, ENABLE);
    I2C_GenerateSTART(I2Cx, ENABLE);

    // reset flags
    I2C_status[n].busy = 1u;
    I2C_status[n].stopReq = 0u;
  } else {
    if (I2C_status[n].busy) {
      I2C_status[n].stopReq = 1u;
    } else {
      while (I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY) == SET)
        ;
      I2C_ITConfig(I2Cx, I2C_IT_EVT | I2C_IT_BUF, DISABLE);
    }
  }
}

/**
 * @brief  Return value of stop request
 * @param  I2Cx I2C Peripheral
 * @retval None
 */
uint8_t I2C_ITStop(I2C_TypeDef *I2Cx) {
  uint8_t n = I2C_type2buf(I2Cx);

  if (I2C_status[n].stopReq) {
    while (I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY) == SET)
      ;
    I2C_ITConfig(I2Cx, I2C_IT_EVT | I2C_IT_BUF, DISABLE);
    I2C_status[n].busy = 0u;
    return 1;
  }

  return 0;
}

/**
 * @brief  Return value of stop status
 * @param  I2Cx I2C Peripheral
 * @retval Value of I2C_StopRequested
 */
uint8_t I2C_GetBusyStatus(I2C_TypeDef *I2Cx) {
  uint8_t n = I2C_type2buf(I2Cx);
  return I2C_status[n].busy;
}
/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

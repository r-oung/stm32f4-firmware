/**
 * @file    stm32f4xx_it.c
 * @author  Raymond Oung
 * @date    2014.07.27
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

#include "clock.h"
#include "i2c.h"
#include "tim.h"
#include "uart.h"

#include "led.h" // debug
#include "ms5611.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
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

/******************************************************************************/
/*                 STM32F40x I2C Interrupt Handlers                           */
/******************************************************************************/
/**
 * @brief  This function handles I2C2 interrupt requests
 * @param  None
 * @retval None
 */
void I2C2_IRQHandler(void) {
  static uint8_t sensor = 0u;
  static uint32_t t0_ms5611 = 0u; // timer for MS5611 [msec]

  if (sensor == 0u) {
    // grab data only when it's ready (i.e. check conversion period)
    if (GetTime() - t0_ms5611 > MS5611_GetConversionPeriod()) {
      if (MS5611_ReadSensorIT()) {
        t0_ms5611 = GetTime();

        if (!I2C_ITStop(I2C2))
          I2C_GenerateSTART(I2C2, ENABLE);
        sensor = 0u;
      }
      return;
    } else {
      sensor = 0u; // data is not ready yet, so go directly to the next sensor
    }
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
 * @brief  This function handles UART1 DMA transmitter interrupt requests
 * @param  None
 * @retval None
 */
void UART1_DMA_TxIRQHandler(void) { UART_TxISR_DMA(1); }

/******************************************************************************/
/*                 STM32F40x TIM Interrupt Handlers                           */
/******************************************************************************/
/**
 * @brief  This function handles TIM2 interrupt requests
 * @param  None
 * @retval None
 */
const uint16_t DISPLAY_PERIOD = 20u; // [msec]
uint32_t t0_display = 0u;            // [msec]
void TIM2_IRQHandler(void) {
  if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET) {
    LED_Blink(1, 100);

    // READ ALL SENSOR DATA (1 kHz)
    static float dt_max = 0.0f;
    float dt;
    float t0 = GetTimeU();
    MS5611_ReadSensor();
    dt = GetTimeU() - t0;
    if (dt > dt_max)
      dt_max = dt;

    // PUBLISH TELEMETRY DATA (50 Hz)
    if (GetTime() - t0_display > DISPLAY_PERIOD) {
      t0_display = GetTime();
      char str[100];
      sprintf(str, "%.1f %+.3f %+.3f %+.3f\r\n", GetTimeU(),
              MS5611_GetPressure(), MS5611_GetTemperature(), dt_max);
      UART_Print(1, str);
    }

    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
  }
}
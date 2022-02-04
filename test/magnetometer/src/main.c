/**
 * @file    main.c
 * @author  Raymond Oung
 * @date    2014.05.13
 * @brief   Main program
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

#include "arm_math.h"
#include "main.h"
#include "nvic_list.h"
#include "stm32f4xx.h"

#include "can.h"
#include "clock.h"
#include "i2c.h"
#include "led.h"
#include "spi.h"
#include "uart.h"

#include "rm3100.h"

/** @addtogroup Main
 * @{
 */

/** @defgroup Private_Defines
 * @{
 */
#ifdef _VERBOSE
#define VERBOSE_UART_ID 1u
#endif

#define W00 0x0000
#define W01 W00 + sizeof(float)
#define W02 W01 + sizeof(float)
#define W10 W02 + sizeof(float)
#define W11 W10 + sizeof(float)
#define W12 W11 + sizeof(float)
#define W20 W12 + sizeof(float)
#define W21 W20 + sizeof(float)
#define W22 W21 + sizeof(float)

#define V0 0x1000
#define V1 V0 + sizeof(float)
#define V2 V1 + sizeof(float)
/**
 * @}
 */

/** @defgroup Private_Variables
 * @{
 */
static float32_t W_f32[9] = {
    1.0f, 0.0f, 0.0f, 0.0f, 1.0f,
    0.0f, 0.0f, 0.0f, 1.0f}; // soft-iron calibration matrix
static float32_t V_f32[3] = {+295.485078f, +203.530330f,
                             +71.393517f}; // hard-iron calibration matrix
/**
 * @}
 */

/** @defgroup Private_Function_Prototypes
 * @{
 */
void ReportError(arm_status status);
void RCC_Configuration(void);
void GPIO_Configuration(void);
void NVIC_Configuration(void);
/**
 * @}
 */

/**
 * @brief  Main program
 * @param  None
 * @retval None
 */
int main(void) {
  // At this stage the microcontroller clock setting is already configured,
  // this is done through SystemInit() function which is called from startup
  // file (startup_stm32f4xx.s) before to branch to application main.
  // To reconfigure the default setting of SystemInit() function, refer to
  // system_stm32f4xx.c file

  //========================================================================
  // INITIALISE PERIPHERALS
  //========================================================================
  RCC_Configuration();
  GPIO_Configuration();
  NVIC_Configuration();

  CLOCK_Init();
  LED_Init(LED1_CLK, LED1_PORT, LED1_PIN);
  LED_OnAll();
  Delay(500);
  LED_OffAll();

  SPI_PeriphInit(SPI1, SPI_BAUD_SLOW, SPI_CPOL0, SPI_CPHA0, SPI1_MOSI_CLK,
                 SPI1_MOSI_PORT, SPI1_MOSI_PIN, SPI1_MOSI_PIN_SRC,
                 SPI1_MISO_CLK, SPI1_MISO_PORT, SPI1_MISO_PIN,
                 SPI1_MISO_PIN_SRC, SPI1_SCLK_CLK, SPI1_SCLK_PORT,
                 SPI1_SCLK_PIN, SPI1_SCLK_PIN_SRC);

  I2C_PeriphInit(I2C2, I2C2_SPEED, I2C2_SDA_CLK, I2C2_SDA_PORT, I2C2_SDA_PIN,
                 I2C2_SDA_PIN_SOURCE, I2C2_SCL_CLK, I2C2_SCL_PORT, I2C2_SCL_PIN,
                 I2C2_SCL_PIN_SOURCE, I2C2_NVIC_PREEMPTION_PRIORITY);

  CAN_PeriphInit(CAN1, CAN1_FILT_NUM, CAN1_GPIO_CLK, CAN1_CLK, CAN1_TX_PORT,
                 CAN1_TX_PIN, CAN1_TX_PIN_SOURCE, CAN1_RX_PORT, CAN1_RX_PIN,
                 CAN1_RX_PIN_SOURCE, CAN1_RX_IRQn,
                 CAN1_RX_NVIC_PREEMPTION_PRIORITY, CAN1_TX_IRQn,
                 CAN1_TX_NVIC_PREEMPTION_PRIORITY);

  UART_PeriphInit(UART1, UART1_BAUD, UART1_DMA_BUF_SIZE, UART1_TX_CLK,
                  UART1_TX_PORT, UART1_TX_PIN, UART1_TX_PIN_SRC, UART1_RX_CLK,
                  UART1_RX_PORT, UART1_RX_PIN, UART1_RX_PIN_SRC,
                  UART1_DMA_RX_NVIC_PREEMPTION_PRIORITY,
                  UART1_DMA_TX_NVIC_PREEMPTION_PRIORITY,
                  UART1_IT_RX_NVIC_PREEMPTION_PRIORITY,
                  UART1_IT_TX_NVIC_PREEMPTION_PRIORITY);

  RM3100_Init(RM3100_SPI, RM3100_CLK, RM3100_PORT, RM3100_PIN);
  // RM3100_BuiltInSelfTest();
  // char out[100];
  // sprintf(out, "RevID: 0x%02X\r\n", RM3100_ReadRevID());
  // UART_Print(VERBOSE_UART_ID, out);
  // while (1) {
  //   Delay(100);
  //   RM3100_InitSingleMeasurement();
  //   char str[100];
  //   sprintf(str, "%u: %u %u %u\r\n", GetTime(), RM3100_GetMagnX(),
  //           RM3100_GetMagnY(), RM3100_GetMagnZ());
  //   UART_Print(VERBOSE_UART_ID, str);
  // }
  //========================================================================

  // set up some matrices for calibration purposes
  arm_matrix_instance_f32 W;   // soft-iron calibration matrix
  arm_matrix_instance_f32 V;   // hard-iron calibration vector
  arm_matrix_instance_f32 B;   // magnetic field vector
  arm_matrix_instance_f32 BV;  // magnetic field vector with hard-iron
                               // compensation
  arm_matrix_instance_f32 BVW; // magnetic field vector with hard-iron
                               // and soft-iron compensation

  float32_t B_f32[3];
  float32_t BV_f32[3];
  float32_t BVW_f32[3];

  arm_mat_init_f32(&W, 3u, 3u, (float32_t *)W_f32);
  arm_mat_init_f32(&V, 3u, 1u, (float32_t *)V_f32);
  arm_mat_init_f32(&B, 3u, 1u, (float32_t *)B_f32);
  arm_mat_init_f32(&BV, 3u, 1u, (float32_t *)BV_f32);
  arm_mat_init_f32(&BVW, 3u, 1u, (float32_t *)BVW_f32);

  uint32_t t0 = 0u;
  while (1) {
    LED_Blink(0, 100);

    if (GetTime() - t0 > 10u) {
      t0 = GetTime();

      if (RM3100_ISR()) {
        // read data from sensor
        B_f32[0] = (float)RM3100_GetMagnX();
        B_f32[1] = (float)RM3100_GetMagnY();
        B_f32[2] = (float)RM3100_GetMagnZ();

        // arm_mat_sub_f32(&B,&V,&BV); // hard-iron calibration
        // arm_mat_mult_f32(&W,&BV,&BVW); // soft-iron calibration

        // send data to CAN bus
        // <add code here>

#ifdef _VERBOSE
        char str[100];
        sprintf(str, "%u: %+.3f %+.3f %+.3f\r\n", GetTime(), B_f32[0], B_f32[1],
                B_f32[2]);
        UART_Print(VERBOSE_UART_ID, str);
#endif
      }
    }
  }
}

/**
 * @brief  Report ARM math errors.
 * @param  status arm_status data type
 * @retval None
 */
void ReportError(arm_status status) {
#ifdef _VERBOSE
  char str[100];
  switch (status) {
  case ARM_MATH_ARGUMENT_ERROR:
    UART_Print(VERBOSE_UART_ID,
               "ARGUMENT ERROR: One or more arguments are incorrect.\r\n");
    break;

  case ARM_MATH_LENGTH_ERROR:
    UART_Print(VERBOSE_UART_ID, "LENGTH ERROR: Size of matrices is not "
                                "compatible with the operation.\r\n");
    break;

  case ARM_MATH_SIZE_MISMATCH:
    UART_Print(VERBOSE_UART_ID, "SIZE MISMATCH: Size of matrices is not "
                                "compatible with the operation.\r\n");
    break;

  case ARM_MATH_NANINF:
    UART_Print(VERBOSE_UART_ID,
               "NANINF: Not-a-number (NaN) or infinity is generated.\r\n");
    break;

  case ARM_MATH_SINGULAR:
    UART_Print(VERBOSE_UART_ID,
               "SINGULAR: Not-a-number (NaN) or infinity is generated.\r\n");
    break;

  case ARM_MATH_TEST_FAILURE:
    UART_Print(VERBOSE_UART_ID, "TEST FAILURE: Test Failed.\r\n");
    break;

  default:
    sprintf(str, "Unknown error, code: %d\r\n", status);
    UART_Print(VERBOSE_UART_ID, str);
    break;
  }
#endif
  while (1)
    ;
}

/**
 * @brief  Configures the different system clocks.
 * @param  None
 * @retval None
 */
void RCC_Configuration(void) {
  // Setup the microcontroller system. Initialize the Embedded Flash Interface,
  // initialize the PLL and update the System Frequency variable.
  SystemInit();

  // Peripheral Clock 1 (PCLK1 = APB1) (168 MHz Max.):
  RCC_PCLK1Config(RCC_HCLK_Div4); // set to 42 MHz

  // Peripheral Clock 2 (PCLK2 = APB2) (168 MHz Max.):
  RCC_PCLK2Config(RCC_HCLK_Div4); // set to 42 MHz
}

/**
 * @brief  Configures the different GPIO ports.
 * @param  None
 * @retval None
 */
void GPIO_Configuration(void) {
  GPIO_InitTypeDef GPIO_InitStructure;

  // Configure all unused GPIO port pins in Analog Input mode (floating input
  // trigger OFF), this will reduce the power consumption and increase the
  // device immunity against EMI/EMC
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;

  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  GPIO_Init(GPIOD, &GPIO_InitStructure);
  GPIO_Init(GPIOE, &GPIO_InitStructure);
}

/**
 * @brief  Configures the nested vectored interrupt controller.
 * @param  None
 * @retval None
 */
void NVIC_Configuration(void) {
  // Set Priority Group
  NVIC_PriorityGroupConfig(NVIC_PRIORITY_GROUP);
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line) {
  // User can add his own implementation to report the file name and line
  // number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
  // line)
  while (1)
    ;
}
#endif
/**
 * @}
 */

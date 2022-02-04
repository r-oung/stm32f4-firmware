/**
 * @file    nvic_list.h
 * @author  Raymond Oung
 * @date    2014.10.24
 * @brief   NVIC preemption priorities
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
#ifndef __NVIC_LIST_H
#define __NVIC_LIST_H

#ifdef __cplusplus
extern "C" {
#endif

/** @addtogroup Source
 * @{
 */

/** @addtogroup Config
 * @{
 */

/** @defgroup NVIC_Priority_Table
 * @{
 */

/**
@code  
 The table below gives the allowed values of the pre-emption priority and subpriority according
 to the Priority Grouping configuration performed by NVIC_PriorityGroupConfig function
  ============================================================================================================================
    NVIC_PriorityGroup   | NVIC_IRQChannelPreemptionPriority | NVIC_IRQChannelSubPriority  | Description
  ============================================================================================================================
   NVIC_PriorityGroup_0  |                0                  |            0-15             |   0 bits for pre-emption priority
                         |                                   |                             |   4 bits for subpriority
  ----------------------------------------------------------------------------------------------------------------------------
   NVIC_PriorityGroup_1  |                0-1                |            0-7              |   1 bits for pre-emption priority
                         |                                   |                             |   3 bits for subpriority
  ----------------------------------------------------------------------------------------------------------------------------    
   NVIC_PriorityGroup_2  |                0-3                |            0-3              |   2 bits for pre-emption priority
                         |                                   |                             |   2 bits for subpriority
  ----------------------------------------------------------------------------------------------------------------------------    
   NVIC_PriorityGroup_3  |                0-7                |            0-1              |   3 bits for pre-emption priority
                         |                                   |                             |   1 bits for subpriority
  ----------------------------------------------------------------------------------------------------------------------------    
   NVIC_PriorityGroup_4  |                0-15               |            0                |   4 bits for pre-emption priority
                         |                                   |                             |   0 bits for subpriority                       
  ============================================================================================================================
@endcode
*/

/**
 * @}
 */
#define NVIC_PRIORITY_GROUP                                                    \
  NVIC_PriorityGroup_4 // This priority can be used in the event that you want
                       // an interrupt to pre-empt another interrupt.

#define CLOCK_NVIC_PREEMPTION_PRIORITY (0x00)

#define UART1_DMA_RX_NVIC_PREEMPTION_PRIORITY (0x01)
#define UART1_DMA_TX_NVIC_PREEMPTION_PRIORITY (0x07)
#define UART1_IT_RX_NVIC_PREEMPTION_PRIORITY (0x07)
#define UART1_IT_TX_NVIC_PREEMPTION_PRIORITY (0x07)

#define UART4_DMA_RX_NVIC_PREEMPTION_PRIORITY (0x02)
#define UART4_DMA_TX_NVIC_PREEMPTION_PRIORITY (0x07)
#define UART4_IT_RX_NVIC_PREEMPTION_PRIORITY (0x02)
#define UART4_IT_TX_NVIC_PREEMPTION_PRIORITY (0x02)

#define I2C1_NVIC_PREEMPTION_PRIORITY (0x07)

#define I2C2_NVIC_PREEMPTION_PRIORITY (0x07)

#define SDIO_DMA_NVIC_PREEMPTION_PRIORITY (0x06)
#define SDIO_IT_NVIC_PREEMPTION_PRIORITY (0x06)

#define CAN1_RX_NVIC_PREEMPTION_PRIORITY (0x02)
#define CAN1_TX_NVIC_PREEMPTION_PRIORITY (0x02)

#define CAN2_RX_NVIC_PREEMPTION_PRIORITY (0x02)
#define CAN2_TX_NVIC_PREEMPTION_PRIORITY (0x02)

#define TIM2_NVIC_PREEMPTION_PRIORITY (0x01)

#ifdef __cplusplus
}
#endif

#endif /* __NVIC_LIST_H */
/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

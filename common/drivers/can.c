/**
 * @file    can.c
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

/* Includes ------------------------------------------------------------------*/
#include "can.h"

/** @addtogroup Source
 * @{
 */

/** @addtogroup Low_Level
 * @{
 */

/** @defgroup CAN
 * @brief CAN setup
 * @{
 */
/** @defgroup CAN_Private_Defines
 * @{
 */
// #define CAN_BAUDRATE_100 // 100 kbps
// #define CAN_BAUDRATE_500 // 500 kbps; maximum recommended speed without
                            // twisted-cable and shielding
#define CAN_BAUDRATE_1000 // 1000 kbps; maximum recommended speed with
                          // twisted-cable and shielding

#define CAN_BUFFER_SIZE 128

typedef struct {
  uint8_t Tx[CAN_BUFFER_SIZE];
  uint8_t Rx[CAN_BUFFER_SIZE];
  uint8_t TxPtr;
  uint8_t RxPtr;
  uint8_t TxPtrEnd;
  uint8_t RxPtrEnd;
  uint32_t id[CAN_BUFFER_SIZE];
} CAN_buffer_t;

CAN_buffer_t CAN_buffer[2];

/**
 * @}
 */

/** @defgroup CAN_Private_Functions
 * @{
 */
static CAN_TypeDef *CAN_id2type(uint8_t p);
static uint8_t CAN_id2buf(uint8_t id);
static uint8_t CAN_type2buf(CAN_TypeDef *CANx);
static void CAN_SendSingleFrame(uint8_t p);
/**
 * @}
 */

/**
 * @brief  Configures the CAN.
 * @param  None
 * @retval None
 */
void CAN_PeriphInit(CAN_TypeDef *CANx, uint8_t filt_num, uint32_t clk,
                    uint32_t pclk, GPIO_TypeDef *tx_port, uint16_t tx_pin,
                    uint8_t tx_pin_src, GPIO_TypeDef *rx_port, uint16_t rx_pin,
                    uint8_t rx_pin_src, uint8_t rxIRQn,
                    uint8_t rxPremptPriority, uint8_t txIRQn,
                    uint8_t txPremptPriority) {
  uint8_t n = CAN_type2buf(CANx);

  RCC_AHB1PeriphClockCmd(clk, ENABLE);  // GPIO clock
  RCC_APB1PeriphClockCmd(pclk, ENABLE); // CAN clock

  if (CANx == CAN1) {
    GPIO_PinAFConfig(tx_port, tx_pin_src, GPIO_AF_CAN1);
    GPIO_PinAFConfig(rx_port, rx_pin_src, GPIO_AF_CAN1);
  }

  if (CANx == CAN2) {
    GPIO_PinAFConfig(tx_port, tx_pin_src, GPIO_AF_CAN2);
    GPIO_PinAFConfig(rx_port, rx_pin_src, GPIO_AF_CAN2);
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
  NVIC_InitStructure.NVIC_IRQChannel = rxIRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = rxPremptPriority;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = txIRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = txPremptPriority;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  // CAN initialization
  CAN_InitTypeDef CAN_InitStructure;
  CAN_FilterInitTypeDef CAN_FilterInitStructure;

  CAN_DeInit(CANx);
  CAN_StructInit(&CAN_InitStructure);
  CAN_InitStructure.CAN_TTCM = DISABLE; // time-triggered communication mode
  CAN_InitStructure.CAN_ABOM = DISABLE; // automatic bus-off management mode
  CAN_InitStructure.CAN_AWUM = DISABLE; // automatic wake-up mode
  CAN_InitStructure.CAN_NART = DISABLE; // non-automatic retransmission mode
  CAN_InitStructure.CAN_RFLM = DISABLE; // receive FIFO locked mode
  CAN_InitStructure.CAN_TXFP = DISABLE; // transmit FIFO priority
  CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;
  // CAN_InitStructure.CAN_Mode = CAN_Mode_LoopBack; // DEBUG

  // To calculate CAN bit-rate:
  // Note1: to read clock frequencies, RCC_GetClocksFreq()
  // Note2: CAN_CLK by default is 168 MHz; should be set to 42 MHz if
  // RCC_PCLK1Config(RCC_HCLK_Div4) in main.c CAN_BitRate = CAN_CLK /
  // (CAN_Prescaler * (CAN_SJW + CAN_BS1 + CAN_BS2))
#if defined(CAN_BAUDRATE_100)
  CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;  // synchronization jump width = 1
  CAN_InitStructure.CAN_BS1 = CAN_BS1_14tq; // 14/16 time quanta
  CAN_InitStructure.CAN_BS2 = CAN_BS2_5tq;  // 5/8 time quanta
  CAN_InitStructure.CAN_Prescaler = 21;
#elif defined(CAN_BAUDRATE_500)
  CAN_InitStructure.CAN_SJW = CAN_SJW_1tq; // synchronization jump width = 1
  CAN_InitStructure.CAN_BS1 = CAN_BS1_7tq; // 7/16 time quanta
  CAN_InitStructure.CAN_BS2 = CAN_BS2_4tq; // 4/8 time quanta
  CAN_InitStructure.CAN_Prescaler = 7;
#elif defined(CAN_BAUDRATE_1000)
  CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;  // synchronization jump width = 1
  CAN_InitStructure.CAN_BS1 = CAN_BS1_14tq; // 6:16 time quanta
  CAN_InitStructure.CAN_BS2 = CAN_BS2_6tq;  // 2:8 time quanta
  CAN_InitStructure.CAN_Prescaler = 2;      // 42 MHz clock
#endif
  CAN_Init(CANx, &CAN_InitStructure);

  // CAN filter initialiation
  CAN_FilterInitStructure.CAN_FilterNumber =
      filt_num; // 0..13 for CAN1, 14..27 for CAN2
  CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
  CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
  CAN_FilterInitStructure.CAN_FilterIdHigh =
      0x0000; // used in identification list mode
  CAN_FilterInitStructure.CAN_FilterIdLow =
      0x0000; // used in identification list mode
  CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000; // used in mask mode
  CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;  // used in mask mode
  CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;
  CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
  CAN_FilterInit(&CAN_FilterInitStructure);

  // set interrupts
  CAN_ITConfig(CANx, CAN_IT_FMP0, ENABLE); // FIFO 0 message pending interrupt
  CAN_ITConfig(CANx, CAN_IT_TME, DISABLE); // transmit mailbox empty interrupt

  // Reset CAN buffers
  uint8_t i;
  for (i = 0; i < CAN_BUFFER_SIZE; i++) {
    CAN_buffer[n].Tx[i] = 0u;
    CAN_buffer[n].Rx[i] = 0u;
  }
  CAN_buffer[n].TxPtr = 0u;
  CAN_buffer[n].RxPtr = 0u;
  CAN_buffer[n].TxPtrEnd = 0u;
  CAN_buffer[n].RxPtrEnd = 0u;
}

/**
 * @brief  CAN peripheral ID to type
 * @param  p Peripheral ID
 * @retval CAN_TypeDef * [CAN1,CAN2]
 */
static CAN_TypeDef *CAN_id2type(uint8_t p) {
  CAN_TypeDef *CANx;
  CANx = CAN1;

  switch (p) {
  case 1:
    CANx = CAN1;
    break;
  case 2:
    CANx = CAN2;
    break;
  default:
    break;
  }

  return CANx;
}

/**
 * @brief  CAN identifier to buffer index
 * @param  id CAN identifier [1,2]
 * @retval Buffer index [0,1]
 */
static uint8_t CAN_id2buf(uint8_t id) {
  uint8_t b = 0u;

  switch (id) {
  case 1:
    b = 0u;
    break;
  case 2:
    b = 1u;
    break;
  default:
    break;
  }

  return b;
}

/**
 * @brief  CAN type to buffer index
 * @param  CANx UART type [CAN1,CAN2]
 * @retval Buffer index [0,1]
 */
static uint8_t CAN_type2buf(CAN_TypeDef *CANx) {
  uint8_t b = 0u;

  if (CANx == CAN1)
    b = 0u;
  if (CANx == CAN2)
    b = 1u;

  return b;
}

/**
 * @brief Write data to CAN bus
 * @param p CAN peripheral ID
 * @param id CAN frame identifier
 * @param data Pointer to data to be sent
 * @param size Size of data to be sent [bytes]
 * @retval Bytes written
 */
uint8_t CAN_Write(uint8_t p, uint32_t id, void *buf, uint8_t size) {
  uint8_t written = 0u;
  uint8_t n = CAN_id2buf(p);

  // copy data to buffer only if it does not exceed the buffer size
  // NOTE: this is not a circular buffer
  if (CAN_buffer[n].TxPtrEnd + size < CAN_BUFFER_SIZE) {
    uint8_t i;
    for (i = 0; i < size; i++) {
      CAN_buffer[n].Tx[CAN_buffer[n].TxPtrEnd + i] = *((uint8_t *)buf + i);
      CAN_buffer[n].id[CAN_buffer[n].TxPtrEnd + i] = id;
    }
    CAN_buffer[n].TxPtrEnd += size; // update buffer counter
    written = size;

    if (CAN_buffer[n].TxPtr == 0u) {
      // if this value is zero, that means the last message has finished
      // transmitting
      CAN_SendSingleFrame(p);
    }
  } else // reset buffer
  {
    CAN_buffer[n].TxPtr = 0u;
    CAN_buffer[n].TxPtrEnd = 0u;
  }

  // send the rest of the frames (if any) using interrupts
  return written;
}

/**
 * @brief CAN transmit interrupt service routine
 *        Writes one byte to the transmit data register and disables
 *        the transmission interrupt when the buffer is empty
 * @param CANx CAN peripheral number
 * @retval None
 */
void CAN_TxISR(uint8_t p) {
  uint8_t n = CAN_id2buf(p);

  // disable transmit mailbox empty interrupt so that it doesn't keep calling
  // this
  CAN_ITConfig(CAN_id2type(p), CAN_IT_TME, DISABLE);

  // check if we've reached the end of the buffer
  if (CAN_buffer[n].TxPtr == CAN_buffer[n].TxPtrEnd) {
    // reset pointers
    CAN_buffer[n].TxPtr = 0u;
    CAN_buffer[n].TxPtrEnd = 0u;
    return;
  }

  CAN_SendSingleFrame(p);
}

/**
 * @brief Send a single CAN frame
 * @param CANx CAN peripheral number
 * @retval None
 */
static void CAN_SendSingleFrame(uint8_t p) {
  uint8_t n = CAN_id2buf(p);
  uint32_t id = CAN_buffer[n].id[CAN_buffer[n].TxPtr];

  CanTxMsg TxMessage;
  TxMessage.IDE =
      CAN_Id_Standard;  // use standard frame [CAN_Id_Standard, CAN_Id_Extended]
  TxMessage.StdId = id; // Standard Node ID [0 to 0x1FF]
  TxMessage.RTR =
      CAN_RTR_Data; // transmit data frame [CAN_RTR_Data, CAN_RTR_Remote]
  TxMessage.DLC = 0u;

  // send data
  while ((TxMessage.DLC < 8u) &&
         (CAN_buffer[n].TxPtr < CAN_buffer[n].TxPtrEnd) &&
         id == CAN_buffer[n].id[CAN_buffer[n].TxPtr]) // send at most 8 bytes
  {
    TxMessage.Data[TxMessage.DLC++] = CAN_buffer[n].Tx[CAN_buffer[n].TxPtr++];
  }

  CAN_Transmit(CAN_id2type(p), &TxMessage);
  CAN_ITConfig(CAN_id2type(p), CAN_IT_TME,
               ENABLE); // transmit mailbox empty interrupt
}

// NOTE: Read TEC and REC for CAN bus health
/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/**
 * @file    dw1000.c
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

/**
 * 
 * Operational design choice when employing DW1000 for real-time localisation
 * systems See Section 9, (pg. 191/213) of DW1000 User Manual 2.01 Note:
 * - Free space LOS: 60 m at 6.8 Mbps, 250 m at 110 kbps
 * - Lower centre frequency gets more range than higher centre frequency
 * - Wider bandwidth channels (e.g. CH-4/-6) have more range than standard 500
 * MHz
 *   because more energy can be sent at a given dBm/MHz regulatory limit,
 *   but increases power consumption
 * - Channel centre frequency and bandwidth depends on regional regulations,
 *   see Table 57 in Section 10.5 to see a list of channels supported by DW1000
 * - Longer preamble gives improved range performance and better first path time
 * of arrival
 *   information while a shorter preamble gives a shorter air time and saves
 *   power
 * - Higher PRF gives more accuracy on the first path timestamp and slightly
 *   improved operating range, however requires more power
 *
 * IEEE 802.15.4 UWB PHY has 16 defined channel/bands:
 * ref: http://en.wikipedia.org/wiki/List_of_UWB_channels
 * DW1000 supports a subset of these, which are listed in Table 57
 * Available Channels: 1-5, 7 (3.4 GHz to 4.5 GHz, 6.5 GHz)
 * Available PRF: 1-24
 * @TODO Use Dynamic Preamble Select (DPS), which is intended as a security
 * mechanism for two-way ranging, where devices switch to using one of the DPS
 * specific preamble codes for the ranging exchange, and perhaps a different one
 * for each direction of communication
 *
 * Spectrum Regulation for Countries:
 * Ref: http://infoscience.epfl.ch/record/148347/files/EPFL_TH4719.pdf (pg. 28)
 * USA:    - 3.1 GHz to 10.6 GHz
 *         - max. power emission limit of -41.3 dBm/MHz (indoor and outdoor)
 *         - emission between 0.96 GHz and 1.61 GHz must be limited to -75
 *         dBm/MHz
 * Europe: - equipment must cease transmission withint 10 sec unless they
 * receive acknowledgement
 *           from an associated transceiver that its transmission is being
 *           received
 *         - can not be used in a fixed outdoor location or connected to a fixed
 *         outdoor antenna or
 *           in vehicles
 *         - low duty cycle considerations
 * Japan:  - two bands: 3.4 GHz to 4.8 GHz and 7.25 GHz to 10.25 GHz
 *         - devices operating in the 3.4 GHz to 4.8 GHz range should use Detect
 *         and Avoid (DAA)
 *         - for devices not equipped with interferrence mitigation techniques,
 *         average power
 *           shall be -70 dBm/MHz and peak power shall be -64 dBm/MHz in 3.4 GHz
 *           to 4.8 GHz
 *         - second band (7.25 GHz to 10.25 GHz) does not require DAA
 *         - average power spectral density limited to -41.3 dBm/MHz (indoors)
 *
 * Summary
 * DW1000 can be used...
 * USA: All channels
 * Europe: Ch-1 to Ch-4 at -70 dBm/MHz, Ch-5/-7 at -41.3 dBm/MHz
 * Japan: Ch-1 to Ch-4 at -70 dB/MHz or at -41.3 dBm/MHz with DAA/LDC
 *
 * UWB Channels
 * See Section 9, pg. 191/213 of DW1000 User Manual 2.01
 *
 * Ch-1
 * Center Frequency: 3494.4 MHz
 * Bandwidth: 499.2 MHz
 * Preamble Codes: 1, 2 (16 MHz); 9, 10, 11, 12 (64 MHz)
 *
 * Ch-2
 * Center Frequency: 3993.6 MHz
 * Bandwidth: 499.2 MHz
 * Preamble Codes: 3, 4 (16 MHz); 9, 10, 11, 12 (64 MHz)
 *
 * Ch-3
 * Center Frequency: 4492.8 MHz
 * Bandwidth: 499.2 MHz
 * Preamble Codes: 5, 6 (16 MHz); 9, 10, 11, 12 (64 MHz)
 *
 * Ch-4
 * Center Frequency: 3993.6 MHz
 * Bandwidth: 1331.2 MHz
 * Preamble Codes: 7, 8 (16 MHz); 17, 18, 19, 20 (64 MHz)
 *
 * Ch-5
 * Center Frequency: 6489.6 MHz
 * Bandwidth: 499.2 MHz
 * Preamble Codes: 3, 4 (16 MHz); 9, 10, 11, 12 (64 MHz)
 *
 * Ch-7
 * Center Frequency: 6489.6 MHz
 * Bandwidth: 1081.6 MHz
 * Preamble Codes: 7, 8 (16 MHz); 17, 18, 19, 20 (64 MHz)
 *
 * 6.8 Mbps Preamble Length: 64, 128, 256
 * 850 kbps Preamble Length: 256, 512, 1024
 * 110 kbps Preamble Length: 2048, 4096
 */

/* Includes ------------------------------------------------------------------*/
#include "dw1000.h"
#include "clock.h"
#include "deca_regs.h"
#include "spi.h"
#include <math.h>

#include "led.h" // debug

#ifdef _VERBOSE
#include "main.h"
#include "uart.h"
#include <stdio.h>
#endif

/** @addtogroup Source
 * @{
 */

/** @addtogroup Low_Level
 * @{
 */

/** @defgroup DW1000
 * @brief This file provides firmware functions to manage DW1000s.
 * @{
 */
/** @defgroup DW1000_Private_Defines
 * @{
 */
#ifdef _VERBOSE
#define VERBOSE_UART_ID 4u
#endif

#define DEVICE_ID                                                              \
  0xDECA0130 // factory set device ID, used for device verification and SPI
             // functionality
#define CONFIG_RETRY 5u // number of configuration attempts to be made

// extended unique identifier
#define OUI 0x112233     // 24-bit manufacturer company ID
#define EID 0x1122334455 // 40-bit extension ID
/**
 * @}
 */

/** @defgroup DW1000_Private_Variables
 * @{
 */
typedef enum _DW1000_pac_size_t {
  PAC_SIZE_8 = 8u,
  PAC_SIZE_16 = 16u,
  PAC_SIZE_32 = 32u,
  PAC_SIZE_64 = 64u
} DW1000_pac_size_t;

static struct _DW1000_dev_id_t {
  uint8_t rev;
  uint8_t ver;
  uint8_t model;
  uint16_t ridtag;
} DW1000_dev_id;

static struct _DW1000_config_t {
  DW1000_channel_t channel;
  DW1000_data_rate_t data_rate;
  DW1000_prf_t prf;
  DW1000_preamble_code_t preamble_code;
  DW1000_preamble_length_t preamble_length;
  DW1000_pac_size_t pac_size;
} DW1000_config;

static struct _DW1000_rx_sinfo_t {
  uint16_t preamble_count;
  uint16_t std_noise;
  uint16_t fp_index;
  uint16_t fp_ampl1;
  uint16_t fp_ampl2;
  uint16_t fp_ampl3;
  uint16_t cir_pwr;
} DW1000_rx_sinfo;

static SPI_TypeDef *DW1000_SPIx;

static GPIO_TypeDef *DW1000_EN_port;
static uint16_t DW1000_EN_pin;

static GPIO_TypeDef *DW1000_CS_port;
static uint16_t DW1000_CS_pin;

static GPIO_TypeDef *DW1000_WAKE_port;
static uint16_t DW1000_WAKE_pin;

static GPIO_TypeDef *DW1000_NRST_port;
static uint16_t DW1000_NRST_pin;

static uint64_t DW1000_rx_timestamp;
static uint64_t DW1000_tx_timestamp;
/**
 * @}
 */

/** @defgroup DW1000_Private_Functions
 * @{
 */
void DW1000_EN(uint8_t val);
void DW1000_CS(uint8_t val);
void DW1000_WAKE(uint8_t val);
void DW1000_Reset(void);

uint8_t DW1000_ReadByte(void);
uint8_t DW1000_WriteByte(uint8_t byte);

void DW1000_ShortPause(void);
void DW1000_Read(uint8_t id, uint16_t sub_addr, uint8_t *buf, uint16_t size);
void DW1000_Write(uint8_t id, uint16_t sub_addr, void *buf, uint16_t size);

uint32_t DW1000_ReadDevID(void);

void DW1000_ConfigEUI(uint32_t oui, uint64_t eid);
void DW1000_ConfigPAN(uint16_t pan_id, uint16_t short_address);
void DW1000_ConfigSys(void);
void DW1000_ConfigTxFCTRL(uint8_t size);
void DW1000_ConfigRxTimeout(uint8_t timeout);
void DW1000_ConfigSniffMode(float on_off_ratio);
void DW1000_ConfigSmartTxPowerControl(void);
void DW1000_ConfigManualTxPowerControl(void);
void DW1000_ConfigTxPowerControl(void);
void DW1000_ConfigChannelControl(void);
void DW1000_ConfigSFDSequence(void);
void DW1000_ConfigAGC(void);
void DW1000_ConfigDRX(void);
void DW1000_ConfigAnalogRFBlock(void);
void DW1000_ConfigTxCalibBlock(void);
void DW1000_ConfigFreqSynthControlBlock(void);
void DW1000_ConfigLDE(void);
void DW1000_ConfigPMSC(void);

void DW1000_ConfigSysEventMask(void);
uint8_t DW1000_GetSysStatus(uint64_t status_mask);
void DW1000_ClearSysStatus(void);

uint32_t DW1000_ReadOTP(uint16_t reg);

void DW1000_LoadMicroCode(void);

void DW1000_ConfigLEDS(void);
/**
 * @}
 */

/**
 * @brief  Initialise DW1000
 * @param  None
 * @retval None
 */
void DW1000_Init(SPI_TypeDef *SPIx, uint32_t en_clk, GPIO_TypeDef *en_port,
                 uint16_t en_pin, uint32_t cs_clk, GPIO_TypeDef *cs_port,
                 uint16_t cs_pin, uint32_t wake_clk, GPIO_TypeDef *wake_port,
                 uint16_t wake_pin, uint32_t nrst_clk, GPIO_TypeDef *nrst_port,
                 uint16_t nrst_pin, uint32_t irq_clk, GPIO_TypeDef *irq_port,
                 uint16_t irq_pin, uint8_t irq_port_src, uint8_t irq_pin_src,
                 uint32_t irq_line, uint8_t irq_IRQn,
                 uint8_t irq_premptPriority) {
  RCC_AHB1PeriphClockCmd(en_clk, ENABLE);
  RCC_AHB1PeriphClockCmd(cs_clk, ENABLE);
  RCC_AHB1PeriphClockCmd(nrst_clk, ENABLE);
  RCC_AHB1PeriphClockCmd(irq_clk, ENABLE);

  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

  GPIO_InitStructure.GPIO_Pin = cs_pin;
  GPIO_Init(cs_port, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = en_pin;
  GPIO_Init(en_port, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = wake_pin;
  GPIO_Init(wake_port, &GPIO_InitStructure);

  // NRST is set to tri-state by default
  GPIO_InitStructure.GPIO_Pin = nrst_pin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(nrst_port, &GPIO_InitStructure);

  // IRQ set to Pull Down to prevent unnecessary EXT IRQ while DW1000 is in
  // SLEEP mode
  GPIO_InitStructure.GPIO_Pin = irq_pin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_Init(irq_port, &GPIO_InitStructure);

  SYSCFG_EXTILineConfig(irq_port_src, irq_pin_src);

  EXTI_InitTypeDef EXTI_InitStructure;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;

  EXTI_InitStructure.EXTI_Line = irq_line;
  EXTI_Init(&EXTI_InitStructure);

  NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = irq_premptPriority;
  NVIC_InitStructure.NVIC_IRQChannel = irq_IRQn;
  NVIC_Init(&NVIC_InitStructure);

  DW1000_SPIx = SPIx;
  DW1000_EN_port = en_port;
  DW1000_EN_pin = en_pin;
  DW1000_CS_port = cs_port;
  DW1000_CS_pin = cs_pin;
  DW1000_WAKE_port = wake_port;
  DW1000_WAKE_pin = wake_pin;

  // turn on power to DW1000
  DW1000_EN(1);
  Delay(1000);

  if (DW1000_ReadDevID() != DEVICE_ID) {
#ifdef _VERBOSE
    UART_Print(VERBOSE_UART_ID, "ERROR: Failed to configure device\r\n");
#endif
    LED_On(0);
    while (1)
      ;
  }

  DW1000_ConfigEUI(OUI, EID);
}

/**
 * @brief  Toggle enable
 * @param  val 1=High; 0=Low
 * @retval None
 */
void DW1000_EN(uint8_t val) {
  switch (val) {
  case 0u:
    GPIO_ResetBits(DW1000_EN_port, DW1000_EN_pin);
    break;
  case 1u:
    GPIO_SetBits(DW1000_EN_port, DW1000_EN_pin);
    break;
  default:
    break;
  }
}

/**
 * @brief  Toggle chip select
 * @param  val 1=High; 0=Low
 * @retval None
 */
void DW1000_CS(uint8_t val) {
  switch (val) {
  case 0u:
    GPIO_ResetBits(DW1000_CS_port, DW1000_CS_pin);
    break;
  case 1u:
    GPIO_SetBits(DW1000_CS_port, DW1000_CS_pin);
    break;
  default:
    break;
  }
}

/**
 * @brief  Toggle wake
 * @param  val 1=High; 0=Low
 * @retval None
 */
void DW1000_WAKE(uint8_t val) {
  switch (val) {
  case 0u:
    GPIO_ResetBits(DW1000_WAKE_port, DW1000_WAKE_pin);
    break;
  case 1u:
    GPIO_SetBits(DW1000_WAKE_port, DW1000_WAKE_pin);
    break;
  default:
    break;
  }
}

/**
 * @brief  Reset device
 * @param  None
 * @retval None
 */
void DW1000_Reset(void) {
  GPIO_InitTypeDef GPIO_InitStructure;

  // enable GPIO
  GPIO_InitStructure.GPIO_Pin = DW1000_NRST_pin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(DW1000_NRST_port, &GPIO_InitStructure);

  // drive NRST low for 10 nsec
  GPIO_ResetBits(DW1000_NRST_port, DW1000_NRST_pin);
  Delay(10);

  // NRST is set to tri-state by default
  GPIO_InitStructure.GPIO_Pin = DW1000_NRST_pin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(DW1000_NRST_port, &GPIO_InitStructure);
}

//--------------------------------------------------------------------------------

/**
 * @brief  Read a single byte
 * @param  None
 * @retval Value of the read byte
 */
uint8_t DW1000_ReadByte(void) { return DW1000_WriteByte(0x00); }

/**
 * @brief  Write a single byte
 * @param  byte byte to send
 * @retval Value of the received byte
 */
uint8_t DW1000_WriteByte(uint8_t byte) {
  // Loop while DR register in not empty
  while (SPI_I2S_GetFlagStatus(DW1000_SPIx, SPI_I2S_FLAG_TXE) == RESET)
    ;

  // Send byte through the SPI1 peripheral
  SPI_I2S_SendData(DW1000_SPIx, byte);

  // Wait to receive a byte
  while (SPI_I2S_GetFlagStatus(DW1000_SPIx, SPI_I2S_FLAG_RXNE) == RESET)
    ;

  // Return the byte read from the SPI bus
  return SPI_I2S_ReceiveData(DW1000_SPIx);
}

//--------------------------------------------------------------------------------

void DW1000_ShortPause(void) {
  // @TODO A short delay is required between consecutive read/writes
  float t0 = GetTimeU();
  while (GetTimeU() - t0 < .005f)
    ;
}

/**
 * @brief  Read from SPI
 * @param
 * @retval None
 */
void DW1000_Read(uint8_t id, uint16_t sub_addr, uint8_t *buf, uint16_t size) {
  DW1000_CS(0);

  // construct header
  if (sub_addr > 0x7F) // 3-octet header
  {
    uint8_t hdr[3];
    hdr[0] = 0x00 | 0x40 | id;
    hdr[1] = 0x80 | (0xFF & (sub_addr >> 0u));
    hdr[2] = 0x00 | (0xFF & (sub_addr >> 7u));

    DW1000_WriteByte(hdr[0]);
    DW1000_WriteByte(hdr[1]);
    DW1000_WriteByte(hdr[2]);
  } else if (sub_addr > 0x00) // 2-octet header
  {
    uint8_t hdr[2];
    hdr[0] = 0x00 | 0x40 | id;
    hdr[1] = 0x00 | (0xFF & (sub_addr >> 0u));

    DW1000_WriteByte(hdr[0]);
    DW1000_WriteByte(hdr[1]);
  } else // 1-octet header
  {
    uint8_t hdr = 0x00 | id;

    DW1000_WriteByte(hdr);
  }

  // read data
  uint16_t i;
  for (i = 0; i < size; i++) {
    buf[i] = DW1000_ReadByte();
  }

  DW1000_CS(1);
  DW1000_ShortPause();
}

/**
 * @brief  Write to SPI
 * @param
 * @retval None
 */
void DW1000_Write(uint8_t id, uint16_t sub_addr, void *buf, uint16_t size) {
  DW1000_CS(0);

  // construct and send header
  if (sub_addr > 0x7F) // 3-octet header
  {
    uint8_t hdr[3];
    hdr[0] = 0x80 | 0x40 | id;
    hdr[1] = 0x80 | (0xFF & (sub_addr >> 0u));
    hdr[2] = 0x00 | (0xFF & (sub_addr >> 7u));

    DW1000_WriteByte(hdr[0]);
    DW1000_WriteByte(hdr[1]);
    DW1000_WriteByte(hdr[2]);
  } else if (sub_addr > 0x00) // 2-octet header
  {
    uint8_t hdr[2];
    hdr[0] = 0x80 | 0x40 | id;
    hdr[1] = 0x00 | (0xFF & (sub_addr >> 0u));

    DW1000_WriteByte(hdr[0]);
    DW1000_WriteByte(hdr[1]);
  } else // 1-octet header
  {
    uint8_t hdr = 0x80 | id;

    DW1000_WriteByte(hdr);
  }

  // write data
  uint16_t i;
  for (i = 0; i < size; i++) {
    DW1000_WriteByte(*((uint8_t *)buf + i));
  }

  DW1000_CS(1);
  DW1000_ShortPause();
}

//--------------------------------------------------------------------------------

// 0=Failure, 1=Success
uint8_t DW1000_Config(DW1000_channel_t ch, DW1000_data_rate_t data_rate,
                      DW1000_prf_t prf, DW1000_preamble_code_t preamble_code,
                      DW1000_preamble_length_t preamble_length) {
  // Check preamble code
  // Preamble codes allows devices to operate simultaneously, as if on separate
  // channels Channels have recommended preamble codes, see pg. 200/213 of
  // DW1000 User Manual 2.01
  switch (prf) {
  case PRF_4MHZ:
    return 0u;
    break;

  case PRF_16MHZ: {
    switch (ch) {
    case CH1: {
      if (preamble_code != PREAMBLE_CODE_1 && preamble_code != PREAMBLE_CODE_2)
        return 0u;
    } break;

    case CH2: {
      if (preamble_code != PREAMBLE_CODE_3 && preamble_code != PREAMBLE_CODE_4)
        return 0u;
    } break;

    case CH3: {
      if (preamble_code != PREAMBLE_CODE_5 && preamble_code != PREAMBLE_CODE_6)
        return 0u;
    } break;

    case CH4: {
      if (preamble_code != PREAMBLE_CODE_7 && preamble_code != PREAMBLE_CODE_8)
        return 0u;
    } break;

    case CH5: {
      if (preamble_code != PREAMBLE_CODE_3 && preamble_code != PREAMBLE_CODE_4)
        return 0u;
    } break;

    case CH7: {
      if (preamble_code != PREAMBLE_CODE_7 && preamble_code != PREAMBLE_CODE_8)
        return 0u;
    } break;
    }
  } break;

  case PRF_64MHZ: {
    switch (ch) {
    case CH1: {
      if (preamble_code != PREAMBLE_CODE_9 &&
          preamble_code != PREAMBLE_CODE_10 &&
          preamble_code != PREAMBLE_CODE_11 &&
          preamble_code != PREAMBLE_CODE_12)
        return 0u;
    } break;

    case CH2: {
      if (preamble_code != PREAMBLE_CODE_9 &&
          preamble_code != PREAMBLE_CODE_10 &&
          preamble_code != PREAMBLE_CODE_11 &&
          preamble_code != PREAMBLE_CODE_12)
        return 0u;
    } break;

    case CH3: {
      if (preamble_code != PREAMBLE_CODE_9 &&
          preamble_code != PREAMBLE_CODE_10 &&
          preamble_code != PREAMBLE_CODE_11 &&
          preamble_code != PREAMBLE_CODE_12)
        return 0u;
    } break;

    case CH4: {
      if (preamble_code != PREAMBLE_CODE_17 &&
          preamble_code != PREAMBLE_CODE_18 &&
          preamble_code != PREAMBLE_CODE_19 &&
          preamble_code != PREAMBLE_CODE_20)
        return 0u;
    } break;

    case CH5: {
      if (preamble_code != PREAMBLE_CODE_9 &&
          preamble_code != PREAMBLE_CODE_10 &&
          preamble_code != PREAMBLE_CODE_11 &&
          preamble_code != PREAMBLE_CODE_12)
        return 0u;
    } break;

    case CH7: {
      if (preamble_code != PREAMBLE_CODE_17 &&
          preamble_code != PREAMBLE_CODE_18 &&
          preamble_code != PREAMBLE_CODE_19 &&
          preamble_code != PREAMBLE_CODE_20)
        return 0u;
    } break;
    }
  } break;
  }

  // Check preamble length
  // Preamble lengths affects operational range and accuracy of timestamps
  // Data rate have recommended preamble lengths, see pg. 192/213 of DW1000 User
  // Manual 2.01
  switch (data_rate) {
  case DATA_RATE_110: {
    if (preamble_length != PREAMBLE_LENGTH_2048 &&
        preamble_length != PREAMBLE_LENGTH_4096)
      return 0u;
  } break;

  case DATA_RATE_850: {
    if (preamble_length != PREAMBLE_LENGTH_256 &&
        preamble_length != PREAMBLE_LENGTH_512 &&
        preamble_length != PREAMBLE_LENGTH_1024)
      return 0u;
  } break;

  case DATA_RATE_6800: {
    if (preamble_length != PREAMBLE_LENGTH_64 &&
        preamble_length != PREAMBLE_LENGTH_128 &&
        preamble_length != PREAMBLE_LENGTH_256)
      return 0u;
  } break;
  }

  // Set PAC size
  // Preamble codes have an associated PAC size, see pg. 29/213 of DW1000 User
  // Manual 2.01
  uint8_t pac_size = 0u;
  switch (preamble_length) {
  case PREAMBLE_LENGTH_64:
    pac_size = PAC_SIZE_8;
    break;
  case PREAMBLE_LENGTH_128:
    pac_size = PAC_SIZE_8;
    break;
  case PREAMBLE_LENGTH_256:
    pac_size = PAC_SIZE_16;
    break;
  case PREAMBLE_LENGTH_512:
    pac_size = PAC_SIZE_16;
    break;
  case PREAMBLE_LENGTH_1024:
    pac_size = PAC_SIZE_32;
    break;
  case PREAMBLE_LENGTH_2048:
    pac_size = PAC_SIZE_64;
    break;
  case PREAMBLE_LENGTH_4096:
    pac_size = PAC_SIZE_64;
    break;
  }

  // Initialise DW1000 configurations
  DW1000_config.channel = ch;
  DW1000_config.data_rate = data_rate;
  DW1000_config.prf = prf;
  DW1000_config.preamble_code = preamble_code;
  DW1000_config.preamble_length = preamble_length;
  DW1000_config.pac_size = pac_size;

#ifdef _VERBOSE
  char str[128];
  UART_Print(VERBOSE_UART_ID, "---------------------------------\r\n");
  UART_Print(VERBOSE_UART_ID, "DW1000 CONFIGURATIONS:\r\n");
  sprintf(str,
          "Channel: %u\r\nData Rate: %u\r\nPRF: 0x%02x\r\nPreamble-Code: "
          "%u\r\nPreamble-Length: %u\r\nPAC-Size: %u\r\n",
          DW1000_config.channel, DW1000_config.data_rate, DW1000_config.prf,
          DW1000_config.preamble_code, DW1000_config.preamble_length,
          DW1000_config.pac_size);
  UART_Print(VERBOSE_UART_ID, str);
  UART_Print(VERBOSE_UART_ID, "---------------------------------\r\n");
#endif

  // Configure all sub-systems
  DW1000_ConfigPAN(0x1234, 0x5678); // @TODO set in DW1000_Config()?

  DW1000_ConfigSys();
  DW1000_ConfigTxFCTRL(0u);
  DW1000_ConfigRxTimeout(2u); // [msec]
  // DW1000_ConfigSniffMode(2.0f); // [on/off ratio]
  DW1000_ConfigTxPowerControl();
  DW1000_ConfigChannelControl();
  DW1000_ConfigSFDSequence();
  DW1000_ConfigAGC();
  DW1000_ConfigDRX();
  DW1000_ConfigAnalogRFBlock();
  DW1000_ConfigTxCalibBlock();
  DW1000_ConfigFreqSynthControlBlock();
  DW1000_ConfigLDE();
  DW1000_ConfigPMSC();

  // Load LDE microcode, needed for RTLS
  DW1000_LoadMicroCode();

  // Configure interrupts
  DW1000_ConfigSysEventMask();

  // Configure LEDs
  DW1000_ConfigLEDS();

#ifdef _VERBOSE
  UART_Print(VERBOSE_UART_ID, "---------------------------------\r\n\n");
#endif
  return 1u;
}

//--------------------------------------------------------------------------------
// READ-ONLY REGISTERS

// Read device identifier (0x00)
// See pg. 61/213 of DW1000 User Manual 2.01
uint32_t DW1000_ReadDevID(void) {
  uint32_t dev_id_val;
  DW1000_Read(DEV_ID_ID, 0x00, (uint8_t *)&dev_id_val, DEV_ID_LEN);
  DW1000_dev_id.rev = (DEV_ID_REV_MASK & dev_id_val) >> 0u;
  DW1000_dev_id.ver = (DEV_ID_VER_MASK & dev_id_val) >> 4u;
  DW1000_dev_id.model = (DEV_ID_MODEL_MASK & dev_id_val) >> 8u;
  DW1000_dev_id.ridtag = (DEV_ID_RIDTAG_MASK & dev_id_val) >> 16u;
#ifdef _VERBOSE
  char str[100];
  sprintf(str, "\r\nDevice ID: 0x%08X\r\n", (unsigned int)dev_id_val);
  UART_Print(VERBOSE_UART_ID, str);
  Delay(5);
  sprintf(str,
          "\r\nRegister ID Tag: 0x%04X\r\nModel: 0x%02x\r\nVersion: "
          "%u\r\nRev.: %u\r\n",
          (unsigned int)DW1000_dev_id.ridtag, (unsigned int)DW1000_dev_id.model,
          (unsigned int)DW1000_dev_id.ver, (unsigned int)DW1000_dev_id.rev);
  UART_Print(VERBOSE_UART_ID, str);
  Delay(5);
#endif

  return dev_id_val;
}

//--------------------------------------------------------------------------------
// CONFIGURATION REGISTERS

// Configure extended unique identifier (EUI) (0x01)
// See pg. 62/213 of DW1000 User Manual 2.01
// IEEE 802.15.4 compliance requires every device to have a unique 64-bit
// identifier:
// - High-order 24-bits are a company identifier, registered with
// http://standards.ieee.org/develop/regauth/oui/
// - Low-order 40-bits of the EUI are extension identifier uniquely chosen by
// the manufacturer for each device manufacturered and NEVER repeated
// - Note: EUI is also used for Receive Frame Filtering
void DW1000_ConfigEUI(uint32_t oui, uint64_t eid) {
  uint8_t eui[8];
  eui[0] = (0x00000000FF & eid) >> 0u;
  eui[1] = (0x000000FF00 & eid) >> 8u;
  eui[2] = (0x0000FF0000 & eid) >> 16u;
  eui[3] = (0x00FF000000 & eid) >> 24u;
  eui[4] = (0xFF00000000 & eid) >> 32u;
  eui[5] = (0x0000FF & oui) >> 0u;
  eui[6] = (0x00FF00 & oui) >> 8u;
  eui[7] = (0xFF0000 & oui) >> 16u;

  DW1000_Write(EUI_64_ID, 0X00, eui, EUI_64_LEN);
#ifdef _VERBOSE
  uint8_t eui_val[8];
  DW1000_Read(EUI_64_ID, 0X00, eui_val, EUI_64_LEN);
  char str[100];
  sprintf(str,
          "EUI: 0x%02x%02x%02x%02x%02x%02x%02x%02x "
          "(0x%02x%02x%02x%02x%02x%02x%02x%02x)\r\n",
          eui[7], eui[6], eui[5], eui[4], eui[3], eui[2], eui[1], eui[0],
          eui_val[7], eui_val[6], eui_val[5], eui_val[4], eui_val[3],
          eui_val[2], eui_val[1], eui_val[0]);
  UART_Print(VERBOSE_UART_ID, str);
  Delay(5);
#endif
}

// Configure PAN identifier and short address (0x03)
// See pg. 63/213 of DW1000 User Manual 2.01
// In a IEEE 802.15.4 personal area network (PAN), the PAN coordinator node
// determines the PAN ID for the network and assigns it a short 16-bit address
// to all nodes associated with the PAN. This identifier will be used to filter
// out other PANs.
void DW1000_ConfigPAN(uint16_t pan_id, uint16_t short_address) {
  uint32_t val32 = (pan_id << 16u) | short_address;

  DW1000_Write(PANADR_ID, 0X00, &val32, PANADR_LEN);
#ifdef _VERBOSE
  uint32_t panadr_val;
  DW1000_Read(PANADR_ID, 0X00, (uint8_t *)&panadr_val, PANADR_LEN);
  char str[100];
  sprintf(str, "PANADR: 0x%08x (0x%08x)\r\n", (unsigned int)panadr_val,
          (unsigned int)val32);
  UART_Print(VERBOSE_UART_ID, str);
  Delay(5);
#endif
}

// Configure System Register (0x04)
// See pg. 64/213 of DW1000 User Manual 2.01
// - FFEN = 0, disable frame filtering only after all frame filtering control
// bits have been enabled
// - FFBC, FFAB, FFAD, FFAA, FFAM, FFAR, FFA4, FFA5 TBD
// - HIRQ_POL = 1, active-high interrupts (enabled by default)
// - SPI_EDGE = 0, SPI_EDGE=0 gives highest rate operation, SPI_EDGE=1 gives
// most robust operation
// - DIS_FCE = 0, uses the IEEE 802.15.4-2011 standard of frame check error
// handling
// - DIS_DRXB = 0, disable double RX buffer
// - DIS_PHE = 0, recommended (set to 1 only for debugging)
// - DIS_RSDE = 0, recommended
// - FCS_INIT2F = 0, standard required for IEEE 802.15.4 compliance
// - PHR_MODE = 00, standard required for IEEE 802.15.4 compliance
// - DIS_STXP = 0, smart TX power control applies at 6.8 Mbps data rate and
// increases power for frame transmission rate of < 1 msec
// - RXM110K = 0, configuration for 850 kbps and 6.8 Mbps
// - RXWTOE = 1, disable receiver if no valid frame is received within a
// specified timeout
// - RXAUTR = 1, enable receiver auto-re-enable
// - AUTOACK = 0, disable automatic acknledgement
// - AACKPEND = 0, disable automatic acknowledgement pending bit control
void DW1000_ConfigSys(void) {
  uint32_t val32 = 0u;
  if (DW1000_config.data_rate == DATA_RATE_110)
    val32 |= SYS_CFG_RXM110K;

  // val32 |= SYS_CFG_RESET | SYS_CFG_RXAUTR | SYS_CFG_RXWTOE;
  val32 |= SYS_CFG_RESET | SYS_CFG_RXAUTR | SYS_CFG_RXWTOE |
           SYS_CFG_DIS_STXP; // disable smart power control

  DW1000_Write(SYS_CFG_ID, 0X00, &val32, SYS_CFG_LEN);
#ifdef _VERBOSE
  uint32_t sys_cfg_val;
  DW1000_Read(SYS_CFG_ID, 0X00, (uint8_t *)&sys_cfg_val, SYS_CFG_LEN);
  char str[100];
  sprintf(str, "SYS-CFG: 0x%08X (0x%08X)\r\n", (unsigned int)sys_cfg_val,
          (unsigned int)val32);
  UART_Print(VERBOSE_UART_ID, str);
  Delay(5);
#endif
}

// Enable receiver frame timeout
// See pg. 64/213 of DW1000 User Manual 2.01
void DW1000_RxTimeout(FunctionalState NewState) {
  uint32_t val32 = 0u;
  if (DW1000_config.data_rate == DATA_RATE_110)
    val32 |= SYS_CFG_RXM110K;

  switch (NewState) {
  case ENABLE:
    val32 |= SYS_CFG_RESET | SYS_CFG_RXAUTR | SYS_CFG_RXWTOE;
    break;
  case DISABLE:
    val32 |= SYS_CFG_RESET | SYS_CFG_RXAUTR;
    break;
  }

  DW1000_Write(SYS_CFG_ID, 0X00, &val32, SYS_CFG_LEN);
#if 0
  uint32_t sys_cfg_val;
  DW1000_Read(SYS_CFG_ID, 0X00, (uint8_t *)&sys_cfg_val, SYS_CFG_LEN);
  char str[100]; sprintf(str, "SYS-CFG: 0x%08X (0x%08X)\r\n", (unsigned int)sys_cfg_val, (unsigned int)val32); UART_Print(VERBOSE_UART_ID,str); Delay(5);
#endif
}

// Configure Transmit Frame Control Register (0x08)
// See pg. 69/213 of DW1000 User Manual 2.01
// - TFLEN = <?>, transmit frame length (default is 12) [bytes]
// - TFLE = 0, standard required for IEEE 802.15.4 compliance
// - TXBR = <?>, set data rate
// - TR = x, has no operational effect on DW1000
// - TXPRF = <?>, pulse repetition frequency and will depend on the preamble
// length
// - TXPSR = <?>, depends on the preamble length
// - PE = <?>, depends on the preamble length
// - TXBOFFS = 0, not needed
// - IFSDELAY = 0, inter-frame spacing for setting a delay between successive
// transmitted frames
void DW1000_ConfigTxFCTRL(uint8_t size) {
  // Check that frame length is less than 127 bytes long
  // If not, set to maximum value
  if (size > 0x7F) {
    size = 0x7F;
  }

  uint32_t val32 = (uint32_t)size;
  switch (DW1000_config.data_rate) {
  case DATA_RATE_110:
    val32 |= TX_FCTRL_TXBR_110k;
    break;
  case DATA_RATE_850:
    val32 |= TX_FCTRL_TXBR_850k;
    break;
  case DATA_RATE_6800:
    val32 |= TX_FCTRL_TXBR_6M;
    break;
  }

  switch (DW1000_config.prf) {
  case PRF_4MHZ:
    val32 |= TX_FCTRL_TXPRF_4M;
    break;
  case PRF_16MHZ:
    val32 |= TX_FCTRL_TXPRF_16M;
    break;
  case PRF_64MHZ:
    val32 |= TX_FCTRL_TXPRF_64M;
    break;
  }

  switch (DW1000_config.preamble_length) {
  case PREAMBLE_LENGTH_64:
    val32 |= TX_FCTRL_TXPSR_PE_64;
    break;
  case PREAMBLE_LENGTH_128:
    val32 |= TX_FCTRL_TXPSR_PE_128;
    break;
  case PREAMBLE_LENGTH_256:
    val32 |= TX_FCTRL_TXPSR_PE_256;
    break;
  case PREAMBLE_LENGTH_512:
    val32 |= TX_FCTRL_TXPSR_PE_512;
    break;
  case PREAMBLE_LENGTH_1024:
    val32 |= TX_FCTRL_TXPSR_PE_1024;
    break;
  case PREAMBLE_LENGTH_2048:
    val32 |= TX_FCTRL_TXPSR_PE_2048;
    break;
  case PREAMBLE_LENGTH_4096:
    val32 |= TX_FCTRL_TXPSR_PE_4096;
    break;
  }

  // @TODO Ignore IFSDELAY (inter-frame spacing)

  DW1000_Write(TX_FCTRL_ID, 0x00, &val32, 4u);
#ifdef _VERBOSE
  uint32_t tx_fctrl_val;
  DW1000_Read(TX_FCTRL_ID, 0x00, (uint8_t *)&tx_fctrl_val, 4u);
  char str[100];
  sprintf(str, "TX-FCTRL: 0x%08X (0x%08X)\r\n", (unsigned int)tx_fctrl_val,
          (unsigned int)val32);
  UART_Print(VERBOSE_UART_ID, str);
  Delay(5);
#endif
}

// Configure Receiver Frame Timeout Period (0x0C)
// See pg. 73/213 of DW1000 User Manual 2.01
// @TODO Is this needed if DRX_PRETOC is enabled?
// Enable receiver frame timeout; when timer reaches its timeout state, the
// receiver is disabled and the device goes into standby/idle mode Note: timeout
// RXWTOE must be enabled in the SYS_CFG register
// @param timeout [msec]
void DW1000_ConfigRxTimeout(uint8_t timeout) {
  // timeout cannot be greater than 65 msec
  if (timeout > 65u)
    timeout = 65u;

  // Configure Receiver Timeout (0x0C)
  uint16_t val16 = timeout * 1000u;
  DW1000_Write(RX_FWTO_ID, 0x00, &val16, RX_FWTO_LEN);
#ifdef _VERBOSE
  uint16_t rx_fwto_val;
  DW1000_Read(RX_FWTO_ID, 0x00, (uint8_t *)&rx_fwto_val, RX_FWTO_LEN);
  char str[100];
  sprintf(str, "RX-FWTO: 0x%04X (0x%04X)\r\n", (unsigned int)rx_fwto_val,
          (unsigned int)val16);
  UART_Print(VERBOSE_UART_ID, str);
  Delay(5);
#endif
}

// Configure Sniff Mode (0x1D)
// See Section 4.5 (pg. 37/213) and pg. 98/213 of DW1000 User Manual 2.01
void DW1000_ConfigSniffMode(float on_off_ratio) {
  // On duration is programmed in units of PAC (min. value 2)
  // Note: one PAC symbol is approximatley 1 usec
  // Regular Sniff Mode: Off duration is programmed in units of 1 usec (uses 125
  // MHz digital PLL clock) Low Duty-Cycle Sniff Mode: Off duration is
  // programmed in units of 6.6 usec (uses 19.2 MHz XTI clock)
  uint8_t sniff_ont = 2; // [PAC units]
  if (sniff_ont > 0x0F)
    sniff_ont = 0x0F;
  uint8_t sniff_offt =
      (uint8_t)((sniff_ont / on_off_ratio) * DW1000_config.pac_size); // [usec]

  // Configure Sniff Mode
  uint16_t val16 = (sniff_offt << 8u) | sniff_ont;
  DW1000_Write(RX_SNIFF_ID, 0x00, &val16, RX_SNIFF_LEN);
#ifdef _VERBOSE
  uint16_t rx_sniff_val;
  DW1000_Read(RX_SNIFF_ID, 0x00, (uint8_t *)&rx_sniff_val, RX_SNIFF_LEN);
  char str[100];
  sprintf(str, "RX-SNIFF: 0x%04X (0x%04X)\r\n", (unsigned int)rx_sniff_val,
          (unsigned int)val16);
  UART_Print(VERBOSE_UART_ID, str);
  Delay(5);
#endif
}

// Configure Smart Transmit Power Control (0x1E)
// See pg. 100/213 of DW1000 User Manual 2.01
void DW1000_ConfigSmartTxPowerControl(void) {
  // See Table 17, pg. 103/213 of DW1000 User Manual 2.01
  uint32_t val32;
  switch (DW1000_config.prf) {
  case PRF_4MHZ:
    break;

  case PRF_16MHZ: {
    switch (DW1000_config.channel) {
    case CH1:
      val32 = 0x15355575;
      break;
    case CH2:
      val32 = 0x15355575;
      break;
    case CH3:
      val32 = 0x0F2F4F6F;
      break;
    case CH4:
      val32 = 0x1F1F3F5F;
      break;
    case CH5:
      val32 = 0x0E082848;
      break;
    case CH7:
      val32 = 0x32527292;
      break;
    }
  } break;

  case PRF_64MHZ: {
    switch (DW1000_config.channel) {
    case CH1:
      val32 = 0x07274767;
      break;
    case CH2:
      val32 = 0x07274767;
      break;
    case CH3:
      val32 = 0x2B4B6B8B;
      break;
    case CH4:
      val32 = 0x3A5A7A9A;
      break;
    case CH5:
      val32 = 0x25456585;
      break;
    case CH7:
      val32 = 0x5171B1D1;
      break;
    }
  } break;
  }

  val32 = 0x1F1F1F1F; // fuck it, maximum power!!!
  DW1000_Write(TX_POWER_ID, 0x00, &val32, TX_POWER_LEN);
#ifdef _VERBOSE
  uint32_t tx_power_val;
  DW1000_Read(TX_POWER_ID, 0x00, (uint8_t *)&tx_power_val, TX_POWER_LEN);
  char str[100];
  sprintf(str, "TX-POWER: 0x%08X (0x%08X)\r\n", (unsigned int)tx_power_val,
          (unsigned int)val32);
  UART_Print(VERBOSE_UART_ID, str);
  Delay(5);
#endif
}

// Configure Manual Transmit Power Control (0x1E)
// See pg. 101/213 of DW1000 User Manual 2.01
void DW1000_ConfigManualTxPowerControl(void) {
  // See Table 18, pg. 103/213 of DW1000 User Manual 2.01
  uint32_t val32;
  switch (DW1000_config.prf) {
  case PRF_4MHZ:
    break;

  case PRF_16MHZ: {
    switch (DW1000_config.channel) {
    case CH1:
      val32 = 0x75757575;
      break;
    case CH2:
      val32 = 0x75757575;
      break;
    case CH3:
      val32 = 0x6F6F6F6F;
      break;
    case CH4:
      val32 = 0x5F5F5F5F;
      break;
    case CH5:
      val32 = 0x48484848;
      break;
    case CH7:
      val32 = 0x92929292;
      break;
    }
  } break;

  case PRF_64MHZ: {
    switch (DW1000_config.channel) {
    case CH1:
      val32 = 0x67676767;
      break;
    case CH2:
      val32 = 0x67676767;
      break;
    case CH3:
      val32 = 0x8B8B8B8B;
      break;
    case CH4:
      val32 = 0x9A9A9A9A;
      break;
    case CH5:
      val32 = 0x85858585;
      break;
    case CH7:
      val32 = 0xD1D1D1D1;
      break;
    }
  } break;
  }

  val32 = 0x1F1F1F1F; // maximum power!!!
  DW1000_Write(TX_POWER_ID, 0x00, &val32, TX_POWER_LEN);
#ifdef _VERBOSE
  uint32_t tx_power_val;
  DW1000_Read(TX_POWER_ID, 0x00, (uint8_t *)&tx_power_val, TX_POWER_LEN);
  char str[100];
  sprintf(str, "TX-POWER: 0x%08X (0x%08X)\r\n", (unsigned int)tx_power_val,
          (unsigned int)val32);
  UART_Print(VERBOSE_UART_ID, str);
  Delay(5);
#endif
}

// Configure Transmit Power Control (0x1E)
// See pg. 100/213 of DW1000 User Manual 2.01
void DW1000_ConfigTxPowerControl(void) {
  // Check if smart transmit power control is enabled
  // Read system register (0x04)
  // See pg. 64/213 of DW1000 User Manual 2.01
  uint32_t sys_cfg_val;
  DW1000_Read(SYS_CFG_ID, 0X00, (uint8_t *)&sys_cfg_val, SYS_CFG_LEN);

  // Configure transmit power control
  if ((SYS_CFG_DIS_STXP & sys_cfg_val) == 0u) {
    DW1000_ConfigSmartTxPowerControl();
  } else {
    DW1000_ConfigManualTxPowerControl();
  }
}

// Configure Channel Control Register (0x1F)
// See pg. 103/213 of DW1000 User Manual 2.01
void DW1000_ConfigChannelControl(void) {
  uint32_t val32 = 0u;
  val32 |= DW1000_config.channel
           << CHAN_CTRL_TX_CHAN_SHIFT; // transmitter channel
  val32 |= DW1000_config.channel << CHAN_CTRL_RX_CHAN_SHIFT; // receiver channel
  val32 |= DW1000_config.prf << CHAN_CTRL_RXFPRF_SHIFT;      // receiver PRF
  val32 |= DW1000_config.preamble_code
           << CHAN_CTRL_TX_PCOD_SHIFT; // transmitter preamble code
  val32 |= DW1000_config.preamble_code
           << CHAN_CTRL_RX_PCOD_SHIFT; // receiver preamble code
  // val32 |= CHAN_CTRL_DWSFD; // enable non-standard DecaWave proprietary SFD
  // sequence

  DW1000_Write(CHAN_CTRL_ID, 0x00, &val32, CHAN_CTRL_LEN);
#ifdef _VERBOSE
  uint32_t chan_ctrl_val;
  DW1000_Read(CHAN_CTRL_ID, 0x00, (uint8_t *)&chan_ctrl_val, CHAN_CTRL_LEN);
  char str[100];
  sprintf(str, "CHAN-CTRL: 0x%08X (0x%08X)\r\n", (unsigned int)chan_ctrl_val,
          (unsigned int)val32);
  UART_Print(VERBOSE_UART_ID, str);
  Delay(5);
#endif
}

// Configure SFD Sequence Register (0x21)
// See pg. 105/213 of DW1000 User Manual 2.01
void DW1000_ConfigSFDSequence(void) {
  // See Table 19, pg. 108/213 of DW1000 User Manual 2.01
  uint8_t val8 = 0u;
  switch (DW1000_config.data_rate) {
  case DATA_RATE_110:
    val8 |= 0u;
    break;
  case DATA_RATE_850:
    val8 |= 16u;
    break;
  case DATA_RATE_6800:
    val8 |= 8u;
    break;
  }

  DW1000_Write(USR_SFD_ID, 0x00, &val8, USR_SFD_LEN);
#ifdef _VERBOSE
  uint8_t usr_sfd_val;
  DW1000_Read(USR_SFD_ID, 0x00, &usr_sfd_val, USR_SFD_LEN);
  char str[100];
  sprintf(str, "USR-SFD: 0x%02X (0x%02X)\r\n", (unsigned int)usr_sfd_val,
          (unsigned int)val8);
  UART_Print(VERBOSE_UART_ID, str);
  Delay(5);
#endif
}

// Configure Automatic Gain Control Registers (0x23)
// See pg. 109/213 of DW1000 User Manual 2.01
void DW1000_ConfigAGC(void) {
#ifdef _VERBOSE
  char str[100];
#endif

  // AGC_TUNE1 (0x23:06)
  // See pg. 110/213 of DW1000 User Manual 2.01
  uint16_t val16;
  switch (DW1000_config.prf) {
  case PRF_4MHZ:
    break;
  case PRF_16MHZ:
    val16 = AGC_TUNE1_16M;
    break;
  case PRF_64MHZ:
    val16 = AGC_TUNE1_64M;
    break;
  }

  DW1000_Write(AGC_CTRL_ID, AGC_TUNE1_OFFSET, &val16, AGC_TUNE1_LEN);
#ifdef _VERBOSE
  uint16_t agc_tune1_val;
  DW1000_Read(AGC_CTRL_ID, AGC_TUNE1_OFFSET, (uint8_t *)&agc_tune1_val,
              AGC_TUNE1_LEN);
  sprintf(str, "AGC-TUNE1: 0x%04X (0x%04X)\r\n", (unsigned int)agc_tune1_val,
          (unsigned int)val16);
  UART_Print(VERBOSE_UART_ID, str);
  Delay(5);
#endif

  // AGC_TUNE2
  // See pg. 111/213 of DW1000 User Manual 2.01
  uint32_t val32 = AGC_TUNE2_VAL;
  DW1000_Write(AGC_CTRL_ID, AGC_TUNE2_OFFSET, &val32, AGC_TUNE2_LEN);
#ifdef _VERBOSE
  uint32_t agc_tune2_val;
  DW1000_Read(AGC_CTRL_ID, AGC_TUNE2_OFFSET, (uint8_t *)&agc_tune2_val,
              AGC_TUNE2_LEN);
  sprintf(str, "AGC-TUNE2: 0x%08X (0x%08X)\r\n", (unsigned int)agc_tune2_val,
          (unsigned int)val32);
  UART_Print(VERBOSE_UART_ID, str);
  Delay(5);
#endif

  // AGC_TUNE3
  // See pg. 112/213 of DW1000 User Manual 2.01
  val16 = AGC_TUNE3_VAL;
  DW1000_Write(AGC_CTRL_ID, AGC_TUNE3_OFFSET, &val16, AGC_TUNE3_LEN);
#ifdef _VERBOSE
  uint16_t agc_tune3_val;
  DW1000_Read(AGC_CTRL_ID, AGC_TUNE3_OFFSET, (uint8_t *)&agc_tune3_val,
              AGC_TUNE3_LEN);
  sprintf(str, "AGC-TUNE3: 0x%04X (0x%04X)\r\n", (unsigned int)agc_tune3_val,
          (unsigned int)val16);
  UART_Print(VERBOSE_UART_ID, str);
  Delay(5);
#endif
}

// Configure Digital Receiver (0x27)
// See pg. 130/213 of DW1000 User Manual 2.01
void DW1000_ConfigDRX(void) {
#ifdef _VERBOSE
  char str[100];
#endif

  // Configure DRX_TUNE0b (0x27:02)
  // See pg. 130/213 of DW1000 User Manual 2.01
  uint16_t val16;
#if 1 // standard SFD
  switch (DW1000_config.data_rate) {
  case DATA_RATE_110:
    val16 = 0x000A;
    break;
  case DATA_RATE_850:
    val16 = 0x0001;
    break;
  case DATA_RATE_6800:
    val16 = 0x0001;
    break;
  }
#else
  switch (DW1000_config.data_rate) {
  case DATA_RATE_110:
    val16 = 0x0016;
    break;
  case DATA_RATE_850:
    val16 = 0x0006;
    break;
  case DATA_RATE_6800:
    val16 = 0x0002;
    break;
  }
#endif

  DW1000_Write(DRX_CONF_ID, DRX_TUNE0b_OFFSET, &val16, DRX_TUNE0b_LEN);
#ifdef _VERBOSE
  uint16_t drx_tune0b_val;
  DW1000_Read(DRX_CONF_ID, DRX_TUNE0b_OFFSET, (uint8_t *)&drx_tune0b_val,
              DRX_TUNE0b_LEN);
  sprintf(str, "DRX-TUNE0b: 0x%04X (0x%04X)\r\n", (unsigned int)drx_tune0b_val,
          (unsigned int)val16);
  UART_Print(VERBOSE_UART_ID, str);
  Delay(5);
#endif

  // Configure DRX_TUNE1a (0x27:04)
  // See pg. 131/213 of DW1000 User Manual 2.01
  switch (DW1000_config.prf) {
  case PRF_4MHZ:
    break;
  case PRF_16MHZ:
    val16 = 0x0087;
    break;
  case PRF_64MHZ:
    val16 = 0x008D;
    break;
  }

  DW1000_Write(DRX_CONF_ID, DRX_TUNE1a_OFFSET, &val16, DRX_TUNE1a_LEN);
#ifdef _VERBOSE
  uint16_t drx_tune1a_val;
  DW1000_Read(DRX_CONF_ID, DRX_TUNE1a_OFFSET, (uint8_t *)&drx_tune1a_val,
              DRX_TUNE1a_LEN);
  sprintf(str, "DRX-TUNE1a: 0x%04X (0x%04X)\r\n", (unsigned int)drx_tune1a_val,
          (unsigned int)val16);
  UART_Print(VERBOSE_UART_ID, str);
  Delay(5);
#endif

  // Configure DRX_TUNE1b (0x27:06)
  // See pg. 132/213 of DW1000 User Manual 2.01
  switch (DW1000_config.preamble_length) {
  case PREAMBLE_LENGTH_64:
    val16 = 0x0010;
    break;
  case PREAMBLE_LENGTH_128:
    val16 = 0x0020;
    break;
  case PREAMBLE_LENGTH_256:
    val16 = 0x0020;
    break;
  case PREAMBLE_LENGTH_512:
    val16 = 0x0020;
    break;
  case PREAMBLE_LENGTH_1024:
    val16 = 0x0020;
    break;
  case PREAMBLE_LENGTH_2048:
    val16 = 0x0064;
    break;
  case PREAMBLE_LENGTH_4096:
    val16 = 0x0064;
    break;
  }

  DW1000_Write(DRX_CONF_ID, DRX_TUNE1b_OFFSET, &val16, DRX_TUNE1b_LEN);
#ifdef _VERBOSE
  uint16_t drx_tune1b_val;
  DW1000_Read(DRX_CONF_ID, DRX_TUNE1b_OFFSET, (uint8_t *)&drx_tune1b_val,
              DRX_TUNE1b_LEN);
  sprintf(str, "DRX-TUNE1b: 0x%04X (0x%04X)\r\n", (unsigned int)drx_tune1b_val,
          (unsigned int)val16);
  UART_Print(VERBOSE_UART_ID, str);
  Delay(5);
#endif

  // Configure DRX_TUNE2 (0x27:08)
  // See pg. 132/213 of DW1000 User Manual 2.01
  uint32_t val32;
  switch (DW1000_config.prf) {
  case PRF_4MHZ:
    break;

  case PRF_16MHZ: {
    switch (DW1000_config.pac_size) {
    case PAC_SIZE_8:
      val32 = DRX_TUNE2_PAC08_PRF16;
      break;
    case PAC_SIZE_16:
      val32 = DRX_TUNE2_PAC16_PRF16;
      break;
    case PAC_SIZE_32:
      val32 = DRX_TUNE2_PAC32_PRF16;
      break;
    case PAC_SIZE_64:
      val32 = DRX_TUNE2_PAC64_PRF16;
      break;
    }
    break;
  } break;

  case PRF_64MHZ: {
    switch (DW1000_config.pac_size) {
    case PAC_SIZE_8:
      val32 = DRX_TUNE2_PAC08_PRF64;
      break;
    case PAC_SIZE_16:
      val32 = DRX_TUNE2_PAC16_PRF64;
      break;
    case PAC_SIZE_32:
      val32 = DRX_TUNE2_PAC32_PRF64;
      break;
    case PAC_SIZE_64:
      val32 = DRX_TUNE2_PAC64_PRF64;
      break;
    }
    break;
  } break;
  }

  DW1000_Write(DRX_CONF_ID, DRX_TUNE2_OFFSET, &val32, DRX_TUNE2_LEN);
#ifdef _VERBOSE
  uint32_t drx_tune2_val;
  DW1000_Read(DRX_CONF_ID, DRX_TUNE2_OFFSET, (uint8_t *)&drx_tune2_val,
              DRX_TUNE2_LEN);
  sprintf(str, "DRX-TUNE2: 0x%08X (0x%08X)\r\n", (unsigned int)drx_tune2_val,
          (unsigned int)val32);
  UART_Print(VERBOSE_UART_ID, str);
  Delay(5);
#endif

  // Configure DRX_SFDTOC (0x27:20)
  // See pg. 133/213 of DW1000 User Manual 2.01
  // See Table 19, pg. 108/213 of DW1000 User Manual 2.01
  val16 = DW1000_config.preamble_length + 1u;
  switch (DW1000_config.data_rate) {
  case DATA_RATE_110:
    val16 += 64u;
    break;
  case DATA_RATE_850:
    val16 += 16u;
    break;
  case DATA_RATE_6800:
    val16 += 8u;
    break;
  }

  DW1000_Write(DRX_CONF_ID, DRX_SFDTOC_OFFSET, &val16, DRX_SFDTOC_LEN);
#ifdef _VERBOSE
  uint16_t drx_sfdtoc_val;
  DW1000_Read(DRX_CONF_ID, DRX_SFDTOC_OFFSET, (uint8_t *)&drx_sfdtoc_val,
              DRX_SFDTOC_LEN);
  sprintf(str, "DRX-SFDTOC: 0x%04X (0x%04X)\r\n", (unsigned int)drx_sfdtoc_val,
          (unsigned int)val16);
  UART_Print(VERBOSE_UART_ID, str);
  Delay(5);
#endif

  // Configure DRX_PRETOC (0x27:24)
  // See pg. 134/213 of DW1000 User Manual 2.01
  // @TODO Find an appropriate value for PRETOC
  val16 = 16u; // timeout [msec] = PAC-size * DRX_PRETOC_VAL
  DW1000_Write(DRX_CONF_ID, DRX_PRETOC_OFFSET, &val16, DRX_PRETOC_LEN);
#ifdef _VERBOSE
  uint16_t drx_pretoc_val;
  DW1000_Read(DRX_CONF_ID, DRX_PRETOC_OFFSET, (uint8_t *)&drx_pretoc_val,
              DRX_PRETOC_LEN);
  sprintf(str, "DRX-PRETOC: 0x%04X (0x%04X)\r\n", (unsigned int)drx_pretoc_val,
          (unsigned int)val16);
  UART_Print(VERBOSE_UART_ID, str);
  Delay(5);
#endif

  // Configure DRX_TUNE4H (0x27:26)
  // See pg. 135/213 of DW1000 User Manual 2.01
  switch (DW1000_config.preamble_length) {
  case PREAMBLE_LENGTH_64:
    val16 = 0x0010;
    break;
  case PREAMBLE_LENGTH_128:
    val16 = 0x0028;
    break;
  case PREAMBLE_LENGTH_256:
    val16 = 0x0028;
    break;
  case PREAMBLE_LENGTH_512:
    val16 = 0x0028;
    break;
  case PREAMBLE_LENGTH_1024:
    val16 = 0x0028;
    break;
  case PREAMBLE_LENGTH_2048:
    val16 = 0x0028;
    break;
  case PREAMBLE_LENGTH_4096:
    val16 = 0x0028;
    break;
  }

  DW1000_Write(DRX_CONF_ID, DRX_DRX_TUNE4H_OFFSET, &val16, DRX_PRETOC_LEN);
#ifdef _VERBOSE
  uint16_t drx_tune4h_val;
  DW1000_Read(DRX_CONF_ID, DRX_DRX_TUNE4H_OFFSET, (uint8_t *)&drx_tune4h_val,
              DRX_PRETOC_LEN);
  sprintf(str, "DRX-TUNE4H: 0x%04X (0x%04X)\r\n", (unsigned int)drx_tune4h_val,
          (unsigned int)val16);
  UART_Print(VERBOSE_UART_ID, str);
  Delay(5);
#endif
}

// Configure Analog RF Block (0x28)
// See pg. 135/213 of DW1000 User Manual 2.01
void DW1000_ConfigAnalogRFBlock(void) {
#ifdef _VERBOSE
  char str[100];
#endif

  // Configure RF_RXCTRLH (0x28:0B)
  // See pg. 137/213 of DW1000 User Manual 2.01
  uint8_t val8;
  switch (DW1000_config.channel) {
  case CH1:
    val8 = 0xD8;
    break;
  case CH2:
    val8 = 0xD8;
    break;
  case CH3:
    val8 = 0xD8;
    break;
  case CH4:
    val8 = 0xBC;
    break;
  case CH5:
    val8 = 0xD8;
    break;
  case CH7:
    val8 = 0xBC;
    break;
  }

  DW1000_Write(RF_CONF_ID, RF_RXCTRLH_OFFSET, &val8, RF_TXCTRL_OFFSET);
#ifdef _VERBOSE
  uint8_t rf_rxctrlh_val;
  DW1000_Read(RF_CONF_ID, RF_RXCTRLH_OFFSET, &rf_rxctrlh_val, RF_TXCTRL_LEN);
  sprintf(str, "RF-RXCTRLH: 0x%02X (0x%02X)\r\n", (unsigned int)rf_rxctrlh_val,
          (unsigned int)val8);
  UART_Print(VERBOSE_UART_ID, str);
  Delay(5);
#endif

  // Configure RF_TXCTRL (0x28:0x0C)
  // See pg. 137/213 of DW1000 User Manual 2.01
  uint32_t val32;
  switch (DW1000_config.channel) {
  case CH1:
    val32 = RF_TXCTRL_CH1;
    break;
  case CH2:
    val32 = RF_TXCTRL_CH2;
    break;
  case CH3:
    val32 = RF_TXCTRL_CH3;
    break;
  case CH4:
    val32 = RF_TXCTRL_CH4;
    break;
  case CH5:
    val32 = RF_TXCTRL_CH5;
    break;
  case CH7:
    val32 = RF_TXCTRL_CH7;
    break;
  }

  DW1000_Write(RF_CONF_ID, RF_TXCTRL_OFFSET, &val32, RF_TXCTRL_LEN);
#ifdef _VERBOSE
  uint32_t rf_txctrl_val;
  DW1000_Read(RF_CONF_ID, RF_TXCTRL_OFFSET, (uint8_t *)&rf_txctrl_val,
              RF_TXCTRL_LEN);
  sprintf(str, "RF-TXCTRL: 0x%08X (0x%08X)\r\n", (unsigned int)rf_txctrl_val,
          (unsigned int)val32);
  UART_Print(VERBOSE_UART_ID, str);
  Delay(5);
#endif
}

// Configure Transmitter Calibration Block (0x2A)
// See pg. 140/213 of DW1000 User Manual 2.01
void DW1000_ConfigTxCalibBlock(void) {
  // Configure TC_PGDELAY (0x2A:0B)
  // See pg. 142/213 of DW1000 User Manual 2.01
  uint8_t val8;
  switch (DW1000_config.channel) {
  case CH1:
    val8 = TC_PGDELAY_CH1;
    break;
  case CH2:
    val8 = TC_PGDELAY_CH2;
    break;
  case CH3:
    val8 = TC_PGDELAY_CH3;
    break;
  case CH4:
    val8 = TC_PGDELAY_CH4;
    break;
  case CH5:
    val8 = TC_PGDELAY_CH5;
    break;
  case CH7:
    val8 = TC_PGDELAY_CH7;
    break;
  }

  DW1000_Write(TX_CAL_ID, TC_PGDELAY_OFFSET, &val8, TC_PGDELAY_LEN);
#ifdef _VERBOSE
  uint8_t tc_pgdelay_val;
  DW1000_Read(TX_CAL_ID, TC_PGDELAY_OFFSET, &tc_pgdelay_val, TC_PGDELAY_LEN);
  char str[100];
  sprintf(str, "TC-PGDELAY: 0x%02X (0x%02X)\r\n", (unsigned int)tc_pgdelay_val,
          (unsigned int)val8);
  UART_Print(VERBOSE_UART_ID, str);
  Delay(5);
#endif
}

// Configure Frequency Synthesiser Control Block (0x2B)
// See pg. 143/213 of DW1000 User Manual 2.01
void DW1000_ConfigFreqSynthControlBlock(void) {
#ifdef _VERBOSE
  char str[100];
#endif

  // Configure FS_PLLCFG (0x2B:07)
  // See pg. 144/213 of DW1000 User Manual 2.01
  uint64_t val32;
  switch (DW1000_config.channel) {
  case CH1:
    val32 = FS_PLLCFG_CH1;
    break;
  case CH2:
    val32 = FS_PLLCFG_CH2;
    break;
  case CH3:
    val32 = FS_PLLCFG_CH3;
    break;
  case CH4:
    val32 = FS_PLLCFG_CH4;
    break;
  case CH5:
    val32 = FS_PLLCFG_CH5;
    break;
  case CH7:
    val32 = FS_PLLCFG_CH7;
    break;
  }

  DW1000_Write(FS_CTRL_ID, FS_PLLCFG_OFFSET, &val32, FS_PLLCFG_LEN);
#ifdef _VERBOSE
  uint32_t fs_pllcfg_val;
  DW1000_Read(FS_CTRL_ID, FS_PLLCFG_OFFSET, (uint8_t *)&fs_pllcfg_val,
              FS_PLLCFG_LEN);
  sprintf(str, "FS-PLLCFG: 0x%08X (0x%08X)\r\n", (unsigned int)fs_pllcfg_val,
          (unsigned int)val32);
  UART_Print(VERBOSE_UART_ID, str);
  Delay(5);
#endif

  // Configure FS_PLLTUNE (0x2B:0B)
  // See pg. 145/213 of DW1000 User Manual 2.01
  uint8_t val8;
  switch (DW1000_config.channel) {
  case CH1:
    val8 = FS_PLLTUNE_CH1;
    break;
  case CH2:
    val8 = FS_PLLTUNE_CH2;
    break;
  case CH3:
    val8 = FS_PLLTUNE_CH3;
    break;
  case CH4:
    val8 = FS_PLLTUNE_CH4;
    break;
  case CH5:
    val8 = FS_PLLTUNE_CH5;
    break;
  case CH7:
    val8 = FS_PLLTUNE_CH7;
    break;
  }

  DW1000_Write(FS_CTRL_ID, FS_PLLTUNE_OFFSET, &val8, FS_PLLTUNE_LEN);
#ifdef _VERBOSE
  uint8_t fs_plltune_val;
  DW1000_Read(FS_CTRL_ID, FS_PLLTUNE_OFFSET, &fs_plltune_val, FS_PLLTUNE_LEN);
  sprintf(str, "FS-PLLTUNE: 0x%02X (0x%02X)\r\n", (unsigned int)fs_plltune_val,
          (unsigned int)val8);
  UART_Print(VERBOSE_UART_ID, str);
  Delay(5);
#endif
}

// Configure Leading Edge Detection Interface (0x2E)
// See pg. 161/213 of DW1000 User Manual 2.01
void DW1000_ConfigLDE(void) {
#ifdef _VERBOSE
  char str[100];
#endif

  // Configure LDE_CFG1 (0x2E:0806)
  // See pg. 162/213 of DW1000 User Manual 2.01
  // Note: a value of 13 (0xD) provides more accuracy in LOS conditions, but 12
  // (0xC) may be better for NLOS
  uint8_t val8 = 0x6D;
  DW1000_Write(LDE_IF_ID, LDE_CFG1_OFFSET, &val8, LDE_CFG1_LEN);
#ifdef _VERBOSE
  uint8_t lde_cfg1_val;
  DW1000_Read(LDE_IF_ID, LDE_CFG1_OFFSET, &lde_cfg1_val, LDE_CFG1_LEN);
  sprintf(str, "LDE-CFG1: 0x%02X (0x%02X)\r\n", (unsigned int)lde_cfg1_val,
          (unsigned int)val8);
  UART_Print(VERBOSE_UART_ID, str);
  Delay(5);
#endif

  // Configure LDE_CFG2 (0x2E:1806)
  // See pg. 164/213 of DW1000 User Manual 2.01
  uint16_t val16;
  switch (DW1000_config.prf) {
  case PRF_4MHZ:
    break;
  case PRF_16MHZ:
    val16 = LDE_CFG2_PRF16;
    break;
  case PRF_64MHZ:
    val16 = LDE_CFG2_PRF64;
    break;
  }

  DW1000_Write(LDE_IF_ID, LDE_CFG2_OFFSET, &val16, LDE_CFG2_LEN);
#ifdef _VERBOSE
  uint16_t lde_cfg2_val;
  DW1000_Read(LDE_IF_ID, LDE_CFG2_OFFSET, (uint8_t *)&lde_cfg2_val,
              LDE_CFG2_LEN);
  sprintf(str, "LDE-CFG2: 0x%04X (0x%04X)\r\n", (unsigned int)lde_cfg2_val,
          (unsigned int)val16);
  UART_Print(VERBOSE_UART_ID, str);
  Delay(5);
#endif

  // Configure LDE_REPC (0x2E:2804)
  // See pg. 164/213 of DW1000 User Manual 2.01
  switch (DW1000_config.preamble_code) {
  case PREAMBLE_CODE_1:
    val16 = 0x5998;
    break;
  case PREAMBLE_CODE_2:
    val16 = 0x5998;
    break;
  case PREAMBLE_CODE_3:
    val16 = 0x51EA;
    break;
  case PREAMBLE_CODE_4:
    val16 = 0x428E;
    break;
  case PREAMBLE_CODE_5:
    val16 = 0x451E;
    break;
  case PREAMBLE_CODE_6:
    val16 = 0x2E14;
    break;
  case PREAMBLE_CODE_7:
    val16 = 0x8000;
    break;
  case PREAMBLE_CODE_8:
    val16 = 0x51EA;
    break;
  case PREAMBLE_CODE_9:
    val16 = 0x28F4;
    break;
  case PREAMBLE_CODE_10:
    val16 = 0x3332;
    break;
  case PREAMBLE_CODE_11:
    val16 = 0x3AE0;
    break;
  case PREAMBLE_CODE_12:
    val16 = 0x3D70;
    break;
  case PREAMBLE_CODE_17:
    val16 = 0x3332;
    break;
  case PREAMBLE_CODE_18:
    val16 = 0x35C2;
    break;
  case PREAMBLE_CODE_19:
    val16 = 0X35C2;
    break;
  case PREAMBLE_CODE_20:
    val16 = 0x47AE;
    break;
  case PREAMBLE_CODE_21:
    val16 = 0x3AE0;
    break;
  case PREAMBLE_CODE_22:
    val16 = 0x3850;
    break;
  case PREAMBLE_CODE_23:
    val16 = 0x30A2;
    break;
  case PREAMBLE_CODE_24:
    val16 = 0x3850;
    break;
  }

  if (DW1000_config.data_rate == DATA_RATE_110)
    val16 >>= 3u; // divide by 8
  DW1000_Write(LDE_IF_ID, LDE_REPC_OFFSET, &val16, LDE_REPC_LEN);
#ifdef _VERBOSE
  uint16_t lde_repc_val;
  DW1000_Read(LDE_IF_ID, LDE_REPC_OFFSET, (uint8_t *)&lde_repc_val,
              LDE_REPC_LEN);
  sprintf(str, "LDE-REPC: 0x%04X (0x%04X)\r\n", (unsigned int)lde_repc_val,
          (unsigned int)val16);
  UART_Print(VERBOSE_UART_ID, str);
  Delay(5);
#endif
}

// Configure Power Management and System Control (0x36)
// See pg. 174/213 of DW1000 User Manual 2.01
void DW1000_ConfigPMSC(void) {
  // Configure PMSC_CTRL0 (0x36:00)
  // See pg. 175/213 of DW1000 User Manual 2.01
  // uint32_t val32 = PMSC_CTRL0_DEFAULT | PMSC_CTRL0_FACE | PMSC_CTRL0_AMCE |
  // PMSC_CTRL0_GPDCE | PMSC_CTRL0_KHZCLKEN;
  uint32_t val32 = PMSC_CTRL0_DEFAULT | PMSC_CTRL0_GPDCE | PMSC_CTRL0_KHZCLKEN;
  DW1000_Write(PMSC_ID, PMSC_CTRL0_OFFSET, &val32, PMSC_CTRL0_LEN);
#ifdef _VERBOSE
  uint32_t pmsc_ctrl0_val;
  DW1000_Read(PMSC_ID, PMSC_CTRL0_OFFSET, (uint8_t *)&pmsc_ctrl0_val,
              PMSC_CTRL0_LEN);
  char str[100];
  sprintf(str, "PMSC-CTRL0: 0x%08X (0x%08X)\r\n", (unsigned int)pmsc_ctrl0_val,
          (unsigned int)val32);
  UART_Print(VERBOSE_UART_ID, str);
  Delay(5);
#endif
}

//--------------------------------------------------------------------------------

// Configure system event mask register (0x0E)
// See pg. 77/213 and 80/213 of DW1000 User Manual 2.01
// This is used to configure interrupts (see SYS_MASK and SYS_STATUS)
// - SYS_MASK_MRXPHE // Mask receiver PHY header error event
// - SYS_MASK_MRXDFR // Mask receiver data frame ready event
// - SYS_MASK_MRXFCG // Mask receiver FCS good event (used simultaneously with
// MRXDFR)
// - SYS_MASK_MRXFCE // Mask receiver FCS error event (used simultaneously with
// MRXDFR)
// - SYS_MASK_MRXRFSL // Mask receiver Reed Solomon Frame Sync Loss event
// - SYS_MASK_MRXRFTO  Mask Receive Frame Wait Timeout event
// - SYS_MASK_MRXOVRR // Mask Receiver Overrun event
// - SYS_MASK_MRXPTO // Mask Preamble detection timeout event
// - SYS_MASK_MRXSFDTO // Mask Receive SFD timeout event
void DW1000_ConfigSysEventMask(void) {
  // uint32_t val32 = SYS_MASK_MRXPHE | SYS_MASK_MRXDFR | SYS_MASK_MRXRFSL |
  // SYS_MASK_MRXRFTO | SYS_MASK_MRXOVRR | SYS_MASK_MRXPTO | SYS_MASK_MRXSFDTO;
  // uint32_t val32 = SYS_MASK_MRXDFR | SYS_MASK_MRXRFTO; // @TODO: why are we
  // listening for frame timeout events? If it's a mobile node, receiver will
  // automatically re-enable
  //                                                      // after a
  //                                                      transmission; if it's
  //                                                      a static node,
  //                                                      receiver will always
  //                                                      be on
  uint32_t val32 = SYS_MASK_MRXFCG;

  DW1000_Write(SYS_MASK_ID, 0x00, &val32, SYS_MASK_LEN);
#ifdef _VERBOSE
  uint32_t sys_mask_val;
  DW1000_Read(SYS_MASK_ID, 0x00, (uint8_t *)&sys_mask_val, SYS_MASK_LEN);
  char str[100];
  sprintf(str, "SYS-MASK: 0x%08X (0x%08X)\r\n", (unsigned int)sys_mask_val,
          (unsigned int)val32);
  UART_Print(VERBOSE_UART_ID, str);
  Delay(5);
#endif
}

// Read status register (0x0F)
// See pg. 80/213 of DW1000 User Manual 2.01
// 0=Failure, 1=Success
uint8_t DW1000_GetSysStatus(uint64_t status_mask) {
  uint8_t data[5];
  DW1000_Read(SYS_STATUS_ID, 0x00, data, SYS_STATUS_LEN);

  uint32_t status_lsb = ((uint32_t)data[3] << 24u) |
                        ((uint32_t)data[2] << 16u) | ((uint32_t)data[1] << 8u) |
                        ((uint32_t)data[0] << 0u);
  uint64_t status = data[4];
  status <<= 32u;
  status |= status_lsb;
  status &= status_mask;

  if (status_mask == status) {
    return 1;
  }

  return 0u;
}

// Clear status register (0x0F)
// See pg. 80/213 of DW1000 User Manual 2.01
void DW1000_ClearSysStatus(void) {
  uint8_t data[5] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  DW1000_Write(SYS_STATUS_ID, 0x00, data, SYS_STATUS_LEN);
}

// Print status register (0x0F)
// See pg. 80/213 of DW1000 User Manual 2.01
void DW1000_PrintSysStatus(void) {
  uint8_t data[5];
  DW1000_Read(SYS_STATUS_ID, 0x00, data, SYS_STATUS_LEN);

  uint32_t status_lsb = ((uint32_t)data[3] << 24u) |
                        ((uint32_t)data[2] << 16u) | ((uint32_t)data[1] << 8u) |
                        ((uint32_t)data[0] << 0u);
  uint64_t status = data[4];
  status <<= 32u;
  status |= status_lsb;

#ifdef _VERBOSE
  char str[100];
  sprintf(str, "%u: (0x%02x%02x%02x%02x%02x) | ", (unsigned int)GetTime(),
          data[4], data[3], data[2], data[1], data[0]);
  UART_Print(VERBOSE_UART_ID, str);

  // ALL RX GOOD EVENTS
  if ((status & SYS_STATUS_RXDFR) > 0u) {
    UART_Print(VERBOSE_UART_ID, "01-RXDFR | ");
  } /* Receiver Data Frame Ready */
  if ((status & SYS_STATUS_RXFCG) > 0u) {
    UART_Print(VERBOSE_UART_ID, "02-RXFCG | ");
  } /* Receiver FCS Good */
  if ((status & SYS_STATUS_RXPRD) > 0u) {
    UART_Print(VERBOSE_UART_ID, "03-RXPRD | ");
  } /* Receiver Preamble Detected status */
  if ((status & SYS_STATUS_RXSFDD) > 0u) {
    UART_Print(VERBOSE_UART_ID, "04-RXSFDD | ");
  } /* Receiver Start Frame Delimiter Detected. */
  if ((status & SYS_STATUS_RXPHD) > 0u) {
    UART_Print(VERBOSE_UART_ID, "05-RXPHD | ");
  } /* Receiver PHY Header Detect */
  if ((status & SYS_STATUS_LDEDONE) > 0u) {
    UART_Print(VERBOSE_UART_ID, "06-LDEDONE | ");
  } /* LDE processing done */

  // All RX ERROR EVENTS
  if ((status & SYS_STATUS_RXPHE) > 0u) {
    UART_Print(VERBOSE_UART_ID, "07-RXPHE | ");
  } /* Receiver PHY Header Error */
  if ((status & SYS_STATUS_RXFCE) > 0u) {
    UART_Print(VERBOSE_UART_ID, "08-RXFCE | ");
  } /* Receiver FCS Error */
  if ((status & SYS_STATUS_RXRFSL) > 0u) {
    UART_Print(VERBOSE_UART_ID, "09-RXRFSL | ");
  } /* Receiver Reed Solomon Frame Sync Loss */
  if ((status & SYS_STATUS_RXSFDTO) > 0u) {
    UART_Print(VERBOSE_UART_ID, "10-RXSFDTO | ");
  } /* Receive SFD timeout */
  if ((status & SYS_STATUS_RXRFTO) > 0u) {
    UART_Print(VERBOSE_UART_ID, "11-RXRFTO | ");
  } /* Receive Frame Wait Timeout */
  if ((status & SYS_STATUS_RXPTO) > 0u) {
    UART_Print(VERBOSE_UART_ID, "12-RXPTO | ");
  } /* Preamble detection timeout */
  if ((status & SYS_STATUS_AFFREJ) > 0u) {
    UART_Print(VERBOSE_UART_ID, "13-AFFREJ | ");
  } /* Automatic Frame Filtering rejection */
  if ((status & SYS_STATUS_LDEERR) > 0u) {
    UART_Print(VERBOSE_UART_ID, "14-LDEERR | ");
  } /* Leading edge detection processing error */
  if ((status & SYS_STATUS_RXOVRR) > 0u) {
    UART_Print(VERBOSE_UART_ID, "15-RXOVRR | ");
  } /* Receiver Overrun */
  if ((status & SYS_STATUS_RXRSCS) > 0u) {
    UART_Print(VERBOSE_UART_ID, "16-RXRSCS | ");
  } /* Receiver Reed-Solomon Correction Status */
  if ((status & SYS_STATUS_RXPREJ) > 0u) {
    UART_Print(VERBOSE_UART_ID, "17-RXPREJ | ");
  } /* Receiver Preamble Rejection */

  // ALL RX DOUBLE BUFFER MODE EVENTS
  if ((status & SYS_STATUS_HSRBP) > 0u) {
    UART_Print(VERBOSE_UART_ID, "18-HSRBP | ");
  } /* Host Side Receive Buffer Pointer */
  if ((status & SYS_STATUS_ICRBP) > 0u) {
    UART_Print(VERBOSE_UART_ID, "19-ICRBP | ");
  } /* IC side Receive Buffer Pointer READ ONLY */

  // ALL TX EVENTS
  if ((status & SYS_STATUS_AAT) > 0u) {
    UART_Print(VERBOSE_UART_ID, "20-AAT | ");
  } /* Automatic Acknowledge Trigger */
  if ((status & SYS_STATUS_TXFRB) > 0u) {
    UART_Print(VERBOSE_UART_ID, "21-TXFRB | ");
  } /* Transmit Frame Begins */
  if ((status & SYS_STATUS_TXPRS) > 0u) {
    UART_Print(VERBOSE_UART_ID, "22-TXPRS | ");
  } /* Transmit Preamble Sent */
  if ((status & SYS_STATUS_TXPHS) > 0u) {
    UART_Print(VERBOSE_UART_ID, "23-TXPHS | ");
  } /* Transmit PHY Header Sent */
  if ((status & SYS_STATUS_TXFRS) > 0u) {
    UART_Print(VERBOSE_UART_ID, "24-TXFRS | ");
  } /* Transmit Frame Sent: This is set when the transmitter has completed the
       sending of a frame */
  if ((status & SYS_STATUS_TXBERR) > 0u) {
    UART_Print(VERBOSE_UART_ID, "25-TXBERR | ");
  } /* Transmit Buffer Error */
  if ((status & SYS_STATUS_TXPUTE) > 0u) {
    UART_Print(VERBOSE_UART_ID, "26-TXPUTE | ");
  } /* Transmit power up time error */

  // ALL CLOCK EVENTS
  if ((status & SYS_STATUS_CPLOCK) > 0u) {
    UART_Print(VERBOSE_UART_ID, "27-CPLOCK | ");
  } /* Clock PLL Lock */
  if ((status & SYS_STATUS_ESYNCR) > 0u) {
    UART_Print(VERBOSE_UART_ID, "28-ESYNCR | ");
  } /* External Sync Clock Reset */
  if ((status & SYS_STATUS_RFPLL_LL) > 0u) {
    UART_Print(VERBOSE_UART_ID, "29-RFPLL_LL | ");
  } /* RF PLL Losing Lock */
  if ((status & SYS_STATUS_CLKPLL_LL) > 0u) {
    UART_Print(VERBOSE_UART_ID, "30-CLKPLL_LL | ");
  } /* Clock PLL Losing Lock */

  // MISC EVENTS
  if ((status & SYS_STATUS_GPIOIRQ) > 0u) {
    UART_Print(VERBOSE_UART_ID, "31-GPIOIRQ | ");
  } /* GPIO interrupt */
  if ((status & SYS_STATUS_SLP2INIT) > 0u) {
    UART_Print(VERBOSE_UART_ID, "32-SLP2INIT | ");
  } /* SLEEP to INIT */
  if ((status & SYS_STATUS_HPDWARN) > 0u) {
    UART_Print(VERBOSE_UART_ID, "33-HPDWARN | ");
  } /* Half Period Delay Warning */
  if ((status & SYS_STATUS_IRQS) > 0u) {
    UART_Print(VERBOSE_UART_ID, "34-IRQS | ");
  } /* Interrupt Request Status READ ONLY */

  UART_Print(VERBOSE_UART_ID, "\r\n");
#endif
}

/**
 * @brief  Read register from OTP (0x2D)
 *         See pg. 56/213 and 155/213 of DW1000 User Manual 2.01
 * @param  None
 * @retval None
 */
uint32_t DW1000_ReadOTP(uint16_t reg) {
  uint32_t otp_rdat_val;

  uint8_t val8;
  uint16_t val16 = reg;
  DW1000_Write(OTP_IF_ID, OTP_ADDR, &val16, OTP_ADDR_LEN); // write OTP address

  val8 = 0x03;
  DW1000_Write(OTP_IF_ID, OTP_CTRL, &val8, 1u); // command a read
  val8 = 0x01;
  DW1000_Write(OTP_IF_ID, OTP_CTRL, &val8, 1u); // disable read
  DW1000_Read(OTP_IF_ID, OTP_RDAT, (uint8_t *)&otp_rdat_val,
              OTP_RDAT_LEN); // read OTP register
  val8 = 0x00;
  DW1000_Write(OTP_IF_ID, OTP_CTRL, &val8, 1u); // disable read

  return otp_rdat_val;
}

//--------------------------------------------------------------------------------

/**
 * @brief  Load microcode from ROM to RAM
 *         This is needed for receiving timestamp (RTLS) and diagnostic
 * information from received frames
 * @param  None
 * @retval None
 */
void DW1000_LoadMicroCode(void) {
#ifdef _VERBOSE
  char str[100];
  UART_Print(VERBOSE_UART_ID, "---------------------------------\r\n");
  UART_Print(VERBOSE_UART_ID, "LOAD MICROCODE\r\n");
#endif

  // See pg. 175/213 of DW1000 User Manual 2.01
  uint16_t val16 = 0x0301;
  DW1000_Write(PMSC_ID, PMSC_CTRL0_OFFSET, &val16, 2u);
#ifdef _VERBOSE
  uint16_t pmsc_ctrl0_val;
  DW1000_Read(PMSC_ID, PMSC_CTRL0_OFFSET, (uint8_t *)&pmsc_ctrl0_val, 2u);
  sprintf(str, "PMSC-CTRL0: 0x%04X (0x%04X)\r\n", (unsigned int)pmsc_ctrl0_val,
          (unsigned int)val16);
  UART_Print(VERBOSE_UART_ID, str);
  Delay(5);
#endif

  // See pg. 157/213 of DW1000 User Manual 2.01
  // Note: this register will automatically clear when loading the microcode is
  // accomplished
  val16 = OTP_CTRL_LDELOAD;
  DW1000_Write(OTP_IF_ID, OTP_CTRL, &val16, OTP_CTRL_LEN);
  Delay(1); // wait at least 150 usec

  // See pg. 175/213 of DW1000 User Manual 2.01
  val16 = 0x0200;
  DW1000_Write(PMSC_ID, PMSC_CTRL0_OFFSET, &val16, 2u);
#ifdef _VERBOSE
  DW1000_Read(PMSC_ID, PMSC_CTRL0_OFFSET, (uint8_t *)&pmsc_ctrl0_val,
              PMSC_CTRL0_LEN);
  sprintf(str, "PMSC-CTRL0: 0x%08X (0x%08X)\r\n", (unsigned int)pmsc_ctrl0_val,
          (unsigned int)val16);
  UART_Print(VERBOSE_UART_ID, str);
  Delay(5);
  UART_Print(VERBOSE_UART_ID, "---------------------------------\r\n");
#endif
}

//--------------------------------------------------------------------------------

// Configure LEDs
void DW1000_ConfigLEDS(void) {
#ifdef _VERBOSE
  char str[100];
#endif

  // Configure GPIO control and status (0x26)
  // See pg. 117/213 of DW1000 User Manual 2.01
  // - RXOKLED: Receiver completes the reception of a frame with a good FCS/CRC
  // - SFDLED: Receiver detects the SFD sequence in the RX frame
  // - RXLED: Stays on for a brief period after the receiver is turned off
  // - TXLED: Asserted when the transmitter completes sending a frame
  uint32_t val32 =
      GPIO_RXOKLED_EN | GPIO_SFDLED_EN | GPIO_RXLED_EN | GPIO_TXLED_EN;
  DW1000_Write(GPIO_CTRL_ID, GPIO_MODE_OFFSET, &val32, GPIO_MODE_LEN);
#ifdef _VERBOSE
  uint32_t gpio_mode;
  DW1000_Read(GPIO_CTRL_ID, GPIO_MODE_OFFSET, (uint8_t *)&gpio_mode,
              GPIO_MODE_LEN);
  sprintf(str, "GPIO-MODE: 0x%08X (0x%08X)\r\n", (unsigned int)gpio_mode,
          (unsigned int)val32);
  UART_Print(VERBOSE_UART_ID, str);
  Delay(5);
#endif

// Configure LED management settings (0x36:28)
// See pg. 181/213 of DW1000 User Manual 2.01
#define PMSC_LEDC_BLINK_TIM 0x00000001UL
  val32 = PMSC_LEDC_BLNKEN | PMSC_LEDC_BLINK_TIM;
  DW1000_Write(PMSC_ID, PMSC_LEDC_OFFSET, &val32, PMSC_LEDC_LEN);
#ifdef _VERBOSE
  uint32_t pmsc_ledc;
  DW1000_Read(PMSC_ID, PMSC_LEDC_OFFSET, (uint8_t *)&pmsc_ledc, PMSC_LEDC_LEN);
  sprintf(str, "PMSC-LEDC: 0x%08X (0x%08X)\r\n", (unsigned int)pmsc_ledc,
          (unsigned int)val32);
  UART_Print(VERBOSE_UART_ID, str);
  Delay(5);
#endif
}

//--------------------------------------------------------------------------------
// READ/WRITE DATA BUFFERS

/**
 * @brief  Receive data over DW1000
 *         RXFCG triggers this interrupt, indicating the completion of the frame
 *         reception process and that this frame has passed CRC checking
 *         It also implicitly indicates that LDE adjustments have been made
 * @param  buf Pointer to receive buffer
 * @retval Number of bytes received
 */
// @TODO Frame quality information is not necessary for static nodes
uint8_t DW1000_Receive(uint8_t *buf) {
  // Leading edge detection processing error
  if (DW1000_GetSysStatus(SYS_STATUS_LDEERR)) {
    UART_Print(VERBOSE_UART_ID, "LDEERR\r\n");
    DW1000_ClearSysStatus();
    return 0u;
  }

  // Read Rx Frame Information Register (0x10)
  // See pg. 88/213 of DW1000 User Manual 2.01
  uint32_t rx_finfo;
  DW1000_Read(RX_FINFO_ID, 0x00, (uint8_t *)&rx_finfo, RX_FINFO_LEN);
  uint8_t frame_length = RX_FINFO_RXFLEN_MASK & rx_finfo;
  DW1000_rx_sinfo.preamble_count =
      (RX_FINFO_RXPACC_MASK & rx_finfo) >> RX_FINFO_RXPACC_SHIFT;

  // Read data from Receive Data Buffer
  // See pg. 88/213 of DW1000 User Manual 2.01
  DW1000_Read(RX_BUFFER_ID, 0x00, buf, frame_length - 2u);

  // Read Rx Frame Quality Information (0x12)
  // See pg. 90/213 of DW1000 User Manual 2.01
  uint16_t rx_fqual_val[4];
  DW1000_Read(RX_FQUAL_ID, 0x00, (uint8_t *)rx_fqual_val, RX_FQUAL_LEN);
  DW1000_rx_sinfo.std_noise = rx_fqual_val[0]; // standard deviation of noise
  DW1000_rx_sinfo.fp_ampl2 = rx_fqual_val[1];  // first path amplitude point 2
  DW1000_rx_sinfo.fp_ampl3 = rx_fqual_val[2];  // first path amplitude point 3
  DW1000_rx_sinfo.cir_pwr = rx_fqual_val[3];   // channel impulse response power

  // Read Receive Time Stamp (0x15)
  // See pg. 93/213 of DW1000 User Manual 2.01
  uint8_t data[RX_TIME_LLEN];
  DW1000_Read(RX_TIME_ID, 0x00, data, RX_TIME_LLEN); // adjusted timestamp
  uint32_t ts_lsb = ((uint32_t)data[3] << 24u) | ((uint32_t)data[2] << 16u) |
                    ((uint32_t)data[1] << 8u) | ((uint32_t)data[0] << 0u);
  DW1000_rx_timestamp = data[4];
  DW1000_rx_timestamp <<= 32u;
  DW1000_rx_timestamp |= ts_lsb;
  DW1000_rx_sinfo.fp_index = ((uint16_t)data[6] << 8u) | (uint16_t)data[5];
  DW1000_rx_sinfo.fp_ampl1 = ((uint16_t)data[8] << 8u) |
                             (uint16_t)data[7]; // first path amplitude point 1

  return frame_length - 2u;
}

/**
 * @brief  Transmit data over DW1000
 *         Write to transmit data buffer (0x09)
 * @param  buf Pointer to transmit buffer
 * @param  size Size of transmit buffer [0..127]
 * @retval None
 */
void DW1000_Transmit(uint8_t *buf, uint8_t size) {
  if (size > TX_BUFFER_LEN)
    size = TX_BUFFER_LEN;

  // Write data to Transmit Data Buffer
  // See pg. 73/213 of DW1000 User Manual 2.01
  DW1000_Write(TX_BUFFER_ID, 0x00, buf, size);

  // Set frame length (size + 2 CRC bytes)
  // @TODO This can be initialised at the beginning; no need to dynamically
  // change it if transmission size is fixed
  uint8_t val8 = size + 2u;
  DW1000_Write(TX_FCTRL_ID, 0x00, &val8, 1u);
  // char str[100]; sprintf(str,"size: %u\r\n", size);

  // Start transmission
  // Configure system control register (0x0D)
  // See pg. 75/213 of DW1000 User Manual 2.01
  // - SFCST = 0, Enable auto-FCS transmission (automatically append CRC)
  // - TXSTRT = 1, Start transmission
  // - TXDLYS = 0, Disable transmitter delayed sending
  // - CANSFCS = 0, Enable auto-FCS transmission
  // - TRXOFF = 0, Transceiver on
  // - WAIT4RESP = 1, Wait for response (automatically turn on receiver after
  // transmission)
  // - RXENAB = 0, Disable receiver
  // - RXDLYE = 0, Disable receiver delayed receiving
  // - HRBPT = 0, Disable host side receive buffer pointer toggle (double buffer
  // mode)
  uint32_t val32 = SYS_CTRL_TXSTRT | SYS_CTRL_WAIT4RESP;
  DW1000_Write(SYS_CTRL_ID, 0x00, &val32, SYS_CTRL_LEN);

  // Wait until transmitter has completed sending the frame
  // See pg. 80/213 of DW1000 User Manual 2.01
  uint8_t cnt = 0u;
  while (!DW1000_GetSysStatus(SYS_STATUS_TXFRS)) {
    if (cnt++ > 100) {
      // UART_Print(VERBOSE_UART_ID,"ERROR: SYS_STATUS_TXFRS | Status unknown..
      // Attempting recovery\r\n"); Delay(5); DW1000_PrintSysStatus();
      // UART_Print(VERBOSE_UART_ID,"<");
      // LED_Blink(0,100);
      LED_Toggle(0);
      DW1000_ClearSysStatus();
      return;
    }
  }

  // Read Transmit Time Stamp (0x17)
  // See pg. 95/213 of DW1000 User Manual 2.01
  uint8_t data[TX_STAMP_LEN];
  DW1000_Read(TX_TIME_ID, 0x00, data, TX_STAMP_LEN); // adjusted timestamp
  uint32_t tsl = ((uint32_t)data[3] << 24u) + ((uint32_t)data[2] << 16u) +
                 ((uint32_t)data[1] << 8u) + ((uint32_t)data[0] << 0u);
  DW1000_tx_timestamp = data[4];
  DW1000_tx_timestamp <<= 32u;
  DW1000_tx_timestamp |= tsl;
}

/**
 * @brief  Get system timestamp [DW1000 time units]
 *         Note: See pg. 69/213 of DW1000 User Manual 2.01
 *         Counter wrap period is 2^40 / (128 x 499.2e6) = 17.2074 sec
 * @param  None
 * @retval Timestamp [DW1000 time units]
 */
uint64_t DW1000_GetSysTimestamp(void) {
  uint8_t data[SYS_TIME_LEN];
  DW1000_Read(SYS_TIME_ID, 0x00, data, SYS_TIME_LEN); // raw timestamp

  uint32_t ts_lsb = ((uint32_t)data[3] << 24u) | ((uint32_t)data[2] << 16u) |
                    ((uint32_t)data[1] << 8u) | ((uint32_t)data[0] << 0u);
  uint64_t ts = data[4];
  ts <<= 32u;
  ts |= ts_lsb;

  // DEBUG: print to console
  // char str[100]; sprintf(str, "SYS: %.6f\r\n", ts*DW1000_TIME_UNIT);
  // UART_Print(VERBOSE_UART_ID,str); char str[100]; sprintf(str, "SYS:
  // %02x%02x%02x%02x%02x\r\n", data[4],data[3],data[2],data[1],data[0]);
  // UART_Print(VERBOSE_UART_ID,str);

  return ts;
}

uint64_t DW1000_GetRxTimestamp(void) { return DW1000_rx_timestamp; }
uint64_t DW1000_GetTxTimestamp(void) { return DW1000_tx_timestamp; }

float DW1000_GetSNR(void) {
  // Compute signal quality (or signal to noise ratio) using standard deviation
  // of channel impulse response estimate and first path amplitude See
  // Section 4.7 (pg. 41/213) for Assessing quality of reception and RX
  // timestamp
  return 10.0f * log10f((float)DW1000_rx_sinfo.fp_ampl2 /
                        (float)DW1000_rx_sinfo.std_noise); // [dBm]
}

float DW1000_GetFPPower(void) {
  // Compute estimated receive first path signal power
  // Note: As a rule of thumb, if the difference between rx_power and fp_power
  // is < 6 dBm, the channel is likely to be LOS. If the difference is greater
  // than 10 dBm, then it is likely to be NLOS. estimate first path signal power
  // [dBm]
  float num = DW1000_rx_sinfo.fp_ampl1 * DW1000_rx_sinfo.fp_ampl1 +
              DW1000_rx_sinfo.fp_ampl2 * DW1000_rx_sinfo.fp_ampl2 +
              DW1000_rx_sinfo.fp_ampl3 * DW1000_rx_sinfo.fp_ampl3;
  if (DW1000_rx_sinfo.cir_pwr == 0u || num == 0u)
    return 0.0f;
  float fp_power =
      10.0f * log10f(num / (DW1000_rx_sinfo.preamble_count *
                            DW1000_rx_sinfo.preamble_count)); // [dBm]
  switch (DW1000_config.prf) {
  case PRF_4MHZ:
    break;
  case PRF_16MHZ:
    fp_power -= 115.72f;
    break;
  case PRF_64MHZ:
    fp_power -= 121.74f;
    break;
  }

  return fp_power; // [dBm]
}

float DW1000_GetRxPower(void) {
  // Compute estimated receive signal power and first path signal power
  // Note: As a rule of thumb, if the difference between rx_power and fp_power
  // is < 6 dBm, the channel is likely to be LOS. If the difference is greater
  // than 10 dBm, then it is likely to be NLOS. estimate receive signal power
  // [dBm]
  if (DW1000_rx_sinfo.cir_pwr == 0u || DW1000_rx_sinfo.preamble_count == 0u)
    return 0.0f;
  float rx_power =
      10.0f *
      log10f((DW1000_rx_sinfo.cir_pwr << 17u) /
             (DW1000_rx_sinfo.preamble_count * DW1000_rx_sinfo.preamble_count));
  switch (DW1000_config.prf) {
  case PRF_4MHZ:
    break;
  case PRF_16MHZ:
    rx_power -= 115.72f;
    break;
  case PRF_64MHZ:
    rx_power -= 121.74f;
    break;
  }

  return rx_power; // [dBm]
}

void DW1000_TransceiverOff(void) {
  uint32_t val32 = SYS_CTRL_TRXOFF;
  DW1000_Write(SYS_CTRL_ID, 0x00, &val32, SYS_CTRL_LEN);
}
void DW1000_ReceiverOn(void) {
  uint32_t val32 = SYS_CTRL_RXENAB;
  DW1000_Write(SYS_CTRL_ID, 0x00, &val32, SYS_CTRL_LEN);
}

//--------------------------------------------------------------------------------
// UTILITY FUNCTIONS

// Read background energy level; only provides a relative value between channels
// for comparison purposes See pg. 110/213 of DW1000 User Manual 2.01
float DW1000_ReadBackgroundEnergy(DW1000_channel_t ch) {
  // set DIS_AM bit to 0
  uint16_t val16 = 0x00;
  DW1000_Write(AGC_CTRL_ID, AGC_CTRL1_OFFSET, &val16, AGC_CTRL1_LEN);

  // turn on receiver
  DW1000_ReceiverOn();

  // wait 32 usec for AGC to settle
  Delay(1);

  // set DIS_AM bit to freeze the result
  val16 = AGC_CTRL1_DIS_AM;
  DW1000_Write(AGC_CTRL_ID, AGC_CTRL1_OFFSET, &val16, AGC_CTRL1_LEN);

  // read EDG1 and EDV2
  uint32_t agc_stat1_val;
  DW1000_Read(AGC_CTRL_ID, AGC_STAT1_OFFSET, (uint8_t *)&agc_stat1_val,
              AGC_STAT1_LEN);
  uint8_t edg1 =
      (agc_stat1_val & AGC_STAT1_EDG1_MASK) >>
      6u; // 5-bit gain value relates to input noise power measurement
  uint8_t edv2 =
      (agc_stat1_val & AGC_STAT1_EDV2_MASK) >>
      11u; // 9-bit value relates to the input noise power measurement

  char str[100];
  sprintf(str, "AGC-STAT1: 0x%08X\r\n", (unsigned int)agc_stat1_val);
  UART_Print(VERBOSE_UART_ID, str);
  sprintf(str, "EDG1: 0x%02X (%u)\r\n", (unsigned int)edg1, (unsigned int)edg1);
  UART_Print(VERBOSE_UART_ID, str);
  sprintf(str, "EDV2: 0x%02X (%u)\r\n", (unsigned int)edv2, (unsigned int)edv2);
  UART_Print(VERBOSE_UART_ID, str);

  // compute noise energy level
  float nel = ((float)edv2 - 40.0f) * powf(10.0, (float)edg1);

  switch (ch) {
  case CH1:
    nel *= 1.3335f;
    break;
  case CH2:
    nel *= 1.3335f;
    break;
  case CH3:
    nel *= 1.3335f;
    break;
  case CH4:
    nel *= 1.3335f;
    break;
  case CH5:
    nel *= 1.0000f;
    break;
  case CH7:
    nel *= 1.0000f;
    break;
  }

  return nel;
}

// Read temperature
// See pg. 57/213 of DW1000 User Manual 2.01
static volatile float temperature = 0.0f;
static volatile float voltage = 0.0f;
float DW1000_ReadTemperature(void) {
  uint8_t data[2];
  uint8_t val8;

  val8 = 0x80;
  DW1000_Write(RF_CONF_ID, 0x11, &val8, 1u); // Enable TLD Bias
  val8 = 0x0A;
  DW1000_Write(RF_CONF_ID, 0x12, &val8, 1u); // Enable TLD Bias and ADC Bias
  val8 = 0x0F;
  DW1000_Write(RF_CONF_ID, 0x12, &val8,
               1u); // Enable Outputs (only after Biases are up and running)

  // read SAR inputs
  val8 = 0x00;
  DW1000_Write(TX_CAL_ID, TC_SARC_OFFSET, &val8, 1u);
  val8 = 0x01;
  DW1000_Write(TX_CAL_ID, TC_SARC_OFFSET, &val8, 1u);

  // read temperature register
  // DW1000_Read(TX_CAL_ID, TC_SARL_SAR_LTEMP_OFFSET, &data, 1u); // read
  // temperature
  DW1000_Read(TX_CAL_ID, TC_SARL_SAR_LVBAT_OFFSET, data,
              2u); // read temperature

  temperature =
      1.13f * data[1] -
      113.0f; // refer to dwt_readtempvbat() in deca_device.c of DecaWave's API
  voltage =
      0.0057f * data[0] +
      2.3f; // refer to dwt_readtempvbat() in deca_device.c of DecaWave's API

  // clear SAR enable
  val8 = 0x00;
  DW1000_Write(TX_CAL_ID, TC_SARC_OFFSET, &val8, 1u);

  return temperature;
}

float DW1000_GetVoltage(void) { return voltage; }
float DW1000_GetTemperature(void) { return temperature; }

// Read accumulator CIR memory (0x25)
// See pg. 116/213 of DW1000 User Manual 2.01
void DW1000_ReadAccumulatorCIR(uint16_t *real, uint16_t *img) {
  // Since there is an internal memory access delay when reading the
  // accumulator, the first octet output is a dummy octet that should be
  // discarded
  uint8_t data[ACC_MEM_LEN + 1u];
  DW1000_Read(ACC_MEM_ID, 0x00, data, ACC_MEM_LEN + 1u);

  uint8_t cnt = 0u;
  uint8_t i;
  for (i = 1u; i < ACC_MEM_LEN + 1u; i += 4u) {
    real[cnt] = ((uint16_t)data[i + 1u] << 8u) | (uint16_t)data[i + 0u];
    img[cnt] = ((uint16_t)data[i + 3u] << 8u) | (uint16_t)data[i + 2u];
    cnt++;
  }
}

//--------------------------------------------------------------------------------
// LEGACY
// this is what is suggested in the user manual
// keep for legacy purposes
void DW1000_ConfigDefault(void) {
#ifdef _VERBOSE
  char str[100];
#endif

  // DEFAULT:
  // Channel 5, Preamble Code 4, and Mode 2
  // Data Rate: 6.8 Mbps, PRF: 16 MHz, Preamble: 128 Symbols, PAC Size: 8, Data
  // Length: 12 Bytes, Packet Duration 152 usec Use case: RTLS, TDOA Scheme,
  // Short Range, High Density

  // Initialise DW1000 configurations
  DW1000_config.channel = CH5;
  DW1000_config.data_rate = DATA_RATE_6800;
  DW1000_config.prf = PRF_16MHZ;
  DW1000_config.preamble_code = PREAMBLE_CODE_4;
  DW1000_config.preamble_length = PREAMBLE_LENGTH_128;
  DW1000_config.pac_size = PAC_SIZE_8;

  //-----------------------------------------------------------------
  // Default configurations are sub-optimal and should be modified
  // See Section 2.5.5 (pg. 21/213) of DW1000 User Manual 2.01
  uint8_t val8;
  uint16_t val16;
  uint32_t val32;

  // RX_FWTO
  // See pg. 74/213 of DW1000 User Manual 2.01
  // set receiver frame wait timeout period
  val16 = 2000u; // [usec]
  DW1000_Write(RX_FWTO_ID, 0x00, &val16, RX_FWTO_LEN);
#ifdef _VERBOSE
  uint16_t rx_fwto_val;
  DW1000_Read(RX_FWTO_ID, 0x00, (uint8_t *)&rx_fwto_val, RX_FWTO_LEN);
  sprintf(str, "RX-FWTO: 0x%04X = 0x%04X\r\n", (unsigned int)rx_fwto_val,
          (unsigned int)val16);
  UART_Print(4, str);
  Delay(5);
#endif

  // RX_SNIFF
  // Configure Sniff Mode
  // See pg. 98/213 of DW1000 User Manual 2.01

  // On duration is programmed in units of PAC (min. value 2)
  // Note: one PAC symbol is approximatley 1 usec
  // Regular Sniff Mode: Off duration is programmed in units of 1 usec (uses 125
  // MHz digital PLL clock) Low Duty-Cycle Sniff Mode: Off duration is
  // programmed in units of 6.6 usec (uses 19.2 MHz XTI clock)
  //   float on_off_ratio = 1.0f;
  //   uint8_t sniff_ont = 4; // [PAC units]
  //   uint8_t sniff_offt = (uint8_t)((sniff_ont/on_off_ratio)*8u); // [usec]
  //   val16 = (sniff_offt<<8u) | sniff_ont;
  //   DW1000_Write(RX_SNIFF_ID, 0x00, &val16, RX_SNIFF_LEN);
  // #ifdef _VERBOSE
  //   uint16_t rx_sniff_val;
  //   DW1000_Read(RX_SNIFF_ID, 0x00, (uint8_t *)&rx_sniff_val, RX_SNIFF_LEN);
  //   sprintf(str, "RX-SNIFF: 0x%04X (0x%04X)\r\n", (unsigned int)rx_sniff_val,
  //   (unsigned int)val16); UART_Print(VERBOSE_UART_ID,str); Delay(5);
  // #endif
  //-------------------------------------------------------------------------

  // AGC_TUNE1
  // See pg. 110/213 of DW1000 User Manual 2.01
  val16 = AGC_TUNE1_16M;
  DW1000_Write(AGC_CTRL_ID, AGC_TUNE1_OFFSET, &val16, AGC_TUNE1_LEN);
#ifdef _VERBOSE
  uint16_t agc_tune1_val;
  DW1000_Read(AGC_CTRL_ID, AGC_TUNE1_OFFSET, (uint8_t *)&agc_tune1_val,
              AGC_TUNE1_LEN);
  sprintf(str, "AGC-TUNE1: 0x%04X (0x%04X)\r\n", (unsigned int)agc_tune1_val,
          (unsigned int)val16);
  UART_Print(VERBOSE_UART_ID, str);
  Delay(5);
#endif

  // AGC_TUNE2
  // See pg. 110/213 of DW1000 User Manual 2.01
  val32 = AGC_TUNE2_VAL;
  DW1000_Write(AGC_CTRL_ID, AGC_TUNE2_OFFSET, &val32, AGC_TUNE2_LEN);
#ifdef _VERBOSE
  uint32_t agc_tune2_val;
  DW1000_Read(AGC_CTRL_ID, AGC_TUNE2_OFFSET, (uint8_t *)&agc_tune2_val,
              AGC_TUNE2_LEN);
  sprintf(str, "AGC-TUNE2: 0x%08X (0x%08X)\r\n", (unsigned int)agc_tune2_val,
          (unsigned int)val32);
  UART_Print(VERBOSE_UART_ID, str);
  Delay(5);
#endif

  // AGC_TUNE3
  // See pg. 112/213 of DW1000 User Manual 2.01
  val16 = AGC_TUNE3_VAL;
  DW1000_Write(AGC_CTRL_ID, AGC_TUNE3_OFFSET, &val16, AGC_TUNE3_LEN);
#ifdef _VERBOSE
  uint16_t agc_tune3_val;
  DW1000_Read(AGC_CTRL_ID, AGC_TUNE3_OFFSET, (uint8_t *)&agc_tune3_val,
              AGC_TUNE3_LEN);
  sprintf(str, "AGC-TUNE3: 0x%04X (0x%04X)\r\n", (unsigned int)agc_tune3_val,
          (unsigned int)val16);
  UART_Print(VERBOSE_UART_ID, str);
  Delay(5);
#endif

  // DRX_TUNE2
  // See pg. 132/213 of DW1000 User Manual 2.01
  val32 = DRX_TUNE2_PAC08_PRF16;
  DW1000_Write(DRX_CONF_ID, DRX_TUNE2_OFFSET, &val32, DRX_TUNE2_LEN);
#ifdef _VERBOSE
  uint32_t drx_tune2_val;
  DW1000_Read(DRX_CONF_ID, DRX_TUNE2_OFFSET, (uint8_t *)&drx_tune2_val,
              DRX_TUNE2_LEN);
  sprintf(str, "DRX-TUNE2: 0x%08X (0x%08X)\r\n", (unsigned int)drx_tune2_val,
          (unsigned int)val32);
  UART_Print(VERBOSE_UART_ID, str);
  Delay(5);
#endif

  // DRX_SFDTOC
  // See pg. 133/213 of DW1000 User Manual 2.01
  // enable SFD detection timeout
  // val16 = 0xFF;
  val16 = 0x89;
  DW1000_Write(DRX_CONF_ID, DRX_SFDTOC_OFFSET, &val16, DRX_SFDTOC_LEN);
#ifdef _VERBOSE
  uint16_t drx_sfdtoc_val;
  DW1000_Read(DRX_CONF_ID, DRX_SFDTOC_OFFSET, (uint8_t *)&drx_sfdtoc_val,
              DRX_SFDTOC_LEN);
  sprintf(str, "DRX-SFDTOC: 0x%04X = 0x%04X\r\n", (unsigned int)drx_sfdtoc_val,
          (unsigned int)val16);
  UART_Print(4, str);
  Delay(5);
#endif

  // DRX_PRETOC
  // See pg. 134/213 of DW1000 User Manual 2.01
  // enable preamble detection timeout
  // Note: preamble length (1024) divided by PAC size (32)
  val16 = 16u;
  DW1000_Write(DRX_CONF_ID, DRX_PRETOC_OFFSET, &val16, DRX_PRETOC_LEN);
#ifdef _VERBOSE
  uint16_t drx_pretoc_val;
  DW1000_Read(DRX_CONF_ID, DRX_PRETOC_OFFSET, (uint8_t *)&drx_pretoc_val,
              DRX_PRETOC_LEN);
  sprintf(str, "DRX-PRETOC: 0x%04X = 0x%04X\r\n", (unsigned int)drx_pretoc_val,
          (unsigned int)val16);
  UART_Print(4, str);
  Delay(5);
#endif

  // LDE_CFG1
  // See pg. 162/213 of DW1000 User Manual 2.01
  val8 = 0x6D;
  DW1000_Write(LDE_IF_ID, LDE_CFG1_OFFSET, &val8, LDE_CFG1_LEN);
#ifdef _VERBOSE
  uint8_t lde_cfg1_val;
  DW1000_Read(LDE_IF_ID, LDE_CFG1_OFFSET, &lde_cfg1_val, LDE_CFG1_LEN);
  sprintf(str, "LDE-CFG1: 0x%02X (0x%02X)\r\n", (unsigned int)lde_cfg1_val,
          (unsigned int)val8);
  UART_Print(VERBOSE_UART_ID, str);
  Delay(5);
#endif

  // LDE_CFG2
  // See pg. 164/213 of DW1000 User Manual 2.01
  val16 = LDE_CFG2_PRF16;
  DW1000_Write(LDE_IF_ID, LDE_CFG2_OFFSET, &val16, LDE_CFG2_LEN);
#ifdef _VERBOSE
  uint16_t lde_cfg2_val;
  DW1000_Read(LDE_IF_ID, LDE_CFG2_OFFSET, (uint8_t *)&lde_cfg2_val,
              LDE_CFG2_LEN);
  sprintf(str, "LDE-CFG2: 0x%04X (0x%04X)\r\n", (unsigned int)lde_cfg2_val,
          (unsigned int)val16);
  UART_Print(VERBOSE_UART_ID, str);
  Delay(5);
#endif

  // TX_POWER
  // See pg. 100/213 of DW1000 User Manual 2.01
  // val32 = 0x0E082848; // default value
  val32 = 0x1F1F1F1F; // maximum power!!!
  DW1000_Write(TX_POWER_ID, 0x00, &val32, TX_POWER_LEN);
#ifdef _VERBOSE
  uint32_t tx_power_val;
  DW1000_Read(TX_POWER_ID, 0x00, (uint8_t *)&tx_power_val, TX_POWER_LEN);
  sprintf(str, "TX-POWER: 0x%08X (0x%08X)\r\n", (unsigned int)tx_power_val,
          (unsigned int)val32);
  UART_Print(VERBOSE_UART_ID, str);
  Delay(5);
#endif

  // RF_TXCTRL
  // See pg. 137/213 of DW1000 User Manual 2.01
  // @TODO why is the value of this register 0xDE1E3FE0 instead of 0x001E3FE0?
  val32 = RF_TXCTRL_CH5;
  DW1000_Write(RF_CONF_ID, RF_TXCTRL_OFFSET, &val32, RF_TXCTRL_LEN);
#ifdef _VERBOSE
  uint32_t rf_txctrl_val;
  DW1000_Read(RF_CONF_ID, RF_TXCTRL_OFFSET, (uint8_t *)&rf_txctrl_val,
              RF_TXCTRL_LEN);
  sprintf(str, "RF-TXCTRL: 0x%08X (0x%08X)\r\n", (unsigned int)rf_txctrl_val,
          (unsigned int)val32);
  UART_Print(VERBOSE_UART_ID, str);
  Delay(5);
#endif

  // TC_PGDELAY
  // See pg. 142/213 of DW1000 User Manual 2.01
  val8 = TC_PGDELAY_CH5;
  DW1000_Write(TX_CAL_ID, TC_PGDELAY_OFFSET, &val8, TC_PGDELAY_LEN);
#ifdef _VERBOSE
  uint8_t tc_pgdelay_val;
  DW1000_Read(TX_CAL_ID, TC_PGDELAY_OFFSET, &tc_pgdelay_val, TC_PGDELAY_LEN);
  sprintf(str, "TC-PGDELAY: 0x%02X (0x%02X)\r\n", (unsigned int)tc_pgdelay_val,
          (unsigned int)val8);
  UART_Print(VERBOSE_UART_ID, str);
  Delay(5);
#endif

  // FS_PLLTUNE
  // See pg. 145/213 of DW1000 User Manual 2.01
  val8 = FS_PLLTUNE_CH5;
  DW1000_Write(FS_CTRL_ID, FS_PLLTUNE_OFFSET, &val8, FS_PLLTUNE_LEN);
#ifdef _VERBOSE
  uint8_t fs_plltune_val;
  DW1000_Read(FS_CTRL_ID, FS_PLLTUNE_OFFSET, &fs_plltune_val, FS_PLLTUNE_LEN);
  sprintf(str, "FS-PLLTUNE: 0x%02X (0x%02X)\r\n", (unsigned int)fs_plltune_val,
          (unsigned int)val8);
  UART_Print(VERBOSE_UART_ID, str);
  Delay(5);
#endif

  //-------------------------------------------------------------------------
  // LOAD MICROCODE

  // See pg. 175/213 of DW1000 User Manual 2.01
  val16 = 0x0301;
  DW1000_Write(PMSC_ID, PMSC_CTRL0_OFFSET, &val16, 2u);
#ifdef _VERBOSE
  uint32_t pmsc_ctrl0_val;
  DW1000_Read(PMSC_ID, PMSC_CTRL0_OFFSET, (uint8_t *)&pmsc_ctrl0_val, 2u);
  sprintf(str, "PMSC-CTRL0: 0x%04X (0x%04X)\r\n", (unsigned int)pmsc_ctrl0_val,
          (unsigned int)val16);
  UART_Print(VERBOSE_UART_ID, str);
  Delay(5);
#endif

  // See pg. 157/213 of DW1000 User Manual 2.01
  // Note: this register will automatically clear when loading the microcode is
  // accomplished
  val16 = OTP_CTRL_LDELOAD;
  DW1000_Write(OTP_IF_ID, OTP_CTRL, &val16, OTP_CTRL_LEN);
  Delay(1); // wait at least 150 usec

  // See pg. 175/213 of DW1000 User Manual 2.01
  val16 = 0x0200;
  DW1000_Write(PMSC_ID, PMSC_CTRL0_OFFSET, &val16, 2u);
#ifdef _VERBOSE
  DW1000_Read(PMSC_ID, PMSC_CTRL0_OFFSET, (uint8_t *)&pmsc_ctrl0_val,
              PMSC_CTRL0_LEN);
  sprintf(str, "PMSC-CTRL0: 0x%08X (0x%08X)\r\n", (unsigned int)pmsc_ctrl0_val,
          (unsigned int)val16);
  UART_Print(VERBOSE_UART_ID, str);
  Delay(5);
#endif

  //-------------------------------------------------------------------------
  // ENABLE INTERRUPTS

  val32 = SYS_MASK_MRXFCG;
  DW1000_Write(SYS_MASK_ID, 0x00, &val32, SYS_MASK_LEN);
#ifdef _VERBOSE
  uint32_t sys_mask_val;
  DW1000_Read(SYS_MASK_ID, 0x00, (uint8_t *)&sys_mask_val, SYS_MASK_LEN);
  sprintf(str, "SYS-MASK: 0x%08X (0x%08X)\r\n", (unsigned int)sys_mask_val,
          (unsigned int)val32);
  UART_Print(VERBOSE_UART_ID, str);
  Delay(5);
#endif

  //-------------------------------------------------------------------------
  // ENABLE LEDS

  val32 = GPIO_RXOKLED_EN | GPIO_SFDLED_EN | GPIO_RXLED_EN | GPIO_TXLED_EN;
  DW1000_Write(GPIO_CTRL_ID, GPIO_MODE_OFFSET, &val32, GPIO_MODE_LEN);
#ifdef _VERBOSE
  uint32_t gpio_mode;
  DW1000_Read(GPIO_CTRL_ID, GPIO_MODE_OFFSET, (uint8_t *)&gpio_mode,
              GPIO_MODE_LEN);
  sprintf(str, "GPIO-MODE: 0x%08X (0x%08X)\r\n", (unsigned int)gpio_mode,
          (unsigned int)val32);
  UART_Print(VERBOSE_UART_ID, str);
  Delay(5);
#endif

// Configure LED management settings (0x36:28)
// See pg. 181/213 of DW1000 User Manual 2.01
#define PMSC_LEDC_BLINK_TIM 0x00000001UL
  val32 = PMSC_LEDC_BLNKEN | PMSC_LEDC_BLINK_TIM;
  DW1000_Write(PMSC_ID, PMSC_LEDC_OFFSET, &val32, PMSC_LEDC_LEN);
#ifdef _VERBOSE
  uint32_t pmsc_ledc;
  DW1000_Read(PMSC_ID, PMSC_LEDC_OFFSET, (uint8_t *)&pmsc_ledc, PMSC_LEDC_LEN);
  sprintf(str, "PMSC-LEDC: 0x%04X (0x%04X)\r\n", (unsigned int)pmsc_ledc,
          (unsigned int)val32);
  UART_Print(VERBOSE_UART_ID, str);
  Delay(5);
#endif

  //-------------------------------------------------------------------------
  // CONFIGURE POWER MANAGEMENT SYSTEM CONTROL

  // Configure PMSC_CTRL0 (0x36:00)
  // See pg. 175/213 of DW1000 User Manual 2.01
  val32 = PMSC_CTRL0_DEFAULT | PMSC_CTRL0_GPDCE | PMSC_CTRL0_KHZCLKEN;
  DW1000_Write(PMSC_ID, PMSC_CTRL0_OFFSET, &val32, PMSC_CTRL0_LEN);
#ifdef _VERBOSE
  DW1000_Read(PMSC_ID, PMSC_CTRL0_OFFSET, (uint8_t *)&pmsc_ctrl0_val,
              PMSC_CTRL0_LEN);
  sprintf(str, "PMSC-CTRL0: 0x%08X (0x%08X)\r\n", (unsigned int)pmsc_ctrl0_val,
          (unsigned int)val32);
  UART_Print(VERBOSE_UART_ID, str);
  Delay(5);
#endif

  //-------------------------------------------------------------------------
  // CONFIGURE TRANSMIT CONTROL

  val32 = TX_FCTRL_TXPSR_PE_128 | TX_FCTRL_TXPRF_16M | TX_FCTRL_TXBR_6M;
  DW1000_Write(TX_FCTRL_ID, 0x00, &val32, 4u);
#ifdef _VERBOSE
  uint32_t tx_fctrl_val;
  DW1000_Read(TX_FCTRL_ID, 0x00, (uint8_t *)&tx_fctrl_val, 4u);
  sprintf(str, "TX-FCTRL: 0x%08X (0x%08X)\r\n", (unsigned int)tx_fctrl_val,
          (unsigned int)val32);
  UART_Print(VERBOSE_UART_ID, str);
  Delay(5);
#endif
}

// read all registers for debugging purposes
void DW1000_ReadAllRegisters(void) {
  UART_Print(VERBOSE_UART_ID, "\n----------------------------\r\n");
  UART_Print(VERBOSE_UART_ID, "READING ALL REGISTERS...\r\n");
  UART_Print(VERBOSE_UART_ID, "----------------------------\r\n");

  char str[100];

  uint32_t sys_cfg_val;
  DW1000_Read(SYS_CFG_ID, 0X00, (uint8_t *)&sys_cfg_val, SYS_CFG_LEN);
  sprintf(str, "SYS-CFG: 0x%08X\r\n", (unsigned int)sys_cfg_val);
  UART_Print(VERBOSE_UART_ID, str);
  Delay(5);

  uint32_t tx_fctrl_val;
  DW1000_Read(TX_FCTRL_ID, 0x00, (uint8_t *)&tx_fctrl_val, 4u);
  sprintf(str, "TX-FCTRL: 0x%08X\r\n", (unsigned int)tx_fctrl_val);
  UART_Print(VERBOSE_UART_ID, str);
  Delay(5);

  uint16_t rx_fwto_val;
  DW1000_Read(RX_FWTO_ID, 0x00, (uint8_t *)&rx_fwto_val, RX_FWTO_LEN);
  sprintf(str, "RX-FWTO: 0x%04X\r\n", (unsigned int)rx_fwto_val);
  UART_Print(VERBOSE_UART_ID, str);
  Delay(5);

  uint32_t tx_power_val;
  DW1000_Read(TX_POWER_ID, 0x00, (uint8_t *)&tx_power_val, TX_POWER_LEN);
  sprintf(str, "TX-POWER: 0x%08X\r\n", (unsigned int)tx_power_val);
  UART_Print(VERBOSE_UART_ID, str);
  Delay(5);

  uint32_t chan_ctrl_val;
  DW1000_Read(CHAN_CTRL_ID, 0x00, (uint8_t *)&chan_ctrl_val, CHAN_CTRL_LEN);
  sprintf(str, "CHAN-CTRL: 0x%08X\r\n", (unsigned int)chan_ctrl_val);
  UART_Print(VERBOSE_UART_ID, str);
  Delay(5);

  uint8_t usr_sfd_val;
  DW1000_Read(USR_SFD_ID, 0x00, &usr_sfd_val, USR_SFD_LEN);
  sprintf(str, "USR-SFD: 0x%02X\r\n", (unsigned int)usr_sfd_val);
  UART_Print(VERBOSE_UART_ID, str);
  Delay(5);

  uint16_t agc_tune1_val;
  DW1000_Read(AGC_CTRL_ID, AGC_TUNE1_OFFSET, (uint8_t *)&agc_tune1_val,
              AGC_TUNE1_LEN);
  sprintf(str, "AGC-TUNE1: 0x%04X\r\n", (unsigned int)agc_tune1_val);
  UART_Print(VERBOSE_UART_ID, str);
  Delay(5);

  uint32_t agc_tune2_val;
  DW1000_Read(AGC_CTRL_ID, AGC_TUNE2_OFFSET, (uint8_t *)&agc_tune2_val,
              AGC_TUNE2_LEN);
  sprintf(str, "AGC-TUNE2: 0x%08X\r\n", (unsigned int)agc_tune2_val);
  UART_Print(VERBOSE_UART_ID, str);
  Delay(5);

  uint16_t agc_tune3_val;
  DW1000_Read(AGC_CTRL_ID, AGC_TUNE3_OFFSET, (uint8_t *)&agc_tune3_val,
              AGC_TUNE3_LEN);
  sprintf(str, "AGC-TUNE3: 0x%04X\r\n", (unsigned int)agc_tune3_val);
  UART_Print(VERBOSE_UART_ID, str);
  Delay(5);

  uint16_t drx_tune0b_val;
  DW1000_Read(DRX_CONF_ID, DRX_TUNE0b_OFFSET, (uint8_t *)&drx_tune0b_val,
              DRX_TUNE0b_LEN);
  sprintf(str, "DRX-TUNE0b: 0x%04X\r\n", (unsigned int)drx_tune0b_val);
  UART_Print(VERBOSE_UART_ID, str);
  Delay(5);

  uint16_t drx_tune1a_val;
  DW1000_Read(DRX_CONF_ID, DRX_TUNE1a_OFFSET, (uint8_t *)&drx_tune1a_val,
              DRX_TUNE1a_LEN);
  sprintf(str, "DRX-TUNE1a: 0x%04X\r\n", (unsigned int)drx_tune1a_val);
  UART_Print(VERBOSE_UART_ID, str);
  Delay(5);

  uint16_t drx_tune1b_val;
  DW1000_Read(DRX_CONF_ID, DRX_TUNE1b_OFFSET, (uint8_t *)&drx_tune1b_val,
              DRX_TUNE1b_LEN);
  sprintf(str, "DRX-TUNE1b: 0x%04X\r\n", (unsigned int)drx_tune1b_val);
  UART_Print(VERBOSE_UART_ID, str);
  Delay(5);

  uint32_t drx_tune2_val;
  DW1000_Read(DRX_CONF_ID, DRX_TUNE2_OFFSET, (uint8_t *)&drx_tune2_val,
              DRX_TUNE2_LEN);
  sprintf(str, "DRX-TUNE2: 0x%08X\r\n", (unsigned int)drx_tune2_val);
  UART_Print(VERBOSE_UART_ID, str);
  Delay(5);

  uint16_t drx_sfdtoc_val;
  DW1000_Read(DRX_CONF_ID, DRX_SFDTOC_OFFSET, (uint8_t *)&drx_sfdtoc_val,
              DRX_SFDTOC_LEN);
  sprintf(str, "DRX-SFDTOC: 0x%04X\r\n", (unsigned int)drx_sfdtoc_val);
  UART_Print(VERBOSE_UART_ID, str);
  Delay(5);

  uint16_t drx_pretoc_val;
  DW1000_Read(DRX_CONF_ID, DRX_PRETOC_OFFSET, (uint8_t *)&drx_pretoc_val,
              DRX_PRETOC_LEN);
  sprintf(str, "DRX-PRETOC: 0x%04X\r\n", (unsigned int)drx_pretoc_val);
  UART_Print(VERBOSE_UART_ID, str);
  Delay(5);

  uint16_t drx_tune4h_val;
  DW1000_Read(DRX_CONF_ID, DRX_DRX_TUNE4H_OFFSET, (uint8_t *)&drx_tune4h_val,
              DRX_PRETOC_LEN);
  sprintf(str, "DRX-TUNE4H: 0x%04X\r\n", (unsigned int)drx_tune4h_val);
  UART_Print(VERBOSE_UART_ID, str);
  Delay(5);

  uint8_t rf_rxctrlh_val;
  DW1000_Read(RF_CONF_ID, RF_RXCTRLH_OFFSET, &rf_rxctrlh_val, RF_TXCTRL_LEN);
  sprintf(str, "RF-RXCTRLH: 0x%02X\r\n", (unsigned int)rf_rxctrlh_val);
  UART_Print(VERBOSE_UART_ID, str);
  Delay(5);

  uint32_t rf_txctrl_val;
  DW1000_Read(RF_CONF_ID, RF_TXCTRL_OFFSET, (uint8_t *)&rf_txctrl_val,
              RF_TXCTRL_LEN);
  sprintf(str, "RF-TXCTRL: 0x%08X\r\n", (unsigned int)rf_txctrl_val);
  UART_Print(VERBOSE_UART_ID, str);
  Delay(5);

  uint8_t tc_pgdelay_val;
  DW1000_Read(TX_CAL_ID, TC_PGDELAY_OFFSET, &tc_pgdelay_val, TC_PGDELAY_LEN);
  sprintf(str, "TC-PGDELAY: 0x%02X\r\n", (unsigned int)tc_pgdelay_val);
  UART_Print(VERBOSE_UART_ID, str);
  Delay(5);

  uint32_t fs_pllcfg_val;
  DW1000_Read(FS_CTRL_ID, FS_PLLCFG_OFFSET, (uint8_t *)&fs_pllcfg_val,
              FS_PLLCFG_LEN);
  sprintf(str, "FS-PLLCFG: 0x%08X\r\n", (unsigned int)fs_pllcfg_val);
  UART_Print(VERBOSE_UART_ID, str);
  Delay(5);

  uint8_t fs_plltune_val;
  DW1000_Read(FS_CTRL_ID, FS_PLLTUNE_OFFSET, &fs_plltune_val, FS_PLLTUNE_LEN);
  sprintf(str, "FS-PLLTUNE: 0x%02X\r\n", (unsigned int)fs_plltune_val);
  UART_Print(VERBOSE_UART_ID, str);
  Delay(5);

  uint8_t lde_cfg1_val;
  DW1000_Read(LDE_IF_ID, LDE_CFG1_OFFSET, &lde_cfg1_val, LDE_CFG1_LEN);
  sprintf(str, "LDE-CFG1: 0x%02X\r\n", (unsigned int)lde_cfg1_val);
  UART_Print(VERBOSE_UART_ID, str);
  Delay(5);

  uint16_t lde_cfg2_val;
  DW1000_Read(LDE_IF_ID, LDE_CFG2_OFFSET, (uint8_t *)&lde_cfg2_val,
              LDE_CFG2_LEN);
  sprintf(str, "LDE-CFG2: 0x%04X\r\n", (unsigned int)lde_cfg2_val);
  UART_Print(VERBOSE_UART_ID, str);
  Delay(5);

  uint16_t lde_repc_val;
  DW1000_Read(LDE_IF_ID, LDE_REPC_OFFSET, (uint8_t *)&lde_repc_val,
              LDE_REPC_LEN);
  sprintf(str, "LDE-REPC: 0x%04X\r\n", (unsigned int)lde_repc_val);
  UART_Print(VERBOSE_UART_ID, str);
  Delay(5);

  uint32_t pmsc_ctrl0_val;
  DW1000_Read(PMSC_ID, PMSC_CTRL0_OFFSET, (uint8_t *)&pmsc_ctrl0_val,
              PMSC_CTRL0_LEN);
  sprintf(str, "PMSC-CTRL0: 0x%08X\r\n", (unsigned int)pmsc_ctrl0_val);
  UART_Print(VERBOSE_UART_ID, str);
  Delay(5);

  uint32_t sys_mask_val;
  DW1000_Read(SYS_MASK_ID, 0x00, (uint8_t *)&sys_mask_val, SYS_MASK_LEN);
  sprintf(str, "SYS-MASK: 0x%08X\r\n", (unsigned int)sys_mask_val);
  UART_Print(VERBOSE_UART_ID, str);
  Delay(5);

  uint32_t gpio_mode;
  DW1000_Read(GPIO_CTRL_ID, GPIO_MODE_OFFSET, (uint8_t *)&gpio_mode,
              GPIO_MODE_LEN);
  sprintf(str, "GPIO-MODE: 0x%08X\r\n", (unsigned int)gpio_mode);
  UART_Print(VERBOSE_UART_ID, str);
  Delay(5);

  uint32_t pmsc_ledc;
  DW1000_Read(PMSC_ID, PMSC_LEDC_OFFSET, (uint8_t *)&pmsc_ledc, PMSC_LEDC_LEN);
  sprintf(str, "PMSC-LEDC: 0x%08X\r\n", (unsigned int)pmsc_ledc);
  UART_Print(VERBOSE_UART_ID, str);
  Delay(5);
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

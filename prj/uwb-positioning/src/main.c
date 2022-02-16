/**
 * @file    main.c
 * @author  Raymond Oung
 * @date    2014.08.11
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
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "stm32f4xx.h"

#include "can.h"
#include "clock.h"
#include "dip.h"
#include "i2c.h"
#include "led.h"
#include "main.h"
#include "nvic_list.h"
#include "spi.h"
#include "tim.h"
#include "uart.h"

#include "deca_regs.h"
#include "dw1000.h"
#include "m24aa02uid.h"
#include "mpu9150.h"
#include "range.h"
#include "tinysync.h"
#include "trilateration.h"

/** @addtogroup Main
 * @{
 */

/* Private defines
 * ------------------------------------------------------------*/
// #define RANGING_DEMO

#define VERBOSE_UART_ID 4u
#define START_BYTE '@'
#define F_CUTOFF 20.0f // [Hz]

/* Private types -------------------------------------------------------------*/
typedef enum _node_t { NODE_STATIC = 0u, NODE_MOBILE = 1u } node_t;

typedef struct _UWB_static_node_info_t {
  uint32_t uid;   // static node unique identifier
  uint16_t range; // range [m x1000]
  uint8_t snr;    // signal to noise ratio [dBm x10] (0-25)
  uint8_t pwr_fp; // first pass signal power [dBm] (0-200)
  uint8_t pwr_rx; // receiver signal power [dBm] (0-200)
} __attribute__((__packed__)) UWB_static_node_info_t;

typedef struct _UWB_telemetry_msg_t {
  uint8_t start;              // start byte '@'
  uint32_t timestamp;         // [msec]
  uint8_t id;                 // mobile node ID
  float x;                    // position x [m]
  float y;                    // position y [m]
  float z;                    // position z [m]
  uint8_t num_selected_nodes; // number of static nodes
  UWB_static_node_info_t nodes[16];
  uint16_t chksum; // fletcher checksum
} __attribute__((__packed__)) UWB_telemetry_msg_t;

typedef struct _UWB_node_t {
  uint32_t uid;        // node ID
  node_t type;         // NODE_STATIC or NODE_MOBILE
  uint32_t tkn_dst_id; // destination ID for token (mobile node only)
  uint8_t tkn_holder;  // declare token holder (mobile node only)
  float bias;          // range bias [m]
  float x;             // position x [m]
  float y;             // position y [m]
  float z;             // position z [m]
} UWB_node_t;

/* Private function prototypes -----------------------------------------------*/
float UTIL_LPF1(float y, float x, float alpha);
float UTIL_LPF1_GetAlpha(float f_cutoff, float f_sample);
void RCC_Configuration(void);
void GPIO_Configuration(void);
void NVIC_Configuration(void);

// append 16-bit fletcher checksum to the end of data
void UTIL_SetFletcher16(uint8_t *data, uint16_t size) {
  uint16_t sum1 = 0u;
  uint16_t sum2 = 0u;

  uint8_t i;
  for (i = 0u; i < (size - 2u); i++) { // excludes checksum bytes
    sum1 = (sum1 + data[i]) % 0xFF;
    sum2 = (sum2 + sum1) % 0xFF;
  }

  data[size - 2u] = 0xFF - ((sum1 + sum2) % 0xFF);
  data[size - 1u] = 0xFF - ((sum1 + data[size - 2u]) % 0xFF);
}

// check 16-bit fletcher checksum at the end of data
uint8_t UTIL_ChkFletcher16(uint8_t *data, uint16_t size) {
  uint16_t sum1 = 0u;
  uint16_t sum2 = 0u;

  uint8_t i;
  for (i = 0u; i < size; i++) {
    sum1 = (sum1 + data[i]) % 0xFF;
    sum2 = (sum2 + sum1) % 0xFF;
  }

  return ((sum2 << 8u) | sum1) == 0u;
}

/**
 * @brief Compute running mean
 * @param[in] mean_prev running mean at t-1
 * @param[in] x new sample to be included in the mean at t
 * @param[in] n total samples thus far at t
 * @retval Running mean.
 */
float UTIL_GetMean(float mean, float x, uint32_t n) {
  return mean + (x - mean) / n;
}

/**
 * @brief Compute running average
 * @param y old and new filtered data
 * @param x new sample to be filtered
 * @param alpha smoothing factor, i.e. (sample_time)/(RC + sample_time)
 * @retval 1st Order Low-Pass filtered result.
 */
float UTIL_LPF1(float y, float x, float alpha) { return y + alpha * (x - y); }

/**
 * @brief Compute alpha smoothing factor, i.e. (sample_time)/(RC + sample_time)
 * @param f_cutoff cut-off frequency [Hz]
 * @param f_sample sampling frequency [Hz]
 * @retval 1st Order Low-Pass smoothing factor, alpha.
 */
float UTIL_LPF1_GetAlpha(float f_cutoff, float f_sample) {
  float t_sample = 1.0f / f_sample;               // [sec]
  float rc = (float)(1.0f / (M_2_PI * f_cutoff)); // [rad/s]
  return t_sample / (rc + t_sample);
}

void UTIL_Output2Console(uint8_t num_selected_nodes, uint8_t num_total_nodes,
                         UWB_node_t *static_nodes, float *range) {
  uint8_t i;

  // if (!isnan(TRILAT_GetX() && !isnan(TRILAT_GetY()) && !isnan(TRILAT_GetZ())
  // && 	TRILAT_GetX() > 0.0f && TRILAT_GetY() > 0.0f))
  {
    static uint32_t t0 = 0u;
    if (GetTime() - t0 > 1000.0f / F_CUTOFF) {
      t0 = GetTime();

      char str[100];
      sprintf(str, "%u ", (unsigned int)GetTime());
      UART_Print(VERBOSE_UART_ID, str);
      // sprintf(str,"%.3f %.3f %.3f ", TRILAT_GetX(), TRILAT_GetY(),
      // TRILAT_GetZ()); UART_Print(VERBOSE_UART_ID,str);
      sprintf(str, "%.3f %.3f %.3f ", TRILAT_GetX(), TRILAT_GetY(), 0.0f);
      UART_Print(VERBOSE_UART_ID, str);
      for (i = 0; i < num_total_nodes; i++) {
        sprintf(str, "%.3f ", range[i]);
        UART_Print(VERBOSE_UART_ID, str); // FILTERED RANGE
      }

      for (i = 0; i < num_total_nodes; i++) {
        sprintf(str, "%u ",
                (uint8_t)roundf(10 * NODE_GetSNR(static_nodes[i].uid)));
        UART_Print(VERBOSE_UART_ID, str); // SNR
      }

      for (i = 0; i < num_total_nodes; i++) {
        sprintf(str, "%u ",
                (uint8_t)roundf(-NODE_GetPwrFP(static_nodes[i].uid)));
        UART_Print(VERBOSE_UART_ID, str); // Power FP
      }

      for (i = 0; i < num_total_nodes; i++) {
        sprintf(str, "%u ",
                (uint8_t)roundf(-NODE_GetPwrRX(static_nodes[i].uid)));
        UART_Print(VERBOSE_UART_ID, str); // Power RX
      }

      sprintf(str, "%u ", num_selected_nodes);
      UART_Print(VERBOSE_UART_ID, str); // number of static nodes

      UART_Print(VERBOSE_UART_ID, "\r\n");
    }
  }
}

void UTIL_OutputTelemetry(uint8_t num_selected_nodes, UWB_node_t mobile_node,
                          UWB_node_t *static_nodes, float *range) {
  uint8_t i;
  UWB_telemetry_msg_t msg;

  if (!isnan(TRILAT_GetX()) && !isnan(TRILAT_GetY()) && !isnan(TRILAT_GetZ()) &&
      TRILAT_GetX() > 0.0f && TRILAT_GetY() > 0.0f) {
    static uint32_t t0_telemetry = 0u;
    if (GetTime() - t0_telemetry > 1000.0f / F_CUTOFF) {
      t0_telemetry = GetTime();

      msg.start = START_BYTE;
      msg.timestamp = GetTime();                   // [msec]
      msg.id = mobile_node.uid;                    // mobile node ID
      msg.x = TRILAT_GetX();                       // position x [m]
      msg.y = TRILAT_GetY();                       // position y [m]
      msg.z = 1.5f;                                // position z [m]
      msg.num_selected_nodes = num_selected_nodes; // number of static nodes
      for (i = 0; i < num_selected_nodes; i++) {
        msg.nodes[i].uid = static_nodes[i].uid; // static node unique identifier
        msg.nodes[i].range = range[i];          // range [m x1000]
        msg.nodes[i].snr = (uint8_t)roundf(
            10 * NODE_GetSNR(
                     static_nodes[i].uid)); // signal to noise ratio [dBm x10]
        msg.nodes[i].pwr_fp = (uint8_t)roundf(-NODE_GetPwrFP(
            static_nodes[i].uid)); // first pass signal power [dBm]
        msg.nodes[i].pwr_rx = (uint8_t)roundf(
            -NODE_GetPwrRX(static_nodes[i].uid)); // receiver signal power [dBm]
      }
      UTIL_SetFletcher16((uint8_t *)&msg,
                         sizeof(UWB_telemetry_msg_t)); // checksum
      UART_Write_DMA(VERBOSE_UART_ID, &msg, sizeof(msg));
    }
  }
}

/**
 * @brief  Main program
 * @param  None
 * @retval None
 */
int main(void) {
  //========================================================================
  // DECLARE STATIC NODES
  //========================================================================
	UWB_node_t node00;
	node00.uid = 0x00;
	node00.type = NODE_STATIC;
	node00.x = 0.0f;
	node00.y = 0.0f;
	node00.z = 0.0f;

	UWB_node_t node01;
	node01.uid = 0x01;
	node01.type = NODE_STATIC;
	node01.x = 0.8f;
	node01.y = 0.0f;
	node01.z = 0.00f;

	UWB_node_t node02;
	node02.uid = 0x02;
	node02.type = NODE_STATIC;
	node02.x = 3.10f;
	node02.y = 0.85f;
	node02.z = 0.0f;

	UWB_node_t node03;
	node03.uid = 0x03;
	node03.type = NODE_STATIC;
	node03.x = 0.0f;
	node03.y = 0.85f;
	node03.z = 0.0f;

	UWB_node_t node04;
	node04.uid = 0x04;
	node04.type = NODE_STATIC;
	node04.x = 0.0f;
	node04.y = 0.0f;
	node04.z = 0.0f;

	UWB_node_t node05;
	node05.uid = 0x05;
	node05.type = NODE_STATIC;
	node05.x = 0.8f;
	node05.y = 0.0f;
	node05.z = 0.0f;

	UWB_node_t node06;
	node06.uid = 0x06;
	node06.type = NODE_STATIC;
	node06.x = 0.8f;
	node06.y = 0.8f;
	node06.z = 0.0f;

	UWB_node_t node07;
	node07.uid = 0x07;
	node07.type = NODE_STATIC;
	node07.x = 0.0f;
	node07.y = 0.8f;
	node07.z = 0.0f;

	UWB_node_t node08;
	node08.uid = 0x08;
	node08.type = NODE_STATIC;
	node08.x = 3.10f;
	node08.y = 0.0f;
	node08.z = 0.0f;

	UWB_node_t node09;
	node09.uid = 0x09;
	node09.type = NODE_STATIC;
	node09.x = 0.8f;
	node09.y = 0.0f;
	node09.z = 0.0f;

	UWB_node_t node0A;
	node0A.uid = 0x0A;
	node0A.type = NODE_STATIC;
	node0A.x = 0.8f;
	node0A.y = 0.8f;
	node0A.z = 0.0f;

	UWB_node_t node0B;
	node0B.uid = 0x0B;
	node0B.type = NODE_STATIC;
	node0B.x = 0.0f;
	node0B.y = 0.8f;
	node0B.z = 0.0f;

  //========================================================================
  // DECLARE MOBILE NODES
  //========================================================================
  UWB_node_t nodeF0;
  nodeF0.uid = 0xF0;
  nodeF0.type = NODE_MOBILE;
  nodeF0.tkn_dst_id = nodeF0.uid;
  nodeF0.tkn_holder = 1u;

  UWB_node_t nodeF1;
  nodeF1.uid = 0xF1;
  nodeF1.type = NODE_MOBILE;
  nodeF1.tkn_dst_id = nodeF1.uid;
  nodeF1.tkn_holder = 0u;

  // define static nodes to be used
#ifdef RANGING_DEMO // ranging demo
  UWB_node_t all_nodes[] = {node00, node01, node02, node03, node04,
                            node05, node06, node07, node08, node09,
                            node0A, nodeF0, nodeF1};
  UWB_node_t static_nodes[] = {node00, node01, node02, node03, node04, node05,
                               node06, node07, node08, node09, node0A};
#else // positioning demo
  UWB_node_t all_nodes[] = {node00, node01, node02, node03, node04, node05,
                            node06, node07, node08, node09, nodeF0, nodeF1};
  UWB_node_t static_nodes[] = {node00, node01, node02, node03, node04,
                               node05, node06, node07, node08, node09};
#endif
  uint8_t NUM_ALL_NODES = sizeof(all_nodes) / sizeof(UWB_node_t);
  uint8_t NUM_STATIC_NODES = sizeof(static_nodes) / sizeof(UWB_node_t);

  // configure low-pass filter
  float lpf_alpha = UTIL_LPF1_GetAlpha(
      F_CUTOFF, 1000.0f / NUM_STATIC_NODES); // use alpha=0 to remove filter
  float f_range[TRILAT_MAX_STATIC_NODES];
  uint8_t i;
  for (i = 0u; i < TRILAT_MAX_STATIC_NODES; i++) {
    f_range[i] = 0.0f; // reset filtered range values
  }

  //========================================================================
  // PERIPHERAL INITIALIZATION
  //========================================================================
  RCC_Configuration();
  GPIO_Configuration();
  NVIC_Configuration();

  CLOCK_Init();
  LED_Init(LED1_CLK, LED1_PORT, LED1_PIN);
  LED_OnAll();
  Delay(500);
  LED_OffAll();

  DIP_Init(ADDR0_CLK, ADDR0_PORT, ADDR0_PIN);
  DIP_Init(ADDR1_CLK, ADDR1_PORT, ADDR1_PIN);
  DIP_Init(ADDR2_CLK, ADDR2_PORT, ADDR2_PIN);

  UART_PeriphInit(UART4, UART4_BAUD, UART4_DMA_BUF_SIZE, UART4_TX_CLK,
                  UART4_TX_PORT, UART4_TX_PIN, UART4_TX_PIN_SRC, UART4_RX_CLK,
                  UART4_RX_PORT, UART4_RX_PIN, UART4_RX_PIN_SRC,
                  UART4_DMA_RX_NVIC_PREEMPTION_PRIORITY,
                  UART4_DMA_TX_NVIC_PREEMPTION_PRIORITY,
                  UART4_IT_RX_NVIC_PREEMPTION_PRIORITY,
                  UART4_IT_TX_NVIC_PREEMPTION_PRIORITY);

  SPI_PeriphInit(SPI1, SPI_BAUD_SLOW, SPI_CPOL0, SPI_CPHA0, SPI1_MOSI_CLK,
                 SPI1_MOSI_PORT, SPI1_MOSI_PIN, SPI1_MOSI_PIN_SRC,
                 SPI1_MISO_CLK, SPI1_MISO_PORT, SPI1_MISO_PIN,
                 SPI1_MISO_PIN_SRC, SPI1_SCLK_CLK, SPI1_SCLK_PORT,
                 SPI1_SCLK_PIN, SPI1_SCLK_PIN_SRC);

  CAN_PeriphInit(CAN1, CAN1_FILT_NUM, CAN1_GPIO_CLK, CAN1_CLK, CAN1_TX_PORT,
                 CAN1_TX_PIN, CAN1_TX_PIN_SOURCE, CAN1_RX_PORT, CAN1_RX_PIN,
                 CAN1_RX_PIN_SOURCE, CAN1_RX_IRQn,
                 CAN1_RX_NVIC_PREEMPTION_PRIORITY, CAN1_TX_IRQn,
                 CAN1_TX_NVIC_PREEMPTION_PRIORITY);

  DW1000_Init(DW1000_SPI, DW1000_EN_CLK, DW1000_EN_PORT, DW1000_EN_PIN,
              DW1000_CS_CLK, DW1000_CS_PORT, DW1000_CS_PIN, DW1000_WAKE_CLK,
              DW1000_WAKE_PORT, DW1000_WAKE_PIN, DW1000_NRST_CLK,
              DW1000_NRST_PORT, DW1000_NRST_PIN, DW1000_IRQ_CLK,
              DW1000_IRQ_PORT, DW1000_IRQ_PIN, DW1000_IRQ_PORT_SRC,
              DW1000_IRQ_PIN_SRC, DW1000_IRQ_LINE, DW1000_IRQ_IRQ,
              DW1000_IRQ_NVIC_PREEMPTION_PRIORITY);

  // DIP_SetVal(node0A.uid);
  DIP_SetVal(nodeF0.uid); // MANUALLY SET NODE ID HERE!!!

  //========================================================================
  // DW1000 UWB CONFIGURATION
  //========================================================================
  // Note:
  // Channel depends on legal authorities; Channel and Data Rate affect distance
  // and bandwidth PRF affects robustess and range, Section 9.3 Preamble code
  // depends on Channel and PRF, Section 10.5 Preamble length depends on Data
  // Rate, Section 9.3

  // @TODO 110K and 850K data rates don't work properly!!!
  // <<<<<<<<<<<<<<<<<<<<<<<<< Japan: Ch-1 to Ch-4 at -41.3 dBm/MHz with DAA/LDC
  // >>DW1000_Config(CH1, DATA_RATE_110, PRF_64MHZ, PREAMBLE_CODE_9,
  // PREAMBLE_LENGTH_4096); // long range, low node density, low resolution
  // >>DW1000_Config(CH1, DATA_RATE_850, PRF_64MHZ, PREAMBLE_CODE_9,
  // PREAMBLE_LENGTH_1024); // short range, high node density, high resolution
  // DW1000_Config(CH1, DATA_RATE_6800, PRF_64MHZ, PREAMBLE_CODE_9,
  // PREAMBLE_LENGTH_256); // short range, high node density, high resolution
  // DW1000_Config(CH2, DATA_RATE_110, PRF_64MHZ, PREAMBLE_CODE_9,
  // PREAMBLE_LENGTH_4096); // long range, low node density, low resolution
  // DW1000_Config(CH2, DATA_RATE_6800, PRF_64MHZ, PREAMBLE_CODE_9,
  // PREAMBLE_LENGTH_256); // short range, high node density, high resolution
  // DW1000_Config(CH3, DATA_RATE_110, PRF_64MHZ, PREAMBLE_CODE_9,
  // PREAMBLE_LENGTH_4096); // long range, low node density, low resolution
  // DW1000_Config(CH3, DATA_RATE_6800, PRF_64MHZ, PREAMBLE_CODE_9,
  // PREAMBLE_LENGTH_256); // short range, high node density, high resolution
  // DW1000_Config(CH4, DATA_RATE_110, PRF_64MHZ, PREAMBLE_CODE_17,
  // PREAMBLE_LENGTH_4096); // long range, low node density, low resolution
  // DW1000_Config(CH4, DATA_RATE_6800, PRF_64MHZ, PREAMBLE_CODE_17,
  // PREAMBLE_LENGTH_256); // short range, high node density, high resolution

  // Europe: Ch-5/-7 at -41.3 dBm/MHz
  // DW1000_Config(CH5, DATA_RATE_110, PRF_64MHZ, PREAMBLE_CODE_9,
  // PREAMBLE_LENGTH_4096); // long range, low node density, low resolution
  // DW1000_Config(CH5, DATA_RATE_6800, PRF_64MHZ, PREAMBLE_CODE_9,
  // PREAMBLE_LENGTH_256); // short range, high node density, high resolution
  // DW1000_Config(CH7, DATA_RATE_110, PRF_64MHZ, PREAMBLE_CODE_17,
  // PREAMBLE_LENGTH_4096); // long range, low node density, low resolution
  // DW1000_Config(CH7, DATA_RATE_6800, PRF_64MHZ, PREAMBLE_CODE_17,
  // PREAMBLE_LENGTH_256); // short range, high node density, high resolution

  // DecaWave defaults configuration
  // DW1000_Config(CH5, DATA_RATE_6800, PRF_16MHZ, PREAMBLE_CODE_4,
  // PREAMBLE_LENGTH_128); // short range, high node density, high resolution
  DW1000_ConfigDefault(); // DEBUG @TODO From preliminary tests, this provides
                          // the best resolution in comparison to the mode above
// DW1000_ReadAllRegisters();

// DEBUG
#if 0
	float background_energy = DW1000_ReadBackgroundEnergy(CH5);
	char str[100]; sprintf(str, "Background Energy: %.3f\r\n", background_energy); UART_Print(VERBOSE_UART_ID,str);
	while(1);
#endif

  // initialise node to receiver or transmitter
  // initialise token holder
  volatile uint8_t first2wait;
  static volatile UWB_node_t this_node;
  uint8_t k;
  for (k = 0u; k < NUM_ALL_NODES; k++) {
    // step through each node
    if (all_nodes[k].uid == DIP_GetVal()) {
      this_node = all_nodes[k];
      switch (this_node.type) {
      case NODE_STATIC: {
        DW1000_RxTimeout(DISABLE);
        DW1000_ReceiverOn();
      } break;

      case NODE_MOBILE: {
        DW1000_RxTimeout(ENABLE);

        if (!this_node.tkn_holder) {
          first2wait = 1u;
          RANGE_ClrToken();
        } else {
          first2wait = 0u;
          RANGE_SetToken();
        }
      } break;
      }
    }
  }

  // upon completing initialisation, increase baud-rate
  SPI_SetBaudPrescaler(1u, SPI_BAUD_FAST);

  //========================================================================
  // RANGE INITIALIZATION
  //========================================================================
  RANGE_Init(this_node.uid);
  switch (this_node.type) {
  case NODE_MOBILE: {
    for (i = 0u; i < NUM_STATIC_NODES; i++) {
      RANGE_AddStaticNode(static_nodes[i].uid);
    }
    RANGE_UpdateStaticNodes();
  } break;

  case NODE_STATIC: {
    while (1) {
      Delay(5000);
      LED_On(0);
      Delay(100);
      LED_Off(0);
    }
  } break;
  }

  //========================================================================
  // TRILATERATION INITIALIZATION
  //========================================================================
  if (this_node.type == NODE_MOBILE) {
    for (i = 0u; i < NUM_STATIC_NODES; i++) {
      TRILAT_AddStaticNode(static_nodes[i].x, static_nodes[i].y,
                           static_nodes[i].z);
    }
    TRILAT_UpdateStaticNodes();
  }

//========================================================================
// MAIN LOOP
//========================================================================
// @TODO: Static nodes should come on periodically to check for mobile nodes
// once a mobile node is detected, static node should stay on until timeout,
// i.e. turn off when mobile nodes leave the vicinity
// Mobile nodes should turn on only when transmitting
#define PROBE_TIMEOUT 3u // [msec]
#define TOKEN_TIMEOUT                                                          \
  PROBE_TIMEOUT *NUM_STATIC_NODES +                                            \
      1u // must be larger than the time required to send all probes [msec]
  while (1) {
    static uint8_t id = 0u;

    // check that static node-ID is valid
    if (id < NUM_STATIC_NODES) {
      uint32_t t1 = GetTime();
      while (!NODE_GetTxReady()) {
        if (GetTime() - t1 > PROBE_TIMEOUT) {
          break;
        }
      }

      // send ranging probe and adjust token destination ID
      if (id == NUM_STATIC_NODES - 1u) {
        // if this is the last static node in the set, send the token
        // destination identifier
        RANGE_SendProbe(static_nodes[id].uid, this_node.tkn_dst_id);
      } else {
        // if this is not the last static node in the set, don't send the token
        // destination identifier
        RANGE_SendProbe(static_nodes[id].uid, this_node.uid);
      }

      // run watchdog ISR
      NODE_WatchdogISR(static_nodes[id].uid);

      // increment id
      id++;
    } else // after probing all nodes, run trilateration, etc.
    {

#ifdef RANGING_DEMO // ranging demo


      if (GetTime() > 15000u) // wait for warmup
      {
        static uint8_t first = 1u;
        static float t0_bias = 0.0f; // [msec]
        static uint32_t cnt[16u];

        // estimate bias error
        if (first) {
          first = 0u;
          for (i = 0u; i < NUM_STATIC_NODES; i++) {
            static_nodes[i].bias = 0.0f; // clear bias
            f_range[i] = 1.0f;           // set initial condition [m]
            cnt[i] = 1u;
          }
          lpf_alpha = UTIL_LPF1_GetAlpha(20.0f, 1000.0f / NUM_STATIC_NODES);

          t0_bias = GetTime(); // [msec]
        }

        if (GetTime() - t0_bias < 1000u) {
          for (i = 0u; i < NUM_STATIC_NODES; i++) {
            float val = NODE_GetRange(static_nodes[i].uid) - 154.0f;
            if (fabs(val) < 10.0f) {
              static_nodes[i].bias =
                  UTIL_GetMean(static_nodes[i].bias,
                               NODE_GetRange(static_nodes[i].uid), cnt[i]++);
              // char str[100]; sprintf(str, "%u: val: %.3f, bias: %.3f, range:
              // %.3f, cnt: %u\r\n", i, val, static_nodes[10].bias,
              // NODE_GetRange(static_nodes[10].uid), cnt[10]);
              // UART_Print(VERBOSE_UART_ID,str);
            }
          }
        } else {
          LED_On(0);
          for (i = 0u; i < NUM_STATIC_NODES; i++) {
            float val = NODE_GetRange(static_nodes[i].uid) -
                        static_nodes[i].bias + 1.0f; // [m]
            if (val > 0.0f && val < 10.0f) {
              f_range[i] = UTIL_LPF1(f_range[i], val, lpf_alpha); // filtered
            }
          }
          char str[100];
          sprintf(str, "%.1f cm            \r", 100.0f * f_range[10]);
          UART_Print(VERBOSE_UART_ID, str);
        }
      } else {
        if (GetTime() > 14000u) {
          UART_Print(VERBOSE_UART_ID, "Initializing...done\r");
        } else if (GetTime() > 13000u) {
          UART_Print(VERBOSE_UART_ID, "Initializing...    \r");
        } else if (GetTime() > 12000u) {
          UART_Print(VERBOSE_UART_ID, "Initializing..     \r");
        } else if (GetTime() > 11000u) {
          UART_Print(VERBOSE_UART_ID, "Initializing.      \r");
        } else if (GetTime() > 10000u) {
          UART_Print(VERBOSE_UART_ID, "Initializing       \r");
        } else if (GetTime() > 9000u) {
          UART_Print(VERBOSE_UART_ID, "Initializing...    \r");
        } else if (GetTime() > 8000u) {
          UART_Print(VERBOSE_UART_ID, "Initializing..     \r");
        } else if (GetTime() > 7000u) {
          UART_Print(VERBOSE_UART_ID, "Initializing.      \r");
        } else if (GetTime() > 6000u) {
          UART_Print(VERBOSE_UART_ID, "Initializing       \r");
        } else if (GetTime() > 5000u) {
          UART_Print(VERBOSE_UART_ID, "Initializing...    \r");
        } else if (GetTime() > 4000u) {
          UART_Print(VERBOSE_UART_ID, "Initializing..     \r");
        } else if (GetTime() > 3000u) {
          UART_Print(VERBOSE_UART_ID, "Initializing.      \r");
        } else if (GetTime() > 2000u) {
          UART_Print(VERBOSE_UART_ID, "Initializing       \r");
        }
      }

#else // positioning demo
      #define NODE_THRESHOLD 6u

      // run trilateration when range to a sufficient number of static nodes is
      // known
      uint8_t num_selected_nodes = 0u;
      float s_range[NUM_STATIC_NODES]; // selected valid range values
      for (i = 0u; i < NUM_STATIC_NODES; i++) {
        s_range[i] = 0.0f; // clear buffer

        // float val = NODE_GetRange(static_nodes[i].uid); // [m] USE WHEN
        // CALIBRATING
        float val =
            NODE_GetRange(static_nodes[i].uid) - static_nodes[i].bias; // [m]
        if (val > 0.0f && val < 10.0f) // @TODO sometimes range is unusually
                                       // large, so filter it out
        {
          f_range[i] = UTIL_LPF1(f_range[i], val, lpf_alpha); // filtered
          s_range[num_selected_nodes++] = f_range[i];
          TRILAT_AddStaticNode(static_nodes[i].x, static_nodes[i].y,
                               static_nodes[i].z);
        }
      }

      if (TRILAT_UpdateStaticNodes() == 1u) {
        TRILAT_UpdateMobileNode(s_range);

        if (!isnan(TRILAT_GetX()) && !isnan(TRILAT_GetY()) &&
            !isnan(TRILAT_GetZ())) {
          LED_On(0);

          // run trilateration when range to a sufficient number of static nodes
          // is known
          uint8_t num_selected_nodes = 0u;
          float s_range[NUM_STATIC_NODES]; // selected valid range values
          for (i = 0u; i < NUM_STATIC_NODES; i++) {
            s_range[i] = 0.0f; // clear buffer

            // now trilaterate only on selected nodes
            if (static_nodes[i].uid < NODE_THRESHOLD) {
              // float val = NODE_GetRange(static_nodes[i].uid); // [m] USE WHEN
              // CALIBRATING
              float val = NODE_GetRange(static_nodes[i].uid) -
                          static_nodes[i].bias; // [m]
              if (val > 0.0f && val < 8.0f)     // @TODO sometimes range is
                                            // unusually large, so filter it out
              {
                f_range[i] = UTIL_LPF1(f_range[i], val, lpf_alpha); // filtered
                s_range[num_selected_nodes++] = f_range[i];
                TRILAT_AddStaticNode(static_nodes[i].x, static_nodes[i].y,
                                     static_nodes[i].z);
              }
            }
          }

          if (TRILAT_UpdateStaticNodes() == 1u) {
            TRILAT_UpdateMobileNode(s_range);
            UTIL_OutputTelemetry(num_selected_nodes, this_node, static_nodes,
                                 s_range);
          }
        } else {
          LED_Off(0);
        }
      }

      //----------------------------------------------------------------------------------------------------------
      // MULTIPLE MOBILE NODES: WAIT FOR TOKEN
      //----------------------------------------------------------------------------------------------------------
      if (this_node.uid != this_node.tkn_dst_id) {
        // turn transceiver off and listen for token
        // DW1000_TransceiverOff();
        DW1000_RxTimeout(DISABLE);
        DW1000_ReceiverOn();

        if (first2wait) // this is only used when turning on the nodes for the
                        // first time
        {
          first2wait = 0u;

          // wait until we get the token
          while (!RANGE_GetToken())
            ;
        } else {
          // wait until we get the token or until timeout
          uint32_t to = GetTime();
          while (!RANGE_GetToken()) {
            if (GetTime() - to > TOKEN_TIMEOUT) {
              break;
            }
          }
        }
        DW1000_RxTimeout(ENABLE);
      }
#endif
      id = 0u; // reset node
    }
  }
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

  // Infinite loop
  while (1)
    ;
}
#endif

/**
 * @}
 */

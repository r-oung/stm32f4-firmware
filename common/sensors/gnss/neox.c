/**
 * @file    neox.c
 * @author  Raymond Oung
 * @date    2013.10.29
 * @brief   NEOx global navigation satellite system (GNSS) functions
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
#include "neox.h"
#include "clock.h"
#include "uart.h"
#include "ubx.h"

#ifdef _VERBOSE
#include <stdio.h>
#include <string.h>
#endif

/** @addtogroup Source
 * @{
 */

/** @addtogroup Utilities
 * @{
 */

/** @defgroup NEOx
 * @brief Functions that handle the NEOx global navigation satellite system
 * module
 * @{
 */

/** @defgroup NEOx_Private_Defines
 * @{
 */
#ifdef _VERBOSE
#define VERBOSE_UART_ID 1u
#endif

#define NEOx_BAUD 115200u
#define NEOx_ACK_TIMEOUT 100u // timeout [msec]
/**
 * @}
 */

/** @defgroup NEOx_Private_Variables
 * @{
 */
static volatile struct _NEOx_ecefP_t {
  uint32_t timestamp; // [msec]
  uint32_t iTOW;      // GPS time of the week of the navigation epoch [msec]
  int32_t ecefX;      // ECEF X coordinate [cm]
  int32_t ecefY;      // ECEF Y coordinate [cm]
  int32_t ecefZ;      // ECEF Z coordinate [cm]
  uint32_t pAcc;      // position accuracy estimate [cm]
} NEOx_ecefP;

static volatile struct _NEOx_ecefV_t {
  uint32_t timestamp; // [msec]
  uint32_t iTOW;      // GPS time of the week of the navigation epoch [msec]
  int32_t ecefVX;     // ECEF X velocity [cm/s]
  int32_t ecefVY;     // ECEF Y velocity [cm/s]
  int32_t ecefVZ;     // ECEF Z velocity [cm/s]
  uint32_t sAcc;      // velocity accuracy estimate [cm/s]
} NEOx_ecefV;

static volatile struct _NEOx_llh_t {
  uint32_t timestamp; // [msec]
  uint32_t iTOW;      // GPS time of the week of the navigation epoch [msec]
  int32_t lon;        // longitude [deg]
  int32_t lat;        // lattitude [deg]
  int32_t height;     // height above ellipsoid [mm]
  uint32_t hMSL;      // height above mean sea level [mm]
  uint32_t hAcc;      // horizontal accuracy estimate [mm]
  uint32_t vAcc;      // vertical accuracy estimate [mm]
} NEOx_llh;

static volatile struct _NEOx_nedV_t {
  uint32_t timestamp; // [msec]
  uint32_t iTOW;      // GPS time of the week of the navigation epoch [msec]
  int32_t velN;       // north velocity component [cm/s]
  int32_t velE;       // east velocity component [cm/s]
  int32_t velD;       // down velocity component [cm/s]
  uint32_t speed;     // speed (3D) [cm/s]
  uint32_t gspeed;    // ground speed (2D) [cm/s]
  int32_t heading;    // heading of motion (2D) [deg]
  uint32_t sAcc;      // speed accuracy estimate [cm/s]
  uint32_t cAcc;      // course / heading accuracy estimate [deg]
} NEOx_nedV;

static volatile struct _NEOx_dop_t {
  uint32_t timestamp; // GPS time of week of the navigation epoch [msec]
  uint32_t iTOW;      // GPS time of the week of the navigation epoch [msec]
  uint16_t gDOP;      // geometric DOP
  uint16_t pDOP;      // position DOP
  uint16_t tDOP;      // time DOP
  uint16_t vDOP;      // vertical DOP
  uint16_t hDOP;      // horizontal DOP
  uint16_t nDOP;      // northing DOP
  uint16_t eDOP;      // easting DOP
} NEOx_dop;

static volatile struct _NEOx_svinfo_t {
  uint32_t iTOW;     // GPS time of week [msec]
  uint8_t svid;      // Satellite ID
  uint8_t flags;     // Bitmask
  uint8_t quality;   // Bitfield
  uint8_t cno;       // Carrier to Noise Ratio (Signal Strength) [dBHz]
  int8_t elev;       // Elevation in integer degrees [deg]
  int16_t azim;      // Azimuth in integer degrees [deg]
  int32_t prRes;     // Pseudo range residual [cm]
} NEOx_svinfo[0xFF]; // track up to 255 channels, channel = array index

static volatile uint8_t NEOx_numTrkChHw =
    0u; // number of tracking channels available in hardware (read only)
static volatile uint8_t NEOx_numTrkChUse =
    0u; // number of tracking channels to use (<=numTrkChHw)
static volatile struct _NEOx_gnssCfg_t {
  uint8_t resTrkCh; // number of reserved (minimum) tracking channels for this
                    // GNSS system
  uint8_t maxTrkCh; // maximum number of tracking channels used for this GNSS
                    // system (>=resTrkCh)
  uint32_t flags;   // 0=Disable, 1=Enable GNSS system
} NEOx_gnssCfg[7];  // track up to 7 GNSS systems, system = array index [0=GPS,
                    // 1=SBAS, 5=QZSS, 6=GLONASS]

static volatile uint8_t NEOx_numSV =
    0u; // number of satellites used in Nav Solution
static volatile uint8_t NEOx_portID = 0u;

static uint8_t UBX_ACK_CFG_NAV5 = 0u;
static uint8_t UBX_ACK_CFG_NAVX5 = 0u;
static uint8_t UBX_ACK_CFG_GNSS = 0u;
static uint8_t UBX_ACK_CFG_SBAS = 0u;
static uint8_t UBX_ACK_CFG_MSG = 0u;
static uint8_t UBX_ACK_CFG_RST = 0u;
static uint8_t UBX_ACK_CFG_RATE = 0u;
static uint8_t UBX_ACK_CFG_CFG = 0u;

static uint8_t UBX_msgClass = 0u;
static uint8_t UBX_msgID = 0u;
static uint16_t UBX_msgLength = 0u;

const uint32_t NEOx_baudList[6] = {4800u,  9600u,  19200u,
                                   38400u, 57600u, 115200u};
/**
 * @}
 */

/** @defgroup NEOx_Private_Function Prototypes
 * @{
 */
static void NEOx_ForceBaud(uint8_t p, uint32_t baud);

static void NEOx_SetACKVal(uint8_t msgID);
static void NEOx_ClrACKVal(uint8_t msgID);
static uint8_t NEOx_GetACKVal(uint8_t msgID);
static uint8_t NEOx_GetACK(uint8_t msgID);

static void NEOx_ConfigPRT(uint32_t baud);
static uint8_t NEOx_ConfigMSG(uint8_t msgClass, uint8_t msgID, uint8_t rate);
// static uint8_t NEOx_ConfigRST(uint16_t navBbrMask, uint8_t resetMode);
static uint8_t NEOx_ConfigRATE(uint16_t measRate, uint16_t timeRef);
// static uint8_t NEOx_ConfigCFG(uint8_t option);
static uint8_t NEOx_ConfigSBAS(uint8_t continent);
static uint8_t NEOx_ConfigNAVX5(uint8_t usePPP);
static uint8_t NEOx_ConfigNAV5(uint8_t dynMode1);

static uint8_t NEOx_GetConfigGNSS(void);
/**
 * @}
 */

#ifdef _VERBOSE
static void NEOx_PrintGNSSInfo(uint8_t gnssID);
static void NEOx_PrintSVInfo(uint8_t chn);
#endif

/**
 * @brief  Runs through all baud rates and attempts to change the baud rate of
 * NEOx
 * @param  p UART peripheral ID
 * @param  baud Baudrate
 * @retval None
 */
static void NEOx_ForceBaud(uint8_t p, uint32_t baud) {
  uint8_t i;
  for (i = 0u; i < 6u; i++) {
    Delay(100);
    UART_SetBaud(p, NEOx_baudList[i]);

    Delay(100);
    NEOx_ConfigPRT(baud);
  }

  UART_SetBaud(p, baud);
  Delay(100);
}

/**
 * @brief  Initialize NEOx
 * @param  p UART peripheral ID
 * @param  measPeriod Navigation/Measurement rate value [msec]
 * @param  sbas Choose continent for selecting SBAS receivers (see Pg. 8 of 196
 * in GPS.G7-SW-12001-B)
 * @param  dynMode Dynamic platform model (see Pg. 1 of 196 in
 * GPS.G7-SW-12001-B)
 * @retval 0=Fail, 1=Succeed
 */
uint8_t NEOx_Init(uint8_t p, uint16_t measPeriod, uint8_t sbas,
                  uint8_t dynMode) {
  uint8_t retval = 1u;

  NEOx_portID = p;

  // Initialise UART interrupts
  UART_IT_RxIRQ(p, ENABLE);
  UART_DMA_RxIRQ(p, DISABLE);
  UART_DMA_RxModeConfig(p,
                        DMA_Mode_Normal); // used in hybrid IT-DMA mode; change
                                          // DMA receive mode to normal

  //--------------------------------------------------------------------
  // GNSS CONFIGURATION
  NEOx_ForceBaud(p, NEOx_BAUD);
  retval &= NEOx_ConfigRATE(measPeriod, CFG_RATE_TIMEREF_UTC);
  retval &= NEOx_ConfigSBAS(sbas);
  retval &= NEOx_SetDynMode(dynMode);

  //--------------------------------------------------------------------
  // GNSS OUTPUT CONFIGURATION
  // Note: Select the output packet to return
  retval &= NEOx_ConfigMSG(UBX_CLASS_NAV, UBX_ID_NAV_POSECEF,
                           0u); // position solution in ECEF
  retval &= NEOx_ConfigMSG(UBX_CLASS_NAV, UBX_ID_NAV_VELECEF,
                           0u); // velocity solution in ECEF
  retval &= NEOx_ConfigMSG(UBX_CLASS_NAV, UBX_ID_NAV_DOP,
                           0u); // dilution of precision
  retval &= NEOx_ConfigMSG(UBX_CLASS_NAV, UBX_ID_NAV_SOL,
                           1u); // navigation solution information
  retval &= NEOx_ConfigMSG(UBX_CLASS_NAV, UBX_ID_NAV_PVT,
                           0u); // navigation position velocity time solution
  retval &= NEOx_ConfigMSG(UBX_CLASS_NAV, UBX_ID_NAV_SVINFO,
                           0u); // space vehicle information
  //--------------------------------------------------------------------

#ifdef _VERBOSE
  NEOx_GetConfigGNSS();
#endif

  return retval;
}

//------------------------------------------------------------------------

/**
 * @brief  Set acknowledgement value of message
 * @param  msgID Message ID
 * @retval None
 */
static void NEOx_SetACKVal(uint8_t msgID) {
  switch (msgID) {
  case UBX_ID_CFG_NAV5:
    UBX_ACK_CFG_NAV5 = 1u;
    break;
  case UBX_ID_CFG_NAVX5:
    UBX_ACK_CFG_NAVX5 = 1u;
    break;
  case UBX_ID_CFG_GNSS:
    UBX_ACK_CFG_GNSS = 1u;
    break;
  case UBX_ID_CFG_SBAS:
    UBX_ACK_CFG_SBAS = 1u;
    break;
  case UBX_ID_CFG_MSG:
    UBX_ACK_CFG_MSG = 1u;
    break;
  case UBX_ID_CFG_RST:
    UBX_ACK_CFG_RST = 1u;
    break;
  case UBX_ID_CFG_RATE:
    UBX_ACK_CFG_RATE = 1u;
    break;
  case UBX_ID_CFG_CFG:
    UBX_ACK_CFG_CFG = 1u;
    break;
  default:
    break;
  }
}

/**
 * @brief  Clear acknowledgement value of message
 * @param  msgID Message ID
 * @retval None
 */
static void NEOx_ClrACKVal(uint8_t msgID) {
  switch (msgID) {
  case UBX_ID_CFG_NAV5:
    UBX_ACK_CFG_NAV5 = 0u;
    break;
  case UBX_ID_CFG_NAVX5:
    UBX_ACK_CFG_NAVX5 = 0u;
    break;
  case UBX_ID_CFG_GNSS:
    UBX_ACK_CFG_GNSS = 0u;
    break;
  case UBX_ID_CFG_SBAS:
    UBX_ACK_CFG_SBAS = 0u;
    break;
  case UBX_ID_CFG_MSG:
    UBX_ACK_CFG_MSG = 0u;
    break;
  case UBX_ID_CFG_RST:
    UBX_ACK_CFG_RST = 0u;
    break;
  case UBX_ID_CFG_RATE:
    UBX_ACK_CFG_RATE = 0u;
    break;
  case UBX_ID_CFG_CFG:
    UBX_ACK_CFG_CFG = 0u;
    break;
  default:
    break;
  }
}

/**
 * @brief  Get acknowledgement value of message
 * @param  msgID Message ID
 * @retval 1=ACK, 0=NAK
 */
static uint8_t NEOx_GetACKVal(uint8_t msgID) {
  switch (msgID) {
  case UBX_ID_CFG_NAV5:
    return UBX_ACK_CFG_NAV5;
    break;
  case UBX_ID_CFG_NAVX5:
    return UBX_ACK_CFG_NAVX5;
    break;
  case UBX_ID_CFG_GNSS:
    return UBX_ACK_CFG_GNSS;
    break;
  case UBX_ID_CFG_SBAS:
    return UBX_ACK_CFG_SBAS;
    break;
  case UBX_ID_CFG_MSG:
    return UBX_ACK_CFG_MSG;
    break;
  case UBX_ID_CFG_RST:
    return UBX_ACK_CFG_RST;
    break;
  case UBX_ID_CFG_RATE:
    return UBX_ACK_CFG_RATE;
    break;
  case UBX_ID_CFG_CFG:
    return UBX_ACK_CFG_CFG;
    break;
  default:
    return 0u;
    break;
  }
}

/**
 * @brief  Get acknowledgement value of message after timeout period
 * @param  msgID Message ID
 * @retval 1=ACK, 0=NAK
 */
static uint8_t NEOx_GetACK(uint8_t msgID) {
  uint32_t t0 = GetTime();
  uint32_t dt = 0u;
  uint8_t val = 0u;

  while ((dt < NEOx_ACK_TIMEOUT) && !val) {
    dt = GetTime() - t0;
    val = NEOx_GetACKVal(msgID);
  }

  if (dt >= NEOx_ACK_TIMEOUT) {
    NEOx_ClrACKVal(msgID);
    return ERROR;
  } else {
    NEOx_ClrACKVal(msgID);
    return SUCCESS;
  }
}

//------------------------------------------------------------------------

/**
 * @brief  NEO DMA Receive Interrupt Service Routine;
 *		   This grabs the remaining bytes from the message captured in
 *NEOx_ISR_IT()
 * @param buffer Pointer to buffer
 * @retval None
 */
void NEOx_ISR_DMA(void *buffer) {
#ifdef _VERBOSE
  char str[100];
#endif

  if (UBX_ValidCheckSum2((uint8_t *)buffer, UBX_msgClass, UBX_msgID,
                         UBX_msgLength)) {
    switch (UBX_msgClass) {
    case UBX_CLASS_NAV: {
      switch (UBX_msgID) {
      case UBX_ID_NAV_DOP: {
#ifdef _VERBOSE
        UART_Print(VERBOSE_UART_ID, "NAV-DOP ");
#endif
        UBX_NAV_DOP_t *pkt = (UBX_NAV_DOP_t *)buffer;

        NEOx_dop.timestamp = GetTime(); // [msec]
        NEOx_dop.iTOW =
            pkt->iTOW; // GPS time of week of the navigation epoch [msec]
        NEOx_dop.gDOP = pkt->gDOP; // geometric DOP
        NEOx_dop.pDOP = pkt->pDOP; // position DOP
        NEOx_dop.tDOP = pkt->tDOP; // time DOP
        NEOx_dop.vDOP = pkt->vDOP; // vertical DOP
        NEOx_dop.hDOP = pkt->hDOP; // horizontal DOP
        NEOx_dop.nDOP = pkt->nDOP; // northing DOP
        NEOx_dop.eDOP = pkt->eDOP; // easting DOP
      } break;

      case UBX_ID_NAV_POSECEF: {
#ifdef _VERBOSE
        UART_Print(VERBOSE_UART_ID, "NAV-POSECEF ");
#endif
        UBX_NAV_POSECEF_t *pkt = (UBX_NAV_POSECEF_t *)buffer;

        NEOx_ecefP.timestamp = GetTime(); // [msec]
        NEOx_ecefP.iTOW =
            pkt->iTOW; // GPS time of week of the navigation epoch [msec]
        NEOx_ecefP.ecefX = pkt->ecefX; // ECEF X coordinate [cm]
        NEOx_ecefP.ecefY = pkt->ecefY; // ECEF Y coordinate [cm]
        NEOx_ecefP.ecefZ = pkt->ecefZ; // ECEF Z coordinate [cm]
        NEOx_ecefP.pAcc = pkt->pAcc;   // position accuracy estimate [cm]
      } break;

      case UBX_ID_NAV_POSLLH: {
#ifdef _VERBOSE
        UART_Print(VERBOSE_UART_ID, "NAV-POSLLH ");
#endif
        UBX_NAV_POSLLH_t *pkt = (UBX_NAV_POSLLH_t *)buffer;

        NEOx_llh.timestamp = GetTime(); // [msec]
        NEOx_llh.iTOW =
            pkt->iTOW; // GPS time of week of the navigation epoch [msec]
        NEOx_llh.lon = pkt->lon;       // longitude [deg]
        NEOx_llh.lat = pkt->lat;       // lattitude [deg]
        NEOx_llh.height = pkt->height; // height above ellipsoid [mm]
        NEOx_llh.hMSL = pkt->hMSL;     // height above mean sea level [mm]
        NEOx_llh.hAcc = pkt->hAcc;     // horizontal accuracy estimate [mm]
        NEOx_llh.vAcc = pkt->vAcc;     // vertical accuracy estimate [mm]
      } break;

      case UBX_ID_NAV_PVT: {
#ifdef _VERBOSE
        UART_Print(VERBOSE_UART_ID, "NAV-PVT ");
#endif

        UBX_NAV_PVT_t *pkt = (UBX_NAV_PVT_t *)buffer;

        NEOx_numSV =
            pkt->numSV; // number of satellite used in navigation solution
        NEOx_dop.pDOP = pkt->pDOP; // position dilution of precision

        NEOx_llh.timestamp = GetTime(); // [msec]
        NEOx_llh.iTOW =
            pkt->iTOW; // GPS time of week of the navigation epoch [msec]
        NEOx_llh.lon = pkt->lon;       // longitude [deg]
        NEOx_llh.lat = pkt->lat;       // lattitude [deg]
        NEOx_llh.height = pkt->height; // height above ellipsoid [mm]
        NEOx_llh.hMSL = pkt->hMSL;     // height above mean sea level [mm]
        NEOx_llh.hAcc = pkt->hAcc;     // horizontal accuracy estimate [mm]
        NEOx_llh.vAcc = pkt->vAcc;     // vertical accuracy estimate [mm]

        NEOx_nedV.timestamp = GetTime(); // [msec]
        NEOx_nedV.velN = pkt->velN;      // north velocity component [mm/s]
        NEOx_nedV.velE = pkt->velE;      // east velocity component [mm/s]
        NEOx_nedV.velD = pkt->velD;      // down velocity component [mm/s]
        NEOx_nedV.gspeed =
            pkt->gspeed; // ground speed (2D) [mm/s] <-- UBX_NAV_PVT_t uses
                         // int32_t, which is different from UBX_NAV_VELNED_t
        NEOx_nedV.heading = pkt->heading; // heading of motion (2D) [deg]
        NEOx_nedV.sAcc = pkt->sAcc;       // speed accuracy estimate [mm/s]
        NEOx_nedV.cAcc = pkt->cAcc; // course / heading accuracy estimate [deg]
      } break;

      case UBX_ID_NAV_SOL: {
#ifdef _VERBOSE
        UART_Print(VERBOSE_UART_ID, "NAV-SOL ");
#endif
        UBX_NAV_SOL_t *pkt = (UBX_NAV_SOL_t *)buffer;

        NEOx_numSV =
            pkt->numSV; // number of satellite used in navigation solution
        NEOx_dop.pDOP = pkt->pDOP; // position dilution of precision

        NEOx_ecefP.timestamp = GetTime(); // [msec]
        NEOx_ecefP.iTOW =
            pkt->iTOW; // GPS time of week of the navigation epoch [msec]
        NEOx_ecefP.ecefX = pkt->ecefX; // ECEF X coordinate [cm]
        NEOx_ecefP.ecefY = pkt->ecefY; // ECEF Y coordinate [cm]
        NEOx_ecefP.ecefZ = pkt->ecefZ; // ECEF Z coordinate [cm]
        NEOx_ecefP.pAcc = pkt->pAcc;   // position accuracy estimate [cm]

        NEOx_ecefV.timestamp = GetTime(); // [msec]
        NEOx_ecefV.iTOW =
            pkt->iTOW; // GPS time of week of the navigation epoch [msec]
        NEOx_ecefV.ecefVX = pkt->ecefVX; // ECEF X coordinate [cm/s]
        NEOx_ecefV.ecefVY = pkt->ecefVY; // ECEF Y coordinate [cm/s]
        NEOx_ecefV.ecefVZ = pkt->ecefVZ; // ECEF Z coordinate [cm/s]
        NEOx_ecefV.sAcc = pkt->sAcc;     // velocity accuracy estimate [cm/s]
      } break;

      case UBX_ID_NAV_SVINFO: {
#ifdef _VERBOSE
        UART_Print(VERBOSE_UART_ID, "NAV-SVINFO ");
#endif

        UBX_NAV_SVINFO_t *pkt = (UBX_NAV_SVINFO_t *)buffer;
        uint8_t numCh = pkt->numCh;

        uint8_t i;
        for (i = 0; i < numCh; i++) {
          UBX_NAV_SVINFO_repBlock_t *blk =
              (UBX_NAV_SVINFO_repBlock_t *)(buffer + sizeof(UBX_NAV_SVINFO_t) +
                                            i * sizeof(
                                                    UBX_NAV_SVINFO_repBlock_t));
          NEOx_svinfo[blk->chn].iTOW = pkt->iTOW;   // GPS time of week [msec]
          NEOx_svinfo[blk->chn].svid = blk->svid;   // Satellite ID
          NEOx_svinfo[blk->chn].flags = blk->flags; // Bitmask
          NEOx_svinfo[blk->chn].quality = blk->quality; // Bitfield
          NEOx_svinfo[blk->chn].cno =
              blk->cno; // Carrier to Noise Ratio (Signal Strength) [dBHz]
          NEOx_svinfo[blk->chn].elev =
              blk->elev; // Elevation in integer degrees [deg]
          NEOx_svinfo[blk->chn].azim =
              blk->azim; // Azimuth in integer degrees [deg]
          NEOx_svinfo[blk->chn].prRes =
              blk->prRes; // Pseudo range residual [cm]

#ifdef _VERBOSE
          NEOx_PrintSVInfo(blk->chn);
#endif
        }
      } break;

      case UBX_ID_NAV_VELECEF: {
#ifdef _VERBOSE
        UART_Print(VERBOSE_UART_ID, "NAV-VELECEF ");
#endif
        UBX_NAV_VELECEF_t *pkt = (UBX_NAV_VELECEF_t *)buffer;

        NEOx_ecefV.timestamp = GetTime(); // [msec]
        NEOx_ecefV.iTOW =
            pkt->iTOW; // GPS time of week of the navigation epoch [msec]
        NEOx_ecefV.ecefVX = pkt->ecefVX; // ECEF X coordinate [cm/s]
        NEOx_ecefV.ecefVY = pkt->ecefVY; // ECEF Y coordinate [cm/s]
        NEOx_ecefV.ecefVZ = pkt->ecefVZ; // ECEF Z coordinate [cm/s]
        NEOx_ecefV.sAcc = pkt->sAcc;     // velocity accuracy estimate [cm/s]
      } break;

      case UBX_ID_NAV_VELNED: {
#ifdef _VERBOSE
        UART_Print(VERBOSE_UART_ID, "NAV-VELNED ");
#endif
        UBX_NAV_VELNED_t *pkt = (UBX_NAV_VELNED_t *)buffer;

        NEOx_nedV.timestamp = GetTime(); // [msec]
        NEOx_nedV.iTOW =
            pkt->iTOW; // GPS time of week of the navigation epoch [msec]
        NEOx_nedV.velN = pkt->velN;       // north velocity component [mm/s]
        NEOx_nedV.velE = pkt->velE;       // east velocity component [mm/s]
        NEOx_nedV.velD = pkt->velD;       // down velocity component [mm/s]
        NEOx_nedV.speed = pkt->speed;     // speed (3D) [mm/s]
        NEOx_nedV.gspeed = pkt->gspeed;   // ground speed (2D) [mm/s]
        NEOx_nedV.heading = pkt->heading; // heading of motion (2D) [deg]
        NEOx_nedV.sAcc = pkt->sAcc;       // speed accuracy estimate [mm/s]
        NEOx_nedV.cAcc = pkt->cAcc; // course / heading accuracy estimate [deg]
      } break;

      default:
#ifdef _VERBOSE
        sprintf(str, "Unrecognised NAV message: 0x%02x\r\n", UBX_msgID);
        UART_Print(VERBOSE_UART_ID, str);
#endif
        break;
      }
    } break;

      char str[100];
    case UBX_CLASS_CFG: {
      switch (UBX_msgID) {
      case UBX_ID_CFG_GNSS:
#ifdef _VERBOSE
        UART_Print(VERBOSE_UART_ID, "CFG-GNSS ");
#endif

        UBX_CFG_GNSS_t *pkt = (UBX_CFG_GNSS_t *)buffer;
        uint8_t numConfigBlocks = pkt->numConfigBlocks;
        NEOx_numTrkChHw = pkt->numTrkChHw; // number of tracking channels
                                           // available in hardware (read only)
        NEOx_numTrkChUse = pkt->numTrkChUse; // number of tracking channels to
                                             // use (<=numTrkChHw)

        uint8_t i;
        for (i = 0; i < numConfigBlocks; i++) {
          UBX_CFG_GNSS_configBlock_t *blk =
              (UBX_CFG_GNSS_configBlock_t
                   *)(buffer + sizeof(UBX_CFG_GNSS_t) +
                      i * sizeof(UBX_CFG_GNSS_configBlock_t));
          NEOx_gnssCfg[blk->gnssID].resTrkCh =
              blk->resTrkCh; // number of reserved (minimum) tracking channels
                             // for this GNSS system
          NEOx_gnssCfg[blk->gnssID].maxTrkCh =
              blk->maxTrkCh; // maximum number of tracking channels used for
                             // this GNSS system (>=resTrkCh)
          NEOx_gnssCfg[blk->gnssID].flags =
              blk->flags; // 0=Disable, 1-Enable GNSS system

#ifdef _VERBOSE
          NEOx_PrintGNSSInfo(blk->gnssID);
#endif
        }
        break;

      default:
#ifdef _VERBOSE
        sprintf(str, "Unrecognised CFG message: 0x%02x\r\n", UBX_msgID);
        UART_Print(VERBOSE_UART_ID, str);
#endif
        break;
      }
    }

    case UBX_CLASS_ACK: {
      switch (UBX_msgID) {
      case UBX_ID_ACK_ACK: {
        UBX_ACK_ACK_t *pkt;
        pkt = (UBX_ACK_ACK_t *)buffer;

        if (pkt->clsID == UBX_CLASS_CFG) {
          NEOx_SetACKVal(pkt->msgID);
        }
      } break;

      case UBX_ID_ACK_NAK: {
        UBX_ACK_NAK_t *pkt;
        pkt = (UBX_ACK_NAK_t *)buffer;

        if (pkt->clsID == UBX_CLASS_CFG) {
          NEOx_ClrACKVal(pkt->msgID);
        }
      } break;

      default:
#ifdef _VERBOSE
        sprintf(str, "Unrecognised ACK message: 0x%02x\r\n", UBX_msgID);
        UART_Print(VERBOSE_UART_ID, str);
#endif
        break;
      }
    } break;

    default:
#ifdef _VERBOSE
      sprintf(str, "Unrecognised UBX_CLASS: 0x%02x\r\n", UBX_msgClass);
      UART_Print(VERBOSE_UART_ID, str);
#endif
      break;
    }
  } else {
#ifdef _VERBOSE
    UART_Print(VERBOSE_UART_ID, "FAIL CHKSUM\r\n");
#endif
  }

  UART_DMA_RxIRQ(NEOx_portID, DISABLE); // disable UART DMA receive interrupt
  UART_IT_RxIRQ(NEOx_portID, ENABLE);   // enable UART receive interrupt
}

#if 1 // HYBRID UART RECEIVE INTERRUPT and UART DMA INTERRUPT

/**
 * @brief  NEO IT Receive Interrupt Service Routine;
 *		   This grabs the first 4 bytes of incoming data to determine
 *the type of packet that is about to be received
 * @param byte Incoming byte
 * @retval None
 */
void NEOx_ISR_IT(uint8_t byte) {
  static uint8_t cnt = 0u;
  static uint8_t
      buffer[6]; // [SYNC_CHAR_1, SYNC_CHAR_2, MSG_CLASS, MSG_ID, MSG_LENGTH]

  // prevent overflow
  if (cnt >= UBX_HDR_SIZE)
    cnt = 0u;

  // add byte to the buffer
  buffer[cnt++] = byte;

  if (buffer[0] != UBX_SYNC_CHAR_1) // grab first sync byte
  {
    cnt = 0u;
    return;
  }

  if (cnt < UBX_SYNC_SIZE)
    return;
  if (buffer[1] != UBX_SYNC_CHAR_2) // grab second sync byte
  {
    cnt = 0u;
    return;
  }

  // grab the next two bytes, which indicates message class and message ID
  if (cnt < UBX_HDR_SIZE)
    return;

  UBX_msgClass = buffer[2];
  UBX_msgID = buffer[3];
  UBX_msgLength = ((uint16_t)buffer[4] << 0u) | ((uint16_t)buffer[5] << 8u);

  UART_IT_RxIRQ(NEOx_portID, DISABLE); // disable the UART receive interrupt
  UART_DMA_RxBufferSizeConfig(
      NEOx_portID,
      UBX_msgLength + UBX_CHECKSUM_SIZE); // adjusted DMA buffer size
  UART_DMA_RxIRQ(NEOx_portID, ENABLE);    // enable DMA receive interrupt
  cnt = 0u;                               // reset counter
}

#else // UART RECEIVE INTERRUPT (KEEP THIS FOR DEBUGGING PURPOSES)
/**
 * @brief  UART IT Receive Interrupt Service Routine;
 * @retval None
 */
void NEOx_ISR_IT(uint8_t byte) {
  static uint8_t buffer[UBX_MAX_DATA_SIZE];
  static uint8_t cnt = 0u;

  // prevent overflow
  if (cnt >= UBX_MAX_DATA_SIZE) {
    cnt = 0u; // reset byte counter
  }

  // add byte to the buffer
  buffer[cnt++] = byte;

  if (cnt < 1)
    return;
  if (buffer[0] != UBX_SYNC_CHAR_1) {
    cnt = 0u;
    return;
  }

  if (cnt < 2)
    return;
  if (buffer[1] != UBX_SYNC_CHAR_2) {
    cnt = 0u;
    return;
  }

  if (cnt < 4)
    return;

  switch (buffer[2]) // message class
  {
  case UBX_CLASS_NAV:
    switch (buffer[3]) // message ID
    {
    case UBX_ID_NAV_POSECEF:
      if (cnt == sizeof(UBX_NAV_POSECEF_t) + UBX_OVERHEAD) {
        if (UBX_ValidCheckSum((uint8_t *)buffer, sizeof(UBX_NAV_POSECEF_t))) {
#ifdef _VERBOSE
          UART_Print(VERBOSE_UART_ID, "NAV-POSECEF ");
#endif
          UBX_NAV_POSECEF_t *pkt = (UBX_NAV_POSECEF_t *)(buffer + UBX_HDR_SIZE);

          NEOx_ecefP.timestamp = GetTime(); // [msec]
          NEOx_ecefP.ecefX = pkt->ecefX;    // ECEF X coordinate [cm]
          NEOx_ecefP.ecefY = pkt->ecefY;    // ECEF Y coordinate [cm]
          NEOx_ecefP.ecefZ = pkt->ecefZ;    // ECEF Z coordinate [cm]
          NEOx_ecefP.pAcc = pkt->pAcc;      // position accuracy estimate [cm]
        }
        cnt = 0u;
        return;
      }
      break;

    case UBX_ID_NAV_POSLLH:
      if (cnt == sizeof(UBX_NAV_POSLLH_t) + UBX_OVERHEAD) {
        if (UBX_ValidCheckSum((uint8_t *)buffer, sizeof(UBX_NAV_POSLLH_t))) {
#ifdef _VERBOSE
          UART_Print(VERBOSE_UART_ID, "NAV-POSLLH ");
#endif
          UBX_NAV_POSLLH_t *pkt = (UBX_NAV_POSLLH_t *)(buffer + UBX_HDR_SIZE);

          NEOx_llh.timestamp = GetTime(); // [msec]
          NEOx_llh.lon = pkt->lon;        // longitude [deg]
          NEOx_llh.lat = pkt->lat;        // lattitude [deg]
          NEOx_llh.height = pkt->height;  // height above ellipsoid [mm]
          NEOx_llh.hMSL = pkt->hMSL;      // height above mean sea level [mm]
          NEOx_llh.hAcc = pkt->hAcc;      // horizontal accuracy estimate [mm]
          NEOx_llh.vAcc = pkt->vAcc;      // vertical accuracy estimate [mm]
        }
        cnt = 0u;
        return;
      }
      break;

    case UBX_ID_NAV_DOP:
      if (cnt == sizeof(UBX_NAV_DOP_t) + UBX_OVERHEAD) {
        if (UBX_ValidCheckSum((uint8_t *)buffer, sizeof(UBX_NAV_DOP_t))) {
#ifdef _VERBOSE
          UART_Print(VERBOSE_UART_ID, "NAV-DOP ");
#endif
          UBX_NAV_DOP_t *pkt = (UBX_NAV_DOP_t *)(buffer + UBX_HDR_SIZE);

          NEOx_dop.timestamp = GetTime(); // [msec]
          NEOx_dop.iTOW =
              pkt->iTOW; // GPS time of week of the navigation epoch [msec]
          NEOx_dop.gDOP = pkt->gDOP; // geometric DOP
          NEOx_dop.pDOP = pkt->pDOP; // position DOP
          NEOx_dop.tDOP = pkt->tDOP; // time DOP
          NEOx_dop.vDOP = pkt->vDOP; // vertical DOP
          NEOx_dop.hDOP = pkt->hDOP; // horizontal DOP
          NEOx_dop.nDOP = pkt->nDOP; // northing DOP
          NEOx_dop.eDOP = pkt->eDOP; // easting DOP
        }
        cnt = 0u;
        return;
      }
      break;

    case UBX_ID_NAV_PVT:
      if (cnt == sizeof(UBX_NAV_PVT_t) + UBX_OVERHEAD) {
        if (UBX_ValidCheckSum((uint8_t *)buffer, sizeof(UBX_NAV_PVT_t))) {
#ifdef _VERBOSE
          UART_Print(VERBOSE_UART_ID, "NAV-PVT ");
#endif
          UBX_NAV_PVT_t *pkt = (UBX_NAV_PVT_t *)(buffer + UBX_HDR_SIZE);

          NEOx_numSV = pkt->numSV;

          NEOx_llh.timestamp = GetTime(); // [msec]
          NEOx_llh.lon = pkt->lon;        // longitude [deg]
          NEOx_llh.lat = pkt->lat;        // lattitude [deg]
          NEOx_llh.height = pkt->height;  // height above ellipsoid [mm]
          NEOx_llh.hMSL = pkt->hMSL;      // height above mean sea level [mm]
          NEOx_llh.hAcc = pkt->hAcc;      // horizontal accuracy estimate [mm]
          NEOx_llh.vAcc = pkt->vAcc;      // vertical accuracy estimate [mm]

          NEOx_nedV.timestamp = GetTime(); // [msec]
          NEOx_nedV.velN = pkt->velN;      // north velocity component [cm/s]
          NEOx_nedV.velE = pkt->velE;      // east velocity component [cm/s]
          NEOx_nedV.velD = pkt->velD;      // down velocity component [cm/s]
          // NEOx_nedV.gspeed = pkt->gspeed; // ground speed (2D) [cm/s] <--
          // UBX_NAV_PVT_t uses int32_t, which is different from
          // UBX_NAV_VELNED_t
          NEOx_nedV.heading = pkt->heading; // heading of motion (2D) [deg]
          NEOx_nedV.sAcc = pkt->sAcc;       // speed accuracy estimate [cm/s]
          NEOx_nedV.cAcc =
              pkt->cAcc; // course / heading accuracy estimate [deg]
        }
        cnt = 0u;
        return;
      }
      break;

    case UBX_ID_NAV_SOL:
      if (cnt == sizeof(UBX_NAV_SOL_t) + UBX_OVERHEAD) {
        if (UBX_ValidCheckSum((uint8_t *)buffer, sizeof(UBX_NAV_SOL_t))) {
#ifdef _VERBOSE
          UART_Print(VERBOSE_UART_ID, "NAV-SOL ");
#endif
          UBX_NAV_SOL_t *pkt = (UBX_NAV_SOL_t *)(buffer + UBX_HDR_SIZE);

          NEOx_ecefP.timestamp = GetTime(); // [msec]
          NEOx_ecefP.ecefX = pkt->ecefX;    // ECEF X coordinate [cm]
          NEOx_ecefP.ecefY = pkt->ecefY;    // ECEF Y coordinate [cm]
          NEOx_ecefP.ecefZ = pkt->ecefZ;    // ECEF Z coordinate [cm]
          NEOx_ecefP.pAcc = pkt->pAcc;      // position accuracy estimate [cm]

          NEOx_ecefV.timestamp = GetTime(); // [msec]
          NEOx_ecefV.ecefVX = pkt->ecefVX;  // ECEF X coordinate [cm/s]
          NEOx_ecefV.ecefVY = pkt->ecefVY;  // ECEF Y coordinate [cm/s]
          NEOx_ecefV.ecefVZ = pkt->ecefVZ;  // ECEF Z coordinate [cm/s]
          NEOx_ecefV.sAcc = pkt->sAcc;      // velocity accuracy estimate [cm/s]
        }
        cnt = 0u;
        return;
      }
      break;

    case UBX_ID_NAV_VELECEF:
      if (cnt == sizeof(UBX_NAV_VELECEF_t) + UBX_OVERHEAD) {
        if (UBX_ValidCheckSum((uint8_t *)buffer, sizeof(UBX_NAV_VELECEF_t))) {
#ifdef _VERBOSE
          UART_Print(VERBOSE_UART_ID, "NAV-VELECEF ");
#endif
          UBX_NAV_VELECEF_t *pkt = (UBX_NAV_VELECEF_t *)(buffer + UBX_HDR_SIZE);

          NEOx_ecefV.timestamp = GetTime(); // [msec]
          NEOx_ecefV.ecefVX = pkt->ecefVX;  // ECEF X coordinate [cm/s]
          NEOx_ecefV.ecefVY = pkt->ecefVY;  // ECEF Y coordinate [cm/s]
          NEOx_ecefV.ecefVZ = pkt->ecefVZ;  // ECEF Z coordinate [cm/s]
          NEOx_ecefV.sAcc = pkt->sAcc;      // velocity accuracy estimate [cm/s]
        }
        cnt = 0u;
        return;
      }
      break;

    case UBX_ID_NAV_VELNED:
      if (cnt == sizeof(UBX_NAV_VELNED_t) + UBX_OVERHEAD) {
        if (UBX_ValidCheckSum((uint8_t *)buffer, sizeof(UBX_NAV_VELNED_t))) {
#ifdef _VERBOSE
          UART_Print(VERBOSE_UART_ID, "NAV-VELNED ");
#endif
          UBX_NAV_VELNED_t *pkt = (UBX_NAV_VELNED_t *)(buffer + UBX_HDR_SIZE);

          NEOx_nedV.timestamp = GetTime();  // [msec]
          NEOx_nedV.velN = pkt->velN;       // north velocity component [cm/s]
          NEOx_nedV.velE = pkt->velE;       // east velocity component [cm/s]
          NEOx_nedV.velD = pkt->velD;       // down velocity component [cm/s]
          NEOx_nedV.speed = pkt->speed;     // speed (3D) [cm/s]
          NEOx_nedV.gspeed = pkt->gspeed;   // ground speed (2D) [cm/s]
          NEOx_nedV.heading = pkt->heading; // heading of motion (2D) [deg]
          NEOx_nedV.sAcc = pkt->sAcc;       // speed accuracy estimate [cm/s]
          NEOx_nedV.cAcc =
              pkt->cAcc; // course / heading accuracy estimate [deg]
        }
        cnt = 0u;
        return;
      }
      break;

    default:
      cnt = 0u;
      break;
    }
    break;

  case UBX_CLASS_ACK:
    switch (buffer[3]) // message ID
    {
    case UBX_ID_ACK_ACK:

      if (cnt == sizeof(UBX_ACK_ACK_t) + UBX_OVERHEAD) {
        if (UBX_ValidCheckSum((uint8_t *)buffer, sizeof(UBX_ACK_ACK_t))) {
          UBX_ACK_ACK_t *pkt;
          pkt = (UBX_ACK_ACK_t *)(buffer + UBX_HDR_SIZE);

          if (pkt->clsID == UBX_CLASS_CFG) {
            NEOx_SetACK(pkt->msgID, 1u);
          }
        }
        cnt = 0u;
        return;
      }
      break;

    case UBX_ID_ACK_NAK:
      if (cnt == sizeof(UBX_ACK_NAK_t) + UBX_OVERHEAD) {
        if (UBX_ValidCheckSum((uint8_t *)buffer, sizeof(UBX_ACK_NAK_t))) {
          UBX_ACK_NAK_t *pkt;
          pkt = (UBX_ACK_NAK_t *)(buffer + UBX_HDR_SIZE);

          if (pkt->clsID == UBX_CLASS_CFG) {
            NEOx_SetACK(pkt->msgID, 0u);
          }
        }
        cnt = 0u;
        return;
      }
      break;

    default:
      cnt = 0u;
      break;
    }
    break;

  default:
    cnt = 0u;
    break;
  }

  return;
}
#endif

//------------------------------------------------------------------------

// clsID: 0x06, msgID: 0x00
static void NEOx_ConfigPRT(uint32_t baud) {
  UBX_pkt_t UBX_pkt;
  UBX_SetHeader((uint8_t *)&UBX_pkt, UBX_CLASS_CFG, UBX_ID_CFG_PRT,
                sizeof(UBX_CFG_PRT_t));

  UBX_CFG_PRT_t *pkt = (UBX_CFG_PRT_t *)UBX_pkt.data;

  pkt->portID = CFG_PRT_UART;
  pkt->reserved0 = 0x00;
  pkt->txReady = 0x0000;
  pkt->mode = CFG_PRT_MODE_STOPBIT_1 | CFG_PRT_MODE_PARITY_NONE |
              CFG_PRT_MODE_CHARLEN_8;
  pkt->baudrate = baud;
  pkt->inProtoMask = CFG_PRT_PROTO_MASK_UBX;
  pkt->outProtoMask = CFG_PRT_PROTO_MASK_UBX;
  pkt->flags = 0x0000;
  pkt->reserved5 = 0x0000;

  UBX_SetCheckSum((uint8_t *)&UBX_pkt, UBX_pkt.length);
  UART_Write_IT(NEOx_portID, (uint8_t *)&UBX_pkt,
                UBX_pkt.length + UBX_OVERHEAD);
}

// clsID: 0x06, msgID: 0x01
static uint8_t NEOx_ConfigMSG(uint8_t msgClass, uint8_t msgID, uint8_t rate) {
  UBX_pkt_t UBX_pkt;
  UBX_SetHeader((uint8_t *)&UBX_pkt, UBX_CLASS_CFG, UBX_ID_CFG_MSG,
                sizeof(UBX_CFG_MSG_t));

  UBX_CFG_MSG_t *pkt = (UBX_CFG_MSG_t *)UBX_pkt.data;

  pkt->msgClass = msgClass; // message class
  pkt->msgID = msgID;       // message identifier
  pkt->rate = rate;         // send rate on current port

  UBX_SetCheckSum((uint8_t *)&UBX_pkt, UBX_pkt.length);
  UART_Write_IT(NEOx_portID, (uint8_t *)&UBX_pkt,
                UBX_pkt.length + UBX_OVERHEAD);

  return NEOx_GetACK(UBX_ID_CFG_MSG);
}

// clsID: 0x06, msgID: 0x04
#if 0
static uint8_t NEOx_ConfigRST(uint16_t navBbrMask, uint8_t resetMode) 
{
	UBX_pkt_t UBX_pkt;
	UBX_SetHeader((uint8_t *)&UBX_pkt,UBX_CLASS_CFG,UBX_ID_CFG_RST,sizeof(UBX_CFG_RST_t));

	UBX_CFG_RST_t *pkt = (UBX_CFG_RST_t *)UBX_pkt.data;

	pkt->navBbrMask = navBbrMask; // BBR sections to clear 
	pkt->resetMode = resetMode; // reset type
	pkt->reserved1 = 0x00; // reserved
	
	UBX_SetCheckSum((uint8_t *)&UBX_pkt,UBX_pkt.length);
	UART_Write_IT(NEOx_portID,(uint8_t *)&UBX_pkt,UBX_pkt.length+UBX_OVERHEAD);

	return NEOx_GetACK(UBX_ID_CFG_RST);
}
#endif

// clsID: 0x06, msgID: 0x08
static uint8_t NEOx_ConfigRATE(uint16_t measRate, uint16_t timeRef) {
  UBX_pkt_t UBX_pkt;
  UBX_SetHeader((uint8_t *)&UBX_pkt, UBX_CLASS_CFG, UBX_ID_CFG_RATE,
                sizeof(UBX_CFG_RATE_t));

  UBX_CFG_RATE_t *pkt = (UBX_CFG_RATE_t *)UBX_pkt.data;

  pkt->measRate = measRate;
  pkt->navRate = 1u;
  pkt->timeRef = timeRef;

  UBX_SetCheckSum((uint8_t *)&UBX_pkt, UBX_pkt.length);
  UART_Write_IT(NEOx_portID, (uint8_t *)&UBX_pkt,
                UBX_pkt.length + UBX_OVERHEAD);

  uint8_t retval = NEOx_GetACK(UBX_ID_CFG_RATE);
#ifdef _VERBOSE
  if (!retval) {
    UART_Print(VERBOSE_UART_ID, "ERROR: NEOx_ConfigRATE()\r\n");
  }
#endif
  return retval;
}

// clsID: 0x06, msgID: 0x09
#if 0
static uint8_t NEOx_ConfigCFG(uint8_t option)
{
	UBX_pkt_t UBX_pkt;
	UBX_SetHeader((uint8_t *)&UBX_pkt,UBX_CLASS_CFG,UBX_ID_CFG_CFG,sizeof(UBX_CFG_CFG_t));

	UBX_CFG_CFG_t *pkt = (UBX_CFG_CFG_t *)UBX_pkt.data;
	
	switch (option)
	{
		case CLEAR:
			pkt->clearMask = CFG_CFG_MASK_IOPORT | CFG_CFG_MASK_MSGCONF | CFG_CFG_MASK_NAVCONF; // mask with configuration sub-sections to clear (i.e. load default configurations to permanent configurations in non-volatile memory)
			pkt->saveMask = 0u; // mask with configuration sub-section to save (i.e. save current configuration to non-volatile memory)
			pkt->loadMask = 0u; // mask with configuration sub-sections to load (i.e. load permanent configurations from non-volatile memory to current configurations)
			break;
		
		case SAVE:
			pkt->clearMask = 0u; // mask with configuration sub-sections to clear (i.e. load default configurations to permanent configurations in non-volatile memory)
			pkt->saveMask = CFG_CFG_MASK_IOPORT | CFG_CFG_MASK_MSGCONF | CFG_CFG_MASK_NAVCONF; // mask with configuration sub-section to save (i.e. save current configuration to non-volatile memory)
			pkt->loadMask = 0u; // mask with configuration sub-sections to load (i.e. load permanent configurations from non-volatile memory to current configurations)
			break;
		
		default:
			pkt->clearMask = 0u; // mask with configuration sub-sections to clear (i.e. load default configurations to permanent configurations in non-volatile memory)
			pkt->saveMask = 0u; // mask with configuration sub-section to save (i.e. save current configuration to non-volatile memory)
			pkt->loadMask = 0u; // mask with configuration sub-sections to load (i.e. load permanent configurations from non-volatile memory to current configurations)
			
			break;
	}

	pkt->deviceMask = CFG_CFG_DEVEEPROM; // mask which selects the devices for this command
	
	UBX_SetCheckSum((uint8_t *)&UBX_pkt,UBX_pkt.length);
	UART_Write_IT(NEOx_portID,(uint8_t *)&UBX_pkt,UBX_pkt.length+UBX_OVERHEAD);

	uint8_t retval = NEOx_GetACK(UBX_ID_CFG_CFG);
#ifdef _VERBOSE
	if (!retval) {UART_Print(VERBOSE_UART_ID,"ERROR: NEOx_ConfigCFG()\r\n");}
#endif
	return retval;
}
#endif

// clsID: 0x06, msgID: 0x16
static uint8_t NEOx_ConfigSBAS(uint8_t continent) {
  UBX_pkt_t UBX_pkt;
  UBX_SetHeader((uint8_t *)&UBX_pkt, UBX_CLASS_CFG, UBX_ID_CFG_SBAS,
                sizeof(UBX_CFG_SBAS_t));

  UBX_CFG_SBAS_t *pkt = (UBX_CFG_SBAS_t *)UBX_pkt.data;

  pkt->mode = CFG_SBAS_MODE_ENABLED;                           // SBAS mode
  pkt->usage = CFG_SBAS_USAGE_RANGE | CFG_SBAS_USAGE_DIFFCORR; // SBAS usage
  pkt->maxSBAS = 3u; // maximum number of SBAS prioritzed tracking channels
                     // (valid range: 0 - 3) to use (obsolete and superseeded by
                     // UBX-CFG-GNSS in protocol versions 14.00+)

  switch (continent) {
  case NEOx_SBAS_N_AMERICA:
    pkt->scanmode1 =
        CFG_SBAS_SCANMODE1_PRN_133 | CFG_SBAS_SCANMODE1_PRN_135 |
        CFG_SBAS_SCANMODE1_PRN_138; // which SBAS PRN numbers to search for; if
                                    // all bits are set to zero, auto-scan (i.e.
                                    // all valid PRNs) are searched
    pkt->scanmode2 = 0x00;          // continuation of scanmode bitmask
    break;

  case NEOx_SBAS_EUROPE:
    pkt->scanmode1 =
        CFG_SBAS_SCANMODE1_PRN_120 | CFG_SBAS_SCANMODE1_PRN_124 |
        CFG_SBAS_SCANMODE1_PRN_126; // which SBAS PRN numbers to search for; if
                                    // all bits are set to zero, auto-scan (i.e.
                                    // all valid PRNs) are searched
    pkt->scanmode2 = 0x00;          // continuation of scanmode bitmask
    break;

  case NEOx_SBAS_ASIA:
    pkt->scanmode1 =
        CFG_SBAS_SCANMODE1_PRN_129 |
        CFG_SBAS_SCANMODE1_PRN_137; // which SBAS PRN numbers to search for; if
                                    // all bits are set to zero, auto-scan (i.e.
                                    // all valid PRNs) are searched
    pkt->scanmode2 = 0x00;          // continuation of scanmode bitmask
    break;

  default:
    pkt->scanmode1 =
        CFG_SBAS_SCANMODE1_PRN_120 | CFG_SBAS_SCANMODE1_PRN_124 |
        CFG_SBAS_SCANMODE1_PRN_126 | CFG_SBAS_SCANMODE1_PRN_127 |
        CFG_SBAS_SCANMODE1_PRN_129 | CFG_SBAS_SCANMODE1_PRN_133 |
        CFG_SBAS_SCANMODE1_PRN_135 | CFG_SBAS_SCANMODE1_PRN_137 |
        CFG_SBAS_SCANMODE1_PRN_138; // which SBAS PRN numbers to search for; if
                                    // all bits are set to zero, auto-scan (i.e.
                                    // all valid PRNs) are searched
    pkt->scanmode2 = 0x00;          // continuation of scanmode bitmask
    break;
  }

  UBX_SetCheckSum((uint8_t *)&UBX_pkt, UBX_pkt.length);
  UART_Write_IT(NEOx_portID, (uint8_t *)&UBX_pkt,
                UBX_pkt.length + UBX_OVERHEAD);

  uint8_t retval = NEOx_GetACK(UBX_ID_CFG_SBAS);
#ifdef _VERBOSE
  if (!retval) {
    UART_Print(VERBOSE_UART_ID, "ERROR: NEOx_ConfigSBAS()\r\n");
  }
#endif
  return retval;
}

// clsID: 0x06, msgID: 0x23
static uint8_t NEOx_ConfigNAVX5(uint8_t usePPP) {
  UBX_pkt_t UBX_pkt;
  UBX_SetHeader((uint8_t *)&UBX_pkt, UBX_CLASS_CFG, UBX_ID_CFG_NAVX5,
                sizeof(UBX_CFG_NAVX5_t));

  UBX_CFG_NAVX5_t *pkt = (UBX_CFG_NAVX5_t *)UBX_pkt.data;

  pkt->version = 0u; // message version (0 for this version)
  pkt->mask1 = UBX_CFG_NAVX5_MASK_MINMAX | UBX_CFG_NAVX5_MASK_MINCNO |
               UBX_CFG_NAVX5_MASK_INITIAL3DFIX | UBX_CFG_NAVX5_MASK_WKNROLL |
               UBX_CFG_NAVX5_MASK_PPP; // first parameters bitmask; only the
                                       // flagged parameters will be applied,
                                       // unused bits must be set to 0.
  pkt->reserved_0 = 0u;                // always set to zero
  pkt->reserved_1 = 0u;                // always set to zero
  pkt->reserved_2 = 0u;                // always set to zero
  pkt->minSVs = 4u;     // minimum number of satellites for navigation [#SVs]
  pkt->maxSVs = 22u;    // maximum number of satellites for navigation [#SVs]
  pkt->minCNO = 7u;     // minimum satellite signal level for navigation [dBHz]
  pkt->reserved_5 = 0u; // always set to zero
  pkt->iniFix3D = 0u;   // initial Fix must be 3D flag (0=false/1=true)
  pkt->reserved_6 = 0u; // always set to zero
  pkt->reserved_7 = 0u; // always set to zero
  pkt->reserved_8 = 0u; // always set to zero
  pkt->wknRollover =
      1691u; // GPS week rollover number; GPS week numbers will be set correctly
             // from this week up to 1024 weeks after this week. Setting this to
             // 0 reverts to firmware default.
  pkt->reserved_9 = 0u;  // always set to zero
  pkt->reserved_10 = 0u; // always set to zero
  pkt->reserved_11 = 0u; // always set to zero
  pkt->usePPP = usePPP;  // use Precise Point Positioning flag (0=false/1=true)
  pkt->aopCfg = 1u;      // assistNow Autonomous configuration
  pkt->reserved_12 = 0u; // always set to zero
  pkt->reserved_13 = 0u; // always set to zero
  pkt->aopOrbMaxErr = 100u; // maximum acceptable (modelled) AssistNow
                            // Autonomous orbit error [m] [5..1000]
  pkt->reserved_14 = 0u;    // always set to zero
  pkt->reserved_15 = 0u;    // always set to zero
  pkt->reserved_3 = 0u;     // always set to zero
  pkt->reserved_4 = 0u;     // always set to zero

  UBX_SetCheckSum((uint8_t *)&UBX_pkt, UBX_pkt.length);
  UART_Write_IT(NEOx_portID, (uint8_t *)&UBX_pkt,
                UBX_pkt.length + UBX_OVERHEAD);

  uint8_t retval = NEOx_GetACK(UBX_ID_CFG_NAVX5);
#ifdef _VERBOSE
  if (!retval) {
    UART_Print(VERBOSE_UART_ID, "ERROR: NEOx_ConfigNAVX5()\r\n");
  }
#endif
  return retval;
}

// clsID: 0x06, msgID: 0x24
static uint8_t NEOx_ConfigNAV5(uint8_t dynMode1) {
  UBX_pkt_t UBX_pkt;
  UBX_SetHeader((uint8_t *)&UBX_pkt, UBX_CLASS_CFG, UBX_ID_CFG_NAV5,
                sizeof(UBX_CFG_NAV5_t));

  UBX_CFG_NAV5_t *pkt = (UBX_CFG_NAV5_t *)UBX_pkt.data;

  pkt->mask = UBX_CFG_NAV5_MASK_DYN | UBX_CFG_NAV5_MASK_POSFIXMODE |
              UBX_CFG_NAV5_MASK_POSMASK | UBX_CFG_NAV5_MASK_TIMEMASK |
              UBX_CFG_NAV5_MASK_STATICHOLDMASK |
              UBX_CFG_NAV5_MASK_DGPSMASK; // parameters bitmask. only the masked
                                          // parameters will be applied
  pkt->dynMode1 = dynMode1;               // dynamic platform model
  pkt->fixMode = UBX_CFG_NAV5_FIX_MODE_AUTO; // position fixing mode
  pkt->fixedAlt = 0;                         // fixed altitude [m]
  pkt->fixedAltVar = 0; // fixed altitude variance for 2D mode [m^2]
  pkt->minElev =
      UBX_CFG_NAV5_MIN_ELEV_DEFAULT; // minimum elevation for a GNSS satellite
                                     // to be used in NAV [deg]
  pkt->drLimit = 0u;                 // reserved [sec]
  pkt->pDop = 25u;                   // position DOP mask to use
  pkt->tDop = 25u;                   // time DOP mask to use
  pkt->pAcc = 100u;                  // position accuracy mask [m]
  pkt->tAcc = 300u;                  // time accuracy mask [m]
  pkt->staticHoldThresh = 0u;        // static hold threshold [cm/s]
  pkt->dgpsTimeOut = 60u;            // DGPS timeout [sec]
  pkt->cnoThreshNumSVs = 0u; // number of satellites required to have C/N0 above
                             // cnoThres for a fix to be attempted
  pkt->cnoThresh =
      0u; // C/N0 threshold for decideding whether to attempt a fix [dBHz]
  pkt->reserved2 = 0u; // always set to zero
  pkt->reserved3 = 0u; // always set to zero
  pkt->reserved4 = 0u; // always set to zero

  UBX_SetCheckSum((uint8_t *)&UBX_pkt, UBX_pkt.length);
  UART_Write_IT(NEOx_portID, (uint8_t *)&UBX_pkt,
                UBX_pkt.length + UBX_OVERHEAD);

  uint8_t retval = NEOx_GetACK(UBX_ID_CFG_NAV5);
#ifdef _VERBOSE
  if (!retval) {
    UART_Print(VERBOSE_UART_ID, "ERROR: NEOx_ConfigNAV5()\r\n");
  }
#endif
  return retval;
}

// clsID: 0x06, msgID: 0x3E
static uint8_t NEOx_GetConfigGNSS(void) {
  UBX_pkt_t UBX_pkt;
  UBX_SetHeader((uint8_t *)&UBX_pkt, UBX_CLASS_CFG, UBX_ID_CFG_GNSS, 0u);
  UBX_SetCheckSum((uint8_t *)&UBX_pkt, 0u);
  UART_Write_IT(NEOx_portID, (uint8_t *)&UBX_pkt, UBX_OVERHEAD);

  uint8_t retval = NEOx_GetACK(UBX_ID_CFG_GNSS);
#ifdef _VERBOSE
  if (!retval) {
    UART_Print(VERBOSE_UART_ID, "ERROR: NEOx_GetConfigGNSS()\r\n");
  }
#endif
  return retval;
}

//------------------------------------------------------------------------

/**
 * @brief  Set dynamic mode
 * @param  dynMode Dynamic mode [NEOx_PORTABLE,..,NEOx_AIRBORNE_4G]
 * @retval 0=Fail, 1=Success
 */
uint8_t NEOx_SetDynMode(uint8_t dynMode) {
  uint8_t retval = 1u;

  switch (dynMode) {
  case NEOx_PORTABLE:
    retval &= NEOx_ConfigNAV5(UBX_CFG_NAV5_DYN_MODE_PORTABLE);
    break;
  case NEOx_STATIONARY:
    retval &= NEOx_ConfigNAV5(UBX_CFG_NAV5_DYN_MODE_STATIONARY);
    break;
  case NEOx_PEDESTRIAN:
    retval &= NEOx_ConfigNAV5(UBX_CFG_NAV5_DYN_MODE_PEDESTRIAN);
    break;
  case NEOx_AUTOMOTIVE:
    retval &= NEOx_ConfigNAV5(UBX_CFG_NAV5_DYN_MODE_AUTOMOTIVE);
    break;
  case NEOx_SEA:
    retval &= NEOx_ConfigNAV5(UBX_CFG_NAV5_DYN_MODE_SEA);
    break;
  case NEOx_AIRBORNE_1G:
    retval &= NEOx_ConfigNAV5(UBX_CFG_NAV5_DYN_MODE_AIRBORNE_1G);
    break;
  case NEOx_AIRBORNE_2G:
    retval &= NEOx_ConfigNAV5(UBX_CFG_NAV5_DYN_MODE_AIRBORNE_2G);
    break;
  case NEOx_AIRBORNE_4G:
    retval &= NEOx_ConfigNAV5(UBX_CFG_NAV5_DYN_MODE_AIRBORNE_4G);
    break;
  default:
    retval &= NEOx_ConfigNAV5(UBX_CFG_NAV5_DYN_MODE_PORTABLE);
    break;
  }

  switch (dynMode) {
  case NEOx_STATIONARY:
    retval &= NEOx_ConfigNAVX5(NEOx_PPP_ENABLE);
    break;
  default:
    NEOx_ConfigNAVX5(NEOx_PPP_DISABLE);
    break;
  }

  return retval;
}

//------------------------------------------------------------------------

__inline float NEOx_GetEcefPX(void) {
  return NEOx_ecefP.ecefX / 100.0f;
} // ECEF X coordinate [m]
__inline float NEOx_GetEcefPY(void) {
  return NEOx_ecefP.ecefY / 100.0f;
} // ECEF Y coordinate [m]
__inline float NEOx_GetEcefPZ(void) {
  return NEOx_ecefP.ecefZ / 100.0f;
} // ECEF Z coordinate [m]
__inline float NEOx_GetEcefPAcc(void) {
  return NEOx_ecefP.pAcc / 100.0f;
} // position accuracy estimate [m]

__inline float NEOx_GetEcefVX(void) {
  return NEOx_ecefV.ecefVX / 100.0f;
} // ECEF X velocity [m/s]
__inline float NEOx_GetEcefVY(void) {
  return NEOx_ecefV.ecefVY / 100.0f;
} // ECEF Y velocity [m/s]
__inline float NEOx_GetEcefVZ(void) {
  return NEOx_ecefV.ecefVZ / 100.0f;
} // ECEF Z velocity [m/s]
__inline float NEOx_GetEcefVAcc(void) {
  return NEOx_ecefV.sAcc / 100.0f;
} // velocity accuracy estimate [m/s]

__inline float NEOx_GetLon(void) {
  return NEOx_llh.lon / 1e7f;
} // longitude, scaling 1e-7 [deg]
__inline float NEOx_GetLat(void) {
  return NEOx_llh.lat / 1e7f;
} // latitude, scaling 1e-7 [deg]
__inline float NEOx_GetMSL(void) {
  return NEOx_llh.hMSL / 1000.0f;
} // height above mean sea level [m]
__inline float NEOx_GetHAcc(void) {
  return NEOx_llh.hAcc / 1000.0f;
} // horizontal accuracy estimate [m]
__inline float NEOx_GetVAcc(void) {
  return NEOx_llh.vAcc / 1000.0f;
} // vertical accuracy estimate [m]

__inline float NEOx_GetVelN(void) {
  return NEOx_nedV.velN / 1000.0f;
} // north velocity component [m/s]
__inline float NEOx_GetVelE(void) {
  return NEOx_nedV.velE / 1000.0f;
} // east velocity component [m/s]
__inline float NEOx_GetVelD(void) {
  return NEOx_nedV.velD / 1000.0f;
} // down velocity component [m/s]

__inline float NEOx_GetHeading(void) {
  return (float)NEOx_nedV.heading / 1e5f;
} // heading of motion (2D), scaling 1e-5 [deg]

__inline float NEOx_GetGDOP(void) {
  return NEOx_dop.gDOP / 100.0f;
} // geometric DOP
__inline float NEOx_GetPDOP(void) {
  return NEOx_dop.pDOP / 100.0f;
} // position DOP
__inline float NEOx_GetTDOP(void) { return NEOx_dop.tDOP / 100.0f; } // time DOP
__inline float NEOx_GetVDOP(void) {
  return NEOx_dop.vDOP / 100.0f;
} // vertical DOP
__inline float NEOx_GetHDOP(void) {
  return NEOx_dop.hDOP / 100.0f;
} // horizontal DOP
__inline float NEOx_GetNDOP(void) {
  return NEOx_dop.nDOP / 100.0f;
} // northing DOP
__inline float NEOx_GetEDOP(void) {
  return NEOx_dop.eDOP / 100.0f;
} // easting DOP

__inline uint8_t NEOx_GetSV(void) {
  return NEOx_numSV;
} // number of satellites used in navigation solution

//------------------------------------------------------------------------

#ifdef _VERBOSE
static void NEOx_PrintGNSSInfo(uint8_t gnssID) {
  char str[100];
  sprintf(str,
          "\r\nGNSS-ID: %u, Reserved Tracking Channels: %u, Max. Tracking "
          "Channels: %u Flags: %u",
          gnssID, NEOx_gnssCfg[gnssID].resTrkCh, NEOx_gnssCfg[gnssID].maxTrkCh,
          NEOx_gnssCfg[gnssID].flags);
  UART_Print(VERBOSE_UART_ID, str);
}

static void NEOx_PrintSVInfo(uint8_t chn) {
  char str[100];
  sprintf(str,
          "\r\nCH: %u, SV-ID: %u, Flg: 0x%02x, Qlty: %u, CNO: %u [dBHz], "
          "(e,a): %d, %d [deg], res: %d",
          chn, NEOx_svinfo[chn].svid, NEOx_svinfo[chn].flags,
          NEOx_svinfo[chn].quality, NEOx_svinfo[chn].cno, NEOx_svinfo[chn].elev,
          NEOx_svinfo[chn].azim, NEOx_svinfo[chn].prRes);
  UART_Print(VERBOSE_UART_ID, str);
  // Note: CNO should be greater than 40, ref:
  // http://www.micro-modular.com/docs/AddlRsrc/RF-Antenna_DG_100714.pdf
}
#endif

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

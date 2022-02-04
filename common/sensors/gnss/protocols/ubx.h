/**
 * @file    ubx.h
 * @author  Raymond Oung
 * @date    2013.11.06
 * @brief   ublox UBX binary protocol (version 14) functions
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
#ifndef __UBX_H
#define __UBX_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/** @addtogroup Source
 * @{
 */

/** @addtogroup Low_Level
 * @{
 */

/** @addtogroup NEOx
 * @{
 */

// UBX PACKET OVERHEAD
#define UBX_SYNC_CHAR_1 0xB5
#define UBX_SYNC_CHAR_2 0x62
#define UBX_SYNC_SIZE 2u
#define UBX_CLASS_SIZE 1u
#define UBX_ID_SIZE 1u
#define UBX_LENGTH_SIZE 2u
#define UBX_HDR_SIZE                                                           \
  (UBX_SYNC_SIZE + UBX_CLASS_SIZE + UBX_ID_SIZE + UBX_LENGTH_SIZE)
#define UBX_CHECKSUM_SIZE 2u
#define UBX_MAX_DATA_SIZE 128u                          // maximum data length
#define UBX_OVERHEAD (UBX_HDR_SIZE + UBX_CHECKSUM_SIZE) // total overhead

// CLASSES
#define UBX_CLASS_NAV                                                          \
  0x01 // Navigation Results: Position, Speed, Time, Acc, Heading, DOP, SVs used
#define UBX_CLASS_RXM                                                          \
  0x02 // Receiver Manager Messages: Satellite Status, RTC Status
#define UBX_CLASS_INF                                                          \
  0x04 // Information Messages: Printf-Style Messages, with IDs such as Error,
       // Warning, Notice
#define UBX_CLASS_ACK 0x05 // ACK/NAK Messages: as replies to CFG Input Messages
#define UBX_CLASS_CFG                                                          \
  0x06 // Configuration input messages: Set Dynamic Model, Set DOP Mask, Set
       // Baud Rate, etc.
#define UBX_CLASS_MON                                                          \
  0x0A // Monitoring Messages: Communication Status, CPU Load, Stack Usage, Task
       // Status
#define UBX_CLASS_AID                                                          \
  0x0B // AssistNow Aiding Messages: Ephemeris, Almanac, other A-GPS data input
#define UBX_CLASS_TIM                                                          \
  0x0D // Timing Messages: Time Pulse Output, Timemark Results
#define UBX_CLASS_LOG                                                          \
  0x21 // Logging Messages: Log creation, deletion, info and retrieval

// NAV IDs
#define UBX_ID_NAV_POSECEF 0X01
#define UBX_ID_NAV_POSLLH 0X02
#define UBX_ID_NAV_DOP 0x04
#define UBX_ID_NAV_SOL 0x06
#define UBX_ID_NAV_PVT 0x07
#define UBX_ID_NAV_VELECEF 0X11
#define UBX_ID_NAV_VELNED 0X12
#define UBX_ID_NAV_SVINFO 0x30

// CFG IDs
#define UBX_ID_CFG_PRT 0x00
#define UBX_ID_CFG_MSG 0x01
#define UBX_ID_CFG_RST 0x04
#define UBX_ID_CFG_RATE 0x08
#define UBX_ID_CFG_CFG 0x09
#define UBX_ID_CFG_SBAS 0x16
#define UBX_ID_CFG_NAVX5 0x23
#define UBX_ID_CFG_NAV5 0x24
#define UBX_ID_CFG_GNSS 0x3E

// ACK IDs
#define UBX_ID_ACK_NAK 0x00
#define UBX_ID_ACK_ACK 0x01

//---------------------------------------------------------------------------------------------
// generic UBX packet
typedef struct _UBX_pkt_t {
  uint8_t sync_char_1;
  uint8_t sync_char_2;
  uint8_t msg_class;
  uint8_t id;
  uint16_t length;
  uint8_t data[UBX_MAX_DATA_SIZE + UBX_CHECKSUM_SIZE];
} __attribute__((packed)) UBX_pkt_t;

//---------------------------------------------------------------------------------------------
// message acknowledged
typedef struct _UBX_ACK_ACK_t {
  uint8_t clsID; // class ID of the acknowledged message
  uint8_t msgID; // message ID of the acknowledged message
} __attribute__((packed)) UBX_ACK_ACK_t;

//---------------------------------------------------------------------------------------------
// message not-acknowledged
typedef struct _UBX_ACK_NAK_t {
  uint8_t clsID; // class ID of the acknowledged message
  uint8_t msgID; // message ID of the acknowledged message
} __attribute__((packed)) UBX_ACK_NAK_t;

//---------------------------------------------------------------------------------------------
#define CFG_PRT_DDC 0x00  // DDC (I2C Compatible)
#define CFG_PRT_UART 0x01 // UART 1
#define CFG_PRT_USB 0x03  // USB
#define CFG_PRT_SPI 0x04  // SPI

#define CFG_PRT_MODE_CHARLEN_5 0x0000
#define CFG_PRT_MODE_CHARLEN_6 0x0040
#define CFG_PRT_MODE_CHARLEN_7 0x0080
#define CFG_PRT_MODE_CHARLEN_8 0x00C0

#define CFG_PRT_MODE_PARITY_EVEN 0x0000
#define CFG_PRT_MODE_PARITY_ODD 0x0200
#define CFG_PRT_MODE_PARITY_NONE 0x0800

#define CFG_PRT_MODE_STOPBIT_1 0x0000
#define CFG_PRT_MODE_STOPBIT_1_5 0x1000
#define CFG_PRT_MODE_STOPBIT_2 0x2000
#define CFG_PRT_MODE_STOPBIT_0_5 0x3000

#define CFG_PRT_PROTO_MASK_UBX 0x01
#define CFG_PRT_PROTO_MASK_NMEA 0x02
#define CFG_PRT_PROTO_MASK_RTCM 0x04

#define CFG_PRT_EXTENDED_TX_TIMEOUT 0x02

// port configuration for UART
typedef struct _UBX_CFG_PRT_t {
  uint8_t portID;        // port identifier number
  uint8_t reserved0;     // reserved
  uint16_t txReady;      // tx ready pin configuration
  uint32_t mode;         // a bit mask describing the UART mode
  uint32_t baudrate;     // baudrate [bps]
  uint16_t inProtoMask;  // a mask describing which input protocols are active
  uint16_t outProtoMask; // a mask describing which output protocols are active
  uint16_t flags;        // flag bit mask
  uint16_t reserved5;    // always set to zero
} __attribute__((packed)) UBX_CFG_PRT_t;

//---------------------------------------------------------------------------------------------
// set message rate(s)
typedef struct _UBX_CFG_MSG_t {
  uint8_t msgClass; // message class
  uint8_t msgID;    // message identifier
  uint8_t rate;     // send rate on current port
} __attribute__((packed)) UBX_CFG_MSG_t;

//---------------------------------------------------------------------------------------------
#define CFG_RST_HOTSTART 0x0000
#define CFG_RST_WARMSTART 0x0001
#define CFG_RST_COLDSTART 0xFFFF

#define CFG_RST_HARDWARE 0x00           // hardware reset (watchdog)
#define CFG_RST_SOFTWARE 0x01           // controlled software reset
#define CFG_RST_SOFTWARE_GNSS_ONLY 0x02 // controlled software reset (GNSS only)
#define CFG_RST_HARDWARE_AFTER_SHUTDOWN                                        \
  0x04                          // hardware reset (watchdog) after shutdown
#define CFG_RST_GNSS_STOP 0x08  // controlled GNSS stop
#define CFG_RST_GNSS_START 0x09 // controlled GNSS start

// reset receiver / clear backup data structures
typedef struct _UBX_CFG_RST_t {
  uint16_t navBbrMask; // BBR sections to clear
  uint8_t resetMode;   // reset type
  uint8_t reserved1;   // reserved
} __attribute__((packed)) UBX_CFG_RST_t;

//---------------------------------------------------------------------------------------------
#define CFG_RATE_TIMEREF_UTC 0x00 // UTC alignment reference time
#define CFG_RATE_TIMEREF_GPS 0x01 // GPS alignment reference time

// set message rate(s)
typedef struct _UBX_CFG_RATE_t {
  uint16_t measRate; // measurement rate, GPS measurements are taken every
                     // measRate [msec]
  uint16_t
      navRate; // navigation rate, in number of measurement cycles; this
               // parameter cannot be changed, and must be set to 1 [cycles]
  uint16_t timeRef; // Alignment to reference time: 0 = UTC time, 1 = GPS time
} __attribute__((packed)) UBX_CFG_RATE_t;

//---------------------------------------------------------------------------------------------
#define CFG_CFG_MASK_IOPORT 0x0001   // port settings
#define CFG_CFG_MASK_MSGCONF 0x0002  // message configuration
#define CFG_CFG_MASK_INFMSG 0x0004   // INF message configuration
#define CFG_CFG_MASK_NAVCONF 0x0008  // navigation configuration
#define CFG_CFG_MASK_RXMCONF 0x0010  // receiver manager configuration
#define CFG_CFG_MASK_RINVCONF 0x0200 // remote inventory configuration
#define CFG_CFG_MASK_ANTCONF 0x0400  // antenna configuration

#define CFG_CFG_DEVBBR 0x01    // device battery backed RAM
#define CFG_CFG_DEVFLASH 0x02  // device Flash
#define CFG_CFG_DEVEEPROM 0x04 // device EEPROM
#define CFG_CFG_SPIFLASH 0x10  // device SPI Flash

// clear, save, and load configurations
typedef struct _UBX_CFG_CFG_t {
  uint32_t clearMask; // mask with configuration sub-sections to clear (i.e.
                      // load default configurations to permanent configurations
                      // in non-volatile memory)
  uint32_t saveMask;  // mask with configuration sub-section to save (i.e. save
                      // current configuration to non-volatile memory)
  uint32_t loadMask;  // mask with configuration sub-sections to load (i.e. load
                      // permanent configurations from non-volatile memory to
                      // current configurations)
  uint8_t deviceMask; // mask which selects the devices for this command
} __attribute__((packed)) UBX_CFG_CFG_t;

//---------------------------------------------------------------------------------------------
#define CFG_SBAS_MODE_ENABLED 0x01
#define CFG_SBAS_MODE_TEST 0x02

#define CFG_SBAS_USAGE_RANGE                                                   \
  0x01 // use SBAS GEOs as a ranging source (for navigation)
#define CFG_SBAS_USAGE_DIFFCORR 0x02  // use SBAS differential corrections
#define CFG_SBAS_USAGE_INTEGRITY 0x04 // use SBAS integrity information

#define CFG_SBAS_SCANMODE1_PRN_120                                             \
  0x00000001 // EGNOS; 15.5 deg W; Inmarsat 3F2 AOR-E
#define CFG_SBAS_SCANMODE1_PRN_121 0x00000002
#define CFG_SBAS_SCANMODE1_PRN_122 0x00000004
#define CFG_SBAS_SCANMODE1_PRN_123 0x00000008
#define CFG_SBAS_SCANMODE1_PRN_124 0x00000010 // EGNOS; 21.5 deg W; Artemis
#define CFG_SBAS_SCANMODE1_PRN_125 0x00000020
#define CFG_SBAS_SCANMODE1_PRN_126                                             \
  0x00000040 // EGNOS; 25 deg W; Inmarsat 3F5 IOR-W
#define CFG_SBAS_SCANMODE1_PRN_127 0x00000080 // GAGAN; 55.1 E; Inmarsat 4 F1
#define CFG_SBAS_SCANMODE1_PRN_128 0x00000100
#define CFG_SBAS_SCANMODE1_PRN_129 0x00000200 // MSAS; 140 deg E; MTSAT-1R
#define CFG_SBAS_SCANMODE1_PRN_130 0x00000400
#define CFG_SBAS_SCANMODE1_PRN_131 0x00000800
#define CFG_SBAS_SCANMODE1_PRN_132 0x00001000
#define CFG_SBAS_SCANMODE1_PRN_133 0x00002000 // WAAS; 98 deg W; AMR
#define CFG_SBAS_SCANMODE1_PRN_134 0x00004000
#define CFG_SBAS_SCANMODE1_PRN_135                                             \
  0x00008000 // WAAS; 133.1 deg W; PanAmSat Galaxy XV
#define CFG_SBAS_SCANMODE1_PRN_136 0x00010000
#define CFG_SBAS_SCANMODE1_PRN_137 0x00020000 // MSAS; 145 deg E; MTSAT-2
#define CFG_SBAS_SCANMODE1_PRN_138                                             \
  0x00040000 // WAAS; 107.3 deg W; TeleSat Anik F1R
#define CFG_SBAS_SCANMODE1_PRN_139 0x00080000
#define CFG_SBAS_SCANMODE1_PRN_140 0x00100000
#define CFG_SBAS_SCANMODE1_PRN_141 0x00200000
#define CFG_SBAS_SCANMODE1_PRN_142 0x00400000
#define CFG_SBAS_SCANMODE1_PRN_143 0x00800000
#define CFG_SBAS_SCANMODE1_PRN_144 0x01000000
#define CFG_SBAS_SCANMODE1_PRN_145 0x02000000
#define CFG_SBAS_SCANMODE1_PRN_146 0x04000000
#define CFG_SBAS_SCANMODE1_PRN_147 0x08000000
#define CFG_SBAS_SCANMODE1_PRN_148 0x10000000
#define CFG_SBAS_SCANMODE1_PRN_149 0x20000000
#define CFG_SBAS_SCANMODE1_PRN_150 0x40000000
#define CFG_SBAS_SCANMODE1_PRN_151 0x80000000

#define CFG_SBAS_SCANMODE2_PRN_152 0x00000001
#define CFG_SBAS_SCANMODE2_PRN_153 0x00000002
#define CFG_SBAS_SCANMODE2_PRN_154 0x00000004
#define CFG_SBAS_SCANMODE2_PRN_155 0x00000008
#define CFG_SBAS_SCANMODE2_PRN_156 0x00000010
#define CFG_SBAS_SCANMODE2_PRN_157 0x00000020
#define CFG_SBAS_SCANMODE2_PRN_158 0x00000040

// SBAS configuration
typedef struct _UBX_CFG_SBAS_t {
  uint8_t mode;    // SBAS mode
  uint8_t usage;   // SBAS usage
  uint8_t maxSBAS; // maximum number of SBAS prioritzed tracking channels (valid
                   // range: 0 - 3) to use (obsolete and superseeded by
                   // UBX-CFG-GNSS in protocol versions 14.00+)
  uint32_t
      scanmode1; // which SBAS PRN numbers to search for; if all bits are set to
                 // zero, auto-scan (i.e. all valid PRNs) are searched
  uint8_t scanmode2; // continuation of scanmode bitmask
} __attribute__((packed)) UBX_CFG_SBAS_t;

//---------------------------------------------------------------------------------------------
#define UBX_CFG_NAVX5_MASK_MINMAX 0x0004       // apply min/max SVs settings
#define UBX_CFG_NAVX5_MASK_MINCNO 0x0008       // apply minimum C/N0 settings
#define UBX_CFG_NAVX5_MASK_INITIAL3DFIX 0x0040 // apply initial 3D fix settings
#define UBX_CFG_NAVX5_MASK_WKNROLL                                             \
  0x0200                              // apply GPS week number rollover settings
#define UBX_CFG_NAVX5_MASK_PPP 0x2000 // apply PPP flag
#define UBX_CFG_NAVX5_MASK_AOP                                                 \
  0x4000 // apply use AOP flag and aopOrbMaxErr setting (AssistNow Autonomous)

// navigation expert settings
typedef struct _UBX_CFG_NAVX5_t {
  uint16_t version; // message version (0 for this version)
  uint16_t mask1; // first parameters bitmask; only the flagged parameters will
                  // be applied, unused bits must be set to 0.
  uint32_t reserved_0; // always set to zero
  uint8_t reserved_1;  // always set to zero
  uint8_t reserved_2;  // always set to zero
  uint8_t minSVs;      // minimum number of satellites for navigation [#SVs]
  uint8_t maxSVs;      // maximum number of satellites for navigation [#SVs]
  uint8_t minCNO;      // minimum satellite signal level for navigation [dBHz]
  uint8_t reserved_5;  // always set to zero
  uint8_t iniFix3D;    // initial Fix must be 3D flag (0=false/1=true)
  uint8_t reserved_6;  // always set to zero
  uint8_t reserved_7;  // always set to zero
  uint8_t reserved_8;  // always set to zero
  uint16_t
      wknRollover; // GPS week rollover number; GPS week numbers will be set
                   // correctly from this week up to 1024 weeks after this week.
                   // Setting this to 0 reverts to firmware default.
  uint32_t reserved_9;   // always set to zero
  uint8_t reserved_10;   // always set to zero
  uint8_t reserved_11;   // always set to zero
  uint8_t usePPP;        // use Precise Point Positioning flag (0=false/1=true)
  uint8_t aopCfg;        // assitNow Autonomous configuration
  uint8_t reserved_12;   // always set to zero
  uint8_t reserved_13;   // always set to zero
  uint16_t aopOrbMaxErr; // maximum acceptable (modelled) AssistNow Autonomous
                         // orbit error [m] [5..1000]
  uint8_t reserved_14;   // always set to zero
  uint8_t reserved_15;   // always set to zero
  uint16_t reserved_3;   // always set to zero
  uint32_t reserved_4;   // always set to zero
} __attribute__((packed)) UBX_CFG_NAVX5_t;

//---------------------------------------------------------------------------------------------
#define UBX_CFG_NAV5_MASK_DYN 0x0001        // apply dynamic model settings
#define UBX_CFG_NAV5_MASK_MINEL 0x0002      // apply minimum elevation settings
#define UBX_CFG_NAV5_MASK_POSFIXMODE 0x0004 // apply fix mode settings
#define UBX_CFG_NAV5_MASK_DRLIM 0x0008      // reserved
#define UBX_CFG_NAV5_MASK_POSMASK 0x0010    // apply position mask settings
#define UBX_CFG_NAV5_MASK_TIMEMASK 0x0020   // apply time mask settings
#define UBX_CFG_NAV5_MASK_STATICHOLDMASK 0x0040 // apply static hold settings
#define UBX_CFG_NAV5_MASK_DGPSMASK 0x0080       // apply DGPS settings

#define UBX_CFG_NAV5_DYN_MODE_PORTABLE 0u
#define UBX_CFG_NAV5_DYN_MODE_STATIONARY 2u
#define UBX_CFG_NAV5_DYN_MODE_PEDESTRIAN 3u
#define UBX_CFG_NAV5_DYN_MODE_AUTOMOTIVE 4u
#define UBX_CFG_NAV5_DYN_MODE_SEA 5u
#define UBX_CFG_NAV5_DYN_MODE_AIRBORNE_1G 6u
#define UBX_CFG_NAV5_DYN_MODE_AIRBORNE_2G 7u
#define UBX_CFG_NAV5_DYN_MODE_AIRBORNE_4G 8u

#define UBX_CFG_NAV5_FIX_MODE_2D 1u
#define UBX_CFG_NAV5_FIX_MODE_3D 2u
#define UBX_CFG_NAV5_FIX_MODE_AUTO 3u

#define UBX_CFG_NAV5_MIN_ELEV_DEFAULT                                          \
  5 // minimum elevation for a GNSS satellite to be used in NAV [deg]

// navigation engine settings
typedef struct _UBX_CFG_NAV5_t {
  uint16_t
      mask; // parameters bitmask. only the masked parameters will be applied
  uint8_t dynMode1;     // dynamic platform model
  uint8_t fixMode;      // position fixing mode
  int32_t fixedAlt;     // fixed altitude [m]
  uint32_t fixedAltVar; // fixed altitude variance for 2D mode [m^2]
  int8_t
      minElev; // minimum elevation for a GNSS satellite to be used in NAV [deg]
  uint8_t drLimit;          // reserved [sec]
  uint16_t pDop;            // position DOP mask to use
  uint16_t tDop;            // time DOP mask to use
  uint16_t pAcc;            // position accuracy mask [m]
  uint16_t tAcc;            // time accuracy mask [m]
  uint8_t staticHoldThresh; // static hold threshold [cm/s]
  uint8_t dgpsTimeOut;      // DGPS timeout [sec]
  uint8_t cnoThreshNumSVs;  // number of satellites required to have C/N0 above
                            // cnoThres for a fix to be attempted
  uint8_t cnoThresh;  // C/N0 threshold for decideding whether to attempt a fix
                      // [dBHz]
  uint16_t reserved2; // always set to zero
  uint32_t reserved3; // always set to zero
  uint32_t reserved4; // always set to zero
} __attribute__((packed)) UBX_CFG_NAV5_t;

//---------------------------------------------------------------------------------------------
#define CFG_GNSS_ID_GPS 0
#define CFG_GNSS_ID_SBAS 1
#define CFG_GNSS_ID_QZSS 5
#define CFG_GNSS_ID_GLONASS 6

#define CFG_GNSS_FLAG_EN 0x01

// GNSS system configuration
// Note: Due to repeated block, memory buffer for this packet needs to be
// allocated dynamically
typedef struct _UBX_CFG_GNSS_configBlock_t {
  uint8_t gnssID;    // GNSS identifier
  uint8_t resTrkCh;  // number of reserved (minimum) tracking channels for this
                     // GNSS system
  uint8_t maxTrkCh;  // maximum number of tracking channels used for this GNSS
                     // system (>=resTrkChn)
  uint8_t reserved1; // reserved
  uint32_t flags;    // bitfield of flags
} __attribute__((packed)) UBX_CFG_GNSS_configBlock_t;

typedef struct _UBX_CFG_GNSS_t {
  uint8_t msgVer;     // message version (=0 for this version)
  uint8_t numTrkChHw; // number of tracking channels available in hardware (read
                      // only)
  uint8_t numTrkChUse;     // number of tracking channels to use (<= numTrkChHw)
  uint8_t numConfigBlocks; // number of configuration blocks following
} __attribute__((packed)) UBX_CFG_GNSS_t;

//---------------------------------------------------------------------------------------------
// dilution of precision
typedef struct _UBX_NAV_DOP_t {
  uint32_t iTOW; // GPS time of week of the navigation epoch [msec]
  uint16_t gDOP; // geometric DOP
  uint16_t pDOP; // position DOP
  uint16_t tDOP; // time DOP
  uint16_t vDOP; // vertical DOP
  uint16_t hDOP; // horizontal DOP
  uint16_t nDOP; // northing DOP
  uint16_t eDOP; // easting DOP
} __attribute__((packed)) UBX_NAV_DOP_t;

//---------------------------------------------------------------------------------------------
// position solution in ECEF
typedef struct _UBX_NAV_POSECEF_t {
  uint32_t iTOW; // GPS time of week of the navigation epoch [msec]
  int32_t ecefX; // ECEF X coordinate [cm]
  int32_t ecefY; // ECEF Y coordinate [cm]
  int32_t ecefZ; // ECEF Z coordinate [cm]
  uint32_t pAcc; // position accuracy estimate [cm]
} __attribute__((packed)) UBX_NAV_POSECEF_t;

//---------------------------------------------------------------------------------------------
// geodetic position solution
typedef struct _UBX_NAV_POSLLH_t {
  uint32_t iTOW;  // GPS time of week of the navigation epoch [msec]
  int32_t lon;    // longitude [deg]
  int32_t lat;    // lattitude [deg]
  int32_t height; // height above ellipsoid [mm]
  uint32_t hMSL;  // height above mean sea level [mm]
  uint32_t hAcc;  // horizontal accuracy estimate [mm]
  uint32_t vAcc;  // vertical accuracy estimate [mm]
} __attribute__((packed)) UBX_NAV_POSLLH_t;

//---------------------------------------------------------------------------------------------
#define NAV_PVT_FIX_TYPE_NO_FIX 0x00
#define NAV_PVT_FIX_TYPE_DEAD_RECKONING_ONLY 0x01
#define NAV_PVT_FIX_TYPE_2D_FIX 0x02
#define NAV_PVT_FIX_TYPE_3D_FIX 0x03
#define NAV_PVT_FIX_TYPE_GNSS_DEAD_RECKONKING 0x04
#define NAV_PVT_FIX_TYPE_TIME_ONLY 0x05

#define NAV_PVT_VALID_VALIDDATE 0X01 // 1 = valid UTC date
#define NAV_PVT_VALID_VALIDTIME 0x02 // 1 = valid UTC time of day
#define NAV_PVT_VALID_FULLYRESOLVED                                            \
  0X04 // 1 = UTC time of day has been fully resolved (no seconds uncertainty)

#define NAV_PVT_FLAGS_GNSSFIXOK                                                \
  0X01 // a valid fix (i.e. within DOP and accuracy masks)
#define NAV_PVT_FLAGS_DIFFSOLN                                                 \
  0x02 // 1 = if differential corrections were applied
#define NAV_PVT_FLAGS_PSMSTATE_NONE 0x00 // no PSM is active
#define NAV_PVT_FLAGS_PSMSTATE_ENABLED                                         \
  0x04 // ENABLED (an intermediate state before ACQUISITION state)
#define NAV_PVT_FLAGS_PSMSTATE_ACQUISITION 0x08 // ACQUISITION
#define NAV_PVT_FLAGS_PSMSTATE_TRACKING 0x0C    // TRACKING
#define NAV_PVT_FLAGS_PSMSTATE_POWER_OPTIMIZED_TRACKING                        \
  0x10                                       // POWER OPTIMIZED TRACKING
#define NAV_PVT_FLAGS_PSMSTATE_INACTIVE 0x14 // INACTIVE

// navigation position velocity time solution
typedef struct _UBX_NAV_PVT_t {
  uint32_t iTOW;      // GPS time of week of the navigation epoch [msec]
  uint16_t year;      // year (UTC) [year]
  uint8_t month;      // month, range 1..12 (UTC) [month]
  uint8_t day;        // day of month, range 1..31 (UTC) [day]
  uint8_t hour;       // hour of day, range 0..23 (UTC) [hour]
  uint8_t min;        // minute of hour, range 0..59 (UTC) [min]
  uint8_t sec;        // seconds of minute, range 0..60 (UTC) [sec]
  uint8_t valid;      // validity flags
  uint32_t tAcc;      // time accuracy estimate (UTC) [nsec]
  int32_t nano;       // fraction of second, range -1e9..+1e9 (UTC) [nsec]
  uint8_t fixType;    // GNSS fix type, range 0..5
  uint8_t flags;      // fix status flags
  uint8_t reserved1;  // reserved
  uint8_t numSV;      // number of satellites used in Nav Solution
  int32_t lon;        // longitude [deg]
  int32_t lat;        // latitude [deg]
  int32_t height;     // height above ellipsoid [mm]
  int32_t hMSL;       // height above mean sea level [mm]
  uint32_t hAcc;      // horizontal accuracy estimate [mm]
  uint32_t vAcc;      // vertical accuracy estimate [mm]
  int32_t velN;       // NED north velocity [mm/s]
  int32_t velE;       // NED east velocity [mm/s]
  int32_t velD;       // NED down velocity [mm/s]
  int32_t gspeed;     // ground speed 2D [mm/s]
  int32_t heading;    // heading of motion 2D [deg]
  uint32_t sAcc;      // speed accuracy estimate [mm/s]
  uint32_t cAcc;      // course / heading accuracy estimate [deg]
  uint16_t pDOP;      // position DOP
  uint16_t reserved2; // reserved
  uint32_t reserved3; // reserved
} __attribute__((packed)) UBX_NAV_PVT_t;

//---------------------------------------------------------------------------------------------
#define NAV_SOL_FLAGS_GPSFIXOK                                                 \
  0x01 // >1 = fix within limits (e.g. DOP and accuracy)
#define NAV_SOL_FLAGS_DIFFSOLN 0x02 // 1 = DGPS used
#define NAV_SOL_FLAGS_WKNSET 0x04   // 1 = valid GPS week number
#define NAV_SOL_FLAGS_TOWSET 0x08 // 1 = valid GPS time of week (iTOW and fTOW)

// navigation solution information
typedef struct _UBX_NAV_SOL_t {
  uint32_t iTOW; // GPS time of week [msec]
  int32_t fTOW; // fractional remainder of rounded iTOW [nsec] [-500000..500000]
  int16_t week; // GPS week (GPS time)
  uint8_t
      gpsFix; // GPS fix type, [0..5] (0x00 = No Fix, 0x01 = Dead Reckoning
              // only, 0x02 = 2D-Fix, 0x03 = 3D-Fix, 0x04 = GPS + Dead Reckoning
              // Combined, 0x05 = Time only fix, 0x06..0xff: Reserved)
  uint8_t flags;      // Fix Status Flags
  int32_t ecefX;      // ECEF X-coordinate [cm]
  int32_t ecefY;      // ECEF Y-coordinate [cm]
  int32_t ecefZ;      // ECEF Z-coordinate [cm]
  uint32_t pAcc;      // 3D Position Accuracy Estimate [cm]
  int32_t ecefVX;     // ECEF X-velocity [cm/s]
  int32_t ecefVY;     // ECEF Y-velocity [cm/s]
  int32_t ecefVZ;     // ECEF Z-velocity [cm/s]
  uint32_t sAcc;      // Speed Accuracy Estimate [cm/s]
  uint16_t pDOP;      // Position DOP
  uint8_t reserved1;  // Reserved
  uint8_t numSV;      // Number of SVs used in Nav Solution
  uint32_t reserved2; // reserved
} __attribute__((packed)) UBX_NAV_SOL_t;

//---------------------------------------------------------------------------------------------
#define NAV_SVINFO_FLAGS_SVUSED 0u
#define NAV_SVINFO_FLAGS_DIFFCORR 1u
#define NAV_SVINFO_FLAGS_ORBITAVAIL 2u
#define NAV_SVINFO_FLAGS_ORBITEPH 3u
#define NAV_SVINFO_FLAGS_UNHEALTHY 4u
#define NAV_SVINFO_FLAGS_ORBITALM 5u
#define NAV_SVINFO_FLAGS_ORBITAOP 6u
#define NAV_SVINFO_FLAGS_SMOOTHED 7u

#define NAV_SVINFO_QUALITY_IDLE 0u
#define NAV_SVINFO_QUALITY_SEARCHING 1u
#define NAV_SVINFO_QUALITY_ACQUIRED 2u
#define NAV_SVINFO_QUALITY_DETECTED 3u
#define NAV_SVINFO_QUALITY_CODE_LOCK 4u
#define NAV_SVINFO_QUALITY_CODE_CARRIER_LOCK 5u

// space vehicle information
// Note: Due to repeated block, memory buffer for this packet needs to be
// allocated dynamically
typedef struct _UBX_NAV_SVINFO_repBlock_t {
  uint8_t chn;     // Channel number (255 for SVs not assigned to a channel)
  uint8_t svid;    // Satellite ID
  uint8_t flags;   // Bitmask
  uint8_t quality; // Bitfield
  uint8_t cno;     // Carrier to Noise Ratio (Signal Strength) [dBHz]
  int8_t elev;     // Elevation in integer degrees [deg]
  int16_t azim;    // Azimuth in integer degrees [deg]
  int32_t prRes;   // Pseudo range residual [cm]
} __attribute__((packed)) UBX_NAV_SVINFO_repBlock_t;

typedef struct _UBX_NAV_SVINFO_t {
  uint32_t iTOW;       // GPS time of week [msec]
  uint8_t numCh;       // Number of channels
  uint8_t globalFlags; // Bitmask
  uint16_t reserved2;  // Reserved
} __attribute__((packed)) UBX_NAV_SVINFO_t;

//---------------------------------------------------------------------------------------------
// velocity solution in ECEF
typedef struct _UBX_NAV_VELECEF_t {
  uint32_t iTOW;  // GPS time of week of the navigation epoch [msec]
  int32_t ecefVX; // ECEF X coordinate [cm/s]
  int32_t ecefVY; // ECEF Y coordinate [cm/s]
  int32_t ecefVZ; // ECEF Z coordinate [cm/s]
  uint32_t sAcc;  // velocity accuracy estimate [cm/s]
} __attribute__((packed)) UBX_NAV_VELECEF_t;

//---------------------------------------------------------------------------------------------
// velocity solution in NED
typedef struct _UBX_NAV_VELNED_t {
  uint32_t iTOW;   // GPS time of week of the navigation epoch [msec]
  int32_t velN;    // north velocity component [cm/s]
  int32_t velE;    // east velocity component [cm/s]
  int32_t velD;    // down velocity component [cm/s]
  uint32_t speed;  // speed (3D) [cm/s]
  uint32_t gspeed; // ground speed (2D) [cm/s]
  int32_t heading; // heading of motion (2D) [deg]
  uint32_t sAcc;   // speed accuracy estimate [cm/s]
  uint32_t cAcc;   // course / heading accuracy estimate [deg]
} __attribute__((packed)) UBX_NAV_VELNED_t;

//---------------------------------------------------------------------------------------------

enum { CLEAR = 0, SAVE = !CLEAR };

/** @defgroup NEOx_Exported_Functions
 * @{
 */
void UBX_SetHeader(uint8_t *buffer, uint8_t msgClass, uint8_t msgID,
                   uint16_t length);
void UBX_SetCheckSum(uint8_t *buffer, uint8_t msgLength);

uint8_t UBX_ValidCheckSum(uint8_t *buffer, uint8_t msg_length);
uint8_t UBX_ValidCheckSum2(uint8_t *buffer, uint8_t msgClass, uint8_t msgID,
                           uint16_t msgLength);
uint8_t UBX_GetPacketSize(uint8_t msgClass, uint8_t msgID, uint16_t msgLength);
/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* __NEOx_H */

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/**
 * @file    sdcard.c
 * @author  Raymond Oung
 * @date    2014.10.31
 * @brief   SD card high-level interface
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
#include <stdlib.h>
#include <string.h>

#include "diskio.h"
#include "ff.h"
#include "sdcard.h"

#ifdef _VERBOSE
#include "clock.h"
#include "uart.h"
#include <stdio.h>
#define uartID 1u
#endif

/** @defgroup SD_Card_Private_Variables
 * @{
 */
static FATFS SD_fs32;
static FIL SD_fil;
static uint8_t SD_init = 0;
static uint8_t SD_mutex = 0;

FILINFO Finfo;
#if _USE_LFN
char Lfname[512];
#endif

/**
 * @}
 */

/** @defgroup SD_Card_Private_Functions
 * @{
 */
void SD_Card_PrintResult(FRESULT res);
/**
 * @}
 */

/**
 * @brief  Initialise SD card
 * @param  None
 * @retval 0=Fail, 1=Success
 */
uint8_t SD_Card_Init(void) {
  SD_init = SD_Card_Mount();

#ifdef _VERBOSE
  SD_Card_PrintInfo();
#endif

  return SD_init;
}

/**
 * @brief  Mount SD card
 * @param  None
 * @retval 0=Fail, 1=Success
 */
uint8_t SD_Card_Mount(void) {
  if (SD_init)
    return 1u;

#ifdef _VERBOSE
  UART_Print(uartID, "Mounting volume...");
  Delay(10);
#endif

  memset(&SD_fs32, 0, sizeof(FATFS));
  FRESULT res = f_mount(0, &SD_fs32); // mount the drive

#ifdef _VERBOSE
  SD_Card_PrintResult(res);
#endif

  if (res != FR_OK)
    return 0u;

  return 1u;
}

/**
 * @brief  Unmount SD card
 * @param  None
 * @retval 0=Fail, 1=Success
 */
uint8_t SD_Card_Umount(void) {
  FRESULT res;
#ifdef _VERBOSE
  UART_Print(uartID, "Unmounting volume...");
  Delay(10);
#endif
  res = f_mount(0, NULL);
#ifdef _VERBOSE
  SD_Card_PrintResult(res);
#endif
  if (res != FR_OK)
    return 0;
  SD_init = 0;
  return 1;
}

/**
 * @brief  Open file in R/W
 *		   Create a new file if none is available
 * @param  filename Name of file to open
 * @retval 0=Fail, 1=Success
 */
static uint8_t sd_prev_open = 0u;
uint8_t SD_Card_OpenFile(const char *filename) {
  if (!SD_init)
    return 0u;

#ifdef _VERBOSE
  UART_Print(uartID, "Opening file...");
  Delay(10);
#endif

  FRESULT res =
      f_open(&SD_fil, filename, FA_CREATE_ALWAYS | FA_WRITE | FA_READ);

#ifdef _VERBOSE
  SD_Card_PrintResult(res);
#endif

  if (res != FR_OK)
    return 0u;

  if (!sd_prev_open) {
#ifdef _VERBOSE
    UART_Print(uartID, "Seeking end of file...");
    Delay(10);
#endif

    res = f_lseek(&SD_fil, f_size(&SD_fil));

#ifdef _VERBOSE
    SD_Card_PrintResult(res);
#endif

    if (res != FR_OK)
      return 0u;
    sd_prev_open = 1u;
  }

  return 1u;
}

/**
 * @brief  Close file
 * @param  None
 * @retval 0=Fail, 1=Success
 */
uint8_t SD_Card_CloseFile(void) {
  if (!SD_init)
    return 0u;

#ifdef _VERBOSE
  UART_Print(uartID, "Closing file...");
  Delay(10);
#endif

  FRESULT res = f_close(&SD_fil);

#ifdef _VERBOSE
  SD_Card_PrintResult(res);
#endif

  if (res != FR_OK)
    return 0u;
  sd_prev_open = 0u;

  return 1u;
}

/**
 * @brief  Read from SD card
 * @param  buffer Pointer to buffer
 * @param  buf_size Size of buffer
 * @retval 0=Fail, 1=Success
 */
uint16_t SD_Card_Read(void *buffer, uint16_t buf_size) {
  if (!SD_init)
    return 0u;

  uint16_t num_read = 0u;
  FRESULT res = f_read(&SD_fil, buffer, buf_size, (UINT *)&num_read);
  if (res == FR_OK)
    return num_read;

  return 0u;
}

/**
 * @brief  Write to SD card
 * @param  buffer Pointer to buffer
 * @param  buf_size Size of buffer
 * @retval 0=Fail, 1=Success
 */
uint16_t SD_Card_Write(void *buffer, uint16_t buf_size) {
  if (!SD_init)
    return 0u;
  if (SD_mutex)
    return 0u;
  SD_mutex = 1u;

  uint16_t num_written = 0u;
  FRESULT res = f_write(&SD_fil, buffer, buf_size, (UINT *)&num_written);
  if (res == FR_OK) {
    SD_mutex = 0u;
    return num_written;
  }

#ifdef _VERBOSE
  char out[80];
  sprintf(out, "ERROR: Write FRESULT: %u\r\n", res);
  UART_Print(uartID, out);
  Delay(10);
#endif

  SD_mutex = 0u;
  return 0u;
}

/**
 * @brief  Ensures that data is written to the SD Card
 * @param  None
 * @retval 0=Fail, 1=Success
 */
uint8_t SD_Card_Sync(void) {
  if (!SD_init)
    return 0u;
  if (SD_mutex)
    return 0u;
  SD_mutex = 1u;

  FRESULT res = f_sync(&SD_fil);
  if (res == FR_OK) {
    SD_mutex = 0u;
    return 1u;
  }

#ifdef _VERBOSE
  char out[80];
  sprintf(out, "ERROR: Sync returned %u\n", res);
  UART_Print(uartID, out);
  Delay(10);
#endif

  SD_mutex = 0u;
  return 0u;
}

//-----------------------------------------------------------------------------------------

#ifdef _VERBOSE
/**
 * @brief  Diagnostic tool for print SD card information
 * @param  None
 * @retval None
 */
void SD_Card_PrintInfo(void) {
#if _FS_MINIMIZE < 2
  FRESULT res;
  long p2;
  FATFS *fs32Ptr = &SD_fs32;
  res = f_getfree("", (DWORD *)&p2, &fs32Ptr);
  if (res != FR_OK)
    return;
  char out[512];
  sprintf(
      out,
      "\r\nFAT type = %u (%s)\r\nBytes/Cluster = %lu\r\nNumber of FATs = %u\r\n"
      "Root DIR entries = %u\r\nSectors/FAT = %lu\r\nNumber of clusters = "
      "%lu\r\n"
      "FAT start (lba) = %lu\r\nDIR start (lba,cluster) = %lu\r\nData start "
      "(lba) = %lu\r\n\n",
      (WORD)SD_fs32.fs_type,
      (SD_fs32.fs_type == FS_FAT12)   ? "FAT12"
      : (SD_fs32.fs_type == FS_FAT16) ? "FAT16"
                                      : "FAT32",
      (DWORD)SD_fs32.csize * 512, (WORD)SD_fs32.n_fats, SD_fs32.n_rootdir,
      SD_fs32.fsize, (DWORD)SD_fs32.n_fatent - 2, SD_fs32.fatbase,
      SD_fs32.dirbase, SD_fs32.database);
  UART_Print(uartID, out);
  Delay(10);

#endif
}

/**
 * @brief  Diagnostic tool for printing directory structure
 * @param  None
 * @retval None
 */
void SD_Card_LS(void) {
  DIR SD_dir;

#if _FS_MINIMIZE < 2
  FRESULT res;
  long p1;
  UINT s1, s2;
  res = f_opendir(&SD_dir, "");
  if (res != FR_OK)
    return;
  p1 = s1 = s2 = 0;
  char out[512];
  for (;;) {
#if _USE_LFN
    Finfo.lfname = Lfname;
    Finfo.lfsize = sizeof(Lfname);
#endif
    res = f_readdir(&SD_dir, &Finfo);
    if ((res != FR_OK) || !Finfo.fname[0])
      break;
    if (Finfo.fattrib & AM_DIR) {
      s2++;
    } else {
      s1++;
      p1 += Finfo.fsize;
    }
    sprintf(out, "%c%c%c%c%c %u/%02u/%02u %02u:%02u %9lu  %s",
            (Finfo.fattrib & AM_DIR) ? 'D' : '-',
            (Finfo.fattrib & AM_RDO) ? 'R' : '-',
            (Finfo.fattrib & AM_HID) ? 'H' : '-',
            (Finfo.fattrib & AM_SYS) ? 'S' : '-',
            (Finfo.fattrib & AM_ARC) ? 'A' : '-', (Finfo.fdate >> 9) + 1980,
            (Finfo.fdate >> 5) & 15, Finfo.fdate & 31, (Finfo.ftime >> 11),
            (Finfo.ftime >> 5) & 63, Finfo.fsize, &(Finfo.fname[0]));
    UART_Print(uartID, out);
    Delay(10);

#if _USE_LFN
    sprintf(out, "  %s\r\n", Lfname);
    UART_Print(uartID, out);
    Delay(10);
#else
    UART_Print(uartID, "\r\n");
    Delay(10);
#endif
  }
  sprintf(out, "%4u File(s),%10lu bytes total\r\n%4u SD_dir(s)", s1, p1, s2);
  UART_Print(uartID, out);
  Delay(10);

  FATFS *fs32Ptr = &SD_fs32;
  if (f_getfree("", (DWORD *)&p1, &fs32Ptr) == FR_OK) {
    sprintf(out, ", %10lu bytes free\r\n", p1 * SD_fs32.csize * 512);
    UART_Print(uartID, out);
    Delay(10);

  } else {
    UART_Print(uartID, "\r\n");
    Delay(10);
  }
#endif
}

/**
 * @brief  Private function used in _VERBOSE
 * @param  None
 * @retval None
 */
void SD_Card_PrintResult(FRESULT res) {
  if (res == FR_OK) {
    UART_Print(uartID, " success\r\n");
  } else {
    UART_Print(uartID, " error\r\n");
  }

  Delay(10);
}
#endif
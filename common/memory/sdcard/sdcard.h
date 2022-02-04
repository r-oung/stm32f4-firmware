/**
 * @file    sdcard.h
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SDCARD_H
#define __SDCARD_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"

/** @defgroup SD_Card_Exported_Functions
 * @{
 */
uint8_t SD_Card_Init(void);

uint8_t SD_Card_Mount(void);
uint8_t SD_Card_Umount(void);
uint8_t SD_Card_OpenFile(const char *filename);
uint8_t SD_Card_CloseFile(void);
uint16_t SD_Card_Read(void *buffer, uint16_t buf_size);
uint16_t SD_Card_Write(void *buffer, uint16_t buf_size);
uint8_t SD_Card_Sync(void);

void SD_Card_Write2Buffer(uint8_t *buffer, uint16_t buf_size);
void SD_Card_ISR(void);

#ifdef _VERBOSE
void SD_Card_PrintInfo(void);
void SD_Card_LS(void);
#endif
/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* __SDCARD_H */

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/****************************************************************************
 * boards/rk3576/drivers/drivers/st77916/st77916_hw.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.
 * The ASF licenses this file to you under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with
 * the License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
 * implied.  See the License for the specific language governing
 * permissions and limitations under the License.
 ****************************************************************************/

/****************************************************************************
 * ST77916/GC9B72 LCD controller hardware register definitions.
 *
 * This header collects the raw controller hardware definitions for the
 * two supported chips (ST77916 and GC9B72): command bytes, MADCTL/COLMOD
 * register bit fields, GRAM geometry and the QSPI opcodes used to
 * transport commands to the panel.  The active chip is selected at build
 * time via CONFIG_LCD_ST77916_CHIP_TYPE_*: only the initialization
 * sequence and a few per-chip values (GRAM size, QSPI opcodes) differ;
 * everything else is shared.  These are panel hardware details and are
 * kept separate from the driver implementation (st77916.c) and the public
 * driver interface (st77916.h).
 *
 ****************************************************************************/

#ifndef __BOARDS_RK3576_DRIVERS_DRIVERS_ST77916_ST77916_HW_H
#define __BOARDS_RK3576_DRIVERS_DRIVERS_ST77916_ST77916_HW_H

#include <nuttx/config.h>
#include <stddef.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if !defined(CONFIG_LCD_ST77916_CHIP_TYPE_ST77916) && \
    !defined(CONFIG_LCD_ST77916_CHIP_TYPE_GC9B72)
#warning "Controller chip is not defined, falling back to st77916"
#define CONFIG_LCD_ST77916_CHIP_TYPE_ST77916 1
#endif

/* Command definitions */

#define ST77916_CMD_NOP     0x00 /* No operation */
#define ST77916_CMD_SWRST   0x01 /* Software reset */
#define ST77916_CMD_RDDID   0x04 /* Read display ID */
#define ST77916_CMD_SLPIN   0x10 /* Sleep in */
#define ST77916_CMD_SLPOUT  0x11 /* Sleep out */
#define ST77916_CMD_NOROFF  0x12 /* Normal off */
#define ST77916_CMD_NORON   0x13 /* Normal on */
#define ST77916_CMD_INVOFF  0x20 /* Display inversion off */
#define ST77916_CMD_INVON   0x21 /* Display inversion on */
#define ST77916_CMD_DISPOFF 0x28 /* Display off */
#define ST77916_CMD_DISPON  0x29 /* Display on */
#define ST77916_CMD_CASET   0x2A /* Column address set */
#define ST77916_CMD_RASET   0x2B /* Row address set */
#define ST77916_CMD_RAMWR   0x2C /* Write data */
#define ST77916_CMD_RAMRD   0x2E /* Read data */
#define ST77916_CMD_TEOFF   0x34 /* Tearing effect pin off */
#define ST77916_CMD_TEON    0x35 /* Tearing effect pin on */
#define ST77916_CMD_MADCTL  0x36 /* Memory data access control */
#define ST77916_CMD_COLMOD  0x3A /* Interface pixel format */

/* MADCTL bit definitions */
#define ST77916_CMD_MADCTL_MY  (1 << 7) /* flip Y */
#define ST77916_CMD_MADCTL_MX  (1 << 6) /* flip X */
#define ST77916_CMD_MADCTL_MV  (1 << 5) /* swap XY */
#define ST77916_CMD_MADCTL_BGR (1 << 3) /* swap RGB to BGR */

/* COLMOD bit definitions */
#define ST77916_CMD_COLMOD_16BIT 0x65
#define ST77916_CMD_COLMOD_18BIT 0x66

/* Sleep-out settle delay in microseconds.  After SLPOUT the panel needs
 * this long before DISPON is valid.  The driver issues SLPOUT once at
 * init, records the timestamp, and waits out only the un-elapsed part of
 * this window on the first power-on.
 */

#define ST77916_SLEEPOUT_US 120000

/* Same delay expressed in nanoseconds; used to compare against a
 * clock_systime_timespec() delta whose tv_nsec field is in nanoseconds.
 */

#define ST77916_SLEEPOUT_NS (ST77916_SLEEPOUT_US * 1000)

/* GRAM (panel display memory) size in pixels.  The framebuffer display
 * area (xres x yres) is a window inside this memory located at
 * (xoffset, yoffset); it must fit entirely within the GRAM.
 */

#ifdef CONFIG_LCD_ST77916_CHIP_TYPE_ST77916
#define ST77916_GRAM_WIDTH  360
#define ST77916_GRAM_HEIGHT 390
#elif defined(CONFIG_LCD_ST77916_CHIP_TYPE_GC9B72)
#define ST77916_GRAM_WIDTH  360
#define ST77916_GRAM_HEIGHT 360
#endif /* CONFIG_LCD_ST77916_CHIP_TYPE_ST77916 */

/* QSPI opcodes used to transport LCD commands and pixel data.  The LCD
 * command byte is carried in the middle byte of a 24-bit address
 * (0x00XX00); the opcode selects the line width of the address/data
 * phases.  Naming is <opcode phase 1>-<address phase 2>-<data phase 3>
 * lines, the opcode itself is always sent single-line.
 *
 * The opcode values are per-chip (selected below with
 * CONFIG_LCD_ST77916_CHIP_TYPE_*): ST77916 and GC9B72 use different
 * numbers for the quad phases, so do not hard-code them in the driver.
 */

#ifdef CONFIG_LCD_ST77916_CHIP_TYPE_ST77916

#define ST77916_QSPI_CMD_WRITE_1_1_1 0x02
#define ST77916_QSPI_CMD_WRITE_1_1_2 0xA2
#define ST77916_QSPI_CMD_WRITE_1_1_4 0x32
#define ST77916_QSPI_CMD_WRITE_1_4_4 0x38
#define ST77916_QSPI_CMD_READ_1_1_1  0x0B
#define ST77916_QSPI_ADDRLEN         3

#elif defined(CONFIG_LCD_ST77916_CHIP_TYPE_GC9B72)

#define ST77916_QSPI_CMD_WRITE_1_1_1 0x02
/* GC9B72 does not support the 1-1-2 write mode */
#define ST77916_QSPI_CMD_WRITE_1_1_4 0x32
#define ST77916_QSPI_CMD_WRITE_1_4_4 0x12
#define ST77916_QSPI_CMD_READ_1_1_1  0x03
#define ST77916_QSPI_ADDRLEN         3

#endif

typedef struct
{
  uint8_t *data;
  size_t len;
} st77916_init_seq_entry_t;

#define _ST77916_INIT_SEQ(...)                                  \
  {                                                             \
    .data = (uint8_t[]){ __VA_ARGS__ },                         \
    .len = sizeof((uint8_t[]){ __VA_ARGS__ }) / sizeof(uint8_t) \
  }

#ifdef CONFIG_LCD_ST77916_CHIP_TYPE_ST77916

static const st77916_init_seq_entry_t g_st77916_init_seq[] = {
  _ST77916_INIT_SEQ(0xF0, 0x28),
  _ST77916_INIT_SEQ(0xF2, 0x28),
  _ST77916_INIT_SEQ(0x73, 0xF0),
  _ST77916_INIT_SEQ(0x7C, 0xD1),
  _ST77916_INIT_SEQ(0x83, 0xE0),
  _ST77916_INIT_SEQ(0x84, 0x61),
  _ST77916_INIT_SEQ(0xF2, 0x82),
  _ST77916_INIT_SEQ(0xF0, 0x00),

  _ST77916_INIT_SEQ(0xF0, 0x01),
  _ST77916_INIT_SEQ(0xF1, 0x01),
  _ST77916_INIT_SEQ(0xB0, 0x5E),
  _ST77916_INIT_SEQ(0xB1, 0x55),
  _ST77916_INIT_SEQ(0xB2, 0x24),
  _ST77916_INIT_SEQ(0xB3, 0x01),
  _ST77916_INIT_SEQ(0xB4, 0x87),
  _ST77916_INIT_SEQ(0xB5, 0x44),
  _ST77916_INIT_SEQ(0xB6, 0x8B),
  _ST77916_INIT_SEQ(0xB7, 0x40),
  _ST77916_INIT_SEQ(0xB8, 0x86),
  _ST77916_INIT_SEQ(0xB9, 0x15),
  _ST77916_INIT_SEQ(0xBA, 0x00),
  _ST77916_INIT_SEQ(0xBB, 0x08),
  _ST77916_INIT_SEQ(0xBC, 0x08),
  _ST77916_INIT_SEQ(0xBD, 0x00),
  _ST77916_INIT_SEQ(0xBE, 0x00),
  _ST77916_INIT_SEQ(0xBF, 0x07),
  _ST77916_INIT_SEQ(0xC0, 0x80),
  _ST77916_INIT_SEQ(0xC1, 0x10),
  _ST77916_INIT_SEQ(0xC2, 0x37),
  _ST77916_INIT_SEQ(0xC3, 0x80),
  _ST77916_INIT_SEQ(0xC4, 0x10),
  _ST77916_INIT_SEQ(0xC5, 0x37),
  _ST77916_INIT_SEQ(0xC6, 0xA9),
  _ST77916_INIT_SEQ(0xC7, 0x41),
  _ST77916_INIT_SEQ(0xC8, 0x01),
  _ST77916_INIT_SEQ(0xC9, 0xA9),
  _ST77916_INIT_SEQ(0xCA, 0x41),
  _ST77916_INIT_SEQ(0xCB, 0x01),
  _ST77916_INIT_SEQ(0xCC, 0x7F),
  _ST77916_INIT_SEQ(0xCD, 0x7F),
  _ST77916_INIT_SEQ(0xCE, 0xFF),
  _ST77916_INIT_SEQ(0xD0, 0x91),
  _ST77916_INIT_SEQ(0xD1, 0x68),
  _ST77916_INIT_SEQ(0xD2, 0x68),
  _ST77916_INIT_SEQ(0xF5, 0x00, 0xA5),
  _ST77916_INIT_SEQ(0xDD, 0x40),
  _ST77916_INIT_SEQ(0xDE, 0x40),
  _ST77916_INIT_SEQ(0xF1, 0x10),
  _ST77916_INIT_SEQ(0xF0, 0x00),

  _ST77916_INIT_SEQ(0xF0, 0x02),
  _ST77916_INIT_SEQ(0xE0, 0xF0, 0x10, 0x18, 0x0D, 0x0C, 0x38, 0x3E, 0x44, 0x51,
                    0x39, 0x15, 0x15, 0x30, 0x34),
  _ST77916_INIT_SEQ(0xE1, 0xF0, 0x0F, 0x17, 0x0D, 0x0B, 0x07, 0x3E, 0x33, 0x51,
                    0x39, 0x15, 0x15, 0x30, 0x34),

  _ST77916_INIT_SEQ(0xF0, 0x10),
  _ST77916_INIT_SEQ(0xF3, 0x10),
  _ST77916_INIT_SEQ(0xE0, 0x08),
  _ST77916_INIT_SEQ(0xE1, 0x00),
  _ST77916_INIT_SEQ(0xE2, 0x00),
  _ST77916_INIT_SEQ(0xE3, 0x00),
  _ST77916_INIT_SEQ(0xE4, 0xE0),
  _ST77916_INIT_SEQ(0xE5, 0x06),
  _ST77916_INIT_SEQ(0xE6, 0x21),
  _ST77916_INIT_SEQ(0xE7, 0x03),
  _ST77916_INIT_SEQ(0xE8, 0x05),
  _ST77916_INIT_SEQ(0xE9, 0x02),
  _ST77916_INIT_SEQ(0xEA, 0xE9),
  _ST77916_INIT_SEQ(0xEB, 0x00),
  _ST77916_INIT_SEQ(0xEC, 0x00),
  _ST77916_INIT_SEQ(0xED, 0x14),
  _ST77916_INIT_SEQ(0xEE, 0xFF),
  _ST77916_INIT_SEQ(0xEF, 0x00),
  _ST77916_INIT_SEQ(0xF8, 0xFF),
  _ST77916_INIT_SEQ(0xF9, 0x00),
  _ST77916_INIT_SEQ(0xFA, 0x00),
  _ST77916_INIT_SEQ(0xFB, 0x30),
  _ST77916_INIT_SEQ(0xFC, 0x00),
  _ST77916_INIT_SEQ(0xFD, 0x00),
  _ST77916_INIT_SEQ(0xFE, 0x00),
  _ST77916_INIT_SEQ(0xFF, 0x00),
  _ST77916_INIT_SEQ(0x60, 0x40),
  _ST77916_INIT_SEQ(0x61, 0x05),
  _ST77916_INIT_SEQ(0x62, 0x00),
  _ST77916_INIT_SEQ(0x63, 0x42),
  _ST77916_INIT_SEQ(0x64, 0xDA),
  _ST77916_INIT_SEQ(0x65, 0x00),
  _ST77916_INIT_SEQ(0x66, 0x00),
  _ST77916_INIT_SEQ(0x67, 0x00),
  _ST77916_INIT_SEQ(0x68, 0x00),
  _ST77916_INIT_SEQ(0x69, 0x00),
  _ST77916_INIT_SEQ(0x6A, 0x00),
  _ST77916_INIT_SEQ(0x6B, 0x00),
  _ST77916_INIT_SEQ(0x70, 0x40),
  _ST77916_INIT_SEQ(0x71, 0x04),
  _ST77916_INIT_SEQ(0x72, 0x00),
  _ST77916_INIT_SEQ(0x73, 0x42),
  _ST77916_INIT_SEQ(0x74, 0xD9),
  _ST77916_INIT_SEQ(0x75, 0x00),
  _ST77916_INIT_SEQ(0x76, 0x00),
  _ST77916_INIT_SEQ(0x77, 0x00),
  _ST77916_INIT_SEQ(0x78, 0x00),
  _ST77916_INIT_SEQ(0x79, 0x00),
  _ST77916_INIT_SEQ(0x7A, 0x00),
  _ST77916_INIT_SEQ(0x7B, 0x00),
  _ST77916_INIT_SEQ(0x80, 0x48),
  _ST77916_INIT_SEQ(0x81, 0x00),
  _ST77916_INIT_SEQ(0x82, 0x07),
  _ST77916_INIT_SEQ(0x83, 0x02),
  _ST77916_INIT_SEQ(0x84, 0xD7),
  _ST77916_INIT_SEQ(0x85, 0x04),
  _ST77916_INIT_SEQ(0x86, 0x00),
  _ST77916_INIT_SEQ(0x87, 0x00),
  _ST77916_INIT_SEQ(0x88, 0x48),
  _ST77916_INIT_SEQ(0x89, 0x00),
  _ST77916_INIT_SEQ(0x8A, 0x09),
  _ST77916_INIT_SEQ(0x8B, 0x02),
  _ST77916_INIT_SEQ(0x8C, 0xD9),
  _ST77916_INIT_SEQ(0x8D, 0x04),
  _ST77916_INIT_SEQ(0x8E, 0x00),
  _ST77916_INIT_SEQ(0x8F, 0x00),
  _ST77916_INIT_SEQ(0x90, 0x48),
  _ST77916_INIT_SEQ(0x91, 0x00),
  _ST77916_INIT_SEQ(0x92, 0x0B),
  _ST77916_INIT_SEQ(0x93, 0x02),
  _ST77916_INIT_SEQ(0x94, 0xDB),
  _ST77916_INIT_SEQ(0x95, 0x04),
  _ST77916_INIT_SEQ(0x96, 0x00),
  _ST77916_INIT_SEQ(0x97, 0x00),
  _ST77916_INIT_SEQ(0x98, 0x48),
  _ST77916_INIT_SEQ(0x99, 0x00),
  _ST77916_INIT_SEQ(0x9A, 0x0D),
  _ST77916_INIT_SEQ(0x9B, 0x02),
  _ST77916_INIT_SEQ(0x9C, 0xDD),
  _ST77916_INIT_SEQ(0x9D, 0x04),
  _ST77916_INIT_SEQ(0x9E, 0x00),
  _ST77916_INIT_SEQ(0x9F, 0x00),
  _ST77916_INIT_SEQ(0xA0, 0x48),
  _ST77916_INIT_SEQ(0xA1, 0x00),
  _ST77916_INIT_SEQ(0xA2, 0x06),
  _ST77916_INIT_SEQ(0xA3, 0x02),
  _ST77916_INIT_SEQ(0xA4, 0xD6),
  _ST77916_INIT_SEQ(0xA5, 0x04),
  _ST77916_INIT_SEQ(0xA6, 0x00),
  _ST77916_INIT_SEQ(0xA7, 0x00),
  _ST77916_INIT_SEQ(0xA8, 0x48),
  _ST77916_INIT_SEQ(0xA9, 0x00),
  _ST77916_INIT_SEQ(0xAA, 0x08),
  _ST77916_INIT_SEQ(0xAB, 0x02),
  _ST77916_INIT_SEQ(0xAC, 0xD8),
  _ST77916_INIT_SEQ(0xAD, 0x04),
  _ST77916_INIT_SEQ(0xAE, 0x00),
  _ST77916_INIT_SEQ(0xAF, 0x00),
  _ST77916_INIT_SEQ(0xB0, 0x48),
  _ST77916_INIT_SEQ(0xB1, 0x00),
  _ST77916_INIT_SEQ(0xB2, 0x0A),
  _ST77916_INIT_SEQ(0xB3, 0x02),
  _ST77916_INIT_SEQ(0xB4, 0xDA),
  _ST77916_INIT_SEQ(0xB5, 0x04),
  _ST77916_INIT_SEQ(0xB6, 0x00),
  _ST77916_INIT_SEQ(0xB7, 0x00),
  _ST77916_INIT_SEQ(0xB8, 0x48),
  _ST77916_INIT_SEQ(0xB9, 0x00),
  _ST77916_INIT_SEQ(0xBA, 0x0C),
  _ST77916_INIT_SEQ(0xBB, 0x02),
  _ST77916_INIT_SEQ(0xBC, 0xDC),
  _ST77916_INIT_SEQ(0xBD, 0x04),
  _ST77916_INIT_SEQ(0xBE, 0x00),
  _ST77916_INIT_SEQ(0xBF, 0x00),
  _ST77916_INIT_SEQ(0xC0, 0x10),
  _ST77916_INIT_SEQ(0xC1, 0x47),
  _ST77916_INIT_SEQ(0xC2, 0x56),
  _ST77916_INIT_SEQ(0xC3, 0x65),
  _ST77916_INIT_SEQ(0xC4, 0x74),
  _ST77916_INIT_SEQ(0xC5, 0x88),
  _ST77916_INIT_SEQ(0xC6, 0x99),
  _ST77916_INIT_SEQ(0xC7, 0x01),
  _ST77916_INIT_SEQ(0xC8, 0xBB),
  _ST77916_INIT_SEQ(0xC9, 0xAA),
  _ST77916_INIT_SEQ(0xD0, 0x10),
  _ST77916_INIT_SEQ(0xD1, 0x47),
  _ST77916_INIT_SEQ(0xD2, 0x56),
  _ST77916_INIT_SEQ(0xD3, 0x65),
  _ST77916_INIT_SEQ(0xD4, 0x74),
  _ST77916_INIT_SEQ(0xD5, 0x88),
  _ST77916_INIT_SEQ(0xD6, 0x99),
  _ST77916_INIT_SEQ(0xD7, 0x01),
  _ST77916_INIT_SEQ(0xD8, 0xBB),
  _ST77916_INIT_SEQ(0xD9, 0xAA),
  _ST77916_INIT_SEQ(0xF3, 0x01),
  _ST77916_INIT_SEQ(0xF0, 0x00)
};

#elif defined(CONFIG_LCD_ST77916_CHIP_TYPE_GC9B72)

static const st77916_init_seq_entry_t g_st77916_init_seq[] = {
  _ST77916_INIT_SEQ(0xFE),
  _ST77916_INIT_SEQ(0xEF),

  _ST77916_INIT_SEQ(0x80, 0x19),
  _ST77916_INIT_SEQ(0x82, 0x09),
  _ST77916_INIT_SEQ(0x83, 0x03),
  _ST77916_INIT_SEQ(0x88, 0x00),
  _ST77916_INIT_SEQ(0x89, 0x38),
  _ST77916_INIT_SEQ(0x8A, 0x40),
  _ST77916_INIT_SEQ(0x8B, 0x0A),
  _ST77916_INIT_SEQ(0x8C, 0x00),

  _ST77916_INIT_SEQ(0x81, 0xFF),
  _ST77916_INIT_SEQ(0x84, 0xFF),
  _ST77916_INIT_SEQ(0x85, 0xFF),
  _ST77916_INIT_SEQ(0x86, 0xFF),
  _ST77916_INIT_SEQ(0x87, 0xFF),
  _ST77916_INIT_SEQ(0x8E, 0xFF),
  _ST77916_INIT_SEQ(0x8F, 0xFF),

  _ST77916_INIT_SEQ(0x98, 0x3E),
  _ST77916_INIT_SEQ(0x99, 0x3E),

  _ST77916_INIT_SEQ(0x7D, 0x72),

  _ST77916_INIT_SEQ(0x70, 0x02, 0x03, 0x03, 0x06, 0x03, 0x03, 0x09, 0x07, 0x09,
                    0x03),

  _ST77916_INIT_SEQ(0x90, 0x06, 0x06, 0x01, 0x01),
  _ST77916_INIT_SEQ(0x93, 0x02, 0xFF, 0x00),
  _ST77916_INIT_SEQ(0xCB, 0x02),

  _ST77916_INIT_SEQ(0xFB, 0x00, 0x00),
  _ST77916_INIT_SEQ(0xF6, 0xC0),

  _ST77916_INIT_SEQ(0x6C, 0x00, 0x00, 0x22, 0x00, 0xCC, 0x04, 0x58),
  _ST77916_INIT_SEQ(0xAA, 0x0B, 0x00),
  _ST77916_INIT_SEQ(0xEC, 0x07),

  _ST77916_INIT_SEQ(0xF9, 0x40),

  _ST77916_INIT_SEQ(0xEB, 0x01, 0x67),

  _ST77916_INIT_SEQ(0x74, 0x01, 0x60, 0x00, 0x00, 0x00, 0x00),

  _ST77916_INIT_SEQ(0xB5, 0x14, 0x14, 0x14),

  _ST77916_INIT_SEQ(0x6E, 0x0B, 0x0B, 0x09, 0x09, 0x13, 0x13, 0x11, 0x11, 0x16,
                    0x15, 0x01, 0x04, 0x00, 0x0D, 0x1D, 0x00, 0x00, 0x1D, 0x0D,
                    0x00, 0x04, 0x08, 0x15, 0x16, 0x12, 0x12, 0x14, 0x14, 0x0A,
                    0x0A, 0x0C, 0x0C),

  _ST77916_INIT_SEQ(0x60, 0x38, 0x1C, 0x13, 0x56),
  _ST77916_INIT_SEQ(0x61, 0xF8, 0x0A, 0x13, 0x56),
  _ST77916_INIT_SEQ(0x62, 0xF8, 0x0B, 0x13, 0x56),
  _ST77916_INIT_SEQ(0x63, 0x38, 0x1C, 0x13, 0x56),

  _ST77916_INIT_SEQ(0x64, 0x38, 0x20, 0x72, 0xF8, 0x13, 0x56),
  _ST77916_INIT_SEQ(0x65, 0x78, 0x1A, 0x70, 0x0B, 0x56, 0x13),
  _ST77916_INIT_SEQ(0x66, 0x38, 0x24, 0x72, 0xFC, 0x13, 0x56),

  _ST77916_INIT_SEQ(0x68, 0xB3, 0x08, 0x0E, 0x08, 0x0E, 0x0A, 0x0A),
  _ST77916_INIT_SEQ(0x69, 0xB3, 0x08, 0x0E, 0x08, 0x0E, 0x0A, 0x0A),
  _ST77916_INIT_SEQ(0x6A, 0x00, 0x00),

  _ST77916_INIT_SEQ(0x7C, 0xB6, 0x29),

  _ST77916_INIT_SEQ(0xAC, 0x40),

  _ST77916_INIT_SEQ(0xC3, 0x1A),
  _ST77916_INIT_SEQ(0xC4, 0x24),
  _ST77916_INIT_SEQ(0xC9, 0x2F),

  _ST77916_INIT_SEQ(0xF0, 0x11, 0x17, 0x08, 0x06, 0x05, 0x38),
  _ST77916_INIT_SEQ(0xF1, 0x4D, 0x72, 0x72, 0x2D, 0x34, 0x8F),
  _ST77916_INIT_SEQ(0xF2, 0x11, 0x17, 0x08, 0x06, 0x05, 0x38),
  _ST77916_INIT_SEQ(0xF3, 0x4D, 0x72, 0x72, 0x2D, 0x34, 0x8F),

  _ST77916_INIT_SEQ(0xFE),
  _ST77916_INIT_SEQ(0xEE)
};

#endif /* CONFIG_LCD_ST77916_CHIP_TYPE_ST77916 */

#undef _ST77916_INIT_SEQ

#define ST77916_INIT_SEQ_LEN \
  (sizeof(g_st77916_init_seq) / sizeof(g_st77916_init_seq[0]))

#endif /* __BOARDS_RK3576_DRIVERS_DRIVERS_ST77916_ST77916_HW_H */

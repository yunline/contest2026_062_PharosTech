/****************************************************************************
 * boards/rk3576/drivers/drivers/st77916/st77916.c
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
 * ST77916/GC9B72 QSPI LCD lower-half driver.
 *
 * The two controllers share the same MIPI DCS command set, transmitted
 * over a MIPI DBI (Type C) QSPI bus; the active chip is selected at
 * build time via
 * CONFIG_LCD_ST77916_CHIP_TYPE_* and only changes the initialization
 * sequence and a few hardware details in st77916_hw.h.
 *
 * This file implements the NuttX LCD lower-half protocol described in
 * <nuttx/lcd/lcd.h>.  The driver does not own the QSPI lower-half device:
 * board init code obtains the NuttX QSPI handle and passes it in
 * via st77916_lcdinitialize().
 *
 * The NuttX upper half binds to the returned struct lcd_dev_s *:
 *   - lcd_framebuffer (CONFIG_LCD_FRAMEBUFFER)  -> /dev/fbN
 *   - lcd_dev         (CONFIG_LCD_DEV)          -> /dev/lcdN
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <sys/types.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/kmalloc.h>
#include <nuttx/lcd/lcd.h>
#include <nuttx/signal.h>
#include <nuttx/spi/qspi.h>

#include "st77916.h"
#include "st77916_hw.h"

#ifdef CONFIG_LCD_ST77916

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Power level range.  Provide a default if the board config does not
 * select one, and keep it within the range of the uint8_t power field.
 */

#if !defined(CONFIG_LCD_MAXPOWER) || CONFIG_LCD_MAXPOWER < 1
#define CONFIG_LCD_MAXPOWER 1
#endif

#if CONFIG_LCD_MAXPOWER > 255
#error "CONFIG_LCD_MAXPOWER must be <= 255 to fit in the power field"
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct st77916_lcd_dev_s
{
  /* Publicly visible lower-half LCD device (must be first) */

  struct lcd_dev_s dev;

  /* QSPI lower-half device (owned by the board, not by this driver) */

  struct qspi_dev_s *qspi;

  /* Framebuffer (logical display) geometry and pixel format, bound by
   * st77916_lcdinitialize().  xres/yres describe the framebuffer display
   * region and do NOT change with the MADCTL orientation: rotation and
   * flip only remap the framebuffer content onto the physical panel inside
   * the pixel data path, so the logical display stays upright and its
   * dimensions remain fixed regardless of how the panel presents it.
   */

  uint8_t fmt;     /* see FB_FMT_* */
  fb_coord_t xres; /* Framebuffer width in pixel columns */
  fb_coord_t yres; /* Framebuffer height in pixel rows */

  /* Panel orientation / color order, bound by st77916_lcdinitialize().
   * This is the raw ST77916 MADCTL (command 0x36) register byte that
   * controls how the framebuffer content is mapped to the physical panel.
   */

  uint8_t madctl;

  /* Offset of the display area origin in the framebuffer (GRAM)
   * coordinate system, in pixel columns/rows.  The write window is always
   * programmed in GRAM coordinates: the pixel data path adds these
   * offsets directly to the column/row addresses.  This is independent of
   * the MADCTL orientation, which only controls how the GRAM content is
   * scanned onto the physical panel; no per-orientation handling is
   * required for the offsets.
   */

  fb_coord_t xoff;
  fb_coord_t yoff;

  /* Current panel power level, 0 (off) .. CONFIG_LCD_MAXPOWER (full on).
   * Updated by st77916_setpower() and returned by st77916_getpower().
   */

  uint8_t power;

  /* Monotonic timestamp of the last ST77916_CMD_SLPOUT.  A panel needs
   * ~120 ms after sleep-out before DISPON is valid; recording the
   * timestamp lets st77916_setpower() pay that settle delay only once
   * (at init, when SLPOUT is issued) and then skip both the redundant
   * SLPOUT and the longest waits on later power-ups.
   */

  struct timespec sleepout_ts;

  /* Optional select notification callback, bound by
   * st77916_lcdinitialize().  When non-NULL it is invoked with true
   * every time the driver acquires the QSPI bus and with false right
   * before releasing it (see st77916_lock()/st77916_unlock()).
   */

  st77916_select_cb_t select_cb;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* LCD data transfer methods */

static int st77916_putrun(struct lcd_dev_s *dev, fb_coord_t row,
                          fb_coord_t col, const uint8_t *buffer,
                          size_t npixels);
static int st77916_putarea(struct lcd_dev_s *dev, fb_coord_t row_start,
                           fb_coord_t row_end, fb_coord_t col_start,
                           fb_coord_t col_end, const uint8_t *buffer,
                           fb_coord_t stride);

/* LCD configuration methods */

static int st77916_getvideoinfo(struct lcd_dev_s *dev,
                                struct fb_videoinfo_s *vinfo);
static int st77916_getplaneinfo(struct lcd_dev_s *dev, unsigned int planeno,
                                struct lcd_planeinfo_s *pinfo);

/* LCD helper functions */

static uint8_t st77916_bpp(uint8_t fmt);
static int st77916_command(struct st77916_lcd_dev_s *priv, uint8_t cmd,
                           const uint8_t *data, uint16_t data_len);
static uint8_t st77916_madctl_encode(uint8_t flags);

/* QSPI bus lock/unlock helpers (each fires the select callback) */

static void st77916_lock(struct st77916_lcd_dev_s *priv);
static void st77916_unlock(struct st77916_lcd_dev_s *priv);

/* LCD specific controls */

static int st77916_getpower(struct lcd_dev_s *dev);
static int st77916_setpower(struct lcd_dev_s *dev, int power);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: st77916_bpp
 *
 * Description:
 *   Return the number of bits per pixel for the given framebuffer pixel
 *   format.  The driver supports only RGB565 (16-bit, FB_FMT_RGB16_565)
 *   and RGB666 (24-bit, FB_FMT_RGB24); any other format returns 0 and is
 *   rejected by st77916_lcdinitialize().
 *
 ****************************************************************************/

static uint8_t st77916_bpp(uint8_t fmt)
{
  switch (fmt)
    {
      case FB_FMT_RGB16_565:
        return 16;
      case FB_FMT_RGB24:
        return 24;
      default:
        return 0;
    }
}

/****************************************************************************
 * Name: st77916_command
 *
 * Description:
 *   Send a single ST77916 LCD command over the QSPI bus.  The command is
 *   wrapped in a QSPI frame with opcode ST77916_QSPI_CMD_WRITE_1_1_1
 *   (0x02, address and data both on a single line), a 24-bit address of
 *   value 0x00XX00 (XX is the LCD command byte), and an optional payload
 *   written after the address.
 *
 *   This is a fine-grained primitive: it does NOT lock the QSPI bus.
 *   The caller MUST hold the QSPI bus lock (QSPI_LOCK(..., true)) for the
 *   whole sequence of commands so that the frames of a multi-command
 *   operation are not interleaved with transfers from other devices.
 *
 * Input Parameters:
 *   priv     - The ST77916 device private structure.
 *   cmd      - ST77916 LCD command (ST77916_CMD_*).  Must fit in one byte.
 *   data     - Optional command payload; ignored if data_len is zero.
 *   data_len - Payload length in bytes.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int st77916_command(struct st77916_lcd_dev_s *priv, uint8_t cmd,
                           const uint8_t *data, uint16_t data_len)
{
  struct qspi_cmdinfo_s cinfo;

  DEBUGASSERT(priv != NULL);
  DEBUGASSERT(priv->qspi != NULL);
  DEBUGASSERT(data_len == 0 || data != NULL);

  /* Build the QSPI command frame: 1-1-1 mode, so no quad/dual flags.
   * The caller is responsible for locking the bus.
   */

  memset(&cinfo, 0, sizeof(cinfo));
  cinfo.flags = QSPICMD_ADDRESS;
  cinfo.cmd = ST77916_QSPI_CMD_WRITE_1_1_1;
  cinfo.addrlen = ST77916_QSPI_ADDRLEN;
  cinfo.addr = (cmd << 8); /* 0x00XX00 */

  if (data_len)
    {
      cinfo.flags |= QSPICMD_WRITEDATA;
      cinfo.buffer = (FAR void *)data;
      cinfo.buflen = data_len;
    }

  return QSPI_COMMAND(priv->qspi, &cinfo);
}

/* Convenience wrapper that builds a byte array from a comma-separated
 * payload and sends it as an ST77916 command.  Variadic so that
 * multi-byte payloads (with commas) are captured correctly by the
 * preprocessor.  For commands without a payload, call ST77916_COMMAND_NOARG
 */

#define ST77916_COMMAND(priv, cmd, ...)                      \
  st77916_command((priv), (cmd), (uint8_t[]){ __VA_ARGS__ }, \
                  sizeof((uint8_t[]){ __VA_ARGS__ }));

#define ST77916_COMMAND_NOARG(priv, cmd) \
  st77916_command((priv), (cmd), NULL, 0);

/****************************************************************************
 * Name: st77916_putrun
 *
 * Description:
 *   Write a partial raster line to the LCD:
 *
 *   dev     - The lcd device
 *   row     - Starting row to write to (range: 0 <= row < yres)
 *   col     - Starting column to write to (range: 0 <= col <= xres-npixels)
 *   buffer  - The buffer containing the run to be written to the LCD
 *   npixels - The number of pixels to write to the LCD
 *             (range: 0 < npixels <= xres-col)
 *
 *   There is no dedicated single-line write command on the ST77916; the
 *   ST77916 uses the fixed CASET->RASET->RAMWR sequence for every pixel
 *   transfer.  A run is therefore implemented as a single-row putarea.
 *
 ****************************************************************************/

static int st77916_putrun(struct lcd_dev_s *dev, fb_coord_t row,
                          fb_coord_t col, const uint8_t *buffer,
                          size_t npixels)
{
  /* A run of npixels at (row, col) is a one-row area spanning
   * [col, col + npixels - 1].  stride = 0 selects the contiguous path in
   * st77916_putarea().
   */

  DEBUGASSERT(npixels > 0);

  return st77916_putarea(dev, row, row, col, col + npixels - 1, buffer, 0);
}

/****************************************************************************
 * Name: st77916_putarea
 *
 * Description:
 *   Write a rectangular area to the LCD.  The write window is programmed
 *   in GRAM coordinates: priv->xoff/yoff are added to the column/row
 *   addresses so callers only deal with framebuffer coordinates.
 *
 *   NOTE: the caller's buffer is passed straight through to the QSPI
 *   lower-half (QSPI_MEMORY) with no staging copy.  Any DMA-related
 *   buffer prerequisites (alignment, addressability) are the QSPI
 *   lower-half's responsibility, not this driver's.  stride must be 0 or
 *   equal to one tight row (row_bytes): the transfer is sent as a single
 *   contiguous block, so a stride introducing a row gap cannot be
 *   expressed without a staging copy and is rejected with -EINVAL.
 *
 ****************************************************************************/

static int st77916_putarea(struct lcd_dev_s *dev, fb_coord_t row_start,
                           fb_coord_t row_end, fb_coord_t col_start,
                           fb_coord_t col_end, const uint8_t *buffer,
                           fb_coord_t stride)
{
  struct st77916_lcd_dev_s *priv = (struct st77916_lcd_dev_s *)dev;
  struct qspi_meminfo_s meminfo;
  uint32_t rows;
  uint32_t cols;
  uint32_t bpp;
  uint32_t row_bytes;
  int ret;

  DEBUGASSERT(priv != NULL);
  DEBUGASSERT(priv->qspi != NULL);
  DEBUGASSERT(buffer != NULL);

  /* Validate the caller-supplied geometry.  row_start > row_end or
   * col_start > col_end would make the unsigned row/col counts below
   * underflow, and coordinates outside xres/yres are out of range.
   */

  if (row_start > row_end || col_start > col_end)
    {
      return -EINVAL;
    }

  if (col_end >= priv->xres || row_end >= priv->yres)
    {
      return -ERANGE;
    }

  rows = row_end - row_start + 1;
  cols = col_end - col_start + 1;
  bpp = st77916_bpp(priv->fmt);
  row_bytes = cols * (bpp >> 3);

  /* The caller's buffer is passed straight to the QSPI lower-half as one
   * contiguous block of rows * row_bytes bytes, so there is no staging to
   * absorb a row pitch.  stride must therefore be 0 or exactly row_bytes
   * (a tightly packed area); anything else cannot be represented as a
   * single contiguous transfer and is rejected.
   */

  if (stride != 0 && stride != row_bytes)
    {
      return -EINVAL;
    }

  /* Write window in GRAM coordinates: framebuffer coordinates plus the
   * panel origin offset.
   */

  st77916_lock(priv);

  ret = ST77916_COMMAND(priv, ST77916_CMD_CASET,       /* set column address */
                        (col_start + priv->xoff) >> 8, /* x start h */
                        (col_start + priv->xoff) & 0xff, /* x start l */
                        (col_end + priv->xoff) >> 8,     /* x end  h*/
                        (col_end + priv->xoff) & 0xff);  /* x end l */
  if (ret < 0)
    {
      lcderr("ERROR: Failed to set column address (CASET): %d\n", ret);
      goto xfer_err;
    }

  ret = ST77916_COMMAND(priv, ST77916_CMD_RASET,         /* set row address */
                        (row_start + priv->yoff) >> 8,   /* y start h */
                        (row_start + priv->yoff) & 0xff, /* y start l */
                        (row_end + priv->yoff) >> 8,     /* y end h */
                        (row_end + priv->yoff) & 0xff);  /* y end l */
  if (ret < 0)
    {
      lcderr("ERROR: Failed to set row address (RASET): %d\n", ret);
      goto xfer_err;
    }

  /* Data payload: the caller's pixels, passed through without staging.
   * The QSPI memory opcode (ST77916_QSPI_CMD_WRITE_1_4_4, selected per
   * chip in st77916_hw.h) sends the address single-line and the data on
   * four lines; the opcode itself stays single-line because
   * QSPIMEM_QUADIO does not affect the CMDB phase in the FSPI driver.
   */

  memset(&meminfo, 0, sizeof(meminfo));
  meminfo.flags = QSPIMEM_WRITE | QSPIMEM_QUADIO;
  meminfo.cmd = ST77916_QSPI_CMD_WRITE_1_4_4;
  meminfo.addrlen = ST77916_QSPI_ADDRLEN;
  meminfo.addr = ST77916_CMD_RAMWR << 8; /* 0x00XX00 */
  /* Write-only transfer: the QSPI lower-half reads this buffer and must
   * not modify it, so the const qualification is dropped explicitly.
   */
  meminfo.buffer = (FAR void *)buffer;
  meminfo.buflen = rows * row_bytes;

  ret = QSPI_MEMORY(priv->qspi, &meminfo);
  if (ret < 0)
    {
      lcderr("ERROR: Failed to write %u bytes of pixel data (RAMWR): %d\n",
             rows * row_bytes, ret);
      goto xfer_err;
    }

  st77916_unlock(priv);

  return OK;

xfer_err:
  st77916_unlock(priv);

  return ret;
}

/****************************************************************************
 * Name: st77916_madctl_encode
 *
 * Description:
 *   Translate the driver-interface ST77916_MADCTL_* flags into the raw
 *   ST77916 MADCTL register (command 0x36) byte.  The public flags use
 *   sequential bit positions; the hardware bit offsets are an internal
 *   detail handled here.
 *
 ****************************************************************************/

static uint8_t st77916_madctl_encode(uint8_t flags)
{
  uint8_t hw = 0;

  if ((flags & ST77916_MADCTL_MY) != 0)
    {
      hw |= ST77916_CMD_MADCTL_MY;
    }

  if ((flags & ST77916_MADCTL_MX) != 0)
    {
      hw |= ST77916_CMD_MADCTL_MX;
    }

  if ((flags & ST77916_MADCTL_MV) != 0)
    {
      hw |= ST77916_CMD_MADCTL_MV;
    }

  if ((flags & ST77916_MADCTL_BGR) != 0)
    {
      hw |= ST77916_CMD_MADCTL_BGR;
    }

  return hw;
}

/****************************************************************************
 * Name: st77916_lock
 *
 * Description:
 *   Acquire the QSPI bus lock and notify the select callback (if any)
 *   that the panel is now selected.  This is the single place where the
 *   upper layer is coupled to the QSPI mutex, so every bus acquisition
 *   in the driver funnels through it and the callback is always fired
 *   right after the lock is taken.
 *
 ****************************************************************************/

static void st77916_lock(struct st77916_lcd_dev_s *priv)
{
  QSPI_LOCK(priv->qspi, true);

  if (priv->select_cb)
    {
      priv->select_cb(true);
    }
}

/****************************************************************************
 * Name: st77916_unlock
 *
 * Description:
 *   Notify the select callback (if any) that the panel is about to be
 *   deselected, then release the QSPI bus lock.  This is the counterpart
 *   of st77916_lock(); every bus release funnels through here so the
 *   callback is always fired right before the lock is dropped.
 *
 ****************************************************************************/

static void st77916_unlock(struct st77916_lcd_dev_s *priv)
{
  if (priv->select_cb)
    {
      priv->select_cb(false);
    }

  QSPI_LOCK(priv->qspi, false);
}

/****************************************************************************
 * Name: st77916_getvideoinfo
 *
 * Description:
 *   Get information about the LCD video controller configuration.
 *
 ****************************************************************************/

static int st77916_getvideoinfo(struct lcd_dev_s *dev,
                                struct fb_videoinfo_s *vinfo)
{
  struct st77916_lcd_dev_s *priv = (struct st77916_lcd_dev_s *)dev;

  DEBUGASSERT(vinfo != NULL);

  vinfo->fmt = priv->fmt;
  vinfo->xres = priv->xres;
  vinfo->yres = priv->yres;
  vinfo->nplanes = 1;
  return OK;
}

/****************************************************************************
 * Name: st77916_getrun
 *
 * Description:
 *   Read a run of pixels from the panel GRAM.
 *
 *   This panel is write-only over the QSPI bus: no GRAM readback path is
 *   implemented.  A stub returning -ENOSYS is installed instead of a NULL
 *   callback so NuttX's lcd_dev GETRUN/GETAREA ioctls fail cleanly rather
 *   than calling through a NULL function pointer.
 *
 ****************************************************************************/

static int st77916_getrun(struct lcd_dev_s *dev, fb_coord_t row,
                          fb_coord_t col, uint8_t *buffer, size_t npixels)
{
  (void)dev;
  (void)row;
  (void)col;
  (void)buffer;
  (void)npixels;

  return -ENOSYS;
}

/****************************************************************************
 * Name: st77916_getarea
 *
 * Description:
 *   Read a rectangular area from the panel GRAM.
 *
 *   See st77916_getrun(): readback is not supported, so this returns
 *   -ENOSYS instead of letting lcd_dev emulate GETAREA via a NULL getrun.
 *
 ****************************************************************************/

static int st77916_getarea(struct lcd_dev_s *dev, fb_coord_t row_start,
                           fb_coord_t row_end, fb_coord_t col_start,
                           fb_coord_t col_end, uint8_t *buffer,
                           fb_coord_t stride)
{
  (void)dev;
  (void)row_start;
  (void)row_end;
  (void)col_start;
  (void)col_end;
  (void)buffer;
  (void)stride;

  return -ENOSYS;
}

/****************************************************************************
 * Name: st77916_getplaneinfo
 *
 * Description:
 *   Get information about the configuration of each LCD color plane.
 *
 ****************************************************************************/

static int st77916_getplaneinfo(struct lcd_dev_s *dev, unsigned int planeno,
                                struct lcd_planeinfo_s *pinfo)
{
  struct st77916_lcd_dev_s *priv = (struct st77916_lcd_dev_s *)dev;

  DEBUGASSERT(pinfo != NULL);

  if (planeno >= 1)
    {
      return -ENODEV;
    }

  pinfo->putrun = st77916_putrun;
  pinfo->putarea = st77916_putarea;
  pinfo->getrun = st77916_getrun;
  pinfo->getarea = st77916_getarea;
  pinfo->buffer = NULL;
  pinfo->bpp = st77916_bpp(priv->fmt);
  pinfo->dev = dev;
  return OK;
}

/****************************************************************************
 * Name: st77916_getpower
 *
 * Description:
 *   Get the current panel power setting (0 .. CONFIG_LCD_MAXPOWER).
 *
 ****************************************************************************/

static int st77916_getpower(struct lcd_dev_s *dev)
{
  struct st77916_lcd_dev_s *priv = (struct st77916_lcd_dev_s *)dev;

  return priv->power;
}

/****************************************************************************
 * Name: st77916_setpower
 *
 * Description:
 *   Enable/disable LCD panel power (0: full off - CONFIG_LCD_MAXPOWER:
 *   full on).  At power 0 the display is turned off; the panel is NOT put
 *   back into sleep-in mode so that a subsequent power-on is fast.
 *
 *   At power > 0 the display is turned on.  The ~120 ms sleep-out settle
 *   delay is paid exactly once: st77916_lcdinitialize() issues SLPOUT and
 *   records sleepout_ts, and here we only wait out whatever part of that
 *   window has not elapsed yet before sending DISPON.  If the window has
 *   already elapsed (or power-on happens late), DISPON is sent directly
 *   with no redundant SLPOUT and no dead wait.
 *
 *   NOTE: this fast path assumes the panel stays in sleep-out mode after
 *   init (setpower(0) no longer issues SLPIN).  If SLPIN were issued
 *   elsewhere, DISPON alone would not wake the panel.
 *
 ****************************************************************************/

static int st77916_setpower(struct lcd_dev_s *dev, int power)
{
  struct st77916_lcd_dev_s *priv = (struct st77916_lcd_dev_s *)dev;
  int ret;

  DEBUGASSERT((unsigned)power <= CONFIG_LCD_MAXPOWER);

  st77916_lock(priv);

  if (power > 0)
    {
      /* The sleep-out settle delay is paid only once, at init.  If the
       * 120 ms window from the init SLPOUT has not elapsed yet, wait out
       * the remainder (with the bus released) before DISPON; otherwise
       * send DISPON directly.
       */

      struct timespec now;
      struct timespec delta;

      clock_systime_timespec(&now);
      clock_timespec_subtract(&now, &priv->sleepout_ts, &delta);

      if (delta.tv_sec == 0 && delta.tv_nsec < ST77916_SLEEPOUT_NS)
        {
          long remain_ns = ST77916_SLEEPOUT_NS - delta.tv_nsec;
          useconds_t remain_us = (useconds_t)(remain_ns / 1000);

          st77916_unlock(priv);
          nxsig_usleep(remain_us);
          st77916_lock(priv);
        }

      ret = ST77916_COMMAND_NOARG(priv, ST77916_CMD_DISPON);
      if (ret < 0)
        {
          lcderr("ERROR: Failed to send command DISPON: %d\n", ret);
          goto xfer_err;
        }
    }
  else
    {
      ret = ST77916_COMMAND_NOARG(priv, ST77916_CMD_DISPOFF);
      if (ret < 0)
        {
          lcderr("ERROR: Failed to send command DISPOFF: %d\n", ret);
          goto xfer_err;
        }
    }

  st77916_unlock(priv);

  /* Save the new power level */

  priv->power = power > 0 ? (uint8_t)power : 0;
  return OK;

xfer_err:
  st77916_unlock(priv);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: st77916_lcdinitialize
 *
 * Description:
 *   Initialize the ST77916 LCD controller and return the lower-half LCD
 *   device.
 *
 *   COORDINATE SYSTEM: xres, yres, xoffset and yoffset are all expressed
 *   in the framebuffer (GRAM) coordinate system.  They are NOT panel
 *   coordinates after MADCTL transformation.  The write window is always
 *   programmed in GRAM coordinates; madctl only controls how the GRAM
 *   content is scanned onto the physical panel and never changes the
 *   meaning of these parameters.
 *
 * Input Parameters:
 *   qspi   - NuttX QSPI lower-half device obtained by board code
 *            (e.g. rk3576_fspi_initialize(1, 0)).
 *   fmt    - Framebuffer pixel format.  The driver supports only
 *            FB_FMT_RGB16_565 (16-bit RGB565) and FB_FMT_RGB24 (24-bit
 *            RGB666); any other format is rejected.
 *   xres   - Framebuffer width in pixel columns.  This is the logical
 *            display region and does NOT change with rotation/flip; the
 *            madctl flags only remap it onto the physical panel.
 *   yres   - Framebuffer height in pixel rows (same note as xres).
 *   xoffset - Offset of the display area origin in the framebuffer (GRAM)
 *            coordinate system, in pixel columns.  The write window is
 *            always programmed in GRAM coordinates: the driver adds this
 *            to the column address, so callers only deal with framebuffer
 *            coordinates regardless of panel orientation.
 *   yoffset - Same as xoffset, but in pixel rows (row address offset).
 *   madctl - Panel orientation and color order flags, see ST77916_MADCTL_*.
 *            These are driver-interface flags, not raw register bits.
 *   invert  - When true, the ST77916_CMD_INVON (display inversion on)
 *            command is sent during panel initialization to invert the
 *            display colors; when false the command is skipped.
 *   te_en   - When true, the panel's Tearing Effect (TE) output is
 *            enabled: ST77916_CMD_TEON is sent using the mode given by
 *            te_mode.  When false, no TE command is sent and te_mode is
 *            ignored.
 *   te_mode - TE output mode used when te_en is true, see
 *            st77916_te_mode_e.  ST77916_TE_MODE_VSYNC selects the
 *            V-Blanking-only output (one pulse per frame);
 *            ST77916_TE_MODE_HSYNC selects the V- and H-Blanking output
 *            (one pulse per scanline).
 *   select_cb - Optional select notification callback (see
 *            st77916_select_cb_t).  It is invoked with true each time the
 *            driver acquires the QSPI bus and with false right before it
 *            releases it.  May be NULL to disable the notification.
 *
 * Returned Value:
 *   A non-NULL reference to the LCD lower-half device on success; NULL on
 *   failure.
 *
 ****************************************************************************/

struct lcd_dev_s *st77916_lcdinitialize(
    struct qspi_dev_s *qspi, uint8_t fmt, fb_coord_t xres, fb_coord_t yres,
    fb_coord_t xoffset, fb_coord_t yoffset, uint8_t madctl, bool invert,
    bool te_en, enum st77916_te_mode_e te_mode, st77916_select_cb_t select_cb)
{
  struct st77916_lcd_dev_s *priv;
  size_t i;
  int ret;

  if (qspi == NULL)
    {
      lcderr("ERROR: qspi handle is NULL\n");
      return NULL;
    }

  /* The driver supports only RGB565 (16-bit) and RGB666 (24-bit). */

  if (fmt != FB_FMT_RGB16_565 && fmt != FB_FMT_RGB24)
    {
      lcderr("ERROR: unsupported pixel format 0x%02x (driver supports "
             "FB_FMT_RGB16_565 and FB_FMT_RGB24 only)\n",
             fmt);
      return NULL;
    }

  /* The display area dimensions must be non-zero, and the area must fit
   * entirely inside the GRAM at the requested offset.  The comparisons
   * are written so that no unsigned arithmetic can underflow.
   */

  if (xres == 0 || yres == 0)
    {
      lcderr("ERROR: invalid panel resolution %u x %u\n", xres, yres);
      return NULL;
    }

  if (xoffset >= ST77916_GRAM_WIDTH || xres > ST77916_GRAM_WIDTH - xoffset)
    {
      lcderr("ERROR: display area offset %u + width %u exceeds GRAM width "
             "%u\n",
             xoffset, xres, ST77916_GRAM_WIDTH);
      return NULL;
    }

  if (yoffset >= ST77916_GRAM_HEIGHT || yres > ST77916_GRAM_HEIGHT - yoffset)
    {
      lcderr("ERROR: display area offset %u + height %u exceeds GRAM "
             "height %u\n",
             yoffset, yres, ST77916_GRAM_HEIGHT);
      return NULL;
    }

  /* Allocate the device private structure.  Each call creates an
   * independent instance so that multiple panels can be driven by the
   * same driver.
   */

  priv = (struct st77916_lcd_dev_s *)kmm_zalloc(sizeof(*priv));
  if (priv == NULL)
    {
      lcderr("ERROR: Failed to allocate LCD device\n");
      return NULL;
    }

  /* Bind the board-provided QSPI lower-half device */

  priv->qspi = qspi;

  /* Store the panel geometry, pixel format and orientation flags.  The
   * driver-interface flags are translated to the hardware MADCTL byte so
   * the rest of the driver only deals with register-level values.
   */

  priv->xres = xres;
  priv->yres = yres;
  priv->fmt = fmt;
  priv->madctl = st77916_madctl_encode(madctl);

  /* Store the panel origin offset applied when programming the write
   * window (see st77916_putarea()/st77916_putrun()).
   */

  priv->xoff = xoffset;
  priv->yoff = yoffset;

  priv->power = 0;

  /* Initialize the lower-half vtable */

  priv->dev.getvideoinfo = st77916_getvideoinfo;
  priv->dev.getplaneinfo = st77916_getplaneinfo;
  priv->dev.setpower = st77916_setpower;
  priv->dev.getpower = st77916_getpower;
  priv->dev.getareaalign = NULL;
  priv->dev.getcontrast = NULL;
  priv->dev.setcontrast = NULL;
  priv->dev.getframerate = NULL;
  priv->dev.setframerate = NULL;
  priv->dev.open = NULL;
  priv->dev.close = NULL;

  /* Bind the optional select notification callback.  From here on every
   * st77916_lock()/st77916_unlock() will fire it, including the panel
   * register initialization below.
   */

  priv->select_cb = select_cb;

/* return value checking macro */
#define _CHECK_XFER_ERR(_ret)                                         \
  do                                                                  \
    {                                                                 \
      int _ret_value = (_ret);                                        \
      if (_ret_value < 0)                                             \
        {                                                             \
          lcderr("ERROR: Failed to send st77916 init sequence: %d\n", \
                 _ret_value);                                         \
          goto xfer_err;                                              \
        }                                                             \
    }                                                                 \
  while (0)

  /* Panel register initialization */

  st77916_lock(priv);

  ret = ST77916_COMMAND_NOARG(priv, ST77916_CMD_SWRST);
  _CHECK_XFER_ERR(ret);

  /* sleep with lock released */
  st77916_unlock(priv);
  nxsig_usleep(5000);
  st77916_lock(priv);

  /* send init command sequence */

  for (i = 0; i < ST77916_INIT_SEQ_LEN; i++)
    {
      st77916_init_seq_entry_t init_seq_entry = g_st77916_init_seq[i];

      DEBUGASSERT(init_seq_entry.len >= 1);

      uint8_t cmd = init_seq_entry.data[0];
      uint8_t *args = &(init_seq_entry.data[1]);
      uint8_t len = init_seq_entry.len - 1;

      ret = st77916_command(priv, cmd, args, len);
      _CHECK_XFER_ERR(ret);
    }

  /* setup registers */
  ret = ST77916_COMMAND(priv, ST77916_CMD_MADCTL, priv->madctl);
  _CHECK_XFER_ERR(ret);

  ret = ST77916_COMMAND(priv, ST77916_CMD_COLMOD,
                        fmt == FB_FMT_RGB24 ? ST77916_CMD_COLMOD_18BIT
                                            : ST77916_CMD_COLMOD_16BIT);
  _CHECK_XFER_ERR(ret);

  if (invert)
    {
      ret = ST77916_COMMAND_NOARG(priv, ST77916_CMD_INVON);
      _CHECK_XFER_ERR(ret);
    }

  /* Tearing Effect (TE) configuration
   * When enabled, turn on the TE output pin in the mode requested by the
   * caller: VSYNC (V-Blanking only, one pulse per frame) or HSYNC
   * (V- and H-Blanking, one pulse per scanline).  The enum values match
   * the SET_TEAR_ON mode byte; anything unknown falls back to VSYNC.
   */

  if (te_en)
    {
      uint8_t teon_arg;

      if (te_mode == ST77916_TE_MODE_HSYNC)
        {
          teon_arg = ST77916_TE_MODE_HSYNC;
        }
      else
        {
          teon_arg = ST77916_TE_MODE_VSYNC;
        }

      ret = ST77916_COMMAND(priv, ST77916_CMD_TEON, teon_arg);
      _CHECK_XFER_ERR(ret);
    }

  ret = ST77916_COMMAND_NOARG(priv, ST77916_CMD_SLPOUT);
  _CHECK_XFER_ERR(ret);

  clock_systime_timespec(&priv->sleepout_ts);

  st77916_unlock(priv);

  return &priv->dev;

#undef _CHECK_XFER_ERR
xfer_err:
  st77916_unlock(priv);

  kmm_free(priv);

  return NULL;
}

/****************************************************************************
 * Name: st77916_lcd_uninitialize
 *
 * Description:
 *   Uninitialize the ST77916 LCD controller and release the resources
 *   allocated by st77916_lcdinitialize().
 *
 * Input Parameters:
 *   dev - The LCD lower-half device previously returned by
 *         st77916_lcdinitialize().
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void st77916_lcduninitialize(struct lcd_dev_s *dev)
{
  struct st77916_lcd_dev_s *priv;

  DEBUGASSERT(dev != NULL);

  /* The public lcd_dev_s is the first member of the private structure */

  priv = (struct st77916_lcd_dev_s *)dev;

  /* Panel power off */

  st77916_setpower(dev, 0);

  /* Free the device private structure */

  kmm_free(priv);
}

#endif /* CONFIG_LCD_ST77916 */

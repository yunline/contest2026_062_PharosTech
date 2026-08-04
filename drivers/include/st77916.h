/****************************************************************************
 * boards/rk3576/drivers/include/st77916.h
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
 * ST77916 LCD driver public interface.
 *
 * This is the shared public-interface directory for all Nyabula custom
 * drivers (boards/rk3576/drivers/include).  Driver implementations live under
 * boards/rk3576/drivers/drivers/<driver>/ and include their public headers
 * from here.
 ****************************************************************************/

#ifndef __BOARDS_RK3576_DRIVERS_INCLUDE_ST77916_H
#define __BOARDS_RK3576_DRIVERS_INCLUDE_ST77916_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/lcd/lcd.h>
#include <nuttx/spi/qspi.h>
#include <sys/types.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Panel orientation and color order flags.  These are ORed together and
 * passed to st77916_lcdinitialize() as the madctl argument.  The bit
 * positions are driver-interface only (0..3) and are independent of the
 * underlying controller registers; the driver translates them to the
 * hardware MADCTL bits internally.
 */

#define ST77916_MADCTL_MY  (1 << 0) /* Vertical flip */
#define ST77916_MADCTL_MX  (1 << 1) /* Horizontal flip */
#define ST77916_MADCTL_MV  (1 << 2) /* Swap axes */
#define ST77916_MADCTL_BGR (1 << 3) /* Panel color order BGR */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Select notification callback.
 *
 * The driver invokes this callback every time it acquires ownership of the
 * QSPI bus (the "selected" state) so the caller can perform custom work
 * (e.g. drive a panel RESET/backlight GPIO, log timing, etc.) while the
 * panel is guaranteed to be the active QSPI device.  It is called with
 * select == true right after the bus is locked and with select == false
 * right before the bus is unlocked.  It may be NULL to disable the
 * notification.
 *
 *   select - true when the QSPI bus has just been acquired (panel is
 *            selected); false when it is about to be released (panel is
 *            deselected).
 */

typedef void (*st77916_select_cb_t)(bool select);

/* Tearing Effect (TE) output mode.  These values match the controller's
 * SET_TEAR_ON (0x35) mode parameter so they can be passed straight to the
 * panel.  Selected via the te_mode argument of st77916_lcdinitialize()
 * when te_en is true.
 *
 *   ST77916_TE_MODE_VSYNC - V-Blanking information only: the TE output
 *                           pulses once per frame (M = 0).  Use as a
 *                           frame-start / tearing reference.
 *   ST77916_TE_MODE_HSYNC - V- and H-Blanking information: the TE output
 *                           pulses once per scanline (M = 1).
 */

enum st77916_te_mode_e
{
  ST77916_TE_MODE_VSYNC = 0x00, /* V-Blanking only, one pulse per frame */
  ST77916_TE_MODE_HSYNC = 0x01  /* V- and H-Blanking, one pulse per line */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_LCD_ST77916

/****************************************************************************
 * Name: st77916_lcdinitialize
 *
 * Description:
 *   Initialize the ST77916 LCD controller and return the lower-half LCD
 *   device.  The QSPI lower-half handle is obtained and passed in by the
 *   board init code; the driver does not own the QSPI bus.
 *
 *   The returned lower-half device is bound to the NuttX LCD upper half
 *   by the board code, e.g. via lcd_framebuffer (CONFIG_LCD_FRAMEBUFFER,
 *   exposes /dev/fbN) or the lcd_dev character driver (CONFIG_LCD_DEV,
 *   exposes /dev/lcdN).
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
 *   fmt    - Framebuffer pixel format.  The ST77916 supports only
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
    bool te_en, enum st77916_te_mode_e te_mode, st77916_select_cb_t select_cb);

/****************************************************************************
 * Name: st77916_lcduninitialize
 *
 * Description:
 *   Uninitialize the ST77916 LCD controller and release the resources
 *   allocated by st77916_lcdinitialize().  This may be called to free
 *   up the instance when a panel is removed or a driver is being torn
 *   down.
 *
 * Input Parameters:
 *   dev - The LCD lower-half device previously returned by
 *         st77916_lcdinitialize().
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void st77916_lcduninitialize(struct lcd_dev_s *dev);

#endif /* CONFIG_LCD_ST77916 */

#endif /* __BOARDS_RK3576_DRIVERS_INCLUDE_ST77916_H */

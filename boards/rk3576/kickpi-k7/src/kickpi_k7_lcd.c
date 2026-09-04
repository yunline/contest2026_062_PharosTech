/****************************************************************************
 * boards/arm64/rk3576/kickpi_k7/src/kickpi_k7_lcd.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "kickpi_k7.h"
#include <nuttx/config.h>
#include <stdbool.h>

#ifdef CONFIG_KICKPI_K7_LCD

#include "rk3576_gpio.h"
#include <errno.h>
#include <nuttx/spi/qspi.h>
#include <syslog.h>

#include "rk3576_fspi.h"

#include "st77916.h"
#include <nuttx/clk/clk.h>
#include <nuttx/signal.h>

#include <nuttx/lcd/lcd_dev.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* FSPI1 pins are muxed on GPIO2_A with IOMUX alternate function AF2.  The
 * pinset macros carry only the pin identity (port + pin); the AF is applied
 * via rk3576_gpio_set_af().  NOTE: these pins share GPIO2_A with SDMMC, so
 * the SD card must be removed before any FSPI operation.
 */

#define GPIO_FSPI1_D0   (GPIO_PORT2 | GPIO_PIN_A0)
#define GPIO_FSPI1_D1   (GPIO_PORT2 | GPIO_PIN_A1)
#define GPIO_FSPI1_D2   (GPIO_PORT2 | GPIO_PIN_A2)
#define GPIO_FSPI1_D3   (GPIO_PORT2 | GPIO_PIN_A3)
#define GPIO_FSPI1_CSN0 (GPIO_PORT2 | GPIO_PIN_A4)
#define GPIO_FSPI1_CLK  (GPIO_PORT2 | GPIO_PIN_A5)

#define GPIO_LCD_SEL    (GPIO_PORT4 | GPIO_PIN_A6)
#define GPIO_LCD_RST    (GPIO_PORT4 | GPIO_PIN_A4)

/* IOMUX alternate function of the FSPI1 signal group */

#define KICKPI_K7_LCD_FSPI1_AF 2

/* Claim a pin (once) and mux it to the given IOMUX alternate function.
 * The handle is retained for the lifetime of the FSPI/LCD peripheral, so
 * it is never released here.
 */

#define _SET_GPIO_AF(pinset, af)                        \
  do                                                    \
    {                                                   \
      static FAR struct gpio_dev_s *handle;             \
      if (handle == NULL)                               \
        {                                               \
          ret = rk3576_gpio_get((pinset), &handle);     \
          if (ret < 0)                                  \
            {                                           \
              syslog(LOG_ERR,                           \
                     "ERROR: failed to get fspi gpio, " \
                     "pinset: %u\n",                    \
                     (pinset));                         \
              return ret;                               \
            }                                           \
        }                                               \
      rk3576_gpio_set_af(handle, (af));                 \
    }                                                   \
  while (0)

/* qspi frequency */
#define LCD_QSPI_FREQ 80000000

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* LCD device pointers for dual-screen support */

static FAR struct lcd_dev_s *g_lcd[2];

/* GPIO handles for the LCD panel/reset GPIOs (claimed in
 * kickpi_k7_lcd_initialize and kept for the driver lifetime). */

static FAR struct gpio_dev_s *g_lcd_sel_gpio;
static FAR struct gpio_dev_s *g_lcd_rst_gpio;

static void lcd0_select_cb(bool sel)
{
  if (sel)
    {
      rk3576_gpio_write_bit(g_lcd_sel_gpio, false);
    }
}

static void lcd1_select_cb(bool sel)
{
  if (sel)
    {
      rk3576_gpio_write_bit(g_lcd_sel_gpio, true);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: kickpi_k7_lcd_initialize
 *
 * Description:
 *   Initialize the ST77916 LCD panel over the FSPI1 CS0 (QSPI) bus.
 *   This is called from board_app_initialize() when CONFIG_KICKPI_K7_LCD
 *   is enabled.
 *
 *   NOTE: FSPI1 pins and SDMMC share GPIO2_A, so the SD card must be
 *   removed before any FSPI operation.  The GPIO configuration is safe
 *   at boot (the SDMMC driver only touches pins when a card is inserted),
 *   but the actual SPI transfers are deferred until this function is called.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

int kickpi_k7_lcd_initialize(void)
{
  FAR struct qspi_dev_s *qspi;
  FAR struct lcd_dev_s *lcd0;
  FAR struct lcd_dev_s *lcd1;
  int ret;

  /* Mux the FSPI1 data/control pins to their IOMUX alternate function
   * (AF2).  Note that FSPI1 pins share GPIO2_A with SDMMC.
   */

  _SET_GPIO_AF(GPIO_FSPI1_D0, KICKPI_K7_LCD_FSPI1_AF);
  _SET_GPIO_AF(GPIO_FSPI1_D1, KICKPI_K7_LCD_FSPI1_AF);
  _SET_GPIO_AF(GPIO_FSPI1_D2, KICKPI_K7_LCD_FSPI1_AF);
  _SET_GPIO_AF(GPIO_FSPI1_D3, KICKPI_K7_LCD_FSPI1_AF);
  _SET_GPIO_AF(GPIO_FSPI1_CSN0, KICKPI_K7_LCD_FSPI1_AF);
  _SET_GPIO_AF(GPIO_FSPI1_CLK, KICKPI_K7_LCD_FSPI1_AF);

  /* sel - panel select output.  The handle is kept for the select
   * callbacks (lcd0/lcd1_select_cb).
   */

  ret = rk3576_gpio_get(GPIO_LCD_SEL, &g_lcd_sel_gpio);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: failed to get LCD_SEL gpio: %d\n", ret);
      return ret;
    }

  rk3576_gpio_set_mode(g_lcd_sel_gpio, RK3576_GPIO_OUTPUT);

  /* rst - LCD reset output, driven high (out of reset) */

  ret = rk3576_gpio_get(GPIO_LCD_RST, &g_lcd_rst_gpio);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: failed to get LCD_RST gpio: %d\n", ret);
      return ret;
    }

  rk3576_gpio_set_mode(g_lcd_rst_gpio, RK3576_GPIO_OUTPUT);

  /* Assert reset and hold for 100 us */
  rk3576_gpio_write_bit(g_lcd_rst_gpio, false);
  nxsig_usleep(100);

  /* Deassert reset */
  rk3576_gpio_write_bit(g_lcd_rst_gpio, true);

  /* Wait for the initiallization of CPLD and LCD controller  */
  nxsig_usleep(20000);

  qspi = rk3576_fspi_initialize(1, 0);

  if (qspi == NULL)
    {
      syslog(LOG_ERR, "ST77916: rk3576_fspi_initialize() returned NULL\n");
      return -ENODEV;
    }

  QSPI_LOCK(qspi, true);
  QSPI_SETFREQUENCY(qspi, LCD_QSPI_FREQ);
  QSPI_LOCK(qspi, false);

  /* Initialize lcd0 */

  lcd0 =
      st77916_lcdinitialize(qspi, FB_FMT_RGB16_565, 360, 360, 0, 0, 0, false,
                            true, ST77916_TE_MODE_HSYNC, lcd0_select_cb);
  if (lcd0 == NULL)
    {
      syslog(LOG_ERR, "ST77916: st77916_lcdinitialize() failed\n");
      return -ENODEV;
    }
  g_lcd[0] = lcd0;

  ret = lcddev_register(0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ST77916: lcddev_register(/dev/lcd0) failed: %d\n", ret);
      return ret;
    }

  syslog(LOG_INFO, "ST77916: /dev/lcd0 registered (%p)\n", lcd0);

  /* Initialize lcd1 */

  lcd1 =
      st77916_lcdinitialize(qspi, FB_FMT_RGB16_565, 360, 360, 0, 0, 0, false,
                            true, ST77916_TE_MODE_HSYNC, lcd1_select_cb);
  if (lcd1 == NULL)
    {
      syslog(LOG_ERR, "ST77916: st77916_lcdinitialize() failed\n");
      return -ENODEV;
    }
  g_lcd[1] = lcd1;

  ret = lcddev_register(1);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ST77916: lcddev_register(/dev/lcd1) failed: %d\n", ret);
      return ret;
    }

  syslog(LOG_INFO, "ST77916: /dev/lcd1 registered (%p)\n", lcd1);

  return OK;
}

/****************************************************************************
 * Name: board_lcd_getdev
 *
 * Description:
 *   Return a reference to the LCD object for the specified LCD.
 *   This allows support for multiple LCD devices (dual-screen).
 *
 * Input Parameters:
 *   lcddev - LCD device number (0 or 1)
 *
 * Returned Value:
 *   LCD device pointer if success or NULL if failed.
 *
 ****************************************************************************/

#ifdef CONFIG_LCD
FAR struct lcd_dev_s *board_lcd_getdev(int lcddev)
{
  if (lcddev < 0 || lcddev > 1 || g_lcd[lcddev] == NULL)
    {
      syslog(LOG_ERR, "board_lcd_getdev: invalid lcddev %d\n", lcddev);
      return NULL;
    }

  return g_lcd[lcddev];
}
#endif

#endif /* CONFIG_KICKPI_K7_LCD */

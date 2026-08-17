/****************************************************************************
 * apps/graphics/nyabula_display/nyabula_display.c
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

#include "nyabula_display.h"
#include "nyabula_dual_lcd.h"

#include <lvgl/lvgl.h>
#include <nuttx/clock.h>

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static uint32_t nyabula_tick_get_ms(void);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static nyabula_dual_lcd_t *g_dual_lcd = NULL;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nyabula_tick_get_ms
 *
 * Description:
 *   Returns the current NuttX system time in milliseconds, used as the
 *   LVGL time base via lv_tick_set_cb().  Because the callback is queried
 *   lazily by LVGL, animations advance with the real system clock even
 *   when the render loop only wakes on TE edges -- no dedicated 1ms
 *   lv_tick_inc() feeder is required, and time never drifts when wake-ups
 *   are skipped.
 *
 ****************************************************************************/

static uint32_t nyabula_tick_get_ms(void)
{
  return (uint32_t)(clock_systime_ticks() * MSEC_PER_TICK);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int nyabula_display_init(const char *lcd_path0, const char *lcd_path1,
                         int width, int height)
{
  /* Initialize LVGL */
  lv_init();

  /* Bind LVGL's time base to the NuttX system clock. */
  lv_tick_set_cb(nyabula_tick_get_ms);

  /* Create dual LCD display */
  g_dual_lcd = nyabula_dual_lcd_create(lcd_path0, lcd_path1, width, height);
  if (g_dual_lcd == NULL)
    {
      LV_LOG_ERROR("Failed to create dual LCD display");
      return -1;
    }

  LV_LOG_USER("Nyabula dual display initialized successfully");
  return 0;
}

void nyabula_display_deinit(void)
{
  if (g_dual_lcd)
    {
      nyabula_dual_lcd_destroy(g_dual_lcd);
      g_dual_lcd = NULL;
    }

  lv_deinit();
  LV_LOG_USER("Nyabula display deinitialized");
}

int nyabula_display_task(void)
{
  if (!g_dual_lcd)
    {
      LV_LOG_ERROR("Dual display not initialized");
      return -1;
    }

  nyabula_dual_lcd_task(g_dual_lcd);
  return 0;
}

lv_display_t *nyabula_display_get_screen(int screen_id)
{
  if (!g_dual_lcd)
    {
      LV_LOG_ERROR("Dual display not initialized");
      return NULL;
    }

  return nyabula_dual_lcd_get_display(g_dual_lcd, screen_id);
}

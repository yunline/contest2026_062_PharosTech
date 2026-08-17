/****************************************************************************
 * apps/graphics/nyabula_display/nyabula_display.h
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

#ifndef __NYABULA_DISPLAY_H
#define __NYABULA_DISPLAY_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <lvgl/lvgl.h>

#ifdef __cplusplus
extern "C" {
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: nyabula_display_init
 *
 * Description:
 *   Initialize LVGL dual display devices with double buffering.
 *   Both screens share a single QSPI bus.
 *
 * Input Parameters:
 *   lcd_path0 - Path to first LCD device (e.g., "/dev/lcd0")
 *   lcd_path1 - Path to second LCD device (e.g., "/dev/lcd1")
 *   width     - Screen width in pixels
 *   height    - Screen height in pixels
 *
 * Returned Value:
 *   0 on success, negative value on failure.
 *
 ****************************************************************************/

int nyabula_display_init(const char *lcd_path0, const char *lcd_path1,
                         int width, int height);

/****************************************************************************
 * Name: nyabula_display_deinit
 *
 * Description:
 *   Deinitialize LVGL display and input devices.
 *
 ****************************************************************************/

void nyabula_display_deinit(void);

/****************************************************************************
 * Name: nyabula_display_task
 *
 * Description:
 *   Run one step of the display render loop from the calling thread (e.g.
 *   main).  Call this repeatedly in a loop after the UI has been built
 *   (LVGL is single-threaded).  The display pipeline (TE + transfer
 *   threads) is started by init; this only hosts lv_timer_handler().
 *
 * Returned Value:
 *   0 on success, negative value on failure.
 *
 ****************************************************************************/

int nyabula_display_task(void);

/****************************************************************************
 * Name: nyabula_display_get_screen
 *
 * Description:
 *   Get LVGL display instance for a specific screen.
 *
 * Input Parameters:
 *   screen_id - Screen index (0 or 1)
 *
 * Returned Value:
 *   Pointer to lv_display_t on success, NULL on failure.
 *
 ****************************************************************************/

lv_display_t *nyabula_display_get_screen(int screen_id);

#ifdef __cplusplus
}
#endif

#endif /* __NYABULA_DISPLAY_H */

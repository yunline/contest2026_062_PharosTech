/****************************************************************************
 * apps/graphics/nyabula_display/nyabula_dual_demo.h
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

#ifndef __NYABULA_DUAL_DEMO_H
#define __NYABULA_DUAL_DEMO_H

#include <lvgl/lvgl.h>

#ifdef __cplusplus
extern "C" {
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Build two independent, animated UIs, one per display.  Each UI is rooted
 * on its own display's active screen (obtained via
 * lv_display_get_screen_active(), NOT the global lv_screen_active()), so
 * the two screens are fully isolated from each other even though both run
 * in the same single LVGL instance.  The caller then drives rendering by
 * calling nyabula_display_task() (which runs lv_timer_handler() and the
 * per-display lv_refr_now()) in a loop. */

void nyabula_dual_demo_create(lv_display_t *disp0, lv_display_t *disp1);

#ifdef __cplusplus
}
#endif

#endif /* __NYABULA_DUAL_DEMO_H */

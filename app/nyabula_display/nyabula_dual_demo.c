/****************************************************************************
 * apps/graphics/nyabula_display/nyabula_dual_demo.c
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

#include <string.h>

#include "nyabula_dual_demo.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Screen geometry is 360 x 360 (ST77916, RGB565 on the dual-LCD rig). */

#define DEMO_SIZE 360

/* Pointer: a short "blade" arc that we spin by driving lv_arc_set_rotation
 * through the same 0..360 animation tick. */

#define DEMO_BLADE_R   100 /* Blade arc radius */
#define DEMO_BLADE_W   14  /* Blade thickness */
#define DEMO_BLADE_ARC 60  /* Blade angular width (a swept fan) */

/* Static rim ring (screen 0) */

#define DEMO_RIM_R 155

/* Bouncing ball / square margins */

#define DEMO_MARGIN  20
#define DEMO_BALL_R  22 /* Ball radius (screen 0) */
#define DEMO_BLOCK_S 34 /* Square side (screen 1) */

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct demo0_s
{
  lv_obj_t *blade; /* Rotating blade arc (screen 0) */
  lv_obj_t *ball;  /* Horizontally bouncing ball (screen 0) */
  int32_t x0;      /* Ball left x at v=0 */
  int32_t dx;      /* Ball horizontal travel across one half-period */
};

struct demo1_s
{
  lv_obj_t *blade; /* Rotating blade arc (screen 1, opposite direction) */
  lv_obj_t *block; /* Vertically bouncing square (screen 1) */
  int32_t y0;      /* Block y at v=0 */
  int32_t dy;      /* Block vertical travel across one half-period */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct demo0_s g_d0;
static struct demo1_s g_d1;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* One shared animation drives both the blade spin and the ball travel on
 * screen 0, so they stay in perfect sync.  v runs 0..360 then (playback)
 * 360..0: the blade waves back and forth through a full revolution while
 * the ball glides left-right across the panel.
 */

static void demo0_tick_anim(void *var, int32_t v)
{
  struct demo0_s *s = (struct demo0_s *)var;

  lv_arc_set_rotation(s->blade, v);
  lv_obj_set_x(s->ball, s->x0 + (s->dx * v) / 360);
}

/* Screen 1: blade spins the opposite phase, block travels vertically. */

static void demo1_tick_anim(void *var, int32_t v)
{
  struct demo1_s *s = (struct demo1_s *)var;

  lv_arc_set_rotation(s->blade, (360 - v) % 360);
  lv_obj_set_y(s->block, s->y0 - (s->dy * v) / 360);
}

/* Build a "blade" arc: a short, thick segment with no background so only
 * the indicator (the blade) is drawn; rotation spins it like a fan blade.
 */

static lv_obj_t *blade_create(lv_obj_t *scr, lv_color_t color)
{
  lv_obj_t *b;
  lv_obj_t *hub;

  b = lv_arc_create(scr);
  lv_obj_center(b);
  lv_obj_set_size(b, DEMO_BLADE_R * 2, DEMO_BLADE_R * 2);
  lv_arc_set_rotation(b, 0);
  lv_arc_set_range(b, 0, 360);
  lv_arc_set_bg_angles(b, 0, 0); /* no bg ring */
  lv_arc_set_start_angle(b, 0);
  lv_arc_set_end_angle(b, DEMO_BLADE_ARC); /* the blade */

  lv_obj_set_style_arc_width(b, DEMO_BLADE_W, LV_PART_INDICATOR);
  lv_obj_set_style_arc_color(b, color, LV_PART_INDICATOR);
  lv_obj_set_style_arc_opa(b, LV_OPA_COVER, LV_PART_INDICATOR);
  lv_obj_set_style_arc_rounded(b, true, LV_PART_INDICATOR);

  /* Small hub dot at the centre. */
  hub = lv_obj_create(scr);
  lv_obj_set_size(hub, 16, 16);
  lv_obj_set_style_radius(hub, LV_RADIUS_CIRCLE, 0);
  lv_obj_set_style_bg_color(hub, color, 0);
  lv_obj_set_style_bg_opa(hub, LV_OPA_COVER, 0);
  lv_obj_set_style_border_width(hub, 0, 0);
  lv_obj_center(hub);

  return b;
}

/****************************************************************************
 * Screen 0  --  spinning fan blade + horizontally bouncing ball
 ****************************************************************************/

static void demo0_build(lv_obj_t *scr)
{
  lv_anim_t a;
  lv_obj_t *rim;
  lv_obj_t *ball;

  /* Static rim ring. */
  rim = lv_arc_create(scr);
  lv_obj_center(rim);
  lv_obj_set_size(rim, DEMO_RIM_R * 2, DEMO_RIM_R * 2);
  lv_arc_set_bg_angles(rim, 0, 360);
  lv_arc_set_rotation(rim, 90);
  lv_obj_set_style_arc_width(rim, 3, LV_PART_MAIN); /* bg */
  lv_obj_set_style_arc_color(rim, lv_color_hex(0x2a3a4a), LV_PART_MAIN);
  lv_obj_set_style_arc_opa(rim, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_set_style_arc_width(rim, DEMO_BLADE_W + 4, LV_PART_INDICATOR);
  lv_obj_set_style_arc_color(rim, lv_color_hex(0x1c2733), LV_PART_INDICATOR);

  g_d0.blade = blade_create(scr, lv_color_hex(0x29b6f6));

  /* Ball: from top-left, across the width. */
  ball = lv_obj_create(scr);
  lv_obj_set_size(ball, DEMO_BALL_R * 2, DEMO_BALL_R * 2);
  lv_obj_set_style_radius(ball, LV_RADIUS_CIRCLE, 0);
  lv_obj_set_style_bg_color(ball, lv_color_hex(0xff5252), 0);
  lv_obj_set_style_bg_opa(ball, LV_OPA_COVER, 0);
  lv_obj_set_style_border_width(ball, 0, 0);
  lv_obj_align(ball, LV_ALIGN_TOP_LEFT, DEMO_MARGIN, DEMO_MARGIN);
  g_d0.ball = ball;
  g_d0.x0 = DEMO_MARGIN;
  g_d0.dx = DEMO_SIZE - 2 * DEMO_MARGIN - 2 * DEMO_BALL_R;

  lv_anim_init(&a);
  lv_anim_set_var(&a, &g_d0);
  lv_anim_set_exec_cb(&a, demo0_tick_anim);
  lv_anim_set_values(&a, 0, 360);
  lv_anim_set_duration(&a, 1200);
  lv_anim_set_playback_duration(&a, 1200); /* 0..360..0 => waves */
  lv_anim_set_repeat_count(&a, LV_ANIM_REPEAT_INFINITE);
  lv_anim_start(&a);
}

/****************************************************************************
 * Screen 1  --  counter-spinning fan + vertically bouncing square
 ****************************************************************************/

static void demo1_build(lv_obj_t *scr)
{
  lv_anim_t a;
  lv_obj_t *block;

  g_d1.blade = blade_create(scr, lv_color_hex(0x69f0ae));

  /* Square: from bottom-left, up the height. */
  block = lv_obj_create(scr);
  lv_obj_set_size(block, DEMO_BLOCK_S, DEMO_BLOCK_S);
  lv_obj_set_style_radius(block, 6, 0);
  lv_obj_set_style_bg_color(block, lv_color_hex(0xffb300), 0);
  lv_obj_set_style_bg_opa(block, LV_OPA_COVER, 0);
  lv_obj_set_style_border_width(block, 0, 0);
  lv_obj_align(block, LV_ALIGN_BOTTOM_LEFT, DEMO_MARGIN, -DEMO_MARGIN);
  g_d1.block = block;
  g_d1.y0 = DEMO_SIZE - DEMO_MARGIN - DEMO_BLOCK_S;
  g_d1.dy = DEMO_SIZE - 2 * DEMO_MARGIN - DEMO_BLOCK_S;

  lv_anim_init(&a);
  lv_anim_set_var(&a, &g_d1);
  lv_anim_set_exec_cb(&a, demo1_tick_anim);
  lv_anim_set_values(&a, 0, 360);
  lv_anim_set_duration(&a, 2000);
  lv_anim_set_playback_duration(&a, 2000); /* different period */
  lv_anim_set_repeat_count(&a, LV_ANIM_REPEAT_INFINITE);
  lv_anim_start(&a);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nyabula_dual_demo_create
 *
 * Description:
 *   Build one independent animated UI per display.  Each UI is rooted on
 *   its own display's active screen via lv_display_get_screen_active()
 *   (NOT the global lv_screen_active()), so the two screens never share
 *   objects even though both animations run inside the single LVGL
 *   instance that nyabula_display_task() drives.
 *
 * Input Parameters:
 *   disp0 - LVGL display for screen 0 (or NULL to skip)
 *   disp1 - LVGL display for screen 1 (or NULL to skip)
 *
 ****************************************************************************/

void nyabula_dual_demo_create(lv_display_t *disp0, lv_display_t *disp1)
{
  lv_obj_t *scr;

  memset(&g_d0, 0, sizeof(g_d0));
  memset(&g_d1, 0, sizeof(g_d1));

  if (disp0)
    {
      scr = lv_display_get_screen_active(disp0);
      if (scr)
        {
          lv_obj_set_style_bg_color(scr, lv_color_hex(0x101418), 0);
          demo0_build(scr);
        }
    }

  if (disp1)
    {
      scr = lv_display_get_screen_active(disp1);
      if (scr)
        {
          lv_obj_set_style_bg_color(scr, lv_color_hex(0x101418), 0);
          demo1_build(scr);
        }
    }
}

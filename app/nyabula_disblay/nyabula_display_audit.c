/****************************************************************************
 * apps/graphics/nyabula_display/nyabula_display_audit.c
 *
 * Audit / profiling instrumentation for the dual-LCD layer.
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0
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
 *
 ****************************************************************************/

/* The dual-screen compat layer bypasses LVGL's built-in sysmon, so we keep
 * our own per-screen runtime metrics in one global structure (g_audit) in
 * this dedicated translation unit.  The measurement sites in the display
 * layer call the nyabula_disp_audit_*() entry points declared in
 * nyabula_display_audit.h.
 *
 * Everything here -- the g_audit storage, the helper audit_now_us() and
 * every measurement function -- is wrapped in #if NYABULA_AUDIT.  When the
 * switch is 0 this file compiles to almost nothing (no storage, no
 * functions, no references), so there are no unused-variable / unused-
 * function -Werror hazards and zero runtime overhead.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "nyabula_display_audit.h"

#include <debug.h>
#include <string.h>
#include <time.h>

#include "nyabula_dual_lcd.h"

#ifdef CONFIG_NYABULA_DISPLAY_AUDIT

/****************************************************************************
 * Private Types / Data
 ****************************************************************************/

/* Per-screen audit state, held in a single global struct so no state is
 * stashed in per-screen scheduler structs or function-local variables. */

struct nyabula_audit_screen_s
{
  uint32_t render_us;         /* Interval between two flush callbacks */
  uint32_t render_cpu_us;     /* Wall time of one lv_refr_now() */
  uint32_t flush_wait_us;     /* Time stuck in flush_wait_cb (double-buf) */
  uint32_t defer_wait_us;     /* Time deferred in render loop waiting for a
                               * free buffer (v6 _db_pending) before render */
  uint32_t trans_us;          /* One half-frame PUTAREA ioctl time */
  uint32_t frame_interval_us; /* Front-half transfer period (approx frame) */
  uint32_t fps_x100;          /* Transfer FPS * 100 */
  uint32_t half_count;        /* Number of front-half transfers */
  uint32_t refr_calls;        /* lv_refr_now() invocations */
  uint32_t flush_count;       /* flush_cb fires */
  uint32_t te_frame_us;       /* Real TE frame period */
  uint32_t loop_interval_us;  /* nyabula_dual_lcd_task() iteration period */
  uint32_t wait_kick_us;      /* Wait on render_kick semaphore */

  /* Internal timestamps (u32, microsecond monotonic, valid ~71 min). */
  uint32_t last_render_end_us;
  uint32_t last_half_start_us;
  uint32_t last_loop_us;
  uint32_t last_te_frame_us;
  uint32_t meas_t0; /* scratch start stamp for a measurement */
};

static struct
{
  struct nyabula_audit_screen_s scr[NYABULA_DUAL_LCD_MAX_SCREENS];
  unsigned print_frame; /* Accumulated TE frames for periodic print */
} g_audit;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static uint32_t audit_now_us(void);
static void nyabula_disp_audit_print(const struct nyabula_dual_lcd_s *dual);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: audit_now_us
 *
 * Description:
 *   Current monotonic time in microseconds (truncated to u32; valid for
 *   ~71 minutes, far beyond a debug session window).  Uses CLOCK_MONOTONIC
 *   so wall-clock jumps do not skew the measured durations / periods.
 *
 ****************************************************************************/

static uint32_t audit_now_us(void)
{
  struct timespec ts;

  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (uint32_t)((uint64_t)ts.tv_sec * 1000000ULL +
                    (uint64_t)(ts.tv_nsec / 1000));
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nyabula_disp_audit_init
 *
 * Description:
 *   Zero the global audit state.  Called once at create time.
 *
 ****************************************************************************/

void nyabula_disp_audit_init(void) { memset(&g_audit, 0, sizeof(g_audit)); }

/****************************************************************************
 * Name: nyabula_disp_audit_flush_end
 *
 * Description:
 *   Called from flush_cb when a frame render completes: records render time
 *   (= interval between two consecutive flushes) and counts real flushes.
 *
 ****************************************************************************/

void nyabula_disp_audit_flush_end(int sid)
{
  struct nyabula_audit_screen_s *a = &g_audit.scr[sid];
  uint32_t now;

  a->flush_count++;

  now = audit_now_us();
  if (a->last_render_end_us != 0)
    {
      a->render_us = now - a->last_render_end_us;
    }

  a->last_render_end_us = now;
}

/****************************************************************************
 * Name: nyabula_disp_audit_flush_wait_start / flush_wait_end
 *
 * Description:
 *   Measure how long flush_wait_cb actually blocks waiting for a free
 *   double-buffer.
 *
 ****************************************************************************/

void nyabula_disp_audit_flush_wait_start(int sid)
{
  g_audit.scr[sid].meas_t0 = audit_now_us();
}

void nyabula_disp_audit_flush_wait_end(int sid)
{
  struct nyabula_audit_screen_s *a = &g_audit.scr[sid];
  a->flush_wait_us = audit_now_us() - a->meas_t0;
}

/****************************************************************************
 * Name: nyabula_disp_audit_defer_wait_start / defer_wait_end
 *
 * Description:
 *   Measure how long the render loop defers a screen whose double-buffer is
 *   full before rendering it (the v6 "_db_pending" wait).  The deferral
 *   lets the OTHER screen(s) render first, so this captures the cost of the
 *   deferred screen only, not the shared-thread stall.
 *
 ****************************************************************************/

void nyabula_disp_audit_defer_wait_start(int sid)
{
  g_audit.scr[sid].meas_t0 = audit_now_us();
}

void nyabula_disp_audit_defer_wait_end(int sid)
{
  struct nyabula_audit_screen_s *a = &g_audit.scr[sid];
  a->defer_wait_us = audit_now_us() - a->meas_t0;
}

/****************************************************************************
 * Name: nyabula_disp_audit_transfer_start / transfer_end
 *
 * Description:
 *   Measure one half-frame transfer (ioctl) wall time.  The start also
 *   tracks the front-half refresh period which yields the transfer FPS.
 *
 ****************************************************************************/

void nyabula_disp_audit_transfer_start(int sid, int start_line)
{
  struct nyabula_audit_screen_s *a = &g_audit.scr[sid];
  uint32_t now;

  now = audit_now_us();
  a->meas_t0 = now;

  /* A front half (row 0) begins a new frame: measure the refresh period. */
  if (start_line == 0)
    {
      if (a->last_half_start_us != 0)
        {
          a->frame_interval_us = now - a->last_half_start_us;
          if (a->frame_interval_us > 0)
            {
              a->fps_x100 = 100000000u / a->frame_interval_us;
            }
        }

      a->last_half_start_us = now;
      a->half_count++;
    }
}

void nyabula_disp_audit_transfer_end(int sid)
{
  struct nyabula_audit_screen_s *a = &g_audit.scr[sid];
  a->trans_us = audit_now_us() - a->meas_t0;
}

/****************************************************************************
 * Name: nyabula_disp_audit_loop
 *
 * Description:
 *   Called at the top of nyabula_dual_lcd_task(): measures the drive-loop
 *   iteration period (entry to entry).
 *
 ****************************************************************************/

void nyabula_disp_audit_loop(const struct nyabula_dual_lcd_s *dual)
{
  struct nyabula_audit_screen_s *a = &g_audit.scr[0];
  uint32_t now;

  now = audit_now_us();
  if (a->last_loop_us != 0)
    {
      a->loop_interval_us = now - a->last_loop_us;
    }

  a->last_loop_us = now;
  (void)dual;
}

/****************************************************************************
 * Name: nyabula_disp_audit_render_start / render_end
 *
 * Description:
 *   Measure the wall clock of one lv_refr_now() call and count refreshes.
 *
 ****************************************************************************/

void nyabula_disp_audit_render_start(int sid)
{
  struct nyabula_audit_screen_s *a = &g_audit.scr[sid];
  a->refr_calls++;
  a->meas_t0 = audit_now_us();
}

void nyabula_disp_audit_render_end(int sid)
{
  struct nyabula_audit_screen_s *a = &g_audit.scr[sid];
  a->render_cpu_us = audit_now_us() - a->meas_t0;
}

/****************************************************************************
 * Name: nyabula_disp_audit_waitkick_start / waitkick_end
 *
 * Description:
 *   Measure how long the render thread blocks on render_kick.
 *
 ****************************************************************************/

void nyabula_disp_audit_waitkick_start(const struct nyabula_dual_lcd_s *dual)
{
  g_audit.scr[0].meas_t0 = audit_now_us();
  (void)dual;
}

void nyabula_disp_audit_waitkick_end(const struct nyabula_dual_lcd_s *dual)
{
  struct nyabula_audit_screen_s *a = &g_audit.scr[0];
  a->wait_kick_us = audit_now_us() - a->meas_t0;
  (void)dual;
}

/****************************************************************************
 * Name: nyabula_disp_audit_te_frame
 *
 * Description:
 *   Called each quarter tick from the TE thread.  On each frame boundary
 *   (tick wraps to 0) it measures the real TE frame period and, every
 *   NYABULA_AUDIT_PRINT_EVERY frames, prints all audit metrics.
 *
 ****************************************************************************/

void nyabula_disp_audit_te_frame(const struct nyabula_dual_lcd_s *dual,
                                 uint8_t tick)
{
  struct nyabula_audit_screen_s *a = &g_audit.scr[0];
  uint32_t now;

  if (tick != 0)
    {
      return;
    }

  now = audit_now_us();
  if (a->last_te_frame_us != 0)
    {
      a->te_frame_us = now - a->last_te_frame_us;
    }

  a->last_te_frame_us = now;

  g_audit.print_frame++;
  if ((g_audit.print_frame % NYABULA_AUDIT_PRINT_EVERY) == 0)
    {
      nyabula_disp_audit_print(dual);
    }
}

/****************************************************************************
 * Name: nyabula_disp_audit_print
 *
 * Description:
 *   Print the per-screen audit metrics.
 *
 ****************************************************************************/

static void nyabula_disp_audit_print(const struct nyabula_dual_lcd_s *dual)
{
  int sid;

  for (sid = 0; sid < NYABULA_DUAL_LCD_MAX_SCREENS; sid++)
    {
      const struct nyabula_audit_screen_s *a = &g_audit.scr[sid];

      if (!dual->screen[sid].initialized)
        {
          continue;
        }

      _info("[audit] screen %d: render=%u us  cpu=%u us  transfer=%u us  "
            "fps=%u.%02u (%u us/frame, %u hits)  flushwait=%u us"
            " deferwait=%u us"
            "%s loop=%u us waitkick=%u us  refr=%u flush=%u teframe=%u us\n",
            sid, a->render_us, a->render_cpu_us, a->trans_us,
            a->fps_x100 / 100, a->fps_x100 % 100, a->frame_interval_us,
            a->half_count, a->flush_wait_us, a->defer_wait_us,
            (sid == 0) ? "  " : "", (sid == 0) ? a->loop_interval_us : 0u,
            (sid == 0) ? a->wait_kick_us : 0u, a->refr_calls, a->flush_count,
            (sid == 0) ? a->te_frame_us : 0u);
    }
}

#endif /* CONFIG_NYABULA_DISPLAY_AUDIT */

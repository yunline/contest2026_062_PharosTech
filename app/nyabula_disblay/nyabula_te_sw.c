/****************************************************************************
 * apps/graphics/nyabula_display/nyabula_te_sw.c
 *
 * Software TE frame clock source (CONFIG_NYABULA_DISPLAY_TE_SW).
 *
 * Until the real TE signal is routed from the panels, this source
 * synthesizes the two TE edges per screen at the target frame rate with a
 * high-priority thread and calls the framework's edge callbacks -- the
 * same callbacks a future GPIO TE source (deferred to a thread) would use.
 *
 *   TE polarity: HIGH = blanking, LOW = scanning.
 *     - scan-start edge (TE falling)  at frame_start + phase_us[sid]
 *     - blank-start edge (TE rising)  at frame_start + phase_us[sid]
 *                                                  + ACTIVE_US
 *
 * Each screen carries its own phase offset (phase_us); screen 1 is offset
 * by 1/4 frame period (90 deg) so its edges do not coincide with screen 0's.
 * This staggers the two screens' blanking windows, giving a milder load
 * profile on the shared QSPI bus.
 *
 * Per frame, the four events (2 edges x 2 screens) are sorted by absolute
 * time and fired in order.  The frame boundary target is advanced onto the
 * ideal frame line (absolute TIMER_ABSTIME), so per-event scheduling jitter
 * does not accumulate into the long-run frame period.
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

#include "nyabula_te.h"

#include <pthread.h>
#include <sched.h>
#include <string.h>
#include <time.h>

#include <lvgl/lvgl.h>

/* One scheduled edge event. */

typedef struct
{
  struct timespec ts; /* absolute fire time */
  int sid;            /* screen 0/1 */
  bool blank_start;   /* true = blank-start edge, false = scan-start edge */
} te_edge_ev_t;

/* Software TE source instance.  The public nyabula_te_t handle aliases the
 * first member. */

struct nyabula_te_s
{
  pthread_t thread;
  bool running;

  /* Per-screen phase offset within the frame, in us (screen 0 = 0, screen 1
   * defaults to 90 deg so their blanking windows do not coincide). */
  uint32_t phase_us[NYABULA_TE_MAX_SCREENS];

  struct nyabula_dual_lcd_s *dual;
  nyabula_te_callbacks_t cb;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Advance a timespec by `us` microseconds in place (normalizing ns). */

static void te_ts_add_us(struct timespec *ts, useconds_t us)
{
  ts->tv_nsec += (long)us * 1000L;
  while (ts->tv_nsec >= 1000000000L)
    {
      ts->tv_nsec -= 1000000000L;
      ts->tv_sec++;
    }
}

/* Compare two timespecs: <0 if a earlier, 0 if equal, >0 if a later. */

static int te_ts_cmp(const struct timespec *a, const struct timespec *b)
{
  if (a->tv_sec != b->tv_sec)
    {
      return (a->tv_sec < b->tv_sec) ? -1 : 1;
    }

  if (a->tv_nsec != b->tv_nsec)
    {
      return (a->tv_nsec < b->tv_nsec) ? -1 : 1;
    }

  return 0;
}

static void *te_thread_func(void *arg)
{
  struct nyabula_te_s *t = (struct nyabula_te_s *)arg;
  useconds_t frame_us;
  useconds_t active_us;
  struct timespec frame_ts;
  te_edge_ev_t ev[NYABULA_TE_MAX_SCREENS * 2];
  int sid;
  int i;
  int j;

  frame_us = 1000000 / NYABULA_TE_REFRESH_HZ;
  active_us = NYABULA_TE_ACTIVE_US;

  /* Initialize the absolute frame-boundary target, one frame ahead so the
   * first TE edges fire ~frame_us into the run.  From here on we sleep to
   * ABSOLUTE target times (TIMER_ABSTIME), accumulating the frame step onto
   * a fixed baseline each frame.  usleep()'s unpredictable per-sleep
   * overhead then shows up only as a non-cumulative per-event residual
   * instead of being baked into the frame period. */
  clock_gettime(CLOCK_MONOTONIC, &frame_ts);
  te_ts_add_us(&frame_ts, frame_us);

  while (t->running)
    {
      int n = 0;

      /* Collect the two edges per screen for this frame. */
      for (sid = 0; sid < NYABULA_TE_MAX_SCREENS; sid++)
        {
          ev[n].ts = frame_ts;
          te_ts_add_us(&ev[n].ts, t->phase_us[sid]);
          ev[n].sid = sid;
          ev[n].blank_start = false;
          n++;

          ev[n].ts = frame_ts;
          te_ts_add_us(&ev[n].ts, t->phase_us[sid] + active_us);
          ev[n].sid = sid;
          ev[n].blank_start = true;
          n++;
        }

      /* Insertion sort the (tiny) event list by absolute time. */
      for (i = 1; i < n; i++)
        {
          te_edge_ev_t key = ev[i];

          j = i - 1;
          while (j >= 0 && te_ts_cmp(&ev[j].ts, &key.ts) > 0)
            {
              ev[j + 1] = ev[j];
              j--;
            }

          ev[j + 1] = key;
        }

      for (i = 0; i < n; i++)
        {
          clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ev[i].ts, NULL);
          if (!t->running)
            {
              break;
            }

          if (ev[i].blank_start)
            {
              if (t->cb.blank_start != NULL)
                {
                  t->cb.blank_start(t->dual, ev[i].sid);
                }
            }
          else
            {
              if (t->cb.scan_start != NULL)
                {
                  t->cb.scan_start(t->dual, ev[i].sid);
                }
            }
        }

      if (!t->running)
        {
          break;
        }

      /* Advance the frame boundary onto the ideal frame line (do NOT
       * re-anchor to the actual wake-up moment, otherwise every late-wake
       * delta would be baked into the next period permanently). */
      te_ts_add_us(&frame_ts, frame_us);
    }

  return NULL;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

nyabula_te_t *nyabula_te_init(struct nyabula_dual_lcd_s *dual,
                              const nyabula_te_callbacks_t *cb)
{
  struct nyabula_te_s *t;
  int ret;

  t = (struct nyabula_te_s *)lv_malloc(sizeof(*t));
  if (t == NULL)
    {
      return NULL;
    }

  memset(t, 0, sizeof(*t));
  t->dual = dual;
  t->running = false;
  if (cb != NULL)
    {
      t->cb = *cb;
    }

  /* Phase stagger: screen 1 leads screen 0 by 90 deg (1/4 frame period) so
   * their blanking windows do not coincide, de-correlating the two screens'
   * QSPI write bursts on the shared bus.  A real hardware TE provides the
   * physical phase instead (this field is unused in the GPIO source). */
  t->phase_us[0] = 0;
  t->phase_us[1] = (1000000 / NYABULA_TE_REFRESH_HZ) / 4;

  t->running = true;

  ret = nyabula_te_create_thread_prio(&t->thread, te_thread_func, t,
                                      NYABULA_TE_PRIORITY);
  if (ret != 0)
    {
      t->running = false;
      lv_free(t);
      return NULL;
    }

  return (nyabula_te_t *)t;
}

void nyabula_te_deinit(nyabula_te_t *te)
{
  struct nyabula_te_s *t = (struct nyabula_te_s *)te;

  if (t == NULL)
    {
      return;
    }

  t->running = false;

  /* The thread sleeps on clock_nanosleep (up to ~one frame); it checks
   * running after each sleep and exits on the next boundary. */
  pthread_join(t->thread, NULL);

  lv_free(t);
}

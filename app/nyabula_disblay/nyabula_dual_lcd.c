/****************************************************************************
 * apps/graphics/nyabula_display/nyabula_dual_lcd.c
 *
 * Dual-screen dual-buffer LVGL interface layered on the v6 "BoundedChase"
 * frame scheduler.
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

/****************************************************************************
 * Design overview
 *
 * Two panels (ST77916, 360x360 RGB565) share ONE QSPI write bus, so at any
 * instant only one screen can be written.  This layer connects LVGL to the
 * panels using a double-buffer pipeline and a faithful C port of the v6
 * "BoundedChase" scheduler (see vsyncalg/V6_DESIGN.md and
 * vsyncalg/algos/bounded_chase.py).
 *
 * Thread model
 *   - Render loop    : hosted by the caller (main) via
 *nyabula_dual_lcd_task(), which runs lv_timer_handler() (LVGL is
 *single-threaded).
 *   - TE thread       : software frame clock standing in for the real TE
 *                     signal.  Fires the scan edges at the frame rate and
 *                     feeds the ISR-safe edge handlers.  The two screens'
 *                     clocks are staggered 90-deg to de-correlate their QSPI
 *                     write bursts.
 *   - Transfer thread: pops half-block write jobs and issues a BLOCKING
 *                     LCDDEVIO_PUTAREA ioctl (QSPI DMA).  It does not
 *                     consume CPU while the DMA is in flight.
 *
 * TE signal / state machine
 *   The panel TE is not yet wired in hardware, so a software frame clock
 *   synthesizes the two edges of the scan:
 *     * falling edge (scan has completed the front half 0..half-1):
 *          lock the latest fully-rendered buffer, submit the FRONT half
 *          (0..half) as one block, and kick off the next frame's render.
 *     * rising  edge (a full frame has been scanned):
 *          submit the BACK half (half..total) from the SAME locked buffer.
 *   Writing generation always comes from a buffer whose render has fully
 *   completed, and both halves come from the same buffer => tearing is
 *   avoided while bandwidth is sufficient.
 *
 *   The edge entry points (nyabula_te_edge_rise/fall) are kept ISR-safe:
 *   they only update state and post semaphores.  When the real TE signal
 *   is routed (gpio_irqattach), the same functions can be called from an
 *   ISR with no change to the algorithm.
 *
 * Backpressure (the heart of v6)
 *   Each screen tracks submitted_lines - written_lines ("outstanding").
 *   When it reaches the half-frame budget, submission is deferred so the
 *   write queue cannot grow unboundedly => content degrades to
 *   alternate-frame refresh instead of freezing when the bus is saturated.
 *
 * Fair dispatch
 *   The transfer thread serves job[0] and job[1] round-robin so neither
 *   screen can monopolize the shared bus.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "nyabula_dual_lcd.h"

#include <debug.h>
#include <errno.h>
#include <fcntl.h>
#include <nuttx/lcd/lcd_dev.h>
#include <string.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>

#include <lvgl/lvgl.h>

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int32_t align_round_up(int32_t v, uint16_t align);
static void rounder_cb(lv_event_t *e);

/* v6 state machine (per screen) */
static int engine_remaining(uint32_t submitted, uint32_t written);
static bool engine_can_submit(const nyabula_screen_t *scr);
static void te_edge_rise(nyabula_dual_lcd_t *dual, int sid);
static void te_edge_fall(nyabula_dual_lcd_t *dual, int sid);

/* Job queue */
static int job_enqueue(nyabula_dual_lcd_t *dual, const nyabula_write_job_t *j);
static int job_dequeue_fair(nyabula_dual_lcd_t *dual,
                            nyabula_write_job_t *out);

/* TE software frame clock thread (stands in for the real TE ISR) */
static void *te_thread_func(void *arg);

/* Threads */
static void *transfer_thread_func(void *arg);

/* Buffer / display helpers */
static void rounder_cb(lv_event_t *e);
static void te_refr_req_cb(lv_event_t *e);
static void flush_cb(lv_display_t *disp, const lv_area_t *area,
                     uint8_t *color_p);
static void flush_wait_cb(lv_display_t *disp);
static void display_release_cb(lv_event_t *e);
static int screen_display_on(nyabula_screen_t *scr);
static int screen_init(nyabula_screen_t *scr, int sid, const char *dev_path,
                       int width, int height);
static void screen_destroy(nyabula_screen_t *scr);

/****************************************************************************
 * Name: align_round_up
 *
 * Description:
 *   Round v up to the next multiple of align.
 *
 ****************************************************************************/

static int32_t align_round_up(int32_t v, uint16_t align)
{
  return (v + align - 1) & ~(align - 1);
}

/****************************************************************************
 * Name: rounder_cb
 *
 * Description:
 *   LVGL invalidation-area alignment, requested by the LCD upper half via
 *   LCDDEVIO_GETAREAALIGN.  Aligns dirty-area coordinates to the panel's
 *   transfer constraints.
 *
 ****************************************************************************/

static void rounder_cb(lv_event_t *e)
{
  nyabula_screen_t *scr = lv_event_get_user_data(e);
  lv_area_t *area;
  struct lcddev_area_align_s *align_info;
  int32_t w;
  int32_t h;

  /* Only align whole-screen flushes for the full-refresh pipeline. */
  if (lv_event_get_code(e) != LV_EVENT_INVALIDATE_AREA)
    {
      return;
    }

  area = lv_event_get_param(e);
  align_info = &scr->align;

  area->x1 &= ~(align_info->col_start_align - 1);
  area->y1 &= ~(align_info->row_start_align - 1);

  w = align_round_up(lv_area_get_width(area), align_info->width_align);
  h = align_round_up(lv_area_get_height(area), align_info->height_align);

  area->x2 = area->x1 + w - 1;
  area->y2 = area->y1 + h - 1;
}

/****************************************************************************
 * Name: te_refr_req_cb
 *
 * Description:
 *   LVGL event callback for LV_EVENT_REFR_REQUEST.  LVGL's built-in
 *   disp_event_cb resumes the default refresh timer on this event, which
 *   would make it auto-render outside the TE cadence.  That built-in
 *   handler is registered first, so our callback runs AFTER it; here we
 *   pause the refresh timer again to cancel that auto-resume.  Rendering is
 *   thereby driven exclusively by the TE falling edge via explicit
 *   lv_refr_now() (the timer handle is kept non-NULL so lv_refr_now works).
 *
 *   Note: lv_refr_now() -> _lv_display_refr_timer() re-pauses the timer
 *   itself after running one frame, so the timer never auto-runs.
 *
 ****************************************************************************/

static void te_refr_req_cb(lv_event_t *e)
{
  lv_display_t *disp;
  nyabula_screen_t *scr;
  lv_timer_t *tmr;

  if (lv_event_get_code(e) != LV_EVENT_REFR_REQUEST)
    {
      return;
    }

  disp = lv_event_get_current_target(e);
  if (disp == NULL)
    {
      return;
    }

  scr = lv_display_get_driver_data(disp);
  if (!scr || !scr->initialized)
    {
      return;
    }

  /* Cancel LVGL's auto-resume of the refresh timer (TE-exclusive render). */
  tmr = lv_display_get_refr_timer(disp);
  if (tmr != NULL)
    {
      lv_timer_pause(tmr);
    }
}

/****************************************************************************
 * v6 state machine helpers
 ****************************************************************************/

/* Outline lines still "in flight" (submitted but not yet written) for a
 * screen.  Mirrors BoundedChaseScheduler._outstanding(). */

static int engine_remaining(uint32_t submitted, uint32_t written)
{
  if (submitted <= written)
    {
      return 0;
    }

  return (int)(submitted - written);
}

/* May this screen submit the next half-block?  Backpressure applies only
 * when the in-flight line count reaches the half-frame budget.  When
 * bandwidth is sufficient this is always true (== v5/v6 optimal). */

static bool engine_can_submit(const nyabula_screen_t *scr)
{
  return engine_remaining(scr->submitted_lines, scr->written_lines) <
         NYABULA_PENDING_BUDGET_LINES;
}

/****************************************************************************
 * Name: te_edge_rise
 *
 * Description:
 *   Rising edge of the scan (a full frame has been scanned, blanking has
 *   just ended and a new frame scan is starting).  Submit the BACK half of
 *   the locked buffer, same generation as the front half pushed at the
 *   falling edge => the whole frame shows one generation => 0 tearing.
 *
 *   ISR-safe: only updates state and posts semaphores.
 *
 ****************************************************************************/

static void te_edge_rise(nyabula_dual_lcd_t *dual, int sid)
{
  nyabula_screen_t *scr;
  nyabula_write_job_t job;

  if (dual->quitting)
    {
      return;
    }

  scr = &dual->screen[sid];
  if (!scr->initialized)
    {
      return;
    }

  sem_wait(&scr->st_mutex);

  /* Nothing locked at the falling edge -> nothing to complete. */
  if (scr->half_buf < 0)
    {
      sem_post(&scr->st_mutex);
      return;
    }

  /* Backpressure: defer the BACK-half push until outstanding < budget. */
  if (!engine_can_submit(scr))
    {
      sem_post(&scr->st_mutex);
      return;
    }

  memset(&job, 0, sizeof(job));
  job.screen_id = sid;
  job.start_line = scr->half_lines;
  job.num_lines = scr->total_lines - scr->half_lines;
  job.buf_idx = scr->half_buf;
  job.gen = scr->buf[scr->half_buf].gen;

  if (job_enqueue(dual, &job) == 0)
    {
      /* Lines are now "submitted". */
      scr->submitted_lines += (uint32_t)job.num_lines;
    }

  sem_post(&scr->st_mutex);
}

/****************************************************************************
 * Name: te_edge_fall
 *
 * Description:
 *   Falling edge of the scan: the panel has scanned rows 0..half-1 (the
 *   front half is therefore safe to update this frame).
 *
 *   1) Lock the latest fully-rendered buffer (consume render_done).
 *   2) Submit the FRONT half (0..half) of the locked buffer as one block,
 *      subject to backpressure.
 *
 *   The next frame's render does not need to be kicked off here: LVGL
 *   self-drives animated content in its render thread.  This handler only
 *   decides push timing, so it stays ISR-safe.
 *
 *   Mirrors BoundedChaseScheduler._on_scan_half().
 *
 *   ISR-safe: only updates state and posts semaphores.
 *
 ****************************************************************************/

static void te_edge_fall(nyabula_dual_lcd_t *dual, int sid)
{
  nyabula_screen_t *scr;
  nyabula_write_job_t job;

  if (dual->quitting)
    {
      return;
    }

  scr = &dual->screen[sid];
  if (!scr->initialized)
    {
      return;
    }

  sem_wait(&scr->st_mutex);

  /* 1) Lock the latest fully-rendered buffer into the half-push slot, but
   * only when the previously locked buffer is done -- i.e. its back half
   * was submitted and the transfer thread released half_buf to -1.  If an
   * old back half is still pending (deferred by v6 backpressure), do NOT
   * overwrite half_buf: that would drop the old buffer's back-half
   * reference and its fill_pending would never decrement, leaking that
   * physical buffer forever and progressively locking this screen out.
   * (v6's _half_gen is a generation number -- free to be overwritten,
   * dropping only one frame's content -- whereas half_buf is a physical
   * buffer index bound to the buffer's lifetime, so it must not be
   * re-pointed while its back half is unwritten.) */
  if (scr->half_buf < 0 && scr->render_done_buf >= 0)
    {
      scr->half_buf = scr->render_done_buf;
      scr->render_done_buf = -1;
    }

  /* 2) Drive the next frame's render at the falling edge (v6 starts a fresh
   *    render here so the content is ready for the next write window).  This
   *    is the only render driver: the render loop calls lv_refr_now() for
   *    this screen.  A static screen is not force-redrawn because
   *    lv_refr_now() skips drawing when there is no invalid area.  This is
   *    ISR-safe (flag + sem post only). */
  scr->render_request = true;
  sem_post(&dual->render_kick);

  /* 3) Submit the front half of the locked generation.  Backpressure may
   *    defer this push only; it never blocks the render request above. */
  if (scr->half_buf < 0)
    {
      sem_post(&scr->st_mutex);
      return; /* nothing rendered yet -> keep current content */
    }

  if (!engine_can_submit(scr))
    {
      sem_post(&scr->st_mutex);
      return; /* backpressure: defer push until bus drains */
    }

  memset(&job, 0, sizeof(job));
  job.screen_id = sid;
  job.start_line = 0;
  job.num_lines = scr->half_lines;
  job.buf_idx = scr->half_buf;
  job.gen = scr->buf[scr->half_buf].gen;

  if (job_enqueue(dual, &job) == 0)
    {
      scr->submitted_lines += (uint32_t)job.num_lines;
    }

  sem_post(&scr->st_mutex);
}

/****************************************************************************
 * Job queue (bounded, guarded by job_mutex + job_avail/job_space counting
 * semaphores).
 ****************************************************************************/

static int job_enqueue(nyabula_dual_lcd_t *dual, const nyabula_write_job_t *j)
{
  int ret = -EAGAIN;

  if (sem_trywait(&dual->job_space) != 0)
    {
      /* Queue full; respect backpressure and drop the submission. */
      return -EAGAIN;
    }

  sem_wait(&dual->job_mutex);

  dual->job_queue[dual->job_tail] = *j;
  dual->job_tail = (dual->job_tail + 1) % NYABULA_JOB_QUEUE_SIZE;
  dual->job_count++;
  ret = 0;

  sem_post(&dual->job_mutex);
  sem_post(&dual->job_avail);

  return ret;
}

static int job_dequeue_fair(nyabula_dual_lcd_t *dual, nyabula_write_job_t *out)
{
  int i;
  int last;

  sem_wait(&dual->job_avail);
  sem_wait(&dual->job_mutex);

  /* Fair dispatch: prefer the earliest queued job belonging to the screen
   * other than the last one served, so neither screen can monopolize the
   * shared bus (mirrors Simulation._pop_fair). */
  last = dual->last_bus_screen;
  if (last >= 0 && dual->job_count > 1)
    {
      for (i = 0; i < dual->job_count; i++)
        {
          int idx = (dual->job_head + i) % NYABULA_JOB_QUEUE_SIZE;
          if (dual->job_queue[idx].screen_id != last)
            {
              *out = dual->job_queue[idx];
              /* Shift the hole out by pulling elements forward. */
              while (i > 0)
                {
                  int prev = (dual->job_head + i - 1) % NYABULA_JOB_QUEUE_SIZE;
                  dual->job_queue[idx] = dual->job_queue[prev];
                  idx = prev;
                  i--;
                }

              dual->job_head = (dual->job_head + 1) % NYABULA_JOB_QUEUE_SIZE;
              dual->job_count--;
              sem_post(&dual->job_mutex);
              sem_post(&dual->job_space);
              return 0;
            }
        }
    }

  /* Ordinary FIFO head (or single-screen case). */
  *out = dual->job_queue[dual->job_head];
  dual->job_head = (dual->job_head + 1) % NYABULA_JOB_QUEUE_SIZE;
  dual->job_count--;
  sem_post(&dual->job_mutex);
  sem_post(&dual->job_space);

  return 0;
}

/****************************************************************************
 * Name: te_ts_add_us
 *
 * Description:
 *   Advance a timespec by `us` microseconds in place (normalizing ns).
 *   Used by the absolute-time TE calibration to step the quarter-frame
 *   target clock without accumulating drift.
 *
 ****************************************************************************/

static void te_ts_add_us(struct timespec *ts, useconds_t us)
{
  ts->tv_nsec += (long)us * 1000L;
  while (ts->tv_nsec >= 1000000000L)
    {
      ts->tv_nsec -= 1000000000L;
      ts->tv_sec++;
    }
}

/****************************************************************************
 * Name: te_thread_func
 *
 * Description:
 *   Software TE frame clock.  Until the real TE signal is routed from the
 *   panels, this thread synthesizes the scan edges at the target frame
 *   rate and feeds them to the ISR-safe edge handlers -- the same entry
 *   points a future gpio_irqattach ISR would call.  Keeping it on its own
 *   thread mirrors the asynchronous nature of a hardware TE interrupt.

 *   The frame period is divided into 4 quarter-frame ticks (90-deg each).
 *   Each screen carries its own phase offset (te_phase_shift); screen 1 is
 *   offset by 1 tick (90-deg) so its fall/rise edges do not coincide with
 *   screen 0's.  This staggers the two screens' write submissions, giving
 *   a milder load profile on the shared QSPI bus.
 *
 *   Per screen, per frame:
 *     phase 0 : falling edge (front half scanned out) -> submit front half
 *     phase 2 : rising  edge (full frame scanned)     -> submit back half
 *
 ****************************************************************************/

static void *te_thread_func(void *arg)
{
  nyabula_dual_lcd_t *dual = (nyabula_dual_lcd_t *)arg;
  useconds_t period_us;
  useconds_t quarter_us;
  struct timespec next_ts;
  uint8_t tick;
  int sid;

  period_us = 1000000 / NYABULA_DUAL_LCD_REFRESH_HZ;
  quarter_us = period_us / 4;
  tick = 0;

  /* Initialize the absolute quarter-frame target clock, one quarter ahead so
   * the first TE edge fires ~quarter_us into the run.  From here on we sleep
   * to ABSOLUTE target times (TIMER_ABSTIME), accumulating the step onto a
   * fixed baseline each quarter.  usleep()'s unpredictable per-sleep overhead
   * then shows up only as a non-cumulative per-quarter residual instead of
   * 4x amplified into the frame period. */
  clock_gettime(CLOCK_MONOTONIC, &next_ts);
  te_ts_add_us(&next_ts, quarter_us);

  while (dual->running)
    {
      for (sid = 0; sid < NYABULA_DUAL_LCD_MAX_SCREENS; sid++)
        {
          /* phase only needs the low 2 bits: tick is kept in 0..3 so the
           * counter never grows beyond 4 (no long-run overflow). */
          int phase = (int)((tick + dual->screen[sid].te_phase_shift) & 3u);

          if (phase == 0)
            {
              /* Falling edge: front half of this screen's frame scanned. */
              te_edge_fall(dual, sid);
            }
          else if (phase == 2)
            {
              /* Rising edge: full frame scanned -> back half, same gen. */
              te_edge_rise(dual, sid);
            }
        }

      /* Sleep to the CURRENT absolute target, then advance the target by one
       * quarter onto the IDEAL clock line -- do NOT re-anchor to the actual
       * wake-up moment.  If we re-anchored on the real wake time, every
       * late-wake delta (scheduling delay, higher-prio interrupt, other load)
       * would be baked into the next period permanently, and the long-run
       * frame period would settle at (quarter + ~1ms) instead of period_us
       * (measured: target 50Hz landed at 41.67Hz / 24ms).  Keeping the
       * absolute target accumulating on the ideal line means a late wake
       * shortens only the NEXT sleep (it is caught up), so single-frame
       * jitter does not accumulate and the long-run period stays exactly
       * period_us.
       *
       * quarter_us = period_us/4 truncates (60Hz -> 16666/4 = 4166us, i.e.
       * 4*4166 = 16664 != 16666).  To hold the full frame at exactly
       * period_us we add the truncation remainder (period_us - 3*quarter_us)
       * on the final quarter (tick==3) and quarter_us on the other three, so
       * one frame = (p-3q) + q + q + q = p (0 truncation drift). */
      clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_ts, NULL);

      if (tick == 3)
        {
          /* Closing the frame: use the exact per-frame remainder so the
           * four quarters sum to period_us (0 truncation drift). */
          te_ts_add_us(&next_ts, period_us - 3 * quarter_us);
        }
      else
        {
          te_ts_add_us(&next_ts, quarter_us);
        }

      tick = (uint8_t)((tick + 1) & 3u);

      nyabula_disp_audit_te_frame(dual, tick);
    }

  return NULL;
}

/****************************************************************************
 * Name: nyabula_dual_lcd_task
 *
 * Description:
 *   Run one step of the render loop from the CALLER's own thread (e.g. the
 *   main thread).  Each call:
 *     1) advances LVGL via lv_timer_handler() (timers, animation, refresh),
 *     2) consumes any pending TE-driven render requests: invalidates the
 *        screen's active object once so LVGL renders exactly one full frame
 *        (FULL mode) to feed the v6 pipeline.
 *
 *   The display layer starts the TE/transfer threads in create(); the
 *   caller hosts the lv_timer_handler() call (LVGL is single-threaded, and
 *   running it in main avoids an extra thread).  The UI must be built
 *   before the first call.
 *
 *   Animation is unaffected: LVGL's animation timer and the default
 *   refresh re-arm path are untouched; we only add one extra full-screen
 *   invalidate per TE falling edge.
 *
 *   Blocks up to the LVGL idle period when idle (wakes early on a render
 *   kick), then returns so the caller can drive the loop.
 *
 ****************************************************************************/

void nyabula_dual_lcd_task(nyabula_dual_lcd_t *dual)
{
  struct timespec ts;
  uint32_t idle;

  if (!dual)
    {
      return;
    }

  nyabula_disp_audit_loop(dual);

  /* Run one cycle of the LVGL state machine (timers, animation).  This
   * advances animations (time from the NuttX clock via lv_tick_set_cb) and
   * accumulates their invalidations; the default refresh timer is kept
   * paused (see te_refr_req_cb), so it does NOT itself render.  The render
   * happens below via lv_refr_now() driven by the TE falling edge, keeping
   * the pipeline TE-aligned. */
  idle = lv_timer_handler();

  /* Consume the TE-driven render requests: for each screen whose falling
   * edge requested a frame, issue lv_refr_now() (the render driver, kept
   * TE-exclusive via te_refr_req_cb).  lv_refr_now() advances animations too
   * and, crucially, skips drawing when there is no invalid area -- so a
   * static screen is not force-redrawn each frame (saves CPU and QSPI
   * bandwidth), while a running animation is rendered exactly once per TE
   * period with the freshest accumulated content.
   *
   * The two screens share this single render thread, so a screen whose
   * double-buffer is full must NOT stall the loop here (v6 "defer not drop",
   * see Simulation._db_pending).  We render in two passes:
   *   Pass 1 - render every screen that has a free buffer (fill_pending < 2)
   *            right away; defer any screen whose buffers are both full.
   *   Pass 2 - after the other screen(s) have rendered, wait (CPU-free) for
   *            a free buffer on each deferred screen and render it.  This
   *            guarantees a full double-buffer on one screen no longer
   *            serializes (and starves) the other screen's render. */
  {
    bool armed[NYABULA_DUAL_LCD_MAX_SCREENS];
    bool deferred[NYABULA_DUAL_LCD_MAX_SCREENS];
    int sid;

    /* Collect the armed render requests (consume the bools once). */
    for (sid = 0; sid < NYABULA_DUAL_LCD_MAX_SCREENS; sid++)
      {
        nyabula_screen_t *scr = &dual->screen[sid];

        armed[sid] = false;
        deferred[sid] = false;

        if (!scr->initialized || !scr->disp)
          {
            continue;
          }

        sem_wait(&scr->st_mutex);
        if (scr->render_request)
          {
            scr->render_request = false;
            armed[sid] = true;
          }

        sem_post(&scr->st_mutex);
      }

    /* Pass 1: render screens with a free buffer; defer the rest. */
    for (sid = 0; sid < NYABULA_DUAL_LCD_MAX_SCREENS; sid++)
      {
        nyabula_screen_t *scr = &dual->screen[sid];
        bool room;

        if (!armed[sid])
          {
            continue;
          }

        sem_wait(&scr->st_mutex);
        room = (scr->fill_pending < NYABULA_DUAL_LCD_MAX_SCREENS);
        sem_post(&scr->st_mutex);

        if (room)
          {
            nyabula_disp_audit_render_start(sid);
            lv_refr_now(scr->disp);
            nyabula_disp_audit_render_end(sid);
          }
        else
          {
            /* Both buffers hold rendered-but-unwritten generations: defer to
             * pass 2 so the other screen can render meanwhile. */
            deferred[sid] = true;
          }
      }

    /* Pass 2: render the deferred screens once a buffer frees.  By here the
     * other screen(s) have already rendered, so this wait is screen-local
     * and signalled (no busy CPU, no cross-screen serialization). */
    for (sid = 0; sid < NYABULA_DUAL_LCD_MAX_SCREENS; sid++)
      {
        nyabula_screen_t *scr = &dual->screen[sid];

        if (!deferred[sid])
          {
            continue;
          }

        nyabula_disp_audit_defer_wait_start(sid);
        sem_wait(&scr->st_mutex);
        while (scr->fill_pending >= NYABULA_DUAL_LCD_MAX_SCREENS &&
               scr->initialized)
          {
            /* Wait (signalled) for the transfer thread to finish a back-half
             * write and release a buffer.  This never blocks the render of
             * the sibling screen, which already happened in pass 1. */
            sem_post(&scr->st_mutex);
            sem_wait(&scr->buf_free);
            sem_wait(&scr->st_mutex);
          }

        sem_post(&scr->st_mutex);
        nyabula_disp_audit_defer_wait_end(sid);

        nyabula_disp_audit_render_start(sid);
        lv_refr_now(scr->disp);
        nyabula_disp_audit_render_end(sid);
      }
  }

  /* Wait for either a TE edge kick (posted by the TE thread so we service
   * lv_timer_handler() promptly at the frame boundary) or until the LVGL
   * idle period elapses -- whichever comes first.  The timeout is computed
   * relative to NOW so the wait never extends beyond `idle` ms from this
   * moment, keeping the LVGL clock true regardless of how long this call
   * took before reaching the wait. */
  if (idle == 0)
    {
      idle = 1;
    }

  clock_gettime(CLOCK_REALTIME, &ts);
  ts.tv_nsec += (long)idle * 1000000L;
  if (ts.tv_nsec >= 1000000000L)
    {
      ts.tv_sec += ts.tv_nsec / 1000000000L;
      ts.tv_nsec %= 1000000000L;
    }

  nyabula_disp_audit_waitkick_start(dual);
  sem_timedwait(&dual->render_kick, &ts);
  nyabula_disp_audit_waitkick_end(dual);
}

/****************************************************************************
 * Name: transfer_thread_func
 *
 * Description:
 *   Transfer thread.  Pops half-block write jobs (front or back) from the
 *   job queue and issues a BLOCKING LCDDEVIO_PUTAREA ioctl (QSPI DMA) for
 *   each -- the call blocks until the DMA completes but does not consume
 *   CPU meanwhile.  Fair dispatch alternates between the two screens so
 *   neither can monopolize the shared bus.
 *
 *   After a BACK-half job completes, the buffer generation is fully
 *   written to GRAM: fill_pending is decremented, written_lines is updated,
 *   and the render thread is signalled that the buffer is free.
 *
 ****************************************************************************/

static void *transfer_thread_func(void *arg)
{
  nyabula_dual_lcd_t *dual = (nyabula_dual_lcd_t *)arg;
  nyabula_screen_t *scr;
  struct lcddev_area_s lcd_area;
  nyabula_write_job_t job;
  int ret;

  memset(&lcd_area, 0, sizeof(lcd_area));

  while (dual->running)
    {
      job_dequeue_fair(dual, &job);

      if (!dual->running)
        {
          break;
        }

      scr = &dual->screen[job.screen_id];

      lcd_area.row_start = job.start_line;
      lcd_area.row_end = job.start_line + job.num_lines - 1;
      lcd_area.col_start = 0;
      lcd_area.col_end = scr->width - 1;
      lcd_area.stride = scr->stride;
      lcd_area.data =
          scr->buf[job.buf_idx].data + (int64_t)job.start_line * scr->stride;

      /* Measure transfer wall time + front-half refresh period. */
      nyabula_disp_audit_transfer_start(job.screen_id, job.start_line);

      ret = ioctl(scr->fd, LCDDEVIO_PUTAREA, (unsigned long)&lcd_area);
      if (ret < 0)
        {
          /* Log only on failure (does not spam the log). */
          lcdinfo("screen %d PUTAREA rows %u-%u failed: %d\n", job.screen_id,
                  lcd_area.row_start, lcd_area.row_end, -ret);
        }

      nyabula_disp_audit_transfer_end(job.screen_id);

      /* Lines physically written to the panel for this screen. */
      sem_wait(&scr->st_mutex);
      scr->written_lines += (uint32_t)job.num_lines;

      /* If this was the BACK half, the whole generation is now in GRAM and
       * the locked buffer is free for LVGL to reuse. */
      if (job.start_line >= scr->half_lines && job.num_lines > 0)
        {
          if (scr->fill_pending > 0)
            {
              scr->fill_pending--;
            }

          /* The locked buffer may now be handed back to LVGL. */
          if (scr->half_buf == job.buf_idx)
            {
              scr->half_buf = -1;
            }

          sem_post(&scr->buf_free);
        }

      sem_post(&scr->st_mutex);
    }

  return NULL;
}

/****************************************************************************
 * Name: flush_wait_cb
 *
 * Description:
 *   LVGL flush-wait callback, replacing LVGL's default busy `while(flushing)`
 *   spin with a blocking-but-CPU-free wait, and implementing the v6
 *   "temporarily defer when both buffers are full" semantics (see
 *   Simulation._db_ok / _db_pending).
 *
 *   Called by LVGL (in draw_buf_flush) right before it begins drawing into
 *   the next buffer.  fill_pending counts "rendered but not yet fully
 *   written to GRAM" generations (physical buffer count).  When BOTH
 *   buffers are full (fill_pending >= 2), a new render would need a third
 *   buffer, so we defer -- i.e. wait (without eating CPU) for the transfer
 *   thread to finish a back-half write, release a buffer and post buf_free.
 *   When fill_pending < 2 the pipeline is legal (1 in-flight + 1 render
 *   working buffer) and we return immediately, exactly like v6's _db_ok.
 *
 *   Under a healthy bus this never waits longer than one back-half DMA;
 *   transient heavy backpressure merely pauses rendering (not permanently
 *   frozen) until a buffer is released.  This matches v6: deferred, not
 *   dropped, and recovers automatically.
 *
 ****************************************************************************/

static void flush_wait_cb(lv_display_t *disp)
{
  nyabula_screen_t *scr = lv_display_get_driver_data(disp);

  if (!scr || !scr->initialized)
    {
      return;
    }

  nyabula_disp_audit_flush_wait_start(scr->screen_id);

  sem_wait(&scr->st_mutex);
  while (scr->fill_pending >= NYABULA_DUAL_LCD_MAX_SCREENS && scr->initialized)
    {
      /* Both offscreen buffers hold rendered-but-unwritten generations: a
       * new frame needs a third buffer.  Defer until the transfer thread
       * writes a back half, releases a buffer and posts buf_free.  The
       * wait is signalled (no CPU burn). */
      sem_post(&scr->st_mutex);
      sem_wait(&scr->buf_free);
      sem_wait(&scr->st_mutex);
    }

  sem_post(&scr->st_mutex);

  nyabula_disp_audit_flush_wait_end(scr->screen_id);
}

/****************************************************************************
 * Name: flush_cb
 *
 * Description:
 *   LVGL flush callback, called from the render thread when LVGL finishes
 *   rendering a full frame into one of the two buffers.  Records the
 *   buffer's fresh generation so the v6 edge handlers can push its halves,
 *   and releases LVGL to continue rendering the other buffer.
 *
 *   Waiting for a full double-buffer is handled by flush_wait_cb() (above)
 *   in LVGL's own wait slot, so this callback never blocks the render
 *   thread itself.
 *
 ****************************************************************************/

static void flush_cb(lv_display_t *disp, const lv_area_t *area,
                     uint8_t *color_p)
{
  nyabula_screen_t *scr = lv_display_get_driver_data(disp);
  int buf_idx;

  (void)area; /* Full-refresh: LVGL flushes the whole screen each frame. */

  buf_idx = (color_p == scr->buf[0].data) ? 0 : 1;

  nyabula_disp_audit_flush_end(scr->screen_id);

  sem_wait(&scr->st_mutex);

  if (scr->initialized)
    {
      /* This buffer now holds a fresh full generation. */
      scr->gen_counter++;
      scr->buf[buf_idx].gen = scr->gen_counter;
      scr->render_done_buf = buf_idx;
      scr->fill_pending++;
    }

  sem_post(&scr->st_mutex);

  /* Release this buffer so LVGL resumes rendering into the other one. */
  lv_display_flush_ready(disp);
}

/****************************************************************************
 * Name: display_release_cb
 *
 * Description:
 *   Called when the LVGL display is deleted; closes the LCD fd and detaches
 *   the flush callback.
 *
 ****************************************************************************/

static void display_release_cb(lv_event_t *e)
{
  lv_display_t *disp;
  nyabula_screen_t *scr;

  disp = (lv_display_t *)lv_event_get_user_data(e);
  scr = lv_display_get_driver_data(disp);

  if (scr)
    {
      lv_display_set_driver_data(disp, NULL);
      lv_display_set_flush_cb(disp, NULL);

      /* Symmetric power-down: we powered the panel on via SETPOWER in
       * screen_display_on(), so turn it off before the fd is closed. */
      if (scr->fd >= 0)
        {
          ioctl(scr->fd, LCDDEVIO_SETPOWER, (unsigned long)0);
        }

      if (scr->fd >= 0)
        {
          close(scr->fd);
          scr->fd = -1;
        }

      LV_LOG_USER("Screen released");
    }
}

/****************************************************************************
 * Name: screen_display_on
 *
 * Description:
 *   Before the LVGL pipeline has rendered anything, the panel GRAM still
 *   holds whatever garbage it was powered up with.  Powering the panel on
 *   at that point would flash uninitialized content (snow) until the first
 *   frame is written.  To avoid that, this helper writes one full black
 *   frame to GRAM with a blocking LCDDEVIO_PUTAREA, then turns the panel on
 *   with LCDDEVIO_SETPOWER so the panel comes up to stable black instead of
 *   garbage.  This deliberately does not depend on the TE/vsync scheduler:
 *   a plain direct GRAM fill is always valid before any live rendering.
 *
 * Input Parameters:
 *   scr - The screen to pre-fill and power on.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.  On failure the
 *   panel is left unpowered (or partially filled), so it may show startup
 *   noise until a real frame is rendered.
 *
 ****************************************************************************/

static int screen_display_on(nyabula_screen_t *scr)
{
  struct lcddev_area_s area;
  uint8_t *black;
  int ret;

  /* Allocate a temporary full-frame black buffer.  It is plain malloc'd
   * memory: st77916_putarea() stages it into its own DMA-safe buffer, so
   * no special alignment is required here. */
  black = lv_malloc(scr->buf_size);
  if (black == NULL)
    {
      LV_LOG_ERROR("Screen %d: failed to allocate black fill buffer",
                   scr->screen_id);
      return -ENOMEM;
    }

  memset(black, 0, scr->buf_size);

  /* Fill the whole GRAM with black. */
  memset(&area, 0, sizeof(area));
  area.row_start = 0;
  area.row_end = scr->height - 1;
  area.col_start = 0;
  area.col_end = scr->width - 1;
  area.stride = scr->stride;
  area.data = black;

  ret = ioctl(scr->fd, LCDDEVIO_PUTAREA, (unsigned long)&area);
  if (ret < 0)
    {
      LV_LOG_ERROR("Screen %d: black fill PUTAREA failed: %d", scr->screen_id,
                   -ret);
      lv_free(black);
      return ret;
    }

  lv_free(black);

  /* Turn the panel on now that its GRAM holds stable black. */
  ret = ioctl(scr->fd, LCDDEVIO_SETPOWER, LCD_FULL_ON);
  if (ret < 0)
    {
      LV_LOG_ERROR("Screen %d: SETPOWER failed: %d", scr->screen_id, -ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: screen_init
 *
 * Description:
 *   Initialize a single screen: open the LCD device, allocate two full-frame
 *   RGB565 buffers, set up the v6 scheduler state and the LVGL display.
 *
 ****************************************************************************/

static int screen_init(nyabula_screen_t *scr, int sid, const char *dev_path,
                       int width, int height)
{
  int ret;

  scr->screen_id = sid;
  scr->width = width;
  scr->height = height;
  scr->stride = width * 2; /* RGB565: 2 bytes per pixel */
  scr->total_lines = height;
  scr->half_lines = height / 2;
  scr->buf_size = (uint32_t)width * height * 2;
  scr->gen_counter = 0;
  scr->render_done_buf = -1;
  scr->half_buf = -1;
  scr->submitted_lines = 0;
  scr->written_lines = 0;
  scr->fill_pending = 0;
  scr->render_request = false;
  scr->initialized = false;

  /* 0 lines written by the driver yet: prime the written counter so the
   * backpressure bookkeeping starts consistent. */

  /* Open LCD device */
  LV_LOG_USER("Opening LCD device: %s", dev_path);
  scr->fd = open(dev_path, O_RDWR);
  if (scr->fd < 0)
    {
      LV_LOG_ERROR("Failed to open %s: %d", dev_path, errno);
      return -1;
    }

  /* Get alignment info */
  ret = ioctl(scr->fd, LCDDEVIO_GETAREAALIGN, &scr->align);
  if (ret < 0)
    {
      LV_LOG_WARN("ioctl(GETAREAALIGN) failed, using defaults");
      scr->align.col_start_align = 1;
      scr->align.row_start_align = 1;
      scr->align.width_align = 1;
      scr->align.height_align = 1;
    }

  /* Allocate the two full-frame RGB565 double buffers (the buffer count is
   * fixed at two).  On failure, fall into the cleanup labels in order of how
   * many buffers were actually allocated. */
  scr->buf[0].data = lv_malloc(scr->buf_size);
  if (!scr->buf[0].data)
    {
      LV_LOG_ERROR("Failed to allocate buffer 0 (%u bytes)", scr->buf_size);
      goto err_fd; /* nothing allocated yet; just close the fd */
    }

  scr->buf[0].gen = -1;
  memset(scr->buf[0].data, 0, scr->buf_size);

  scr->buf[1].data = lv_malloc(scr->buf_size);
  if (!scr->buf[1].data)
    {
      LV_LOG_ERROR("Failed to allocate buffer 1 (%u bytes)", scr->buf_size);
      goto err_buf0;
    }

  scr->buf[1].gen = -1;
  memset(scr->buf[1].data, 0, scr->buf_size);

  /* Initialize sync primitives */
  sem_init(&scr->buf_free, 0, 0);
  sem_init(&scr->st_mutex, 0, 1);

  /* Create LVGL display with full-refresh double buffering. */
  scr->disp = lv_display_create(width, height);
  if (!scr->disp)
    {
      LV_LOG_ERROR("Failed to create LVGL display");
      goto err_disp;
    }

  /* LVGL renders one full frame per buffer and calls flush_cb once for the
   * whole screen; this matches the v6 whole-half-block model. */
  lv_display_set_buffers(scr->disp, scr->buf[0].data, scr->buf[1].data,
                         scr->buf_size, LV_DISPLAY_RENDER_MODE_FULL);
  lv_display_set_flush_cb(scr->disp, flush_cb);
  lv_display_set_flush_wait_cb(scr->disp, flush_wait_cb);
  lv_display_set_driver_data(scr->disp, scr);

  /* Retain the default refresh timer: lv_refr_now() needs a non-NULL
   * disp->refr_timer handle to drive a frame (it guards on
   * `if(disp->refr_timer)`).  But we neutralize its auto-running via
   * te_refr_req_cb(), which pauses it on every REFR_REQUEST (after LVGL's
   * built-in handler has resumed it), so rendering is driven exclusively
   * by the TE falling edge through explicit lv_refr_now(). */
  lv_display_add_event_cb(scr->disp, rounder_cb, LV_EVENT_INVALIDATE_AREA,
                          scr);
  lv_display_add_event_cb(scr->disp, te_refr_req_cb, LV_EVENT_REFR_REQUEST,
                          scr);
  lv_display_add_event_cb(scr->disp, display_release_cb, LV_EVENT_DELETE,
                          scr->disp);

  scr->initialized = true;
  LV_LOG_USER("Screen initialized: %dx%d, stride=%d, buf_size=%u", width,
              height, scr->stride, scr->buf_size);

  /* Pre-fill GRAM with black and power the panel on.  This happens before
   * any live rendering so the panel never shows uninitialized GRAM garbage;
   * the TEM/vsync pipeline then refreshes real frames on top. */
  ret = screen_display_on(scr);
  if (ret < 0)
    {
      LV_LOG_WARN("Screen %d display-on incomplete: %d", sid, -ret);
      /* Non-fatal: rendering still works, the panel just may have shown
       * startup noise before the first real frame. */
    }

  return 0;

err_disp:
  sem_destroy(&scr->buf_free);
  sem_destroy(&scr->st_mutex);
  lv_free(scr->buf[1].data);
  scr->buf[1].data = NULL;
  lv_free(scr->buf[0].data);
  scr->buf[0].data = NULL;
  goto err_fd;

err_buf0:
  lv_free(scr->buf[0].data);
  scr->buf[0].data = NULL;
  /* fall through */

err_fd:
  close(scr->fd);
  scr->fd = -1;
  return -1;
}

/****************************************************************************
 * Name: screen_destroy
 *
 * Description:
 *   Tear down a single screen and release all resources.
 *
 ****************************************************************************/

static void screen_destroy(nyabula_screen_t *scr)
{
  if (!scr->initialized)
    {
      return;
    }

  if (scr->disp)
    {
      lv_display_delete(scr->disp);
      scr->disp = NULL;
    }

  /* Symmetric power-down (idempotent: display_release_cb may already have
   * turned the panel off and closed the fd on lv_display_delete above). */
  if (scr->fd >= 0)
    {
      ioctl(scr->fd, LCDDEVIO_SETPOWER, (unsigned long)0);
    }

  /* Release the two double buffers (double-buffered pipeline). */
  if (scr->buf[0].data)
    {
      lv_free(scr->buf[0].data);
      scr->buf[0].data = NULL;
    }

  if (scr->buf[1].data)
    {
      lv_free(scr->buf[1].data);
      scr->buf[1].data = NULL;
    }

  if (scr->fd >= 0)
    {
      close(scr->fd);
      scr->fd = -1;
    }

  sem_destroy(&scr->buf_free);
  sem_destroy(&scr->st_mutex);

  scr->initialized = false;
  LV_LOG_USER("Screen destroyed");
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nyabula_dual_lcd_create
 *
 * Description:
 *   Create the dual-LCD context with the two threads (software TE frame
 *   clock + transfer) and the v6 BoundedChase scheduler.
 *
 ****************************************************************************/

nyabula_dual_lcd_t *nyabula_dual_lcd_create(const char *dev_path0,
                                            const char *dev_path1, int width,
                                            int height)
{
  nyabula_dual_lcd_t *dual;
  int ret;

  LV_ASSERT_NULL(dev_path0);
  LV_ASSERT_NULL(dev_path1);

  if (width <= 0)
    {
      width = NYABULA_DUAL_LCD_DEF_WIDTH;
    }

  if (height <= 0)
    {
      height = NYABULA_DUAL_LCD_DEF_HEIGHT;
    }

  dual = lv_malloc_zeroed(sizeof(nyabula_dual_lcd_t));
  if (!dual)
    {
      LV_LOG_ERROR("Failed to allocate dual LCD context");
      return NULL;
    }

  dual->running = false;
  dual->job_head = 0;
  dual->job_tail = 0;
  dual->job_count = 0;
  dual->last_bus_screen = -1;

  sem_init(&dual->job_mutex, 0, 1);
  sem_init(&dual->job_avail, 0, 0);
  sem_init(&dual->job_space, 0, NYABULA_JOB_QUEUE_SIZE);
  sem_init(&dual->render_kick, 0, 0);

  nyabula_disp_audit_init();

  /* Initialize screen 0 */
  ret = screen_init(&dual->screen[0], 0, dev_path0, width, height);
  if (ret < 0)
    {
      LV_LOG_ERROR("Failed to initialize screen 0");
      goto err_screen0;
    }

  /* Initialize screen 1 */
  ret = screen_init(&dual->screen[1], 1, dev_path1, width, height);
  if (ret < 0)
    {
      LV_LOG_ERROR("Failed to initialize screen 1");
      goto err_screen1;
    }

  /* Stagger the software TE clocks: screen 1 leads screen 0 by 90-deg (1/4
   * frame period) so their scan edges -- and the resulting QSPI write
   * submissions -- do not collide.  Screen 0 keeps the default phase 0. */
  dual->screen[1].te_phase_shift = 1;

  dual->running = true;

  /* Start the software TE frame clock thread (placeholder for TE ISR). */
  ret = pthread_create(&dual->te_thread, NULL, te_thread_func, dual);
  if (ret != 0)
    {
      LV_LOG_ERROR("Failed to create TE thread: %d", ret);
      dual->running = false;
      goto err_te_thread;
    }

  /* Start the transfer (QSPI DMA) thread. */
  ret =
      pthread_create(&dual->transfer_thread, NULL, transfer_thread_func, dual);
  if (ret != 0)
    {
      LV_LOG_ERROR("Failed to create transfer thread: %d", ret);
      dual->running = false;
      pthread_join(dual->te_thread, NULL);
      goto err_te_thread; /* te thread already joined; fall into cleanup */
    }

  LV_LOG_USER("Dual LCD created successfully (v6 bounded chase, %d fps); "
              "run nyabula_dual_lcd_task() in a loop to drive rendering",
              NYABULA_DUAL_LCD_REFRESH_HZ);
  return dual;

err_te_thread:
  screen_destroy(&dual->screen[0]);
  screen_destroy(&dual->screen[1]);
  /* fall through */

err_screen1:
  screen_destroy(&dual->screen[0]);
  /* fall through */

err_screen0:
  sem_destroy(&dual->job_mutex);
  sem_destroy(&dual->job_avail);
  sem_destroy(&dual->job_space);
  sem_destroy(&dual->render_kick);
  lv_free(dual);
  return NULL;
}

/****************************************************************************
 * Name: nyabula_dual_lcd_destroy
 *
 * Description:
 *   Stop all threads and release resources.
 *
 ****************************************************************************/

void nyabula_dual_lcd_destroy(nyabula_dual_lcd_t *dual)
{
  if (!dual)
    {
      return;
    }

  /* Stop the threads. */
  dual->running = false;

  /* Wake the TE thread (sleeping between edges -> it checks running on the
   * next usleep boundary) and the transfer thread (blocked on job_avail).
   * The render loop is hosted by the caller, not joined here. */
  sem_post(&dual->render_kick);
  sem_post(&dual->job_avail);

  pthread_join(dual->te_thread, NULL);
  pthread_join(dual->transfer_thread, NULL);

  screen_destroy(&dual->screen[0]);
  screen_destroy(&dual->screen[1]);

  sem_destroy(&dual->job_mutex);
  sem_destroy(&dual->job_avail);
  sem_destroy(&dual->job_space);
  sem_destroy(&dual->render_kick);

  lv_free(dual);
  LV_LOG_USER("Dual LCD destroyed");
}

/****************************************************************************
 * Name: nyabula_dual_lcd_get_display
 *
 * Description:
 *   Get the LVGL display instance for a specific screen.
 *
 ****************************************************************************/

lv_display_t *nyabula_dual_lcd_get_display(nyabula_dual_lcd_t *dual,
                                           int screen_id)
{
  if (!dual || screen_id < 0 || screen_id >= NYABULA_DUAL_LCD_MAX_SCREENS)
    {
      LV_LOG_ERROR("Invalid dual LCD context or screen ID: %d", screen_id);
      return NULL;
    }

  if (!dual->screen[screen_id].initialized)
    {
      LV_LOG_ERROR("Screen %d not initialized", screen_id);
      return NULL;
    }

  return dual->screen[screen_id].disp;
}

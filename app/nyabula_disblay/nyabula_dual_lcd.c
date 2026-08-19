/****************************************************************************
 * apps/graphics/nyabula_display/nyabula_dual_lcd.c
 *
 * Dual-screen dual-buffer LVGL interface layered on the v8 "BoundedChase"
 * frame scheduler (an in-house dual-panel scheduling algorithm; its C port
 * lives in nyabula_v8_scheduler.c).
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
 * instant only one screen can be written.  This layer is the FRAMEWORK:
 * it owns the threads, the LVGL integration and the DMA (PUTAREA) device
 * access, and passes all *scheduling decisions* to the v8 algorithm
 * (nyabula_v8_scheduler.c).  Framework and algorithm are fully decoupled:
 * the algorithm only consumes the six callbacks/requests declared in
 * nyabula_v8_scheduler.h and knows nothing about LVGL or DMA.
 *
 * Thread model
 *   - Render loop    : logically a render thread, hosted by the caller
 *                   (main) via nyabula_dual_lcd_task(), which runs
 *                   lv_timer_handler() (LVGL is single-threaded).
 *   - TE thread       : software frame clock standing in for the real TE
 *                     signal.  Fires the scan edges at the frame rate and
 *                     feeds the ISR-safe edge handlers.  The two screens'
 *                     clocks are staggered 90-deg to de-correlate their QSPI
 *                     write bursts.  High priority for correct timing;
 *                     swap for a gpio ISR once hardware TE is wired.
 *   - Transfer thread: pops half-block write jobs and issues a BLOCKING
 *                     LCDDEVIO_PUTAREA ioctl (QSPI DMA).  It does not
 *                     consume CPU while the DMA is in flight.
 *
 * TE signal / state machine
 *   The panel TE is not yet wired in hardware, so a software frame clock
 *   synthesizes the two edges of the scan and forwards each to the v8
 *   algorithm via nyabula_v8_on_scan_half() (falling edge, front half
 *   scanned) and nyabula_v8_on_scan_start() (rising edge, full frame
 *   scanned).  All "which half, which generation, when" decisions live in
 *   the algorithm; the framework only executes the resulting write jobs on
 *   the DMA thread and reports block completion back to the algorithm.
 *
 *   The edge entry points are kept ISR-safe: they only forward to v8 and
 *   post semaphores.  When the real TE signal is routed (gpio_irqattach),
 *   the same functions are called from an ISR with no change.
 *
 * Framework <-> algorithm interface (see nyabula_v8_scheduler.h):
 *   framework -> algorithm : on_scan_start / on_scan_half / on_render_done /
 *                            on_block_write_done
 *   algorithm -> framework : request_render / request_write / on_gen_complete
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "nyabula_dual_lcd.h"

#include <debug.h>
#include <errno.h>
#include <fcntl.h>
#include <nuttx/lcd/lcd_dev.h>
#include <sched.h>
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

/* Create a thread with an explicit FIFO priority (lower number = higher). */
static int create_thread_prio(pthread_t *thr, void *(*fn)(void *), void *arg,
                              int prio);

/* TE edge forwarding to the v8 algorithm (ISR-safe). */
static void te_edge_rise(nyabula_dual_lcd_t *dual, int sid);
static void te_edge_fall(nyabula_dual_lcd_t *dual, int sid);

/* v8 request callbacks (algorithm -> framework). */
static void v8_request_render(struct nyabula_screen *scr, int gen);
static bool v8_request_write(struct nyabula_screen *scr, int start_line,
                             int num_lines, int gen);
static void v8_on_gen_complete(struct nyabula_screen *scr, int gen);

/* Job queue (strict FIFO, bounded). */
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
 * v8 request callbacks (algorithm -> framework)
 ****************************************************************************/

/* Map a generation number to the physical offscreen buffer currently
 * holding that generation's rendered content, or -1 if none.  The v8
 * algorithm keys everything by generation; this lookup is the only place
 * the framework resolves a gen back to a concrete buffer index + base
 * address for the DMA write. */

static int buf_idx_of_gen(const nyabula_screen_t *scr, int gen)
{
  if (gen >= 0 && scr->buf[0].gen == gen)
    {
      return 0;
    }

  if (gen >= 0 && scr->buf[1].gen == gen)
    {
      return 1;
    }

  return -1;
}

/* algorithm -> framework: request render of a new generation for a screen.
 * Runs in the TE edge context (ISR-safe): only posts a render request for
 * the render loop to consume.  The actual lv_refr_now() happens later, on
 * the caller's thread in nyabula_dual_lcd_task(). */

static void v8_request_render(struct nyabula_screen *scr, int gen)
{
  (void)gen; /* the algorithm already assigned gen; we just mark "render". */

  if (!scr->initialized)
    {
      return;
    }

  /* Record the generation LVGL is about to draw.  When flush_cb fires, the
   * framework binds this generation to the physical buffer LVGL chose and
   * reports on_render_done back to the algorithm. */
  sem_wait(&scr->st_mutex);
  scr->render_gen = gen;
  scr->render_request = true;
  sem_post(&scr->st_mutex);
}

/* algorithm -> framework: request a half-frame block write of `num_lines`
 * lines starting at `start_line`, carrying generation `gen`.  Runs in the
 * TE edge context; enqueues a job for the transfer thread (DMA).  Returns
 * true if the job was enqueued, false if the queue is full (block dropped). */

static bool v8_request_write(struct nyabula_screen *scr, int start_line,
                             int num_lines, int gen)
{
  nyabula_dual_lcd_t *dual;
  nyabula_write_job_t job;
  int idx;
  int ret;

  if (num_lines <= 0)
    {
      return false;
    }

  dual = (nyabula_dual_lcd_t *)scr->dual;
  if (!dual || dual->quitting)
    {
      return false;
    }

  /* Resolve the generation back to a concrete buffer.  The algorithm only
   * ever writes generations that are already rendered (in _pending_gens),
   * so this must succeed; if it does not, drop defensively. */
  sem_wait(&scr->st_mutex);
  idx = buf_idx_of_gen(scr, gen);
  if (idx < 0)
    {
      sem_post(&scr->st_mutex);
      return false;
    }

  memset(&job, 0, sizeof(job));
  job.screen_id = scr->screen_id;
  job.start_line = start_line;
  job.num_lines = num_lines;
  job.buf_idx = idx;
  job.gen = gen;

  sem_post(&scr->st_mutex);

  ret = job_enqueue(dual, &job);
  return (ret == 0);
}

/* algorithm -> framework: a generation's two blocks are both fully written
 * to GRAM; its offscreen buffer is free for LVGL to reuse.  Post buf_free
 * so a blocked flush_wait_cb (or the render defer wait) can proceed. */

static void v8_on_gen_complete(struct nyabula_screen *scr, int gen)
{
  int idx;

  if (!scr->initialized)
    {
      return;
    }

  sem_wait(&scr->st_mutex);
  idx = buf_idx_of_gen(scr, gen);
  if (idx >= 0)
    {
      scr->buf[idx].gen = -1; /* buffer is now free */
    }
  sem_post(&scr->st_mutex);

  sem_post(&scr->buf_free);
}

/****************************************************************************
 * Name: te_edge_rise
 *
 * Description:
 *   Rising edge of the scan (a full frame has been scanned).  Forwarded to
 *   the v8 algorithm, which decides whether to write the back half.
 *
 *   ISR-safe: only forwards state and posts semaphores.
 *
 ****************************************************************************/

static void te_edge_rise(nyabula_dual_lcd_t *dual, int sid)
{
  nyabula_screen_t *scr;

  if (dual->quitting)
    {
      return;
    }

  scr = &dual->screen[sid];
  if (!scr->initialized)
    {
      return;
    }

  /* The edge handlers may already hold st_mutex via request callbacks that
   * also take it; to avoid recursive deadlock the algorithm itself is not
   * guarded by st_mutex here (its state is only touched from the single TE
   * thread and the transfer thread's completion callback, see _on_block_*
   * synchronization note).  Forward to v8. */
  nyabula_v8_on_scan_start(&scr->v8, scr);
}

/****************************************************************************
 * Name: te_edge_fall
 *
 * Description:
 *   Falling edge of the scan: the panel has scanned rows 0..half-1.  Forwarded
 *   to the v8 algorithm, which locks the latest rendered generation, submits
 *   the front half and kicks the next frame render.
 *
 *   ISR-safe: only forwards state and posts semaphores.
 *
 ****************************************************************************/

static void te_edge_fall(nyabula_dual_lcd_t *dual, int sid)
{
  nyabula_screen_t *scr;

  if (dual->quitting)
    {
      return;
    }

  scr = &dual->screen[sid];
  if (!scr->initialized)
    {
      return;
    }

  nyabula_v8_on_scan_half(&scr->v8, scr);

  /* Wake the render loop once per falling edge so the pending render
   * request (posted by v8_request_render via the algorithm) is consumed
   * promptly at the frame boundary. */
  sem_post(&dual->render_kick);
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
  /* Strict FIFO: the shared bus serves write jobs in submission order
   * (first-come first-served), matching the physical reality of a single
   * controller / single bus.  Fairness between screens is the ALGORITHM's
   * job at submission time, not the framework's.  The old round-robin
   * reorder here was a v6 leftover that violated this boundary. */
  sem_wait(&dual->job_avail);
  sem_wait(&dual->job_mutex);

  *out = dual->job_queue[dual->job_head];
  dual->job_head = (dual->job_head + 1) % NYABULA_JOB_QUEUE_SIZE;
  dual->job_count--;
  sem_post(&dual->job_mutex);
  sem_post(&dual->job_space);

  return 0;
}

/****************************************************************************
 * Name: create_thread_prio
 *
 * Description:
 *   Create a pthread with an explicit SCHED_FIFO priority (lower number =
 *   higher priority in NuttX).  Used to give the software TE frame clock a
 *   high priority so its edges fire on time and are not delayed by the
 *   render or transfer threads.
 *
 ****************************************************************************/

static int create_thread_prio(pthread_t *thr, void *(*fn)(void *), void *arg,
                              int prio)
{
  pthread_attr_t attr;
  struct sched_param param;
  int ret;

  pthread_attr_init(&attr);
  param.sched_priority = prio;
  ret = pthread_attr_setschedparam(&attr, &param);
  if (ret != 0)
    {
      pthread_attr_destroy(&attr);
      return ret;
    }

  ret = pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
  if (ret != 0)
    {
      pthread_attr_destroy(&attr);
      return ret;
    }

  ret = pthread_create(thr, &attr, fn, arg);
  pthread_attr_destroy(&attr);
  return ret;
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
 *        (FULL mode) to feed the v8 pipeline.
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
  int sid;

  if (!dual)
    {
      return;
    }

  /* Run one cycle of the LVGL state machine (timers, animation).  This
   * advances animations (time from the NuttX clock via lv_tick_set_cb) and
   * accumulates their invalidations; the default refresh timer is kept
   * paused (see te_refr_req_cb), so it does NOT itself render.  The render
   * happens below via lv_refr_now() driven by the v8 algorithm's render
   * request, keeping the pipeline TE-aligned. */
  idle = lv_timer_handler();

  /* Consume TE/v8-driven render requests: for each screen whose algorithm
   * requested a frame, issue lv_refr_now().  lv_refr_now() advances
   * animations too and skips drawing when there is no invalid area.  The
   * double-buffer gating is entirely the algorithm's job (v8 _can_render):
   * it only requests a render when a working buffer is available, so the
   * render loop never has to decide deferral here. */
  for (sid = 0; sid < NYABULA_DUAL_LCD_MAX_SCREENS; sid++)
    {
      nyabula_screen_t *scr = &dual->screen[sid];
      bool request;

      if (!scr->initialized || !scr->disp)
        {
          continue;
        }

      sem_wait(&scr->st_mutex);
      request = scr->render_request;
      if (request)
        {
          scr->render_request = false;
        }

      sem_post(&scr->st_mutex);

      if (request)
        {
          lv_refr_now(scr->disp);
        }
    }

  /* Wait for either a TE edge kick (posted by the TE thread so we service
   * lv_timer_handler() promptly at the frame boundary) or until the LVGL
   * idle period elapses -- whichever comes first.  The timeout is computed
   * relative to NOW so the wait never extends beyond `idle` ms from this
   * moment, keeping the LVGL clock true. */
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

  sem_timedwait(&dual->render_kick, &ts);
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
 *   After each block completes, the edge is reported to the v8 algorithm
 *   (nyabula_v8_on_block_write_done), which decrements its in-flight count
 *   and self-determines generation completion (freeing the double buffer).
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

      ret = ioctl(scr->fd, LCDDEVIO_PUTAREA, (unsigned long)&lcd_area);
      if (ret < 0)
        {
          /* Log only on failure (does not spam the log). */
          lcdinfo("screen %d PUTAREA rows %u-%u failed: %d\n", job.screen_id,
                  lcd_area.row_start, lcd_area.row_end, -ret);
        }

      /* Block transfer complete: report the edge to the v8 algorithm, which
       * decrements its in-flight count and self-determines whether the whole
       * generation is now written (releasing the double buffer).  The DMA
       * hardware gives us exactly one interrupt per submitted block, which
       * is the physical signal v8 keys its bookkeeping on. */
      nyabula_v8_on_block_write_done(&scr->v8, job.gen);
    }

  return NULL;
}

/****************************************************************************
 * Name: flush_wait_cb
 *
 * Description:
 *   LVGL flush-wait callback, replacing LVGL's default busy `while(flushing)`
 *   spin with a blocking-but-CPU-free wait.
 *
 *   A buffer is "free" once its generation has been fully written to GRAM
 *   (gen set back to -1 by on_gen_complete, which posts buf_free).  Wait
 *   here while BOTH offscreen buffers are still occupied (gen != -1), i.e.
 *   a new draw would overwrite content whose DMA transfer has not finished.
 *   Once at least one buffer is free, return immediately.
 *
 *   The v8 algorithm already guarantees it only requests a render when a
 *   working buffer is available (_can_render), so under a healthy bus this
 *   rarely blocks; it is a defensive net for LVGL's internal buffer
 *   alternation racing a slow back-half DMA.
 *
 ****************************************************************************/

static void flush_wait_cb(lv_display_t *disp)
{
  nyabula_screen_t *scr = lv_display_get_driver_data(disp);
  bool both_busy;

  if (!scr || !scr->initialized)
    {
      return;
    }

  sem_wait(&scr->st_mutex);
  both_busy = (scr->buf[0].gen != -1 && scr->buf[1].gen != -1);
  while (both_busy && scr->initialized)
    {
      /* Both offscreen buffers hold rendered-but-unwritten content: a new
       * draw would need a third buffer.  Wait (signalled) for the transfer
       * thread to finish a generation and post buf_free. */
      sem_post(&scr->st_mutex);
      sem_wait(&scr->buf_free);
      sem_wait(&scr->st_mutex);
      both_busy = (scr->buf[0].gen != -1 && scr->buf[1].gen != -1);
    }

  sem_post(&scr->st_mutex);
}

/****************************************************************************
 * Name: flush_cb
 *
 * Description:
 *   LVGL flush callback, called from the render thread when LVGL finishes
 *   rendering a full frame into one of the two buffers.  Binds the
 *   generation (assigned by the v8 algorithm via request_render, stored in
 *   scr->render_gen) to the physical buffer LVGL chose, then reports
 *   on_render_done so the algorithm moves the generation into its pending
 *   (awaiting-write) set.
 *
 *   A full double-buffer wait is handled by flush_wait_cb() above in LVGL's
 *   own wait slot, so this callback never blocks the render thread itself.
 *
 ****************************************************************************/

static void flush_cb(lv_display_t *disp, const lv_area_t *area,
                     uint8_t *color_p)
{
  nyabula_screen_t *scr = lv_display_get_driver_data(disp);
  int gen;
  int buf_idx;
  int done_gen;

  (void)area; /* Full-refresh: LVGL flushes the whole screen each frame. */

  buf_idx = (color_p == scr->buf[0].data) ? 0 : 1;

  sem_wait(&scr->st_mutex);

  /* The generation LVGL was asked to draw (request_render) is now realized
   * in this physical buffer.  Bind it and report completion to the v8
   * algorithm. */
  gen = scr->render_gen;
  done_gen = -1;
  if (scr->initialized && gen != -1)
    {
      scr->buf[buf_idx].gen = gen;
      done_gen = gen;
      scr->render_gen = -1;
    }

  sem_post(&scr->st_mutex);

  if (done_gen != -1)
    {
      /* Content is ready: the algorithm moves done_gen from "rendering" to
       * "pending write".  Called outside st_mutex (the algorithm's own
       * state is not shared with the framework's st_mutex). */
      nyabula_v8_on_render_done(&scr->v8, done_gen);
    }

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
 *   RGB565 buffers, set up the LVGL display.  The v8 algorithm context is
 *   initialized (with callbacks) by the caller after the dual context is
 *   built, since it needs the dual back-pointer.
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
  scr->render_gen = -1;
  scr->render_request = false;
  scr->dual = NULL; /* set by the caller (nyabula_dual_lcd_create) */
  scr->initialized = false;

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
   * whole screen; this matches the v8 whole-half-block model. */
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
 *   clock + transfer) and wire the v8 scheduling algorithm by registering
 *   its request callbacks.
 *
 ****************************************************************************/

nyabula_dual_lcd_t *nyabula_dual_lcd_create(const char *dev_path0,
                                            const char *dev_path1, int width,
                                            int height)
{
  nyabula_dual_lcd_t *dual;
  nyabula_v8_callbacks_t v8_cb;
  int sid;
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

  sem_init(&dual->job_mutex, 0, 1);
  sem_init(&dual->job_avail, 0, 0);
  sem_init(&dual->job_space, 0, NYABULA_JOB_QUEUE_SIZE);
  sem_init(&dual->render_kick, 0, 0);

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

  /* Wire the v8 algorithm for both screens: register its request callbacks
   * and set the dual back-pointer so the algorithm's write request can reach
   * the job queue.  The algorithm only ever sees the opaque screen handle
   * and these callbacks -- no LVGL/DMA detail. */
  v8_cb.request_render = v8_request_render;
  v8_cb.request_write = v8_request_write;
  v8_cb.on_gen_complete = v8_on_gen_complete;

  for (sid = 0; sid < NYABULA_DUAL_LCD_MAX_SCREENS; sid++)
    {
      dual->screen[sid].dual = dual;
      nyabula_v8_init(&dual->screen[sid].v8, dual->screen[sid].half_lines,
                      dual->screen[sid].total_lines, &v8_cb);
      dual->screen[sid].v8.screen = &dual->screen[sid];
    }

  /* TE clock phase (0 = no stagger): screen 1 leads screen 0 by 0-deg so
   * both screens' scan edges coincide.  This is the harshest operating point
   * for the shared QSPI bus (both screens submit writes at the same instant).
   * Originally 1 (90-deg stagger); set to 0 to stress the scheduler. */
  dual->screen[1].te_phase_shift = 1;

  dual->running = true;

  /* Start the software TE frame clock thread (placeholder for TE ISR), at
   * high priority so edges fire on time regardless of render/transfer load. */
  ret = create_thread_prio(&dual->te_thread, te_thread_func, dual,
                           NYABULA_DUAL_LCD_TE_PRIORITY);
  if (ret != 0)
    {
      LV_LOG_ERROR("Failed to create TE thread: %d", ret);
      dual->running = false;
      goto err_te_thread;
    }

  /* Start the transfer (QSPI DMA) thread at a priority below the TE thread
   * (so TE wins) but above the render/main thread (so DMA keeps busy). */
  ret = create_thread_prio(&dual->transfer_thread, transfer_thread_func, dual,
                           NYABULA_DUAL_LCD_TRANSFER_PRIORITY);
  if (ret != 0)
    {
      LV_LOG_ERROR("Failed to create transfer thread: %d", ret);
      dual->running = false;
      pthread_join(dual->te_thread, NULL);
      goto err_te_thread; /* te thread already joined; fall into cleanup */
    }

  LV_LOG_USER("Dual LCD created successfully (v8 bounded chase, %d fps); "
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

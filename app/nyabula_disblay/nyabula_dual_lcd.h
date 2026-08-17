/****************************************************************************
 * apps/graphics/nyabula_display/nyabula_dual_lcd.h
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

#ifndef __NYABULA_DUAL_LCD_H
#define __NYABULA_DUAL_LCD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <lvgl/lvgl.h>
#include <nuttx/lcd/lcd_dev.h>
#include <pthread.h>
#include <semaphore.h>

#ifdef __cplusplus
extern "C" {
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Maximum number of screens sharing the single QSPI write bus. */

#define NYABULA_DUAL_LCD_MAX_SCREENS 2

/* Default screen geometry (ST77916, 360x360 RGB565). */

#ifndef NYABULA_DUAL_LCD_DEF_WIDTH
#define NYABULA_DUAL_LCD_DEF_WIDTH 360
#endif

#ifndef NYABULA_DUAL_LCD_DEF_HEIGHT
#define NYABULA_DUAL_LCD_DEF_HEIGHT 360
#endif

/* Default refresh rate in Hz driving the software TE signal. */

#ifndef NYABULA_DUAL_LCD_REFRESH_HZ
#define NYABULA_DUAL_LCD_REFRESH_HZ 60
#endif

/* ------------------------------------------------------------------
 * v6 "BoundedChase" scheduler knobs (see vsyncalg/V6_DESIGN.md).
 * Quantities are in LINES so they are independent of panel resolution.
 * ------------------------------------------------------------------ */

/* Single-flight backpressure budget: the maximum number of lines a screen
 * may have "submitted but not yet physically written to GRAM" (in-flight).
 * Equal to HALF the frame (180 lines for 360).  When exceeded, submission
 * is deferred until the bus drains.  This prevents "content freezing" when
 * the shared QSPI bus lacks bandwidth. */

#define NYABULA_PENDING_BUDGET_LINES 180

/* Size of the transfer job queue.  Entry of a job is gated by the
 * backpressure budget, so this only needs a couple of half-blocks per
 * screen plus margin. */

#define NYABULA_JOB_QUEUE_SIZE 8

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* A full-frame RGB565 offscreen buffer.  "generation" (gen) is a
 * monotonically increasing number identifying the content frame held in
 * the buffer.  -1 means "no valid rendered content yet". */

typedef struct
{
  uint8_t *data; /* Buffer base address (buf_size bytes) */
  int gen;       /* Content generation, -1 = invalid / free */
} nyabula_buf_t;

/* Per-screen scheduler state mirroring one v6 BoundedChaseScheduler
 * instance (see vsyncalg/algos/bounded_chase.py). */

typedef struct
{
  /* LCD device (/dev/lcd0 or /dev/lcd1) */
  int fd;
  lv_display_t *disp;

  /* Geometry */
  int width;
  int height;
  int stride;        /* Bytes per scan line (RGB565 => width*2) */
  uint32_t buf_size; /* Bytes per full frame buffer */
  int total_lines;   /* height */
  int half_lines;    /* total_lines / 2 */

  struct lcddev_area_align_s align;

  /* Double buffers (full screen each) */
  nyabula_buf_t buf[2];

  /* ---- v6 state machine (per screen) ---- */
  int gen_counter;          /* Next generation number to assign */
  int render_done_buf;      /* Buffer index of latest fully-rendered frame,
                             *   -1 = none (== _render_done, stored as buf idx) */
  int half_buf;             /* Buffer locked at falling edge for the front+back
                             *   half push, -1 = none (== _half_gen) */
  uint32_t submitted_lines; /* Cumulative lines submitted (monotonic) */
  uint32_t written_lines;   /* Cumulative lines actually written to GRAM */

  /* Double-buffer accounting: how many "rendered but not yet fully written
   * to GRAM" generations exist (physical buffer-count constraint). */
  int fill_pending;

  /* Posted by the transfer thread whenever a buffer generation becomes
   * fully written (back half DMA done) and is therefore free for LVGL to
   * reuse.  The render thread's flush_wait_cb waits here while a target
   * buffer is still locked. */
  sem_t buf_free;

  /* TE-driven render request.  With the default refresh timer deleted, a
   * frame is rendered ONLY when we explicitly call lv_refr_now(); the TE
   * falling edge sets this flag + posts render_kick so the render loop
   * issues exactly one lv_refr_now() for this screen at the frame boundary
   * (v6 "kick render at falling edge").  Guarded by st_mutex. */
  bool render_request;

  /* Serializes access to this screen's scheduler state (render_done_buf,
   * half_buf, fill_pending, gen_counter, buf[].gen) shared between the TE
   * thread, the transfer thread and the render thread (flush_cb).
   * The edge handlers (te_edge_*) are intended to be usable from a real
   * TE ISR, so this is a mutex-like binary semaphore that is safe to
   * post from both thread and (upcoming) interrupt context. */
  sem_t st_mutex;

  /* Software TE phase offset, in quarter-frame ticks (0..3).  Screen 0
   * uses 0; screen 1 uses 1 so its scan edges are shifted 90-deg (1/4 frame
   * period) relative to screen 0.  This staggers the two screens' write
   * requests so the shared QSPI bus is not hit by both screens at exactly
   * the same instant (a milder operating point for the scheduler). */
  uint8_t te_phase_shift;

  bool initialized;
} nyabula_screen_t;

/* A pending write block queued for the transfer thread: a half-frame
 * (front or back). */
typedef struct
{
  int screen_id;  /* 0 or 1 */
  int start_line; /* Row offset within GRAM (0 for front, half for back) */
  int num_lines;  /* half_lines for both front & back */
  int buf_idx;    /* Offscreen buffer holding the content */
  int gen;        /* Content generation */
} nyabula_write_job_t;

/* Dual screen context. */
typedef struct
{
  nyabula_screen_t screen[NYABULA_DUAL_LCD_MAX_SCREENS];

  /* Transfer thread: blocking QSPI DMA, one half-block at a time, fairly
   * alternating between the two screens. */
  pthread_t transfer_thread;

  /* Software TE frame clock.  A dedicated thread (standing in for the real
   * TE signal, which is not yet wired in hardware) fires the scan-start
   * (rising) and scan-half (falling) edges at the frame rate by calling
   * the ISR-safe edge handlers -- exactly how a future gpio TE ISR would.
   * It is kept OFF the render thread so the render thread may safely block
   * while waiting for a transfer-competed buffer. */
  pthread_t te_thread;
  bool running;

  /* Write-job queue consumed by the transfer thread (bounded). */
  nyabula_write_job_t job_queue[NYABULA_JOB_QUEUE_SIZE];
  int job_head;
  int job_tail;
  int job_count;
  sem_t job_mutex;     /* Guards the queue (binary mutex) */
  sem_t job_avail;     /* Count of queued jobs */
  sem_t job_space;     /* Count of free queue slots */
  int last_bus_screen; /* Fair dispatch: last screen served (round-robin) */

  /* Render-kick semaphore: wakes the render loop (run by the caller's
   * thread via nyabula_dual_lcd_task()) to consume TE-driven render
   * requests sooner than the default idle sleep.  The render loop and the
   * TE/transfer threads are decoupled from main by this layer; only the
   * lv_timer_handler() call is hosted by the caller. */
  sem_t render_kick;

  bool quitting;
} nyabula_dual_lcd_t;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: nyabula_dual_lcd_create
 *
 * Description:
 *   Create dual LCD display drivers with double buffering support.
 *   Both screens share a single QSPI bus, so DMA transfers are serialized.
 *
 * Input Parameters:
 *   dev_path0 - Path to first LCD device (e.g., "/dev/lcd0")
 *   dev_path1 - Path to second LCD device (e.g., "/dev/lcd1")
 *   width     - Screen width in pixels
 *   height    - Screen height in pixels
 *
 * Returned Value:
 *   Pointer to the created dual LCD context on success, NULL on failure.
 *
 ****************************************************************************/

nyabula_dual_lcd_t *nyabula_dual_lcd_create(const char *dev_path0,
                                            const char *dev_path1, int width,
                                            int height);

/****************************************************************************
 * Name: nyabula_dual_lcd_destroy
 *
 * Description:
 *   Destroy dual LCD display drivers and release resources.
 *
 * Input Parameters:
 *   dual - Pointer to the dual LCD context to destroy.
 *
 ****************************************************************************/

void nyabula_dual_lcd_destroy(nyabula_dual_lcd_t *dual);

/****************************************************************************
 * Name: nyabula_dual_lcd_task
 *
 * Description:
 *   Run one step of the render loop from the caller's own thread.  Each
 *   call advances LVGL (lv_timer_handler()) and consumes any pending
 *   TE-driven render requests.  The caller (e.g. main) should invoke this
 *   in a loop; the display layer starts the TE/transfer threads in
 *   create().  The caller must have finished building the UI before the
 *   first call (LVGL is single-threaded).
 *
 *   Returns after processing one render step (blocking up to the LVGL idle
 *   period when there is nothing to do, or waking early on a render kick).
 *
 * Input Parameters:
 *   dual - Pointer to the dual LCD context.
 *
 ****************************************************************************/

void nyabula_dual_lcd_task(nyabula_dual_lcd_t *dual);

/****************************************************************************
 * Name: nyabula_dual_lcd_get_display
 *
 * Description:
 *   Get LVGL display instance for a specific screen.
 *
 * Input Parameters:
 *   dual      - Pointer to the dual LCD context.
 *   screen_id - Screen index (0 or 1).
 *
 * Returned Value:
 *   Pointer to lv_display_t on success, NULL on failure.
 *
 ****************************************************************************/

lv_display_t *nyabula_dual_lcd_get_display(nyabula_dual_lcd_t *dual,
                                           int screen_id);

#ifdef __cplusplus
}
#endif

#endif /* __NYABULA_DUAL_LCD_H */

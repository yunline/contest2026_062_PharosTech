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

/* BlankGated algorithm layer: framework and scheduler are decoupled; the
 * algorithm issues render/write requests via the registered callbacks. */
#include "nyabula_blankgated_scheduler.h"

/* TE edge source abstraction (software frame clock now, panel TE GPIO
 * interrupt later; selected by CONFIG_NYABULA_DISPLAY_TE_SOURCE). */
#include "nyabula_te.h"

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

/* Default refresh rate in Hz.  The TE source (see nyabula_te.h) derives the
 * software frame clock from this when the SW source is selected. */

#ifndef NYABULA_DUAL_LCD_REFRESH_HZ
#define NYABULA_DUAL_LCD_REFRESH_HZ 60
#endif

/* Transfer thread priority (NuttX: lower number = higher priority).  The
 * transfer (blocking PUTAREA DMA) sits below the TE source but above the
 * caller-hosted render loop so the DMA keeps the bus busy.  The TE priority
 * lives in nyabula_te.h (NYABULA_TE_PRIORITY). */

#ifndef NYABULA_DUAL_LCD_TRANSFER_PRIORITY
#define NYABULA_DUAL_LCD_TRANSFER_PRIORITY 98
#endif

/* Size of the transfer job queue (whole-frame jobs).  The algorithm only
 * issues one write at a time (single-flight shared bus), so this only needs
 * a couple of slots plus margin. */

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

/* Forward declaration: each screen holds a back-pointer to its owning dual
 * context so the algorithm's request callbacks can reach the job queue /
 * threads without a global lookup. */

struct nyabula_dual_lcd_s;

/* Per-screen state and LVGL handle.  The scheduling state lives in the
 * shared nyabula_bg_t in the dual context (see
 * nyabula_blankgated_scheduler.h); this struct holds only the framework-side
 * resources (device, LVGL display, buffers, sync).
 *
 * The tag "nyabula_screen" matches the opaque forward-declaration used by
 * the algorithm header, so the algorithm can reference screens purely by
 * pointer without knowing their layout. */

struct nyabula_screen
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

  struct lcddev_area_align_s align;

  /* Double buffers (full screen each) */
  nyabula_buf_t buf[2];

  /* Generation currently being rendered by LVGL (assigned by the algorithm
   * via request_render; flush_cb binds this generation to the physical
   * buffer and reports on_render_done back). */
  int render_gen;

  /* Posted by the transfer thread whenever a buffer generation becomes
   * fully written (whole-frame DMA done) and is therefore free for LVGL to
   * reuse.  The render thread's flush_wait_cb waits here while a target
   * buffer is still locked. */
  sem_t buf_free;

  /* TE-driven render request.  The TE scan-start edge (via the algorithm)
   * requests a render; the render loop consumes this flag to issue exactly
   * one lv_refr_now() for this screen at the frame boundary.  Guarded by
   * st_mutex. */
  bool render_request;

  /* Serializes access to this screen's scheduler state shared between the
   * TE thread, the transfer thread and the render thread (flush_cb).
   * The edge entry points are intended to be usable from a real TE ISR,
   * so this is a mutex-like binary semaphore safe to post from both thread
   * and (upcoming) interrupt context. */
  sem_t st_mutex;

  /* Screen index (0 or 1), set at create time. */
  int screen_id;

  /* Back-pointer to the owning dual context (set at create time), used by
   * the request callbacks to reach the job queue / threads. */
  struct nyabula_dual_lcd_s *dual;

  bool initialized;
};

typedef struct nyabula_screen nyabula_screen_t;

/* A pending whole-frame write queued for the transfer thread.  The BlankGated
 * scheduler writes the ENTIRE frame (all total_lines rows, from row 0) in one
 * blocking QSPI DMA per generation, gated on the target screen's blanking. */
typedef struct
{
  int screen_id; /* 0 or 1 */
  int buf_idx;   /* Offscreen buffer holding the content */
  int gen;       /* Content generation */
} nyabula_write_job_t;

/* Dual screen context.  The tag is exposed so the audit interface (in
 * nyabula_display_audit.h) can forward-declare it without pulling in the
 * full definition. */
typedef struct nyabula_dual_lcd_s
{
  nyabula_screen_t screen[NYABULA_DUAL_LCD_MAX_SCREENS];

  /* BlankGated scheduling algorithm context (shared by both screens so the
   * single shared-bus write is arbitrated here). */
  nyabula_bg_t bg;

  /* Transfer thread: blocking QSPI DMA, one whole-frame at a time. */
  pthread_t transfer_thread;

  /* TE edge source (software frame clock or panel TE GPIO interrupt, see
   * nyabula_te.h).  It calls the edge callbacks on scan-start/blank-start;
   * the framework only forwards them to the algorithm.  Allocated by
   * nyabula_te_init(), released by nyabula_te_deinit(). */
  nyabula_te_t *te;
  bool running;

  /* Write-job queue consumed by the transfer thread (bounded, strict FIFO). */
  nyabula_write_job_t job_queue[NYABULA_JOB_QUEUE_SIZE];
  int job_head;
  int job_tail;
  int job_count;
  sem_t job_mutex; /* Guards the queue (binary mutex) */
  sem_t job_avail; /* Count of queued jobs */
  sem_t job_space; /* Count of free queue slots */

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

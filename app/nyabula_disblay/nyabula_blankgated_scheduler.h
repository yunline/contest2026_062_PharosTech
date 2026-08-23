/****************************************************************************
 * apps/graphics/nyabula_display/nyabula_blankgated_scheduler.h
 *
 * "BlankGated" scheduler: algorithm layer for the dual-panel shared-write-bus
 * tearing problem, ported from the vsyncalg_plusplus Python reference
 * (schedulers.BlankGatedScheduler / chat.md).
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

#ifndef __NYABULA_BLANKGATED_SCHEDULER_H
#define __NYABULA_BLANKGATED_SCHEDULER_H

/****************************************************************************
 * Design notes
 *
 * The panel controller cannot produce a falling edge exactly at line 180
 * (it only outputs a TE level: HIGH during blanking, LOW during scanning),
 * which is what the previous v8 "BoundedChase" scheduler required.  This
 * scheduler instead gates a whole-frame QSPI write on "entering blanking"
 * and lets the write run into the active scan ("catch-up scan"):
 *
 *   铁律: 写（xfer）只能在目标屏处于垂直消隐期时启动；一旦启动，允许跨入
 *   active 区域继续写——因为写（7ms）比扫描（7.5ms）快，写指针始终领先
 *   扫描指针，因此不会撕裂。
 *
 * So the two physically-observable edges are:
 *   - TE rising edge (LOW -> HIGH) = scanning finished, blanking begins.
 *     This is the ONLY gate at which a (whole-frame) QSPI write may start.
 *   - TE falling edge (HIGH -> LOW) = blanking ended, a new frame scan
 *     begins.  A new render is requested here.
 *
 * State (per screen, plus a shared-bus single-flight flag):
 *   rendering_gen : generation currently being rendered by LVGL.
 *   ready_gen     : rendered generation awaiting its whole-frame write.
 *   xfer_gen      : generation whose whole-frame write is in flight.
 *   in_blanking   : screen is inside the vertical blanking window.
 *   want          : a new frame is wanted (set on scan start, consumed by
 *                   the next successful render request).
 *
 * Invariants:
 *   1) render gating  : a new render is only requested while at most one
 *                       buffer is busy (pending <= 1, so pending + rendering
 *                       <= 2 legal double buffer).  Otherwise the render is
 *                       deferred (algorithm voluntarily skips a frame) and
 *                       retried on a later edge once a write releases a
 *                       buffer.
 *   2) write gating   : a whole-frame write starts only when the target
 *                       screen is in_blanking AND the shared bus is free
 *                       (single-flight: at most one write in flight across
 *                       both screens).
 *   3) buffer safety  : the framework's flush_wait_cb is the hard net that
 *                       keeps LVGL from drawing into a buffer whose write is
 *                       not finished; the render gate keeps it from being
 *                       needed in the healthy case.
 *   4) generation is complete exactly when its single whole-frame write
 *       finishes (no block-ledger needed, unlike v8's front/back halves).
 *
 * All state lives in this struct (one instance per dual-LCD context, shared
 * by both screens so the single-bus arbitration is done here).
 ****************************************************************************/

#include <semaphore.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define NYABULA_BG_MAX_SCREENS 2

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Opaque handle: the framework's "screen".  The algorithm only uses it to
 * distinguish screens in callbacks and never reads its internals.  The real
 * definition lives in nyabula_dual_lcd.h (struct nyabula_screen). */

struct nyabula_screen;

/* Algorithm callback table: the framework registers these request functions
 * at create time; the algorithm uses them to issue render/write requests. */

typedef struct
{
  /* Request a full-frame render of a new generation gen for a screen.  The
   * framework implements this by driving LVGL to render the screen (on the
   * render-thread context). */
  void (*request_render)(struct nyabula_screen *screen, int gen);

  /* Submit the WHOLE frame (all total_lines rows) of generation gen to the
   * transfer thread (enqueued, executed as one blocking QSPI DMA).  Returns
   * bool: true = enqueued (an on_xfer_done will follow); false = enqueue
   * failed and the framework dropped the job.  The algorithm rolls back on
   * false, avoiding a phantom in-flight that would stall the pipeline. */
  bool (*request_write)(struct nyabula_screen *screen, int gen);

  /* The whole-frame write of generation gen has completed; the generation's
   * physical offscreen buffer is returned to the CPU and the framework may
   * mark it free for LVGL reuse. */
  void (*on_gen_complete)(struct nyabula_screen *screen, int gen);
} nyabula_bg_callbacks_t;

/* BlankGated algorithm context (one per dual-LCD context, shared by both
 * screens so the shared-bus single-flight is coordinated here). */

typedef struct nyabula_bg_s
{
  /* Generation counter (algorithm-maintained, not framework-owned). */
  int gen;

  /* Per-screen ledger.  A generation lives in exactly one of the three
   * states below; -1 = no generation in that state. */
  int rendering_gen[NYABULA_BG_MAX_SCREENS];
  int ready_gen[NYABULA_BG_MAX_SCREENS];
  int xfer_gen[NYABULA_BG_MAX_SCREENS];

  /* Per-screen: inside the blanking window / wants a new frame. */
  bool in_blanking[NYABULA_BG_MAX_SCREENS];
  bool want[NYABULA_BG_MAX_SCREENS];

  /* Shared-bus single-flight: a whole-frame write is currently in flight
   * (across both screens). */
  bool xfer_busy;

  /* Serializes the algorithm state between the TE thread, the render
   * thread (flush_cb) and the transfer thread.  A future real TE GPIO ISR
   * must NOT call the entry points directly from interrupt context; it
   * should defer to this thread (post a semaphore / work queue), the
   * standard NuttX pattern. */
  sem_t lock;

  /* Screen back-pointers (set by the framework at create time). */
  struct nyabula_screen *screen[NYABULA_BG_MAX_SCREENS];

  /* Framework-registered request callbacks. */
  nyabula_bg_callbacks_t cb;
} nyabula_bg_t;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Initialize the algorithm context (bind the request callbacks).  The
 * screen[] back-pointers must be set by the framework afterwards. */
void nyabula_bg_init(nyabula_bg_t *bg, const nyabula_bg_callbacks_t *cb);

/* framework -> algorithm: TE falling edge (blanking ended, a new frame scan
 * begins): clear in_blanking, request a new frame. */
void nyabula_bg_on_scan_start(nyabula_bg_t *bg, int sid);

/* framework -> algorithm: TE rising edge (scanning finished, blanking
 * begins): set in_blanking and try to start a whole-frame write. */
void nyabula_bg_on_blank_start(nyabula_bg_t *bg, int sid);

/* framework -> algorithm: one render completed, content gen is ready. */
void nyabula_bg_on_render_done(nyabula_bg_t *bg, int sid, int gen);

/* framework -> algorithm: the whole-frame write of gen completed. */
void nyabula_bg_on_xfer_done(nyabula_bg_t *bg, int sid, int gen);

#ifdef __cplusplus
}
#endif

#endif /* __NYABULA_BLANKGATED_SCHEDULER_H */

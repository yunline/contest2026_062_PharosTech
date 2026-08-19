/****************************************************************************
 * apps/graphics/nyabula_display/nyabula_v8_scheduler.h
 *
 * v8 "BoundedChase" chase scheduler: algorithm layer, fully decoupled from
 * the framework (threads/LVGL/DMA).
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

#ifndef __NYABULA_V8_SCHEDULER_H
#define __NYABULA_V8_SCHEDULER_H

/****************************************************************************
 * Design notes
 *
 * v8 is a "chase" scheduler that explicitly maintains the double-buffer
 * state on the ALGORITHM side. It only consumes the 6 interfaces the
 * framework exposes and knows nothing of LVGL/DMA/threads:
 *
 *   framework -> algorithm (edge callbacks, invoked by the framework when the
 *   real event occurs):
 *     on_scan_start(screen)          TE rising edge: a full frame scanned,
 *                                    a new frame scan begins
 *     on_scan_half (screen)          TE falling edge: front half (0..half-1)
 *                                    scanned
 *     on_render_done(screen, gen)    one render for the screen completed;
 *                                    content gen is ready
 *     on_block_write_done(screen,gen) one half-screen block DMA write done
 *
 *   algorithm -> framework (requests issued by the algorithm, implemented by
 *   the framework-registered callbacks):
 *     request_render(screen, gen)    request rendering a new frame gen
 *     request_write(screen, start, n, gen)  request writing n lines (half-
 *                                    screen block) of gen starting at start
 *
 * Key mechanisms (the difference vs v6's "depend on the simulator fallback"):
 *   1) Self-maintained generation counter _gen (not handed out by
 *      framework/simulator).
 *   2) Self-maintained double-buffer ledger:
 *        _rendering_gen : generation being rendered (None = none)
 *        _pending_gens  : set of "rendered but not yet transferred"
 *                         generations (one buffer each)
 *      Render gating _can_render: allow only while pending <= 1 (pending 1 +
 *      rendering 1 = 2, legal). Write gating: only write already-rendered
 *      generations (those in _pending_gens).
 *   3) Backpressure (anti-freeze): submit the next half-screen block only
 *      while in-flight block count < PENDING_BUDGET (=1), driven precisely
 *      by the on_block_write_done edge (never by per-line progress or a
 *      global in-flight count).
 *   4) Generation-complete detection (self-maintained): a generation is fully
 *      transferred only once "2 blocks (front/back half) submitted AND both
 *      blocks received their done callback", at which point it is removed
 *      from _pending_gens (releasing the buffer).
 *
 * All state lives in the algorithm's own (nyabula_v8_t) struct and never
 * intrudes into the framework's screen struct.
 ****************************************************************************/

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Maximum in-flight blocks per screen = 1 (single-flight half screen). Once
 * reached, defer the next block so the bus drains => no freeze under
 * bandwidth shortage. Driven precisely by on_block_write_done (block-level
 * interrupt). */

#ifndef NYABULA_V8_PENDING_BUDGET
#define NYABULA_V8_PENDING_BUDGET 1
#endif

/* Fixed number of blocks per generation (front + back half, one each). */

#define NYABULA_V8_BLOCKS_PER_GEN 2

/* Upper bound on simultaneously pending generations per screen (double
 * buffer: pending + rendering, at most 2). Used for the fixed-size ledger
 * tables and defensive checks, avoiding dynamic allocation on the RTOS. */

#define NYABULA_V8_MAX_PENDING_GENS 2

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Opaque handle: the framework's "screen". The algorithm only uses it to
 * distinguish screens in callbacks and never reads its internals. The real
 * definition lives in nyabula_dual_lcd.h (struct nyabula_screen). */

struct nyabula_screen;

/* Algorithm callback table: the framework registers these request functions
 * at create time; the algorithm uses them to issue render/write requests. */

typedef struct
{
  /* Request a full-frame render of a new generation gen for a screen. The
   * framework implements this by driving LVGL to render the screen (on the
   * render-thread context). */
  void (*request_render)(struct nyabula_screen *screen, int gen);

  /* Submit n lines (a half-screen block) of gen, starting at start_line, to
   * the transfer thread (enqueued, executed per line by DMA). Line numbers
   * wrap. Returns bool: true = enqueued (a block-done will follow); false =
   * enqueue failed (queue full / resource short) and the framework dropped
   * the block. The algorithm rolls back its in-flight/gen counters on false,
   * avoiding a "phantom in-flight" that would permanently stall backpressure.
   */
  bool (*request_write)(struct nyabula_screen *screen, int start_line,
                        int num_lines, int gen);

  /* The two blocks (front + back half) of generation gen have both been
   * transferred; the generation's physical offscreen buffer is returned to
   * the CPU and the framework may mark it free for LVGL reuse. Called by the
   * algorithm when it detects generation completion. */
  void (*on_gen_complete)(struct nyabula_screen *screen, int gen);
} nyabula_v8_callbacks_t;

/* v8 algorithm context (one per screen). Held by the framework and passed
 * through to the algorithm entry points. */

typedef struct nyabula_v8_s
{
  /* Generation counter (algorithm-maintained, not framework-owned). */
  int gen;

  /* Double-buffer ledger: generation being rendered (-1 = none). */
  int rendering_gen;

  /* Pending generation set: fixed-size slots holding "rendered, awaiting
   * transfer" generations. Capacity NYABULA_V8_MAX_PENDING_GENS (=2). */
  int pending_gens[NYABULA_V8_MAX_PENDING_GENS];
  int pending_count;

  /* Most recently rendered generation (locked at the falling edge; "locking"
   * moves it into half_gen). */
  int render_done;

  /* Generation locked at the falling edge: front half written (or pending),
   * the rising edge writes the same generation's back half. */
  int half_gen;

  /* In-flight block count (submit +1, block-write-done -1), for
   * backpressure. */
  int inflight_blocks;

  /* Generation-block ledger (self-maintained, to detect when a generation is
   * fully transferred):
   *   gen_submitted[g]   : submitted block count (cumulative; 2 per gen)
   *   gen_outstanding[g] : submitted blocks not yet done-completed */
  int gen_submitted[NYABULA_V8_MAX_PENDING_GENS];
  int gen_outstanding[NYABULA_V8_MAX_PENDING_GENS];

  /* Tracked generation numbers (aligned with the two slots above; -1 =
   * empty). */
  int tracked_gen[NYABULA_V8_MAX_PENDING_GENS];

  /* Framework inputs: per-screen half-screen line count (half_line) and total
   * line count. Written by the framework at create time. */
  int half_lines;
  int total_lines;

  /* Back-pointer to the owning screen (written by the framework at create
   * time, used for the on_gen_complete callback). */
  struct nyabula_screen *screen;

  /* Framework-registered request callbacks. */
  nyabula_v8_callbacks_t cb;
} nyabula_v8_t;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Initialize the algorithm context (bind half_lines/total_lines and the
 * request callbacks). */
void nyabula_v8_init(nyabula_v8_t *v8, int half_lines, int total_lines,
                     const nyabula_v8_callbacks_t *cb);

/* framework -> algorithm: TE rising edge (a full frame scanned, new frame
 * scan begins). */
void nyabula_v8_on_scan_start(nyabula_v8_t *v8, struct nyabula_screen *screen);

/* framework -> algorithm: TE falling edge (front half 0..half-1 scanned). */
void nyabula_v8_on_scan_half(nyabula_v8_t *v8, struct nyabula_screen *screen);

/* framework -> algorithm: one render completed, content gen is ready. */
void nyabula_v8_on_render_done(nyabula_v8_t *v8, int gen);

/* framework -> algorithm: one half-screen block DMA write completed, its
 * content gen is now in GRAM. */
void nyabula_v8_on_block_write_done(nyabula_v8_t *v8, int gen);

#ifdef __cplusplus
}
#endif

#endif /* __NYABULA_V8_SCHEDULER_H */

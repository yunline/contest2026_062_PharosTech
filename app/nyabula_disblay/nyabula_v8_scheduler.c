/****************************************************************************
 * apps/graphics/nyabula_display/nyabula_v8_scheduler.c
 *
 * v8 "BoundedChase" chase scheduler: algorithm-layer implementation.
 *
 * This is the C port of the final iteration of an in-house scheduling
 * algorithm developed (via a simulator) for the dual-panel shared-write-bus
 * tearing problem.  Its defining property versus earlier iterations is that
 * it maintains the double-buffer ledger on the algorithm side, without
 * relying on any simulator/host fallback, and drives backpressure off the
 * physical block-transfer-done edge rather than any per-line progress query.
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

#include "nyabula_v8_scheduler.h"

#include <stdbool.h>
#include <string.h>

/****************************************************************************
 * Algorithm overview
 *
 * 1) Per-screen ledger:
 *      rendering_gen : generation currently being rendered (set on start,
 *                      cleared by on_render_done).
 *      pending_gens  : set of "rendered but not yet fully written"
 *                      generations (added on render_done, removed once the
 *                      generation is fully transferred per block_write_done).
 * 2) Render gating (_can_render): a new render requires one working buffer;
 *    after completion it becomes "pending" (still occupying one buffer). So
 *    it is allowed only when "pending count <= 1" (pending 1 + rendering 1 =
 *    2, legal). If both pending slots are already full, reject (the algorithm
 *    voluntarily drops a frame) and recover naturally on a later scan edge
 *    once a block_write_done releases a buffer.
 * 3) Write gating: only write "already rendered" generations (those in
 *    pending_gens); never write a generation still being rendered.
 * 4) Backpressure: submit a new half-screen block only while in-flight block
 *    count < PENDING_BUDGET (=1, single-flight half screen), driven by the
 *    block-write-done edge (never by per-line DMA progress).
 * 5) Safe window (same as v6): write the front half on the falling edge and
 *    the back half on the rising edge, both from the same generation.
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static bool pending_contains(const nyabula_v8_t *v8, int gen)
{
  int i;

  for (i = 0; i < v8->pending_count; i++)
    {
      if (v8->pending_gens[i] == gen)
        {
          return true;
        }
    }

  return false;
}

static void pending_add(nyabula_v8_t *v8, int gen)
{
  if (pending_contains(v8, gen))
    {
      return;
    }

  if (v8->pending_count < NYABULA_V8_MAX_PENDING_GENS)
    {
      v8->pending_gens[v8->pending_count++] = gen;
    }
}

static void pending_remove(nyabula_v8_t *v8, int gen)
{
  int i;

  for (i = 0; i < v8->pending_count; i++)
    {
      if (v8->pending_gens[i] == gen)
        {
          /* Swap with the last element and shrink (order is irrelevant). */
          v8->pending_gens[i] = v8->pending_gens[v8->pending_count - 1];
          v8->pending_count--;
          return;
        }
    }
}

/* In-flight block count for this screen (submitted but not yet fully written
 * via DMA). */

static int inflight(const nyabula_v8_t *v8) { return v8->inflight_blocks; }

/* Number of "rendered but not yet fully written" generations (each occupies
 * one offscreen buffer). */

static int pending_count(const nyabula_v8_t *v8) { return v8->pending_count; }

/* Render gating: a new render needs one working buffer; after completion the
 * generation becomes "pending" (still one buffer). Allowed only while
 * pending <= 1 (pending 1 + rendering 1 = 2, legal); pending == 2 would need
 * a third buffer, so reject. */

static bool can_render(const nyabula_v8_t *v8)
{
  return pending_count(v8) <= 1;
}

/* Backpressure: whether another half-screen block may be submitted (prevents
 * freezing when the bus lacks bandwidth). */

static bool can_submit(const nyabula_v8_t *v8)
{
  return inflight(v8) < NYABULA_V8_PENDING_BUDGET;
}

/* Find the generation-block ledger slot for gen, or -1 if absent. */

static int gen_slot(const nyabula_v8_t *v8, int gen)
{
  int i;

  for (i = 0; i < NYABULA_V8_MAX_PENDING_GENS; i++)
    {
      if (v8->tracked_gen[i] == gen)
        {
          return i;
        }
    }

  return -1;
}

/* Ensure gen has a ledger slot (return the slot, or -1 if out of slots; the
 * caller guarantees capacity = max pending generations). */

static int gen_slot_ensure(nyabula_v8_t *v8, int gen)
{
  int i = gen_slot(v8, gen);

  if (i >= 0)
    {
      return i;
    }

  for (i = 0; i < NYABULA_V8_MAX_PENDING_GENS; i++)
    {
      if (v8->tracked_gen[i] == -1)
        {
          v8->tracked_gen[i] = gen;
          v8->gen_submitted[i] = 0;
          v8->gen_outstanding[i] = 0;
          return i;
        }
    }

  return -1;
}

/* Submit one whole block (never split). Only write "rendered" generations,
 * never a generation still rendering.
 *
 * The in-flight block / per-generation counters are incremented only after
 * request_write actually enqueues the block (returns true). On enqueue
 * failure (queue full / resource short) we do NOT increment them, producing
 * no "phantom in-flight" entry -- otherwise inflight_blocks / gen_outstanding
 * would never receive a matching block-done callback and the backpressure /
 * generation-completion accounting would stall forever.
 */

static void write_block(nyabula_v8_t *v8, struct nyabula_screen *screen,
                        int start_line, int num_lines, int gen)
{
  int slot;
  bool accepted;

  if (num_lines <= 0)
    {
      return;
    }

  if (!pending_contains(v8, gen))
    {
      /* Write gating: gen is not yet rendered (or already transferred). */
      return;
    }

  if (v8->cb.request_write == NULL)
    {
      return;
    }

  slot = gen_slot_ensure(v8, gen);
  if (slot < 0)
    {
      return; /* Ledger slots exhausted (should not happen). */
    }

  /* Submit first; only account on success. */
  accepted = v8->cb.request_write(screen, start_line, num_lines, gen);
  if (!accepted)
    {
      return; /* Enqueue failed: do not ++ inflight/outstanding. */
    }

  v8->inflight_blocks++;
  v8->gen_submitted[slot]++;
  v8->gen_outstanding[slot]++;
}

/* Try to start rendering a new frame for the screen (gated by double-buffer
 * capacity). Returns true if a render was actually requested. */

static bool try_start_render(nyabula_v8_t *v8, struct nyabula_screen *screen)
{
  if (v8->rendering_gen != -1)
    {
      return false; /* This screen is already rendering (safety net). */
    }

  if (!can_render(v8))
    {
      /* Double-buffer gate: pending is full -> defer (algorithm drops a
       * frame); recover on a later scan edge once a block_write_done
       * releases a buffer. */
      return false;
    }

  v8->gen++;
  v8->rendering_gen = v8->gen;

  if (v8->cb.request_render != NULL)
    {
      v8->cb.request_render(screen, v8->gen);
    }

  return true;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void nyabula_v8_init(nyabula_v8_t *v8, int half_lines, int total_lines,
                     const nyabula_v8_callbacks_t *cb)
{
  int i;

  memset(v8, 0, sizeof(*v8));

  v8->gen = 0;
  v8->rendering_gen = -1;
  v8->render_done = -1;
  v8->half_gen = -1;
  v8->half_lines = half_lines;
  v8->total_lines = total_lines;

  for (i = 0; i < NYABULA_V8_MAX_PENDING_GENS; i++)
    {
      v8->tracked_gen[i] = -1;
    }

  if (cb != NULL)
    {
      v8->cb = *cb;
    }
}

/* ------------------------------------------------------------------
 * Falling edge: rows 0..half-1 scanned -> lock the latest rendered
 * generation, submit the front half as one block, and kick the next render.
 * ------------------------------------------------------------------ */

void nyabula_v8_on_scan_half(nyabula_v8_t *v8, struct nyabula_screen *screen)
{
  int half = v8->half_lines;
  int g;

  /* Lock the latest rendered generation (consume render_done once). */
  if (v8->render_done != -1)
    {
      v8->half_gen = v8->render_done;
      v8->render_done = -1;
    }

  g = v8->half_gen;

  /* Start rendering the next frame (parallel, off the bus). The double-buffer
   * gate is enforced inside. */
  try_start_render(v8, screen);

  if (g == -1)
    {
      return; /* No rendered content yet -> keep showing the old frame. */
    }

  if (!can_submit(v8))
    {
      return; /* Backpressure. */
    }

  write_block(v8, screen, 0, half, g);
}

/* ------------------------------------------------------------------
 * Rising edge: the previous frame's full 360 rows are scanned -> write the
 * back half (same generation as the front half).
 * ------------------------------------------------------------------ */

void nyabula_v8_on_scan_start(nyabula_v8_t *v8, struct nyabula_screen *screen)
{
  int half = v8->half_lines;
  int gen = v8->half_gen;

  if (gen == -1)
    {
      return;
    }

  if (!can_submit(v8))
    {
      return; /* Backpressure. */
    }

  write_block(v8, screen, half, v8->total_lines - half, gen);
}

/* ------------------------------------------------------------------
 * Render done: content is ready; the generation moves to "pending" (working
 * buffer -> pending buffer; buffer occupancy is unchanged).
 * ------------------------------------------------------------------ */

void nyabula_v8_on_render_done(nyabula_v8_t *v8, int gen)
{
  if (v8->rendering_gen == gen)
    {
      v8->rendering_gen = -1;
    }

  pending_add(v8, gen);
  v8->render_done = gen;
}

/* ------------------------------------------------------------------
 * Block write done: in-flight count -1; the algorithm self-determines
 * whether the generation is now fully transferred (releasing a double
 * buffer).
 * ------------------------------------------------------------------ */

void nyabula_v8_on_block_write_done(nyabula_v8_t *v8, int gen)
{
  int slot;

  if (v8->inflight_blocks > 0)
    {
      v8->inflight_blocks--;
    }

  slot = gen_slot(v8, gen);
  if (slot < 0)
    {
      return;
    }

  if (v8->gen_outstanding[slot] > 0)
    {
      v8->gen_outstanding[slot]--;
    }

  /* Generation-complete test: the generation has submitted a full 2 blocks
   * (front + back half) AND its outstanding count reached 0. We cannot use a
   * bare "outstanding reached 0" test -- when the first (front) half
   * completes the outstanding count is already 0, but the back half has not
   * yet been submitted (it waits for the rising edge), which would be
   * mis-detected as generation completion. */
  if (v8->gen_submitted[slot] >= NYABULA_V8_BLOCKS_PER_GEN &&
      v8->gen_outstanding[slot] == 0)
    {
      /* Release the ledger slot and drop from the pending set (both buffers
       * are now back to the CPU). */
      v8->tracked_gen[slot] = -1;
      v8->gen_submitted[slot] = 0;
      v8->gen_outstanding[slot] = 0;
      pending_remove(v8, gen);

      /* Notify the framework: the generation's physical buffer is free to be
       * reused by LVGL. */
      if (v8->cb.on_gen_complete != NULL)
        {
          v8->cb.on_gen_complete(v8->screen, gen);
        }
    }
}

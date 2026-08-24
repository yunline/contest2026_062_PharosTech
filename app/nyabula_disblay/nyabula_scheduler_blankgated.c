/****************************************************************************
 * apps/graphics/nyabula_display/nyabula_scheduler_blankgated.c
 *
 * "BlankGated" scheduler: C port of the vsyncalg_plusplus
 * BlankGatedScheduler (see nyabula_scheduler_blankgated.h for the design
 * notes).  This is the algorithm layer: it only consumes the four edge
 * callbacks and issues render/write requests through the registered
 * callbacks, and knows nothing about LVGL, DMA or threads.
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

#include "nyabula_scheduler_blankgated.h"

#include <string.h>

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Number of "rendered but not yet fully written" slots for one screen
 * (each occupies one offscreen buffer). */

static int sch_bg_pending_count(const nyabula_sch_bg_t *sch_bg, int sid)
{
  int n = 0;

  if (sch_bg->ready_slot[sid] != -1)
    {
      n++;
    }

  if (sch_bg->xfer_slot[sid] != -1)
    {
      n++;
    }

  return n;
}

/* Render gating: a new render needs one working slot; after completion the
 * slot becomes "ready" (still one slot).  Allowed only while pending <= 1
 * (pending 1 + rendering 1 = 2, legal); pending == 2 would need a third
 * slot, so reject and defer (voluntarily drop a frame). */

static bool sch_bg_can_render(const nyabula_sch_bg_t *sch_bg, int sid)
{
  return sch_bg_pending_count(sch_bg, sid) <= 1;
}

/* Pick a free offscreen slot for screen sid.  A slot is free iff it is NOT
 * the rendering slot AND NOT the ready slot AND NOT the xfer slot (strict
 * three-state check, mirroring the reference BlankGatedScheduler's
 * _free_slot with busy = {render, ready, xfer}).  Prefers next_slot, then
 * the other; returns -1 if both are busy. */

static int sch_bg_free_slot(const nyabula_sch_bg_t *sch_bg, int sid)
{
  int prefer = sch_bg->next_slot[sid];
  int slot;

  /* Try preferred slot first, then the alternate. */
  for (slot = prefer;; slot = 1 - slot)
    {
      if (slot != sch_bg->rendering_slot[sid] &&
          slot != sch_bg->ready_slot[sid] && slot != sch_bg->xfer_slot[sid])
        {
          return slot;
        }

      if (slot == 1 - prefer)
        {
          break;
        }
    }

  return -1;
}

/* Try to start rendering a new frame for screen sid (gated by double-buffer
 * capacity and the per-screen "want" flag).  Caller holds the algorithm
 * lock. */

static void sch_bg_try_start_render(nyabula_sch_bg_t *sch_bg, int sid)
{
  int slot;

  if (!sch_bg->want[sid])
    {
      return;
    }

  if (sch_bg->rendering_slot[sid] != -1)
    {
      return; /* This screen is already rendering. */
    }

  if (!sch_bg_can_render(sch_bg, sid))
    {
      /* Double-buffer gate: pending is full -> defer (algorithm drops a
       * frame); recover on a later edge once a write releases a buffer. */
      return;
    }

  /* Algorithm owns slot selection (mirrors _free_slot in the reference). */
  slot = sch_bg_free_slot(sch_bg, sid);
  if (slot < 0)
    {
      /* Should be unreachable given sch_bg_can_render above, but stay safe. */
      return;
    }

  sch_bg->rendering_slot[sid] = slot;
  sch_bg->next_slot[sid] = 1 - slot;
  sch_bg->want[sid] = false;

  if (sch_bg->cb.request_render != NULL)
    {
      sch_bg->cb.request_render(sch_bg->screen[sid], slot);
    }
}

/* Try to start a whole-frame QSPI write for any screen that has a rendered
 * frame ready AND is inside its blanking window, while the shared bus is
 * free (single-flight).  Caller holds the algorithm lock. */

static void sch_bg_try_start_xfer(nyabula_sch_bg_t *sch_bg)
{
  int s;

  if (sch_bg->xfer_busy)
    {
      return;
    }

  /* A priority, then B (matches chat.md: if A.pending && A.in_blanking ->
   * write A, else if B.pending && B.in_blanking -> write B). */
  for (s = 0; s < NYABULA_SCH_BG_MAX_SCREENS; s++)
    {
      int slot;
      bool ok;

      if (sch_bg->ready_slot[s] == -1 || !sch_bg->in_blanking[s])
        {
          continue;
        }

      slot = sch_bg->ready_slot[s];
      sch_bg->xfer_busy = true;
      sch_bg->ready_slot[s] = -1;
      sch_bg->xfer_slot[s] = slot;

      ok = (sch_bg->cb.request_write != NULL) &&
           sch_bg->cb.request_write(sch_bg->screen[s], slot);
      if (!ok)
        {
          /* Enqueue failed: roll back so a "phantom in-flight" never stalls
           * the single-flight bus / slot accounting. */
          sch_bg->xfer_busy = false;
          sch_bg->xfer_slot[s] = -1;
          sch_bg->ready_slot[s] = slot;
        }

      return;
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void nyabula_sch_bg_init(nyabula_sch_bg_t *sch_bg,
                         const nyabula_sch_bg_callbacks_t *cb)
{
  int i;

  memset(sch_bg, 0, sizeof(*sch_bg));

  for (i = 0; i < NYABULA_SCH_BG_MAX_SCREENS; i++)
    {
      sch_bg->rendering_slot[i] = -1;
      sch_bg->ready_slot[i] = -1;
      sch_bg->xfer_slot[i] = -1;
      sch_bg->next_slot[i] = 0;
      sch_bg->in_blanking[i] = false;
      sch_bg->want[i] = false;
    }

  sch_bg->xfer_busy = false;
  sem_init(&sch_bg->lock, 0, 1);

  if (cb != NULL)
    {
      sch_bg->cb = *cb;
    }
}

/* ------------------------------------------------------------------
 * TE falling edge (blanking ended, scanning begins): leave the blanking
 * window and ask for a new frame.
 * ------------------------------------------------------------------ */

void nyabula_sch_bg_on_scan_start(nyabula_sch_bg_t *sch_bg, int sid)
{
  sem_wait(&sch_bg->lock);

  sch_bg->in_blanking[sid] = false;
  sch_bg->want[sid] = true;
  sch_bg_try_start_render(sch_bg, sid);

  sem_post(&sch_bg->lock);
}

/* ------------------------------------------------------------------
 * TE rising edge (scanning finished, blanking begins): the only gate at
 * which a (whole-frame) write may start.
 * ------------------------------------------------------------------ */

void nyabula_sch_bg_on_blank_start(nyabula_sch_bg_t *sch_bg, int sid)
{
  sem_wait(&sch_bg->lock);

  sch_bg->in_blanking[sid] = true;
  sch_bg_try_start_xfer(sch_bg);

  sem_post(&sch_bg->lock);
}

/* ------------------------------------------------------------------
 * Render done: content is ready; the slot moves to "ready" (working
 * slot -> ready slot; buffer occupancy is unchanged).  Kick off a write
 * (if in blanking) and the next render.
 * ------------------------------------------------------------------ */

void nyabula_sch_bg_on_render_done(nyabula_sch_bg_t *sch_bg, int sid, int slot)
{
  sem_wait(&sch_bg->lock);

  if (sch_bg->rendering_slot[sid] == slot)
    {
      sch_bg->rendering_slot[sid] = -1;
    }

  sch_bg->ready_slot[sid] = slot;

  sch_bg_try_start_xfer(sch_bg);
  sch_bg_try_start_render(sch_bg, sid);

  sem_post(&sch_bg->lock);
}

/* ------------------------------------------------------------------
 * Render skipped: the algorithm requested a render of `slot`, but LVGL had
 * no invalid area and skipped the draw entirely (static frame).  The slot is
 * NOT marked ready -- the panel GRAM already holds the latest content, so
 * no write is needed.  Only clearing rendering_slot keeps the pipeline
 * rotating without a needless GRAM write.
 * ------------------------------------------------------------------ */

void nyabula_sch_bg_on_render_skip(nyabula_sch_bg_t *sch_bg, int sid, int slot)
{
  sem_wait(&sch_bg->lock);

  if (sch_bg->rendering_slot[sid] == slot)
    {
      sch_bg->rendering_slot[sid] = -1;
    }

  /* Deliberately do NOT set ready_slot: nothing new was rendered, so there
   * is nothing to write out.  GRAM already shows the latest frame. */

  sch_bg_try_start_render(sch_bg, sid);

  sem_post(&sch_bg->lock);
}

/* ------------------------------------------------------------------
 * Whole-frame write done: release the single-flight bus and (because a
 * slot is complete exactly when its single whole-frame write ends) tell the
 * framework the buffer is free; then try the next write / render.
 * ------------------------------------------------------------------ */

void nyabula_sch_bg_on_xfer_done(nyabula_sch_bg_t *sch_bg, int sid, int slot)
{
  sem_wait(&sch_bg->lock);

  if (sch_bg->xfer_slot[sid] == slot)
    {
      sch_bg->xfer_slot[sid] = -1;
    }

  sch_bg->xfer_busy = false;

  /* The whole-frame write of slot is complete: the physical buffer is free
   * for LVGL to reuse. */
  if (sch_bg->cb.on_buf_free != NULL)
    {
      sch_bg->cb.on_buf_free(sch_bg->screen[sid], slot);
    }

  sch_bg_try_start_xfer(sch_bg);
  sch_bg_try_start_render(sch_bg, sid);

  sem_post(&sch_bg->lock);
}

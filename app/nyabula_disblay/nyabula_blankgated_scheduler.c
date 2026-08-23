/****************************************************************************
 * apps/graphics/nyabula_display/nyabula_blankgated_scheduler.c
 *
 * "BlankGated" scheduler: C port of the vsyncalg_plusplus
 * BlankGatedScheduler (see nyabula_blankgated_scheduler.h for the design
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

#include "nyabula_blankgated_scheduler.h"

#include <string.h>

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Number of "rendered but not yet fully written" generations for one screen
 * (each occupies one offscreen buffer). */

static int bg_pending_count(const nyabula_bg_t *bg, int sid)
{
  int n = 0;

  if (bg->ready_gen[sid] != -1)
    {
      n++;
    }

  if (bg->xfer_gen[sid] != -1)
    {
      n++;
    }

  return n;
}

/* Render gating: a new render needs one working buffer; after completion the
 * generation becomes "ready" (still one buffer).  Allowed only while
 * pending <= 1 (pending 1 + rendering 1 = 2, legal); pending == 2 would
 * need a third buffer, so reject and defer (voluntarily drop a frame). */

static bool bg_can_render(const nyabula_bg_t *bg, int sid)
{
  return bg_pending_count(bg, sid) <= 1;
}

/* Try to start rendering a new frame for screen sid (gated by double-buffer
 * capacity and the per-screen "want" flag).  Caller holds the algorithm
 * lock. */

static void bg_try_start_render(nyabula_bg_t *bg, int sid)
{
  if (!bg->want[sid])
    {
      return;
    }

  if (bg->rendering_gen[sid] != -1)
    {
      return; /* This screen is already rendering. */
    }

  if (!bg_can_render(bg, sid))
    {
      /* Double-buffer gate: pending is full -> defer (algorithm drops a
       * frame); recover on a later edge once a write releases a buffer. */
      return;
    }

  bg->gen++;
  bg->rendering_gen[sid] = bg->gen;
  bg->want[sid] = false;

  if (bg->cb.request_render != NULL)
    {
      bg->cb.request_render(bg->screen[sid], bg->gen);
    }
}

/* Try to start a whole-frame QSPI write for any screen that has a rendered
 * frame ready AND is inside its blanking window, while the shared bus is
 * free (single-flight).  Caller holds the algorithm lock. */

static void bg_try_start_xfer(nyabula_bg_t *bg)
{
  int s;

  if (bg->xfer_busy)
    {
      return;
    }

  /* A priority, then B (matches chat.md: if A.pending && A.in_blanking ->
   * write A, else if B.pending && B.in_blanking -> write B). */
  for (s = 0; s < NYABULA_BG_MAX_SCREENS; s++)
    {
      int gen;
      bool ok;

      if (bg->ready_gen[s] == -1 || !bg->in_blanking[s])
        {
          continue;
        }

      gen = bg->ready_gen[s];
      bg->xfer_busy = true;
      bg->ready_gen[s] = -1;
      bg->xfer_gen[s] = gen;

      ok = (bg->cb.request_write != NULL) &&
           bg->cb.request_write(bg->screen[s], gen);
      if (!ok)
        {
          /* Enqueue failed: roll back so a "phantom in-flight" never stalls
           * the single-flight bus / generation accounting. */
          bg->xfer_busy = false;
          bg->xfer_gen[s] = -1;
          bg->ready_gen[s] = gen;
        }

      return;
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void nyabula_bg_init(nyabula_bg_t *bg, const nyabula_bg_callbacks_t *cb)
{
  int i;

  memset(bg, 0, sizeof(*bg));

  for (i = 0; i < NYABULA_BG_MAX_SCREENS; i++)
    {
      bg->rendering_gen[i] = -1;
      bg->ready_gen[i] = -1;
      bg->xfer_gen[i] = -1;
      bg->in_blanking[i] = false;
      bg->want[i] = false;
    }

  bg->gen = 0;
  bg->xfer_busy = false;
  sem_init(&bg->lock, 0, 1);

  if (cb != NULL)
    {
      bg->cb = *cb;
    }
}

/* ------------------------------------------------------------------
 * TE falling edge (blanking ended, scanning begins): leave the blanking
 * window and ask for a new frame.
 * ------------------------------------------------------------------ */

void nyabula_bg_on_scan_start(nyabula_bg_t *bg, int sid)
{
  sem_wait(&bg->lock);

  bg->in_blanking[sid] = false;
  bg->want[sid] = true;
  bg_try_start_render(bg, sid);

  sem_post(&bg->lock);
}

/* ------------------------------------------------------------------
 * TE rising edge (scanning finished, blanking begins): the only gate at
 * which a (whole-frame) write may start.
 * ------------------------------------------------------------------ */

void nyabula_bg_on_blank_start(nyabula_bg_t *bg, int sid)
{
  sem_wait(&bg->lock);

  bg->in_blanking[sid] = true;
  bg_try_start_xfer(bg);

  sem_post(&bg->lock);
}

/* ------------------------------------------------------------------
 * Render done: content is ready; the generation moves to "ready" (working
 * buffer -> ready buffer; buffer occupancy is unchanged).  Kick off a write
 * (if in blanking) and the next render.
 * ------------------------------------------------------------------ */

void nyabula_bg_on_render_done(nyabula_bg_t *bg, int sid, int gen)
{
  sem_wait(&bg->lock);

  if (bg->rendering_gen[sid] == gen)
    {
      bg->rendering_gen[sid] = -1;
    }

  bg->ready_gen[sid] = gen;

  bg_try_start_xfer(bg);
  bg_try_start_render(bg, sid);

  sem_post(&bg->lock);
}

/* ------------------------------------------------------------------
 * Whole-frame write done: release the single-flight bus and (because a
 * generation is complete exactly when its single whole-frame write ends)
 * tell the framework the buffer is free; then try the next write / render.
 * ------------------------------------------------------------------ */

void nyabula_bg_on_xfer_done(nyabula_bg_t *bg, int sid, int gen)
{
  sem_wait(&bg->lock);

  if (bg->xfer_gen[sid] == gen)
    {
      bg->xfer_gen[sid] = -1;
    }

  bg->xfer_busy = false;

  /* The whole-frame write of gen is complete: the physical buffer is free
   * for LVGL to reuse. */
  if (bg->cb.on_gen_complete != NULL)
    {
      bg->cb.on_gen_complete(bg->screen[sid], gen);
    }

  bg_try_start_xfer(bg);
  bg_try_start_render(bg, sid);

  sem_post(&bg->lock);
}

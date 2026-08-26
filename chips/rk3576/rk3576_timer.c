/****************************************************************************
 * vendor/rockchip/chips/rk3576/rk3576_timer.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * RK3576 TIMER lower-half driver.  Implements the NuttX
 * struct timer_lowerhalf_s so the upper-half (drivers/timers/timer.c) can
 * register a /dev/timerN character device.
 *
 * Each instance maps to one TIMER_NS_0 / TIMER_NS_1 channel.  Channels are
 * configured as follows (TRM §14.5.2 programming flow):
 *
 *   - user-defined count mode (timer_mode=1): the counter is NOT reloaded
 *     automatically on expiry; the ISR re-arms it when a periodic callback
 *     requests reload.
 *   - count-down (count_mode=1): loads LOAD_COUNT1/0 and counts down to 0.
 *   - interrupt enabled to notify the upper-half on expiry.
 *
 * The counting clock is resolved at runtime through the NuttX CLK
 * framework: each channel's clk_timerN is looked up and enabled, and its
 * actual rate is read from hardware with clk_get_rate().  The CRU mux that
 * selects the source is never touched by this driver: the timer blocks
 * intentionally stay on their reset default xin_osc0 (24 MHz).  This is a
 * deliberate design choice, because the root mux is shared across whole
 * timer blocks (TIMER_NS_0 CH0..CH5 share one mux, TIMER_NS_1 CH0/3/4/5
 * share another), so switching it for one channel would silently change
 * the rate of every other channel in that block.  Load values are computed
 * from the real counting-clock frequency read from hardware instead of a
 * hard-coded constant, which also stays correct if a bootloader earlier
 * reprogrammed the mux away from the 24 MHz default.
 *
 * The hardware counter and its two 32-bit load registers are 64-bit, so the
 * whole derived load count is programmed (low half into LOAD_COUNT0, high
 * half into LOAD_COUNT1) and both current-value registers are read back.
 * The NuttX timer framework carries timeouts as uint32_t microseconds
 * (struct timer_status_s), which is the only binding constraint on the
 * interval (~2^32 us ≈ 71 minutes); the hardware never limits it.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <debug.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include <nuttx/arch.h>
#include <nuttx/clk/clk.h>
#include <nuttx/irq.h>
#include <nuttx/spinlock.h>
#include <nuttx/timers/timer.h>

#include "arm64_arch.h"
#include "hardware/rk3576_timer.h"
#include "rk3576_timer.h"

#ifdef CONFIG_RK3576_TIMER

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Convert microseconds to a 64-bit load count at a given counting-clock
 * frequency.  freq is cached in priv->freq from clk_get_rate() at init,
 * so the conversion always tracks the actual clock source selected by the
 * CRU mux.  The split handles non-integral frequencies.
 */

#define TIMER_USEC_TO_TICKS(freq, us)         \
  (((uint64_t)(us) * ((freq) / 1000000ULL)) + \
   (((uint64_t)(us) * ((freq) % 1000000ULL)) / 1000000ULL))

/* ticks -> us.  Defensive form: guard against freq < 1 MHz (freq/1e6 == 0
 * would divide by zero) by using a single 64-bit multiply-then-divide.
 * With t bounded by UINT32_MAX us * freq the product stays well inside
 * uint64_t for every real RK3576 timer rate.
 */

#define TIMER_TICKS_TO_USEC(freq, t) \
  ((uint32_t)(((uint64_t)(t)*1000000ULL) / ((freq) ? (freq) : 1ULL)))

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* One TIMER channel.  The first member is the lower-half ops pointer so a
 * struct timer_lowerhalf_s pointer can be up-cast to this structure.
 */

struct rk3576_timer_s
{
  FAR const struct timer_ops_s *ops; /* Lower-half operations (must be 1st) */
  uintptr_t base;                    /* Channel register base address        */
  int irq;                           /* GIC INTID for this channel           */
  bool started;                      /* Counter currently running            */
  uint32_t timeout;                  /* Current timeout in microseconds      */
  uint32_t gen;                      /* Bumped by START/STOP/SETTIMEOUT      */
  tccb_t callback;                   /* Upper-half expiry callback           */
  FAR void *arg;                     /* Callback argument                    */
  FAR struct clk_s *clk;             /* Channel counting clock (clk_timerN)  */
  uint32_t freq;                     /* Counting-clock frequency (Hz)        */
  spinlock_t lock;                   /* Protects regs + state vs. ISR/ioctl */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static uint32_t rk3576_timer_getreg(FAR struct rk3576_timer_s *priv,
                                    uint32_t off);
static void rk3576_timer_putreg(FAR struct rk3576_timer_s *priv, uint32_t off,
                                uint32_t val);

static int rk3576_timer_start(FAR struct timer_lowerhalf_s *lower);
static int rk3576_timer_stop(FAR struct timer_lowerhalf_s *lower);
static int rk3576_timer_getstatus(FAR struct timer_lowerhalf_s *lower,
                                  FAR struct timer_status_s *status);
static int rk3576_timer_settimeout(FAR struct timer_lowerhalf_s *lower,
                                   uint32_t timeout);
static void rk3576_timer_setcallback(FAR struct timer_lowerhalf_s *lower,
                                     tccb_t callback, FAR void *arg);

/* Lock-free inner helpers shared by the public lower-half methods and the
 * ISR path.  Callers must already hold priv->lock.
 */

static int rk3576_timer_start_locked(FAR struct rk3576_timer_s *priv);
static int rk3576_timer_stop_locked(FAR struct rk3576_timer_s *priv);
static int rk3576_timer_settimeout_locked(FAR struct rk3576_timer_s *priv,
                                          uint32_t timeout);
static int rk3576_timer_ioctl(FAR struct timer_lowerhalf_s *lower, int cmd,
                              unsigned long arg);
static int rk3576_timer_maxtimeout(FAR struct timer_lowerhalf_s *lower,
                                   FAR uint32_t *maxtimeout);

static int rk3576_timer_interrupt(int irq, FAR void *context, FAR void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct timer_ops_s g_rk3576_timer_ops = {
  .start = rk3576_timer_start,
  .stop = rk3576_timer_stop,
  .getstatus = rk3576_timer_getstatus,
  .settimeout = rk3576_timer_settimeout,
  .setcallback = rk3576_timer_setcallback,
  .ioctl = rk3576_timer_ioctl,
  .maxtimeout = rk3576_timer_maxtimeout,
};

/* Base addresses for the two non-secure timer blocks. */

static const uintptr_t g_timer_base[2] = {
  RK3576_TIMER_NS0_ADDR,
  RK3576_TIMER_NS1_ADDR,
};

/* GIC INTIDs of TIMER_NS_0 CH0..CH5 (TRM Table 1-3, IDs 77..82). */

static const int g_timer_irq[2][RK3576_TIMER_CHANS] = {
  {
      RK3576_IRQ_TIMER_NS_0_CH0,
      RK3576_IRQ_TIMER_NS_0_CH1,
      RK3576_IRQ_TIMER_NS_0_CH2,
      RK3576_IRQ_TIMER_NS_0_CH3,
      RK3576_IRQ_TIMER_NS_0_CH4,
      RK3576_IRQ_TIMER_NS_0_CH5,
  },
  {
      RK3576_IRQ_TIMER_NS_1_CH0,
      RK3576_IRQ_TIMER_NS_1_CH1,
      RK3576_IRQ_TIMER_NS_1_CH2,
      RK3576_IRQ_TIMER_NS_1_CH3,
      RK3576_IRQ_TIMER_NS_1_CH4,
      RK3576_IRQ_TIMER_NS_1_CH5,
  },
};

/* Static per-channel instances; only the slots a board actually uses are
 * handed out by rk3576_timer_initialize().
 */

static struct rk3576_timer_s g_rk3576_timer[2][RK3576_TIMER_CHANS];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static uint32_t rk3576_timer_getreg(FAR struct rk3576_timer_s *priv,
                                    uint32_t off)
{
  return getreg32(priv->base + off);
}

static void rk3576_timer_putreg(FAR struct rk3576_timer_s *priv, uint32_t off,
                                uint32_t val)
{
  putreg32(val, priv->base + off);
}

/****************************************************************************
 * Name: rk3576_timer_interrupt
 *
 * Description:
 *   TIMER channel interrupt handler.  Clears the interrupt status and
 *   invokes the upper-half callback.  If the callback returns true (i.e.
 *   a periodic timer), the channel is re-armed with the requested next
 *   interval before being restarted.
 ****************************************************************************/

static int rk3576_timer_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct rk3576_timer_s *priv = arg;
  irqstate_t flags;
  tccb_t callback;
  FAR void *cbarg;
  uint32_t gen;
  uint32_t next_interval = 0;

  /* The ISR races with user ioctl (TCIOC_START/STOP/SETTIMEOUT), which the
   * upper-half `struct timer_upperhalf_s->lock` serializes among processes
   * but NOT against this interrupt path.  Take priv->lock to serialize the
   * shared register writes and priv fields.  The ISR runs with interrupts
   * already masked on this CPU, and spin_lock_irqsave is safe here.
   */

  flags = spin_lock_irqsave(&priv->lock);

  /* Write 1 to clear the interrupt (TRM §14.4.3, W1C). */

  rk3576_timer_putreg(priv, RK3576_TIMER_INTSTATUS, TIMER_INTSTATUS_PD);

  /* Snapshot callback/arg and the state-generation counter under the lock
   * so the user callback runs without holding the spinlock (it must stay
   * free to call back into this driver).  Any ioctl that changes the
   * running state (START/STOP/SETTIMEOUT) bumps priv->gen, so comparing it
   * after the callback detects whether the state changed while the lock
   * was dropped.
   */

  callback = priv->callback;
  cbarg = priv->arg;
  gen = priv->gen;

  if (callback == NULL)
    {
      /* No upper-half handler registered: nothing to notify, the channel
       * simply expired and is now stopped.  Fall through to the common
       * unlock/return.
       */

      priv->started = false;
      spin_unlock_irqrestore(&priv->lock, flags);
      return OK;
    }

  spin_unlock_irqrestore(&priv->lock, flags);

  if (callback(&next_interval, cbarg))
    {
      /* Periodic: reload with the (possibly updated) interval and restart
       * the down-counter.  Re-arm only if no ioctl changed the running
       * state while the callback ran with the lock dropped; otherwise leave
       * the channel exactly as the ioctl set it.
       */

      flags = spin_lock_irqsave(&priv->lock);
      if (priv->gen == gen)
        {
          if (next_interval > 0)
            {
              rk3576_timer_settimeout_locked(priv, next_interval);
            }

          rk3576_timer_start_locked(priv);
        }
      spin_unlock_irqrestore(&priv->lock, flags);
      return OK;
    }

  /* One-shot: the channel has expired.  Only mark it stopped if the
   * state did not change during the callback; a concurrent TCIOC_START
   * has already re-enabled the hardware, so leave priv->started as the
   * ioctl set it.
   */

  flags = spin_lock_irqsave(&priv->lock);
  if (priv->gen == gen)
    {
      priv->started = false;
    }
  spin_unlock_irqrestore(&priv->lock, flags);

  return OK;
}

/****************************************************************************
 * Name: rk3576_timer_start
 *
 * Description:
 *   Start the timer, counting down from the previously loaded value.
 ****************************************************************************/

static int rk3576_timer_start_locked(FAR struct rk3576_timer_s *priv)
{
  uint32_t control;

  /* user-defined count-down, interrupt enabled, timer enabled. */

  control = TIMER_CONTROL_MODE | TIMER_CONTROL_COUNT_MODE |
            TIMER_CONTROL_INT_EN | TIMER_CONTROL_EN;
  rk3576_timer_putreg(priv, RK3576_TIMER_CONTROL, control);

  priv->started = true;
  return OK;
}

/****************************************************************************
 * Name: rk3576_timer_start
 *
 * Description:
 *   Start the timer, counting down from the previously loaded value.
 *   Serializes with the ISR reload path via priv->lock.
 ****************************************************************************/

static int rk3576_timer_start(FAR struct timer_lowerhalf_s *lower)
{
  FAR struct rk3576_timer_s *priv = (FAR struct rk3576_timer_s *)lower;
  irqstate_t flags;
  int ret;

  flags = spin_lock_irqsave(&priv->lock);
  priv->gen++;
  ret = rk3576_timer_start_locked(priv);
  spin_unlock_irqrestore(&priv->lock, flags);

  return ret;
}

static int rk3576_timer_stop_locked(FAR struct rk3576_timer_s *priv)
{
  rk3576_timer_putreg(priv, RK3576_TIMER_CONTROL, 0);
  priv->started = false;
  return OK;
}

/****************************************************************************
 * Name: rk3576_timer_stop
 *
 * Description:
 *   Stop the timer.  The counter is disabled; reconfiguration is allowed
 *   once timer_en is low (TRM §14.5.1).
 ****************************************************************************/

static int rk3576_timer_stop(FAR struct timer_lowerhalf_s *lower)
{
  FAR struct rk3576_timer_s *priv = (FAR struct rk3576_timer_s *)lower;
  irqstate_t flags;
  int ret;

  flags = spin_lock_irqsave(&priv->lock);
  priv->gen++;
  ret = rk3576_timer_stop_locked(priv);
  spin_unlock_irqrestore(&priv->lock, flags);

  return ret;
}

/****************************************************************************
 * Name: rk3576_timer_getstatus
 *
 * Description:
 *   Get the current timer status: flags, configured timeout (us) and the
 *   remaining time (us) until expiry.  The remaining time is read from the
 *   current-value registers, which hold the live down-count.
 ****************************************************************************/

static int rk3576_timer_getstatus(FAR struct timer_lowerhalf_s *lower,
                                  FAR struct timer_status_s *status)
{
  FAR struct rk3576_timer_s *priv = (FAR struct rk3576_timer_s *)lower;
  irqstate_t flags;
  uint32_t cur_hi;
  uint32_t cur_lo;
  uint32_t cur_hi2;
  uint64_t current;

  /* Serialize with the ISR reload path / TCIOC_STOP so started, timeout and
   * the live count form a consistent snapshot.
   */

  flags = spin_lock_irqsave(&priv->lock);

  status->flags = (priv->started ? TCFLAGS_ACTIVE : 0) |
                  (priv->callback != NULL ? TCFLAGS_HANDLER : 0);
  status->timeout = priv->timeout;

  /* The APB interface and each CURR_VALUE register are 32-bit only, so the
   * 64-bit down-counter cannot be read atomically in a single bus access.
   * Guard against tearing between the two halves with the classic
   * double-read/high-word-check: reread CURR_VALUE1 until a stable value
   * is seen.  RE-READ is cheap because the high 32 bits only change once
   * every 2^32/freq (≈42 s at 100 MHz, ≈179 s at 24 MHz).
   */

  do
    {
      cur_hi = rk3576_timer_getreg(priv, RK3576_TIMER_CURR_VALUE1);
      cur_lo = rk3576_timer_getreg(priv, RK3576_TIMER_CURR_VALUE0);
      cur_hi2 = rk3576_timer_getreg(priv, RK3576_TIMER_CURR_VALUE1);
    }
  while (cur_hi != cur_hi2);

  current = ((uint64_t)cur_hi2 << 32) | (uint64_t)cur_lo;

  status->timeleft = TIMER_TICKS_TO_USEC(priv->freq, current);

  spin_unlock_irqrestore(&priv->lock, flags);

  return OK;
}

/****************************************************************************
 * Name: rk3576_timer_settimeout_locked
 *
 * Description:
 *   Lock-free core of settimeout(); caller must hold priv->lock.
 ****************************************************************************/

static int rk3576_timer_settimeout_locked(FAR struct rk3576_timer_s *priv,
                                          uint32_t timeout)
{
  uint64_t ticks;

  /* Must disable before reprogramming load registers (TRM §14.5.1). */

  rk3576_timer_putreg(priv, RK3576_TIMER_CONTROL, 0);
  priv->started = false;

  ticks = TIMER_USEC_TO_TICKS(priv->freq, timeout);
  if (ticks == 0)
    {
      ticks = 1;
    }

  /* The counter and its two load registers are 64-bit, so no upper-bound
   * check is needed: ticks is derived from a 32-bit microsecond timeout,
   * at most UINT32_MAX * 100 (100 MHz) ≈ 2^38.6, far below the 64-bit
   * hardware limit.  Both halves of the load count are programmed below.
   */

  /* Load value: low 32 bits into LOAD_COUNT0, high into LOAD_COUNT1. */

  rk3576_timer_putreg(priv, RK3576_TIMER_LOAD_COUNT0, (uint32_t)ticks);
  rk3576_timer_putreg(priv, RK3576_TIMER_LOAD_COUNT1, (uint32_t)(ticks >> 32));

  priv->timeout = timeout;
  return OK;
}

/****************************************************************************
 * Name: rk3576_timer_settimeout
 *
 * Description:
 *   Set a new timeout value (in microseconds).  The timer is disabled, the
 *   load count is programmed, and the timer is left stopped so the caller
 *   can start it via TCIOC_START when ready.
 ****************************************************************************/

static int rk3576_timer_settimeout(FAR struct timer_lowerhalf_s *lower,
                                   uint32_t timeout)
{
  FAR struct rk3576_timer_s *priv = (FAR struct rk3576_timer_s *)lower;
  irqstate_t flags;
  int ret;

  flags = spin_lock_irqsave(&priv->lock);
  priv->gen++;
  ret = rk3576_timer_settimeout_locked(priv, timeout);
  spin_unlock_irqrestore(&priv->lock, flags);

  return ret;
}

/****************************************************************************
 * Name: rk3576_timer_setcallback
 *
 * Description:
 *   Register (or clear) the upper-half callback invoked on expiry.
 ****************************************************************************/

static void rk3576_timer_setcallback(FAR struct timer_lowerhalf_s *lower,
                                     tccb_t callback, FAR void *arg)
{
  FAR struct rk3576_timer_s *priv = (FAR struct rk3576_timer_s *)lower;
  irqstate_t flags;

  flags = spin_lock_irqsave(&priv->lock);
  priv->callback = callback;
  priv->arg = arg;
  spin_unlock_irqrestore(&priv->lock, flags);
}

/****************************************************************************
 * Name: rk3576_timer_ioctl
 *
 * Description:
 *   Forward any ioctl commands not handled by the upper-half driver.
 *   None are implemented for this hardware, so always return -ENOTTY.
 ****************************************************************************/

static int rk3576_timer_ioctl(FAR struct timer_lowerhalf_s *lower, int cmd,
                              unsigned long arg)
{
  return -ENOTTY;
}

/****************************************************************************
 * Name: rk3576_timer_maxtimeout
 *
 * Description:
 *   Return the maximum supported timeout in microseconds.  The hardware
 *   64-bit counter could represent far more, but the NuttX timer framework
 *   carries the timeout as uint32_t microseconds, so UINT32_MAX is the
 *   effective ceiling (~71 minutes).
 ****************************************************************************/

static int rk3576_timer_maxtimeout(FAR struct timer_lowerhalf_s *lower,
                                   FAR uint32_t *maxtimeout)
{
  *maxtimeout = UINT32_MAX;
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rk3576_timer_initialize
 *
 * Description:
 *   Return the lower-half handle for one RK3576 TIMER channel for the
 *   board to pass to timer_register().  The counting-clock source is never
 *   switched by this driver: the timer blocks stay on their reset default
 *   xin_osc0 (24 MHz), because the root CRU mux is shared across whole
 *   timer blocks and reprogramming it for one channel would silently change
 *   the rate of every other channel in that block.  The channel's actual
 *   rate is still read from hardware with clk_get_rate(), so load
 *   conversions stay correct even if a bootloader earlier moved the mux
 *   away from the 24 MHz default.
 *
 * Input Parameters:
 *   timer    - Timer block index 0 or 1.
 *   channel  - Channel index within the block (0-based,
 *              < RK3576_TIMER_CHANS).
 *
 * Returned Value:
 *   A timer_lowerhalf_s handle on success; NULL on an invalid timer/channel
 *   or if the interrupt could not be attached.
 ****************************************************************************/

FAR struct timer_lowerhalf_s *rk3576_timer_initialize(int timer, int channel)
{
  FAR struct rk3576_timer_s *priv;
  char clk_name[16];
  FAR struct clk_s *clk;
  uint32_t freq;
  int ret;

  if (timer < 0 || timer >= 2)
    {
      tmrerr("ERROR: invalid TIMER block %d\n", timer);
      return NULL;
    }

  if (channel < 0 || channel >= RK3576_TIMER_CHANS)
    {
      tmrerr("ERROR: TIMER_NS%d channel %d out of range (max %d)\n", timer,
             channel, RK3576_TIMER_CHANS - 1);
      return NULL;
    }

  priv = &g_rk3576_timer[timer][channel];

  /* Initialize the channel state before the IRQ line can ever be enabled:
   * base/lock/ops first, then stop and clear the hardware, then resolve
   * the counting clock, and only afterwards attach + enable the GIC IRQ.
   * This guarantees that a stale pending interrupt can never enter the ISR
   * with a half-initialized priv or an enabled counter.
   */

  priv->ops = &g_rk3576_timer_ops;
  priv->base =
      g_timer_base[timer] + (uint32_t)channel * RK3576_TIMER_CH_STRIDE;
  priv->irq = g_timer_irq[timer][channel];
  priv->started = false;
  priv->timeout = 0;
  priv->gen = 0;
  priv->callback = NULL;
  priv->arg = NULL;
  priv->clk = NULL;
  priv->freq = 0;
  spin_lock_init(&priv->lock);

  /* Leave the channel disabled with a clean control word and clear any
   * stale pending interrupt status (W1C) before the GIC line is enabled.
   */

  rk3576_timer_putreg(priv, RK3576_TIMER_CONTROL, 0);
  rk3576_timer_putreg(priv, RK3576_TIMER_INTSTATUS, TIMER_INTSTATUS_PD);

  /* Resolve the counting clock through the CLK framework.  Each channel
   * maps to clk_timerN with N = timer*6 + channel (TRM Table 14-1):
   *   TIMER_NS_0 CH0..CH5 -> clk_timer0..clk_timer5
   *   TIMER_NS_1 CH0..CH5 -> clk_timer6..clk_timer11
   * The source is left at the reset default xin_osc0 (24 MHz) and its
   * actual frequency is read from hardware, so load conversions always use
   * the real counting clock instead of a hard-coded 24 MHz constant (this
   * stays correct even if a bootloader moved the shared mux).
   */

  snprintf(clk_name, sizeof(clk_name), "clk_timer%d", timer * 6 + channel);
  clk = clk_get(clk_name);
  if (clk == NULL)
    {
      tmrerr("ERROR: failed to get clock %s\n", clk_name);
      return NULL;
    }

  ret = clk_enable(clk);
  if (ret < 0)
    {
      tmrerr("ERROR: failed to enable clock %s\n", clk_name);
      return NULL;
    }

  freq = clk_get_rate(clk);
  if (freq == 0)
    {
      tmrerr("ERROR: clock %s has invalid rate 0\n", clk_name);
      clk_disable(clk);
      return NULL;
    }

  priv->clk = clk;
  priv->freq = freq;

  /* Attach the interrupt handler and enable the GIC line last, after priv
   * is fully initialized and the channel is stopped with no pending
   * interrupt, so the ISR can never observe an uninitialized state.
   */

  ret = irq_attach(g_timer_irq[timer][channel], rk3576_timer_interrupt, priv);
  if (ret < 0)
    {
      tmrerr("ERROR: failed to attach IRQ %d\n", g_timer_irq[timer][channel]);
      clk_disable(clk);
      return NULL;
    }

  up_enable_irq(g_timer_irq[timer][channel]);

  return (FAR struct timer_lowerhalf_s *)priv;
}

#endif /* CONFIG_RK3576_TIMER */

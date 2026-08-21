/****************************************************************************
 * vendor/rockchip/chips/rk3576/rk3576_timer.h
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
 * Public API for the RK3576 TIMER lower-half driver.
 *
 * The driver implements the NuttX struct timer_lowerhalf_s so the
 * upper-half (drivers/timers/timer.c) can register a /dev/timerN character
 * device.  Each instance maps to one TIMER_NS_0 or TIMER_NS_1 channel.
 ****************************************************************************/

#ifndef __VENDOR_ROCKCHIP_CHIPS_RK3576_RK3576_TIMER_H
#define __VENDOR_ROCKCHIP_CHIPS_RK3576_RK3576_TIMER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/timers/timer.h>

#ifdef CONFIG_RK3576_TIMER

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RK3576_TIMER_CHANS 6 /* Channels per timer block */

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: rk3576_timer_initialize
 *
 * Description:
 *   Return the lower-half handle for one RK3576 TIMER channel for the
 *   board to pass to timer_register().
 *
 *   The counting-clock source is not configurable and its selecting CRU
 *   mux is never touched by this driver: the timer blocks stay on their
 *   reset default xin_osc0 (24 MHz).  This is deliberate, because the root
 *   mux is shared across whole timer blocks (TIMER_NS_0 CH0..CH5 share
 *   clk_timer0_root_sel, and TIMER_NS_1 CH0/3/4/5 share clk_timer1_root_sel;
 *   only TIMER_NS_1 CH1/CH2 have independent muxes).  Reprogramming a
 *   shared mux for one channel would silently change the frequency of every
 *   other channel in that block.
 *
 * Input Parameters:
 *   timer    - Timer block index (RK3576_TIMER_NS0 or RK3576_TIMER_NS1).
 *   channel  - Channel index within the block (0-based, < RK3576_TIMER_CHANS).
 *
 * Returned Value:
 *   A timer_lowerhalf_s handle on success; NULL on an invalid timer/channel
 *   or if the interrupt could not be attached.
 *
 ****************************************************************************/

FAR struct timer_lowerhalf_s *rk3576_timer_initialize(int timer, int channel);

#endif /* CONFIG_RK3576_TIMER */
#endif /* __VENDOR_ROCKCHIP_CHIPS_RK3576_RK3576_TIMER_H */

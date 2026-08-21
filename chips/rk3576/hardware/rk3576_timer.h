/****************************************************************************
 * vendor/rockchip/chips/rk3576/hardware/rk3576_timer.h
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
 * The RK3576 TIMER is a simple programmable interval timer with two count
 * modes (count-up / count-down) and two timer modes (free-running /
 * user-defined).  Each channel occupies an independent 0x1000 register
 * window.  The counting clock is switched between 100 MHz and 24 MHz
 * (clk_timern, sourced from clk_matrix_100m / osc_xin per TRM Table 14-1).
 *
 * This header covers the non-secure (NS) instances, which the AP can
 * access directly: TIMER_NS_0 (6 channels @ 0x2ACC0000) and TIMER_NS_1
 * (6 channels @ 0x2ACD0000).
 *
 * Reference: Rockchip RK3576 TRM Part1 V1.2, Chapter 14 "TIMER".
 ****************************************************************************/

#ifndef __VENDOR_ROCKCHIP_RK3576_HARDWARE_RK3576_TIMER_H
#define __VENDOR_ROCKCHIP_RK3576_HARDWARE_RK3576_TIMER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "rk3576_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Instance base addresses live in rk3576_memorymap.h:
 *   RK3576_TIMER_NS0_ADDR (@ 0x2ACC0000)  ->  TIMER_NS_0, 6 channels
 *   RK3576_TIMER_NS1_ADDR (@ 0x2ACD0000)  ->  TIMER_NS_1, 6 channels
 * Each channel occupies an independent 0x1000 window, so a channel base is
 *   base + channel * RK3576_TIMER_CH_STRIDE
 */

#define RK3576_TIMER_CH_STRIDE 0x1000

/* Per-channel register offset (identical layout for every channel) ********/

#define RK3576_TIMER_LOAD_COUNT0 0x0000 /* Load value, low 32 bits       */
#define RK3576_TIMER_LOAD_COUNT1 0x0004 /* Load value, high 32 bits      */
#define RK3576_TIMER_CURR_VALUE0 0x0008 /* Current value, low 32 bits    */
#define RK3576_TIMER_CURR_VALUE1 0x000C /* Current value, high 32 bits   */
#define RK3576_TIMER_CONTROL     0x0010 /* Control register              */
#define RK3576_TIMER_INTSTATUS   0x0018 /* Interrupt status (W1C)        */
#define RK3576_TIMER_REVISION    0x001C /* Version (RO)                  */

/* TIMER_CONTROL (0x0010) bit fields ****************************************/

#define TIMER_CONTROL_EN         (1 << 0) /* 1: enable the timer            */
#define TIMER_CONTROL_MODE       (1 << 1) /* 1: user-defined, 0: free-run   */
#define TIMER_CONTROL_INT_EN     (1 << 2) /* 1: interrupt enable            */
#define TIMER_CONTROL_COUNT_MODE (1 << 3) /* 1: count-down, 0: count-up */

/* TIMER_INTSTATUS (0x0018) bit fields.  Writing 1 clears the interrupt. */

#define TIMER_INTSTATUS_PD (1 << 0) /* Interrupt status / W1C         */

#endif /* __VENDOR_ROCKCHIP_RK3576_HARDWARE_RK3576_TIMER_H */

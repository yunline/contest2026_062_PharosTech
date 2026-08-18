/****************************************************************************
 * apps/graphics/nyabula_display/nyabula_display_audit.h
 *
 * Built-in audit / profiling instrumentation for the dual-LCD layer.
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

#ifndef __NYABULA_DISPLAY_AUDIT_H
#define __NYABULA_DISPLAY_AUDIT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* ------------------------------------------------------------------
 * Built-in audit / profiling instrumentation.
 *
 * The dual-screen compat layer bypasses LVGL's own sysmon / FPS auditor,
 * so we provide our own per-screen metrics:
 *   - render time  : time between two consecutive flush callbacks
 *   - transfer time: wall time of one half-frame QSPI PUTAREA ioctl
 *   - transfer FPS : 1 / (interval between two front-half transfers),
 *                    i.e. the refresh period for the front half of the frame
 *
 * The audit state and all measurement code live in a dedicated translation
 * unit (nyabula_display_audit.c).  When NYABULA_AUDIT is 0 that file
 * compiles to nothing (no storage, no functions) and the entry points
 * below expand to empty macros, stripping all instrumentation -- and its
 * overhead -- from the build.
 *
 * The switch is controlled by the Kconfig option CONFIG_NYABULA_DISPLAY_AUDIT
 * (see Kconfig).  It can still be overridden at compile time by defining
 * NYABULA_AUDIT explicitly (e.g. a -D flag); the #ifndef guard keeps that
 * escape hatch.
 * ------------------------------------------------------------------ */

/* Print the audit values every this many TE frames (one frame = one scan
 * period of the software TE frame clock). */

#ifndef NYABULA_AUDIT_PRINT_EVERY
#define NYABULA_AUDIT_PRINT_EVERY 20
#endif

/****************************************************************************
 * Public Type Declarations
 ****************************************************************************/

/* Forward declaration of the dual-LCD context.  The audit interface only
 * passes a const pointer through to read the per-screen "initialized" flag
 * (for the periodic print); it never dereferences other fields, so an
 * opaque pointer is sufficient here.  The full definition lives in
 * nyabula_dual_lcd.h (included only by the audit implementation TU). */

struct nyabula_dual_lcd_s;

#ifdef CONFIG_NYABULA_DISPLAY_AUDIT

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Zero the global audit state.  Call once at create time. */
void nyabula_disp_audit_init(void);

/* flush_cb: one real frame render completed -> count it and measure the
 * render interval (time since the previous flush). */
void nyabula_disp_audit_flush_end(int sid);

/* flush_wait_cb: measure how long the render thread waits for a free
 * double-buffer. */
void nyabula_disp_audit_flush_wait_start(int sid);
void nyabula_disp_audit_flush_wait_end(int sid);

/* Deferred render wait: the render loop defers a screen whose double-buffer
 * is full (v6 _db_pending), rendering other screens first, then waits here
 * for a free buffer before rendering the deferred screen. */
void nyabula_disp_audit_defer_wait_start(int sid);
void nyabula_disp_audit_defer_wait_end(int sid);

/* Transfer thread: measure one half-frame PUTAREA ioctl wall time (and the
 * front-half refresh period that yields the transfer FPS). */
void nyabula_disp_audit_transfer_start(int sid, int start_line);
void nyabula_disp_audit_transfer_end(int sid);

/* nyabula_dual_lcd_task(): drive-loop iteration period. */
void nyabula_disp_audit_loop(const struct nyabula_dual_lcd_s *dual);

/* Render loop: measure one lv_refr_now() call and count refreshes. */
void nyabula_disp_audit_render_start(int sid);
void nyabula_disp_audit_render_end(int sid);

/* Render loop: measure how long it blocks on render_kick. */
void nyabula_disp_audit_waitkick_start(const struct nyabula_dual_lcd_s *dual);
void nyabula_disp_audit_waitkick_end(const struct nyabula_dual_lcd_s *dual);

/* TE thread: called each quarter tick; on frame boundaries measures the real
 * TE frame period and periodically prints all metrics. */
void nyabula_disp_audit_te_frame(const struct nyabula_dual_lcd_s *dual,
                                 uint8_t tick);

#else /* !def CONFIG_NYABULA_DISPLAY_AUDIT */

/* Empty stubs: absorb the arguments so call sites stay identical regardless
 * of the audit switch (no unused-variable / unused-argument warnings). */

#define nyabula_disp_audit_init()
#define nyabula_disp_audit_flush_end(_s)          (void)(_s)
#define nyabula_disp_audit_flush_wait_start(_s)   (void)(_s)
#define nyabula_disp_audit_flush_wait_end(_s)     (void)(_s)
#define nyabula_disp_audit_defer_wait_start(_s)   (void)(_s)
#define nyabula_disp_audit_defer_wait_end(_s)     (void)(_s)
#define nyabula_disp_audit_transfer_start(_s, _r) ((void)(_s), (void)(_r))
#define nyabula_disp_audit_transfer_end(_s)       (void)(_s)
#define nyabula_disp_audit_loop(_d)               (void)(_d)
#define nyabula_disp_audit_render_start(_s)       (void)(_s)
#define nyabula_disp_audit_render_end(_s)         (void)(_s)
#define nyabula_disp_audit_waitkick_start(_d)     (void)(_d)
#define nyabula_disp_audit_waitkick_end(_d)       (void)(_d)
#define nyabula_disp_audit_te_frame(_d, _t)       ((void)(_d), (void)(_t))

#endif /* CONFIG_NYABULA_DISPLAY_AUDIT */

#ifdef __cplusplus
}
#endif

#endif /* __NYABULA_DISPLAY_AUDIT_H */

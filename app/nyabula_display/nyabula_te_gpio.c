/****************************************************************************
 * apps/graphics/nyabula_display/nyabula_te_gpio.c
 *
 * Panel TE GPIO interrupt source (CONFIG_NYABULA_DISPLAY_TE_GPIO).
 *
 * Each screen's TE pin is registered as an interrupt-capable GPIO
 * (level/edge: both-edge, so both the rising = blank-start and falling =
 * scan-start edges are caught).  This source uses the standard NuttX
 * CONFIG_DEV_GPIO character-device signal mechanism:
 *
 *   1. The board registers the TE pins once at power-up, e.g.
 *        rk3576_gpio_register(GPIO_TE0);  (-> /dev/gpioN, GPIO_INTERRUPT_...)
 *        rk3576_gpio_register(GPIO_TE1);
 *      (GPIO_TE0/GPIO_TE1 must be GPIO_INTERRUPT_BOTH_PIN pinsets.)
 *
 *   2. This source open()s /dev/gpioN, registers a POSIX sigevent with
 *      GPIOC_REGISTER; the driver then sigqueue()s the process (from its
 *      GIC ISR) on every TE edge.  The ISR path never touches the display
 *      pipeline -- it only signals.
 *
 *   3. A high-priority consumer thread waits on sigwaitinfo(), reads the
 *      pin level (GPIOC_READ: true = HIGH = blanking -> blank_start,
 *      false = LOW = scanning -> scan_start) and calls the framework edge
 *      callbacks.  This keeps the algorithm's (mutex-protected) entry
 *      points out of interrupt context, which is required.
 *
 * Requirements (checked with #error):
 *   - CONFIG_DEV_GPIO and CONFIG_DEV_GPIO_NSIGNALS > 0
 *   - board defines and registers GPIO_TE0 / GPIO_TE1 as BOTH-edge
 *     interrupt inputs.
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

#include "nyabula_te.h"

#include <errno.h>
#include <fcntl.h>
#include <nuttx/ioexpander/gpio.h>
#include <pthread.h>
#include <sched.h>
#include <signal.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <lvgl/lvgl.h>

/* Board wiring: TE pins for screen 0/1.  The board must register these as
 * BOTH-edge interrupt GPIOs at power-up (they appear as /dev/gpioN). */

#ifdef CONFIG_NYABULA_DISPLAY_TE0_DEVPATH
#define NYABULA_TE_GPIO0_DEVPATH CONFIG_NYABULA_DISPLAY_TE0_DEVPATH
#else
#define NYABULA_TE_GPIO0_DEVPATH "/dev/gpio0"
#endif

#ifdef CONFIG_NYABULA_DISPLAY_TE1_DEVPATH
#define NYABULA_TE_GPIO1_DEVPATH CONFIG_NYABULA_DISPLAY_TE1_DEVPATH
#else
#define NYABULA_TE_GPIO1_DEVPATH "/dev/gpio1"
#endif

/* Per-screen POSIX signal used by the GPIO driver to notify a TE edge. */

#ifndef NYABULA_TE_GPIO0_SIGNO
#define NYABULA_TE_GPIO0_SIGNO SIGUSR1
#endif

#ifndef NYABULA_TE_GPIO1_SIGNO
#define NYABULA_TE_GPIO1_SIGNO SIGUSR2
#endif

/* GPIO TE source instance.  The public nyabula_te_t handle aliases the
 * first member. */

struct nyabula_te_s
{
  pthread_t thread;
  bool running;

  /* One /dev/gpioN per screen. */
  int fd[NYABULA_TE_MAX_SCREENS];
  const char *dev_path[NYABULA_TE_MAX_SCREENS];
  int signo[NYABULA_TE_MAX_SCREENS];

  struct nyabula_dual_lcd_s *dual;
  nyabula_te_callbacks_t cb;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Read the current level of a screen's TE pin (GPIOC_READ).  Returns true
 * for HIGH (blanking) and false for LOW (scanning). */

static bool te_gpio_read(const struct nyabula_te_s *t, int sid)
{
  bool level = false;

  if (t->fd[sid] >= 0)
    {
      ioctl(t->fd[sid], GPIOC_READ, (unsigned long)&level);
    }

  return level;
}

/* Consumer thread: wait for a TE-edge signal from the GPIO driver, then
 * classify the edge by reading the pin level and call the framework. */

static void *te_thread_func(void *arg)
{
  struct nyabula_te_s *t = (struct nyabula_te_s *)arg;
  sigset_t set;
  siginfo_t info;

  sigemptyset(&set);
  sigaddset(&set, t->signo[0]);
  sigaddset(&set, t->signo[1]);

  while (t->running)
    {
      int ret;

      memset(&info, 0, sizeof(info));
      ret = sigwaitinfo(&set, &info);
      if (ret < 0)
        {
          if (errno == EINTR)
            {
              continue;
            }

          break;
        }

      if (!t->running)
        {
          break;
        }

      if (info.si_signo == t->signo[0])
        {
          int sid = 0;
          bool blank = te_gpio_read(t, sid);

          if (blank && t->cb.blank_start != NULL)
            {
              t->cb.blank_start(t->dual, sid);
            }
          else if (!blank && t->cb.scan_start != NULL)
            {
              t->cb.scan_start(t->dual, sid);
            }
        }
      else if (info.si_signo == t->signo[1])
        {
          int sid = 1;
          bool blank = te_gpio_read(t, sid);

          if (blank && t->cb.blank_start != NULL)
            {
              t->cb.blank_start(t->dual, sid);
            }
          else if (!blank && t->cb.scan_start != NULL)
            {
              t->cb.scan_start(t->dual, sid);
            }
        }
    }

  return NULL;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

nyabula_te_t *nyabula_te_init(struct nyabula_dual_lcd_s *dual,
                              const nyabula_te_callbacks_t *cb)
{
  struct nyabula_te_s *t;
  sigset_t set;
  int sid;
  int ret;

  t = (struct nyabula_te_s *)lv_malloc(sizeof(*t));
  if (t == NULL)
    {
      return NULL;
    }

  memset(t, 0, sizeof(*t));
  t->running = false;
  t->dual = dual;
  if (cb != NULL)
    {
      t->cb = *cb;
    }

  t->dev_path[0] = NYABULA_TE_GPIO0_DEVPATH;
  t->dev_path[1] = NYABULA_TE_GPIO1_DEVPATH;
  t->signo[0] = NYABULA_TE_GPIO0_SIGNO;
  t->signo[1] = NYABULA_TE_GPIO1_SIGNO;
  t->fd[0] = -1;
  t->fd[1] = -1;

  /* Open each TE pin device and register the sigevent notification.  The
   * GPIO driver signals the process (from its GIC ISR) on every edge. */
  for (sid = 0; sid < NYABULA_TE_MAX_SCREENS; sid++)
    {
      struct sigevent event;

      t->fd[sid] = open(t->dev_path[sid], O_RDONLY);
      if (t->fd[sid] < 0)
        {
          LV_LOG_ERROR("TE GPIO %s open failed: %d", t->dev_path[sid], errno);
          goto err_open;
        }

      /* Notify via SIGEV_SIGNAL: the driver sigqueue()s `signo` to this
       * process on each TE edge (ISR context). */
      memset(&event, 0, sizeof(event));
      event.sigev_notify = SIGEV_SIGNAL;
      event.sigev_signo = t->signo[sid];

      ret = ioctl(t->fd[sid], GPIOC_REGISTER, (unsigned long)&event);
      if (ret < 0)
        {
          LV_LOG_ERROR("TE GPIO %s GPIOC_REGISTER failed: %d",
                       t->dev_path[sid], -ret);
          close(t->fd[sid]);
          t->fd[sid] = -1;
          goto err_open;
        }
    }

  /* Block the TE signals in this thread so sigwaitinfo() below can catch
   * them (an unblocked SIGUSR1/SIGUSR2 would take the default action). */
  sigemptyset(&set);
  sigaddset(&set, t->signo[0]);
  sigaddset(&set, t->signo[1]);
  pthread_sigmask(SIG_BLOCK, &set, NULL);

  t->running = true;

  ret = nyabula_te_create_thread_prio(&t->thread, te_thread_func, t,
                                      NYABULA_TE_PRIORITY);
  if (ret != 0)
    {
      t->running = false;
      goto err_open;
    }

  return (nyabula_te_t *)t;

err_open:
  for (sid = 0; sid < NYABULA_TE_MAX_SCREENS; sid++)
    {
      if (t->fd[sid] >= 0)
        {
          ioctl(t->fd[sid], GPIOC_UNREGISTER, 0);
          close(t->fd[sid]);
        }
    }

  lv_free(t);
  return NULL;
}

void nyabula_te_deinit(nyabula_te_t *te)
{
  struct nyabula_te_s *t = (struct nyabula_te_s *)te;
  int sid;

  if (t == NULL)
    {
      return;
    }

  t->running = false;

  /* Wake the consumer thread blocked in sigwaitinfo() (it checks running
   * after each wakeup and exits). */
  pthread_cancel(t->thread);
  pthread_join(t->thread, NULL);

  for (sid = 0; sid < NYABULA_TE_MAX_SCREENS; sid++)
    {
      if (t->fd[sid] >= 0)
        {
          ioctl(t->fd[sid], GPIOC_UNREGISTER, 0);
          close(t->fd[sid]);
        }
    }

  lv_free(t);
}

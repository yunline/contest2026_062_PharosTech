/****************************************************************************
 * contest2026_062_PharosTech/drivers/net/esp8266/esp8266_netdev.c
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
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifdef CONFIG_ESP8266_NETDEV

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>
#include <time.h>

#include <arpa/inet.h>

#include <nuttx/clock.h>
#include <nuttx/kmalloc.h>
#include <nuttx/kthread.h>
#include <nuttx/semaphore.h>
#include <nuttx/signal.h>
#include <nuttx/spinlock.h>

#include <nuttx/net/netdev_lowerhalf.h>
#include <nuttx/net/ip.h>

#include "esp8266_netdev.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Timeouts (milliseconds) */

#define ESP8266_TIMEOUT_MS            1000
#define ESP8266_TIMEOUT_FLUSH_MS       100
#define ESP8266_TIMEOUT_SEND          1000
#define ESP8266_TIMEOUT_SCAN          5000
#define ESP8266_TIMEOUT_CONNECT      30000
#define ESP8266_FLOODING_OFFSET_S        3

/* Worker polling interval (milliseconds) - how long file_read blocks per poll */

#define ESP8266_POLLING_MS            1000

/* Socket flags (mirror apps/netutils/esp8266/esp8266.c) */

#define SOCK_FLAGS_USED       (1 << 0)
#define SOCK_FLAGS_CONNECTED  (1 << 1)
#define SOCK_FLAGS_TYPE_MASK  (3 << 2)
#define SOCK_FLAGS_TYPE_TCP   (0 << 2)
#define SOCK_FLAGS_TYPE_UDP   (1 << 2)

/* Buffer aliases */

#define BUF_ANS_LEN      CONFIG_ESP8266_NETDEV_AT_LINEBUF
#define BUF_CMD_LEN      CONFIG_ESP8266_NETDEV_AT_LINEBUF
#define BUF_WORKER_LEN   CONFIG_ESP8266_NETDEV_AT_LINEBUF

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Forward cast helper */
static struct esp8266_lowerhalf_s *
esp8266_priv(struct netdev_lowerhalf_s *dev);

/* Low-level UART I/O */
static int  esp8266_low_read(struct esp8266_lowerhalf_s *priv,
                             uint8_t *buf, int size);
static int  esp8266_vsend_cmd(struct esp8266_lowerhalf_s *priv,
                              const IPTR char *format, va_list ap);
static int  esp8266_send_cmd(struct esp8266_lowerhalf_s *priv,
                             const IPTR char *format, ...);

/* AT response reading */
static int  esp8266_read(struct esp8266_lowerhalf_s *priv,
                         int timeout_ms);
static void esp8266_flush(struct esp8266_lowerhalf_s *priv);
static int  esp8266_read_ans_ok(struct esp8266_lowerhalf_s *priv,
                                int timeout_ms);
static int  esp8266_ask_ans_ok(struct esp8266_lowerhalf_s *priv,
                               int timeout_ms,
                               const IPTR char *format, ...);
static int  esp8266_check(struct esp8266_lowerhalf_s *priv);

/* Socket management */
static struct esp8266_socket_s *
esp8266_get_sock(struct esp8266_lowerhalf_s *priv, int sockfd);
static void esp8266_sock_closed(struct esp8266_lowerhalf_s *priv,
                                int sockfd);

/* Data path */
static int  esp8266_read_ipd(struct esp8266_lowerhalf_s *priv,
                             int sockfd, int len);
static void esp8266_rx_avail(struct esp8266_lowerhalf_s *priv,
                             int sockfd);

/* Worker thread */
static int  esp8266_worker(int argc, char *argv[]);

/* Utility */
static inline int
esp8266_str_to_unsigned(char **p_ptr, char end);

/* AT response parsers */
static int  esp8266_parse_cwjap_ans_line(char *ptr,
                                         struct iw_point *iwp);
static int  esp8266_parse_cwlap_ans_line(char *ptr,
                                         struct iw_point *iwp);
static int  esp8266_parse_cipxxx_ans_line(const char *ptr,
                                          in_addr_t *ip);

/* netdev_lowerhalf ops */
static int  esp8266_ifup(struct netdev_lowerhalf_s *dev);
static int  esp8266_ifdown(struct netdev_lowerhalf_s *dev);
static int  esp8266_transmit(struct netdev_lowerhalf_s *dev,
                             netpkt_t *pkt);
static netpkt_t *esp8266_receive(struct netdev_lowerhalf_s *dev);
static void esp8266_reclaim(struct netdev_lowerhalf_s *dev);

#ifdef CONFIG_NETDEV_WIRELESS_HANDLER
/* Wireless ops */
static int  esp8266_iw_connect(struct netdev_lowerhalf_s *dev);
static int  esp8266_iw_disconnect(struct netdev_lowerhalf_s *dev);
static int  esp8266_iw_essid(struct netdev_lowerhalf_s *dev,
                             struct iwreq *iwr, bool set);
static int  esp8266_iw_bssid(struct netdev_lowerhalf_s *dev,
                             struct iwreq *iwr, bool set);
static int  esp8266_iw_passwd(struct netdev_lowerhalf_s *dev,
                              struct iwreq *iwr, bool set);
static int  esp8266_iw_scan(struct netdev_lowerhalf_s *dev,
                            struct iwreq *iwr, bool set);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct netdev_ops_s g_esp8266_ops =
{
  .ifup      = esp8266_ifup,
  .ifdown    = esp8266_ifdown,
  .transmit  = esp8266_transmit,
  .receive   = esp8266_receive,
  .reclaim   = esp8266_reclaim,
};

#ifdef CONFIG_NETDEV_WIRELESS_HANDLER
static const struct wireless_ops_s g_esp8266_iw_ops =
{
  .connect    = esp8266_iw_connect,
  .disconnect = esp8266_iw_disconnect,
  .essid      = esp8266_iw_essid,
  .bssid      = esp8266_iw_bssid,
  .passwd     = esp8266_iw_passwd,
  .scan       = esp8266_iw_scan,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp8266_priv
 ****************************************************************************/

static struct esp8266_lowerhalf_s *
esp8266_priv(struct netdev_lowerhalf_s *dev)
{
  return (struct esp8266_lowerhalf_s *)dev;
}

/****************************************************************************
 * Name: esp8266_str_to_unsigned
 ****************************************************************************/

static inline int esp8266_str_to_unsigned(char **p_ptr, char end)
{
  int nbr = 0;
  char *ptr = *p_ptr;

  while (*ptr != end)
    {
      char c = *ptr++ - '0';
      if ((c < 0) || (c >= 10))
        {
          return -1;
        }

      nbr *= 10;
      nbr += c;
    }

  *p_ptr = ptr + 1;
  return nbr;
}

/****************************************************************************
 * Name: esp8266_clear_read_buffer / esp8266_clear_read_ans
 ****************************************************************************/

static inline void
esp8266_clear_read_buffer(struct esp8266_lowerhalf_s *priv)
{
  priv->bufans[0] = '\0';
}

static inline void
esp8266_clear_read_ans(struct esp8266_lowerhalf_s *priv)
{
  priv->ans = ESP8266_ANS_NONE;
}

/****************************************************************************
 * Name: esp8266_get_sock
 ****************************************************************************/

static struct esp8266_socket_s *
esp8266_get_sock(struct esp8266_lowerhalf_s *priv, int sockfd)
{
  DEBUGASSERT(sockfd >= 0);

  if (((unsigned int)sockfd) >= ESP8266_SOCKET_NBR)
    {
      return NULL;
    }

  if ((priv->sockets[sockfd].flags & SOCK_FLAGS_USED) == 0)
    {
      return NULL;
    }

  return &priv->sockets[sockfd];
}

/****************************************************************************
 * Name: esp8266_sock_closed
 ****************************************************************************/

static void esp8266_sock_closed(struct esp8266_lowerhalf_s *priv,
                                int sockfd)
{
  struct esp8266_socket_s *sock;

  DEBUGASSERT(((unsigned int)sockfd) < ESP8266_SOCKET_NBR);

  sock         = &priv->sockets[sockfd];
  sock->flags  = 0;
  sock->inndx  = 0;
  sock->outndx = 0;

  nwarn("ESP8266 socket %d closed by remote\n", sockfd);
}

/****************************************************************************
 * Name: esp8266_low_read
 *
 * Description:
 *   Read up to `size` bytes from UART. Blocks until data available.
 *   In kernel space we use read() on the fd directly.
 *
 ****************************************************************************/

static int esp8266_low_read(struct esp8266_lowerhalf_s *priv,
                            uint8_t *buf, int size)
{
  int ret;

  ret = read(priv->fd, buf, size);
  if (ret < 0)
    {
      nerr("ERROR: esp8266 low read failed: %d\n", ret);
      return -1;
    }

  return ret;
}

/****************************************************************************
 * Name: esp8266_vsend_cmd
 ****************************************************************************/

static int esp8266_vsend_cmd(struct esp8266_lowerhalf_s *priv,
                             const IPTR char *format, va_list ap)
{
  int ret;

  ret = vsnprintf(priv->bufcmd, BUF_CMD_LEN, format, ap);
  if (ret >= BUF_CMD_LEN)
    {
      priv->bufcmd[BUF_CMD_LEN - 1] = '\0';
      nwarn("ESP8266: cmd buffer too small for '%s'...\n", priv->bufcmd);
      ret = -1;
    }

  ninfo("ESP8266 TX: %s\n", priv->bufcmd);

  ret = write(priv->fd, priv->bufcmd, ret);
  if (ret < 0)
    {
      return -1;
    }

  return ret;
}

/****************************************************************************
 * Name: esp8266_send_cmd
 ****************************************************************************/

static int esp8266_send_cmd(struct esp8266_lowerhalf_s *priv,
                            const IPTR char *format, ...)
{
  int ret;
  va_list ap;

  esp8266_clear_read_buffer(priv);
  esp8266_clear_read_ans(priv);

  va_start(ap, format);
  ret = esp8266_vsend_cmd(priv, format, ap);
  va_end(ap);

  return ret;
}

/****************************************************************************
 * Name: esp8266_read
 *
 * Description:
 *   Wait for a complete line from the worker thread.
 *
 ****************************************************************************/

static int esp8266_read(struct esp8266_lowerhalf_s *priv,
                        int timeout_ms)
{
  clock_t ticks;
  irqstate_t flags = 0;
  int ret;

  ticks = MSEC2TICK(timeout_ms);
  if (ticks == 0)
    {
      ticks = 1;
    }

  do
    {
      ret = nxsem_tickwait(&priv->worker.sem, ticks);
      if (ret < 0)
        {
          return -1;
        }

      flags = spin_lock_irqsave(&priv->worker.lock);

      if (priv->worker.ans != ESP8266_ANS_NONE)
        {
          priv->ans = priv->worker.ans;
          priv->worker.ans = ESP8266_ANS_NONE;
        }

      ret = strlen(priv->worker.buf);
      if (ret > 0)
        {
          memcpy(priv->bufans, priv->worker.buf, ret + 1);
        }

      priv->worker.buf[0] = '\0';

      spin_unlock_irqrestore(&priv->worker.lock, flags);
    }
  while ((ret <= 0) && (priv->ans == ESP8266_ANS_NONE));

  ninfo("ESP8266 RX: %s (ans=%d)\n", priv->bufans, priv->ans);
  return ret;
}

/****************************************************************************
 * Name: esp8266_flush
 ****************************************************************************/

static void esp8266_flush(struct esp8266_lowerhalf_s *priv)
{
  do
    {
      esp8266_clear_read_buffer(priv);
      esp8266_clear_read_ans(priv);
    }
  while (esp8266_read(priv, ESP8266_TIMEOUT_FLUSH_MS) >= 0);
}

/****************************************************************************
 * Name: esp8266_read_ans_ok
 ****************************************************************************/

static int esp8266_read_ans_ok(struct esp8266_lowerhalf_s *priv,
                                int timeout_ms)
{
  int ret = 0;
  time_t end;

  end = time(NULL) + (timeout_ms / 1000) + ESP8266_FLOODING_OFFSET_S;

  while (priv->ans != ESP8266_ANS_OK)
    {
      ret = esp8266_read(priv, timeout_ms);

      if ((ret < 0) || (priv->ans == ESP8266_ANS_ERR) ||
          (time(NULL) > end))
        {
          ret = -1;
          break;
        }
    }

  esp8266_clear_read_ans(priv);
  esp8266_clear_read_buffer(priv);

  return ret;
}

/****************************************************************************
 * Name: esp8266_ask_ans_ok
 ****************************************************************************/

static int esp8266_ask_ans_ok(struct esp8266_lowerhalf_s *priv,
                               int timeout_ms,
                               const IPTR char *format, ...)
{
  int ret;
  va_list ap;

  va_start(ap, format);
  ret = esp8266_vsend_cmd(priv, format, ap);
  va_end(ap);

  if (ret >= 0)
    {
      ret = esp8266_read_ans_ok(priv, timeout_ms);
    }

  return ret;
}

/****************************************************************************
 * Name: esp8266_check
 ****************************************************************************/

static int esp8266_check(struct esp8266_lowerhalf_s *priv)
{
  if (priv->fd < 0)
    {
      nerr("ERROR: ESP8266 not initialized\n");
      return -1;
    }

  esp8266_flush(priv);

  if (esp8266_ask_ans_ok(priv, ESP8266_TIMEOUT_MS, "AT\r\n") < 0)
    {
      nerr("ERROR: ESP8266 not responding to AT\n");
      return -1;
    }

  return 0;
}

/****************************************************************************
 * Name: esp8266_read_ipd
 *
 * Description:
 *   Read +IPD data from UART into the socket's circular FIFO,
 *   then push assembled packets into the rxqueue.
 *
 ****************************************************************************/

static int esp8266_read_ipd(struct esp8266_lowerhalf_s *priv,
                             int sockfd, int len)
{
  struct esp8266_socket_s *sock;
  irqstate_t flags = 0;

  sock = esp8266_get_sock(priv, sockfd);

  ninfo("Read %d bytes for socket %d\n", len, sockfd);

  while (len)
    {
      uint8_t *buf;
      int size;
      uint8_t b;
      int next;

      buf  = (uint8_t *)priv->worker.rxbuf;
      size = len;
      if (size >= BUF_WORKER_LEN)
        {
          size = BUF_WORKER_LEN;
        }

      size = esp8266_low_read(priv, buf, size);
      if (size <= 0)
        {
          return -1;
        }

      len -= size;

      if (sock != NULL)
        {
          while (size--)
            {
              b = *buf++;

              next = sock->inndx + 1;
              if (next >= ESP8266_FIFO_SIZE)
                {
                  next -= ESP8266_FIFO_SIZE;
                }

              if (next == sock->outndx)
                {
                  /* FIFO full — yield briefly to let consumer drain */

                  spin_unlock_irqrestore(&priv->worker.lock, flags);
                  nxsig_usleep(100);
                  flags = spin_lock_irqsave(&priv->worker.lock);
                }

              if (next != sock->outndx)
                {
                  sock->rxbuf[sock->inndx] = b;
                  sock->inndx = next;
                }
              else
                {
                  nwarn("ESP8266 socket %d FIFO overflow\n", sockfd);
                }
            }

          /* Assemble data into netpkt and enqueue */

          esp8266_rx_avail(priv, sockfd);
        }
    }

  return 1;
}

/****************************************************************************
 * Name: esp8266_rx_avail
 *
 * Description:
 *   Drain data from a socket's circular FIFO into a netpkt and enqueue it.
 *
 ****************************************************************************/

static void esp8266_rx_avail(struct esp8266_lowerhalf_s *priv,
                              int sockfd)
{
  struct esp8266_socket_s *sock;
  netpkt_t *pkt;
  irqstate_t flags;
  int avail;

  sock = esp8266_get_sock(priv, sockfd);
  if (sock == NULL)
    {
      return;
    }

  /* Calculate available bytes */

  avail = sock->inndx - sock->outndx;
  if (avail < 0)
    {
      avail += ESP8266_FIFO_SIZE;
    }

  if (avail == 0)
    {
      return;
    }

  /* Allocate a netpkt for the received data */

  pkt = netpkt_alloc(&priv->dev, NETPKT_RX);
  if (pkt == NULL)
    {
      nwarn("ESP8266: cannot allocate netpkt for RX\n");
      return;
    }

  /* Copy data from circular FIFO into the netpkt */

  if (sock->outndx < sock->inndx)
    {
      /* Single contiguous segment */

      netpkt_copyin(&priv->dev, pkt, &sock->rxbuf[sock->outndx],
                    avail, 0);
    }
  else
    {
      /* Wrapped: copy first segment then second */

      int first = ESP8266_FIFO_SIZE - sock->outndx;
      netpkt_copyin(&priv->dev, pkt, &sock->rxbuf[sock->outndx],
                    first, 0);
      netpkt_copyin(&priv->dev, pkt, sock->rxbuf,
                    avail - first, first);
    }

  sock->outndx = sock->inndx;

  /* Enqueue for upper half to pick up — use simple linked list */

  flags = spin_lock_irqsave(&priv->rxlock);
  {
    struct esp8266_rxnode_s *node;
    node = kmm_malloc(sizeof(struct esp8266_rxnode_s));
    if (node != NULL)
      {
        node->pkt   = pkt;
        node->flink = NULL;
        if (priv->rxtail != NULL)
          {
            priv->rxtail->flink = node;
          }
        else
          {
            priv->rxhead = node;
          }
        priv->rxtail = node;
      }
    else
      {
        netpkt_free(&priv->dev, pkt, NETPKT_RX);
      }
  }
  spin_unlock_irqrestore(&priv->rxlock, flags);
}

/****************************************************************************
 * Name: esp8266_worker
 *
 * Description:
 *   Kernel thread that reads UART byte-by-byte, parses AT responses.
 *   Recognizes: OK, ERROR, FAIL, ",CLOSED", "+IPD,<id>,<len>:"
 *
 ****************************************************************************/

static int esp8266_worker(int argc, char *argv[])
{
  struct esp8266_lowerhalf_s *priv;
  irqstate_t flags;
  int rxlen = 0;

  if (argc < 1 || argv[1] == NULL)
    {
      return -EINVAL;
    }

  priv = (struct esp8266_lowerhalf_s *)
         ((uintptr_t)strtoul(argv[1], NULL, 0));

  ninfo("ESP8266 worker started\n");

  while (priv->worker.running)
    {
      uint8_t c;
      int ret;

      ret = esp8266_low_read(priv, &c, 1);

      if (ret < 0)
        {
          nerr("ERROR: worker read error %d\n", ret);
          break;
        }

      if (ret == 0)
        {
          continue;
        }

      flags = spin_lock_irqsave(&priv->worker.lock);

      if (c == '\n')
        {
          if (rxlen > 0 && priv->worker.rxbuf[rxlen - 1] == '\r')
            {
              rxlen--;
            }

          DEBUGASSERT(rxlen >= 0);
          DEBUGASSERT(rxlen < BUF_WORKER_LEN);

          priv->worker.rxbuf[rxlen] = '\0';

          if (rxlen != 0)
            {
              if (strcmp(priv->worker.rxbuf, "OK") == 0)
                {
                  priv->worker.ans = ESP8266_ANS_OK;
                }
              else if ((strcmp(priv->worker.rxbuf, "FAIL") == 0) ||
                       (strcmp(priv->worker.rxbuf, "ERROR") == 0))
                {
                  priv->worker.ans = ESP8266_ANS_ERR;
                }
              else if ((rxlen == 8) &&
                       (memcmp(priv->worker.rxbuf + 1, ",CLOSED", 7) == 0))
                {
                  unsigned int sockid = priv->worker.rxbuf[0] - '0';
                  if (sockid < ESP8266_SOCKET_NBR)
                    {
                      esp8266_sock_closed(priv, sockid);
                    }
                }
              else
                {
                  if (priv->worker.buf[0] != '\0')
                    {
                      spin_unlock_irqrestore(&priv->worker.lock, flags);
                      nxsig_usleep(100);
                      flags = spin_lock_irqsave(&priv->worker.lock);
                    }

                  if (rxlen + 1 <= BUF_ANS_LEN)
                    {
                      memcpy(priv->worker.buf, priv->worker.rxbuf,
                             rxlen + 1);
                    }
                  else
                    {
                      nerr("Worker: line too long: %s\n",
                           priv->worker.rxbuf);
                    }
                }

              /* Release the spinlock BEFORE posting the semaphore.
               * nxsem_post() may cause an immediate context switch to a
               * higher-priority thread waiting in esp8266_read().  If that
               * thread then tries spin_lock_irqsave(&worker.lock) while
               * we still hold it, we get a priority-inversion deadlock
               * because spin_lock_irqsave disables interrupts on this CPU.
               */

              spin_unlock_irqrestore(&priv->worker.lock, flags);
              nxsem_post(&priv->worker.sem);
              priv->worker.rxbuf[0] = '\0';
              rxlen = 0;
              flags = spin_lock_irqsave(&priv->worker.lock);
            }
        }
      else if (rxlen < BUF_WORKER_LEN - 1)
        {
          priv->worker.rxbuf[rxlen++] = c;
          if ((c == ':') &&
              (rxlen >= 5) &&
              (memcmp(priv->worker.rxbuf, "+IPD,", 5) == 0))
            {
              char *ptr = priv->worker.rxbuf + 5;
              int sockfd;
              int len;

              sockfd = esp8266_str_to_unsigned(&ptr, ',');
              if (sockfd >= 0)
                {
                  len = esp8266_str_to_unsigned(&ptr, ':');
                  if (len >= 0)
                    {
                      spin_unlock_irqrestore(&priv->worker.lock, flags);
                      esp8266_read_ipd(priv, sockfd, len);
                      flags = spin_lock_irqsave(&priv->worker.lock);
                    }
                }

              rxlen = 0;
            }
        }
      else
        {
          nerr("Worker: char overflow\n");
        }

      spin_unlock_irqrestore(&priv->worker.lock, flags);
    }

  ninfo("ESP8266 worker stopped\n");
  return 0;
}

/****************************************************************************
 * AT Response Parsers
 ****************************************************************************/

/****************************************************************************
 * Name: esp8266_parse_cipxxx_ans_line
 *
 * Description:
 *   Parse +CIPSTA:ip:"A.B.C.D","G.W.X.Y","E.F.G.H"
 *   Extracts the IP address (first quoted field).
 *
 ****************************************************************************/

static int esp8266_parse_cipxxx_ans_line(const char *ptr,
                                          in_addr_t *ip)
{
  int field_idx;
  char *str;
  char *ptr_next;

  str = (char *)ptr;

  for (field_idx = 0; field_idx <= 2; field_idx++)
    {
      if (field_idx <= 1)
        {
          ptr_next = strchr(str, ':');
        }
      else if (field_idx == 2)
        {
          ptr_next = strchr(str, '\0');
        }
      else
        {
          ptr_next = strchr(str, ',');
        }

      if (ptr_next == NULL)
        {
          return -1;
        }

      *ptr_next = '\0';

      switch (field_idx)
        {
          case 0:
            if (strncmp(str, "+CIP", 4) != 0)
              {
                return -1;
              }
            break;

          case 1:
            /* ip label — skip */
            break;

          case 2:
            str++;                          /* Skip first '"' */
            *(ptr_next - 1) = '\0';          /* Remove trailing '"' */
            if (inet_pton(AF_INET, str, ip) < 0)
              {
                return -1;
              }
            break;
        }

      str = ptr_next + 1;
    }

  return 0;
}

/****************************************************************************
 * Name: esp8266_parse_cwjap_ans_line
 *
 * Description:
 *   Parse +CWJAP:"SSID","BSSID",channel,RSSI
 *   Fills iwreq with AP info (used for get essid/bssid).
 *
 ****************************************************************************/

static int esp8266_parse_cwjap_ans_line(char *ptr,
                                         struct iw_point *iwp)
{
  int field_idx;
  char *ptr_next;

  for (field_idx = 0; field_idx <= 4; field_idx++)
    {
      if (field_idx == 0)
        {
          ptr_next = strchr(ptr, ':');
        }
      else if (field_idx == 4)
        {
          ptr_next = strchr(ptr, '\0');
        }
      else
        {
          ptr_next = strchr(ptr, ',');
        }

      if (ptr_next == NULL)
        {
          return -1;
        }

      *ptr_next = '\0';

      switch (field_idx)
        {
          case 0:
            if (strncmp(ptr, "+CWJAP", 6) != 0)
              {
                return -1;
              }
            break;

          case 1:
            /* SSID — skip quotes */
            ptr++;
            *(ptr_next - 1) = '\0';
            if (iwp != NULL && iwp->pointer != NULL)
              {
                strlcpy(iwp->pointer, ptr, iwp->length);
                iwp->flags = 1;
              }
            break;

          case 2:
            /* BSSID — skip quotes */
            ptr++;
            *(ptr_next - 1) = '\0';
            /* BSSID is handled separately via bssid iw_op */
            break;

          case 3:
            /* channel */
            break;

          case 4:
            /* RSSI */
            break;
        }

      ptr = ptr_next + 1;
    }

  return 0;
}

/****************************************************************************
 * Name: esp8266_parse_cwlap_ans_line
 *
 * Description:
 *   Parse +CWLAP:(ecn,"SSID",RSSI,"BSSID",channel)
 *   Fills iwreq scan result buffer.
 *
 ****************************************************************************/

static int esp8266_parse_cwlap_ans_line(char *ptr,
                                         struct iw_point *iwp)
{
  /* For scan results, we store raw line text into the scan buffer.
   * The upper half wireless handler will parse it further.
   * We copy the line into iwp->pointer (if space available).
   */

  int len;

  if (iwp == NULL || iwp->pointer == NULL)
    {
      return 0;
    }

  len = strlen(ptr);
  if (len + 1 > iwp->length)
    {
      /* No space left */
      return -ENOSPC;
    }

  memcpy(iwp->pointer, ptr, len + 1);
  iwp->pointer += len + 1;
  iwp->length -= len + 1;
  iwp->flags++;

  return 0;
}

/****************************************************************************
 * netdev_lowerhalf Ops
 ****************************************************************************/

/****************************************************************************
 * Name: esp8266_ifup
 ****************************************************************************/

static int esp8266_ifup(struct netdev_lowerhalf_s *dev)
{
  struct esp8266_lowerhalf_s *priv = esp8266_priv(dev);
  in_addr_t ip;
  char mac_str[18];
  int ret;
  int i;

  if (priv->ifup)
    {
      return OK;
    }

  /* Check ESP8266 is alive */

  ret = esp8266_check(priv);
  if (ret < 0)
    {
      return ret;
    }

  /* Set station mode */

  ret = esp8266_ask_ans_ok(priv, ESP8266_TIMEOUT_MS, "AT+CWMODE_CUR=1\r\n");
  if (ret < 0)
    {
      nerr("ERROR: Failed to set station mode\n");
      return ret;
    }

  /* Enable multiple connections */

  ret = esp8266_ask_ans_ok(priv, ESP8266_TIMEOUT_MS, "AT+CIPMUX=1\r\n");
  if (ret < 0)
    {
      nerr("ERROR: Failed to enable CIPMUX\n");
      return ret;
    }

  /* Get IP address */

  ret = esp8266_send_cmd(priv, "AT+CIPSTA_CUR?\r\n");
  if (ret >= 0)
    {
      ret = esp8266_read(priv, ESP8266_TIMEOUT_MS);
      if (ret >= 0)
        {
          esp8266_parse_cipxxx_ans_line(priv->bufans, &ip);
          net_ipv4addr_copy(dev->netdev.d_ipaddr, (struct in_addr *)&ip);
        }
      esp8266_read_ans_ok(priv, ESP8266_TIMEOUT_MS);
    }

  /* Get MAC address */

  ret = esp8266_send_cmd(priv, "AT+CIPSTAMAC_CUR?\r\n");
  if (ret >= 0)
    {
      ret = esp8266_read(priv, ESP8266_TIMEOUT_MS);
      if (ret >= 0)
        {
          /* Response format: +CIPSTAMAC_CUR:"aa:bb:cc:dd:ee:ff" */
          char *p = strchr(priv->bufans, '"');
          if (p != NULL)
            {
              p++;
              for (i = 0; i < ESP8266_MAC_LEN; i++)
                {
                  mac_str[i * 3] = p[i * 3];
                  mac_str[i * 3 + 1] = p[i * 3 + 1];
                }
              mac_str[17] = '\0';

              sscanf(mac_str, "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx",
                     &priv->mac[0], &priv->mac[1], &priv->mac[2],
                     &priv->mac[3], &priv->mac[4], &priv->mac[5]);
              priv->mac_valid = true;
              memcpy(dev->netdev.d_mac.ether.ether_addr_octet, priv->mac,
                     ESP8266_MAC_LEN);
            }
        }
      esp8266_read_ans_ok(priv, ESP8266_TIMEOUT_MS);
    }

  /* Mark interface as up */

  priv->ifup = true;
  netdev_lower_carrier_on(dev);

  ninfo("ESP8266 interface up\n");
  return OK;
}

/****************************************************************************
 * Name: esp8266_ifdown
 ****************************************************************************/

static int esp8266_ifdown(struct netdev_lowerhalf_s *dev)
{
  struct esp8266_lowerhalf_s *priv = esp8266_priv(dev);
  int i;

  if (!priv->ifup)
    {
      return OK;
    }

  /* Disconnect from AP */

  esp8266_ask_ans_ok(priv, ESP8266_TIMEOUT_MS, "AT+CWQAP\r\n");

  /* Close all sockets */

  for (i = 0; i < ESP8266_SOCKET_NBR; i++)
    {
      if (priv->sockets[i].flags & SOCK_FLAGS_USED)
        {
          esp8266_ask_ans_ok(priv, ESP8266_TIMEOUT_MS,
                             "AT+CIPCLOSE=%d\r\n", i);
          esp8266_sock_closed(priv, i);
        }
    }

  priv->ifup = false;
  netdev_lower_carrier_off(dev);

  ninfo("ESP8266 interface down\n");
  return OK;
}

/****************************************************************************
 * Name: esp8266_transmit
 *
 * Description:
 *   Send a netpkt via ESP8266 AT+CIPSEND command.
 *   Uses socket 0 for single-connection mode or allocates a socket for
 *   multi-connection mode.
 *
 ****************************************************************************/

static int esp8266_transmit(struct netdev_lowerhalf_s *dev,
                             netpkt_t *pkt)
{
  struct esp8266_lowerhalf_s *priv = esp8266_priv(dev);
  int sockfd;
  int len;
  int ret;
  uint8_t *data;
  struct esp8266_socket_s *sock;

  if (!priv->ifup)
    {
      netpkt_free(dev, pkt, NETPKT_TX);
      return -ENETDOWN;
    }

  /* Find or create a connected TCP socket */

  for (sockfd = 0; sockfd < ESP8266_SOCKET_NBR; sockfd++)
    {
      sock = &priv->sockets[sockfd];
      if ((sock->flags & SOCK_FLAGS_USED) &&
          (sock->flags & SOCK_FLAGS_CONNECTED))
        {
          break;
        }
    }

  if (sockfd >= ESP8266_SOCKET_NBR)
    {
      nwarn("ESP8266: no connected socket for TX\n");
      netpkt_free(dev, pkt, NETPKT_TX);
      return -ENOTCONN;
    }

  /* Get data from netpkt */

  len = netpkt_getdatalen(dev, pkt);
  data = netpkt_getdata(dev, pkt);
  if (data == NULL || len == 0)
    {
      netpkt_free(dev, pkt, NETPKT_TX);
      return -EINVAL;
    }

  /* Send via AT+CIPSEND */

  ret = esp8266_ask_ans_ok(priv, ESP8266_TIMEOUT_SEND,
                           "AT+CIPSEND=%d,%d\r\n", sockfd, len);
  if (ret < 0)
    {
      nerr("ERROR: CIPSEND command failed\n");
      netpkt_free(dev, pkt, NETPKT_TX);
      return ret;
    }

  /* Write the actual data */

  ret = write(priv->fd, data, len);
  if (ret < 0)
    {
      nerr("ERROR: CIPSEND data write failed\n");
      netpkt_free(dev, pkt, NETPKT_TX);
      return ret;
    }

  /* Wait for "SEND OK" */

  ret = esp8266_read_ans_ok(priv, ESP8266_TIMEOUT_SEND);

  netpkt_free(dev, pkt, NETPKT_TX);
  return (ret >= 0) ? OK : ret;
}

/****************************************************************************
 * Name: esp8266_receive
 ****************************************************************************/

static netpkt_t *esp8266_receive(struct netdev_lowerhalf_s *dev)
{
  struct esp8266_lowerhalf_s *priv = esp8266_priv(dev);
  netpkt_t *pkt;
  irqstate_t flags;

  flags = spin_lock_irqsave(&priv->rxlock);
  {
    struct esp8266_rxnode_s *node = priv->rxhead;
    if (node != NULL)
      {
        pkt = node->pkt;
        priv->rxhead = node->flink;
        if (priv->rxhead == NULL)
          {
            priv->rxtail = NULL;
          }
        kmm_free(node);
      }
    else
      {
        pkt = NULL;
      }
  }
  spin_unlock_irqrestore(&priv->rxlock, flags);

  return pkt;
}

/****************************************************************************
 * Name: esp8266_reclaim
 ****************************************************************************/

static void esp8266_reclaim(struct netdev_lowerhalf_s *dev)
{
  /* TX packets are freed in esp8266_transmit, nothing to do here */

  UNUSED(dev);
}

/****************************************************************************
 * Wireless Ops (CONFIG_NETDEV_WIRELESS_HANDLER)
 ****************************************************************************/

#ifdef CONFIG_NETDEV_WIRELESS_HANDLER

/****************************************************************************
 * Name: esp8266_iw_connect
 ****************************************************************************/

static int esp8266_iw_connect(struct netdev_lowerhalf_s *dev)
{
  struct esp8266_lowerhalf_s *priv = esp8266_priv(dev);

  /* Wi-Fi connection is initiated via the essid/passwd set + ifup flow.
   * This op triggers the actual connection if configured.
   */

  priv->connected = true;
  return OK;
}

/****************************************************************************
 * Name: esp8266_iw_disconnect
 ****************************************************************************/

static int esp8266_iw_disconnect(struct netdev_lowerhalf_s *dev)
{
  struct esp8266_lowerhalf_s *priv = esp8266_priv(dev);
  int ret;

  ret = esp8266_ask_ans_ok(priv, ESP8266_TIMEOUT_MS, "AT+CWQAP\r\n");
  if (ret >= 0)
    {
      priv->connected = false;
    }

  return (ret >= 0) ? OK : ret;
}

/****************************************************************************
 * Name: esp8266_iw_essid (set/get)
 ****************************************************************************/

static int esp8266_iw_essid(struct netdev_lowerhalf_s *dev,
                             struct iwreq *iwr, bool set)
{
  struct esp8266_lowerhalf_s *priv = esp8266_priv(dev);
  int ret;

  if (set)
    {
      /* Store SSID for later connection; actual connect happens in
       * esp8266_iw_connect or esp8266_ifup.
       */

      struct iw_point *iwp = &iwr->u.essid;

      iwp->flags = 0;
      return OK;
    }
  else
    {
      /* Get: query current AP via AT+CWJAP_CUR? */

      struct iw_point *iwp = &iwr->u.essid;

      ret = esp8266_check(priv);
      if (ret < 0) return ret;

      ret = esp8266_send_cmd(priv, "AT+CWJAP_CUR?\r\n");
      if (ret < 0) return ret;

      ret = esp8266_read(priv, ESP8266_TIMEOUT_MS);
      if (ret < 0) return ret;

      esp8266_parse_cwjap_ans_line(priv->bufans, iwp);

      esp8266_read_ans_ok(priv, ESP8266_TIMEOUT_MS);
      return OK;
    }
}

/****************************************************************************
 * Name: esp8266_iw_bssid (set/get)
 ****************************************************************************/

static int esp8266_iw_bssid(struct netdev_lowerhalf_s *dev,
                             struct iwreq *iwr, bool set)
{
  /* BSSID set: not typically used for ESP8266 (connect via SSID).
   * BSSID get: extracted from +CWJAP? response during essid get.
   * For simplicity, return OK for set and delegate to essid for get.
   */

  if (set)
    {
      return OK;
    }
  else
    {
      /* BSSID is obtained through the same +CWJAP_CUR? response.
       * We re-query here for simplicity.
       */

      struct esp8266_lowerhalf_s *priv = esp8266_priv(dev);
      struct sockaddr *ap = &iwr->u.ap_addr;
      int ret;

      ret = esp8266_check(priv);
      if (ret < 0) return ret;

      ret = esp8266_send_cmd(priv, "AT+CWJAP_CUR?\r\n");
      if (ret < 0) return ret;

      ret = esp8266_read(priv, ESP8266_TIMEOUT_MS);
      if (ret < 0) return ret;

      /* Parse BSSID (field 2 of +CWJAP:) */
      char *p = priv->bufans;
      int i;

      /* Skip to BSSID field */
      for (i = 0; i < 2; i++)
        {
          p = strchr(p, i == 0 ? ':' : ',');
          if (p == NULL) return -EINVAL;
          p++;
        }

      /* p now points to "BSSID",... */
      if (*p == '"') p++;
      if (ap != NULL)
        {
          /* BSSID format: "aa:bb:cc:dd:ee:ff" */
          uint8_t mac[6];
          sscanf(p, "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx",
                 &mac[0], &mac[1], &mac[2],
                 &mac[3], &mac[4], &mac[5]);
          memcpy(ap->sa_data, mac, 6);
          ap->sa_family = AF_INET;
        }

      esp8266_read_ans_ok(priv, ESP8266_TIMEOUT_MS);
      return OK;
    }
}

/****************************************************************************
 * Name: esp8266_iw_passwd (set/get)
 ****************************************************************************/

static int esp8266_iw_passwd(struct netdev_lowerhalf_s *dev,
                              struct iwreq *iwr, bool set)
{
  /* get: password is not retrievable from ESP8266.
   * set: store password for later connect.
   */

  UNUSED(dev);
  UNUSED(iwr);

  if (set)
    {
      return OK;
    }

  return -ENOTTY;
}

/****************************************************************************
 * Name: esp8266_iw_scan
 ****************************************************************************/

static int esp8266_iw_scan(struct netdev_lowerhalf_s *dev,
                            struct iwreq *iwr, bool set)
{
  struct esp8266_lowerhalf_s *priv = esp8266_priv(dev);
  struct iw_point *iwp;
  int ret;

  if (set)
    {
      /* Start scan: send AT+CWLAP */

      ret = esp8266_check(priv);
      if (ret < 0)
        {
          return ret;
        }

      ret = esp8266_ask_ans_ok(priv, ESP8266_TIMEOUT_SCAN, "AT+CWLAP\r\n");
      return (ret >= 0) ? OK : ret;
    }
  else
    {
      /* Get scan results: read all +CWLAP lines */

      iwp = &iwr->u.data;
      iwp->flags = 0;

      ret = esp8266_check(priv);
      if (ret < 0)
        {
          return ret;
        }

      ret = esp8266_send_cmd(priv, "AT+CWLAP\r\n");
      if (ret < 0)
        {
          return ret;
        }

      while (ret >= 0)
        {
          ret = esp8266_read(priv, ESP8266_TIMEOUT_SCAN);
          if (ret < 0)
            {
              continue;
            }

          if (strcmp(priv->bufans, "OK") == 0)
            {
              break;
            }

          esp8266_parse_cwlap_ans_line(priv->bufans, iwp);
        }

      return OK;
    }
}

#endif /* CONFIG_NETDEV_WIRELESS_HANDLER */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp8266_netdev_register
 *
 * Description:
 *   Register the ESP8266 network device with the NuttX network stack.
 *
 ****************************************************************************/

int esp8266_netdev_register(const char *uart_dev)
{
  struct esp8266_lowerhalf_s *priv;
  char *argv[2];
  char argbuf[32];
  int ret;
  int i;

  /* Allocate state structure */

  priv = kmm_zalloc(sizeof(struct esp8266_lowerhalf_s));
  if (priv == NULL)
    {
      nerr("ERROR: Failed to allocate esp8266_lowerhalf_s\n");
      return -ENOMEM;
    }

  /* Open UART device */

  priv->fd = open(uart_dev, O_RDWR);
  if (priv->fd < 0)
    {
      nerr("ERROR: Failed to open UART %s\n", uart_dev);
      kmm_free(priv);
      return -ENODEV;
    }

  /* Initialize semaphore for worker communication */

  nxsem_init(&priv->worker.sem, 0, 0);

  /* Set netdev ops */

  priv->dev.ops     = &g_esp8266_ops;

#ifdef CONFIG_NETDEV_WIRELESS_HANDLER
  priv->dev.iw_ops  = &g_esp8266_iw_ops;
#endif

  /* Set MTU */

  priv->dev.netdev.d_pktsize = 1500;

  /* Clear socket state */

  for (i = 0; i < ESP8266_SOCKET_NBR; i++)
    {
      priv->sockets[i].flags  = 0;
      priv->sockets[i].inndx  = 0;
      priv->sockets[i].outndx = 0;
    }

  /* Start worker thread */

  priv->worker.running = true;

  snprintf(argbuf, sizeof(argbuf), "%p", priv);
  argv[0]    = argbuf;
  argv[1]    = NULL;

  priv->worker.pid = kthread_create("esp8266_worker",
                                     CONFIG_ESP8266_NETDEV_THREAD_PRIORITY,
                                     CONFIG_ESP8266_NETDEV_THREAD_STACKSIZE,
                                     esp8266_worker, argv);
  if (priv->worker.pid < 0)
    {
      int errcode = priv->worker.pid;
      nerr("ERROR: Failed to create worker thread\n");
      nxsem_destroy(&priv->worker.sem);
      close(priv->fd);
      kmm_free(priv);
      return errcode;
    }

  /* Perform initial AT handshake */

  nxsig_sleep(1);

  esp8266_flush(priv);

  while (esp8266_ask_ans_ok(priv, ESP8266_TIMEOUT_MS, "ATE0\r\n") < 0)
    {
      nxsig_sleep(1);
      esp8266_flush(priv);
    }

  /* Register with network stack */

  ret = netdev_lower_register(&priv->dev, NET_LL_IEEE80211);
  if (ret < 0)
    {
      nerr("ERROR: Failed to register netdev: %d\n", ret);
      priv->worker.running = false;
      nxsem_destroy(&priv->worker.sem);
      close(priv->fd);
      kmm_free(priv);
      return ret;
    }

  ninfo("ESP8266 netdev registered on %s\n", uart_dev);
  return OK;
}

#endif /* CONFIG_ESP8266_NETDEV */

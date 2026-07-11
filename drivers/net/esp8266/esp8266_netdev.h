/****************************************************************************
 * contest2026_062_PharosTech/drivers/net/esp8266/esp8266_netdev.h
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

#ifndef __DRIVERS_NET_ESP8266_ESP8266_NETDEV_H
#define __DRIVERS_NET_ESP8266_ESP8266_NETDEV_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/net/netdev_lowerhalf.h>
#include <nuttx/wireless/wireless.h>

#ifdef CONFIG_ESP8266_NETDEV

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ESP8266_SOCKET_NBR   4
#define ESP8266_FIFO_SIZE    2048     /* Must be a power of 2 */

#define ESP8266_MAC_LEN      6

/* ESP8266 AT response codes */

#define ESP8266_ANS_NONE     0
#define ESP8266_ANS_OK       1
#define ESP8266_ANS_ERR      (-1)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Simple singly-linked list node for pending RX packets.
 * This avoids dependency on CONFIG_IOB_NCHAINS.
 */

struct esp8266_rxnode_s
{
  struct esp8266_rxnode_s *flink;
  netpkt_t                *pkt;
};

/* Socket for data communication (0-4, 0 is for single connection) */

struct esp8266_socket_s
{
  uint8_t   flags;
  uint16_t  inndx;
  uint16_t  outndx;
  uint8_t   rxbuf[ESP8266_FIFO_SIZE];
};

/* Worker thread state for parsing AT responses */

struct esp8266_worker_s
{
  bool      running;
  pid_t     pid;

  char      rxbuf[CONFIG_ESP8266_NETDEV_AT_LINEBUF];

  sem_t     sem;  /* Inform that something is received */
  char      buf[CONFIG_ESP8266_NETDEV_AT_LINEBUF]; /* Last complete line */
  int8_t    ans;  /* Last answer received (OK/ERROR/FAIL) */

  spinlock_t lock;
};

/* ESP8266 lower-half netdev state structure */

struct esp8266_lowerhalf_s
{
  /* Must be first so we can cast to netdev_lowerhalf_s */

  struct netdev_lowerhalf_s dev;

  /* UART communication */

  int                fd;

  /* Worker thread for AT parsing */

  struct esp8266_worker_s worker;

  /* Data sockets */

  struct esp8266_socket_s sockets[ESP8266_SOCKET_NBR];

  /* AT command / response buffers */

  int8_t             ans;
  char               bufans[CONFIG_ESP8266_NETDEV_AT_LINEBUF];
  char               bufcmd[CONFIG_ESP8266_NETDEV_AT_LINEBUF];

  /* RX queue for packets received from ESP8266 */

  struct esp8266_rxnode_s *rxhead;
  struct esp8266_rxnode_s *rxtail;
  spinlock_t         rxlock;

  /* MAC address learned from ESP8266 */

  uint8_t            mac[ESP8266_MAC_LEN];
  bool               mac_valid;

  /* Connection state */

  bool               connected;  /* Wi-Fi associated */
  bool               ifup;

  /* AP scan callback state */

  struct iwreq  *scan_iwr;
  sem_t              scan_sem;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: esp8266_netdev_register
 *
 * Description:
 *   Register the ESP8266 network device with the NuttX network stack.
 *
 * Input Parameters:
 *   uart_dev - Path to the UART device (e.g. "/dev/ttyS1")
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure.
 *
 ****************************************************************************/

int esp8266_netdev_register(const char *uart_dev);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_ESP8266_NETDEV */
#endif /* __DRIVERS_NET_ESP8266_ESP8266_NETDEV_H */

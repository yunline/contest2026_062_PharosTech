# ##############################################################################
# contest2026_062_PharosTech/drivers/net/esp8266/CMakeLists.txt
#
# NOTE: This file is not directly used by the build system. The esp8266
# netdev driver is compiled as part of the board target via:
#   vendor/rockchip/boards/rk3576/kickpi-k7/src/CMakeLists.txt
#
# This file is kept for reference and potential standalone builds.
#
# SPDX-License-Identifier: Apache-2.0
# ##############################################################################

if(CONFIG_ESP8266_NETDEV)
  target_sources(drivers PRIVATE esp8266_netdev.c)
  target_include_directories(drivers PRIVATE ${CMAKE_CURRENT_LIST_DIR})
  target_include_directories(board PRIVATE ${CMAKE_CURRENT_LIST_DIR})
endif()

# ##############################################################################
# boards/rk3576/drivers/drivers/st77916/CMakeLists.txt
#
# SPDX-License-Identifier: Apache-2.0
#
# ST77916 LCD driver — kernel-side lower-half driver.
#
# Compiled into libdrivers.a (the `drivers` kernel library).  In CMake mode the
# upstream `drivers/platform` source-compilation channel is not implemented
# (nuttx/CMakeLists.txt only symlinks board drivers into
# $BINDIR/drivers/platform for Kconfig; nuttx/drivers' nuttx_add_subdirectory()
# globs the *source tree* and never reaches that build-tree symlink).  So we
# inject the source into the `drivers` target directly here; this CMakeLists is
# reached from the board's CMakeLists (see boards/rk3576/kickpi-k7/CMakeLists).
###############################################################################

if(CONFIG_LCD_ST77916)
  target_sources(drivers PRIVATE st77916.c)
endif()

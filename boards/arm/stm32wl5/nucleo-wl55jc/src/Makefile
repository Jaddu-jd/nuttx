############################################################################
# boards/arm/stm32wl5/nucleo-wl55jc/src/Makefile
#
# Licensed to the Apache Software Foundation (ASF) under one or more
# contributor license agreements.  See the NOTICE file distributed with
# this work for additional information regarding copyright ownership.  The
# ASF licenses this file to you under the Apache License, Version 2.0 (the
# "License"); you may not use this file except in compliance with the
# License.  You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
# License for the specific language governing permissions and limitations
# under the License.
#
############################################################################

include $(TOPDIR)/Make.defs

CSRCS = stm32_boot.c stm32_leds.c stm32_spi.c stm32_bringup.c

CSRCS-$(CONFIG_BOARDCTL) += stm32_appinit.c
CSRCS-$(CONFIG_ARCH_BUTTONS) += stm32_buttons.c
CSRCS-$(CONFIG_ARCH_BOARD_FLASH_MOUNT) += stm32_flash.c
CSRCS-$(CONFIG_SPI_DRIVER) += stm32_spi.c
CSRCS-$(CONFIG_ARCH_BOARD_IPCC) += stm32_ipcc.c

ifeq ($(CONFIG_VIDEO_FB),y)
ifeq ($(CONFIG_LCD_SSD1680),y)
  CSRCS += stm32_ssd1680.c
endif
endif

CSRCS += $(CSRCS-y)

include $(TOPDIR)/boards/Board.mk

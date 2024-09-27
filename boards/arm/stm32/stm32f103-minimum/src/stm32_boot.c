/****************************************************************************
 * boards/arm/stm32/stm32f103-minimum/src/stm32_boot.c
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
#include <nuttx/spi/spi.h>
#include <debug.h>

#include <nuttx/board.h>
#include <arch/board/board.h>
#include <debug.h>
#include <syslog.h>

#include "arm_internal.h"
// #include "stm32f103_minimum.h"

// #include "stm32_gpio.h"
/****************************************************************************
 * Public Functions
 ****************************************************************************/
void board_peripheral_reset(int ms){
  // stm32_gpiowrite(GPIO_CS_MAG, 1);
  usleep(1000 * ms);
  syslog(LOG_DEBUG, "Reset done in %d ms \n", ms);
}
int board_app_initialize(uintptr_t arg){
  //syslog(LOG_SYSLOG, "Initializing board applications \n");
  board_peripheral_reset(10);
  #if defined(CONFIG_STM32_SPI1) || defined(CONFIG_STM32_SPI2)
  stm32_spidev_initialize();
#endif
return stm32_bringup();
}

/****************************************************************************
 * Name: stm32_boardinitialize
 *
 * Description:
 *   All STM32 architectures must provide the following entry point.  This
 *   entry point is called early in the initialization -- after all memory
 *   has been configured and mapped but before any devices have been
 *   initialized.
 *
 ****************************************************************************/

void stm32_boardinitialize(void)
{
  syslog(LOG_SYSLOG,"Initializing board applications\n");
  // board_peripheral_reset(10);
  //syslog(LOG_SYSLOG, "[Boot] function called \n");
  // stm32_configgpio(GPIO_CS_MAG);
  /* Configure on-board LEDs if LED support has been selected. */
// int ret = stm32_configgpio(GPIO_CS_MAG);
// printf("returned value %d while configuring gpiopin\n",ret);


  /* Configure SPI chip selects if
   * 1) SPI is not disabled, and
   * 2) the weak function stm32_spidev_initialize() has been brought into
   * the link.
   */

#if defined(CONFIG_STM32_SPI1) || defined(CONFIG_STM32_SPI2)
  stm32_spidev_initialize();
#endif

  /* Initialize USB is
   * 1) USBDEV is selected,
   * 2) the USB controller is not disabled, and
   * 3) the weak function stm32_usbinitialize() has been brought
   * into the build.
   */

#if defined(CONFIG_USBDEV) && defined(CONFIG_STM32_USB)
  stm32_usbinitialize();
#endif
// return stm32_bringup();
}

/****************************************************************************
 * Name: board_late_initialize
 *
 * Description:
 *   If CONFIG_BOARD_LATE_INITIALIZE is selected, then an additional
 *   initialization call will be performed in the boot-up sequence to a
 *   function called board_late_initialize().  board_late_initialize()
 *   will be called immediately after up_initialize() is called and just
 *   before the initial application is started. This additional
 *   initialization phase may be used, for example, to initialize
 *   board-specific device drivers.
 *
 ****************************************************************************/

#ifdef CONFIG_BOARD_LATE_INITIALIZE
void board_late_initialize(void)
{
#ifndef CONFIG_BOARDCTL
  /* Perform board initialization here instead of from the
   * board_app_initialize().
   */

  stm32_bringup();
#endif
}
#endif

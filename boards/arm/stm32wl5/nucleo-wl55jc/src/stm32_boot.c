/****************************************************************************
 * boards/arm/stm32wl5/nucleo-wl55jc/src/stm32_boot.c
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

#include <debug.h>
#include <stdio.h>
#include <syslog.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/spi/spi.h>
#include <stdbool.h>

#include <arch/board/board.h>
#include <string.h>

#ifdef CONFIG_VIDEO_FB
#include <nuttx/video/fb.h>
#endif
#include <fcntl.h>
#include "arm_internal.h"
#include "nucleo-wl55jc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/


 /************************************************************************************
 * Name: board_peripheral_reset
 *
 * Description:
 *
 ************************************************************************************/
 void board_peripheral_reset(int ms)
{
	// Setting the kill switch control gpio


	usleep(ms * 1000);
	syslog(LOG_DEBUG, "reset done, %d ms\n", ms);
	stm32wl5_gpiowrite(GPIO_MAG_CS, true);

}


/****************************************************************************
 * Name: stm32wl5_board_initialize
 *
 * Description:
 *   All STM32WL5 architectures must provide the following entry point.
 *   This entry point is called early in the initialization -- after all
 *   memory has been configured and mapped but before any devices have been
 *   initialized.
 *
 ****************************************************************************/

void stm32wl5_board_initialize(void)
{
  /* Configure on-board LEDs, which are always enabled */

  // board_leds_initialize();
  // sleep(10);
  // stm32_bringup();
  
  stm32wl5_configgpio(GPIO_MAG_CS);
  // int i=0;
  // while(i<5){
    syslog(LOG_DEBUG, "INIT board ms\n");
    
  //   usleep(10000);
  //   i++;
  // }
  stm32_bringup();

}

/****************************************************************************
 * Name: board_late_initialize
 *
 * Description:
 *   If CONFIG_BOARD_LATE_INITIALIZE is selected, then an additional
 *   initialization call will be performed in the boot-up sequence to a
 *   function called board_late_initialize(). board_late_initialize() will be
 *   called immediately after up_initialize() is called and just before the
 *   initial application is started.  This additional initialization phase
 *   may be used, for example, to initialize board-specific device drivers.
 *
 ****************************************************************************/

#ifdef CONFIG_BOARD_LATE_INITIALIZE
void board_late_initialize(void)
{
#if defined(CONFIG_VIDEO_FB)
  int ret;
#endif

#if defined(CONFIG_STM32WL5_SPI1) || defined(CONFIG_STM32WL5_SPI2S2)
  stm32wl5_spidev_initialize();
#endif

#if defined(CONFIG_LCD_SSD1680) && !defined(CONFIG_VIDEO_FB)
  board_lcd_initialize();
#endif

#ifdef CONFIG_VIDEO_FB
  ret = fb_register(0, 0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: fb_register() failed: %d\n", ret);
    }
#endif

  /* Perform NSH initialization here instead of from the NSH.  This
   * alternative NSH initialization is necessary when NSH is ran in
   * user-space but the initialization function must run in kernel space.
   */

#if defined(CONFIG_NSH_LIBRARY) && !defined(CONFIG_NSH_ARCHINIT)
  board_app_initialize(0);
#endif

stm32_bringup();
}
#endif


static struct spi_dev_s *spi1;

int board_app_initialize(uintptr_t arg)
{
  stm32wl5_configgpio(GPIO_MAG_CS);
//   stm32wl5_wdg_setup();
  printf("Initializing board applications.\n");
  stm32wl5_configgpio(GPIO_MAG_CS);

  board_peripheral_reset(10);
  

#if defined(CONFIG_STM32WL5_SPI1) || defined(CONFIG_STM32WL5_SPI1)

  /* Configure SPI chip selects if 1) SPI is not disabled, and 2) the weak
   * function stm32_spidev_initialize() has been brought into the link.
   */

  // if (stm32wl5_spidev_initialize())
    {
      stm32wl5_spidev_initialize(); 
    }
#endif

#ifdef CONFIG_BOARD_LATE_INITIALIZE
  /* Board initialization already performed by board_late_initialize() */

  return OK;
#else
  /* Perform board-specific initialization */
  
  return stm32_bringup();
  
#endif
}


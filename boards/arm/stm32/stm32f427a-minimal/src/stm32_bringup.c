/****************************************************************************
 * boards/arm/stm32/stm32f427a-minimal/src/stm32_bringup.c
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

#include <stdbool.h>
#include <stdio.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/board.h>
#include <nuttx/fs/fs.h>
#include <nuttx/kmalloc.h>

#ifdef CONFIG_ADC_ADS7953
#include <nuttx/analog/ads7953.h>
#endif

#if defined(CONFIG_MTD_SST25XX) || defined(CONFIG_MTD_PROGMEM)
#  include <nuttx/mtd/mtd.h>
#endif

// #ifndef CONFIG_STM32F427V_FLASH_MINOR
// #define CONFIG_STM32F427V_FLASH_MINOR 0
// #endif

// #ifdef CONFIG_STM32F427V_FLASH_CONFIG_PART
// #ifdef CONFIG_PLATFORM_CONFIGDATA
// #  include <nuttx/mtd/configdata.h>
// #endif
// #endif

#ifdef CONFIG_STM32_OTGHS
#  include "stm32_usbhost.h"
#endif

#include "stm32.h"
#include "stm32f427a.h"


/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y :
 *     Called from board_late_initialize().
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y && CONFIG_BOARDCTL=y :
 *     Called from the NSH library
 *
 ****************************************************************************/

int stm32_bringup(void)
{
  int ret = 0;

#if defined(CONFIG_STM32_SPI1) 
  struct spi_dev_s *spi1;
#endif

#if defined(CONFIG_STM32_SPI2) 
  struct spi_dev_s *spi2;
#endif

#if defined(CONFIG_STM32_SPI3) 
  struct spi_dev_s *spi3;
#endif

#if defined(CONFIG_STM32_SPI4)
  struct spi_dev_s *spi;
#endif

#if defined(CONFIG_STM32_SPI5)
  struct spi_dev_s *spi;
#endif

/* Created But not used; to be used later to improve readability and modularity*/
#if defined(CONFIG_ADC_ADS7953)
  struct ads7953_config_s ads7953_config;
#endif

#ifdef HAVE_USBHOST
  /* Initialize USB host operation.  stm32_usbhost_initialize() starts a
   * thread will monitor for USB connection and disconnection events.
   */

  ret = stm32_usbhost_initialize();
  if (ret != OK)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize USB host: %d\n", ret);
      return ret;
    }
#endif

#ifdef HAVE_USBMONITOR
  /* Start the USB Monitor */

  ret = usbmonitor_start();
  if (ret != OK)
    {
      syslog(LOG_ERR, "ERROR: Failed to start USB monitor: %d\n", ret);
    }
#endif

#ifdef CONFIG_SENSORS_L3GD20
  ret = board_l3gd20_initialize(0, 5);
  if (ret != OK)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize l3gd20 sensor:"
             " %d\n", ret);
    }
#endif

#ifdef CONFIG_PWM
  /* Initialize PWM and register the PWM device. */

  ret = stm32_pwm_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_pwm_setup() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_ADC
  /* Initialize ADC and register the ADC device. */
  ret = stm32_adc_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_adc_setup() failed: %d\n", ret);
    }
#endif //CONFIG_ADC

#ifdef CONFIG_ADC_ADS7953

  syslog(LOG_INFO,"[BRINGUP] INFO: Initializing SPI Bus 2");
  spi2 = stm32_spibus_initialize(2);
  if(!spi2){
    syslog(LOG_ERR, "[BRINGUP] ERROR: stm32_spibus_initialize() failed for SPI 2: %d \n", ret);
    printf("Failed to initialize SPI Bus 2");
  }
  
  printf("Initialized SPI Bus 2 Successfully");

  ret = ads7953_register("/dev/ext_adc1",spi2, 0);
  if(ret < 0){
    syslog(LOG_ERR,"[BRINGUP] ERROR: ads7953_register() failed %d \n", ret);
    printf("Could not register External ADC");
    return -ENODEV;
  }

#endif  //CONFIG_ADC_ADS7953


#ifdef CONFIG_STM32_CAN_CHARDRIVER
  /* Initialize CAN and register the CAN driver. */

  ret = stm32_can_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_can_setup failed: %d\n", ret);
    }
#endif

  UNUSED(ret);
  return OK;
}
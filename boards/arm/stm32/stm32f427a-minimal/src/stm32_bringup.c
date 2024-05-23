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
#include <string.h>

#include <nuttx/spi/spi.h>
#include <stm32_spi.h>
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
#if defined(CONFIG_MTD_MT25QL) || defined(CONFIG_MTD_PROGMEM)
#  include <nuttx/mtd/mtd.h>
#endif

#ifndef CONFIG_STM32F427A_FLASH_MINOR
#define CONFIG_STM32F427A_FLASH_MINOR 0
#endif

#ifdef CONFIG_STM32F427A_FLASH_CONFIG_PART
#ifdef CONFIG_PLATFORM_CONFIGDATA
#  include <nuttx/mtd/configdata.h>
#endif
#endif

#include <nuttx/sensors/lis3mdl.h>

#include "stm32.h"
#include "stm32f427a.h"

#ifdef CONFIG_SENSORS_LIS3MDL
typedef struct mag_priv_s
{
  struct lis3mdl_config_s dev;
  xcpt_t handler;
  void *arg;
  uint32_t intcfg;
};


/* IRQ/GPIO access callbacks.  These operations all hidden behind
 * callbacks to isolate the MRF24J40 driver from differences in GPIO
 * interrupt handling by varying boards and MCUs.  If possible,
 * interrupts should be configured on both rising and falling edges
 * so that contact and loss-of-contact events can be detected.
 *
 *   irq_attach       - Attach the MRF24J40 interrupt handler to the GPIO
 *                      interrupt
 *   irq_enable       - Enable or disable the GPIO interrupt
 */

static int stm32_attach_irq(const struct lis3mdl_config_s *lower,
                            xcpt_t handler, void *arg)
{
  struct mag_priv_s *priv = (struct mag_priv_s *)lower;

  DEBUGASSERT(priv != NULL);

  /* Just save the handler for use when the interrupt is enabled */

  priv->handler = handler;
  priv->arg     = arg;
  return OK;
}

static struct mag_priv_s mag0 =
{
  .dev.attach = stm32_attach_irq,
  .dev.spi_devid = SPIDEV_USER(0),
  .handler = NULL,
  .intcfg = GPIO_LIS3MDL_INT,
};
#endif

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

#if defined(CONFIG_MTD)
  struct mtd_dev_s *mtd;
#if defined (CONFIG_MTD_MT25QL)
  struct mtd_geometry_s geo;
#endif  // CONFIG_MTD_MT25QL
#endif  // CONFIG_MTD

#if defined(CONFIG_MTD_PARTITION_NAMES)
  const char *partname = CONFIG_STM32F427A_FLASH_PART_NAMES;
#endif // CONFIG_MTD_PARTITION_NAMES


  /* Configure SPI-based devices */


#ifdef CONFIG_SENSORS_LIS3MDL

  /* Init SPI Bus again */

  spi5 = stm32_spibus_initialize(5);
  if (!spi5)
  {
    printf("[BRING_UP] ERROR: Failed to Initialize SPI 5 bus.\n");
  } else {
    printf("[BRING_UP] Initialized bus on SPI port 5.\n");

    SPI_SETFREQUENCY(spi5, 1000000);
    SPI_SETBITS(spi5, 8);
    SPI_SETMODE(spi5, SPIDEV_MODE0);
  }

  ret = lis3mdl_register("/dev/mag0", spi5, &mag0.dev);
  if (ret < 0)
  {
    printf("[BRING_UP] Error: Failed to register LIS3MDL driver.\n");
  } else {
    printf("[BRING_UP] LIS3MDL registered on SPI 5.\n");
  }
#endif  // CONFIG_SENSORS_LIS3MDL

#if defined(CONFIG_MTD) && defined(CONFIG_MTD_PROGMEM)
  mtd = progmem_initialize();
  if (mtd == NULL)
    {
      syslog(LOG_ERR, "ERROR: progmem_initialize\n");
    }

  ret = register_mtddriver("/dev/flash", mtd, 0, mtd);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: register_mtddriver() failed: %d\n", ret);
    }

#endif

#ifdef CONFIG_STM32_SPI3
  /* Get the SPI port */

  syslog(LOG_INFO, "Initializing SPI port 3\n");
  printf("Initalizaing SPI PORT 3.\n");
  
  spi3 = stm32_spibus_initialize(3);
  if (!spi3)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize SPI port 3\n");
      printf("Error initialzing SPI PORT 3\n");
    } else {
      syslog(LOG_INFO, "Successfully initialized SPI port 3\n");
      printf("SPI PORT 3 Successfully Initalized.\n");
    }


  /* Now bind the SPI interface to the SST25F064 SPI FLASH driver.  This
   * is a FLASH device that has been added external to the board (i.e.
   * the board does not ship from STM with any on-board FLASH.
   */

#if defined(CONFIG_MTD) && defined(CONFIG_MTD_MT25QL)
  syslog(LOG_INFO, "Bind SPI to the SPI flash driver\n");

  mtd = mt25ql_initialize(spi3);
  if (!mtd)
    {
      syslog(LOG_ERR, "ERROR: Failed to bind SPI port 3 to the SPI FLASH"
                      " driver\n");
    }
  else
    {
      syslog(LOG_INFO, "Successfully bound SPI port 3 to the SPI FLASH"
                       " driver\n");

      /* Get the geometry of the FLASH device */

      ret = mtd->ioctl(mtd, MTDIOC_GEOMETRY,
                       (unsigned long)((uintptr_t)&geo));
      if (ret < 0)
        {
          printf("ERROR: mtd->ioctl failed: %d\n", ret);
        }

#endif  /* CONFIG_MTD...*/
#endif  /* CONFIG_STM32_SPI3 */


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


  UNUSED(ret);
  return OK;
}
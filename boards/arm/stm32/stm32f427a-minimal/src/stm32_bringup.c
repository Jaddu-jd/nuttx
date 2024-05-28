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
#include<stdio.h>
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

// #ifndef CONFIG_STM32F427V_FLASH_MINOR
// #define CONFIG_STM32F427V_FLASH_MINOR 0
// #endif

// #ifdef CONFIG_STM32F427V_FLASH_CONFIG_PART
// #ifdef CONFIG_PLATFORM_CONFIGDATA
// #  include <nuttx/mtd/configdata.h>
// #endif
// #endif
#if defined(CONFIG_MTD_MT25QL) || defined(CONFIG_MTD_PROGMEM) || defined(CONFIG_MTD_M25P)
#  include <nuttx/mtd/mtd.h>
// #include <nuttx/mtd/mtd
#endif



#ifndef CONFIG_STM32F427A_FLASH_MINOR
#define CONFIG_STM32F427A_FLASH_MINOR 0
#endif

#ifdef CONFIG_STM32F427A_FLASH_CONFIG_PART
#ifdef CONFIG_PLATFORM_CONFIGDATA
#  include <nuttx/mtd/configdata.h>
#endif
#endif

#include "stm32.h"
#include "stm32f427a.h"


#ifdef CONFIG_SENSORS_MPU60X0
  #include <nuttx/sensors/mpu60x0.h>
#endif

#ifdef CONFIG_MTD_MX25L
  #include <nuttx/mtd/mtd.h>
#endif

#ifdef CONFIG_SENSORS_LIS3MDL
#include <nuttx/sensors/lis3mdl.h>
#endif

#ifdef CONFIG_SENSORS_MPU60X0
  #include <nuttx/sensors/mpu60x0.h>
#endif

#ifdef CONFIG_MTD_MX25L
  #include <nuttx/mtd/mtd.h>
#endif

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
  struct spi_dev_s *spi4;
#endif

#if defined(CONFIG_STM32_SPI5)
  struct spi_dev_s *spi5;
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

#if defined(CONFIG_MTD) || defined(CONFIG_MTD_M25P)
  struct mtd_dev_s *mtd;
  #if defined(CONFIG_MTD_MX25L)
struct mtd_dev_s *mtd4;
  #endif

#if defined (CONFIG_MTD_MT25QL) || defined(CONFIG_MTD_M25P)
  struct mtd_geometry_s geo;
#endif  // CONFIG_MTD_MT25QL
#endif  // CONFIG_MTD

#if defined(CONFIG_MTD_PARTITION_NAMES)
  const char *partname = CONFIG_STM32F427A_FLASH_PART_NAMES;
#endif // CONFIG_MTD_PARTITION_NAMES


  /* Configure SPI-based devices */

#ifdef CONFIG_SENSORS_MPU60X0
struct mpu_config_s *mpu_config = NULL;
  spi5 = stm32_spibus_initialize(5);
  if(!spi5){
    printf("[Bringup ] Error : Failed to initialize spi 5 bus");
  }
  else{
    printf("[Bring up ] Successfully initialized spi 5 bus  for mpu");
  }
  SPI_SETFREQUENCY(spi5, 1000000);
  SPI_SETBITS(spi5,8);
  SPI_SETMODE(spi5, SPIDEV_MODE0);
   mpu_config = kmm_zalloc(sizeof(struct mpu_config_s));
   printf("the size of mpu_confi is %d", sizeof(mpu_config));
    // if (mpu_config == NULL)
    //   {
    //     printf("ERROR: Failed to allocate mpu60x0 driver\n");
    //   }
    //  else{
        mpu_config->spi = spi5;
        mpu_config->spi_devid = SPIDEV_IMU(0);
        ret = mpu60x0_register("/dev/mpu6050", mpu_config);
        if(ret<0){
          printf("[bring up] failed to initialize driver of mppu 6500");
        }
        else{
          printf("[bringup ] successfully initialized driver of mpu 6500");
        }
    //  }
#endif

#ifdef CONFIG_SENSORS_LIS3MDL


static struct mag_priv_s mag0 =
{
  .dev.attach = stm32_attach_irq,
  .dev.spi_devid = SPIDEV_USER(0),
  .handler = NULL,
  .intcfg = GPIO_LIS3MDL_DRDY,
};

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
// #if defined(CONFIG_MTD) && defined(CONFIG_MTD_M25P)
  // mtd = progmem_initialize();
//   if (mtd == NULL)
//     {
//       syslog(LOG_ERR, "ERROR: progmem_initialize\n");
//     }

//   ret = register_mtddriver("/dev/flash", mtd, 0, mtd);
//   ret = register_partition_with_mtd("/dev/mtd",
//                                 mode_t mode, mtd,
//                                 off_t firstblock, off_t nblocks);
//   if (ret < 0)
//     {
//       syslog(LOG_ERR, "ERROR: register_mtddriver() failed: %d\n", ret);
//     }

// #endif

#ifdef CONFIG_STM32_SPI3
  /* Get the SPI port */

  syslog(LOG_INFO, "Initializing SPI port 3\n");
  printf("Initalizaing SPI PORT 3.\n");
  
  spi3 = stm32_spibus_initialize(3);
  if (!spi3)
  {
    printf("[BRING_UP] ERROR: Failed to Initialize SPI 3 bus.\n");
  } else {
    printf("[BRING_UP] Initialized bus on SPI port 3.\n");

    SPI_SETFREQUENCY(spi3, 1000000);
    SPI_SETBITS(spi3, 8);
    SPI_SETMODE(spi3, SPIDEV_MODE0);
  }


  /* Now bind the SPI interface to the SST25F064 SPI FLASH driver.  This
   * is a FLASH device that has been added external to the board (i.e.
   * the board does not ship from STM with any on-board FLASH.
   */

#if defined(CONFIG_MTD) && defined(CONFIG_MTD_M25P) || defined(CONFIG_MTD_MT25QL)
  syslog(LOG_INFO, "Bind SPI to the SPI flash driver\n");
  #if defined(CONFIG_MTD_MT25QL)
  mtd = mt25ql_initialize(spi3);
  #endif
  #if defined(CONFIG_MTD_MX25L)
  // mtd4 = mx25l_initialize_spi(spi4);S
  #endif
  #if defined(CONFIG_MTD_M25P)
  mtd = m25p_initialize(spi3);//mx25l_initialize_spi(spi3);
   ret = register_mtddriver("/dev/flash", mtd, 0, mtd);

  // ret = register_partition_with_mtd("/dev/mtd",
//                                 mode_t mode, mtd,
//                                 off_t firstblock, off_t nblocks);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: register_mtddriver() failed: %d\n", ret);
    }
    else{
      printf("mtd driver registered successfully ... \r\n");
    }
    
  #endif

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

#ifdef CONFIG_STM32F427A_FLASH_PART
        {
          int partno;
          int partsize;
          int partoffset;
          int partszbytes;
          int erasesize;
          const char *partstring = CONFIG_STM32F427A_FLASH_PART_LIST;
          // const char partstring = 4096;
          const char *ptr;
          struct mtd_dev_s *mtd_part;
          char  partref[16];

          /* Now create a partition on the FLASH device */

          partno = 0;
          ptr = partstring;
          partoffset = 0;


          /* Get the Flash erase size */

          erasesize = geo.erasesize;
          printf("======================the errasesize is %d ======= partstring %d pointer(ptr) %d \n", erasesize, partsize);
          while (*ptr != '\0')
            {
              /* Get the partition size */

            
              partsize = atoi(partstring);
              partszbytes = (partsize << 10); /* partsize is defined in KB */

              /* Check if partition size is bigger then erase block */
          printf("======================the partsize and partszbytess,partoffset is %d and %d  and %d=======\n",partsize, partszbytes, partoffset);
              if(partoffset >= partszbytes){
                  break;
                }
              if (partszbytes < erasesize)
                {
                  printf("ERROR: Partition size is lesser than erasesize!\n");
                  return -1;
                }

              /* Check if partition size is multiple of erase block */

              if ((partszbytes % erasesize) != 0)
                {
                  printf("ERROR: Partition size is not multiple of"
                       " erasesize!\n");
                  return -1;
                }

              mtd_part    = mtd_partition(mtd, partoffset,
                                          partszbytes / erasesize); 

              partoffset += partszbytes / erasesize;

#ifdef CONFIG_STM32F427A_FLASH_CONFIG_PART
              /* Test if this is the config partition */

              if (CONFIG_M25P_MANUFACTURER == partno)
                {
                  /* Register the partition as the config device */

                  // mtdconfig_register(mtd_part);
                }
              else
#endif
                {
                  /* Now initialize a SMART Flash block device and bind it
                   * to the MTD device.
                   */

#if defined(CONFIG_MTD_SMART) //&& defined(CONFIG_FS_SMARTFS)
                  snprintf(partref, sizeof(partref), "p%d", partno);
                  smart_initialize(CONFIG_STM32F427A_FLASH_MINOR,
                                   mtd_part, partref);
#endif
                }

#if defined(CONFIG_MTD_PARTITION_NAMES)
              /* Set the partition name */

              if (mtd_part == NULL)
                {
                  ferr("ERROR: failed to create partition %s\n", partname);
                  return -1;
                }

              mtd_setpartitionname(mtd_part, partname);

              /* Now skip to next name.  We don't need to split the string
               * here because the MTD partition logic will only display names
               * up to the comma, thus allowing us to use a single static
               * name in the code.
               */

              while (*partname != ',' && *partname != '\0')
                {
                  /* Skip to next ',' */

                  partname++;
                }

              if (*partname == ',')
                {
                  partname++;
                }
#endif

              /* Update the pointer to point to the next size in the list */

              while ((*ptr >= '0') && (*ptr <= '9'))
                {
                  ptr++;
                }

              if (*ptr == ',')
                {
                  ptr++;
                }

              /* Increment the part number */

              partno++;
            }
        }
#endif /* CONFIG_STM32F427A_FLASH_PART */
    }

#endif /* CONFIG_MTD */
#endif /* CONFIG_STM32_SPI3 */

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
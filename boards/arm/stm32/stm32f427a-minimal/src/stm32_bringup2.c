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

#if defined(CONFIG_MTD_MT25QL) || defined(CONFIG_MTD_PROGMEM) || defined(CONFIG_MTD_M25P)
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


#ifdef CONFIG_SENSORS_MPU60X0
  #include <nuttx/sensors/mpu60x0.h>
#endif

#ifdef CONFIG_MTD_MX25L
  #include <nuttx/mtd/mtd.h>
#endif

#ifdef CONFIG_SENSORS_LIS3MDL

struct mag_priv_s
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

#if defined(CONFIG_STM32_SPI2)
  struct spi_dev_s *spi2;
#endif

#if defined(CONFIG_STM32_SPI3)
  struct spi_dev_s *spi3;
#endif

#if defined(CONFIG_STM32_SPI4)
  // struct spi_dev_s *spi4;
#endif

#if defined(CONFIG_STM32_SPI5)
  struct spi_dev_s *spi5;
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
  int ret;

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

#ifdef CONFIG_STM32_SPI3  // Close the block properly with #endif at the end
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
  printf("Reached ckpt 1 bringup mtd\n");
  #if defined(CONFIG_MTD_MT25QL)
  mtd = mt25ql_initialize(spi3);
  #else
  mtd = m25p_initialize(spi3);
  #endif // defined(CONFIG_MTD_MT25QL)
  if (!mtd)
    {
      syslog(LOG_ERR, "ERROR: Failed to bind SPI port 3 to the SPI FLASH driver\n");
    }
  else
    {
#if defined(CONFIG_STM32F427A_FLASH_PART)
      {int count = 0;
        int partno;
        int partsize;
        int partoffset = 0;
        char  partname[4];
        FAR struct mtd_dev_s *mtd_part;
        const char *partstring = CONFIG_STM32F427A_FLASH_PART_LIST;
        const char *ptr;
        FAR struct mtd_geometry_s geo;

        /* Now create a partition on the FLASH device */
        printf("Successfully initialized the m25p driver \n");
        /* Get the geometry of the FLASH device */
        ret = mtd->ioctl(mtd, MTDIOC_GEOMETRY, (unsigned long)((uintptr_t)&geo));
        if (ret < 0)
          {
            syslog(LOG_ERR, "ERROR: m25p_ioctl(MTDIOC_GEOMETRY) failed: %d\n", ret);
          }

        /* Parse the partition list */
        ptr = partstring;
        partno = 0;

        while (*ptr != '\0')
          {
            partsize = atoi(ptr);
            mtd_part = mtd_partition(mtd, partoffset, (partsize >> 2) * geo.neraseblocks / 100);
            partoffset += (partsize >> 2) * geo.neraseblocks / 100;
            sprintf(partname, "/dev/mtd%d", partno);
            printf("partition name %s", partname);
            ret = register_mtddriver(partname, mtd_part, 0777, NULL);

            if (ret < 0)
              {
                syslog(LOG_ERR, "ERROR: register_mtddriver /dev/mtdblock%d failed: %d\n",
                       partno, ret);
              }
              char partname2[30];
            sprintf(partname2, "mtd%d", partno);
            ret = nx_mount(partname,partname2, "smartfs", 0, NULL);
            if(ret<0){
              printf("\n-----------------Some error moounting lfs %d\n",ret);
            } 
            else{
              printf("\n-----------------Mount successful\n");
            } 
#if defined(CONFIG_MTD_PARTITION_NAMES)
            mtd_setpartitionname(mtd_part, partname);
#endif
            partno++;
            while ((*ptr >= '0') && (*ptr <= '9'))
              {
                ptr++;
              }

            if (*ptr == ',')
              {
                ptr++;
              }
          }
      }
#else
      printf("Registering mtd flash driver \n");
      ret = register_mtddriver("/dev/mtd0", mtd, 0, mtd);
      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: Failed to register m25p driver as /dev/mtd0: %d\n",
                 ret);
        }
      else
        {
          printf("Successfully binded spiflash to the driver mtd \n");

        }
#endif
    }
#endif
  /* Initialize the flash file system. */
  ret = mount("/dev/mtd", "mnt", "smartfs", 0, NULL);
  if(ret<0){
    printf("error mounting little fs on /mnt folder");
  }
  else{
    printf("Successfully mounted littlfs on mnt folder");
  }
  return OK;
#endif  // CONFIG_STM32_SPI3

  return OK;
}

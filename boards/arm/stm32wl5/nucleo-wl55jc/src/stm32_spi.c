/****************************************************************************
 * boards/arm/stm32wl5/nucleo-wl55jc/src/stm32_spi.c
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

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>
#include <errno.h>

#include <arch/board/board.h>
#include <nuttx/spi/spi.h>
#include <stm32wl5_spi.h>

#include "arm_internal.h"
#include "chip.h"

#include "stm32wl5.h"
#include "nucleo-wl55jc.h"

#if defined(CONFIG_STM32WL5_SPI1) || defined(CONFIG_STM32WL5_SPI2S2)

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Global driver instances */

#ifdef CONFIG_STM32WL5_SPI1
struct spi_dev_s *g_spi1;
#endif
#ifdef CONFIG_STM32WL5_SPI2S2
struct spi_dev_s *g_spi2;
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the Nucleo-wl55JC
 *   boards.
 *
 ****************************************************************************/

void weak_function stm32wl5_spidev_initialize(void)
{
#ifdef CONFIG_STM32WL5_SPI1
  /* Configure SPI-based devices */

  g_spi1 = stm32wl5_spibus_initialize(1);
  if (!g_spi1)
    {
      spierr("ERROR: FAILED to initialize SPI port 1\n");
      return;
    }

#ifdef CONFIG_LCD_SSD1680
  spiinfo("Preparing additional lines for SSD1680 device\n");
  stm32wl5_configgpio(GPIO_SSD1680_CS);    /* SSD1680 chip select */
  stm32wl5_configgpio(GPIO_SSD1680_CMD);   /* SSD1680 data/!command */
  stm32wl5_configgpio(GPIO_SSD1680_RST);   /* SSD1680 reset */
  stm32wl5_configgpio(GPIO_SSD1680_BUSY);  /* SSD1680 busy */
#endif

#endif

#ifdef CONFIG_STM32_SPI2S2
  /* Configure SPI-based devices */

  g_spi2 = stm32_spibus_initialize(2);
#endif
}

/****************************************************************************
 * Name:  stm32_spi1/2s2elect and stm32_spi1/2s2status
 *
 * Description:
 *   The external functions, stm32_spi1/2s2select and stm32_spi1/2s2status
 *   must be provided by board-specific logic.  They are implementations of
 *   the select and status methods of the SPI interface defined by struct
 *   spi_ops_s (see include/nuttx/spi/spi.h). All other methods (including
 *   stm32wl5_spibus_initialize()) are provided by common STM32 logic.
 *   To use this common SPI logic on your board:
 *
 *   1. Provide logic in stm32_boardinitialize() to configure SPI chip select
 *      pins.
 *   2. Provide stm32_spi1/2s2select() and stm32_spi1/2s2status() functions
 *      in your board-specific logic.  These functions will perform chip
 *      selection and status operations using GPIOs in the way your board is
 *      configured.
 *   3. Add a calls to stm32wl5_spibus_initialize() in your low level
 *      application initialization logic
 *   4. The handle returned by stm32wl5_spibus_initialize() may then be used
 *      to bind the SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ****************************************************************************/

#ifdef CONFIG_STM32WL5_SPI1
void stm32wl5_spi1select(struct spi_dev_s *dev, uint32_t devid,
                      bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid, selected ? "assert" :
          "de-assert");

#if defined(CONFIG_LCD_SSD1680)
  if (devid == SPIDEV_DISPLAY(0))
    {
      stm32wl5_gpiowrite(GPIO_SSD1680_CS, !selected);
    }
#endif

#if defined(CONFIG_CAN_MCP2515)
  if (devid == SPIDEV_CANBUS(0))
    {
      stm32wl5_gpiowrite(GPIO_MCP2515_CS, !selected);
    }
#endif

#ifdef HAVE_MMCSD
  if (devid == SPIDEV_MMCSD(0))
    {
      stm32wl5_gpiowrite(GPIO_SPI_CS_SD_CARD, !selected);
    }
#endif
}

uint8_t stm32wl5_spi1status(struct spi_dev_s *dev, uint32_t devid)
{
#if defined(CONFIG_LCD_SSD1680)
  if (devid == SPIDEV_DISPLAY(0))
    {
      return SPI_STATUS_PRESENT;
    }
#endif

  return 0;
}

int stm32wl5_spi1cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
#if defined(CONFIG_LCD_SSD1680)
  if (devid == SPIDEV_DISPLAY(0))
    {
      stm32wl5_gpiowrite(GPIO_SSD1680_CMD, !cmd);
    }
#endif

  return OK;
}

#endif

#ifdef CONFIG_STM32WL5_SPI2S2
void stm32wl5_spi2s2select(struct spi_dev_s *dev, uint32_t devid,
                      bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid, selected ? "assert" :
          "de-assert");
}

uint8_t stm32wl5_spi2s2status(struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}

int stm32wl5_spi2s2cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  return OK;
}

#endif

/****************************************************************************
 * Name: stm32_spi1cmddata
 *
 * Description:
 *   Set or clear the SH1101A A0 or SD1306 D/C n bit to select data (true)
 *   or command (false). This function must be provided by platform-specific
 *   logic. This is an implementation of the cmddata method of the SPI
 *   interface defined by struct spi_ops_s (see include/nuttx/spi/spi.h).
 *
 * Input Parameters:
 *
 *   spi - SPI device that controls the bus the device that requires the CMD/
 *         DATA selection.
 *   devid - If there are multiple devices on the bus, this selects which one
 *         to select cmd or data.  NOTE:  This design restricts, for example,
 *         one one SPI display per SPI bus.
 *   cmd - true: select command; false: select data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_SPI_CMDDATA
#ifdef CONFIG_STM32_SPI1
int stm32_spi1cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
#if defined(CONFIG_LCD_SSD16800)
  if (devid == SPIDEV_DISPLAY(0))
    {
      stm32_gpiowrite(GPIO_SSD1680_CMD, !cmd);
    }
#endif

  return OK;
}
#endif

#ifdef CONFIG_STM32_SPI2
int stm32_spi2cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  return OK;
}
#endif

#ifdef CONFIG_STM32_SPI3
int stm32_spi3cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  return OK;
}
#endif
#endif /* CONFIG_SPI_CMDDATA */

#endif /* CONFIG_STM32_SPI1 || CONFIG_STM32_SPI2 || CONFIG_STM32_SPI3 */

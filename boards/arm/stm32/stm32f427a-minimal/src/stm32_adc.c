/****************************************************************************
 * boards/arm/stm32/stm32f427a-minimal/src/stm32_adc.c
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
#include <errno.h>
#include <debug.h>

#include <nuttx/board.h>
#include <nuttx/analog/adc.h>

#include <stdio.h>

#include "stm32.h"

#include <stdint.h>
#include <string.h>

#include <nuttx/spi/spi.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"

#include "nuttx/analog/ads7953.h"


#if defined(CONFIG_ADC) && (defined(CONFIG_STM32_ADC1) || defined(CONFIG_STM32_ADC3))

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* 1 or 2 ADC devices (DEV1, DEV2).
 * ADC1 and ADC3 supported for now.
 */

#if defined(CONFIG_STM32_ADC1)
#  define DEV1_PORT 1
#endif

#if defined(CONFIG_STM32_ADC3)
#  if defined(DEV1_PORT)
#    define DEV2_PORT 3
#  else
#    define DEV1_PORT 3
#  endif
#endif

/* The number of ADC channels in the conversion list */

/* TODO DMA */

#define ADC1_NCHANNELS 14
#define ADC3_NCHANNELS 1

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* DEV 1 */

#if DEV1_PORT == 1

#define DEV1_NCHANNELS ADC1_NCHANNELS

/* Identifying number of each ADC channel (even if NCHANNELS is less ) */

static const uint8_t g_chanlist1[DEV1_NCHANNELS] =
{
  0,
  2,
  3,
  4,
  5,
  6,
  7,
  8,
  9,
  10,
  11,
  12,
  14,
  15,
};

/* Configurations of pins used by each ADC channel */

static const uint32_t g_pinlist1[DEV1_NCHANNELS]  =
{
  GPIO_ADC3_IN0,                /* PA0/J5   UNREG CURRENT               ADC1/2/3 COMP. */  

  GPIO_ADC3_IN2,                /* PA2/K2   MAIN 3V3 CURRENT            ADC1/2/3 COMP. */
  GPIO_ADC3_IN3,                /* PA3/K3   3V3 COM CURRENT             ADC1/2/3 COMP. */

  GPIO_ADC1_IN4,                /* PA4/N2   5V CURRENT                  ADC1/2 COMP. */
  GPIO_ADC1_IN5,                /* PA5/M3   BATTERY MONITOR             ADC1/2 COMP. */
  GPIO_ADC1_IN6,                /* PA6/N3   SOLAR PANEL 1 CURRENT       ADC1/2 COMP. */
  GPIO_ADC1_IN7,                /* PA7/K4   3V3 2 CURRENT               ADC1/2 COMP. */
  GPIO_ADC1_IN8,                /* PB0/N4   SOLAR PANEL 4 CURRENT       ADC1/2 COMP. */
  GPIO_ADC1_IN9,                /* PB1/K5   SOLAR PANEL 5 CURRENT       ADC1/2 COMP. */

  GPIO_ADC3_IN10,               /* PC0/G6   BATTERY CURRENT             ADC1/2/3 COMP. */
  GPIO_ADC3_IN11,               /* PC1/H5   SOLAR PANEL TOTAL CURRENT   ADC1/2/3 COMP. */
  GPIO_ADC3_IN12,               /* PC2/H6   RAW CURRENT                 ADC1/2/3 COMP. */

  GPIO_ADC1_IN14,               /* PC4/L4   SOLAR PANEL 2 CURRENT       ADC1/2 COMP. */
  GPIO_ADC1_IN15,               /* PC5/M4   SOLAR PANEL 3 CURRENT       ADC1/2 COMP. */
};

#elif DEV1_PORT == 3

#define DEV1_NCHANNELS ADC3_NCHANNELS

/* Identifying number of each ADC channel */

static const uint8_t g_chanlist1[1] =
{
  14,
};

/* Configurations of pins used by each ADC channel */

static const uint32_t g_pinlist1[1] =
{
  GPIO_ADC3_IN14,                /* PF4/G3    4V CURRENT    ADC3 COMP. */
};

#endif /* DEV1_PORT == 1 */

#ifdef DEV2_PORT

/* DEV 2 */

#if DEV2_PORT == 3

#define DEV2_NCHANNELS ADC3_NCHANNELS

/* Identifying number of each ADC channel */

static const uint8_t g_chanlist2[1] =
{
  14,
};

/* Configurations of pins used by each ADC channel */

static const uint32_t g_pinlist2[1] =
{
  GPIO_ADC3_IN14,                /* PF4/G3    4V CURRENT    ADC3 COMP. */
};

#endif /* DEV2_PORT == 3 */
#endif /* DEV2_PORT */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_adc_setup
 *
 * Description:
 *   Initialize ADC and register the ADC driver.
 *
 ****************************************************************************/

int stm32_adc_setup(void)
{
  static bool initialized = false;
  struct adc_dev_s *adc;
  int ret;
  int i;

  /* Check if we have already initialized */

  if (!initialized)
    {
      /* DEV1 */

      /* Configure the pins as analog inputs for the selected channels */

      for (i = 0; i < DEV1_NCHANNELS; i++)
        {
          stm32_configgpio(g_pinlist1[i]);
        }

      /* Call stm32_adcinitialize() to get an instance of the ADC interface */

      adc = stm32_adcinitialize(DEV1_PORT, g_chanlist1, DEV1_NCHANNELS);
      if (adc == NULL)
        {
          printf("ERROR: Failed to get ADC interface 1\n");
          return -ENODEV;
        }

      /* Register the ADC driver at "/dev/adc0" */
      ret = adc_register("/dev/adc0", adc);
      if (ret < 0)
        {
          printf("ERROR: adc_register /dev/adc0 failed: %d\n", ret);
          return ret;
        }

#ifdef DEV2_PORT
      /* DEV2 */

      /* Configure the pins as analog inputs for the selected channels */

      for (i = 0; i < DEV2_NCHANNELS; i++)
        {
          stm32_configgpio(g_pinlist2[i]);
        }

      /* Call stm32_adcinitialize() to get an instance of the ADC interface */

      adc = stm32_adcinitialize(DEV2_PORT, g_chanlist2, DEV2_NCHANNELS);
      if (adc == NULL)
        {
          aerr("ERROR: Failed to get ADC interface 2\n");
          return -ENODEV;
        }

      /* Register the ADC driver at "/dev/adc1" */

      ret = adc_register("/dev/adc1", adc);
      if (ret < 0)
        {
          aerr("ERROR: adc_register /dev/adc1 failed: %d\n", ret);
          return ret;
        }
#endif

      initialized = true;
    }

  return OK;
}

#endif /* CONFIG_ADC && (CONFIG_STM32_ADC1 || CONFIG_STM32_ADC3) */

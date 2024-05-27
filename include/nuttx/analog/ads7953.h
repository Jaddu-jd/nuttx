/****************************************************************************
 * include/nuttx/analog/ltc1867l.h
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

#ifndef __INCLUDE_NUTTX_ANALOG_ADS7953_H
#define __INCLUDE_NUTTX_ANALOG_ADS7953_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/spi/spi.h>
#include <stdint.h>

#include <nuttx/analog/ioctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* ADS7953 Configuration ***************************************************/

/*commands to set the modes of the ADC*/
#define 		MANUAL_MODE_1  			0x1A
#define			MANUAL_MODE_2			0xC0

#define 		AUTO_1_MODE_1 			0x2C
#define			AUTO_1_MODE_2			0x0F

#define 		AUTO_2_MODE_1 			0x3C
#define			AUTO_2_MODE_2			0x00

#define 		AUTO_2_MODE2_1 			0x38
#define			AUTO_2_MODE2_2			0x00

/*commands to program the modes of the ADC*/
#define			AUTO_1_PROGRAM_1		0x80			//this makes the ADC to go into programming AUTO-1 mode.
#define 		AUTO_1_PROGRAM_2		0x00
// #define 		AUTO_1_SEQUENCE			0x7FFF			//this gives the sequence of the channels to be sampled.
#define 		ADC_AUTO_2_PROGRAM_1		0x91
#define			ADC_AUTO_2_PROGRAM_2		0xC0

#define			ADC_AUTO_2_PROGRAM2_1		0x93
#define			ADC_AUTO_2_PROGRAM2_2		0xC0

#define 		MAX_ADC_CHANNELS		12

/* IOCTL Commands
 * Cmd: ANIOC_ADC_MANUAL_SELECT           Arg: none
 * Cmd: ANIOC_ADC_AUTO_2_SELECT           Arg: none
 * Cmd: ANIOC_ADC_AUTO_2_PROGRAM          Arg: none
 * Cmd: ANIOC_ADC_AUTO_2_SELECT_READ      Arg: uint8_t *  -- pointer to data read from ADC 
 */
#define ANIOC_ADC_MANUAL_SELECT           _ANIOC(ANIOC_ADS7953_FIRST + 0)
#define ANIOC_ADC_AUTO_2_SELECT           _ANIOC(ANIOC_ADS7953_FIRST + 1)
#define ANIOC_ADC_AUTO_2_PROGRAM          _ANIOC(ANIOC_ADS7953_FIRST + 2)
#define ANIOC_ADC_AUTO_2_SELECT_READ      _ANIOC(ANIOC_ADS7953_FIRST + 3)

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct ads7953_config_s
{
  /* For users on SPI.
   *
   *  spi_devid : the SPI master's slave-select number
   *              for the chip, as used in SPI_SELECT(..., dev_id, ...)
   *  spi       : the SPI master device, as used in SPI_SELECT(spi, ..., ...)
   */

  FAR struct spi_dev_s *spi;
  int spi_devid;
};

typedef struct ads7953_data_config_s
{
  uint8_t channel_id: 4;      /* This will be the channel number returned in struct adc_msg_s for a conversion */
  uint16_t adc_value : 12;     /* Raw data from ADC */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: adds7953_register
 *
 * Description:
 *   Register the ads7953 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/adc0"
 *   spi - An instance of the SPI interface to use to communicate with
 *     ads7953
 *   spidev - SPI device number
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ads7953_register(FAR const char *devpath,
                     FAR struct spi_dev_s *spi, int spidev);

#endif /* __INCLUDE_NUTTX_ANALOG_ADS7953 */

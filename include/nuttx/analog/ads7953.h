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

#if defined(CONFIG_ADC_ADS7953)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* LTC1867L Configuration ***************************************************/

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

/****************************************************************************
 * Public Types
 ****************************************************************************/


// enum ltc1867l_analog_input_mode_e
// {
//   LTC1867L_UNIPOLAR = LTC1867L_CONFIG_BIT_UNI,
//   LTC1867L_BIPOLAR = 0,
// };

begin_packed_struct struct ads7953_channel_config_s
{
  uint8_t channel;                                                     /* This will be the channel number returned in struct adc_msg_s for a conversion */
//   enum ltc1867l_analog_multiplexer_config_e analog_multiplexer_config; /* Analog multiplexer configuration */
//   enum ltc1867l_analog_input_mode_e analog_inputmode;                  /* Analog input mode */
} end_packed_struct;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: ltc1867l_register
 *
 * Description:
 *   Register the LTC1867L character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/adc0"
 *   spi - An instance of the SPI interface to use to communicate with
 *     LTC1867L
 *   devno - SPI device number
 *   channel_config - A pointer to an array which holds the configuration
 *     for each sampled channel.
 *   channel_config_count - Number of channels to sample
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ads7953_register(FAR const char *devpath,
                     FAR struct spi_dev_s *spi, int spidev);

#endif /* CONFIG_ADC_LTC1867L */
#endif /* __INCLUDE_NUTTX_ANALOG_LTC1867L_H */

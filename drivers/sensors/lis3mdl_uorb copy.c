/****************************************************************************
 * drivers/sensors/lis3mdl.c
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

#include <assert.h>
#include <errno.h>
#include <debug.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <sys/param.h>

#include <nuttx/mutex.h>
#include <nuttx/nuttx.h>
#include <nuttx/kmalloc.h>
#include <nuttx/wqueue.h>
#include <nuttx/random.h>

#include <nuttx/fs/fs.h>

#include <nuttx/sensors/sensor.h>
#include <nuttx/sensors/ioctl.h>

#include <nuttx/sensors/lis3mdl.h>

#if defined(CONFIG_SPI) && defined(CONFIG_SENSORS_LIS3MDL_ORB)

/****************************************************************************
 * Private
 ****************************************************************************/

struct lis3mdl_dev_s
{
  FAR struct lis3mdl_dev_s *flink;     /* Supports a singly linked list of
                                        * drivers */
  FAR struct spi_dev_s *spi;           /* Pointer to the SPI instance */
  FAR struct lis3mdl_config_s *config; /* Pointer to the configuration
                                        * of the LIS3MDL sensor */
  uint64_t timestamp;                  /* Units is microseconds */
  struct sensor_lowerhalf_s lower;     /* The struct of lower half driver */
#if CONFIG_SENSORS_LIS3MDL_ORB_BUFFER_SIZE > 0
  struct work_s work; /* The work queue is responsible for
                       * retrieving the data from the
                       * sensor after the arrival of new
                       * data was signalled in an interrupt */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void lis3mdl_read_register(FAR struct lis3mdl_dev_s *dev,
                                  uint8_t const reg_addr,
                                  uint8_t *reg_data);
static void lis3mdl_write_register(FAR struct lis3mdl_dev_s *dev,
                                   uint8_t const reg_addr,
                                   uint8_t const reg_data);
static void lis3mdl_reset(FAR struct lis3mdl_dev_s *dev);
static void lis3mdl_read_measurement_data(FAR struct lis3mdl_dev_s *dev,
                                          FAR struct sensor_mag *data);
static void lis3mdl_read_magnetic_data(FAR struct lis3mdl_dev_s *dev,
                                       uint16_t *x_mag, uint16_t *y_mag,
                                       uint16_t *z_mag);
static void lis3mdl_read_temperature(FAR struct lis3mdl_dev_s *dev,
                                     uint16_t *temperature);
static int lis3mdl_interrupt_handler(int irq, FAR void *context, 
                                     FAR void *arg);
static int lis3mdl_activate(FAR struct sensor_lowerhalf_s *lower,
                            FAR struct file *filep, bool enable);
#if CONFIG_SENSORS_LIS3MDL_ORB_BUFFER_SIZE > 0
static void lis3mdl_worker(FAR void *arg);
#else
static int lis3mdl_fetch(FAR struct sensor_lowerhalf_s *lower,
                         FAR struct file *filep,
                         FAR char *buffer, size_t buflen);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* the lower half sensor driver operations for sensor register */

static const struct sensor_ops_s g_lis3mdl_ops =
{
        .activate = lis3mdl_activate,
        .set_interval = NULL,
        .batch = NULL,
#if CONFIG_SENSORS_LIS3MDL_ORB_BUFFER_SIZE > 0
        .fetch = NULL,
#else
        .fetch = lis3mdl_fetch,
#endif
        .control = NULL
};

/* Single linked list to store instances of drivers */

static struct lis3mdl_dev_s *g_lis3mdl_list = 0;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lis3mdl_read_register
 ****************************************************************************/

static void lis3mdl_read_register(FAR struct lis3mdl_dev_s *dev,
                                  uint8_t const reg_addr, uint8_t *reg_data)
{
  /* Lock the SPI bus so that only one device can access it at the same
   * time
   */

  SPI_LOCK(dev->spi, true);

  /* Set CS to low which selects the LIS3MDL */

  SPI_SELECT(dev->spi, dev->config->spi_devid, true);

  /* Transmit the register address from where we want to read - the MSB
   * needs to be set to indicate the read indication.
   */

  SPI_SEND(dev->spi, reg_addr | 0x80);

  /* Write an idle byte while receiving the required data */

  *reg_data = (uint8_t)(SPI_SEND(dev->spi, 0));

  /* Set CS to high which deselects the LIS3MDL */

  SPI_SELECT(dev->spi, dev->config->spi_devid, false);

  /* Unlock the SPI bus */

  SPI_LOCK(dev->spi, false);
}

/****************************************************************************
 * Name: lis3mdl_write_register
 ****************************************************************************/

static void lis3mdl_write_register(FAR struct lis3mdl_dev_s *dev,
                                   uint8_t const reg_addr,
                                   uint8_t const reg_data)
{
  /* Lock the SPI bus so that only one device can access it at the same
   * time
   */

  SPI_LOCK(dev->spi, true);

  /* Set CS to low which selects the LIS3MDL */

  SPI_SELECT(dev->spi, dev->config->spi_devid, true);

  /* Transmit the register address from where we want to read */

  SPI_SEND(dev->spi, reg_addr);

  /* Transmit the content which should be written in the register */

  SPI_SEND(dev->spi, reg_data);

  /* Set CS to high which deselects the LIS3MDL */

  SPI_SELECT(dev->spi, dev->config->spi_devid, false);

  /* Unlock the SPI bus */

  SPI_LOCK(dev->spi, false);
}

/****************************************************************************
 * Name: lis3mdl_reset
 ****************************************************************************/

static void lis3mdl_reset(FAR struct lis3mdl_dev_s *dev)
{
  lis3mdl_write_register(dev,
                         LIS3MDL_CTRL_REG_2,
                         LIS3MDL_CTRL_REG_2_SOFT_RST_BM);

  up_mdelay(100);
}

/****************************************************************************
 * Name: lis3mdl_interrupt_handler
 ****************************************************************************/

static void lis3mdl_read_measurement_data(FAR struct lis3mdl_dev_s *dev,
                                          FAR struct sensor_mag *data)
{
  /* Magnetic data */

  uint16_t x_mag = 0;
  uint16_t y_mag = 0;
  uint16_t z_mag = 0;

  lis3mdl_read_magnetic_data(dev, &x_mag, &y_mag, &z_mag);

  /* Temperature */

  uint16_t temperature = 0;

  lis3mdl_read_temperature(dev, &temperature);

  data->x = (float) (x_mag);
  data->y = (float) (y_mag);
  data->z = (float) (z_mag);
  data->temperature = (float) temperature;
  data->timestamp = dev->timestamp;

  /* Feed sensor data to entropy pool */

  add_sensor_randomness((x_mag << 16) ^ (y_mag << 10) ^ (z_mag << 2) ^
                        temperature);
}

/****************************************************************************
 * Name: lis3mdl_read_magnetic_data
 ****************************************************************************/

static void lis3mdl_read_magnetic_data(FAR struct lis3mdl_dev_s *dev,
                                       uint16_t *x_mag, uint16_t *y_mag,
                                       uint16_t *z_mag)
{
  /* Lock the SPI bus so that only one device can access it at the same
   * time
   */

  SPI_LOCK(dev->spi, true);

  /* Set CS to low which selects the LIS3MDL */
  SPI_SELECT(dev->spi, dev->config->spi_devid, true);

  /* Transmit the register address from where we want to start reading
   * 0x80 -> MSB is set -> Read Indication 0x40 -> MSB-1 (MS-Bit) is
   * set -> auto increment of address when reading multiple bytes.
   */
  SPI_SEND(dev->spi, (LIS3MDL_OUT_X_L_REG | 0x80 | 0x40)); /* RX */
  *x_mag = ((uint16_t)(SPI_SEND(dev->spi, 0)) << 0);       /* LSB */
  *x_mag |= ((uint16_t)(SPI_SEND(dev->spi, 0)) << 8);      /* MSB */

  *y_mag = ((uint16_t)(SPI_SEND(dev->spi, 0)) << 0);  /* LSB */
  *y_mag |= ((uint16_t)(SPI_SEND(dev->spi, 0)) << 8); /* MSB */

  *z_mag = ((uint16_t)(SPI_SEND(dev->spi, 0)) << 0);  /* LSB */
  *z_mag |= ((uint16_t)(SPI_SEND(dev->spi, 0)) << 8); /* MSB */

  /* Set CS to high which deselects the LIS3MDL */
  SPI_SELECT(dev->spi, dev->config->spi_devid, false);

  /* Unlock the SPI bus */

  SPI_LOCK(dev->spi, false);
}

/****************************************************************************
 * Name: lis3mdl_read_temperature
 ****************************************************************************/

static void lis3mdl_read_temperature(FAR struct lis3mdl_dev_s *dev,
                                     uint16_t *temperature)
{
  /* Lock the SPI bus so that only one device can access it at the same
   * time
   */

  SPI_LOCK(dev->spi, true);

  /* Set CS to low which selects the LIS3MDL */

  SPI_SELECT(dev->spi, dev->config->spi_devid, true);

  /* Transmit the register address from where we want to start reading
   * 0x80 -> MSB is set -> Read Indication 0x40 -> MSB-1 (MS-Bit) is
   * set -> auto increment of address when reading multiple bytes.
   */

  SPI_SEND(dev->spi, (LIS3MDL_TEMP_OUT_L_REG | 0x80 | 0x40));

  /* RX */

  *temperature = ((uint16_t)(SPI_SEND(dev->spi, 0)) << 0);  /* LSB */
  *temperature |= ((uint16_t)(SPI_SEND(dev->spi, 0)) << 8); /* MSB */

  /* Set CS to high which deselects the LIS3MDL */
  SPI_SELECT(dev->spi, dev->config->spi_devid, false);

  /* Unlock the SPI bus */

  SPI_LOCK(dev->spi, false);
}

/****************************************************************************
 * Name: lis3mdl_interrupt_handler
 ****************************************************************************/

static int lis3mdl_interrupt_handler(int irq, FAR void *context,
                                     FAR void *arg)
{
  /* This function should be called upon a rising edge on the LIS3MDL DRDY
   * pin since it signals that new data has been measured.
   */

  FAR struct lis3mdl_dev_s *priv = 0;
  int ret;

  /* Find out which LIS3MDL device caused the interrupt */

  for (priv = g_lis3mdl_list; priv && priv->config->irq != irq;
       priv = priv->flink)
  {
    DEBUGASSERT(priv != NULL);
  }

  priv->timestamp = sensor_get_timestamp();

#if CONFIG_SENSORS_LIS3MDL_ORB_BUFFER_SIZE > 0
  /* Task the worker with retrieving the latest sensor data. We should not do
   * this in a interrupt since it might take too long. Also we cannot lock
   * the SPI bus from within an interrupt.
   */

  DEBUGASSERT(priv->work.worker == NULL);
  ret = work_queue(HPWORK, &priv->work, lis3mdl_worker, priv, 0);
  if (ret < 0)
  {
    snerr("ERROR: Failed to queue work: %d\n", ret);
    return ret;
  }
#else
  priv->lower.notify_event(priv->lower.priv);
#endif
  return OK;
}

#if CONFIG_SENSORS_LIS3MDL_ORB_BUFFER_SIZE > 0
/****************************************************************************
 * Name: lis3mdl_worker
 ****************************************************************************/

static void lis3mdl_worker(FAR void *arg)
{
  struct sensor_mag temp;

  FAR struct lis3mdl_dev_s *priv = (FAR struct lis3mdl_dev_s *)(arg);
  DEBUGASSERT(priv != NULL);

  /* Read out the latest sensor data */

  lis3mdl_read_measurement_data(priv, &temp);

  /* push to upper half driver */
  priv->lower.push_event(priv->lower.priv, &temp,
                         sizeof(struct sensor_mag));
}
#else
static int lis3mdl_fetch(FAR struct sensor_lowerhalf_s *lower,
                         FAR struct file *filep,
                         FAR char *buffer, size_t buflen)
{
  FAR struct lis3mdl_dev_s *priv = container_of(lower,
                                                FAR struct lis3mdl_dev_s,
                                                lower);
  if (buflen != sizeof(struct sensor_mag))
    return 0;
  DEBUGASSERT(priv != NULL);

  /* read out the latest sensor data */
  lis3mdl_read_measurement_data(priv, (FAR struct sensor_mag *)buffer);

  return sizeof(struct sensor_mag);
}
#endif

/****************************************************************************
 * Name: lis3mdl_activate
 ****************************************************************************/

static int lis3mdl_activate(FAR struct sensor_lowerhalf_s *lower,
                            FAR struct file *filep, bool enable)
{
  FAR struct lis3mdl_dev_s *priv = container_of(lower,
                                                FAR struct lis3mdl_dev_s,
                                                lower);
  struct sensor_mag temp;

#ifdef CONFIG_DEBUG_SENSORS_INFO
  uint8_t reg_content;
  uint8_t reg_addr;
#endif

  DEBUGASSERT(priv != NULL);
  if (enable == true)
  {
    /* perform a reset */
    lis3mdl_reset(priv);

    /* Enable DRDY signal on INT 2 */

    /* Enable * - the maximum full scale mode. * Full scale = +/- 1.6 mT (16
     * Gauss).
     */

    lis3mdl_write_register(priv,
                           LIS3MDL_CTRL_REG_2,
                           LIS3MDL_CTRL_REG_2_FS_1_BM |
                               LIS3MDL_CTRL_REG_2_FS_0_BM);

    /* Enable - temperature sensor - ultra high performance mode (UMP) for X
     * and Y - fast output data rates This results in a output data rate of
     * 155 Hz for X and Y.
     */

    lis3mdl_write_register(priv,
                           LIS3MDL_CTRL_REG_1,
                           LIS3MDL_CTRL_REG_1_TEMP_EN_BM |
                               LIS3MDL_CTRL_REG_1_OM_1_BM |
                               LIS3MDL_CTRL_REG_1_OM_0_BM |
                               LIS3MDL_CTRL_REG_1_FAST_ODR_BM);

    /* Enable * - ultra high performance mode (UMP) for Z * This should result
     * to the same output data rate as for X and Y.
     */

    lis3mdl_write_register(priv,
                           LIS3MDL_CTRL_REG_4,
                           LIS3MDL_CTRL_REG_4_OMZ_1_BM |
                               LIS3MDL_CTRL_REG_4_OMZ_0_BM);

    /* Enable * - block data update for magnetic sensor data * This should
     * prevent race conditions when reading sensor data.
     */

    lis3mdl_write_register(priv,
                           LIS3MDL_CTRL_REG_5,
                           LIS3MDL_CTRL_REG_5_BDU_BM);

    /* Enable continuous conversion mode - the device starts measuring now. */

    lis3mdl_write_register(priv, LIS3MDL_CTRL_REG_3, 0);

    /* Read measurement data to ensure DRDY is low */

    lis3mdl_read_measurement_data(priv, &temp);

    /* Read back the content of all control registers for debug purposes */
#ifdef CONFIG_DEBUG_SENSORS_INFO
    reg_content = 0;
    for (reg_addr = LIS3MDL_CTRL_REG_1;
         reg_addr <= LIS3MDL_CTRL_REG_5;
         reg_addr++)
    {
      lis3mdl_read_register(priv, reg_addr, &reg_content);
      sninfo("R#%04x = %04x\n", reg_addr, reg_content);
    }

    lis3mdl_read_register(priv, LIS3MDL_STATUS_REG, &reg_content);
    sninfo("STATUS_REG = %04x\n", reg_content);
#endif
  }
  else
  {
    lis3mdl_reset(priv);
  }

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lis3mdl_register
 *
 * Description:
 *   Register the LIS3MDL MAG as a sensor device
 *
 * Input Parameters:
 *   devno   - The device number, used to bind the device path
 *             as /dev/uorb/sensor_mag_uncalN
 *   spi     - An instance of the SPI interface to use to communicate with
 *             LIS3MDL
 *   config  - configuration for the LIS3MDL driver. For details see
 *             description above.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int lis3mdl_register(int devno, FAR struct spi_dev_s *spi,
                     FAR struct lis3mdl_config_s const *config)
{
  FAR struct lis3mdl_dev_s *priv;
  int ret;

  /* Sanity check */

  DEBUGASSERT(spi != NULL);
  DEBUGASSERT(config != NULL);

  /* Initialize the LIS3MDL device structure */

  priv = (FAR struct lis3mdl_dev_s *)
      kmm_malloc(sizeof(struct lis3mdl_dev_s));
  if (priv == NULL)
  {
    snerr("ERROR: Failed to allocate instance\n");
    ret = -ENOMEM;
    goto errout;
  }

  priv->spi = spi;
  priv->config = config;
#if CONFIG_SENSORS_LIS3MDL_ORB_BUFFER_SIZE > 0
  priv->work.worker = NULL;
#endif
  priv->timestamp = 0;

  priv->lower.type = SENSOR_TYPE_MAGNETIC_FIELD;
  priv->lower.nbuffer = CONFIG_SENSORS_LIS3MDL_ORB_BUFFER_SIZE;
  priv->lower.ops = &g_lis3mdl_ops;
  priv->lower.uncalibrated = true;

  /* Setup SPI frequency and mode */

  SPI_SETFREQUENCY(spi, LIS3MDL_SPI_FREQUENCY);
  SPI_SETMODE(spi, LIS3MDL_SPI_MODE);

  /* Attach the interrupt handler */

  ret = priv->config->attach(priv->config, &lis3mdl_interrupt_handler);
  if (ret < 0)
  {
    snerr("ERROR: Failed to attach interrupt\n");
    goto errout;
  }

  /* Register the character driver */

  // ret = register_driver(devpath, &g_lis3mdl_fops, 0666, priv);
  ret = sensor_register(&priv->lower, devno);
  if (ret < 0)
  {
    snerr("ERROR: Failed to register driver: %d\n", ret);
    kmm_free(priv);
    goto errout;
  }

  /* Since we support multiple LIS3MDL devices are supported, we will need to
   * add this new instance to a list of device instances so that it can be
   * found by the interrupt handler based on the received IRQ number.
   */

  priv->flink = g_lis3mdl_list;
  g_lis3mdl_list = priv;

errout:
  return ret;
}

#endif /* CONFIG_SPI && CONFIG_SENSORS_LIS3MDL */

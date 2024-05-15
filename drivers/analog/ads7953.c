#include <nuttx/config.h>

#include <sys/types.h>
#include <sys/ioctl.h>
#include <stdio.h>
#include <string.h>
#include <fixedmath.h>
#include <stdbool.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/sensors/ioctl.h>
#include <nuttx/analog/ads7953.h>
#include <nuttx/mutex.h>

#define ADS7953_SPI_MODE       (SPIDEV_MODE0)
#define CONFIG_ADS7953_SPI_FREQUENCY    1000000

struct ads7953_dev_s
{
  FAR const struct adc_callback_s *cb;
  FAR struct spi_dev_s *spi;
  int spidev;
  unsigned int devno;
  FAR struct ads7953_channel_config_s *channel_config;
  int channel_config_count;
  mutex_t lock;
};

/* Standard character drivers */
static int ads7953_open(FAR struct file *filep);
static ssize_t ads7953_read(FAR struct file *filep, FAR char *buffer,size_t buflen);
static ssize_t ads7953_write(FAR struct file *filep, FAR const char *buffer,size_t buflen);
static int ads7953_ioctl(FAR struct file *filep, int cmd, unsigned long arg);

/* Drivers used to actually interact with the hardware (SPI) */
static void ads7953_write16(FAR struct ads7953_dev_s *priv, uint8_t *cmd);
static void ads7953_read16(FAR struct ads7953_dev_s *priv, uint8_t *cmd, uint8_t *data);

/* This is for upper level file operation to access from custom applications */
static const struct file_operations g_ads7953ops =
{
    ads7953_open,   /* open */
    NULL,           /* close */
    ads7953_read,   /* read */
    ads7953_write,  /* write */
    NULL,           /* seek not used */
    ads7953_ioctl,  /* ioctl */
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

static inline void ads7953_configspi(FAR struct spi_dev_s *spi){
    SPI_SETMODE(spi, ADS7953_SPI_MODE);
    SPI_SETBITS(spi, 8);
    SPI_HWFEATURES(spi, 0);
    SPI_SETFREQUENCY(spi, CONFIG_ADS7953_SPI_FREQUENCY);
}

/*
* @brief    function to write/send the data to SPI 
*           these functions are not called by the upper level applications
*           
*     
*/
static void ads7953_write16(FAR struct ads7953_dev_s *priv, uint8_t *cmd){
  //if preemption is enabled and IRQ is required, use enter_critical_section() function 
  SPI_LOCK(priv->spi, true);    //if thread lock then it is required, otherwise not
  ads7953_configspi(priv->spi); //configure SPI, set speed, type, HW features etc.
  SPI_SELECT(priv->spi, priv->spidev, true);  //selects the SPI to transfer data (including setting CS pin)
  SPI_EXCHANGE(priv->spi, cmd, NULL, 2);
  SPI_SELECT(priv->spi, priv->spidev, false);   //deselect the SPI after use

  SPI_LOCK(priv->spi, false);     //unlock the thread
  return;
}

static void ads7953_read16(FAR struct ads7953_dev_s *priv, uint8_t *cmd, uint8_t *data){
  
  SPI_LOCK(priv->spi, true);
  ads7953_configspi(priv->spi);
  SPI_SELECT(priv->spi, priv->spidev, true);
  SPI_EXCHANGE(priv->spi,cmd, data, 2);
  SPI_SELECT(priv->spi, priv->spidev, false);
  SPI_LOCK(priv->spi, false);
  return;
}

static int ads7953_open(FAR struct file *filep)
{
  printf("Opening the ads7953 file");
  FAR struct inode *inode = filep->f_inode;
  FAR struct ads7953_dev_s *priv = inode->i_private;
  printf("Opened the ADS7953 file \n");
  return OK;
}

static ssize_t ads7953_read(FAR struct file *filep, FAR char *buffer,
                            size_t buflen)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct ads7953_dev_s *priv = inode->i_private;
  uint8_t temp[2] = {'\0'};
  uint8_t *buf = '\0';
  temp[0] = AUTO_2_MODE2_1;
  temp[1] = AUTO_2_MODE2_2;
  ads7953_read16(priv, temp, buf);
  memcpy(buffer, buf, sizeof(buf));
  return sizeof(buf);
}

/****************************************************************************
 * Name: adxl372_write
 ****************************************************************************/

static ssize_t ads7953_write(FAR struct file *filep, FAR const char *buffer,
                        size_t buflen)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct ads7953_dev_s *priv = inode->i_private;

  ads7953_write16(priv, (int16_t )buffer); //maybe mutex is required but the actual use of it is not well understood so skip for now
}

// static ssize_t ads7953_ioctl(FAR struct file *filep, int cmd, unsigned long arg){

// }

/* 
*
*/
static int ads7953_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct ads7953_dev_s *priv = inode->i_private;
  int ret = OK;
  uint8_t temp[2] = {'\0'};
  switch (cmd)
    {
      /* switch to manual select mode arg: none*/
      case SNIOC_ADC_MANUAL_SELECT:
        {
          // FAR uint8_t *ptr = (FAR uint8_t *)((uintptr_t)arg);
          // DEBUGASSERT(ptr != NULL);
          temp[0]  = MANUAL_MODE_1;
          temp[1] = MANUAL_MODE_2;
          ads7953_write16(priv, temp);
          printf("Manual Mode set: %x \n", temp[0] << 8 | temp[1]);
        }
        break;

      /* send the ADC1 select command */
      case SNIOC_ADC_AUTO_2_SELECT:
        {
          // FAR uint8_t *ptr = (FAR uint8_t *)((uintptr_t)arg);
          // DEBUGASSERT(ptr != NULL);
          temp[0] = AUTO_2_MODE_1; 
          temp[1] = AUTO_2_MODE_2;
          ads7953_write16(priv, temp);
          printf("Auto 2 Mode set: %x \n", temp[0] << 8 | temp[1]);
        }
        break;
      
      /* send the ADC1 program command */
      case SNIOC_ADC_AUTO_2_PROGRAM:
        {
          // FAR uint8_t *ptr = (FAR uint8_t *)((uintptr_t)arg);
          // DEBUGASSERT(ptr != NULL);
          temp[0]  = ADC_AUTO_2_PROGRAM2_1;
          temp[1] = ADC_AUTO_2_PROGRAM2_2;
          ads7953_write16(priv, temp);
          printf("Auto 2 Program mode set: %x \n", temp[0] << 8 | temp[1]);
        }
        break;

      /* send the ADC1 program command */
      case SNIOC_ADC_AUTO_2_SELECT_READ:
        {
          FAR uint8_t *ptr = (FAR uint8_t *)((uintptr_t)arg);
          DEBUGASSERT(ptr != NULL);
          temp[0]  = AUTO_2_MODE2_1;
          temp[1] = AUTO_2_MODE2_2;
          ads7953_read16(priv, temp, ptr);
          printf("Auto 2 Read Mode: %x \n", temp[0] << 8 | temp[1]);
          printf("Data from read function: %d \n", *ptr);
        }
        break;
      
      
      /* Read the configuration register. Arg: uint8_t* pointer */

      // case SNIOC_READCONF:
      //   {
      //     FAR uint8_t *ptr = (FAR uint8_t *)((uintptr_t)arg);
      //     DEBUGASSERT(ptr != NULL);
      //     *ptr = adt7320_read_reg8(priv, ADT7320_CONF_REG);
      //   }
      //   break;

      /* Write to the configuration register. Arg:  uint8_t value */

      // case SNIOC_WRITECONF:
      //   adt7320_write_reg8(priv, ADT7320_CONF_REG, (uint8_t)arg);
      //   break;

      /* Report samples in Fahrenheit */

      // case SNIOC_FAHRENHEIT:
      //   priv->fahrenheit = true;
      //   sninfo("Fahrenheit\n");
      //   break;

      /* Report samples in Celsius */

      // case SNIOC_CENTIGRADE:
      //   priv->fahrenheit = false;
      //   sninfo("Celsius\n");
      //   break;

      /* Read the critical temperature register. Arg: b16_t* pointer */

      // case SNIOC_READTCRIT:
      //   {
      //     FAR b16_t *ptr = (FAR b16_t *)((uintptr_t)arg);
      //     int16_t temp_raw;
      //     DEBUGASSERT(ptr != NULL);
      //     temp_raw = adt7320_read_reg16(priv, ADT7320_TCRIT_REG);
      //     *ptr = b8tob16(temp_raw << 1);
      //   }
      //   break;

      /* Write to the critical temperature register. Arg: b16_t value */

      // case SNIOC_WRITETCRIT:
      //   adt7320_write_reg16(priv, ADT7320_TCRIT_REG,
      //                       b16tob8((b16_t)arg) >> 1);
      //   break;

      /* Read the hysteresis temperature register. Arg: b16_t* */

      // case SNIOC_READTHYS:
      //   {
      //     FAR b16_t *ptr = (FAR b16_t *)((uintptr_t)arg);
      //     uint8_t tmp;
      //     DEBUGASSERT(ptr != NULL);
      //     tmp = adt7320_read_reg8(priv, ADT7320_THYST_REG);
      //     *ptr = uitoub16(tmp);
      //   }
      //   break;

      /* Write to the hysteresis temperature register. Arg: b16_t value */

      // case SNIOC_WRITETHYS:
      //   adt7320_write_reg8(priv, ADT7320_THYST_REG, ub16toi((b16_t)arg));
      //   break;

      /* Read the low temperature register. Arg: b16_t* pointer */

      // case SNIOC_READTLOW:
      //   {
      //     FAR b16_t *ptr = (FAR b16_t *)((uintptr_t)arg);
      //     int16_t temp_raw;
      //     DEBUGASSERT(ptr != NULL);
      //     temp_raw = adt7320_read_reg16(priv, ADT7320_TLOW_REG);
      //     *ptr = b8tob16(temp_raw << 1);
      //   }
      //   break;

      /* Write to the low temperature register. Arg: b16_t value */

      // case SNIOC_WRITETLOW:
      //   adt7320_write_reg16(priv, ADT7320_TLOW_REG,
      //                       b16tob8((b16_t)arg) >> 1);
      //   break;

      /* Read the high temperature register. Arg: b16_t* pointer */

      // case SNIOC_READTHIGH:
      //   {
      //     FAR b16_t *ptr = (FAR b16_t *)((uintptr_t)arg);
      //     int16_t temp_raw;
      //     DEBUGASSERT(ptr != NULL);
      //     temp_raw = adt7320_read_reg16(priv, ADT7320_THIGH_REG);
      //     *ptr = b8tob16(temp_raw << 1);
      //   }
      //   break;

      /* Write to the high temperature register. Arg: b16_t value */

      // case SNIOC_WRITETHIGH:
      //   adt7320_write_reg16(priv, ADT7320_THIGH_REG,
      //                       b16tob8((b16_t)arg) >> 1);
      //   break;

      default:
        sninfo("Unrecognized cmd: %d\n", cmd);
        ret = -ENOTTY;
        break;
    }

  return ret;
}
/*
*  @brief    this is the first step of running any sensor
*            registers the device and ports it, creating a virtual file
*/
int ads7953_register(FAR const char *devpath,
                     FAR struct spi_dev_s *spi, int spidev)
{
  FAR struct ads7953_dev_s *priv = {'\0'};
  int ret;

  /* Sanity check */
  DEBUGASSERT(spi != NULL);

  /* Initialize the ADS7953 device structure */
  nxmutex_init(&priv->lock);
  priv = kmm_malloc(sizeof(struct ads7953_dev_s));
  if (priv == NULL)
    {
      snerr("ERROR: Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->spi        = spi;
  priv->spidev     = spidev;

  /* Register the character driver */
  ret = register_driver(devpath, &g_ads7953ops, 0666, priv);

  if (ret < 0)
    {
      snerr("ERROR: Failed to register driver: %d\n", ret);
      nxmutex_destroy(&priv->lock);
      kmm_free(priv);
    }
  return ret;
}
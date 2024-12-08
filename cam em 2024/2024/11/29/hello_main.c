/****************************************************************************
 * apps/examples/hello/hello_main.c
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
#include "gpio_definitions.h"
#include <nuttx/fs/ioctl.h>
#include <stdio.h>
#include <fcntl.h>
#include <syslog.h>
// #include <stdlib.h>
// #include <unistd.h>
// #include <errno.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int gpio_write(uint32_t pin, uint8_t mode);

void handshake()
{
  uint8_t command_in[7] = {'\0'};
  int ret, fd1, i = 0;
  
  fd1 = open("/dev/ttyS1", O_RDWR);
  if (fd1 < 0)
  {
    printf("Failed to open UART device: %d\n", errno);
    return;
  }
  else{
    printf("opened uart device for handshaking\n");
  }

  ret = read(fd1,command_in, 7);
  if(ret > 0){
    printf("data received is %s\n,command_in");
  }
  close(fd1);
  fd1 = open("/dev/ttyS1", O_RDWR);
  uint8_t data2[]={0x53,0x01,0x02,0x03,0x04,0x7e,'\0'};
  ret = write(fd1,data2, 7);
 if(ret > 0){
    printf("data sent ");
  }

  if (command_in[0] == 0x53 || command_in[1] == 0x53)
  {
    printf("Handshake successful with command: %02X %02X %02X %02X %02X %02X\n", command_in[0],  command_in[1], command_in[2], command_in[3], command_in[4], command_in[5]);
  }
  else
  {
    printf("Handshake failed, unexpected command:  %02X %02X %02X %02X %02X %02X\n", command_in[0],  command_in[1], command_in[2], command_in[3], command_in[4], command_in[5]);
  }

  close(fd1);
}

int main(int argc, FAR char *argv[])
{
  printf("\n************************* S2S camera mission turned on ***************\n");

  uint32_t counter = 0, counter2 = 0;
  uint8_t fd, ret, data;
  uint8_t data1[27500] = {'\0'}, data2[35600] = {'\0'};

  printf("**************************** Starting handshake sequence ********************\n");
  handshake(); // Uncomment if handshake is needed
  gpio_write(GPIO_OCP_EN, false);

  fd = open("/dev/ttyS3", O_RDWR);
  ioctl(fd, TCFLSH, 2);
  ioctl(fd, TCDRN, NULL);
  printf("Flushed tx rx buffer\n");
  close(fd);

  sleep(1);

  gpio_write(GPIO_OCP_EN, true);
  sleep(1);

  printf("Command verified\nEnabling OCP\n");

  fd = open("/dev/ttyS3", O_RDONLY);
  while (1)
  {
    ret = read(fd, &data, 1);
    data1[counter] = data;

    if (counter > 0 && data1[counter - 1] == 0xff && data1[counter] == 0xd9)
    {
      printf(" %02x %02x\n", data1[counter - 1], data1[counter]);
      break;
    }
    counter++;
  }
  close(fd);
  printf("\nTotal size of data received : %d \n\n", counter);

  printf("\n---------------------------------------------------------------------------------------------------------------------\n");
  printf("--------->RGB photo\n");
  counter2 = 0;

  fd = open("/dev/ttyS2", O_RDONLY);
  while (1)
  {
    ret = read(fd, &data, 1);
    data2[counter2] = data;

    if (counter2 > 0 && data2[counter2 - 1] == 0xff && data2[counter2] == 0xd9)
    {
      printf(" %02x %02x\n", data2[counter2 - 1], data2[counter2]);
      break;
    }
    counter2++;
  }
  close(fd);

  printf("\n******************* Disabling OCP ****************\n");
  gpio_write(GPIO_OCP_EN, false);

  printf("\nTotal size of data received : %d \n\n", counter2);

  for (int16_t i = 0; i <= counter; i++)
  {
    printf("%02x ", data1[i]);
  }
  printf("\n \n------------------------\nData2 : %d \n\n", counter2);

  for (int16_t i = 0; i <= counter2; i++)
  {
    printf("%02x ", data2[i]);
  }

  // printf("\n----------------------------------------Image captured and saving-------------------------------------------\n");
  //write to file
  struct file file_pointer, file_pointer2;
  fd = file_open(&file_pointer,"/mnt/fs/sfm/mtd_mission/cam_nir.txt", O_CREAT | O_WRONLY | O_APPEND);
  ssize_t bytes_written = file_write(&file_pointer, data1, counter + 1);
  printf("\nThe total bytes of nir image written to mtd_mission/cam_nir.txt is %d\n",bytes_written);
  file_syncfs(&file_pointer);
  file_close(&file_pointer);
  fd =-9;
  fd = file_open(&file_pointer2,"/mnt/fs/sfm/mtd_mission/cam_rgb.txt", O_CREAT | O_WRONLY | O_APPEND);
  bytes_written = file_write(&file_pointer2, data2, counter2 + 1);
  printf("\nThe total bytes of nir image written to mtd_mission/cam_rgb.txt is %d\n",bytes_written);
  file_syncfs(&file_pointer2);
  file_close(&file_pointer2);

  printf("\n----------------------------------------Saved to flash memory-------------------------------------------\n");
  fd = -1;
  uint8_t x[]={0xff,0xd9,'\0'};
  // fd = open("/dev/ttyS1", O_RDWR);
  // int ret2;
  // if(ret2 >= 0){
  //   printf("0xff 0xd9 sent to OBC");
  // }
  // close(fd);
  sleep(160000);

  return 0;
}


int gpio_write(uint32_t pin, uint8_t mode)
{
  gpio_config_s gpio_numval;
  int fd = open(ETX_LED_DRIVER_PATH, O_WRONLY);
  if (fd < 0)
  {
    syslog(LOG_ERR, "Error opening %s for GPIO WRITE...", ETX_LED_DRIVER_PATH);
    return -1;
  }
  gpio_numval.gpio_num = pin;
  gpio_numval.gpio_val = mode;
  if (gpio_numval.gpio_val > 1 || gpio_numval.gpio_num < 0)
  {
    syslog(LOG_ERR, "Undefined GPIO pin or set mode selected...\n");
    close(fd);
    return -2;
  }
  int ret = write(fd, (const void *)&gpio_numval, sizeof(gpio_config_s));
  close(fd);
  if (ret < 0)
  {
    syslog(LOG_ERR, "Unable to write to gpio pin...\n");
  }
  return ret;
}

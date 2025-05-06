
#include <nuttx/config.h>

#include <stdbool.h>
#include <stdio.h>
#include <syslog.h>
#include <debug.h>
#include <errno.h>
#include <string.h>

#include <nuttx/spi/spi.h>
#include <stm32wl5_spi.h>
#include <nuttx/board.h>
#include <nuttx/fs/fs.h>
#include <nuttx/kmalloc.h>

int stm32_bringup(void){
    int i=0;
    while(i<10){
        syslog(LOG_DEBUG, "BRIng up");
        // printf("testing...\n");
        i++;

    }
     
    return OK;
  }
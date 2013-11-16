/* Copyright (c) 2012 Bob Cousins bobcousins42@googlemail.com       */
/* All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in
     the documentation and/or other materials provided with the
     distribution.
   * Neither the name of the copyright holders nor the names of
     contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.
*/

/* --------------------------------------------------------------------------

  Adding a new device, e.g. "mydev"

  1. write functions mydev_init, mydev_putchar, mydev_getchar as needed
  2. include "mydev.h" in this file
  3. put device entry into Devices table (below)

  That's it!

  open the device with :

    LW_FILE *my_file = lw_fopen ("mydev", "rw");

  then use LW_IO

    lw_fprintf (my_file, "Hello mydev world\n");

---------------------------------------------------------------------------*/



#include <inttypes.h>
#include <errno.h>
#include "stdbool.h"
#include "string.h"

#ifndef _CROSSWORKS
#include <sys/unistd.h>
#endif

// lib_hal
#include "hal_uart.h"
#ifdef HAVE_USB_SERIAL
#include "usb_serial.h"
#endif

// lib_r2c2
#include "LiquidCrystal.h"

#include "lw_syscalls.h"
#include "lw_io.h"
#include "lw_ioctl.h"

extern LW_FILE file_table [];


#ifndef EBADF
#define     EBADF 9
#endif

#ifndef EINVAL
#define EINVAL  22
#endif

// this is a mini /dev
typedef struct {
  char *name;

  uint16_t  dev_major;
  uint16_t  dev_minor;
} tDeviceDesc;

// registered drivers
// index by dev_major?
typedef struct {
  uint16_t  dev_major;

  void (*dev_init) (int dev_num);
  // open
  // close
  void (*dev_putc)    (int dev_num, char c);      //write
  char (*dev_getc)    (int dev_num);        //read
  int (*dev_rx_avail) (int dev_num);
//  int (*dev_ioctl) (int cmd, argList args);
} tDriverDesc;

// DEVICES TABLE
// index by device number, not file number
static tDeviceDesc Devices[] = {
  
#if CFG_HAL_USE_UART0 == TRUE
  {"uart0", 0, 0}, 
#endif

#if CFG_HAL_USE_UART1 == TRUE
  {"uart1", 0, 1}, 
#endif

//TODO: 
#if CFG_HAL_USE_UART2 == TRUE
  {"uart2", 0, 2}, 
#endif

#if CFG_HAL_USE_UART3 == TRUE
  {"uart3", 0, 3},
#endif

#ifdef HAVE_USB_SERIAL   
  {"usbser", 1, 0}, // USB serial
#endif

  {"lcd", 2, 0} 
};

#define NUM_DEVICES sizeof(Devices) / sizeof(tDeviceDesc)

// TODO: dev_minor
static tDriverDesc DriverTable[] = {
  
  {0, hal_uart_init, hal_uart_send, hal_uart_receive, hal_uart_data_available}, 

#ifdef HAVE_USB_SERIAL   
  {1, usb_serial_init, usb_serial_writechar, usb_serial_popchar, usb_serial_rxchars}, // USB serial
#endif

  {2, lcd_initialise, lcd_writechar, NULL, NULL} 
};

#define NUM_DRIVERS sizeof(DriverTable) / sizeof(tDriverDesc)


//
static void dev_write_block (tDriverDesc *pDriver, int dev_num, const char *ptr, int len)
{
  int j;
  for (j=0; j < len; j++)
    pDriver->dev_putc (dev_num, *ptr++);
}

static void dev_read_block (tDriverDesc *pDriver, int dev_num, char *ptr, int len)
{
  int j;
  for (j=0; j < len; j++)
  {
    *ptr = pDriver->dev_getc (dev_num);
    ptr++;
  }
}

// actually init drivers
void _sys_init_devices(void)
{
  int j;
  for (j=0; j < NUM_DRIVERS; j++)
    if (DriverTable[j].dev_init != NULL)
      DriverTable[j].dev_init(0);   //TODO: dev num?
}

// ---------------------------------------------------------------------------
// File/device functions
// ---------------------------------------------------------------------------

int _open (const char *name, int flags, int mode)
{
  bool found;
  int dev_num;
  int handle;

  found = false;
  for (dev_num=0 ; dev_num < NUM_DEVICES ; dev_num++ )
  {
    if (strcmp (Devices[dev_num].name, name) == 0)
    {
      found = true;
      break;
    }
  }

  if (!found)
    return -1;

  // find next free
  found = false;
  for (handle = 0; handle < MAX_FILES; handle++)
  {
    if (!file_table[handle].in_use)
    {
      found = true;
      break;
    }
  }

  if (found)
  {
    file_table[handle].in_use = 1;
    file_table[handle].dev_major = Devices[dev_num].dev_major;
    file_table[handle].dev_minor = Devices[dev_num].dev_minor;
    file_table[handle].handle = handle;
    file_table[handle].flags = flags;
    file_table[handle].mode = mode;

    return handle;
  }
  else
    return -1;
}

int _close(int file) 
{
  return -1;
}

/*
 read
 Read characters from a file. `libc' subroutines will use this system routine for input from all files, including stdin
 Returns -1 on error or blocks until the number of characters have been read.
 */
int _read(int file, char *ptr, int len) 
{
  int dev_num;

  if (len == 0)
    return 0;
       
  if (file < MAX_FILES)
  {
    dev_num = file_table [file].dev_major;
    dev_read_block (&DriverTable[dev_num], file_table [file].dev_minor, ptr, len);
    return len;
  }
  else
  {
    errno = EBADF;
    return -1;
  }
}

/*
 write
 Write characters to a file. `libc' subroutines will use this system routine for output to all files, including stdout
 Returns -1 on error or number of bytes sent
 */
int _write(int file, const char *ptr, int len) 
{
  // write to a file
  int dev_num;
   
  if (file < MAX_FILES)
  {
    dev_num = file_table [file].dev_major;
    dev_write_block (&DriverTable[dev_num], file_table [file].dev_minor, ptr, len);
    return len;
  }
  else
  {
    errno = EBADF;
    return -1;
  }
}

int _ioctl(int file, int cmd, va_list args) 
{
  int dev_num;
  int result;
   
  if (file < MAX_FILES)
  {
    dev_num = file_table [file].dev_major;

    switch (cmd)
    {
      case LW_FIONREAD:
      {
        int *pNum = va_arg (args, int *);

        if (pNum != NULL)
          *pNum = DriverTable[dev_num].dev_rx_avail(file_table [file].dev_minor);
         
        result = 0;
        //errno = 0;
      }
      break;

      default:
        errno = EINVAL;
        result = -1;
    }
  }
  else
  {
    errno = EBADF;
    result = -1;
  }

  return result;

}

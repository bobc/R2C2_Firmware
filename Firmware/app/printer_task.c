/* Copyright (C) 2009-2010 Michael Moon aka Triffid_Hunter   */
/* Copyright (c) 2011 Jorge Pinto - casainho@gmail.com       */
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

#include <stdint.h>
#include <stdlib.h>

#include "rtos_api.h"

// HAL
#include "buzzer.h"
#include "spi.h"
#include "sys_util.h"
#include "timer.h"
#include "hal_uart.h"

#ifdef HAVE_USB_SERIAL
#include "usb_serial.h"
#endif

// lib_r2c2
#include "debug.h"
#include "lw_syscalls.h"

// lib_FatFs
#ifdef HAVE_FILESYSTEM
#include "ff.h"
#include "sdcard.h"
#endif

// app
#include "gcode_parse.h"
#include "pin_control.h"
#include "app_config.h"
#include "temp.h"
#include "temp_controller.h"
#include "planner.h"
#include "stepper.h"
#include "motor_current_limit.h"

// config
#include "app_config.h"
#include "config_pins.h"

// tasks
#include "printer_task.h"
#include "eth_shell_task.h"
#include "usb_shell_task.h"
#include "uart_shell_task.h"
#include "gcode_task.h"
#include "ui_task.h"

///
#ifdef HAVE_FILESYSTEM
FATFS   fs;       /* Work area (file system object) for logical drive */
#endif

volatile uint16_t activity_timer = 0;


//
// printer task
static  long timer1 = 0;

// app_SysTick will be called every milli-second
void app_SysTick(void)
{
#ifdef HAVE_FILESYSTEM
  static uint8_t counter = 0;

  /* 100ms tick for SDCard ***********************************************/
  counter++;
  if (counter > 99)
  {
    MMC_disk_timerproc();
    counter = 0;
  }
  /***********************************************************************/
#endif
}

//
void aux_init (uint8_t index)
{
    // set output pin
    set_pin_mode (config.aux_output[index].pin_output, OUTPUT);
    write_pin (config.aux_output[index].pin_output, DISABLE);
}


//
static void machine_init(void)
{
  int axis, j;

  // Note: the order of initialization is important

  for (j=0; j < CFG_MAX_AUX_OUTPUTS; j++)
    aux_init (j);


  /* Heated Bed */
  ctc_init_channel (&config.heated_bed_ctc);

  /* Extruders */
  for (j=0; j < CFG_MAX_EXTRUDERS; j++)
    ctc_init_channel (&config.extruder_ctc[j]);



  ctc_init();

  temp_init();

  /* setup stepper axes */
  
  //OPTION: reset control
  set_pin_mode (config.pin_all_steppers_reset, OUTPUT);
  write_pin (config.pin_all_steppers_reset, DISABLE); /* Disable reset state for all stepper motors */

  // Option
  if (config.have_digipot)
    motor_current_init ();  
  
  for (axis = 0; axis < CFG_MAX_AXES; axis++)
  {
    if (config.axis [axis].is_configured)
    {
      unsigned motor = config.axis [axis].motor_index;

      if (motor != -1)
      {
        set_pin_mode (config.motor_driver [motor].pin_step, OUTPUT);
        set_pin_mode (config.motor_driver [motor].pin_dir, OUTPUT);
        set_pin_mode (config.motor_driver [motor].pin_enable, OUTPUT);
      
        set_pin_mode (config.axis [axis].pin_min_limit, INPUT);
        set_pin_mode (config.axis [axis].pin_max_limit, INPUT);
    
        // could be shared?
        axis_enable(axis);

        if (config.have_digipot)
          motor_current_set (axis, 0.25);
      }
    }
  }
}



static void check_boot_request (void)
{
  if (digital_read (BOOT_SW_PORT, _BV(BOOT_SW_PIN_NUMBER)) == 0)
  {
    sys_reboot();
  }
}

static void PrinterInit (void)
{
#ifdef HAVE_FILESYSTEM
  FRESULT res;
#endif

//TODO: what are the order of dependencies here? 
  app_config_set_defaults();	// <-- sets default IO pins - buzzer, lcd, ?

#ifdef HAVE_FILESYSTEM
  /* Register a work area for logical drive 0 */
  res = f_mount(0, &fs);
  if (res)
    debug("Err mount fs\n");
  else  
  {
    // read_config will use SPI and a message output (control interface or debug)	//TODO?
    app_config_read(); // <-- sets IO pins - buzzer, lcd, ?
    
    // re-init if changed?
    //!    buzzer_init (config.buzzer_pin);
  }
#endif

  // init devices?

  // set up motors etc
  machine_init();

  //TODO: CTC


}


static  tShellParams uart_shell_params;

void printer_task_init ( void *pvParameters )
{

  PrinterInit();

  // -- init complete, can now start other tasks --
  // GCode engine
  lw_TaskCreate( gcode_task_init, gcode_task_poll,         "Gcode", 512, ( void * ) NULL, LWR_IDLE_PRIORITY, NULL );

#ifdef HAVE_USB_SERIAL
  // GCode control interfaces
  lw_TaskCreate( usb_shell_task_init, usb_shell_task_poll, "USBSh", 128, ( void * ) NULL, LWR_IDLE_PRIORITY, NULL );
#endif

#ifdef HAVE_ETHERNET
  // option
  lw_TaskCreate( eth_shell_task_init, eth_shell_task_poll, "EthSh", 128, ( void * ) NULL, LWR_IDLE_PRIORITY, NULL );
#endif

#ifdef CFG_APP_USE_UART_SHELL
  // TODO: option, uart number
  // Start a Gcode shell on UART
  uart_shell_params.in_file = lw_fopen (CFG_APP_UART_SHELL_NAME, "rw"); 
  uart_shell_params.out_file = uart_shell_params.in_file;
  lw_TaskCreate( uart_task_init, uart_task_poll,    "UartSh", 128, ( void * ) &uart_shell_params, LWR_IDLE_PRIORITY, NULL );
#endif

#ifdef CFG_APP_USE_UI
  // option
  // start up user interface
  lw_TaskCreate( ui_task_init, ui_task_poll,        "UiTask", 256, ( void * ) NULL, LWR_IDLE_PRIORITY, NULL );
#endif

#ifdef HAVE_FILESYSTEM
  // now do GCode startup
  exec_gcode_file ("autoexec.g");
#endif

  // -- all startup done, signal readiness
#ifdef CFG_APP_HAVE_BUZZER
  buzzer_play(1500, 100); /* low beep */
  buzzer_wait();
  buzzer_play(2500, 200); /* high beep */
#endif
}

#define DELAY1 100
void printer_task_poll( void *pvParameters )
{
    /* Do every 100ms */
    if (timer1 < millis())
    {
      timer1 = millis() + DELAY1;

      //TODO: there are two types of timeout, 
      // 1: disable steppers when idle to avoid nuisance noise
      // 2: safe power off (steppers, heaters) after being idle for a while, in case machine is unattended and
      //    host has stopped without clean up, or user has just forgot to turn things off.
       
      /* If there is no activity during 30 seconds, power off the machine */
      if (activity_timer > (30 * 1000/DELAY1))
      {
        atx_power_off();
      }
      else
      {
        activity_timer++;
      }
    }

#ifdef CFG_APP_USE_BOOT_BUTTON
    // OPTION: enter bootloader on "Boot" button
    check_boot_request();
#endif

}

// not used by LW_RTOS
static void PrinterTask( void *pvParameters )
{
    // TASK INIT
    printer_task_init ( pvParameters );
  
    // TASK BODY
    for( ;; )
    {
        printer_task_poll( pvParameters );
    }
}


/*
  required start sequence:
  //TODO: where to read autoexec.g, set config defaults

  1.  create PrinterTask [main]

  2.  start scheduler    [main]
       [FreeRTOS: system tick is now running] 

  3.  init buzzer [no config, no debug IO, requires timer]

  4.  init USB CDC-serial [ no config ]

  5.  read_config files from SD [requires timer, debug IO]

  6.  can now initialize peripherals: steppers, CTC, ADC

  7.  start temperature monitoring [requires timer, adc]

  8.  start the other tasks [requires above stuff]

  9.  system is now ready to accept general GCode commands
*/



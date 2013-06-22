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

#include "rtos_api.h"

/* lib_HAL */
#include "adc.h"
#include "buzzer.h"
#include "spi.h"
#include "sys_util.h"
#include "timer.h"
#include "uart.h"

/* lib_r2c2 */
#include "debug.h"
#include "lw_io.h"
#include "lw_syscalls.h"
#include "soundplay.h"

// app
#include "printer_task.h"

// board config
#include "config_pins.h"

//TODO:

#ifdef _DEBUG
#define DBG_INIT()   uart_init()
#define DBGF(s)   	 uart3_writestr(s)
#else
#define DBG_INIT()
#define DBGF(s)
#endif


/**********************************************************************/


void fatal_error (void)
{
  for( ;; )
  {
    buzzer_play_sync (FREQ_B4, 1000);
    buzzer_play_sync (FREQ_A4, 1000);
  }
}

#ifdef USE_FREERTOS
/**********************************************************************/
/* Called from every tick interrupt */
void vApplicationTickHook( void )
{
  static unsigned long ulTicksSinceLastDisplay = 0;

  ulTicksSinceLastDisplay++;

  timer_SysTick(); //TODO ?
}

/**********************************************************************/
void vApplicationStackOverflowHook( xTaskHandle *pxTask, signed char *pcTaskName )
{
  /* This function will get called if a task overflows its stack. */

  ( void ) pxTask;
  ( void ) pcTaskName;

  //DBGF ("stkov\n");

  fatal_error();
}

#else
#endif

void hal_init(void)
{
  sys_initialise();

#if !defined(USE_FREERTOS)
  timer_init(); // start millisecond timers/callback
#endif

  adc_init();

  // initialise some drivers useful for debugging
  buzzer_init (PinDef (BUZZER_PORT, BUZZER_PIN_NUMBER,0,0) );	// [hal: requires io pins]
}

void lib_printer_init (void)
{
  tPinDef sd_spi_sck  = SD_SPI_SCK ;
  tPinDef sd_spi_mosi = SD_SPI_MOSI; 
  tPinDef sd_spi_miso = SD_SPI_MISO; 
  tPinDef sd_spi_ssel = SD_SPI_SSEL;

  // initialize low-level USB serial and UART drivers [hal]
  _sys_init_devices();

  // open standard files
  lw_initialise();

  dbg_init();	// [requires io pins (uart)?]
  
  // if filesystem?
  /* initialize SPI for SDCard */
  spi_configure (sd_spi_sck, sd_spi_mosi, sd_spi_miso, sd_spi_ssel);
  spi_init(SD_SPI_CHANNEL);
}

/**********************************************************************
 * @brief	Main sub-routine
 **********************************************************************/
int main(void)
{
  LW_RTOS_RESULT res;

  hal_init();

  //TODO	
  DBG_INIT();
  DBGF ("init\n");

  lib_printer_init();

  /* Create the main system task. 
  *  NB if using FreeRTOS: our system timer tick is called from FreeRTOS timer tick, which only runs after scheduler has started.
  *  Therefore, we start only PrinterTask to do initialisation which requires timer, namely the FatFs/SD code.
  */
  //TODO: Check stack usage
  
  res = lw_TaskCreate (printer_task_init, printer_task_poll,  "Print", 512, ( void * ) NULL, LWR_IDLE_PRIORITY, NULL );
  if (res != LWR_OK)
    debug ("error starting PrinterTask\n");

  /* Start the scheduler. */
  lw_TaskScheduler();
  
  /* should never get here */
  DBGF ("main:err\n");
  fatal_error();
  while(1) ;
}

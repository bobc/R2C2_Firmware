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

#include "hal_uart.h"

#include "rtos_api.h"

#include "gcode_parse.h"
#include "gcode_task.h"
#include "eth_shell_task.h"

#include "enc28j60.h"

#define DBG uart_writestr

static volatile tLineBuffer LineBuf;
static tGcodeInputMsg GcodeInputMsg;

void eth_shell_task_init ( void *pvParameters )
{
    // TASK INIT
    //NetInit();

    GcodeInputMsg.pLineBuf = &LineBuf;

    // say hi to host
    //serial_writestr("Start\r\nOK\r\n");
}

void eth_shell_task_poll ( void *pvParameters )
{
    uint8_t c;
    eParseResult parse_result;

    // no-op

}

void EthShellTask( void *pvParameters )
{
    (void) pvParameters; /* Just to prevent compiler warnings about the unused parameter. */

    eth_shell_task_init (pvParameters);

    // TASK BODY

    // process received data
    for( ;; )
    {
        eth_shell_task_poll (pvParameters);
    }
}


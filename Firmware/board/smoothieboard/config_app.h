/* Copyright (c) 2012 Bob Cousins bobcousins42@googlemail.com              */
/* **************************************************************************
   All rights reserved.

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
****************************************************************************/
// **************************************************************************
// Description:
//
// **************************************************************************

#ifndef _CONFIG_APP_H
#define _CONFIG_APP_H

// --------------------------------------------------------------------------
// Includes
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Defines
// --------------------------------------------------------------------------


// number of stepper drivers
#define MAX_AXES        4

// defines number of extruders
#define MAX_EXTRUDERS   1

#define MAX_BUTTONS     10


// TODO: sort out use of max number vs actual number configured
#define NUM_AXES    MAX_AXES

// axis map for 3D printer with 1 extruder for compatibility with existing code
#define X_AXIS      0
#define Y_AXIS      1
#define Z_AXIS      2
#define E_AXIS      3

// --------------------------------------------------------------------------
// firmware build options
// --------------------------------------------------------------------------

//#define USE_FREERTOS
#define F_CPU 100000000 /* 100MHz */

//#define CFG_APP_USE_BOOT_BUTTON

//#define CFG_APP_USE_UI

//#define CFG_APP_USE_UART_SHELL
#define CFG_APP_UART_SHELL_NAME "uart0"

//#define CFG_APP_HAVE_BUZZER

// --------------------------------------------------------------------------
//
// --------------------------------------------------------------------------

#endif // _CONFIG_APP_H

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

// defines number of stepper motor channels (including extruders)
#define CFG_MAX_MOTORS          6

// defines number of coordinate axes (including extruders)
#define CFG_MAX_AXES            6

// defines number of movement axes (not extruders)
#define CFG_MAX_MOTION_AXES     3

// defines number of extruders
#define CFG_MAX_EXTRUDERS       3

// defines number of temperature sensors
#define CFG_MAX_SENSORS         4

// defines number of aux outputs
#define CFG_MAX_AUX_OUTPUTS     2

// defines number of buttons (external Control Panel hardware)
#define CFG_MAX_BUTTONS         10


// TODO: sort out use of max number vs actual number configured
//#define NUM_AXES                CFG_MAX_AXES


// --------------------------------------------------------------------------
// Config defaults
// --------------------------------------------------------------------------

#define CFG_MACHINE_MODEL       0

#define CFG_ACCELERATION        750.0
#define CFG_JUNCTION_DEVIATION  0.05

#define CFG_AUTO_POWER_OFF_TIME     30
#define CFG_DEBUG_FLAGS             0
#define CFG_STEP_LED_FLASH_METHOD   0
#define CFG_BEEP_ON_EVENTS          0

#define CFG_STEPS_PER_MM_X      87.58
#define CFG_STEPS_PER_MM_Y      87.58
#define CFG_STEPS_PER_MM_Z      2560
#define CFG_STEPS_PER_MM_E      3360

#define CFG_MAX_FEEDRATE_X      30000     // mm/min
#define CFG_MAX_FEEDRATE_Y      30000     // mm/min
#define CFG_MAX_FEEDRATE_Z      120       // mm/min
#define CFG_MAX_FEEDRATE_E      6000      // mm/min

#define CFG_SEARCH_FEEDRATE_X   120
#define CFG_SEARCH_FEEDRATE_Y   120
#define CFG_SEARCH_FEEDRATE_Z   60
#define CFG_SEARCH_FEEDRATE_E   600

#define CFG_HOMING_FEEDRATE_X   2000
#define CFG_HOMING_FEEDRATE_Y   2000
#define CFG_HOMING_FEEDRATE_Z   60

#define CFG_HOME_DIRECTION_X    -1
#define CFG_HOME_DIRECTION_Y    1
#define CFG_HOME_DIRECTION_Z    -1

// THE COORDINATES TO SET WHEN HOME FOUND
#define CFG_HOME_POS_X          -135
#define CFG_HOME_POS_Y          115
#define CFG_HOME_POS_Z          0

#define CFG_PRINTING_VOL_X      270
#define CFG_PRINTING_VOL_Y      205
#define CFG_PRINTING_VOL_Z      210

#define CFG_WAIT_ON_TEMP        1
#define CFG_ENABLE_EXTRUDER_1   1

// --------------------------------------------------------------------------
// 
// --------------------------------------------------------------------------

// axis numbers 
// motion axes must start at 0 and be < extruders

// for 3D printer with 3 extruders 
#define X_AXIS      0
#define Y_AXIS      1
#define Z_AXIS      2

#define E_AXIS      3
#define E1_AXIS     4
#define E2_AXIS     5

// TODO: run time config ? XYZ ABC UVW E<n> etc
// XYZ ABC UVW
// XYZ E6
// tool definitions

#define X_MOTORS      _BV(0)
#define Y_MOTORS      _BV(1)
#define Z_MOTORS      _BV(2)

#define E0_MOTORS     _BV(3)
#define E1_MOTORS     _BV(4)
#define E2_MOTORS     _BV(5)



// --------------------------------------------------------------------------
// firmware build options
// --------------------------------------------------------------------------

#define CFG_MAX_DUTY_CYCLE 255

//#define CFG_APP_USE_BOOT_BUTTON

//#define CFG_APP_USE_UI

#define CFG_APP_USE_UART_SHELL
#define CFG_APP_UART_SHELL_NAME "uart0"

//#define CFG_APP_HAVE_BUZZER
//#define USE_FREERTOS




#endif // _CONFIG_APP_H

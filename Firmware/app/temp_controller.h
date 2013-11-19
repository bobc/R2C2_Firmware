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

#ifndef _TEMP_CONTROLLER_H
#define _TEMP_CONTROLLER_H

// --------------------------------------------------------------------------
// Includes
// --------------------------------------------------------------------------

#include "app_config.h"

// --------------------------------------------------------------------------
// Defines
// --------------------------------------------------------------------------

// number of heatbeds + extruders
#define NUMBER_OF_CTCS    4   

#define CTC_HEATBED_0     0

#define CTC_EXTRUDER_0    1

#define CTC_EXTRUDER_1    2
#define CTC_EXTRUDER_2    3

// --------------------------------------------------------------------------
// Types
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Public Variables
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Public functions
// --------------------------------------------------------------------------

void ctc_init ();

void ctc_init_channel (tCtcSettings *ctc_config);

// set target temperature
void ctc_set_target_temp (uint8_t ctc_number, uint16_t temperature );

// get current temp
uint16_t ctc_get_current_temp (uint8_t ctc_number);

// return target temperature
uint16_t ctc_get_target_temp (uint8_t ctc_number);

// true if last read temp is close to target temp, false otherwise
uint8_t ctc_temp_achieved (uint8_t ctc_number);

//TODO: and update heater with PID

// --------------------------------------------------------------------------
//
// --------------------------------------------------------------------------

#endif // _TEMP_CONTROLLER_H


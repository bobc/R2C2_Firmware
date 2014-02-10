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

#ifndef _PIN_CONTROL_H
#define _PIN_CONTROL_H

#include "app_config.h"

// ==========================================================================
// IO functions
// TODO move to separate module
// ==========================================================================


void axis_enable    (unsigned axis);
void axis_disable   (unsigned axis);
void axis_step      (unsigned axis);
void axis_unstep    (unsigned axis);
void axis_set_direction (unsigned axis, unsigned dir);
bool axis_min (unsigned axis);

void enable_all_axes (void);
void disable_all_axes (void);

// Setting step outputs happens in time critical interrupts
// these macros can be defined to faster/direct versions if necessary

#define x_step_fast()   axis_step(X_AXIS)
#define x_unstep_fast() axis_unstep(X_AXIS)

#define y_step_fast()   axis_step(Y_AXIS)
#define y_unstep_fast() axis_unstep(Y_AXIS)

#define z_step_fast()   axis_step(Z_AXIS)
#define z_unstep_fast() axis_unstep(Z_AXIS)

#define e_step_fast()   axis_step(E_AXIS)
#define e_unstep_fast() axis_unstep(E_AXIS)


// TODO: aux outputs
#define aux_output_on(index)     write_pin(config.aux_output[index].pin, ENABLE);
#define aux_output_off(index)    write_pin(config.aux_output[index].pin, DISABLE);


#define extruder_fan_on()   aux_output_on(0)  
#define extruder_fan_off()  aux_output_off(0)    


// other functions
//  for ATX power supply?
void atx_power_on();
void atx_power_off();


#endif
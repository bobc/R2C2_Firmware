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
// **************************************************************************
// Description:
//
// **************************************************************************

/*
    cuurent control
        digipot
            mcp4451
                i2c
*/

// --------------------------------------------------------------------------
// Includes
// --------------------------------------------------------------------------

//
#include "mcp4451.h"
#include "motor_current_limit.h"

#include "app_config.h"
#include "config_pins.h"

// --------------------------------------------------------------------------
// Externals
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Local defines
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Types
// --------------------------------------------------------------------------
typedef void (*digipot_init_t) (tPinDef scl_pin, tPinDef sda_pin);

typedef void (*digipot_set_pot_t) (int pot_num, int wiper_val);
// --------------------------------------------------------------------------
// Variables
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Public Variables
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Private Variables
// --------------------------------------------------------------------------

struct {
  float conversion_factor;
  digipot_init_t    init;
  digipot_set_pot_t set_pot;

} digipot_driver

= { 113.33, mcp4451_init, mcp4451_set_pot  };

static float current_limit [MAX_AXES];

// --------------------------------------------------------------------------
// Function prototypes
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Private functions
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Public functions
// --------------------------------------------------------------------------

void motor_current_init (void)
{
  //digipot address, number?

    digipot_driver.init (config.digipot_i2c_scl, config.digipot_i2c_sda );
//    mcp4451_init (config.digipot_i2c_scl, config.digipot_i2c_sda );
}


// --------------------------------------------------------------------------
//! @brief Set the current limit for a motor
//! @param[in]  motor_num the motor number (0 to n-1)
//! @param[in]  current   the current limit (Amps)
// --------------------------------------------------------------------------
void motor_current_set (int motor_num, float current)
{
  int wiper = current * digipot_driver.conversion_factor;
    
  current_limit [motor_num] = current;

  digipot_driver.set_pot (motor_num, wiper);
}

float motor_current_get (int motor_num)
{
    // get pot?
    // convert to current
    
    return current_limit [motor_num];
}

// --------------------------------------------------------------------------
//
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
//! @brief
//! @param[in]
//! @param[out]
//! @return
// --------------------------------------------------------------------------


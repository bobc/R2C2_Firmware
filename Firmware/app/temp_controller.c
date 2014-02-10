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

// --------------------------------------------------------------------------
// Includes
// --------------------------------------------------------------------------


// hal
#include "hal_adc.h"
#include "hal_pwm.h"
#include "ios.h"
#include "timer.h"

// app
#include "pin_control.h"
#include "stepper.h"    // TODO: steptimeout
#include "thermistor_tables.h"
#include "temp.h"
#include "temp_controller.h"
#include "printer_task.h"

// config
#include "app_config.h"


static tTimer       ctcTimer;
static int          num_ctcs;
static tCtcSettings *ctc_configs [NUMBER_OF_CTCS]; // max ctcs
static uint16_t     target_temp  [NUMBER_OF_CTCS] = {0};

static bool running;

void ctcTimerCallback (tTimer *pTimer)
{
  /* Manage heaters */

  for (int j=0; j < num_ctcs; j++)
  {
    if (ctc_get_current_temp(j) < target_temp[j])
    {
      switch (ctc_configs[j]->output.pwm_method)
      {
        case hal_pwm_bang_bang:
        default:
        {
            write_pin(ctc_configs[j]->output.pin, ENABLE);
            break;
        }

        case hal_pwm_hw_pwm:
        {
          // Max duty
          hal_pwm_chan_set_duty (ctc_configs[j]->output.channel, CFG_MAX_DUTY_CYCLE);
        }
      }
    }
    else
    {
      switch (ctc_configs[j]->output.pwm_method)
      {
        case hal_pwm_bang_bang:
        default:
        {
            write_pin(ctc_configs[j]->output.pin, DISABLE);
            break;
        }

        case hal_pwm_hw_pwm:
        {
          hal_pwm_chan_set_duty (ctc_configs[j]->output.channel, 0);
        }
      }

    }
  }

}


void ctc_init (void)
{
  AddSlowTimer (&ctcTimer);
  StartSlowTimer (&ctcTimer, 200, ctcTimerCallback);
  ctcTimer.AutoReload = 1;
}


void ctc_init_channel (tCtcSettings *ctc_config)
{
  ctc_configs [num_ctcs] = ctc_config;

  // heater output pin
  set_pin_mode (ctc_config->output.pin, OUTPUT);
  write_pin (ctc_config->output.pin, DISABLE);

  if (ctc_config->output.pwm_method == hal_pwm_hw_pwm)
  {
    hal_pwm_chan_configure (ctc_config->output.channel, ctc_config->output.pin);
    hal_pwm_chan_start (ctc_config->output.channel);
  }

  num_ctcs++;
}

void ctc_set_target_temp (uint8_t ctc_number, uint16_t temperature )
{
  if (temperature > 0)
  {
    activity_timer = 0;
//?    power_on();
  }

  target_temp[ctc_number] = temperature;
}

uint16_t ctc_get_target_temp (uint8_t ctc_number)
{
  return target_temp[ctc_number];
}

uint16_t ctc_get_current_temp (uint8_t ctc_number)
{
  return temp_get(ctc_configs[ctc_number]->sensor_index);
}

uint8_t	ctc_temp_achieved(uint8_t ctc_number)
{
  if (temp_get(ctc_configs[ctc_number]->sensor_index) >= target_temp[ctc_number] - 2 )
    return 255;

  return 0;
}

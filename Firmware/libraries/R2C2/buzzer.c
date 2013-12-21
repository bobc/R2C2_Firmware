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



#include "buzzer.h"
#include "timer.h"
#include "hal_pwm.h"

static tTimer buzzerTimer;

static uint16_t pwm_channel = 3;    // r2c2
//
// Internal functions
//

static void buzzer_pwm_set_frequency (uint16_t frequency)
{
    hal_pwm_set_frequency (frequency);
    hal_pwm_chan_set_duty      (pwm_channel, Q15_50_PERCENT);
}

static void buzzer_pwm_start (void)
{
    hal_pwm_start ();
    hal_pwm_chan_start (pwm_channel);
}

static void buzzer_pwm_stop (void)
{
    hal_pwm_chan_stop (pwm_channel);
    hal_pwm_stop ();
}

//
static void buzzerTimerCallback (tTimer *pTimer)
{
  (void)pTimer; // not used

  buzzer_pwm_stop();
}

//
// Public functions
//

void buzzer_play (uint16_t frequency, uint16_t duration)
{
  buzzer_pwm_set_frequency(frequency);
  buzzer_pwm_start();
  
  StartSlowTimer (&buzzerTimer, duration, buzzerTimerCallback); 
}

void buzzer_wait(void)
{
  while (buzzerTimer.Running)
      {};
  
}

void buzzer_play_sync (uint16_t frequency, uint32_t duration)
{
  buzzer_pwm_set_frequency(frequency);
  buzzer_pwm_start();
  
  while (duration--)
    delay_us (1000);
  buzzer_pwm_stop();
}


//TODO: pin config
void buzzer_init (tPinDef pindef)
{
    hal_pwm_chan_configure (pwm_channel, pindef);
  
    AddSlowTimer (&buzzerTimer);
}


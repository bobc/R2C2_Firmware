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

#ifndef _HAL_PWM_H
#define _HAL_PWM_H

// --------------------------------------------------------------------------
// Includes
// --------------------------------------------------------------------------

#include <stdint.h>

#include "ios.h"

// --------------------------------------------------------------------------
// Defines
// --------------------------------------------------------------------------

#define NUM_PWM_CHANNELS    6

//
#define PWM_MAX_DUTY_CYCLE	255
#define PWM_MIN_DUTY_CYCLE	0

//
#define DUTY_CYCLE(percent) ((percent) * 32768ul / 100)

#define Q15_25_PERCENT  (0x2000)
#define Q15_50_PERCENT  (0x4000)
#define Q15_75_PERCENT  (0x6000)
#define Q15_100_PERCENT (0x7FFF)

// --------------------------------------------------------------------------
// Types
// --------------------------------------------------------------------------

typedef enum {
  hal_pwm_none,
  hal_pwm_bang_bang,
  hal_pwm_hw_pwm,
  hal_pwm_timer,
  hal_pwm_soft
  } tHalPwmMethod;

// --------------------------------------------------------------------------
// Public Variables
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Public functions
// --------------------------------------------------------------------------

void hal_pwm_init (void);

void hal_pwm_set_frequency (uint32_t frequency);
void hal_pwm_start         (void);
void hal_pwm_stop          (void);

void hal_pwm_chan_configure     (uint16_t channel, tPinDef pindef);
void hal_pwm_chan_set_duty      (uint16_t channel, uint16_t duty_cycle);
void hal_pwm_chan_start         (uint16_t channel);
void hal_pwm_chan_stop          (uint16_t channel);

// --------------------------------------------------------------------------
//
// --------------------------------------------------------------------------

#endif // _$NAME


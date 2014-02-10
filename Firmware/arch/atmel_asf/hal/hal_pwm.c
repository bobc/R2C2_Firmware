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

#include <string.h>

#include "hal_pwm.h"

#include "asf.h"
#include "pwm.h"

// --------------------------------------------------------------------------
// Externals
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Local defines
// --------------------------------------------------------------------------

#define PWM_FREQUENCY       1000
#define PWM_RESOLUTION      8

#define INIT_DUTY_VALUE     0

// --------------------------------------------------------------------------
// Types
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Variables
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Public Variables
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Private Variables
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Function prototypes
// --------------------------------------------------------------------------


// --------------------------------------------------------------------------
// Private functions
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Public functions
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
//
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
//! @brief
//! @param[in]
//! @param[out]
//! @return
// --------------------------------------------------------------------------

void hal_pwm_init (void)
{

//  pwm_channel_disable(PWM, 5);

  pmc_enable_periph_clk (ID_PWM);

	/* Set PWM clock A as PWM_FREQUENCY*PERIOD_VALUE (clock B is not used) */
	pwm_clock_t clock_setting = {
		.ul_clka = PWM_FREQUENCY * PWM_MAX_DUTY_CYCLE,
		.ul_clkb = 0,
		.ul_mck = sysclk_get_cpu_hz()
	};
	pwm_init(PWM, &clock_setting);

}

void hal_pwm_set_frequency (uint32_t frequency)
{
	pwm_clock_t clock_setting = {
		.ul_clka = frequency * PWM_MAX_DUTY_CYCLE,
		.ul_clkb = 0,
		.ul_mck = sysclk_get_cpu_hz()
	};
	pwm_init(PWM, &clock_setting);

}

void hal_pwm_start ()
{
}

void hal_pwm_stop ()
{
}

void hal_pwm_chan_configure     (uint16_t channel, tPinDef pindef)
{
  pwm_channel_t pwm_channel;

	/* Initialize PWM channel */

  // pio_set_peripheral(PIOC, PIO_TYPE_PIO_PERIPH_B, _BV(pindef.pin_number) ); //TODO
  pio_configure (PIOC, PIO_TYPE_PIO_PERIPH_B, _BV(pindef.pin_number), PIO_DEFAULT);

  memset (&pwm_channel, 0, sizeof(pwm_channel));

	/* Period is left-aligned */
	pwm_channel.alignment = PWM_ALIGN_LEFT;
	
  if (pin_is_active_low(pindef.modes))
    pwm_channel.polarity = PWM_HIGH;
  else
  	pwm_channel.polarity = PWM_LOW;

	/* Use PWM clock A as source clock */
	pwm_channel.ul_prescaler = PWM_CMR_CPRE_CLKA;
	/* Period value of output waveform */
	pwm_channel.ul_period = PWM_MAX_DUTY_CYCLE;
	/* Duty cycle value of output waveform */
	pwm_channel.ul_duty = INIT_DUTY_VALUE;
	pwm_channel.channel = channel;

	pwm_channel_init(PWM, &pwm_channel);

}

void hal_pwm_chan_set_duty      (uint16_t channel, uint16_t duty_cycle)
{
  pwm_channel_t pwm_channel;

  memset (&pwm_channel, 0, sizeof(pwm_channel));

	pwm_channel.ul_period = PWM_MAX_DUTY_CYCLE;
	pwm_channel.ul_duty = duty_cycle;
	pwm_channel.channel = channel;

	pwm_channel_update_duty(PWM, &pwm_channel, duty_cycle);
}

/* Enable PWM Channel Output */
void hal_pwm_chan_start         (uint16_t channel)
{
	pwm_channel_enable(PWM, channel);
}

void hal_pwm_chan_stop          (uint16_t channel)
{
	pwm_channel_disable(PWM, channel);
}

//
// --------------------------------------------------------------------------



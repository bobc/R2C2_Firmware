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

#include "asf.h"
#include "adc.h"

#include "hal_adc.h"


/* Initialize ADC for reading sensors */
void hal_adc_init(void)
{
	/* Enable peripheral clock. */
#if SAM3S || SAM3N || SAM3XA || SAM4S
	uint32_t i;
	pmc_enable_periph_clk(ID_ADC);
#elif SAM3U
  #ifdef ADC_12B
    pmc_enable_periph_clk(ID_ADC12B);
  #else
    pmc_enable_periph_clk(ID_ADC);
  #endif
#endif

	/* Initialize ADC. */
#if SAM3S || SAM3N || SAM3XA || SAM4S
  adc_init(ADC, SystemCoreClock, ADC_FREQ_MAX, ADC_STARTUP_FAST);
#elif SAM3U
  #ifdef ADC_12B
    adc12b_init(ADC12B, sysclk_get_cpu_hz(), 6400000, 10, 10);
  #else
    adc_init(ADC, sysclk_get_cpu_hz(), 6400000, 10);
  #endif
#endif


//
  adc_configure_timing(ADC, 0, ADC_SETTLING_TIME_3, 1);
  adc_configure_trigger(ADC, ADC_TRIG_SW, 0); // Disable hardware trigger.
  adc_disable_interrupt(ADC, 0xFFFFFFFF); // Disable all ADC interrupts.
  adc_disable_all_channel(ADC);

}


void hal_adc_configure_pin (tPinDef pindef)
{
  // enabling the ADC channel will automatically configure the pin when needed
}  
  
uint16_t hal_analog_read(uint8_t adc_channel)
{
    uint32_t ulValue = 0;

    // Enable the corresponding channel
    adc_enable_channel( ADC, adc_channel );

    // Start the ADC
    adc_start( ADC );

    // Wait for end of conversion
    while ((adc_get_status(ADC) & ADC_ISR_DRDY) != ADC_ISR_DRDY)
      ;

    // Read the value
    ulValue = adc_get_latest_value(ADC);
    //!! ulValue = mapResolution(ulValue, ADC_RESOLUTION, _readResolution);

    // Disable the corresponding channel
    adc_disable_channel(ADC, adc_channel);

    return ulValue;
}

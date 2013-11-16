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

#include "asf.h"

#include "ios.h"


static  Pio * GetPio (uint8_t port_num)
{
	switch (port_num)
	{
	case 0: 
		return PIOA;
		break;
	case 1:
		return PIOB;
		break;
	case 2:
		return PIOC;
		break;
	case 3:
		return PIOD;
		break;
	default:
	   return NULL;
	}
}

#if 1
/* Initialize all the IO pins */
/* Example of usage: pin_mode(PORT_0, X_STEP_PIN, OUTPUT); */
void pin_mode(uint8_t portNum, uint32_t bitMask, uint8_t dir)
{
    Pio *p_pio;
    
    p_pio = GetPio (portNum);
    if (p_pio != NULL)
    {
        if (dir == OUTPUT)
        	pio_configure (p_pio, PIO_OUTPUT_0, bitMask, 0);
        else
	        pio_configure (p_pio, PIO_INPUT, bitMask, PIO_PULLUP);
    }
}

/* Example of usage: digital_write(PORT_0, X_STEP_PIN, HIGH); */
void digital_write(uint8_t portNum, uint32_t bitMask, uint8_t state)
{
    Pio *p_pio;
    
    p_pio = GetPio (portNum);
    if (p_pio != NULL)
    {
        if (state)
    		pio_set (p_pio, bitMask);
        else
            pio_clear (p_pio, bitMask);
    }
}

/* Example of usage: value = digital_read(PORT_0, PIN); */
uint32_t digital_read(uint8_t portNum, uint32_t bitMask)
{
    //  return ((FIO_ReadValue(portNum) & bitMask) ? 1 : 0);
    Pio *p_pio;
    
    p_pio = GetPio (portNum);
    if (p_pio != NULL)
      	return pio_get (p_pio, PIO_INPUT, bitMask) ? 1 : 0;
    else
        return 0;
}
#endif

// 
// Using tPinDef
//

tPinDef   PinDef (uint8_t port, uint8_t pin_number, uint8_t modes, uint8_t function)
{
  tPinDef result;
  result.port = port;
  result.pin_number = pin_number;
  result.modes = modes;
  result.function = function;
  return result;
}

void      set_pin_mode (tPinDef pin, uint8_t dir)
{
    if (pin.port != UNDEFINED_PORT)
    {
        Pio *p_pio;
    
        p_pio = GetPio (pin.port);
        if (p_pio != NULL)
        {
            if (dir == OUTPUT)
              pio_configure (p_pio, PIO_OUTPUT_0, _BV(pin.pin_number), 0);
            else
              pio_configure (p_pio, PIO_INPUT, _BV(pin.pin_number), PIO_PULLUP);
        }

        //pin_mode (pin.port, _BV(pin.pin_number), dir);
    }
}



uint32_t  read_pin (tPinDef pin)
{
  if (pin.port == UNDEFINED_PORT)
    return 0;
  else
    return digital_read(pin.port, _BV(pin.pin_number)) ^ pin_is_active_low(pin.modes);
}

void  write_pin (tPinDef pin, uint8_t state)
{
  if (pin.port != UNDEFINED_PORT)
  {
    if (state ^ pin_is_active_low (pin.modes))
      digital_write (pin.port, _BV(pin.pin_number), 1);
    else
      digital_write (pin.port, _BV(pin.pin_number), 0);
  }
}

void ios_init (void)
{
	pmc_enable_periph_clk (ID_PIOA);
	pmc_enable_periph_clk (ID_PIOB);
	pmc_enable_periph_clk (ID_PIOC);
	pmc_enable_periph_clk (ID_PIOD);

}


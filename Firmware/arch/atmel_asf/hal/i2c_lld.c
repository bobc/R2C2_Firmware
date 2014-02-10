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
// Description: STUB functions for i2c
//
// **************************************************************************

// --------------------------------------------------------------------------
// Includes
// --------------------------------------------------------------------------

#include "asf.h"

#include "ios.h"
#include "twi.h"

// --------------------------------------------------------------------------
// Externals
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Local defines
// --------------------------------------------------------------------------

/** TWI Bus Clock 400kHz */
#define TWI_CLK     400000

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

Twi * twi_config [2] = {TWI0, TWI1};

// --------------------------------------------------------------------------
// Function prototypes
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Private functions
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Public functions
// --------------------------------------------------------------------------

void i2c_init (int channel, tPinDef scl, tPinDef sda)
{
	twi_options_t opt;

  	/* Configure the options of TWI driver */
	opt.master_clk = sysclk_get_cpu_hz();
	opt.speed      = TWI_CLK;

	/* Enable the peripheral clock for TWI */
  switch (channel)
  {
  case 0:
    pmc_enable_periph_clk(ID_TWI0);
    twi_master_init(TWI0, &opt);
    break;
  case 1:
    pmc_enable_periph_clk(ID_TWI1);
    twi_master_init(TWI1, &opt);
    break;
  }


}

int i2c_start (int channel)
{
    return 0;
}

void i2c_stop (int channel)
{
}

void i2c_reset (int channel)
{
}


int i2c_read (int channel, int address, char *data, int length, int stop)
{
    twi_packet_t packet_rx;

    packet_rx.chip        = address;
    packet_rx.addr_length = 0;
    packet_rx.buffer      = data;
    packet_rx.length      = length;

    twi_master_read(twi_config[channel], &packet_rx);

    return 0;
}

int i2c_write (int channel, int address, char *data, int length, int stop)
{
    twi_packet_t packet_tx;

    packet_tx.chip        = address;
//    packet_tx.addr[0]     = EEPROM_MEM_ADDR >> 8;
    packet_tx.addr_length = 0;
    packet_tx.buffer      = (uint8_t *) data;
    packet_tx.length      = length;

    twi_master_write(twi_config[channel], &packet_tx);

    return 0;
}


int i2c_read_byte (int channel, int last)
{
    return 0;
}

int i2c_write_byte (int channel, int data)
{
    return 0;
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

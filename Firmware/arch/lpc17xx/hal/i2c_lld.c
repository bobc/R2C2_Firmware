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

#include "lpc17xx_i2c.h"
#include "lpc17xx_pinsel.h"

#include "i2c.h"

// --------------------------------------------------------------------------
// Externals
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Local defines
// --------------------------------------------------------------------------


#define NUM_I2C   3

static LPC_I2C_TypeDef * i2c_p [NUM_I2C] = 
{
  LPC_I2C0,
  LPC_I2C1,
  LPC_I2C2
};

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

void i2c_init (int channel, tPinDef scl, tPinDef sda)         
{
	PINSEL_CFG_Type PinCfg;

	/*
	 * Init I2C pin connect
	 */
	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = 0;

  // 0,27,1	
  // 0,28,1	
	PinCfg.Portnum = sda.port;
  PinCfg.Pinnum = sda.pin_number;
	PinCfg.Funcnum = sda.function;
	PINSEL_ConfigPin(&PinCfg);

	PinCfg.Portnum = scl.port;
  PinCfg.Pinnum = scl.pin_number;
	PinCfg.Funcnum = scl.function;
	PINSEL_ConfigPin(&PinCfg);

  // 0,10,2
  // 0,11,2

	// Initialize I2C peripheral
  //TODO: speed
	I2C_Init (i2c_p [channel], 100000);

	/* Enable I2C operation */
	I2C_Cmd (i2c_p [channel], ENABLE);
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
	I2C_M_SETUP_Type transferMCfg;

	transferMCfg.sl_addr7bit = address;
	transferMCfg.tx_data = NULL ;
	transferMCfg.tx_length = 0;
	transferMCfg.rx_data = data;
	transferMCfg.rx_length = length;
	transferMCfg.retransmissions_max = 3;
	I2C_MasterTransferData(i2c_p [channel], &transferMCfg, I2C_TRANSFER_POLLING);

    return 0;
}

int i2c_write (int channel, int address, char *data, int length, int stop)
{
	I2C_M_SETUP_Type transferMCfg;

  memset (&transferMCfg,  0x00, sizeof(transferMCfg));

	transferMCfg.sl_addr7bit = address;
	transferMCfg.tx_data = data;
	transferMCfg.tx_length = length;
	transferMCfg.rx_data = NULL;
	transferMCfg.rx_length = 0;
	transferMCfg.retransmissions_max = 3;
	I2C_MasterTransferData(i2c_p [channel], &transferMCfg, I2C_TRANSFER_POLLING);
	
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

/* Copyright (c) 2007, 2010, ChaN, Martin Thomas, Mike Anton */
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

#ifndef SPI_H_
#define SPI_H_

#include <stdint.h>

#include "ios.h"

enum speed_setting { INTERFACE_SLOW, INTERFACE_FAST };

void spi_configure (tPinDef SClk, tPinDef Mosi, tPinDef Miso, tPinDef SSel);

void spi_init (void);

void spi_set_speed( enum speed_setting speed );
void spi_close(void);

uint8_t spi_transmit_and_receive (uint8_t out );
uint8_t spi_receive_byte (void);

//! @param[out] Data buffer to store received data
//! @param[in] Byte count (must be multiple of 4)
void spi_receive_block (uint8_t *buff, uint16_t byte_count);

void spi_transmit_block (const uint8_t *buff);    /* 512 byte data block to be transmitted */



#endif /* SPI_H_ */

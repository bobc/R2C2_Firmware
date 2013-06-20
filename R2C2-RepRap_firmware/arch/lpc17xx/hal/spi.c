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

#include "lpc_types.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_clkpwr.h"
#include "lpc17xx_ssp.h"

#include "spi.h"

static LPC_SSP_TypeDef * pSPI;

/*-----------------------------------------------------------------------*/
/* SPI low-level functions                                               */
/*-----------------------------------------------------------------------*/

void spi_configure (tPinDef SClk, tPinDef Mosi, tPinDef Miso, tPinDef SSel)
{
  PINSEL_CFG_Type PinCfg;

  /*
   * Initialize SPI pin connect
   */
   
  /* SSEL as GPIO, pull-up mounted */
  PinCfg.Funcnum   = PINSEL_FUNC_0;
  PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
  PinCfg.Pinmode   = PINSEL_PINMODE_PULLUP;
  PinCfg.Portnum   = SSel.port;
  PinCfg.Pinnum    = SSel.pin_number;
  GPIO_SetDir(SSel.port, _BV(SSel.pin_number), 1);
  PINSEL_ConfigPin(&PinCfg);

  /* SCK alternate function 0b10 */
  PinCfg.Funcnum   = PINSEL_FUNC_2;
  PinCfg.Pinmode   = PINSEL_PINMODE_PULLDOWN;
  PinCfg.Portnum   = SClk.port;
  PinCfg.Pinnum    = SClk.pin_number;
  PINSEL_ConfigPin(&PinCfg);

  /* MISO */
  PinCfg.Pinmode   = PINSEL_PINMODE_PULLUP;
  PinCfg.Portnum   = Miso.port;
  PinCfg.Pinnum    = Miso.pin_number;
  PINSEL_ConfigPin(&PinCfg);

  /* MOSI */
  PinCfg.Portnum   = Mosi.port;
  PinCfg.Pinnum    = Mosi.pin_number;
  PINSEL_ConfigPin(&PinCfg);
 }

void spi_init (int spi_channel)
{
  SSP_CFG_Type SSP_ConfigStruct;

  if (spi_channel == 0)
    pSPI = LPC_SSP0;
  else
    pSPI = LPC_SSP1;

  /* initialize SSP configuration structure */
  SSP_ConfigStruct.CPHA = SSP_CPHA_FIRST;
  SSP_ConfigStruct.CPOL = SSP_CPOL_HI;
  SSP_ConfigStruct.ClockRate = 200000; /* 200KHz */
  SSP_ConfigStruct.Databit = SSP_DATABIT_8;
  SSP_ConfigStruct.Mode = SSP_MASTER_MODE;
  SSP_ConfigStruct.FrameFormat = SSP_FRAME_SPI;
  SSP_Init(pSPI, &SSP_ConfigStruct);

  /* Enable SSP peripheral */
  SSP_Cmd(pSPI, ENABLE);
}

void spi_set_speed( enum speed_setting speed )
{
  if ( speed == INTERFACE_SLOW )
  {
    setSSPclock(pSPI, 400000);
  }
  else
  {
    setSSPclock(pSPI, 25000000);
  }
}

void spi_close(void)
{
  PINSEL_CFG_Type PinCfg;

  SSP_Cmd(pSPI, DISABLE);
  SSP_DeInit(pSPI);

/*
  PinCfg.Funcnum   = PINSEL_FUNC_0;
  PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
  PinCfg.Pinmode   = PINSEL_PINMODE_PULLDOWN;
  PinCfg.Pinnum    = 16;
  PinCfg.Portnum   = 0;
  PINSEL_ConfigPin(&PinCfg);
  PinCfg.Pinnum    = 15;
  PINSEL_ConfigPin(&PinCfg);
  PinCfg.Pinnum    = 17;
  PINSEL_ConfigPin(&PinCfg);
  PinCfg.Pinnum    = 18;
  PINSEL_ConfigPin(&PinCfg);
*/
}

uint8_t spi_transmit_and_receive ( uint8_t out )
{
  uint8_t in;

  pSPI->DR = out;
  while (pSPI->SR & SSP_SR_BSY ) { ; }
  in = pSPI->DR;

  return in;
}

uint8_t spi_receive_byte (void)
{
  return spi_transmit_and_receive (0xff);
}

/* Alternative macro to receive data fast */
#define rcvr_spi_m(dst)  *(dst)=spi_rw(0xff)

#define FIFO_ELEM 8           /* "8 frame FIFOs for both transmit and receive.*/

void spi_receive_block (
        uint8_t *buff,         /* Data buffer to store received data */
        uint16_t byte_count            /* Byte count (must be multiple of 4) */
)
{
        uint16_t hwtr, startcnt, i, rec;

        hwtr = byte_count/2;
        if ( byte_count < FIFO_ELEM ) {
                startcnt = hwtr;
        } else {
                startcnt = FIFO_ELEM;
        }

        pSPI->CR0 |= SSP_CR0_DSS(16); // DSS to 16 bit

        for ( i = startcnt; i; i-- ) {
                pSPI->DR = 0xffff;  // fill TX FIFO
        }

        do {
                while ( !(pSPI->SR & SSP_SR_RNE ) ) {
                        // wait for data in RX FIFO (RNE set)
                }
                rec = pSPI->DR;
                if ( i < ( hwtr - startcnt ) ) {
                        pSPI->DR = 0xffff;
                }
                *buff++ = (uint8_t)(rec >> 8);
                *buff++ = (uint8_t)(rec);
                i++;
        } while ( i < hwtr );

        pSPI->CR0 = ( pSPI->CR0 & ~SSP_CR0_DSS(16) ) | SSP_CR0_DSS(8); // DSS to 8 bit
}

void spi_transmit_block (
        const uint8_t *buff    /* 512 byte data block to be transmitted */
)
{
        uint16_t cnt;
        int16_t data;

        pSPI->CR0 |= SSP_CR0_DSS(16); // DSS to 16 bit

        for ( cnt = 0; cnt < ( 512 / 2 ); cnt++ ) {
                while ( !( pSPI->SR & SSP_SR_TNF ) ) {
                        ; // wait for TX FIFO not full (TNF)
                }
                data  = (*buff++) << 8;
                data |= *buff++;
                pSPI->DR = data;
        }

        while ( pSPI->SR & SSP_SR_BSY ) {
                // wait for BSY gone
        }
        while ( pSPI->SR & SSP_SR_RNE ) {
                data = pSPI->DR; // drain receive FIFO
        }

        pSPI->CR0 = ( pSPI->CR0 & ~SSP_CR0_DSS(16) ) | SSP_CR0_DSS(8); // DSS to 8 bit
}

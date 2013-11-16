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

#include <stddef.h>

#include "asf.h"

#include "str_buffer.h"

#include "hal_uart.h"

// --------------------------------------------------------------------------
// Externals
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Local defines
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Types
// --------------------------------------------------------------------------

typedef struct {

  union {  
    Uart *uart_def_p;
    Usart *usart_def_p;
  };

  // Current Tx Interrupt enable state
  bool TxIntEnabled;

  tStrBuffer rx_buffer;
  tStrBuffer tx_buffer;

} tUartControl;

// --------------------------------------------------------------------------
// Variables
// --------------------------------------------------------------------------

#define USE_INTERRUPT

#define CFG_MAX_UART  5

// don't allocate buffers for UARTs we don't use

tUartControl uart0 = {.uart_def_p = UART};

tUartControl usart0 = {.usart_def_p = USART0};
//tUartControl uart1 = {.uart_def_p = (LPC_UART_TypeDef *)LPC_UART1};
//tUartControl uart2 = {.uart_def_p = LPC_UART2};
//tUartControl uart3 = {.uart_def_p = LPC_UART3};

static tUartControl *uart_control [CFG_MAX_UART] = 
{
  &uart0,  
  &usart0,
  NULL,
  NULL,
  NULL
};

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

static tUartControl *_get_uart_control (int uart_num)
{
  if ((uart_num >= 0) && (uart_num < CFG_MAX_UART))
  {
    return uart_control [uart_num];
  }
  else
    return NULL;
}

// --------------------------------------------------------------------------
//  Functions for UART peripheral
// --------------------------------------------------------------------------

void UART_Handler(void)
{
    tUartControl *pControl;
	uint32_t ul_status;
	uint8_t c;

	pControl = uart_control[0]; 

	/* Read UART Status. */
	ul_status = uart_get_status(UART);

	if (ul_status & UART_SR_RXRDY)
	{
		uart_read(UART, &c);
		
		/* Check if buffer has more space
		 * If no space, data will be discarded
		 */
		if (str_buf_is_full (&pControl->rx_buffer))
        {
            pControl->rx_buffer.header.err_overflow = 1;
        }
        else
        {
            str_buf_putc (&pControl->rx_buffer, c);
		}
	}
	
	if (ul_status & UART_SR_TXRDY)
	{
		uart_disable_interrupt (UART, UART_IER_TXRDY);

    	while (!str_buf_is_empty (&pControl->tx_buffer))
        {
            c = str_buf_getc (&pControl->tx_buffer);
            uart_write (pControl->uart_def_p, c);
    
            if ( !uart_is_tx_ready (pControl->uart_def_p) )
                break;
        }
        
        /* If there is no more data to send, disable the transmit
            interrupt - else enable it or keep it enabled */
	    if (str_buf_is_empty (&pControl->tx_buffer)) 
        {
		    uart_disable_interrupt (UART, UART_IER_TXRDY);
            pControl->TxIntEnabled = false;
        }
        else
        {
            // Set Tx Interrupt state
            pControl->TxIntEnabled = true;
            uart_enable_interrupt (UART, UART_IER_TXRDY);
        }

	} //
	
}

static void _uart_init (void)
{
    tUartControl *pControl;
	sam_uart_opt_t config;
	int result;
	
	pControl = uart_control[0]; 
	
	//

	pmc_enable_periph_clk (ID_UART);
	
	//
	pio_configure (PIOA, PIO_PERIPH_A | PIO_PULLUP, PIO_PA8A_URXD, PIO_DEFAULT);
	pio_configure (PIOA, PIO_PERIPH_A, PIO_PA9A_UTXD, PIO_DEFAULT);
		
	//
	config.ul_mck = sysclk_get_cpu_hz();
	//config.ul_baudrate = BAUD;
	config.ul_baudrate = 115200;
	config.ul_mode = UART_MR_PAR_NO;
	config.ul_chmode = 0;

    str_buf_init_std (&pControl->rx_buffer);
    str_buf_init_std (&pControl->tx_buffer);
	
	result = uart_init (UART, &config);

	uart_reset_tx(UART);
	uart_reset_rx(UART);

	UART->UART_CR = UART_CR_RSTSTA;

	uart_enable_rx(UART);	
	uart_enable_tx(UART);
	
	uart_enable_interrupt (UART, US_IER_RXRDY);
	NVIC_EnableIRQ(UART_IRQn);

}

// --------------------------------------------------------------------------
// Public functions
// --------------------------------------------------------------------------

void hal_uart_init(int uart_num)
{
    switch (uart_num)
    {
    case 0:
        _uart_init();
        break;
    }
}

bool hal_uart_configure(int uart_num, tPortSettings *port_settings_p)
{
    return false;
}

int  hal_uart_data_available(int uart_num)
{
  tUartControl *pControl;

  pControl = _get_uart_control (uart_num);
  if (pControl == NULL)
    return 0;

  return !str_buf_is_empty (&pControl->rx_buffer);
}

char hal_uart_receive(int uart_num)
{
  tUartControl *pControl;
//  LPC_UART_TypeDef *pUart;

  pControl = _get_uart_control (uart_num);
  if (pControl == NULL)
    return 0;
//  pUart = pControl->uart_def_p;
	
#ifdef USE_INTERRUPT
  if (str_buf_is_empty (&pControl->rx_buffer))
    return 0;
  else
    return str_buf_getc (&pControl->rx_buffer);
#else
  return UART_ReceiveByte(pUart);
#endif
}

void hal_uart_send(int uart_num, char byte)
{
  tUartControl *pControl;

  pControl = _get_uart_control (uart_num);
  if (pControl == NULL)
    return;

#ifdef USE_INTERRUPT
  // put byte into buffer
  str_buf_putc (&pControl->tx_buffer, byte);

  // re-enable Tx Interrupt
  uart_enable_interrupt (UART, UART_IER_TXRDY);
#else
  
//todo:
	while ( (pUart->LSR & UART_LSR_THRE) == 0) ;
	UART_SendByte(pUart, byte);
#endif
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

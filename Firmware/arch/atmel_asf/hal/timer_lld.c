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


#include <stdint.h>

#include "asf.h"


#include "timer.h"
#include "timer_lld.h"


static tHwTimer HwTimer [NUM_HARDWARE_TIMERS];


typedef struct
  {
    Tc          *pTimerRegs;
    uint16_t    channel;
    IRQn_Type   IRQ_Id;
    
  } tTimerConfig ;

static const tTimerConfig TimerConfig [NUM_HARDWARE_TIMERS] =
  {
    { TC0, 0, TC0_IRQn},
    { TC0, 1, TC1_IRQn},
    { TC0, 2, TC2_IRQn},
    { TC1, 0, TC3_IRQn},
    { TC1, 1, TC4_IRQn},
    { TC1, 2, TC5_IRQn},
    { TC2, 0, TC6_IRQn},
    { TC2, 1, TC7_IRQn},
    { TC2, 2, TC8_IRQn},
  };


static inline void TIMER_IRQHandlerGeneric (uint16_t timerNum)
{
  const tTimerConfig *pConfig = &TimerConfig [timerNum]; 
  volatile uint32_t int_status;

	/* Clear status bit to acknowledge interrupt */
	int_status = tc_get_status(pConfig->pTimerRegs, pConfig->channel);

  if (HwTimer[timerNum].timerCallback)
      HwTimer[timerNum].timerCallback (&HwTimer[timerNum], int_status);
}

/**
 *  Interrupt handlers for TCx interrupts.
 */
void TC0_Handler(void)
{
    TIMER_IRQHandlerGeneric (0);
}

void TC1_Handler(void)
{
    TIMER_IRQHandlerGeneric (1);
}

void TC2_Handler(void)
{
    TIMER_IRQHandlerGeneric (2);
}

void TC3_Handler(void)
{
    TIMER_IRQHandlerGeneric (3);
}

void TC4_Handler(void)
{
    TIMER_IRQHandlerGeneric (4);
}

void TC5_Handler(void)
{
    TIMER_IRQHandlerGeneric (5);
}

void TC6_Handler(void)
{
    TIMER_IRQHandlerGeneric (6);
}

void TC7_Handler(void)
{
    TIMER_IRQHandlerGeneric (7);
}

void TC8_Handler(void)
{
    TIMER_IRQHandlerGeneric (8);
}


/* Set the timer interval. 

 * With default setup, ticks are in 1/42MHz ~= 23.8ns,
 * Maximum time of 2^32 ticks
 */
void setHwTimerInterval (uint16_t timerNum, uint32_t ticks)
{
  const tTimerConfig *pConfig = &TimerConfig [timerNum]; 

  tc_write_rc (pConfig->pTimerRegs, pConfig->channel, ticks);
}

// setup hardware timer timerNum
// @param timerCallback: if not-NULL, will be called on timer reload
//
// For convenience various default values are setup: 
//   a timer period of 1ms is set, this can be changed by setHwTimerInterval().
void setupHwTimer (uint16_t timerNum, tHwTimerCallback timerCallback)
{
    const tTimerConfig *pConfig = &TimerConfig [timerNum]; 
     
    tc_init (pConfig->pTimerRegs, pConfig->channel, TC_CMR_TCCLKS_TIMER_CLOCK1 | TC_CMR_CPCTRG );

    tc_write_rc (pConfig->pTimerRegs, pConfig->channel, 1 * HAL_HW_TICKS_PER_MILLISECOND);

    // store the callback address for later use
    HwTimer[timerNum].timerCallback = timerCallback;

    if (timerCallback)
    {
        /* Configure and enable interrupt on RC compare */
        NVIC_EnableIRQ((IRQn_Type) pConfig->IRQ_Id);
        tc_enable_interrupt(pConfig->pTimerRegs, pConfig->channel, TC_IER_CPCS);
    }

}



// start the timer and enable interrupts
void enableHwTimer (uint16_t timerNum)
{
    const tTimerConfig *pConfig = &TimerConfig [timerNum]; 

    tc_start (pConfig->pTimerRegs, pConfig->channel);
}

// stop the timer and disable interrupts
void disableHwTimer (uint16_t timerNum)
{
    const tTimerConfig *pConfig = &TimerConfig [timerNum]; 

    tc_stop (pConfig->pTimerRegs, pConfig->channel);
}

uint8_t isHwTimerEnabled(uint16_t timerNum)
{
    return false;
}

// LPC17xx 
// set and enable a MatchX value and interrupt. If set, the timer callback will be called
// when timer is enabled
void setHwTimerMatch (uint16_t timerNum, uint16_t matchReg, uint32_t interval)
{
  // not supported!
}



static void SysTickTimer_Init(void)
{
  // Setup SysTick Timer to interrupt at 1 ms intervals
  // Lowest priority = 31

  if (SysTick_Config(sysclk_get_cpu_hz() / 1000)) 
  {
		//oops
		while (1);
	}

}

// IRQ handler referenced in exception vectors
void SysTick_Handler(void)
{
	timer_sys_tick();
}


void timer_lld_init (void)
{
	SysTickTimer_Init ();
	
	pmc_enable_periph_clk(ID_TC0);
	pmc_enable_periph_clk(ID_TC1);
	pmc_enable_periph_clk(ID_TC2);
	pmc_enable_periph_clk(ID_TC3);

}


// END


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

#include "timer.h"
#include "timer_lld.h"



/* Set the timer interval. With default setup,
 * ticks are in 1/100MHz = 10ns ticks, minimum of 200 ticks = 2us
 * Maximum time of 2^32*10ns = 42.94967296 seconds. */
void setHwTimerInterval (uint16_t timerNum, uint32_t ticks)
{

}

// setup hardware timer timerNum
// timerCallback: if not-NULL, will be called on any timer interrupt. Default is Match0
// For convenience various default values are setup: a default reload/Match0 period of 
// 10us is set, this can be changed by setHwTimerInterval.
void setupHwTimer (uint16_t timerNum, tHwTimerCallback timerCallback)
{
}



// start the timer and enable interrupts
void enableHwTimer (uint16_t timerNum)
{
}

// stop the timer and disable interrupts
void disableHwTimer (uint16_t timerNum)
{
}

uint8_t isHwTimerEnabled(uint16_t timerNum)
{
  return false;
}

// set and enable a MatchX value and interrupt. If set, the timer callback will be called
// when timer is enabled
void setHwTimerMatch (uint16_t timerNum, uint16_t matchReg, uint32_t interval)
{
}



static void SysTickTimer_Init(void)
{
  // Setup SysTick Timer to interrupt at 1 ms intervals
  // Lowest priority = 31
}

// IRQ handler referenced in extception vectors
void SysTick_Handler(void)
{
	timer_sys_tick();
}


void timer_lld_init (void)
{
	SysTickTimer_Init ();
}


// END


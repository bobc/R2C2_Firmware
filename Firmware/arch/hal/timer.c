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


#include <stddef.h>

//#include "ios.h"
#include "timer.h"
#include "timer_lld.h"

static volatile long millis_ticks;

static tTimer *SlowTimerHead;
static tTimer *SlowTimerTail;


void timer_init (void)
{
	timer_lld_init ();
}

// this should be called at 1ms rate by a timer
// process the slow timer list
void timer_sys_tick (void)
{
  tTimer 		*pTimer;

  millis_ticks++;

  pTimer = SlowTimerHead;
  while (pTimer)
  {
    if (pTimer->Running)
    {
      if (pTimer->Current > 0)
        pTimer->Current--;

      if (pTimer->Current == 0)
      {
        if (pTimer->AutoReload)
          pTimer->Current = pTimer->Reload;
        else 
          pTimer->Running = 0;
        
        pTimer->Expired = 1;
        if (pTimer->timerCallback)
          pTimer->timerCallback(pTimer);
      }
    }
    pTimer = pTimer->pNext;
  }
  
  // app_SysTick will be called every milli-second
  app_SysTick ();
}
/***********************************************************************/

//NB result is signed 32 bit
long millis(void)
{
  return millis_ticks;
}

void delay_ms(int ms)
{
  int start = millis();

  while (millis() - start <= ms)
    ;
}

//TODO: Portable
void delayMicrosecondsInterruptible(volatile int us)
{
  // approximate delay
  us = us * 10;
  while (us)
  {
      us--;
  }
}

//TODO: Portable
// delay( microseconds )
void delay (int d)
{
  while (d > 65535) 
  {
    delayMicrosecondsInterruptible(65534);
    d -= 65535;
  }
  delayMicrosecondsInterruptible(d & 0xFFFF);
}

static bool find_entry (tTimer *pTimer)
{
  tTimer *pThis;

  pThis = SlowTimerHead;
  while (pThis != NULL)
  {
    if (pThis == pTimer)
      return true;
    pThis = pThis->pNext;
  }
  return false;
}

// Slow timers
bool AddSlowTimer (tTimer *pTimer)
{
  if (!find_entry (pTimer))
  {
    pTimer->pNext = NULL;
    if (SlowTimerHead == NULL)
    {
      SlowTimerHead = pTimer;
      SlowTimerTail = pTimer;
    }
    else
    {
      SlowTimerTail->pNext = pTimer;
      SlowTimerTail = pTimer;
    }

    return true;
  }
  else
    return false;
}

void StartSlowTimer (tTimer *pTimer, uint32_t intervalMillis, tTimerCallback timerCallback)
{
  pTimer->Reload = intervalMillis;
  pTimer->Current = pTimer->Reload;
  pTimer->timerCallback = timerCallback;
  pTimer->Expired = 0;
  pTimer->Running = 1;
}

void StopSlowTimer (tTimer *pTimer)
{
  pTimer->Running = false;
}
// END


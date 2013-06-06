/* Copyright (c) 2012 Bob Cousins bobcousins42@googlemail.com              */
/* **************************************************************************
   All rights reserved.

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
****************************************************************************/

// **************************************************************************
// Description:
//
// **************************************************************************

#ifndef _LW_RTOS_H
#define _LW_RTOS_H

// --------------------------------------------------------------------------
// Includes
// --------------------------------------------------------------------------

#include <stdint.h>
#include <stddef.h>

// --------------------------------------------------------------------------
// Defines
// --------------------------------------------------------------------------

#define LWR_MAX_DELAY 1000 // max waiting time in ticks

// --------------------------------------------------------------------------
// Types
// --------------------------------------------------------------------------

// Note: Handles are pointers
typedef void* tTaskHandle;
typedef void* tQueueHandle;

typedef uint32_t tTicks;

typedef enum {
    LWR_OK,
    LWR_ERROR
    }
    LW_RTOS_RESULT;

typedef enum {
    LWR_IDLE_PRIORITY = 0
    } tTaskPriority;

typedef void (*fpTaskInit) ( void *pvParameters );

typedef void (*fpTaskPoll) ( void *pvParameters );

// --------------------------------------------------------------------------
// Public Variables
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Public functions
// --------------------------------------------------------------------------

void lw_mem_free ( void *pv );

void *lw_mem_malloc( size_t xSize );


LW_RTOS_RESULT lw_TaskCreate (  fpTaskPoll    TaskPoll,
                                fpTaskInit    TaskInit,
                                const char * const pName,
							    uint16_t       StackSize,
							    void           *pvParameters,
							    uint16_t       uPriority,
							    tTaskHandle    *pTaskId);

LW_RTOS_RESULT lw_TaskDelete (tTaskHandle TaskId);

LW_RTOS_RESULT lw_TaskScheduler (void);


//LW_RTOS_RESULT lw_QueueCreate (uint16_t uQueueLength, uint16_t ItemSize, tQueueHandle *pQueueId);

tQueueHandle lw_QueueCreate (uint16_t uQueueLength, uint16_t ItemSize);

uint16_t lw_QueueMessagesWaiting (const tQueueHandle QueueId);

LW_RTOS_RESULT lw_QueueGet (tQueueHandle QueueId, void * pItemBuffer, tTicks Timeout);

LW_RTOS_RESULT lw_QueuePeek (tQueueHandle QueueId, void * pItemBuffer, tTicks Timeout);

LW_RTOS_RESULT lw_QueuePut (tQueueHandle QueueId, void * pItemBuffer, tTicks Timeout);


#endif
// --------------------------------------------------------------------------


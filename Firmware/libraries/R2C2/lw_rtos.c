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

#include "lw_rtos.h"
#include "lw_queue.h"

#include "nk_heap.h"
// --------------------------------------------------------------------------
// Externals
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Local defines
// --------------------------------------------------------------------------

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

// --------------------------------------------------------------------------
//
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
//! @brief
//! @param[in]
//! @param[out]
//! @return
// --------------------------------------------------------------------------


void lw_mem_free ( void *pv )
{
    free (pv);
}

void *lw_mem_malloc( size_t xSize )
{
    return malloc (xSize);
}






tQueueHandle lw_QueueCreate (uint16_t uQueueLength, uint16_t ItemSize)
{
    tQueueHeader *pQueue;
    uint8_t *pData;
     
    pQueue = malloc (sizeof (tQueueHeader) + sizeof (uQueueLength * ItemSize));
    pData = pQueue + 1;

    QueueInit (pQueue, pData, uQueueLength, ItemSize);

    return pQueue;
}


uint16_t lw_QueueMessagesWaiting (const tQueueHandle QueueId)
{
    tQueueHeader *pQueue = (tQueueHeader *)QueueId;

    if ( QueueIsEmpty (pQueue))
        return 0;
    else
        return 1;
}


LW_RTOS_RESULT lw_QueueGet (tQueueHandle QueueId, void * pItemBuffer, tTicks Timeout)
{
    if (QueueGet ( (tQueueHeader *)QueueId, pItemBuffer))
      return LWR_OK;
    else
      return LWR_ERROR;
}


LW_RTOS_RESULT lw_QueuePeek (tQueueHandle QueueId, void * pItemBuffer, tTicks Timeout)
{
    return LWR_ERROR;
}


LW_RTOS_RESULT lw_QueuePut (tQueueHandle QueueId, void * pItemBuffer, tTicks Timeout)
{
    if (QueuePut ( (tQueueHeader *)QueueId, pItemBuffer))
      return LWR_OK;
    else
      return LWR_ERROR;
}


// --------------------------------------------------------------------------
//
// --------------------------------------------------------------------------

#if 0
void vPortFree( void *pv )
{
}

void *pvPortMalloc( size_t xSize )
{
    return NULL;
}



portBASE_TYPE xTaskCreate(
                              pdTASK_CODE pvTaskCode,
                              const char * const pcName,
                              unsigned short usStackDepth,
                              void *pvParameters,
                              unsigned portBASE_TYPE uxPriority,
                              xTaskHandle *pvCreatedTask
                          )
{
  return pdPASS;
}

void vTaskDelete( xTaskHandle pxTask )
{
}

void vTaskStartScheduler( void )
{
}



xQueueHandle xQueueCreate( unsigned portBASE_TYPE uxQueueLength, unsigned portBASE_TYPE uxItemSize )
{
  return 0;
}


unsigned portBASE_TYPE uxQueueMessagesWaiting( const xQueueHandle xQueue )
{
  return 0;
}

signed portBASE_TYPE xQueueReceive( xQueueHandle xQueue, void * const pvBuffer, portTickType xTicksToWait )
{
  return pdTRUE;
}


signed portBASE_TYPE xQueueSend( xQueueHandle xQueue, const void * const pvItemToQueue, portTickType xTicksToWait)
{
  return pdTRUE;
}



signed portBASE_TYPE xQueueGenericReceive( xQueueHandle xQueue, void * const pvBuffer, portTickType xTicksToWait, portBASE_TYPE xJustPeek )
{
  return pdTRUE;
}


signed portBASE_TYPE xQueueGenericSend( xQueueHandle xQueue, const void * const pvItemToQueue, portTickType xTicksToWait, portBASE_TYPE xCopyPosition )
{
  return pdTRUE;
}
#endif
// --------------------------------------------------------------------------
//
// --------------------------------------------------------------------------

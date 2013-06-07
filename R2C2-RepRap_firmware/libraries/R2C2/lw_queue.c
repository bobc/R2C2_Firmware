/* ========================================================================== */
/*                                                                            */
/*   Filename.c                                                               */
/*   (c) 2001 Author                                                          */
/*                                                                            */
/*   Description                                                              */
/*                                                                            */
/* ========================================================================== */

#include <stddef.h>
#include <string.h>

#include "lw_queue.h"

static int IncWrap (int count, int max)
{
   count++;
   if (count >= max)
      count = 0;
   return count;
}

void QueueInit (tQueueHeader *pQueueHeader, tpQueueData pData, uint16_t MaxEntries, uint16_t ItemSize)
{
   if (pQueueHeader != NULL)
   {
      pQueueHeader->First = 0;         
      pQueueHeader->Last = 0;         
      pQueueHeader->Count = 0;
      
      pQueueHeader->MaxEntries = MaxEntries;
      pQueueHeader->ItemSize = ItemSize;
      pQueueHeader->pData = pData;
   }
}



bool QueuePut (tQueueHeader *pQueueHeader, tpQueueData pMessage)
{
   if (pQueueHeader != NULL)
   {
      if (pQueueHeader->Count < pQueueHeader->MaxEntries)
      {
         uint8_t *pDest;
         
         pDest = pQueueHeader->pData +  pQueueHeader->ItemSize * pQueueHeader->Last;
         
         memcpy (pDest, pMessage, pQueueHeader->ItemSize);
         
         pQueueHeader->Last = IncWrap (pQueueHeader->Last, pQueueHeader->MaxEntries);
         pQueueHeader->Count++;
         return true;
      }
      else
      {
         // DebugError("SendMessageToQueue");
         return false; // queue full
      }
   }
   else
      return false; // invalid queue
}

bool QueueIsEmpty (tQueueHeader *pQueueHeader)
{
    return pQueueHeader->Count == 0;
}

bool QueueIsFull (tQueueHeader *pQueueHeader)
{
    return pQueueHeader->Count == pQueueHeader->MaxEntries;
}


bool QueueGet (tQueueHeader *pQueueHeader, tpQueueData pMessage)
{
    if (pQueueHeader->Count != 0)
    {
        uint8_t *pSrc;
     
        pSrc = pQueueHeader->pData +  pQueueHeader->ItemSize * pQueueHeader->First;

        memcpy (pMessage, pSrc, pQueueHeader->ItemSize);
      
        pQueueHeader->First = IncWrap (pQueueHeader->First, pQueueHeader->MaxEntries);
        pQueueHeader->Count--;
   
        return true;
    }
    else
        return false;
}



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

void QueueInit (tQueue *pQueue, uint8_t *pData, int NumEntries, int ItemSize)
{
   if (pQueue != NULL)
   {
      pQueue->Header.First = 0;         
      pQueue->Header.Last = 0;         
      pQueue->Header.Count = 0;
      
      pQueue->Header.NumEntries = NumEntries;
      pQueue->Header.ItemSize = ItemSize;
      pQueue->Header.pData = pData;
   }
}



bool QueuePut (tQueue *pQueue, tpMessageData pMessage)
{
   if (pQueue != NULL)
   {
      if (pQueue->Header.Count < pQueue->Header.NumEntries)
      {
         uint8_t *pDest;
         
         pDest = pQueue->Header.pData +  pQueue->Header.ItemSize * pQueue->Header.Last;
         
         memcpy (pDest, pMessage, pQueue->Header.ItemSize);
         
         pQueue->Header.Last = IncWrap (pQueue->Header.Last, pQueue->Header.NumEntries);
         pQueue->Header.Count++;
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

bool QueueEmpty (tQueue *pQueue)
{
    return pQueue->Header.Count == 0;
}


bool QueueGet (tQueue *pQueue, tpMessageData pMessage)
{
    if (pQueue->Header.Count != 0)
    {
        uint8_t *pSrc;
     
        pSrc = pQueue->Header.pData +  pQueue->Header.ItemSize * pQueue->Header.First;

        memcpy (pMessage, pSrc, pQueue->Header.ItemSize);
      
        pQueue->Header.First = IncWrap (pQueue->Header.First, pQueue->Header.NumEntries);
        pQueue->Header.Count--;
   
        return true;
    }
    else
        return false;
}



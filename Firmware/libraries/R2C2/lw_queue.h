/* ========================================================================== */
/*                                                                            */
/*   Filename.c                                                               */
/*   (c) 2001 Author                                                          */
/*                                                                            */
/*   Description                                                              */
/*                                                                            */
/* ========================================================================== */

#ifndef _LW_QUEUE_H
#define _LW_QUEUE_H

#include <stdbool.h>
#include <stdint.h>

typedef uint8_t * tpQueueData;

typedef struct {
    uint16_t First;
    uint16_t Last;
    uint16_t Count;
    
    uint16_t MaxEntries;
    uint16_t ItemSize;
    
    uint8_t *pData;
   
} tQueueHeader;

typedef struct {
    tQueueHeader Header;
    
    uint8_t Data [4];
} tQueue;


void QueueInit (tQueueHeader *pQueueHeader, uint8_t *pData, uint16_t NumEntries, uint16_t ItemSize);

bool QueuePut (tQueueHeader *pQueueHeader, tpQueueData pMessage);

bool QueueGet (tQueueHeader *pQueueHeader, tpQueueData pMessage);

bool QueueIsEmpty (tQueueHeader *pQueueHeader);
bool QueueIsFull (tQueueHeader *pQueueHeader);

//



//---------------------------------------------------------------------------
#endif // _LW_QUEUE_H

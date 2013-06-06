/* ========================================================================== */
/*                                                                            */
/*   Filename.c                                                               */
/*   (c) 2001 Author                                                          */
/*                                                                            */
/*   Description                                                              */
/*                                                                            */
/* ========================================================================== */

#ifndef _QUEUE_H
#define _QUEUE_H

#include <stdbool.h>
#include <stdint.h>

typedef void * tpMessageData;

typedef struct {
    int First;
    int Last;
    int Count;
    
    int NumEntries;   
    int ItemSize;
    
    uint8_t *pData;
   
} tQueueHeader;

typedef struct {
    tQueueHeader Header;
    
    uint8_t Data [0];
} tQueue;


void QueueInit (tQueue *pQueue, uint8_t *pData, int NumEntries, int ItemSize);

bool QueuePut (tQueue *pQueue, tpMessageData pMessage);

bool QueueEmpty (tQueue *pQueue);

bool QueueGet (tQueue *pQueue, tpMessageData pMessage);

//



//---------------------------------------------------------------------------
#endif // _QUEUE_H
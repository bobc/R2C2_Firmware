/* ========================================================================== */
/*                                                                            */
/*   Filename.c                                                               */
/*   (c) 2001 Author                                                          */
/*                                                                            */
/*   Description                                                              */
/*                                                                            */
/* ========================================================================== */

#include <stddef.h>

#include "lw_heap.h"

#define MIN_BLOCK_SIZE_X (sizeof(tHeader) + sizeof(tTrailer) + 8)

static tHeader Headers [16];

static char DataPool [1024];

static tHeader *pFreeChain;
static tHeader *pAllocChain;

//#define CALC_TRAILER_OFFSET(pHeader) ((uint8_t *)pHeader + pHeader->size)
 
static void init_block (tHeader *pHeader, uint32_t size)
{
    tTrailer *pTrailer;

    pHeader->guard = LW_HEAP_MAGIC;
    pHeader->size  = size;
    pHeader->flags.all = 0;
    pHeader->flags.free  = 1;
    pHeader->pNext = NULL;
    
    pTrailer = (tTrailer *) ( (uint8_t *)pHeader + pHeader->size - sizeof(tTrailer) );
    
    pTrailer->guard = LW_HEAP_MAGIC;
    pTrailer->size = size;
    pTrailer->flags.all = 0;
}
 
void lw_heap_init (void *p_pool_mem, uint32_t pool_size)
{
    tHeader *pHeader;

    pAllocChain = NULL;
    pHeader = (tHeader *)p_pool_mem;
    
    init_block (pHeader, pool_size);
    pFreeChain  = pHeader;
}

void *lw_malloc (uint32_t size)
{
    void *pResult = NULL;
    tHeader *pBlock;
    tHeader *pNewBlock;
    
    pBlock = pFreeChain;
    while (pBlock != NULL)
    {
        if (pBlock->size >= size)
        {
            // alloc here
            pNewBlock = pBlock;
            
            // if size of remaining block < min, alloc all to new block
            
        }
        else
            pBlock = pBlock->pNext;
    }
    
    return pResult;
}

void lw_free (void *mem)
{
}
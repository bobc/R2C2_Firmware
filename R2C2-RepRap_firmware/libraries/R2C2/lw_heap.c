/* ========================================================================== */
/*                                                                            */
/*   Filename.c                                                               */
/*   (c) 2001 Author                                                          */
/*                                                                            */
/*   Description                                                              */
/*                                                                            */
/* ========================================================================== */

#include <stddef.h>

#ifndef min
#define min(a,b) ((a)<(b) ? (a) : (b))
#endif

#ifndef max
#define max(a,b) ((a)>(b) ? (a) : (b))
#endif

#include "lw_heap.h"

// round up to multiple of 4
#define ROUND4(x) ( ((x)+3) & ~3 )
#define MIN_BLOCK_SIZE   (sizeof(tFreeHeader) + sizeof(tTrailer) + 8)
#define BLOCK_SIZE(size) (sizeof(tFreeHeader) + sizeof(tTrailer) + ROUND4(size))

typedef struct {
   uint8_t  *p_pool_base;
   uint32_t  pool_size;

   tFreeHeader *pFreeChain;
//   tAllocHeader *pAllocChain;

   uint32_t mem_used;
   uint32_t mem_free;
} tPool;

static tFreeHeader *pFreeChain;
//static tAllocHeader *pAllocChain;

static uint32_t mem_used;
static uint32_t mem_free;

//#define CALC_TRAILER_OFFSET(pHeader) ((uint8_t *)pHeader + pHeader->size)
 
static void init_block (tFreeHeader *pHeader, uint32_t size)
{
    tTrailer *pTrailer;

    pHeader->guard = LW_HEAP_MAGIC;
    pHeader->control.size  = size;  // ALLOC_FLAG is cleared
    pHeader->pNext = NULL;
    
    pTrailer = (tTrailer *) ( (uint8_t *)pHeader + pHeader->control.size - sizeof(tTrailer) );
    
    pTrailer->guard = LW_HEAP_MAGIC;
    pTrailer->control.size = size;  // ALLOC_FLAG is cleared
}
 
void lw_heap_init (void *p_pool_mem, uint32_t pool_size)
{
    tFreeHeader *pHeader;

//!    pAllocChain = NULL;
    pHeader = (tFreeHeader *)p_pool_mem;
    
    init_block (pHeader, pool_size);
    pFreeChain  = pHeader;

    mem_used = 0;
    mem_free = pool_size;
}

void *lw_malloc (uint32_t size)
{
    uint8_t *pResult = NULL;
    tFreeHeader *pBlock, *pPrevBlock;
    tFreeHeader *pNewBlock;
    
    pPrevBlock = NULL;
    pBlock = pFreeChain;
    while (pBlock != NULL)
    {
        if (pBlock->control.size >= size)
        {
            // alloc here
            pNewBlock = pBlock;

            uint32_t alloc_size = max (MIN_BLOCK_SIZE, BLOCK_SIZE (size));
            
            // if size of remaining block < min, alloc all to new block
            uint32_t remaining_size = pBlock->control.size - alloc_size;

            pBlock = (tFreeHeader *) ( (uint8_t *)pBlock + alloc_size );
            init_block (pBlock, remaining_size);

            // relink free chain
            pBlock->pNext = pNewBlock->pNext;
            if (pPrevBlock == NULL)
               pFreeChain = pBlock;
            else
               pPrevBlock->pNext = pBlock; 

            // now init new alloc block
            init_block (pNewBlock, alloc_size);
            pNewBlock->control.size |= ALLOC_FLAG;

            pResult = (void *)pNewBlock;
            pResult += sizeof (tAllocHeader);

            // update pool stats
            mem_free -= alloc_size;
            mem_used += alloc_size;

            break;
        }
        else
        {
            pPrevBlock = pBlock;
            pBlock = pBlock->pNext;
        }
    }
    
    return pResult;
}

void lw_free (void *mem)
{                      
    tFreeHeader *pBlock, *pNextBlock;

   // return block to free chain

   pBlock = (tFreeHeader *) ( (uint8_t *)mem - sizeof (tAllocHeader) );

   pBlock->control.size &= ~ALLOC_FLAG;

   // insert at front of free chain
   pBlock->pNext = pFreeChain;
   pFreeChain = pBlock;
      
   mem_free += pBlock->control.size;
   mem_used -= pBlock->control.size;

   /// *** TEST
   // coalesce free blocks
   pNextBlock = (tFreeHeader *) ( (uint8_t *)pBlock + pBlock->control.size );

   if ( (pNextBlock->control.size & ALLOC_FLAG)==0)
   {
      init_block (pBlock, pBlock->control.size + pNextBlock->control.size);
   }
}

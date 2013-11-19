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

#ifndef min
#define min(a,b) ((a)<(b) ? (a) : (b))
#endif

#ifndef max
#define max(a,b) ((a)>(b) ? (a) : (b))
#endif


// round up to multiple of 4
#define ROUND4(x) ( ((x)+3) & ~3 )
#define MIN_BLOCK_SIZE   (sizeof(tFreeHeader) + sizeof(tTrailer) + 8)
#define BLOCK_SIZE(size) (sizeof(tFreeHeader) + sizeof(tTrailer) + ROUND4(size))

#define ALLOC_BLOCK_SIZE(size) (sizeof(tAllocHeader) + sizeof(tTrailer) + ROUND4(size))

#define LW_HEAP_MAGIC 0xC001CAFE

#define ALLOC_FLAG (1ul << 31)
#define SIZE_MASK  (~ALLOC_FLAG)

//
typedef struct {
   uint32_t    size; // 31 bits
} tControl;

typedef struct free_header {
    uint32_t      guard;
    tControl      control;

    struct free_header * pNext;
//    struct free_header * pPrev;
} tFreeHeader;

typedef struct {
    uint32_t    guard;
    tControl    control;
} tAllocHeader;


typedef struct {
    uint32_t    guard;
    tControl    control;
} tTrailer;

typedef struct {
    tAllocHeader header;
    char         data [4];
    tTrailer     trailer;
} tBlock;

//
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

//

static void mark_free (tAllocHeader *pHeader)
{
    tTrailer *pTrailer;

    pHeader->control.size &= ~ALLOC_FLAG;

    pTrailer = (tTrailer *) ( (uint8_t *)pHeader + pHeader->control.size - sizeof(tTrailer) );
    pTrailer->control.size &= ~ALLOC_FLAG;
}

static void mark_alloc (tFreeHeader *pHeader)
{
    tTrailer *pTrailer;
    uint32_t size = pHeader->control.size & SIZE_MASK;

    pHeader->control.size |= ALLOC_FLAG;

    pTrailer = (tTrailer *) ( (uint8_t *)pHeader + size - sizeof(tTrailer) );
    pTrailer->control.size |= ALLOC_FLAG;
}


static void init_free_block (tFreeHeader *pHeader, uint32_t size)
{
    tTrailer *pTrailer;

    pHeader->guard = LW_HEAP_MAGIC;
    pHeader->control.size  = size;  // ALLOC_FLAG is cleared
    pHeader->pNext = NULL;
    
    pTrailer = (tTrailer *) ( (uint8_t *)pHeader + size - sizeof(tTrailer) );
    pTrailer->guard = LW_HEAP_MAGIC;
    pTrailer->control.size = size;  // ALLOC_FLAG is cleared
}
 
static void init_alloc_block (tAllocHeader *pHeader, uint32_t size)
{
    tTrailer *pTrailer;

    pHeader->guard = LW_HEAP_MAGIC;
    pHeader->control.size  = size | ALLOC_FLAG;
    
    pTrailer = (tTrailer *) ( (uint8_t *)pHeader + size - sizeof(tTrailer) );
    pTrailer->guard = LW_HEAP_MAGIC;
    pTrailer->control.size = size | ALLOC_FLAG;
}
 

void lw_heap_init (uint32_t *p_pool_mem, uint32_t pool_size)
{
   tAllocHeader *pHeader;

//!    pAllocChain = NULL;
    
   // create guard blocks
   mem_used = ALLOC_BLOCK_SIZE(0) * 2;
   mem_free = pool_size - mem_used;

   pHeader = (tAllocHeader *)p_pool_mem;
   init_alloc_block (pHeader, ALLOC_BLOCK_SIZE(0));

   pHeader = (tAllocHeader *) ( (uint8_t *)p_pool_mem + ALLOC_BLOCK_SIZE(0) + mem_free );
   init_alloc_block (pHeader, ALLOC_BLOCK_SIZE(0));

   // init the free block and link to free chain
   pFreeChain = (tFreeHeader *) ( (uint8_t *)p_pool_mem + ALLOC_BLOCK_SIZE(0));
   init_free_block (pFreeChain, mem_free);

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

            // determine the size
            uint32_t alloc_size = max (MIN_BLOCK_SIZE, BLOCK_SIZE (size));
            
            // if size of remaining block < min, alloc all to new block
            uint32_t remaining_size = pBlock->control.size - alloc_size;

            // init the free block
            pBlock = (tFreeHeader *) ( (uint8_t *)pBlock + alloc_size );
            init_free_block (pBlock, remaining_size);

            // relink free chain
            pBlock->pNext = pNewBlock->pNext;
            if (pPrevBlock == NULL)
               pFreeChain = pBlock;
            else
               pPrevBlock->pNext = pBlock; 

            // now init new alloc block
            init_alloc_block ((tAllocHeader *)pNewBlock, alloc_size);

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

   // mark block as free
   pBlock = (tFreeHeader *) ( (uint8_t *)mem - sizeof (tAllocHeader) );
   mark_free ((tAllocHeader *)pBlock);

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
      init_free_block (pBlock, pBlock->control.size + pNextBlock->control.size);
      pBlock->pNext = pNextBlock->pNext;
   }
}

/* ========================================================================== */
/*                                                                            */
/*   Filename.c                                                               */
/*   (c) 2001 Author                                                          */
/*                                                                            */
/*   Description                                                              */
/*                                                                            */
/* ========================================================================== */

#ifndef _LW_HEAP_H
#define _LW_HEAP_H

#include <stdint.h>

#define LW_HEAP_MAGIC 0xC001CAFE

#define ALLOC_FLAG (1ul << 31)

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

// p_pool_mem should be word aligned
void lw_heap_init (void *p_pool_mem, uint32_t pool_size);

void *lw_malloc (uint32_t size);

void lw_free (void *mem);

#endif

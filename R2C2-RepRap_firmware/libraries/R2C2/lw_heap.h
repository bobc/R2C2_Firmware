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

typedef union {
        uint32_t    all;
        
        uint32_t    free:1;
        uint32_t    allocated:1;
    } tFlags;

typedef struct header {
    uint32_t    guard;
    uint32_t    size;
    struct header * pNext;
    tFlags      flags;
} tHeader;

typedef struct {
    uint32_t    guard;
    uint32_t    size;
    tFlags      flags;
} tTrailer;

typedef struct {

    tHeader header;
    
    char data [4];
    
    tTrailer trailer;
} tBlock;


//

// p_pool_mem should be word aligned
void lw_heap_init (void *p_pool_mem, uint32_t pool_size);

void *lw_malloc (uint32_t size);

void lw_free (void *mem);

#endif

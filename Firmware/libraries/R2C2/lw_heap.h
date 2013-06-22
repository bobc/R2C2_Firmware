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

//

// p_pool_mem should be word aligned as some systems may not support byte alignment
void lw_heap_init (uint32_t *p_pool_mem, uint32_t pool_size);

// max size is actually 2^31
void *lw_malloc (uint32_t size);

void lw_free (void *mem);

#endif

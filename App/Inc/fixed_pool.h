/**
  ******************************************************************************
  * @file    fixed_pool.h
  * @brief   Generic static pool of fixed-size blocks (docs 7.1). Works exactly
  *          like the FreeRTOS-Plus-TCP network buffer pool
  *          (BufferAllocation_1.c): a counting semaphore says HOW MANY blocks
  *          are free, an intrusive List_t says WHICH ones, a critical section
  *          protects the list.
  ******************************************************************************
  * Every block type starts with a ListItem_t (offsetof == 0, checked where the
  * type is declared). The pool and the block array are owned and declared
  * static by the module that uses the pool (app_evt.c, report.c); this file
  * only provides the code and has no global variables of its own.
  */
#ifndef FIXED_POOL_H
#define FIXED_POOL_H

#include <stddef.h>

#include "FreeRTOS.h"
#include "list.h"
#include "semphr.h"

/* The application's mapping of the hooks below (D22), read first so that it
 * wins over the empty defaults. */
#include "app_trace_hooks.h"

/* Trace hooks, like iptraceFAILED_TO_OBTAIN_NETWORK_BUFFER (BA1:318). Empty
 * unless app_trace_hooks.h (or the build) defines them. */
#ifndef traceFIXED_POOL_GET_FAILED
    #define traceFIXED_POOL_GET_FAILED( pxPool )
#endif
#ifndef traceFIXED_POOL_INVALID_BLOCK
    #define traceFIXED_POOL_INVALID_BLOCK( pxPool, pvBlock )
#endif
#ifndef traceFIXED_POOL_DOUBLE_RELEASE
    #define traceFIXED_POOL_DOUBLE_RELEASE( pxPool, pvBlock )
#endif

/* What the owner tells Pool_Init about its block array. */
typedef struct xFIXED_POOL_CFG
{
    void * pvBlocks;            /* static array of uxBlockCount blocks        */
    size_t uxBlockSize;         /* bytes per block, includes the ListItem_t   */
    UBaseType_t uxBlockCount;   /* number of blocks (semaphore max and start) */
    UBaseType_t uxIsrReserve;   /* an ISR takes only while free > this        */
} FixedPoolCfg_t;

/* One pool. Must be zero-initialised (static) before Pool_Init: xSem == NULL
 * means "not initialised yet", like xNetworkBufferSemaphore == NULL. */
typedef struct xFIXED_POOL
{
    StaticSemaphore_t xSemBuf;  /* storage of the counting semaphore          */
    SemaphoreHandle_t xSem;     /* count = number of free blocks              */
    List_t xFreeList;           /* the free blocks, linked by their ListItem_t */
    uint8_t * pucBase;          /* first block                                */
    size_t uxBlockSize;
    UBaseType_t uxBlockCount;
    UBaseType_t uxIsrReserve;
    UBaseType_t uxMinFree;      /* lowest free count since Pool_Init          */
} FixedPool_t;

/* Creates the semaphore and puts every block in the free list. Call once,
 * before the scheduler starts. A second call on an initialised pool changes
 * nothing and returns pdPASS (as xNetworkBuffersInitialise). Returns pdFAIL
 * for a NULL pool or an invalid configuration. */
BaseType_t Pool_Init( FixedPool_t * pxPool, const FixedPoolCfg_t * pxCfg );

/* Takes one free block (task context only). Waits at most xWait ticks for one
 * to be released; the application always passes 0. Returns the block, or NULL
 * when the pool is empty, not initialised, or its free list is corrupt. */
void * Pool_Get( FixedPool_t * pxPool, TickType_t xWait );

/* Gives a block back (task context only); never waits. Returns pdPASS, or
 * pdFAIL when pvBlock is NULL, not a block of this pool, or already free
 * (a double release): then neither the free list nor the semaphore changes. */
BaseType_t Pool_Release( FixedPool_t * pxPool, void * pvBlock );

/* Number of free blocks right now (any context). 0 for a NULL or not
 * initialised pool. */
UBaseType_t Pool_GetFree( const FixedPool_t * pxPool );

/* Lowest number of free blocks seen by Pool_Get since Pool_Init (any
 * context). 0 for a NULL or not initialised pool. */
UBaseType_t Pool_GetMinFree( const FixedPool_t * pxPool );

/* ISR versions, as BufferAllocation_1.c has them. No ISR takes or gives a
 * block in this application, so they are compiled only with APP_POOL_ISR 1
 * (report 062). Interrupt priority must be 5..15
 * (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY). Neither ever waits. */
#ifndef APP_POOL_ISR
    #define APP_POOL_ISR    0
#endif

#if ( APP_POOL_ISR == 1 )

/* Takes a block only while more than uxIsrReserve blocks are free, so the
 * last uxIsrReserve blocks stay for tasks. Returns NULL otherwise. Does not
 * update the minimum-free statistic (as pxNetworkBufferGetFromISR). */
void * Pool_GetFromISR( FixedPool_t * pxPool );

/* Same checks and result as Pool_Release. *pxHigherPriorityTaskWoken is set
 * to pdTRUE when giving the semaphore woke a higher priority task (never set
 * to pdFALSE; may be NULL); the ISR then ends with portYIELD_FROM_ISR(). */
BaseType_t Pool_ReleaseFromISR( FixedPool_t * pxPool, void * pvBlock,
                                BaseType_t * pxHigherPriorityTaskWoken );

#endif /* APP_POOL_ISR */

#endif /* FIXED_POOL_H */

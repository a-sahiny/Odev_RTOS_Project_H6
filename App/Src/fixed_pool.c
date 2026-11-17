/**
  ******************************************************************************
  * @file    fixed_pool.c
  * @brief   Generic static pool of fixed-size blocks, built the same way as
  *          FreeRTOS-Plus-TCP's BufferAllocation_1.c (docs 7.1).
  ******************************************************************************
  * RTOS objects (one set per pool, stored inside FixedPool_t):
  *   xSem       counting semaphore, max = start = block count
  *              taken by Pool_Get (consumer), given by Pool_Release (producer)
  *   xFreeList  kernel List_t of the free blocks, guarded by a critical section
  * Line numbers "BA1:nnn" refer to
  * ThirdParty/FreeRTOS-Plus-TCP/source/portable/BufferManagement/BufferAllocation_1.c
  */
#include "fixed_pool.h"

#include <stdint.h>

#include "task.h"

static BaseType_t prvCfgIsValid( const FixedPoolCfg_t * pxCfg );
static UBaseType_t prvBlockIndex( const FixedPool_t * pxPool, const void * pvBlock );

/*---------------------------------------------------------------------------*/
/* Pool_Init  (BA1:170-235, xNetworkBuffersInitialise)                       */
/*---------------------------------------------------------------------------*/
BaseType_t Pool_Init( FixedPool_t * pxPool, const FixedPoolCfg_t * pxCfg )
{
    BaseType_t xReturn = pdFAIL;
    UBaseType_t x;
    ListItem_t * pxItem;

    if( ( pxPool != NULL ) && ( prvCfgIsValid( pxCfg ) == pdPASS ) )
    {
        /* Only initialise once (BA1:177). */
        if( pxPool->xSem == NULL )
        {
            /* Start count = max count = number of blocks: all are free
             * (BA1:185-189). */
            pxPool->xSem = xSemaphoreCreateCountingStatic( pxCfg->uxBlockCount,
                                                           pxCfg->uxBlockCount,
                                                           &( pxPool->xSemBuf ) );
            configASSERT( pxPool->xSem != NULL );

            if( pxPool->xSem != NULL )
            {
                pxPool->pucBase = ( uint8_t * ) pxCfg->pvBlocks;
                pxPool->uxBlockSize = pxCfg->uxBlockSize;
                pxPool->uxBlockCount = pxCfg->uxBlockCount;
                pxPool->uxIsrReserve = pxCfg->uxIsrReserve;

                vListInitialise( &( pxPool->xFreeList ) );   /* BA1:201 */

                for( x = 0U; x < pxCfg->uxBlockCount; x++ )
                {
                    /* The list item is the first member of every block, so
                     * its address is the block address. */
                    pxItem = ( ListItem_t * ) ( pxPool->pucBase + ( ( size_t ) x * pxCfg->uxBlockSize ) );

                    /* Initialise and set the owner of the list item (BA1:214-215). */
                    vListInitialiseItem( pxItem );
                    listSET_LIST_ITEM_OWNER( pxItem, pxItem );

                    /* BA1 leaves the item value to the zeroed static array;
                     * set it explicitly so vListInsert keeps array order. */
                    listSET_LIST_ITEM_VALUE( pxItem, 0U );

                    /* Currently, all blocks are available (BA1:218). */
                    vListInsert( &( pxPool->xFreeList ), pxItem );
                }

                pxPool->uxMinFree = pxCfg->uxBlockCount;     /* BA1:221 */
            }
        }

        xReturn = ( pxPool->xSem != NULL ) ? pdPASS : pdFAIL;
    }

    return xReturn;
}

/*---------------------------------------------------------------------------*/
/* Pool_Get  (BA1:238-323, pxGetNetworkBufferWithDescriptor)                 */
/*---------------------------------------------------------------------------*/
void * Pool_Get( FixedPool_t * pxPool, TickType_t xWait )
{
    void * pvReturn = NULL;
    ListItem_t * pxItem = NULL;
    BaseType_t xInvalid = pdFALSE;
    UBaseType_t uxCount;

    if( ( pxPool != NULL ) && ( pxPool->xSem != NULL ) )
    {
        /* If the semaphore can be taken, the free list holds a block
         * (BA1:248-250). */
        if( xSemaphoreTake( pxPool->xSem, xWait ) == pdPASS )
        {
            /* Protect the list: other tasks (and ISRs) use it too. BA1 uses
             * ipconfigBUFFER_ALLOC_LOCK(), which defaults to this call
             * (BA1:101-102, 252-268). */
            taskENTER_CRITICAL();
            {
                pxItem = ( ListItem_t * ) listGET_OWNER_OF_HEAD_ENTRY( &( pxPool->xFreeList ) );

                if( ( prvBlockIndex( pxPool, pxItem ) != 0U ) &&
                    ( listIS_CONTAINED_WITHIN( &( pxPool->xFreeList ), pxItem ) != pdFALSE ) )
                {
                    ( void ) uxListRemove( pxItem );
                }
                else
                {
                    xInvalid = pdTRUE;
                }
            }
            taskEXIT_CRITICAL();

            if( xInvalid == pdTRUE )
            {
                /* Corrupt free list (BA1:270-281). The semaphore count is
                 * already taken and is not given back, as in BA1. */
                traceFIXED_POOL_INVALID_BLOCK( pxPool, pxItem );
                configASSERT( xInvalid == pdFALSE );
            }
            else
            {
                /* Reading UBaseType_t, no critical section needed; latch the
                 * lowest free count since boot (BA1:284-292). */
                uxCount = listCURRENT_LIST_LENGTH( &( pxPool->xFreeList ) );

                if( pxPool->uxMinFree > uxCount )
                {
                    pxPool->uxMinFree = uxCount;
                }

                pvReturn = pxItem;
            }
        }
        else
        {
            /* Pool empty (BA1:314-319). */
            traceFIXED_POOL_GET_FAILED( pxPool );
        }
    }

    return pvReturn;
}

/*---------------------------------------------------------------------------*/
/* Pool_Release  (BA1:383-421, vReleaseNetworkBufferAndDescriptor)           */
/*---------------------------------------------------------------------------*/
BaseType_t Pool_Release( FixedPool_t * pxPool, void * pvBlock )
{
    BaseType_t xReturn = pdFAIL;
    BaseType_t xAlreadyFree;
    ListItem_t * pxItem = ( ListItem_t * ) pvBlock;   /* first member of the block */

    if( ( pxPool == NULL ) || ( pxPool->xSem == NULL ) || ( pvBlock == NULL ) )
    {
        /* Nothing to give back. */
    }
    else if( prvBlockIndex( pxPool, pvBlock ) == 0U )
    {
        /* Not a block of this pool (BA1:387-390): leave it untouched. */
        traceFIXED_POOL_INVALID_BLOCK( pxPool, pvBlock );
        configASSERT( pdFALSE );
    }
    else
    {
        /* Put the block back in the free list BEFORE giving the semaphore:
         * a task that takes the semaphore must find a block (BA1:393-406). */
        taskENTER_CRITICAL();
        {
            xAlreadyFree = listIS_CONTAINED_WITHIN( &( pxPool->xFreeList ), pxItem );

            if( xAlreadyFree == pdFALSE )
            {
                vListInsertEnd( &( pxPool->xFreeList ), pxItem );
            }
        }
        taskEXIT_CRITICAL();

        if( xAlreadyFree != pdFALSE )
        {
            /* Double release (BA1:408-412): inserting the item again would
             * corrupt the list and hand one block to two owners. */
            traceFIXED_POOL_DOUBLE_RELEASE( pxPool, pvBlock );
        }
        else
        {
            ( void ) xSemaphoreGive( pxPool->xSem );       /* BA1:415 */
            xReturn = pdPASS;
        }
    }

    return xReturn;
}

/*---------------------------------------------------------------------------*/
/* Pool_GetFree  (BA1:430-433, uxGetNumberOfFreeNetworkBuffers)              */
/*---------------------------------------------------------------------------*/
UBaseType_t Pool_GetFree( const FixedPool_t * pxPool )
{
    UBaseType_t uxFree = 0U;

    if( ( pxPool != NULL ) && ( pxPool->xSem != NULL ) )
    {
        /* One aligned word, read without a critical section as in BA1. */
        uxFree = listCURRENT_LIST_LENGTH( &( pxPool->xFreeList ) );
    }

    return uxFree;
}

/*---------------------------------------------------------------------------*/
/* Pool_GetMinFree  (BA1:424-427, uxGetMinimumFreeNetworkBuffers)            */
/*---------------------------------------------------------------------------*/
UBaseType_t Pool_GetMinFree( const FixedPool_t * pxPool )
{
    UBaseType_t uxMinFree = 0U;

    if( ( pxPool != NULL ) && ( pxPool->xSem != NULL ) )
    {
        uxMinFree = pxPool->uxMinFree;
    }

    return uxMinFree;
}

#if ( APP_POOL_ISR == 1 )

/*---------------------------------------------------------------------------*/
/* Pool_GetFromISR  (BA1:326-361, pxNetworkBufferGetFromISR)                 */
/*---------------------------------------------------------------------------*/
void * Pool_GetFromISR( FixedPool_t * pxPool )
{
    void * pvReturn = NULL;
    ListItem_t * pxItem = NULL;
    BaseType_t xInvalid = pdFALSE;
    UBaseType_t uxSavedInterruptStatus;

    if( ( pxPool != NULL ) && ( pxPool->xSem != NULL ) )
    {
        /* Take only while more than uxIsrReserve blocks are free, so a fast
         * interrupt cannot empty the pool and starve the tasks (BA1:334-339;
         * BA1 uses one constant threshold of 3). */
        if( uxQueueMessagesWaitingFromISR( ( QueueHandle_t ) pxPool->xSem ) > pxPool->uxIsrReserve )
        {
            if( xSemaphoreTakeFromISR( pxPool->xSem, NULL ) == pdPASS )   /* BA1:341 */
            {
                /* ipconfigBUFFER_ALLOC_LOCK_FROM_ISR() in BA1 (BA1:94-96, 344). */
                uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();
                {
                    pxItem = ( ListItem_t * ) listGET_OWNER_OF_HEAD_ENTRY( &( pxPool->xFreeList ) );

                    /* BA1 removes the head without checking; the same check
                     * as Pool_Get is made here. */
                    if( ( prvBlockIndex( pxPool, pxItem ) != 0U ) &&
                        ( listIS_CONTAINED_WITHIN( &( pxPool->xFreeList ), pxItem ) != pdFALSE ) )
                    {
                        ( void ) uxListRemove( pxItem );           /* BA1:347 */
                    }
                    else
                    {
                        xInvalid = pdTRUE;
                    }
                }
                taskEXIT_CRITICAL_FROM_ISR( uxSavedInterruptStatus );

                if( xInvalid == pdTRUE )
                {
                    traceFIXED_POOL_INVALID_BLOCK( pxPool, pxItem );
                    configASSERT( xInvalid == pdFALSE );
                }
                else
                {
                    pvReturn = pxItem;
                }
            }
        }

        if( pvReturn == NULL )
        {
            traceFIXED_POOL_GET_FAILED( pxPool );           /* BA1:355-358 */
        }
    }

    return pvReturn;
}

/*---------------------------------------------------------------------------*/
/* Pool_ReleaseFromISR  (BA1:364-380, vNetworkBufferReleaseFromISR)          */
/*---------------------------------------------------------------------------*/
BaseType_t Pool_ReleaseFromISR( FixedPool_t * pxPool, void * pvBlock,
                                BaseType_t * pxHigherPriorityTaskWoken )
{
    BaseType_t xReturn = pdFAIL;
    BaseType_t xAlreadyFree;
    ListItem_t * pxItem = ( ListItem_t * ) pvBlock;   /* first member of the block */
    UBaseType_t uxSavedInterruptStatus;

    if( ( pxPool == NULL ) || ( pxPool->xSem == NULL ) || ( pvBlock == NULL ) )
    {
        /* Nothing to give back. */
    }
    else if( prvBlockIndex( pxPool, pvBlock ) == 0U )
    {
        /* BA1 does not check the address here; Pool_Release does. */
        traceFIXED_POOL_INVALID_BLOCK( pxPool, pvBlock );
        configASSERT( pdFALSE );
    }
    else
    {
        /* Back in the list before the semaphore is given (BA1:368-374). */
        uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();
        {
            /* BA1 inserts without this check; a second insert of an item that
             * is already listed would corrupt the list. */
            xAlreadyFree = listIS_CONTAINED_WITHIN( &( pxPool->xFreeList ), pxItem );

            if( xAlreadyFree == pdFALSE )
            {
                vListInsertEnd( &( pxPool->xFreeList ), pxItem );   /* BA1:372 */
            }
        }
        taskEXIT_CRITICAL_FROM_ISR( uxSavedInterruptStatus );

        if( xAlreadyFree != pdFALSE )
        {
            traceFIXED_POOL_DOUBLE_RELEASE( pxPool, pvBlock );
        }
        else
        {
            /* BA1:376 */
            ( void ) xSemaphoreGiveFromISR( pxPool->xSem, pxHigherPriorityTaskWoken );
            xReturn = pdPASS;
        }
    }

    return xReturn;
}

#endif /* APP_POOL_ISR */

/*---------------------------------------------------------------------------*/
/* prvBlockIndex  (BA1:141-152, bIsValidNetworkDescriptor)                   */
/* Returns 1 + the block's index, or 0 when pvBlock is not the start of one  */
/* of this pool's blocks. NULL, addresses below or above the array and       */
/* addresses inside a block all give 0.                                      */
/*---------------------------------------------------------------------------*/
static UBaseType_t prvBlockIndex( const FixedPool_t * pxPool, const void * pvBlock )
{
    UBaseType_t uxIndex = 0U;

    /* Below the array the unsigned difference wraps to a huge value, so one
     * comparison rejects both sides (as BA1:143-146). */
    uintptr_t uxOffset = ( uintptr_t ) pvBlock - ( uintptr_t ) pxPool->pucBase;

    if( ( uxOffset < ( ( uintptr_t ) pxPool->uxBlockSize * pxPool->uxBlockCount ) ) &&
        ( ( uxOffset % pxPool->uxBlockSize ) == 0U ) )
    {
        uxIndex = ( UBaseType_t ) ( uxOffset / pxPool->uxBlockSize ) + 1U;
    }

    return uxIndex;
}

/*---------------------------------------------------------------------------*/
/* prvCfgIsValid: what the loop in Pool_Init relies on                       */
/*---------------------------------------------------------------------------*/
static BaseType_t prvCfgIsValid( const FixedPoolCfg_t * pxCfg )
{
    BaseType_t xValid = pdFAIL;

    if( ( pxCfg != NULL ) &&
        ( pxCfg->pvBlocks != NULL ) &&
        ( pxCfg->uxBlockCount > 0U ) &&
        /* each block holds at least its list item ...                       */
        ( pxCfg->uxBlockSize >= sizeof( ListItem_t ) ) &&
        /* ... and every block start is aligned for a ListItem_t             */
        ( ( ( uintptr_t ) pxCfg->pvBlocks % _Alignof( ListItem_t ) ) == 0U ) &&
        ( ( pxCfg->uxBlockSize % _Alignof( ListItem_t ) ) == 0U ) &&
        /* an ISR must be able to take at least one block                     */
        ( pxCfg->uxIsrReserve < pxCfg->uxBlockCount ) )
    {
        xValid = pdPASS;
    }

    return xValid;
}

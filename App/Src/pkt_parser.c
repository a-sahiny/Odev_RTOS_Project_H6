/**
  ******************************************************************************
  * @file    pkt_parser.c
  * @brief   Parser tasks of the shared pipeline (docs D3, 3.1 steps 5a / 5b,
  *          5.1, 16.5, 17.1): one record (message) of sbCrit / sbLow -> one
  *          validated AppDesc_t in ctrlQueue / workQueue. Künye: report 060;
  *          MessageBuffer: report 062 (B6).
  ******************************************************************************
  * RTOS objects owned:
  *   critParser  static task, APP_PRIO_CRIT_PARSER (6, above the IP task:
  *               D16), APP_STACK_CRIT_PARSER_WORDS. Waits on sbCrit for a
  *               record, <= APP_PARSER_WAIT_MS.
  *   lowParser   static task, APP_PRIO_LOW_PARSER (1),
  *               APP_STACK_LOW_PARSER_WORDS. Waits on sbLow,
  *               <= APP_PARSER_WAIT_MS.
  * Task notification used (no object): index 0 of each parser, taken by the
  * kernel while it blocks in xMessageBufferReceive (stream_buffer.c:1133).
  * Used, not owned: sbCrit / sbLow (this file is the only reader, 1000 ms),
  * the descriptor pools and the event queues of app_evt.c (wait 0).
  * Not RTOS objects: each lane's state (ParserState_t) is read and written by
  * its own parser task only.
  */
#include "pkt_parser.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "message_buffer.h"

#include "stm32f7xx.h"      /* DWT->CYCCNT (M2) */

#include "app_config.h"
#include "app_stats.h"
#include "app_evt.h"        /* AppDesc_Get / Release, AppEvt_Send */
#include "app_types.h"      /* UdpRecHdr_t, eAppClass_t */
#include "console.h"        /* Console_TakeHoldMs (P5) */
#include "pkt_format.h"     /* Pkt_Decode, PktView_t */
#include "udp_in.h"         /* UdpIn_GetStream */

/* Offsets inside one lane's L4 counter group (P9). Both groups in
 * app_stats.h have exactly this order; the asserts below hold it. */
typedef enum ePARSER_STAT
{
    eParserStatRecRead = 0,
    eParserStatPktOk,
    eParserStatPktShort,        /* ePktErrShort .. ePktErrFlags in order */
    eParserStatPktSof,
    eParserStatPktVersion,
    eParserStatPktLength,
    eParserStatPktCrc,
    eParserStatPktFlags,
    eParserStatPktWrongPort,
    eParserStatPktTypeLen,
    eParserStatSeqDup,
    eParserStatSeqGap,
    eParserStatSeqOld,
    eParserStatSeqResync,
    eParserStatEvtPosted,
    eParserStatQueueFull,       /* CTRLQ_FULL / WORKQ_FULL */
    eParserStatCount
} eParserStat_t;

/* One accepted packet TYPE of a lane (P3): its length rule, the event it
 * becomes and its SEQ channel (docs 15.2, 16.5). */
typedef struct xPARSER_ROUTE
{
    uint8_t ucType;
    uint8_t ucMinLen;               /* payload bytes                          */
    uint8_t ucMaxLen;
    uint8_t ucCountStride;          /* != 0: LEN = 1 + stride * payload[ 0 ],
                                     * payload[ 0 ] = 1..APP_ENV_MAX_CHANNELS (ENV) */
    eAppEvent_t eEvent;
    uint8_t ucChannel;              /* index into the lane's SEQ table        */
} ParserRoute_t;

/* SEQ state of one channel (docs 16.5, P2). */
typedef struct xPARSER_SEQ
{
    uint16_t usLast;                /* last accepted SEQ                      */
    uint8_t ucSeen;                 /* 0 until the channel's first packet     */
    uint8_t ucOldRun;               /* consecutive old packets                */
} ParserSeq_t;

#define PARSER_MAX_CHANNELS     3U  /* low lane: ENV, BULK, CAL               */

/* What a parser task owns and changes: only that task touches it. */
typedef struct xPARSER_STATE
{
    MessageBufferHandle_t xBuffer;              /* read once at task start  */
    TaskHandle_t xTask;                         /* NULL = not created yet   */
    ParserSeq_t xSeq[ PARSER_MAX_CHANNELS ];
    uint8_t ucRecord[ APP_REC_MAX_BYTES ];      /* header + UDP payload     */
} ParserState_t;

/* What differs between the two parser tasks (D3: the code is the same). */
typedef struct xPARSER_CFG
{
    const char * pcName;
    eAppClass_t eClass;             /* lane: its MessageBuffer and pool      */
    eAppTaskId_t eTask;             /* for Stats_SetTaskHandle               */
    UBaseType_t uxPriority;
    uint32_t ulStackWords;
    StackType_t * puxStack;
    StaticTask_t * pxTcb;
    ParserState_t * pxState;
    eStatId_t eStatFirst;           /* first counter of the lane's L4 group  */
    uint8_t ucMeasureM2;            /* 1: M1 -> M2 histogram (P6)            */
    uint8_t ucDropStale;            /* 1: SEQ_DUP / SEQ_OLD packets are dropped (data, P2) */
    const ParserRoute_t * pxRoutes; /* the TYPEs this lane accepts (P3)      */
    uint8_t ucRouteCount;
    eAppTarget_t eTarget;           /* ctrlQueue / workQueue                 */
    uint8_t ucApplyHold;            /* 1: HOLD from the console (P5, T03)    */
    uint8_t ucDataCap;              /* payload bytes a descriptor of the lane's pool holds */
    const ParserRoute_t * pxReplyRoute; /* != NULL: a TYPE nobody knows and a wrong LEN go to
                                         * the lane's handler, which answers them (commands:
                                         * E_TYPE / E_LEN, docs 16.2). NULL: counted and
                                         * dropped here (data gets no reply)                 */
} ParserCfg_t;

/* Commands, port 5000: one SEQ channel. LEN, value ranges and REQ_ID stay
 * with cmd.c, which answers with an error code (docs 16.2, 16.3; report 085):
 * the lengths below only document the format, prvRoute does not apply them
 * on this lane. */
static const ParserRoute_t xCritRoutes[] =
{
    { APP_TYPE_CMD_START,     0U,                0U,                0U, eAppCmdEvent, 0U },
    { APP_TYPE_CMD_STOP,      0U,                0U,                0U, eAppCmdEvent, 0U },
    { APP_TYPE_CMD_SET_RATE,  APP_LEN_SET_RATE,  APP_LEN_SET_RATE,  0U, eAppCmdEvent, 0U },
    { APP_TYPE_CMD_GET_STATS, APP_LEN_GET_STATS, APP_LEN_GET_STATS, 0U, eAppCmdEvent, 0U },
    { APP_TYPE_CMD_GET_IMU,   APP_LEN_GET_IMU,   APP_LEN_GET_IMU,   0U, eAppCmdEvent, 0U }    /* report 086 */
};

/* A packet on the command port whose TYPE no lane knows: it still goes to
 * ctrlTask as a command (same SEQ channel), and cmd.c answers E_TYPE. */
static const ParserRoute_t xCmdUnknownRoute = { 0U, 0U, APP_PKT_MAX_PAYLOAD, 0U, eAppCmdEvent, 0U };

/* Data, port 5001: one SEQ channel per stream (docs 16.5). */
static const ParserRoute_t xLowRoutes[] =
{
    { APP_TYPE_ENV,  3U, ( uint8_t ) ( 1U + ( 2U * APP_ENV_MAX_CHANNELS ) ), 2U, eAppEnvEvent, 0U },
    { APP_TYPE_BULK, ( uint8_t ) ( APP_BULK_SEED_BYTES + APP_BULK_MIN_PATTERN ),
                     ( uint8_t ) ( APP_BULK_SEED_BYTES + APP_BULK_MAX_PATTERN ), 0U, eAppBulkEvent, 1U },
    { APP_TYPE_CAL,  APP_LEN_CAL, APP_LEN_CAL, 0U, eAppCalEvent, 2U }
};

static StaticTask_t xCritParserTcb;
static StackType_t xCritParserStack[ APP_STACK_CRIT_PARSER_WORDS ];
static ParserState_t xCritState;

static StaticTask_t xLowParserTcb;
static StackType_t xLowParserStack[ APP_STACK_LOW_PARSER_WORDS ];
static ParserState_t xLowState;

static const ParserCfg_t xParserCfgs[ eAppClassCount ] =
{
    {   /* eAppClassCrit */
        "critParser", eAppClassCrit, eAppTaskCritParser, APP_PRIO_CRIT_PARSER,
        APP_STACK_CRIT_PARSER_WORDS, xCritParserStack, &xCritParserTcb, &xCritState,
        eStatRecReadCrit, 1U, 0U, xCritRoutes, ( uint8_t ) ( sizeof( xCritRoutes ) / sizeof( xCritRoutes[ 0 ] ) ),
        eAppTargetCtrl, 0U, ( uint8_t ) APP_CRIT_DESC_DATA_BYTES, &xCmdUnknownRoute
    },
    {   /* eAppClassLow */
        "lowParser", eAppClassLow, eAppTaskLowParser, APP_PRIO_LOW_PARSER,
        APP_STACK_LOW_PARSER_WORDS, xLowParserStack, &xLowParserTcb, &xLowState,
        eStatRecReadLow, 0U, 1U, xLowRoutes, ( uint8_t ) ( sizeof( xLowRoutes ) / sizeof( xLowRoutes[ 0 ] ) ),
        eAppTargetWork, 1U, ( uint8_t ) APP_LOW_DESC_DATA_BYTES, NULL
    }
};

static void prvParserTask( void * pvParameters );
static size_t prvReadRecord( const ParserCfg_t * pxCfg, UdpRecHdr_t * pxHdr );
static const ParserRoute_t * prvCheckPacket( const ParserCfg_t * pxCfg, size_t uxPayload, PktView_t * pxView );
static const ParserRoute_t * prvRoute( const ParserCfg_t * pxCfg, const PktView_t * pxView );
static const ParserRoute_t * prvFindRoute( const ParserCfg_t * pxCfg, uint8_t ucType );
static BaseType_t prvIsForeignType( const ParserCfg_t * pxCfg, uint8_t ucType );
static BaseType_t prvTrackSeq( const ParserCfg_t * pxCfg, uint8_t ucChannel, uint16_t usSeq );
static void prvDeliver( const ParserCfg_t * pxCfg, const ParserRoute_t * pxRoute, const UdpRecHdr_t * pxHdr,
                        const PktView_t * pxView );
static void prvApplyHold( void );
static void prvCount( const ParserCfg_t * pxCfg, eParserStat_t eStat );

/* Pkt_Decode's reasons map onto the PKT_* counters in the same order. */
_Static_assert( ( ePktErrFlags - ePktErrShort ) == ( eParserStatPktFlags - eParserStatPktShort ), "decode reasons = PKT_* order" );
_Static_assert( ( APP_CRIT_DESC_DATA_BYTES >= APP_LEN_SET_RATE ) && ( APP_CRIT_DESC_DATA_BYTES >= APP_LEN_GET_STATS ),
                "every command payload fits a crit descriptor" );
_Static_assert( ( 1U + ( 2U * APP_ENV_MAX_CHANNELS ) ) <= APP_LOW_DESC_DATA_BYTES, "ENV fits a low descriptor" );
_Static_assert( APP_LEN_CAL <= APP_LOW_DESC_DATA_BYTES, "CAL fits a low descriptor" );
_Static_assert( ( APP_CRIT_DESC_DATA_BYTES <= UINT8_MAX ) && ( APP_LOW_DESC_DATA_BYTES <= UINT8_MAX ), "ucDataCap is 8 bits" );

/* P9: the two L4 groups have the same layout. */
_Static_assert( ( eStatRecReadCrit + eParserStatQueueFull ) == eStatCtrlqFull, "crit L4 group layout" );
_Static_assert( ( eStatRecReadLow + eParserStatQueueFull ) == eStatWorkqFull, "low L4 group layout" );
_Static_assert( ( eStatRecReadCrit + eParserStatPktTypeLen ) == eStatPktTypeLenCrit, "crit L4 group layout" );
_Static_assert( ( eStatRecReadLow + eParserStatPktTypeLen ) == eStatPktTypeLenLow, "low L4 group layout" );
_Static_assert( ( eStatRecReadCrit + eParserStatEvtPosted ) == eStatEvtPostedCrit, "crit L4 group layout" );
_Static_assert( ( eStatRecReadLow + eParserStatSeqResync ) == eStatSeqResyncLow, "low L4 group layout" );
_Static_assert( ( eAppClassCrit == 0 ) && ( eAppClassLow == 1 ), "xParserCfgs order" );
_Static_assert( sizeof( "critParser" ) <= configMAX_TASK_NAME_LEN, "task name fits" );
_Static_assert( APP_STACK_CRIT_PARSER_WORDS == APP_STACK_LOW_PARSER_WORDS, "same code, same stack" );

/*---------------------------------------------------------------------------*/
BaseType_t Parser_Init( void )
{
    UBaseType_t x;
    const ParserCfg_t * pxCfg;

    for( x = 0U; x < ( UBaseType_t ) eAppClassCount; x++ )
    {
        pxCfg = &xParserCfgs[ x ];

        /* The task reads its MessageBuffer at start: UdpIn_Init must have
         * run (P8). Caught here, before the scheduler. */
        configASSERT( UdpIn_GetStream( pxCfg->eClass ) != NULL );

        if( pxCfg->pxState->xTask == NULL )
        {
            /* Static TCB and stack: the kernel asserts NULL buffers itself
             * (tasks.c:1276-1290), the only way to get NULL (report 062,
             * B9). The configuration is the task parameter: one function,
             * two tasks (D3). */
            pxCfg->pxState->xTask = xTaskCreateStatic( prvParserTask,
                                                       pxCfg->pcName,
                                                       pxCfg->ulStackWords,
                                                       ( void * ) pxCfg,
                                                       pxCfg->uxPriority,
                                                       pxCfg->puxStack,
                                                       pxCfg->pxTcb );

            /* For the stack high-water mark in the snapshot. */
            Stats_SetTaskHandle( pxCfg->eTask, pxCfg->pxState->xTask );
        }
    }

    return pdPASS;
}

/*---------------------------------------------------------------------------*/
static void prvParserTask( void * pvParameters )
{
    const ParserCfg_t * const pxCfg = ( const ParserCfg_t * ) pvParameters;
    UdpRecHdr_t xHdr;
    PktView_t xView;
    const ParserRoute_t * pxRoute;
    size_t uxRecord;

    pxCfg->pxState->xBuffer = UdpIn_GetStream( pxCfg->eClass );

    for( ; ; )
    {
        uxRecord = prvReadRecord( pxCfg, &xHdr );

        if( uxRecord != 0U )
        {
            /* P5: lowParser only, with a record in hand and before it is
             * processed, so the record that woke it waits as well (T03). */
            if( pxCfg->ucApplyHold != 0U )
            {
                prvApplyHold();
            }

            pxRoute = prvCheckPacket( pxCfg, uxRecord - APP_REC_HEADER_BYTES, &xView );

            if( ( pxRoute != NULL ) && ( prvTrackSeq( pxCfg, pxRoute->ucChannel, xView.usSeq ) == pdTRUE ) )
            {
                prvDeliver( pxCfg, pxRoute, &xHdr, &xView );
            }
        }
        /* 0: time out (P10); wait for the next record. */
    }
}

/*---------------------------------------------------------------------------*/
/* prvDeliver: the packet into a descriptor of the lane's pool, the event    */
/* into the lane's queue (docs 3.1 step 5, 6.3). Never waits.                */
/*---------------------------------------------------------------------------*/
static void prvDeliver( const ParserCfg_t * pxCfg, const ParserRoute_t * pxRoute, const UdpRecHdr_t * pxHdr,
                        const PktView_t * pxView )
{
    AppDesc_t * pxDesc;
    AppEvent_t xEvent;
    size_t uxCopy;

    /* Wait 0. NULL: the pool is empty; the pool hook has already counted
     * CRIT_DESC_EMPTY / LOW_DESC_EMPTY (D23). */
    pxDesc = AppDesc_Get( pxCfg->eClass );

    if( pxDesc != NULL )
    {
        /* Everything the handler needs, so it never parses again (docs 5.2).
         * ucClass was set by AppDesc_Get. usLen is the packet's LEN. On the
         * data lane the route table keeps every LEN within the pool's
         * capacity (static asserts above). On the command lane a wrong LEN
         * is delivered too, so that cmd.c can answer E_LEN: then only the
         * bytes that fit are copied, and nobody reads past them (cmd.c reads
         * the payload only after it has checked the length). */
        uxCopy = ( pxView->ucLen <= pxCfg->ucDataCap ) ? ( size_t ) pxView->ucLen : ( size_t ) pxCfg->ucDataCap;

        pxDesc->ucType = pxView->ucType;
        pxDesc->usSeq = pxView->usSeq;
        pxDesc->usReqId = pxView->usReqId;
        pxDesc->ulSrcIp = pxHdr->ulSrcIp;           /* network order: the reply goes back here */
        pxDesc->usSrcPort = pxHdr->usSrcPort;
        pxDesc->usLen = pxView->ucLen;
        pxDesc->ulRxCycles = pxHdr->ulRxCycles;     /* M1, for LAT_US = M3 - M1 */
        ( void ) memcpy( pxDesc->ucData, pxView->pucPayload, uxCopy );

        xEvent.eEventType = pxRoute->eEvent;
        xEvent.pvData = pxDesc;

        if( AppEvt_Send( pxCfg->eTarget, &xEvent, 0U ) == pdPASS )
        {
            /* The descriptor belongs to the receiver now (docs 6.3 rule 1). */
            prvCount( pxCfg, eParserStatEvtPosted );
        }
        else
        {
            /* Rule 2: not sent, still ours. */
            prvCount( pxCfg, eParserStatQueueFull );
            AppDesc_Release( pxDesc );
        }
    }
}

/*---------------------------------------------------------------------------*/
/* prvApplyHold: the artificial HOLD injection (docs 13.5, T03; P5).         */
/*---------------------------------------------------------------------------*/
static void prvApplyHold( void )
{
    /* Takes and clears the pending HOLD in one atomic step (report 044);
     * at most APP_HOLD_MAX_MS, checked where the console line arrived. */
    uint32_t ulMs = Console_TakeHoldMs();

    if( ulMs != 0U )
    {
        vTaskDelay( APP_MS( ulMs ) );
    }
}

/*---------------------------------------------------------------------------*/
/* prvCheckPacket: everything about one packet that is not its SEQ. Returns  */
/* its route, or NULL after counting the one reason it was refused. The      */
/* view points into ucRecord.                                                */
/*---------------------------------------------------------------------------*/
static const ParserRoute_t * prvCheckPacket( const ParserCfg_t * pxCfg, size_t uxPayload, PktView_t * pxView )
{
    const ParserRoute_t * pxRoute = NULL;
    ePktResult_t eResult;

    /* SOF, VER, LEN, CRC8, reserved FLAGS bits, in this order (report 006). */
    eResult = Pkt_Decode( &( pxCfg->pxState->ucRecord[ APP_REC_HEADER_BYTES ] ), uxPayload, pxView );

    if( eResult != ePktOk )
    {
        /* NULL pointers only: our own buffer and view are never NULL. */
        configASSERT( eResult != ePktErrNull );
        prvCount( pxCfg, ( eParserStat_t ) ( ( uint32_t ) eParserStatPktShort +
                                             ( ( uint32_t ) eResult - ( uint32_t ) ePktErrShort ) ) );
    }
    else if( ( pxView->ucFlags & APP_PKT_FLAG_KNOWN_MASK ) != 0U )
    {
        /* RESP / ERR from the host: a request is never a reply (P4). */
        prvCount( pxCfg, eParserStatPktFlags );
    }
    else
    {
        /* TYPE on this port and its LEN (P3); counts its own reason. */
        pxRoute = prvRoute( pxCfg, pxView );

        if( pxRoute != NULL )
        {
            prvCount( pxCfg, eParserStatPktOk );
        }
    }

    return pxRoute;
}

/*---------------------------------------------------------------------------*/
/* prvRoute: the lane's entry for this TYPE (P3; docs 15.2, 16.2).           */
/* Data lane: the TYPE must be one of the lane's and its LEN right, or the   */
/* packet is counted and dropped (nobody is answered on that lane).          */
/* Command lane (pxReplyRoute != NULL): a TYPE of another lane is dropped    */
/* the same way (PKT_WRONG_PORT); every other packet is a command and goes   */
/* to ctrlTask, also with a TYPE nobody knows or a wrong LEN: cmd.c answers  */
/* E_TYPE / E_LEN and starts nothing (docs 16.2, 16.3; report 085).          */
/*---------------------------------------------------------------------------*/
static const ParserRoute_t * prvRoute( const ParserCfg_t * pxCfg, const PktView_t * pxView )
{
    const ParserRoute_t * pxRoute = prvFindRoute( pxCfg, pxView->ucType );

    if( pxRoute == NULL )
    {
        if( ( pxCfg->pxReplyRoute != NULL ) && ( prvIsForeignType( pxCfg, pxView->ucType ) == pdFALSE ) )
        {
            /* Nobody's TYPE, on the command port: answered with E_TYPE. */
            pxRoute = pxCfg->pxReplyRoute;
        }
        else
        {
            /* A TYPE of the other lane or of the board (TELEM); on the data
             * lane also a TYPE nobody knows. */
            prvCount( pxCfg, eParserStatPktWrongPort );
        }
    }
    else if( pxCfg->pxReplyRoute != NULL )
    {
        /* A command: its LEN is checked by cmd.c, which answers E_LEN. */
    }
    else if( ( pxView->ucLen < pxRoute->ucMinLen ) || ( pxView->ucLen > pxRoute->ucMaxLen ) ||
             ( ( pxRoute->ucCountStride != 0U ) &&
               ( ( pxView->pucPayload[ 0 ] == 0U ) ||
                 ( pxView->pucPayload[ 0 ] > APP_ENV_MAX_CHANNELS ) ||
                 ( pxView->ucLen != ( 1U + ( ( uint32_t ) pxRoute->ucCountStride * pxView->pucPayload[ 0 ] ) ) ) ) ) )
    {
        /* ucMinLen >= 3 for the counted TYPE (ENV), so pucPayload[ 0 ]
         * exists whenever it is read. */
        prvCount( pxCfg, eParserStatPktTypeLen );
        pxRoute = NULL;
    }
    else
    {
        /* accepted */
    }

    return pxRoute;
}

/*---------------------------------------------------------------------------*/
/* prvFindRoute: the entry of ucType in the lane's own table, or NULL.       */
/*---------------------------------------------------------------------------*/
static const ParserRoute_t * prvFindRoute( const ParserCfg_t * pxCfg, uint8_t ucType )
{
    const ParserRoute_t * pxRoute = NULL;
    uint8_t x;

    for( x = 0U; x < pxCfg->ucRouteCount; x++ )
    {
        if( pxCfg->pxRoutes[ x ].ucType == ucType )
        {
            pxRoute = &( pxCfg->pxRoutes[ x ] );
            break;
        }
    }

    return pxRoute;
}

/*---------------------------------------------------------------------------*/
/* prvIsForeignType: a TYPE that exists but does not belong to this lane:    */
/* one of another lane's table, or TELEM (board -> host only, docs 15.2).    */
/*---------------------------------------------------------------------------*/
static BaseType_t prvIsForeignType( const ParserCfg_t * pxCfg, uint8_t ucType )
{
    BaseType_t xForeign = ( ucType == APP_TYPE_TELEM ) ? pdTRUE : pdFALSE;
    UBaseType_t x;

    for( x = 0U; ( x < ( UBaseType_t ) eAppClassCount ) && ( xForeign == pdFALSE ); x++ )
    {
        if( ( &xParserCfgs[ x ] != pxCfg ) && ( prvFindRoute( &xParserCfgs[ x ], ucType ) != NULL ) )
        {
            xForeign = pdTRUE;
        }
    }

    return xForeign;
}

/*---------------------------------------------------------------------------*/
/* prvTrackSeq: docs 16.5 on one channel (P2). pdTRUE: deliver the packet.   */
/*---------------------------------------------------------------------------*/
static BaseType_t prvTrackSeq( const ParserCfg_t * pxCfg, uint8_t ucChannel, uint16_t usSeq )
{
    ParserSeq_t * const pxSeq = &( pxCfg->pxState->xSeq[ ucChannel ] );
    BaseType_t xDeliver = pdTRUE;
    uint16_t usDelta;

    configASSERT( ucChannel < PARSER_MAX_CHANNELS );    /* route tables are constants */

    if( pxSeq->ucSeen == 0U )
    {
        /* First packet of the channel since reset: it sets the base, no gap
         * is counted (docs 16.6: the board's restart is not the host's). */
        pxSeq->ucSeen = 1U;
        pxSeq->usLast = usSeq;
        pxSeq->ucOldRun = 0U;
    }
    else
    {
        /* Modulo 65536: SEQ wraps (docs 15.1). */
        usDelta = ( uint16_t ) ( usSeq - pxSeq->usLast );

        if( usDelta == 0U )
        {
            prvCount( pxCfg, eParserStatSeqDup );
            pxSeq->ucOldRun = 0U;
            xDeliver = ( pxCfg->ucDropStale != 0U ) ? pdFALSE : pdTRUE;
        }
        else if( usDelta < APP_SEQ_HALF_RANGE )
        {
            if( usDelta > 1U )
            {
                Stats_Add( ( eStatId_t ) ( ( uint32_t ) pxCfg->eStatFirst + ( uint32_t ) eParserStatSeqGap ),
                           ( uint32_t ) usDelta - 1U );
            }

            pxSeq->usLast = usSeq;
            pxSeq->ucOldRun = 0U;
        }
        else
        {
            pxSeq->ucOldRun++;

            if( pxSeq->ucOldRun >= APP_SEQ_RESYNC_COUNT )
            {
                /* The host started again: take its SEQ as the new base and
                 * accept the packet (P2). */
                prvCount( pxCfg, eParserStatSeqResync );
                pxSeq->usLast = usSeq;
                pxSeq->ucOldRun = 0U;
            }
            else
            {
                prvCount( pxCfg, eParserStatSeqOld );
                xDeliver = ( pxCfg->ucDropStale != 0U ) ? pdFALSE : pdTRUE;
            }
        }
    }

    return xDeliver;
}

/*---------------------------------------------------------------------------*/
/* prvReadRecord: one whole message = one record (docs 5.1). Returns the     */
/* record's total length (header + payload), 0 on a time out. The payload    */
/* starts at ucRecord[ APP_REC_HEADER_BYTES ].                               */
/*---------------------------------------------------------------------------*/
static size_t prvReadRecord( const ParserCfg_t * pxCfg, UdpRecHdr_t * pxHdr )
{
    ParserState_t * const pxState = pxCfg->pxState;
    size_t uxGot;
    uint32_t ulM2;

    /* Blocks while the buffer is empty, then gives exactly one message
     * (stream_buffer.c:1113-1133, :1290-1330). ucRecord holds the largest
     * record, so no message is ever too long for it (:1310-1313 would leave
     * it in the buffer and return 0). */
    uxGot = xMessageBufferReceive( pxState->xBuffer, pxState->ucRecord, sizeof( pxState->ucRecord ),
                                   APP_MS( APP_PARSER_WAIT_MS ) );

    if( uxGot != 0U )
    {
        /* M2: the record is in hand (docs 8.1). */
        ulM2 = DWT->CYCCNT;
        prvCount( pxCfg, eParserStatRecRead );

        /* The callback writes the header in front of every record (an empty
         * datagram is 12 B): a shorter message is a bug. */
        configASSERT( uxGot >= APP_REC_HEADER_BYTES );
        ( void ) memcpy( pxHdr, pxState->ucRecord, APP_REC_HEADER_BYTES );

        if( pxCfg->ucMeasureM2 != 0U )
        {
            /* Unsigned difference: correct across a CYCCNT wrap. Single
             * writer of eLatCritParse: critParser (P6). */
            Stats_LatencyAdd( eLatCritParse, ulM2 - pxHdr->ulRxCycles );
        }
    }

    return uxGot;
}

/*---------------------------------------------------------------------------*/
static void prvCount( const ParserCfg_t * pxCfg, eParserStat_t eStat )
{
    Stats_Inc( ( eStatId_t ) ( ( uint32_t ) pxCfg->eStatFirst + ( uint32_t ) eStat ) );
}

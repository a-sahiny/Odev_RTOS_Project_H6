/**
  ******************************************************************************
  * @file    cmd.c
  * @brief   Command handler of ctrlTask (docs 16.1-16.4): validate, execute,
  *          reply. Künye: report 065. GET_IMU (roll / pitch / yaw, deadline
  *          12 ms): report 086 / 088.
  ******************************************************************************
  * No RTOS object. Runs in ctrlTask only; its state (reply buffer, reply
  * SEQ) is touched by that task alone.
  * Used, not owned: UDP socket 5000 (udp_in.c) through FreeRTOS_sendto,
  * which waits at most APP_CTRL_SEND_TIMEOUT_MS (its SO_SNDTIMEO) for a
  * network buffer and the IP task's queue; xSysEvents (sys_state.c), whose
  * MEAS_EN and FAULT bits ctrlTask alone writes.
  */
#include "cmd.h"

#include <stdint.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "FreeRTOS_IP.h"
#include "FreeRTOS_Sockets.h"

#include "stm32f7xx.h"      /* DWT->CYCCNT (M3) */

#include "app_config.h"
#include "app_stats.h"
#include "ctrl_task.h"      /* CtrlTask_GetActiveFaults */
#include "meas_task.h"      /* Meas_SetDestination / SetRateHz / GetRateHz / GetAttitude */
#include "pkt_format.h"
#include "report.h"         /* Report_Fault (START clears a FAULT) */
#include "sys_state.h"
#include "udp_in.h"         /* UdpIn_GetCtrlSocket */
#include "uart_tx.h"        /* UartTx_GetLastImuReply (GET_IMU, report 091) */
#include "worker_task.h"    /* Worker_GetEnvStats (GET_STATS page 3; step 16) */

/* Largest reply payload: the common part plus GET_STATS (step 15c). */
#define CMD_EXTRA_MAX_BYTES     ( APP_PKT_MAX_PAYLOAD - APP_RESP_COMMON_BYTES )

/* GET_STATS (K14): pages 0..2 hold the counters, page 3 the ENV summaries
 * (docs 5.2: "GET_STATS sayfa 3"). Head: page u8, tick u32, n u8. */
#define CMD_COUNTER_PAGES       ( APP_STATS_PAGES - 1U )
#define CMD_COUNTERS_PER_PAGE   ( ( ( uint32_t ) eStatCount + CMD_COUNTER_PAGES - 1U ) / CMD_COUNTER_PAGES )
#define CMD_STATS_HEAD_BYTES    6U

/* Type-specific reply bytes a duplicate window entry keeps: SET_RATE's u16. */
#define CMD_DEDUP_EXTRA_BYTES   2U

/* One remembered command (docs 16.4). The payload of every valid command is
 * kept whole (<= 2 B, room for 4): no hash, so no false match. A request
 * with a wrong LEN or an unknown TYPE may carry more; its LEN and its first
 * APP_DEDUP_PAYLOAD_BYTES bytes are kept (report 085 K2). */
typedef struct xCMD_DEDUP
{
    uint32_t ulSrcIp;               /* key, network order                     */
    uint16_t usReqId;               /* key; 0 = free slot (REQ_ID 0 is never stored) */
    uint8_t ucType;
    uint8_t ucLen;
    uint8_t ucData[ APP_DEDUP_PAYLOAD_BYTES ];
    uint8_t ucStatus;               /* the first answer                       */
    uint8_t ucExtraLen;             /* 0 for GET_STATS: never cached (docs 16.4) */
    uint8_t ucExtra[ CMD_DEDUP_EXTRA_BYTES ];
    TickType_t xTime;               /* when it was stored                     */
} CmdDedup_t;

typedef enum eCMD_SEEN
{
    eCmdNew = 0,                    /* not in the window                      */
    eCmdReplay,                     /* same key, same content                 */
    eCmdConflict                    /* same key, other content                */
} eCmdSeen_t;

static uint8_t ucReply[ APP_PKT_MAX_BYTES ];        /* the packet being sent  */
static uint8_t ucPayload[ APP_PKT_MAX_PAYLOAD ];    /* its payload            */
static uint16_t usReplySeq;                          /* board -> host replies  */
static CmdDedup_t xDedup[ APP_DEDUP_SLOTS ];
static uint8_t ucDedupNext;                          /* the oldest slot: the next one written */
static StatsSnapshot_t xSnap;                        /* GET_STATS; too big for ctrlTask's stack */
static ImuAttitude_t xImu;                           /* GET_IMU: the copy prvValidate took */

static eCmdSeen_t prvDedupCheck( const AppDesc_t * pxDesc, const CmdDedup_t ** ppxEntry );
static void prvDedupStore( const AppDesc_t * pxDesc, uint8_t ucStatus, const uint8_t * pucExtra, size_t uxExtra );
static size_t prvKeptBytes( const AppDesc_t * pxDesc );
static uint8_t prvValidate( const AppDesc_t * pxDesc );
static uint8_t prvValidateStart( void );
static uint8_t prvValidateGetImu( void );
static size_t prvWriteImu( const AppDesc_t * pxDesc, uint8_t * pucOut );
static void prvExecute( const AppDesc_t * pxDesc );
static size_t prvTypeSpecific( const AppDesc_t * pxDesc, uint8_t ucStatus, uint8_t * pucOut );
static size_t prvExecGetStats( uint8_t ucPage, uint8_t * pucOut );
static void prvSendResponse( const AppDesc_t * pxDesc, uint8_t ucStatus, size_t uxExtra );
static size_t prvBuildCommon( const AppDesc_t * pxDesc, uint8_t ucStatus );

_Static_assert( APP_STATS_PAGES == 4U, "3 counter pages + the ENV page" );
_Static_assert( ( CMD_STATS_HEAD_BYTES + ( 4U * CMD_COUNTERS_PER_PAGE ) ) <= CMD_EXTRA_MAX_BYTES, "a counter page fits a reply" );
_Static_assert( ( CMD_STATS_HEAD_BYTES + ( 16U * APP_ENV_MAX_CHANNELS ) ) <= CMD_EXTRA_MAX_BYTES, "the ENV page fits a reply" );
_Static_assert( CMD_COUNTERS_PER_PAGE <= UINT8_MAX, "n is one byte" );

_Static_assert( APP_RESP_COMMON_BYTES == 6U, "STATUS EPOCH(2) STATE LAT_US(2)" );
_Static_assert( ( APP_LEN_SET_RATE == 2U ) && ( APP_LEN_GET_STATS == 1U ), "payload layout below" );
_Static_assert( APP_DEDUP_PAYLOAD_BYTES >= APP_LEN_SET_RATE, "a command payload fits a dedup entry whole" );
_Static_assert( APP_LEN_IMU_REPLY == ( 4U + 4U + ( 9U * 2U ) + 1U + 4U + 4U ), "GET_IMU reply layout below (reports 086, 091)" );
_Static_assert( APP_IMU_REPLY_PREV_OFFSET == 27U, "prev follows flags" );
_Static_assert( ( APP_IMU_REPLY_RX_OFFSET + 4U ) == APP_LEN_IMU_REPLY, "t_rx is the last field" );
_Static_assert( APP_LEN_IMU_REPLY <= CMD_EXTRA_MAX_BYTES, "the GET_IMU reply fits" );
_Static_assert( ( APP_DEDUP_SLOTS > 0U ) && ( APP_DEDUP_SLOTS <= UINT8_MAX ), "slot index is 8 bits" );

/*---------------------------------------------------------------------------*/
void Cmd_Init( void )
{
    /* The first reply carries SEQ 1 (docs 15.4). After a reset the window is
     * empty (docs 16.6). */
    usReplySeq = 0U;
    ( void ) memset( xDedup, 0, sizeof( xDedup ) );
    ucDedupNext = 0U;
}

/*---------------------------------------------------------------------------*/
/* Cmd_Handle: docs 16.3.                                                    */
/*---------------------------------------------------------------------------*/
eAppDescAction_t Cmd_Handle( void * pvData )
{
    const AppDesc_t * const pxDesc = ( const AppDesc_t * ) pvData;
    const CmdDedup_t * pxSeen = NULL;
    uint8_t ucStatus;
    size_t uxExtra;

    /* The dispatcher gives every eAppCmdEvent its descriptor (docs 6.2). */
    configASSERT( pxDesc != NULL );

    if( pxDesc->usReqId == 0U )
    {
        /* A command must carry an id (docs 15.1); not stored. */
        ucStatus = APP_ST_E_REQID;
        Stats_Inc( eStatCmdInvalid );
        uxExtra = prvTypeSpecific( pxDesc, ucStatus, &ucPayload[ APP_RESP_COMMON_BYTES ] );
    }
    else if( pxDesc->ucType == APP_TYPE_CMD_GET_IMU )
    {
        /* A read with no side effect that the host asks up to 100 times a
         * second: it stays out of the duplicate window, which would
         * otherwise forget START / STOP within 80 ms (report 086 G2). A
         * repeated REQ_ID simply reads the attitude again. */
        ucStatus = prvValidate( pxDesc );
        /* Its own counters (report 092): the GET_IMU stream (up to 50 per
         * second) does not change what START / STOP / SET_RATE / GET_STATS
         * counted, so those stay readable while the IMU is being watched. */
        Stats_Inc( ( ucStatus == APP_ST_OK ) ? eStatCmdImuOk : eStatCmdImuRefused );
        uxExtra = prvTypeSpecific( pxDesc, ucStatus, &ucPayload[ APP_RESP_COMMON_BYTES ] );
    }
    else
    {
        switch( prvDedupCheck( pxDesc, &pxSeen ) )
        {
            case eCmdReplay:
                /* The first answer again, no side effect (docs 16.4). GET_STATS
                 * is not cached: its counters are read now. */
                ucStatus = pxSeen->ucStatus;
                Stats_Inc( eStatCmdReplay );

                if( pxSeen->ucExtraLen != 0U )
                {
                    ( void ) memcpy( &ucPayload[ APP_RESP_COMMON_BYTES ], pxSeen->ucExtra, pxSeen->ucExtraLen );
                    uxExtra = pxSeen->ucExtraLen;
                }
                else
                {
                    uxExtra = prvTypeSpecific( pxDesc, ucStatus, &ucPayload[ APP_RESP_COMMON_BYTES ] );
                }
                break;

            case eCmdConflict:
                /* Deterministic, the first request stays (docs 16.4). */
                ucStatus = APP_ST_E_CONFLICT;
                Stats_Inc( eStatCmdConflict );
                uxExtra = prvTypeSpecific( pxDesc, ucStatus, &ucPayload[ APP_RESP_COMMON_BYTES ] );
                break;

            case eCmdNew:
            default:
                /* Validation has no side effect; execution runs only for a
                 * valid request (docs 16.3). */
                ucStatus = prvValidate( pxDesc );

                if( ucStatus == APP_ST_OK )
                {
                    prvExecute( pxDesc );
                    Stats_Inc( eStatCmdNew );
                }
                else
                {
                    Stats_Inc( eStatCmdInvalid );
                }

                uxExtra = prvTypeSpecific( pxDesc, ucStatus, &ucPayload[ APP_RESP_COMMON_BYTES ] );

                /* A refused request is stored as well (D19). */
                prvDedupStore( pxDesc, ucStatus, &ucPayload[ APP_RESP_COMMON_BYTES ], uxExtra );
                break;
        }
    }

    prvSendResponse( pxDesc, ucStatus, uxExtra );

    /* Nothing keeps the descriptor (docs 6.3 rule 3; K9). */
    return eAppDescRelease;
}

/*---------------------------------------------------------------------------*/
/* prvDedupCheck: the window is the last APP_DEDUP_SLOTS commands and at     */
/* most APP_DEDUP_WINDOW_MS, whichever ends first (docs 16.4).               */
/*---------------------------------------------------------------------------*/
static eCmdSeen_t prvDedupCheck( const AppDesc_t * pxDesc, const CmdDedup_t ** ppxEntry )
{
    eCmdSeen_t eSeen = eCmdNew;
    const TickType_t xNow = xTaskGetTickCount();
    const CmdDedup_t * pxEntry;
    uint8_t x;

    for( x = 0U; x < ( uint8_t ) APP_DEDUP_SLOTS; x++ )
    {
        pxEntry = &xDedup[ x ];

        /* Unsigned tick difference: right across a tick wrap. */
        if( ( pxEntry->usReqId == pxDesc->usReqId ) && ( pxEntry->ulSrcIp == pxDesc->ulSrcIp ) &&
            ( ( TickType_t ) ( xNow - pxEntry->xTime ) < APP_MS( APP_DEDUP_WINDOW_MS ) ) )
        {
            /* Same key: same content is a repeat, other content a conflict.
             * Content = TYPE, LEN and the payload bytes the entry keeps. */
            if( ( pxEntry->ucType == pxDesc->ucType ) && ( pxEntry->ucLen == pxDesc->usLen ) &&
                ( memcmp( pxEntry->ucData, pxDesc->ucData, prvKeptBytes( pxDesc ) ) == 0 ) )
            {
                eSeen = eCmdReplay;
            }
            else
            {
                eSeen = eCmdConflict;
            }

            *ppxEntry = pxEntry;
            break;
        }
    }

    return eSeen;
}

/*---------------------------------------------------------------------------*/
/* prvDedupStore: into the oldest slot (ring); the request with its answer.  */
/*---------------------------------------------------------------------------*/
static void prvDedupStore( const AppDesc_t * pxDesc, uint8_t ucStatus, const uint8_t * pucExtra, size_t uxExtra )
{
    CmdDedup_t * const pxEntry = &xDedup[ ucDedupNext ];

    /* REQ_ID 0 is answered before this (E_REQID) and never stored, so 0
     * still means "free". Pkt_Decode keeps LEN <= 200: it fits ucLen. */
    configASSERT( ( pxDesc->usReqId != 0U ) && ( pxDesc->usLen <= APP_PKT_MAX_PAYLOAD ) );

    pxEntry->ulSrcIp = pxDesc->ulSrcIp;
    pxEntry->usReqId = pxDesc->usReqId;
    pxEntry->ucType = pxDesc->ucType;
    pxEntry->ucLen = ( uint8_t ) pxDesc->usLen;
    ( void ) memset( pxEntry->ucData, 0, sizeof( pxEntry->ucData ) );
    ( void ) memcpy( pxEntry->ucData, pxDesc->ucData, prvKeptBytes( pxDesc ) );
    pxEntry->ucStatus = ucStatus;
    pxEntry->xTime = xTaskGetTickCount();

    /* GET_STATS: kept for the conflict check, its answer is not (docs 16.4). */
    if( ( pxDesc->ucType != APP_TYPE_CMD_GET_STATS ) && ( uxExtra <= CMD_DEDUP_EXTRA_BYTES ) )
    {
        pxEntry->ucExtraLen = ( uint8_t ) uxExtra;
        ( void ) memcpy( pxEntry->ucExtra, pucExtra, uxExtra );
    }
    else
    {
        pxEntry->ucExtraLen = 0U;
    }

    ucDedupNext = ( uint8_t ) ( ( ucDedupNext + 1U ) % APP_DEDUP_SLOTS );
}

/*---------------------------------------------------------------------------*/
/* prvKeptBytes: the payload bytes of a request that the descriptor holds    */
/* and a duplicate window entry keeps. Every valid command: all of them.     */
/*---------------------------------------------------------------------------*/
static size_t prvKeptBytes( const AppDesc_t * pxDesc )
{
    return ( pxDesc->usLen <= APP_DEDUP_PAYLOAD_BYTES ) ? ( size_t ) pxDesc->usLen : ( size_t ) APP_DEDUP_PAYLOAD_BYTES;
}

/*---------------------------------------------------------------------------*/
/* prvValidate: type, length, range and state, in this order; no side effect */
/* (docs 16.2, 16.3). critParser hands over every packet of the command port */
/* that is not another lane's TYPE, so an unknown TYPE and a wrong LEN are    */
/* answered here: E_TYPE / E_LEN (report 085). The payload is read only     */
/* after its length has been checked.                                        */
/*---------------------------------------------------------------------------*/
static uint8_t prvValidate( const AppDesc_t * pxDesc )
{
    uint8_t ucStatus = APP_ST_OK;
    uint16_t usRate;

    switch( pxDesc->ucType )
    {
        case APP_TYPE_CMD_START:
            ucStatus = ( pxDesc->usLen != 0U ) ? APP_ST_E_LEN : prvValidateStart();
            break;

        case APP_TYPE_CMD_STOP:

            if( pxDesc->usLen != 0U )
            {
                ucStatus = APP_ST_E_LEN;
            }
            break;

        case APP_TYPE_CMD_SET_RATE:

            if( pxDesc->usLen != APP_LEN_SET_RATE )
            {
                ucStatus = APP_ST_E_LEN;
            }
            else
            {
                usRate = Pkt_ReadU16( pxDesc->ucData );

                if( ( usRate < APP_RATE_MIN_HZ ) || ( usRate > APP_RATE_MAX_HZ ) )
                {
                    ucStatus = APP_ST_E_RANGE;
                }
            }
            break;

        case APP_TYPE_CMD_GET_STATS:

            if( pxDesc->usLen != APP_LEN_GET_STATS )
            {
                ucStatus = APP_ST_E_LEN;
            }
            else if( pxDesc->ucData[ 0 ] >= APP_STATS_PAGES )
            {
                ucStatus = APP_ST_E_RANGE;
            }
            else
            {
                /* a valid page */
            }
            break;

        case APP_TYPE_CMD_GET_IMU:
            ucStatus = ( pxDesc->usLen != APP_LEN_GET_IMU ) ? APP_ST_E_LEN : prvValidateGetImu();
            break;

        default:
            /* Not a command TYPE (docs 16.2). */
            ucStatus = APP_ST_E_TYPE;
            break;
    }

    return ucStatus;
}

/*---------------------------------------------------------------------------*/
/* prvValidateStart: docs 16.1: NET_READY; no FAULT, or its conditions are   */
/* over.                                                                     */
/*---------------------------------------------------------------------------*/
static uint8_t prvValidateStart( void )
{
    uint8_t ucStatus = APP_ST_OK;
    EventBits_t uxState = SysState_Get();

    if( ( uxState & APP_BIT_NET_READY ) == 0U )
    {
        ucStatus = APP_ST_E_STATE;
    }
    else if( ( ( uxState & APP_BIT_FAULT ) != 0U ) && ( CtrlTask_GetActiveFaults() != 0U ) )
    {
        ucStatus = APP_ST_E_STATE;
    }
    else
    {
        /* allowed */
    }

    return ucStatus;
}

/*---------------------------------------------------------------------------*/
/* prvValidateGetImu: report 086: only while a measurement runs (after START,*/
/* before STOP / FAULT) and once it has a sample. Takes the copy the reply   */
/* is written from (meas cannot change the START in between: only ctrlTask   */
/* handles START).                                                           */
/*---------------------------------------------------------------------------*/
static uint8_t prvValidateGetImu( void )
{
    uint8_t ucStatus = APP_ST_OK;

    if( ( SysState_Get() & APP_BIT_MEAS_EN ) == 0U )
    {
        ucStatus = APP_ST_E_STATE;
    }
    else if( Meas_GetAttitude( &xImu ) != pdPASS )
    {
        /* The first sample of this START is not in yet (<= 10 ms). */
        ucStatus = APP_ST_E_STATE;
    }
    else
    {
        /* a sample to answer with */
    }

    return ucStatus;
}

/*---------------------------------------------------------------------------*/
/* prvWriteImu: the 35 B after the common part, big-endian (086, 091).      */
/*---------------------------------------------------------------------------*/
static size_t prvWriteImu( const AppDesc_t * pxDesc, uint8_t * pucOut )
{
    size_t x;

    Pkt_WriteU32( &pucOut[ 0 ], xImu.ulIndex );
    Pkt_WriteU32( &pucOut[ 4 ], xImu.ulTick );

    for( x = 0U; x < 3U; x++ )
    {
        Pkt_WriteU16( &pucOut[ 8U + ( 2U * x ) ], ( uint16_t ) xImu.sAccel[ x ] );
        Pkt_WriteU16( &pucOut[ 14U + ( 2U * x ) ], ( uint16_t ) xImu.sGyro[ x ] );
        Pkt_WriteU16( &pucOut[ 20U + ( 2U * x ) ], ( uint16_t ) xImu.sAngle[ x ] );
    }

    pucOut[ 26 ] = xImu.ucFlags;
    /* The board's own M1 -> M6 of the previous reply (uartTx measured it
     * after that line left): the host sees the deadline in board time,
     * without its USB / Windows part (report 091). */
    Pkt_WriteU32( &pucOut[ APP_IMU_REPLY_PREV_OFFSET ], UartTx_GetLastImuReply() );
    Pkt_WriteU32( &pucOut[ APP_IMU_REPLY_RX_OFFSET ], pxDesc->ulRxCycles );     /* M1 */

    return APP_LEN_IMU_REPLY;
}

/*---------------------------------------------------------------------------*/
/* prvExecute: the side effects, here and only here (docs 16.3).             */
/*---------------------------------------------------------------------------*/
static void prvExecute( const AppDesc_t * pxDesc )
{
    BaseType_t xResult = pdPASS;

    switch( pxDesc->ucType )
    {
        case APP_TYPE_CMD_START:

            /* A FAULT whose conditions are over is cleared by START (docs
             * 19); prvValidate made sure none is active. */
            if( ( SysState_Get() & APP_BIT_FAULT ) != 0U )
            {
                xResult = SysState_TryClearFault( 0U );
                configASSERT( xResult == pdPASS );
                Report_Fault( pdFALSE, 0U );
            }

            /* Telemetry goes where START came from (docs 16.1), then the
             * measurement may run. */
            Meas_SetDestination( pxDesc->ulSrcIp, pxDesc->usSrcPort );
            xResult = SysState_SetMeasEnabled( pdTRUE );
            configASSERT( xResult == pdPASS );      /* FAULT is clear here */
            break;

        case APP_TYPE_CMD_STOP:
            ( void ) SysState_SetMeasEnabled( pdFALSE );
            break;

        case APP_TYPE_CMD_SET_RATE:
            Meas_SetRateHz( Pkt_ReadU16( pxDesc->ucData ) );
            break;

        default:
            /* GET_STATS and GET_IMU only read (step 15c, report 086). */
            break;
    }

    ( void ) xResult;
}

/*---------------------------------------------------------------------------*/
/* prvTypeSpecific: the bytes after the common part (docs 15.3). Written     */
/* after execution, so SET_RATE reports the rate now in force, valid or not  */
/* (K5), also with E_LEN. An unknown TYPE has nothing after the common part. */
/*---------------------------------------------------------------------------*/
static size_t prvTypeSpecific( const AppDesc_t * pxDesc, uint8_t ucStatus, uint8_t * pucOut )
{
    size_t uxLen = 0U;

    if( pxDesc->ucType == APP_TYPE_CMD_SET_RATE )
    {
        Pkt_WriteU16( pucOut, Meas_GetRateHz() );
        uxLen = 2U;
    }
    else if( ( pxDesc->ucType == APP_TYPE_CMD_GET_STATS ) && ( ucStatus == APP_ST_OK ) )
    {
        /* Only a valid page has data; prvValidate checked 0..3. */
        uxLen = prvExecGetStats( pxDesc->ucData[ 0 ], pucOut );
    }
    else if( ( pxDesc->ucType == APP_TYPE_CMD_GET_IMU ) && ( ucStatus == APP_ST_OK ) )
    {
        /* prvValidate took the copy; an error reply has the common part only. */
        uxLen = prvWriteImu( pxDesc, pucOut );
    }
    else
    {
        /* nothing after the common part */
    }

    return uxLen;
}

/*---------------------------------------------------------------------------*/
/* prvExecGetStats: page u8, tick u32, n u8, n x u32 (docs 15.3; K14).       */
/* Pages 0..2: the counters, read in one snapshot (docs 18.1: the window is  */
/* between two GET_STATS, both counters and tick from the same instant).     */
/* Report 092: page 0 takes the snapshot, pages 1 and 2 answer from that     */
/* same snapshot with the same tick, so the three pages show one instant.    */
/* (A snapshot per page let an event between two pages, e.g. worker         */
/* finishing a queued BULK, shift the 18.2 equations by one on the board.)   */
/* A page 1 / 2 with no page 0 before it answers the last snapshot; the tick */
/* in the head tells the host which instant it got.                          */
/* Page 3: ENV, per channel count, min, max, mean (docs 5.2).                */
/*---------------------------------------------------------------------------*/
static size_t prvExecGetStats( uint8_t ucPage, uint8_t * pucOut )
{
    static ReportEnvChannel_t xEnv[ APP_ENV_MAX_CHANNELS ];
    uint32_t ulTick;
    uint32_t ulFirst;
    uint32_t ulCount = 0U;
    uint32_t x;
    size_t uxChannels;

    if( ucPage < CMD_COUNTER_PAGES )
    {
        if( ucPage == 0U )
        {
            /* Report 092: critParser has the same priority (P6). A tick right
             * after it queued this request (before it counted EVT_POSTED)
             * gives ctrlTask the CPU by time slicing; a snapshot then would
             * see PKT_OK without EVT_POSTED (board: L4 / L5 +-1 now and
             * then). The yield lets a ready P6 task finish its step (it
             * blocks on sbCrit next); with none ready it returns at once. */
            taskYIELD();
            Stats_Snapshot( &xSnap );
        }

        ulTick = xSnap.ulTick;
        ulFirst = ( uint32_t ) ucPage * CMD_COUNTERS_PER_PAGE;
        ulCount = ( ( ( uint32_t ) eStatCount - ulFirst ) < CMD_COUNTERS_PER_PAGE ) ?
                  ( ( uint32_t ) eStatCount - ulFirst ) : CMD_COUNTERS_PER_PAGE;

        for( x = 0U; x < ulCount; x++ )
        {
            Pkt_WriteU32( &pucOut[ CMD_STATS_HEAD_BYTES + ( 4U * x ) ], xSnap.ulCounters[ ulFirst + x ] );
        }
    }
    else
    {
        ulTick = ( uint32_t ) xTaskGetTickCount();
        uxChannels = Worker_GetEnvStats( xEnv, APP_ENV_MAX_CHANNELS );

        /* i16 values as their 32-bit two's complement (sign-extended). */
        for( x = 0U; x < ( uint32_t ) uxChannels; x++ )
        {
            Pkt_WriteU32( &pucOut[ CMD_STATS_HEAD_BYTES + ( 16U * x ) ], xEnv[ x ].ulCount );
            Pkt_WriteU32( &pucOut[ CMD_STATS_HEAD_BYTES + ( 16U * x ) + 4U ], ( uint32_t ) ( int32_t ) xEnv[ x ].sMin );
            Pkt_WriteU32( &pucOut[ CMD_STATS_HEAD_BYTES + ( 16U * x ) + 8U ], ( uint32_t ) ( int32_t ) xEnv[ x ].sMax );
            Pkt_WriteU32( &pucOut[ CMD_STATS_HEAD_BYTES + ( 16U * x ) + 12U ], ( uint32_t ) ( int32_t ) xEnv[ x ].sMean );
        }

        ulCount = 4U * ( uint32_t ) uxChannels;
    }

    pucOut[ 0 ] = ucPage;
    Pkt_WriteU32( &pucOut[ 1 ], ulTick );
    pucOut[ 5 ] = ( uint8_t ) ulCount;

    return CMD_STATS_HEAD_BYTES + ( 4U * ( size_t ) ulCount );
}

/*---------------------------------------------------------------------------*/
/* prvSendResponse: header, common part, send to the request's source.       */
/*---------------------------------------------------------------------------*/
static void prvSendResponse( const AppDesc_t * pxDesc, uint8_t ucStatus, size_t uxExtra )
{
    PktHeader_t xHdr;
    struct freertos_sockaddr xDest;
    size_t uxLen;
    int32_t lSent;
    Socket_t xSocket = UdpIn_GetCtrlSocket();

    /* A command came in on this socket, so it is open. */
    configASSERT( xSocket != NULL );
    configASSERT( uxExtra <= CMD_EXTRA_MAX_BYTES );

    ( void ) prvBuildCommon( pxDesc, ucStatus );

    usReplySeq++;
    xHdr.ucType = pxDesc->ucType;           /* replies carry the request's TYPE (docs 15.2) */
    xHdr.ucFlags = ( uint8_t ) ( APP_PKT_FLAG_RESP | ( ( ucStatus != APP_ST_OK ) ? APP_PKT_FLAG_ERR : 0U ) );
    xHdr.usSeq = usReplySeq;
    xHdr.usReqId = pxDesc->usReqId;

    uxLen = Pkt_Encode( ucReply, sizeof( ucReply ), &xHdr, ucPayload, APP_RESP_COMMON_BYTES + uxExtra );
    configASSERT( uxLen != 0U );            /* sizes are fixed above */

    ( void ) memset( &xDest, 0, sizeof( xDest ) );
    xDest.sin_len = ( uint8_t ) sizeof( xDest );
    xDest.sin_family = FREERTOS_AF_INET4;
    xDest.sin_port = pxDesc->usSrcPort;                 /* network order, as received */
    xDest.sin_address.ulIP_IPv4 = pxDesc->ulSrcIp;

    /* Waits at most SO_SNDTIMEO (5 ms) for a network buffer and the IP queue
     * (FreeRTOS_Sockets.c:1404-1444). 0 / < 0: the reply is lost; the host
     * repeats with the same REQ_ID (docs 16.4). */
    lSent = FreeRTOS_sendto( xSocket, ucReply, uxLen, 0, &xDest, ( socklen_t ) sizeof( xDest ) );

    if( lSent <= 0 )
    {
        Stats_Inc( eStatCmdReplyFail );
    }
}

/*---------------------------------------------------------------------------*/
/* prvBuildCommon: STATUS, EPOCH, STATE, LAT_US (docs 15.3). M3 is taken     */
/* here, just before the reply is built (docs 8.1; K3).                      */
/*---------------------------------------------------------------------------*/
static size_t prvBuildCommon( const AppDesc_t * pxDesc, uint8_t ucStatus )
{
    uint32_t ulCycles = DWT->CYCCNT - pxDesc->ulRxCycles;      /* M3 - M1, across a wrap */
    uint32_t ulUs = ulCycles / ( uint32_t ) ( configCPU_CLOCK_HZ / 1000000UL );

    /* ctrlTask is the one writer of eLatCmd. */
    Stats_LatencyAdd( eLatCmd, ulCycles );

    ucPayload[ 0 ] = ucStatus;
    Pkt_WriteU16( &ucPayload[ 1 ], SysState_GetEpoch() );
    ucPayload[ 3 ] = ( uint8_t ) SysState_Get();                /* bits 0..2 (report 022) */
    Pkt_WriteU16( &ucPayload[ 4 ], ( uint16_t ) ( ( ulUs > 0xFFFFU ) ? 0xFFFFU : ulUs ) );

    return APP_RESP_COMMON_BYTES;
}

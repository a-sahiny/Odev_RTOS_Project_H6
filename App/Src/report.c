/**
  ******************************************************************************
  * @file    report.c
  * @brief   One-line JSON reports and the report block pool (docs 5.2).
  ******************************************************************************
  * RTOS object (one):
  *   xReportPool  fixed_pool (static counting semaphore + List_t, the
  *                BufferAllocation_1.c pattern), APP_REPORT_COUNT blocks of
  *                ReportDesc_t.
  *                Take (Pool_Get, wait 0): the report writers ctrlTask (P6),
  *                vnetRx (P4), worker (P1).
  *                Give (Pool_Release): uartTx (P4) after copying the line, or
  *                the writer when UartTx_PostReport refused the block.
  * A block has one owner at a time (writer -> queue -> uartTx), so its text
  * needs no lock. The JSON writer state lives on the caller's stack.
  */
#include "report.h"

#include <stddef.h>

#include "FreeRTOS.h"
#include "task.h"           /* configASSERT uses taskDISABLE_INTERRUPTS */

#include "app_config.h"
#include "app_stats.h"
#include "fixed_pool.h"
#include "sys_state.h"      /* SysState_GetEpoch */
#include "uart_tx.h"

/* One report being written. */
typedef struct xREPORT_WRITER
{
    ReportDesc_t * pxDesc;      /* NULL: no block, the report is dropped     */
    size_t uxPos;               /* characters written to cText               */
    uint8_t ucOverflow;         /* 1: something did not fit (R3)             */
} ReportWriter_t;

static FixedPool_t xReportPool;
static ReportDesc_t xReportBlocks[ APP_REPORT_COUNT ];
static uint8_t ucReportInit;    /* 1 after a successful Report_Init          */

static uint8_t prvBegin( ReportWriter_t * pxW, const char * pcType );
static void prvPutRaw( ReportWriter_t * pxW, const char * pcText );
static void prvPutKey( ReportWriter_t * pxW, const char * pcKey );
static void prvPutDigits( ReportWriter_t * pxW, uint32_t ulValue );
static void prvPutI32Digits( ReportWriter_t * pxW, int32_t lValue );
static void prvPutLevel( ReportWriter_t * pxW, uint32_t ulValue );
static void prvPutU32Array( ReportWriter_t * pxW, const char * pcKey, const uint32_t * pulValues, size_t uxCount );
static void prvPutResources( ReportWriter_t * pxW, const StatsSnapshot_t * pxSnap, const ReportInject_t * pxInj );
static void prvPutU32( ReportWriter_t * pxW, const char * pcKey, uint32_t ulValue );
static void prvPutStr( ReportWriter_t * pxW, const char * pcKey, const char * pcValue );
static void prvFinish( ReportWriter_t * pxW );
static void prvPutChar( ReportWriter_t * pxW, char cChar );

_Static_assert( APP_REPORT_MAX_CHARS <= UINT16_MAX, "usLen is 16 bits" );
/* APP_UDP_PORTS_JSON is fixed text (B13): it must name these three ports. */
_Static_assert( ( APP_UDP_PORT_CRIT == 5000U ) && ( APP_UDP_PORT_LOW == 5001U ) && ( APP_UDP_PORT_TELEM == 5002U ),
                "APP_UDP_PORTS_JSON lists 5000, 5001, 5002" );

/*---------------------------------------------------------------------------*/
BaseType_t Report_Init( void )
{
    FixedPoolCfg_t xCfg;
    BaseType_t xResult = pdPASS;

    if( ucReportInit == 0U )
    {
        xCfg.pvBlocks = xReportBlocks;
        xCfg.uxBlockSize = sizeof( ReportDesc_t );
        xCfg.uxBlockCount = APP_REPORT_COUNT;
        xCfg.uxIsrReserve = APP_REPORT_ISR_RESERVE;

        xResult = Pool_Init( &xReportPool, &xCfg );

        /* Pool counters (empty / invalid / double) through the pool hooks. */
        if( ( xResult == pdPASS ) && ( Stats_RegisterPool( &xReportPool, eStatPoolReportEmpty ) == false ) )
        {
            xResult = pdFAIL;
        }

        if( xResult == pdPASS )
        {
            ucReportInit = 1U;
        }
    }

    return xResult;
}

/*---------------------------------------------------------------------------*/
void Report_Release( ReportDesc_t * pxReport )
{
    /* NULL: Pool_Release returns pdFAIL silently. A foreign block or a
     * second release is caught and counted by fixed_pool (report 011). */
    ( void ) Pool_Release( &xReportPool, pxReport );
}

/*---------------------------------------------------------------------------*/
void Report_Ack( const char * pcResult )
{
    ReportWriter_t xW;

    if( prvBegin( &xW, "ack" ) != 0U )
    {
        prvPutStr( &xW, "result", pcResult );
        prvFinish( &xW );
    }
}

/*---------------------------------------------------------------------------*/
void Report_Fault( BaseType_t xSet, uint32_t ulWhy )
{
    ReportWriter_t xW;

    if( prvBegin( &xW, "fault" ) != 0U )
    {
        prvPutU32( &xW, "set", ( xSet != pdFALSE ) ? 1U : 0U );
        prvPutU32( &xW, "why", ulWhy );
        prvFinish( &xW );
    }
}

/*---------------------------------------------------------------------------*/
void Report_ImuRate( uint16_t usHz, const uint8_t * pucWritten, const uint8_t * pucRead, BaseType_t xOk )
{
    ReportWriter_t xW;
    uint32_t ulW[ 3 ];
    uint32_t ulR[ 3 ];
    size_t x;

    for( x = 0U; x < 3U; x++ )
    {
        ulW[ x ] = pucWritten[ x ];
        ulR[ x ] = pucRead[ x ];
    }

    if( prvBegin( &xW, "imu_rate" ) != 0U )
    {
        prvPutU32( &xW, "hz", usHz );
        prvPutU32Array( &xW, "w", ulW, 3U );
        prvPutU32Array( &xW, "r", ulR, 3U );
        prvPutU32( &xW, "ok", ( xOk != pdFALSE ) ? 1U : 0U );
        prvFinish( &xW );
    }
}

/*---------------------------------------------------------------------------*/
void Report_Boot( void )
{
    ReportWriter_t xW;

    if( prvBegin( &xW, "boot" ) != 0U )
    {
        prvPutU32( &xW, "ep", ( uint32_t ) SysState_GetEpoch() );
        prvPutStr( &xW, "fw", APP_FW_VERSION_STR );
        prvFinish( &xW );
    }
}

/*---------------------------------------------------------------------------*/
void Report_Hello( void )
{
    ReportWriter_t xW;

    if( prvBegin( &xW, "hello" ) != 0U )
    {
        prvPutStr( &xW, "fw", APP_FW_VERSION_STR );
        prvPutStr( &xW, "board", APP_BOARD_STR );

        /* Build-time text (report 062, B13): the address made from the same
         * four numbers as net_app.c's array, the ports checked below. */
        prvPutStr( &xW, "ip", APP_NET_IP_STR );
        prvPutKey( &xW, "ports" );
        prvPutRaw( &xW, APP_UDP_PORTS_JSON );

        prvPutU32( &xW, "ep", ( uint32_t ) SysState_GetEpoch() );
        prvFinish( &xW );
    }
}

/*---------------------------------------------------------------------------*/
void Report_Error( const char * pcReason )
{
    ReportWriter_t xW;

    if( prvBegin( &xW, "error" ) != 0U )
    {
        prvPutStr( &xW, "reason", pcReason );
        prvFinish( &xW );
    }
}

/*---------------------------------------------------------------------------*/
void Report_RxError( const char * pcReason )
{
    ReportWriter_t xW;

    if( prvBegin( &xW, "rx_err" ) != 0U )
    {
        prvPutStr( &xW, "reason", pcReason );
        prvFinish( &xW );
    }
}

/*---------------------------------------------------------------------------*/
void Report_Stats( const StatsSnapshot_t * pxSnap, const ReportInject_t * pxInj )
{
    ReportWriter_t xW;

    if( ( pxSnap != NULL ) && ( prvBegin( &xW, "st" ) != 0U ) )
    {
        prvPutResources( &xW, pxSnap, pxInj );
        prvFinish( &xW );
    }
}

/*---------------------------------------------------------------------------*/
void Report_State( const StatsSnapshot_t * pxSnap, const ReportInject_t * pxInj )
{
    ReportWriter_t xW;

    if( ( pxSnap != NULL ) && ( prvBegin( &xW, "state" ) != 0U ) )
    {
        prvPutResources( &xW, pxSnap, pxInj );
        prvFinish( &xW );
    }
}

/*---------------------------------------------------------------------------*/
void Report_Env( uint32_t ulTick, const ReportEnvChannel_t * pxChannels, size_t uxCount )
{
    ReportWriter_t xW;
    size_t i;

    if( uxCount > APP_ENV_MAX_CHANNELS )
    {
        uxCount = APP_ENV_MAX_CHANNELS;
    }

    if( ( pxChannels == NULL ) && ( uxCount > 0U ) )
    {
        uxCount = 0U;
    }

    if( prvBegin( &xW, "env" ) != 0U )
    {
        prvPutU32( &xW, "t", ulTick );
        prvPutKey( &xW, "ch" );
        prvPutChar( &xW, '[' );

        for( i = 0U; i < uxCount; i++ )
        {
            if( i > 0U )
            {
                prvPutChar( &xW, ',' );
            }

            /* [count,min,max,mean] */
            prvPutChar( &xW, '[' );
            prvPutDigits( &xW, pxChannels[ i ].ulCount );
            prvPutChar( &xW, ',' );
            prvPutI32Digits( &xW, pxChannels[ i ].sMin );
            prvPutChar( &xW, ',' );
            prvPutI32Digits( &xW, pxChannels[ i ].sMax );
            prvPutChar( &xW, ',' );
            prvPutI32Digits( &xW, pxChannels[ i ].sMean );
            prvPutChar( &xW, ']' );
        }

        prvPutChar( &xW, ']' );
        prvFinish( &xW );
    }
}

/*---------------------------------------------------------------------------*/
/* prvPutResources: the fields of "st" / "state" (docs 18.3, see report.h).  */
/* STATE and the fault reasons are read live; everything else comes from    */
/* the snapshot, so the levels in one line belong together.                 */
/*---------------------------------------------------------------------------*/
static void prvPutResources( ReportWriter_t * pxW, const StatsSnapshot_t * pxSnap, const ReportInject_t * pxInj )
{
    uint32_t ulTmp[ 3 ];
    size_t i;

    prvPutU32( pxW, "t", pxSnap->ulTick );
    prvPutU32( pxW, "ep", ( uint32_t ) SysState_GetEpoch() );
    prvPutU32( pxW, "s", ( uint32_t ) ( SysState_Get() & 0x07U ) );
    prvPutU32( pxW, "why", SysState_GetFaultReasons() );

    ulTmp[ 0 ] = ( pxInj != NULL ) ? pxInj->ulMask : 0U;
    ulTmp[ 1 ] = ( pxInj != NULL ) ? pxInj->ulHoldMs : 0U;
    ulTmp[ 2 ] = ( pxInj != NULL ) ? pxInj->ulTxDelayMs : 0U;
    prvPutU32Array( pxW, "inj", ulTmp, 3U );

    ulTmp[ 0 ] = pxSnap->ulNbFree;
    ulTmp[ 1 ] = pxSnap->ulNbMinFree;
    prvPutU32Array( pxW, "nb", ulTmp, 2U );

    prvPutKey( pxW, "ipq" );
    prvPutLevel( pxW, pxSnap->ulIpqMinFree );

    /* Pools in registration order: [0] crit, [1] low, [2] report. */
    ulTmp[ 0 ] = pxSnap->ulPoolFree[ 0 ];
    ulTmp[ 1 ] = pxSnap->ulPoolMinFree[ 0 ];
    prvPutU32Array( pxW, "cp", ulTmp, 2U );
    ulTmp[ 0 ] = pxSnap->ulPoolFree[ 1 ];
    ulTmp[ 1 ] = pxSnap->ulPoolMinFree[ 1 ];
    prvPutU32Array( pxW, "lp", ulTmp, 2U );
    ulTmp[ 0 ] = pxSnap->ulPoolFree[ 2 ];
    ulTmp[ 1 ] = pxSnap->ulPoolMinFree[ 2 ];
    prvPutU32Array( pxW, "rp", ulTmp, 2U );

    ulTmp[ 0 ] = pxSnap->ulGauges[ eGaugeCtrlqMinFree ];
    ulTmp[ 1 ] = pxSnap->ulGauges[ eGaugeWorkqMinFree ];
    ulTmp[ 2 ] = pxSnap->ulGauges[ eGaugeUartTxqMinFree ];
    prvPutU32Array( pxW, "q", ulTmp, 3U );

    ulTmp[ 0 ] = pxSnap->ulGauges[ eGaugeSbUartRxMax ];
    ulTmp[ 1 ] = pxSnap->ulGauges[ eGaugeSbCritMax ];
    ulTmp[ 2 ] = pxSnap->ulGauges[ eGaugeSbLowMax ];
    prvPutU32Array( pxW, "sb", ulTmp, 3U );

    prvPutU32( pxW, "lag", pxSnap->ulCounters[ eStatUartRxDmaLag ] );

    ulTmp[ 0 ] = pxSnap->ulHeapFree;
    ulTmp[ 1 ] = pxSnap->ulHeapMinFree;
    prvPutU32Array( pxW, "heap", ulTmp, 2U );

    /* 7 application tasks (eAppTaskId_t order), then the IP and idle task. */
    prvPutKey( pxW, "hwm" );
    prvPutChar( pxW, '[' );
    for( i = 0U; i < ( size_t ) eAppTaskCount; i++ )
    {
        prvPutLevel( pxW, pxSnap->ulStackHwm[ i ] );
        prvPutChar( pxW, ',' );
    }
    prvPutLevel( pxW, pxSnap->ulIpTaskStackHwm );
    prvPutChar( pxW, ',' );
    prvPutLevel( pxW, pxSnap->ulIdleStackHwm );
    prvPutChar( pxW, ']' );
}

/*---------------------------------------------------------------------------*/
/* prvPutLevel: a resource level; STATS_NA (= UINT32_MAX, also a low-water   */
/* gauge never updated) is written as -1 so the line stays short.           */
/*---------------------------------------------------------------------------*/
static void prvPutLevel( ReportWriter_t * pxW, uint32_t ulValue )
{
    if( ulValue == STATS_NA )
    {
        prvPutRaw( pxW, "-1" );
    }
    else
    {
        prvPutDigits( pxW, ulValue );
    }
}

/*---------------------------------------------------------------------------*/
/* prvPutU32Array: ,"<pcKey>":[v0,v1,...], each through prvPutLevel.         */
/*---------------------------------------------------------------------------*/
static void prvPutU32Array( ReportWriter_t * pxW, const char * pcKey, const uint32_t * pulValues, size_t uxCount )
{
    size_t i;

    prvPutKey( pxW, pcKey );
    prvPutChar( pxW, '[' );

    for( i = 0U; i < uxCount; i++ )
    {
        if( i > 0U )
        {
            prvPutChar( pxW, ',' );
        }
        prvPutLevel( pxW, pulValues[ i ] );
    }

    prvPutChar( pxW, ']' );
}

/*---------------------------------------------------------------------------*/
/* prvPutI32Digits: signed decimal (ENV min / max / mean, R5). The magnitude */
/* is taken as uint32_t so INT32_MIN works too.                              */
/*---------------------------------------------------------------------------*/
static void prvPutI32Digits( ReportWriter_t * pxW, int32_t lValue )
{
    uint32_t ulMagnitude;

    if( lValue < 0 )
    {
        prvPutChar( pxW, '-' );
        ulMagnitude = ( uint32_t ) ( -( lValue + 1 ) ) + 1U;
    }
    else
    {
        ulMagnitude = ( uint32_t ) lValue;
    }

    prvPutDigits( pxW, ulMagnitude );
}

/*---------------------------------------------------------------------------*/
/* prvBegin: take a block (no wait) and write {"type":"<pcType>". 0: no      */
/* block, the report is dropped (the pool hook counts REPORT_DROP).          */
/*---------------------------------------------------------------------------*/
static uint8_t prvBegin( ReportWriter_t * pxW, const char * pcType )
{
    pxW->pxDesc = ( ReportDesc_t * ) Pool_Get( &xReportPool, 0U );
    pxW->uxPos = 0U;
    pxW->ucOverflow = 0U;

    if( pxW->pxDesc != NULL )
    {
        prvPutRaw( pxW, "{\"type\":\"" );
        prvPutRaw( pxW, pcType );
        prvPutChar( pxW, '"' );
    }

    return ( pxW->pxDesc != NULL ) ? 1U : 0U;
}

/*---------------------------------------------------------------------------*/
/* prvPutChar: one character if it fits; otherwise mark the overflow (R3).   */
/*---------------------------------------------------------------------------*/
static void prvPutChar( ReportWriter_t * pxW, char cChar )
{
    if( pxW->uxPos < APP_REPORT_MAX_CHARS )
    {
        pxW->pxDesc->cText[ pxW->uxPos ] = cChar;
        pxW->uxPos++;
    }
    else
    {
        pxW->ucOverflow = 1U;
    }
}

/*---------------------------------------------------------------------------*/
/* prvPutRaw: our own constant text (JSON punctuation, keys, type names),    */
/* copied as it is.                                                          */
/*---------------------------------------------------------------------------*/
static void prvPutRaw( ReportWriter_t * pxW, const char * pcText )
{
    while( *pcText != '\0' )
    {
        prvPutChar( pxW, *pcText );
        pcText++;
    }
}

/*---------------------------------------------------------------------------*/
/* prvPutKey: ,"<pcKey>":                                                    */
/*---------------------------------------------------------------------------*/
static void prvPutKey( ReportWriter_t * pxW, const char * pcKey )
{
    prvPutRaw( pxW, ",\"" );
    prvPutRaw( pxW, pcKey );
    prvPutRaw( pxW, "\":" );
}

/*---------------------------------------------------------------------------*/
/* prvPutU32: ,"<pcKey>":<decimal>.                                          */
/*---------------------------------------------------------------------------*/
static void prvPutU32( ReportWriter_t * pxW, const char * pcKey, uint32_t ulValue )
{
    prvPutKey( pxW, pcKey );
    prvPutDigits( pxW, ulValue );
}

/*---------------------------------------------------------------------------*/
/* prvPutDigits: <decimal> alone (also inside arrays and the IP address). No */
/* snprintf (reentrancy, stack, flash): the digits are built backwards in a  */
/* 10-character buffer.                                                      */
/*---------------------------------------------------------------------------*/
static void prvPutDigits( ReportWriter_t * pxW, uint32_t ulValue )
{
    char cDigits[ 10 ];             /* UINT32_MAX = 4294967295: 10 digits */
    size_t uxCount = 0U;

    do
    {
        cDigits[ uxCount ] = ( char ) ( '0' + ( char ) ( ulValue % 10U ) );
        uxCount++;
        ulValue /= 10U;
    } while( ulValue != 0U );

    while( uxCount > 0U )
    {
        uxCount--;
        prvPutChar( pxW, cDigits[ uxCount ] );
    }
}

/*---------------------------------------------------------------------------*/
/* prvPutStr: ,"<pcKey>":"<pcValue>". '"', '\' and control characters would  */
/* break the JSON line: they become '?' (R4). NULL writes "".                */
/*---------------------------------------------------------------------------*/
static void prvPutStr( ReportWriter_t * pxW, const char * pcKey, const char * pcValue )
{
    char cChar;

    prvPutKey( pxW, pcKey );
    prvPutChar( pxW, '"' );

    if( pcValue != NULL )
    {
        while( *pcValue != '\0' )
        {
            cChar = *pcValue;

            if( ( cChar == '"' ) || ( cChar == '\\' ) || ( ( uint8_t ) cChar < 0x20U ) || ( ( uint8_t ) cChar == 0x7FU ) )
            {
                cChar = '?';
            }

            prvPutChar( pxW, cChar );
            pcValue++;
        }
    }

    prvPutChar( pxW, '"' );
}

/*---------------------------------------------------------------------------*/
/* prvFinish: close the object and hand the block to uartTx. If something    */
/* did not fit, or uartTx refused the block, the block goes back to the      */
/* pool and the report is dropped.                                           */
/*---------------------------------------------------------------------------*/
static void prvFinish( ReportWriter_t * pxW )
{
    prvPutChar( pxW, '}' );

    /* R3: every report's longest form fits (report 038): an overflow is a
     * programming error. Stop in debug; never send half a JSON line. */
    configASSERT( pxW->ucOverflow == 0U );

    if( pxW->ucOverflow != 0U )
    {
        Report_Release( pxW->pxDesc );
    }
    else
    {
        pxW->pxDesc->usLen = ( uint16_t ) pxW->uxPos;

        if( UartTx_PostReport( pxW->pxDesc ) != pdPASS )
        {
            /* uart_tx counted it (TXQ_FULL / TXQ_RESERVE). */
            Report_Release( pxW->pxDesc );
        }
    }

    pxW->pxDesc = NULL;
}

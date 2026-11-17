/**
  ******************************************************************************
  * @file    udp_in.c
  * @brief   UDP receive callbacks -> MessageBuffers, and the sockets
  *          (docs D3, 3.1 steps 3-4, 5.1). Künye: report 052; MessageBuffer:
  *          report 062 (B6).
  ******************************************************************************
  * RTOS objects:
  *   sbCrit   static MessageBuffer (a StreamBuffer in message mode),
  *            APP_SB_CRIT_BYTES usable. Writer: the IP task only
  *            (prvOnCritReceive), wait 0. Reader: critParser only (P6),
  *            <= APP_PARSER_WAIT_MS.
  *   sbLow    static MessageBuffer, APP_SB_LOW_BYTES usable. Writer: the IP
  *            task only (prvOnLowReceive), wait 0. Reader: lowParser only
  *            (P1), <= APP_PARSER_WAIT_MS.
  *            The kernel wakes a reader with ITS task notification index 0
  *            (stream_buffer.c:191): the parsers must not use it otherwise.
  */
#include "udp_in.h"

#include <string.h>

#include "FreeRTOS.h"
#include "task.h"           /* configASSERT uses taskDISABLE_INTERRUPTS */
#include "message_buffer.h"
#include "FreeRTOS_IP.h"
#include "FreeRTOS_Sockets.h"

#include "stm32f7xx.h"      /* DWT->CYCCNT (M1) */

#include "app_config.h"
#include "app_stats.h"
#include "app_types.h"      /* UdpRecHdr_t */

/* What differs between the two lanes; the code is the same (D3). */
typedef struct xUDP_LANE
{
    MessageBufferHandle_t * pxStream;   /* &sbCrit / &sbLow (set by UdpIn_Init) */
    size_t uxUsableBytes;               /* usable size of that MessageBuffer     */
    uint16_t usPort;                    /* host order                            */
    eGaugeId_t eGaugeMax;
    eStatId_t eStatCb;
    eStatId_t eStatWritten;
    eStatId_t eStatFull;
    eStatId_t eStatBadLen;
} UdpLane_t;

/* The kernel keeps one byte of a StreamBuffer free (stream_buffer.c:783),
 * message mode too: the storage is one byte larger than the usable size
 * (W4). Every message also takes APP_MB_LENGTH_BYTES of it. */
static StaticMessageBuffer_t xSbCritBuf;
static uint8_t ucSbCritStorage[ APP_SB_CRIT_BYTES + 1U ];
static MessageBufferHandle_t sbCrit = NULL;     /* NULL = UdpIn_Init not called */

static StaticMessageBuffer_t xSbLowBuf;
static uint8_t ucSbLowStorage[ APP_SB_LOW_BYTES + 1U ];
static MessageBufferHandle_t sbLow = NULL;

/* A record is put together here and written with one call (docs 5.1). Both
 * callbacks run in the IP task, one after the other: one array is enough
 * (W1). 4-aligned for the header copy. */
static uint8_t ucRecord[ APP_REC_MAX_BYTES ] __attribute__( ( aligned( 4 ) ) );

static const UdpLane_t xCritLane =
{
    &sbCrit, APP_SB_CRIT_BYTES, APP_UDP_PORT_CRIT, eGaugeSbCritMax,
    eStatCbCrit, eStatSbWrittenCrit, eStatCbCritSbFull, eStatDgramBadLenCrit
};
static const UdpLane_t xLowLane =
{
    &sbLow, APP_SB_LOW_BYTES, APP_UDP_PORT_LOW, eGaugeSbLowMax,
    eStatCbLow, eStatSbWrittenLow, eStatCbLowSbFull, eStatDgramBadLenLow
};

static BaseType_t prvOnCritReceive( Socket_t xSocket, void * pvData, size_t xLength,
                                    const struct freertos_sockaddr * pxFrom,
                                    const struct freertos_sockaddr * pxDest );
static BaseType_t prvOnLowReceive( Socket_t xSocket, void * pvData, size_t xLength,
                                   const struct freertos_sockaddr * pxFrom,
                                   const struct freertos_sockaddr * pxDest );
static void prvWriteRecord( const UdpLane_t * pxLane, uint32_t ulRxCycles, const void * pvData,
                            size_t xLength, const struct freertos_sockaddr * pxFrom );
static Socket_t prvOpenSocket( uint16_t usPort, FOnUDPReceive_t pxOnReceive, uint32_t ulSendTimeoutMs );

/* Written once by UdpIn_OpenSockets (ctrlTask); xCtrlSocket last, so a
 * non-NULL xCtrlSocket means all three are open (W5, W6). */
static Socket_t xCtrlSocket = NULL;     /* 5000: receive callback + replies */
static Socket_t xLowSocket = NULL;      /* 5001: receive callback only      */
static Socket_t xTelemSocket = NULL;    /* 5002: telemetry, send only       */

_Static_assert( APP_REC_MAX_BYTES == ( APP_REC_HEADER_BYTES + APP_PKT_MAX_BYTES ), "record = header + largest packet" );

/*---------------------------------------------------------------------------*/
BaseType_t UdpIn_Init( void )
{
    if( sbCrit == NULL )
    {
        /* Static storage: the kernel asserts the only failure causes (NULL
         * buffers, a size not above the length field, stream_buffer.c:
         * 359, 453-479), all fixed here, so there is no NULL to check
         * (report 062, B9). */
        sbCrit = xMessageBufferCreateStatic( sizeof( ucSbCritStorage ), ucSbCritStorage, &xSbCritBuf );
        sbLow = xMessageBufferCreateStatic( sizeof( ucSbLowStorage ), ucSbLowStorage, &xSbLowBuf );
    }

    return pdPASS;
}

/*---------------------------------------------------------------------------*/
MessageBufferHandle_t UdpIn_GetStream( eAppClass_t eClass )
{
    /* Internal callers only (the two parsers): another class is a bug. */
    configASSERT( eClass < eAppClassCount );

    return ( eClass == eAppClassCrit ) ? sbCrit : sbLow;
}

/*---------------------------------------------------------------------------*/
/* The receive callbacks (IP task, FreeRTOS_UDP_IPv4.c:409-438). xLength is  */
/* the UDP payload without Ethernet padding (FreeRTOS_IP.c:1918-1933).       */
/* Always 1 = consumed (W9): the datagram never goes to the socket list and  */
/* the stack releases the network buffer right away (FreeRTOS_IP.c:1857).    */
/*---------------------------------------------------------------------------*/
static BaseType_t prvOnCritReceive( Socket_t xSocket, void * pvData, size_t xLength,
                                    const struct freertos_sockaddr * pxFrom,
                                    const struct freertos_sockaddr * pxDest )
{
    uint32_t ulRxCycles = DWT->CYCCNT;      /* M1, first thing (W7) */

    ( void ) xSocket;
    ( void ) pxDest;

    prvWriteRecord( &xCritLane, ulRxCycles, pvData, xLength, pxFrom );

    return 1;
}

static BaseType_t prvOnLowReceive( Socket_t xSocket, void * pvData, size_t xLength,
                                   const struct freertos_sockaddr * pxFrom,
                                   const struct freertos_sockaddr * pxDest )
{
    uint32_t ulRxCycles = DWT->CYCCNT;

    ( void ) xSocket;
    ( void ) pxDest;

    prvWriteRecord( &xLowLane, ulRxCycles, pvData, xLength, pxFrom );

    return 1;
}

/*---------------------------------------------------------------------------*/
/* prvWriteRecord: header + payload as ONE message (docs 5.1). With wait 0  */
/* the kernel writes the whole message (length in front) or nothing and     */
/* returns 0 (stream_buffer.c:1021-1043): no room check, no half record.    */
/*---------------------------------------------------------------------------*/
static void prvWriteRecord( const UdpLane_t * pxLane, uint32_t ulRxCycles, const void * pvData,
                            size_t xLength, const struct freertos_sockaddr * pxFrom )
{
    MessageBufferHandle_t xBuffer = *( pxLane->pxStream );
    UdpRecHdr_t xHdr;
    size_t uxRecord = APP_REC_HEADER_BYTES + xLength;
    size_t uxSpaces;

    Stats_Inc( pxLane->eStatCb );

    if( xLength > APP_PKT_MAX_BYTES )
    {
        /* Larger than any packet and than the parser's buffer (W2). An
         * empty datagram goes on: the parser rejects it as PKT_SHORT. */
        Stats_Inc( pxLane->eStatBadLen );
    }
    else
    {
        xHdr.ulSrcIp = pxFrom->sin_address.ulIP_IPv4;      /* network order (W10) */
        xHdr.usSrcPort = pxFrom->sin_port;                 /* network order */
        xHdr.usDstPort = pxLane->usPort;                   /* host order */
        xHdr.ulRxCycles = ulRxCycles;

        ( void ) memcpy( ucRecord, &xHdr, APP_REC_HEADER_BYTES );
        ( void ) memcpy( &ucRecord[ APP_REC_HEADER_BYTES ], pvData, xLength );

        /* Only for the gauge: the level this write reaches, read before it.
         * The send may wake critParser (P6), which preempts the IP task
         * inside the call and empties the buffer again (W8). */
        uxSpaces = xMessageBufferSpacesAvailable( xBuffer );

        if( xMessageBufferSend( xBuffer, ucRecord, uxRecord, 0U ) == uxRecord )
        {
            Stats_GaugeMax( pxLane->eGaugeMax,
                            ( uint32_t ) ( ( pxLane->uxUsableBytes - uxSpaces ) + APP_MB_LENGTH_BYTES + uxRecord ) );
            Stats_Inc( pxLane->eStatWritten );
        }
        else
        {
            /* 0: no room for the whole message, nothing was written. */
            Stats_Inc( pxLane->eStatFull );
        }
    }
}

/*---------------------------------------------------------------------------*/
BaseType_t UdpIn_OpenSockets( void )
{
    BaseType_t xResult = pdPASS;
    Socket_t xCtrl;
    Socket_t xLow = NULL;
    Socket_t xTelem = NULL;

    /* The callbacks write into the MessageBuffers: UdpIn_Init first. */
    configASSERT( sbCrit != NULL );

    if( xCtrlSocket == NULL )
    {
        xCtrl = prvOpenSocket( APP_UDP_PORT_CRIT, prvOnCritReceive, APP_CTRL_SEND_TIMEOUT_MS );

        if( xCtrl != NULL )
        {
            xLow = prvOpenSocket( APP_UDP_PORT_LOW, prvOnLowReceive, 0U );
        }

        if( xLow != NULL )
        {
            xTelem = prvOpenSocket( APP_UDP_PORT_TELEM, NULL, APP_TELEM_SEND_TIMEOUT_MS );
        }

        if( xTelem != NULL )
        {
            /* xCtrlSocket last: it marks "all open" (W6). */
            xTelemSocket = xTelem;
            xLowSocket = xLow;
            xCtrlSocket = xCtrl;
        }
        else
        {
            /* None is left open (W5); the caller retries or raises FAULT. */
            if( xLow != NULL )
            {
                ( void ) FreeRTOS_closesocket( xLow );
            }

            if( xCtrl != NULL )
            {
                ( void ) FreeRTOS_closesocket( xCtrl );
            }

            xResult = pdFAIL;
        }
    }

    return xResult;
}

/*---------------------------------------------------------------------------*/
struct xSOCKET * UdpIn_GetCtrlSocket( void )
{
    return xCtrlSocket;
}

struct xSOCKET * UdpIn_GetTelemSocket( void )
{
    return xTelemSocket;
}

/*---------------------------------------------------------------------------*/
/* prvOpenSocket: one UDP socket bound to usPort. pxOnReceive != NULL: the   */
/* receive callback; NULL: a send-only socket, every datagram to it is       */
/* dropped by the stack (FREERTOS_SO_UDP_MAX_RX_PACKETS 0, W3), otherwise    */
/* the first one would hold a network buffer for good                        */
/* (FreeRTOS_UDP_IPv4.c:439-451). ulSendTimeoutMs != 0: FREERTOS_SO_SNDTIMEO */
/* (docs 10.3). FreeRTOS_bind waits for the IP task without a time limit     */
/* (FreeRTOS_Sockets.c:1752, :1764). NULL on any failure, nothing left open. */
/*---------------------------------------------------------------------------*/
static Socket_t prvOpenSocket( uint16_t usPort, FOnUDPReceive_t pxOnReceive, uint32_t ulSendTimeoutMs )
{
    Socket_t xSocket;
    F_TCP_UDP_Handler_t xHandler;
    struct freertos_sockaddr xAddress;
    TickType_t xTimeout;
    UBaseType_t uxMaxRx = 0U;
    BaseType_t xOk = pdTRUE;

    /* The socket and its event group come from the heap (FreeRTOS_Sockets.c:674, :686). */
    xSocket = FreeRTOS_socket( FREERTOS_AF_INET, FREERTOS_SOCK_DGRAM, FREERTOS_IPPROTO_UDP );

    if( xSocket == FREERTOS_INVALID_SOCKET )
    {
        xSocket = NULL;
    }
    else
    {
        if( pxOnReceive != NULL )
        {
            ( void ) memset( &xHandler, 0, sizeof( xHandler ) );
            xHandler.pxOnUDPReceive = pxOnReceive;
            xOk = ( FreeRTOS_setsockopt( xSocket, 0, FREERTOS_SO_UDP_RECV_HANDLER, &xHandler, sizeof( xHandler ) ) == 0 ) ? pdTRUE : pdFALSE;
        }
        else
        {
            xOk = ( FreeRTOS_setsockopt( xSocket, 0, FREERTOS_SO_UDP_MAX_RX_PACKETS, &uxMaxRx, sizeof( uxMaxRx ) ) == 0 ) ? pdTRUE : pdFALSE;
        }

        if( ( xOk != pdFALSE ) && ( ulSendTimeoutMs != 0U ) )
        {
            xTimeout = pdMS_TO_TICKS( ulSendTimeoutMs );
            xOk = ( FreeRTOS_setsockopt( xSocket, 0, FREERTOS_SO_SNDTIMEO, &xTimeout, sizeof( xTimeout ) ) == 0 ) ? pdTRUE : pdFALSE;
        }

        if( xOk != pdFALSE )
        {
            ( void ) memset( &xAddress, 0, sizeof( xAddress ) );
            xAddress.sin_len = ( uint8_t ) sizeof( xAddress );
            xAddress.sin_family = ( uint8_t ) FREERTOS_AF_INET4;
            xAddress.sin_port = FreeRTOS_htons( usPort );
            xAddress.sin_address.ulIP_IPv4 = 0U;        /* any local address */
            xOk = ( FreeRTOS_bind( xSocket, &xAddress, sizeof( xAddress ) ) == 0 ) ? pdTRUE : pdFALSE;
        }

        if( xOk == pdFALSE )
        {
            ( void ) FreeRTOS_closesocket( xSocket );
            xSocket = NULL;
        }
    }

    return xSocket;
}

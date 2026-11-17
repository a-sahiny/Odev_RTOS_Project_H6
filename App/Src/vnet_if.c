/**
  ******************************************************************************
  * @file    vnet_if.c
  * @brief   VNET: a FreeRTOS-Plus-TCP network interface over USART1, and the
  *          vnetRx task (docs 3.1 step 2, 10.1, 13, 14). Künye: report 050.
  ******************************************************************************
  * RTOS object (one):
  *   vnetRx   static task, APP_PRIO_VNET_RX (4), APP_STACK_VNET_RX_WORDS.
  *            Waits on sbUartRx (xStreamBufferReceive, <= APP_UART_RX_WAIT_MS), on a
  *            network buffer (<= APP_VNET_NETBUF_WAIT_MS) and on the IP task's
  *            event queue (<= APP_VNET_IPQ_WAIT_MS). Its task notification
  *            index 0 belongs to the StreamBuffer (stream_buffer.c:191).
  * Used, owned elsewhere: sbUartRx (uart_link, vnetRx is its only reader),
  * the network buffer pool and xNetworkEventQueue (stack), xUartTxQueue
  * (uart_tx, through prvVNetOutput in the IP task).
  * Data without an RTOS object, vnetRx only: xLineRx, xLine, ucRxFrameBuf.
  * pxVNetInterface: written once by NetApp_Init before the scheduler.
  */
#include "vnet_if.h"

#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "stream_buffer.h"
#include "FreeRTOS_IP.h"
#include "FreeRTOS_IP_Private.h"        /* IPStackEvent_t, xSendEventStructToIPTask, checksums */
#include "FreeRTOS_Routing.h"
#include "NetworkBufferManagement.h"

#include "app_config.h"
#include "app_stats.h"
#include "console.h"
#include "crc.h"
#include "line_rx.h"
#include "pkt_format.h"                 /* Pkt_ReadU16 */
#include "report.h"
#include "uart_link.h"
#include "uart_tx.h"

/* Header layout (IEEE 802.3, RFC 791, RFC 768). */
#define VNET_ETH_HDR_BYTES          14U
#define VNET_ETH_TYPE_OFFSET        12U
#define VNET_ETH_TYPE_IPV4          0x0800U
#define VNET_ETH_TYPE_ARP           0x0806U
#define VNET_MAC_BYTES              6U
#define VNET_IP_MIN_HDR_BYTES       20U
#define VNET_IP_PROTO_OFFSET        9U
#define VNET_IP_PROTO_UDP           17U
#define VNET_UDP_DST_PORT_OFFSET    2U

/* Bytes read from sbUartRx per call (V10). */
#define VNET_READ_CHUNK             64U

/* The frame copy starts 2 bytes into a 4-aligned array, so the IP header
 * (offset 14) is 4-aligned, as in the stack's own buffers (ipBUFFER_PADDING). */
#define VNET_FRAME_OFFSET           2U

static struct xNetworkInterface * pxVNetInterface = NULL;

static StaticTask_t xVNetRxTcb;
static StackType_t xVNetRxStack[ APP_STACK_VNET_RX_WORDS ];
static TaskHandle_t xVNetRxTask = NULL;

static LineRx_t xLineRx;
static HostLine_t xLine;
static uint8_t ucRxFrameBuf[ VNET_FRAME_OFFSET + LINE_RX_FRAME_CAP ] __attribute__( ( aligned( 4 ) ) );

static BaseType_t prvVNetInitialise( NetworkInterface_t * pxInterface );
static BaseType_t prvVNetOutput( NetworkInterface_t * pxInterface,
                                 NetworkBufferDescriptor_t * const pxDescriptor,
                                 BaseType_t xReleaseAfterSend );
static BaseType_t prvVNetGetLinkStatus( NetworkInterface_t * pxInterface );
static void prvVNetRxTask( void * pvParameters );
static void prvOnResult( eLineRxResult_t eResult );
static void prvOnLine( void );
static void prvCountFrame( eLineFcs_t eFcs );
static void prvDeliverFrame( uint8_t * pucFrame, size_t uxLen, eLineFcs_t eFcs );
static eStatId_t prvCheckFrame( const uint8_t * pucFrame, size_t uxLen );
static eStatId_t prvEmulateRxChecksum( uint8_t * pucFrame, size_t uxLen );
static void prvCountUdpPort( const uint8_t * pucFrame, size_t uxLen );
static void prvReject( eStatId_t eId );

_Static_assert( APP_FRAME_FCS_BYTES == 4U, "FCS is one CRC-32" );
_Static_assert( LINE_RX_FRAME_CAP >= ( APP_FRAME_MAX_BYTES + APP_FRAME_FCS_BYTES ), "frame copy holds frame + FCS" );
_Static_assert( sizeof( "vnetRx" ) <= configMAX_TASK_NAME_LEN, "task name fits" );

/*---------------------------------------------------------------------------*/
struct xNetworkInterface * VNet_FillInterfaceDescriptor( BaseType_t xIndex, struct xNetworkInterface * pxInterface )
{
    /* Same steps as pxSTM32_FillInterfaceDescriptor (NetworkInterface.c:1998-2019). */
    ( void ) xIndex;

    ( void ) memset( pxInterface, 0, sizeof( *pxInterface ) );
    pxInterface->pcName = "vnet0";
    pxInterface->pvArgument = NULL;
    pxInterface->pfInitialise = prvVNetInitialise;
    pxInterface->pfOutput = prvVNetOutput;
    pxInterface->pfGetPhyLinkStatus = prvVNetGetLinkStatus;

    pxVNetInterface = pxInterface;

    return FreeRTOS_AddNetworkInterface( pxInterface );
}

/*---------------------------------------------------------------------------*/
/* prvVNetInitialise (IP task): start the UART reception. The IP task calls  */
/* it again later if it fails.                                               */
/*---------------------------------------------------------------------------*/
static BaseType_t prvVNetInitialise( NetworkInterface_t * pxInterface )
{
    ( void ) pxInterface;

    return UartLink_StartRx();
}

/*---------------------------------------------------------------------------*/
/* prvVNetOutput (IP task): hand the frame to uartTx without waiting         */
/* (docs 10.1). Refused (queue full, reserve, bad length): the buffer goes   */
/* back here, in the IP task (docs 9.1).                                     */
/*---------------------------------------------------------------------------*/
static BaseType_t prvVNetOutput( NetworkInterface_t * pxInterface,
                                 NetworkBufferDescriptor_t * const pxDescriptor,
                                 BaseType_t xReleaseAfterSend )
{
    BaseType_t xResult = pdFAIL;

    ( void ) pxInterface;

    /* Zero-copy only, as the STM32 driver (V12): with
     * ipconfigZERO_COPY_TX_DRIVER 1 the stack always passes pdTRUE. */
    configASSERT( xReleaseAfterSend == pdTRUE );

    if( xReleaseAfterSend != pdFALSE )
    {
        xResult = UartTx_PostFrame( pxDescriptor );

        if( xResult != pdPASS )
        {
            vReleaseNetworkBufferAndDescriptor( pxDescriptor );
        }
    }

    return xResult;
}

/*---------------------------------------------------------------------------*/
static BaseType_t prvVNetGetLinkStatus( NetworkInterface_t * pxInterface )
{
    /* No cable: the UART is always "linked". */
    ( void ) pxInterface;

    return pdTRUE;
}

/*---------------------------------------------------------------------------*/
BaseType_t VNet_CreateRxTask( void )
{
    if( xVNetRxTask == NULL )
    {
        LineRx_Init( &xLineRx, pdMS_TO_TICKS( APP_LINE_IDLE_TIMEOUT_MS ) );

        /* Static TCB and stack: NULL only for NULL buffers, which the kernel
         * asserts itself (tasks.c:1276-1290; report 062, B9). */
        xVNetRxTask = xTaskCreateStatic( prvVNetRxTask,
                                         "vnetRx",
                                         APP_STACK_VNET_RX_WORDS,
                                         NULL,
                                         APP_PRIO_VNET_RX,
                                         xVNetRxStack,
                                         &xVNetRxTcb );

        Stats_SetTaskHandle( eAppTaskVnetRx, xVNetRxTask );
    }

    return pdPASS;
}

/*---------------------------------------------------------------------------*/
/* prvVNetRxTask: bytes -> lines -> console / frames. The only reader of     */
/* sbUartRx. Back at xStreamBufferReceive at least every APP_UART_RX_WAIT_MS, so */
/* LineRx_Poll sees a silent line within 200 ms + 20 ms.                     */
/*---------------------------------------------------------------------------*/
static void prvVNetRxTask( void * pvParameters )
{
    uint8_t ucChunk[ VNET_READ_CHUNK ];
    size_t uxRead;
    size_t i;
    TickType_t xNow;
    uint32_t ulLoss;

    const StreamBufferHandle_t xRx = UartLink_GetRxStream();

    ( void ) pvParameters;

    for( ; ; )
    {
        /* sbUartRx has one reader, this task (stream_buffer.c:1119); the
         * kernel wakes it with its notification index 0 (report 092). */
        uxRead = xStreamBufferReceive( xRx, ucChunk, sizeof( ucChunk ), pdMS_TO_TICKS( APP_UART_RX_WAIT_MS ) );

        /* After the read (V10): a loss reported meanwhile also drops the
         * line these bytes belong to; the safe side. */
        xNow = xTaskGetTickCount();
        ulLoss = UartLink_GetLossCount();

        for( i = 0U; i < uxRead; i++ )
        {
            prvOnResult( LineRx_Push( &xLineRx, ucChunk[ i ], xNow, ulLoss ) );
        }

        prvOnResult( LineRx_Poll( &xLineRx, xNow ) );
    }
}

/*---------------------------------------------------------------------------*/
/* prvOnResult: one line event -> counters (decision L1: line_rx does not    */
/* count, this is the single mapping, docs 18.2 L2).                         */
/*---------------------------------------------------------------------------*/
static void prvOnResult( eLineRxResult_t eResult )
{
    switch( eResult )
    {
        case eLineRxNone:
            break;

        case eLineRxReady:
            Stats_Inc( eStatLinesTotal );
            prvOnLine();
            break;

        case eLineRxTooLong:
            Stats_Inc( eStatLinesTotal );
            prvReject( eStatLineTooLong );
            break;

        case eLineRxUartLoss:
            Stats_Inc( eStatLinesTotal );
            prvReject( eStatLineUartLoss );
            break;

        case eLineRxTimeout:
            Stats_Inc( eStatLinesTotal );
            prvReject( eStatLineTimeout );
            break;

        default:
            configASSERT( eResult == eLineRxNone );
            break;
    }
}

/*---------------------------------------------------------------------------*/
/* prvOnLine: a complete line (docs 13.5, 14).                               */
/*---------------------------------------------------------------------------*/
static void prvOnLine( void )
{
    uint8_t * pucFrame = &ucRxFrameBuf[ VNET_FRAME_OFFSET ];

    switch( LineRx_ParseLine( &xLineRx, &xLine, pucFrame, LINE_RX_FRAME_CAP ) )
    {
        case eLineHello:
        case eLineState:
        case eLineHold:
        case eLineTxDelay:
            Stats_Inc( eStatConsoleCmds );
            Console_Handle( &xLine );
            break;

        case eLineUnknown:
            /* The console answers {"type":"error","reason":"unknown_command"}. */
            Stats_Inc( eStatLineUnknown );
            Console_Handle( &xLine );
            break;

        case eLineBadArg:
            Stats_Inc( eStatLineBadArg );
            Console_Handle( &xLine );
            break;

        case eLineBadFlag:
            prvReject( eStatLineBadFlag );
            break;

        case eLineBadHex:
            prvReject( eStatLineBadHex );
            break;

        case eLineFrameLength:
            /* A frame line all the same (decision L1): counted as a frame. */
            prvCountFrame( xLine.eFcs );
            prvReject( eStatFrameLength );
            break;

        case eLineFrame:
            prvCountFrame( xLine.eFcs );
            prvDeliverFrame( pucFrame, xLine.uxFrameLen, xLine.eFcs );
            break;

        default:
            configASSERT( pdFALSE );
            break;
    }
}

/*---------------------------------------------------------------------------*/
static void prvCountFrame( eLineFcs_t eFcs )
{
    Stats_Inc( eStatFrames );

    if( eFcs == eLineFcs1 )
    {
        Stats_Inc( eStatRxFcs1 );
    }
    else if( eFcs == eLineFcs0 )
    {
        Stats_Inc( eStatRxFcs0 );
    }
    else
    {
        Stats_Inc( eStatRxFcsNone );
    }
}

/*---------------------------------------------------------------------------*/
/* prvDeliverFrame: checks, then a network buffer for a valid frame only     */
/* (docs 14: no side effect for a bad line), then the IP task.              */
/*---------------------------------------------------------------------------*/
static void prvDeliverFrame( uint8_t * pucFrame, size_t uxLen, eLineFcs_t eFcs )
{
    eStatId_t eReject = eStatCount;
    uint32_t ulFcs;
    NetworkBufferDescriptor_t * pxDesc;
    IPStackEvent_t xRxEvent;

    if( eFcs == eLineFcs1 )
    {
        /* line_rx guarantees 60..256 bytes plus the 4 FCS bytes. The FCS is
         * sent low byte first (crc.h). */
        uxLen -= APP_FRAME_FCS_BYTES;
        ulFcs = Crc32_Eth( pucFrame, uxLen );

        if( ( pucFrame[ uxLen ] != ( uint8_t ) ulFcs ) ||
            ( pucFrame[ uxLen + 1U ] != ( uint8_t ) ( ulFcs >> 8 ) ) ||
            ( pucFrame[ uxLen + 2U ] != ( uint8_t ) ( ulFcs >> 16 ) ) ||
            ( pucFrame[ uxLen + 3U ] != ( uint8_t ) ( ulFcs >> 24 ) ) )
        {
            eReject = eStatFrameFcs;
        }
    }

    if( eReject == eStatCount )
    {
        eReject = prvCheckFrame( pucFrame, uxLen );
    }

    if( eReject == eStatCount )
    {
        eReject = prvEmulateRxChecksum( pucFrame, uxLen );
    }

    if( eReject != eStatCount )
    {
        prvReject( eReject );
    }
    else
    {
        pxDesc = pxGetNetworkBufferWithDescriptor( uxLen, pdMS_TO_TICKS( APP_VNET_NETBUF_WAIT_MS ) );

        if( pxDesc == NULL )
        {
            /* A resource drop, not bad input: counted, no rx_err (V2). */
            Stats_Inc( eStatRxNoNetbuf );
        }
        else
        {
            /* As the STM32 driver fills a received buffer (V11). */
            ( void ) memcpy( pxDesc->pucEthernetBuffer, pucFrame, uxLen );
            pxDesc->xDataLength = uxLen;
            pxDesc->pxInterface = pxVNetInterface;
            pxDesc->pxEndPoint = FreeRTOS_MatchingEndpoint( pxVNetInterface, pxDesc->pucEthernetBuffer );
            #if ( ipconfigUSE_LINKED_RX_MESSAGES != 0 )
                pxDesc->pxNextBuffer = NULL;
            #endif

            xRxEvent.eEventType = eNetworkRxEvent;
            xRxEvent.pvData = ( void * ) pxDesc;

            if( xSendEventStructToIPTask( &xRxEvent, pdMS_TO_TICKS( APP_VNET_IPQ_WAIT_MS ) ) != pdPASS )
            {
                /* prvSendRxEvent (NetworkInterface.c:1646-1658): the buffer is
                 * still ours. */
                iptraceETHERNET_RX_EVENT_LOST();
                vReleaseNetworkBufferAndDescriptor( pxDesc );
                Stats_Inc( eStatRxIpqFull );
            }
            else
            {
                Stats_Inc( eStatToStack );
                prvCountUdpPort( pucFrame, uxLen );
            }
        }
    }
}

/*---------------------------------------------------------------------------*/
/* prvCheckFrame: what a MAC with type filtering would drop (docs 14,        */
/* ipconfigETHERNET_DRIVER_FILTERS_FRAME_TYPES 1). eStatCount = accepted.    */
/*---------------------------------------------------------------------------*/
static eStatId_t prvCheckFrame( const uint8_t * pucFrame, size_t uxLen )
{
    static const uint8_t ucBroadcast[ VNET_MAC_BYTES ] = { 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU };
    eStatId_t eReject = eStatCount;
    const NetworkEndPoint_t * pxEndPoint = FreeRTOS_FirstEndPoint( pxVNetInterface );
    uint16_t usEtherType = Pkt_ReadU16( &pucFrame[ VNET_ETH_TYPE_OFFSET ] );

    /* line_rx guarantees >= 60 bytes, so the Ethernet header is there. */
    ( void ) uxLen;
    configASSERT( pxEndPoint != NULL );     /* NetApp_Init added it: a bug otherwise (B9) */

    if( ( usEtherType != VNET_ETH_TYPE_IPV4 ) && ( usEtherType != VNET_ETH_TYPE_ARP ) )
    {
        eReject = eStatFrameEthertype;
    }
    else if( ( memcmp( pucFrame, pxEndPoint->xMACAddress.ucBytes, VNET_MAC_BYTES ) != 0 ) &&
             ( memcmp( pucFrame, ucBroadcast, VNET_MAC_BYTES ) != 0 ) )
    {
        eReject = eStatFrameDstMac;
    }
    else
    {
        /* accepted */
    }

    return eReject;
}

/*---------------------------------------------------------------------------*/
/* prvEmulateRxChecksum: the checksum offload the stack expects with         */
/* ipconfigDRIVER_INCLUDED_RX_IP_CHECKSUM 1. The same two calls as           */
/* FreeRTOS_IPv4.c:484-514 (V1), with a counter for each reason.             */
/*---------------------------------------------------------------------------*/
static eStatId_t prvEmulateRxChecksum( uint8_t * pucFrame, size_t uxLen )
{
    eStatId_t eReject = eStatCount;
    size_t uxIpHdrBytes;

    if( Pkt_ReadU16( &pucFrame[ VNET_ETH_TYPE_OFFSET ] ) == VNET_ETH_TYPE_IPV4 )
    {
        uxIpHdrBytes = ( size_t ) ( pucFrame[ VNET_ETH_HDR_BYTES ] & 0x0FU ) * 4U;

        if( ( ( pucFrame[ VNET_ETH_HDR_BYTES ] >> 4 ) != 4U ) ||
            ( uxIpHdrBytes < VNET_IP_MIN_HDR_BYTES ) ||
            ( uxLen < ( VNET_ETH_HDR_BYTES + uxIpHdrBytes ) ) )
        {
            /* Not a usable IPv4 header: the header check fails as well. */
            eReject = eStatIpCsum;
        }
        else if( usGenerateChecksum( 0U, &pucFrame[ VNET_ETH_HDR_BYTES ], uxIpHdrBytes ) != ipCORRECT_CRC )
        {
            eReject = eStatIpCsum;
        }
        else if( usGenerateProtocolChecksum( pucFrame, uxLen, pdFALSE ) != ipCORRECT_CRC )
        {
            /* UDP / ICMP checksum, or a length it cannot accept. A UDP
             * checksum of 0 is refused too (ipconfigUDP_PASS_ZERO_CHECKSUM_PACKETS 0). */
            eReject = eStatUdpCsum;
        }
        else
        {
            /* accepted */
        }
    }

    return eReject;
}

/*---------------------------------------------------------------------------*/
/* prvCountUdpPort: TO_STACK_CRIT / _LOW by UDP destination port, for the    */
/* L3 comparison with the callbacks (docs 18.2). Called for an accepted      */
/* frame, so the IPv4 header is valid.                                       */
/*---------------------------------------------------------------------------*/
static void prvCountUdpPort( const uint8_t * pucFrame, size_t uxLen )
{
    size_t uxUdp;
    uint16_t usDstPort;

    if( ( Pkt_ReadU16( &pucFrame[ VNET_ETH_TYPE_OFFSET ] ) == VNET_ETH_TYPE_IPV4 ) &&
        ( pucFrame[ VNET_ETH_HDR_BYTES + VNET_IP_PROTO_OFFSET ] == VNET_IP_PROTO_UDP ) )
    {
        uxUdp = VNET_ETH_HDR_BYTES + ( ( size_t ) ( pucFrame[ VNET_ETH_HDR_BYTES ] & 0x0FU ) * 4U );

        if( uxLen >= ( uxUdp + VNET_UDP_DST_PORT_OFFSET + 2U ) )
        {
            usDstPort = Pkt_ReadU16( &pucFrame[ uxUdp + VNET_UDP_DST_PORT_OFFSET ] );

            if( usDstPort == APP_UDP_PORT_CRIT )
            {
                Stats_Inc( eStatToStackCrit );
            }
            else if( usDstPort == APP_UDP_PORT_LOW )
            {
                Stats_Inc( eStatToStackLow );
            }
            else
            {
                /* another port: only TO_STACK */
            }
        }
    }
}

/*---------------------------------------------------------------------------*/
/* prvReject: count the reason and tell the host (V2). The rx_err reason is  */
/* the counter's name in lower case (docs 13.5).                             */
/*---------------------------------------------------------------------------*/
static void prvReject( eStatId_t eId )
{
    const char * pcReason;

    switch( eId )
    {
        case eStatLineTooLong:    pcReason = "line_too_long";   break;
        case eStatLineUartLoss:   pcReason = "line_uart_loss";  break;
        case eStatLineTimeout:    pcReason = "line_timeout";    break;
        case eStatLineBadFlag:    pcReason = "line_bad_flag";   break;
        case eStatLineBadHex:     pcReason = "line_bad_hex";    break;
        case eStatFrameLength:    pcReason = "frame_length";    break;
        case eStatFrameFcs:       pcReason = "frame_fcs";       break;
        case eStatFrameEthertype: pcReason = "frame_ethertype"; break;
        case eStatFrameDstMac:    pcReason = "frame_dst_mac";   break;
        case eStatIpCsum:         pcReason = "ip_csum";         break;
        case eStatUdpCsum:        pcReason = "udp_csum";        break;
        default:                  pcReason = NULL;              break;
    }

    configASSERT( pcReason != NULL );
    Stats_Inc( eId );

    if( pcReason != NULL )
    {
        Report_RxError( pcReason );
    }
}

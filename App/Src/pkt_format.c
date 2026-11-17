/**
  ******************************************************************************
  * @file    pkt_format.c
  * @brief   Application packet format (docs 15). Knows the layout only, not
  *          the meaning: which type may arrive on which port, value ranges
  *          and the duplicate window are handled by pkt_parser.c and cmd.c.
  ******************************************************************************
  * No RTOS objects, no global or static variables: every function works on
  * its parameters and locals only, so any task may call it at any time.
  */
#include "pkt_format.h"

#include <string.h>

#include "app_config.h"
#include "crc.h"

/* Field offsets inside a packet (docs 15.1) */
#define PKT_OFS_SOF         0U
#define PKT_OFS_VER         1U
#define PKT_OFS_TYPE        2U
#define PKT_OFS_FLAGS       3U
#define PKT_OFS_SEQ         4U
#define PKT_OFS_REQ_ID      6U
#define PKT_OFS_LEN         8U
#define PKT_OFS_PAYLOAD     9U

_Static_assert( PKT_OFS_PAYLOAD == APP_PKT_HEADER_BYTES, "payload starts right after the header" );

static ePktResult_t prvCheckHeader( const uint8_t * pucBuf, size_t uxLen );

/*---------------------------------------------------------------------------*/
/* Big-endian byte helpers                                                   */
/*---------------------------------------------------------------------------*/
uint16_t Pkt_ReadU16( const uint8_t * pucSrc )
{
    /* Most significant byte first. */
    return ( uint16_t ) ( ( ( uint16_t ) pucSrc[ 0 ] << 8 ) | ( uint16_t ) pucSrc[ 1 ] );
}

uint32_t Pkt_ReadU32( const uint8_t * pucSrc )
{
    return ( ( uint32_t ) pucSrc[ 0 ] << 24 ) |
           ( ( uint32_t ) pucSrc[ 1 ] << 16 ) |
           ( ( uint32_t ) pucSrc[ 2 ] << 8 ) |
           ( uint32_t ) pucSrc[ 3 ];
}

void Pkt_WriteU16( uint8_t * pucDst, uint16_t usValue )
{
    pucDst[ 0 ] = ( uint8_t ) ( usValue >> 8 );
    pucDst[ 1 ] = ( uint8_t ) ( usValue & 0xFFU );
}

void Pkt_WriteU32( uint8_t * pucDst, uint32_t ulValue )
{
    pucDst[ 0 ] = ( uint8_t ) ( ulValue >> 24 );
    pucDst[ 1 ] = ( uint8_t ) ( ( ulValue >> 16 ) & 0xFFUL );
    pucDst[ 2 ] = ( uint8_t ) ( ( ulValue >> 8 ) & 0xFFUL );
    pucDst[ 3 ] = ( uint8_t ) ( ulValue & 0xFFUL );
}

/*---------------------------------------------------------------------------*/
/* Decode                                                                    */
/*---------------------------------------------------------------------------*/

/* Checks everything that can be checked before the CRC position is known:
 * size, SOF, VER and the LEN field against the datagram size. */
static ePktResult_t prvCheckHeader( const uint8_t * pucBuf, size_t uxLen )
{
    size_t uxPayloadLen;

    if( uxLen < APP_PKT_OVERHEAD_BYTES )
    {
        return ePktErrShort;
    }

    if( pucBuf[ PKT_OFS_SOF ] != APP_PKT_SOF )
    {
        return ePktErrSof;
    }

    if( pucBuf[ PKT_OFS_VER ] != APP_PKT_VERSION )
    {
        return ePktErrVersion;
    }

    uxPayloadLen = ( size_t ) pucBuf[ PKT_OFS_LEN ];

    /* Strict size: no missing and no trailing bytes are tolerated. */
    if( ( uxPayloadLen > APP_PKT_MAX_PAYLOAD ) ||
        ( uxLen != ( APP_PKT_OVERHEAD_BYTES + uxPayloadLen ) ) )
    {
        return ePktErrLength;
    }

    return ePktOk;
}

ePktResult_t Pkt_Decode( const uint8_t * pucBuf, size_t uxLen, PktView_t * pxView )
{
    ePktResult_t eResult;
    size_t uxCrcOffset;

    if( ( pucBuf == NULL ) || ( pxView == NULL ) )
    {
        return ePktErrNull;
    }

    eResult = prvCheckHeader( pucBuf, uxLen );

    if( eResult != ePktOk )
    {
        return eResult;
    }

    /* The header is trusted only for the length here: the CRC byte is the
     * last byte, it covers everything before it. */
    uxCrcOffset = uxLen - 1U;

    if( Crc8_Compute( pucBuf, uxCrcOffset ) != pucBuf[ uxCrcOffset ] )
    {
        return ePktErrCrc;
    }

    /* Checked after the CRC: a reserved bit in a corrupted packet is counted
     * as corruption, a reserved bit in an intact packet as a protocol error. */
    if( ( pucBuf[ PKT_OFS_FLAGS ] & ( uint8_t ) ~APP_PKT_FLAG_KNOWN_MASK ) != 0U )
    {
        return ePktErrFlags;
    }

    pxView->ucType = pucBuf[ PKT_OFS_TYPE ];
    pxView->ucFlags = pucBuf[ PKT_OFS_FLAGS ];
    pxView->usSeq = Pkt_ReadU16( &pucBuf[ PKT_OFS_SEQ ] );
    pxView->usReqId = Pkt_ReadU16( &pucBuf[ PKT_OFS_REQ_ID ] );
    pxView->ucLen = pucBuf[ PKT_OFS_LEN ];
    pxView->pucPayload = &pucBuf[ PKT_OFS_PAYLOAD ];

    return ePktOk;
}

/*---------------------------------------------------------------------------*/
/* Encode                                                                    */
/*---------------------------------------------------------------------------*/
size_t Pkt_Encode( uint8_t * pucOut,
                   size_t uxCap,
                   const PktHeader_t * pxHdr,
                   const uint8_t * pucPayload,
                   size_t uxPayloadLen )
{
    size_t uxTotal;

    /* Every check happens before the first write: on error pucOut keeps its
     * old content. */
    if( ( pucOut == NULL ) || ( pxHdr == NULL ) )
    {
        return 0U;
    }

    if( ( pucPayload == NULL ) && ( uxPayloadLen > 0U ) )
    {
        return 0U;
    }

    if( uxPayloadLen > APP_PKT_MAX_PAYLOAD )
    {
        return 0U;
    }

    if( ( pxHdr->ucFlags & ( uint8_t ) ~APP_PKT_FLAG_KNOWN_MASK ) != 0U )
    {
        return 0U;
    }

    uxTotal = APP_PKT_OVERHEAD_BYTES + uxPayloadLen;

    if( uxCap < uxTotal )
    {
        return 0U;
    }

    /* Payload first: memmove is correct even when the caller already built
     * the payload in place (pucPayload == pucOut + 9) or overlaps it. */
    if( uxPayloadLen > 0U )
    {
        ( void ) memmove( &pucOut[ PKT_OFS_PAYLOAD ], pucPayload, uxPayloadLen );
    }

    pucOut[ PKT_OFS_SOF ] = APP_PKT_SOF;
    pucOut[ PKT_OFS_VER ] = APP_PKT_VERSION;
    pucOut[ PKT_OFS_TYPE ] = pxHdr->ucType;
    pucOut[ PKT_OFS_FLAGS ] = pxHdr->ucFlags;
    Pkt_WriteU16( &pucOut[ PKT_OFS_SEQ ], pxHdr->usSeq );
    Pkt_WriteU16( &pucOut[ PKT_OFS_REQ_ID ], pxHdr->usReqId );
    pucOut[ PKT_OFS_LEN ] = ( uint8_t ) uxPayloadLen;

    /* CRC8 over everything before it, written as the last byte. */
    pucOut[ uxTotal - 1U ] = Crc8_Compute( pucOut, uxTotal - 1U );

    return uxTotal;
}

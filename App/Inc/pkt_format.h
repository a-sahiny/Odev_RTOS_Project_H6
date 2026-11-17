/**
  ******************************************************************************
  * @file    pkt_format.h
  * @brief   Application packet format (docs 15): decode, encode and the
  *          big-endian byte helpers. Pure C, no RTOS calls, re-entrant.
  ******************************************************************************
  *   SOF(0xA5) VER(0x01) TYPE FLAGS SEQ(2) REQ_ID(2) LEN | PAYLOAD | CRC8
  * Multi-byte fields are big-endian (network order, D13) and are always read
  * and written byte by byte: no struct casts, no alignment assumptions.
  */
#ifndef PKT_FORMAT_H
#define PKT_FORMAT_H

#include <stddef.h>
#include <stdint.h>

/*----------------------------------------------------------------------------
 * Big-endian byte helpers. The caller guarantees that the pointer is valid
 * and that 2 (U16) or 4 (U32) bytes are available; no checks are made here.
 * Signed fields (ENV, CAL, TELEM) are read as ( int16_t ) Pkt_ReadU16( p ):
 * GCC defines this conversion as two's complement.
 *--------------------------------------------------------------------------*/
uint16_t Pkt_ReadU16( const uint8_t * pucSrc );
uint32_t Pkt_ReadU32( const uint8_t * pucSrc );
void Pkt_WriteU16( uint8_t * pucDst, uint16_t usValue );
void Pkt_WriteU32( uint8_t * pucDst, uint32_t ulValue );

/*----------------------------------------------------------------------------
 * Decode
 *--------------------------------------------------------------------------*/

/* Result of Pkt_Decode. Every rejected packet has exactly one reason, which
 * the caller counts (docs 16.2: these are the "silent" rejects, no reply). */
typedef enum ePKT_RESULT
{
    ePktOk = 0,
    ePktErrNull,        /* NULL buffer or NULL view                         */
    ePktErrShort,       /* fewer bytes than header + CRC (10)               */
    ePktErrSof,         /* first byte is not APP_PKT_SOF                    */
    ePktErrVersion,     /* VER is not APP_PKT_VERSION                       */
    ePktErrLength,      /* LEN > 200, or datagram size != 10 + LEN          */
    ePktErrCrc,         /* CRC8 over bytes 0 .. 8+LEN does not match        */
    ePktErrFlags,       /* a reserved FLAGS bit is set                      */
    ePktResultCount
} ePktResult_t;

/* Decoded fields of one packet. pucPayload points INTO the caller's buffer
 * (no copy): the view is valid only while that buffer is left unchanged. */
typedef struct xPKT_VIEW
{
    uint8_t ucType;
    uint8_t ucFlags;
    uint16_t usSeq;
    uint16_t usReqId;
    uint8_t ucLen;                  /* payload bytes, 0..APP_PKT_MAX_PAYLOAD */
    const uint8_t * pucPayload;     /* first payload byte (valid if ucLen 0) */
} PktView_t;

/* Checks one complete datagram (uxLen = UDP payload length) and fills
 * *pxView only when the result is ePktOk; on any error *pxView is untouched.
 * Checks in this order: size, SOF, VER, LEN, CRC8, FLAGS. */
ePktResult_t Pkt_Decode( const uint8_t * pucBuf, size_t uxLen, PktView_t * pxView );

/*----------------------------------------------------------------------------
 * Encode
 *--------------------------------------------------------------------------*/

/* Header fields the sender chooses; SOF, VER, LEN and CRC8 are filled in. */
typedef struct xPKT_HEADER
{
    uint8_t ucType;
    uint8_t ucFlags;        /* only APP_PKT_FLAG_RESP / APP_PKT_FLAG_ERR      */
    uint16_t usSeq;
    uint16_t usReqId;
} PktHeader_t;

/* Builds one packet in pucOut (capacity uxCap) and returns its total size
 * (10 + uxPayloadLen). Returns 0 and leaves pucOut untouched when: pucOut or
 * pxHdr is NULL, pucPayload is NULL with uxPayloadLen > 0, uxPayloadLen > 200,
 * uxCap < 10 + uxPayloadLen, or pxHdr->ucFlags has a reserved bit.
 * The payload may already be in place at pucOut + 9 (in-place build). */
size_t Pkt_Encode( uint8_t * pucOut,
                   size_t uxCap,
                   const PktHeader_t * pxHdr,
                   const uint8_t * pucPayload,
                   size_t uxPayloadLen );

#endif /* PKT_FORMAT_H */

/**
  ******************************************************************************
  * @file    line_rx.c
  * @brief   Host line assembler and parser (docs 14).
  ******************************************************************************
  * No RTOS object, no RTOS call, no static or global variable: everything is
  * in the caller's LineRx_t (owned by vnetRx). Host-testable as it is.
  */
#include "line_rx.h"

#include <string.h>     /* memcmp: keywords are compared by length */

/* prvHexValue for a character that is not a hex digit. */
#define LINE_RX_NOT_HEX     0xFFU

static void prvAppend( LineRx_t * pxRx, uint8_t ucByte );
static void prvDiscard( LineRx_t * pxRx );
static uint8_t prvHexValue( char cChar );
static eLineKind_t prvParseFlag( const char * pcText, size_t uxLen, eLineFcs_t * peFcs, size_t * puxSkip );
static eLineKind_t prvParseFrame( const char * pcText, size_t uxLen, HostLine_t * pxOut,
                                  uint8_t * pucFrame, size_t uxCap );
static uint8_t prvParseU32( const char * pcText, size_t uxLen, uint32_t ulMax, uint32_t * pulValue );

_Static_assert( LINE_RX_FRAME_CAP >= APP_FRAME_MAX_BYTES + APP_FRAME_FCS_BYTES, "frame buffer size" );
_Static_assert( APP_FRAME_MIN_BYTES <= APP_FRAME_MAX_BYTES, "frame length range" );

/*---------------------------------------------------------------------------*/
void LineRx_Init( LineRx_t * pxRx, TickType_t xIdleTimeout )
{
    if( pxRx != NULL )
    {
        pxRx->uxUsed = 0U;
        pxRx->xLastByte = 0U;
        pxRx->xIdleTimeout = xIdleTimeout;
        pxRx->ulLastLoss = 0U;
        pxRx->ucState = ( uint8_t ) eLineRxStateNormal;
        pxRx->ucReady = 0U;
    }
}

/*---------------------------------------------------------------------------*/
eLineRxResult_t LineRx_Push( LineRx_t * pxRx, uint8_t ucByte, TickType_t xNow, uint32_t ulLossCount )
{
    eLineRxResult_t eResult = eLineRxNone;

    if( pxRx != NULL )
    {
        /* The line returned as Ready last time has been handled: a new line
         * starts with this byte. */
        if( pxRx->ucReady != 0U )
        {
            pxRx->ucReady = 0U;
            pxRx->uxUsed = 0U;
        }

        /* Any byte is line activity, also '\r' and bytes being discarded
         * (LineRx_Poll measures the silence from here). */
        pxRx->xLastByte = xNow;

        /* 1. UART loss since the previous byte: bytes are missing from the
         *    line being collected, or from the start of the next one. Drop
         *    it, even when it is still empty (as the skeleton's
         *    console_task). Counted once: not again while discarding. */
        if( ulLossCount != pxRx->ulLastLoss )
        {
            pxRx->ulLastLoss = ulLossCount;

            if( pxRx->ucState == ( uint8_t ) eLineRxStateNormal )
            {
                prvDiscard( pxRx );
                eResult = eLineRxUartLoss;
            }
        }

        /* 2. The byte itself. */
        if( ucByte == ( uint8_t ) '\r' )
        {
            /* ignored in both states */
        }
        else if( ucByte == ( uint8_t ) '\n' )
        {
            if( pxRx->ucState == ( uint8_t ) eLineRxStateDiscard )
            {
                /* Resynchronised: the next byte starts a clean line. */
                pxRx->ucState = ( uint8_t ) eLineRxStateNormal;
                pxRx->uxUsed = 0U;
            }
            else if( pxRx->uxUsed > 0U )
            {
                pxRx->ucReady = 1U;
                eResult = eLineRxReady;
            }
            else
            {
                /* empty line: ignored, not counted */
            }
        }
        else if( pxRx->ucState == ( uint8_t ) eLineRxStateNormal )
        {
            if( pxRx->uxUsed < APP_LINE_MAX_CHARS )
            {
                prvAppend( pxRx, ucByte );
            }
            else
            {
                /* Character number APP_LINE_MAX_CHARS + 1. */
                prvDiscard( pxRx );
                eResult = eLineRxTooLong;
            }
        }
        else
        {
            /* discarding: swallowed */
        }
    }

    return eResult;
}

/*---------------------------------------------------------------------------*/
eLineRxResult_t LineRx_Poll( LineRx_t * pxRx, TickType_t xNow )
{
    eLineRxResult_t eResult = eLineRxNone;

    if( pxRx != NULL )
    {
        /* Unsigned difference: right across a tick wrap. */
        if( ( TickType_t ) ( xNow - pxRx->xLastByte ) >= pxRx->xIdleTimeout )
        {
            if( pxRx->ucState == ( uint8_t ) eLineRxStateDiscard )
            {
                /* L2: the broken line's newline may never come; without
                 * this the next valid line would be swallowed up to its own
                 * newline. Already counted as TooLong / UartLoss: not
                 * counted again. */
                pxRx->ucState = ( uint8_t ) eLineRxStateNormal;
                pxRx->uxUsed = 0U;
            }
            else if( ( pxRx->uxUsed > 0U ) && ( pxRx->ucReady == 0U ) )
            {
                /* Half line: drop it and stay NORMAL (docs 14). Its late
                 * rest, if it ever comes, ends with its own newline and is
                 * rejected by the parser; the line after it starts clean. */
                pxRx->uxUsed = 0U;
                eResult = eLineRxTimeout;
            }
            else
            {
                /* nothing collected, or a complete line waiting to be parsed */
            }
        }
    }

    return eResult;
}

/*---------------------------------------------------------------------------*/
eLineKind_t LineRx_ParseLine( const LineRx_t * pxRx, HostLine_t * pxOut, uint8_t * pucFrame, size_t uxCap )
{
    /* Keywords exactly as the skeleton (decision L4): upper case; HELLO and
     * STATE alone on the line; HOLD / TXDELAY / FRAME followed by ONE space. */
    static const char cFrameKey[] = "FRAME ";
    static const char cHelloKey[] = "HELLO";
    static const char cStateKey[] = "STATE";
    static const char cHoldKey[] = "HOLD ";
    static const char cTxDelayKey[] = "TXDELAY ";
    const size_t uxFrameKeyLen = sizeof( cFrameKey ) - 1U;
    const size_t uxHoldKeyLen = sizeof( cHoldKey ) - 1U;
    const size_t uxTxDelayKeyLen = sizeof( cTxDelayKey ) - 1U;
    eLineKind_t eKind = eLineUnknown;
    const char * pcLine;
    size_t uxLen;

    if( ( pxRx != NULL ) && ( pxOut != NULL ) && ( pucFrame != NULL ) )
    {
        pcLine = pxRx->cLine;
        uxLen = pxRx->uxUsed;

        pxOut->eFcs = eLineFcsNone;
        pxOut->uxFrameLen = 0U;
        pxOut->ulValue = 0U;

        if( ( uxLen >= uxFrameKeyLen ) && ( memcmp( pcLine, cFrameKey, uxFrameKeyLen ) == 0 ) )
        {
            eKind = prvParseFrame( &pcLine[ uxFrameKeyLen ], uxLen - uxFrameKeyLen, pxOut, pucFrame, uxCap );
        }
        else if( ( uxLen == ( sizeof( cHelloKey ) - 1U ) ) && ( memcmp( pcLine, cHelloKey, uxLen ) == 0 ) )
        {
            eKind = eLineHello;
        }
        else if( ( uxLen == ( sizeof( cStateKey ) - 1U ) ) && ( memcmp( pcLine, cStateKey, uxLen ) == 0 ) )
        {
            eKind = eLineState;
        }
        else if( ( uxLen >= uxHoldKeyLen ) && ( memcmp( pcLine, cHoldKey, uxHoldKeyLen ) == 0 ) )
        {
            eKind = ( prvParseU32( &pcLine[ uxHoldKeyLen ], uxLen - uxHoldKeyLen, APP_HOLD_MAX_MS,
                                   &pxOut->ulValue ) != 0U ) ? eLineHold : eLineBadArg;
        }
        else if( ( uxLen >= uxTxDelayKeyLen ) && ( memcmp( pcLine, cTxDelayKey, uxTxDelayKeyLen ) == 0 ) )
        {
            eKind = ( prvParseU32( &pcLine[ uxTxDelayKeyLen ], uxLen - uxTxDelayKeyLen, APP_TXDELAY_MAX_MS,
                                   &pxOut->ulValue ) != 0U ) ? eLineTxDelay : eLineBadArg;
        }
        else
        {
            /* eLineUnknown */
        }

        pxOut->eKind = eKind;
    }

    return eKind;
}

/*---------------------------------------------------------------------------*/
/* prvParseU32: decimal digits only, at least one, value <= ulMax; the       */
/* skeleton's parse_u32 rule (leading zeros allowed, overflow-safe test      */
/* before every multiply). *pulValue is written only on success. 1 = ok.     */
/*---------------------------------------------------------------------------*/
static uint8_t prvParseU32( const char * pcText, size_t uxLen, uint32_t ulMax, uint32_t * pulValue )
{
    uint32_t ulValue = 0U;
    uint32_t ulDigit;
    size_t i;
    uint8_t ucOk = ( uxLen > 0U ) ? 1U : 0U;

    for( i = 0U; ( i < uxLen ) && ( ucOk != 0U ); i++ )
    {
        if( ( pcText[ i ] < '0' ) || ( pcText[ i ] > '9' ) )
        {
            ucOk = 0U;
        }
        else
        {
            ulDigit = ( uint32_t ) ( pcText[ i ] - '0' );

            /* ulValue * 10 + ulDigit <= ulMax, without overflowing. */
            if( ( ulDigit > ulMax ) || ( ulValue > ( ( ulMax - ulDigit ) / 10U ) ) )
            {
                ucOk = 0U;
            }
            else
            {
                ulValue = ( ulValue * 10U ) + ulDigit;
            }
        }
    }

    if( ucOk != 0U )
    {
        *pulValue = ulValue;
    }

    return ucOk;
}

/*---------------------------------------------------------------------------*/
/* prvHexValue: '0'..'9', 'A'..'F', 'a'..'f' -> 0..15; anything else ->       */
/* LINE_RX_NOT_HEX (the skeleton's nibble()).                                */
/*---------------------------------------------------------------------------*/
static uint8_t prvHexValue( char cChar )
{
    uint8_t ucValue = LINE_RX_NOT_HEX;

    if( ( cChar >= '0' ) && ( cChar <= '9' ) )
    {
        ucValue = ( uint8_t ) ( cChar - '0' );
    }
    else if( ( cChar >= 'A' ) && ( cChar <= 'F' ) )
    {
        ucValue = ( uint8_t ) ( ( cChar - 'A' ) + 10 );
    }
    else if( ( cChar >= 'a' ) && ( cChar <= 'f' ) )
    {
        ucValue = ( uint8_t ) ( ( cChar - 'a' ) + 10 );
    }
    else
    {
        /* not a hex digit */
    }

    return ucValue;
}

/*---------------------------------------------------------------------------*/
/* prvParseFlag: the text after "FRAME ". A first field of ONE character     */
/* followed by a space is the FCS flag (docs 13.4): '0' / '1', anything else */
/* is eLineBadFlag. No such field: no flag (eLineFcsNone). A hex field is    */
/* always even, so a one-character field is never hex. *puxSkip = characters */
/* to skip before the hex (2 or 0).                                          */
/*---------------------------------------------------------------------------*/
static eLineKind_t prvParseFlag( const char * pcText, size_t uxLen, eLineFcs_t * peFcs, size_t * puxSkip )
{
    eLineKind_t eKind = eLineFrame;

    *peFcs = eLineFcsNone;
    *puxSkip = 0U;

    if( ( uxLen >= 2U ) && ( pcText[ 1 ] == ' ' ) )
    {
        *puxSkip = 2U;

        if( pcText[ 0 ] == '0' )
        {
            *peFcs = eLineFcs0;
        }
        else if( pcText[ 0 ] == '1' )
        {
            *peFcs = eLineFcs1;
        }
        else
        {
            eKind = eLineBadFlag;
        }
    }

    return eKind;
}

/*---------------------------------------------------------------------------*/
/* prvParseFrame: flag, then the hex (the skeleton's parse_hex rule: the     */
/* WHOLE input is checked before anything is written), then the length.     */
/* Order of the checks: flag -> hex -> length -> write.                      */
/*---------------------------------------------------------------------------*/
static eLineKind_t prvParseFrame( const char * pcText, size_t uxLen, HostLine_t * pxOut,
                                  uint8_t * pucFrame, size_t uxCap )
{
    eLineFcs_t eFcs;
    size_t uxSkip;
    size_t uxHexLen;
    size_t uxBytes;
    size_t uxFcsBytes;
    size_t i;
    const char * pcHex;
    eLineKind_t eKind;

    eKind = prvParseFlag( pcText, uxLen, &eFcs, &uxSkip );

    if( eKind == eLineFrame )
    {
        pxOut->eFcs = eFcs;
        pcHex = &pcText[ uxSkip ];
        uxHexLen = uxLen - uxSkip;

        /* 1. Hex: not empty, even, every character a hex digit. */
        if( ( uxHexLen == 0U ) || ( ( uxHexLen & 1U ) != 0U ) )
        {
            eKind = eLineBadHex;
        }
        else
        {
            for( i = 0U; i < uxHexLen; i++ )
            {
                if( prvHexValue( pcHex[ i ] ) == LINE_RX_NOT_HEX )
                {
                    eKind = eLineBadHex;
                    break;
                }
            }
        }
    }

    if( eKind == eLineFrame )
    {
        /* 2. Length: the frame without its FCS must be 60..256 bytes, and the
         *    bytes must fit the caller's buffer. */
        uxBytes = uxHexLen / 2U;
        uxFcsBytes = ( eFcs == eLineFcs1 ) ? APP_FRAME_FCS_BYTES : 0U;

        if( ( uxBytes < ( APP_FRAME_MIN_BYTES + uxFcsBytes ) ) ||
            ( uxBytes > ( APP_FRAME_MAX_BYTES + uxFcsBytes ) ) ||
            ( uxBytes > uxCap ) )
        {
            eKind = eLineFrameLength;
        }
        else
        {
            /* 3. Everything checked: write. */
            for( i = 0U; i < uxBytes; i++ )
            {
                pucFrame[ i ] = ( uint8_t ) ( ( uint8_t ) ( prvHexValue( pcHex[ 2U * i ] ) << 4 ) |
                                              prvHexValue( pcHex[ ( 2U * i ) + 1U ] ) );
            }

            pxOut->uxFrameLen = uxBytes;
        }
    }

    return eKind;
}

/*---------------------------------------------------------------------------*/
/* prvAppend: one more character; the caller checked that it fits.           */
/*---------------------------------------------------------------------------*/
static void prvAppend( LineRx_t * pxRx, uint8_t ucByte )
{
    pxRx->cLine[ pxRx->uxUsed ] = ( char ) ucByte;
    pxRx->uxUsed++;
}

/*---------------------------------------------------------------------------*/
/* prvDiscard: drop the line and swallow everything up to the next '\n'.     */
/*---------------------------------------------------------------------------*/
static void prvDiscard( LineRx_t * pxRx )
{
    pxRx->uxUsed = 0U;
    pxRx->ucState = ( uint8_t ) eLineRxStateDiscard;
}

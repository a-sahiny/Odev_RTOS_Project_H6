/**
  ******************************************************************************
  * @file    line_rx.h
  * @brief   Host line assembler and parser (docs 13.4, 13.5, 14). Pure C:
  *          no RTOS object, no RTOS call, no global state. All state lives
  *          in the caller's LineRx_t; time comes from the caller (xNow).
  ******************************************************************************
  * One LineRx_t has one user (vnetRx). The functions are reentrant: they
  * touch only their parameters. line_rx counts nothing; the caller turns the
  * results into app_stats counters (decision L1).
  */
#ifndef LINE_RX_H
#define LINE_RX_H

#include <stddef.h>
#include <stdint.h>

#include "FreeRTOS.h"       /* TickType_t only */

#include "app_config.h"

/* What LineRx_Push / LineRx_Poll report. Every value except eLineRxNone is
 * one line for the L2 count (docs 18.2: lines_total). */
typedef enum eLINE_RX_RESULT
{
    eLineRxNone = 0,        /* byte taken, nothing finished               */
    eLineRxReady,           /* a complete line waits for LineRx_ParseLine */
    eLineRxTooLong,         /* line longer than APP_LINE_MAX_CHARS: dropped */
    eLineRxUartLoss,        /* bytes were lost on the UART: line dropped  */
    eLineRxTimeout          /* half line, no byte for xIdleTimeout: dropped */
} eLineRxResult_t;

/* Assembler state (docs 14). */
typedef enum eLINE_RX_STATE
{
    eLineRxStateNormal = 0, /* collecting a line                          */
    eLineRxStateDiscard     /* swallowing the rest of a broken line up to '\n' */
} eLineRxState_t;

typedef struct xLINE_RX
{
    char cLine[ APP_LINE_MAX_CHARS ];   /* the line, without '\r' / '\n', not 0-terminated */
    size_t uxUsed;                      /* characters in cLine                         */
    TickType_t xLastByte;               /* tick of the last byte (any byte)            */
    TickType_t xIdleTimeout;            /* half-line timeout, ticks                    */
    uint32_t ulLastLoss;                /* UART loss count seen with the last byte     */
    uint8_t ucState;                    /* eLineRxState_t                              */
    uint8_t ucReady;                    /* 1: cLine holds a line returned as Ready     */
} LineRx_t;

/* What a complete line is (LineRx_ParseLine, docs 13.4, 13.5, 14). */
typedef enum eLINE_KIND
{
    eLineUnknown = 0,       /* no known keyword (LINE_UNKNOWN)                  */
    eLineFrame,             /* FRAME, flag and hex valid, length 60..256 + FCS  */
    eLineHello,
    eLineState,
    eLineHold,              /* ulValue = ms, 0..APP_HOLD_MAX_MS                 */
    eLineTxDelay,           /* ulValue = ms, 0..APP_TXDELAY_MAX_MS              */
    eLineBadFlag,           /* FRAME x <hex>, x not 0 / 1 (LINE_BAD_FLAG)       */
    eLineBadHex,            /* FRAME with a non-hex character, odd or empty hex (LINE_BAD_HEX) */
    eLineBadArg,            /* HOLD / TXDELAY number missing or out of range (LINE_BAD_ARG) */
    eLineFrameLength        /* FRAME hex valid, frame < 60 or > 256 bytes without FCS (FRAME_LENGTH) */
} eLineKind_t;

/* FCS flag of a FRAME line (D7). */
typedef enum eLINE_FCS
{
    eLineFcsNone = 0,       /* "FRAME <hex>"   (RX_FCS_NONE)                    */
    eLineFcs0,              /* "FRAME 0 <hex>" (RX_FCS_0)                       */
    eLineFcs1               /* "FRAME 1 <hex>" (RX_FCS_1): last 4 bytes are FCS */
} eLineFcs_t;

/* Result of LineRx_ParseLine. The frame bytes themselves go to the caller's
 * buffer (pucFrame). */
typedef struct xHOST_LINE
{
    eLineKind_t eKind;
    eLineFcs_t eFcs;        /* eLineFrame and eLineFrameLength                  */
    size_t uxFrameLen;      /* eLineFrame: bytes in pucFrame, FCS included      */
    uint32_t ulValue;       /* eLineHold / eLineTxDelay                         */
} HostLine_t;

/* Frame bytes with FCS: the size LineRx_ParseLine's pucFrame needs. */
#define LINE_RX_FRAME_CAP       ( APP_FRAME_MAX_BYTES + APP_FRAME_FCS_BYTES )

/* Empty line, NORMAL state, loss count 0 (uart_link starts at 0 too).
 * xIdleTimeout: APP_MS( APP_LINE_IDLE_TIMEOUT_MS ). NULL does nothing. */
void LineRx_Init( LineRx_t * pxRx, TickType_t xIdleTimeout );

/* Feeds one received byte. xNow: the current tick; ulLossCount:
 * UartLink_GetLossCount() read with this byte. eLineRxReady means cLine /
 * uxUsed hold a complete line until the next LineRx_Push. NULL pxRx returns
 * eLineRxNone. */
eLineRxResult_t LineRx_Push( LineRx_t * pxRx, uint8_t ucByte, TickType_t xNow, uint32_t ulLossCount );

/* Checks the silence since the last byte. Call after every xStreamBufferReceive
 * (at least every APP_UART_RX_WAIT_MS). After xIdleTimeout ticks without a
 * byte: a half line is dropped (eLineRxTimeout, state stays NORMAL); a line
 * being discarded ends (back to NORMAL, nothing counted: decision L2). A
 * Ready line that has not been parsed yet is left alone. NULL returns
 * eLineRxNone. */
eLineRxResult_t LineRx_Poll( LineRx_t * pxRx, TickType_t xNow );

/* Decodes the line held after eLineRxReady (read by its length: it is not
 * 0-terminated and may contain any byte). Fills *pxOut; for eLineFrame the
 * frame bytes (FCS included) go to pucFrame (uxCap bytes; LINE_RX_FRAME_CAP
 * is enough). A rejected line writes nothing to pucFrame. Returns
 * pxOut->eKind; eLineUnknown for NULL arguments. */
eLineKind_t LineRx_ParseLine( const LineRx_t * pxRx, HostLine_t * pxOut, uint8_t * pucFrame, size_t uxCap );

#endif /* LINE_RX_H */

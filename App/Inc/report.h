/**
  ******************************************************************************
  * @file    report.h
  * @brief   One-line JSON reports to the host and the report block pool
  *          (docs 5.2, 13.5, 16.6, 18.3, 19).
  ******************************************************************************
  * Every Report_X() takes a block from xReportPool (no wait), writes one JSON
  * line into it and hands it to uartTx. Nothing waits: without a block or a
  * queue slot the report is dropped and counted (REPORT_DROP / TXQ_*).
  * Task context only.
  */
#ifndef REPORT_H
#define REPORT_H

#include <stdint.h>

#include <stddef.h>

#include "FreeRTOS.h"

#include "app_types.h"
#include "app_stats.h"

/* Active artificial injection (docs 13.5, 20): the bit mask and its
 * durations. Owned by console.c; the caller passes it in. */
typedef struct xREPORT_INJECT
{
    uint32_t ulMask;            /* bit 0 HOLD, bit 1 TXDELAY                  */
    uint32_t ulHoldMs;
    uint32_t ulTxDelayMs;
} ReportInject_t;

/* One ENV channel summary (docs 5.2: count, min, max, mean of the i16
 * values). Owned by worker; the caller passes them in. */
typedef struct xREPORT_ENV_CHANNEL
{
    uint32_t ulCount;
    int16_t sMin;
    int16_t sMax;
    int16_t sMean;
} ReportEnvChannel_t;

/* Creates the report pool (APP_REPORT_COUNT blocks) and registers it with
 * app_stats (eStatPoolReportEmpty group, D23). Call once from App_Init,
 * before the scheduler; a second call changes nothing. pdFAIL if the pool
 * or the registration fails. */
BaseType_t Report_Init( void );

/* Gives a report block back. uartTx calls it right after copying the line
 * (without waiting for the DMA); report.c calls it when uartTx refused the
 * block. NULL is ignored. */
void Report_Release( ReportDesc_t * pxReport );

/* {"type":"ack","result":"<pcResult>"}: console command accepted
 * (hold_requested, txdelay_set). */
void Report_Ack( const char * pcResult );

/* {"type":"fault","set":0|1,"why":<APP_FAULT_* bits>}: FAULT raised or
 * cleared (docs 19). */
void Report_Fault( BaseType_t xSet, uint32_t ulWhy );

/* {"type":"imu_rate","hz":<rate>,"w":[div,dlpf,dlpf],"r":[...],"ms":<waited>,
 * "ok":0|1}: every IMU rate write (imu_spi.c): what was written to
 * SMPLRT_DIV / CONFIG / ACCEL_CONFIG2, what reads back, and how long
 * SMPLRT_DIV took to read back the new value. ok 0: it did not (IMU_WHO_AM_I
 * counted as well). meas only. */
void Report_ImuRate( uint16_t usHz, const uint8_t * pucWritten, const uint8_t * pucRead, uint32_t ulWaitMs,
                     BaseType_t xOk );

/* {"type":"boot","ep":<EPOCH>,"fw":"<version>"}: once at start-up, from
 * ctrlTask (docs 16.6). */
void Report_Boot( void );

/* {"type":"hello","fw":...,"board":...,"ip":"a.b.c.d","ports":[5000,5001,5002],
 * "ep":<EPOCH>}: answer to the HELLO console command (docs 13.5). */
void Report_Hello( void );

/* {"type":"error","reason":"<pcReason>"}: console command refused
 * (unknown_command, bad_arg). */
void Report_Error( const char * pcReason );

/* {"type":"rx_err","reason":"<pcReason>"}: a FRAME line was rejected
 * (bad_flag, bad_hex, frame_length, fcs, ...; docs 13.5). */
void Report_RxError( const char * pcReason );

/* {"type":"st",...}: resource levels of docs 18.3, one line (decision R2-A),
 * every APP_STATS_REPORT_PERIOD_MS from worker. Counters are not in it
 * (GET_STATS). Fields, arrays in this order:
 *   t     snapshot tick          ep    EPOCH        s    STATE bits 0..2
 *   why   fault reasons          inj   [mask, hold ms, txdelay ms]
 *   nb    [free, min] network buffers                ipq  IP queue min free
 *   cp / lp / rp  [free, min] of the crit / low / report pools
 *   q     min free of [ctrlQueue, workQueue, xUartTxQueue]
 *   sb    max fill of [sbUartRx, sbCrit, sbLow]      lag  UART RX DMA laps lost
 *   heap  [free, min]
 *   hwm   stack words never used: critParser, ctrlTask, vnetRx, uartTx, meas,
 *         lowParser, worker, IP task, idle
 * A value that is not available (STATS_NA, or a low-water gauge never
 * updated) is written as -1. Longest line ~255 characters. */
void Report_Stats( const StatsSnapshot_t * pxSnap, const ReportInject_t * pxInj );

/* {"type":"state",...}: the same fields as Report_Stats, for the STATE
 * console command (vnetRx). */
void Report_State( const StatsSnapshot_t * pxSnap, const ReportInject_t * pxInj );

/* {"type":"env","t":<tick>,"ch":[[count,min,max,mean],...]}: ENV channel
 * summaries from worker (decision R2-A). uxCount is clipped to
 * APP_ENV_MAX_CHANNELS. Longest line ~310 characters. */
void Report_Env( uint32_t ulTick, const ReportEnvChannel_t * pxChannels, size_t uxCount );

#endif /* REPORT_H */

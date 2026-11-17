/**
  ******************************************************************************
  * @file    app_stats.h
  * @brief   Counters of the whole system (docs 18). Every counter belongs to one
  *          layer and only ever grows; the docs 18.2 equations compare a
  *          layer's input with its outputs over the same window.
  ******************************************************************************
  * Kept free of FreeRTOS / TCP includes on purpose: app_trace_hooks.h includes
  * this header from inside FreeRTOSIPConfig.h, i.e. in every TCP/IP source.
  */
#ifndef APP_STATS_H
#define APP_STATS_H

#include <stdbool.h>
#include <stdint.h>

/* Forward declarations instead of FreeRTOS / fixed_pool includes (see above):
 * "struct tskTaskControlBlock *" is exactly TaskHandle_t, and
 * "struct xFIXED_POOL" is FixedPool_t. */
struct tskTaskControlBlock;
struct xFIXED_POOL;

/*----------------------------------------------------------------------------
 * Counter ids, grouped by layer (docs 18.2). "Crit" = port 5000 lane,
 * "Low" = port 5001 lane.
 *--------------------------------------------------------------------------*/
typedef enum eSTAT_ID
{
    /* L1 UART RX */
    eStatRxBytes = 0,           /* bytes the RX event callback received        */
    eStatUartRxSbOvf,           /* bytes dropped: sbUartRx full                */
    eStatUartRxDmaLag,          /* DMA overtook the callback (data lost)       */
    eStatUartRxTcAfterIdle,     /* empty TC that cannot be a lap: after a wrapping IDLE (091) or < one lap time (092) */
    eStatUartOre,               /* overrun error                               */
    eStatUartFe,                /* framing error                               */
    eStatUartNe,                /* noise error                                 */

    /* L2 lines */
    eStatLinesTotal,
    eStatConsoleCmds,           /* HELLO / STATE / HOLD / TXDELAY              */
    eStatLineTooLong,
    eStatLineUartLoss,
    eStatLineTimeout,
    eStatLineUnknown,
    eStatLineBadFlag,
    eStatLineBadHex,
    eStatLineBadArg,

    /* L2 frames */
    eStatFrames,                /* = RxFcsNone + RxFcs0 + RxFcs1               */
    eStatRxFcsNone,             /* "FRAME <hex>"                               */
    eStatRxFcs0,                /* "FRAME 0 <hex>"                             */
    eStatRxFcs1,                /* "FRAME 1 <hex>"                             */
    eStatFrameLength,
    eStatFrameFcs,
    eStatFrameEthertype,
    eStatFrameDstMac,
    eStatIpCsum,                /* IPv4 header unusable or its checksum wrong  */
    eStatUdpCsum,               /* upper-layer (UDP / ICMP) checksum wrong (V1) */
    eStatRxNoNetbuf,
    eStatRxIpqFull,
    eStatToStack,               /* frames handed to the IP task                */
    eStatToStackCrit,           /* ... of them UDP to port 5000                */
    eStatToStackLow,            /* ... of them UDP to port 5001                */

    /* L3 UDP receive callback (IP task, udp_in.c): Cb = SbWritten + SbFull + DgramBadLen */
    eStatCbCrit,                /* datagrams the port 5000 callback received   */
    eStatSbWrittenCrit,         /* ... written to sbCrit as one record         */
    eStatCbCritSbFull,          /* ... not written: no room for the record     */
    eStatDgramBadLenCrit,       /* ... not written: length 0 or > 210 (W2)     */
    eStatCbLow,
    eStatSbWrittenLow,
    eStatCbLowSbFull,
    eStatDgramBadLenLow,

    /* L4 parsers: critical lane */
    eStatRecReadCrit,
    eStatPktOkCrit,
    eStatPktShortCrit,          /* ePktErrShort                                */
    eStatPktSofCrit,            /* ePktErrSof                                  */
    eStatPktVersionCrit,        /* ePktErrVersion                              */
    eStatPktLengthCrit,         /* ePktErrLength                               */
    eStatPktCrcCrit,            /* ePktErrCrc                                  */
    eStatPktFlagsCrit,          /* ePktErrFlags                                */
    eStatPktWrongPortCrit,      /* a TYPE of another lane (ENV, BULK, CAL, TELEM) */
    eStatPktTypeLenCrit,        /* stays 0: a command with a wrong LEN is answered
                                 * E_LEN by cmd.c and counted in eStatCmdInvalid
                                 * (report 085); kept for the common group layout */
    eStatSeqDupCrit,
    eStatSeqGapCrit,
    eStatSeqOldCrit,
    eStatSeqResyncCrit,
    eStatEvtPostedCrit,
    eStatCtrlqFull,

    /* L4 parsers: low lane */
    eStatRecReadLow,
    eStatPktOkLow,
    eStatPktShortLow,
    eStatPktSofLow,
    eStatPktVersionLow,
    eStatPktLengthLow,
    eStatPktCrcLow,
    eStatPktFlagsLow,
    eStatPktWrongPortLow,
    eStatPktTypeLenLow,
    eStatSeqDupLow,
    eStatSeqGapLow,
    eStatSeqOldLow,
    eStatSeqResyncLow,
    eStatEvtPostedLow,
    eStatWorkqFull,

    /* Pools (D23). Each group is Empty, Invalid, Double in this order:
     * Stats_RegisterPool() gets the first id of a group. */
    eStatPoolCritEmpty,         /* CRIT_DESC_EMPTY                             */
    eStatPoolCritInvalid,
    eStatPoolCritDouble,
    eStatPoolLowEmpty,          /* LOW_DESC_EMPTY                              */
    eStatPoolLowInvalid,
    eStatPoolLowDouble,
    eStatPoolReportEmpty,       /* REPORT_DROP                                 */
    eStatPoolReportInvalid,
    eStatPoolReportDouble,
    eStatPoolUnregistered,      /* pool event from a pool nobody registered    */

    /* L5 handlers. evt_posted_crit + evt_net_posted + evt_posted_low =
     * evt_handled_ctrl + evt_handled_work + evt_no_handler (report 056 E4). */
    eStatEvtHandledCtrl,
    eStatEvtHandledWork,
    eStatEvtNoHandler,          /* event type missing from a handler table     */
    eStatEvtNetPosted,          /* network hook: NetUp / NetDown in ctrlQueue  */
    eStatEvtNetLost,            /* network hook: ctrlQueue full                */
    eStatCmdNew,
    eStatCmdReplay,
    eStatCmdConflict,
    eStatCmdInvalid,            /* answered with an error status: E_LEN, E_RANGE,
                                 * E_STATE, E_TYPE, E_REQID                      */
    eStatCmdReplyFail,         /* FreeRTOS_sendto of a reply gave 0 / < 0 (report 065 K7) */
    eStatCmdImuOk,              /* GET_IMU answered OK (report 092: not in CMD_NEW)          */
    eStatCmdImuRefused,         /* GET_IMU answered with an error (not in CMD_INVALID)       */
    /* worker (report 072 K9): EnvPkts + EnvBad + BulkPkts + CalApplied +
     * CalRejected = evt_handled_work; BulkBad is a part of BulkPkts. */
    eStatEnvPkts,               /* ENV packets taken into the channel table   */
    eStatEnvBad,                /* ENV packet whose n does not match its length */
    eStatBulkPkts,
    eStatBulkBytes,
    eStatBulkBad,
    eStatCalApplied,
    eStatCalRejected,

    /* L6 telemetry and IMU */
    eStatTelemGen,
    eStatTelemSendtoOk,
    eStatTelemSendtoFail,
    eStatImuWhoAmI,
    eStatImuDrdyTimeout,
    eStatImuSpiTimeout,

    /* L7 UART TX. txq_ok = count( eLatTxqCtrl ) + count( eLatTxqOther ) once
     * the queue is empty (report 062, B2). */
    eStatIfOut,                 /* frames the IP task gave to pfOutput         */
    eStatTxReportIn,            /* report blocks given to UartTx_PostReport    */
    eStatTxqOk,                 /* items put in xUartTxQueue (frames + reports) */
    eStatTxqFull,               /* refused: no slot at all                     */
    eStatTxqReserve,            /* non-control item refused: last slots are control */
    eStatTxBadLen,              /* frame refused before the queue: length 0 or > 256 */
    eStatTxLines,
    eStatTxUartStall,
    eStatImuReplyOver10,        /* GET_IMU reply: M1 -> M6 > 10 ms (target; report 086) */
    eStatImuReplyMiss,          /* GET_IMU reply: M1 -> M6 > 12 ms (deadline missed)    */

    /* System state (sys_state.c) */
    eStatFaultRaised,           /* FAULT went 0 -> 1 (SysState_RaiseFault)     */

    /* Inside FreeRTOS-Plus-TCP (app_trace_hooks.h) */
    eStatNbGetFail,             /* iptraceFAILED_TO_OBTAIN_NETWORK_BUFFER      */
    eStatSendtoNoBuf,           /* iptraceNO_BUFFER_FOR_SENDTO                 */
    eStatIpqEventLost,          /* iptraceSTACK_TX_EVENT_LOST                  */
    eStatArpTxDropped,          /* iptracePACKET_DROPPED_TO_GENERATE_ARP       */
    eStatArpDelayed,            /* iptraceDELAYED_ARP_REQUEST_STARTED          */
    eStatArpDelayedFull,        /* iptraceDELAYED_ARP_BUFFER_FULL              */
    eStatArpExpired,            /* iptraceARP_TABLE_ENTRY_EXPIRED              */
    eStatIpEvtRx,               /* iptraceNETWORK_EVENT_RECEIVED: eNetworkRxEvent */
    eStatIpEvtTx,               /* ... eStackTxEvent                           */
    eStatIpEvtArpTimer,         /* ... eARPTimerEvent                          */
    eStatIpEvtOther,            /* ... every other event type                  */

    eStatCount
} eStatId_t;

/* Pool events reported by the traceFIXED_POOL_* hooks. The values are the
 * offsets inside a pool's counter group (Empty, Invalid, Double). */
typedef enum eSTAT_POOL_EVENT
{
    eStatPoolEvtGetFailed = 0,  /* pool empty (or ISR reserve reached)         */
    eStatPoolEvtInvalid,        /* address is not a block of this pool         */
    eStatPoolEvtDouble,         /* block released twice                        */
    eStatPoolEvtCount
} eStatPoolEvent_t;

/* Application tasks whose stack high-water mark the snapshot reports. The
 * IP task and the idle task are read from the stack / kernel directly. */
typedef enum eAPP_TASK_ID
{
    eAppTaskCritParser = 0,
    eAppTaskCtrl,
    eAppTaskVnetRx,
    eAppTaskUartTx,
    eAppTaskMeas,
    eAppTaskLowParser,
    eAppTaskWorker,
    eAppTaskCount
} eAppTaskId_t;

/* Number of pools Stats_RegisterPool can hold. */
#define STATS_MAX_POOLS    3U

/*----------------------------------------------------------------------------
 * Gauge ids (docs 18.3). A gauge is a level that goes up and down; only its
 * extreme since boot is kept. The high-water gauges come first, then the
 * low-water gauges from eGaugeFirstMin on.
 *--------------------------------------------------------------------------*/
typedef enum eGAUGE_ID
{
    /* Highest fill seen, in bytes (Stats_GaugeMax) */
    eGaugeSbUartRxMax = 0,      /* writer: UART RX event callback (ISR)        */
    eGaugeSbCritMax,            /* writer: UDP callback (IP task)              */
    eGaugeSbLowMax,             /* writer: UDP callback (IP task)              */

    /* Lowest free space seen, in items (Stats_GaugeMin) */
    eGaugeFirstMin,
    eGaugeCtrlqMinFree = eGaugeFirstMin,   /* writer: ctrlTask, after a receive (E2) */
    eGaugeWorkqMinFree,                    /* writer: worker, after a receive (E2)   */
    eGaugeUartTxqMinFree,                  /* writers: IP task (pfOutput), report writers */

    eGaugeCount
} eGaugeId_t;

/*----------------------------------------------------------------------------
 * Latencies (docs 8.1): sample count, sum and worst case (report 062, B1:
 * no buckets, no p99). Times are DWT cycle differences converted to
 * microseconds. Each id has exactly ONE writer task.
 *--------------------------------------------------------------------------*/
typedef enum eLAT_ID
{
    eLatCritParse = 0,          /* M1 -> M2: UDP callback -> critParser read   writer: critParser */
    eLatCmd,                    /* M1 -> M3: callback -> reply built (LAT_US)  writer: ctrlTask   */
    eLatTxqCtrl,                /* M4 -> M5: in xUartTxQueue, CTRL class       writer: uartTx     */
    eLatTxqOther,               /* M4 -> M5: every other item (B2)             writer: uartTx     */
    eLatImuReply,               /* M1 -> M6: GET_IMU request -> reply line out writer: uartTx     */
    eLatCount
} eLatId_t;

typedef struct xLAT_HIST
{
    uint32_t ulCount;                           /* samples                    */
    uint32_t ulMaxUs;                           /* worst sample               */
    uint64_t ullSumUs;                          /* mean = sum / count; 32 bits
                                                 * would overflow after ~71 min */
} LatHist_t;

/*----------------------------------------------------------------------------
 * Snapshot (docs 18.1, 18.3): everything at one instant, for the JSON "st"
 * report, GET_STATS and STATE. About 0.6 KB: callers keep a static copy,
 * never one on their stack.
 *--------------------------------------------------------------------------*/

/* "Not available" in the snapshot's resource fields. */
#define STATS_NA    UINT32_MAX

typedef struct xSTATS_SNAPSHOT
{
    uint32_t ulTick;                            /* xTaskGetTickCount()        */
    uint32_t ulCounters[ eStatCount ];          /* one instant: copy and check (report 092) */
    uint32_t ulGauges[ eGaugeCount ];           /* same copy and check        */
    LatHist_t xLat[ eLatCount ];                /* each latency consistent    */

    /* Resource levels, read after the copy */
    uint32_t ulHeapFree;                        /* bytes                      */
    uint32_t ulHeapMinFree;
    uint32_t ulNbFree;                          /* network buffers            */
    uint32_t ulNbMinFree;
    uint32_t ulIpqMinFree;                      /* IP task queue, items       */
    uint32_t ulPoolFree[ STATS_MAX_POOLS ];     /* [0] crit, [1] low, [2] report */
    uint32_t ulPoolMinFree[ STATS_MAX_POOLS ];  /* STATS_NA if not registered */
    uint32_t ulStackHwm[ eAppTaskCount ];       /* words never used; STATS_NA if no handle */
    uint32_t ulIpTaskStackHwm;
    uint32_t ulIdleStackHwm;
} StatsSnapshot_t;

/* Clears every counter, gauge and histogram (high-water gauges to 0,
 * low-water gauges to UINT32_MAX = "nothing seen yet"). Call once from
 * App_Init, before the scheduler. */
void Stats_Init( void );

/* counter += 1 / += ulValue. Lock-free (LDREX/STREX): callable from any task
 * and from any interrupt, whatever its priority. Ids >= eStatCount are
 * ignored (configASSERT in debug). */
void Stats_Inc( eStatId_t eId );
void Stats_Add( eStatId_t eId, uint32_t ulValue );

/* One counter's current value (any context); 0 for an invalid id. For a
 * consistent set of counters use Stats_Snapshot (step 5e). */
uint32_t Stats_Get( eStatId_t eId );

/* Keep the highest (GaugeMax) or lowest (GaugeMin) value ever passed in.
 * Lock-free compare-and-swap: callable from any task and any interrupt.
 * GaugeMax takes only high-water ids, GaugeMin only low-water ids; a wrong
 * id is ignored (configASSERT in debug). */
void Stats_GaugeMax( eGaugeId_t eId, uint32_t ulValue );
void Stats_GaugeMin( eGaugeId_t eId, uint32_t ulValue );

/* One gauge's current extreme (any context); 0 for an invalid id. */
uint32_t Stats_GaugeGet( eGaugeId_t eId );

/* Adds one latency sample of ulCycles DWT cycles. Task context only, and
 * only from the one writer task of eId (see eLatId_t). */
void Stats_LatencyAdd( eLatId_t eId, uint32_t ulCycles );

/* Copies one histogram as a consistent whole (task context). An invalid id
 * gives an all-zero copy. */
void Stats_LatencyGet( eLatId_t eId, LatHist_t * pxOut );

/* Remembers a task's handle for the stack high-water mark. Called by App_Init
 * right after the task is created, before the scheduler starts. */
void Stats_SetTaskHandle( eAppTaskId_t eTask, struct tskTaskControlBlock * xHandle );

/* The handle stored for eTask, or NULL (any context). */
struct tskTaskControlBlock * Stats_GetTaskHandle( eAppTaskId_t eTask );

/* Links a pool to its counter group (D23). eFirst must be the first id of a
 * group: eStatPoolCritEmpty, eStatPoolLowEmpty or eStatPoolReportEmpty.
 * Called by the pool's owner in App_Init, before the scheduler. Returns false
 * (and asserts) for a NULL pool, a wrong eFirst, a pool or group registered
 * twice, or a full table. */
bool Stats_RegisterPool( const struct xFIXED_POOL * pxPool, eStatId_t eFirst );

/* Counts one pool event in the registered pool's group, or in
 * eStatPoolUnregistered if the pool is unknown. Called by the
 * traceFIXED_POOL_* hooks; any context, including ISRs. */
void Stats_PoolEvent( const struct xFIXED_POOL * pxPool, eStatPoolEvent_t eEvent );

/* Fills *pxOut. Task context, scheduler running. Counters and gauges are
 * copied together in one short critical section, so the docs 18.2
 * equations hold inside one snapshot; each histogram is consistent on its
 * own; resource levels are read afterwards with interrupts enabled. */
void Stats_Snapshot( StatsSnapshot_t * pxOut );

#endif /* APP_STATS_H */

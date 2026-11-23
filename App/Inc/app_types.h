/**
  ******************************************************************************
  * @file    app_types.h
  * @brief   Types shared by several App/ modules: the generic event
  *          {eEventType, pvData}, the application descriptor, the dispatcher
  *          loop description, the StreamBuffer record header, the IMU sample
  *          and the UART TX work item. Design reference: docs 5.1, 6, 21.3.
  ******************************************************************************
  */
#ifndef APP_TYPES_H
#define APP_TYPES_H

#include <stddef.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "list.h"
#include "queue.h"
#include "task.h"           /* TimeOut_t (AppTimer_t) */

#include "app_config.h"
#include "app_stats.h"      /* eStatId_t, eGaugeId_t (AppEvtLoop_t); stdint only */

/*----------------------------------------------------------------------------
 * Generic event, the same shape as the IP task's IPStackEvent_t
 * (FreeRTOS_IP_Private.h:234-238). The receiver looks at eEventType first and
 * only then decides how to read pvData. pvData is either a descriptor pointer
 * or NULL; integers are never smuggled inside it.
 *--------------------------------------------------------------------------*/
typedef enum eAPP_EVENT
{
    eAppNoEvent = -1,       /* queue timeout: only the timers run          */
    eAppCmdEvent = 0,       /* pvData = AppDesc_t * (command)   -> ctrlTask */
    eAppNetUpEvent,         /* pvData = NULL                    -> ctrlTask */
    eAppNetDownEvent,       /* pvData = NULL                    -> ctrlTask */
    eAppEnvEvent,           /* pvData = AppDesc_t * (ENV)       -> worker   */
    eAppBulkEvent,          /* pvData = AppDesc_t * (BULK)      -> worker   */
    eAppCalEvent,           /* pvData = AppDesc_t * (CAL)       -> worker   */
    eAppEventCount
} eAppEvent_t;

typedef struct xAPP_EVENT
{
    eAppEvent_t eEventType;
    void * pvData;
} AppEvent_t;

/* What a handler tells the dispatcher about the descriptor it was given.
 * Mirrors eReleaseBuffer / eFrameConsumed of the IP task. */
typedef enum eAPP_DESC_ACTION
{
    eAppDescRelease = 0,    /* dispatcher returns it to its pool           */
    eAppDescKept            /* handler keeps it and releases it later      */
} eAppDescAction_t;

/* The two lanes of the same pipeline (docs 17.1). */
typedef enum eAPP_CLASS
{
    eAppClassCrit = 0,      /* port 5000, parser above the IP task         */
    eAppClassLow,           /* port 5001, parser at the bottom             */
    eAppClassCount
} eAppClass_t;

/* Event queues a parser or a hook can post to. */
typedef enum eAPP_TARGET
{
    eAppTargetCtrl = 0,     /* ctrlQueue -> ctrlTask                       */
    eAppTargetWork,         /* workQueue -> worker                         */
    eAppTargetCount
} eAppTarget_t;

/*----------------------------------------------------------------------------
 * Application descriptor: a small header that describes one validated packet.
 * The payload follows the header (flexible array member); its capacity is
 * fixed by the pool the block came from (APP_CRIT_/APP_LOW_DESC_DATA_BYTES).
 *
 * xPoolListItem MUST be the first member (fixed_pool.c, docs 7.1): it links
 * the block into its pool's free list, like xBufferListItem does for a
 * network buffer. "Is it free?" = "is the item in the free list?"; no
 * separate state or index field is kept.
 *--------------------------------------------------------------------------*/
typedef struct xAPP_DESC
{
    ListItem_t xPoolListItem; /* fixed_pool free list; only the pool uses it */
    uint8_t ucClass;        /* eAppClass_t                                 */
    uint8_t ucType;         /* packet TYPE                                 */
    uint16_t usSeq;         /* packet SEQ                                  */
    uint16_t usReqId;       /* packet REQ_ID (0 for data packets)          */
    uint32_t ulSrcIp;       /* network byte order                          */
    uint16_t usSrcPort;     /* network byte order                          */
    uint16_t usLen;         /* the packet's LEN. ucData holds that many
                             * bytes, or the pool's capacity if LEN is
                             * larger (only a command with a wrong LEN,
                             * which cmd.c answers with E_LEN; report 085) */
    uint32_t ulRxCycles;    /* M1: DWT cycle count at callback arrival     */
    uint8_t ucData[];       /* the payload (CRC8 checked)                  */
} AppDesc_t;

_Static_assert( offsetof( AppDesc_t, xPoolListItem ) == 0U, "pool list item must be the first member" );

/*----------------------------------------------------------------------------
 * Dispatcher loop: one copy of the loop code (AppEvt_RunLoop), two users
 * (ctrlTask, worker) that differ only in queue, handler table and timers.
 *--------------------------------------------------------------------------*/
typedef eAppDescAction_t ( * AppEvtHandler_t )( void * pvData );

/* A loop timer: the IP task's IPTimer_t (FreeRTOS_IP.h:211-219) plus the
 * function it runs (report 056 E1). Started with AppTimer_Start; checked
 * and restarted by the loop with xTaskCheckForTimeOut, which also handles
 * the tick count overflow. Only the loop's own task touches it. */
typedef struct xAPP_TIMER
{
    uint32_t
        bActive : 1,                /* running, must be checked            */
        bExpired : 1;               /* period over, pfRun is due           */
    TimeOut_t xTimeOut;             /* tick (and overflow count) at start  */
    TickType_t ulRemainingTime;     /* ticks left, updated at each check   */
    TickType_t ulReloadTime;        /* the period                          */
    void ( * pfRun )( void );       /* executed in the loop's task         */
} AppTimer_t;

typedef struct xAPP_EVT_LOOP
{
    QueueHandle_t xQueue;               /* the task's own event queue      */
    const AppEvtHandler_t * pxHandlers; /* eAppEventCount entries, NULL = unhandled */
    AppTimer_t * pxTimers;              /* NULL when uxTimerCount is 0     */
    UBaseType_t uxTimerCount;
    TickType_t xMaxSleep;               /* upper bound of one queue wait (ipconfigMAX_IP_TASK_SLEEP_TIME) */
    eStatId_t eStatHandled;             /* counted for each event given to a handler (E5) */
    eGaugeId_t eGaugeMinFree;           /* lowest free space after a receive (E2, E5)     */
} AppEvtLoop_t;

/*----------------------------------------------------------------------------
 * Record header written in front of every UDP payload in sbCrit / sbLow
 * (docs 5.1). The MessageBuffer keeps the record boundaries and its length
 * (report 062, B6): the payload length is the message length minus this
 * header. Natural alignment, no padding: 12 bytes.
 *--------------------------------------------------------------------------*/
typedef struct xUDP_REC_HDR
{
    uint32_t ulSrcIp;       /* network byte order                          */
    uint16_t usSrcPort;     /* network byte order                          */
    uint16_t usDstPort;     /* host byte order                             */
    uint32_t ulRxCycles;    /* DWT cycle count at callback arrival (M1)    */
} UdpRecHdr_t;

_Static_assert( sizeof( UdpRecHdr_t ) == APP_REC_HEADER_BYTES, "record header must stay 12 bytes" );

/*----------------------------------------------------------------------------
 * One IMU sample (6 axes + temperature, raw LSB, calibration applied later)
 *--------------------------------------------------------------------------*/
typedef struct xSAMPLE
{
    uint32_t ulTick;        /* xTaskGetTickCount() at DRDY                 */
    uint32_t ulDrdyCycles;  /* DWT cycle count at DRDY: TELEM latency start */
    int16_t sAccel[ 3 ];    /* x, y, z                                     */
    int16_t sGyro[ 3 ];     /* x, y, z                                     */
    int16_t sTemp;
} Sample_t;

/*----------------------------------------------------------------------------
 * IMU offsets from a CAL packet (docs 5.2): worker -> xCalMailbox -> meas
 *--------------------------------------------------------------------------*/
typedef struct xIMU_CAL
{
    int16_t sOffset[ 6 ];   /* ax, ay, az, gx, gy, gz (raw LSB)            */
} ImuCal_t;

_Static_assert( sizeof( ImuCal_t ) == APP_LEN_CAL, "a CAL payload is one ImuCal_t" );

/*----------------------------------------------------------------------------
 * UART TX work item: the same {type, void *} pattern as AppEvent_t
 *--------------------------------------------------------------------------*/
typedef enum eUART_TX_TYPE
{
    eUartTxFrame = 0,       /* pvData = NetworkBufferDescriptor_t *        */
    eUartTxReport           /* pvData = ReportDesc_t *                     */
} eUartTxType_t;

/* TX class (docs 10.1, 18.2 L7; report 062, B2). The reserve needs one fact
 * only: is it a command reply? Only eUartTxClassCtrl may use the last
 * APP_TXQ_RESERVE queue slots. The order is the order of eLatTxqCtrl /
 * eLatTxqOther (uart_tx.c checks it). */
typedef enum eUART_TX_CLASS
{
    eUartTxClassCtrl = 0,   /* IPv4 UDP from port 5000: command replies     */
    eUartTxClassOther,      /* telemetry, ARP, ICMP, reports, anything else */
    eUartTxClassCount
} eUartTxClass_t;

/* {type, void *} plus two measuring fields (decision U1, report 045).
 * 12 B on the target (arm-none-eabi enums are one byte). */
typedef struct xUART_TX_ITEM
{
    eUartTxType_t eType;
    eUartTxClass_t eClass;
    void * pvData;
    uint32_t ulPostCycles;  /* M4: DWT cycle count when it entered the queue */
} UartTxItem_t;

/* One JSON line waiting for the TX worker (from the report pool). */
typedef struct xREPORT_DESC
{
    ListItem_t xPoolListItem;                   /* first member: fixed_pool  */
    uint16_t usLen;                             /* characters in cText, no '\n' */
    char cText[ APP_REPORT_MAX_CHARS ];
} ReportDesc_t;

_Static_assert( offsetof( ReportDesc_t, xPoolListItem ) == 0U, "pool list item must be the first member" );

#endif /* APP_TYPES_H */

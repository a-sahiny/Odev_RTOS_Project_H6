/**
  ******************************************************************************
  * @file    app_config.h
  * @brief   Every limit, priority, stack size, timeout, port and protocol
  *          constant of the telemetry station. Nothing else in App/ may use a
  *          bare number for these. Design reference: docs/01_TASARIM_TASLAGI.md
  ******************************************************************************
  * Units are in the name: _MS milliseconds, _US microseconds, _HZ hertz,
  * _BYTES / _SIZE bytes, _WORDS stack words (4 bytes), _LEN queue items.
  * Waits are converted to ticks only where they are used: APP_MS(x).
  */
#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include "FreeRTOS.h"

#define APP_MS( ms )                    pdMS_TO_TICKS( ms )

/*----------------------------------------------------------------------------
 * Firmware identity (HELLO reply)
 *--------------------------------------------------------------------------*/
#define APP_FW_VERSION_STR              "0.10"  /* 0.4: TELEM per IMU sample, SET_RATE = IMU rate, 1 Mbaud; 0.5: 216 MHz; 0.6: imu_rate report; 0.8: SPI2 SCK idle fix; 0.9: first DRDY after a rate change; 0.10: EPOCH soğuk açılışta RNG'den */
#define APP_BOARD_STR                   "STM32F746G-DISCO"

/*----------------------------------------------------------------------------
 * Telemetri hedefi: derleme ayarı.
 * PC    : TELEM yalnız USART1'den, START ile (önceki davranış).
 * MP257 : ham TELEM verisi açılıştan itibaren USART6'dan; PC'den START gelirse
 *         USART1'den de önceki gibi gider.
 *--------------------------------------------------------------------------*/
#define APP_TELEM_TARGET_PC             0
#define APP_TELEM_TARGET_MP257          1
#ifndef APP_TELEM_TARGET                /* -DAPP_TELEM_TARGET=1 ile de seçilir */
#define APP_TELEM_TARGET                APP_TELEM_TARGET_PC
#endif

#define APP_RAW_BAUD                    460800U /* USART6: 23 bayt = 0,5 ms  */

/*----------------------------------------------------------------------------
 * Task priorities (docs 4). IP task = ipconfigIP_TASK_PRIORITY = 5.
 * The critical lane runs ABOVE the IP task on purpose (docs 4.1, D16).
 *--------------------------------------------------------------------------*/
#define APP_PRIO_CRIT_PARSER            ( configMAX_PRIORITIES - 1 )    /* 6 */
#define APP_PRIO_CTRL                   ( configMAX_PRIORITIES - 1 )    /* 6 */
#define APP_PRIO_VNET_RX                4
#define APP_PRIO_UART_TX                5       /* TELEM line out at once (DRDY -> TX done) */
#define APP_PRIO_MEAS                   3
#define APP_PRIO_LOW_PARSER             1
#define APP_PRIO_WORKER                 1

/*----------------------------------------------------------------------------
 * Task stacks, in words (checked later with high-water marks)
 *--------------------------------------------------------------------------*/
#define APP_STACK_CRIT_PARSER_WORDS     256
#define APP_STACK_CTRL_WORDS            512
#define APP_STACK_VNET_RX_WORDS         384
#define APP_STACK_UART_TX_WORDS         320
#define APP_STACK_MEAS_WORDS            384
#define APP_STACK_LOW_PARSER_WORDS      256
#define APP_STACK_WORKER_WORDS          384

/*----------------------------------------------------------------------------
 * Virtual network end-point (docs 2)
 *--------------------------------------------------------------------------*/
/* The address once as four numbers; the array for net_app.c and the text
 * for the hello report are both made from them, so they cannot differ
 * (report 062, B13). No U suffix: the text must read "192", not "192U". */
#define APP_NET_IP_0                    192
#define APP_NET_IP_1                    168
#define APP_NET_IP_2                    50
#define APP_NET_IP_3                    2
#define APP_NET_IP_ADDR                 { APP_NET_IP_0, APP_NET_IP_1, APP_NET_IP_2, APP_NET_IP_3 }
#define APP_STR_( x )                   #x
#define APP_STR( x )                    APP_STR_( x )
#define APP_NET_IP_STR                  APP_STR( APP_NET_IP_0 ) "." APP_STR( APP_NET_IP_1 ) "." \
                                        APP_STR( APP_NET_IP_2 ) "." APP_STR( APP_NET_IP_3 )
#define APP_NET_NETMASK                 { 255U, 255U, 255U, 0U }
#define APP_NET_GATEWAY                 { 192U, 168U, 50U, 1U }
#define APP_NET_DNS                     { 0U, 0U, 0U, 0U }

#define APP_UDP_PORT_CRIT               5000U   /* commands (important data) */
#define APP_UDP_PORT_LOW                5001U   /* ENV / BULK / CAL          */
#define APP_UDP_PORT_TELEM              5002U   /* telemetry source port     */
/* The hello report's "ports" value as fixed text (B13); report.c checks it
 * against the three numbers above. */
#define APP_UDP_PORTS_JSON              "[5000,5001,5002]"

/*----------------------------------------------------------------------------
 * UART1 link to the host (docs 13). 10 bits per character on the wire.
 *--------------------------------------------------------------------------*/
/* 1 Mbaud: the TELEM line (167 characters) leaves in 1.67 ms, under the 2 ms
 * limit; at 625000 even an empty UDP frame line (137 characters) took
 * 2.19 ms. USART1 runs on PCLK2 = 108 MHz (SYSCLK 216 MHz, main.c):
 * 1 Mbaud is exact with 16x oversampling (USARTDIV = 108). */
#define APP_UART_BAUD                   1000000U
#define APP_UART_CHAR_US                10U     /* 10 / 1000000 s = 10 us    */
#define APP_UART_RX_DMA_BYTES           256U    /* circular, ~2.6 ms of data */
#define APP_UART_RX_SB_BYTES            2048U   /* ISR -> vnetRx             */
#define APP_UART_RX_SB_TRIGGER          1U
#define APP_UART_RX_WAIT_MS             20U     /* vnetRx read timeout       */
#define APP_UART_TX_DMA_BYTES           544U    /* one longest line + margin */
#define APP_UART_TX_MARGIN_MS           20U     /* added to the line time    */

/*----------------------------------------------------------------------------
 * Host line parser (docs 14)
 *--------------------------------------------------------------------------*/
#define APP_LINE_MAX_CHARS              540U
#define APP_LINE_IDLE_TIMEOUT_MS        200U
#define APP_FRAME_MIN_BYTES             60U     /* without FCS               */
#define APP_FRAME_MAX_BYTES             256U    /* without FCS               */
#define APP_FRAME_FCS_BYTES             4U
#define APP_HOLD_MAX_MS                 10000U
#define APP_TXDELAY_MAX_MS              1000U

/*----------------------------------------------------------------------------
 * Virtual driver waits (docs 3.1, 7)
 *--------------------------------------------------------------------------*/
#define APP_VNET_NETBUF_WAIT_MS         2U      /* network buffer pool       */
#define APP_VNET_IPQ_WAIT_MS            2U      /* IP task event queue       */

/*----------------------------------------------------------------------------
 * UART TX queue and worker (docs 10)
 *--------------------------------------------------------------------------*/
#define APP_TXQ_LEN                     8U      /* report 086: 6 -> 8        */
#define APP_TXQ_RESERVE                 2U      /* last slots: control only  */
#define APP_TXQ_RECV_WAIT_MS            1000U

/*----------------------------------------------------------------------------
 * Application packet (docs 15)
 *--------------------------------------------------------------------------*/
#define APP_PKT_SOF                     0xA5U
#define APP_PKT_VERSION                 0x01U
#define APP_PKT_HEADER_BYTES            9U      /* SOF..LEN                  */
#define APP_PKT_OVERHEAD_BYTES          10U     /* header + CRC8             */
#define APP_PKT_MAX_PAYLOAD             200U
#define APP_PKT_MAX_BYTES               ( APP_PKT_OVERHEAD_BYTES + APP_PKT_MAX_PAYLOAD )

#define APP_PKT_FLAG_RESP               0x01U
#define APP_PKT_FLAG_ERR                0x02U
#define APP_PKT_FLAG_KNOWN_MASK         ( APP_PKT_FLAG_RESP | APP_PKT_FLAG_ERR )

#define APP_TYPE_CMD_START              0x01U
#define APP_TYPE_CMD_STOP               0x02U
#define APP_TYPE_CMD_SET_RATE           0x03U
#define APP_TYPE_CMD_GET_STATS          0x04U
/* 0x05 was GET_IMU: TELEM carries the attitude at every IMU sample now. */
#define APP_TYPE_ENV                    0x20U
#define APP_TYPE_BULK                   0x21U
#define APP_TYPE_CAL                    0x22U
#define APP_TYPE_TELEM                  0x30U

#define APP_ST_OK                       0x00U
#define APP_ST_E_LEN                    0x01U
#define APP_ST_E_RANGE                  0x02U
#define APP_ST_E_STATE                  0x03U
#define APP_ST_E_CONFLICT               0x04U
#define APP_ST_E_TYPE                   0x05U
#define APP_ST_E_REQID                  0x06U

#define APP_RESP_COMMON_BYTES           6U      /* STATUS EPOCH(2) STATE LAT_US(2) */
#define APP_STATE_BIT_NET_READY         0x01U   /* protocol copy of the bits */
#define APP_STATE_BIT_MEAS_EN           0x02U
#define APP_STATE_BIT_FAULT             0x04U

/* Payload sizes and ranges per type */
#define APP_LEN_SET_RATE                2U
#define APP_LEN_GET_STATS               1U
/* SET_RATE sets the IMU sample rate (SMPLRT_DIV and DLPF, imu_spi.c); every
 * sample is one TELEM. Only these rates: each divides the IMU's 1 kHz. */
#define APP_RATE_LIST_HZ                { 10U, 20U, 50U, 100U, 200U }
#define APP_RATE_COUNT                  5U
#define APP_RATE_MIN_HZ                 10U
#define APP_RATE_MAX_HZ                 200U
#define APP_RATE_DEFAULT_HZ             100U
#define APP_STATS_PAGES                 4U
#define APP_ENV_MAX_CHANNELS            8U
#define APP_BULK_SEED_BYTES             4U
#define APP_BULK_MIN_PATTERN            1U
#define APP_BULK_MAX_PATTERN            196U
#define APP_LEN_CAL                     12U     /* 6 x int16                 */
#define APP_CAL_MAX_ABS                 4096    /* raw LSB, either sign      */

/* TELEM (0x30), one per IMU sample, big-endian: a xyz i16, g xyz i16,
 * roll pitch yaw i16 (0.01 deg), flags u8, prev_seq u16, prev_us u16.
 * prev_*: the SEQ of the TELEM before this one and its DRDY -> TX done time
 * (the last bit of its line left the UART), measured on the board; valid
 * when APP_TELEM_FLAG_PREV is set. The sample order is the header SEQ. */
#define APP_LEN_TELEM                   23U
#define APP_TELEM_FLAGS_OFFSET          18U
#define APP_TELEM_PREV_OFFSET           19U     /* prev_seq, then prev_us    */
#define APP_TELEM_TARGET_MS             10U     /* DRDY -> TX done goal      */
#define APP_TELEM_DEADLINE_MS           12U     /* DRDY -> TX done deadline  */
/* meas -> uartTx: the DRDY time of each TELEM in flight, by SEQ (power of 2,
 * at least the TX queue length). */
#define APP_TELEM_DRDY_SLOTS            8U

/*----------------------------------------------------------------------------
 * UDP input: callback -> MessageBuffer -> parser (docs 5.1). One message =
 * one record = 12 B header + the UDP payload; the kernel stores its length
 * in front of it (APP_MB_LENGTH_BYTES, stream_buffer.c:222).
 *--------------------------------------------------------------------------*/
#define APP_REC_HEADER_BYTES            12U
#define APP_REC_MAX_BYTES               ( APP_REC_HEADER_BYTES + APP_PKT_MAX_BYTES )
#define APP_MB_LENGTH_BYTES             sizeof( configMESSAGE_BUFFER_LENGTH_TYPE )
#define APP_SB_CRIT_BYTES               512U
#define APP_SB_LOW_BYTES                2048U
#define APP_PARSER_WAIT_MS              1000U   /* liveness loop             */
#define APP_UDP_MAX_RX_PACKETS          1U      /* socket list safety net    */

/*----------------------------------------------------------------------------
 * Event queues, descriptor pools, dispatcher loops (docs 5, 6)
 *--------------------------------------------------------------------------*/
#define APP_CTRLQ_LEN                   6U
#define APP_WORKQ_LEN                   8U
#define APP_CRIT_DESC_COUNT             4U
#define APP_CRIT_DESC_DATA_BYTES        4U      /* largest command payload: 2 */
#define APP_LOW_DESC_COUNT              8U
#define APP_LOW_DESC_DATA_BYTES         APP_PKT_MAX_PAYLOAD
/* Blocks an ISR may never take (Pool_GetFromISR takes only while more than
 * this many are free; BufferAllocation_1.c uses a single constant of 3). */
#define APP_CRIT_DESC_ISR_RESERVE       1U
#define APP_LOW_DESC_ISR_RESERVE        3U
#define APP_CTRL_MAX_SLEEP_MS           100U
#define APP_WORK_MAX_SLEEP_MS           1000U
#define APP_HEALTH_PERIOD_MS            100U
#define APP_STATS_REPORT_PERIOD_MS      1000U

/*----------------------------------------------------------------------------
 * JSON reports (docs 13)
 *--------------------------------------------------------------------------*/
#define APP_REPORT_COUNT                2U
#define APP_REPORT_MAX_CHARS            320U
#define APP_REPORT_ISR_RESERVE          1U

/*----------------------------------------------------------------------------
 * Commands, duplicate window, sequence numbers (docs 16)
 *--------------------------------------------------------------------------*/
#define APP_DEDUP_SLOTS                 8U
#define APP_DEDUP_WINDOW_MS             10000U
#define APP_DEDUP_PAYLOAD_BYTES         APP_CRIT_DESC_DATA_BYTES
#define APP_SEQ_HALF_RANGE              32768U
#define APP_SEQ_RESYNC_COUNT            3U
#define APP_CTRL_SEND_TIMEOUT_MS        5U      /* FREERTOS_SO_SNDTIMEO      */
#define APP_TELEM_SEND_TIMEOUT_MS       2U

/*----------------------------------------------------------------------------
 * Event group (docs 12). Bits are states, never counters.
 *--------------------------------------------------------------------------*/
#define APP_BIT_NET_READY               ( ( EventBits_t ) 0x01U )
#define APP_BIT_MEAS_EN                 ( ( EventBits_t ) 0x02U )
#define APP_BIT_FAULT                   ( ( EventBits_t ) 0x04U )
#define APP_NET_READY_WAIT_MS           1000U
#define APP_MEAS_WAIT_MS                100U

/*----------------------------------------------------------------------------
 * Measurement and IMU (docs 3.3). Module answers WHO_AM_I 0x74.
 * The SPI prescaler names come from the HAL; they expand where used.
 *--------------------------------------------------------------------------*/
#define APP_MEAS_SOURCE_IMU             0
#define APP_MEAS_SOURCE_SYNTH           1
#ifndef APP_MEAS_SOURCE                 /* -DAPP_MEAS_SOURCE=1: no IMU needed */
#define APP_MEAS_SOURCE                 APP_MEAS_SOURCE_IMU
#endif

/* The IMU rate is the SET_RATE rate (APP_RATE_*). A DRDY is waited for one
 * sample period + this margin: 20 ms at 100 Hz, 110 ms at 10 Hz. */
#define APP_IMU_DRDY_MARGIN_MS          10U
#define APP_IMU_DRDY_TIMEOUT_MAX_MS     ( ( 1000U / APP_RATE_MIN_HZ ) + APP_IMU_DRDY_MARGIN_MS )
#define APP_IMU_SPI_TIMEOUT_MS          5U
#define APP_IMU_FAIL_LIMIT              3U      /* consecutive -> FAULT IMU  */
/* SPI2 is on APB1 = 54 MHz (SYSCLK 216 MHz / 4). */
#define APP_IMU_SPI_PRESC_CONFIG        SPI_BAUDRATEPRESCALER_64  /* 844 kHz <= 1 MHz */
#define APP_IMU_SPI_PRESC_DATA          SPI_BAUDRATEPRESCALER_16  /* 3.375 MHz <= 20 MHz */

/* Scales of the imu_spi.c set-up: +-4 g, +-1000 dps (report 087). */
#define APP_IMU_ACCEL_LSB_PER_G         8192U
#define APP_IMU_GYRO_LSB_PER_DPS_X10    328U    /* 32.8 LSB per deg/s        */

/*----------------------------------------------------------------------------
 * Attitude (report 086/087): complementary filter in meas, every sample.
 *--------------------------------------------------------------------------*/
#define APP_ATT_BIAS_MS                 1000U   /* gyro bias: first 1 s after START */
#define APP_ATT_FLAG_BIAS_READY         0x01U   /* gyro bias measured        */
#define APP_ATT_FLAG_ACC_USED           0x02U   /* accel corrected this one  */
#define APP_TELEM_FLAG_PREV             0x04U   /* prev_seq / prev_us valid  */

/*----------------------------------------------------------------------------
 * FAULT conditions (docs 19). Reason bits live outside the event group.
 *--------------------------------------------------------------------------*/
#define APP_FAULT_IMU                   0x01U
#define APP_FAULT_POOL                  0x02U
#define APP_FAULT_UART_ORE              0x04U
#define APP_FAULT_UART_TX_STALL         0x08U
#define APP_FAULT_HEAP                  0x10U
/* 0x20 was DESYNC: with MessageBuffers the record stream cannot slip
 * (report 062, B6), so there is nothing left to detect. */

#define APP_FAULT_POOL_MIN_FREE         2U
#define APP_FAULT_POOL_CHECKS           2U      /* consecutive health checks */
#define APP_FAULT_ORE_PER_S             3U
#define APP_FAULT_HEAP_MIN_BYTES        2048U

/*----------------------------------------------------------------------------
 * Halt line (docs 13.1): one line through UartLink_HaltWrite (DMA), then stop.
 * Not FAULT above: FAULT is a state the system recovers from, a halt is not.
 *--------------------------------------------------------------------------*/
#define APP_HALT_TEXT_MAX               24U     /* "why" / "at" characters   */
#define APP_HALT_LINE_BYTES             96U     /* static line buffer        */

/*----------------------------------------------------------------------------
 * Consistency checks: a wrong edit above fails the build, not the board.
 *--------------------------------------------------------------------------*/
_Static_assert( APP_PRIO_CRIT_PARSER < configMAX_PRIORITIES, "priority out of range" );
_Static_assert( APP_PRIO_LOW_PARSER < APP_PRIO_VNET_RX, "low lane must stay below the link tasks" );
_Static_assert( APP_TXQ_RESERVE < APP_TXQ_LEN, "TX reserve must leave room for bulk frames" );
_Static_assert( APP_LINE_MAX_CHARS >= ( 8U + 2U * ( APP_FRAME_MAX_BYTES + APP_FRAME_FCS_BYTES ) ),
                "longest FRAME 1 line must fit the line buffer" );
_Static_assert( APP_UART_TX_DMA_BYTES >= ( 9U + 2U * ( APP_FRAME_MAX_BYTES + APP_FRAME_FCS_BYTES ) ),
                "longest TX line (+ newline) must fit the DMA buffer" );
_Static_assert( APP_UART_TX_DMA_BYTES >= ( APP_REPORT_MAX_CHARS + 1U ), "JSON line must fit the DMA buffer" );
_Static_assert( APP_SB_CRIT_BYTES >= ( 2U * ( APP_MB_LENGTH_BYTES + APP_REC_MAX_BYTES ) ), "sbCrit must hold two largest records" );
_Static_assert( APP_SB_LOW_BYTES >= ( 2U * ( APP_MB_LENGTH_BYTES + APP_REC_MAX_BYTES ) ), "sbLow must hold two largest records" );
_Static_assert( APP_CRIT_DESC_DATA_BYTES >= APP_LEN_SET_RATE, "command payload must fit a crit descriptor" );
_Static_assert( ( APP_BULK_SEED_BYTES + APP_BULK_MAX_PATTERN ) <= APP_LOW_DESC_DATA_BYTES, "BULK must fit a low descriptor" );
_Static_assert( APP_TELEM_DRDY_SLOTS >= APP_TXQ_LEN, "every TELEM the TX queue can hold has a DRDY slot" );
_Static_assert( ( APP_TELEM_DRDY_SLOTS & ( APP_TELEM_DRDY_SLOTS - 1U ) ) == 0U, "DRDY slots: a power of 2" );
_Static_assert( APP_CRIT_DESC_ISR_RESERVE < APP_CRIT_DESC_COUNT, "ISR reserve must leave blocks for an ISR" );
_Static_assert( APP_LOW_DESC_ISR_RESERVE < APP_LOW_DESC_COUNT, "ISR reserve must leave blocks for an ISR" );
_Static_assert( APP_REPORT_ISR_RESERVE < APP_REPORT_COUNT, "ISR reserve must leave blocks for an ISR" );

#endif /* APP_CONFIG_H */

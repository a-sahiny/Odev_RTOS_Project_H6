/**
  ******************************************************************************
  * @file    app_main.c
  * @brief   Application start-up in one place (docs 21.4) and the halt line
  *          (docs 13.1). Künye: report 082.
  ******************************************************************************
  * RTOS objects owned: none. Every object is created by the Init calls in
  * App_Init, before the scheduler (report 082 section 3 lists them).
  * Not RTOS objects: the halt line buffer, written only with interrupts off.
  */
#include "app_main.h"

#include <stddef.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

#include "app_config.h"
#include "app_evt.h"
#include "app_stats.h"
#include "ctrl_task.h"
#include "meas_task.h"
#include "net_app.h"
#include "pkt_parser.h"
#include "report.h"
#include "sys_state.h"
#include "uart_link.h"
#include "uart_tx.h"
#include "udp_in.h"
#include "vnet_if.h"
#include "worker_task.h"

static void prvCheck( BaseType_t xResult, const char * pcStep );
static size_t prvPutRaw( size_t uxPos, const char * pcText );
static size_t prvPutText( size_t uxPos, const char * pcText );

/* The literal parts of the longest line plus two full texts and the NUL. */
#define APP_HALT_FIXED_CHARS    ( sizeof( "{\"type\":\"halt\",\"why\":\"" ) - 1U \
                                  + sizeof( "\",\"at\":\"" ) - 1U                 \
                                  + sizeof( "\"}\n" ) - 1U )

_Static_assert( APP_HALT_FIXED_CHARS + ( 2U * APP_HALT_TEXT_MAX ) + 1U <= APP_HALT_LINE_BYTES, "the longest halt line fits the buffer" );

/* Static, not on the stack: in the stack overflow hook the stack is the one
 * thing not to trust. Written only with interrupts off. */
static char cHaltLine[ APP_HALT_LINE_BYTES ];

/*---------------------------------------------------------------------------*/
/* App_Init: report 082 A1 order. Before the scheduler nothing runs, so the  */
/* order matters only where an Init looks at another module's handle.       */
/*---------------------------------------------------------------------------*/
void App_Init( void )
{
    /* First: the pool Inits register with Stats, the task Inits store their
     * handles there; a later memset would zero what they counted. */
    Stats_Init();
    SysState_Init();

    /* Event queues and descriptor pools before NetApp_Init: the network hook
     * posts NetUp to ctrlQueue (report 056 E9). */
    prvCheck( AppEvt_Init(), "AppEvt_Init" );
    prvCheck( Report_Init(), "Report_Init" );

    UartLink_Init();
    prvCheck( UartTx_Init(), "UartTx_Init" );

    /* The MessageBuffers before their readers (report 060 P8). */
    prvCheck( UdpIn_Init(), "UdpIn_Init" );
    prvCheck( Parser_Init(), "Parser_Init" );

    prvCheck( VNet_CreateRxTask(), "VNet_CreateRxTask" );

    /* Cmd_Init runs at the start of ctrlTask (report 065). WorkerTask_Init
     * before Meas_Init: it creates the CAL mailbox (docs 21.5). */
    prvCheck( CtrlTask_Init(), "CtrlTask_Init" );
    prvCheck( WorkerTask_Init(), "WorkerTask_Init" );
    prvCheck( Meas_Init(), "Meas_Init" );

    /* Last: the interface, the end-point and the IP task. */
    prvCheck( NetApp_Init(), "NetApp_Init" );
}

/*---------------------------------------------------------------------------*/
/* prvCheck: docs 21.2. A failed start-up step stops the board with its name */
/* on the UART (A3); the UART is ready since MX_USART1_UART_Init.            */
/*---------------------------------------------------------------------------*/
static void prvCheck( BaseType_t xResult, const char * pcStep )
{
    if( xResult != pdPASS )
    {
        App_Halt( "init", pcStep );
    }
}

/*---------------------------------------------------------------------------*/
void App_Halt( const char * pcWhy, const char * pcDetail )
{
    size_t uxPos;

    /* First: no task and no interrupt of priority 5..15 runs any more
     * (report 082 section 7). The DMA does not need one to send the line. */
    taskDISABLE_INTERRUPTS();

    uxPos = prvPutRaw( 0U, "{\"type\":\"halt\",\"why\":\"" );
    uxPos = prvPutText( uxPos, pcWhy );

    if( pcDetail != NULL )
    {
        uxPos = prvPutRaw( uxPos, "\",\"at\":\"" );
        uxPos = prvPutText( uxPos, pcDetail );
    }

    uxPos = prvPutRaw( uxPos, "\"}\n" );
    cHaltLine[ uxPos ] = '\0';

    /* Report 092: one DMA transfer, left running; the endless loop below does
     * not wait for it (a line half sent by uartTx is aborted first). */
    UartLink_HaltWrite( ( const uint8_t * ) cHaltLine, uxPos );

    for( ; ; )
    {
    }
}

/*---------------------------------------------------------------------------*/
/* prvPutRaw: fixed text of this file; the static assert above bounds it.    */
/*---------------------------------------------------------------------------*/
static size_t prvPutRaw( size_t uxPos, const char * pcText )
{
    while( *pcText != '\0' )
    {
        cHaltLine[ uxPos ] = *pcText;
        uxPos++;
        pcText++;
    }

    return uxPos;
}

/*---------------------------------------------------------------------------*/
/* prvPutText: caller text, possibly from a damaged TCB. At most             */
/* APP_HALT_TEXT_MAX characters; only [A-Za-z0-9_-] passes, anything else is */
/* '?', so the line stays valid JSON whatever the bytes are. NULL is "?".    */
/*---------------------------------------------------------------------------*/
static size_t prvPutText( size_t uxPos, const char * pcText )
{
    size_t uxCount = 0U;
    char cChar;

    if( pcText == NULL )
    {
        pcText = "?";
    }

    while( ( uxCount < APP_HALT_TEXT_MAX ) && ( pcText[ uxCount ] != '\0' ) )
    {
        cChar = pcText[ uxCount ];

        if( !( ( ( cChar >= 'a' ) && ( cChar <= 'z' ) ) ||
               ( ( cChar >= 'A' ) && ( cChar <= 'Z' ) ) ||
               ( ( cChar >= '0' ) && ( cChar <= '9' ) ) ||
               ( cChar == '_' ) || ( cChar == '-' ) ) )
        {
            cChar = '?';
        }

        cHaltLine[ uxPos ] = cChar;
        uxPos++;
        uxCount++;
    }

    return uxPos;
}

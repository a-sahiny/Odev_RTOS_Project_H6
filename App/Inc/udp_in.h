/**
  ******************************************************************************
  * @file    udp_in.h
  * @brief   Where received UDP data leaves the TCP/IP stack: the receive
  *          callbacks of ports 5000 / 5001 write one record per datagram into
  *          the MessageBuffers sbCrit / sbLow (docs D3, 5.1); the three UDP
  *          sockets.
  ******************************************************************************
  * Künye: report 052; MessageBuffer: report 062 (B6).
  */
#ifndef UDP_IN_H
#define UDP_IN_H

#include "FreeRTOS.h"
#include "message_buffer.h"

#include "app_types.h"      /* eAppClass_t */

/* = Socket_t (FreeRTOS_Sockets.h); declared by tag so this header does not
 * pull in the TCP/IP stack. */
struct xSOCKET;

/* App_Init, before the scheduler. Creates the two static MessageBuffers. A
 * second call does nothing. Always pdPASS: static objects cannot fail. */
BaseType_t UdpIn_Init( void );

/* ctrlTask, once, after SysState_WaitNetReady (FreeRTOS_bind waits for the
 * IP task without a time limit). Opens and binds the sockets 5000 (receive
 * callback, send timeout 5 ms), 5001 (receive callback) and 5002 (send only,
 * 2 ms). pdFAIL: none is left open; the caller decides what to do. After a
 * success, further calls do nothing and return pdPASS. */
BaseType_t UdpIn_OpenSockets( void );

/* The MessageBuffer of a lane, for its parser (the only reader). NULL before
 * UdpIn_Init; an unknown class asserts. */
MessageBufferHandle_t UdpIn_GetStream( eAppClass_t eClass );

/* The sending sockets: 5000 for ctrlTask's replies, 5002 for meas'
 * telemetry. NULL until UdpIn_OpenSockets has succeeded. */
struct xSOCKET * UdpIn_GetCtrlSocket( void );
struct xSOCKET * UdpIn_GetTelemSocket( void );

#endif /* UDP_IN_H */

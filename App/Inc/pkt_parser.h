/**
  ******************************************************************************
  * @file    pkt_parser.h
  * @brief   The two parser tasks of the shared pipeline (docs D3, 3.1, 17.1):
  *          critParser (P6) reads sbCrit, lowParser (P1) reads sbLow; one
  *          function, two configurations.
  ******************************************************************************
  * Künye: report 060.
  */
#ifndef PKT_PARSER_H
#define PKT_PARSER_H

#include "FreeRTOS.h"

/* App_Init, before the scheduler, after UdpIn_Init (the StreamBuffers) and
 * AppEvt_Init (pools and queues). Creates critParser and lowParser (static
 * TCB and stack) and records their handles for the stack high-water marks.
 * A second call does nothing. Always pdPASS: static objects cannot fail. */
BaseType_t Parser_Init( void );

#endif /* PKT_PARSER_H */

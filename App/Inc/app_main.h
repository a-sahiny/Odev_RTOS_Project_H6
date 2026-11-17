/**
  ******************************************************************************
  * @file    app_main.h
  * @brief   Application start-up (App_Init) and the halt line (App_Halt).
  *          Step 18, künye: report 082.
  ******************************************************************************
  */
#ifndef APP_MAIN_H
#define APP_MAIN_H

/* main.c USER CODE 2, once, before vTaskStartScheduler(). Creates every
 * application RTOS object and the IP task (report 082 section 3). A failed
 * step halts with {"type":"halt","why":"init","at":"<step>"} (A2, A3). */
void App_Init( void );

/* Stops the system for good: interrupts off (BASEPRI), one line
 *   {"type":"halt","why":"<pcWhy>"[,"at":"<pcDetail>"]}
 * through UartLink_HaltWrite (DMA, report 092), then an endless loop. Any context; never
 * returns. Both texts are filtered to [A-Za-z0-9_-] (others '?') and cut at
 * APP_HALT_TEXT_MAX characters; pcDetail NULL leaves "at" out (A4). */
void App_Halt( const char * pcWhy, const char * pcDetail );

#endif /* APP_MAIN_H */

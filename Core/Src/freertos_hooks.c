/**
  ******************************************************************************
  * @file    freertos_hooks.c
  * @brief   FreeRTOS application hooks enabled in FreeRTOSConfig.h.
  *          Both hooks stop the system through App_Halt: one halt line on
  *          UART1 (docs 13.1, report 082 A4), then an endless loop with
  *          interrupts off, so the fault can still be inspected with the
  *          debugger (pause and look at the call stack / pcTaskName).
  ******************************************************************************
  */
#include "FreeRTOS.h"
#include "task.h"

#include "app_main.h"

#if (configCHECK_FOR_STACK_OVERFLOW > 0)
/* Called when a task overflows its stack (configCHECK_FOR_STACK_OVERFLOW = 2),
 * from vTaskSwitchContext (tasks.c:5259): PendSV, main stack, BASEPRI
 * already 5. The overflowing stack is not used. pcTaskName may be damaged;
 * App_Halt filters it. Line: {"type":"halt","why":"stack","at":"<task>"}. */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
  (void)xTask;

  App_Halt("stack", pcTaskName);
}
#endif

#if (configUSE_MALLOC_FAILED_HOOK == 1)
/* Called when pvPortMalloc() fails (heap_4.c:340, the calling task, after
 * xTaskResumeAll): configTOTAL_HEAP_SIZE is too small for what is created at
 * run time (the UDP sockets). Line: {"type":"halt","why":"malloc"}. */
void vApplicationMallocFailedHook(void)
{
  App_Halt("malloc", NULL);
}
#endif

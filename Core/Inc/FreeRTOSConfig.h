/**
  ******************************************************************************
  * @file    FreeRTOSConfig.h
  * @brief   FreeRTOS Kernel V11.3.1 configuration for STM32F746G-DISCO
  *          (STM32F746NGH6, Cortex-M7 r0p1, GCC port GCC/ARM_CM7/r0p1, heap_4).
  ******************************************************************************
  * Tick         : 1 kHz -> 1 tick = 1 ms (SysTick, kernel owned).
  * HAL timebase : TIM1 (stm32f7xx_hal_timebase_tim.c), SysTick is free for
  *                the kernel.
  * Interrupts   : NVIC_PRIORITYGROUP_4, 4 priority bits (0 = highest, 15 =
  *                lowest). ISRs that call FreeRTOS "...FromISR" APIs must have
  *                a numeric priority >= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY
  *                (5..15). ISRs at 0..4 must not call any FreeRTOS API.
  ******************************************************************************
  */
#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/* SystemCoreClock is updated by HAL_RCC_ClockConfig() in SystemClock_Config().
 * Guarded so the header stays usable from assembler sources. */
#if !defined( __ASSEMBLER__ ) && !defined( __IAR_SYSTEMS_ASM__ )
    #include <stdint.h>
    extern uint32_t SystemCoreClock;
#endif

/*-----------------------------------------------------------------------------
 * Scheduler and tick
 *---------------------------------------------------------------------------*/
#define configUSE_PREEMPTION                        1
#define configUSE_TIME_SLICING                      1
#define configUSE_PORT_OPTIMISED_TASK_SELECTION     1   /* Cortex-M7 CLZ, max 32 priorities */
#define configUSE_TICKLESS_IDLE                     0
#define configCPU_CLOCK_HZ                          ( SystemCoreClock )
#define configTICK_RATE_HZ                          ( ( TickType_t ) 1000 )  /* 1 ms tick */
#define configTICK_TYPE_WIDTH_IN_BITS               TICK_TYPE_WIDTH_32_BITS
#define configMAX_PRIORITIES                        ( 7 )
#define configMINIMAL_STACK_SIZE                    ( ( uint16_t ) 128 )     /* words (512 bytes) */
#define configMAX_TASK_NAME_LEN                     ( 16 )
#define configIDLE_SHOULD_YIELD                     1
#define configTASK_NOTIFICATION_ARRAY_ENTRIES       1
#define configQUEUE_REGISTRY_SIZE                   8
#define configNUM_THREAD_LOCAL_STORAGE_POINTERS     0
#define configUSE_MINI_LIST_ITEM                    1
#define configSTACK_DEPTH_TYPE                      uint32_t
#define configMESSAGE_BUFFER_LENGTH_TYPE            size_t
#define configUSE_NEWLIB_REENTRANT                  0

/*-----------------------------------------------------------------------------
 * Memory allocation (portable/MemMang/heap_4.c)
 *---------------------------------------------------------------------------*/
#define configSUPPORT_DYNAMIC_ALLOCATION            1
#define configSUPPORT_STATIC_ALLOCATION             1
#define configKERNEL_PROVIDED_STATIC_MEMORY         1   /* kernel supplies Idle/Timer task memory */
#define configTOTAL_HEAP_SIZE                       ( ( size_t ) ( 64 * 1024 ) )  /* + FreeRTOS-Plus-TCP sockets */
#define configAPPLICATION_ALLOCATED_HEAP            0
#define configHEAP_CLEAR_MEMORY_ON_FREE             1

/*-----------------------------------------------------------------------------
 * Hooks (implemented in Core/Src/freertos_hooks.c)
 *---------------------------------------------------------------------------*/
#define configUSE_IDLE_HOOK                         0
#define configUSE_TICK_HOOK                         0
#define configUSE_MALLOC_FAILED_HOOK                1
#define configCHECK_FOR_STACK_OVERFLOW              2
#define configUSE_DAEMON_TASK_STARTUP_HOOK          0

/*-----------------------------------------------------------------------------
 * Run time / trace
 *---------------------------------------------------------------------------*/
#define configGENERATE_RUN_TIME_STATS               0
#define configUSE_TRACE_FACILITY                    1
#define configUSE_STATS_FORMATTING_FUNCTIONS        0

/*-----------------------------------------------------------------------------
 * Kernel objects
 *---------------------------------------------------------------------------*/
#define configUSE_TASK_NOTIFICATIONS                1
#define configUSE_MUTEXES                           1
#define configUSE_RECURSIVE_MUTEXES                 1
#define configUSE_COUNTING_SEMAPHORES               1
#define configUSE_QUEUE_SETS                        0
#define configUSE_EVENT_GROUPS                      1
#define configUSE_STREAM_BUFFERS                    1
#define configUSE_CO_ROUTINES                       0
#define configMAX_CO_ROUTINE_PRIORITIES             ( 2 )

/*-----------------------------------------------------------------------------
 * Software timers
 *---------------------------------------------------------------------------*/
#define configUSE_TIMERS                            1
#define configTIMER_TASK_PRIORITY                   ( configMAX_PRIORITIES - 1 )
#define configTIMER_QUEUE_LENGTH                    10
#define configTIMER_TASK_STACK_DEPTH                ( configMINIMAL_STACK_SIZE * 2 )

/*-----------------------------------------------------------------------------
 * Optional API functions
 *---------------------------------------------------------------------------*/
#define INCLUDE_vTaskPrioritySet                    1
#define INCLUDE_uxTaskPriorityGet                   1
#define INCLUDE_vTaskDelete                         1
#define INCLUDE_vTaskSuspend                        1
#define INCLUDE_xTaskResumeFromISR                  1
#define INCLUDE_xTaskDelayUntil                     1
#define INCLUDE_vTaskDelay                          1
#define INCLUDE_xTaskGetSchedulerState              1   /* used by SysTick_Handler */
#define INCLUDE_xTaskGetCurrentTaskHandle           1
#define INCLUDE_uxTaskGetStackHighWaterMark         1
#define INCLUDE_xTaskGetIdleTaskHandle              1
#define INCLUDE_eTaskGetState                       1
#define INCLUDE_xTimerPendFunctionCall              1
#define INCLUDE_xTaskAbortDelay                     0
#define INCLUDE_xTaskGetHandle                      0
#define INCLUDE_xSemaphoreGetMutexHolder            1

/*-----------------------------------------------------------------------------
 * Cortex-M interrupt priorities
 *---------------------------------------------------------------------------*/
#ifdef __NVIC_PRIO_BITS
    #define configPRIO_BITS                         __NVIC_PRIO_BITS
#else
    #define configPRIO_BITS                         4   /* STM32F7: 16 priority levels */
#endif

/* Lowest priority, used for the kernel's PendSV and SysTick. */
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY         15

/* Highest priority from which ...FromISR() APIs may be called. */
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY    5

#define configKERNEL_INTERRUPT_PRIORITY \
    ( configLIBRARY_LOWEST_INTERRUPT_PRIORITY << ( 8 - configPRIO_BITS ) )
#define configMAX_SYSCALL_INTERRUPT_PRIORITY \
    ( configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << ( 8 - configPRIO_BITS ) )

/*-----------------------------------------------------------------------------
 * Assert: stops here on misconfiguration (wrong ISR priority, NULL handle, ...)
 *---------------------------------------------------------------------------*/
#define configASSERT( x )             \
    do {                              \
        if( ( x ) == 0 )              \
        {                             \
            taskDISABLE_INTERRUPTS(); \
            for( ; ; ) {}             \
        }                             \
    } while( 0 )

/*-----------------------------------------------------------------------------
 * Exception handler mapping to the CMSIS vector names (startup_stm32f746nghx.s)
 * SVC and PendSV are naked handlers and are installed directly, so CubeMX must
 * not generate SVC_Handler / PendSV_Handler in stm32f7xx_it.c.
 * xPortSysTickHandler is NOT mapped: SysTick_Handler in stm32f7xx_it.c calls
 * it once the scheduler has started.
 *---------------------------------------------------------------------------*/
#define vPortSVCHandler     SVC_Handler
#define xPortPendSVHandler  PendSV_Handler

#endif /* FREERTOS_CONFIG_H */

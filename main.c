//*****************************************************************************
//
// Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/
//
//
//  Redistribution and use in source and binary forms, with or without
//  modification, are permitted provided that the following conditions
//  are met:
//
//    Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
//    Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the
//    distribution.
//
//    Neither the name of Texas Instruments Incorporated nor the names of
//    its contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
//  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
//  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
//  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
//  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//*****************************************************************************

// Standard includes
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

/* POSIX Header files */
#include <pthread.h>

/* RTOS header files */
#include "FreeRTOS.h"
#include "task.h"

// SimpleLink includes
#include <ti/drivers/net/wifi/simplelink.h>
/* TI-RTOS Header files */
#include <ti/drivers/GPIO.h>
/* Example/Board Header files */
#include "Board.h"

//
#include <unabto_platform.h>

// free-rtos/ ti_rtos includes

// Hardware & DriverLib library includes.
#include "hw_common_reg.h"
#include "hw_ints.h"
#include "hw_memmap.h"
#include "hw_types.h"
#include "interrupt.h"
#include "prcm.h"
#include "rom.h"
#include "rom_map.h"
#include "uart.h"
#include "utils.h"
#include "gpio.h"
#include "shamd5.h"
#include "aes.h"

// Common interface includes
#include "uart_term.h"


/* Stack size in bytes */
#define THREADSTACKSIZE     (2048)
//
#define TASK_STACK_SIZE     (1024)
//
#define SPAWN_TASK_PRIORITY     (9)

//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************
pthread_t  g_NetworkTask = (pthread_t)NULL;
pthread_t  g_UNabtoTask  = (pthread_t)NULL;
pthread_t  gSpawn_thread = (pthread_t)NULL;

#if defined(ccs)
extern void (*const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif
//*****************************************************************************
//                 GLOBAL VARIABLES -- End
//*****************************************************************************

//******************************************************************************
//                    FUNCTION DECLARATIONS
//******************************************************************************
extern void *Network(void *pvParameters);
extern void *UNabto(void *pvParameters);


//*****************************************************************************
//
//! Application defined hook (or callback) function - the tick hook.
//! The tick interrupt can optionally call this
//!
//! \param  none
//!
//! \return none
//!
//*****************************************************************************
void vApplicationTickHook(void) {
}

//*****************************************************************************
//
//! Application defined hook (or callback) function - assert
//!
//! \param  none
//!
//! \return none
//!
//*****************************************************************************
void vAssertCalled(const char *pcFile, unsigned long ulLine) {
    while(1)
    {

    }
}

//*****************************************************************************
//
//! Application defined idle task hook
//!
//! \param  none
//!
//! \return none
//!
//*****************************************************************************
void vApplicationIdleHook(void) {
}

//*****************************************************************************
//
//! Application provided stack overflow hook function.
//!
//! \param  handle of the offending task
//! \param  name  of the offending task
//!
//! \return none
//!
//*****************************************************************************
void vApplicationStackOverflowHook(TaskHandle_t pxTask,
                                   signed char *pcTaskName) {
    (void)pxTask;
    (void)pcTaskName;

    while(1)
    {
    }
}

void vApplicationMallocFailedHook() {
    /* Handle Memory Allocation Errors */
    while(1)
    {
    }
}

//*****************************************************************************
//
//! Application startup display on UART
//!
//! \param  none
//!
//! \return none
//!
//*****************************************************************************
static void DisplayBanner(void) {
    UART_PRINT("*************************************************\n\r");
    UART_PRINT("                   CC3220 + uNabto               \n\r");
    UART_PRINT("*************************************************\n\r");
    UART_PRINT("\n\r");
}

//******************************************************************************
//                            MAIN FUNCTION
//******************************************************************************
int main(void)
{
    int                 retc = -1;
    pthread_attr_t      pAttrs;
    struct sched_param  priParam;
    pthread_attr_t      pAttrs_spawn;
    struct sched_param  priParam_spawn;
    int                 detachState;

    /* Call board init functions */
    Board_initGeneral();

    /* Initializes the SPI interface to the Network Processor and peripheral SPI (if defined in the board file) */
    Board_initSPI();
    Board_initGPIO();

    InitTerm();

    DisplayBanner();

    /* Create the sl_Task internal spawn thread */
    pthread_attr_init(&pAttrs_spawn);
    priParam_spawn.sched_priority = SPAWN_TASK_PRIORITY;
    retc = pthread_attr_setschedparam(&pAttrs_spawn, &priParam_spawn);
    retc |= pthread_attr_setstacksize(&pAttrs_spawn, TASK_STACK_SIZE);

    /* The SimpleLink host driver architecture mandate spawn thread to be created prior to calling Sl_start (turning the NWP on). */
    /* The purpose of this thread is to handle asynchronous events sent from the NWP.
     * Every event is classified and later handled by the Host driver event handlers. */
    retc = pthread_create(&gSpawn_thread, &pAttrs_spawn, sl_Task, NULL);
    if(retc < 0)
    {
        /* Handle Error */
         UART_PRINT("Network Terminal - Unable to create spawn thread \n");
         return(NULL);
    }

    /* Set priority and stack size attributes */
    pthread_attr_init(&pAttrs);
    priParam.sched_priority = 1;

    detachState = PTHREAD_CREATE_DETACHED;
    retc = pthread_attr_setdetachstate(&pAttrs, detachState);
    if (retc != 0) {
        /* pthread_attr_setdetachstate() failed */
        while (1);
    }

    pthread_attr_setschedparam(&pAttrs, &priParam);

    retc |= pthread_attr_setstacksize(&pAttrs, THREADSTACKSIZE);
    if (retc != 0) {
        /* pthread_attr_setstacksize() failed */
        while (1);
    }

    //
    // Start the Network Task
    //
    retc = pthread_create(&g_NetworkTask, &pAttrs, Network, NULL);
    if (retc != 0) {
        /* pthread_create() failed */
        while (1);
    }

    //
    // Start the uNabto Task
    //
    retc = pthread_create(&g_UNabtoTask, &pAttrs, UNabto, NULL);
    if (retc != 0) {
        /* pthread_create() failed */
        while (1);
    }

    /* Start the FreeRTOS scheduler */
    vTaskStartScheduler();

    return (0);
}

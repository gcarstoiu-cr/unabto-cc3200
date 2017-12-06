//*****************************************************************************
// unabto_task.c
//
// uNabto Task
//
//*****************************************************************************

#include <ctype.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>

// free-rtos/ ti_rtos includes
#include "FreeRTOS.h"
#include "task.h"

// Hardware & DriverLib library includes.

#include "rom_map.h"

// Common interface includes


// Nabto Includes
#include <unabto/unabto_app.h>
#include <unabto/unabto_common_main.h>
#include <unabto_version.h>
#include "unabto_platform.h"
#include "stream_echo.h"

//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************

uint8_t g_ucLedState = 0;
extern unsigned long g_uiIpAddress;
extern volatile unsigned long g_ulStatus;

//*****************************************************************************
//                 GLOBAL VARIABLES -- End
//*****************************************************************************

//******************************************************************************
//                    FUNCTION DECLARATIONS
//******************************************************************************
extern void wait_event();

int hctoi(const unsigned char h) {
    if (isdigit(h)) {
        return h - '0';
    } else {
        return toupper(h) - 'A' + 10;
    }
}

//*****************************************************************************
//
//! uNabto Task
//!
//! \param  pvParameters - Parameters to the task's entry function
//!
//! \return None
//!
//*****************************************************************************
void* UNabto(void* pvParameters) {
    // device id and key from developer.nabto.com
    const char* nabtoId = "nvnwkucw.f7tuuf.appmyproduct.com";
    const char* presharedKey = "e76b98723bcecc85eade6a2b1a4fd54e";

    // Initialize uNabto
    nabto_main_setup* nms = unabto_init_context();
    nms->ipAddress = g_uiIpAddress;
    nms->id = nabtoId;
    nms->secureAttach = 1;
    nms->secureData = 1;
    nms->cryptoSuite = CRYPT_W_AES_CBC_HMAC_SHA256;

    const char* p;
    unsigned char* up;
    for (p = presharedKey, up = nms->presharedKey; *p; p += 2, ++up) {
        *up = hctoi(p[0]) * 16 + hctoi(p[1]);  // hex string to byte array
    }

    while ((!IS_CONNECTED(g_ulStatus)) || (!IS_IP_ACQUIRED(g_ulStatus))) {
        usleep(500);
    }

    srand(xTaskGetTickCount());

    stream_echo_init();
    unabto_init();


    while (true) {
        wait_event();
    }

    return NULL;
}

application_event_result application_event(application_request* appreq,
                                           unabto_query_request* r_b,
                                           unabto_query_response* w_b) {
    return AER_REQ_INV_QUERY_ID;
}

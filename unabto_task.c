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

////DEBUG
#include <ti/drivers/Timer.h>

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

static int32_t ServerSocket = -1;

static void SocketInit(int32_t localAddr, int32_t localPort)
{
    int32_t sd = sl_Socket(SL_AF_INET, SL_SOCK_DGRAM, 0);
    if (sd < 0) {
        NABTO_LOG_ERROR(
            ("Unable to create socket: (%i) '%s'.", errno, strerror(errno)));
    }

    struct SlSockAddrIn_t sa;
    memset(&sa, 0, sizeof(sa));
    sa.sin_family = SL_AF_INET;
    sa.sin_addr.s_addr = sl_Htonl(localAddr);
    sa.sin_port = sl_Htons(localPort);

    int status = sl_Bind(sd, (struct SlSockAddr_t*)&sa, sizeof(sa));
    if (status < 0) {
        NABTO_LOG_ERROR(("Unable to bind socket: (%i) '%s' localport %i", errno,
                         strerror(errno), localPort));
        sl_Close(sd);
    }

    SlSockNonblocking_t enableOption;
    enableOption.NonBlockingEnabled = 1;
    sl_SetSockOpt(sd, SL_SOL_SOCKET, SL_SO_NONBLOCKING, (uint8_t*)&enableOption,
                  sizeof(enableOption));

    ServerSocket = sd;

    NABTO_LOG_INFO(("Socket opened: ip=" PRIip ", port=%u",
                     MAKE_IP_PRINTABLE(localAddr), (int)localPort));
}

////DEBUG
static Timer_Handle timer0;
static Timer_Params params;
static uint32_t timerValue = 0u;
static uint32_t timerValue2 = 0u;
static bool isTimerInit = false;

static void debug_timer_init(void)
{
    if (!isTimerInit)
    {
        Timer_Params_init(&params);
        params.period = 0xFFFFFF00;
        params.periodUnits = Timer_PERIOD_COUNTS;
        params.timerMode = Timer_FREE_RUNNING;

        timer0 = Timer_open(Board_TIMER0, &params);

        if (timer0 == NULL) {
            /* Failed to initialized timer */
            while (1);
        }

        if (Timer_start(timer0) == Timer_STATUS_ERROR) {
                        /* Failed to start timer */
                        while (1);
                    }

        isTimerInit = true;
    }
}

typedef struct
{
    uint32_t timerTicks;
    uint32_t dataSize;
} debugMeasData_t;

static debugMeasData_t measBuffer[350] = {0};
static void debug_save_meas_data(uint32_t ticks, uint32_t size)
{
    static int i = 0;
    if (i < sizeof(measBuffer)/sizeof(measBuffer[0]))
    {
        measBuffer[i].timerTicks = ticks;
        measBuffer[i].dataSize = size;
        i += 1;
    }
}

static uint8_t TestBuffer[1510];

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
//    // device id and key from developer.nabto.com
//    const char* nabtoId = "nvnwkucw.f7tuuf.appmyproduct.com";
//    const char* presharedKey = "e76b98723bcecc85eade6a2b1a4fd54e";
//
//    // Initialize uNabto
//    nabto_main_setup* nms = unabto_init_context();
//
//    nms->id = nabtoId;
//    nms->secureAttach = 1;
//    nms->secureData = 1;
//    nms->cryptoSuite = CRYPT_W_AES_CBC_HMAC_SHA256;
//
//    const char* p;
//    unsigned char* up;
//    for (p = presharedKey, up = nms->presharedKey; *p; p += 2, ++up) {
//        *up = hctoi(p[0]) * 16 + hctoi(p[1]);  // hex string to byte array
//    }

    while ((!IS_CONNECTED(g_ulStatus)) || (!IS_IP_ACQUIRED(g_ulStatus))) {
        usleep(500);
    }

    SocketInit(g_uiIpAddress, 5570);
    debug_timer_init();

//    nms->ipAddress = g_uiIpAddress;
//    srand(xTaskGetTickCount());
//
//    stream_echo_init();
//    unabto_init();


    struct SlTimeval_t timeout_val;
    timeout_val.tv_sec = 5;
    SlFdSet_t read_fds;


    uint32_t remoteIp;
    uint16_t remotePort;
    int32_t readSize = 0;
    static uint32_t readCount = 0;
    while (true)
    {

        timeout_val.tv_sec = 5;
        SL_SOCKET_FD_ZERO(&read_fds);
        SL_SOCKET_FD_SET(ServerSocket, &read_fds);

        ////DEBUG
        //timerValue = Timer_getCount(timer0);
        int nfds = sl_Select(ServerSocket + 1, &read_fds, NULL, NULL, &timeout_val);
        //timerValue = Timer_getCount(timer0) - timerValue;

        readSize = -1;
        if (nfds < 0) {
            NABTO_LOG_ERROR(("Select returned error"));
        } else if (nfds > 0) {
            if (SL_SOCKET_FD_ISSET(ServerSocket, &read_fds)) {
                readSize = nabto_read(ServerSocket, TestBuffer, sizeof(TestBuffer), &remoteIp, &remotePort);
                readCount += 1;
            }
        }

        timerValue = Timer_getCount(timer0);
        // Save the no. of timer ticks converted to microseconds, after each packet has been recvd
        debug_save_meas_data(timerValue/80, readSize);
//        //wait_event();
//        unabto_tick();
//        usleep(5000);
    }

    return NULL;
}

application_event_result application_event(application_request* appreq,
                                           unabto_query_request* r_b,
                                           unabto_query_response* w_b) {
    return AER_REQ_INV_QUERY_ID;
}

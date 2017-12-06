#ifndef _UNABTO_PLATFORM_H_
#define _UNABTO_PLATFORM_H_

/* RTOS header files */
#include <ti/drivers/net/wifi/simplelink.h>
#include "FreeRTOS.h"
#include "task.h"

#include <platforms/unabto_common_types.h>
#include "unabto_platform_types.h"
#include "uart_term.h"

/**
* Generic typedefs
*/


/**
* Socket related definitions
*/

#define NABTO_INVALID_SOCKET -1

/**
* Time related definitions
*/
#define nabtoGetStamp(void) xTaskGetTickCount()
#define nabtoMsec2Stamp

/**
* Logging related definitions
*/



#define NABTO_LOG_BASIC_PRINT(severity, msg) \
    do {                                     \
        UART_PRINT msg;                      \
        UART_PRINT("\r\n");                  \
    } while (0)

/**
* Other defines
*/
#define LOOP_FOREVER()                       \
 do {                                        \
    } while (1)

#define NABTO_FATAL_EXIT LOOP_FOREVER();

#define MSEC_SLEEP(x)\
    if((x * 1000) >= 1000000)\
    {\
        sleep(x /1000);\
    }\
    else\
    {\
        usleep(1000*x);\
    }

#define SHOW_WARNING(ret, errortype)        UART_PRINT("\n\r[line:%d, error code:%d] %s\n\r", __LINE__, ret, errortype);

#define ASSERT_ON_ERROR(ret, errortype)\
        {\
            if(ret < 0)\
            {\
                SHOW_WARNING(ret, errortype);\
                return -1;\
            }\
        }

#define SET_STATUS_BIT(status_variable, bit) status_variable |= (1<<(bit))

#define CLR_STATUS_BIT(status_variable, bit) status_variable &= ~(1<<(bit))

#define GET_STATUS_BIT(status_variable, bit)    \
                                (0 != (status_variable & (1<<(bit))))

#define IS_CONNECTED(status_variable)       \
                GET_STATUS_BIT(status_variable, STATUS_BIT_CONNECTION)

#define IS_IP_ACQUIRED(status_variable)     \
                GET_STATUS_BIT(status_variable, STATUS_BIT_IP_ACQUIRED)

typedef enum{

    STATUS_BIT_NWP_INIT = 0,          /* This bit is set: Network Processor is powered up */


    STATUS_BIT_CONNECTION,            /* This bit is set: the device is connected
                                         to the AP or client is connected to device (AP) */

    STATUS_BIT_IP_LEASED,             /* This bit is set: the device has leased IP to
                                         any connected client */

    STATUS_BIT_IP_ACQUIRED,           /* This bit is set: the device has acquired an IP */


    STATUS_BIT_P2P_DEV_FOUND,         /* If this bit is set: the device (P2P mode)
                                         found any p2p-device in scan */

    STATUS_BIT_P2P_REQ_RECEIVED,      /* If this bit is set: the device (P2P mode)
                                         found any p2p-negotiation request */

    STATUS_BIT_CONNECTION_FAILED,     /* If this bit is set: the device(P2P mode)
                                         connection to client(or reverse way) is failed */

    STATUS_BIT_PING_STARTED,          /* This bit is set: device is undergoing ping operation */


    STATUS_BIT_SCAN_RUNNING,          /* This bit is set: Scan is running is background */


    STATUS_BIT_IPV6_ACQUIRED,         /* If this bit is set: the device has acquired
                                         an IPv6 address */

    STATUS_BIT_IPV6_GLOBAL_ACQUIRED,  /* If this bit is set: the device has acquired
                                         an IPv6 address */

    STATUS_BIT_IPV6_LOCAL_ACQUIRED,   /* If this bit is set: the device has acquired
                                        an IPv6 address */

    STATUS_BIT_AUTHENTICATION_FAILED, /* If this bit is set: Authentication with ENT AP failed. */


    STATUS_BIT_RESET_REQUIRED,


    STATUS_BIT_TX_STARED

}e_StatusBits;

#endif

//*****************************************************************************
// network_task.c
//
// Network Interface
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

#include <stdio.h>
#include <pthread.h>

#include "unabto_platform.h"
// Simplelink includes
#include <ti/drivers/net/wifi/simplelink.h>

// driverlib includes
#include "rom_map.h"
#include "utils.h"

// common interface includes
#include "uart_term.h"

//*****************************************************************************
//                          LOCAL DEFINES
//*****************************************************************************

#define AUTO_CONNECTION_TIMEOUT_COUNT (50)  // 5 Sec

#define SSID_LEN_MAX            (32)
#define BSSID_LEN_MAX           (6)
#define SL_STOP_TIMEOUT         (200)
#define CHANNEL_MASK_ALL        (0x1FFF)
#define RSSI_TH_MAX             (-95)

#define DEVICE_ERROR            ("Device error, please refer \"DEVICE ERRORS CODES\" section in errors.h")
#define WLAN_ERROR              ("WLAN error, please refer \"WLAN ERRORS CODES\" section in errors.h")
#define BSD_SOCKET_ERROR        ("BSD Socket error, please refer \"BSD SOCKET ERRORS CODES\" section in errors.h")
#define SL_SOCKET_ERROR         ("Socket error, please refer \"SOCKET ERRORS CODES\" section in errors.h")
#define NETAPP_ERROR            ("Netapp error, please refer \"NETAPP ERRORS CODES\" section in errors.h")
#define OS_ERROR                ("OS error, please refer \"NETAPP ERRORS CODES\" section in errno.h")

//
#define SSID_NAME       "<SSID name>"
#define SECURITY_TYPE   SL_WLAN_SEC_TYPE_WPA_WPA2
#define SECURITY_KEY    "SSID password"

//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************
extern pthread_t g_NetworkTask;
volatile unsigned long g_ulStatus = 0;               // SimpleLink Status
extern unsigned long g_uiIpAddress = 0;              // Device IP address
unsigned char g_ucConnectionSSID[SSID_LEN_MAX + 1];  // Connection SSID
unsigned char g_ucConnectionBSSID[BSSID_LEN_MAX];    // Connection BSSID

//*****************************************************************************
//                 GLOBAL VARIABLES -- End
//*****************************************************************************

//*****************************************************************************
// SimpleLink Asynchronous Event Handlers -- Start
//*****************************************************************************

//*****************************************************************************
//
//! \brief The Function Handles WLAN Events
//!
//! \param[in]  pWlanEvent - Pointer to WLAN Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkWlanEventHandler(SlWlanEvent_t *pWlanEvent) {
    if (pWlanEvent == NULL) {
        UART_PRINT("Null pointer\n\r");
        LOOP_FOREVER();
    }
    switch (pWlanEvent->Id) {
    case SL_WLAN_EVENT_CONNECT: {
        SET_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION);

        //
        // Information about the connected AP (like name, MAC etc) will be
        // available in 'slWlanConnectAsyncResponse_t'
        // Applications can use it if required
        //
        //  slWlanConnectAsyncResponse_t *pEventData = NULL;
        // pEventData = &pWlanEvent->EventData.STAandP2PModeWlanConnected;
        //

        // Copy new connection SSID and BSSID to global parameters
        memcpy(g_ucConnectionSSID,
               pWlanEvent->Data.Connect.SsidName,
               pWlanEvent->Data.Connect.SsidLen);
        memcpy(g_ucConnectionBSSID,
               pWlanEvent->Data.Connect.Bssid,
               BSSID_LEN_MAX);

        UART_PRINT(
            "[WLAN EVENT] STA Connected to the AP: %s , "
            "BSSID: %x:%x:%x:%x:%x:%x\n\r",
            g_ucConnectionSSID, g_ucConnectionBSSID[0], g_ucConnectionBSSID[1],
            g_ucConnectionBSSID[2], g_ucConnectionBSSID[3],
            g_ucConnectionBSSID[4], g_ucConnectionBSSID[5]);
    } break;

    case SL_WLAN_EVENT_DISCONNECT: {
        SlWlanEventDisconnect_t *pEventData = NULL;

        CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION);
        CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_ACQUIRED);
        CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_IPV6_ACQUIRED);

        pEventData = &pWlanEvent->Data.Disconnect;

        // If the user has initiated 'Disconnect' request,
        //'reason_code' is SL_WLAN_DISCONNECT_USER_INITIATED
        if (SL_WLAN_DISCONNECT_USER_INITIATED ==
            pEventData->ReasonCode) 
		{
            UART_PRINT(
                "\n\r[WLAN EVENT]Device disconnected from the AP: %s, "
                "BSSID: %x:%x:%x:%x:%x:%x on application's "
                "request \n\r",
                g_ucConnectionSSID, g_ucConnectionBSSID[0],
                g_ucConnectionBSSID[1], g_ucConnectionBSSID[2],
                g_ucConnectionBSSID[3], g_ucConnectionBSSID[4],
                g_ucConnectionBSSID[5]);
        } else {
            UART_PRINT(
                "[WLAN ERROR]Device disconnected from the AP AP: %s, "
                "BSSID: %x:%x:%x:%x:%x:%x on an ERROR..!! \n\r",
                g_ucConnectionSSID, g_ucConnectionBSSID[0],
                g_ucConnectionBSSID[1], g_ucConnectionBSSID[2],
                g_ucConnectionBSSID[3], g_ucConnectionBSSID[4],
                g_ucConnectionBSSID[5]);
        }
        memset(g_ucConnectionSSID, 0, sizeof(g_ucConnectionSSID));
        memset(g_ucConnectionBSSID, 0, sizeof(g_ucConnectionBSSID));
    } break;

    default: {
        UART_PRINT("[WLAN EVENT] Unexpected event [0x%x]\n\r",
                   pWlanEvent->Id);
    } break;
    }
}

//*****************************************************************************
//
//! \brief This function handles network events such as IP acquisition, IP
//!           leased, IP released etc.
//!
//! \param[in]  pNetAppEvent - Pointer to NetApp Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkNetAppEventHandler(SlNetAppEvent_t *pNetAppEvent) {
    if (pNetAppEvent == NULL) {
        UART_PRINT("Null pointer\n\r");
        LOOP_FOREVER();
    }

    switch (pNetAppEvent->Id) {
    case SL_NETAPP_EVENT_IPV4_ACQUIRED: {
        SlIpV4AcquiredAsync_t *pEventData = NULL;

        SET_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_ACQUIRED);

        // Ip Acquired Event Data
        pEventData = &pNetAppEvent->Data.IpAcquiredV4;
        g_uiIpAddress = pEventData->Ip;

        // Gateway IP address not set!!!


        UART_PRINT(
            "[NETAPP EVENT] IP Acquired: IP=%d.%d.%d.%d , "
            "Gateway=%d.%d.%d.%d\n\r",
            SL_IPV4_BYTE(g_uiIpAddress, 3),
            SL_IPV4_BYTE(g_uiIpAddress, 2),
            SL_IPV4_BYTE(g_uiIpAddress, 1),
            SL_IPV4_BYTE(g_uiIpAddress, 0),

            SL_IPV4_BYTE(pEventData->Gateway, 3),
            SL_IPV4_BYTE(pEventData->Gateway, 2),
            SL_IPV4_BYTE(pEventData->Gateway, 1),
            SL_IPV4_BYTE(pEventData->Gateway, 0));
    } break;

    default: {
        UART_PRINT("[NETAPP EVENT] Unexpected event [0x%x] \n\r",
                   pNetAppEvent->Id);
    } break;
    }
}

//*****************************************************************************
//
//! \brief This function handles HTTP server events
//!
//! \param[in]  pServerEvent - Contains the relevant event information
//! \param[in]    pServerResponse - Should be filled by the user with the
//!                                      relevant response information
//!
//! \return None
//!
//****************************************************************************
void SimpleLinkHttpServerEventHandler(SlNetAppHttpServerEvent_t *pHttpEvent,
                                      SlNetAppHttpServerResponse_t *pHttpResponse)
{
    // Unused in this application
}

//*****************************************************************************
//
//! \brief This function handles General Events
//!
//! This handler gets called whenever a general error is reported
//! by the NWP / Host driver. Since these errors are not fatal,
//! application can handle them.
//! \param[in]     pDevEvent - pointer to device error event.
//!
//! \return None
//!
//! \note           For more information, please refer to: user.h in the porting
//!                 folder of the host driver and the  CC3120/CC3220 NWP programmer's
//!                 guide (SWRU455) section 17.9.
//*****************************************************************************
void SimpleLinkGeneralEventHandler(SlDeviceEvent_t *pDevEvent) {
    if (pDevEvent == NULL) {
        UART_PRINT("Null pointer\n\r");
        LOOP_FOREVER();
    }

    //
    // Most of the general errors are not FATAL are are to be handled
    // appropriately by the application
    //
    UART_PRINT("[GENERAL EVENT] - ID=[%d] Sender=[%d]\n\n",
               pDevEvent->Data.Error.Code,
               pDevEvent->Data.Error.Source);
}

//*****************************************************************************
//
//! This function handles socket events indication
//!
//! \param[in]      pSock - Pointer to Socket Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkSockEventHandler(SlSockEvent_t *pSock) {
    if (pSock == NULL) {
        return;
    }

    //
    // This application doesn't work w/ socket - Events are not expected
    //
}

/*!
    \brief          SimpleLinkFatalErrorEventHandler

    This handler gets called whenever a socket event is reported
    by the NWP / Host driver. After this routine is called, the user's
    application must restart the device in order to recover.

    \param          slFatalErrorEvent    -   pointer to fatal error event.

    \return         void

    \note           For more information, please refer to: user.h in the porting
                    folder of the host driver and the  CC3120/CC3220 NWP programmer's
                    guide (SWRU455) section 17.9.

*/
void SimpleLinkFatalErrorEventHandler(SlDeviceFatal_t *slFatalErrorEvent)
{

    switch (slFatalErrorEvent->Id)
    {
        case SL_DEVICE_EVENT_FATAL_DEVICE_ABORT:
        {
            UART_PRINT("\n\r[ERROR] - FATAL ERROR: Abort NWP event detected: "
                        "AbortType=%d, AbortData=0x%x\n\r",
                        slFatalErrorEvent->Data.DeviceAssert.Code,
                        slFatalErrorEvent->Data.DeviceAssert.Value);
        }
        break;

        case SL_DEVICE_EVENT_FATAL_DRIVER_ABORT:
        {
            UART_PRINT("\n\r[ERROR] - FATAL ERROR: Driver Abort detected. \n\r");
        }
        break;

        case SL_DEVICE_EVENT_FATAL_NO_CMD_ACK:
        {
            UART_PRINT("\n\r[ERROR] - FATAL ERROR: No Cmd Ack detected "
                        "[cmd opcode = 0x%x] \n\r",
                                        slFatalErrorEvent->Data.NoCmdAck.Code);
        }
        break;

        case SL_DEVICE_EVENT_FATAL_SYNC_LOSS:
        {
            UART_PRINT("\n\r[ERROR] - FATAL ERROR: Sync loss detected n\r");
        }
        break;

        case SL_DEVICE_EVENT_FATAL_CMD_TIMEOUT:
        {
            UART_PRINT("\n\r[ERROR] - FATAL ERROR: Async event timeout detected "
                        "[event opcode =0x%x]  \n\r",
                                    slFatalErrorEvent->Data.CmdTimeout.Code);
        }
        break;

        default:
            UART_PRINT("\n\r[ERROR] - FATAL ERROR: Unspecified error detected \n\r");
        break;
    }
}

/*!
    \brief          SimpleLinkNetAppRequestEventHandler

    This handler gets called whenever a NetApp event is reported
    by the NWP / Host driver. User can write he's logic to handle
    the event here.

    \param          pNetAppRequest     -   Pointer to NetApp request structure.

    \param          pNetAppResponse    -   Pointer to NetApp request Response.

    \note           For more information, please refer to: user.h in the porting
                    folder of the host driver and the  CC3120/CC3220 NWP programmer's
                    guide (SWRU455) section 17.9.

    \return         void

*/
void SimpleLinkNetAppRequestEventHandler(SlNetAppRequest_t *pNetAppRequest, SlNetAppResponse_t *pNetAppResponse)
{
    /* Unused in this application */
}

/*!
    \brief          SimpleLinkNetAppRequestMemFreeEventHandler

    This handler gets called whenever the NWP is done handling with
    the buffer used in a NetApp request. This allows the use of
    dynamic memory with these requests.

    \param          pNetAppRequest     -   Pointer to NetApp request structure.

    \param          pNetAppResponse    -   Pointer to NetApp request Response.

    \note           For more information, please refer to: user.h in the porting
                    folder of the host driver and the  CC3120/CC3220 NWP programmer's
                    guide (SWRU455) section 17.9.

    \return         void

*/
void SimpleLinkNetAppRequestMemFreeEventHandler(uint8_t *buffer)
{
    /* Unused in this application */
}


//*****************************************************************************
// SimpleLink Asynchronous Event Handlers -- End
//*****************************************************************************

//*****************************************************************************
//
//! \brief This function initializes the application variables
//!
//! \param    None
//!
//! \return None
//!
//*****************************************************************************
static void InitializeAppVariables() {
    g_ulStatus = 0;
    g_uiIpAddress = 0;
    memset(g_ucConnectionSSID, 0, sizeof(g_ucConnectionSSID));
    memset(g_ucConnectionBSSID, 0, sizeof(g_ucConnectionBSSID));
}

//*****************************************************************************
//! \brief This function puts the device in its default state. It:
//!           - Set the mode to STATION
//!           - Configures connection policy to Auto and AutoSmartConfig
//!           - Deletes all the stored profiles
//!           - Enables DHCP
//!           - Disables Scan policy
//!           - Sets Tx power to maximum
//!           - Sets power policy to normal
//!           - Unregister mDNS services
//!           - Remove all filters
//!
//! \param   none
//! \return  On success, zero is returned. On error, negative is returned
//*****************************************************************************
long ConfigureSimpleLinkToDefaultState()
{
    uint8_t                              ucConfigOpt = 0;
	uint16_t                             usConfigLen = 0;
	uint8_t                              ucPower = 0;

    int32_t                              RetVal = -1;
    int32_t                              Mode = -1;
    uint32_t                             IfBitmap = 0;
    SlWlanScanParamCommand_t             ScanDefault = {0};
    SlWlanRxFilterOperationCommandBuff_t RxFilterIdMask = {{0}};
    SlDeviceVersion_t                    ver = {0};
    
	/* Turn NWP on */
    Mode = sl_Start(0, 0, 0);
    ASSERT_ON_ERROR(Mode, DEVICE_ERROR);

    // If the device is not in station-mode, try configuring it in station-mode
    if (ROLE_STA != Mode) 
	{
        if (ROLE_AP == Mode) 
		{
            // If the device is in AP mode, we need to wait for this event
            // before doing anything
            while (!IS_IP_ACQUIRED(g_ulStatus)) {
#ifndef SL_PLATFORM_MULTI_THREADED
                _SlNonOsMainLoopTask();
#endif
            }
        }

        // Switch to STA role and restart
        Mode = sl_WlanSetMode(ROLE_STA);
        ASSERT_ON_ERROR(Mode, WLAN_ERROR);

        RetVal = sl_Stop(SL_STOP_TIMEOUT);
        ASSERT_ON_ERROR(RetVal, DEVICE_ERROR);

         Mode = sl_Start(0, 0, 0);
         ASSERT_ON_ERROR(Mode, DEVICE_ERROR);
     }
     // Check if the device is in station again
     if(Mode != ROLE_STA)
	 {
         // We don't want to proceed if the device is not coming up in
         // STA-mode
         UART_PRINT("Failed to configure device to it's default state");
         return -1;
     }
    

    // Get the device's version-information
    ucConfigOpt = SL_DEVICE_GENERAL_VERSION;
    usConfigLen = sizeof(SlDeviceVersion_t);
    RetVal = sl_DeviceGet(SL_DEVICE_GENERAL, &ucConfigOpt, &usConfigLen, (uint8_t *)(&ver));
    ASSERT_ON_ERROR(RetVal, DEVICE_ERROR);

    UART_PRINT("Host Driver Version: %s\n\r", SL_DRIVER_VERSION);
    UART_PRINT("Build Version %d.%d.%d.%d.31.%d.%d.%d.%d.%d.%d.%d.%d\n\r",
               ver.NwpVersion[0], ver.NwpVersion[1], ver.NwpVersion[2], ver.NwpVersion[3],
               ver.FwVersion[0],  ver.FwVersion[1],  ver.FwVersion[2],  ver.FwVersion[3],
               ver.PhyVersion[0], ver.PhyVersion[1], ver.PhyVersion[2], ver.PhyVersion[3]);

    // Set connection policy to Auto + SmartConfig
    //      (Device's default connection policy)
    RetVal = sl_WlanPolicySet(SL_WLAN_POLICY_CONNECTION,
                               SL_WLAN_CONNECTION_POLICY(1, 0, 0, 0), NULL, 0);
    ASSERT_ON_ERROR(RetVal, WLAN_ERROR);

    // Disable Auto Provisioning
    RetVal = sl_WlanProvisioning(SL_WLAN_PROVISIONING_CMD_STOP, 0xFF, 0, NULL, 0x0);
    ASSERT_ON_ERROR(RetVal, WLAN_ERROR);

    // Remove all profiles
    RetVal = sl_WlanProfileDel(0xFF);
    ASSERT_ON_ERROR(RetVal, WLAN_ERROR);

    /* enable DHCP client */
    RetVal = sl_NetCfgSet(SL_NETCFG_IPV4_STA_ADDR_MODE, SL_NETCFG_ADDR_DHCP, 0, 0);
    ASSERT_ON_ERROR(RetVal, NETAPP_ERROR);
	
    //
    /* Disable ipv6 */
    IfBitmap = !(SL_NETCFG_IF_IPV6_STA_LOCAL | SL_NETCFG_IF_IPV6_STA_GLOBAL);
    RetVal = sl_NetCfgSet(SL_NETCFG_IF, SL_NETCFG_IF_STATE, sizeof(IfBitmap),(const unsigned char *)&IfBitmap);
    ASSERT_ON_ERROR(RetVal, NETAPP_ERROR);

    /* Configure scan parameters to default */
    ScanDefault.ChannelsMask = CHANNEL_MASK_ALL;
    ScanDefault.RssiThreshold = RSSI_TH_MAX;

    RetVal = sl_WlanSet(SL_WLAN_CFG_GENERAL_PARAM_ID, SL_WLAN_GENERAL_PARAM_OPT_SCAN_PARAMS, sizeof(ScanDefault), (uint8_t *)&ScanDefault);
    ASSERT_ON_ERROR(RetVal, WLAN_ERROR);
	 
    // Disable scans
    ucConfigOpt = SL_WLAN_SCAN_POLICY(0, 0);
    RetVal = sl_WlanPolicySet(SL_WLAN_POLICY_SCAN, ucConfigOpt, NULL, 0);
    ASSERT_ON_ERROR(RetVal, WLAN_ERROR);

    // Set Tx power level for station mode
    // Number between 0-15, as dB offset from max power - 0 will set max power
    ucPower = 0;
    RetVal = sl_WlanSet(SL_WLAN_CFG_GENERAL_PARAM_ID,
                         SL_WLAN_GENERAL_PARAM_OPT_STA_TX_POWER, 1,
                         (unsigned char *)&ucPower);
    ASSERT_ON_ERROR(RetVal, WLAN_ERROR);

    /* Set NWP Power policy to 'normal' */
    RetVal = sl_WlanPolicySet(SL_WLAN_POLICY_PM, SL_WLAN_NORMAL_POLICY, NULL, 0);
    ASSERT_ON_ERROR(RetVal, WLAN_ERROR);

    // Unregister mDNS services
    RetVal = sl_NetAppMDNSUnRegisterService(0, 0, 0);
    ASSERT_ON_ERROR(RetVal, NETAPP_ERROR);

    // Remove  all 64 filters (8*8)
    memset(RxFilterIdMask.FilterBitmap , 0xFF, 8);
    RetVal = sl_WlanSet(SL_WLAN_RX_FILTERS_ID, SL_WLAN_RX_FILTER_REMOVE, sizeof(SlWlanRxFilterOperationCommandBuff_t),(uint8_t *)&RxFilterIdMask);
    ASSERT_ON_ERROR(RetVal, WLAN_ERROR);

    /* Set NWP role as STA */
    RetVal = sl_WlanSetMode(ROLE_STA);
    ASSERT_ON_ERROR(RetVal, WLAN_ERROR);

    /* For changes to take affect, we restart the NWP */
    RetVal = sl_Stop(0xFF);
    ASSERT_ON_ERROR(RetVal, DEVICE_ERROR);

    Mode = sl_Start(0, 0, 0);
    ASSERT_ON_ERROR(Mode, DEVICE_ERROR);
    if(ROLE_STA != Mode)
    {
        UART_PRINT("Failed to configure device to it's default state");
        RetVal = -1;
    }

    return RetVal;  // Success
}

//****************************************************************************
//
//!  \brief Connecting to a WLAN Accesspoint
//!
//!   This function connects to the required AP (SSID_NAME) with Security
//!   parameters specified in te form of macros at the top of this file
//!
//!   \param[in]              None
//!
//!   \return       status value
//!
//!   \warning    If the WLAN connection fails or we don't aquire an IP
//!            address, It will be stuck in this function forever.
//
//****************************************************************************
long WlanConnect() {
    SlWlanSecParams_t secParams = {0};
    long lRetVal = 0;

    secParams.Key = (signed char *)SECURITY_KEY;
    secParams.KeyLen = strlen(SECURITY_KEY);
    secParams.Type = SECURITY_TYPE;

    lRetVal = sl_WlanConnect((signed char *)SSID_NAME, strlen(SSID_NAME), 0,
                             &secParams, 0);
    ASSERT_ON_ERROR(lRetVal, WLAN_ERROR);

    while ((!IS_CONNECTED(g_ulStatus)) || (!IS_IP_ACQUIRED(g_ulStatus))) {
// Wait for WLAN Event
#ifndef SL_PLATFORM_MULTI_THREADED
        _SlNonOsMainLoopTask();
#endif
    }

    return 0;
}

//*****************************************************************************
//
//! Network Task
//!
//! \param  pvParameters - Parameters to the task's entry function
//!
//! \return None
//!
//*****************************************************************************
void *Network(void *pvParameters) {
    long lRetVal = -1;

    // Initialize Global Variables
    InitializeAppVariables();

    //
    // Following function configure the device to default state by cleaning
    // the persistent settings stored in NVMEM (viz. connection profiles &
    // policies, power policy etc)
    //
    // Applications may choose to skip this step if the developer is sure
    // that the device is in its desired state at start of applicaton
    //
    // Note that all profiles and persistent settings that were done on the
    // device will be lost
    //
    lRetVal = ConfigureSimpleLinkToDefaultState();
    if (lRetVal < 0) {
        UART_PRINT("Failed to configure the device in its default state \n\r");
        LOOP_FOREVER();
    }

    UART_PRINT("Device is configured in default state \n\r");

    //
    // Assumption is that the device is configured in station mode already
    // and it is in its default state
    //
//    lRetVal = sl_Start(0, 0, 0);
//    if (lRetVal < 0 || lRetVal != ROLE_STA) {
//        UART_PRINT("Failed to start the device \n\r");
//        LOOP_FOREVER();
//    }

    UART_PRINT("Device started as STATION \n\r");

    UART_PRINT("Connecting to AP: %s ...\r\n", SSID_NAME);

    //
    // Connecting to WLAN AP
    //
    lRetVal = WlanConnect();
    if (lRetVal < 0) {
        UART_PRINT("Failed to establish connection w/ an AP \n\r");
        LOOP_FOREVER();
    }

    UART_PRINT("Connected to AP: %s \n\r", SSID_NAME);

    UART_PRINT("Device IP: %d.%d.%d.%d\n\r\n\r", SL_IPV4_BYTE(g_uiIpAddress, 3),
               SL_IPV4_BYTE(g_uiIpAddress, 2), SL_IPV4_BYTE(g_uiIpAddress, 1),
               SL_IPV4_BYTE(g_uiIpAddress, 0));

    return NULL;
}

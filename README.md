# uNabto CC3220 Streaming Demo

This project turns the CC3220 into a Nabto streaming echo server. It uses FreeRTOS and leverages the CC3220's hardware accelerated cryptography capabilities.

# How to set it up

## Step 1: Clone the repository

*Hint: Preferably, do this in your Code Composer Studio workspace.*

```
mkdir unabto-cc3220 && cd unabto-cc3220
git clone --recursive https://github.com/gcarstoiu-cr/unabto-cc3200.git .
cd unabto
git checkout master
```

## Step 2: Import Project into Code Composer Studio

1. *File* > *Import*
2. Select *Code Compose Studio* > *CCS Projects*
3. Press *Next*
4. *Browse...* to the `unabto-cc3200` folder inside you CCS workspace
5. Untick *Copy projects into workspace*
6. Press *Finish*

If your CC3200 SDK is not located in `C:/TI/simplelink_cc32xx_sdk_1_50_00_06` goto

1. *Project* > *Properties*
2. *Select Resource* > *Linked Resources*

and fix the *Path Variables* in order to point to the right directories.

## Step 3: Set Wi-Fi SSID and Password

If not done for previous SDK examples, open `network_task.c` and configure the access-point defines `SSID_NAME`, `SECURITY_TYPE` and `SECURITY_KEY`.

## Step 4: Set Nabto Device ID and Key

Enter your *Device ID* and *Key* from [developer.nabto.com](developer.nabto.com) in `unabto_task.c` lines 66 and 67:

```
    char* nabtoId = "<DEVICE ID>";
    const char* presharedKey = "<KEY>";
```

## Step 5: Build the project

*Project* > *Build All*.

## Step 6: Flash the Image

Please refer to the [CC31xx & CC32xx UniFlash Quick Start Guide](http://processors.wiki.ti.com/index.php/CC31xx_%26_CC32xx_UniFlash_Quick_Start_Guide#CC32xx_MCU_image_flashing) for flashing the image located in `<YOUR-CCS-WORSPACE>\unabto-cc3200\Release\unabto-cc3200.bin`.

# How to test the application

Using a serial terminal you should see a printout similar to the following every time the CC3200 starts up:

```
*************************************************
                   CC3220 + uNabto
*************************************************

Host Driver Version: 2.0.1.22
Build Version 3.5.0.0.31.2.0.0.0.2.2.0.5
Device is configured in default state
Device started as STATION
Connecting to AP: ZZZ ...
[WLAN EVENT] STA Connected to the AP: ZZZ , BSSID: 18:a6:f7:e7:ed:7f
[NETAPP EVENT] IP Acquired: IP=192.168.0.101 , Gateway=192.168.0.1
Device id: 'deviceid.demo.nab.to'
Program Release 4.2.0-alpha.0
Connected to AP: ZZZsizeof(stream__)=u
SECURE ATTACH: 1, DATA: 1
NONCE_SIZE: 32, CLEAR_TEXT: 0

Device IP: 192.168.0.101

Nabto was successfully initialized
SECURE ATTACH: 1, DATA: 1
NONCE_SIZE: 32, CLEAR_TEXT: 0
State change from IDLE to WAIT_DNS
Resolving DNS for deviceid.demo.nab.to
Resolved DNS for deviceid.demo.nab.to to:
  Controller ip: 54.72.234.97
State change from WAIT_DNS to WAIT_BS
State change from WAIT_BS to WAIT_GSP
GSP address: 52.31.229.62:5565
########    U_INVITE with LARGE nonce sent, version: - URL: -
State change from WAIT_GSP to ATTACHED
```

Using the modified [Read Stream Tester](https://github.com/gcarstoiu-cr/echo-stream-tester.git) application (branch cc3220-stream) you can now test the stream server by sending and receiving e.g. 1 MB of data:

```
./echo_stream_tester <DEVICE ID> 1000000
```


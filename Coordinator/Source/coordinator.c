/****************************************************************************
 * $Rev::               $: Revision of last commit
 * $Author::            $: Author of last commit
 * $Date::              $: Date of last commit
 * $HeadURL$
 ****************************************************************************
 * This software is owned by Jennic and/or its supplier and is protected
 * under applicable copyright laws. All rights are reserved. We grant You,
 * and any third parties, a license to use this software solely and
 * exclusively on Jennic products. You, and any third parties must reproduce
 * the copyright and warranty notice and any other legend of ownership on each
 * copy or partial copy of the software.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS". JENNIC MAKES NO WARRANTIES, WHETHER
 * EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE,
 * ACCURACY OR LACK OF NEGLIGENCE. JENNIC SHALL NOT, IN ANY CIRCUMSTANCES,
 * BE LIABLE FOR ANY DAMAGES, INCLUDING, BUT NOT LIMITED TO, SPECIAL,
 * INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR ANY REASON WHATSOEVER.
 *
 * Copyright Jennic Ltd 2009. All rights reserved
 ****************************************************************************/

#define UART                    E_AHI_UART_0
#define CACHED_AVERAGES         10
#define BYTE_TO_BINARY_PATTERN  "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)  \
  (byte & 0x80 ? '1' : '0'), \
  (byte & 0x40 ? '1' : '0'), \
  (byte & 0x20 ? '1' : '0'), \
  (byte & 0x10 ? '1' : '0'), \
  (byte & 0x08 ? '1' : '0'), \
  (byte & 0x04 ? '1' : '0'), \
  (byte & 0x02 ? '1' : '0'), \
  (byte & 0x01 ? '1' : '0') 

/****************************************************************************/
/***        Include files                                                 ***/
/****************************************************************************/
#include <jendefs.h>
#include <AppHardwareApi.h>
#include <AppQueueApi.h>
#include <mac_sap.h>
#include <mac_pib.h>
#include <AppApiTof.h>
#include <LedControl.h>
#include "LcdDriver.h"
#include "config.h"
#include "Printf.h"
#include <math.h>

//#include "gdb.h"  // not reqd for x47 build


/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/

/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/
typedef enum
{
    E_STATE_IDLE,
    E_STATE_ENERGY_SCANNING,
    E_STATE_COORDINATOR_STARTED,
}teState;

/* Data type for storing data related to all end devices that have associated */
typedef struct
{
    bool_t bIsAssociated;
    int32 i32TofDistance;
    uint32 u32RssiDistance;
    uint16 u16ShortAdr;
    uint32 u32ExtAdrL;
    uint32 u32ExtAdrH;
    uint8   u8TxPacketSeqNb;
    uint8   u8RxPacketSeqNb;
}tsEndDeviceData;

typedef struct
{
    /* Data related to associated end devices */
    uint16          u16NbrEndDevices;
    tsEndDeviceData sEndDeviceData[MAX_END_DEVICES];
    teState eState;
    uint8   u8Channel;
    double x;
    double y;
}tsCoordinatorData;

/****************************************************************************/
/***        Local Function Prototypes                                     ***/
/****************************************************************************/
PRIVATE void vInitSystem(void);
PRIVATE void vStartEnergyScan(void);
PRIVATE void vStartCoordinator(void);
PRIVATE void vProcessEventQueues(void);
PRIVATE void vProcessIncomingMlme(MAC_MlmeDcfmInd_s *psMlmeInd);
PRIVATE void vProcessIncomingMcps(MAC_McpsDcfmInd_s *psMcpsInd);
PRIVATE void vProcessIncomingHwEvent(AppQApiHwInd_s *psAHI_Ind);
PRIVATE void vHandleNodeAssociation(MAC_MlmeDcfmInd_s *psMlmeInd);
PRIVATE void vHandleEnergyScanResponse(MAC_MlmeDcfmInd_s *psMlmeInd);
PRIVATE void vHandleMcpsDataInd(MAC_McpsDcfmInd_s *psMcpsInd);
PRIVATE void vHandleMcpsDataDcfm(MAC_McpsDcfmInd_s *psMcpsInd);
PRIVATE void vProcessReceivedDataPacket(uint8 *pu8Data, uint8 u8Len, uint16 u16Address);
PRIVATE void vPutChar(unsigned char c);
PRIVATE void reverse(char *str, int len);
PRIVATE int intToStr(uint32 x, char str[], int d);

PRIVATE uint32 GetDistance(uint16 iEndDevice);
PRIVATE void lcd_BuildStatusScreen(void);
PRIVATE void lcd_UpdateStatusScreen(void);
PRIVATE void interrupt_handleDistanceTransmissionReceived(uint8 *pu8Data, uint8 u8Len, uint16 u16Address);
PRIVATE void task_CalculateXYPos(void);

/****************************************************************************/
/***        Exported Variables                                            ***/
/****************************************************************************/

/****************************************************************************/
/***        Local Variables                                               ***/
/****************************************************************************/
/* Handles from the MAC */
PRIVATE void *s_pvMac;
PRIVATE MAC_Pib_s *s_psMacPib;

PRIVATE tsCoordinatorData sCoordinatorData;
PRIVATE bool_t bLedState;

/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/

/****************************************************************************
 *
 * NAME: AppColdStart
 *
 * DESCRIPTION:
 * Entry point for application from boot loader. Initialises system and runs
 * main loop.
 *
 * RETURNS:
 * Never returns.
 *
 ****************************************************************************/
PUBLIC void AppColdStart(void)
{
    volatile int n=0;

#ifdef WATCHDOG_ENABLED
    vAHI_WatchdogStop();
#endif

    vAHI_UartEnable(UART);
    vAHI_UartReset(UART, TRUE, TRUE);
    vAHI_UartSetClockDivisor(UART, E_AHI_UART_RATE_38400);
    vAHI_UartReset(UART, FALSE, FALSE);

    vInitSystem();
    vInitPrintf((void *)vPutChar);
    vLcdResetDefault();
    lcd_BuildStatusScreen();

    //Enable TOF ranging.
    vAppApiTofInit(TRUE);

    vStartEnergyScan();

    vLedInitRfd();

    while (1)
    {
        for(n=0;n<1000000;n++);
        bLedState = !bLedState;
        vLedControl(0, bLedState);
        lcd_BuildStatusScreen();
        vProcessEventQueues();
        task_CalculateXYPos();
    }
}

/****************************************************************************
 *
 * NAME: AppWarmStart
 *
 * DESCRIPTION:
 * Entry point for application from boot loader. Simply jumps to AppColdStart
 * as, in this instance, application will never warm start.
 *
 * RETURNS:
 * Never returns.
 *
 ****************************************************************************/
PUBLIC void AppWarmStart(void)
{
    AppColdStart();
}

/****************************************************************************/
/***        Local Functions                                               ***/
/****************************************************************************/

/****************************************************************************
 *
 * NAME: vInitSystem
 *
 * DESCRIPTION:
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PRIVATE void vInitSystem(void)
{
    /* Setup interface to MAC */
    (void)u32AppQApiInit(NULL, NULL, NULL);
    (void)u32AHI_Init();

	/* Enable high power modules */
	//vAHI_HighPowerModuleEnable(TRUE, TRUE);

    /* Initialise coordinator state */
    sCoordinatorData.eState = E_STATE_IDLE;
    sCoordinatorData.u16NbrEndDevices = 0;

    int i;
    for (i=0; i<MAX_END_DEVICES; i++)
    {
        sCoordinatorData.sEndDeviceData[i].bIsAssociated = FALSE;
        sCoordinatorData.sEndDeviceData[i].i32TofDistance = 0;
        sCoordinatorData.sEndDeviceData[i].u32RssiDistance = 0;
        sCoordinatorData.sEndDeviceData[i].u8RxPacketSeqNb = 0;
        sCoordinatorData.sEndDeviceData[i].u8TxPacketSeqNb = 0;
    }

    /* Set up the MAC handles. Must be called AFTER u32AppQApiInit() */
    s_pvMac = pvAppApiGetMacHandle();
    s_psMacPib = MAC_psPibGetHandle(s_pvMac);

    /* Set Pan ID and short address in PIB (also sets match registers in hardware) */
    MAC_vPibSetPanId(s_pvMac, PAN_ID);
    MAC_vPibSetShortAddr(s_pvMac, COORDINATOR_ADR);

    /* Enable receiver to be on when idle */
    MAC_vPibSetRxOnWhenIdle(s_pvMac, TRUE, FALSE);

    /* Allow nodes to associate */
    s_psMacPib->bAssociationPermit = 1;
}

/****************************************************************************
 *
 * NAME: vProcessEventQueues
 *
 * DESCRIPTION:
 * Check each of the three event queues and process and items found.
 *
 * PARAMETERS:      Name            RW  Usage
 * None.
 *
 * RETURNS:
 * None.
 *
 * NOTES:
 * None.
 ****************************************************************************/
PRIVATE void vProcessEventQueues(void)
{
    MAC_MlmeDcfmInd_s *psMlmeInd;
	MAC_McpsDcfmInd_s *psMcpsInd;
    AppQApiHwInd_s    *psAHI_Ind;

    /* Check for anything on the MCPS upward queue */
    do
    {
        psMcpsInd = psAppQApiReadMcpsInd();
        if (psMcpsInd != NULL)
        {
            vProcessIncomingMcps(psMcpsInd);
            vAppQApiReturnMcpsIndBuffer(psMcpsInd);
        }
    } while (psMcpsInd != NULL);

    /* Check for anything on the MLME upward queue */
    do
    {
        psMlmeInd = psAppQApiReadMlmeInd();
        if (psMlmeInd != NULL)
        {
            vProcessIncomingMlme(psMlmeInd);
            vAppQApiReturnMlmeIndBuffer(psMlmeInd);
        }
    } while (psMlmeInd != NULL);

    /* Check for anything on the AHI upward queue */
    do
    {
        psAHI_Ind = psAppQApiReadHwInd();
        if (psAHI_Ind != NULL)
        {
            vProcessIncomingHwEvent(psAHI_Ind);
            vAppQApiReturnHwIndBuffer(psAHI_Ind);
        }
    } while (psAHI_Ind != NULL);
}

/****************************************************************************
 *
 * NAME: vProcessIncomingMlme
 *
 * DESCRIPTION:
 * Process any incoming managment events from the stack.
 *
 * PARAMETERS:      Name            RW  Usage
 *                  psMlmeInd
 *
 * RETURNS:
 * None.
 *
 * NOTES:
 * None.
 ****************************************************************************/
PRIVATE void vProcessIncomingMlme(MAC_MlmeDcfmInd_s *psMlmeInd)
{
    switch (psMlmeInd->u8Type)
    {
    case MAC_MLME_IND_ASSOCIATE: /* Incoming association request */
        if (sCoordinatorData.eState == E_STATE_COORDINATOR_STARTED)
        {
            vHandleNodeAssociation(psMlmeInd);
        }
        break;

    case MAC_MLME_DCFM_SCAN: /* Incoming scan results */
        if (psMlmeInd->uParam.sDcfmScan.u8ScanType == MAC_MLME_SCAN_TYPE_ENERGY_DETECT)
        {
            if (sCoordinatorData.eState == E_STATE_ENERGY_SCANNING)
            {
                /* Process energy scan results and start device as coordinator */
                vHandleEnergyScanResponse(psMlmeInd);
            }
        }
        break;

    default:
        break;
    }
}

/****************************************************************************
 *
 * NAME: vProcessIncomingData
 *
 * DESCRIPTION:
 * Process incoming data events from the stack.
 *
 * PARAMETERS:      Name            RW  Usage
 *                  psMcpsInd
 *
 * RETURNS:
 * None.
 *
 * NOTES:
 * None.
 ****************************************************************************/
PRIVATE void vProcessIncomingMcps(MAC_McpsDcfmInd_s *psMcpsInd)
{
    /* Only handle incoming data events one device has been started as a
       coordinator */
    if (sCoordinatorData.eState >= E_STATE_COORDINATOR_STARTED)
    {
        switch(psMcpsInd->u8Type)
        {
        case MAC_MCPS_IND_DATA:  /* Incoming data frame */
            vHandleMcpsDataInd(psMcpsInd);
            break;
        case MAC_MCPS_DCFM_DATA: /* Incoming acknowledgement or ack timeout */
            vHandleMcpsDataDcfm(psMcpsInd);
            break;
        default:
            break;
        }
    }
}

/****************************************************************************
 *
 * NAME: vHandleMcpsDataDcfm
 *
 * DESCRIPTION:
 *
 * PARAMETERS:      Name            RW  Usage
 *
 * RETURNS:
 *
 * NOTES:
 ****************************************************************************/
PRIVATE void vHandleMcpsDataDcfm(MAC_McpsDcfmInd_s *psMcpsInd)
{
    if (psMcpsInd->uParam.sDcfmData.u8Status == MAC_ENUM_SUCCESS)
    {
        /* Data frame transmission successful */
    }
    else
    {
        /* Data transmission falied after 3 retries at MAC layer. */
    }
}

/****************************************************************************
 *
 * NAME: vHandleMcpsDataInd
 *
 * DESCRIPTION:
 *
 * PARAMETERS:      Name            RW  Usage
 *
 * RETURNS:
 *
 * NOTES:
 ****************************************************************************/
PRIVATE void vHandleMcpsDataInd(MAC_McpsDcfmInd_s *psMcpsInd)
{
    MAC_RxFrameData_s *psFrame;

    psFrame = &psMcpsInd->uParam.sIndData.sFrame;

    /* Check application layer sequence number of frame and reject if it is
       the same as the last frame, i.e. same frame has been received more
       than once. */
    uint16 u16EndDeviceIndex = psFrame->sSrcAddr.uAddr.u16Short - 1;
    if (psFrame->au8Sdu[0] >= sCoordinatorData.sEndDeviceData[u16EndDeviceIndex].u8RxPacketSeqNb)
    {
        sCoordinatorData.sEndDeviceData[u16EndDeviceIndex].u8RxPacketSeqNb++;

        vProcessReceivedDataPacket(&psFrame->au8Sdu[1],
                                   (psFrame->u8SduLength) - 1,
                                   psFrame->sSrcAddr.uAddr.u16Short);
    }
}
/****************************************************************************
 *
 * NAME: vProcessReceivedDataPacket
 *
 * DESCRIPTION:
 *
 * PARAMETERS:      Name            RW  Usage
 *
 * RETURNS:
 *
 * NOTES:
 ****************************************************************************/
PRIVATE void vProcessReceivedDataPacket(uint8 *pu8Data, uint8 u8Len, uint16 u16Address)
{
    vPrintf("\nReceived data Packet %i long\n", u8Len);
    if (u8Len >= 1)
    {
        uint8 firstByte = pu8Data[0];
        switch(firstByte)
        {
            case 0xd1:
                interrupt_handleDistanceTransmissionReceived(&pu8Data[1], u8Len-1, u16Address);
                break;
            default:
                vPrintf("Unexpected data packet.\n");
                break;
        }
    }
}

PRIVATE void interrupt_handleDistanceTransmissionReceived(uint8 *pu8Data, uint8 u8Len, uint16 u16Address)
{
    vPrintf("\nDistance Transmission Received From Beacon %i.\n", u16Address);
    #ifdef DEBUG_DISTANCE_TRANSMISSION
        vPrintf("TOF  Byte0: "BYTE_TO_BINARY_PATTERN"\n", BYTE_TO_BINARY(pu8Data[0]));
        vPrintf("TOF  Byte1: "BYTE_TO_BINARY_PATTERN"\n", BYTE_TO_BINARY(pu8Data[1]));
        vPrintf("TOF  Byte2: "BYTE_TO_BINARY_PATTERN"\n", BYTE_TO_BINARY(pu8Data[2]));
        vPrintf("TOF  Byte3: "BYTE_TO_BINARY_PATTERN"\n\n", BYTE_TO_BINARY(pu8Data[3]));
        vPrintf("RSSI Byte0: "BYTE_TO_BINARY_PATTERN"\n", BYTE_TO_BINARY(pu8Data[4]));
        vPrintf("RSSI Byte1: "BYTE_TO_BINARY_PATTERN"\n", BYTE_TO_BINARY(pu8Data[5]));
        vPrintf("RSSI Byte2: "BYTE_TO_BINARY_PATTERN"\n", BYTE_TO_BINARY(pu8Data[6]));
        vPrintf("RSSI Byte3: "BYTE_TO_BINARY_PATTERN"\n\n", BYTE_TO_BINARY(pu8Data[7]));
    #endif

    uint32 highByte = ((uint32)pu8Data[0]) << 24;
    uint32 midHighByte = ((uint32)pu8Data[1]) << 16;
    uint32 midLowByte = ((uint32)pu8Data[2]) << 8;
    uint32 lowByte = ((uint32)pu8Data[3]);

    uint16 u16EndDeviceIndex = u16Address - 1;
    sCoordinatorData.sEndDeviceData[u16EndDeviceIndex].i32TofDistance = ((int32)highByte) | midHighByte | midLowByte | lowByte;

    highByte = ((uint32)pu8Data[4]) << 24;
    midHighByte = ((uint32)pu8Data[5]) << 16;
    midLowByte = ((uint32)pu8Data[6]) << 8;
    lowByte = ((uint32)pu8Data[7]);

    sCoordinatorData.sEndDeviceData[u16EndDeviceIndex].u32RssiDistance = highByte | midHighByte | midLowByte | lowByte;

    vPrintf("TOF Distance: %i cm", sCoordinatorData.sEndDeviceData[u16EndDeviceIndex].i32TofDistance);
    vPrintf("RSSI Distance: %i cm", sCoordinatorData.sEndDeviceData[u16EndDeviceIndex].u32RssiDistance);
}

/****************************************************************************
 *
 * NAME: vProcessIncomingHwEvent
 *
 * DESCRIPTION:
 * Process any hardware events.
 *
 * PARAMETERS:      Name            RW  Usage
 *                  psAHI_Ind
 *
 * RETURNS:
 * None.
 *
 * NOTES:
 * None.
 ****************************************************************************/
PRIVATE void vProcessIncomingHwEvent(AppQApiHwInd_s *psAHI_Ind)
{
}

/****************************************************************************
 *
 * NAME: vHandleNodeAssociation
 *
 * DESCRIPTION:
 * Handle request by node to join the network.
 *
 * PARAMETERS:      Name            RW  Usage
 *                  psMlmeInd
 *
 * RETURNS:
 * None.
 *
 * NOTES:
 * None.
 ****************************************************************************/
PRIVATE void vHandleNodeAssociation(MAC_MlmeDcfmInd_s *psMlmeInd)
{
    uint16 u16ShortAdr = 0xffff;
    uint16 u16EndDeviceIndex;


    MAC_MlmeReqRsp_s   sMlmeReqRsp;
    MAC_MlmeSyncCfm_s  sMlmeSyncCfm;

    if (sCoordinatorData.u16NbrEndDevices < MAX_END_DEVICES)
    {
        /* Store end device address data */
        u16EndDeviceIndex    = sCoordinatorData.u16NbrEndDevices;
        u16ShortAdr = END_DEVICE_START_ADR + sCoordinatorData.u16NbrEndDevices;

        sCoordinatorData.sEndDeviceData[u16EndDeviceIndex].u16ShortAdr = u16ShortAdr;

        sCoordinatorData.sEndDeviceData[u16EndDeviceIndex].u32ExtAdrL  =
        psMlmeInd->uParam.sIndAssociate.sDeviceAddr.u32L;

        sCoordinatorData.sEndDeviceData[u16EndDeviceIndex].u32ExtAdrH  =
        psMlmeInd->uParam.sIndAssociate.sDeviceAddr.u32H;
        sCoordinatorData.sEndDeviceData[u16EndDeviceIndex].bIsAssociated = TRUE;
        vPrintf("Beacon %i Associated: %i\n", u16EndDeviceIndex, u16ShortAdr);
        sCoordinatorData.u16NbrEndDevices++;

        sMlmeReqRsp.uParam.sRspAssociate.u8Status = 0; /* Access granted */
    }
    else
    {
        sMlmeReqRsp.uParam.sRspAssociate.u8Status = 2; /* Denied */
    }

    /* Create association response */
    sMlmeReqRsp.u8Type = MAC_MLME_RSP_ASSOCIATE;
    sMlmeReqRsp.u8ParamLength = sizeof(MAC_MlmeRspAssociate_s);
    sMlmeReqRsp.uParam.sRspAssociate.sDeviceAddr.u32H = psMlmeInd->uParam.sIndAssociate.sDeviceAddr.u32H;
    sMlmeReqRsp.uParam.sRspAssociate.sDeviceAddr.u32L = psMlmeInd->uParam.sIndAssociate.sDeviceAddr.u32L;
    sMlmeReqRsp.uParam.sRspAssociate.u16AssocShortAddr = u16ShortAdr;

    sMlmeReqRsp.uParam.sRspAssociate.u8SecurityEnable = FALSE;

    /* Send association response. There is no confirmation for an association
       response, hence no need to check */
    vAppApiMlmeRequest(&sMlmeReqRsp, &sMlmeSyncCfm);
}

/****************************************************************************
 *
 * NAME: vStartEnergyScan
 *
 * DESCRIPTION:
 * Starts an enery sacn on the channels specified.
 *
 * PARAMETERS:      Name            RW  Usage
 * None.
 *
 * RETURNS:
 * None.
 *
 * NOTES:
 * None.
 ****************************************************************************/
PRIVATE void vStartEnergyScan(void)
{
    /* Structures used to hold data for MLME request and response */
    MAC_MlmeReqRsp_s   sMlmeReqRsp;
    MAC_MlmeSyncCfm_s  sMlmeSyncCfm;

    sCoordinatorData.eState = E_STATE_ENERGY_SCANNING;

    /* Start energy detect scan */
    sMlmeReqRsp.u8Type = MAC_MLME_REQ_SCAN;
    sMlmeReqRsp.u8ParamLength = sizeof(MAC_MlmeReqStart_s);
    sMlmeReqRsp.uParam.sReqScan.u8ScanType = MAC_MLME_SCAN_TYPE_ENERGY_DETECT;
    sMlmeReqRsp.uParam.sReqScan.u32ScanChannels = SCAN_CHANNELS;
    sMlmeReqRsp.uParam.sReqScan.u8ScanDuration = ENERGY_SCAN_DURATION;

    vAppApiMlmeRequest(&sMlmeReqRsp, &sMlmeSyncCfm);
}


/****************************************************************************
 *
 * NAME: vHandleEnergyScanResponse
 *
 * DESCRIPTION:
 * Selects a channel with low enery content for use by the wireless UART.
 *
 * PARAMETERS:      Name            RW  Usage
 * None.
 *
 * RETURNS:
 * None.
 *
 * NOTES:
 * None.
 ****************************************************************************/
PRIVATE void vHandleEnergyScanResponse(MAC_MlmeDcfmInd_s *psMlmeInd)
{
    uint8 u8MinEnergy;

	u8MinEnergy = (psMlmeInd->uParam.sDcfmScan.uList.au8EnergyDetect[0]) ;

    sCoordinatorData.u8Channel = CHANNEL_MIN;

	/* Search list to find quietest channel */
    //while (i < psMlmeInd->uParam.sDcfmScan.u8ResultListSize)
    //{
    //    if ((psMlmeInd->uParam.sDcfmScan.uList.au8EnergyDetect[i]) < u8MinEnergy)
    //    {
	//		u8MinEnergy = (psMlmeInd->uParam.sDcfmScan.uList.au8EnergyDetect[i]);
	//		sCoordinatorData.u8Channel = i + CHANNEL_MIN;
	//	}
	//	i++;
    //}
    vStartCoordinator();
}

/****************************************************************************
 *
 * NAME: vStartCoordinator
 *
 * DESCRIPTION:
 * Starts the network by configuring the controller board to act as the PAN
 * coordinator.
 *
 * PARAMETERS:      Name            RW  Usage
 * None.
 *
 * RETURNS:
 * TRUE if network was started successfully otherwise FALSE
 *
 * NOTES:
 * None.
 ****************************************************************************/
PRIVATE void vStartCoordinator(void)
{
    /* Structures used to hold data for MLME request and response */
    MAC_MlmeReqRsp_s   sMlmeReqRsp;
    MAC_MlmeSyncCfm_s  sMlmeSyncCfm;

    sCoordinatorData.eState = E_STATE_COORDINATOR_STARTED;

    /* Start Pan */
    sMlmeReqRsp.u8Type = MAC_MLME_REQ_START;
    sMlmeReqRsp.u8ParamLength = sizeof(MAC_MlmeReqStart_s);
    sMlmeReqRsp.uParam.sReqStart.u16PanId = PAN_ID;
    sMlmeReqRsp.uParam.sReqStart.u8Channel = sCoordinatorData.u8Channel;
    sMlmeReqRsp.uParam.sReqStart.u8BeaconOrder = 0x0F;
    sMlmeReqRsp.uParam.sReqStart.u8SuperframeOrder = 0x0F;
    sMlmeReqRsp.uParam.sReqStart.u8PanCoordinator = TRUE;
    sMlmeReqRsp.uParam.sReqStart.u8BatteryLifeExt = FALSE;
    sMlmeReqRsp.uParam.sReqStart.u8Realignment = FALSE;
    sMlmeReqRsp.uParam.sReqStart.u8SecurityEnable = FALSE;

    vAppApiMlmeRequest(&sMlmeReqRsp, &sMlmeSyncCfm);
}

PRIVATE void lcd_BuildStatusScreen(void)
{
    #ifdef DEBUG_LCD
        vPrintf("lcd_BuildStatusScreen\n");
    #endif
    vLcdClear();
    vLcdWriteText("Esten Rye", 0, 0);
    vLcdWriteTextRightJustified("SEIS 740", 0, 127);
    vLcdWriteText("TOF Triangulation", 1, 0);
    vLcdWriteText("Node 0:", 2, 0);
    vLcdWriteTextRightJustified("Off", 2, 60);
    vLcdWriteText("Node 1:", 2, 64);
    vLcdWriteTextRightJustified("Off", 2, 123);
    vLcdWriteText("A:", 3, 0);
    vLcdWriteText("B:", 4, 0);
    vLcdWriteText("C:", 5, 0);
    vLcdWriteText("X:", 6, 0);
    vLcdWriteText("Y:", 7, 0);
    lcd_UpdateStatusScreen();
}

PRIVATE uint32 GetDistance(uint16 iEndDevice)
{
    uint32 distance;
    if (sCoordinatorData.sEndDeviceData[iEndDevice].i32TofDistance < 500)
    
    {
        distance = sCoordinatorData.sEndDeviceData[iEndDevice].u32RssiDistance;
    }
    else
    {
        distance = sCoordinatorData.sEndDeviceData[iEndDevice].i32TofDistance;
    }
    return distance;
}

PRIVATE void lcd_UpdateStatusScreen(void)
{
    #ifdef DEBUG_LCD
        vPrintf("lcd_UpdateStatusScreen\n");
    #endif
    bool_t beacon0Assigned = sCoordinatorData.sEndDeviceData[0].bIsAssociated;
    bool_t beacon1Assigned = sCoordinatorData.sEndDeviceData[1].bIsAssociated;

    #ifdef DEBUG_LCD
        vPrintf("Beacon 0 Associated: %i\n", beacon0Assigned);
        vPrintf("Beacon 1 Associated: %i\n", beacon1Assigned);
    #endif

    if (beacon0Assigned)
    {
        vLcdWriteTextRightJustified(" On", 2, 60);
    }
    else
    {
        vLcdWriteTextRightJustified("Off", 2, 60);
    }

    if (beacon1Assigned)
    {
        vLcdWriteTextRightJustified(" On", 2, 123);
    }
    else
    {
        vLcdWriteTextRightJustified("Off", 2, 123);
    }

    char output[20];
    
    intToStr(GetDistance(0), output, 0);
    vLcdWriteTextRightJustified(output, 3, 127);
    intToStr(GetDistance(1), output, 0);
    vLcdWriteTextRightJustified(output, 4, 127);
    vLcdWriteTextRightJustified("120", 5, 127);
    intToStr((int)sCoordinatorData.x, output, 0);
    vLcdWriteTextRightJustified(output, 6, 127);
    intToStr((int)sCoordinatorData.y, output, 0);
    vLcdWriteTextRightJustified(output, 7, 127);
    vLcdRefreshAll();
}

PRIVATE void vPutChar(unsigned char c) {
	while ((u8AHI_UartReadLineStatus(UART) & E_AHI_UART_LS_THRE) == 0);
	vAHI_UartWriteData(UART, c);
    while ((u8AHI_UartReadLineStatus(UART) & (E_AHI_UART_LS_THRE | E_AHI_UART_LS_TEMT)) != (E_AHI_UART_LS_THRE | E_AHI_UART_LS_TEMT));
}

// reverses a string 'str' of length 'len'
// retrieved from: https://www.geeksforgeeks.org/convert-floating-point-number-string/
void reverse(char *str, int len)
{
    int i=0, j=len-1, temp;
    while (i<j)
    {
        temp = str[i];
        str[i] = str[j];
        str[j] = temp;
        i++; j--;
    }
}
 
 // Converts a given integer x to string str[].  d is the number
 // of digits required in output. If d is more than the number
 // of digits in x, then 0s are added at the beginning.
 // retrieved from https://www.geeksforgeeks.org/convert-floating-point-number-string/
 // modified to support uint32 numbers.
int intToStr(uint32 x, char str[], int d)
{
    int i = 0;
    while (x)
    {
        str[i++] = (x%10) + '0';
        x = x/10;
    }
 
    // If number of digits required is more, then
    // add 0s at the beginning
    while (i < d)
    {
        str[i++] = '0';
    }
    
    reverse(str, i);
    str[i] = '\0';
    return i;
}

PRIVATE void task_CalculateXYPos(void)
{
    int32 a = (int32)GetDistance(0);
    int32 b = (int32)GetDistance(1);
    int32 c = (int32)120;
    vPrintf("Calculate XY Position\nA: %i\nB: %i\nC: %i\n", a, b, c);
    if (a > 0 && b > 0)
    {
        int32 s = (a + b + c) / 2;
        int32 n = s * (s-a) * (s-b) * (s-c);
        double y = 2 * sqrt(n) / c;
        double x = sqrt(pow(a, 2) - pow(y, 2));
        vPrintf("N: %i\nS: %i\nX: %i\nY: %i\n", n, s, (int)x, (int)y);
        sCoordinatorData.y = y;
        sCoordinatorData.x = x;
    }
}
/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/

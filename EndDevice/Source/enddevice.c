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
#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
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
#include "Printf.h"
#include <Math.h>
#include <LedControl.h>

#include "config.h"

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/

#define MAX_READINGS     20
#define UART             E_AHI_UART_0

/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/
typedef enum
{
	E_STATE_IDLE,
	E_STATE_ACTIVE_SCANNING,
	E_STATE_ASSOCIATING,
	E_STATE_ASSOCIATED
} teState;

typedef struct
{
	teState eState;
	uint8   u8Channel;
	uint8   u8TxPacketSeqNb;
	uint8   u8RxPacketSeqNb;
	uint16  u16Address;
} tsEndDeviceData;

/****************************************************************************/
/***        Local Function Prototypes                                     ***/
/****************************************************************************/
PRIVATE void vInitSystem(void);
PRIVATE void vProcessEventQueues(void);
PRIVATE void vProcessIncomingMlme(MAC_MlmeDcfmInd_s *psMlmeInd);
PRIVATE void vProcessIncomingMcps(MAC_McpsDcfmInd_s *psMcpsInd);
PRIVATE void vProcessIncomingHwEvent(AppQApiHwInd_s *psAHI_Ind);
PRIVATE void vStartActiveScan(uint32 u32ChannelstoScan);
PRIVATE void vHandleActiveScanResponse(MAC_MlmeDcfmInd_s *psMlmeInd);
PRIVATE void vStartAssociate(void);
PRIVATE void vHandleAssociateResponse(MAC_MlmeDcfmInd_s *psMlmeInd);
PRIVATE void vHandleMcpsDataInd(MAC_McpsDcfmInd_s *psMcpsInd);
PRIVATE void vHandleMcpsDataDcfm(MAC_McpsDcfmInd_s *psMcpsInd);
PRIVATE void vProcessReceivedDataPacket(uint8 *pu8Data, uint8 u8Len);
PRIVATE void vPutChar(unsigned char c);
PRIVATE void tx_Distance(int32 i32TofDistance, uint32 u32RssiDistance);

/****************************************************************************/
/***        Exported Variables                                            ***/
/****************************************************************************/

/****************************************************************************/
/***        Local Variables                                               ***/
/****************************************************************************/
/* Handles from the MAC */
PRIVATE void *s_pvMac;
PRIVATE MAC_Pib_s *s_psMacPib;
PRIVATE tsEndDeviceData sEndDeviceData;

PRIVATE bool_t bLedState;
PRIVATE uint32 u32Tick = 0;
PRIVATE uint8 u8CurrentTxHandle = 0x00;

eTofReturn eTofStatus = -1;
volatile bool_t bTofInProgress = FALSE;
tsAppApiTof_Data asTofData[MAX_READINGS];

/* RSSI to Distance (cm) lookup table. Generated from formula in JN-UG-3063 */
uint32 au32RSSIdistance[] = { 502377, 447744, 399052, 355656, 316979, 282508,
		251785, 224404, 200000, 178250, 158866, 141589, 126191, 112468, 100237,
		89337, 79621, 70963, 63246, 56368, 50238, 44774, 39905, 35566, 31698,
		28251, 25179, 22440, 20000, 17825, 15887, 14159, 12619, 11247, 10024,
		8934, 7962, 7096, 6325, 5637, 5024, 4477, 3991, 3557, 3170, 2825, 2518,
		2244, 2000, 1783, 1589, 1416, 1262, 1125, 1002, 893, 796, 710, 632,
		564, 502, 448, 399, 356, 317, 283, 252, 224, 200, 178, 159, 142, 126,
		112, 100, 89, 80, 71, 63, 56, 50, 45, 40, 36, 32, 28, 25, 22, 20, 18,
		16, 14, 13, 11, 10, 9, 8, 7, 6, 6, 5, 4, 4, 4, 3, 3, 3, 2, 2 };

/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/

/****************************************************************************
 *
 * NAME: vTofCallback
 *
 * DESCRIPTION:
 * This function is passed to bAppApiGetTof. Function is called when the tof
 * readings have been completed and stored in asTofData.
 *
 * PASSED:
 * eTofReturn eStatus,
 *
 * RETURNS:
 * None
 *
 ****************************************************************************/
void vTofCallback(eTofReturn eStatus)
{
	eTofStatus = eStatus;
}

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
	int n;
	bool_t  bTofDirection = API_TOF_FORWARDS;
	int32 s32Mean, s32StanDev, i32TofDistance;
	uint32 u32RssiDistance;
	double dStd, dMean;
	uint8  u8NumErrors, u8NumSuccessfullTofs=0;

#ifdef WATCHDOG_ENABLED
	vAHI_WatchdogStop();
#endif

	vAHI_UartEnable(UART);
	vAHI_UartReset(UART, TRUE, TRUE);
	vAHI_UartSetClockDivisor(UART, E_AHI_UART_RATE_115200);
	vAHI_UartReset(UART, FALSE, FALSE);
	vInitPrintf((void *)vPutChar);

	/* Clear screen and tabs */
	vPrintf("\x1B[2J\x1B[H\x1B[3g");
	vPrintf("JN5148 Time of Flight Ranging Demo\n");

	for(n = 0; n < MAX_READINGS; n++)
	{
		asTofData[n].s32Tof       = 0;
		asTofData[n].s8LocalRSSI  = 0;
		asTofData[n].u8LocalSQI   = 0;
		asTofData[n].s8RemoteRSSI = 0;
		asTofData[n].u8RemoteSQI  = 0;
		asTofData[n].u32Timestamp = 0;
		asTofData[n].u8Status     = 0;
	}

	vInitSystem();

	/* Enable TOF ranging. */
	vAppApiTofInit(TRUE);

	vPrintf("Starting Scan\n");
	vStartActiveScan(SCAN_CHANNELS);

	/* Create address for coordinator */
	MAC_Addr_s sAddr;
	sAddr.u8AddrMode     = 2;
	sAddr.u16PanId       = PAN_ID;
	sAddr.uAddr.u16Short = COORDINATOR_ADR;

	vLedInitRfd();

	while (1)
	{
		if (u32Tick++ > (bTofInProgress ? 10000 : 100000))
		{
			bLedState = !bLedState;
			vLedControl(0, bLedState);
			u32Tick = 0;
		}

		if (sEndDeviceData.eState >= E_STATE_ASSOCIATED)
		{
			if(bTofInProgress==FALSE)
			{
				if (bAppApiGetTof( asTofData, &sAddr, MAX_READINGS, API_TOF_FORWARDS, vTofCallback))
				{
					vPrintf("\nForward burst started");
					bTofInProgress = TRUE;
					bTofDirection = API_TOF_FORWARDS;
				} else {
					vPrintf("\nFailed to start ToF");
				}
			}

			/* Check for return code to have been set in callback */
			if (eTofStatus != -1)
			{
				if (eTofStatus == TOF_SUCCESS)
				{
					u8NumSuccessfullTofs++;

					double dAcc = 0.0;
					u32RssiDistance = 0;

					u8NumErrors = 0;

					vPrintf("\n\n| #  \x1BH| ToF (ps) \x1BH| Lcl RSSI \x1BH| Lcl SQI \x1BH| Rmt RSSI \x1BH| Rmt SQI \x1BH| Timestamp \x1BH| Status \x1BH|");
					vPrintf("\n--------------------------------------------------------------------------------");

					for(n = 0; n < MAX_READINGS; n++)
					{
						vPrintf("\n|%d",n);

						/* Only include successful readings */
						if (asTofData[n].u8Status == MAC_TOF_STATUS_SUCCESS)
						{
							dAcc += asTofData[n].s32Tof;
							u32RssiDistance += au32RSSIdistance[asTofData[n].s8LocalRSSI];
							u32RssiDistance += au32RSSIdistance[asTofData[n].s8RemoteRSSI];

							vPrintf("\t|%i\t|%d\t|%d\t|%d\t|%d\t|%d\t|%d\t|",
									asTofData[n].s32Tof,
									asTofData[n].s8LocalRSSI,
									asTofData[n].u8LocalSQI,
									asTofData[n].s8RemoteRSSI,
									asTofData[n].u8RemoteSQI,
									asTofData[n].u32Timestamp,
									asTofData[n].u8Status);
						}
						else
						{
							u8NumErrors++;

							vPrintf("\t|-\t|-\t|-\t|-\t|-\t|-\t|%d\t|",
									asTofData[n].u8Status);
						}
					}

					/* Calculate statistics */
					if(u8NumErrors != MAX_READINGS)
					{
						dMean = dAcc / (MAX_READINGS - u8NumErrors);

						/* Calculate standard deviation = sqrt((1/N)*(sigma(xi-xmean)2) */
						dStd = 0.0;

						/* Accumulate sum of squared deviances */
						for(n = 0; n < MAX_READINGS; n++)
						{
							if(asTofData[n].u8Status == MAC_TOF_STATUS_SUCCESS)
							{
								dStd += ((double)asTofData[n].s32Tof - dMean) * ((double)asTofData[n].s32Tof - dMean);
							}
						}

						/* std = sqrt(mean of sum of squared deviances) */
						dStd /= (MAX_READINGS - u8NumErrors);
						dStd = sqrt(dStd);


						s32StanDev = (int32)dStd;
						s32Mean    = (int32)dMean;

						/* Calculate distances */
						i32TofDistance  = dMean * 0.03;
						u32RssiDistance /= (MAX_READINGS - u8NumErrors) * 2;
					}
					else
					{
						s32StanDev      = 0;
						s32Mean         = 0;
						i32TofDistance  = 0;
						u32RssiDistance = 0;
					}

					vPrintf("\n\nStandDev (ToF): %ips, Mean (ToF): %ips, Errors: %d",
							s32StanDev,
							s32Mean,
							u8NumErrors);

					vPrintf("\nDistance (ToF): %icm, Distance (RSSI): %dcm",
							i32TofDistance,
							u32RssiDistance);
					tx_Distance(i32TofDistance, u32RssiDistance);
				}
				else
				{
					vPrintf("\nToF failed with error %d", eTofStatus);
				}

				/* Reset flags for next ToF burst */
				eTofStatus = -1;
				bTofInProgress = FALSE;
			}
		}

		vProcessEventQueues();

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
	vAHI_HighPowerModuleEnable(TRUE, TRUE);

	/* Initialise end device state */
	sEndDeviceData.eState = E_STATE_IDLE;
	sEndDeviceData.u8TxPacketSeqNb = 0;
	sEndDeviceData.u8RxPacketSeqNb = 0;

	/* Set up the MAC handles. Must be called AFTER u32AppQApiInit() */
	s_pvMac = pvAppApiGetMacHandle();
	s_psMacPib = MAC_psPibGetHandle(s_pvMac);

	/* Set Pan ID in PIB (also sets match register in hardware) */
	MAC_vPibSetPanId(s_pvMac, PAN_ID);

	/* Enable receiver to be on when idle */
	MAC_vPibSetRxOnWhenIdle(s_pvMac, TRUE, FALSE);

	vPrintf("Done Init\n");
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
	/* We respond to several MLME indications and confirmations, depending
       on mode */
	switch (psMlmeInd->u8Type)
	{
	/* Deferred confirmation that the scan is complete */
	case MAC_MLME_DCFM_SCAN:
		if (sEndDeviceData.eState == E_STATE_ACTIVE_SCANNING)
		{
			vHandleActiveScanResponse(psMlmeInd);
		}
		break;

		/* Deferred confirmation that the association process is complete */
	case MAC_MLME_DCFM_ASSOCIATE:
		/* Only respond to this if associating */
		if (sEndDeviceData.eState == E_STATE_ASSOCIATING)
		{
			vHandleAssociateResponse(psMlmeInd);
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
	if (sEndDeviceData.eState >= E_STATE_ASSOCIATED)
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
		/* Data transmission failed after 3 retries at MAC layer. */
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

	if (psFrame->sSrcAddr.uAddr.u16Short == COORDINATOR_ADR)
	{
		if (psFrame->au8Sdu[0] >= sEndDeviceData.u8RxPacketSeqNb)
		{
			sEndDeviceData.u8RxPacketSeqNb++;
			vProcessReceivedDataPacket(&psFrame->au8Sdu[1],
					(psFrame->u8SduLength) - 1);
		}
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
PRIVATE void vProcessReceivedDataPacket(uint8 *pu8Data, uint8 u8Len)
{
}

/****************************************************************************
 *
 * NAME: vStartAssociate
 *
 * DESCRIPTION:
 * Start the association process with the network coordinator.
 *
 * PARAMETERS:      Name            RW  Usage
 * None.
 *
 * RETURNS:
 * None.
 *
 * NOTES:
 * Assumes that a network has been found during the network scan.
 ****************************************************************************/
PRIVATE void vStartAssociate(void)
{
	MAC_MlmeReqRsp_s  sMlmeReqRsp;
	MAC_MlmeSyncCfm_s sMlmeSyncCfm;

	sEndDeviceData.eState = E_STATE_ASSOCIATING;

	/* Create associate request. We know short address and PAN ID of
       coordinator as this is preset and we have checked that received
       beacon matched this */

	sMlmeReqRsp.u8Type = MAC_MLME_REQ_ASSOCIATE;
	sMlmeReqRsp.u8ParamLength = sizeof(MAC_MlmeReqAssociate_s);
	sMlmeReqRsp.uParam.sReqAssociate.u8LogicalChan = sEndDeviceData.u8Channel;
	sMlmeReqRsp.uParam.sReqAssociate.u8Capability = 0x80; /* We want short address, other features off */
	sMlmeReqRsp.uParam.sReqAssociate.u8SecurityEnable = FALSE;
	sMlmeReqRsp.uParam.sReqAssociate.sCoord.u8AddrMode = 2;
	sMlmeReqRsp.uParam.sReqAssociate.sCoord.u16PanId = PAN_ID;
	sMlmeReqRsp.uParam.sReqAssociate.sCoord.uAddr.u16Short = COORDINATOR_ADR;

	/* Put in associate request and check immediate confirm. Should be
       deferred, in which case response is handled by event handler */
	vAppApiMlmeRequest(&sMlmeReqRsp, &sMlmeSyncCfm);
}




/****************************************************************************
 *
 * NAME: vHandleAssociateResponse
 *
 * DESCRIPTION:
 * Handle the response generated by the stack as a result of the associate
 * start request.
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
PRIVATE void vHandleAssociateResponse(MAC_MlmeDcfmInd_s *psMlmeInd)
{
	/* If successfully associated with network coordinator */
	if (psMlmeInd->uParam.sDcfmAssociate.u8Status == MAC_ENUM_SUCCESS)
	{
		vPrintf("Associated");
		sEndDeviceData.u16Address = psMlmeInd->uParam.sDcfmAssociate.u16AssocShortAddr;
		sEndDeviceData.eState = E_STATE_ASSOCIATED;
	}
	else
	{
		vStartActiveScan(SCAN_CHANNELS);
	}
}

/****************************************************************************
 *
 * NAME: vStartActiveScan
 *
 * DESCRIPTION:
 * Start a scan to search for a network to join.
 *
 * PARAMETERS:      Name            RW  Usage
 * 					u32ChannelstoScan
 *
 * RETURNS:
 * None.
 *
 * NOTES:
 * updated 24_7_08, fixes not scanning high channels problem (sbarf)
 ****************************************************************************/
PRIVATE void vStartActiveScan(uint32 u32ChannelstoScan)
{
	MAC_MlmeReqRsp_s  sMlmeReqRsp;
	MAC_MlmeSyncCfm_s sMlmeSyncCfm;

	sEndDeviceData.eState = E_STATE_ACTIVE_SCANNING;

	/* Request scan */
	sMlmeReqRsp.u8Type = MAC_MLME_REQ_SCAN;
	sMlmeReqRsp.u8ParamLength = sizeof(MAC_MlmeReqScan_s);
	sMlmeReqRsp.uParam.sReqScan.u8ScanType = MAC_MLME_SCAN_TYPE_ACTIVE;
	sMlmeReqRsp.uParam.sReqScan.u32ScanChannels = u32ChannelstoScan;

	sMlmeReqRsp.uParam.sReqScan.u8ScanDuration = 3;

	vAppApiMlmeRequest(&sMlmeReqRsp, &sMlmeSyncCfm);
}

/****************************************************************************
 *
 * NAME: vHandleActiveScanResponse
 *
 * DESCRIPTION:
 * Handle the reponse generated by the stack as a result of the network scan.
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
PRIVATE void vHandleActiveScanResponse(MAC_MlmeDcfmInd_s *psMlmeInd)
{
	MAC_PanDescr_s *psPanDesc;
	uint8 i;

	if (psMlmeInd->uParam.sDcfmScan.u8ScanType == MAC_MLME_SCAN_TYPE_ACTIVE)
	{
		if (psMlmeInd->uParam.sDcfmScan.u8Status == MAC_ENUM_SUCCESS)
		{
			i = 0;

			while (i < psMlmeInd->uParam.sDcfmScan.u8ResultListSize)
			{
				psPanDesc = &psMlmeInd->uParam.sDcfmScan.uList.asPanDescr[i];

				if ((psPanDesc->sCoord.u16PanId == PAN_ID)
						&& (psPanDesc->sCoord.u8AddrMode == 2)
						&& (psPanDesc->sCoord.uAddr.u16Short == COORDINATOR_ADR)
						/* Check it is accepting association requests */
						&& (psPanDesc->u16SuperframeSpec & 0x8000))
				{
					sEndDeviceData.u8Channel = psPanDesc->u8LogicalChan;
					vStartAssociate();
					return;
				}
				i++;
			}
		}
	}
	//sbarf: updated section 24_7_08
	// if there are remaining unscanned channels left to scan, scan them

	if(psMlmeInd->uParam.sDcfmScan.u32UnscannedChannels != 0)
		vStartActiveScan(psMlmeInd->uParam.sDcfmScan.u32UnscannedChannels);
	else
		/* Failed to find coordinator: keep trying */
		vStartActiveScan(SCAN_CHANNELS);

}

PRIVATE void vPutChar(unsigned char c) {
	while ((u8AHI_UartReadLineStatus(UART) & E_AHI_UART_LS_THRE) == 0);
	vAHI_UartWriteData(UART, c);
    while ((u8AHI_UartReadLineStatus(UART) & (E_AHI_UART_LS_THRE | E_AHI_UART_LS_TEMT)) != (E_AHI_UART_LS_THRE | E_AHI_UART_LS_TEMT));
}


PRIVATE void tx_Distance(int32 i32TofDistance, uint32 u32RssiDistance)
{
	/* Structures used to hold data for MLME request and response */
	MAC_McpsReqRsp_s sMcpsReqRsp;
	MAC_McpsSyncCfm_s sMcpsSyncCfm;
	uint8 *pu8Payload;

	/* Create frame transmission request */
	sMcpsReqRsp.u8Type = MAC_MCPS_REQ_DATA;
	sMcpsReqRsp.u8ParamLength = sizeof(MAC_McpsReqData_s);

	/* Set handle so we can match confirmation to request */
	sMcpsReqRsp.uParam.sReqData.u8Handle = u8CurrentTxHandle;
	u8CurrentTxHandle +=1;

	/* Use short address for source */
	sMcpsReqRsp.uParam.sReqData.sFrame.sSrcAddr.u8AddrMode = 2;
	sMcpsReqRsp.uParam.sReqData.sFrame.sSrcAddr.u16PanId = PAN_ID;
	sMcpsReqRsp.uParam.sReqData.sFrame.sSrcAddr.uAddr.u16Short = sEndDeviceData.u16Address;

	/* Use short address for destination */
	sMcpsReqRsp.uParam.sReqData.sFrame.sDstAddr.u8AddrMode = 2;
	sMcpsReqRsp.uParam.sReqData.sFrame.sDstAddr.u16PanId = PAN_ID;
	sMcpsReqRsp.uParam.sReqData.sFrame.sDstAddr.uAddr.u16Short = COORDINATOR_ADR;

	/* Frame requires ack but not security, indirect transmit or GTS */
	sMcpsReqRsp.uParam.sReqData.sFrame.u8TxOptions = MAC_TX_OPTION_ACK;

	/* Set payload, only use first 8 bytes */
	sMcpsReqRsp.uParam.sReqData.sFrame.u8SduLength = 10;
	pu8Payload = sMcpsReqRsp.uParam.sReqData.sFrame.au8Sdu;
	vPrintf("\nTransmitting Distance to Coordinator\n");

	pu8Payload[0] = sEndDeviceData.u8TxPacketSeqNb++;
	pu8Payload[1] = (uint8)(0xd1);
	pu8Payload[2] = (uint8)(((uint32)i32TofDistance & 0xff000000uL) >> 24);
	pu8Payload[3] = (uint8)(((uint32)i32TofDistance & 0x00ff0000uL) >> 16);
	pu8Payload[4] = (uint8)(((uint32)i32TofDistance & 0x0000ff00uL) >> 8);
	pu8Payload[5] = (uint8)((uint32)i32TofDistance & 0x000000ffuL);
	pu8Payload[6] = (uint8)((u32RssiDistance & 0xff000000uL) >> 24);
	pu8Payload[7] = (uint8)((u32RssiDistance & 0x00ff0000uL) >> 16);
	pu8Payload[8] = (uint8)((u32RssiDistance & 0x0000ff00uL) >> 8);
	pu8Payload[9] = (uint8)(u32RssiDistance & 0x000000ffuL);

	#ifdef DEBUG_DISTANCE_TRANSMISSION
		vPrintf("TOF  Byte0: "BYTE_TO_BINARY_PATTERN"\n", BYTE_TO_BINARY(pu8Payload[2]));
		vPrintf("TOF  Byte1: "BYTE_TO_BINARY_PATTERN"\n", BYTE_TO_BINARY(pu8Payload[3]));
		vPrintf("TOF  Byte2: "BYTE_TO_BINARY_PATTERN"\n", BYTE_TO_BINARY(pu8Payload[4]));
		vPrintf("TOF  Byte3: "BYTE_TO_BINARY_PATTERN"\n\n", BYTE_TO_BINARY(pu8Payload[5]));
		vPrintf("RSSI Byte0: "BYTE_TO_BINARY_PATTERN"\n", BYTE_TO_BINARY(pu8Payload[6]));
		vPrintf("RSSI Byte1: "BYTE_TO_BINARY_PATTERN"\n", BYTE_TO_BINARY(pu8Payload[7]));
		vPrintf("RSSI Byte2: "BYTE_TO_BINARY_PATTERN"\n", BYTE_TO_BINARY(pu8Payload[8]));
		vPrintf("RSSI Byte3: "BYTE_TO_BINARY_PATTERN"\n\n", BYTE_TO_BINARY(pu8Payload[9]));
	#endif

	vAppApiMcpsRequest(&sMcpsReqRsp, &sMcpsSyncCfm);
}
/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/

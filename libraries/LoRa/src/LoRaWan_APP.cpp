#include <LoRaWan_APP.h>

#if(LoraWan_RGB==1)
#include "Adafruit_NeoPixel.h"
Adafruit_NeoPixel pixels(1, RGB, NEO_GRB + NEO_KHZ800);
#endif

#if REGION_EU868
#include "RegionEU868.h"
#endif

#if REGION_EU433
#include "RegionEU433.h"
#endif

/*!
 * Default datarate
 */
#define LORAWAN_DEFAULT_DATARATE                    DR_5

uint8_t DevEui[] = LORAWAN_DEVICE_EUI;
uint8_t AppEui[] = LORAWAN_APPLICATION_EUI;
uint8_t AppKey[] = LORAWAN_APPLICATION_KEY;

uint8_t NwkSKey[] = LORAWAN_NWKSKEY;
uint8_t AppSKey[] = LORAWAN_APPSKEY;
uint32_t DevAddr = LORAWAN_DEVICE_ADDRESS;

/*!
 *
 */
typedef enum
{
   LORAMAC_HANDLER_UNCONFIRMED_MSG = 0,
   LORAMAC_HANDLER_CONFIRMED_MSG = !LORAMAC_HANDLER_UNCONFIRMED_MSG
}LoRaMacHandlerMsgTypes_t;

/*!
 * Application data structure
 */
typedef struct LoRaMacHandlerAppData_s
{
   LoRaMacHandlerMsgTypes_t MsgType;
   uint8_t Port;
   uint8_t BufferSize;
   uint8_t *Buffer;
}LoRaMacHandlerAppData_t;

//LoRaMacHandlerAppData_t AppData = 
//{
//    .MsgType = LORAMAC_HANDLER_UNCONFIRMED_MSG,
//    .Buffer = NULL,
//    .BufferSize = 0,
//    .Port = 0
//};
LoRaMacHandlerAppData_t AppData;


/*!
 * User application data size
 */
uint8_t AppDataSize = 4;

/*!
 * User application data
 */
uint8_t AppDataBuffer[LORAWAN_APP_DATA_MAX_SIZE];


/*!
 * Defines the application data transmission duty cycle
 */
uint32_t TxDutyCycleTime ;

/*!
 * Timer to handle the application data transmission duty cycle
 */
static TimerEvent_t TxNextPacketTimer;

/*!
 * PassthroughMode mode enable/disable. don't modify it here. 
 * when use PassthroughMode, set it true in app.ino , Reference the example PassthroughMode.ino 
 */
bool PassthroughMode = false;

/*!
 * when use PassthroughMode, Mode_LoraWan to set use lora or lorawan mode . don't modify it here. 
 * it is used to set mode lora/lorawan in PassthroughMode.
 */
bool Mode_LoraWan = true;

/*!
 * Indicates if LoRaMacProcess call is pending.
 * 
 * \warning If variable is equal to 0 then the MCU can be set in low power mode
 */
static uint8_t IsMacProcessPending = 0;

/*!
 * Indicates if a new packet can be sent
 */
static bool NextTx = true;

/*!
 * MAC status strings
 */
const char* MacStatusStrings[] =
{
    "OK",                            // LORAMAC_STATUS_OK
    "Busy",                          // LORAMAC_STATUS_BUSY
    "Service unknown",               // LORAMAC_STATUS_SERVICE_UNKNOWN
    "Parameter invalid",             // LORAMAC_STATUS_PARAMETER_INVALID
    "Frequency invalid",             // LORAMAC_STATUS_FREQUENCY_INVALID
    "Datarate invalid",              // LORAMAC_STATUS_DATARATE_INVALID
    "Frequency or datarate invalid", // LORAMAC_STATUS_FREQ_AND_DR_INVALID
    "No network joined",             // LORAMAC_STATUS_NO_NETWORK_JOINED
    "Length error",                  // LORAMAC_STATUS_LENGTH_ERROR
    "Region not supported",          // LORAMAC_STATUS_REGION_NOT_SUPPORTED
    "Skipped APP data",              // LORAMAC_STATUS_SKIPPED_APP_DATA
    "Duty-cycle restricted",         // LORAMAC_STATUS_DUTYCYCLE_RESTRICTED
    "No channel found",              // LORAMAC_STATUS_NO_CHANNEL_FOUND
    "No free channel found",         // LORAMAC_STATUS_NO_FREE_CHANNEL_FOUND
    "Busy beacon reserved time",     // LORAMAC_STATUS_BUSY_BEACON_RESERVED_TIME
    "Busy ping-slot window time",    // LORAMAC_STATUS_BUSY_PING_SLOT_WINDOW_TIME
    "Busy uplink collision",         // LORAMAC_STATUS_BUSY_UPLINK_COLLISION
    "Crypto error",                  // LORAMAC_STATUS_CRYPTO_ERROR
    "FCnt handler error",            // LORAMAC_STATUS_FCNT_HANDLER_ERROR
    "MAC command error",             // LORAMAC_STATUS_MAC_COMMAD_ERROR
    "ClassB error",                  // LORAMAC_STATUS_CLASS_B_ERROR
    "Confirm queue error",           // LORAMAC_STATUS_CONFIRM_QUEUE_ERROR
    "Multicast group undefined",     // LORAMAC_STATUS_MC_GROUP_UNDEFINED
    "Unknown error",                 // LORAMAC_STATUS_ERROR
};
/*!
 * MAC event info status strings.
 */
const char* EventInfoStatusStrings[] =
{ 
    "OK",                            // LORAMAC_EVENT_INFO_STATUS_OK
    "Error",                         // LORAMAC_EVENT_INFO_STATUS_ERROR
    "Tx timeout",                    // LORAMAC_EVENT_INFO_STATUS_TX_TIMEOUT
    "Rx 1 timeout",                  // LORAMAC_EVENT_INFO_STATUS_RX1_TIMEOUT
    "Rx 2 timeout",                  // LORAMAC_EVENT_INFO_STATUS_RX2_TIMEOUT
    "Rx1 error",                     // LORAMAC_EVENT_INFO_STATUS_RX1_ERROR
    "Rx2 error",                     // LORAMAC_EVENT_INFO_STATUS_RX2_ERROR
    "Join failed",                   // LORAMAC_EVENT_INFO_STATUS_JOIN_FAIL
    "Downlink repeated",             // LORAMAC_EVENT_INFO_STATUS_DOWNLINK_REPEATED
    "Tx DR payload size error",      // LORAMAC_EVENT_INFO_STATUS_TX_DR_PAYLOAD_SIZE_ERROR
    "Downlink too many frames loss", // LORAMAC_EVENT_INFO_STATUS_DOWNLINK_TOO_MANY_FRAMES_LOSS
    "Address fail",                  // LORAMAC_EVENT_INFO_STATUS_ADDRESS_FAIL
    "MIC fail",                      // LORAMAC_EVENT_INFO_STATUS_MIC_FAIL
    "Multicast fail",                // LORAMAC_EVENT_INFO_STATUS_MULTICAST_FAIL
    "Beacon locked",                 // LORAMAC_EVENT_INFO_STATUS_BEACON_LOCKED
    "Beacon lost",                   // LORAMAC_EVENT_INFO_STATUS_BEACON_LOST
    "Beacon not found"               // LORAMAC_EVENT_INFO_STATUS_BEACON_NOT_FOUND
};

enum eDeviceState_LoraWan DeviceState;









/*!
 * Executes the network Join request
 */
static void JoinNetwork( void )
{
    LoRaMacStatus_t status;
    MlmeReq_t mlmeReq;
    mlmeReq.Type = MLME_JOIN;
    mlmeReq.Req.Join.Datarate = LORAWAN_DEFAULT_DATARATE;

    // Starts the join procedure
    status = LoRaMacMlmeRequest( &mlmeReq );
    printf( "\r\n###### ===== MLME-Request - MLME_JOIN ==== ######\r\n" );
    printf( "STATUS      : %s\r\n", MacStatusStrings[status] );

    if( status == LORAMAC_STATUS_OK )
    {
        printf( "###### ===== JOINING ==== ######\r\n" );
        DeviceState = DEVICE_STATE_SLEEP;
    }
    else
    {
        DeviceState = DEVICE_STATE_CYCLE;
    }
}

/*!
 * \brief   Prepares the payload of the frame
 *
 * \retval  [0: frame could be send, 1: error]
 */
bool SendFrame( void )
{
	McpsReq_t mcpsReq;
	LoRaMacTxInfo_t txInfo;

	if( LoRaMacQueryTxPossible( AppDataSize, &txInfo ) != LORAMAC_STATUS_OK )
	{
		// Send empty frame in order to flush MAC commands
		mcpsReq.Type = MCPS_UNCONFIRMED;
		mcpsReq.Req.Unconfirmed.fBuffer = NULL;
		mcpsReq.Req.Unconfirmed.fBufferSize = 0;
		mcpsReq.Req.Unconfirmed.Datarate = LORAWAN_DEFAULT_DATARATE;
	}
	else
	{
		if( IsTxConfirmed == false )
		{
			printf("unconfirmed uplink sending ...\r\n");
			mcpsReq.Type = MCPS_UNCONFIRMED;
			mcpsReq.Req.Unconfirmed.fPort = AppPort;
			mcpsReq.Req.Unconfirmed.fBuffer = AppDataBuffer;
			mcpsReq.Req.Unconfirmed.fBufferSize = AppDataSize;
			mcpsReq.Req.Unconfirmed.Datarate = LORAWAN_DEFAULT_DATARATE;
		}
		else
		{
			printf("confirmed uplink sending ...\r\n");
			mcpsReq.Type = MCPS_CONFIRMED;
			mcpsReq.Req.Confirmed.fPort = AppPort;
			mcpsReq.Req.Confirmed.fBuffer = AppDataBuffer;
			mcpsReq.Req.Confirmed.fBufferSize = AppDataSize;
			mcpsReq.Req.Confirmed.NbTrials = ConfirmedNbTrials;
			mcpsReq.Req.Confirmed.Datarate = LORAWAN_DEFAULT_DATARATE;
		}
	}

	// Update global variable
	AppData.MsgType = ( mcpsReq.Type == MCPS_CONFIRMED ) ? LORAMAC_HANDLER_CONFIRMED_MSG : LORAMAC_HANDLER_UNCONFIRMED_MSG;
	AppData.Port = mcpsReq.Req.Unconfirmed.fPort;
	AppData.Buffer = (uint8_t*)mcpsReq.Req.Unconfirmed.fBuffer;
	AppData.BufferSize = mcpsReq.Req.Unconfirmed.fBufferSize;

	LoRaMacStatus_t status;
	status = LoRaMacMcpsRequest( &mcpsReq );
	printf( "\r\n###### ===== MCPS-Request ==== ######\r\n" );
	printf( "STATUS      : %s\r\n", MacStatusStrings[status] );

	if( LoRaMacMcpsRequest( &mcpsReq ) == LORAMAC_STATUS_OK )
	{
		return false;
	}
	return true;
}

/*!
 * \brief Function executed on TxNextPacket Timeout event
 */
static void OnTxNextPacketTimerEvent( void* context )
{
	MibRequestConfirm_t mibReq;
	LoRaMacStatus_t status;

	TimerStop( &TxNextPacketTimer );

	mibReq.Type = MIB_NETWORK_ACTIVATION;
	status = LoRaMacMibGetRequestConfirm( &mibReq );

	if( status == LORAMAC_STATUS_OK )
	{
		if( mibReq.Param.NetworkActivation == ACTIVATION_TYPE_NONE )
		{
			// Network not joined yet. Try to join again
			JoinNetwork();
		}
		else
		{
			DeviceState = DEVICE_STATE_SEND;
			NextTx = true;
		}
	}
}

/*!
 * \brief   MCPS-Confirm event function
 *
 * \param   [IN] mcpsConfirm - Pointer to the confirm structure,
 *               containing confirm attributes.
 */
static void McpsConfirm( McpsConfirm_t *mcpsConfirm )
{
	printf( "\r\n###### ===== MCPS-Confirm ==== ######\r\n" );
	printf( "STATUS      : %s\r\n", EventInfoStatusStrings[mcpsConfirm->Status] );
	if( mcpsConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK )
	{
		switch( mcpsConfirm->McpsRequest )
		{
			case MCPS_UNCONFIRMED:
			{
				// Check Datarate
				// Check TxPower
				break;
			}
			case MCPS_CONFIRMED:
			{
				// Check Datarate
				// Check TxPower
				// Check AckReceived
				// Check NbTrials
				break;
			}
			case MCPS_PROPRIETARY:
			{
				break;
			}
			default:
				break;
		}
	}
	//NextTx = true;
}

#if(LoraWan_RGB==1)
void RGB_ON(uint32_t color,uint32_t time)
{
	uint8_t red,green,blue;
	red=(uint8_t)(color>>16);
	green=(uint8_t)(color>>8);
	blue=(uint8_t)color;
	pinMode(Vext,OUTPUT);
	digitalWrite(Vext,LOW); //SET POWER
	delay(1);
	pixels.begin(); // INITIALIZE RGB strip object (REQUIRED)
	pixels.clear(); // Set all pixel colors to 'off'
	pixels.setPixelColor(0, pixels.Color(red, green, blue));
	pixels.show();   // Send the updated pixel colors to the hardware.
	if(time>0)
	{
		delay(time);
	}
}

void RGB_OFF(void)
{
	RGB_ON(0,0);
	digitalWrite(Vext,HIGH);
}
#endif

/*  get the BatteryVoltage in mV. */
uint16_t GetBatteryVoltage(void)
{
	pinMode(ADC_CTL,OUTPUT);
	digitalWrite(ADC_CTL,LOW);
	uint16_t volt=analogRead(ADC)*2;
	digitalWrite(ADC_CTL,HIGH);
	return volt;
}


void __attribute__((weak)) DownLinkDataHandle(McpsIndication_t *mcpsIndication)
{
	printf("+REV DATA:%s,RXSIZE %d,PORT %d\r\n",mcpsIndication->RxSlot?"RXWIN2":"RXWIN1",mcpsIndication->BufferSize,mcpsIndication->Port);
	printf("+REV DATA:");
	for(uint8_t i=0;i<mcpsIndication->BufferSize;i++)
	{
		printf("%02X",mcpsIndication->Buffer[i]);
	}
	printf("\r\n");
}

/*!
 * \brief   MCPS-Indication event function
 *
 * \param   [IN] mcpsIndication - Pointer to the indication structure,
 *               containing indication attributes.
 */
static void McpsIndication( McpsIndication_t *mcpsIndication )
{
	printf( "\r\n###### ===== MCPS-Indication ==== ######\r\n" );
	printf( "STATUS      : %s\r\n", EventInfoStatusStrings[mcpsIndication->Status] );
	if( mcpsIndication->Status != LORAMAC_EVENT_INFO_STATUS_OK )
	{
		return;
	}

	printf( "receive data: rssi = %d, snr = %d, datarate = %d\r\n", mcpsIndication->Rssi, (int)mcpsIndication->Snr,(int)mcpsIndication->RxDatarate);

#if (LoraWan_RGB==1)
	RGB_ON(COLOR_RECEIVED, 200);
	RGB_OFF();
#endif

	switch( mcpsIndication->McpsIndication )
	{
		case MCPS_UNCONFIRMED:
		{
			break;
		}
		case MCPS_CONFIRMED:
		{
			break;
		}
		case MCPS_PROPRIETARY:
		{
			break;
		}
		case MCPS_MULTICAST:
		{
			break;
		}
		default:
			break;
	}

	// Check Multicast
	// Check Port
	// Check Datarate
	// Check FramePending
	if( mcpsIndication->FramePending == true )
	{
		// The server signals that it has pending data to be sent.
		// We schedule an uplink as soon as possible to flush the server.
		OnTxNextPacketTimerEvent(NULL);
	}
	// Check Buffer
	// Check BufferSize
	// Check Rssi
	// Check Snr
	// Check RxSlot
	if( mcpsIndication->RxData == true )
	{
		DownLinkDataHandle(mcpsIndication);
	}
}

/*!
 * \brief   MLME-Confirm event function
 *
 * \param   [IN] mlmeConfirm - Pointer to the confirm structure,
 *               containing confirm attributes.
 */
static void MlmeConfirm( MlmeConfirm_t *mlmeConfirm )
{
	printf( "\r\n###### ===== MLME-Confirm ==== ######\r\n" );
	printf( "STATUS      : %s\r\n", EventInfoStatusStrings[mlmeConfirm->Status] );
	
	switch( mlmeConfirm->MlmeRequest )
	{
		case MLME_JOIN:
		{
			if( mlmeConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK )
			{
#if (LoraWan_RGB==1)
				RGB_ON(COLOR_JOINED,500);
				RGB_OFF();
#endif
				MibRequestConfirm_t mibGet;
				printf( "###### ===== JOINED ==== ######\r\n" );
				printf( "\r\nOTAA\r\n\r\n" );

				mibGet.Type = MIB_DEV_ADDR;
				LoRaMacMibGetRequestConfirm( &mibGet );
				printf( "DevAddr     : %08lX\r\n", mibGet.Param.DevAddr );
				printf( "\n\r\n" );

				mibGet.Type = MIB_CHANNELS_DATARATE;
				LoRaMacMibGetRequestConfirm( &mibGet );
				printf( "DATA RATE   : DR_%d\r\n", mibGet.Param.ChannelsDatarate );
				printf( "\r\n" );
				// Status is OK, node has joined the network
				
				//in PassthroughMode,do nothing while joined
				if(PassthroughMode == false)
				{
					// Status is OK, node has joined the network
					DeviceState = DEVICE_STATE_SEND;
				}
			}
			else
			{
				uint32_t rejoin_delay = 30000;
				printf("join failed\r\n");
				TimerSetValue( &TxNextPacketTimer, rejoin_delay );
				TimerStart( &TxNextPacketTimer );
			}
			break;
		}
		case MLME_LINK_CHECK:
		{
			if( mlmeConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK )
			{
				// Check DemodMargin
				// Check NbGateways
			}
			break;
		}
		default:
			break;
	}
	//NextTx = true;
}

/*!
 * \brief   MLME-Indication event function
 *
 * \param   [IN] mlmeIndication - Pointer to the indication structure.
 */
static void MlmeIndication( MlmeIndication_t *mlmeIndication )
{
	if( mlmeIndication->Status != LORAMAC_EVENT_INFO_STATUS_BEACON_LOCKED )
	{
		printf( "\r\n###### ===== MLME-Indication ==== ######\r\n" );
		printf( "STATUS      : %s\r\n", EventInfoStatusStrings[mlmeIndication->Status] );
	}
	switch( mlmeIndication->MlmeIndication )
	{
		case MLME_SCHEDULE_UPLINK:
		{// The MAC signals that we shall provide an uplink as soon as possible
			OnTxNextPacketTimerEvent(NULL);
			break;
		}
		default:
			break;
	}
}


//static void lwan_dev_params_update( void )
//{
//#ifdef REGION_EU868
	//LoRaMacChannelAdd( 3, ( ChannelParams_t )EU868_LC4 );
	//LoRaMacChannelAdd( 4, ( ChannelParams_t )EU868_LC5 );
	//LoRaMacChannelAdd( 5, ( ChannelParams_t )EU868_LC6 );
	//LoRaMacChannelAdd( 6, ( ChannelParams_t )EU868_LC7 );
	//LoRaMacChannelAdd( 7, ( ChannelParams_t )EU868_LC8 );
//#endif

//#ifdef REGION_EU433
	//	LoRaMacChannelAdd( 3, ( ChannelParams_t )EU433_LC4 );
		//LoRaMacChannelAdd( 4, ( ChannelParams_t )EU433_LC5 );
		//LoRaMacChannelAdd( 5, ( ChannelParams_t )EU433_LC6 );
		//LoRaMacChannelAdd( 6, ( ChannelParams_t )EU433_LC7 );
		//LoRaMacChannelAdd( 7, ( ChannelParams_t )EU433_LC8 );
//#endif

	//MibRequestConfirm_t mibReq;
	//uint16_t channelsMaskTemp[6];
	//channelsMaskTemp[0] = 0x00FF;
	//channelsMaskTemp[1] = 0x0000;
	//channelsMaskTemp[2] = 0x0000;
	//channelsMaskTemp[3] = 0x0000;
	//channelsMaskTemp[4] = 0x0000;
	//channelsMaskTemp[5] = 0x0000;

	//mibReq.Type = MIB_CHANNELS_DEFAULT_MASK;
	//mibReq.Param.ChannelsMask = channelsMaskTemp;
	//LoRaMacMibSetRequestConfirm(&mibReq);

	//mibReq.Type = MIB_CHANNELS_MASK;
	//mibReq.Param.ChannelsMask = channelsMaskTemp;
	//LoRaMacMibSetRequestConfirm(&mibReq);
//}

uint8_t BoardGetBatteryLevel()
{
	int8 batlevel = ((GetBatteryVoltage()-3.7)/(4.2-3.7))*100;
	return batlevel;
}


void OnMacProcessNotify( void )
{
	IsMacProcessPending = 1;
}




void LoRaWanClass::Tick()
{
	// Tick the RTC to execute callback in context of the main loop (in stead of the IRQ)
	TimerProcess( );
	// Process Radio IRQ
	if( Radio.IrqProcess != NULL )
	{
		Radio.IrqProcess( );
	}
	// Processes the LoRaMac events
	LoRaMacProcess( );
}

//LoRaMacPrimitives_t LoRaMacPrimitive;
//LoRaMacCallback_t LoRaMacCallbacks;

void LoRaWanClass::Init(DeviceClass_t CLASS,LoRaMacRegion_t REGION)
{
	printf("LoRaWanClass::Init\r\n");
	
	AppData.MsgType = LORAMAC_HANDLER_UNCONFIRMED_MSG;
	AppData.Buffer = NULL;
	AppData.BufferSize = 0;
	AppData.Port = 0;

	LoRaMacPrimitive.MacMcpsConfirm = McpsConfirm;
	LoRaMacPrimitive.MacMcpsIndication = McpsIndication;
	LoRaMacPrimitive.MacMlmeConfirm = MlmeConfirm;
	LoRaMacPrimitive.MacMlmeIndication = MlmeIndication;
	
	LoRaMacCallbacks.GetBatteryLevel = BoardGetBatteryLevel;
	LoRaMacCallbacks.GetTemperatureLevel = NULL;
	LoRaMacCallbacks.MacProcessNotify = OnMacProcessNotify;
	
	//MibRequestConfirm_t mibReq;
	
	//mibReq.Type = MIB_DEVICE_CLASS;
	//mibReq.Param.Class = CLASS;
	//LoRaMacMibSetRequestConfirm( &mibReq );
	
	LoRaMacStatus_t status;
	status = LoRaMacInitialization( &LoRaMacPrimitive, &LoRaMacCallbacks, REGION );
	if ( status != LORAMAC_STATUS_OK )
	{
		printf( "LoRaMac wasn't properly initialized, error: %s", MacStatusStrings[status] );
		// Fatal error, endless loop.
		while ( 1 )
		{
		}
	}
	DeviceState = DEVICE_STATE_RESTORE;
}

void LoRaWanClass::Restore()
{
	printf("LoRaWanClass::Restore\r\n");
	DeviceState = DEVICE_STATE_START;
}

void LoRaWanClass::Start()
{
	printf("LoRaWanClass::Start\r\n");
	
	TimerStop( &TxNextPacketTimer );
	TimerInit( &TxNextPacketTimer, OnTxNextPacketTimerEvent );

	MibRequestConfirm_t mibReq;

	mibReq.Type = MIB_PUBLIC_NETWORK;
	mibReq.Param.EnablePublicNetwork = LORAWAN_PUBLIC_NETWORK;
	LoRaMacMibSetRequestConfirm( &mibReq );

	mibReq.Type = MIB_ADR;
	mibReq.Param.AdrEnable = LORAWAN_ADR_ON;
	LoRaMacMibSetRequestConfirm( &mibReq );

#if defined( REGION_EU868 ) || defined( REGION_RU864 ) || defined( REGION_CN779 ) || defined( REGION_EU433 )
	//LoRaMacTestSetDutyCycleOn( LORAWAN_DUTYCYCLE_ON );

#if( USE_TTN_NETWORK == 1 )
	printf("Use TTN Network\r\n");
	mibReq.Type = MIB_RX2_DEFAULT_CHANNEL;
	mibReq.Param.Rx2DefaultChannel = ( RxChannelParams_t ){ 869525000, DR_3 };
	LoRaMacMibSetRequestConfirm( &mibReq );

	mibReq.Type = MIB_RX2_CHANNEL;
	mibReq.Param.Rx2Channel = ( RxChannelParams_t ){ 869525000, DR_3 };
	LoRaMacMibSetRequestConfirm( &mibReq );

	mibReq.Type = MIB_RXC_DEFAULT_CHANNEL;
	mibReq.Param.RxCDefaultChannel = ( RxChannelParams_t ){ 869525000, DR_3 };
	LoRaMacMibSetRequestConfirm( &mibReq );

	mibReq.Type = MIB_RXC_CHANNEL;
	mibReq.Param.RxCChannel = ( RxChannelParams_t ){ 869525000, DR_3 };
	LoRaMacMibSetRequestConfirm( &mibReq );
#endif

#endif
	mibReq.Type = MIB_SYSTEM_MAX_RX_ERROR;
	mibReq.Param.SystemMaxRxError = 20;
	LoRaMacMibSetRequestConfirm( &mibReq );

	LoRaMacStart( );

	LoRaMacStatus_t status;
	mibReq.Type = MIB_NETWORK_ACTIVATION;
	status = LoRaMacMibGetRequestConfirm( &mibReq );

	if( status == LORAMAC_STATUS_OK )
	{
		if( mibReq.Param.NetworkActivation == ACTIVATION_TYPE_NONE )
		{
			DeviceState = DEVICE_STATE_JOIN;
		}
		else
		{
			DeviceState = DEVICE_STATE_SEND;
			NextTx = true;
		}
	}
}


extern "C" void GetNetInfo(void);

void LoRaWanClass::Join()
{
	printf("LoRaWanClass::Join\r\n");
	MibRequestConfirm_t mibReq;
	
//#if( OVER_THE_AIR_ACTIVATION == 0 )
	// Tell the MAC layer which network server version are we connecting too.
	//mibReq.Type = MIB_ABP_LORAWAN_VERSION;
	//mibReq.Param.AbpLrWanVersion.Value = ABP_ACTIVATION_LRWAN_VERSION;
	//LoRaMacMibSetRequestConfirm( &mibReq );
//#endif

//#if( ABP_ACTIVATION_LRWAN_VERSION == ABP_ACTIVATION_LRWAN_VERSION_V10x )
	//mibReq.Type = MIB_GEN_APP_KEY;
	//mibReq.Param.GenAppKey = GenAppKey;
	//LoRaMacMibSetRequestConfirm( &mibReq );
//#else
	//mibReq.Type = MIB_APP_KEY;
	//mibReq.Param.AppKey = AppKey;
	//LoRaMacMibSetRequestConfirm( &mibReq );
//#endif

	//mibReq.Type = MIB_NWK_KEY;
	//mibReq.Param.NwkKey = NwkKey;
	//LoRaMacMibSetRequestConfirm( &mibReq );


	if( OVER_THE_AIR_ACTIVATION )
	{
		printf("joining...\r\n");
		
		//MibRequestConfirm_t mibReq;
		
		mibReq.Type = MIB_DEV_EUI;
		mibReq.Param.DevEui = DevEui;
		LoRaMacMibSetRequestConfirm( &mibReq );
		LoRaMacMibGetRequestConfirm( &mibReq );
		printf( "DevEui      : %02X", mibReq.Param.DevEui[0] );
		for( int i = 1; i < 8; i++ )
		{
			printf( "-%02X", mibReq.Param.DevEui[i] );
		}
		printf( "\r\n" );
		
		mibReq.Type = MIB_JOIN_EUI;
		mibReq.Param.JoinEui = AppEui; //AppEui is renamed to JoinEui
		LoRaMacMibSetRequestConfirm( &mibReq );
		LoRaMacMibGetRequestConfirm( &mibReq );
		printf( "AppEui      : %02X", mibReq.Param.JoinEui[0] );
		for( int i = 1; i < 8; i++ )
		{
			printf( "-%02X", mibReq.Param.JoinEui[i] );
		}
		printf( "\r\n" );
		
		
		mibReq.Type = MIB_NWK_KEY;
		mibReq.Param.NwkKey = AppKey;
		LoRaMacMibSetRequestConfirm( &mibReq );
		LoRaMacMibGetRequestConfirm( &mibReq );
		printf( "NwkKey      : %02X", mibReq.Param.AppKey[0] );
		for( int i = 1; i < 16; i++ )
		{
			printf( "-%02X", mibReq.Param.AppKey[i] );
		}
		printf( "\r\n" );
		
		
		//mibReq.Type = MIB_APP_KEY;
		//mibReq.Param.AppKey = AppKey;
		//LoRaMacMibSetRequestConfirm( &mibReq );
		//LoRaMacMibGetRequestConfirm( &mibReq );
		//printf( "AppKey      : %02X", mibReq.Param.AppKey[0] );
		//for( int i = 1; i < 16; i++ )
		//{
		//	printf( "-%02X", mibReq.Param.AppKey[i] );
		//}
		//printf( "\r\n" );
		
		//Join the network
		JoinNetwork();
		//mibReq.Param.NetworkActivation == ACTIVATION_TYPE_OTAA;
	}
	else
	{
		//MibRequestConfirm_t mibReq;

		//mibReq.Type = MIB_NET_ID;
		//mibReq.Param.NetID = LORAWAN_NETWORK_ID;
		//LoRaMacMibSetRequestConfirm( &mibReq );

		//mibReq.Type = MIB_DEV_ADDR;
		//mibReq.Param.DevAddr = DevAddr;
		//LoRaMacMibSetRequestConfirm( &mibReq );

		//mibReq.Type = MIB_NWK_SKEY;
		//mibReq.Param.NwkSKey = NwkSKey;
		//LoRaMacMibSetRequestConfirm( &mibReq );

		//mibReq.Type = MIB_APP_SKEY;
		//mibReq.Param.AppSKey = AppSKey;
		//LoRaMacMibSetRequestConfirm( &mibReq );

		//mibReq.Type = MIB_NETWORK_JOINED;
		//mibReq.Param.IsNetworkJoined = true;
		//mibReq.Param.NetworkActivation == ACTIVATION_TYPE_ABP;
		//LoRaMacMibSetRequestConfirm( &mibReq );
		
		//DeviceState = DEVICE_STATE_SEND;
	}
}

void LoRaWanClass::Send(DeviceClass_t CLASS)
{
	printf("LoRaWanClass::Send\r\n");
	if( NextTx == true )
	{
		MibRequestConfirm_t mibReq;
		
		mibReq.Type = MIB_DEVICE_CLASS;
		mibReq.Param.Class = CLASS;
		LoRaMacMibSetRequestConfirm( &mibReq );
		
		//MibRequestConfirm_t mibReq;
		//mibReq.Type = MIB_DEVICE_CLASS;
		//LoRaMacMibGetRequestConfirm( &mibReq );
		
		//if(CLASS == CLASS_C)
		//{
			//if( mibReq.Param.Class!= CLASS_C )
			//{
			//	mibReq.Param.Class = CLASS_C;
			//	LoRaMacMibSetRequestConfirm( &mibReq );
			//}
		//}

		//if(CLASS == CLASS_A)
		//{
			//if( mibReq.Param.Class!= CLASS_A )
			//{
				//mibReq.Param.Class = CLASS_A;
				//LoRaMacMibSetRequestConfirm( &mibReq );
			//}
		//}
		
		
		NextTx = SendFrame( );
	}
	DeviceState = DEVICE_STATE_CYCLE;
}

void LoRaWanClass::Cycle(uint32_t dutycycle)
{
	DeviceState = DEVICE_STATE_SLEEP;
	TimerSetValue( &TxNextPacketTimer, dutycycle );
	TimerStart( &TxNextPacketTimer );
}

void LoRaWanClass::Sleep()
{
	CRITICAL_SECTION_BEGIN( );
	if( IsMacProcessPending == 1 )
	{
		// Clear flag and prevent MCU to go into low power modes.
		IsMacProcessPending = 0;
	}
	else
	{
		// The MCU wakes up through events
		//BoardLowPowerHandler( );
		LowPower_Handler( );
	}
	CRITICAL_SECTION_END( );
	
	
	//LowPower_Handler( );
	// Process Radio IRQ
	//Radio.IrqProcess( );
}

void LoRaWanClass::Ifskipjoin()
{
//if saved net info is OK in lorawan mode, skip join.
	if(CheckNetInfo()&&Mode_LoraWan){
		if(PassthroughMode==false)
		{
			printf("Wait 3s for user key to rejoin network\r\n");
			uint16_t i=0;
			pinMode(GPIO7,INPUT);
			while(i<=3000)
			{
				if(digitalRead(GPIO7)==LOW)//if user key down, rejoin network;
				{
					NetInfoDisable();
					pinMode(GPIO7,OUTPUT);
					digitalWrite(GPIO7,HIGH);
					return;
				}
				delay(1);
				i++;
			}
			pinMode(GPIO7,OUTPUT);
			digitalWrite(GPIO7,HIGH);
		}
#if(AT_SUPPORT)
		getDevParam();
#endif
		Init(CLASS,REGION);
		GetNetInfo();
		if(PassthroughMode==false){
			printf("User key not detected,Use reserved Net\r\n");
		}
		else{
			printf("Use reserved Net\r\n");
		}
		if(PassthroughMode==false)
		{
			int32_t temp=randr(0,APP_TX_DUTYCYCLE);
			printf("Next packet send %d ms later(random time from 0 to APP_TX_DUTYCYCLE)\r\n",temp);
			Cycle(temp);//send packet in a random time to avoid network congestion.
		}
		DeviceState = DEVICE_STATE_SLEEP;
	}
}


LoRaWanClass LoRaWAN;


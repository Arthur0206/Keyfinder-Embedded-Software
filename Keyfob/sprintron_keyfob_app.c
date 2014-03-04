/**************************************************************************************************
  Filename:       keyfobdemo.c
  Revised:        $Date: 2013-08-15 15:28:40 -0700 (Thu, 15 Aug 2013) $
  Revision:       $Revision: 34986 $

  Description:    Key Fob Demo Application.

  Copyright 2009 - 2013 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED �AS IS� WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
**************************************************************************************************/

/*********************************************************************
 * INCLUDES
 */

#include "bcomdef.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"

#include "OnBoard.h"
#include "hal_adc.h"
#include "hal_led.h"
#include "hal_key.h"

#include "buzzer.h"

#if defined ( ACC_BMA250 )
#  include "bma250.h"
#elif defined ( ACC_CMA3000 )
#  include "cma3000d.h"
#endif

#include "gatt.h"

#include "hci.h"

#include "gapgattserver.h"
#include "gattservapp.h"
#include "gatt_profile_uuid.h"

#if defined ( PLUS_BROADCASTER )
  #include "peripheralBroadcaster.h"
#else
  #include "peripheral.h"
#endif

#include "gapbondmgr.h"

#include "devinfoservice.h"
#include "battservice.h"
#include "Proxreporter.h"
#include "sprintron_keyfob_profile.h"
#include "sprintron_keyfob_app.h"

#include "osal_snv.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

// Delay between power-up and starting advertising (in ms)
#define STARTDELAY                            500

// Number of beeps before buzzer stops by itself
#define BUZZER_MAX_BEEPS                      200

// Buzzer beep tone frequency for "High Alert" (in Hz)
#define BUZZER_ALERT_HIGH_FREQ                4096

// Buzzer beep tone frequency for "Low Alert" (in Hz)
#define BUZZER_ALERT_LOW_FREQ                 250

// How often to check battery voltage (in ms)
#define BATTERY_CHECK_PERIOD                  10000

//GAP Peripheral Role desired connection parameters

// Use limited discoverable mode to advertise for 30.72s, and then stop, or
// use general discoverable mode to advertise indefinitely

//#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_LIMITED
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_GENERAL

/************************************************************************************/
// These 4 items are not used anymore, but remain here for reference. 

// Minimum connection interval (units of 1.25ms, 80=100ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     80

// Maximum connection interval (units of 1.25ms, 800=1000ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     800

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_DESIRED_SLAVE_LATENCY         0

// Supervision timeout value (units of 10ms, 1000=10s) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_CONN_TIMEOUT          1000
/************************************************************************************/

// request RSSI value periodically. Default value is 0 means no periodic event for reading RSSI.
#define DEFAULT_DESIRED_RSSI_READ_RATE        0

// Whether to enable automatic parameter update request when a connection is formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         TRUE

// Connection Pause Peripheral time value (in seconds)
#define DEFAULT_CONN_PAUSE_PERIPHERAL         5

// Fast adv time after pressing button
#define FAST_ADV_TIME                         30

// buzzer_state values
#define BUZZER_OFF                            0
#define BUZZER_ON                             1

// Company Identifier: Texas Instruments Inc. (13)
#define TI_COMPANY_ID                         0x000D

#define INVALID_CONNHANDLE                    0xFFFF

#if defined ( PLUS_BROADCASTER )
  #define ADV_IN_CONN_WAIT                    500 // delay 500 ms
#endif

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
static uint8 keyfobapp_TaskID;   // Task ID for internal task/event processing

static gaprole_States_t gapProfileState = GAPROLE_INIT;

// Sprintron Keyfob State Variables
static int8 keyfobRssiValue = RSSI_VALUE_DEFAULT_VALUE;   
static int8 keyfobClientTxPwr = CLIENT_TX_POWER_DEFAULT_VALUE; 
static int8 keyfobProximityConfig = PROXIMITY_CONFIG_DEFAULT_VALUE;    
static uint8 keyfobProximityAlert = PROXIMITY_ALERT_IN_RANGE;  
static uint32 keyfobAudioVisualAlert = AUDIO_VISUAL_ALERT_ALL_OFF; 
static uint16 keyfobDeviceConfigParameters[5] = { 
	                                              // connection parameters
	                                              CONNECTION_INTERVAL_DEFAULT_VALUE,
	                                              SUPERVISION_TIMEOUT_DEFAULT_VALUE,
	                                              SLAVE_LATENCY_DEFAULT_VALUE,
	                                              // normal adv interval
	                                              NORMAL_ADV_INTERVAL_DEFAULT_VALUE,
	                                              // av alert lasting time
	                                              AUDIO_VISUAL_ALERT_TIME_DEFAULT_VALUE };


#ifdef USE_WHITE_LIST_ADV
// set initialized value to a random addr, so we can recognize when it is connected at the first time after system boot up.
uint8 connectedDeviceBDAddr[B_ADDR_LEN] = {0x13,0x24,0x35,0x46,0x57,0x68};
#endif

// GAP - SCAN RSP data (max size = 31 bytes)
static uint8 deviceName[] =
{
  // complete name
  0x11,   // length of first data structure (11 bytes excluding length byte)
  0x09,   // AD Type = Complete local name
  0x53,   // 'S'
  0x70,   // 'p'
  0x72,   // 'r'
  0x69,   // 'i'
  0x6e,   // 'n'
  0x74,   // 't'
  0x72,   // 'r'
  0x6f,   // 'o'
  0x6e,   // 'n'
  0x20,   // ' '
  0x4b,   // 'K'
  0x65,   // 'e'
  0x79,   // 'y'
  0x66,   // 'f'
  0x6f,   // 'o'
  0x62    // 'b'
};


// GAP - Advertisement data (max size = 31 bytes, though this is
// best kept short to conserve power while advertisting)
static uint8 advertData[] =
{
  0x02,   // length of first data structure (2 bytes excluding length byte)
  GAP_ADTYPE_FLAGS,   // AD Type = Flags
  DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

  0x07,
  GAP_ADTYPE_MANUFACTURER_SPECIFIC, //manufacturer specific data
  0, //put BD addr here
  0, //put BD addr here
  0, //put BD addr here
  0, //put BD addr here
  0, //put BD addr here
  0, //put BD addr here

  // service UUID, to notify central devices what services are included
  // in this peripheral
  0x0B,   // length of second data structure
  GAP_ADTYPE_16BIT_MORE,   // list of 16-bit UUID's available, but not complete list
  LO_UINT16( SPRINTRON_RSSI_REPORT_SERVICE_UUID ),
  HI_UINT16( SPRINTRON_RSSI_REPORT_SERVICE_UUID ),
  LO_UINT16( SPRINTRON_PROXIMITY_ALERT_SERVICE_UUID ),
  HI_UINT16( SPRINTRON_PROXIMITY_ALERT_SERVICE_UUID ),
  LO_UINT16( SPRINTRON_CLIENT_TX_POWER_SERVICE_UUID ),
  HI_UINT16( SPRINTRON_CLIENT_TX_POWER_SERVICE_UUID ),
  LO_UINT16( SPRINTRON_AUDIO_VISUAL_ALERT_SERVICE_UUID ),
  HI_UINT16( SPRINTRON_AUDIO_VISUAL_ALERT_SERVICE_UUID ),
  LO_UINT16( SPRINTRON_DEVICE_CONFIG_SERVICE_UUID ),
  HI_UINT16( SPRINTRON_DEVICE_CONFIG_SERVICE_UUID )
};

// GAP GATT Attributes
static uint8 attDeviceName[GAP_DEVICE_NAME_LEN] = "Sprintron Keyfob";

// Buzzer state
static uint8 buzzer_state = BUZZER_OFF;
static uint8 buzzer_beep_count = 0;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void keyfobapp_ProcessOSALMsg( osal_event_hdr_t *pMsg );
static void keyfobapp_PerformAlert( void );
static void keyfobapp_StopAlert( void );
static void keyfobapp_HandleKeys( uint8 shift, uint8 keys );
static void peripheralStateNotificationCB( gaprole_States_t newState );
static void sprintronKeyfobAttrChangedCB( uint8 attrParamID );
static void updateRssiProximityAlert( int8 newRSSI );
static void updateConnParametersDeviceConfig( void );
static void putBDAddrIntoAdvData( uint8 *bd_addr );
static uint8 isProximityAlertToggleNeeded( void );

/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapRolesCBs_t keyFob_PeripheralCBs =
{
  peripheralStateNotificationCB,  // Profile State Change Callbacks
  updateRssiProximityAlert,   // When a valid RSSI is read from controller
  putBDAddrIntoAdvData
};

// GAP Bond Manager Callbacks
static gapBondCBs_t keyFob_BondMgrCBs =
{
  NULL,                     // Passcode callback (not used by application)
  NULL                      // Pairing / Bonding state Callback (not used by application)
};

// Proximity Peripheral Profile Callbacks
static sprintronKeyfobCBs_t keyFob_ProfileCBs =
{
  sprintronKeyfobAttrChangedCB,              // Whenever attributes in the service changes
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

 /*********************************************************************
 * @fn      putBDAddrIntoAdvData
 *
 * @brief   Added by Sprintron
 *
 * @param  none
 *
 * @return  Out_Of_Range_Status
 */
static void putBDAddrIntoAdvData( uint8 *bd_addr )
{
  advertData[5] = bd_addr[0];
  advertData[6] = bd_addr[1];
  advertData[7] = bd_addr[2];
  advertData[8] = bd_addr[3];
  advertData[9] = bd_addr[4];
  advertData[10] = bd_addr[5];
  GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( advertData ), advertData );
}
 
/*********************************************************************
 * @fn      isProximityAlertToggleNeeded
 *
 * @brief   Added by Sprintron
 *
 * @param  none
 *
 * @return  Out_Of_Range_Status
 */

static bool isProximityAlertToggleNeeded()
{
  uint8 new_status;

  uint8 old_status;
  sprintronKeyfob_GetParameter( SPRINTRON_PROXIMITY_ALERT, &old_status );
  
  if (keyfobClientTxPwr - keyfobRssiValue <= keyfobProximityConfig)
  {
    new_status = PROXIMITY_ALERT_IN_RANGE;
  }
  else
  {
    new_status = PROXIMITY_ALERT_OUT_OF_RANGE;
  }

  if (new_status != old_status)
    return true;
  else 
    return false;
}

/*********************************************************************
 * @fn      updateRssiProximityAlert
 *
 * @brief   Added by Sprintron
 *              This function will be called when RSSI value is available every 1s. 
 *              This function updates server_RSSI_value attr and Out_of_Range_Status attr. 
 * @param   rssi value
 *
 * @return  none
 */

static void updateRssiProximityAlert( int8 newRSSI )
{
  keyfobRssiValue = newRSSI;
  sprintronKeyfob_SetParameter( SPRINTRON_RSSI_VALUE,  sizeof ( int8 ), &keyfobRssiValue );
    
  // To do - set this value only when status changes.
  if ( isProximityAlertToggleNeeded() ) 
  {  
    keyfobProximityAlert = (keyfobProximityAlert == PROXIMITY_ALERT_OUT_OF_RANGE) ? 
                              PROXIMITY_ALERT_IN_RANGE : PROXIMITY_ALERT_OUT_OF_RANGE;

    sprintronKeyfob_SetParameter( SPRINTRON_PROXIMITY_ALERT,  sizeof ( int8 ), &keyfobProximityAlert );
  }
}

/*********************************************************************
 * @fn      updateConnParametersDeviceConfig
 *
 * @brief   Added by Sprintron
 *              This function will be called when:
 *              1. device is connected
 *              2. GAP send GAPROLE_CONN_PARAM_UPDATED event
 *              to write current connection parameters into corresponding attributes.
 * @param   rssi value
 *
 * @return  none
 */
static void updateConnParametersDeviceConfig( uint16 paramIdx )
{
    switch( paramIdx )
    { 
      case CONFIG_IDX_CONNECTION_INTERVAL:
        {
          // update the 3 connection parameters into local copy keyfobDeviceConfigParameters
          GAPRole_GetParameter( GAPROLE_CONN_INTERVAL, keyfobDeviceConfigParameters + CONFIG_IDX_CONNECTION_INTERVAL );
          GAPRole_GetParameter( GAPROLE_CONN_TIMEOUT, keyfobDeviceConfigParameters + CONFIG_IDX_SUPERVISION_TIMEOUT );
          GAPRole_GetParameter( GAPROLE_CONN_LATENCY, keyfobDeviceConfigParameters + CONFIG_IDX_SLAVE_LATENCY );

          // update the 3 connection parameter into corresponding attributes
          sprintronKeyfob_SetParameter( SPRINTRON_DEVICE_CONFIG_PARAMETERS, sizeof(keyfobDeviceConfigParameters), keyfobDeviceConfigParameters );
        }
        break;
        
      case CONFIG_IDX_NORMAL_ADV_INTERVAL:
        {
          // update normal adv interval into local copy keyfobDeviceConfigParameters
          keyfobDeviceConfigParameters[CONFIG_IDX_NORMAL_ADV_INTERVAL] = GAP_GetParamValue( TGAP_CONN_ADV_INT_MIN );
          
          // update normal adv interval into corresponding attributes
          sprintronKeyfob_SetParameter( SPRINTRON_DEVICE_CONFIG_PARAMETERS, sizeof(keyfobDeviceConfigParameters), keyfobDeviceConfigParameters );
        }
        break;
        
      default:
        break;
    }
}

/*********************************************************************
 * @fn      KeyFobApp_Init
 *
 * @brief   Initialization function for the Key Fob App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notificaiton ... ).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void KeyFobApp_Init( uint8 task_id )
{
  keyfobapp_TaskID = task_id;
    
  // Setup the GAP
  VOID GAP_SetParamValue( TGAP_CONN_PAUSE_PERIPHERAL, DEFAULT_CONN_PAUSE_PERIPHERAL );
  // setup normal adv interval to default value
  VOID GAP_SetParamValue( TGAP_CONN_ADV_INT_MIN, NORMAL_ADV_INTERVAL_DEFAULT_VALUE );
  VOID GAP_SetParamValue( TGAP_CONN_ADV_INT_MAX, NORMAL_ADV_INTERVAL_DEFAULT_VALUE );

  // Setup the GAP Peripheral Role Profile
  {
#ifdef USE_WHITE_LIST_ADV 
    // Start adv right after device is initialized.
    uint8 initial_advertising_enable = TRUE;
#else
    // For the CC2540DK-MINI keyfob, device doesn't start advertising until button is pressed
    uint8 initial_advertising_enable = FALSE;
#endif

    // When in limited adv mode, use this variable to set adv periodical interval.
    // when set to 0. adv won't restart periodically. To change adv last time, use 
    // GAP_SetParamValue(TGAP_LIM_ADV_TIMEOUT, 180);	
    uint16 gapRole_AdvertOffTime = 0;

    uint8 enable_update_request = DEFAULT_ENABLE_UPDATE_REQUEST;
    uint16 desired_min_interval = CONNECTION_INTERVAL_DEFAULT_VALUE;
    uint16 desired_max_interval = CONNECTION_INTERVAL_DEFAULT_VALUE;
    uint16 desired_slave_latency = SLAVE_LATENCY_DEFAULT_VALUE;
    uint16 desired_conn_timeout = SUPERVISION_TIMEOUT_DEFAULT_VALUE;
    uint16 desired_rssi_read_rate = DEFAULT_DESIRED_RSSI_READ_RATE;

#ifdef USE_WHITE_LIST_ADV 
    uint8 snvStatus;
	uint8 adv_filter_policy = GAP_FILTER_POLICY_WHITE;
	
	GAPRole_SetParameter(GAPROLE_ADV_FILTER_POLICY, sizeof( uint8 ), &adv_filter_policy);

    // read previously connected device's BD Addr from NV
    snvStatus = osal_snv_read( SPRINTRON_KEYFOB_NV_ITEM_WHITELIST_DEVICE_ID,
                             B_ADDR_LEN,
                             (uint8 *)connectedDeviceBDAddr );

    // if read success, add it into white list. 
    // if fail, means device is never connected to a device. don't add white list. (connectedDeviceBDAddr remains as initialized value)
    if (snvStatus == SUCCESS)
    {
	  // add previously connected device into whitelist.
	  VOID HCI_LE_AddWhiteListCmd( HCI_PUBLIC_DEVICE_ADDRESS, connectedDeviceBDAddr );
    }
#endif

    // Set the GAP Role Parameters
    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );

    // this is for limited adv mode, but keep it here if later we switch to limited adv mode.
    GAPRole_SetParameter( GAPROLE_ADVERT_OFF_TIME, sizeof( uint16 ), &gapRole_AdvertOffTime );

    // read RSSI and update out_of_range_status attr & server_RSSI_value attr every 1 second
    GAPRole_SetParameter( GAPROLE_RSSI_READ_RATE, sizeof( uint16 ), &desired_rssi_read_rate );

    GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, sizeof ( deviceName ), deviceName );

    GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( advertData ), advertData );

  
    GAPRole_SetParameter( GAPROLE_PARAM_UPDATE_ENABLE, sizeof( uint8 ), &enable_update_request );
    GAPRole_SetParameter( GAPROLE_MIN_CONN_INTERVAL, sizeof( uint16 ), &desired_min_interval );
    GAPRole_SetParameter( GAPROLE_MAX_CONN_INTERVAL, sizeof( uint16 ), &desired_max_interval );
    GAPRole_SetParameter( GAPROLE_SLAVE_LATENCY, sizeof( uint16 ), &desired_slave_latency );
    GAPRole_SetParameter( GAPROLE_TIMEOUT_MULTIPLIER, sizeof( uint16 ), &desired_conn_timeout );
  }
  
  // Set the GAP Attributes
  GGS_SetParameter( GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName );

  // Setup the GAP Bond Manager
  {
    uint8 pairMode = GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
    uint8 mitm = TRUE;
    uint8 ioCap = GAPBOND_IO_CAP_DISPLAY_ONLY;
    uint8 bonding = TRUE;
    
    GAPBondMgr_SetParameter( GAPBOND_PAIRING_MODE, sizeof ( uint8 ), &pairMode );
    GAPBondMgr_SetParameter( GAPBOND_MITM_PROTECTION, sizeof ( uint8 ), &mitm );
    GAPBondMgr_SetParameter( GAPBOND_IO_CAPABILITIES, sizeof ( uint8 ), &ioCap );
    GAPBondMgr_SetParameter( GAPBOND_BONDING_ENABLED, sizeof ( uint8 ), &bonding );
  }

  // Initialize GATT attributes
  GGS_AddService( GATT_ALL_SERVICES );         // GAP
  GATTServApp_AddService( GATT_ALL_SERVICES ); // GATT attributes
  DevInfo_AddService();   // Device Information Service
  sprintronKeyfob_AddService( GATT_ALL_SERVICES );  // Sprintron Keyfob Profile
  ProxReporter_AddService(PP_TX_PWR_LEVEL_SERVICE); // Tx Power service
  Batt_AddService( );     // Battery Service

  // make sure buzzer is off
  buzzerStop();

  // makes sure LEDs are off
  HalLedSet( (HAL_LED_1 | HAL_LED_2), HAL_LED_MODE_OFF );

  // For keyfob board set GPIO pins into a power-optimized state
  // Note that there is still some leakage current from the buzzer,
  // accelerometer, LEDs, and buttons on the PCB.

  P0SEL = 0; // Configure Port 0 as GPIO
  P1SEL = 0x40; // Configure Port 1 as GPIO, except P1.6 for peripheral function for buzzer
  P2SEL = 0; // Configure Port 2 as GPIO

  P0DIR = 0xFC; // Port 0 pins P0.0 and P0.1 as input (buttons),
                // all others (P0.2-P0.7) as output
  P1DIR = 0xFF; // All port 1 pins (P1.0-P1.7) as output
  P2DIR = 0x1F; // All port 1 pins (P2.0-P2.4) as output

  P0 = 0x03; // All pins on port 0 to low except for P0.0 and P0.1 (buttons)
  P1 = 0;   // All pins on port 1 to low
  P2 = 0;   // All pins on port 2 to low


  // initialize the ADC for battery reads
  HalAdcInit();

  // Register for all key events - This app will handle all key events
  RegisterForKeys( keyfobapp_TaskID );

#if defined ( DC_DC_P0_7 )

  // Enable stack to toggle bypass control on TPS62730 (DC/DC converter)
  HCI_EXT_MapPmIoPortCmd( HCI_EXT_PM_IO_PORT_P0, HCI_EXT_PM_IO_PORT_PIN7 );

#endif // defined ( DC_DC_P0_7 )
  
  // Setup a delayed profile startup
  osal_start_timerEx( keyfobapp_TaskID, KFD_START_DEVICE_EVT, STARTDELAY );
}

/*********************************************************************
 * @fn      KeyFobApp_ProcessEvent
 *
 * @brief   Key Fob Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  none
 */
uint16 KeyFobApp_ProcessEvent( uint8 task_id, uint16 events )
{
  if ( events & SYS_EVENT_MSG )
  {
    uint8 *pMsg;

    if ( (pMsg = osal_msg_receive( keyfobapp_TaskID )) != NULL )
    {
      keyfobapp_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );

      // Release the OSAL message
      VOID osal_msg_deallocate( pMsg );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  if ( events & KFD_START_DEVICE_EVT )
  {
    // Start the Device
    VOID GAPRole_StartDevice( &keyFob_PeripheralCBs );

    // Start Bond Manager
    VOID GAPBondMgr_Register( &keyFob_BondMgrCBs );

    // Start the Proximity Profile
    VOID sprintronKeyfob_RegisterAppCBs( &keyFob_ProfileCBs );

    // Set timer for first battery read event
    osal_start_timerEx( keyfobapp_TaskID, KFD_BATTERY_CHECK_EVT, BATTERY_CHECK_PERIOD );

    // Request to read BD_ADDR. This need to be done after callback is setup.
    HCI_ReadBDADDRCmd();
    //Set the proximity attribute values to default
    sprintronKeyfob_SetParameter( SPRINTRON_RSSI_VALUE,  sizeof ( int8 ), &keyfobRssiValue );
    sprintronKeyfob_SetParameter( SPRINTRON_PROXIMITY_CONFIG,  sizeof ( int8 ), &keyfobProximityConfig );
    sprintronKeyfob_SetParameter( SPRINTRON_PROXIMITY_ALERT,  sizeof ( uint8 ), &keyfobProximityAlert );
    sprintronKeyfob_SetParameter( SPRINTRON_CLIENT_TX_POWER,  sizeof ( int8 ), &keyfobClientTxPwr );
    sprintronKeyfob_SetParameter( SPRINTRON_AUDIO_VISUAL_ALERT,  sizeof ( keyfobAudioVisualAlert ), &keyfobAudioVisualAlert );
    sprintronKeyfob_SetParameter( SPRINTRON_DEVICE_CONFIG_PARAMETERS,  sizeof ( keyfobDeviceConfigParameters ), keyfobDeviceConfigParameters );

    // Set LED1 on to give feedback that the power is on, and a timer to turn off
    HalLedSet( HAL_LED_1, HAL_LED_MODE_ON );
    osal_pwrmgr_device( PWRMGR_ALWAYS_ON ); // To keep the LED on continuously.
    osal_start_timerEx( keyfobapp_TaskID, KFD_POWERON_LED_TIMEOUT_EVT, 1000 );
    
    return ( events ^ KFD_START_DEVICE_EVT );
  }

  if ( events & KFD_POWERON_LED_TIMEOUT_EVT )
  {
    osal_pwrmgr_device( PWRMGR_BATTERY ); // Revert to battery mode after LED off
    HalLedSet( HAL_LED_1, HAL_LED_MODE_OFF ); 
    return ( events ^ KFD_POWERON_LED_TIMEOUT_EVT );
  }

  if ( events & KFD_BATTERY_CHECK_EVT )
  {
    // Restart timer
    if ( BATTERY_CHECK_PERIOD )
    {
      osal_start_timerEx( keyfobapp_TaskID, KFD_BATTERY_CHECK_EVT, BATTERY_CHECK_PERIOD );
    }

    // perform battery level check
    Batt_MeasLevel( );

    return (events ^ KFD_BATTERY_CHECK_EVT);
  }

  if ( events & KFD_TOGGLE_BUZZER_EVT )
  {
    // if this event was triggered while buzzer is on, turn it off, increment beep_count,
    // check whether max has been reached, and if not set the OSAL timer for next event to
    // turn buzzer back on.
    if ( buzzer_state == BUZZER_ON )
    {
      buzzerStop();
      buzzer_state = BUZZER_OFF;
      buzzer_beep_count++;
      #if defined ( POWER_SAVING )
        osal_pwrmgr_device( PWRMGR_BATTERY );
      #endif

      // check to see if buzzer has beeped maximum number of times
      // if it has, then don't turn it back on
      if ( ( buzzer_beep_count < keyfobDeviceConfigParameters[CONFIG_IDX_AUDIO_VISUAL_ALERT_TIME] ) &&
           ( ((uint8 *)keyfobAudioVisualAlert)[BUZZER_ALERT_BYTE_ORDER] != BUZZER_ALERT_OFF) )
      {
        osal_start_timerEx( keyfobapp_TaskID, KFD_TOGGLE_BUZZER_EVT, 800 );
      }

      // if reach maximum number of times, reset counter, and set SPRINTRON_AUDIO_VISUAL_ALERT attribute to OFF.
      if ( buzzer_beep_count >= keyfobDeviceConfigParameters[CONFIG_IDX_AUDIO_VISUAL_ALERT_TIME] )
      {
        // set visual alert level to off.
        ((uint8 *)keyfobAudioVisualAlert)[BUZZER_ALERT_BYTE_ORDER] = BUZZER_ALERT_OFF;
        sprintronKeyfob_SetParameter( SPRINTRON_AUDIO_VISUAL_ALERT,  sizeof ( keyfobAudioVisualAlert ), &keyfobAudioVisualAlert );

        // reset beep counter.
        buzzer_beep_count = 0;
      }
    }
    else if ( ((uint8 *)keyfobAudioVisualAlert)[BUZZER_ALERT_BYTE_ORDER] != BUZZER_ALERT_OFF )
    {
      // if this event was triggered while the buzzer is off then turn it on if appropriate
      keyfobapp_PerformAlert();
    }

    return (events ^ KFD_TOGGLE_BUZZER_EVT);
  }


#if defined ( PLUS_BROADCASTER )
  if ( events & KFD_ADV_IN_CONNECTION_EVT )
  {
    uint8 turnOnAdv = TRUE;
    // Turn on advertising while in a connection
    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &turnOnAdv );
  }
#endif

#ifdef USE_WHITE_LIST_ADV 
  if (events & KFD_NON_WHITELIST_START_EVT)
  {
    uint8 adv_filter_policy = GAP_FILTER_POLICY_ALL;
    uint8 turnOnAdv = TRUE;

    // set adv filter policy to accept connection and scan request from all devices
    GAPRole_SetParameter( GAPROLE_ADV_FILTER_POLICY, sizeof( uint8 ), &adv_filter_policy );

    // turn on adv
    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof(uint8), &turnOnAdv );

    // set a 30s timer for stopping the non-whitelist adv state.
    osal_start_timerEx( keyfobapp_TaskID, KFD_NON_WHITELIST_STOP_EVT, FAST_ADV_TIME );
  }

  if (events & KFD_NON_WHITELIST_STOP_EVT)
  {
    // if not in connection, then adv should be on at the time
    if ( gapProfileState != GAPROLE_CONNECTED ) 
    {
      // turn off adv first
      uint8 turnOnAdv = FALSE;
      GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof(uint8), &turnOnAdv );

      // turn on adv with a delay. if no delay it won't work for some reasons
      osal_start_timerEx( keyfobapp_TaskID, KFD_WHITELIST_START_EVT, 500 );
    }
    else // if in connection, then adv should be off at the time
    {
      uint8 adv_filter_policy = GAP_FILTER_POLICY_WHITE;

      // set adv filter policy to only accept devices in whitelist
      GAPRole_SetParameter( GAPROLE_ADV_FILTER_POLICY, sizeof( uint8 ), &adv_filter_policy );
    }

    // Turn off the LED that shows we're advertising without whitelist. 
    HalLedSet( HAL_LED_2, HAL_LED_MODE_OFF );
  }

  if (events & KFD_WHITELIST_START_EVT)
  {
    uint8 adv_filter_policy = GAP_FILTER_POLICY_WHITE;
    uint8 turnOnAdv = TRUE;
    
    // set adv filter policy to only accept devices in whitelist
    GAPRole_SetParameter( GAPROLE_ADV_FILTER_POLICY, sizeof( uint8 ), &adv_filter_policy );

    // turn on adv
    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof(uint8), &turnOnAdv );
  }
#endif

  // this event will be triggered every connection interval
  if (events & KFD_CONNECTION_INTERVAL_EVT)
  {
	// call HCI command to read RSSI on every connection interval
	uint16 conn_handle;

	// get connection handle
	GAPRole_GetParameter( GAPROLE_CONNHANDLE, &conn_handle );

	// call hci command to read rssi value
	VOID HCI_ReadRssiCmd( conn_handle );
  }

  // Discard unknown events
  return 0;
}

/*********************************************************************
 * @fn      keyfobapp_ProcessOSALMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void keyfobapp_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
  switch ( pMsg->event )
  {
    case KEY_CHANGE:
      keyfobapp_HandleKeys( ((keyChange_t *)pMsg)->state, ((keyChange_t *)pMsg)->keys );
      break;
  }
}

/*********************************************************************
 * @fn      keyfobapp_HandleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_2
 *                 HAL_KEY_SW_1
 *
 * @return  none
 */
static void keyfobapp_HandleKeys( uint8 shift, uint8 keys )
{

  (void)shift;  // Intentionally unreferenced parameter

  if ( keys & HAL_KEY_SW_1 )
  {
    // Can use this button to do some test. Not needed in production - Sprintron
  }

  if ( keys & HAL_KEY_SW_2 )
  {
#ifdef USE_WHITE_LIST_ADV
    // if device is not in a connection, pression the right key should change adv police to not use whitelist.
    if ( gapProfileState != GAPROLE_CONNECTED )
    {
      uint8 turnOnAdv = FALSE;

      // turn off adv first
      GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof(uint8), &turnOnAdv );

      // turn on adv with a delay. if no delay it won't work for some reasons
      osal_start_timerEx( keyfobapp_TaskID, KFD_NON_WHITELIST_START_EVT, 500 );
    }
#else //USE_WHITE_LIST_ADV
    // if device is not in a connection, pressing the right key should toggle
    // advertising on and off
    if( gapProfileState != GAPROLE_CONNECTED )
    {
      uint8 current_adv_enabled_status;
      uint8 new_adv_enabled_status;

      //Find the current GAP advertisement status
      GAPRole_GetParameter( GAPROLE_ADVERT_ENABLED, &current_adv_enabled_status );

      if( current_adv_enabled_status == FALSE )
      {
        new_adv_enabled_status = TRUE;
      }
      else
      {
        new_adv_enabled_status = FALSE;
      }

      //change the GAP advertisement status to opposite of current status
      GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &new_adv_enabled_status );
    }
#endif //USE_WHITE_LIST_ADV

  }

}

/*********************************************************************
 * @fn      keyfobapp_PerformAlert
 *
 * @brief   Performs an alert
 *
 * @param   none
 *
 * @return  none
 */
static void keyfobapp_PerformAlert( void )
{
    switch( ((uint8 *)keyfobAudioVisualAlert)[BUZZER_ALERT_BYTE_ORDER] )
    {
    case BUZZER_ALERT_LOW:

  #if defined ( POWER_SAVING )
        osal_pwrmgr_device( PWRMGR_ALWAYS_ON );
  #endif

      buzzerStart( BUZZER_ALERT_LOW_FREQ );
      buzzer_state = BUZZER_ON;
      // only run buzzer for 200ms
      osal_start_timerEx( keyfobapp_TaskID, KFD_TOGGLE_BUZZER_EVT, 200 );

      HalLedSet( (HAL_LED_1 | HAL_LED_2), HAL_LED_MODE_OFF );
      break;

    case BUZZER_ALERT_HIGH:

  #if defined ( POWER_SAVING )
        osal_pwrmgr_device( PWRMGR_ALWAYS_ON );
  #endif

      buzzerStart( BUZZER_ALERT_HIGH_FREQ );
      buzzer_state = BUZZER_ON;
      // only run buzzer for 200ms
      osal_start_timerEx( keyfobapp_TaskID, KFD_TOGGLE_BUZZER_EVT, 200 );

      HalLedSet( HAL_LED_1, HAL_LED_MODE_ON );
      HalLedSet( HAL_LED_2, HAL_LED_MODE_FLASH );
      break;

    case BUZZER_ALERT_OFF:
        // Fall through
    default:
      keyfobapp_StopAlert();
      break;
    }

}

/*********************************************************************
 * @fn      keyfobapp_StopAlert
 *
 * @brief   Stops an alert
 *
 * @param   none
 *
 * @return  none
 */
void keyfobapp_StopAlert( void )
{

  buzzerStop();
  buzzer_state = BUZZER_OFF;
  HalLedSet( (HAL_LED_1 | HAL_LED_2), HAL_LED_MODE_OFF );


  #if defined ( POWER_SAVING )
    osal_pwrmgr_device( PWRMGR_BATTERY );
  #endif
}

/*********************************************************************
 * @fn      peripheralStateNotificationCB
 *
 * @brief   Notification from the profile of a state change.
 *
 * @param   newState - new state
 *
 * @return  none
 */
static void peripheralStateNotificationCB( gaprole_States_t newState )
{
  uint16 connHandle = INVALID_CONNHANDLE;

  if ( gapProfileState != newState )
  {
    switch( newState )
    {
    case GAPROLE_STARTED:
      {
        // Set the system ID from the bd addr
        uint8 systemId[DEVINFO_SYSTEM_ID_LEN];
        GAPRole_GetParameter(GAPROLE_BD_ADDR, systemId);

        // shift three bytes up
        systemId[7] = systemId[5];
        systemId[6] = systemId[4];
        systemId[5] = systemId[3];

        // set middle bytes to zero
        systemId[4] = 0;
        systemId[3] = 0;

        DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, systemId);
      }
      break;

    //if the state changed to advertising, initially assume that keyfob is in range
    case GAPROLE_ADVERTISING:
      {
        uint8 adv_filter_policy;
      	GAPRole_GetParameter(GAPROLE_ADV_FILTER_POLICY, &adv_filter_policy);

        if ( adv_filter_policy == GAP_FILTER_POLICY_ALL)
        {
          // Visual feedback that we are advertising to all devices (not using whitelist).
          HalLedSet( HAL_LED_2, HAL_LED_MODE_ON );
        }

        // update noraml adv interval in device config service attribute table
        updateConnParametersDeviceConfig( CONFIG_IDX_NORMAL_ADV_INTERVAL );
      }
      break;
      
    //if the state changed to connected, initially assume that keyfob is in range      
    case GAPROLE_CONNECTED:
      {
#ifdef USE_WHITE_LIST_ADV
		uint8 default_connectedDeviceBDAddr[B_ADDR_LEN] = {0x13,0x24,0x35,0x46,0x57,0x68};
        uint8 newly_connectedDeviceBDAddr[B_ADDR_LEN];
#endif
        uint8 adv_filter_policy = GAP_FILTER_POLICY_WHITE;

        GAPRole_GetParameter( GAPROLE_CONNHANDLE, &connHandle );

#ifdef USE_WHITE_LIST_ADV
        GAPRole_GetParameter( GAPROLE_CONN_BD_ADDR, newly_connectedDeviceBDAddr );

        // save newly connected device's BD Addr into NV ram and white list, if any of the following is true:
        // 1. the keyfob is connected at the 1st time (ie. connectedDeviceBDAddr == {0x13,0x24,0x35,0x46,0x57,0x68})
        // 2. newly connected device's BD addr is different from previously connected device's BD addr.
        // Corner case: if the previously connected device really has BD Addr {0x13,0x24,0x35,0x46,0x57,0x68}, we might store the same value into 
        // NV ram everytime when device is reconnected, which will consume more power.
        if ( osal_memcmp( (void*)default_connectedDeviceBDAddr, (void*)connectedDeviceBDAddr, B_ADDR_LEN) ||
             !osal_memcmp( (void*)newly_connectedDeviceBDAddr, (void*)connectedDeviceBDAddr, B_ADDR_LEN) )
        {
		  // update previously connected device's BD Addr.
		  osal_memcpy( (void*)connectedDeviceBDAddr, (void*)newly_connectedDeviceBDAddr, B_ADDR_LEN );
			
		  // store newly connected device into NV ram.
		  VOID osal_snv_write( SPRINTRON_KEYFOB_NV_ITEM_WHITELIST_DEVICE_ID,
							   B_ADDR_LEN,
							   (uint8 *)connectedDeviceBDAddr );   
			
		  // add newly connected device into white list.
		  VOID HCI_LE_ClearWhiteListCmd();
		  VOID HCI_LE_AddWhiteListCmd( HCI_PUBLIC_DEVICE_ADDRESS, connectedDeviceBDAddr );
        }

        // set adv filter policy to only accept devices in whitelist
        GAPRole_SetParameter( GAPROLE_ADV_FILTER_POLICY, sizeof( uint8 ), &adv_filter_policy );
#endif

        #if defined ( PLUS_BROADCASTER )
          osal_start_timerEx( keyfobapp_TaskID, KFD_ADV_IN_CONNECTION_EVT, ADV_IN_CONN_WAIT );
        #endif

		// udpate connection parameters in device config service attribute table.
		updateConnParametersDeviceConfig( CONFIG_IDX_CONNECTION_INTERVAL );

        // Turn off the LED that shows we're advertising without whitelist.
        HalLedSet( HAL_LED_2, HAL_LED_MODE_OFF );

        // setup connection interval event callback
        HCI_EXT_ConnEventNoticeCmd( keyfobapp_TaskID, KFD_CONNECTION_INTERVAL_EVT );
      }
      break;

    case GAPROLE_WAITING:
      {
        // then the link was terminated intentionally by the slave or master,
        // or advertising timed out

        // if beep status is on, turn it off, and stop alert.
        if( ((uint8 *)keyfobAudioVisualAlert)[BUZZER_ALERT_BYTE_ORDER] != BUZZER_ALERT_OFF)
        {
          ((uint8 *)keyfobAudioVisualAlert)[BUZZER_ALERT_BYTE_ORDER] = BUZZER_ALERT_OFF;
          sprintronKeyfob_SetParameter( SPRINTRON_AUDIO_VISUAL_ALERT,  sizeof ( keyfobAudioVisualAlert ), &keyfobAudioVisualAlert );
          keyfobapp_StopAlert();
        }

        // Turn off the LED that shows that we're advertising without whitelist.
        HalLedSet( HAL_LED_2, HAL_LED_MODE_OFF );
      }
      break;

    case GAPROLE_WAITING_AFTER_TIMEOUT:
      {
        // the link was dropped due to supervision timeout

        // if beep status is on, turn it off, and stop alert.
        if( ((uint8 *)keyfobAudioVisualAlert)[BUZZER_ALERT_BYTE_ORDER] != BUZZER_ALERT_OFF )
        {
          ((uint8 *)keyfobAudioVisualAlert)[BUZZER_ALERT_BYTE_ORDER] = BUZZER_ALERT_OFF;
          sprintronKeyfob_SetParameter( SPRINTRON_AUDIO_VISUAL_ALERT,  sizeof ( keyfobAudioVisualAlert ), &keyfobAudioVisualAlert );
          keyfobapp_StopAlert();
        }
      }
      break;

	// udpate connection parameters for corresponding attributes.
    case GAPROLE_CONN_PARAM_UPDATED:
      {
		// udpate connection parameters in device config service attribute table.
		updateConnParametersDeviceConfig( CONFIG_IDX_CONNECTION_INTERVAL );
      }
      break;
    
    default:
      // do nothing
      break;
    }
  }

  gapProfileState = newState;
}

/*********************************************************************
 * @fn      sprintronKeyfobAttrChangedCB
 *
 * @brief   Notification from the profile of an atrribute change by
 *          a connected device.
 *
 * @param   attrParamID - Profile's Attribute Parameter ID
 *            PP_LINK_LOSS_ALERT_LEVEL  - The link loss alert level value
 *            PP_IM_ALERT_LEVEL  - The immediate alert level value
 *
 * @return  none
 */
static void sprintronKeyfobAttrChangedCB( uint8 attrParamID )
{
  switch( attrParamID )
  {

  case SPRINTRON_CLIENT_TX_POWER:
    {
      sprintronKeyfob_GetParameter( SPRINTRON_CLIENT_TX_POWER, &keyfobClientTxPwr );
    }
    break;

  case SPRINTRON_PROXIMITY_CONFIG:
    {
      sprintronKeyfob_GetParameter( SPRINTRON_PROXIMITY_CONFIG, &keyfobProximityConfig );
  	}
    break;

  case SPRINTRON_AUDIO_VISUAL_ALERT:
    {
	  sprintronKeyfob_GetParameter( SPRINTRON_AUDIO_VISUAL_ALERT, &keyfobAudioVisualAlert );

      if( ((uint8 *)keyfobAudioVisualAlert)[BUZZER_ALERT_BYTE_ORDER] != BUZZER_ALERT_OFF)
      {
        keyfobapp_PerformAlert();
        buzzer_beep_count = 0;
      }
      else
      {
        keyfobapp_StopAlert();
      }

      if( ((uint8 *)keyfobAudioVisualAlert)[LED_ALERT_BYTE_ORDER] == LED_ALERT_OFF)
      {
        // Turn off the LED that shows that we're advertising without whitelist.
        HalLedSet( HAL_LED_2, HAL_LED_MODE_OFF );
      }
      else
      {
        // Turn off the LED that shows that we're advertising without whitelist.
        HalLedSet( HAL_LED_2, HAL_LED_MODE_ON );
      }
    }
    break;

  case SPRINTRON_DEVICE_CONFIG_PARAMETERS:
    { 
      // the code to send connection update and change normal adv interval is in sprintron_keyfob_profile.c
      sprintronKeyfob_GetParameter( SPRINTRON_DEVICE_CONFIG_PARAMETERS, keyfobDeviceConfigParameters );
    }
    break;
    
  default:
    // should not reach here!
    break;
  }

}

/*********************************************************************
*********************************************************************/
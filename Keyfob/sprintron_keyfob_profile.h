/**************************************************************************************************
  Filename:       sprintronKeyfob.h
  Revised:        $Date: 2013-09-05 15:48:17 -0700 (Thu, 05 Sep 2013) $
  Revision:       $Revision: 35223 $

  Description:    This file contains Proximity - Reporter header file.


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
  PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, 
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

#ifndef SPRINTRON__KEYFOB__PROFILE_H
#define SPRINTRON__KEYFOB__PROFILE_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */

/*********************************************************************
 * CONSTANTS
 */

// Profile Parameters - used by set parameter & get parameter functions
#define SPRINTRON_MAN_SEC                               0
#define SPRINTRON_RSSI_VALUE                            1
#define SPRINTRON_PROXIMITY_CONFIG                      2
#define SPRINTRON_PROXIMITY_ALERT                       3
#define SPRINTRON_CLIENT_TX_POWER                       4
#define SPRINTRON_AUDIO_VISUAL_ALERT                    5
#define SPRINTRON_DEVICE_CONFIG_PARAMETERS              6

// Sprintron Keyfob character value define
#define MAN_SEC_FLAG_UNKNOWN                            0x0
#define MAN_SEC_FLAG_VALID                              0x1
#define MAN_SEC_FLAG_INVALID                            0x2

#define RSSI_VALUE_DEFAULT_VALUE                        0xFF

#define PROXIMITY_CONFIG_DEFAULT_VALUE                  0x0
#define PROXIMITY_ALERT_IN_RANGE                        0x0
#define PROXIMITY_ALERT_OUT_OF_RANGE                    0x1

#define CLIENT_TX_POWER_DEFAULT_VALUE                   0x0

#define AUDIO_VISUAL_ALERT_ALL_OFF                      0x00000000
#define BUZZER_ALERT_BYTE_ORDER                         0x0
#define BUZZER_ALERT_OFF                                0x0
#define BUZZER_ALERT_LOW                                0x1
#define BUZZER_ALERT_HIGH                               0x2
#define LED_ALERT_BYTE_ORDER                            0x1
#define LED_ALERT_OFF                                   0x0
#define LED_ALERT_ON                                    0x1


// unit is 1.25ms => 400 = 500ms
#define CONNECTION_INTERVAL_DEFAULT_VALUE               400
// unit is 10ms => 500 = 5s
#define SUPERVISION_TIMEOUT_DEFAULT_VALUE               500
#define SLAVE_LATENCY_DEFAULT_VALUE                     0
// Normal advertising interval in 625us units.  625*32 = 20ms (recommended)
#define NORMAL_ADV_INTERVAL_DEFAULT_VALUE               32
#define AUDIO_VISUAL_ALERT_TIME_DEFAULT_VALUE           200

// Sprintron Service UUID
#define SPRINTRON_MAN_SEC_SERVICE_UUID                  0xFFA0
#define SPRINTRON_RSSI_REPORT_SERVICE_UUID              0xFFA1
#define SPRINTRON_PROXIMITY_ALERT_SERVICE_UUID          0xFFA2
#define SPRINTRON_CLIENT_TX_POWER_SERVICE_UUID          0xFFA4
#define SPRINTRON_AUDIO_VISUAL_ALERT_SERVICE_UUID       0xFFA5
#define SPRINTRON_DEVICE_CONFIG_SERVICE_UUID            0xFFA6

// Sprintron Char UUID
#define SPRINTRON_MAN_SEC_UUID                          0xFFC0
#define SPRINTRON_RSSI_VALUE_UUID                       0xFFC1
#define SPRINTRON_PROXIMITY_ALERT_UUID                  0xFFC2
#define SPRINTRON_PROXIMITY_CONFIG_UUID                 0xFFC3
#define SPRINTRON_CLIENT_TX_POWER_UUID                  0xFFC4
#define SPRINTRON_AUDIO_VISUAL_ALERT_UUID               0xFFC5
#define SPRINTRON_DEVICE_CONFIG_PARAMETERS_UUID         0xFFC6

// Sprintron Keyfob Profile Services bit fields
#define SPRINTRON_RSSI_MAN_SEC_SERVICE                  0x00000001
#define SPRINTRON_RSSI_REPORT_SERVICE                   0x00000002
#define SPRINTRON_PROXIMITY_ALERT_SERVICE               0x00000004
#define SPRINTRON_CLIENT_TX_POWER_SERVICE               0x00000008
#define SPRINTRON_AUDIO_VISUAL_ALERT_SERVICE            0x00000010
#define SPRINTRON_DEVICE_CONFIG_SERVICE                 0x00000020

// Device config parameters sequence
#define CONFIG_IDX_CONNECTION_INTERVAL                  0
#define CONFIG_IDX_SUPERVISION_TIMEOUT                  1
#define CONFIG_IDX_SLAVE_LATENCY                        2
#define CONFIG_IDX_NORMAL_ADV_INTERVAL                  3
#define CONFIG_IDX_AUDIO_VISUAL_ALERT_TIME              4

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * Profile Callbacks
 */

// Callback when the device has been started.  Callback event to 
// the Notify of an attribute change.
typedef void (*spAttrChange_t)( uint8 attrParamID );

typedef struct
{
  spAttrChange_t        pfnAttrChange;  // Whenever the Link Loss Alert attribute changes
} sprintronKeyfobCBs_t;

/*********************************************************************
 * API FUNCTIONS 
 */
 
/*
 * sprintronKeyfob_InitService- Initializes the Proximity Reporter service by
 *          registering GATT attributes with the GATT server. Only call
 *          this function once.

 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 */
extern bStatus_t sprintronKeyfob_AddService( uint32 services );

/*
 * sprintronKeyfob_RegisterAppCBs - Registers the application callback function.
 *                    Only call this function once.
 *
 *    appCallbacks - pointer to application callbacks.
 */
extern bStatus_t sprintronKeyfob_RegisterAppCBs( sprintronKeyfobCBs_t *appCallbacks );


/*
 * sprintronKeyfob_SetParameter - Set a Proximity Reporter parameter.
 *
 *    param - Profile parameter ID
 *    len - length of data to right
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 */
extern bStatus_t sprintronKeyfob_SetParameter( uint8 param, uint8 len, void *value );
  
/*
 * sprintronKeyfob_GetParameter - Get a Proximity Reporter parameter.
 *
 *    param - Profile parameter ID
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 */
extern bStatus_t sprintronKeyfob_GetParameter( uint8 param, void *value );

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* SPRINTRON__KEYFOB__PROFILE_H */

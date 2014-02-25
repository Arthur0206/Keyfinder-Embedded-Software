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
#define SPRINTRON_RSSI_REPORT                           0
#define SPRINTRON_PROXIMITY_CONFIG                      1
#define SPRINTRON_PROXIMITY_ALERT                       2
#define SPRINTRON_CLIENT_TX_POWER                       3
#define SPRINTRON_AUDIO_VISUAL_ALERT                    4

// Sprintron Keyfob character value define
#define RSSI_REPORT_DEFAULT_VALUE                       0xFF

#define PROXIMITY_CONFIG_DEFAULT_VALUE                  0x0
#define PROXIMITY_ALERT_IN_RANGE                        0x0
#define PROXIMITY_ALERT_OUT_OF_RANGE                    0x1

#define PROXIMITY_CLIENT_TX_POWER_DEFAULT_VALUE         0x0

#define AUDIO_VISUAL_ALERT_OFF                          0x0
#define AUDIO_VISUAL_ALERT_ON                           0x1

// Sprintron Service UUID
#define SPRINTRON_RSSI_REPORT_SERVICE_UUID              0xFFA1
#define SPRINTRON_PROXIMITY_ALERT_SERVICE_UUID          0xFFA2
#define SPRINTRON_CLIENT_TX_POWER_SERVICE_UUID          0xFFA4
#define SPRINTRON_AUDIO_VISUAL_ALERT_SERVICE_UUID       0xFFA5

// Sprintron Char UUID
#define SPRINTRON_RSSI_REPORT_UUID                      0xFFC1
#define SPRINTRON_PROXIMITY_ALERT_UUID                  0xFFC2
#define SPRINTRON_PROXIMITY_CONFIG_UUID                 0xFFC3
#define SPRINTRON_CLIENT_TX_POWER_UUID                  0xFFC4
#define SPRINTRON_AUDIO_VISUAL_ALERT_UUID               0xFFC5

// Sprintron Keyfob Profile Services bit fields
#define SPRINTRON_RSSI_REPORT_SERVICE                   0x00000001
#define SPRINTRON_PROXIMITY_ALERT_SERVICE               0x00000002
#define SPRINTRON_CLIENT_TX_POWER_SERVICE               0x00000004
#define SPRINTRON_AUDIO_VISUAL_ALERT_SERVICE            0x00000008

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

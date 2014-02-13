/**************************************************************************************************
  Filename:       sprintronKeyfob.c
  Revised:        $Date: 2013-08-15 15:28:40 -0700 (Thu, 15 Aug 2013) $
  Revision:       $Revision: 34986 $

  Description:    Proximity Profile - Reporter Role


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

/*********************************************************************
 * INCLUDES
 */
#include "bcomdef.h"
#include "OSAL.h"
#include "linkdb.h"
#include "att.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gatt_profile_uuid.h"
#include "gattservapp.h"
#include "gapbondmgr.h"

#include "sprintronKeyfob.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */
#define PP_DEFAULT_TX_POWER               0
#define PP_DEFAULT_PATH_LOSS              0x7F

#define SERVAPP_NUM_ATTR_SUPPORTED        5

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */


// Sprintron Keyfob Service UUID
CONST uint8 skServiceUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16( SPRINTRON_KEYFOB_SERVICE_UUID ), HI_UINT16( SPRINTRON_KEYFOB_SERVICE_UUID )
};

// Sprintron Keyfob Server RSSI UUID
CONST uint8 skServerRssiUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16( SPRINTRON_KEYFOB_SERVER_RSSI_UUID ), HI_UINT16( SPRINTRON_KEYFOB_SERVER_RSSI_UUID )
};

// Sprintron Keyfob Client Tx Power UUID
CONST uint8 skClientTxPowerUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16( SPRINTRON_KEYFOB_CLIENT_TX_POWER_UUID ), HI_UINT16( SPRINTRON_KEYFOB_CLIENT_TX_POWER_UUID )
};

// Sprintron Keyfob Out Of Range Threshold UUID
CONST uint8 skOutOfRangeThresholdUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16( SPRINTRON_KEYFOB_OUT_OF_RANGE_THRESHOLD_UUID ), HI_UINT16( SPRINTRON_KEYFOB_OUT_OF_RANGE_THRESHOLD_UUID )
};

// Sprintron Keyfob Out Of Range Status UUID
CONST uint8 skOutOfRangeStatusUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16( SPRINTRON_KEYFOB_OUT_OF_RANGE_STATUS_UUID ), HI_UINT16( SPRINTRON_KEYFOB_OUT_OF_RANGE_STATUS_UUID )
};

// Sprintron Keyfob Beep Status UUID
CONST uint8 skBeepStatusUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16( SPRINTRON_KEYFOB_BEEP_STATUS_UUID ), HI_UINT16( SPRINTRON_KEYFOB_BEEP_STATUS_UUID )
};

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
static sprintronKeyfobCBs_t *sk_AppCBs = NULL;

/*********************************************************************
 * Profile Attributes - variables
 */

static CONST gattAttrType_t sprintronKeyfobService = { ATT_BT_UUID_SIZE, sprintronKeyfobServiceUUID };

static uint8 sprintronKeyfobServerRssiCharProps = GATT_PROP_READ | GATT_PROP_NOTIFY;
static int8 sprintronKeyfobServerRssi = SERVER_RSSI_DEFAULT_VALUE;
static gattCharCfg_t sprintronKeyfobServerRssiConfig[GATT_MAX_NUM_CONN];

static uint8 sprintronKeyfobClientTxPowerCharProps = GATT_PROP_READ | GATT_PROP_WRITE;
static int8 sprintronKeyfobClientTxPower = CLIENT_TX_POWER_DEFAULT_VALUE;

static uint8 sprintronKeyfobOutOfRangeThresholdCharProps = GATT_PROP_READ | GATT_PROP_WRITE;
static int8 sprintronKeyfobOutOfRangeThreshold = OUT_OF_RANGE_THRESHOLD_DEFAULT_VALUE;

static uint8 sprintronKeyfobOutOfRangeStatusCharProps = GATT_PROP_READ | GATT_PROP_NOTIFY;
static uint8 sprintronKeyfobOutOfRangeStatus = OUT_OF_RANGE_STATUS_IN_RANGE;
static gattCharCfg_t sprintronKeyfobOutOfRangeConfig[GATT_MAX_NUM_CONN];

static uint8 sprintronKeyfobBeepStatusCharProps = GATT_PROP_READ | GATT_PROP_WRITE;
static uint8 sprintronKeyfobBeepStatus = BEEP_STATUS_NONE;

/*********************************************************************
 * Profile Attributes - Table
 */
// Link Loss Service Atttribute Table
static gattAttribute_t sprintronKeyfobAttrTbl[] = 
{
  // Sprintron Keyfob service 
  {
    { ATT_BT_UUID_SIZE, primaryServiceUUID },
	GATT_PERMIT_READ,
	0,
	(uint8 *)&sprintronKeyfobService
  },
/////////// Server RSSI ////////////
    // Characteristic Declaration
    {
      {ATT_BT_UUID_SIZE, characterUUID},
	  GATT_PERMIT_READ,
	  0,
	  (uint8 *)&sprintronKeyfobServerRssiCharProps
    },
	  // Server RSSI
	  { 
	    { ATT_BT_UUID_SIZE, skServerRssiUUID },
	    GATT_PERMIT_READ, 
	    0, 
	    (uint8 *)&sprintronKeyfobServerRssi 
	  },
      // Characteristic configuration
      { 
	    { ATT_BT_UUID_SIZE, clientCharCfgUUID },
	    GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
	    0, 
	    (uint8 *)sprintronKeyfobServerRssiConfig 
	  },
/////////// Client tx power ////////////
    // Characteristic Declaration
    {
      {ATT_BT_UUID_SIZE, characterUUID},
      GATT_PERMIT_READ,
      0,
      (uint8 *)&sprintronKeyfobClientTxPowerCharProps
    },
	  // Client tx power
	  { 
	    { ATT_BT_UUID_SIZE, skClientTxPowerUUID },
	    GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
	    0, 
	    (uint8 *)&sprintronKeyfobClientTxPower 
	  },
/////////// Out of range threshold ////////////
    // Characteristic Declaration
    {
      {ATT_BT_UUID_SIZE, characterUUID},
      GATT_PERMIT_READ,
      0,
      (uint8 *)&sprintronKeyfobOutOfRangeThresholdCharProps
    },
	  // Out of range threshold
	  { 
	    { ATT_BT_UUID_SIZE, skOutOfRangeThresholdUUID },
	    GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
	    0, 
	    (uint8 *)&sprintronKeyfobOutOfRangeThreshold 
	  },
/////////// Out of range status ////////////
    // Characteristic Declaration
    {
      {ATT_BT_UUID_SIZE, characterUUID},
      GATT_PERMIT_READ,
      0,
      (uint8 *)&sprintronKeyfobOutOfRangeStatusCharProps
    },
	  // Out of range status
	  { 
	    { ATT_BT_UUID_SIZE, skOutOfRangeStatusUUID },
	    GATT_PERMIT_READ, 
	    0, 
	    (uint8 *)&sprintronKeyfobOutOfRangeStatus 
	  },
	  // Characteristic configuration
	  { 
	    { ATT_BT_UUID_SIZE, clientCharCfgUUID },
	    GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
	    0, 
	    (uint8 *)sprintronKeyfobOutOfRangeConfig 
	  },
/////////// Server beep status ////////////
    // Characteristic Declaration
    {
      {ATT_BT_UUID_SIZE, characterUUID},
      GATT_PERMIT_READ,
      0,
      (uint8 *)&sprintronKeyfobBeepStatusCharProps
    },
	  // Server beep status
	  { 
	    { ATT_BT_UUID_SIZE, skBeepStatusUUID },
	    GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
	    0, 
	    (uint8 *)&sprintronKeyfobBeepStatus 
	  },
}

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static uint8 sprintronKeyfob_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr, 
                                    uint8 *pValue, uint8 *pLen, uint16 offset, uint8 maxLen );
static bStatus_t sprintronKeyfob_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                         uint8 *pValue, uint8 len, uint16 offset );

/*********************************************************************
 * PROFILE CALLBACKS
 */
// Prox Reporter Service Callbacks
CONST gattServiceCBs_t sprintronKeyfobCBs =
{
  sprintronKeyfob_ReadAttrCB,  // Read callback function pointer
  sprintronKeyfob_WriteAttrCB, // Write callback function pointer
  NULL                      // Authorization callback function pointer
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      sprintronKeyfob_AddService
 *
 * @brief   Initializes the Proximity Reporter service by
 *          registering GATT attributes with the GATT server.
 *          Only call this function once.
 *
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 *
 * @return   Success or Failure
 */
bStatus_t SprintronKeyfob_AddService( uint32 services )
{
  uint8 status = SUCCESS;

  // Currently this profile only contain 1 service
  if ( services & SPRINTRON_KEYFOB_SERVICE )
  {
    // Initialize Client Characteristic Configuration attributes
    GATTServApp_InitCharCfg( INVALID_CONNHANDLE, sprintronKeyfobServerRssiConfig );

    GATTServApp_InitCharCfg( INVALID_CONNHANDLE, sprintronKeyfobOutOfRangeConfig );

    // Register Sprintron Keyfob attribute list and CBs with GATT Server App  
    status = GATTServApp_RegisterService( sprintronKeyfobAttrTbl, 
                                          GATT_NUM_ATTRS( sprintronKeyfobAttrTbl ),
                                          &sprintronKeyfobCBs );
  }

  return ( status );
}


/*********************************************************************
 * @fn      sprintronKeyfob_RegisterAppCBs
 *
 * @brief   Registers the application callback function. Only call 
 *          this function once.
 *
 * @param   callbacks - pointer to application callbacks.
 *
 * @return  SUCCESS or bleAlreadyInRequestedMode
 */
bStatus_t sprintronKeyfob_RegisterAppCBs( sprintronKeyfobCBs_t *appCallbacks )
{
  if ( appCallbacks )
  {
    sk_AppCBs = appCallbacks;
    
    return ( SUCCESS );
  }
  else
  {
    return ( bleAlreadyInRequestedMode );
  }
}
   

/*********************************************************************
 * @fn      sprintronKeyfob_SetParameter
 *
 * @brief   Set a Proximity Reporter parameter.
 *
 * @param   param - Profile parameter ID
 * @param   len - length of data to right
 * @param   value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t sprintronKeyfob_SetParameter( uint8 param, uint8 len, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
    case SPRINTRON_KEYFOB_SERVER_RSSI:
      if ( len == sizeof ( int8 ) ) 
      {
        sprintronKeyfobServerRssi = *((int8*)value);
		
        // See if Notification has been enabled
        GATTServApp_ProcessCharCfg( sprintronKeyfobServerRssiConfig, (int8 *)&sprintronKeyfobServerRssi, FALSE, 
                                    sprintronKeyfobAttrTbl, GATT_NUM_ATTRS( sprintronKeyfobAttrTbl ),
                                    INVALID_TASK_ID );
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

	case SPRINTRON_KEYFOB_OUT_OF_RANGE_THRESHOLD:
      if ( len == sizeof ( int8 ) ) 
      {
        sprintronKeyfobClientTxPower = *((int8*)value);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case SPRINTRON_KEYFOB_CLIENT_TX_POWER:
      if ( len == sizeof ( int8 ) ) 
      {
        sprintronKeyfobOutOfRangeThreshold = *((int8*)value);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;
	  
	case SPRINTRON_KEYFOB_OUT_OF_RANGE_STATUS:
      if ( (len == sizeof ( uint8 )) && ((*((uint8*)value) <= OUT_OF_RANGE_STATUS_OUT_OF_RANGE)) ) 
      {
        sprintronKeyfobOutOfRangeStatus = *((uint8*)value);
		
        // See if Notification has been enabled
        GATTServApp_ProcessCharCfg( sprintronKeyfobOutOfRangeConfig, (uint8 *)&sprintronKeyfobOutOfRangeStatus, FALSE, 
                                    sprintronKeyfobAttrTbl, GATT_NUM_ATTRS( sprintronKeyfobAttrTbl ),
                                    INVALID_TASK_ID );
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

	case SPRINTRON_KEYFOB_BEEP_STATUS:
      if ( (len == sizeof ( uint8 )) && ((*((uint8*)value) <= BEEP_STATUS_HIGH)) ) 
      {
        sprintronKeyfobBeepStatus = *((uint8*)value);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    default:
      ret = INVALIDPARAMETER;
      break;
  }
  
  return ( ret );
}

/*********************************************************************
 * @fn      sprintronKeyfob_GetParameter
 *
 * @brief   Get a Proximity Reporter parameter.
 *
 * @param   param - Profile parameter ID
 * @param   value - pointer to data to put.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t sprintronKeyfob_GetParameter( uint8 param, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
    case SPRINTRON_KEYFOB_SERVER_RSSI:
      *((int8*)value) = sprintronKeyfobServerRssi;
      break;
      
    case SPRINTRON_KEYFOB_CLIENT_TX_POWER:
      *((int8*)value) = sprintronKeyfobClientTxPower;
      break;
      
    case SPRINTRON_KEYFOB_OUT_OF_RANGE_THRESHOLD:
      *((int8*)value) = sprintronKeyfobOutOfRangeThreshold;
      break;

    case SPRINTRON_KEYFOB_OUT_OF_RANGE_STATUS:
      *((uint8*)value) = sprintronKeyfobOutOfRangeStatus;
      break;

    case SPRINTRON_KEYFOB_BEEP_STATUS:
      *((uint8*)value) = sprintronKeyfobBeepStatus;
      break;

    default:
      ret = INVALIDPARAMETER;
      break;
  }
  
  return ( ret );
}

/*********************************************************************
 * @fn          sprintronKeyfob_ReadAttrCB
 *
 * @brief       Read an attribute.
 *
 * @param       
 *
 * @return      Success or Failure
 */
static uint8 sprintronKeyfob_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr, 
                                    uint8 *pValue, uint8 *pLen, uint16 offset, uint8 maxLen )
{
  uint16 uuid;
  bStatus_t status = SUCCESS;

  // Make sure it's not a blob operation
  if ( offset > 0 )
  {
    return ( ATT_ERR_ATTR_NOT_LONG );
  }  

  if ( pAttr->type.len == ATT_BT_UUID_SIZE )
  {
    // 16-bit UUID
    uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);
    
    switch ( uuid )
    {
      // No need for "GATT_SERVICE_UUID" or "GATT_CLIENT_CHAR_CFG_UUID" cases;
      // gattserverapp handles those types for reads 
      case SPRINTRON_KEYFOB_SERVER_RSSI_UUID:
      case SPRINTRON_KEYFOB_CLIENT_TX_POWER_UUID:
      case SPRINTRON_KEYFOB_OUT_OF_RANGE_THRESHOLD_UUID:
      case SPRINTRON_KEYFOB_OUT_OF_RANGE_STATUS_UUID:
      case SPRINTRON_KEYFOB_BEEP_STATUS_UUID:
        *pLen = 1;
        pValue[0] = *pAttr->pValue;
        break;
      
      default:
        // Should never get here!
        *pLen = 0;
        status = ATT_ERR_ATTR_NOT_FOUND;
        break;
    }
  }
  else
  {
    //128-bit UUID
    *pLen = 0;
    status = ATT_ERR_INVALID_HANDLE;
  }

  return ( status );
}

/*********************************************************************
 * @fn      sprintronKeyfob_WriteAttrCB
 *
 * @brief   Validate attribute data prior to a write operation
 *
 * @param   connHandle – connection message was received on
 * @param   pReq - pointer to request
 *
 * @return  Success or Failure
 */
static bStatus_t sprintronKeyfob_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                                 uint8 *pValue, uint8 len, uint16 offset )
{
  bStatus_t status = SUCCESS;
  uint8 notify = 0xFF;

  if ( pAttr->type.len == ATT_BT_UUID_SIZE )
  { 
    // 16-bit UUID
    uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);
    switch ( uuid )
    {
      case SPRINTRON_KEYFOB_SERVER_RSSI_UUID:
      case SPRINTRON_KEYFOB_CLIENT_TX_POWER_UUID:
      case SPRINTRON_KEYFOB_OUT_OF_RANGE_THRESHOLD_UUID:
        // Validate the value
        // Make sure it's not a blob operation
        if ( offset == 0 )
        {
          if ( len > 1 )
            status = ATT_ERR_INVALID_VALUE_SIZE;
        }
        else
        {
          status = ATT_ERR_ATTR_NOT_LONG;
        }
        
        //Write the value
        if ( status == SUCCESS )
        {
          int8 *pCurValue = (int8 *)pAttr->pValue;
          
          *pCurValue = pValue[0];
          if ( pAttr->pValue == &sprintronKeyfobServerRssi )
            notify = SPRINTRON_KEYFOB_SERVER_RSSI;        
          else if ( pAttr->pValue == &sprintronKeyfobClientTxPower )
            notify = SPRINTRON_KEYFOB_CLIENT_TX_POWER;   
          else // if ( pAttr->pValue == &sprintronKeyfobOutOfRangeThreshold )
            notify = SPRINTRON_KEYFOB_OUT_OF_RANGE_THRESHOLD;     			
        }
        
        break;

      case SPRINTRON_KEYFOB_OUT_OF_RANGE_STATUS_UUID:
      case SPRINTRON_KEYFOB_BEEP_STATUS_UUID:
        // Validate the value
        // Make sure it's not a blob operation
        if ( offset == 0 )
        {
          if ( len > 1 )
            status = ATT_ERR_INVALID_VALUE_SIZE;
          else
          {
            if ( pAttr->pValue == &sprintronKeyfobOutOfRangeStatus && pValue[0] > OUT_OF_RANGE_STATUS_OUT_OF_RANGE )
              status = ATT_ERR_INVALID_VALUE;
            else if ( pAttr->pValue == &sprintronKeyfobBeepStatus && pValue[0] > BEEP_STATUS_HIGH )
              status = ATT_ERR_INVALID_VALUE;
          }
        }
        else
        {
          status = ATT_ERR_ATTR_NOT_LONG;
        }
        
        //Write the value
        if ( status == SUCCESS )
        {
          uint8 *pCurValue = (uint8 *)pAttr->pValue;
          
          *pCurValue = pValue[0];
          if ( pAttr->pValue == &sprintronKeyfobOutOfRangeStatus )
            notify = SPRINTRON_KEYFOB_OUT_OF_RANGE_STATUS;        
          else // if ( pAttr->pValue == &sprintronKeyfobBeepStatus )
            notify = SPRINTRON_KEYFOB_BEEP_STATUS;    			
        }
        
        break;

      case GATT_CLIENT_CHAR_CFG_UUID:
        status = GATTServApp_ProcessCCCWriteReq( connHandle, pAttr, pValue, len,
                                                 offset, GATT_CLIENT_CFG_NOTIFY );
        break;
        
      default:
        // Should never get here!
        status = ATT_ERR_ATTR_NOT_FOUND;
        break;
    }
  }
  else
  {
    // 128-bit UUID
    status = ATT_ERR_INVALID_HANDLE;
  }    
  
  // If an attribute changed then callback function to notify application of change
  if ( (notify != 0xFF) && sk_AppCBs && sk_AppCBs->pfnAttrChange )
    sk_AppCBs->pfnAttrChange( notify );

  return ( status );
}

/*********************************************************************
*********************************************************************/

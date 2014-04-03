/**************************************************************************************************
  Filename:       keyfobdemo.h
  Revised:        $Date: 2012-10-11 12:11:30 -0700 (Thu, 11 Oct 2012) $
  Revision:       $Revision: 31784 $

  Description:    This file contains Key Fob Demo Application header file.


  Copyright 2009 - 2011  Texas Instruments Incorporated. All rights reserved.

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

#ifndef SPRINTRON__KEYFOB__APP_H
#define SPRINTRON__KEYFOB__APP_H

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

#define USE_WHITE_LIST_ADV                                1

#define MAX_WHITELIST_LEN                                 8

// Key Fob Task Events
#define KFD_START_DEVICE_EVT                              0x0001
#define KFD_BATTERY_CHECK_EVT                             0x0002
#define KFD_TOGGLE_BUZZER_EVT                             0x0008
#define KFD_ADV_IN_CONNECTION_EVT                         0x0010
#define KFD_POWERON_LED_TIMEOUT_EVT                       0x0020
#define KFD_NON_WHITELIST_STOP_EVT                        0x0040
#define KFD_NON_WHITELIST_START_EVT                       0x0080
#define KFD_WHITELIST_START_EVT                           0x0100
#define KFD_CONNECTION_INTERVAL_EVT                       0x0200
#define KFD_BUZZER_ALERT_TIME_EXPIRED_EVT                 0x0400
#define KFD_LED_ALERT_TIME_EXPIRED_EVT                    0x0800
#define KFD_BOND_NOT_COMPLETE_IN_TIME_EVT                 0x1000
#define KFD_LONG_PRESS_COMPLETE_EVT                       0x2000
#define KFD_SHORT_LONG_PRESS_NOTIFY_COMPLETE_EVT          0x4000

// Key press event type: pressed or released
#define KEY_IS_PRESSED                                    0x0
#define KEY_IS_RELEASED                                   0x1

// States of key press state machine
#define NOT_PRESSED                                       0x0
#define PRESSED_COUNTING                                  0x1
#define SHORT_PRESS_ACCHIEVED                             0x2
#define LONG_PRESS_ACCHIEVED                              0x3

// Key Fob NV Item ID
// According to osal api document, application can use item id range from 0x80~0xfe, we choose something in between.

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * FUNCTIONS
 */

/*
 * Task Initialization for the BLE Application
 */
extern void KeyFobApp_Init( uint8 task_id );

/*
 * Task Event Processor for the BLE Application
 */
extern uint16 KeyFobApp_ProcessEvent( uint8 task_id, uint16 events );

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* SPRINTRON__KEYFOB__APP_H */

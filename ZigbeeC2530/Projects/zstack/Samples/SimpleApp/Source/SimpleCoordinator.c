
/**************************************************************************************************
  Filename:       SimpleApp.c
  Revised:        $Date: 2007-12-04 11:46:18 -0800 (Tue, 04 Dec 2007) $
  Revision:       $Revision: 16007 $

  Description:    Sample application utilizing the Simple API.


  Copyright 2007 Texas Instruments Incorporated. All rights reserved.

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
  PROVIDED 揂S IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, 
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

/******************************************************************************
 * INCLUDES
 */

#include "ZComDef.h"
#include "OSAL.h"
#include "sapi.h"
#include "hal_key.h"
#include "hal_led.h"
#include "DebugTrace.h"
#include "SimpleApp.h"

#if defined( MT_TASK )
  #include "osal_nv.h"
#endif
#include "mt_app.h"
#include "mt_uart.h"
#include "mt.h"


/*********************************************************************
 * CONSTANTS
 */

// Application States
#define APP_INIT                           0
#define APP_START                          1

// Application osal event identifiers
#define MY_START_EVT                0x0001

// Same definitions as in SimpleSensor.c
#define TEMP_REPORT     0x01
#define BATTERY_REPORT 0x02
/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

static uint8 myAppState = APP_INIT;
static uint8 myStartRetryDelay = 10;

#if defined( MT_TASK )
extern uint8 aExtendedAddress[8];
#endif

void zb_HanderMsg(osal_event_hdr_t *pMsg);

static void processCommand(uint16 cmd, byte *dat, uint8 len);
/*********************************************************************
 * GLOBAL VARIABLES
 */

// Inputs and Outputs for Switch device
#define NUM_OUT_CMD_COLLECTOR                2 //定义路由器输出指令数
#define NUM_IN_CMD_COLLECTOR                 3 //定义路由器输入指令数

// List of output and input commands for Switch device
const cId_t zb_InCmdList[NUM_IN_CMD_COLLECTOR] =
{
    ID_CMD_READ_RES,
    ID_CMD_WRITE_RES,
    ID_CMD_REPORT,
};
const cId_t zb_OutCmdList[NUM_OUT_CMD_COLLECTOR] =
{
  ID_CMD_READ_REQ,
  ID_CMD_WRITE_REQ,
};

// Define SimpleDescriptor for Switch device
const SimpleDescriptionFormat_t zb_SimpleDesc =
{
  MY_ENDPOINT_ID,             //  Endpoint
  MY_PROFILE_ID,              //  Profile ID
  DEV_ID_COLLECTOR,          //  Device ID
  DEVICE_VERSION_COLLECTOR,  //  Device Version
  0,                          //  Reserved
  NUM_IN_CMD_COLLECTOR,      //  Number of Input Commands
  (cId_t *) zb_InCmdList,     //  Input Command List
  NUM_OUT_CMD_COLLECTOR,     //  Number of Output Commands
  (cId_t *) zb_OutCmdList              //  Output Command List
};

/******************************************************************************
 * @fn          zb_HandleOsalEvent
 *
 * @brief       The zb_HandleOsalEvent function is called by the operating
 *              system when a task event is set
 *
 * @param       event - Bitmask containing the events that have been set
 *
 * @return      none
 */
void zb_HandleOsalEvent( uint16 event )
{
   uint8 startOptions;
  uint8 logicalType;
  if (event & ZB_ENTRY_EVENT) {
        zb_ReadConfiguration( ZCD_NV_LOGICAL_TYPE, sizeof(uint8), &logicalType );
        if ( logicalType != ZG_DEVICETYPE_COORDINATOR )
        {
          logicalType = ZG_DEVICETYPE_COORDINATOR;
          zb_WriteConfiguration(ZCD_NV_LOGICAL_TYPE, sizeof(uint8), &logicalType);
        }

        // Do more configuration if necessary and then restart device with auto-start bit set
        // write endpoint to simple desc...dont pass it in start req..then reset


        zb_ReadConfiguration( ZCD_NV_STARTUP_OPTION, sizeof(uint8), &startOptions );
        if (startOptions != ZCD_STARTOPT_AUTO_START) {
          startOptions = ZCD_STARTOPT_AUTO_START;
          zb_WriteConfiguration( ZCD_NV_STARTUP_OPTION, sizeof(uint8), &startOptions );
        }
  }
  
}

//自定义用户消息处理事件
void zb_HanderMsg(osal_event_hdr_t *msg)
{
  mtSysAppMsg_t *pMsg = (mtSysAppMsg_t*)msg;
  
  uint16 dAddr;
  uint16 cmd;
  uint16 addr = NLME_GetShortAddr();//获取本身短地址
  
  HalLedSet( HAL_LED_1, HAL_LED_MODE_OFF );
  HalLedSet( HAL_LED_1, HAL_LED_MODE_BLINK );
  if (pMsg->hdr.event == MT_SYS_APP_MSG) {
    //if (pMsg->appDataLen < 4) return;
    dAddr = pMsg->appData[0]<<8 | pMsg->appData[1];//data0-data1是16位的短地址号
    cmd = pMsg->appData[2]<<8 | pMsg->appData[3];//data2-data3是控制指令
    if (dAddr == addr) {//如果此地址与本身地址一样，就进行对应处理
      processCommand(cmd, pMsg->appData+4, pMsg->appDataLen-4);
    } else {//反之，把这个信息转发给相应地址的设备
      zb_SendDataRequest(dAddr, cmd, pMsg->appDataLen-4, pMsg->appData+4, 0, AF_ACK_REQUEST, 0 );
    }
  }
}

/*********************************************************************
 * @fn      zb_HandleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 EVAL_SW4
 *                 EVAL_SW3
 *                 EVAL_SW2
 *                 EVAL_SW1
 *
 * @return  none
 */
void zb_HandleKeys( uint8 shift, uint8 keys )
{
 
  
}
/******************************************************************************
 * @fn          zb_StartConfirm
 *
 * @brief       The zb_StartConfirm callback is called by the ZigBee stack
 *              after a start request operation completes
 *
 * @param       status - The status of the start operation.  Status of
 *                       ZB_SUCCESS indicates the start operation completed
 *                       successfully.  Else the status is an error code.
 *
 * @return      none
 */
//开始启动事件，如果事件成功则不重启MY_START_EVT
void zb_StartConfirm( uint8 status )
{

 // If the device sucessfully started, change state to running
  if ( status == ZB_SUCCESS )
  {
    myAppState = APP_START;
    //zb_AllowBind(0xff);
  }
  else
  {
    // Try again later with a delay
    osal_start_timerEx( sapi_TaskID, MY_START_EVT, myStartRetryDelay );
  }
}
/******************************************************************************
 * @fn          zb_SendDataConfirm
 *
 * @brief       The zb_SendDataConfirm callback function is called by the
 *              ZigBee after a send data operation completes
 *
 * @param       handle - The handle identifying the data transmission.
 *              status - The status of the operation.
 *
 * @return      none
 */
void zb_SendDataConfirm( uint8 handle, uint8 status )
{
}
/******************************************************************************
 * @fn          zb_BindConfirm
 *
 * @brief       The zb_BindConfirm callback is called by the ZigBee stack
 *              after a bind operation completes.
 *
 * @param       commandId - The command ID of the binding being confirmed.
 *              status - The status of the bind operation.
 *
 * @return      none
 */
void zb_BindConfirm( uint16 commandId, uint8 status )
{
}
/******************************************************************************
 * @fn          zb_AllowBindConfirm
 *
 * @brief       Indicates when another device attempted to bind to this device
 *
 * @param
 *
 * @return      none
 */
void zb_AllowBindConfirm( uint16 source )
{
}
/******************************************************************************
 * @fn          zb_FindDeviceConfirm
 *
 * @brief       The zb_FindDeviceConfirm callback function is called by the
 *              ZigBee stack when a find device operation completes.
 *
 * @param       searchType - The type of search that was performed.
 *              searchKey - Value that the search was executed on.
 *              result - The result of the search.
 *
 * @return      none
 */
void zb_FindDeviceConfirm( uint8 searchType, uint8 *searchKey, uint8 *result )
{
}
/******************************************************************************
 * @fn          zb_ReceiveDataIndication
 *
 * @brief       The zb_ReceiveDataIndication callback function is called
 *              asynchronously by the ZigBee stack to notify the application
 *              when data is received from a peer device.
 *
 * @param       source - The short address of the peer device that sent the data
 *              command - The commandId associated with the data
 *              len - The number of bytes in the pData parameter
 *              pData - The data sent by the peer device
 *
 * @return      none
 */
//接收节点发回来的数据，处理后由串口给上位机
void zb_ReceiveDataIndication( uint16 source, uint16 command, uint16 len, uint8 *pData  )
{
    
    HalLedSet( HAL_LED_1, HAL_LED_MODE_OFF );
    HalLedSet( HAL_LED_1, HAL_LED_MODE_BLINK );
    mtOSALSerialData_t* msg = (mtOSALSerialData_t*)osal_msg_allocate(sizeof(mtOSALSerialData_t)+len+4);
    if (msg) {
      msg->hdr.event = MT_SYS_APP_RSP_MSG;
      msg->hdr.status = len+4;
      msg->msg = (byte*)(msg+1);
      msg->msg[0] = (source>>8)&0xff;
      msg->msg[1] = source&0xff;
      msg->msg[2] = (command>>8)&0xff;
      msg->msg[3] = command&0xff;
      osal_memcpy(msg->msg+4, pData, len);
      osal_msg_send( MT_TaskID, (uint8 *)msg );
    } 
}











/******************************************************************************
 * @fn          myApp_ReadTemperature
 *
 * @brief       Reports temperature sensor reading
 *
 * @param
 *
 * @return
 */


static int paramWrite(uint16 pid, byte *dat)
{
  int len = 0;
  switch (pid) {
  default:
    break;
  }
  return len;
}

static int paramRead(uint16 pid, byte *dat)
{
  int len = 0;
  switch (pid) {
  case 0x0001:
    dat[0] = 0x11; dat[1] = 0x33;
    len = 2;
    break;
  case 0x0002:
    dat[0] = 0x22; dat[1] = 0x44;
    len = 2;
    break;
  case 0x0003:
    dat[0] = 0x00; dat[1] = 0x01;
    len = 2;
    break;
  case 0x0004:
    dat[0] = dat[1] = dat[2] = dat[3] = dat[4] = dat[5] = 1;
    len = 6;
    break;
  case 0x0005:
    dat[0] = DEV_ID_COLLECTOR;
    len = 1;
    break;
  /* -----------  网络参数 ------------------- */  
  case 0x0014: //mac地址
     /*osal_nv_read( ZCD_NV_EXTADDR, 0, Z_EXTADDR_LEN, pBuf ); rm by liren */
    ZMacGetReq( ZMacExtAddr, dat ); // add by liren
    // Outgoing extended address needs to be reversed
    MT_ReverseBytes( dat, Z_EXTADDR_LEN );
    len = Z_EXTADDR_LEN;
    break;
  case 0x0015:
  {
      uint8 assocCnt = 0;
      uint16 *assocList;
      int i;
#if defined(RTR_NWK) && !defined( NONWK )
    assocList = AssocMakeList( &assocCnt );
#else
    assocCnt = 0;
    assocList = NULL;
#endif
    dat[0] = assocCnt;
    for (i=0; i<assocCnt&&i<16; i++) {
      dat[1+2*i] = HI_UINT16(assocList[i]);
      dat[1+2*i+1] = LO_UINT16(assocList[i]);
    }
    len = 1 + 2 * assocCnt;
    break;
  }
 /* ------------------------------------ */

  }
  return len;
}

static void processCommand(uint16 cmd, byte *pData, uint8 len)
{
  int i;
  uint16 pid;
  byte dat[64];
  byte rlen = 1;
  int ret;

  switch (cmd) {
  case ID_CMD_WRITE_REQ:
    for (i=0; i<len; i+=2) {
      pid = pData[i]<<8 | pData[i+1];
      ret = paramWrite(pid, pData+2);
      if (ret <= 0) {
        dat[0] = 1;
        zb_ReceiveDataIndication( 0, ID_CMD_WRITE_RES, 1, dat );
        return;
      } 
      i += ret;
    }
    dat[0] = 0;
    zb_ReceiveDataIndication( 0, ID_CMD_WRITE_RES, 1, dat);
    break;
  case ID_CMD_READ_REQ:
    for (i=0; i<len; i+=2) {
      pid = pData[i]<<8 | pData[i+1];
      dat[rlen++] = pData[i];
      dat[rlen++] = pData[i+1];
      ret = paramRead(pid, dat+rlen);
      if (ret <= 0) {
        dat[0] = 1;
        zb_ReceiveDataIndication( 0, ID_CMD_READ_RES, 1, dat );
        return;
      }
      rlen += ret;
    }
    dat[0] = 0;
    zb_ReceiveDataIndication( 0, ID_CMD_READ_RES, rlen, dat );
    break;
  }  
}

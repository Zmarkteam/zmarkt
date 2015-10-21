
/**************************************************************************************************
  Filename:       SimpleSwitch.c
  Revised:        $Date: 2007-10-27 17:16:54 -0700 (Sat, 27 Oct 2007) $
  Revision:       $Revision: 15793 $

  Description:    Sample application for a simple light switch utilizing the Simple API.

 
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
#include "hal_adc.h"
#include "hal_mcu.h"
#include "SimpleApp.h"


#include "mt.h"

/*********************************************************************
 * CONSTANTS
 */

// Application States
#define APP_INIT                           0    // Initial state(初始状态)
#define APP_START                          1    // Sensor has joined network(传感器已加入网络)
#define APP_BOUND                          2    // Sensor is bound to collector(传感器也绑定到适配器)

// Application osal event identifiers
// Bit mask of events ( from 0x0000 to 0x00FF )
#define MY_START_EVT                0x0001      //自定义的开始事件
#define MY_REPORT_EVT               0x0002     //自定义的上报事件

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

static uint8 myAppState = APP_INIT;           //myAppState:自定义的APP状态

static uint16 myStartRetryDelay = 10000;      // milliseconds

static uint16 myCheckCardDelay = 250;         //milliseconds

static uint8 rfidOn = 1;                     //RFID的开启和关闭
/*********************************************************************
 * GLOBAL VARIABLES
 */

// Inputs and Outputs for Switch device
#define NUM_IN_CMD_SENSOR                 2       //传感器模式:输入模式
#define NUM_OUT_CMD_SENSOR                3       //传感器模式:输出模式


const cId_t zb_InCmdList[NUM_IN_CMD_SENSOR] =
{
  ID_CMD_READ_REQ,                               //读中断
  ID_CMD_WRITE_REQ,                              //写中断
};
const cId_t zb_OutCmdList[NUM_OUT_CMD_SENSOR] =
{
    ID_CMD_READ_RES,
    ID_CMD_WRITE_RES,
    ID_CMD_REPORT,
};





// Define SimpleDescriptor for Switch device
const SimpleDescriptionFormat_t zb_SimpleDesc =
{
  MY_ENDPOINT_ID,             //  Endpoint
  MY_PROFILE_ID,              //  Profile ID
  DEV_ID_SENSOR,              //  Device ID
  DEVICE_VERSION_SENSOR,      //  Device Version
  0,                          //  Reserved
  NUM_IN_CMD_SENSOR,          //  Number of Input Commands
  (cId_t *) zb_InCmdList,             //  Input Command List
  NUM_OUT_CMD_SENSOR,         //  Number of Output Commands
  (cId_t *) zb_OutCmdList     //  Output Command List
};

/********************************************************
 *
 */
void zb_HanderMsg(osal_event_hdr_t *msg);

/*********************************************************************
 * LOCAL FUNCTIONS
 */


static int paramWrite(uint16 pid, byte *dat);
static int paramRead(uint16 pid, byte *dat);


void rfid_uart_init(void);
void rfid_uart_send(unsigned char ch);
int  rfid_uart_recv(void);
void rfid_commands(char *cmd, int len);
char datxor(char *dat, int len);
int rfid_id(char *id);

/*****************************************************************************
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
 
   if (event & ZB_ENTRY_EVENT) {
        uint8 startOptions;
        uint8 logicalType;
  
        zb_ReadConfiguration( ZCD_NV_LOGICAL_TYPE, sizeof(uint8), &logicalType );
        if ( logicalType != ZG_DEVICETYPE_ENDDEVICE )
        //if ( logicalType != ZG_DEVICETYPE_ROUTER )
        //if ( logicalType != ZG_DEVICETYPE_COORDINATOR )
        {
          logicalType = ZG_DEVICETYPE_ENDDEVICE;
          //logicalType = ZG_DEVICETYPE_ROUTER;
          //logicalType = ZG_DEVICETYPE_COORDINATOR;
          zb_WriteConfiguration(ZCD_NV_LOGICAL_TYPE, sizeof(uint8), &logicalType);
        }

        // Do more configuration if necessary and then restart device with auto-start bit set
        // write endpoint to simple desc...dont pass it in start req..then reset
        zb_ReadConfiguration( ZCD_NV_STARTUP_OPTION, sizeof(uint8), &startOptions );
        if (startOptions != ZCD_STARTOPT_AUTO_START) {
          startOptions = ZCD_STARTOPT_AUTO_START;
          zb_WriteConfiguration( ZCD_NV_STARTUP_OPTION, sizeof(uint8), &startOptions );
        } 
        
        rfid_uart_init();
        
        osal_start_timerEx( sapi_TaskID, MY_REPORT_EVT, myCheckCardDelay );
  }
  
  if ( event & MY_START_EVT )
  {  
    zb_StartRequest();
  }

  if (event & MY_REPORT_EVT) {

    if (rfidOn) {
      static char card = 0;
      static char id[4];
      
      //rfid_uart_init();
      
      if (rfid_id(id)) {
        if (card == 0) {
          card = 1;
          HalLedSet( HAL_LED_1, HAL_LED_MODE_ON );
          if (myAppState == APP_START) {
            unsigned char dat[6];
            dat[0] = 0x08;
            dat[1] = 0x10;
            dat[2] = id[0];
            dat[3] = id[1];
            dat[4] = id[2];
            dat[5] = id[3];
            /*0：为要上报数据给地址为0的节点*/
            /*ID_CMD_REPORT:为命令事件*/
            /*6：为数据长度*/
            /*dat:为数据*/
            /*0, AF_ACK_REQUEST, 0为默认值*/
            zb_SendDataRequest(0, ID_CMD_REPORT, 6, dat, 0, AF_ACK_REQUEST, 0 );
          }
        }
        myCheckCardDelay = 1000;
      } else {
        if (card) {
          card = 0;
          HalLedSet( HAL_LED_1, HAL_LED_MODE_OFF );
        }
        myCheckCardDelay = 250;
      }
    }   
    osal_start_timerEx( sapi_TaskID, MY_REPORT_EVT, myCheckCardDelay );
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
void zb_StartConfirm( uint8 status )
{
  if ( status == ZB_SUCCESS )
  {
    myAppState = APP_START;
    // Set event to bind to a collector
    //osal_start_timerEx( sapi_TaskID, MY_FIND_COLLECTOR_EVT, myBindRetryDelay );
  }
  else
  {
    // Try joining again later with a delay
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
  if ( status != ZSuccess )
  {
    // Remove bindings to the existing collector
  }
  else
  {
    // send data ??
  }
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


void zb_HanderMsg(osal_event_hdr_t *msg)
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
void zb_ReceiveDataIndication( uint16 source, uint16 command, uint16 len, uint8 *pData  )
{
  int i;
  uint16 pid;
  byte dat[64];
  byte rlen = 1;
  int ret;
  
  HalLedSet( HAL_LED_1, HAL_LED_MODE_OFF );
  HalLedSet( HAL_LED_1, HAL_LED_MODE_BLINK ); 
  switch (command) {
  case ID_CMD_WRITE_REQ:
    for (i=0; i<len; i+=2) {
      pid = pData[i]<<8 | pData[i+1];
      ret = paramWrite(pid, pData+2);
      if (ret <= 0) {
        dat[0] = 1;
        zb_SendDataRequest( source, ID_CMD_WRITE_RES, 1, dat, 0, AF_ACK_REQUEST, 0 );
        return;
      } 
      i += ret;
    }
    dat[0] = 0;
    zb_SendDataRequest( source, ID_CMD_WRITE_RES, 1, dat, 0, AF_ACK_REQUEST, 0 );
    break;
  case ID_CMD_READ_REQ:
    for (i=0; i<len; i+=2) {
      pid = pData[i]<<8 | pData[i+1];
      dat[rlen++] = pData[i];
      dat[rlen++] = pData[i+1];
      ret = paramRead(pid, dat+rlen);
      if (ret <= 0) {
        dat[0] = 1;
        zb_SendDataRequest( source, ID_CMD_READ_RES, 1, dat, 0, AF_ACK_REQUEST, 0 );
        return;
      }
      rlen += ret;
    }
    dat[0] = 0;
    zb_SendDataRequest( source, ID_CMD_READ_RES, rlen, dat, 0, AF_ACK_REQUEST, 0 );
    break;
  }
}


/******************************************************************************
 * @fn          
 *
 * @brief       Reports temperature sensor reading
 *
 * @param
 *
 * @return
 */


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
  case 0x0801:
    rfidOn = dat[0];
    len = 1;
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
    dat[0] = DEV_ID_RFID;
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
  case 0x0801:
    dat[0] = rfidOn;
    len = 1;
    break;
  case 0x0810:  /* 主动读取卡片ID*/
    
    len = 0;
    break;
  }
  return len;
}







#define RFID_UART 0 //1


/*UART0通信初始化
-------------------------------------------------------*/
void rfid_uart_init(void)
{
#if 1
   P0SEL |=  0x0C;                  //初始化UART0端口
   PERCFG&= ~0x01;                  //选择UART0为可选位置一
#else
   PERCFG |= 0x01;
   P1SEL |= 0xc0;
#endif
   U0UCR = 0;                       //设置停止位与奇偶校验 
   U0CSR = 0xC0;                    //设置为UART模式,而且使能接受器
   
   U0GCR = 11;
   U0BAUD = 216;                    //设置UART0波特率为115200bps
  
}
/*UART0发送数据
-------------------------------------------------------*/
void  rfid_uart_send(unsigned char data)
{
  U0DBUF = data;
  while(!UTX0IF);    //等待UART空闲时发送数据
  UTX0IF = 0;
}

int rfid_uart_recv(void)
{
  int data = -1;
  unsigned int i=50000;
  while (!URX0IF && --i) { }//查询是否收到数据，否则继续等待
  if ( i > 0 ) {
    data = 0xff & U0DBUF;          //提取接收到的数据
    URX0IF = 0;
  }
  return data;          
}

void rfid_commands(char *cmd, int len)
{
  int i;
  for (i=0; i<len; i++)
    rfid_uart_send(cmd[i]);
}

char datxor(char *dat, int len)
{
  char x = 'X';
  char i;
  for (i=0; i<len; i++) {
    x ^= dat[i];
  }
  return x;
}

char cmd[16];
char dat[32];
int rfid_id(char *id)
{
  
  char idx = 0;
  cmd[0] = 0x02, cmd[1] = 0x00, cmd[2] = 0x02;
  rfid_commands(cmd, 3);
  
  dat[idx++] = rfid_uart_recv();
  dat[idx++] = rfid_uart_recv();
  dat[idx++] = rfid_uart_recv();
  if (dat[0]!=0x02 || dat[1]!=0x00 || dat[2]!=0x00){
    dat[idx++] = rfid_uart_recv(); // ck
    return 0;
  }
  dat[idx++] = rfid_uart_recv();
  dat[idx++] = rfid_uart_recv();
  dat[idx++] = rfid_uart_recv();
  dat[idx++] = rfid_uart_recv();
  
  dat[idx++] = rfid_uart_recv();

  if (datxor(dat, idx)) {
    return 0;
  }
  if (id != NULL) {
    id[0] = dat[3];
    id[1] = dat[4];
    id[2] = dat[5];
    id[3] = dat[6];
  }
  return 1;
}

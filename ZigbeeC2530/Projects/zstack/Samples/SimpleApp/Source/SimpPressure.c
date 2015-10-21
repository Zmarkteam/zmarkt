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
  PROVIDED �AS IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, 
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
#define APP_INIT                           0    // Initial state(��ʼ״̬)
#define APP_START                          1    // Sensor has joined network(�������Ѽ�������)
#define APP_BOUND                          2    // Sensor is bound to collector(������Ҳ�󶨵�������)

// Application osal event identifiers
// Bit mask of events ( from 0x0000 to 0x00FF )
#define MY_START_EVT                0x0001      //�Զ���Ŀ�ʼ�¼�
#define MY_REPORT_EVT               0x0002     //�Զ�����ϱ��¼�
#define IIC_ADDRESS                 0x1D

#define IIC_SDA                     P0_6 
#define IIC_SCL                     P0_7
/*********************************************************************
 * TYPEDEFS
 */

/*****************************************************mnbn****************
 * LOCAL VARIABLES
 */

static uint8 myAppState = APP_INIT;           //myAppState:�Զ����APP״̬

static uint16 myStartRetryDelay = 10000;      // milliseconds

static uint16 myCheckCardDelay = 2000;         //milliseconds

static uint8 IIC_PRE_ON = 0;                     //RFID�Ŀ����͹ر�       
/*********************************************************************
 * GLOBAL VARIABLES
 */

// Inputs and Outputs for Switch device
#define NUM_IN_CMD_SENSOR                 2       //������ģʽ:����ģʽ
#define NUM_OUT_CMD_SENSOR                3       //������ģʽ:���ģʽ


const cId_t zb_InCmdList[NUM_IN_CMD_SENSOR] =
{
  ID_CMD_READ_REQ,                               //���ж�
  ID_CMD_WRITE_REQ,                              //д�ж�
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
void IIC_WAIT(void);
void IIC_QMAKE(uint16 QMAKE);
void IIC_DELAY(uint16 dlen);
void IIC_ACC_INIT(void);
void IIC_BMP085_INIT(void);
void IIC_START(void);
void IIC_STOP(void);
void IIC_ACK(void);
void IIC_WRITE_BYTE(uint8 wdata);
char IIC_READ_BYTE(void);
void IIC_WRITE(uint8 byte_add,uint8 wdata);
char IIC_READ(uint8 byte_add);
long BMP085_SAMPLE(short OSS);
void rfid_uart_init(void);
void rfid_uart_send(char data);

short AC_1=0,AC_2=0,AC_3=0;
short B_1=0,B_2=0,MB=0,MC=0,MD=0;
unsigned short AC_4=0,AC_5=0,AC_6=0;
long Temp=0,Pre=0;
int temp;
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
        
        IIC_ACC_INIT();
        IIC_BMP085_INIT();
        osal_start_timerEx( sapi_TaskID, MY_REPORT_EVT, myCheckCardDelay );
  }
  
  if ( event & MY_START_EVT )
  {  
    zb_StartRequest();
  }
 // float pre =0;
  unsigned char maobao[10];
  int i,j,k;
  float p;
  if (event & MY_REPORT_EVT) {
      
    if (myAppState == APP_START) {
      HalLedSet( HAL_LED_1, HAL_LED_MODE_ON );
      if (IIC_PRE_ON){
        unsigned char dat[6];
        for(i = 0;i < 10;i++)
        {
         p = BMP085_SAMPLE(3)/1000;
         maobao[i] = (int)p;
         IIC_QMAKE(10);
        }
        for(j = 0;j<10;j++)
        {
          for(k=0;k<10-j;k++)
            if(maobao[k]>maobao[k+1])
            {
              temp = maobao[k];
              maobao[k]=maobao[k+1];
              maobao[k+1]=temp;
            }
        }
        temp = (maobao[1]+maobao[2]+maobao[3]+maobao[4]+maobao[5]+maobao[6]+maobao[7]+maobao[8]+maobao[9])/10;

        dat[0] = 0x0c;
        dat[1] = 0x10;
        dat[2] = ((int)temp/1000);
        dat[3] = ((int)temp%1000/100);
        dat[4] = ((int)temp%1000%100/10);
        dat[5] = ((int)temp%10);
       
      // rfid_uart_send(dat[0]);
      // rfid_uart_send(dat[1]);
      // rfid_uart_send(dat[2]);
      // rfid_uart_send(dat[3]);
      // rfid_uart_send(dat[4]);
      // rfid_uart_send(dat[5]);
        zb_SendDataRequest(0, ID_CMD_REPORT, 6, dat, 0, AF_ACK_REQUEST, 0 );
        
      }
    }
     else 
         HalLedSet( HAL_LED_1, HAL_LED_MODE_OFF );
   
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
  case 0x0c01:
    IIC_PRE_ON = dat[0];
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
  case 0x0005:dat[0] = DEV_ID_PRE;len = 1;break;
    
  /* -----------  ������� ------------------- */  
  case 0x0014: //mac��ַ
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
  case 0x0c01:
    dat[0] = IIC_PRE_ON;
    len = 1;
    break;
  case 0x0c02:  /* ��ȡ���ٶȴ�����Xֵ*/
    dat[0] = temp/1000;
    dat[1] = temp%1000/100;
    dat[2] = temp%1000%100/10;
    dat[3] = temp%10;
    len = 4;
    break;
  case 0x0c10:  /* �����ϱ�X,Y,Z��ֵ*/
    //dat[0] = IIC_READ(0x06);
    //dat[1] = IIC_READ(0x07);
    //dat[2] = IIC_READ(0x08);
    len = 3;
    break;
    
  }
  return len;
}






void IIC_WAIT(void)
{IIC_DELAY(5);}
//#define RFID_UART 0 //1
void IIC_DELAY(uint16 dlen)        //5us��ʱ
{
        uint16 j;
        for(j=0;j<dlen;j--)
        {
          asm("nop");
        }
}

void IIC_QMAKE(uint16 QMAKE)
{
  unsigned int x,y;
  for(x=QMAKE;x>0;x--)
  {for(y=111;y>0;y--);}
}
/*ACC��ʼ��
-------------------------------------------------------*/
void IIC_ACC_INIT(void)
{
   P0DIR |= (1<<7)|(1<<6);        //P0_7P0_6Ϊ���
   rfid_uart_init();
   IIC_SDA = 1;
    IIC_WAIT();
    IIC_SCL = 1;
    IIC_WAIT();
    IIC_QMAKE(200);
}

void IIC_START(void)               //�������ܣ�I2Cͨ�ſ�ʼ
{
    IIC_SDA = 1;
    IIC_DELAY(5);
    IIC_SCL = 1;
    IIC_DELAY(5);
    IIC_SDA = 0;
    IIC_DELAY(5);
	
}

void IIC_STOP(void)                //�������ܣ�I2Cͨ��ֹͣ
{
    IIC_SDA = 0;
    IIC_DELAY(5);
    IIC_SCL = 1;
    IIC_DELAY(5);
    IIC_SDA = 1;
    IIC_DELAY(5);
}

void IIC_ACK(void)                 //�������ܣ�I2Cͨ�Ų�Ӧ��λ
{	
   IIC_SDA = 1;
   IIC_SCL = 1;
   IIC_DELAY(5);
   IIC_SCL = 0;
}

void IIC_WRITE_BYTE(unsigned char wdata)//�������ܣ���I2C�ӻ�д��һ���ֽ�
{
     unsigned char i,temp,temp1;

	 temp1 = wdata;
     for(i=0;i<8;i++)
	{
          IIC_SCL = 0;
          IIC_DELAY(5);
          temp = temp1;
          temp = temp&0x80;
          if(temp == 0x80)
          IIC_SDA = 1;			
          else
          IIC_SDA = 0;
          IIC_DELAY(5);
          IIC_SCL = 1;
          IIC_DELAY(5);
          IIC_SCL = 0;
          IIC_DELAY(5);
          temp1=temp1<<1;	
	}

}
char IIC_READ_BYTE(void)			 //�������ܣ���I2C�ӻ��ж���һ���ֽ�
{
	 char IIC_DATA;
     for(int i=0;i<8;i++)
	{	
          IIC_DATA = IIC_DATA<<1;
          IIC_SDA=1;
          IIC_DELAY(5);
          IIC_SCL = 0;
          IIC_DELAY(5);
          IIC_SCL = 1;
          IIC_DELAY(5);
          if(IIC_SDA == 1)
          IIC_DATA |= 0x01;
	}
	return IIC_DATA;
}
void IIC_WRITE(unsigned char byte_add,unsigned char wdata)//�������ܣ�����ַд��һ�ֽ�����
{
    unsigned char T=0xEE;
     IIC_START();
     IIC_WRITE_BYTE(T);
     IIC_ACK();
     IIC_WRITE_BYTE(byte_add);
     IIC_ACK();
     IIC_WRITE_BYTE(wdata);
     IIC_ACK();
     IIC_STOP();
}
char IIC_READ(unsigned char byte_add)	           //�������ܣ�����ַ����һ�ֽ�����
{
     unsigned char T=0xEE;
     char IIC_DATA;
     IIC_START();
     IIC_WRITE_BYTE(T);
     IIC_ACK();
     IIC_WRITE_BYTE(byte_add);
     IIC_ACK();
     T = 0xEF;
     IIC_START();
     IIC_WRITE_BYTE(T);
     IIC_ACK();
     IIC_DATA = IIC_READ_BYTE();
     IIC_ACK();
     IIC_STOP();
     return IIC_DATA;
}



void IIC_BMP085_INIT(void)
{
        unsigned char memo[22];
	unsigned char i,*PX;
	void *PF;
	//��EEPROM�ж�ȡԤ�����
	for(i=0;i<22;i++){memo[i]=IIC_READ(0xAA+i);}
	PX=memo;
	PF=&AC_1;		
	for(i=0;i<2;i++){*((char *)PF+i)=*(PX+i);}
	PF=&AC_2;		
	for(i=0;i<2;i++){*((char *)PF+i)=*(PX+2+i);}
	PF=&AC_3;		
	for(i=0;i<2;i++){*((char *)PF+i)=*(PX+4+i);}
	PF=&AC_4;		
	for(i=0;i<2;i++){*((char *)PF+i)=*(PX+6+i);}
	PF=&AC_5;		
	for(i=0;i<2;i++){*((char *)PF+i)=*(PX+8+i);}
	PF=&AC_6;		
	for(i=0;i<2;i++){*((char *)PF+i)=*(PX+10+i);}
	PF=&B_1;		
	for(i=0;i<2;i++){*((char *)PF+i)=*(PX+12+i);}
	PF=&B_2;		
	for(i=0;i<2;i++){*((char *)PF+i)=*(PX+14+i);}
	PF=&MB;		
	for(i=0;i<2;i++){*((char *)PF+i)=*(PX+16+i);}
	PF=&MC;		
	for(i=0;i<2;i++){*((char *)PF+i)=*(PX+18+i);}
	PF=&MD;		
	for(i=0;i<2;i++){*((char *)PF+i)=*(PX+20+i);}
}

long BMP085_SAMPLE(short OSS)//�¶ȡ�ѹ������
{
	long UT=0;			//�¶�ԭʼֵ	
	long UP=0;			//ѹ��ԭʼֵ
	long X1,X2,X3;
	long B3,B5,B6;
	unsigned long B4,B7;
	unsigned char i,*PX,*PX1,BMPmemo[2],BMPmemo1[3];
	void *PF,*PF1;

	IIC_WRITE(0xF4,0x2E);	//��������¶�
	IIC_QMAKE(10);							//�ȴ�AD���ӳ�4.5ms����
	for(i=0;i<2;i++){BMPmemo[i]=IIC_READ(0xF6+i);}//��ȡ���
	PX=BMPmemo;	   						
	PF=&UT;								
	for(i=0;i<2;i++){*((char *)PF+2+i)=*(PX+i);}
       // IIC_QMAKE(10);
	IIC_WRITE(0xF4,0x34+(OSS<<6));//�������ѹ��
	IIC_QMAKE(60);							//�ӳ�ʱ���ӹ�����ʽ������������ֲ�
	for(i=0;i<3;i++){BMPmemo1[i]=IIC_READ(0xF6+i);}//��ȡ���
	PX1=BMPmemo1;	   						
	PF1=&UP;								
	for(i=0;i<3;i++){*((char *)PF1+1+i)=*(PX1+i);}
	UP=UP>>(8-OSS);

	X1=(UT-AC_6)*AC_5/32768; 		//�����¶�
	X2=MC;
	X2=X2*2048/(X1+MD);
	B5=X1+X2;
	Temp=(B5+8)/16;

	B6=B5-4000;			//����ѹ��
	X1=B_2;
	X1=(X1*(B6*B6/4096))/2048;
	X2=AC_2;
	X2=X2*B6/2048;
	X3=X1+X2;
	B3=AC_1;
	B3=(((B3*4+X3)<<OSS)+2)/4;	
	X1=AC_3;
	X1=X1*B6/8192;
	X2=B_1;
	X2=(X2*(B6*B6/4096))/65536;
	X3=((X1+X2)+2)/4;
	B4=AC_4;
	B4=B4*(unsigned long)(X3+32768)/32768;
	B7=((unsigned long)UP-B3)*(50000>>OSS);	
	if(B7<0x80000000){Pre=(B7*2)/B4;}
	else {Pre=(B7/B4)*2;}
	X1=(Pre/256)*(Pre/256);
	X1=(X1*3038)/65536;
	X2=(-7357*Pre)/65536;
	Pre=Pre+(X1+X2+3791)/16;
        return Pre;
}
void rfid_uart_init(void)
{
#if 1
   P0SEL |=  0x0C;                  //��ʼ��UART0�˿�
   PERCFG&= ~0x01;                  //ѡ��UART0Ϊ��ѡλ��һ
#else
   PERCFG |= 0x01;
   P1SEL |= 0xc0;
#endif
   U0UCR = 2;                       //����ֹͣλ����żУ�� 
   U0CSR = 0xC0;                    //����ΪUARTģʽ,����ʹ�ܽ�����
   
   U0GCR = 11;
   U0BAUD = 216;                    //����UART0������Ϊ115200bps
  
}

void  rfid_uart_send(char data)
{
  U0DBUF = data;
  while(!UTX0IF);    //�ȴ�UART����ʱ��������
  UTX0IF = 0;
}

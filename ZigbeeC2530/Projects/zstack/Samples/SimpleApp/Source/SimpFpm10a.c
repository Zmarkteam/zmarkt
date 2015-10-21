#include "ZComDef.h"
#include "OSAL.h"
#include "sapi.h"
#include "hal_key.h"
#include "hal_led.h"
#include "hal_adc.h"
#include "hal_mcu.h"
#include "SimpleApp.h"
#include "mt.h"

#define  uchar unsigned char
#define  uint unsigned int
uchar data_uart[18];
const uchar FP_Pack_Head[6] = {0xEF,0x01,0xFF,0xFF,0xFF,0xFF};  //Э���ͷ
const uchar FP_Get_Img[6] = {0x01,0x00,0x03,0x01,0x0,0x05};    //���ָ��ͼ��
const uchar FP_Img_To_Buffer1[7]={0x01,0x0,0x04,0x02,0x01,0x0,0x08}; //��ͼ����뵽BUFFER1
const uchar FP_Img_To_Buffer2[7]={0x01,0x0,0x04,0x02,0x02,0x0,0x09}; //��ͼ����뵽BUFFER2
const uchar FP_Reg_Model[6]={0x01,0x0,0x03,0x05,0x0,0x09}; //��BUFFER1��BUFFER2�ϳ�����ģ��
const uchar FP_Search[11]={0x01,0x0,0x08,0x04,0x01,0x0,0x0,0x03,0xA1,0x0,0xB2}; //����ָ��������Χ0 - 929
const uchar FP_Delet_All_Model[6]={0x01,0x00,0x03,0x0d,0x00,0x11};//ɾ��ָ��ģ�������е�ģ��
volatile unsigned char  FP_Save_Finger[9]={0x01,0x00,0x06,0x06,0x01,0x00,0x0B,0x00,0x19};//��BUFFER1�е��������ŵ�ָ����λ��
volatile unsigned char FP_Delete_Model[10]={0x01,0x00,0x07,0x0C,0x0,0x0,0x0,0x1,0x0,0x0}; //ɾ��ָ����ģ��
// Application States
#define APP_INIT                           0    // Initial state
#define APP_START                          1    // Sensor has joined network
#define APP_BOUND                          2    // Sensor is bound to collector

// Application osal event identifiers
// Bit mask of events ( from 0x0000 to 0x00FF )
#define MY_START_EVT                0x0001
#define MY_REPORT_EVT               0x0002

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
char index_command =0;

static uint8 myAppState = APP_INIT;

static uint16 myStartRetryDelay = 10000;      // milliseconds

static uint16 myCheckCardDelay = 4000;         //milliseconds

static uint8 Fpm10aon = 1;

uint zhiwen_id[2]={0x00,0x00};
uchar z = 0;
/*********************************************************************
 * GLOBAL VARIABLES
 */

// Inputs and Outputs for Switch device
#define NUM_IN_CMD_SENSOR                 2
#define NUM_OUT_CMD_SENSOR                3


const cId_t zb_InCmdList[NUM_IN_CMD_SENSOR] =
{
  ID_CMD_READ_REQ,
  ID_CMD_WRITE_REQ,
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
void zb_HanderMsg(osal_event_hdr_t *msg);
//void zb_HandleKeys( uint8 shift, uint8 keys );

static int paramWrite(uint16 pid, byte *dat);
static int paramRead(uint16 pid, byte *dat);


void rfid_uart_init(void);
void rfid_uart_send(unsigned char ch);
int  rfid_uart_recv(void);
void rfid_commands(char *cmd, int len);
void FINGERPRINT_Cmd_Search_Finger(void);
void FINGERPRINT_Cmd_Save_Finger(uint number);
void FINGERPRINT_Cmd_Reg_Model(void);
void FINGERPRINT_Cmd_Img_To_Buffer2(void);
void FINGERPRINT_Cmd_Img_To_Buffer1(void);
void FINGERPRINT_Cmd_Delete_All_Model(void);
void FINGERPRINT_Cmd_Get_Img(void);
uchar test_fig(void);
void FINGERPRINT_Cmd_Delete_Model(unsigned int uiID_temp);
void delayxms(uint xms);
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
   byte dat_1[32];
   char mod=0;
   if (event & ZB_ENTRY_EVENT) 
   {
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
        if (startOptions != ZCD_STARTOPT_AUTO_START)
        {
          startOptions = ZCD_STARTOPT_AUTO_START;
          zb_WriteConfiguration( ZCD_NV_STARTUP_OPTION, sizeof(uint8), &startOptions );
        } 
        
        rfid_uart_init();//��Ҫ�������ʳ�ʼ��Ϊ56000
        
        osal_start_timerEx( sapi_TaskID, MY_REPORT_EVT, myCheckCardDelay );
  }
  if ( event & MY_START_EVT )
  {  
    zb_StartRequest();
  }
  if (event & MY_REPORT_EVT) 
  {
     if(Fpm10aon==1)//ֵΪ1ʱ˵����λ����ϵͳ���������ָ֤��
     {
       if(z==1)
       {
         delayxms(50000);delayxms(50000);delayxms(50000);delayxms(50000);delayxms(50000);
         delayxms(50000);delayxms(50000);
         z=0;
       }
         if(test_fig()==0&&Fpm10aon==1)  
          {
           FINGERPRINT_Cmd_Get_Img();//��ȡָ��y_scan_lu();
           FINGERPRINT_Cmd_Img_To_Buffer1();//��ָ����Ϣ����BUFFER1
           FINGERPRINT_Cmd_Search_Finger();//����999��ģ����Ϣ
           if(data_uart[9]==0)//ƥ�䵽ָ��
           {
            dat_1[0]=0x09;
            dat_1[1]=0x02;
            dat_1[2]=1;
            dat_1[3]=data_uart[10];
            dat_1[4]=data_uart[11];
            zb_SendDataRequest(0, ID_CMD_REPORT, 5, dat_1, 0, AF_ACK_REQUEST, 0 ); //����λ�����������֤�ɹ�
            HalLedSet( HAL_LED_1, HAL_LED_MODE_OFF );
            delayxms(30000);
           }
           if(data_uart[9]==0x09)//û��ƥ�䵽ָ��
           { 
            dat_1[0]=0x09;
            dat_1[1]=0x02;
            dat_1[2]=0;
            zb_SendDataRequest(0, ID_CMD_REPORT, 3, dat_1, 0, AF_ACK_REQUEST, 0 );//����λ�����������֤ʧ�ܷǷ��û�
           }
          } 

      }       
      else if(index_command==2)//��λ������¼��ָ��ָ��
      {   
          mod = 0;
        // while(test_fig()==2); 
         while(mod==0)
         {
           while((test_fig())!=0)
           {delayxms(100);}
           mod=1;
           FINGERPRINT_Cmd_Get_Img();//��һ�ζ�ȡָ����Ϣ
           if(data_uart[9]!=0)//¼��ʧ��������mod
           mod=0;
           FINGERPRINT_Cmd_Img_To_Buffer1();//����һ�ζ�ȡ��ָ�ƴ�ŵ�������1
           if(data_uart[9]!=0)//¼��ʧ��������mod
           mod=0; 
           FINGERPRINT_Cmd_Get_Img();//�ڶ��ζ�ȡָ����Ϣ
           //delayxms(6000);
           if(data_uart[9]!=0)//¼��ʧ��������mod
           mod=0;
           FINGERPRINT_Cmd_Img_To_Buffer2();//���ڶ��ζ�ȡ��ָ�ƴ�ŵ�������2
           if(data_uart[9]!=0)//¼��ʧ��������mod
           mod=0;
           FINGERPRINT_Cmd_Reg_Model();//����ָ��ģ��
           //delayxms(1000);
           if(data_uart[9]!=0)//¼��ʧ��������mod
           mod=0;
           FINGERPRINT_Cmd_Save_Finger((zhiwen_id[0]<<8)|zhiwen_id[1]/*zhiwen_id[0]+zhiwen_id[1]*/);//��ģ��洢��ĳ��ַ
           if(data_uart[9]!=0)//¼��ʧ��������mod
           mod=0;
           //uart1_send_one_byte(mod);
           if(mod==0)
           {  
              HalLedSet( HAL_LED_1, HAL_LED_MODE_ON );
              dat_1[0]=0x09;
              dat_1[1]=0x03;
              dat_1[2]=0;//�������ݵ�2λΪ0��ʾ¼��ʧ��
              dat_1[3]=zhiwen_id[0];
              dat_1[4]=zhiwen_id[1];
              zb_SendDataRequest( 0, ID_CMD_REPORT, 5, dat_1, 0, AF_ACK_REQUEST, 0 );//����λ������ָ��¼��ʧ��
              HalLedSet( HAL_LED_1, HAL_LED_MODE_OFF);
           }
           if(mod==1)
           { 
	     if(data_uart[9]==0)
	     { 
               HalLedSet( HAL_LED_1, HAL_LED_MODE_ON );
               dat_1[0]=0x09;
               dat_1[1]=0x03;
               dat_1[2]=1;//�������ݵ�2λΪ1��ʾ¼��ɹ�
               dat_1[3]=zhiwen_id[0];
               dat_1[4]=zhiwen_id[1];
               zb_SendDataRequest( 0, ID_CMD_REPORT, 5, dat_1, 0, AF_ACK_REQUEST, 0 );//����λ������ָ��¼��ɹ�
             }
           }    
         }
         index_command = 0;
         Fpm10aon = 1;//�����ָ�ƹ���״̬��
         z=1;
      }
     else if(index_command==3){  //��λ������ɾ������ָ����Ϣ
        FINGERPRINT_Cmd_Delete_Model((zhiwen_id[0]<<8)|zhiwen_id[1]/*zhiwen_id[0]+zhiwen_id[1]*/);
        if(data_uart[9]==0)
        { 
          HalLedSet( HAL_LED_1, HAL_LED_MODE_OFF );
          dat_1[0]=0x09;
          dat_1[1]=0x04;
          dat_1[2]=1;//�������ݵ�2λΪ1��ʾɾ���ɹ�
          dat_1[3]=zhiwen_id[0];
          dat_1[4]=zhiwen_id[1];
          zb_SendDataRequest(0,ID_CMD_REPORT, 5, dat_1, 0, AF_ACK_REQUEST, 0 );//����λ������ָ��ɾ���ɹ��ɹ�
        }
        else{ 
          dat_1[0]=0x09;
          dat_1[1]=0x04;
          dat_1[2]=0;//�������ݵ�2λΪ1��ʾɾ��ʧ��
          dat_1[3]=zhiwen_id[0];
          dat_1[4]=zhiwen_id[1];
          zb_SendDataRequest(0, ID_CMD_REPORT, 5, dat_1, 0, AF_ACK_REQUEST, 0 );//ָ��ɾ��ʧ��
        }
        index_command = 0;
        Fpm10aon = 1;
      }
  }
   osal_start_timerEx( sapi_TaskID, MY_REPORT_EVT, myCheckCardDelay );  
}
void zb_HandleKeys( uint8 shift, uint8 keys )
{
  //index_command=3;
  //Fpm10aon = 0;
  FINGERPRINT_Cmd_Delete_All_Model();
  
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
  HalLedSet( HAL_LED_1, HAL_LED_MODE_BLINK ); //���ӳɹ��ƴ�
  switch (command) 
  {
  case ID_CMD_WRITE_REQ:
    for (i=0; i<len; i+=2)
    {
      pid = pData[i]<<8 | pData[i+1];
      ret = paramWrite(pid, pData+2);
      if (ret <= 0)
      {
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
    for (i=0; i<len; i+=2)
    {
      pid = pData[i]<<8 | pData[i+1];
      dat[rlen++] = pData[i];
      dat[rlen++] = pData[i+1];
      ret = paramRead(pid, dat+rlen);
      if (ret <= 0)
      {
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
  switch (pid)
  {
  case 0x0901:
      Fpm10aon = dat[0];
      len = 1;
      break; 
  case 0x0903:
      index_command =2;
      Fpm10aon = 0;
      zhiwen_id[0] = dat[1];
      zhiwen_id[1] = dat[2];
      len = 3;
      break;
  case 0x0904:
      index_command =3;
      Fpm10aon = 0;
      zhiwen_id[0] = dat[1];
      zhiwen_id[1] = dat[2];
      len = 3;
      break;
  }
  return len;
}

static int paramRead(uint16 pid, byte *dat)
{
  int len = 0;
  switch (pid) {
  case 0x0001:
    dat[0] = 0x20; dat[1] = 0x12;
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
    dat[0] = DEV_ID_PFM10A;
    len = 1;
    break;
    
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
  case 0x0901:
    dat[0] = Fpm10aon;
    len = 1;
    break;
  case 0x0902:  
   // dat[] = 
    len = 1;
    break;
  }
  return len;
}


#define RFID_UART 0 //1


/*UART0ͨ�ų�ʼ��
-------------------------------------------------------*/
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
   
   U0GCR = 10;
   U0BAUD = 216;                    //����UART0������Ϊ56700bps
  
}
/*UART0��������
-------------------------------------------------------*/
void  rfid_uart_send(uchar data)
{
  U0DBUF = data;
  while(!UTX0IF);    //�ȴ�UART����ʱ��������
  UTX0IF = 0;
}

int rfid_uart_recv(void)
{
  int data = -1;
  //int i=30000;
  while (!URX0IF /*&& --i*/) { }
  data = 0xff & U0DBUF;          //��ȡ���յ�������
  URX0IF = 0;
  return data;          
}

//FINGERPRINT_���ָ��ͼ������
void FINGERPRINT_Cmd_Get_Img(void)
{
    uchar i;

   for(i=0;i<6;i++) //���Ͱ�ͷ
       rfid_uart_send(FP_Pack_Head[i]);
    
    for(i=0;i<6;i++) //�������� 0x1d
       rfid_uart_send(FP_Get_Img[i]);

	for(i=0;i<12;i++)//����Ӧ����Ϣ
	  data_uart[i]=rfid_uart_recv();
}

//ɾ������ָ��ģ��
void FINGERPRINT_Cmd_Delete_All_Model(void)
{
    uchar i;

    for(i=0;i<6;i++) //���Ͱ�ͷ
       rfid_uart_send(FP_Pack_Head[i]);
    
    for(i=0;i<6;i++) //�������� 0x1d
       rfid_uart_send(FP_Delet_All_Model[i]);
	   
	for(i=0;i<12;i++)//����Ӧ����Ϣ
		data_uart[i]=rfid_uart_recv();
	
}


//��ͼ��ת��������������Buffer1��
void FINGERPRINT_Cmd_Img_To_Buffer1(void)
{
	unsigned char i;
    
	for(i=0;i<6;i++)    //���Ͱ�ͷ
	{
    	 rfid_uart_send(FP_Pack_Head[i]);   
    }
           
    for(i=0;i<7;i++)   //�������� ��ͼ��ת���� ������ ����� CHAR_buffer1
    {
         rfid_uart_send(FP_Img_To_Buffer1[i]);
    }


	for(i=0;i<12;i++)//��Ӧ����Ϣ
	{
		 data_uart[i]=rfid_uart_recv();//��Ӧ�����ݴ�ŵ�������
	}

}


//��ͼ��ת��������������Buffer2��
void FINGERPRINT_Cmd_Img_To_Buffer2(void)
{
	unsigned char i;
    for(i=0;i<6;i++)    //���Ͱ�ͷ
	{
    	 rfid_uart_send(FP_Pack_Head[i]);   
   	}
           
   	for(i=0;i<7;i++)   //�������� ��ͼ��ת���� ������ ����� CHAR_buffer2
    {
      	rfid_uart_send(FP_Img_To_Buffer2[i]);
   	}

	for(i=0;i<12;i++)
	{
		data_uart[i]=rfid_uart_recv();//����Ӧ����Ϣ
	}
}

//��BUFFER1 �� BUFFER2 �е�������ϲ���ָ��ģ��
void FINGERPRINT_Cmd_Reg_Model(void)
{
    unsigned char i;    

   for(i=0;i<6;i++) //��ͷ
    {
      rfid_uart_send(FP_Pack_Head[i]);   
    }

    for(i=0;i<6;i++) //����ϲ�ָ��ģ��
    {
      rfid_uart_send(FP_Reg_Model[i]);   
    }

	for(i=0;i<12;i++)
	{
		data_uart[i]=rfid_uart_recv();
	}
}
void FINGERPRINT_Cmd_Save_Finger(uint number)//�洢ģ�浽�ض���ַ
{
           uint temp = 0;
		   unsigned char i;

       //    FP_Save_Finger[5] = ucH_Char;
	   		FP_Save_Finger[5] = (number>>8);
       //    FP_Save_Finger[6] = ucL_Char;
	   		FP_Save_Finger[6] = number;
           
		   for(i=0;i<7;i++)   //����У���
		   	   temp = temp + FP_Save_Finger[i];
			    
		//   FP_Save_Finger[7]=(temp & 0x00FF00) >> 8; //���У������
		 //  FP_Save_Finger[8]= temp & 0x0000FF;
		 	FP_Save_Finger[7]=(uchar)(temp>>8);//���У������
			FP_Save_Finger[8]=(uchar)temp;
		   
           for(i=0;i<6;i++)    
    	       rfid_uart_send(FP_Pack_Head[i]);        //���Ͱ�ͷ
           for(i=0;i<9;i++)  
      		 rfid_uart_send(FP_Save_Finger[i]);      //�������� ��ͼ��ת���� ������ ����� CHAR_buffer1 		                                                                                                                                                                                                                                                                                                                    led4=0;

		   for (i=0;i<12;i++)
           		data_uart[i]=rfid_uart_recv(); 
}
//����ȫ���û�999ö
void FINGERPRINT_Cmd_Search_Finger(void)
{
       unsigned char i;	   
	   for(i=0;i<6;i++)   //������������ָ�ƿ�
           {
    	      rfid_uart_send(FP_Pack_Head[i]);   
   		   }

       for(i=0;i<11;i++)
           {
    	      rfid_uart_send(FP_Search[i]);   
   		   }

		for(i=0;i<16;i++)
		{
			data_uart[i]=rfid_uart_recv();
		}
}
uchar test_fig(void)//���ָ��ģ��¼��ָ�����������00��ʾ¼��ɹ���02����ָ��03¼��ʧ��
{
	uchar fig_dat;
	FINGERPRINT_Cmd_Get_Img();
	//delayxms(20);
	fig_dat=data_uart[9];
	return(fig_dat);
}
//ɾ��ָ��ģ�����ָ��ָ��ģ��
void FINGERPRINT_Cmd_Delete_Model(uint uiID_temp)
{
   volatile unsigned int uiSum_temp = 0;
   unsigned char i;    
	 
	FP_Delete_Model[4]=(uiID_temp>>8);
	FP_Delete_Model[5]=(uiID_temp);
	
	for(i=0;i<8;i++)
	    uiSum_temp = uiSum_temp + FP_Delete_Model[i];
	
	FP_Delete_Model[8]=(uchar)(uiSum_temp>>8);
	FP_Delete_Model[9]=(uchar)uiSum_temp;
	 
    for(i=0;i<6;i++) //��ͷ
      rfid_uart_send(FP_Pack_Head[i]);   

    for(i=0;i<10;i++) //����ϲ�ָ��ģ��
      rfid_uart_send(FP_Delete_Model[i]);   
    for(i=0;i<12;i++)
       data_uart[i]=rfid_uart_recv();                
}
void delayxms(uint xms)
{
  while(xms--);
  while(xms--);
  while(xms--);
  while(xms--);
  while(xms--);
  while(xms--);
}

/*************************************************************************************************************
 * �ļ���:	uart.c
 * ����:	CC2530 ������غ���
 * ����:	lee
 * ����ʱ��:	2015-11-03 22:50
 * ����޸�ʱ��:2015-11-03
 * ��ϸ:	������غ���
			�������ʱ��Ϊϵͳʱ�ӵ�1/16
*************************************************************************************************************/
#include "ZComDef.h"
#include "OSAL.h"
#include "sapi.h"
#include "hal_key.h"
#include "hal_led.h"
#include "hal_adc.h"
#include "hal_mcu.h"
#include "SimpleApp.h"


#include "mt.h"

#include "uart.h"

#define DEBUG_ENABLED
void CC2530_DEBUG(uint8 *fmt ,...)
{
#ifdef DEBUG_ENABLED
	va_list	arg_ptr;
	uint8	LocalText[64];
	uint8 	cnt;
	uint8	m;
	for(cnt=0 ; cnt<64 ; cnt++) 
	{
		LocalText[cnt] = 0x00;
	}

	va_start(arg_ptr, fmt);
	vsprintf(LocalText, fmt, arg_ptr);
	va_end(arg_ptr);
	for(m=0 ; m<64 ; m++) 
	{
		if(LocalText[m]==0x00)
		{
			break;
		}
	}
	HalUARTWrite(MT_UART_DEFAULT_PORT,LocalText,m);
#else
;
#endif	
}
/*************************************************************************************************************
 * 文件名:	uart.c
 * 功能:	CC2530 串口相关函数
 * 作者:	lee
 * 创建时间:	2015-11-03 22:50
 * 最后修改时间:2015-11-03
 * 详细:	串口相关函数
			串口最大时钟为系统时钟的1/16
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
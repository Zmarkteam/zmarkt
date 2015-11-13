/*************************************************************************************************************
 * �ļ���:	uart.h
 * ����:	CC2530 ������غ���
 * ����:	cp1300@139.com
* ����ʱ��:	2013-06-07 21:33
 * ����޸�ʱ��:2013-06-07
 * ��ϸ:	������غ���
*************************************************************************************************************/
#ifndef _UART_H_
#define _UART_H_

#include "OSAL.h"
#include "hal_mcu.h"
#include "stdio.h"
#include "hal_uart.h"
#include "MT_UART.h"

//UAR
void CC2530_DEBUG_DMA(uint8 *fmt ,...); 
void CC2530_DEBUG_ISR(uint8 *fmt ,...);

#define PRINTK(X,...)     CC2530_DEBUG(X,##__VA_ARGS__)

#endif //_UART_H_
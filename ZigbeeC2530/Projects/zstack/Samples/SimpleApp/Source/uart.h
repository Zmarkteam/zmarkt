/*************************************************************************************************************
 * 文件名:	uart.h
 * 功能:	CC2530 串口相关函数
 * 作者:	cp1300@139.com
* 创建时间:	2013-06-07 21:33
 * 最后修改时间:2013-06-07
 * 详细:	串口相关函数
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
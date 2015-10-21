
#include <ioCC2530.h>
#include <stdio.h>
#include "dht11.h"

#define COM_DHT11_CLR   (P1_2=0)
#define COM_DHT11_SEL   (P1_2=1)

#define COM_DHT11_IN    (P1DIR &= ~0x04)
#define COM_DHT11_OUT   (P1DIR |= 0x04)

#define COM_DHT11_PIN   (P1_2)
#pragma optimize=none
void halMcuWaitUs(unsigned int usec)
{
    usec>>= 1;
    while(usec--)
    {
        asm("NOP");
        asm("NOP");
        asm("NOP");
        asm("NOP");
        asm("NOP");
        asm("NOP");
        asm("NOP");
        asm("NOP");
        asm("NOP");
        asm("NOP");
        asm("NOP");
        asm("NOP");
        asm("NOP");
        asm("NOP");
        asm("NOP");
        asm("NOP");
        asm("NOP");
    }
}

char check_dht11_has(void){
        COM_DHT11_OUT;
        COM_DHT11_CLR;
        halMcuWaitUs(18000);
        COM_DHT11_SEL;
        halMcuWaitUs(40);
        COM_DHT11_IN;
        if(!COM_DHT11_PIN){
                return 1;
        } else{
                return 0;
        }
}

unsigned char read_DHT11_COM(void){
	unsigned char i,data,flag,temp;
	for(i=0;i<8;i++){
		flag = 2;
		COM_DHT11_IN;
		while((!COM_DHT11_PIN) && flag++);
		halMcuWaitUs(30);
		temp = 0;
		if(COM_DHT11_PIN) temp=1;
		flag = 2;
		while(COM_DHT11_PIN && flag++);
		if(flag == 1) break;
		data<<=1;
		data|=temp;
	}
	return data;
}

unsigned char get_DHT11_DATA(char select){
	unsigned char temp_H,temp_L,rh_H,rh_L,ch_data,flag;
	COM_DHT11_OUT;
	COM_DHT11_CLR;
	halMcuWaitUs(18000);
	COM_DHT11_SEL;
	halMcuWaitUs(40);
	COM_DHT11_IN;
	if(!COM_DHT11_PIN){
		flag = 2;
		while((!COM_DHT11_PIN) && flag++);
		flag = 2;
		while(COM_DHT11_PIN && flag++);
		rh_H = read_DHT11_COM();
		rh_L = read_DHT11_COM();
		temp_H= read_DHT11_COM();
		temp_L = read_DHT11_COM();
		ch_data = read_DHT11_COM();
		halMcuWaitUs(1);
		COM_DHT11_OUT;
		COM_DHT11_SEL;
	}
	flag = temp_H + temp_L + rh_H + rh_L;
	if (flag == ch_data){
		if(select == 0){
			return temp_H;
		} else{
			return rh_H;
		}
	}
}
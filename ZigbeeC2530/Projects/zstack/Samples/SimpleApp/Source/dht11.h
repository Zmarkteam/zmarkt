#ifndef __DHT11_H__
#define __DHT11_H__
void halMcuWaitUs(unsigned int usec);
char check_dht11_has(void);
unsigned char read_DHT11_COM(void);
unsigned char get_DHT11_DATA(char select);
#endif

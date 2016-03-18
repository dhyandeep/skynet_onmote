#include "contiki.h"
#include "/home/dhyandeepak/Desktop/contiki-2.7/cpu/msp430/dev/uart1.h"
#include <stdio.h>
#define SERIAL_BUF_SIZE 128
int currentnum=0,val=0;
static int uart_rx_callback(unsigned char c) {
     uint8_t u;
     printf("\nReceived %c",c);
     u = (uint8_t)c;
     //printf("\nReceived %u",u);
     if(c!=' ')
     {
		 val=c-'0';
		 currentnum=currentnum*10+val;
	 }
	 else
	 {
		 printf("\nvalue:%d",currentnum);
		 val=0;
		 currentnum=0;
	 }
     return u;
}
PROCESS(moteread_process, "moteread process");
static char rx_buf[SERIAL_BUF_SIZE];
/*---------------------------------------------------------------------------*/

AUTOSTART_PROCESSES(&moteread_process);

/*---------------------------------------------------------------------------*/


PROCESS_THREAD(moteread_process, ev, data)
{
	
	PROCESS_BEGIN();
	uart1_init(BAUD2UBR(115200)); //set the baud rate as necessary
	uart1_set_input(uart_rx_callback); //set the callback function
	PROCESS_END();
}
/*---------------------------------------------------------------------------*/


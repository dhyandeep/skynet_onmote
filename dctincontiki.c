/*
 * Copyright (c) 2006, Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 *
 */

/**
 * \file
 *         A very simple Contiki application showing how Contiki programs look
 * \author
 *         Adam Dunkels <adam@sics.se>
 */

#include "contiki.h"
#include <stdio.h> /* For printf() */
//#include <cfs/cfs.h>
#include <math.h>
#include "/home/dhyandeepak/Desktop/contiki-2.7/cpu/msp430/dev/uart1.h"

#define N 8
#define LENGTH 8
#define CONTIKI_NO_NET=1
#define UART_DEFAULT_TXBUFSIZE 1

float mat1[N*N];
float mat2[N*N];



int currentnum=0,val=0,counter=0;
unsigned char c;
static int uart_rx_callback(unsigned char c) //read from serial port
{
     uint8_t u;
     //printf("\nReceived %c",c);
     u = (uint8_t)c;
     //printf("\nReceived %u",u);
     if(c!=' ')
     {
		 val=c-'0';
		 currentnum=currentnum*10+val;
	 }
	 else 
	 {
		// printf("\nvalue:%d",currentnum);
		 if(counter<64)
		 mat1[counter]=currentnum;
		 else
		 mat2[counter%64]=currentnum;
		 val=0;
		 currentnum=0;
		 counter++;
		 printf("\nno:%d ",counter);
		 
	// if(counter==16)
	 
	 }
	 if(counter==128)
	 {
		 int i;
		 for(i=0;i<N*N;i++)
		 {
			
			if(i%N==0)
			printf("\n");
			printf("%d ",(int)mat1[i] );
			
		
		}
		for(i=0;i<N*N;i++)
		{
			
			if(i%N==0)
			printf("\n");
			printf("%d ",(int)mat2[i] );
			
		
		}
		
		performdct();
		uart1_writeb('<');
		counter=0;					//calculate dct and take inverse
	 }
	 
     return u;
}

static inline void fdct_1d(float *dst, const float *src,
                           int dst_stridea, int dst_strideb,
                           int src_stridea, int src_strideb)
{
    int i;

    for (i = 0; i < N; i++) {
        const float x00 = src[0*src_stridea] + src[7*src_stridea];
        const float x01 = src[1*src_stridea] + src[6*src_stridea];
        const float x02 = src[2*src_stridea] + src[5*src_stridea];
        const float x03 = src[3*src_stridea] + src[4*src_stridea];
        const float x04 = src[0*src_stridea] - src[7*src_stridea];
        const float x05 = src[1*src_stridea] - src[6*src_stridea];
        const float x06 = src[2*src_stridea] - src[5*src_stridea];
        const float x07 = src[3*src_stridea] - src[4*src_stridea];
        const float x08 = x00 + x03;
        const float x09 = x01 + x02;
        const float x0a = x00 - x03;
        const float x0b = x01 - x02;
        const float x0c = 1.38703984532215*x04 + 0.275899379282943*x07;
        const float x0d = 1.17587560241936*x05 + 0.785694958387102*x06;
        const float x0e = -0.785694958387102*x05 + 1.17587560241936*x06;
        const float x0f = 0.275899379282943*x04 - 1.38703984532215*x07;
        const float x10 = 0.353553390593274 * (x0c - x0d);
        const float x11 = 0.353553390593274 * (x0e - x0f);
        dst[0*dst_stridea] = 0.353553390593274 * (x08 + x09);
        dst[1*dst_stridea] = 0.353553390593274 * (x0c + x0d);
        dst[2*dst_stridea] = 0.461939766255643*x0a + 0.191341716182545*x0b;
        dst[3*dst_stridea] = 0.707106781186547 * (x10 - x11);
        dst[4*dst_stridea] = 0.353553390593274 * (x08 - x09);
        dst[5*dst_stridea] = 0.707106781186547 * (x10 + x11);
        dst[6*dst_stridea] = 0.191341716182545*x0a - 0.461939766255643*x0b;
        dst[7*dst_stridea] = 0.353553390593274 * (x0e + x0f);
        dst += dst_strideb;
        src += src_strideb;
    }
}

static void fdct(float *dst, const float *src)
{
    float tmp[N*N];
    fdct_1d(tmp, src, 1, N, 1, N);
    fdct_1d(dst, tmp, N, 1, N, 1);
}

static inline void idct_1d(float *dst, const float *src,
                           int dst_stridea, int dst_strideb,
                           int src_stridea, int src_strideb)
{
    int i;

    for (i = 0; i < N; i++) {
        const float x00 = 1.4142135623731*src[0*src_stridea];
        const float x01 = 1.38703984532215*src[1*src_stridea] + 0.275899379282943*src[7*src_stridea];
        const float x02 = 1.30656296487638*src[2*src_stridea] + 0.541196100146197*src[6*src_stridea];
        const float x03 = 1.17587560241936*src[3*src_stridea] + 0.785694958387102*src[5*src_stridea];
        const float x04 = 1.4142135623731*src[4*src_stridea];
        const float x05 = -0.785694958387102*src[3*src_stridea] + 1.17587560241936*src[5*src_stridea];
        const float x06 = 0.541196100146197*src[2*src_stridea] - 1.30656296487638*src[6*src_stridea];
        const float x07 = -0.275899379282943*src[1*src_stridea] + 1.38703984532215*src[7*src_stridea];
        const float x09 = x00 + x04;
        const float x0a = x01 + x03;
        const float x0b = 1.4142135623731*x02;
        const float x0c = x00 - x04;
        const float x0d = x01 - x03;
        const float x0e = 0.353553390593274 * (x09 - x0b);
        const float x0f = 0.353553390593274 * (x0c + x0d);
        const float x10 = 0.353553390593274 * (x0c - x0d);
        const float x11 = 1.4142135623731*x06;
        const float x12 = x05 + x07;
        const float x13 = x05 - x07;
        const float x14 = 0.353553390593274 * (x11 + x12);
        const float x15 = 0.353553390593274 * (x11 - x12);
        const float x16 = 0.5*x13;
        const float x08 = -x15;
        dst[0*dst_stridea] = 0.25 * (x09 + x0b) + 0.353553390593274*x0a;
        dst[1*dst_stridea] = 0.707106781186547 * (x0f - x08);
        dst[2*dst_stridea] = 0.707106781186547 * (x0f + x08);
        dst[3*dst_stridea] = 0.707106781186547 * (x0e + x16);
        dst[4*dst_stridea] = 0.707106781186547 * (x0e - x16);
        dst[5*dst_stridea] = 0.707106781186547 * (x10 - x14);
        dst[6*dst_stridea] = 0.707106781186547 * (x10 + x14);
        dst[7*dst_stridea] = 0.25 * (x09 + x0b) - 0.353553390593274*x0a;
        dst += dst_strideb;
        src += src_strideb;
    }
}

static void idct(float *dst, const float *src)
{
    float tmp[N*N];
    idct_1d(tmp, src, 1, N, 1, N);
    idct_1d(dst, tmp, N, 1, N, 1);
}




void  performdct()
{
	float fourier1[N*N],fourier2[N*N],inverse[N*N];
	
	//init_dct();
	fdct(fourier1,mat1);
	fdct(fourier2,mat2);
	int i=0;
	for(i=0;i<64;i++)
	{
		fourier1[i]=(fourier1[i]+fourier2[i])/2.0;
	}
	idct(inverse,fourier1);
	printf("\n");
	
	for(i=0;i<64;i++)
	{
		if((i%8==0))
		printf("\n");
		printf("%d ",(int)inverse[i]);
		
	
	}
}
/*---------------------------------------------------------------------------*/
PROCESS(hello_world_process, "dct process");
AUTOSTART_PROCESSES(&hello_world_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(hello_world_process, ev, data)
{
  PROCESS_BEGIN();
  
	uart1_init(BAUD2UBR(115200)); //set the baud rate as necessary
	uart1_set_input(uart_rx_callback); //set the callback function
	
	//printf("dct\n");
    
    //printf(">");
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/


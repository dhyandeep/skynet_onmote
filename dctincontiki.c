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
#include <cfs/cfs.h>
#include <math.h>

#define N 8
#define LENGTH 8
typedef unsigned char u8_t; 
char* INFILE1 ="/home/dhyandeepak/Desktop/outa.bmp";
char* INFILE2 ="/home/dhyandeepak/Desktop/outb.bmp";
char* OUTFILE ="/home/dhyandeepak/Desktop/out.pgm";

u8_t mat[N][N];
unsigned char info1[56];


static float dct_matrix    [N*N];
static float dct_trp_matrix[N*N];

void init_dct(void)
{
    int i, j;

    // dct matrix
    for (i = 0; i < N; i++)
        for (j = 0; j < N; j++)
            if (i == 0)
                dct_matrix[i*N + j] = 1 / sqrt(N);
            else
                dct_matrix[i*N + j] = sqrt(2./N) * cos(((2*j+1)*i*M_PI) / (2*N));

    // dct matrix transposed
    for (i = 0; i < N; i++)
        for (j = 0; j < N; j++)
            dct_trp_matrix[i*N + j] = dct_matrix[j*N + i];
}

static void dct_1d_ref(float *dst, const float *src,
                       int stridea, int strideb,
                       const float *matrix)
{
    int x;
    for (x = 0; x < N; x++) {
        int i, j;
        for (j = 0; j < N; j++) {
            float sum = 0.;
            for (i = 0; i < N; i++)
                sum += matrix[j*N + i] * src[i*stridea];
            dst[j*stridea] = sum;
        }
        dst += strideb;
        src += strideb;
    }
}

static void fdct_ref(float *dst, const float *src)
{
    float tmp[N*N];
    dct_1d_ref(tmp, src, 1, N, dct_matrix);
    dct_1d_ref(dst, tmp, N, 1, dct_matrix);
  
}

static void idct_ref(float *dst, const float *src)
{
    float tmp[N*N];
    dct_1d_ref(tmp, src, 1, N, dct_trp_matrix);
    dct_1d_ref(dst, tmp, N, 1, dct_trp_matrix);
}
char rev[4];
void intochar(int num)
{
	char str[4];
	
	int i,k,dig;
	for(i=0;num>0;i++)
	{
		dig=num%10;
		num=num/10;
		
		str[i]='0'+dig;
		
	}
	k=0;
	for(i=i-1;i>=0;i--)
	{
		rev[k]=str[i];
		k++;
	}
   rev[k]='\0';
   //printf("%s",rev);`
   
}
/*---------------------------------------------------------------------------*/
PROCESS(hello_world_process, "dct process");
AUTOSTART_PROCESSES(&hello_world_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(hello_world_process, ev, data)
{
  PROCESS_BEGIN();
  printf("dct\n");
  int i,j;
  int arr[N*N];
   float result[N*N];
   printf("<");
	for(i=0;i<N*N;i++)
	{
			arr[i]=i%16;
			if(i%N==0)
			printf("\n");
			printf("%d ",arr[i] );
			
		
	}
    fdct_ref(result,arr);
      printf(">");
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/


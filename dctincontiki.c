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
#include <stdlib.h>
#include <string.h>
#include <stdio.h> /* For printf() */
#include <cfs/cfs.h>
#include <cfs-coffee.h>
#include <math.h>

#define N 8
#define LENGTH 128
char* INFILE1 ="/home/dhyandeepak/Desktop/input1.bmp";
char* INFILE2 ="/home/dhyandeepak/Desktop/input2.bmp";
char* OUTFILE ="/home/dhyandeepak/Desktop/out.pgm";

int mat[128][128];
unsigned char info1[56];
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


int* filehand(char* filename) {
 
    
  
	int fd_read = cfs_open(filename, CFS_READ);
	int i=0,j=0;
	unsigned char info1[56],data[16384],rmng[2000];
	cfs_read(fd_read,info1, 56); 										// read the 54-byte headr
	int width1 = *(int*)&info1[18];
	int height1 = *(int*)&info1[22];
	int offset=*(int*)&info1[10];
	int size = width1 * height1;
	

	cfs_read(fd_read,rmng, offset-56);
	cfs_read(fd_read,data, size);
	
	
	printf("Reading from file:\n");
	for(i=0;i<16384;i++)
	{
		if(((i!=0)&&(i%128))==0)
		{
			j++;
			printf("\n");
			
		}
		mat[j][i%128]=data[i];
		printf("%d ",mat[j][i%128]);
	}

	cfs_close(fd_read);
	
    return *mat;   
}
char rev[4];
char* intochar(int num)
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
   //printf("%s",rev);
   return rev;
}
void filewrite(char *OUT)
{
	int i,j,k,pix;
	char ch1[4];
	char ch2[4];
	int fd_write = cfs_open(OUT, CFS_WRITE);
	char* header="P2\n128 128\n255\n";
	char* space=" ";
	char* nline="\n";
	
	
	cfs_write(fd_write,header,15);
	printf("\n\n\nwriting\n");
	for(i=0;i<128;i++)
	{
		for(j=0;j<128;j=j+1)
		{
			*ch1=intochar(mat[i][j]);
			cfs_write(fd_write,rev,4);
			//printf("%s ",rev);
			cfs_write(fd_write,space,1);
			
		}
			cfs_write(fd_write,nline,1);
			//printf("\n");
	}
	
	cfs_close(fd_write);
}


/*---------------------------------------------------------------------------*/
PROCESS(hello_world_process, "dct process");
AUTOSTART_PROCESSES(&hello_world_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(hello_world_process, ev, data)
{
  PROCESS_BEGIN();
  printf("dct\n");
   int i,j,k,l,p,in1[128][128],in2[128][128];
    int* matrix1,*matrix2;
    float src1[N*N],src2[N*N];
    float out_fdct1[N*N],out_fdct2[N*N], out_idct[N*N];
    
    
	matrix1=filehand(INFILE1);
	for(i=0;i<LENGTH;i++)
			for(j=0;j<LENGTH;j++)
				in1[i][j]=mat[i][j];//store the returned matrix in in1

	matrix2=filehand(INFILE2);
	for(i=0;i<LENGTH;i++)
			for(j=0;j<LENGTH;j++)					
				in2[i][j]=(mat[i][j]);//store the returned matrix in in2
	
	
	init_dct();                        //dct computation starts
	for(j=0;j<LENGTH/N;j++)
	{
	   for(i=0;i<LENGTH/N;i++)
	   {
			for(k=0;k<N;k++)
			{
				for(l=0;l<N;l++)
				{
					src1[l+N*k]=in1[(N*j+k)][(l+N*i)];//store 8X8 image block into an array
					src2[l+N*k]=in2[(N*j+k)][(l+N*i)];
					printf("src:%f ",src1[l+N*k]);
				}
				printf("\n");
			}
			printf("\n");
			
			fdct_ref(out_fdct1, src1);//find fast dct for each 8X8 blocks
			fdct_ref(out_fdct2, src2);
			int med;
			for(med=0;med<64;med++)
			{
				out_fdct1[med]=(out_fdct1[med]+out_fdct2[med])/2;//picking the best pixel from 2 inputs
			}
			
			idct_ref(out_idct, out_fdct1);
			for(k=0;k<N;k++)
			{
				for(l=0;l<N;l++)
				{		
					mat[(N*j+k)][(l+N*i)]=out_idct[l+N*k];//store 8X8  block's compressed values into the matrix itself
				}
			}
	   }
	   
	}
    filewrite(OUTFILE);//write the matrix into file
    
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/

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
#define LENGTH 64
#define PROCESS_CONF_NO_PROCESS_NAMES 1
typedef unsigned char u8_t; 
char* INFILE1 ="/home/dhyandeepak/Desktop/outa.bmp";
char* INFILE2 ="/home/dhyandeepak/Desktop/outb.bmp";
char* OUTFILE ="/home/dhyandeepak/Desktop/out.pgm";

u8_t mat[LENGTH][LENGTH];
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
void filehand(char* filename) {
 
    
  
	u8_t fd_read = cfs_open(filename, CFS_READ);
	int i=0,j=0;
	unsigned char info1[56],data[LENGTH*LENGTH],rmng[2000];
	cfs_read(fd_read,info1, 56); 										// read the 54-byte headr
	int width1 = *(int*)&info1[18];
	int height1 = *(int*)&info1[22];
	int offset=*(int*)&info1[10];
	int size = width1 * height1;
	

	cfs_read(fd_read,rmng, offset-56);
	cfs_read(fd_read,data, size);
	
	
	printf("Reading from file:\n");
	for(i=0;i<LENGTH*LENGTH;i++)
	{
		if(((i!=0)&&(i%LENGTH))==0)
		{
			j++;
			printf("\n");
			
		}
		mat[j][i%LENGTH]=data[i];
		printf("%d ",mat[j][i%LENGTH]);
	}

	cfs_close(fd_read);
	
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
   //printf("%s",rev);
   
}
void filewrite(char* out)
{
	int i,j,k,pix;
	int fd_write = cfs_open(out, CFS_WRITE);
	char* header="P2\n64 64\n255\n";	//change image size of the image here also
	char* space=" ";
	char* nline="\n";
	
	
	cfs_write(fd_write,header,15);
	printf("\n\n\nwriting\n");
	for(i=0;i<LENGTH;i++)
	{
		for(j=0;j<LENGTH;j=j+1)
		{
			intochar(mat[i][j]);
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
   int i,j,k,l,p,in1[LENGTH][LENGTH],in2[LENGTH][LENGTH];
    
    float src1[N*N],src2[N*N];
    float out_fdct1[N*N],out_fdct2[N*N], out_idct[N*N];
    
    
	filehand(INFILE1);
	for(i=0;i<LENGTH;i++)
			for(j=0;j<LENGTH;j++)
				in1[i][j]=mat[i][j];//store the returned matrix in in1

	filehand(INFILE2);
	for(i=0;i<LENGTH;i++)
			for(j=0;j<LENGTH;j++)					
				in2[i][j]=(mat[i][j]);//store the returned matrix in in2
	
	

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
					//printf("src:%f ",src1[l+N*k]);
				}
				printf("\n");
			}
			printf("\n");
			init_dct();
			fdct_ref(out_fdct1, src1);//find fast dct for each 8X8 blocks
			fdct_ref(out_fdct2, src2);
			int med;
			for(med=0;med<N*N;med++)
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

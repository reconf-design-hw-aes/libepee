/*************************************
 *
 * speed_test_double_buf.c
 *
 * This file is part of EPEE project (version 2.0)
 * http: *cecaraw.pku.edu.cn
 * 
 * Description
 * Speed test for zero-copy with double buffer.
 * For speed test, the demo hardware should be changed. The DMA loopback in usr_dma.v 
 * should be changed to get all data out of DMA host2board and always input data into
 * DMA board2host FIFO. And this operation should operate in a clock domain whose frequency
 * is not less than that of EPEE libarry.
 * For EPEE, The X4Gen2 with 64 bit interface runs in 250MHz.
 *
 * Author(s):
 *   - Jian Gong, jian.gong@pku.edu.cn
 *
 * History:
 *
 * ------------------------------------
 *
 * Copyright (c) 2013, Center for Energy-Efficient Computing and Applications,
 * Peking University. All rights reserved.
 *
 * The FreeBSD license
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials
 *    provided with the distribution.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE EPEE PROJECT ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * EPEE PROJECT OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * The views and conclusions contained in the software and documentation
 * are those of the authors and should not be interpreted as representing
 * official policies, either expressed or implied, of the EPEE Project.
 *
 *************************************/

#include <stdio.h>
#include <malloc.h>
#include "sPcie.h"
#include "sPcieZerocopy.h"

/*This should be 8 in X8Gen2/X8Gen1 mode, can be 4 in other modes*/
#define MIN_DMA_PAYLOAD 8
#define MAX_DMA_PAYLOAD 1024*1024
#define USER_BUF_SIZE (MAX_DMA_PAYLOAD * 2 * 100)

char * zero_copy_buf;
char * sourceBuf;
char * targetBuf;

int j; // for speed test, DMA payload this time

void * dma_h2b(void *p)
{
	struct timeval pc2b_start, pc2b_stop;
	long pc2b_starts, pc2b_startus, pc2b_stops, pc2b_stopus;
	struct timeval b2pc_start, b2pc_stop;
	long b2pc_starts, b2pc_startus, b2pc_stops, b2pc_stopus;

	int k;
	gettimeofday(&pc2b_start, NULL);
	for(k = 0; k < USER_BUF_SIZE ; k = k + 2 * j){
		memcpy(&zero_copy_buf[0], &sourceBuf[k], j);
		while(0 > zerocopy_host2board(0, j));
		memcpy(&zero_copy_buf[MAX_DMA_PAYLOAD/4], &sourceBuf[k + j], j);
		while(0 > zerocopy_host2board(MAX_DMA_PAYLOAD/4, j));
	}
	gettimeofday(&b2pc_stop, NULL);
	// calculate speed
	pc2b_starts = pc2b_start.tv_sec;
	pc2b_startus = pc2b_start.tv_usec;
	b2pc_stops = b2pc_stop.tv_sec;
	b2pc_stopus = b2pc_stop.tv_usec;
	//printf("%d\t", j);
	printf("h2b: %d\t%.2lf\n", j, (USER_BUF_SIZE*8/(((double)b2pc_stops - (double)pc2b_starts) + ((double)b2pc_stopus - (double)pc2b_startus)/1000000))/1024/1024);
	return NULL;
}

void * dma_b2h(void *p)
{
	struct timeval pc2b_start, pc2b_stop;
	long pc2b_starts, pc2b_startus, pc2b_stops, pc2b_stopus;
	struct timeval b2pc_start, b2pc_stop;
	long b2pc_starts, b2pc_startus, b2pc_stops, b2pc_stopus;
	int k;
	gettimeofday(&pc2b_start, NULL);
	for(k = 0; k < USER_BUF_SIZE ; k = k + 2 * j){
		while(0 > zerocopy_board2host(2*MAX_DMA_PAYLOAD/4, j));
		memcpy(&targetBuf[k], &zero_copy_buf[2*MAX_DMA_PAYLOAD/4], j);
		while(0 > zerocopy_board2host(3*MAX_DMA_PAYLOAD/4, j));
		memcpy(&targetBuf[k + j], &zero_copy_buf[3*MAX_DMA_PAYLOAD/4], j);
	}
	gettimeofday(&b2pc_stop, NULL);
	// calculate speed
	pc2b_starts = pc2b_start.tv_sec;
	pc2b_startus = pc2b_start.tv_usec;
	b2pc_stops = b2pc_stop.tv_sec;
	b2pc_stopus = b2pc_stop.tv_usec;
	//printf("%d\t", j);
	printf("b2h: %d\t%.2lf\n", j, (USER_BUF_SIZE*8/(((double)b2pc_stops - (double)pc2b_starts) + ((double)b2pc_stopus - (double)pc2b_startus)/1000000))/1024/1024);
	return NULL;
}

int main()
{
	int i, k;

	pthread_t dma_read_thread;
	pthread_t dma_write_thread;

	char ip_mode;
	
	zero_copy_buf = (char *)get_zerocopy_buffer();
	sourceBuf = (char *)malloc(USER_BUF_SIZE * sizeof(char));
	targetBuf = (char *)malloc(USER_BUF_SIZE * sizeof(char));
	
	ip_mode = get_pcie_cfg_mode();
	printf(">configure mode:x%dgen%d\t", (ip_mode>>4)&0x0F, (ip_mode&0x0F));
	ip_mode = get_pcie_cur_mode();
	printf(" current mode:x%dgen%d\n", (ip_mode>>4)&0x0F, (ip_mode&0x0F));

	printf("==============PCIe Software Speed Test=============\n");
	printf(">test PCIe system , speed software can detect.\n");
	printf("=====================================================\n");
	printf("payload\tthroughput\n");
	printf("(Byte)\t(Mbps)\n");

	for(i = 0; i < USER_BUF_SIZE; i++){
		sourceBuf[i] = i;
	}

	////////////////
	// warm up
	////////////////
	for(j = 8; j <= 4*1024; j*=2){
		usr_reset();
		for(k = 0; k < 100 ; k++){
			// DMA
			while(0 > zerocopy_host2board(0, j));
			// check DMA done
			while(0 != get_host2board_status());
			// DMA
			while(0 > zerocopy_board2host(MAX_DMA_PAYLOAD/2, j));
			// check DMA done
			while(0 != get_board2host_status());
		}
	}

	////////////////
	// real test
	////////////////
	puts("DMA host2board");
	for(j = MIN_DMA_PAYLOAD; j <= MAX_DMA_PAYLOAD; j*=2){
		usr_reset();

		pthread_create(&dma_read_thread, NULL, dma_h2b, NULL);

		pthread_join(dma_read_thread, NULL);
	}
	puts("DMA board2host");
	for(j = MIN_DMA_PAYLOAD; j <= MAX_DMA_PAYLOAD; j*=2){
		usr_reset();

		pthread_create(&dma_write_thread, NULL, dma_b2h, NULL);

		pthread_join(dma_write_thread, NULL);
	}
	puts("DMA full duplex");
	for(j = MIN_DMA_PAYLOAD; j <= MAX_DMA_PAYLOAD; j*=2){
		usr_reset();

		pthread_create(&dma_read_thread, NULL, dma_h2b, NULL);
		pthread_create(&dma_write_thread, NULL, dma_b2h, NULL);

		pthread_join(dma_read_thread, NULL);
		pthread_join(dma_write_thread, NULL);
	}
	
	printf("=====================================================\n");
	free(sourceBuf);
	free(targetBuf);
	release_zerocopy_buffer(zero_copy_buf);
	return 0;
}


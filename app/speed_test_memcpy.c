/*************************************
 *
 * speed_test_memcpy.c
 *
 * This file is part of EPEE project (version 2.0)
 * http: *cecaraw.pku.edu.cn
 * 
 * Description
 * Speed test using memory copy.
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
#include <sys/time.h>
#include "sPcie.h"

#define _REPEAT_ 1
/*This should be 8 in X8Gen2/X8Gen1 mode, can be 4 in other modes*/
#define DMA_MIN_SIZE 8

char gReadData[1024*4096];
char gWriteData[1024*4096];

int main()
{
	int i, j, k;
	int error_flag;

	struct timeval pc2b_start, pc2b_stop;
	long pc2b_starts, pc2b_startus, pc2b_stops, pc2b_stopus;
	struct timeval b2pc_start, b2pc_stop;
	long b2pc_starts, b2pc_startus, b2pc_stops, b2pc_stopus;

	char ip_mode;
	ip_mode = get_pcie_cfg_mode();
	printf(">configure mode:x%dgen%d\t", (ip_mode>>4)&0x0F, (ip_mode&0x0F));
	ip_mode = get_pcie_cur_mode();
	printf(" current mode:x%dgen%d\n", (ip_mode>>4)&0x0F, (ip_mode&0x0F));

	printf("==============PCIe Software Speed Test=============\n");
	printf(">test PCIe system , speed software can detect.\n");
	printf("=====================================================\n");
	printf("payload\tw_speed\tr_speed\tw_time\tr_time\n");
	printf("(Byte)\t(Mbps)\t(Mbps)\t(ns)\t(ns)\n");

	for(i = 0; i < 1024*4096; i++){
		gWriteData[i] = i;
	}
	for(j = DMA_MIN_SIZE; j <= 4*1024; j*=2){
		usr_reset();
		gettimeofday(&pc2b_start, NULL);
		for(k = 0; k < _REPEAT_ ; k++){
			if(-1 == dma_host2board(j, &gWriteData[0])){
				printf("dma_host2board ERROR\n");
				return -1;
			}
		}
		gettimeofday(&pc2b_stop, NULL);
		gettimeofday(&b2pc_start, NULL);
		for(k = 0; k < _REPEAT_; k++){
			if(-1 == dma_board2host(j, &gReadData[0])){
				printf("dma_board2host ERROR\n");
				return -1;
			}
		}
		gettimeofday(&b2pc_stop, NULL);

		// calculate speed
		b2pc_starts = b2pc_start.tv_sec;
		b2pc_startus = b2pc_start.tv_usec;
		b2pc_stops = b2pc_stop.tv_sec;
		b2pc_stopus = b2pc_stop.tv_usec;
		pc2b_starts = pc2b_start.tv_sec;
		pc2b_startus = pc2b_start.tv_usec;
		pc2b_stops = pc2b_stop.tv_sec;
		pc2b_stopus = pc2b_stop.tv_usec;
		printf("%d\t", j);
		printf("%.2lf\t", (j*(double)_REPEAT_*8/(((double)b2pc_stops - (double)b2pc_starts) + ((double)b2pc_stopus - (double)b2pc_startus)/1000000))/1024/1024);
		printf("%.2lf\t", (j*(double)_REPEAT_*8/(((double)pc2b_stops - (double)pc2b_starts) + ((double)pc2b_stopus - (double)pc2b_startus)/1000000))/1024/1024);
		printf("%.2lf\t", (((double)b2pc_stops - (double)b2pc_starts)*1000000000 + ((double)b2pc_stopus - (double)b2pc_startus))*1000/(double)_REPEAT_);
		printf("%.2lf\n", (((double)pc2b_stops - (double)pc2b_starts)*1000000000 + ((double)pc2b_stopus - (double)pc2b_startus))*1000/(double)_REPEAT_);
	}
	printf("=====================================================\n");
	return 0;
}

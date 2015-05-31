/*************************************
 *
 * zerocopy_test.c
 *
 * This file is part of EPEE project (version 2.0)
 * http: *cecaraw.pku.edu.cn
 * 
 * Description
 * Verifing the zero-copy functions will.
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
#include "sPcie.h"
#include "sPcieZerocopy.h"

#define DMA_LEN 1024*2048
/*This should be 8 in X8Gen2/X8Gen1 mode, can be 4 in other modes*/
#define DMA_MIN_SIZE 8

int main()
{
	char * sourceBuf;
	char * targetBuf;
	char ip_mode;
	int i, j, k,p;
	int error = -1;

	ip_mode = get_pcie_cfg_mode();
	printf(">configure mode:x%dgen%d\t", (ip_mode>>4)&0x0F, (ip_mode&0x0F));
	ip_mode = get_pcie_cur_mode();
	printf(" current mode:x%dgen%d\n", (ip_mode>>4)&0x0F, (ip_mode&0x0F));

	// get zerocopy buffer
	sourceBuf = (char *)get_zerocopy_buffer();
	targetBuf = sourceBuf + BUF_SIZE / 2;
	
	// initialize source buffer
	for(j = 0; j < DMA_LEN; j++){
		sourceBuf[j] = (char)((j+1)&0xFF);
	}

	// small data
	for(k = DMA_MIN_SIZE; k <= 4096; k+=DMA_MIN_SIZE){
		error = -1;
		printf("Test %d\n", k);
		// DMA
		while(0 > zerocopy_host2board(0, k));
		// check DMA done
		while(0 != get_host2board_status());
		// DMA
		while(0 > zerocopy_board2host(BUF_SIZE/2, k));
		// check DMA done
		while(0 != get_board2host_status());
		// check data
		for(j = 0; j < k; j++){
			if(sourceBuf[j] != (~targetBuf[j])){
				printf("ERROR\n");
				error = j;
				break;
			}
		}
		if(error != -1){
			printf("         source target\n");
			i = error;
			//for(i = error; i < error + 10; i++){
			printf("%8d: 0x%2x 0x%2x\n", i, sourceBuf[i]&0xFF, targetBuf[i]&0xFF);
			//}
			break;
		}
	}
	
	// big data up to DMA_LEN
	for(k = DMA_MIN_SIZE; k <= DMA_LEN; k*=2){
		error = -1;
		printf("Test %d\n", k);
		// DMA
		while(0 > zerocopy_host2board(0, k));
		while(0 > zerocopy_board2host(BUF_SIZE/2, k));
		// check DMA done
		while(0 != get_host2board_status());
		while(0 != get_board2host_status());
		// check data
		for(j = 0; j < k; j++){
			if(sourceBuf[j] != (~targetBuf[j])){
				printf("ERROR\n");
				error = j;
				break;
			}
		}
		if(error != -1){
			printf("         source target\n");
			i = error;
			//for(i = error; i < error + 10; i++){
			printf("%8d: 0x%2x 0x%2x\n", i, sourceBuf[i]&0xFF, targetBuf[i]&0xFF);
			//}
			break;
		}
	}
	release_zerocopy_buffer(sourceBuf);
	return 0;
}


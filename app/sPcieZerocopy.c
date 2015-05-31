/*************************************
 *
 * sPcieZerocopy.c
 *
 * This file is part of EPEE project (version 2.0)
 * http: *cecaraw.pku.edu.cn
 * 
 * Description
 * The zero-copy APIs are implemented in this source file.
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

#include "sPcieZerocopy.h"

/*****************************
 * function : get_zerocopy_buffer
 * engineer : Jian Gong
 * discription : Get the zerocopy buffer in driver, buffer size is BUF_SIZE
 *
 * return : return address of the buffer when success, NULL when error
 *
 * arguments :
 *         none
 *****************************/
void * get_zerocopy_buffer()
{
	char devname[] = DEV_NAME;
	int g_devFile = -1;
	char * devfilename = devname;
	void * buf_pointer;

	g_devFile = open(devfilename, O_RDWR);

	if(g_devFile < 0){
		return NULL;
	}
	
	buf_pointer = mmap(NULL, BUF_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, g_devFile, 0);
	
	close(g_devFile);
	return buf_pointer;
}

/*****************************
 * function : release_zerocopy_buffer
 * engineer : Jian Gong
 * discription : munmap the buffer
 *
 * return : 0 when success, -1 when error
 *
 * arguments :
 *         buf : pointer to the zero copy buffer
 *****************************/
int release_zerocopy_buffer(void * buf)
{
	char devname[] = DEV_NAME;
	int g_devFile = -1;
	char * devfilename = devname;
	int rtn_val;

	g_devFile = open(devfilename, O_RDWR);

	if(g_devFile < 0){
		return -1;
	}
	
	rtn_val = munmap(buf, BUF_SIZE);
	
	close(g_devFile);
	return rtn_val;
}

/*****************************
 * function : zerocopy_host2board
 * engineer : Jian Gong
 * discription : DMA data from zerocopy buffer to FPGA
 *
 * return : return size of DMA data when success, -1 when error
 *
 * arguments :
 *         offset : offset of the 1st data to be DMAed, (offset % 4) should be 0
 *         len : DMA data len in Byte, (len % 4) should be 0
 *****************************/
int zerocopy_host2board(int offset, int len)
{
	char devname[] = DEV_NAME;
	int g_devFile = -1;
	char * devfilename = devname;
	struct zero_copy_dma_req_struct dma_req;
	int rtn_val;

	if(offset < 0 || (offset + len) > BUF_SIZE || len < 0 || (offset % 4) != 0 || (len %4) != 0)
		return -1;
	
	g_devFile = open(devfilename, O_RDWR);
	if(g_devFile < 0){
		return -1;
	}
	
	dma_req.offset = offset;
	dma_req.count = len;
	rtn_val = ioctl(g_devFile, ZERO_COPY_WRITE, &dma_req);
	
	close(g_devFile);
	return rtn_val;
}

/*****************************
 * function : zerocopy_board2host
 * engineer : Jian Gong
 * discription : DMA data from FPGA to zerocopy buffer
 *
 * return : return size of DMA data when success, -1 when error
 *
 * arguments :
 *         offset : offset of the 1st data to be stored in zerocopy buffer, (offset % 4) should be 0
 *         len : DMA data len in Byte, (len % 4) should be 0
 *****************************/
int zerocopy_board2host(int offset, int len)
{
	char devname[] = DEV_NAME;
	int g_devFile = -1;
	char * devfilename = devname;
	struct zero_copy_dma_req_struct dma_req;
	int rtn_val;

	if(offset < 0 || (offset + len) > BUF_SIZE || len < 0 || (offset % 4) != 0 || (len %4) != 0)
		return -1;
	
	g_devFile = open(devfilename, O_RDWR);
	if(g_devFile < 0){
		return -1;
	}
	
	dma_req.offset = offset;
	dma_req.count = len;
	rtn_val = ioctl(g_devFile, ZERO_COPY_READ, &dma_req);
	
	close(g_devFile);
	return rtn_val;
}

/*****************************
 * function : get_host2board_status
 * engineer : Jian Gong
 * discription : Get host to board DMA status
 *
 * return : return 0 when idle, 1 when busy, -1 when error
 *
 * arguments :
 *         none
 *****************************/
int get_host2board_status()
{
	char devname[] = DEV_NAME;
	int g_devFile = -1;
	char * devfilename = devname;
	int rtn_val;

	g_devFile = open(devfilename, O_RDWR);

	if(g_devFile < 0){
		return -1;
	}
	
	rtn_val = ioctl(g_devFile, WRITE_STATUS, 0);
	
	close(g_devFile);
	return rtn_val;
}

/*****************************
 * function : get_board2host_status
 * engineer : Jian Gong
 * discription : Get board to host DMA status
 *
 * return : return 0 when idle, 1 when busy, -1 when error
 *
 * arguments :
 *         none
 *****************************/
int get_board2host_status()
{
	char devname[] = DEV_NAME;
	int g_devFile = -1;
	char * devfilename = devname;
	int rtn_val;

	g_devFile = open(devfilename, O_RDWR);

	if(g_devFile < 0){
		return -1;
	}
	
	rtn_val = ioctl(g_devFile, READ_STATUS, 0);
	
	close(g_devFile);
	return rtn_val;
}

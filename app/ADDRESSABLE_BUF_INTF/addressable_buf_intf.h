/*************************************
 *
 * addressable_buf_intf.h
 *
 * This file is part of EPEE project (version 2.0)
 * http: *cecaraw.pku.edu.cn
 * 
 * Description
 * addressable buffer interface library.
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

// PIO Channel select, the fixed address
#define PIO_CHN_PRE 0x18000

int read_usr_reg_ch(unsigned int reg)
{
	unsigned int value;
	int reg_num;
	if(reg > 1024*64)
		return -1;
	reg_num = (PIO_CHN_PRE) + reg;
	if(0 > read_usr_reg(reg_num, &value))
		return -1;
	return value;
}

int write_usr_reg_ch(unsigned int reg, unsigned int data)
{
	unsigned int value;
	int reg_num;
	if(reg > 1024*64)
		return -1;
	reg_num = (PIO_CHN_PRE) + reg;
	value = data;
	// printf("reg = %d, data = 0x%08x\n", reg, data);
	return write_usr_reg(reg_num, &value);
	// return 0;
}

/*****************************
 * function : dma_host2board
 * engineer : Jian Gong
 * discription : dma from host(PC) memory to FPGA board, please note
 *			that int is stored in mem in opposite byte manner
 *
 * return : return 0 if dma done successfully, else return -1
 *          if len is more than max payload or the buffer in FPGA
 *          can't contain the data, -1 will be returned
 * arguments :
 *         len: length of data to DMA (in Byte), it should : (len % 8 == 0 && len != 0)
 *         p_data: pointer of the first data
 *         addr : 1st addr of the data
 *****************************/
int dma_host2board_addressable(unsigned int len, unsigned char *p_data, int addr);

/*****************************
 * function : dma_board2host
 * engineer : Jian Gong
 * discription : dma from FPGA board to host(PC) memory, please note
 *			that int is stored in mem in opposite byte manner
 *
 * return : return 0 if dma done successfully, else return -1
 *          when data available in FPGA buffer is less than argument
 *          len, or argument len is more than max payload, -1 will be
 *          returned
 * arguments :
 *         len: length of data to DMA (in Byte), it should : (len % 8 == 0 && len != 0)
 *         p_data: pointer of the first data
 *         addr : 1st addr of the data
 *****************************/
int dma_board2host_addressable(unsigned int len, unsigned char *p_data, int addr);

int dma_host2board_addressable(unsigned int len, unsigned char *p_data, int addr)
{
	if(0 != read_usr_reg_ch(1)) // busy
		return -1;
	if((len % 8) != 0)
		return -1;
	write_usr_reg_ch(2, len/8); // len
	write_usr_reg_ch(3, addr); // addr
	write_usr_reg_ch(0, 1); // start
	return dma_host2board(len, p_data);
}

int dma_board2host_addressable(unsigned int len, unsigned char *p_data, int addr)
{
	if(0 != read_usr_reg_ch(5)) // busy
		return -1;
	if((len % 8) != 0)
		return -1;
	write_usr_reg_ch(6, len/8); // len
	write_usr_reg_ch(7, addr); // addr
	write_usr_reg_ch(4, 1); // start
	return dma_board2host(len, p_data);
}

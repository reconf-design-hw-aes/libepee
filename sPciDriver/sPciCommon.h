/*************************************
 *
 * sPciCommon.h
 *
 * This file is part of EPEE project (version 2.0)
 * http: *cecaraw.pku.edu.cn
 * 
 * Description
 * Some common parameters of the driver and user mode library
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

#ifndef __SPCICOMMON_H_
#define __SPCICOMMON_H_

/*register address size*/
#define REG_SIZE (1024*1024)
/*MAX length of DMA buffer*/
#define BUF_SIZE (1024*4096)

/*enum for ioctl*/
enum{
	SYS_PIO_WRITE,
	SYS_PIO_READ,
	RESERVED,
	PCIE_CFG_MODE,
	PCIE_CUR_MODE,
	USR_PIO_WRITE,
	USR_PIO_READ,
	USR_INT_WAIT,
	ZERO_COPY_READ, /*file operation read : DMA write*/
	ZERO_COPY_WRITE, /*file operation write : DMA read*/
	READ_STATUS, /*file operation read : DMA write*/
	WRITE_STATUS /*file operation write : DMA read*/
};

/****************** struct for pio **********************/
struct register_struct{
	int reg;
	unsigned int value;
};

/****************** struct for zero copy dma **********************/
struct zero_copy_dma_req_struct{
	int offset;
	unsigned int count;
};

#endif
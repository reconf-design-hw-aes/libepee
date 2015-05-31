/*************************************
 *
 * sPcie.h
 *
 * This file is part of EPEE project (version 2.0)
 * http: *cecaraw.pku.edu.cn
 * 
 * Description
 * API description
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

#ifndef __SPCIE_H_
#define __SPCIE_H_

#include <sys/wait.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <termios.h>
#include <fcntl.h>
#include <errno.h>

#include "sPciCommon.h"

#define DEV_NAME "/dev/sPciDriver"

#ifdef __cplusplus
extern "C" {
#endif

/*****************************
 * function : get_pcie_cfg_mode
 * engineer : Jian Gong
 * discription : gets configered PCI Express mode (eg. x1gen1, x4gen2...)
 *
 * return : return the configered mode, the higher 4bits is number of lanes
 *          the lower 4bits is generation(gen1/gen2)
 *          0x11:x1gen1, 0x12:x1gen2, 0x21:x2gen1, 0x22:x2gen2...
 * arguments :
 *         none
 *****************************/
char get_pcie_cfg_mode();

/*****************************
 * function : get_pcie_cur_mode
 * engineer : Jian Gong
 * discription : gets current PCI Express mode (eg. x1gen1, x4gen2...)
 *
 * return : return the configered mode, the higher 4bits is number of lanes
 *          the lower 4bits is generation(gen1/gen2)
 *          0x11:x1gen1, 0x12:x1gen2, 0x21:x2gen1, 0x22:x2gen2...
 * arguments :
 *         none
 *****************************/
char get_pcie_cur_mode();

/*****************************
 * function : reset
 * engineer : Jian Gong
 * discription : reset the user hardware and the DMA system
 *
 * return : return 0 if reset successfully, else return -1
 * arguments :
 *         none
 *****************************/
int sys_reset();
int usr_reset();
int host2board_reset();
int board2host_reset();

/*****************************
 * function : get_host2board_count
 * engineer : Jian Gong
 * discription : get how many data has been dma from host to board
 *
 * return : return data count, if error, return -1
 * arguments :
 *         none
 *****************************/
int get_host2board_count();

/*****************************
 * function : get_board2host_count
 * engineer : Jian Gong
 * discription : get how many data has been dma from board to host
 *               (including the data prepared to be dma)
 *
 * return : return data count, if error, return -1
 * arguments :
 *         none
 *****************************/
int get_board2host_count();
 
/*****************************
 * function : read_usr_reg
 * engineer : Jian Gong
 * discription : read user register
 *
 * return : return 0 if read reg successfully, else return -1
 *          if the number of reg is invalid, -1 will be returned
 * arguments :
 *         reg: the number of the reg to read
 *         p_data: pointer to the value of the reg
 *****************************/
int read_usr_reg(unsigned int reg, unsigned int *p_data);

/*****************************
 * function : write_usr_reg
 * engineer : Jian Gong
 * discription : write user register
 *
 * return : return 0 if read reg successfully, else return -1
 *          if the number of reg is invalid, -1 will be returned
 * arguments :
 *         reg: the number of the reg to read
 *         p_data: pointer to the value of the reg
 *****************************/
int write_usr_reg(unsigned int reg, unsigned int *p_data);

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
 *         len: length of data to DMA (in Byte), it should : (len % 4 == 0 && len != 0)
 *         p_data: pointer of the first data
 *****************************/
int dma_host2board(unsigned int len, void *p_data);
int dma_host2board_unblocking(unsigned int len, void *p_data);

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
 *         len: length of data to DMA (in Byte), it should : (len % 4 == 0 && len != 0)
 *         p_data: pointer of the first data
 *****************************/
int dma_board2host(unsigned int len, void *p_data);

/*****************************
 * function : block_until_interrupt
 * engineer : Jian Gong
 * discription : Block the current process and wait for user defined 
 *               interrupt.
 *
 * return : return 0 if an interrupt occurred. If timeout, return -1
 * arguments : 
 *         vector_num: the vectory of the interrupt, ranges from 0 to 7
 *****************************/
int block_until_interrupt(int vector_num);


#ifdef __cplusplus
}
#endif
#endif

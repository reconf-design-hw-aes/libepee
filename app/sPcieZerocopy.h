/*************************************
 *
 * sPcieZerocopy.h
 *
 * This file is part of EPEE project (version 2.0)
 * http: *cecaraw.pku.edu.cn
 * 
 * Description
 * Zero-Copy APIs
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

#ifndef __SPCIEZEROCOPY_H_
#define __SPCIEZEROCOPY_H_

#include <sys/wait.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <termios.h>
#include <fcntl.h>
#include <errno.h>

#include "sPciCommon.h"

#define DEV_NAME "/dev/sPciDriver"

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
void * get_zerocopy_buffer();

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
int release_zerocopy_buffer(void * buf);

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
int zerocopy_host2board(int offset, int len);

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
int zerocopy_board2host(int offset, int len);

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
int get_host2board_status();

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
int get_board2host_status();

#endif

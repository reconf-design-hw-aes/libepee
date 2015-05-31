/*************************************
 *
 * partial_reconf.c
 *
 * This file is part of EPEE project (version 2.0)
 * http: *cecaraw.pku.edu.cn
 * 
 * Description
 * Partial reconfiguration.
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

int main(int argc, char ** argv)
{
	FILE * fp;
	int data;
	int len, i;
	char * bitstream;
	int startp, endp; // start & end of the bitstream
	
	if(argc != 2){
		puts("Usage: ./configure filename");
		return -1;
	}
	fp = fopen(argv[1], "r");
	if(NULL == fp){
		puts("open file error!");
		return -1;
	}
	else{
		puts("begin configure");
	}
	
	fseek(fp, 0l, SEEK_END);
	len = ftell(fp);
	fseek(fp, 0l, SEEK_SET);

	bitstream = (char *)malloc(10 + len*sizeof(char));
	i = 0;
	while(0 < fread(&bitstream[i++], sizeof(char), 1, fp) && i < len);
	
	// start syn
	for(i = 0; i < len; i++){// syn DW is 0xaa995566
		if(((0xff & bitstream[i]) == 0x0aa) && ((0xff & bitstream[i+1]) == 0x099) && ((0xff & bitstream[i+2]) == 0x055) && ((0xff & bitstream[i+3]) == 0x066))
			break;
	}
	startp = i;
	
	// end syn
	for(i = len - 4; i >= 0; i--){ // syn DW is 0x0000000d
		if(((0xff & bitstream[i]) == 0x00) && ((0xff & bitstream[i+1]) == 0x00) && ((0xff & bitstream[i+2]) == 0x00) && ((0xff & bitstream[i+3]) == 0x0d))
			break;
	}
	endp = i;
	
	if(startp >= endp){
		puts("Bit file error!");
		return -1;
	}
	
	/*send start signal*/
	write_usr_reg_ch(0, 1); // write reg0 with 1
	/*send configure data*/
	for(i = startp; i <= endp; i+=4){
//printf("%x %x %x %x\n", bitstream[i], bitstream[1+i], bitstream[2+i], bitstream[3+i]);
		data = (((bitstream[i] << 24) & 0xFF000000) | ((bitstream[i+1] << 16) & 0x00FF0000) | ((bitstream[i+2] << 8) & 0x0000FF00) | ((bitstream[i+3] << 0) & 0x000000FF));
		write_usr_reg_ch(2, data);
	}
	/*send end signal*/
	write_usr_reg_ch(1, 1);
	puts("configure done");
	
	free(bitstream);
	fclose(fp);
	return 0;
}

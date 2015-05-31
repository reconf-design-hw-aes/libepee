/*************************************
 *
 * reset_test.c
 *
 * This file is part of EPEE project (version 2.0)
 * http: *cecaraw.pku.edu.cn
 * 
 * Description
 * Test reset of EPEE
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

int main()
{
	int cmd;
	int board2host_data_count;
	int host2board_data_count;
	printf("1. sys_reset\n");
	printf("2. usr_reset\n");
	printf("3. host2board_reset\n");
	printf("4. board2host_reset\n");
	printf("choose the kind of reset:");
	scanf("%d", &cmd);
	board2host_data_count = get_board2host_count();
	host2board_data_count = get_host2board_count();
	printf("host2board data count (DW) : %d\n", host2board_data_count);
	printf("board2host data count (DW) : %d\n", board2host_data_count);
	switch(cmd){
		case 1:
			printf("start sys_reset\n");
			sys_reset();
			break;
		case 2:
			printf("start usr_reset\n");
			usr_reset();
			break;
		case 3:
			printf("start host2board_reset\n");
			host2board_reset();
			break;
		case 4:
			printf("start board2host_reset\n");
			board2host_reset();
			break;
		default:
			printf("cmd error\n");
			return 0;
	}
	board2host_data_count = get_board2host_count();
	host2board_data_count = get_host2board_count();
	printf("host2board data count (DW) : %d\n", host2board_data_count);
	printf("board2host data count (DW) : %d\n", board2host_data_count);
	
	return 0;
}

/*************************************
 *
 * INT_test.c
 *
 * This file is part of EPEE project (version 2.0)
 * http: *cecaraw.pku.edu.cn
 * 
 * Description
 * Test interrupt
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

#include <pthread.h>
#include <stdio.h>
#include "sPcie.h"

void * wait_for_int0(void *p)
{
	int i;
	for(i = 0; i < 1000; i++){
		printf("test %d: wait for interrupt 0 ...\n", i);
		if(-1 == block_until_interrupt(0)){
			printf("interrupt error 0\n");
		}
		else{
			printf("interrupt 0 ok\n");
		}
	}
	return NULL;
}

void * wait_for_int1(void *p)
{
	int i;
	for(i = 0; i < 1000; i++){
		printf("test %d: wait for interrupt 1 ...\n", i);
		if(-1 == block_until_interrupt(1)){
			printf("interrupt error 1\n");
		}
		else{
			printf("interrupt 1 ok\n");
		}
	}
	return NULL;
}

int main()
{
	pthread_t interrupt0, interrupt1;
	pthread_create(&interrupt0, NULL, wait_for_int0, NULL);
	pthread_create(&interrupt1, NULL, wait_for_int1, NULL);

	pthread_join(interrupt0, NULL);
	pthread_join(interrupt1, NULL);
	return 0;
}

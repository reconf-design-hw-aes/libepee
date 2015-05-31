/*************************************
 *
 * sPcie.c
 *
 * This file is part of EPEE project (version 2.0)
 * http: *cecaraw.pku.edu.cn
 * 
 * Description
 * The EPEE software APIs are implemented in this source file.
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

#include "sPcie.h"

char get_pcie_cfg_mode()
{
  char devname[] = DEV_NAME;
  int g_devFile = -1;

  unsigned int reg_value;
  char* devfilename = devname;

  char ret;

  g_devFile = open(devfilename, O_RDWR);

  if ( g_devFile < 0 )  {
    printf("Error opening device file\n");
    return -1;
  }

  if (ioctl(g_devFile, PCIE_CFG_MODE, &reg_value) < 0) {
    return -1;
  }
  ret = reg_value&0x000000FF;

  close(g_devFile);
  return ret;
}

char get_pcie_cur_mode()
{
  char devname[] = DEV_NAME;
  int g_devFile = -1;

  unsigned int reg_value;
  char* devfilename = devname;

  char ret;

  g_devFile = open(devfilename, O_RDWR);

  if ( g_devFile < 0 )  {
    printf("Error opening device file\n");
    return -1;
  }

  if (ioctl(g_devFile, PCIE_CUR_MODE, &reg_value) < 0) {
    return -1;
  }
  ret = (reg_value>>16)&0x000000FF;
  
  close(g_devFile);
  return ret;
}

int sys_reset()
{
  char devname[] = DEV_NAME;
  int g_devFile = -1;
  char* devfilename = devname;

  struct register_struct reg;

  g_devFile = open(devfilename, O_RDWR);
  if(g_devFile < 0){
    return -1;
  }

  reg.reg = 1;
  reg.value = 1;
  if (ioctl(g_devFile, SYS_PIO_WRITE, &reg) < 0) {
    return -1;
  }
  while(1){
    reg.reg = 1;
    reg.value = 0;
    if (ioctl(g_devFile, SYS_PIO_READ, &reg) < 0) {
      return -1;
    }
    if((reg.value & 0x10) && (reg.value & 0x20) && (reg.value & 0x40)){
      break;
    }
  }
  reg.reg = 1;
  reg.value = 0;
  if (ioctl(g_devFile, SYS_PIO_WRITE, &reg) < 0) {
    return -1;
  }
  close(g_devFile);
  
  return 0;
}

int usr_reset()
{
  char devname[] = DEV_NAME;
  int g_devFile = -1;
  char* devfilename = devname;

  struct register_struct reg;

  g_devFile = open(devfilename, O_RDWR);
  if(g_devFile < 0){
    return -1;
  }

  reg.reg = 1;
  reg.value = 2;
  if (ioctl(g_devFile, SYS_PIO_WRITE, &reg) < 0) {
    return -1;
  }
  while(1){
    reg.reg = 1;
    reg.value = 0;
    if (ioctl(g_devFile, SYS_PIO_READ, &reg) < 0) {
      return -1;
    }
    if(reg.value & 0x40){
      break;
    }
  }
  reg.reg = 1;
  reg.value = 0;
  if (ioctl(g_devFile, SYS_PIO_WRITE, &reg) < 0) {
    return -1;
  }
  close(g_devFile);
  
  return 0;
}

int host2board_reset()
{
  char devname[] = DEV_NAME;
  int g_devFile = -1;
  char* devfilename = devname;

  struct register_struct reg;

  g_devFile = open(devfilename, O_RDWR);
  if(g_devFile < 0){
    return -1;
  }

  reg.reg = 1;
  reg.value = 4;
  if (ioctl(g_devFile, SYS_PIO_WRITE, &reg) < 0) {
    return -1;
  }
  while(1){
    reg.reg = 1;
    reg.value = 0;
    if (ioctl(g_devFile, SYS_PIO_READ, &reg) < 0) {
      return -1;
    }
//printf("reg0 = 0x%x\n", reg.reg);
    if(reg.value & 0x10){
      break;
    }
  }
  reg.reg = 1;
  reg.value = 0;
  if (ioctl(g_devFile, SYS_PIO_WRITE, &reg) < 0) {
    return -1;
  }
  close(g_devFile);
  
  return 0;
}

int board2host_reset()
{
  char devname[] = DEV_NAME;
  int g_devFile = -1;
  char* devfilename = devname;

  struct register_struct reg;

  g_devFile = open(devfilename, O_RDWR);
  if(g_devFile < 0){
    return -1;
  }

  reg.reg = 1;
  reg.value = 8;
  if (ioctl(g_devFile, SYS_PIO_WRITE, &reg) < 0) {
    return -1;
  }
  while(1){
    reg.reg = 1;
    reg.value = 0;
    if (ioctl(g_devFile, SYS_PIO_READ, &reg) < 0) {
      return -1;
    }
    if(reg.value & 0x20){
      break;
    }
  }
  reg.reg = 1;
  reg.value = 0;
  if (ioctl(g_devFile, SYS_PIO_WRITE, &reg) < 0) {
    return -1;
  }
  close(g_devFile);
  
  return 0;
}

int get_host2board_count()
{
  char devname[] = DEV_NAME;
  int g_devFile = -1;
  char* devfilename = devname;

  struct register_struct reg_struct;

  g_devFile = open(devfilename, O_RDWR);
  if(g_devFile < 0){
    return -1;
  }

  reg_struct.reg = 11;
  reg_struct.value = 0;
  if (ioctl(g_devFile, SYS_PIO_READ, &reg_struct) < 0) {
    return -1;
  }
  close(g_devFile);
  
  return reg_struct.value;
}

int get_board2host_count()
{
  char devname[] = DEV_NAME;
  int g_devFile = -1;
  char* devfilename = devname;

  struct register_struct reg_struct;

  g_devFile = open(devfilename, O_RDWR);
  if(g_devFile < 0){
    return -1;
  }

  reg_struct.reg = 12;
  reg_struct.value = 0;
  if (ioctl(g_devFile, SYS_PIO_READ, &reg_struct) < 0) {
    return -1;
  }

  close(g_devFile);
  
  return reg_struct.value;
}

int read_usr_reg(unsigned int reg, unsigned int *p_data)
{
  char devname[] = DEV_NAME;
  int g_devFile = -1;
  char* devfilename = devname;

  struct register_struct reg_struct;

  g_devFile = open(devfilename, O_RDWR);
  if(g_devFile < 0){
    return -1;
  }

  reg_struct.reg = reg;
  reg_struct.value = 0;
  if (ioctl(g_devFile, USR_PIO_READ, &reg_struct) < 0) {
    return -1;
  }

  *p_data = reg_struct.value;

  close(g_devFile);
  
  return 0;
}

int write_usr_reg(unsigned int reg, unsigned int *p_data)
{
  char devname[] = DEV_NAME;
  int g_devFile = -1;
  char* devfilename = devname;

  struct register_struct reg_struct;

  g_devFile = open(devfilename, O_RDWR);
  if(g_devFile < 0){
    return -1;
  }

  reg_struct.reg = reg;
  reg_struct.value = *p_data;
  if (ioctl(g_devFile, USR_PIO_WRITE, &reg_struct) < 0) {
    return -1;
  }

  close(g_devFile);
  
  return 0;
}

int dma_host2board(unsigned int len, void *p_data)
{
  char devname[] = DEV_NAME;
  int g_devFile = -1;
  char* devfilename = devname;

  int ret_val = 0;
/*if(4096 < len)return -1;*/
  g_devFile = open(devfilename, O_RDWR);

  if(g_devFile < 0){
    return -1;
  }

  ret_val = write(g_devFile, (char *)p_data, len);

  close(g_devFile);
  
  return ret_val;
}

int dma_host2board_unblocking(unsigned int len, void *p_data)
{
  char devname[] = DEV_NAME;
  int g_devFile = -1;
  char* devfilename = devname;

  int ret_val = 0;
/*if(4096 < len)return -1;*/
  g_devFile = open(devfilename, O_RDWR | O_NONBLOCK);

  if(g_devFile < 0){
    return -1;
  }

  ret_val = write(g_devFile, (char *)p_data, len);

  close(g_devFile);
  
  return ret_val;
}

int dma_board2host(unsigned int len, void *p_data)
{
  char devname[] = DEV_NAME;
  int g_devFile = -1;
  char* devfilename = devname;

  int ret_val = 0;
/*if(4096 < len)return -1;*/
  g_devFile = open(devfilename, O_RDWR);

  if(g_devFile < 0){
    return -1;
  }

  ret_val = read(g_devFile, (char *)p_data, len);

  close(g_devFile);
  
  return ret_val;
}

int block_until_interrupt(int vector_num)
{
  char devname[] = DEV_NAME;
  int g_devFile = -1;
  char* devfilename = devname;

  int ret_val = 0;

  if(vector_num < 0 || vector_num > 7)
    return -1;

  g_devFile = open(devfilename, O_RDWR);

  if(g_devFile < 0){
    return -1;
  }

  if ((ret_val = ioctl(g_devFile, USR_INT_WAIT, vector_num)) < 0) {
    return -1;
  }

  close(g_devFile);
  
  return ret_val;
}

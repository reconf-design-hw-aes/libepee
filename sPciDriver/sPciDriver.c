/*************************************
 *
 * sPciDriver.c
 *
 * This file is part of EPEE project (version 2.0)
 * http: *cecaraw.pku.edu.cn
 * 
 * Description
 * The driver of EPEE
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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/fs.h>
#include <linux/version.h>

#include "sPciCommon.h"

/*Comment this line in 32bit OS*/
#define OS_64BIT_EN
/*Comment this line when using 64bit interface*/
// #define EPEE_HW_INTERFACE_128

#define PCI_VENDOR_ID_XILINX      0x10ee

#define SUCCESS 0
#define ERROR   -1

#define HAVE_REGION  0x01
#define HAVE_IRQ     0x02
#define HAVE_REG_DEV 0x04

#define SPCI_DMA_MASK 0x00FFFFFFFFFFFF

#define SYS_REG_OFFSET(REG_NUM) ((REG_NUM) * 4)
#define USR_REG_OFFSET(REG_NUM) ((REG_NUM) * 4 + (REG_SIZE / 2))

MODULE_LICENSE("DUAL BSD/GPL");

const int pci_devMay = 241; /*mayjor number not dynamic*/
const char pci_devName[] = "sPci"; /*name of the device*/
struct pci_dev * pci_dev_struct = NULL; /*pci device struct*/
unsigned long pci_bar_hw_addr; /*hardware base address of the device*/
unsigned long pci_bar_size; /*hardware base memory size of the device*/
void * pci_bar_vir_addr = NULL; /*hardware base virtual address*/

/** Buffer for DMA read/write descriptors
  */
dma_addr_t DMA_R_DSC_ADDR; /*dma address of dma read descriptor buffer*/
dma_addr_t DMA_W_DSC_ADDR; /*dma address of dma write descriptor buffer*/
char * dma_r_descriptor_buf; /*dma read descriptor buffer*/
char * dma_w_descriptor_buf; /*dma write descriptor buffer*/

/** These buffers are used to buffer dma data
  */
dma_addr_t DMA_R_BUF_ADDR; /*dma address of dma read buffer*/
dma_addr_t DMA_W_BUF_ADDR; /*dma address of dma write buffer*/
char * dma_r_buf; /*dma read data buffer*/
char * dma_w_buf; /*dma write data buffer*/
/** Zero copy buffer
  */
dma_addr_t ZERO_COPY_BUF_ADDR; /*dma address*/
char * zero_copy_buf; /*virtual memory address*/

/*wait queue for blocking*/
static wait_queue_head_t dma_r_wait; /*dma read wait queue*/
static wait_queue_head_t dma_w_wait; /*dma write wait queue*/
static wait_queue_head_t user_wait[8];  /*user wait queue*/

atomic_t dma_r_int_occur; /*initial is 0, when interrupt occur, it will increase by 1*/
atomic_t dma_w_int_occur; /*initial is 0, when interrupt occur, it will increase by 1*/
atomic_t user_int_occur[8]; /*initial is 0, when interrupt occur, it will increase by 1*/
/*dma status*/
atomic_t dma_r_status; /*0:initial, 1:dma done, 2:dma error*/
atomic_t dma_w_status; /*0:initial, 1:dma done, 2:dma error*/

/*DMA busy*/
atomic_t dma_r_busy;
atomic_t dma_w_busy;

/*semaphore for critical resources when dma/interrupt*/
struct semaphore dma_r_sem;
struct semaphore dma_w_sem;
struct semaphore usr_int_sem;

/** device driver state flags
  */
unsigned int pci_state_flag = 0x00; /*status of pci driver*/
unsigned int msi_int_enabled = 0; /*indicates whether MSI interrupt is enabled*/

/****************** device init and exit ******************/
/*when insmod, pci_init will be called*/
static int  __init pci_init(void);
/*when rmmod, pci_exit will be called*/
static void pci_exit(void);

/*when calling function pci_register_driver, pci_probe will be called*/
static int  pci_probe(struct pci_dev * pci_dev, const struct pci_device_id * pci_id);
/*when calling function pci_unregister_driver, pci_remove will be called*/
static void pci_remove(struct pci_dev * pci_dev);

/******************** irq handler ************************/
/*interrupt handler of pci driver*/
static irqreturn_t pci_IRQHandler (int irq, void *dev_id);
/******************** interrupt handler (called by irq handler) *******************/
void dma_r_int_handler(u32 hw_status);
void dma_w_int_handler(u32 hw_status);
void user_int_handler(u32 hw_status);

/******************** file operations ********************/
/*item for file_operations: open*/
int     pci_open(struct inode *inode, struct file *filep);
/*item for file_operations: release*/
int     pci_release(struct inode *inode, struct file *filep);
/*item for file_operations: read*/
ssize_t pci_read(struct file *filep, char *buf, size_t count, loff_t *f_pos);
/*item for file_operations: write*/
ssize_t pci_write(struct file *filep, const char *buf, size_t count, loff_t *f_pos);
/*item for file_operations: unlocked ioctl*/
long    pci_unlocked_ioctl(struct file *filep, unsigned int cmd, unsigned long arg);
/*item for file_operations: mmap*/
static int pci_map(struct file * filep, struct vm_area_struct * vma);
/******************** read and write operations (called by pci_read/pci_write) *************/
ssize_t pci_read_blocking(char *buf, size_t count);
ssize_t pci_read_zerocopy(int offset, size_t count);
ssize_t pci_write_blocking(const char *buf, size_t count);
ssize_t pci_write_unblocking(const char *buf, size_t count);
ssize_t pci_write_zerocopy(int offset, size_t count);

/******************** register read and write *******************/
u32  pci_reg_read(int reg_num);
void pci_reg_write(int reg_num, u32 data);

/*file operations*/
struct file_operations pci_fops = {
	read:    pci_read,
	write:   pci_write,
	unlocked_ioctl: pci_unlocked_ioctl,
	open:    pci_open,
	release: pci_release,
	mmap:    pci_map,
};

/*?*/
static struct pci_device_id pci_ids[] = {
	{PCI_DEVICE(PCI_VENDOR_ID_XILINX, 0x6011),},
	{PCI_DEVICE(PCI_VENDOR_ID_XILINX, 0x6022),},
	{PCI_DEVICE(PCI_VENDOR_ID_XILINX, 0x6024),},
	{PCI_DEVICE(PCI_VENDOR_ID_XILINX, 0x7011),},
	{PCI_DEVICE(PCI_VENDOR_ID_XILINX, 0x7022),},
	{PCI_DEVICE(PCI_VENDOR_ID_XILINX, 0x7024),},
	{PCI_DEVICE(PCI_VENDOR_ID_XILINX, 0x7028),},
	{PCI_DEVICE(PCI_VENDOR_ID_XILINX, 0x0007),},
	{0,}
};

/*driver module information*/
static struct pci_driver pci_driver_struct = {
	name  :    pci_devName,
	id_table : pci_ids,
	probe :    pci_probe,
	remove:    pci_remove
};

module_init(pci_init);
module_exit(pci_exit);

/********************** init and exit function **********************/
/*when insmod, pci_init will be called*/
static int  __init pci_init(void)
{
	int return_value;
	int i;
	/*find PCI device, to check if the device is in PCIe bus*/
	for(i = 0; i < sizeof(pci_ids) / sizeof(struct pci_device_id); i++){
		if(pci_ids[i].vendor != 0)
			pci_dev_struct = pci_get_device(pci_ids[i].vendor, pci_ids[i].device, pci_dev_struct);
		if(NULL != pci_dev_struct)
			break;
	}
	if(NULL == pci_dev_struct){
		printk(KERN_INFO"%s:<init>Module init error. Hardware not found. Check you device id with \"lspci\"", pci_devName);
		return ERROR;
	}

	return_value = pci_register_driver(&pci_driver_struct);
	if(return_value < 0)	{
		printk(KERN_INFO"%s:<init>Module init error.", pci_devName);
		return return_value;
	}
	
	/*initial wait queue*/
	init_waitqueue_head(&dma_r_wait);
	init_waitqueue_head(&dma_w_wait);
	for(i = 0; i < 8; i++){
		init_waitqueue_head(&user_wait[i]);
	}
	
	/*initial atomic_t for interrupt*/
	atomic_set(&dma_r_int_occur, 0);
	atomic_set(&dma_w_int_occur, 0);
	for(i = 0; i < 8; i++){
		atomic_set(&user_int_occur[i], 0);
	}
	
	/*dma & user interrupt status*/
	atomic_set(&dma_r_status, 0);
	atomic_set(&dma_w_status, 0);
	
	/*dma busy*/
	atomic_set(&dma_r_busy, 0);
	atomic_set(&dma_w_busy, 0);

	sema_init(&dma_r_sem, 1);
	sema_init(&dma_w_sem, 1);
	sema_init(&usr_int_sem, 1);

	/*write DMA descriptor address*/
	pci_reg_write(SYS_REG_OFFSET(2), DMA_R_DSC_ADDR);
	pci_reg_write(SYS_REG_OFFSET(4), DMA_W_DSC_ADDR);
	#ifdef OS_64BIT_EN
	pci_reg_write(SYS_REG_OFFSET(3), DMA_R_DSC_ADDR>>32);

	pci_reg_write(SYS_REG_OFFSET(5), DMA_W_DSC_ADDR>>32);
	#else
	pci_reg_write(SYS_REG_OFFSET(3), 0);
	pci_reg_write(SYS_REG_OFFSET(5), 0);
	#endif
	
	/*enable interrupt*/
	if(pci_state_flag & HAVE_IRQ){
		pci_reg_write(SYS_REG_OFFSET(9), 0x00000001);
	}
	
	return return_value;
}

/*when rmmod, pci_exit will be called*/
static void pci_exit(void)
{
	pci_unregister_driver(&pci_driver_struct);
}

/********************** probe and remove function **********************/
/*when calling function pci_register_driver, pci_probe will be called*/
static int pci_probe(struct pci_dev * pci_dev, const struct pci_device_id * pci_id)
{
	/*get pci_dev struct, store it in pci_dev_struct*/
	pci_dev_struct = pci_dev;
	if(NULL == pci_dev_struct){
		printk(KERN_INFO"%s:<probe>struct pci_dev_struct is NULL\n", pci_devName);
		return ERROR;
	}
	
	/**
	 * Remap the IO memory space
	 */
	/*get base hardware address from pci_dev struct*/
	pci_bar_hw_addr = pci_resource_start(pci_dev_struct, 0);
	if(0 > pci_bar_hw_addr){
		printk(KERN_INFO"%s:<probe>base hardware address not set\n", pci_devName);
		return ERROR;
	}
	/*get base memory space size*/
	pci_bar_size = pci_resource_len(pci_dev_struct, 0);
	/*map hardware memory space to virtual space*/
	pci_bar_vir_addr = ioremap(pci_bar_hw_addr, pci_bar_size);
	if(0 == pci_bar_vir_addr){
		printk(KERN_INFO"%s:<probe>ioremap ERROR when mapping to virtual address\n", pci_devName);
		return ERROR;
	}

	/**
	 * init hardware
	 */
	/*check memory region*/
	if(0 > check_mem_region(pci_bar_hw_addr, REG_SIZE)){
		printk(KERN_INFO"%s:<probe>memory in use\n", pci_devName);
		return ERROR;
	}
	request_mem_region(pci_bar_hw_addr, REG_SIZE, "3GIO_Demo_Drv");
	pci_state_flag = pci_state_flag | HAVE_REGION;
	
	/*enable device*/
	if(0 > pci_enable_device(pci_dev_struct)){
		printk(KERN_INFO"%s:<probe>device enable error\n", pci_devName);
		return ERROR;
	}
	
	/*set DMA mask*/
	if(0 != dma_set_mask(&pci_dev->dev, SPCI_DMA_MASK)){
		printk(KERN_INFO"%s:<probe>set DMA mask error\n", pci_devName);
		return ERROR;
	}
	
	/*enable bus master*/
	pci_set_master(pci_dev);
  
	/*enable MSI*/
	if(0 > pci_enable_msi(pci_dev_struct)){
		printk(KERN_INFO"%s:<probe>msi emable error, legacy interrupt will be used.\n", pci_devName);
		msi_int_enabled = 0;
	}
	else
		msi_int_enabled = 1;
	
	/*request IRQ*/
	if(0 > request_irq(pci_dev_struct->irq, &pci_IRQHandler, IRQF_SHARED, pci_devName, pci_dev_struct)){
		printk(KERN_INFO"%s:<probe>request irq error\n", pci_devName);
		return ERROR;
	}
	pci_state_flag = pci_state_flag | HAVE_IRQ;
	
	/**
	 * require descriptor buffers
	 */
	/*dma read descriptor buffer*/
	dma_r_descriptor_buf = pci_alloc_consistent(pci_dev_struct, BUF_SIZE, &DMA_R_DSC_ADDR);
	if(NULL == dma_r_descriptor_buf){
		printk(KERN_INFO"%s:<probe>DMA read descriptor buffer alloc error\n", pci_devName);
		return ERROR;
	}
	
	/*dma board to host buffer*/
	dma_w_descriptor_buf = pci_alloc_consistent(pci_dev_struct, BUF_SIZE, &DMA_W_DSC_ADDR);
	if(NULL == dma_w_descriptor_buf){
		printk(KERN_INFO"%s:<probe>DMA write descriptor buffer alloc error\n", pci_devName);
		return ERROR;
	}
	
	/**
	 * require data buffers
	 */
	/*dma read buffer*/
	dma_r_buf = pci_alloc_consistent(pci_dev_struct, BUF_SIZE, &DMA_R_BUF_ADDR);
	if(NULL == dma_r_buf){
		printk(KERN_INFO"%s:<probe>dma read buffer alloc error\n", pci_devName);
		return ERROR;
	}
	
	/*dma write buffer*/
	dma_w_buf = pci_alloc_consistent(pci_dev_struct, BUF_SIZE, &DMA_W_BUF_ADDR);
	if(NULL == dma_w_buf){
		printk(KERN_INFO"%s:<probe>dma write buffer alloc error\n", pci_devName);
		return ERROR;
	}
	
	/**
	 * require zero copy buffer
	 */
	zero_copy_buf = pci_alloc_consistent(pci_dev_struct, BUF_SIZE, &ZERO_COPY_BUF_ADDR);
	if(NULL == zero_copy_buf){
		printk(KERN_INFO"%s:<probe>zero copy buffer alloc error\n", pci_devName);
		return ERROR;
	}
	
	/**
	 * register pci device
	 */
	if(0 > register_chrdev(pci_devMay, pci_devName, &pci_fops)){
		printk(KERN_INFO"%s:<probe>driver not registered\n", pci_devName);
		return ERROR;
	}
	pci_state_flag = pci_state_flag | HAVE_REG_DEV;
	
	return 0;
}

/*when calling function pci_unregister_driver, pci_remove will be called*/
static void pci_remove(struct pci_dev * pci_dev)
{
	if(pci_state_flag & HAVE_REGION){
		release_mem_region(pci_bar_hw_addr, REG_SIZE);
	}

	/*free IRQ number*/
	if(pci_state_flag & HAVE_IRQ){
		free_irq(pci_dev_struct->irq, pci_dev_struct);
	}

	/*disable MSI interrupt*/
	if(msi_int_enabled != 0)
		pci_disable_msi(pci_dev_struct);
	
	/*free memory allocated to our Endpoint*/
	pci_free_consistent(pci_dev_struct, BUF_SIZE, dma_r_descriptor_buf, DMA_R_DSC_ADDR);
	pci_free_consistent(pci_dev_struct, BUF_SIZE, dma_w_descriptor_buf, DMA_W_DSC_ADDR);
	dma_r_descriptor_buf = NULL;
	dma_w_descriptor_buf = NULL;
	pci_free_consistent(pci_dev_struct, BUF_SIZE, dma_r_buf, DMA_R_BUF_ADDR);
	pci_free_consistent(pci_dev_struct, BUF_SIZE, dma_w_buf, DMA_W_BUF_ADDR);
	dma_r_buf = NULL;
	dma_w_buf = NULL;
	pci_free_consistent(pci_dev_struct, BUF_SIZE, zero_copy_buf, ZERO_COPY_BUF_ADDR);
	zero_copy_buf = NULL;

	/*free memory pointed to by virtual address*/
	if(NULL != pci_bar_vir_addr){
		iounmap(pci_bar_vir_addr);
	}
	pci_bar_vir_addr = NULL;
	
	/*unregister device driver*/
	if(pci_state_flag & HAVE_REG_DEV){
		unregister_chrdev(pci_devMay, pci_devName);
	}
	
	pci_state_flag = 0;
}

/********************** interrupt handler **********************/
static irqreturn_t pci_IRQHandler (int irq, void *dev_id)
{
	u32 status_reg = 0;
	status_reg = pci_reg_read(SYS_REG_OFFSET(8));
	if(((status_reg & 0x00000010) != 0) && ((status_reg & 0x00000020) == 0)){
		// DMA read interrupt
		dma_r_int_handler(status_reg);
		return IRQ_HANDLED;
	}
	else if(((status_reg & 0x00000400) != 0) && ((status_reg & 0x00000000800) == 0)){
		// DMA write interrupt
		dma_w_int_handler(status_reg);
		return IRQ_HANDLED;
	}
	else if(((status_reg & 0x00004000) != 0)){
		// user interrupt
		user_int_handler(status_reg);
		return IRQ_HANDLED;
	}
	else{
		// this interrupt will be discarded
		return IRQ_NONE;
	}
}

void dma_r_int_handler(u32 hw_status)
{
	if(hw_status & 0x00000004){
		// dma read done
		atomic_set(&dma_r_status, 1);
	}
	else if(hw_status & 0x00000008){
		// dma read error
		atomic_set(&dma_r_status, 2);
	}
	if(hw_status & 0x00000002){
		// it is a blocking dma read
		atomic_inc(&dma_r_int_occur);
		wake_up_interruptible(&dma_r_wait);
	}
	else{
		// it is an unblocking dma read
		atomic_set(&dma_r_busy, 0);
	}
	// write register to assert dma read interrupt handled
	pci_reg_write(SYS_REG_OFFSET(9), 0x00000003);
}

void dma_w_int_handler(u32 hw_status)
{
	if(hw_status & 0x00000100){
		// dma write done
		atomic_set(&dma_w_status, 1);
	}
	else if(hw_status & 0x00000200){
		// dma write error
		atomic_set(&dma_w_status, 2);
	}
	if(hw_status & 0x00000080){
		// it is a blocking dma write
		atomic_inc(&dma_w_int_occur);
		wake_up_interruptible(&dma_w_wait);
	}
	else{
		// it is an unblocking dma write
		atomic_set(&dma_w_busy, 0);
	}
	// write register to assert dma write interrupt handled
	pci_reg_write(SYS_REG_OFFSET(9), 0x00000005);
}

void user_int_handler(u32 hw_status)
{
	int int_vector = ((hw_status >> 23) & 0x00000007);
	if(hw_status & 0x00004000){
		// user interrupt req is asserted
		atomic_inc(&user_int_occur[int_vector]);
		wake_up_interruptible(&user_wait[int_vector]);
	}
	pci_reg_write(SYS_REG_OFFSET(9), 0x00000009);
}

/********************** file operations **********************/
/*item for file_operations: open*/
int pci_open(struct inode *inode, struct file *filep)
{
	return SUCCESS;
}

/*item for file_operations: release*/
int pci_release(struct inode *inode, struct file *filep)
{
	return SUCCESS;
}

/*item for file_operations: unlocked ioctl*/
long pci_unlocked_ioctl(struct file *filep, unsigned int cmd, unsigned long arg)
{
	struct register_struct * user_reg;
	struct zero_copy_dma_req_struct * dma_req;
	int ret_value;
	int temp_for_int = 1;
	switch(cmd){
		case SYS_PIO_WRITE:
			user_reg = (struct register_struct *)arg;
			pci_reg_write(SYS_REG_OFFSET(user_reg->reg), (u32)user_reg->value);
			break;
		case SYS_PIO_READ:
			user_reg = (struct register_struct *)arg;
			user_reg->value = pci_reg_read(SYS_REG_OFFSET(user_reg->reg));
			break;
		case PCIE_CFG_MODE:
			pci_read_config_dword(pci_dev_struct, 0x6C, &ret_value);
			*((u32*)arg) = ret_value;
			break;
		case PCIE_CUR_MODE:
			pci_read_config_dword(pci_dev_struct, 0x70, &ret_value);
			*((u32*)arg) = ret_value;
			break;
		case USR_PIO_WRITE:
			user_reg = (struct register_struct *)arg;
			pci_reg_write(USR_REG_OFFSET(user_reg->reg), (u32)user_reg->value);
			break;
		case USR_PIO_READ:
			user_reg = (struct register_struct *)arg;
			user_reg->value = pci_reg_read(USR_REG_OFFSET(user_reg->reg));
			break;
		case USR_INT_WAIT:
			// arg is the interrupt vector
			down(&usr_int_sem);
			temp_for_int = pci_reg_read(SYS_REG_OFFSET(10));
			if(((temp_for_int >> arg) & 0x1) || (arg > 7)){
				up(&usr_int_sem);
				return -1; // another process is waiting for the int
			}
			atomic_set(&user_int_occur[arg], 0); // reset the atomic user_int_occur
			pci_reg_write(SYS_REG_OFFSET(10), (0x8 | arg)); // set the reg10[arg] = 1
			up(&usr_int_sem);
			temp_for_int = 1;
			while(temp_for_int){
				wait_event_interruptible_timeout(user_wait[arg], \
					(atomic_read(&user_int_occur[arg]) > 0), (10*HZ/1000));
				if(0 != signal_pending(current)){
					pci_reg_write(SYS_REG_OFFSET(10), (0x0 | arg)); // set the reg10[arg] = 0
					atomic_set(&user_int_occur[arg], 0); // reset the atomic user_int_occur
					break;
				}
				if(atomic_add_negative(-1, &user_int_occur[arg])){
					atomic_inc(&user_int_occur[arg]);
				}
				else{
					temp_for_int = 0;
				}
			}
			break;
		case ZERO_COPY_READ: /*zerocopy DMA write*/
			dma_req = (struct zero_copy_dma_req_struct *)arg;
			#ifdef EPEE_HW_INTERFACE_128
				if(0 != (dma_req->count % 8))
					return -1;
			#endif
			return pci_read_zerocopy(dma_req->offset, dma_req->count);
		case ZERO_COPY_WRITE: /*zerocopy DMA read*/
			dma_req = (struct zero_copy_dma_req_struct *)arg;
			#ifdef EPEE_HW_INTERFACE_128
				if(0 != (dma_req->count % 8))
					return -1;
			#endif
			return pci_write_zerocopy(dma_req->offset, dma_req->count);
		case READ_STATUS: /*DMA write, return 1 when busy, 0 when idle*/
			return atomic_read(&dma_w_busy);
		case WRITE_STATUS: /*DMA read, return 1 when busy, 0 when idle*/
			return atomic_read(&dma_r_busy);
		default:;
	}
	return 0;
}

/*item for file_operations: mmap*/
/*offset value is not used here*/
static int pci_map(struct file * filep, struct vm_area_struct * vma)
{
	unsigned long phy_addr;
	unsigned long pfn;
	
	if(vma->vm_end - vma->vm_start > BUF_SIZE)
		return -1;
	
	phy_addr = __pa(zero_copy_buf);
	pfn = (phy_addr >> PAGE_SHIFT);
	if(remap_pfn_range(vma, vma->vm_start, pfn, vma->vm_end - vma->vm_start, vma->vm_page_prot)){
		printk(KERN_INFO"<pci_map>: mmap error\n");
		return -1;
	}
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 7, 0)
	vma->vm_flags |= (VM_DONTEXPAND | VM_DONTDUMP);
#else
	vma->vm_flags |= VM_RESERVED;
#endif
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	return 0;
}

/*item for file_operations: read*/
/*This is the dma write operation*/
ssize_t pci_read(struct file *filep, char *buf, size_t count, loff_t *f_pos)
{
	#ifdef EPEE_HW_INTERFACE_128
		if(0 != (count % 8))
			return -1;
	#endif
	if(filep->f_flags & O_NONBLOCK){
		// non blocking 
		return -1;
	}
	else{
		// blocking
		return pci_read_blocking(buf, count);
	}
}

/*item for file_operations: write*/
/*This is the dma read operation*/
ssize_t pci_write(struct file *filep, const char *buf, size_t count, loff_t *f_pos)
{
	#ifdef EPEE_HW_INTERFACE_128
		if(0 != (count % 8))
			return -1;
	#endif
	if(filep->f_flags & O_NONBLOCK){
		// non blocking 
		return pci_write_unblocking(buf, count);
	}
	else{
		// blocking
		return pci_write_blocking(buf, count);
	}
}

/******************** read and write operations (called by pci_read/pci_write) *************/
ssize_t pci_read_blocking(char *buf, size_t count)
{ /*dma write*/
	int slice_num; /*number of descirptors*/
	int last_slice_len; /*the last descriptor's require length*/
	int temp_for_int;
	int i = 0;
	u32 status_reg = 0;
	if(count % 4 != 0 || count <= 0)
		return -1;
	down(&dma_w_sem);
	if(0 < atomic_read(&dma_w_busy)){ // busy
		// device busy: dma has started and not finished
		up(&dma_w_sem);
		return -1;
	}
	status_reg = pci_reg_read(SYS_REG_OFFSET(8));
	if(((status_reg & 0x00000040) != 0) && ((status_reg & 0x00000800) == 0)){
		// device busy: dma has started and not finished
		up(&dma_w_sem);
		return -1;
	}
	// set dma_w_busy
	atomic_set(&dma_w_busy, 1);
	// fill DMA write descriptors	
	slice_num = (((count % (4*1024)) == 0) ? (count / (4*1024)) : (count / (4*1024) + 1));
	last_slice_len = (count - (slice_num - 1) * 4 * 1024) / 4;
	for(i = 0; i < slice_num; i++){
		// 1st Descriptor: dma write
		#ifdef OS_64BIT_EN
		dma_w_descriptor_buf[i*16+0] = 0x0c;
		#else
		dma_w_descriptor_buf[i*16+0] = 0x08;
		#endif
		dma_w_descriptor_buf[i*16+1] = 0x00;
		dma_w_descriptor_buf[i*16+2] = ((i == slice_num - 1) ? (last_slice_len >> 8) : 0x04);
		dma_w_descriptor_buf[i*16+3] = ((i == slice_num - 1) ? (last_slice_len >> 0) : 0x00);
	
		#ifdef OS_64BIT_EN
		dma_w_descriptor_buf[i*16+4] = 0xFF & ((DMA_W_BUF_ADDR+i*4096) >> 56);
		dma_w_descriptor_buf[i*16+5] = 0xFF & ((DMA_W_BUF_ADDR+i*4096) >> 48);
		dma_w_descriptor_buf[i*16+6] = 0xFF & ((DMA_W_BUF_ADDR+i*4096) >> 40);
		dma_w_descriptor_buf[i*16+7] = 0xFF & ((DMA_W_BUF_ADDR+i*4096) >> 32);
		#else
		dma_w_descriptor_buf[i*16+4] = 0x00;
		dma_w_descriptor_buf[i*16+5] = 0x00;
		dma_w_descriptor_buf[i*16+6] = 0x00;
		dma_w_descriptor_buf[i*16+7] = 0x00;
		#endif
		
			//DMA_W_BUF_ADDR low 32bit
		dma_w_descriptor_buf[i*16+8] = 0xFF & ((DMA_W_BUF_ADDR+i*4096) >> 24);
		dma_w_descriptor_buf[i*16+9] = 0xFF & ((DMA_W_BUF_ADDR+i*4096) >> 16);
		dma_w_descriptor_buf[i*16+10] = 0xFF & ((DMA_W_BUF_ADDR+i*4096) >> 8);
		dma_w_descriptor_buf[i*16+11] = 0xFF & ((DMA_W_BUF_ADDR+i*4096) >> 0);
			// not used 4 Bytes
		dma_w_descriptor_buf[i*16+12] = 0x00;
		dma_w_descriptor_buf[i*16+13] = 0x00;
		dma_w_descriptor_buf[i*16+14] = 0x00;
		dma_w_descriptor_buf[i*16+15] = 0x00;
	}
	// zero-read descriptor
	#ifdef OS_64BIT_EN
	dma_w_descriptor_buf[i*16+0] = 0x14;
	#else
	dma_w_descriptor_buf[i*16+0] = 0x10;
	#endif
	dma_w_descriptor_buf[i*16+1] = 0x00;
	dma_w_descriptor_buf[i*16+2] = 0x00;
	dma_w_descriptor_buf[i*16+3] = 0x00;
	
	#ifdef OS_64BIT_EN
	dma_w_descriptor_buf[i*16+4] = 0xFF & ((DMA_W_BUF_ADDR+count - 1) >> 56);
	dma_w_descriptor_buf[i*16+5] = 0xFF & ((DMA_W_BUF_ADDR+count - 1) >> 48);
	dma_w_descriptor_buf[i*16+6] = 0xFF & ((DMA_W_BUF_ADDR+count - 1) >> 40);
	dma_w_descriptor_buf[i*16+7] = 0xFF & ((DMA_W_BUF_ADDR+count - 1) >> 32);
	#else
	dma_w_descriptor_buf[i*16+4] = 0x00;
	dma_w_descriptor_buf[i*16+5] = 0x00;
	dma_w_descriptor_buf[i*16+6] = 0x00;
	dma_w_descriptor_buf[i*16+7] = 0x00;
	#endif
	
		//DMA_W_BUF_ADDR low 32bit; ZERO-read
	dma_w_descriptor_buf[i*16+8] = 0xFF & ((DMA_W_BUF_ADDR+count - 1) >> 24);
	dma_w_descriptor_buf[i*16+9] = 0xFF & ((DMA_W_BUF_ADDR+count - 1) >> 16);
	dma_w_descriptor_buf[i*16+10] = 0xFF & ((DMA_W_BUF_ADDR+count - 1) >> 8);
	dma_w_descriptor_buf[i*16+11] = 0xFF & ((DMA_W_BUF_ADDR+count - 1) >> 0);
	
	// reset the atomic dma_w_int_occur
	atomic_set(&dma_w_int_occur, 0);
	// start DMA write
	#ifdef OS_64BIT_EN
	pci_reg_write(SYS_REG_OFFSET(7), 0x00000007 | (0x3FFF0000 & (slice_num+1) << 16));
	#else
	pci_reg_write(SYS_REG_OFFSET(7), 0x00000003 | (0x3FFF0000 & (slice_num+1) << 16));
	#endif
	up(&dma_w_sem);
	
	temp_for_int = 1;
	while(temp_for_int){
		wait_event_interruptible_timeout(dma_w_wait, \
			(atomic_read(&dma_w_int_occur) > 0), (10*HZ/1000));
		if(0 != signal_pending(current)){ // user process killed
			atomic_set(&dma_w_busy, 0);
			return -1;
		}
		if(atomic_add_negative(-1, &dma_w_int_occur)){
			atomic_inc(&dma_w_int_occur);
		}
		else{
			temp_for_int = 0;
		}
	}
	memcpy(buf, dma_w_buf, count);
	atomic_set(&dma_w_busy, 0);
	return count;
}

ssize_t pci_read_zerocopy(int offset, size_t count)
{ /*dma write*/
	int slice_num; /*number of descirptors*/
	int last_slice_len; /*the last descriptor's require length*/
	int i = 0;
	u32 status_reg = 0;
	if(count % 4 != 0 || count <= 0 || (offset + count) > BUF_SIZE)
		return -1;
	down(&dma_w_sem);
	if(0 < atomic_read(&dma_w_busy)){ // busy
		// device busy: dma has started and not finished
		up(&dma_w_sem);
		return -1;
	}
	status_reg = pci_reg_read(SYS_REG_OFFSET(8));
	if(((status_reg & 0x00000040) != 0) && ((status_reg & 0x00000800) == 0)){
		// device busy: dma has started and not finished
		up(&dma_w_sem);
		return -1;
	}
	// set dma_w_busy
	atomic_set(&dma_w_busy, 1);
	// fill DMA write descriptors	
	slice_num = (((count % (4*1024)) == 0) ? (count / (4*1024)) : (count / (4*1024) + 1));
	last_slice_len = (count - (slice_num - 1) * 4 * 1024) / 4;
	for(i = 0; i < slice_num; i++){
		// 1st Descriptor: dma write
		#ifdef OS_64BIT_EN
		dma_w_descriptor_buf[i*16+0] = 0x0c;
		#else
		dma_w_descriptor_buf[i*16+0] = 0x08;
		#endif
		dma_w_descriptor_buf[i*16+1] = 0x00;
		dma_w_descriptor_buf[i*16+2] = ((i == slice_num - 1) ? (last_slice_len >> 8) : 0x04);
		dma_w_descriptor_buf[i*16+3] = ((i == slice_num - 1) ? (last_slice_len >> 0) : 0x00);
	
		#ifdef OS_64BIT_EN
		dma_w_descriptor_buf[i*16+4] = 0xFF & ((ZERO_COPY_BUF_ADDR + offset+i*4096) >> 56);
		dma_w_descriptor_buf[i*16+5] = 0xFF & ((ZERO_COPY_BUF_ADDR + offset+i*4096) >> 48);
		dma_w_descriptor_buf[i*16+6] = 0xFF & ((ZERO_COPY_BUF_ADDR + offset+i*4096) >> 40);
		dma_w_descriptor_buf[i*16+7] = 0xFF & ((ZERO_COPY_BUF_ADDR + offset+i*4096) >> 32);
		#else
		dma_w_descriptor_buf[i*16+4] = 0x00;
		dma_w_descriptor_buf[i*16+5] = 0x00;
		dma_w_descriptor_buf[i*16+6] = 0x00;
		dma_w_descriptor_buf[i*16+7] = 0x00;
		#endif
		
			//dma addr low 32bit
		dma_w_descriptor_buf[i*16+8] = 0xFF & ((ZERO_COPY_BUF_ADDR + offset+i*4096) >> 24);
		dma_w_descriptor_buf[i*16+9] = 0xFF & ((ZERO_COPY_BUF_ADDR + offset+i*4096) >> 16);
		dma_w_descriptor_buf[i*16+10] = 0xFF & ((ZERO_COPY_BUF_ADDR + offset+i*4096) >> 8);
		dma_w_descriptor_buf[i*16+11] = 0xFF & ((ZERO_COPY_BUF_ADDR + offset+i*4096) >> 0);
			// not used 4 Bytes
		dma_w_descriptor_buf[i*16+12] = 0x00;
		dma_w_descriptor_buf[i*16+13] = 0x00;
		dma_w_descriptor_buf[i*16+14] = 0x00;
		dma_w_descriptor_buf[i*16+15] = 0x00;
	}
	// zero-read descriptor
	#ifdef OS_64BIT_EN
	dma_w_descriptor_buf[i*16+0] = 0x14;
	#else
	dma_w_descriptor_buf[i*16+0] = 0x10;
	#endif
	dma_w_descriptor_buf[i*16+1] = 0x00;
	dma_w_descriptor_buf[i*16+2] = 0x00;
	dma_w_descriptor_buf[i*16+3] = 0x00;
	
	#ifdef OS_64BIT_EN
	dma_w_descriptor_buf[i*16+4] = 0xFF & ((ZERO_COPY_BUF_ADDR + offset+count - 1) >> 56);
	dma_w_descriptor_buf[i*16+5] = 0xFF & ((ZERO_COPY_BUF_ADDR + offset+count - 1) >> 48);
	dma_w_descriptor_buf[i*16+6] = 0xFF & ((ZERO_COPY_BUF_ADDR + offset+count - 1) >> 40);
	dma_w_descriptor_buf[i*16+7] = 0xFF & ((ZERO_COPY_BUF_ADDR + offset+count - 1) >> 32);
	#else
	dma_w_descriptor_buf[i*16+4] = 0x00;
	dma_w_descriptor_buf[i*16+5] = 0x00;
	dma_w_descriptor_buf[i*16+6] = 0x00;
	dma_w_descriptor_buf[i*16+7] = 0x00;
	#endif
		//dma addr low 32bit; ZERO-read
	dma_w_descriptor_buf[i*16+8] = 0xFF & ((ZERO_COPY_BUF_ADDR + offset+count - 1) >> 24);
	dma_w_descriptor_buf[i*16+9] = 0xFF & ((ZERO_COPY_BUF_ADDR + offset+count - 1) >> 16);
	dma_w_descriptor_buf[i*16+10] = 0xFF & ((ZERO_COPY_BUF_ADDR + offset+count - 1) >> 8);
	dma_w_descriptor_buf[i*16+11] = 0xFF & ((ZERO_COPY_BUF_ADDR + offset+count - 1) >> 0);

	// start DMA write
	#ifdef OS_64BIT_EN
	pci_reg_write(SYS_REG_OFFSET(7), 0x00000005 | (0x3FFF0000 & (slice_num+1) << 16));
	#else
	pci_reg_write(SYS_REG_OFFSET(7), 0x00000001 | (0x3FFF0000 & (slice_num+1) << 16));
	#endif
	up(&dma_w_sem);
	
	return count;
}

ssize_t pci_write_blocking(const char *buf, size_t count)
{ /*dma read*/
	int slice_num; /*number of descirptors*/
	int last_slice_len; /*the last descriptor's require length*/
	int temp_for_int;
	u32 status_reg = 0;
	int i = 0;
	if(count % 4 != 0 || count <= 0)
		return -1;
	down(&dma_r_sem);
	// check if DMA busy in sw
	if(0 < atomic_read(&dma_r_busy)){ // busy
		// device busy: dma has started and not finished
		up(&dma_r_sem);
		return -1;
	}
	// check if DMA busy in hw
	status_reg = pci_reg_read(SYS_REG_OFFSET(8));
	if(((status_reg & 0x00000001) != 0) && ((status_reg & 0x00000020) == 0)){
		// device busy: dma has started and not finished
		up(&dma_r_sem);
		return -1;
	}
	// set dma_r_busy
	atomic_set(&dma_r_busy, 1);
	memcpy(dma_r_buf, buf, count);
	// fill dma read descriptor
	slice_num = (((count % (4*1024)) == 0) ? (count / (4*1024)) : (count / (4*1024) + 1));
	last_slice_len = (count - (slice_num - 1) * 4 * 1024) / 4;
	for(i = 0; i < slice_num; i++){
	    #ifdef OS_64BIT_EN
		dma_r_descriptor_buf[i*16+0] = ((i == slice_num - 1) ? 0x14 : 0x04);
		#else
		dma_r_descriptor_buf[i*16+0] = ((i == slice_num - 1) ? 0x10 : 0x00);
		#endif
		dma_r_descriptor_buf[i*16+1] = 0x00;
		dma_r_descriptor_buf[i*16+2] = ((i == slice_num - 1) ? (last_slice_len >> 8) : 0x04);
		dma_r_descriptor_buf[i*16+3] = ((i == slice_num - 1) ? (last_slice_len >> 0) : 0x00);
	
		#ifdef OS_64BIT_EN
		dma_r_descriptor_buf[i*16+4] = 0xFF & ((DMA_R_BUF_ADDR + i * 4096) >> 56);
		dma_r_descriptor_buf[i*16+5] = 0xFF & ((DMA_R_BUF_ADDR + i * 4096) >> 48);
		dma_r_descriptor_buf[i*16+6] = 0xFF & ((DMA_R_BUF_ADDR + i * 4096) >> 40);
		dma_r_descriptor_buf[i*16+7] = 0xFF & ((DMA_R_BUF_ADDR + i * 4096) >> 32);
		#else
		dma_r_descriptor_buf[i*16+4] = 0x00;
		dma_r_descriptor_buf[i*16+5] = 0x00;
		dma_r_descriptor_buf[i*16+6] = 0x00;
		dma_r_descriptor_buf[i*16+7] = 0x00;
		#endif
	
		dma_r_descriptor_buf[i*16+8] = 0xFF & ((DMA_R_BUF_ADDR + i * 4096) >> 24);
		dma_r_descriptor_buf[i*16+9] = 0xFF & ((DMA_R_BUF_ADDR + i * 4096) >> 16);
		dma_r_descriptor_buf[i*16+10] = 0xFF & ((DMA_R_BUF_ADDR + i * 4096) >> 8);
		dma_r_descriptor_buf[i*16+11] = 0xFF & ((DMA_R_BUF_ADDR + i * 4096) >> 0);
	
		dma_r_descriptor_buf[i*16+12] = 0x00;
		dma_r_descriptor_buf[i*16+13] = 0x00;
		dma_r_descriptor_buf[i*16+14] = 0x00;
		dma_r_descriptor_buf[i*16+15] = 0x00;
	}
	// reset the atomic dma_r_int_occur
	atomic_set(&dma_r_int_occur, 0);
	// start dma read
	#ifdef OS_64BIT_EN
	pci_reg_write(SYS_REG_OFFSET(6), 0x00000007 | (0x3FFF0000 & slice_num << 16));
	#else
	pci_reg_write(SYS_REG_OFFSET(6), 0x00000003 | (0x3FFF0000 & slice_num << 16));
	#endif
	up(&dma_r_sem);
	
	temp_for_int = 1;
	while(temp_for_int){
		wait_event_interruptible_timeout(dma_r_wait, \
			(atomic_read(&dma_r_int_occur) > 0), (10*HZ/1000));
		if(0 != signal_pending(current)){ // user process killed
			atomic_set(&dma_r_busy, 0);
			return -1;
		}
		if(atomic_add_negative(-1, &dma_r_int_occur)){
			atomic_inc(&dma_r_int_occur);
		}
		else{
			temp_for_int = 0;
		}
	}
	atomic_set(&dma_r_busy, 0);
	return count;
}

ssize_t pci_write_unblocking(const char *buf, size_t count)
{ /*dma read*/
	// chaining not supported currently
	int slice_num; /*number of descirptors*/
	int last_slice_len; /*the last descriptor's require length*/
	u32 status_reg = 0;
	int i = 0;
	if(count % 4 != 0)
		return -1;
	down(&dma_r_sem);
	// check if DMA busy in sw
	if(0 < atomic_read(&dma_r_busy)){ // busy
		// device busy: dma has started and not finished
		up(&dma_r_sem);
		return -1;
	}
	// check if DMA busy in hw
	status_reg = pci_reg_read(SYS_REG_OFFSET(8));
	if(((status_reg & 0x00000001) != 0) && ((status_reg & 0x00000020) == 0)){
		// device busy: dma has started and not finished
		up(&dma_r_sem);
		return -1;
	}
	if(count <= 0){
		// count invalid
		up(&dma_r_sem);
		return -1;
	}
	// set dma_r_busy
	atomic_set(&dma_r_busy, 1);
	memcpy(dma_r_buf, buf, count);
	// fill dma read descriptor
	slice_num = (((count % (4*1024)) == 0) ? (count / (4*1024)) : (count / (4*1024) + 1));
	last_slice_len = (count - (slice_num - 1) * 4 * 1024) / 4;
	for(i = 0; i < slice_num; i++){
	    #ifdef OS_64BIT_EN
		dma_r_descriptor_buf[i*16+0] = ((i == slice_num - 1) ? 0x14 : 0x04);
		#else
		dma_r_descriptor_buf[i*16+0] = ((i == slice_num - 1) ? 0x10 : 0x00);
		#endif
		dma_r_descriptor_buf[i*16+1] = 0x00;
		dma_r_descriptor_buf[i*16+2] = ((i == slice_num - 1) ? (last_slice_len >> 8) : 0x04);
		dma_r_descriptor_buf[i*16+3] = ((i == slice_num - 1) ? (last_slice_len >> 0) : 0x00);
	
		#ifdef OS_64BIT_EN
		dma_r_descriptor_buf[i*16+4] = 0xFF & ((DMA_R_BUF_ADDR + i * 4096) >> 56);
		dma_r_descriptor_buf[i*16+5] = 0xFF & ((DMA_R_BUF_ADDR + i * 4096) >> 48);
		dma_r_descriptor_buf[i*16+6] = 0xFF & ((DMA_R_BUF_ADDR + i * 4096) >> 40);
		dma_r_descriptor_buf[i*16+7] = 0xFF & ((DMA_R_BUF_ADDR + i * 4096) >> 32);
		#else
		dma_r_descriptor_buf[i*16+4] = 0x00;
		dma_r_descriptor_buf[i*16+5] = 0x00;
		dma_r_descriptor_buf[i*16+6] = 0x00;
		dma_r_descriptor_buf[i*16+7] = 0x00;
		#endif
	
		dma_r_descriptor_buf[i*16+8] = 0xFF & ((DMA_R_BUF_ADDR + i * 4096) >> 24);
		dma_r_descriptor_buf[i*16+9] = 0xFF & ((DMA_R_BUF_ADDR + i * 4096) >> 16);
		dma_r_descriptor_buf[i*16+10] = 0xFF & ((DMA_R_BUF_ADDR + i * 4096) >> 8);
		dma_r_descriptor_buf[i*16+11] = 0xFF & ((DMA_R_BUF_ADDR + i * 4096) >> 0);
	
		dma_r_descriptor_buf[i*16+12] = 0x00;
		dma_r_descriptor_buf[i*16+13] = 0x00;
		dma_r_descriptor_buf[i*16+14] = 0x00;
		dma_r_descriptor_buf[i*16+15] = 0x00;
	}
	// start dma read
	#ifdef OS_64BIT_EN
	pci_reg_write(SYS_REG_OFFSET(6), 0x00000005 | (0x3FFF0000 & slice_num << 16));
	#else
	pci_reg_write(SYS_REG_OFFSET(6), 0x00000001 | (0x3FFF0000 & slice_num << 16));
	#endif
	up(&dma_r_sem);

	return count;
}

ssize_t pci_write_zerocopy(int offset, size_t count)
{ /*dma read*/
	// dma read in memcpy mode is not supported
	int slice_num; /*number of descirptors*/
	int last_slice_len; /*the last descriptor's require length*/
	u32 status_reg = 0;
	int i = 0;
	if(count % 4 != 0 || count <= 0 || (offset + count) > BUF_SIZE)
		return -1;
	down(&dma_r_sem);
	// check if DMA busy in sw
	if(0 < atomic_read(&dma_r_busy)){ // busy
		// device busy: dma has started and not finished
		up(&dma_r_sem);
		return -1;
	}
	// check if DMA busy in hw
	status_reg = pci_reg_read(SYS_REG_OFFSET(8));
	if(((status_reg & 0x00000001) != 0) && ((status_reg & 0x00000020) == 0)){
		// device busy: dma has started and not finished
		up(&dma_r_sem);
		return -1;
	}
	// set dma_r_busy
	atomic_set(&dma_r_busy, 1);
	// fill dma read descriptor
	slice_num = (((count % (4*1024)) == 0) ? (count / (4*1024)) : (count / (4*1024) + 1));
	last_slice_len = (count - (slice_num - 1) * 4 * 1024) / 4;
	for(i = 0; i < slice_num; i++){
	    #ifdef OS_64BIT_EN
		dma_r_descriptor_buf[i*16+0] = ((i == slice_num - 1) ? 0x14 : 0x04);
		#else
		dma_r_descriptor_buf[i*16+0] = ((i == slice_num - 1) ? 0x10 : 0x00);
		#endif
		dma_r_descriptor_buf[i*16+1] = 0x00;
		dma_r_descriptor_buf[i*16+2] = ((i == slice_num - 1) ? (last_slice_len >> 8) : 0x04);
		dma_r_descriptor_buf[i*16+3] = ((i == slice_num - 1) ? (last_slice_len >> 0) : 0x00);
	
		#ifdef OS_64BIT_EN
		dma_r_descriptor_buf[i*16+4] = 0xFF & ((ZERO_COPY_BUF_ADDR + offset + i * 4096) >> 56);
		dma_r_descriptor_buf[i*16+5] = 0xFF & ((ZERO_COPY_BUF_ADDR + offset + i * 4096) >> 48);
		dma_r_descriptor_buf[i*16+6] = 0xFF & ((ZERO_COPY_BUF_ADDR + offset + i * 4096) >> 40);
		dma_r_descriptor_buf[i*16+7] = 0xFF & ((ZERO_COPY_BUF_ADDR + offset + i * 4096) >> 32);
		#else
		dma_r_descriptor_buf[i*16+4] = 0x00;
		dma_r_descriptor_buf[i*16+5] = 0x00;
		dma_r_descriptor_buf[i*16+6] = 0x00;
		dma_r_descriptor_buf[i*16+7] = 0x00;
		#endif
	
		dma_r_descriptor_buf[i*16+8] = 0xFF & ((ZERO_COPY_BUF_ADDR + offset + i * 4096) >> 24);
		dma_r_descriptor_buf[i*16+9] = 0xFF & ((ZERO_COPY_BUF_ADDR + offset + i * 4096) >> 16);
		dma_r_descriptor_buf[i*16+10] = 0xFF & ((ZERO_COPY_BUF_ADDR + offset + i * 4096) >> 8);
		dma_r_descriptor_buf[i*16+11] = 0xFF & ((ZERO_COPY_BUF_ADDR + offset + i * 4096) >> 0);
	
		dma_r_descriptor_buf[i*16+12] = 0x00;
		dma_r_descriptor_buf[i*16+13] = 0x00;
		dma_r_descriptor_buf[i*16+14] = 0x00;
		dma_r_descriptor_buf[i*16+15] = 0x00;
	}
	// start dma read
	#ifdef OS_64BIT_EN
	pci_reg_write(SYS_REG_OFFSET(6), 0x00000005 | (0x3FFF0000 & slice_num << 16));
	#else
	pci_reg_write(SYS_REG_OFFSET(6), 0x00000001 | (0x3FFF0000 & slice_num << 16));
	#endif
	up(&dma_r_sem);

	return count;
}

/******************** register read & write operations *******************/
u32 pci_reg_read(int offset)
{
	u32 data;
	u32 formatted_data;
	if(offset < 0 || offset >= REG_SIZE){
		printk(KERN_INFO"<pci_reg_read>: invalid PIO address\n");
		return -1;
	}
	data = readl(pci_bar_vir_addr + offset);
	formatted_data = ((data << 24) & 0xFF000000) | ((data << 8) & 0x00FF0000) | \
					 ((data >> 8) & 0x0000FF00) | ((data >> 24) & 0x000000FF);
	return formatted_data;
}

void pci_reg_write(int offset, u32 data)
{
	u32 formatted_data;
	if(offset < 0 || offset >= REG_SIZE){
		printk(KERN_INFO"<pci_reg_write>: invalid PIO address\n");
		return;
	}
	formatted_data = ((data << 24) & 0xFF000000) | ((data << 8) & 0x00FF0000) | \
					 ((data >> 8) & 0x0000FF00) | ((data >> 24) & 0x000000FF);
	writel(formatted_data, pci_bar_vir_addr + offset);
}


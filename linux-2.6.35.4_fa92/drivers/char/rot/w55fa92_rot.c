/* w55fa92_rot.c
 *
 * Copyright (c) 2014 Nuvoton technology corporation
 * All rights reserved.
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/cdev.h>
#include <linux/dma-mapping.h>
#include <linux/pagemap.h>
#include <linux/interrupt.h>

#include <linux/slab.h>

#include <linux/poll.h>
#include <linux/sched.h>
#include <asm/errno.h>
#include <asm/cacheflush.h>

#include <asm/mach/map.h>
#include <mach/w55fa92_reg.h>
#include <linux/bootmem.h>
 
#include "rot.h"
#include <mach/w55fa92_rot.h>



#define DRIVER_NAME "w55fa92-rot"

#define ROT_DEVICE_FILE_NAME	"rot"


static int major_no = ROT_MAJOR;
module_param (major_no, int, 0644);
static int minor_no = ROT_MINOR;
module_param (minor_no, int, 0644);
static int dev_no_created = 0;
static struct class *rot_class = NULL;
static struct device *rot_device = NULL;

static struct cdev rot_dev;
static int rot_dev_created = 0;
/* Lock ROT */
static struct semaphore rot_lock;

/* Wait Queue */
//volatile BOOL bIsROTComplete = TRUE;
static DECLARE_WAIT_QUEUE_HEAD(rot_wq);

static int irq_requested = 0;
// Single-open policy.
static atomic_t rot_avail = ATOMIC_INIT (1);

extern void rotIntHandler(void);
static irqreturn_t rot_irq_handler(int irq, void *dev_id, struct pt_regs *r)
{
	S_ROT_PRIV *priv = (S_ROT_PRIV *)dev_id;
	
	wake_up_interruptible(&rot_wq);
	//DBG_PRINTF("rot_irq_handler\n"); 
    	   
    rotIntHandler();
	priv->bIsROTComplete = TRUE;
	return IRQ_HANDLED;
}

extern unsigned int w55fa92_rot_v;	
extern unsigned int w55fa92_rot_p;
static int w55fa92_rot_open(struct inode *inode, struct file *file)
{
	int ret = 0;
	S_ROT_PRIV *priv;
	
	if (! atomic_dec_and_test (&rot_avail)) {
		atomic_inc (&rot_avail);
		ERR_PRINTF("ROT driver Busy\n");
		return -EBUSY;
	}
	DBG_PRINTF("ROT driver is free\n");
	if( down_trylock(&rot_lock) ){
		ERR_PRINTF("ROT has been opened\n");
		return -EBUSY;
	}

	priv = (S_ROT_PRIV*)kmalloc(sizeof(S_ROT_PRIV), GFP_KERNEL);
	file->private_data = priv;
	
	ret = request_irq(IRQ_ROT, (irq_handler_t)rot_irq_handler, IRQF_DISABLED | IRQF_IRQPOLL, DRIVER_NAME, priv);
	if (ret) {
		ERR_PRINTF("cannot get irq %d - err %d\n", IRQ_ROT, ret);
		ret = -EBUSY;
	}
	else
 		irq_requested = 1;

	priv->u32MemVirAddr = w55fa92_rot_v;
	priv->u32MemPhyAddr	= w55fa92_rot_p;
	
	rotOpen();
	return ret;
}

static int w55fa92_rot_close(struct inode *inode, struct file *file)
{
	S_ROT_PRIV *priv = file->private_data;
	
	DBG_PRINTF("ROT close\n");	
	
	// Disable IRQ
	if (irq_requested) {
		free_irq(IRQ_ROT, priv);
		irq_requested = 0;
	}	
	rotClose();
	priv->state = ROT_CLOSE;
	
	up(&rot_lock);	

	if (priv) {
		kfree(priv);
		file->private_data = priv = NULL;
	}

	atomic_inc (&rot_avail);
	
	return 0;
}

static int w55fa92_rot_mmap(struct file *file, struct vm_area_struct *vma)
{
#if 1
	unsigned long size  = vma->vm_end-vma->vm_start;
	unsigned long page, pos;
	unsigned long start = vma->vm_start;
	S_ROT_PRIV *priv = file->private_data;

	pos = priv->u32MemVirAddr;

	while (size > 0)
	{
		page = vmalloc_to_pfn((void *)pos);
		if (remap_pfn_range(vma, start, page, PAGE_SIZE, PAGE_SHARED))
		{
			ERR_PRINTF("remap error\n");
			return -EAGAIN;
		}
		start += PAGE_SIZE;
		pos += PAGE_SIZE;
		if (size > PAGE_SIZE)
			size -= PAGE_SIZE;
		else
			size = 0;
	}
	return 0;
#else
	unsigned long size  = vma->vm_end-vma->vm_start;
	unsigned long pos;
	S_ROT_PRIV *priv = file->private_data;

	pos = priv->u32MemPhyAddr; 
	vma->vm_flags |= (VM_IO | VM_RESERVED);
	vma->vm_page_prot = pgprot_noncached (vma->vm_page_prot);

	if (remap_pfn_range (vma, vma->vm_start, (unsigned long)pos >> PAGE_SHIFT, size, vma->vm_page_prot) < 0) {
		DBG_PRINTF("rot mmap fail\n");
       		return -EIO;
    }
	return 0;
#endif
}

static S_ROT sRotConf;
static S_ROT_BUF sRotBuf;
static int w55fa92_rot_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	S_ROT_PRIV *priv = file->private_data;
	int ret=0;	
	switch (cmd) {	
	case ROT_BUF_INFO:
		DBG_PRINTF("ROT Get Buf Info\n");		
		sRotBuf.u32Width = CONFIG_ROTATION_WIDTH;
		sRotBuf.u32Height = CONFIG_ROTATION_HEIGHT;
		sRotBuf.u32PixelWidth = CONFIG_ROTATION_PIXEL_WIDTH;
		sRotBuf.u32PhysicalAddr = priv->u32MemPhyAddr;
		if (copy_to_user((void *)arg, (S_ROT_BUF*)&sRotBuf, sizeof(S_ROT_BUF))) {
			ERR_PRINTF("copy_to_user error ROT_IO_GET\n");
			ret = -EFAULT;
		}	
		break;
	case ROT_IO_GET:				
		DBG_PRINTF("ROT Get Configuration\n");		
		if (copy_to_user((void *)arg, (S_ROT*)&sRotConf, sizeof(S_ROT))) {
			ERR_PRINTF("copy_to_user error ROT_IO_GET\n");
			ret = -EFAULT;
		}	
		break;
	case ROT_IO_SET:
		DBG_PRINTF("ROT Set Configuration\n");			
		if (copy_from_user((S_ROT*)&sRotConf, (void *)arg, sizeof(S_ROT))) {	
			ERR_PRINTF("copy_from_user error ROT_IO_SET\n");	
			ret = -EFAULT;		
		}

		DBG_PRINTF("sRotConf.eRotFormat = %x\n", sRotConf.eRotFormat);		// Assigned the rotation format RGB888/RGB565/YUV422
		DBG_PRINTF("sRotConf.eBufSize =  %x\n", sRotConf.eBufSize);		// Assigned the buffer size for on the fly rotation
		DBG_PRINTF("sRotConf.eRotDir = %x\n", sRotConf.eRotDir);			// Left/Right

		DBG_PRINTF("sRotConf.u32RotDimHW = %x\n", sRotConf.u32RotDimHW);				// Rotation Dimension  [31:16]==> Height of source pattern, [15:0]==> Width of source pattern 
		DBG_PRINTF("sRotConf.u32SrcLineOffset = %x\n", sRotConf.u32SrcLineOffset);			// Source line offset, pixel unit
		DBG_PRINTF("sRotConf.u32DstLineOffset = %x\n", sRotConf.u32DstLineOffset);			// Destination line offset, pixel unit
		DBG_PRINTF("sRotConf.u32SrcAddr = %x\n", sRotConf.u32SrcAddr);				// Buffer physical start address of source image
		DBG_PRINTF(" sRotConf.u32DstAddr = %x\n", sRotConf.u32DstAddr);				// Buffer physical start address of rotated image
		rotImageConfig(&sRotConf);
			
		break;			
	case ROT_IO_TRIGGER:
		DBG_PRINTF("ROT Trigger\n");
		flush_cache_all();
		if( rotTrigger() == Successful )
			priv->bIsROTComplete = FALSE;		
		else
			ERR_PRINTF("ROT engine is busy\n");	
		break;
	case ROT_POLLING_INTERRUPT:
		DBG_PRINTF("ROT Polling Compplete\n");		
		if (copy_to_user((void *)arg, (void *)((unsigned int)&priv->bIsROTComplete), sizeof(unsigned int)))
		{		
			ret = -EFAULT;
		}
		break;
	}
	return ret;
}

static void rot_cleanup (void)
{
	dev_t dev = MKDEV(major_no, minor_no);

	/* Auto create device file */
	if (rot_device) {
		device_destroy(rot_class, dev);
		rot_device = NULL;
	}
	if (rot_class) {
		class_destroy(rot_class);
		rot_class = NULL;
	}
	/* Auto create device file */

	if (rot_dev_created)
	{
		cdev_del(&rot_dev);
		rot_dev_created = 0;
	}		

	if (dev_no_created) {
		unregister_chrdev_region(dev, 1);
		dev_no_created = 0;
	}
}

static struct file_operations w55fa92_rot_fops = {
	.owner		= THIS_MODULE,
	.open		= w55fa92_rot_open,
	.release	= w55fa92_rot_close,
	.mmap		= w55fa92_rot_mmap,
	.ioctl		= w55fa92_rot_ioctl,
};

static int __init w55fa92_rot_init(void)
{
	int err = 0;
	unsigned int u32PhyAddr;
	dev_t dev;
	do
	{
		err = alloc_chrdev_region(&dev, minor_no, 1, DRIVER_NAME);
		if (err) {
			ERR_PRINTF(KERN_ERR "[ROT Driver] alloc_chrdev_region failed\n");
			break;
		}
		major_no = MAJOR(dev);

		dev_no_created = 1;
		DBG_PRINTF("fa92 rot register char device Successful!\n");	

		cdev_init(&rot_dev, &w55fa92_rot_fops);
		rot_dev.owner = THIS_MODULE;
		rot_dev.ops = &w55fa92_rot_fops;

		err = cdev_add(&rot_dev, dev, 1);
		if (err) {
			ERR_PRINTF(KERN_NOTICE "Error adding w55fa92 ROT char device!\n");
			break;
		}		
		rot_dev_created	= 1;

		rot_class = class_create(THIS_MODULE, DRIVER_NAME);
		if(IS_ERR(rot_class)) {
			ERR_PRINTF(KERN_ERR "[ROT Driver] class_create failed\n");
			err = PTR_ERR(rot_class);
			rot_class = NULL;
			break;
		}
		rot_device = device_create(rot_class, NULL, MKDEV(major_no, minor_no), NULL, ROT_DEVICE_FILE_NAME);
		if (IS_ERR(rot_device)) {
			ERR_PRINTF(KERN_ERR "[ROT Driver] device_create failed\n");
			err = PTR_ERR(rot_device);
			rot_device = NULL;
			break;
		}

		u32PhyAddr = dmamalloc_phy(CONFIG_ROTATION_WIDTH*CONFIG_ROTATION_HEIGHT*CONFIG_ROTATION_PIXEL_WIDTH);

		/* initialize locks */
		init_MUTEX(&rot_lock);
		/* init waitQueue */
		init_waitqueue_head(&rot_wq);
		printk("w55fa92 ROT driver has been initialized successfully! Phy addr = 0x%x\n",u32PhyAddr);
	}while(0);

	if (err) {
		rot_cleanup ();
	}else{
		
	}	

	return err;
}

static void __exit w55fa92_rot_cleanup(void)
{
	rot_cleanup();
}

//console_initcall (w55fa92_rot_alloc_mem);
module_init(w55fa92_rot_init);
module_exit(w55fa92_rot_cleanup);

MODULE_DESCRIPTION("HW Video Decoder driver for W55fa92");
MODULE_LICENSE("GPL");


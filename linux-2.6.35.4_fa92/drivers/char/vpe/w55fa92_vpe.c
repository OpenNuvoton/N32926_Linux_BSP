/* w55fa92_vpe.c
 *
 * Copyright (c) 2009 Nuvoton technology corporation
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
 
#include "vpe.h"
#include <mach/w55fa92_vpe.h>

#include <linux/syscalls.h>

#define DBG_PRINTF(...)
//#define DBG_PRINTF	printk

#define DRIVER_NAME "w55fa92-vpe"

#define VPE_DYNAMIC_ALLOC_DEVICE_NUM
#define VPE_AUTOMATIC_CREATE_DEVICE_FILE

#ifdef VPE_AUTOMATIC_CREATE_DEVICE_FILE
#define VPE_DEVICE_FILE_NAME	"vpe"
#endif

static int major_no = VPE_MAJOR;
module_param (major_no, int, 0644);
static int minor_no = VPE_MINOR;
module_param (minor_no, int, 0644);
static int dev_no_created = 0;

#ifdef VPE_AUTOMATIC_CREATE_DEVICE_FILE
static struct class *vpe_class = NULL;
static struct device *vpe_device = NULL;
#endif	// #ifdef VPE_AUTOMATIC_CREATE_DEVICE_FILE
static struct cdev vpe_dev;
static int vpe_dev_created = 0;
/* Lock VPE */
static struct semaphore vpe_lock;

/* Lock VPE Trigger */
///static struct semaphore vpe_lock_int;

/* Wait Queue */
volatile BOOL bIsVPEComplete = 1;
static DECLARE_WAIT_QUEUE_HEAD(vpe_wq);

/* To avoid the MMU table was reserved by kernel kill application */
__u32 vpe_stop = 0;

static int irq_requested = 0;
// Single-open policy.
static atomic_t vpe_avail = ATOMIC_INIT (1);

#define	VPE_MEM_SIZE		4*1024 		// use it if VPE need to allocate memory from system

///static unsigned long _vpe_mem_start_addr = -1;
static unsigned int _MMU_mode=1;

#define vpe_get_pgd() 							\
	({										\
		unsigned long pg;					\
		__asm__("mrc		p15, 0, %0, c2, c0, 0"	\
				:	"=r" (pg) : : "cc");			\
		pg &= ~0x3fff;						\
	})

extern __u32 vpeIntHandler(void);
static irqreturn_t vpe_irq_handler(int irq, void *dev_id, struct pt_regs *r)
{
	vpe_priv_t *priv = (vpe_priv_t *)dev_id;
	__u32 ret;

    	DBG_PRINTF("vpe_irq_handler\n");    
    	ret = vpeIntHandler();
	if(ret==1){ 	
		wake_up_interruptible(&vpe_wq);	
		bIsVPEComplete=1;
		priv->state = VPE_FINISH;
	}else if(ret==2){
		
	}
	
	return IRQ_HANDLED;
}


static int w55fa92_vpe_open(struct inode *inode, struct file *file)
{
	int ret = 0;
///	int i; 
///	int pgd;
	pgd_t *pgd;
///	int* ppgd;
	vpe_priv_t *priv;
	
	if (! atomic_dec_and_test (&vpe_avail)) {
		atomic_inc (&vpe_avail);
		printk("VPE driver Busy\n");
		return -EBUSY;
	}
	DBG_PRINTF("VPE driver free\n");

	//printk("vpe_open\n");	
	if( down_trylock(&vpe_lock) ){
		printk("VPE has been opened\n");
		return -EBUSY;
	}

	priv = (vpe_priv_t *)kmalloc(sizeof(vpe_priv_t), GFP_KERNEL);

	file->private_data = priv;
	
	DBG_PRINTF("\ncurrent->mm = 0x%x\n", current->mm);

	vpeOpen();
	vpeEnableInt(VPE_INT_COMP);				
	vpeEnableInt(VPE_INT_PAGE_FAULT);	
	vpeEnableInt(VPE_INT_PAGE_MISS);
	vpeEnableInt(VPE_INT_DMA_ERR);	
	pgd = cpu_get_pgd();
	vpeIoctl(VPE_IOCTL_SET_MMU_ENTRY,
				TRUE,					// MMU Enable 
				(__u32)pgd,				// TLB Entry
				0x0);
	//DrvBLT_EnableInt();
///	ret = request_irq(IRQ_GVE_VPE, vpe_irq_handler, SA_INTERRUPT, DRIVER_NAME, priv);
	ret = request_irq(IRQ_VPE, (irq_handler_t)vpe_irq_handler, IRQF_DISABLED | IRQF_IRQPOLL, DRIVER_NAME, priv);
	if (ret) {
		printk("cannot get irq %d - err %d\n", IRQ_VPE, ret);
		ret = -EBUSY;
	//	DrvBLT_DisableInt();
	//	DrvBLT_Close();
	}
	else
 		irq_requested = 1;
	vpe_stop = 0;	
	return ret;
}
static unsigned int s_isPrtDbgMsg;
static int w55fa92_vpe_close(struct inode *inode, struct file *file)
{
	vpe_priv_t *priv = file->private_data;
	
	if ( s_isPrtDbgMsg )
	printk("vpe_close\n");	
	
	priv->state = VPE_CLOSE;
	while( (((__raw_readl(REG_VPE_TG)) & VPE_GO) == VPE_GO) );
	while( bIsVPEComplete == 0);					/* To avoid go bit fast than interrupt */
	
	// Disable IRQ
	if (irq_requested) {
		free_irq(IRQ_VPE, priv);
		irq_requested = 0;
	}
	vpeClose();
	up(&vpe_lock);	

	if (priv) {
		kfree(priv);
		file->private_data = priv = NULL;
	}

	atomic_inc (&vpe_avail);
	
	return 0;
}

/**
  * @brief  Lock in user pages.
  *
  * @note   Through sys_mlock() (equivalent to mlock() in user space), fix unexpected page table modification by Linux kernel 
  *         due to its emulation of 'access' or 'young' flag to support swap (CONFIG_SWAP) even though swap is disabled.
  * @note   sys_mlock() just locks pages in memory, but doesn't pin them. If page migration (CONFIG_MIGRATION) is enabled, 
  *         there will be another issue for MMU-enabled IP. Future kernel support to pin pages (mm_mpin)?
  * @note   According to test, sys_mlock() just locks pages that have paged in, but doesn't lock in pages that have paged out.
  *         So manually tap user address to fault in all these pages. Order is significant here: sys_mlock() first, and then 
  *         manually tap user address.
  */
static int vpe_mlock(unsigned long start, size_t len)
{
    int err = 0;
    
    if (! len) {
        return err;
    }
    
    // Lock user pages in memory by sys_mlock().
    err = sys_mlock(start, len);
    if (err) {
        printk("mlock(0x%08lx, 0x%08x) failed\n", start, len);
        return err;
    }
    
    // Manually fault in all user pages by tapping user address.
    {
        uint8_t tap;
        unsigned long va_start = start;
        unsigned long va_end = start + len;
        unsigned long va_start_pgaln = va_start & PAGE_MASK;
		unsigned long va_end_pgaln = (va_end + PAGE_SIZE - 1)  & PAGE_MASK;
        unsigned long va_ind;
        
        err = get_user(tap, (uint8_t *) va_start);
        if (err) {
            printk("get_user(0x%08lx) failed\n", va_start);
            return err;
        }
        
        
        err = get_user(tap, (uint8_t *) (va_end - 1));
        if (err) {
            printk("get_user(0x%08lx) failed\n", va_end - 1);
            return err;
        }
        
        for (va_ind = va_start_pgaln; va_ind < va_end_pgaln; va_ind += PAGE_SIZE) {
            err = get_user(tap, (uint8_t *) va_ind);
            if (err) {
                printk("get_user(0x%08lx) failed\n", va_ind);
                return err;
            }
        }
    }
    
    return err;
}

static vpe_transform_t workbuf;
///static int w55fa92_vpe_ioctl(struct inode *inode, struct file *file, unsigned int cmd, void *arg)
static int w55fa92_vpe_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	vpe_priv_t *priv = file->private_data;
	int ret=0;
	unsigned int value;
	int err = 0;
	// FIXME
	//vpe_transform_t workbuf;

	switch (cmd) {
	case VPE_INIT:
		DBG_PRINTF("VPE_INIT\n");	
		break;	
	case VPE_IO_SET:
		DBG_PRINTF("VPE_IO_SET\n");		
		if (copy_from_user((unsigned int*)&value, (void *)arg, sizeof(unsigned int))) {

			ret = -EFAULT;
			break;
		}		
		s_isPrtDbgMsg = value;
		DBG_PRINTF("VPE_IO_SET set value = 0x%x\n",value);		
		break;				
	case VPE_IO_GET:						
		if (copy_to_user((void *)arg, (vpe_transform_t *)&workbuf, sizeof(vpe_transform_t))) {
			ret = -EFAULT;
			break;
		}	
		
		break;			
	case VPE_GET_MMU_MODE:
		DBG_PRINTF("VPE_GET_MMU_MODE\n");				
		if (copy_to_user((void *)arg, (unsigned int *)&_MMU_mode, sizeof(unsigned int))) {
			ret = -EFAULT;
			break;
		}			
		DBG_PRINTF("Get MMU mode = %d\n", _MMU_mode);			
		break;	
	case VPE_SET_MMU_MODE:
		DBG_PRINTF("VPE_SET_MMU_MODE\n");				
		if (copy_from_user((unsigned int *)&_MMU_mode, (void *)arg, sizeof(unsigned int))) {
			ret = -EFAULT;
			break;
		}				
		DBG_PRINTF("Set MMU mode = %d\n", _MMU_mode);
		break;
	case VPE_SET_FORMAT_TRANSFORM:
		DBG_PRINTF("VPE_SET_FORMAT_TRANSFORM\n");	
		
		if (copy_from_user((vpe_transform_t *)&workbuf, (void *)arg, sizeof(vpe_transform_t))) {		
			ret = -EFAULT;
			break;
		}

		DBG_PRINTF("VPE_SET_FORMAT_TRANSFORM \n");			
		DBG_PRINTF("Src PacY addr = 0x%x, U addr = 0x%x, V addr = 0x%x\n", workbuf.src_addrPacY,  workbuf.src_addrU,  workbuf.src_addrV);
		vpeIoctl(VPE_IOCTL_SET_SRCBUF_ADDR,
							(UINT32)workbuf.src_addrPacY,				// MMU on, the is virtual address, MMU off, the is physical address. 
							(UINT32)workbuf.src_addrU,	
							(UINT32)workbuf.src_addrV);
		
		DBG_PRINTF("Src format = 0x%x\n", workbuf.src_format);
		DBG_PRINTF("Dst format = 0x%x\n", workbuf.dest_format);	
		vpeIoctl(VPE_IOCTL_SET_FMT,
							workbuf.src_format,		/* Src Format */
							workbuf.dest_format,		/* Dst Format */
							(UINT32)NULL);					/* Useless */
	
		DBG_PRINTF("src_width = %d, src_height=%d\n", workbuf.src_width, workbuf.src_height);
		vpeIoctl(VPE_IOCTL_SET_SRC_DIMENSION,						
							workbuf.src_width,
							workbuf.src_height,
							(UINT32)NULL);					/* Useless */

		DBG_PRINTF("L offset = %d, R offset = %d\n",workbuf.src_leftoffset, workbuf.src_rightoffset);	
		vpeIoctl(VPE_IOCTL_SET_SRC_OFFSET,		
							(UINT32)workbuf.src_leftoffset,	/* Src Left offset */
							(UINT32)workbuf.src_rightoffset,	/* Src right offset */
							(UINT32)NULL);
		DBG_PRINTF("Dest addr = 0x%x, format = %d. width = %d, hight = %d\n",workbuf.dest_addrPac, workbuf.dest_format, workbuf.dest_width, workbuf.dest_height);
		vpeIoctl(VPE_IOCTL_SET_DSTBUF_ADDR,
							(UINT32)workbuf.dest_addrPac,				// MMU on, the is virtual address, MMU off, the is physical address. 
							(UINT32)NULL,	
							(UINT32)NULL);				
		vpeIoctl(VPE_IOCTL_SET_DST_DIMENSION,	
							workbuf.dest_width,
							workbuf.dest_height,
							(UINT32)NULL);	

		DBG_PRINTF("Dst L offset = %d, Dst R offset = %d\n",workbuf.dest_leftoffset, workbuf.dest_rightoffset);
		vpeIoctl(VPE_IOCTL_SET_DST_OFFSET,		
							(UINT32)workbuf.dest_leftoffset,	/* Src Left offset */
							(UINT32)workbuf.dest_rightoffset,	/* Src right offset */
							(UINT32)NULL);
		DBG_PRINTF("Algorithm= %d, Rotation = %d\n",workbuf.algorithm, workbuf.rotation);
			
		

		vpeIoctl(VPE_IOCTL_SET_FILTER,
							workbuf.algorithm,		//DDA or Bilinear
							(UINT32)NULL,
							(UINT32)NULL);
		if(_MMU_mode==0)
		{
			vpeIoctl(VPE_IOCTL_SET_MMU_ENTRY,
							(UINT32)0,				// Disable MMU mode. 
							(UINT32)0,	
							(UINT32)NULL);
			vpeIoctl(VPE_IOCTL_HOST_OP,
							VPE_HOST_VDEC_LINE,
							workbuf.rotation,		
							(UINT32)NULL);
		}
		else
		{
			vpeIoctl(VPE_IOCTL_HOST_OP,
							VPE_HOST_FRAME,
							workbuf.rotation,		
							(UINT32)NULL);
		}

#if 0
		{
			UINT32 i;
			printk("\nRegister \n");
			for(i=0; i<0x100; i= i+4)
			{
				printk("                0x%x",  inl(REG_VPE_TG+i) );
				if( (i%16)==0)
					printk("\n");	
			}
		}
#endif
		break;	
	case VPE_POLLING_INTERRUPT:
		DBG_PRINTF("VPE Polling TRIGGER\n");		
///		if (copy_to_user((void *)arg, (unsigned int *)&bIsVPEComplete, sizeof(unsigned int))) 
		if (copy_to_user((void *)arg, (void *)((UINT32)&bIsVPEComplete), sizeof(unsigned int)))
		{		
			ret = -EFAULT;
		}
		break;	
	case VPE_WAIT_INTERRUPT:
		DBG_PRINTF("VPE Check TRIGGER\n");
		wait_event_interruptible(vpe_wq, bIsVPEComplete!=0);
		if(bIsVPEComplete==0)
			ret = -ERESTARTSYS;

		break;			
	case VPE_TRIGGER: {
		// Set up MMU table.
		int pixel_width = (workbuf.dest_format == VPE_DST_PACKET_RGB888 ? 4 : 2);
		unsigned long usr_buf = (unsigned long) workbuf.dest_addrPac;
		int usr_buf_sz = (workbuf.dest_width + workbuf.dest_leftoffset + workbuf.dest_rightoffset) * workbuf.dest_height * pixel_width;
		int nr_pages = (((usr_buf + usr_buf_sz + PAGE_SIZE - 1) & PAGE_MASK) - (usr_buf & PAGE_MASK)) >> PAGE_SHIFT;
		struct page **pages = NULL;
		
		if( vpe_stop ==1 )
			break;
		DBG_PRINTF ("pixel_width = %d\n", pixel_width);
		DBG_PRINTF ("usr_buf = 0x%08x\n", usr_buf);
		DBG_PRINTF ("usr_buf_sz = 0x%08x\n", usr_buf_sz);
		DBG_PRINTF ("nr_pages = %d\n", nr_pages);
		do {
			int i = 0;
			int ret = 0;
			
			pages = kmalloc (nr_pages * sizeof (struct page *), GFP_KERNEL);
			// TESTTEST
			DBG_PRINTF ("pages = 0x%08x\n", pages);
			if (! pages) {
				printk ("kmalloc failed\n");
				break;
			}
			
			down_read (&current->mm->mmap_sem);
			ret = get_user_pages (current, current->mm, usr_buf & PAGE_MASK, nr_pages, 1, 0, pages, NULL);
			up_read (&current->mm->mmap_sem);
			// TESTTEST
			DBG_PRINTF("ret = %d\n", ret);
			
			for (i = 0; i < ret; i ++) {
				page_cache_release (pages[i]);
				pages[i] = NULL;
			}
		}while (0);
		if (pages) {
			kfree (pages);
		}
		pages = NULL;
		
		if ( _MMU_mode == 1 )
		{
			do {
				// Fault in destination buffer.
				err = vpe_mlock(usr_buf, usr_buf_sz);
				if (err) {
					break;
				}
				
				pixel_width = (workbuf.src_format == VPE_SRC_PACKET_RGB888 ? 4 : 2);
				
				if ( (workbuf.src_format <= VPE_SRC_PACKET_RGB888) && (workbuf.src_format >= VPE_SRC_PACKET_YUV422) )
				{
					/* VPE_SRC_PACKET_YUV422, VPE_SRC_PACKET_RGB555, VPE_SRC_PACKET_RGB565 and VPE_SRC_PACKET_RGB888 case */
					// Fault in source buffer.
					err = vpe_mlock(workbuf.src_addrPacY, (workbuf.src_width * pixel_width * workbuf.src_height));
					if (err) {
						break;
					}
				}
				else if ( (workbuf.src_format <= 11) && (workbuf.src_format >= 1) )
				{
					/* VPE_SRC_PLANAR_YUV420, VPE_SRC_PLANAR_YUV411, VPE_SRC_PLANAR_YUV422, VPE_SRC_PLANAR_YUV422T and VPE_SRC_PLANAR_YUV444 case */
					// Fault in source buffer.
					usr_buf_sz = workbuf.src_addrU - workbuf.src_addrPacY;
					err = vpe_mlock(workbuf.src_addrPacY, usr_buf_sz);
					if (err) {
						break;
					}
					if ( (workbuf.src_format <= 2) && (workbuf.src_format >= 1) )
					{
						usr_buf_sz = workbuf.src_width * workbuf.src_height / 4;
					}
					else if ( (workbuf.src_format <= 7) && (workbuf.src_format >= 3) )
					{
						usr_buf_sz = workbuf.src_width * workbuf.src_height / 2;
					}
					err = vpe_mlock(workbuf.src_addrU, usr_buf_sz);
					if (err) {
						break;
					}
					err = vpe_mlock(workbuf.src_addrV, usr_buf_sz);
					if (err) {
						break;
					}
				}
				else
				{
					/* VPE_SRC_PLANAR_YONLY case */
					// Fault in source buffer.
					err = vpe_mlock(workbuf.src_addrPacY, (workbuf.src_width * workbuf.src_height));
					if (err) {
						break;
					}
				}
			}
			while (0);
		}
		
		flush_cache_all();
		bIsVPEComplete = 0;
		DBG_PRINTF("VPE_TRIGGER\n");
			vpeIoctl(VPE_IOCTL_TRIGGER,
								(UINT32)NULL,
								(UINT32)NULL,
								(UINT32)NULL);
		break;
	}							
	case VPE_STOP:
		printk("VPE Driver Stop\n");
		while( (((__raw_readl(REG_VPE_TG)) & VPE_GO) == VPE_GO) );
		wake_up_interruptible(&vpe_wq);	
		bIsVPEComplete=1;
		
		priv->state = VPE_FINISH;
		vpe_stop = 1; 
		break;							
	default:
		printk("Unsupported cmd = 0x%x\n",cmd);
		return -ENOIOCTLCMD;
	}
	return ret;
}

static void vpe_cleanup (void)
{
	dev_t dev = MKDEV(major_no, minor_no);

#ifdef VPE_AUTOMATIC_CREATE_DEVICE_FILE
	if (vpe_device) {
		device_destroy(vpe_class, dev);
		vpe_device = NULL;
	}
	if (vpe_class) {
		class_destroy(vpe_class);
		vpe_class = NULL;
	}
#endif	// #ifdef VPE_AUTOMATIC_CREATE_DEVICE_FILE

	if (vpe_dev_created)
	{
		cdev_del(&vpe_dev);
		vpe_dev_created = 0;
	}		

	if (dev_no_created) {
		unregister_chrdev_region(dev, 1);
		dev_no_created = 0;
	}
}

static struct file_operations w55fa92_vpe_fops = {
	.owner		= THIS_MODULE,
	.open		= w55fa92_vpe_open,
	.release		= w55fa92_vpe_close,
//	.read		= w55fa92_vpe_read,
//	.write		= w55fa92_vpe_write,
//	.mmap		= w55fa92_vpe_mmap,
	.ioctl		= w55fa92_vpe_ioctl,
//	.poll			= w55fa92_vpe_poll,
};
#if 0
static int __init w55fa92_vpe_alloc_mem (void)
{
	// To support allocate the total physical memory from system.
	void *txmem = alloc_bootmem_low_pages (VPE_MEM_SIZE);
	if (txmem) {
		_vpe_mem_start_addr = virt_to_phys ((unsigned long) txmem);
		
//		BITSTREAMBUF = _vpe_mem_start_addr;
//		BISTCodeAddr = BITSTREAMBUF + BITSTREAMSIZE;
//		SLICEBUFADDR = BISTCodeAddr + BISTCodeSize;		
//		FRAMEBUFADDR = SLICEBUFADDR + SLICE_SAVE_SIZE;		
	}
	else {
		printk (KERN_ERR "%s: alloc_bootmem_low_pages 0x%08x failed\n", __FUNCTION__, VPE_MEM_SIZE);
	}
	printk (KERN_INFO "%s: allocate memory start = 0x%08x, size = 0x%08x\n", __FUNCTION__, _vpe_mem_start_addr, VPE_MEM_SIZE);
}
#endif
static int __init w55fa92_vpe_init(void)
{
	int err = 0;

	do {
#ifdef VPE_DYNAMIC_ALLOC_DEVICE_NUM
		dev_t dev;
		err = alloc_chrdev_region(&dev, minor_no, 1, DRIVER_NAME);
		if (err) {
			printk(KERN_ERR "[VPE Driver] alloc_chrdev_region failed\n");
			break;
		}
		major_no = MAJOR(dev);
#else
	dev_t dev = MKDEV(VPE_MAJOR, VPE_MINOR);
	
	err = register_chrdev_region(dev, 1, DRIVER_NAME);
	if (err) {
		printk("fa92 vpe initial the device error!\n");
		break;
	}
#endif	// #ifdef VPE_DYNAMIC_ALLOC_DEVICE_NUM
		dev_no_created = 1;
	printk("fa92 vpe register char device Successful!\n");	
	
	cdev_init(&vpe_dev, &w55fa92_vpe_fops);
	vpe_dev.owner = THIS_MODULE;
	vpe_dev.ops = &w55fa92_vpe_fops;

	err = cdev_add(&vpe_dev, dev, 1);
	if (err) {
		printk(KERN_NOTICE "Error adding w55fa92 VPE char device!\n");
		break;
	}
		
	vpe_dev_created	= 1;

#ifdef VPE_AUTOMATIC_CREATE_DEVICE_FILE
		vpe_class = class_create(THIS_MODULE, DRIVER_NAME);
		if(IS_ERR(vpe_class)) {
			printk(KERN_ERR "[VPE Driver] class_create failed\n");
			err = PTR_ERR(vpe_class);
			vpe_class = NULL;
			break;
		}
		
		vpe_device = device_create(vpe_class, NULL, MKDEV(major_no, minor_no), NULL, VPE_DEVICE_FILE_NAME);
		if (IS_ERR(vpe_device)) {
			printk(KERN_ERR "[VPE Driver] device_create failed\n");
			err = PTR_ERR(vpe_device);
			vpe_device = NULL;
			break;
		}
#endif	// #ifdef VPE_AUTOMATIC_CREATE_DEVICE_FILE

	/* initialize locks */
	init_MUTEX(&vpe_lock);
	/* init waitQueue */
	init_waitqueue_head(&vpe_wq);
	printk("w55fa92 VPE driver has been initialized successfully!\n");
	}
	while (0);

	if (err) {
		vpe_cleanup ();
	}

	return err;
}

static void __exit w55fa92_vpe_cleanup(void)
{
	vpe_cleanup();
}

//console_initcall (w55fa92_vpe_alloc_mem);
module_init(w55fa92_vpe_init);
module_exit(w55fa92_vpe_cleanup);

MODULE_DESCRIPTION("HW Video Decoder driver for W55fa92");
MODULE_LICENSE("GPL");


//--------------------------------------------------------------
//
// Copyright (c) Nuvoton Technology Corp. All rights reserved.
//
//--------------------------------------------------------------

#include <linux/kernel.h>
#include <linux/timer.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/miscdevice.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>

#include <linux/compiler.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/list.h>
#include <linux/bootmem.h>
#include <linux/cdev.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>

#include <asm/io.h>
#include <asm/page.h>
#include <asm/cacheflush.h>
#include <asm/uaccess.h>
#include <asm/bitops.h>

#include <mach/w55fa92_reg.h>
#include <mach/w55fa92_aac.h>
#include "DrvAAC.h"

#define AAC_DRIVER_NAME "w55fa92-aac"

#define AAC_DYNAMIC_ALLOC_DEVICE_NUM
#define AAC_AUTOMATIC_CREATE_DEVICE_FILE

#ifdef AAC_AUTOMATIC_CREATE_DEVICE_FILE
#define AAC_DEVICE_FILE_NAME	"aac"
#endif

#if defined(CONFIG_AAC_DRIVER_USE_INTERNAL_SRAM)
	#define DEF_AAC_DRIVER_MEMORY_ADDRESS	(0xFF000000)		//Internal SRAM(8K)
	#define DEF_AAC_DRIVER_MEMORY_SIZE		(8*1024)
#endif

static int major_no = 200;	// FIXME
module_param (major_no, int, 0644);
static int minor_no = 0;
module_param (minor_no, int, 0644);
static int dev_no_created = 0;

#ifdef AAC_AUTOMATIC_CREATE_DEVICE_FILE
static struct class *aac_class = NULL;
static struct device *aac_device = NULL;
#endif	// #ifdef AAC_AUTOMATIC_CREATE_DEVICE_FILE

static int irq_line = IRQ_MDCT;	// FIXME
module_param (irq_line, int, 0644);
static int irq_requested = 0;

static struct cdev aac_dev;
static int aac_dev_created = 0;

// Single-open policy.
static atomic_t aac_avail = ATOMIC_INIT(1);

enum {
	AAC_STAT_MDCT_BY		=	0,
	AAC_STAT_DMAIN_BY		=	1,
	AAC_STAT_DMAOUT_BY		=	2,
	AAC_STAT_ERR			=	3
};

enum {
	MAX_AAC_BUFF		=	8192
};

typedef struct aac_priv {
	unsigned long					stat;	// Use bit operations to ensure atomicity.
	aac_dec_ctx_s 					decoder;
	aac_enc_ctx_s					encoder;
	
	void *							buff_relay_va;		// relay buffer for use with buffer in virtual address
	unsigned long					buff_relay_pa;
	void *							mmaped_addr;
	
	struct semaphore 				sem;
	wait_queue_head_t				wq;
} aac_priv_t;

aac_priv_t *glob_priv = NULL;

static int w55fa92_aac_open(struct inode *inode, struct file *filp);
static int w55fa92_aac_close(struct inode *inode, struct file *file);
static int w55fa92_aac_ioctl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg);
static int w55fa92_aac_mmap(struct file *filp, struct vm_area_struct *vma);

static struct file_operations w55fa92_aac_fops = {
	.owner		= THIS_MODULE,
	.open		= w55fa92_aac_open,
	.release	= w55fa92_aac_close,
//	.read		= w55fa92_aac_read,
//	.write		= w55fa92_aac_write,
	.mmap		= w55fa92_aac_mmap,
    .ioctl      = w55fa92_aac_ioctl
//	.poll		= w55fa92_aac_poll,
};

void AAC_SET_MDCT_BY(void)
{
	set_bit(AAC_STAT_MDCT_BY, &glob_priv->stat);
}
void AAC_SET_DMAIN_BY(void)
{
	set_bit(AAC_STAT_DMAIN_BY, &glob_priv->stat);
}
void AAC_SET_DMAOUT_BY(void)
{
	set_bit(AAC_STAT_DMAOUT_BY, &glob_priv->stat);
}

void AAC_WAIT_MDCT_DONE(void)
{
	wait_event(glob_priv->wq, (! test_bit(AAC_STAT_MDCT_BY, &glob_priv->stat)));
}
void AAC_WAIT_DMAIN_DONE(void)
{
	wait_event(glob_priv->wq, (! test_bit(AAC_STAT_DMAIN_BY, &glob_priv->stat)));
}
void AAC_WAIT_DMAOUT_DONE(void)
{
	wait_event(glob_priv->wq, (! test_bit(AAC_STAT_DMAOUT_BY, &glob_priv->stat)));
}

static irqreturn_t aac_irq_handler(int irq, void *dev_id)
{
	aac_priv_t *priv = (aac_priv_t *) dev_id;

	if (inp32(REG_MDCTINT) & MDCT_INT) {
		outp32(REG_MDCTINT, MDCT_INT);
		clear_bit(AAC_STAT_MDCT_BY, &priv->stat);
	}
	else if (inp32(REG_MDCTINT) & DMAIN_INT) {
		outp32(REG_MDCTINT, DMAIN_INT);
		clear_bit(AAC_STAT_DMAIN_BY, &priv->stat);
	}
	else if (inp32(REG_MDCTINT) & DMAOUT_INT) {
		outp32(REG_MDCTINT, DMAOUT_INT);
		clear_bit(AAC_STAT_DMAOUT_BY, &priv->stat);
	}
	
	wake_up(&priv->wq);
	
	return IRQ_HANDLED;
}

int w55fa92_aac_open(struct inode *inode, struct file *filp)
{
	int err = 0;
	
	if (! atomic_dec_and_test(&aac_avail)) {
		atomic_inc(&aac_avail);
		return -EBUSY;
	}
	
	do {
		aac_priv_t *priv = (aac_priv_t *) kmalloc(sizeof (aac_priv_t), GFP_KERNEL);
		if (! priv) {
			err = -ENOMEM;
			break;
		}
		glob_priv = priv;
		
		memset(priv, 0x00, sizeof (aac_priv_t));
		init_MUTEX(&priv->sem);
		init_waitqueue_head (&priv->wq);
		
		filp->private_data = priv;

#if defined(CONFIG_AAC_DRIVER_USE_INTERNAL_SRAM)
		printk("Use internal 8KB SRAM as DMA buffer.\n");
		priv->buff_relay_pa	= DEF_AAC_DRIVER_MEMORY_ADDRESS;
		priv->buff_relay_va = ioremap ( DEF_AAC_DRIVER_MEMORY_ADDRESS, MAX_AAC_BUFF );				
#else	
		priv->buff_relay_va = dma_alloc_coherent(NULL, MAX_AAC_BUFF, (dma_addr_t *) &priv->buff_relay_pa, GFP_KERNEL);
#endif
		priv->mmaped_addr = NULL;
		if (! priv->buff_relay_va) {
			printk(KERN_ERR "[AAC Driver] DMA alloc/mapping 0x%08x failed.\n", MAX_AAC_BUFF );
			err = -ENOMEM;
			break;
		}		

		err = request_irq(irq_line, aac_irq_handler, IRQF_DISABLED, AAC_DRIVER_NAME, priv);
		if (err) {
			printk(KERN_ERR "[AAC Driver] request_irq failed, irq_line = %d, err= %d\n", irq_line, err);
			break;
		}
		irq_requested = 1;
		
		{	// GCR may access by other drivers, so disable all interrupts for sync.
			unsigned long flags;
			local_irq_save(flags);
			DrvAAC_Open();
			local_irq_restore(flags);
		}
	}
	while (0);
	
	if (err) {
		w55fa92_aac_close(inode, filp);
	}

	return err;
}

int w55fa92_aac_close(struct inode *inode, struct file *filp)
{
	aac_priv_t *priv = filp->private_data;
	
	{	// GCR may access by other drivers, so disable all interrupts for sync.
		unsigned long flags;
		local_irq_save(flags);
		DrvAAC_Close();
		local_irq_restore(flags);
	}
		
	if (irq_requested) {
		free_irq(irq_line, priv);
		irq_requested = 0;
	}
	
	glob_priv = NULL;
	if (priv) {
		if (priv->buff_relay_va) {

#ifdef CONFIG_AAC_DRIVER_USE_INTERNAL_SRAM
		iounmap(priv->buff_relay_va);
#else
		dma_free_coherent(NULL, MAX_AAC_BUFF, priv->buff_relay_va, priv->buff_relay_pa);
#endif					
			priv->buff_relay_va = NULL;
			priv->buff_relay_pa = 0;
		}
		
		kfree(priv);
		filp->private_data = priv = NULL;
	}

	atomic_inc(&aac_avail);
	
	return 0;
}

static int w55fa92_check_in_mapped_region ( aac_priv_t *priv, unsigned int query_addr ) 
{
	if ( (query_addr >= (unsigned int)(priv->mmaped_addr)) && 
		(query_addr < (unsigned int)(priv->mmaped_addr+MAX_AAC_BUFF))  )
		return 1;
	return 0;
}


int w55fa92_aac_ioctl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg)
{
	aac_priv_t *priv = filp->private_data;
	int nonblock = !!(filp->f_flags & O_NONBLOCK);
	int err = 0;
	unsigned int offset = 0;
	
	(void) nonblock;	// Suppress "unused variable" warning.
	
	// Extract the type and number bitfields, and don't decode wrong commands.
	if (_IOC_TYPE(cmd) != AAC_IOC_MAGIC) {
		return -ENOTTY;
	}
	if (_IOC_NR(cmd) > AAC_IOC_MAXNR) {
		return -ENOTTY;
	}
	
	// The direction is a bitmask, and VERIFY_WRITE catches R/W transfers. 
	// `Type' is user-oriented, while access_ok is kernel-oriented, so the concept of "read" and "write" is reversed.
	if ((_IOC_DIR(cmd) & _IOC_READ) &&
		! access_ok(VERIFY_WRITE, (void __user *) arg, _IOC_SIZE(cmd))) {
		return -ENOTTY;
	}
	if ((_IOC_DIR(cmd) & _IOC_WRITE) &&
		! access_ok(VERIFY_READ, (void __user *) arg, _IOC_SIZE(cmd))) {
		return -ENOTTY;
	}
	
	switch (cmd) {
	case AAC_IOCSENC:
	
		if (copy_from_user((void *) &priv->encoder, (void *) arg, sizeof (aac_enc_ctx_s))) {
			err = -EFAULT;
			break;
		}
		
		if ((priv->encoder.i32Size << 2 ) > MAX_AAC_BUFF) {
			printk(KERN_ERR "[AAC Driver] %d(%d) larger than max encoder buffer size: %d.\n", priv->encoder.i32Size, priv->encoder.i32Size<<2, MAX_AAC_BUFF);
			err = -EFAULT;
			break;
		}
		
		//printk( "Enc -> %x, %x, %d\n", (unsigned int)priv->mmaped_addr, (unsigned int)priv->encoder.inbuf, priv->encoder.i32Size  );						
		if ( w55fa92_check_in_mapped_region(priv, (unsigned int)priv->encoder.inbuf ) )
		{
			offset = (unsigned int)((void*)priv->encoder.inbuf - priv->mmaped_addr);
			
			flush_cache_all();
			
			DrvAAC_Encoder((INT32 *) (priv->buff_relay_pa+offset), (INT32 *) (priv->buff_relay_pa+offset), priv->encoder.i32Size);
			if ( (unsigned int)priv->encoder.outbuf != (unsigned int)(priv->mmaped_addr+offset) )
			{
				if (copy_to_user(priv->encoder.outbuf, (void *) ((unsigned long) priv->buff_relay_va+offset), priv->encoder.i32Size << 1)) {
					err = -EFAULT;
					break;
				}				
			}			

		} else {
			
			if (copy_from_user(priv->buff_relay_va, priv->encoder.inbuf, (priv->encoder.i32Size << 2))) {
				err = -EFAULT;
				break;
			}			

			flush_cache_all();

			DrvAAC_Encoder((INT32 *) priv->buff_relay_pa, (INT32 *) priv->buff_relay_pa, priv->encoder.i32Size);

			if (copy_to_user(priv->encoder.outbuf, (void *) ((unsigned long) priv->buff_relay_va), priv->encoder.i32Size << 1)) {
				err = -EFAULT;
				break;
			}
									
		} 
		
		break;

	case AAC_IOCSDEC:
	
		if (copy_from_user((void *) &priv->decoder, (void *) arg, sizeof (aac_dec_ctx_s))) {
			err = -EFAULT;
			break;
		}
		
		if ( (priv->decoder.i32Size << 2 ) > MAX_AAC_BUFF) {
			printk(KERN_ERR "[AAC Driver] %d larger than max decoder buffer size: %d.\n", priv->decoder.i32Size, MAX_AAC_BUFF);
			err = -EFAULT;
			break;
		}

		//printk( "Dec -> %x, %x, %d\n", (unsigned int)priv->mmaped_addr, (unsigned int)priv->decoder.outbuf, priv->decoder.i32Size  );
		if ( w55fa92_check_in_mapped_region(priv, (unsigned int)priv->decoder.outbuf ) )
		{
			offset = (unsigned int)((void*)priv->decoder.outbuf - priv->mmaped_addr);			

			if ( (unsigned int)priv->decoder.inbuf != (unsigned int)(priv->mmaped_addr+offset) )
				if (copy_from_user(  (void*)(priv->buff_relay_va+offset), priv->decoder.inbuf, (priv->decoder.i32Size << 1))){
					err = -EFAULT;
					break;
				}
				
			flush_cache_all();
			DrvAAC_Decoder(priv->decoder.i32Size, (INT32*)(priv->buff_relay_pa+offset), (INT32*)(priv->buff_relay_pa+offset) );
			
		} else {

			if (copy_from_user(priv->buff_relay_va, priv->decoder.inbuf, (priv->decoder.i32Size << 1))){
				err = -EFAULT;
				break;
			}				
			
			flush_cache_all();
			
			DrvAAC_Decoder(priv->decoder.i32Size, (INT32 *) priv->buff_relay_pa, (INT32 *) (priv->buff_relay_pa));
			
			if (copy_to_user(priv->decoder.outbuf, (void *) ((unsigned long) priv->buff_relay_va), priv->decoder.i32Size << 2)) {
				err = -EFAULT;
				break;
			}			
			
		}
		break;

	case AAC_IOCTRIGGER:	// Start encode/decode.
		// TO BE CONTINUED
		break;
	
	case AAC_IOCWAIT:	// Wait finish.
		// TO BE CONTINUED
		break;
		
	default:
		err = -ENOTTY;
	}
	
	return err;
}

static int w55fa92_aac_mmap(struct file *filp, struct vm_area_struct *vma)
{
	aac_priv_t *priv = filp->private_data;

	int err = 0;
	
	unsigned long region_origin = vma->vm_pgoff * PAGE_SIZE;
	unsigned long region_length = vma->vm_end - vma->vm_start;	
	unsigned long physical_addr = priv->buff_relay_pa + region_origin;	
	unsigned long user_virtaddr = vma->vm_start;
	      
	if ( region_length != MAX_AAC_BUFF ) {
		printk ("%s: Mapped size != MAX_AAC_BUFF \n", __FUNCTION__);
		err = -ENXIO;
		goto EXIT_W55FA92_AAC_MMAP;	
	}
	
	vma->vm_flags		|=  VM_RESERVED;	/* Wayne: Importanat */
	vma->vm_page_prot 	= pgprot_noncached (vma->vm_page_prot);

	if ( remap_pfn_range(vma, user_virtaddr,  physical_addr>> PAGE_SHIFT, region_length, vma->vm_page_prot) < 0 )
    {
		err = -ENXIO;
		goto EXIT_W55FA92_AAC_MMAP;			
	}

	priv->mmaped_addr = (void*)user_virtaddr;
	//printk( "vsize=%lu, P=%x mmaped address=%x, vma->vm_flags=%x\n", region_length, (__u32)priv->buff_relay_pa, (__u32)vma->vm_start, vma->vm_flags );		
	
EXIT_W55FA92_AAC_MMAP:
	return err;		
}

static void aac_cleanup (void)
{
	dev_t dev_no = MKDEV(major_no, minor_no);
	
#ifdef AAC_AUTOMATIC_CREATE_DEVICE_FILE
	if (aac_device) {
		device_destroy(aac_class, dev_no);
		aac_device = NULL;
	}
	if (aac_class) {
		class_destroy(aac_class);
		aac_class = NULL;
	}
#endif	// #ifdef AAC_AUTOMATIC_CREATE_DEVICE_FILE
	
	if (aac_dev_created) {
		cdev_del(&aac_dev);
		aac_dev_created = 0;
	}
	
	if (dev_no_created) {
		unregister_chrdev_region(dev_no, 1);
		dev_no_created = 0;
	}
}

static int __init w55fa92_aac_init(void)
{
	int err = 0;
	
	do {
#ifdef AAC_DYNAMIC_ALLOC_DEVICE_NUM
		dev_t dev_no;
		err = alloc_chrdev_region(&dev_no, minor_no, 1, AAC_DRIVER_NAME);
		if (err) {
			printk(KERN_ERR "[AAC Driver] alloc_chrdev_region failed\n");
			break;
		}
		major_no = MAJOR(dev_no);
#else
		dev_t dev_no = MKDEV(major_no, minor_no);
		err = register_chrdev_region(dev_no, 1, AAC_DRIVER_NAME);
		if (err) {
			printk(KERN_ERR "[AAC Driver] register_chrdev_region failed\n");
			break;
		}
#endif	// #ifdef AAC_DYNAMIC_ALLOC_DEVICE_NUM
		dev_no_created = 1;

    	cdev_init(&aac_dev, &w55fa92_aac_fops);
		aac_dev.owner = THIS_MODULE;
		err = cdev_add (&aac_dev, dev_no, 1);
		if (err) {
			printk (KERN_ERR "[AAC Driver] cdev_add failed\n");
			break;
    	}
    	aac_dev_created = 1;
		
#ifdef AAC_AUTOMATIC_CREATE_DEVICE_FILE
		aac_class = class_create(THIS_MODULE, AAC_DRIVER_NAME);
		if(IS_ERR(aac_class)) {
			printk(KERN_ERR "[AAC Driver] class_create failed\n");
			err = PTR_ERR(aac_class);
			aac_class = NULL;
			break;
		}
		
		aac_device = device_create(aac_class, NULL, MKDEV(major_no, minor_no), NULL, AAC_DEVICE_FILE_NAME);
		if (IS_ERR(aac_device)) {
			printk(KERN_ERR "[AAC Driver] device_create failed\n");
			err = PTR_ERR(aac_device);
			aac_device = NULL;
			break;
		}
#endif	// #ifdef AAC_AUTOMATIC_CREATE_DEVICE_FILE
	}
	while (0);
	
	if (err) {
		aac_cleanup ();
	}
	
	return err;
}

static void __exit w55fa92_aac_exit(void)
{
	aac_cleanup();
}

module_init(w55fa92_aac_init);
module_exit(w55fa92_aac_exit);

MODULE_DESCRIPTION("W55FA92 AAC driver");
MODULE_LICENSE("GPL");

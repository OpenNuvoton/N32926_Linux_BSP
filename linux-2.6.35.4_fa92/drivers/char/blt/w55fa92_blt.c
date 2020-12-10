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
#include <linux/syscalls.h>

#include <asm/io.h>
#include <asm/page.h>
#include <asm/cacheflush.h>
#include <asm/uaccess.h>
#include <asm/bitops.h>

#include <mach/w55fa92_reg.h>
#include <mach/w55fa92_blt.h>
#include "blt.h"

#define BLT_DRIVER_NAME "w55fa92-blt"

#define BLT_DYNAMIC_ALLOC_DEVICE_NUM
#define BLT_AUTOMATIC_CREATE_DEVICE_FILE

#ifdef BLT_AUTOMATIC_CREATE_DEVICE_FILE
#define BLT_DEVICE_FILE_NAME	"blt"
#endif

static int major_no = 198;
module_param (major_no, int, 0644);
static int minor_no = 0;
module_param (minor_no, int, 0644);
static int dev_no_created = 0;

#ifdef BLT_AUTOMATIC_CREATE_DEVICE_FILE
static struct class *blt_class = NULL;
static struct device *blt_device = NULL;
#endif	// #ifdef BLT_AUTOMATIC_CREATE_DEVICE_FILE

static int irq_line = IRQ_BLT;
module_param (irq_line, int, 0644);
static int irq_requested = 0;

static struct cdev blt_dev;
static int blt_dev_created = 0;

// Single-open policy.
static atomic_t blt_avail = ATOMIC_INIT(1);

enum {
	BLT_STAT_BY			=	0,
	BLT_STAT_MMU		=	1,
	BLT_STAT_ERR		=	2,
};

typedef struct blt_priv {
	unsigned long					stat;	// Use bit operations to ensure atomicity.
	
	S_DRVBLT_BLIT_OP 				blitop;
	S_DRVBLT_FILL_OP 				fillop;
	
	S_DRVBLT_ARGB8					pal[256];
	S_DRVBLT_BLIT_TRANSFORMATION	xform;
	
	struct semaphore 				sem;
	wait_queue_head_t				wq;
} blt_priv_t;

static int w55fa92_blt_open(struct inode *inode, struct file *filp);
static int w55fa92_blt_close(struct inode *inode, struct file *file);
static int w55fa92_blt_ioctl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg);
static ssize_t w55fa92_blt_read(struct file *filp, char __user *buf, size_t count, loff_t *ppos);


static struct file_operations w55fa92_blt_fops = {
	.owner		= THIS_MODULE,
	.open		= w55fa92_blt_open,
	.release	= w55fa92_blt_close,
	.read		= w55fa92_blt_read,		// For backward-compatible with W55FA93's BLT driver.
//	.write		= w55fa92_blt_write,
//	.mmap		= w55fa92_blt_mmap,
    .ioctl      = w55fa92_blt_ioctl
//	.poll		= w55fa92_blt_poll,
};

static void _cmplt_hdlr(unsigned long usrDat)
{
	blt_priv_t *priv = (blt_priv_t *) usrDat;
	
	clear_bit(BLT_STAT_BY, &priv->stat);
	wake_up_interruptible(&priv->wq);
}

static irqreturn_t blt_irq_handler(int irq, void *dev_id)
{
	//blt_priv_t *priv = (blt_priv_t *) dev_id;
	_blt_intr_hdlr();
	
	return IRQ_HANDLED;
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
static int blt_mlock(unsigned long start, size_t len)
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

int w55fa92_blt_open(struct inode *inode, struct file *filp)
{
	int err = 0;
	
	if (! atomic_dec_and_test(&blt_avail)) {
		atomic_inc(&blt_avail);
		return -EBUSY;
	}
	
	do {
		blt_priv_t *priv = (blt_priv_t *) kmalloc(sizeof (blt_priv_t), GFP_KERNEL);
		if (! priv) {
			err = -ENOMEM;
			break;
		}
		
		memset(priv, 0x00, sizeof (blt_priv_t));
		init_MUTEX(&priv->sem);
		init_waitqueue_head (&priv->wq);
		
		filp->private_data = priv;
	
		err = request_irq(irq_line, blt_irq_handler, IRQF_DISABLED, BLT_DRIVER_NAME, priv);
		if (err) {
			printk(KERN_ERR "[BLT Driver] request_irq failed, irq_line = %d, err= %d\n", irq_line, err);
			break;
		}
		irq_requested = 1;
		
		//{	// GCR may access by other drivers, so disable all interrupts for sync.
		//	unsigned long flags;
		//	local_irq_save(flags);
		//	bltOpen();
		//	local_irq_restore(flags);
		//}
		bltOpen();	// Take care of race condition in itself.
		
		bltInstallCallback(BLT_INT_CMPLT, _cmplt_hdlr, (unsigned long) priv, NULL, NULL);
		bltEnableInt(BLT_INT_CMPLT);
	}
	while (0);
	
	if (err) {
		w55fa92_blt_close(inode, filp);
	}

	return err;
}

int w55fa92_blt_close(struct inode *inode, struct file *filp)
{
	blt_priv_t *priv = filp->private_data;

	bltDisableInt(BLT_INT_CMPLT);
	bltInstallCallback(BLT_INT_CMPLT, _cmplt_hdlr, 0, NULL, NULL);
	
	//{	// GCR may access by other drivers, so disable all interrupts for sync.
	//	unsigned long flags;
	//	local_irq_save(flags);
	//	bltClose();
	//	local_irq_restore(flags);
	//}
	bltClose();	// Take care of race condition in itself.
		
	if (irq_requested) {
		free_irq(irq_line, priv);
		irq_requested = 0;
	}
	
	if (priv) {
		kfree(priv);
		filp->private_data = priv = NULL;
	}

	atomic_inc(&blt_avail);
	
	return 0;
}

ssize_t w55fa92_blt_read(struct file *filp, char __user *buf, size_t count, loff_t *ppos)
{
	blt_priv_t *priv = filp->private_data;
	int nonblock = filp->f_flags & O_NONBLOCK;
	ssize_t ret = 0;
	
	do {
		if (nonblock) {
			if (test_bit(BLT_STAT_BY, &priv->stat)) {
				ret = -EAGAIN;
			}
			break;
		}

		ret = wait_event_interruptible(priv->wq, (! test_bit(BLT_STAT_BY, &priv->stat)));
		if (ret) {
			break;
		}
	}
	while(0);
	
	return ret;
}

int w55fa92_blt_ioctl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg)
{
	blt_priv_t *priv = filp->private_data;
	int nonblock = !!(filp->f_flags & O_NONBLOCK);
	int err = 0;
	
	// Extract the type and number bitfields, and don't decode wrong commands.
	if (_IOC_TYPE(cmd) != BLT_IOC_MAGIC) {
		return -ENOTTY;
	}
	if (_IOC_NR(cmd) > BLT_IOC_MAXNR) {
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
	case BLT_IOCFLUSH:
		do {
			if (nonblock) {
				if (test_bit(BLT_STAT_BY, &priv->stat)) {
					err = -EAGAIN;
				}
				break;
			}

			err = wait_event_interruptible(priv->wq, (! test_bit(BLT_STAT_BY, &priv->stat)));
			if (err) {
				break;
			}
		}
		while(0);
		break;

	case BLT_IOCTRIGGER:
		if (test_bit(BLT_STAT_BY, &priv->stat)) {
			err = -EBUSY;
			break;
		}
		
		do {
			if (test_bit(BLT_STAT_MMU, &priv->stat)) {
                if (bltGetFillOP()) {	// Fault in destination buffer for fill op.
                    S_DRVBLT_DEST_FB dest;
					bltGetDestFrameBuf(&dest);
                                        
                    err = blt_mlock(dest.u32FrameBufAddr, dest.i32Stride * dest.i16Height);
                    if (err) {
                        break;
                    }
                }
                else {	// Fault in source/destination buffers for blit op.
                    // Get source setting of H/W, exclusive of palette-related setting.
					bltGetSrcImage(&priv->blitop.src);
					// Get destination setting of H/W.
					bltGetDestFrameBuf(&priv->blitop.dest);
                    
                    // Fault in source buffer.
                    err = blt_mlock(priv->blitop.src.u32SrcImageAddr, priv->blitop.src.i32Stride * priv->blitop.src.i16Height);
                    if (err) {
                        break;
                    }
                    
                    // Fault in destination buffer.
                    err = blt_mlock(priv->blitop.dest.u32FrameBufAddr, priv->blitop.dest.i32Stride * priv->blitop.dest.i16Height);
                    if (err) {
                        break;
                    }
                }
                /*
				int nr_pages = 0;
				unsigned long va_start, va_end, va_start_pgaln, va_end_pgaln;
				int nr_pages_ret;
				
				// Relevant buffers may have not mapped yet for BLT MMU. Fault in them manually.
				if (bltGetFillOP()) {	// Fault in destination buffer for fill op.
					// Get destination setting of H/W.
					S_DRVBLT_DEST_FB dest;
					bltGetDestFrameBuf(&dest);
					
					va_start = dest.u32FrameBufAddr;
					va_end = va_start + dest.i32Stride * dest.i16Height;
					va_start_pgaln = va_start & PAGE_MASK;
					va_end_pgaln = (va_end + PAGE_SIZE - 1)  & PAGE_MASK;
					nr_pages = (va_end_pgaln - va_start_pgaln) >> PAGE_SHIFT;
					down_read(&current->mm->mmap_sem);
					nr_pages_ret = get_user_pages(current, current->mm, va_start_pgaln, nr_pages, 1, 0, NULL, NULL);
					up_read(&current->mm->mmap_sem);
				}
				else {	// Fault in source/destination buffers for blit op.
					// Get source setting of H/W, exclusive of palette-related setting.
					bltGetSrcImage(&priv->blitop.src);
					// Get destination setting of H/W.
					bltGetDestFrameBuf(&priv->blitop.dest);
			
					// Fault in source buffer.
					va_start = priv->blitop.src.u32SrcImageAddr;
					va_end = va_start + priv->blitop.src.i32Stride * priv->blitop.src.i16Height;
					va_start_pgaln = va_start & PAGE_MASK;
					va_end_pgaln = (va_end + PAGE_SIZE - 1)  & PAGE_MASK;
					nr_pages = (va_end_pgaln - va_start_pgaln) >> PAGE_SHIFT;
					down_read(&current->mm->mmap_sem);
					nr_pages_ret = get_user_pages(current, current->mm, va_start_pgaln, nr_pages, 1, 0, NULL, NULL);
					up_read(&current->mm->mmap_sem);
					
					// Fault in destination buffer.
					va_start = priv->blitop.dest.u32FrameBufAddr;
					va_end = va_start + priv->blitop.dest.i32Stride * priv->blitop.dest.i16Height;
					va_start_pgaln = va_start & PAGE_MASK;
					va_end_pgaln = (va_end + PAGE_SIZE - 1)  & PAGE_MASK;
					nr_pages = (va_end_pgaln - va_start_pgaln) >> PAGE_SHIFT;
					down_read(&current->mm->mmap_sem);
					nr_pages_ret = get_user_pages(current, current->mm, va_start_pgaln, nr_pages, 1, 0, NULL, NULL);
					up_read(&current->mm->mmap_sem);
				}*/
				
				bltmmuFlushTLB();   // Flush MMU TLB because OS'es MMU table may change after the last trigger.
			}
			
			set_bit(BLT_STAT_BY, &priv->stat);
			flush_cache_all();
			bltTrigger();
		}
		while (0);
		break;

	case BLT_IOCSBLIT:
		if (test_bit(BLT_STAT_BY, &priv->stat)) {
			err = -EBUSY;
			break;
		}
		
		if (copy_from_user((void *) &priv->blitop, (void *) arg, sizeof (priv->blitop))) {
			err = -EFAULT;
			break;
		}

		do {
			bltSetFillOP(0);
			
			if (priv->blitop.src.pSARGB8) {
				// WARNING: Don't copy from user space to kernel space directly, even though it may be OK here.
				S_DRVBLT_ARGB8 *pal_prtl = priv->pal + priv->blitop.src.u32PaletteInx;
				if (copy_from_user(pal_prtl, priv->blitop.src.pSARGB8, sizeof (S_DRVBLT_ARGB8) * priv->blitop.src.u32Num)) {
					err = -EFAULT;
					break;
				}
				bltSetColorPalette(priv->blitop.src.u32PaletteInx, priv->blitop.src.u32Num, pal_prtl);
			}
			
			if (priv->blitop.transformation) {
				// WARNING: Don't copy from user space to kernel space directly, even though it may be OK here.
				if (copy_from_user(&priv->xform, priv->blitop.transformation, sizeof (priv->xform))) {
					err = -EFAULT;
					break;
				}
				
				bltSetTransformMatrix(priv->xform.matrix);
				bltSetSrcFormat(priv->xform.srcFormat);
				bltSetDisplayFormat(priv->xform.destFormat);
				bltSetTransformFlag(priv->xform.flags);
				bltSetColorMultiplier(priv->xform.colorMultiplier);
				bltSetColorOffset(priv->xform.colorOffset);
				bltSetFillStyle(priv->xform.fillStyle);
			}
			
			bltSetSrcImage(priv->blitop.src);			
			bltSetDestFrameBuf(priv->blitop.dest);
		}
		while (0);
		break;

	case BLT_IOCGBLIT:
		// Get source setting of H/W, exclusive of palette-related setting.
		bltGetSrcImage(&priv->blitop.src);
		// Get destination setting of H/W.
		bltGetDestFrameBuf(&priv->blitop.dest);
		// Get transformation setting of H/W.
		bltGetTransformMatrix(&priv->xform.matrix);
		priv->xform.srcFormat = bltGetSrcFormat();
		priv->xform.destFormat = bltGetDisplayFormat();
		priv->xform.flags = bltGetTransformFlag();
		bltGetColorMultiplier(&priv->xform.colorMultiplier);
		bltGetColorOffset(&priv->xform.colorOffset);
		priv->xform.fillStyle = bltGetFillStyle();
		// Get palette setting of H/W.
		bltGetColorPalette(0, 256, priv->pal);

		do {
			S_DRVBLT_BLIT_OP __user *blitop_usr = (S_DRVBLT_BLIT_OP *) arg;
			S_DRVBLT_ARGB8 __user *pSARGB8_orig;
			unsigned int u32Num_orig;
			unsigned int u32PaletteInx_orig;
			S_DRVBLT_BLIT_TRANSFORMATION * __user transformation_orig;
		
			// Save passed in arguments from user space, which will be overwritten later.
			err = get_user(pSARGB8_orig, (S_DRVBLT_ARGB8 * __user *) &blitop_usr->src.pSARGB8);
			if (err) {
				break;
			}
			err = get_user(u32Num_orig, (unsigned int __user *) &blitop_usr->src.u32Num);
			if (err) {
				break;
			}
			err = get_user(u32PaletteInx_orig, (unsigned int __user *) &blitop_usr->src.u32PaletteInx);
			if (err) {
				break;
			}
			err = get_user(transformation_orig, (S_DRVBLT_BLIT_TRANSFORMATION * __user *) &blitop_usr->transformation);
			if (err) {
				break;
			}
		
			// Copy all blit-related setting of H/W to user space, exclusive of transformation and palette-related setting.
			if (copy_to_user((void *)arg, &priv->blitop, sizeof (priv->blitop))) {
				err = -EFAULT;
				break;
			}
		
			// Restore passed in arguments from user space.
			err = put_user(pSARGB8_orig, (S_DRVBLT_ARGB8 * __user *) &blitop_usr->src.pSARGB8);
			if (err) {
				break;
			}
			err = put_user(u32Num_orig, (unsigned int __user *) &blitop_usr->src.u32Num);
			if (err) {
				break;
			}
			err = put_user(u32PaletteInx_orig, (unsigned int __user *) &blitop_usr->src.u32PaletteInx);
			if (err) {
				break;
			}
			err = put_user(transformation_orig, (S_DRVBLT_BLIT_TRANSFORMATION * __user *) &blitop_usr->transformation);
			if (err) {
				break;
			}
				
			if (transformation_orig) {	// Copy transformation setting of H/W to user space as requested.
				copy_to_user(transformation_orig, &priv->xform, sizeof (priv->xform));
			}
			
			if (pSARGB8_orig) {	// Copy palette setting of H/W to user space as requested.
				S_DRVBLT_ARGB8 *pal_prtl = priv->pal + u32PaletteInx_orig;
				copy_to_user(pSARGB8_orig, pal_prtl, sizeof (S_DRVBLT_ARGB8) * u32Num_orig);
			}
		}
		while (0);
		break;

	case BLT_IOCSFILL:
		if (test_bit(BLT_STAT_BY, &priv->stat)) {
			err = -EBUSY;
			break;
		}
		
		if (copy_from_user((void *) &priv->fillop, (void *) arg, sizeof (priv->fillop))) {
			err = -EFAULT;
			break;
		}

		bltSetFillOP(1);
		{
			S_DRVBLT_DEST_FB dest;
			
			dest.u32FrameBufAddr = priv->fillop.u32FrameBufAddr;
			dest.i32XOffset = priv->fillop.rect.i16Xmin;
			dest.i32YOffset = priv->fillop.rect.i16Ymin;
			dest.i32Stride = priv->fillop.rowBytes;
			dest.i16Width = priv->fillop.rect.i16Xmax - priv->fillop.rect.i16Xmin;
			dest.i16Height = priv->fillop.rect.i16Ymax - priv->fillop.rect.i16Ymin;
			bltSetDestFrameBuf(dest);	
		}
		bltSetARGBFillColor(priv->fillop.color);
		bltSetDisplayFormat(priv->fillop.format);
		bltSetFillAlpha(!!(priv->fillop.blend));
		break;
		
	case BLT_IOCGFILL:
		{
			S_DRVBLT_DEST_FB dest;
			
			bltGetDestFrameBuf(&dest);
			priv->fillop.u32FrameBufAddr = dest.u32FrameBufAddr;
			priv->fillop.rect.i16Xmin = dest.i32XOffset;
			priv->fillop.rect.i16Ymin = dest.i32YOffset;
			priv->fillop.rowBytes = dest.i32Stride;
			priv->fillop.rect.i16Xmax = priv->fillop.rect.i16Xmin + dest.i16Width;
			priv->fillop.rect.i16Ymax = priv->fillop.rect.i16Ymin + dest.i16Height;
		}
		bltGetARGBFillColor(&priv->fillop.color);
		priv->fillop.format = bltGetDisplayFormat();
		priv->fillop.blend = bltGetFillAlpha();
			
		if (copy_to_user((void *) arg, (void *) &priv->fillop, sizeof (priv->fillop))) {
			err = -EFAULT;
			break;
		}
		break;
	
	case BLT_SET_RGB565_COLORKEY:	// access_ok may fail because this I/O control code doesn't follow I/F. But for backward-compatible with W55FA93's BLT driver, keep it.
	case BLT_IOCTRGB565COLORKEY:
		if (test_bit(BLT_STAT_BY, &priv->stat)) {
			err = -EBUSY;
			break;
		}
		
		bltSetRGB565TransparentColor((unsigned int) arg);
		break;
		
	case BLT_IOCQRGB565COLORKEY:
		err = bltGetRGB565TransparentColor();
		break;
		
	case BLT_IOCTENABLERGB565COLORKEY:
		if (test_bit(BLT_STAT_BY, &priv->stat)) {
			err = -EBUSY;
			break;
		}
		
		bltSetRGB565TransparentCtl(1);
		break;
		
	case BLT_IOCTDISABLERGB565COLORKEY:
		if (test_bit(BLT_STAT_BY, &priv->stat)) {
			err = -EBUSY;
			break;
		}
		
		bltSetRGB565TransparentCtl(0);
		break;
	
    case BLT_IOCTBLTSRCFMTPREMULALPHA:
		if (test_bit(BLT_STAT_BY, &priv->stat)) {
			err = -EBUSY;
			break;
		}
		
		bltSetRevealAlpha(eDRVBLT_EFFECTIVE);
		break;

    case BLT_IOCTBLTSRCFMTNONPREMULALPHA:
		if (test_bit(BLT_STAT_BY, &priv->stat)) {
			err = -EBUSY;
			break;
		}
		
		bltSetRevealAlpha(eDRVBLT_NO_EFFECTIVE);
		break;

	case BLT_IOCQBUSY:
		err = test_bit(BLT_STAT_BY, &priv->stat);
		break;
		
	case BLT_IOCTMMU:
		if (test_bit(BLT_STAT_BY, &priv->stat)) {
			err = -EBUSY;
			break;
		}
		
		do {
			if (test_bit(BLT_STAT_MMU, &priv->stat) && ! arg) {	// MMU enabled. Disable it.
				bltmmuTeardown();	// Tear down BLT MMU.
				clear_bit(BLT_STAT_MMU, &priv->stat);
			}
			else if (! test_bit(BLT_STAT_MMU, &priv->stat) && arg) {	// MMU disabled. Enable it.
				// Use cpu_get_pgd instead of current->mm->pgd because current->mm->pgd may not refer to the translation table CPU is using during context switch,
				// even though that will not happen here.
				bltmmuSetup(__pa(cpu_get_pgd()), 1, 1);	// Set up BLT MMU.
				set_bit(BLT_STAT_MMU, &priv->stat);
			}
		}
		while (0);
		break;

	case BLT_IOCQMMU:
		err = test_bit(BLT_STAT_MMU, &priv->stat);
		break;
		
	default:
		err = -ENOTTY;
	}
	
	return err;
}

static void blt_cleanup (void)
{
	dev_t dev_no = MKDEV(major_no, minor_no);
	
#ifdef BLT_AUTOMATIC_CREATE_DEVICE_FILE
	if (blt_device) {
		device_destroy(blt_class, dev_no);
		blt_device = NULL;
	}
	if (blt_class) {
		class_destroy(blt_class);
		blt_class = NULL;
	}
#endif	// #ifdef BLT_AUTOMATIC_CREATE_DEVICE_FILE
	
	if (blt_dev_created) {
		cdev_del(&blt_dev);
		blt_dev_created = 0;
	}
	
	if (dev_no_created) {
		unregister_chrdev_region(dev_no, 1);
		dev_no_created = 0;
	}
}

static int __init w55fa92_blt_init(void)
{
	int err = 0;
	
	do {
#ifdef BLT_DYNAMIC_ALLOC_DEVICE_NUM
		dev_t dev_no;
		err = alloc_chrdev_region(&dev_no, minor_no, 1, BLT_DRIVER_NAME);
		if (err) {
			printk(KERN_ERR "[BLT Driver] alloc_chrdev_region failed\n");
			break;
		}
		major_no = MAJOR(dev_no);
#else
		dev_t dev_no = MKDEV(major_no, minor_no);
		err = register_chrdev_region(dev_no, 1, BLT_DRIVER_NAME);
		if (err) {
			printk(KERN_ERR "[BLT Driver] register_chrdev_region failed\n");
			break;
		}
#endif	// #ifdef BLT_DYNAMIC_ALLOC_DEVICE_NUM
		dev_no_created = 1;

    	cdev_init(&blt_dev, &w55fa92_blt_fops);
		blt_dev.owner = THIS_MODULE;
		err = cdev_add (&blt_dev, dev_no, 1);
		if (err) {
			printk (KERN_ERR "[BLT Driver] cdev_add failed\n");
			break;
    	}
    	blt_dev_created = 1;
		
#ifdef BLT_AUTOMATIC_CREATE_DEVICE_FILE
		blt_class = class_create(THIS_MODULE, BLT_DRIVER_NAME);
		if(IS_ERR(blt_class)) {
			printk(KERN_ERR "[BLT Driver] class_create failed\n");
			err = PTR_ERR(blt_class);
			blt_class = NULL;
			break;
		}
		
		blt_device = device_create(blt_class, NULL, MKDEV(major_no, minor_no), NULL, BLT_DEVICE_FILE_NAME);
		if (IS_ERR(blt_device)) {
			printk(KERN_ERR "[BLT Driver] device_create failed\n");
			err = PTR_ERR(blt_device);
			blt_device = NULL;
			break;
		}
#endif	// #ifdef BLT_AUTOMATIC_CREATE_DEVICE_FILE
	}
	while (0);
	
	if (err) {
		blt_cleanup ();
	}
	
	return err;
}

static void __exit w55fa92_blt_exit(void)
{
	blt_cleanup();
}

module_init(w55fa92_blt_init);
module_exit(w55fa92_blt_exit);

MODULE_DESCRIPTION("W55FA92 BLT driver");
MODULE_LICENSE("GPL");

/* favc_module.h */
#ifndef _FAVCMODULE_H_
#define _FAVCMODULE_H_

#ifdef LINUX_ENV
#include <linux/module.h>
#include <linux/version.h>

#include <linux/init.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/delay.h>
#include <linux/poll.h>
#include <linux/string.h>
#include <linux/ioport.h>       /* request_region */
#include <linux/interrupt.h>    /* mark_bh */
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <linux/pci.h>
#include <asm/io.h>
#include <asm/page.h>
#include <linux/mm.h>
#include <asm/mach/map.h> //lichun@add
#endif

//#include <mach/platform/spec.h>
//#include <mach/fmem.h>
#define FAVC_DECODER_MINOR    22 //(10,20)
#define FAVC_ENCODER_MINOR    23 //(10,21)

#ifdef CONFIG_ENABLE_ENCODER_ONLY
#define SUPPORT_DECODER_DEFAULT_YES 0
#define SUPPORT_ENCODER_DEFAULT_YES 1
#endif
#ifdef CONFIG_ENABLE_DECODER_ONLY
#define SUPPORT_DECODER_DEFAULT_YES 1
#define SUPPORT_ENCODER_DEFAULT_YES 0
#endif
#ifdef CONFIG_ENABLE_CODEC_BOTH
#define SUPPORT_DECODER_DEFAULT_YES 1
#define SUPPORT_ENCODER_DEFAULT_YES 1
#endif


#ifdef LINUX_ENV
int favc_encoder_open(struct inode *inode, struct file *filp);
int favc_encoder_ioctl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg);
int favc_encoder_mmap(struct file *file, struct vm_area_struct *vma);
int favc_encoder_release(struct inode *inode, struct file *filp);

int favc_decoder_open(struct inode *inode, struct file *filp);
int favc_decoder_ioctl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg);
int favc_decoder_mmap(struct file *file, struct vm_area_struct *vma);
int favc_decoder_release(struct inode *inode, struct file *filp);
#else //LINUX_ENV
int favc_encoder_open(void);
int favc_encoder_mmap(void);
int favc_encoder_release(void);

int favc_decoder_open(void);
int favc_decoder_mmap(void);
int favc_decoder_release(void);
int init_favc(void);
int favc_decoder_ioctl(void* handle, unsigned int cmd, void* arg);
int favc_encoder_ioctl(void* handle, unsigned int cmd, void* arg);
#endif //LINUX_ENV

#ifdef LINUX_ENV
//void *hkmalloc(size_t size, uint8_t alignment, uint8_t reserved_size);
//void hkfree(void *mem_ptr);
//void *hconsistent_alloc(uint32_t size, uint8_t align_size, uint8_t reserved_size, void **phy_ptr);
//void hconsistent_free(void * virt_ptr, void * phy_ptr);

//unsigned int user_va_to_pa(unsigned int addr);
//int check_continued(unsigned int addr,int size);
#endif //LINUX_ENV

unsigned int TimeOutCheck(unsigned int tick);

//#if 0
//#define F_DEBUG(fmt, args...) printk(KERN_ALERT "FAVC: " fmt, ## args)
//#else
//#define F_DEBUG(a...)
//#endif

#endif

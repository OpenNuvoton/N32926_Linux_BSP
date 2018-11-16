#ifdef LINUX_ENV

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/hdreg.h>	/* HDIO_GETGEO */
#include <linux/fs.h>
#include <linux/blkdev.h>
#include <linux/pci.h>
#include <linux/spinlock.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <linux/version.h>
#include <linux/interrupt.h>
#include <linux/time.h>
#include <linux/mm.h>
#include <linux/slab.h>

#include <mach/w55fa92_reg.h>
#include <mach/favc_avcodec.h>

#else

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "wblib.h"
#endif 

#include "memory.h"
#include "port.h"


void* nv_malloc(int size, int alignment)
{
#if 0
	void *ptr;
	ptr = (void *)h264_malloc(size, alignment);
	sysprintf("malloc at 0x%x, size = 0x%x(%d), alignment at %d\n",(unsigned int)ptr, size,size,alignment);	
	return ptr;
#else	
    return (void *)h264_malloc(size, alignment);
#endif    
}

int nv_free(void* ptr)
{
#if 0
	sysprintf("driver : free at 0x%x\n",(unsigned int)ptr);	
	h264_free(ptr);	
#else
	h264_free(ptr);
#endif	
	
	return 0;      
}




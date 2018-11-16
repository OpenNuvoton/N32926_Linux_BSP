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
#include <stdlib.h>
#include "wblib.h"
#endif

#include "memory.h"
#include "sequence.h"

#include "user_define.h"

//#define CACHE_BIT31 	0x80000000

#define ET_SIZE 300      //!< size of error text buffer
char errortext[ET_SIZE]; //!< buffer for error message for exit with error()

#define	MALLOC_DBG	0

void *h264_malloc(unsigned int size,int alignment)
{
	uint8_t *mem_ptr;
	uint8_t *tmp;

	if (!alignment) {
#ifdef LINUX_ENV	   
		if ((mem_ptr = (uint8_t *) kmalloc(size + 1,GFP_KERNEL)) != NULL) { 
#else
		if ((mem_ptr = (uint8_t *) malloc(size + 1)) != NULL) {
#endif		    
			//*mem_ptr = 0;
			*(uint8_t *)(CACHE_BIT31 | (unsigned int)mem_ptr) = 0;
			return (void *) mem_ptr++;
		}
	} else {
#ifdef LINUX_ENV
		if ((tmp = (uint8_t *) kmalloc(size + alignment, GFP_KERNEL)) != NULL) {
#else	    
		if ((tmp = (uint8_t *) malloc(size + alignment)) != NULL) {
#endif		    
		    
			mem_ptr = (uint8_t *) ((ptr_t) (tmp + alignment - 1) & (~(ptr_t) (alignment - 1)));
			if (mem_ptr == tmp)
				mem_ptr += alignment;
#if 1				
			*(mem_ptr - 1) = (uint8_t) (mem_ptr - tmp);
#if MALLOC_DBG		
		    Console_Printf("Mem Allocate 0x%x, size=0x%x, alignment to 0x%x\n", tmp,size,mem_ptr );
#endif				
#else			
			*(uint8_t *)(CACHE_BIT31 | (unsigned int)(mem_ptr-1)) = (uint8_t) (mem_ptr - tmp);
#if MALLOC_DBG		
		    Console_Printf("Mem Allocate 0x%x, size=0x%x, alignment to 0x%x\n", tmp,size,mem_ptr );
#endif				
#endif
			return (void *) mem_ptr;
		}
	}
	return NULL;

}

void h264_free(void *mem_ptr)
{
		
  if (mem_ptr)
  {
#if 1
	 //Console_Printf("Try free 0x%x, ",mem_ptr);
	 //Console_Printf("real at at=0x%x \n", (uint8_t *) mem_ptr - *((uint8_t *) mem_ptr - 1));
#ifdef LINUX_ENV
	kfree((uint8_t *) mem_ptr - *((uint8_t *) mem_ptr - 1));
#else	 
	free((uint8_t *) mem_ptr - *((uint8_t *) mem_ptr - 1));
#endif	
#else  
#if MALLOC_DBG		
	 Console_Printf("Try free 0x%x, ",mem_ptr);
	 Console_Printf("real at at=0x%x \n", (uint8_t *) mem_ptr - *((uint8_t *) (CACHE_BIT31 | (unsigned int)((uint8_t *)mem_ptr - 1))) );
#endif	  
    //free((uint8_t *) mem_ptr - *((uint8_t *) mem_ptr - 1));
	free((uint8_t *) mem_ptr - *((uint8_t *) (CACHE_BIT31 | (unsigned int)((uint8_t *)mem_ptr - 1))));    
#endif	
  }    
}

/*!
 ************************************************************************
 * \brief
 *    Exit program if memory allocation failed (using error())
 * \param where
 *    string indicating which memory allocation failed
 ************************************************************************
 */
/*
void h264_no_mem_exit(char *where)
{
   snprintf(errortext, ET_SIZE, "Could not allocate memory: %s",where);
   error (errortext, 100);
}
*/

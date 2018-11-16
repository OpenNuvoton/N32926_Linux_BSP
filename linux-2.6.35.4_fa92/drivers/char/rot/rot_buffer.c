/****************************************************************
 *                                                              *
 * Copyright (c) Nuvoton Technology Corp. All rights reserved.  *
 *                                                              *
 ****************************************************************/
#include <linux/module.h>#include <asm/io.h>
#include <linux/mm.h>
#include <linux/dma-mapping.h>
#include "rot.h"


//#define ERR_PRINTF			printk
//#define DBG_PRINTF(...)		
#if 0//__BOOT_MEM__
unsigned int get_rot_buffer_size(void)
{
	unsigned int width, height;
	unsigned int pixel_width;
	unsigned int size=0;

	width = CONFIG_ROTATION_WIDTH;
	height = CONFIG_ROTATION_HEIGHT;
	pixel_width = CONFIG_ROTATION_PIXEL_WIDTH;
	size = (width*height+4096)*pixel_width;		
	return size;
}

EXPORT_SYMBOL(get_rot_buffer_size);
#endif
unsigned int w55fa92_rot_v, w55fa92_rot_p;

uint32_t dmamalloc_phy(unsigned long size)
{
	unsigned long adr;
	unsigned int i;
	int buf;
	unsigned long total_size;

	total_size = size;
	total_size = PAGE_ALIGN(total_size);
	DBG_PRINTF("ROR buf size=%x", (UINT32)total_size);
	w55fa92_rot_v = (u32)dma_alloc_writecombine(NULL/*dev*/, 
												total_size,
												&w55fa92_rot_p, 
												GFP_KERNEL);
	if (!w55fa92_rot_v){
		printk("Alloc memory size %d fail: err = 0x%x\n", (int32_t)total_size, (int32_t)w55fa92_rot_v);
		return 0;
	}
	memset((void *)w55fa92_rot_v, 0, total_size); 					/* Clear the ram out, no junk to the user */
	adr = (unsigned long) w55fa92_rot_v;
		
	while (total_size > 0) {
		SetPageReserved(vmalloc_to_page((void *)adr));
		adr += PAGE_SIZE;
		total_size -= PAGE_SIZE;
	}
	return w55fa92_rot_p;
}

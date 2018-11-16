/* jpegcodec.c
 *
 * Copyright (c) 2008 Nuvoton technology corporation
 * All rights reserved.
 * <clyu2@nuvoton.com>
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/list.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/mm.h>
#include <linux/dma-mapping.h>
#include <linux/pagemap.h>
#include <linux/interrupt.h>
#include <linux/smp_lock.h>
#include <linux/vmalloc.h>
#include <linux/init.h>
#include <linux/spinlock.h>
#include <linux/videodev.h>
#include <asm/cacheflush.h>
#include <asm/io.h>

#include <linux/clk.h>
#include <mach/w55fa92_reg.h>
#include <mach/jpegcodec.h>
#include <mach/drvjpeg.h>

#include <linux/videodev2.h>
#include <media/v4l2-device.h>

#include "jpegpriv.h"

#define JPEG_CMD_NONE                0
#define JPEG_CMD_YUV422_TO_RGB555    1
#define JPEG_CMD_YUV422_TO_RGB565    2
#define JPEG_CMD_EXIT                3

static u32 jpeg_command = JPEG_CMD_NONE;

static DECLARE_COMPLETION(jpeg_thread_exit);
static DECLARE_WAIT_QUEUE_HEAD(jpeg_wq);
static DECLARE_WAIT_QUEUE_HEAD(jpegd_wq); // this is used by kernel thread
static int jpeg_nr = -1;
//static struct semaphore jpeg_lock;
static atomic_t jpeg_avail = ATOMIC_INIT(1);

module_param(jpeg_nr, int, 0);


jpeg_priv_t jpeg_priv;
static u32 jpeg_align_width,jpeg_width,g_JPEG_RAW_SIZE = 0, enc_reserved_size = 0, TotalDataSize = 0,JPEG_RAW_SIZE_DOW = 0, total_data_row = 0, current_data_row = 0, enc_stride = 0, TotalBufferSize = 0, enc_buffer_from_user = 0, enc_yaddr_from_user = 0, enc_uaddr_from_user = 0, enc_vaddr_from_user = 0;
static u32 enc_thumbnail= 0,enc_reserved =0;
static u32 jpeg_thumbnail_size = 0;
static u32 jpeg_thumbnail_bitstreamsize = 0;
static u32 Windec_width,  Windec_height;

static u32 jpeg_thumbnail_offset = 0;
/* Default Quantization-Table 0 ~ 2 */
__u8 g_au8QTable0[64] = { 0x06, 0x04, 0x04, 0x05, 0x04, 0x04, 0x06, 0x05,
                      0x05, 0x05, 0x06, 0x06, 0x06, 0x07, 0x08, 0x0E,
                      0x09, 0x08, 0x08, 0x08, 0x08, 0x11, 0x0C, 0x0D,
                      0x0A, 0x0E, 0x14, 0x11, 0x15, 0x14, 0x13, 0x11,
                      0x13, 0x13, 0x16, 0x18, 0x1F, 0x1A, 0x16, 0x17,
                      0x1D, 0x17, 0x13, 0x13, 0x1B, 0x25, 0x1B, 0x1D,
                      0x20, 0x21, 0x23, 0x23, 0x23, 0x15, 0x1A, 0x26,
                      0x29, 0x26, 0x22, 0x28, 0x1F, 0x22, 0x23, 0x21 },
      g_au8QTable1[64] = { 0x06, 0x06, 0x06, 0x08, 0x07, 0x08, 0x10, 0x09,
                      0x09, 0x10, 0x21, 0x16, 0x13, 0x16, 0x21, 0x21,
                      0x21, 0x21, 0x21, 0x21, 0x21, 0x21, 0x21, 0x21,
                      0x21, 0x21, 0x21, 0x21, 0x21, 0x21, 0x21, 0x21,
                      0x21, 0x21, 0x21, 0x21, 0x21, 0x21, 0x21, 0x21,
                      0x21, 0x21, 0x21, 0x21, 0x21, 0x21, 0x21, 0x21,
                      0x21, 0x21, 0x21, 0x21, 0x21, 0x21, 0x21, 0x21,
                      0x21, 0x21, 0x21, 0x21, 0x21, 0x21, 0x21, 0x21 },
      g_au8QTable2[64] = { 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03,
                      0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03,
                      0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03,
                      0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03,
                      0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03,
                      0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03,
                      0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03,
                      0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03 };

__u8 g_au8QTableUser0[64] = { 0x06, 0x04, 0x04, 0x05, 0x04, 0x04, 0x06, 0x05,
                      0x05, 0x05, 0x06, 0x06, 0x06, 0x07, 0x08, 0x0E,
                      0x09, 0x08, 0x08, 0x08, 0x08, 0x11, 0x0C, 0x0D,
                      0x0A, 0x0E, 0x14, 0x11, 0x15, 0x14, 0x13, 0x11,
                      0x13, 0x13, 0x16, 0x18, 0x1F, 0x1A, 0x16, 0x17,
                      0x1D, 0x17, 0x13, 0x13, 0x1B, 0x25, 0x1B, 0x1D,
                      0x20, 0x21, 0x23, 0x23, 0x23, 0x15, 0x1A, 0x26,
                      0x29, 0x26, 0x22, 0x28, 0x1F, 0x22, 0x23, 0x21 },
      g_au8QTableUser1[64] = { 0x06, 0x06, 0x06, 0x08, 0x07, 0x08, 0x10, 0x09,
                      0x09, 0x10, 0x21, 0x16, 0x13, 0x16, 0x21, 0x21,
                      0x21, 0x21, 0x21, 0x21, 0x21, 0x21, 0x21, 0x21,
                      0x21, 0x21, 0x21, 0x21, 0x21, 0x21, 0x21, 0x21,
                      0x21, 0x21, 0x21, 0x21, 0x21, 0x21, 0x21, 0x21,
                      0x21, 0x21, 0x21, 0x21, 0x21, 0x21, 0x21, 0x21,
                      0x21, 0x21, 0x21, 0x21, 0x21, 0x21, 0x21, 0x21,
                      0x21, 0x21, 0x21, 0x21, 0x21, 0x21, 0x21, 0x21 };


#if 1
unsigned int jpeg_v;
#else
extern unsigned int jpeg_v;
#endif
/*******************************/
/* Memory management functions */
/*******************************/
#if 0
static void *jpeg_dmamalloc(unsigned long size)
{
	jpeg_priv_t *priv = (jpeg_priv_t *)&jpeg_priv;
	void *mem;
	unsigned long adr;

	size = PAGE_ALIGN(size);
	//printk("jpeg codec mmap size=%d\n", size);
	priv->vaddr = jpeg_v;//dma_alloc_writecombine(NULL/*dev*/, size,&priv->paddr, GFP_KERNEL);	
	//priv->paddr = CONFIG_JPEG_CODEC_BUFFER_ADDR;
 	priv->paddr = kmalloc(sizeof(__u8*) * CONFIG_JPEG_CODEC_BUFFER_SIZE, GFP_KERNEL);

	//printk("jpeg priv->paddr=%x,priv->vaddr=%x\n", priv->paddr, priv->vaddr);
	if (!priv->vaddr)
		return NULL;

	memset(priv->vaddr, 0, size); /* Clear the ram out, no junk to the user */

#if 0  // There's no where to be swap out.
	adr = (unsigned long) priv->vaddr;
	while (size > 0) {
		SetPageReserved(vmalloc_to_page((void *)adr));
		adr += PAGE_SIZE;
		size -= PAGE_SIZE;
	}
#endif

	return (void *)priv->vaddr;
}
#endif
// TESTTEST
static void jpegcodec_release_priv (jpeg_priv_t *priv)
{	
	if (priv->pages) {
		__u32 i;
						
		for (i = 0; i < priv->nr_pages; i ++) {
			page_cache_release (priv->pages[i]);
		}
		priv->nr_pages = 0;
		
		kfree(priv->pages);
		priv->pages = NULL;
	}
}

static int jpegcodec_open(struct file *file)
{
	struct video_device *dev = video_devdata(file);
	jpeg_priv_t *priv = (jpeg_priv_t *)dev->priv;
	
    /*	Critical section */
    if (! atomic_dec_and_test(&jpeg_avail)) {
        atomic_inc(&jpeg_avail);
        return -EBUSY;
    }
    
	struct clk *clk = clk_get(NULL, "jpg");

	BUG_ON(IS_ERR(clk));
	clk_enable(clk);

	priv->file = file;
	//JPEG Open
	w55fa92_Jpeg_Open ();
	
	//JPEG Init
	w55fa92_Jpeg_Init();
	enc_reserved_size = 0;
	enc_reserved = 0;
	enc_buffer_from_user = 0;
	jpeg_thumbnail_size = 0;
	enc_thumbnail = 0;
	priv->decopw_en = 0;
	priv->decopw_tcount = 0;
	priv->decopw_end = 0;	
	priv->vaddr_src = 0;
	priv->vaddr_dst = 0;
	priv->paddr_src = 0;
	priv->paddr_dst = 0;	
	current_data_row = 0;
	enc_stride = 0;
	/* Use the default Quantization-table 0, Quantization-table 1 */
	w55fa92_Jpeg_SetQTAB(g_au8QTable0,g_au8QTable1, 0, 2);
	//for Encode end
 	
 	// TESTTEST
 	priv->nr_pages = 0;
	return 0;
}
static int jpegcodec_close(struct file *file)
{
	struct video_device *dev = video_devdata(file);
	jpeg_priv_t *priv = (jpeg_priv_t *)dev->priv;

	struct clk *clk = clk_get(NULL, "jpg");

	jpeg_priv.state = JPEG_CLOSED;
	outp32( REG_JMCR, ((inp32(REG_JMCR) & ~(ENG_RST | JPG_EN)) | ENG_RST) );
	outp32( REG_JMCR, inp32(REG_JMCR) & ~(ENG_RST | JPG_EN) );

	_DRVJPEG_INT_DISABLE(DRVJPEG_DEC_INTE | DRVJPEG_DER_INTE | DRVJPEG_DHE_INTE);
	_DRVJPEG_CLEAR_INT(DRVJPEG_DEC_INTS | DRVJPEG_DER_INTS | DRVJPEG_DHE_INTS);
	//w55fa92_Jpeg_Close ();

	clk_disable(clk);
	// TESTTEST
	if(priv)
		jpegcodec_release_priv (priv);

	atomic_inc(&jpeg_avail);
		
	return 0;
}
/*
* read information after encode/decode finished
* don't support read data
*/static ssize_t jpegcodec_read(struct file *file, char __user *buf,
		      size_t count, loff_t *ppos)
{
	struct video_device *dev = video_devdata(file);
	jpeg_priv_t *priv = (jpeg_priv_t *)dev->priv;
	int nonblock = file->f_flags & O_NONBLOCK;
	int ret;
	int size, i;
	jpeg_info_t *jpeginfo;
	
	if(priv->encode && priv->buffercount > 1)
	{
		if(priv->bufferend_bak == priv->bufferend)
		{
			if(nonblock)
			{
				ret = -EAGAIN;
				goto out1;
			}
			wait_event_interruptible(jpeg_wq, (priv->bufferend_bak != priv->bufferend));
		}
	}
	else
	{
		if(!IS_FINISHED(priv->state) && priv->state != JPEG_MEM_SHORTAGE && priv->state != JPEG_DECODE_PARAM_ERROR)
		{
			if(nonblock)
			{
				ret = -EAGAIN;
				goto out1;
			}
			wait_event_interruptible(jpeg_wq, (IS_FINISHED(priv->state) || (priv->state == JPEG_MEM_SHORTAGE) || (priv->state == JPEG_DECODE_PARAM_ERROR)));
		}
	}
	
//ok:
	down(&priv->lock);
	
	priv->bufferend_bak = priv->bufferend;
	

	{
		size = sizeof(jpeg_info_t) + sizeof(__u32) * priv->buffercount;
		jpeginfo = kmalloc(size, GFP_KERNEL);
		if(!jpeginfo)
		{
			ret = -ENOMEM;
			goto out1;
		}
		if(count != size)
		{
		  //printk("request %d, but should be %d\n", count, size);
			ret = -EINVAL;
			goto out;
		}
		jpeginfo->yuvformat = priv->yuvformat;
		if(priv->encode)
		{ 
			jpeginfo->width = priv->width;
		}
		else
		{
			jpeginfo->dec_stride =  priv->width;
	
			jpeginfo->width = priv->width - priv->dec_stride;
		}	
			
    		jpeginfo->height = priv->height;
		jpeginfo->state = priv->state;
  		//printk("jpeginfo.image_size=%x, %d\n", jpeginfo->image_size, priv->buffercount);
	    if(IS_DECODED(priv->state))
		{

			if(priv->decode_output_format != DRVJPEG_DEC_PRIMARY_PLANAR_YUV)
			{
				i = priv->height*priv->width*priv->byte2pixel;
				i /= 2;//byte2pixel is twice of actual bytes
				jpeginfo->image_size[0] = i;	
			}
			else
				jpeginfo->image_size[0]  = g_JPEG_RAW_SIZE;
			
		}
		else
		{
	    	for(i = 0; i < priv->buffercount; i++)
	    		jpeginfo->image_size[i] = priv->image_size[i];
	    }
	    jpeginfo->bufferend = priv->bufferend;
	    if(priv->decopw_en)
		    jpeginfo->image_size[0] = JPEG_RAW_SIZE_DOW;

	    if (copy_to_user(buf, (void*)jpeginfo, size)) {
	      //printk("copy_to_user error\n");
			ret = -EFAULT;
			goto out;
		}
		*ppos = sizeof(jpeginfo);
	}


	ret = size;
out:
    kfree(jpeginfo);
out1:
	up(&priv->lock);
	return ret;
}

/*
* write data for encode/decode
*/
static ssize_t jpegcodec_write(struct file *file, const char __user *buf, 
			  size_t count, loff_t *ppos)
{
	struct video_device *dev = video_devdata(file);
	jpeg_priv_t *priv = (jpeg_priv_t *)dev->priv;
	int nonblock = file->f_flags & O_NONBLOCK;
	int ret;
	__u32	address;
	
	if(priv->encode == 0 && priv->decInWait_buffer_size != 0 && priv->decInWait_buffer_empty == 1)
	{
		down(&priv->lock);
		
		if(priv->decInWait_counter == 0)	
		{
			address  = priv->vaddr;
			//printk("Counter %d Address 0x%X Len %d\n",priv->decInWait_counter, address,count);	
		}
		else
		{
			if((priv->decInWait_counter & 0x1) == 0)
			{
				address  = priv->vaddr + priv->decInWait_buffer_size / 2;	
				//printk("Counter %d Address 0x%X Len %d\n",priv->decInWait_counter, address,count);	
			}
			else
			{
				address  = priv->vaddr;
				//printk("Counter %d Address 0x%X Len %d\n",priv->decInWait_counter, address,count);	
			}
		}

		if (copy_from_user((void *)address, (void *)buf, count)) {
		  //printk("copy_to_user\n");
			ret = -EFAULT;
			goto out;
		}
		
	}
	else	
	{
		//printk("jpegcodec_write\n");
		if(!IS_FINISHED(priv->state))
		{
			if(nonblock)
			{
				ret = -EAGAIN;
				printk("jpegcodec_write Out\n");
				goto out;
			}
			wait_event_interruptible(jpeg_wq, IS_FINISHED(priv->state));
		}
		down(&priv->lock);
		if (copy_from_user((void *)priv->vaddr, (void *)buf, count)) {
		  //printk("copy_to_user\n");
			ret = -EFAULT;
			goto out;
		}
	}
	up(&priv->lock);

	ret = count;
out:
	return ret;
}


static int jpegcodec_mmap (struct file *file, struct vm_area_struct *vma)
{	
	struct video_device *dev = video_devdata(file);
	jpeg_priv_t *priv = (jpeg_priv_t *)dev->priv;
	//unsigned long start = vma->vm_start;
	unsigned long size  = vma->vm_end-vma->vm_start;
	unsigned long pos;
	
	//if (size > (((priv->buffersize * priv->buffercount) + PAGE_SIZE - 1) & ~(PAGE_SIZE - 1)))
	//	return -EINVAL;
	if(size > CONFIG_JPEG_CODEC_BUFFER_SIZE)
		return -EINVAL;
	
	//pos = jpeg_dmamalloc(file, CONFIG_JPEG_CODEC_BUFFER_SIZE);
	//if (!pos)
	//	return -ENOMEM;
	pos = priv->vaddr;
	priv->mmap_bufsize = size;
	
	//printk("jpegcodec_mmap priv->buffercount=%d size = %d, start=%x\n", priv->buffercount, size, start);
#if 0	
	while (size > 0) {
		page = vmalloc_to_pfn((void *)pos);
		if (remap_pfn_range(vma, start, page, PAGE_SIZE, PAGE_SHARED))
			return -EAGAIN;

		start += PAGE_SIZE;
		pos += PAGE_SIZE;
		if (size > PAGE_SIZE)
			size -= PAGE_SIZE;
		else
			size = 0;
	}
#else
    if(remap_pfn_range(vma, vma->vm_start, priv->paddr >> PAGE_SHIFT, size, vma->vm_page_prot) < 0)
        return(-EAGAIN);
    
#endif	
	jpeg_priv.state = JPEG_IDLE;
	return 0;
	
}


static int jpegcodec_ioctl(struct file *file,
				 unsigned int cmd, void *arg)
{
	struct video_device *dev = video_devdata(file);
	jpeg_priv_t *priv = (jpeg_priv_t *)dev->priv;
	int ret = 0;
	jpeg_param_t param;
	__u32 output_format,u32BufferSize;
	int nr_pages;
	int ret1;	

	switch (cmd) {
		case JPEG_TRIGGER:
			flush_cache_all();
			file->f_pos = 0;
			if(priv->encode)
			{
				jpeg_thumbnail_bitstreamsize = 0;
				jpeg_thumbnail_offset = 0;
				priv->state = JPEG_ENCODING;
				if(enc_stride)
				{
					_DRVJPEG_SET_YSTRIDE(enc_stride);
					outp32(REG_JUSTRIDE, enc_stride/2);
					outp32(REG_JVSTRIDE, enc_stride/2);
				}
				if(enc_buffer_from_user == 0)
				{
					if(priv->paddr_src == 0)
					{
						  //printk("jpeg encode source addr:%x\n", priv->paddr);
						_DRVJPEG_SET_YADDR(priv->paddr);
					}
					if(priv->paddr_dst == 0)
					{
						  //printk("jpeg encode dst addr:%x\n", priv->paddr + priv->src_bufsize);
						_DRVJPEG_SET_BITSTREAM_ADDR(priv->paddr + priv->src_bufsize);
					}
					if(priv->encode_source_format == DRVJPEG_ENC_SRC_PACKET)
						u32BufferSize = priv->encode_height * priv->encode_width * 2;
					else
					{
						if(priv->encode_image_format == DRVJPEG_ENC_PRIMARY_YUV422)
							u32BufferSize = priv->encode_height * priv->encode_width * 2;
						else
							u32BufferSize = priv->encode_height * priv->encode_width * 3/2;
					}

					if(priv->paddr_src == 0 && (u32BufferSize > CONFIG_JPEG_CODEC_BUFFER_SIZE))
					{
						printk("Config Buffer size is 0x%X\nNeed Buffer size is 0x%X\n",CONFIG_JPEG_CODEC_BUFFER_SIZE, u32BufferSize);
						priv->state = JPEG_MEM_SHORTAGE;
						w55fa92_Jpeg_Init();
						enc_reserved_size = 0;
						enc_reserved = 0;
						enc_buffer_from_user = 0;
						jpeg_thumbnail_size = 0;
						jpeg_thumbnail_bitstreamsize = 0;
						jpeg_thumbnail_offset = 0;	
						enc_thumbnail = 0;
						priv->decopw_en = 0;
						priv->decopw_tcount = 0;
						priv->decopw_end = 0;
						return -JPEG_MEM_SHORTAGE;
						break;
					}
				}
				else
				{
					_DRVJPEG_SET_BITSTREAM_ADDR(priv->paddr + priv->src_bufsize);
					/* add for videoin */
					_DRVJPEG_SET_YADDR(enc_yaddr_from_user);
					_DRVJPEG_SET_UADDR(enc_uaddr_from_user);
					_DRVJPEG_SET_VADDR(enc_vaddr_from_user);
				}
				if(enc_thumbnail && enc_buffer_from_user)
				{
					__u8 *u8Addr;
					u16 u16ratioH,u16ratioW;
					u32 scaled_height, scaled_width;
					if(enc_reserved_size)
						enc_reserved = 1;
					u8Addr = (__u8 *) priv->vaddr;
					*(u8Addr + 0) = 0xFF;
					*(u8Addr + 1) = 0xD8;
					*(u8Addr + 2) = 0xFF;
					*(u8Addr + 3) = 0xE1;
					if(enc_reserved_size == 0)
					{
						enc_reserved_size = 8;
						*(u8Addr + 6) = 0xFF;
						*(u8Addr + 7) = 0xFF;							
					}
					else
					{
						int i,TMP;
						enc_reserved_size = enc_reserved_size + 6;
						TMP = enc_reserved_size;						
						enc_reserved_size = (enc_reserved_size + 0x03) & ~0x03;	
						TMP = enc_reserved_size  - TMP;
						u8Addr = (__u8 *) priv->vaddr + enc_reserved_size;
						
						for(i = 0; i< TMP;i++)
							*(u8Addr - (i+1)) = 0xFF;

					}
					if(jpeg_thumbnail_size == DRVJPEG_ENC_THUMBNAIL_QVGA)
					{
						scaled_height = 240;
						scaled_width = 320;
					}
					else
					{
						scaled_height = 120;
						scaled_width = 160;
					}
					_DRVJPEG_SET_BITSTREAM_ADDR(priv->paddr + enc_reserved_size);
					//printk("Thumbnail start address 0x%X\n",priv->vaddr + enc_reserved_size);
					if(enc_reserved)
						jpeg_thumbnail_offset = enc_reserved_size - 12;
					else
					jpeg_thumbnail_offset = enc_reserved_size;
						/* Downscale */

					if(w55fa92_Jpeg_CalScalingFactor(
							DRVJPEG_ENC_PLANAR_DOWNSCALE_MODE,	//Up / Down Scaling
							priv->encode_height,					//Original Height
							priv->encode_width,					//Original Width
							scaled_height,		//Scaled Height
							scaled_width,		//Scaled Width
							&u16ratioH,					//Horizontal Ratio
							&u16ratioW					//Vertical Ratio
					) != 0)
					{
						priv->state = JPEG_ENCODE_PARAM_ERROR;
						w55fa92_Jpeg_Init();
						enc_reserved_size = 0;
						enc_reserved = 0;
						enc_buffer_from_user = 0;
						jpeg_thumbnail_size = 0;
						jpeg_thumbnail_bitstreamsize = 0;
						jpeg_thumbnail_offset = 0;
						enc_thumbnail = 0;
						priv->decopw_en = 0;
						priv->decopw_tcount = 0;
						priv->decopw_end = 0;
					  	wake_up_interruptible(&jpeg_wq);
					        return 1;
					}
					else
					{
							
						w55fa92_Jpeg_SetScalingFactor(DRVJPEG_ENC_PLANAR_DOWNSCALE_MODE, u16ratioH, u16ratioW);
						w55fa92_Jpeg_SetDimension(scaled_height,scaled_width);
						outp32(REG_JSRCH, priv->encode_height);
					}   
				}
				else
				{
					if(enc_reserved_size != 0)
					{
						__u32 u32Tmp;
						__u8 *u8Addr;
						u32Tmp = enc_reserved_size + 4;
						if(u32Tmp % 2)
							u32Tmp++;
						if((u32Tmp % 4) == 0)
							u32Tmp+=2;		
						if(u32Tmp >= 0xFFFF)
							u32Tmp = 65534;				
						outp32(REG_JPSCALU, inp32(REG_JPSCALU) | A_JUMP);	
						outp32(REG_JRESERVE, u32Tmp);	
						if(enc_buffer_from_user == 0)					
							u8Addr = (__u8 *) priv->vaddr + priv->src_bufsize;		
						else
							u8Addr = (__u8 *) priv->vaddr;
						*(u8Addr + 2) = 0xFF;
						*(u8Addr + 3) = 0xE0;
						*(u8Addr + 4) = ((u32Tmp- 4) & 0xFF00) >> 8;
						*(u8Addr + 5) = (u32Tmp - 4) & 0xFF;
					}
				}

			    	if(priv->scale && enc_thumbnail != 1)
			    	{
			    		u16 u16ratioH,u16ratioW;
			    		
					if(priv->encode_height <= priv->scaled_height && priv->encode_width <= priv->scaled_width)
					{
	
						if(w55fa92_Jpeg_CalScalingFactor(
								DRVJPEG_ENC_UPSCALE_MODE,	//Up / Down Scaling
								priv->encode_height,					//Original Height
								priv->encode_width,					//Original Width
								priv->scaled_height,		//Scaled Height
								priv->scaled_width,		//Scaled Width
								&u16ratioH,					//Horizontal Ratio
								&u16ratioW					//Vertical Ratio
						) != 0)
						{
						        priv->state = JPEG_ENCODE_PARAM_ERROR;
						         wake_up_interruptible(&jpeg_wq);
						         return 1;
						}
						else
						{
							w55fa92_Jpeg_SetScalingFactor(DRVJPEG_ENC_UPSCALE_MODE, u16ratioH, u16ratioW);
							w55fa92_Jpeg_SetDimension(priv->scaled_height,priv->scaled_width);
							outp32(REG_JSRCH, priv->encode_height);

						}      
					}
					else if(priv->encode_height > priv->scaled_height && priv->encode_width > priv->scaled_width)				
					{
						if(w55fa92_Jpeg_CalScalingFactor(
								DRVJPEG_ENC_PLANAR_DOWNSCALE_MODE,	//Up / Down Scaling
								priv->encode_height,					//Original Height
								priv->encode_width,					//Original Width
								priv->scaled_height,		//Scaled Height
								priv->scaled_width,		//Scaled Width
								&u16ratioH,					//Horizontal Ratio
								&u16ratioW					//Vertical Ratio
						) != 0)
						{
						        priv->state = JPEG_ENCODE_PARAM_ERROR;
						         wake_up_interruptible(&jpeg_wq);
						         return 1;
						}
						else
						{
							w55fa92_Jpeg_SetScalingFactor(DRVJPEG_ENC_PLANAR_DOWNSCALE_MODE, u16ratioH, u16ratioW);	
							w55fa92_Jpeg_SetDimension(priv->scaled_height,priv->scaled_width);
							outp32(REG_JSRCH, priv->encode_height);
						}  
					}
					else
					{
						priv->state = JPEG_ENCODE_PARAM_ERROR;
						wake_up_interruptible(&jpeg_wq);
						return 1;
					}
			    	}
				/* Trigger JPEG decoder */
				//printk("Encode Trigger\n");
	    			w55fa92_Jpeg_Trigger();					
			}
			else
			{
				priv->state = JPEG_DECODING;
				priv->decInWait_buffer_empty = 0;

				if(priv->paddr_src == 0)
					_DRVJPEG_SET_BITSTREAM_ADDR(priv->paddr);
				if(priv->paddr_dst == 0)
					_DRVJPEG_SET_YADDR(priv->paddr + priv->src_bufsize);

				priv->decopw_yaddr = inp32(REG_JYADDR0);
				priv->decopw_bitaddr = inp32(REG_JIOADDR0); 

				if(priv->windec_en)
				{
					outp32(REG_JWINDEC0, (priv->windec_mcuy_start << 16) | priv->windec_mcux_start);
    					outp32(REG_JWINDEC1, (priv->windec_mcuy_end << 16) | priv->windec_mcux_end); 
					outp32(REG_JMCR, inp32(REG_JMCR) | WIN_DEC);
					Windec_width = 16 * ( priv->windec_mcux_end - priv->windec_mcux_start + 1);
					Windec_height = 16 * ( priv->windec_mcuy_end - priv->windec_mcuy_start + 1);
    					outp32(REG_JWINDEC2, priv->windec_stride );  /* Stride */  
				}

				outp32(REG_JMCR, JPG_EN | inp32(REG_JMCR));    

				outp32(REG_JMCR, ~JPG_EN & inp32(REG_JMCR));
			}

			break;
		case JPEG_S_PARAM:
			//JPEG Init
			w55fa92_Jpeg_Init();
			priv->decopw_en = 0;
			priv->decopw_tcount = 0;
			priv->decopw_end = 0;
			priv->vaddr_src = 0;
			priv->vaddr_dst = 0;
			priv->paddr_src = 0;
			priv->paddr_dst = 0;		
 			// TESTTEST
 			//priv->nr_pages = 0;


			priv->state = JPEG_IDLE;

			if (copy_from_user((void*)&param, (void *)arg, sizeof(param))) {
			  //printk("copy_from_user error\n"./j	_);
				ret = -EFAULT;
				break;
			}


			//set decode/encode
			if(param.encode)//0 decode; 1 encode
			{
				priv->encode = 1;
				if(param.encode_width && param.encode_height)
				{
					priv->encode_width = param.encode_width;
					priv->encode_height = param.encode_height;
					priv->scale = param.scale;
					if(param.scale)
					{						
						priv->scaled_width = param.scaled_width ;
						priv->scaled_height = param.scaled_height;
					}
					
					_DRVJPEG_SET_YSTRIDE(priv->encode_width);		
				    _DRVJPEG_SET_USTRIDE(priv->encode_width/2);
				    _DRVJPEG_SET_VSTRIDE(priv->encode_width/2);
				    /* Primary Encode Image Width / Height */
    				w55fa92_Jpeg_SetDimension(priv->encode_height,priv->encode_width);  	
				    //Set Encode Source Image Height    
				    _DRVJPEG_SET_SOURCE_IMAGE_HEIGHT(priv->encode_height);
				}
				//qadjust: the larger the better quality[2-16](0.25Q, 0.5Q, 0.75Q, Q, 1.25Q, 1.5Q, 1.75Q, 2Q, 2.25Q, 2.5Q, 2.75Q, 3Q, 3.25Q, 3.5Q, 3.75Q) 
				//qscaling: the smaller the better quality[1-16]

				if(param.qadjust && param.qscaling)
				{
					priv->qadjust = param.qadjust;
					priv->qscaling = param.qscaling;
					w55fa92_Jpeg_AdjustQTAB(DRVJPEG_ENC_PRIMARY,param.qadjust,param.qscaling);
				}
				/* Encode mode, encoding primary image, YUV 4:2:2/4:2:0 */
				if(/*param.encode_source_format&&*/param.encode_image_format)
				{				
					if(param.encode_source_format == DRVJPEG_ENC_SRC_PLANAR)
					{
						_DRVJPEG_SET_UADDR(priv->paddr+ priv->encode_width * priv->encode_height);

						if(param.encode_image_format == DRVJPEG_ENC_PRIMARY_YUV422)
							_DRVJPEG_SET_VADDR(priv->paddr+ priv->encode_width * priv->encode_height*3/2);
						else
							_DRVJPEG_SET_VADDR(priv->paddr+ priv->encode_width * priv->encode_height*5/4);
					}
					priv->encode_image_format = param.encode_image_format;
					priv->encode_source_format = param.encode_source_format;
				}
				else//default
				{
					priv->encode_image_format = DRVJPEG_ENC_PRIMARY_YUV422;
					priv->encode_source_format = DRVJPEG_ENC_SRC_PACKET;
				}
				w55fa92_Jpeg_SetEncodeMode(priv->encode_source_format, priv->encode_image_format);
				/* Include Quantization-Table and Huffman-Table */
				_DRVJPEG_ENC_SET_HEADER_CONTROL(DRVJPEG_ENC_PRIMARY_QTAB | DRVJPEG_ENC_PRIMARY_HTAB); 
				
				/* Encode Complete Interrupt Enable and clear the Encode Complete Interrupt */
				_DRVJPEG_INT_ENABLE(DRVJPEG_ENC_INTE);
				_DRVJPEG_CLEAR_INT(DRVJPEG_ENC_INTS);

			}
			else
			{	
				priv->encode = 0;//decode
				priv->dec_stride = param.dec_stride;
				priv->scale = param.scale;
				priv->decInWait_buffer_empty = 1;
	 			priv->decInWait_counter = 0;
				priv->decopw_TargetBuffersize = param.decopw_TargetBuffersize;
				if(param.decopw_en)
				{						
					priv->decopw_en = 1;
					priv->decopw_vaddr = param.decopw_vaddr;
					//printk("priv->decopw_vaddr 0x%X\n", priv->decopw_vaddr);
					priv->decopw_tcount = 0;
					priv->decopw_end = 0;
					jpeg_width = param.dec_stride;
					priv->dec_stride = 0;	/* Disable Stride */
					priv->decopw_page_index = 0;
					priv->decopw_page_offset = 0;

					
					
					TotalBufferSize = priv->decopw_TargetBuffersize;
					//printk("TargetBuffersize 0x%X\n",TotalBufferSize);
					
					
					// TESTTEST
					jpegcodec_release_priv (priv);
					
					/* GET PAGES */
					if (priv->pages)
					{	
					//	printk("Free pages %d\n",priv->pages);
						kfree(priv->pages);
					}
					
					// Support non-page-aligned decode buffer.
					//nr_pages = (TotalBufferSize + PAGE_SIZE - 1) >> PAGE_SHIFT;
					nr_pages = (((priv->decopw_vaddr + TotalBufferSize + PAGE_SIZE - 1) & PAGE_MASK) - (priv->decopw_vaddr & PAGE_MASK)) >> PAGE_SHIFT;
					priv->decopw_page_offset = priv->decopw_vaddr & (PAGE_SIZE - 1);
					
					priv->pages = kmalloc(nr_pages * sizeof(struct page *), GFP_KERNEL);
					
					if (!priv->pages)
					{
						priv->state = JPEG_MEM_SHORTAGE;
						priv->decopw_en = 0;
						priv->decopw_tcount = 0;
						priv->decopw_end = 0;
						return -JPEG_MEM_SHORTAGE;
					}

					down_read(&current->mm->mmap_sem);
					ret1 = get_user_pages(current, current->mm, (unsigned long)priv->decopw_vaddr,
				     	nr_pages, 1, 0, priv->pages, NULL);
					up_read(&current->mm->mmap_sem);

					
					if (ret1 < nr_pages) {
						nr_pages = ret1;
						priv->state = JPEG_MEM_SHORTAGE;
						priv->decopw_en = 0;
						priv->decopw_tcount = 0;
						priv->decopw_end = 0;
        					return -JPEG_MEM_SHORTAGE;
					}

					// TESTTEST
					priv->nr_pages = ret1;
				}
				else
				{
					priv->decopw_en = 0;
				}

				if(param.windec_en)
				{	
					priv->windec_en = 1;
					priv->windec_mcux_start = param.windec_mcux_start;
					priv->windec_mcux_end = param.windec_mcux_end;
					priv->windec_mcuy_start = param.windec_mcuy_start;
					priv->windec_mcuy_end = param.windec_mcuy_end;
					priv->windec_stride = param.windec_stride;
				}
				else
					priv->windec_en = 0;

				priv->decInWait_buffer_size =  param.decInWait_buffer_size;
				if(param.scale)
				{					
					priv->scaled_width = param.scaled_width ;
					priv->scaled_height = param.scaled_height;
				}
				/* Decode mode */
				output_format = param.decode_output_format;
				priv->decode_output_format = param.decode_output_format;
				priv->convert = JPEG_CMD_NONE;
				    
				w55fa92_Jpeg_SetDecodeMode(output_format);
				if(output_format == DRVJPEG_DEC_PRIMARY_PACKET_RGB888)
					priv->byte2pixel = 8;//avoid decimal so twice of orignal
				else
					priv->byte2pixel = 4;//avoid decimal so twice of orignal
	
				/* Decode Complete /Decode Header End/Decode Error Interrupt Enable and clear the Decode Complete /Decode Header End/Decode Error Interrupt */
				if(priv->decInWait_buffer_size != 0)
				{
					_DRVJPEG_DEC_SET_INPUT_WAIT((priv->decInWait_buffer_size/2048));
					if(priv->decopw_en)
						_DRVJPEG_INT_ENABLE(DRVJPEG_DEC_INTE | DRVJPEG_DER_INTE | DRVJPEG_DHE_INTE | DRVJPEG_IPW_INTE|DRVJPEG_DOW_INTE);
					else
						_DRVJPEG_INT_ENABLE(DRVJPEG_DEC_INTE | DRVJPEG_DER_INTE | DRVJPEG_DHE_INTE | DRVJPEG_IPW_INTE);


				}
				else	
				{
					if(priv->decopw_en)
						_DRVJPEG_INT_ENABLE(DRVJPEG_DEC_INTE | DRVJPEG_DER_INTE | DRVJPEG_DHE_INTE | DRVJPEG_DOW_INTE);						
					else
						_DRVJPEG_INT_ENABLE(DRVJPEG_DEC_INTE | DRVJPEG_DER_INTE | DRVJPEG_DHE_INTE);
  						
				}
				_DRVJPEG_CLEAR_INT(DRVJPEG_DEC_INTS | DRVJPEG_DER_INTS | DRVJPEG_DHE_INTS | DRVJPEG_IPW_INTS | DRVJPEG_DOW_INTS);
			}
			if(param.buffersize)
				priv->buffersize = param.buffersize;
			if(param.buffercount)
				priv->buffercount = param.buffercount;
			//Set output Image Address
			if(param.vaddr_dst)
			{
				priv->vaddr_dst = param.vaddr_dst;
				priv->paddr_dst = param.paddr_dst;
				if(param.encode)
					_DRVJPEG_SET_BITSTREAM_ADDR(priv->paddr_dst);
				else
				{
					_DRVJPEG_SET_YADDR(priv->paddr_dst);
				}
				
			}
			//Set Bit stream Address
			if(param.vaddr_src)
			{
				priv->vaddr_src = param.vaddr_src;
				priv->paddr_src = param.paddr_src;
				if(param.encode){
					_DRVJPEG_SET_YADDR(priv->paddr_src);
					if(param.encode_source_format == DRVJPEG_ENC_SRC_PLANAR)
					{
						_DRVJPEG_SET_UADDR(priv->paddr_src+ priv->encode_width * priv->encode_height);

						if(param.encode_image_format == DRVJPEG_ENC_PRIMARY_YUV422)
							_DRVJPEG_SET_VADDR(priv->paddr_src+ priv->encode_width * priv->encode_height*3/2);
						else
							_DRVJPEG_SET_VADDR(priv->paddr_src+ priv->encode_width * priv->encode_height*5/4);
					}
		    			priv->src_bufsize = param.src_bufsize;	
				}
				else{
		    			_DRVJPEG_SET_BITSTREAM_ADDR(priv->paddr_src);
				}
		    	}
    			
			//if(param.src_bufsize)
				priv->src_bufsize = param.src_bufsize;
			if(param.dst_bufsize)
				priv->dst_bufsize = param.dst_bufsize;
			break;
		case JPEG_G_PARAM:
			param.vaddr_src = priv->vaddr_src;
			param.vaddr_dst = priv->vaddr_dst;
			param.paddr_src = priv->paddr_src;
			param.paddr_dst = priv->paddr_dst;
			param.decopw_vaddr = priv->decopw_vaddr;
			param.src_bufsize = priv->src_bufsize;
			param.dst_bufsize = priv->dst_bufsize;
			
			if (copy_to_user((void*)arg, (void *)&param, sizeof(param))) {
			  //printk("copy_to_user error\n");
					ret = -EFAULT;
					break;
			}
			break;
		case JPEG_STATE:
			{
				jpeg_state_t *state = (jpeg_state_t *)arg;
				*state = priv->state;
			}
			break;
		case JPEG_DECIPW_BUFFER_STATE:
			{
				__u32 *state = (__u32 *)arg;
				*state = priv->decInWait_buffer_empty;
			}
			break;
		case JPEG_G_DECIPW_BUFFER_SIZE:
			{	
				__u32 *state = (__u32 *)arg;	
				if(priv->decInWait_buffer_empty == 1)		
				{
					if(priv->decInWait_counter == 0)				
						*state = priv->decInWait_buffer_size;
					else
						*state = priv->decInWait_buffer_size / 2;
				}
				else
					*state = 0;	
			}
			break;
		case JPEG_DECODE_RESUME:
			_DRVJPEG_DEC_RESUME_INPUT_WAIT();
			priv->decInWait_buffer_empty = 0;
			break;
		case JPEG_G_INFO:
			{
				jpeg_info_t *jpeginfo;
				jpeginfo = kmalloc(sizeof(jpeg_info_t) + sizeof(__u32) * priv->buffercount, GFP_KERNEL);
					
				if(!jpeginfo)
					return -ENOMEM;
				jpeginfo->yuvformat = priv->yuvformat;
				jpeginfo->dec_stride = priv->dec_stride;
				jpeginfo->width = priv->width;
    				jpeginfo->height = priv->height;

				jpeginfo->state = priv->state;

    				jpeginfo->image_size[0] = priv->image_size[0];	
				
				if (copy_to_user((void*)arg, (void *)jpeginfo, sizeof(jpeg_info_t))) {
				  //printk("copy_to_user error\n");
					ret = -EFAULT;
				}
				kfree(jpeginfo);
				break;
			}
		case JPEG_GET_JPEG_BUFFER:
		{
			u32 buffersize = CONFIG_JPEG_CODEC_BUFFER_SIZE;
			if (copy_to_user((void*)arg, (void *)&buffersize, sizeof(buffersize))) {
			  //printk("copy_to_user error\n");
					ret = -EFAULT;
					break;
			}
			break;
		}
		case JPEG_GET_JPEG_BUFFER_PADDR:
		{
			u32 buffer = priv->paddr;
			if (copy_to_user((void*)arg, (void *)&buffer, sizeof(buffer))) {
			  //printk("copy_to_user error\n");
					ret = -EFAULT;
					break;
			}
			break;
		}	
		case JPEG_SET_ENCOCDE_RESERVED:
			enc_reserved_size = (u32)arg;		
			break;
		case JPEG_SET_ENC_THUMBNAIL:
			jpeg_thumbnail_size = (u32)arg;
			enc_thumbnail = 1;
			break;
		case JPEG_GET_ENC_THUMBNAIL_SIZE:		
			if (copy_to_user((void*)arg, (void *)&jpeg_thumbnail_bitstreamsize, sizeof(jpeg_thumbnail_bitstreamsize))) {
			  //printk("copy_to_user error\n");
					ret = -EFAULT;
					break;
			}
			break;
		case JPEG_GET_ENC_THUMBNAIL_OFFSET:		
			if (copy_to_user((void*)arg, (void *)&jpeg_thumbnail_offset, sizeof(jpeg_thumbnail_offset))) {
			  //printk("copy_to_user error\n");
					ret = -EFAULT;
					break;
			}
			break;
		case JPEG_FLUSH_CACHE:
			flush_cache_all();
			break;
		case JPEG_SET_ENC_STRIDE:
			enc_stride = (u32)arg;
			break;
		case JPEG_SET_ENC_USER_QTABLE0:
			if (copy_from_user((void*)g_au8QTableUser0, (void *)arg, 64 * sizeof(__u8))) {
			//  printk("copy_from_user error\n"./j	_);
				ret = -EFAULT;
				break;
			}			
			break;
		case JPEG_SET_ENC_USER_QTABLE1:
			if (copy_from_user((void*)g_au8QTableUser1, (void *)arg, 64 * sizeof(__u8))) {
			// printk("copy_from_user error\n"./j	_);
				ret = -EFAULT;
				break;
			}				
			break;
		case JPEG_ACTIVE_ENC_DEFAULTQTABLE:
			w55fa92_Jpeg_SetQTAB(g_au8QTable0,g_au8QTable1, 0, 2);
			break;
		case JPEG_ACTIVE_ENC_USER_QTABLE:
			w55fa92_Jpeg_SetQTAB(g_au8QTableUser0,g_au8QTableUser1, 0, 2);
			break;
		case JPEG_SET_ENC_USER_YADDRESS:
			enc_yaddr_from_user = (u32)arg;	
			enc_buffer_from_user++;
			break;
		case JPEG_SET_ENC_USER_UADDRESS:
			enc_uaddr_from_user = (u32)arg;	
			enc_buffer_from_user++;
			break;
		case JPEG_SET_ENC_USER_VADDRESS:
			enc_vaddr_from_user = (u32)arg;	
			enc_buffer_from_user++;
			break;
		default:
			return -ENOIOCTLCMD;
	}
	return 0;
}

static struct v4l2_file_operations jpegcodec_fops = {
	.owner =  THIS_MODULE,
	.read =   jpegcodec_read,
	.write =  jpegcodec_write,
	.poll =   NULL,
	.ioctl =  jpegcodec_ioctl,
	.unlocked_ioctl =  jpegcodec_ioctl,
	.mmap =   jpegcodec_mmap,	
	.get_unmapped_area = NULL,	
	.mmap =   jpegcodec_mmap,
	.open =   jpegcodec_open,
	.release =jpegcodec_close
};

#ifdef CONFIG_PM

/* suspend and resume support for the lcd controller */

static int jpegcodec_suspend(struct platform_device *dev, pm_message_t state)
{

	return 0;
}

static int jpegcodec_resume(struct platform_device *dev)
{

	return 0;
}

#else
#define jpegcodec_suspend NULL
#define jpegcodec_resume  NULL
#endif

static irqreturn_t jpegirq_handler(int irq, void *dev_id, struct pt_regs *r)
{
	jpeg_priv_t *priv = (jpeg_priv_t *)dev_id;
	__u32 u32interruptStatus;
	__u32 u32BufferSize,i;	
	__u32 address,row, row_size,row_data_size;	   
	/* Get the interrupt status */	     
	u32interruptStatus = _DRVJPEG_GET_INT_STATUS();
	/* It's Header Decode Complete Interrupt */
    if(u32interruptStatus & DHE_INTS)
    {
	__u16 u16Width,UVWidth,UVHeight, height;
	
    	/* Get the JPEG format */
    	priv->yuvformat = _DRVJPEG_DEC_GET_DECODED_IMAGE_FORMAT();
    	/* Get the decoded image dimension */
   	w55fa92_Jpeg_GetDecodedDimension((__u16*)&priv->height,(__u16*)&priv->width); 	

	current_data_row = 0;
	
	total_data_row = priv->height;
	if((total_data_row % 2) && priv->decode_output_format == DRVJPEG_DEC_PRIMARY_PACKET_YUV422)
		total_data_row++;

	//printk("<Header Decode Complete> -> %d x %d  JINTCR 0x%X JITCR 0x%X JDOWFBS 0x%X\n", priv->width, priv->height,inp32(REG_JINTCR), inp32(REG_JITCR),inp32(REG_JDOWFBS) );	

	if(priv->windec_en)	
	{
		int max_mcux,max_mcuy;
		max_mcux = priv->width/16-1;
		max_mcuy = priv->height/16-1;
		if(priv->windec_mcux_end > max_mcux || priv->windec_mcuy_end > max_mcuy) 	
		{
			priv->state = JPEG_DECODE_PARAM_ERROR;
			w55fa92_Jpeg_Init();
			enc_reserved_size = 0;
			enc_reserved = 0;
			enc_buffer_from_user = 0;
			jpeg_thumbnail_size = 0;
			jpeg_thumbnail_bitstreamsize = 0;
			jpeg_thumbnail_offset = 0;	
			enc_thumbnail = 0;
			priv->decopw_en = 0;
			priv->decopw_tcount = 0;
			priv->decopw_end = 0;
			outp32(REG_JMCR,0x2);
		        outp32(REG_JMCR,0);
			wake_up_interruptible(&jpeg_wq);
			return IRQ_HANDLED;
		}
	}


    	if(priv->scale)
    	{
 			u16 u16RatioH,u16RatioW,orignal_width,orignal_height;	
			u32 output;
			if(priv->decode_output_format != DRVJPEG_DEC_PRIMARY_PLANAR_YUV)
		    		output = DRVJPEG_DEC_PACKET_DOWNSCALE_MODE;
			else
				output = DRVJPEG_DEC_PLANAR_DOWNSCALE_MODE;
			
			if(priv->windec_en)
			{
				if(priv->decode_output_format == DRVJPEG_DEC_PRIMARY_PLANAR_YUV)
				{
					priv->state = JPEG_DECODE_PARAM_ERROR;
					w55fa92_Jpeg_Init();
					enc_reserved_size = 0;
					enc_reserved = 0;
					enc_buffer_from_user = 0;
					jpeg_thumbnail_size = 0;
					jpeg_thumbnail_bitstreamsize = 0;
					jpeg_thumbnail_offset = 0;	
					enc_thumbnail = 0;
					priv->decopw_en = 0;
					priv->decopw_tcount = 0;
					priv->decopw_end = 0;
					outp32(REG_JMCR,0x2);
				        outp32(REG_JMCR,0);
					wake_up_interruptible(&jpeg_wq);
			     	  	return IRQ_HANDLED;
				}
				
				orignal_height = Windec_height;
				orignal_width = Windec_width;
			}
			else
			{
				orignal_height = priv->height;
				orignal_width = priv->width;
			}		
			if(w55fa92_Jpeg_CalScalingFactor(
					output ,			//Up / Down Scaling
					orignal_height,				//Original Height
					orignal_width,				//Original Width
					priv->scaled_height,			//Scaled Height
					priv->scaled_width,			//Scaled Width
					&u16RatioH,				//Horizontal Ratio
					&u16RatioW				//Vertical Ratio
					) != 0)
			{
				//printf("Downscale Fail\n");
				priv->state = JPEG_DECODE_PARAM_ERROR;
				w55fa92_Jpeg_Init();
				enc_reserved_size = 0;
				enc_reserved = 0;
				enc_buffer_from_user = 0;
				jpeg_thumbnail_size = 0;
				jpeg_thumbnail_bitstreamsize = 0;
				jpeg_thumbnail_offset = 0;	
				enc_thumbnail = 0;
				priv->decopw_en = 0;
				priv->decopw_tcount = 0;
				priv->decopw_end = 0;
				outp32(REG_JMCR,0x2);
			        outp32(REG_JMCR,0);
				wake_up_interruptible(&jpeg_wq);
			        return IRQ_HANDLED;
			}
			else
			{
				//printf("Downscale OK\n");
				w55fa92_Jpeg_SetScalingFactor(output,u16RatioH,u16RatioW);
				priv->width = priv->scaled_width;
				priv->height = priv->scaled_height;
				if(priv->decode_output_format == DRVJPEG_DEC_PRIMARY_PLANAR_YUV)
				{
					if(priv->width % 2) 
					    	UVWidth = priv->width / 2 + 1;       
					else
		        			UVWidth = priv->width / 2;
        				UVHeight = height = priv->height;	
					/* Sets the height of U and V for YUV420 image */    
					if((priv->yuvformat == DRVJPEG_DEC_YUV420)|| (priv->yuvformat == DRVJPEG_DEC_YUV422T))
					{
					        if(height % 2)
				        	{
					        	UVHeight = height / 2 + 1;
				        		height++;            
				        }    
				        else
				        	UVHeight = height / 2;            
					}
					_DRVJPEG_SET_UADDR(priv->paddr + priv->src_bufsize + priv->width * priv->height);
					_DRVJPEG_SET_VADDR(priv->paddr + priv->src_bufsize  + priv->width * priv->height + UVWidth * UVHeight);	
					g_JPEG_RAW_SIZE = priv->width * priv->height + 2 * UVWidth * UVHeight;				
					priv->dec_stride = 0;	
				}
			}   		
    		
    	}
    	else
    	{
		if(priv->yuvformat == DRVJPEG_DEC_YUV411)   
	    	{
			/* 32-pixel alignment for YUV411 raw data */
			if(priv->width % 32)
		        	priv->width = (priv->width & 0xFFFFFFE0) + 32;
	   	}
	    	else if((priv->yuvformat == DRVJPEG_DEC_YUV444) || (priv->yuvformat == DRVJPEG_DEC_YUV422T))
		{
			/* 8-pixel alignment for YUV444 raw data */
			if(priv->width % 8)
		    	{
		        	priv->width = (priv->width & 0xFFFFFFF8) + 8;
		    	}
		}
		else
		{
		    	/* 16-pixel alignment for YUV422 or YUV420 raw data */
		    	if(priv->width % 16)
		        	priv->width = (priv->width & 0xFFFFFFF0) + 16;
		}
		if(priv->decode_output_format == DRVJPEG_DEC_PRIMARY_PLANAR_YUV)
		{
			if(priv->yuvformat == DRVJPEG_DEC_YUV411)
  			{   /* For YUV411 raw data */
			        UVWidth = priv->width/4;        
			}
			else if((priv->yuvformat == DRVJPEG_DEC_YUV444) || (priv->yuvformat == DRVJPEG_DEC_YUV422T))
			{   /* For YUV444 raw data */	
				UVWidth = priv->width;
			}
			/* Set the U-component and V-componente width for YUV422 or YUV420 raw data */	       
			else if(priv->width % 2) 
			    	UVWidth = priv->width / 2 + 1;       
			else
        			UVWidth = priv->width / 2;
	        	
			UVHeight = height = priv->height;	
			/* Sets the height of U and V for YUV420 image */
			if(priv->yuvformat == DRVJPEG_DEC_YUV420)
			{
			    /* 16-pixel alignment for YUV422 or YUV420 raw data */
			    if(priv->height % 16)
			        priv->height = (priv->height & 0xFFFFFFF0) + 16;			
				UVHeight = priv->height / 2;
			}
			else if(priv->yuvformat == DRVJPEG_DEC_YUV422)
			{
			    /* 8-pixel alignment for YUV444 raw data */
			    if(priv->height % 8)
			        priv->height = (priv->height & 0xFFFFFFF8) + 8; 
			    UVHeight = priv->height;    			
			}
			else if(priv->yuvformat == DRVJPEG_DEC_YUV444)
			{
			    /* 8-pixel alignment for YUV444 raw data */
			    if(priv->height % 8)
			        priv->height = (priv->height & 0xFFFFFFF8) + 8; 
			    UVHeight = priv->height;    			
			}	
			else if(priv->yuvformat == DRVJPEG_DEC_YUV411)
			{
			    /* 8-pixel alignment for YUV444 raw data */
			    if(priv->height % 8)
			        priv->height = (priv->height & 0xFFFFFFF8) + 8; 
			    UVHeight = priv->height;    	
			}
			else if(priv->yuvformat == DRVJPEG_DEC_YUV422T)
			{
			    /* 16-pixel alignment for YUV422 or YUV420 raw data */
			    if(priv->height % 16)
			        priv->height = (priv->height & 0xFFFFFFF0) + 16;					
				UVHeight = priv->height / 2;
			}
			else
			{
			    /* 8-pixel alignment for raw data */
			    if(priv->height % 8)
			        priv->height = (priv->height & 0xFFFFFFF8) + 8; 			
				UVHeight = priv->height;
			}
			_DRVJPEG_SET_UADDR(priv->paddr + priv->src_bufsize + priv->width * priv->height);
			_DRVJPEG_SET_VADDR(priv->paddr + priv->src_bufsize + priv->width * priv->height + UVWidth * UVHeight);
			g_JPEG_RAW_SIZE = priv->width * priv->height + 2 * UVWidth * UVHeight;
			priv->dec_stride = 0;
		}
	}
	jpeg_align_width = priv->width;

	if(jpeg_width > jpeg_align_width)
		jpeg_width = jpeg_align_width;


    	if(priv->dec_stride >= priv->width)
    	{
		
		priv->dec_stride = priv->dec_stride - priv->width;
    		_DRVJPEG_SET_YSTRIDE(priv->dec_stride);
    		u16Width = priv->width + priv->dec_stride; 
    	}
    	else
	{
    		_DRVJPEG_SET_YSTRIDE(0);   
		priv->dec_stride = 0;
		u16Width = priv->width;
			
	}
	/* Set the image dimension */ 	
	w55fa92_Jpeg_SetDimension(priv->height, u16Width);     

	//printk("priv->decopw_en -> %d\n", priv->decopw_en);
	if(priv->decopw_en)
	{
		__u32	u32BufferSize,height;					

		priv->decopw_mcuy = priv->height / 16 - 1;
		if((priv->height % 16) != 0)
		{
			priv->decopw_mcuy++;	
			height = (priv->height & 0xFFFFFFF0) + 16;
		}		
		else
			height = priv->height;
 		
		priv->decopw_mcux = u16Width / 16 - 1;

		if(priv->decode_output_format == DRVJPEG_DEC_PRIMARY_PACKET_RGB888)
		{
			u32BufferSize =/* (CONFIG_JPEG_CODEC_BUFFER_SIZE - priv->src_bufsize )*/priv->dst_bufsize  / u16Width / 4;	
			priv->decopw_tmcuynum = u32BufferSize  / 16;
			if(priv->decopw_tmcuynum > priv->decopw_mcuy)
				priv->decopw_tmcuynum = priv->decopw_mcuy;
			priv->decopw_tsize =  u16Width * priv->decopw_tmcuynum * 16 * 4;
			TotalDataSize = height * u16Width * 4;	
		}
		else
		{
			u32BufferSize = /*(CONFIG_JPEG_CODEC_BUFFER_SIZE - priv->src_bufsize )*/priv->dst_bufsize  / u16Width / 2;
			priv->decopw_tmcuynum = u32BufferSize  / 16;
			if(priv->decopw_tmcuynum > priv->decopw_mcuy)
				priv->decopw_tmcuynum = priv->decopw_mcuy;
			priv->decopw_tsize =  u16Width * priv->decopw_tmcuynum * 16 * 2;
			TotalDataSize = height * u16Width * 2;	
		}
		priv->decopw_tnum = TotalDataSize / priv->decopw_tsize ;
		
		if((TotalBufferSize % priv->decopw_tsize)  != 0)			
			priv->decopw_tnum++; 

		if(priv->decopw_tsize > TotalDataSize)
			outp32(REG_JDOWFBS,TotalDataSize/4);
		else
		outp32(REG_JDOWFBS,priv->decopw_tsize/4);
		
		//printk("<First Trigger in DHE 0x%X>\n",inp32(REG_JDOWFBS) * 4);
		//printk("	<Remaining 0x%X>\n",TotalDataSize);
		if(u32BufferSize < 1)
		{
			printk("Config Buffer size is 0x%X\nNeed Buffer size is 0x%X\n",CONFIG_JPEG_CODEC_BUFFER_SIZE, u32BufferSize);
			priv->state = JPEG_MEM_SHORTAGE;
			w55fa92_Jpeg_Init();
			enc_reserved_size = 0;
			enc_reserved = 0;
			enc_buffer_from_user = 0;
			jpeg_thumbnail_size = 0;
			jpeg_thumbnail_bitstreamsize = 0;
			jpeg_thumbnail_offset = 0;
			enc_thumbnail = 0;
			priv->decopw_en = 0;
			priv->decopw_tcount = 0;
			priv->decopw_end = 0;
        		wake_up_interruptible(&jpeg_wq);
		}
		else
		{
 			priv->state = JPEG_DECODED_HEADER;	
			w55fa92_Jpeg_GetDecodedDimension((__u16*)&priv->height,(__u16*)&priv->width); 	
			if(priv->decode_output_format == DRVJPEG_DEC_PRIMARY_PACKET_YUV422 && priv->width % 2)
				priv->width++;				
    			outp32(REG_JITCR, inp32(REG_JITCR) | BIT18);
		}
	}
	else
	{
		if(priv->windec_en)
		{
			if(priv->scale)
				u32BufferSize = priv->scaled_width * priv->scaled_height;	
			else
				u32BufferSize  = 16 * (priv->windec_mcux_end - priv->windec_mcux_start + 1) * 16 * (priv->windec_mcuy_end - priv->windec_mcuy_start + 1);
			if(priv->decode_output_format == DRVJPEG_DEC_PRIMARY_PACKET_RGB888)
				u32BufferSize = priv->src_bufsize + u32BufferSize  * 4;
			else
				u32BufferSize = priv->src_bufsize + u32BufferSize  * 2;
		}
		else
		{
			if(priv->paddr_dst == 0)
			{
				if(priv->decode_output_format == DRVJPEG_DEC_PRIMARY_PACKET_RGB888)
					u32BufferSize = priv->src_bufsize + priv->height * u16Width * 4;
				else
					u32BufferSize = priv->src_bufsize + priv->height * u16Width * 2;
			}
			else
			{
				u32BufferSize = priv->src_bufsize;
			}
		}

		if(u32BufferSize  > CONFIG_JPEG_CODEC_BUFFER_SIZE)
		{
			printk("Config Buffer size is 0x%X\nNeed Buffer size is 0x%X\n",CONFIG_JPEG_CODEC_BUFFER_SIZE, u32BufferSize);
			priv->state = JPEG_MEM_SHORTAGE;
			w55fa92_Jpeg_Init();
			enc_reserved_size =0;
			enc_buffer_from_user = 0;
			jpeg_thumbnail_size = 0;
			jpeg_thumbnail_bitstreamsize = 0;
			jpeg_thumbnail_offset = 0;
			enc_thumbnail = 0;
			priv->decopw_en = 0;
			priv->decopw_tcount = 0;
			priv->decopw_end = 0;
        		wake_up_interruptible(&jpeg_wq);
		}
		else
		    priv->state = JPEG_DECODED_HEADER;			
	}
    	/* Clear interrupt status */  
	_DRVJPEG_CLEAR_INT(DHE_INTS);
    }
    /* It's Encode Complete Interrupt */
    else if(u32interruptStatus & ENC_INTS)
    {	
	//printk("Encode Complete\n");
	
    	/* Clear interrupt status */  
	enc_stride = 0;
    	_DRVJPEG_CLEAR_INT(ENC_INTS);   

		if(enc_thumbnail == 1) //Thumbnail encode complete
		{				
			__u32 thumbnailsize = _DRVJPEG_GET_ENC_PRIMARY_BITSTREAM_SIZE();
			__u8 *u8Addr;
			jpeg_thumbnail_bitstreamsize = thumbnailsize;
			enc_reserved_size = thumbnailsize + enc_reserved_size;	
			if(enc_reserved_size % 4)
				enc_reserved_size = (enc_reserved_size + 0x03) & ~0x03;		
			//printk("Thumbnail size %d\n",thumbnailsize);		
			u8Addr = (__u8 *) priv->vaddr;		
			*(u8Addr + 4) = ((enc_reserved_size - 4 + 2)  & 0xFF00) >> 8;
			*(u8Addr + 5) = (enc_reserved_size - 4 + 2) & 0xFF;			
  			
			///printk("Paimary start address 0x%X\n",priv->vaddr + enc_reserved_size);
				
			outp32(REG_JPRIQC, 0x000000F4);
			outp32(REG_JTHBQC, 0x000000F4);
			outp32(REG_JPRST, 0x00000004);
			outp32(REG_JTRST, 0x00000004);

			// Disable the Primary Up-scaling & Scaling-down
			outp32(REG_JPSCALU, 0x00000000);
			outp32(REG_JPSCALD, 0x00000000);

			// Reset JUPRAT and JSRCH
			outp32(REG_JUPRAT, 0x00000000);
			outp32(REG_JSRCH, 0x00000FFF);
			//-------------------------------------------
	
			/* Reset JPEG (JMCR [1]) */
			outp32(REG_JMCR,0x00000002);
			outp32(REG_JMCR,0x00000000);
			outp32(REG_JMACR, 0x00400000);  //Can't use single buffer			

			priv->encode = 1;

			_DRVJPEG_SET_YSTRIDE(priv->encode_width);		
			_DRVJPEG_SET_USTRIDE(priv->encode_width/2);
			_DRVJPEG_SET_VSTRIDE(priv->encode_width/2);
			/* Primary Encode Image Width / Height */
    			w55fa92_Jpeg_SetDimension(priv->encode_height,priv->encode_width);  	
			//Set Encode Source Image Height    
			_DRVJPEG_SET_SOURCE_IMAGE_HEIGHT(priv->encode_height);
			w55fa92_Jpeg_SetEncodeMode(priv->encode_source_format, priv->encode_image_format);
			/* Include Quantization-Table and Huffman-Table */
			_DRVJPEG_ENC_SET_HEADER_CONTROL(DRVJPEG_ENC_PRIMARY_QTAB | DRVJPEG_ENC_PRIMARY_HTAB); 
			
			/* Encode Complete Interrupt Enable and clear the Encode Complete Interrupt */
			_DRVJPEG_INT_ENABLE(DRVJPEG_ENC_INTE);
			_DRVJPEG_CLEAR_INT(DRVJPEG_ENC_INTS);
			priv->state = JPEG_ENCODING;
			if(enc_buffer_from_user == 0)
			{
				if(priv->paddr_src == 0)
				{
					  //printk("jpeg encode source addr:%x\n", priv->paddr);
					_DRVJPEG_SET_YADDR(priv->paddr);
					}
					if(priv->paddr_dst == 0)
					{
						  //printk("jpeg encode dst addr:%x\n", priv->paddr + priv->src_bufsize);
						_DRVJPEG_SET_BITSTREAM_ADDR(priv->paddr + priv->src_bufsize);
					}
					if(priv->encode_source_format == DRVJPEG_ENC_SRC_PACKET)
						u32BufferSize = priv->encode_height * priv->encode_width * 2;
					else
					{
						if(priv->encode_image_format == DRVJPEG_ENC_PRIMARY_YUV422)
							u32BufferSize = priv->encode_height * priv->encode_width * 2;
						else
							u32BufferSize = priv->encode_height * priv->encode_width * 3/2;
					}

					if(u32BufferSize > CONFIG_JPEG_CODEC_BUFFER_SIZE)
					{
						printk("Config Buffer size is 0x%X\nNeed Buffer size is 0x%X\n",CONFIG_JPEG_CODEC_BUFFER_SIZE, u32BufferSize);
						priv->state = JPEG_MEM_SHORTAGE;
						w55fa92_Jpeg_Init();
						enc_reserved_size = 0;
						enc_reserved = 0;
						enc_buffer_from_user = 0;
						jpeg_thumbnail_size = 0;
						jpeg_thumbnail_bitstreamsize = 0;
						jpeg_thumbnail_offset = 0;
						enc_thumbnail = 0;
						priv->decopw_en = 0;
						priv->decopw_tcount = 0;
						priv->decopw_end = 0;
						return -JPEG_MEM_SHORTAGE;
					}
				}
				else
				{
					_DRVJPEG_SET_BITSTREAM_ADDR(priv->paddr + enc_reserved_size);
					/* add for videoin */
					_DRVJPEG_SET_YADDR(enc_yaddr_from_user);
					_DRVJPEG_SET_UADDR(enc_uaddr_from_user);
					_DRVJPEG_SET_VADDR(enc_vaddr_from_user);
				}

			    if(priv->scale)
			    {
			    		u16 u16ratioH,u16ratioW;
			    		
					if(w55fa92_Jpeg_CalScalingFactor(
							DRVJPEG_ENC_UPSCALE_MODE,	//Up / Down Scaling
							priv->encode_height,					//Original Height
							priv->encode_width,					//Original Width
							priv->scaled_height,		//Scaled Height
							priv->scaled_width,		//Scaled Width
							&u16ratioH,					//Horizontal Ratio
							&u16ratioW					//Vertical Ratio
					) != 0)
					{
					        priv->state = JPEG_ENCODE_PARAM_ERROR;
					         wake_up_interruptible(&jpeg_wq);
					         return 1;
					}
					else
					{
						w55fa92_Jpeg_SetScalingFactor(DRVJPEG_ENC_UPSCALE_MODE, u16ratioH, u16ratioW);
						w55fa92_Jpeg_SetDimension(priv->scaled_height,priv->scaled_width);
						outp32(REG_JSRCH, priv->encode_height);

					}      
			    }
				/* Trigger JPEG decoder */
				//printk("Encode Trigger Primary\n");
	    			w55fa92_Jpeg_Trigger();	
				wake_up_interruptible(&jpeg_wq);	
		}

	if(enc_thumbnail == 0 || enc_thumbnail == 2)
	{
		
		/* Get the Encode Bit stream length */
		if(enc_thumbnail == 0)
		{
			priv->image_size[priv->bufferend] = _DRVJPEG_GET_ENC_PRIMARY_BITSTREAM_SIZE();				
		}
		else
		{
			__u8 *u8Addr;

			u8Addr = (__u8 *) priv->vaddr;
			*(u8Addr + enc_reserved_size) = 0xFF;		
			*(u8Addr + enc_reserved_size + 1) = 0xFF;	
			priv->image_size[priv->bufferend] = _DRVJPEG_GET_ENC_PRIMARY_BITSTREAM_SIZE() + enc_reserved_size;
			//printk("Paimary size %d\n",_DRVJPEG_GET_ENC_PRIMARY_BITSTREAM_SIZE() );
			//printk("JPEG size %d\n",priv->image_size[priv->bufferend]);
			
		}			
		enc_buffer_from_user = 0;
		enc_reserved_size = 0;
		enc_reserved = 0;
		enc_thumbnail = 0;
		jpeg_thumbnail_size = 0;
	        priv->bufferend = (priv->bufferend + 1) % priv->buffercount;
	        priv->state = JPEG_ENCODED_IMAGE;
		wake_up_interruptible(&jpeg_wq);	
	}
	else if(enc_thumbnail == 1)
		enc_thumbnail = 2;      
    }
    /* Decode output Wait */
    else if (u32interruptStatus & JPG_DOW_INTS)
    {
	//printk("<Decode output Wait> JINTCR 0x%X JITCR 0x%X JDOWFBS 0x%X\n", inp32(REG_JINTCR), inp32(REG_JITCR),inp32(REG_JDOWFBS));
	enc_reserved_size = 0;
	enc_reserved = 0;
	enc_buffer_from_user = 0;
	jpeg_thumbnail_size = 0;
	jpeg_thumbnail_bitstreamsize = 0;
	jpeg_thumbnail_offset = 0;
	enc_thumbnail = 0;
 	if(priv->decopw_en)
	{	
		__u32 u32TmpAddr ,data_to_fill,u32TMP;	
		/* Copy data from tmp buffer to destination buffer (priv->pages) */
		//size = (inp32(REG_JDOWFBS) * 4);

		address = priv->vaddr + priv->src_bufsize;

		if(priv->decode_output_format == DRVJPEG_DEC_PRIMARY_PACKET_RGB888)
		{
			row = (inp32(REG_JDOWFBS) * 4)  / jpeg_align_width / 4;
			row_size = jpeg_align_width * 4;
			row_data_size = jpeg_width * 4;
		}
		else
		{
			row = (inp32(REG_JDOWFBS) * 4) / jpeg_align_width / 2;	
			row_size = jpeg_align_width * 2;	
			if(priv->decode_output_format == DRVJPEG_DEC_PRIMARY_PACKET_YUV422 && ((jpeg_width % 2) != 0))
				row_data_size = (jpeg_width + 1) * 2;
			else
				row_data_size = jpeg_width * 2;
		}
	
		//printk("current row %d\n",current_data_row);
		//printk("jpeg_align_width  %d\n",jpeg_align_width );
		//printk("row  %d\n",row );
		//printk("row_data_size  %d\n",row_data_size );
		if((current_data_row + row) > total_data_row)
		{
			row = total_data_row - current_data_row;
			//printk("--> row  %d\n",row );
		}
		for(i =0;i<row;i++)
		{
			data_to_fill = row_data_size;
			u32TmpAddr = address;
fill_again:
			if(data_to_fill + priv->decopw_page_offset >= PAGE_SIZE)
			{	
					
				u32TMP = PAGE_SIZE - priv->decopw_page_offset;
					
				memcpy((char*)(page_address(priv->pages[priv->decopw_page_index++]) + priv->decopw_page_offset), (char*)u32TmpAddr, u32TMP);			
				priv->decopw_page_offset = 0;	
				data_to_fill = data_to_fill - u32TMP; 
				u32TmpAddr = u32TmpAddr + u32TMP;
				if(data_to_fill != 0)
					goto fill_again;
			}
			else
			{
				memcpy((char*)(page_address(priv->pages[priv->decopw_page_index]) + priv->decopw_page_offset), (char*)u32TmpAddr, data_to_fill);
				
				priv->decopw_page_offset += data_to_fill;
			}
			address += row_size;				
		}		
		current_data_row  = current_data_row  + row;
		if(TotalDataSize != (inp32(REG_JDOWFBS) * 4))
		{	
			//printk("<Trigger in DOW -> 0x%X>\n", inp32(REG_JDOWFBS) * 4);	
			//printk("	<Remaining 0x%X>\n",TotalDataSize);   
			
			TotalDataSize = TotalDataSize - inp32(REG_JDOWFBS) * 4;

			if(priv->decopw_tsize > TotalDataSize)
				outp32(REG_JDOWFBS,TotalDataSize/4);
			else
				outp32(REG_JDOWFBS,priv->decopw_tsize/4);
			/* Decode mode */
			priv->convert = JPEG_CMD_NONE;			
			//printk("<Trigger in DOW -> 0x%X>\n", inp32(REG_JDOWFBS) * 4);	
			//printk("	<Remaining 0x%X>\n\n\n",TotalDataSize);    
			_DRVJPEG_CLEAR_INT(JPG_DOW_INTS);				
			_DRVJPEG_SET_YADDR(priv->decopw_yaddr);	
			flush_cache_all();
			outp32(REG_JITCR, inp32(REG_JITCR) | BIT18);
		}
		else
		{
			if(TotalDataSize > 0)
			{
				TotalDataSize = TotalDataSize - inp32(REG_JDOWFBS) * 4;
				//printk("[Final -> 0x%X]\n", inp32(REG_JDOWFBS) * 4);	
				//printk("	[Remaining 0x%X]\n",TotalDataSize);    
				outp32(REG_JDOWFBS,0xFFFFFFFF);
				if(priv->decInWait_buffer_size != 0)
				{
					_DRVJPEG_INT_ENABLE(DRVJPEG_DEC_INTE | DRVJPEG_DER_INTE | DRVJPEG_DHE_INTE | DRVJPEG_IPW_INTE);
				}
				else	
				{
					_DRVJPEG_INT_ENABLE(DRVJPEG_DEC_INTE | DRVJPEG_DER_INTE | DRVJPEG_DHE_INTE);  						
				}
				
				_DRVJPEG_CLEAR_INT(JPG_DOW_INTS);	
				flush_cache_all();
			}
		}
	}   		
    		
    }
    /* It's Decode Complete Interrupt */
    else if(u32interruptStatus & DEC_INTS) 
    {
	//printk("<Decode Complete> JINTCR 0x%X JITCR 0x%X JDOWFBS 0x%X\n", inp32(REG_JINTCR), inp32(REG_JITCR),inp32(REG_JDOWFBS));
	/* Get the image dimension */
	if(!priv->decopw_en)
		w55fa92_Jpeg_GetDimension((__u16*)&priv->height,(__u16*)&priv->width); 	   
	/* Clear interrupt status */    
	_DRVJPEG_CLEAR_INT(DEC_INTS);
	if(priv->decopw_en)
	{			
		__u32 u32TmpAddr ,data_to_fill,u32TMP;	
		/* Copy data from tmp buffer to destination buffer (priv->pages) */
		//size = (inp32(REG_JDOWFBS) * 4);

		address = priv->vaddr + priv->src_bufsize;

		if(priv->decode_output_format == DRVJPEG_DEC_PRIMARY_PACKET_RGB888)
		{
			row = (inp32(REG_JDOWFBS) * 4)  / jpeg_align_width / 4;
			row_size = jpeg_align_width * 4;
			row_data_size = jpeg_width * 4;
		}
		else
		{
			row = (inp32(REG_JDOWFBS) * 4) / jpeg_align_width / 2;	
			row_size = jpeg_align_width * 2;	
			if(priv->decode_output_format == DRVJPEG_DEC_PRIMARY_PACKET_YUV422 && ((jpeg_width % 2) != 0))
				row_data_size = (jpeg_width + 1) * 2;
			else
				row_data_size = jpeg_width * 2;
		}
	
		//printk("current row %d\n",current_data_row);
		//printk("jpeg_align_width  %d\n",jpeg_align_width );
		//printk("row  %d\n",row );
		//printk("row_data_size  %d\n",row_data_size );
		if(current_data_row < total_data_row)
		{
			if((current_data_row + row) > total_data_row)
			{
				row = total_data_row - current_data_row;
				//printk("--> row  %d\n",row );
			}
			for(i =0;i<row;i++)
			{
				data_to_fill = row_data_size;
				u32TmpAddr = address;
fill_again_final:
				if(data_to_fill + priv->decopw_page_offset >= PAGE_SIZE)
				{	
					
					u32TMP = PAGE_SIZE - priv->decopw_page_offset;
					
					memcpy((char*)(page_address(priv->pages[priv->decopw_page_index++]) + priv->decopw_page_offset), (char*)u32TmpAddr, u32TMP);			
					priv->decopw_page_offset = 0;	
					data_to_fill = data_to_fill - u32TMP; 
					u32TmpAddr = u32TmpAddr + u32TMP;
					if(data_to_fill != 0)
					goto fill_again_final;
				}
				else
				{
					memcpy((char*)(page_address(priv->pages[priv->decopw_page_index]) + priv->decopw_page_offset), (char*)u32TmpAddr, data_to_fill);
				
					priv->decopw_page_offset += data_to_fill;
				}
				address += row_size;				
			}		
			flush_cache_all();
		}
	
		if(priv->decode_output_format == DRVJPEG_DEC_PRIMARY_PACKET_YUV422 && ((jpeg_width % 2) != 0))
			row_data_size = (jpeg_width + 1);
		else
			row_data_size = jpeg_width;
		if(priv->decode_output_format == DRVJPEG_DEC_PRIMARY_PACKET_RGB888)
			JPEG_RAW_SIZE_DOW = row_data_size * priv->height * 4 ;		
		else
			JPEG_RAW_SIZE_DOW = row_data_size * priv->height * 2 ;	
	}   		

	enc_reserved_size = 0;
	enc_buffer_from_user = 0;
	//printk("Decode Complete\n");
	/* Call the User-defined call back function */	
       	priv->state = JPEG_DECODED_IMAGE;	
        wake_up_interruptible(&jpeg_wq);
        // JPEG engine reset
       	outp32(REG_JMCR,0x2);
        outp32(REG_JMCR,0);
	if(priv->decopw_en)
	{
		outp32(REG_AHBIPRST, JPG_RST);
		outp32(REG_AHBIPRST, 0);
	}
   
    }
    /* It's Decode Input Wait Interrupt */
    else if(u32interruptStatus & IPW_INTS) 
    {
	priv->decInWait_counter++;	
	 priv->decInWait_buffer_empty = 1;	
	_DRVJPEG_CLEAR_INT(IPW_INTS);    
    }
    /* It's Decode Error Interrupt */
    else if(u32interruptStatus & DER_INTS)     
    {   
      //printk("decode errir\n");
    	/* Clear interrupt status */  
     	_DRVJPEG_CLEAR_INT(DER_INTS);      
	enc_reserved_size = 0;
	enc_reserved = 0;
	enc_buffer_from_user= 0;
	jpeg_thumbnail_size = 0;
	jpeg_thumbnail_bitstreamsize = 0;
	jpeg_thumbnail_offset = 0;
	enc_thumbnail = 0;
     	 /* Call the User-defined call back function */	
        //g_pJPEG_IntHandler(DER_INTS,0,0,0,0);
        priv->state = JPEG_DECODE_ERROR;
        wake_up_interruptible(&jpeg_wq);
    }

	//printk("jpegirq_handler\n");
	return IRQ_HANDLED;
}

extern void ImageProcess_Y0UY1V2RGB(s8 *bySrcbuf, s8 *byDesbuf, u32 u32Width, u32 u32Height, u32 u32RGB555Enabled);
#if 0
static int jpeg_kernel_thread(void *param)
{
    jpeg_priv_t *priv = (jpeg_priv_t *)param;

	daemonize("jpegd");

	for(;;){
    
		wait_event_interruptible(jpegd_wq, (jpeg_command != JPEG_CMD_NONE));
		if(jpeg_command == JPEG_CMD_YUV422_TO_RGB555) {
		    ImageProcess_Y0UY1V2RGB(priv->vaddr + priv->src_bufsize, 
		                            priv->vaddr + priv->src_bufsize, priv->width, priv->height, 1);
		} else if(jpeg_command == JPEG_CMD_YUV422_TO_RGB565) {
		    ImageProcess_Y0UY1V2RGB(priv->vaddr + priv->src_bufsize, 
		                            priv->vaddr + priv->src_bufsize, priv->width, priv->height, 0);
		} else if(jpeg_command == JPEG_CMD_EXIT) {
		    break;
        }
		jpeg_command = 0;
        priv->state = JPEG_DECODED_IMAGE;
        wake_up_interruptible(&jpeg_wq);
	}

	//printk("quit jpeg kernel thread\n");
	complete_and_exit(&jpeg_thread_exit, 0);
} 

static __s32 jpeg_set_engine(void *dev_id, void* sbuffer)
{
	jpeg_priv_t *priv = (jpeg_priv_t *)dev_id;
	
	//printk("sbuffer=%x\n",sbuffer);
	//priv->file->f_pos = 0;

	priv->state = JPEG_ENCODING;
	
	w55fa92_Jpeg_Init();
	priv->decopw_en = 0;
	priv->decopw_tcount = 0;
	priv->decopw_end = 0;
	//Adjust the QT Table
	w55fa92_Jpeg_AdjustQTAB(DRVJPEG_ENC_PRIMARY,priv->qadjust, priv->qscaling);
	
	_DRVJPEG_SET_YADDR((__u32)sbuffer);

	if(priv->paddr_dst == 0)
	{
		//printk("set dst_addr %x, priv->bufferend=%d\n",
		//	priv->paddr + priv->buffersize * priv->bufferend, 
		//	priv->bufferend);
		_DRVJPEG_SET_BITSTREAM_ADDR(priv->paddr + priv->buffersize * priv->bufferend);
	}
	w55fa92_Jpeg_Trigger();
}
#endif
void jpegcodec_release(struct video_device *vfd)
{
	//kfree(vfd);
}

extern unsigned int w55fa92_ahb_clock;
int __devinit jpegcodec_init(void)
{
	int ret = 0;	
	int result;

	jpeg_priv_t *priv = (jpeg_priv_t *)&jpeg_priv;
	
	printk("jpegcodec_init\n");

	//	printk("videoin_param=%d, jpeg_param=%d, jpeginfo=%d\n", 
	//    	sizeof(videoin_param_t), sizeof(jpeg_param_t), sizeof(jpeg_info_t));
	/* initialize locks */
	init_MUTEX(&priv->lock);
//	init_MUTEX(&jpeg_lock);
//	priv->jdev.owner = THIS_MODULE;
//	priv->jdev.vfl_type = VID_TYPE_JPEG_DECODER | VID_TYPE_JPEG_ENCODER;
//	priv->jdev.hardware = VID_HARDWARE_W55FA93;
	priv->jdev.release = jpegcodec_release;
	priv->jdev.fops = &jpegcodec_fops;
	priv->jdev.priv = &jpeg_priv;
	//priv->vaddr = jpeg_dmamalloc(CONFIG_JPEG_CODEC_BUFFER_SIZE);

	priv->vaddr = (u32)dma_alloc_writecombine(NULL, CONFIG_JPEG_CODEC_BUFFER_SIZE,
                                            &priv->paddr, GFP_KERNEL); 
	jpeg_v = priv->vaddr;
	if (!priv->vaddr)
		return -ENOMEM;
	priv->image_size = kmalloc(sizeof(__u32) * 10/*priv->buffercount*/, GFP_KERNEL);
	if(!priv->image_size)
	{

		return -ENOMEM;
	}
	
	jpeg_priv.state = JPEG_CLOSED;
	result = video_register_device(&priv->jdev, VFL_TYPE_GRABBER, jpeg_nr);

	if (result == -1) {
		printk("%s: video_register_device failed\n", __FUNCTION__);

		kfree(priv->image_size);
		return -EPIPE;
	}
	if (!request_mem_region((unsigned long)W55FA92_VA_JPEG, W55FA92_SZ_JPEG, "w55fa92-jpeg"))
	{
		printk("%s: request_mem_region failed\n", __FUNCTION__);
		video_unregister_device(&jpeg_priv.jdev);

		kfree(priv->image_size);
		return -EBUSY;
	}
	//ret = request_irq(IRQ_JPEG, jpegirq_handler, SA_INTERRUPT, "w55fa93-jpeg", priv);
	ret = request_irq(IRQ_JPEG, (irq_handler_t)jpegirq_handler, IRQF_DISABLED | IRQF_IRQPOLL, "w55fa92-jpeg", priv);

	if (ret) {
		printk("cannot get irq %d - err %d\n", IRQ_JPEG, ret);
		ret = -EBUSY;
		goto release_mem;
	}
	
	//videoin_register_outputdev((void*)priv, &videoin_fops);
	//kernel_thread(jpeg_kernel_thread, priv, 0);

	return ret;

release_mem:
	video_unregister_device(&priv->jdev);
	release_mem_region((unsigned long)W55FA92_VA_JPEG, W55FA92_SZ_JPEG);
	free_irq(IRQ_JPEG,priv);

	kfree(priv->image_size);
	return ret;

}

static void __exit jpegcodec_cleanup(void)
{
	jpeg_priv_t *priv = (jpeg_priv_t *)&jpeg_priv;
	
	video_unregister_device(&priv->jdev);
	release_mem_region((unsigned long)W55FA92_VA_JPEG, W55FA92_SZ_JPEG);
	free_irq(IRQ_JPEG,priv);
	kfree(priv->image_size);
	
	dma_free_writecombine(NULL, CONFIG_JPEG_CODEC_BUFFER_SIZE, (void *)priv->vaddr, priv->paddr);
   	 jpeg_command = JPEG_CMD_EXIT;
	wake_up_interruptible(&jpegd_wq);	
	wait_for_completion(&jpeg_thread_exit);

}

module_init(jpegcodec_init);
module_exit(jpegcodec_cleanup);

MODULE_DESCRIPTION("HW Jpeg codec driver for the W55FA92");
MODULE_LICENSE("GPL");

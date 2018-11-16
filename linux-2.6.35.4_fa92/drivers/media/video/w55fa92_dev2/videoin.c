/* videoin.c
 *
 * Copyright (c) Nuvoton technology corporation
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */
//#include <linux/config.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/list.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/mm.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/smp_lock.h>
#include <linux/vmalloc.h>
#include <linux/init.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#if 0
#include <linux/videodev.h>
#else
#include <linux/videodev2.h>
#include <media/v4l2-device.h>
#endif 
#include <linux/jiffies.h>

#include <asm/io.h>
#if 0
#include <asm/arch/w55fa95_reg.h>
#include <asm/arch/fb.h>
#include <asm/arch/w55fa95_fb.h>
#include <asm/arch/videoin.h>
#include <asm/arch/DrvVideoin.h>
#include <asm/arch-w55fa95/videodev_ex.h>
#else
#include <mach/w55fa92_reg.h>
#include <mach/fb.h>
#include <mach/w55fa92_fb.h>
#include <mach/videoin.h>
//#include <mach/DrvVideoin.h>
#include "DrvVideoin.h"
#include <mach/videodev_ex.h>
#endif
#include <linux/time.h>
#include "videoinpriv.h"


#define __BOOT_MEM__

static atomic_t vin_avail = ATOMIC_INIT(1);
//static spinlock_t spin_vin_opc = SPIN_LOCK_UNLOCKED;
/* for preview report to fb driver and capture DEV2 device*/

volatile unsigned int bIsVideoIn2Enable = 0;
EXPORT_SYMBOL(bIsVideoIn2Enable);
#ifdef CONFIG_W55FA92_VIDEOIN_DEV1					/* If define VIDEOIN_DEV1 */
	extern unsigned int w55fa92_VIN_PAC_BUFFER;
#else
	volatile unsigned int w55fa92_VIN_PAC_BUFFER;	/* If not define VIDEOIN_DEV1 */	
	EXPORT_SYMBOL(w55fa92_VIN_PAC_BUFFER);
#endif

static DECLARE_WAIT_QUEUE_HEAD(videoin_wq);
//static int videoin_nr = -1;
//module_param(videoin_nr, int, 1);

static int videoin_dev2 = 0;

//module_param(videoin_dev2, int, 1);
module_param(videoin_dev2, int, 0); 

/* Global variable */
static videoin_priv_t 		videoin_priv;
static VINDEV_T 			DevVin;
static VINIOCTL_T 			DevIoctl;


//volatile int 		i32FrameNumber;
//volatile int 		i32ReadFrameNum;

#ifdef CONFIG_SENSOR_OV9660_DEV2
	static __u32 u32Sensor = OV_9660;
#elif defined  CONFIG_SENSOR_OV7725_DEV2
	static __u32 u32Sensor = OV_7725;
#elif defined  CONFIG_SENSOR_OV7670_DEV2
	static __u32 u32Sensor = OV_7670;
#elif defined  CONFIG_SENSOR_NT99140_DEV2
	static __u32 u32Sensor = NT_99140;
#elif defined  CONFIG_SENSOR_NT99141_DEV2
	static __u32 u32Sensor = NT_99141;
#elif defined  CONFIG_SENSOR_NT99142_DEV2
	static __u32 u32Sensor = NT_99142;
#elif defined  CONFIG_SENSOR_NT99050_DEV2
	static __u32 u32Sensor = NT_99050;
#elif defined  CONFIG_SENSOR_HI702_DEV2
	static __u32 u32Sensor = HI_702;
#elif defined  CONFIG_SENSOR_NT99160_DEV2
	static __u32 u32Sensor = NT_99160;
#elif defined  CONFIG_SENSOR_NT99252_DEV2
	static __u32 u32Sensor = NT_99252;
#elif defined  CONFIG_SENSOR_NT99240_DEV2
	static __u32 u32Sensor = NT_99240;
#elif defined  CONFIG_SENSOR_NT99340_DEV2
	static __u32 u32Sensor = NT_99340;
#elif defined  CONFIG_SENSOR_HM1375_DEV2
	static __u32 u32Sensor = HM_1375;
#elif defined  CONFIG_SENSOR_HM1055_DEV2
	static __u32 u32Sensor = HM_1055;	
#elif defined  CONFIG_SENSOR_SP1628_DEV2
	static __u32 u32Sensor = SP_1628;	
#elif defined  CONFIG_SENSOR_TW9912_DEV2
	static __u32 u32Sensor = TW_9912;	
#elif defined  CONFIG_SENSOR_GC0308_DEV2
	static __u32 u32Sensor = GC_0308;
#elif defined  CONFIG_SENSOR_OV10633_DEV2
	static __u32 u32Sensor = OV_10633;		
#elif defined  CONFIG_SENSOR_SC1046_DEV2
	static __u32 u32Sensor = SC_1046;	
#elif defined  CONFIG_SENSOR_GM7150_DEV2
	static __u32 u32Sensor = GM_7150;
#elif defined  CONFIG_SENSOR_TW9900_DEV2
	static __u32 u32Sensor = TW_9900;
#elif defined  CONFIG_SENSOR_TVP5150_DEV2
	static __u32 u32Sensor = TVP_5150;	
#elif defined  CONFIG_SENSOR_XC7021_DEV2
	static __u32 u32Sensor = XC_7021;	
#else
# error "please select one sensor"
#endif


#define ERR_PRINTF		printk
//#undef  DBG_PRINTF
//#define DBG_PRINTF		printk
#define DBG_PRINTF(...)
//#define ENTER() 		printk("%s\n",__FUNCTION__)
#define ENTER(...) 


static unsigned int FramePackerAddr[CONFIG_VIN_DEV2_PREVIEW_BUF_NO]; 	/* Preview start address for packet buffer */
static planar_buf_t FramePlanarAddr[CONFIG_VIN_DEV2_ENCODE_BUF_NO]; 	/* Encode start address for planar buffer */

/* Packet Frame Buffer Pointer x3 */
volatile static videoIn_buf_t videoIn_preview_buf[CONFIG_VIN_DEV2_PREVIEW_BUF_NO]={	
					{0,0,{0,0},FRAME_READY},	//Packet Pipe buf 0 for preview
					{0,0,{0,0},FRAME_READY},	//Packet Pipe buf 1 for preview
					{0,0,{0,0},FRAME_READY}		//Packet Pipe buf 2 for preview
					};

#ifdef CONFIG_SUPPORT_SHARE_PACKET_BUFFER_DEV1	/* Share packet buffer from sensor port 1 */
extern videoIn_buf_t videoIn_share_preview_buf[];
extern unsigned int u32SyncSharePacketBuffer;
#endif
#ifdef CONFIG_SUPPORT_SHARE_PLANAR_BUFFER_DEV1	/* Share packet buffer from sensor port 1 */
extern videoIn_buf_t videoIn_share_encode_buf[];
#endif					
volatile static videoIn_buf_t videoIn_encode_buf[CONFIG_VIN_DEV2_ENCODE_BUF_NO]={	
					{0,0,{0,0},FRAME_READY},	//Planar Pipe buf 0 for encode
					{0,0,{0,0},FRAME_READY},	//Planar Pipe buf 1 for encode
//#ifdef CONFIG_SUPPORT_EXTRA_BUFFER_DEV2
#if CONFIG_VIN_DEV2_ENCODE_BUF_NO == 3
					{0,0,{0,0},FRAME_READY}		//Planar Pipe buf 2 for encode
#endif
					};			
//#ifdef CONFIG_MOTION_DETECTION_DEV2
volatile static videoin_motion_t videoIn_md_Offset_info[VIDEOIN_DIFF_BUFFER_NUM];//={0};
volatile static videoIn_buf_t videoIn_diff_buf[VIDEOIN_DIFF_BUFFER_NUM]={	
					{0,0,{0,0},FRAME_READY},	//OutY buf for motion detection ==> only for reference	
					{0,0,{0,0},FRAME_READY}		//Diff buf for motion detection	==> Valid to report to AP t			
					};				
//#endif
#if defined(CONFIG_W55FA92_VIDEOIN_DEV1)
extern unsigned int w55fa92_vin_dev1_v;	
extern unsigned int w55fa92_vin_dev1_p;	
#endif 
#if defined(CONFIG_W55FA92_VIDEOIN_DEV2)
extern unsigned int w55fa92_vin_dev2_v;	
extern unsigned int w55fa92_vin_dev2_p;	
#endif

//#define 	MPRINTF	printk
#define 	MPRINTF(...)	

#ifdef __BOOT_MEM__
static unsigned long videoin_dmamalloc_bootmem(unsigned int encode_size, 
												unsigned int preview_size, 
												unsigned int diff_size)
{
	unsigned int buf_v, buf_p;
	unsigned int buf;
	unsigned int total_size = 0; 
	videoin_priv_t *priv = (videoin_priv_t *)&videoin_priv;

	buf_v = w55fa92_vin_dev2_v; 
	buf_p = w55fa92_vin_dev2_p;
	DBG_PRINTF("memory allocate for port 2\n");

	MPRINTF("VideoIn 2 reserved buffer size  = 0x%x\n", priv->i32AllocBufSize);
	MPRINTF("VideoIn 2 remaining buffer size = 0x%x\n", priv->i32RemainingBufSize);
	MPRINTF("VideoIn 2 reserved start buffer addr = 0x%x\n", w55fa92_vin_dev2_p);
	for(buf=0; buf<CONFIG_VIN_DEV2_ENCODE_BUF_NO; buf=buf+1){
#ifdef CONFIG_SUPPORT_SHARE_PLANAR_BUFFER_DEV1	/* share sensor port 1 planar buffer */
		videoIn_encode_buf[buf].u32PhysAddr = videoIn_share_encode_buf[buf].u32PhysAddr;

		//!!!! priv->videombuf.offsets[i] information will base on the videoIn_encode_buf[buf].u32VirtAddr to calculate
		//!!!! if share planar. the value will be assigned to head of allocated memory to avoid the issue. 
		videoIn_encode_buf[buf].u32VirtAddr = buf_v;  //videoIn_share_encode_buf[buf].u32VirtAddr;

		MPRINTF("Shared Planar buf %d Phys addr = 0x%x\n", buf, videoIn_encode_buf[buf].u32PhysAddr);
		MPRINTF("Shared Planar buf %d Virt addr = 0x%x\n", buf, videoIn_encode_buf[buf].u32VirtAddr);
#else	
		videoIn_encode_buf[buf].u32PhysAddr = buf_p;
		videoIn_encode_buf[buf].u32VirtAddr = buf_v;
		MPRINTF("Planar buf %d Phys addr = 0x%x\n", buf, videoIn_encode_buf[buf].u32PhysAddr);
		MPRINTF("Planar buf %d Virt addr = 0x%x\n", buf, videoIn_encode_buf[buf].u32VirtAddr);		
		buf_p += encode_size;
		buf_v += encode_size;
		total_size += encode_size;
		priv->i32RemainingBufSize -= encode_size;
		if(priv->i32RemainingBufSize<0){
			printk("Specified memory is not enough, please modify kernel configuration file\n");
			return 0;
		}
#endif		
	}
	for(buf=0; buf<CONFIG_VIN_DEV2_PREVIEW_BUF_NO; buf=buf+1){
#ifdef CONFIG_SUPPORT_SHARE_PACKET_BUFFER_DEV1	/* share sensor port 1 packet buffer */
		videoIn_preview_buf[buf].u32PhysAddr = videoIn_share_preview_buf[buf].u32PhysAddr;

		//!!!! priv->videombuf.offsets[i] information will base on the videoIn_encode_buf[buf].u32VirtAddr to calculate
		//!!!! if share planar. the value will be assigned to head of allocated memory to avoid the issue. 
		videoIn_preview_buf[buf].u32VirtAddr = buf_v; //videoIn_share_preview_buf[buf].u32VirtAddr;

		MPRINTF("Shared Packet buf %d Phys addr = 0x%x\n", buf, videoIn_preview_buf[buf].u32PhysAddr);
		MPRINTF("Shared Packet buf %d Virt addr = 0x%x\n", buf, videoIn_preview_buf[buf].u32VirtAddr);
#else
		videoIn_preview_buf[buf].u32PhysAddr = buf_p;
		videoIn_preview_buf[buf].u32VirtAddr = buf_v;
		MPRINTF("Packet buf %d Phys addr = 0x%x\n", buf, videoIn_preview_buf[buf].u32PhysAddr);
		MPRINTF("Packet buf %d Virt addr = 0x%x\n", buf, videoIn_preview_buf[buf].u32VirtAddr);
		buf_p += preview_size;
		buf_v += preview_size;
		total_size += preview_size;
		priv->i32RemainingBufSize -= preview_size;
		if(priv->i32RemainingBufSize<0){
			printk("Specified memory is not enough, please modify kernel configuration file\n");
			return 0;
		}
#endif
	}	
#ifdef CONFIG_MOTION_DETECTION_DEV2
	for(buf=0; buf<VIDEOIN_DIFF_BUFFER_NUM; buf=buf+1){
		videoIn_diff_buf[buf].u32PhysAddr = buf_p;
		videoIn_diff_buf[buf].u32VirtAddr = buf_v;	
		MPRINTF("Diff buf %d Phys addr = 0x%x\n", buf, videoIn_diff_buf[buf].u32PhysAddr);
		MPRINTF("Diff buf %d Virt addr = 0x%x\n", buf, videoIn_diff_buf[buf].u32VirtAddr);
		buf_p += diff_size;
		buf_v += diff_size;
	}
#endif
#ifdef test_remove	
	buf_v = w55fa92_vin_dev1_v;
	while (total_size > 0) {
			SetPageReserved(vmalloc_to_page((void *)buf_v));
			buf_v += PAGE_SIZE;
			total_size -= PAGE_SIZE;
	}
#endif

	return videoIn_encode_buf[0].u32PhysAddr;
}
#else
static unsigned long videoin_dmamalloc_planar_phy(unsigned long size)
{
	videoin_priv_t *priv = (videoin_priv_t *)&videoin_priv;
	unsigned long adr;
	unsigned int i;
	int buf;
	unsigned long total_size;
	ENTER();

	for(buf=0; buf<CONFIG_VIN_DEV2_ENCODE_BUF_NO; buf=buf+1){
		total_size = size;
		total_size = PAGE_ALIGN(total_size);
		DBG_PRINTF("videoin planar buf size=%x", (UINT32)total_size);
		priv->vaddr = (u32)dma_alloc_writecombine(NULL/*dev*/, 
													total_size,
													&priv->paddr, 
													GFP_KERNEL);
		if (!priv->vaddr){
			printk("Alloc memory size %d fail: err = 0x%x\n", (int32_t)total_size, (int32_t)priv->vaddr);
			while((--buf)>0)
			{
				printk("free planar buf %d\n", buf);
				dma_free_writecombine(NULL/*dev*/,
					total_size,
					(void *)videoIn_encode_buf[buf].u32VirtAddr,
					videoIn_encode_buf[buf].u32PhysAddr);
			}	
			return 0;
		}
		memset((void *)priv->vaddr, 0, total_size); 					/* Clear the ram out, no junk to the user */
		adr = (unsigned long) priv->vaddr;
		
		videoIn_encode_buf[buf].u32PhysAddr = priv->paddr;
		videoIn_encode_buf[buf].u32VirtAddr = adr;
		
		while (total_size > 0) {
			SetPageReserved(vmalloc_to_page((void *)adr));
			adr += PAGE_SIZE;
			total_size -= PAGE_SIZE;
		}
	}

	return videoIn_encode_buf[0].u32PhysAddr;
}
static unsigned long videoin_dmamalloc_packet_phy(unsigned long size)
{
	videoin_priv_t *priv = (videoin_priv_t *)&videoin_priv;
	unsigned long adr;
	unsigned int i;
	int buf;
	unsigned long total_size;
	ENTER();
	for(buf=0; buf<CONFIG_VIN_DEV2_PREVIEW_BUF_NO; buf=buf+1){
		total_size = size;
		total_size = PAGE_ALIGN(total_size);
		DBG_PRINTF("videoin packet buf size=%x", (UINT32)total_size);
		priv->vaddr = (u32)dma_alloc_writecombine(NULL/*dev*/, 
													total_size,
													&priv->paddr, 
													GFP_KERNEL);
		if (!priv->vaddr){
			printk("Alloc memory size %d fail: err = 0x%x\n", (int32_t)total_size, (int32_t)priv->vaddr);
			while((--buf)>0)
			{
				printk("free packet buf %d\n", buf);
				dma_free_writecombine(NULL/*dev*/,
					total_size,
					(void *)videoIn_preview_buf[buf].u32VirtAddr,
					videoIn_preview_buf[buf].u32PhysAddr);
			}	
			return 0;
		}
		memset((void *)priv->vaddr, 0, total_size); 					/* Clear the ram out, no junk to the user */
		adr = (unsigned long) priv->vaddr;
		
		videoIn_preview_buf[buf].u32PhysAddr = priv->paddr;
		videoIn_preview_buf[buf].u32VirtAddr = adr;
		
		while (total_size > 0) {
			SetPageReserved(vmalloc_to_page((void *)adr));
			adr += PAGE_SIZE;
			total_size -= PAGE_SIZE;
		}
	}
	return priv->paddr;
}

static void videoin_free_dmamalloc_planar_phy(unsigned long planar_size)
{
	unsigned long total_size;		
	int buf;
	ENTER();	
	total_size = planar_size;
	total_size = PAGE_ALIGN(total_size);
	for(buf=0; buf<CONFIG_VIN_DEV2_ENCODE_BUF_NO; buf=buf+1){
		dma_free_writecombine(NULL/*dev*/,
					total_size,
					(void *)videoIn_encode_buf[buf].u32VirtAddr,
					videoIn_encode_buf[buf].u32PhysAddr);			
	}
}
static void videoin_free_dmamalloc_packet_phy(unsigned long packet_size)
{
	unsigned long total_size;	
	int buf;
	ENTER();	
	total_size = packet_size;
	total_size = PAGE_ALIGN(total_size);
	for(buf=0; buf<CONFIG_VIN_DEV2_PREVIEW_BUF_NO; buf=buf+1){
		dma_free_writecombine(NULL/*dev*/,
					total_size,
					(void *)videoIn_preview_buf[buf].u32VirtAddr,
					videoIn_preview_buf[buf].u32PhysAddr);			
	}
}	
#endif

//#ifdef CONFIG_W55FA92_VIDEOIN_DEV1
//INT32 register_vin_port1_ioctl(VINIOCTL_T* pVinIoctl);
//INT32 register_vin_port1_device(VINDEV_T* pVinDev);
//#endif 
#ifdef CONFIG_W55FA92_VIDEOIN_DEV2
INT32 register_vin_port2_ioctl(VINIOCTL_T* pVinIoctl);
INT32 register_vin_port2_device(VINDEV_T* pVinDev);
#endif 
extern unsigned int get_vin_dev2_buffer_size(void);
static int videoin_open(struct file *file)
{
	struct video_device *dev = video_devdata(file);
	videoin_priv_t *priv = (videoin_priv_t *)dev->priv;
	//int buf_num;

    /*	Critical section */
    if (! atomic_dec_and_test(&vin_avail)) {
        atomic_inc(&vin_avail);
        return -EBUSY;
    }
    printk("VideoIn port 2 opened(%d)\n", vin_avail.counter);
#if 0
		for(buf_num=0; buf_num<CONFIG_VIN_DEV2_PREVIEW_BUF_NO; buf_num=buf_num+1){
			unsigned int* pu32Addr = (unsigned int*)videoIn_preview_buf[buf_num].u32VirtAddr;	
			unsigned int i;
			for(i=0; i<priv->i32packet_width*priv->i32packet_height*16/8;i=i+4){//Clear packet buffer, Black in YUV422 is Y=0x0, U=V=0x80 
				*pu32Addr++=0x80008000; //2 Pixel
			}					
		}
#endif
		w55fa92_VIN_PAC_BUFFER = videoIn_preview_buf[0].u32PhysAddr;	
		/* Driver Layer */
#if 0
	//#ifdef CONFIG_W55FA92_VIDEOIN_DEV1
		register_vin_port1_device(&DevVin);	
		priv->pDevVin = (VINDEV_T*)&DevVin;

		/* IOCTL function */
		register_vin_port1_ioctl(&DevIoctl);
		priv->pDevIoctl = &DevIoctl;
	//#endif
#endif
#if 1
	//#ifdef CONFIG_W55FA92_VIDEOIN_DEV2
		register_vin_port2_device(&DevVin);	
		priv->pDevVin = (VINDEV_T*)&DevVin;
		/* IOCTL function */
		register_vin_port2_ioctl(&DevIoctl);
		priv->pDevIoctl = &DevIoctl;
	//#endif
#endif	
		priv->i32FrameNumber = -1;
		priv->i32ReadFrameNum = 0;
		priv->grab_sync = -1;
		priv->start_end_capture = 0;
		priv->videommap.frame = 0;	

		priv->i32UserPacketStride = 0;	//New create for dual sensor
		priv->i32UserPlanarStride = 0;	//New crate for dual sensor
		priv->i32UserPlanarHeight = 0;	//New create for dual sensor V tile
		priv->i32planar_pos_x = 0;			//New create for dual sensor
		priv->i32planar_pos_y = 0;			//New create for dual sensor
	//	bIsVideoIn2Enable = 1;

		if(priv->sensor_intf->sensor_poweron)
			priv->sensor_intf->sensor_poweron(TRUE);

		if(priv->sensor_intf->sensor_suspend)
			priv->sensor_intf->sensor_suspend(FALSE);

		if(priv->sensor_intf->sensor_init){
			if (priv->sensor_intf->sensor_init(u32Sensor, priv) < 0){
				outp32(REG_LCM_FSADDR, priv->u32FbPhyAddr);
				printk("Init Sensor fail\n");
				atomic_inc(&vin_avail);
				return -EBUSY;
			}
			else
				priv->i32SensorID=u32Sensor;
		}	
		//vin_set_init(file);
		priv->pDevVin->EnableInt(eVIDEOIN_VINT);		
		
	printk("video driver open successful\n");
	return 0;
}

static int videoin_close(struct file *file)
{
	struct video_device *dev = video_devdata(file);
	videoin_priv_t *priv = (videoin_priv_t *)dev->priv;

	printk("%s: current_task:%s pid:%d\n", __FUNCTION__, current->comm, current->pid);
	if((inp32(REG_VPECTL+priv->u32PortAddr)&(PKEN|VPEEN))==(PKEN|VPEEN))
	{//
#ifndef CONFIG_SHARE_SENSOR 
		priv->pDevVin->SetOperationMode(TRUE);			/* One shutter mode */
		while(priv->pDevVin->GetOperationMode()==TRUE);
#endif
	}
#if defined(CONFIG_SENSOR_GM7150_DEV2) || defined(CONFIG_SENSOR_TW9900_DEV2) || defined(CONFIG_SENSOR_TVP5150_DEV2)	
	priv->pDevVin->SetPipeEnable(TRUE, eVIDEOIN_BOTH_PIPE_DISABLE);
	priv->pDevVin->DisableInt(eVIDEOIN_VINT);
	priv->pDevVin->SetPipeEnable(FALSE, eVIDEOIN_BOTH_PIPE_DISABLE);
	priv->pDevVin->DisableInt(eVIDEOIN_VINT);
#endif	 
	//if(priv->sensor_intf->sensor_reset) 
		//priv->sensor_intf->sensor_reset(TRUE);			

	bIsVideoIn2Enable = 0;//for preview	
	priv->pDevVin->Close();	
#ifndef CONFIG_SHARE_SENSOR									/* don't restore VPOST address if share sensor */
	outp32(REG_LCM_FSADDR, priv->u32FbPhyAddr);
	//priv->pDevVin->Close();									
	/* If shared sensor, don't close engine due to kernel configuration default choice GPB[1:0] as interface SCKO and SPCK*/
	/* Close driver 2 if shared sensor will result in driver 1 pins are switched to GPIO */
	if(priv->sensor_intf->sensor_suspend)		
		priv->sensor_intf->sensor_suspend(TRUE);		/* Sensor suspend */
	if(priv->sensor_intf->sensor_poweron)				/* Sensor power off */
		priv->sensor_intf->sensor_poweron(FALSE);
#endif		

	atomic_inc(&vin_avail);
	return 0;
}
/*
* read data after encode/decode finished
*/
static ssize_t videoin_read(struct file *file, char __user *buf,
		      size_t count, loff_t *ppos)
{
	struct video_device *dev = video_devdata(file);
	videoin_priv_t *priv = (videoin_priv_t *)dev->priv;

	/* int nonblock = file->f_flags & O_NONBLOCK; */
	int ret, size;

	ENTER();
	
	if(videoIn_encode_buf[priv->i32ReadFrameNum].u8FrameState != FRAME_CAPTUREDONE)
	{
		DBG_PRINTF("read frame %d FrameState %d\n", priv->i32ReadFrameNum, videoIn_encode_buf[priv->i32ReadFrameNum].u8FrameState);
		interruptible_sleep_on( &videoin_wq );
	}

	size = priv->videowin.height * priv->videowin.width *16/8;
	DBG_PRINTF("Packet W*H = %d * %d \n", priv->videowin.width , priv->videowin.height);
	priv->vaddr = videoIn_encode_buf[priv->i32ReadFrameNum].u32VirtAddr;

	down(&priv->lock);
	if (size >= count)
		size = count;

	DBG_PRINTF("Dst buf addr = 0x%x\n", (UINT32)buf);
	DBG_PRINTF("Src buf addr = 0x%x\n", (UINT32)(priv->vaddr));
	if (copy_to_user(buf, (void *)priv->vaddr, size))
	{
		ERR_PRINTF("videoin_read copy_to_user error\n");
		ret = -EFAULT;
		goto out;
	}
	*ppos += size;
	up(&priv->lock);
	
	videoIn_encode_buf[priv->i32ReadFrameNum].u8FrameState = FRAME_READY;
	if(priv->i32FrameNumber == -1)
	{
		priv->i32FrameNumber = priv->i32ReadFrameNum;
		priv->pDevVin->SetBaseStartAddress(eVIDEOIN_PACKET, 0, videoIn_encode_buf[priv->i32FrameNumber].u32PhysAddr);	
		priv->pDevVin->SetShadowRegister();	
		videoIn_encode_buf[priv->i32FrameNumber].u8FrameState = FRAME_GRABBING;
		DBG_PRINTF("Read capture %d\n", priv->i32FrameNumber);
	}
	else
	{
		DBG_PRINTF("NOT read capture %d\n",priv->i32ReadFrameNum);
	}
	priv->i32ReadFrameNum = (priv->i32ReadFrameNum+1)%CONFIG_VIN_DEV2_ENCODE_BUF_NO;

	ret = size;
out:
	return ret;
}

static int videoin_mmap (struct file *file, struct vm_area_struct *vma)
{
	/* struct video_device *dev = video_devdata(file); */
	/* videoin_priv_t *priv = (videoin_priv_t *)dev->priv; */
	//unsigned long start = vma->vm_start;
	unsigned long size  = vma->vm_end-vma->vm_start;
	//unsigned long page, pos;
	unsigned long pos;
	struct video_device *dev = video_devdata(file);
	videoin_priv_t *priv = (videoin_priv_t *)dev->priv;
	ENTER();
#if defined(CONFIG_SUPPORT_SHARE_PLANAR_BUFFER_DEV1)
	/* If shared planar buffer, mapping start address from preview buffer */
	DBG_PRINTF("Mapping packet buf = %d\n", priv->i32MappingBuf);
	pos = videoIn_preview_buf[0].u32PhysAddr;
#else
	if(priv->i32MappingBuf < CONFIG_VIN_DEV2_ENCODE_BUF_NO)
	{
		DBG_PRINTF("Mapping planar buf = %d\n", priv->i32MappingBuf);
		pos = videoIn_encode_buf[priv->i32MappingBuf].u32PhysAddr;
	}
	else
	{
		DBG_PRINTF("Mapping packet buf = %d\n", priv->i32MappingBuf);
		pos = videoIn_preview_buf[(priv->i32MappingBuf-CONFIG_VIN_DEV2_ENCODE_BUF_NO)].u32PhysAddr;
	}
#endif
	vma->vm_flags |= (VM_IO | VM_RESERVED);
	vma->vm_page_prot = pgprot_noncached (vma->vm_page_prot);

	if (remap_pfn_range (vma, vma->vm_start, (unsigned long)pos >> PAGE_SHIFT, size, vma->vm_page_prot) < 0) {
		DBG_PRINTF("Vin mmap fail\n");
       		return -EIO;
    }
	return 0;
#if 0
#ifdef CONFIG_SUPPORT_SHARE_MEMORY_DEV1
	pos = videoIn_encode_buf[0].u32PhysAddr;
	vma->vm_flags |= (VM_IO | VM_RESERVED);
	vma->vm_page_prot = pgprot_noncached (vma->vm_page_prot);
		
	//printk("Bitstream mmap\n");
	if (remap_pfn_range (vma, vma->vm_start, (unsigned long)pos >> PAGE_SHIFT, size, vma->vm_page_prot) < 0) {
		DBG_PRINTF("Vin mmap fail\n");
       		return -EIO;
       	}
	return 0;		
#else
	if(priv->i32MappingBuf < ONFIG_VIN_DEV2_ENCODE_BUF_NO)
	{
		DBG_PRINTF("Mapping planar buf = %d\n", priv->i32MappingBuf);
		pos = videoIn_encode_buf[priv->i32MappingBuf].u32VirtAddr;
	}
	else
	{
		DBG_PRINTF("Mapping packet buf = %d\n", priv->i32MappingBuf);
		pos = videoIn_preview_buf[(priv->i32MappingBuf-ONFIG_VIN_DEV2_ENCODE_BUF_NO)].u32VirtAddr;
	}
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
#endif
#endif
}


//static long videoin_ioctl(struct file *file,
//				 unsigned int cmd, void *arg)
static long videoin_ioctl(struct file *file, unsigned int cmd, void *arg)
{
	int ret = 0;
	//ENTER();
	struct video_device *dev = video_devdata(file);
	videoin_priv_t *priv = (videoin_priv_t *)dev->priv;

	//printk("cmd = %d\n",cmd);
	//printk("videoin ioctl, cmd =											 %d\n", cmd);
	//printk("videoin ioctl, cmd = 											 %d\n", cmd);
	//printk("videoin ioctl, cmd = 											 %d\n", cmd);
	switch (cmd) {
		case VIDIOCGCAP:
			DBG_PRINTF("VIDIOCGCAP cmd\n");
			ret = priv->pDevIoctl->get_capability(file,
								 			cmd,
											arg);
			break;
		case VIDIOCGWIN:
			ret = priv->pDevIoctl->get_overlay_window(file,
									 			cmd,
												arg);
			break;
		case VIDIOCSWIN:
			ret = priv->pDevIoctl->set_overlay_window(file,
									 			cmd,
												arg);
			break;
		case VIDIOCGPICT:
			ret = priv->pDevIoctl->get_picture_properties(file,
										 			cmd,
													arg);
			break;
		case VIDIOCSPICT:
			ret = priv->pDevIoctl->set_picture_properties(file,
										 			cmd,
													arg);
			break;
		case VIDIOCCAPTURE:
			ret = priv->pDevIoctl->start_end_capture(file,
									 			cmd,
												arg);
			break;
		case VIDIOCMCAPTURE:
			ret = priv->pDevIoctl->grab_frames(file,
							 			cmd,
										arg);
			break;
		case VIDIOCGMBUF:
			ret = priv->pDevIoctl->get_map_buffer_info(file,
									 			cmd,
												arg);
			break;
		case VIDIOCSYNC:
			ret = priv->pDevIoctl->sync(file,
						 			cmd,
									arg);
			break;

		case VIDIOCGCAPTIME:
			ret = priv->pDevIoctl->get_capture_time(file,
									 			cmd,
												arg);
			break;
		case VIDIOCGSYSUPTIME:
			ret = priv->pDevIoctl->get_sys_up_time(file,
								 			cmd,
											arg);
			break;
		case VIDIOCSBRIGHTNESS:
			ret = priv->pDevIoctl->set_brightness(file,
								 			cmd,
											arg);
			break;
		case VIDIOCGBRIGHTNESS:
			ret = priv->pDevIoctl->get_brightness(file,
								 			cmd,
											arg);
			break;
		case VIDIOCSCONTRAST:
			ret = priv->pDevIoctl->set_contrast(file,
								 			cmd,
											arg);
			break;
		case VIDIOCGCONTRAST:
			ret = priv->pDevIoctl->get_contrast(file,
								 			cmd,
											arg);
			break;
		case VIDIOCSSHARPNESS:
			ret = priv->pDevIoctl->set_sharpness(file,
								 			cmd,
											arg);
			break;
		case VIDIOCGSHARPNESS:
			ret = priv->pDevIoctl->get_sharpness(file,
								 			cmd,
											arg);
			break;
		case VIDIOCSWHITEBALANCE:
			ret = priv->pDevIoctl->set_white_balance(file,
									 			cmd,
												arg);
			break;
		case VIDIOCGWHITEBALANCE:
			ret = priv->pDevIoctl->get_white_balance(file,
									 			cmd,
												arg);
			break;
		case VIDIOCSNOISEREDUCTION:
			ret = priv->pDevIoctl->set_noise_reduction(file,
									 			cmd,
												arg);
			break;
		case VIDIOCGNOISEREDUCTION:
			ret = priv->pDevIoctl->get_noise_reduction(file,
										 			cmd,
													arg);
			break;
		case VIDIOCSCOLORSATURATION:
			ret = priv->pDevIoctl->set_color_saturation(file,
										 			cmd,
													arg);
			break;
		case VIDIOCGCOLORSATURATION:
			ret = priv->pDevIoctl->get_color_saturation(file,
										 			cmd,
													arg);
			break;
		case VIDIOCSFLICKERFREQ:
			ret = priv->pDevIoctl->set_flicker_freq(file,
									 			cmd,
												arg);
			break;
		case VIDIOCSIRLED:
			ret = priv->pDevIoctl->set_IR_led(file,
							 			cmd,
										arg);
			break;
		case VIDIOCGIRLEDONOFF:
			ret = priv->pDevIoctl->get_IR_led(file,
							 			cmd,
										arg);
			break;
		case VIDIOCSPREVIEW:
		{
			/* struct video_device *dev = video_devdata(file); */
			int i32IsEnablePreview;
			i32IsEnablePreview = (int)arg;
			priv->i32IsEnablePreview = i32IsEnablePreview;
			if(i32IsEnablePreview){
				DBG_PRINTF("Enable preview \n");
				bIsVideoIn2Enable = 1;
				w55fa92_VIN_PAC_BUFFER = videoIn_preview_buf[0].u32PhysAddr;
			}
			else{
				DBG_PRINTF("Disabel preview \n");			
				bIsVideoIn2Enable = 0;
				outp32(REG_LCM_FSADDR, priv->u32FbPhyAddr);
			}
			DBG_PRINTF("\nbIsVideoIn2Enable = %d\n", bIsVideoIn2Enable);
		}
			break;			
//#ifdef CONFIG_ZOOM
		/* Zooming { */
		case VIDIOC_CROPCAP:
			priv->pDevIoctl->get_cropping_capability(file,
									 			cmd,
												arg);
			break;
		case	VIDIOC_G_CROP:
			priv->pDevIoctl->get_cropping_window(file,
								 			cmd,
											arg);
			break;
		case 	VIDIOC_S_CROP:
			priv->pDevIoctl->set_cropping_window(file,
								 			cmd,
											arg);
			break;	
		/* Zooming } */
//#endif
		/* User ctrl { */
		case VIDIOC_QUERYCTRL:
			ret = priv->pDevIoctl->query_user_control(file,
												cmd,
												arg);
			break;
		case VIDIOC_QUERY_SENSOR_ID:
           	ret = priv->pDevIoctl->query_sensor_id(file,
		                                            cmd,
		                                            arg);
                        break;
		case VIDIOC_G_CTRL:			
		case VIDIOC_S_CTRL:
			return priv->pDevIoctl->user_ctrl(file,
										cmd,
										arg);
			break;
		/* User ctrl } */
//#ifdef CONFIG_MOTION_DETECTION_DEV2
		/* Motion Detect { */
		case VIDIOC_G_DIFF_OFFSET:	// Non-standard	ioctl 
			ret =priv->pDevIoctl->query_diff_offset(file,
												cmd,
												arg);	
			break;
		case VIDIOC_G_DIFF_SIZE:	// Non-standard	ioctl 
			ret = priv->pDevIoctl->query_diff_size(file,
											cmd,
											arg);	
			break;
		case VIDIOC_S_MOTION_THRESHOLD:	// Non-standard	ioctl
			priv->pDevIoctl->set_motion_threshold(file,
											cmd,
											arg);
			break;
		/* Motion Detect } */
//#endif	



		case VIDIOC_G_BUFINFO:
		{
			S_BUF_INFO sBufInfo;
	#ifdef CONFIG_SUPPORT_SHARE_MEMORY_DEV2	
			sBufInfo.u32RemainBufSize = w55fa92_vde_total_size;
			sBufInfo.u32RemainBufPhyAdr = w55fa92_vde_physaddr_start;
	#else
			sBufInfo.u32RemainBufSize = 0;
			sBufInfo.u32RemainBufPhyAdr = 0;
	#endif
			if (copy_to_user((void *)arg, (void*)&sBufInfo, sizeof(S_BUF_INFO))) {
				ERR_PRINTF("copy_to_user error VIDIOC_G_BUFINFO\n");
				return -EFAULT;
			}

		}
		break;
		
		case VIDIOC_G_PACKET_INFO: // Non-standard for map packet address.
			ret = priv->pDevIoctl->query_packet_info(file,
														cmd,
														arg);	
			break;
		case VIDIOC_G_PLANAR_INFO: // Non-standard for map packet address.
			ret = priv->pDevIoctl->query_planar_info(file,
														cmd,
														arg);	
			break;
		case VIDIOC_S_MAP_BUF:
			DBG_PRINTF("VIDIOC_S_MAP_BUF cmd receive\n");
			ret = priv->pDevIoctl->set_mapping_buffer(file,
														cmd,	
														arg);
			break;
		case VIDIOC_G_KERNEL_INFO:
			{
				S_KERNEL_CONFIG sKernelConfig;

			#ifdef CONFIG_SHARE_SENSOR
				sKernelConfig.i32SharedSensor =1;	/* Share Sensor */
			#else
				sKernelConfig.i32SharedSensor =0;	/* Independent sensor */
			#endif				

			#ifdef CONFIG_SUPPORT_SHARE_PACKET_BUFFER_DEV1
				sKernelConfig.i32DependentPacketBuf =1;	/* Combination video0 and video1 packet buffer */
			#else
				sKernelConfig.i32DependentPacketBuf =0;	/* Independent packet buffer */
			#endif				
			
			#ifdef CONFIG_SUPPORT_SHARE_PLANAR_BUFFER_DEV1
				sKernelConfig.i32DependentPlanarBuf =1;	/* Combination video0 and video1 planar buffer */
			#else
				sKernelConfig.i32DependentPlanarBuf =0;	/* Independent planar buffer */
			#endif

           	 	if (copy_to_user((void*)arg, (void*)&sKernelConfig, sizeof(S_KERNEL_CONFIG))) {
                	ERR_PRINTF("Error copy_to_user VIDIOC_G_KERNEL_INFO \n");
                	ret = -EFAULT;
        		}	        
			}	
            break;
		case VIDIOC_S_PACKET_STRIDE:	/* The function need to be called before VIDIOCSWIN */
			/* User specified packet stride */
			priv->i32UserPacketStride = (int)arg;
			DBG_PRINTF("\nUser specified packet stride = %d\n", priv->i32UserPacketStride);
			break;		

		case VIDIOC_S_PLANAR_STRIDE:	/* The function need to be called before VIDIOC_S_CROP */
			/* User specified planar stride */
			priv->i32UserPlanarStride = (int)arg;
			DBG_PRINTF("\nUser specified planar stride = %d\n", priv->i32UserPlanarStride);
			break;	
		case VIDIOC_S_PLANAR_HEIGHT:	/* The function need to be called before VIDIOC_S_CROP */
			/* User specified planar height */
			priv->i32UserPlanarHeight = (int)arg;
			printk("\nUser specified planar height = %d\n", priv->i32UserPlanarHeight);
			break;	
		case VIDIOC_S_PLANAR_POSITION:
			/* User specified planar start position*/
			DBG_PRINTF("\nUser specified planar start position = %d\n", priv->i32UserPlanarStride);
			ret = priv->pDevIoctl->set_planar_position(file,
														cmd,
														arg);	
		break;		
		default:
			return -ENOIOCTLCMD;
	}
	return ret;
}

static struct v4l2_file_operations  videoin_fops = {
	.owner 		=  	THIS_MODULE,
	.open 		=   videoin_open,
	.release 	=	videoin_close,
	.read 		=   videoin_read,
	.mmap 		=   videoin_mmap,
	.ioctl 		=  	videoin_ioctl,
	.unlocked_ioctl = 	videoin_ioctl,
	//.llseek = no_llseek,
};
//#define DBG_REGISTER_MESSAGE
//#define DBG_FRAME_INT
static irqreturn_t irq_handler(int irq, void *dev_id)
{
	UINT32 u32IntStatus;
	videoin_priv_t *priv = (videoin_priv_t *)&videoin_priv;

	UINT32 u32PacAddr, u32PlaAddr;
	BOOL bEngEnable;
	E_VIDEOIN_PIPE ePipeEnable;
#ifdef DBG_FRAME_INT
	UINT32 i;	
	static UINT32 uframe =0;		
	{
		static UINT32 u32Frame = 0;
		u32Frame = u32Frame+1;
		if((u32Frame%100) ==0)
			printk("IRQ2 Frame = %x\n", u32Frame);
	}
#endif
	priv->pDevVin->GetBaseStartAddress(eVIDEOIN_PLANAR, 0, &u32PlaAddr);
	priv->pDevVin->GetBaseStartAddress(eVIDEOIN_PACKET, 0, &u32PacAddr);
	priv->pDevVin->GetPipeEnable(&bEngEnable, &ePipeEnable);

#if 0 //sync planar and packet with same buffer
	/* Preview pipe buffer switching */
	if((ePipeEnable & eVIDEOIN_PACKET) == eVIDEOIN_PACKET) {
		/*Packet pipe enable*/
		if(u32PacAddr==priv->pFramePackerAddr[0]){
			priv->pDevVin->SetBaseStartAddress(eVIDEOIN_PACKET, 0, priv->pFramePackerAddr[1]);
			w55fa92_VIN_PAC_BUFFER = videoIn_preview_buf[0].u32PhysAddr;	
		}
		else if(u32PacAddr==priv->pFramePackerAddr[1]){		
			priv->pDevVin->SetBaseStartAddress(eVIDEOIN_PACKET, 0, priv->pFramePackerAddr[2]);
			w55fa92_VIN_PAC_BUFFER = videoIn_preview_buf[1].u32PhysAddr;	
		}
		else if(u32PacAddr==priv->pFramePackerAddr[2]){
			priv->pDevVin->SetBaseStartAddress(eVIDEOIN_PACKET, 0, priv->pFramePackerAddr[0]);
			w55fa92_VIN_PAC_BUFFER = videoIn_preview_buf[2].u32PhysAddr;
		}
	}
#endif 							
	
//#ifdef CONFIG_SUPPORT_EXTRA_BUFFER_DEV2
#if CONFIG_VIN_DEV2_ENCODE_BUF_NO == 3
	if(priv->i32FrameNumber!=-1)
	{
		if (u32PlaAddr==priv->pFramePlanarAddr[0].u32PlaY){
			/* Current use YUV 0. Switch to another one buffer	*/
			if(priv->grab_sync != 1){
				priv->videoIn_encode_buf[0].u8FrameState = FRAME_CAPTUREDONE;	
				priv->videoIn_encode_buf[1].u8FrameState = FRAME_GRABBING;		
				priv->pDevVin->SetBaseStartAddress(eVIDEOIN_PLANAR, 0, priv->pFramePlanarAddr[1].u32PlaY);		
				priv->pDevVin->SetBaseStartAddress(eVIDEOIN_PLANAR, 1, priv->pFramePlanarAddr[1].u32PlaU);	
				priv->pDevVin->SetBaseStartAddress(eVIDEOIN_PLANAR, 2, priv->pFramePlanarAddr[1].u32PlaV);	
	
			#ifdef CONFIG_SUPPORT_SHARE_PACKET_BUFFER_DEV1	/* To sych packet with sensor 1 */
								
			#else
				priv->pDevVin->SetBaseStartAddress(eVIDEOIN_PACKET, 0, priv->pFramePackerAddr[1]);
				if(bIsVideoIn2Enable == TRUE)
					w55fa92_VIN_PAC_BUFFER = videoIn_preview_buf[0].u32PhysAddr;		
			#endif
	
				jiffies_to_timeval(jiffies - INITIAL_JIFFIES, (struct timeval*)&priv->videoIn_encode_buf[0].tvCapTime);
			}else{
				videoIn_encode_buf[0].u8FrameState = FRAME_GRABBING;	
			}
		}	
		else if(u32PlaAddr==priv->pFramePlanarAddr[1].u32PlaY){
			if(priv->grab_sync != 2){					
				priv->videoIn_encode_buf[1].u8FrameState = FRAME_CAPTUREDONE;	
				priv->videoIn_encode_buf[2].u8FrameState = FRAME_GRABBING;	
				priv->pDevVin->SetBaseStartAddress(eVIDEOIN_PLANAR, 0, priv->pFramePlanarAddr[2].u32PlaY);		
				priv->pDevVin->SetBaseStartAddress(eVIDEOIN_PLANAR, 1, priv->pFramePlanarAddr[2].u32PlaU);	
				priv->pDevVin->SetBaseStartAddress(eVIDEOIN_PLANAR, 2, priv->pFramePlanarAddr[2].u32PlaV);	
		
			#ifdef CONFIG_SUPPORT_SHARE_PACKET_BUFFER_DEV1	/* To sych packet with sensor 1 */					
			
			#else
				priv->pDevVin->SetBaseStartAddress(eVIDEOIN_PACKET, 0, priv->pFramePackerAddr[2]);
				if(bIsVideoIn2Enable == TRUE)				
					w55fa92_VIN_PAC_BUFFER = videoIn_preview_buf[1].u32PhysAddr;			
			#endif	
				jiffies_to_timeval(jiffies - INITIAL_JIFFIES, (struct timeval*)&priv->videoIn_encode_buf[1].tvCapTime);
			}else{
				videoIn_encode_buf[1].u8FrameState = FRAME_GRABBING;	
			}		
		}
		else if(u32PlaAddr==priv->pFramePlanarAddr[2].u32PlaY){
			if(priv->grab_sync != 0){		
				priv->videoIn_encode_buf[2].u8FrameState = FRAME_CAPTUREDONE;	
				priv->videoIn_encode_buf[0].u8FrameState = FRAME_GRABBING;	
				priv->pDevVin->SetBaseStartAddress(eVIDEOIN_PLANAR, 0, priv->pFramePlanarAddr[0].u32PlaY);		
				priv->pDevVin->SetBaseStartAddress(eVIDEOIN_PLANAR, 1, priv->pFramePlanarAddr[0].u32PlaU);	
				priv->pDevVin->SetBaseStartAddress(eVIDEOIN_PLANAR, 2, priv->pFramePlanarAddr[0].u32PlaV);	
		
			#ifdef CONFIG_SUPPORT_SHARE_PACKET_BUFFER_DEV1	/* To sych packet with sensor 1 */					
			
			#else
				priv->pDevVin->SetBaseStartAddress(eVIDEOIN_PACKET, 0, priv->pFramePackerAddr[0]);
				if(bIsVideoIn2Enable == TRUE)
					w55fa92_VIN_PAC_BUFFER = videoIn_preview_buf[2].u32PhysAddr;
			#endif	
				jiffies_to_timeval(jiffies - INITIAL_JIFFIES, (struct timeval*)&priv->videoIn_encode_buf[2].tvCapTime);
			}else{
				videoIn_encode_buf[2].u8FrameState = FRAME_GRABBING;	
			}		
		}
	}	
#else
	if(u32PlaAddr==priv->pFramePlanarAddr[0].u32PlaY){
		/* Current use YUV 0. Switch to another one buffer	*/
		if(priv->grab_sync != 1){
			priv->videoIn_encode_buf[0].u8FrameState = FRAME_CAPTUREDONE;	
			priv->videoIn_encode_buf[1].u8FrameState = FRAME_GRABBING;		
			priv->pDevVin->SetBaseStartAddress(eVIDEOIN_PLANAR, 0, priv->pFramePlanarAddr[1].u32PlaY);		
			priv->pDevVin->SetBaseStartAddress(eVIDEOIN_PLANAR, 1, priv->pFramePlanarAddr[1].u32PlaU);	
			priv->pDevVin->SetBaseStartAddress(eVIDEOIN_PLANAR, 2, priv->pFramePlanarAddr[1].u32PlaV);	

			jiffies_to_timeval(jiffies - INITIAL_JIFFIES, (struct timeval*)&priv->videoIn_encode_buf[0].tvCapTime);
		}else{
			videoIn_encode_buf[0].u8FrameState = FRAME_GRABBING;	
		}			
	}
	else {
		/* Current use YUV 1. Switch to another one buffer */	
		if(priv->grab_sync != 0){
			priv->videoIn_encode_buf[1].u8FrameState = FRAME_CAPTUREDONE;	
			priv->videoIn_encode_buf[0].u8FrameState = FRAME_GRABBING;	
			priv->pDevVin->SetBaseStartAddress(eVIDEOIN_PLANAR, 0, priv->pFramePlanarAddr[0].u32PlaY);		
			priv->pDevVin->SetBaseStartAddress(eVIDEOIN_PLANAR, 1, priv->pFramePlanarAddr[0].u32PlaU);	
			priv->pDevVin->SetBaseStartAddress(eVIDEOIN_PLANAR, 2, priv->pFramePlanarAddr[0].u32PlaV);	
			
			jiffies_to_timeval(jiffies - INITIAL_JIFFIES, (struct timeval*)&priv->videoIn_encode_buf[1].tvCapTime);
		}else{
			priv->videoIn_encode_buf[1].u8FrameState = FRAME_GRABBING;	
		}
	}
	/* Switch Preview Buffer */
  #ifdef CONFIG_SUPPORT_SHARE_PACKET_BUFFER_DEV1	/* To sych packet with sensor 1 */					
			
  #else
	if(u32PacAddr == priv->pFramePackerAddr[0]){
		priv->pDevVin->SetBaseStartAddress(eVIDEOIN_PACKET, 0, priv->pFramePackerAddr[1]);
		if(bIsVideoIn2Enable==TRUE)
			w55fa92_VIN_PAC_BUFFER = videoIn_preview_buf[0].u32PhysAddr;
	}else if(u32PacAddr == priv->pFramePackerAddr[1]){
		priv->pDevVin->SetBaseStartAddress(eVIDEOIN_PACKET, 0, priv->pFramePackerAddr[2]);
		if(bIsVideoIn2Enable==TRUE)
			w55fa92_VIN_PAC_BUFFER = videoIn_preview_buf[1].u32PhysAddr;
	}else if(u32PacAddr == priv->pFramePackerAddr[2]){
		priv->pDevVin->SetBaseStartAddress(eVIDEOIN_PACKET, 0, priv->pFramePackerAddr[0]);
		if(bIsVideoIn2Enable==TRUE)
			w55fa92_VIN_PAC_BUFFER = videoIn_preview_buf[2].u32PhysAddr;
	}
  #endif
#endif

#ifdef CONFIG_SUPPORT_SHARE_PACKET_BUFFER_DEV1
	if(u32SyncSharePacketBuffer==0){					
		priv->pDevVin->SetBaseStartAddress(eVIDEOIN_PACKET, 0, priv->pFramePackerAddr[0]);	
		//printk("B:0\n");
	}
	else if(u32SyncSharePacketBuffer==1){	
		priv->pDevVin->SetBaseStartAddress(eVIDEOIN_PACKET, 0, priv->pFramePackerAddr[1]);
		//printk("B:1\n");
	}
	else if(u32SyncSharePacketBuffer==2){
		priv->pDevVin->SetBaseStartAddress(eVIDEOIN_PACKET, 0, priv->pFramePackerAddr[2]);
		//printk("B:2\n");
	}
#endif
			
#ifdef DBG_REGISTER_MESSAGE 
	uframe = uframe+1;
	if(uframe%200==0){
		uint32_t ubuf;
		printk("Frame = %d\n", uframe);
		for(ubuf=0; ubuf<CONFIG_VIN_DEV2_ENCODE_BUF_NO; ubuf=ubuf+1)
		{
			printk("priv->pFramePlanarAddr[%d].u32PlaY = 0x%x\n", ubuf, priv->pFramePlanarAddr[ubuf].u32PlaY);
			printk("priv->pFramePlanarAddr[%d].u32PlaU = 0x%x\n", ubuf, priv->pFramePlanarAddr[ubuf].u32PlaU);
			printk("priv->pFramePlanarAddr[%d].u32PlaV = 0x%x\n", ubuf, priv->pFramePlanarAddr[ubuf].u32PlaV);
		}
	}
#endif

	if(priv->i32Zooming==FALSE)
		priv->pDevVin->SetShadowRegister();
	priv->pDevVin->SetOperationMode(TRUE);
	/* Clear Interrupt */
	u32IntStatus = priv->pDevVin->PollInt();
	if( (u32IntStatus & (VINTEN | VINT)) == (VINTEN | VINT))
		priv->pDevVin->ClearInt(eVIDEOIN_VINT);
	else if((u32IntStatus & (ADDRMEN|ADDRMINT)) == (ADDRMEN|ADDRMINT))
		priv->pDevVin->ClearInt(eVIDEOIN_ADDRMINT);
	else if ((u32IntStatus & (MEINTEN|MEINT)) == (MEINTEN|MEINT))
		priv->pDevVin->ClearInt(eVIDEOIN_MEINT);
	else if ((u32IntStatus & (MDINTEN|MDINT)) == (MDINTEN|MDINT))
		priv->pDevVin->ClearInt(eVIDEOIN_MDINT);

	wake_up_interruptible(&videoin_wq);    
	return IRQ_HANDLED;
}

static void videoin_release(struct video_device *vfd)
{
	DBG_PRINTF("%s\n",__FUNCTION__);
	kfree(vfd);
}

//#ifdef CONFIG_W55FA92_VIDEOIN_DEV1
//extern INT32 register_vin_port1_Sensor(NVT_SENSOR_T **sensor_intf);
//#endif 
#ifdef CONFIG_W55FA92_VIDEOIN_DEV2
extern INT32 register_vin_port2_Sensor(NVT_SENSOR_T **sensor_intf);
#endif 

static int __devinit videoin_init(void)
{
	int ret = 0;
	int i;
	
	unsigned int u32VinEncBufSize, u32VinPreBufSize;
//#ifdef CONFIG_MOTION_DETECTION_DEV2
	unsigned int  u32VinDiffBufSize;
//#endif
	videoin_priv_t *priv = (videoin_priv_t *)&videoin_priv;
	
	ENTER();
	
#ifdef CONFIG_SUPPORT_SHARE_MEMORY_DEV2
	w55fa92_vde_total_size = _vde_total_size;
	w55fa92_vde_physaddr_start = _vde_mem_start_addr;
	w55fa92_vde_virtaddr_start = _vde_vir_mem_start_addr;
#endif

	
	/* initialize locks */
	init_MUTEX(&videoin_priv.lock);
	
	/* read back the frame buffer physical address */
	priv->u32FbPhyAddr = inp32(REG_LCM_FSADDR);
	priv->i32FrameNumber = -1; 
	priv->i32ReadFrameNum = 0;
	priv->u32PortAddr = 0x800;
	priv->videoIn_preview_buf = (videoIn_buf_t*)videoIn_preview_buf;
	priv->videoIn_encode_buf = (videoIn_buf_t*)videoIn_encode_buf;
	priv->pFramePackerAddr = (unsigned int*)FramePackerAddr;
	priv->pFramePlanarAddr = (planar_buf_t*)FramePlanarAddr;	

	priv->i32AllocBufSize = get_vin_dev2_buffer_size();	//DEV 2
	priv->i32RemainingBufSize = priv->i32AllocBufSize;
	
	//priv->i32AllocBufSize = CONFIG_VIN_DEV2_BUFFER_SIZE;	//DEV 2
	//priv->i32RemainingBufSize = CONFIG_VIN_DEV2_BUFFER_SIZE;
	
#if 0
	priv->jdev.owner = THIS_MODULE;
	priv->jdev.type = VID_TYPE_CAPTURE | VID_TYPE_SCALES;
	priv->jdev.hardware = VID_HARDWARE_W55FA93;
#endif
	priv->jdev.release = videoin_release;
	priv->jdev.fops = &videoin_fops;
	priv->jdev.priv = &videoin_priv;	
	
	priv->videopic.brightness = 0;
	priv->videopic.hue = 0;
	priv->videopic.colour = 0;
	priv->videopic.contrast = 0;
	priv->videopic.whiteness= 0;
	priv->videopic.depth = 16;
	priv->videopic.palette = VIDEO_PALETTE_YUV422;
	priv->videowin.x = 0;
	priv->videowin.y = 0;

	priv->i32packet_width = CONFIG_VIN_DEV2_PREVIEW_WIDTH;
	priv->i32packet_height = CONFIG_VIN_DEV2_PREVIEW_HEIGHT;

	priv->i32MappingBuf = 0;	/* Default mapping planar buffer 0 */
	priv->i32Zooming = FALSE;

	priv->videowin.width = priv->i32packet_width;
	priv->videowin.height = priv->i32packet_height;
	priv->videowin.chromakey= -1;
	priv->videowin.flags = 0 ;
	priv->videowin.clips = NULL ;
	priv->videowin.clipcount = 0;
	priv->grab_sync = -1;
	priv->start_end_capture = 0;
	priv->videommap.frame = 0;	

	register_vin_port2_Sensor(&(priv->sensor_intf));

	priv->videocropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE; 
	priv->videocropcap.bounds.left = 0; 
	priv->videocropcap.bounds.top = 0;
	priv->videocropcap.bounds.width = 640; 
	priv->videocropcap.bounds.height = 480;

	priv->videocropcap.defrect.left = 0; 
	priv->videocropcap.defrect.top = 0;
	priv->videocropcap.defrect.width = 640; 
	priv->videocropcap.defrect.height = 480;

	priv->videocropcap.pixelaspect.numerator = 12;		/* Suppose current image size VGA */
	priv->videocropcap.pixelaspect.denominator = 16;	/* Zoomming step */


	priv->videommap.width = priv->sensor_intf->u16MaxImgWidth;
	priv->videommap.height = priv->sensor_intf->u16MaxImgHeight;
#ifdef CONFIG_PLANAR_YUV422_YUV420_DEV2
	priv->videommap.format = VIDEO_PALETTE_YUV422P;
#else
	priv->videommap.format = VIDEO_PALETTE_YUV420P;
#endif	
	strcpy((char *)&priv->videocap.name, "W55FA92 Camera dev2");
	priv->videocap.type = VID_TYPE_CAPTURE | VID_TYPE_SUBCAPTURE | VID_TYPE_SCALES;
	priv->videocap.channels = 1;
	priv->videocap.audios = 0;	
	priv->videocap.minwidth = 160;
	priv->videocap.minheight = 120;
	priv->videocap.maxwidth = priv->sensor_intf->u16MaxImgWidth;
	priv->videocap.maxheight = priv->sensor_intf->u16MaxImgHeight;
	priv->i32IRLedMode = 0;
	priv->i32IsIRLedOn = FALSE;
	
	bIsVideoIn2Enable = 0;
	#ifdef CONFIG_SUPPORT_SHARE_PACKET_BUFFER_DEV1
	u32VinPreBufSize = 0;
	#else
	u32VinPreBufSize =(priv->i32packet_width*priv->i32packet_height)*2;

	#endif

	#ifdef CONFIG_SUPPORT_SHARE_PLANAR_BUFFER_DEV1
		u32VinEncBufSize = 0;
	#else		
		#ifdef CONFIG_PLANAR_YUV422_YUV420_DEV2
		u32VinEncBufSize = priv->sensor_intf->u16MaxImgHeight * priv->sensor_intf->u16MaxImgWidth * 2;
		#endif
		#ifdef CONFIG_PLANAR_YUV420_DEV2
		u32VinEncBufSize = priv->sensor_intf->u16MaxImgHeight * priv->sensor_intf->u16MaxImgWidth * 3/2;
		#endif
	#endif
	
#ifdef CONFIG_MOTION_DETECTION_DEV2
	if((priv->i32packet_width/8)%4==0)
		u32VinDiffBufSize = priv->i32packet_width/8*priv->i32packet_height/8;
	else
		u32VinDiffBufSize = (priv->videowin.width/8+(4-((priv->i32packet_width/8)%4))) *priv->i32packet_height/8;	
	priv->videoIn_diff_buf = (videoIn_buf_t*)videoIn_diff_buf;
	priv->videoIn_md_Offset_info = (videoin_motion_t*)videoIn_md_Offset_info;
#else
	u32VinDiffBufSize = 0;	
	priv->videoIn_diff_buf = (videoIn_buf_t*)videoIn_diff_buf;
	priv->videoIn_md_Offset_info = (videoin_motion_t*)videoIn_md_Offset_info;	
#endif
#ifdef __BOOT_MEM__
	if( videoin_dmamalloc_bootmem((u32VinEncBufSize+PAGE_SIZE), (u32VinPreBufSize+PAGE_SIZE), (u32VinDiffBufSize+PAGE_SIZE) ) <= 0)
	{
		printk("Video buffer is not enough\n");
		return -EPIPE;
	}
#else	
	if( videoin_dmamalloc_planar_phy((u32VinEncBufSize+PAGE_SIZE)) <= 0)
		return -EPIPE;
	if( videoin_dmamalloc_packet_phy((u32VinPreBufSize+PAGE_SIZE)) <= 0)
		return -EPIPE;
#endif
	DBG_PRINTF("Encode buf no.= %d\n", CONFIG_VIN_DEV2_ENCODE_BUF_NO);		
	DBG_PRINTF("Preview buf no.= %d\n", CONFIG_VIN_DEV2_PREVIEW_BUF_NO);
	for(i=0; i<CONFIG_VIN_DEV2_ENCODE_BUF_NO; i++)
	{	
		priv->videombuf.offsets[i] = 0 + (videoIn_encode_buf[i].u32VirtAddr - videoIn_encode_buf[0].u32VirtAddr);	
		MPRINTF("mbuf[%d].offset = 0x%x\n", i, priv->videombuf.offsets[i]);
	}
	for(i=CONFIG_VIN_DEV2_ENCODE_BUF_NO; i<(CONFIG_VIN_DEV2_ENCODE_BUF_NO+CONFIG_VIN_DEV2_PREVIEW_BUF_NO); i++)
	{
		priv->videombuf.offsets[i] = 0 + (videoIn_preview_buf[(i-CONFIG_VIN_DEV2_ENCODE_BUF_NO)].u32VirtAddr - videoIn_encode_buf[0].u32VirtAddr);	
		MPRINTF("mbuf[%d].offset = 0x%x\n", i, priv->videombuf.offsets[i]);
	}	

#ifdef CONFIG_MOTION_DETECTION_DEV2
	for(i=CONFIG_VIN_DEV2_ENCODE_BUF_NO+CONFIG_VIN_DEV2_PREVIEW_BUF_NO; 
		i<CONFIG_VIN_DEV2_ENCODE_BUF_NO+CONFIG_VIN_DEV2_PREVIEW_BUF_NO+VIDEOIN_DIFF_BUFFER_NUM; i=i+1){
		priv->videombuf.offsets[i] = 0 + (priv->videoIn_diff_buf[i-(CONFIG_VIN_DEV2_ENCODE_BUF_NO+CONFIG_VIN_DEV2_PREVIEW_BUF_NO)].u32VirtAddr -\
										videoIn_encode_buf[0].u32VirtAddr);
		MPRINTF("mbuf[%d].offset = 0x%x\n", i, priv->videombuf.offsets[i]);
		priv->videoIn_md_Offset_info[i-(CONFIG_VIN_DEV2_ENCODE_BUF_NO+CONFIG_VIN_DEV2_PREVIEW_BUF_NO)].u32MapOffsetAddr = priv->videombuf.offsets[i];
		priv->videoIn_md_Offset_info[i-(CONFIG_VIN_DEV2_ENCODE_BUF_NO+CONFIG_VIN_DEV2_PREVIEW_BUF_NO)].u32Size = u32VinDiffBufSize;
	}
#else
	priv->videoIn_md_Offset_info[i-(CONFIG_VIN_DEV2_ENCODE_BUF_NO+CONFIG_VIN_DEV2_PREVIEW_BUF_NO)].u32MapOffsetAddr = 0;
	priv->videoIn_md_Offset_info[i-(CONFIG_VIN_DEV2_ENCODE_BUF_NO+CONFIG_VIN_DEV2_PREVIEW_BUF_NO)].u32Size = 0;	
#endif
	priv->videombuf.frames = CONFIG_VIN_DEV2_ENCODE_BUF_NO; 
	/* The frames only report encode buffer no. Other buffers need through VIDIOC_G_PACKET_INFO and VIDIOC_G_PLANAR_INFO */
#ifdef CONFIG_MOTION_DETECTION_DEV2
	priv->videombuf.size = (u32VinEncBufSize + PAGE_SIZE)*CONFIG_VIN_DEV2_ENCODE_BUF_NO +\
							(u32VinPreBufSize+ PAGE_SIZE)*CONFIG_VIN_DEV2_PREVIEW_BUF_NO +\
							(u32VinDiffBufSize+ PAGE_SIZE)*VIDEOIN_DIFF_BUFFER_NUM;
#else
	priv->videombuf.size = (u32VinEncBufSize + PAGE_SIZE)*CONFIG_VIN_DEV2_ENCODE_BUF_NO + (u32VinPreBufSize+ PAGE_SIZE)*CONFIG_VIN_DEV2_PREVIEW_BUF_NO;
#endif
	MPRINTF("DIFF Temp Y Addr = 0x%x\n", priv->videoIn_md_Offset_info[0].u32MapOffsetAddr); 
	MPRINTF("DIFF Temp Size = 0x%x\n", priv->videoIn_md_Offset_info[0].u32Size);
	MPRINTF("DIFF Outout Addr = 0x%x\n", priv->videoIn_md_Offset_info[1].u32MapOffsetAddr); 
	MPRINTF("DIFF Output Size = 0x%x\n", priv->videoIn_md_Offset_info[1].u32Size);

	/* register video device */
	if (video_register_device(&priv->jdev, VFL_TYPE_GRABBER, videoin_dev2) == -1) {
		printk("%s: video_register_device failed\n", __FUNCTION__);
	#if 1 /* the buffer can not be free if bootmem */
	#else	
		videoin_free_dmamalloc_planar_phy((u32VinEncBufSize + PAGE_SIZE));
		videoin_free_dmamalloc_packet_phy((u32VinPreBufSize + PAGE_SIZE));
	#endif
		return -EPIPE;
	}

#ifndef CONFIG_W55FA92_VIDEOIN_DEV1  	
	if (!request_mem_region((unsigned long)W55FA92_VA_VIDEOIN, W55FA92_SZ_VIDEOIN, "w55fa92-videoin-dev2")){
		printk("%s: request_mem_region failed\n", __FUNCTION__);
		video_unregister_device(&videoin_priv.jdev);
	#if 1 /* the buffer can not be free if bootmem */
	#else	
		videoin_free_dmamalloc_planar_phy((u32VinEncBufSize + PAGE_SIZE));
		videoin_free_dmamalloc_packet_phy((u32VinPreBufSize + PAGE_SIZE));
	#endif
		return -EBUSY;
	}
#endif 

	#ifdef CONFIG_SUPPORT_SHARE_MEMORY_DEV2
	w55fa92_vde_remaining_size = w55fa92_vde_total_size; /* After allocate packet/planar, the w55fa92_vde_total_size = remaining size */
	#endif

	/* register irq handler */
	ret = request_irq(IRQ_CAP2, irq_handler,  IRQF_DISABLED | IRQF_SHARED, "w55fa92-videoin-dev2", priv);
	if (ret) {
		printk("cannot get irq %d - err %d\n", IRQ_CAP2, ret);
		ret = -EBUSY;
		goto release_mem;
	}

	/* Sensor power down in booting*/
	if(priv->sensor_intf->sensor_suspend)
		priv->sensor_intf->sensor_suspend(TRUE);
	if(priv->sensor_intf->sensor_poweron)
		priv->sensor_intf->sensor_poweron(FALSE);

	printk("Video capture device 2 initialize successful\n");
	return ret;

release_mem:
	video_unregister_device(&priv->jdev);
	release_mem_region((unsigned long)W55FA92_VA_VIDEOIN, W55FA92_SZ_VIDEOIN);
	free_irq(IRQ_CAP,priv);
#if 1 /* the buffer can not be free if bootmem */
#else	
	videoin_free_dmamalloc_planar_phy((u32VinEncBufSize + PAGE_SIZE));
	videoin_free_dmamalloc_packet_phy((u32VinPreBufSize + PAGE_SIZE));
#endif
	printk("Video capture device 1 itialize fail\n");
	return ret;
}

static void __exit videoin_cleanup(void)
{
	unsigned int u32VinEncBufSize, u32VinPreBufSize;
	videoin_priv_t *priv = (videoin_priv_t *)&videoin_priv;

	ENTER(); 	
	u32VinEncBufSize = priv->sensor_intf->u16MaxImgHeight * priv->sensor_intf->u16MaxImgWidth * 2;
	u32VinPreBufSize =(priv->i32packet_width*priv->i32packet_height)*2;
	video_unregister_device(&priv->jdev);
	release_mem_region((unsigned long)W55FA92_VA_VIDEOIN, W55FA92_SZ_VIDEOIN);
	free_irq(IRQ_CAP,priv);
#if 1 /* the buffer can not be free if bootmem */
#else	
	videoin_free_dmamalloc_planar_phy((u32VinEncBufSize + PAGE_SIZE));
	videoin_free_dmamalloc_packet_phy((u32VinPreBufSize + PAGE_SIZE));
#endif
}

module_init(videoin_init);
module_exit(videoin_cleanup);

//EXPORT_SYMBOL(videoin_register_outputdev);

MODULE_DESCRIPTION("video in driver for the W55FA92");
MODULE_LICENSE("GPL");

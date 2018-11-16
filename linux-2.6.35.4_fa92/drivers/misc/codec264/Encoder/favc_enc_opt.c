/********************************************/
/* Faraday avc encoder driver operations    */
/********************************************/
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
#include <linux/clk.h>

#include <mach/w55fa92_reg.h>
#include <mach/favc_avcodec.h>

#else
#include <stdio.h>
#include <string.h>
#include "w55fa92_reg.h"
#include "wblib.h"
#include "favc_avcodec.h"
#endif

#include "h264dec.h"
#include "AVCdec.h"

#include "port.h"
#include "register.h"
#include "encoder.h"
#include "common.h"
#include "slice.h"
#include "sequence.h"
#include "bs.h"
#include "user_define.h"

//#include "h264_enc_api.h"
#include "favc_version.h"


#define ALLO_NONINDEPEND    // used for mem alloc testing
#ifdef CONFIG_ENABLE_DECODER_ONLY
#define MAX_ENC_NUM     (1)
#else
#define MAX_ENC_NUM     CONFIG_W55FA92_AVC_ENCODER_NUM
#endif
#define ENC_IDX_MASK_n(n)  (0x1 << n)
#define ENC_IDX_FULL    0xFFFFFFFF


// following global variables need modified for duplex
extern unsigned int    out_phy_buffer, out_virt_buffer;//,enc_recon_buf, enc_refer_buf;
extern struct semaphore favc_enc_mutex;
extern unsigned long _ENCODER_BUF_START,_DECODER_BUF_START;
extern unsigned int dec_bs_phy_buffer;

extern DECODER_INFO DECODER_INST[4];
extern ENCODER_INFO ENCODER_INST[2];

static int              enc_idx=0;

struct enc_private      enc_data[MAX_ENC_NUM];

struct clk *clk_264enc = NULL;

extern int favc_check_continued(unsigned int addr, int size);
//extern unsigned int favc_user_va_to_pa(unsigned int addr);
extern int h264_encoder_spspps(void *handle,FAVC_ENC_PARAM *mEncParam);
extern int h264_encoder_nvop_nal(void *handle,FAVC_ENC_PARAM *mEncParam);

extern void* nv_malloc(int size, int alignment);
extern int nv_free(void* ptr);

#ifdef EVALUATION_PERFORMANCE
extern void get_drv_start(void);
#endif

#ifdef LINUX_ENV
int favc_encoder_open(struct inode *inode, struct file *filp)
#else
int favc_encoder_open(void)
#endif
{
    int dev;
    int i;
    

    int used_index=-1;
    
    used_index = (used_index >> MAX_ENC_NUM) << MAX_ENC_NUM;
    used_index = used_index ^ (-1); 
    if ((enc_idx & used_index) == used_index)
    {
        printk("Encoder Device Service Full,0x%x!\n",enc_idx);
        return -EFAULT;        
    }
	
	down(&favc_enc_mutex);
	
    dev=0;	
    while( dev < MAX_ENC_NUM) {
    	if( enc_idx&ENC_IDX_MASK_n(dev) ) {
    		dev++;
    	} else {
    		enc_idx |= ENC_IDX_MASK_n(dev);	
    		break;
    	}
    }	
	
    //printk("idx=%d enc_idx=0x%x\n",idx,enc_idx);
    filp->private_data=kmalloc(sizeof(unsigned int),GFP_KERNEL);
    if(filp->private_data==0)    {
        printk("Can't allocate memory!\n");
        up(&favc_enc_mutex);
        return -EFAULT;
    }
    *(unsigned int *)filp->private_data=dev;
    memset(&enc_data[dev],0,sizeof(struct enc_private));
    
    //Enable H.264 Encoder clock for first instance
    if (dev ==0)
    {
		unsigned long flags;        
        // enable clock
        //outp32(REG_AHBCLK2, inp32(REG_AHBCLK2) | VENC_CKE);
	    clk_264enc = clk_get(NULL, "venc");
	    clk_enable(clk_264enc);        
        printk("Enable H.264 encoder clock\n");
		
        // reset IP
		local_irq_save(flags);         
        outp32(REG_AHBIPRST, VEN_RST);
        outp32(REG_AHBIPRST, 0); 
		local_irq_restore(flags);                
    }
    
    
    enc_data[dev].enc_handle=0;		
    for (i=0; i<SUPPORT_MAX_MSLIC_NUM; i++) {
        
        enc_data[dev].slicebuf[i].recon_phy_buffer = 0;
        enc_data[dev].slicebuf[i].refer_phy_buffer = 0;
        enc_data[dev].slicebuf[i].sysinfo_phy_buffer = 0;
        enc_data[dev].slicebuf[i].dma_phy_buffer = 0;        
        
        enc_data[dev].slicebuf[i].mslice_mb=0;
        enc_data[dev].slicebuf[i].mslice_height = enc_data[dev].slicebuf[i].mslice_width=0;
    }

    up(&favc_enc_mutex);
    
    return 0; /* success */
}
int h264e_init(FAVC_ENC_PARAM * param, int dev)
{
    unsigned int tmp;
    unsigned int mslice_height;
    int i;
    unsigned int size=0;
    unsigned int roi_x, roi_y, roi_width, roi_height;	
    unsigned int recon_size, refer_size, sysinfo_size, dma_size, slice_offset;	
    unsigned int sysinfo_ba=0,pre_sysinfo_size=0, total_sysinfo=0;;

	// Clear setting for next bitstream encode
	if (dev < MAX_ENC_NUM)
	{
		if (enc_data[dev].enc_handle != 0x0)
		memset(enc_data[dev].enc_handle, 0, sizeof(h264_encoder));
	}	
		
    //if(param->u32API_version != H264VER)  {
    //    printk("Fail API Version v%d.%d (Current Driver v%d.%d)\n",
    //    param->u32API_version>>16,param->u32API_version&0xffff, H264VER_MAJOR, H264VER_MINOR);
    //    return -EFAULT;
    //}
    if (param->u32FrameWidth & 0xF){
        tmp = param->u32FrameWidth;
        param->u32FrameWidth = ((tmp + 15) >> 4) << 4;
        printk("REPLACE width %d with %d\n", tmp, param->u32FrameWidth);
    }
    if (param->u32FrameHeight & 0xF){
        tmp = param->u32FrameHeight;
        param->u32FrameHeight = ((tmp + 15) >> 4) << 4;
        printk("REPLACE height %d with %d\n", tmp, param->u32FrameHeight);
    }
    if (param->u32FrameWidth <= (8 * 16)){
        // the width of H.264 should larger than 8 macro width
        printk("The width is too SMALL ( width<=8x16) for H.264 encoder\n");
        return -EFAULT;
    }

    #define IMG_FMT_SUPPORT 0	// only H.264 2D format is supported.

    if (param->img_fmt > IMG_FMT_SUPPORT) {
        printk("img_fmt error (cur: %d)\n", param->img_fmt);
        return -EFAULT;
    }
    /*
    else if (param->img_fmt == 1) { // 2D mp4 mode
        int distance_uv = -15 + (param->u32FrameWidth * param->u32FrameHeight/4)/4;
        if ((distance_uv >= 0x8000) || (distance_uv < -0x8000)) {
            printk ("[Error] Not support such high resolution (%dx%d) at MP4-2D mode\n", param->u32FrameWidth, param->u32FrameHeight);
            return -EFAULT;
        }
    }
    */


    tmp = param->u32FrameWidth * param->u32FrameHeight >> 8;

    if ((param->control & 1) || (tmp < LIMIT_MAX_MB)) {
        enc_data[dev].mslice_num = 1;
        enc_data[dev].mslice_enable = 0;
        //printk("SingleSlice encoding, One frame %d slices max mb %d\n", enc_data[dev].mslice_num, LIMIT_MAX_MB);
    }
    else{
        enc_data[dev].mslice_num = (tmp + LIMIT_MAX_MB - 1) / LIMIT_MAX_MB;
        enc_data[dev].mslice_enable = 1;
        //printk("MultiSlice encoding, One frame %d slices total %d max mb %d\n", enc_data[dev].mslice_num, mb_num, LIMIT_MAX_MB);
    }
    enc_data[dev].frame_width = param->u32FrameWidth;
    enc_data[dev].frame_height = param->u32FrameHeight;

    if (param->bROIEnable){
        if ((param->u32ROIWidth == 0) || (param->u32ROIHeight == 0)){
            printk("Error: ROIwidth, ROIHeight (%d, %d) should not be 0\n", param->u32ROIWidth, param->u32ROIHeight);
            return -EFAULT;
        }
        if (param->u32ROIWidth & 0xF){
            tmp = param->u32ROIWidth;
            param->u32ROIWidth = ((tmp + 0xF) >> 4) << 4;
            printk("REPLACE ROI width %d with %d\n", tmp, param->u32ROIWidth);
        }
        if (param->u32ROIHeight & 0xF){
            tmp = param->u32ROIHeight;
            param->u32ROIHeight = ((tmp + 0xF) >> 4) << 4;
            printk("REPLACE ROI height %d with %d\n", tmp, param->u32ROIHeight);
        }
        if (param->u32ROIX & 0xF){
            tmp = param->u32ROIX;
            param->u32ROIX = ((tmp + 0xF) >> 4) << 4;
            printk("REPLACE ROI x %d with %d\n", tmp, param->u32ROIX);
        }
        if (param->u32ROIY & 0xF){
            tmp = param->u32ROIY;
            param->u32ROIY = ((tmp + 0xF) >> 4) << 4;
            printk("REPLACE ROI y %d with %d\n", tmp, param->u32ROIY);
        }

        if (((param->u32ROIX + param->u32ROIWidth) > param->u32FrameWidth) || 
           ((param->u32ROIY + param->u32ROIHeight) > param->u32FrameHeight)){
            printk("Error: Setting ROI, X Y = (%d, %d) , W H = (%d, %d), FW FH = (%d, %d)\n", 
           param->u32ROIX, param->u32ROIY, 
           param->u32ROIWidth, param->u32ROIHeight, 
           param->u32FrameWidth, param->u32FrameHeight);
            return -EFAULT;
        }
        enc_data[dev].roi_width = param->u32ROIWidth;
        enc_data[dev].roi_height = param->u32ROIHeight;
        param->pic_height = enc_data[dev].roi_height;
        param->pic_width = enc_data[dev].roi_width;
        roi_x = param->u32ROIX;
        roi_y = param->u32ROIY;
        roi_width = param->u32ROIWidth;
        roi_height = param->u32ROIHeight;
    } //if (param->bROIEnable)

    else {
        param->u32ROIX = 0;
        param->pic_height = enc_data[dev].frame_height;
        param->pic_width = enc_data[dev].frame_width;
        roi_x = 0;
        roi_y = 0;
        roi_width = 0;
        roi_height = 0;
    }
    enc_data[dev].roi_enable = param->bROIEnable;

    //========================================================================================
	tmp = enc_data[dev].frame_height;
	mslice_height = (((tmp / enc_data[dev].mslice_num) + (PIXEL_Y - 1)) / PIXEL_Y) * PIXEL_Y;

	// MultiSlice
	for (i = 0; i < enc_data[dev].mslice_num; i++){
		sbuf * pslc = &enc_data[dev].slicebuf[i];
		pslc->mslice_width = enc_data[dev].frame_width;
		if (tmp >= mslice_height) pslc->mslice_height = mslice_height;
		else pslc->mslice_height = tmp;
		tmp -= pslc->mslice_height;
		pslc->mslice_mb = pslc->mslice_width * pslc->mslice_height / SIZE_Y;
		if (i >= (enc_data[dev].mslice_num - 1))
			pslc->mslice_last = 1;
		else
			pslc->mslice_last = 0;

		// ROI
		if (param->bROIEnable){
			if (roi_height > 0){
				if (roi_y <= enc_data[dev].slicebuf[i].mslice_height){  // KC: Set msslice_ROIHeight for each slice
					pslc->mslice_ROI = 1;
					pslc->mslice_ROIX = roi_x;
					pslc->mslice_ROIY = roi_y;
					pslc->mslice_ROIWidth = enc_data[dev].roi_width;
					if (roi_height > pslc->mslice_height)
						pslc->mslice_ROIHeight = pslc->mslice_height - pslc->mslice_ROIY;
					else
						pslc->mslice_ROIHeight = roi_height;
					roi_height -= enc_data[dev].slicebuf[i].mslice_ROIHeight;
					roi_y = 0;
					pslc->mslice_ROILast = (roi_height == 0)? 1: 0;
				}
				else{                                                   
					pslc->mslice_ROI = 1;
					pslc->mslice_ROIX = 0;
					pslc->mslice_ROIY = 0;
					pslc->mslice_ROIWidth = 0;
					pslc->mslice_ROIHeight = 0;
					pslc->mslice_ROILast = 0;
					roi_y -= pslc->mslice_height;
				}
			}
			else{
				pslc->mslice_ROI = 1;
				pslc->mslice_ROIX = 0;
				pslc->mslice_ROIY = 0;
				pslc->mslice_ROIWidth = 0;
				pslc->mslice_ROIHeight = 0;
				pslc->mslice_ROILast = 0;
			}
			pslc->mslice_ROIMB = pslc->mslice_ROIWidth * pslc->mslice_ROIHeight / SIZE_Y;
#if 1
            // Important : Reserved max resolution to avoid DMA buffer destoried by multi-instance
//20140617	recon_size = refer_size = ENC_RECON_BUF_SIZE;	
            recon_size = refer_size = ENCODER_INST[dev].u32BSsize;	    // BS size is same as recon buf size			
#else			
			recon_size = refer_size = DIV_16(pslc->mslice_ROIWidth* pslc->mslice_ROIHeight* 3 / 2);
#endif			
			sysinfo_size = DIV_16(SYSBUF_SIZE(pslc->mslice_ROIWidth, pslc->mslice_ROIHeight));
		}
		else{
			pslc->mslice_ROI = 0;
			pslc->mslice_ROIX = 0;
			pslc->mslice_ROIY = 0;
			pslc->mslice_ROIWidth = 0;
			pslc->mslice_ROIHeight = 0;
			pslc->mslice_ROILast = 0;
			pslc->mslice_ROIMB = 0;
#if 1	
            // Important : Reserved max resolution to avoid DMA buffer destoried by multi-instance
//20140617			recon_size = refer_size = ENC_RECON_BUF_SIZE;	
            recon_size = refer_size = ENCODER_INST[dev].u32BSsize;		    // BS size is same as recon buf size		
#else
			recon_size = refer_size = DIV_16(pslc->mslice_width * pslc->mslice_height * 3 / 2);
#endif			
			sysinfo_size = DIV_16(SYSBUF_SIZE(pslc->mslice_width, pslc->mslice_height));
		}

		dma_size = DIV_16(DMA_BUF_SIZE + DMA_BUF_SIZE);

        if (i > 0)
            slice_offset = (enc_data[dev].slicebuf[i-1].mslice_width * enc_data[dev].slicebuf[i-1].mslice_height * 3 /2);
        else   
            slice_offset = 0;
		/* allocate recontructed */
        // It needs to adjust buffer address for multi-instance
        // Important : Allocate physical reconstructed buffer here
//0617  if ((void *)(pslc->refer_phy_buffer = enc_refer_buf + dev * _avc_enc_1instance_size + slice_offset) == NULL){	 + dev * _avc_enc_1instance_size + slice_offset) == NULL){	
		if ((void *)(pslc->recon_phy_buffer = _ENCODER_BUF_START + ENCODER_INST[dev].u32Offset + ENCODER_INST[dev].u32BSsize + slice_offset) == NULL){			    		    	
		    
			printk("Memory recon_virt_buffer allocation error!\n");
			return -EFAULT;
		}
        //printk("%s : recon_phy_buffer = 0x%x, size = 0x%x \n", __FUNCTION__,pslc->recon_phy_buffer, recon_size);		

		size = refer_size + sysinfo_size + dma_size;
		/* allocate refer */
        // Important : Allocate physical Reference buffer here
//0617		if ((void *)(pslc->refer_phy_buffer = enc_refer_buf + dev * _avc_enc_1instance_size + slice_offset) == NULL){		
            if ((void *)(pslc->refer_phy_buffer = _ENCODER_BUF_START + ENCODER_INST[dev].u32Offset + ENCODER_INST[dev].u32BSsize*2+ slice_offset) == NULL){	    		    
			printk("Memory refer_virt_buffer allocation error!\n");
			return -EFAULT;
		}
        //printk("%s : refer_phy_buffer = 0x%x, size = 0x%x \n", __FUNCTION__, pslc->refer_phy_buffer, refer_size);
        
		/* allocate sysinfo buffer*/
		// Important : Allocate sysinfo buffer here  
		if (i==0)
		{
		    pslc->sysinfo_phy_buffer = pslc->refer_phy_buffer + refer_size;
		    sysinfo_ba = pslc->sysinfo_phy_buffer;
		    pre_sysinfo_size = sysinfo_size;
        }		    
		else
		{
		    pslc->sysinfo_phy_buffer = sysinfo_ba+pre_sysinfo_size;
		    pre_sysinfo_size = sysinfo_size;
		    //sysinfo_ba = pslc->sysinfo_phy_buffer;
        }		    
		    
        //printk("%s : sysinfo_phy_buffer = 0x%x, size = 0x%x \n", __FUNCTION__, pslc->sysinfo_phy_buffer, sysinfo_size);	

		/* allocate dma buffer */
        // Important : Allocate physical DMA buffer here        
        total_sysinfo = DIV_16(SYSBUF_SIZE(ENCODER_INST[dev].u32MaxWidth, ENCODER_INST[dev].u32MaxHeight));        
        //printk("Max width = %d, height = %d, total inf = 0x%x\n",ENCODER_INST[dev].u32MaxWidth,ENCODER_INST[dev].u32MaxHeight, total_sysinfo);        
        if (i==0)
        {
             pslc->dma_phy_buffer = sysinfo_ba + total_sysinfo;
        }
        else
        {
             pslc->dma_phy_buffer = sysinfo_ba + total_sysinfo + dma_size;            
        }
        
        //printk("%s : dma_phy_buffer = 0x%x, size = 0x%x \n", __FUNCTION__, pslc->dma_phy_buffer, dma_size);	
        
		// init for first slice encoding
		param->multi_slice = enc_data[dev].mslice_enable;
		param->u32FrameWidth = pslc->mslice_width;
		param->u32FrameHeight = pslc->mslice_height;
		param->pu8ReConstructFrame = (unsigned char*) pslc->recon_phy_buffer;
		param->pu8ReferenceFrame = (unsigned char*) pslc->refer_phy_buffer;
		param->pu8SysInfoBuffer = (unsigned char*) pslc->sysinfo_phy_buffer;
		
		// set DMA phycial Address & Virtual Address
		param->pu8DMABuffer_phy = (unsigned char*) pslc->dma_phy_buffer;
		param->pu8DMABuffer_virt = (unsigned char *)phys_to_virt_92((unsigned int) pslc->dma_phy_buffer);
        //printk("%s : dma_virt_buffer = 0x%x\n", __FUNCTION__, (unsigned int)param->pu8DMABuffer_virt);			

		param->bROIEnable = pslc->mslice_ROI;
		param->u32ROIX = pslc->mslice_ROIX;
		param->u32ROIY = pslc->mslice_ROIY;
		param->u32ROIWidth = pslc->mslice_ROIWidth;
		param->u32ROIHeight = pslc->mslice_ROIHeight;
		// Important : Allocate physical reconstructed buffer here
		//param->pu8BitstreamAddr = (unsigned char*)out_phy_buffer;
//0617		param->pu8BitstreamAddr = (unsigned char*) (_ENCODER_BUF_START+ dev * _avc_enc_1instance_size);		
		param->pu8BitstreamAddr = (unsigned char*) (_ENCODER_BUF_START+ ENCODER_INST[dev].u32Offset);			
		//printk("%s : init BS Buf addr = 0x%x\n",__FUNCTION__, (unsigned int)param->pu8BitstreamAddr);
		param->threshold_disable = 0;
		param->chroma_threshold = 4;
		param->luma_threshold = 4;
		param->beta_offset = 0;
		param->alpha_offset = 0;
		param->chroma_qp_offset = 0;
		param->disable_ilf = 0;
		param->no_frames = 300; // not used if IGNORE_FRAME_NO is enabled
//		param->pfnDmaMalloc = hconsistent_alloc;
//		param->pfnDmaFree = hconsistent_free;
//		param->pfnMalloc = hkmalloc;
//		param->pfnFree = hkfree;

		if (enc_data[dev].enc_handle == NULL){
			// first time to allocate h264_encoder memory
			enc_data[dev].enc_handle = h264_encoder_init(param);
			if (enc_data[dev].enc_handle == NULL){
				printk("Error to create encoder structure!\n");
				return - EFAULT;
			}
		}
		// init every default DMA command
		//printk("Calling h264_encoder_reinit\n");
		h264_encoder_reinit(enc_data[dev].enc_handle, param, param->img_fmt);
	}
	return 0;
}

#ifdef LINUX_ENV
int favc_encoder_ioctl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg)
#else
int favc_encoder_ioctl(void* handle, unsigned int cmd, void* arg)
#endif
{
    FAVC_ENC_PARAM     tEncParam;
    FAVC_VUI_PARAM     tVUIParam;
    FAVC_CROP_PARAM    tCROPParam;
    int                 i, ret=0;
#ifdef LINUX_ENV    
    int                 dev=*(int *)filp->private_data;
	//int					dev=0;      
#else
	int					dev=0;    
#endif    
    int mslice_len;
    unsigned int sysbuf_len=0;
    unsigned int YFrameBase, UFrameBase, VFrameBase;
	avc_workbuf_t workbuf;    
   
    //printk("%s : dev = %d\n",__FUNCTION__,dev);
 
   
  
    if (((1 << dev)&enc_idx) == 0) {
        printk("No Support index %d for 0x%x\n",dev,enc_idx);
        return -EFAULT;
    }
	
    down(&favc_enc_mutex);


    switch(cmd) {
    case FAVC_IOCTL_ENCODE_INIT:
	//printk("%s : cmd= FAVC_IOCTL_ENCODE_INIT\n",__FUNCTION__);     

        if (copy_from_user((void *)&tEncParam, (void *)arg, sizeof(tEncParam)))
        {
            printk("Error copy_from_user() of FAVC_IOCTL_ENCODE_INIT\n");
            ret = -EFAULT;
            goto encoder_ioctl_exit;
        }
        
        if (tEncParam.u32FrameWidth * tEncParam.u32FrameHeight > ENCODER_INST[dev].u32MaxWidth * ENCODER_INST[dev].u32MaxHeight)
        {
           printk("Warming : Enc instance-%d specify encode img size = %d * %d, but menuconfig is %d * %d\n", 
                    dev, tEncParam.u32FrameWidth, tEncParam.u32FrameHeight, ENCODER_INST[dev].u32MaxWidth, ENCODER_INST[dev].u32MaxHeight);
		   ret=-EFAULT;                
        }        
      
	//printk("%s : h264e_init\n",__FUNCTION__);  
        tEncParam.img_fmt = 0; // 2D h264 format
        if (h264e_init(&tEncParam, dev) < 0) {
            printk("FAVC_IOCTL_ENCODE_INIT error\n");
            ret = -EFAULT;
        }
        break;

    case FAVC_IOCTL_ENCODE_INIT_MP4:
	     printk("MP4 2D format is not supported now\n");
	     ret = -EFAULT;
       
        break;

    case FAVC_IOCTL_ENCODE_VUI:
 
        if (copy_from_user((void *)&tVUIParam, (void *)arg, sizeof(tVUIParam)))
        {
            printk("Error copy_from_user() of FAVC_IOCTL_ENCODE_INIT_VUI\n");
            ret = -EFAULT;
            goto encoder_ioctl_exit;
        }
       
        if ( enc_data[dev].enc_handle == NULL ) {
            printk("Error: enc_handle %d is NULL\n", dev);
            ret=-EFAULT;
            goto encoder_ioctl_exit;
        }	
        h264_encoder_init_vui(enc_data[dev].enc_handle, &tVUIParam);

        if (copy_to_user((void *)arg,(void *)&tVUIParam,sizeof(tVUIParam)))
        {
            printk("Error copy_to_user() of FAVC_IOCTL_ENCODE_INIT_VUI\n");
            ret = -EFAULT;
        }
       
        break;

    case FAVC_IOCTL_ENCODE_CROP:
   
        if (copy_from_user((void *)&tCROPParam, (void *)arg, sizeof(tCROPParam)))
        {
            printk("Error copy_from_user() of FAVC_IOCTL_ENCODE_CROP\n");
            ret = -EFAULT;
            goto encoder_ioctl_exit;
        }
       
        if ( enc_data[dev].enc_handle == NULL ) {	
            printk("Error: enc_handle %d is NULL\n", dev);
            ret=-EFAULT;
            goto encoder_ioctl_exit;
        }
        h264_encoder_init_crop(enc_data[dev].enc_handle, &tCROPParam);

        if (copy_to_user((void *)arg,(void *)&tCROPParam,sizeof(tCROPParam)))
        {
            printk("Error copy_to_user() of FAVC_IOCTL_ENCODE_CROP\n");
            ret = -EFAULT;
        }
       
        break;


    case FAVC_IOCTL_ENCODE_FRAME:
	//printk("%s : cmd= FAVC_IOCTL_ENCODE_FRAME\n",__FUNCTION__);    
#ifdef EVALUATION_PERFORMANCE
        get_drv_start();
#endif
        if (copy_from_user((void *)&tEncParam, (void *)arg, sizeof(tEncParam)))
        {
            printk("Error copy_from_user() of FAVC_IOCTL_ENCODE_FRAME\n");
            ret = -EFAULT;
            goto encoder_ioctl_exit;
        }
       
        if ( enc_data[dev].enc_handle == NULL ) {	
            printk("Error: enc_handle %d is NULL\n", dev);
            ret=-EFAULT;
            goto encoder_ioctl_exit;
        }	
            
        //check Qp value
        if((int)tEncParam.u32Quant > MaxQp || (int)tEncParam.u32Quant < MinQp){
            printk("[Error]: Qp Error Qp:%d MaxQp:%d  MinQp:%d  \n",tEncParam.u32Quant,tEncParam.u32MaxQuant,tEncParam.u32MinQuant);
            ret=-EFAULT;
            goto encoder_ioctl_exit;    	            
        }
            
        enc_data[dev].mslice_first_mb=0;       // Don't forget reset mslice_first_mb to 0, it will influence pEnc->frame_num 		
        enc_data[dev].mslice_stride_yuv=0;   // Don't forget reset mslice_stride_yuv to 0, it will influence Y,U,V or UV base

        YFrameBase = (unsigned int)tEncParam.pu8YFrameBaseAddr;
        UFrameBase = (unsigned int)tEncParam.pu8UFrameBaseAddr;
        VFrameBase = (unsigned int)tEncParam.pu8VFrameBaseAddr;        

        if (h264_fmt(enc_data[dev].enc_handle) == 1) { // mp4-2D
            int distance_uv = -15 + (((int)VFrameBase - (int)UFrameBase))/4;
            if ((distance_uv >= 0x8000) || (distance_uv < -0x8000)) {
                printk ("[Error] Distance between U & V base address is too large (0x%08x, 0x%08x) at MP4-2D mode\n",
                  (int)tEncParam.pu8UFrameBaseAddr, (int)tEncParam.pu8VFrameBaseAddr);
                ret = -EFAULT;
                goto encoder_ioctl_exit;
            }
        }

        tEncParam.multi_slice = enc_data[dev].mslice_enable;
        //printk("\nmslice_num: %d, mslice_enable: %d\n", enc_data[dev].mslice_num, enc_data[dev].mslice_enable);

        for ( i = 0; i<enc_data[dev].mslice_num; i++ ) {  
           // ROI parameter update
           // Multi-slice is not support dynamic ROI
           if ( enc_data[dev].mslice_num > 1 ) {
               tEncParam.u32ROIX = enc_data[dev].slicebuf[i].mslice_ROIX;
               tEncParam.u32ROIY = enc_data[dev].slicebuf[i].mslice_ROIY;
           } else {
               if ( tEncParam.bROIEnable == 0 ) {
                   tEncParam.u32ROIX = enc_data[dev].slicebuf[i].mslice_ROIX;
                   tEncParam.u32ROIY = enc_data[dev].slicebuf[i].mslice_ROIY;
               }
           }
           tEncParam.u32ROIWidth = enc_data[dev].slicebuf[i].mslice_ROIWidth;
           tEncParam.u32ROIHeight = enc_data[dev].slicebuf[i].mslice_ROIHeight;
           tEncParam.bROIEnable = enc_data[dev].slicebuf[i].mslice_ROI;
           // MultiSlice reference memory buffer update		  				  
           tEncParam.u32FrameWidth= enc_data[dev].slicebuf[i].mslice_width;
           tEncParam.u32FrameHeight = enc_data[dev].slicebuf[i].mslice_height;	
           tEncParam.pu8ReConstructFrame=(unsigned char *)enc_data[dev].slicebuf[i].recon_phy_buffer;
           tEncParam.pu8ReferenceFrame=(unsigned char *)enc_data[dev].slicebuf[i].refer_phy_buffer;
           tEncParam.pu8SysInfoBuffer=(unsigned char *)enc_data[dev].slicebuf[i].sysinfo_phy_buffer;
           tEncParam.pu8DMABuffer_phy=(unsigned char *)enc_data[dev].slicebuf[i].dma_phy_buffer;
        
           // Multislice Y,U,V or UV base update
           switch (h264_fmt(enc_data[dev].enc_handle)) {
           case 3: // uyvy-1D
               tEncParam.pu8YFrameBaseAddr=(unsigned char *)YFrameBase + enc_data[dev].mslice_stride_yuv*SIZE_Y * 2;
               break;

           case 0: // h264-2D
               tEncParam.pu8YFrameBaseAddr = (unsigned char *)YFrameBase + enc_data[dev].mslice_stride_yuv*SIZE_Y ;
               tEncParam.pu8UVFrameBaseAddr = (unsigned char *)UFrameBase + enc_data[dev].mslice_stride_yuv*( SIZE_U + SIZE_V);	  
               break;

           default: // mp4-2D, 420YUV-1D
               tEncParam.pu8YFrameBaseAddr = (unsigned char *)YFrameBase + enc_data[dev].mslice_stride_yuv*SIZE_Y ;
               tEncParam.pu8UFrameBaseAddr = (unsigned char *)UFrameBase + enc_data[dev].mslice_stride_yuv*SIZE_U ;
               tEncParam.pu8VFrameBaseAddr = (unsigned char *)VFrameBase + enc_data[dev].mslice_stride_yuv*SIZE_V ;
               break;
           }

	//printk("%s : h264_encoder_encode\n",__FUNCTION__); 
           mslice_len = 0;  // Don't forget reset mslice_len to 0  
           if ( enc_data[dev].roi_enable == 1) {
               //printk("\ndrv: %x %x %x %x\n", enc_data[dev].slicebuf[i].mslice_ROI,
               //      enc_data[dev].slicebuf[i].mslice_ROIWidth,
               //      enc_data[dev].slicebuf[i].mslice_ROIHeight,
               //      enc_data[dev].slicebuf[i].mslice_ROIMB);
               if ( (enc_data[dev].slicebuf[i].mslice_ROI ==1) 
                   && (	enc_data[dev].slicebuf[i].mslice_ROIWidth != 0 )
                   && ( enc_data[dev].slicebuf[i].mslice_ROIHeight != 0)
                   && ( 	enc_data[dev].slicebuf[i].mslice_ROIMB != 0) ) {	
                   mslice_len = h264_encoder_encode( enc_data[dev].enc_handle,
                                 &tEncParam, 
                                 enc_data[dev].mslice_first_mb, 
                                 enc_data[dev].slicebuf[i].mslice_ROILast, 
                                 i);
                }
            } else {
                mslice_len = h264_encoder_encode(enc_data[dev].enc_handle,
                                 &tEncParam,
                                 enc_data[dev].mslice_first_mb, 
                                 enc_data[dev].slicebuf[i].mslice_last, 
                                 i);
            }


            if ( mslice_len < 0) {
                ret = -EFAULT;
                goto encoder_ioctl_exit;
            }
            else {

                // Bitstresm is written to phsical address buffer for application to handle. Driver only reportes this encoded length for application now
                
                flush_cache_all();
                    
//20140617                if (copy_to_user((void *)(tEncParam.bitstream + tEncParam.bitstream_size),(void *)out_virt_buffer+dev * ENC_ONE_INSTANCE_SIZE, mslice_len))
                if (copy_to_user((void *)(tEncParam.bitstream + tEncParam.bitstream_size),(void *)out_virt_buffer+ENCODER_INST[dev].u32Offset, mslice_len))                
                {
                    printk("Error copy_to_user() of FAVC_IOCTL_ENCODE_FRAME: out_virt_buffer\n");
                    ret = -EFAULT;
                    goto encoder_ioctl_exit;
                }
                                
                tEncParam.bitstream_size += mslice_len;
            }

//20160617            if (tEncParam.bitstream_size > ENC_BITSTREAM_SIZE)
            if (tEncParam.bitstream_size > ENCODER_INST[dev].u32BSsize)            
                printk("ERROR : BS length is bigger than buffer size\n");
                
            //printk("STRIDE_YUV %d\n", enc_data[dev].mslice_stride_yuv);
            enc_data[dev].mslice_stride_yuv += enc_data[dev].slicebuf[i].mslice_mb; // update mslice_stride_yuv						 
	     	  
            if ( enc_data[dev].roi_enable == 1) { // update mb skip number
                enc_data[dev].mslice_first_mb += enc_data[dev].slicebuf[i].mslice_ROIMB;
            } else {
                enc_data[dev].mslice_first_mb += enc_data[dev].slicebuf[i].mslice_mb;
            }

            //printk("skip mb run %d stride %d\n", enc_data[dev].mslice_first_mb, enc_data[dev].mslice_stride_yuv);
        } //for

        //toggle DMA buffer
        for ( i = 0; i<enc_data[dev].mslice_num; i++ ) { 
            h264_toggle_DMA_buffer(enc_data[dev].enc_handle,i);
        }
      
        //printk("Encode return size = %d\n",tEncParam.bitstream_size);
        if (copy_to_user((void *)arg,(void *)&tEncParam,sizeof(tEncParam)))
        {
            printk("Error copy_to_user() of FAVC_IOCTL_ENCODE_FRAME: tEncParam\n");
            ret = -EFAULT;
        }
       
        break;


    case FAVC_IOCTL_GET_SYSINFO:
        sysbuf_len=0;		
        for ( i = 0; i<enc_data[dev].mslice_num; i++ ) {   	
            int sys_inf_size=0;
            sbuf * pslc = &enc_data[dev].slicebuf[i];
            if(pslc->mslice_ROI){
                sys_inf_size = DIV_16(SYSBUF_SIZE(pslc->mslice_ROIWidth, pslc->mslice_ROIHeight));
            }
            else{
                sys_inf_size = DIV_16(SYSBUF_SIZE(pslc->mslice_width, pslc->mslice_height));
            }

            if (copy_to_user((void *)(arg+sysbuf_len),(unsigned char *)(pslc->sysinfo_phy_buffer), sys_inf_size ))
            {
                printk("Error copy_to_user() of FAVC_IOCTL_GET_SYSINFO\n");
                ret = -EFAULT;
                goto encoder_ioctl_exit;
            }
           
            sysbuf_len += sys_inf_size;	
        }
        break;

    case FAVC_IOCTL_GET_SPSPPS: 
  
        if (copy_from_user((void *)&tEncParam, (void *)arg, sizeof(tEncParam)))
        {
            printk("Error copy_from_user() of FAVC_IOCTL_GET_SPSPPS\n");
            ret = -EFAULT;
            goto encoder_ioctl_exit;
        }
       
        tEncParam.bitstream_size =  h264_encoder_spspps(enc_data[dev].enc_handle,&tEncParam);		


//0617  if (copy_to_user((void *)tEncParam.bitstream,(void *)out_phy_buffer, tEncParam.bitstream_size))
        if (copy_to_user((void *)tEncParam.bitstream,(void *)(out_virt_buffer + ENCODER_INST[dev].u32Offset), tEncParam.bitstream_size))        
        {
            printk("Error copy_to_user() of FAVC_IOCTL_GET_SPSPPS: out_phy_buffer\n");
            ret = -EFAULT;
        }
        if (copy_to_user((void *)arg,(void *)&tEncParam,sizeof(tEncParam)))
        {
            printk("Error copy_to_user() of FAVC_IOCTL_GET_SPSPPS: tEncParam\n");
            ret = -EFAULT;
        }
        
        break;

/*
    case FAVC_IOCTL_ENCODE_NVOP: // not implement OK, don't use it
  
        if (copy_from_user((void *)&tEncParam, (void *)arg, sizeof(tEncParam)))
        {
            printk("Error copy_from_user() of FAVC_IOCTL_ENCODE_NVOP\n");
            ret = -EFAULT;
            goto encoder_ioctl_exit;
        }
       
        tEncParam.nvop_ioctl = 1;
        tEncParam.pu8BitstreamAddr=(unsigned char *)out_phy_buffer;
        for ( i = 0; i<enc_data[dev].mslice_num; i++ ) {   			
            mslice_len =  h264_encoder_nvop_nal(enc_data[dev].enc_handle,&tEncParam);		
            tEncParam.pu8BitstreamAddr=(unsigned char *)out_phy_buffer;


        if (copy_to_user((void *)(tEncParam.bitstream + tEncParam.bitstream_size),(void *)out_phy_buffer, mslice_len))
        {
            printk("Error copy_to_user() of FAVC_IOCTL_ENCODE_NVOP: out_phy_buffer\n");
            ret = -EFAULT;
            goto encoder_ioctl_exit;
        }
           
            tEncParam.bitstream_size += mslice_len;   
        }
       
        if (copy_to_user((void *)arg,(void *)&tEncParam,sizeof(tEncParam)))
        {
            printk("Error copy_to_user() of FAVC_IOCTL_ENCODE_NVOP: tEncParam\n");
            ret = -EFAULT;
        }
        
        break;
*/        
    case FAVC_IOCTL_ENCODE_GET_BSINFO:
        //printk("%s : FAVC_IOCTL_ENCODE_GET_BSINFO\n",__FUNCTION__);
		//workbuf.addr = out_phy_buffer + dev * ENC_ONE_INSTANCE_SIZE;	
		/* 20160617
		workbuf.addr = _ENCODER_BUF_START + dev * ENC_ONE_INSTANCE_SIZE;			
		workbuf.size = _avc_enc_buf_size;	
		workbuf.offset = dev *  ((_avc_enc_buf_size + 4095)/ 4096) * 4096;					        // Used it as the index by application
		*/
		workbuf.addr = _ENCODER_BUF_START + ENCODER_INST[dev].u32Offset;			
		workbuf.size = ENCODER_INST[dev].u32BSsize;	
		workbuf.offset = ((ENCODER_INST[dev].u32Offset + 4095)/ 4096) * 4096;					        // Used it as the index by application		
		
		//printk("%s : FAVC_IOCTL_ENCODE_GET_BSINFO -> addr=0x%x, size=0x%x, offset=0x%x\n",__FUNCTION__,workbuf.addr, workbuf.size, workbuf.offset);
		if (copy_to_user((void *)arg, (avc_workbuf_t *)&workbuf, sizeof(avc_workbuf_t))) {
			ret = -EFAULT;
			break;
		}    
        break;      
        
    case FAVC_IOCTL_ENCODE_GET_YUVSRCINFO:
        // Use Decoder buffer to verify the encoder in driver development phase
        //In the real case, the YUV buffer will be allocated by VideoIn driver, not the AVC driver
        //printk("%s : FAVC_IOCTL_ENCODE_GET_YUVINFO\n",__FUNCTION__);    
        
		workbuf.addr = dec_bs_phy_buffer;	
		//workbuf.addr = 0xe5b000;				
//0617		workbuf.size = _avc_dec_buf_size;	
		workbuf.size = DECODER_INST[0].u32BSsize;				
		//workbuf.offset = _avc_enc_total_size;			
		
		printk("%s : FAVC_IOCTL_ENCODE_GET_YUVSRCINFO -> addr=0x%x, size=0x%x\n",__FUNCTION__,workbuf.addr, workbuf.size);		
		if (copy_to_user((void *)arg, (avc_workbuf_t *)&workbuf, sizeof(avc_workbuf_t))) {
			ret = -EFAULT;
			break;
		}            
        break;          
		
    default:
        printk("[Error] Encoder Not support such IOCTL 0x%x\n", cmd);
        ret = -EFAULT;
        break;
    }

encoder_ioctl_exit:
#ifdef LINUX_ENV
    up(&favc_enc_mutex);
#endif    
    return ret;
}

//get continue input Y,U,V User address

#ifdef LINUX_ENV
int favc_encoder_mmap(struct file *filp, struct vm_area_struct *vma)
#else
int favc_encoder_mmap(void)
#endif
{
        int dev=*(int *)filp->private_data;    
        int err = 0;	
	unsigned long vsize = vma->vm_end - vma->vm_start;
	
	//printk ("Encoder Enter : vm_start= 0x%x, vm_end = 0x%x, vm_pgoff=0x%x vm_flag=0x%x, vm_page_prot=0x%x\n",(unsigned int)vma->vm_start, (unsigned int)vma->vm_end, 
	//       (unsigned int)vma->vm_pgoff,(unsigned int)vma->vm_flags,(unsigned int)vma->vm_page_prot);		
	//printk("_avc_enc_buf_size = 0x%x, _avc_dec_buf_size = 0x%x,vsize = 0x%x\n",(unsigned int)_avc_enc_buf_size, (unsigned int)_avc_dec_buf_size, (unsigned int)vsize);	
	
	if (out_phy_buffer == 0) {
		printk ("%s: allocate physical memory failed\n", __FUNCTION__);
		err = -EIO;
		return err;
	}	
	
	
    // Map Encoder Bitstream Buffer for each instance here
//0617	if ((vma->vm_pgoff == dev * _avc_enc_buf_size / 4096) && vsize == _avc_enc_buf_size) { // Map Encoder Buffer address to application	
	if ((vma->vm_pgoff == ENCODER_INST[dev].u32Offset / 4096) && vsize == ENCODER_INST[dev].u32BSsize) { // Map Encoder Buffer address to application		

		vma->vm_flags |= (VM_IO | VM_RESERVED);
		vma->vm_page_prot = pgprot_noncached (vma->vm_page_prot);
		
		//printk("%s : ENC BS mmap\n",__FUNCTION__);		
		//printk("%s : mmap Encoder buffer =0x%x, size = 0x%x\n",__FUNCTION__, (unsigned int)(_ENCODER_BUF_START+ + dev * ENC_ONE_INSTANCE_SIZE),(unsigned int)_avc_dec_buf_size);
		
//20140617		if ((_ENCODER_BUF_START+ + dev * ENC_ONE_INSTANCE_SIZE) & 0xFFF)
		if ((_ENCODER_BUF_START+ ENCODER_INST[dev].u32Offset) & 0xFFF)		
		    printk("Warning : MMAP addr =0x%xis not 4K alignmented has problem\n",(unsigned int)(_ENCODER_BUF_START + ENCODER_INST[dev].u32Offset));
		    
//20140617		if (remap_pfn_range (vma, vma->vm_start, (unsigned long)(_ENCODER_BUF_START+ dev * ENC_ONE_INSTANCE_SIZE) >> PAGE_SHIFT, vsize, vma->vm_page_prot) < 0) {		    
		if (remap_pfn_range (vma, vma->vm_start, (unsigned long)(_ENCODER_BUF_START+ ENCODER_INST[dev].u32Offset) >> PAGE_SHIFT, vsize, vma->vm_page_prot) < 0) {		    		    
			printk("%s : Enc BS mmap fail\n",__FUNCTION__);			
        		err = -EIO;
        	}
	}
	// This is only for verify encoder. The YUV source address should be allocated by VideoIn driver
	/*
#if 1
    //debug
    else if ((vma->vm_pgoff != 0) && (vsize == _avc_dec_buf_size)) {
    //else if ((vma->vm_pgoff != 0) && (vsize == 0x280000)) {        
#else	
	else if (vma->vm_pgoff == YUVoffset && vsize == _avc_dec_buf_size) {
#endif	    
		vma->vm_flags |= (VM_IO | VM_RESERVED);
		vma->vm_page_prot = pgprot_noncached (vma->vm_page_prot);

	
		printk("%s : mmap Decoder buffer =0x%x, size = 0x%x\n",__FUNCTION__, (unsigned int)_DECODER_BUF_START ,(unsigned int)_avc_dec_buf_size);
		//printk("%s : mmap Decoder buffer =, size = \n",__FUNCTION__);		
		if (remap_pfn_range (vma, vma->vm_start, (unsigned long)_DECODER_BUF_START >> PAGE_SHIFT, vsize, vma->vm_page_prot) < 0) {		    	
	    
			printk("%s : Enc YUV source mmap fail\n",__FUNCTION__);
        		err = -EIO;
        	}		
	}
	*/
	else {
		printk ("Encoder Unsupport mmap vma setting: \n");
		printk ("vm_start= 0x%x, vm_end = 0x%x, vm_pgoff=0x%x\n",(unsigned int)vma->vm_start, (unsigned int)vma->vm_end, (unsigned int)vma->vm_pgoff);
		err = -EINVAL;
	}	
	
	return err;	
}


#ifdef LINUX_ENV
int favc_encoder_release(struct inode *inode, struct file *filp)
#else
int favc_encoder_release(void)
#endif
{
#ifdef LINUX_ENV
    int  dev=*(int *)filp->private_data;
#else    
int dev=0;
#endif   
    int i; 

    //printk("%s Enter\n",__FUNCTION__);
	

    down(&favc_enc_mutex);

 //printk("%s : Calling h264_encoder_destroy\n",__FUNCTION__);
    if(enc_data[dev].enc_handle)
        h264_encoder_destroy(enc_data[dev].enc_handle);
    enc_data[dev].enc_handle=0; 
 
  //printk("%s : free private_data = 0x%x\n",__FUNCTION__, (unsigned int)filp->private_data);
    if(filp->private_data)
        kfree(filp->private_data);
    filp->private_data=0;    

   
    for ( i = 0; i<enc_data[dev].mslice_num; i++ ) {
        if(enc_data[dev].slicebuf[i].recon_phy_buffer)
        { 
            //printk("%s free recon_phy_buffer at 0x%x\n",__FUNCTION__, (unsigned int)enc_data[dev].slicebuf[i].recon_phy_buffer);            
            //nv_free((void *)enc_data[dev].slicebuf[i].recon_phy_buffer);  
            enc_data[dev].slicebuf[i].recon_phy_buffer = 0;          
		}
        if(enc_data[dev].slicebuf[i].refer_phy_buffer)
        {
            //printk("%s free refer_phy_buffer at 0x%x\n",__FUNCTION__, (unsigned int)enc_data[dev].slicebuf[i].refer_phy_buffer);  
            //nv_free((void *)enc_data[dev].slicebuf[i].refer_phy_buffer);
            enc_data[dev].slicebuf[i].refer_phy_buffer=0;
        }               

        enc_data[dev].slicebuf[i].dma_phy_buffer=0;
        enc_data[dev].slicebuf[i].sysinfo_phy_buffer=0;	 
    }
	
  //printk("%s : ENC_IDX_MASK_n\n",__FUNCTION__);	
    enc_idx &= (~(ENC_IDX_MASK_n(dev)));
    
    if (enc_idx == 0)
    {
        // disable clock
	    clk_disable(clk_264enc);
	    clk_put(clk_264enc);         
        //outp32(REG_AHBCLK2, inp32(REG_AHBCLK2) & ~VENC_CKE); 
//        printk("Disable H.264 encoder clock\n");               
    }
        

    up(&favc_enc_mutex);

  //printk("%s : Exit\n",__FUNCTION__);		
    return 0;
}		

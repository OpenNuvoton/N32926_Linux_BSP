/********************************************/
/* Faraday avc decoder driver operations    */
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
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/clk.h>
#else
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#endif

#include <mach/w55fa92_reg.h>
#include "../h264dec.h"
#include "AVCdec.h"
#include "decoder.h"

#include "favc_version.h"
#include "encoder.h"
//#include "user_define.h"

#ifndef LINUX_ENV
#include "wblib.h"
#endif

#ifdef CONFIG_ENABLE_ENCODER_ONLY
#define MAX_DEC_NUM         (1)
#else
#define MAX_DEC_NUM         CONFIG_W55FA92_AVC_DECODER_NUM
#endif

#define DEC_IDX_MASK_n(n)  (0x1<< n)
#define DEC_IDX_FULL        0xFFFFFFFF

extern unsigned int    dec_bs_phy_buffer;
extern unsigned long _ENCODER_BUF_START,_DECODER_BUF_START;
extern unsigned int dec_out_phy_buf;

extern DECODER_INFO DECODER_INST[4];

#ifdef LINUX_ENV
extern spinlock_t           favc_dec_lock;
static int    dec_idx=0;
extern struct semaphore     favc_dec_mutex;
#endif

struct dec_private
{
    void            *dec_handle;
    unsigned int    frame_width;
    unsigned int    frame_height;
    unsigned int    video_width;
    unsigned int    video_height;
};

struct dec_private      dec_data[MAX_DEC_NUM];

extern int favc_check_continued(unsigned int addr, int size);

#ifdef EVALUATION_PERFORMANCE
extern void dec_performance_count(void);
extern void get_dap_start(void);
extern void get_dap_stop(void);
extern void get_cp_start(void);
extern void get_cp_stop(void);
extern void get_total_start(void);
extern void get_total_stop(void);
extern void dec_performance_reset(void);
extern void dec_performance_report(void);
#endif

static FAVC_DEC_RESULT     _tDecResult;

struct clk *clk_264dec = NULL;

int favc_decoder_open(struct inode *inode, struct file *filp)
{
    int idx;

    int used_index=-1;
    
    used_index = (used_index >> MAX_DEC_NUM) << MAX_DEC_NUM;
    used_index = used_index ^ (-1); 
    if ((dec_idx & used_index) == used_index)
    {
        printk("Decoder Device Service Full,0x%x!\n",dec_idx);
        return -EFAULT;
    }
    	
    down(&favc_dec_mutex);
    idx=0;	
    while( idx < MAX_DEC_NUM) {
        if( dec_idx&DEC_IDX_MASK_n(idx) ) {
            idx++;
        } else{
            dec_idx |= DEC_IDX_MASK_n(idx);
            break;
        }
    }
    filp->private_data=kmalloc(sizeof(unsigned int),GFP_KERNEL);
    if(filp->private_data==0)   {
        printk("Can't allocate memory!\n");
        up(&favc_dec_mutex);
        return -EFAULT;
    }	
    *(unsigned int *)filp->private_data=idx;
    memset(&dec_data[idx],0,sizeof(struct dec_private));
    
    //Enable H.264 Decoder clock for first instance
    if (idx ==0)
    {
		unsigned long flags;
		
        // enable clock
	    clk_264dec = clk_get(NULL, "vdec");
	    clk_enable(clk_264dec);
     
//        outp32(REG_AHBCLK2, inp32(REG_AHBCLK2) | VDEC_CKE);
        dbg_printk("Enable H.264 decoder clock\n");		
		
		// reset IP
		local_irq_save(flags);        
        outp32(REG_AHBIPRST, VDE_RST);
        outp32(REG_AHBIPRST, 0); 
		local_irq_restore(flags);               
    }    
    
    up(&favc_dec_mutex);

    return 0; /* success */
}

int favc_decoder_OneFrame(void *ptDecHandle, FAVC_DEC_RESULT *result,int dev)
{
	int ret = 0;
 retry:
	ret = AVCDec_OneFrame(ptDecHandle, result, dev);

	if(ret == RETCODE_HEADER_READY)
		goto retry;

		
	return ret;
}


int favc_decoder_ioctl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg)
{
    int                 dev=*(int *)filp->private_data;
    int                 ret = 0;
    
    FAVC_DEC_PARAM      tDecParam;
	avc_workbuf_t workbuf;    
	
	dbg_printk("%s : cmd=%x\n",__FUNCTION__, cmd);
    
    if(((1 << dev) & dec_idx) == 0) {
        printk("No Support index %d for 0x%x\n",dev,dec_idx);
        return -EFAULT;
    }    

    down(&favc_dec_mutex);
    switch(cmd){
        case FAVC_IOCTL_DECODE_INIT:
	        dbg_printk("%s : cmd= FAVC_IOCTL_DECODE_INIT\n",__FUNCTION__);        
      
            if (copy_from_user((void *)&tDecParam, (void *)arg, sizeof(tDecParam)))
            {
                printk("Error copy_from_user() of FAVC_IOCTL_DECODE_INIT\n");
                ret=-EFAULT;
                goto decoder_ioctl_exit;
            }
           
            dbg_printk("%s : copy_from_user : OK\n",__FUNCTION__); 
            
             
            dec_data[dev].frame_width=tDecParam.u32FrameBufferWidth;
            dec_data[dev].frame_height=tDecParam.u32FrameBufferHeight;
            dec_data[dev].video_width=0;
            dec_data[dev].video_height=0;
            if(tDecParam.u32API_version!=H264VER) {
                 printk("Fail API Version v%d.%d (Current Driver v%d.%d)\n",
		        tDecParam.u32API_version>>16,tDecParam.u32API_version&0xffff,H264VER_MAJOR,H264VER_MINOR);
		        printk("Please upgrade your H264 driver and re-compiler AP.\n");
		        ret=-EFAULT;
		        goto decoder_ioctl_exit;
            }

            // KC : Set Decoder BS buffer here
           //tDecParam.pu8BitStream_phy = (unsigned char *)dec_bs_phy_buffer; 
//20150617            tDecParam.pu8BitStream_phy = (unsigned char *)_DECODER_BUF_START + dev * DEC_ONE_INSTANCE_SIZE;  
            tDecParam.pu8BitStream_phy = (unsigned char *)_DECODER_BUF_START + DECODER_INST[dev].u32Offset; 
    
            
            dbg_printk("%s : tDecParam.pu8BitStream_phy = 0x%x, dev=%d\n",__FUNCTION__,(unsigned int)tDecParam.pu8BitStream_phy, dev); 
            
         
	        dbg_printk("%s : Calling AVCDec_Init\n",__FUNCTION__); 
            if ((ret=AVCDec_Init(&tDecParam, (void **)&(dec_data[dev].dec_handle), dev)) != RETCODE_OK){
                printk("FAVC_Init decoder failure, Error Number %d\n",ret);
                ret=-EFAULT;
                goto decoder_ioctl_exit;
            }
            //F_DEBUG("decoder create finished\n");
	        //dbg_printk("%s : AVCDec_Init : OK",__FUNCTION__);             
            break;
			
 
        case FAVC_IOCTL_DECODE_FRAME:
#ifdef EVALUATION_PERFORMANCE
            get_dap_stop();
            get_total_start();  
#endif        
	        dbg_printk("%s : cmd= FAVC_IOCTL_DECODE_FRAME\n",__FUNCTION__);         
       
            if (copy_from_user((void *)&tDecParam, (void *)arg, sizeof(tDecParam)))
            {
                printk("Error copy_from_user() of FAVC_IOCTL_DECODE_FRAME\n");
                ret=-EFAULT;
                goto decoder_ioctl_exit;
            }
           
            dbg_printk("%s : Calling AVCDec_ReInit\n",__FUNCTION__); 
            AVCDec_ReInit(dec_data[dev].dec_handle);
            
#ifdef EVALUATION_PERFORMANCE
            get_cp_start();
#endif
            dbg_printk("%s : Calling AVCDec_FillBuffer, size=%d\n",__FUNCTION__,tDecParam.u32Pkt_size);
            if ((ret = AVCDec_FillBuffer(dec_data[dev].dec_handle, tDecParam.pu8Pkt_buf, tDecParam.u32Pkt_size, 1)) != RETCODE_OK) {
                printk("AVCDec_FillBuffer() decoder failure, Error Number %d\n",ret);                
                ret=-EFAULT;
                goto decoder_ioctl_exit;
            }
            
#ifdef EVALUATION_PERFORMANCE
            get_cp_stop();
#endif 
            
            if (tDecParam.u32Pkt_size ==0)
	            AVCDec_EndOfData(dec_data[dev].dec_handle);


            decoder_dummy_write(dec_data[dev].dec_handle);
			
      
          AVCDec_SetOutputAddr(dec_data[dev].dec_handle,
                (unsigned char *)(tDecParam.pu8Display_addr[0]),
                (unsigned char *)(tDecParam.pu8Display_addr[1]),
                (unsigned char *)(tDecParam.pu8Display_addr[2]));                
            AVCDec_SetCrop(dec_data[dev].dec_handle, tDecParam.crop_x, tDecParam.crop_y);
            
            //printk("%s : Calling favc_decoder_OneFrame\n",__FUNCTION__); 
            if((ret=favc_decoder_OneFrame(dec_data[dev].dec_handle, &_tDecResult,dev) )< 0){
//ret=-EFAULT;
                tDecParam.got_picture=0;
            } else {
            	dec_data[dev].video_width=_tDecResult.u32Width; 
                dec_data[dev].video_height=_tDecResult.u32Height;            
              	tDecParam.got_picture=1;
            }
            //printk("%s : favc_decoder_OneFrame  ret=%d\n",__FUNCTION__,ret); 
            memcpy((unsigned char *)&tDecParam.tResult, (unsigned char *)&_tDecResult,sizeof(FAVC_DEC_RESULT));
            
            //printk("%s : copy_to user\n",__FUNCTION__); 
            if (copy_to_user((void *)arg,(void *)&tDecParam,sizeof(tDecParam)))
            {
                printk("Error copy_to_user() of FAVC_IOCTL_DECODE_FRAME\n");
                ret=-EFAULT;
                goto decoder_ioctl_exit;
            }       
            //printk("%s : copy_to user arg = %x, tDecParam=%x\n",__FUNCTION__, (unsigned int)arg, (unsigned int)&tDecParam); 
#ifdef EVALUATION_PERFORMANCE
            get_dap_start();
            get_total_stop();     
            dec_performance_count();                     
#endif 
            
            break;
            
        case FAVC_IOCTL_DECODE_GET_BSSIZE:
       
//0617		if (copy_to_user((void *)arg, (void *)&_avc_dec_buf_size, sizeof(int))) {	
		if (copy_to_user((void *)arg, (void *)&DECODER_INST[dev].u32BSsize, sizeof(int))) {			    	    
			ret = -EFAULT;
			break;
		}         
            break;    
            
        case FAVC_IOCTL_DECODE_GET_BSINFO:          // It is only for debug. Should not call here
       
		//workbuf.addr = dec_bs_phy_buffer;	
//20140617		workbuf.addr = _DECODER_BUF_START + dev * DEC_ONE_INSTANCE_SIZE;	
//20160617		workbuf.size = _avc_dec_buf_size;	
		workbuf.addr = _DECODER_BUF_START + DECODER_INST[dev].u32Offset;	
		workbuf.size = DECODER_INST[dev].u32BSsize;		
		workbuf.offset = ((1 +4095)/ 4096) * 4096;        // return 4096 
		       
		//if (copy_to_user((void *)arg, (void *)&_avc_dec_buf_size, sizeof(int))) {
		if (copy_to_user((void *)arg, (avc_workbuf_t *)&workbuf, sizeof(avc_workbuf_t))) {			    
			ret = -EFAULT;
			break;
		}         
            break;              
            
        case FAVC_IOCTL_DECODE_GET_OUTPUT_INFO:
        // For Output Buffer Information in mmap
		//workbuf.addr = dec_out_phy_buf;
//20140617		workbuf.addr = _DECODER_BUF_START + dev * DEC_ONE_INSTANCE_SIZE + DECODER_SUBTOTAL_SIZE;			
//20140617		workbuf.size = _avc_dec_output_buf_size;
        workbuf.addr = _DECODER_BUF_START + DECODER_INST[dev].u32Offset + DECODER_INST[dev].u32OutputBufOffset;
        workbuf.size = DECODER_INST[dev].u32OutputBufSize;	
		workbuf.offset = 0;        // return 0 for Output buffer mmap 
       
		if (copy_to_user((void *)arg, (avc_workbuf_t *)&workbuf, sizeof(avc_workbuf_t))) {		    
			ret = -EFAULT;
			break;
		}         
            break;
#ifdef EVALUATION_PERFORMANCE	            
        case FAVC_IOCTL_PERFORMANCE_RESET: 
            dec_performance_reset();
            break;
        case FAVC_IOCTL_PERFORMANCE_REPORT:
            dec_performance_report();          
            break;            
#endif        
			
        default:
            printk("[Error] Decoder Not support such IOCTL 0x%x\n", cmd);
            break;
    }

decoder_ioctl_exit:
    up(&favc_dec_mutex);
    
    return ret;
}

//get continue output Y,U,V User address
int favc_decoder_mmap(struct file *filp, struct vm_area_struct *vma)
{
    int dev=*(int *)filp->private_data;     
    int err = 0;	
	unsigned long vsize = vma->vm_end - vma->vm_start;
	
	//printk ("Enter : vm_start= 0x%x, vm_end = 0x%x, vm_pgoff=0x%x vm_flag=0x%x, vm_page_prot=0x%x\n",(unsigned int)vma->vm_start, (unsigned int)vma->vm_end, 
	//        (unsigned int)vma->vm_pgoff,(unsigned int)vma->vm_flags,(unsigned int)vma->vm_page_prot);		
	//printk("_avc_dec_output_buf_size = 0x%x, vsize = 0x%x\n",(unsigned int)_avc_dec_output_buf_size, (unsigned int)vsize);		        

//0617	if (vma->vm_pgoff == 0 && vsize == _avc_dec_output_buf_size) {    // Map Decoder Output Buffer address to application
	if (vma->vm_pgoff == 0 && vsize == DECODER_INST[dev].u32OutputBufSize) {    // Map Decoder Output Buffer address to application	
		vma->vm_flags |= (VM_IO | VM_RESERVED);
		vma->vm_page_prot = pgprot_noncached (vma->vm_page_prot);
		
		//printk("H264 decoder output mmap\n");
		printk("dev = %d, map output buf addr= 0x%x\n", dev, (unsigned int)(_DECODER_BUF_START+ DECODER_INST[dev].u32Offset + DECODER_INST[dev].u32OutputBufOffset));
		//if (remap_pfn_range (vma, vma->vm_start, (unsigned long)dec_out_phy_buf >> PAGE_SHIFT, _avc_dec_output_buf_size, vma->vm_page_prot) < 0) {
//20140617		if (remap_pfn_range (vma, vma->vm_start, (unsigned long)(_DECODER_BUF_START+ dev * DEC_ONE_INSTANCE_SIZE + DECODER_SUBTOTAL_SIZE)>> PAGE_SHIFT, _avc_dec_output_buf_size, vma->vm_page_prot) < 0) {		    
            if (remap_pfn_range (vma, vma->vm_start, (unsigned long)(_DECODER_BUF_START+ DECODER_INST[dev].u32Offset + DECODER_INST[dev].u32OutputBufOffset)>> PAGE_SHIFT, DECODER_INST[dev].u32OutputBufSize, vma->vm_page_prot) < 0) {	    
			printk("H264 decoder Output mmap fail\n");
        		err = -EIO;
        	}		
	}	
//0617	else if (vma->vm_pgoff != 0 && vsize == _avc_dec_buf_size) { // Map Decoder bitstream Buffer address to application	(Not Support Now)
	else if (vma->vm_pgoff != 0 && vsize == DECODER_INST[dev].u32BSsize) { // Map Decoder bitstream Buffer address to application	(Not Support Now)	
	   //     err = -EIO;     // Don't support BS buffer mmap now.
	    
		vma->vm_flags |= (VM_IO | VM_RESERVED);
		vma->vm_page_prot = pgprot_noncached (vma->vm_page_prot);

//20140617 if (remap_pfn_range (vma, vma->vm_start, (unsigned long)(_DECODER_BUF_START+ dev * DEC_ONE_INSTANCE_SIZE) >> PAGE_SHIFT, vsize, vma->vm_page_prot) < 0) {			    		
		if (remap_pfn_range (vma, vma->vm_start, (unsigned long)(_DECODER_BUF_START+ DECODER_INST[dev].u32Offset) >> PAGE_SHIFT, vsize, vma->vm_page_prot) < 0) {			    
			printk("IP Reg mmap fail\n");			
        		err = -EIO;
        	}
        	
	}
	else {
		printk ("H264 Decoder Unsupport mmap vma setting: \n");
		err = -EINVAL;
	}	
	    

	return err;		

}


int favc_decoder_release(struct inode *inode, struct file *filp)
{

    int dev=*(int *)filp->private_data;
   
    down(&favc_dec_mutex);
   
    if(dec_data[dev].dec_handle)
        AVCDec_Release(dec_data[dev].dec_handle);

    if(filp->private_data)
        kfree(filp->private_data);
    filp->private_data=0;
   
    dec_data[dev].dec_handle=0;
 
    dec_idx &= (~(DEC_IDX_MASK_n(dev)) );	
    //printk("favc_decoder_release\n");
    
    
    if (dec_idx == 0)
    {
        // disable decoder clock
        //outp32(REG_AHBCLK2, inp32(REG_AHBCLK2) & ~VDEC_CKE); 
	    clk_disable(clk_264dec);
	    clk_put(clk_264dec);        
        //printk("Disable H.264 decoder clock\n");              
    }
    
    up(&favc_dec_mutex);	

    return 0;
}	



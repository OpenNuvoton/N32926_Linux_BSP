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


#include "AVCdec.h"
#include "decoder.h"


extern DECODER *_pAVCDecHandle;
extern unsigned int w55fa92_vde_v;

int32_t
AVCDec_Init(FAVC_DEC_PARAM * ptParam, void ** pptDecHandle, unsigned char ndev)
{
	return decoder_create(ptParam, pptDecHandle, ndev);	

}

void
AVCDec_ReInit(void * ptDecHandle)
{
  decoder_reinit(ptDecHandle);
}


uint32_t
AVCDec_QueryEmptyBuffer (void * ptDecHandle)
{
	DECODER * dec = (DECODER *)ptDecHandle;

	return dec->u32BS_buf_sz - dec->u32BS_buf_sz_remain - 64;	// keep 64 byte gate	

}

AVC_RET                                 
AVCDec_FillBuffer(void * ptDecHandle, uint8_t * ptBuf, uint32_t u32BufSize, bool bfitFrameSize)
{
	DECODER * dec = (DECODER *)ptDecHandle;
	uint32_t size_after_fill = 0;
	uint32_t circular_size = 0;
	uint32_t resisual = 0;
	
	if (dec->pfnSemWait) dec->pfnSemWait (dec->pvSemaphore);	

	if (u32BufSize > (dec->u32BS_buf_sz - dec->u32BS_buf_sz_remain))
		return RETCODE_ERR_FILL_BUFFER;

	if (u32BufSize)
	{
		if(u32BufSize & 3) // multiple of word?
			u32BufSize = u32BufSize+(4-(u32BufSize & 3));
		  
		size_after_fill = dec->u32BS_sw_offset + u32BufSize;
		
		// christie '05/10/24 bitstream sync between CPU and HW
		if(size_after_fill > dec->u32BS_buf_sz)
		{
			circular_size = size_after_fill - dec->u32BS_buf_sz;
			
			//dbg_printk("%s : 1. Calling copy_from_user\n",__FUNCTION__);
		
			if (copy_from_user((void *)(dec->pu8BS_start_virt+dec->u32BS_sw_offset),(void *)ptBuf, u32BufSize-circular_size))
			{
				printk("Error copy_from_user() in 1 memcpy of AVCDec_FillBuffer\n");
			}	
				

			//ptBuf += (u32BufSize - circular_size);

			dec->u32BS_sw_offset = circular_size;
		}
		else
		{

			//dbg_printk("%s : 2. Calling copy_from_user, BS_start_phy=0x%x, sw_offset=%d, ptBuf=0x%x, Size=%d\n"
			 //   ,__FUNCTION__, (unsigned int)dec->pu8BS_start_virt, (unsigned int)dec->u32BS_sw_offset, (unsigned int)ptBuf, (unsigned int)u32BufSize);			
			    
			 //printk("Access pu8BS_start_phy\n");			    
			 //outp32(w55fa92_vde_v, 0x1234);  
			 //outp32(dec->pu8BS_start_phy+dec->u32BS_sw_offset, 0x1234);   
			 //printk("Access ptBuf\n");
			 //outp32(ptBuf, 0x4567);
			    
			if (copy_from_user((void *)(dec->pu8BS_start_virt+dec->u32BS_sw_offset), (void *)ptBuf, u32BufSize))
			{
				printk("Error copy_from_user() in 3 memcpy of AVCDec_FillBuffer\n");
			}		
			
			dec->u32BS_sw_offset += u32BufSize;  
		}
	}

	if(bfitFrameSize)
	{
		// fill in dummy 0
		resisual = 64 - (u32BufSize & (64 - 1)) + (4*32) + 64;
		size_after_fill = dec->u32BS_sw_offset + resisual;
		//printk("%s : 3. memset\n",__FUNCTION__);	
		if(size_after_fill > dec->u32BS_buf_sz)
		{		
		  circular_size = size_after_fill - dec->u32BS_buf_sz;
//		  memset((unsigned char *)(CACHE_BIT31 | (unsigned int)(dec->pu8BS_start_phy + dec->u32BS_sw_offset)), 0, (resisual - circular_size));			  	
  		  //memset((unsigned char *)(CACHE_BIT31 | (unsigned int)dec->pu8BS_start_phy), 0, circular_size);
		  memset((unsigned char *)(dec->pu8BS_start_virt + dec->u32BS_sw_offset), 0, (resisual - circular_size));			  	
  		  memset((unsigned char *)dec->pu8BS_start_virt, 0, circular_size);  		  
		  dec->u32BS_sw_offset = circular_size;
		}	
		else
		{
		  //memset((unsigned char *)(CACHE_BIT31 | (unsigned int)(dec->pu8BS_start_phy + dec->u32BS_sw_offset)), 0, resisual);
		  //printk("Fill 0 at 0x%x, size=0x%x\n",(unsigned int)(dec->pu8BS_start_virt + dec->u32BS_sw_offset), resisual);
		  memset((unsigned char *)(dec->pu8BS_start_virt + dec->u32BS_sw_offset), 0, resisual);		  		  
		  dec->u32BS_sw_offset += resisual;  
		}	
	}
	dec->u32BS_buf_sz_remain += (u32BufSize + resisual);
	dec->u32BS_sw_cur_fill_size += (u32BufSize + resisual);
	
	// HW setting
   	decoder_fill_bs_reg(ptDecHandle,(u32BufSize+63)/64 * 64);	
	
	if (dec->pfnSemSignal) dec->pfnSemSignal(dec->pvSemaphore);
		
	return RETCODE_OK;
}

uint32_t
AVCDec_QueryFilledBuffer (void * ptDecHandle)
{
	DECODER * dec = (DECODER *)ptDecHandle;

	return dec->u32BS_buf_sz_remain;
}


void
AVCDec_InvalidBS (void * ptDecHandle)
{
	DECODER * dec = (DECODER *)ptDecHandle;

	if (dec->pfnSemWait) dec->pfnSemWait (dec->pvSemaphore);
    dec->u32BS_hw_prev_offset = 0;
	dec->u32BS_buf_sz_remain = 0;
	if (dec->pfnSemSignal) dec->pfnSemSignal(dec->pvSemaphore);
}

void
AVCDec_EndOfData (void * ptDecHandle)
{
    DECODER *dec = (DECODER *)ptDecHandle;	

	dec->bBS_end_of_data = TRUE;	 
}

void
AVCDec_SetOutputAddr (void * ptDecHandle,
		uint8_t * pu8output_phy, uint8_t * pu8output_u_phy, uint8_t * pu8output_v_phy)
{
	DECODER * dec = (DECODER *)ptDecHandle;	

#if 0
	dec->output_base_phy = (unsigned char *)virt_to_phys((void *)pu8output_phy);
  	if ( dec->output_fmt == OUTPUT_FMT_YUV420) {	
	    dec->output_base_u_phy = (unsigned char *)virt_to_phys((void *)pu8output_u_phy);
	    dec->output_base_v_phy = (unsigned char *)virt_to_phys((void *)pu8output_v_phy);
    }	
#else    
	dec->output_base_phy = (unsigned char *)pu8output_phy;
  	if ( dec->output_fmt == OUTPUT_FMT_YUV420) {	
	    dec->output_base_u_phy = (unsigned char *)pu8output_u_phy;
	    dec->output_base_v_phy = (unsigned char *)pu8output_v_phy;
    }	
#endif	
    dbg_printk("output_base_phy = 0x%x, out_base_virt= 0x%x\n", (unsigned int)dec->output_base_phy, (unsigned int)pu8output_phy);
}

AVC_RET
AVCDec_OneFrame(void * ptDecHandle, FAVC_DEC_RESULT * ptResult, int dev)
{	
	AVC_RET ret;
	ret = decoder_decode(ptDecHandle, ptResult, dev);
	//printk("%s , ret=%x\n",__FUNCTION__,ret);
	ptResult->isISlice =  isISlice(ptDecHandle);
	return ret;
}

void AVCDec_SetCrop(void * ptDecHandle, int x, int y)
{	
	DECODER * dec = (DECODER *)ptDecHandle;

	dec->crop_x = (x+15)/16*16;
	dec->crop_y = (y+15)/16*16;
}

int32_t
AVCDec_Sync_OneFrame(void * ptDecHandle)            // KC : not called
{
	int32_t status;
	status = decoder_sync(ptDecHandle);
	
  	return status;
}

void
AVCDec_Release(void * ptDecHandle)
{  	
  	decoder_destroy(ptDecHandle);
}





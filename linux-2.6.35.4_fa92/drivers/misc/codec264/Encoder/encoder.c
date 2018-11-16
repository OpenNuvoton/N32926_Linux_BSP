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
#include "w55fa92_reg.h"
#include "wblib.h"
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

void frame_code_I(h264_encoder *pEnc);
void frame_code_P(h264_encoder *pEnc);
void reset_encoder(h264_encoder *pEnc);

static void h264_dma_set_curimg(h264_encoder *pEnc,int toggle);
static void h264_dma_set_all(h264_encoder *pEnc,int toggle);
static void h264_dma_init(h264_encoder *pEnc);
static void h264_reg_init(h264_encoder *pEnc, int idx);

#define FAVC_Encoder_TIMEOUT 100
#define ED1_Size (720*576)


#ifdef EVALUATION_PERFORMANCE
#define INTERVAL 5
#define TIME_INTERVAL 1000000*INTERVAL
typedef struct {
    unsigned int start;
    unsigned int stop;
    unsigned int hw_start;
    unsigned int hw_stop;
    unsigned int ap_start;
    unsigned int ap_stop;
    unsigned int cpy_start;
    unsigned int cpy_stop;
    unsigned int drv_start;
    unsigned int drv_stop;
} FRAME_TIME;

typedef struct {
    unsigned int drv_total;     // Total consumption time in FAVC_IOCTL_ENCODE_FRAME cmd (= total+cpy_total+...)
    unsigned int hw_total;      // Totla expend time for H/W encode bitstream (issue Encode Trigger cmd to complete).
    unsigned int ap_total;      // Total expend time for encode bitstream (not used in encoder)
    unsigned int cpy_total;     // Total expend time to copy bitstream between driver and application
    unsigned int total;         // Total expend time to encode bitstream (h264_encoder_encode expend time = hw_total + CPU time)
    unsigned int count;         // Total encoded frame number
} TOTAL_TIME;

typedef unsigned long long uint64;

static struct timeval tv_init, tv_curr;
static FRAME_TIME timeframe;
static TOTAL_TIME timetotal;

static uint64 time_delta(struct timeval *start, struct timeval *stop)
{
    uint64 secs, usecs;
    
    secs = stop->tv_sec - start->tv_sec;
    usecs = stop->tv_usec - start->tv_usec;
    if (usecs < 0)
    {
        secs--;
        usecs += 1000000;
    }
    return secs * 1000000 + usecs;
}

static void performance_count(void)
{
    timetotal.total += (timeframe.stop - timeframe.start); 
    timetotal.hw_total += (timeframe.hw_stop - timeframe.hw_start); 
    timetotal.drv_total += (timeframe.drv_stop - timeframe.drv_start); 
    timetotal.ap_total += (timeframe.ap_stop - timeframe.ap_start);
    timetotal.cpy_total += (timeframe.cpy_stop - timeframe.cpy_start);
}

static void performance_report(void)
{
    if ((tv_init.tv_sec ==0) && (tv_init.tv_usec==0))      // skip 1st data
        return; 
        
    if( timetotal.count != 0) {
        u32 tmp;
        tmp = time_delta( &tv_init, &tv_curr);
#if 0
        tmp = tmp/1000000;
        printk("Henc hw=%08d, ap=%08d, drv=%08d, count=%08d (unit :us)\n", 
            timetotal.hw_total/timetotal.count,
            timetotal.ap_total/timetotal.count,
            (timetotal.drv_total/timetotal.count),
            timetotal.count/tmp);
#else
            printk("H.264 enc fps = %4d.%d\n",  timetotal.count * 1000000 / timetotal.drv_total,
                                                timetotal.count * 1000000 % timetotal.drv_total * 100 / timetotal.drv_total);            
#endif
    }
}

static void performance_reset(void)
{
    timetotal.total = 0;
    timetotal.hw_total = 0;
    timetotal.drv_total = 0;
    timetotal.ap_total = 0;
    timetotal.count = 0;
}

static unsigned int get_counter(void)
{
    struct timeval tv;
    do_gettimeofday(&tv);

    return tv.tv_sec * 1000000 + tv.tv_usec;
}
void get_drv_start(void)
{
    timeframe.drv_start = get_counter();
}


#endif //#ifdef EVALUATION_PERFORMANCE


#ifdef LINUX_ENV
spinlock_t avcenc_request_lock = SPIN_LOCK_UNLOCKED;
wait_queue_head_t avc_enc_queue;
static int h264e_int;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,19))
irqreturn_t encoder_int_handler(int irq, void *dev_id)
#else
irqreturn_t encoder_int_handler(int irq, void *dev_id, struct pt_regs *dummy)
#endif
{
    unsigned long flags;
		 
    spin_lock_irqsave( &avcenc_request_lock, flags);

  #ifdef EVALUATION_PERFORMANCE
    timeframe.hw_stop = get_counter();
  #endif

    if(!CHECK_FRAME_DONE())
        printk("Warning: No frame done, but interrupt!\n");
    
    REG_BITWRITE(REG_264_STS2,STS2_FD_INT_STS,0); // set the interrupt register status to 0
  	
    if(REG_BITREAD(REG_264_STS2,STS2_FD_INT_STS))
        printk("Warning: error status!\n");

    h264e_int = 1;  
//printk("Set h264e_int\n");  
    
    wake_up(&avc_enc_queue);
    spin_unlock_irqrestore( &avcenc_request_lock, flags);	
    return IRQ_HANDLED;
}
#endif // #ifdef LINUX_ENV


void h264_dma_set_curimg(h264_encoder *pEnc,int toggle)
{
    unsigned int *pdma = (pEnc->pdma_buf+(toggle*DMA_BUF_WORD));

//for Non-Cache bit
     //pdma = (unsigned int *)((unsigned int)pdma | (unsigned int)0x80000000);
     
    {
        pdma[2] = REG_BITVAL(GSMBA_SMBA,(((unsigned int)pEnc->mEncParam.pu8YFrameBaseAddr+pEnc->roi_offset_y) >> 2)) | 
		REG_BITVAL(GSMBA_INCREMENT,3); // GSMBA

        // current MB chroma
        if( pEnc->mp4_2d  ) {
            pdma[6] = REG_BITVAL(GSMBA_SMBA,(((unsigned int)pEnc->mEncParam.pu8UFrameBaseAddr+pEnc->roi_offset_uv) >> 2) ) | 
		REG_BITVAL(GSMBA_INCREMENT,1); // GSMBA
            pdma[8] = REG_BITVAL(GBKWI_HALF_SLOT, (pEnc->roi_mb_width-1)) | //; // GBKWI
		REG_BITVAL(GBKWI_SYSMEM_BLOCK_WIDTH, 8) |
		REG_BITVAL(GBKWI_SYSMEM_LINE_OFFSET, (unsigned int)(((-15)+((pEnc->mEncParam.pu8VFrameBaseAddr-pEnc->mEncParam.pu8UFrameBaseAddr)/4)) & 0x0ffff)); // GBKWI    
        } else
            pdma[6] = REG_BITVAL(GSMBA_SMBA,(((unsigned int)(pEnc->mEncParam.pu8UVFrameBaseAddr+(pEnc->roi_offset_uv<<1))) >> 2) ) | 
		REG_BITVAL(GSMBA_INCREMENT,2); // GSMBA
    }
   
}

void h264_dma_set_all(h264_encoder *pEnc,int toggle)
{
    unsigned char *addr_y_recon,*addr_c_recon;
    unsigned char *addr_y_recon_row_1,*addr_c_recon_row_1;

    unsigned int recon_offset_y,recon_offset_c;
    unsigned char *addr_ilf_y,*addr_ilf_c;

    unsigned char *addr_y0_reference,*addr_y1_reference,*addr_c_reference;
    unsigned int reference_offset_y0,reference_offset_y1,reference_offset_c;

    unsigned char *addr_y1_reference_top;
    unsigned char *addr_y0_reference_bot;
    unsigned char *addr_y1_reference_bot;
    unsigned char *addr_c_reference_bot;

    unsigned int *pdma = (pEnc->pdma_buf+(toggle*DMA_BUF_WORD));
    
//for Non-Cache bit
     //pdma = (unsigned int *)((unsigned int)pdma | (unsigned int)0x80000000);     
  
    addr_y_recon = (toggle? pEnc->mEncParam.pu8ReferenceFrame:pEnc->mEncParam.pu8ReConstructFrame);
    addr_c_recon = addr_y_recon + pEnc->frame_luma_size; // the starting address of UV component of reconstructed image
    addr_y_recon_row_1 = addr_y_recon+pEnc->mb_width*256;
    addr_c_recon_row_1 = addr_c_recon+pEnc->mb_width*128;
    recon_offset_y = (-63-pEnc->mb_width*64+48) & 0x0ffff;
    recon_offset_c = (-31-pEnc->mb_width*32+16) & 0x0ffff;

    addr_ilf_y = addr_y_recon + 192;
    addr_ilf_c = addr_c_recon + 64;

    addr_y0_reference = (toggle ? pEnc->mEncParam.pu8ReConstructFrame:pEnc->mEncParam.pu8ReferenceFrame);
    addr_y1_reference = addr_y0_reference + 224;
    addr_c_reference = addr_y0_reference + pEnc->frame_luma_size;

    reference_offset_y0 = (-63+pEnc->mb_width*64)&0x0ffff;
    reference_offset_y1 = (-63+4*pEnc->mb_width*64)&0x0ffff;
    reference_offset_c  = (-31+pEnc->mb_width*32)&0x0ffff;

    addr_y1_reference_top = addr_y0_reference+2*pEnc->mb_width*256;
    addr_y0_reference_bot = addr_y0_reference+pEnc->frame_luma_size - 2*pEnc->mb_width*256;
    addr_y1_reference_bot = addr_y0_reference_bot - 2*pEnc->mb_width*256 + 224;
    addr_c_reference_bot  = addr_y0_reference+pEnc->frame_luma_size + ((pEnc->mb_height-2)*pEnc->mb_width*128);
    //addr_c_reference_bot  = addr_y0_reference+pEnc->frame_luma_size - (2*pEnc->mb_width*128);

//#ifdef _DEBUG
#if 0
    printk("addr_y_recon 0x%x\n",addr_y_recon);
    printk("addr_c_recon 0x%x\n",addr_c_recon);
    printk("addr_y_recon_row_1 0x%x\n",addr_y_recon_row_1);
    printk("addr_c_recon_row_1 0x%x\n",addr_c_recon_row_1);

    printk("recon_offset_y 0x%x\n",recon_offset_y);
    printk("recon_offset_c 0x%x\n",recon_offset_c);

    printk("addr_ilf_y 0x%x\n",addr_ilf_y);
    printk("addr_ilf_c 0x%x\n",addr_ilf_c);

    printk("addr_y0_reference 0x%x\n",addr_y0_reference);
    printk("addr_y1_reference 0x%x\n",addr_y1_reference);
    printk("addr_c_reference 0x%x\n",addr_c_reference);

    printk("reference_offset_y0 0x%x\n",reference_offset_y0);
    printk("reference_offset_y1 0x%x\n",reference_offset_y1);
    printk("reference_offset_c 0x%x\n",reference_offset_c);

    printk("addr_y1_reference_top 0x%x\n",addr_y1_reference_top);
    printk("addr_y0_reference_bot 0x%x\n",addr_y0_reference_bot);
    printk("addr_y1_reference_bot 0x%x\n",addr_y1_reference_bot);
    printk("addr_c_reference_bot 0x%x\n",addr_c_reference_bot);
#endif
  
    // to set the DMA command chain
    // next DMA address
    pdma[0] = ((unsigned int)(pEnc->mEncParam.pu8DMABuffer_phy+(toggle*DMA_BUF_SIZE)));  
    pdma[1] = DMA_BUF_WORD;

    {
        pdma[2] = REG_BITVAL(GSMBA_SMBA,(((unsigned int)pEnc->mEncParam.pu8YFrameBaseAddr+pEnc->roi_offset_y)>>2)) | 
		REG_BITVAL(GSMBA_INCREMENT,3); // GSMBA
        pdma[3] = REG_BITVAL(GCTRL_ROI_STYPE,1) | REG_BITVAL(GCTRL_SLOT_TRANSFER_LEN,64); // GCTRL
        pdma[4] = REG_BITVAL(GBKWI_HALF_SLOT,(pEnc->roi_mb_width-1)); // GBKWI
        pdma[5] = REG_BITVAL(GCBB_CIRCULAR_BUF_BASEADR,(((pEnc->roi_frame_mb_width - pEnc->roi_mb_width+1)*SIZE_Y)>>2)); // GCBB  

        // current MB chroma
        if( pEnc->mp4_2d  ) {
            pdma[6] = REG_BITVAL(GSMBA_SMBA,(((unsigned int)pEnc->mEncParam.pu8UFrameBaseAddr+pEnc->roi_offset_uv)>>2) ) | 
		REG_BITVAL(GSMBA_INCREMENT,1); // GSMBA
            pdma[7] = REG_BITVAL(GCTRL_STYPE,1) |
		REG_BITVAL(GCTRL_ROI_STYPE,1) |  
		REG_BITVAL(GCTRL_SLOT_TRANSFER_LEN,32); // GCTRL
            pdma[8] = REG_BITVAL(GBKWI_HALF_SLOT,(pEnc->roi_mb_width-1)) | //; // GBKWI
		REG_BITVAL(GBKWI_SYSMEM_BLOCK_WIDTH,8) |
		REG_BITVAL(GBKWI_SYSMEM_LINE_OFFSET,(unsigned int)(((-15)+((pEnc->mEncParam.pu8VFrameBaseAddr-pEnc->mEncParam.pu8UFrameBaseAddr)/4)) & 0x0ffff)); // GBKWI
            pdma[9] = REG_BITVAL(GCBB_CIRCULAR_BUF_BASEADR,(((pEnc->roi_frame_mb_width - pEnc->roi_mb_width+1)*SIZE_U)>>2)); // GCBB
        } else {
            pdma[6] = REG_BITVAL(GSMBA_SMBA,(((unsigned int)(pEnc->mEncParam.pu8UVFrameBaseAddr+(pEnc->roi_offset_uv<<1)))>>2) ) | 
		REG_BITVAL(GSMBA_INCREMENT,2); // GSMBA
            pdma[7] = REG_BITVAL(GCTRL_ROI_STYPE,1) | 
		REG_BITVAL(GCTRL_SLOT_TRANSFER_LEN,32); // GCTRL
            pdma[8] = REG_BITVAL(GBKWI_HALF_SLOT,(pEnc->roi_mb_width-1)); // GBKWI
            pdma[9] = REG_BITVAL(GCBB_CIRCULAR_BUF_BASEADR,(((pEnc->roi_frame_mb_width - pEnc->roi_mb_width+1)*SIZE_U*2)>>2)); // GCBB
        }
	
    }


    // reconstruct MB luma
    pdma[10] = REG_BITVAL(GSMBA_SMBA,(((unsigned int)addr_y_recon_row_1)>>2) ) | 
		REG_BITVAL(GSMBA_INCREMENT,3); // GSMBA
    pdma[11] = REG_BITVAL(GCTRL_DIR,1) |
		REG_BITVAL(GCTRL_STYPE,1) |
		REG_BITVAL(GCTRL_SLOT_TRANSFER_LEN,80); // GCTRL
    pdma[12] = REG_BITVAL(GBKWI_SYSMEM_BLOCK_WIDTH,32) |
		REG_BITVAL(GBKWI_SYSMEM_LINE_OFFSET,(unsigned int)recon_offset_y); // GBKWI                      
  //pdma[13] = 0; // GCBB  
  // reconstruct MB chroma
  pdma[14] = REG_BITVAL(GSMBA_SMBA,(((unsigned int)addr_c_recon_row_1)>>2)) | 
             REG_BITVAL(GSMBA_INCREMENT,2); // GSMBA
  pdma[15] = REG_BITVAL(GCTRL_DIR,1) |
             REG_BITVAL(GCTRL_STYPE,1) |                    // KC : Faraday reply 2D address mode
             REG_BITVAL(GCTRL_SLOT_TRANSFER_LEN,48); // GCTRL
  pdma[16] = REG_BITVAL(GBKWI_SYSMEM_BLOCK_WIDTH,16) |
             REG_BITVAL(GBKWI_SYSMEM_LINE_OFFSET,(unsigned int)recon_offset_c); // GBKWI                      
  //pdma[17] = 0; // GCBB
  
  // luma ilf upper block
  pdma[18] = REG_BITVAL(GSMBA_SMBA,(((unsigned int)addr_ilf_y)>>2)) | 
             REG_BITVAL(GSMBA_INCREMENT,3); // GSMBA
  pdma[19] = REG_BITVAL(GCTRL_SLOT_TRANSFER_LEN,16); // GCTRL
  //pdma[20] = 0; // GBKWI
  //pdma[21] = 0; // GCBB
  
  // chroma ilf upper block
  pdma[22] = REG_BITVAL(GSMBA_SMBA,(((unsigned int)addr_ilf_c)>>2)) | 
             REG_BITVAL(GSMBA_INCREMENT,2); // GSMBA
  pdma[23] = REG_BITVAL(GCTRL_SLOT_TRANSFER_LEN,16); // GCTRL
  //pdma[24] = 0; // GBKWI
  //pdma[25] = 0; // GCBB
  
  // system information write
  pdma[26] = REG_BITVAL(GSMBA_SMBA,(((unsigned int)pEnc->sys_info_buf)>>2)) | 
             REG_BITVAL(GSMBA_INCREMENT,1); // GSMBA
  pdma[27] = REG_BITVAL(GCTRL_SLOT_TRANSFER_LEN,0) |
  	    	REG_BITVAL(GCTRL_DIR,1) |
         	REG_BITVAL(GCTRL_STYPE,0) |
            REG_BITVAL(GCTRL_SLOT_TRANSFER_LEN,12); // GCTRL
            

  pdma[28] = REG_BITVAL(GBKWI_HALF_SLOT,(pEnc->mb_width*pEnc->mb_height-1)); // GBKWI

  pdma[29] = REG_BITVAL(GCBB_CIRCULAR_BUF_BASEADR,(((unsigned int)pEnc->sys_info_buf)>>2)); // GCBB
  
  // system information read
  pdma[30] = REG_BITVAL(GSMBA_SMBA,(((unsigned int)pEnc->sys_info_buf)>>2)) | 
             REG_BITVAL(GSMBA_INCREMENT,1); // GSMBA
  pdma[31] =  REG_BITVAL(GCTRL_SLOT_TRANSFER_LEN,0) |
  	    REG_BITVAL(GCTRL_DIR,0) |
             REG_BITVAL(GCTRL_STYPE,0) |
             REG_BITVAL(GCTRL_SLOT_TRANSFER_LEN,12); // GCTRL
             
  pdma[32] = REG_BITVAL(GBKWI_HALF_SLOT,(pEnc->mb_width*pEnc->mb_height-1)); // GBKWI

  pdma[33] = REG_BITVAL(GCBB_CIRCULAR_BUF_BASEADR,(((unsigned int)pEnc->sys_info_buf)>>2)); // GCBB
  
  // reference luma group 0
  pdma[34] = REG_BITVAL(GSMBA_SMBA,(((unsigned int)addr_y0_reference)>>2) ) | 
             REG_BITVAL(GSMBA_INCREMENT,3); // GSMBA
  pdma[35] = REG_BITVAL(GCTRL_STYPE,1) |
             REG_BITVAL(GCTRL_SLOT_TRANSFER_LEN,192); // GCTRL
  pdma[36] = REG_BITVAL(GBKWI_SYSMEM_BLOCK_WIDTH,32) |
             REG_BITVAL(GBKWI_SYSMEM_LINE_OFFSET,(unsigned int)reference_offset_y0); // GBKWI  
  //pdma[37] = 0; // GCBB  
  
  // reference luma group 1
  pdma[38] = REG_BITVAL(GSMBA_SMBA,(((unsigned int)addr_y1_reference)>>2) ) | 
             REG_BITVAL(GSMBA_INCREMENT,3); // GSMBA
  pdma[39] = REG_BITVAL(GCTRL_STYPE,1) |
             REG_BITVAL(GCTRL_SLOT_TRANSFER_LEN,16); // GCTRL
  pdma[40] = REG_BITVAL(GBKWI_SYSMEM_BLOCK_WIDTH,4) |
             REG_BITVAL(GBKWI_SYSMEM_LINE_OFFSET,(unsigned int)reference_offset_y1); // GBKWI  
  //pdma[41] = 0; // GCBB  
  
  // reference chroma
  pdma[42] = REG_BITVAL(GSMBA_SMBA,(((unsigned int)addr_c_reference)>>2) ) |  
             REG_BITVAL(GSMBA_INCREMENT,2); // GSMBA
  pdma[43] = REG_BITVAL(GCTRL_STYPE,1) |
             REG_BITVAL(GCTRL_SLOT_TRANSFER_LEN,96); // GCTRL
  pdma[44] = REG_BITVAL(GBKWI_SYSMEM_BLOCK_WIDTH,16) |
             REG_BITVAL(GBKWI_SYSMEM_LINE_OFFSET,(unsigned int)reference_offset_c); // GBKWI  
  //pdma[45] = 0; // GCBB  
  
  // reference luma group 0 top
  pdma[46] = REG_BITVAL(GSMBA_SMBA,(((unsigned int)addr_y0_reference)>>2) ) | 
             REG_BITVAL(GSMBA_INCREMENT,3); // GSMBA
  pdma[47] = REG_BITVAL(GCTRL_STYPE,1) |
             REG_BITVAL(GCTRL_SLOT_TRANSFER_LEN,128); // GCTRL
  pdma[48] = REG_BITVAL(GBKWI_SYSMEM_BLOCK_WIDTH,32) |
             REG_BITVAL(GBKWI_SYSMEM_LINE_OFFSET,(unsigned int)reference_offset_y0); // GBKWI  
  //pdma[49] = 0; // GCBB 

  // reference luma group 1 top
  pdma[50] = REG_BITVAL(GSMBA_SMBA,(((unsigned int)addr_y1_reference_top)>>2) ) | 
                 REG_BITVAL(GSMBA_INCREMENT,3); // GSMBA
  pdma[51] = REG_BITVAL(GCTRL_SLOT_TRANSFER_LEN,8); // GCTRL
  //pdma[52] = 0; // GBKWI  
  //pdma[53] = 0; // GCBB
  
  // reference chroma top
  pdma[54] = REG_BITVAL(GSMBA_SMBA,(((unsigned int)addr_c_reference)>>2)) |  
             REG_BITVAL(GSMBA_INCREMENT,2); // GSMBA
  pdma[55] = REG_BITVAL(GCTRL_STYPE,1) |
             REG_BITVAL(GCTRL_SLOT_TRANSFER_LEN,64); // GCTRL
  pdma[56] = REG_BITVAL(GBKWI_SYSMEM_BLOCK_WIDTH,16) |
             REG_BITVAL(GBKWI_SYSMEM_LINE_OFFSET,(unsigned int)reference_offset_c); // GBKWI  
  //pdma[57] = 0; // GCBB
  
  // reference luma group0 bot
  pdma[58] = REG_BITVAL(GSMBA_SMBA,(((unsigned int)addr_y0_reference_bot)>>2)) |
             REG_BITVAL(GSMBA_INCREMENT,3); // GSMBA
  pdma[59] = REG_BITVAL(GCTRL_STYPE,1) |
             REG_BITVAL(GCTRL_SLOT_TRANSFER_LEN,128); // GCTRL
  pdma[60] = REG_BITVAL(GBKWI_SYSMEM_BLOCK_WIDTH,32) |
             REG_BITVAL(GBKWI_SYSMEM_LINE_OFFSET,(unsigned int)reference_offset_y0); // GBKWI  
  //pdma[61] = 0; // GCBB
  
  // reference luma group1 bot
  pdma[62] = REG_BITVAL(GSMBA_SMBA,(((unsigned int)addr_y1_reference_bot)>>2)) |
             REG_BITVAL(GSMBA_INCREMENT,3); // GSMBA
  pdma[63] = REG_BITVAL(GCTRL_SLOT_TRANSFER_LEN,8); // GCTRL
  //pdma[64] = 0; // GBKWI
  //pdma[65] = 0; // GCBB
  
  // reference chroma bot
  pdma[66] = REG_BITVAL(GSMBA_SMBA,(((unsigned int)addr_c_reference_bot)>>2)) |
             REG_BITVAL(GSMBA_INCREMENT,2); // GSMBA
  pdma[67] = REG_BITVAL(GCTRL_STYPE,1) |
             REG_BITVAL(GCTRL_SLOT_TRANSFER_LEN,64); // GCTRL
  pdma[68] = REG_BITVAL(GBKWI_SYSMEM_BLOCK_WIDTH,16) |
             REG_BITVAL(GBKWI_SYSMEM_LINE_OFFSET,(unsigned int)reference_offset_c); // GBKWI  
  //pdma[69] = 0; // GCBB
  
  // reconstructed MB luma
  pdma[70] = REG_BITVAL(GSMBA_SMBA,(((unsigned int)addr_y_recon)>>2)) |
             REG_BITVAL(GSMBA_INCREMENT,3); // GSMBA
  pdma[71] = REG_BITVAL(GCTRL_DIR,1) |
             REG_BITVAL(GCTRL_SLOT_TRANSFER_LEN,64); // GCTRL
  pdma[72] = REG_BITVAL(GBKWI_SYSMEM_BLOCK_WIDTH,32) |
             REG_BITVAL(GBKWI_SYSMEM_LINE_OFFSET,(unsigned int)recon_offset_y); // GBKWI  
  //pdma[73] = 0; // GCBB
  
  // reconstructed MB chroma
  pdma[74] = REG_BITVAL(GSMBA_SMBA,(((unsigned int)addr_c_recon)>>2)) |
                       REG_BITVAL(GSMBA_INCREMENT,2); // GSMBA
  pdma[75] = REG_BITVAL(GCTRL_DIR,1) |
                       REG_BITVAL(GCTRL_SLOT_TRANSFER_LEN,32); // GCTRL
  pdma[76] = REG_BITVAL(GBKWI_SYSMEM_BLOCK_WIDTH,16) |
                       REG_BITVAL(GBKWI_SYSMEM_LINE_OFFSET,(unsigned int)recon_offset_c); // GBKWI  
  //pdma[77] = 0; // GCBB
}


void h264_dma_init(h264_encoder *pEnc)
{
  
    h264_dma_set_all(pEnc,0);
    h264_dma_set_all(pEnc,1);
  
}

void* h264_encoder_init(FAVC_ENC_PARAM *pParam)
{
    h264_encoder *pEnc;  
		
    if ((pEnc = (h264_encoder* )nv_malloc(sizeof(h264_encoder),32)) == NULL)     
    {
        printk("malloc H.264 pEnc fail\n");  
        return (void *)pEnc;
    }
    memset(pEnc,0,sizeof(h264_encoder));

    return (void *)pEnc;
}

void h264_encoder_reinit(void *handle,FAVC_ENC_PARAM *pParam,unsigned int mp4yuv)
{
    h264_encoder *pEnc=(h264_encoder *)handle;
    int i;
   	 
    pParam->luma_threshold = 4;
    pParam->chroma_threshold = 4;
    pParam->threshold_disable=0;
  
    // create a copy of param
    //printk("memcpy FAVC_ENC_PARAM\n");
    memcpy(&pEnc->mEncParam,pParam,sizeof(FAVC_ENC_PARAM));
    
//    printk("Set pu8DMABuffer_phy\n");    
  // should be word aligned and virtual address
  pEnc->pdma_buf = (unsigned int*)pEnc->mEncParam.pu8DMABuffer_virt;
//  pEnc->pdma_buf = (unsigned int*)pEnc->mEncParam.pu8DMABuffer_phy;  
//for Non-Cache bit
    // Buggy Here. Why??
  //pEnc->pdma_buf = (unsigned int *)((unsigned int)pEnc->pdma_buf | (unsigned int)CACHE_BIT31); 
  
  pEnc->mp4_2d = mp4yuv;	
  // set the DMA buffer to 0
  //printk("memset pdma_buf at 0x%x\n", pEnc->pdma_buf);    
  

  memset(pEnc->pdma_buf,0,DMA_BUF_SIZE<<1);     // Dual DMA buffer


  //printk("After memset pdma_buf\n");     
  for ( i = 0; i < SUPPORT_MAX_MSLIC_NUM ; i++)
      pEnc->dma_buffer_selector[i] = 0;
      
  //printk("Set pu8SysInfoBuffer\n");     
  pEnc->sys_info_buf = (unsigned char*)pEnc->mEncParam.pu8SysInfoBuffer;
  
  pEnc->mb_width = (pEnc->mEncParam.u32FrameWidth + PIXEL_Y -1)/PIXEL_Y;
  pEnc->mb_height = (pEnc->mEncParam.u32FrameHeight + PIXEL_Y -1)/PIXEL_Y;
  pEnc->frame_luma_size = pEnc->mEncParam.u32FrameWidth * pEnc->mEncParam.u32FrameHeight;
  pEnc->multi_slice = pEnc->mEncParam.multi_slice;
  //pEnc->slice_group_change_cycle = 23;
  pEnc->roi_enable = pParam->bROIEnable;
 
    if(pEnc->roi_enable == 1 ) {
	  unsigned int roi_mb_pos;
	  pEnc->roi_x         = pParam->u32ROIX;
	  pEnc->roi_y         = pParam->u32ROIY;
	  roi_mb_pos = (pEnc->mb_width * (pEnc->roi_y>>4) + (pEnc->roi_x>>4));
	  pEnc->roi_width     = pParam->u32ROIWidth;
	  pEnc->roi_height    = pParam->u32ROIHeight;
      pEnc->roi_mb_width  = (pEnc->roi_width + PIXEL_Y -1) / PIXEL_Y;
      pEnc->roi_mb_height = (pEnc->roi_height + PIXEL_Y -1) / PIXEL_Y;
	  pEnc->roi_offset_y  = roi_mb_pos * SIZE_Y;
	  pEnc->roi_offset_uv = roi_mb_pos * SIZE_U;
	  pEnc->roi_frame_mb_width  = pEnc->mb_width;
	  pEnc->roi_frame_mb_height = pEnc->mb_height;
	    
      // we override the mb_width and mb_height rihgt here for ROI's sake
      pEnc->mb_width = pEnc->roi_mb_width;
	  pEnc->mb_height = pEnc->roi_mb_height;
	  #if 0
	  printk("Driver ROI ( %d, %d) width %d height %d mb width %d height %d\n", 
	  	pEnc->roi_x,
	  	pEnc->roi_y,
	  	pEnc->roi_width,
	  	pEnc->roi_height,
	  	pEnc->mb_width,
	  	pEnc->mb_height);
	  #endif
	  // we override the frame_luma_size rihgt here for ROI's sake
	  pEnc->frame_luma_size = pEnc->roi_width * pEnc->roi_height;
    } else {
      pEnc->roi_x         = 0;
	  pEnc->roi_y         = 0;
	  pEnc->roi_width     = 0;
	  pEnc->roi_height    = 0;
      pEnc->roi_mb_width  = 0;
      pEnc->roi_mb_height = 0;
	  pEnc->roi_offset_y  = 0;
	  pEnc->roi_offset_uv = 0;
	  pEnc->roi_frame_mb_width  = 0;
	  pEnc->roi_frame_mb_height = 0;
    }
  
  pEnc->mb_count = pEnc->mb_width * pEnc->mb_height;
  
  #ifdef WATER_MARK_ENABLE
    pEnc->watermark_frame_number = 0;
    pEnc->watermark_last_checksum = pParam->watermark_init_pattern; 
  #endif
  
  pEnc->iframe = 0;
  pEnc->very_first_flag = 1;
  pEnc->sps.vui_parameters_present_flag = 0;
  // to initialize the DMA buffer
  //printk("Calling h264_dma_init\n");
  h264_dma_init(pEnc);

  return;
}
void h264_encoder_init_crop(void *handle, FAVC_CROP_PARAM *pcrop)
{
  h264_encoder *pEnc=(h264_encoder *)handle;
  crop_param CROP_set;
  memset(&CROP_set, 0, sizeof(crop_param));
  
  CROP_set.left_offset  = pcrop->left_offset;
  CROP_set.right_offset = pcrop->right_offset;
  CROP_set.top_offset   = pcrop->top_offset;
  CROP_set.buttom_offset= pcrop->buttom_offset;

  // user defined end
  pEnc->sps.frame_cropping_flag = 1;
  init_crop_parameters(pEnc, &CROP_set);
  return;

}
void h264_encoder_init_vui(void *handle, FAVC_VUI_PARAM *pVUI)
{
  h264_encoder *pEnc=(h264_encoder *)handle;
  vui_param VUI;
  
  memset(&VUI, 0, sizeof(vui_param));
  VUI.video_format=pVUI->video_format;
  VUI.colour_description_present_flag=pVUI->colour_description_present_flag;
  VUI.colour_primaries=pVUI->colour_primaries;
  VUI.transfer_characteristics=pVUI->transfer_characteristics;
  VUI.matrix_coefficients=pVUI->matrix_coefficients;
  VUI.chroma_location_info_present_flag=pVUI->chroma_location_info_present_flag;
  VUI.chroma_sample_loc_type_top_field=pVUI->chroma_sample_loc_type_top_field;
  VUI.chroma_sample_loc_type_bottom_field=pVUI->chroma_sample_loc_type_bottom_field;
  
  // user defined end
  pEnc->sps.vui_parameters_present_flag = 1;
  init_vui_parameters(pEnc, &VUI);
  return;
}



void h264_reg_init(h264_encoder *pEnc, int idx)
{

    // let's set register here
//    REG_WRITE(PARM0,REG_BITVAL(PARM0_PIC_WIDTH,(pEnc->mb_width-1) ) |
    outp32(REG_264_PARM0,REG_BITVAL(PARM0_PIC_WIDTH,(pEnc->mb_width-1) ) |    
                    REG_BITVAL(PARM0_PIC_HEIGHT,(pEnc->mb_height-1) ) |
                    REG_BITVAL(PARM0_PIC_MB_COUNT,(pEnc->mb_count-1)) );

#ifdef ENABLE_9_INTRA_PREDICTION_MODE
    REG_WRITE(REG_264_PARM1,REG_BITVAL(PARM1_CHROMA_QP_OFFSET,pEnc->mEncParam.chroma_qp_offset ) |
                    REG_BITVAL(PARM1_ALPHA_OFFSET,pEnc->mEncParam.alpha_offset ) |
                    REG_BITVAL(PARM1_BETA_OFFSET,pEnc->mEncParam.beta_offset) |
                    REG_BITVAL(PARM1_LCT,pEnc->mEncParam.luma_threshold) |
                    REG_BITVAL(PARM1_CCT,pEnc->mEncParam.chroma_threshold) |
                    REG_BITVAL(PARM1_TD,pEnc->mEncParam.threshold_disable) |             
                    REG_BITVAL(PARM1_IMODE,1) );
#else
    REG_WRITE(REG_264_PARM1,REG_BITVAL(PARM1_CHROMA_QP_OFFSET,pEnc->mEncParam.chroma_qp_offset ) |
                    REG_BITVAL(PARM1_ALPHA_OFFSET,pEnc->mEncParam.alpha_offset ) |
                    REG_BITVAL(PARM1_BETA_OFFSET,pEnc->mEncParam.beta_offset) |
                    REG_BITVAL(PARM1_LCT,pEnc->mEncParam.luma_threshold) |
                    REG_BITVAL(PARM1_CCT,pEnc->mEncParam.chroma_threshold) |
                    REG_BITVAL(PARM1_TD,pEnc->mEncParam.threshold_disable) |             
                    REG_BITVAL(PARM1_IMODE,0) );
#endif
    
    REG_WRITE(REG_264_PARM4,REG_BITVAL(PARM4_ID,pEnc->mEncParam.disable_ilf ) |
                    REG_BITVAL(PARM4_QP,pEnc->mEncParam.u32Quant ));
    
    // set DMA chain command address
    REG_WRITE(REG_264_DMAA0,REG_BITVAL(DMAA0_SMBA,(((unsigned int)(pEnc->mEncParam.pu8DMABuffer_phy+(pEnc->dma_buffer_selector[idx]*DMA_BUF_SIZE)))>>2)));                    

#ifdef ENC_2DMA
    REG_WRITE(DMAA0_2,REG_BITVAL(DMAA0_2_SMBA,(((unsigned int)(pEnc->mEncParam.pu8DMABuffer_phy+(pEnc->dma_buffer_selector[idx]*DMA_BUF_SIZE)))>>2)));
#endif


    switch (pEnc->mp4_2d) {
    case 0:	// H2642D
        REG_WRITE(REG_264_PARM6,REG_BITVAL(PARM6_MPEG4_2D,0)); // to set the H264 2D mode
        break;

    default: // MPEG42D
        REG_WRITE(REG_264_PARM6,REG_BITVAL(PARM6_MPEG4_2D,1)); // to set the MPEG 2D mode 
        break;
    }


    if(pEnc->very_first_flag) {
#ifdef WATER_MARK_ENABLE
        if(pEnc->mEncParam.watermark_enable)
            REG_WRITE(PARM2,REG_BITVAL(PARM2_WATERMARK_PATTERN,pEnc->mEncParam.watermark_init_pattern)); // initial pattern : 1596450536
#endif

        // set DMA chain command address    
        REG_WRITE(REG_264_DMAC0,REG_BITVAL(DMAC0_TRANSFER_LEN,DMA_BUF_WORD));

        REG_WRITE(REG_264_DMAC1,REG_BITVAL(DMAC1_LENGTH,0x10) |
                    REG_BITVAL(DMAC1_TRANSFER_TYPE,0) |
                    REG_BITVAL(DMAC1_TRANSFER_DIR,1) );
    
        outp32(REG_264_PARM3,0x1fffff );
        outp32(REG_264_PARM5, 0x0);



        REG_WRITE(REG_264_STS2,REG_BITVAL(STS2_FD_INT_STS,0) |
                     REG_BITVAL(STS2_FD_INT_MASK,0) );

        if ( pEnc->first_mb == 0 ) {
            //start_sequence(pEnc);
            init_sps_pps(pEnc);	
            generate_sps_pps(pEnc);
        }
    }
    else {
        if  ((pEnc->mEncParam.ssp_output==1) && ( pEnc->first_mb == 0) ) {
            generate_sps_pps(pEnc);
        } else if (pEnc->mEncParam.ssp_output==0) {
            if ( pEnc->mEncParam.intra < 0) {
                if ( (pEnc->iframe == 0)|| (
                  ((pEnc->mEncParam.u32IPInterval > 0) && (pEnc->iframe >= pEnc->mEncParam.u32IPInterval))&& ( pEnc->first_mb == 0) ) ) {
                    generate_sps_pps(pEnc);
                }
            } else if ( ( pEnc->mEncParam.intra == 1) &&( pEnc->first_mb == 0)) {
                generate_sps_pps(pEnc);
            } 
        } else if (pEnc->mEncParam.ssp_output==-1) {
        // default setting: SPS+PPS output only on IDR frame
        // if very_first_flag is equal to 1, sps+pps will output.
        }
    }
}

int h264_encoder_spspps(void *handle,FAVC_ENC_PARAM *mEncParam)
{
    h264_encoder *pEnc=(h264_encoder *)handle;
    
    pEnc->mEncParam.ssp_output=mEncParam->ssp_output;
    // set bitstream destination address again
    REG_WRITE(REG_264_DMAA1,REG_BITVAL(DMAA1_INCREMENT,1) |
                  REG_BITVAL(DMAA1_SMBA,(((unsigned int)pEnc->mEncParam.pu8BitstreamAddr)>>2)));

#ifdef ENC_2DMA
    REG_WRITE(DMAA1_2,REG_BITVAL(DMAA1_2_INCREMENT,1) |
                  REG_BITVAL(DMAA1_2_SMBA,(((unsigned int)pEnc->mEncParam.pu8BitstreamAddr)>>2)));
#endif
    	
    if(pEnc->very_first_flag) {	
        pEnc->bitstream_length = 0 ;	       		
    } else {		
        if ( pEnc->mEncParam.ssp_output == 1) {	
	    generate_sps_pps(pEnc);
	    
	    pEnc->bitstream_length = REG_READ(REG_264_STS1);	 	 	
        } else {
	    pEnc->bitstream_length = 0 ;	
        }
    }	
	
    	
    return pEnc->bitstream_length;
}

void h264_encoder_setyuv_addr(void *handle,unsigned char *yaddr,unsigned char *uaddr,unsigned char *vaddr)
{
  h264_encoder *pEnc=(h264_encoder *)handle;
  
  pEnc->mEncParam.pu8YFrameBaseAddr = yaddr;  
  pEnc->mEncParam.pu8UFrameBaseAddr = uaddr;
  pEnc->mEncParam.pu8VFrameBaseAddr = vaddr;
  pEnc->mEncParam.pu8UVFrameBaseAddr = uaddr;
  
}
int h264_fmt(void * handle)
{
    h264_encoder *pEnc=(h264_encoder *)handle;
	return pEnc->mp4_2d;
}

void h264_toggle_DMA_buffer(void *handle,int idx)
{
	h264_encoder *pEnc=(h264_encoder *)handle;
	
	pEnc->dma_buffer_selector[idx] ^=1;
	
	return;
}

unsigned int encoder_decide_time_slot(unsigned int width,unsigned int height)
{
	unsigned int tmp;
	
	tmp=((width*height)*FAVC_Encoder_TIMEOUT)/ED1_Size;
	
	if(tmp<FAVC_Encoder_TIMEOUT)
		tmp=FAVC_Encoder_TIMEOUT;
		
	if(tmp>(FAVC_Encoder_TIMEOUT*5))
		tmp=(FAVC_Encoder_TIMEOUT*5);
		
#define _FPGA_		   // enlarge delay for FPGA enviornment
		
#ifdef _FPGA_
	return tmp * 1000;	
#else	
	return tmp;
#endif	
}

int h264_encoder_encode(void *handle,FAVC_ENC_PARAM *mEncParam, int first_mb, int last, int idx)
{
    h264_encoder *pEnc=(h264_encoder *)handle;
    unsigned int encoder_time_slot;

#ifdef EVALUATION_PERFORMANCE
    // encode start timestamp
    timeframe.start = get_counter();
#endif	

    if ( !CHECK_FRAME_DONE()) {
        printk("[Error]: Encode not idle, but doing encode!\n");
        return -1;
    }
	
    pEnc->mEncParam.pu8YFrameBaseAddr=mEncParam->pu8YFrameBaseAddr;
    pEnc->mEncParam.pu8UFrameBaseAddr=mEncParam->pu8UFrameBaseAddr;
    pEnc->mEncParam.pu8VFrameBaseAddr=mEncParam->pu8VFrameBaseAddr;
    pEnc->mEncParam.pu8UVFrameBaseAddr=mEncParam->pu8UVFrameBaseAddr;
    pEnc->mEncParam.u32Quant=mEncParam->u32Quant;
    pEnc->multi_slice = mEncParam->multi_slice;
    
    //printk("%s : Enc Y Src =0x%x, U=0x%x, V=0x%x\n",__FUNCTION__,(unsigned int)mEncParam->pu8YFrameBaseAddr, 
    //                    (unsigned int)mEncParam->pu8UFrameBaseAddr,(unsigned int)mEncParam->pu8VFrameBaseAddr);
	
    if ( pEnc->mEncParam.intra != mEncParam->intra)	
        pEnc->mEncParam.intra = mEncParam->intra;
    if ( pEnc->mEncParam.ssp_output != mEncParam->ssp_output)	
        pEnc->mEncParam.ssp_output = mEncParam->ssp_output;
    if ( ( pEnc->mEncParam.u32IPInterval  != mEncParam->u32IPInterval) && (mEncParam->u32IPInterval != 0 ) )
        pEnc->IPInterval = mEncParam->u32IPInterval;
	
//#if 0	
//    if ( ( pEnc->mEncParam.u32Quant >= pEnc->mEncParam.u32MaxQuant ) 
//       && ((pEnc->mEncParam.u32Quant + pEnc->mEncParam.chroma_qp_offset) < 51 )
//       && (pEnc->mEncParam.chroma_qp_offset < 12 ) )
//        pEnc->mEncParam.chroma_qp_offset++;
//    if ( ( pEnc->mEncParam.u32Quant < pEnc->mEncParam.u32MaxQuant ) 
//       && (pEnc->mEncParam.chroma_qp_offset > 0 ) )
//        pEnc->mEncParam.chroma_qp_offset--;

//  #if 0
//    printk("intra %d interval %d Quant %d\n", 
//	pEnc->mEncParam.intra,
//	pEnc->IPInterval,
//	pEnc->mEncParam.u32Quant);
//  #endif
//#endif
	//printk("%s, multi_slice\n",__FUNCTION__);
    if(pEnc->multi_slice) {   	
        pEnc->mEncParam.u32FrameWidth = mEncParam->u32FrameWidth;
        pEnc->mEncParam.u32FrameHeight = mEncParam->u32FrameHeight;
        pEnc->mb_width = (pEnc->mEncParam.u32FrameWidth + PIXEL_Y -1)/PIXEL_Y;
        pEnc->mb_height = (pEnc->mEncParam.u32FrameHeight + PIXEL_Y -1)/PIXEL_Y;
        pEnc->frame_luma_size = pEnc->mEncParam.u32FrameWidth * pEnc->mEncParam.u32FrameHeight;		
        if(pEnc->roi_enable) {	
            unsigned int roi_mb_pos;
            pEnc->roi_x         = mEncParam->u32ROIX;
            pEnc->roi_y         = mEncParam->u32ROIY;
            roi_mb_pos = (pEnc->mb_width * (pEnc->roi_y>>4) + (pEnc->roi_x>>4));
            pEnc->roi_width     = mEncParam->u32ROIWidth;
            pEnc->roi_height    = mEncParam->u32ROIHeight;
            pEnc->roi_mb_width  = (pEnc->roi_width + PIXEL_Y-1) / PIXEL_Y;
            pEnc->roi_mb_height = (pEnc->roi_height + PIXEL_Y-1) / PIXEL_Y;
            pEnc->roi_offset_y  = roi_mb_pos * SIZE_Y;
            pEnc->roi_offset_uv = roi_mb_pos * SIZE_U;
            pEnc->roi_frame_mb_width  = pEnc->mb_width;
            pEnc->roi_frame_mb_height = pEnc->mb_height;

            // we override the mb_width and mb_height rihgt here for ROI's sake
            pEnc->mb_width = pEnc->roi_mb_width;
            pEnc->mb_height = pEnc->roi_mb_height;
            // we override the frame_luma_size rihgt here for ROI's sake
            pEnc->frame_luma_size = pEnc->roi_width * pEnc->roi_height;
//#if 0
//            printk("DRIVER ROI: ( %d, %d) width %d height %d\n", 
//		pEnc->roi_x, pEnc->roi_y,
//		pEnc->roi_width, pEnc->roi_height);
//#endif
            encoder_time_slot=encoder_decide_time_slot(pEnc->roi_width,pEnc->roi_height);
        }
        else{
            encoder_time_slot=encoder_decide_time_slot(pEnc->mEncParam.u32FrameWidth,pEnc->mEncParam.u32FrameHeight);
        }
		
        pEnc->mb_count = pEnc->mb_width * pEnc->mb_height;
        pEnc->mEncParam.pu8ReConstructFrame =   mEncParam->pu8ReConstructFrame;
        pEnc->mEncParam.pu8ReferenceFrame = mEncParam->pu8ReferenceFrame;
        pEnc->mEncParam.pu8SysInfoBuffer = mEncParam->pu8SysInfoBuffer;
        pEnc->mEncParam.pu8DMABuffer_phy = mEncParam->pu8DMABuffer_phy;
//        pEnc->mEncParam.pu8DMABuffer_virt = mEncParam->pu8DMABuffer_virt;
        pEnc->mEncParam.pu8DMABuffer_virt = (unsigned char *)phys_to_virt_92((unsigned int)pEnc->mEncParam.pu8DMABuffer_phy);
        pEnc->pdma_buf = (unsigned int*)pEnc->mEncParam.pu8DMABuffer_virt;
//        pEnc->pdma_buf = (unsigned int*)pEnc->mEncParam.pu8DMABuffer_phy; 
//for Non-Cache bit
//     pEnc->pdma_buf = (unsigned int *)((unsigned int)pEnc->pdma_buf | (unsigned int)CACHE_BIT31);               
        pEnc->sys_info_buf = (unsigned char*)pEnc->mEncParam.pu8SysInfoBuffer; 	
    } 
    else {
        if(pEnc->roi_enable) {	
            unsigned int roi_mb_pos;
            pEnc->roi_x         = mEncParam->u32ROIX;
            pEnc->roi_y         = mEncParam->u32ROIY;
            roi_mb_pos = (((pEnc->mEncParam.u32FrameWidth + PIXEL_Y -1)/PIXEL_Y) * (pEnc->roi_y>>4) + (pEnc->roi_x>>4));
			
            //pEnc->roi_width     = mEncParam->u32ROIWidth;
            //pEnc->roi_height    = mEncParam->u32ROIHeight;
            //pEnc->roi_mb_width  = (pEnc->roi_width + PIXEL_Y-1) / PIXEL_Y;
            //pEnc->roi_mb_height = (pEnc->roi_height + PIXEL_Y-1) / PIXEL_Y;
            pEnc->roi_offset_y  = roi_mb_pos * SIZE_Y;
            pEnc->roi_offset_uv = roi_mb_pos * SIZE_U;
            //pEnc->roi_frame_mb_width  = pEnc->mb_width;
            //pEnc->roi_frame_mb_height = pEnc->mb_height;

            // we override the mb_width and mb_height rihgt here for ROI's sake
            //pEnc->mb_width = pEnc->roi_mb_width;
            //pEnc->mb_height = pEnc->roi_mb_height;
            // we override the frame_luma_size rihgt here for ROI's sake
            //pEnc->frame_luma_size = pEnc->roi_width * pEnc->roi_height;
#if 0
            printk("DRIVER ROI: ( %d, %d) width %d height %d\n", 
		pEnc->roi_x, pEnc->roi_y,
		pEnc->roi_width, pEnc->roi_height);
#endif
            encoder_time_slot=encoder_decide_time_slot(pEnc->roi_width,pEnc->roi_height);
        }
        else{
            encoder_time_slot=encoder_decide_time_slot(pEnc->mEncParam.u32FrameWidth,pEnc->mEncParam.u32FrameHeight);
        }
    }
    //printk("%s, set first_mb\n",__FUNCTION__);
    pEnc->first_mb = first_mb;
    h264_dma_set_curimg(pEnc,pEnc->dma_buffer_selector[idx]);
    h264_reg_init(pEnc, idx); // out sps + pps

    //SWAP(pEnc->mEncParam.pu8ReConstructFrame,pEnc->mEncParam.pu8ReferenceFrame)
  
    // to write the new quant value
    REG_BITWRITE(REG_264_PARM4, PARM4_QP, pEnc->mEncParam.u32Quant);
  
#ifdef WATER_MARK_ENABLE
    if(pEnc->mEncParam.watermark_enable) {
        if(!pEnc->very_first_flag) // skip the first frame since the initial watermark pattern has already stored during 'h264_encoder_create()' function call 
            REG_WRITE(PARM2,REG_BITVAL(PARM2_WATERMARK_PATTERN,pEnc->watermark_last_checksum));
    }
#endif 
    //printk("%s, slice type\n",__FUNCTION__);
    //pEnc->first_mb = first_mb;
    pEnc->slice_type = slice_type(pEnc);

    //printk("%s, start nalu_header\n",__FUNCTION__);
    start_nalu_header( pEnc); 
    start_slice(pEnc, last);
  
    //printk("%s, Encode BS Buf addr = 0x%x\n",__FUNCTION__,(unsigned int)pEnc->mEncParam.pu8BitstreamAddr);
    // set bitstream destination address again
    REG_WRITE(REG_264_DMAA1,REG_BITVAL(DMAA1_INCREMENT,1) |
	REG_BITVAL(DMAA1_SMBA,(((unsigned int)pEnc->mEncParam.pu8BitstreamAddr)>>2)));

#ifdef ENC_2DMA
    REG_WRITE(DMAA1_2,REG_BITVAL(DMAA1_2_INCREMENT,1) |
	REG_BITVAL(DMAA1_2_SMBA,(((unsigned int)pEnc->mEncParam.pu8BitstreamAddr)>>2)));
#endif

#ifdef WATER_MARK_ENABLE
    {
        unsigned int enable_watermark = (pEnc->mEncParam.watermark_enable) ? (pEnc->watermark_frame_number? 0:1):0;    
        // if the watermark is enabled and the watermark_enable is equal to 2`b11 , then we should force 
        // the bit 23 (PARM1_TD) of register PARM1 to 1 ; otherwise, set it to user specified value
        if(enable_watermark && pEnc->mEncParam.watermark_enable==0x3) {        
            REG_BITWRITE(REG_264_PARM1,PARM1_TD,1);
        } else {
            REG_BITWRITE(REG_264_PARM1,PARM1_TD,pEnc->mEncParam.threshold_disable);
        }
    }
#endif
  
    
#if 1 
#if 0
    DECLARE_WAITQUEUE(wait, current);
    add_wait_queue(&avc_enc_queue, &wait);
    //set_current_state(TASK_INTERRUPTIBLE);
    set_current_state(TASK_UNINTERRUPTIBLE);
#endif

    //printk("clear h264e_int\n");
    h264e_int = 0;
    // to start the encoding
    switch(pEnc->slice_type)
    {
    case I_SLICE :
        //printk("%s, start frame_code_I\n",__FUNCTION__);
        frame_code_I(pEnc);
      	mEncParam->keyframe = pEnc->mEncParam.keyframe = 1;
        //printk("I_SLICE %d\n",pEnc->iframe);
        break;

    case P_SLICE :
        //printk("%s, start frame_code_P\n",__FUNCTION__);    
        frame_code_P(pEnc);
        mEncParam->keyframe = pEnc->mEncParam.keyframe = 0;
        //printk("P_SLICE %d\n",pEnc->iframe);	
        break;
    } //switch

#ifdef EVALUATION_PERFORMANCE
    timeframe.hw_start = get_counter();
#endif	

    if (wait_event_timeout(avc_enc_queue, h264e_int == 1, msecs_to_jiffies(encoder_time_slot)) <= 0) {
        if (h264e_int != 1) {
            printk("h.264 encoder time out\n");      
            reset_encoder(pEnc);
            return -1;
        }
    }	

    
    while(!CHECK_FRAME_DONE())
        printk("<NOT DONE>");

#else 
    // to start the encoding
    switch(pEnc->slice_type) {
    case I_SLICE :
        frame_code_I(pEnc);
        //pFrameStatus->keyframe = 1;
        mEncParam->keyframe = pEnc->mEncParam.keyframe = 1;	  
        break;

    case P_SLICE :
        frame_code_P(pEnc);
        //pFrameStatus->keyframe = 0;
        mEncParam->keyframe = pEnc->mEncParam.keyframe = 0;
        break;
    } //switch
  
    {
        int i=0;
        while(!CHECK_FRAME_DONE())
        {
       
        }
    }
#endif

    // to report the bitstream length
    pEnc->bitstream_length = REG_READ(REG_264_STS1);
   
    // to toggle the DMA buffer selector 
    //pEnc->dma_buffer_selector[idx] ^=1;

    if ( last == 1) {   //KC: last == 1 for last slice encode
        pEnc->frame_num++;
        pEnc->iframe++;
        pEnc->very_first_flag = 0;
    }
    terminate_slice(pEnc);
  
    if ( ( pEnc->slice_type == I_SLICE) 
       && ( pEnc->mEncParam.u32IPInterval  != pEnc->IPInterval) 
       && (pEnc->IPInterval != 0 )  )
        pEnc->mEncParam.u32IPInterval = pEnc->IPInterval;
  
    // to clear bitstream length counter
    CLEAR_BITSTREAM_LENGTH();

  
#ifdef WATER_MARK_ENABLE
    if(pEnc->mEncParam.watermark_enable) {
        // store the last checksum for next frame's watermark pattern
        pEnc->watermark_last_checksum = REG_READ(PARM2);

        //printf("watermark_last_checksum = %d(0x%x)\n",pEnc->watermark_last_checksum,pEnc->watermark_last_checksum);

        pEnc->watermark_frame_number++;
        if(pEnc->mEncParam.watermark_interval > 0)
            pEnc->watermark_frame_number %= pEnc->mEncParam.watermark_interval;
    }
#endif

    // 0x4C , read SATD
    mEncParam->frame_cost = pEnc->mEncParam.frame_cost =REG_READ(REG_264_STS3);

#ifdef EVALUATION_PERFORMANCE
    // encode end timestamp
    do_gettimeofday(&tv_curr);
    timetotal.count++;
    timeframe.stop = timeframe.drv_stop = get_counter();
    performance_count();
    if (  time_delta( &tv_init, &tv_curr) > TIME_INTERVAL ) {
        performance_report();
        performance_reset();
        tv_init.tv_sec = tv_curr.tv_sec;
        tv_init.tv_usec = tv_curr.tv_usec;
    }
#endif

    //printk("Encode Bitstream length = %d\n",pEnc->bitstream_length);
    return pEnc->bitstream_length;
}

// 
// h264_encoder_nvop_nal
// Description: output a NVOP frame for Slice-P by main CPU
// return: return NVOP bitstream length 
// 2007.11.01 TC.Kuo Add
//
#define add_value(v) { *bs++=v; cnt++;}
int h264_encoder_nvop_nal(void *handle,FAVC_ENC_PARAM *mEncParam)
{
    h264_encoder *pEnc=(h264_encoder *)handle;
    unsigned int log2_max_frame_num_minus4;  
    unsigned int log2_max_pic_order_cnt_lsb_minus4;
    signed int slice_qp_delta;
    int mb_skip_run;
    bs_t bs_slice;
    unsigned char *bs;
    unsigned char slice_buf[256];
    int i_count=0, slice_count=0; //,idx=0;
    unsigned int data, cnt=0;
    unsigned int fnum;

    //check last frame slice_type
    if ( mEncParam->nvop_ioctl == 1) { //for FAVC_IOCTL_ENCODE_NVOP
        if ( pEnc->mEncParam.intra == 0) {  
            if ( (pEnc->iframe == 0) ||
              ((pEnc->mEncParam.u32IPInterval > 0)&& (pEnc->iframe >= pEnc->mEncParam.u32IPInterval)) ) {
                pEnc->slice_type = I_SLICE;
            } else {
                pEnc->slice_type = P_SLICE;
            }
        } else if ( pEnc->mEncParam.intra == 1) {
            pEnc->slice_type = I_SLICE;
        } else {
            pEnc->slice_type = P_SLICE;
        }
    }

    // check for FAVC_IOCTL_ENCODE_NVOP and skip_mb_run overflow
    if ( pEnc->slice_type == I_SLICE) 
        return 0;

    //printk("NVOP. P_SLICE %d fr %d\n", pEnc->iframe, pEnc->frame_num);
    bs = mEncParam->pu8BitstreamAddr;
    bs_init( &bs_slice, slice_buf, 256);
    // byte stream NAL unit syntax with emulation_prevention_three_byte disabled
    //U(0, 8); // 8  bits , additional zero byte 'leading_zero_8bits' if it is parameter sets and first slice in picture
    add_value(0);
    //bs_write(&bs, 8, 0);
    //U(1,24); // 24 bits , start code prefix
    //bs_write(&bs, 24, 1);
    add_value(0);
    add_value(0);
    add_value(1);
    
    // start to encode SLICE NALU header with emulation_prevention_three_byte disabled
    if(pEnc->very_first_flag) { 
        //U(((0<<7)|(NALU_PRIORITY_HIGHEST<<5)|NALU_TYPE_IDR),8);  // u(8) , (forbidden_zero_bit | nal_ref_idc | nal_unit_type)
        //bs_write( &bs,  8, (0<<7)|(NALU_PRIORITY_HIGHEST<<5)|NALU_TYPE_IDR);
        add_value((0<<7)|(NALU_PRIORITY_HIGHEST<<5)|NALU_TYPE_IDR);
    } else {  
        //U(((0<<7)|(NALU_PRIORITY_HIGH<<5)|NALU_TYPE_SLICE),8); // u(8) , (forbidden_zero_bit | nal_ref_idc | nal_unit_type)
        //bs_write(&bs, 8, (NALU_PRIORITY_HIGH<<5)|NALU_TYPE_SLICE);
        add_value((0<<7)|(NALU_PRIORITY_HIGHEST<<5)|NALU_TYPE_IDR);
    }

    ///////////////////////////////////////////////////////////////////////////
    // start to encode the slice_header()
    //UE_RBSP(0); // ue(v),"SH: first_mb_in_slice" , since we only have one slice for one picture so we set it to zero
    bs_write_ue(&bs_slice, 0);
    //UE_RBSP(pEnc->slice_type+5); // ue(v),"SH: slice_type" , to add 5 in order to signal that the whole picture has the same slice type
    bs_write_ue(&bs_slice,pEnc->slice_type+5);
    // the original value was obtained from the active_pps->pic_parameter_set_id , but since we 
    // don't support multiple parameter sets, so we set it to zero.
    //UE_RBSP(pEnc->pps.pic_parameter_set_id); // ue(v),"SH: pic_parameter_set_id"
    bs_write_ue(&bs_slice, pEnc->pps.pic_parameter_set_id);
  
    // it was obtained from Log2MaxFrameNum & input->no_frames
    // we set the Log2MaxFrameNum to 0 and the log2_max_frame_num_minus4 will be equal to 0
    //if (input->Log2MaxFrameNum < 4)
    //    log2_max_frame_num_minus4 = MAX((int)(CeilLog2(pEnc->mEncParam.no_frames))-4,0);
    //else 
    //    log2_max_frame_num_minus4 = input->Log2MaxFrameNum - 4;      
    log2_max_frame_num_minus4 = 0; // (4-4)

    //#define IMG_NUMBER (img->number-start_frame_no_in_this_IGOP)
    //img->frame_num = (input->intra_period && input->idr_enable ? IMG_NUMBER % input->intra_period : IMG_NUMBER) % (1 << (log2_max_frame_num_minus4 + 4)); 
    fnum = pEnc->frame_num % (1 << (log2_max_frame_num_minus4 + 4));
    //U_RBSP(pEnc->frame_num,log2_max_frame_num_minus4+4); // u(v),"SH: frame_num"
    bs_write(&bs_slice, log2_max_frame_num_minus4+4, fnum);
  
    if ( mEncParam->nvop_ioctl == 1) { 
        pEnc->frame_num++;
    }

    // if it is IDR  ???
    if(pEnc->very_first_flag) {
        //UE_RBSP(0); // ue(v),"SH: idr_pic_id"
        bs_write_ue(&bs_slice,0);
    }


 
    // reference from X264's code
    //if( h->sps->i_poc_type == 0 )
    //{
    //    h->sh.i_poc_lsb = h->fdec->i_poc & ( (1 << h->sps->i_log2_max_poc_lsb) - 1 );
    //    h->sh.i_delta_poc_bottom = 0;   /* XXX won't work for field */
    //}
    
    //if( sh->sps->i_poc_type == 0 )
    //{
    //    bs_write( s, sh->sps->i_log2_max_poc_lsb, sh->i_poc_lsb );
    //    if( sh->pps->b_pic_order && !sh->b_field_pic )
    //    {
    //        bs_write_se( s, sh->i_delta_poc_bottom );
    //    }
    //}
    log2_max_pic_order_cnt_lsb_minus4 = pEnc->sps.log2_max_pic_order_cnt_lsb_minus4;
    pEnc->toppoc %= (1 << (log2_max_pic_order_cnt_lsb_minus4 + 4));
    //U_RBSP(pEnc->toppoc,log2_max_pic_order_cnt_lsb_minus4+4); // u(v),"SH: pic_order_cnt_lsb"  
    bs_write(&bs_slice, log2_max_pic_order_cnt_lsb_minus4+4, pEnc->toppoc);
    if ( mEncParam->nvop_ioctl == 1) { 
        pEnc->toppoc++; pEnc->toppoc++;
    }	


    if(pEnc->slice_type == P_SLICE) {
        //U_RBSP(0,1); // u(1),"SH: num_ref_idx_active_override_flag"
        bs_write1(&bs_slice, 1);
        // ref_pic_list_reordering
        //U_RBSP(0,1); // u(1),"SH: ref_pic_list_reordering_flag_l0"
        bs_write1(&bs_slice, 1);
    }
    
    // dec_ref_pic_marking
    if(pEnc->very_first_flag) {
        //U_RBSP(0,1); // u(1),"SH: no_output_of_prior_pics_flag"
        bs_write1(&bs_slice, 1); 
        //U_RBSP(0,1); // u(1),"SH: long_term_reference_flag"
        bs_write1(&bs_slice, 1);
    } else {
        //U_RBSP(0,1); // u(1),"SH: adaptive_ref_pic_buffering_flag"
        bs_write1(&bs_slice, 1);
    }
  
    // (currSlice->qp - 26 - active_pps->pic_init_qp_minus26)
    slice_qp_delta = (pEnc->mEncParam.u32Quant-26-0);
    //SE_RBSP(slice_qp_delta); // se(v),"SH: slice_qp_delta"
    bs_write_se( &bs_slice, slice_qp_delta);
    ///////////////////////////////////////////////////////////////////////////////
    // start to encode slice_data() for mb_skip_run
    mb_skip_run = pEnc->mb_width*pEnc->mb_height;
    bs_write_ue(&bs_slice, mb_skip_run);
 
    ///////////////////////////////////////////////////////////////////////////////
    // start to pack rbsp_slice_trailing_bits()
    bs_rbsp_trailing(&bs_slice);
  
    //////////////////////////////////////////////////////////////////////////////
    // start to emulation prevention byte process
    slice_count = bs_slice.p - bs_slice.p_start;
    //bs_init( &bs_slice, slice_buf, 256);
    bs_slice.p = bs_slice.p_start;
    bs_slice.i_left  = 8;
    while( slice_count ) {
        data = bs_read1(&bs_slice);
        slice_count--;		
        if( i_count == 2 && data <= 0x03 ) {
            add_value(0x03);
            i_count = 0;
        }
        if( data == 0 )
            i_count++;
        else
            i_count = 0;
    
        add_value(data);
    }

    /////////////////////////////////////////////////////////////////////////
    pEnc->iframe++;
    if(pEnc->mEncParam.u32IPInterval > 0)
        pEnc->iframe %= pEnc->mEncParam.u32IPInterval;
    pEnc->very_first_flag = 0;
    //printk("nvop length %d\n", cnt);
    return  cnt;
}

void h264_encoder_destroy(void *handle)
{
    h264_encoder *pEnc=(h264_encoder *)handle;

//#ifdef LINUX_ENV
//    pEnc->mEncParam.pfnFree(pEnc);
//#else
    //printk("%s enter to free\n",__FUNCTION__);
	nv_free(pEnc);
//#endif    
    pEnc=0;
}

void reset_encoder(h264_encoder *pEnc)
{

    REG_WRITE(REG_264_CMD0,REG_BITVAL(CMD0_RESET,1)); // to reset entire encoser HW
}

void frame_code_I(h264_encoder *pEnc)
{
//    printk("start to encode I frame\n"); 
//while(1);
#ifdef WATER_MARK_ENABLE
    unsigned int enable_watermark = (pEnc->mEncParam.watermark_enable) ? (pEnc->watermark_frame_number? 0:1):0;    
    REG_WRITE(REG_264_CMD0,REG_BITVAL(CMD0_ENCODE,1) | 
                   REG_BITVAL(CMD0_SLICE_TYPE,0) |
                   REG_BITVAL(CMD0_WATERMARK_ENABLE,(enable_watermark? pEnc->mEncParam.watermark_enable:0) )); // to code I frame with watermark function

#else
//while(1);
    REG_WRITE(REG_264_CMD0,REG_BITVAL(CMD0_ENCODE,1) | REG_BITVAL(CMD0_SLICE_TYPE,0) ); // to code I frame
#endif
}

void frame_code_P(h264_encoder *pEnc)
{
    //printf("start to encode P frame\n");
    //REG_WRITE(CMD0,0x08 | REG_BITVAL(CMD0_SLICE_TYPE,1) );

#ifdef WATER_MARK_ENABLE
    unsigned int enable_watermark = (pEnc->mEncParam.watermark_enable) ? (pEnc->watermark_frame_number? 0:1):0;
    REG_WRITE(CMD0,REG_BITVAL(CMD0_ENCODE,1) |
                   REG_BITVAL(CMD0_SLICE_TYPE,1) |
                   REG_BITVAL(CMD0_WATERMARK_ENABLE,(enable_watermark? pEnc->mEncParam.watermark_enable:0) )); // to code P frame with watermark function
#else
    REG_WRITE(REG_264_CMD0,REG_BITVAL(CMD0_ENCODE,1) | REG_BITVAL(CMD0_SLICE_TYPE,1) ); // to code P frame
#endif
}

unsigned char *h264_get_sysinfo(void *handle)
{
    h264_encoder *pEnc = (h264_encoder *)handle;
    return (unsigned char *) (pEnc->sys_info_buf);
}

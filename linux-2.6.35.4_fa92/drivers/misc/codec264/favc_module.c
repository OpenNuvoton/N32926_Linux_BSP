#include <linux/version.h>
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
#include <linux/interrupt.h>
#include <linux/mm.h>
#include <linux/clk.h>

//#include <linux/bootmem.h>

#include "AVCdec.h"
#include "favc_version.h"
#include "user_define.h"

#include "favc_module.h"

struct clk *clk_264Reg = NULL;

extern unsigned int w55fa92_vde_v,w55fa92_vde_p;

static int TOTAL_VDE_BUF_SIZE=0;

int h264_max_width = 1920, h264_max_height=1072;

DECODER_INFO DECODER_INST[4];
ENCODER_INFO ENCODER_INST[2];

unsigned long _ENCODER_BUF_START,_DECODER_BUF_START;
unsigned long _avc_mem_start_addr = -1, _avc_vir_mem_start_addr=-1,_avc_total_size=-1;

EXPORT_SYMBOL(_avc_mem_start_addr);
EXPORT_SYMBOL(_avc_total_size);
EXPORT_SYMBOL(_avc_vir_mem_start_addr);

#define DRIVER_NAME "w55fa92-avc"
#define	W55FA92_VDE_INT_NUM	33
#define	W55FA92_VEN_INT_NUM 34

module_param(h264_max_width, uint, S_IRUGO|S_IWUSR);
MODULE_PARM_DESC(h264_max_width, "Max Width");

module_param(h264_max_height, uint, S_IRUGO|S_IWUSR);
MODULE_PARM_DESC(h264_max_height, "Max Height");


unsigned int support_decoder = SUPPORT_DECODER_DEFAULT_YES;
unsigned int support_encoder = SUPPORT_ENCODER_DEFAULT_YES;

module_param(support_decoder, uint, S_IRUGO|S_IWUSR);
MODULE_PARM_DESC(support_decoder, "Support Decoder");

module_param(support_encoder, uint, S_IRUGO|S_IWUSR);
MODULE_PARM_DESC(support_encoder, "Support Encoder");


struct semaphore    favc_enc_mutex;
struct semaphore    favc_dec_mutex;


extern wait_queue_head_t avc_dec_queue;
extern spinlock_t avcdec_request_lock;
extern wait_queue_head_t avc_enc_queue;
extern spinlock_t avcenc_request_lock;

unsigned int    dec_bs_phy_buffer=0;
unsigned int    out_phy_buffer=0, out_virt_buffer=0;


int ENCODER_TOTAL_SIZE=0,DECODER_TOTAL_SIZE=0; 

static int irq_dec_allocate=0, irq_enc_allocate=0;

#ifdef LINUX_ENV
irqreturn_t decoder_int_handler(int irq, void *dev_id);
irqreturn_t encoder_int_handler(int irq, void *dev_id);
#else	//LINUX_ENV
int decoder_int_handler(int irq, void *dev_id);
int encoder_int_handler(int irq, void *dev_id);
#endif

#ifdef LINUX_ENV
struct file_operations favc_decoder_fops = {
  	.owner = THIS_MODULE,
	.ioctl = favc_decoder_ioctl,
	.mmap = favc_decoder_mmap,
	.open = favc_decoder_open,
	.release = favc_decoder_release,
};
	
struct miscdevice favc_decoder_dev = {
	MISC_DYNAMIC_MINOR,	
	"w55fa92_264dec",
	&favc_decoder_fops,
};
	
struct file_operations favc_encoder_fops = {
  	.owner = THIS_MODULE,
	.ioctl = favc_encoder_ioctl,
	.mmap = favc_encoder_mmap,
	.open = favc_encoder_open,
	.release = favc_encoder_release,
};
	
struct miscdevice favc_encoder_dev = {
	MISC_DYNAMIC_MINOR,		
	"w55fa92_264enc",
	&favc_encoder_fops,
};
#endif

static int __init w55fa92_avc_alloc_mem (void)
{
    //unsigned int encoder_req_buf_size;
    
    printk("%s : Kernel allocate phy buf addr =0x%x, vir_addr = 0x%x\n", __FUNCTION__, w55fa92_vde_p, w55fa92_vde_v);
    printk("ENCODER_TOTAL_SIZE = 0x%x, DECODER_TOTAL_SIZE = 0x%x\n",ENCODER_TOTAL_SIZE,DECODER_TOTAL_SIZE);        
   
	_ENCODER_BUF_START = w55fa92_vde_p;                     // Phyical Address		
	out_virt_buffer =  w55fa92_vde_v;                       // Virtual address for encoder	
			
        // Encoder and Decoder use individual buffer
	_DECODER_BUF_START = _ENCODER_BUF_START + ENCODER_TOTAL_SIZE;


                                                                // Below is physical address for decoder
	dec_bs_phy_buffer = _DECODER_BUF_START;                 // Decoder Bitstream Buffer Address
	out_phy_buffer = _ENCODER_BUF_START;                    // Encoder Bitstream Buffer Address

	printk("%s,_ENCODER_BUF_START = 0x%x,  _DECODER_BUF_START = 0x%x,\n",__FUNCTION__,
		         (unsigned int)_ENCODER_BUF_START, (unsigned int)_DECODER_BUF_START);
		        
        		        	
		return 0;        	        

}

static int __init w55fa92_avc_init(void)
{
    int ret = 0;
//    unsigned int size, bs_size, mb_info_size,intra_pred_size;
    
    
    M_DEBUG("w55fa92_avc_init\n");

    //------------------------
    // 264 Decoder part
    //------------------------
    if (support_decoder) {
        printk("fa92 AVC Decoder Supported\n");        
   
        if((ret=misc_register(&favc_decoder_dev))<0) {
            M_DEBUG("can't get decoder major number\n");
            return ret;
        }
        else
            
        init_MUTEX(&favc_dec_mutex);

        if((void*)dec_bs_phy_buffer == NULL)    
        {
            printk("Memory dec_bs_phy_buffer allocation error!\n");
            goto fail_irq_d;
        }
      

        spin_lock_init(&avcdec_request_lock);
        printk("FAVC Decoder IRQ mode(%d)v%d.%d\n",W55FA92_VDE_INT_NUM, H264VER_MAJOR, H264VER_MINOR);

        if(request_irq(W55FA92_VDE_INT_NUM, (irq_handler_t)decoder_int_handler, IRQF_DISABLED , DRIVER_NAME, NULL) < 0)        
        {
            printk("Error to allocate dec irq handler!!\n");
            goto fail_irq_d;
        }
        else
            irq_dec_allocate=1;
            
	    if (!request_mem_region((unsigned long)W55FA92_VA_VDE, W55FA92_SZ_VDE, DRIVER_NAME))	
	    {
    		printk("%s: request_mem_region W55FA92_VA_VDE failed!\n", __FUNCTION__);
    		ret = -EBUSY;
	    }    
 
            
        init_waitqueue_head(&avc_dec_queue);
        goto Continue_Encoder;
        
fail_irq_d:

        misc_deregister(&favc_decoder_dev);
        goto Error_Exit;         
    }

Continue_Encoder:

    //------------------------
    // 264 Encoder part
    //------------------------
    if (support_encoder) {    
        
        printk("fa92 AVC Encoder Supported\n");
   
	    if((ret=misc_register(&favc_encoder_dev))<0) {
	        M_DEBUG("can't get decoder major number\n");
	        goto Error_Exit;
	    }
	    init_MUTEX(&favc_enc_mutex);

//Put encoder bitstream memory in system memory.

    	if((void*)out_phy_buffer == NULL)
	    {
	        printk("Memory out_phy_buffer allocation error!\n");
	        goto fail_irq_e;
	    }


	    spin_lock_init(&avcenc_request_lock);
	    printk("FAVC Encoder IRQ mode(%d)v%d.%d\n",W55FA92_VEN_INT_NUM, H264VER_MAJOR, H264VER_MINOR);

	    if(request_irq(W55FA92_VEN_INT_NUM, (irq_handler_t)encoder_int_handler, IRQF_DISABLED, DRIVER_NAME, NULL)<0)
	    {
	        printk("Error to allocate enc irq handler!!\n");
	        goto fail_irq_e;
	    }
	    else
	        irq_enc_allocate = 1;
	    
	    if (!request_mem_region((unsigned long)W55FA92_VA_ENC, W55FA92_SZ_ENC, DRIVER_NAME))	
	    {
    		printk("%s: request_mem_region W55FA92_VA_ENC failed!\n", __FUNCTION__);
    		ret= -EBUSY;
	    }    
	    	    
	    init_waitqueue_head(&avc_enc_queue);
	    
	    goto Continue_Exit;
fail_irq_e:	    
        misc_deregister(&favc_encoder_dev);	 
        goto Error_Exit;   
 
	}

Continue_Exit:

	 clk_264Reg = clk_get(NULL, "vde");
	 clk_enable(clk_264Reg); 

    printk("H264 Driver Version v%d.%d\n",H264VER_MAJOR, H264VER_MINOR);

    return 0;
    
Error_Exit: 
		
    if (ret < 0)
        return ret;

    return -1;
}


static void __exit w55fa92_avc_cleanup(void)
{
    M_DEBUG("w55fa92_avc_cleanup\n");

    // decoder part
    if (support_decoder) {    
        
	    release_mem_region((unsigned long)W55FA92_VA_VDE, W55FA92_SZ_VDE);	        
        misc_deregister(&favc_decoder_dev);

        if (irq_dec_allocate)
        {
            free_irq(W55FA92_VDE_INT_NUM, NULL);
            irq_dec_allocate=0;
        }        

    }


    // encoder part
    if (support_encoder) { 
        
	    release_mem_region((unsigned long)W55FA92_VA_ENC, W55FA92_SZ_ENC);
        misc_deregister(&favc_encoder_dev);

        if (irq_enc_allocate)
        {
            free_irq(W55FA92_VEN_INT_NUM, NULL);
            irq_enc_allocate=0;
        }    
    }

}

unsigned int get_instance_resolution(int encoder, int instance, int *width, int *height)
{
#if defined(CONFIG_ENABLE_DECODER_ONLY) || defined(CONFIG_ENABLE_CODEC_BOTH)    
    if (encoder == 0)
    {
        switch(instance)
        {
            case 0:
#if defined(CONFIG_D1_MAX_RESOLUTION_QVGA)
            *width = 320;
            *height = 240;
#elif defined(CONFIG_D1_MAX_RESOLUTION_WQVGA)
            *width = 480;
            *height = 272; 
#elif defined(CONFIG_D1_MAX_RESOLUTION_VGA)
            *width = 640;
            *height = 480;   
#elif defined(CONFIG_D1_MAX_RESOLUTION_WVGA)
            *width = 800;
            *height = 480;  
#elif defined(CONFIG_D1_MAX_RESOLUTION_SVGA)
            *width = 800;
            *height = 608;  
#elif defined(CONFIG_D1_MAX_RESOLUTION_D1)
            *width = 720;
            *height = 480;   
#elif defined(CONFIG_D1_MAX_RESOLUTION_720P)
            *width = 1280;
            *height = 720; 
#elif defined(CONFIG_D1_MAX_RESOLUTION_1080P)
            *width = 1920;
            *height = 1072;  
#endif             
            break;
            case 1:
#if defined(CONFIG_D2_MAX_RESOLUTION_QVGA)
            *width = 320;
            *height = 240;
#elif defined(CONFIG_D2_MAX_RESOLUTION_WQVGA)
            *width = 480;
            *height = 272; 
#elif defined(CONFIG_D2_MAX_RESOLUTION_VGA)
            *width = 640;
            *height = 480;   
#elif defined(CONFIG_D2_MAX_RESOLUTION_WVGA)
            *width = 800;
            *height = 480;  
#elif defined(CONFIG_D2_MAX_RESOLUTION_SVGA)
            *width = 800;
            *height = 608;  
#elif defined(CONFIG_D2_MAX_RESOLUTION_D1)
            *width = 720;
            *height = 480;   
#elif defined(CONFIG_D2_MAX_RESOLUTION_720P)
            *width = 1280;
            *height = 720; 
#elif defined(CONFIG_D2_MAX_RESOLUTION_1080P)
            *width = 1920;
            *height = 1072;  
#endif              
            break;
            case 2:
#if defined(CONFIG_D3_MAX_RESOLUTION_QVGA)
            *width = 320;
            *height = 240;
#elif defined(CONFIG_D3_MAX_RESOLUTION_WQVGA)
            *width = 480;
            *height = 272; 
#elif defined(CONFIG_D3_MAX_RESOLUTION_VGA)
            *width = 640;
            *height = 480;   
#elif defined(CONFIG_D3_MAX_RESOLUTION_WVGA)
            *width = 800;
            *height = 480;  
#elif defined(CONFIG_D3_MAX_RESOLUTION_SVGA)
            *width = 800;
            *height = 608;  
#elif defined(CONFIG_D3_MAX_RESOLUTION_D1)
            *width = 720;
            *height = 480;   
#elif defined(CONFIG_D3_MAX_RESOLUTION_720P)
            *width = 1280;
            *height = 720; 
#elif defined(CONFIG_D3_MAX_RESOLUTION_1080P)
            *width = 1920;
            *height = 1072;  
#endif              
            break;
            case 3:
#if defined(CONFIG_D4_MAX_RESOLUTION_QVGA)
            *width = 320;
            *height = 240;
#elif defined(CONFIG_D4_MAX_RESOLUTION_WQVGA)
            *width = 480;
            *height = 272; 
#elif defined(CONFIG_D4_MAX_RESOLUTION_VGA)
            *width = 640;
            *height = 480;   
#elif defined(CONFIG_D4_MAX_RESOLUTION_WVGA)
            *width = 800;
            *height = 480;  
#elif defined(CONFIG_D4_MAX_RESOLUTION_SVGA)
            *width = 800;
            *height = 608;  
#elif defined(CONFIG_D4_MAX_RESOLUTION_D1)
            *width = 720;
            *height = 480;   
#elif defined(CONFIG_D4_MAX_RESOLUTION_720P)
            *width = 1280;
            *height = 720; 
#elif defined(CONFIG_D4_MAX_RESOLUTION_1080P)
            *width = 1920;
            *height = 1072;  
#endif              
            break;
            default:
                printk("Unsupported Decode instance\n");             
        }
      
    }
#endif 
    
#if defined(CONFIG_ENABLE_ENCODER_ONLY) || defined(CONFIG_ENABLE_CODEC_BOTH)       
    if (encoder)
    {
        switch (instance)
        {
            case 0:
#if defined(CONFIG_E1_MAX_RESOLUTION_QVGA)
            *width = 320;
            *height = 240;
#elif defined(CONFIG_E1_MAX_RESOLUTION_WQVGA)
            *width = 480;
            *height = 272; 
#elif defined(CONFIG_E1_MAX_RESOLUTION_VGA)
            *width = 640;
            *height = 480;   
#elif defined(CONFIG_E1_MAX_RESOLUTION_WVGA)
            *width = 800;
            *height = 480;  
#elif defined(CONFIG_E1_MAX_RESOLUTION_SVGA)
            *width = 800;
            *height = 608;  
#elif defined(CONFIG_E1_MAX_RESOLUTION_D1)
            *width = 720;
            *height = 480;   
#elif defined(CONFIG_E1_MAX_RESOLUTION_720P)
            *width = 1280;
            *height = 720; 
#elif defined(CONFIG_E1_MAX_RESOLUTION_1080P)
            *width = 1920;
            *height = 1072;  
#endif                 
            break;
            case 1:
#if defined(CONFIG_E2_MAX_RESOLUTION_QVGA)
            *width = 320;
            *height = 240;
#elif defined(CONFIG_E2_MAX_RESOLUTION_WQVGA)
            *width = 480;
            *height = 272; 
#elif defined(CONFIG_E2_MAX_RESOLUTION_VGA)
            *width = 640;
            *height = 480;   
#elif defined(CONFIG_E2_MAX_RESOLUTION_WVGA)
            *width = 800;
            *height = 480;  
#elif defined(CONFIG_E2_MAX_RESOLUTION_SVGA)
            *width = 800;
            *height = 608;  
#elif defined(CONFIG_E2_MAX_RESOLUTION_D1)
            *width = 720;
            *height = 480;   
#elif defined(CONFIG_E2_MAX_RESOLUTION_720P)
            *width = 1280;
            *height = 720; 
#elif defined(CONFIG_E2_MAX_RESOLUTION_1080P)
            *width = 1920;
            *height = 1072;  
#endif            
            break;
            default:
                printk("Unsupported Encode instance\n"); 
        }
          
    }  
#endif        
    return 0;      
    
}

#define OUTPUT_MENUCONFIG_DATA  1

unsigned int get_avc_buffer_size(void)
{
    int i,width,height;
    int bitstream_size, YUV_size;
#if defined(CONFIG_ENABLE_DECODER_ONLY) || defined(CONFIG_ENABLE_CODEC_BOTH)    
    int output_size, intra_size;
#endif    
#if defined(CONFIG_ENABLE_ENCODER_ONLY) || defined(CONFIG_ENABLE_CODEC_BOTH)    
    int sysinfo_size, dma_size;
#endif    
        
    memset(&DECODER_INST, 0, sizeof(DECODER_INST));
            
#if defined(CONFIG_ENABLE_DECODER_ONLY) || defined(CONFIG_ENABLE_CODEC_BOTH)
   
    for(i=0;i<CONFIG_W55FA92_AVC_DECODER_NUM;i++)
    {
        get_instance_resolution(0,i,&width,&height);
        DECODER_INST[i].u32MaxWidth = width;
        DECODER_INST[i].u32MaxHeight = height;  
        
#if OUTPUT_MENUCONFIG_DATA   
        printk("Decoder Instance-%d, width = %d, height = %d\n",i,  width, height);    
#endif
        bitstream_size = DIV_4096(width * height * 3 /2);                   // Decode bitstream buffer size
        YUV_size = DIV_1024(width * height * 3 /2) * MAXIUMUFRAMNUM;        // Reconstruct & Reference Buffer size
        intra_size = ((width + 15) / 16) * 8 * 4;                           // DEC_INTRA_MB_BUF_SIZE
        output_size = DIV_4096(width * height * 2) * 2;                     // Use 2 output buffer for VPE convert and Decoder destination
        
        if (i> 0)
            DECODER_INST[i].u32Offset = DECODER_INST[i-1].u32Offset + DECODER_INST[i-1].u32RequiredSize;
        DECODER_INST[i].u32OutputBufOffset = DIV_4096(bitstream_size + YUV_size + intra_size*2);
        DECODER_INST[i].u32OutputBufSize = output_size;
        DECODER_INST[i].u32DECBufsize = YUV_size;
        DECODER_INST[i].u32INTRAsize = intra_size;
        DECODER_INST[i].u32BSsize = bitstream_size;
        DECODER_INST[i].u32RequiredSize = DIV_4096(bitstream_size + YUV_size + intra_size*2 + output_size);               

#if OUTPUT_MENUCONFIG_DATA   
        printk("Decoder Instance-%d, offset = 0x%x, total buf size = 0x%x\n",i,  DECODER_INST[i].u32Offset, DECODER_INST[i].u32RequiredSize);    
#endif        
        
    }
#endif

    memset(&ENCODER_INST, 0, sizeof(ENCODER_INST));
#if defined(CONFIG_ENABLE_ENCODER_ONLY) || defined(CONFIG_ENABLE_CODEC_BOTH)

    for(i=0;i<CONFIG_W55FA92_AVC_ENCODER_NUM;i++)
    {
        get_instance_resolution(1,i,&width,&height);
        ENCODER_INST[i].u32MaxWidth = width;
        ENCODER_INST[i].u32MaxHeight = height; 
               
#if OUTPUT_MENUCONFIG_DATA   
        printk("Encoder Instance-%d, width = %d, height = %d\n",i,  width, height);    
#endif       
        
        bitstream_size = DIV_4096(width * height * 3 /2);                   // Decode bitstream buffer size
        YUV_size = DIV_4096(width * height * 3 /2);                         // Reconstruct & Reference Buffer size
        sysinfo_size = DIV_16(((width + 15) / 16) * ((height + 15) / 16) * 64);      // used for Intra_pred_size and mb_info_size
        dma_size = ((2+19*4) * 4 * 2) *2;
       
        ENCODER_INST[i].u32BSsize = bitstream_size;
         if (i> 0)
            ENCODER_INST[i].u32Offset = ENCODER_INST[i-1].u32Offset + ENCODER_INST[i-1].u32RequiredSize; 
        ENCODER_INST[i].u32RequiredSize = DIV_4096(bitstream_size + YUV_size * 2 + sysinfo_size + dma_size); 
        
#if OUTPUT_MENUCONFIG_DATA   
        printk("Encoder Instance-%d, offset = 0x%x, total buf size = 0x%x\n",i,  ENCODER_INST[i].u32Offset, ENCODER_INST[i].u32RequiredSize);    
#endif         
    }
#endif

#if defined(CONFIG_ENABLE_ENCODER_ONLY) || defined(CONFIG_ENABLE_CODEC_BOTH)    
    for(i=0;i<CONFIG_W55FA92_AVC_ENCODER_NUM;i++)
    {
        ENCODER_TOTAL_SIZE += ENCODER_INST[i].u32RequiredSize;
    }        
#endif

#if defined(CONFIG_ENABLE_DECODER_ONLY) || defined(CONFIG_ENABLE_CODEC_BOTH)
    for(i=0;i<CONFIG_W55FA92_AVC_DECODER_NUM;i++)
    {
        DECODER_TOTAL_SIZE += DECODER_INST[i].u32RequiredSize;       
    }   
#endif
    
    TOTAL_VDE_BUF_SIZE = ENCODER_TOTAL_SIZE + DECODER_TOTAL_SIZE;
    printk("TOTAL_VDE_BUF_SIZE = 0x%x, dec_total=0x%x, enc_total=0x%x\n",TOTAL_VDE_BUF_SIZE, DECODER_TOTAL_SIZE, ENCODER_TOTAL_SIZE);
    
    return TOTAL_VDE_BUF_SIZE;

}

EXPORT_SYMBOL(get_avc_buffer_size);

EXPORT_SYMBOL(favc_enc_mutex);
EXPORT_SYMBOL(favc_dec_mutex);

console_initcall (w55fa92_avc_alloc_mem);
module_init(w55fa92_avc_init);
module_exit(w55fa92_avc_cleanup);
MODULE_LICENSE("GPL");


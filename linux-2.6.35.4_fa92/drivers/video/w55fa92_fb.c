/* linux/driver/vedio/w55fa92fb.c
 *
 * Copyright (c) 2008 Nuvoton technology corporation
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * Changelog:
 *
 *   2008/12/18     Jhe add this file for nuvoton W55FA92 LCD Controller.
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/tty.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/wait.h>
#include <linux/platform_device.h>
#include <linux/clk.h>

#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/div64.h>
#include <asm/cacheflush.h>
#include <asm/mach/map.h>
#include <mach/regs-clock.h>
#include <mach/w55fa92_reg.h>
#include <mach/fb.h>

#include <asm/param.h> 
#include <linux/timer.h>
#include <mach/w55fa92_osd.h>
#include <mach/w55fa92_fb.h>
#include <mach/DrvEDMA.h>

#ifdef CONFIG_PM
#include <linux/pm.h>
#endif

#undef 	outl 
#undef 	inl
#define outl 	writel
#define inl 	readl
#define EDMA_USE_IRQ  
//#define DUAL_BUF      1             // 1-> Use VDMA for VPOST dual buffer mode, 0-> Use Signle buffer for VPOST


//#define WB_FB_DEBUG
//#define WB_FB_DEBUG_ENTER_LEAVE

#ifdef WB_FB_DEBUG
#define DBG(fmt, arg...)			printk(fmt, ##arg)
#else
#define DBG(fmt, arg...)
#endif

#ifdef WB_FB_DEBUG_ENTER_LEAVE
#define ENTER()					DBG("[%-10s] : Enter\n", __FUNCTION__)
#define LEAVE()					DBG("[%-10s] : Leave\n", __FUNCTION__)
#else
#define ENTER()
#define LEAVE()
#endif

/*
	Allocate Video & OSD memory size, 
	just equal to screen size (LCDWIDTH*LCDHEIGHT*LCDBPP/8)
*/	
unsigned long video_alloc_len;

/* video global mapping variables */
unsigned long video_buf_mmap;
unsigned long video_dma_mmap;
unsigned long video_cpu_mmap;

#ifdef CONFIG_W55FA92_OSD_SUPPORT
unsigned long osd_dma_mmap;
unsigned long osd_cpu_mmap;
unsigned long osd_size;
unsigned long osd_offset;
osd_cmd_t* osd_ptr, osd_block, osd_buffer;
volatile static long g_osd_buf_addr;
volatile static long g_osd_x0;
volatile static long g_osd_fristWrite = 1;
#ifdef CONFIG_RGBx888_FORMAT
#define PIXEL_BSIZE		4
#else
#define PIXEL_BSIZE		2
#endif

#ifdef CONFIG_W55FA92_OSD_SUPPORT
static u32 _osd_mem_v = 0;
static u32 _osd_mem_p = 0;
#endif
#ifdef CONFIG_TWO_OSD_BUF
//static u32 _osd_mem_v = 0;
//static u32 _osd_mem_p = 0;
static u32 _osd_trigger= 0;
static int vdma_ch2 = 0;
static int osd_dirty = 0;

int g_osdunlock_cmd_sent = 0;
int g_osdunlock_cmd_sent_flag = 0;
//#define OPT_COPY_OSD_AND_FB

#endif	// CONFIG_TWO_OSD_BUF
#endif

void w55fa92fb_set_CMR(unsigned int arg);
void w55fa92fb_set_pwm_channel(unsigned int arg);
extern unsigned int g_iopan_cmd_flag;

static DECLARE_WAIT_QUEUE_HEAD(wq);
static DECLARE_WAIT_QUEUE_HEAD(vsyncq);

/* AHB & APB system clock information */
extern unsigned int w55fa92_ahb_clock;
extern unsigned int w55fa92_apb_clock;

extern unsigned int w55fa92_external_clock;

/* FSC */
#if defined(CONFIG_W55FA92_VIDEOIN_DEV1) 
	extern unsigned int bIsVideoInEnable;
#endif
#if defined(CONFIG_W55FA92_VIDEOIN_DEV2)
	extern unsigned int bIsVideoIn2Enable;
#endif
#if defined(CONFIG_W55FA92_VIDEOIN_DEV1) || defined(CONFIG_W55FA92_VIDEOIN_DEV2)
	extern unsigned int w55fa92_VIN_PAC_BUFFER;
#endif

/* PWM duty cycle */
unsigned int g_PWMOutputDuty=0;
unsigned int w55fa92_pwm_channel;

static int vdma_ch = 0;
static u32 _auto_disable = 0;
static int vdma_ch_flag = -1;

#define OPT_MMU_ALLOCATE_FB
static u32 _bg_mem_v = 0;
static u32 _bg_mem_p = 0;

#ifdef OPT_MMU_ALLOCATE_FB
	extern unsigned int w55fa92_fb_mem_v;
	extern unsigned int w55fa92_fb_mem_p;
#endif	

extern unsigned int w55fa92_upll_clock;
extern unsigned int w55fa92_apll_clock;
#ifdef CONFIG_W55FA92_PINGPONG_SUPPORT
int g_fbiopan_cmd_sent = 0;
int g_fbiopan_cmd_cnt = 0;
int g_var_xoffset = 0, g_var_yoffset = 0;
int g_yres;


//#else
//#define OPT_FB_LOCK			// fb lock/unlock 
//static int g_fbunlock_cmd_sent = 0;
#endif

#define OPT_FB_LOCK	
static int g_fbunlock_cmd_sent = 0;
static int g_fb_cmd_flag = 0;		// check if lock/unlock cmd has been sent or not
static volatile int g_fb_vsync_flag = 0;		// check if vsync int encountered or not

int g_map_size;
volatile long g_bg_mmp_p;
volatile long g_bg_mmp_v;
static u32 _tv_system = 0;		// 0: NTSC, 1: PAL

#ifdef CONFIG_W55FA92_TV_LCD_SWITCH
	
	#define TV_QVGA			0
	#define TV_VGA			1	
	#define TV_D1			2		

	#define TV_SHORT_DELAY		30 	// 300 mS
	#define TV_IRQ_NUM 				W55FA92_IRQ(4)  // nIRQ_GPIO2
	#define Enable_IRQ(n)     		outl(1 << (n),REG_AIC_MECR)
	#define Disable_IRQ(n)    		outl(1 << (n),REG_AIC_MDCR)

	static u32 _tv_mem_v = 0;
	static u32 _tv_mem_p = 0;
	static u32 _IsTVmode = 0;			// TV or LCD mode
	
	#ifdef CONFIG_W55FA92_TV_QVGA	
		static u32 _TVmode = TV_QVGA;	// TV QVGA output
		#define TVWIDTH		320
		#define TVHEIGHT	240		
		
	#elif CONFIG_W55FA92_TV_VGA			
		static u32 _TVmode = TV_VGA;	// TV VGA output		
		#define TVWIDTH		640
		#define TVHEIGHT	480		
		
	#elif CONFIG_W55FA92_TV_D1			
		static u32 _TVmode = TV_D1;		// TV VGA output		
		#define TVWIDTH		720
		#define TVHEIGHT	480		
	#endif		
	
	#define TV_BUFF_SIZE	TVWIDTH*TVHEIGHT*2			
	static struct timer_list tv_timer;	// TV/LCM switch debunce
	static void tv_Long_TimeOut(unsigned long data);
	static int tv_detect = 0; 
	static int tv_lcd_update = 0; 	
//	static struct fb_info *tv_info;	

	static int w55fa92fb_ioctl(struct fb_info *info, unsigned int cmd, unsigned long arg);
#endif

static struct clk *clk_hclk4 = NULL;
static struct clk *clk_edma = NULL;
extern void nvt_lock(void);
extern void nvt_unlock(void);

static u32 _dma_len = 0;
static u32 _dma_dst = 0;
static u32 _dma_dst_v = 0;

#define OPT_OSD_FLEXIBLE
#ifdef OPT_OSD_FLEXIBLE
	unsigned long g_osd_size = 0;   	
#endif	
#ifdef CONFIG_W55FA92_TV_LCD_SWITCH

static volatile u32 _vpost_int = LCDCInt_VINT;
#if defined(CONFIG_W55FA92_SYSMGR) || defined (CONFIG_W55FA92_SYSMGR_MODULE)
#include <mach/w55fa92_sysmgr.h>
extern void sysmgr_report(unsigned status);
#endif

//	#define LCDCInt_TVFIELDINT		BIT2				// TV Odd/Even Field Interrupt.	
//	#define LCDCInt_VINT			BIT1				// LCD VSYNC/RD End Interrupt.	

#else
	#if defined(CONFIG_TVOUT_D1_720x480) || defined(CONFIG_TVOUT_VGA_640x480) || defined (CONFIG_TVOUT_QVGA_320x240)
	#define     TV_OUTPUT_DEVICE        1       // Use TVFILED_INT for IRQ (TV Filed Interrupt)
	#else
	#define     TV_OUTPUT_DEVICE        0       // Use VINT for IRQ (LCD VSYNC End interrupt)
	#endif
#endif

#if defined(CONFIG_TVOUT_QVGA_320x240) || defined(CONFIG_TVOUT_VGA_640x480) || defined(CONFIG_TVOUT_D1_720x480)
	#include "w55fa92_TV.c"
#endif

#ifdef CONFIG_W55FA92_OSD_SUPPORT
static int w55fa92_osd_function(osd_cmd_t* osd_ptr);
#endif

#ifdef	CONFIG_GIANTPLUS_GPM1006D0_320X240
#include "w55fa92_GIANTPLUS_GPM1006D0.c"
#endif

#ifdef	CONFIG_TOPPLY_320X240
#include "w55fa92_TOPPLY_320x240.c"
#endif

#ifdef	CONFIG_HANNSTARR_HSD043I9W1_480x272_16B
#include "w55fa92_HannStar_HSD043I9W1_16B.c"
#endif

#ifdef	CONFIG_HANNSTARR_HSD043I9W1_480x272_18B
#include "w55fa92_HannStar_HSD043I9W1_18B.c"
#endif

#ifdef	CONFIG_AMPIRE_800x480_16B
#include "w55fa92_Ampire_800x480_16B.c"
#endif

#ifdef	CONFIG_AMPIRE_800x480_18B
#include "w55fa92_Ampire_800x480_18B.c"
#endif

#ifdef	CONFIG_AMPIRE_800x480_24B
#include "w55fa92_Ampire_800x480_24B.c"
#endif

#ifdef	CONFIG_KD070D10_800x480_18B
#include "w55fa92_KD070D_800x480_18B.c"
#endif

#ifdef	CONFIG_KD070D10_800x480_16B
#include "w55fa92_KD070D_800x480_16B.c"
#endif

#ifdef CONFIG_ILITEK_ILI9341_240x320
#include "w55fa92_ILITEK_ILI9341.c"
#endif

#ifdef CONFIG_FW050TFT_800x480_24B
#include "w55fa92_FW050TFT_800x480_24B.c"
#endif

#if 0
	#ifdef	CONFIG_SHARP_LQ035Q1DH02_320X240
	#include "w55fa92_Sharp_LQ035Q1DH02.c"
	#endif
	
	#ifdef	CONFIG_WINTEK_WMF3324_320X240
	#include "w55fa92_Wintek_WMF3324.c"
	#endif

	#ifdef	CONFIG_AMPIRE_800x600
	#include "w55fa92_Ampire_800x600.c"
	#endif
	
	#ifdef	CONFIG_GOWORLD_GWMTF9360A_320x240
	#include "w55fa92_GOWORLD_GWMTF9360A.c"
	#endif
	
	#ifdef	CONFIG_GOWORLD_GWMTF9615A_480x272
	#include "w55fa92_GOWORLD_GWMTF9615A.c"
	#endif
	
	#ifdef	CONFIG_GOWORLD_GW8973_480x272
	#include "w55fa92_GOWORLD_GW8973.c"
	#endif
	
	#ifdef	CONFIG_VG680_640x480
	#include "w55fa92_VG680.c"
	#endif
#endif

volatile static int osd_hsize = LCDWIDTH;
volatile static int osd_vsize = LCDHEIGHT;

scale_cmd_t* scale_ptr, scale_buf;

	#define VA_RGB555	0x00	
	#define VA_RGB565	0x01		
	#define VA_RGBx888	0x02		
	#define VA_RGB888x	0x03				
	#define VA_CbYCrY	0x04					
	#define VA_YCbYCr	0x05						
	#define VA_CrYCbY	0x06					
	#define VA_YCrYCb	0x07						

#ifdef CONFIG_W55FA92_TV_LCD_SWITCH

	#define OSD_2_VA	0
	#define VA_2_OSD	1
	
	#define OSD_RGB555	0x08	
	#define OSD_RGB565	0x09		
	#define OSD_RGBx888	0x0A		
	#define OSD_RGB888x	0x0B				
	#define OSD_CbYCrY	0x04					
	#define OSD_YCbYCr	0x05						
	#define OSD_CrYCbY	0x06					
	#define OSD_YCrYCb	0x07						

	u8 osd_format[]= {0,0,0,0,VA_CbYCrY,VA_YCbYCr,VA_CrYCbY,VA_YCrYCb,VA_RGB555,VA_RGB565,VA_RGBx888,VA_RGB888x}; 
	u8 va_format[]= {OSD_RGB555,OSD_RGB565,OSD_RGBx888,OSD_RGB888x,OSD_CbYCrY,OSD_YCbYCr,OSD_CrYCbY,OSD_YCrYCb};
#endif

	u8 edma_format[]= {eDRVEDMA_RGB555,eDRVEDMA_RGB565,eDRVEDMA_RGB888,0,eDRVEDMA_YCbCr422,eDRVEDMA_YCbCr422,eDRVEDMA_YCbCr422,eDRVEDMA_YCbCr422};

static int changeMode = 0;
static int currentMode = VA_RGB565;
static int destMode = VA_RGB565;

#ifdef CONFIG_W55FA92_PINGPONG_SUPPORT
	static int w55fa92fb_pan_display(struct fb_var_screeninfo *var, struct fb_info *info)
	{
		if (g_iopan_cmd_flag)
		{
			g_iopan_cmd_flag = 0;
			g_fbiopan_cmd_sent = 1;
		}
		
	    g_var_xoffset = var->xoffset;
	    g_var_yoffset = var->yoffset;
		return 0;
	}
#endif	
u32 _edma_test = 0;	

#ifdef EDMA_USE_IRQ
void fb_edma_irq_handler(unsigned int arg)
{
//	printk("fb_edma_irq_handler\n");
	w55fa92_edma_free(vdma_ch);  
	vdma_ch_flag = -1;
	if (changeMode==2)			
	{
		changeMode = 3;
		// disable VDMA color space transform                    
      	w55fa92_edma_clear_cst(vdma_ch);                    
		currentMode = destMode;  
		if (_auto_disable)
		{
	#ifdef CONFIG_W55FA92_TV_LCD_SWITCH
			outl((inl(REG_LCM_LCDCInt) & 0xFFFFFF00) | (_vpost_int << 16), REG_LCM_LCDCInt); 
	#else			
		#if TV_OUTPUT_DEVICE
			outl((inl(REG_LCM_LCDCInt) & 0xFFFFFF00) | 0x40000, REG_LCM_LCDCInt);
		#else
			outl((inl(REG_LCM_LCDCInt) & 0xFFFFFF00) | 0x20000, REG_LCM_LCDCInt);
		#endif
	#endif
		}		
	}       	
#ifdef OPT_FB_LOCK	
	if(g_fbunlock_cmd_sent==2)
	{					
		g_fbunlock_cmd_sent = 0;
	}		
#endif		

#ifdef CONFIG_TWO_OSD_BUF
	if (g_osdunlock_cmd_sent==2)
	{
		g_osdunlock_cmd_sent = 0;
	}			
#endif	
	
}

// emda transfe abort is encountered
void fb_edma_abort_irq_handler(unsigned int arg)	
{
	w55fa92_edma_free(vdma_ch);  
	if (changeMode==2)			
	{
		changeMode = 3;
      	w55fa92_edma_clear_cst(vdma_ch);                    
		currentMode = destMode;  
		if (_auto_disable)
		{
	#ifdef CONFIG_W55FA92_TV_LCD_SWITCH
			outl((inl(REG_LCM_LCDCInt) & 0xFFFFFF00) | (_vpost_int << 16), REG_LCM_LCDCInt); 
	#else			
		#if TV_OUTPUT_DEVICE
			outl((inl(REG_LCM_LCDCInt) & 0xFFFFFF00) | 0x40000, REG_LCM_LCDCInt);
		#else
			outl((inl(REG_LCM_LCDCInt) & 0xFFFFFF00) | 0x20000, REG_LCM_LCDCInt);
		#endif
	#endif
		}		
	}       	
}
#endif


#ifdef CONFIG_W55FA92_TV_LCD_SWITCH

	static void change_format(int path)
	{
		int format;

		DBG("before foramt change !!! \n");										
		DBG("REG_LCM_OSD_CTL = 0x%x !!! \n", inl(REG_LCM_OSD_CTL));										
		DBG("REG_LCM_LCDCCtl = 0x%x !!! \n", inl(REG_LCM_LCDCCtl));										
				
		switch(path)
		{
			case OSD_2_VA:
				format = (inl(REG_LCM_OSD_CTL) & OSD_CTL_OSD_FSEL) >> 24;
				format = osd_format[format];
				outl((inl(REG_LCM_LCDCCtl) & ~LCDCCtl_FBDS) | (format<<1), REG_LCM_LCDCCtl);  		// copy OSD foramt to VA
				break;
				
			case VA_2_OSD:
			default:
				format = (inl(REG_LCM_LCDCCtl) & LCDCCtl_FBDS) >> 1;
				format = va_format[format];
				outl((inl(REG_LCM_OSD_CTL) & ~OSD_CTL_OSD_FSEL) | (format<<24) , REG_LCM_OSD_CTL);	// copy VA foramt to OSD
				break;
		}			
		DBG("after foramt change !!! \n");										
		DBG("REG_LCM_OSD_CTL = 0x%x !!! \n", inl(REG_LCM_OSD_CTL));										
		DBG("REG_LCM_LCDCCtl = 0x%x !!! \n", inl(REG_LCM_LCDCCtl));										
	}		

	static void display_to_lcd(void)
	{
			unsigned int reg[0x30];
			int ii;
			
			// read out registers
			for (ii=0; ii<0x21; ii++)
			{
				reg[ii] = inl(REG_LCM_LCDCCtl+ii*4);
			}				
			
			outl(inl(REG_AHBIPRST) | VPOST_RST, REG_AHBIPRST);
			outl(inl(REG_AHBIPRST) & ~VPOST_RST, REG_AHBIPRST);	

			// write in registers
			for (ii=0x20; ii>=0; ii--)			
			{
				outl(reg[ii], REG_LCM_LCDCCtl+ii*4);
			}				

			// TV output disable
		   	outl(inl(REG_LCM_LCDCPrm)& ~LCDCPrm_LCDSynTv, REG_LCM_LCDCPrm);		// async with TV
		   	outl((inl(REG_LCM_TVCtl)& 0xFFFFF0DA)|(0x00000410), REG_LCM_TVCtl); // TV DAC disable	

			_vpost_int = LCDCInt_VINT;
			outl((inl(REG_LCM_LCDCInt) & 0xFFF0FF00) | (_vpost_int << 16), REG_LCM_LCDCInt); // enable VIN	
			outl((inl(REG_LCM_LCDCCtl) | LCDCCtl_LCDRUN), REG_LCM_LCDCCtl); 	// LCD ON						

			/*set frambuffer start phy addr*/
			outl(video_dma_mmap, REG_LCM_FSADDR);
			
	#ifndef CONFIG_W55FA92_TV_QVGA

			// disable OSD
			outl(inl(REG_LCM_OSD_CTL) & ~OSD_CTL_OSD_EN , REG_LCM_OSD_CTL);   // OSD disabled
	
			change_format(OSD_2_VA);
	#endif

			switch_vpost_clcok_for_lcm();
			
			// enable LCD controller
			outl(inl(REG_LCM_LCDCCtl) | LCDCCtl_LCDRUN, REG_LCM_LCDCCtl);  // enable LCD run
	}			

	static void display_to_tv(void)
	{
			unsigned int clock_div;			
			
			_vpost_int = LCDCInt_TVFIELDINT;
			outl(inl(REG_LCM_LCDCInt) & 0x800000, REG_LCM_LCDCInt); // clear register under run
			outl((inl(REG_LCM_LCDCInt) & 0xFFF0FF00) | (_vpost_int << 16), REG_LCM_LCDCInt); // enable VIN	

	#ifndef CONFIG_W55FA92_TV_QVGA
			/*set frambuffer start phy addr*/
			outl(_tv_mem_p, REG_LCM_FSADDR);
			outl(osd_dma_mmap, REG_LCM_OSD_ADDR);

			// enable OSD function
			change_format(VA_2_OSD);
			
#ifdef CONFIG_RGBx888_FORMAT
			outl((inl(REG_LCM_LCDCCtl) & ~LCDCCtl_FBDS) | (0x02<<1) , REG_LCM_LCDCCtl);			// set VA format to RGBx888			
#else			
			outl((inl(REG_LCM_LCDCCtl) & ~LCDCCtl_FBDS) | (0x01<<1) , REG_LCM_LCDCCtl);			// set VA format to RGB565			
#endif			
	
			outl( inl(REG_LCM_LINE_STRIPE) & LINE_STRIPE_F1_LSL , REG_LCM_LINE_STRIPE);			// reset OSD line stripe

			outl((LCDWIDTH-1)+((LCDHEIGHT-1) << 16) , REG_LCM_OSD_SIZE);		// set OSD size
			outl((TVWIDTH-LCDWIDTH)/2+(((TVHEIGHT-LCDHEIGHT)/2) << 16) , REG_LCM_OSD_SP);// 1st start position					
			outl((LCDWIDTH+(TVWIDTH-LCDWIDTH)/2)+((LCDHEIGHT +((TVHEIGHT-LCDHEIGHT)/2)-1) << 16) , REG_LCM_OSD_BEP);	// 1st end position
			outl(0x00000000, REG_LCM_OSD_BO);		// offset between 1st and 2nd bar
			outl(inl(REG_LCM_OSD_CTL) | OSD_CTL_OSD_EN , REG_LCM_OSD_CTL);  	// OSD enabled
			outl(inl(REG_LCM_OSD_CTL) &  ~(0x01 << 28), REG_LCM_OSD_CTL);   	// disable OSD color-key							
	#endif			
			
			// enable TV function 
		   	outl(inl(REG_LCM_LCDCPrm) | LCDCPrm_LCDSynTv, REG_LCM_LCDCPrm);		// sync with TV
			outl((inl(REG_LCM_LCDCCtl) & ~LCDCCtl_LCDRUN), REG_LCM_LCDCCtl); 	// LCD off
		  	outl((inl(REG_LCM_TVCtl)& ~(TVCtl_LCDSrc+TVCtl_LCDSrc))|(0x00000500), REG_LCM_TVCtl);  //Frame Buffer Size:640x480; 
	    	outl((inl(REG_LCM_TVCtl)& 0xFFFFF0C0)|(0x00000509), REG_LCM_TVCtl);	// DAC enabled
	    	outl(inl(REG_LCM_TVCtl)|(_tv_system << 2), REG_LCM_TVCtl);	// select TV system 

	    	if (_TVmode == TV_QVGA)
	    	{
	    		outl((inl(REG_LCM_TVCtl)& ~TVCtl_TvInter), REG_LCM_TVCtl);				// non-interlace mode is selected	    			    		
	    		outl((inl(REG_LCM_TVCtl)& ~TVCtl_FBSIZE)|(0x00000000), REG_LCM_TVCtl);	// QVGA size is selected	    		
			}
			else if (_TVmode == TV_D1)	    					
			{
	    		outl((inl(REG_LCM_TVCtl)& ~TVCtl_FBSIZE)|(0x0000C000), REG_LCM_TVCtl);	// D1 size is selected
			}
			else 
			{
	    		outl((inl(REG_LCM_TVCtl)& ~TVCtl_FBSIZE)|(0x00008000), REG_LCM_TVCtl);	// VGA size is selected				
			}				
	    	
			nvt_lock();
    	
			// set VPOST clock for TV
	#if !defined(CONFIG_W55FA92_TV_FROM_APLL)			
			clock_div = w55fa92_upll_clock / 27000;
			clock_div /= 2;
			clock_div &= 0xFF;
			clock_div --;
			
			outl((inl(REG_CLKDIV1) & ~VPOST_S) | (3<<3), REG_CLKDIV1);		// VPOST clock from UPLL			
			outl((inl(REG_CLKDIV1) & ~VPOST_N0) | 1, REG_CLKDIV1);					// divider 2 in VPOST_N0
			outl((inl(REG_CLKDIV1) & ~VPOST_N1) | (clock_div << 8), REG_CLKDIV1);	
	#else
			w55fa92_set_apll_clock(135000);		
			clock_div = w55fa92_apll_clock / 27000;
			clock_div &= 0xFF;
			clock_div --;
			
			outl((inl(REG_CLKDIV1) & ~VPOST_S) | (2<<3), REG_CLKDIV1);		// VPOST clock from APLL			
			outl((inl(REG_CLKDIV1) & ~VPOST_N0), REG_CLKDIV1);				
			outl((inl(REG_CLKDIV1) & ~VPOST_N1) | (clock_div << 8), REG_CLKDIV1);	
	#endif			
			nvt_unlock();	
	}			


	static void tv_Short_TimeOut(unsigned long data)
	{
		int pin_status;

//		ENTER();

#ifdef CONFIG_TV_DETECT_GPD3	
		pin_status = inl(REG_GPIOD_PIN) & 0x0008;
#endif
		
		if (!pin_status)	// TV plug_out
		{
	       	_IsTVmode = 0;
			display_to_lcd();
	       	w55fa92fb_ioctl(0, VIDEO_DISPLAY_ON, 00);			// backlight is ON
#if defined(CONFIG_W55FA92_SYSMGR) || defined (CONFIG_W55FA92_SYSMGR_MODULE)	       	
            sysmgr_report(SYSMGR_STATUS_DISPLAY_LCD);			
#endif            
			printk("TV plug out !!!\n");			
		}		
		else								// TV plug_in		
		{
	       	_IsTVmode = 1;
			display_to_tv();
	       	w55fa92fb_ioctl(0, VIDEO_DISPLAY_OFF, 00);		// backlight is OFF
#if defined(CONFIG_W55FA92_SYSMGR) || defined (CONFIG_W55FA92_SYSMGR_MODULE)	       	
            sysmgr_report(SYSMGR_STATUS_DISPLAY_TV);						
#endif            
			printk("TV plug in !!!\n");			
		}			

      	Enable_IRQ(TV_IRQ_NUM);			        
//		LEAVE();      	
	}
	
	static irqreturn_t tv_detect_irq(int irq, void *dev_id, struct pt_regs *regs)
	{
//			ENTER();	

	      	Disable_IRQ(TV_IRQ_NUM);		
	
	#ifdef CONFIG_TV_DETECT_GPD3
	        src = inl(REG_IRQTGSRC1);
	        outl(src & 0x00080000, REG_IRQTGSRC1);
            del_timer(&tv_timer);
            tv_timer.data = 0UL;
            tv_timer.expires = jiffies +  TV_SHORT_DELAY;
            tv_timer.function = tv_Short_TimeOut;
            add_timer(&tv_timer);
	
	#endif				
				
//		LEAVE();      	
	        return IRQ_HANDLED;
	}
#endif	// CONFIG_W55FA92_TV_LCD_SWITCH

#ifdef CONFIG_W55FA92_OSD_SUPPORT
static int w55fa92_osd_function(osd_cmd_t* osd_ptr)
{
	int ii,jj;

#ifdef CONFIG_RGBx888_FORMAT	
	int* ptr;
#else
	short* ptr;
#endif
	volatile int osd_addr;	

	ENTER();
	
	int osd_format;
	
	osd_format = osd_ptr->format;
//	printk("osd_format = 0x%x !!!\n", osd_format);
	
	if ( (osd_format&0xFF00) != 0xAB00)		// for backward compatible old frame buffer driver
		osd_format = OSD_RGB565 & 0x00FF;
	else 
		osd_format &= 0x00FF;			
		
	switch(osd_ptr->cmd)
	{
		case OSD_Close:
			outl(inl(REG_LCM_OSD_CTL) & ~OSD_CTL_OSD_EN , REG_LCM_OSD_CTL);   // OSD disabled
		
			break;
			
		case OSD_Open:
			outl((inl(REG_LCM_OSD_CTL) & ~OSD_CTL_OSD_FSEL)|(osd_format << 24) , REG_LCM_OSD_CTL);	// RGB565 format is the default
			outl( inl(REG_LCM_LINE_STRIPE) & LINE_STRIPE_F1_LSL , REG_LCM_LINE_STRIPE);			// reset OSD line stripe
			
			// set OSD size, position
			outl((osd_hsize-1)+((osd_vsize-1) << 16) , REG_LCM_OSD_SIZE);		
			outl(0x00000000 , REG_LCM_OSD_SP);		// 1st start position			
			outl((osd_hsize)+((osd_vsize) << 16) , REG_LCM_OSD_BEP);		// 1st end position
			outl(0x00000000, REG_LCM_OSD_BO);		// offset between 1st and 2nd bar
			break;
		
		case OSD_Show:
			outl(inl(REG_LCM_OSD_CTL) | OSD_CTL_OSD_EN , REG_LCM_OSD_CTL);   // OSD enabled
			break;
		
		case OSD_Hide:
			outl(inl(REG_LCM_OSD_CTL) & ~OSD_CTL_OSD_EN , REG_LCM_OSD_CTL);   // OSD disabled		
			break;
		
		case OSD_Clear:
			// fill assigned block size by color-key color
	#ifdef CONFIG_RGBx888_FORMAT
			ptr = (int*)osd_cpu_mmap;
			for (ii=0; ii<osd_size/4; ii++)	// osd_size is the allocated OSD buffer for display
				*(ptr++) = inl(REG_LCM_OSD_CTL) & 0x00FFFFFF;	//xRGB888
	#else
			ptr = (short*)osd_cpu_mmap;
			for (ii=0; ii<osd_size/2; ii++)	// osd_size is the allocated OSD buffer for display
				*(ptr++) = inl(REG_LCM_OSD_CTL) & 0xFFFF;	//RGB565
	#endif				
				
			break;
		
		case OSD_Fill:
			// fill assigned block size by assigned color
			if ( (osd_ptr->x0 + osd_ptr->x0_size) > osd_hsize)	// OSD x-axis > limit
				return -2;
			
			if ( (osd_ptr->y0 + osd_ptr->y0_size) > osd_vsize)	// OSD y-axis > limit
				return -2;
	
	#ifdef CONFIG_RGBx888_FORMAT
			ptr = (int*)osd_cpu_mmap;			
			ptr += osd_ptr->y0 * LCDWIDTH + osd_ptr->x0;
			for(ii=0; ii<osd_ptr->y0_size; ii++)
			{
				for (jj=0; jj<osd_ptr->x0_size; jj++)
				{
					*(ptr+jj) = osd_ptr->color;
				}				
				ptr += LCDWIDTH;			
			}
	#else			
			osd_addr = osd_cpu_mmap;	
			osd_addr += (osd_ptr->y0 * LCDWIDTH + osd_ptr->x0)* 2;			
			ptr = (short*)osd_addr;			
			for(ii=0; ii<osd_ptr->y0_size; ii++)
			{
				for (jj=0; jj<osd_ptr->x0_size; jj++)
				{
					*(ptr+jj) = osd_ptr->color & 0xFFFF;		//RGB565					
				}				
				ptr += LCDWIDTH;			
			}
	#endif			
			break;

		case OSD_FillBlock:
			if ( (osd_ptr->y0 + osd_ptr->y0_size) > osd_vsize)	// OSD y-axis > limit
				return -2;
	
			if ( (osd_ptr->x0 + osd_ptr->x0_size) > osd_hsize)	// OSD x-axis > limit
				return -2;
			
			osd_block = *osd_ptr;
			g_osd_fristWrite = 1;
			break;

  		case OSD_SetTrans:
			outl((inl(REG_LCM_OSD_CTL) & ~OSD_CTL_OSD_TC) |  (osd_ptr->color & 0xFFFFFF), REG_LCM_OSD_CTL);   // OSD transparent color
			outl(inl(REG_LCM_OSD_CTL) |  (0x01 << 28), REG_LCM_OSD_CTL);   // enable OSD color-key
  			break;

  		case OSD_ClrTrans:
			outl(inl(REG_LCM_OSD_CTL) &  ~BIT28, REG_LCM_OSD_CTL);   // disable OSD color-key
  			break;

  		case OSD_SetBlend:
			outl((inl(REG_LCM_OSD_ALPHA) & ~OSD_ALPHA) | (osd_ptr->alpha & 0xFF), REG_LCM_OSD_ALPHA);	// given weighting value
			outl(inl(REG_LCM_OSD_ALPHA) | OSD_ALPHA_EN, REG_LCM_OSD_ALPHA);   // enable OSD alpha blending
  			break;

  		case OSD_ClrBlend:
			outl(inl(REG_LCM_OSD_ALPHA) & ~OSD_ALPHA_EN, REG_LCM_OSD_ALPHA);   // disable OSD alpha blending
  			break;
			
		default:
			break;						
	}		
	LEAVE();
	return 0;
	
}
#endif

static void w55fa92fb_set_lcdaddr(struct w55fa92fb_info *fbi)
{
	ENTER();
	
	/*set frambuffer start phy addr*/
	outl(video_dma_mmap, REG_LCM_FSADDR);
	
#ifdef CONFIG_W55FA92_OSD_SUPPORT
	outl(osd_dma_mmap, REG_LCM_OSD_ADDR);
#endif	

	LEAVE();
}


static int w55fa92fb_check_var(struct fb_var_screeninfo *var,
			       struct fb_info *info)
{
	struct w55fa92fb_info *fbi = info->par;
	
	printk("check_var(var=%p, info=%p)\n", var, info);
	/* validate x/y resolution */
	if (var->yres > fbi->mach_info->yres.max)
		var->yres = fbi->mach_info->yres.max;
	else if (var->yres < fbi->mach_info->yres.min)
		var->yres = fbi->mach_info->yres.min;

	if (var->xres > fbi->mach_info->xres.max)
		var->xres = fbi->mach_info->xres.max;
	else if (var->xres < fbi->mach_info->xres.min)
		var->xres = fbi->mach_info->xres.min;

	/* validate bpp */
	if (var->bits_per_pixel > fbi->mach_info->bpp.max)
		var->bits_per_pixel = fbi->mach_info->bpp.max;
	else if (var->bits_per_pixel < fbi->mach_info->bpp.min)
		var->bits_per_pixel = fbi->mach_info->bpp.min;

#ifdef CONFIG_RGBx888_FORMAT
	/* set r/g/b positions */
	if (var->bits_per_pixel == 32) {
		var->red.offset			= 16;
		var->green.offset		= 8;
		var->blue.offset		= 0;
		var->red.length			= 8;
		var->green.length		= 8;
		var->blue.length		= 8;
		var->transp.length	= 0;
	} 
#else
	/* set r/g/b positions */
	if (var->bits_per_pixel == 16) {
		var->red.offset			= 11;
		var->green.offset		= 5;
		var->blue.offset		= 0;
		var->red.length			= 5;
		var->green.length		= 6;
		var->blue.length		= 5;
		var->transp.length	= 0;
	} 
#endif	
	else 
	{
		var->red.length			= var->bits_per_pixel;
		var->red.offset			= 0;
		var->green.length		= var->bits_per_pixel;
		var->green.offset		= 0;
		var->blue.length		= var->bits_per_pixel;
		var->blue.offset		= 0;
		var->transp.length	= 0;
	}
	
	if ((var->yres_virtual > fbi->mach_info->yres.defval*2) 
	||  (var->xres_virtual > fbi->mach_info->xres.defval))
		return -EINVAL;
			
	return 0;
}

static void w55fa92fb_activate_var(struct w55fa92fb_info *fbi,
				   struct fb_var_screeninfo *var)
{
}

static int w55fa92fb_set_par(struct fb_info *info)
{
	struct w55fa92fb_info *fbi = info->par;
	struct fb_var_screeninfo *var = &info->var;

	printk("w55fa92fb_set_par !!! ================= \n");	
#ifdef CONFIG_RGBx888_FORMAT
	if (var->bits_per_pixel == 32)
#else	
	if (var->bits_per_pixel == 16)
#endif	
		fbi->fb->fix.visual = FB_VISUAL_TRUECOLOR;
	else
		fbi->fb->fix.visual = FB_VISUAL_PSEUDOCOLOR;

	printk("var->width = 0x%x !!!\n", var->width);	
	printk("var->bits_per_pixel = 0x%x !!!\n", var->bits_per_pixel);		
	
	fbi->fb->fix.line_length     = (var->width*var->bits_per_pixel)/8;

	/* activate this new configuration */
	w55fa92fb_activate_var(fbi, var);
	return 0;
}	

/* from pxafb.c */
static inline unsigned int chan_to_field(unsigned int chan, struct fb_bitfield *bf)
{
	chan &= 0xffff;
	chan >>= 16 - bf->length;
	return chan << bf->offset;
}

static int w55fa92fb_setcolreg(unsigned regno,
			       unsigned red, unsigned green, unsigned blue,
			       unsigned transp, struct fb_info *info)
{
	struct w55fa92fb_info *fbi = info->par;
	unsigned int val;
	switch (fbi->fb->fix.visual) {
	case FB_VISUAL_TRUECOLOR:
		/* true-colour, use pseuo-palette */
		if (regno < 16) {
			u32 *pal = fbi->fb->pseudo_palette;

			val  = chan_to_field(red,   &fbi->fb->var.red);
			val |= chan_to_field(green, &fbi->fb->var.green);
			val |= chan_to_field(blue,  &fbi->fb->var.blue);

			pal[regno] = val;
		}
		break;

	case FB_VISUAL_PSEUDOCOLOR:
		if (regno < 256) {
			/* currently assume RGB 5-6-5 mode */
			val  = ((red   >>  0) & 0xf800);
			val |= ((green >>  5) & 0x07e0);
			val |= ((blue  >> 11) & 0x001f);
		}
		break;

	default:
		return 1;   /* unknown type */
	}
	return 0;
}

static int w55fa92fb_blank(int blank_mode, struct fb_info *info)
{
    return w55fa92fb_blank_device(blank_mode, info);
}


static int i32OpenCount = 0;

static int w55fa92fb_open(struct fb_info *info, int init)
{
	ENTER();
	
	if (!(inl(REG_AHBCLK)&VPOST_CKE))
		return -EFAULT;	

#ifdef CONFIG_W55FA92_PINGPONG_SUPPORT
	g_fbiopan_cmd_sent = 0;
	g_var_yoffset = 0;
	g_bg_mmp_p = _bg_mem_p;	
	g_fbunlock_cmd_sent = 0;	                               	
	g_fb_cmd_flag = 0;
#else
	g_fbunlock_cmd_sent = 0;
	g_fb_cmd_flag = 0;	
#endif

#ifdef CONFIG_TWO_OSD_BUF
	g_osdunlock_cmd_sent = 0;
	g_osdunlock_cmd_sent_flag = 0;
#endif
	i32OpenCount++;

#ifdef CONFIG_W55FA92_TV_LCD_SWITCH
			outl((inl(REG_LCM_LCDCInt) & 0xFFFFFF00) | (_vpost_int << 16), REG_LCM_LCDCInt); // enable VIN	
	        _auto_disable = 0;	//## Chris  1;  set as _auto_disable=0 could let VIN continue enable, may degrade 004_bitmap.swf fps 10% on screen but not affect the console output fps.			
#else
	#if TV_OUTPUT_DEVICE
			outl((inl(REG_LCM_LCDCInt) & 0xFFFFFF00) | 0x40000, REG_LCM_LCDCInt); // enable VIN	//### Chris
	#else
			outl((inl(REG_LCM_LCDCInt) & 0xFFFFFF00) | 0x20000, REG_LCM_LCDCInt); // enable VIN	//### Chris
	#endif		
	        _auto_disable = 0;	//## Chris  1;  set as _auto_disable=0 could let VIN continue enable, may degrade 004_bitmap.swf fps 10% on screen but not affect the console output fps.
#endif      

  

	LEAVE();
		        	
    return 0;
}

#ifdef CONFIG_W55FA92_OSD_SUPPORT
static ssize_t w55fa92_write(struct fb_info *info, const char __user *buffer,
				 size_t count, loff_t *ppos)
				 
{
	int osd_buf_addr;
	int x_offset;
	int retval;

	ENTER();

	retval = count;
	
	if (!count)
		return -EFAULT;	

	/* OSD buffer can be written in several "write()" function calling */
	if (g_osd_fristWrite)
	{
		g_osd_x0 = osd_block.x0;
		osd_buf_addr = osd_cpu_mmap;
		osd_buf_addr += osd_block.y0 * LCDWIDTH * PIXEL_BSIZE;			
		osd_buf_addr += osd_block.x0 * PIXEL_BSIZE;
		g_osd_buf_addr = osd_buf_addr;
	}
	else
	{
		osd_buf_addr = g_osd_buf_addr;		
	}				
	
	// check if current bufffer addres locate between assigned OSD buffer
	if ((osd_buf_addr < osd_cpu_mmap) || (osd_buf_addr > (osd_cpu_mmap + osd_size)))
		return -EFAULT;	

	// check if "write()" beyond assigned OSD buffer
	if ((osd_buf_addr + count) > (osd_cpu_mmap + osd_size))
		return -EFAULT;	

	if (g_osd_fristWrite)
	{
		while(count/(osd_block.x0_size * PIXEL_BSIZE))
		{
			copy_from_user((unsigned int *)osd_buf_addr, (void *)buffer, osd_block.x0_size * PIXEL_BSIZE);
			buffer += osd_block.x0_size * PIXEL_BSIZE;
			osd_buf_addr += LCDWIDTH * PIXEL_BSIZE;			
			osd_block.y0_size --;
			osd_block.y0 ++;

			if (count >= (osd_block.x0_size * PIXEL_BSIZE))
				count -= osd_block.x0_size * PIXEL_BSIZE;
		}	
	
		while(count%(osd_block.x0_size * PIXEL_BSIZE))
		{
			copy_from_user((unsigned int *)osd_buf_addr, (void *)buffer, count%(osd_block.x0_size * PIXEL_BSIZE));
			osd_buf_addr += count%(osd_block.x0_size * PIXEL_BSIZE);			
			g_osd_x0 = osd_block.x0 + (count%(osd_block.x0_size * PIXEL_BSIZE)) / PIXEL_BSIZE;
			count -= count%(osd_block.x0_size * PIXEL_BSIZE);
			break;
		}	

	}
	else
	{
		x_offset = osd_block.x0_size - (g_osd_x0 - osd_block.x0);
		x_offset *= PIXEL_BSIZE;

		if (count >= x_offset)
		{
			copy_from_user((unsigned int *)osd_buf_addr, (void *)buffer, x_offset);					    
			
			count -= x_offset;
			buffer += x_offset;			
			
			osd_buf_addr -= (g_osd_x0 - osd_block.x0) * PIXEL_BSIZE;
			g_osd_x0 = osd_block.x0;						
			
			osd_block.y0_size --;			
			osd_block.y0 ++ ;

			osd_buf_addr += LCDWIDTH * PIXEL_BSIZE;							
			
			while(count/(osd_block.x0_size * PIXEL_BSIZE))
			{
				copy_from_user((unsigned int *)osd_buf_addr, (void *)buffer, osd_block.x0_size * PIXEL_BSIZE);
				buffer += osd_block.x0_size * PIXEL_BSIZE;
				osd_buf_addr += LCDWIDTH * PIXEL_BSIZE;			
				osd_block.y0_size --;
				osd_block.y0 ++ ;				
				
				if (count >= (osd_block.x0_size * PIXEL_BSIZE))
					count -= osd_block.x0_size * PIXEL_BSIZE;
			}	
		
			while(count%(osd_block.x0_size * PIXEL_BSIZE))
			{
				copy_from_user((unsigned int *)osd_buf_addr, (void *)buffer, count%(osd_block.x0_size * PIXEL_BSIZE));					    
				osd_buf_addr += count%(osd_block.x0_size * PIXEL_BSIZE);			
				g_osd_x0 = osd_block.x0 + (count%(osd_block.x0_size * PIXEL_BSIZE)) / PIXEL_BSIZE;
				count -= count%(osd_block.x0_size * PIXEL_BSIZE);				
				break;
			}	
		}			
		else
		{
			copy_from_user((unsigned int *)osd_buf_addr, (void *)buffer, count);					    			
			osd_buf_addr += count;
			g_osd_x0 += count/PIXEL_BSIZE;
		}			
	}				

	g_osd_fristWrite = 0;
	g_osd_buf_addr = osd_buf_addr;	

	LEAVE();
	
	return retval;
}
#endif	// CONFIG_W55FA92_OSD_SUPPORT

static int w55fa92fb_close(struct fb_info *info, int init)
{

	ENTER();

#ifdef CONFIG_TWO_OSD_BUF
	g_osdunlock_cmd_sent_flag = 0;
#endif
	_auto_disable = 0;
	
	i32OpenCount --;
	if (i32OpenCount == 1)		// console will call "fb_open()" one time
	{
#ifdef CONFIG_RGBx888_FORMAT
		//printk("w55fa95fb_close\n");
		outl((inl(REG_LCM_LCDCCtl) & ~LCDCCtl_FBDS) | ((DISPLAY_MODE_RGBx888&0x07)<<1) , REG_LCM_LCDCCtl); // change frame buffer source format	
		currentMode = VA_RGB888x;		
#else	
		//printk("w55fa95fb_close\n");
		outl((inl(REG_LCM_LCDCCtl) & ~LCDCCtl_FBDS) | ((DISPLAY_MODE_RGB565&0x07)<<1) , REG_LCM_LCDCCtl); // change frame buffer source format	
		currentMode = VA_RGB565;		
#endif		

	}
	else if (i32OpenCount == 0)
		return -1;		

	LEAVE();
	
	return 0;
}

static int mode_change(int *from, int to)
{
#if TV_OUTPUT_DEVICE
        int IsEnable, vpostINT = LCDCInt_TVFIELDINTEN;
#else
        int IsEnable, vpostINT = LCDCInt_VINTEN;
#endif

	ENTER();
        if (*from == to)
                return(0);
		destMode = to;

#ifdef CONFIG_W55FA92_TV_LCD_SWITCH

		if (_IsTVmode)
			vpostINT = LCDCInt_TVFIELDINTEN;
#endif

        IsEnable = (inl(REG_LCM_LCDCInt) & vpostINT) ? 1: 0;
        if (IsEnable) {
      		outl((inl(REG_LCM_LCDCInt) & 0xFFFFFF00) & ~vpostINT, REG_LCM_LCDCInt); // disable VIN or TVField INT
#ifdef CONFIG_TWO_FB_BUF
			if ( vdma_ch_flag != -1 )
	       		while (w55fa92_edma_isbusy(vdma_ch));                
#endif
        }
	
        changeMode = 1;        
        outl((inl(REG_LCM_LCDCInt) & 0xFFFFFF00) | vpostINT, REG_LCM_LCDCInt); 			// enable VIN or TVField INT
#ifdef CONFIG_TWO_FB_BUF
        wait_event_interruptible(wq, (changeMode == 0));
#else
		outl((inl(REG_LCM_LCDCCtl) & ~0xE) | ((destMode&0x07)<<1), REG_LCM_LCDCCtl);
		currentMode = destMode;
#endif
        if (!IsEnable) {
                outl((inl(REG_LCM_LCDCInt) & 0xFFFFFF00) & ~vpostINT, REG_LCM_LCDCInt); // disable VIN or TVField INT
#ifdef CONFIG_TWO_FB_BUF
			if ( vdma_ch_flag != -1 )
       		 	while (w55fa92_edma_isbusy(vdma_ch));                
#endif
        }
	LEAVE();        
		return 0;		
}

static int w55fa92fb_ioctl_additional(struct fb_info *info, unsigned int cmd, unsigned long arg)
{
	unsigned int buffer[5];
	
	ENTER();

	memset(buffer,0,5);
	switch(cmd)
	{
		case VIDEO_FORMAT_CHANGE:		
			mode_change(&currentMode, arg);
			
#ifdef CONFIG_W55FA92_TV_LCD_SWITCH
	#ifndef CONFIG_W55FA92_TV_QVGA
			outl((inl(REG_LCM_OSD_CTL) & ~OSD_CTL_OSD_FSEL)|(va_format[arg&0x07] << 24) , REG_LCM_OSD_CTL);	// change OSD format
	#endif
#endif	
			
#ifdef CONFIG_W55FA92_TV_LCD_SWITCH
			if (_IsTVmode)
				display_to_tv();			
			else
				display_to_lcd();
#endif	
			break;
			
#ifdef CONFIG_W55FA92_OSD_SUPPORT

		case OSD_SEND_CMD:
			copy_from_user((unsigned int *)&osd_buffer, (void *)arg, sizeof(osd_buffer));					    
			return w55fa92_osd_function(&osd_buffer);
			break;
#endif			

#ifdef CONFIG_W55FA92_TV_LCD_SWITCH
		case VIDEO_DISPLAY_LCD:
			printk("video displayed by LCD only\n");
			display_to_lcd();
	       	w55fa92fb_ioctl(0, VIDEO_DISPLAY_ON, 00);			// backlight is ON			
			break;				

		case VIDEO_DISPLAY_TV:
			printk("video displayed by TV only\n");
			display_to_tv();
	       	w55fa92fb_ioctl(0, VIDEO_DISPLAY_OFF, 00);			// backlight is OFF			
			break;				
		 case IOCTL_LCD_ENABLE_INT:
		 
#ifdef CONFIG_W55FA92_TV_LCD_SWITCH

			outl((inl(REG_LCM_LCDCInt) & 0xFFFFFF00) | (_vpost_int << 16), REG_LCM_LCDCInt); // enable VIN	
	        _auto_disable = 0;	//## Chris  1;  set as _auto_disable=0 could let VIN continue enable, may degrade 004_bitmap.swf fps 10% on screen but not affect the console output fps.			
#else
	#if TV_OUTPUT_DEVICE
			outl((inl(REG_LCM_LCDCInt) & 0xFFFFFF00) | 0x40000, REG_LCM_LCDCInt); // enable VIN	//### Chris
	#else
			outl((inl(REG_LCM_LCDCInt) & 0xFFFFFF00) | 0x20000, REG_LCM_LCDCInt); // enable VIN	//### Chris
	#endif		
	        _auto_disable = 0;	//## Chris  1;  set as _auto_disable=0 could let VIN continue enable, may degrade 004_bitmap.swf fps 10% on screen but not affect the console output fps.
#endif      
            break;
                
		 case IOCTL_LCD_DISABLE_INT:
	         
#ifdef CONFIG_W55FA92_TV_LCD_SWITCH
			outl((inl(REG_LCM_LCDCInt) & 0xFFFFFF00) & ~(_vpost_int << 16), REG_LCM_LCDCInt); // enable VIN	
#else
	#if TV_OUTPUT_DEVICE
			outl((inl(REG_LCM_LCDCInt) & 0xFFFFFF00) & ~0x40000, REG_LCM_LCDCInt); // enable VIN	//### Chris
	#else
			outl((inl(REG_LCM_LCDCInt) & 0xFFFFFF00) & ~0x20000, REG_LCM_LCDCInt); // enable VIN	//### Chris
	#endif		
#endif      
	         
#ifdef CONFIG_TWO_FB_BUF
			if ( vdma_ch_flag != -1 )
       		 	while (w55fa92_edma_isbusy(vdma_ch));
#endif            
             _auto_disable = 1;
             break;
			
#endif

		default:
			break;	
	}

	LEAVE();
		
	return 0;    
}


static int w55fa92fb_ioctl(struct fb_info *info, unsigned int cmd, unsigned long arg)
{
	int err = 0;	
	unsigned char buff[4*0x70];
	ENTER();
	
	switch(cmd)
	{
		case VIDEO_FORMAT_CHANGE:		

#ifdef CONFIG_W55FA92_OSD_SUPPORT
		case OSD_SEND_CMD:
#endif		
			return w55fa92fb_ioctl_additional(info, cmd, arg);		

#ifdef CONFIG_W55FA92_TV_LCD_SWITCH
		case VIDEO_DISPLAY_LCD:
			_IsTVmode = 0;
			return w55fa92fb_ioctl_additional(info, cmd, arg);		
						
		case VIDEO_DISPLAY_TV:		
			_IsTVmode = 1;		
			return w55fa92fb_ioctl_additional(info, cmd, arg);		

		case IOCTL_LCD_ENABLE_INT:
		case IOCTL_LCD_DISABLE_INT:		
			return w55fa92fb_ioctl_additional(info, cmd, arg);		
		case VIDEO_TV_SYSTEM:
			_tv_system  = arg & 0x01;	// 0: NTSC, 1: PAL
	    	outl(inl(REG_LCM_TVCtl)& ~0x04, REG_LCM_TVCtl);	
	    	outl(inl(REG_LCM_TVCtl)|(_tv_system << 2), REG_LCM_TVCtl);	// select TV system 
			break;				
#else
		case VIDEO_TV_SYSTEM:
			_tv_system  = arg & 0x01;	// 0: NTSC, 1: PAL
	    	outl(inl(REG_LCM_TVCtl)& ~0x04, REG_LCM_TVCtl);	
	    	outl(inl(REG_LCM_TVCtl)|(_tv_system << 2), REG_LCM_TVCtl);	// select TV system 
			break;				

		case VIDEO_DISPLAY_LCD:
		case VIDEO_DISPLAY_TV:		
		case IOCTL_LCD_ENABLE_INT:
		case IOCTL_LCD_DISABLE_INT:		
#endif

		case IOCTLCLEARSCREEN:	
		case VIDEO_ACTIVE_WINDOW_COORDINATES:
		case IOCTL_LCD_BRIGHTNESS:
		case IOCTL_LCD_GET_DMA_BASE:
		case DUMP_LCD_REG:		
		case VIDEO_DISPLAY_OFF:		
    		return w55fa92fb_ioctl_device(info, cmd, arg);				

		case VIDEO_DISPLAY_ON:
#ifndef CONFIG_W55FA92_TV_LCD_SWITCH    				
    		return w55fa92fb_ioctl_device(info, cmd, arg);				
#else
			if(_IsTVmode)
				break;
			else				
    			return w55fa92fb_ioctl_device(info, cmd, arg);							
#endif		

		case IOCTL_GET_FB_OFFLINE:
			if(copy_to_user((int *)arg, (int *)&_bg_mem_p, sizeof(int)) < 0) {
			    printk("failed..\n");
			    return(-EFAULT);
			}
			printk("Get FB offline address = %d\n", (unsigned int) _bg_mem_p);
			break;

#ifdef CONFIG_W55FA92_OSD_SUPPORT
		case IOCTL_GET_OSD_OFFLINE:
			if(copy_to_user((int *)arg, (int *)&(_osd_mem_p), sizeof(int)) < 0) {
			    printk("failed..\n");
			    return(-EFAULT);
			}
			printk("Get OSD offlinet address = %d\n", (unsigned int) _osd_mem_p);
			break;

		case IOCTL_GET_OSD_OFFSET:
			if(copy_to_user((int *)arg, (int *)&osd_offset, sizeof(int)) < 0) {
			    printk("failed..\n");
			    return(-EFAULT);
			}
			printk("Get OSD offset address = %d\n", (unsigned int) osd_offset);
			break;

	#ifdef CONFIG_TWO_OSD_BUF					
		case IOCTL_OSD_LOCK:
			if ( vdma_ch_flag != -1 )
				while (w55fa92_edma_isbusy(vdma_ch));

			g_osdunlock_cmd_sent = 0;
			break;

		case IOCTL_OSD_UNLOCK:
			g_osdunlock_cmd_sent_flag = 1;
			if ( vdma_ch_flag != -1 )
				while (w55fa92_edma_isbusy(vdma_ch));

			g_osdunlock_cmd_sent = 1;
			break;
	#endif
#endif
	
	  #ifdef OPT_FB_LOCK	
		case IOCTL_FB_LOCK:
			g_fb_cmd_flag = 1;

#ifdef CONFIG_TWO_FB_BUF
			if ( vdma_ch_flag != -1 )
				while (w55fa92_edma_isbusy(vdma_ch));
#endif            
			g_fbunlock_cmd_sent = 0;
			break;

		case IOCTL_FB_UNLOCK:
			g_fb_cmd_flag = 1;		

#ifdef CONFIG_TWO_FB_BUF
			if ( vdma_ch_flag != -1 )
				while (w55fa92_edma_isbusy(vdma_ch));
#endif            
			g_fbunlock_cmd_sent = 1;
			break;
			
		case IOCTL_FB_LOCK_RESET:
			g_fb_cmd_flag = 0;		
			break;
			
	  #endif			

		case IOCTL_WAIT_VSYNC:
			g_fb_vsync_flag = 1;
        	if (wait_event_interruptible(vsyncq, (g_fb_vsync_flag == 0))) {
        		err = -ERESTARTSYS;
        	}
			break;

		case IOCTL_FB_SCALE_ENABLE:
			copy_from_user((unsigned int *)&scale_buf, (void *)arg, sizeof(scale_buf));					    
			scale_ptr = (scale_cmd_t *)&scale_buf;					    

			outl(inl(REG_LCM_LCDCCtl) & ~LCDCCtl_LCDRUN, REG_LCM_LCDCCtl); 
			memcpy((char*)buff, (char*)REG_LCM_LCDCCtl, 4*0x70);
			outl(inl(REG_AHBCLK) | VPOST_CKE | HCLK4_CKE, REG_AHBCLK);
			outl(inl(REG_AHBIPRST) | VPOST_RST, REG_AHBIPRST);
			outl(inl(REG_AHBIPRST) & ~VPOST_RST, REG_AHBIPRST);	

			memcpy((char*)REG_LCM_LCDCCtl, (char*)buff, 4*0x70);
			if( (info->var.xres < scale_ptr->src_xres) 
			 || (info->var.yres < scale_ptr->src_yres) )
			{
				return(-EFAULT);
			}

			outl( (inl(REG_LCM_FB_SIZE) & ~(FB_X | FB_Y))
			    | ((scale_ptr->src_xres-1 & 0xFFFF) << 16)
			    |  (scale_ptr->src_yres-1 & 0xFFFF), REG_LCM_FB_SIZE);

			outl( (inl(REG_LCM_SCO_SIZE) & ~(SCOL_X | SCOL_Y))
			    | ((info->var.xres-1 & 0xFFFF) << 16)
			    |  (info->var.yres-1 & 0xFFFF), REG_LCM_SCO_SIZE);

			outl(inl(REG_LCM_LCDCCtl) | LCDCCtl_SC_EN, REG_LCM_LCDCCtl); 	// enable scaling
			outl(inl(REG_LCM_LCDCCtl) | LCDCCtl_LCDRUN, REG_LCM_LCDCCtl); 			
			break;
			
			
		case IOCTL_FB_SCALE_DISABLE:
			outl(inl(REG_LCM_LCDCCtl) & ~LCDCCtl_LCDRUN, REG_LCM_LCDCCtl); 
			memcpy((char*)buff, (char*)REG_LCM_LCDCCtl, 4*0x70);
			outl(inl(REG_AHBCLK) | VPOST_CKE | HCLK4_CKE, REG_AHBCLK);
			outl(inl(REG_AHBIPRST) | VPOST_RST, REG_AHBIPRST);
			outl(inl(REG_AHBIPRST) & ~VPOST_RST, REG_AHBIPRST);	
 
 			memcpy((char*)REG_LCM_LCDCCtl, (char*)buff, 4*0x70);
			outl( (inl(REG_LCM_FB_SIZE) & ~(FB_X | FB_Y))
			    | ((info->var.xres-1 & 0xFFFF) << 16)
			    |  (info->var.yres-1 & 0xFFFF), REG_LCM_FB_SIZE);

			outl(inl(REG_LCM_LCDCCtl) & ~LCDCCtl_SC_EN, REG_LCM_LCDCCtl); 	// enable scaling		
			outl(inl(REG_LCM_LCDCCtl) | LCDCCtl_LCDRUN, REG_LCM_LCDCCtl); 						
			break;
		default:
		    return(-ENOIOCTLCMD);		
			break;	
	}

	LEAVE();
		
	return err;    
}

static struct fb_ops w55fa92fb_ops = {
	.owner		= THIS_MODULE,
	.fb_check_var	= w55fa92fb_check_var,
	.fb_set_par		= w55fa92fb_set_par,
	.fb_blank			= w55fa92fb_blank,
	.fb_setcolreg	= w55fa92fb_setcolreg,
	.fb_fillrect	= cfb_fillrect,
	.fb_copyarea	= cfb_copyarea,
	.fb_imageblit	= cfb_imageblit,
	.fb_open	= w55fa92fb_open,
#ifdef CONFIG_W55FA92_OSD_SUPPORT	
	.fb_write	= w55fa92_write,
#endif
	.fb_release	= w55fa92fb_close,			//### Chris	
	.fb_ioctl			= w55fa92fb_ioctl, 
#ifdef CONFIG_W55FA92_PINGPONG_SUPPORT	
	.fb_pan_display	= w55fa92fb_pan_display,	
#endif	
	/*.fb_mmap 			= w55fa92fb_mmap,*/
};

void LCDDelay(unsigned int nCount)
{
		unsigned volatile int i;
		
		for(;nCount!=0;nCount--)
			for(i=0;i<100;i++);
}

#ifdef CONFIG_ASSIGN_FB_ADDR
extern unsigned int w55fa92_fb_v;
#endif

/*
	1.	If OSD is not selected and 1 FB buffer selected, it means that
		the memory layout will be 
		a.	FB_online only

	2.	If OSD is not selected and 2 FB buffers selected, i.e., FB_online+FB_offline, 
		it means that the memory layout will be 
		a.	FB_online+FB_offline (x1 or x2) (memory is continuous)
		  >	if PINGPONG selected, then FB_offlinex2
		  >	if PINGPONG not selected, then FB_offlinex1
		  
	3.	Suppose only one OSD buffer selected, i.e., OSD_onlinex1, the memory layout shall be 
		a.	FB_online
		b.	FB_offline (x1 or x2)+OSD_online (memory is continuous)
		
	4.	Suppose 2 OSD buffers selected, i.e., OSD_onlinex1+1xOSD_offlinex1, it means that the memory layout will be 
		a.	FB_online+OSD_online (memory is continuous)
		b.	FB_offline (x1 or x2)+OSD_offline (memory is continuous)
		
*/
static int __init w55fa92fb_map_video_memory(struct w55fa92fb_info *fbi)
{
	ENTER();
	
#ifdef CONFIG_W55FA92_PINGPONG_SUPPORT		
	#ifdef CONFIG_W55FA92_OSD_SUPPORT		
	        fbi->map_size = PAGE_ALIGN(fbi->fb->fix.smem_len/3 + PAGE_SIZE);
	#else
	        fbi->map_size = PAGE_ALIGN(fbi->fb->fix.smem_len/2 + PAGE_SIZE);
	#endif        
#else
	#ifdef CONFIG_W55FA92_OSD_SUPPORT		
	        fbi->map_size = PAGE_ALIGN(fbi->fb->fix.smem_len/2 + PAGE_SIZE);
	#else
	        fbi->map_size = PAGE_ALIGN(fbi->fb->fix.smem_len + PAGE_SIZE);
	#endif        
#endif        
    printk("1*** fbi->fb->fix.smem_len = 0x%x\n", fbi->fb->fix.smem_len);  
        
        /* Allocate the whole buffer size for both video */
             
#ifdef CONFIG_TWO_FB_BUF
	#ifndef OPT_MMU_ALLOCATE_FB
		#ifdef CONFIG_W55FA92_PINGPONG_SUPPORT
			#ifdef CONFIG_W55FA92_OSD_SUPPORT		
				g_map_size = fbi->fb->fix.smem_len/3;
			#else
				g_map_size = fbi->fb->fix.smem_len/2;		
			#endif			

			/* "dma_alloc_writecombine" can't allocate more than 4 MB */
			#ifdef CONFIG_W55FA92_OSD_SUPPORT	
			    _bg_mem_v  = (int) dma_alloc_writecombine(fbi->dev, fbi->map_size*3, &_bg_mem_p, GFP_KERNEL);
			#else
		        _bg_mem_v  = (int) dma_alloc_writecombine(fbi->dev, fbi->map_size*2, &_bg_mem_p, GFP_KERNEL);
			#endif                                            
		#else
			#ifdef CONFIG_W55FA92_OSD_SUPPORT	
			    _bg_mem_v  = (int) dma_alloc_writecombine(fbi->dev, fbi->map_size*2, &_bg_mem_p, GFP_KERNEL);                              
			#else
		        _bg_mem_v  = (int) dma_alloc_writecombine(fbi->dev, fbi->map_size, &_bg_mem_p, GFP_KERNEL);                              
			#endif                                            
		#endif		

	#else		
		_bg_mem_v = w55fa92_fb_mem_v;
		_bg_mem_p = w55fa92_fb_mem_p;
			
		#ifdef CONFIG_W55FA92_PINGPONG_SUPPORT
			#ifdef CONFIG_W55FA92_OSD_SUPPORT		
				g_map_size = fbi->fb->fix.smem_len/3;
			#else
				g_map_size = fbi->fb->fix.smem_len/2;		
			#endif			
		#endif		
	#endif	
	// off-screen buffer for dual buffer
    //printk("1*** _bg_mem_v 0x%x\n", _bg_mem_v);  
	//printk("1*** _bg_mem_p 0x%x\n", _bg_mem_p);
        if (_bg_mem_v == 0)
                return -ENOMEM;
                
	#if defined(CONFIG_W55FA92_TV_LCD_SWITCH) && (!defined(CONFIG_W55FA92_TV_QVGA))
	    _tv_mem_v  = (int) dma_alloc_writecombine(fbi->dev, TV_BUFF_SIZE, &_tv_mem_p, GFP_KERNEL);  	
        if (_tv_mem_v == 0)
                return -ENOMEM;
                
        memset((char*)_tv_mem_v, 0x00, TV_BUFF_SIZE); // clear TV buffer to black color
	#endif	
	// TV background buffer; only for support TV/LCD switch application	
                
#endif

#ifndef CONFIG_ASSIGN_FB_ADDR

	#ifdef CONFIG_W55FA92_OSD_SUPPORT	
		#ifdef CONFIG_TWO_FB_BUF
			#ifdef CONFIG_TWO_OSD_BUF		    
	        fbi->map_cpu  = (u_char *)dma_alloc_writecombine(fbi->dev, fbi->map_size*2, &fbi->map_dma, GFP_KERNEL);
			#else		
			fbi->map_cpu  = (u_char *) dma_alloc_writecombine(fbi->dev, fbi->map_size, &fbi->map_dma, GFP_KERNEL);	
			#endif	        
		#else
	        fbi->map_cpu  = (u_char *) dma_alloc_writecombine(fbi->dev, fbi->map_size*2, &fbi->map_dma, GFP_KERNEL);
		#endif        
	#else
	        fbi->map_cpu  = (u_char *) dma_alloc_writecombine(fbi->dev, fbi->map_size, &fbi->map_dma, GFP_KERNEL);		
	#endif		

	// on-screen buffer for single/dual buffer buffer
    //printk("2*** fbi->map_cpu 0x%x\n", fbi->map_cpu);  
	//printk("2*** fbi->map_dma 0x%x\n", fbi->map_dma);
#else
	fbi->map_cpu = w55fa92_fb_v;
    fbi->map_dma = CONFIG_FRAME_BUFFER_ADDR;
	outl((inl(REG_LCM_LCDCInt)& ~0x0003), REG_LCM_LCDCInt);		// clear Vsync/Hsync sync Flags				
	while(1)
	{
		printk("REG_LCM_LCDCInt = 0x%x !!!\n", inl(REG_LCM_LCDCInt));				

#if	!defined(CONFIG_W55FA92_FB_INIT)
		if (inl(REG_LCM_LCDCInt) & 0x02)	// wait VSync 
#endif		
		{
			
			outl(CONFIG_FRAME_BUFFER_ADDR, REG_LCM_FSADDR);					
			
#ifdef CONFIG_RGBx888_FORMAT
			outl((inl(REG_LCM_LCDCCtl)& 0xFFFEFFF0)|(0x10005), REG_LCM_LCDCCtl);	// change source flormat to RGB565	
#else	
			outl((inl(REG_LCM_LCDCCtl)& 0xFFFEFFF0)|(0x10003), REG_LCM_LCDCCtl);	// change source flormat to RGB565	
#endif		
			printk("Vsync flag is encountered !!!\n");
			break;
		}					
	}

#endif

#ifdef CONFIG_W55FA92_PINGPONG_SUPPORT		
	#ifdef CONFIG_W55FA92_OSD_SUPPORT		
	        fbi->map_size = fbi->fb->fix.smem_len/3;
		#ifdef CONFIG_TWO_OSD_BUF		    	        
			_osd_mem_v = _bg_mem_v + 2*fbi->map_size;
			_osd_mem_p = _bg_mem_p + 2*fbi->map_size;			
		#else
			// if one OSD buffer is configured, on-line OSD address is set to "_osd_mem_p" 
			_osd_mem_v = osd_cpu_mmap;		//
			_osd_mem_p = osd_dma_mmap;
		#endif	        
	#else
	        fbi->map_size = fbi->fb->fix.smem_len/2;
	#endif        
#else
	#ifdef CONFIG_W55FA92_OSD_SUPPORT		
	        fbi->map_size = fbi->fb->fix.smem_len/2;
		#ifdef CONFIG_TWO_OSD_BUF		    	        
			_osd_mem_v = _bg_mem_v + fbi->map_size;
			_osd_mem_p = _bg_mem_p + fbi->map_size;			
		#else
			// if one OSD buffer is configured, on-line OSD address is set to "_osd_mem_p" 
			_osd_mem_v = osd_cpu_mmap;
			_osd_mem_p = osd_dma_mmap;
		#endif	        
	#else
	        fbi->map_size = fbi->fb->fix.smem_len;
	#endif        
#endif        

#ifdef CONFIG_TWO_FB_BUF
        memcpy((char*)_bg_mem_v, (char*)fbi->map_cpu, fbi->map_size); // make two buffer consistent

	#ifdef CONFIG_W55FA92_PINGPONG_SUPPORT
        memcpy((char*)_bg_mem_v+fbi->map_size, (char*)fbi->map_cpu, fbi->map_size); // make two buffer consistent	
	#endif
    	if( (inl(REG_AHBCLK) & EDMA0_CKE) ==0)
    	{
    		printk("EDMA Ch0 clock is off, turn it\n\r");	
    		clk_enable(clk_edma);				// enable EDMA clock    		
    	}	
        _dma_len = fbi->map_size;               // Transfer Size
        _dma_dst = fbi->map_dma;                // Physical Destination Addr
        _dma_dst_v = (long)fbi->map_cpu;        // Virtual Destination Addr        
#endif

        if (fbi->map_cpu) {
                /* prevent initial garbage on screen */
                //memset(fbi->map_cpu, 0xff, fbi->map_size);
#ifdef CONFIG_TWO_FB_BUF
		// Dual Buffer
	        fbi->screen_dma	= _bg_mem_p;
	        fbi->fb->screen_base = (void*)_bg_mem_v;
#else
    		// Single Buffer
    		fbi->screen_dma		= fbi->map_dma;
    		fbi->fb->screen_base	= fbi->map_cpu;
#endif
                fbi->fb->fix.smem_start  = fbi->screen_dma;
        } 
        else {
#ifdef CONFIG_TWO_FB_BUF
	#ifndef OPT_MMU_ALLOCATE_FB
		#ifdef CONFIG_W55FA92_PINGPONG_SUPPORT
			#ifdef CONFIG_W55FA92_OSD_SUPPORT	
		                dma_free_writecombine(fbi->dev, fbi->map_size*3, (void*)_bg_mem_v, _bg_mem_p);	
			#else
		                dma_free_writecombine(fbi->dev, fbi->map_size*2, (void*)_bg_mem_v, _bg_mem_p);
			#endif                
		#else
			#ifdef CONFIG_W55FA92_OSD_SUPPORT	
		                dma_free_writecombine(fbi->dev, fbi->map_size*2, (void*)_bg_mem_v, _bg_mem_p);	
			#else
		                dma_free_writecombine(fbi->dev, fbi->map_size, (void*)_bg_mem_v, _bg_mem_p);
			#endif                
		#endif                
	#endif		
	
	#if defined(CONFIG_W55FA92_TV_LCD_SWITCH) && (!defined(CONFIG_W55FA92_TV_QVGA))
                dma_free_writecombine(fbi->dev, TV_BUFF_SIZE, (void*)_tv_mem_v, _tv_mem_p);	
	#endif	
#endif
		}

        /* video_buf_mmap is the LCD physical starting address, cpu is the virtual */
        video_cpu_mmap=(unsigned long)fbi->map_cpu;
        video_dma_mmap=(unsigned long)fbi->map_dma;
        video_buf_mmap=(unsigned long)fbi->map_size;
        //memset(fbi->map_cpu, 0x33, g_LCDWholeBuffer);

#ifdef CONFIG_W55FA92_OSD_SUPPORT
	#ifdef CONFIG_TWO_FB_BUF
		#ifdef CONFIG_TWO_OSD_BUF		    
        osd_cpu_mmap=(unsigned long)video_cpu_mmap + fbi->map_size;
        osd_dma_mmap=(unsigned long)video_dma_mmap + fbi->map_size;
        osd_size = fbi->map_size;
		#else	
        osd_cpu_mmap=(unsigned long)_bg_mem_v + fbi->map_size;
        osd_dma_mmap=(unsigned long)_bg_mem_p + fbi->map_size;
        osd_size = fbi->map_size;
		#endif
	#else
        osd_cpu_mmap=(unsigned long)video_cpu_mmap + fbi->map_size;
        osd_dma_mmap=(unsigned long)video_dma_mmap + fbi->map_size;
        osd_size = fbi->map_size;
	#endif        
        
	#ifdef CONFIG_W55FA92_PINGPONG_SUPPORT        
		#ifdef CONFIG_TWO_FB_BUF
			#ifdef CONFIG_TWO_OSD_BUF		    
	        osd_cpu_mmap=(unsigned long)video_cpu_mmap + fbi->map_size;
	        osd_dma_mmap=(unsigned long)video_dma_mmap + fbi->map_size;
//	        osd_cpu_mmap=(unsigned long)_bg_mem_v + 2*fbi->map_size;
//	        osd_dma_mmap=(unsigned long)_bg_mem_p + 2*fbi->map_size;
	        #else	
	        osd_cpu_mmap=(unsigned long)_bg_mem_v + 2*fbi->map_size;
	        osd_dma_mmap=(unsigned long)_bg_mem_p + 2*fbi->map_size;
			#endif	        
		#endif

        osd_offset = 2*fbi->map_size;	
	#else
        osd_offset = fbi->map_size;	
	#endif        
	
#ifdef CONFIG_TWO_OSD_BUF		    
        memset(osd_cpu_mmap, 0x00, fbi->map_size); // clear OSD on-line buffer
#endif 
#endif

	LEAVE();

        return fbi->map_cpu ? 0 : -ENOMEM;
}

static inline void w55fa92fb_unmap_video_memory(struct w55fa92fb_info *fbi)
{
	ENTER();

#ifndef CONFIG_ASSIGN_FB_ADDR 

	#ifdef CONFIG_W55FA92_OSD_SUPPORT	
		#ifdef CONFIG_TWO_FB_BUF	
#ifdef CONFIG_TWO_OSD_BUF		    
			dma_free_writecombine(fbi->dev,/*g_LCDWholeBuffer*/fbi->map_size*2,(void*)fbi->map_cpu, fbi->map_dma);	
#else		
			dma_free_writecombine(fbi->dev,/*g_LCDWholeBuffer*/fbi->map_size,(void*)fbi->map_cpu, fbi->map_dma);	
#endif			
		#else
			dma_free_writecombine(fbi->dev,/*g_LCDWholeBuffer*/fbi->map_size*2, (void*)fbi->map_cpu, fbi->map_dma);			
		#endif			
		
	#else
			dma_free_writecombine(fbi->dev,/*g_LCDWholeBuffer*/fbi->map_size, (void*)fbi->map_cpu, fbi->map_dma);
	#endif		
	
#endif

#ifdef CONFIG_TWO_FB_BUF
	#ifndef OPT_MMU_ALLOCATE_FB
		#ifdef CONFIG_W55FA92_PINGPONG_SUPPORT
			#ifdef CONFIG_W55FA92_OSD_SUPPORT	
		                dma_free_writecombine(fbi->dev, fbi->map_size*3, (void*)_bg_mem_v, _bg_mem_p);	
			#else
		                dma_free_writecombine(fbi->dev, fbi->map_size*2, (void*)_bg_mem_v, _bg_mem_p);
			#endif                
		#else
			#ifdef CONFIG_W55FA92_OSD_SUPPORT	
		                dma_free_writecombine(fbi->dev, fbi->map_size*2, (void*)_bg_mem_v, _bg_mem_p);	
			#else
		                dma_free_writecombine(fbi->dev, fbi->map_size, (void*)_bg_mem_v, _bg_mem_p);
			#endif                
		#endif                
	#endif                
	
	#if defined(CONFIG_W55FA92_TV_LCD_SWITCH) && (!defined(CONFIG_W55FA92_TV_QVGA))
                dma_free_writecombine(fbi->dev, TV_BUFF_SIZE, (void*)_tv_mem_v, _tv_mem_p);	
	#endif	
#endif

	LEAVE();
}

/*
 * w55fa92fb_init_registers - Initialise all LCD-related registers
 */
#ifdef CONFIG_W55FA92_FB_INIT 
static int w55fa92fb_init_registers(struct w55fa92fb_info *fbi)
{

	return w55fa92fb_init_device(fbi);

}	
#endif


void w55fa92fb_init_pwm(void)
{
#ifdef CONFIG_W55FA92_PWM_INIT
	/* Enable PWM clock */
	struct clk *clk = clk_get(NULL, "pwm");

	BUG_ON(IS_ERR(clk));
	clk_enable(clk);

	nvt_lock();	
	outl(inl(REG_CLKDIV5) & ~0x1FFF, REG_CLKDIV5);
	nvt_unlock();		

	/* Set divider to 1 */
	outl(0x4444, REG_PWM_CSR);
	
	/* Set all to Toggle mode  */
	outl(0x0C0C0C0C, REG_PCR);	

	// set PWM clock to 1MHz
	outl((((w55fa92_external_clock/1000000 - 1) << 8)|(w55fa92_external_clock/1000000 - 1)), REG_PPR);	
#endif
	w55fa92fb_init_pwm_device();
}

//#define PWM_OUTPUT_FREQUENCY 500	
//#define PWM_DEFAULT_OUTPUT_FREQUENCY 500

void w55fa92fb_set_CMR(unsigned int arg)
{
	unsigned int value;
#ifdef CONFIG_REVERSE_PWM
	value = g_PWMOutputDuty - arg + 1;
#else
	value = arg;
#endif
printk("PWM value = %d\n", value);
	if(value > g_PWMOutputDuty)
		value = g_PWMOutputDuty;
	else if(value < 1)
		value = 1;
	
	switch(w55fa92_pwm_channel)
	{
		case PWM0:
			outl(value, REG_CMR0);
			outl((inl(REG_PCR) | 0xD), REG_PCR);
			break;
		case PWM1:
			outl(value, REG_CMR1);
			outl((inl(REG_PCR) | 0xD00), REG_PCR);
			break;
		case PWM2:
			outl(value, REG_CMR2);
			outl((inl(REG_PCR) | 0xD0000), REG_PCR);
			break;
		case PWM3:
			outl(value, REG_CMR3);
			outl((inl(REG_PCR) | 0xD000000), REG_PCR);
			break;	
	}
}

void w55fa92fb_set_pwm_channel(unsigned int arg)
{
	w55fa92_pwm_channel = arg;
#ifdef CONFIG_W55FA92_SETPWM
#ifdef CONFIG_W55FA92_PWM_INIT
	outl(arg, REG_POE);
	outl(arg, REG_PIER);

	switch(w55fa92_pwm_channel)
	{
		case PWM0:
			/* Enable Channel 0 */	
			outl(0x0808080D, REG_PCR);	
			/* Set Channel 0 Pin function */
			outl(((inl(REG_GPDFUN0) & ~MF_GPD0) | 0x02), REG_GPDFUN0);	
			/* PWM frequency should be between 100Hz and 1KHz */
			/* Set PWM Output frequency 500Hz */
			outl((1000000)/CONFIG_PWM_OUTPUT_FREQUENCY, REG_CNR0);  // duty 2000 ~ 1
			/* default PWM duty is full backlight */
#ifdef CONFIG_REVERSE_PWM
			outl(1, REG_CMR0);	
#else
			outl((1000000)/CONFIG_PWM_DEFAULT_OUTPUT_FREQUENCY, REG_CMR0);	
#endif
			/* Enable Channel 0 */	
			outl(0x0808080D, REG_PCR);				
			break;
		case PWM1:
			/* Enable Channel 1 */	
			outl(0x08080D08, REG_PCR);	
			/* Set Channel 1 Pin function */	
			outl(((inl(REG_GPDFUN0) & ~MF_GPD1) | 0x20), REG_GPDFUN0);
			/* PWM frequency should be between 100Hz and 1KHz */
			/* Set PWM Output frequency 500Hz */
			outl((1000000)/CONFIG_PWM_OUTPUT_FREQUENCY, REG_CNR1);  // duty 2000 ~ 1
			/* default PWM duty is full backlight */
#ifdef CONFIG_REVERSE_PWM
			outl(1, REG_CMR0);	
#else
			outl((1000000)/CONFIG_PWM_DEFAULT_OUTPUT_FREQUENCY, REG_CMR1);	
#endif
			/* Enable Channel 1 */	
			outl(0x08080D08, REG_PCR);
			break;
		case PWM2:
			/* Enable Channel 2 */	
			outl(0x080D0808, REG_PCR);	
			/* Set Channel Pin function */
			outl(((inl(REG_GPDFUN0) & ~MF_GPD2) | 0x200), REG_GPDFUN0);
			/* PWM frequency should be between 100Hz and 1KHz */
			/* Set PWM Output frequency 500Hz */
#ifdef CONFIG_REVERSE_PWM
			outl(1, REG_CMR0);	
#else
			outl((1000000)/CONFIG_PWM_OUTPUT_FREQUENCY, REG_CNR2);  // duty 2000 ~ 1
#endif
			/* default PWM duty is full backlight */
			outl((1000000)/CONFIG_PWM_DEFAULT_OUTPUT_FREQUENCY, REG_CMR2);	
			break;
		case PWM3:
			/* Enable Channel 3 */	
			outl(0x0D080808, REG_PCR);	
			/* Set Channel 3 Pin function */
			outl(((inl(REG_GPDFUN0) & ~MF_GPD3) | 0x2000), REG_GPDFUN0);	
			/* PWM frequency should be between 100Hz and 1KHz */
			/* Set PWM Output frequency 500Hz */
			outl((1000000)/CONFIG_PWM_OUTPUT_FREQUENCY, REG_CNR3);  // duty 2000 ~ 1
			/* default PWM duty is full backlight */
#ifdef CONFIG_REVERSE_PWM
			outl(1, REG_CMR0);	
#else
			outl((1000000)/CONFIG_PWM_DEFAULT_OUTPUT_FREQUENCY, REG_CMR3);	
#endif
			/* Enable Channel 3 */	
			outl(0x0D080808, REG_PCR);	
			break;
	}
#endif
	g_PWMOutputDuty = (1000000)/CONFIG_PWM_OUTPUT_FREQUENCY;
	//printk("LCD PWM Output Duty = 0x%x\r\n", g_PWMOutputDuty);
#endif
}

static irqreturn_t w55fa92fb_irq(int irq, void *dev_id)
{
//	ENTER();
	
        int ret;
        unsigned long lcdirq = inl(REG_LCM_LCDCInt);
    g_fb_vsync_flag = 0;		// for PX10 test @20120731
   	wake_up_interruptible(&vsyncq);
        
#if defined(CONFIG_W55FA92_VIDEOIN_DEV1) && !defined(CONFIG_W55FA92_VIDEOIN_DEV2)	
		if (bIsVideoInEnable==1)		
		{
        	outl(w55fa92_VIN_PAC_BUFFER, REG_LCM_FSADDR);
        	outl((inl(REG_LCM_LCDCInt) & 0xFFFEFF00), REG_LCM_LCDCInt);
        	if(!changeMode)
        		return IRQ_HANDLED;
		}
#endif
#if !defined(CONFIG_W55FA92_VIDEOIN_DEV1) && defined(CONFIG_W55FA92_VIDEOIN_DEV2)	
		if(bIsVideoIn2Enable==1)
		{
        	outl(w55fa92_VIN_PAC_BUFFER, REG_LCM_FSADDR);
        	outl((inl(REG_LCM_LCDCInt) & 0xFFFEFF00), REG_LCM_LCDCInt);
        	if(!changeMode)        	
        		return IRQ_HANDLED;
		}
#endif
#if defined(CONFIG_W55FA92_VIDEOIN_DEV1) && defined(CONFIG_W55FA92_VIDEOIN_DEV2)	
		if( (bIsVideoInEnable==1) || (bIsVideoIn2Enable==1))
		{
        	outl(w55fa92_VIN_PAC_BUFFER, REG_LCM_FSADDR);
        	outl((inl(REG_LCM_LCDCInt) & 0xFFFEFF00), REG_LCM_LCDCInt);
        	if(!changeMode)
	        	return IRQ_HANDLED;
		}
#endif
	
//	printk("Enter LCD Int Status = 0x%x\n", lcdirq);

#ifdef CONFIG_W55FA92_TV_LCD_SWITCH
        if (lcdirq & _vpost_int){
#else
	#if TV_OUTPUT_DEVICE
        //if ((lcdirq & 0x4) && (inl(REG_LCM_TVCtl) & 0x80000000)){
        if (lcdirq & LCDCInt_TVFIELDINT){
	#else
        if (lcdirq & LCDCInt_VINT) {
	#endif            
#endif	
#ifdef CONFIG_TWO_FB_BUF

	#ifdef CONFIG_TWO_OSD_BUF
		//====================================================================
		// check OSD whether OSD is unlock or ont	
		//====================================================================	
			if(g_osdunlock_cmd_sent == 1)
			{
			//	g_osdunlock_cmd_sent = 2;

				 if ( vdma_ch_flag != -1 )
					goto VDMA_EXIT;
	

		        ret = w55fa92_edma_request(vdma_ch,"w55fa92-VPOST");
	       	    if (ret < 0) {
	        		printk("Request VDMA fail.\n");
	        		goto VDMA_EXIT;
		        }
				else
				{
	      			vdma_ch_flag = vdma_ch;
	      			
		#ifdef OPT_OSD_FLEXIBLE	      			
					if (g_osd_size)
	    				ret = w55fa92_edma_setup_single(vdma_ch, _bg_mem_p + g_map_size*2, osd_dma_mmap, g_osd_size);
					else	    				
	    				ret = w55fa92_edma_setup_single(vdma_ch, _bg_mem_p + g_map_size*2, osd_dma_mmap, g_map_size);					
		#else
	    			ret = w55fa92_edma_setup_single(vdma_ch, _bg_mem_p + g_map_size*2, osd_dma_mmap, g_map_size);
		#endif	    			
			
	    	        if (ret < 0) {
	    	        	printk("w55fa92_edma_setup_single failed and returns %d\n", ret);
	    	        	goto VDMA_FREE;
	    	        }
			#ifdef EDMA_USE_IRQ
					ret = w55fa92_edma_setup_handlers(vdma_ch, 2, (void*)fb_edma_irq_handler, NULL);			
	    	        if (ret < 0) {
	    	        	printk("w55fa92_edma_setup_handlers failed and returns %d\n", ret);
	    	        	goto VDMA_FREE;
	    	        }
			#endif
					flush_cache_all();
	    			w55fa92_edma_trigger(vdma_ch);
	    			
			#ifndef EDMA_USE_IRQ
					while (w55fa92_edma_isbusy(vdma_ch));
	    			w55fa92_edma_trigger_done(vdma_ch);
			#endif	
					g_osdunlock_cmd_sent = 2;
				}			

				#ifdef OPT_COPY_OSD_AND_FB
				//-------------------------------------------------
				// check whether FB is dirty or not
				//-------------------------------------------------				
					#ifdef CONFIG_W55FA92_PINGPONG_SUPPORT
						if (g_fbiopan_cmd_sent == 1)
						{
							if(g_var_yoffset == g_yres)				
								g_bg_mmp_v = _bg_mem_v+g_map_size;					
							else
								g_bg_mmp_v = _bg_mem_v;

				    	    memcpy(_dma_dst_v, g_bg_mmp_v, g_map_size); 
							flush_cache_all();    	    
							g_fbiopan_cmd_sent = 0;						
						}						
					#endif
					
				  	#ifdef OPT_FB_LOCK	
						if(g_fbunlock_cmd_sent == 1)				
						{
				    	    memcpy(_dma_dst_v, _bg_mem_v, g_map_size);
							flush_cache_all();    	    
							g_fbunlock_cmd_sent = 0;						
						}						
					#endif						

				#endif	//OPT_COPY_OSD_AND_FB
				
        		goto VDMA_EXIT;				
			}				
	
	#endif	// CONFIG_TWO_OSD_BUF


	#ifdef CONFIG_W55FA92_PINGPONG_SUPPORT

			if (g_fbiopan_cmd_sent == 1)
			{
				if(g_var_yoffset == g_yres)				
					g_bg_mmp_p = _bg_mem_p+g_map_size;					
				else
					g_bg_mmp_p = _bg_mem_p;

		#ifndef CONFIG_W55FA92_TV_LCD_SWITCH					
		
			#ifndef CONFIG_TWO_OSD_BUF		    
					outl(_bg_mem_p+g_map_size*2, REG_LCM_OSD_ADDR);		
				
				#ifdef CONFIG_W55FA92_OSD_SUPPORT					
			        osd_cpu_mmap=(unsigned long)_bg_mem_v + g_map_size*2;
			        osd_dma_mmap=(unsigned long)_bg_mem_p + g_map_size*2;
				#endif		    
			
			#else
		        osd_cpu_mmap=(unsigned long)video_cpu_mmap + g_map_size;
		        osd_dma_mmap=(unsigned long)video_dma_mmap + g_map_size;
				outl(osd_dma_mmap, REG_LCM_OSD_ADDR);									    
			#endif				
		#endif				

			}				
	#endif	// CONFIG_W55FA92_PINGPONG_SUPPORT
		#ifdef OPT_FB_LOCK
			if (g_fb_cmd_flag)
			{
				if(g_fbunlock_cmd_sent == 1)				
				{
					g_fbunlock_cmd_sent = 2;
					g_bg_mmp_p = _bg_mem_p;											
				}						
				else if (!changeMode)
					goto VDMA_EXIT;																					
			}
			else
			{
			#ifdef CONFIG_W55FA92_PINGPONG_SUPPORT			
//        	printk("g_fbiopan_cmd_sent = 0x%x !!!\n", g_fbiopan_cmd_sent);        	
					if ((g_fbiopan_cmd_sent==2) && (!changeMode)) 
					//	g_bg_mmp_p = _bg_mem_p;											
						goto VDMA_EXIT;
					else if (g_fbiopan_cmd_sent==0)					
						g_bg_mmp_p = _bg_mem_p;											
			#else
					g_bg_mmp_p = _bg_mem_p;				
			#endif
			}
		#else	// OPT_FB_LOCK
			#ifdef CONFIG_W55FA92_PINGPONG_SUPPORT			
					if ((g_fbiopan_cmd_sent==2) && (!changeMode)) 
						goto VDMA_EXIT;
					else if (g_fbiopan_cmd_sent==0)					
						g_bg_mmp_p = _bg_mem_p;											
			#else
					g_bg_mmp_p = _bg_mem_p;				
			#endif
		#endif	// OPT_FB_LOCK		
			
			
            if (changeMode==1) {
				if ( vdma_ch_flag != -1 )
					goto VDMA_EXIT;

	             // trigger VDMA to do format transfer from on-screen to off-screen frame buffer
		        vdma_ch = w55fa92_vdma_find_and_request("w55fa92-VPOST");			
	       	    if( (vdma_ch != 0)&&(vdma_ch != 5) &&(vdma_ch != 8) ) {
	        		printk("Request VDMA fail.\n");
	        		goto VDMA_EXIT;
		        }
				else
				{
					vdma_ch_flag = vdma_ch;
       	 			w55fa92_edma_setup_cst(vdma_ch, edma_format[currentMode], edma_format[destMode]);

		#ifdef CONFIG_W55FA92_TV_LCD_SWITCH	
			#ifdef CONFIG_W55FA92_TV_QVGA
    				ret = w55fa92_edma_setup_single(vdma_ch, _dma_dst, g_bg_mmp_p, _dma_len);    										
			#else
					if (_vpost_int == LCDCInt_VINT)
    					ret = w55fa92_edma_setup_single(vdma_ch, _dma_dst, g_bg_mmp_p, _dma_len);    										
					else
    					ret = w55fa92_edma_setup_single(vdma_ch, osd_dma_mmap, g_bg_mmp_p, _dma_len);    										    				
			#endif    				
		#else
	    			ret = w55fa92_edma_setup_single(vdma_ch, _dma_dst, g_bg_mmp_p, _dma_len);    										    			
		#endif    			    	        
		
	    	        if (ret < 0) {
	    	        	printk("w55fa92_edma_setup_single failed and returns %d\n", ret);
	    	        	goto VDMA_FREE;
	    	        }

					flush_cache_all();
					#ifdef EDMA_USE_IRQ
								ret = w55fa92_edma_setup_handlers(vdma_ch, 2, (void*)fb_edma_irq_handler, NULL);
				    	        if (ret < 0) {
				    	        	printk("w55fa92_edma_setup_handlers failed and returns %d\n", ret);
				    	        	goto VDMA_FREE;
				    	        }

								ret = w55fa92_edma_setup_handlers(vdma_ch, 1, (void*)fb_edma_abort_irq_handler, NULL);
				    	        if (ret < 0) {
				    	        	printk("w55fa92_edma_abort_setup_handlers failed and returns %d\n", ret);
				    	        	goto VDMA_FREE;
				    	        }
					#endif
					changeMode = 2;           									
	    			w55fa92_edma_trigger(vdma_ch);

		#if !defined(EDMA_USE_IRQ)
					
					// wait format transfer is complete	    			
		            while (DrvEDMA_IsCHBusy(vdma_ch));
	    			w55fa92_edma_trigger_done(vdma_ch);

					// restore to RGB565 format                    
		 //       	w55fa92_edma_setup_cst(vdma_ch, eDRVEDMA_RGB565, eDRVEDMA_RGB565);                    
      				w55fa92_edma_clear_cst(vdma_ch);            	             		 
	   	   			w55fa92_edma_free(vdma_ch);          
					vdma_ch_flag = -1;
					currentMode = destMode;       	   			
					changeMode = 3;
		#endif	    			
	    		}
                    
        		goto VDMA_EXIT;

        	}	
        	else if (changeMode==2)
        		goto VDMA_EXIT;    
        		    	
			if ( vdma_ch_flag != -1 )
				goto VDMA_EXIT;

             // trigger VDMA to copy off-screen frame buffer to on-screen frame buffer
	        vdma_ch = w55fa92_vdma_find_and_request("w55fa92-VPOST");
       	    if( (vdma_ch != 0)&&(vdma_ch != 5) &&(vdma_ch != 8) ) {
        		printk("Request VDMA fail.\n");
        		goto VDMA_EXIT;
	        }
			else
			{
      			vdma_ch_flag = vdma_ch;
//        		printk("vdma_ch = 0x%x !!!\n", vdma_ch);					
      						
		#ifdef CONFIG_W55FA92_TV_LCD_SWITCH	
			#ifdef CONFIG_W55FA92_TV_QVGA
    				ret = w55fa92_edma_setup_single(vdma_ch, g_bg_mmp_p, _dma_dst, _dma_len);    										
			#else
				if (_vpost_int == LCDCInt_VINT)
    				ret = w55fa92_edma_setup_single(vdma_ch, g_bg_mmp_p, _dma_dst, _dma_len);    							
				else
    				ret = w55fa92_edma_setup_single(vdma_ch, g_bg_mmp_p, osd_dma_mmap, _dma_len);
			#endif    				
		#else
    			ret = w55fa92_edma_setup_single(vdma_ch, g_bg_mmp_p, _dma_dst, _dma_len);
		#endif    			    	        
		
    	        if (ret < 0) {
    	        	printk("w55fa92_edma_setup_single failed and returns %d\n", ret);
    	        	goto VDMA_FREE;
    	        }
		
		#ifdef EDMA_USE_IRQ
				ret = w55fa92_edma_setup_handlers(vdma_ch, 2, (void*)fb_edma_irq_handler, NULL);
    	        if (ret < 0) {
    	        	printk("w55fa92_edma_setup_handlers failed and returns %d\n", ret);
    	        	goto VDMA_FREE;
    	        }
				ret = w55fa92_edma_setup_handlers(vdma_ch, 1, (void*)fb_edma_abort_irq_handler, NULL);
    	        if (ret < 0) {
    	        	printk("w55fa92_edma_abort_setup_handlers failed and returns %d\n", ret);
    	        	goto VDMA_FREE;
    	        }
		#endif
				flush_cache_all();
    			w55fa92_edma_trigger(vdma_ch);
    			
           		if (changeMode==3) {    		
           			outl((inl(REG_LCM_LCDCCtl) & ~0xE) | ((destMode&0x07)<<1), REG_LCM_LCDCCtl);	          	           			
           			changeMode = 0;
            	    wake_up_interruptible(&wq);           			
				}           			
    			
		#ifdef EDMA_USE_IRQ
        		goto VDMA_EXIT;
		#else
	            while (DrvEDMA_IsCHBusy(vdma_ch));
	                        ;
    			//w55fa92_edma_disable(VDMA_CH);
    			w55fa92_edma_trigger_done(vdma_ch);
		#endif	
    		}
VDMA_FREE:
   	   w55fa92_edma_free(vdma_ch);  
	   vdma_ch_flag = -1;
	} 
#else 
	}
VDMA_FREE:
#endif
 
VDMA_EXIT:


#ifdef CONFIG_W55FA92_TV_LCD_SWITCH
        if (! _auto_disable)	//### Chris
        {	
                outl((inl(REG_LCM_LCDCInt) & 0xFFFFFF00) | (_vpost_int << 16), REG_LCM_LCDCInt);
    	}
    	else{
                outl((inl(REG_LCM_LCDCInt) & 0xFFFFFF00) & ~(_vpost_int << 16), REG_LCM_LCDCInt); // disable VIN
		}
#else
        if (! _auto_disable)	//### Chris
        {	
#if TV_OUTPUT_DEVICE        	
                outl((inl(REG_LCM_LCDCInt) & 0xFFFFFF00) | 0x40000, REG_LCM_LCDCInt);
#else
               outl((inl(REG_LCM_LCDCInt) & 0xFFFFFF00) | 0x20000, REG_LCM_LCDCInt);
#endif                
    	}
    	else{
//        		printk("### Disable VIN: lcdirq=0x%x, disable=%d###\n",lcdirq,_auto_disable);	//## Chris
#if TV_OUTPUT_DEVICE
                outl((inl(REG_LCM_LCDCInt) & 0xFFFFFF00) & ~0x40000, REG_LCM_LCDCInt); // disable VIN
#else
                outl((inl(REG_LCM_LCDCInt) & 0xFFFFFF00) & ~0x20000, REG_LCM_LCDCInt); // disable VIN
#endif                
		}
#endif

	#ifdef CONFIG_W55FA92_PINGPONG_SUPPORT
		if (g_fbiopan_cmd_sent == 1)
			g_fbiopan_cmd_sent = 2;

		if (_auto_disable)
			g_fbiopan_cmd_sent = 0;		// work around for "w55fa92fb_pan_display" to be run one time
	#endif			


//	LEAVE();

        return IRQ_HANDLED;
}
static char driver_name[] = "w55fa92fb";

static int __devinit w55fa92fb_probe(struct platform_device *pdev)
{
	struct w55fa92fb_info *info;
	struct fb_info	   *fbinfo;
	static struct w55fa92fb_mach_info *mach_info;
	struct w55fa92fb_hw *mregs;	
	unsigned long page; /* For LCD page reserved */
	int ret;
	int irq;
	int i;

	ENTER();
	
	printk("###########w55fa92 frame buffer probe############\n");

 	mach_info = pdev->dev.platform_data;
	if (mach_info == NULL) {
		dev_err(&pdev->dev,"no platform data for lcd, cannot attach\n");
		return -EINVAL;
	}
	
	mregs = &mach_info->regs;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0){
		dev_err(&pdev->dev, "no irq for device\n");
		return -ENOENT;
	}

	fbinfo = framebuffer_alloc(sizeof(struct w55fa92fb_info), &pdev->dev);
	if (!fbinfo){
		return -ENOMEM;
	}
	
	info = fbinfo->par;
	info->fb = fbinfo;
	platform_set_drvdata(pdev, fbinfo);
	strcpy(fbinfo->fix.id, driver_name);

	memcpy(&info->regs, &mach_info->regs, sizeof(info->regs));
	
	info->mach_info		  		  	= pdev->dev.platform_data;

	fbinfo->fix.type	   				= FB_TYPE_PACKED_PIXELS;
	fbinfo->fix.type_aux	    	= 0;
	fbinfo->fix.xpanstep	    	= 0;
	fbinfo->fix.ypanstep	    	= 0;
	fbinfo->fix.ywrapstep	    	= 0;
	fbinfo->fix.accel	    			= FB_ACCEL_NONE;

	fbinfo->var.nonstd	   			= 0;
	fbinfo->var.activate	    	= FB_ACTIVATE_NOW;
	fbinfo->var.height	    		= mach_info->height;
	fbinfo->var.width	    			= mach_info->width;
	fbinfo->var.accel_flags   	= 0;
	fbinfo->var.vmode	   		 		= FB_VMODE_NONINTERLACED;

	fbinfo->fbops		    				= &w55fa92fb_ops;
	fbinfo->flags		    				= FBINFO_FLAG_DEFAULT;
	fbinfo->pseudo_palette  	  = &info->pseudo_pal;

	fbinfo->var.xres	    			= mach_info->xres.defval;
	fbinfo->var.xres_virtual  	= mach_info->xres.defval;
	fbinfo->var.yres	    			= mach_info->yres.defval;
#ifdef CONFIG_W55FA92_PINGPONG_SUPPORT	
	fbinfo->var.yres_virtual  	= 2*mach_info->yres.defval;
	fbinfo->fix.ypanstep	    = 1;	
#else	
	fbinfo->var.yres_virtual  	= mach_info->yres.defval;
#endif	
	fbinfo->var.bits_per_pixel	= mach_info->bpp.defval;

	fbinfo->var.upper_margin    = mach_info->upper_margin;
	fbinfo->var.lower_margin    = mach_info->lower_margin;
	fbinfo->var.vsync_len	    	= mach_info->vsync_len;

	fbinfo->var.left_margin	    = mach_info->left_margin;
	fbinfo->var.right_margin    = mach_info->right_margin;
	fbinfo->var.hsync_len	    	= mach_info->hsync_len;

	fbinfo->var.red.offset      = 11;
	fbinfo->var.green.offset    = 5;
	fbinfo->var.blue.offset     = 0;
	fbinfo->var.transp.offset   = 0;
	fbinfo->var.red.length      = 5;
	fbinfo->var.green.length    = 6;
	fbinfo->var.blue.length     = 5;
	fbinfo->var.transp.length   = 0;

#ifdef CONFIG_W55FA92_PINGPONG_SUPPORT
	#ifdef CONFIG_W55FA92_OSD_SUPPORT
		fbinfo->fix.smem_len        =	mach_info->xres.max *
					      				mach_info->yres.max *
					      				mach_info->bpp.max / 8 * 3;
	#else				      				
		fbinfo->fix.smem_len        =	mach_info->xres.max *
					      				mach_info->yres.max *
					      				mach_info->bpp.max / 8 * 2;
	#endif
	
	g_yres = mach_info->yres.max;
	
#else
	#ifdef CONFIG_W55FA92_OSD_SUPPORT
		fbinfo->fix.smem_len        =	mach_info->xres.max *
					      				mach_info->yres.max *
					      				mach_info->bpp.max / 8 * 2;
	#else				      				
		fbinfo->fix.smem_len        =	mach_info->xres.max *
					      				mach_info->yres.max *
					      				mach_info->bpp.max / 8;
	#endif
#endif	

	video_alloc_len = fbinfo->fix.smem_len;
	
	for (i = 0; i < 256; i++)
		info->palette_buffer[i] = PALETTE_BUFF_CLEAR;

	if (!request_mem_region((unsigned long)W55FA92_VA_VPOST, SZ_4K, "w55fa92-lcd")){
		ret = -EBUSY;
		goto dealloc_fb;
	}
   if ( clk_hclk4 == NULL )
   {
		clk_hclk4 = clk_get(NULL, "hclk4");
        if (IS_ERR(clk_hclk4)) {
            ret = PTR_ERR(clk_hclk4);
            goto release_mem;       	
		}                
   }

   if ( clk_edma == NULL )
   {
		clk_edma = clk_get(NULL, "edma0");
        if (IS_ERR(clk_edma)) {
            ret = PTR_ERR(clk_edma);
            goto release_mem;       	
		}                
   }
    info->clk = clk_get(&pdev->dev, NULL);
    if (IS_ERR(info->clk)) {
            ret = PTR_ERR(info->clk);
            goto release_mem;       	
    }

  	// 2.Enable IP!|s clock
    clk_enable(clk_hclk4);
    clk_enable(info->clk);

#ifdef CONFIG_W55FA92_FB_INIT

  	// 3.Reset IP
	outl(VPOST_RST, REG_AHBIPRST);
	outl(0, REG_AHBIPRST);
	
	outl((inl(REG_LCM_LCDCInt) & 0xFFFFFF00) | 0x20000, REG_LCM_LCDCInt); // enable VIN /* SW add for bg/fp */
#endif

#ifdef	CONFIG_KD070D10_800x480_16B
      	w55fa92fb_ioctl(0, VIDEO_DISPLAY_ON, 00);			// backlight is ON
#endif
	
#ifdef CONFIG_TWO_FB_BUF  
    	clk_enable(clk_edma);				// enable EDMA clock
#endif        
	
	 ret = request_irq(irq, w55fa92fb_irq, IRQF_DISABLED, pdev->name, info);
	if (ret) {
		dev_err(&pdev->dev, "cannot get irq %d - err %d\n", irq, ret);
		ret = -EBUSY;
		goto release_mem;
	}

	msleep(1);

	/* Initialize video memory */
	ret = w55fa92fb_map_video_memory(info);
	if (ret) {
		ret = -ENOMEM;

	}
	/* video & osd(??) buffer together */	
	for (page = (unsigned long)video_cpu_mmap; 
		       page < PAGE_ALIGN((unsigned long)ret + video_buf_mmap/*g_LCDWholeBuffer*/);
		       page += PAGE_SIZE){
           SetPageReserved(virt_to_page(page));
	 }	
	
#ifdef CONFIG_W55FA92_FB_INIT
	ret = w55fa92fb_init_registers(info);
#endif

#ifdef 	CONFIG_W55FA92_SETPWM
	w55fa92fb_init_pwm();
#endif

	ret = w55fa92fb_check_var(&fbinfo->var, fbinfo);
	ret = w55fa92fb_set_par(fbinfo);
	ret = register_framebuffer(fbinfo);
	if (ret < 0) {
		printk(KERN_ERR "Failed to register framebuffer device: %d\n", ret);
		goto free_video_memory;
	}

    w55fa92fb_probe_device();
    
	/* create device files */
	//device_create_file(&pdev->dev, &dev_attr_debug);

	w55fa92fb_set_lcdaddr(info);
	
#ifdef CONFIG_W55FA92_TV_LCD_SWITCH
	#ifndef CONFIG_W55FA92_TV_QVGA
		change_format(VA_2_OSD);
	#endif
#endif
#ifdef CONFIG_TV_DETECT_GPD3

        init_timer(&tv_timer);

        outl(inl(REG_GPDFUN) & ~MF_GPD3, REG_GPDFUN);				// headset detect plug IN/OUT
        outl(inl(REG_GPIOD_OMD) & ~(0x0008), REG_GPIOD_OMD); 		// port D3 input
        outl(inl(REG_GPIOD_PUEN) | (0x0008), REG_GPIOD_PUEN); 		// port D3 pull-up

		if (! (inl(REG_GPIOD_PIN) & 0x0008)	) 	// TV plug_out
		{
	       	_IsTVmode = 0;		
			display_to_lcd();
	       	w55fa92fb_ioctl(0, VIDEO_DISPLAY_ON, 00);			// backlight is ON			
#if defined(CONFIG_W55FA92_SYSMGR) || defined (CONFIG_W55FA92_SYSMGR_MODULE)	       	
            sysmgr_report(SYSMGR_STATUS_DISPLAY_LCD);						
#endif            
		}		
		else								// headset plug_in		
		{
	       	_IsTVmode = 1;
			display_to_tv();
	       	w55fa92fb_ioctl(0, VIDEO_DISPLAY_OFF, 00);			// backlight is OFF			
#if defined(CONFIG_W55FA92_SYSMGR) || defined (CONFIG_W55FA92_SYSMGR_MODULE)	       	
            sysmgr_report(SYSMGR_STATUS_DISPLAY_TV);						
#endif            
		}			

        outl(inl(REG_IRQTGSRC1) & 0x00080000, REG_IRQTGSRC1);               
        outl((inl(REG_IRQSRCGPD) & ~(0x000000c0)) | 0x00000080, REG_IRQSRCGPD); // port D3 as nIRQ2 source
        outl(inl(REG_IRQENGPD) | 0x00080008, REG_IRQENGPD); 		// falling/rising edge trigger
        
        outl(0x10,  REG_AIC_SCCR); // force clear previous interrupt, 

//        printk("register the headset_detect_irq\n");
	    if (request_irq(TV_IRQ_NUM, tv_detect_irq, IRQF_DISABLED, "FA92_tv_DETECT", NULL) != 0) {
	            printk("register the tv_detect_irq failed!\n");
	            return -1;
	    }
	    
        Enable_IRQ(TV_IRQ_NUM);	    
        
		//outl((1<<4), REG_AIC_MECR);		        
#endif		
	
	printk("w55fa92 LCD driver has been installed successfully\n");

	return 0;

free_video_memory:
	w55fa92fb_unmap_video_memory(info);
release_mem:
 	release_mem_region((unsigned long)W55FA92_VA_VPOST, W55FA92_SZ_VPOST);
dealloc_fb:
	framebuffer_release(fbinfo);

	LEAVE();

	return ret;

	
} 

/* 
 *			w55fa92fb_stop_lcd
 *
 * 			shutdown the lcd controller
 */
static void w55fa92fb_stop_lcd(void)
{
	unsigned long flags;

	local_irq_save(flags);

	w55fa92fb_stop_device();

	local_irq_restore(flags);
}

/*
 *  Cleanup
 */
static int w55fa92fb_remove(struct platform_device *pdev)
{
	struct fb_info *fbinfo = platform_get_drvdata(pdev);
	struct w55fa92fb_info *info = fbinfo->par;
	int irq;

	ENTER();

	w55fa92fb_stop_lcd();
	msleep(1);

	w55fa92fb_unmap_video_memory(info);

	irq = platform_get_irq(pdev, 0);
	free_irq(irq,info);
	release_mem_region((unsigned long)W55FA92_VA_VPOST, W55FA92_SZ_VPOST);
	unregister_framebuffer(fbinfo);

	    clk_disable(info->clk);
	    clk_disable(clk_hclk4);                
    
#ifdef CONFIG_TWO_FB_BUF  
    	clk_disable(clk_edma);				// disable EDMA clock
		clk_put(clk_edma);
#endif        
    clk_put(info->clk);
    clk_put(clk_hclk4);                

	LEAVE();

	return 0;
}

#ifdef CONFIG_PM

/* suspend and resume support for the lcd controller */

static int w55fa92fb_suspend(struct platform_device *dev, pm_message_t state)
{
	struct fb_info	   *fbinfo = platform_get_drvdata(dev);
	struct w55fa92fb_info *info = fbinfo->par;

	w55fa92fb_stop_lcd();
	msleep(1);
	clk_disable(info->clk);
	return 0;
}

static int w55fa92fb_resume(struct platform_device *dev)
{
	struct fb_info	   *fbinfo = platform_get_drvdata(dev);
	struct w55fa92fb_info *info = fbinfo->par;

	clk_enable(info->clk);
	msleep(1);

	w55fa92fb_init_registers(info);

	return 0;
}

#else
#define w55fa92fb_suspend NULL
#define w55fa92fb_resume  NULL
#endif

static struct platform_driver w55fa92fb_driver = {
	.driver		= {
		.name	= "w55fa92-lcd",
		.owner	= THIS_MODULE,
	},

	.probe		= w55fa92fb_probe,
	.remove		= __devexit_p(w55fa92fb_remove),   //w55fa92fb_remove,
//	.suspend	= w55fa92fb_suspend,
//	.resume		= w55fa92fb_resume,
};

static int __init w55fa92fb_init(void)
{
	/*set up w55fa92 register*/
	printk("---w55fa92fb_init ----w55fa92 frame buffer init \n");
	w55fa92_fb_set_platdata(&w55fa92_lcd_platdata);
	return platform_driver_register(&w55fa92fb_driver);
}

static void __exit w55fa92fb_cleanup(void)
{
	platform_driver_unregister(&w55fa92fb_driver);
}

module_init(w55fa92fb_init);
module_exit(w55fa92fb_cleanup);

MODULE_DESCRIPTION("Framebuffer driver for the W55FA92");
MODULE_LICENSE("GPL"); 

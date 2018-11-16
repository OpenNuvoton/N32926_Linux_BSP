struct w55fa92fb_mach_info w55fa92_lcd_platdata = {
			.width	= LCDWIDTH,
			.height	= LCDWIDTH,

.xres = {
			.defval	= LCDWIDTH,
			.min		= LCDWIDTH,
			.max		= LCDWIDTH,
		},
		
.yres = {
			.defval	= LCDHEIGHT,
			.min		= LCDHEIGHT,
			.max		= LCDHEIGHT,
},

.bpp = {
			.defval	= LCDBPP,
			.min		= LCDBPP,
			.max		= LCDBPP,
},


			hsync_len   :  64, 
			vsync_len    :  6,
			left_margin :  125, 
			upper_margin :  70,
			right_margin:  115,  
			lower_margin :  36,
			sync:		0,		
			cmap_static:	0,

};

extern unsigned int w55fa92_upll_clock;
extern unsigned int w55fa92_apll_clock;

//=====================================================	
// I2C interface declaration
//=====================================================		
#include <linux/i2c.h>
struct cat3626_priv {
	struct i2c_msg		msg[4];
	struct i2c_client	*client;
};
	
static struct i2c_client *save_client;
static struct i2c_client i2c_client; 
static int cat3626_probe(struct i2c_client *client,
			const struct i2c_device_id *did)
{
	struct cat3626_priv	*cat3626;	
	int ret = 0;
	cat3626 = kzalloc(sizeof(struct cat3626_priv), GFP_KERNEL);
	
	if (!cat3626)
	{
		printk("i2c allocate fail !!!\n");
		return -ENOMEM;
	}		

	cat3626->client = client;
	i2c_set_clientdata(client, cat3626);
	save_client = client;
	return ret;
}

static int cat3626_remove(struct i2c_client *client)
{
	struct cat3626_priv *cat3626 = i2c_get_clientdata(client);
	kfree(cat3626);
	
	return 0;
}

static const struct i2c_device_id cat3626_id[] = {

	{ "cat3626", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, cat3626_id);

static struct i2c_driver cat3626_i2c_driver = {
	.driver = {
		.name = "cat3626",
	},
	.probe    = cat3626_probe,
	.remove   = cat3626_remove,
	.id_table = cat3626_id,
};

static int __init i2c_init(void)
{
	int ret;

	ENTER();
	ret = i2c_add_driver(&cat3626_i2c_driver);
//	if(ret)
//		ERRLEAVE();
//	else
//		LEAVE();
	return ret;
}
static void __exit i2c_cleanup(void)
{
	ENTER();
	i2c_del_driver(&cat3626_i2c_driver);
	LEAVE();
}

MODULE_LICENSE("GPL");
module_init(i2c_init);
module_exit(i2c_cleanup);

#ifdef CONFIG_w55fa92_FB_INIT 

	static int w55fa92fb_mpu_trigger(void)
	{
		#if 0	
			printk("Single trigger MPU !!!\n");
			
			// single trigger mode	
			outl(inl(REG_LCM_MPUCMD)&~(MPUCMD_CMD_DISn|MPUCMD_MPU_ON|MPUCMD_MPU_CS|MPUCMD_MPU_RWn), REG_LCM_MPUCMD);
			outl(inl(REG_LCM_MPUCMD)|MPUCMD_WR_RS|MPUCMD_MPU_CS, REG_LCM_MPUCMD);						
			outl(inl(REG_LCM_MPUCMD)&~MPUCMD_DIS_SEL, REG_LCM_MPUCMD);		// select Single Mode
			outl(inl(REG_LCM_MPUCMD)&~MPUCMD_CMD_DISn, REG_LCM_MPUCMD);		// turn on Display Mode			
			outl(inl(REG_LCM_MPUCMD)|MPUCMD_MPU_ON, REG_LCM_MPUCMD);		// trigger command output		
	
		#else
			printk("Continue trigger MPU !!!\n");

			// continue trigger mode
			outl(inl(REG_LCM_MPUCMD)&~(MPUCMD_CMD_DISn|MPUCMD_MPU_ON|MPUCMD_MPU_CS|MPUCMD_MPU_RWn), REG_LCM_MPUCMD);
			outl(inl(REG_LCM_MPUCMD)|MPUCMD_WR_RS|MPUCMD_MPU_CS, REG_LCM_MPUCMD);			
			outl(inl(REG_LCM_MPUCMD)|MPUCMD_DIS_SEL, REG_LCM_MPUCMD);		// select Continue Mode
			outl(inl(REG_LCM_MPUCMD)&~MPUCMD_CMD_DISn, REG_LCM_MPUCMD);		// turn on Display Mode			
			outl(inl(REG_LCM_MPUCMD)|MPUCMD_MPU_ON, REG_LCM_MPUCMD);		// trigger command output		
		#endif
		return 0;
	}

	static int w55fa92fb_init_device(struct w55fa92fb_info *fbi)
	{
		unsigned int clock_div;
		
	  	// Reset IP
		outl(inl(REG_AHBIPRST) | VPOSTRST, REG_AHBIPRST);
		outl(inl(REG_AHBIPRST) & ~VPOSTRST, REG_AHBIPRST);	
	
		clock_div = w55fa92_upll_clock / 40000;
		clock_div &= 0xFF;
		clock_div --;
		
		// given clock divider for VPOST 
		outl((inl(REG_CLKDIV1) & ~VPOST_N0) | 0, REG_CLKDIV1);					
		outl((inl(REG_CLKDIV1) & ~VPOST_N1) | (clock_div << 8), REG_CLKDIV1);	
		
	//	outl((inl(REG_CLKDIV1) & ~VPOST_S), REG_CLKDIV1);				// VPOST clock from UPLL
		outl((inl(REG_CLKDIV1) & ~VPOST_S) | (3<<3), REG_CLKDIV1);		// VPOST clock from UPLL
	
		// configure LCD interface  // enable sync with TV, LCD type select 
	   	outl(inl(REG_LCM_LCDCPrm)& ~LCDCPrm_LCDSynTv, REG_LCM_LCDCPrm);			// async with TV
	   	outl((inl(REG_LCM_LCDCPrm)& ~LCDCPrm_LCDTYPE) | 0x03, REG_LCM_LCDCPrm);	// MPU-type LCD
		/*
		0x0  // High Resolution mode
		0x1  // Sync-type TFT LCD
		0x2  // Sync-type Color STN LCD
		0x3  // MPU-type LCD
		*/

	//	ILITEK_ILI9341_MPU_Init();

		// enable VPOST 8-bit pins 
		outl(inl(REG_GPBFUN) | 0xC0000000, REG_GPBFUN);		// enable LPCLK pin
		outl(inl(REG_GPCFUN) | 0x0000FFFF, REG_GPCFUN);		// enable LVDATA[7:0] pins
		outl(inl(REG_GPDFUN) | 0x00FC0000, REG_GPDFUN);		// enable HSYNC/VSYNC/VDEN pins	
		
		// async TV 
		outl((inl(REG_LCM_LCDCPrm) & (~LCDCPrm_LCDSynTv)), REG_LCM_LCDCPrm);		
		
		// MPU 8+8+8 mode
		outl((inl(REG_LCM_MPUCMD) & ~MPUCMD_MPU_SI_SEL) | 0x03, REG_LCM_MPUCMD);	// MPU 8+8+8 mode			
		
		// MPU timing
		outl(0x01010201, REG_LCM_MPUTS);								// MPU timing
		
		// set both "active pixel per line" and "active lines per screen" for Syn type LCD   
	   	outl(0x00EF013F, REG_LCM_TCON3);	// 240x320
   		outl(0x00EF013F, REG_LCM_FB_SIZE);  // 240x320   	   		   	
	   	outl(0x00F00100, REG_LCM_TCON4);	// 480
	   		
		// set TV control register and LCD from frame buffer source
	   	outl((inl(REG_LCM_TVCtl)& 0xFFFFF0DA)|(0x00000410), REG_LCM_TVCtl);   // DAC disable	
	   	
	#ifdef CONFIG_RGBx888_FORMAT
		// enable LCD controller
		outl((inl(REG_LCM_LCDCCtl)& 0xFFFEFFF0)|(0x10005), REG_LCM_LCDCCtl);	// RGBx888 format
	#else   	
		// enable LCD controller
		outl((inl(REG_LCM_LCDCCtl)& 0xFFFEFFF0)|(0x10003), REG_LCM_LCDCCtl);	// RGBx565 format
	#endif	

		// trigger in Continue Mode
		w55fa92fb_mpu_trigger();	
		return 0;
	}
#endif	


static int w55fa92fb_ioctl_device(struct fb_info *info, unsigned int cmd, unsigned long arg)
{
	unsigned int buffer[5];
	unsigned int clock_div;	
	static char led_reg[4] = {0,0,0,0};
	
	memset(buffer,0,5);
	switch(cmd)
	{
		case IOCTLCLEARSCREEN:	
			memset((void *)video_cpu_mmap, 0xff, video_alloc_len); 		
			break;

		case VIDEO_DISPLAY_ON:
			// control EN/DISABLE singal	
//	    	outl(inl(REG_GPIOA_DOUT) | 0x80, REG_GPIOA_DOUT);    
//	    	mdelay(100);

		//	CAT3626_WriteData(0x03, 0x3f);			
			break;
			
		case VIDEO_DISPLAY_OFF:
//	    	outl(inl(REG_GPIOA_DOUT) & ~0x80, REG_GPIOA_DOUT);    
			break;
			
		case IOCTL_LCD_BRIGHTNESS:
			// Need to implement PWM backlight control here !!!
			copy_from_user((unsigned int *)buffer, (void *)arg, 1*sizeof(unsigned int));
			
//			if (buffer[0]>=63)
//				buffer[0] = 63;

			break;

		case VIDEO_DISPLAY_LCD:
			printk("video displayed by LCD only\n");
			break;				


		case VIDEO_DISPLAY_TV:
			printk("video displayed by TV only\n");
			break;				


		case IOCTL_LCD_ENABLE_INT:
        	outl((inl(REG_LCM_LCDCInt) & 0xFFFFFF00) | 0x100000, REG_LCM_LCDCInt); // enable VIN
	        //_auto_disable = 0;
            break;
                
		case IOCTL_LCD_DISABLE_INT:
	    	outl((inl(REG_LCM_LCDCInt) & 0xFFFFFF00) & ~0x100000, REG_LCM_LCDCInt); // disable VIN
#ifdef CONFIG_TWO_FB_BUF
			if ( vdma_ch_flag != -1 )
         		 while (w55fa92_edma_isbusy(vdma_ch));
#endif            
            _auto_disable = 1;
            break;

#ifdef CONFIG_TWO_FB_BUF
		case IOCTL_LCD_GET_DMA_BASE:
		  if(copy_to_user((unsigned int *)arg, (unsigned int *)&_bg_mem_p, sizeof(unsigned int)) < 0) {
		    printk("failed..\n");
		    return(-EFAULT);
		  }
			printk("!!!%x\n", *(unsigned int *)&_bg_mem_p);
			break;
#endif            
		case DUMP_LCD_REG:		//ken add
		{
			unsigned int reg_array[5];
			reg_array[0] = inl(REG_LCM_TCON1);
			reg_array[1] = inl(REG_LCM_TCON2);
			reg_array[2] = inl(REG_LCM_TCON3);
			reg_array[3] = inl(REG_LCM_TCON4);
			reg_array[4] = inl(REG_LCM_LCDCCtl);
			if(copy_to_user((unsigned int *)arg, (unsigned int *)&reg_array, 5*sizeof(unsigned int)) < 0) {
		    		printk("failed..\n");
		    		return(-EFAULT);
		  	}	
		}		
			break;
		default:
			break;	
	}
		
	return 0;    
}

static int w55fa92fb_blank_device(int blank_mode, struct fb_info *info)
{
    return 0;
}


void w55fa92fb_init_pwm_device(void)
{
	w55fa92fb_set_pwm_channel(PWM0);
}

static int w55fa92fb_probe_device(void)
{
	outl((inl(REG_LCM_LCDCInt) & 0xFFFFFF00) | 0x100000, REG_LCM_LCDCInt); // enable VINTEN
	
#ifdef CONFIG_RGBx888_FORMAT
	/* enable lcd*/
	outl((inl(REG_LCM_LCDCCtl)& 0xFFFEFFF0)|(0x10005), REG_LCM_LCDCCtl);
#else	
	/* enable lcd*/
	outl((inl(REG_LCM_LCDCCtl)& 0xFFFEFFF0)|(0x10003), REG_LCM_LCDCCtl);
#endif   
	
	/* MPU continue trigger */   
	outl(inl(REG_LCM_LCDCCtl) | LCDCCtl_LCDRUN, REG_LCM_LCDCCtl); 	//va-enable
	outl(inl(REG_LCM_MPUCMD) & ~(MPUCMD_CMD_DISn|MPUCMD_MPU_ON|MPUCMD_MPU_CS|MPUCMD_MPU_RWn), REG_LCM_MPUCMD);			
	outl(inl(REG_LCM_MPUCMD) | MPUCMD_DIS_SEL, REG_LCM_MPUCMD);		// select Continue Mode
	outl(inl(REG_LCM_MPUCMD) & ~MPUCMD_CMD_DISn, REG_LCM_MPUCMD);	// turn on Display Mode		
	outl(inl(REG_LCM_MPUCMD) | MPUCMD_MPU_ON, REG_LCM_MPUCMD);		// trigger command output		
   	return 0;
}

static void w55fa92fb_stop_device(void)
{
	outl((inl(REG_LCM_LCDCCtl)& 0xFFFEFFFE), REG_LCM_LCDCCtl);
}


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



static int w55fa92fb_init_device(struct w55fa92fb_info *fbi)
{
	unsigned int clock_div;
	
  	// Reset IP
	outl(VPOST_RST, REG_AHBIPRST);
	outl(0, REG_AHBIPRST);	

//	printk("w55fa92_upll_clock = 0x%x\n", w55fa92_upll_clock);		   	

//	w55fa92_upll_clock = 192000;
	clock_div = w55fa92_upll_clock / 27000;
	clock_div /= 2;
	clock_div &= 0xFF;
	clock_div --;
	
	nvt_lock();			
	// given clock divider for VPOST 
	outl((inl(REG_CLKDIV1) & ~VPOST_N0) | 1, REG_CLKDIV1);					// divider 2 in VPOST_N0
	outl((inl(REG_CLKDIV1) & ~VPOST_N1) | (clock_div << 8), REG_CLKDIV1);	
	
//	outl((inl(REG_CLKDIV1) & ~VPOST_S), REG_CLKDIV1);				// VPOST clock from UPLL
	outl((inl(REG_CLKDIV1) & ~VPOST_S) | (3<<3), REG_CLKDIV1);		// VPOST clock from UPLL
	nvt_unlock();			
	
	// enable VPOST function pins
	outl((inl(REG_GPBFUN1)&~MF_GPB15)|0x20000000, REG_GPBFUN1);		// enable LPCLK pin
	outl(0x22222222, REG_GPCFUN0);									// enable LVDATA[7:0] pins
	outl((inl(REG_GPDFUN1)&0xFFFF000F)|0x00002220, REG_GPDFUN1);	// enable VDEN/VSYNC/HSYNC pins	
	
	// configure LCD interface  // enable sync with TV, LCD type select 
   	outl(inl(REG_LCM_LCDCPrm)& ~LCDCPrm_LCDSynTv, REG_LCM_LCDCPrm);	// async with TV
   	outl((inl(REG_LCM_LCDCPrm)& ~LCDCPrm_LCDTYPE) | 0x01, REG_LCM_LCDCPrm);	// Sync-type TFT LCD
	/*
	0x0  // High Resolution mode
	0x1  // Sync-type TFT LCD
	0x2  // Sync-type Color STN LCD
	0x3  // MPU-type LCD
	*/
	outl((inl(REG_LCM_LCDCPrm)& ~LCDCPrm_LCDDataSel) | 0x00, REG_LCM_LCDCPrm);	// CCIR601 mode

	// LCDSrc, TVSrc: Frame buffer; NotchE; 
  	outl((inl(REG_LCM_TVCtl)& ~(TVCtl_LCDSrc+TVCtl_LCDSrc))|(0x00008500), REG_LCM_TVCtl);  
		  
	///outl((inl(REG_LCM_TVDisCtl)& 0xFE000000)|(0x00F016A0), REG_LCM_TVDisCtl);

   	// set Horizontal scanning line timing for Syn type LCD 
	outl(0x050EB040, REG_LCM_TCON1);	
	
	// set Vertical scanning line timing for Syn type LCD   
	outl(0x00120303, REG_LCM_TCON2);	

	// set both "active pixel per line" and "active lines per screen" for Syn type LCD   
   	//outl(0x013F00EF, REG_LCM_TCON3);	// 320x240
   	outl(0x016700EF, REG_LCM_TCON3);	// 360x240
   	outl(0x013F00EF, REG_LCM_FB_SIZE);  // 320x240   	   	
	outl(0x016700EF, REG_LCM_SCO_SIZE);	// scaling to 360x240
	outl(inl(REG_LCM_LCDCCtl) | 0x20, REG_LCM_LCDCCtl);	

   	outl(0x14000003, REG_LCM_TCON4);	// signal polarity

	// set TV control register and LCD from frame buffer source
   	outl((inl(REG_LCM_TVCtl)& 0xFFFFF0DA)|(0x00000410), REG_LCM_TVCtl);   // DAC disable	
   	
	// enable LCD controller
	outl((inl(REG_LCM_LCDCCtl)& 0xFFFEFFF0)|(0x10003), REG_LCM_LCDCCtl);
	return 0;
}

static int w55fa92fb_ioctl_device(struct fb_info *info, unsigned int cmd, unsigned long arg)
{
	unsigned int buffer[5];
	
	memset(buffer,0,5);
	
	switch(cmd)
	{
		case IOCTLCLEARSCREEN:	
			memset((void *)video_cpu_mmap, 0xff, video_alloc_len); 		
			break;
		#if 0		
		case VIDEO_ACTIVE_WINDOW_COORDINATES:
			/* Get the start line of video window */
			copy_from_user((unsigned int *)buffer, (void *)arg, 1*sizeof(unsigned int));			
			/* Reset the original start line of video window */
			outl(0x000107FF, REG_LCM_VA_WIN);
			outl( ((buffer[0]&0x7FF) << 16) | inl(REG_LCM_VA_WIN), REG_LCM_VA_WIN);
			break;
		#endif

		case IOCTL_LCD_BRIGHTNESS:
			// Need to implement PWM backlight control here !!!			
			printk("Set Backlight\n");
			copy_from_user((unsigned int *)buffer, (void *)arg, 1*sizeof(unsigned int));
			w55fa92fb_set_CMR(buffer[0]);
			break;

		case VIDEO_DISPLAY_ON:
			outl(inl(REG_GPDFUN0) & ~MF_GPD1, REG_GPDFUN0);	
			outl(inl(REG_GPIOD_OMD) | 0x02, REG_GPIOD_OMD);				
			outl(inl(REG_GPIOD_DOUT) | 0x02, REG_GPIOD_DOUT);		// backlight ON (for Nuvoton FA93 demo board only)
			break;
			
		case VIDEO_DISPLAY_OFF:
			outl(inl(REG_GPDFUN0) & ~MF_GPD1, REG_GPDFUN0);	
			outl(inl(REG_GPIOD_OMD) | 0x02, REG_GPIOD_OMD);				
			outl(inl(REG_GPIOD_DOUT) & ~0x02, REG_GPIOD_DOUT);		// backlight OFF (for Nuvoton FA93 demo board only)
			break;

#if !defined(CONFIG_w55fa92_TV_LCD_SWITCH)					
		 case IOCTL_LCD_ENABLE_INT:
             outl((inl(REG_LCM_LCDCInt) & 0xFFFFFF00) | 0x20000, REG_LCM_LCDCInt); // enable VIN
	         //_auto_disable = 0;
                break;
		 case IOCTL_LCD_DISABLE_INT:
	         outl((inl(REG_LCM_LCDCInt) & 0xFFFFFF00) & ~0x20000, REG_LCM_LCDCInt); // disable VIN
#ifdef CONFIG_TWO_FB_BUF
  			 if ( vdma_ch_flag != -1 )
         		 while (w55fa92_edma_isbusy(vdma_ch));
#endif            
	              _auto_disable = 1;
	              break;
#endif

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
#ifndef CONFIG_w55fa92_TV_LCD_SWITCH	
	outl((inl(REG_LCM_LCDCInt) & 0xFFFFFF00) | 0x20000, REG_LCM_LCDCInt); // enable VINTEN
#endif	
	
	/* enable lcd*/
	outl((inl(REG_LCM_LCDCCtl)& 0xFFFEFFF0)|(0x10003), REG_LCM_LCDCCtl);
   	return 0;
}

static void w55fa92fb_stop_device(void)
{
	outl((inl(REG_LCM_LCDCCtl)& 0xFFFEFFFE), REG_LCM_LCDCCtl);
}

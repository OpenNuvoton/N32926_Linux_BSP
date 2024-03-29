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

extern u32 _osd_trigger;

volatile static int backlight_ctrl_busy = 0;	// for backlight control timer

static int w55fa92fb_init_device(struct w55fa92fb_info *fbi)
{
	unsigned int clock_div, u32prediv;
	

  	// Reset IP
	outl(VPOST_RST, REG_AHBIPRST);
	outl(0, REG_AHBIPRST);	

	// given clock divider for VPOST 
	clock_div = w55fa92_upll_clock / 48000;	// VPOST clock = 30 MHz
	u32prediv = 0;
	clock_div --; 

	// given clock divider for VPOST 
	nvt_lock();		
	outl((inl(REG_CLKDIV1) & ~VPOST_N0) | u32prediv, REG_CLKDIV1);					// divider 2 in VPOST_N0
	outl((inl(REG_CLKDIV1) & ~VPOST_N1) | (clock_div << 8), REG_CLKDIV1);	
	
//	outl((inl(REG_CLKDIV1) & ~VPOST_S), REG_CLKDIV1);				// VPOST clock from UPLL
	outl((inl(REG_CLKDIV1) & ~VPOST_S) | (3<<3), REG_CLKDIV1);		// VPOST clock from UPLL
	nvt_unlock();		

	// enable VPOST function pins
	outl((inl(REG_GPBFUN1)&~MF_GPB15)|0x20000000, REG_GPBFUN1);	// enable LPCLK pin
	outl(0x22222222, REG_GPCFUN0);									// enable LVDATA[7:0] pins
	outl(0x22222222, REG_GPCFUN1);									// enable LVDATA[15:8] pins		
	outl((inl(REG_GPDFUN1)&0xFFFF000F)|0x00002220, REG_GPDFUN1);	// enable VDEN/VSYNC/HSYNC pins	
	outl((inl(REG_GPEFUN0)&~(MF_GPE0+MF_GPE1))|0x22, REG_GPEFUN0);	// enable LVDATA[17:16] pins
	outl((inl(REG_GPBFUN0)&0x0FFFFFFF)|0x20000000, REG_GPBFUN0);  	// enable LVDATA[18] pin
	outl((inl(REG_GPBFUN1)&0xFFF00000)|0x00022222, REG_GPBFUN1);  	// enable LVDATA[23:19] pins
	
	// configure LCD interface  // enable sync with TV, LCD type select 
   	outl(inl(REG_LCM_LCDCPrm)& ~LCDCPrm_LCDSynTv, REG_LCM_LCDCPrm);	// async with TV
   	outl(inl(REG_LCM_LCDCPrm)& ~LCDCPrm_LCDTYPE, REG_LCM_LCDCPrm);	// High Resolution mode
	/*
	0x0  // High Resolution mode
	0x1  // Sync-type TFT LCD
	0x2  // Sync-type Color STN LCD
	0x3  // MPU-type LCD
	*/

	// configure LCD parallel data bus 
 	outl( (inl(REG_LCM_LCDCCtl)& ~LCDCCtl_PRDB_SEL) | (0x02 << 20), REG_LCM_LCDCCtl);	// 24-bit mode 
	/*
	0x0  // 16-bit mode (RGB565 output)
	0x1  // 18-bit mode (RGB666 output)
	0x2  // 24-bit mode (RGB888 output)		
	*/
	
	// LCDSrc, TVSrc: Frame buffer; NotchE; enable TV encoder; NTSC; Frame Buffer Size:640x480; Interlace
  	outl((inl(REG_LCM_TVCtl)& 0xFFFF30DA)|(0x00008529), REG_LCM_TVCtl);  

		  
	outl((inl(REG_LCM_TVDisCtl)& 0xFE000000)|(0x00F016A0), REG_LCM_TVDisCtl);

   	// set Horizontal scanning line timing for Syn type LCD 
	outl(0x0A0A00D2, REG_LCM_TCON1);	
	
	// set Vertical scanning line timing for Syn type LCD   
	outl((inl(REG_LCM_TCON2)& 0xFF000000)|(0x000A1716), REG_LCM_TCON2);

	// set both "active pixel per line" and "active lines per screen" for Syn type LCD   
   	outl(0x03FF0257, REG_LCM_TCON3);	// 1024x600
   	outl(0x03FF0257, REG_LCM_FB_SIZE);  	// 1024x600, frame buffer size
   	outl(0x4000C502, REG_LCM_TCON4);	// signal polarity

	// set TV control register and LCD from frame buffer source
   	outl((inl(REG_LCM_TVCtl)& 0xFFFFF0DA)|(0x00000410), REG_LCM_TVCtl);   // DAC disable	
   	
	// enable LCD controller
	outl((inl(REG_LCM_LCDCCtl)& 0xFFFEFFF0)|(0x10003), REG_LCM_LCDCCtl);	// RGBx565 format

	// set keypad register

#ifdef CONFIG_w55fa92_KEYPAD_MxN	
	outl(0x090FF057, REG_LCM_TCON1);	
	outl((inl(REG_LCM_TCON1)& ~0xFF000000)|(0x3A000000), REG_LCM_TCON1);		
	outl(0x00320038, REG_LCM_KPI_HS_DLY);			
#endif		

	// enable video data preload mode
	outl(inl(REG_LCM_COLORSET) | BIT31, REG_LCM_COLORSET);	// enable preload mode
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
			
		case IOCTL_LCD_BRIGHTNESS:
			// Need to implement PWM backlight control here !!!
			copy_from_user((unsigned int *)buffer, (void *)arg, 1*sizeof(unsigned int));
			w55fa92fb_set_CMR(buffer[0]);
			break;

		case VIDEO_DISPLAY_LCD:
			printk("video displayed by LCD only\n");

			// given clock divider for VPOST 
//			outl((inl(REG_CLKDIV1) & ~VPOST_S) | (0x03 << 3), REG_CLKDIV1);	// 00:clock from CLK_in, 01:32K, 10:APLL, 11:UPLL

			// enable VPOST function pins
			outl((inl(REG_GPBFUN1)&~MF_GPB15)|0x20000000, REG_GPBFUN1);	// enable LPCLK pin
			outl(0x22222222, REG_GPCFUN0);									// enable LVDATA[7:0] pins
			outl(0x22222222, REG_GPCFUN1);									// enable LVDATA[15:8] pins		
			outl((inl(REG_GPDFUN1)&0xFFFF000F)|0x00002220, REG_GPDFUN1);	// enable VDEN/VSYNC/HSYNC pins	
			outl((inl(REG_GPEFUN0)&~(MF_GPE0+MF_GPE1))|0x22, REG_GPEFUN0);	// enable LVDATA[17:16] pins

	#ifdef OPT_FGPA // for FPGA
//			outl((inl(0xB00000F0)&0xFFFFF000)|0x00000555, 0xB00000F0);  	// enable LVDATA[23:18] pins	
	#else
			outl((inl(REG_GPBFUN1)&0xFFF00000)|0x00022222, REG_GPBFUN1);  	// enable LVDATA[23:18] pins	
	#endif		
		   	
			// configure LCD interface  // enable sync with TV, LCD type select 
		   	outl(inl(REG_LCM_LCDCPrm)& ~LCDCPrm_LCDSynTv, REG_LCM_LCDCPrm);	// async with TV
		   	outl(inl(REG_LCM_LCDCPrm)& ~LCDCPrm_LCDTYPE, REG_LCM_LCDCPrm);	// High Resolution mode
			/*
			0x0  // High Resolution mode
			0x1  // Sync-type TFT LCD
			0x2  // Sync-type Color STN LCD
			0x3  // MPU-type LCD
			*/
		
			// configure LCD parallel data bus 
		  	outl( (inl(REG_LCM_LCDCCtl)& ~LCDCCtl_PRDB_SEL) | (0x02 << 20), REG_LCM_LCDCCtl);	// 24-bit mode 	
			/*
			0x0  // 16-bit mode (RGB565 output)
			0x1  // 18-bit mode (RGB666 output)
			0x2  // 24-bit mode (RGB888 output)		
			*/
			
			// LCDSrc, TVSrc: Frame buffer; NotchE; enable TV encoder; NTSC; Frame Buffer Size:640x480; Interlace
		  	outl((inl(REG_LCM_TVCtl)& 0xFFFF30DA)|(0x00008529), REG_LCM_TVCtl);  
		
				  
			outl((inl(REG_LCM_TVDisCtl)& 0xFE000000)|(0x00F016A0), REG_LCM_TVDisCtl);


			// set Horizontal scanning line timing for Syn type LCD 
			outl(0x0A0A00D2, REG_LCM_TCON1);	
			
			// set Vertical scanning line timing for Syn type LCD   
			outl((inl(REG_LCM_TCON2)& 0xFF000000)|(0x000A1716), REG_LCM_TCON2);
		
			// set both "active pixel per line" and "active lines per screen" for Syn type LCD   
			outl(0x03FF0257, REG_LCM_TCON3);	// 1024x600
			outl(0x03FF0257, REG_LCM_FB_SIZE);  	// 1024x600, frame buffer size
			outl(0x4000C502, REG_LCM_TCON4);	// signal polarity

			// set TV control register and LCD from frame buffer source
		   	outl((inl(REG_LCM_TVCtl)& 0xFFFFF0DA)|(0x00000410), REG_LCM_TVCtl);   // DAC disable	

			// enable LCD controller
			outl((inl(REG_LCM_LCDCCtl)& 0xFFFEFFF0)|(0x10003), REG_LCM_LCDCCtl);	// RGBx565 format
			break;				


		case VIDEO_DISPLAY_TV:
			printk("video displayed by TV only\n");

			// disable VPOST function pins
			outl((inl(REG_GPBFUN1)&~MF_GPB15), REG_GPBFUN1);			// disable LPCLK pin
			outl(0x00000000, REG_GPCFUN0);								// disable LVDATA[7:0] pins
			outl(0x00000000, REG_GPCFUN1);								// disable LVDATA[15:8] pins		
			outl((inl(REG_GPDFUN1)&0xFFFF000F), REG_GPDFUN1);			// disable VDEN/VSYNC/HSYNC pins	
			outl((inl(REG_GPEFUN0)&~(MF_GPE0+MF_GPE1)), REG_GPEFUN0);	// disable LVDATA[17:16] pins
	#ifdef OPT_FGPA // for FPGA
//			outl((inl(0xB00000F0)&0xFFFFF000), 0xB00000F0);  			// disable LVDATA[23:18] pins	
	#else
			outl((inl(REG_GPBFUN1)&0xFFF00000), REG_GPBFUN1);  			// disable LVDATA[23:18] pins	
	#endif		
//			outl((inl(REG_CLKDIV1) & ~VPOST_S) | (0x00 << 3), REG_CLKDIV1);	// 00:clock from CLK_in, 01:32K, 10:APLL, 11:UPLL
//			outl((inl(REG_CLKDIV1) & ~VPOST_N1) | (0x00 << 8), REG_CLKDIV1);	// divider value = 0x01 (temp)

		   	outl(inl(REG_LCM_LCDCPrm) | LCDCPrm_LCDSynTv, REG_LCM_LCDCPrm);	// sync with TV
			outl((inl(REG_LCM_LCDCCtl) & ~LCDCCtl_LCDRUN), REG_LCM_LCDCCtl); // LCD off
	    	outl((inl(REG_LCM_TVCtl)& 0xFFFFF0CA)|(0x00000501), REG_LCM_TVCtl);   // DAC enabled
			outl((inl(REG_LCM_LCDCCtl)& 0xFFFEFFF0)|(0x10002), REG_LCM_LCDCCtl); // LCD off, RGB565 format
			break;				

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

#ifdef CONFIG_TWO_FB_BUF
		case IOCTL_LCD_GET_DMA_BASE:
		  if(copy_to_user((unsigned int *)arg, (unsigned int *)&_bg_mem_p, sizeof(unsigned int)) < 0) {
		    printk("failed..\n");
		    return(-EFAULT);
		  }
	//		printk("!!!%x\n", *(unsigned int *)&_bg_mem_p);
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


/* linux/driver/input/w55fa92_ts.c
 *
 * Copyright (c) 2013 Nuvoton technology corporation
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * Changelog:
 *
 *   2013/07/11   SWChou add this file for nuvoton touch screen driver.
 */
 


#include <linux/init.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <asm/io.h>
#include <mach/hardware.h>
#include <mach/irqs.h>
#include <mach/w55fa92_reg.h>
#undef BIT
#include <linux/input.h>
#include <linux/clk.h>
#include "w55fa92_ts_adc.h"

#ifdef CONFIG_BATTERY_DETECTION
extern struct timer_list battery_timer;
#endif

//#define DBG
#ifdef DBG
#define ENTER()		printk("ENTER : %s  : %s\n",__FILE__, __FUNCTION__)
#define LEAVE()		printk("LEAVE : %s  : %s\n",__FILE__, __FUNCTION__)
#define ERRLEAVE()	printk("ERRLEAVE : %s  : %s\n",__FILE__, __FUNCTION__)
#define DBG_PRINTF	printk
#else
#define ENTER(...)
#define LEAVE(...)	
#define ERRLEAVE(...)	
#define DBG_PRINTF(...)	
#endif

static PFN_ADC_CALLBACK (g_psVoltageDetectionCallBack)[1] = {0};
static void VoltageDetect_callback(UINT32 u32code)
{
	ENTER();
	if( g_psVoltageDetectionCallBack[0]!=0)
		g_psVoltageDetectionCallBack[0](u32code);	
	LEAVE();
}

void Install_VoltageDetectionCallback(PFN_ADC_CALLBACK pfnCallback)
{	
	ENTER();
	g_psVoltageDetectionCallBack[0] = pfnCallback; 
	LEAVE();
}

static UINT32 priority = 2; /* from channel 1. bit addressing */
#define DBGPRIORITY(...)
void timer_check_battery(unsigned long dummy)
{
	INT32 i32ret;
	UINT32 u32Channelmap=0;				/* bit addressing */
	UINT32 u32Channel = 0;
	
	ENTER();
#ifdef CONFIG_BATTERY_CHANNEL_1
	u32Channelmap = u32Channelmap | (1<<1);
#endif
#ifdef CONFIG_BATTERY_CHANNEL_2	
	u32Channelmap = u32Channelmap | (1<<2);
#endif
#ifdef CONFIG_BATTERY_CHANNEL_3	
	u32Channelmap = u32Channelmap | (1<<3);
#endif
#ifdef CONFIG_BATTERY_CHANNEL_4
	u32Channelmap = u32Channelmap | (1<<4);
#endif
#ifdef CONFIG_BATTERY_CHANNEL_5	
	u32Channelmap = u32Channelmap | (1<<5);
#endif
#ifdef CONFIG_BATTERY_CHANNEL_6	
	u32Channelmap = u32Channelmap | (1<<6);
#endif
#ifdef CONFIG_BATTERY_CHANNEL_7
	u32Channelmap = u32Channelmap | (1<<7);
#endif
	DBGPRIORITY("u32Channelmap 0x%x\n", u32Channelmap);
	DBGPRIORITY("priority 0x%x\n", priority);
	do
	{	
		uint32_t u32Active = u32Channelmap & priority;	
		
		if(u32Active!=0)
		{
			do{
				u32Active = u32Active>>1;								
				u32Channel = u32Channel+1;
			}while( ((u32Active&0x1) == 0) && (u32Channel<=8) );
			
			DBGPRIORITY("Check channel %d\n", u32Channel);
			i32ret = DrvADC_VoltageDetection(u32Channel);//return E_ADC_BUSY or Successful		
			if(i32ret==E_ADC_BUSY){//do again after 10ms
				DBG_PRINTF("BZ\n");
				//mod_timer(&battery_timer, jiffies + INIT_INTERVAL_TIME); 
				mod_timer(&battery_timer, jiffies + BATTERY_INTERVAL_TIME); 				
				LEAVE();
				
				DBGPRIORITY("next priority = %d\n\n\n", priority);
				return; 
			}else{//Successful, change the first proioity for next channel,
				priority = priority<<1;
				if(priority >= 0x100)			
					priority = 2;				 				
				mod_timer(&battery_timer, jiffies + BATTERY_INTERVAL_TIME); 
				DBGPRIORITY("next priority = %d\n\n\n", priority);
				return;
			}					
		}		
	}while(0);
	DBGPRIORITY("next priority = %d\n\n\n", priority);	
	priority = priority<<1;
	if(priority >= 0x100)			
		priority = 2;
	mod_timer(&battery_timer, jiffies + BATTERY_INTERVAL_TIME); 
	LEAVE();
}
void init_batterydetection(void)
{
	PFN_ADC_CALLBACK pfnOldCallback;

	ENTER();
	DrvADC_InstallCallback(eADC_AIN,
						VoltageDetect_callback,
						&pfnOldCallback);
	LEAVE();
}

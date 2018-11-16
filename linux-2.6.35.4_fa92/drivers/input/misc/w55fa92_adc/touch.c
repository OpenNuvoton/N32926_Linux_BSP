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

#ifdef CONFIG_TOUCH_DETECTION
extern struct timer_list touch_timer;
extern struct input_dev *w55fa92_dev;
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

/*==============================================================================
	For sysmgr to disable touch if backlight disable 
==============================================================================*/
static u32 bisEnableTouch=1;
u32 w55fa92_ts_pressing = 0;
void disable_touch(void)
{
	printk("ADC: %s\n",__FUNCTION__);
	bisEnableTouch = 0;
}
void enable_touch(void)
{
	printk("ADC: %s\n",__FUNCTION__);
	bisEnableTouch = 1;
}
EXPORT_SYMBOL(disable_touch);	
EXPORT_SYMBOL(enable_touch);
EXPORT_SYMBOL(w55fa92_ts_pressing);
/*==============================================================================
==============================================================================*/
static uint16_t u16LastX =0;
static uint16_t u16LastY =0;
void report_touch(BOOL bIsValid, UINT16 u16X, UINT16 u16Y, UINT16 u16Press)
{	
	if(bisEnableTouch==1)
	{
		if( u16Press ==0 ){/* pressure = 0, update the position to last position */
			u16X = u16LastX;
			u16Y = u16LastY;
		}
		input_report_key(w55fa92_dev, BTN_TOUCH, bIsValid);
		input_report_abs(w55fa92_dev, ABS_X, u16X);
		input_report_abs(w55fa92_dev, ABS_Y, u16Y);
		input_report_abs(w55fa92_dev, ABS_PRESSURE, u16Press);	
		input_sync(w55fa92_dev); /* sync must place in the last */

		u16LastX = u16X;	/* Recording current position to last position */
		u16LastY = u16Y;	
	}
}

static void Position_callback(UINT32 u32code)	
{
	UINT16 u16X =  (u32code>>16)&0x7FFF;	/* In driver has judge valid or invalid */
	UINT16 u16Y =  u32code & 0x7FFF;
	report_touch(1, u16X, u16Y, 1000);
}
/*
	i32PenState = 0 ==> Up
	i32PenState = 1 ==> Dn
	i32PenState = 2 ==> Dn -> Up
*/
void timer_check_touch(unsigned long dummy)
{
	INT32 i32ret;
	static INT32 i32PenState = 0;	
	i32ret = DrvADC_PenDetection(0);//4 wire==>FALSE
	if(i32ret==E_ADC_BUSY){//do again after 10ms
		DBG_PRINTF("BZ\n");
		mod_timer(&touch_timer, jiffies + INIT_INTERVAL_TIME);
		return;
	}else if(i32ret == E_TOUCH_UP) {
		w55fa92_ts_pressing = 0;
		if(i32PenState == 1)		
			i32PenState = 2;	//2 means from down to up
	}else {
		DBG_PRINTF("DN\n");
		i32PenState = 1;
		w55fa92_ts_pressing = 1;
	}	
	if(i32PenState == 2){
		report_touch(0, 0, 0, 0);
		i32PenState = 0;		// after report to input layer,  i32PenState = up state
	}
	mod_timer(&touch_timer, jiffies + TOUCH_INTERVAL_TIME); 
}
void init_touchpanel(void)
{
	PFN_ADC_CALLBACK pfnOldCallback;

	DrvADC_InstallCallback(eADC_POSITION,
						Position_callback,
						&pfnOldCallback);
/*
	DrvADC_InstallCallback(eADC_TOUCH,
						TouchPanel_callback,
						&pfnOldCallback);
*/
}

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

#ifdef CONFIG_KEYPAD_DETECTION
extern struct timer_list keypad_timer;
extern struct input_dev *w55fa92_keypad_input_dev;
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
	For sysmgr to know the key pressing state
==============================================================================*/
u32 w55fa92_key_pressing = 0;
EXPORT_SYMBOL(w55fa92_key_pressing);
/*==============================================================================
==============================================================================*/

#define ASC_KEY_ENTER		13	
#define ASC_KEY_HOME	 	19
#define ASC_KEY_UP			14	/* Compound key */
#define ASC_KEY_DOWN		15
#define ASC_KEY_LEFT		1
#define ASC_KEY_RIGHT		2
#define ASC_KEY_A		13	
#define ASC_KEY_B	 	19


#define KEY_COUNT			6
const int key_map[KEY_COUNT] = {
#if defined(CONFIG_DOORBELL_DB_PINCODE)
	ASC_KEY_B, 		ASC_KEY_A, 
	ASC_KEY_RIGHT, 	ASC_KEY_LEFT, 
	ASC_KEY_UP, 	ASC_KEY_DOWN
#else
	ASC_KEY_ENTER, 	ASC_KEY_HOME, 
	ASC_KEY_UP, 	ASC_KEY_DOWN, 
	ASC_KEY_LEFT, 	ASC_KEY_RIGHT
#endif
};


void report_keypad(INT32 bIsValid, UINT16 u16MapKeyCode)
{	
	INT32 i, j;

	if(bIsValid==0){//No Key pressing
		for(i=0, j=1; i<KEY_COUNT; i=i+1 ){
			input_report_key(w55fa92_keypad_input_dev, key_map[i], 0);     //All key up
		    input_sync(w55fa92_keypad_input_dev);
			j=j<<1;
		}
	}else{
		for(i=0, j=1; i<KEY_COUNT; i= i+1){
			if( (u16MapKeyCode&j) !=0)	
				input_report_key(w55fa92_keypad_input_dev, key_map[i], 1);  //key down				
			else
				input_report_key(w55fa92_keypad_input_dev, key_map[i], 0);  //key up				
			input_sync(w55fa92_keypad_input_dev);						
			j=j<<1;
		}
	}
}



typedef struct tagKeyMap{
	UINT32 u32AinKey;
	UINT32 u32MapKey;
}S_KEYMAP;

static S_KEYMAP sKeymap[] = {
#if defined(CONFIG_DOORBELL_DB_PINCODE)
		{0x616, 1}, 				//S6     	[B] (Board no3/no4 ==> 0x612/0x61B)
		{0x6F5, (1<<1)}, 			//S5		[A]
		{0x9C0, (1<<2)}, 					//S4		[R]
		{0xB41, (1<<3)}, 					//S3		[L]
		{0xC33, (1<<4)}, 					//S2		[U]
		{0xCD8, (1<<5)}, 					//S1		[D]
		
		{0x410, (1 | (1<<1))}, 		//S6+S5
		{0x4E1, (1 | (1<<2))}, 		//S6+S4
		{0x53A, (1 | (1<<3))}, 		//S6+S3				
		{0x56C, (1 | (1<<4))},		//S6+S2				 
		{0x58C, (1 | (1<<5))}, 		//S6+S1
#else
		{0xCD8, 1}, 				//Enter
		{0xC33, (1<<1)}, 			//Home
		{0x616, (1<<2)}, 			//Up
		{0x6F5, (1<<3)},			//Down
		{0x9C0, (1<<4)},			//Left	
		{0xB41, (1<<5)},			//Right	

		{0x58C, ((1<<2) | (1))}, 	//UP+Enter
		{0x56C, ((1<<2) | (1<<1))}, //UP+Home
		{0x410, ((1<<2) | (1<<3))}, //UP+DOWN		
		{0x4E0, ((1<<2) | (1<<4))},	//UP+LEFT			 
		{0x53A, ((1<<2) | (1<<5))}, //UP+RIGHT
#endif				
					};
static UINT32 keymap(UINT16 u16AinCode)
{
	UINT16 u16AinMax, u16AinMin;
	UINT32 u32idx;
	UINT32 u32Item = sizeof(sKeymap)/sizeof(sKeymap[0]);
	for(u32idx=0; u32idx<u32Item ; u32idx=u32idx+1){
		if(u16AinCode < (sKeymap[0].u32AinKey + sKeymap[u32Item-1].u32AinKey)/2){//compound key
			u16AinMax = sKeymap[u32idx].u32AinKey+0xF;
			u16AinMin = sKeymap[u32idx].u32AinKey-0xF;
		}else{
			u16AinMax = sKeymap[u32idx].u32AinKey+0x50;
			u16AinMin = sKeymap[u32idx].u32AinKey-0x50;		
		}			
		if( (u16AinCode>u16AinMin) && (u16AinCode<u16AinMax) )
			return sKeymap[u32idx].u32MapKey;
	}	
	return 0;
}	
/*
	i32PenState = 0 ==> Up
	i32PenState = 1 ==> Dn
	i32PenState = 2 ==> Dn -> Up
*/
void timer_check_keypad(unsigned long dummy)
{
	INT32 i32ret;
	UINT32 u32Channel, u32KeyCode, u32MapKey;	
	static INT32 i32KeyState = 0;	
	
	ENTER();
	u32Channel = 2;
	i32ret = DrvADC_KeyDetection(u32Channel, &u32KeyCode);
	if(i32ret==E_ADC_BUSY){//do again after 10ms
		DBG_PRINTF("BZ\n");
		mod_timer(&keypad_timer, jiffies + INIT_INTERVAL_TIME);
		return;
	}else if(i32ret == E_KEYPAD_UP) {
		w55fa92_key_pressing = 0;
		if(i32KeyState == 1)		
			i32KeyState = 2;	//2 means all key from down to up
	}else {
		DBG_PRINTF("DN\n");
		i32KeyState = 1;
		w55fa92_key_pressing = 1;
		u32MapKey = keymap(u32KeyCode);		
		if(u32MapKey!=0)
			report_keypad(1, u32MapKey);
	}	
	if(i32KeyState == 2){
		report_keypad(0, 0);
		i32KeyState = 0;		// after report to input layer,  i32PenState = up state
	}
	mod_timer(&keypad_timer, jiffies + KEYPAD_INTERVAL_TIME); 	
}
void init_keypad(void)
{

}



/*
case 1: press [1] then release	
	press [1] on going
		1. 		1 1 1 	[1] means down
		2. 		1 1 2	[2] means repeat	
		3. 		1 1 2		
		....
		n. 		1 1 2 	
		n+1.	1 1 0	[0] means up 	 

case 2:	press [1] then press key[2], then release key[1]. then release key[2] 
		1. 		1 1 1 	[1] means down
		2. 		1 1 2	[2] means repeat	
		3. 		1 1 2		
		....
		n. 		1 1 2 	
		n+1.	1 2 1   [1] mean key [2] down
 		n+2.	1 2 2   [2] mean key [2] repeat
		....
		m.		1 1 0   [2] mean key [1] up
		m+1		1 2 2 
		....
		o.		1 2 0   [2] mean key [2] up


case 3:	press [1] then press key[2], then release key[2]. then release key[1] 
		1. 		1 1 1 	[1] means down
		2. 		1 1 2	[2] means repeat	
		3. 		1 1 2		
		....
		n. 		1 1 2 	
		n+1.	1 2 1   [1] mean key [2] down
 		n+2.	1 2 2   [2] mean key [2] repeat
		....
		m.		1 2 0   [2] mean key [2] up
		//Stop print
		o.		1 1 0   [2] mean key [1] up

*/

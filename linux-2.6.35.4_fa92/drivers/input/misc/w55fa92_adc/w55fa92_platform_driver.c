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

#define BIT(x)  	(1UL<<((x)%BITS_PER_LONG))
#define LONG(x) 	((x)/BITS_PER_LONG)

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


#define TOUCH_IRQ_NUM             W55FA92_IRQ(28)  // nIRQ0
static irqreturn_t adc_isr(int irq, 
							void *dev_id)
{
	AdcIntHandler();
	return IRQ_HANDLED;	
}


/* ==============================================================================================*/
#ifdef CONFIG_KEYPAD_DETECTION
INT32 i32KeypadAdcOpenCount = 0;
static spinlock_t spin_kpd_opc = SPIN_LOCK_UNLOCKED;
struct timer_list keypad_timer;
struct input_dev *w55fa92_keypad_input_dev;
extern void timer_check_keypad(unsigned long dummy);


static int w55fa92_kpd_open(struct input_dev *dev)
{  
	int result, err;
	ENTER();

	spin_lock(&spin_kpd_opc);
	i32KeypadAdcOpenCount++;
	if(i32KeypadAdcOpenCount != 1){// kpd had been opened,  
		ERRLEAVE();
		spin_unlock(&spin_kpd_opc);
		return 0;
	}
	else
		spin_unlock(&spin_kpd_opc);

	DrvADC_Open();	/* Will check open count in the function */

	init_timer(&keypad_timer);
	keypad_timer.function = timer_check_keypad;		// timer handler for keypad 
	keypad_timer.data = 1;
	mod_timer(&keypad_timer, jiffies + KEYPAD_INTERVAL_TIME);
	DBG_PRINTF("Init keypad timer\n");
	 
	result = request_irq(TOUCH_IRQ_NUM, 				/*  IRQ number */
					adc_isr, 							/*  IRQ Handler */
					IRQF_DISABLED | IRQF_SHARED, 		/*  irqflags */	
					"w55fa92-adc-keypad",							/*  *devname */
					w55fa92_keypad_input_dev);	/*  *dev_id */
	if(result!=0){
		printk("register ADC ISR failed!\n");
		err = -EINTR;
		return err;
	}else{
		DBG_PRINTF("requet irq successful\n");	
	}  

#if 0
	result = request_irq(KPD_IRQ_NUM, 				/*  IRQ number */
					NULL, 							/*  IRQ Handler */
					IRQF_DISABLED | IRQF_SHARED, 	/*  irqflags */
					"KPD", 							/*  *devname */
					w55fa92_keypad_input_dev);		/*  *dev_id */

	if(result!=0){
		printk("register ADC ISR failed!\n");
		err = -EINTR;
		return err;
	}else
		DBG_PRINTF("requet irq successful\n");	
#endif

	LEAVE();		
	return 0;
}
static void w55fa92_kpd_close(struct input_dev *dev)
{	
	ENTER();

	spin_lock(&spin_kpd_opc);
	i32KeypadAdcOpenCount--;
	if(i32KeypadAdcOpenCount<0)
		i32KeypadAdcOpenCount = 0;
	if(i32KeypadAdcOpenCount != 0)
	{// Touchadc still need to be opened.
		spin_unlock(&spin_kpd_opc);
		return;
	}
	else
		spin_unlock(&spin_kpd_opc);	
	del_timer(&keypad_timer);	
	DrvADC_Close();		/* Will check open count in the function */
	free_irq(TOUCH_IRQ_NUM, w55fa92_keypad_input_dev);
	LEAVE();
}




int __devinit w55fa92_kpd_init(void)
{
	int ret = 0, i;

    ENTER();	

	if (!(w55fa92_keypad_input_dev = input_allocate_device())) {
                printk("W55FA92 Keypad Drvier Allocate Memory Failed!\n");
                ret = -ENOMEM;
                LEAVE();
				return ret; 
        }

    w55fa92_keypad_input_dev->name = "W55FA92 Keypad";
    w55fa92_keypad_input_dev->phys = "input/event1";
    w55fa92_keypad_input_dev->id.bustype = BUS_HOST;
    w55fa92_keypad_input_dev->id.vendor  = 0x0005;
    w55fa92_keypad_input_dev->id.product = 0x0001;
    w55fa92_keypad_input_dev->id.version = 0x0100;

    w55fa92_keypad_input_dev->open    = w55fa92_kpd_open;
    w55fa92_keypad_input_dev->close   = w55fa92_kpd_close;

    w55fa92_keypad_input_dev->evbit[0] = BIT(EV_KEY) | BIT(EV_SYN) |  BIT(EV_REP);

    for (i = 0; i < KEY_MAX; i++)
            set_bit(i+1, w55fa92_keypad_input_dev->keybit);

    ret = input_register_device(w55fa92_keypad_input_dev);
    if (ret) {

            input_free_device(w55fa92_keypad_input_dev);
            return ret;
    }

	// must set after input device register!!!
    w55fa92_keypad_input_dev->rep[REP_DELAY] = 200; //250ms
    w55fa92_keypad_input_dev->rep[REP_PERIOD] = 100; //ms

/*
	init_timer(&keypad_timer);
	keypad_timer.function = timer_check_keypad;		// timer handler for keypad 
	mod_timer(&keypad_timer, jiffies + KEYPAD_INTERVAL_TIME);
	DBG_PRINTF("Init keypad timer\n");
*/

    printk("W55FA92 keypad driver has been initialized successfully!\n");
	LEAVE();
    return 0;
}

static void __exit w55fa92_kpd_exit(void)
{
	ENTER();	

	LEAVE();
}
module_init(w55fa92_kpd_init);
module_exit(w55fa92_kpd_exit);

#endif
/* ==============================================================================================*/
#if defined(CONFIG_TOUCH_DETECTION) || defined(CONFIG_BATTERY_DETECTION)

struct input_dev *w55fa92_dev;


#ifdef CONFIG_BATTERY_DETECTION
#include <linux/platform_device.h>
static struct platform_device *sys_adc;
static volatile unsigned int do_check = 1;

static volatile unsigned int w55fa92_channel7 = 0;
static volatile unsigned int w55fa92_channel6 = 0;	
static volatile unsigned int w55fa92_channel5 = 0;	
static volatile unsigned int w55fa92_channel4 = 0;	
static volatile unsigned int w55fa92_channel3 = 0;	
static volatile unsigned int w55fa92_lux = 0;	//channel 2
static volatile unsigned int w55fa92_vol = 0;	//channel 1

void VoltageDetectionCallback(unsigned int u32Vol)
{
	int32_t chn;
	ENTER();
	chn = DrvADC_GetChannel();
	//printk("chn-%d = %d\n", chn, u32Vol);
	switch(chn)
	{
		case 1:
			w55fa92_vol = u32Vol;
			break;
		case 2:
			w55fa92_lux = u32Vol;
			break;
		case 3:
			w55fa92_channel3 = u32Vol;
			break;
		case 4:
			w55fa92_channel4 = u32Vol;
			break;
		case 5:
			w55fa92_channel5 = u32Vol;
			break;
		case 6:
			w55fa92_channel6 = u32Vol;
			break;			
		case 7:
			w55fa92_channel7 = u32Vol;
			break;	
	}
	LEAVE();
}
#endif



#ifdef CONFIG_TOUCH_DETECTION
	struct timer_list touch_timer;
	extern void timer_check_touch(unsigned long dummy);
#endif
#ifdef CONFIG_BATTERY_DETECTION
	struct timer_list battery_timer;
	extern void timer_check_battery(unsigned long dummy);
	void Install_VoltageDetectionCallback(PFN_ADC_CALLBACK pfnCallback);
#endif


static spinlock_t spin_ts_opc = SPIN_LOCK_UNLOCKED;
INT32 i32TouchAdcOpenCount = 0;
static int w55fa92ts_open(struct input_dev *dev)
{  
	int result,err;

	ENTER();
	spin_lock(&spin_ts_opc);
	i32TouchAdcOpenCount++;
	if(i32TouchAdcOpenCount != 1){// Ts has open,  
		ERRLEAVE();
		spin_unlock(&spin_ts_opc);
		return 0;
	}
	else
		spin_unlock(&spin_ts_opc);

	result = request_irq(TOUCH_IRQ_NUM , 						/*  IRQ number */
							adc_isr, 						/*  IRQ Handler */
							IRQF_DISABLED | IRQF_SHARED, 	/*  irqflags */
							"w55fa92-adc-touch", 							/*  *devname */
							w55fa92_dev);					/*  *dev_id */

	if(result!=0){
		printk("register ADC ISR failed!\n");
		err = -EINTR;
		return err;
	}else
		DBG_PRINTF("requet irq successful\n");


	DrvADC_Open();	/* Will check open count in the function */
#ifdef CONFIG_TOUCH_DETECTION
	init_timer(&touch_timer);
	touch_timer.function = timer_check_touch;		// timer handler for touch 
	mod_timer(&touch_timer, jiffies + TOUCH_INTERVAL_TIME);
	DBG_PRINTF("Init touch detection timer\n");
	init_touchpanel();
#endif
#ifdef CONFIG_BATTERY_DETECTION
	init_timer(&battery_timer);
	battery_timer.function = timer_check_battery;	// timer handler for battery 
	mod_timer(&battery_timer, jiffies + INIT_INTERVAL_TIME);
	DBG_PRINTF("Init battery detection timer\n");
	Install_VoltageDetectionCallback(VoltageDetectionCallback);	
	init_batterydetection();
#endif
		

	
	

	LEAVE();		
	return 0;
}
static void w55fa92ts_close(struct input_dev *dev)
{	
	ENTER();
	
	spin_lock(&spin_ts_opc);
	i32TouchAdcOpenCount--;
	if(i32TouchAdcOpenCount<0)
		i32TouchAdcOpenCount = 0;
	if(i32TouchAdcOpenCount != 0)
	{// Touchadc still need to be open.
		spin_unlock(&spin_ts_opc);
		return;
	}
	else
		spin_unlock(&spin_ts_opc);

#ifdef CONFIG_TOUCH_DETECTION
	del_timer(&touch_timer);	
#endif
#ifdef CONFIG_BATTERY_DETECTION
	del_timer(&battery_timer);	
#endif

	DrvADC_Close();		/* Will check open count in the function */
	free_irq(TOUCH_IRQ_NUM, w55fa92_dev);
	LEAVE();
}

//static int __init w55fa92ts_probe(struct platform_device *pdev)
static int w55fa92ts_probe(struct platform_device *pdev)
{
	int err;

	ENTER();		

	/* Get Interrupt Channel */
#if 0
	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "no irq for device\n");
		printk("W55FA95 TS Get IRQ Failed!\n");
		return -ENOENT;
	} 
	else {
		DBG_PRINTF("ADC IRQ number %d\n", irq); 
	  	irqnum = irq;
	}
#endif 
	/* Allocate Register Space. Is it need??? */
	if (!request_mem_region((unsigned long)W55FA92_VA_TP, SZ_4K, "w55fa92-ts")) 
	{
		return -EBUSY;
	}
	DBG_PRINTF("requet memory region successful\n");
	
	if (!(w55fa92_dev = input_allocate_device())) {
		printk(KERN_ERR "w55fa92_dev: not enough memory\n");
		err = -ENOMEM;
		goto fail;
	}
	DBG_PRINTF("allocate w55fa92_dev successful\n");

	w55fa92_dev->name = "W55FA92 TouchScreen";
	w55fa92_dev->phys = "w55fa92/event0";
	w55fa92_dev->id.bustype = BUS_HOST;
	w55fa92_dev->id.vendor  = 0x0005;
	w55fa92_dev->id.product = 0x0001;
	w55fa92_dev->id.version = 0x0100;

	w55fa92_dev->open    = w55fa92ts_open;
	w55fa92_dev->close   = w55fa92ts_close;

	w55fa92_dev->evbit[0] = BIT(EV_KEY) | BIT(EV_ABS) | BIT(EV_SYN);
	w55fa92_dev->keybit[LONG(BTN_TOUCH)] = BIT(BTN_TOUCH);
	input_set_abs_params(w55fa92_dev, ABS_X, 0, 0x1000, 0, 0);
	input_set_abs_params(w55fa92_dev, ABS_Y, 0, 0x1000, 0, 0);
	input_set_abs_params(w55fa92_dev, ABS_PRESSURE, 0, 1000, 0, 0);  	

	input_register_device(w55fa92_dev);
	DBG_PRINTF("Register touch screen successful\n");

#ifdef CONFIG_BATTERY_DETECTION
	w55fa92_vol = PowerOnBatteryDetection(); 
#endif

	return 0;

fail:	
	input_free_device(w55fa92_dev);
	
	ERRLEAVE();
	return err;
}

static int w55fa92ts_remove(struct platform_device *pdev)
{
	ENTER();	
	input_unregister_device(w55fa92_dev);
	LEAVE();
    return 0;
}

static struct platform_driver w55fa92ts_driver = {
	.probe		= w55fa92ts_probe,
	.remove		= w55fa92ts_remove,
	.driver		= {
		.name	= "w55fa92-ts",
		.owner	= THIS_MODULE,
	},
};



#ifdef CONFIG_BATTERY_DETECTION

static ssize_t read_adc(struct device *dev, 
							struct device_attribute *attr, 
							char *buf)
{
	ENTER();	
	if(do_check == 0)
	{
		buf[0] = 0;			
		buf[1] = 0;
		buf[2] = 0;			
		buf[3] = 0;
	}else if(do_check == 1){
	    buf[0] = (char)(w55fa92_lux & 0xff);			//channel 2
		buf[1] = (char)((w55fa92_lux & 0xff00) >> 8);
		buf[2] = (char)(w55fa92_vol & 0xff);			//channel 1
		buf[3] = (char)((w55fa92_vol & 0xff00) >> 8);
	}else if(do_check == 2){
		buf[0] = (char)(w55fa92_channel4 & 0xff);			
		buf[1] = (char)((w55fa92_channel4 & 0xff00) >> 8);	//channel 4
		buf[2] = (char)(w55fa92_channel3 & 0xff);
		buf[3] = (char)((w55fa92_channel3 & 0xff00) >> 8);	//channel 3
	}else if(do_check == 3){
		buf[0] = (char)(w55fa92_channel6 & 0xff);			
		buf[1] = (char)((w55fa92_channel6 & 0xff00) >> 8);	//channel 6
		buf[2] = (char)(w55fa92_channel5 & 0xff);
		buf[3] = (char)((w55fa92_channel5 & 0xff00) >> 8);	//channel 5
	}else if(do_check == 4){
		buf[0] = 0;			
		buf[1] = 0;	//channel 8
		buf[2] = (char)(w55fa92_channel7 & 0xff);
		buf[3] = (char)((w55fa92_channel7 & 0xff00) >> 8);	//channel 7
	}


	//w55fa92_lux = w55fa92_lux+2;
	//w55fa92_vol = w55fa92_vol+1;
    return 4;
}

static ssize_t write_adc(struct device *dev, 
							struct device_attribute *attr, 
							const char *buffer, 
							size_t count)
{
	ENTER();	

	if(buffer[0] == '0') {
		do_check = 0;
	}else if(buffer[0] == '1') {
		do_check = 1;	
	}else if(buffer[0] == '2') {
		do_check = 2;	
	}else if(buffer[0] == '3') {
		do_check = 3;	
	}else if(buffer[0] == '4') {
		do_check = 4;	
	}
	LEAVE();
	return count;
}


/* Attach the sysfs read method */
DEVICE_ATTR(adc, 0622, read_adc, write_adc);

/* Attribute Descriptor */
static struct attribute *adc_attrs[] = {
        &dev_attr_adc.attr,
        NULL
};

/* Attribute group */
static struct attribute_group adc_attr_group = {
        .attrs = adc_attrs,
         };

#endif

//int __devinit w55fa92ts_init(void)
static int __init w55fa92ts_init(void)
{
	int ret;

    ENTER();	

#ifdef CONFIG_BATTERY_DETECTION
	printk("Register platform device for low battery detection\n");	
    sys_adc = platform_device_register_simple("w55fa92-adc", -1, NULL, 0);
    if(sys_adc == NULL)
        printk("register adc detection module failed\n");
    sysfs_create_group(&sys_adc->dev.kobj, &adc_attr_group);
#endif
	ret = platform_driver_register(&w55fa92ts_driver);

	LEAVE();
	return ret; 
}

static void __exit w55fa92ts_exit(void)
{
	ENTER();	

	platform_driver_unregister(&w55fa92ts_driver);
#ifdef CONFIG_BATTERY_DETECTION
	platform_device_unregister(sys_adc);
#endif
	LEAVE();
}

module_init(w55fa92ts_init);
module_exit(w55fa92ts_exit);
#endif //#if defined(CONFIG_TOUCH_DETECTION) || defined(CONFIG_BATTERY_DETECTION)
MODULE_AUTHOR("PX20 SWChou <SWChou@nuvoton.com>");
MODULE_DESCRIPTION("w55fa92 touch screen driver!");
MODULE_LICENSE("GPL");

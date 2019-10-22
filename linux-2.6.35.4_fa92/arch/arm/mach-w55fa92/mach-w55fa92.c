/*
 * linux/arch/arm/mach-w55fa92/mach-w55fa92.c
 *
 * Based on mach-s3c2410/mach-smdk2410.c by Jonas Dietsche
 *
 * Copyright (C) 2008 Nuvoton technology corporation.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation;version 2 of the License.
 *   history:
 *     Wang Qiang (rurality.linux@gmail.com) add LCD support
 *
 */

#include <linux/platform_device.h>
#include <linux/delay.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach-types.h>

#include <mach/map.h>
#include <mach/fb.h>
#include <mach/irqs.h>
#include <mach/serial.h>
#include <mach/w55fa92_reg.h>
#include "cpu.h"
#include <linux/i2c.h> /* sw add */

//#define DBG
#ifdef DBG
#define ENTER()		printk("ENTER : %s  : %s\n",__FILE__, __FUNCTION__)
#define LEAVE()		printk("LEAVE : %s  : %s\n",__FILE__, __FUNCTION__)
#define ERRLEAVE()	printk("ERRLEAVE : %s  : %s\n",__FILE__, __FUNCTION__)
#else
#define ENTER(...)
#define LEAVE(...)	
#define ERRLEAVE(...)	
#endif

static struct w55fa92_uartcfg w55fa92_uartcfgs[] = {
	[0] = {
		.hwport	     = 0,
		.flags	     = 0,
		.ucon	     = 0,
		.ulcon	     = 0,
		.ufcon	     = 0,
	},
	[1] = {
		.hwport	     = 1,
		.flags	     = 0,
		.ucon	     = 0,
		.ulcon	     = 0,
		.ufcon	     = 0,
	}
};

/* I2C clients */ /* sw add */
static struct i2c_board_info __initdata w55fa92_i2c_clients[] = {

        {
                I2C_BOARD_INFO("nau8822", 0x1a),
        },
        
#ifdef CONFIG_SND_SOC_W55FA92_SPU
        {
                I2C_BOARD_INFO("w55fa92_dac", 0x7f),     //            
        },
#endif       

#if defined(CONFIG_FM23)
	{
		I2C_BOARD_INFO("fm23", 0x60),
	},
#endif

#ifdef CONFIG_SND_SOC_W55FA92_ADC
        {
                I2C_BOARD_INFO("w55fa92_adc_i2c", 0x6f),     //            
        },
#endif 
      
#if defined(CONFIG_SENSOR_OV7670_DEV1)
		{
                I2C_BOARD_INFO("ov7670", 0x21),
        },
#endif
#if defined(CONFIG_SENSOR_OV9660_DEV1)
		{
                I2C_BOARD_INFO("ov9660", 0x30),
        },
#endif
#if defined(CONFIG_SENSOR_OV7725_DEV1)
		{
                I2C_BOARD_INFO("ov7725", 0x21),
        },
#endif 
#if defined(CONFIG_SENSOR_NT99050_DEV1) 
		{
                I2C_BOARD_INFO("nt99050", 0x21),
        },
#endif 
#if defined(CONFIG_SENSOR_NT99140_DEV1) 
		{
                I2C_BOARD_INFO("nt99140", 0x2a),
        },
#endif
#if defined(CONFIG_SENSOR_NT99141_DEV1) 
		{
                I2C_BOARD_INFO("nt99141", 0x2a),
        },
#endif
#if defined(CONFIG_SENSOR_NT99142_DEV1) 
		{
                I2C_BOARD_INFO("nt99142", 0x2a),
        },
#endif
#if defined(CONFIG_SENSOR_NT99160_DEV1) 
		{
                I2C_BOARD_INFO("nt99160", 0x2a),
        },
#endif
#if defined(CONFIG_SENSOR_NT99252_DEV1) 
		{
                I2C_BOARD_INFO("nt99252", 0x36),
        },
#endif
#if defined(CONFIG_SENSOR_NT99240_DEV1) 
		{
                I2C_BOARD_INFO("nt99240", 0x36),
        },
#endif
#if defined(CONFIG_SENSOR_NT99340_DEV1) 
		{
                I2C_BOARD_INFO("nt99340", 0x3b),
        },
#endif
#if defined(CONFIG_SENSOR_HM1375_DEV1)
		{
                I2C_BOARD_INFO("hm1375", 0x24),
        },
#endif
#if defined(CONFIG_SENSOR_HM1055_DEV1)
		{
                I2C_BOARD_INFO("hm1055", 0x24),
        },
#endif
#if defined(CONFIG_SENSOR_SP1628_DEV1)
		{
                I2C_BOARD_INFO("sp1628", 0x3C),
        },
#endif
#if defined(CONFIG_SENSOR_TW9912_DEV1)	/* If pin SIAD high, 7 bits slave address = 0x45 */	
		{								/* If pin SIAD low, 7 bits slave address = 0x44 */	
                I2C_BOARD_INFO("tw9912", 0x44),
        },
#endif
#if defined(CONFIG_SENSOR_GC0308_DEV1)											
		{								
                I2C_BOARD_INFO("gc0308", 0x21),	
        },
#endif
#if defined(CONFIG_SENSOR_GC0303_DEV1)											
		{								
                I2C_BOARD_INFO("gc0303", 0x18),	
        },
#endif
#if defined(CONFIG_SENSOR_TVP5150_DEV1)											
		{								
                I2C_BOARD_INFO("tvp5150", 0x5d),	
        },
#endif
#if defined(CONFIG_SENSOR_OV10633_DEV1)								
		{								
                I2C_BOARD_INFO("ov10633", 0x30),	
        },
#endif
#if defined(CONFIG_SENSOR_SC1046_DEV1)						
		{								
                I2C_BOARD_INFO("sc1046", 0x30),	
        },
#endif
#if defined(CONFIG_SENSOR_GM7150_DEV1)						
		{								
                I2C_BOARD_INFO("gm7150", 0x5D),	
        },
#endif
#if defined(CONFIG_SENSOR_TW9900_DEV1)						
		{								
                I2C_BOARD_INFO("tw9900", 0x45),	
        },
#endif
#if defined(CONFIG_SENSOR_XC7021_DEV1) 
		{
                I2C_BOARD_INFO("xc7021", 0x1B),
        },
#endif

#if defined(CONFIG_SENSOR_SC2133_DEV1) || defined(CONFIG_SENSOR_SC2033_DEV1) || defined(CONFIG_SENSOR_SC1143_DEV1)
		{
                I2C_BOARD_INFO("sc2133", 0x30),
        },
#endif
#if defined(CONFIG_SENSOR_OV2710_DEV1)
		{
                I2C_BOARD_INFO("sc2133", 0x36),
        },
#endif
#if defined(CONFIG_SENSOR_PO2210N_DEV1)
		{
                I2C_BOARD_INFO("po2210n", 0x77),
        },
#endif

#if defined(CONFIG_SENSOR_OV7670_DEV2)
		{
                I2C_BOARD_INFO("ov7670_dev2", 0x21),
        },
#endif
#if defined(CONFIG_SENSOR_OV9660_DEV2)
		{
                I2C_BOARD_INFO("ov9660_dev2", 0x30),
        },
#endif
#if defined(CONFIG_SENSOR_OV7725_DEV2)
		{
                I2C_BOARD_INFO("ov7725_dev2", 0x21),
        },
#endif 
#if defined(CONFIG_SENSOR_NT99050_DEV2)
		{
                I2C_BOARD_INFO("nt99050_dev2", 0x21),
        },
#endif 
#if defined(CONFIG_SENSOR_NT99140_DEV2)
		{
                I2C_BOARD_INFO("nt99140_dev2", 0x2a),
        },
#endif
#if defined(CONFIG_SENSOR_NT99141_DEV2)
		{
                I2C_BOARD_INFO("nt99141_dev2", 0x2a),
        },
#endif
#if defined(CONFIG_SENSOR_NT99142_DEV2) 
		{
                I2C_BOARD_INFO("nt99142_dev2", 0x2a),
        },
#endif
#if defined(CONFIG_SENSOR_NT99160_DEV2)
		{
                I2C_BOARD_INFO("nt99160_dev2", 0x2a),
        },
#endif
#if defined(CONFIG_SENSOR_NT99252_DEV2)
		{
                I2C_BOARD_INFO("nt99252_dev2", 0x36),
        },
#endif
#if defined(CONFIG_SENSOR_NT99240_DEV2)
		{
                I2C_BOARD_INFO("nt99240_dev2", 0x36),
        },
#endif
#if defined(CONFIG_SENSOR_NT99340_DEV2)
		{
                I2C_BOARD_INFO("nt99340_dev2", 0x3b),
        },
#endif
#if defined(CONFIG_SENSOR_HM1375_DEV2)
		{
                I2C_BOARD_INFO("hm1375_dev2", 0x24),
        },
#endif
#if defined(CONFIG_SENSOR_HM1055_DEV2)
		{
                I2C_BOARD_INFO("hm1055_dev2", 0x24),
        },
#endif
#if defined(CONFIG_SENSOR_SP1628_DEV2)
		{
                I2C_BOARD_INFO("sp1628_dev2", 0x3C),
        },
#endif
#if defined(CONFIG_SENSOR_TW9912_DEV2)	/* If pin SIAD high, 7 bits slave address = 0x45 */										
		{								/* If pin SIAD low, 7 bits slave address = 0x44 */	
                I2C_BOARD_INFO("tw9912_dev2", 0x44),	
        },
#endif
#if defined(CONFIG_SENSOR_GC0308_DEV2)								
		{								
                I2C_BOARD_INFO("gc0308_dev2", 0x21),	
        },
#endif
#if defined(CONFIG_SENSOR_OV10633_DEV2)						
		{								
                I2C_BOARD_INFO("ov10633_dev2", 0x30),	
        },
#endif
#if defined(CONFIG_SENSOR_SC1046_DEV2)						
		{								
                I2C_BOARD_INFO("sc1046_dev2", 0x30),	
        },
#endif
#if defined(CONFIG_SENSOR_GM7150_DEV2)						
		{								
                I2C_BOARD_INFO("gm7150_dev2", 0x5D),	
        },
#endif
#if defined(CONFIG_SENSOR_TW9900_DEV2)						
		{								
                I2C_BOARD_INFO("tw9900_dev2", 0x45),	
        },
#endif
#if defined(CONFIG_SENSOR_TVP5150_DEV2)						
		{								
                I2C_BOARD_INFO("tvp5150_dev2", 0x45),	
        },
#endif
#if defined(CONFIG_SENSOR_XC7021_DEV2) 
		{
                I2C_BOARD_INFO("xc7021_dev2", 0x1B),
        },
#endif
#if defined(CONFIG_SENSOR_SC2133_DEV2) || defined(CONFIG_SENSOR_SC2033_DEV2) || defined(CONFIG_SENSOR_SC1143_DEV2)
		{
                I2C_BOARD_INFO("sc2133_dev2", 0x30),
        },
#endif
#if defined(CONFIG_SENSOR_OV2710_DEV2)
		{
                I2C_BOARD_INFO("sc2133_dev2", 0x36),
        },
#endif
#if defined(CONFIG_SENSOR_PO2210N_DEV2)
		{
                I2C_BOARD_INFO("po2210n_dev2", 0x77),
        },
#endif

#ifdef CONFIG_EEPROM_AT24
        {
                I2C_BOARD_INFO("at24", 0x50),
                .type="24c16",
        },        
#endif
#ifdef CONFIG_I2C_TS
        {
                I2C_BOARD_INFO("tsc2007", 0x48),
                .type = "tsc2007",
                .platform_data = &nuc900_tsc2007_data,
                .irq = IRQ_GROUP0,
        },
#endif 
#ifdef CONFIG_RTC_DRV_PCF8563        
        {
		I2C_BOARD_INFO("rtc-pcf8563", 0x51),
		.type = "pcf8563",
        },
#endif        
#ifdef CONFIG_RTC_DRV_HT1382
        {
                I2C_BOARD_INFO("rtc-ht1382", 0x68),
                .type = "ht1382",
        },        
#endif
};

#if defined (CONFIG_I2C_BUS_W55FA92) && defined (CONFIG_I2C_GPIO_W55FA92)
static struct i2c_board_info __initdata w55fa92_i2c_port1_clients[] = {


};
#endif

extern void w55fa92_poweroff(void);
static void __init mach_w55fa92_map_io(void)
	
{
	w55fa92_map_io();
	w55fa92_init_clocks();
	w55fa92_init_uarts(w55fa92_uartcfgs, ARRAY_SIZE(w55fa92_uartcfgs));

	pm_power_off = w55fa92_poweroff;
}

static void __init w55fa92_init(void)
{
	w55fa92_board_init();
	ENTER();
	i2c_register_board_info(0, w55fa92_i2c_clients, sizeof(w55fa92_i2c_clients)/sizeof(struct i2c_board_info)); /* sw add */
#if defined (CONFIG_I2C_BUS_W55FA92) && defined (CONFIG_I2C_GPIO_W55FA92)	
	i2c_register_board_info(1, w55fa92_i2c_port1_clients, sizeof(w55fa92_i2c_port1_clients)/sizeof(struct i2c_board_info));
#endif
	LEAVE();	
}

MACHINE_START(W55FA92, "W55FA92")
	.phys_io	= W55FA92_PA_UART,
	.io_pg_offst	= (((u32)W55FA92_VA_UART) >> 18) & 0xfffc,
#ifdef CONFIG_INITRAMFS_ROOT_UID
	.boot_params	= 0x0,
#else
	.boot_params	= 0x100,
#endif
	.map_io		= mach_w55fa92_map_io,
	.init_irq	= w55fa92_init_irq,
	.init_machine	= w55fa92_init,
	.timer		= &w55fa92_timer,
MACHINE_END

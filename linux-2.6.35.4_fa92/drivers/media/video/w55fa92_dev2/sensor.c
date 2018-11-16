/* sensor.c
 *
 * Copyright (c) 2008 Nuvoton technology corporation
 * All rights reserved.
 * <clyu2@nuvoton.com>
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */


//#include <linux/config.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/list.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/mm.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/smp_lock.h>
#include <linux/vmalloc.h>
#include <linux/init.h>
#include <linux/spinlock.h>
#include <linux/delay.h>

#if 0
#include <linux/videodev.h>
#else
#include <linux/videodev2.h>
#include <media/v4l2-device.h>
#endif 

#include <linux/jiffies.h>
#if 0
#include <asm/arch/videoin.h>
#include <asm/arch/DrvVideoin.h>
#include <asm/arch/w55fa95_reg.h>
#include <asm/arch/fb.h>
#include <asm/arch/w55fa95_fb.h>
#include <asm/arch/w55fa95_gpio.h>
#else
#include <mach/w55fa92_reg.h>
#include <mach/fb.h>
#include <mach/w55fa92_fb.h>
#include <mach/videoin.h>
//#include <mach/DrvVideoin.h>
#include "DrvVideoin.h"
#include <mach/videodev_ex.h>
#include <mach/w55fa92_gpio.h>
#endif

#include <linux/moduleparam.h>
#include <asm/io.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/i2c-id.h>
#include <linux/i2c-dev.h>

#include "videoinpriv.h"
#include "DrvI2C.h"	/* */
#include "DrvVideoin.h"

#include <media/v4l2-chip-ident.h>
#include <media/v4l2-common.h>
#include <media/soc_camera.h>
 

#define _REG_TABLE_SIZE(nTableName)	sizeof(nTableName)/sizeof(struct OV_RegValue)

#define ERR_PRINTF			printk


//#define DBG
#ifdef DBG
#define ENTER()		printk("ENTER : %s  : %s\n",__FILE__, __FUNCTION__)
#define LEAVE()		printk("LEAVE : %s  : %s\n",__FILE__, __FUNCTION__)
#define ERRLEAVE()	printk("ERRLEAVE : %s  : %s : %d\n",__FILE__, __FUNCTION__, __LINE__)
#define SDBG		printk	
#else
#define ENTER(...)
#define LEAVE(...)	
#define ERRLEAVE(...)	
#define SDBG(...)
#endif

extern unsigned int w55fa92_upll_clock, w55fa92_apll_clock, w55fa92_ahb_clock;

#ifndef CONFIG_NON_STANDARD_I2C_DEV2
#define __STANDARD_I2C__
#endif

struct OV_RegValue{
	__u8	uRegAddr;
	__u8	uValue;
};

struct OV_RegTable{
	struct OV_RegValue *sRegTable;
	__u16 uTableSize;
};

#ifdef CONFIG_SENSOR_OV7670_DEV2
static struct OV_RegValue g_sOV7670_RegValue[] = 
{
	{0x12, 0x80},{0x11, 0x80},{0x3A, 0x04},{0x12, 0x00},{0x17, 0x13},{0x18, 0x01},{0x32, 0xB6}, 
	{0x2B, 0x10},{0x19, 0x02},{0x1A, 0x7A},{0x03, 0x0F},{0x0C, 0x00},{0x3E, 0x00},{0x70, 0x3A},
	{0x71, 0x35},{0x72, 0x11},{0x73, 0xF0},{0xA2, 0x3B},{0x1E, 0x07},{0x7a, 0x1e},{0x7b, 0x09},
	{0x7c, 0x14},{0x7d, 0x29},{0x7e, 0x50},{0x7f, 0x5F},{0x80, 0x6C},{0x81, 0x79},{0x82, 0x84},
	{0x83, 0x8D},{0x84, 0x96},{0x85, 0xA5},{0x86, 0xB0},{0x87, 0xC6},{0x88, 0xD8},{0x89, 0xE9},
	{0x55, 0x00},{0x13, 0xE0},{0x00, 0x00},{0x10, 0x00},{0x0D, 0x40},{0x42, 0x00},{0x14, 0x18},
	{0xA5, 0x02},{0xAB, 0x03},{0x24, 0x48},{0x25, 0x40},{0x26, 0x82},{0x9F, 0x78},{0xA0, 0x68},
	{0xA1, 0x03},{0xA6, 0xd2},{0xA7, 0xd2},{0xA8, 0xF0},{0xA9, 0x80},{0xAA, 0x14},{0x13, 0xE5},
	{0x0E, 0x61},{0x0F, 0x4B},{0x16, 0x02},{0x21, 0x02},{0x22, 0x91},{0x29, 0x07},{0x33, 0x0B},
	{0x35, 0x0B},{0x37, 0x1D},{0x38, 0x71},{0x39, 0x2A},{0x3C, 0x78},{0x4D, 0x40},{0x4E, 0x20},
	{0x69, 0x00},{0x6B, 0x0A},{0x74, 0x10},{0x8D, 0x4F},{0x8E, 0x00},{0x8F, 0x00},{0x90, 0x00},
	{0x91, 0x00},{0x96, 0x00},{0x9A, 0x80},{0xB0, 0x84},{0xB1, 0x0C},{0xB2, 0x0E},{0xB3, 0x7e},
	{0xB1, 0x00},{0xB1, 0x0c},{0xB8, 0x0A},{0x44, 0xfF},{0x43, 0x00},{0x45, 0x4a},{0x46, 0x6c},
	{0x47, 0x26},{0x48, 0x3a},{0x59, 0xd6},{0x5a, 0xff},{0x5c, 0x7c},{0x5d, 0x44},{0x5b, 0xb4},
	{0x5e, 0x10},{0x6c, 0x0a},{0x6d, 0x55},{0x6e, 0x11},{0x6f, 0x9e},{0x6A, 0x40},{0x01, 0x40},
	{0x02, 0x40},{0x13, 0xf7},{0x4f, 0x78},{0x50, 0x72},{0x51, 0x06},{0x52, 0x24},{0x53, 0x6c},
	{0x54, 0x90},{0x58, 0x1e},{0x62, 0x08},{0x63, 0x10},{0x64, 0x08},{0x65, 0x00},{0x66, 0x05},
	{0x41, 0x08},{0x3F, 0x00},{0x75, 0x44},{0x76, 0xe1},{0x4C, 0x00},{0x77, 0x01},{0x3D, 0xC2},
	{0x4B, 0x09},{0xC9, 0x60},{0x41, 0x08/*0x18*/},{0x56, 0x40},{0x34, 0x11},{0x3b, 0x02},{0xa4, 0x89},
	{0x92, 0x00},{0x96, 0x00},{0x97, 0x30},{0x98, 0x20},{0x99, 0x20},{0x9A, 0x84},{0x9B, 0x29},
	{0x9C, 0x03},{0x9D, 0x99},{0x9E, 0x7F},{0x78, 0x00},{0x94, 0x08},{0x95, 0x0D},{0x79, 0x01},
	{0xc8, 0xf0},{0x79, 0x0f},{0xc8, 0x00},{0x79, 0x10},{0xc8, 0x7e},{0x79, 0x0a},{0xc8, 0x80},
	{0x79, 0x0b},{0xc8, 0x01},{0x79, 0x0c},{0xc8, 0x0f},{0x79, 0x0d},{0xc8, 0x20},{0x79, 0x09},
	{0xc8, 0x80},{0x79, 0x02},{0xc8, 0xc0},{0x79, 0x03},{0xc8, 0x40},{0x79, 0x05},{0xc8, 0x30},
	{0x79, 0x26},{0x3b, 0x82},{0x43, 0x02},{0x44, 0xf2}
};
#endif
#ifdef CONFIG_SENSOR_OV9660_DEV2
static struct OV_RegValue g_sOV9660_RegValue[]=
{//OV9660
		{0x12, 0x80},
	#if 1
		{0xd5, 0xff}, {0xd6, 0x3f}, {0x3d, 0x3c}, {0x11, 0x80},	{0x2a, 0x00}, {0x2b, 0x00},	//PCLK = SCLK
	#else		
		{0xd5, 0xff}, {0xd6, 0x3f}, {0x3d, 0x3c}, {0x11, 0x81},	{0x2a, 0x00}, {0x2b, 0x00},	//PCLK = SCLK/2
	#endif	
		{0x3a, 0xd9}, {0x3b, 0x00},	{0x3c, 0x58}, {0x3e, 0x50},	{0x71, 0x00}, {0x15, 0x00},
		{0xD7, 0x10}, {0x6a, 0x24},	{0x85, 0xe7}, {0x63, 0x00}, {0x12, 0x40}, {0x4d, 0x09},
	#if 0		
		{0x17, 0x0c}, {0x18, 0x5c},	{0x19, 0x02}, {0x1a, 0x3f},	{0x03, 0x03}, {0x32, 0xb4},	//641*48?
	#else
		{0x17, 0x0b}, {0x18, 0x5c},	{0x19, 0x02}, {0x1a, 0x3f},	{0x03, 0x03}, {0x32, 0xb4}, //648x48?
	#endif		
		{0x2b, 0x00}, {0x5c, 0x80},	{0x36, 0xb4}, {0x65, 0x10}, {0x70, 0x02}, {0x71, 0x9f},
		{0x64, 0xa4}, {0x5c, 0x80},	{0x43, 0x00}, {0x5D, 0x55}, {0x5E, 0x57}, {0x5F, 0x21},
		{0x24, 0x3e}, {0x25, 0x38},	{0x26, 0x72}, {0x14, 0x68}, {0x0C, 0x38}, {0x4F, 0x4f},
		{0x50, 0x42}, {0x5A, 0x67}, {0x7d, 0x30}, {0x7e, 0x00}, {0x82, 0x03}, {0x7f, 0x00},
		{0x83, 0x07}, {0x80, 0x03}, {0x81, 0x04}, {0x96, 0xf0}, {0x97, 0x00}, {0x92, 0x33},
		{0x94, 0x5a}, {0x93, 0x3a},	{0x95, 0x48}, {0x91, 0xfc}, {0x90, 0xff}, {0x8e, 0x4e},
		{0x8f, 0x4e}, {0x8d, 0x13},	{0x8c, 0x0c}, {0x8b, 0x0c},	{0x86, 0x9e}, {0x87, 0x11},
		{0x88, 0x22}, {0x89, 0x05},	{0x8a, 0x03}, {0x9b, 0x0e},	{0x9c, 0x1c}, {0x9d, 0x34},
		{0x9e, 0x5a}, {0x9f, 0x68},	{0xa0, 0x76}, {0xa1, 0x82},	{0xa2, 0x8e}, {0xa3, 0x98},
		{0xa4, 0xa0}, {0xa5, 0xb0},	{0xa6, 0xbe}, {0xa7, 0xd2},	{0xa8, 0xe2}, {0xa9, 0xee},
		{0xaa, 0x18}, {0xAB, 0xe7},	{0xb0, 0x43}, {0xac, 0x04},	{0x84, 0x40}, {0xad, 0x82},
   #if 0		
		{0xd9, 0x11}, {0xda, 0x00},	{0xae, 0x10}, {0xab, 0xe7},	{0xb9, 0x50}, {0xba, 0x3c},		//641*48?	
		{0xbb, 0x50}, {0xbc, 0x3c},	{0xbd, 0x8},  {0xbe, 0x19},	{0xbf, 0x2},  {0xc0, 0x8},
   #else
		{0xd9, 0x11}, {0xda, 0x00},	{0xae, 0x10}, {0xab, 0xe7},	{0xb9, 0x51}, {0xba, 0x3c},		//648x48?
		{0xbb, 0x51}, {0xbc, 0x3c},	{0xbd, 0x8},  {0xbe, 0x19},	{0xbf, 0x2},  {0xc0, 0x8},	
   #endif		
		{0xc1, 0x2a}, {0xc2, 0x34},	{0xc3, 0x2d}, {0xc4, 0x2d},	{0xc5, 0x0},  {0xc6, 0x98},
		{0xc7, 0x18}, {0x69, 0x48},	{0x74, 0xc0}, {0x7c, 0x28},	{0x65, 0x11}, {0x66, 0x00},
		{0x41, 0xc0}, {0x5b, 0x24},	{0x60, 0x82}, {0x05, 0x07},	{0x03, 0x03}, {0xd2, 0x94},
		{0xc8, 0x06}, {0xcb, 0x40},	{0xcc, 0x40}, {0xcf, 0x00},	{0xd0, 0x20}, {0xd1, 0x00},
		{0xc7, 0x18}, {0x0d, 0x92},	{0x0d, 0x90}		
};
#endif
#ifdef CONFIG_SENSOR_OV7725_DEV2
static struct OV_RegValue g_sOV7725_RegValue[] = 
{
	//NewKen sensor module initial for OV7725 20110613
		{0x12, 0x80},
//		{0x00, 0x00}, {0x00, 0x00}, {0x00, 0x64},	 	//Newken only
		{0x12, 0x00}, {0x3D, 0x03},
		{0x17, 0x22}, {0x18, 0xA4}, {0x19, 0x07}, {0x1A, 0xF0},
		{0x32, 0x02}, {0x29, 0xA0}, {0x2C, 0xF0},
		{0x2A, 0x02}, {0x65, 0x20}, {0x11, 0x01},
//		{0x15, 0x03},								//Newken olny
		{0x42, 0x7F}, {0x63, 0xE0}, {0x64, 0xFF},
		{0x66, 0x00},		
//		{0x66, 0xC0}, 								// For Video_In
		{0x67, 0x48}, {0x0D, 0x41},
		{0x0E, 0x01}, {0x0F, 0xC5}, {0x14, 0x11},
		{0x22, 0x7F}, {0x23, 0x03}, {0x24, 0x40},
		{0x25, 0x30}, {0x26, 0xA1}, {0x2B, 0x00},
		{0x6B, 0xAA}, {0x13, 0xEF}, {0x90, 0x05},
		{0x91, 0x01}, {0x92, 0x03}, {0x93, 0x00},
		{0x94, 0x90}, {0x95, 0x8A}, {0x96, 0x06},
		{0x97, 0x0B}, {0x98, 0x95}, {0x99, 0xA0},
		{0x9A, 0x1E}, {0x9B, 0x08}, {0x9C, 0x20},
		{0x9E, 0x81}, {0xA6, 0x04}, {0x7E, 0x0C},
		{0x7F, 0x24}, {0x80, 0x3A}, {0x81, 0x60},
		{0x82, 0x70}, {0x83, 0x7E}, {0x84, 0x8A},
		{0x85, 0x94}, {0x86, 0x9E}, {0x87, 0xA8},
		{0x88, 0xB4}, {0x89, 0xBE}, {0x8A, 0xCA},
		{0x8B, 0xD8}, {0x8C, 0xE2}, {0x8D, 0x28},
		{0x46, 0x05}, {0x47, 0x00}, {0x48, 0x00},
		{0x49, 0x12}, {0x4A, 0x00}, {0x4B, 0x13},
		{0x4C, 0x21}, 
		{0x0C, 0x10}, 
//		{0x0C, 0x00},								// For Video_In
		{0x09, 0x00},
		{0xFF, 0xFF}, {0xFF, 0xFF}	
};
#endif 
static struct OV_RegTable g_OV_InitTable[] =
{
#ifdef CONFIG_SENSOR_OV9660_DEV2
	{g_sOV9660_RegValue,_REG_TABLE_SIZE(g_sOV9660_RegValue)},		
#elif defined  CONFIG_SENSOR_OV7670_DEV2
	{g_sOV7670_RegValue,_REG_TABLE_SIZE(g_sOV7670_RegValue)},
#elif defined  CONFIG_SENSOR_OV7725_DEV2
	{g_sOV7725_RegValue,_REG_TABLE_SIZE(g_sOV7725_RegValue)},
#else
	{0,0}
#endif
};

static __u8 g_uOvDeviceID[]= 
{
#ifdef CONFIG_SENSOR_OV9660_DEV2
	0x60,		// 0v9660
#elif defined  CONFIG_SENSOR_OV7670_DEV2
	0x42,		// ov7670	
#elif defined  CONFIG_SENSOR_OV7725_DEV2
	0x42,		// ov7725
#else
	0x42		// not a device ID
#endif
};

#ifdef __STANDARD_I2C__
static struct i2c_client *save_client;
static struct i2c_client i2c_client; 
static int sensor_probe(struct i2c_client *client,
			const struct i2c_device_id *did)
{
	int ret = 0;
	
	ENTER();
	printk("%s\n", __FUNCTION__);
	memcpy(&i2c_client, client, sizeof(struct i2c_client));
	save_client = &i2c_client;
	LEAVE();
	return ret;
}

static int sensor_remove(struct i2c_client *client)
{
	ENTER();
	LEAVE();
	return 0;
}

static const struct i2c_device_id sensor_id[] = {
#ifdef CONFIG_SENSOR_OV9660_DEV2
	{ "ov9660_dev2", 0 },
#elif defined  CONFIG_SENSOR_OV7670_DEV2
	{ "ov7670_dev2", 0 },
#elif defined  CONFIG_SENSOR_OV7725_DEV2
	{ "ov7725_dev2", 0 },
#endif
	{ }
};
MODULE_DEVICE_TABLE(i2c, sensor_id);

static struct i2c_driver sensor_i2c_driver = {
#ifdef CONFIG_SENSOR_OV9660_DEV2
	.driver = {
		.name = "ov9660_dev2",
	},
#elif defined  CONFIG_SENSOR_OV7670_DEV2
	.driver = {
		.name = "ov7670_dev2",
	},
#elif defined  CONFIG_SENSOR_OV7725_DEV2
	.driver = {
		.name = "ov7725_dev2",
	},
#endif
	.probe    = sensor_probe,
	.remove   = sensor_remove,
	.id_table = sensor_id,
};
#endif

#ifndef CONFIG_SHARE_SENSOR	
static void schedule_mdelay(UINT32 u32Delay_ms)
{
	unsigned long j=0;	
	j = jiffies + u32Delay_ms*HZ/1000; 	/* 2Xms~30ms */			
	while( time_before(jiffies,j) )
		schedule();	
}
#endif 
#ifndef __STANDARD_I2C__
static void I2C_Delay(UINT32 u32Delay)
{
	volatile UINT32 i;
	for(;u32Delay!=0;u32Delay--)
		for(i=0;i<5;i++);
}

static BOOL 
I2C_Write_8bitSlaveAddr_8bitReg_8bitData(UINT8 uAddr, UINT8 uRegAddr, UINT8 uData)
{
	volatile unsigned int u32Delay = 0x100;

	ENTER();
	DrvI2C_SendStart();
	while(u32Delay--);		
	if ( (DrvI2C_WriteByte(uAddr,DrvI2C_Ack_Have,8)==FALSE) ||			// Write ID address to sensor
		 (DrvI2C_WriteByte(uRegAddr,DrvI2C_Ack_Have,8)==FALSE) ||	// Write register address to sensor
		 (DrvI2C_WriteByte(uData,DrvI2C_Ack_Have,8)==FALSE) )		// Write data to sensor
	{
		DrvI2C_SendStop();
		
		printk("Non-standard I2C error, Slave addr: reg_addr = 0x%x : 0x%x\n", uAddr, uRegAddr);
		return FALSE;
	}
	DrvI2C_SendStop();
	if (uRegAddr==0x12 && (uData&0x80)!=0)
	{
		mdelay(20);			
	}
	LEAVE();
	return TRUE;
}


static UINT8 I2C_Read_8bitSlaveAddr_8bitReg_8bitData(UINT8 uAddr, UINT8 uRegAddr)
{

	UINT8 u8Data;
	volatile UINT32 u32Delay = 0x100;
	
	ENTER();

	// 2-Phase(ID address, register address) write transmission
	DrvI2C_SendStart();
	while(u32Delay--);		
	DrvI2C_WriteByte(uAddr,DrvI2C_Ack_Have,8);		// Write ID address to sensor
	DrvI2C_WriteByte(uRegAddr,DrvI2C_Ack_Have,8);	// Write register address to sensor
	DrvI2C_SendStop();

	// 2-Phase(ID-address, data(8bits)) read transmission
	DrvI2C_SendStart();
	DrvI2C_WriteByte(uAddr|0x01,DrvI2C_Ack_Have,8);		// Write ID address to sensor
	u8Data = DrvI2C_ReadByte(DrvI2C_Ack_Have,8);		// Read data from sensor
	DrvI2C_SendStop();
  
	LEAVE();
	return u8Data;

}
#endif 


static UINT8 DrvVideoIn_I2cWriteOV(__u8 uAddr, __u8 uRegAddr, __u8 uData)
{
	
	int ret=0;	
#ifdef __STANDARD_I2C__
	struct i2c_msg msg;
	u8 buf[2];
	volatile int i;
	#ifdef CONFIG_SHARE_SENSOR
	return 0;
	#endif				
	printk("save client = %d", (unsigned int)save_client);

	msg.flags=!I2C_M_RD;
	msg.addr=save_client->addr;
	msg.len=2;
	msg.buf=buf;		

	buf[0]=(u8)uRegAddr;
	buf[1]=uData;

	ret=i2c_transfer(save_client->adapter,&msg,1);
	for(i=0;i<10; i=i+1)
		udelay(500);
	return ret;
#else
	#ifdef CONFIG_SHARE_SENSOR
	return 0;
	#endif				
	I2C_Write_8bitSlaveAddr_8bitReg_8bitData(uAddr, uRegAddr, uData);
	return ret;
#endif
}
 
static UINT8 DrvVideoIn_I2cReadOV(__u8 uAddr, __u8 uRegAddr)
{
#ifdef __STANDARD_I2C__
		struct i2c_msg msgs;
		int ret=-1;
		u8 buf[3];
		#ifdef CONFIG_SHARE_SENSOR
		return 0;
		#endif					
		msgs.flags=!I2C_M_RD;
		msgs.addr=save_client->addr;
		msgs.len=1;
		msgs.buf=buf;
		buf[0]=(u8)(uRegAddr);

		ret=i2c_transfer(save_client->adapter,&msgs,1);
		
		msgs.flags=I2C_M_RD;
		msgs.addr=save_client->addr;
		msgs.len=1;
		msgs.buf=buf;

		ret=i2c_transfer(save_client->adapter,&msgs,1);
		return buf[0];
#else
		ENTER();
		#ifdef CONFIG_SHARE_SENSOR
		return 0;
		#endif			
		return I2C_Read_8bitSlaveAddr_8bitReg_8bitData(uAddr, uRegAddr);
#endif	
}


static __s32 OvRegConfig(__u32 nIndex)
{
	__u32 i;
	__u16 uTableSize;
	__u8  uDeviceID;
	__s32 res = 0; 
	struct OV_RegValue *psRegValue;
	ENTER();

#ifdef __STANDARD_I2C__
	
#else	
	printk("Non Standard I2C.\n");
  #ifdef CONFIG_DEV_BOARD_DEV2
	DrvI2C_Open(eDRVGPIO_GPIOB, 					
				eDRVGPIO_PIN13, 
				eDRVGPIO_GPIOB,
				eDRVGPIO_PIN14, 
				(PFN_DRVI2C_TIMEDELY)I2C_Delay);
	outp32(REG_GPBFUN1, inp32(REG_GPBFUN1) & (~MF_GPB13));
	outp32(REG_GPBFUN1, inp32(REG_GPBFUN1) & (~MF_GPB14));
  #endif	
	
  #ifdef CONFIG_DEMO_BOARD_DEV2			
	outp32(REG_GPBFUN0, inp32(REG_GPBFUN0) & (~MF_GPB4));
	outp32(REG_GPAFUN1, (inp32(REG_GPAFUN1) & (~MF_GPA12)) | (2<<16));
	DrvI2C_Open(eDRVGPIO_GPIOB, 					
				eDRVGPIO_PIN4, 
				eDRVGPIO_GPIOA,
				eDRVGPIO_PIN12, 
				(PFN_DRVI2C_TIMEDELY)I2C_Delay);	
	outp32(REG_GPAFUN1, (inp32(REG_GPAFUN1) & (~MF_GPA12)) | (2<<16));
  #endif
#endif
	//SDBG("nIndex = %d\n", nIndex);	
	if ( nIndex >= (sizeof(g_uOvDeviceID)/sizeof(__u8)) )
		return -EBUSY;	
	
	uTableSize = g_OV_InitTable[nIndex].uTableSize;
	psRegValue = g_OV_InitTable[nIndex].sRegTable;
	uDeviceID = g_uOvDeviceID[nIndex];

	if ( psRegValue == 0 )
		return -EBUSY;	
	

	for(i=0;i<uTableSize; i++, psRegValue++)
	{
		udelay(10);	
		DrvVideoIn_I2cWriteOV(uDeviceID, (psRegValue->uRegAddr), (psRegValue->uValue));
	}
	if(res>=0)
		printk("driver i2c initial done\n");
	else
		printk("driver i2c initial fail\n");	
	return res;
}

static BOOL bIsI2CAdd = FALSE; 
static int i2c_init(void)
{
	int ret=0;
#ifdef __STANDARD_I2C__	

	ENTER();
	ret = i2c_add_driver(&sensor_i2c_driver);

	if(ret){
		ERRLEAVE();
	}	
	else{		
		bIsI2CAdd = TRUE; 
		printk("I2C added\n");
		LEAVE();
	}
#endif
	return ret;
}
static void i2c_cleanup(void)
{
#ifdef __STANDARD_I2C__	
	ENTER();
	i2c_del_driver(&sensor_i2c_driver);
	bIsI2CAdd = FALSE; 
	LEAVE();
#endif
}


#ifdef CONFIG_W55FA92_VIDEOIN_DEV2
extern void video_port1_pin_init(
	BOOL bIsEnableSnrClock,
	E_VIDEOIN_SNR_SRC eSnrSrc,	
	UINT32 u32SensorFreqKHz,						//KHz unit
	E_VIDEOIN_DEV_TYPE eDevType
	);
#endif 
#ifdef CONFIG_SHARE_SENSOR
extern void GetVideo0SensorResoultion(UINT16* pSenWidth, UINT16* pSenHeight);
#endif
static __s32 InitSensor(__u32 u32Sensor, void *priv)
{
	__u32 u32PacStride, u32PlaStride;
	__s32 res = 0; 
	videoin_priv_t *vin_priv = (videoin_priv_t *) priv;
#ifdef CONFIG_SENSOR_OV9660_DEV2
	printk("Init OV9660 \n"); 		
#elif defined  CONFIG_SENSOR_OV7670_DEV2
	printk("Init OV7670 \n"); 	
#elif defined  CONFIG_SENSOR_OV7725_DEV2
	printk("Init OV7725 \n"); 	
#endif	

#ifdef CONFIG_VIDEO_PORT2_0_DEV2
	printk("Init port 2 \n");
	vin_priv->pDevVin->Init(TRUE, eVIDEOIN_SNR_UPLL, 24000, eVIDEOIN_2ND_SNR_CCIR601);	
#endif															
#ifdef CONFIG_VIDEO_PORT2_1_DEV2
	printk("Init port 2_1 \n");	
	vin_priv->pDevVin->Init(TRUE, eVIDEOIN_SNR_UPLL, 24000, eVIDEOIN_3RD_SNR_CCIR601);
#endif

#ifdef CONFIG_W55FA92_VIDEOIN_DEV1
	{/* If exist the first video capture device  */
		if( (inp32(REG_AHBCLK) & VIN0_CKE) == 0)/* If VideoIn 1 close, do reset */{
			/* 2014-01-10 */	
			video_port1_pin_init(TRUE, eVIDEOIN_SNR_UPLL, 24000, eVIDEOIN_SNR_CCIR601);	
#ifndef CONFIG_SHARE_SENSOR
			Snr2_PowerDown(FALSE);				 /* Reset the first and second sensor */
			Snr2_Reset(TRUE);
#endif						
		}										/* Else do nothing */
	}
#else	
	Snr2_PowerDown(FALSE);
	Snr2_Reset(TRUE);	
#endif	
#ifndef CONFIG_SHARE_SENSOR
	schedule_mdelay(10);
	i2c_init();
	schedule_mdelay(10);
#endif							
	vin_priv->pDevVin->Open(64000, 24000);
	switch (u32Sensor)
	{//OV9660 will be 30f/s @ sensor clock 24MHz for VGA output.  
		case OV_7725:		
			vin_priv->sensor_intf->u8SensorDevID = g_uOvDeviceID[0];//g_uOvDeviceID[pDevVin->ov7725];	 
#ifndef CONFIG_SHARE_SENSOR
			SDBG("Init OV_7725 \n");
			schedule_mdelay(10);
			res = OvRegConfig(0);
			if( res<0 ){
					printk("Sensor initial fail \n");
					return res;	
			}
			else
				printk("Sensor initial successful \n");		
			vin_priv->sensor_intf->u16CurImgHeight = vin_priv->sensor_intf->u16MaxImgHeight;
			vin_priv->sensor_intf->u16CurImgWidth = vin_priv->sensor_intf->u16MaxImgWidth;
#else
			{
				UINT16 u16Width, u16Height;
				/* From DEV1 get sensor output dimension */	
				GetVideo0SensorResoultion(&u16Width, &u16Height);
				vin_priv->sensor_intf->u16CurImgWidth = u16Width;
				vin_priv->sensor_intf->u16CurImgHeight = u16Height;
				/* 2014-01-10 */
				vin_priv->videocropcap.defrect.left = 0;
				vin_priv->videocropcap.defrect.top = 0;
				vin_priv->videocropcap.defrect.width = u16Width;
				vin_priv->videocropcap.defrect.height = u16Height;
			} 
#endif
			vin_priv->videocrop.c.left = 0;	/*X*/
			vin_priv->videocrop.c.top = 0;	/*Y*/	
			vin_priv->videocrop.c.width = vin_priv->sensor_intf->u16CurImgWidth;
			vin_priv->videocrop.c.height = vin_priv->sensor_intf->u16CurImgHeight;	

			vin_priv->videocropcap.bounds.left = 0; 
			vin_priv->videocropcap.bounds.top = 0;
			vin_priv->videocropcap.bounds.width = vin_priv->sensor_intf->u16CurImgWidth; 
			vin_priv->videocropcap.bounds.height = vin_priv->sensor_intf->u16CurImgHeight;

			vin_priv->videocropcap.defrect.left = 0; 
			vin_priv->videocropcap.defrect.top = 0;
			vin_priv->videocropcap.defrect.width = vin_priv->sensor_intf->u16CurImgWidth; 
			vin_priv->videocropcap.defrect.height =  vin_priv->sensor_intf->u16CurImgHeight;

			if(vin_priv->sensor_intf->u16CurImgWidth==1280){
				vin_priv->videocropcap.pixelaspect.numerator = 18;	/* Suppose current image size HD */
				vin_priv->videocropcap.pixelaspect.denominator = 32;	/* Zoomming step */
			}else {
				vin_priv->videocropcap.pixelaspect.numerator = 12;	/* Suppose current image size VGA/SVGA */
				vin_priv->videocropcap.pixelaspect.denominator = 16;	/* Zoomming step */
			}


			vin_priv->pDevVin->Open(72000, 24000);					

			vin_priv->pDevVin->SetBaseStartAddress(eVIDEOIN_PACKET, 0, vin_priv->videoIn_preview_buf[0].u32PhysAddr);
	
			vin_priv->pDevVin->EnableInt(eVIDEOIN_VINT);
			
			vin_priv->pDevVin->SetSensorPolarity(TRUE, 
										FALSE, 
										TRUE);
			vin_priv->pDevVin->SetDataFormatAndOrder(eVIDEOIN_IN_UYVY, 
											eVIDEOIN_IN_YUV422, 									
											eVIDEOIN_OUT_YUV422);											
			vin_priv->pDevVin->SetCropWinStartAddr(0,					//UINT16 u16VerticalStart, 		Y
											0);					//UINT16 u16HorizontalStart, 	X
			/* Sensor subsample resolution (640, 480)*/
			vin_priv->pDevVin->SetCropWinSize(vin_priv->sensor_intf->u16MaxImgHeight,		//UINT16 u16Height, 
									 vin_priv->sensor_intf->u16MaxImgWidth);		//UINT16 u16Width;									 
			
			vin_priv->pDevVin->PreviewPipeSize(vin_priv->videowin.height, vin_priv->videowin.width);
			
			vin_priv->pDevVin->GetStride(&u32PacStride, &u32PlaStride);
			vin_priv->pDevVin->SetStride(vin_priv->videowin.width, u32PlaStride);															
			vin_priv->pDevVin->SetPipeEnable(FALSE,							// It means planar disable
										eVIDEOIN_PACKET);

			vin_priv->pDevVin->SetShadowRegister();
		break;

		case OV_7670:
			vin_priv->sensor_intf->u8SensorDevID = g_uOvDeviceID[0];//g_uOvDeviceID[pDevVin->ov7725];
#ifndef CONFIG_SHARE_SENSOR
			SDBG("Init OV_7670 \n"); 											
			schedule_mdelay(10);
			res = OvRegConfig(0);
			if( res<0 ){
					printk("Sensor initial fail \n");
					return res;	
			}
			else
				printk("Sensor initial successful \n");		
			vin_priv->sensor_intf->u16CurImgHeight = vin_priv->sensor_intf->u16MaxImgHeight;
			vin_priv->sensor_intf->u16CurImgWidth = vin_priv->sensor_intf->u16MaxImgWidth;
#else
			{
				UINT16 u16Width, u16Height;
				/* From DEV1 get sensor output dimension */	
				GetVideo0SensorResoultion(&u16Width, &u16Height);
				vin_priv->sensor_intf->u16CurImgWidth = u16Width;
				vin_priv->sensor_intf->u16CurImgHeight = u16Height;
				/* 2014-01-10 */
				vin_priv->videocropcap.defrect.left = 0;
				vin_priv->videocropcap.defrect.top = 0;
				vin_priv->videocropcap.defrect.width = u16Width;
				vin_priv->videocropcap.defrect.height = u16Height;
			} 
#endif
			vin_priv->videocrop.c.left = 0;	/*X*/
			vin_priv->videocrop.c.top = 0;	/*Y*/	
			vin_priv->videocrop.c.width = vin_priv->sensor_intf->u16CurImgWidth;
			vin_priv->videocrop.c.height = vin_priv->sensor_intf->u16CurImgHeight;	

			vin_priv->videocropcap.bounds.left = 0; 
			vin_priv->videocropcap.bounds.top = 0;
			vin_priv->videocropcap.bounds.width = vin_priv->sensor_intf->u16CurImgWidth; 
			vin_priv->videocropcap.bounds.height = vin_priv->sensor_intf->u16CurImgHeight;

			vin_priv->videocropcap.defrect.left = 0; 
			vin_priv->videocropcap.defrect.top = 0;
			vin_priv->videocropcap.defrect.width = vin_priv->sensor_intf->u16CurImgWidth; 
			vin_priv->videocropcap.defrect.height =  vin_priv->sensor_intf->u16CurImgHeight;

			if(vin_priv->sensor_intf->u16CurImgWidth==1280){
				vin_priv->videocropcap.pixelaspect.numerator = 18;	/* Suppose current image size HD */
				vin_priv->videocropcap.pixelaspect.denominator = 32;	/* Zoomming step */
			}else {
				vin_priv->videocropcap.pixelaspect.numerator = 12;	/* Suppose current image size VGA/SVGA */
				vin_priv->videocropcap.pixelaspect.denominator = 16;	/* Zoomming step */
			}


			vin_priv->pDevVin->Open(72000, 24000);								
			
			vin_priv->pDevVin->SetBaseStartAddress(eVIDEOIN_PACKET, 0, vin_priv->videoIn_preview_buf[0].u32PhysAddr);
	
			vin_priv->pDevVin->EnableInt(eVIDEOIN_VINT);
			
			vin_priv->pDevVin->SetSensorPolarity(TRUE, 
										FALSE, 
										TRUE);
			vin_priv->pDevVin->SetDataFormatAndOrder(eVIDEOIN_IN_UYVY, 
											eVIDEOIN_IN_YUV422, 									
											eVIDEOIN_OUT_YUV422);											
			vin_priv->pDevVin->SetCropWinStartAddr(0,					//UINT16 u16VerticalStart, 		Y
											0);					//UINT16 u16HorizontalStart, 	X
			/* Sensor subsample resolution (640, 480)*/
			vin_priv->pDevVin->SetCropWinSize(vin_priv->sensor_intf->u16MaxImgHeight,		//UINT16 u16Height, 
									 vin_priv->sensor_intf->u16MaxImgWidth);		//UINT16 u16Width;									 
			
			vin_priv->pDevVin->PreviewPipeSize(vin_priv->videowin.height, vin_priv->videowin.width);
			
			vin_priv->pDevVin->GetStride(&u32PacStride, &u32PlaStride);
			vin_priv->pDevVin->SetStride(vin_priv->videowin.width, u32PlaStride);
			vin_priv->pDevVin->SetPipeEnable(FALSE,							// It means planar disable
										eVIDEOIN_PACKET);

			vin_priv->pDevVin->SetShadowRegister();
		break;

		case OV_9660:
			vin_priv->sensor_intf->u8SensorDevID = g_uOvDeviceID[0];
#ifndef CONFIG_SHARE_SENSOR
			SDBG("Init OV_9660 \n"); 
			schedule_mdelay(10);											
			res = OvRegConfig(0);
			if( res<0 ){
					printk("Sensor initial fail \n");
					return res;	
			}
			else
				printk("Sensor initial successful \n");			
			vin_priv->sensor_intf->u16CurImgWidth = vin_priv->sensor_intf->u16MaxImgWidth;
			vin_priv->sensor_intf->u16CurImgHeight = vin_priv->sensor_intf->u16MaxImgHeight;
#else
			{
				UINT16 u16Width, u16Height;
				/* From DEV1 get sensor output dimension */	
				GetVideo0SensorResoultion(&u16Width, &u16Height);
				vin_priv->sensor_intf->u16CurImgWidth = u16Width;
				vin_priv->sensor_intf->u16CurImgHeight = u16Height;
				/* 2014-01-10 */
				vin_priv->videocropcap.defrect.left = 0;
				vin_priv->videocropcap.defrect.top = 0;
				vin_priv->videocropcap.defrect.width = u16Width;
				vin_priv->videocropcap.defrect.height = u16Height;
			} 
#endif
			vin_priv->videocrop.c.left = 0;	/*X*/
			vin_priv->videocrop.c.top = 0;	/*Y*/	
			vin_priv->videocrop.c.width = vin_priv->sensor_intf->u16CurImgWidth;
			vin_priv->videocrop.c.height = vin_priv->sensor_intf->u16CurImgHeight;	

			vin_priv->videocropcap.bounds.left = 0; 
			vin_priv->videocropcap.bounds.top = 0;
			vin_priv->videocropcap.bounds.width = vin_priv->sensor_intf->u16CurImgWidth; 
			vin_priv->videocropcap.bounds.height = vin_priv->sensor_intf->u16CurImgHeight;

			vin_priv->videocropcap.defrect.left = 0; 
			vin_priv->videocropcap.defrect.top = 0;
			vin_priv->videocropcap.defrect.width = vin_priv->sensor_intf->u16CurImgWidth; 
			vin_priv->videocropcap.defrect.height =  vin_priv->sensor_intf->u16CurImgHeight;

			if(vin_priv->sensor_intf->u16CurImgWidth==1280){
				vin_priv->videocropcap.pixelaspect.numerator = 18;	/* Suppose current image size HD */
				vin_priv->videocropcap.pixelaspect.denominator = 32;	/* Zoomming step */
			}else {
				vin_priv->videocropcap.pixelaspect.numerator = 12;	/* Suppose current image size VGA/SVGA */
				vin_priv->videocropcap.pixelaspect.denominator = 16;	/* Zoomming step */
			}			

			vin_priv->pDevVin->Open(72000, 24000);					
			
			vin_priv->pDevVin->SetBaseStartAddress(eVIDEOIN_PACKET, 0, vin_priv->videoIn_preview_buf[0].u32PhysAddr);

			vin_priv->pDevVin->EnableInt(eVIDEOIN_VINT);
			
			vin_priv->pDevVin->SetSensorPolarity(TRUE, 
										FALSE, 
										TRUE);
			vin_priv->pDevVin->SetDataFormatAndOrder(eVIDEOIN_IN_UYVY,
											eVIDEOIN_IN_YUV422, 									
											eVIDEOIN_OUT_YUV422);											
			vin_priv->pDevVin->SetCropWinStartAddr(0,					//UINT16 u16VerticalStart, 	Y
										4);					//UINT16 u16HorizontalStart, 	X
			/* Sensor subsample resolution (640, 480)*/
			vin_priv->pDevVin->SetCropWinSize(vin_priv->sensor_intf->u16MaxImgHeight,		//UINT16 u16Height, 
									 vin_priv->sensor_intf->u16MaxImgWidth);		//UINT16 u16Width;									 				 
		
			vin_priv->pDevVin->PreviewPipeSize(vin_priv->videowin.height, vin_priv->videowin.width);
			vin_priv->pDevVin->GetStride(&u32PacStride, &u32PlaStride);
			vin_priv->pDevVin->SetStride(vin_priv->videowin.width, u32PlaStride);							
					
			vin_priv->pDevVin->SetPipeEnable(FALSE,							// It means planar disable
									eVIDEOIN_PACKET);		
			vin_priv->pDevVin->SetShadowRegister();				

		break;	

		default:
			return FALSE;
		break;
	}	// switch (u32Sensor)	
	return 0;	
}



static BOOL PoweronSensor(BOOL bIsPowerOn)
{
	if(bIsPowerOn == TRUE){		
		/* do after Open() */
	}else{
		if(bIsI2CAdd==TRUE){
			i2c_cleanup();
			printk("I2C removed\n");
			bIsI2CAdd = FALSE;	
		}
	}	
	return TRUE;	
}

static BOOL	
OVReadWriteBrightness(
	void *priv, 
	INT32 *pi32Value, 
	BOOL bIsRead
)
{
	videoin_priv_t *vin_priv = (videoin_priv_t *) priv;
	ENTER();
	if(vin_priv->sensor_intf->u8SensorDevID != g_uOvDeviceID[0])
		return FALSE;
	if(bIsRead)
		*pi32Value = (DrvVideoIn_I2cReadOV(vin_priv->sensor_intf->u8SensorDevID, 0x55)&0xff);
	else
		DrvVideoIn_I2cWriteOV(vin_priv->sensor_intf->u8SensorDevID, 0x55, *pi32Value);

	return TRUE;
}

static BOOL	
OVReadWriteContrast(
	void *priv, 
	INT32 *pi32Value, 
	BOOL bIsRead
)
{
	videoin_priv_t *vin_priv = (videoin_priv_t *) priv;
	ENTER();
	if(vin_priv->sensor_intf->u8SensorDevID != g_uOvDeviceID[0])
		return FALSE;
	if(bIsRead)
		*pi32Value = (DrvVideoIn_I2cReadOV(vin_priv->sensor_intf->u8SensorDevID, 0x56)&0xff);
	else
		DrvVideoIn_I2cWriteOV(vin_priv->sensor_intf->u8SensorDevID, 0x56, *pi32Value);

	return TRUE;
}

static BOOL	
OVReadWriteSharpness(
	void *priv, 
	INT32 *pi32Value, 
	BOOL bIsRead
)
{
	videoin_priv_t *vin_priv = (videoin_priv_t *) priv;
	ENTER();
	if(vin_priv->sensor_intf->u8SensorDevID != g_uOvDeviceID[0])
		return FALSE;
	if(bIsRead)
		*pi32Value = (DrvVideoIn_I2cReadOV(vin_priv->sensor_intf->u8SensorDevID, 0x3f)&0xff);
	else
		DrvVideoIn_I2cWriteOV(vin_priv->sensor_intf->u8SensorDevID, 0x3f, *pi32Value);

	return TRUE;
}


static BOOL	
OVReadWriteWhiteBalance(
	void *priv, 
	INT32 *pi32Value, 
	BOOL bIsRead
)
{
	videoin_priv_t *vin_priv = (videoin_priv_t *) priv;
	ENTER();
	if(vin_priv->sensor_intf->u8SensorDevID != g_uOvDeviceID[0])
		return FALSE;
	if(bIsRead)
		*pi32Value = (DrvVideoIn_I2cReadOV(vin_priv->sensor_intf->u8SensorDevID, 0x6f)&0xff);
	else
		DrvVideoIn_I2cWriteOV(vin_priv->sensor_intf->u8SensorDevID, 0x6f, *pi32Value);

	return TRUE;
}

static BOOL	
OVReadWriteNoiseReduction(
	void *priv, 
	INT32 *pi32Value, 
	BOOL bIsRead
)
{
	videoin_priv_t *vin_priv = (videoin_priv_t *) priv;
	ENTER();
	if(vin_priv->sensor_intf->u8SensorDevID != g_uOvDeviceID[0])
		return FALSE;
	if(bIsRead)
		*pi32Value = (DrvVideoIn_I2cReadOV(vin_priv->sensor_intf->u8SensorDevID, 0x4c)&0xff);
	else
		DrvVideoIn_I2cWriteOV(vin_priv->sensor_intf->u8SensorDevID, 0x4c, *pi32Value);

	return TRUE;
}

static BOOL	
OVReadWriteColorSaturation(
	void *priv, 
	INT32 *pi32Value, 
	BOOL bIsRead
)
{
	videoin_priv_t *vin_priv = (videoin_priv_t *) priv;
	ENTER();
	if(vin_priv->sensor_intf->u8SensorDevID != g_uOvDeviceID[0])
		return FALSE;
	if(bIsRead)
		*pi32Value = (DrvVideoIn_I2cReadOV(vin_priv->sensor_intf->u8SensorDevID, 0xc9)&0xff);
	else
		DrvVideoIn_I2cWriteOV(vin_priv->sensor_intf->u8SensorDevID, 0xc9, *pi32Value);

	return TRUE;
}
#if 0
/*================================================== V4L2 User Ctrl ================================================== */
struct v4l2_queryctrl
{
	__u32		     id;
	enum v4l2_ctrl_type  type;
	__u8		 name[32];	/* Whatever */
	__s32		 minimum;	/* Note signedness */
	__s32		 maximum;
	__s32		 step;
	__s32		 default_value;
	__u32               flags;
	__u32		reserved[2];
};

/*
 *	C O N T R O L S
 */
struct v4l2_control
{
	__u32		     id;
	__s32		     value;
};

#endif

static UINT16 u16SensRegAddr = 0; 
#define V4L2_CID_PRIVATE_I2C_SET_REG_ADDR     	(V4L2_CID_PRIVATE_BASE + 0)
#define V4L2_CID_PRIVATE_I2C_WRITE     			(V4L2_CID_PRIVATE_BASE + 1)
#define V4L2_CID_PRIVATE_I2C_READ     			(V4L2_CID_PRIVATE_BASE + 2)
#define V4L2_CID_PRIVATE_GET_SENSOR_CLOCK		(V4L2_CID_PRIVATE_BASE + 3)	
#define V4L2_CID_PRIVATE_SET_SENSOR_CLOCK		(V4L2_CID_PRIVATE_BASE + 4)	

#define V4L2_CID_PRIVATE_LASTP1     			(V4L2_CID_PRIVATE_BASE + 5)


static const struct v4l2_queryctrl no_ctrl = {
	.name  = "42",
	.flags = V4L2_CTRL_FLAG_DISABLED,
};
static const struct v4l2_queryctrl video_ctrls[] = {
	/* --- private --- */
	{
		.id            	= V4L2_CID_PRIVATE_I2C_SET_REG_ADDR,
		.name          	= "i2c_set_addr",
		.minimum       = 0,
		.maximum       = 255,
		.step          	= 1,
		.type          	= V4L2_CTRL_TYPE_INTEGER,
	},{
		.id            	= V4L2_CID_PRIVATE_I2C_WRITE,
		.name          	= "i2c_write",
		.minimum       = 0,
		.maximum       = 255,
		.step          	= 1,
		.type          	= V4L2_CTRL_TYPE_INTEGER,
	},{
		.id            = V4L2_CID_PRIVATE_I2C_READ,
		.name          = "i2c_read",
		.minimum       = 0,
		.maximum       = 255,
		.step          	= 1,
		.type          = V4L2_CTRL_TYPE_INTEGER,
	},
	{
		.id            = V4L2_CID_PRIVATE_GET_SENSOR_CLOCK,
		.name          = "get_sensor_clock",
		.minimum       = 12000000,
		.maximum       = 24000000,
		.step          	= 1000000,
		.type          = V4L2_CTRL_TYPE_INTEGER,
	},
	{
		.id            = V4L2_CID_PRIVATE_SET_SENSOR_CLOCK,
		.name          = "set_sensor_clock",
		.minimum       = 12000000,
		.maximum       = 24000000,
		.step          	= 1000000,
		.type          = V4L2_CTRL_TYPE_INTEGER,
	},

};
static const unsigned int CTRLS = ARRAY_SIZE(video_ctrls);

static const struct v4l2_queryctrl* ctrl_by_id(unsigned int id)
{
	unsigned int i;

	for (i = 0; i < CTRLS; i++)
		if (video_ctrls[i].id == id)
			return video_ctrls+i;
	return NULL;
}

static int SensorUserPrivateCtrl(struct file *file,
		 							unsigned int cmd, 
									unsigned long *arg)
{
	const struct v4l2_queryctrl *ctrl;
	struct v4l2_queryctrl *c = (struct v4l2_queryctrl *)arg;

	if ((c->id <  V4L2_CID_BASE ||
	     c->id >= V4L2_CID_LASTP1) &&
	    (c->id <  V4L2_CID_PRIVATE_BASE ||
	     c->id >= V4L2_CID_PRIVATE_LASTP1))
		return -EINVAL;
	ctrl = ctrl_by_id(c->id);
	*c = (NULL != ctrl) ? *ctrl : no_ctrl;
	return 0;
}

static BOOL	
SensorI2cWriteData(
	void *priv, 
	struct v4l2_control *c
)
{
	const struct v4l2_queryctrl* ctrl;
	videoin_priv_t *vin_priv = (videoin_priv_t *) priv;

	ctrl = ctrl_by_id(c->id);
	if (NULL == ctrl)
		return -EINVAL;
	SDBG("set_control name=%s val=%d\n",ctrl->name,c->value);
	switch (ctrl->type) {
	case V4L2_CTRL_TYPE_BOOLEAN:
	case V4L2_CTRL_TYPE_MENU:
	case V4L2_CTRL_TYPE_INTEGER:
		if (c->value < ctrl->minimum)
			c->value = ctrl->minimum;
		if (c->value > ctrl->maximum)
			c->value = ctrl->maximum;
		break;
	default:
		/* nothing */;
	};

	if(vin_priv->sensor_intf->u8SensorDevID != g_uOvDeviceID[0])
		return FALSE;
	DrvVideoIn_I2cWriteOV(vin_priv->sensor_intf->u8SensorDevID, u16SensRegAddr, c->value);
	return TRUE;
}
static BOOL	
SensorI2cReadData(
	void *priv, 
	struct v4l2_control *c
)
{
	videoin_priv_t *vin_priv = (videoin_priv_t *) priv;
	ENTER();	
	if(vin_priv->sensor_intf->u8SensorDevID != g_uOvDeviceID[0])
		return FALSE;
	c->value = (DrvVideoIn_I2cReadOV(vin_priv->sensor_intf->u8SensorDevID, u16SensRegAddr)&0xff);
	return TRUE;
}

static INT32 
SensorI2cSetRegAddr(
	void *priv, 
	struct v4l2_control *c
)
{
	u16SensRegAddr  = c->value;
	SDBG("Specified sensor addr = 0x%x\n", u16SensRegAddr);
	return 0;
}
static BOOL	
get_sensor_clock(
	void *priv, 
	struct v4l2_control *c
)
{
	unsigned int uDiv0, uDiv1, u32PllClock;
	videoin_priv_t *vin_priv = (videoin_priv_t *) priv;

	if(vin_priv->sensor_intf->u8SensorDevID != g_uOvDeviceID[0])
		return FALSE;

	/* Platform dependence */
#ifdef CONFIG_W55FA92_VIDEOIN_DEV1
//	u32PllClock = w55fa92_upll_clock*1000;
//	if(u32PllClock==0)
//		u32PllClock = w55fa92_apll_clock*1000;
//	uDiv0 = ((inp32(REG_CLKDIV0)&SENSOR0_N0)>> 19)+1;
//	uDiv1 = ((inp32(REG_CLKDIV0)&SENSOR0_N1)>> 24)+1;
#endif
#ifdef CONFIG_W55FA92_VIDEOIN_DEV2
	if( ((w55fa92_apll_clock%24000)==0) && ((w55fa92_apll_clock%20000)==0) && ((w55fa92_apll_clock%16000)==0) )
		u32PllClock = w55fa92_apll_clock*1000;
	else
		u32PllClock = w55fa92_upll_clock*1000;
	uDiv0 = ((inp32(REG_CLKDIV5)&SENSOR1_N0)>> 13)+1;
	uDiv1 = ((inp32(REG_CLKDIV5)&SENSOR1_N1)>> 18)+1;
#endif

	c->value = u32PllClock/(uDiv0*uDiv1);
	return TRUE;
}

static BOOL	
set_sensor_clock(
	void *priv, 
	struct v4l2_control *c
)
{
	unsigned int uDiv0, uDiv1, uSenDiv, u32PllClock;
	const struct v4l2_queryctrl* ctrl;
	videoin_priv_t *vin_priv = (videoin_priv_t *) priv;

	if(vin_priv->sensor_intf->u8SensorDevID != g_uOvDeviceID[0])
		return FALSE;

	ctrl = ctrl_by_id(c->id);
	if (NULL == ctrl)
		return -EINVAL;
	DBG_PRINTF("set_control name=%s val=%d\n",ctrl->name,c->value);
	switch (ctrl->type) {
	case V4L2_CTRL_TYPE_BOOLEAN:
	case V4L2_CTRL_TYPE_MENU:
	case V4L2_CTRL_TYPE_INTEGER: 
		if (c->value < ctrl->minimum)
			c->value = ctrl->minimum;
		if (c->value > ctrl->maximum)
			c->value = ctrl->maximum;
		break;
	default:
		/* nothing */;
	};

	/* Platform dependence */
	if( ((w55fa92_apll_clock%24000)==0) && ((w55fa92_apll_clock%20000)==0) && ((w55fa92_apll_clock%16000)==0) )
		u32PllClock = w55fa92_apll_clock*1000;
	else
		u32PllClock = w55fa92_upll_clock*1000;

	uSenDiv = u32PllClock/c->value;
	if(u32PllClock%c->value != 0)
		uSenDiv = uSenDiv+1;

	for(uDiv1=1; uDiv1<=16; uDiv1 = uDiv1+1)	
	{//uDiv0 should be start from 1
		for(uDiv0=1; uDiv0<=8; uDiv0 = uDiv0+1)	
		{
			if(uSenDiv==uDiv0*uDiv1)
				break;
		}
		if( uDiv0 >= 9 ) continue;
		if(uSenDiv==uDiv0*uDiv1)
				break;
	}	
	uDiv0 = uDiv0-1;
	uDiv1 = uDiv1-1;
#ifdef CONFIG_W55FA92_VIDEOIN_DEV1
//	outp32(REG_CLKDIV0, (inp32(REG_CLKDIV0) & ~(SENSOR0_N1 | SENSOR0_N0)) | ((uDiv0<<19) | (uDiv1<<24)) );
#endif
#ifdef CONFIG_W55FA92_VIDEOIN_DEV2
	outp32(REG_CLKDIV5, (inp32(REG_CLKDIV5) & ~(SENSOR1_N1 | SENSOR1_N0)) | ((uDiv0<<13) | (uDiv1<<18)) );
#endif
	return TRUE;
}
/* ------------------------------------------------------------------ */
static INT32 SensorI2cReadCtrl(void *priv,
				 				struct v4l2_control *c)
{
	const struct v4l2_queryctrl* ctrl;

	ctrl = ctrl_by_id(c->id);
	SDBG("Get_control name=%s\n",ctrl->name);
	if (NULL == ctrl)
		return -EINVAL;
	switch (c->id) {
/*
	case V4L2_CID_PRIVATE_I2C_WRITE:
		break;
*/
	case V4L2_CID_PRIVATE_I2C_READ:
		if( SensorI2cReadData(priv, c) == FALSE)
		{
			printk("i2c read fail\n");	
			return -EINVAL;	/* I2c read fail */
		}	
		break;
	case V4L2_CID_PRIVATE_I2C_SET_REG_ADDR:
		c->value = u16SensRegAddr;
		break;
	case V4L2_CID_PRIVATE_GET_SENSOR_CLOCK:
		if( get_sensor_clock(priv, c) == FALSE)
		{
			printk("get sensor clock fail\n");	
			return -EINVAL;
		}	
		break;		
	default:
		return -EINVAL;
	}
	return 0;
}
 

static int SensorI2cWriteCtrl(void *priv, 
					struct v4l2_control *c)
{
	const struct v4l2_queryctrl* ctrl;
	//unsigned long flags;
	//int restart_overlay = 0;

	ctrl = ctrl_by_id(c->id);
	if (NULL == ctrl)
		return -EINVAL;
	SDBG("set_control name=%s val=%d\n",ctrl->name,c->value);
	switch (ctrl->type) {
	case V4L2_CTRL_TYPE_BOOLEAN:
	case V4L2_CTRL_TYPE_MENU:
	case V4L2_CTRL_TYPE_INTEGER:
		if (c->value < ctrl->minimum)
			c->value = ctrl->minimum;
		if (c->value > ctrl->maximum)
			c->value = ctrl->maximum;
		break;
	default:
		/* nothing */;
	};
	switch (c->id) {
	case V4L2_CID_PRIVATE_I2C_WRITE:
		if(SensorI2cWriteData(priv, c)==FALSE)
		{
			printk("i2c write faIl\n");	
			return -EINVAL;	/* I2c write fail */
		}
		break;	
/*
	case V4L2_CID_PRIVATE_I2C_READ:
		break;	
*/
	case V4L2_CID_PRIVATE_I2C_SET_REG_ADDR:
		u16SensRegAddr  = c->value;
		printk("Specified sensor addr = 0x%x\n", u16SensRegAddr);
		break;
	case V4L2_CID_PRIVATE_SET_SENSOR_CLOCK:
		if( set_sensor_clock(priv, c) == FALSE)
		{
			printk("get sensor clock fail\n");	
			return -EINVAL;
		}	
		break;		
	default:
		return -EINVAL;
	}
	return 0;
}


#if 0
/*================================================== V4L2 User Ctrl ================================================== */
#endif 

static NVT_SENSOR_T nvt_sensor_ov = {
	sensor_init:					InitSensor,
	sensor_poweron:					PoweronSensor,

#ifdef CONFIG_SENSOR_POWERDOWN
	sensor_suspend:					Snr2_PowerDown,
#else
	sensor_suspend:					Snr2_PowerDown,
#endif
#ifdef CONFIG_SENSOR_RESET
	sensor_reset:					Snr2_Reset,
#else	
	sensor_reset:					Snr2_Reset,
#endif
	read_write_brightness:			OVReadWriteBrightness,
	read_write_contrast:			OVReadWriteContrast,
	read_write_sharpness:			OVReadWriteSharpness,
	read_write_white_balance:		OVReadWriteWhiteBalance,
	read_write_noise_reduction:		OVReadWriteNoiseReduction,
	read_write_color_saturation:	OVReadWriteColorSaturation,

	query_private_user_ctrl:		SensorUserPrivateCtrl,    /* OK */
	sensor_i2c_setRegAddr:			SensorI2cSetRegAddr, 	/* OK */
	sensor_set_ctrl:				SensorI2cWriteCtrl,
	sensor_get_ctrl:				SensorI2cReadCtrl,

	change_image_resolution: 		NULL,
	set_flicker_freq:				NULL,
	low_lux_detect:					NULL,
	control_IR_led:					NULL,
#if 0
	u16MaxImgHeight:				480,		 
	u16MaxImgWidth: 				640,
#else
	u16MaxImgHeight:				CONFIG_VIN_DEV2_ENCODE_HEIGHT,		 
	u16MaxImgWidth: 				CONFIG_VIN_DEV2_ENCODE_WIDTH,
#endif
};
#ifdef CONFIG_W55FA92_VIDEOIN_DEV2
INT32 register_vin_port2_Sensor(NVT_SENSOR_T **sensor_intf)
{
	*sensor_intf = (NVT_SENSOR_T*)(&nvt_sensor_ov);
	return 0;
}
#endif



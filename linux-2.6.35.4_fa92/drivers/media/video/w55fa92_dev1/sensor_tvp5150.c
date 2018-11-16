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
#include "TVP5150/sensor_tvp5150_reg.h"

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
//IMPORT_SYMBOL(w55fa95_FB_BG_PHY_ADDR);
//extern unsigned int w55fa95_FB_BG_PHY_ADDR;
//#define outp32(addr, value)		writel(value, addr)
//#define inp32(addr)				readl(addr)
 

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
//extern VINDEV_T* pDevVin;
#define __STANDARD_I2C__

//extern videoIn_buf_t videoIn_preview_buf[];


struct OV_RegValue{
	__u8	uRegAddr;
	__u8	uValue;
};

struct OV_RegTable{
	struct OV_RegValue *sRegTable;
	__u16 uTableSize;
};
#define REG_VALUE_INIT		0
#define REG_VALUE_NTSC		1
#define REG_VALUE_PAL		2

#ifdef CONFIG_NTSC_SYSTEM_DEV1		//NTSC & PAL ??
#define CROP_START_X		CONFIG_NTSC_CROP_START_X	//76 for one field. 6 for two field
#define CROP_START_Y		CONFIG_NTSC_CROP_START_Y	//12 for one field. 35 for two field
#endif
#ifdef CONFIG_PAL_SYSTEM_DEV1		//NTSC & PAL ??		
#define CROP_START_X		CONFIG_PAL_CROP_START_X		
#define CROP_START_Y		CONFIG_PAL_CROP_START_Y		
#endif




static struct OV_RegValue g_sTVP5150_Init[] = 
{
	{ /* 0x00 */
		TVP5150_VD_IN_SRC_SEL_1,0x00
	},
	{ /* 0x01 */
		TVP5150_ANAL_CHL_CTL,0x15
	},
	{ /* 0x02 */
		TVP5150_OP_MODE_CTL,0x00
	},
	{ /* 0x03 */
		TVP5150_MISC_CTL,0x0D
	},
	{ /* 0x06 */
		TVP5150_COLOR_KIL_THSH_CTL,0x10
	},
	{ /* 0x07 */
		TVP5150_LUMA_PROC_CTL_1,0x20	// Luma signal delay ::  0 pixel clocks delay 0x20, 0x23, 0x28
	},
	{ /* 0x08 */
		TVP5150_LUMA_PROC_CTL_2,0x4C	//Luminance filter select: 0x48 comb filter disabled, chroma trap filter enable !!!!! 
	},
	{ /* 0x09 */
		TVP5150_BRIGHT_CTL,0x80
	},
	{ /* 0x0a */
		TVP5150_SATURATION_CTL,0xA0
	},
	{ /* 0x0b */
		TVP5150_HUE_CTL,0x00
	},
	{ /* 0x0c */
		TVP5150_CONTRAST_CTL,0x84
	},
	{ /* 0x0d */
		TVP5150_DATA_RATE_SEL,0x47
	},
	{ /* 0x0e */
		TVP5150_LUMA_PROC_CTL_3,0x00	//Luminance filter stop band bandwidth:: No notch
	},
	{ /* 0x0f */
		TVP5150_CONF_SHARED_PIN,0x08
	},
	{ /* 0x11 */
		TVP5150_ACT_VD_CROP_ST_MSB,0x00
	},
	{ /* 0x12 */
		TVP5150_ACT_VD_CROP_ST_LSB,0x00
	},
	{ /* 0x13 */
		TVP5150_ACT_VD_CROP_STP_MSB,0x00
	},
	{ /* 0x14 */
		TVP5150_ACT_VD_CROP_STP_LSB,0x00
	},
	{ /* 0x15 */
		TVP5150_GENLOCK,0x01
	},
	{ /* 0x16 */
		TVP5150_HORIZ_SYNC_START,0x80
	},
	{ /* 0x18 */
		TVP5150_VERT_BLANKING_START,0x00
	},
	{ /* 0x19 */
		TVP5150_VERT_BLANKING_STOP,0x00
	},
	{ /* 0x1a */
		TVP5150_CHROMA_PROC_CTL_1,0x0c
	},
	{ /* 0x1b */
		TVP5150_CHROMA_PROC_CTL_2,0x15	//Chrominance filter select: No notch = 0x10
	},
	{ /* 0x1c */
		TVP5150_INT_RESET_REG_B,0x00
	},
	{ /* 0x1d */
		TVP5150_INT_ENABLE_REG_B,0x00
	},
	{ /* 0x1e */
		TVP5150_INTT_CONFIG_REG_B,0x00
	},
	{ /* 0x2e */
		TVP5150_MACROVISION_ON_CTR,0x0f
	},
	{ /* 0x2f */
		TVP5150_MACROVISION_OFF_CTR,0x01
	},
	{ /* 0xbb */
		TVP5150_TELETEXT_FIL_ENA,0x00
	},
	{ /* 0xc0 */
		TVP5150_INT_STATUS_REG_A,0x00
	},
	{ /* 0xc1 */
		TVP5150_INT_ENABLE_REG_A,0x00
	},
	{ /* 0xc2 */
		TVP5150_INT_CONF,0x04
	},
	{ /* 0xc8 */
		TVP5150_FIFO_INT_THRESHOLD,0x80
	},
	{ /* 0xc9 */
		TVP5150_FIFO_RESET,0x00
	},
	{ /* 0xca */
		TVP5150_LINE_NUMBER_INT,0x00
	},
	{ /* 0xcb */
		TVP5150_PIX_ALIGN_REG_LOW,0x4e
	},
	{ /* 0xcc */
		TVP5150_PIX_ALIGN_REG_HIGH,0x00
	},
	{ /* 0xcd */
		TVP5150_FIFO_OUT_CTRL,0x01
	},
	{ /* 0xcf */
		TVP5150_FULL_FIELD_ENA,0x00
	},
	{ /* 0xd0 */
		TVP5150_LINE_MODE_INI,0x00
	},
	{ /* 0xfc */
		TVP5150_FULL_FIELD_MODE_REG,0x7f
	},
};


static struct OV_RegTable g_OV_InitTable[] =
{
	{g_sTVP5150_Init,_REG_TABLE_SIZE(g_sTVP5150_Init)},	
};

static __u8 g_uOvDeviceID[]= 
{
	0xBA,			/* If pin SIAD high, 8 bits slave address = 0xBA */				
					/* If pin SIAD low, 8 bits slave address = 0xB8 */	
};

static struct i2c_client *save_client;
static struct i2c_client i2c_client; 
static int sensor_probe(struct i2c_client *client,
			const struct i2c_device_id *did)
{
	int ret = 0;
	
	ENTER();
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
	{ "tvp5150", 0 },
};
MODULE_DEVICE_TABLE(i2c, sensor_id);

static struct i2c_driver sensor_i2c_driver = {

	.driver = {
		.name = "tvp5150",
	},

	.probe    = sensor_probe,
	.remove   = sensor_remove,
	.id_table = sensor_id,
};
 
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
	LEAVE();
	return TRUE;
}


static UINT8 I2C_Read_8bitSlaveAddr_8bitReg_8bitData(UINT8 uAddr, UINT8 uRegAddr)
{

	UINT8 u8Data;

	ENTER();

	// 2-Phase(ID address, register address) write transmission
	DrvI2C_SendStart();
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
	
	msg.flags=!I2C_M_RD;
	msg.addr=save_client->addr;
	msg.len=2;
	msg.buf=buf;		

	buf[0]=(u8)uRegAddr;
	buf[1]=uData;

	ret=i2c_transfer(save_client->adapter,&msg,1);
	if (uRegAddr==0x12 && (uData&0x80)!=0)
	{
		mdelay(20);			
	}
	if(ret<0)
		return FALSE;
	else
		return TRUE;
#else
	ret = I2C_Write_8bitSlaveAddr_8bitReg_8bitData(uAddr, uRegAddr, uData);
	return ret;
#endif
}
 
static UINT8 DrvVideoIn_I2cReadOV(__u8 uAddr, __u8 uRegAddr)
{
#ifdef __STANDARD_I2C__
	#if 1
		struct i2c_msg msgs;
		int ret=-1;
		u8 buf[3];
		
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
		UINT8 val;
		//int ret;
		ENTER();
		i2c_smbus_write_byte(save_client, uRegAddr);
		val = i2c_smbus_read_byte(save_client);
		SDBG("read value = 0x%x\n", val);	
		return val;		
	#endif
#else
		ENTER();
		return I2C_Read_8bitSlaveAddr_8bitReg_8bitData(uAddr, uRegAddr);
#endif	
}

static void schedule_mdelay(UINT32 u32Delay_ms)
{
	unsigned long j=0;	
	j = jiffies + u32Delay_ms*HZ/1000; 	/* 2Xms~30ms */			
	while( time_before(jiffies,j) )
		schedule();	
}

#if 0
static void dump_reg(__u8 c)
{
	printk("tvp5150: Video input source selection #1 = 0x%02x\n",
					DrvVideoIn_I2cReadTVP5150(c, TVP5150_VD_IN_SRC_SEL_1));
	printk("tvp5150: Analog channel controls = 0x%02x\n",
					DrvVideoIn_I2cReadTVP5150(c, TVP5150_ANAL_CHL_CTL));
	printk("tvp5150: Operation mode controls = 0x%02x\n",
					DrvVideoIn_I2cReadTVP5150(c, TVP5150_OP_MODE_CTL));
	printk("tvp5150: Miscellaneous controls = 0x%02x\n",
					DrvVideoIn_I2cReadTVP5150(c, TVP5150_MISC_CTL));
	printk("tvp5150: Autoswitch mask= 0x%02x\n",
					DrvVideoIn_I2cReadTVP5150(c, TVP5150_AUTOSW_MSK));
	printk("tvp5150: Color killer threshold control = 0x%02x\n",
					DrvVideoIn_I2cReadTVP5150(c, TVP5150_COLOR_KIL_THSH_CTL));
	printk("tvp5150: Luminance processing controls #1 #2 and #3 = %02x %02x %02x\n",
					DrvVideoIn_I2cReadTVP5150(c, TVP5150_LUMA_PROC_CTL_1),
					DrvVideoIn_I2cReadTVP5150(c, TVP5150_LUMA_PROC_CTL_2),
					DrvVideoIn_I2cReadTVP5150(c, TVP5150_LUMA_PROC_CTL_3));
	printk("tvp5150: Brightness control = 0x%02x\n",
					DrvVideoIn_I2cReadTVP5150(c, TVP5150_BRIGHT_CTL));
	printk("tvp5150: Color saturation control = 0x%02x\n",
					DrvVideoIn_I2cReadTVP5150(c, TVP5150_SATURATION_CTL));
	printk("tvp5150: Hue control = 0x%02x\n",
					DrvVideoIn_I2cReadTVP5150(c, TVP5150_HUE_CTL));
	printk("tvp5150: Contrast control = 0x%02x\n",
					DrvVideoIn_I2cReadTVP5150(c, TVP5150_CONTRAST_CTL));
	printk("tvp5150: Outputs and data rates select = 0x%02x\n",
					DrvVideoIn_I2cReadTVP5150(c, TVP5150_DATA_RATE_SEL));
	printk("tvp5150: Configuration shared pins = 0x%02x\n",
					DrvVideoIn_I2cReadTVP5150(c, TVP5150_CONF_SHARED_PIN));
	printk("tvp5150: Active video cropping start = 0x%02x%02x\n",
					DrvVideoIn_I2cReadTVP5150(c, TVP5150_ACT_VD_CROP_ST_MSB),
					DrvVideoIn_I2cReadTVP5150(c, TVP5150_ACT_VD_CROP_ST_LSB));
	printk("tvp5150: Active video cropping stop  = 0x%02x%02x\n",
					DrvVideoIn_I2cReadTVP5150(c, TVP5150_ACT_VD_CROP_STP_MSB),
					DrvVideoIn_I2cReadTVP5150(c, TVP5150_ACT_VD_CROP_STP_LSB));
	printk("tvp5150: Genlock/RTC = 0x%02x\n",
					DrvVideoIn_I2cReadTVP5150(c, TVP5150_GENLOCK));
	printk("tvp5150: Horizontal sync start = 0x%02x\n",
					DrvVideoIn_I2cReadTVP5150(c, TVP5150_HORIZ_SYNC_START));
	printk("tvp5150: Vertical blanking start = 0x%02x\n",
					DrvVideoIn_I2cReadTVP5150(c, TVP5150_VERT_BLANKING_START));
	printk("tvp5150: Vertical blanking stop = 0x%02x\n",
					DrvVideoIn_I2cReadTVP5150(c, TVP5150_VERT_BLANKING_STOP));
	printk("tvp5150: Chrominance processing control #1 and #2 = %02x %02x\n",
					DrvVideoIn_I2cReadTVP5150(c, TVP5150_CHROMA_PROC_CTL_1),
					DrvVideoIn_I2cReadTVP5150(c, TVP5150_CHROMA_PROC_CTL_2));
	printk("tvp5150: Interrupt reset register B = 0x%02x\n",
					DrvVideoIn_I2cReadTVP5150(c, TVP5150_INT_RESET_REG_B));
	printk("tvp5150: Interrupt enable register B = 0x%02x\n",
					DrvVideoIn_I2cReadTVP5150(c, TVP5150_INT_ENABLE_REG_B));
	printk("tvp5150: Interrupt configuration register B = 0x%02x\n",
					DrvVideoIn_I2cReadTVP5150(c, TVP5150_INTT_CONFIG_REG_B));
	printk("tvp5150: Video standard = 0x%02x\n",
					DrvVideoIn_I2cReadTVP5150(c, TVP5150_VIDEO_STD));
	printk("tvp5150: Chroma gain factor: Cb=0x%02x Cr=0x%02x\n",
					DrvVideoIn_I2cReadTVP5150(c, TVP5150_CB_GAIN_FACT),
					DrvVideoIn_I2cReadTVP5150(c, TVP5150_CR_GAIN_FACTOR));
	printk("tvp5150: Macrovision on counter = 0x%02x\n",
					DrvVideoIn_I2cReadTVP5150(c, TVP5150_MACROVISION_ON_CTR));
	printk("tvp5150: Macrovision off counter = 0x%02x\n",
					DrvVideoIn_I2cReadTVP5150(c, TVP5150_MACROVISION_OFF_CTR));
	printk("tvp5150: ITU-R BT.656.%d timing(TVP5150AM1 only)\n",
					(DrvVideoIn_I2cReadTVP5150(c, TVP5150_REV_SELECT)&1)?3:4);
	printk("tvp5150: Device ID = %02x%02x\n",
					DrvVideoIn_I2cReadTVP5150(c, TVP5150_MSB_DEV_ID),
					DrvVideoIn_I2cReadTVP5150(c, TVP5150_LSB_DEV_ID));
	printk("tvp5150: ROM version = (hex) %02x.%02x\n",
					DrvVideoIn_I2cReadTVP5150(c, TVP5150_ROM_MAJOR_VER),
					DrvVideoIn_I2cReadTVP5150(c, TVP5150_ROM_MINOR_VER));
	printk("tvp5150: Vertical line count = 0x%02x%02x\n",
					DrvVideoIn_I2cReadTVP5150(c, TVP5150_VERT_LN_COUNT_MSB),
					DrvVideoIn_I2cReadTVP5150(c, TVP5150_VERT_LN_COUNT_LSB));
	printk("tvp5150: Interrupt status register B = 0x%02x\n",
					DrvVideoIn_I2cReadTVP5150(c, TVP5150_INT_STATUS_REG_B));
	printk("tvp5150: Interrupt active register B = 0x%02x\n",
					DrvVideoIn_I2cReadTVP5150(c, TVP5150_INT_ACTIVE_REG_B));
	printk("tvp5150: Status regs #1 to #5 = %02x %02x %02x %02x %02x\n",
					DrvVideoIn_I2cReadTVP5150(c, TVP5150_STATUS_REG_1),
					DrvVideoIn_I2cReadTVP5150(c, TVP5150_STATUS_REG_2),
					DrvVideoIn_I2cReadTVP5150(c, TVP5150_STATUS_REG_3),
					DrvVideoIn_I2cReadTVP5150(c, TVP5150_STATUS_REG_4),
					DrvVideoIn_I2cReadTVP5150(c, TVP5150_STATUS_REG_5));

	printk("tvp5150: Teletext filter enable = 0x%02x\n",
					DrvVideoIn_I2cReadTVP5150(c, TVP5150_TELETEXT_FIL_ENA));
	printk("tvp5150: Interrupt status register A = 0x%02x\n",
					DrvVideoIn_I2cReadTVP5150(c, TVP5150_INT_STATUS_REG_A));
	printk("tvp5150: Interrupt enable register A = 0x%02x\n",
					DrvVideoIn_I2cReadTVP5150(c, TVP5150_INT_ENABLE_REG_A));
	printk("tvp5150: Interrupt configuration = 0x%02x\n",
					DrvVideoIn_I2cReadTVP5150(c, TVP5150_INT_CONF));
	printk("tvp5150: VDP status register = 0x%02x\n",
					DrvVideoIn_I2cReadTVP5150(c, TVP5150_VDP_STATUS_REG));
	printk("tvp5150: FIFO word count = 0x%02x\n",
					DrvVideoIn_I2cReadTVP5150(c, TVP5150_FIFO_WORD_COUNT));
	printk("tvp5150: FIFO interrupt threshold = 0x%02x\n",
					DrvVideoIn_I2cReadTVP5150(c, TVP5150_FIFO_INT_THRESHOLD));
	printk("tvp5150: FIFO reset = 0x%02x\n",
					DrvVideoIn_I2cReadTVP5150(c, TVP5150_FIFO_RESET));
	printk("tvp5150: Line number interrupt = 0x%02x\n",
					DrvVideoIn_I2cReadTVP5150(c, TVP5150_LINE_NUMBER_INT));
	printk("tvp5150: Pixel alignment register = 0x%02x%02x\n",
					DrvVideoIn_I2cReadTVP5150(c, TVP5150_PIX_ALIGN_REG_HIGH),
					DrvVideoIn_I2cReadTVP5150(c, TVP5150_PIX_ALIGN_REG_LOW));
	printk("tvp5150: FIFO output control = 0x%02x\n",
					DrvVideoIn_I2cReadTVP5150(c, TVP5150_FIFO_OUT_CTRL));
	printk("tvp5150: Full field enable = 0x%02x\n",
					DrvVideoIn_I2cReadTVP5150(c, TVP5150_FULL_FIELD_ENA));
	printk("tvp5150: Full field mode register = 0x%02x\n",
					DrvVideoIn_I2cReadTVP5150(c, TVP5150_FULL_FIELD_MODE_REG));
}
#endif

static void TVP5150SetVideoformat(int std)
{
	__u8  uDeviceID;
	__u8  fmt = 0;
	
	uDeviceID  = g_uOvDeviceID[0];
	
	/* First tests should be against specific std */
	if (std == TVP_STD_ALL) {
		fmt=0;	/* Autodetect mode */
	} else if (std & TVP_STD_NTSC_443) {
		fmt=0xa;
	} else if (std & TVP_STD_PAL_M) {
		fmt=0x6;
	} else if (std & (TVP_STD_PAL_N| TVP_STD_PAL_Nc)) {
		fmt=0x8;
	} else {
		/* Then, test against generic ones */
		if (std & TVP_STD_NTSC) {
			fmt=0x2;
		} else if (std & TVP_STD_PAL) {
			fmt=0x4;
		}
	}
	printk("TVP5150 Video format : %02x. \r\n",fmt);
	DrvVideoIn_I2cWriteOV(uDeviceID, TVP5150_VIDEO_STD, fmt);
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
	printk("Standard I2C.\n");	
#else	
	printk("Non Standard I2C.\n");
	outp32(REG_GPBFUN1, inp32(REG_GPBFUN1)& ~(MF_GPB14|MF_GPB13));
	DrvI2C_Open(eDRVGPIO_GPIOB, 					
				eDRVGPIO_PIN13, 
				eDRVGPIO_GPIOB,
				eDRVGPIO_PIN14, 
				(PFN_DRVI2C_TIMEDELY)I2C_Delay);
	
#endif
	//SDBG("nIndex = %d\n", nIndex);	
	if ( nIndex >= (sizeof(g_uOvDeviceID)/sizeof(__u8)) )
		return -EBUSY;	
	
	uTableSize = g_OV_InitTable[REG_VALUE_INIT].uTableSize;
	psRegValue = g_OV_InitTable[REG_VALUE_INIT].sRegTable;
	uDeviceID = g_uOvDeviceID[0];
	if ( psRegValue == 0 )
		return -EBUSY;	
	for(i=0;i<uTableSize; i++, psRegValue++)
	{
		udelay(10);	
		res = DrvVideoIn_I2cWriteOV(uDeviceID, (psRegValue->uRegAddr), (psRegValue->uValue));
		if(res==FALSE)
			return -EBUSY;
	}

#if defined CONFIG_PAL_SYSTEM_DEV1
	TVP5150SetVideoformat(TVP_STD_PAL);	
#elif defined (CONFIG_NTSC_SYSTEM_DEV1)
	TVP5150SetVideoformat(TVP_STD_NTSC);
#else		// default PAL
	TVP5150SetVideoformat(TVP_STD_PAL);	
#endif

	//dump_reg(uDeviceID);

	if(res>=0)
		printk("sensor initial done\n");
	else
		printk("sensor initial fail\n");	
	return res;
}
static BOOL bIsI2CAdd = FALSE; 
static int i2c_init(void)
{
	int ret;
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
	LEAVE();
#endif
}

#ifdef CONFIG_SHARE_SENSOR	
static UINT16 u16SensorWidth, u16SensorHeight;
static void SensorOutputResolution(UINT16 u16ImageWidth, 
								UINT16 u16ImageHeight)
{
	u16SensorWidth = u16ImageWidth;
	u16SensorHeight = u16ImageHeight;
}
void GetVideo0SensorResoultion(UINT16* pSenWidth, UINT16* pSenHeight)
{
	*pSenWidth = u16SensorWidth;
	*pSenHeight = u16SensorHeight;
}
#endif

#ifdef CONFIG_W55FA92_VIDEOIN_DEV2
extern void video_port2_pin_init(
	BOOL bIsEnableSnrClock,
	E_VIDEOIN_SNR_SRC eSnrSrc,	
	UINT32 u32SensorFreqKHz,						//KHz unit
	E_VIDEOIN_DEV_TYPE eDevType
	);
#endif 

static __s32 InitSensor(__u32 u32Sensor, void *priv)
{
	__u32 u32PacStride, u32PlaStride;
	__s32 res = 0; 
	videoin_priv_t *vin_priv = (videoin_priv_t *) priv;

	printk("Init TVP5150 \n"); 		
	vin_priv->pDevVin->Init(TRUE, eVIDEOIN_SNR_UPLL, 24000, eVIDEOIN_SNR_CCIR601);	
	
#ifdef CONFIG_W55FA92_VIDEOIN_DEV2
	{/* If exist the second video capture device ==> Enable 2nd port's pin function  */
		if( (inp32(REG_AHBCLK2) & VIN1_CKE) == 0)/* VideoIn port 2 close */{
	#ifdef CONFIG_VIDEO_PORT2_0_DEV2				
			video_port2_pin_init(TRUE, eVIDEOIN_SNR_UPLL, 24000, eVIDEOIN_2ND_SNR_CCIR601);	 
			printk("Init port 2 for reset and power down\n");
	#endif																
	#ifdef CONFIG_VIDEO_PORT2_1_DEV2	
			video_port2_pin_init(TRUE, eVIDEOIN_SNR_UPLL, 24000, eVIDEOIN_3RD_SNR_CCIR601);
			printk("Init port 2_1 for reset and power down\n");
	#endif		
			Snr1_PowerDown(FALSE);
			Snr1_Reset(TRUE);					   /* Reset the first and second sensor */
		}	
	}
#else	
	Snr1_PowerDown(FALSE);
	Snr1_Reset(TRUE);	
#endif

	schedule_mdelay(10);
	i2c_init();
	schedule_mdelay(10);

	vin_priv->pDevVin->Open(64000, 24000);
	
	SDBG("Init TVP5150 \n"); 								
	res = OvRegConfig(0);
	if( res<0 ){
		printk("Sensor initial fail \n");
		return res;	
	}
	else
		printk("Sensor initial successful \n");
	printk("After REG_CLKDIV0 = 0x%x\n", inp32(REG_CLKDIV0));
	vin_priv->sensor_intf->u8SensorDevID = g_uOvDeviceID[0];//g_uOvDeviceID[pDevVin->ov7725];
	vin_priv->sensor_intf->u16CurImgHeight = vin_priv->sensor_intf->u16MaxImgHeight;
	vin_priv->sensor_intf->u16CurImgWidth = vin_priv->sensor_intf->u16MaxImgWidth;
#ifdef CONFIG_SHARE_SENSOR											
	SensorOutputResolution(vin_priv->sensor_intf->u16CurImgWidth, vin_priv->sensor_intf->u16CurImgHeight);				
#endif
	vin_priv->videocrop.c.left = CROP_START_X;	/*X*/
	vin_priv->videocrop.c.top = CROP_START_Y;	/*Y*/	
	vin_priv->videocrop.c.width = vin_priv->sensor_intf->u16CurImgWidth;
	vin_priv->videocrop.c.height = vin_priv->sensor_intf->u16CurImgHeight;	

	vin_priv->videocropcap.bounds.left = CROP_START_X; 
	vin_priv->videocropcap.bounds.top = CROP_START_Y;
	vin_priv->videocropcap.bounds.width = vin_priv->sensor_intf->u16CurImgWidth; 
	vin_priv->videocropcap.bounds.height = vin_priv->sensor_intf->u16CurImgHeight;

	vin_priv->videocropcap.defrect.left = CROP_START_X; 
	vin_priv->videocropcap.defrect.top = CROP_START_Y;
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
	
	
#ifdef CONFIG_ONE_FIELD_DEV1
	//vin_priv->pDevVin->SetInputType(2, eVIDEOIN_TYPE_CCIR601, FALSE);
	//vin_priv->pDevVin->SetStandardCCIR656(FALSE);		/* non-standard CCIR656 */
	vin_priv->pDevVin->SetSensorPolarity(TRUE, 
								TRUE, 
								FALSE);		
	vin_priv->pDevVin->SetFrameRateScaleFactor(1, 2);
#endif
#ifdef CONFIG_TWO_FIELDS_DEV1
	vin_priv->pDevVin->SetSensorPolarity(FALSE, 
								FALSE, 
								TRUE);	
	vin_priv->pDevVin->SetInputType(3, eVIDEOIN_TYPE_CCIR656,FALSE);
	vin_priv->pDevVin->SetStandardCCIR656(TRUE);		/* standard CCIR656 mode*/
#endif

	vin_priv->pDevVin->SetDataFormatAndOrder(eVIDEOIN_IN_VYUY, 
									eVIDEOIN_IN_YUV422, 									
									eVIDEOIN_OUT_YUV422);											
	#if 0
	vin_priv->pDevVin->SetCropWinStartAddr(40,					//UINT16 u16VerticalStart, 		Y
									4);					//UINT16 u16HorizontalStart, 	X
	#else
	vin_priv->pDevVin->SetCropWinStartAddr(CROP_START_Y,					//UINT16 u16VerticalStart, 		Y
									CROP_START_X);					//UINT16 u16HorizontalStart, 	X
	#endif
#if 0
	vin_priv->pDevVin->SetInputType(0,	/* 0: Both fields are disabled. 1: Field 1 enable. 2: Field 2 enable. 3: Both fields are enable */
				eVIDEOIN_TYPE_CCIR656,	/* 0: CCIR601.	1: CCIR656 */
				0);						/* swap? */
 	vin_priv->pDevVin->SetStandardCCIR656(TRUE);		/* standard CCIR656 mode*/
#endif
	
	
	/* Sensor subsample resolution (640, 480)*/
	vin_priv->pDevVin->SetCropWinSize(vin_priv->sensor_intf->u16MaxImgHeight,		//UINT16 u16Height, 
							 vin_priv->sensor_intf->u16MaxImgWidth);		//UINT16 u16Width;									 
	
	vin_priv->pDevVin->PreviewPipeSize(vin_priv->videowin.height, vin_priv->videowin.width);
	
	vin_priv->pDevVin->GetStride(&u32PacStride, &u32PlaStride);
	vin_priv->pDevVin->SetStride(vin_priv->videowin.width, u32PlaStride);
	//vin_priv->pDevVin->SetFrameRateScaleFactor(1, 2);	
	vin_priv->pDevVin->EncodePipeSize(240, 320);
	vin_priv->pDevVin->PreviewPipeSize(240, 320);
	vin_priv->pDevVin->SetPipeEnable(TRUE,							// It means planar disable
								eVIDEOIN_PACKET);

	vin_priv->pDevVin->SetShadowRegister();
	vin_priv->pDevVin->SetOperationMode(TRUE);
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

//#define TVP5150_BRIGHT_CTL			0x10
//#define TVP5150_CONTRAST_CTL			0x11
//#define TVP5150_SATURATION_CTL		0x13

static BOOL	
OVReadWriteBrightness(
	void *priv, 
	INT32 *pi32Value, 
	BOOL bIsRead
)
{
	videoin_priv_t *vin_priv = (videoin_priv_t *) priv;
	if(vin_priv->sensor_intf->u8SensorDevID != g_uOvDeviceID[0])
		return FALSE;
	
	if(bIsRead)
		*pi32Value = (DrvVideoIn_I2cReadOV(vin_priv->sensor_intf->u8SensorDevID, TVP5150_BRIGHT_CTL)&0xff);
	else
		DrvVideoIn_I2cWriteOV(vin_priv->sensor_intf->u8SensorDevID, TVP5150_BRIGHT_CTL, *pi32Value);

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
	if(vin_priv->sensor_intf->u8SensorDevID != g_uOvDeviceID[0])
		return FALSE;
	if(bIsRead)
		*pi32Value = (DrvVideoIn_I2cReadOV(vin_priv->sensor_intf->u8SensorDevID, TVP5150_CONTRAST_CTL)&0xff);
	else
		DrvVideoIn_I2cWriteOV(vin_priv->sensor_intf->u8SensorDevID, TVP5150_CONTRAST_CTL, *pi32Value);

	return TRUE;
}
#if 0
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
#endif
static BOOL	
OVReadWriteColorSaturation(
	void *priv, 
	INT32 *pi32Value, 
	BOOL bIsRead
)
{
	videoin_priv_t *vin_priv = (videoin_priv_t *) priv;
	if(vin_priv->sensor_intf->u8SensorDevID != g_uOvDeviceID[0])
		return FALSE;
	if(bIsRead)
		*pi32Value = (DrvVideoIn_I2cReadOV(vin_priv->sensor_intf->u8SensorDevID, TVP5150_SATURATION_CTL)&0xff);
	else{
		DrvVideoIn_I2cWriteOV(vin_priv->sensor_intf->u8SensorDevID, TVP5150_SATURATION_CTL, *pi32Value);		//U Saturation
		DrvVideoIn_I2cWriteOV(vin_priv->sensor_intf->u8SensorDevID, TVP5150_SATURATION_CTL+1, *pi32Value);		//V Saturation
	}
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
	if( ((w55fa92_apll_clock%24000)==0) && ((w55fa92_apll_clock%20000)==0) && ((w55fa92_apll_clock%16000)==0) )
		u32PllClock = w55fa92_apll_clock*1000;
	else
		u32PllClock = w55fa92_upll_clock*1000;
	uDiv0 = ((inp32(REG_CLKDIV0)&SENSOR0_N0)>> 19)+1;
	uDiv1 = ((inp32(REG_CLKDIV0)&SENSOR0_N1)>> 24)+1;
#endif
#ifdef CONFIG_W55FA92_VIDEOIN_DEV2
//	u32PllClock = w55fa92_upll_clock*1000;
//	if(u32PllClock==0)
//		u32PllClock = w55fa92_apll_clock*1000;
//	uDiv0 = ((inp32(REG_CLKDIV5)&SENSOR1_N0)>> 13)+1;
//	uDiv1 = ((inp32(REG_CLKDIV5)&SENSOR1_N1)>> 18)+1;
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
	outp32(REG_CLKDIV0, (inp32(REG_CLKDIV0) & ~(SENSOR0_N1 | SENSOR0_N0)) | ((uDiv0<<19) | (uDiv1<<24)) );
#endif
#ifdef CONFIG_W55FA92_VIDEOIN_DEV2
//	outp32(REG_CLKDIV5, (inp32(REG_CLKDIV5) & ~(SENSOR1_N1 | SENSOR1_N0)) | ((uDiv0<<13) | (uDiv1<<18)) );
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
	sensor_init:				InitSensor,
	sensor_poweron:				PoweronSensor,

#ifdef CONFIG_SENSOR_POWERDOWN
	sensor_suspend:				Snr1_PowerDown,
#else
	sensor_suspend:				Snr1_PowerDown,
#endif
#ifdef CONFIG_SENSOR_RESET
	sensor_reset:				Snr1_Reset,
#else	
	sensor_reset:				Snr1_Reset,
#endif
	read_write_brightness:			OVReadWriteBrightness,
	read_write_contrast:			OVReadWriteContrast,
	read_write_sharpness:			NULL, //OVReadWriteSharpness,
	read_write_white_balance:		NULL, //OVReadWriteWhiteBalance,
	read_write_noise_reduction:		NULL, //OVReadWriteNoiseReduction,
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
	u16MaxImgHeight:				CONFIG_VIN_DEV1_ENCODE_HEIGHT,		 
	u16MaxImgWidth: 				CONFIG_VIN_DEV1_ENCODE_WIDTH,
#endif	
};
#ifdef CONFIG_W55FA92_VIDEOIN_DEV1
INT32 register_vin_port1_Sensor(NVT_SENSOR_T **sensor_intf)
{
	*sensor_intf = (NVT_SENSOR_T*)(&nvt_sensor_ov);
	return 0;
}
#endif


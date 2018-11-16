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
#endif
#include <linux/videodev2.h>
#include <media/v4l2-device.h>
#include <linux/jiffies.h>
#if 0
#include <asm/arch/videoin.h>
#include <asm/arch/DrvVideoin.h>
#include <asm/arch/w55fa93_reg.h>
#include <asm/arch/fb.h>
#include <asm/arch/w55fa93_fb.h>
#include <asm/arch/w55fa93_gpio.h>
#endif
#include <mach/w55fa92_reg.h>
#include <mach/fb.h>
#include <mach/w55fa92_fb.h>
#include <mach/videoin.h>
//#include <mach/DrvVideoin.h>
#include "DrvVideoin.h"
#include <mach/videodev_ex.h>
#include <mach/w55fa92_gpio.h>

#include <asm/io.h>
#include <linux/i2c.h>

#include "videoinpriv.h"
#include "DrvI2C.h"

//#define LCDWIDTH	480
//#define LCDHEIGHT	272
//#define LCDBPP		16
#ifndef CONFIG_NON_STANDARD_I2C_DEV2
#define __STANDARD_I2C__
#endif

#define CHIP_VERSION_H		0x3000
#define CHIP_VERSION_L		0x3001
#define NT99142_CHIP_ID_H	0x14
#define NT99142_CHIP_ID_L	0x20

#define _REG_TABLE_SIZE(nTableName)	sizeof(nTableName)/sizeof(struct NT_RegValue)

#define ERR_PRINTF			printk
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
//#define outp32(addr, value)		outl(value, addr)
//#define inp32(addr)			inl(addr)
//#define DBG_PRINTF	printk
//#define DBG_PRINTF(...)
//extern videoIn_buf_t videoIn_buf[];
//extern videoIn_buf_t videoIn_preview_buf[];

#define REG_VALUE_INIT		0
#define REG_VALUE_VGA		1	//640x480
#define REG_VALUE_SVGA		2	//720X480
#define REG_VALUE_HD720		3	//1280X720

struct NT_RegValue{
	__u16	uRegAddr;
	__u8	uValue;
};

struct NT_RegTable{
	struct NT_RegValue *sRegTable;
	__u16 uTableSize;
};

extern VINDEV_T* pDevVin;
/* 20MHz sensor clock, 60MHz Pixel clock */
static struct NT_RegValue g_sNT99142_Init[] = 
{//20MHz sensor clock
	#include "NT99142/NT99142_Init.dat" 
};
static struct NT_RegValue g_sNT99142_HD720[] = 
{//20MHz sensor clock
#ifdef CONFIG_TARGET_25FPS_DEV2	
	#include "NT99142/NT99142_HD720_PCLK_60MHz.dat"
#elif defined CONFIG_TARGET_23FPS_DEV2
	#include "NT99142/NT99142_HD720_PCLK_56MHz.dat"
#elif defined CONFIG_TARGET_15FPS_DEV2
	#include "NT99142/NT99142_HD720_PCLK_60MHz_15FPS.dat"
#elif defined CONFIG_TARGET_10FPS_DEV2
	#include "NT99142/NT99142_HD720_PCLK_60MHz_10FPS.dat"
#endif
};
static struct NT_RegValue g_sNT99142_SVGA[] = 
{//20MHz sensor clock
#ifdef CONFIG_TARGET_25FPS_DEV2	
	#include "NT99142/NT99142_SVGA_PCLK_60MHz.dat"
#elif defined CONFIG_TARGET_23FPS_DEV2
	#include "NT99142/NT99142_SVGA_PCLK_56MHz.dat"
#elif defined CONFIG_TARGET_15FPS_DEV2
	#include "NT99142/NT99142_SVGA_PCLK_60MHz_15FPS.dat"
#elif defined CONFIG_TARGET_10FPS_DEV2
	#include "NT99142/NT99142_SVGA_PCLK_60MHz_10FPS.dat"
#endif
};
static struct NT_RegValue g_sNT99142_VGA[] = 
{//20MHz sensor clock
#ifdef CONFIG_TARGET_25FPS_DEV2	
  	#include "NT99142/NT99142_VGA_PCLK_60MHz.dat"
#elif defined CONFIG_TARGET_23FPS_DEV2
	#include "NT99142/NT99142_VGA_PCLK_56MHz.dat"
#elif defined CONFIG_TARGET_15FPS_DEV2
	#include "NT99142/NT99142_VGA_PCLK_60MHz_15FPS.dat"
#elif defined CONFIG_TARGET_10FPS_DEV2
	#include "NT99142/NT99142_VGA_PCLK_60MHz_10FPS.dat"
#endif
};
static struct NT_RegTable g_NT99142_InitTable[] =
{
	{g_sNT99142_Init,_REG_TABLE_SIZE(g_sNT99142_Init)},
	{g_sNT99142_VGA,_REG_TABLE_SIZE(g_sNT99142_VGA)},	
	{g_sNT99142_SVGA,_REG_TABLE_SIZE(g_sNT99142_SVGA)},		
	{g_sNT99142_HD720,_REG_TABLE_SIZE(g_sNT99142_HD720)},
	{0,0}
};


/* 16MHz sensor clock, 48MHz Pixel clock */
static struct NT_RegValue g_sNT99142_16M_HD720[] = 
{//16MHz sensor clock
#ifdef CONFIG_TARGET_25FPS_DEV2	
	#include "NT99142/NT99142_HD720_PCLK_48MHz.dat"
#elif defined CONFIG_TARGET_15FPS_DEV2
	#include "NT99142/NT99142_HD720_PCLK_48MHz_15FPS.dat"
#elif defined CONFIG_TARGET_10FPS_DEV2
	#include "NT99142/NT99142_HD720_PCLK_48MHz_10FPS.dat"
#endif
};
static struct NT_RegValue g_sNT99142_16M_SVGA[] = 
{//16MHz sensor clock
#ifdef CONFIG_TARGET_25FPS_DEV2	
	#include "NT99142/NT99142_SVGA_PCLK_48MHz.dat"
#elif defined CONFIG_TARGET_15FPS_DEV2
	#include "NT99142/NT99142_SVGA_PCLK_48MHz_15FPS.dat"
#elif defined CONFIG_TARGET_10FPS_DEV2
	#include "NT99142/NT99142_SVGA_PCLK_48MHz_10FPS.dat"
#endif
};
static struct NT_RegValue g_sNT99142_16M_VGA[] = 
{//16MHz sensor clock
#ifdef CONFIG_TARGET_25FPS_DEV2	
  	#include "NT99142/NT99142_VGA_PCLK_48MHz.dat"
#elif defined CONFIG_TARGET_15FPS_DEV2
	#include "NT99142/NT99142_VGA_PCLK_48MHz_15FPS.dat"
#elif defined CONFIG_TARGET_10FPS_DEV2
	#include "NT99142/NT99142_VGA_PCLK_48MHz_10FPS.dat"
#endif
};
static struct NT_RegTable g_NT99142_16M_InitTable[] =
{
	{g_sNT99142_Init,_REG_TABLE_SIZE(g_sNT99142_Init)},
	{g_sNT99142_16M_VGA,_REG_TABLE_SIZE(g_sNT99142_16M_VGA)},	
	{g_sNT99142_16M_SVGA,_REG_TABLE_SIZE(g_sNT99142_16M_SVGA)},		
	{g_sNT99142_16M_HD720,_REG_TABLE_SIZE(g_sNT99142_16M_HD720)},
	{0,0}
};
static __u8 g_uOvDeviceID= 0x54;	// NT99142
#ifdef __STANDARD_I2C__
static struct i2c_client *save_client;
static struct i2c_client i2c_client; 
static int sensor_probe(struct i2c_client *client,
			const struct i2c_device_id *did)
{
	int ret = 0;
	memcpy(&i2c_client, client, sizeof(struct i2c_client));
	save_client = &i2c_client;
	return ret;
}

static int sensor_remove(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id sensor_id[] = {

	{ "nt99142_dev2", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, sensor_id);

static struct i2c_driver sensor_i2c_driver = {
	.driver = {
		.name = "nt99142_dev2",
	},
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
unsigned int uCount=0;
static BOOL I2C_Write_8bitSlaveAddr_16bitReg_8bitData(UINT8 uAddr, UINT16 uRegAddr, UINT8 uData)
{
	// 3-Phase(ID address, regiseter address, data(8bits)) write transmission
	volatile UINT32 u32Delay = 0x100;
	DrvI2C_SendStart();
	while(u32Delay--);		
	if ( (DrvI2C_WriteByte(uAddr,DrvI2C_Ack_Have,8)==FALSE) ||			// Write ID address to sensor
		 (DrvI2C_WriteByte((UINT8)(uRegAddr>>8),DrvI2C_Ack_Have,8)==FALSE) ||	// Write register address to sensor
		 (DrvI2C_WriteByte((UINT8)(uRegAddr&0xff),DrvI2C_Ack_Have,8)==FALSE) ||	// Write register address to sensor
		 (DrvI2C_WriteByte(uData,DrvI2C_Ack_Have,8)==FALSE) )		// Write data to sensor
	{
		printk("I2C NACK\n");
		DrvI2C_SendStop();
		return FALSE;
	}
	DrvI2C_SendStop();
	return TRUE;
}

static UINT8 I2C_Read_8bitSlaveAddr_16bitReg_8bitData(UINT8 uAddr, UINT16 uRegAddr)
{
	UINT8 u8Data;
	
	// 3-Phase(ID address, register address) write transmission
	volatile UINT32 u32Delay = 0x100;
	DrvI2C_SendStart();
	while(u32Delay--);			
	DrvI2C_WriteByte(uAddr,DrvI2C_Ack_Have,8);		// Write ID address to sensor
	DrvI2C_WriteByte((UINT8)(uRegAddr>>8),DrvI2C_Ack_Have,8);	// Write register address to sensor
	DrvI2C_WriteByte((UINT8)(uRegAddr&0xff),DrvI2C_Ack_Have,8);	// Write register address to sensor	
	DrvI2C_SendStop();

	// 2-Phase(ID-address, data(8bits)) read transmission
	DrvI2C_SendStart();
	DrvI2C_WriteByte(uAddr|0x01,DrvI2C_Ack_Have,8);		// Write ID address to sensor
	u8Data = DrvI2C_ReadByte(DrvI2C_Ack_Have,8);		// Read data from sensor
	DrvI2C_SendStop();
	
	return u8Data;

}
static void I2C_Delay(UINT32 u32Delay)
{
	volatile UINT32 i;
	for(;u32Delay!=0;u32Delay--)
		for(i=0;i<50;i++);
}
#endif 
static s8  DrvVideoIn_I2cWriteNT(__u8 uAddr, __u16 uRegAddr, __u8 uData)
{
	//DBG_PRINTF("%s\n",__FUNCTION__);	
#ifdef __STANDARD_I2C__
	struct i2c_msg msg;
	u8 buf[3];
	int ret=-1;
	#ifdef CONFIG_SHARE_SENSOR
		return 0;
	#endif			
	msg.flags=!I2C_M_RD;
	msg.addr=save_client->addr;
	msg.len=3;
	msg.buf=buf;		

	buf[0]=(u8)(uRegAddr>>8);
	buf[1]=(u8)(uRegAddr&0xff);
	buf[2]=uData;

	ret=i2c_transfer(save_client->adapter,&msg,1);
	return ret;
		
#else
	#ifdef CONFIG_SHARE_SENSOR
		return 0;
	#endif		
	I2C_Write_8bitSlaveAddr_16bitReg_8bitData(uAddr, uRegAddr, uData);
#endif
	return TRUE;
}


static __s8  DrvVideoIn_I2cReadNT(__u8 uAddr, __u16 uRegAddr)
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
	msgs.len=2;
	msgs.buf=buf;
	buf[0]=(u8)(uRegAddr>>8);
	buf[1]=(u8)(uRegAddr&0xff);

	ret=i2c_transfer(save_client->adapter,&msgs,1);
	
	msgs.flags=I2C_M_RD;
	msgs.addr=save_client->addr;
	msgs.len=1;
	msgs.buf=buf;

	ret=i2c_transfer(save_client->adapter,&msgs,1);
	return buf[0];
#else
	#ifdef CONFIG_SHARE_SENSOR
		return 0;
	#endif		
	return I2C_Read_8bitSlaveAddr_16bitReg_8bitData(uAddr,uRegAddr);
#endif
}
static int s_i32SensorResIndex; 
static int s_i32SensorClk = 20000;
static void NTSetResolution(int index)
{
  	__u32 i;
	__u16 uTableSize;
	__u8  uDeviceID;
	__u16 ExprosureH,ExprosureL,Exprosure;
	__u8 AEstep = 0, Step = 0;
	
	struct NT_RegValue *psRegValue;

	s_i32SensorResIndex = index;
	printk("NTSetResolution:%d\n",index);
	printk("sensor change resolution begin----- \n");
	if(index>REG_VALUE_HD720)
		return ;
	uDeviceID = g_uOvDeviceID;

	if(index!=REG_VALUE_INIT){
		//setting exporsure time
		ExprosureH = DrvVideoIn_I2cReadNT(uDeviceID,0x3012);
		ExprosureL = DrvVideoIn_I2cReadNT(uDeviceID,0x3013);
		AEstep = DrvVideoIn_I2cReadNT(uDeviceID,0x32c8);
		Exprosure = (ExprosureH<<8) | ExprosureL;
		Step = Exprosure / (AEstep*2);	
	}	
    if(s_i32SensorClk > 16000){
		uTableSize = g_NT99142_InitTable[0].uTableSize;
		psRegValue = g_NT99142_InitTable[0].sRegTable;
	}else if(s_i32SensorClk <= 16000){
		uTableSize = g_NT99142_16M_InitTable[0].uTableSize;
		psRegValue = g_NT99142_16M_InitTable[0].sRegTable;
	}
	printk("Programming sensor init value\n");
	for(i=0;i<uTableSize; i++, psRegValue++){
		uint8_t id0;
		udelay(10);
		DrvVideoIn_I2cWriteNT(uDeviceID, (psRegValue->uRegAddr), (psRegValue->uValue));
#if 1
		id0=DrvVideoIn_I2cReadNT(uDeviceID,(psRegValue->uRegAddr));
		if(id0!=(psRegValue->uValue)){
			DBG_PRINTF("reg=0x%04x w=0x%02x r=0x%02x\n",(psRegValue->uRegAddr),(psRegValue->uValue),id0);			
		}			
#endif

	}
	printk("Programming sensor resolution value = %d\n", index);
	if(s_i32SensorClk > 16000){
		uTableSize = g_NT99142_InitTable[index].uTableSize;
		psRegValue = g_NT99142_InitTable[index].sRegTable;
	}else if(s_i32SensorClk <= 16000){
		uTableSize = g_NT99142_16M_InitTable[index].uTableSize;
		psRegValue = g_NT99142_16M_InitTable[index].sRegTable;
	}
	for(i=0;i<uTableSize; i++, psRegValue++){
		uint8_t id0;
		udelay(10);
		DrvVideoIn_I2cWriteNT(uDeviceID, (psRegValue->uRegAddr), (psRegValue->uValue));
		if(psRegValue->uRegAddr==0x3021){//Software reset delay a period
			mdelay(10);
		}
#if 0
		id0=DrvVideoIn_I2cReadNT(uDeviceID,(psRegValue->uRegAddr));
		if(id0!=(psRegValue->uValue)){
			DBG_PRINTF("reg=0x%04x w=0x%02x r=0x%02x\n",(psRegValue->uRegAddr),(psRegValue->uValue),id0);			
		}			
#endif
	}
/*
	if(index!=REG_VALUE_INIT){
		//setting exporsure time
		DrvVideoIn_I2cWriteNT(uDeviceID, 0x3201,(0xdf&(DrvVideoIn_I2cReadNT(uDeviceID,0x3201))));
		AEstep = DrvVideoIn_I2cReadNT(uDeviceID,0x32c8);
		Exprosure = Step * (AEstep*2);
		ExprosureL = Exprosure & 0x00ff;
		ExprosureH =  Exprosure >> 8;
		DrvVideoIn_I2cWriteNT(uDeviceID, 0x3012,(UINT8)ExprosureH);
		DrvVideoIn_I2cWriteNT(uDeviceID, 0x3013,(UINT8)ExprosureL);
		DrvVideoIn_I2cWriteNT(uDeviceID, 0x3201,(0x20|(DrvVideoIn_I2cReadNT(uDeviceID,0x3201))));
		DrvVideoIn_I2cWriteNT(uDeviceID, 0x3060,0x01);
	}
*/
	printk("sensor change resolution end-----\n");
	return ;
}

/*
	NT99142 power on sequence
	
RST	  		  | <T1>|-----------------------
--------------------|

MCLK----------||||||||||||

T1 need 300 MCLK 

*/
#ifndef CONFIG_SHARE_SENSOR
static void NT99142RegConfig(void)
{
  	__u32 i;
	__u16 uTableSize;
	__u8  uDeviceID;
	__u8 id0,id1;
	struct NT_RegValue *psRegValue;

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
	
	uTableSize = g_NT99142_InitTable[REG_VALUE_INIT].uTableSize;
	psRegValue = g_NT99142_InitTable[REG_VALUE_INIT].sRegTable;
	uDeviceID = g_uOvDeviceID;
	 
	DBG_PRINTF("uDeviceID = 0x%x\n", uDeviceID);
	//DBG_PRINTF("REG_GPBFUN = 0x%x\n", inp32(REG_GPBFUN));

	/*check device id*/
	if(1){
		id0=(u8)DrvVideoIn_I2cReadNT(g_uOvDeviceID,CHIP_VERSION_H);
		id1=(u8)DrvVideoIn_I2cReadNT(g_uOvDeviceID,CHIP_VERSION_L);
		printk("detectd sensor id0=%0x id1=%02x\n",id0,id1);
	}

	/*camera init*/
	if ( psRegValue == 0 ){
		DBG_PRINTF("NTRegConfig psRegValue == 0");
		return;	
	}
	for(i=0;i<uTableSize; i++, psRegValue++)
	{
		udelay(10);
		DrvVideoIn_I2cWriteNT(uDeviceID, (psRegValue->uRegAddr), (psRegValue->uValue));
#if 1
		id0=DrvVideoIn_I2cReadNT(uDeviceID,(psRegValue->uRegAddr));
		if(id0!=(psRegValue->uValue)){
			DBG_PRINTF("reg=0x%04x w=0x%02x r=0x%02x\n",(psRegValue->uRegAddr),(psRegValue->uValue),id0);			
		}			
#endif
	}

	/*set to vga*/
	//NTSetResolution(REG_VALUE_HD720);
	//NTSetResolution(REG_VALUE_SVGA);
	//NTSetResolution(REG_VALUE_VGA);

}
#endif
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
	videoin_priv_t *vin_priv = (videoin_priv_t *) priv;
	DBG_PRINTF("%s\n",__FUNCTION__);	

	printk("Init NT_99142 \n");
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
			printk("Reset and non power-down sensor\n");
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
							
	vin_priv->pDevVin->Open(96000, 20000);
	vin_priv->sensor_intf->u8SensorDevID = g_uOvDeviceID;
#ifndef CONFIG_SHARE_SENSOR
	schedule_mdelay(10);
	NT99142RegConfig();	
	NTSetResolution(REG_VALUE_VGA);

	vin_priv->sensor_intf->u16CurImgHeight = 480;
	vin_priv->sensor_intf->u16CurImgWidth = 640;
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

	vin_priv->pDevVin->EnableInt(eVIDEOIN_VINT);
	vin_priv->pDevVin->SetSensorPolarity(FALSE, FALSE, TRUE);
	vin_priv->pDevVin->SetDataFormatAndOrder(eVIDEOIN_IN_VYUY, 
									eVIDEOIN_IN_YUV422, 									
									eVIDEOIN_OUT_YUV422);			
	vin_priv->pDevVin->SetCropWinStartAddr(0,					//UINT16 u16VerticalStart, 	Y
								0);					//UINT16 u16HorizontalStart, 	X

	vin_priv->videocrop.c.width = vin_priv->sensor_intf->u16CurImgWidth;
	vin_priv->videocrop.c.height = vin_priv->sensor_intf->u16CurImgHeight;
	vin_priv->videocrop.c.left = 0;	/*X*/
	vin_priv->videocrop.c.top = 0;	/*Y*/


	/* Sensor subsample resolution (640, 480)*/
	vin_priv->pDevVin->SetCropWinSize(vin_priv->sensor_intf->u16CurImgHeight, vin_priv->sensor_intf->u16CurImgWidth);
	vin_priv->pDevVin->PreviewPipeSize(vin_priv->videowin.height, vin_priv->videowin.width);	

	
	vin_priv->pDevVin->GetStride(&u32PacStride, &u32PlaStride);
	vin_priv->pDevVin->SetStride(vin_priv->videowin.width, u32PlaStride);
	vin_priv->pDevVin->SetPipeEnable(FALSE,							// It means planar disable
								eVIDEOIN_PACKET);		//	

	vin_priv->pDevVin->SetShadowRegister();

	return TRUE;	
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
NTReadWriteBrightness(
	void *priv, 
	INT32 *pi32Value, 
	BOOL bIsRead
)
{
	videoin_priv_t *vin_priv = (videoin_priv_t *) priv;
	if(vin_priv->sensor_intf->u8SensorDevID != g_uOvDeviceID)
		return FALSE;
	if(bIsRead)
		*pi32Value = (DrvVideoIn_I2cReadNT(vin_priv->sensor_intf->u8SensorDevID, 0x32fc)&0xff);
	else
		DrvVideoIn_I2cWriteNT(vin_priv->sensor_intf->u8SensorDevID, 0x32fc, *pi32Value);

	return TRUE;
}


static BOOL	
NTReadWriteSharpness(
	void *priv, 
	INT32 *pi32Value, 
	BOOL bIsRead
)
{
	videoin_priv_t *vin_priv = (videoin_priv_t *) priv;
	if(vin_priv->sensor_intf->u8SensorDevID != g_uOvDeviceID)
		return FALSE;
	if(bIsRead)
		*pi32Value = (DrvVideoIn_I2cReadNT(vin_priv->sensor_intf->u8SensorDevID, 0x3301)&0xff);
	else
		DrvVideoIn_I2cWriteNT(vin_priv->sensor_intf->u8SensorDevID, 0x3301, *pi32Value);

	return TRUE;
}

static BOOL	
NTReadWriteNoiseReduction(
	void *priv, 
	INT32 *pi32Value, 
	BOOL bIsRead
)
{
	videoin_priv_t *vin_priv = (videoin_priv_t *) priv;
	if(vin_priv->sensor_intf->u8SensorDevID != g_uOvDeviceID)
		return FALSE;
	if(bIsRead)
		*pi32Value = (DrvVideoIn_I2cReadNT(vin_priv->sensor_intf->u8SensorDevID, 0x3300)&0x3f);
	else
		DrvVideoIn_I2cWriteNT(vin_priv->sensor_intf->u8SensorDevID, 0x3300, (*pi32Value & 0x3f));

	return TRUE;
}


static BOOL	
NTSetFlickerFreq(
	void *priv, 
	UINT32 u32FlickerFreq
)
{
	UINT8 u8AECntl0; 
	videoin_priv_t *vin_priv = (videoin_priv_t *) priv;
	if(vin_priv->sensor_intf->u8SensorDevID != g_uOvDeviceID)
		return FALSE;
	if((u32FlickerFreq != 50) && (u32FlickerFreq != 60))
		return FALSE;
	u8AECntl0 = (DrvVideoIn_I2cReadNT(vin_priv->sensor_intf->u8SensorDevID, 0x32BB)&0xff);
	if(u32FlickerFreq == 50){
		u8AECntl0 = u8AECntl0 & ~0xa;
		u8AECntl0 = u8AECntl0 | 0xa;
	}
	else{
		u8AECntl0 = u8AECntl0 & ~0xa;
		u8AECntl0 = u8AECntl0 | 0x2;
	}
	DrvVideoIn_I2cWriteNT(vin_priv->sensor_intf->u8SensorDevID, 0x32BB, u8AECntl0);
	return TRUE;
}

typedef struct
{
	UINT16 u16ImageWidth; 
	UINT16 u16ImageHeight;
	UINT8 i8ResolIdx;
}S_NTSuppResol;

#define NT_RESOL_SUPP_CNT  4

static S_NTSuppResol s_asNTSuppResolTable[NT_RESOL_SUPP_CNT] = {
	{0, 0, REG_VALUE_INIT},	
	{640, 480, REG_VALUE_VGA},
	{800, 600, REG_VALUE_SVGA},
	{1280, 720, REG_VALUE_HD720}
};




static BOOL	
NTChangeImageResolution(
	void *priv, 
	UINT16 u16ImageWidth, 
	UINT16 u16ImageHeight
)
{
	INT8 i;
	INT8 i8WidthIdx;
	INT8 i8HeightIdx;
	INT8 i8SensorIdx;	
	videoin_priv_t *vin_priv = (videoin_priv_t *) priv;

#ifdef CONFIG_SHARE_SENSOR	
	/* If share sensor from capture port 0, get port 0 sensor output dimension */										
	GetVideo0SensorResoultion(&u16ImageWidth, &u16ImageHeight);											
#endif
		
	if(u16ImageWidth > vin_priv->sensor_intf->u16MaxImgWidth){
		printk("The specified width over the sensor supported\n");
		return FALSE;
	}
	if(u16ImageHeight > vin_priv->sensor_intf->u16MaxImgHeight){
		printk("The specified height over the sensor supported\n");
		return FALSE;
	}
	
	printk("NTChangeImageResolution:%dx%d\n", u16ImageWidth, u16ImageHeight);
	for(i = 0; i < NT_RESOL_SUPP_CNT ; i ++){
		if(u16ImageWidth <= s_asNTSuppResolTable[i].u16ImageWidth)
			break;
	}
	if(i == NT_RESOL_SUPP_CNT)
		return FALSE;
	i8WidthIdx = i;
	for(i = 0; i < NT_RESOL_SUPP_CNT ; i ++){
		if(u16ImageHeight <= s_asNTSuppResolTable[i].u16ImageHeight)
			break;
	}
	if(i == NT_RESOL_SUPP_CNT)
		return FALSE;
	i8HeightIdx = i;

	if(i8HeightIdx >= i8WidthIdx){
		i8SensorIdx = i8HeightIdx;
	}
	else{
		i8SensorIdx = i8WidthIdx;
	}

	//NT99142RegConfig();			
	NTSetResolution(s_asNTSuppResolTable[i8SensorIdx].i8ResolIdx);

	vin_priv->sensor_intf->u16CurImgHeight = s_asNTSuppResolTable[i8SensorIdx].u16ImageHeight;
	vin_priv->sensor_intf->u16CurImgWidth = s_asNTSuppResolTable[i8SensorIdx].u16ImageWidth;

	/* sw 2013 0227 add */	
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
	
	vin_priv->pDevVin->SetCropWinSize(vin_priv->sensor_intf->u16CurImgHeight, vin_priv->sensor_intf->u16CurImgWidth);
	vin_priv->pDevVin->PreviewPipeSize(vin_priv->videowin.height, vin_priv->videowin.width);

	return TRUE;
}

static BOOL	
NTIRLedOnOff(
	void *priv, 
	BOOL bIsOn
)
{
	if(bIsOn){
#if 0// CONFIG_SENSOR_NT99142_IR
			printk("IR led on \n");
			w55fa93_gpio_set(GPIO_GROUP_D, 5, 1);
#endif
	}
	else{
#if 0// CONFIG_SENSOR_NT99142_IR
			printk("IR led off \n");
			w55fa93_gpio_set(GPIO_GROUP_D, 5, 0);
#endif

	}
	return TRUE;
}

#define LOW_LUX_GATE	0x0A00
#define HIGH_LUX_GATE	0x07C5

static int s_i32PrintCnt = 0;
static BOOL s_bIsLowLux = FALSE;

static BOOL	
NTLowLuxDetect(
	void *priv
)
{
	UINT8 u8ShutterH; 
	UINT8 u8ShutterL;
	UINT16 u16Shutter;
	UINT8 u8RegGain; 
	UINT32 u32Gain;
	UINT32 u32AE;

	videoin_priv_t *vin_priv = (videoin_priv_t *) priv;
	if(vin_priv->sensor_intf->u8SensorDevID != g_uOvDeviceID)
		return FALSE;
	
	s_i32PrintCnt ++;
	if((s_i32PrintCnt % 2) != 0)
		return s_bIsLowLux;


	u8ShutterH = (DrvVideoIn_I2cReadNT(vin_priv->sensor_intf->u8SensorDevID, 0x3012)&0xff);
	u8ShutterL = (DrvVideoIn_I2cReadNT(vin_priv->sensor_intf->u8SensorDevID, 0x3013)&0xff);

	u16Shutter = (uint16_t)(u8ShutterH << 8) | u8ShutterL;

	u8RegGain = (DrvVideoIn_I2cReadNT(vin_priv->sensor_intf->u8SensorDevID, 0x301d)&0xff);
	u32Gain = (((u8RegGain & 0x80) >> 7) + 1) * (((u8RegGain & 0x40) >> 6) + 1) *
			(((u8RegGain & 0x20) >> 5) + 1) * (((u8RegGain & 0x10) >> 4) + 1) * ((((u8RegGain & 0x0F) + 1) / 16) + 1);

	u32AE = u16Shutter * u32Gain;

	if(s_i32PrintCnt >= 30){
		s_i32PrintCnt = 0;
	}
 
	if(u32AE >= LOW_LUX_GATE){
		if(s_bIsLowLux == FALSE){
			printk("lux detect low \n");
			s_bIsLowLux = TRUE;
			NTIRLedOnOff(priv, TRUE);
		}
	}
	else if(u32AE <= HIGH_LUX_GATE){
		if(s_bIsLowLux == TRUE){
			printk("lux detect high \n");
			s_bIsLowLux = FALSE;
			NTIRLedOnOff(priv, FALSE);
		}
	} 

	return s_bIsLowLux;
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

#define V4L2_CID_PRIVATE_LASTP1     				 (V4L2_CID_PRIVATE_BASE + 5)

static const struct v4l2_queryctrl no_ctrl = {
	.name  = "42",
	.flags = V4L2_CTRL_FLAG_DISABLED,
};
static const struct v4l2_queryctrl video_ctrls[] = {
	/* --- private --- */
	{
		.id            	= V4L2_CID_PRIVATE_I2C_SET_REG_ADDR,
		.name          	= "i2c_set_addr",
		.minimum       = 0x3000,
		.maximum       = 0x3400,
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

	vin_priv = (videoin_priv_t *) priv;
	if(vin_priv->sensor_intf->u8SensorDevID != g_uOvDeviceID)
		return FALSE;
	DrvVideoIn_I2cWriteNT(vin_priv->sensor_intf->u8SensorDevID, u16SensRegAddr, c->value);
	return TRUE;
}
static BOOL	
SensorI2cReadData(
	void *priv, 
	struct v4l2_control *c
)
{
	videoin_priv_t *vin_priv = (videoin_priv_t *) priv;
	if(vin_priv->sensor_intf->u8SensorDevID != g_uOvDeviceID)
		return FALSE;
	c->value = (DrvVideoIn_I2cReadNT(vin_priv->sensor_intf->u8SensorDevID, u16SensRegAddr)&0xff);
	return TRUE;
}

static INT32 
SensorI2cSetRegAddr(
	void *priv, 
	struct v4l2_control *c
)
{
	u16SensRegAddr  = c->value;
	printk("Specified sensor addr = 0x%x\n", u16SensRegAddr);
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

	if(vin_priv->sensor_intf->u8SensorDevID != g_uOvDeviceID)
		return FALSE;

	/* Platform dependence */
#ifdef CONFIG_W55FA92_VIDEOIN_DEV1
//	if( ((w55fa92_apll_clock%24000)==0) && ((w55fa92_apll_clock%20000)==0) && ((w55fa92_apll_clock%16000)==0) )
//		u32PllClock = w55fa92_apll_clock*1000;
//	else
//		u32PllClock = w55fa92_upll_clock*1000;
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

	if(vin_priv->sensor_intf->u8SensorDevID != g_uOvDeviceID)
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

	printk("old sensor clock: %d\n", s_i32SensorClk);
	s_i32SensorClk = c->value / 1000;
	printk("new sensor clock: %d\n", s_i32SensorClk);

	NTSetResolution(s_i32SensorResIndex);

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
static int SensorI2cReadCtrl(void *priv,
				 	struct v4l2_control *c)
{
	const struct v4l2_queryctrl* ctrl;

	ctrl = ctrl_by_id(c->id);
	DBG_PRINTF("Get_control name=%s\n",ctrl->name);
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
static NVT_SENSOR_T nvt_sensor_nt99142 = {
	sensor_init:					InitSensor,
	sensor_poweron:					PoweronSensor,
	sensor_suspend:					Snr2_PowerDown,
	sensor_reset:					Snr2_Reset,

	read_write_brightness:			NTReadWriteBrightness,
	read_write_contrast:			NULL,
	read_write_sharpness:			NTReadWriteSharpness,
	read_write_white_balance:		NULL,
	read_write_noise_reduction:		NTReadWriteNoiseReduction,
	read_write_color_saturation:	NULL, 

	query_private_user_ctrl:		SensorUserPrivateCtrl,    /* OK */
	sensor_i2c_setRegAddr:			SensorI2cSetRegAddr, 	/* OK */
	sensor_set_ctrl:				SensorI2cWriteCtrl,
	sensor_get_ctrl:				SensorI2cReadCtrl,

	change_image_resolution: 		NTChangeImageResolution,
	set_flicker_freq:				NULL, //NTSetFlickerFreq,
	low_lux_detect:					NULL, //NTLowLuxDetect,
	control_IR_led:					NULL, //NTIRLedOnOff,
#if 0
	u16MaxImgHeight:				720,		 
	u16MaxImgWidth: 				1280,
#else
	u16MaxImgHeight:				CONFIG_VIN_DEV2_ENCODE_HEIGHT,		 
	u16MaxImgWidth: 				CONFIG_VIN_DEV2_ENCODE_WIDTH,
#endif	
};

#ifdef CONFIG_W55FA92_VIDEOIN_DEV2
INT32 register_vin_port2_Sensor(NVT_SENSOR_T **sensor_intf)
{
	*sensor_intf = (NVT_SENSOR_T*)(&nvt_sensor_nt99142);
	return 0;
}
#endif 



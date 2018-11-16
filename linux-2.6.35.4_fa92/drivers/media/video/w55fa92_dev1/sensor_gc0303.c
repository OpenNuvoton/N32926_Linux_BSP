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


#define INIT_START_ROW  	27  	//(480-204*2),最大起始行//43  //13
#define INIT_START_COL  	56 		//(640-240*2),最大起始列 //56 //16

#define MAX_START_ROW  		72  	//(480-204*2),最大起始行
#define MAX_START_COL  		160 	//(640-240*2),最大起始列

#define DE_GAIN 			250

#define R_Device_Addr   	(0x00|0x80)
#define R_HBlankLine    	(0x01|0x80)
#define R_VBlankLine    	(0x02|0x80)
#define R_Expose_H			(0x03|0x80)
#define R_Expose_L			(0x04|0x80)
#define R_GlobalBKComp 		(0x05|0x80)
#define R_GRedBKComp 		(0x06|0x80)
#define R_GGreenBKComp 		(0x07|0x80)
#define R_GBlueBKComp 		(0x08|0x80)
#define R_BlueBKComp 		(0x09|0x80)
#define R_GRedGC_H	 		(0x0A|0x80)
#define R_GRedGC_L	 		(0x0B|0x80)
#define R_RedGC_H	 		(0x0C|0x80)
#define R_RedGC_L	 		(0x0D|0x80)
#define R_GreenGC_H	 		(0x0E|0x80)
#define R_GreenGC_L	 		(0x0F|0x80)
#define R_BlueGC_H	 		(0x10|0x80)
#define R_BlueGC_L	 		(0x11|0x80)
#define R_Row_H	 			(0x12|0x80)
#define R_Row_L	 			(0x13|0x80)
#define R_Col_H	 			(0x14|0x80)
#define R_Col_L	 			(0x15|0x80)
#define R_WinH_H	 		(0x16|0x80)
#define R_WinH_L	 		(0x17|0x80)
#define R_WinW_H	 		(0x18|0x80)
#define R_WinW_L	 		(0x19|0x80)
#define R_Gamma        		(0x1A|0x80)
#define R_FrameCtrl     	(0x1F|0x80)

#define WIN_HEIGHT_U            1		//408
#define WIN_HEIGHT_L            0x9A	//0x98
#define WIN_WIDTH_U             1		//490
#define WIN_WIDTH_L             0xEA//0xE0//

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


static struct OV_RegValue g_sGC0303_RegValue[] = 
{
//	#include "GC0303/GC0303_VGA.dat"
    {R_HBlankLine,0x18}, {R_VBlankLine,0x22}, {R_Expose_H,0x00},
    {R_Expose_L,0x80},{R_GlobalBKComp,0xC0},{R_FrameCtrl,0x9D}, //{R_FrameCtrl,0x85},
    {R_WinH_H,WIN_HEIGHT_U},{R_WinH_L,WIN_HEIGHT_L},{R_WinW_H,WIN_WIDTH_U},
    {R_WinW_L,WIN_WIDTH_L},{R_GRedGC_H,0},{R_GRedGC_L,0},
	{R_GreenGC_H,0},{R_GreenGC_L,0},{R_RedGC_H,0},
	{R_RedGC_L,0},{R_BlueGC_H,(DE_GAIN>>8)&0xFF},{R_BlueGC_L,DE_GAIN&0xFF},
	{R_Row_L,INIT_START_ROW},{R_Col_L,INIT_START_COL},{0x9e,0x08}
}; 

static struct OV_RegTable g_OV_InitTable[] =
{
	{g_sGC0303_RegValue,_REG_TABLE_SIZE(g_sGC0303_RegValue)},		
};

static __u8 g_uOvDeviceID[]= 
{
	0x30
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
	{ "gc0303", 0 },
	{"",0},
};
MODULE_DEVICE_TABLE(i2c, sensor_id);

static struct i2c_driver sensor_i2c_driver = {
	.driver = {
		.name = "gc0303",
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
	return ret;
#else
	I2C_Write_8bitSlaveAddr_8bitReg_8bitData(uAddr, uRegAddr, uData);
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
	//outp32(REG_GPBFUN1, inp32(REG_GPBFUN1)& ~(MF_GPB14|MF_GPB13));
	//DrvI2C_Open(eDRVGPIO_GPIOB, 					
	//			eDRVGPIO_PIN13, 
	//			eDRVGPIO_GPIOB,
	//			eDRVGPIO_PIN14, 
	//			(PFN_DRVI2C_TIMEDELY)I2C_Delay);
	outp32(REG_GPAFUN0, inp32(REG_GPAFUN0)& ~(MF_GPA0));	
	outp32(REG_GPBFUN1, inp32(REG_GPBFUN1)& ~(MF_GPB14));
	DrvI2C_Open(eDRVGPIO_GPIOB,
			eDRVGPIO_PIN14,  
			eDRVGPIO_GPIOA, 					
			eDRVGPIO_PIN0, 
			(PFN_DRVI2C_TIMEDELY)I2C_Delay);
	
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
		printk("psRegValue->uRegAddr is %x\n",psRegValue->uRegAddr);
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
	int ret = 0;
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

	printk("Init GC0303 \n"); 		
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
#if 0
	schedule_mdelay(10);
	i2c_init();
	schedule_mdelay(10);
#else
	mdelay(10);
	i2c_init();
	mdelay(10);
#endif
	vin_priv->pDevVin->Open(64000, 24000);
	
	SDBG("Init GC0303 \n"); 	
	res = OvRegConfig(0);
	if( res<0 ){
		printk("Sensor initial fail \n");
		return res;	
	}
	else
		printk("Sensor initial successful \n");
	vin_priv->sensor_intf->u8SensorDevID = g_uOvDeviceID[0];//g_uOvDeviceID[pDevVin->ov7725];
	vin_priv->sensor_intf->u16CurImgHeight = vin_priv->sensor_intf->u16MaxImgHeight;
	vin_priv->sensor_intf->u16CurImgWidth = vin_priv->sensor_intf->u16MaxImgWidth;
#ifdef CONFIG_SHARE_SENSOR											
	SensorOutputResolution(vin_priv->sensor_intf->u16CurImgWidth, vin_priv->sensor_intf->u16CurImgHeight);				
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

	if(vin_priv->sensor_intf->u16CurImgWidth==640){
		vin_priv->videocropcap.pixelaspect.numerator = 18;	/* Suppose current image size HD */
		vin_priv->videocropcap.pixelaspect.denominator = 32;	/* Zoomming step */
	}else {
		vin_priv->videocropcap.pixelaspect.numerator = 12;	/* Suppose current image size VGA/SVGA */
		vin_priv->videocropcap.pixelaspect.denominator = 16;	/* Zoomming step */
	}


	vin_priv->pDevVin->Open(72000, 24000);					

	vin_priv->pDevVin->SetBaseStartAddress(eVIDEOIN_PACKET, 0, vin_priv->videoIn_preview_buf[0].u32PhysAddr);

	vin_priv->pDevVin->EnableInt(eVIDEOIN_VINT);
	
	vin_priv->pDevVin->SetSensorPolarity(FALSE, 
								TRUE, 
								TRUE);
	printk("Sensor SetSensorPolarity 011\n");
/*

	vin_priv->pDevVin->SetSensorPolarity(FALSE, 
								FALSE, 
								TRUE);
	printk("Sensor SetSensorPolarity 001\n");

	vin_priv->pDevVin->SetSensorPolarity(FALSE, 
								FALSE, 
								FALSE);
	printk("Sensor SetSensorPolarity 000\n");

	vin_priv->pDevVin->SetSensorPolarity(FALSE, 
								TRUE, 
								FALSE);
	printk("Sensor SetSensorPolarity 010\n");


	vin_priv->pDevVin->SetSensorPolarity(TRUE, 
								TRUE, 
								TRUE);
	printk("Sensor SetSensorPolarity 111\n");

	vin_priv->pDevVin->SetSensorPolarity(TRUE, 
								FALSE, 
								TRUE);
	printk("Sensor SetSensorPolarity 101\n");

	vin_priv->pDevVin->SetSensorPolarity(TRUE, 
								TRUE, 
								FALSE);
	printk("Sensor SetSensorPolarity 110\n");

	vin_priv->pDevVin->SetSensorPolarity(TRUE, 
								FALSE, 
								FALSE);
	printk("Sensor SetSensorPolarity 100\n");
*/
/*
*/
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
	u16MaxImgHeight:				720,		 
	u16MaxImgWidth: 				1280,
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


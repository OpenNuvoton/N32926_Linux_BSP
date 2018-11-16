/* 
 * w55fa92adc.c   w55fa92adc Driver based on wm9878.c
 * Copyright (c) 2010 Nuvoton technology corporation
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * Changelog:
 */
/*
 * wm8978.c  --  WM8978 ALSA SoC Audio Codec driver
 *
 * Copyright (C) 2009-2010 Guennadi Liakhovetski <g.liakhovetski@gmx.de>
 * Copyright (C) 2007 Carlos Munoz <carlos@kenati.com>
 * Copyright 2006-2009 Wolfson Microelectronics PLC.
 * Based on wm8974 and wm8990 by Liam Girdwood <lrg@slimlogic.co.uk>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/clk.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h> 
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/tlv.h>
#include <asm/div64.h>
#include <asm/io.h>
#include <mach/w55fa92_reg.h>
#include "w55fa92adc.h"

#define __TEST__
//#define ERR1
 
//#define DBG
#ifdef DBG
#define ENTER()		printk("ENTER : %s  : %s\n",__FILE__, __FUNCTION__)
#define LEAVE()		printk("LEAVE : %s  : %s\n",__FILE__, __FUNCTION__)
#define ERRLEAVE()	printk("ERRLEAVE : %s  : %s\n",__FILE__, __FUNCTION__)
#define PUSH()		printk("                                                                                        \n")	

#define SDBG		printk
#else
#define ENTER(...)
#define LEAVE(...)	
#define ERRLEAVE(...)	
#define SDBG(...)
#endif
static spinlock_t adc_i2c_lock;
static struct snd_soc_codec *w55fa92adc_codec;


/* w55fa92adc register cache. Note that register 0 is not included in the cache. */
#if 0
static const u16 w55fa92adc_reg[w55fa92adc_CACHEREGNUM] = {
        0x0000, 0x0000, 0x0000, 0x0000,	/* 0x00...0x03 */
        0x0050, 0x0000, 0x0140, 0x0000,	/* 0x04...0x07 */
        0x0000, 0x0000, 0x0000, 0x00ff,	/* 0x08...0x0b */
        0x00ff, 0x0000, 0x0100, 0x00ff,	/* 0x0c...0x0f */
        0x00ff, 0x0000, 0x012c, 0x002c,	/* 0x10...0x13 */
        0x002c, 0x002c, 0x002c, 0x0000,	/* 0x14...0x17 */
        0x0032, 0x0000, 0x0000, 0x0000,	/* 0x18...0x1b */
        0x0000, 0x0000, 0x0000, 0x0000,	/* 0x1c...0x1f */
        0x0038, 0x000b, 0x0032, 0x0000,	/* 0x20...0x23 */
        0x0008, 0x000c, 0x0093, 0x00e9,	/* 0x24...0x27 */
        0x0000, 0x0000, 0x0000, 0x0000,	/* 0x28...0x2b */
        0x0033, 0x0010, 0x0010, 0x0100,	/* 0x2c...0x2f */
        0x0100, 0x0002, 0x0001, 0x0001,	/* 0x30...0x33 */
        0x0039, 0x0039, 0x0039, 0x0039,	/* 0x34...0x37 */
        0x0001,	0x0001,			/* 0x38...0x3b */
};
#endif 
/* codec private data */
#define W55FA92_ADC_CACHEREGNUM	2
#define W55FA92_ADC_FIRSTREG	0x01
#define W55FA92_ADC_LASTREG		0x02
struct w55fa92adc_priv {
        struct snd_soc_codec codec; 
		 u16 reg_cache[W55FA92_ADC_CACHEREGNUM];
};

struct w55fa92adc_priv *w55fa92adc;

static const u16 w55fa92_adc_reg[W55FA92_ADC_CACHEREGNUM] = {
        0x1E, 0x0e	/* 0x00...0x01 */		// default value of pseudo registers for w55fa92 adc
};
/****************************************************************************************************
 * w55fa92 driver layer
 ****************************************************************************************************/

#define ERRCODE		unsigned int
#define UINT8		unsigned char
#define UINT16		unsigned short int
#define UINT32		unsigned int
#define INT8		char
#define INT16		short int
#define INT32		int	
#define VOID		void
#define BOOL	 	unsigned int
#define FALSE		0
#define TRUE		1
#define PBOOL		BOOL*
#define PUINT32		UINT32*
#define PUINT16		UINT16*
#define PUINT8		UINT8*
#define PINT32		INT32*
#define PINT16		INT16*
#define PINT8		INT8*

#define Successful	0	
#define outp32(addr, value)		writel(value, addr)
#define inp32(addr)				readl(addr)


#define AR_ERROR_CODE					0xB800E000
#define E_AR_BUSY  						(0xB800E000 | 0x01)

typedef void (*PFN_AUR_CALLBACK)(void);


typedef enum{
	eAUR_MODE_0 = 0,
	eAUR_MODE_1,
	eAUR_MODE_2,
	eAUR_MODE_3
}E_AUR_MODE;

typedef enum{ 
	eAUR_OTL_N3 = 0,                                             
	eAUR_OTL_N4P6,
	eAUR_OTL_N6P2,
	eAUR_OTL_N7P8,
	eAUR_OTL_N9P4,
	eAUR_OTL_N11,	
	eAUR_OTL_N12P6,
	eAUR_OTL_N14P2,
	eAUR_OTL_N15P8,
	eAUR_OTL_N17P4,
	eAUR_OTL_N19,
	eAUR_OTL_N20P6,
	eAUR_OTL_N22P2,
	eAUR_OTL_N23P8,
	eAUR_OTL_N25P4,
	eAUR_OTL_N27	
}E_AUR_AGC;

typedef enum{ 
	eAUR_CLAMP_GAIN_0 = 0,         
	eAUR_CLAMP_GAIN_P1P6,                                             
	eAUR_CLAMP_GAIN_P3P2,
	eAUR_CLAMP_GAIN_P4P8,
	eAUR_CLAMP_GAIN_P6P4,
	eAUR_CLAMP_GAIN_P8,
	eAUR_CLAMP_GAIN_P9P6,
	eAUR_CLAMP_GAIN_P11P2,
	eAUR_CLAMP_GAIN_P12P8,
	eAUR_CLAMP_GAIN_P14P4,
	eAUR_CLAMP_GAIN_P16,
	eAUR_CLAMP_GAIN_P17P6,
	eAUR_CLAMP_GAIN_P19P2,
	eAUR_CLAMP_GAIN_P20P8,
	eAUR_CLAMP_GAIN_P22P4	
}E_AUR_MAX_GAIN, E_AUR_MIN_GAIN;

typedef enum
{
	eAUR_SPS_48000 = 48000,
	eAUR_SPS_44100 = 44100,
	eAUR_SPS_32000 = 32000,	
	eAUR_SPS_24000 = 24000,
	eAUR_SPS_22050 = 22050,
	eAUR_SPS_16000 = 16000,
	eAUR_SPS_12000 = 12000,
	eAUR_SPS_11025 = 11025,
	eAUR_SPS_8000 = 8000,
	eAUR_SPS_96000 = 96000,
	eAUR_SPS_192000 = 192000		
}E_AUR_SPS;

typedef enum
{
	eAUR_ORDER_MONO_32BITS =0,
	eAUR_ORDER_MONO_16BITS,
	eAUR_ORDER_STEREO_16BITS,
	eAUR_ORDER_MONO_24BITS
}E_AUR_ORDER;

typedef enum
{
	eAUR_DIGI_MIC_GAIN_P0 =0,
	eAUR_DIGI_MIC_GAIN_P1P6,
	eAUR_DIGI_MIC_GAIN_P3P2,
	eAUR_DIGI_MIC_GAIN_P4P8,
	eAUR_DIGI_MIC_GAIN_P6P4,
	eAUR_DIGI_MIC_GAIN_P8,
	eAUR_DIGI_MIC_GAIN_P9P6,
	eAUR_DIGI_MIC_GAIN_P11P2,
	eAUR_DIGI_MIC_GAIN_P12P8,
	eAUR_DIGI_MIC_GAIN_P14P4,
	eAUR_DIGI_MIC_GAIN_P16,
	eAUR_DIGI_MIC_GAIN_P17P6,
	eAUR_DIGI_MIC_GAIN_P19P2,
	eAUR_DIGI_MIC_GAIN_P20P8,
	eAUR_DIGI_MIC_GAIN_P22P4,
	eAUR_DIGI_MIC_GAIN_P24
}E_AUR_DIGI_MIC_GAIN;

typedef enum
{
	eAUR_MONO_LINE_IN = 0,
	eAUR_MONO_MIC_IN,
	eAUR_MONO_DIGITAL_MIC_IN,
	eAUR_STEREO_DIGITAL_MIC_IN,
	
	eAUR_STEREO_LINE_IN	/* Test Only */	
}E_AUR_MIC_SEL;

typedef enum
{
	E_AGC_LEVEL = 0	
}E_AUR_AGC_LEVEL;	

INT32 DrvAUR_AutoClampingGain(UINT32 u32MaxGain, UINT32 u32MinGain);
INT32 DrvAUR_AutoGainTiming(UINT32 u32Attack, UINT32 u32Recovery, UINT32 u32Hold);
INT32 DrvAUR_NoiseGatCtrl(BOOL bIsEnable, UINT32 u32Gain, UINT32 u32Level);
INT32 DrvAUR_NoiseGateTiming(UINT32 u32DelayTime, UINT32 u32InTime, UINT32 u32OutTime);

extern INT32 DrvAUR_AudioI2cRead(UINT32 u32Addr, UINT8* p8Data);
extern INT32 DrvAUR_AudioI2cWrite(UINT32 u32Addr, UINT32 u32Data);

INT32 DrvAUR_SetSampleRate(E_AUR_SPS eSampleRate);
INT32 DrvAUR_AutoGainCtrl(BOOL bIsEnable, BOOL bIsChangeStep, E_AUR_AGC_LEVEL eLevel);	
VOID DrvAUR_StartRecord(E_AUR_MODE eMode);
VOID DrvAUR_StopRecord(VOID);
INT32 DrvAUR_Open(E_AUR_MIC_SEL eMIC, BOOL bIsCoworkEDMA);
void DrvAUR_EnableInt(void);
void DrvAUR_DisableInt(void);
INT32 DrvAUR_InstallCallback(PFN_AUR_CALLBACK pfnCallback, PFN_AUR_CALLBACK* pfnOldCallback);
VOID DrvAUR_SetDataOrder(E_AUR_ORDER eOrder);
INT32 DrvAUR_Close(void);

#define REAL_CHIP

#define 	REG_ADC_H20		0x20
	#define 	HPF_EN			BIT3
	#define 	STEREO_ADC		BIT2
	#define 	OSR				BIT1
	#define 	ADCEN			BIT0
#define 	REG_ADC_H21		0x21
	#define 	PDBIAS_L			BIT6
	#define 	PDPGAL_L			BIT5
	#define 	PDPGAR_L			BIT4
	#define 	PDL_L			BIT3
	#define 	PDR_L			BIT2
	#define 	ADC_EN_SEL		NVTBIT(1, 0)
	
#define 	REG_ADC_H22		0x22
	#define 	ADC_VOL_BOOST	BIT4
	#define 	ADC_VOLL		NVTBIT(3, 0)
	
#define 	REG_ADC_H23		0x23
	#define 	ADC_VOL_OD		BIT4
	#define 	ADC_VOLR		NVTBIT(3, 0)	
	
#define 	REG_ADC_H24		0x24
	#define 	RESADJ			NVTBIT(2, 0)

#define 	REG_ADC_H25		0x25
	#define 	BYPASSPLL		BIT7
	#define 	PDPLL			BIT6
	#define 	SRS				NVTBIT(4, 0)

#define 	REG_ADC_H26		0x26
	#define 	ADCSWRESET		BIT1
	
#define 	REG_ADC_H29		0x29
	#define 	MICIN_SEL		BIT7
	


INT32 DrvAUR_AutoClampingGain(UINT32 u32MaxGain, UINT32 u32MinGain)
{
	outp32(REG_AR_AGC1, (inp32(REG_AR_AGC1) & ~(MAXGAIN|MINGAIN))  | 
						(((u32MaxGain<<20)&MAXGAIN) | ((u32MinGain<<16)&MINGAIN)) );
	return Successful;
}
INT32 DrvAUR_AutoGainTiming(UINT32 u32Attack, UINT32 u32Recovery, UINT32 u32Hold)
{
	outp32(REG_AR_AGC1, (inp32(REG_AR_AGC1) & ~RECOVERY)  | 
						(u32Recovery& RECOVERY) );
						
	outp32(REG_AR_AGC2, (inp32(REG_AR_AGC2) & ~(ATTACK | HOLD))  | 
						(((u32Attack<<16) & ATTACK) |  (u32Hold & HOLD)) );	
	return Successful;
}
INT32 DrvAUR_NoiseGatCtrl(BOOL bIsEnable, UINT32 u32Gain, UINT32 u32Level)
{
	outp32(REG_AR_NG, (inp32(REG_AR_NG) & ~(NG_GAIN | NG_LEVEL))  | 
						( ((u32Gain<<8)&NG_GAIN) | (u32Level&NG_LEVEL)) );
	outp32(REG_AR_NG, (inp32(REG_AR_NG) & ~NG_EN) | 	
						((bIsEnable<<31)&NG_EN) );			
	return Successful;
}
INT32 DrvAUR_NoiseGateTiming(UINT32 u32DelayTime, UINT32 u32InTime, UINT32 u32OutTime)
{
	outp32(REG_AR_NG, (inp32(REG_AR_NG) & ~(IN_NG_TIME | OUT_NG_TIME))  | 
						( ((u32InTime<<20)&IN_NG_TIME) | ((u32OutTime<<16) &OUT_NG_TIME)) );
	outp32(REG_AR_NG, (inp32(REG_AR_NG) & ~DLYTIME) | 	
						((u32DelayTime<<24)&DLYTIME) );	
	return Successful;
}

INT32 DrvAUR_AudioI2cRead(UINT32 u32Addr, UINT8* p8Data)
{
	UINT32 u32AGCEnable = FALSE;
	UINT32 u32Ctrl = 0;	
	if(inp32(REG_AR_AGC1)&AGC_EN){	
		u32AGCEnable = TRUE;
		outp32(REG_AR_AGC1, (inp32(REG_AR_AGC1) & ~AGC_EN)); /* Disable AGC */
	}
	/* if i2C is working, the i2c keep working, so program still need to check the busy bit */
	spin_lock(&adc_i2c_lock);
	do
	{		
	
	}while((inp32(REG_SDADC_CTL)&AR_BUSY) != 0);		
	u32Addr = u32Addr & 0xFF;
	u32Ctrl = (u32Ctrl | (0x800000 | (u32Addr<<8)));
	u32Ctrl = u32Ctrl | 0x20000000;
	outp32(REG_SDADC_CTL, u32Ctrl);
	do
	{		
	}while((inp32(REG_SDADC_CTL)&AR_BUSY) != 0);
	*p8Data = inp32(REG_SDADC_CTL)&0xFF;
	spin_unlock(&adc_i2c_lock);
	if(u32AGCEnable == TRUE)	
		outp32(REG_AR_AGC1, (inp32(REG_AR_AGC1) | AGC_EN)); /* Enable AGC */	
	return Successful;
}
/*
	u32Data: 0 ~ 15, 0 	means mute. 			==> Write "15" to ADC 
					 1 	means 0db, 				==> write "0" to ADC
					 14 means upscale 20.8db. 	==> write "13" to ADC
					 15 meams upscale 22.4db. 	==> write "14" to ADC
*/
INT32 DrvAUR_AudioI2cWrite(UINT32 u32Addr, UINT32 u32Data)
{
	UINT32 u32AGCEnable = FALSE;
	UINT32 u32Ctrl = 0;
	if(inp32(REG_AR_AGC1)&AGC_EN){	
		u32AGCEnable = TRUE;
		outp32(REG_AR_AGC1, (inp32(REG_AR_AGC1) & ~AGC_EN)); /* Disable AGC */
	}

	/* if i2C keeps working, so program still need to check the busy bit */
	spin_lock(&adc_i2c_lock);	
	do
	{		
	}while((inp32(REG_SDADC_CTL)&AR_BUSY) != 0);		 
/*
	if(u32Addr == REG_ADC_H22){//Left
		SDBG("Want REG_ADC_H22 = %x\n", u32Data);
		if(u32Data&BIT4)
			outp32(REG_SDADC_AGBIT, inp32(REG_SDADC_AGBIT) | 0x10);
		else
			outp32(REG_SDADC_AGBIT, inp32(REG_SDADC_AGBIT) & ~0x10);		
	}
	if(u32Addr==REG_ADC_H23){//Right
		SDBG("Want REG_ADC_H23 = %x\n", u32Data);
		if(u32Data&BIT4)
			outp32(REG_SDADC_AGBIT, inp32(REG_SDADC_AGBIT) | 0x1);
		else
			outp32(REG_SDADC_AGBIT, inp32(REG_SDADC_AGBIT) & ~0x1);	
	}
*/

	
	if( u32Addr == REG_ADC_H22 ){//Left //Need to keep pre-gain if H22
		if( (u32Data&0x0F) ==0 )
			u32Data = u32Data | 0xF; //Mute
		else
			u32Data = (0x10) | ((u32Data&0x0F) -1);

		SDBG("Write H22 %d\n", u32Data);
	}
	if(u32Addr==REG_ADC_H23){//Right
		if(u32Data&BIT4)
			outp32(REG_SDADC_AGBIT, inp32(REG_SDADC_AGBIT) | 0x1);
		else
			outp32(REG_SDADC_AGBIT, inp32(REG_SDADC_AGBIT) & ~0x1);	
	}	
	u32Addr = u32Addr & 0xFF;
	u32Data = u32Data & 0xFF;

	u32Ctrl = u32Ctrl | (0x810000 | (u32Addr<<8)  | u32Data) ;
	u32Ctrl = u32Ctrl | 0x20000000;
	outp32(REG_SDADC_CTL, u32Ctrl);		
	
	do
	{		
	}while((inp32(REG_SDADC_CTL)&AR_BUSY) != 0);
	spin_unlock(&adc_i2c_lock);

	//Update the cache 
	if(u32Addr == REG_ADC_H22)
		w55fa92adc->reg_cache[0] = u32Data;
	else if(u32Addr == REG_ADC_H23)
		w55fa92adc->reg_cache[1] = u32Data;

	if(u32AGCEnable == TRUE)
		outp32(REG_AR_AGC1, (inp32(REG_AR_AGC1) | AGC_EN)); /* Enable AGC */	
	return Successful;
}

VOID DrvAUR_StartRecord(E_AUR_MODE eMode)
{
	UINT8 u8Data;
	UINT32 u32Reg = inp32(REG_AR_CON) & ~INT_MOD; 
	u32Reg = u32Reg | ((eMode <<2)&INT_MOD);
	outp32(REG_AR_CON, u32Reg);
	outp32(REG_AR_CON, inp32(REG_AR_CON)|AR_EDMA);
	
	DrvAUR_AudioI2cRead(REG_ADC_H20, &u8Data); 
	u8Data = u8Data | ADCEN;
	DrvAUR_AudioI2cWrite(REG_ADC_H20, u8Data);	
}
VOID DrvAUR_StopRecord(void)
{
	UINT8 u8Data;
	outp32(REG_AR_CON , inp32(REG_AR_CON)& ~AR_EDMA);
	DrvAUR_AudioI2cRead(REG_ADC_H20, &u8Data); 
	u8Data = u8Data & ~ADCEN;
	DrvAUR_AudioI2cWrite(REG_ADC_H20, u8Data);
}

INT32 DrvAUR_SetSampleRate(E_AUR_SPS eSampleRate)
{
	UINT8 u8I2cData;
	INT32 i32ErrCode = 0;
	/* Set Sample Rate */
	do
	{
		i32ErrCode = DrvAUR_AudioI2cRead(REG_ADC_H25, &u8I2cData);
	}while(i32ErrCode != Successful);
#if 0
	sysDelay(1);
#endif
	u8I2cData = u8I2cData &  (~SRS);
	
	switch(eSampleRate)
	{
		case eAUR_SPS_48000:
			u8I2cData = u8I2cData | 0x0;
			break;
		case eAUR_SPS_44100:
			u8I2cData = u8I2cData | 0x8;
			break;
		case eAUR_SPS_32000:
			u8I2cData = u8I2cData | 0x10;
			break;	
		case eAUR_SPS_24000:
			u8I2cData = u8I2cData | 0x01;
			break;
		case eAUR_SPS_22050:
			u8I2cData = u8I2cData | 0x09;
			break;	
		case eAUR_SPS_16000:
			u8I2cData = u8I2cData | 0x11;
			break;
		case eAUR_SPS_12000:
			u8I2cData = u8I2cData | 0x02;
			break;
		case eAUR_SPS_11025:
			u8I2cData = u8I2cData | 0x0A;
			break;
		case eAUR_SPS_8000:
			u8I2cData = u8I2cData | 0x12;
			break;
		case eAUR_SPS_96000:
			u8I2cData = u8I2cData | 0x04;
			break;
		case eAUR_SPS_192000:
			u8I2cData = u8I2cData | 0x04;
			break;
	}

	do
	{
		i32ErrCode = DrvAUR_AudioI2cWrite(REG_ADC_H25, u8I2cData);
	}while(i32ErrCode != Successful);
	
	/* Set OSR */
	do
	{
		DrvAUR_AudioI2cRead(REG_ADC_H20, &u8I2cData);
	}while(i32ErrCode != Successful);
	SDBG("Readback H20 = 0x%x\n", u8I2cData);
	if(eSampleRate==eAUR_SPS_192000)
	{
		u8I2cData = u8I2cData | OSR;
		SDBG("Will Write H20 = 0x%x\n", u8I2cData);
		do
		{
			i32ErrCode = DrvAUR_AudioI2cWrite(REG_ADC_H20, u8I2cData);
		}while(i32ErrCode != Successful);
		
		DrvAUR_AudioI2cRead(REG_ADC_H20, &u8I2cData);
		SDBG("H20 = 0x%x\n", u8I2cData);
		
	}
	else
	{
		u8I2cData = u8I2cData & (~OSR);
		SDBG("Will Write H20 = 0x%x\n", u8I2cData);
		do
		{
			i32ErrCode = DrvAUR_AudioI2cWrite(REG_ADC_H20, u8I2cData);
		}while(i32ErrCode != Successful);	
	}	
	return Successful;
}
VOID DrvAUR_SetDataOrder(E_AUR_ORDER eOrder)
{
	UINT32 u32RegData = (inp32(REG_AR_DIGIM) & ~SMPL_MODE) |  (eOrder<<8);
	outp32(REG_AR_DIGIM, u32RegData);
}

VOID DrvAUR_SetDigiMicGain(BOOL bIsEnable, E_AUR_DIGI_MIC_GAIN eDigiGain)
{//Digital MIC only
	UINT32 u32RegData = inp32(REG_AR_DIGIM) & ~ (DIGIM_EN|DIGIM_LV);
	outp32(REG_AR_DIGIM, u32RegData | (((bIsEnable<<7) |DIGIM_EN) | (eDigiGain&DIGIM_LV)) );
}
INT32 DrvAUR_AutoGainCtrl(BOOL bIsEnable, BOOL bIsChangeStep, E_AUR_AGC_LEVEL eLevel)
{
	outp32(REG_AR_AGC1, (inp32(REG_AR_AGC1) & ~(GSTEP | OTL)) | 
						(((bIsChangeStep<<28)&GSTEP) | ((eLevel<<24) & OTL)) );	
	outp32(REG_AR_AGC1, (inp32(REG_AR_AGC1) & ~AGC_EN) | 	
						((bIsEnable << 31) & AGC_EN)	);						
	return Successful;
}

static PFN_AUR_CALLBACK g_psADCCallBack;

INT32 
DrvAUR_InstallCallback(	
	PFN_AUR_CALLBACK pfnCallback,
	PFN_AUR_CALLBACK* pfnOldCallback
	)
{
 
	*pfnOldCallback = g_psADCCallBack;
    	g_psADCCallBack = pfnCallback; 	    		
    	return Successful;
}

void 
AurIntHandler(void)
{
	if(g_psADCCallBack !=0)
	        g_psADCCallBack();  
	outp32(REG_AR_CON, (inp32(REG_AR_CON) | AR_INT));	/* Write one clear */
}

struct clk *clk;
INT32 DrvAUR_Open(E_AUR_MIC_SEL eMIC, BOOL bIsCoworkEDMA)
{
	
	UINT8 u8RegDac;

	ENTER();
	spin_lock_init(&adc_i2c_lock);

	//outp32(REG_APBCLK, inp32(REG_APBCLK) | ADC_CKE);
	clk = clk_get(NULL, "adc");
	clk_enable(clk);

#ifdef  REAL_CHIP
	outp32(REG_CLKDIV3, (inp32(REG_CLKDIV3) & ~(ADC_N1 | ADC_S| ADC_N0)) );				/* Fed to ADC clock need 12MHz=External clock */
#else	
	outp32(REG_CLKDIV3, (inp32(REG_CLKDIV3) & ~(ADC_N1 | ADC_S| ADC_N0)) | (1<<24) );		/* FPGA: Divider need to be 1 at least for I2C clock*/
#endif 
		
	outp32(REG_APBIPRST, ADCRST);
	outp32(REG_APBIPRST, 0);
	
	outp32(REG_AR_CON, inp32(REG_AR_CON) & ~AR_RST);	/* 0: reset mode */
	outp32(REG_AR_CON, inp32(REG_AR_CON) | AR_RST);		/* 1: Normal mode */
	
	
	//outp32(REG_SDADC_CTL, ((inp32(REG_SDADC_CTL) & ~SCK_DIV) | (0x3F<<24)) );
	DrvAUR_AudioI2cWrite(REG_ADC_H25, 0x00);
	DrvAUR_AudioI2cWrite(REG_ADC_H26, 0x02); 
	DrvAUR_AudioI2cWrite(REG_ADC_H26, 0x00); /* Reset AD Filter and I2S parts except I2C block */
	mdelay(1);
	DrvAUR_AudioI2cWrite(REG_ADC_H26, 0x02);
	
	DrvAUR_AudioI2cRead(REG_ADC_H20, &u8RegDac);
	if ((eMIC == eAUR_STEREO_DIGITAL_MIC_IN) || (eMIC == eAUR_STEREO_LINE_IN))
		u8RegDac = u8RegDac | (HPF_EN  | ADCEN | STEREO_ADC);
	else
		u8RegDac = (u8RegDac | (HPF_EN  | ADCEN)) & ~STEREO_ADC;
	SDBG("Reg 20 will write value = 0x%x\n", u8RegDac);	
	DrvAUR_AudioI2cWrite(REG_ADC_H20, u8RegDac);	
	//DrvAUR_AudioI2cWrite(REG_ADC_H29, 0x80);		/* Analog MIC */
	{
		UINT8 udata;	
		DrvAUR_AudioI2cRead(REG_ADC_H20, &udata);
		SDBG("read back Reg 20 bw written value = 0x%x\n\n\n", udata);	
	}
	if(bIsCoworkEDMA)
		outp32(REG_AR_CON , inp32(REG_AR_CON)| AR_EDMA);
	else
		outp32(REG_AR_CON , inp32(REG_AR_CON)& ~AR_EDMA);	
	
	switch(eMIC)
	{
		case eAUR_STEREO_LINE_IN:
		case eAUR_MONO_LINE_IN:
			DrvAUR_AudioI2cWrite(REG_ADC_H29, 0x0);		/* Analog Line in */
			u8RegDac = (UINT8) (~(PDBIAS_L | PDPGAL_L | PDPGAR_L | PDL_L | PDR_L | 3));	
			DrvAUR_AudioI2cWrite(REG_ADC_H21, u8RegDac);
			break;
		case eAUR_MONO_MIC_IN:
			DrvAUR_AudioI2cWrite(REG_ADC_H29, 0x0);		/* Analog MIC */
			u8RegDac = (UINT8)(~(PDBIAS_L | PDPGAL_L | PDPGAR_L | PDL_L | PDR_L | ADC_EN_SEL)  | 0x02);	
			DrvAUR_AudioI2cWrite(REG_ADC_H21, u8RegDac);
			break;			
		case eAUR_MONO_DIGITAL_MIC_IN:
			DrvAUR_AudioI2cWrite(REG_ADC_H29, 0x80);			/* Digital MIC */
			u8RegDac = (UINT8)(~(PDBIAS_L | PDPGAL_L | PDPGAR_L | PDL_L | PDR_L | ADC_EN_SEL)  | 0x02);	
			DrvAUR_AudioI2cWrite(REG_ADC_H21, u8RegDac);			
			break;			
		case 	eAUR_STEREO_DIGITAL_MIC_IN:
			DrvAUR_AudioI2cWrite(REG_ADC_H29, 0x80);			/* Digital MIC */
			u8RegDac = (UINT8)(~(PDBIAS_L | PDPGAL_L | PDPGAR_L | PDL_L | PDR_L | ADC_EN_SEL) | 0x02);	
			DrvAUR_AudioI2cWrite(REG_ADC_H21, u8RegDac);			
			break;	
	
				
	}	
#ifndef CONFIG_ARCH_W55FA92
	sysInstallISR(IRQ_LEVEL_1, 
					IRQ_AUDIO, 
					(PVOID)AurIntHandler);
	sysEnableInterrupt(IRQ_AUDIO);		
#endif
	LEAVE();
	return Successful;		
}
void DrvAUR_EnableInt(void)
{
	outp32(REG_AR_CON, (inp32(REG_AR_CON) | AR_INT_EN) );
}	
void DrvAUR_DisableInt(void)
{
	outp32(REG_AR_CON, (inp32(REG_AR_CON) & ~AR_INT_EN) );
}	

INT32 DrvAUR_Close(void)
{	
	UINT8 u8RegDac;
	/* Reset AD Filter and I2S parts except I2C block */
	DrvAUR_AudioI2cWrite(REG_ADC_H26, 0x02);
	DrvAUR_AudioI2cWrite(REG_ADC_H26, 0x00);	 
	DrvAUR_AudioI2cWrite(REG_ADC_H26, 0x02);
		
	/* Power down VBIAS, IBIAS, Left/Right Channel PGA, Left/Right Channel SDM  */
	u8RegDac = (UINT8)(PDBIAS_L | PDPGAL_L | PDPGAR_L | PDL_L | PDR_L );	
	DrvAUR_AudioI2cWrite(REG_ADC_H21, u8RegDac);
		
	//outp32(REG_AHBCLK, inp32(REG_AHBCLK) & ~ADC_CKE);
	//clk = clk_get(NULL, "adc");
	clk_disable(clk);
	clk_put(clk);
	return Successful;
}


void DrvADC_Mute(BOOL bIsMuteEnable)
{
#if 1
	
#else
	UINT8 u8CodecData; 
	static UINT8 u8LeftChannel =0, u8RightChannel =0; 
	static UINT8 u8MuteEnable = FALSE;
	if(bIsMuteEnable){
		SDBG("Enable mute\n");
		u8MuteEnable = TRUE;
		DrvAUR_AudioI2cRead(REG_ADC_H22, &u8LeftChannel);
		DrvAUR_AudioI2cRead(REG_ADC_H23, &u8RightChannel);		
		DrvAUR_AudioI2cRead(REG_ADC_H22, &u8CodecData);
		DrvAUR_AudioI2cWrite(REG_ADC_H22, (u8CodecData| 0xF)); /* Mute left channel, 0xF means mute */
		DrvAUR_AudioI2cRead(REG_ADC_H23, &u8CodecData);
		DrvAUR_AudioI2cWrite(REG_ADC_H23, (u8CodecData| 0xF)); /* Mute right channel, 0xF means mute */
	}else{
		SDBG("Disable mute\n");
		if(u8MuteEnable==TRUE)
		{
			SDBG("H22 reg data = 0x%x\n", u8LeftChannel);
			SDBG("H23 reg data = 0x%x\n", u8RightChannel);
			DrvAUR_AudioI2cWrite(REG_ADC_H22, u8LeftChannel); 	
			DrvAUR_AudioI2cWrite(REG_ADC_H23, u8RightChannel); 
			u8MuteEnable = FALSE;
		}	
	}	
#endif
}

/****************************************************************************************************
 * wrapper layer
 ****************************************************************************************************/
#ifdef CONFIG_TOUCHSCREEN_w55fa92
extern int w55fa92ts_open_again(void);
extern int w55fa92ts_close_again(void);
#endif
void adcStopRecord(void)
{
	UINT32 u32Idx;
	ENTER();

	DrvAUR_StopRecord();

	for(	u32Idx=0; u32Idx<0x60; u32Idx=u32Idx+0x4)
	{
		SDBG("0x%x = 0x%x\n", u32Idx, inp32(W55FA92_VA_ADC+u32Idx));
		u32Idx = u32Idx+0x04;
		SDBG("0x%x = 0x%x\n", u32Idx, inp32(W55FA92_VA_ADC+u32Idx));
		u32Idx = u32Idx+0x04;
		SDBG("0x%x = 0x%x\n", u32Idx, inp32(W55FA92_VA_ADC+u32Idx));
		u32Idx = u32Idx+0x04;
		SDBG("0x%x = 0x%x\n", u32Idx, inp32(W55FA92_VA_ADC+u32Idx));	
	}
	//2011-0506 w55fa92_adc_close();
	//2011-0506 w55fa92ts_open_again();
	LEAVE();
	
	return;
}
static void schedule_delay(uint32_t u32Delayms)
{
	volatile unsigned long j=0;	
	j = jiffies + u32Delayms*HZ/1000; 	/* u32Delayms~ u32Delayms+10ms */			
	while( time_before(jiffies,j) )
		schedule();	
}
int adcStartRecord(void)
{
	ENTER();
#if 0
	//g_u32Period = 4; 
	//adcSetRecordCallBackFunction(fnCallBack);
	DrvADC_EnableInt(eDRVADC_AUD_INT);		
	DrvADC_StartRecord(eDRVADC_RECORD_MODE_1);
#else
	//outp32(REG_AR_CON, inp32(REG_AR_CON)|AR_EDMA);
	DrvAUR_StartRecord(eAUR_MODE_1);
	SDBG("schedule_delay start\n");
	schedule_delay(500);	/* Schedule delay 500ms for bias stable */
	SDBG("schedule_delay end\n");
#endif
	LEAVE();
	
	return 0;
}
void w55fa92_adc_recording_setup(int nsampleRate)
{
	
	ENTER();
#if 0
	DrvADC_Open(eDRVADC_RECORD,			//Record mode
					eDRVSYS_APLL, 		//Source clock come from UPLL
					8);					//Deafult 8K sample rate. 
	//install_drvadc_calibration_callback(adc_calibration_callback);
	DrvADC_SetGainControl(eDRVADC_PRE_P14, 
							eDRVADC_POST_P0);  

	DrvADC_SetAutoGainTiming(4,		//Period
								4,		//Attack
								4,		//Recovery	
								4);		//Hold

	DrvADC_SetAutoGainControl(TRUE,
					    		//11, 			//Output target -12db
								//15, 			//Output target -6db
								//13, 			//Output target -9db
								12, 			//Output target -10.5db
					    		eDRVADC_BAND_P0P5,
					    		eDRVADC_BAND_N0P5);

	DrvADC_SetOffsetCancellation(FALSE,   	//BOOL bIsMuteEnable,
									FALSE, 	//BOOL bIsOffsetCalibration,
									FALSE, 	//BOOL bIsHardwareMode,
									0x10);	//UINT32 u32Offset

	DrvADC_SetOffsetCancellationEx(1,		//255 sample
									512);	//Delay sample count
    DrvADC_SetNoiseGate(FALSE, eDRVADC_NG_N48);

	outp32(REG_AUDIO_CON, inp32(REG_AUDIO_CON) & 
			~AUDIO_INT_MODE & ~AUDIO_INT_EN);	// one sample if finish    
	outp32(REG_AGCP1,inp32(REG_AGCP1) | 0x80000000);	// Enabe EDMA for ADC
#else
#ifdef CONFIG_ANALOG_MIC/* Both pregain and postgain are workable */
	DrvAUR_Open(eAUR_MONO_MIC_IN, FALSE);
#endif
#ifdef CONFIG_LINE_IN	/* pregain is useless. post gain is workable */
	DrvAUR_Open(eAUR_MONO_LINE_IN, FALSE);
#endif
#ifdef CONFIG_DIGITAL_MIC_MONO
	DrvAUR_Open(eAUR_MONO_DIGITAL_MIC_IN, FALSE);
#endif
#ifdef CONFIG_DIGITAL_MIC_STEREO
	DrvAUR_Open(eAUR_STEREO_DIGITAL_MIC_IN, FALSE);
#endif
#ifdef CONFIG_MIC_IN_DISABLE_GAIN
	DrvAUR_Open(eAUR_MONO_MIC_IN, FALSE);
#endif
#ifndef CONFIG_ARCH_W55FA92
	DrvAUR_InstallCallback(AudioRecordSampleDone, &pfnOldCallback);	
#endif

#ifdef CONFIG_MIC_IN_DISABLE_GAIN
	DrvAUR_AudioI2cWrite(REG_ADC_H22, 0x00);	/* Pregain + Postgain = 0db */
#else
	DrvAUR_AudioI2cWrite(REG_ADC_H22, 0x1E);	/* Pregain + Postgain = 20db + 22.4db */
#endif	
	DrvAUR_AudioI2cWrite(REG_ADC_H23, 0x0E);	/* Postgain */	

#if defined(CONFIG_DIGITAL_MIC_MONO)|| defined(CONFIG_DIGITAL_MIC_MONO)
	DrvAUR_SetDigiMicGain(TRUE, eAUR_DIGI_MIC_GAIN_P19P2);
#endif
	
	DrvAUR_DisableInt();
	DrvAUR_SetSampleRate(nsampleRate);
	DrvAUR_AutoGainTiming(0x20, 0x20, 0x20);
	DrvAUR_AutoGainCtrl(FALSE, FALSE, eAUR_OTL_N3);
	DrvAUR_SetDataOrder(eAUR_ORDER_MONO_16BITS);
#endif
}
INT32 adcInit(int nsampleRate)
{
	ENTER();
#if 0
	#ifdef CONFIG_TOUCHSCREEN_w55fa92
		w55fa92ts_close_again();
	#endif 
	SDBG("Close touch panel \n");
	w55fa92_adc_recording_setup(nsampleRate);
	adcStartRecord();
#else
	SDBG("Close touch panel \n");
	w55fa92_adc_recording_setup(nsampleRate);
	adcStartRecord();		
#endif
	LEAVE();
	return 0;	
}

#define W55FA92_ADC_RESET					0x00		
#define W55FA92_ADC_POWER_MANAGEMENT		0x01
#define W55FA92_ADC_LEFT_DIGITAL_VOLUME		0x02
#define W55FA92_ADC_RIGHT_DIGITAL_VOLUME	0x03
#if 0
static const DECLARE_TLV_DB_SCALE(digital_tlv, -2400, 160, 0);
											//	 | 	   |  |--> least value is not muted
											//   |	   |------> step size = 1.6dB (160*0.01)
											//   |-------------> least value = -24dB (-2400*0.01)
#else
static const DECLARE_TLV_DB_SCALE(digital_tlv, -4200, 160, 1);
											//	 | 	   |  |--> least value is muted
											//   |	   |------> step size = 1.6dB (160*0.01)
											//   |-------------> least value = -42dB (-4200*0.01)
#endif
/****************************************************************************************************
 * Alsa codec layer
 ****************************************************************************************************/
static const struct snd_kcontrol_new w55fa92adc_snd_controls[] = {
	/* Input PGA volume */
#if 1
	SOC_DOUBLE_R_TLV("PCM Volume",  //"Input PGA Volume",
		(REG_ADC_H22), 					/* Left channel address */
		(REG_ADC_H23),					/* Right channel address */
		0, 								/* min value */
		16, 							/* max value */
		0, 								/* Offset */
		digital_tlv),
#else
	SOC_DOUBLE_R_TLV("PCM Volume", W55FA92_ADC_LEFT_DIGITAL_VOLUME, W55FA92_ADC_RIGHT_DIGITAL_VOLUME,
        				0, 15, 1, digital_tlv),			/* set (24-0)/1.6+1 volume levels */
#endif
};

/* Mixer #1: Output (OUT1, OUT2) Mixer: mix AUX, Input mixer output and DAC */
static const struct snd_kcontrol_new w55fa92adc_left_out_mixer[] = {

};

static const struct snd_kcontrol_new w55fa92adc_right_out_mixer[] = {

};

/* OUT3/OUT4 Mixer not implemented */

/* Mixer #2: Input PGA Mute */
static const struct snd_kcontrol_new w55fa92adc_left_input_mixer[] = {      

};
static const struct snd_kcontrol_new w55fa92adc_right_input_mixer[] = {

};

static const struct snd_soc_dapm_widget w55fa92adc_dapm_widgets[] = {  
        SND_SOC_DAPM_ADC("Left ADC", "Power Down",
        W55FA92_ADC_POWER_MANAGEMENT, 1, 1),
        SND_SOC_DAPM_ADC("Right ADC", "Power Down",
        W55FA92_ADC_POWER_MANAGEMENT, 2, 1),
        
        SND_SOC_DAPM_OUTPUT("LHP"),
        SND_SOC_DAPM_OUTPUT("RHP"),
};

static const struct snd_soc_dapm_route audio_map[] = {  
	{"LHP", NULL, "Left ADC"},	// "destination <-- switch <-- source", define left DAC path
    //{"RHP", NULL, "Right ADC"},	// "destination <-- switch <-- source", define Right DAC path
};
#if 1
static int w55fa92adc_add_widgets(struct snd_soc_codec *codec)
{
	ENTER();
        snd_soc_dapm_new_controls(codec, w55fa92adc_dapm_widgets,
                                  ARRAY_SIZE(w55fa92adc_dapm_widgets));

        /* set up the w55fa92adc audio map */
        snd_soc_dapm_add_routes(codec, audio_map, ARRAY_SIZE(audio_map));
	LEAVE();
        return 0;
} 
#endif 

/*
 * Configure w55fa92adc clock dividers.
 */
static int w55fa92adc_set_dai_clkdiv(struct snd_soc_dai *codec_dai,
                                 int div_id, int div)
{
	ENTER();
	LEAVE();
    return 0;
}

/*
 * @freq:	when .set_pll() us not used, freq is codec MCLK input frequency
 */
static int w55fa92adc_set_dai_sysclk(struct snd_soc_dai *codec_dai, int clk_id,
                                 unsigned int freq, int dir)
{
	ENTER();
	LEAVE();
	return 0;
}

/*
 * Set ADC and Voice DAC format.
 */
static int w55fa92adc_set_dai_fmt(struct snd_soc_dai *codec_dai, unsigned int fmt)
{	
	ENTER();
	SDBG("Record from ADC is only support sample rate 8K/11025/16K sample rate");
	LEAVE();
	return 0;
}

/*
 * Set PCM DAI bit size and sample rate.
 */
static int w55fa92adc_hw_params(struct snd_pcm_substream *substream,
                            struct snd_pcm_hw_params *params,
                            struct snd_soc_dai *dai)
{
#if 0
	unsigned int nSamplingRate;

	ENTER();
	nSamplingRate = params_rate(params);
	adcInit(nSamplingRate);
	if(nSamplingRate==16000)
		w55fa92_set_apll_clock(184320);
	else if(nSamplingRate==11025)	
		w55fa92_set_apll_clock(169344);
	else
		w55fa92_set_apll_clock(184320);	/* 8K sample rate */ 
	SDBG("Sample Rate = %d", nSamplingRate);
	DrvADC_AudioRecordSampleRate((UINT32)eDRVSYS_APLL, nSamplingRate);	/* Hz unit */	
	LEAVE();
#else
	unsigned int nSamplingRate;
	ENTER();
	nSamplingRate = params_rate(params);
	adcInit(nSamplingRate);
	DrvAUR_SetSampleRate(nSamplingRate);
	LEAVE();
#endif
	return 0;
}

static int w55fa92adc_mute(struct snd_soc_dai *dai, int mute)
{	

	struct snd_soc_codec *codec = dai->codec;
#if 0
	ENTER();
    dev_dbg(codec->dev, "%s: %d\n", __func__, mute);

    if (mute){
        DrvADC_Mute(TRUE);
		SDBG("       Enable Mute\n");
	}else{
        DrvADC_Mute(FALSE);
		SDBG("       Disable Mute\n");			
	}
	LEAVE();
	return 0;
#else
	ENTER();

    dev_dbg(codec->dev, "%s: %d\n", __func__, mute);
	if (mute){
        DrvADC_Mute(TRUE);
		SDBG("       Enable Mute\n");
	}else{
        DrvADC_Mute(FALSE);
		SDBG("       Disable Mute\n");			
	}

	LEAVE();
	return 0;
#endif 
}

static int w55fa92adc_set_bias_level(struct snd_soc_codec *codec,
                                 enum snd_soc_bias_level level)
{	
	ENTER();	
	LEAVE();
    return 0;
}

static struct snd_soc_dai_ops w55fa92adc_codec_dai_ops = {
        .hw_params		= w55fa92adc_hw_params,
        .digital_mute	= w55fa92adc_mute,
        .set_fmt		= w55fa92adc_set_dai_fmt,
        .set_clkdiv		= w55fa92adc_set_dai_clkdiv,
        .set_sysclk		= w55fa92adc_set_dai_sysclk,
};

/* Also supports 12kHz */
struct snd_soc_dai w55fa92adc_codec_dai = {
        .name = "w55fa92ADC HiFi",
        .id = 1,
	#if 0	
        .playback = {
                .stream_name = "Playback",
                .channels_min = 1,
                .channels_max = 2,
                .rates = SNDRV_PCM_RATE_8000_48000,
                .formats = NAU8822_FORMATS,
        },
	#endif	
        .capture = {
                .stream_name = "Capture",
                .channels_min = 1,
                .channels_max = 1,
                .rates = (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |SNDRV_PCM_RATE_16000 |\
							SNDRV_PCM_RATE_22050 | SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 |\
							SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_192000),
                .formats = SNDRV_PCM_FMTBIT_S16_LE,
        },
        .ops = &w55fa92adc_codec_dai_ops,
       };
EXPORT_SYMBOL_GPL(w55fa92adc_codec_dai);

static int w55fa92adc_suspend(struct platform_device *pdev, pm_message_t state)
{	
    //struct snd_soc_device *socdev = platform_get_drvdata(pdev);
    //struct snd_soc_codec *codec = socdev->card->codec;
	SDBG("Suspend\n");
	ENTER();  
	LEAVE();
    return 0;
}

static int w55fa92adc_resume(struct platform_device *pdev)
{	
	SDBG("Resume\n");
	ENTER();
	LEAVE();
    return 0;
}

static int w55fa92adc_probe(struct platform_device *pdev)
{
    struct snd_soc_device *socdev = platform_get_drvdata(pdev);
    struct snd_soc_codec *codec;
    int ret = 0;

	ENTER();
    if (w55fa92adc_codec == NULL) {
       	dev_err(&pdev->dev, "Codec device not registered\n");
		ERRLEAVE();
       	return -ENODEV;
    }

    socdev->card->codec = w55fa92adc_codec;
    codec = w55fa92adc_codec;

    /* register pcms */
    ret = snd_soc_new_pcms(socdev, SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1);
    if (ret < 0) {
            dev_err(codec->dev, "failed to create pcms: %d\n", ret);
			ERRLEAVE();	
            goto pcm_err;
    }
#if 1 /* Add volume control */
    snd_soc_add_controls(codec, w55fa92adc_snd_controls,
                            ARRAY_SIZE(w55fa92adc_snd_controls));
    w55fa92adc_add_widgets(codec);
#endif 
	LEAVE();
	return ret;
pcm_err:
	ERRLEAVE();
    return ret;
}

/* power down chip */
static int w55fa92adc_remove(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);

	ENTER();
        snd_soc_free_pcms(socdev);
        snd_soc_dapm_free(socdev);
	LEAVE();
        return 0;
}

struct snd_soc_codec_device soc_codec_dev_w55fa92adc = {
        .probe		= w55fa92adc_probe,
        .remove		= w55fa92adc_remove,
        .suspend	= w55fa92adc_suspend,
        .resume		= w55fa92adc_resume,
};
EXPORT_SYMBOL_GPL(soc_codec_dev_w55fa92adc);


static __devinit int w55fa92adc_register(struct w55fa92adc_priv *w55fa92adc)
{	
	int ret;
	struct snd_soc_codec *codec = &w55fa92adc->codec;

	ENTER();
	
	if (w55fa92adc_codec) {
		dev_err(codec->dev, "Another w55fa92adc is registered\n");
		ERRLEAVE();
		return -EINVAL;
	}

	/*
	 * Set default system clock to PLL, it is more precise, this is also the
	 * default hardware setting
	 */
/*
	nau8822->sysclk = NAU8822_PLL;
*/
	mutex_init(&codec->mutex);

#if 1
	INIT_LIST_HEAD(&codec->dapm_widgets);
	INIT_LIST_HEAD(&codec->dapm_paths);
#endif 
	snd_soc_codec_set_drvdata(codec, w55fa92adc);
	codec->name = "W55FA92_ADC";	/* The name can not be NULL */
	codec->owner = THIS_MODULE;
	codec->bias_level = SND_SOC_BIAS_OFF;

	codec->set_bias_level = w55fa92adc_set_bias_level;
    codec->dai = &w55fa92adc_codec_dai;
	codec->num_dai = 1;
	
	codec->reg_cache_size = W55FA92_ADC_CACHEREGNUM;
   	codec->reg_cache = &w55fa92adc->reg_cache;
	memcpy(codec->reg_cache, w55fa92_adc_reg, sizeof(w55fa92_adc_reg));

//	ret = snd_soc_codec_set_cache_io(codec, 7, 9, SND_SOC_I2C);
  	ret = snd_soc_codec_set_cache_io(codec, 8, 8, SND_SOC_I2C);
    if (ret < 0) {
            dev_err(codec->dev, "Failed to set cache I/O: %d\n", ret);
            goto err;
    }

    w55fa92adc_codec_dai.dev = codec->dev;

	w55fa92adc_codec = codec;
	ret = snd_soc_register_codec(codec);
	if (ret != 0) {
		dev_err(codec->dev, "Failed to register codec: %d\n", ret);
		ERRLEAVE();
		goto err;
	}	

	ret = snd_soc_register_dai(&w55fa92adc_codec_dai);
	if (ret != 0) {
			dev_err(codec->dev, "Failed to register DAI: %d\n", ret);
	ERRLEAVE();
			goto err_codec;
	}
	printk("After snd_soc_register_dai\n");

	LEAVE();
    return 0;

err_codec:
    snd_soc_unregister_codec(codec);
err:
    kfree(w55fa92adc);
	ERRLEAVE();
        return ret;
}

static __devexit void w55fa92adc_unregister(struct w55fa92adc_priv *w55fa92adc)
{
	ENTER();

    w55fa92adc_set_bias_level(&w55fa92adc->codec, SND_SOC_BIAS_OFF);
    snd_soc_unregister_dai(&w55fa92adc_codec_dai);
    snd_soc_unregister_codec(&w55fa92adc->codec);
    kfree(w55fa92adc);
    w55fa92adc_codec = NULL;

	LEAVE();
}

static __devinit int w55fa92_adc_i2c_probe(struct i2c_client *i2c,
                                      const struct i2c_device_id *id)
{
	
#if 0	//Due to volume, it need to be a global variable. 
        struct w55fa92adc_priv *w55fa92adc;
#endif 
        struct snd_soc_codec *codec;
		ENTER();	
        w55fa92adc = kzalloc(sizeof(struct w55fa92adc_priv), GFP_KERNEL);
        if (w55fa92adc == NULL)
                return -ENOMEM;

        codec = &w55fa92adc->codec;
#if 0
		codec->read = read_reg_cache;
		codec->write = w55fa92_adc_write;
		codec->control_data = i2c;
#else
		codec->hw_write = (hw_write_t)i2c_master_send;
		codec->control_data = i2c;
#endif
        i2c_set_clientdata(i2c, w55fa92adc);
        
        codec->dev = &i2c->dev;
//        codec->dev = 0x11;	// temp value, non-zero 

        return w55fa92adc_register(w55fa92adc);
}

static __devexit int w55fa92_adc_i2c_remove(struct i2c_client *client)
{
        struct w55fa92adc_priv *w55fa92adc_priv = i2c_get_clientdata(client);
        w55fa92adc_unregister(w55fa92adc_priv);
        return 0;
}

static const struct i2c_device_id w55fa92_adc_i2c_id[] = {
        { "w55fa92_adc_i2c", 0 },	/* link mach-w55fa92.c */
        { }
};
MODULE_DEVICE_TABLE(i2c, w55fa92_dac_i2c_id);

static struct i2c_driver w55fa92_adc_i2c_driver = {
        .driver = {
                .name = "W55FA92_ADC_I2C",		/* Same as codec->name */
                .owner = THIS_MODULE,
        },
        .probe =    w55fa92_adc_i2c_probe,
        .remove =   __devexit_p(w55fa92_adc_i2c_remove),
        .id_table = w55fa92_adc_i2c_id,
};

static int __init w55fa92adc_modinit(void)
{
	int ret =0;
	ENTER();
#if 1
	ret = i2c_add_driver(&w55fa92_adc_i2c_driver);
	printk("w55fa92-dac-i2c ret = 0x%x \n", ret);        
	return 0;
#else
	w55fa92adc_moduleinit();
#endif
	LEAVE();
	return ret;
}
module_init(w55fa92adc_modinit);

static void __exit w55fa92adc_exit(void)
{
	ENTER();
	i2c_del_driver(&w55fa92_adc_i2c_driver);
	LEAVE();
}
module_exit(w55fa92adc_exit);

MODULE_DESCRIPTION("ASoC w55fa92ADC codec driver");
MODULE_LICENSE("GPL");

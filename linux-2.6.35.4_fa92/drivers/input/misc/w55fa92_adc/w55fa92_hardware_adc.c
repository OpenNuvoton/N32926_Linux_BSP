/****************************************************************
 *                                                              *
 * Copyright (c) Nuvoton Technology Corp. All rights reserved.  *
 *                                                              *
 ****************************************************************/
#ifdef CONFIG_INPUT_W55FA92_ADC
	#include <asm/io.h>
	#include <mach/irqs.h>
	#include <mach/w55fa92_reg.h>
	#include <linux/input.h>
	#include <linux/clk.h>
	#include <linux/delay.h>
	#include <linux/sched.h>	
	#include <linux/clk.h>
	#include "w55fa92_ts_adc.h"
#else
	#include <stdio.h>
	#include <string.h>
	#include "w55fa92_reg.h"
	#include "wblib.h"
	#include "w55fa92_ts_adc.h"
#endif



//static UINT16 g_Xpos, g_Ypos;

static volatile UINT16 g_Z1press, g_Z2press;
static volatile UINT16 g_KeyCode;
static volatile UINT16 g_Voltage;

volatile UINT32 u32AdcMode = E_ADC_NONE;

static PFN_ADC_CALLBACK (g_psAdcCallBack)[5] = {0};
//PFN_VIDEOIN_CALLBACK (pfnVideoInIntHandlerTable)[2][4]={0};
// 0 KEY	
// 1 Pen down
// 2 Normal ADC Conversion
// 4 Touch XY Position
// 5 Touch Z Position

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


INT32 DrvADC_InstallCallback(E_ADC_INT_TYPE eIntType,
	PFN_ADC_CALLBACK pfnCallback,
	PFN_ADC_CALLBACK* pfnOldCallback)
{
 	if( (eIntType>eADC_PRESSURE) ||  (eIntType<eADC_KEY))
 		return -1;
	pfnOldCallback = &(g_psAdcCallBack[eIntType]);
    g_psAdcCallBack[eIntType] = pfnCallback; 	    		
    return Successful;
}

static spinlock_t mode_lock = SPIN_LOCK_UNLOCKED;
static void adcSetOperationMode(UINT32 u32Mode)
{
#ifdef CONFIG_INPUT_W55FA92_ADC
	#if 0
	UINT32 u32IntH, u32IntL;
	u32IntH = inp32(REG_AIC_IMRH);
	u32IntL = inp32(REG_AIC_IMR);
	outp32(REG_AIC_MDCRH, 0xFFFFFFFF);
	outp32(REG_AIC_MDCR, 0xFFFFFFFF);
	u32AdcMode = u32Mode;	
	outp32(REG_AIC_MECRH, u32IntH);
	outp32(REG_AIC_MECR, u32IntL);
	#else
	unsigned long flags;
	//printk("Set mode %d\n", u32AdcMode);
	spin_lock_irqsave(&mode_lock, flags);
	//DBG_PRINTF("spin_lock_irqsave\n");
	u32AdcMode = u32Mode;	
	spin_unlock_irqrestore(&mode_lock, flags);
	//DBG_PRINTF("spin_unck_irqrestore\n");
	#endif
#else
	BOOL bIsIBitEnable = sysGetIBitState();
	if(bIsIBitEnable==TRUE)
		sysSetLocalInterrupt(DISABLE_IRQ);
	
	u32AdcMode = u32Mode;	
	
	if(bIsIBitEnable==TRUE)		
		sysSetLocalInterrupt(ENABLE_IRQ);	
#endif
}	
static UINT32 adcGetOperationMode(void)
{
	return u32AdcMode;
}
static UINT32 u32ValidPosition;	
static UINT32 u32ValidPressure;	
#ifdef CONFIG_INPUT_W55FA92_ADC
void AdcIntHandler(void)
#else
static void AdcIntHandler(void)
#endif
{
	UINT32 u32IntStatus;
	UINT32 u32OperationMode;


	u32IntStatus = inp32(REG_TP_INTST);
	u32OperationMode = adcGetOperationMode();
	//printk("Int\n");	
#if 0	
	if((u32IntStatus &INT_KEY) ==INT_KEY){//Key
		DBG_PRINTF("Keypad Int\n");		
		if(u32OperationMode == E_ADC_KEY_WAIT){			
			DBG_PRINTF("Keypad down\n");				
	        	DrvADC_DisableInt(eADC_KEY);
			outp32(REG_TP_INTST, (inp32(REG_TP_INTST) & ~(INT_NOR|INT_TC) ) | INT_KEY);	/* Write one clear */
			/* Force ADC to convese key code */
			adcSetOperationMode(E_ADC_KEY);
			DrvADC_EnableInt(eADC_AIN);	
			outp32(REG_TP_CTL1, (inp32(REG_TP_CTL1) & ~(SW_GET|GET_PRESSURE|GET_XY|GET_X|GET_Y)) | SW_GET);
		}else if (u32OperationMode == E_ADC_KEY){
			   DBG_PRINTF("Get Keypad value\n");	
			   g_psAdcCallBack[0](inp32(REG_NORM_DATA));  	/* Notice Upper lay key pressing */				
			   outp32(REG_TP_INTST, (inp32(REG_TP_INTST) & ~(INT_NOR|INT_TC) ) | INT_KEY);	/* Write one clear */
			   DrvADC_DisableInt(eADC_AIN);				   
			   adcSetOperationMode(E_ADC_NONE);	
		}
	}
#endif		
	if((u32IntStatus &INT_TC) ==INT_TC){//Touch	
		#if 0	
		if(u32OperationMode == E_ADC_NONE){
				
				printk("Touch down\n");		
				DrvADC_DisableInt(eADC_TOUCH);
				outp32(REG_TP_INTST, (inp32(REG_TP_INTST) & ~(INT_NOR|INT_KEY)) | INT_TC);	/* Write one clear */
					
				adcSetOperationMode(E_ADC_TP_GETXY);
				DrvADC_EnableInt(eADC_AIN);			
				/* Force ADC to get (X, Y) Position */
				outp32(REG_TP_CTL1, (inp32(REG_TP_CTL1) & ~(SW_GET|GET_PRESSURE|GET_XY|GET_X|GET_Y)) | GET_XY);
			
		}
		#endif
	}
	if((u32IntStatus &INT_NOR) ==INT_NOR){//AIN
		DBG_PRINTF("Normal Int\n");	
		if(u32OperationMode == E_ADC_TP_GETXY){		
			u32ValidPosition = 	inp32(REG_XY_DATA);	
			if( (u32ValidPosition&(BIT31|BIT15)) == (BIT31|BIT15) ){
#if 0	//Get position then pressure
				adcSetOperationMode(E_ADC_TP_GETZ);	
				/* Force ADC to get Z Pressure */			
				outp32(REG_TP_CTL1, (inp32(REG_TP_CTL1) & ~(SW_GET|GET_PRESSURE|GET_XY|GET_X|GET_Y)) | GET_PRESSURE);	
#else	//Only get position 
				if( g_psAdcCallBack[3]!=0){	
					DBG_PRINTF("Report XY = 0x%x\n", (u32ValidPosition & ~(BIT31|BIT15)) );
					g_psAdcCallBack[3]((u32ValidPosition & ~(BIT31|BIT15)));  /* Notice Upper lay XY Position */
				}	
				adcSetOperationMode(E_ADC_NONE);
				outp32(REG_TP_CTL1, (inp32(REG_TP_CTL1) & ~(SW_GET|GET_PRESSURE|GET_XY|GET_X|GET_Y)));	
#endif
			}else{
				u32ValidPosition = 0;
				adcSetOperationMode(E_ADC_NONE);
				outp32(REG_TP_CTL1, (inp32(REG_TP_CTL1) & ~(SW_GET|GET_PRESSURE|GET_XY|GET_X|GET_Y)));	
			}		
			outp32(REG_TP_INTST, (inp32(REG_TP_INTST)&~(INT_NOR|INT_KEY)) | INT_NOR);	/* Write one clear */				
		}else if(u32OperationMode == E_ADC_TP_GETZ){
			DBG_PRINTF("???Get Z value\n");
			g_Z1press = inp32(REG_Z_DATA) >>16;
			g_Z2press = inp32(REG_Z_DATA) & 0xFFFF;	
			u32ValidPressure = inp32(REG_Z_DATA);
			
			if( (u32ValidPressure &(BIT31|BIT15)) == (BIT31|BIT15) ){
				if( g_psAdcCallBack[3]!=0)
					g_psAdcCallBack[3](u32ValidPosition);  /* Notice Upper lay XY Position */	
				if( g_psAdcCallBack[4]!=0)	
					g_psAdcCallBack[4](u32ValidPressure);  	/* Notice Upper lay Z Pressure */	
			}else
				u32ValidPressure = 0;
			adcSetOperationMode(E_ADC_NONE);
			/* Force ADC to None */
			outp32(REG_TP_CTL1, (inp32(REG_TP_CTL1) & ~(SW_GET|GET_PRESSURE|GET_XY|GET_X|GET_Y)));	
			outp32(REG_TP_INTST, (inp32(REG_TP_INTST)&~(INT_NOR|INT_KEY)) | INT_NOR);	/* Write one clear */			
		}else if (u32OperationMode == E_ADC_VD){
			DBG_PRINTF("Get battery value\n");			 
			g_Voltage = inp32(REG_NORM_DATA);	
			DBG_PRINTF("Get Battery value %d\n", g_Voltage);	
			if( g_psAdcCallBack[2]!=0)
				g_psAdcCallBack[2](g_Voltage);  	/* Notice Upper lay voltage detect */	
				adcSetOperationMode(E_ADC_NONE);
			  outp32(REG_TP_INTST, (inp32(REG_TP_INTST)&~(INT_NOR|INT_KEY)) | INT_NOR);	/* Write one clear */
		}else if (u32OperationMode == E_ADC_KEY){
			   DBG_PRINTF("Get Keypad value\n");
			   g_KeyCode = 	inp32(REG_NORM_DATA);	
			   if( g_psAdcCallBack[0]!=0)		
			  	 g_psAdcCallBack[0](inp32(REG_NORM_DATA));  	/* Notice Upper lay key pressing */				
			   outp32(REG_TP_INTST, (inp32(REG_TP_INTST) & ~(INT_NOR|INT_TC) ) | INT_KEY);	/* Write one clear */
			   DrvADC_DisableInt(eADC_AIN);				   	
				adcSetOperationMode(E_ADC_NONE);
		}		
	}	
	if(u32OperationMode == E_ADC_NONE )
		outp32(REG_TP_INTST, (inp32(REG_TP_INTST))  | (INT_KEY|INT_NOR | INT_TC));	/* Alway clear all interrupt status if ADC free */
}
#define REAL_CHIP
#ifdef CONFIG_INPUT_W55FA92_ADC
INT32 i32AdcOpenCnt = 0;
#endif
extern void nvt_lock(void);
extern void nvt_unlock(void);
INT32 DrvADC_Open(void)
{	
	//UINT8 u8RegDac;
	/* Enable Clock */
	struct clk *clk;
#ifdef CONFIG_INPUT_W55FA92_ADC
	if(i32AdcOpenCnt != 0){
		printk("ADC has been open\n");//ADC had been opened.		
		i32AdcOpenCnt = i32AdcOpenCnt+1;	//Second~... time
		return 	Successful;							
	}
	i32AdcOpenCnt = i32AdcOpenCnt+1;		//First time 
#endif


	//outp32(REG_APBCLK, inp32(REG_APBCLK) | TOUCH_CKE);
	clk = clk_get(NULL, "touch");
	clk_enable(clk);
	/* Specified Clock */
	nvt_lock();
#ifdef  REAL_CHIP
	outp32(REG_CLKDIV5, (inp32(REG_CLKDIV5) & ~(TOUCH_N1 | TOUCH_S| TOUCH_N0)) );		/* Fed to ADC clock need 12MHz=External clock */
#else	
	outp32(REG_CLKDIV5, (inp32(REG_CLKDIV5) & ~(TOUCH_N1 | TOUCH_S| TOUCH_N0)) | (1<<22) );/* FPGA: Divider need to be 1 at least for I2C clock*/
#endif 	
	nvt_unlock();

	/* IP Reset */
	outp32(REG_APBIPRST, TOUCHRST);
	outp32(REG_APBIPRST, 0);	
	
	outp32(REG_TP_CTL1, inp32(REG_TP_CTL1) & ~(PD_Power|PD_BUF| ADC_SLEEP) ); //| PD_BUF); //If REF_SEL!=0 PD_BUF==>1
	outp32(REG_TP_CTL1, inp32(REG_TP_CTL1) | REF_SEL);		//Vref reference voltage
	
	outp32(REG_TP_CTL2, 0xFF);	/* Delay 48 ADC */
	printk("ADC opening  REG_TP_CTL1 = 0x%x\n", inp32(REG_TP_CTL1));
#ifdef CONFIG_INPUT_W55FA92_ADC

#else
	sysInstallISR(IRQ_LEVEL_1, 
					IRQ_TOUCH, 
					(PVOID)AdcIntHandler);
	sysEnableInterrupt(IRQ_TOUCH);		
#endif
	return Successful;		
}
INT32 DrvADC_GetChannel(void)
{
	return ((inp32(REG_TP_CTL1)&IN_SEL)>>16);
}
INT32 DrvADC_Channel(UINT32 u32Channel)
{
	if(u32Channel>7)
		return -1;
	outp32(REG_TP_CTL1, (inp32(REG_TP_CTL1) & ~IN_SEL) | (u32Channel<<16));
	return Successful;
}

INT32 DrvADC_EnableInt(E_ADC_INT_TYPE eIntType)
{
	if( (eIntType>eADC_AIN) ||  (eIntType<eADC_KEY))
 		return -1;
	outp32(REG_TP_INTST, ((inp32(REG_TP_INTST) & ~(INT_NOR|INT_TC|INT_KEY)) |  ((1<<eIntType)<<4)) );
	return Successful;	
}	
INT32 DrvADC_DisableInt(E_ADC_INT_TYPE eIntType)
{
	if( (eIntType>eADC_AIN) ||  (eIntType<eADC_KEY))
 		return -1;
	outp32(REG_TP_INTST, (inp32(REG_TP_INTST)  & ~(INT_NOR|INT_TC|INT_KEY) ) &  ~((1<<eIntType)<<4)  ) ;
	return Successful;		
}	

INT32 DrvADC_Close(void)
{			
	struct clk *clk;
#ifdef CONFIG_INPUT_W55FA92_ADC
	i32AdcOpenCnt = i32AdcOpenCnt-1;
	if(i32AdcOpenCnt < 0)
		i32AdcOpenCnt = 0;
	if(i32AdcOpenCnt != 0){
		printk("ADC Still need to be opened\n");
		return Successful; 			//ADC Still need to be opened
	}		
#endif
	//outp32(REG_APBCLK, inp32(REG_APBCLK) & ~TOUCH_CKE);
	clk = clk_get(NULL, "touch");
	clk_disable(clk);

	printk("ADC Close\n");	
	return Successful;
}
/*
	The function should be called every 20ms.  
*/
void TouchDelay(UINT32 u32Dly)
{//In CPU 162MHz. The u32Dly will be 1. 
#ifdef CONFIG_INPUT_W55FA92_ADC
	udelay(10*u32Dly);
#else
	volatile UINT32 u32Delay;
	for(u32Delay =0; u32Delay<(1*(u32Dly+1)); u32Delay=u32Delay+1);
#endif
}
INT32 DrvADC_PenDetection(BOOL bIs5Wire)
{	
	if(adcGetOperationMode() == E_ADC_NONE){ 
		adcSetOperationMode(E_ADC_TP_WAIT);
		if(bIs5Wire==1)
			outp32(REG_TP_CTL1, inp32(REG_TP_CTL1)  | TSMODE);
		else
			outp32(REG_TP_CTL1, inp32(REG_TP_CTL1)  & ~TSMODE);
		DrvADC_DisableInt(eADC_TOUCH);	
		outp32(REG_TP_CTL1, inp32(REG_TP_CTL1)  | IN_SEL);
		outp32(REG_TP_CTL1, (inp32(REG_TP_CTL1) & ~(XP_EN | XM_EN | YP_EN)) | (YM_EN| PLLUP) );
				
		TouchDelay(1);			/* Due to PLLUP bit cause the INT_TC was set. */
		outp32(REG_TP_INTST, (inp32(REG_TP_INTST) &~(INT_NOR|INT_KEY) ) | INT_TC);	/* Write one clear */		
			
		TouchDelay(1);	
		
		if(inp32(REG_TP_INTST)&INT_TC) {
			DrvADC_DisableInt(eADC_TOUCH);
			
			//outp32(REG_TP_CTL1, (inp32(REG_TP_CTL1) & ~PLLUP) );
			
			outp32(REG_TP_INTST, (inp32(REG_TP_INTST) & ~(INT_NOR|INT_KEY) ) | INT_TC);	/* Write one clear */
			/* Force ADC to convese key code */
			adcSetOperationMode(E_ADC_TP_GETXY);
			DrvADC_EnableInt(eADC_AIN);	
			//outp32(REG_TP_CTL1, (inp32(REG_TP_CTL1) & ~(SW_GET|GET_PRESSURE|GET_XY|GET_X|GET_Y)) | SW_GET);
			outp32(REG_TP_CTL1, (inp32(REG_TP_CTL1) & ~(SW_GET|GET_PRESSURE|GET_XY|GET_X|GET_Y)) | GET_XY);
		}else {
			outp32(REG_TP_CTL1, (inp32(REG_TP_CTL1) & ~PLLUP) );
			outp32(REG_TP_INTST, (inp32(REG_TP_INTST) & ~(INT_NOR|INT_KEY) ) | INT_TC);	/* Write one clear */
			adcSetOperationMode(E_ADC_NONE);
			return E_TOUCH_UP;
		}
		
		return Successful;
	}	
	return E_ADC_BUSY;
}


void DrvADC_Wakeup(E_ADC_WAKEUP_TYPE eWakeupSrc)
{
	if(eWakeupSrc&eADC_WAKEUP_TOUCH){
		outp32(REG_TP_CTL1, (inp32(REG_TP_CTL1) & ~(XP_EN | XM_EN | YP_EN)) | (YM_EN| PLLUP) );
				
		TouchDelay(1);			/* Due to PLLUP bit cause the INT_TC was set. */
		outp32(REG_TP_INTST, (inp32(REG_TP_INTST) &~(INT_NOR|INT_KEY) ) | INT_TC);	/* Write one clear */		
	}else{
		outp32(REG_TP_CTL1, (inp32(REG_TP_CTL1) & ~(XP_EN | XM_EN | YP_EN)) );
	}
	if(eWakeupSrc&eADC_WAKEUP_KEY){
		outp32(REG_TP_CTL1, inp32(REG_TP_CTL1) & ~PD_Power ); //Not to Power down key pad
	}else{
		outp32(REG_TP_CTL1, inp32(REG_TP_CTL1) | PD_Power );    //Power down key pad
	}	
}				
/*
	The function should be called every 20ms.  
*/
void schedule_delay(void)
{
	unsigned long j=0;	
	j = jiffies + 30*HZ/1000; 	/* 2Xms~30ms */			
	while( time_before(jiffies,j) )
			schedule();
}
INT32 DrvADC_KeyDetection(UINT32 u32Channel, UINT32* pu32KeyCode)
{
#if 1
	/*
		Due to both drivers(touch+LVD and Keypad) use same Irq.
		Keypad will do't use interrupt 
	*/
	if(adcGetOperationMode() == E_ADC_NONE){
		adcSetOperationMode(E_ADC_KEY_WAIT);
		DrvADC_Channel(u32Channel);
		outp32(REG_TP_INTST, (inp32(REG_TP_INTST) | INT_KEY));	/* Write one clear */
		DrvADC_DisableInt(eADC_KEY);
		if( (inp32(REG_TP_INTST) & INT_KEY)  == INT_KEY){/* Key press */
			g_KeyCode = 0xFFFF; /* 12 bits ADC, ADC value can't report up to 0xFFFF */ 
			adcSetOperationMode(E_ADC_KEY);
		#if 1
			DrvADC_EnableInt(eADC_AIN);
			outp32(REG_TP_CTL1, (inp32(REG_TP_CTL1) & ~(SW_GET|GET_PRESSURE|GET_XY|GET_X|GET_Y)) | SW_GET); /* trigger NORMAL conversion */			
			while(g_KeyCode==0xFFFF);
			*pu32KeyCode = g_KeyCode;
		#else	
			DrvADC_DisableInt(eADC_AIN);	
			outp32(REG_TP_CTL1, (inp32(REG_TP_CTL1) & ~(SW_GET|GET_PRESSURE|GET_XY|GET_X|GET_Y)) | SW_GET); /* trigger NORMAL conversion */							
			udelay(10);
			while((inp32(REG_TP_INTST)&INT_NOR) !=INT_NOR){	
				udelay(10);//schedule_delay();
				printk("wait complete");
			}
			g_KeyCode = inp32(REG_NORM_DATA);	
			if( g_psAdcCallBack[0]!=0)		
			  	 g_psAdcCallBack[0](inp32(REG_NORM_DATA));  	/* Notice Upper lay key pressing */				
			outp32(REG_TP_INTST, (inp32(REG_TP_INTST) & ~(INT_NOR|INT_TC) ) | INT_KEY);	/* Write one clear */
			adcSetOperationMode(E_ADC_NONE);

			*pu32KeyCode = g_KeyCode;	
		#endif
			return Successful;	
		}	
		else{/* Key not press */
			*pu32KeyCode = 0;
			DBG_PRINTF("Key up\n");
			adcSetOperationMode(E_ADC_NONE);
			return E_KEYPAD_UP;	
		}		
	}
	return E_ADC_BUSY;	
#else
	if(adcGetOperationMode() == E_ADC_NONE){
		adcSetOperationMode(E_ADC_KEY_WAIT);
		DrvADC_Channel(u32Channel);
		outp32(REG_TP_INTST, (inp32(REG_TP_INTST) | INT_KEY));	/* Write one clear */
		//DrvADC_EnableInt(eADC_KEY);
		if( (inp32(REG_TP_INTST) & INT_KEY)  == INT_KEY){/* Key press */
			g_KeyCode = 0xFFFF; /* 12 bits ADC, ADC value can't report up to 0xFFFF */ 
			adcSetOperationMode(E_ADC_KEY);
			DrvADC_EnableInt(eADC_AIN);
			outp32(REG_TP_CTL1, (inp32(REG_TP_CTL1) & ~(SW_GET|GET_PRESSURE|GET_XY|GET_X|GET_Y)) | SW_GET); /* trigger NORMAL conversion */
			while(g_KeyCode==0xFFFF);
			*pu32KeyCode = g_KeyCode;
			return Successful;	
		}	
		else{/* Key not press */
			*pu32KeyCode = 0;
			DBG_PRINTF("Key up\n");
			adcSetOperationMode(E_ADC_NONE);
			return E_KEYPAD_UP;	
		}		
	}
	return E_ADC_BUSY;
#endif	
	
}
/*
	The function should be called every 20ms.  
*/
INT32 DrvADC_VoltageDetection(UINT32 u32Channel)
{
	ENTER();
	if(adcGetOperationMode() == E_ADC_NONE){ 
		adcSetOperationMode(E_ADC_VD);
		DrvADC_Channel(u32Channel);
		outp32(REG_TP_INTST, (inp32(REG_TP_INTST) | INT_NOR));	/* Write one clear */
		DrvADC_EnableInt(eADC_AIN);
		
		//outp32(REG_TP_CTL1, inp32(REG_TP_CTL1) | (XP_EN | IN_SEL));	/*  test only */
		//outp32(REG_TP_CTL1, inp32(REG_TP_CTL1) | (IN_SEL));	/*  test only */
		//outp32(REG_TP_CTL1, inp32(REG_TP_CTL1) | XP_EN ); /*  test only */
		
		outp32(REG_TP_CTL1, inp32(REG_TP_CTL1) | SW_GET);
		LEAVE();
		return Successful;
	}	
	LEAVE();
	return E_ADC_BUSY;	
}
unsigned int PowerOnBatteryDetection(void)
{
	int i32Channel=2, vol;
#ifdef CONFIG_BATTERY_CHANNEL_1
	i32Channel = 1;
#endif
#ifdef CONFIG_BATTERY_CHANNEL_2	
	i32Channel = 2;
#endif
#ifdef CONFIG_BATTERY_CHANNEL_3	
	i32Channel = 3;
#endif
	/* First time to get voltage, don't need to consider the  */
	DrvADC_Open();
	adcSetOperationMode(E_ADC_VD);	
	DrvADC_Channel(i32Channel);
	outp32(REG_TP_INTST, (inp32(REG_TP_INTST) | INT_NOR));	/* Write one clear */
	DrvADC_DisableInt(eADC_AIN);
	outp32(REG_TP_CTL1, inp32(REG_TP_CTL1) | SW_GET);	
	while((inp32(REG_TP_INTST)&INT_NOR) !=INT_NOR);
	vol = inp32(REG_NORM_DATA);			
	outp32(REG_TP_INTST, (inp32(REG_TP_INTST)&~(INT_NOR|INT_KEY)) | INT_NOR);	/* Write one clear */
	adcSetOperationMode(E_ADC_NONE);
	DrvADC_Close();
	return vol;
}


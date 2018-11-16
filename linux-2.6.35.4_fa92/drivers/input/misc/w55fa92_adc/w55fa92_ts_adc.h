/****************************************************************
 *                                                              *
 * Copyright (c) Nuvoton Technology Corp. All rights reserved. *
 *                                                              *
 ****************************************************************/
 
#ifndef __W55FA92_TOUCH_H__
#define __W55FA92_TOUCH_H__

#ifdef CONFIG_INPUT_W55FA92_ADC
	
#else
	#include "wblib.h"
#endif

#ifdef  __cplusplus
extern "C"
{
#endif




/* Define data type (struct, union¡K) */
// #define Constant
#define E_ADC_NONE					0
#define E_ADC_TP_WAIT				1
#define E_ADC_TP_GETXY				2
#define E_ADC_TP_GETZ				3
#define E_ADC_VD					4
#define E_ADC_KEY_WAIT				5 
#define E_ADC_KEY					6

//Error message
// E_VIDEOIN_INVALID_INT					Invalid interrupt
// E_VIDEOIN_INVALID_BUF					Invalid buffer
// E_VIDEOIN_INVALID_PIPE					Invalid pipe
// E_VIDEOIN_INVALID_COLOR_MODE			Invalid color mode
#define ADC_ERROR_CODE					0xB800F000
#define E_ADC_BUSY  					(ADC_ERROR_CODE | 0x01)

#define E_TOUCH_UP					1
#define E_KEYPAD_UP					1




#ifdef CONFIG_INPUT_W55FA92_ADC
	#define VOID		void
	#define INT32		int
	#define UINT32		unsigned int
	#define BOOL		unsigned int
	#define UINT16 		unsigned short int 
	#define Successful	0
	
	#undef 	inp32
	#undef 	outp32
	#define outp32(addr, value)		writel(value, addr)
	#define inp32(addr)				readl(addr)

	#define TOUCH_INTERVAL_TIME  	HZ/50	/* HZ = 100. INTERVAL_TIME=2 tick==> 20ms */
	#define BATTERY_INTERVAL_TIME	HZ/10	/* HZ = 10. 100ms */
	#define KEYPAD_INTERVAL_TIME	HZ/20	/* HZ = 100. INTERVAL_TIME=5 tick==> 50ms */
	#define INIT_INTERVAL_TIME		1		/* 10ms */

	void init_touchpanel(void);
	void init_batterydetection(void);	
	void init_keypad(void);
#endif 

typedef VOID (*PFN_ADC_CALLBACK)(unsigned int);


typedef enum{
	eADC_KEY 	= 0,		/* Hardware */
	eADC_TOUCH   = 1,
	eADC_AIN 	= 2,
	
	eADC_POSITION = 3,	/* Software */
	eADC_PRESSURE = 4 			
}E_ADC_INT_TYPE;

typedef enum{
	eADC_WAKEUP_KEY 	= 1,		/* Hardware */
	eADC_WAKEUP_TOUCH   = 2	
}E_ADC_WAKEUP_TYPE;

INT32 DrvADC_Open(void);
INT32 DrvADC_Close(void);
INT32 DrvADC_InstallCallback(E_ADC_INT_TYPE eIntType,
	PFN_ADC_CALLBACK pfnCallback,
	PFN_ADC_CALLBACK* pfnOldCallback);
INT32 DrvADC_EnableInt(E_ADC_INT_TYPE eIntType);
INT32 DrvADC_DisableInt(E_ADC_INT_TYPE eIntType);

void DrvADC_Wakeup(E_ADC_WAKEUP_TYPE eWakeupSrc); /* Only support eADC_KEY & eADC_TOUCH */
INT32 DrvADC_PenDetection(BOOL bIs5Wire);
INT32 DrvADC_KeyDetection(UINT32 u32Channel, UINT32* pu32KeyCode);
INT32 DrvADC_VoltageDetection(UINT32 u32Channel);
INT32 DrvADC_GetChannel(void);
unsigned int PowerOnBatteryDetection(void);
#ifdef CONFIG_INPUT_W55FA92_ADC
void AdcIntHandler(void);
#endif
/* API  */
#if 0
INT32 DrvAUR_AutoClampingGain(UINT32 u32MaxGain, UINT32 u32MinGain);
INT32 DrvAUR_AutoGainTiming(UINT32 u32Attack, UINT32 u32Recovery, UINT32 u32Hold);
INT32 DrvAUR_NoiseGatCtrl(BOOL bIsEnable, UINT32 u32Gain, UINT32 u32Level);
INT32 DrvAUR_NoiseGateTiming(UINT32 u32DelayTime, UINT32 u32InTime, UINT32 u32OutTime);
INT32 DrvAUR_AudioI2cRead(UINT32 u32Addr, UINT8* p8Data);
INT32 DrvAUR_AudioI2cWrite(UINT32 u32Addr, UINT32 u32Data);
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
#endif 

#ifdef __cplusplus
}
#endif

#endif /* __W55FA92_TOUCH_H__ */


















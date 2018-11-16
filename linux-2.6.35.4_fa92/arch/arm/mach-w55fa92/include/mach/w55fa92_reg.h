/* linux/include/asm-arm/arch-w55fa92/w55fa92_reg.h
 *
 * Copyright (c) 2008 Nuvoton technology corporation
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * Changelog:
 *
 */

#ifndef _W55FA92_REG_H
#define _W55FA92_REG_H

#include <mach/map.h>

/*
   Register map
*/
#define 	SYS_BA    		W55FA92_VA_GCR	/* System Manager Control */
#define   	GCR_BA			SYS_BA
#define    	CLK_BA			(W55FA92_VA_GCR+0x200)	/* Clock Controller */
#define 	ROT_BA			W55FA92_VA_ROT	/* Rotation Engine Control Registers */
#define 	SDRAM_BA	    W55FA92_VA_SDIC	/* SDRAM Inteface Control Registers */
#define    	CRC_BA    		W55FA92_VA_CRC	/* CRC Control */
#define    	EDMA_BA			W55FA92_VA_EDMA	/* EDMA Controller */
#define    	SPU_BA			W55FA92_VA_SPU	/* SPU Controller */
#define    	I2S_BA			W55FA92_VA_I2SM	/* I2S Controller */
#define    	VPOST_BA  		W55FA92_VA_VPOST	/* VPOST Controller */
#define    	VIN_BA    		W55FA92_VA_VIDEOIN	/* Video-In Controller */
#define     MDCT_BA    		W55FA92_VA_MDCT	/* AAC MDCT / IMDCT Controller */
#define		UOHCI20_BA		W55FA92_VA_UOHCI20	/* USB 2.0 OHCI Host Control*/
#define    	DMAC_BA    		W55FA92_VA_SIC	/* DMA Control */
#define    	FMI_BA    		(W55FA92_VA_SIC+0x800)	/* Flash Memory Card Interface */
#define    	SDIODMAC_BA  	W55FA92_VA_SDIO	/* SDIO DMA Control */
#define    	SDIOFMI_BA  	(W55FA92_VA_SDIO+0x800)	/* SDIO Interface */
#define    	USBD_BA   		W55FA92_VA_USBD	/* USB Device Control */
#define    	USB_BA   		W55FA92_VA_USBD	/* USB Device Control */
#define    	USBH_BA   		W55FA92_VA_USBH	/* USB Host Control */
#define    	JPG_BA    		W55FA92_VA_JPEG	/* JPEG Engine Control */
#define		UEHCI20_BA		W55FA92_VA_UEHCI20	/* USB 2.0 EHCI Host Control*/
#define 	GE_BA			W55FA92_VA_GVE	/* GVE Control */
#define 	VPE_BA			(W55FA92_VA_GVE+0x800)	/* VPE Control */
#define     BLT_BA          W55FA92_VA_BLT  /* 2D BLT Accelerator */
#define 	EMAC_BA			W55FA92_VA_EMC	/* Ethernet MAC */
#define     AES_BA          W55FA92_VA_AES  /* AES Control */
#define    	AIC_BA    		W55FA92_VA_IRQ	/* Interrupt Controller */
#define    	GPIO_BA   		W55FA92_VA_GPIO	/* GPIO Control */
#define    	TMR_BA    		W55FA92_VA_TIMER	/* Timer Control */
#define    	TMR2_BA    		W55FA92_VA_TIMER2	/* Timer 2 Control */
#define    	RTC_BA    		W55FA92_VA_RTC	/* Real Time Clock Control */
#define    	I2C_BA    		W55FA92_VA_I2C	/* I2C Control */
#define    	KPI_BA    		W55FA92_VA_KPI	/* KPI Control */
#define    	PWM_BA    		W55FA92_VA_PWM	/* Pulse Width Modulation (PWM) Control */
#define    	UART_BA   		W55FA92_VA_UART	/* UART Control (console) */
#define    	RF_BA    		W55FA92_VA_RF	/* RF Codec Control */
#define    	RSC_BA    		W55FA92_VA_RSC	/* RS Codec Control */
#define    	SPI0_BA  		W55FA92_VA_SPI0	/* Serial Interface Control 0 */
#define    	SPI1_BA  		(SPI0_BA + 0x400)	/* Serial Interface Control 1 */
#define    	AR_BA    		W55FA92_VA_ADC	/* ADC Control */
#define 	TP_BA			W55FA92_VA_TP	/* Touch Panel Control */
#define     H264D_BA        W55FA92_VA_VDE  /* H.264 Decoder Controller */
#define     H264E_BA        W55FA92_VA_ENC  /* H.264 Encoder Controller */

// Define one bit mask
#define BIT0	0x00000001
#define BIT1	0x00000002
#define BIT2	0x00000004
#define BIT3	0x00000008
#define BIT4	0x00000010
#define BIT5	0x00000020
#define BIT6	0x00000040
#define BIT7	0x00000080
#define BIT8	0x00000100
#define BIT9	0x00000200
#define BIT10	0x00000400
#define BIT11	0x00000800
#define BIT12	0x00001000
#define BIT13	0x00002000
#define BIT14	0x00004000
#define BIT15	0x00008000
#define BIT16	0x00010000
#define BIT17	0x00020000
#define BIT18	0x00040000
#define BIT19	0x00080000
#define BIT20	0x00100000
#define BIT21	0x00200000
#define BIT22	0x00400000
#define BIT23	0x00800000
#define BIT24	0x01000000
#define BIT25	0x02000000
#define BIT26	0x04000000
#define BIT27	0x08000000
#define BIT28	0x10000000
#define BIT29	0x20000000
#define BIT30	0x40000000
#define BIT31	0x80000000

// Define bits mask
#define NVTBIT(start,end) ((0xFFFFFFFFUL >> (31 - start)) & (0xFFFFFFFFUL >>end  << end))

#define REG_CHIPID	(GCR_BA+0x00)	// R	Chip Identification Register
	#define CHIP_ID			NVTBIT(23, 0)	// Chip Identification

#define REG_CHIPCFG	(GCR_BA+0x04)	// R/W	Chip Power-On Configuration Register
	#define HVMODE			BIT17			// ARM vector mode
	#define CAP_SRC		BIT14			// Common mode. Capture0 and Capture1 share the common sensor.
	#define ROT_IRM		BIT13			// IR mode. Rotate Engine's 32KB SRAM is for internal RAM or working memory
	#define DLL_TEST		BIT12			// Enter DLL test mode
	#define DQSWAP		BIT11			// Swap DDRII DQ8/DQ15 and DQ9/DQ14
	#define NTYPE			BIT10			// NAND Type
	#define NPAGE			NVTBIT(9, 8)	// NAND Page Size
	#define NADDR			BIT7 			// NAND Address Cycle
	#define CLK_SRC			BIT6			// System Clock Source Selection
	#define SDRAMSEL		NVTBIT(5, 4)	// SDRAM Type Selection
	#define COPMODE			NVTBIT(3, 0)	// Chip Operation Mode Selection

#define REG_AHBCTL	(GCR_BA+0x10)	// R/W	AHB Bus Arbitration Control Register
	#define IPACT			BIT5 		// Interrupt Active Status
	#define IPEN			BIT4 		// CPU Priority Raising Enable during Interrupt Period
	#define PRTMOD1			BIT1 		// Priority Mode Control 1
	#define PRTMOD0			BIT0 		// Priority Mode Control 0

#define REG_AHBIPRST	(GCR_BA+0x14)	// R/W	AHB IP Reset Control Resister
	#define CRC1_RST		BIT24
	#define SDIO_RST            BIT23
	#define CRC_RST		BIT22
	#define UHC20_RST		BIT21
	#define ROT_RST		BIT20	//GVE Reset
	#define VPE_RST		BIT19	//VPE Reset
	#define VIN1_RST		BIT18	//VIN1 Reset
	#define JPG_RST			BIT17 	// JPG Reset
	#define BLT_RST		BIT16	// BLT Reset
	#define VDE_RST		BIT15 	// H.264 Video Decoder Reset
	#define IPSEC_RST		BIT14	//AES Reset
	#define VEN_RST		BIT13 	// H.264 Video Encoder Reset
	#define AAC_RST		BIT12 	// OVG Reset
	#define VIN0_RST		BIT11 	// VIN 0 Reset
	#define VPOST_RST		BIT10 	// VPOST Reset
	#define I2S_RST			BIT9 		// I2S Reset
	#define SPU_RST		BIT8 		// SPU Reset
	#define UHC_RST		BIT7 		// UHC Reset
	#define UDC_RST		BIT6 		// UDC Reset
	#define SIC_RST			BIT5 		// SIC Reset
	#define TIC_RST			BIT4 		// TIC Reset
	#define EDMA_RST		BIT3 		// EDMA Reset
	#define SRAM_RST		BIT2 		// SRAM Reset
	#define EMAC_RST		BIT1		// EDMA Reset
	#define SDIC_RST		BIT0 		// SDIC Reset

#define REG_APBIPRST	(GCR_BA+0x18)	// R/W	APB IP Reset Control Resister
	#define TMR3RST			BIT17		// Timer 3 Reset
	#define TMR2RST			BIT16		// Timer 2 Reset
	#define KPIRST			BIT15		// KPI Reset
	#define ADCRST			BIT14		// ADC Reset
	#define SPI1RST			BIT13 		// SPI 1 Reset
	#define SPI0RST			BIT12 		// SPI 0 Reset
	#define RSCRST			BIT11 		// RSC Reset
	#define PWMRST			BIT10 		// PWM Reset
	#define RFCRST			BIT9 		// RFC Reset
	#define I2CRST			BIT8 		// I2C Reset

	#define UART1RST			BIT7 		// UART 1 Reset
	#define UART0RST			BIT6 		// UART 0 Reset
	#define TMR1RST			BIT5 		// TMR1 Reset
	#define TMR0RST			BIT4 		// TMR0 Reset
	#define WDTRST			BIT3 		// WDT Reset
	//#define RTCRST			BIT2 		// RTC Reset
	#define GPIORST			BIT1 		// RTC Reset
	#define TOUCHRST		BIT0 		// TOUCH Reset

#define REG_MISCR	(GCR_BA+0x20)	// R/W	Miscellaneous Control Register
	#define WDTRSTEN		BIT24			// WatchDog Timer Reset Connection Enable
	#define UTMISnoop		BIT23			// UTMI Monitor Mode Enable
	#define SEL_HSCUR		NVTBIT(19, 18)	// USB 2.0 PHY Control SEL_HSCUR
	#define SEL_PHASE		NVTBIT(17, 16)	// USB 2.0 PHY Control SEL_PHASE

	#define LVR_RDY			BIT9 			// Low Voltage Reset Function Ready
	#define LVR_EN			BIT8 			// Low Voltage Reset Function Enable
	#define CPURSTON		BIT1 			// CPU always keep in reset state for TIC
	#define CPURST		 	BIT0 			// CPU one shutte reset.

#define REG_SDRBIST	(GCR_BA+0x24)	// R/W	Power Management Control Register
	#define TEST_BUSY		BIT31 			// Test BUSY
	#define CON_BUSY			BIT30			// Connection Test Busy
	#define BIST_BUSY		BIT29			// BIST Test Busy
	#define TEST_FAIL			BIT28 			// Test Failed
	#define CON_FAIL			BIT27			// Connection Test Failed
	#define BIST_FAIL			BIT26 			// BIST Test Failed

#define REG_CRBIST	(GCR_BA+0x28)	// R/W	Cache RAM BIST Control & Status Register
	#define ICV_F			BIT29		// I-Cache Valid RAM BIST Failed Flag
	#define ICT_F			BIT28		// I-Cache Tag RAM BIST Failed Flag
	#define ICD3_F			BIT27		// I-Cache Data RAM 3 BIST Failed Flag
	#define ICD2_F			BIT26		// I-Cache Data RAM 2 BIST Failed Flag
	#define ICD1_F			BIT25		// I-Cache Data RAM 1 BIST Failed Flag
	#define ICD0_F			BIT24		// I-Cache Data RAM 0 BIST Failed Flag
	#define MMU_F			BIT23		// MMU RAM BIST Failed Flag
	#define DCDIR_F			BIT22		// D-Cache Dirty RAM BIST Failed Flag
	#define DCV_F			BIT21		// D-Cache Valid RAM BIST Failed Flag
	#define DCT_F			BIT20		// D-Cache Tag RAM BIST Failed Flag
	#define DCD3_F			BIT19		// D-Cache Data RAM 3 BIST Failed Flag
	#define DCD2_F			BIT18		// D-Cache Data RAM 2 BIST Failed Flag
	#define DCD1_F			BIT17		// D-Cache Data RAM 1 BIST Failed Flag
	#define DCD0_F			BIT16		// D-Cache Data RAM 0 BIST Failed Flag

	#define BISTEN			BIT15		// Cache RAM BIST Test Enable
	#define ICV_R			BIT13		// I-Cache Valid RAM BIST Running Flag
	#define ICT_R			BIT12		// I-Cache Tag RAM BIST Running Flag
	#define ICD3_R			BIT11		// I-Cache Data RAM 3 BIST Running Flag
	#define ICD2_R			BIT10		// I-Cache Data RAM 2 BIST Running Flag
	#define ICD1_R			BIT9		// I-Cache Data RAM 1 BIST Running Flag
	#define ICD0_R			BIT8		// I-Cache Data RAM 0 BIST Running Flag
	#define MMU_R			BIT7		// MMU RAM BIST Running Flag
	#define DCDIR_R			BIT6		// D-Cache Dirty RAM BIST Running Flag
	#define DCV_R			BIT5		// D-Cache Valid RAM BIST Running Flag
	#define DCT_R			BIT4		// D-Cache Tag RAM BIST Running Flag
	#define DCD3_R			BIT3		// D-Cache Data RAM 3 BIST Running Flag
	#define DCD2_R			BIT2		// D-Cache Data RAM 2 BIST Running Flag
	#define DCD1_R			BIT1		// D-Cache Data RAM 1 BIST Running Flag
	#define DCD0_R			BIT0		// D-Cache Data RAM 0 BIST Running Flag

#define REG_EDSSR	(GCR_BA+0x2C)	// R/W	EDMA Service Selection Control Register
#define REG_ED0SSR	(GCR_BA+0x2C)	// R/W	EDMA Service Selection Control Register
	#define CH4_TXSEL		NVTBIT(30, 28)	// EDMA Channel 4 Tx Selection
	#define CH3_TXSEL		NVTBIT(26, 24)	// EDMA Channel 3 Tx Selection
	#define CH2_TXSEL		NVTBIT(22, 20)	// EDMA Channel 2 Tx Selection
	#define CH1_TXSEL		NVTBIT(18, 16)	// EDMA Channel 1 Tx Selection
	#define CH4_RXSEL		NVTBIT(14, 12)	// EDMA Channel 4 Rx Selection
	#define CH3_RXSEL		NVTBIT(10, 8)	// EDMA Channel 3 Rx Selection
	#define CH2_RXSEL		NVTBIT(6, 4)	// EDMA Channel 2 Rx Selection
	#define CH1_RXSEL		NVTBIT(2, 0)	// EDMA Channel 1 Rx Selection

#define REG_ED1SSR	(GCR_BA+0x30)	// R/W	EDMA Service Selection Control Register
	#define CH12_TXSEL		NVTBIT(30, 28)	// EDMA Channel 12 Tx Selection
	#define CH11_TXSEL		NVTBIT(26, 24)	// EDMA Channel 11 Tx Selection
	#define CH10_TXSEL		NVTBIT(22, 20)	// EDMA Channel 10 Tx Selection
	#define CH9_TXSEL		NVTBIT(18, 16)	// EDMA Channel 9 Tx Selection
	#define CH12_RXSEL		NVTBIT(14, 12)	// EDMA Channel 12 Rx Selection
	#define CH11_RXSEL		NVTBIT(10, 8)	// EDMA Channel 11 Rx Selection
	#define CH10_RXSEL		NVTBIT(6, 4)	// EDMA Channel 10 Rx Selection
	#define CH9_RXSEL		NVTBIT(2, 0)	// EDMA Channel 9 Rx Selection

#define REG_MISSR	(GCR_BA+0x34)	// R/W	Miscellaneous Status Register
	#define KPI_WS			BIT31	//KPI Wake-Up Status
	#define ADC_WS			BIT30	//ADC Wake-Up Status
	#define UHC_WS			BIT29	//UHC Wake-Up Status
	#define UDC_WS			BIT28	//UDC Wake-Up Status
	#define UART_WS			BIT27	//UART Wake-Up Status
	#define SDH_WS			BIT26	//SDH Wake-Up Status
	#define RTC_WS			BIT25	//RTC Wake-Up Status
	#define GPIO_WS			BIT24	//GPIO Wake-Up Status
	#define KPI_WE				BIT23	//KPI Wake-Up Enable
	#define ADC_WE			BIT22	//ADC Wake-Up Enable
	#define UHC_WE			BIT21	//UHC Wake-Up Enable
	#define UDC_WE			BIT20	//UDC Wake-Up Enable
	#define UART_WE			BIT19	//UART Wake-Up Enable
	#define SDH_WE			BIT18	//SDH Wake-Up Enable 
	#define RTC_WE			BIT17	//RTC Wake-Up Enable
	#define GPIO_WE			BIT16	//GPIO Wake-Up Enable
	#define UHC20_WS			BIT14	//UHC20 Wake-Up Status
	#define EMAC_WS			BIT15	//EMAC Wake-Up Status
	#define UHC20_WE			BIT7		//UHC20 Wake-Up Enable
	#define EMAC_WE			BIT6		//EMAC Wake-Up Enable
	#define POR12_RST			BIT5		//POR12 Reset Active Status
	#define CPU_RST			BIT4		//CPU RST
	#define WDT_RST			BIT3		//WDT RST
	#define KPI_RST			BIT2		//KPI RST
	#define LVR_RST			BIT1		//LVR RST
	#define EXT_RST			BIT0		//EXT RST
	
#define REG_POR_LVRD	(GCR_BA+0x74)	// R/W	Miscellaneous Status Register
	#define UPLL_LKDT			BIT6		
	#define APLL_LKDT			BIT5		
	#define POR_ENB			BIT4		
	#define EN_LVR				BIT3		
	#define EN_LVD			BIT2		
	#define LVD_SEL			BIT1		
	#define LVD_OUTB			BIT0		

#define REG_GPAFUN0	(GCR_BA+0x80)	// R/W	Multi Function Pin Control Register 0	
	#define MF_GPA7				NVTBIT(31, 28)		// GPA[7] Multi Function
	#define MF_GPA6				NVTBIT(27, 24)		// GPA[6] Multi Function
	#define MF_GPA5				NVTBIT(23, 20)		// GPA[5] Multi Function
	#define MF_GPA4				NVTBIT(19, 16)		// GPA[4] Multi Function
	#define MF_GPA3				NVTBIT(15, 12)		// GPA[3] Multi Function
	#define MF_GPA2				NVTBIT(11, 8)		// GPA[2] Multi Function
	#define MF_GPA1				NVTBIT(7, 4)		// GPA[1] Multi Function
	#define MF_GPA0				NVTBIT(3, 0)		// GPA[0] Multi Function

#define REG_GPAFUN1	(GCR_BA+0x84)	// R/W	Multi Function Pin Control Register 0	
	#define MF_GPA15				NVTBIT(31, 28)		// GPA[7] Multi Function
	#define MF_GPA14				NVTBIT(27, 24)		// GPA[6] Multi Function
	#define MF_GPA13				NVTBIT(23, 20)		// GPA[5] Multi Function
	#define MF_GPA12				NVTBIT(19, 16)		// GPA[4] Multi Function
	#define MF_GPA11				NVTBIT(15, 12)		// GPA[3] Multi Function
	#define MF_GPA10				NVTBIT(11, 8)		// GPA[2] Multi Function
	#define MF_GPA9				NVTBIT(7, 4)		// GPA[1] Multi Function
	#define MF_GPA8				NVTBIT(3, 0)		// GPA[0] Multi Function


#define REG_GPBFUN0	(GCR_BA+0x88)	// R/W	Multi Function Pin Control Register 0
	
	#define MF_GPB7			NVTBIT(31, 28)	// GPB[7] Multi Function
	#define MF_GPB6			NVTBIT(27, 24)	// GPB[6] Multi Function
	#define MF_GPB5			NVTBIT(23, 20)	// GPB[5] Multi Function
	#define MF_GPB4			NVTBIT(19,16)	// GPB[4] Multi Function
	#define MF_GPB3			NVTBIT(15, 12)	// GPB[3] Multi Function
	#define MF_GPB2			NVTBIT(11, 8)	// GPB[2] Multi Function
	#define MF_GPB1			NVTBIT(7, 4)	// GPB[1] Multi Function
	#define MF_GPB0			NVTBIT(3, 0)	// GPB[0] Multi Function

#define REG_GPBFUN1	(GCR_BA+0x8C)	// R/W	Multi Function Pin Control Register 0
	#define MF_GPB15			NVTBIT(31, 28)	// GPB[15] Multi Function
	#define MF_GPB14			NVTBIT(27, 24)	// GPB[14] Multi Function
	#define MF_GPB13			NVTBIT(23, 20)	// GPB[13] Multi Function
	#define MF_GPB12			NVTBIT(19, 16)	// GPB[12] Multi Function
	#define MF_GPB11			NVTBIT(15, 12)	// GPB[11] Multi Function
	#define MF_GPB10			NVTBIT(11, 8)	// GPB[10] Multi Function
	#define MF_GPB9			NVTBIT(7, 4)	// GPB[9] Multi Function
	#define MF_GPB8			NVTBIT(3, 0)	// GPB[8] Multi Function

#define REG_GPCFUN0	(GCR_BA+0x90)	// R/W	Multi Function Pin Control Register 0
	#define MF_GPC7			NVTBIT(31, 28)	// GPC[7] Multi Function
	#define MF_GPC6			NVTBIT(27, 24)	// GPC[6] Multi Function
	#define MF_GPC5			NVTBIT(23, 20)	// GPC[5] Multi Function
	#define MF_GPC4			NVTBIT(19, 16)	// GPC[4] Multi Function
	#define MF_GPC3			NVTBIT(15, 12)	// GPC[3] Multi Function
	#define MF_GPC2			NVTBIT(11, 8)	// GPC[2] Multi Function
	#define MF_GPC1			NVTBIT(7, 4)	// GPC[1] Multi Function
	#define MF_GPC0			NVTBIT(3, 0)	// GPC[0] Multi Function

#define REG_GPCFUN1	(GCR_BA+0x94)	// R/W	Multi Function Pin Control Register 0
	#define MF_GPC15			NVTBIT(31, 28)	// GPC[15] Multi Function
	#define MF_GPC14			NVTBIT(27, 24)	// GPC[14] Multi Function
	#define MF_GPC13			NVTBIT(23, 20)	// GPC[13] Multi Function
	#define MF_GPC12			NVTBIT(19, 16)	// GPC[12] Multi Function
	#define MF_GPC11			NVTBIT(15, 12)	// GPC[11] Multi Function
	#define MF_GPC10			NVTBIT(11, 8)	// GPC[10] Multi Function
	#define MF_GPC9			NVTBIT(7, 4)	// GPC[9] Multi Function
	#define MF_GPC8			NVTBIT(3, 0)	// GPC[8] Multi Function


#define REG_GPDFUN0	(GCR_BA+0x98)	// R/W	Multi Function Pin Control Register 0	
	#define MF_GPD7			NVTBIT(31, 28)	// GPD[7] Multi Function
	#define MF_GPD6			NVTBIT(27, 24)	// GPD[6] Multi Function
	#define MF_GPD5			NVTBIT(23, 20)	// GPD[5] Multi Function
	#define MF_GPD4			NVTBIT(19, 16)	// GPD[4] Multi Function
	#define MF_GPD3			NVTBIT(15, 12)	// GPD[3] Multi Function
	#define MF_GPD2			NVTBIT(11, 8)	// GPD[2] Multi Function
	#define MF_GPD1			NVTBIT(7, 4)	// GPD[1] Multi Function
	#define MF_GPD0			NVTBIT(3, 0)	// GPD[0] Multi Function
	
#define REG_GPDFUN1	(GCR_BA+0x9C)	// R/W	Multi Function Pin Control Register 0		
	#define MF_GPD15			NVTBIT(31, 28)	// GPD[15] Multi Function
	#define MF_GPD14			NVTBIT(27, 24)	// GPD[14] Multi Function
	#define MF_GPD13			NVTBIT(23, 20)	// GPD[13] Multi Function
	#define MF_GPD12			NVTBIT(19, 16)	// GPD[12] Multi Function
	#define MF_GPD11			NVTBIT(15, 12)	// GPD[11] Multi Function
	#define MF_GPD10			NVTBIT(11, 8)	// GPD[10] Multi Function
	#define MF_GPD9			NVTBIT(7, 4)	// GPD[9] Multi Function
	#define MF_GPD8			NVTBIT(3, 0)	// GPD[8] Multi Function	

#define REG_GPEFUN0	(GCR_BA+0xA0)	// R/W	Multi Function Pin Control Register 0	
	#define MF_GPE7			NVTBIT(31, 28)		// GPE[7] Multi Function
	#define MF_GPE6			NVTBIT(27, 24)		// GPE[6] Multi Function
	#define MF_GPE5			NVTBIT(23, 20)		// GPE[5] Multi Function
	#define MF_GPE4			NVTBIT(19, 16)		// GPE[4] Multi Function
	#define MF_GPE3			NVTBIT(15, 12)		// GPE[3] Multi Function
	#define MF_GPE2			NVTBIT(11, 8)		// GPE[2] Multi Function
	#define MF_GPE1			NVTBIT(7, 4)		// GPE[1] Multi Function
	#define MF_GPE0			NVTBIT(3, 0)		// GPE[0] Multi Function
	
#define REG_GPEFUN1	(GCR_BA+0xA4)	// R/W	Multi Function Pin Control Register 0		
	#define MF_GPE15			NVTBIT(31, 28)		// GPE[15] Multi Function
	#define MF_GPE14			NVTBIT(27, 24)		// GPE[14] Multi Function
	#define MF_GPE13			NVTBIT(23, 20)		// GPE[13] Multi Function
	#define MF_GPE12			NVTBIT(19, 16)		// GPE[12] Multi Function
	#define MF_GPE11			NVTBIT(15, 12)		// GPE[11] Multi Function
	#define MF_GPE10			NVTBIT(11, 8)		// GPE[10] Multi Function
	#define MF_GPE9			NVTBIT(7, 4)		// GPE[9] Multi Function
	#define MF_GPE8			NVTBIT(3, 0)		// GPE[8] Multi Function	

#define REG_MISFUN		(GCR_BA+0xA8)	// R/W	Miscellaneous Multi Function Control Register
	#define MF_I2S				BIT0				// I2S I/F Functional Selection

#define REG_SPI_LVD_GPH 	(GCR_BA+0xAC)	// R/W	PI[3:2] And LVDATA[23:18] (GPIO H) Control Register
	#define DS_SPI_GPH			NVTBIT(23, 22)		// SPI_DATA[3:2] (GPIO_H[7:6]) Pin Driver Strength Control
	#define DS_LVD_GPH		NVTBIT(21, 16)		// LVDATAT[23:18] (GPIO_H[5:0]) Pin Driver Strength Control
	#define SL_SPI_GPH			NVTBIT(7, 6)		// SPI_DATA[3:2] (GPIO_H[7:6]) Pin Slew Rate Control
	#define SL_LVD_GPH		NVTBIT(5, 0)		// LVDATA[23:18] (GPIO_H[5:0]) Pin Slew Rate Control

#define REG_MISCPCR	(GCR_BA+0xB0)	// R/W	Miscellaneous Pin Control Register
	#define SL_MD			BIT7				// MD Pin Slew Rate Control
	#define SL_MA			BIT6				// MA Pin Slew Rate Control
	#define SL_MCTL		BIT5				// Memory I/F Control Pin Slew Rate Control
	#define SL_MCLK		BIT4				// MCLK Pin Rate Control
	#define DS_MD			BIT3				// MD Pins Driving Strength Control
	#define DS_MA			BIT2				// MA Pins Driving Strength Control
	#define DS_MCTL			BIT1				// MCTL Pins Driving Strength Control
	#define DS_MCLK			BIT0				// MCLK Pins Driving Strength Control

#define REG_MISC_SL_GPA	(GCR_BA+0xB4)	// R/W	GPIO A Slew Rate Control Register
#define REG_MISC_SL_GPB	(GCR_BA+0xB8)	// R/W	GPIO B Slew Rate Control Register
#define REG_MISC_SL_GPC	(GCR_BA+0xBC)	// R/W	GPIO C Slew Rate Control Register
#define REG_MISC_SL_GPD	(GCR_BA+0xC0)	// R/W	GPIO D Slew Rate Control Register
#define REG_MISC_SL_GPE	(GCR_BA+0xC4)	// R/W	GPIO E Slew Rate Control Register
#define REG_MISC_SL_ND	(GCR_BA+0xC8)	// R/W  NAND Data PAD Slew Rate Control Register
	#define SL_ND			NVTBIT(7, 0)		// NAND Data Pin Slew Rate Control

#define REG_MISC_DS_GPA	(GCR_BA+0xCC)	// R/W	GPIO A Driver Strength Control Register
#define REG_MISC_DS_GPB	(GCR_BA+0xD0)	// R/W	GPIO B Driver Strength Control Register
#define REG_MISC_DS_GPC	(GCR_BA+0xD4)	// R/W	GPIO C Driver Strength Control Register
#define REG_MISC_DS_GPD	(GCR_BA+0xD8)	// R/W	GPIO D Driver Strength Control Register
#define REG_MISC_DS_GPE	(GCR_BA+0xDC)	// R/W	GPIO E Driver Strength Control Register
#define REG_MISC_DS_ND	(GCR_BA+0xE0)	// R/W  NAND Data PAD Driver Strength Control Register
	#define DS_ND			NVTBIT(7, 0)		// NAND Data Pin Driver Strength Control

#define REG_MISC_SSEL	(GCR_BA+0xE4)
	#define SDRAM_MCLK		NVTBIT(9, 8)		// MCLK Pins Driving Strength Control and Mode.
	#define SDRAM_MA		NVTBIT(7, 6)		// MA Pins Driving Strength Control and Mode.
	#define SDRAM_MBA		NVTBIT(5, 4)		// MBA Pins Driving Strength  Control and Mode.
	#define SDRAM_MCTL		NVTBIT(3, 2)		// MCTL Pins Driver Strength Control and Mode.
	#define SDRAM_MD		NVTBIT(1, 0)		// MD Data Pins Driver Strength Control and Mode

#define REG_GPGFUN0	(GCR_BA+0xE8)	// R/W	Multi Function Pin Control Register 0	
	#define MF_GPG7			NVTBIT(31, 28)	// GPG[7] Multi Function
	#define MF_GPG6			NVTBIT(27, 24)	// GPG[6] Multi Function
	#define MF_GPG5			NVTBIT(23, 20)	// GPG[5] Multi Function
	#define MF_GPG4			NVTBIT(19, 16)	// GPG[4] Multi Function
	#define MF_GPG3			NVTBIT(15, 12)	// GPG[3] Multi Function
	#define MF_GPG2			NVTBIT(11, 8)	// GPG[2] Multi Function
	#define MF_GPG1			NVTBIT(7, 4)	// GPG[1] Multi Function
	#define MF_GPG0			NVTBIT(3, 0)	// GPG[0] Multi Function
	
#define REG_GPGFUN1	(GCR_BA+0xEC)	// R/W	Multi Function Pin Control Register 0		
	#define MF_GPG15			NVTBIT(31, 28)	// GPG[15] Multi Function
	#define MF_GPG14			NVTBIT(27, 24)	// GPG[14] Multi Function
	#define MF_GPG13			NVTBIT(23, 20)	// GPG[13] Multi Function
	#define MF_GPG12			NVTBIT(19, 16)	// GPG[12] Multi Function
	#define MF_GPG11			NVTBIT(15, 12)	// GPG[11] Multi Function
	#define MF_GPG10			NVTBIT(11, 8)	// GPG[10] Multi Function
	#define MF_GPG9			NVTBIT(7, 4)	// GPG[9] Multi Function
	#define MF_GPG8			NVTBIT(3, 0)	// GPG[8] Multi Function

#define REG_GPHFUN	(GCR_BA+0xF0)	// R/W	Dummy GPIO 
	#define MF_GPH15			NVTBIT(31, 30)		// GPH[15] Multi Function
	#define MF_GPH14			NVTBIT(29, 28)		// GPH[14] Multi Function
	#define MF_GPH13			NVTBIT(27, 26)		// GPH[13] Multi Function
	#define MF_GPH12			NVTBIT(25, 24)		// GPH[12] Multi Function
	#define MF_GPH11			NVTBIT(23, 22)		// GPH[11] Multi Function
	#define MF_GPH10			NVTBIT(21, 20)		// GPH[10] Multi Function
	#define MF_GPH9			NVTBIT(19, 18)		// GPH[9] Multi Function
	#define MF_GPH8			NVTBIT(17, 16)		// GPH[8] Multi Function
	#define MF_GPH7			NVTBIT(15, 14)		// GPH[7] Multi Function
	#define MF_GPH6			NVTBIT(13, 12)		// GPH[6] Multi Function
	#define MF_GPH5			NVTBIT(11, 10)		// GPH[5] Multi Function
	#define MF_GPH4			NVTBIT(9, 8)		// GPH[4] Multi Function
	#define MF_GPH3			NVTBIT(7, 6)		// GPH[3] Multi Function
	#define MF_GPH2			NVTBIT(5, 4)		// GPH[2] Multi Function
	#define MF_GPH1			NVTBIT(3, 2)		//GPH[1] Multi Function
	#define MF_GPH0			NVTBIT(1, 0)		// GPH[0] Multi Function
	
#define REG_SHRPIN_TVDAC	(GCR_BA+0xF4)	// R/W	Share Pin With TV DAC
	#define SMTVDACAEN		BIT31	

#define REG_SHRPIN_AUDIO	(GCR_BA+0xF8)	// R/W	Share Pins with AUDIO ADC
	#define MIC_AEN			BIT31			// Analog and Digital Share I/O Pad Control Bit for SAR-ADC MICP, MICN
	#define AIN2_AEN			BIT30			// Analog and Digital Share I/O Pad Control Bit for SAR-ADC AIN2 Pins
	#define AIN3_AEN			BIT29			// Analog and Digital Share I/O Pad Control Bit for SAR-ADC AIN3 Pins
	#define AIN4_AEN			BIT28			// Analog and Digital Share I/O Pad Control Bit for SAR-ADC AIN4 Pins

#define REG_SHRPIN_TOUCH	(GCR_BA+0xFC)	// R/W	Share Pins with TOUCH ADC
	#define SAR_AHS_AEN		BIT31			// Analog and Digital Share I/O Pad Control Bit for SAR-ADC AHS Pins.
	#define TP_AEN				BIT30			// Analog and Digital Share I/O Pad Control Bit for SAR-ADC XP, XM, YP, YM Pins
	#define MIC_BIAS_AEN		BIT29			// Analog and Digital Share I/O Pad Control Bit for SAR-ADC MIC_BIAS Pin

#define REG_EXTRST_DEBOUNCE	(GCR_BA+0x100)	// R/W	External RESET Debounce Control Register	
	#define EXTRST_DEBOUNCE		BIT0				// External RESET Debounce Control

#define REG_DEBOUNCE_CNTR	(GCR_BA+0x104)	// R/W	 External RESET Debounce Counter Register	
	#define DEBOUNCE_CNTR		NVTBIT(15, 0)				// External RESET Debounce Counter
				

/*
	Clock controller Registers
*/
#define REG_PWRCON	(CLK_BA+0x00)	// R/W	System Power Down Control Register
	#define PRE_SCALAR			NVTBIT(23, 8)		// Pre-Scalar counter
	#define SEN1_OFF_ST		BIT5				// Sensor 1 clock level if clock off state
	#define SEN0_OFF_ST		BIT4				// Sensor 0 clock level if clock off state
	#define INT_EN				BIT3				// Power On Interrupt Enable
	#define INTSTS				BIT2				// Power Down interrupt status
	#define XIN_CTL			BIT1				// Crystal pre-divide control for Wake-up from power down mode
	#define XTAL_EN			BIT0				// Crystal (Power Down) Control

#define REG_AHBCLK	(CLK_BA+0x04)	// R/W¡@AHB Clock Enable Control Register
	#define SDIO_CKE        BIT31			// SDIO Controller Engine Clock, 0 = Disable, 1 = Enable
	#define ADO_CKE			BIT30			// Audio DAC Engine Clock Enable Control0 = Disable1 = Enable
	#define SEN_CKE			BIT29			// Sensor Interface Clock Enable Control0 = Disable1 = Enable
	#define SEN0_CKE			SEN_CKE
	#define VIN_CKE			BIT28			// Capture Clock Enable Control (Also is Capture engine clock enable control)0 = Disable1 = Enable
	#define VIN0_CKE			VIN_CKE
	#define VPOST_CKE			BIT27			// VPOST Clock Enable Control (Also is VPOST engine clock enable control)0 = Disable1 = Enable
	#define I2S_CKE			BIT26			// I2S Controller Clock Enable Control0 = Disable1 = Enable
	#define SPU_CKE			BIT25			// SPU Clock Enable Control0 = Disable1 = Enable
	#define HCLK4_CKE			BIT24			// HCLK4 Clock Enable Control0 = Disable1 = Enable
	#define SD_CKE			BIT23			// SD Card Controller Engine Clock Enable Control0 = Disable1 = ENable
	#define NAND_CKE			BIT22			// NAND Controller Clock Enable Control0 = Disable1 = ENable
	#define SIC_CKE			BIT21			// SIC Clock Enable Control0 = Disable1 = ENable
	//#define OVG_CKE			BIT20			// Graphic Processing Unit Clock Enable Control0 = Disable1 = ENable
	#define GVE_CKE			BIT19			// GE4P Clock Enable Control0 = Disable1 = ENable
	#define USBD_CKE			BIT18			// USB Device Clock Enable Control0 = Disable1 = Enable
	#define USBH_CKE			BIT17			// USB Host Controller Clock Enable Control0 = Disable1 = Enable
	#define HCLK3_CKE			BIT16			// HCLK3 Clock Enable Control.0 = Disable1 = Enable
	#define VDE_CKE			BIT15			// Video Decoder Clock Enable Control
	#define EDMA4_CKE			BIT14			// EDMA Controller Channel 4 Clock Enable Control
	#define EDMA3_CKE			BIT13			// EDMA Controller Channel 3 Clock Enable Control
	#define EDMA2_CKE			BIT12			// EDMA Controller Channel 2 Clock Enable Control
	#define EDMA1_CKE			BIT11			// EDMA Controller Channel 1 Clock Enable Control
	#define EDMA0_CKE			BIT10			// EDMA Controller Channel 0 Clock Enable Control
	#define IPSEC_CKE			BIT9				// AES Clock Enable Control. 0 = Disable; 1 = Enable.
	#define HCLK1_CKE			BIT8				// HCLK1 Clock Enable Control.0 = Disable1 = Enable
	#define JPG_CKE			BIT7				// JPEG Clock Enable
	#define VPE_CKE			BIT6				// Video Engine Clock Enable Control
	#define BLT_CKE			BIT5				// Bitblt Engine Clock Enable Control
	#define DRAM_CKE			BIT4				// SDRAM and SDRAM Controller Clock Enable Control.0 = Disable1 = Enable
	#define SRAM_CKE			BIT3				// SRAM Controller Clock Enable Control.0 = Disable1 = Enable
	#define HCLK_CKE			BIT2				// HCLK Clock Enable Control. (This clock is used for DRAM controller, SRAM controller and AHB-to-AHB bridge)0 = Disable1 = Enable
	#define APBCLK_CKE			BIT1				// APB Clock Enable Control.0 = Disable1 = Enable
	#define CPU_CKE	 		BIT0				// CPU Clock Enable Control

#define REG_APBCLK	(CLK_BA+0x08)	// R/W¡@APB Clock Enable Control Register
	#define KPI_CKE			BIT25 			// KPI Clock Enable Control
	#define TIC_CKE			BIT24 			// TIC Clock Enable
	#define TMR3_CKE			BIT17		//Timer3 Clock Enable Control
	#define TMR2_CKE			BIT16		//Timer2 Clock Enable Control
	#define WDCLK_CKE			BIT15			// Watch Dog Clock Enable Control (Also is Watch Dog engine clock enable control)
	#define RSC_CKE			BIT12			// Reed-Solomon Codec Clock Enable Control
	#define CONVENC_CKE		BIT11			// Convolution Encoder Clock Enable Control
	#define TOUCH_CKE			BIT10			// TOUCH Clock Enable Control
	#define TMR1_CKE			BIT9				// Timer1 Clock Enable Control0 = Disable1 = Enable
	#define TMR0_CKE			BIT8				// Timer0 Clock Enable Control0 = Disable1 = Enable
	#define SPIMS1_CKE			BIT7				// SPIM (Master Only) Clock Enable Control0 = Disable1 = Enable
	#define SPIMS0_CKE			BIT6				// SPIMS (Master / Slave) Clock Enable Control0 = Disable1 = Enable
	#define PWM_CKE			BIT5				// PWM Clock Enable Control0 = Disable1 = Enable
	#define UART1_CKE			BIT4				// UART1 Clock Enable Control0 = Disable1 = Enable
	#define UART0_CKE			BIT3				// UART0 Clock Enable Control0 = Disable1 = Enable
	#define RTC_CKE			BIT2				// RTC Clock Enable Control (NOT X32K clock enable control)0 = Disable1 = Enable
	#define I2C_CKE			BIT1				// I2C Clock Enable Control0 = Disable1 = Enable
	#define ADC_CKE			BIT0				// ADC Clock Enable Control (Also is ADC engine clock enable control)0 = Disable1 = Enable

#define REG_CLKDIV0	(CLK_BA+0x0C)	// R/W¡@Clock Divider Register0
	//#define SYSTEM_INTEG		NVTBIT(31, 29)		// Sensor clock divide number from sensor clock source
	//#define SYSTEM_FRACTM		NVTBIT(28, 27)		// Sensor clock divide number from sensor clock source
	#define SENSOR0_N1		NVTBIT(27, 24)		// Sensor clock divide number from sensor clock source
	#define SENSOR0_S			NVTBIT(23, 22)		// Sensor clock source select 00 = XIN. 01 = X32K. 10 = APLL. 11 = UPLL
	#define SENSOR0_N0		NVTBIT(21, 19)		// Sensor clock pre-divider number from Sensor clock source if Sensor clock source select is APLL or UPLL
	#define KPI_N0				NVTBIT(18, 12)		// KPI Engine Clock Divider Bits [3:0]
	#define SYSTEM_FRACTL		NVTBIT(11, 8)		// SYSTEM clock divide number from system clock source
	#define SYSTEM_N1			SYSTEM_FRACTL	// SYSTEM clock divide number from system clock source
	#define KPI_S				BIT5				// KPI Engine Clock Source Selection
	#define SYSTEM_S			NVTBIT(4, 3)		// System clock source select 00 = XIN. 01 = X32K. 10 = APLL. 11 = UPLL
	#define SYSTEM_N0			NVTBIT(2, 0)		// SYSTEM clock pre-divider number from system clock source if System clock source select is APLL or UPLL

#define REG_CLKDIV1	(CLK_BA+0x10)	// R/W¡@Clock Divider Register1
	#define ADO_N1			NVTBIT(31, 24)		// Audio DAC engine clock divide number from Audio DAC engine clock source
	#define ADO_S				NVTBIT(20, 19)		// Audio DAC engine clock source select 00 = XIN. 01 = X32K. 10 = APLL. 11 = UPLL
	#define ADO_N0			NVTBIT(18, 16)		// Audio DAC engine clock pre-divide number
	#define VPOST_N1			NVTBIT(15, 8)		// VPOST engine clock divide number from VPOST engine clock source
	#define VPOST_S			NVTBIT(4, 3)		// VPOST engine clock source select 00 = XIN. 01 = X32K. 10 = APLL. 11 = UPLL
	#define VPOST_N0			NVTBIT(2, 0)		// VPOST engine clock pre-divide number

#define REG_CLKDIV2	(CLK_BA+0x14)	// R/W¡@Clock Divider Register2
	#define SD_N1				NVTBIT(31, 24)		// SD engine clock divide number from SD engine clock source
	#define SD_S				NVTBIT(20, 19)		// SD engine clock source select  00 = XIN. 01 = X32K. 10 = APLL. 11 = UPLL
	#define SD_N0				NVTBIT(18, 16)		// SD engine clock pre-divide number
	#define U20PHY_SS			NVTBIT(22, 21)		// USB Device
	#define U20PHY_N			NVTBIT(15, 12)		// USB Device
	#define USB_N1			NVTBIT(11, 8)		// USB host engine clock divide number from USB engine clock source
	#define U20PHY_DS			NVTBIT(7, 5)		// USB Device
	#define USB_S				NVTBIT(4, 3)		// USB host engine clock source select 00 = XIN. 01 = X32K. 10 = APLL. 11 = UPLL
	#define USB_N0			NVTBIT(2, 0)		// USB host engine clock Pre-divide number

#define REG_CLKDIV3	(CLK_BA+0x18)	// R/W¡@Clock Divider Register3
	#define ADC_N1			NVTBIT(31, 24) 	// ADC engine clock divide number from ADC engine clock source
	#define ADC_S				NVTBIT(20, 19) 	// ADC engine clock source select 00 = XIN. 01 = X32K. 10 = APLL. 11 = UPLL
	#define ADC_N0			NVTBIT(18, 16) 	// ADC engine clock pre-divide number from ADC engine clock source
	#define UART1_N1			NVTBIT(15, 13) 	// UART1 engine clock divide number from UART1 engine clock source
	#define UART1_S			NVTBIT(12, 11)		// UART1 engine clock source select 00 = XIN. 01 = X32K. 10 = APLL. 11 = UPLL
	#define UART1_N0			NVTBIT(10, 8)  		// UART1 engine clock pre-divide number from UART1 engine clock source
	#define UART0_N1			NVTBIT(7, 5) 		// UART0 engine clock divide number from UART1 engine clock source
	#define UART0_S			NVTBIT(4, 3) 		// UART0 engine clock source select 00 = XIN. 01 = X32K. 10 = APLL. 11 = UPLL
	#define UART0_N0			NVTBIT(2, 0)   		// UART0 engine clock pre-divide number from UART1 engine clock source

#define REG_CLKDIV4	(CLK_BA+0x1C)	// R/W¡@Clock Divider Register4
	#define VIN1_N				NVTBIT(29, 27)		// JPG 	engine clock divide number from HCLK4
	#define JPG_N				NVTBIT(26, 24)		// JPG 	engine clock divide number from HCLK3
	#define GPIO_N			NVTBIT(23, 17)		// GPIO engine clock divide number from GPIO engine clock source
	#define GPIO_S				BIT16 			// GPIO engine clock source select0 = XIN1 = X32K
	#define CAP_N				NVTBIT(14, 12)		// Capture engine clock divide number from HCLK4 clock.Engine Clock frequency = HCLK4 / (CAP_N + 1)
	#define VIN0_N				CAP_N
	#define CHG_APB			BIT11
	#define APB_N				NVTBIT(10, 8) 		// APB clock divide number from HCLK1 clock. The HCLK1 clock frequency is the lower of system clock divided by 2 or the CPU clockThe APB clock frequency = (HCLK1 frequency) / (APB_N + 1)
	#define HCLK234_N			NVTBIT(7, 4)		// HCLK2, HCLK3 and HCLK4 clock divide number from HCLK clock. The HCLK clock frequency is the system clock frequency divided by two.The HCLK2,3,4 clock frequency = (HCLK frequency) / (HCLK234_N + 1)
	#define CPU_N				NVTBIT(3, 0)		// CPU clock divide number from System clock.The CPU clock frequency = (System frequency) / (CPU_N + 1)

#define REG_APLLCON	(CLK_BA+0x20)	// R/W	APLL Control Register
	#define BP				BIT15			// PLL By Pass Control
	#define PD				BIT14			// Power Down Mode
	#define OUT_DV			NVTBIT(12, 11)		// PLL Output Divider Control
	#define IN_DV				NVTBIT(10, 7)		// PLL Input Divider Control
	#define FB_DV				NVTBIT(6, 0)		// PLL Feedback Divider Control
 
#define REG_UPLLCON	(CLK_BA+0x24)	// R/W	UPLL Control Register
#define REG_MPLLCON	(CLK_BA+0x28)	// R/W	MPLL Control Register

#define REG_CLK_TREG	(CLK_BA+0x30)
	#define TEST_CKE			BIT7				// Test Colck Output Enable.
	#define SW_CLK				BIT6			// Software Generated Clock.
	#define TEST_CLK_SEL		NVTBIT(5, 0)		// Test Clock Select.

#define REG_AHBCLK2	(CLK_BA+0x34)	// R/W¡@AHB Clock Enable Control Register2
	#define CRC1_CKE				BIT19			// CRC1 Clock Enable Control
	#define ROTE_CKE				BIT18			// Rotate Engine Clock Enable Control
	#define EDMA26_CKE			BIT17			// EDMA#2 Controller Channel 6 Clock Enable Control
	#define EDMA25_CKE			BIT16			// EDMA#2 Controller Channel 5 Clock Enable Control
	#define EDMA24_CKE			BIT15			// EDMA#2 Controller Channel 4 Clock Enable Control
	#define EDMA23_CKE			BIT14			// EDMA#2 Controller Channel 3 Clock Enable Control
	#define EDMA22_CKE			BIT13			// EDMA#2 Controller Channel 2 Clock Enable Control
	#define EDMA21_CKE			BIT12			// EDMA#2 Controller Channel 1 Clock Enable Control
	#define EDMA20_CKE			BIT11			// EDMA#2 Controller Channel 0 Clock Enable Control
	#define VENC_CKE            BIT10           // H.264 Encoder clock enable
	#define VDEC_CKE            BIT9            // H.264 Decoder clock enable
	#define CRC_CKE				BIT6				// CRC Clock Enable Control
	#define AAC_CKE				BIT8				// AAC Controller Clock enable control	
	#define SEN1_CKE			BIT3				// EDMA Controller Channel 6 Clock Enable Control
	#define VIN1_CKE			BIT2				// EDMA Controller Channel 5 Clock Enable Control
	#define EDMA6_CKE			BIT1				// EDMA Controller Channel 6 Clock Enable Control
	#define EDMA5_CKE			BIT0				// EDMA Controller Channel 5 Clock Enable Control

#define REG_CLKDIV5	(CLK_BA+0x38)	// R/W¡@AHB Clock Enable Control Register2
	#define TOUCH_N1			NVTBIT(31, 27)		// TOUCH Engine Clock Divider Bits [4:0]
	#define TOUCH_S			NVTBIT(26, 25)		// TOUCH Engine Clock Source Selectione Control
	#define TOUCH_N0			NVTBIT(24, 22)		// TOUCH Engine Clock Divider If PLL
	#define SENSOR1_N1		NVTBIT(21, 18)		// Sensor Clock Divider Bits [4:0]
	#define SENSOR1_S			NVTBIT(17, 16)		// Sensor Clock Source Selectione Control
	#define SENSOR1_N0		NVTBIT(15, 13)		// Sensor Clock Divider If PLL
	#define PWM_N1			NVTBIT(12, 5)		// PWM Clock Divider Bits [4:0]
	#define PWM_S				NVTBIT(4, 3)		// PWM Clock Source Selectione Control
	#define PWM_N0			NVTBIT(2, 0)		// PWM Clock Divider If PLL

#define REG_CLKDIV6	(CLK_BA+0x3C)	
#define REG_CLKDIV7	(CLK_BA+0x40)
	#define I2S_N1			NVTBIT(23, 16)		// I2S engine clock divide number 1
	#define I2S_S				NVTBIT(12, 11)		// I2S engine clock source select 00 = XIN. 01 = X32K. 10 = APLL. 11 = UPLL
	#define I2S_N0			NVTBIT(10, 8)		// I2S engine clock pre-divide number
	#define DRAM_N1			NVTBIT(7, 5)		// DRAM Clock Divider 1
	#define DRAM_S			NVTBIT(4, 3)		// DRAM Clock
	#define DRAM_N0			NVTBIT(2, 0)		// DRAM Clock Divider 0

#define REG_CLKDIV8 	(CLK_BA+0x44)				// R/W¡@Clock Divider Register2
	#define SDIO_N1				NVTBIT(12, 5)		// SD engine clock divide number from SD engine clock source
	#define SDIO_S				NVTBIT(4, 3)		// SD engine clock source select  00 = XIN. 01 = X32K. 10 = APLL. 11 = UPLL
	#define SDIO_N0				NVTBIT(2, 0)		// SD engine clock pre-divide number
	
/*
	Rotation engine controller registers
*/	
#define REG_SCCR	(ROT_BA+0x00)		// R/W¡@SRAM Controller Control Register
	#define ROTSW_RST				BIT0				// SRAM Controller software reset
	
#define REG_RCR	(ROT_BA+0x10)		// R/W¡@Control Register
	#define IMG_DFMT				NVTBIT(7, 6)		// Image Data Format
	#define LINE_BUF_SIZE			NVTBIT(5, 4)		// Line Buffer Size
	#define ROTE_DIR				BIT1				// Rotation Direction
	#define ROTE_EN				BIT0				// Rotation Enable
	
#define REG_RICR	(ROT_BA+0x14)		// R/W¡@Interrupt Control Register
	#define SRAM_OF_EN			BIT20				// SRAM read/write overflow Interrupt Enable
	#define TG_ABORT_EN			BIT18				// Target Abort Interrupt Enable
	#define ROTE_INT_EN			BIT16				// Rotation Finished Interrupt Enable
	#define SRAM_OF				BIT4				// SRAM read/write overflow
	#define TG_ABORT			        BIT2				// EAHB Master Receive Target Abort
	#define ROTE_FINISH			BIT0				// Rotation Finished
	
#define REG_RIS		(ROT_BA+0x18)		// R/W¡@Rotation Image Size
	#define ROT_HEIGHT				NVTBIT(27, 16)		// Height of rotation image size (must be even number)
	#define ROT_WIDTH				NVTBIT(11, 0)		// Width of rotation image size (must be even number)
	
#define REG_RSISA	(ROT_BA+0x1C)		// R/W¡@Source Image Starting Address
#define REG_RSILOFF	(ROT_BA+0x20)		// R/W¡@Source Image Line Offset
	#define RSI_LOFF				NVTBIT(11, 0)		// Rotation Source Image Line Offset (right-offset by byte-unit)
#define REG_RDISA	(ROT_BA+0x24)		// R/W¡@Destination Image Starting Address
#define REG_RDILOFF	(ROT_BA+0x2C)		// R/W¡@Destination Image Line Offset
	#define RDI_LOFF				NVTBIT(11, 0)		// Rotation Destination  Image Line Offset (right-offset by byte-unit)

/*
	SDRAM controller registers
*/
#define REG_SDOPM 	(SDRAM_BA + 0x00)	// R/W	SDRAM Controller Operation Mode Control Register
		#define AUTO_DQSPHASE	BIT26
		#define RD2WR_CTL		BIT20			// Read-To-Write Turn Around Control
		#define OEDELAY		BIT19			// Output Enable Delay Half MCLK
		#define LOWFREQ		BIT18			// Low Frequency Mode
		#define PREACTBNK     	BIT17			// Pre-Active Bank
		#define AUTOPDN       	BIT16			// Auto Power Down Mode
		#define SEL_SCLKI            BIT15
		#define RDBUFTH       	NVTBIT(10, 8)		// The AHB read SDRAM read buffer threshold control
		#define SD_TYPE       	NVTBIT(6, 5)		// SDRAM Type
		#define PCHMODE      	BIT4				// SDRAM Type
		#define OPMODE        	BIT3				// Open Page Mode
		#define MCLKMODE      	BIT2				// Auto Pre-Charge Mode
		#define DRAM_EN      	BIT1				// SDRAM Controller Enable

#define REG_SDCMD 	(SDRAM_BA + 0x04)	// R/W	SDRAM Command Register
		#define AUTOEXSELFREF	BIT5				// Auto Exit Self-Refresh
		#define SELF_REF       	BIT4				// Self-Refresh Command
		#define REF_CMD       	BIT3				// Auto Refresh Command
		#define PALL_CMD       	BIT2				// Pre-Charge All Bank Command
		#define CKE_H           	BIT1    			// CKE High
		#define INITSTATE		BIT0				// Initial State
#define REG_SDREF 	(SDRAM_BA + 0x08)	// R/W	SDRAM Controller Refresh Control Register
		#define REF_EN			BIT15			// Refresh Period Counter Enable
		#define REFRATE		NVTBIT(14, 0)		// Refresh Count Value
#define REG_SDSIZE0	(SDRAM_BA + 0x10)	// R/W	SDRAM 0 Size Register
#define REG_SDSIZE1	(SDRAM_BA + 0x14)	// R/W	SDRAM 0 Size Register
		#define BASADDR		NVTBIT(28, 21) 	// Base Address
		#define BUSWD			BIT3				// SDRAM Data Bus width
		#define DRAMSIZE		NVTBIT(2, 0)		// Size of SDRAM Device

#define REG_SDMR		(SDRAM_BA + 0x18)	// R/W	SDRAM Mode Register
		#define SDMR_CONFIGURE	NVTBIT(17, 7) // SDRAM Dependent Configuration
		#define LATENCY			NVTBIT(6, 4) 	// CAS Latency
		#define BRSTTYPE			BIT3			// Burst Type
		#define BRSTLENGTH		NVTBIT(2, 0) 	// Burst Length
#define REG_SDEMR	(SDRAM_BA + 0x1C)	// R/W	SDRAM Extended Mode Register
		#define SDEMR_CONFIGURE	NVTBIT(17, 2) 	// SDRAM Dependent Configuration
		#define DRVSTRENGTH		BIT1 			// Output Drive Strength
		#define DLLEN				BIT0			// DLL Enable
#define REG_SDEMR2	(SDRAM_BA + 0x20)	// R/W	SDRAM Extended Mode Register 2
		#define SDEMR2_MR_DEF		NVTBIT(17, 15) 	// Mode Register Definition
		#define SDEMR2_CONFIGURE	NVTBIT(13, 0)		// SDRAM Dependent Configuration
#define REG_SDEMR3 (SDRAM_BA+ 0x24)	// R/W	SDRAM Extended Mode Register 3
		#define SDEMR3_MR_DEF		NVTBIT(17, 15) 	// Mode Register Definition
		#define SDEMR3_CONFIGURE	NVTBIT(13, 0)	  	// SDRAM Dependent Configuration
#define REG_SDTIME		(SDRAM_BA + 0x28)	// R/W	SDRAM Timing Control Register
		#define TWTR				NVTBIT(30, 29) 	// Internal Write to Read Command Delay
		#define TRRD				NVTBIT(28, 27) 	// Active Bank a to Active Bank b Command Delay
		#define TRC				NVTBIT(26, 22) 	// Active to Active Command Delay
		#define TXSR				NVTBIT(21, 17) 	// Exit SELF REFRESH to ACTIVE Command Delay
		#define TRFC				NVTBIT(16, 12) 	// AUTO REFRESH Period
		#define TRAS				NVTBIT(11, 8) 		// ACTIVE to PRECHARGE Command Delay
		#define TRCD				NVTBIT(7, 5) 		// Active to READ or WRITE Delay
		#define TRP				NVTBIT(4, 2) 		// PRECHARGE Command Period
		#define TWR 				NVTBIT(1, 0) 		// WRITE Recovery Time
#define REG_DQSODS	(SDRAM_BA + 0x30)	// R/W	DQS Output Delay Selection Register
		#define DQSINVEN			BIT13 			// DQS Invert Enable
		#define DQS1_ODS			NVTBIT(12, 8) 		// DQS1 Output Delay Selection
		#define DQS0_ODS			NVTBIT(4, 0) 		// DQS0 Output Delay Selection
#define REG_CKDQSDS	(SDRAM_BA + 0x34)	// R/W	Clock and DQS Delay Selection Register
		#define DQS1_DS1			NVTBIT(23, 20) 	// DQS1 Input Delay Selection 1
		#define DQS1_DS0			NVTBIT(19, 16) 	// DQS1 Input Delay Selection 0
		#define DQS0_DS1			NVTBIT(15, 12) 	// DQS0 Input Delay Selection 1
		#define DQS0_DS0			NVTBIT(11, 8) 		// DQS0 Input Delay Selection 0
		#define DCLK_DS			NVTBIT(7, 4) 		// Data Clock Delay Selection
		#define DCLKSRCSEL			BIT3 				// Data Clock Source Selection
		#define MCLK_ODS			NVTBIT(2, 0) 		// MCLK Output Delay Selection
#define REG_TESTCR	(SDRAM_BA + 0x40)	// R/W	SDRAM test control register
		#define STATUS_CLR			BIT31 			// Test Status Clear
		#define TEST_EN			BIT30			// Connection Test Enable
		#define BIST_EN			BIT29 			// SDRAM BIST Enable
		#define MARCH_C			BIT28			// MARCH_C Algorithm Used
		#define MAX_ADDR			NVTBIT(26, 0) 		// Maximum Test Address
#define REG_TSTATUS	(SDRAM_BA + 0x44)	// R	SDRAM test status register
		#define TEST_BUSY			BIT31 			// Test BUSY
		#define CON_BUSY			BIT30			// Connection Test Busy
		#define BIST_BUSY			BIT29			// BIST Test Busy
		#define TEST_FAIL			BIT28 			// Test Failed
		#define CON_FAIL			BIT27			// Connection Test Failed
		#define BIST_FAIL			BIT26 			// BIST Test Failed
#define REG_TFDATA		(SDRAM_BA+ 0x48)	// R	SDRAM test fail data
#define REG_TGDATA		(SDRAM_BA + 0x4C)// R	SDRAM test Gold data
		#define TGDATA			NVTBIT(31, 0) 		// Test Gold data

/* Timer Registers */
//#define TMR_BA		w55fa92_VA_TIMER	/* Timer */
#define REG_TCSR0		(TMR_BA+0x00)  /* Timer Control and Status Register 0 */
#define REG_TCSR1		(TMR_BA+0x04)  /* Timer Control and Status Register 1 */
#define REG_TICR0		(TMR_BA+0x08)  /* Timer Initial Control Register 0 */
#define REG_TICR1		(TMR_BA+0x0C)  /* Timer Initial Control Register 1 */
#define REG_TDR0		(TMR_BA+0x10)  /* Timer Data Register 0 */
#define REG_TDR1		(TMR_BA+0x14)  /* Timer Data Register 1 */
#define REG_TISR		(TMR_BA+0x18)  /* Timer Interrupt Status Register */
#define REG_WTCR		(TMR_BA+0x1C)  /* Watchdog Timer Control Register */
		#define PRESCALE_MODE	BIT8

#define REG_TCSR2		(TMR2_BA+0x00)  /* Timer Control and Status Register 2 */
#define REG_TCSR3		(TMR2_BA+0x04)  /* Timer Control and Status Register 3 */
#define REG_TICR2		(TMR2_BA+0x08)  /* Timer Initial Control Register 2 */
#define REG_TICR3		(TMR2_BA+0x0C)  /* Timer Initial Control Register 3 */
#define REG_TDR2		(TMR2_BA+0x10)  /* Timer Data Register 2 */
#define REG_TDR3		(TMR2_BA+0x14)  /* Timer Data Register 3 */
#define REG_TISR2		(TMR2_BA+0x18)  /* Timer Interrupt Status Register */

/*  VPOST Control Registers */
#define REG_LCM_LCDCCtl  	 		(VPOST_BA+0x00)  	// R/W: LCD Controller Control Register
	#define LCDCCtl_FSADDR_SEL		BIT31
	#define LCDCCtl_HAW_656			BIT30				// CCIR656 mode select
	#define LCDCCtl_PRDB_SEL		NVTBIT(21,20)		// Parallel RGB data bus selection for High Resolution Mode
	#define LCDCCtl_YUV_CLIP_EN		BIT17				// YUV clipped enable bit, Y(0 ~ 240), UV (0 ~235)
	#define LCDCCtl_YUVBL			BIT16				// YUV big endian(0) or little endian(1)
	#define LCDCCtl_SC_EN			BIT5				// Scaling Enable bit ("REG_LCM_SCO_SIZE" set the output size)
	#define LCDCCtl_FBSMODE			BIT4				// Enable bit for frame buffer base address switched in Vsync period mode
	#define LCDCCtl_FBDS 			NVTBIT(3,1)			// Frame buffer data selection
	#define LCDCCtl_LCDRUN 			BIT0				// LCD Controller Run.

#define REG_LCM_LCDCPrm   		(VPOST_BA+0x04)		// R/W: LCD Controller Parameter Register
	#define LCDCPrm_Even_Field_AL	NVTBIT(31,28)
	#define LCDCPrm_Odd_Field_AL	NVTBIT(27,24)
	#define LCDCPrm_F1_EL			NVTBIT(23,15)
	#define LCDCPrm_LCDSynTv		BIT8			// LCD timming Synch with TV
	#define LCDCPrm_SRGB_EL_SEL		NVTBIT(7,6)		// serial RGB Through Mode even line selection
	#define LCDCPrm_SRGB_OL_SEL		NVTBIT(5,4)		// serial RGB Through Mode odd line selection
	#define LCDCPrm_LCDDataSel		NVTBIT(3,2)		// LCD data interface Select
	#define LCDCPrm_LCDTYPE	  		NVTBIT(1,0)		// LCD device Type Select.

#define REG_LCM_LCDCInt 		(VPOST_BA+0x08)		// R/W: LCD Controller Interrupt Register
	#define LCDCInt_MPUCPLINTEN		BIT20 				// MPU Frame Complete Enable
	#define LCDCInt_TVFIELDINTEN	BIT18				// TV Even/Odd Field Interrupt Enable.
	#define LCDCInt_VINTEN			BIT17				// LCD VSYNC Interrupt Enable.
	#define LCDCInt_HINTEN			BIT16				// LCD HSYNC Interrupt Enable.
	#define LCDCInt_MPUCPL			BIT4				// MPU Frame Complete
	#define LCDCInt_TVFIELDINT		BIT2				// TV Odd/Even Field Interrupt.
	#define LCDCInt_VINT			BIT1				// LCD VSYNC/RD End Interrupt.
	#define LCDCInt_HINT			BIT0				// LCD HSYNC/WR End Interrupt.

#define REG_MPURD	 			(VPOST_BA+0x0c)		// Reserved

#define REG_LCM_TCON1 			(VPOST_BA+0x10)		// R/W: Timing Control Register 1
	#define TCON1_HSPW   			NVTBIT(23,16)		// Horizontal sync pulse width determines the HSYNC pulse's high level width by counting the number of the LCD Pixel Clock.
	#define TCON1_HBPD   			NVTBIT(15,8)		// Horizontal back porch is the number of LCD Pixel Clock periods between the falling edge of HSYNC and the start of active data.
	#define TCON1_HFPD   			NVTBIT(7,0)			// Horizontal front porch is the number of LCD Pixel Clock periods between the end of active data and the rising edge of HSYNC.

#define REG_LCM_TCON2 			(VPOST_BA+0x14)		// R/W: Timing Control Register 2
	#define TCON2_VSPW				NVTBIT(23,16)		// Vertical sync pulse width determines the VSYNC pulse's high level width by counting the number of inactive lines.
	#define TCON2_VBPD				NVTBIT(15,8)		// Vertical back porch is the number of inactive lines at the start of a frame, after vertical synchronization period.
	#define TCON2_VFPD				NVTBIT(7,0)		// Vertical front porch is the number of inactive lines at the end of a frame, before vertical synchronization period.

#define REG_LCM_TCON3 			(VPOST_BA+0x18)		// R/W: Timing Control Register 3
	#define TCON3_PPL				NVTBIT(31,16)		// Pixel Per-LineThe PPL bit field specifies the number of pixels in each line or row of screen.
	#define TCON3_LPP				NVTBIT(15,0)		// Lines Per-Panel The LPP bit field specifies the number of active lines per screen.

#define REG_LCM_TCON4 			(VPOST_BA+0x1c)		// R : Timing Control Register 4
	#define TCON4_TAPN				NVTBIT(31,20)		// Horizontal Total Active Pixel Number
	#define TCON4_MVPW				NVTBIT(19,8)
	#define TCON4_MPU_FMARKP		BIT5
	#define TCON4_MPU_VSYNCP		BIT4
	#define TCON4_VSP				BIT3				// LCD VSYNC Polarity.
	#define TCON4_HSP				BIT2				// LCD HSYNC Polarity.
	#define TCON4_DEP				BIT1				// LCD VDEN Polarity.
	#define TCON4_PCLKP				BIT0				// LCD Pixel Clock Polarity.

#define REG_LCM_MPUCMD 			(VPOST_BA+0x20)		// R/W: MPU-type LCD Command Register
	#define MPUCMD_MPU_VFPIN_SEL	BIT31
	#define MPUCMD_DIS_SEL			BIT30
	#define MPUCMD_CMD_DISn			BIT29				// Select command mode or display mode
	#define MPUCMD_MPU_CS			BIT28				// Set CS pin
	#define MPUCMD_MPU_ON			BIT27				// Trig to write or read from MPU in command mode
	#define MPUCMD_BUSY				BIT26				// Command interface is busy.
	#define MPUCMD_WR_RS			BIT25				// Write/Read RS Setting.
	#define MPUCMD_MPU_RWn			BIT24				// Read Status or data.
	#define MPUCMD_MPU68			BIT23				// MPU interface selection, reserved in w55fa92
	#define MPUCMD_FMARK			BIT22				// Frame Mark Detection Disable/Enable
	#define MPUCMD_MPU_SI_SEL		NVTBIT(19,16)		// MPU-type system interface selection
	#define MPUCMD_MPU_CMD			NVTBIT(15,0)			// MPU-type LCD command/parameter data, read data

#define REG_LCM_MPUTS 			(VPOST_BA+0x24)		// R/W: MPU type LCD timing setting
	#define MPUTS_CSnF2DCt			NVTBIT(31,24)		// CSn fall edge to data change clock counter
	#define MPUTS_WRnR2CSnRt		NVTBIT(23,16)		// WRn rising edge to CSn rising clock counter
	#define MPUTS_WRnLWt			NVTBIT(15, 8)		// WR low pulse clock counter
	#define MPUTS_CSnF2WRnFt		NVTBIT( 7, 0)		// CSn falling edge to WR falling edge clock counter

#define REG_LCM_OSD_CTL 		(VPOST_BA+0x28)		// R/W : OSD Control Register
	#define OSD_CTL_OSD_EN			BIT31			// OSD enable/disable
	#define OSD_CTL_OSD_TPEN		BIT28			// OSD transparent enable/disable
	#define OSD_CTL_OSD_FSEL		NVTBIT(27,24)	// OSD foramt selection
	#define OSD_CTL_OSD_TC			NVTBIT(23,0)	// OSD transparent color setting for different color format

#define REG_LCM_OSD_SIZE 		(VPOST_BA+0x2C)		// R/W: OSD Picture Size
	#define OSD_SIZE_OSD_VSIZE		NVTBIT(25,16)	// OSD vertical size (+1)
	#define OSD_SIZE_OSD_HSIZE		NVTBIT(9,0)		// OSD horizontal size (+1)

#define REG_LCM_OSD_SP	 		(VPOST_BA+0x30)		// R/W: OSD Start Position
	#define OSD_SP_OSD_SY			NVTBIT(25,16)	// OSD vertical start position
	#define OSD_SP_OSD_SX			NVTBIT(9,0)		// OSD horizontal start position

#define REG_LCM_OSD_BEP	 		(VPOST_BA+0x34)		// R/W: OSD Bar End Position
	#define OSD_BEP_OSD_1BEY		NVTBIT(25,16)	// OSD vertical 1st line End Position
	#define OSD_BEP_OSD_1BEX		NVTBIT(9,0)		// OSD horizontal 1st line End Position

#define REG_LCM_OSD_BO			(VPOST_BA+0x38)		// R/W: OSD Bar Offset
	#define OSD_BO_OSD_BOY			NVTBIT(25,16)	// OSD vertical offset between 1st and 2nd lines
	#define OSD_BO_OSD_BOX			NVTBIT(9,0)		// OSD horizontal offset between 1st and 2nd lines

#define REG_LCM_CBAR	  		(VPOST_BA+0x3C)		// R/W: Color Burst Avtive Region
	#define CBAR_CTL_EQ6SEL			BIT28
	#define CBAR_CTL_HCBEPC			NVTBIT(25,16)
	#define CBAR_CTL_HCBBPC			NVTBIT(9,0)

#define REG_LCM_TVCtl			(VPOST_BA+0x40)		// R/W: TvControl Register
	#define TVCtl_TvField 			BIT31				// Tv field status (read only)
														//   	 1 = Odd field, 0 = even field
	#define TVCtl_TvFFFE 			BIT26				// Tv Flicker Free Filtr enable/disable
	#define TVCtl_NTSC_TYPE			NVTBIT(23,22)		// NTSC type selection
															//	00 = NTSC
															//  01 = NTSC-J
															//  10 = NTSC-433
															//	11 = NTSC (same as 00)
	#define TVCtl_PAL_TYPE			NVTBIT(21,20)		// PAL type selection
															//	00 = PAL
															//  01 = PAL-Nc
															//  10 = PAL-M
															//	11 = PAL (same as 00)
	#define TVCtl_PAL288 			BIT17				// Support 288-line mode (source pictire size must be 640x576 or 720x576)

	#define TVCtl_DAC_NORMAL		BIT16				// 1: DAC noraml; 0: DAC debug
	#define TVCtl_TvCMM 			BIT16				// TV Color Modulation Method, reserved in w55fa92
															//   1 = 27 MHz, 0 = 13.5 MHz
															
	#define TVCtl_TV_D1 			BIT15				// TV D1 size
	#define TVCtl_FBSIZE 			NVTBIT(15,14)		// Frame Buffer Size in Tv NonInterlance Mode; these two bits are removed in FA92
															//	00 = 320x240 (QVGA)
															//  01 = 640x240 (HVGA)
															//  10 = 640x480 (VGA) or 640x576 (PAL288 mode)
															//  11 = 720x480 (D1) or 720x576 (PAL288 mode)
	#define TVCtl_LCDSrc 			NVTBIT(11,10)		// LCD image source selection
	#define TVCtl_TvSrc 			NVTBIT(9,8)			// TV image source selection
	#define TVCtl_Noninter_TYPE		BIT7				// Non-interlace type selection
															//   1 = 262 lines, 0 = 263 lines
	#define TVCtl_TvLBSA 			BIT6				// Tv Line Buffer Scaling Alograthim (320->640)
	#define TVCtl_Tvdac 			BIT4				// Tv DAC Enable/Disable
	#define TVCtl_TvInter 			BIT3				// Interlance or Non Interlance
	#define TVCtl_TvSys 			BIT2				// TV System Selection.
	#define TVCtl_TvColor 			BIT1				// TV Color Selection Color/Black.
	#define TVCtl_TvSleep 			BIT0				// TV Encoder Enable/Disable.

#define REG_TVOUT_FLT			(VPOST_BA+0x44)		// R/W: TV output filter register
	#define YLPF_SEL	 			BIT6				// Luma low-pass filter selection
															//   1 = 9-tap, 0 = disable
	#define UVLPF_SEL	 			BIT4				// Chroma low-pass filter selection
															//   1 = 9-tap, 0 = disable
	#define YUP_SEL		 			NVTBIT(3,2)			// Luma upsample filter selection
															//   01 = 2-tap, 00 = disable
															//   10 = 3-tap, 11 = 7-tap
	#define UVUP_SEL		 		NVTBIT(1,0)			// Luma upsample filter selection
															//   01 = 2-tap, 00 = disable
															//   10 = 3-tap, 11 = 7-tap

#define REG_TVOUT_ADJ			(VPOST_BA+0x48)		// R/W: TV output active adjust register
	#define VER_ACTADJ	 			NVTBIT(28,24)		// TV vertical output active position adjust
	#define HOR_ACTADJ	 			NVTBIT(21,16)		// TV horizontal output active position adjust


#define REG_LCM_COLORSET  		(VPOST_BA+0x4C)		// R/W: Backdraw Color Setting Register
	#define VD_SWAP_MODE	 		NVTBIT(25,24)		// LCD data bus swapped mode selection
	#define COLORSET_Color_R		NVTBIT(23,16)		// Color R value
	#define COLORSET_Color_G		NVTBIT(15,8)		// Color G value
	#define COLORSET_Color_B		NVTBIT(7,0)			// Color B value

#define REG_LCM_FSADDR    		(VPOST_BA+0x50)		// R/W: Frame Buffer Start Address

#define REG_LCM_TVDisCtl  		(VPOST_BA+0x54)		// R/W: TV Display Start Control Register
	#define TVDisCtl_FFRHS			NVTBIT(31,24)
	#define TVDisCtl_LCDHB			NVTBIT(23,16)		// LCD H bland setting for Syn TV Display
	#define TVDisCtl_TVDVS			NVTBIT(15,8)		// TV Display Start Line Register
	#define TVDisCtl_TVDHS			NVTBIT(7,0)			// TV Display Start Pixel Register

#define REG_LCM_CBACtl  		(VPOST_BA+0x58)		// R/W: Color Burst Amplitude Control Register
	#define CBACtl_CBA_CB4			NVTBIT(29,24)
	#define CBACtl_CBA_CB3			NVTBIT(21,16)
	#define CBACtl_CBA_CB2			NVTBIT(13,8)
	#define CBACtl_CBA_CB1			NVTBIT(5,0)

#define REG_LCM_OSD_ADDR  		(VPOST_BA+0x5C)		// R/W: OSD Frame Buffer Start Address
#define REG_TV_FFFSET1	 	  	(VPOST_BA+0x60)		// FV flick free filter setting register 1

#define REG_LCM_TVContrast		(VPOST_BA+0x64)		// R/W: Tv contrast adjust setting register
	#define TVContrast_Cr_contrast	NVTBIT(23,16)		// Cr compoment contrast adjust
	#define TVContrast_Cb_contrast	NVTBIT(15,8)		// Cb compoment contrast adjust
	#define TVContrast_Y_contrast	NVTBIT(7,0)			// Y  compoment contrast adjust

#define REG_LCM_TVBright  		(VPOST_BA+0x68)		// R/W: Tv Bright adjust setting register
	#define TVBright_Cr_gain		NVTBIT(23,16)		// Cr compoment bright adjust
	#define TVBright_Cb_gain		NVTBIT(15,8)		// Cb compoment bright adjust
	#define TVBright_Y_bright		NVTBIT(7,0)			// Y  compoment bright adjust

#define REG_TV_FFFSET2	 	  	(VPOST_BA+0x6C)		// FV flick free filter setting register 2

#define REG_LCM_LINE_STRIPE		(VPOST_BA+0x70)		// R/W : Line Stripe Offset
	#define OSD_LSD					NVTBIT(31,16)		// OSD buffer line stripe offset register
	#define LINE_STRIPE_F1_LSL		NVTBIT(15,0)		// frame buffer line stripe offset register

#define REG_LCM_RGBin		  	(VPOST_BA+0x74)		// RGB888 data input for RGB2YCbCr equation

#define REG_LCM_YCbCrout  		(VPOST_BA+0x78)		// YCbCr data output for RGB2YCbCr equation

#define REG_LCM_YCbCrin	  		(VPOST_BA+0x7C)		// YCbCr data input for YCbCr2RGB equation
	#define YCbCrin_Yin				NVTBIT(23,16)		// Y byte data input
	#define YCbCrin_Cbin			NVTBIT(15, 8)		// Cb byte data input
	#define YCbCrin_Crin			NVTBIT( 7, 0)		// Cr byte data input

#define REG_LCM_RGBout	  		(VPOST_BA+0x80)		// RGB data output for YCbCr2RGB equation
	#define RGBout_Rout				NVTBIT(23,16)		// R byte data output
	#define RGBout_Gout				NVTBIT(15, 8)		// G byte data output
	#define RGBout_Bout				NVTBIT( 7, 0)		// B byte data output

#define REG_LCM_LBTestCtl		(VPOST_BA+0x88)		// R: Line Buffer Test Control
	#define LBTestCtl_FinishR		NVTBIT(18,16)		// 
	#define LBTestCtl_BistFailR		NVTBIT(10, 8)		// 
	#define LBTestCtl_BistModeR		NVTBIT( 2, 0)		// 

#define REG_LCM_OSD_MASK		(VPOST_BA+0x90)		// R/W: OSD transparent mask control, REG_LCM_OSD_MASK[23:0]

#define REG_LCM_OSD_ALPHA		(VPOST_BA+0x94)		// R/W: OSD constant alpha blending
	#define OSD_ALPHA_EN			BIT31				// OSD alpha enable bit
	#define OSD_ALPHA				NVTBIT(7, 0)		// OSD alpha value

#define REG_LCM_VA_TEST			(VPOST_BA+0x98)		// R/W: Frame Buffer Check Sum
	#define VA_CHECK_START			BIT31				// checksum start
	#define VA_CHECKSUM				NVTBIT(15, 0)		// VA checksum

#define REG_LCM_KPI_HS_DLY		(VPOST_BA+0x9C)		// R/W: LCD share Hsync Bus to KPI Time Setting
	#define KPI_REF_SYNC			BIT31				// Set to 1 will use larger timing range to share LCD bus
	#define KPI_HFD					NVTBIT(26,16)		// Hsync Front Delay time for KPI
	#define KPI_HBD					NVTBIT(10, 0)		// Hsync Back Delay time for KPI

#define REG_LCM_FB_SIZE			(VPOST_BA+0xA0)		// R/W: Frame buffer size
	#define FB_X					NVTBIT(31,16)		// X-axis size
	#define FB_Y					NVTBIT(15, 0)		// Y-axis size

#define REG_LCM_SCO_SIZE		(VPOST_BA+0xA4)		// R/W: Scaling size
	#define SCOL_X					NVTBIT(31,16)		// X-axis scaling
	#define SCOL_Y					NVTBIT(15, 0)		// Y-axis scaling

/*
 VideoIn Control Registers
*/
/*
 VideoIn Control Registers
*/
#define REG_VPECTL  		(VIN_BA + 0x00)	// R/W: Video Pre-processor Control Register
			#define VPRST		BIT24					// Video Pre-processor Reset.
			#define UPDATE 		BIT20					// Video-In Update Register at New Frame
			#define CAPONE		BIT16					// Video-In One Shutter
			#define VPRBIST		BIT8					// Video-In One Shutter
			#define PKEN		BIT6					// Packet Output Enable
			#define PNEN		BIT5					// Planar Output Enable
			#define ADDRSW		BIT3					// Packet Buffer Address select
			#define FBMODE		BIT2					// Packet Frame Buffer Control by FSC
			#define VPEEN		BIT0					// Planar Output Enable

#define REG_VPEPAR		(VIN_BA + 0x04)	// R/W: Video Pre-processor Parameter Register
			#define MB_PLANAR		BIT31					// YUV output in macro block planar mode.	
			#define VPEBFIN		BIT28					// BIST Finish [Read Only]
			#define BFAIL			NVTBIT(27, 24)		// BIST Fail Flag [Read Only]
			#define FLDID			BIT20				// Field ID [Read Only]
			#define FLD1EN		BIT17					// Field 1 Input Enable
			#define FLD0EN		BIT16					// Field 0 Input Enable
			#define FLDDETP		BIT15					// Field Detect Position
			#define FLDDETM 		BIT14				// Field Detect Mode (By HSYNC or input FIELD PIN)
			#define FLDSWAP		BIT13					// Swap Input Field
			#define SCEF		NVTBIT(12, 11)			// Special Color Effect.
			#define VSP			BIT10					// Sensor Vsync Polarity.
			#define HSP			BIT9					// Sensor Hsync Polarity
			#define PCLKP			BIT8				// Sensor Pixel Clock Polarity
			#define PNFMT		BIT7					// Planar Output Format
			#define RANGE		BIT6					// Scale Input YUV CCIR601 color range to full range
			#define OUTFMT 		NVTBIT(5, 4)			// Image Data Format Output to System Memory.
			#define PDORD		NVTBIT(3, 2)			// Sensor Output Type
			#define SNRTYPE 		BIT1				// device is CCIR601 or CCIR656
			#define INFMT 		BIT0					// Sensor Output Format


#define REG_VPEINT  		(VIN_BA + 0x08)	// R/W: Video Pre-processor Interrupt  Register
			#define MDINTEN		BIT20					// Motion Detection Interrupt Enable
			#define ADDRMEN  	BIT19					// Address Match Interrupt Enable.
			#define MEINTEN		BIT17					// System Memory Error Interrupt Enable.
			#define VINTEN		BIT16					// Video Frame End Interrupt Enable.
			#define MDINT		BIT4					// Motion Detection Output Finsish Interrupt
			#define ADDRMINT		BIT3				// Memory Address Match Interrupt Flag.
			#define MEINT		BIT1					// System Memory Error Interrupt. If read this bit shows 1, Memory Error occurs. Write 0 to clear it.
			#define VINT			BIT0				// Video Frame End Interrupt. If read this bit shows 1, received a frame complete. Write 0 to clear it.

#define REG_VPOSTERIZE		(VIN_BA + 0x0C)	// R/W: Video Post-processor Control  Register
			#define YCOM		NVTBIT(23, 16)			// Y Component Posterizing factor
			#define UCOM		NVTBIT(15, 8)			// U Component Posterizing factor
			#define VCOM		NVTBIT(7, 0)			// V Component Posterizing factor


#define REG_VPEMD  		(VIN_BA + 0x10)	// R/W: Motion Detection  Register
			#define MDTHR	  	NVTBIT(20, 16)			// MD Differential Threshold
			#define MDDF			NVTBIT(11, 10)		// MD Detect Frequence
			#define MDSM			BIT9				// MD Save Mode
			#define MDBS			BIT8				// MD Block Size
			#define MDEN			BIT0				// MD Enable

#define REG_MDADDR  		(VIN_BA + 0x14)	// R/W: Motion Detection Output Address Register
#define REG_MDYADDR  	(VIN_BA + 0x18)	// R/W: Motion Detection Output Address Register

#define REG_VSEPIA		(VIN_BA + 0x1C)	// R/W: SEPIA Color Effect Control  Register
			#define SEPIAU		NVTBIT(15, 8)			// U Component Sepia Factor
			#define SEPIAV		NVTBIT(7, 0)			// V Component Sepia Factor

#define REG_VPECWSP  	(VIN_BA + 0x20)	// R/W:  Cropping Window Starting Address Register
			#define CWSPV		NVTBIT(26, 16)			// Cropping Window Vertical Starting Address
			#define CWSPH		NVTBIT(11, 0)			// Cropping Window Horizontal  Starting Address

#define REG_VPECWS	 	(VIN_BA + 0x24)	// R/W:  Cropping Window Size Register
			#define CWSH			NVTBIT(26, 16)		// Cropping Image Window Height
			#define CWSW		NVTBIT(11, 0)			// Cropping Image Window Width

#define REG_VPEPKDSL  		(VIN_BA + 0x28)	// R/W  : Packet Scaling Vertical/Horizontal Factor Register
#define REG_VPEPNDSL 		(VIN_BA + 0x2C)	// R?W  : Planar Scaling Vertical/Horizontal Factor Register
			#define DSVNL			NVTBIT(31, 24)		// Scaling Vertical Factor N
			#define DSVML			NVTBIT(23, 16)		// Scaling Vertical Factor M
			#define DSHNL			NVTBIT(15, 8)		// Scaling Horizontal Factor N
			#define DSHML			NVTBIT(7, 0)		// Scaling Horizontal Factor M

#define REG_VPEPKDSH  		(VIN_BA + 0x48)	// R/W  : Packet Scaling Vertical/Horizontal Factor Register
#define REG_VPEPNDSH 		(VIN_BA + 0x4C)	// R?W  : Planar Scaling Vertical/Horizontal Factor Register
			#define DSVNH			NVTBIT(31, 24)		// Scaling Vertical Factor N
			#define DSVMH			NVTBIT(23, 16)		// Scaling Vertical Factor M
			#define DSHNH			NVTBIT(15, 8)		// Scaling Horizontal Factor N
			#define DSHMH			NVTBIT(7, 0)		// Scaling Horizontal Factor M

#define REG_VPEFRC  		(VIN_BA + 0x30)		// R/W  : Scaling Frame Rate Factor Register
			#define FRCN			NVTBIT(13, 8)		// Scaling Frame Rate Factor N
			#define FRCM			NVTBIT(5, 0)		// Scaling Frame Rate Factor M

/*
#define REG_VWIDTH  		(VIN_BA + 0x34)		// R/W  : Frame Output Pixel Straight Width Register
			#define PNOW		BIT(27, 16)				// Planar Frame Output Pixel Straight Width
			#define PKOW		BIT(11, 0)				// Packet Frame Output Pixel Straight Width
*/
#define REG_VSTRIDE 		(VIN_BA + 0x34)		// R/W  : Frame Stride Register
			#define PNSTRIDE		NVTBIT(27, 16)		// Planar Frame Stride
			#define PKSTRIDE		NVTBIT(11, 0)		// Packet Frame Stride

#define REG_VFIFO 		(VIN_BA + 0x3C)		// R/W  : FIFO threshold Register
			#define FTHP			NVTBIT(27, 24)		// Packet FIFO Threshold
			#define PTHY			NVTBIT(19, 16)		// Planar Y FIFO Threshold
			#define PTHU			NVTBIT(10, 8)		// Planar U FIFO Threshold
			#define PTHV			NVTBIT(2, 0)		// Planar V FIFO Threshold

#define REG_CMPADDR 		(VIN_BA + 0x40)		// R/W  : Current Packet System Memory Address Register
#define REG_CURADDRP 	(VIN_BA + 0x50)		// R/W  : FIFO threshold Register
#define REG_CURADDRY 	(VIN_BA + 0x54)	// R/W  : Current Planar Y System Memory Address Register
#define REG_CURADDRU 	(VIN_BA + 0x58)	// R/W  : Current Planar U System Memory Address Register
#define REG_CURADDRV 	(VIN_BA + 0x5C)	// R/W  : Current Planar V System Memory Address Register
#define REG_PACBA0 		(VIN_BA + 0x60)	// R/W  : System Memory Packet 0 Base Address Register
#define REG_PACBA1 		(VIN_BA + 0x64)	// R/W  : System Memory Packet 1 Base Address Register
#define REG_YBA0 			(VIN_BA + 0x80)	// R/W  : System Memory Planar Y Base Address Register
#define REG_UBA0 			(VIN_BA + 0x84)	// R/W  : System Memory Planar U Base Address Register
#define REG_VBA0 			(VIN_BA + 0x88)	// R/W  : System Memory Planar V Base Address Register


/*
 SIC Control Registers
*/

#define     REG_FB_0		(DMAC_BA+0x000)  /* Shared Buffer (FIFO) */

#define	REG_DMACCSR			(DMAC_BA+0x400)  /* DMAC Control and Status Register */
	#define	FMI_BUSY			BIT9			// FMI DMA transfer is in progress
	#define SG_EN				BIT3			// DMAC Scatter-gather function enable
	#define DMAC_SWRST			BIT1			// DMAC software reset enable
	#define DMAC_EN				BIT0			// DMAC enable

#define     REG_DMACSAR	    (DMAC_BA+0x408)  /* DMAC Transfer Starting Address Register */
    #define PAD_ORDER           BIT0            // Physical Address Descriptor table fetching in order or out of order 
#define     REG_DMACBCR		(DMAC_BA+0x40C)  /* DMAC Transfer Byte Count Register */
#define     REG_DMACIER		(DMAC_BA+0x410)  /* DMAC Interrupt Enable Register */
	#define	WEOT_IE				BIT1			// Wrong EOT encounterred interrupt enable
	#define TABORT_IE			BIT0			// DMA R/W target abort interrupt enable

#define     REG_DMACISR		(DMAC_BA+0x414)  /* DMAC Interrupt Status Register */
	#define	WEOT_IF				BIT1			// Wrong EOT encounterred interrupt flag
	#define TABORT_IF			BIT0			// DMA R/W target abort interrupt flag


/* Flash Memory Card Interface Registers */
#define REG_FMICR				(FMI_BA+0x000)   	/* FMI Control Register */
	#define	FMI_SM_EN				BIT3				// enable FMI SM function
	#define FMI_SD_EN				BIT1				// enable FMI SD function
	#define FMI_SWRST				BIT0				// enable FMI software reset

#define REG_FMIIER				(FMI_BA+0x004)   	/* FMI DMA Transfer Starting Address Register */
	#define	FMI_DAT_IE				BIT0				// enable DMAC READ/WRITE targe abort interrupt generation

#define REG_FMIISR				(FMI_BA+0x008)   	/* FMI DMA Byte Count Register */
	#define	FMI_DAT_IF				BIT0				// DMAC READ/WRITE targe abort interrupt flag register

/* Secure Digit Registers */
#define REG_SDCR				(FMI_BA+0x020)   	/* SD Control Register */
	#define	SDCR_CLK_KEEP1			BIT31				// SD-1 clock keep control
	#define	SDCR_SDPORT				NVTBIT(30,29)		// SD port select
	#define	SDCR_SDPORT_0			0					// SD-0 port selected
	#define	SDCR_SDPORT_1			BIT29				// SD-1 port selected
	#define	SDCR_SDPORT_2			BIT30				// SD-2 port selected
	#define	SDCR_CLK_KEEP2			BIT28				// SD-1 clock keep control
	#define	SDCR_SDNWR				NVTBIT(27,24)		// Nwr paramter for Block Write operation
	#define SDCR_BLKCNT				NVTBIT(23,16)		// Block conut to be transferred or received
	#define	SDCR_DBW				BIT15				// SD data bus width selection
	#define	SDCR_SWRST				BIT14				// enable SD software reset
	#define	SDCR_CMD_CODE			NVTBIT(13,8)		// SD Command Code
	#define	SDCR_CLK_KEEP			BIT7				// SD Clock Enable
	#define SDCR_8CLK_OE			BIT6				// 8 Clock Cycles Output Enable
	#define SDCR_74CLK_OE			BIT5				// 74 Clock Cycle Output Enable
	#define SDCR_R2_EN				BIT4				// Response R2 Input Enable
	#define SDCR_DO_EN				BIT3				// Data Output Enable
	#define SDCR_DI_EN				BIT2				// Data Input Enable
	#define SDCR_RI_EN				BIT1				// Response Input Enable
	#define SDCR_CO_EN				BIT0				// Command Output Enable

#define REG_SDARG 				(FMI_BA+0x024)   	/* SD command argument register */

#define REG_SDIER				(FMI_BA+0x028)   	/* SD interrupt enable register */
	#define	SDIER_CD2SRC			BIT31				// SD card detection source selection: SD-DAT3 or GPIO
	#define	SDIER_CDSRC				BIT30				// SD card detection source selection: SD-DAT3 or GPIO
	#define	SDIER_R1B_IEN			BIT24				// R1b interrupt enable
	#define	SDIER_WKUP_EN			BIT14				// SDIO wake-up signal geenrating enable
	#define	SDIER_DITO_IEN			BIT13				// SD data input timeout interrupt enable
	#define	SDIER_RITO_IEN			BIT12				// SD response input timeout interrupt enable
	#define SDIER_SDIO1_IEN			BIT11				// SDIO1 Interrupt Status Enable (SDIO issue interrupt via DAT[1]
	#define SDIER_SDIO_IEN			BIT10				// SDIO Interrupt Status Enable (SDIO issue interrupt via DAT[1]
	#define SDIER_CD_IEN			BIT8				// CD# Interrupt Status Enable
	#define SDIER_SDIO2_IEN			BIT2				// SDIO2 Interrupt Status Enable (SDIO issue interrupt via DAT[1]
	#define SDIER_CRC_IEN			BIT1				// CRC-7, CRC-16 and CRC status error interrupt enable
	#define SDIER_BLKD_IEN			BIT0				// Block transfer done interrupt interrupt enable

#define REG_SDISR				(FMI_BA+0x02C)   	/* SD interrupt status register */
	#define	SDISR_R1B_IF			BIT24				// R1b interrupt flag
	#define	SDISR_SDIO2_IF			BIT22			    // SDIO2 interrupt flag (SDIO issue interrupt via DAT[1]		  
	#define SDISR_SD2_DATA1			BIT20			    // SD2 DAT1 pin status                                         
	#define SDISR_SD1_DATA1			BIT19			    // SD1 DAT1 pin status                                         
	#define SDISR_SD_DATA1			BIT18			    // SD DAT1 pin status
	#define SDISR_CD_Card			BIT16			    // CD detection pin status
	#define	SDISR_DITO_IF			BIT13			    // SD data input timeout interrupt flag
	#define	SDISR_RITO_IF			BIT12			    // SD response input timeout interrupt flag
	#define	SDISR_SDIO1_IF			BIT11			    // SDIO1 interrupt flag (SDIO issue interrupt via DAT[1]
	#define	SDISR_SDIO_IF			BIT10			    // SDIO interrupt flag (SDIO issue interrupt via DAT[1]
	#define	SDISR_CD_IF				BIT8			    // CD# interrupt flag
	#define SDISR_SD_DATA0			BIT7			    // SD DATA0 pin status
	#define SDISR_CRC				NVTBIT(6,4)		    // CRC status
	#define SDISR_CRC_16			BIT3			    // CRC-16 Check Result Status
	#define SDISR_CRC_7				BIT2			    // CRC-7 Check Result Status
	#define	SDISR_CRC_IF			BIT1			    // CRC-7, CRC-16 and CRC status error interrupt status
	#define	SDISR_BLKD_IF			BIT0			    // Block transfer done interrupt interrupt status

#define REG_SDRSP0				(FMI_BA+0x030)   	/* SD receive response token register 0 */
#define REG_SDRSP1				(FMI_BA+0x034)   	/* SD receive response token register 1 */
#define REG_SDBLEN				(FMI_BA+0x038)   	/* SD block length register */
#define REG_SDTMOUT 			(FMI_BA+0x03C)   	/* SD RI/DI timeout register */

/* NAND-type Flash Registers */
// old nuc930
#define     REG_SM_ECC48_ST0		(FMI_BA+0x0D4)   /* ECC Register */
#define     REG_SM_ECC48_ST1		(FMI_BA+0x0D8)   /* ECC Register */
#define     REG_BCH_ECC_BIT_ADDR0	(FMI_BA+0x220)   /* NAND Flash BCH error bit address for error bit 0-7 Register */
#define     REG_BCH_ECC_BIT_ADDR1	(FMI_BA+0x224)   /* NAND Flash BCH error bit address for error bit 8-14 Register */

// new FA92
#define REG_SMCSR				(FMI_BA+0x0A0)   	/* NAND Flash Control and Status Register */
	#define SMCR_CS1				BIT26				// SM chip select
	#define SMCR_CS0				BIT25				// SM chip select
	#define SMCR_CS					BIT25				// SM chip select
	#define SMCR_ECC_EN				BIT23				// SM chip select
	#define SMCR_BCH_TSEL 			NVTBIT(22,18)		// BCH T4/8/12/15/24 selection
		#define BCH_T15 				BIT22			    // BCH T15 selected
		#define BCH_T12 				BIT21			    // BCH T12 selected
		#define BCH_T8	 				BIT20			    // BCH T8 selected
		#define BCH_T4	 				BIT19			    // BCH T4 selected
		#define BCH_T24	 				BIT18			    // BCH T24 selected

	#define SMCR_PSIZE 				NVTBIT(17,16)		// SM page size selection
		#define PSIZE_8K 			    BIT17+BIT16			// page size 8K selected
		#define PSIZE_4K 			    BIT17				// page size 4K selected
		#define PSIZE_2K 			    BIT16				// page size 2K selected
		#define PSIZE_512			    0					// page size 512 selected

	#define SMCR_SRAM_INIT			BIT9				// SM RA0_RA1 initial bit (to 0xFFFF_FFFF)
	#define SMCR_ECC_3B_PROTECT		BIT8				// ECC protect redundant 3 bytes
	#define SMCR_ECC_CHK			BIT7				// ECC parity check enable bit during read page
	#define SMCR_PROT_REGION_EN 	BIT5				// Protect Region enable
	#define SMCR_REDUN_AUTO_WEN 	BIT4				// Redundant auto write enable
	#define SMCR_REDUN_REN 			BIT3				// Redundant read enable
	#define SMCR_DWR_EN 			BIT2				// DMA write data enable
	#define SMCR_DRD_EN 			BIT1				// DMA read data enable
	#define SMCR_SM_SWRST 			BIT0				// SM software reset


#define REG_SMTCR				(FMI_BA+0x0A4)   	/* NAND Flash Timing Control Register */

#define REG_SMIER				(FMI_BA+0x0A8)   	/* NAND Flash Interrupt Control Register */
	#define	SMIER_RB1_IE			BIT11				// RB1 pin rising-edge detection interrupt enable
	#define	SMIER_RB0_IE			BIT10				// RB0 pin rising-edge detection interrupt enable
	#define	SMIER_RB_IE				BIT10				// RB0 pin rising-edge detection interrupt enable
	#define SMIER_PROT_REGION_WR_IE BIT3				// Protect Region write detect interrupt enable
	#define SMIER_ECC_FIELD_IE 		BIT2				// ECC field error check interrupt enable
	#define SMIER_DMA_IE			BIT0				// DMA RW data complete interrupr enable

#define REG_SMISR				(FMI_BA+0x0AC)   	/* NAND Flash Interrupt Status Register */
	#define	SMISR_RB1		 		BIT19				// RB1 pin status
	#define	SMISR_RB0 				BIT18				// RB0 pin status
	#define	SMISR_RB 				BIT18				// RB pin status
	#define	SMISR_RB1_IF		 	BIT11				// RB pin rising-edge detection interrupt flag
	#define	SMISR_RB0_IF	 		BIT10				// RB pin rising-edge detection interrupt flag
	#define SMISR_PROT_REGION_WR_IF BIT3				// Protect Region write detect interrupt flag
	#define SMISR_ECC_FIELD_IF 		BIT2				// ECC field error check interrupt flag
	#define SMISR_DMA_IF			BIT0				// DMA RW data complete interrupr flag


#define REG_SMCMD				(FMI_BA+0x0B0)   	/* NAND Flash Command Port Register */

#define REG_SMADDR				(FMI_BA+0x0B4)   	/* NAND Flash Address Port Register */
	#define	EOA_SM	 				BIT31				// end of SM address for last SM address

#define REG_SMDATA				(FMI_BA+0x0B8)   	/* NAND Flash Data Port Register */

#define REG_SMREAREA_CTL 		(FMI_BA+0x0BC)  	/* NAND Flash redundnat area control register */
	#define SMRE_MECC 				NVTBIT(31,16)		// Mask ECC parity code to NAND during Write Page Data to NAND by DMAC
	#define	SMRE_REA128_EXT			NVTBIT(8,0)			// Redundant area enabled byte number

#define REG_SM_ECC_ST0			(FMI_BA+0x0D0)   	/* ECC Register */
	#define ECCST_F4_ECNT			NVTBIT(30,26)		// error count of ECC for field 4
	#define ECCST_F4_STAT 			NVTBIT(25,24)		// error status of ECC for field 4
	#define ECCST_F3_ECNT 	        NVTBIT(22,18)		// error count of ECC for field 3
	#define ECCST_F3_STAT 	        NVTBIT(17,16)		// error status of ECC for field 3
	#define ECCST_F2_ECNT 	        NVTBIT(14,10)		// error count of ECC for field 2
	#define ECCST_F2_STAT 	        NVTBIT(9,8)			// error status of ECC for field 2
	#define ECCST_F1_ECNT 	        NVTBIT(6,2)			// error count of ECC for field 1
	#define ECCST_F1_STAT 	        NVTBIT(1,0)			// error status of ECC for field 1

#define REG_SM_ECC_ST1		    (FMI_BA+0x0D4)   	/* ECC Register */
	#define ECCST_F8_ECNT			NVTBIT(30,26)		// error count of ECC for field 8
	#define ECCST_F8_STAT 			NVTBIT(25,24)		// error status of ECC for field 8
	#define ECCST_F7_ECNT 	        NVTBIT(22,18)		// error count of ECC for field 7
	#define ECCST_F7_STAT 	        NVTBIT(17,16)		// error status of ECC for field 7
	#define ECCST_F6_ECNT 	        NVTBIT(14,10)		// error count of ECC for field 6
	#define ECCST_F6_STAT 	        NVTBIT(9,8)			// error status of ECC for field 6
	#define ECCST_F5_ECNT 	        NVTBIT(6,2)			// error count of ECC for field 5
	#define ECCST_F5_STAT 	        NVTBIT(1,0)			// error status of ECC for field 5

#define REG_SM_ECC_ST2		    (FMI_BA+0x0D8)   	/* ECC Register */
	#define ECCST_F12_ECNT			NVTBIT(30,26)		// error count of ECC for field 12
	#define ECCST_F12_STAT 			NVTBIT(25,24)		// error status of ECC for field 12
	#define ECCST_F11_ECNT 	        NVTBIT(22,18)		// error count of ECC for field 11
	#define ECCST_F11_STAT 	        NVTBIT(17,16)		// error status of ECC for field 11
	#define ECCST_F10_ECNT 	        NVTBIT(14,10)		// error count of ECC for field 10
	#define ECCST_F10_STAT 	        NVTBIT(9,8)			// error status of ECC for field 10
	#define ECCST_F9_ECNT 	        NVTBIT(6,2)			// error count of ECC for field 9
	#define ECCST_F9_STAT 	        NVTBIT(1,0)			// error status of ECC for field 9

#define REG_SM_ECC_ST3		    (FMI_BA+0x0DC)   	/* ECC Register */
	#define ECCST_F16_ECNT			NVTBIT(30,26)		// error count of ECC for field 16
	#define ECCST_F16_STAT 			NVTBIT(25,24)		// error status of ECC for field 16
	#define ECCST_F15_ECNT 	        NVTBIT(22,18)		// error count of ECC for field 15
	#define ECCST_F15_STAT 	        NVTBIT(17,16)		// error status of ECC for field 15
	#define ECCST_F14_ECNT 	        NVTBIT(14,10)		// error count of ECC for field 14
	#define ECCST_F14_STAT 	        NVTBIT(9,8)			// error status of ECC for field 14
	#define ECCST_F13_ECNT 	        NVTBIT(6,2)			// error count of ECC for field 13
	#define ECCST_F13_STAT 	        NVTBIT(1,0)			// error status of ECC for field 13


#define REG_SM_PROT_ADDR0	    (FMI_BA+0x0E0)   	/* Smart-Media Protect Region End Address 0 Register */
#define REG_SM_PROT_ADDR1	    (FMI_BA+0x0E4)   	/* Smart-Media Protect Region End Address 1 Register */
	#define SM_PROT_ADDR1 	        NVTBIT(7,0)			// High address for Protect Region End Address

#define REG_BCH_ECC_ADDR0	    (FMI_BA+0x100)   	/* NAND Flash BCH error byte address for error bit 0-1 Register */
#define REG_BCH_ECC_ADDR1	    (FMI_BA+0x104)   	/* NAND Flash BCH error byte address for error bit 2-3 Register */
#define REG_BCH_ECC_ADDR2	    (FMI_BA+0x108)   	/* NAND Flash BCH error byte address for error bit 4-5 Register */
#define REG_BCH_ECC_ADDR3	    (FMI_BA+0x10C)   	/* NAND Flash BCH error byte address for error bit 6-7 Register */
#define REG_BCH_ECC_ADDR4	    (FMI_BA+0x110)   	/* NAND Flash BCH error byte address for error bit 8-9 Register */
#define REG_BCH_ECC_ADDR5	    (FMI_BA+0x114)   	/* NAND Flash BCH error byte address for error bit 10-11 Register */
#define REG_BCH_ECC_ADDR6	    (FMI_BA+0x118)   	/* NAND Flash BCH error byte address for error bit 12-13 Register */
#define REG_BCH_ECC_ADDR7	    (FMI_BA+0x11C)   	/* NAND Flash BCH error byte address for error bit 14-15 Register */
#define REG_BCH_ECC_ADDR8	    (FMI_BA+0x120)   	/* NAND Flash BCH error byte address for error bit 16-17 Register */
#define REG_BCH_ECC_ADDR9	    (FMI_BA+0x124)   	/* NAND Flash BCH error byte address for error bit 18-19 Register */
#define REG_BCH_ECC_ADDR10	    (FMI_BA+0x128)   	/* NAND Flash BCH error byte address for error bit 20-21 Register */
#define REG_BCH_ECC_ADDR11	    (FMI_BA+0x12C)   	/* NAND Flash BCH error byte address for error bit 22-23 Register */

#define REG_BCH_ECC_DATA0	    (FMI_BA+0x160)   	/* NAND Flash BCH error data for error bit 0-3 Register */
#define REG_BCH_ECC_DATA1	    (FMI_BA+0x164)   	/* NAND Flash BCH error data for error bit 4-7 Register */
#define REG_BCH_ECC_DATA2	    (FMI_BA+0x168)   	/* NAND Flash BCH error data for error bit 8-11 Register */
#define REG_BCH_ECC_DATA3	    (FMI_BA+0x16C)   	/* NAND Flash BCH error data for error bit 12-15 Register */
#define REG_BCH_ECC_DATA4	    (FMI_BA+0x170)   	/* NAND Flash BCH error data for error bit 16-19 Register */
#define REG_BCH_ECC_DATA5	    (FMI_BA+0x174)   	/* NAND Flash BCH error data for error bit 20-23 Register */

#define REG_SMRA_0				(FMI_BA+0x200)   /* NAND Flash Redundant Area Register */
#define REG_SMRA_1		        (FMI_BA+0x204)
#define REG_SMRA_2		        (FMI_BA+0x208)
#define REG_SMRA_3		        (FMI_BA+0x20C)
#define REG_SMRA_4		        (FMI_BA+0x210)
#define REG_SMRA_5		        (FMI_BA+0x214)
#define REG_SMRA_6		        (FMI_BA+0x218)
#define REG_SMRA_7		        (FMI_BA+0x21C)
#define REG_SMRA_8		        (FMI_BA+0x220)
#define REG_SMRA_9		        (FMI_BA+0x224)
#define REG_SMRA_10		        (FMI_BA+0x228)
#define REG_SMRA_11		        (FMI_BA+0x22C)
#define REG_SMRA_12		        (FMI_BA+0x230)
#define REG_SMRA_13		        (FMI_BA+0x234)
#define REG_SMRA_14		        (FMI_BA+0x238)
#define REG_SMRA_15		        (FMI_BA+0x23C)
#define REG_SMRA_16		        (FMI_BA+0x240)
#define REG_SMRA_17		        (FMI_BA+0x244)
#define REG_SMRA_18		        (FMI_BA+0x248)
#define REG_SMRA_19		        (FMI_BA+0x24C)
#define REG_SMRA_20		        (FMI_BA+0x250)
#define REG_SMRA_21		        (FMI_BA+0x254)
#define REG_SMRA_22		        (FMI_BA+0x258)
#define REG_SMRA_23		        (FMI_BA+0x25C)
#define REG_SMRA_24		        (FMI_BA+0x260)
#define REG_SMRA_25		        (FMI_BA+0x264)
#define REG_SMRA_26		        (FMI_BA+0x268)
#define REG_SMRA_27		        (FMI_BA+0x26C)
#define REG_SMRA_28		        (FMI_BA+0x270)
#define REG_SMRA_29		        (FMI_BA+0x274)
#define REG_SMRA_30		        (FMI_BA+0x278)
#define REG_SMRA_31		        (FMI_BA+0x27C)
#define REG_SMRA_32		        (FMI_BA+0x280)
#define REG_SMRA_33		        (FMI_BA+0x284)
#define REG_SMRA_34		        (FMI_BA+0x288)
#define REG_SMRA_35		        (FMI_BA+0x28C)
#define REG_SMRA_36		        (FMI_BA+0x290)
#define REG_SMRA_37		        (FMI_BA+0x294)
#define REG_SMRA_38		        (FMI_BA+0x298)
#define REG_SMRA_39		        (FMI_BA+0x29C)
#define REG_SMRA_40		        (FMI_BA+0x2A0)
#define REG_SMRA_41		        (FMI_BA+0x2A4)
#define REG_SMRA_42		        (FMI_BA+0x2A8)
#define REG_SMRA_43		        (FMI_BA+0x2AC)
#define REG_SMRA_44		        (FMI_BA+0x2B0)
#define REG_SMRA_45		        (FMI_BA+0x2B4)
#define REG_SMRA_46		        (FMI_BA+0x2B8)
#define REG_SMRA_47		        (FMI_BA+0x2BC)
#define REG_SMRA_48		        (FMI_BA+0x2C0)
#define REG_SMRA_49		        (FMI_BA+0x2C4)
#define REG_SMRA_50		        (FMI_BA+0x2C8)
#define REG_SMRA_51		        (FMI_BA+0x2CC)
#define REG_SMRA_52		        (FMI_BA+0x2D0)
#define REG_SMRA_53		        (FMI_BA+0x2D4)
#define REG_SMRA_54		        (FMI_BA+0x2D8)   /* NAND Flash Redundant Area Register */


/*
 SDIO Control Registers
*/
#define REG_SDIOFB_0            (SDIODMAC_BA+0x000)  /* Shared Buffer (FIFO) */

#define	REG_SDIODMACCSR         (SDIODMAC_BA+0x400)  /* DMAC Control and Status Register */
#define REG_SDIODMACSAR         (SDIODMAC_BA+0x408)  /* DMAC Transfer Starting Address Register */
#define REG_SDIODMACBCR         (SDIODMAC_BA+0x40C)  /* DMAC Transfer Byte Count Register */
#define REG_SDIODMACIER         (SDIODMAC_BA+0x410)  /* DMAC Interrupt Enable Register */
#define REG_SDIODMACISR         (SDIODMAC_BA+0x414)  /* DMAC Interrupt Status Register */

/* SDIO Flash Memory Card Interface Registers */
#define REG_SDIOFMICR           (SDIOFMI_BA+0x000)  /* FMI Control Register */
#define REG_SDIOFMIIER          (SDIOFMI_BA+0x004)  /* FMI DMA Transfer Starting Address Register */
#define REG_SDIOFMIISR          (SDIOFMI_BA+0x008)  /* FMI DMA Byte Count Register */
/* Secure Digit Registers */
#define REG_SDIOCR              (SDIOFMI_BA+0x020)  /* SD Control Register */
#define REG_SDIOARG             (SDIOFMI_BA+0x024)  /* SD command argument register */
#define REG_SDIOIER             (SDIOFMI_BA+0x028)  /* SD interrupt enable register */
	#define	SDIER_CD1SRC			BIT31				// SDIO1 card detection source selection: SD-DAT3 or GPIO
	#define SDIER_CD1_IEN			BIT9				// SDIO1 card detect interrupt Enable
#define REG_SDIOISR             (SDIOFMI_BA+0x02C)  /* SD interrupt status register */
	#define SDISR_CD1_Card			BIT17			    // SDIO1 card detection pin status
	#define	SDISR_CD1_IF            BIT9			    // SDIO1 card detect interrupt flag
#define REG_SDIORSP0            (SDIOFMI_BA+0x030)  /* SD receive response token register 0 */
#define REG_SDIORSP1            (SDIOFMI_BA+0x034)  /* SD receive response token register 1 */
#define REG_SDIOBLEN            (SDIOFMI_BA+0x038)  /* SD block length register */
#define REG_SDIOTMOUT           (SDIOFMI_BA+0x03C)  /* SD RI/DI timeout register */


/* SPU Control Registers */
#define REG_SPU_CTRL			(SPU_BA+0x00)		// SPU control and status register
	#define SPU_SWRST				BIT16				// SPU SW reset
	#define RampUpMod	 			NVTBIT(14,12)		// Ramp up mode select
	#define SPU_I2S_EN					BIT8				// I2S output enable/disable
	#define SPU_END_CTRL			BIT2				// SPU END function control
	#define SPU_END_FLAG			BIT1				// SPU END function flag
	#define SPU_EN					BIT0				// SPU enable/disable

#define REG_SPU_DAC_PAR			(SPU_BA+0x04)		// DAC parameter register
	#define PLLSELMOD				BIT31				// Select the SPU PLL Clock comes from
	#define DAC_RST					BIT30				// DAC Reset Signal
	#define DAC_ZERO_MOD			BIT27				// DAC zer cross mode select, 1: digital gain, 0: analog gain
	#define DAC_ZERO_EN				BIT26				// DAC zero cross detection enable
	#define ZERO_EN					BIT25				// Zero cross detection enable
	#define EQU_EN					BIT24				// Equalizer enable

#define REG_SPU_DAC_AG			(SPU_BA+0x08)		// DAC auto gain control
	#define SAMPLE_STEP	 			NVTBIT(31,24)		
	#define GAIN_STEP				NVTBIT(23,16)		
	#define SAVE_GAIN				NVTBIT(15,8)		
	
#define REG_SPU_EQGain0			(SPU_BA+0x0C)		// Equalizer bands 08 - 01 gain control
	#define Gain08					NVTBIT(31,28)		// Gain08 control
	#define Gain07					NVTBIT(27,24)		// Gain07 control
	#define Gain06					NVTBIT(23,20)		// Gain06 control
	#define Gain05					NVTBIT(19,16)		// Gain05 control
	#define Gain04					NVTBIT(15,12)		// Gain04 control
	#define Gain03					NVTBIT(11,8)		// Gain03 control
	#define Gain02					NVTBIT(07,4)		// Gain02 control
	#define Gain01					NVTBIT(03,0)		// Gain01 control

#define REG_SPU_EQGain1			(SPU_BA+0x10)		// Equalizer bands 10 - 09 and DC gain control
	#define Gaindc					NVTBIT(19,16)		// DC control
	#define Gain10					NVTBIT(07,4)		// Gain10 control
	#define Gain09					NVTBIT(03,0)		// Gain09 control

#define REG_SPU_CH_EN			(SPU_BA+0x14)		// Channel enable register

#define REG_SPU_CH_IRQ			(SPU_BA+0x018)		// Channel iterrupt request flag register

#define REG_SPU_CH_PAUSE		(SPU_BA+0x1C)		// Channel PAUSE register

#define REG_SPU_CH_CTRL			(SPU_BA+0x20)		// Channel control register
	#define CH_NO					NVTBIT(28,24)		// Select chanel index number
	
	#define VIR_I2C_IRQ_EN			BIT17				// Virtual I2C interrupt enable
	#define VIR_I2C_IRQ_FG			BIT16				// Virtual I2C interrupt flag
	#define FN_IRQ_FG				BIT12				// Channel function done interrupt flag
	#define FN_IRQ_EN				BIT8				// Channel function done interrupt enable/disable
	#define UP_IRQ					BIT7				// Interrupt for DFA update in partial update function
	#define UP_DFA					BIT6				// DFA update in partial update function
	#define UP_PAN					BIT5				// PAN update in partial update function
	#define UP_VOL					BIT4				// Volume update in partial update function
	#define UP_PAUSE_ADDR			BIT3				// Pause Address update in partial update function	(only for mono/stereo PCM16 use)
	#define CH_FN					NVTBIT(1,0)			// Channel function register

#define REG_SPU_S_ADDR			(SPU_BA+0x24)		// Source start (base) address register

#define REG_SPU_M_ADDR			(SPU_BA+0x28)		// Threshold address register

#define REG_SPU_E_ADDR			(SPU_BA+0x2C)		// End start address register

#define REG_SPU_TONE_PULSE		(SPU_BA+0x28)		// Tone Pulse control register
	#define TONE_P1					NVTBIT(31,16)		// Tone Pulse 1
	#define TONE_P0					NVTBIT(15,0)		// Tone Pulse 0

#define REG_SPU_TONE_AMP		(SPU_BA+0x2C)		// Tone Amplitude control register
	#define TONE_AMP1				NVTBIT(31,16) 		// Tone Amplitude 1
	#define TONE_AMP0				NVTBIT(15,0)		// Tone Amplitude 0

#define REG_SPU_CH_PAR_1		(SPU_BA+0x30)		// Channel parameter 1 register
	#define CH_VOL					NVTBIT(30,24) 		// Channel volume register
	#define PAN_L					NVTBIT(20,16)		// Output right channel PAN
	#define PAN_R					NVTBIT(12,8)		// Output left channel PAN
	#define SRC_TYPE				NVTBIT(2,0)			// Channel sound type

#define REG_SPU_CH_PAR_2		(SPU_BA+0x34)		// Channel parameter 2 register
	#define DFA						NVTBIT(12,0)		// DFA

#define REG_SPU_CH_EVENT		(SPU_BA+0x38)		// DMA down counter register
	#define SUB_IDX					NVTBIT(29,24)		// Sub-index of user event
	#define EVENT_IDX				NVTBIT(23,16)		// Index of user event
	#define EV_USR_FG				BIT13				// User event interrupt flag
	#define EV_SLN_FG				BIT12				// Slient event interrupt flag
	#define EV_LP_FG				BIT11				// Loop Start event interrupt flag
	#define EV_PAUSE_FG				BIT11				// Pause Address interrupt flag
	#define EV_END_FG				BIT10				// End event interrupt flag
	#define END_FG					BIT9				// End address interrupt flag
	#define TH_FG					BIT8				// Threshold address interrupt flag

	#define AT_CLR_EN				BIT7				// Enable Bit for auto interrupt flag clear after read event register
	#define EV_USR_EN				BIT5				// Enable Bit for User event interrupt flag
	#define EV_SLN_EN				BIT4				// Enable Bit for Slient event interrupt flag
	#define EV_LP_EN				BIT3				// Enable Bit for Loop Start event interrupt flag
	#define EV_PAUSE_EN				BIT3				// Enable Bit for Pause Address event interrupt flag
	#define EV_END_EN				BIT2				// Enable Bit for End event interrupt flag
	#define END_EN					BIT1				// Enable Bit for End address
	#define TH_EN					BIT0				// Enable Bit for Threshold address

#define REG_SPU_CUR_ADDR		(SPU_BA+0x40)			// DMA down counter register

#define REG_SPU_LP_ADDR			(SPU_BA+0x44)			// DMA down counter register
#define REG_SPU_PA_ADDR			(SPU_BA+0x44)			// Pause Address for mono/stereo PCM16

#define REG_SPU_DAC_CTRL		(SPU_BA+0x50)			// Pause Address for mono/stereo PCM16
	#define V_I2C_BUSY				BIT31				// virtual I2C busy flag
	#define V_I2C_SCK_DIV				NVTBIT(30,24)		// virtual I2C clock divider, clock speed = SPU_CLK frequency/(SCK_DIV * 4)
	#define V_I2C_ID				NVTBIT(23,17)		// virtual I2C ID, default ID = 0x40
	#define V_I2C_RW				BIT16				// virtual I2C R/W signal
	#define V_I2C_ADDR				NVTBIT(15,8)		// virtual I2C address
	#define V_I2C_DATA				NVTBIT(7,0)			// virtual I2C data	

/* I2S Control Registers */

#define REG_I2S_ACTL_CON		(I2S_BA+0x00)		// Audio Control Register
	#define	P_PAUSE_IRQ_EN			BIT22				// Pause Function Interrupt Request enable Bit
	#define	R_DMA_IRQ_EN			BIT21				// Recording DMA Interrupt Request enable Bit
	#define P_DMA_IRQ_EN			BIT20				// Playback DMA Interrupt Request enable Bit
	#define R_FIFO_FULL_IRQ_EN		BIT19				// Recording FIFO full Interrupt Request enable Bit
	#define R_FIFO_EMPTY_IRQ_EN		BIT18				// Recording FIFO empty Interrupt Request enable Bit
	#define P_FIFO_FULL_IRQ_EN		BIT17				// Playback FIFO full Interrupt Request enable Bit
	#define P_FIFO_EMPTY_IRQ_EN		BIT16				// Playback FIFO empty Interrupt Request enable Bit
	#define R_DMA_IRQ_SEL			NVTBIT(15,14)		// Recording DMA Interrupt Request selection Bits
	#define P_DMA_IRQ_SEL			NVTBIT(13,12)		// Playback DMA Interrupt Request selection Bits

	#define R_DMA_IRQ				BIT11				// Playback DMA Interrupt Request Bit
	#define P_DMA_IRQ				BIT10				// Recording DMA Interrupt Request Bit
	#define I2S_BITS_16_24			BIT9				// 16/24 bits selection
	#define FIFO_TH					BIT7				// FIFO Threshold Control Bit
	#define IRQ_DMA_CNTER_EN		BIT4				// IRQ_DMA counter function enable Bit
	#define IRQ_DMA_DATA_ZERO_EN	BIT3				// IRQ_DMA_DATA zero and sign detect enable Bit
	#define I2S_EN					BIT1				// I2S interface enable Bit

#define	REG_I2S_ACTL_RESET		(I2S_BA+0x04)		// Sub block reset control
	#define ACTL_RESET_				BIT16				// Audio Controller Reset Control Bit
	#define RECORD_SINGLE			NVTBIT(15,14)		// Record Single/Dual Channel Select Bits
	#define PLAY_STEREO				BIT12				// Playback Single/Dual Channel Select Bits
	#define I2S_RECORD				BIT6				// I2S Record Control Bit
	#define I2S_PLAY				BIT5				// I2S Playback Control Bit
	#define DMA_CNTER_EN			BIT4				// DMA counter function enable Bit
	#define DMA_DATA_ZERO_EN		BIT3				// DMA_DATA zero and sign detect enable Bit
	#define AC_RESET				BIT1				// AC link Sub Block RESET Control Bit
	#define I2S_RESET				BIT0				// I2S Sub Block RESET Control Bit

#define	REG_I2S_ACTL_RDSTB		(I2S_BA+0x08)		// DMA record destination base address
#define REG_I2S_ACTL_RDST_LENGTH (I2S_BA+0x0C)		// DMA record destination address length
#define REG_I2S_ACTL_RDSTC		(I2S_BA+0x10)		// DMA record destination current address
#define REG_I2S_ACTL_PDSTB		(I2S_BA+0x14)		// DMA play destination base address
#define REG_I2S_ACTL_PDST_LENGTH (I2S_BA+0x18)		// DMA play destination address length
#define REG_I2S_ACTL_PDSTC		(I2S_BA+0x1C)		// DMA play destination current address

#define REG_I2S_ACTL_RSR 		(I2S_BA+0x20)		// Audio controller FIFO and DMA status register for playback
	#define R_DMA_RIA_SN			NVTBIT(7,5)			// Recording DMA inidicative address selection number Bits
	#define R_FIFO_FULL				BIT2				// Playback FIFO Full Indicatior Bit
	#define R_FIFO_EMPTY			BIT1				// Playback FIFO Empty Indicatior Bit
	#define R_DMA_RIA_IRQ			BIT0				// Recording DMA inidicative address interrupt Request Bit

#define REG_I2S_ACTL_PSR		(I2S_BA+0x24)		// Audio controller FIFO and DMA status register for playback
	#define P_PAUSE_IRQ				BIT8				// Pause Function Interrupt Request Bit
	#define P_DMA_RIA_SN			NVTBIT(7,5)			// Playback DMA inidicative address selection number Bits
	#define DMA_CNTER_IRQ			BIT4				// DMA counter IRQ
	#define DMA_DATA_ZERO_IRQ		BIT3				// DMA_DATA zero IRQ
	#define P_FIFO_FULL				BIT2				// Playback FIFO Full Indicatior Bit
	#define P_FIFO_EMPTY			BIT1				// Playback FIFO Empty Indicatior Bit
	#define P_DMA_RIA_IRQ			BIT0				// Playback DMA inidicative address interrupt Request Bit

#define REG_I2S_ACTL_I2SCON		(I2S_BA+0x28)		// I2S controll register
	#define PRS						NVTBIT(19,16)		// I2S Frequency Pre-scaler Selection Bits
//	define	MCLK_SEL1				BIT9				// MCLK clock selection when MCLK_CON is active
//	#define MCLK_CON				BIT8				// MCLK clock selection
	#define BCLK_SEL				NVTBIT(7,6)			// I2S Serial Data Clock Frequency Selection Bit
	#define FS_SEL					BIT5				// I2S Sampling Frequency Selection Bit
	#define MCLK_SEL				BIT4				// I2S MCLK Output Selection Bit
	#define I2S_FORMAT				BIT3				// I2S Format Selection Bit

#define REG_I2S_ACTL_COUNTER	(I2S_BA+0x2C)		// DMA down counter register



/*
	USB 2.0 EHCI Host Control
*/

#define REG_EHCVNR	(UEHCI20_BA+0x000)	//EHCI Version Number Register
#define REG_EHCSPR	(UEHCI20_BA+0x004)	//EHCI Structural Parameters Register
#define REG_EHCCPR	(UEHCI20_BA+0x008)	//EHCI Capability Parameters Register

#define REG_UCMDR	(UEHCI20_BA+0x020)	//USB Command Register	
#define REG_USTSR	(UEHCI20_BA+0x024)	//USB Status Register	
#define REG_UIENR	(UEHCI20_BA+0x028)	//USB Interrupt Enable Register	
#define REG_UFINDR	(UEHCI20_BA+0x02C)	//USB Frame Index Register	
#define REG_UPFLBAR	(UEHCI20_BA+0x034)	//USB Periodic Frame List Base Address Register	
#define REG_UCALAR	(UEHCI20_BA+0x038)	//USB Current Asynchronous List Address Register	
#define REG_UASSTR	(UEHCI20_BA+0x03C)	//USB Asynchronous Schedule Sleep Timer Register	

#define REG_UCFGR	(UEHCI20_BA+0x060)	//USB Configure Flag Register	
#define REG_UPSCR0	(UEHCI20_BA+0x064)	//USB Port 0 Status and Control Register	
#define REG_UPSCR1	(UEHCI20_BA+0x068)	//USB Port 0 Status and Control Register			/* NOT USE */
#define REG_UHCBIST	(UEHCI20_BA+0x0C0)	//USB Host Controller BIST Register	
#define REG_USBPCR0	(UEHCI20_BA+0x0C4)	//USB PHY 0 Control Register	
#define REG_USBPCR1	(UEHCI20_BA+0x0C8)	//USB PHY 0 Control Register						/* NOT USE */
#define REG_USBDBG	(UEHCI20_BA+0x0CC)	//USB Debug Register	





#define REG_HcRev		(UOHCI20_BA+0x800)	//Host Controller Revision Register	
#define REG_HcControl	(UOHCI20_BA+0x804)	//Host Controller Control Register	
#define REG_HcComSts	(UOHCI20_BA+0x808)	//Host Controller Command Status Register	
#define REG_HcIntSts	(UOHCI20_BA+0x80C)	//Host Controller Interrupt Status Register	
#define REG_HcIntEn		(UOHCI20_BA+0x810)	//Host Controller Interrupt Enable Register	
#define REG_HcIntDis	(UOHCI20_BA+0x814)	//Host Controller Interrupt Disable Register	
#define REG_HcHCCA		(UOHCI20_BA+0x818)	//Host Controller Communication Area Register	
#define REG_HcPerCED	(UOHCI20_BA+0x81C)	//Host Controller Period Current ED Register	
#define REG_HcCtrHED	(UOHCI20_BA+0x820)	//Host Controller Control Head ED Register	
#define REG_HcCtrCED	(UOHCI20_BA+0x824)	//Host Controller Control Current ED Register	
#define REG_HcBlkHED	(UOHCI20_BA+0x828)	//Host Controller Bulk Head ED Register	
#define REG_HcBlkCED	(UOHCI20_BA+0x82C)	//Host Controller Bulk Current ED Register	
#define REG_HcDoneH		(UOHCI20_BA+0x830)	//Host Controller Done Head Register	
#define REG_HcFmIntv	(UOHCI20_BA+0x834)	//Host Controller Frame Interval Register	
#define REG_HcFmRem		(UOHCI20_BA+0x838)	//Host Controller Frame Remaining Register	
#define REG_HcFNum		(UOHCI20_BA+0x83C)	//Host Controller Frame Number Register	
#define REG_HcPerSt		(UOHCI20_BA+0x840)	//Host Controller Periodic Start Register	
#define REG_HcLSTH		(UOHCI20_BA+0x844)	//Host Controller Low Speed Threshold Register	
#define REG_HcRhDeA		(UOHCI20_BA+0x848)	//Host Controller Root Hub Descriptor A Register	
#define REG_HcRhSts		(UOHCI20_BA+0x850)	//Host Controller Root Hub Status Register	
#define REG_HcRhPrt1	(UOHCI20_BA+0x854)	//Host Controller Root Hub Port Status [1]	
#define REG_HcRhPrt2	(UOHCI20_BA+0x858)	//Host Controller Root Hub Port Status [1]			/* NOT USE */
#define REG_OpModEn		(UOHCI20_BA+0xA04)	//USB Operational Mode Enable Register	





/*
	USB Host controller Registers (OHCI) 1.1
*/
#define REG_HC_REVISION         	(USBH_BA+0x000)	// HcRevision - Revision Register
#define REG_HC_CONTROL         	 	(USBH_BA+0x004)	// HcControl  - Control Register
#define REG_HC_CMD_STATUS       	(USBH_BA+0x008)	// HcCommandStatus - Command Status Register
#define REG_HC_INT_STATUS       	(USBH_BA+0x00C)	// HcInterruptStatus  - Interrupt Status Register
#define REG_HC_INT_ENABLE       	(USBH_BA+0x010)	// HcInterruptEnable - Interrupt Enable Register
#define REG_HC_INT_DISABLE      	(USBH_BA+0x014)	// HcInterruptDisable - Interrupt Disable Registe																																				r
#define REG_HC_HCCA             		(USBH_BA+0x018)	// HcHCCA - Communication Area Register
#define REG_HC_PERIOD_CURED     	(USBH_BA+0x01C)	// HcPeriodCurrentED - Period Current ED Register
#define REG_HC_CTRL_HEADED      	(USBH_BA+0x020)	// HcControlHeadED - Control Head ED Register
#define REG_HC_CTRL_CURED       	(USBH_BA+0x024)	// HcControlCurrentED - Control Current ED Regist																																				er
#define REG_HC_BULK_HEADED      	(USBH_BA+0x028)	// HcBulkHeadED - Bulk Head ED Register
#define REG_HC_BULK_CURED       	(USBH_BA+0x02C)	// HcBulkCurrentED - Bulk Current ED Register
#define REG_HC_DONE_HEAD        	(USBH_BA+0x030)	// HcBulkCurrentED - Done Head Register
#define REG_HC_FM_INTERVAL      	(USBH_BA+0x034)	// HcFmInterval - Frame Interval Register
#define REG_HC_FM_REMAINING     	(USBH_BA+0x038)	// HcFrameRemaining - Frame Remaining Register
#define REG_HC_FM_NUMBER        	(USBH_BA+0x03C)	// HcFmNumber - Frame Number Register
#define REG_HC_PERIOD_START     	(USBH_BA+0x040)	// HcPeriodicStart - Periodic Start Register
#define REG_HC_LS_THRESHOLD     	(USBH_BA+0x044)	// HcLSThreshold - Low Speed Threshold Register
#define REG_HC_RH_DESCRIPTORA   	(USBH_BA+0x048)	// HcRhDescriptorA - Root Hub Descriptor A Register
#define REG_HC_RH_DESCRIPTORB   	(USBH_BA+0x04C)	// HcRevision - Root Hub Descriptor B Register
#define REG_HC_RH_STATUS        	(USBH_BA+0x050)	// HcRhStatus - Root Hub Status Register
#define REG_HC_RH_PORT_STATUS1  	(USBH_BA+0x054)	// HcRevision - Root Hub Port Status [1]
#define REG_HC_RH_PORT_STATUS2  	(USBH_BA+0x058)	// HcRevision - Root Hub Port Status [2]
#define REG_HC_RH_OP_MODE          	(USBH_BA+0x204)

/* Jpeg Control Registers */
//#define JPEG_BASE	w55fa92_VA_JPEG		/* Jpeg Control */

#define REG_JMCR (JPG_BA+0x00)				// R/W: JPEG Mode Control Register
	#define RESUMEI		BIT9				// Resume JPEG Operation for Input On-the-Fly Mode
	#define RESUMEO		BIT8				// Resume JPEG Operation for Output On-the-Fly Mode
	#define ENC_DEC		BIT7				// JPEG Encode/Decode Mode
	#define WIN_DEC		BIT6				// JPEG Window Decode Mode
	#define PRI		BIT5				// Encode Primary Image
	#define THB		BIT4				// Encode Thumbnail Image
	#define EY422		BIT3				// Encode Image Format
	#define QT_BUSY		BIT2				// Quantization-Table Busy Status (Read-Only)
	#define ENG_RST		BIT1				// Soft Reset JPEG Engine (Except JPEG Control Registers)
	#define JPG_EN		BIT0				// JPEG Engine Operation Control

#define REG_JHEADER	(JPG_BA+0x04)			// R/W: JPEG Encode Header Control Register
	#define P_JFIF		BIT7				// Primary JPEG Bit-stream Include JFIF Header
	#define P_HTAB		BIT6				// Primary JPEG Bit-stream Include Huffman-Table
	#define P_QTAB		BIT5				// Primary JPEG Bit-stream Include Quantization-Table
	#define P_DRI		BIT4				// Primary JPEG Bit-stream Include Restart Interval
	#define T_JFIF		BIT3				// Thumbnail JPEG Bit-stream Include JFIF Header
	#define T_HTAB		BIT2				// Thumbnail JPEG Bit-stream Include Huffman-Table
	#define T_QTAB		BIT1				// Thumbnail JPEG Bit-stream Include Quantization-Table
	#define T_DRI		BIT0				// Thumbnail JPEG Bit-stream Include Restart Interval

#define REG_JITCR	(JPG_BA+0x08)			// R/W: JPEG Image Type Control Register
	#define Dec_Scatter_Gather	BIT18
	#define DEC_OTF	BIT17					// Decoder on the fly with VPE	
	#define ARGB8888	BIT16				// ARGB8888
	#define PLANAR_ON	BIT15				// Packet On
	#define ORDER		BIT14					// Decode Packet Data Order
	#define RGB_555_565	BIT13				// RGB555 & RGB565
	#define ROTATE		NVTBIT(12,11)			// Encode Image Rotate
	#define DYUV_MODE	NVTBIT(10,8)			// Decoded Image YUV Color Format (Read-Only)
	#define EXIF		BIT7				// Encode Quantization-Table & Huffman-Table Header Format Selection
	#define EY_ONLY		BIT6				// Encode Gray-level (Y-component Only) Image
	#define DHEND		BIT5				// Header Decode Complete Stop Enable
	#define DTHB		BIT4				// Decode Thumbnail Image Only
	#define E3QTAB		BIT3				// Numbers of Quantization-Table are Used For Encode
	#define D3QTAB		BIT2				// Numbers of Quantization-Table are Used For Decode (Read-Only)
	#define ERR_DIS		BIT1				// Decode Error Engine Abort
	#define PDHTAB		BIT0				// Programmable Huffman-Table Function For Decode

#define REG_JPRIQC	(JPG_BA+0x10)			// R/W: JPEG Primary Q-Table Control Register
	#define P_QADJUST	NVTBIT(7,4)			// Primary Quantization-Table Adjustment
	#define P_QVS		NVTBIT(3,0)			// Primary Quantization-Table Scaling Control

#define REG_JTHBQC	(JPG_BA+0x14)			// R/W: JPEG Thumbnail Q-Table Control Register
	#define T_QADJUST	NVTBIT(7,4)			// Thumbnail Quantization-Table Adjustment
	#define T_QVS		NVTBIT(3,0)			// Thumbnail Quantization-Table Scaling Control

#define REG_JPRIWH	(JPG_BA+0x18)			// R/W: JPEG Encode Primary Width/Height Register
	#define P_HEIGHT	NVTBIT(27,16)			// Primary Encode Image Height
	#define P_WIDTH		NVTBIT(11,0)			// Primary Encode Image Width

#define REG_JTHBWH	(JPG_BA+0x1C)			// R/W: JPEG Encode Thumbnail Width/Height Register
	#define T_HEIGHT	NVTBIT(27,16)			// Thumbnail Encode Image Height
	#define T_WIDTH		NVTBIT(11,0)			// Thumbnail Encode Image Width

#define REG_JPRST	(JPG_BA+0x20)			// R/W:  JPEG Encode Primary Restart Interval Register
	#define P_RST		NVTBIT(7,0)				// Primary Encode Restart Interval Value

#define REG_JTRST	(JPG_BA+0x24)			// R/W: JPEG Encode Thumbnail Restart Interval
	#define T_RST		NVTBIT(7,0)				// Thumbnail Encode Restart Interval Value

#define REG_JDECWH	(JPG_BA+0x28)			// R:  JPEG Decode Image Width/Height Register
	#define DEC_HEIGHT	NVTBIT(31,16)			// 13-bit Bit Stream Buffer threshold
	#define DEC_WIDTH	NVTBIT(15,0)			// 13-bit Header Offset Address

#define REG_JINTCR	(JPG_BA+0x2C)			// R/W:  JPEG Interrupt Control and Status Register
	#define JPG_DOW_INTE	BIT28
	#define JPG_DOW_INTS	BIT24
	#define JPG_WAITI	BIT23				// JPEG Input Wait Status (Read-Only)
	#define JPG_WAITO	BIT22				// JPEG Output Wait Status (Read-Only)
	#define BAbort		BIT16				// JPEG Memory Access Error Status (Read-Only)
	#define CER_INTE	BIT15				// Un-complete Capture On-The-Fly Frame Occur Interrupt Enable
	#define DHE_INTE	BIT14				// JPEG Header Decode End Wait Interrupt Enable
	#define IPW_INTE	BIT13				// Input Wait Interrupt Enable
	#define OPW_INTE	BIT12				// Output Wait Interrupt Enable
	#define ENC_INTE	BIT11				// Encode Complete Interrupt Enable
	#define DEC_INTE	BIT10				// Decode Complete Interrupt Enable
	#define DER_INTE	BIT9				// Decode Error Interrupt Enable
	#define EER_INTE	BIT8				// Encode (On-The-Fly) Error Interrupt Enable
	#define CER_INTS	BIT7				// Un-complete Capture On-The-Fly Frame Occur Interrupt Status
	#define DHE_INTS	BIT6				// JPEG  Header Decode End Wait Interrupt Status
	#define IPW_INTS	BIT5				// Input Wait Interrupt Status
	#define OPW_INTS	BIT4				// Output Wait Interrupt Status
	#define ENC_INTS	BIT3				// Encode Complete Interrupt Status
	#define DEC_INTS	BIT2				// Decode Complete Interrupt Status
	#define DER_INTS	BIT1				// Decode Error Interrupt Status
	#define EER_INTS	BIT0				// Encode (On-The-Fly) Error Interrupt Status

#define REG_JDOWFBS	(JPG_BA+0x3c)

#define REG_JPEG_BSBAD (JPG_BA+0x40)		// R/W:  JPEG Test Control Register
	#define BIST_ST		NVTBIT(23,16)			// Internal SRAM BIST Status (Read-Only)
	#define TEST_DOUT	NVTBIT(15,8)			// Test Data Output (Read-Only)
	#define TEST_ON		BIT7				// Test Enable
	#define BIST_ON		BIT6				// Internal SRAM BIST Mode Enable
	#define BIST_FINI	BIT5				// Internal SRAM BIST Mode Finish (Read-Only)
	#define BSBAD_BIST_FAIL	BIT4				// Internal SRAM BIST Mode Fail (Read-Only)
	#define TEST_SEL	NVTBIT(3,0)			// Test Data Selection

#define REG_JWINDEC0 (JPG_BA+0x44)			// R/W: JPEG Window Decode Mode Control Register 0
	#define MCU_S_Y		NVTBIT(24,16)			// MCU Start Position Y For Window Decode Mode
	#define MCU_S_X		NVTBIT(8,0)			// MCU Start Position X For Window Decode Mode

#define REG_JWINDEC1 (JPG_BA+0x48)			// R/W: JPEG Window Decode Mode Control Register 1
	#define MCU_E_Y		NVTBIT(24,16)			// MCU End Position Y For Window Decode Mode
	#define MCU_E_X		NVTBIT(8,0)			// MCU End Position X For Window Decode Mode

#define REG_JWINDEC2 (JPG_BA+0x4C)			// R/W:  JPEG Window Decode Mode Control Register 2
	#define WD_WIDTH	NVTBIT(11,0))			// Image Width (Y-Stride) For Window Decode Mode

#define REG_JMACR	 (JPG_BA+0x50)			// R/W: JPEG Memory Address Mode Control Register
	#define FLY_SEL		NVTBIT(29,24)			// Hardware Memory On-the-Fly Access Image Buffer-Size Selection for Encode
	#define FLY_TYPE	NVTBIT(23,22)			//
	#define BSF_SEL		NVTBIT(17,8)			// Memory On-the-Fly Access Bitstream Buffer-Size Selection
	#define FLY_ON		BIT7				// Hardware Memory On-the-Fly Access Mode
	#define IP_SF_ON	BIT3				// Software Memory On-the-Fly Access Mode for Data Input
	#define OP_SF_ON	BIT2				// Software Memory On-the-Fly Access Mode for Data Output
	#define ENC_MODE	NVTBIT(1,0)			// JPEG Memory Address Mode Control

#define REG_JPSCALU	(JPG_BA+0x54)			// R/W: JPEG Primary Scaling-Up Control Register
	#define JPSCALU_8X	BIT6				// Primary Image Up-Scaling For Encode
	#define A_JUMP		BIT2				// Reserve Buffer Size In JPEG Bit-stream For Software Application


#define REG_JPSCALD (JPG_BA+0x58)		// R/W: JPEG Primary Scaling-Down Control Register
	#define PSX_ON		BIT15				// Primary Image Horizontal Down-Scaling For Encode/Decode
	#define PS_LPF_ON	BIT14				// Primary Image Down-Scaling Low Pass Filter For Decode
	#define PSCALX_F	NVTBIT(12,8)			// Primary Image Horizontal Down-Scaling Factor
	#define PSCALY_F	NVTBIT(5,0)			// Primary Image Vertical Down-Scaling Factor

#define REG_JTSCALD	(JPG_BA+0x5C)			// R/W: JPEG Thumbnail  Scaling-Down Control Register
	#define TSX_ON		BIT15				// Thumbnail Image Horizontal Down-Scaling For Encode/Decode
	#define TSCALX_F	NVTBIT(14,8)			// Thumbnail Image Horizontal Down-Scaling Factor
	#define TSCALY_F	NVTBIT(7,0)			// Thumbnail Image Vertical Down-Scaling Factor

#define REG_JDBCR	(JPG_BA+0x60)			// R/W: JPEG Dual-Buffer Control Register
	#define DBF_EN		BIT7				// Dual Buffering Control
	#define IP_BUF		BIT4				// Input Dual Buffer Control

#define REG_JRESERVE (JPG_BA+0x70)			// R/W: JPEG Encode Primary Bit-stream Reserved Size Register
	#define RES_SIZE	NVTBIT(15,0)			// Primary Encode Bit-stream Reserved Size

#define REG_JOFFSET	(JPG_BA+0x74)			// R/W: JPEG Offset Between Primary & Thumbnail Register
	#define OFFSET_SIZE	NVTBIT(23,0)		// Primary/Thumbnail Starting Address Offset Size

#define REG_JFSTRIDE (JPG_BA+0x78)			// R/W: JPEG Encode Bit-stream Frame Stride Register
	#define F_STRIDE	NVTBIT(23,0)		// JPEG Encode Bit-stream Frame Stride

#define REG_JYADDR0 (JPG_BA+0x7C)			// R/W: JPEG Y Component Frame Buffer-0 Starting Address Register
	#define Y_IADDR0	NVTBIT(31,0)			// JPEG Y Component Frame Buffer-0 Starting Address

#define REG_JUADDR0 (JPG_BA+0x80)			// R/W: JPEG U Component Frame Buffer-0 Starting Address Register
	#define U_IADDR0	NVTBIT(31,0)			// JPEG U Component Frame Buffer-0 Starting Address

#define REG_JVADDR0	(JPG_BA+0x84)			// R/W: JPEG V Component Frame Buffer-0 Starting Address Register
	#define V_IADDR0	NVTBIT(31,0)			// JPEG V Component Frame Buffer-0 Starting Address

#define REG_JYADDR1 (JPG_BA+0x88)			// R/W: JPEG Y Component Frame Buffer-1 Starting Address Register
	#define Y_IADDR1	NVTBIT(31,0)			// JPEG Y Component Frame Buffer-1 Starting Address

#define REG_JUADDR1 (JPG_BA+0x8C)			// R/W: JPEG U Component Frame Buffer-1 Starting Address Register
	#define U_IADDR1	NVTBIT(31,0)			// JPEG U Component Frame Buffer-1 Starting Address

#define REG_JVADDR1 (JPG_BA+0x90)			// R/W: JPEG V Component Frame Buffer-1 Starting Address Register
	#define V_IADDR1	NVTBIT(31,0)			// JPEG V Component Frame Buffer-1 Starting Address

#define REG_JYSTRIDE (JPG_BA+0x94)			// R/W: JPEG Y Component Frame Buffer Stride Register
	#define Y_STRIDE	NVTBIT(11,0)			// JPEG Y Component Frame Buffer Stride

#define REG_JUSTRIDE (JPG_BA+0x98)			// R/W: JPEG U Component Frame Buffer Stride Register
	#define U_STRIDE	NVTBIT(11,0)			// JPEG U Component Frame Buffer Stride

#define REG_JVSTRIDE (JPG_BA+0x9C)			// R/W: JPEG V Component Frame Buffer Stride Register
	#define V_STRIDE	NVTBIT(11,0)			// JPEG V Component Frame Buffer Stride

#define REG_JIOADDR0 (JPG_BA+0xA0)			// R/W: JPEG Bit-stream Frame Buffer-0 Starting Address Register
	#define IO_IADDR0	NVTBIT(31,0)			// JPEG Bit-stream Frame Buffer-0 Starting Address

#define REG_JIOADDR1 (JPG_BA+0xA4)			// R/W: JPEG Bit-stream Frame Buffer-1 Starting Address Register
	#define IO_IADDR1	NVTBIT(31,0)			// JPEG Bit-stream Frame Buffer-1 Starting Address

#define REG_JPRI_SIZE (JPG_BA+0xA8)		// R  : JPEG Encode Primary Image Bit-stream Size Register
	#define PRI_SIZE	NVTBIT(23,0)			// JPEG Primary Image Encode Bit-stream Size

#define REG_JTHB_SIZE (JPG_BA+0xAC)		// R  : JPEG Encode Thumbnail Image Bit-stream Size Register
	#define THB_SIZE	NVTBIT(15,0)			// JPEG Thumbnail Image Encode Bit-stream Size

#define REG_JUPRAT (JPG_BA+0xB0)			// R/W: JPEG Encode Up-Scale Ratio Register
	#define S_HEIGHT	NVTBIT(29,16)			// JPEG Image Height Up-Scale Ratio
	#define S_WIDTH		NVTBIT(13,0)			// JPEG Image Width Up-Scale Ratio

#define REG_JBSFIFO (JPG_BA+0xB4)			// R/W: JPEG Bit-stream FIFO Control Register
	#define BSFIFO_HT	NVTBIT(6,4)			// Bit-stream FIFO High-Threshold Control
	#define BSFIFO_LT	NVTBIT(2,0)			// Bit-stream FIFO Low-Threshold Control

#define REG_JSRCH (JPG_BA+0xB8)			// R/W: JPEG Bit-stream FIFO Control Register
	#define JSRCH_JSRCH	NVTBIT(11,0)				// JPEG Encode Source Image Height

#define REG_JQTAB0 (JPG_BA+0x100)		// R/W: JPEG Quantization-Table 0 Register

#define REG_JQTAB1 (JPG_BA+0x140)		// R/W: JPEG Quantization-Table 1 Register

#define REG_JQTAB2 (JPG_BA+0x180)		// R/W: JPEG Quantization-Table 2 Register


/* Advance Interrupt Controller (AIC) Registers */
//#define AIC_BA		w55fa92_VA_IRQ		/* Interrupt Controller */
#define REG_AIC_SCR1    (AIC_BA+0x000)
#define REG_AIC_IRSR    (AIC_BA+0x100)   /* Interrupt raw status register */
#define REG_AIC_IRSRH   	(AIC_BA+0x104)   /* Interrupt raw status register */
#define REG_AIC_IASR    	(AIC_BA+0x108)   /* Interrupt active status register */
#define REG_AIC_IASRH   (AIC_BA+0x10C)   /* Interrupt active status register */
#define REG_AIC_ISR     	(AIC_BA+0x110)   /* Interrupt status register */
#define REG_AIC_ISRH    	(AIC_BA+0x114)   /* Interrupt status register */
#define REG_AIC_IPER    	(AIC_BA+0x118)   /* Interrupt priority encoding register */
#define REG_AIC_ISNR   	(AIC_BA+0x120)   /* Interrupt source number register */
#define REG_AIC_OISR    	(AIC_BA+0x124)   /* Output interrupt status register */
#define REG_AIC_IMR     	(AIC_BA+0x128)   /* Interrupt mask register */
#define REG_AIC_IMRH     (AIC_BA+0x12C)   /* Interrupt mask register */
#define REG_AIC_MECR    (AIC_BA+0x130)   /* Mask enable command register */
#define REG_AIC_MECRH 	(AIC_BA+0x134)   /* Mask enable command register */
#define REG_AIC_MDCR    (AIC_BA+0x138)   /* Mask disable command register */
#define REG_AIC_MDCRH  (AIC_BA+0x13C)   /* Mask disable command register */
#define REG_AIC_SSCR    	(AIC_BA+0x140)	 /* Source set command register */
#define REG_AIC_SSCRH  (AIC_BA+0x144)	 /* Source set command register */
#define REG_AIC_SCCR    (AIC_BA+0x148)   /* Source clear command register */
#define REG_AIC_SCCRH   (AIC_BA+0x14C)   /* Source clear command register */
#define REG_AIC_EOSCR 	(AIC_BA+0x150)   /* End of service command register */
#define REG_AIC_TEST    (AIC_BA+0x160)   /* ICE/Debug mode register */
#define AIC_IPER        (0x118)
#define AIC_ISNR        (0x120)
#define AIC_EOSCR       (0x150)



#define W55FA92_COM_TX    (0x00)
#define W55FA92_COM_RX    (0x00)
#define W55FA92_COM_IER   (0x04)
#define W55FA92_COM_FCR   (0x08)
#define W55FA92_COM_LCR   (0x0C)
#define W55FA92_COM_MCR   (0x10)
#define W55FA92_COM_MSR   (0x14)
#define W55FA92_COM_FSR   (0x18)
#define W55FA92_COM_ISR   (0x1C)
#define W55FA92_COM_TOR   (0x20)
#define W55FA92_COM_BAUD  (0x24)

#define UART_FCR_RTS_Tri_lev		NVTBIT(19,16)	// RTS Trigger Level
	#define UARTx_FCR_RTS_LEVEL1	0x00000
	#define UARTx_FCR_RTS_LEVEL4	0x10000
	#define UARTx_FCR_RTS_LEVEL8	0x20000
	#define UARTx_FCR_RTS_LEVEL14	0x30000
	#define UARTx_FCR_RTS_LEVEL30	0x40000
	#define UARTx_FCR_RTS_LEVEL46	0x50000
	#define UARTx_FCR_RTS_LEVEL62	0x60000
#define UART_FCR_RFITL			NVTBIT(7,4)	// RX FIFO Interrupt (Irpt_RDA) Trigger Level
	#define UARTx_FCR_FIFO_LEVEL1	0x00
	#define UARTx_FCR_FIFO_LEVEL4	0x10
	#define UARTx_FCR_FIFO_LEVEL8	0x20
	#define UARTx_FCR_FIFO_LEVEL14	0x30
	#define UARTx_FCR_FIFO_LEVEL30	0x40
	#define UARTx_FCR_FIFO_LEVEL46	0x50
	#define UARTx_FCR_FIFO_LEVEL62	0x60
#define UART_FCR_TFR			BIT2		// TX Software Reset
#define UART_FCR_RFR			BIT1		// RX Software Reset

#define UART_TXRXFIFO_RESET     (UART_FCR_RFR | UART_FCR_TFR)

#define UART_FSR_ROE    0x00000000      // Rx Overrun error
#define UART_FSR_PE     0x00000010      // Parity error
#define UART_FSR_FE     0x00000020      // Frame error
#define UART_FSR_BI     0x00000040      // Break interrupt
#define UART_FSR_RFE    0x00004000      // Rx FIFO empty
#define UART_FSR_RFF    0x00008000      // Rx FIFO full
#define UART_FSR_RPMASK (0x00003F00)    // Rx FIFO pointer
#define UART_FSR_TFE    0x00400000      // Tx FIFO empty
#define UART_FSR_TFF    0x00800000      // Tx FIFO full
#define UART_FSR_TPMASK (0x003F0000)    // Tx FIFO pointer
#define UART_FSR_TOE    0x01000000      // Tx Overrun error
#define UART_FSR_TEMT   0x10000000      // Transmitter empty

#define W55FA92_FSRSTAT_ANY     (UART_FSR_ROE | UART_FSR_TOE | UART_FSR_FE | UART_FSR_BI)

#define UART_LCR_WLEN5  0x00
#define UART_LCR_WLEN6  0x01
#define UART_LCR_WLEN7  0x02
#define UART_LCR_WLEN8  0x03
#define UART_LCR_CSMASK (0x3)
#define UART_LCR_PARITY 0x08
#define UART_LCR_NPAR   0x00
#define UART_LCR_OPAR   0x00
#define UART_LCR_EPAR   0x10
#define UART_LCR_PMMASK (0x30)
#define UART_LCR_SPAR   0x20
#define UART_LCR_SBC    0x40
#define UART_LCR_NSB    0x00
#define UART_LCR_NSB1_5 0x04

#define UART_IER_nDBGACK		BIT31		// ICE debug mode acknowledge enable
#define UART_IER_EDMA_RX		BIT15		// RX EDMA Enable
#define UART_IER_EDMA_TX		BIT14		// TX EDMA Enable
#define UART_IER_Auto_CTS		BIT13		// CTS Auto Flow Control Enable
#define UART_IER_Auto_RTS		BIT12		// RTS Auto Flow Control Enable
#define UART_IER_TOUT			BIT11		// Time Out Counter Enable
#define UART_IER_Wake			BIT6		// Wake up interrupt enable for INTR[wakeup]
#define UART_IER_BUF_ERR		BIT5		// Buffer Error interrupt enable
#define UART_IER_RTO			BIT4		// RX Time out Interrupt Enable
#define UART_IER_MSI			BIT3		// MODEM Status Interrupt (Irpt_MOS) Enable
#define UART_IER_RLSI			BIT2		// Receive Line Status Interrupt (Irpt_RLS) Enable
#define UART_IER_THRI			BIT1		// Transmit Holding Register Empty Interrupt (Irpt_THRE) Enable
#define UART_IER_RDI			BIT0		// Receive Data Available Interrupt (Irpt_RDA) Enable

#define UART_ISR_EDMA_RX_Flag           BIT31           // EDMA RX Mode Flag
#define UART_ISR_HW_Wake_INT            BIT30           // Wake up Interrupt pin status
#define UART_ISR_HW_Buf_Err_INT         BIT29           // Buffer Error Interrupt pin status
#define UART_ISR_HW_Tout_INT            BIT28           // Time out Interrupt pin status
#define UART_ISR_HW_Modem_INT           BIT27           // MODEM Status Interrupt pin status
#define UART_ISR_HW_RLS_INT             BIT26           // Receive Line Status Interrupt pin status
#define UART_ISR_Rx_ack_st              BIT25           // TX ack pin status
#define UART_ISR_Rx_req_St              BIT24           // TX req pin status
#define UART_ISR_EDMA_TX_Flag           BIT23           // EDMA TX Mode Flag
#define UART_ISR_HW_Wake_IF             BIT22           // Wake up Flag
#define UART_ISR_HW_Buf_Err_IF          BIT21           // Buffer Error Flag
#define UART_ISR_HW_Tout_IF             BIT20           // Time out Flag
#define UART_ISR_HW_Modem_IF            BIT19           // MODEM Status Flag
#define UART_ISR_HW_RLS_IF              BIT18           // Receive Line Status Flag
#define UART_ISR_Tx_ack_st              BIT17           // TX ack pin status
#define UART_ISR_Tx_req_St              BIT16           // TX req pin status
#define UART_ISR_Soft_RX_Flag           BIT15           // Software RX Mode Flag
#define UART_ISR_Wake_INT               BIT14           // Wake up Interrupt pin status
#define UART_ISR_Buf_Err_INT            BIT13           // Buffer Error Interrupt pin status
#define UART_ISR_Tout_INT               BIT12           // Time out interrupt Interrupt pin status
#define UART_ISR_Modem_INT              BIT11           // MODEM Status Interrupt pin status
#define UART_ISR_RLS_INT                BIT10           // Receive Line Status Interrupt pin status
#define UART_ISR_THRE_INT               BIT9            // Transmit Holding Register Empty Interrupt pin status
#define UART_ISR_RDA_INT                BIT8            // Receive Data Available Interrupt pin status
#define UART_ISR_Soft_TX_Flag           BIT7            // Software TX Mode Flag
#define UART_ISR_Wake_IF                BIT6            // Wake up Flag
#define UART_ISR_Buf_Err_IF             BIT5            // Buffer Error Flag
#define UART_ISR_Tout_IF                BIT4            // Time out interrupt Flag
#define UART_ISR_Modem_IF               BIT3            // MODEM Status Flag
#define UART_ISR_RLS_IF                 BIT2            // Receive Line Status Flag
#define UART_ISR_THRE_IF                BIT1            // Transmit Holding Register Empty Flag
#define UART_ISR_RDA_IF                 BIT0            // Receive Data Available Flag

#define UART_MCR_RTS_st			BIT13
#define UART_MCR_Lev_RTS		BIT9
#define UART_MCR_LBME			BIT4		// Loop-back Mode Enable
#define UART_MCR_RTS			BIT1		// Complement version of DTR# (Data-Terminal-Ready) signal

#define UART_MSR_Lev_CTS		BIT8
#define UART_MSR_DCD			BIT7		// Complement version of Data Carrier Detect (nDCD#) input
#define UART_MSR_RI			BIT6		// Complement version of ring indicator (RI#) input
#define UART_MSR_DSR			BIT5		// Complement version of data set ready (DSR#) input
#define UART_MSR_CTS_st			BIT4		// Complement version of clear to send (CTS#) input
#define UART_MSR_DDCD			BIT3		// DCD# State Change
#define UART_MSR_TERI			BIT2		// Tailing Edge of RI#
#define UART_MSR_DDSR			BIT1		// DSR# State Change
#define UART_MSR_DCTS			BIT0		// CTS# State Change








/* GPIO Control Registers */
//#define GPIO_BA		w55fa92_VA_GPIO		/* GPIO Control */
#define REG_GPIOA_OMD   (GPIO_BA+0x0000)   // GPIO port A bit Output mode Enable
#define REG_GPIOA_PUEN  (GPIO_BA+0x0004)	 // GPIO port A Bit Pull-up Resistor Enable
#define REG_GPIOA_DOUT  (GPIO_BA+0x0008)   // GPIO port A data output value
#define REG_GPIOA_PIN   (GPIO_BA+0x000C)	 // GPIO port A Pin Value

#define REG_GPIOB_OMD   (GPIO_BA+0x0010)   // GPIO port B bit Output mode Enable
#define REG_GPIOB_PUEN  (GPIO_BA+0x0014)	 // GPIO port B Bit Pull-up Resistor Enable
#define REG_GPIOB_DOUT  (GPIO_BA+0x0018)   // GPIO port B data output value
#define REG_GPIOB_PIN   (GPIO_BA+ 0x001C)	 // GPIO port B Pin Value

#define REG_GPIOC_OMD   (GPIO_BA+0x0020)  // GPIO port C bit Output mode Enable
#define REG_GPIOC_PUEN  (GPIO_BA+0x0024)	// GPIO port C Bit Pull-up Resistor Enable
#define REG_GPIOC_DOUT  (GPIO_BA+0x0028)  // GPIO port C data output value
#define REG_GPIOC_PIN   (GPIO_BA+0x002C)	// GPIO port C Pin Value

#define REG_GPIOD_OMD   (GPIO_BA+0x0030)  // GPIO port D bit Output mode Enable
#define REG_GPIOD_PUEN  (GPIO_BA+0x0034)	// GPIO port D Bit Pull-up Resistor Enable
#define REG_GPIOD_DOUT  (GPIO_BA+0x0038)  // GPIO port D data output value
#define REG_GPIOD_PIN   (GPIO_BA+0x003C)	// GPIO port D Pin Value

#define REG_GPIOE_OMD   (GPIO_BA+0x0040)  // GPIO port E bit Output mode Enable
#define REG_GPIOE_PUEN  (GPIO_BA+0x0044)	// GPIO port E Bit Pull-up Resistor Enable
#define REG_GPIOE_DOUT  (GPIO_BA+0x0048)  // GPIO port E data output value
#define REG_GPIOE_PIN   (GPIO_BA+ 0x004C)	// GPIO port E Pin Value

#define REG_GPIOG_OMD   (GPIO_BA+0x0050)  // GPIO port G bit Output mode Enable
#define REG_GPIOG_PUEN  (GPIO_BA+0x0054)	// GPIO port G Bit Pull-up Resistor Enable
#define REG_GPIOG_DOUT  (GPIO_BA+0x0058)  // GPIO port G data output value
#define REG_GPIOG_PIN   (GPIO_BA+ 0x005C)	// GPIO port G Pin Value

#define REG_GPIOH_OMD   (GPIO_BA+0x0060)  // GPIO port H bit Output mode Enable
#define REG_GPIOH_PUEN  (GPIO_BA+0x0064)	// GPIO port H Bit Pull-up Resistor Enable
#define REG_GPIOH_DOUT  (GPIO_BA+0x0068)  // GPIO port H data output value
#define REG_GPIOH_PIN   (GPIO_BA+ 0x006C)	// GPIO port H Pin Value

#define REG_DBNCECON   (GPIO_BA+0x0070)  // External Interrupt Debounce Control

#define REG_IRQSRCGPA  (GPIO_BA+0x0080)  // GPIO Port A IRQ Source Grouping
#define REG_IRQSRCGPB  (GPIO_BA+0x0084)  // GPIO Port B IRQ Source Grouping
#define REG_IRQSRCGPC  (GPIO_BA+0x0088)  // GPIO Port C IRQ Source Grouping
#define REG_IRQSRCGPD  (GPIO_BA+0x008C)  // GPIO Port D IRQ Source Grouping
#define REG_IRQSRCGPE  (GPIO_BA+0x0090)  // GPIO Port E IRQ Source Grouping
#define REG_IRQSRCGPG  (GPIO_BA+0x0094)  // GPIO Port G IRQ Source Grouping
#define REG_IRQSRCGPH  (GPIO_BA+0x0098)  // GPIO Port H IRQ Source Grouping

#define REG_IRQENGPA   (GPIO_BA+0x00A0)  // GPIO Port A Interrupt Enable
#define REG_IRQENGPB   (GPIO_BA+0x00A4)  // GPIO Port B Interrupt Enable
#define REG_IRQENGPC   (GPIO_BA+0x00A8)  // GPIO Port C Interrupt Enable
#define REG_IRQENGPD   (GPIO_BA+0x00AC)  // GPIO Port D Interrupt Enable
#define REG_IRQENGPE   (GPIO_BA+0x00B0)  // GPIO Port E Interrupt Enable
#define REG_IRQENGPG   (GPIO_BA+0x00B4)  // GPIO Port G Interrupt Enable
#define REG_IRQENGPH   (GPIO_BA+0x00B8)  // GPIO Port H Interrupt Enable

#define REG_IRQLHSEL   (GPIO_BA+0x00C0)  // Interrupt Latch Trigger Selection Register

#define REG_IRQLHGPA   (GPIO_BA+0x00D0)  // GPIO Port A Interrupt Latch Value
#define REG_IRQLHGPB   (GPIO_BA+0x00D4)  // GPIO Port B Interrupt Latch Value
#define REG_IRQLHGPC   (GPIO_BA+0x00D8)  // GPIO Port C Interrupt Latch Value
#define REG_IRQLHGPD   (GPIO_BA+0x00DC)  // GPIO Port D Interrupt Latch Value
#define REG_IRQLHGPE   (GPIO_BA+0x00E0)  // GPIO Port E Interrupt Latch Value
#define REG_IRQLHGPG   (GPIO_BA+0x00E4)  // GPIO Port G Interrupt Latch Value
#define REG_IRQLHGPH   (GPIO_BA+0x00E8)  // GPIO Port H Interrupt Latch Value

#define REG_IRQTGSRC0   (GPIO_BA+0x00F0) // IRQ0~3 Interrupt Trigger Source Indicator from GPIO Port A and GPIO Port B
#define REG_IRQTGSRC1   (GPIO_BA+0x00F4) // IRQ0~3 Interrupt Trigger Source Indicator from GPIO Port C and GPIO Port D
#define REG_IRQTGSRC2   (GPIO_BA+0x00F8) // IRQ0~3 Interrupt Trigger Source Indicator from GPIO Port E and GPIO Port G
#define REG_IRQTGSRC3   (GPIO_BA+0x00FC) // IRQ0~3 Interrupt Trigger Source Indicator from GPIO Port H


//#define PA_VA_USB_BASE	w55fa92_VA_USBD

#define REG_USBD_IRQ_STAT_L			(USBD_BA+0x00)		/* interrupt status low register */
#define REG_USBD_IRQ_ENB_L		(USBD_BA+0x08)		/* interrupt enable low register */
#define REG_USBD_IRQ_STAT		(USBD_BA+0x10)		/* usb interrupt status register */
#define REG_USBD_IRQ_ENB		(USBD_BA+0x14)		/* usb interrupt enable register */
#define REG_USBD_OPER		(USBD_BA+0x18)		/* usb operation register */
#define REG_USBD_FRAME_CNT		(USBD_BA+0x1c)		/* usb frame count register */
#define REG_USBD_ADDR		(USBD_BA+0x20)		/* usb address register */
#define REG_USBD_TEST		(USBD_BA+0x24)		/* usb test mode register */
#define REG_USBD_CEP_DATA_BUF	(USBD_BA+0x28)		/* control-ep data buffer register */
#define REG_USBD_CEP_CTRL_STAT	(USBD_BA+0x2c)		/* control-ep control and status register */
#define REG_USBD_CEP_IRQ_ENB	(USBD_BA+0x30)		/* control-ep interrupt enable register */
#define REG_USBD_CEP_IRQ_STAT	(USBD_BA+0x34)		/* control-ep interrupt status register */
#define REG_USBD_IN_TRNSFR_CNT	(USBD_BA+0x38)		/* in-transfer data count register */
#define REG_USBD_OUT_TRNSFR_CNT	(USBD_BA+0x3c)		/* out-transfer data count register */
#define REG_USBD_CEP_CNT		(USBD_BA+0x40)		/* control-ep data count register */
#define REG_USBD_SETUP1_0		(USBD_BA+0x44)		/* setup byte1 & byte0 register */
#define REG_USBD_SETUP3_2		(USBD_BA+0x48)		/* setup byte3 & byte2 register */
#define REG_USBD_SETUP5_4		(USBD_BA+0x4c)		/* setup byte5 & byte4 register */
#define REG_USBD_SETUP7_6		(USBD_BA+0x50)		/* setup byte7 & byte6 register */
#define REG_USBD_CEP_START_ADDR	(USBD_BA+0x54)		/* control-ep ram start address register */
#define REG_USBD_CEP_END_ADDR	(USBD_BA+0x58)		/* control-ep ram end address register */
#define REG_USBD_DMA_CTRL_STS	(USBD_BA+0x5c)		/* dma control and status register */
#define REG_USBD_DMA_CNT		(USBD_BA+0x60)		/* dma count register */

/* endpoint A/B/C/D/E/F data buffer register */
#define REG_USBD_EPA_DATA_BUF	(USBD_BA+0x64)		/* endpoint A data buffer register */
#define REG_USBD_EPB_DATA_BUF	(USBD_BA+0x8c)		/* endpoint B data buffer register */
#define REG_USBD_EPC_DATA_BUF	(USBD_BA+0xb4)		/* endpoint C data buffer register */
#define REG_USBD_EPD_DATA_BUF	(USBD_BA+0xdc)		/* endpoint D data buffer register */
#define REG_USBD_EPE_DATA_BUF	(USBD_BA+0x104)	/* endpoint E data buffer register */
#define REG_USBD_EPF_DATA_BUF	(USBD_BA+0x12c)	/* endpoint F data buffer register */

/* endpoint A/B/C/D/E/F interrupt status register */
#define REG_USBD_EPA_IRQ_STAT	(USBD_BA+0x68)		/* endpoint A interrupt status register */
#define REG_USBD_EPB_IRQ_STAT	(USBD_BA+0x90)		/* endpoint B interrupt status register */
#define REG_USBD_EPC_IRQ_STAT	(USBD_BA+0xb8)		/* endpoint C interrupt status register */
#define REG_USBD_EPD_IRQ_STAT	(USBD_BA+0xe0)		/* endpoint D interrupt status register */
#define REG_USBD_EPE_IRQ_STAT	(USBD_BA+0x108)	/* endpoint E interrupt status register */
#define REG_USBD_EPF_IRQ_STAT	(USBD_BA+0x130)	/* endpoint F interrupt status register */

/* endpoint A/B/C/D/E/F interrupt enable register */
#define REG_USBD_EPA_IRQ_ENB	(USBD_BA+0x6c)		/* endpoint A interrupt enable register */
#define REG_USBD_EPB_IRQ_ENB	(USBD_BA+0x94)		/* endpoint B interrupt enable register */
#define REG_USBD_EPC_IRQ_ENB	(USBD_BA+0xbc)		/* endpoint C interrupt enable register */
#define REG_USBD_EPD_IRQ_ENB	(USBD_BA+0xe4)		/* endpoint D interrupt enable register */
#define REG_USBD_EPE_IRQ_ENB	(USBD_BA+0x10c)	/* endpoint E interrupt enable register */
#define REG_USBD_EPF_IRQ_ENB	(USBD_BA+0x134)	/* endpoint F interrupt enable register */

/* data count available in endpoint A/B/C/D/E/F buffer */
#define REG_USBD_EPA_DATA_CNT	(USBD_BA+0x70)		/* data count available in endpoint A buffer */
#define REG_USBD_EPB_DATA_CNT	(USBD_BA+0x98)		/* data count available in endpoint B buffer */
#define REG_USBD_EPC_DATA_CNT	(USBD_BA+0xc0)		/* data count available in endpoint C buffer */
#define REG_USBD_EPD_DATA_CNT	(USBD_BA+0xe8)		/* data count available in endpoint D buffer */
#define REG_USBD_EPE_DATA_CNT	(USBD_BA+0x110)	/* data count available in endpoint E buffer */
#define REG_USBD_EPF_DATA_CNT	(USBD_BA+0x138)	/* data count available in endpoint F buffer */

/* endpoint A/B/C/D/E/F response register set/clear */
#define REG_USBD_EPA_RSP_SC		(USBD_BA+0x74)		/* endpoint A response register set/clear */
#define REG_USBD_EPB_RSP_SC		(USBD_BA+0x9c)		/* endpoint B response register set/clear */
#define REG_USBD_EPC_RSP_SC		(USBD_BA+0xc4)		/* endpoint C response register set/clear */
#define REG_USBD_EPD_RSP_SC		(USBD_BA+0xec)		/* endpoint D response register set/clear */
#define REG_USBD_EPE_RSP_SC		(USBD_BA+0x114)	/* endpoint E response register set/clear */
#define REG_USBD_EPF_RSP_SC		(USBD_BA+0x13c)	/* endpoint F response register set/clear */

/* endpoint A/B/C/D/E/F max packet size register */
#define REG_USBD_EPA_MPS		(USBD_BA+0x78)		/* endpoint A max packet size register */
#define REG_USBD_EPB_MPS		(USBD_BA+0xa0)		/* endpoint B max packet size register */
#define REG_USBD_EPC_MPS		(USBD_BA+0xc8)		/* endpoint C max packet size register */
#define REG_USBD_EPD_MPS		(USBD_BA+0xf0)		/* endpoint D max packet size register */
#define REG_USBD_EPE_MPS		(USBD_BA+0x118)	/* endpoint E max packet size register */
#define REG_USBD_EPF_MPS		(USBD_BA+0x140)	/* endpoint F max packet size register */

/* endpoint A/B/C/D/E/F transfer count register */
#define REG_USBD_EPA_TRF_CNT	(USBD_BA+0x7c)		/* endpoint A transfer count register */
#define REG_USBD_EPB_TRF_CNT	(USBD_BA+0xa4)		/* endpoint B transfer count register */
#define REG_USBD_EPC_TRF_CNT	(USBD_BA+0xcc)		/* endpoint C transfer count register */
#define REG_USBD_EPD_TRF_CNT	(USBD_BA+0xf4)		/* endpoint D transfer count register */
#define REG_USBD_EPE_TRF_CNT	(USBD_BA+0x11c)	/* endpoint E transfer count register */
#define REG_USBD_EPF_TRF_CNT	(USBD_BA+0x144)	/* endpoint F transfer count register */

/* endpoint A/B/C/D/E/F configuration register */
#define REG_USBD_EPA_CFG		(USBD_BA+0x80)		/* endpoint A configuration register */
#define REG_USBD_EPB_CFG		(USBD_BA+0xa8)		/* endpoint B configuration register */
#define REG_USBD_EPC_CFG		(USBD_BA+0xd0)		/* endpoint C configuration register */
#define REG_USBD_EPD_CFG		(USBD_BA+0xf8)		/* endpoint D configuration register */
#define REG_USBD_EPE_CFG		(USBD_BA+0x120)	/* endpoint E configuration register */
#define REG_USBD_EPF_CFG		(USBD_BA+0x148)	/* endpoint F configuration register */

/* endpoint A/B/C/D/E/F ram start address register */
#define REG_USBD_EPA_START_ADDR	(USBD_BA+0x84)		/* endpoint A ram start address register */
#define REG_USBD_EPB_START_ADDR	(USBD_BA+0xac)		/* endpoint B ram start address register */
#define REG_USBD_EPC_START_ADDR	(USBD_BA+0xd4)		/* endpoint C ram start address register */
#define REG_USBD_EPD_START_ADDR	(USBD_BA+0xfc)		/* endpoint D ram start address register */
#define REG_USBD_EPE_START_ADDR	(USBD_BA+0x124)	/* endpoint E ram start address register */
#define REG_USBD_EPF_START_ADDR	(USBD_BA+0x14c)	/* endpoint F ram start address register */

#define REG_USBD_EPA_END_ADDR	(USBD_BA+0x88)		/* endpoint A ram end address register */
#define REG_USBD_EPB_END_ADDR	(USBD_BA+0xb0)		/* endpoint B ram end address register */
#define REG_USBD_EPC_END_ADDR	(USBD_BA+0xd8)		/* endpoint C ram end address register */
#define REG_USBD_EPD_END_ADDR	(USBD_BA+0x100)	/* endpoint D ram end address register */
#define REG_USBD_EPE_END_ADDR	(USBD_BA+0x128)	/* endpoint E ram end address register */
#define REG_USBD_EPF_END_ADDR	(USBD_BA+0x150)	/* endpoint F ram end address register */
#define USB_MEM_TEST	(USBD_BA+0x154)	/* endpoint F ram end address register */
#define REG_USBD_HEAD_WORD0		(USBD_BA+0x158)	/* first head data */
#define REG_USBD_HEAD_WORD1		(USBD_BA+0x15C)	/* second head data */
#define REG_USBD_HEAD_WORD2		(USBD_BA+0x160)	/* third head data */
#define REG_USBD_EPA_HEAD_CNT		(USBD_BA+0x164)	/* EPA head count */
#define REG_USBD_EPB_HEAD_CNT		(USBD_BA+0x168)	/* EPB head count */
#define REG_USBD_EPC_HEAD_CNT		(USBD_BA+0x16C)	/* EPC head count */
#define REG_USBD_EPD_HEAD_CNT		(USBD_BA+0x170)	/* EPD head count */
#define REG_USBD_EPE_HEAD_CNT		(USBD_BA+0x174)	/* EPE head count */
#define REG_USBD_EPF_HEAD_CNT		(USBD_BA+0x178)	/* EPF head count */
#define REG_USBD_AHB_DMA_ADDR	(USBD_BA+0x700)		/* AHB_DMA address register */
#define REG_USBD_PHY_CTL		(USBD_BA+0x704)		/* AHB_DMA address register */
	#define bisten		BIT0
	#define bisterr		BIT1
	#define siddq		BIT2
	#define xo_on		BIT3
	#define clk_sel		NVTBIT(5,4)
	#define refclk		BIT6
	#define clk48		BIT7
	#define vbus_detect	BIT8
	#define Phy_suspend	BIT9
	#define Vbus_status	BIT31







//#define I2C_BASE	w55fa92_VA_I2C	/* I2CH Control */
#define REG_I2C_CSR 		(I2C_BA+0x0000)		// R/W: Control and Status  Register
	#define I2C_RXACK 		BIT11			// Received ACK from Slave
    	#define I2C_BUSY 		BIT10			// I2C bus busy
    	#define I2C_AL 			BIT9				// Arbitration lost
    	#define I2C_TIP 			BIT8				// Transfer in progress
    	#define TX_NUM 			NVTBIT(5,4)			// Transmit byte count
    	#define SGMST_EN 			BIT3				// Single master mode enable
    	#define CSR_IF 			BIT2				// Interrupt flag
    	#define CSR_IE 			BIT1				// Interrupt enable
    	#define I2C_EN 			BIT0				// I2C core enabl

#define REG_I2C_DIVIDER (I2C_BA+0x0004)	// R/W:Clock Prescale Register
       #define I2C_DIV 			NVTBIT(15,0)

#define REG_I2C_CMDR 	(I2C_BA+0x0008)			// R/W:Command Register
	#define I2C_START 			BIT4				// Generate start condition
	#define I2C_STOP 			BIT3				// Generate stop condition
//	#define I2C_READ 			BIT2				// Read data from slave
//    	#define I2C_WRITE 			BIT1				// Write data from slave
    	#define I2C_ACK 				BIT0					// Send ACK to slave

#define REG_I2C_SWR 	(I2C_BA+0x000C)			// R/W:Software Mode Register
	#define SER 				BIT5					// Serial interface SDO status
	#define SDR 				BIT4					// Serial interface SDA status
    	#define SCR 				BIT3					// Serial interface SCK status
    	#define SEW 				BIT2					// Serial interface SDO output control
    	#define SDW 				BIT1					// Serial interface SDW status control
    	#define SCW 				BIT0					// Serial interface SCW output control

#define REG_I2C_RxR 		(I2C_BA+0x0010)			// R:Data Receive Register
#define REG_I2C_TxR 		(I2C_BA+0x0014)			// R/W:Data Transmit Register

#define REG_KPICONF		(KPI_BA+0x0000)	// R/W	Keypad controller configuration Register
	#define KROW 			NVTBIT(31,28)		// Keypad Matrix ROW number
	#define KCOL 			NVTBIT(25,24)		// Keypad Matrix COL Number
	#define DB_EN			BIT21			// Scan In Signal De-bounce Enable
	#define DB_CLKSEL 		NVTBIT(19,16)		// Scan In De-bounce sampling cycle selection
	#define PRESCALE		NVTBIT(15,8)		// Row Scan Cycle Pre-scale Value
	#define INPU 			BIT6			// key Scan In Pull-UP Enable Register
	#define WAKEUP			BIT5			// Lower Power Wakeup Enable
	#define ODEN				BIT4		// Open Drain Enable
	#define INTEN			BIT3		// Key Interrupt Enable Control
	#define RKINTEN			BIT2			// Release Key Interrupt Enable Cintrol
	#define PKINTEN			BIT1			// Press Key Interrupt Enable Control
	#define ENKP				BIT0			// Keypad Scan Enable

#define REG_KPI3KCONF	(KPI_BA+0x0004)	// R/W	Keypad controller 3-keys configuration register
	#define EN3KYRST 		BIT24			// Enable Three-key Reset
	#define K32R 			NVTBIT(21,18)		// The #2 Key Row Address
	#define K32C				NVTBIT(17,16)		// The #2 Key Column Address
	#define K31R				NVTBIT(13,10)		// The #1 Key Row Address
	#define K31C 			NVTBIT(9,8)		// The #1 Key Column Address
	#define K30R				NVTBIT(5,2)		// The #0 Key Row Address
	#define K30C				NVTBIT(1,0)		//	The #0 Key Column Address

#define REG_KPISTATUS	(KPI_BA+0x0008)	// R/O	Key Pad Interface Status Register	
	#define PKEY_INT			BIT4			// Press key interrupt
	#define RKEY_INT			BIT3			// Release key interrupt
	#define KEY_INT			BIT2			// Key Interrupt
	#define RST_3KEY		BIT1			// 3-Keys Reset Flag
	#define PDWAKE			BIT0			// Power Down Wakeup Flag

#define REG_KPIRSTC		(KPI_BA+0x000c)	// R/O	Keypad reset period controller register
	#define RSTC				NVTBIT(7,0)			// 3-key Reset Period Count

#define REG_KPIKEST0	(KPI_BA+0x0010)	// R/O	Keypad state register 0
#define REG_KPIKEST1	(KPI_BA+0x0014)	// R/O	Keypad state register 1
#define REG_KPIKPE0		(KPI_BA+0x0018)	// R/O	Lower 32 Press Key event indicator
#define REG_KPIKPE1		(KPI_BA+0x001C)	// R/O	Higher 32 Press Key event indicator
#define REG_KPIKRE0		(KPI_BA+0x0020)	// R/O	Lower 32 Realease Key event indicator
#define REG_KPIKRE1		(KPI_BA+0x0024)	// R/O	Higher 32 Realease Key event indicator
#define REG_KPIPRESCALDIV	(KPI_BA+0x0028)	// Prescale divider
	#define PRESCALDIV 		NVTBIT(7,0)
#define REG_KPILCM		(KPI_BA+0x002C)	// Keypad and LCM Bus Share Setting
	#define LCMMODE			BIT31		// LCM Mode Enable
	#define KPI_8BIT			BIT30		// KPI_8BIT
	#define HSDBROW			NVTBIT(11,8)		// Row scan time
	#define HSDBNUM			NVTBIT(7,0)		// Debounce time
#define REG_KPISUS		(KPI_BA+0x0030)	// Keypad Suspend Mode Setting
	#define SUSFORCE		BIT31		// Force KPI into suspend mode
	#define SUSCNUM			NVTBIT(19,0)		// suspend mode detection


//#define SPI_BASE_0	(w55fa92_VA_SPI0)
#define REG_SPI0_CNTRL		(SPI0_BA + 0x0000)
	#define SIO_DIR			BIT24			//Serial Input / Output Direction Control
	#define Tx_NUM			NVTBIT(23,21)		// Transmit/Receive Numbers.
	#define BYTE_ENDIN		BIT20			//BYTE ENDIN
	#define BYTE_SLEEP		BIT19			//BYTE SLEEP
	#define SLAVE			BIT18			// SLAVE Mode Indication
	#define IE				BIT17			// Interrupt Enable¡P
	#define IFG				BIT16			// Interrupt Flag¡P
	#define SLEEP			NVTBIT(15,12)		// Suspend Interval.
	#define CLKP				BIT11			// Clock Polarity¡P
	#define LSB				BIT10			// Send LSB First¡P
	#define BIT_MODE		NVTBIT(9,8)			// Bit Mode¡P
	#define Tx_BIT_LEN		NVTBIT(7,3)		// Transmit Bit Length.
	#define Tx_NEG			BIT2			// Transmit On Negative Edge¡P
	#define Rx_NEG			BIT1			// Receive On Negative Edge¡P
	#define GO_BUSY			BIT0			// Go and Busy Status¡P
	
#define REG_SPI0_DIVIDER	(SPI0_BA + 0x0004)
	#define SPI0_DIV			NVTBIT(15,0)		//Clock Divider Register
	
#define REG_SPI0_SSR		(SPI0_BA + 0x0008)
	#define LTRIG_FLAG		BIT5			// Level Trigger Flag
	#define SS_LTRIG			BIT4			// Slave Select Level Trigger¡P
	#define ASS				BIT3			// Automatic Slave Select¡P
	#define SS_LVL			BIT2			// Slave Select Active Level.
											// It defines the active level of device/slave select signal (mw_ss_o).
	#define SSR				NVTBIT(1,0)		// Slave Select register
	
#define REG_SPI0_JS			(SPI0_BA + 0x0010)
	#define READY			BIT8			// Slave is ready to transmit / recieve data
	#define JS_INT_FLAG		NVTBIT(7,5)		// Joystick Mode Interrupt Flag
	#define JS_RW			BIT4			// Read / Write Mode
	#define JS				BIT0			// Dongle Joystick mode
	
#define REG_SPI0_TURBO		(SPI0_BA + 0x0014)
	#define TURBO			BIT0			// SPI Turbo mode
	
#define REG_SPI0_EDMA		(SPI0_BA + 0x0018)
	#define EDMA_RW		BIT1			// EDMA Read or EDMA Write
	#define EDMA_GO			BIT0			// EDMA Start

#define REG_SPI0_RX0		(SPI0_BA + 0x0020)
#define REG_SPI0_RX1		(SPI0_BA + 0x0024)
#define REG_SPI0_RX2		(SPI0_BA + 0x0028)
#define REG_SPI0_RX3		(SPI0_BA + 0x002c)
#define REG_SPI0_RX4		(SPI0_BA + 0x0030)
#define REG_SPI0_RX5		(SPI0_BA + 0x0034)
#define REG_SPI0_RX6		(SPI0_BA + 0x0038)
#define REG_SPI0_RX7		(SPI0_BA + 0x003c)

#define REG_SPI0_TX0		(SPI0_BA + 0x0020)
#define REG_SPI0_TX1		(SPI0_BA + 0x0024)
#define REG_SPI0_TX2		(SPI0_BA + 0x0028)
#define REG_SPI0_TX3		(SPI0_BA + 0x002c)
#define REG_SPI0_TX4		(SPI0_BA + 0x0030)
#define REG_SPI0_TX5		(SPI0_BA + 0x0034)
#define REG_SPI0_TX6		(SPI0_BA + 0x0038)
#define REG_SPI0_TX7		(SPI0_BA + 0x003c)

//#define SPI_BASE_1	(w55fa92_VA_SPI1)
#define REG_SPI1_CNTRL		(SPI1_BA + 0x0000)
#define REG_SPI1_DIVIDER	(SPI1_BA + 0x0004)
#define REG_SPI1_SSR		(SPI1_BA + 0x0008)

#define REG_SPI1_JS			(SPI1_BA + 0x0010)
#define REG_SPI1_TURBO		(SPI1_BA + 0x0014)
#define REG_SPI1_EDMA		(SPI1_BA + 0x0018)

#define REG_SPI1_RX0		(SPI1_BA + 0x0020)
#define REG_SPI1_RX1		(SPI1_BA + 0x0024)
#define REG_SPI1_RX2		(SPI1_BA + 0x0028)
#define REG_SPI1_RX3		(SPI1_BA + 0x002c)
#define REG_SPI1_RX4		(SPI1_BA + 0x0030)
#define REG_SPI1_RX5		(SPI1_BA + 0x0034)
#define REG_SPI1_RX6		(SPI1_BA + 0x0038)
#define REG_SPI1_RX7		(SPI1_BA + 0x003c)

#define REG_SPI1_TX0		(SPI1_BA + 0x0020)
#define REG_SPI1_TX1		(SPI1_BA + 0x0024)
#define REG_SPI1_TX2		(SPI1_BA + 0x0028)
#define REG_SPI1_TX3		(SPI1_BA + 0x002c)
#define REG_SPI1_TX4		(SPI1_BA + 0x0030)
#define REG_SPI1_TX5		(SPI1_BA + 0x0034)
#define REG_SPI1_TX6		(SPI1_BA + 0x0038)
#define REG_SPI1_TX7		(SPI1_BA + 0x003c)

//#define PWM_BASE	(w55fa92_VA_PWM)

#define REG_PPR         (PWM_BA+0x000)			// R/W: PWM Prescaler Register
	#define DZI1            NVTBIT(31,24)	 	// Dead zone interval register 1
	#define DZI0            NVTBIT(23,16)			// Dead zone interval register 0
	#define CP1             NVTBIT(15,8)			// Clock prescaler 1 for PWM Timer channel 2 & 3
	#define CP0             NVTBIT(7,0)			// Clock prescaler 0 for PWM Timer channel 0 & 1

#define REG_PWM_CSR     (PWM_BA+0x004)		// R/W: PWM Clock Select Register
	#define CSR3            NVTBIT(14,12)			// Select clock input for channel 3
	#define CSR2            NVTBIT(10,8)			// Select clock input for channel 2
	#define CSR1            NVTBIT(6,4)			// Select clock input for channel 1
	#define CSR0            NVTBIT(2,0)			// Select clock input for channel 0

#define REG_PCR         (PWM_BA+0x008)			// R/W: PWM Control Register
	#define CH3MOD          BIT27				// Channel 3 toggle/one shot mode
	#define CH3INV          BIT26				// Channel 3 inverter on/off
	#define CH3EN           BIT24				// Channel 3 enable/disable
	#define CH2MOD          BIT19				// Channel 2 toggle/one shot mode
	#define CH2INV          BIT18				// Channel 2 inverter on/off
	#define CH2EN           BIT16				// Channel 2 enable/disable
		#define CH1MOD          BIT11				// Channel 1 toggle/one shot mode
		#define CH1INV          BIT10				// Channel 1 inverter on/off
		#define CH1EN           BIT8				// Channel 1 enable/disable
	#define DZEN1           BIT5				// Dead-Zone generator 1 enable/disable
	#define DZEN0           BIT4				// Dead-Zone generator 0 enable/disable
	#define CH0MOD          BIT3				// Channel 0 toggle/one shot mode
	#define CH0INV          BIT2				// Channel 0 inverter on/off
	#define CH0EN           BIT0				// Channel 0 enable/disable

#define REG_CNR0        (PWM_BA+0x00C)			// R/W: PWM Counter Register 0
#define REG_CNR1        (PWM_BA+0x018)			// R/W: PWM Counter Register 1
#define REG_CNR2        (PWM_BA+0x024)			// R/W: PWM Counter Register 2
#define REG_CNR3        (PWM_BA+0x030)			// R/W: PWM Counter Register 3
	#define CNR             NVTBIT(15,0)			    // PWM counter/timer buff

#define REG_CMR0        (PWM_BA+0x010)			// R/W: PWM Comparator Register 0
#define REG_CMR1        (PWM_BA+0x01C)			// R/W: PWM Comparator Register 1
#define REG_CMR2        (PWM_BA+0x028)			// R/W: PWM Comparator Register 2
#define REG_CMR3        (PWM_BA+0x034)			// R/W: PWM Comparator Register 3
	#define CMR             NVTBIT(15,0)			    // PWM comparator register

#define REG_PDR0        (PWM_BA+0x014)			// R  : PWM Data Register 0
#define REG_PDR1        (PWM_BA+0x020)			// R  : PWM Data Register 1
#define REG_PDR2        (PWM_BA+0x02C)			// R  : PWM Data Register 2
#define REG_PDR3        (PWM_BA+0x038)			// R  : PWM Data Register 3
	#define PDR             NVTBIT(15,0)			// PWM data register

#define REG_PIER        (PWM_BA+0x040)			// R/W: PWM Interrupt Enable Register
	#define PIER3           BIT3				// PWM timer channel 3 interrupt enable/disable
	#define PIER2           BIT2				// PWM timer channel 2 interrupt enable/disable
	#define PIER1           BIT1				// PWM timer channel 1 interrupt enable/disable
	#define PIER0           BIT0				// PWM timer channel 0 interrupt enable/disable

#define REG_PIIR        (PWM_BA+0x044)			// R/C: PWM Interrupt Identification Register
	#define PIIR3           BIT3				// PWM timer channel 3 interrupt flag
	#define PIIR2           BIT2				// PWM timer channel 2 interrupt flag
	#define PIIR1           BIT1				// PWM timer channel 1 interrupt flag
	#define PIIR0           BIT0				// PWM timer channel 0 interrupt flag

#define REG_CCR0        (PWM_BA+0x050)			//R/W: Capture Control Register
	#define CFLRD1          BIT23			//CFLR1 dirty bit
	#define CRLRD1          BIT22			//CRLR1 dirty bit
	#define CIIR1           BIT20			//Capture Interrupt Indication 1 Enable/Disable
	#define CAPCH1EN        BIT19			//Capture Channel 1 transition Enable/Disable
	#define FL_IE1          BIT18			//Channel1 Falling Interrupt Enable ON/OFF
	#define RL_IE1          BIT17			//Channel1 Rising Interrupt Enable ON/OFF
	#define INV1            BIT16			//Channel 1 Inverter ON/OFF
	#define CFLRD0          BIT7			//CFLR0 dirty bit
	#define CRLRD0          BIT6			//CRLR0 dirty bit
	#define CIIR0           BIT4			//Capture Interrupt Indication 0 Enable/Disable
	#define CAPCH0EN        BIT3			//Capture Channel 0 transition Enable/Disable
	#define FL_IE0          BIT2			//Channel0 Falling Interrupt Enable ON/OFF
	#define RL_IE0          BIT1			//Channel0 Rising Interrupt Enable ON/OFF
	#define INV0            BIT0			//Channel 0 Inverter ON/OFF


#define REG_CCR1        (PWM_BA+0x054)			//R/W: Capture Control Register
	#define CFLRD3          BIT23			//CFLR3 dirty bit
	#define CRLRD3          BIT22			//CRLR3 dirty bit
	#define CIIR3           BIT20			//Capture Interrupt Indication 3 Enable/Disable
	#define CAPCH3EN        BIT19			//Capture Channel 3 transition Enable/Disable
	#define FL_IE3          BIT18			//Channel3 Falling Interrupt Enable ON/OFF
	#define RL_IE3          BIT17			//Channel3 Rising Interrupt Enable ON/OFF
	#define INV3            BIT16			//Channel 3 Inverter ON/OFF
	#define CFLRD2          BIT7			//CFLR2 dirty bit
	#define CRLRD2          BIT6			//CRLR2 dirty bit
	#define CIIR2           BIT4			//Capture Interrupt Indication 2 Enable/Disable
	#define CAPCH2EN        BIT3			//Capture Channel 2 transition Enable/Disable
	#define FL_IE2          BIT2			//Channel2 Falling Interrupt Enable ON/OFF
	#define RL_IE2          BIT1			//Channel2 Rising Interrupt Enable ON/OFF
	#define INV2            BIT0			//Channel 2 Inverter ON/OFF

#define REG_CRLR0       (PWM_BA+0x058)		//R/W: Capture Rising Latch Register (channel 0)
#define REG_CRLR1       (PWM_BA+0x060)		//R/W: Capture Rising Latch Register (channel 1)
#define REG_CRLR2       (PWM_BA+0x068)		//R/W: Capture Rising Latch Register (channel 2)
#define REG_CRLR3       (PWM_BA+0x070)		//R/W: Capture Rising Latch Register (channel 3)
	#define PWM_CRLR0       NVTBIT(15,0)		//Capture Rising Latch Register

#define REG_CFLR0       (PWM_BA+0x05C)		//R/W: Capture Falling Latch Register (channel 0)
#define REG_CFLR1       (PWM_BA+0x064)		//R/W: Capture Falling Latch Register (channel 1)
#define REG_CFLR2       (PWM_BA+0x06C)		//R/W: Capture Falling Latch Register (channel 2)
#define REG_CFLR3       (PWM_BA+0x074)		//R/W: Capture Falling Latch Register (channel 3)
	#define PWM_CFLR0       NVTBIT(15,0)		//Capture Falling Latch Register

#define REG_CAPENR      (PWM_BA+0x078)		//R/W: Capture Input Enable Register
	#define PWM_CAPENR      NVTBIT(3,0)		//Capture Input Enable

#define REG_POE         (PWM_BA+0x07C)			//R/W: PWM Output Enable Register
	#define PWM3            BIT3			//PWM 3 Output Enable
	#define PWM2            BIT2			//PWM 2 Output Enable
	#define PWM1            BIT1			//PWM 1 Output Enable
	#define PWM0            BIT0			//PWM 0 Output Enable
	#define CH1MOD          BIT11				// Channel 1 toggle/one shot mode
	#define CH1INV          BIT10				// Channel 1 inverter on/off
	#define CH1EN           BIT8				// Channel 1 enable/disable
	#define INV1            BIT16			//Channel 1 Inverter ON/OFF
	#define CFLRD0          BIT7			//CFLR0 dirty bit
	#define CRLRD0          BIT6			//CRLR0 dirty bit
	#define CIIR0           BIT4			//Capture Interrupt Indication 0 Enable/Disable
	#define INV3            BIT16			//Channel 3 Inverter ON/OFF
	#define CFLRD2          BIT7			//CFLR2 dirty bit
	#define CRLRD2          BIT6			//CRLR2 dirty bit


#define REG_RTC_INIR    (RTC_BA+0x000)	// R/W: RTC Initiation Register
	#define INIR_INIR   NVTBIT(31,1)		// RTC Initiation
	#define Active      BIT0				// RTC Active Status.

#define REG_RTC_AER     (RTC_BA+0x004)	// R/W: RTC Initiation Register
	#define ENF         BIT16			// RTC Initiation
	#define AER_AER     NVTBIT(15,0)	// RTC Active Status.

#define REG_RTC_FCR (RTC_BA+0x008)	// R/W: RTC Frequency Compensation Register
	#define FC_EN		BIT31
	#define POWER_KEY_DURATION  NVTBIT(26,24)
	#define INTEGER     NVTBIT(23,8)		// Integer  Part
	#define FRACTION    NVTBIT(5,0)			// Fraction Part

#define REG_RTC_TLR     (RTC_BA+0x00C)	// R/W: Time Loading Register
	#define TLR_10HR    NVTBIT(21,20)		// 10 Hour Time Digit
	#define TLR_1HR     NVTBIT(19,16)		// 1 Hour Time Digit
	#define TLR_10MIN   NVTBIT(14,12)		// 10 Min Time Digit
	#define TLR_1MIN    NVTBIT(11,8)		// 1 Min Time Digit
	#define TLR_10SEC   NVTBIT(6,4)			// 10 Sec Time Digit
	#define TLR_1SEC    NVTBIT(3,0)			// 1 Sec Time Digit

#define REG_RTC_CLR     (RTC_BA+0x010)	// R/W: Calendar Loading Register
	#define CLR_10YEAR  NVTBIT(23,20)		// 10-Year Calendar Digit
	#define CLR_1YEAR   NVTBIT(19,16)		// 1-Year Calendar Digit
	#define CLR_10MON   BIT12				// 10-Month Calendar Digit
	#define CLR_1MON    NVTBIT(11,8)		// 1-Month Calendar Digit
	#define CLR_10DAY   NVTBIT(5,4)			// 10-Day Calendar Digit
	#define CLR_1DAY    NVTBIT(3,0)			// 1-Day Calendar Digit

#define REG_RTC_TSSR    (RTC_BA+0x014)	// R/W  : Time Scale Selection Register
	#define TSSR_24_12  BIT0				//24-Hour / 12-Hour Mode Selection

#define REG_RTC_DWR     (RTC_BA+0x018)	// R/W: Day of the Week Register
	#define RTC_DWR     NVTBIT(2,0)			//Day of the Week Register

#define REG_RTC_TAR     (RTC_BA+0x01C)	// R/W: Time Alarm Register
	#define TAR_Mask_HR_Alarm	BIT23		// Mask alarm by hour
	#define TAR_10HR    NVTBIT(21,20)		// 10 Hour Time Digit
	#define TAR_1HR     NVTBIT(19,16)		// 1 Hour Time Digit
	#define TAR_Mask_Min_Alarm	BIT15		// Mask alarm by minute
	#define TAR_10MIN   NVTBIT(14,12)		// 10 Min Time Digit
	#define TAR_1MIN    NVTBIT(11,8)		// 1 Min Time Digit
	#define TAR_Mask_Sec_Alarm	BIT7		// Mask alarm by second
	#define TAR_10SEC   NVTBIT(6,4)			// 10 Sec Time Digit
	#define TAR_1SEC    NVTBIT(3,0)			// 1 Sec Time Digit

#define REG_RTC_CAR     (RTC_BA+0x020)	// R/W: Calendar Alarm Register
	#define CAR_Mask_WD_Alarm	BIT31			// Mask alarm by week day
	#define CAR_Week_day		NVTBIT(30,28)	// Week day alarm bit	
	#define CAR_Mask_Yr_Alarm	BIT24			// Mask alram by year
	#define CAR_10YEAR  NVTBIT(23,20)			// 10-Year Calendar Digit
	#define CAR_1YEAR   NVTBIT(19,16)			// 1-Year Calendar Digit
	#define CAR_Mon_Alarm		BIT15			// Mask alarm by month
	#define CAR_10MON   BIT12					// 10-Month Calendar Digit
	#define CAR_1MON    NVTBIT(11,8)			// 1-Month Calendar Digit
	#define CAR_Mask_Day_Alarm	BIT7			// Mask alarm by day
	#define CAR_10DAY   NVTBIT(5,4)				// 10-Day Calendar Digit
	#define CAR_1DAY    NVTBIT(3,0)				// 1-Day Calendar Digit

#define REG_RTC_LIR     (RTC_BA+0x024)	// R: RTC Leap year Indication Register
	#define LIR_LIR     BIT0			// Leap Year Indication REGISTER

#define REG_RTC_RIER    (RTC_BA+0x028)	// R/W: RTC Interrupt Enable Register
	#define RAIER        BIT3			// RTC Relative Alarm Interrupt Enable
	#define PSWIER        BIT2			// RTC Power Switch Interrupt Enable
	#define TIER        BIT1			// Time Tick Interrupt Enable
	#define AIER        BIT0			// Alarm Interrupt Enable


#define REG_RTC_RIIR    (RTC_BA+0x02C)	// R/W: RTC Interrupt Enable Register
	#define RAI	        BIT3			// RTC Relative Alarm Interrupt Indication
	#define PSWI        BIT2			// RTC Power Switch Interrupt Indication
	#define TI          BIT1			// RTC Time Tick Interrupt Indication
	#define AI          BIT0			// RTC Alarm Interrupt Indication

#define REG_RTC_TTR     (RTC_BA+0x030)	// R/W: RTC Time Tick Register
	#define TTR_TTR     NVTBIT(2,0)		// Time Tick Register

#define REG_RTC_PWRON 	(RTC_BA+0x034)	// R/W: RTC Power Time On Register
	#define RELATIVE_TIME	NVTBIT(31,20)
	#define PCLR_TIME     	NVTBIT(19,16)
	#define SW_STATUS     	NVTBIT(15,8)
	#define PWR_KEY			BIT7
	#define EDGE_TRIG		BIT5
	#define REL_ALARM_EN	BIT4
	#define ALARM_EN  		BIT3
	#define HW_PCLR_EN		BIT2
	#define SW_PCLR     	BIT1
	#define RTC_PWRON   	BIT0

#define REG_RTC_SET (RTC_BA+0x038)
	#define RTC_WEAK		BIT0
	#define RTC_EN			BIT1
	#define RTC_AEN			BIT2
	#define XIN_XC			BIT3
	#define XOUT_XC			BIT4
		
#define REG_OSC_32K (RTC_BA+0x03C)
	#define OSC_32K_EN	BIT0
	
#define REG_1Hz_CNT (RTC_BA+0x040)

#define REG_FLAG	 (RTC_BA+0x044)
	#define RTC_REG_FLAG BIT0

#define REG_RTC_PORCTRL		(RTC_BA+0x50)
	#define POR_Active			NVTBIT(14,0)
	#define POR_EN_Manual		BIT15
	#define POR_Sample_Cnt		NVTBIT(30,16)
	#define POR_Auto_CTRL_En	BIT31
	
#define REG_RTC_DUMMY0	(RTC_BA+0x54)	

#define REG_RTC_DUMMY1	(RTC_BA+0x58)	

//#define EDMA_BA	(w55fa92_VA_EDMA)

#define REG_VDMA_CSR  (EDMA_BA + 0x0000)		// VDMA Control and Status Register CH0
#define REG_PDMA_CSR1  (EDMA_BA + 0x0100)		// PDMA Control and Status Register CH1
#define REG_PDMA_CSR2  (EDMA_BA + 0x0200)		// PDMA Control and Status Register CH2
#define REG_PDMA_CSR3  (EDMA_BA + 0x0300)		// PDMA Control and Status Register CH3
#define REG_PDMA_CSR4  (EDMA_BA + 0x0400)		// PDMA Control and Status Register CH4
#define REG_VDMA_CSR5  (EDMA_BA + 0x0500)		// VDMA Control and Status Register CH5
#define REG_VDMA_CSR8  (EDMA_BA + 0x0800)		// VDMA Control and Status Register CH8
#define REG_PDMA_CSR9  (EDMA_BA + 0x0900)		// PDMA Control and Status Register CH9
#define REG_PDMA_CSR10  (EDMA_BA + 0x0A00)		// PDMA Control and Status Register CH10
#define REG_PDMA_CSR11  (EDMA_BA + 0x0B00)		// PDMA Control and Status Register CH11
#define REG_PDMA_CSR12  (EDMA_BA + 0x0C00)		// PDMA Control and Status Register CH12
	#define TRIG_EN	    BIT23		// Enalbe EDMA Data Read or Write Transfer
	#define APB_TWS       NVTBIT(20, 19)// Peripheral Transfer Width Select
	#define WAR_BCR_SEL   NVTBIT(15, 12)// Wrap Around Transfer Byte Count Interrupt Select
	#define EDMASG_EN         BIT9		// EDMA Scatter-Gather Function Enable
	#define EDMA_SW_RST     BIT8		// EDMA Rest
	#define DAD_SEL       NVTBIT(7, 6)	// Transfer Destination Address Direction Select
	#define SAD_SEL       NVTBIT(5, 4)	// Transfer Source Address Direction Select
	#define MODE_SEL      NVTBIT(3, 2)	// EDMA Mode Select
	#define SW_RST        BIT1		// Software Engine Reset
	#define EDMACEN       BIT0		// EDMA Channel Enable

#define REG_VDMA_SAR  (EDMA_BA + 0x0004)		// VDMA Transfer Source Address Register CH0
#define REG_PDMA_SAR1  (EDMA_BA + 0x0104)		// PDMA Transfer Source Address Register CH1
#define REG_PDMA_SAR2  (EDMA_BA + 0x0204)		// PDMA Transfer Source Address Register CH2
#define REG_PDMA_SAR3  (EDMA_BA + 0x0304)		// PDMA Transfer Source Address Register CH3
#define REG_PDMA_SAR4  (EDMA_BA + 0x0404)		// PDMA Transfer Source Address Register CH4
#define REG_VDMA_SAR5  (EDMA_BA + 0x0504)		// VDMA Transfer Source Address Register CH5
#define REG_VDMA_SAR8  (EDMA_BA + 0x0804)		// VDMA Transfer Source Address Register CH8
#define REG_PDMA_SAR9  (EDMA_BA + 0x0904)		// PDMA Transfer Source Address Register CH9
#define REG_PDMA_SAR10  (EDMA_BA + 0x0A04)		// PDMA Transfer Source Address Register CH10
#define REG_PDMA_SAR11  (EDMA_BA + 0x0B04)		// PDMA Transfer Source Address Register CH11
#define REG_PDMA_SAR12  (EDMA_BA + 0x0C04)		// PDMA Transfer Source Address Register CH12


#define REG_VDMA_DAR  (EDMA_BA + 0x0008)		// VDMA Transfer Destination Address Register CH0
#define REG_PDMA_DAR1  (EDMA_BA + 0x0108)		// PDMA Transfer Destination Address Register CH1
#define REG_PDMA_DAR2  (EDMA_BA + 0x0208)		// PDMA Transfer Destination Address Register CH2
#define REG_PDMA_DAR3  (EDMA_BA + 0x0308)		// PDMA Transfer Destination Address Register CH3
#define REG_PDMA_DAR4  (EDMA_BA + 0x0408)		// PDMA Transfer Destination Address Register CH4
#define REG_VDMA_DAR5  (EDMA_BA + 0x0508)		// VDMA Transfer Destination Address Register CH5
#define REG_VDMA_DAR8  (EDMA_BA + 0x0808)		// VDMA Transfer Destination Address Register CH8
#define REG_PDMA_DAR9  (EDMA_BA + 0x0908)		// PDMA Transfer Destination Address Register CH9
#define REG_PDMA_DAR10  (EDMA_BA + 0x0A08)		// PDMA Transfer Destination Address Register CH10
#define REG_PDMA_DAR11  (EDMA_BA + 0x0B08)		// PDMA Transfer Destination Address Register CH11
#define REG_PDMA_DAR12  (EDMA_BA + 0x0C08)		// PDMA Transfer Destination Address Register CH12


#define REG_VDMA_BCR  (EDMA_BA + 0x000C)		// VDMA Transfer Byte Count Register CH0
#define REG_PDMA_BCR1  (EDMA_BA + 0x010C)		// PDMA Transfer Byte Count Register CH1
#define REG_PDMA_BCR2  (EDMA_BA + 0x020C)		// PDMA Transfer Byte Count Register CH2
#define REG_PDMA_BCR3  (EDMA_BA + 0x030C)		// PDMA Transfer Byte Count Register CH3
#define REG_PDMA_BCR4  (EDMA_BA + 0x040C)		// PDMA Transfer Byte Count Register CH4
#define REG_VDMA_BCR5  (EDMA_BA + 0x050C)		// VDMA Transfer Byte Count Register CH5
#define REG_VDMA_BCR8  (EDMA_BA + 0x080C)		// VDMA Transfer Byte Count Register CH8
#define REG_PDMA_BCR9  (EDMA_BA + 0x090C)		// PDMA Transfer Byte Count Register CH9
#define REG_PDMA_BCR10  (EDMA_BA + 0x0A0C)		// PDMA Transfer Byte Count Register CH10
#define REG_PDMA_BCR11  (EDMA_BA + 0x0B0C)		// PDMA Transfer Byte Count Register CH11
#define REG_PDMA_BCR12  (EDMA_BA + 0x0C0C)		// PDMA Transfer Byte Count Register CH12
	#define WAR_BCR_IF    NVTBIT(23,  0)// PDMA Transfer Byte Count Reigster

#define REG_VDMA_SGAR  (EDMA_BA + 0x0010)		// VDMA Scatter-Gather Table Start Address Register
#define REG_PDMA_SGAR1  (EDMA_BA + 0x0110)		// PDMA Scatter-Gather Table Start Address Register	CH1
#define REG_PDMA_SGAR2  (EDMA_BA + 0x0210)		// PDMA Scatter-Gather Table Start Address Register	CH2
#define REG_PDMA_SGAR3  (EDMA_BA + 0x0310)		// PDMA Scatter-Gather Table Start Address Register	CH3
#define REG_PDMA_SGAR4  (EDMA_BA + 0x0410)		// PDMA Scatter-Gather Table Start Address Register	CH4
#define REG_VDMA_SGAR5  (EDMA_BA + 0x0510)		// VDMA Scatter-Gather Table Start Address Register	CH5
#define REG_VDMA_SGAR8  (EDMA_BA + 0x0810)		// VDMA Scatter-Gather Table Start Address Register	CH8
#define REG_PDMA_SGAR9  (EDMA_BA + 0x0910)		// PDMA Scatter-Gather Table Start Address Register	CH9
#define REG_PDMA_SGAR10  (EDMA_BA + 0x0A10)		// PDMA Scatter-Gather Table Start Address Register	CH10
#define REG_PDMA_SGAR11  (EDMA_BA + 0x0B10)		// PDMA Scatter-Gather Table Start Address Register	CH11
#define REG_PDMA_SGAR12  (EDMA_BA + 0x0C10)		// PDMA Scatter-Gather Table Start Address Register	CH12


#define REG_VDMA_CSAR  (EDMA_BA + 0x0014)		// VDMA Current Source Address Register CH0
#define REG_PDMA_CSAR1  (EDMA_BA + 0x0114)		// PDMA Current Source Address Register CH1
#define REG_PDMA_CSAR2  (EDMA_BA + 0x0214)		// PDMA Current Source Address Register CH2
#define REG_PDMA_CSAR3  (EDMA_BA + 0x0314)		// PDMA Current Source Address Register CH3
#define REG_PDMA_CSAR4  (EDMA_BA + 0x0414)		// PDMA Current Source Address Register CH4
#define REG_VDMA_CSAR5  (EDMA_BA + 0x0514)		// VDMA Current Source Address Register CH5
#define REG_VDMA_CSAR8  (EDMA_BA + 0x0814)		// VDMA Current Source Address Register CH8
#define REG_PDMA_CSAR9  (EDMA_BA + 0x0914)		// PDMA Current Source Address Register CH9
#define REG_PDMA_CSAR10  (EDMA_BA + 0x0A14)		// PDMA Current Source Address Register CH10
#define REG_PDMA_CSAR11  (EDMA_BA + 0x0B14)		// PDMA Current Source Address Register CH11
#define REG_PDMA_CSAR12  (EDMA_BA + 0x0C14)		// PDMA Current Source Address Register CH12


#define REG_VDMA_CDAR  (EDMA_BA + 0x0018)		// VDMA Current Destination Address Register CH0
#define REG_PDMA_CDAR1  (EDMA_BA + 0x0118)		// PDMA Current Destination Address Register CH1
#define REG_PDMA_CDAR2  (EDMA_BA + 0x0218)		// PDMA Current Destination Address Register CH2
#define REG_PDMA_CDAR3  (EDMA_BA + 0x0318)		// PDMA Current Destination Address Register CH3
#define REG_PDMA_CDAR4  (EDMA_BA + 0x0418)		// PDMA Current Destination Address Register CH4
#define REG_VDMA_CDAR5  (EDMA_BA + 0x0518)		// VDMA Current Destination Address Register CH5
#define REG_VDMA_CDAR8  (EDMA_BA + 0x0818)		// VDMA Current Destination Address Register CH8
#define REG_PDMA_CDAR9  (EDMA_BA + 0x0918)		// PDMA Current Destination Address Register CH9
#define REG_PDMA_CDAR10  (EDMA_BA + 0x0A18)		// PDMA Current Destination Address Register CH10
#define REG_PDMA_CDAR11  (EDMA_BA + 0x0B18)		// PDMA Current Destination Address Register CH11
#define REG_PDMA_CDAR12  (EDMA_BA + 0x0C18)		// PDMA Current Destination Address Register CH12


#define REG_VDMA_CBCR  (EDMA_BA + 0x001C)		// VDMA Current Byte Counte Register CH0
#define REG_PDMA_CBCR1  (EDMA_BA + 0x011C)		// PDMA Current Byte Counte Register CH1
#define REG_PDMA_CBCR2  (EDMA_BA + 0x021C)		// PDMA Current Byte Counte Register CH2
#define REG_PDMA_CBCR3  (EDMA_BA + 0x031C)		// PDMA Current Byte Counte Register CH3
#define REG_PDMA_CBCR4  (EDMA_BA + 0x041C)		// PDMA Current Byte Counte Register CH4
#define REG_VDMA_CBCR5  (EDMA_BA + 0x051C)		// VDMA Current Byte Counte Register CH5
#define REG_VDMA_CBCR8  (EDMA_BA + 0x081C)		// VDMA Current Byte Counte Register CH8
#define REG_PDMA_CBCR9  (EDMA_BA + 0x091C)		// PDMA Current Byte Counte Register CH9
#define REG_PDMA_CBCR10  (EDMA_BA + 0x0A1C)		// PDMA Current Byte Counte Register CH10
#define REG_PDMA_CBCR11  (EDMA_BA + 0x0B1C)		// PDMA Current Byte Counte Register CH11
#define REG_PDMA_CBCR12  (EDMA_BA + 0x0C1C)		// PDMA Current Byte Counte Register CH12


#define REG_VDMA_IER  (EDMA_BA + 0x0020)		// VDMA Interrupt Enable Control Register CH0
#define REG_PDMA_IER1  (EDMA_BA + 0x0120)		// PDMA Interrupt Enable Control Register CH1
#define REG_PDMA_IER2  (EDMA_BA + 0x0220)		// PDMA Interrupt Enable Control Register CH2
#define REG_PDMA_IER3  (EDMA_BA + 0x0320)		// PDMA Interrupt Enable Control Register CH3
#define REG_PDMA_IER4  (EDMA_BA + 0x0420)		// PDMA Interrupt Enable Control Register CH4
#define REG_VDMA_IER5  (EDMA_BA + 0x0520)		// VDMA Interrupt Enable Control Register CH5
#define REG_VDMA_IER8  (EDMA_BA + 0x0820)		// VDMA Interrupt Enable Control Register CH8
#define REG_PDMA_IER9  (EDMA_BA + 0x0920)		// PDMA Interrupt Enable Control Register CH9
#define REG_PDMA_IER10  (EDMA_BA + 0x0A20)		// PDMA Interrupt Enable Control Register CH10
#define REG_PDMA_IER11  (EDMA_BA + 0x0B20)		// PDMA Interrupt Enable Control Register CH11
#define REG_PDMA_IER12  (EDMA_BA + 0x0C20)		// PDMA Interrupt Enable Control Register CH12
	#define SG_IEN	    BIT3		// PDMA Scatter-Gather Interrupt Enable
	#define WAR_IE	    BIT2		// PDMA Wrap Around Interrupt Enable
	#define BLKD_IE	    BIT1		// PDMA Block Transfer Done Interrupt Enable
	#define EDMATABORT_IE	   BIT0	// PDMA Read/Write Target Abort Interrupt Enable

#define REG_VDMA_ISR  (EDMA_BA + 0x0024)		// VDMA Interrupt Status Register CH0
#define REG_PDMA_ISR1  (EDMA_BA + 0x0124)		// PDMA Interrupt Status Register CH1
#define REG_PDMA_ISR2  (EDMA_BA + 0x0224)		// PDMA Interrupt Status Register CH2
#define REG_PDMA_ISR3  (EDMA_BA + 0x0324)		// PDMA Interrupt Status Register CH3
#define REG_PDMA_ISR4  (EDMA_BA + 0x0424)		// PDMA Interrupt Status Register CH4
#define REG_VDMA_ISR5  (EDMA_BA + 0x0524)		// VDMA Interrupt Status Register CH5
#define REG_VDMA_ISR8  (EDMA_BA + 0x0824)		// VDMA Interrupt Status Register CH8
#define REG_PDMA_ISR9  (EDMA_BA + 0x0924)		// PDMA Interrupt Status Register CH9
#define REG_PDMA_ISR10  (EDMA_BA + 0x0A24)		// PDMA Interrupt Status Register CH10
#define REG_PDMA_ISR11  (EDMA_BA + 0x0B24)		// PDMA Interrupt Status Register CH11
#define REG_PDMA_ISR12  (EDMA_BA + 0x0C24)		// PDMA Interrupt Status Register CH12
	#define INTR  	    BIT31		// Interrupt Pin Status
	#define INTR6 	    BIT30		// Interrupt Pin Status of Channel 6
	#define INTR5 	    BIT29		// Interrupt Pin Status of Channel 5
	#define INTR4 	    BIT28		// Interrupt Pin Status of Channel 4
	#define INTR3 	    BIT27		// Interrupt Pin Status of Channel 3
	#define INTR2 	    BIT26		// Interrupt Pin Status of Channel 2
	#define INTR1 	    BIT25		// Interrupt Pin Status of Channel 1
	#define INTR0 	    BIT24		// Interrupt Pin Status of Channel 0
	#define EDMABUSY	    BIT15		// EDMA Transfer is in Progress
	#define EDMAWAR_BCR_IF NVTBIT(11,  8)// Wrap Around Transfer Byte Count Interrupt Flag
	#define EDMASG_IF	      BIT3		// Scatter-Gather Interrupt Flag
	#define EDMABLKD_IF	  BIT1		// Block Transfer Done Interrupt Flag
	#define EDMATABORT_IF	  BIT0		// PDMA Read/Write Target Abort Interrupt Flag


#define REG_VDMA_CTCSR  (EDMA_BA + 0x0028)		// VDMA Color Transfer Control Register CH0
#define REG_VDMA_CTCSR5  (EDMA_BA + 0x0528)		// VDMA Color Transfer Control Register CH5
#define REG_VDMA_CTCSR8  (EDMA_BA + 0x0828)		// VDMA Color Transfer Control Register CH8
	#define SOUR_FORMAT NVTBIT(27, 24)   // Source Address Color Format
	#define DEST_FORMAT NVTBIT(19, 16)   // Destination Address Color Format
	#define CLAMPING_EN	  BIT7		// Clamping Enable
	#define COL_TRA_EN	  BIT1		// Color Transfer Enable
	#define STRIDE_EN 	  BIT0		// Stride Mode Enable

#define REG_VDMA_SASOCR  (EDMA_BA + 0x002C)	// VDMA Source Address Stride Offset Control Register CH0
#define REG_VDMA_SASOCR5  (EDMA_BA + 0x052C)	// VDMA Source Address Stride Offset Control Register CH5
#define REG_VDMA_SASOCR8  (EDMA_BA + 0x082C)	// VDMA Source Address Stride Offset Control Register CH8
	#define STBC          NVTBIT(31, 16)// PDMA Stride Transfer Byte Count
	#define SASTOBL       NVTBIT(15,  0)// PDMA Source Address Stride Offset Byte Length

#define REG_VDMA_DASOCR  (EDMA_BA + 0x0030)	// VDMA Destination Address Stride Offset Control Register CH0
#define REG_VDMA_DASOCR5  (EDMA_BA + 0x0530)	// VDMA Destination Address Stride Offset Control Register CH5
#define REG_VDMA_DASOCR8  (EDMA_BA + 0x0830)	// VDMA Destination Address Stride Offset Control Register CH8
	#define DASTOBL       NVTBIT(15,  0)// VDMA Destination Address Stride Offset Byte Length

#define REG_PDMA_POINT1  (EDMA_BA + 0x013C)	// PDMA Internal Buffer Pointer Register CH1
#define REG_PDMA_POINT2  (EDMA_BA + 0x023C)	// PDMA Internal Buffer Pointer Register CH2
#define REG_PDMA_POINT3  (EDMA_BA + 0x033C)	// PDMA Internal Buffer Pointer Register CH3
#define REG_PDMA_POINT4  (EDMA_BA + 0x043C)	// PDMA Internal Buffer Pointer Register CH4
#define REG_PDMA_POINT9  (EDMA_BA + 0x093C)	// PDMA Internal Buffer Pointer Register CH9
#define REG_PDMA_POINT10  (EDMA_BA + 0x0A3C)	// PDMA Internal Buffer Pointer Register CH10
#define REG_PDMA_POINT11  (EDMA_BA + 0x0B3C)	// PDMA Internal Buffer Pointer Register CH11
#define REG_PDMA_POINT12  (EDMA_BA + 0x0C3C)	// PDMA Internal Buffer Pointer Register CH12
	#define PDMA_POINT    NVTBIT(4,  0) // PDMA Internal Buffer Pointer Reigster

#define REG_EDMA_SBUF0_C0  (EDMA_BA + 0x0080)	// VDMA Shared Buffer FIFO 0 Reigster CH0
#define REG_EDMA_SBUF0_C1  (EDMA_BA + 0x0180)	// PDMA Shared Buffer FIFO 0 Reigster CH1
#define REG_EDMA_SBUF0_C2  (EDMA_BA + 0x0280)	// PDMA Shared Buffer FIFO 0 Reigster CH2
#define REG_EDMA_SBUF0_C3  (EDMA_BA + 0x0380)	// PDMA Shared Buffer FIFO 0 Reigster CH3
#define REG_EDMA_SBUF0_C4  (EDMA_BA + 0x0480)	// PDMA Shared Buffer FIFO 0 Reigster CH4
#define REG_EDMA_SBUF0_C5  (EDMA_BA + 0x0580)	// VDMA Shared Buffer FIFO 0 Reigster CH5

#define REG_EDMA_SBUF1_C0  (EDMA_BA + 0x0084)	// VDMA Shared Buffer FIFO 1 Reigster CH0
#define REG_EDMA_SBUF1_C1  (EDMA_BA + 0x0184)	// PDMA Shared Buffer FIFO 1 Reigster CH1
#define REG_EDMA_SBUF1_C2  (EDMA_BA + 0x0284)	// PDMA Shared Buffer FIFO 1 Reigster CH2
#define REG_EDMA_SBUF1_C3  (EDMA_BA + 0x0384)	// PDMA Shared Buffer FIFO 1 Reigster CH3
#define REG_EDMA_SBUF1_C4  (EDMA_BA + 0x0484)	// PDMA Shared Buffer FIFO 1 Reigster CH4
#define REG_EDMA_SBUF1_C5  (EDMA_BA + 0x0584)	// VDMA Shared Buffer FIFO 1 Reigster CH5

#define REG_EDMA_SBUF2_C0  (EDMA_BA + 0x0088)	// VDMA Shared Buffer FIFO 2 Reigster CH0
#define REG_EDMA_SBUF2_C1  (EDMA_BA + 0x0188)	// PDMA Shared Buffer FIFO 2 Reigster CH1
#define REG_EDMA_SBUF2_C2  (EDMA_BA + 0x0288)	// PDMA Shared Buffer FIFO 2 Reigster CH2
#define REG_EDMA_SBUF2_C3  (EDMA_BA + 0x0388)	// PDMA Shared Buffer FIFO 2 Reigster CH3
#define REG_EDMA_SBUF2_C4  (EDMA_BA + 0x0488)	// PDMA Shared Buffer FIFO 2 Reigster CH4
#define REG_EDMA_SBUF2_C5  (EDMA_BA + 0x0588)	// VDMA Shared Buffer FIFO 2 Reigster CH5

#define REG_EDMA_SBUF3_C0  (EDMA_BA + 0x008C)	// VDMA Shared Buffer FIFO 3 Reigster CH0
#define REG_EDMA_SBUF3_C1  (EDMA_BA + 0x018C)	// PDMA Shared Buffer FIFO 3 Reigster CH1
#define REG_EDMA_SBUF3_C2  (EDMA_BA + 0x028C)	// PDMA Shared Buffer FIFO 3 Reigster CH2
#define REG_EDMA_SBUF3_C3  (EDMA_BA + 0x038C)	// PDMA Shared Buffer FIFO 3 Reigster CH3
#define REG_EDMA_SBUF3_C4  (EDMA_BA + 0x048C)	// PDMA Shared Buffer FIFO 3 Reigster CH4
#define REG_EDMA_SBUF3_C5  (EDMA_BA + 0x058C)	// VDMA Shared Buffer FIFO 3 Reigster CH5

#define REG_EDMA_SBUF4_C0  (EDMA_BA + 0x0090)	// VDMA Shared Buffer FIFO 4 Reigster CH0
#define REG_EDMA_SBUF4_C2  (EDMA_BA + 0x0290)	// PDMA Shared Buffer FIFO 4 Reigster CH2
#define REG_EDMA_SBUF4_C3  (EDMA_BA + 0x0390)	// PDMA Shared Buffer FIFO 4 Reigster CH3
#define REG_EDMA_SBUF4_C5  (EDMA_BA + 0x0590)	// VDMA Shared Buffer FIFO 4 Reigster CH5

#define REG_EDMA_SBUF5_C0  (EDMA_BA + 0x0094)	// VDMA Shared Buffer FIFO 5 Reigster CH0
#define REG_EDMA_SBUF5_C2  (EDMA_BA + 0x0294)	// PDMA Shared Buffer FIFO 5 Reigster CH2
#define REG_EDMA_SBUF5_C3  (EDMA_BA + 0x0394)	// PDMA Shared Buffer FIFO 5 Reigster CH3
#define REG_EDMA_SBUF5_C5  (EDMA_BA + 0x0594)	// VDMA Shared Buffer FIFO 5 Reigster CH5

#define REG_EDMA_SBUF6_C0  (EDMA_BA + 0x0098)	// VDMA Shared Buffer FIFO 6 Reigster CH0
#define REG_EDMA_SBUF6_C2  (EDMA_BA + 0x0298)	// PDMA Shared Buffer FIFO 6 Reigster CH2
#define REG_EDMA_SBUF6_C3  (EDMA_BA + 0x0398)	// PDMA Shared Buffer FIFO 6 Reigster CH3
#define REG_EDMA_SBUF6_C5  (EDMA_BA + 0x0598)	// VDMA Shared Buffer FIFO 6 Reigster CH5

#define REG_EDMA_SBUF7_C0  (EDMA_BA + 0x009C)	// VDMA Shared Buffer FIFO 7 Reigster CH0
#define REG_EDMA_SBUF7_C2  (EDMA_BA + 0x029C)	// PDMA Shared Buffer FIFO 7 Reigster CH2
#define REG_EDMA_SBUF7_C3  (EDMA_BA + 0x039C)	// PDMA Shared Buffer FIFO 7 Reigster CH3
#define REG_EDMA_SBUF7_C5  (EDMA_BA + 0x059C)	// VDMA Shared Buffer FIFO 7 Reigster CH5

#define REG_EDMA_SBUF8_C0  (EDMA_BA + 0x00A0)	// VDMA Shared Buffer FIFO 8 Reigster CH0
#define REG_EDMA_SBUF8_C5  (EDMA_BA + 0x05A0)	// VDMA Shared Buffer FIFO 8 Reigster CH5

#define REG_EDMA_SBUF9_C0  (EDMA_BA + 0x00A4)	// VDMA Shared Buffer FIFO 9 Reigster CH0
#define REG_EDMA_SBUF9_C5  (EDMA_BA + 0x05A4)	// VDMA Shared Buffer FIFO 9 Reigster CH5

#define REG_EDMA_SBUF10_C0  (EDMA_BA + 0x00A8)	// VDMA Shared Buffer FIFO 10 Reigster CH0
#define REG_EDMA_SBUF10_C5  (EDMA_BA + 0x05A8)	// VDMA Shared Buffer FIFO 10 Reigster CH5

#define REG_EDMA_SBUF11_C0  (EDMA_BA + 0x00AC)	// VDMA Shared Buffer FIFO 11 Reigster CH0
#define REG_EDMA_SBUF11_C5  (EDMA_BA + 0x05AC)	// VDMA Shared Buffer FIFO 11 Reigster CH5

#define REG_EDMA_SBUF12_C0  (EDMA_BA + 0x00B0)	// VDMA Shared Buffer FIFO 12 Reigster CH0
#define REG_EDMA_SBUF12_C5  (EDMA_BA + 0x05B0)	// VDMA Shared Buffer FIFO 12 Reigster CH5

#define REG_EDMA_SBUF13_C0  (EDMA_BA + 0x00B4)	// VDMA Shared Buffer FIFO 13 Reigster CH0
#define REG_EDMA_SBUF13_C5  (EDMA_BA + 0x05B4)	// VDMA Shared Buffer FIFO 13 Reigster CH5

#define REG_EDMA_SBUF14_C0  (EDMA_BA + 0x00B8)	// VDMA Shared Buffer FIFO 14 Reigster CH0
#define REG_EDMA_SBUF14_C5  (EDMA_BA + 0x05B8)	// VDMA Shared Buffer FIFO 14 Reigster CH5

#define REG_EDMA_SBUF15_C0  (EDMA_BA + 0x00BC)	// VDMA Shared Buffer FIFO 15 Reigster CH0
#define REG_EDMA_SBUF15_C5  (EDMA_BA + 0x05BC)	// VDMA Shared Buffer FIFO 15 Reigster CH5

//#define GE_BA	(w55fa92_GE)
#define REG_2D_GETRIG		(GE_BA+0x0000)	//R/W   GE Trigger Control
        #define CMD_QUEUE   BIT1                    // 2D Command Queue Enable
        #define GE_GO       BIT0                    // 2D Engine Trigger

#define REG_2D_GEXYSOA      (GE_BA+0x0004)  //R/W   GE Source Origin Address

#define REG_2D_TILEXY       (GE_BA+0x0008)  //R/W   GE TileBLT Width / Height
        #define TILE_Y      NVTBIT(15, 8)           // Tile Height
        #define TILE_X      NVTBIT(7, 0)            // Tile Width

#define REG_2D_GERRXY       (GE_BA+0x000C)  //R/W   GE Rotate Refernce Point
#define REG_2D_GEXYD2A      (GE_BA+0x000C)  //R/W   GE Secondary Destination Origin Address
#define REG_2D_SLLACQ       (GE_BA+0x000C)  //R/W   GE The Start Link List Address of Command Queue

#define REG_2D_GEINTS       (GE_BA+0x0010)  //R/W   GE Interrupt Status

#define REG_2D_GEPLS        (GE_BA+0x0014)  //R/W   GE Pattern Location Starting Address

#define REG_2D_GEBER        (GE_BA+0x0018)  //R/W   GE Bresenham Error Term Stepping Constant
#define REG_2D_VSF          (GE_BA+0x0018)  //R/W   GE Vertical Scaling Control 
#define REG_2D_GEBIR        (GE_BA+0x001C)  //R/W   GE Bresenham Initial Error, Pixel Count Major M
#define REG_2D_HSF          (GE_BA+0x001C)  //R/W   GE Horizontal Scaling Control 

#define REG_2D_GECMD        (GE_BA+0x0020)  //R/W   GE Command Control

#define REG_2D_GEBC         (GE_BA+0x0024)  //R/W   GE Background Color
#define REG_2D_GEFC         (GE_BA+0x0028)  //R/W   GE Foreground Color
#define REG_2D_GETC         (GE_BA+0x002C)  //R/W   GE Transparency Color
#define REG_2D_GETCM        (GE_BA+0x0030)  //R/W   GE Transparency Color Mask

#define REG_2D_GEXYDOA      (GE_BA+0x0034)  //R/W   GE Destination/Display Origin Address
#define REG_2D_GESDP        (GE_BA+0x0038)  //R/W   GE Source/Destination Pitch
#define REG_2D_GESSXYL      (GE_BA+0x003C)  //R/W   GE Source Start XY/Linear Address
#define REG_2D_GEDSXYL      (GE_BA+0x0040)  //R/W   GE Destination Start XY/Linear Address
#define REG_2D_GEDIXYL      (GE_BA+0x0044)  //R/W   GE Dimension XY/Linear
#define REG_2D_GECBTL       (GE_BA+0x0048)  //R/W   GE Clipping Window Top/Left Boundary
#define REG_2D_GECBBR       (GE_BA+0x004C)  //R/W   GE Clipping Window Bottom/Right Boundary

#define REG_2D_GEPTNA       (GE_BA+0x0050)  //R/W   GE Pattern A
#define REG_2D_GEPTNB       (GE_BA+0x0054)  //R/W   GE Pattern B

#define REG_2D_GEWPM        (GE_BA+0x0058)  //R/W   GE Write Plane Mask

#define REG_2D_GEMC         (GE_BA+0x005C)  //R/W   GE MISC Control

#define REG_2D_GEHBDW0      (GE_BA+0x0060)  //R/W   GE HostBLT Data Port 0~7
#define REG_2D_GEHBDW1      (GE_BA+0x0064)
#define REG_2D_GEHBDW2      (GE_BA+0x0068)
#define REG_2D_GEHBDW3      (GE_BA+0x006C)
#define REG_2D_GEHBDW4      (GE_BA+0x0070)
#define REG_2D_GEHBDW5      (GE_BA+0x0074)
#define REG_2D_GEHBDW6      (GE_BA+0x0078)
#define REG_2D_GEHBDW7      (GE_BA+0x007C)


//#define VPE_BA	(w55fa92_VPE)
#define REG_VPE_TG		(VPE_BA + 0x00)		//R/W	Video Process Engine (VPE) Trigger Control Register
		#define VPE_GO		BIT0					//Trigger Video Process Engine Operation

#define REG_VPE_PLYA_PK	(VPE_BA + 0x04)		//R/W	VPE Source Planar Y or Packet YUV Start Address
#define REG_VPE_PLUA	(VPE_BA + 0x08)		//R/W	VPE Source Planar U Start Address
#define REG_VPE_PLVA	(VPE_BA + 0x0C)		//R/W	VPE Source Planar V Start Address

#define REG_VPE_INTS	(VPE_BA + 0x10)		//R/W	VPE Interrupt Status Register
		#define TA_INTS		BIT5					//VPE DMA Target Abort or Data Abort Interrupt Status 
		#define DE_INTS		BIT4					//Decoder Block Sequence Error Interrupt Status 
		#define MB_INTS		BIT3					//Decoder MCU/Macro Block Detection Interrupt Status 
		#define PG_MISS		BIT2					//VPE MMU Page Miss Interrupt Status 
		#define PF_INTS		BIT1					//VPE MMU Page Fault Interrupt Status 
		#define VP_INTS		BIT0					//VPE Completion Interrupt Status  

#define REG_VPE_SLORO	(VPE_BA + 0x14)		//R/W	Source Packet / Planar Y Left Line Offset and Right Line Offset (pixel unit)
		#define SRCLLO		NVTBIT(28, 16)			//13-bit Source Packet/Planar Y Left Line Offset
		#define SRCRLO		NVTBIT(12, 0)			//13-bit Source Packet/Planar Y right Line Offset

#define REG_VPE_VYDSF	(VPE_BA + 0x18)		//R/W	Vertical Divider for DDA Scaling Up/Down
		#define VSF_N		NVTBIT(28, 16)			//13-bit Vertical N Scaling Factor
		#define VSF_M		NVTBIT(12, 0)			//13-bit Vertical M Scaling Factor
#define REG_VPE_HXDSF	(VPE_BA + 0x1C)		//R/W	Horizontal Divider for DDA Scaling Up/Down
		#define HSF_N		NVTBIT(28, 16)			//13-bit Horizontal N Scaling Factor
		#define HSF_M		NVTBIT(12, 0)			//13-bit Horizontal M Scaling Factor

#define REG_VPE_CMD		(VPE_BA + 0x20)		//R/W	VPE Command Control Register
		#define CCIR601		BIT31					//Source YUV Level Range
		#define SORC		NVTBIT(30, 28)			//Source YUV/RGB Formats (Read Only)
		#define LEVEL		BIT27					//DEST packet YUV level adjustment to the CCIR601 range
		#define TRACE		BIT26					//Trace the Internal Hardware States (For debugging purpose)
		#define DEST		NVTBIT(25, 24)			//Destination YUV/RGB Formats
		#define BLOCK_SEQ	NVTBIT(23, 20)			//Block type if on the fly
		#define OPCMD		NVTBIT(19, 16)			//Operate Command of Rotate Direction or 3X3 Average Filter
		#define MTLB_SET	NVTBIT(15, 14)			//Micro TLB dump to VPE MMU CR's 80~FF. (For debugging purpose)
		#define Y_ROUND		BIT13					//Y-axis Round Control. (For debugging purpose)
		#define X_ROUND		BIT12					//X-axis Round Control. (For debugging purpose)
		#define SINGLE		BIT11					//VPE Write Buffer Mode
		#define CROP		BIT10					//VPE Cropping function for on the fly.
		#define BYPASS		BIT9					//3X3 Filter On=0/Off=1, 1=bypass
		#define TAP			BIT8					//3X3 Average Filter Tap Coefficients
		#define BILINEAR	BIT7					//VPE Bilinear Up/Down-Scaling Filter Function On/Off
		#define BUSRT		BIT6					//AHB Write Burst Length for Bilinear Filter to Fetch More Pixels
		#define HOST_SEL	NVTBIT(5, 4)			//Host Select (C&M Video Decoder, JPEG Decoder, or 2D)
		#define MB_EN		BIT3					//Macro Block Detection Interrupt Enable for video decoder
		#define PMS_EN		BIT2					//Enable VPE Scatter Gather DMA Operation
		#define PFT_EN		BIT1					//Interrupt enable for VPE Scatter Gather DMA operations.
		#define VPE_INT_EN	BIT0					//Interrupt enable for VPE operations.

#define REG_VPE_DEST_PK	(VPE_BA + 0x24)		//R/W	Data Format Conversion Packet Destination Start Address

#define REG_VPE_DLORO	(VPE_BA + 0x28)		//R/W 	Destination Packet Left Line Offset and Right Line Offset (pixel unit)
		#define DSTLLO		NVTBIT(28, 16)			//13-bit Source Packet Left Line Offset
		#define DSTRLO		NVTBIT(12, 0)			//13-bit Source Packet right Line Offset

#define REG_VPE_RESET	(VPE_BA + 0x34)		//R/W	VPE Reset Control Register
		#define RST_FIFO	BIT1					//RESET VPE FIFO Control
		#define RESET_VPE	BIT0					//RESET VPE Operation

#define REG_VMMU_CR		(VPE_BA + 0x80)		//R/W	 VPE MMU Control Register
		#define MAIN_TLB	BIT4					//VPE MMU Main TLB Service Channels
		#define MAIN_EN		BIT1					//Turn on/off main TLB SRAM Buffer
		#define MMU_EN		BIT0					//Enable or Disable VPE MMU Virtual Address Translation

#define REG_VMMU_TTB	(VPE_BA + 0x84)		//R/W	 VPE MMU Translation Table Base Register
		#define TTB_ADDR	NVTBIT(31, 0)			//32-bit Translation Table Base Address
#define REG_VMMU_PFTVA	(VPE_BA + 0x88)		//R/W	 VPE MMU Page Fault Virtual Address Register
		#define PFTVA		NVTBIT(31, 0)			//32-bit Page Fault Virtual Address
#define REG_VMMU_CMD	(VPE_BA + 0x8C)		//R/W	 VPE MMU Page Fault Interrupt Status Register
		#define MTLB_FAIL	BIT17					//Flush VPE MMU Main TLB Entries Pass/Fail Status (Read Only)
		#define MTLB_FINISH	BIT16					//Flush VPE MMU Main TLB Entries Finish/Busy Status (Read Only)
		#define VPE_FLUSH	BIT2					//Flush VPE MMU Main TLB Entries of Main TLB SRAM
		#define INVALID		BIT1					//Invalidate VPE MMU Micro TLB Entries
		#define RESUME		BIT0					//Resume VPE MMU Transaction

#define REG_VMMU_L1PT0	(VPE_BA + 0x90)		//R/W	 VPE MMU Level-One Page Table Entry 0 descriptor
		#define L1PT0			NVTBIT(31, 0)		//32-bit Descriptor Information for Level-One Page Table Entry 0
#define REG_VMMU_L1PT1	(VPE_BA + 0x94)		//R/W	 VPE MMU Level-One Page Table Entry 1 descriptor
		#define L1PT1			NVTBIT(31, 0)		//32-bit Descriptor Information for Level-One Page Table Entry 1
#define REG_VMMU_L1PT2	(VPE_BA + 0x98)		//R/W	 VPE MMU Level-One Page Table Entry 2 descriptor
		#define L1PT2			NVTBIT(31, 0)		//32-bit Descriptor Information for Level-One Page Table Entry 2
#define REG_VMMU_L1PT3	(VPE_BA + 0x9C)		//R/W	 VPE MMU Level-One Page Table Entry 3 descriptor
		#define L1PT3			NVTBIT(31, 0)		//32-bit Descriptor Information for Level-One Page Table Entry 3	
#define REG_VMMU_L1PT4	(VPE_BA + 0xA0)		//R/W	 VPE MMU Level-One Page Table Entry 0 descriptor
		#define L1PT4			NVTBIT(31, 0)		//32-bit Descriptor Information for Level-One Page Table Entry 0
#define REG_VMMU_L1PT5	(VPE_BA + 0xA4)		//R/W	 VPE MMU Level-One Page Table Entry 1 descriptor
		#define L1PT5			NVTBIT(31, 0)		//32-bit Descriptor Information for Level-One Page Table Entry 1
#define REG_VMMU_L1PT6	(VPE_BA + 0xA8)		//R/W	 VPE MMU Level-One Page Table Entry 2 descriptor
		#define L1PT6			NVTBIT(31, 0)		//32-bit Descriptor Information for Level-One Page Table Entry 2
#define REG_VMMU_L1PT7	(VPE_BA + 0xAC)		//R/W	 VPE MMU Level-One Page Table Entry 3 descriptor
		#define L1PT7			NVTBIT(31, 0)		//32-bit Descriptor Information for Level-One Page Table Entry 3	
		
						
#define REG_VMMU_CVA	(VPE_BA + 0xB0)		//R	 	VPE MMU Current Virtual Address Register
		#define CVA				NVTBIT(31, 0)		//32-bit Current Virtual Address		
#define REG_VMMU_CVPN	(VPE_BA + 0xB4)		//R	 	VPE MMU Current Virtual Page Number Register
		#define CVPN			NVTBIT(31, 0)		//Current Virtual Page Number 			
		
#define REG_VMMU_CPA	(VPE_BA + 0xB8)		//R	 	VPE MMU Current Physical Address Register
		#define CPA				NVTBIT(31, 0)			//Current Physical Address
#define REG_VMMU_CPPN	(VPE_BA + 0xBC)		//R	 	VPE MMU Current Physical Page Number Register
		#define CPPN			NVTBIT(31, 0)		//Current Physical Page Number 			

// Define registers for BLT
#define REG_SET2DA    	(BLT_BA+0x0000)	// BitBlitting Accelerator Enable Set Up Register
	#define	FILL_OP		BIT11					// Fill operation to rectangle	
	#define FILL_STYLE	NVTBIT(10,8)			// Bitmap Fill Style
	#define TRCOLOR_E	BIT7					// RGB565 Transparent Color Enable
	#define TRANS_FLAG	NVTBIT(6,4)				// Transform Flag
	#define S_ALPHA		BIT3					// Reveal Source Image Alpha during Transparency
	#define FILL_BLEND	BIT2					// Alpha blending for Fill operation	
	#define L_ENDIAN	BIT1					// Palette Format Data Order
	#define BLIT_EN		BIT0					// Blit a bitmap to the frame buffer using the hardware accelerator
#define	REG_SFMT		(BLT_BA+0x0004)	// Pixel Format of Source Bitmap Register
	#define S_RGB_bpp8	BIT5					// Bitmap Pixel Format bpp8_clutRGB
	#define S_RGB_bpp4	BIT4					// Bitmap Pixel Format bpp4_clutRGB
	#define S_RGB_bpp2	BIT3					// Bitmap Pixel Format bpp2_clutRGB
	#define S_RGB_bpp1	BIT2					// Bitmap Pixel Format bpp1_clutRGB
	#define S_RGB565	BIT1					// Bitmap Pixel Format bpp16_RGB565
	#define S_ARGB8888	BIT0					// Bitmap Pixel Format bpp32_ARGB8888	
#define REG_DFMT		(BLT_BA+0x0008)	// Pixel Format of Destination Bitmap Register
	#define D_RGB555	BIT2					// Display format RGB555 16 bit format
	#define D_RGB565	BIT1					// Display format RGB565 16 bit format
	#define D_ARGB8888	BIT0					// Display format ARGB8888 32 bit format		
#define REG_BLTINTCR	(BLT_BA+0x000C)	// BLT Interrupt Control and Status Register
	
	#define BLT_PMS_INTS	BIT5					// BLT MMU Page Miss Interrupt Status
	#define BLT_PFT_INTS	BIT4					// BLT MMU Page Fault Interrupt Status
	#define BLT_TABORT		BIT3					// Target/Data Abort Error Status
	
	#define BLT_ERR			BIT2					// Blitting Error Status
	#define BLT_INTE		BIT1					// Blitting Complete Interrupt Enable
	#define BLT_INTS		BIT0					// Blitting Complete Interrupt Status
#define REG_MLTA		(BLT_BA+0x0010)	// Alpha Multiplier Register
	#define OFFSET_A	NVTBIT(31,16)			// Fixed point 8.8 A Color Offset Value
	#define MULTIPLIER_A	NVTBIT(15,0)		// Fixed point 8.8 A Color Multiplier Value
#define REG_MLTR		(BLT_BA+0x0014)	// Red Multiplier Register
	#define OFFSET_R	NVTBIT(31,16)			// Fixed point 8.8 R Color Offset Value
	#define MULTIPLIER_R	NVTBIT(15, 0)		// Fixed point 8.8 R Color Multiplier Value
#define REG_MLTG		(BLT_BA+0x0018)	// Green Multiplier Register
	#define OFFSET_G	NVTBIT(31,16)			// Fixed point 8.8 G Color Offset Value
	#define MULTIPLIER_G	NVTBIT(15,0)		// Fixed point 8.8 G Color Multiplier Value
#define REG_MLTB      	(BLT_BA+0x001C)	// Blue Multiplier Register
	#define OFFSET_B	NVTBIT(31,16)			// Fixed point 8.8 B Color Offset Value
	#define MULTIPLIER_B	NVTBIT(15, 0)		// Fixed point 8.8 B Color Multiplier Value
#define REG_SWIDTH		(BLT_BA+0x0020)	// Width of Source Register
	#define WIDTH_S		NVTBIT(15,0)			// The width of source Register
#define REG_SHEIGHT		(BLT_BA+0x0024)	// Height of Source Register
	#define HEIGHT_S	NVTBIT(15,0)			// The Height of source Register
#define REG_DWIDTH		(BLT_BA+0x0028)	// Width of Destination Register
	#define WIDTH_D		NVTBIT(15,0)			// The width of Destination Register
#define REG_DHEIGHT		(BLT_BA+0x002C)	// Height of Destination Register
	#define HEIGHT_D	NVTBIT(15,0)			// The Height of Destination Register
#define REG_ELEMENTA	(BLT_BA+0x0030)	// Transform Element A Register
#define REG_ELEMENTB	(BLT_BA+0x0034)	// Transform Element B Register
#define REG_ELEMENTC	(BLT_BA+0x0038)	// Transform Element C Register
#define REG_ELEMENTD	(BLT_BA+0x003C)	// Transform Element D Register
#define REG_SADDR		(BLT_BA+0x0040)	// Source Remap Start Address Register
#define REG_DADDR		(BLT_BA+0x0044)	// Frame Buffer Address Register
#define REG_SSTRIDE		(BLT_BA+0x0048)	// Source Stride Register
	#define STRIDE_S	NVTBIT(15,0)			// The Source Stride
#define REG_DSTRIDE		(BLT_BA+0x004C)	// Destination Stride Register
	#define STRIDE_D	NVTBIT(15,0)			// The Destination Stride
#define REG_OFFSETSX	(BLT_BA+0x0050)	// Offset of Source X Register
#define REG_OFFSETSY	(BLT_BA+0x0054)	// Offset of Source Y Register
#define REG_TRCOLOR		(BLT_BA+0x0058)	// RGB565 Transparent Color Register
	#define TR_RGB565	NVTBIT(15,0)			// RGB565 Transparent Color
#define REG_FILLARGB	(BLT_BA+0x0060)	// ARGB Color Values for Fill Operation Register
#define REG_PALETTE		(BLT_BA+0x0400)	// Color Palette Register

// Define registers for BLT MMU
#define	REG_BMMU_CR		(BLT_BA+0x080)		// BLT MMU Control Register
	#define BLT_MMU_RST	BIT16				// 
	#define BLT_PMS_EN	BIT9				// BLT MMU Page Miss Interrupt Enable
	#define BLT_PFT_EN	BIT8				// BLT MMU Page Fault Interrupt Enable
	#define MAIN_TLB	BIT4				// BLT MMU Main TLB Service Channels
	#define MAIN_EN		BIT1				// Turn On/Off BLT MMU Main TLB (SRAM Buffers)
	#define MMU_EN		BIT0				// Enable or Disable BLT MMU Virtual Address Translation
#define	REG_BMMU_TTB	(BLT_BA+0x084)		// BLT MMU Translation Table Base Register
#define	REG_BMMU_PFTVA	(BLT_BA+0x088)		// BLT MMU Page Fault Virtual Address Reg.
#define	REG_BMMU_CMD	(BLT_BA+0x08C)		// BLT MMU Resume and Invalidate Command
	#define MTLB_FAIL	BIT17					// Flush BLT MMU Main TLB Entries Pass/Fail Status
	#define MTLB_FINISH	BIT16					// Flush BLT MMU Main TLB Entries Finish/Busy Status
	#define FLUSH_MAIN_TLB		BIT2			// Flush BLT MMU Main TLB Entries of Main TLB SRAM
	#define	INVALID_MICRO_TLB	BIT1			// Invalidate BLT MMU Micro TLB Entries
	#define RESUME_BLT_MMU		BIT0			// Resume BLT MMU Transaction
#define	REG_BMMU_L1PT0	(BLT_BA+0x090)		// TLB Level-One Page Table Entry 0 Descriptor
#define	REG_BMMU_L1PT1	(BLT_BA+0x094)		// TLB Level-One Page Table Entry 1 Descriptor
#define	REG_BMMU_L1PT2	(BLT_BA+0x098)		// TLB Level-One Page Table Entry 2 Descriptor
#define	REG_BMMU_L1PT3	(BLT_BA+0x09C)		// TLB Level-One Page Table Entry 3 Descriptor
#define	REG_BMMU_L1PT4	(BLT_BA+0x0A0)		// TLB Level-One Page Table Entry 4 Descriptor
#define	REG_BMMU_L1PT5	(BLT_BA+0x0A4)		// TLB Level-One Page Table Entry 5 Descriptor
#define	REG_BMMU_L1PT6	(BLT_BA+0x0A8)		// TLB Level-One Page Table Entry 6 Descriptor
#define	REG_BMMU_L1PT7	(BLT_BA+0x0AC)		// TLB Level-One Page Table Entry 7 Descriptor
#define	REG_BMMU_CVA	(BLT_BA+0x0B0)		// MMU Current Virtual Address Register
#define	REG_BMMU_CVPN	(BLT_BA+0x0B4)		// MMU Current Virtual Page Number Register
#define	REG_BMMU_CPA	(BLT_BA+0x0B8)		// MMU Current Physical Address Register
#define	REG_BMMU_CPPN	(BLT_BA+0x0BC)		// MMU Current Physical Page Number register


//#define AR_BA	(w55fa92_AR)
#define REG_AR_CON		(AR_BA+0x0000)	// R/W	Audio Record control register
		#define AR_RST			BIT31
		#define TCONFIG			BIT30			// R/W	Sigma-Delta ADC Test Mode Enable
		#define TSLAVE				BIT29			// R/W	Sigma-Delta ADC Test Mode I2S Interface
		#define TPLLBY				BIT28			// R/W	Sigma-Delta ADC Test Mode PLL Bypass
		#define AR_EDMA			BIT4			// R/W	EDMA Mode Enable
		#define INT_MOD 			NVTBIT(3, 2)		// R/W	ADC block enable bit (10 bits ADC)
		#define AR_INT_EN			BIT1				// R/W	Interrupt Mask
		#define AR_INT				BIT0				// R/W	Interrupt Status
		
#define REG_AR_AGC1		(AR_BA+0x004)	// R/W: 	Audio Record AGC control register
		#define AGC_EN			BIT31			// Auto gain control enable bit
		#define GSTEP				BIT28			// Gain Change Step
		#define OTL				NVTBIT(27, 24)		// Output Target Level
		#define MAXGAIN			NVTBIT(23, 20)		// AGC MAXGAIN Control Register
		#define MINGAIN			NVTBIT(19, 16)		// AGC MINGAIN Control Register
		#define RECOVERY		NVTBIT(11, 0)		// Recovery time

#define REG_AR_AGC2		(AR_BA+0x008)	// R/W: 	Audio Record AGC control register		
		#define ATTACK			NVTBIT(27, 16)		// Attack time
		#define HOLD				NVTBIT(11, 0)		// Hold time		
		
#define REG_AR_NG			(AR_BA+0x00C)	// R/W: 	Audio Record AGC control register		
		#define NG_EN				BIT31			// Noise Gate Enable
		#define DLYTIME			NVTBIT(27, 24)		// Delay Time
		#define IN_NG_TIME			NVTBIT(23, 20)		// Enter NG Function Time
		#define OUT_NG_TIME		NVTBIT(19, 16)		// AGC MAXGAIN Control Register
		#define NG_GAIN			NVTBIT(11, 8)		// AGC MINGAIN Control Register
		#define NG_LEVEL			NVTBIT(4, 0)		// Attack time	
		
#define REG_AUD_DATA1 		(AR_BA+0x0010)	// R/W: Audio data register 1
#define REG_AUD_DATA2		(AR_BA+0x0014)	// R/W: Audio data register 2
#define REG_AUD_DATA3		(AR_BA+0x0018)	// R/W: Audio data register 3
#define REG_AUD_DATA4		(AR_BA+0x001C)	// R/W: Audio data register 4
#define REG_AUD_DATA5 		(AR_BA+0x0020)	// R/W: Audio data register 5
#define REG_AUD_DATA6		(AR_BA+0x0024)	// R/W: Audio data register 6
#define REG_AUD_DATA7		(AR_BA+0x0028)	// R/W: Audio data register 7
#define REG_AUD_DATA8		(AR_BA+0x002C)	// R/W: Audio data register 8
		#define AUDIO_DATA1		NVTBIT(31, 16)		// Converted audio dat 1 in bufferx
		#define AUDIO_DATA0		NVTBIT(15, 0)		// Converted audio dat 0 in bufferx	

#define REG_SDADC_CTL		(AR_BA+0x0030)	// R/W: Sigma-Delta ADC control register
		#define AR_BUSY			BIT31			// I2C Command State
		#define SCK_DIV			NVTBIT(30, 24)		// SCK Timing Divider
		#define AR_DEVICE_ID		NVTBIT(23, 17)		// SCK Timing Divider
		#define AR_RW				BIT16			// Command is Read from ADC or Write to ADC	
		#define AR_ADDR			NVTBIT(15, 8)		// Sigma-Delta ADC I2C Address Information
		#define AR_DATA			NVTBIT(7, 0)		// Sigma-Delta ADC I2C Data Information
		
#define REG_SDADC_AGBIT		(AR_BA+0x0034)	// R/W: Sigma-Delta Auto Gain Temp Bits register	
		#define AGBIT_L			NVTBIT(15, 8)		// Sigma-Delta ADC Auto Gain Temp Bits for Left Channel
		#define AGBIT_R			NVTBIT(7, 0)		// Sigma-Delta ADC Auto Gain Temp Bits for Right Channel

#define REG_AR_DIGIM			(AR_BA+0x0038)		
		#define SMPL_MODE			NVTBIT(9, 8)		// Audio Sample Mode
		#define DIGIM_EN			BIT7				// Digital Microphone Gain Control Enable	
		#define DIGIM_LV			NVTBIT(3, 0)		// Digital Microphone Gain Level

#if 0
#define REG_ADC_TSC	(ADC_BA+0x0004)	// R/W	Touch screen control register
		#define ADC_TSC_MAV_EN 		BIT16				// R/W  MAV Filter Enable/Disable for the Touch Screen AutoMode
		#define ADC_TSC_XY			BIT8				// R/W	This bit control the X-position or Y-position detection when in semi-auto conversion mode
		#define ADC_TSC_XP			BIT7				// R/W	This bit control the interface to XP of touch screen when in normal conversion mode
		#define ADC_TSC_XM			BIT6				// R/W	This bit control the interface to XM of touch screen when in normal conversion mode
		#define ADC_TSC_YP			BIT5				// R/W	This bit control the interface to YP of touch screen when in normal conversion mode
		#define ADC_TSC_YM			BIT4				// R/W	This bit control the interface to YM of touch screen when in normal conversion mode
		#define ADC_PU_EN			BIT3				// R/W	This bit control the internal pull up PMOS in switch box is enable or disable
		#define ADC_TSC_TYPE		NVTBIT(2,1)			// R/W	The touch screen type selection bits
		#define ADC_UD				BIT0				// R	The up down state for stylus in waiting for trigger mode




#define REG_ADC_DLY		(ADC_BA+0x0008)	// R/W	ADC delay register
		#define ADC_DELAY			NVTBIT(17,0)		// R/W	Delay for Conversion.

#define REG_ADC_XDATA	(ADC_BA+0x000C)	// R	10 bits ADC XDATA register
#define REG_ADC_YDATA	(ADC_BA+0x0010)	// R	10 bits ADC YDATA register

#define REG_AUDADC_CTL		(ADC_BA+0x0014)	// R/W	Low Voltage Detector Control register
		#define APB2AHB				BIT31
		#define AUD_STRTIME			NVTBIT(23,16)		
		#define TP_STRTIME			NVTBIT(15,8)			
		#define ADC_VDD12			BIT4			
		#define MIC_BIAS_EN			BIT3		
		#define ADC_CHSEL			NVTBIT(2, 0)			

#define REG_AUDIO_CON	(ADC_BA+0x001C)	// R/W: ADC Control Register
		#define AUDIO_INT_MODE 		NVTBIT(31,30) 	// Audio interrupt mode selection
		#define AUDIO_INT				BIT29			// Audio interrupt flag bits
		#define AUDIO_INT_EN			BIT28			// Audio interrupt flag bits
		#define AUDIO_VOL_EN			BIT27			// Volume control enable bit
		#define AUDIO_HPEN			BIT26			// Record path high pass enable bit
		#define AUDIO_EN				BIT25			// Record enable
		#define AUDIO_RESET			BIT24			// Record path high pass enable bit
		#define AUDIO_CCYCLE			NVTBIT(23, 16)		// Audio conversion cycle (Minimum value = 34)
		#define AUDIO_DATA			NVTBIT(15, 0)		// Last converted audio data before AUD_INT

#define REG_AUDIO_BUF0 	(ADC_BA+0x0020)	// R/W: Audio data register 0
#define REG_AUDIO_BUF1 	(ADC_BA+0x0024)	// R/W: Audio data register 1
#define REG_AUDIO_BUF2 	(ADC_BA+0x0028)	// R/W: Audio data register 2
#define REG_AUDIO_BUF3 	(ADC_BA+0x002C)	// R/W: Audio data register 3
		#define AUDIO_DATA1			NVTBIT(31, 16)	// Converted audio dat 1 in bufferx
		#define AUDIO_DATA0			NVTBIT(15, 0)		// Converted audio dat 0 in bufferx

#define REG_AGCP1	 	(ADC_BA+0x0030)// R/W: AGC Parameter Register Setting
		#define EDMA_MODE			BIT31			// EDMA mode Enable/Dislabe
		#define DOWNSMPSEL			BIT24			// Down Sample Raate Selection. 1=8X. 0=16X
		#define MAXGAIN				NVTBIT(22, 20)	// AGC MAXGAIN Control Register
		#define MINGAIN				NVTBIT(18, 16)	// AGC MINGAIN Control Register
		#define OTL					NVTBIT(15, 12)	// Output Target Level
		#define UPBAND				BIT11			// Up band for AGC
		#define DOWNBAND			BIT10			// Down band for AGC
		#define PRAGA				NVTBIT(9, 8)		// Pre- Amplifer Gain Control
		#define AUDIO_VOL			NVTBIT(5, 0)		// Audio Volume Control

#define REG_AGC_CON		(ADC_BA+0x0034)	// R/W: AGC Control Register
		#define NG_EN				BIT31			// Noise gate enable
		#define AGC_EN				BIT30			// Auto gain control enable bit
		#define PERIOD				NVTBIT(25, 16)	// Period
		#define ATTACK				NVTBIT(11, 8)		// Attack time
		#define RECOVERY				NVTBIT(7, 4)		// Recovery time
		#define HOLD					NVTBIT(3, 0)		// Hold time

#define REG_OPOC 		(ADC_BA+0x0038)	// R/W: AGC Parameter Register Setting
		#define MUTE_SW				BIT31			// Mute control under software mode
		#define OOC					BIT30			// Hardware OP Offset Calculation Enable = 0. Disable =1
		#define OPOCM				BIT29			// OP Offset Cancellation Method
		#define OPOC_SW				NVTBIT(28, 24)	// OP Offset Cancellation
		#define OPOC_TCSN			NVTBIT(17, 16)	// OP Offset Total Calculation Sample Number
		#define OPOC_DSC				NVTBIT(7, 0)		// OP Offset Caculation Delay Sample Count

#define REG_DELPEAKCTL	(ADC_BA+0x0040)
		#define PEAKFPGASEL			NVTBIT(13, 12)
		#define PEAKFPGATHRESHOLD	NVTBIT(9, 0)

#define REG_NG_CTRL 	(ADC_BA+0x0040)	// R: OP Offset Calculation Status
		#define GAINCHGMOD			NVTBIT(25, 24)	// The OP Offset value after hardware calculation
		#define IN_NG_TIME			NVTBIT(23, 20)
		#define OUT_NG_TIME			NVTBIT(19, 16)
		#define NG_GAIN				NVTBIT(13, 8)
		#define NG_LEVEL			NVTBIT(6, 0)

#define REG_STATEFLAG 	(ADC_BA+0x0044)	// R: OP Offset Calculation Status
		#define OP_MUTE_FLAG		BIT31	// The OP Offset value after hardware calculation
		#define NG_FLAG				BIT30
		#define OPOC_CAL_VAL		NVTBIT(20, 16)
		#define AGC_GAIN_NUM		NVTBIT(13, 8)

#define REG_OVFLCTL		(ADC_BA+0x004C)
		#define OVFL_THLD			NVTBIT(31, 16)
		#define OVFL_EN				BIT0
#define REG_OPOCCALSUM		(ADC_BA+0x0050)
		#define CAL_SUM			NVTBIT(9, 7)

#define REG_TSC_SORT4 	(ADC_BA+0x0070)	// 10th Touch Screen MAV (Minimum)
#define REG_TSC_SORT3 	(ADC_BA+0x0074)	// 9th Touch Screen MAV
#define REG_TSC_SORT2 	(ADC_BA+0x0078)	// 8th Touch Screen MAV
#define REG_TSC_SORT1 	(ADC_BA+0x007C)	// 7th Touch Screen MAV
		#define Y_MAV				NVTBIT(25, 16)		// MAV Y-Data
		#define X_MAV				NVTBIT(9, 0)		// MAV X-Data
		#define TS_VALUE			BIT31

#define REG_TSC_MAV_X 	(ADC_BA+0x0080)	// Touch Screen MAV X-Data
		#define X_MAV_AVG	NVTBIT(9, 0)				// MAV X-Data
#define REG_TSC_MAV_Y 	(ADC_BA+0x0084)	// Touch Screen MAV Y-Data
		#define Y_MAV_AVG	NVTBIT(9, 0)				// MAV Y-Data
#endif 

/*-----------------------------------------------------------------------------
 * Define Registers for Touch panel
 *---------------------------------------------------------------------------*/
//#define TP_BA	(w55fa92_TP)
#define REG_TP_CTL1		(TP_BA+0x00)		// Touch Panel control register 1
		#define XP_EN				BIT23			// Analog Power Switch in XP
		#define XM_EN				BIT22			// Analog Power Switch in XM
		#define YP_EN				BIT21			// Analog Power Switch in YP
		#define YM_EN				BIT20			// Analog Power Switch in YM
		#define PLLUP				BIT19			// Pulls Up the XP to Analog Power Supply.
		#define IN_SEL				NVTBIT(18, 16)		// Analog Input Selection Signals
		#define PD_Power 			BIT15			// Down the Keypad Detection
		#define PD_BUF			BIT14 			// Power Down the Internal Reference Buffer
		#define ADC_SLEEP			BIT13			// Power Down Signal
		#define LOW_SPEED			BIT12			// Low Speed Mode
		#define REF_SEL			NVTBIT(11, 10)		// Analog Reference Pair Selection Signals
		#define SLOW_CMP			BIT9				// Slow Down Comparator Enable
		#define TSMODE			BIT7				// Touch Panel Mod
		#define SW_GET			BIT4				// Software Trigger to Get the Data
		#define GET_PRESSURE		BIT3				// Get the Pressure Information
		#define GET_XY			BIT2				// Get the X and Y Average Information
		#define GET_Y				BIT1				// Get the Y Information
		#define GET_X				BIT0				// Get the X Information

#define REG_TP_CTL2		(TP_BA+0x04)		// Touch Panel control register 2
		#define SPL_CHK_TIME		NVTBIT(7, 0)		// Sample & Check Time
#define REG_TP_INTST		(TP_BA+0x08)		// Touch Panel interrupt register
		#define INT_NOR_MASK		BIT6				// Interrupt Mask
		#define INT_TC_MASK		BIT5				// Pen Down Interrupt Mask
		#define INT_KEY_MASK		BIT4				// Key Pad Interrupt Mask
		#define INT_NOR			BIT2				// Normal Interrupt State
		#define INT_TC				BIT1				// Pen Down Interrupt State
		#define INT_KEY			BIT0				// Key Pad Interrupt State
#define REG_XY_DATA				(TP_BA+0x0C)		// X and Y DATA register
		#define PENX				BIT31			// Pen Down Valid/Invalid for X_DATA
		#define X_DATA			NVTBIT(27, 16)		// X Location Register
		#define PENY				BIT15			// Pen Down Valid/Invalid for Y_DATA
		#define Y_DATA			NVTBIT(11, 0)		// Y Location Register		
#define REG_Z_DATA		(TP_BA+0x10)		// Z1 and Z2 DATA register
		#define PENZ1				BIT31			// Pen Down Valid/Invalid for Z1_DATA
		#define Z1_DATA			NVTBIT(27, 16)		// Z1 Location Register
		#define PENZ2				BIT15			// Pen Down Valid/Invalid for Z2_DATA
		#define Z2_DATA			NVTBIT(11, 0)		// Z2 Location Register	
#define REG_NORM_DATA		(TP_BA+0x14)		// Normal Process DATA register (AIN2 and AI3)
		#define NORM_DATA			NVTBIT(11, 0)		// Normal Process Data Register


/*-----------------------------------------------------------------------------
 * Define Registers for AES
 *---------------------------------------------------------------------------*/
#define REG_AESKW0R     (AES_BA+0x000)      // AES Key Word 0 Register
    #define AESKW0          NVTBIT(31, 0)       // bit 31~0 of security key for AES
#define REG_AESKW1R     (AES_BA+0x004)      // AES Key Word 1 Register
    #define AESKW1          NVTBIT(31, 0)       // bit 63~32 of security key for AES
#define REG_AESKW2R     (AES_BA+0x008)      // AES Key Word 2 Register
    #define AESKW2          NVTBIT(31, 0)       // bit 95~64 of security key for AES
#define REG_AESKW3R     (AES_BA+0x00C)      // AES Key Word 3 Register
    #define AESKW3          NVTBIT(31, 0)       // bit 127~96 of security key for AES
#define REG_AESKW4R     (AES_BA+0x010)      // AES Key Word 4 Register
    #define AESKW4          NVTBIT(31, 0)       // bit 159~128 of security key for AES
#define REG_AESKW5R     (AES_BA+0x014)      // AES Key Word 5 Register
    #define AESKW5          NVTBIT(31, 0)       // bit 191~160 of security key for AES
#define REG_AESKW6R     (AES_BA+0x018)      // AES Key Word 6 Register
    #define AESKW6          NVTBIT(31, 0)       // bit 223~192 of security key for AES
#define REG_AESKW7R     (AES_BA+0x01C)      // AES Key Word 7 Register
    #define AESKW7          NVTBIT(31, 0)       // bit 255~224 of security key for AES

#define REG_AESIV0R     (AES_BA+0x020)      // AES Initial Vector Word 0 Register
    #define AESIV0          NVTBIT(31, 0)       // bit 31~0 of Initial Vector for AES CBC mode
#define REG_AESIV1R     (AES_BA+0x024)      // AES Initial Vector Word 1 Register
    #define AESIV1          NVTBIT(31, 0)       // bit 63~32 of Initial Vector for AES CBC mode
#define REG_AESIV2R     (AES_BA+0x028)      // AES Initial Vector Word 2 Register
    #define AESIV2          NVTBIT(31, 0)       // bit 95~64 of Initial Vector for AES CBC mode
#define REG_AESIV3R     (AES_BA+0x02C)      // AES Initial Vector Word 3 Register
    #define AESIV3          NVTBIT(31, 0)       // bit 127~96 of Initial Vector for AES CBC mode

#define REG_AESCR       (AES_BA+0x030)      // AES Control Register
    #define TRANS           BIT5                // Data format transform for AES
    #define KSIZE           NVTBIT(3, 2)        // define the different key sizes for AES
        #define KSIZE_128       0                   // define the key sizes for 128 bits
        #define KSIZE_192       BIT2                // define the key sizes for 192 bits
        #define KSIZE_256       BIT3                // define the key sizes for 256 bits
    #define ENCRPT          BIT1                // define the encryption or decryption for AES
    #define AESON           BIT0                // enable the AES operation

#define REG_AESSAR      (AES_BA+0x034)      // AES Source Address Register
    #define AESSA           NVTBIT(31,0)        // AES Source Address
#define REG_AESDAR      (AES_BA+0x038)      // AES Destination Address Register
    #define AESDA           NVTBIT(31,0)        // AES Destination Address
#define REG_AESBCR      (AES_BA+0x03C)      // AES Byte Count Register
    #define BCNT            NVTBIT(15,0)        // the byte count of plain text that need to do AES

#define REG_AESISR      (AES_BA+0x040)      // AES Interrupt Status Register
    #define AESOK           BIT1                // AES Operation OK Interrupt status
    #define BERR            BIT0                // Bus Error Interrupt status
#define REG_AESIER      (AES_BA+0x044)      // AES Interrupt Enable Register
    #define ENAESOK         BIT1                // Enable AES Operation OK Interrupt
    #define ENBERR          BIT0                // Enable Bus Error Interrupt

#define REG_ACSAR       (AES_BA+0x048)      // AES Current Source Address Register
    #define CAESA           NVTBIT(31,0)        // Current AES Source Address
#define REG_ACDAR       (AES_BA+0x04C)      // AES Current Destination Address Register
    #define CAESDA          NVTBIT(31,0)        // Current AES Destination Address
#define REG_ACBCR       (AES_BA+0x050)      // AES Current Byte Count Register
    #define CBCNT           NVTBIT(15,0)        // Current Byte Count that AES hasn't read yet



/****************************************************************************************************
 *
 * MAC Registers
 *
 ****************************************************************************************************/
#if 1
#define REG_CAMCMR       (EMAC_BA+0x00)   /* CAM Command Register */
#define REG_CAMEN        (EMAC_BA+0x04)   /* CAM Enable Register */
#define REG_CAM0M_Base   (EMAC_BA+0x08)
#define REG_CAM0L_Base   (EMAC_BA+0x0c)
#define REG_CAMxM_Reg(x) (REG_CAM0M_Base+x*0x8)   /*  */
#define REG_CAMxL_Reg(x) (REG_CAM0L_Base+x*0x8)   /*  */

#define REG_TXDLSA       (EMAC_BA+0x88)   /* Transmit Descriptor Link List Start Address Register */
#define REG_RXDLSA       (EMAC_BA+0x8C)   /* Receive Descriptor Link List Start Address Register */
#define REG_MCMDR        (EMAC_BA+0x90)   /* MAC Command Register */
#define REG_MIID         (EMAC_BA+0x94)   /* MII Management Data Register */
#define REG_MIIDA        (EMAC_BA+0x98)   /* MII Management Control and Address Register */
#define REG_FFTCR		 (EMAC_BA+0x9C)   /* FIFO Threshold Control Register */
#define REG_TSDR         (EMAC_BA+0xa0)   /* Transmit Start Demand Register */
#define REG_RSDR         (EMAC_BA+0xa4)   /* Receive Start Demand Register */
#define REG_DMARFC	   	 (EMAC_BA+0xa8)   /* Maximum Receive Frame Control Register */
#define REG_MIEN         (EMAC_BA+0xac)   /* MAC Interrupt Enable Register */
          /* Status Registers */
#define REG_MISTA        (EMAC_BA+0xb0)   /* MAC Interrupt Status Register */
#define REG_MGSTA        (EMAC_BA+0xb4)   /* MAC General Status Register */
#define REG_MPCNT		 (EMAC_BA+0xb8)   /* Missed Packet Count Register */
#define REG_MRPC         (EMAC_BA+0xbc)   /* MAC Receive Pause Count Register */
#define REG_MRPCC        (EMAC_BA+0xc0)   /* MAC Receive Pause Current Count Register */
#define REG_MREPC        (EMAC_BA+0xc4)   /* MAC Remote Pause Count Register */
#define REG_DMARFS       (EMAC_BA+0xc8)   /* DMA Receive Frame Status Register */
#define REG_CTXDSA       (EMAC_BA+0xcc)   /* Current Transmit Descriptor Start Address Register */
#define REG_CTXBSA       (EMAC_BA+0xd0)   /* Current Transmit Buffer Start Address Register */
#define REG_CRXDSA       (EMAC_BA+0xd4)   /* Current Receive Descriptor Start Address Register */
#define REG_CRXBSA       (EMAC_BA+0xd8)   /* Current Receive Buffer Start Address Register */
          /* Diagnostic Registers */
#define REG_RXFSM        (EMAC_BA+0x200)   /* Receive Finite State Machine Register */
#define REG_TXFSM        (EMAC_BA+0x204)   /* Transmit Finite State Machine Register */
#define REG_FSM0         (EMAC_BA+0x208)   /* Finite State Machine Register 0 */
#define REG_FSM1         (EMAC_BA+0x20c)   /* Finite State Machine Register 1 */
#define REG_DCR          (EMAC_BA+0x210)   /* Debug Configuration Register */
#define REG_DMMIR		 (EMAC_BA+0x214)   /* Debug Mode MAC Information Register */
#define REG_BISTR        (EMAC_BA+0x300)   /* BIST Mode Register */

#endif

//#define CRC_BA	(w55fa92_CRC)
#define REG_CRC_CTL		(CRC_BA + 0x00)		// R/W:	CRC Control Register
	#define CRC_MODE		NVTBIT(31, 30)		// CRC Polynomial Mode
	#define CPU_WDLEN		NVTBIT(29, 28)		// CPU Write Data Length
	#define CHECKSUM_COM		BIT27			// Checksum Complement
	#define WDATA_COM		BIT26			// Write Data Complement
	#define CHECKSUM_RVS		BIT25			// Checksum Reverse
	#define WDATA_RVS			BIT24			// Write Data Order Reverse
	#define CRC_SW_RST		BIT1			// CRC Engine Reset
	#define CRCCEN			BIT0			// CRC Channel Enable

#define REG_CRC_WDATA		(CRC_BA + 0x04)		// R/W: CRC Write Data Register
#define REG_CRC_SEED		(CRC_BA + 0x08)		// R/W: CRC Seed Register
#define REG_CRC_CHECKSUM	(CRC_BA + 0x0C)		// R: CRC Check Sum Register
#define REG_DMA_WDATA		(CRC_BA + 0x100)	// R/W: CRC DMA WDATA Register

//#define RF_BA		(w55fa92_RF)
#define REG_RFCODEC_CTL		(RF_BA + 0x00)		// R/W: RF Codec control register
	#define DAT_LEN_BYTE		NVTBIT(31, 16)		// Data Byte Length
	#define RF_SOFT_RST		BIT15			// Soft Reset
	#define PNCTR_MOD		NVTBIT(6, 4)		// Puncture Mode
	#define RF_INT_EN		BIT3			// Interrupt Enable
	#define RF_INT			BIT2			// Interrupt
	#define RF_MODE			BIT1			// RF Codec Mode
	#define RF_START		BIT0			// Start Encode or Decode Function
#define REG_RFCODEC_DAT		(RF_BA + 0x04)		// R/W: Convolution or Viterbi data register
#define REG_RFCODEC_INTRLV	(RF_BA + 0x08)		// R/W: Interleave buffer

//#define RSC_BA	(w55fa92_RSC)
#define REG_RSC_CNTRL		(RSC_BA + 0x00)		// R/W:	Control and Status Register
	#define RD_BUF_FULL		BIT13			// RSCODEC Read Buffer Full Flag
	#define RD_BUF_EMPTY		BIT12			// RSCODEC Read Buffer Full Flag
	#define WR_BUF_FULL		BIT11			// RSCODEC Write Buffer Full Flag
	#define WR_BUF_EMPTY		BIT10			// RSCODEC Write Buffer Full Flag
	#define RS_IE			BIT9			// Interrupt Enable
	#define RS_IF			BIT8			// Interrupt Flag
	#define CI_CLR			BIT5			// Convolutional Interleaver / De-interleaver Clear FIFO
	#define CI_INIT			BIT4			// Convolutional Interleaver / De-interleaver Initial
	#define CI_EN			BIT3			// Convolutional Interleaver / De-interleaver Enable
	#define RS_EN			BIT2			// Reed-Solomon Core Enable
	#define DEC_MODE		BIT1			// RSCODEC Encode / Decode Mode Select

#define REG_RSC_EDMA		(RSC_BA + 0x04)		// R/W: EDMA Control Register
	#define EDMA_GO			BIT0			// EDMA start
#define REG_RSC_RBUF		(RSC_BA + 0x08)		// R: RSCODEC Read Buffer Register
#define REG_RSC_WBUF		(RSC_BA + 0x0C)		// R/W: RSCODEC Write Buffer Register
#define REG_RSC_STATUS		(RSC_BA + 0x10)		// R: Reed-Solomon decoder status register
#define REG_RSC_ERRNUM0		(RSC_BA + 0x14)		// R: Reed-Solomon decoder error number register 0
#define REG_RSC_ERRNUM1		(RSC_BA + 0x18)		// R: Reed-Solomon decoder error number register 1
#define REG_RSC_ERRNUM2		(RSC_BA + 0x1C)		// R: Reed-Solomon decoder error number register 2
#define REG_RSC_ERRNUM3		(RSC_BA + 0x20)		// R: Reed-Solomon decoder error number register 3


// AAC MDCT defnition
#define REG_MDCTPAR		(MDCT_BA+0x00)
	#define WIN_MODE		NVTBIT(18,16)		// 2 for window 2048, 5 for window 256
	#define DECODEREN		BIT0				// 0 for MDCT encoder mode, 1 for MDCT decoder mode
	#define WIN_2048		0x20000
	#define WIN_256			0x50000

#define REG_MDCTCTL		(MDCT_BA+0x04)
	#define MDCTEN			BIT0 				// 0 for MDCT idle, 1 for MDCT enable

#define REG_MDCTSTATE	(MDCT_BA+0x08)
	#define REORDER_BUSY	BIT3
	#define POST_BUSY		BIT2
	#define FFT_BUSY		BIT1
	#define PRE_BUSY		BIT0

#define REG_MDCTINT		(MDCT_BA+0x0C)
	#define DMAOUT_INT_ENABLE	 BIT18       //((~BIT18) & 0xF0000)
	#define DMAIN_INT_ENABLE	 BIT17       //((~BIT17) & 0xF0000)
	#define MDCT_INT_ENABLE		 BIT16       //((~BIT16) & 0xF0000)
	#define DMAOUT_INT			BIT2
	#define DMAIN_INT			BIT1
	#define MDCT_INT			BIT0

#define REG_DMA_RADDR	(MDCT_BA+0x10)

#define REG_DMA_WADDR 	(MDCT_BA+0x14)

#define REG_DMA_DIRECTION	(MDCT_BA+0x18)
	#define DMA_DIRECTION	BIT0				// 0 for read data in, 1 for write data out

#define REG_DMA_STATE	(MDCT_BA+0x1C)
	#define DMA_STATE		BIT0				// 0 for DMA idle, 1 for DMA enable

#define REG_DMA_LENGTH	(MDCT_BA+0x20)
	#define DMA_LEN			NVTBIT(11,0)

/* H.264 Encoder Register */
#define REG_264_PARM0				(H264E_BA+0x000)   	/* SPS/PPS parameters */
	#define PIC_MB_COUNT			NVTBIT(28, 16)		// number of MB in each encoded picture
	#define PIC_HEIGHT			    NVTBIT(14, 8)		// height of each encoded picture in unit of MB
	#define PIC_WIDTH			    NVTBIT( 6, 0)		// width of each encoded picture in unit of MB		
	
#define REG_264_PARM1				(H264E_BA+0x004)   	/* SPS/PPS parameters */
    #define P1_IMODE		             BIT24				// Intra I 4x4 number of mode
    #define P1_TD		                BIT23				// Coefficient threshold disable
	#define P1_CCT			            NVTBIT(22,20)		// Chroma coefficients threshold
	#define P1_LCT			            NVTBIT(18,16)		// Luma coefficients threshold
	#define P1_BETA_OFFSET	            NVTBIT(15,12)		// Beta offset	
	#define P1_ALPHA_OFFSET            NVTBIT(11, 8)		// Alpha offset	
	#define P1_CHROMA_QP_OFFSET        NVTBIT( 4, 0)		// Chroma QP offset
    
#define REG_264_PARM2				(H264E_BA+0x008)   	/* Watermark pattern */
#define REG_264_PARM3				(H264E_BA+0x00C)   	/* Reserved */
#define REG_264_PARM4				(H264E_BA+0x010)   	/* Qp, ILF disabled */
	#define P4_MB_DELTAQP              NVTBIT(19,12)		// mb_delta QP (reserved)
	#define P4_ENCODE_QP               NVTBIT( 9, 4)		// initial value of QPy
	#define P4_ID                      NVTBIT( 3, 2)		// Disable the deblocking operations
	#define P4_PIC_FCNT_MAX            NVTBIT( 1, 0)		// pic_fcnt_max(reserved)
	
#define REG_264_PARM5				(H264E_BA+0x014)   	/* Reserved */
#define REG_264_PARM6				(H264E_BA+0x018)   	/* Reserved */
    #define P6_CC_ADDRESS_MODE         BIT0				// CbCr data of the current frame in the address mode
#define REG_264_PARM7				(H264E_BA+0x01C)   	/* External packing code */

#define REG_264_CMD0				(H264E_BA+0x020)   	/* Encoder commands */
    #define C0_RESET                   BIT31				// Software resets for the entire encoder
	#define C0_WK_STATUS               NVTBIT( 5, 4)		// Watermark status  
    #define C0_LODA_DMA_CHAIN          BIT3				// Load DMA command chain
    #define C0_ENCODE_SLICE_TYPE       BIT1				// Slice type
    #define C0_ENCODE               BIT0				// Command to encode one picture
    
#define REG_264_CMD1				(H264E_BA+0x024)   	/* External packing commands */
    #define C1_CLEAR_BITSTREAM_LEN                       BIT11				// Clear the counter of the bitstream length
    #define C1_CLOSE_BITSTREAM                       BIT10				// Close bitstream  
    #define C1_ADD_RBSP_TRAILING                       BIT9				// Add the RBSP trailing bits
    #define C1_PREVENT_0x3                       BIT8				// Avoid adding 0x03
    #define C1_EXP_GOLOMB_CODE                       BIT7				// Exp-Goloma coded syntax
    #define C1_EXT_PK_GO                       BIT6				// External packing go
	#define C1_EXT_PK_LEN              NVTBIT( 5, 0)		// External packing length          
                      
//#define REG_264_CMD2				(H264E_BA+0x028)   	/* Reserved */
//#define REG_264_CMD3				(H264E_BA+0x02C)   	/* Reserved */
//#define REG_264_CMD4				(H264E_BA+0x030)   	/* Reserved */
//#define REG_264_CMD5				(H264E_BA+0x034)   	/* Reserved */
//#define REG_264_CMD6				(H264E_BA+0x038)   	/* Reserved */
//#define REG_264_CMD7				(H264E_BA+0x03C)   	/* Reserved */

#define REG_264_STS0				(H264E_BA+0x040)   	/* Encoder status (read only)*/
	#define S0_MB_CNT                  NVTBIT(28,16)		// Current MB count
    #define S0_VLC_DONE                       BIT1				// VLC done
    #define S0_FRAME_DONE                      BIT0				// Frame done
	
#define REG_264_STS1				(H264E_BA+0x044)   	/* Bitstream length */
#define REG_264_STS2				(H264E_BA+0x048)   	/* Interrupt */
    #define S2_FD_INT_MASK                       BIT16				// Interrupt mask
    #define S2_FD_INT_STS                       BIT0				// Interrupt status for the frame done
       
#define REG_264_STS3				(H264E_BA+0x04C)   	/* Frame cost */

#define REG_264_DMATH				(H264E_BA+0x094)   	/* DMA FIFO threshold */
    #define DMATH_HALF_FULL                       BIT1				// Issue an AHB command while the FIFO status is more than half-full
    #define DMATH_HALF_EMPTY                       BIT0				// Issue an AHB command while the FIFO status is more than half-empty
    
#define REG_264_DMAA0				(H264E_BA+0x098)   	/* DMA command loader source address */
	#define DMA_A0_SMBA_                    NVTBIT(31, 2)		// System memory base address  
	
#define REG_264_DMAC0				(H264E_BA+0x09C)   	/* DMA command loader control */
    #define DMA_C0_DIR                       BIT15				// Transfer direction
	#define DMA_C0_LEN                  NVTBIT(11, 0)		// Transfer length
	
#define REG_264_DMAA1				(H264E_BA+0x0A8)   	/* DMA bitstream target address */
	#define DMM_A1_SMBA                    NVTBIT(31, 2)		// System memory base address (Word)
	#define DMM_A1_INC                     NVTBIT( 1, 0)		// System memory address increment
	
#define REG_264_DMAC1				(H264E_BA+0x0AC)   	/* DMA bitstream buffer control */
	#define DMA_C1_SLOT_COUNTER            NVTBIT(24,16)		// Slot counter
    #define DMA_C1_D                       BIT15				// Transfer direction	
	#define DMA_C1_TYPE                    NVTBIT(13,12)		// Transfer type
	#define DMA_C1_LENGTH                  NVTBIT(11, 0)		// Transfer length
			
#define REG_264_DMAW1				(H264E_BA+0x0B0)   	/* DMA bitstream buffer control */
	#define DMA_W1_HALF_SLOT_AMOUNT        NVTBIT(31,24)		// Half of slot amount
	
#define REG_264_DMAB1				(H264E_BA+0x0B4)   	/* DMA bitstream buffer control */
	#define DMA_B1_CBBA                    NVTBIT(31, 2)		// Circular buffer base address (word)


/* H.264 Decoder Register */
#define REG_264_INTS				(H264D_BA+0x014)   	/* Interrupt Status */
    #define MASK_INT_BSM_BUF_EMPTY  BIT21				// Mask the BSM buffer empty interrupt
    #define MASK_INT_MV_OVER_RANGE  BIT20				// Mask MV over the range interrupt
    #define MASK_INT_END_FRAME      BIT19				// Mask end of the frame interrupt
    #define MASK_INT_END_SLICE      BIT18				// Mask end of the slice interrupt
    #define MASK_BUS_ERROR_INT      BIT17				// Mask the bus error interrupt
    #define MASK_TRANS_DONE_INT     BIT16				// Mask the transfer done interrupt
    #define INT_BSM_BUF_EMPTY       BIT5				// BSM buffer empty interrupt flag
    #define INT_MV_OVER_RANGE       BIT4				// MV over range interrupt flag
    #define INT_END_FRAME           BIT3				// Frame-end inerrupt flag
    #define INT_END_SLICE           BIT2				// Slice-end interrupt flag
    #define INT_BUS_ERROR           BIT1				// Bus error interrupt flag
    #define INT_TRANS_DONE          BIT0				// DMA transfer done interrupt flag 

#define REG_264_FRAMESIZE			(H264D_BA+0x018)   	/* Frame Size */
	#define FRAMESIZE               NVTBIT(14, 0)		// Frame Size
	
#define REG_264_BSM_BASE			(H264D_BA+0x01C)   	/* Bitstream memory base address */
#define REG_264_RECON_BASE			(H264D_BA+0x020)   	/* Reconstruction frame base address */
#define REG_264_INTRA_BASE			(H264D_BA+0x024)   	/* Intra data base address */
#define REG_264_MBINFO_BASE			(H264D_BA+0x028)   	/* Macro block information base address */
#define REG_264_SOFT_RST			(H264D_BA+0x02C)   	/* Software reset */
    #define DISABLE_MULTI_SLICE     BIT2				// Disable the multi-slice function
    #define BSM_EMPTY               BIT1				// When Bisteam is empty, the system host should set this bit to '1'.
    #define SRESET                  BIT0				// Software reset 
    
#define REG_264_DISP_Y_BASE			(H264D_BA+0x030)   	/* Display frame base address */
#define REG_264_OFFSET_REG0			(H264D_BA+0x034)   	/* Address offset register 0 */
	#define MBINFO_OFFSET           NVTBIT(23,16)		// Macro block information offset address (unit : 8 words)
	#define INTRA_OFFSET            NVTBIT( 7, 0)		// Intra data offset address (unit : 8 words)
	
#define REG_264_OFFSET_REG1			(H264D_BA+0x038)   	/* Address offset register 1 */
	#define ILFTOP_UV_LINEOFFS_BIT  NVTBIT(27,16)		// UV line offset
	#define ILFTOP_Y_LINEOFFS_BIT   NVTBIT(12, 0)		// Y line offset
	
#define REG_264_RECON_LINEOFFS		(H264D_BA+0x03C)   	/* Reconstruction frame address offset */
	#define RECON_LINEOFFS          NVTBIT(10, 0)		// System memory data line offset
	
#define REG_264_DP_LINEOFFS			(H264D_BA+0x040)   	/* Display frame address offset */
    #define DP_Y_LINEOFFS_BIT9      BIT24				// bit[9] of the DP_Y_LINEOFFS
	#define DP_C_LINEOFFS           NVTBIT(23,16)		// Chroma Line offset
	#define DP_Y_LINEOFFS           NVTBIT( 8, 0)		// Luma Line offset
	
#define REG_264_BSM_COUNT			(H264D_BA+0x044)   	/* Bitstream counter */
	#define BSM_READ_COUNT          NVTBIT(31,16)		// Bitstream read buffer counter
	#define BSM_INC_VALID_COUNT     NVTBIT(15, 0)		// Bitstream buffer valid counter
	
#define REG_264_BSM_BUF_LENGTH		(H264D_BA+0x048)   	/* Bitstream buffer length */
	#define BSM_BUF_LENGTH          NVTBIT(15, 0)		// Prepared space of external memory for bitstream buffer (unit : 16 words)
	
#define REG_264_MISC    			(H264D_BA+0x04C)   	/* Miscellaneous */
    #define LCD_MODE_LARGE          BIT4				// The frame size of the LCD display is larger than the reconstructed frame size
    #define LCD_MODE_SMALL          BIT3				// The frame size of the LCD display is smaller than the reconstructed frame size   
    #define DISPLAY_DIS             BIT2				// Display the data moved function 
	#define DISP_MODE               NVTBIT( 1, 0)		// Display format     
	
#define REG_264_DISP_U_BASE 		(H264D_BA+0x050)   	/* Display the chroma U base address */
#define REG_264_DISP_V_BASE			(H264D_BA+0x054)   	/* Display the chroma V base address */
#define REG_264_LCD_PARAM			(H264D_BA+0x058)   	/* LCD display control parameter */
	#define LCD_LEFT_SHIFT          NVTBIT(23,20)		// 
	#define LCD_TOP_SHIFT           NVTBIT(19,16)	
	#define LCD_DISPLAY_HEIGHT      NVTBIT(15, 8)	
	#define LCD_DISPLAY_WIDTH       NVTBIT( 7, 0)		
	
#define REG_264_LCD_FRAME_LINE_OFF	(H264D_BA+0x05C)   	/* LCD display line offset */
	#define LCD_C_FRAME_LINE_OFFSET NVTBIT(27,16)
	#define LCD_Y_FRAME_LINE_OFFSET NVTBIT(11, 0)	
	
#define REG_264_SREG0   			(H264D_BA+0x080)   	/* System register 0 */
	#define PIC_WIDTH_IN_MBS        NVTBIT(31,24)       // The width of the decoded picture in units of MB
	#define PIC_HEIGHT_IN_MAP_UNITS NVTBIT(23,16)       // The height in the slice group map unit of a decoded frame
    #define FRAME_MBS_ONLY_FLGA     BIT15				// Frame or filed macro block selection 
    #define MB_AFF_FLAG             BIT14				// No switching between the field macro blocks within a picture
    #define DIRECT_8X8_INFERENCE_FLAG BIT13				// 
    #define ENTROPY_CODING_MODE_FLAG BIT12				// Entropy coding table selection           	
	#define NUM_REF_IDX_L0_ACTIVE   NVTBIT(11, 7)	    // Maximum reference index for the reference picture list 0
    #define FIELD_PIC_FLAG          BIT1				// Coded frame slice
    
#define REG_264_SREG1       		(H264D_BA+0x084)   	/* System register 1 */
	#define SLICE_TYPE              NVTBIT(21,20)	    // Slice type
	#define FRAME_NUM               NVTBIT(19,15)	    // Frame Number
	#define QP                      NVTBIT(11, 6)	    // QP of the current slice
	#define CHROMA_QP_INDEX_OFFSET  NVTBIT( 5, 1)	    // Chroma QP index offset
    #define CONSTRAINED_INTRA_PRED_FLAG BIT0			// 
    
#define REG_264_SREG2   			(H264D_BA+0x088)   	/* System register 2 */
	#define CURR_SLICE_NR           NVTBIT(16,12)	    // 
	#define IF_DISABLE_IDC          NVTBIT( 9, 8)	    // Enable the in-loop filter function
	#define IF_ALPHA_C0_OFFSET_DIV2 NVTBIT( 7, 4)	    // In accessing the alpha and tc0 de-blocking filter table
	#define IF_BETA_OFFSET_DIV2     NVTBIT( 3, 0)	    // In accessing the beta de-blocking filter table
				
#define REG_264_ADDR_BSM_CTL        (H264D_BA+0x08C)   	/* Bitstream management control register */
	#define SYS_PARSER_CTL          NVTBIT(10, 8)	    // System parser control
	#define CTL_SHIFT_BITS          NVTBIT( 4, 0)	    // Bitstream shift n bits
	
#define REG_264_ADDR_STATUS0		(H264D_BA+0x090)   	/* Bitstream management control status */
	#define CURRENT_MB_X            NVTBIT(31,24)	    // Current MB number in the X axis
	#define CURRENT_MB_Y            NVTBIT(31,24)	    // Current MB number in the Y axis	
    #define PARSER_IDLE             BIT0				// Parser is at the idle state
    
#define REG_264_STREAM_STATUS	    (H264D_BA+0x094)   	/* Stream status */
    #define RBSP_EMPTY              BIT23				// RBSP buffer empty
    #define RBSP_Q_FULL             BIT22				// RBSP buffer full
    #define IQIT2PPC_Q_FULL         BIT21				// IQIT to the prediction pixel compensator buffer full
    #define IQIT2PPC_Q_EMPTY        BIT20				// IQIT to the prediction pixel compensator buffer empty
    #define INTRA_MQ_EMPTY          BIT19				// Prediction queue buffer empty
    #define INTRA_MQ_FULL           BIT18				// Prediction queue buffer full
    #define PPC2LIF_Q_EMPTY         BIT17				// PPC to the in-loop filter buffer empty
    #define PPC2LIF_Q_FULL          BIT16				// PPC to the in-loop filter buffer full
	#define VERSION_ID              NVTBIT(15,12)	    // Version ID    
    #define IQIT_PARAM_EMPTY        BIT9				// IQIT parameter buffer empty
    #define PPC_PARAM_EMPTY         BIT8				// PPC parameter buffer empty
    #define ILF_PARAM_EMPTY         BIT7				// ILF parameter buffer empty
    #define BSB_EMPTY               BIT6				// Bitstream buffer empty
    #define BSB_LE16                BIT5				// Bistream buffer count shift left 16
    
#define REG_264_ADDR_BSM            (H264D_BA+0x098)   	/* Current bitstream data */
#define REG_264_ADDR_UVLDEN			(H264D_BA+0x09C)   	/* Current UVLD length */
	#define UVLD_LENGTH             NVTBIT( 4, 0)	    // Read port of the UVLD length
	
#define REG_264_ADDR_DECODE_CTL0	(H264D_BA+0x0A0)   	/* Enable decode slice layer */
    #define GO_SD_LAYER             BIT0				// Start decoding the slice data layer

#define REG_264_FER0_FRAME			(H264D_BA+0x100)   	/* Reference frame base address 0 */
	#define REFX_FRAME              NVTBIT(31,10)	    // Reference X frame base address

#endif

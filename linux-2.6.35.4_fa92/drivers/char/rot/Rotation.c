//==================================================================
//                                                                          *
// Copyright (c) 2006 Winbond Electronics Corp. All rights reserved.        *
//                                                                          *
//==================================================================
 
//===================================================================
// 
// FILENAME
//     Rotation.c
//
// VERSION
//     0.1
//
// DESCRIPTION
//     Low Level I/O for Rotation Engine for W55FA92
//
// DATA STRUCTURES
//     None
//
// FUNCTIONS
//     None
//
// HISTORY
//     12/08/30		
//                   
//
// REMARK
//     None
//===================================================================
//#include "W55FA92_reg.h"
//#include "wblib.h"
#include <linux/module.h>
#include <linux/cdev.h>
#include <linux/dma-mapping.h>
#include <linux/pagemap.h>
#include <linux/interrupt.h>
#include <linux/poll.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <asm/errno.h>
#include <asm/cacheflush.h>

#include <asm/mach/map.h>
#include <mach/w55fa92_reg.h>
#include <linux/bootmem.h> 
#include "rot.h"

//PFN_ROT (RotIntHandlerTable)[3]={0};

INT32 rotSrcLineOffset(UINT32 uSrcFmt, UINT32 uSrcPacLineOffset);
INT32 rotDstLineOffset(UINT32 uSrcFmt, UINT32 uDstPacLineOffset);
void rotPacketResetDstBufferAddr(S_ROT* psRotConf);
//void CALLBACK rotIntHandler(void);
void rotInit(BOOL bIsRotEngMode, BOOL bIsRotIntEnable);

 		
//Install the call back function for Rotation
//void rotInstallISR(UINT32 u32IntNum,PVOID pvIsr)
//{
//  	RotIntHandlerTable[u32IntNum] = (PFN_ROT)(pvIsr);
//}
#if 1
void  rotIntHandler(void)
{
	UINT32 u32IntStatus =  inp32(REG_RICR);
    if (u32IntStatus & ROTE_FINISH)
	{//Complete
		outp32(REG_RICR, (inp32(REG_RICR) & 0xFFFF0000) | ROTE_FINISH);		//Clear interrupt Complete status
	}	
	else if( u32IntStatus & TG_ABORT )
	{//Abnormal 
		outp32(REG_RICR, (inp32(REG_RICR) & 0xFFFF0000) | TG_ABORT);		//Clear Target Abort
	}
	else if(u32IntStatus & SRAM_OF )
	{//Abnormal 
		outp32(REG_RICR, (inp32(REG_RICR) & 0xFFFF0000) | SRAM_OF);		//Clear Overflow	
	}
	DBG_PRINTF("Done\n");
}
#endif
//===================================================================
//	
//							
//						
//
//			
//
//
//
//
//===================================================================
VOID rotOpen(VOID)
{
	outp32(REG_AHBCLK2, inp32(REG_AHBCLK2) | ROTE_CKE);	
	outp32(REG_AHBCLK, inp32(REG_AHBCLK) | SRAM_CKE);
		
	outp32(REG_CHIPCFG, inp32(REG_CHIPCFG) & ~ROT_IRM);	//32K RAM for ROT
	
	outp32(REG_AHBIPRST, inp32(REG_AHBIPRST) | ROT_RST);			
	outp32(REG_AHBIPRST, inp32(REG_AHBIPRST) & ~ROT_RST);	
	outp32(REG_SCCR, inp32(REG_SCCR) & ~ROTSW_RST);		/* reset ROT */
	outp32(REG_SCCR, inp32(REG_SCCR) | ROTSW_RST);			
	outp32(REG_SCCR, inp32(REG_SCCR) & ~ROTSW_RST);		/* reset ROT */
//	sysInstallISR(IRQ_LEVEL_1, IRQ_ROT, (PVOID)rotIntHandler);	
//	sysEnableInterrupt(IRQ_ROT);
	outp32(REG_RICR,inp32(REG_RICR) | (SRAM_OF_EN | TG_ABORT_EN | ROTE_INT_EN));	
}

VOID rotClose(VOID)
{
//	sysDisableInterrupt(IRQ_ROT);
	while( (inp32(REG_RCR) & ROTE_EN)==ROTE_EN );
	outp32(REG_RICR,inp32(REG_RICR) | (SRAM_OF_EN | TG_ABORT_EN | ROTE_INT_EN));
	outp32(REG_AHBCLK2, inp32(REG_AHBCLK2) & ~ROTE_CKE);	
	outp32(REG_CHIPCFG, inp32(REG_CHIPCFG) | ROT_IRM);	//32K RAM for CPU
}

INT32 rotImageConfig(S_ROT* psRotConf)
{
	UINT32 uRotCfg=0;
	uRotCfg = uRotCfg | (psRotConf->eRotFormat<<6);	
	uRotCfg = uRotCfg | (psRotConf->eBufSize<<4);
	uRotCfg = uRotCfg |	(psRotConf->eRotDir<<1);	
	outp32(REG_RCR, uRotCfg);
	outp32(REG_RIS, psRotConf->u32RotDimHW);

	rotSrcLineOffset( psRotConf->eRotFormat, psRotConf->u32SrcLineOffset);
	outp32(REG_RSISA, psRotConf->u32SrcAddr);
	rotDstLineOffset( psRotConf->eRotFormat, psRotConf->u32DstLineOffset);
	rotPacketResetDstBufferAddr(psRotConf);	
	return Successful;
}

INT32 rotSrcLineOffset(UINT32 uSrcFmt, UINT32 u32SrcPacLineOffset)
{
	switch(uSrcFmt)
	{
    		case E_ROT_PACKET_RGB565:		
    		case E_ROT_PACKET_YUV422:			
    			u32SrcPacLineOffset = u32SrcPacLineOffset*2;			
    			break;
    		case E_ROT_PACKET_RGB888:		
    			u32SrcPacLineOffset = u32SrcPacLineOffset*4;			
    			break;
	}
	outp32(REG_RSILOFF, u32SrcPacLineOffset);						
	return Successful;
}
INT32 rotDstLineOffset(UINT32 u32SrcFmt, UINT32 u32DstPacLineOffset)
{
	switch(u32SrcFmt)
	{	
    		case E_ROT_PACKET_RGB565:
    		case E_ROT_PACKET_YUV422:			
    			u32DstPacLineOffset = u32DstPacLineOffset*2;			
    			break;
    		case E_ROT_PACKET_RGB888:		
    			u32DstPacLineOffset = u32DstPacLineOffset*4;			
    			break;
	}
	outp32(REG_RDILOFF, u32DstPacLineOffset);						
	return Successful;
}


INT32 rotTrigger(void)
{
	INT32 i32ErrCode = Successful;
	
	if(inp32(REG_RCR) & ROTE_EN)
	{//Busy 
		i32ErrCode =ERR_ROT_BUSY;	
	}
	else
	{//Ready
		outp32(REG_RCR, inp32(REG_RCR) | ROTE_EN);	
	}	
	return i32ErrCode;
}
INT32 rotGetPacketPixelWidth(E_ROTENG_FMT ePacFormat)
{
	INT32 i32PixelWidth = 2;
	switch(ePacFormat)
	{
		case E_ROT_PACKET_RGB565:		i32PixelWidth = 2;	break;					 
		case E_ROT_PACKET_RGB888:		i32PixelWidth = 4;	break;	
		case E_ROT_PACKET_YUV422:		i32PixelWidth = 2;	break;	
	}	
	return i32PixelWidth;
}
VOID rotPacketResetDstBufferAddr(S_ROT* psRotConf)
{
	UINT32 u32OrigPacketRealImagWidth=0, u32OrigPacketRealImagHeight=0, u32RotPacketRealImagWidth=0, u32RotPacketRealImagHeight=0;
	UINT32 u32Poffset=0, u32StartAdd0;
	UINT8 u8PixelWidth;
	UINT32 u32LineOffset = 0;
    
	u32StartAdd0 = psRotConf->u32DstAddr;
	u32OrigPacketRealImagHeight = (psRotConf->u32RotDimHW)>>16;
	u32OrigPacketRealImagWidth =  (psRotConf->u32RotDimHW)&0x0FFFF;
	//Consider Offset				
	
	u32LineOffset = psRotConf->u32DstLineOffset;                          //Pixel unit
	u8PixelWidth=rotGetPacketPixelWidth(psRotConf->eRotFormat);

    	//Consider rotation
	switch(psRotConf->eRotDir)
	{	
		case E_ROT_ROT_L90:			
		case E_ROT_ROT_R90:
			u32RotPacketRealImagWidth=u32OrigPacketRealImagHeight+u32LineOffset;
			u32RotPacketRealImagHeight=u32OrigPacketRealImagWidth;		
			break;	
		default:
			u32RotPacketRealImagWidth=u32OrigPacketRealImagWidth+u32LineOffset;
			u32RotPacketRealImagHeight=u32OrigPacketRealImagHeight;		
			break;	
	}			
	switch(psRotConf->eRotDir)
	{
		case E_ROT_ROT_L90:
		u32Poffset=(u32RotPacketRealImagWidth)*(u32OrigPacketRealImagWidth-1)*u8PixelWidth;						
		break;
		case E_ROT_ROT_R90:
		u32Poffset=(u32OrigPacketRealImagHeight-1)*u8PixelWidth; 		
		break;		
	}	
	//u32StartAdd0 = u32StartAdd0 +u32Poffset;
    u32StartAdd0 = u32StartAdd0 +u32Poffset;
    DBG_PRINTF("Rot_Dst_Address0= 0x%x\n", u32StartAdd0);
    
    DBG_PRINTF("\n");
	outp32(REG_RDISA, u32StartAdd0);
}

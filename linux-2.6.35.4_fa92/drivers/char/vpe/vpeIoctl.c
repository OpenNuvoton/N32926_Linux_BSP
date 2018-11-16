/***************************************************************************
 *                                                                         *
 * Copyright (c) 2009 Nuvoton Technology. All rights reserved.             *
 *                                                                         *
 ***************************************************************************/
 
/****************************************************************************
 * 
 * FILENAME
 *     vpe_io.c
 *
 * VERSION
 *     1.0
 *
 * DESCRIPTION
 *     The file for io control
 *
 * DATA STRUCTURES
 *     None
 *
 * FUNCTIONS
 *     None
 *
 *
 * REMARK
 *     None
 **************************************************************************/
#include <linux/module.h>
#include <linux/cdev.h>
#include <linux/dma-mapping.h>
#include <linux/pagemap.h>
#include <linux/interrupt.h>
#include <linux/poll.h>
#include <asm/errno.h>
#include <asm/cacheflush.h>

#include <asm/mach/map.h>
#include <mach/w55fa92_reg.h>
#include <linux/bootmem.h> 
#include "vpe.h"
#include <mach/w55fa92_vpe.h>

UINT32 u32VpeYBufAddr, u32VpeUBufAddr, u32VpeVBufAddr, u32VpeDstBufAddr; 
UINT16 g_u16SrcWidth, g_u16SrcHeight; 		//Src or Dst dimension. 
UINT16 g_u16DstWidth, g_u16DstHeight; 	
UINT32 g_u16SrcLOffset, g_u16SrcROffset; 	//Src or Dst offset
UINT32 g_u16DstLOffset, g_u16DstROffset; 	
UINT32 u32SrcSize, u32DstSize;
UINT32 u32RotDir;
UINT32 u32SrcComPixelWidth, u32DstComPixelWidth;
PUINT32 pu32TLBEntry;


#define DBG_PRINTF(...)
//#define DBG_PRINTF	printk

#define MAX_COUNT				(16)		//Assume max is 16MB
#define MAX_COMPONENT_ENTRY	(2)		//Each component has 2 entry

/*
typedef struct tagEntryInfo
{
	BOOL bIsInvalid;
	UINT32 u32Count;
}S_ENTRY_INFO;
S_ENTRY_INFO sEntryInfo[4][MAX_COUNT] = {0};
*/

void vpeGetReferenceVirtualAddress(
	PUINT32 pu32YBufAddr,
	PUINT32 pu32UBufAddr,
	PUINT32 pu32VBufAddr,
	PUINT32 pu32DstBufAddr)	
{
	*pu32YBufAddr = u32VpeYBufAddr;
	*pu32UBufAddr = u32VpeUBufAddr;
	*pu32VBufAddr = u32VpeVBufAddr;
	*pu32DstBufAddr = u32VpeDstBufAddr;	
}

/*====================================================
	return 
====================================================*/

static void swap2(UINT32 *x,UINT32 *y)
{
   int temp;
   temp = *x;
   *x = *y;
   *y = temp;
}
static void bublesort(UINT32* list, int n)
{
   int i,j;
   for(i=0;i<(n-1);i++)
      for(j=0;j<(n-(i+1));j++)
             if(list[j] > list[j+1])
                    swap2(&(list[j]),&(list[j+1]));
}
/*
void printlist(int list[],int n)
{
   int i;
   for(i=0;i<n;i++)
      printf("%d\t",list[i]);
}
*/
void srcComponent(UINT32 u32CompentIdx, PUINT32 pu32Num, PUINT32 pu32Den)
{
	UINT32 u32Fmt;
	u32Fmt = (__raw_readl(REG_VPE_CMD) & BLOCK_SEQ) >> 20;	
	switch(u32Fmt)
	{
		case VPE_SRC_PLANAR_YONLY:	*pu32Num =1; *pu32Den=1;	break; 
		case VPE_SRC_PLANAR_YUV420:	
		case VPE_SRC_PLANAR_YUV411:	if(u32CompentIdx==0)	
									{
										*pu32Num =1; *pu32Den=1;	
									}	
									else if(u32CompentIdx==1)	
									{
										*pu32Num =1; *pu32Den=4;	
									}	
									else if(u32CompentIdx==2)	
									{
										*pu32Num =1; *pu32Den=4;		
									}	
									break;													
		case VPE_SRC_PLANAR_YUV422:										
		case VPE_SRC_PLANAR_YUV422T:	if(u32CompentIdx==0)	
									{
										*pu32Num =1; *pu32Den=1;	
									}	
									else if(u32CompentIdx==1)	
									{
										*pu32Num =1; *pu32Den=2;	
									}	
									else if(u32CompentIdx==2)	
									{
										*pu32Num =1; *pu32Den=2;		
									}	
									break;						
	
		case VPE_SRC_PLANAR_YUV444:	*pu32Num =1; *pu32Den=1;	break;		
		
		case VPE_SRC_PACKET_YUV422:
		case	VPE_SRC_PACKET_RGB555:
		case VPE_SRC_PACKET_RGB565:	*pu32Num =2; *pu32Den=1;	break;
			
		case VPE_SRC_PACKET_RGB888:	*pu32Num =4; *pu32Den=1;	break;
	}
}
void dstComponent(UINT32 u32CompentIdx, PUINT32 pu32Num, PUINT32 pu32Den)
{
	UINT32 u32Fmt;
	u32Fmt =  (__raw_readl(REG_VPE_CMD) & DEST) >>24;
	switch(u32Fmt)
	{
		case VPE_DST_PACKET_YUV422:
		case	VPE_DST_PACKET_RGB555:
		case VPE_DST_PACKET_RGB565:	*pu32Num =2; *pu32Den=1;	break;
			
		case VPE_DST_PACKET_RGB888:	*pu32Num =4; *pu32Den=1;	break;
	}
}

#if 1
BOOL LRUEntry[4];
#else
BOOL bIsSrcEntry0;
BOOL bIsSrcEntry1;
BOOL bIsSrcEntry2;
BOOL bIsDstEntry0;
#endif
void InitLRUTable(void)
{	
	UINT32 u32Idx;
#if 1
	for(u32Idx=0;u32Idx<4;u32Idx=u32Idx+1)
		LRUEntry[u32Idx] =  0;
#else	
	LRUEntry[0] =  1;	//Next will update 4 Y
	LRUEntry[1] =  1;	//Next will update 5 U
	LRUEntry[2] =  1;	//Next will update 6 V
	LRUEntry[3] =  0;	//Next will update 3 PAC	
#endif										
}
/*
	According to virtual address of page fault. 
	conversion to physical address 
	return to mmusectiontable index
*/
UINT32 Search(UINT32 u32PageFaultVirAddr)
{
	UINT32 u32MMUIdx;
	u32MMUIdx = u32PageFaultVirAddr /0x100000;
	return u32MMUIdx;
}
UINT32 vpeFindMatchAddr(
	UINT32 u32PageFaultVirAddr,
	PUINT32 pu32ComIdx			/* Indicate which component(ebtry) page fault. To update the new entry according to page fault address */
	)
{
	//UINT32 u32YBufAddr, u32UBufAddr,  u32VBufAddr, u32DstBufAddr;
	UINT32 u32BufAddr[4];
	UINT32 u32Offset[4];
	UINT32 u32Tmp[4];
	UINT32 u32Idx=0;
	UINT32 u32MMUIdx;
	
	vpeGetReferenceVirtualAddress(&(u32BufAddr[0]), &(u32BufAddr[1]), &(u32BufAddr[2]), &(u32BufAddr[3]));
	do
	{
		if(u32BufAddr[u32Idx]<=u32PageFaultVirAddr)		/* Report page fault address must be >= pattern start address */				
			u32Offset[u32Idx] = u32PageFaultVirAddr - u32BufAddr[u32Idx];				
		else
			u32Offset[u32Idx]  = 0x7FFFFFFF;
		u32Idx = u32Idx+1;
	}while(u32Idx<4);
	memcpy(u32Tmp, u32Offset, 16);				/* 4 component with word length */
	bublesort(u32Offset, 4);						/* Sorting, the min value will be index-0 */ 
	u32Idx=0;
	while(u32Offset[0]!=u32Tmp[u32Idx])			/* Compare */
		u32Idx=u32Idx+1;						/* u32Idx will be the component index. 0==> Y, 1==>U. 2==>V, 3==>Dst */
		
		
	/* 	LRU */	
	if(LRUEntry[u32Idx]==0)	
		LRUEntry[u32Idx] = 1;
	else
	{
		LRUEntry[u32Idx] = 0;
		u32Idx = u32Idx+4;
	}
	*pu32ComIdx = u32Idx;			
	
	DBG_PRINTF("Component Index = %d\n", u32Idx);	
	u32MMUIdx = Search(u32PageFaultVirAddr);
	DBG_PRINTF("MMU Table Inx = %d\n", u32MMUIdx);	
	return u32MMUIdx;							/* Report level 1 entry index */
}


void vpeHostOperation(
	E_VPE_HOST eHost,
	E_VPE_CMD_OP eOper	
	)
{
	__raw_writel((__raw_readl(REG_VPE_CMD)&~(HOST_SEL|OPCMD))|
						(((eHost<<4)&HOST_SEL) |
						((eOper<<16)&OPCMD))  , REG_VPE_CMD);							 
}
/*==========================================================================
	MMU disable, u32YPacAddr, u32UAddr, u32VAddr is physical address.
	--------------------------------------------------------------------------
	MMU enable, u32YPacAddr, u32UAddr, u32VAddr are virtual address.
==========================================================================*/
void vpeSetSrcBufAddr(		
	UINT32 u32YPacAddr,
	UINT32 u32UAddr,
	UINT32 u32VAddr)
{		
	__raw_writel(u32YPacAddr, REG_VPE_PLYA_PK);
	__raw_writel(u32UAddr, REG_VPE_PLUA);
	__raw_writel(u32VAddr, REG_VPE_PLVA);	
	u32VpeYBufAddr = u32YPacAddr; 
	u32VpeUBufAddr = u32UAddr; 
	u32VpeVBufAddr = u32VAddr;
}
void vpeSetDstBufAddr(UINT32 u32PacAddr)
{	
	__raw_writel(u32PacAddr, REG_VPE_DEST_PK);
	u32VpeDstBufAddr = u32PacAddr;
}
ERRCODE vpeSetFmtOperation(
	E_VPE_SRC_FMT eSrcFmt,
	E_VPE_DST_FMT eDstFmt,
	E_VPE_CMD_OP eOper)
{	
	
///	UINT32 u32BlockSeq=0;
	if(eDstFmt>VPE_DST_PACKET_RGB888)
			return -1; 
	if(eSrcFmt>VPE_SRC_PACKET_RGB888)
			return -1;	
	if(eOper>VPE_OP_MIRROR_H)
			return -1;	
	
	#if 0
	switch(eSrcFmt)		
	{
		
		case VPE_SRC_PLANAR_YUV420: 
		//case VPE_SRC_PLANAR_YUV400: 				// Y Only		
									u32BlockSeq=1; break;
		case VPE_SRC_PLANAR_YUV411:	u32BlockSeq=2; break;								
		case VPE_SRC_PLANAR_YUV422: 	u32BlockSeq=3; break;	
		case VPE_SRC_PLANAR_YUV422T: 	u32BlockSeq=5; break;	
		case VPE_SRC_PLANAR_YUV444: 	u32BlockSeq=9; break;			
		case VPE_SRC_PACKET_YUV422:	
		case VPE_SRC_PACKET_RGB555:
		case VPE_SRC_PACKET_RGB565:
		case VPE_SRC_PACKET_RGB888:	u32BlockSeq=0; break;			
	}	
	#endif
#if 0	
	if(eSrcFmt==VPE_SRC_PLANAR_YUV400)
		u32BlockSeq=0;
			
	__raw_writel((__raw_readl(REG_VPE_CMD) & ~BLOCK_SEQ) |
				((u32BlockSeq<<20)&BLOCK_SEQ), REG_VPE_CMD);
	if(eSrcFmt!=VPE_SRC_PLANAR_YUV422T)
	{//Planar YUV422 Treanpose doesn't map to source format. 							
		__raw_writel((__raw_readl(REG_VPE_CMD) & ~SORC) |
					((eSrcFmt<< 28) & SORC), REG_VPE_CMD);		
	}	
#else
	__raw_writel((__raw_readl(REG_VPE_CMD) & ~BLOCK_SEQ) |
				((eSrcFmt<< 20) & BLOCK_SEQ), REG_VPE_CMD);					
#endif				
				
	__raw_writel((__raw_readl(REG_VPE_CMD) & ~DEST) |
					((eDstFmt<< 24) & DEST), REG_VPE_CMD);			

	return Successful;		
}
void vpeSetSrcOffset(
	UINT16 u16LeftOff, 
	UINT16 u16RightOff)
{//Pixel unit
	__raw_writel((__raw_readl(REG_VPE_SLORO) & ~(SRCLLO|SRCRLO)) |
					(((u16LeftOff<< 16) & SRCLLO) | (u16RightOff &SRCRLO)) , REG_VPE_SLORO);
}
void vpeSetDstOffset(
	UINT16 u16LeftOff, 
	UINT16 u16RightOff)
{//Pixel unit
	__raw_writel((__raw_readl(REG_VPE_DLORO) & ~(DSTLLO|DSTRLO)) |
					(((u16LeftOff<< 16) & DSTLLO) | (u16RightOff &DSTRLO)) , REG_VPE_DLORO);
}

void vpeDstDimension(
	UINT16 u16Width, 
	UINT16 u16Height)
{
	UINT32 u32Height = u16Height;
	UINT32 u32Width = u16Width;
	g_u16DstWidth = u16Width;
	g_u16DstWidth = u16Height;
	__raw_writel((__raw_readl(REG_VPE_VYDSF) & ~VSF_N) |				//Vertical --> Height
					((u32Height<<16) & VSF_N)  , REG_VPE_VYDSF);
																		//Horizontal --> Width		
	__raw_writel((__raw_readl(REG_VPE_HXDSF) & ~HSF_N) |		
					((u32Width<< 16) & HSF_N) , REG_VPE_HXDSF);
							 				
}
void vpeSrcDimension(
	UINT16 u16Width,
	UINT16 u16Height)
{
	g_u16SrcWidth = u16Width;
	g_u16SrcWidth = u16Height;
	__raw_writel((__raw_readl(REG_VPE_VYDSF) & ~VSF_M) |		//Vertical --> Height
				 	(u16Height&VSF_M) , REG_VPE_VYDSF);				 	
	__raw_writel((__raw_readl(REG_VPE_HXDSF) & ~HSF_M) |		//Horizontal --> Width
					  (u16Width &HSF_M) , REG_VPE_HXDSF);			 	
					  
					
}
/*
	VPE will auto converse to full range automaticly.
	If want to set the output pattern as CCIR601. bIsConverseCCIR601 will be set to 1
*/
void vpeSetColorRange(
	BOOL bIsPatCCIR601,
	BOOL bIsConverseCCIR601)
{
	__raw_writel((__raw_readl(REG_VPE_CMD) & ~(CCIR601|LEVEL)) |
					(((bIsPatCCIR601<< 31) & CCIR601) | ((bIsConverseCCIR601<<27)&LEVEL)) , REG_VPE_CMD);
}

void vpeSetScaleFilter(
	BOOL bIsSoftwareMode,
	BOOL bIsBypass3x3Filter,		//Block base 3x3 ==> Line Base 3x3 
	BOOL bisBilinearFilter)
{
#if 0
	__raw_writel((__raw_readl(REG_VPE_CMD) & ~(TAP|BYPASS|BILINEAR)) |
					 	(((bIsSoftwareMode<<8) &TAP) | ((bIsBypass3x3Filter<<9) &BYPASS) |
					 	 ((BisBilinearFilter<<7)&BILINEAR)), REG_VPE_CMD
						);							
#else
	__raw_writel((__raw_readl(REG_VPE_CMD) & ~(TAP|BYPASS|BILINEAR)) |
					 	(((bIsSoftwareMode<<8) &TAP) | ((bIsBypass3x3Filter<<9) &BYPASS) |			//Block base 3x3 filter is need enable if 2 step step 3x3 filter  
					 	 ((bisBilinearFilter<<7)&BILINEAR)), REG_VPE_CMD
						);
	#if 0
	if(bIsBypass3x3Filter==FALSE)
	{//Line base 3x3 enable 
		__raw_writel((__raw_readl(REG_VPE_CMD) | BIT19), REG_VPE_CMD);	//frame base 3X3 filter enable
	}
	else
	{//Line base 3x3 disable 
		__raw_writel((__raw_readl(REG_VPE_CMD) & ~BIT19), REG_VPE_CMD);	//frame base 3X3 filter disable		
	}
	#endif
#endif	
}

void vpeSetScale3x3Coe(
	UINT32 u32Coe1to4,
	UINT32 u32Coe5to8,
	UINT16 u16CoePreCur)
{
	#if 0
	while( (__raw_readl(REG_VPE_TG)&VPE_GO==VPE_GO) );		
	if((u32Coe1to4&0xFF)==0)
	{//Central pixel=0 ==> hardware mode
		__raw_writel(__raw_readl(REG_VPE_CMD)| TAP, REG_VPE_CMD); 	//Hardware mode
	}
	else
	{
		__raw_writel((__raw_readl(REG_VPE_CMD) & ~TAP), REG_VPE_CMD); //Software mode
		__raw_writel(u32Coe1to4, REG_VPE_FCOEF0);
		__raw_writel(u32Coe5to8, REG_VPE_FCOEF1);
		__raw_writel(((__raw_readl(REG_VPE_TG)& ~(TAPC_JUMP|TAPC_CEN)) |
								(u16CoePreCur<<16)&(TAPC_JUMP|TAPC_CEN)) , REG_VPE_TG);
	}							
	#else
	u32Coe1to4 = u32Coe5to8 = u16CoePreCur = 0;
	#endif
}
void vpeSetMatchMacroBlock(
	UINT16 u16YMcu,
	UINT16 u16XMcu)
{	
	UINT32 u32YMcu;
	u32YMcu = u16YMcu;
///	__raw_writel(((u32YMcu << 16) | u16XMcu), REG_VPE_MCU);
}
volatile static UINT32 u32VpeTrigger =0;
void vpeTrigger(void)
{
	UINT32 i,j;
	while( (((__raw_readl(REG_VPE_TG)) & VPE_GO) == VPE_GO) );	
	__raw_writel(0x03, REG_VPE_RESET);
	__raw_writel(0x00, REG_VPE_RESET);
	if((__raw_readl(REG_VMMU_CR)&MMU_EN)==MMU_EN)
	{
		InitLRUTable();				
	}
	
	i = (__raw_readl(REG_VPE_CMD)&HOST_SEL)>>4; 
	j =  (__raw_readl(REG_VPE_CMD)&OPCMD)>>16; 
	if( (i==VPE_HOST_VDEC_LINE) &&  
		((j >=VPE_OP_ROTATE_R90) || (j <=VPE_OP_ROTATE_L90)) )
	{//if Host Select  = VPE_HOST_LINE_BASE and Rotation L and R
		__raw_writel(__raw_readl(REG_VPE_CMD) | BIT11, REG_VPE_CMD);	//Signal buffer. 
	}
	else
	{
		__raw_writel(__raw_readl(REG_VPE_CMD) & ~BIT11, REG_VPE_CMD);	//Dual buffer. 
	}

	__raw_writel(__raw_readl(REG_VPE_TG)|VPE_GO, REG_VPE_TG);
	u32VpeTrigger = u32VpeTrigger+1;
}
BOOL vpeCheckTrigger(void)
{	
	if( (__raw_readl(REG_VPE_TG)&VPE_GO) == VPE_GO )
		return TRUE;
	else
		return FALSE;	
}	
volatile static UINT32 u32MMUEnable = 0;
void vpeEnableVmmu(BOOL bIsEnable)
{
	//__raw_writel(bIsEnable&0x03, REG_VMMU_CR);
	if(bIsEnable==1)
	{
		u32MMUEnable = 1;
		//__raw_writel(0x03, REG_VMMU_CR);		//mmu enable, micro tlb enable
		//__raw_writel(0x13, REG_VMMU_CR);		//SC test mode and mmu enable, micro tlb enable
		__raw_writel(0x1, REG_VMMU_CR);		//mmu enable
	}
	else
	{
		u32MMUEnable = 0;
		__raw_writel(0x00, REG_VMMU_CR);	
	}	
}

void vpeSetTtbAddress(UINT32 u32PhysicalTtbAddr)
{
	UINT32 u32pgd; 
	u32pgd = (u32PhysicalTtbAddr&~0xC0000000);
	__raw_writel(u32pgd, REG_VMMU_TTB);
	pu32TLBEntry = (PUINT32)u32PhysicalTtbAddr;
	DBG_PRINTF("MMU table base (p)= 0x%x\n", u32pgd);
	DBG_PRINTF("MMU table base (v)= 0x%x\n", (UINT32)pu32TLBEntry);
}
UINT32 vpeGetTtbAddress(void)
{
	return (__raw_readl(REG_VMMU_TTB));
}
void vpeSetTtbEntry(UINT32 u32Entry, UINT32 u32Level1Entry)
{//Level 1 entry. 
	if(u32Entry>7)
		return;
	/*	
	
	*/	
	__raw_writel(u32Level1Entry, REG_VMMU_L1PT0+4*u32Entry);		
}
UINT32 vpeGetTtbEntry(UINT32 u32Entry)
{//Level 1 entry. 
	if(u32Entry>3)
		return 0;
	return (__raw_readl(REG_VMMU_L1PT0+4*u32Entry));
}

PUINT32 pu32TTB;
ERRCODE vpeIoctl(UINT32 u32Cmd, UINT32 u32Arg0, UINT32 u32Arg1, UINT32 u32Arg2) 
{
	ERRCODE ErrCode=Successful; 

	switch(u32Cmd)
	{
		case VPE_IOCTL_SET_SRCBUF_ADDR:		
				vpeSetSrcBufAddr(u32Arg0, u32Arg1, u32Arg2);
				if(u32MMUEnable == 1)
				{
					vpeSetTtbEntry(0, pu32TTB[u32Arg0/0x100000]);	
					vpeSetTtbEntry(4,  pu32TTB[u32Arg0/0x100000+1]);	
					vpeSetTtbEntry(1,  pu32TTB[u32Arg1/0x100000]);	
					vpeSetTtbEntry(5,  pu32TTB[u32Arg1/0x100000+1]);	
					vpeSetTtbEntry(2,  pu32TTB[u32Arg2/0x100000]);	
					vpeSetTtbEntry(6,  pu32TTB[u32Arg2/0x100000+1]);
				}	
			break;	
		case VPE_IOCTL_SET_DSTBUF_ADDR:
				vpeSetDstBufAddr(u32Arg0);
				if(u32MMUEnable == 1)	
				{
					vpeSetTtbEntry(3,  pu32TTB[u32Arg0/0x100000]);	
					vpeSetTtbEntry(7,  pu32TTB[u32Arg0/0x100000+1]);
				}
				
			break;	
		case VPE_IOCTL_SET_SRC_OFFSET:
				vpeSetSrcOffset(u32Arg0, u32Arg1);
			break;
		case VPE_IOCTL_SET_DST_OFFSET:
				vpeSetDstOffset(u32Arg0, u32Arg1);
			break;	
		case VPE_IOCTL_SET_SRC_DIMENSION:
				vpeSrcDimension(u32Arg0, u32Arg1);
			break;
		case VPE_IOCTL_SET_DST_DIMENSION:
				vpeDstDimension(u32Arg0, u32Arg1);	
			break;
		case VPE_IOCTL_SET_COLOR_RANGE:			
				vpeSetColorRange(u32Arg0, u32Arg1);
			break;
		case VPE_IOCTL_SET_FILTER:
				if(u32Arg0==VPE_SCALE_DDA)			
					vpeSetScaleFilter(FALSE,			//BOOL bIsSoftware
										TRUE,			//BOOL bIsBypass3x3Filter,
										FALSE);			//BOOL BisBilinearFilter)
			/*
				else if(u32Arg0==VPE_SCALE_3X3)
					vpeSetScaleFilter(FALSE,			//BOOL bIsSoftware,
										FALSE,			//3x3 enable 
										FALSE);			//BOOL BisBilinearFilter)							*/
				else if(u32Arg0==VPE_SCALE_BILINEAR)
					vpeSetScaleFilter(FALSE,			//BOOL bIsSoftware,
										TRUE,			//3x3 disable 
										TRUE);			//BOOL BisBilinearFilter)	
			/*
				else if(u32Arg0==VPE_SCALE_3X3_BILINEAR)   //Not support now
					vpeSetScaleFilter(FALSE,			//BOOL bIsSoftware,
										FALSE,			//3x3 enable
										TRUE);			//Bilinear enable								*/			
			break;
		case VPE_IOCTL_SET_3X3_COEF:
				vpeSetScale3x3Coe(u32Arg0, u32Arg1, u32Arg2);			
			break;
		case VPE_IOCTL_SET_FMT:
				vpeSetFmtOperation(u32Arg0, u32Arg1, u32Arg2);	
			break;
		case VPE_IOCTL_SET_MACRO_BLOCK:
				vpeSetMatchMacroBlock(u32Arg0, u32Arg1);	
			break;
		case VPE_IOCTL_HOST_OP:		
				vpeHostOperation(u32Arg0, u32Arg1);				
			break;	
		case VPE_IOCTL_TRIGGER:	
				vpeTrigger();		
			break;	
		case VPE_IOCTL_CHECK_TRIGGER:				
				ErrCode = vpeCheckTrigger();	//0= Complete, 1=Not Complete, <0 ==> IOCTL Error
			break;	
		case VPE_IOCTL_SET_MMU_ENTRY:
				{
					pu32TTB = (PUINT32)u32Arg1;
					vpeEnableVmmu(u32Arg0);	
					vpeSetTtbAddress(u32Arg1);	
				/*												
					vpeSetTtbEntry(0, pu32TTB[u32VpeYBufAddr/0x100000]);	
					vpeSetTtbEntry(4,  pu32TTB[u32VpeYBufAddr/0x100000+1]);	
					vpeSetTtbEntry(1,  pu32TTB[u32VpeUBufAddr/0x100000]);	
					vpeSetTtbEntry(5,  pu32TTB[u32VpeUBufAddr/0x100000+1]);	
					vpeSetTtbEntry(2,  pu32TTB[u32VpeVBufAddr/0x100000]);	
					vpeSetTtbEntry(6,  pu32TTB[u32VpeVBufAddr/0x100000+1]);	
					vpeSetTtbEntry(3,  pu32TTB[u32VpeDstBufAddr/0x100000]);	
					vpeSetTtbEntry(7,  pu32TTB[u32VpeDstBufAddr/0x100000+1]);	
				*/	
				}
			break;							
		case VPE_IOCTL_SET_TLB_ENTRY:
				if(u32MMUEnable == 1)	
					vpeSetTtbEntry(u32Arg0, u32Arg1);	
			break;											
		default:
			return -1;
	}
	
	
	return ErrCode;
}

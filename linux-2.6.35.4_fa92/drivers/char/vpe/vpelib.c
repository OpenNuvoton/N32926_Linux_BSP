/***************************************************************************
 *                                                                         *
 * Copyright (c) 2009 Nuvoton Technology. All rights reserved.             *
 *                                                                         *
 ***************************************************************************/
 
/****************************************************************************
 * 
 * FILENAME
 *     vpelib.c
 *
 * VERSION
 *     1.0
 *
 * DESCRIPTION
 *     The header file of w55fa92 vpe library.
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
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <asm/errno.h>
#include <asm/cacheflush.h>

#include <asm/mach/map.h>
#include <mach/w55fa92_reg.h>
#include <linux/bootmem.h> 
#include "vpe.h"
#include <mach/w55fa92_vpe.h>

BOOL bIsVPEPageFault = FALSE;
BOOL bIsVPEPageMiss = FALSE;
BOOL bIsVPEAbort = FALSE;
extern PUINT32 pu32TLBEntry;
#define DBG_PRINTF(...)
//#define DBG_PRINTF	printk

struct clk *clk_hclk3 = NULL;
struct clk *clk_vpe = NULL;
struct clk *clk_gve = NULL;

extern UINT32 vpeFindMatchAddr(UINT32 u32PageFaultVirAddr, PUINT32 pu32ComIdx);
extern void vpeSetTtbEntry(UINT32 u32Entry, UINT32 u32Level1Entry);

void vpePageFaultCallback(void)
{
	UINT32 u32PageFaultVirtAddr; 
	UINT32 u32ComIdx;
	UINT32 u32PageFaultIdx; 
	bIsVPEPageFault = TRUE;
	DBG_PRINTF("Page fault******************************\n"); 
	
	u32PageFaultVirtAddr = __raw_readl(REG_VMMU_PFTVA);		
	u32PageFaultIdx = vpeFindMatchAddr(u32PageFaultVirtAddr, &u32ComIdx);				
	vpeSetTtbEntry(u32ComIdx, *(pu32TLBEntry+u32PageFaultIdx));			
	
	DBG_PRINTF("Page faul Addr = 0x%x \n", u32PageFaultVirtAddr); 
	DBG_PRINTF("MMU table base = 0x%x\n", (UINT32)pu32TLBEntry);
	DBG_PRINTF("Update Table index 		= %d\n", u32ComIdx); 
	DBG_PRINTF("Page Fault Virtual Address  	= 0x%x\n", u32PageFaultVirtAddr);														
	DBG_PRINTF("MMU[%d]				= 0x%x\n", u32PageFaultIdx, *(pu32TLBEntry+u32PageFaultIdx));
	__raw_writel(__raw_readl(REG_VMMU_CMD) | RESUME, REG_VMMU_CMD);	//resume		
}	
void vpePageMissCallback(void)
{
	UINT32 u32PageMissVirtuAddr; 
//	UINT32 u32PagMissPhyAddr;
	UINT32 u32ComIdx;
	UINT32 u32PageMissIdx; 

	bIsVPEPageMiss = TRUE;
	DBG_PRINTF("---------------------Page Miss-----------------------------------------------\n"); 	
	u32PageMissVirtuAddr = __raw_readl(REG_VMMU_PFTVA); //In linux must converstion to physical address
	u32PageMissIdx = vpeFindMatchAddr(u32PageMissVirtuAddr, &u32ComIdx);		
	vpeSetTtbEntry(u32ComIdx, *(pu32TLBEntry+u32PageMissIdx));	
	
	DBG_PRINTF("Update Table index 		= %d\n", u32ComIdx); 
	DBG_PRINTF("Page Fault Virtual Address  	= 0x%x\n", u32PageMissVirtuAddr);
	DBG_PRINTF("MMU[%d]				= 0x%x\n", u32PageMissIdx, *(pu32TLBEntry+u32PageMissIdx));

	__raw_writel(__raw_readl(REG_VMMU_CMD) | RESUME, REG_VMMU_CMD);	//resume
}	
/* 
	
*/
#if 0
PFN_VPE_CALLBACK (vpeIntHandlerTable)[6]={0};
#else

#endif
__u32 vpeIntHandler(void)
{
	UINT32 u32VpeInt;
	u32VpeInt = __raw_readl(REG_VPE_INTS)&0x3F;
	if( (u32VpeInt&VP_INTS) == VP_INTS)
	{//VPE complete
		DBG_PRINTF("VPE Complete INT\n");
		__raw_writel((u32VpeInt & ~(TA_INTS | DE_INTS | MB_INTS | PG_MISS | PF_INTS)), REG_VPE_INTS);	/* Clear Interrupt */		
		return 1;
	}	
	if( (u32VpeInt&PF_INTS) == PF_INTS)
	{//Page Fault
		DBG_PRINTF("VPE Page Fault INT\n");
		__raw_writel((u32VpeInt & ~(TA_INTS | DE_INTS | MB_INTS | PG_MISS | VP_INTS)), REG_VPE_INTS);	/* Clear Interrupt */	
		vpePageFaultCallback();
		return 0;	
	}	
	if( (u32VpeInt&PG_MISS) == PG_MISS)
	{//Page Missing
		DBG_PRINTF("VPE Page Miss INT\n");
		__raw_writel((u32VpeInt & ~(TA_INTS | DE_INTS | MB_INTS | PF_INTS |VP_INTS)), REG_VPE_INTS);	/* Clear Interrupt */
		vpePageMissCallback();	
		return 0;
	}	
#if 0
	if( (u32VpeInt&MB_INTS) == MB_INTS)
	{//MB complete, Invalid due to JPEG OTF removed 
		if(vpeIntHandlerTable[3]!=0)	
			vpeIntHandlerTable[3]();			
		__raw_writel((u32VpeInt & ~(TA_INTS | DE_INTS | PG_MISS | PF_INTS |VP_INTS)), REG_VPE_INTS);	/* Clear Interrupt */	
		__raw_writel(__raw_readl(REG_VPE_TG)&~0x01, REG_VPE_TG);
		return;
	}
	if( (u32VpeInt&DE_INTS) == DE_INTS)
	{//Decode error,  Invalid due to JPEG OTF removed 
		if(vpeIntHandlerTable[4]!=0)	
			vpeIntHandlerTable[4]();			/* Clear Interrupt */
		__raw_writel((u32VpeInt & ~(TA_INTS | MB_INTS| PG_MISS | PF_INTS |VP_INTS)), REG_VPE_INTS);		
		return;
	}
#endif
	if( (u32VpeInt&TA_INTS) == TA_INTS)
	{//DMA abort			
		__raw_writel((u32VpeInt & ~(DE_INTS | MB_INTS| PG_MISS | PF_INTS |VP_INTS)), REG_VPE_INTS);		
		printk("VPE Target Abort INT\n");
		bIsVPEAbort = TRUE;			
		return 1;	
	}
	return 0;
}
/*-----------------------------------------------------------------------------------------------------------
*	The Function open VPE driver. 
*	And if specified PLL not meet some costraint, the funtion will search the near frequency and 
*	not over the specified frequency
*	
*	1. Enable clock 
*	2. Set correct clock
*	3. Set multiple pin function
*	3. Reset IP
*	 
*	Return: 
*		Error code or Successful                                                                                                        
-----------------------------------------------------------------------------------------------------------*/

ERRCODE vpeOpen(void)
{
	unsigned long flags;

//	__raw_writel(__raw_readl(REG_AHBCLK) |/*GVE_CKE | HCLK3_CKE*/ HCLK4_CKE | VPE_CKE/* | GE_CKE*/ , REG_AHBCLK);
	clk_hclk3 = clk_get(NULL, "hclk3");
	BUG_ON(IS_ERR(clk_hclk3));
	clk_enable(clk_hclk3);
	// Enable VPE clock.
	clk_vpe = clk_get(NULL, "vpe");
	BUG_ON(IS_ERR(clk_vpe));
	clk_enable(clk_vpe);
	// Enable GVE clock.
	clk_gve = clk_get(NULL, "gve");
	BUG_ON(IS_ERR(clk_gve));
	clk_enable(clk_gve);

	local_irq_save(flags);
	__raw_writel(VPE_RST, REG_AHBIPRST);
	__raw_writel(0, REG_AHBIPRST);	
	local_irq_restore(flags);
	__raw_writel(__raw_readl(REG_VPE_CMD) | BUSRT | BIT13 | BIT12, REG_VPE_CMD);	//Burst write- Dual buffer //Lost Block 	
#if 0
	sysInstallISR(IRQ_LEVEL_1, 
						IRQ_VPE, 
						(PVOID)vpeInIntHandler);						
	sysEnableInterrupt(IRQ_VPE);		
#endif

	return Successful;  
}
/*-----------------------------------------------------------------------------------------------------------
*	The Function close VPE driver. 
*	And if specified PLL not meet some costraint, the funtion will search the near frequency and 
*	not over the specified frequency
*	
*	 
*	Return: 
*		Error code or Successful                                                                                                        
-----------------------------------------------------------------------------------------------------------*/
ERRCODE vpeClose(void)
{
	// IP disable clock
	// Disable VPE clock.
	BUG_ON(IS_ERR(clk_gve));
	clk_disable(clk_gve);
	clk_put(clk_gve);

//	__raw_writel(__raw_readl(REG_AHBCLK) &~(/*GVE_CKE | */VPE_CKE/* | GE_CKE*/) , REG_AHBCLK);
	// Disable VPE clock.
	BUG_ON(IS_ERR(clk_vpe));
	clk_disable(clk_vpe);
	clk_put(clk_vpe);

	// Disable HCLK3 clock.
	BUG_ON(IS_ERR(clk_hclk3));
	clk_disable(clk_hclk3);
	clk_put(clk_hclk3);

	return Successful;  
}
#if 0
/*-----------------------------------------------------------------------------------------------------------
*	The Function install call back function
*   
*	
*	 
*	Return: 
*		Error code or Successful                                                                                                        
-----------------------------------------------------------------------------------------------------------*/
ERRCODE 
vpeInstallCallback(
	E_VPE_INT_TYPE eIntType, 
	PFN_VPE_CALLBACK pfnCallback,
	PFN_VPE_CALLBACK* pfnOldCallback
	)
{
	if(eIntType == VPE_INT_COMP)
	{//VPE complete
		*pfnOldCallback = vpeIntHandlerTable[0];
		vpeIntHandlerTable[0] = (PFN_VPE_CALLBACK)(pfnCallback);
	}	
	else if(eIntType == VPE_INT_PAGE_FAULT)
	{//Page fault
		*pfnOldCallback = vpeIntHandlerTable[1];
		vpeIntHandlerTable[1] = (PFN_VPE_CALLBACK)(pfnCallback);
	}
	else if(eIntType == VPE_INT_PAGE_MISS)
	{
		*pfnOldCallback = vpeIntHandlerTable[2];
		vpeIntHandlerTable[2] = (PFN_VPE_CALLBACK)(pfnCallback);
	}	
	else if(eIntType == VPE_INT_MB_COMP)
	{
		*pfnOldCallback = vpeIntHandlerTable[3];
		vpeIntHandlerTable[3] = (PFN_VPE_CALLBACK)(pfnCallback);
	}	
	else if(eIntType == VPE_INT_MB_ERR)
	{
		*pfnOldCallback = vpeIntHandlerTable[4];
		vpeIntHandlerTable[4] = (PFN_VPE_CALLBACK)(pfnCallback);
	}	
	else if(eIntType == VPE_INT_DMA_ERR)
	{
		*pfnOldCallback = vpeIntHandlerTable[5];
		vpeIntHandlerTable[5] = (PFN_VPE_CALLBACK)(pfnCallback);
	}	
	else
		return E_VPE_INVALID_INT;			
	return Successful;	
}
#endif
ERRCODE 
vpeEnableInt(
	E_VPE_INT_TYPE eIntType
	)
{
	UINT32 u32IntEnable = __raw_readl(REG_VPE_CMD);
	
	if(eIntType>VPE_INT_MB_COMP)
			return -1; 
	
	__raw_writel(u32IntEnable |(1<<eIntType), REG_VPE_CMD);	
	return Successful;
}	
ERRCODE 
vpeDisableInt(
	E_VPE_INT_TYPE eIntType
	)
{
	UINT32 u32IntEnable = __raw_readl(REG_VPE_CMD);
	
	if(eIntType>VPE_INT_MB_COMP)
			return -1; 
	
	__raw_writel(u32IntEnable& ~(1<<eIntType), REG_VPE_CMD);	
	return Successful;
}


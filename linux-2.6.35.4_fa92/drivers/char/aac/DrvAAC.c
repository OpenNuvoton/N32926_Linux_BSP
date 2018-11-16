/****************************************************************
 *                                                              *
 * Copyright (c) Nuvoton Technology Corp. All rights reserved.  *
 *                                                              *
 ****************************************************************/
 
#include "DrvAAC.h"
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <mach/w55fa92_reg.h>


//#define OPT_FPGA_DEBUG
#define OPT_FA95_SPU
#define	E_SUCCESS	0

void AAC_SET_MDCT_BY();
void AAC_SET_DMAIN_BY();
void AAC_SET_DMAOUT_BY();
void AAC_WAIT_MDCT_DONE();
void AAC_WAIT_DMAIN_DONE();
void AAC_WAIT_DMAOUT_DONE();

struct clk *aacclk_hclk4 = NULL;
struct clk *aacclk_aac = NULL;
/*
static void delay(UINT32 kk)
{
	UINT32 ii, jj;
	
	for(ii=0; ii < kk; ii++)
	{
		for(jj=0; jj < 0x10; jj++);	
	}
}
*/

// AAC open
ERRCODE
DrvAAC_Open(void)
{

  
    // enable AHB4 & AHB clock
 //   outp32(REG_AHBCLK, inp32(REG_AHBCLK) | 0x01000004);
  //enable AHB4
//   outp32(REG_AHBCLK, inp32(REG_AHBCLK) | HCLK4_CKE );
       if ( aacclk_hclk4 == NULL )
       {
	aacclk_hclk4 = clk_get(NULL, "hclk4");
	BUG_ON(IS_ERR(aacclk_hclk4));
	clk_enable(aacclk_hclk4);
       }


	// enable AAC engine clock 
//	outp32(REG_AHBCLK2, inp32(REG_AHBCLK2) | AAC_CKE);			// enable AAC engine clock 
        if (aacclk_aac == NULL )
	{
	aacclk_aac = clk_get(NULL, "aac");
	BUG_ON(IS_ERR(aacclk_aac));
	clk_enable(aacclk_aac);
	}
	// reset AAC engine 
//	outp32(REG_AHBIPRST, inp32(REG_AHBIPRST) | AAC_RST);
//	delay(10);
//	outp32(REG_AHBIPRST, inp32(REG_AHBIPRST) & ~AAC_RST);	

	outp32(REG_AHBIPRST, AAC_RST);
	outp32(REG_AHBIPRST, 0);		

	return E_SUCCESS;
}

// AAC close
void DrvAAC_Close(void)
{
   
  	// disable AAC engine clock 
//	outp32(REG_AHBCLK2, inp32(REG_AHBCLK2) & (~AAC_CKE));			// disable AAC engine clock 
        if ( aacclk_aac != NULL )
 	{
	clk_disable(aacclk_aac);
	clk_put(aacclk_aac);
	aacclk_aac = NULL;
	}
        if ( aacclk_hclk4 != NULL )
        {
	clk_disable(aacclk_hclk4);
	clk_put(aacclk_hclk4);
        aacclk_hclk4 = NULL;
	}

}

INT32
DrvAAC_Decoder(
	INT32  i32Size,
	INT32 *pi32inbuf,
	INT32 *pi32outbuf
)
{

   INT32 i32OutputSize, i32InputWord, i32OutputWord;
   i32Size >>= 1;    // n/2 = 1024 or 128
// DMA read
    i32InputWord = i32Size  - 1;
	outp32(REG_DMA_RADDR, (UINT32)pi32inbuf);  // set DMA read buffer
	outp32(REG_DMA_LENGTH, i32InputWord);   // Set DMA length 
	outp32(REG_DMA_DIRECTION, 0);   // 0 for read data in
	
	AAC_SET_DMAIN_BY();
	outp32(REG_MDCTINT, DMAIN_INT_ENABLE);  // enable DMA IN interrupt
    outp32(REG_DMA_STATE, DMA_STATE);        //	DMA enable or idle, poll??
	/*
	while (1)
    {
      if ( inp32(REG_MDCTINT) & DMAIN_INT )
      {
          outp32(REG_MDCTINT, DMAIN_INT);
          break;
      }
    }*/       
	AAC_WAIT_DMAIN_DONE();
	
// MDCT decoder
   if (i32Size == 1024 )
   {
	   outp32(REG_MDCTPAR, WIN_2048|DECODEREN);
	   i32OutputSize = 2048;
   }
   else
   {
       outp32(REG_MDCTPAR, WIN_256|DECODEREN);
       i32OutputSize = 256;
   }
   
   i32OutputWord = i32OutputSize - 1;   
   AAC_SET_MDCT_BY();
   outp32(REG_MDCTINT, MDCT_INT_ENABLE);
   outp32(REG_MDCTCTL, MDCTEN);
   /*
   while (1)
   {
      if ( inp32(REG_MDCTINT) & MDCT_INT )
      {
      	  outp32(REG_MDCTINT, MDCT_INT);
      	  while (1)
      	  {
      	  	 if ((inp32(REG_MDCTSTATE) & 0x0F) == 0 )
      	  	 {
      	  	 	break;
      	  	 }
      	  }	
      	  break;
      }
   }*/
	AAC_WAIT_MDCT_DONE();
	while ((inp32(REG_MDCTSTATE) & 0x0F) != 0);
   
// DMA write
	outp32(REG_DMA_WADDR, (UINT32)pi32outbuf);  // set DMA write buffer
	outp32(REG_DMA_LENGTH, i32OutputWord);   // Set DMA length 
	outp32(REG_DMA_DIRECTION, 1);   // 1 for write data out, 0 for data in
	
	AAC_SET_DMAOUT_BY();
	outp32(REG_MDCTINT, DMAOUT_INT_ENABLE);  // enable DMA write interrupt
    outp32(REG_DMA_STATE, DMA_STATE);        //	DMA enable or idle, poll??
    /*
	while (1)
    {
      if ( inp32(REG_MDCTINT) & DMAOUT_INT )
      {
          outp32(REG_MDCTINT, DMAOUT_INT);
          break;
      }
    }*/
	AAC_WAIT_DMAOUT_DONE();
   	
    return i32OutputSize;    	   	
}



INT32
DrvAAC_Encoder(
	INT32 *pi32inbuf, 
	INT32 *pi32outbuf,
	INT32  i32Size
	
)
{

   INT32 i32OutputSize, i32InputWord, i32OutputWord;
// DMA output
    i32InputWord = i32Size - 1;
	outp32(REG_DMA_RADDR, (UINT32)pi32inbuf);  // set DMA write buffer
	outp32(REG_DMA_LENGTH, i32InputWord);   // Set DMA length 
	outp32(REG_DMA_DIRECTION, 0);   // 0 for read data in
	
	AAC_SET_DMAIN_BY();
	outp32(REG_MDCTINT, DMAIN_INT_ENABLE);  // enable DMA IN interrupt
    outp32(REG_DMA_STATE, DMA_STATE);        //	DMA enable or idle, poll??
    /*
	while (1)
    {
      if ( inp32(REG_MDCTINT) & DMAIN_INT )
      {
          outp32(REG_MDCTINT, DMAIN_INT);
          break;
      }
    }*/
	AAC_WAIT_DMAIN_DONE();
	
// MDCT encoder
   if (i32Size == 2048 )
   {
	   outp32(REG_MDCTPAR, WIN_2048); //|(~DECODEREN));
	   i32OutputSize = 1024;
   }
   else
   {
       outp32(REG_MDCTPAR, WIN_256); //|(~DECODEREN));
       i32OutputSize = 128;
   }
   i32OutputWord = i32OutputSize - 1; 
   AAC_SET_MDCT_BY();
   outp32(REG_MDCTINT, MDCT_INT_ENABLE);
   outp32(REG_MDCTCTL, MDCTEN);
   /*
   while (1)
   {
      if ( inp32(REG_MDCTINT) & MDCT_INT )
      {
      	  outp32(REG_MDCTINT, MDCT_INT);
      	  while (1)
      	  {
      	  	 if ((inp32(REG_MDCTSTATE) & 0x0F) == 0)
      	  	 {
      	  	 	break;
      	  	 }
      	  }	
      	  break;
      }
   }*/
	AAC_WAIT_MDCT_DONE();
	while ((inp32(REG_MDCTSTATE) & 0x0F) != 0);
   
// DMA output
	outp32(REG_DMA_WADDR, (UINT32)pi32outbuf);  // set DMA write buffer
	outp32(REG_DMA_LENGTH, i32OutputWord);   // Set DMA length 
	outp32(REG_DMA_DIRECTION, 1);   // 1 for write data out, 1 for data in
	
	AAC_SET_DMAOUT_BY();
	outp32(REG_MDCTINT, DMAOUT_INT_ENABLE);  // enable DMA OUT interrupt
    outp32(REG_DMA_STATE, DMA_STATE);        //	DMA enable or idle, poll??
    /*
	while (1)
    {
      if ( inp32(REG_MDCTINT) & DMAOUT_INT )
      {
          outp32(REG_MDCTINT, DMAOUT_INT);
          break;
      }
    }*/
	AAC_WAIT_DMAOUT_DONE();
   	
    return i32OutputSize;    	   	
}



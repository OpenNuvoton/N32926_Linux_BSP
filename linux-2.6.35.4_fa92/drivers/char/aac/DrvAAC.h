/****************************************************************
 *                                                              *
 * Copyright (c) Nuvoton Technology Corp. All rights reserved.  *
 *                                                              *
 ****************************************************************/

#ifndef _DRVACC_H
#define _DRVACC_H
 
#include <linux/types.h>
#include <mach/w55fa92_blt.h>
#include <asm/io.h>

#define BOOL						int
#define INT16						int16_t
#define UINT16						uint16_t
#define INT32						int32_t
#define UINT32						uint32_t

enum {
	TRUE	=	1,
	FALSE	=	0
};

#define outp32(addr,value)		iowrite32((u32) value, (void *) addr)
#define inp32(addr)				ioread32((void *) addr)

#define Successful				0
#define ERRCODE					int32_t

// SPU open
ERRCODE
DrvAAC_Open(void);
void DrvAAC_Close(void);

INT32
DrvAAC_Encoder(
	INT32 *pi32inbuf, 
	INT32 *pi32outbuf,
	INT32  i32Size	
);

INT32
DrvAAC_Decoder(
	INT32  i32Size,
	INT32 *pi32inbuf, 
	INT32 *pi32outbuf 
	
);

#endif

/* DrvVPE.h
 *
 * Copyright (c) 2009 Nuvoton technology corporation
 * All rights reserved.
 *  
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#ifndef __ASM_ARM_W55FA92_DRVROT_H
#define __ASM_ARM_W55FA92_DRVROT_H

///#include <linux/config.h>
//#include <linux/videodev.h>

#include <asm/io.h>

typedef enum tagRotEngFmt
{
	E_ROT_PACKET_RGB565 = 0,
	E_ROT_PACKET_RGB888,
	E_ROT_PACKET_YUV422
}E_ROTENG_FMT;
typedef enum tagRotIntNum
{
	E_ROT_COMP_INT =0,
	E_ROT_ABORT_INT=1,
	E_ROT_OVERFLOW_INT=2
}E_ROTENG_INT_NUM;

typedef enum tagRotEngBufSize
{
	E_LBUF_4 = 0,
	E_LBUF_8,
	E_LBUF_16
}E_ROTENG_BUFSIZE;
typedef enum tagRotEngDir
{
	E_ROT_ROT_R90 = 0,
	E_ROT_ROT_L90
}E_ROTENG_DIR;

typedef struct tagRotationEng
{
	E_ROTENG_FMT eRotFormat;		// Assigned the rotation format RGB888/RGB565/YUV422
	E_ROTENG_BUFSIZE eBufSize;		// Assigned the buffer size for on the fly rotation
	E_ROTENG_DIR eRotDir;			// Left/Right
	
	__u32 u32RotDimHW;				// Rotation Dimension  [31:16]==> Height of source pattern, [15:0]==> Width of source pattern 
	__u32 u32SrcLineOffset;			// Source line offset, pixel unit
	__u32 u32DstLineOffset;			// Destination line offset, pixel unit
	__u32 u32SrcAddr;				// Buffer physical start address of source image
	__u32 u32DstAddr;				// Buffer physical start address of rotated image
}S_ROT;	

typedef struct tagBufInfo
{
	__u32 u32Width;
	__u32 u32Height;
	__u32 u32PixelWidth;
	__u32 u32PhysicalAddr;
}S_ROT_BUF;

#define ROT_BUF_INFO				_IOR('o', 30, S_ROT_BUF)	/* Get buffer info */
#define ROT_IO_GET					_IOR('o', 31, S_ROT)		/* Set Rot parameter */
#define ROT_IO_SET					_IOW('o', 32, S_ROT)		/* Set Rot parameter */
#define ROT_IO_TRIGGER				_IO ('o', 33)				/* Trigger */
#define ROT_POLLING_INTERRUPT		_IOR ('o', 34, int)			/* Polling Complete */

#endif//__ASM_ARM_W55FA92_DRVROT_H

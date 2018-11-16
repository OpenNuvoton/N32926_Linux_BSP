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

#ifndef __ASM_ARM_W55FA92_DRVVPE_H
#define __ASM_ARM_W55FA92_DRVVPE_H

///#include <linux/config.h>
#include <linux/videodev.h>

#include <asm/io.h>
/* VPE source format */
typedef enum
{
	VPE_SRC_PLANAR_YONLY  =0,	 
	VPE_SRC_PLANAR_YUV420 =1,
	VPE_SRC_PLANAR_YUV411 =2,
	VPE_SRC_PLANAR_YUV422 =3,		
	VPE_SRC_PLANAR_YUV422T=5,	
	VPE_SRC_PLANAR_YUV444 = 9,
	
	VPE_SRC_PACKET_YUV422 = 12,
	VPE_SRC_PACKET_RGB555 = 13,
	VPE_SRC_PACKET_RGB565 = 14,
	VPE_SRC_PACKET_RGB888 = 15
}E_VPE_SRC_FMT;
/* VPE destination format */
typedef enum 
{
	VPE_DST_PACKET_YUV422=0,
	VPE_DST_PACKET_RGB555,
	VPE_DST_PACKET_RGB565,
	VPE_DST_PACKET_RGB888
}E_VPE_DST_FMT;

/* operation */
typedef enum 
{
	VPE_OP_NORMAL=0x0,
	VPE_OP_ROTATE_R90=0x1,
	VPE_OP_ROTATE_L90,
	VPE_OP_ROTATE_180,
	VPE_OP_MIRROR_V,
	VPE_OP_MIRROR_H
}E_VPE_CMD_OP;

/* scale algorithm */
typedef enum
{
	VPE_SCALE_DDA = 0,				/* 3x3 and Bilinear are disabled */
	//VPE_SCALE_3X3 = 1,				/* Only enable 3x3, Not support now. It has to be approached by 2 steps*/
	VPE_SCALE_BILINEAR = 2,			/* Only enable Bilinear */
	//VPE_SCALE_3X3_BILINEAR = 3		/* Both downscale are enabled, Not support now */
}E_VPE_SCALE_MODE;

/* frame mode or on the fly mode */
typedef enum
{
	VPE_HOST_FRAME  =0,	 		// Block base (8x8 or 16x16)
	VPE_HOST_VDEC_LINE =1,		// Line base. for H264, H.263 annex-j. (4x4 block) 
	//VPE_HOST_JPEG =2,			// Not support now
	//VPE_HOST_VDEC_SW =3		// Software, Block base, Not support now
	VPE_HOST_FRAME_TURBO =3		// Block base, Turbo mode
}E_VPE_HOST;

typedef struct vpe_transform{
	__u32	src_addrPacY;
	__u32	src_addrU;
	__u32	src_addrV;
	__u32	src_format;
	__u32	src_width;
	__u32	src_height;	
	__u32	src_leftoffset;
	__u32	src_rightoffset;	

	__u32	dest_addrPac;
	__u32	dest_format;
	__u32	dest_width;
	__u32	dest_height;
	__u32	dest_leftoffset;
	__u32	dest_rightoffset;

	__u32	algorithm;
	E_VPE_CMD_OP	rotation;
	

} vpe_transform_t;



#define VPE_INIT						_IO ('k', 130)
#define VPE_IO_SET					_IOW('k', 131, unsigned int)
#define VPE_IO_GET					_IOR('k', 132, vpe_transform_t *)
//#define VPE_IO_GET					_IOR('k', 132, unsigned int)
#define VPE_GET_MMU_MODE			_IOR('k', 133, unsigned int)
#define VPE_SET_FORMAT_TRANSFORM	_IOW('k', 134, vpe_transform_t *)
#define VPE_WAIT_INTERRUPT			_IO ('k', 135)
#define VPE_TRIGGER					_IO ('k', 136)
#define VPE_STOP						_IO ('k', 137)
#define VPE_POLLING_INTERRUPT			_IOR('k', 138, unsigned int)
#define VPE_SET_MMU_MODE			_IOW('k', 139, unsigned int)

#endif//__ASM_ARM_W55FA92_DRVVPE_H

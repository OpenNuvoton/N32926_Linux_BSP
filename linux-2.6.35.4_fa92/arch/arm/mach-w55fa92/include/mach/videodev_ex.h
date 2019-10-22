/* videodev_ex.h
 *
 * Copyright (c) Nuvoton technology corporation
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#ifndef __LINUX_VIDEODEV_EX_H
#define __LINUX_VIDEODEV_EX_H

#ifndef __LINUX_VIDEODEV_H
#error Please include videodev.h at first
#endif

typedef struct{
	unsigned int u32RemainBufSize;
	unsigned int u32RemainBufPhyAdr;
}S_BUF_INFO;

typedef struct 
{
	int32_t i32PipeBufNo;
	int32_t i32PipeBufSize;
	int32_t i32CurrPipePhyAddr;
	int32_t i32CurrBufIndex;
}S_PIPE_INFO;

typedef struct{
	int32_t i32SharedSensor;
	int32_t i32DependentPacketBuf;
	int32_t i32DependentPlanarBuf;
}S_KERNEL_CONFIG;

typedef struct{
	int32_t i32planar_pos_x;
	int32_t i32planar_pos_y;
}S_PLANAR_POSITION;

#define	VIDIOCGCAPTIME				_IOR('v',30, struct v4l2_buffer)		/* Get Capture time */
#define	VIDIOCSBRIGHTNESS			_IOW('v',31, int)
#define	VIDIOCGBRIGHTNESS			_IOR('v',32, int)
#define	VIDIOCSCONTRAST				_IOW('v',33, int)
#define	VIDIOCGCONTRAST				_IOR('v',34, int)
#define	VIDIOCSSHARPNESS			_IOW('v',35, int)
#define	VIDIOCGSHARPNESS			_IOR('v',36, int)
#define	VIDIOCSWHITEBALANCE			_IOW('v',37, int)
#define	VIDIOCGWHITEBALANCE			_IOR('v',38, int)
#define	VIDIOCSNOISEREDUCTION		_IOW('v',39, int)
#define	VIDIOCGNOISEREDUCTION		_IOR('v',40, int)
#define	VIDIOCSCOLORSATURATION		_IOW('v',41, int)
#define	VIDIOCGCOLORSATURATION		_IOR('v',42, int)
#define	VIDIOCSPREVIEW				_IOR('v',43, int)
#define	VIDIOCSFLICKERFREQ			_IOW('v',44, int)
#define	VIDIOCGSYSUPTIME			_IOR('v',45, struct timeval)/*Get system up time*/
#define	VIDIOCSIRLED				_IOR('v',46, int)			/* 0: off 1: on 2: auto*/
#define	VIDIOCGIRLEDONOFF			_IOR('v',47, int)			/* 0: off 1: on */


#define VIDIOC_G_DIFF_OFFSET		_IOR('v',48, int)			/* Get diff offset address and size */ 
#define VIDIOC_G_DIFF_SIZE			_IOR('v',49, int)
#define VIDIOC_S_MOTION_THRESHOLD	_IOW('v',50, int)			/* Set motion detection threshold */
#define VIDIOC_QUERY_SENSOR_ID		_IOR('v',51, int)

/* FA95 new create */
#define	VIDIOC_G_BUFINFO			_IOR('v',52, S_BUF_INFO)	

#define VIDIOC_G_PACKET_INFO		_IOR('v',53, S_PIPE_INFO)			/* Get Packet offset address and size */ 
#define VIDIOC_G_PLANAR_INFO		_IOR('v',54, S_PIPE_INFO)			/* Get Planar offset address and size */ 
#define VIDIOC_S_MAP_BUF			_IOW('v',55, int)					/* Specified the mapping buffer */
#define VIDIOC_G_KERNEL_INFO		_IOR('v',56, S_KERNEL_CONFIG)	

#define VIDIOC_S_PACKET_STRIDE		_IOW('v',57, int)
#define VIDIOC_S_PLANAR_STRIDE		_IOW('v',58, int)
#define VIDIOC_S_PLANAR_POSITION	_IOW('v',59, S_PLANAR_POSITION)
#define VIDIOC_S_PLANAR_HEIGHT		_IOW('v',60, int)
	
enum{
        OV_7648=1,
        PP_PO3030K,
        PP_PO2010D2,
        OV_9660,
        OV_9653,
        OV_7725=7725,
        OV_7670=7670,
        OV_2640,
        OV_6880,
        NT_99140=99140,
		NT_99141=99141,
		NT_99142=99142,
        NT_99050=99050,
		NT_99160=99160,
		NT_99252=99252,	
        NT_99240=99240,	
		NT_99340=99340,
		HM_1375=1375,
		HM_1055=1055,
		SP_1628=1628,	
		GM_7150=7150,
		TW_9900=9900,
		TW_9912=9912,
		GC_0303=303,
		GC_0308=308,
		AV_1000=1000,		
		TVP_5150=5150,	
		OV_10633=10633,
		SC_1046=1046,
		XC_7021=7021,
                PO_2210N=2210
};

#endif /* __LINUX_VIDEODEV_EX_H */

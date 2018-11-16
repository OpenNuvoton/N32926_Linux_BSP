/*
 * w55fa92_osd.h
 *
 */

#ifndef _W55FA92OSD_H_
#define _W55FA92OSD_H_

#include <linux/compiler.h>

typedef enum {
  // All functions return -2 on "not open"
  OSD_Close=1,    // ()
  // Disables OSD and releases the buffers (??)
  // returns 0 on success

  OSD_Open,       // (cmd + color_format)
  // Opens OSD with color format
  // returns 0 on success

  OSD_Show,       // (cmd)
  // enables OSD mode
  // returns 0 on success

  OSD_Hide,       // (cmd)
  // disables OSD mode
  // returns 0 on success

  OSD_Clear,      // (cmd )
  // clear OSD buffer with color-key color
  // returns 0 on success
  
  OSD_Fill,      // (cmd +)
  // clear OSD buffer with assigned color
  // returns 0 on success

  OSD_FillBlock,      // (cmd+X-axis)  
  // set OSD buffer with user color data (color data will be sent by "write()" function later
  // returns 0 on success
  
  OSD_SetTrans,   // (transparency{color})
  // Set transparency color-key
  // returns 0 on success

  OSD_ClrTrans,   // (transparency{color})
  // Disable transparency color-key
  // returns 0 on success

  OSD_SetBlend,   // (blending{weight})
  // Enable alpha-blending and give weighting value, 0 - 0xFF (opaque)
  // returns 0 on success

  OSD_ClrBlend,   // (blending{weight})
  // Disable alpha-blending
  // returns 0 on success
} OSD_Command;

typedef enum {
	OSD_RGB555=0xAB08, 
  	OSD_RGB565,       
  	OSD_RGBx888,       
	OSD_RGB888x,  
	OSD_ARGB888,  	
  	OSD_Cb0Y0Cr0Y1=0xAB00,       
  	OSD_Y0Cb0Y1Cr0,       
  	OSD_Cr0Y0Cb0Y1,       
  	OSD_Y0Cr0Y1Cb0,       
  	OSD_Y1Cr0Y0Cb0,       
  	OSD_Cr0Y1Cb0Y0,       
  	OSD_Y1Cb0Y0Cr0,       
  	OSD_Cb0Y1Cr0Y0,       
} OSD_Format;

typedef struct osd_cmd_s {
	int cmd;
	int x0;	
	int y0;
	int x0_size;
	int y0_size;
	int color;		// color_format, color_key
	int alpha;		// alpha blending weight
	int format;	
//	void __user *data;
} osd_cmd_t;

//#define OSD_SIZE_H		160
//#define OSD_SIZE_H		LCDWIDTH


#if 0
	/* OSD_OpenRaw: set 'color' to desired window type */
	typedef enum {
		OSD_BITMAP1,           /* 1 bit bitmap */
		OSD_BITMAP2,           /* 2 bit bitmap */
		OSD_BITMAP4,           /* 4 bit bitmap */
		OSD_BITMAP8,           /* 8 bit bitmap */
		OSD_BITMAP1HR,         /* 1 Bit bitmap half resolution */
		OSD_BITMAP2HR,         /* 2 bit bitmap half resolution */
		OSD_BITMAP4HR,         /* 4 bit bitmap half resolution */
		OSD_BITMAP8HR,         /* 8 bit bitmap half resolution */
		OSD_YCRCB422,          /* 4:2:2 YCRCB Graphic Display */
		OSD_YCRCB444,          /* 4:4:4 YCRCB Graphic Display */
		OSD_YCRCB444HR,        /* 4:4:4 YCRCB graphic half resolution */
		OSD_VIDEOTSIZE,        /* True Size Normal MPEG Video Display */
		OSD_VIDEOHSIZE,        /* MPEG Video Display Half Resolution */
		OSD_VIDEOQSIZE,        /* MPEG Video Display Quarter Resolution */
		OSD_VIDEODSIZE,        /* MPEG Video Display Double Resolution */
		OSD_VIDEOTHSIZE,       /* True Size MPEG Video Display Half Resolution */
		OSD_VIDEOTQSIZE,       /* True Size MPEG Video Display Quarter Resolution*/
		OSD_VIDEOTDSIZE,       /* True Size MPEG Video Display Double Resolution */
		OSD_VIDEONSIZE,        /* Full Size MPEG Video Display */
		OSD_CURSOR             /* Cursor */
	} osd_raw_window_t;
	
	typedef struct osd_cap_s {
		int  cmd;
		long val;
	} osd_cap_t;
	
	
	#define OSD_SEND_CMD            _IOW('v', 160, unsigned long)
	#define OSD_GET_CAPABILITY      _IOR('v', 161, osd_cap_t*)
#endif
#endif

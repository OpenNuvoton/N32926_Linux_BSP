//--------------------------------------------------------------
//
// Copyright (c) Nuvoton Technology Corp. All rights reserved.
//
//--------------------------------------------------------------

#ifndef __W55FA92_BLT_H__
#define __W55FA92_BLT_H__

#include <linux/ioctl.h>

#ifdef	__cplusplus
extern "C"
{
#endif

typedef enum {
	eDRVBLT_DISABLE = 0,
	eDRVBLT_ENABLE = 1
} E_DRVBLT_FILLOP;

typedef enum
{
	eDRVBLT_EFFECTIVE,	
	eDRVBLT_NO_EFFECTIVE	
} E_DRVBLT_REVEAL_ALPHA;

typedef enum
{
	eDRVBLT_BIG_ENDIAN,	
	eDRVBLT_LITTLE_ENDIAN	
} E_DRVBLT_PALETTE_ORDER;

typedef enum {
	eDRVBLT_HASTRANSPARENCY = 1,	
	eDRVBLT_HASCOLORTRANSFORM = 2, 
	eDRVBLT_HASALPHAONLY = 4		
} E_DRVBLT_TRANSFORM_FLAG;

typedef enum {
	eDRVBLT_SRC_ARGB8888 = 1,	
	eDRVBLT_SRC_RGB565 = 2,
	eDRVBLT_SRC_1BPP = 4,
	eDRVBLT_SRC_2BPP = 8,
	eDRVBLT_SRC_4BPP = 16,
	eDRVBLT_SRC_8BPP = 32
} E_DRVBLT_BMPIXEL_FORMAT;

typedef enum {
	eDRVBLT_DEST_ARGB8888 = 1,	
	eDRVBLT_DEST_RGB565 = 2,
	eDRVBLT_DEST_RGB555 = 4
} E_DRVBLT_DISPLAY_FORMAT;

typedef enum {
	eDRVBLT_REPEAT_EDGE = 1,
	eDRVBLT_NOTSMOOTH = 2,
	eDRVBLT_CLIP = 4
} E_DRVBLT_FILL_STYLE;

typedef struct {
	int a;
	int b;
	int c;
	int d;
} S_DRVBLT_MATRIX;

typedef struct {
	unsigned char	u8Blue;
	unsigned char	u8Green;
	unsigned char	u8Red;
	unsigned char	u8Alpha;
} S_DRVBLT_ARGB8;

typedef struct {
	short	i16Blue;
	short	i16Green;
	short	i16Red;
	short	i16Alpha;
} S_DRVBLT_ARGB16;

typedef struct {
	short	i16Xmin;
	short	i16Xmax;
	short	i16Ymin;
	short	i16Ymax;
} S_DRVBLT_RECT;

typedef struct {
	S_DRVBLT_ARGB8*	pSARGB8;
	unsigned int	u32PaletteInx;
	unsigned int	u32Num;
	unsigned int	u32SrcImageAddr;
	int		i32Stride;
	int		i32XOffset;
	int		i32YOffset;
	short		i16Width;
	short		i16Height;
} S_DRVBLT_SRC_IMAGE;

typedef struct {
	unsigned int	u32FrameBufAddr;
	int		i32XOffset;
	int		i32YOffset;
	int		i32Stride;
	short		i16Width;
	short		i16Height;
} S_DRVBLT_DEST_FB;

// NOTE: Size of "enum" may get different dependent on compiler setting. Use "int" instead.

typedef struct {
	int rotationDx, rotationDy;
	S_DRVBLT_MATRIX matrix;
	int srcFormat;	// E_DRVBLT_BMPIXEL_FORMAT
	int destFormat;	// E_DRVBLT_DISPLAY_FORMAT
	int flags;
	S_DRVBLT_ARGB16 colorMultiplier;
	S_DRVBLT_ARGB16 colorOffset;
	int fillStyle;	// E_DRVBLT_FILL_STYLE
	void *userData;
} S_DRVBLT_BLIT_TRANSFORMATION;

typedef struct {
	S_DRVBLT_RECT rect;
	S_DRVBLT_ARGB8 color;
	unsigned int    u32FrameBufAddr;
	int rowBytes;
	int format;	// E_DRVBLT_DISPLAY_FORMAT
	int blend;
} S_DRVBLT_FILL_OP;

typedef struct {
	S_DRVBLT_SRC_IMAGE src;
	S_DRVBLT_DEST_FB dest;
	S_DRVBLT_BLIT_TRANSFORMATION *transformation;
} S_DRVBLT_BLIT_OP;

typedef enum
{
	eDRVBLT_RGB888 = 1,
	eDRVBLT_RGB555 = 2,
	eDRVBLT_RGB565 = 4,
	eDRVBLT_YCbCr422 = 8
} E_DRVBLT_COLOR_FORMAT;


#define BLT_IOC_MAGIC 'v'
#define BLT_IOC_MAXNR 200

#define BLT_TRIGGER						_IO(BLT_IOC_MAGIC, 130)
#define BLT_SET_BLIT					_IOW(BLT_IOC_MAGIC, 131, S_DRVBLT_BLIT_OP)
#define BLT_GET_BLIT					_IOR(BLT_IOC_MAGIC, 132, S_DRVBLT_BLIT_OP)
#define BLT_SET_FILL					_IOW(BLT_IOC_MAGIC, 133, S_DRVBLT_FILL_OP)
#define BLT_GET_FILL					_IOR(BLT_IOC_MAGIC, 134, S_DRVBLT_FILL_OP)
#define BLT_FLUSH						_IO(BLT_IOC_MAGIC, 135)
#define BLT_SET_RGB565_COLORKEY 		_IOW(BLT_IOC_MAGIC, 143, unsigned int)
#define BLT_ENABLE_RGB565_COLORCTL		_IO(BLT_IOC_MAGIC, 144)
#define BLT_DISABLE_RGB565_COLORCTL		_IO(BLT_IOC_MAGIC, 145)
#define BLT_SRCFMT_PREMULALPHA		    _IO(BLT_IOC_MAGIC, 146)
#define BLT_SRCFMT_NONPREMULALPHA       _IO(BLT_IOC_MAGIC, 147)

/* ===========================================================
	S means "Set" through a ptr
	T means Tell" directly with the argument value
	G means "Get": reply by setting through a pointer
	Q means "Query": response is on the return value
	X means "eXchange": switch G and S atomically
	H means "sHift": switch T and Q atomically
  ============================================================ */

#define BLT_IOCTRIGGER					BLT_TRIGGER
#define BLT_IOCSBLIT					BLT_SET_BLIT
#define BLT_IOCGBLIT					BLT_GET_BLIT
#define BLT_IOCSFILL					BLT_SET_FILL
#define BLT_IOCGFILL					BLT_GET_FILL
#define BLT_IOCFLUSH					BLT_FLUSH
#define BLT_IOCTRGB565COLORKEY			_IO(BLT_IOC_MAGIC, 150)
#define BLT_IOCQRGB565COLORKEY			_IO(BLT_IOC_MAGIC, 151)
#define BLT_IOCTENABLERGB565COLORKEY	BLT_ENABLE_RGB565_COLORCTL
#define BLT_IOCTDISABLERGB565COLORKEY	BLT_DISABLE_RGB565_COLORCTL
#define BLT_IOCTBLTSRCFMTPREMULALPHA    BLT_SRCFMT_PREMULALPHA
#define BLT_IOCTBLTSRCFMTNONPREMULALPHA BLT_SRCFMT_NONPREMULALPHA
#define BLT_IOCQBUSY					_IO(BLT_IOC_MAGIC, 152)
#define BLT_IOCTMMU						_IO(BLT_IOC_MAGIC, 153)
#define BLT_IOCQMMU						_IO(BLT_IOC_MAGIC, 154)

#ifdef	__cplusplus
}
#endif

#endif	// __W55FA92_BLT_H__


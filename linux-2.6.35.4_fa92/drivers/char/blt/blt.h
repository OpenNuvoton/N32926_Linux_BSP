//--------------------------------------------------------------
//
// Copyright (c) Nuvoton Technology Corp. All rights reserved.
//
//--------------------------------------------------------------

#ifndef __BLT_H__
#define __BLT_H__

#ifdef  __cplusplus
extern "C"
{
#endif

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

#define BLT_ERR_ID					0xFFFF1500

#define ERR_BLT_INVALID_INT			(BLT_ERR_ID | 0x01)
#define ERR_BLT_INVALID_SRCFMT		(BLT_ERR_ID | 0x02)
#define ERR_BLT_INVALID_DSTFMT		(BLT_ERR_ID | 0x03)

#define outp32(addr,value)		iowrite32((u32) value, (void *) addr)
#define inp32(addr)				ioread32((void *) addr)

#define Successful				0
#define ERRCODE					int32_t

typedef void (*PFN_BLT_CALLBACK) (unsigned long usrDat);

typedef enum {
	BLT_INT_CMPLT		= 1,
	BLT_INT_PGFLT		= 2,
	BLT_INT_PGMS		= 3
} E_BLT_INT_TYPE;

void _blt_intr_hdlr (void);

ERRCODE bltOpen (void);
void bltClose (void);
void bltSetTransformMatrix (S_DRVBLT_MATRIX sMatrix);

void bltGetTransformMatrix (S_DRVBLT_MATRIX* psMatrix);

ERRCODE bltSetSrcFormat (E_DRVBLT_BMPIXEL_FORMAT eSrcFmt);

E_DRVBLT_BMPIXEL_FORMAT 
bltGetSrcFormat(void);

ERRCODE 
bltSetDisplayFormat(
	E_DRVBLT_DISPLAY_FORMAT eDisplayFmt	// [in] Display Format 
);

E_DRVBLT_DISPLAY_FORMAT 
bltGetDisplayFormat(void);

void bltEnableInt (E_BLT_INT_TYPE eIntType);
void bltDisableInt (E_BLT_INT_TYPE eIntType);
BOOL bltIsIntEnabled (E_BLT_INT_TYPE eIntType);
BOOL bltPollInt (E_BLT_INT_TYPE eIntType);
void bltInstallCallback(E_BLT_INT_TYPE eIntType, PFN_BLT_CALLBACK newCb, unsigned long newCbUsrDat, PFN_BLT_CALLBACK *oldCb_p, unsigned long *oldCbUsrDat_p);

void bltSetColorMultiplier(
	S_DRVBLT_ARGB16 sARGB16	// [in] ARGB Multiplier 
);

void bltGetColorMultiplier(
	S_DRVBLT_ARGB16* psARGB16	// [out] ARGB Multiplier 
);

void bltSetColorOffset(
	S_DRVBLT_ARGB16 sARGB16	// [in] ARGB offset 
);

void bltGetColorOffset(
	S_DRVBLT_ARGB16* psARGB16	// [out] ARGB offset 
);

void bltSetSrcImage(
	S_DRVBLT_SRC_IMAGE sSrcImage	// [in] Source Image Setting
);

void bltGetSrcImage(
	S_DRVBLT_SRC_IMAGE *srcimg_p	// [out] Source Image Setting
);

void bltSetDestFrameBuf(
	S_DRVBLT_DEST_FB sFrameBuf	// [in] Frame Buffer Setting
);

void bltGetDestFrameBuf(
	S_DRVBLT_DEST_FB *frmbuff_p	// [out] Frame Buffer Setting
);

void bltSetARGBFillColor(
	S_DRVBLT_ARGB8 sARGB8	// [in] ARGB value for fill operation
);

void bltGetARGBFillColor(
	S_DRVBLT_ARGB8* psARGB8	// [out] ARGB value for fill operation
);

BOOL bltGetBusyStatus(void);

void bltSetFillAlpha(
BOOL bEnable
);

BOOL 
bltGetFillAlpha(void);

void bltSetTransformFlag(
	UINT32 u32TransFlag			// [in] A combination of the enum E_DRVBLT_TRANSFORM_FLAG
);

UINT32 
bltGetTransformFlag(void);

void bltSetPaletteEndian(
	E_DRVBLT_PALETTE_ORDER eEndian	// [in] Palette Endian Type
);

E_DRVBLT_PALETTE_ORDER bltGetPaletteEndian(void);

void bltSetColorPalette(
	UINT32 u32PaletteInx, 		// [in] Color Palette Start index
	UINT32 u32Num, 				// [in] Color Palette number to set
	S_DRVBLT_ARGB8* psARGB	// [in] pointer for Color palette from u32PaletteInx
);

void bltGetColorPalette(
	UINT32 u32PaletteInx, 		// [in] Color Palette Start index
	UINT32 u32Num, 				// [in] Color Palette number to set
	S_DRVBLT_ARGB8* psARGB	// [out] pointer for Color palette from u32PaletteInx
);

void bltSetFillOP(
	E_DRVBLT_FILLOP eOP		// [in] Enable/Disable FillOP
);

BOOL 
bltGetFillOP(void);

void bltSetFillStyle(
	E_DRVBLT_FILL_STYLE eStyle		// [in] Fill Style for Fill Operation
);

E_DRVBLT_FILL_STYLE 
bltGetFillStyle(void);

void bltSetRevealAlpha(
	E_DRVBLT_REVEAL_ALPHA eAlpha		// [in] need / no need un-multiply alpha on source image
);

BOOL 
bltGetRevealAlpha(void);

void bltTrigger(void);

void bltSetRGB565TransparentColor(
	UINT16 u16RGB565	// [in] RGB565 Transparent Color
);

UINT16 
bltGetRGB565TransparentColor(void);

void bltSetRGB565TransparentCtl(
BOOL bEnable
);

BOOL 
bltGetRGB565TransparentCtl(void);

void bltFlush (void);

// Flash Lite BLT I/F
typedef struct {
    S_DRVBLT_MATRIX          sMatrix;
    E_DRVBLT_BMPIXEL_FORMAT  eSrcFmt;
    E_DRVBLT_DISPLAY_FORMAT  eDestFmt;
    
    INT32                       i32Flag;
    S_DRVBLT_ARGB16          sColorMultiplier;
    S_DRVBLT_ARGB16          sColorOffset;
    E_DRVBLT_FILL_STYLE      eFilStyle;
} S_FI_BLITTRANSFORMATION;


typedef struct {
    S_DRVBLT_RECT            sRect;
    S_DRVBLT_ARGB8           sARGB8;
    UINT32                      u32FBAddr;
    INT32                       i32Stride;
    E_DRVBLT_DISPLAY_FORMAT  eDisplayFmt;
    INT32                       i32Blend;
} S_FI_FILLOP;

typedef struct {
	S_DRVBLT_ARGB8			*psARGB8;
	S_DRVBLT_SRC_IMAGE		sDrvSrcImage;
} S_FI_SRC_IMAGE;

typedef struct {
    S_FI_SRC_IMAGE       		sFISrcImage;
    S_DRVBLT_DEST_FB         sDestFB;
    S_FI_BLITTRANSFORMATION		*psTransform;
} S_FI_BLITOP;

void bltFIBlit (S_FI_BLITOP sBiltOp);
void bltFIFill (S_FI_FILLOP sFillOp);

// BLT MMU API
void bltmmuEnableMMU (BOOL enabled);	// Enable or Disable BLT MMU Virtual Address Translation.
BOOL bltmmuIsMMUEnabled (void);	

void bltmmuEnableMainTLB (BOOL enabled);	// Turn On/Off BLT MMU Main TLB (SRAM Buffers)
BOOL bltmmuIsMainTLBEnabled (void);

void bltmmuEnableMainTLBSrcCh (BOOL enabled);	// BLT MMU Main TLB Service Channels.
BOOL bltmmuIsMainTLBSrcChEnabled (void);

void bltmmuSetTTB (UINT32 ttb);	// BLT MMU Translation Table Base Address. 16KB aligned.
UINT32 bltmmuGetTTB (void);

UINT32 bltmmuGetPageFaultVA (void);	// BLT MMU Page Fault Virtual Address.

void bltmmuFlushMainTLB (void);	// Flush BLT MMU Main TLB Entries of Main TLB SRAM.
void bltmmuInvalidateMicroTLB (void);	// Invalidate BLT MMU Micro TLB Entries.
void bltmmuResumeMMU (void);	// Resume BLT MMU Transaction.

void bltmmuSetTTBEntry (int idx, UINT32 entryValue);	// BLT MMU Level-One Page Table.
UINT32 bltmmuGetTTBEntry (int idx);

UINT32 bltmmuGetCurVA (void);	// BLT MMU Current Virtual Address.
UINT32 bltmmuGetCurVPN (void);	// BLT MMU Current Virtual Page Number.
UINT32 bltmmuGetCurPA (void);	// BLT MMU Current Physical Address.
UINT32 bltmmuGetCurPPN (void);	// BLT MMU Current Physical Page Number.

void bltmmuSetup (UINT32 ttb, BOOL mainTLB, BOOL mainTLBSrcCh);	// Set up BLT MMU.
void bltmmuTeardown (void);	// Tear down BLT MMU.
void bltmmuFlushTLB(void);  // Flush BLT MMU TLB related registers.

#ifdef  __cplusplus
}
#endif

#endif	// __BLT_H__







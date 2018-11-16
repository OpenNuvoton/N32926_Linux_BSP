#ifndef _USER_DEFINE_H_
#define _USER_DEFINE_H_

// Constant value, don't change
#define MAX_REF_FRAME_NUM 32
#define MAX_DISP_FRAME_NUM 16
#define MAX_SPS_NUM 32
#define MAX_PPS_NUM 256	

#ifndef _DISABLE_REORDER_
#define DISPLAY_REORDER_CTRL		// Normal Release to define
#endif


#ifdef CONFIG_ENABLE_DECODER_ONLY
#define ENCODE_INSTANCE     (1)
#else
#define ENCODE_INSTANCE CONFIG_W55FA92_AVC_ENCODER_NUM
#endif

#ifdef CONFIG_ENABLE_ENCODER_ONLY
#define	MAXIUMUFRAMNUM	(1)
#else
#define	MAXIUMUFRAMNUM	(CONFIG_W55FA92_AVC_FRAME_NUM + 1)
#define DECODE_INSTANCE CONFIG_W55FA92_AVC_DECODER_NUM
#endif

#define DIV_4096(n)	((n+4095)/4096)*4096
#define DIV_1024(n)	((n+1023)/1024)*1024
#define WIDTH_MB    ((_max_frame_width + 15) /16)
#define HEIGHT_MB   ((_max_frame_height+ 15) /16)

// The Encoder Buffer sequence is as below which must match with buffer allocate in h264e_init(...) function
//------------------------------
// | Encode Bitstream Buffer 1  |
//------------------------------
// | Encode Reconstruct Buffer 1|
//------------------------------
// | Encode Reference Buffer 1  |
//------------------------------
// | Encode SysInfo Buffer 1    |
//------------------------------
// | Encode DMA * 2 Buffer 1    |
//------------------------------
// | Encode Bitstream Buffer 2  |
//------------------------------
// | Encode Reconstruct Buffer 2|
//------------------------------
// | Encode Reference Buffer 2  |
//------------------------------
// | Encode SysInfo Buffer 2    |
//------------------------------
// | Encode DMA * 2 Buffer 2    |
//------------------------------

//------------------------------
// | Decode Bitstream Buffer    |
//------------------------------
// | Decode Reconstruct Buffer  |
//------------------------------
// | Decode Reference Buffer 1  |
//------------------------------
// | Decode Reference Buffer 2  |
//------------------------------
// | Decode Reference Buffer n  |
//------------------------------
// | Decode Intra Pred Buffer   |
//------------------------------
// | Decode MB info Buffer      |
//------------------------------
// | Decode Output Buffer       |
//------------------------------


typedef struct
{
	uint32_t u32MaxWidth;           // Instance-N's Max Width
	uint32_t u32MaxHeight;	        // Instance-N's Max Height
	uint32_t u32BSsize;	            // Bistream Buffer Size for Instance-N	
	uint32_t u32DECBufsize;	        // Reconstruct + Reference buffer size
	uint32_t u32INTRAsize;	        // INTRA Buffer size	
	uint32_t u32OutputBufOffset;    // Output buffer offset to Instance-N's Buffer Start Address
	uint32_t u32OutputBufSize;      // Output buffer size for Instance-N
	uint32_t u32Offset;	            // Offset to Encoder or Decoder Buffer Start Address
	uint32_t u32RequiredSize;		// Total required buffer size for Instance-N (DEC_ONE_INSTANCE_SIZE)
}
DECODER_INFO;	

typedef struct
{
	uint32_t u32MaxWidth;           // Instance-N's Max Width
	uint32_t u32MaxHeight;	        // Instance-N's Max Height
	uint32_t u32RequiredSize;		// Total required buffer size for Instance-N
	uint32_t u32BSsize;	            // Bistream Buffer Size for Instance-N
	uint32_t u32Offset;	            // Offset to Encoder or Decoder Buffer Start Address
}
ENCODER_INFO;

#if 0
#define dbg_printk  printk
#else
#define dbg_printk(...) 
#endif

#endif

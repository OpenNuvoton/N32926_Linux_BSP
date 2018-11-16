#ifndef _FAVCAVCODEC_H_
#define _FAVCAVCODEC_H_

#include "port.h"

//-------------------------------------
// Remove below define for Linux : KC
//#define	LINUX_ENV	0


#define P_SLICE 0x00
#define B_SLICE 0x01
#define I_SLICE 0x02

// Constant value, don't change
/*
#define OUTPUT_FMT_CbYCrY	0
#define OUTPUT_FMT_RGB555	1
#define OUTPUT_FMT_RGB888	2
#define OUTPUT_FMT_RGB565	3
#define OUTPUT_FMT_YUV420	4
#define OUTPUT_FMT_YUV422	5
*/
#define OUTPUT_FMT_YUV420	0
#define OUTPUT_FMT_YUV422	1

//------------------------------

	// ioctl flag, should be consistent with driver definition
#define FAVC_IOCTL_DECODE_INIT    	        0x4170
#define FAVC_IOCTL_DECODE_FRAME   	        0x4172
#define FAVC_IOCTL_DECODE_420_INIT          0x4174
	
#define FAVC_IOCTL_ENCODE_INIT    	        0x4173
#define FAVC_IOCTL_ENCODE_FRAME   	        0x4175
#define FAVC_IOCTL_GET_SYSINFO  	        0x4177
#define FAVC_IOCTL_GET_SPSPPS     	        0x4179
#define FAVC_IOCTL_ENCODE_NVOP	            0x417B
#define FAVC_IOCTL_ENCODE_INIT_MP4          0x417D
#define FAVC_IOCTL_ENCODE_VUI               0x417F
#define FAVC_IOCTL_ENCODE_INIT_WRP          0x4180
#define FAVC_IOCTL_ENCODE_CROP              0x4181

#define FAVC_IOCTL_ENCODE_GET_BSINFO        0x4185

#define FAVC_IOCTL_DECODE_GET_BSSIZE        0x4186
#define FAVC_IOCTL_DECODE_GET_BSINFO        0x4187      // This is only for debug
#define FAVC_IOCTL_DECODE_GET_OUTPUT_INFO   0x4188
#define FAVC_IOCTL_ENCODE_GET_YUVSRCINFO    0x4189

#define FAVC_IOCTL_PERFORMANCE_RESET        0x41FE
#define FAVC_IOCTL_PERFORMANCE_REPORT       0x41FF
	
typedef struct avc_workbuf{
	__u32	addr;
	__u32	size;
	__u32	offset;		
} avc_workbuf_t;	
	
/*****************************************************************************
 * Decoder structures
 ****************************************************************************/
	typedef struct
	{
		Uint32 bEndOfDec;					// Used by driver
		Uint32 u32Width;					// Decoded bitstream width
		Uint32 u32Height;					// Decoded bitstream height
		Uint32 u32UsedBytes;				// Reported used bitstream byte in buffer. It is not accurate. The inaccuracy is less than 256 bytes
		Uint32 u32FrameNum;					// Decoded slice number
//		Uint8 * pu8FrameBaseAddr_phy;		// output frame buffer
//		Uint8 * pu8FrameBaseAddr_U_phy;		// frame buffer (U) if output format is yuv420
//		Uint8 * pu8FrameBaseAddr_V_phy;		// frame buffer (V) if output format is yuv420
		Uint32 isDisplayOut;				// 0 -> Buffer in reorder buffer, 1 -> available buffer, -1 -> last flush frame
		Uint32 isISlice;					// 1-> I Slice, 0 -> P slice
		Uint32 Reserved0;
	} FAVC_DEC_RESULT; 
 
	typedef struct
	{
		uint32_t u32API_version;
		uint32_t u32MaxWidth;				// Not used now
		uint32_t u32MaxHeight;				// Not used now
		uint32_t u32FrameBufferWidth;		// LCM buffer width (in pixel)
		uint32_t u32FrameBufferHeight;		// LCM buffer height (in pixel)
//		uint32_t u32CacheAlign;				// Not used now
		uint32_t u32Pkt_size;				// Current decoding bitstream length
		uint8_t *pu8Pkt_buf;				// Current decoding bitstream buffer address
		uint32_t pu8Display_addr[3];		// Buffer address for decoded data
		uint32_t got_picture;				// 0 -> Decoding has someting error. 1 -> decoding is OK in current bitstream
//		uint32_t u32BS_buf_sz;				// Reserved bitstream buffer size for decoding (Updated by driver in initialization)
		uint8_t * pu8BitStream_phy;			// physical address. buffer for bitstream
//		uint8_t * pu8BitStream;				// virtual address.
//		uint8_t * mb_info_phy;				// physical address for MB info (Updated by driver in initialization)
//		uint8_t * intra_pred_phy;			// Physical address for intra predication (Updated by driver in initialization)
		uint32_t u32OutputFmt;				// Decoded output format, 0-> Planar YUV420 format, 1-> Packet YUV422 foramt
		uint32_t crop_x;					// pixel unit: crop x start point at decoded-frame (not supported now)
		uint32_t crop_y;					// pixel unit: crop y start point at decoded-frame (not supported now)
		
		FAVC_DEC_RESULT tResult;			// Return decoding result by driver
		
	} FAVC_DEC_PARAM; 
 
 	/* API return values */
	typedef enum {
			RETCODE_OK = 0, 				// Operation succeed.
			RETCODE_ERR_MEMORY = 1,			// Operation failed (out of memory).
			RETCODE_ERR_API = 2,			// Operation failed (API version error).
			RETCODE_ERR_HEADER = 3,
			RETCODE_ERR_FILL_BUFFER = 4,
			RETCODE_ERR_FILE_OPEN = 5,
			RETCODE_HEADER_READY =6,
			RETCODE_BS_EMPTY =7,
			RETCODE_WAITING =8,
			RETCODE_DEC_OVERFLOW=9,
			RETCODE_HEADER_FINISH=10,
			RETCODE_DEC_TIMEOUT=11,
			RETCODE_PARSING_TIMEOUT=12,
			RETCODE_ERR_GENERAL=13,
			RETCODE_NOT_SUPPORT=14,
											// Below is added but not used
			RETCODE_FAILURE=15,
			RETCODE_FRAME_NOT_COMPLETE=16	
	} AVC_RET;
 

/*****************************************************************************
 * H.264 Encoder structures
 ****************************************************************************/

typedef struct {
    	unsigned int u32API_version;
		unsigned int u32BitRate;			// The encoded bitrate in bps.
  		unsigned int u32FrameWidth; 		// The width of encoded frame in pels.
  		unsigned int u32FrameHeight; 		// The height of encoded frame in pels.
  		unsigned int fFrameRate; 			// The base frame rate per second
  		unsigned int u32IPInterval; 		// The frame interval between I-frames.
  		unsigned int u32MaxQuant;			// The maximum quantization value. (max = 51)
  		unsigned int u32MinQuant;			// The minimum quantization value. (min=0)
  		unsigned int u32Quant; 				// The frame quantization value for initialization 
  		
        int ssp_output;   					// This variable tells the H.264 must be encoded out sps + pps before slice data.
                                			// ->  1 : force the encoder to output sps+pps  
                                			// ->  0 : force the encoder to output sps+pps on any Slice I frame
                                			// -> -1: (default) only output SPS+PPS on first IDR frame.
    	int intra; 							// This variable tells the H.264 must be encoded out an I-Slice type frame.
                                			// ->  1 : forces the encoder  to create a keyframe.
                                			// ->  0 :  forces the  encoder not to  create a keyframe.
                                			// -> -1: (default) let   the  encoder   decide  (based   on   contents  and u32IPInterval)  		

  /**********************************************************************************************/
  // The additional parameters for Region Of Interest feature
  /**********************************************************************************************/
  		int bROIEnable;  					// To enable the function of encoding rectangular region of interest(ROI) within captured frame. (0 : Disable, 1: Enable)
  		unsigned int u32ROIX;  				// The upper-left corner x coordinate of rectangular region of interest within captured frame if bROIEnable is enabled.
  		unsigned int u32ROIY;  				// The upper-left corner coordinate y of region of interest within captured frame if bROIEnable is enabled
  		unsigned int u32ROIWidth;  			// The width of user-defined rectangular region of interest within the captured frame in pixel units if bROIEnable is enabled
  		unsigned int u32ROIHeight; 			// The height of user-defined rectangular region of interest within the captured frame in pixel units if bROIEnable is enabled
  /**********************************************************************************************/
  // Buffer allocated by Application. 
  /**********************************************************************************************/  
  		unsigned char *pu8YFrameBaseAddr;  	// The base address for input Y frame buffer. (8-byte aligned)
  		unsigned char *pu8UVFrameBaseAddr; 	// The base address for input UV frame buffer in H.264 2D mode.(8-byte aligned)
  		unsigned char *pu8UFrameBaseAddr;  	// The base address for input U frame buffer.(8-byte aligned) (pu8UVFrameBaseAddr must be equal to pu8UFrameBaseAddr)
  		unsigned char *pu8VFrameBaseAddr;  	// The base address for input V frame buffer.(8-byte aligned)
  		unsigned char *pu8BitstreamAddr; 	// The bitstream buffer address while encoding one single frame provided by user (16-byte aligned)
  		
  		void *bitstream;					// Bitstream Buffer address for driver to write bitstream  		
  		
  /**********************************************************************************************/
  // Below filed is updated by driver to application.
  /**********************************************************************************************/   		
  		unsigned int  bitstream_size;		// Bitstream length for current frame (Updated by driver)
  		int keyframe;  						// This parameter is indicated the Slice type of frame. (Updated by Driver, 1-> I slice, 0-> P slice) 
  		int frame_cost;  					// frame_cout is updated by driver.  		
  /**********************************************************************************************/
  // Below filed is used by driver internally. Application doesn' need to care it.
  /**********************************************************************************************/  		
  		unsigned int no_frames; 			// The number of frames to be encoded (Not used now)
		unsigned int threshold_disable; 	// The transform coefficients threshold.
  		unsigned int chroma_threshold;  	// The chroma coefficients threshold (0 ~ 7).
  		unsigned int luma_threshold;    	// The luma coefficients threshold (0 ~ 7).
  		unsigned int beta_offset;       	// The beta offset for in-loop filter.
  		unsigned int alpha_offset;      	/// The alpha offset for in-loop filter.
  		unsigned int chroma_qp_offset;  	// The chroma qp offset (-12 to 12 inclusively).
  		unsigned int disable_ilf;       	// To disable in-loop filter or not
  		unsigned int watermark_enable;      // To enable watermark function or not (Don't enable it now)
  		unsigned int watermark_interval;    // To specify the watermark interval if watermark function is enabled 
  		unsigned int watermark_init_pattern;// To specify the initial watermark pattern if watermark function is enabled   		
  
  		unsigned char *pu8ReConstructFrame;	// The address of reconstruct frame buffer. (16-byte aligned)(size = u32FrameWidth * @ref u32FrameHeight * 3/2)
  		unsigned char *pu8ReferenceFrame;  	// The address of reference frame buffer.(16-byte aligned)(size = u32FrameWidth * @ref u32FrameHeight * 3/2)
  		unsigned char *pu8SysInfoBuffer; 	// The address of system info buffer.(4-byte aligned)(size = MBs_Count_Width+1)/2) *64 *2)
  		unsigned char *pu8DMABuffer_virt; 	// The virtual address of DMA buffer, which size is equal to ((4*19+2)*sizeof(unsigned int));(4-byte aligned)
  		unsigned char *pu8DMABuffer_phy;  	// The physical address of DMA buffer, which size is equal to ((4*19+2)*sizeof(unsigned int));(4-byte aligned)
    	int nvop_ioctl; 					// This parameter is valid only on FAVC_IOCTL_ENCODE_NVOP. (Not implemented)
   		unsigned int multi_slice;
    	unsigned int pic_height; 			// This parameter is used to keep the frame height for sps and pps on Multi Slice mode
    	unsigned int pic_width;	 			// This parameter is used to keep the frame width for sps and pps on Multi Slice mode
		unsigned int img_fmt;				// 0: 2D format, CbCr interleave, named H264_2D (VideoIn supported only)
		unsigned int control;				// 0 : Do NOT force one frame as one slice(default), 1 : Force one frame as one slice
} FAVC_ENC_PARAM;

typedef struct {
	unsigned int  video_format;
	unsigned char colour_description_present_flag;
	unsigned int  colour_primaries;
	unsigned int  transfer_characteristics;
	unsigned int  matrix_coefficients;
	unsigned char chroma_location_info_present_flag;
	unsigned int  chroma_sample_loc_type_top_field;
	unsigned int  chroma_sample_loc_type_bottom_field;
}FAVC_VUI_PARAM ;

typedef struct {
	unsigned int left_offset;
	unsigned int right_offset;
	unsigned int top_offset;
	unsigned int buttom_offset;
}FAVC_CROP_PARAM;

#endif

#ifndef _ENCODER_H_
#define _ENCODER_H_

#include "set.h"
//#include "favc_avcodec.h"
#include <mach/favc_avcodec.h>
#include "h264_enc_api.h"


typedef struct
{
	FAVC_ENC_PARAM mEncParam;
	unsigned int *pdma_buf;
	unsigned char *sys_info_buf;

	unsigned int mb_width;
	unsigned int mb_height;
	unsigned int mb_count;

	//unsigned int sys_info_size; /**< the hardware requirs memory size for storing related system info */
	unsigned int frame_luma_size;
	unsigned int iframe;
	unsigned int  slice_type;  
	unsigned char very_first_flag;
	unsigned int IPInterval;

#ifdef WATER_MARK_ENABLE
	unsigned int watermark_frame_number;
	unsigned int watermark_last_checksum;    
#endif
  
	// mainly used in start_slice() function
	unsigned int frame_num;
	signed int toppoc;
  
	// header related data structure
	// We use only one SPS and one PPS
	seq_parameter_set_rbsp_t sps;  
	pic_parameter_set_rbsp_t pps;
	//unsigned int slice_group_change_cycle;
	unsigned int bitstream_length;
	unsigned int dma_buffer_selector[SUPPORT_MAX_MSLIC_NUM];

	int      roi_enable;
	// ROI's (region of interest) upper-left corner coordinate(x,y) of captured frame.
	unsigned int roi_x;
	unsigned int roi_y;
	// The offset between upper-left corner of captured frame and the upper-left corner of point of ROI (region of interest) .
	unsigned int roi_offset_y;
	unsigned int roi_offset_uv;
	// The original frame mb width and height saved for ROI (region of interest).
	unsigned int roi_frame_mb_width;
	unsigned int roi_frame_mb_height;
	// ROI(region of interest) width and height in pixels units
	unsigned int roi_width;
	unsigned int roi_height;
	// ROI(region of interest) width and height in MacroBlock units
	unsigned int roi_mb_width;
	unsigned int roi_mb_height;	
	unsigned int multi_slice; // multislice mode
	unsigned int first_mb;     // keep the count of skip_mb
	unsigned int mp4_2d;    // keep the source format MP4 2D or H.264 2D, or Raster scan
										// 0: 2D format, CbCr interleave, named H264_2D
										// 1: 2D format, CbCr non-interleave, named MP4_2D
										// 2: 1D format, YUV420 planar, named RASTER_SCAN_420
										// 3: 1D format, YUV420 packed, named RASTER_SCAN_422
} h264_encoder;

struct slice_buf
{
    unsigned int    recon_phy_buffer;
    unsigned int    refer_phy_buffer;
    unsigned int    sysinfo_phy_buffer;
    unsigned int    dma_phy_buffer;

    int mslice_mb;	
    int mslice_last;		
    unsigned int    mslice_height;
    unsigned int    mslice_width;	
    
     /// ROI
    int mslice_ROI;  
    int mslice_ROIMB;		 
    int mslice_ROILast;	
    unsigned int mslice_ROIX;  
    unsigned int mslice_ROIY;  
    unsigned int mslice_ROIWidth;  
    unsigned int mslice_ROIHeight;     
    	
};
typedef struct slice_buf sbuf;

struct enc_private
{
    void          *enc_handle;
    int             frame_width;     // encode frame width
    int             frame_height;   // encode frame height
    
    // ROI
    int roi_enable;
    unsigned int roi_width;
    unsigned int roi_height;
    
    // multislice
    int             mslice_enable;
    int             mslice_num;  
    unsigned int   mslice_first_mb;	
    unsigned int   mslice_stride_yuv;	
    sbuf          slicebuf[SUPPORT_MAX_MSLIC_NUM];  
};

#define MaxQp 51
#define MinQp 0

//#define	EFAULT	1
/*
extern int favc_encoder_open(void);
extern int h264e_init(FAVC_ENC_PARAM * param, int dev);
extern int favc_encoder_release(void);
*/
#endif


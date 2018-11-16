#ifndef _DECODER_H_
#define _DECODER_H_

#include "port.h"
#include "AVCdec.h"
#include "h264_funct.h"
#include "user_define.h"
#include "common.h"

/*****************************************************************************
 * Structures
 ****************************************************************************/
typedef struct
{
	uint8_t  *L0_ref_idx_yaddr_phy;
	uint8_t L0_ref_is_long_term;
	int32_t L0_used_for_ref;
	int32_t L0_ref_lt_pic_num;
	int32_t L0_ref_pic_num;
	int32_t L0_ref_pic_poc;	
}
StorablePicture;

typedef struct
{
	uint8_t  *disp_idx_yaddr;
	uint8_t  *disp_idx_yaddr_phy;
	int32_t disp_pic_poc;	
	int32_t is_flushed;
}
DisplayPicture;

typedef struct
{
	uint32_t u32MaxWidth;
	uint32_t u32MaxHeight;
	uint32_t u32FrameBufferWidth;  	// LCD fb size
	uint32_t u32FrameBufferHeight; 	// LCD fb size		
	uint32_t video_width;  			// actual picture size
	uint32_t video_height; 			// actual picture size	
	uint32_t u32BS_buf_sz;
	uint32_t u32CacheAlign;
	uint32_t bEndOfDec;
	uint32_t u32UsedBytes;
#ifdef LINUX_ENV
//	FAVC_DMA_MALLOC_PTR pfnDmaMalloc;
//	FAVC_DMA_FREE_PTR pfnDmaFree;
//	FAVC_MALLOC_PTR pfnMalloc;
//	FAVC_FREE_PTR pfnFree;
#endif	
	uint32_t u32BS_hw_prev_offset;
	uint32_t u32BS_sw_offset;
	uint32_t u32BS_buf_sz_remain;
	uint32_t u32BS_sw_cur_fill_size;	//byte
	uint32_t bBS_end_of_data;
	uint32_t decoded_frame_num;	
	uint32_t prev_frame_num;
	volatile uint32_t current_status;
	
	uint32_t reinit_very_first_time;	  
	// the saved register for bitstream context switch
	uint32_t register_FRAMESIZE;
	uint32_t register_MBINFO_BASE;
	uint32_t register_INTRA_BASE;
	uint32_t register_OFFSET_REG0;
	uint32_t register_OFFSET_REG1;
	uint32_t register_RECON_LINEOFFS;
	uint32_t register_DP_LINEOFFS;
	uint32_t register_LCD_PARAM;
	uint32_t register_LCD_FRAME_LINE_OFF;
	uint32_t register_MISC;
	uint32_t register_RECON_BASE;

	char  forbidden_zero_bit;
	char  nal_ref_idc;
	char  nal_unit_type;
			
	uint8_t  *rec_frame_yaddr;
	StorablePicture *prev_rec_frame;
	int32_t max_long_term_pic_idx;

	uint8_t  output_fmt;
	uint8_t * output_base_phy;

	//#if (OUTPUT_FMT == OUTPUT_FMT_YUV420)
	uint8_t * output_base_u_phy;
	uint8_t * output_base_v_phy;
	//#endif	

	uint8_t *pu8BS_start_virt;
	uint8_t *pu8BS_start_phy;
	
	void * pvSemaphore;
	SEM_WAIT_PTR pfnSemWait;
	SEM_SIGNAL_PTR pfnSemSignal;	
	
	// internal data structure
	uint8_t active_sps_idx;
	uint8_t active_pps_idx;
	struct_sps           *sps[MAX_SPS_NUM];
	struct_pps           *pps[MAX_PPS_NUM];
	struct_slice_header  *ssh;
	decode_slice_data    *dsd;
	decode_parameter     *decode_para;
	StorablePicture *ref_pic_data[MAX_REF_FRAME_NUM+1];	
	
	// mmco saved parameter	
	uint8_t mmco_ref_is_long_term[MAX_REF_FRAME_NUM+1];
	int32_t mmco_used_for_ref[MAX_REF_FRAME_NUM+1];
	int32_t mmco_ref_lt_pic_num[MAX_REF_FRAME_NUM+1];
	int32_t mmco_ref_pic_num[MAX_REF_FRAME_NUM+1];	
	int32_t last_has_mmco_5;

	// POC calculation
	#ifdef DISPLAY_REORDER_CTRL
	DisplayPicture *disp_pic_data[MAX_DISP_FRAME_NUM];
	uint8_t dpb_size; 				//frame number
	uint8_t dpb_used_size;	
	uint8_t disp_pos;
	int8_t is_dpb_full;
	#endif
	
	int32_t toppoc;      			//poc for this top field
	int32_t bottompoc;   			//poc of bottom field of frame
	int32_t framepoc;    			//poc of this frame	
	int32_t disposable_flag;
	
  	// for POC mode 0:
	int32_t PrevPicOrderCntMsb;
	uint32_t PrevPicOrderCntLsb;
	int32_t PicOrderCntMsb;

 	// for POC mode 1:
	uint32_t AbsFrameNum;
	int32_t ExpectedPicOrderCnt, PicOrderCntCycleCnt, FrameNumInPicOrderCntCycle;
	uint32_t FrameNumOffset;
	int32_t ExpectedDeltaPerPicOrderCntCycle;
	int32_t PreviousPOC, ThisPOC;
	int32_t PreviousFrameNumOffset;	
	int8_t dev_num;

	uint32_t crop_x;				// pixel unit: crop x start point at decoded-frame
	uint32_t crop_y;				// pixel unit: crop y start point at decoded-frame
	
	uint32_t SliceDoneMBNum;
}
DECODER;


int32_t decoder_create(FAVC_DEC_PARAM * ptParam, void ** pptDecHandle, unsigned char ndev);
void decoder_destroy(void * ptDecHandle);
void decoder_fill_bs(void * ptDecHandle);
unsigned int decide_time_slot(void * ptDecHandle);
AVC_RET decoder_decode(void * ptDecHandle, FAVC_DEC_RESULT * ptResult, int dev);
int32_t decoder_sync(void * ptDecHandle);
void decoder_reinit(void * ptDecHandle);
void decoder_reset(void * ptDecHandle);
void decoder_dummy_write(void * ptDecHandle);

int isISlice(void * ptDecHandle);
int cropcom_check (DECODER * dec);

void refresh_int_sts(DECODER *dec);

#endif

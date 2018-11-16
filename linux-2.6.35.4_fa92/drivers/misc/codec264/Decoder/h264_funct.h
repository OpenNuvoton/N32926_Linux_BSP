#ifndef H264_FUNCT_H
#define H264_FUNCT_H

#include "AVCdec.h"


#define AVC_Baseline 66
#define AVC_MainProfile 77
#define Support_Profile AVC_Baseline
#define Support_Level  31//30       // 30 : max 720x576@20, 31 : max 1280x720@30fps
#define Support_Entropy 0

//FREXT Profile IDC definitions
#define FREXT_HP        100      //!< YUV 4:2:0/8 "High"
#define FREXT_Hi10P     110      //!< YUV 4:2:0/10 "High 10"
#define FREXT_Hi422     122      //!< YUV 4:2:2/10 "High 4:2:2"
#define FREXT_Hi444     144      //!< YUV 4:4:4/12 "High 4:4:4"


// STRUCTURES
typedef struct decode_slice_data {
    uint8_t    num_ref_idx_l0_active; // num_ref_idx_l0_active_minus1;
    uint8_t    num_ref_idx_l1_active; // num_ref_idx_l1_active_minus1;
    uint8_t    slice_type;
    uint8_t    curr_slice_nr;        
    uint8_t    frame_num;
    uint8_t    qp;                             // currSlice qp
} decode_slice_data ;


typedef struct struct_sps {
	uint8_t    profile_idc;
	uint8_t    constrained_set;
	uint8_t    level_idc;
	uint8_t    seq_parameter_set_id;
	uint8_t    pic_order_cnt_type;	
	uint8_t    log2_max_frame_num_minus4; //0~12
	uint8_t    log2_max_pic_order_cnt_lsb_minus4; //0~12
	uint8_t    delta_pic_order_always_zero_flag;
	int32_t    offset_for_non_ref_pic; //-2^31 ~ 2^31-1
	int32_t    offset_for_top_to_bottom_field; //-2^31 ~ 2^31-1
	int32_t    offset_for_ref_frame[256]; //-2^31 ~ 2^31-1
	uint8_t    num_ref_frames_in_pic_order_cnt_cycle; //0~255
	  
	uint8_t    num_ref_frames;
	uint8_t    gaps_in_frame_num_value_allowed_flag;
	uint8_t    pic_width_in_mbs;
	uint8_t    pic_height_in_map_units;
	uint8_t    frame_mbs_only_flag;
	uint8_t    mb_aff_flag;
	uint8_t    direct_8x8_inference_flag;
	uint8_t    frame_cropping_flag;
	uint16_t   frame_crop_left_offset;
	uint16_t   frame_crop_right_offset;
	uint16_t   frame_crop_top_offset;
	uint16_t   frame_crop_bottom_offset;	
	uint8_t    vui_parameters_present_flag;
	uint8_t	   bitstream_restriction_flag;
	uint32_t   max_dec_frame_buffering;		
} struct_sps ;

typedef struct struct_pps {
	  uint8_t    pic_parameter_set_id;
	  uint8_t    seq_parameter_set_id;
	  uint8_t    entropy_coding_mode_flag;
	  uint8_t    pic_order_present_flag;
	  uint8_t    num_slice_groups_minus1;
	  uint8_t    num_ref_idx_l0_active_minus1;
	  uint8_t    num_ref_idx_l1_active_minus1;
	  uint8_t    weighted_pred_flag;
	  uint8_t    weighted_bipred_idc;
	  uint8_t    pic_init_qp_minus26;
	  uint8_t    pic_init_qs_minus26;
	  uint8_t    chroma_qp_index_offset;
	  uint8_t    deblocking_filter_control_present_flag;
	  uint8_t    constrained_intra_pred_flag;
	  uint8_t    redundant_pic_cnt_present_flag;
} struct_pps ;

typedef struct struct_slice_header {
// slice header
	uint16_t   first_mb_in_slice;
	uint16_t   frame_num;
	uint8_t    slice_type;
	uint8_t    pic_parameter_set_id;
	uint8_t    field_pic_flag;
	uint8_t    bottom_field_flag;

	uint16_t   idr_pic_id;
	uint16_t   pic_order_cnt_lsb;
	int32_t    delta_pic_order_cnt_bottom;
	int32_t    delta_pic_order_cnt_0;//[0]
	int32_t    delta_pic_order_cnt_1;//[1]

	uint8_t    direct_spatial_mv_pred_flag;
	uint8_t    num_ref_idx_active_override_flag;
	uint8_t    num_ref_idx_l0_active_minus1;
	uint8_t    num_ref_idx_l1_active_minus1;

	uint8_t    cabac_init_idc;                 // model_number
	uint8_t    slice_qp_delta;
	uint8_t    disable_deblocking_filter_idc;  // LFDisableIdc
	uint8_t    slice_alpha_c0_offset_div2;     // LFAlphaC0Offset
	uint8_t    slice_beta_offset_div2;         // LFBetaOffset

	uint8_t    ref_pic_list_reordering_flag_l0;
	uint8_t    ref_pic_list_reordering_flag_l1;
	uint8_t    reordering_of_pic_nums_idc;

	uint16_t   abs_diff_pic_num_minus1;
	uint16_t   long_term_pic_num;

	uint8_t    no_output_of_prior_pics_flag;
	uint8_t    long_term_reference_flag;
	uint8_t    adaptive_ref_pic_marking_mode_flag;
	uint16_t   memory_management_control_operation;
	uint16_t   difference_of_pic_nums_minus1;
	uint16_t   long_term_frame_idx;
	uint16_t   max_long_term_frame_idx_plus1;
} struct_slice_header ;


typedef struct decode_parameter {
    uint32_t    MB_INFO_BASE;
    uint32_t    MB_INFO_OFFSET;
    uint32_t    DISP_OUT_OFFSET;
    uint32_t    INTRA_PRED_BASE;
    uint32_t    INTRA_PRED_OFFSET;
    uint32_t    ILFTOP_Y_LINEOFFS;
    uint32_t    ILFTOP_UV_LINEOFFS;
    uint32_t    RECON_OUT_LINEOFFS;
    uint32_t    DP_Y_OUT_LINEOFFS;
    uint32_t    DP_UV_OUT_LINEOFFS;
} decode_parameter ;


// FUNCTIONS

uint32_t read_u(uint8_t );
uint32_t read_ue(void );
uint32_t read_se(void );


AVC_RET read_nal_pps(void * ptDecHandle);
void rbsp_trailing_bit(void );
AVC_RET read_nal_sps(void * ptDecHandle);
AVC_RET read_nal_slice(void * ptDecHandle);
void decode_slice(void * ptDecHandle);
AVC_RET init_parameters(void * ptDecHandle, int dev);
int read_nal_sei(void * ptDecHandle);
void read_nal_delimiter(void);
void pred_weight_table(void );
void init_lists(void * ptDecHandle);
int ref_pic_list_reordering(void * ptDecHandle);
int reorder_short_or_long_term(void * ptDecHandle, int num_ref_idx_lX_active, int picNumLX, int *refIdxLX, int is_long_term);
void dec_ref_pic_marking(void * ptDecHandle);
void write_frm_addr(void * ptDecHandle);
void write_frm_idx(void * ptDecHandle);
void cropping_frame(void * ptDecHandle);
#ifdef DISPLAY_REORDER_CTRL
int32_t getDpbSize(void * ptDecHandle);
int32_t store_picture_in_dpb(void * ptDecHandle);
int32_t flush_one_pic_from_dpb(void * ptDecHandle);
#endif

#ifdef LINUX_ENV
void qsort(void *const pbase, int total_elems, int size, int (*cmp)(const void*, const void*));
#endif

#endif

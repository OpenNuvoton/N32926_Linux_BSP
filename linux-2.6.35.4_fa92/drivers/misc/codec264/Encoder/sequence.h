#ifndef _SEQUENCE_H_
#define _SEQUENCE_H_
#include "encoder.h"

#define NALU_TYPE_SLICE    1
#define NALU_TYPE_DPA      2
#define NALU_TYPE_DPB      3
#define NALU_TYPE_DPC      4
#define NALU_TYPE_IDR      5
#define NALU_TYPE_SEI      6
#define NALU_TYPE_SPS      7
#define NALU_TYPE_PPS      8
#define NALU_TYPE_AUD      9
#define NALU_TYPE_EOSEQ    10
#define NALU_TYPE_EOSTREAM 11
#define NALU_TYPE_FILL     12

#define NALU_PRIORITY_HIGHEST     3
#define NALU_PRIORITY_HIGH        2
#define NALU_PRIRITY_LOW          1
#define NALU_PRIORITY_DISPOSABLE  0

typedef struct {
 Boolean      aspect_ratio_info_present_flag;
 unsigned int aspect_ratio_idc;
 unsigned int sar_width;
 unsigned int sar_height;
 Boolean      overscan_info_present_flag;
 Boolean      overscan_appropriate_flag;
 Boolean      video_signal_type_present_flag;
 unsigned int video_format;
 Boolean      video_full_range_flag;
 Boolean      colour_description_present_flag;
 unsigned int colour_primaries;
 unsigned int transfer_characteristics;
 unsigned int matrix_coefficients;
 Boolean      chroma_location_info_present_flag;
 unsigned int chroma_sample_loc_type_top_field;
 unsigned int chroma_sample_loc_type_bottom_field;
 Boolean      timing_info_present_flag;
 unsigned int num_units_in_tick;
 unsigned int time_scale;
 Boolean      fixed_frame_rate_flag;  
 Boolean      nal_hrd_parameters_present_flag;  
 unsigned int nal_cpb_cnt_minus1;
 unsigned int nal_bit_rate_scale;
 unsigned int nal_cpb_size_scale;
 unsigned int nal_bit_rate_value_minus1;
 unsigned int nal_cpb_size_value_minus1;
 unsigned int nal_vbr_cbr_flag;
 unsigned int nal_initial_cpb_removal_delay_length_minus1;
 unsigned int nal_cpb_removal_delay_length_minus1;
 unsigned int nal_dpb_output_delay_length_minus1;
 unsigned int nal_time_offset_length;
 
 Boolean      vcl_hrd_parameters_present_flag;  
 unsigned int vcl_cpb_cnt_minus1;
 unsigned int vcl_bit_rate_scale;
 unsigned int vcl_cpb_size_scale;
 unsigned int vcl_bit_rate_value_minus1;
 unsigned int vcl_cpb_size_value_minus1;
 unsigned int vcl_vbr_cbr_flag;
 unsigned int vcl_initial_cpb_removal_delay_length_minus1;
 unsigned int vcl_cpb_removal_delay_length_minus1;
 unsigned int vcl_dpb_output_delay_length_minus1;
 unsigned int vcl_time_offset_length;

 Boolean      low_delay_hrd_flag;
 Boolean      pic_struct_present_flag;
 Boolean      bitstream_restriction_flag;
 Boolean      motion_vectors_over_pic_boundaries_flag;
 unsigned int max_bytes_per_pic_denom;
 unsigned int max_bits_per_mb_denom;
 unsigned int log2_max_mv_length_horizontal;
 unsigned int log2_max_mv_length_vertical;
 unsigned int num_reorder_frames;
 unsigned int max_dec_frame_buffering;
 Boolean      rgb_input_flag;
 unsigned int yuv_format;
}vui_param ;

typedef struct {
	unsigned int left_offset;
	unsigned int right_offset;
	unsigned int top_offset;
	unsigned int buttom_offset;
}crop_param;

void start_sequence(h264_encoder *pEnc);
void terminate_sequence(h264_encoder *pEnc);

void init_vui_parameters(h264_encoder *pEnc, vui_param *pVUI);
void init_sps_nalu(h264_encoder *pEnc);
void init_pps_nalu(h264_encoder *pEnc);
void generate_hrd_parameters(h264_encoder *pEnc);
void generate_vui_parameters(h264_encoder *pEnc);
void generate_sps_nalu(h264_encoder *pEnc);
void generate_pps_nalu(h264_encoder *pEnc);

//lichun
void init_sps_pps(h264_encoder *pEnc);
void generate_sps_pps(h264_encoder *pEnc);
void init_crop_parameters(h264_encoder *pEnc, crop_param *pcrop);
void init_vui_parameters(h264_encoder *pEnc, vui_param *pvui);

#endif


#ifdef LINUX_ENV

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/hdreg.h>	/* HDIO_GETGEO */
#include <linux/fs.h>
#include <linux/blkdev.h>
#include <linux/pci.h>
#include <linux/spinlock.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <linux/version.h>
#include <linux/interrupt.h>
#include <linux/time.h>
#include <linux/mm.h>
#include <linux/slab.h>

#include <mach/w55fa92_reg.h>
#include <mach/favc_avcodec.h>

#else

#include <stdio.h>
#include <string.h>
#include "w55fa92_reg.h"
#include "wbtypes.h"
#include "wblib.h"

#endif

#include "port.h"
#include "h264dec.h"
#include "register.h"
#include "sequence.h"
#include "common.h"

#include "user_define.h"

#ifdef CONFIG_ENCODE_VUI
#define     ENC_VUI_IN_SPS     1
#else
#define     ENC_VUI_IN_SPS     0
#endif

void start_sequence(h264_encoder *pEnc)
{

  init_sps_nalu(pEnc);
  init_pps_nalu(pEnc);

  generate_sps_nalu(pEnc); // to write the SPS NALU (sequence parameter set)
  
  generate_pps_nalu(pEnc); // to write the PPS NALU (picture parameter set)
  
}

void init_sps_pps(h264_encoder *pEnc)
{
  init_sps_nalu(pEnc);
#if ENC_VUI_IN_SPS
  init_vui_parameters(pEnc, NULL);
#endif  
  init_pps_nalu(pEnc);
}

void generate_sps_pps(h264_encoder *pEnc)
{
  generate_sps_nalu(pEnc); // to write the SPS NALU (sequence parameter set)
  generate_pps_nalu(pEnc); // to write the PPS NALU (picture parameter set)
}

void terminate_sequence(h264_encoder *pEnc)
{
}

void init_crop_parameters(h264_encoder *pEnc, crop_param *pcrop)
{
  pEnc->sps.frame_cropping_rect_top_offset    = pcrop->top_offset;
  pEnc->sps.frame_cropping_rect_bottom_offset = pcrop->buttom_offset;
  pEnc->sps.frame_cropping_rect_left_offset   = pcrop->left_offset;
  pEnc->sps.frame_cropping_rect_right_offset  = pcrop->right_offset;
  return;
}

void init_vui_parameters(h264_encoder *pEnc, vui_param *pvui)
{
  unsigned int         SchedSelIdx;
  hrd_parameters_t     *nal_hrd = &(pEnc->sps.vui_seq_parameters.nal_hrd_parameters);
  hrd_parameters_t     *vcl_hrd = &(pEnc->sps.vui_seq_parameters.vcl_hrd_parameters);
  vui_seq_parameters_t *vui     = &(pEnc->sps.vui_seq_parameters);
  
  
  vui->aspect_ratio_info_present_flag      = FALSE;
  vui->aspect_ratio_idc                    = 255;//Extended_SAR;
  vui->sar_width                           = pEnc->mEncParam.pic_width;
  vui->sar_height                          = pEnc->mEncParam.pic_height;
  vui->overscan_info_present_flag          = FALSE;
  vui->overscan_appropriate_flag           = TRUE;
  vui->video_signal_type_present_flag      = FALSE;
  //vui->video_format                        = (unsigned int) pvui->video_format;
  vui->video_format                        = 0;
  vui->video_full_range_flag               = TRUE;
  
  //vui->colour_description_present_flag     = TRUE;
  //vui->colour_primaries                    = 2;
  //vui->transfer_characteristics            = 2;
  //vui->matrix_coefficients                 = 2;
  vui->chroma_location_info_present_flag   = FALSE;
  //vui->chroma_sample_loc_type_top_field    = 0;
  //vui->chroma_sample_loc_type_bottom_field = 0;
  
  vui->timing_info_present_flag            = FALSE;
  vui->time_scale                          = 27000000;
  //vui->num_units_in_tick                   = vui->time_scale/pEnc->mEncParam.fFrameRate;
  //vui->fixed_frame_rate_flag               = TRUE;  
  vui->fixed_frame_rate_flag               = FALSE;  
  
  // NAL HRD parameters
  vui->nal_hrd_parameters_present_flag             = FALSE;  
  nal_hrd->cpb_cnt_minus1                          = 0;
  nal_hrd->bit_rate_scale                          = 0;//(unsigned int) pvui->nal_bit_rate_scale;
  nal_hrd->cpb_size_scale                          = 0;//(unsigned int) pvui->nal_cpb_size_scale;
  for ( SchedSelIdx = 0; SchedSelIdx <= nal_hrd->cpb_cnt_minus1; SchedSelIdx++ ) {
    nal_hrd->bit_rate_value_minus1[SchedSelIdx]    = 0;//(unsigned int) pvui->nal_bit_rate_value_minus1;
    nal_hrd->cpb_size_value_minus1[SchedSelIdx]    = 0;//(unsigned int) pvui->nal_cpb_size_value_minus1;
    nal_hrd->cbr_flag[SchedSelIdx]                 = 0;//(unsigned int) pvui->nal_vbr_cbr_flag;
  }
  nal_hrd->initial_cpb_removal_delay_length_minus1 = 23;
  nal_hrd->cpb_removal_delay_length_minus1         = 23;
  nal_hrd->dpb_output_delay_length_minus1          = 23;
  nal_hrd->time_offset_length                      = 0;

  // VCL HRD parameters
  vui->vcl_hrd_parameters_present_flag             = FALSE;  
  vcl_hrd->cpb_cnt_minus1                          = 0;
  vcl_hrd->bit_rate_scale                          = 0;//log2bin(pEnc->mEncParam.u32BitRate) - 6;
  vcl_hrd->cpb_size_scale                          = 0;//(unsigned int) pvui->vcl_cpb_size_scale;
  for ( SchedSelIdx = 0; SchedSelIdx <= vcl_hrd->cpb_cnt_minus1; SchedSelIdx++ ) {
    vcl_hrd->bit_rate_value_minus1[SchedSelIdx]    = 0;//(unsigned int) pvui->vcl_bit_rate_value_minus1;
    vcl_hrd->cpb_size_value_minus1[SchedSelIdx]    = 0;//(unsigned int) pvui->vcl_cpb_size_value_minus1;
    vcl_hrd->cbr_flag[SchedSelIdx]                 = 1;
  }
  vcl_hrd->initial_cpb_removal_delay_length_minus1 = 23;
  vcl_hrd->cpb_removal_delay_length_minus1         = 23;
  vcl_hrd->dpb_output_delay_length_minus1          = 23;
  vcl_hrd->time_offset_length                      = 0;

  vui->low_delay_hrd_flag                      = FALSE;
  vui->pic_struct_present_flag                 = FALSE;
  vui->bitstream_restriction_flag              = TRUE;
  vui->motion_vectors_over_pic_boundaries_flag = FALSE;
  vui->max_bytes_per_pic_denom                 = FALSE;
  vui->max_bits_per_mb_denom                   = FALSE;
  vui->log2_max_mv_length_horizontal           = 7;
  vui->log2_max_mv_length_vertical             = 7;
  vui->num_reorder_frames                      = 1;
  vui->max_dec_frame_buffering                 = 1;
  
  // special case to signal the RGB format
  if(pvui != NULL && pvui->rgb_input_flag && pvui->yuv_format==3)  {
    #if 0
    printk("VUI: writing Sequence Parameter VUI to signal RGB format\n");
    #endif
    vui->aspect_ratio_info_present_flag = FALSE;
    vui->overscan_info_present_flag = FALSE;
    vui->video_signal_type_present_flag = TRUE;
    vui->video_format = 2;
    vui->video_full_range_flag = TRUE;
    vui->colour_description_present_flag = TRUE;
    vui->colour_primaries = 2;
    vui->transfer_characteristics = 2;
    vui->matrix_coefficients = 0;
    vui->chroma_location_info_present_flag = FALSE;
    vui->timing_info_present_flag = FALSE;
    vui->nal_hrd_parameters_present_flag = FALSE;
    vui->vcl_hrd_parameters_present_flag = FALSE;
    vui->pic_struct_present_flag = FALSE;
    vui->bitstream_restriction_flag = FALSE;
  } 
  if ( vui->fixed_frame_rate_flag == TRUE ) {
    if (vui->pic_struct_present_flag == TRUE) {
        vui->num_units_in_tick = vui->time_scale/(pEnc->mEncParam.fFrameRate*2);
    } else {
        printk("not support this VUI framerate \n");
    }
    vui->num_units_in_tick = vui->time_scale/pEnc->mEncParam.fFrameRate;
  } else {
    vui->num_units_in_tick = vui->time_scale/pEnc->mEncParam.fFrameRate;
  }
  pEnc->sps.vui_parameters_present_flag = TRUE; // set it to 1
  return;
}


void init_sps_nalu(h264_encoder *pEnc)
{
  unsigned int log2_max_frame_num_minus4;
  unsigned int log2_max_pic_order_cnt_lsb_minus4;
  
  pEnc->sps.profile_idc = 66; // to specify baseline
  
  pEnc->sps.constrained_set0_flag = 0;                            // u(1)
  pEnc->sps.constrained_set1_flag = 0;                            // u(1)
  pEnc->sps.constrained_set2_flag = 0;                            // u(1)
  pEnc->sps.constrained_set3_flag = 0;                            // u(1)
  
  #ifdef LARGER_LEVEL
    pEnc->sps.level_idc = 40;     // for larger picture 1920x1088    // u(8)
  #else
    pEnc->sps.level_idc = 31;     // we fix it to level 3.0 = 30    // u(8)
  #endif  
  
  pEnc->sps.seq_parameter_set_id = 0;                             // ue(v)
  
    // according to X264's code
    //sps->i_log2_max_frame_num = 4;  // at least 4
    //while( (1 << sps->i_log2_max_frame_num) <= param->i_keyint_max )
      //{
        //sps->i_log2_max_frame_num++;
      //}
    //sps->i_log2_max_frame_num++;    // just in case
    /*
    log2_max_frame_num_minus4 = 4;
    while( (1 << log2_max_frame_num_minus4) <= KEYINT_MAX )
      {
        log2_max_frame_num_minus4++;
      }
    log2_max_frame_num_minus4++;
    pEnc->sps.log2_max_frame_num_minus4 = log2_max_frame_num_minus4 - 4;
    */
    log2_max_frame_num_minus4 = 0;
    pEnc->sps.log2_max_frame_num_minus4 = log2_max_frame_num_minus4;

  
  pEnc->sps.pic_order_cnt_type = 0; // we set it to zero, for POC mode 0
  
  // according to X264's code
    //if( sps->i_poc_type == 0 )
      //{
        //sps->i_log2_max_poc_lsb = sps->i_log2_max_frame_num + 1;    /* max poc = 2*frame_num */
      //}
    log2_max_pic_order_cnt_lsb_minus4 = 4+1; //log2_max_frame_num + 1;
    pEnc->sps.log2_max_pic_order_cnt_lsb_minus4 = log2_max_pic_order_cnt_lsb_minus4 - 4;
  
  pEnc->sps.num_ref_frames = 1; // currently, hardware supports only one reference frame
  
  pEnc->sps.gaps_in_frame_num_value_allowed_flag = 0; // it is always 0
  
  pEnc->sps.pic_width_in_mbs_minus1 = ((pEnc->mEncParam.pic_width+ 15)/PIXEL_Y)-1;
  pEnc->sps.pic_height_in_map_units_minus1 = ((pEnc->mEncParam.pic_height+ 15)/PIXEL_Y) -1;
  #if 0
  printk("SPS mb width %d pic_width_in_mbs_minus1 %d pic_height_in_map_units_minus1 %d\n", 
  	pEnc->mb_width,
  	pEnc->sps.pic_width_in_mbs_minus1, 
  	pEnc->sps.pic_height_in_map_units_minus1);
  #endif
  pEnc->sps.frame_mbs_only_flag = 1; // for baseline, this bit is always 1
  
  pEnc->sps.direct_8x8_inference_flag = 1; // since it is baseline and there is no b-slice so we set direct_8x8_inference_flag to its default value 1
  
  pEnc->sps.frame_cropping_flag = 0; // we set it to zero currently

  //pEnc->sps.vui_parameters_present_flag = 0; // set it to 0
  return;
}

void init_pps_nalu(h264_encoder *pEnc)
{
  // this parameter is always set to zero
  pEnc->pps.seq_parameter_set_id = 0;
  // this parameter is always set to zero
  pEnc->pps.pic_parameter_set_id = 0;
  
  // since we use only baseline, the 'entropy_coding_mode_flag' is always set to 0
  pEnc->pps.entropy_coding_mode_flag = 0;
  
  //since baseline only supports frame coding, so we set this parameter to 0
  pEnc->pps.pic_order_present_flag = 0;
  
  //this parameter was specified in the encoder configuration file with a parameter called 'num_slice_groups_minus1' , we set it to zero
  pEnc->pps.num_slice_groups_minus1 = 0;
  
  // the frame_mbs_only_flag is alwyas set to 1 in SPS since we support only baseline.
  // and use only one reference frame right now, so we set the 'num_ref_idx_l0_active_minus1'
  // and 'num_ref_idx_l0_active_minus1' to 0
  pEnc->pps.num_ref_idx_l0_active_minus1 = 0;  
  pEnc->pps.num_ref_idx_l1_active_minus1 = 0;
  
  // weighted prediction only supports in Main Profile, so we set the
  // 'weighted_pred_flag' and 'weighted_bipred_idc' to 0.
  // Also you can note that the 'weighted_pred_flag' and 'weighted_bipred_idc' was specified
  // through the encoder configuration file with the parameters called 'WeightedPrediction'
  // and 'WeightedBiprediction' respectively.
  pEnc->pps.weighted_pred_flag = 0;
  pEnc->pps.weighted_bipred_idc = 0;

  // hard coded to zero, QP lives in the slice header  
  pEnc->pps.pic_init_qp_minus26 = 0;
  pEnc->pps.pic_init_qs_minus26 = 0;
  
  // the 'chroma_qp_index_offset' was specified by the encoder parameter 'ChromaQPOffset' in encoder 
  // configuration file. In h.264 spec., the 'chroma_qp_index_offset' shall be in the range -12 to +12
  // inclusively. But the encoder parameter 'ChromaQPOffset' was in the range -51 to +51.
  // Anyway, we set this parameter to 0.
  pEnc->pps.chroma_qp_index_offset = 0;
  
  // the 'deblocking_filter_control_present_flag' was specified by the encoder parameter
  // 'LoopFilterParametersFlag' in encoder configuration file.
  pEnc->pps.deblocking_filter_control_present_flag = 0;
  
  // the 'constrained_intra_pred_flag' was specified by the encoder parameter
  // 'UseConstrainedIntraPred' in encoder configuration file.
  // we always set it to 0 for our hardware
  pEnc->pps.constrained_intra_pred_flag = 0;
  
  // always set this parameter to 0
  pEnc->pps.redundant_pic_cnt_present_flag = 0;

  return;
}

void generate_hrd_parameters(h264_encoder *pEnc)
{
  unsigned int SchedSelIdx = 0;
  hrd_parameters_t *hrd = &(pEnc->sps.vui_seq_parameters.nal_hrd_parameters);

  UE_RBSP(hrd->cpb_cnt_minus1);
  U_RBSP(hrd->bit_rate_scale, 4);
  U_RBSP(hrd->cpb_size_scale, 4);

  for( SchedSelIdx = 0; SchedSelIdx <= (hrd->cpb_cnt_minus1); SchedSelIdx++ )  {
    UE_RBSP(hrd->bit_rate_value_minus1[SchedSelIdx]);
    UE_RBSP(hrd->cpb_size_value_minus1[SchedSelIdx]);
    U_RBSP(hrd->cbr_flag[SchedSelIdx], 1);
  }

  U_RBSP(hrd->initial_cpb_removal_delay_length_minus1, 5);
  U_RBSP(hrd->cpb_removal_delay_length_minus1, 5);
  U_RBSP(hrd->dpb_output_delay_length_minus1, 5);
  U_RBSP(hrd->time_offset_length, 5);

  return;
}

void generate_vui_parameters(h264_encoder *pEnc)
{ 
  vui_seq_parameters_t *vui = &(pEnc->sps.vui_seq_parameters);
  
  U_RBSP(vui->aspect_ratio_info_present_flag,1);
  if (vui->aspect_ratio_info_present_flag) {        
    U_RBSP(vui->aspect_ratio_idc, 8);
    if (vui->aspect_ratio_idc == 255) {
      U_RBSP(vui->sar_width, 16);
      U_RBSP(vui->sar_height,16);
    }
  }  

  U_RBSP(vui->overscan_info_present_flag,1);
  if (vui->overscan_info_present_flag){
    U_RBSP(vui->overscan_appropriate_flag,1);
  } 

  U_RBSP(vui->video_signal_type_present_flag, 1);
  if (vui->video_signal_type_present_flag) {
    U_RBSP(vui->video_format,3);
    U_RBSP(vui->video_full_range_flag, 1);
    U_RBSP(vui->colour_description_present_flag,1);
    if (vui->colour_description_present_flag){
      U_RBSP(vui->colour_primaries, 8);
      U_RBSP(vui->transfer_characteristics, 8);
      U_RBSP(vui->matrix_coefficients, 8);
    }
  }

  U_RBSP(vui->chroma_location_info_present_flag, 1);
  if (vui->chroma_location_info_present_flag){
    UE_RBSP(vui->chroma_sample_loc_type_top_field);
    UE_RBSP(vui->chroma_sample_loc_type_bottom_field);
  }

  U_RBSP(vui->timing_info_present_flag, 1);
  // timing parameters
  if (vui->timing_info_present_flag) {
    U_RBSP(vui->num_units_in_tick, 32);
    U_RBSP(vui->time_scale,32);
    U_RBSP(vui->fixed_frame_rate_flag, 1);
  }
  // end of timing parameters
  // nal_hrd_parameters_present_flag
  U_RBSP(vui->nal_hrd_parameters_present_flag, 1);
  if ( vui->nal_hrd_parameters_present_flag ) {
    generate_hrd_parameters(pEnc);
  }
  // vcl_hrd_parameters_present_flag
  U_RBSP(vui->vcl_hrd_parameters_present_flag, 1);
  if ( vui->vcl_hrd_parameters_present_flag )  {
    generate_hrd_parameters(pEnc);
  }
  if ( vui->nal_hrd_parameters_present_flag || vui->vcl_hrd_parameters_present_flag ) {
    U_RBSP(vui->low_delay_hrd_flag, 1 );
  }
  U_RBSP(vui->pic_struct_present_flag, 1);

  U_RBSP(vui->bitstream_restriction_flag, 1);
  if (vui->bitstream_restriction_flag)  {
    U_RBSP(vui->motion_vectors_over_pic_boundaries_flag, 1);
    UE_RBSP(vui->max_bytes_per_pic_denom );
    UE_RBSP(vui->max_bits_per_mb_denom );
    UE_RBSP(vui->log2_max_mv_length_horizontal );
    UE_RBSP(vui->log2_max_mv_length_vertical );
    UE_RBSP(vui->num_reorder_frames );
    UE_RBSP(vui->max_dec_frame_buffering );
  }
  return;
}

void generate_sps_nalu(h264_encoder *pEnc)
{
  
  // byte stream NAL unit syntax with emulation_prevention_three_byte disabled
  U(0, 8); // 8  bits , additional zero byte 'leading_zero_8bits' if it is parameter sets and first slice in picture
  U(1,24); // 24 bits , start code prefix
  
  // SPS NALU header with emulation_prevention_three_byte disabled
  U(((0<<7)|(NALU_PRIORITY_HIGHEST<<5)|NALU_TYPE_SPS),8); // 8 bits , (forbidden_zero_bit | nal_ref_idc | nal_unit_type)
  
  // SPS RBSP parameters
  U_RBSP(pEnc->sps.profile_idc,8); // u(8),"SPS: profile_idc"
  
  U_RBSP(pEnc->sps.constrained_set0_flag,1);  // u(1),"SPS: constrained_set0_flag"
  U_RBSP(pEnc->sps.constrained_set1_flag,1);  // u(1),"SPS: constrained_set1_flag"
  U_RBSP(pEnc->sps.constrained_set2_flag,1);  // u(1),"SPS: constrained_set2_flag"
  
  // the following two definitions here are different from the h.264 spec.
  // In h.264 spec., there is no such field called 'constrained_set3_flag' but JM has.
  // Also, the h.264 specification denote the reserved_zero as 'reserved_zero_5bits' instead of 'reserved_zero_4bits'
  U_RBSP(pEnc->sps.constrained_set3_flag,1);  // u(1),"SPS: constrained_set3_flag"
  U_RBSP(0,4);  // u(4),"SPS: reserved_zero_4bits"
  
  U_RBSP(pEnc->sps.level_idc,8);  // u(8),"SPS: level_idc" , we fix it to level 3.0 = 30
  
  // Currently, the encoder does not support multiple parameter sets,
  // primarily because the config file does not support it.  Hence the
  // pic_parameter_set_id and the seq_parameter_set_id are always zero.
  UE_RBSP(pEnc->sps.seq_parameter_set_id);  // ue(v),"SPS: seq_parameter_set_id"
  
  UE_RBSP(pEnc->sps.log2_max_frame_num_minus4);  // ue(v),"SPS: log2_max_frame_num_minus4"
  UE_RBSP(pEnc->sps.pic_order_cnt_type);         // ue(v),"SPS: pic_order_cnt_type"
  
  /*
  if (sps->pic_order_cnt_type == 0)
    len+=ue_v ("SPS: log2_max_pic_order_cnt_lsb_minus4",     sps->log2_max_pic_order_cnt_lsb_minus4,         partition);
  else if (sps->pic_order_cnt_type == 1)
  {
    len+=u_1  ("SPS: delta_pic_order_always_zero_flag",        sps->delta_pic_order_always_zero_flag,          partition);
    len+=se_v ("SPS: offset_for_non_ref_pic",                  sps->offset_for_non_ref_pic,                    partition);
    len+=se_v ("SPS: offset_for_top_to_bottom_field",          sps->offset_for_top_to_bottom_field,            partition);
    len+=ue_v ("SPS: num_ref_frames_in_pic_order_cnt_cycle",   sps->num_ref_frames_in_pic_order_cnt_cycle,     partition);
    for (i=0; i<sps->num_ref_frames_in_pic_order_cnt_cycle; i++)
      len+=se_v ("SPS: offset_for_ref_frame",                  sps->offset_for_ref_frame[i],                      partition);
  } */
  UE_RBSP(pEnc->sps.log2_max_pic_order_cnt_lsb_minus4);  // ue(v),"SPS: log2_max_pic_order_cnt_lsb_minus4"

  UE_RBSP(pEnc->sps.num_ref_frames);  // ue(v),"SPS: num_ref_frames" , currently, hardware supports only one reference frame
  
  U_RBSP(pEnc->sps.gaps_in_frame_num_value_allowed_flag,1);  // u(1),"SPS: gaps_in_frame_num_value_allowed_flag" , it is always 0

  UE_RBSP(pEnc->sps.pic_width_in_mbs_minus1);  // ue(v),"SPS: pic_width_in_mbs_minus1"
  
  UE_RBSP(pEnc->sps.pic_height_in_map_units_minus1); // ue(v),"SPS: pic_height_in_map_units_minus1"
  
  U_RBSP(pEnc->sps.frame_mbs_only_flag,1);  // u(1),"SPS: frame_mbs_only_flag" , for baseline, this bit is always 1
  
  U_RBSP(pEnc->sps.direct_8x8_inference_flag,1);  // u(1),"SPS: direct_8x8_inference_flag" , since it is baseline and there is no b-slice so we set direct_8x8_inference_flag to its default value 1
  
  U_RBSP(pEnc->sps.frame_cropping_flag,1);  // u(1),"SPS: frame_cropping_flag"

  if (pEnc->sps.frame_cropping_flag) {
    UE_RBSP(pEnc->sps.frame_cropping_rect_left_offset);  // ue(v) ("SPS: frame_cropping_rect_left_offset"
    UE_RBSP(pEnc->sps.frame_cropping_rect_right_offset); // ue(v) ("SPS: frame_cropping_rect_right_offset"
    UE_RBSP(pEnc->sps.frame_cropping_rect_top_offset);   // ue(v) ("SPS: frame_cropping_rect_top_offset"
    UE_RBSP(pEnc->sps.frame_cropping_rect_bottom_offset);// ue(v) ("SPS: frame_cropping_rect_bottom_offset"
  } 
  
  U_RBSP(pEnc->sps.vui_parameters_present_flag,1);  // u(1),"SPS: vui_parameters_present_flag" , set to 0
  
  if ( pEnc->sps.vui_parameters_present_flag )
  	generate_vui_parameters(pEnc);
   
  RBSP_TRAILING_BITS();

  return;
}

void generate_pps_nalu(h264_encoder *pEnc)
{
  //const unsigned int baseAddress =pEnc->mEncParam.u32BaseAddr;
  
  // byte stream NAL unit syntax with emulation_prevention_three_byte disabled
  U(0, 8); // 8  bits , additional zero byte 'leading_zero_8bits' if it is parameter sets and first slice in picture
  U(1,24); // 24 bits , start code prefix
  
  // PPS NALU header with emulation_prevention_three_byte disabled
  U(((0<<7)|(NALU_PRIORITY_HIGHEST<<5)|NALU_TYPE_PPS),8); // 8 bits , (forbidden_zero_bit | nal_ref_idc | nal_unit_type) 
  
  // PPS RBSP parameters
  
  // Currently, the encoder does not support multiple parameter sets,
  // primarily because the config file does not support it.  Hence the
  // pic_parameter_set_id and the seq_parameter_set_id are always zero.
  UE_RBSP(pEnc->pps.seq_parameter_set_id);  // ue(v),"PPS: pic_parameter_set_id"  , this parameter is always set to zero
  UE_RBSP(pEnc->pps.pic_parameter_set_id);  // ue(v),"PPS: seq_parameter_set_id"  , this parameter is always set to zero
  
  U_RBSP(pEnc->pps.entropy_coding_mode_flag,1); // u(1),"PPS: entropy_coding_mode_flag" , since we use only baseline, the 'entropy_coding_mode_flag' is always set to 0
  
  U_RBSP(pEnc->pps.pic_order_present_flag,1); // u(1),"PPS: pic_order_present_flag" , since baseline only supports frame coding, so we set this parameter to 0
  
  UE_RBSP(pEnc->pps.num_slice_groups_minus1);  // ue(v),"PPS: num_slice_groups_minus1" , this parameter was specified in the encoder configuration file with a parameter called 'num_slice_groups_minus1' , we set it to zero
  
  
  // the frame_mbs_only_flag is alwyas set to 1 in SPS since we support only baseline.
  // and use only one reference frame right now, so we set the 'num_ref_idx_l0_active_minus1'
  // and 'num_ref_idx_l0_active_minus1' to 0
  UE_RBSP(pEnc->pps.num_ref_idx_l0_active_minus1);  // ue(v),"PPS: num_ref_idx_l0_active_minus1"
  UE_RBSP(pEnc->pps.num_ref_idx_l1_active_minus1);  // ue(v),"PPS: num_ref_idx_l1_active_minus1"

  // weighted prediction only supports in Main Profile, so we set the
  // 'weighted_pred_flag' and 'weighted_bipred_idc' to 0.
  // Also you can note that the 'weighted_pred_flag' and 'weighted_bipred_idc' was specified
  // through the encoder configuration file with the parameters called 'WeightedPrediction'
  // and 'WeightedBiprediction' respectively.
  U_RBSP(pEnc->pps.weighted_pred_flag,1); // u(1),"PPS: weighted_pred_flag"
  U_RBSP(pEnc->pps.weighted_bipred_idc,2); // u(2),"PPS: weighted_bipred_idc"
  
  SE_RBSP(pEnc->pps.pic_init_qp_minus26);  // se(v),"PPS: pic_init_qp_minus26" , this parameter was hard coded to zero, QP lives in the slice header
  SE_RBSP(pEnc->pps.pic_init_qs_minus26);  // se(v),"PPS: pic_init_qs_minus26" , this parameter was hard coded to zero, QP lives in the slice header
  
  // the 'chroma_qp_index_offset' was specified by the encoder parameter 'ChromaQPOffset' in encoder 
  // configuration file. In h.264 spec., the 'chroma_qp_index_offset' shall be in the range -12 to +12
  // inclusively. But the encoder parameter 'ChromaQPOffset' was in the range -51 to +51.
  // Anyway, we set this parameter to 0.
  SE_RBSP(pEnc->pps.chroma_qp_index_offset);  // se(v),"PPS: chroma_qp_index_offset"
  
  // the 'deblocking_filter_control_present_flag' was specified by the encoder parameter
  // 'LoopFilterParametersFlag' in encoder configuration file.
  U_RBSP(pEnc->pps.deblocking_filter_control_present_flag,1); // u(1),"PPS: deblocking_filter_control_present_flag"
  
  
  // the 'constrained_intra_pred_flag' was specified by the encoder parameter
  // 'UseConstrainedIntraPred' in encoder configuration file.
  // we always set it to 0 for our hardware
  U_RBSP(pEnc->pps.constrained_intra_pred_flag,1); // u(1),"PPS: constrained_intra_pred_flag"
  
  U_RBSP(pEnc->pps.redundant_pic_cnt_present_flag,1); // u(1),"PPS: redundant_pic_cnt_present_flag" , always set this parameter to 0

  RBSP_TRAILING_BITS();    

  return;
}



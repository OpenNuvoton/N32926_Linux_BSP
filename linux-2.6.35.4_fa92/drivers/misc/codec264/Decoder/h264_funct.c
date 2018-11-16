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


#include "h264dec.h"
#include "port.h"
#include "h264_funct.h"
#include "decoder.h"

#include "user_define.h"

extern unsigned int   dec_ref_phy_buffer;
extern unsigned int    dec_bs_phy_buffer;

extern DECODER_INFO DECODER_INST[4];

#define MIN(a,b)  (((a)<(b)) ? (a) : (b))//LIZG 28/10/2002
#define MAX(a,b)  (((a)<(b)) ? (b) : (a))//LIZG 28/10/2002


#define AHBBASE_LOCAL H264D_BA

#define 	DBG_DPB_MSG		0

int32_t getDpbSize(void * ptDecHandle);

//int slice_count=0;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
uint32_t read_se(void)
{
    uint32_t result ;	
    uint32_t temp_data ;	

    temp_data = read_ue();
    if((temp_data & 0x00000001) == 0x00000001)
        result = temp_data / 2 + 1 ;
    else
        result = -(temp_data / 2);
        
    return result;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// size : bit unit
uint32_t read_u(uint8_t size)
{

    uint32_t result ;

    result = inp32(REG_264_ADDR_BSM);    
    
    outp32(REG_264_ADDR_BSM_CTL, m_sys_parser_ctl(SYS_BSM_SFT) | size);
    result = result >> (32-size); 
    return result;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
uint32_t read_ue(void)
{
	
    uint32_t   result;
    uint32_t   result_tmp1;
    uint32_t   result_tmp2;
    uint32_t   length;
  
    outp32(REG_264_ADDR_BSM_CTL, m_sys_parser_ctl(SYS_BSM_UVLD));    


    length = inp32(REG_264_ADDR_UVLDEN);   
    result_tmp1 = inp32(REG_264_ADDR_BSM);

    
    if(length == 0x00000000){
    	  result = 0;
    } else {
        result_tmp2 = result_tmp1 >>(32-length);
        result = result_tmp2 + (0x0000FFFF >> (16-length)); 
    	outp32(REG_264_ADDR_BSM_CTL, m_sys_parser_ctl(SYS_BSM_SFT) | length);
    }
    
    return result;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void rbsp_trailing_bit(void)
{

    outp32(REG_264_ADDR_BSM_CTL, m_sys_parser_ctl(SYS_BSM_RBSPTB));    
    
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void decode_slice(void * ptDecHandle)
{
    DECODER * dec = (DECODER *)ptDecHandle;

    struct_sps *sps = dec->sps[dec->active_sps_idx];
    struct_pps *pps = dec->pps[dec->active_pps_idx];
    struct_slice_header *ssh = dec->ssh;
    decode_slice_data *dsd = dec->dsd;

    if(ssh->first_mb_in_slice == 0){
        dsd->curr_slice_nr = 0;
        dec->decoded_frame_num++;
    } else {
        dsd->curr_slice_nr = dsd->curr_slice_nr+1;
    }

    //write_slice_data(dec);
    outp32(REG_264_SREG0,  SLICE_INFO0(sps->pic_width_in_mbs, sps->pic_height_in_map_units, sps->frame_mbs_only_flag,
									sps->mb_aff_flag, sps->direct_8x8_inference_flag, pps->entropy_coding_mode_flag,
									dsd->num_ref_idx_l0_active, dsd->num_ref_idx_l1_active, ssh->field_pic_flag, ssh->bottom_field_flag)); 									  
   
    
    outp32(REG_264_SREG1, SLICE_INFO1(dsd->slice_type, dsd->frame_num, pps->weighted_pred_flag,
   									pps->weighted_bipred_idc, dsd->qp,
   									pps->chroma_qp_index_offset, pps->constrained_intra_pred_flag)); 	
   									
    outp32(REG_264_SREG2, SLICE_INFO2(dsd->curr_slice_nr, ssh->cabac_init_idc, ssh->disable_deblocking_filter_idc,
									ssh->slice_alpha_c0_offset_div2, ssh->slice_beta_offset_div2));
    
    // start new slice decoding
  
    __asm__("nop");__asm__("nop");__asm__("nop");
  	
  
    outp32(REG_264_ADDR_DECODE_CTL0, (uint32_t)START_DEC_SLICE);       
}

void ReadHRDParameters(void)
{
    unsigned int read_data, i;
    unsigned int tmp0, tmp1;

    read_data = read_ue();  // cpb_cnt_minus1
    read_u(4);      // bit_rate_scale
    read_u(4);      // cpb_size_scale
    for (i = 0; i<= read_data; i++) {
        #if 1
        tmp0 = read_ue();      // bit_rate_value_minus1
        tmp1 = read_ue();      // cpb_size_value_minus1
        //printk("bit_rate_value_minus1 = %x_%x\n", tmp0);
        //printk("cpb_size_value_minus1 = %x_%x\n", tmp1);
        #else
        tmp0 = read_u(16);
        tmp1 = read_u(7);
        printk("bit_rate_value_minus1 = %x_%x\n", tmp0, tmp1);
        tmp0 = read_u(16);
        tmp1 = read_u(15);
        printk("cpb_size_value_minus1 = %x_%x\n", tmp0, tmp1);
        #endif
        read_u(1);      // cbr_flag
    }
    read_u(5);      // initial_cpb_removal_delay_length_minus1
    read_u(5);      // cpb_removal_delay_length_minus1
    read_u(5);      // dpb_output_delay_length_minus1
    read_u(5);      // time_offset_length
}



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void write_frm_idx(void * ptDecHandle)
{
    DECODER * dec = (DECODER *)ptDecHandle;
    struct_sps *sps = dec->sps[dec->active_sps_idx];
    struct_slice_header *ssh = dec->ssh;
    StorablePicture **ref_pic_data = dec->ref_pic_data;
    StorablePicture *tmp_pic;
    int max_ref_frame;

    max_ref_frame = sps->num_ref_frames;

    tmp_pic = *(ref_pic_data + max_ref_frame);
    tmp_pic->L0_used_for_ref = (dec->nal_ref_idc != 0);   // Reference picture : A picture with nal_ref_idc not equal to 0.
    tmp_pic->L0_ref_is_long_term = 0;    	     
    tmp_pic->L0_ref_pic_num = ssh->frame_num;
    tmp_pic->L0_ref_lt_pic_num = 0xFFFF;
    tmp_pic->L0_ref_pic_poc = dec->framepoc; // obtained in decode_poc();
    dec->rec_frame_yaddr = tmp_pic->L0_ref_idx_yaddr_phy;
    dec->prev_rec_frame = tmp_pic; 
}

void ReadVUI(struct_sps *sps)
{
    uint32_t read_data, read_data_save;
    //skip the feature
    read_data = read_u(1); //sps->vui_seq_parameters.aspect_ratio_info_present_flag
    if(read_data) {
        read_data = read_u(8); //sps->vui_seq_parameters.aspect_ratio_idc
        if (read_data==255) {
            read_u(16); //sps->vui_seq_parameters.sar_width
            read_u(16); //sps->vui_seq_parameters.sar_height
        }
    }

    read_data = read_u(1); //sps->vui_seq_parameters.overscan_info_present_flag
    if (read_data) {
        read_u(1); //sps->vui_seq_parameters.overscan_appropriate_flag
    }

    read_data = read_u(1); //sps->vui_seq_parameters.video_signal_type_present_flag
    if (read_data) {
        read_u(3); //sps->vui_seq_parameters.video_format
        read_u(1); //sps->vui_seq_parameters.video_full_range_flag
        read_data = read_u(1); //sps->vui_seq_parameters.colour_description_present_flag
        if(read_data) {
            read_u(8); //sps->vui_seq_parameters.colour_primaries
            read_u(8); //sps->vui_seq_parameters.transfer_characteristics
            read_u(8); //sps->vui_seq_parameters.matrix_coefficients
        }
    }
    read_data = read_u(1); //sps->vui_seq_parameters.chroma_location_info_present_flag
    if(read_data) {
        read_ue(); //sps->vui_seq_parameters.chroma_sample_loc_type_top_field
        read_ue(); //sps->vui_seq_parameters.chroma_sample_loc_type_bottom_field
    }
    read_data = read_u(1); //sps->vui_seq_parameters.timing_info_present_flag
    if (read_data) {
        read_u(16);
        read_u(16); //sps->vui_seq_parameters.num_units_in_tick
        read_u(16);      
        read_u(16); //sps->vui_seq_parameters.time_scale
        read_u(1); //sps->vui_seq_parameters.fixed_frame_rate_flag
    }
    read_data_save = read_u(1); //sps->vui_seq_parameters.nal_hrd_parameters_present_flag
    if (read_data_save) {
          ReadHRDParameters();    // Tuba 20120522: add hdr parser
        //ReadHRDParameters(p, &(sps->vui_seq_parameters.nal_hrd_parameters));
    }
    read_data = read_u(1); //sps->vui_seq_parameters.vcl_hrd_parameters_present_flag
    if (read_data) {
         ReadHRDParameters();    // Tuba 20120522: add hdr parser
        //ReadHRDParameters(p, &(sps->vui_seq_parameters.vcl_hrd_parameters));
    }
    if (read_data_save || read_data) {
        read_u(1); //sps->vui_seq_parameters.low_delay_hrd_flag
    }
    read_u(1); //sps->vui_seq_parameters.pic_struct_present_flag
    read_data = read_u(1); //sps->vui_seq_parameters.bitstream_restriction_flag
    sps->bitstream_restriction_flag = read_data;

    if (read_data) {
        read_u(1); //sps->vui_seq_parameters.motion_vectors_over_pic_boundaries_flag
        read_ue(); //sps->vui_seq_parameters.max_bytes_per_pic_denom
        read_ue(); //sps->vui_seq_parameters.max_bits_per_mb_denom
        read_ue(); //sps->vui_seq_parameters.log2_max_mv_length_horizontal
        read_ue(); //sps->vui_seq_parameters.log2_max_mv_length_vertical
        read_ue(); //sps->vui_seq_parameters.num_reorder_frames
        sps->max_dec_frame_buffering = read_ue(); //sps->vui_seq_parameters.max_dec_frame_buffering
    }
	return;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
AVC_RET read_nal_sps(void * ptDecHandle)
{
    DECODER * dec = (DECODER *)ptDecHandle;
    uint32_t read_data, sps_id, profile_idc, constrained_set, level_idc;
    int i;
    struct_sps **sps_list = dec->sps;
    struct_sps *sps;

    profile_idc = read_u(8) & 0x000000FF; // sps.profile_idc = read_data[7:0]; profile_idc
/*	
    if(sps->profile_idc > Support_Profile) {
        printf(msg);
        return RETCODE_ERR_HEADER;
    }
*/	
    constrained_set = read_u(8) & 0x000000FF; // sps.constrained_set = read_data[7:0]; constrained_set
    level_idc = read_u(8) & 0x000000FF; // sps.level_idc = read_data[7:0]; level_idc
/*    if(sps->level_idc > Support_Level) {
        printf(msg);
        return RETCODE_ERR_HEADER;
    }
*/	
    sps_id = read_ue() & 0x0000001F; // sps.seq_parameter_set_id = read_data[4:0]; sps_id
    dec->active_sps_idx = sps_id;
	
    if(*(sps_list + sps_id)==NULL) {
        if((*(sps_list + sps_id) = (struct_sps *)nv_malloc(sizeof(struct_sps),32))==NULL)        
            return RETCODE_ERR_MEMORY;
    }

    sps = *(sps_list + sps_id);
    sps->profile_idc = profile_idc;
    sps->constrained_set = constrained_set;
    sps->level_idc = level_idc;
    sps->seq_parameter_set_id = sps_id;
	
    //Add for bitstream encoded by ffdshow H.264
    if((sps->profile_idc==FREXT_HP   ) ||
       (sps->profile_idc==FREXT_Hi10P) ||
       (sps->profile_idc==FREXT_Hi422) ||
       (sps->profile_idc==FREXT_Hi444)) {
        read_data = read_ue(); // sps->chroma_format_idc
    
        // Residue Color Transform
        if(read_data == 3)
            read_u(1); //img->residue_transform_flag

        read_ue(); //sps->bit_depth_luma_minus8
        read_ue(); //sps->bit_depth_chroma_minus8
        read_u(1); //img->lossless_qpprime_flag
        read_u(1); //sps->seq_scaling_matrix_present_flag   
        //if(sps->seq_scaling_matrix_present_flag)
        //{}	 	
    }

    sps->log2_max_frame_num_minus4 = read_ue();// & 0x0000001F; // sps.log2_max_frame_num_minus4 = read_data[4:0];
    sps->pic_order_cnt_type = read_ue() & 0x00000003; // sps.pic_order_cnt_type = read_data[1:0]; pic_order_cnt_type
    
    // initial value
    sps->log2_max_pic_order_cnt_lsb_minus4 = 0;
    sps->delta_pic_order_always_zero_flag  = 0;
    
    if(sps->pic_order_cnt_type == 0) {
        sps->log2_max_pic_order_cnt_lsb_minus4 = read_ue() & 0x0000001F;
    } else if(sps->pic_order_cnt_type == 1) {
        sps->delta_pic_order_always_zero_flag = read_u(1) & 0x00000001; // delta_pic_order_always_zero_flag
        sps->offset_for_non_ref_pic			= read_se();
        sps->offset_for_top_to_bottom_field		= read_se();
        sps->num_ref_frames_in_pic_order_cnt_cycle	= read_ue();
        for(i=0; i<sps->num_ref_frames_in_pic_order_cnt_cycle; i++)
            sps->offset_for_ref_frame[i] = read_se();
    }

    sps->num_ref_frames = read_ue() & 0x0000001F; // sps.num_ref_frames = read_data[4:0]; num_ref_frames
    sps->gaps_in_frame_num_value_allowed_flag = read_u(1) & 0x00000001; // sps.gaps_in_frame_num_value_allowed_flag = read_data[0]; gaps_in_frame_num_value_allowed_flag

    // Support maximum frame size 2048x2048
    sps->pic_width_in_mbs = (read_ue() & 0x000000FF)+1; // sps.pic_width_in_mbs_minus1 = read_data[7:0]; pic_width_in_mbs_minus1
    sps->pic_height_in_map_units = (read_ue() & 0x000000FF)+1; // sps.pic_height_in_map_units_minus1 = read_data[7:0]; pic_height_in_map_units_minus1
    sps->frame_mbs_only_flag = read_u(1) & 0x00000001; // sps.frame_mbs_only_flag = read_data[0];
    
    // initial value
    sps->mb_aff_flag = 0;
   
    if((sps->frame_mbs_only_flag & 0x00000001) == 0) {
        sps->mb_aff_flag = read_u(1) & 0x00000001;  // sps.mb_aff_flag = read_data[0];
    }

    // direct_8x8_inference_flag
    sps->direct_8x8_inference_flag = read_u(1) & 0x00000001; // sps.direct_8x8_inference_flag = read_data[0];

    // frame_cropping_flag
    sps->frame_cropping_flag = read_u(1) & 0x00000001; // sps.frame_cropping_flag = read_data[0];
    
    if(sps->frame_cropping_flag) {
        sps->frame_crop_left_offset = read_ue() & 0x000000FF;
        sps->frame_crop_right_offset = read_ue() & 0x000000FF;
        sps->frame_crop_top_offset = read_ue() & 0x000000FF;
        sps->frame_crop_bottom_offset = read_ue() & 0x000000FF;
    }

    sps->vui_parameters_present_flag = read_u(1) & 0x00000001; // sps.vui_parameters_present_flag = read_data[0];
    
    if(sps->vui_parameters_present_flag)
        ReadVUI(sps);

    rbsp_trailing_bit();

    if(sps->frame_cropping_flag)
    {
        if((sps->frame_crop_left_offset!=0) ||(sps->frame_crop_right_offset!=0) ||(sps->frame_crop_top_offset!=0) ||(sps->frame_crop_bottom_offset!=0))
        {
        	Console_Printf("Frame cropping Not support\n");
        	Console_Printf("crop_left_offset = %d, crop_right_offset=%d, crop_top_offset=%d, crop_bottom_offset=%d\n",
        					sps->frame_crop_left_offset,sps->frame_crop_right_offset,sps->frame_crop_top_offset,sps->frame_crop_bottom_offset );
            return RETCODE_NOT_SUPPORT;
        }
    }

    return RETCODE_OK;
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
AVC_RET read_nal_pps(void * ptDecHandle)
{
    DECODER * dec = (DECODER *)ptDecHandle;
    uint32_t read_data;
    uint8_t pic_parameter_set_id;
    struct_pps **pps_list = dec->pps;
    struct_pps *pps;
    //char msg[100] ;  	

    pic_parameter_set_id = read_ue() & 0x000000FF;  // pic_parameter_set_id
    dec->active_pps_idx = pic_parameter_set_id;
	
    if(*(pps_list + pic_parameter_set_id)==NULL) {
        if((*(pps_list + pic_parameter_set_id) = (struct_pps *)nv_malloc(sizeof(struct_pps),32))==NULL)        
            return RETCODE_ERR_MEMORY;
    }

    pps = *(pps_list + pic_parameter_set_id);
    pps->pic_parameter_set_id = pic_parameter_set_id;
    pps->seq_parameter_set_id = read_ue() & 0x0000001F;  // seq_parameter_set_id

    read_data = read_u(2);
    pps->entropy_coding_mode_flag = (read_data & 0x00000002) >> 1;  // entropy_coding_mode_flag/pic_order_present_flag
    pps->pic_order_present_flag = read_data & 0x00000001; // pps.pic_order_present_flag = read_data[0];

    if(pps->entropy_coding_mode_flag != Support_Entropy){
        //printf(msg);
        //printk("Unsupported PPS:entropy_coding_mode\n");	
        return RETCODE_ERR_HEADER;
    }	

    pps->num_slice_groups_minus1 = read_ue() & 0x0000000F;  // num_slice_groups_minus1=0
    
    //kc
#if 1   
	//Console_Printf("num_slice_groups_minus1 = %d\n",pps->num_slice_groups_minus1); 
    if (pps->num_slice_groups_minus1 > 0)
    {
    	Console_Printf("num_slice_groups_minus1 = %d\n",pps->num_slice_groups_minus1);
    	return RETCODE_ERR_HEADER;
    }
#endif    	

    pps->num_ref_idx_l0_active_minus1 = read_ue() & 0x0000001F;  // num_ref_idx_l0_active_minus1

    pps->num_ref_idx_l1_active_minus1 = read_ue() & 0x0000001F; // num_ref_idx_l1_active_minus1

    read_data = read_u(3);
    pps->weighted_pred_flag = (read_data & 0x00000004) >> 2;  // pps.weighted_pred_flag = read_data[2];
    pps->weighted_bipred_idc = read_data & 0x00000003; // pps.weighted_bipred_idc = read_data[1:0]; // weighted_pred_flag/weighted_bipred_idc

    if(pps->weighted_pred_flag | pps->weighted_bipred_idc) 	{
        //printf(msg);
        //printk("Unsupported PPS:weighted_pred\n");
        return RETCODE_ERR_HEADER;
    }
	
    pps->pic_init_qp_minus26 = read_se() & 0x0000003F;  // pic_init_qp_minus26

    pps->pic_init_qs_minus26 = read_se() & 0x0000003F;  // pic_init_qs_minus26

    pps->chroma_qp_index_offset = read_se() & 0x0000001F;  // chroma_qp_index_offset

    read_data = read_u(3);
    pps->deblocking_filter_control_present_flag = (read_data & 0x00000004) >> 2; // pps.deblocking_filter_control_present_flag = read_data[2]; // deblocking_filter_control_present_flag
    pps->constrained_intra_pred_flag = (read_data & 0x00000002) >> 1; // pps.constrained_intra_pred_flag = read_data[1]; // constrained_intra_pred_flag
    pps->redundant_pic_cnt_present_flag = read_data & 0x00000001; // pps.redundant_pic_cnt_present_flag = read_data[1]; // redundant_pic_cnt_present_flag

    rbsp_trailing_bit();
       
    return RETCODE_OK;	
}

int compare_pic_by_pic_num_desc( const void *arg1, const void *arg2 )
{
    if ( (*(StorablePicture**)arg1)->L0_ref_pic_num < (*(StorablePicture**)arg2)->L0_ref_pic_num)
        return 1;
    if ( (*(StorablePicture**)arg1)->L0_ref_pic_num > (*(StorablePicture**)arg2)->L0_ref_pic_num)
        return -1;
    else
        return 0;
}

int compare_pic_by_is_lt_pic_asc( const void *arg1, const void *arg2 )
{
    if ( (*(StorablePicture**)arg1)->L0_ref_is_long_term < (*(StorablePicture**)arg2)->L0_ref_is_long_term)
        return -1;
    if ( (*(StorablePicture**)arg1)->L0_ref_is_long_term > (*(StorablePicture**)arg2)->L0_ref_is_long_term)
        return 1;
    else
        return 0;
}

int compare_pic_by_used_pic_desc( const void *arg1, const void *arg2 )
{
    //printk("L0_ref_idx_y_phy arg1=%x, arg2=%x ",(unsigned int)(*(StorablePicture**)arg1)->L0_ref_idx_yaddr_phy, (unsigned int)(*(StorablePicture**)arg2)->L0_ref_idx_yaddr_phy);    
    //printk("pic_desc arg1=%d, arg2=%d ",(*(StorablePicture**)arg1)->L0_used_for_ref, (*(StorablePicture**)arg2)->L0_used_for_ref);
    if ( (*(StorablePicture**)arg1)->L0_used_for_ref < (*(StorablePicture**)arg2)->L0_used_for_ref)
    {
        //printk(" < \n");
        return 1;
    }        
    if ( (*(StorablePicture**)arg1)->L0_used_for_ref > (*(StorablePicture**)arg2)->L0_used_for_ref)
    {
        //printk(" > \n");        
        return -1;
    }        
    else
    {
        //printk(" = \n");        
        return 0;
    }        
}

int compare_pic_by_lt_pic_num_asc( const void *arg1, const void *arg2 )
{
    if ( (*(StorablePicture**)arg1)->L0_ref_lt_pic_num < (*(StorablePicture**)arg2)->L0_ref_lt_pic_num)
        return -1;
    if ( (*(StorablePicture**)arg1)->L0_ref_lt_pic_num > (*(StorablePicture**)arg2)->L0_ref_lt_pic_num)
        return 1;
    else
        return 0;
}

void init_lists(void * ptDecHandle)
{
    DECODER * dec = (DECODER *)ptDecHandle;  
    struct_slice_header * ssh = dec->ssh;	
    struct_sps *sps = dec->sps[dec->active_sps_idx];
    StorablePicture **ref_pic_data = dec->ref_pic_data;	
    int i, j, gap_idx, end_num, max_frame_num, ref_count=0, st_ref_count=0; //short term
    StorablePicture *tmp_pic;
    
    //printk("enter init_lists\n");
    // store properties setting by MMCO
    for(i=0 ; i <sps->num_ref_frames; i++) {
        tmp_pic = *(ref_pic_data+i);
      
        for(j =0 ; j <sps->num_ref_frames; j++)  {
            if(dec->mmco_ref_pic_num[j]==tmp_pic->L0_ref_pic_num) {
                //printk("j=%d, dec->mmco_ref_pic_num[j] = %x, tmp_pic->L0_ref_pic_num=%x\n",j,dec->mmco_ref_pic_num[j], tmp_pic->L0_ref_pic_num);
                tmp_pic->L0_ref_is_long_term = dec->mmco_ref_is_long_term[j];
                tmp_pic->L0_used_for_ref = dec->mmco_used_for_ref[j];
                tmp_pic->L0_ref_lt_pic_num = dec->mmco_ref_lt_pic_num[j];
                break;
            }   	
        }
    }   
    //printk("init_lists num_ref_frames=%d\n",sps->num_ref_frames);    
    //printk("frame_num =%d, 

    if((ssh->slice_type == P_SLICE)||((sps->gaps_in_frame_num_value_allowed_flag==1) && (ssh->frame_num != (dec->prev_frame_num+1)))) {
        //printk("log2_max_frame_num_minus4 = %d\n",sps->log2_max_frame_num_minus4);
        max_frame_num = (1<<(sps->log2_max_frame_num_minus4+4));
        
        //printk("frame_num = %d, max_frame_num= %d\n",ssh->frame_num, max_frame_num);
        // insert dummy pictures if there's gap between ssh->frame_num
        end_num = ssh->frame_num % max_frame_num; 	
        //printk("  prev_frame_num = %d\n", dec->prev_frame_num);
        gap_idx = dec->prev_frame_num;
        //printk("gap_idx =%d\n",gap_idx);
	   
#if 0
        if(sps->gaps_in_frame_num_value_allowed_flag==1) {
            if(end_num >= gap_idx)
                frame_gap_num = end_num - gap_idx - 1;
            else
                frame_gap_num = max_frame_num - gap_idx + end_num - 1;	
        }
#else
	   
	   //printk("gaps_in_frame_num_value_allowed_flag =%d, L0_used_for_ref=%d\n",
	   //         sps->gaps_in_frame_num_value_allowed_flag,dec->prev_rec_frame->L0_used_for_ref);
	   if(sps->gaps_in_frame_num_value_allowed_flag==0)
	   {
    		// 2014-01-21: Added to avoid crash when decoding first frame is P frame.
    	    if (dec->prev_rec_frame == NULL)
    	    {
    	         end_num=gap_idx;
    	    }
    	    else
            {   
        	    if(dec->prev_rec_frame->L0_used_for_ref)
        	    {
        	   	    end_num=gap_idx+1;
        	    }
        	    else
        	    {
        	   	    end_num=gap_idx;
        	    }
    	    }
	   }
	   
#endif 
    //printk("Half init_lists\n");  
    //printk("\n\nend_num:%d   gap_idx:%d   sps->gaps_in_frame_num_value_allowed_flag:%d  dec->PreFrame_Used_for_Ref:%d \n",end_num,gap_idx,sps->gaps_in_frame_num_value_allowed_flag,dec->PreFrame_Used_for_Ref);	
	   			   
	   while(gap_idx != end_num) {
	   		ref_count=0;
	   		st_ref_count=0;    
	   		// sort used picture to the beginning of the list
	   		//printk("8. sort by compare_pic_by_used_pic_desc, sps->num_ref_frames=%d\n",sps->num_ref_frames);		   		
       		qsort((void *)(ref_pic_data), sps->num_ref_frames, sizeof(StorablePicture*), compare_pic_by_used_pic_desc); 		   
	   		for(i=0; i<sps->num_ref_frames; i++) {
	     		tmp_pic = *(ref_pic_data + i);
	     		if(tmp_pic->L0_used_for_ref==1) {
	     			ref_count++;
	     		}	
	   		}
	   
	   		// sort short term picture to the beginning of the list
       		//printk("7. sort by compare_pic_by_is_lt_pic_asc\n");	   		
       		qsort((void *)(ref_pic_data), ref_count, sizeof(StorablePicture*), compare_pic_by_is_lt_pic_asc);
	   		for(i=0; i<ref_count; i++) {
	     		tmp_pic = *(ref_pic_data + i);
	     		if(tmp_pic->L0_ref_is_long_term==0) {
	     			st_ref_count++;
	     		}	
	   		}   
       		// sort short term list by pic_num
       		//printk("6. sort by compare_pic_by_pic_num_desc\n");
       		qsort((void *)(ref_pic_data), st_ref_count, sizeof(StorablePicture*), compare_pic_by_pic_num_desc);

 	   		// insert previous frame into DPB	
	   		if(gap_idx == dec->prev_frame_num) {
	     		tmp_pic = dec->prev_rec_frame;
	   		} else {
	     		tmp_pic = *(ref_pic_data + sps->num_ref_frames);
	     		tmp_pic->L0_ref_pic_num = gap_idx;
	     		tmp_pic->L0_ref_is_long_term = 0;
	     		tmp_pic->L0_used_for_ref = 1;
	   		}   
	   		tmp_pic->L0_ref_pic_num = gap_idx;
	   
	   		if(ref_count==sps->num_ref_frames) {
	   	  		if(tmp_pic->L0_ref_is_long_term) {
	   	    		*(ref_pic_data + sps->num_ref_frames) = *(ref_pic_data + st_ref_count -1);
	   	    		*(ref_pic_data + st_ref_count -1) = tmp_pic; //drop the last short term ref frame
		    		// sort long term list by lt_pic_num
		    		//printk("1. sort by compare_pic_by_lt_pic_num_asc\n");
       	    		qsort((void *)(ref_pic_data + st_ref_count -1), sps->num_ref_frames - st_ref_count, sizeof(StorablePicture*), compare_pic_by_lt_pic_num_asc);	
	   	  		} else {
	   	    		*(ref_pic_data + sps->num_ref_frames) = *(ref_pic_data + st_ref_count -1);
	   	  			for(i=st_ref_count-1; i>0; i--)
	   	  	  			*(ref_pic_data + i) = *(ref_pic_data + i - 1);
	   	  			*(ref_pic_data) = tmp_pic; 
		    		// sort long term list by lt_pic_num
		    		//printk("2. sort by compare_pic_by_lt_pic_num_asc\n");		    		
       	    		qsort((void *)(ref_pic_data + st_ref_count), sps->num_ref_frames - st_ref_count, sizeof(StorablePicture*), compare_pic_by_lt_pic_num_asc);	 	
	   	  		}
	   		} else {
	   	  		if(tmp_pic->L0_ref_is_long_term) {
	   	  			for(i=sps->num_ref_frames; i>ref_count; i--)
	   	  	  			*(ref_pic_data + i) = *(ref_pic_data + i - 1);	   	  
	   	    		*(ref_pic_data + ref_count) = tmp_pic; //append to the final node of the list
		   	 		// sort long term list by lt_pic_num
		    		//printk("3. sort by compare_pic_by_lt_pic_num_asc\n");		   	 		
       	   			qsort((void *)(ref_pic_data + st_ref_count), ref_count -st_ref_count +1, sizeof(StorablePicture*), compare_pic_by_lt_pic_num_asc);	
	   	  		} else {
	   	  			for(i=sps->num_ref_frames; i>0; i--)
	   	  	  			*(ref_pic_data + i) = *(ref_pic_data + i - 1);
	   	  			*(ref_pic_data) = tmp_pic; 
		    		// sort long term list by lt_pic_num
		    		//printk("4. sort by compare_pic_by_lt_pic_num_asc\n");
       	    		qsort((void *)(ref_pic_data + st_ref_count +1), ref_count -st_ref_count, sizeof(StorablePicture*), compare_pic_by_lt_pic_num_asc);	 	
         	    		
	   	  		}
			
	   	  		for(i=ref_count +1; i<sps->num_ref_frames; i++)	{
	   	    		tmp_pic = *(ref_pic_data + i);
	   	    		tmp_pic->L0_used_for_ref = -1;
	   	    		tmp_pic->L0_ref_pic_num = -0xFFFF;
	   	  		} 
	   		}
	 	
	 		dec->prev_frame_num = gap_idx;
	    	gap_idx = (gap_idx+1) % max_frame_num;
	    
	    	if(sps->gaps_in_frame_num_value_allowed_flag==0)
	    		break;
		}//while(gap_idx != end_num)
	} else { // I-slice
	    //printk("9. sort by compare_pic_by_used_pic_desc\n");
      	qsort((void *)(ref_pic_data), sps->num_ref_frames+1, sizeof(StorablePicture*), compare_pic_by_used_pic_desc); 		   	
		for(i=0; i<=sps->num_ref_frames; i++) {
	    	tmp_pic = *(ref_pic_data + i);
	    	if(tmp_pic->L0_used_for_ref==1)
	    		ref_count++;
		}
		//printk("10. sort by compare_pic_by_pic_num_desc\n");
    	qsort((void *)(ref_pic_data), ref_count, sizeof(StorablePicture*), compare_pic_by_pic_num_desc);       
	}

    //printk("Checking here\n");
   	for(i =0 ; i <= sps->num_ref_frames; i++) {
    //printk("i=%d, pic = 0x%x\n", i, (unsigned int)*(ref_pic_data+i));   	    
    	tmp_pic = *(ref_pic_data+i);
    	// reorganize pic_num if frame_num reset to 0
    	// situation 1.IDR, 2.mmco5 ->will do mm_unmark_all_reference
    	// situation 3.frame_num exceed "(1<<(sps->log2_max_frame_num_minus4+4))", should do the following 
    	if((ssh->frame_num < (dec->prev_frame_num+1)) && (tmp_pic->L0_ref_is_long_term==0) && (tmp_pic->L0_used_for_ref==1))
     		tmp_pic->L0_ref_pic_num -= (dec->prev_frame_num+1);
	 	// set initial value for MMCO in the picture
	 	dec->mmco_ref_is_long_term[i] = tmp_pic->L0_ref_is_long_term;
	 	dec->mmco_used_for_ref[i] = tmp_pic->L0_used_for_ref;
	 	dec->mmco_ref_lt_pic_num[i] = tmp_pic->L0_ref_lt_pic_num;
	 	dec->mmco_ref_pic_num[i] = tmp_pic->L0_ref_pic_num;   	
   	}
   	
    //printk("exit init_lists\n");   	
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// including reorder_ref_pic_list(); in JM
//void ref_pic_list_reordering(void * ptDecHandle)
int ref_pic_list_reordering(void * ptDecHandle)
{
    DECODER * dec = (DECODER *)ptDecHandle;
    struct_slice_header * ssh = dec->ssh;
    struct_sps *sps = dec->sps[dec->active_sps_idx];
    decode_slice_data *dsd = dec->dsd;
	
    uint32_t read_data ;
    int picNumLXNoWrap, picNumLX;
    int refIdxLX = 0;
    int maxPicNum  = 1<<(sps->log2_max_frame_num_minus4+4);
    int currPicNum = ssh->frame_num;
    int picNumLXPred = currPicNum;
    
    ssh->ref_pic_list_reordering_flag_l0 = 0;
    ssh->reordering_of_pic_nums_idc = 0;
    ssh->abs_diff_pic_num_minus1 = 0;
    ssh->long_term_pic_num = 0;
    
    if(ssh->slice_type != I_SLICE) {
        read_data = read_u(1);
        ssh->ref_pic_list_reordering_flag_l0 = read_data & 0x00000001; // ssh.ref_pic_list_reordering_flag_l0 = read_data[0];

        if(ssh->ref_pic_list_reordering_flag_l0) {
            while(ssh->reordering_of_pic_nums_idc != 3) {
                read_data = read_ue();
                ssh->reordering_of_pic_nums_idc = read_data & 0x00000003; // ssh.reordering_of_pic_nums_idc = read_data[1:0];
                
                if(ssh->reordering_of_pic_nums_idc == 0 ||
                   ssh->reordering_of_pic_nums_idc == 1 ) {
                    read_data = read_ue();
                    ssh->abs_diff_pic_num_minus1 = read_data & 0x0000FFFF; // ssh.abs_diff_pic_num_minus1 = read_data[15:0];
 
 					//buffer management 4/4
      				if (ssh->reordering_of_pic_nums_idc == 0) {
        				if( picNumLXPred - ( ssh->abs_diff_pic_num_minus1 + 1 ) < 0 )
          					picNumLXNoWrap = picNumLXPred - ( ssh->abs_diff_pic_num_minus1 + 1 ) + maxPicNum;
        				else
          					picNumLXNoWrap = picNumLXPred - ( ssh->abs_diff_pic_num_minus1 + 1 );
      				} else {// (ssh->reordering_of_pic_nums_idc == 1)
      					if( picNumLXPred + ( ssh->abs_diff_pic_num_minus1 + 1 )  >=  maxPicNum )
          					picNumLXNoWrap = picNumLXPred + ( ssh->abs_diff_pic_num_minus1 + 1 ) - maxPicNum;
       	 				else
          					picNumLXNoWrap = picNumLXPred + ( ssh->abs_diff_pic_num_minus1 + 1 );
      				}
      				picNumLXPred = picNumLXNoWrap;

      				if( picNumLXNoWrap > currPicNum )
        				picNumLX = picNumLXNoWrap - maxPicNum;
      				else
        				picNumLX = picNumLXNoWrap;
        			//reorder_short_term
          			if (reorder_short_or_long_term(dec, dsd->num_ref_idx_l0_active, picNumLX, &refIdxLX, 0) < 0)
						return -1;
                }
				else if(ssh->reordering_of_pic_nums_idc == 2) {
                    read_data = read_ue();
                    ssh->long_term_pic_num = read_data & 0x000000FF; // ssh.long_term_pic_num = read_data[7:0];
                    //reorder_long_term
                    if (reorder_short_or_long_term(dec, dsd->num_ref_idx_l0_active, ssh->long_term_pic_num, &refIdxLX, 1) <0)
						return -1;
                }
            }
        }
    }
    
    ssh->ref_pic_list_reordering_flag_l1 = 0;
	return 0;
    /*
    if(ssh->slice_type == B_SLICE) {
        read_data = read_u(1);
        ssh->ref_pic_list_reordering_flag_l1 = read_data & 0x00000001; // ssh.ref_pic_list_reordering_flag_l1 = read_data[0];
        
        if(ssh->ref_pic_list_reordering_flag_l1) {
            ssh->reordering_of_pic_nums_idc = 0;
            while(ssh->reordering_of_pic_nums_idc != 3) {
                read_data = read_ue();
                ssh->reordering_of_pic_nums_idc = read_data & 0x00000003;  // ssh.reordering_of_pic_nums_idc = read_data[1:0];
                
                if(ssh->reordering_of_pic_nums_idc == 0 ||
                   ssh->reordering_of_pic_nums_idc == 1 ) {
                    read_data = read_ue();
                    ssh->abs_diff_pic_num_minus1 = read_data & 0x0000FFFF; // ssh.abs_diff_pic_num_minus1 = read_data[15:0];
                }
                else if(ssh->reordering_of_pic_nums_idc == 2) {
                    read_data = read_ue();
                    ssh->long_term_pic_num = read_data & 0x000000FF; // ssh.long_term_pic_num = read_data[7:0];
                }
            }
        }
    }
    */
}

//void reorder_short_or_long_term(void * ptDecHandle, int num_ref_idx_lX_active, int picNumLX, int *refIdxLX, int is_long_term)
int reorder_short_or_long_term(void * ptDecHandle, int num_ref_idx_lX_active, int picNumLX, int *refIdxLX, int is_long_term)
{
  	DECODER * dec = (DECODER *)ptDecHandle;
  	StorablePicture **ref_pic_data = dec->ref_pic_data;
  	int cIdx, nIdx, ref_pic_num, i;
  	StorablePicture *picLX = NULL, *tmp_pic, *max_num_pic;
 
  	//printf("require ref num %d\n", picNumLX);
  	for(cIdx=num_ref_idx_lX_active; cIdx>=0; cIdx--) {//MAX_REF_FRAME_NUM
  		tmp_pic = *(ref_pic_data+cIdx);
		if (tmp_pic == NULL)
			return -1;
    	if(tmp_pic->L0_used_for_ref==1) {
	 		if(is_long_term==1)
        		ref_pic_num = tmp_pic->L0_ref_lt_pic_num;
      		else
        		ref_pic_num = tmp_pic->L0_ref_pic_num;
        
      		if(ref_pic_num==picNumLX) {
        		picLX = tmp_pic;
        		break; 
      		}
    	}  
  	}
  	if(cIdx==-1) 
		return 0;
  
  	max_num_pic = *(ref_pic_data+num_ref_idx_lX_active-1);
  	for( cIdx = num_ref_idx_lX_active-1; cIdx > *refIdxLX; cIdx-- ) {
    	*(ref_pic_data+cIdx) = *(ref_pic_data+cIdx-1);
  	}
  
  	*(ref_pic_data+((*refIdxLX)++)) = picLX;

  	nIdx = *refIdxLX;

  	for( cIdx = *refIdxLX; cIdx < num_ref_idx_lX_active; cIdx++ ) {
    	tmp_pic = *(ref_pic_data+cIdx);
    	if (tmp_pic->L0_ref_idx_yaddr_phy) {
    	  	if(is_long_term==1)
        		ref_pic_num = tmp_pic->L0_ref_lt_pic_num;
      		else
        		ref_pic_num = tmp_pic->L0_ref_pic_num;
      
      		if( (tmp_pic->L0_ref_is_long_term!=is_long_term) || (ref_pic_num != picNumLX))
        		*(ref_pic_data+(nIdx++)) = tmp_pic;
		}  
  	}
  
  	if (max_num_pic->L0_ref_idx_yaddr_phy) {
		if(is_long_term==1)
      		ref_pic_num = max_num_pic->L0_ref_lt_pic_num;
    	else
      		ref_pic_num = max_num_pic->L0_ref_pic_num;
        
    	if( (max_num_pic->L0_ref_is_long_term!=is_long_term) || (ref_pic_num != picNumLX) || (max_num_pic->L0_used_for_ref==-1))
      		*(ref_pic_data+(nIdx++)) = max_num_pic;
  	}
  
  	for(i =0 ; i <= num_ref_idx_lX_active; i++) {
    	tmp_pic = *(ref_pic_data+i);
    	//printf("reorder ref %d, pic_num %d, lt_num %d, used %d, lt %d\n", i, tmp_pic->L0_ref_pic_num, tmp_pic->L0_ref_lt_pic_num, tmp_pic->L0_used_for_ref, tmp_pic->L0_ref_is_long_term);
  	} 
	return 0;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void pred_weight_table(void){} 

void mm_unmark_for_reference(void * ptDecHandle, int parameter, int is_long_term)
{
	DECODER * dec = (DECODER *)ptDecHandle;
	struct_sps *sps = dec->sps[dec->active_sps_idx];
	struct_slice_header * ssh = dec->ssh;  
	int picNumX, ref_pic_num;
	unsigned i;

	if(is_long_term)
		picNumX = parameter;
	else  
		picNumX = ssh->frame_num - (parameter + 1); // parameter = difference_of_pic_nums_minus1

	for(i=0; i<sps->num_ref_frames; i++){
		if(is_long_term)
			ref_pic_num = dec->mmco_ref_lt_pic_num[i];
		else
			ref_pic_num = dec->mmco_ref_pic_num[i];
    
		if ((dec->mmco_used_for_ref[i]==1) && (dec->mmco_ref_is_long_term[i]==is_long_term)){
			if (ref_pic_num == picNumX) {
				dec->mmco_used_for_ref[i] = -1;
				dec->mmco_ref_is_long_term[i] = 0;
				break;//return;
			}
		}
	}
  	return;
}    

void mm_assign_long_term_frame_idx(void * ptDecHandle, int difference_of_pic_nums_minus1, int long_term_frame_idx)
{
	DECODER * dec = (DECODER *)ptDecHandle;
	struct_sps *sps = dec->sps[dec->active_sps_idx];
	struct_slice_header * ssh = dec->ssh;  
	int picNumX, i;

	picNumX = ssh->frame_num - (difference_of_pic_nums_minus1 + 1);

	// remove frames with same long_term_frame_idx
	mm_unmark_for_reference(dec, long_term_frame_idx, 1);

	for(i=0; i<sps->num_ref_frames; i++) {
		if ((dec->mmco_ref_is_long_term[i]==0) && (dec->mmco_ref_pic_num[i] == picNumX)) {          
			dec->mmco_ref_lt_pic_num[i] = long_term_frame_idx;
			dec->mmco_ref_is_long_term[i] = 1;
			dec->mmco_used_for_ref[i] = 1;
			break;//return;
		}
	}  
	return;
}

void mm_update_max_long_term_frame_idx(void * ptDecHandle, int max_long_term_frame_idx_plus1)
{
	DECODER * dec = (DECODER *)ptDecHandle;
	struct_sps *sps = dec->sps[dec->active_sps_idx];
	unsigned i;

	dec->max_long_term_pic_idx = max_long_term_frame_idx_plus1 - 1;

	// check for invalid frames
	for (i=0; i<sps->num_ref_frames; i++){
		if ((dec->mmco_ref_is_long_term[i]==1) && (dec->mmco_ref_lt_pic_num[i] > dec->max_long_term_pic_idx))
		{
			dec->mmco_used_for_ref[i] = -1;
		}
	}
	return;
}
  
void mm_unmark_all_reference (void * ptDecHandle)
{
	DECODER * dec = (DECODER *)ptDecHandle;
	struct_sps *sps = dec->sps[dec->active_sps_idx];
	unsigned int i;
  
	for (i=0; i<sps->num_ref_frames; i++) {
		dec->mmco_used_for_ref[i] = -1;
	}
}
  
void mm_mark_current_picture_long_term(void * ptDecHandle, int long_term_frame_idx)
{
	DECODER * dec = (DECODER *)ptDecHandle;
	StorablePicture * prev_rec = dec->prev_rec_frame;    
 
	// remove long term pictures with same long_term_frame_idx
	mm_unmark_for_reference(dec, long_term_frame_idx, 1);

	prev_rec->L0_ref_is_long_term = 1;
	prev_rec->L0_ref_lt_pic_num = long_term_frame_idx;
	return;
}


void dec_ref_pic_marking(void * ptDecHandle)
{
    DECODER * dec = (DECODER *)ptDecHandle;
	struct_slice_header * ssh = dec->ssh;
    uint32_t read_data ;  
    StorablePicture * prev_rec = dec->prev_rec_frame;

    dec->last_has_mmco_5 = 0;    
    ssh->no_output_of_prior_pics_flag = 0;
    ssh->long_term_reference_flag = 0;
    ssh->adaptive_ref_pic_marking_mode_flag = 0;
    ssh->difference_of_pic_nums_minus1 = 0;
    ssh->long_term_pic_num = 0;
    ssh->long_term_frame_idx = 0;
    ssh->max_long_term_frame_idx_plus1 = 0;
    ssh->memory_management_control_operation = 0;
   
    if(dec->nal_unit_type == 5) {//IDR
        ssh->no_output_of_prior_pics_flag = read_u(1);//read_data & 0x00000001;
        ssh->long_term_reference_flag     = read_u(1);//read_data & 0x00000002;

        mm_unmark_all_reference(dec);
        if(ssh->long_term_reference_flag) {
    	  	dec->max_long_term_pic_idx = 0;
    	  	prev_rec->L0_ref_is_long_term = 1;
    	  	prev_rec->L0_ref_lt_pic_num  = 0;
  		} else {
    	  	dec->max_long_term_pic_idx = -1;
    	  	prev_rec->L0_ref_is_long_term = 0;
  		}
    } else  {
        read_data = read_u(1);
        ssh->adaptive_ref_pic_marking_mode_flag = read_data & 0x00000001;
        if(ssh->adaptive_ref_pic_marking_mode_flag) {
            do {
            	ssh->memory_management_control_operation = read_ue() & 0x00000007;   
            	if(ssh->memory_management_control_operation==0)
            		break;                 
                if( ssh->memory_management_control_operation == 1 ||
                    ssh->memory_management_control_operation == 3) {
                    ssh->difference_of_pic_nums_minus1 = read_ue() & 0x000000FF;
                }
            
                if( ssh->memory_management_control_operation == 2) {
                    ssh->long_term_pic_num = read_ue() & 0x000000FF;
                }

                if( ssh->memory_management_control_operation == 3 ||
                    ssh->memory_management_control_operation == 6) {
                    ssh->long_term_frame_idx = read_ue() & 0x000000FF;
                }

                if( ssh->memory_management_control_operation == 4) {
                    ssh->max_long_term_frame_idx_plus1 = read_ue() & 0x000000FF;
                }
                
    			switch (ssh->memory_management_control_operation) {
      				case 1: //short_term_
        			 mm_unmark_for_reference(dec, ssh->difference_of_pic_nums_minus1, 0);
       				 break;
     	 			case 2: //long_term_
        			 mm_unmark_for_reference(dec, ssh->long_term_pic_num, 1);
        			 break;
      				case 3:
        			 mm_assign_long_term_frame_idx(dec, ssh->difference_of_pic_nums_minus1, ssh->long_term_frame_idx);
        			 break;
      				case 4:
        			 mm_update_max_long_term_frame_idx (dec, ssh->max_long_term_frame_idx_plus1);
        			 break;
      				case 5:
        			 mm_unmark_all_reference(dec);
       				 dec->last_has_mmco_5 = 1;
        			 break;
      				case 6:
       				 mm_mark_current_picture_long_term(dec, ssh->long_term_frame_idx);
        			 break;
      				default:
      				 Console_Printf("invalid memory_management_control_operation in buffer\n");
    			}   
            }while(ssh->memory_management_control_operation !=0 );
            
            if ( dec->last_has_mmco_5 ) {
  				ssh->frame_num = 0;
  				dec->framepoc = 0;
    			prev_rec->L0_ref_pic_num = 0;
    			prev_rec->L0_ref_pic_poc = 0;
			}
        }
    } 
	return;
}

////////////////////////////////////////////////////////////////////////////////////
// KC : section 8.2.1 Decoding process for picture order count
void decode_poc(void * ptDecHandle)
{
    DECODER * dec = (DECODER *)ptDecHandle;    
	struct_sps *sps = dec->sps[dec->active_sps_idx];
	struct_slice_header * ssh = dec->ssh;	
	
	int i;
	// for POC mode 0:
	uint32_t MaxPicOrderCntLsb = (1<<(sps->log2_max_pic_order_cnt_lsb_minus4+4));

  	switch ( sps->pic_order_cnt_type ){
  	  case 0: //=== POC MODE 0      // KC : section 8.2.1.1
    	// 1st
    	if(dec->nal_unit_type==5) {//IDR
    		dec->PrevPicOrderCntMsb = 0;
     		dec->PrevPicOrderCntLsb = 0;
    	} else {
      		if (dec->last_has_mmco_5) {
        		dec->PrevPicOrderCntMsb = 0;
        		dec->PrevPicOrderCntLsb = dec->toppoc;
      		}
    	}
    
    	// Calculate the MSBs of current picture
    	if( ssh->pic_order_cnt_lsb  <  dec->PrevPicOrderCntLsb  &&  
      	    ( dec->PrevPicOrderCntLsb - ssh->pic_order_cnt_lsb )  >=  ( MaxPicOrderCntLsb / 2 ) )
      		dec->PicOrderCntMsb = dec->PrevPicOrderCntMsb + MaxPicOrderCntLsb;
    	else if ( ssh->pic_order_cnt_lsb  >  dec->PrevPicOrderCntLsb  &&
      		    ( ssh->pic_order_cnt_lsb - dec->PrevPicOrderCntLsb )  >  ( MaxPicOrderCntLsb / 2 ) )
      		dec->PicOrderCntMsb = dec->PrevPicOrderCntMsb - MaxPicOrderCntLsb;
    	else
      		dec->PicOrderCntMsb = dec->PrevPicOrderCntMsb;
    
    	// 2nd
    	//if(img->field_pic_flag==0)
    	//{           //frame pix
      	dec->toppoc = dec->PicOrderCntMsb + ssh->pic_order_cnt_lsb;
      	dec->bottompoc = dec->toppoc + ssh->delta_pic_order_cnt_bottom;
      	dec->framepoc = (dec->toppoc < dec->bottompoc)? dec->toppoc : dec->bottompoc;

    	//if ( img->frame_num!=img->PreviousFrameNum)
      	//	img->PreviousFrameNum=img->frame_num;

    	if(!dec->disposable_flag){
      		dec->PrevPicOrderCntLsb = ssh->pic_order_cnt_lsb;
      		dec->PrevPicOrderCntMsb = dec->PicOrderCntMsb;
    	}
    	break;

  	  case 1: //=== POC MODE 1      // KC: section 8.2.1.2
    	// 1st
    	if(dec->nal_unit_type==5) {//IDR
      		dec->FrameNumOffset=0;     //  first pix of IDRGOP, 
      		ssh->delta_pic_order_cnt_0=0; //ignore first delta
      		if(ssh->frame_num) 
      		  printk("frame_num != 0 in idr pix");
    	} else {
      		if (dec->last_has_mmco_5) {
        		dec->PreviousFrameNumOffset = 0;
        		//dec->PreviousFrameNum = 0;
      		}
      		if (ssh->frame_num < dec->prev_frame_num) {   //not first pix of IDRGOP
        		dec->FrameNumOffset = dec->PreviousFrameNumOffset + (1<<(sps->log2_max_frame_num_minus4+4));
      		} else {
        		dec->FrameNumOffset = dec->PreviousFrameNumOffset;
      		}
    	}

    	// 2nd
    	if(sps->num_ref_frames_in_pic_order_cnt_cycle) 
      		dec->AbsFrameNum = dec->FrameNumOffset + ssh->frame_num;
    	else 
      		dec->AbsFrameNum=0;
    	if(dec->disposable_flag && dec->AbsFrameNum>0)
      		dec->AbsFrameNum--;

    	// 3rd
    	dec->ExpectedDeltaPerPicOrderCntCycle=0;

    	if(sps->num_ref_frames_in_pic_order_cnt_cycle)
    	for(i = 0; i < sps->num_ref_frames_in_pic_order_cnt_cycle ;i++)
      		dec->ExpectedDeltaPerPicOrderCntCycle += sps->offset_for_ref_frame[i];

    	if(dec->AbsFrameNum) {
      		dec->PicOrderCntCycleCnt = (dec->AbsFrameNum-1) / sps->num_ref_frames_in_pic_order_cnt_cycle;
      		dec->FrameNumInPicOrderCntCycle = (dec->AbsFrameNum-1) % sps->num_ref_frames_in_pic_order_cnt_cycle;
      		dec->ExpectedPicOrderCnt = dec->PicOrderCntCycleCnt * dec->ExpectedDeltaPerPicOrderCntCycle;
      		for(i = 0 ;i <= dec->FrameNumInPicOrderCntCycle; i++)
        		dec->ExpectedPicOrderCnt += sps->offset_for_ref_frame[i];
    	} else { 
      		dec->ExpectedPicOrderCnt=0;
    	}

    	if(dec->disposable_flag)
      		dec->ExpectedPicOrderCnt += sps->offset_for_non_ref_pic;

    	//if(img->field_pic_flag==0)
    	//{           //frame pix
      	dec->toppoc = dec->ExpectedPicOrderCnt + ssh->delta_pic_order_cnt_0;
      	dec->bottompoc = dec->toppoc + sps->offset_for_top_to_bottom_field + ssh->delta_pic_order_cnt_1;
      	dec->framepoc = (dec->toppoc < dec->bottompoc)? dec->toppoc : dec->bottompoc; 

    	//dec->PreviousFrameNum = ssh->frame_num;
    	dec->PreviousFrameNumOffset = dec->FrameNumOffset;
  
    	break;

  	  case 2: //=== POC MODE 2      // KC: section 8.2.1.3
    	if(dec->nal_unit_type==5) {//IDR
      		dec->FrameNumOffset=0;     //  first pix of IDRGOP, 
      		dec->framepoc = dec->toppoc = dec->bottompoc = 0;
      		if(ssh->frame_num)
      		  printk("frame_num != 0 in idr pix");
    	} else {
      		if (dec->last_has_mmco_5){
        		//dec->PreviousFrameNum = 0;
        		dec->PreviousFrameNumOffset = 0;
      		}
      		if (ssh->frame_num < dec->prev_frame_num)
        		dec->FrameNumOffset = dec->PreviousFrameNumOffset + (1<<(sps->log2_max_frame_num_minus4+4));
      		else 
        		dec->FrameNumOffset = dec->PreviousFrameNumOffset;

      		dec->AbsFrameNum = dec->FrameNumOffset + ssh->frame_num;
      		if(dec->disposable_flag)
        		dec->framepoc = (2*dec->AbsFrameNum - 1);
      		else
        		dec->framepoc = (2*dec->AbsFrameNum); //img->ThisPOC

      		//if (img->field_pic_flag==0)
        	dec->toppoc = dec->bottompoc = dec->framepoc;
    	}

    	//if (!dec->disposable_flag)
      	//	dec->PreviousFrameNum=ssh->frame_num;
    	dec->PreviousFrameNumOffset = dec->FrameNumOffset;
    	break;
 		
 	  default:
    	//error must occurs
    	break;
  	}
	return;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
AVC_RET read_nal_slice(void * ptDecHandle)      // kc:slice_header() in spec
{
	DECODER * dec = (DECODER *)ptDecHandle;
	uint32_t read_data;
	struct_sps **sps_list = dec->sps;
	struct_sps *sps;
	struct_pps **pps_list = dec->pps;
	struct_pps *pps;	
    struct_slice_header *ssh = dec->ssh;
	decode_slice_data   *dsd = dec->dsd;
	
    // init value    
    ssh->field_pic_flag = 0;
    ssh->pic_order_cnt_lsb = 0;
    ssh->pic_order_cnt_lsb = 0;
    ssh->delta_pic_order_cnt_bottom = 0;

    ssh->first_mb_in_slice = read_ue() & 0x00003FFF; // first_mb_in_slice
    
    ssh->slice_type = read_ue() & 0x0000001F; // slice_type

  	if(ssh->slice_type > 4) 
	  ssh->slice_type = ssh->slice_type - 5;

	if((ssh->slice_type != P_SLICE) && (ssh->slice_type != I_SLICE)){
		//printk("Unsupported read_nal_slice: 1-1, ssh->slice_type = 0x%x\n",ssh->slice_type);
		return RETCODE_ERR_HEADER;
	}	
#if 1
	// Check the unsupported bit-stream to aovid panic
	//printk("1. dec->active_pps_idx = %d\n",dec->active_pps_idx);
	pps = *(pps_list + dec->active_pps_idx);
    if (pps->num_slice_groups_minus1 > 0)
    {
    	//Console_Printf("num_slice_groups_minus1 = %d\n",pps->num_slice_groups_minus1);
    	return RETCODE_ERR_HEADER;
    }	
	sps = *(sps_list + dec->active_sps_idx);    
    if (sps->level_idc > Support_Level)
    {
        printk("Level = %d  is larger than supported (%d)\n",sps->level_idc, Support_Level);
    	return RETCODE_ERR_HEADER;        
    }
    //else
    //    printk("sps->level_idc = %d\n",sps->level_idc);
#endif	
	ssh->pic_parameter_set_id = read_ue() & 0x000000FF; // pps_id
  	dec->active_pps_idx = ssh->pic_parameter_set_id;
	//printk("2. dec->active_pps_idx = %d\n",dec->active_pps_idx);  	
	pps = *(pps_list + dec->active_pps_idx);
	if (pps == NULL) {
		printk ("no this pps (%d)\n", dec->active_pps_idx);
		return RETCODE_ERR_HEADER;
	}
	dec->active_sps_idx = pps->seq_parameter_set_id;
	sps = *(sps_list + dec->active_sps_idx);
	if (sps == NULL) {
		printk ("no this sps (%d)\n", dec->active_sps_idx);
		return RETCODE_ERR_HEADER;
	}
    dec->prev_frame_num = ssh->frame_num;
	ssh->frame_num = read_u(sps->log2_max_frame_num_minus4+4);//&0x0000001F;
		
	ssh->bottom_field_flag = 0;
		
    if(!sps->frame_mbs_only_flag){
        ssh->field_pic_flag = read_u(1) & 0x00000001;  // field_pic_flag

        if(ssh->field_pic_flag)
        	ssh->bottom_field_flag =read_u(1) & 0x00000001;  // bottom_field_flag
    }
    
    read_data = 0;    
    if(dec->nal_unit_type == 5) //IDR
      read_data=read_ue();

    ssh->idr_pic_id = read_data&0x0000FFFF; // [15:0] idr_pic_id

    if(sps->pic_order_cnt_type == 0) {
        ssh->pic_order_cnt_lsb =read_u(sps->log2_max_pic_order_cnt_lsb_minus4+4) & 0x0000FFFF; // [15:0] pic_order_cnt_lsb
       
        if(pps->pic_order_present_flag && !ssh->field_pic_flag)
            ssh->delta_pic_order_cnt_bottom = read_se();
   }

   ssh->delta_pic_order_cnt_0 = 0;
   ssh->delta_pic_order_cnt_1 = 0;
    
   if(sps->pic_order_cnt_type == 1 && !sps->delta_pic_order_always_zero_flag) {
        ssh->delta_pic_order_cnt_0 = read_se();

        if(pps->pic_order_present_flag && !ssh->field_pic_flag)
            ssh->delta_pic_order_cnt_1 = read_se();
   }
    
   ssh->direct_spatial_mv_pred_flag = 0;
   if(ssh->slice_type == B_SLICE) 
        ssh->direct_spatial_mv_pred_flag = read_u(1)&0x00000001;

    
   ssh->num_ref_idx_active_override_flag = 0;
   ssh->num_ref_idx_l0_active_minus1 = 0;
   ssh->num_ref_idx_l1_active_minus1 = 0;
    
   if(ssh->slice_type == P_SLICE || ssh->slice_type == B_SLICE) {
        ssh->num_ref_idx_active_override_flag = read_u(1) & 0x00000001;

        if(ssh->num_ref_idx_active_override_flag) { 
            ssh->num_ref_idx_l0_active_minus1 = read_ue() & 0x0000001F;  // [4:0]

            if(ssh->slice_type == B_SLICE)
                ssh->num_ref_idx_l1_active_minus1 =read_ue() & 0x0000001F;  // [4:0]

      		dsd->num_ref_idx_l0_active = ssh->num_ref_idx_l0_active_minus1+1;
      		dsd->num_ref_idx_l1_active = ssh->num_ref_idx_l1_active_minus1+1;
    	} else	{
      		dsd->num_ref_idx_l0_active = pps->num_ref_idx_l0_active_minus1+1;
      		dsd->num_ref_idx_l1_active = pps->num_ref_idx_l1_active_minus1+1;
    	}
    }
    
    if(ssh->first_mb_in_slice == 0){
    	init_lists(dec); //1. sorting DBP and insert previous frame into DPB
    					 //2. store and reset the properties setting by MMCO
    					 //3. reorganize pic_num if frame_num reset to 0
    					 //4. insert dummy pictures if there's gap between ssh->frame_num
    	decode_poc(dec); //calculate poc value for current frame    // KC : poc stand for picture order count. Section 8.2.1 Decoding process for picture odrder count				 
        write_frm_idx(dec); //set current frame parameter
    }    
    if (ref_pic_list_reordering(dec) < 0) {
		printk ("bs error in ref_pic_list_reordering\n");
		return RETCODE_ERR_HEADER;
	}   
    
    if(ssh->first_mb_in_slice == 0){
        write_frm_addr(dec); //set reference frame addr into register
    }    
        
    if( (pps->weighted_pred_flag && ssh->slice_type == P_SLICE) ||
        (pps->weighted_bipred_idc && ssh->slice_type == B_SLICE)) {    	
        pred_weight_table();
    }
    
    if(dec->nal_ref_idc) {
        dec_ref_pic_marking(dec);
    }
 	  
    ssh->cabac_init_idc = 0;
    if(pps->entropy_coding_mode_flag && ssh->slice_type != I_SLICE)
        ssh->cabac_init_idc = read_ue() & 0x00000003;  // [1:0]
    
    ssh->slice_qp_delta = read_se() & 0x0000003F;  // [5:0]

    // initial
    ssh->disable_deblocking_filter_idc = 0;
    ssh->slice_alpha_c0_offset_div2 = 0;
    ssh->slice_beta_offset_div2 = 0;

    
    ssh->disable_deblocking_filter_idc = 0;
    ssh->slice_alpha_c0_offset_div2 = 0;
    ssh->slice_beta_offset_div2 = 0;
  	  
    if(pps->deblocking_filter_control_present_flag) {
        ssh->disable_deblocking_filter_idc = read_ue() & 0x00000003;
        if(ssh->disable_deblocking_filter_idc != 1) { 
            ssh->slice_alpha_c0_offset_div2 = read_se() & 0x0000000F;
            ssh->slice_beta_offset_div2 = read_se() & 0x0000000F;
        }
    }
    
    dsd->slice_type              = ssh->slice_type&0x00000003;
    dsd->frame_num               = ssh->frame_num&0x0000001F;
    dsd->qp                      = ssh->slice_qp_delta + pps->pic_init_qp_minus26 + 0x0000001A;
      
    //printk("2222 read_nal_slice ssh->first_mb_in_slice:%d  \n",ssh->first_mb_in_slice);
	return RETCODE_OK;
}
//////////////read_nal_sei/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

uint32_t show_u(uint8_t size)
{
	uint32_t result;
	
	result = inp32(REG_264_ADDR_BSM);
	result = result >> (32-size);
	
	return result;
}

int read_nal_sei(void * ptDecHandle)
{
	DECODER * dec = (DECODER *)ptDecHandle;
	uint32_t read_data;
	uint32_t payloadType, payloadSize;
	int cnt, i;
	
	cnt = dec->u32BS_buf_sz_remain;
	do{
        payloadType = 0;
        read_data = read_u(8);
        cnt--;
        while (read_data == 0xFF) {
			payloadType+=255;
            read_data = read_u(8);
            cnt--;
		}
		payloadType +=read_data;

        payloadSize = 0;
        read_data = read_u(8);
        cnt--;
        while (read_data == 0xFF) {
			payloadSize+=255;
            read_data = read_u(8);
            cnt--;
		}
		payloadSize +=read_data; // unit:byte

        for (i = 0; i<payloadSize; i++) {
            read_data = read_u(8);
            cnt--;
		}
		read_data = show_u(8);
        if (cnt <= 0) {
            printk("sei error\n");
            return -1;
        }
	}while(read_data!= 0x80);
    if (cnt <= 0)
        return 0;
	return 1;
} 


int cropcom_check (DECODER * dec)
{

    uint32_t lcd_param_c, lcd_param_y;
    uint32_t misc_value = 0;
	int tmpx = dec->video_width - dec->u32FrameBufferWidth;
	int tmpy = dec->video_height - dec->u32FrameBufferHeight;
	
	// Skip the LCD crop or stride to let output continues data
	if ((dec->u32FrameBufferWidth == -1) || (dec->u32FrameBufferHeight==-1))
	{
	    if ( dec->output_fmt == OUTPUT_FMT_YUV422)
            outp32(REG_264_MISC,(inp32(REG_264_MISC) & 0xFFFFFFF3) | 0x01);	    // disable LCD_MODE_LARGE and LCD_MODE_SMALL
        else    
            outp32(REG_264_MISC,inp32(REG_264_MISC) & 0xFFFFFFF3);		        // disable LCD_MODE_LARGE and LCD_MODE_SMALL
        return 0;
    }        
	    
	    

//printk("wh %d %d %d %d\n", dec->video_width, dec->video_height, dec->u32FrameBufferWidth, 
//dec->u32FrameBufferHeight);
#ifndef DISPLAY_REORDER_CTRL	//KC_added
	if ((tmpx >= 0) && (tmpy >= 0 )) {			// lcd small
		if (tmpx > 15*16)
			tmpx = 15*16;
		if (dec->crop_x > tmpx) {
			printk ("limit crop_x from %d to %d\n", dec->crop_x, tmpx);
			dec->crop_x = tmpx;
		}
		if (tmpy > 15*16)
			tmpy = 15*16;
		if (dec->crop_y > tmpy) {
			printk ("limit crop_y from %d to %d\n", dec->crop_y, tmpy);
			dec->crop_y = tmpy;
		}
		misc_value = 0x8;
		outp32(REG_264_LCD_PARAM, ( ((dec->crop_x>>4)<<20) |
													((dec->crop_y>>4)<<16) |
													((dec->u32FrameBufferHeight>>4)<<8) | 
													(dec->u32FrameBufferWidth>>4) ));

	}
	else if (((tmpx < 0) && (tmpy > 0)) ||
			   ((tmpx > 0) && (tmpy < 0))) {
		// error combination
		printk ("NOT support this video/lcd combination:\n");
		printk ("video_width  (%d) < lcd_width (%d) && video_height(%d) > lcd_height (%d)\n",
				dec->video_width, dec->video_height, dec->u32FrameBufferWidth, dec->u32FrameBufferHeight);
		return -1;
	}
	else {
		// lcd larger
		tmpx = -tmpx;
		misc_value = 0x10;
		lcd_param_c = ((tmpx>>4)<<1) +1;
		if (dec->output_fmt == OUTPUT_FMT_YUV422)
			lcd_param_y = ((tmpx>>4)<<3) +1;
		else
			lcd_param_y = ((tmpx>>4)<<2) +1;			

//#define LCD_PARAM_Y 720
/*
		if (lcd_param_y >= LCD_PARAM_Y) {
			printk ("The width difference (%d) between lcd  & vide over the max (%d)", 
																				lcd_param_y, LCD_PARAM_Y);
			return -1;
		}
*/		
	    outp32(REG_264_LCD_FRAME_LINE_OFF, (lcd_param_c<<16)| lcd_param_y);	
	    outp32(REG_264_LCD_PARAM, ((dec->u32FrameBufferHeight>>4)<<8) | (dec->u32FrameBufferWidth>>4));	    		    			
		if (dec->crop_x || dec->crop_y)
			printk ("skip crop setting x, y  (%d %d)\n", dec->crop_x, dec->crop_y);		
	}
#endif		
	if ( dec->output_fmt == OUTPUT_FMT_YUV422)
	    outp32(REG_264_MISC,misc_value | 1);			
	else
	    outp32(REG_264_MISC,misc_value);			

	return 0;
}

void read_nal_delimiter(void){}

AVC_RET init_parameters(void * ptDecHandle, int dev)
{
	DECODER * dec = (DECODER *)ptDecHandle;
	decode_parameter     *decode_para = dec->decode_para;
	struct_sps *sps = dec->sps[dec->active_sps_idx];
	StorablePicture **ref_pic_data = dec->ref_pic_data;
	StorablePicture *tmp_pic;
#ifdef DISPLAY_REORDER_CTRL
	DisplayPicture **disp_pic_data = dec->disp_pic_data;
	DisplayPicture *disp_pic;
#endif
	

	
	int  tmp_index;
	uint32_t  frame_size;
	uint32_t  frame_width;
	
	uint32_t  dec_start_ref_phy;
	
	//dec_start_ref_phy = dec_ref_phy_buffer;
//0617	dec_start_ref_phy = dec_bs_phy_buffer + dev * DEC_ONE_INSTANCE_SIZE + DEC_BITSTREAM_SIZE;
    dec_start_ref_phy = dec_bs_phy_buffer + DECODER_INST[dev].u32Offset + DECODER_INST[dev].u32BSsize;
	                    
	dec->video_width = sps->pic_width_in_mbs <<4; //*16;
	dec->video_height = sps->pic_height_in_map_units <<4; //*16;
	
	frame_size = dec->video_width * dec->video_height;		
	frame_width = sps->pic_width_in_mbs <<4; //* 16;

	// Write framesize in mb unit to register    
	outp32(REG_264_FRAMESIZE,   sps->pic_width_in_mbs * sps->pic_height_in_map_units);

	// The OFFSET "MUST" be the multiplies of 8 //  offset must be aligned to 8-uint32_t
	// Hylin: 8 uint32_ts per MB. May be modified according to MCP220 design.
	decode_para->MB_INFO_OFFSET  = sps->pic_width_in_mbs; // in 8 uint32_t unit
	decode_para->INTRA_PRED_OFFSET = sps->pic_width_in_mbs; // in 8 uint32_t unit

	// in byte unit
	decode_para->DISP_OUT_OFFSET = frame_size << 1; //2*


#ifdef DISPLAY_REORDER_CTRL
	//----------  allocate memory for display reorder
	// DISP BUFFER
	dec->dpb_size = getDpbSize(dec);
	for(tmp_index=dec->dpb_size-1; tmp_index>=0 ; tmp_index--){
		if(*(disp_pic_data+tmp_index) == NULL){
			if((*(disp_pic_data+tmp_index) = nv_malloc(sizeof(DisplayPicture),32)) == NULL)		        
				return RETCODE_ERR_MEMORY;

			disp_pic = *(disp_pic_data+tmp_index);
			disp_pic->disp_pic_poc = 0xFFFF;  
			disp_pic->is_flushed = 0;
			if ( dec->output_fmt == OUTPUT_FMT_YUV420)
			{
#ifdef LINUX_ENV
    error here;
#else			    
				if((disp_pic->disp_idx_yaddr = nv_malloc(frame_size*3/2,32)) == NULL) {			// Buggy in Linux. Must Reserve Buffer in non-Cache region	
#endif				
					return RETCODE_ERR_MEMORY;
				}
				memset((unsigned char *)(CACHE_BIT31 | (unsigned int)disp_pic->disp_idx_yaddr) , 0x0, frame_size*3/2);				
			}
			else
			{
#ifdef LINUX_ENV
    error here;
#else			    
				if((disp_pic->disp_idx_yaddr = nv_malloc(frame_size*2,32)) == NULL) {			// Buggy in Linux. Must Reserve Buffer in non-Cache region	
#endif				
					return RETCODE_ERR_MEMORY;
				}	
				memset((unsigned char *)(CACHE_BIT31 | (unsigned int)disp_pic->disp_idx_yaddr) , 0x0, frame_size*2);
			}
			disp_pic->disp_idx_yaddr_phy = disp_pic->disp_idx_yaddr;
		}
	}
#endif

    // ----------------------------------------

    // in byte unit, not align to 1K byte
    //decode_para->END_ADDR = decode_para->DISP_OUT_BASE + decode_para->DISP_OUT_OFFSET; //not used in current version

    // ILF Y line offset
    decode_para->ILFTOP_Y_LINEOFFS = frame_width*3;
    
    // ILF UV line offset
    decode_para->ILFTOP_UV_LINEOFFS = frame_width;
        
    // Reconstruct line offset
    decode_para->RECON_OUT_LINEOFFS = frame_width - 15;
    
    // Disp Y line offset
#ifndef DISPLAY_REORDER_CTRL    //KC_Added
    if ( dec->output_fmt == OUTPUT_FMT_YUV422) {
        if (dec->u32FrameBufferWidth == -1)
        {
            decode_para->DP_Y_OUT_LINEOFFS = ((dec->video_width>>4)<<3)- 8 + 1;         // Output continue address  
        }    
        else
        {
            decode_para->DP_Y_OUT_LINEOFFS = ((dec->u32FrameBufferWidth>>4)<<3)- 8 + 1; // Output for stride = FrameBufferWidth
        }            
    } else {
        if (dec->u32FrameBufferWidth == -1)
    	    decode_para->DP_Y_OUT_LINEOFFS = ((dec->video_width>>4)<<2)- 4 + 1;        
        else
    	    decode_para->DP_Y_OUT_LINEOFFS = ((dec->u32FrameBufferWidth>>4)<<2)- 4 + 1; // Output for stride = FrameBufferWidth
    }
#else
    if ( dec->output_fmt == OUTPUT_FMT_YUV422) {
        decode_para->DP_Y_OUT_LINEOFFS = ((dec->video_width>>4)<<3)- 8 + 1;
    } else {
    	decode_para->DP_Y_OUT_LINEOFFS = ((dec->video_width>>4)<<2)- 4 + 1;
    }
#endif    

    
    // Disp UV line offset
#ifndef DISPLAY_REORDER_CTRL    //KC_Added    
    if (dec->u32FrameBufferWidth == -1)
        decode_para->DP_UV_OUT_LINEOFFS = ((dec->video_width>>4)<<1)- 2 + 1;    
    else 
        decode_para->DP_UV_OUT_LINEOFFS = ((dec->u32FrameBufferWidth>>4)<<1)- 2 + 1;        // Output for stride = FrameBufferWidth
#else    
    decode_para->DP_UV_OUT_LINEOFFS = ((dec->video_width>>4)<<1)- 2 + 1;    
#endif


    //----------  allocate memory and define base addr for ref frame
    // each frame can allocate independently
		
    if(sps->num_ref_frames > MAX_REF_FRAME_NUM)	{
        Console_Printf("Unsupported sps->num_ref_frames > MAX_REF_FRAME_NUM\n");
        return RETCODE_ERR_HEADER;
    }       
    
    //printk("num_ref_frames = %d\n",sps->num_ref_frames);
	
    for(tmp_index = sps->num_ref_frames; tmp_index >=0; tmp_index--) {
        if(*(ref_pic_data+tmp_index) == NULL){
            if((*(ref_pic_data+tmp_index) = (StorablePicture *)nv_malloc(sizeof(StorablePicture),32))==NULL) {            
				Console_Printf("FAVC allocate mem StorablePicture fail\n");
				return RETCODE_ERR_MEMORY;
			}
   
			// 1k byte align
			tmp_pic = *(ref_pic_data+tmp_index);

// Buffer assigned in H264DecRegisterFrameBuffer(...) function			

            /* Bugg Here
            if ((MAXIUMUFRAMNUM) < sps->num_ref_frames)
            {
				printk("FAVC Reference frame buffer L0_ref_idx_yaddr_phy is not assigned\n");				
	    		return RETCODE_ERR_MEMORY;                
            }
            */
            //printk("index = 0x%x, ref_yaddr_phy = 0x%x\n",tmp_index, dec_start_ref_phy);
            tmp_pic->L0_ref_idx_yaddr_phy = (uint8_t *)dec_start_ref_phy;
            //dec_start_ref_phy += ((frame_size*3/2) / 1024 * 1024);
            dec_start_ref_phy += ((frame_size*3/2 + 1023) / 1024 * 1024);            
			
			//printk("Alloc:L0_ref_idx_yaddr VA 0x%x PA 0x%x\n",tmp_pic->L0_ref_idx_yaddr,tmp_pic->L0_ref_idx_yaddr_phy);
	
			
			tmp_pic->L0_ref_is_long_term = 0;
			tmp_pic->L0_used_for_ref = -1;
			tmp_pic->L0_ref_pic_num = -0xFFFF;
			tmp_pic->L0_ref_lt_pic_num = 0xFFFF;
        }
    }

    tmp_pic = *(ref_pic_data+sps->num_ref_frames);
    dec->rec_frame_yaddr = tmp_pic->L0_ref_idx_yaddr_phy;

    //----------- write into register
    // MB information frame base address
	outp32(REG_264_MBINFO_BASE, decode_para->MB_INFO_BASE);    

    // Intra prediction coefficient base address
	outp32(REG_264_INTRA_BASE,  decode_para->INTRA_PRED_BASE);    

    // MB information frame offset & Intra prediction coefficient offset
	outp32(REG_264_OFFSET_REG0, (decode_para->MB_INFO_OFFSET<<16) | (decode_para->INTRA_PRED_OFFSET));    
     
    // ILF line offset
	outp32(REG_264_OFFSET_REG1, (decode_para->ILFTOP_UV_LINEOFFS<<16) | (decode_para->ILFTOP_Y_LINEOFFS));    

    // RECON_LINEOFFSET
    outp32(REG_264_RECON_LINEOFFS, decode_para->RECON_OUT_LINEOFFS);    

    // DP line offset
     outp32(REG_264_DP_LINEOFFS, (decode_para->DP_Y_OUT_LINEOFFS>>9)<<24 |
			(decode_para->DP_UV_OUT_LINEOFFS<<16) |
			(decode_para->DP_Y_OUT_LINEOFFS & 0x1FF));
	
	//printk("vide_width =0x%x, video_height = 0x%x\n",dec->video_width, dec->video_height);
	//printk("DP_Y_OUT_LINEOFFS = 0x%x, DP_UV_OUT_LINEOFFS= 0x%x\n",decode_para->DP_Y_OUT_LINEOFFS, decode_para->DP_UV_OUT_LINEOFFS);
	
    if (cropcom_check(dec) < 0)
        return RETCODE_ERR_GENERAL;


    // Write ref frame addr
    write_frm_addr(dec); // initial frame address
	
    return RETCODE_OK;
}


// will be called each frame, the addr rotated in function write_frm_idx 
void write_frm_addr(void * ptDecHandle)
{
	DECODER * dec = (DECODER *)ptDecHandle;
    int index;
    uint32_t based_addr;
	StorablePicture **ref_pic_data = dec->ref_pic_data;
	StorablePicture *tmp_pic;
	struct_sps *sps = dec->sps[dec->active_sps_idx];

	// write current Y frame base and reference frame addr
	outp32(REG_264_RECON_BASE, (uint32_t)dec->rec_frame_yaddr);	
	//outp32(REG_264_RECON_BASE, (uint32_t)virt_to_phys((void *)dec->rec_frame_yaddr));	

    based_addr = (uint32_t)AHBBASE_LOCAL + H264_OFF_REF_FRAME;       
    for(index = 0; index < sps->num_ref_frames; index++) {
		tmp_pic = *(ref_pic_data+index);
		if(tmp_pic->L0_used_for_ref==1){
		    // Important
		    //printk("Ref Reg : 0x%x = 0x%x\n", (unsigned int)based_addr, (unsigned int)tmp_pic->L0_ref_idx_yaddr_phy);
          *(uint32_t *)based_addr = (uint32_t)tmp_pic->L0_ref_idx_yaddr_phy;  
          based_addr = based_addr + 4; // uint32_t
        }
    }
    
}


// only for 4:2:0
void cropping_frame(void * ptDecHandle)
{
#if 1
    printk("!! Warning : cropping frame is not supported now\n");
#else
    // Below code only implement YUV420 planr and clear buffer as 0x80 outside of cropping window
    // Due to the H.264 encoder will not support cropping frame, this code is removed temporary now.
	DECODER * dec = (DECODER *)ptDecHandle;
	struct_sps *sps = dec->sps[dec->active_sps_idx];
	int32_t row_idx, valid_row, valid_col;
	uint8_t *ptr;
		
	// Y
	memset(dec->output_base_phy, 0x80, (sps->frame_crop_top_offset<<1)*dec->video_width);
	ptr = dec->output_base_phy + (sps->frame_crop_top_offset<<1)*dec->video_width;
	
	valid_row = dec->video_height - (sps->frame_crop_bottom_offset << 1);
	valid_col = dec->video_width - (sps->frame_crop_right_offset << 1);	

	memset(ptr, 0x80, (sps->frame_crop_left_offset<<1));
	ptr += valid_col;
										   
	for(row_idx = (sps->frame_crop_top_offset<<1) + 1; row_idx < valid_row; row_idx++ )
	{		
		memset(ptr, 0x80, (sps->frame_crop_right_offset + sps->frame_crop_left_offset) << 1);
		ptr += dec->video_width;
	}
	memset(ptr, 0x80, (sps->frame_crop_right_offset<<1) + (sps->frame_crop_bottom_offset<<1)*dec->video_width);
	
	
	if(dec->output_fmt == OUTPUT_FMT_YUV420) {
		// U
		memset(dec->output_base_u_phy, 0x80, sps->frame_crop_top_offset*(dec->video_width >> 1));
		ptr = dec->output_base_u_phy + sps->frame_crop_top_offset*(dec->video_width >> 1);
	
		valid_row = (dec->video_height>>1) - (sps->frame_crop_bottom_offset);
		valid_col = (dec->video_width>>1) - (sps->frame_crop_right_offset);	

		memset(ptr, 0x80, sps->frame_crop_left_offset);
		ptr += valid_col;
		for(row_idx = sps->frame_crop_top_offset + 1; row_idx < valid_row; row_idx++ )
		{		
			memset(ptr, 0x80, (sps->frame_crop_right_offset + sps->frame_crop_left_offset));
			ptr += (sps->frame_crop_right_offset + valid_col);
		}
		memset(ptr, 0x80, sps->frame_crop_right_offset + sps->frame_crop_bottom_offset*(dec->video_width>>1));
		
		
		// V
		memset(dec->output_base_v_phy, 0x80, sps->frame_crop_top_offset*(dec->video_width >> 1));
		ptr = dec->output_base_v_phy + sps->frame_crop_top_offset*(dec->video_width >> 1);
	
		valid_row = (dec->video_height>>1) - (sps->frame_crop_bottom_offset);
		valid_col = (dec->video_width>>1) - (sps->frame_crop_right_offset);	

		memset(ptr, 0x80, sps->frame_crop_left_offset);
		ptr += valid_col;
		for(row_idx = sps->frame_crop_top_offset + 1; row_idx < valid_row; row_idx++ )
		{		
			memset(ptr, 0x80, (sps->frame_crop_right_offset + sps->frame_crop_left_offset));
			ptr += (sps->frame_crop_right_offset + valid_col);
		}
		memset(ptr, 0x80, sps->frame_crop_right_offset + sps->frame_crop_bottom_offset*(dec->video_width>>1));
	}	
#endif	
}


#ifdef DISPLAY_REORDER_CTRL
int32_t getDpbSize(void * ptDecHandle)
{
	DECODER * dec = (DECODER *)ptDecHandle;
	struct_sps *sps = dec->sps[dec->active_sps_idx];
	
  	int32_t pic_size = sps->pic_width_in_mbs * sps->pic_height_in_map_units * 384;
	int32_t size = 0;

	// This value is derived by Table A-1 MaxDPB * 1024
	Console_Printf("sps level: %d\n",sps->level_idc);  		
  	switch (sps->level_idc)
  	{
  	case 10:
    	size = 152064;
    	break;
  	case 11:
    	size = 345600;
    	break;
  	case 12:
    	size = 912384;
    	break;
  	case 13:
    	size = 912384;
    	break;
  	case 20:
    	size = 912384;
    	break;
  	case 21:
    	size = 1824768;
    	break;
  	case 22:
    	size = 3110400;
    	break;
  	case 30:
    	size = 3110400;
    	break;
  	case 31:
    	size = 6912000;
    	break;
  	case 32:
    	size = 7864320;
    	break;
#if 0    	
  	case 33:
    	printk("warning : Level 3.3 is undocumented\n");  	
    	size = 7864320;
    	break;    	
#endif    	
  	case 40:
    	size = 12582912;
    	break;
  	case 41:
    	size = 12582912;
    	break;
  	case 50:
    	size = 7864320;//42393600;
    	break;
#if 0    	    
  	case 51:
    	printk("warning : Level 5.1 is undocumented\n");    	
    	size = 7864320;//70778880;
    	break;     		
#endif    	
  	default:
    	//printf("undefined level\n");
    	Console_Printf("undefined level for %d\n",sps->level_idc);
    	break;
  }

  size /= pic_size;
  size = min( size, 16);
	
		

  if (sps->vui_parameters_present_flag && sps->bitstream_restriction_flag)  {
    if (sps->max_dec_frame_buffering > size)  {
      //printf("max_dec_frame_buffering larger than MaxDpbSize");
       Console_Printf("max_dec_frame_buffering larger than MaxDpbSize");
    }
    size = max (1, sps->max_dec_frame_buffering);

  
  }
  return size;
}


int compare_pic_by_poc_asc( const void *arg1, const void *arg2 )
{
    if ( (*(DisplayPicture**)arg1)->disp_pic_poc < (*(DisplayPicture**)arg2)->disp_pic_poc)
        return -1;
    if ( (*(DisplayPicture**)arg1)->disp_pic_poc > (*(DisplayPicture**)arg2)->disp_pic_poc)
        return 1;
    else
        return 0;
}

int compare_pic_by_is_flushed_desc( const void *arg1, const void *arg2 )
{
    if ( (*(DisplayPicture**)arg1)->is_flushed < (*(DisplayPicture**)arg2)->is_flushed)
        return 1;
    if ( (*(DisplayPicture**)arg1)->is_flushed > (*(DisplayPicture**)arg2)->is_flushed)
        return -1;
    else
        return 0;
}

int32_t store_picture_in_dpb(void * ptDecHandle)
{
	DECODER * dec = (DECODER *)ptDecHandle;
    struct_slice_header *ssh = dec->ssh;	
	DisplayPicture **disp_pic_data = dec->disp_pic_data;
	DisplayPicture *disp_pic;
	int framesize, idx, flushed_pic_num=0, max_flush_count=0;
	int idr_flag;
	
	uint8_t *srcptr,*destptr;
	int i,linewidth;	
	
	printk("store_picture_in_dpb\n");
	
	if( dec->output_fmt == OUTPUT_FMT_YUV420) {
		framesize = dec->video_width*dec->video_height;
	} else {
		framesize = dec->video_width*dec->video_height*2;
	}

	// flush all frames in dpb, (mmco_5 or IDR with output_of_prior_pics)       	
	idr_flag = (dec->nal_unit_type==5) && (ssh->no_output_of_prior_pics_flag==0);
	if (dec->last_has_mmco_5 || idr_flag)	{
		for(idx=0; idx<dec->dpb_used_size; idx++)	{
			 if(idx==dec->disp_pos)
			 	continue;
			 disp_pic = *(disp_pic_data + idx);	
			 disp_pic->is_flushed += 1;
		}
	}
	
	for(idx=0; idx<dec->dpb_used_size; idx++) {
		disp_pic = *(disp_pic_data + idx);
		max_flush_count = max(max_flush_count, disp_pic->is_flushed);			  		
	}
	
	// insert frame(set poc) into dpb	
	disp_pic = *(disp_pic_data + dec->disp_pos);	
	disp_pic->disp_pic_poc = dec->framepoc;
	
	// get smallest poc and flush out
	if(dec->dpb_used_size==dec->dpb_size) {
       	qsort((void *)(disp_pic_data), dec->dpb_size, sizeof(DisplayPicture*), compare_pic_by_is_flushed_desc);
		for(idx=0; idx<dec->dpb_used_size; idx++) {
			disp_pic = *(disp_pic_data + idx);
			if(disp_pic->is_flushed ==max_flush_count)
			  flushed_pic_num++;
			else
			  break;  
		}
		if(flushed_pic_num>0)
       		qsort((void *)(disp_pic_data), flushed_pic_num, sizeof(DisplayPicture*), compare_pic_by_poc_asc);
		else
       		qsort((void *)(disp_pic_data), dec->dpb_size, sizeof(DisplayPicture*), compare_pic_by_poc_asc);


		//output to LCD display buffer, reset the valid position
		disp_pic = *(disp_pic_data);
		disp_pic->is_flushed = 0;	
		// KC : Copy decoded data from reorder display buffer to VPOST buffer in dbp (!!important)

	// Copy Planar Y or PacketYUV
		srcptr =  (uint8_t *)disp_pic->disp_idx_yaddr_phy;
		destptr = (uint8_t *)dec->output_base_phy;
		for (i=0;i<dec->video_height;i++)
		{
			memcpy((uint8_t *)((unsigned int)destptr|(unsigned int)CACHE_BIT31)
				  ,(uint8_t *)((unsigned int)srcptr |(unsigned int)CACHE_BIT31),dec->video_width * 2);
			destptr += dec->u32FrameBufferWidth*2;
			srcptr += dec->video_width*2;
		}
		 
#if DBG_DPB_MSG			 
		Console_Printf( "In dpb %x <- %x  size =%x\n", (unsigned int)dec->output_base_phy | (unsigned int)CACHE_BIT31, 
						(unsigned int)disp_pic->disp_idx_yaddr_phy | (unsigned int)CACHE_BIT31, framesize  );
#endif						
		if ( dec->output_fmt == OUTPUT_FMT_YUV420) {
			linewidth = dec->video_width/2;
			
			srcptr =  (uint8_t *)disp_pic->disp_idx_yaddr_phy +framesize;
			destptr = (uint8_t *)dec->output_base_u_phy;
			
			for (i=0;i<dec->video_height/2;i++)
			{
				memcpy((uint8_t *)((unsigned int)destptr|(unsigned int)CACHE_BIT31)
					  ,(uint8_t *)((unsigned int)srcptr |(unsigned int)CACHE_BIT31),linewidth);
					  
				destptr += dec->u32FrameBufferWidth/2;
				srcptr += linewidth;
			}		
#if DBG_DPB_MSG						
			Console_Printf( "In dpb %x <- %x  size =%x\n", (unsigned int)CACHE_BIT31 | (unsigned int)dec->output_base_u_phy, 
						(unsigned int)CACHE_BIT31 | (unsigned int)(disp_pic->disp_idx_yaddr_phy +framesize), framesize/4  );			
#endif					
			srcptr =  (uint8_t *)disp_pic->disp_idx_yaddr_phy +framesize*5/4;
			destptr = (uint8_t *)dec->output_base_v_phy;			
				
			for (i=0;i<dec->video_height/2;i++)
			{
				memcpy((uint8_t *)((unsigned int)destptr|(unsigned int)CACHE_BIT31)
					  ,(uint8_t *)((unsigned int)srcptr |(unsigned int)CACHE_BIT31),linewidth);
					  
				destptr += dec->u32FrameBufferWidth/2;
				srcptr += linewidth;
			}		
#if DBG_DPB_MSG					   
			Console_Printf( "In dpb %x <- %x  size =%x\n", (unsigned int)CACHE_BIT31 | (unsigned int)dec->output_base_v_phy, 
						(unsigned int)CACHE_BIT31 | (unsigned int)(disp_pic->disp_idx_yaddr_phy +framesize*5/4), framesize/4  );				   
#endif						
		}
				
		dec->disp_pos = 0;
		dec->is_dpb_full = 1;
		return 1;
	} else {
		dec->disp_pos++;
		dec->dpb_used_size++;
		return 0;
	}		
}

int32_t flush_one_pic_from_dpb(void * ptDecHandle)
{
	DECODER * dec = (DECODER *)ptDecHandle;	
	DisplayPicture **disp_pic_data = dec->disp_pic_data;
	DisplayPicture *disp_pic;
	int32_t framesize, idx, flushed_pic_num=0, max_flush_count=0;
	
	uint8_t *srcptr,*destptr;
	int i,linewidth;	
	
	printk("flush_one_pic_from_dpb\n");
		
	if(dec->is_dpb_full==0) {// dpb is not fullfilled
	  dec->disp_pos = 0;	  
	  dec->dpb_used_size--;	  
	  dec->is_dpb_full = 1;
	} else  
	  dec->disp_pos++;
  	
	// sorting for sequential output
    qsort((void *)(disp_pic_data + dec->disp_pos), dec->dpb_used_size-1, sizeof(DisplayPicture*), compare_pic_by_is_flushed_desc);
	
	for(idx= dec->disp_pos; idx<(dec->disp_pos+dec->dpb_used_size-1); idx++){
		disp_pic = *(disp_pic_data + idx);
		max_flush_count = max(max_flush_count, disp_pic->is_flushed);			  		
	}

	for(idx= dec->disp_pos; idx<(dec->disp_pos+dec->dpb_used_size-1); idx++){
		disp_pic = *(disp_pic_data + idx);
		if(disp_pic->is_flushed ==max_flush_count)
		  	flushed_pic_num++;
		else
		  	break;  
	}
    qsort((void *)(disp_pic_data + dec->disp_pos), flushed_pic_num, sizeof(DisplayPicture*), compare_pic_by_poc_asc);
	
	// copy content to display buffer
	disp_pic = *(disp_pic_data + dec->disp_pos);		
	if ( dec->output_fmt == OUTPUT_FMT_YUV420) {
		framesize = dec->video_width*dec->video_height;
	} else {
		framesize = dec->video_width*dec->video_height*2;
	}
	
//sysFlushCache(D_CACHE);
	// Copy Planar Y or PacketYUV
	srcptr =  (uint8_t *)disp_pic->disp_idx_yaddr_phy;
	destptr = (uint8_t *)dec->output_base_phy;
	
	if( dec->output_fmt == OUTPUT_FMT_YUV420)
		linewidth = dec->video_width;
	else 	
		linewidth = dec->video_width*2;
		
	for (i=0;i<dec->video_height;i++)
	{
		memcpy((uint8_t *)((unsigned int)destptr|(unsigned int)CACHE_BIT31)
			  ,(uint8_t *)((unsigned int)srcptr |(unsigned int)CACHE_BIT31),linewidth);
			  
		if( dec->output_fmt == OUTPUT_FMT_YUV420)
			destptr += dec->u32FrameBufferWidth;
		else
			destptr += dec->u32FrameBufferWidth*2;
		srcptr += linewidth;
	}

#if DBG_DPB_MSG		   
	Console_Printf( "In dpb %x <- %x  size =%x\n", (unsigned int)dec->output_base_phy | (unsigned int)CACHE_BIT31, 
						(unsigned int)disp_pic->disp_idx_yaddr_phy | (unsigned int)CACHE_BIT31, framesize  );	
#endif						
	
	if( dec->output_fmt == OUTPUT_FMT_YUV420) {
		linewidth = dec->video_width/2;
		
		srcptr =  (uint8_t *)disp_pic->disp_idx_yaddr_phy +framesize;
		destptr = (uint8_t *)dec->output_base_u_phy;
		
		for (i=0;i<dec->video_height/2;i++)
		{
			memcpy((uint8_t *)((unsigned int)destptr|(unsigned int)CACHE_BIT31)
				  ,(uint8_t *)((unsigned int)srcptr |(unsigned int)CACHE_BIT31),linewidth);
				  
			destptr += dec->u32FrameBufferWidth/2;
			srcptr += linewidth;
		}	
		//memcpy((uint8_t *)((unsigned int)dec->output_base_u_phy | (unsigned int)CACHE_BIT31), 
		// 	   (uint8_t *)((unsigned int)CACHE_BIT31 | (unsigned int)(disp_pic->disp_idx_yaddr_phy +framesize)), framesize/4);
#if DBG_DPB_MSG		 	   
		Console_Printf( "In dpb %x <- %x  size =%x\n", (unsigned int)dec->output_base_u_phy | (unsigned int)CACHE_BIT31, 
						(unsigned int)CACHE_BIT31 | (unsigned int)(disp_pic->disp_idx_yaddr_phy +framesize), framesize  );		 	   
#endif						
		srcptr =  (uint8_t *)disp_pic->disp_idx_yaddr_phy +framesize*5/4;
		destptr = (uint8_t *)dec->output_base_v_phy;			
			
		for (i=0;i<dec->video_height/2;i++)
		{
			memcpy((uint8_t *)((unsigned int)destptr|(unsigned int)CACHE_BIT31)
				  ,(uint8_t *)((unsigned int)srcptr |(unsigned int)CACHE_BIT31),linewidth);
				  
			destptr += dec->u32FrameBufferWidth/2;
			srcptr += linewidth;
		}						
		//memcpy((uint8_t *)((unsigned int)dec->output_base_v_phy | (unsigned int)CACHE_BIT31), 
		//		(uint8_t *)((unsigned int)CACHE_BIT31 | (unsigned int)(disp_pic->disp_idx_yaddr_phy +framesize*5/4)), framesize/4);		
#if DBG_DPB_MSG				
		Console_Printf( "In dpb %x <- %x  size =%x\n", (unsigned int)dec->output_base_v_phy | (unsigned int)CACHE_BIT31, 
						(unsigned int)CACHE_BIT31 | (unsigned int)(disp_pic->disp_idx_yaddr_phy +framesize*5/4), framesize  );					
#endif						
	}
		
	dec->dpb_used_size--;	
		
	if((dec->disp_pos == (dec->dpb_size-1)) || (dec->dpb_used_size==0))
		return -1; // finish flush_dpb (last flush frame)
	else
		return 1;
}
#endif



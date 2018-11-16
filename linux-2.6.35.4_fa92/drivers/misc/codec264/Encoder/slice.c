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
#include "wblib.h"
#endif

#include "port.h"
//#include "h264_reg.h"
#include "h264dec.h"

#include "register.h"
#include "encoder.h"
#include "slice.h"
#include "sequence.h"
#include "common.h"
#include "bs.h"
#include "user_define.h"

void start_nalu_header(h264_encoder *pEnc)
{
  // byte stream NAL unit syntax with emulation_prevention_three_byte disabled
  U(0, 8); // 8  bits , additional zero byte 'leading_zero_8bits' if it is parameter sets and first slice in picture
  U(1,24); // 24 bits , start code prefix
  
  // start to encode SLICE NALU header with emulation_prevention_three_byte disabled
  if(pEnc->very_first_flag)
    { U(((0<<7)|(NALU_PRIORITY_HIGHEST<<5)|NALU_TYPE_IDR),8); } // u(8) , (forbidden_zero_bit | nal_ref_idc | nal_unit_type)
  else  
    { U(((0<<7)|(NALU_PRIORITY_HIGH<<5)|NALU_TYPE_SLICE),8); }// u(8) , (forbidden_zero_bit | nal_ref_idc | nal_unit_type)
}

unsigned int slice_type(h264_encoder *pEnc)
{
    unsigned int type;
    if ( pEnc->mEncParam.intra == -1) {
	  if ( (pEnc->iframe == 0)
	  	||((pEnc->mEncParam.u32IPInterval > 0) && (pEnc->iframe >= pEnc->mEncParam.u32IPInterval))  ) {
	  		//printk("a. I_SLICE %d fr %d\n", pEnc->iframe, pEnc->frame_num);
    		type = I_SLICE;
			pEnc->frame_num = 0;
			pEnc->very_first_flag = 1;
  	  } else {
  	        //printk("b. P_SLICE %d fr %d\n", pEnc->iframe, pEnc->frame_num);
    		type = P_SLICE;
  	  }
     } else if ( pEnc->mEncParam.intra == 1) {
           	//printk("c. I_SLICE %d fr %d\n", pEnc->iframe, pEnc->frame_num);
           	//pEnc->iframe=0;
	  	   	type = I_SLICE;
           	pEnc->frame_num = 0;	
	  	   	pEnc->very_first_flag =1;	   
     } else if ( pEnc->mEncParam.intra == 0){
           	//printk("d. P_SLICE %d fr %d\n", pEnc->iframe, pEnc->frame_num);
	  		type = P_SLICE;
     } else {
           	Console_Printf("EncParam.Intra assigned value err and default set to P_SLICE\n");
	  		type = P_SLICE;	  
     
     }
      return type;	 
}

void start_slice(h264_encoder *pEnc, int last)
{
  unsigned int log2_max_frame_num_minus4;  
  unsigned int log2_max_pic_order_cnt_lsb_minus4;
  signed int slice_qp_delta;
  unsigned int fnum;

  #if 0
  // TC.Kuo 2007.10.22 -->
  if ( pEnc->mEncParam.intra == -1) {
	  if ( (pEnc->iframe == 0)
	  	||((pEnc->mEncParam.u32IPInterval > 0) && (pEnc->iframe >= pEnc->mEncParam.u32IPInterval))  ) {
	  	//printk("a. I_SLICE %d fr %d\n", pEnc->iframe, pEnc->frame_num);
    		pEnc->slice_type = I_SLICE;
  	  } else {
  	         //printk("b. P_SLICE %d fr %d\n", pEnc->iframe, pEnc->frame_num);
    		pEnc->slice_type = P_SLICE;
  	  }
  } else if ( pEnc->mEncParam.intra == 1) {
                  //printk("c. I_SLICE %d fr %d\n", pEnc->iframe, pEnc->frame_num);
                  //pEnc->iframe=0;
		pEnc->slice_type = I_SLICE;
  } else if ( pEnc->mEncParam.intra == 0){
                  //printk("d. P_SLICE %d fr %d\n", pEnc->iframe, pEnc->frame_num);
		pEnc->slice_type = P_SLICE;
  } else {
         Console_Printf("EncParam.Intra assigned value err and default set to P_SLICE\n");
	pEnc->slice_type = P_SLICE;	  
  }
  // TC.Kuo 2007.10.22 --<
  #endif
  #if 0
  // byte stream NAL unit syntax with emulation_prevention_three_byte disabled
  U(0, 8); // 8  bits , additional zero byte 'leading_zero_8bits' if it is parameter sets and first slice in picture
  U(1,24); // 24 bits , start code prefix
  
  // start to encode SLICE NALU header with emulation_prevention_three_byte disabled
  if(pEnc->very_first_flag)
    { U(((0<<7)|(NALU_PRIORITY_HIGHEST<<5)|NALU_TYPE_IDR),8); } // u(8) , (forbidden_zero_bit | nal_ref_idc | nal_unit_type)
  else  
    { U(((0<<7)|(NALU_PRIORITY_HIGH<<5)|NALU_TYPE_SLICE),8); }// u(8) , (forbidden_zero_bit | nal_ref_idc | nal_unit_type)
  #endif
  
  // start to encode the slice header
  //UE_RBSP(first_mb); // ue(v),"SH: first_mb_in_slice" , if  on picture has one slice, we set it to zero
  if( pEnc->first_mb < 4096 ) {
	  UE_RBSP(pEnc->first_mb);
  } else {
	int zero;
	unsigned int value=0;
     
	zero = zero_M(pEnc->first_mb);
	value = pEnc->first_mb + 1 - (0x1 << 12);
    value |= (0x1 << 12 );
    U(value, zero+zero+1);
  }
	
  UE_RBSP(pEnc->slice_type+5); // ue(v),"SH: slice_type" , to add 5 in order to signal that the whole picture has the same slice type

  
  // the original value was obtained from the active_pps->pic_parameter_set_id , but since we 
  // don't support multiple parameter sets, so we set it to zero.
  UE_RBSP(pEnc->pps.pic_parameter_set_id); // ue(v),"SH: pic_parameter_set_id"

  // it was obtained from Log2MaxFrameNum & input->no_frames
  // we set the Log2MaxFrameNum to 0 and the log2_max_frame_num_minus4 will be equal to 0
  //if (input->Log2MaxFrameNum < 4)
    //log2_max_frame_num_minus4 = MAX((int)(CeilLog2(pEnc->mEncParam.no_frames))-4,0);
  //else 
    //log2_max_frame_num_minus4 = input->Log2MaxFrameNum - 4;      
  log2_max_frame_num_minus4 = 0; // (4-4)
  //#define IMG_NUMBER (img->number-start_frame_no_in_this_IGOP)
  //img->frame_num = (input->intra_period && input->idr_enable ? IMG_NUMBER % input->intra_period : IMG_NUMBER) % (1 << (log2_max_frame_num_minus4 + 4)); 
  fnum = pEnc->frame_num % (1 << (log2_max_frame_num_minus4 + 4));
  U_RBSP(fnum,log2_max_frame_num_minus4+4); // u(v),"SH: frame_num"
  
  // if it is IDR  ???
  if(pEnc->very_first_flag) {
    UE_RBSP(0); // ue(v),"SH: idr_pic_id"
  }
  

    // reference from X264's code
    //if( h->sps->i_poc_type == 0 )
    //{
    //    h->sh.i_poc_lsb = h->fdec->i_poc & ( (1 << h->sps->i_log2_max_poc_lsb) - 1 );
    //    h->sh.i_delta_poc_bottom = 0;   /* XXX won't work for field */
    // }
    
    // if( sh->sps->i_poc_type == 0 )
    // {
    //   bs_write( s, sh->sps->i_log2_max_poc_lsb, sh->i_poc_lsb );
    //   if( sh->pps->b_pic_order && !sh->b_field_pic )
    //     {
    //       bs_write_se( s, sh->i_delta_poc_bottom );
    //     }
    // }
    log2_max_pic_order_cnt_lsb_minus4 = pEnc->sps.log2_max_pic_order_cnt_lsb_minus4;
    pEnc->toppoc %= (1 << (log2_max_pic_order_cnt_lsb_minus4 + 4));
    //U_RBSP(toppoc & ~((((unsigned int)(-1)) << (log2_max_pic_order_cnt_lsb_minus4+4))),log2_max_pic_order_cnt_lsb_minus4+4); // u(v),"SH: pic_order_cnt_lsb"  
    U_RBSP(pEnc->toppoc,log2_max_pic_order_cnt_lsb_minus4+4); // u(v),"SH: pic_order_cnt_lsb"  
    if  ( last == 1) { 
	pEnc->toppoc++; pEnc->toppoc++;
    }
  
  if(pEnc->slice_type == P_SLICE) {
    U_RBSP(0,1); // u(1),"SH: num_ref_idx_active_override_flag"
    // ref_pic_list_reordering
    U_RBSP(0,1); // u(1),"SH: ref_pic_list_reordering_flag_l0"
  }
    
  // dec_ref_pic_marking
  if(pEnc->very_first_flag) {
    U_RBSP(0,1); // u(1),"SH: no_output_of_prior_pics_flag"
    U_RBSP(0,1); // u(1),"SH: long_term_reference_flag"
  } else {
    U_RBSP(0,1); // u(1),"SH: adaptive_ref_pic_buffering_flag"
  }
  
  // (currSlice->qp - 26 - active_pps->pic_init_qp_minus26)
  slice_qp_delta = (pEnc->mEncParam.u32Quant-26-0);
  SE_RBSP(slice_qp_delta); // se(v),"SH: slice_qp_delta"
}

void terminate_slice(h264_encoder *pEnc)
{
  //pEnc->iframe++;
  if(pEnc->mEncParam.u32IPInterval > 0) {
    pEnc->iframe %= pEnc->mEncParam.u32IPInterval;
    pEnc->frame_num %= pEnc->mEncParam.u32IPInterval;
  } else {
    pEnc->iframe = 0;
    pEnc->frame_num = 0;
  }
  
  //pEnc->very_first_flag = 0;
  
}


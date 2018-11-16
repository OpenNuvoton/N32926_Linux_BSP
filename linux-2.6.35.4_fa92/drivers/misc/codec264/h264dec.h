#ifndef	_H264DEC_H_
#define _H264DEC_H_

#ifdef _USE_INT_
	//#define INT_MASK_BIT 0xFFC10000
	#define INT_MASK_BIT 0xFFF30000	
#else
	#define INT_MASK_BIT 0xFFF30000
#endif	

	#define H264_OFF_REF_FRAME 0x100
	
// SW Status Control
	#define FRAME_DONE 0x1
	#define HW_BS_EMPTY 0x2
	#define PREPARE_DECODE 0x4
	#define SLICE_TRIGGER 0x8
	#define SLICE_DONE 0x10
	#define ERROR_BS 0x20

	#define CLEAR_BS_EMPTY 0xFD
	#define CLEAR_SLICE_TRIGGER 0xF7

		
// Register Definition
	//--- 14h INTS
	#define INTS_END_SLICE 0x4
	#define INTS_END_FRAME 0x8
	#define INTS_MV_OVER_RANGE 0x10
	#define INTS_BSM_BUF_EMPTY 0x20
	#define INTS_CLEAR_SLICE 0xFFFFFFFB
	#define INTS_CLEAR_FRAME 0xFFFFFFF3
	#define INTS_CLEAR_BS_EMPTY 0xFFFFFFDF
	#define INTS_CLEAR_MV_OVER 0xFFFFFFEF
	
	//--- 20h SW_RESET
	#define END_OF_STREAM 0x2
	
	//--- 3Ch RECON_LINEOFFS
	#define RECON_Y_UV_LINEOFFS(v) ((uint32_t)(v)<<16) | (uint32_t)(v)
	

	//--- 80h SREG0
   	 // pic_width_in_mbs[7:0], pic_height_in_map_units[7:0], frame_mbs_only_flag,
   	 // mb_aff_flag, direct_8x8_inference_flag, entropy_coding_mode_flag,
   	 // num_ref_idx_l0_active[4:0], num_ref_idx_l1_active[4:0], field_pic_flag, bottom_field_flag
	#define SLICE_INFO0(a,b,c,d,e,f,g,h,i,j) (((uint32_t)(a)<<24) | (((uint32_t)(b)& 0x00FF)<<16) |(((uint32_t)(c)& 1)<<15) |(((uint32_t)(d)& 1)<<14) |(((uint32_t)(e)& 1)<<13) | (((uint32_t)(f)& 1)<<12) |(((uint32_t)(g)& 0x001F)<<7) | (((uint32_t)(h)& 0x001F)<<2) |(((uint32_t)(i) & 1)<<1) | ((uint32_t)(j)& 1)) 
										    

	//--- 84h SREG1
  	 // dsd.slice_type[1:0], dsd.frame_num[4:0], dsd.weighted_pred_flag, 
  	 // dsd.weighted_bipred_idc[1:0], dsd.qp[5:0],
   	 // dsd.chroma_qp_index_offset[4:0], dsd.constrained_intra_pred_flag 
	#define SLICE_INFO1(a,b,c,d,e,f,g) ( (((uint32_t)(a)& 3)<<20) | (((uint32_t)(b)& 0x001F)<<15) |(((uint32_t)(c)& 1)<<14) |(((uint32_t)(d)& 3)<<12) | (((uint32_t)(e)& 0x003F)<<6) |(((uint32_t)(f)& 0x1F)<<1) | ((uint32_t)(g)& 1)) 


	//--- 88h SREG2
  	 // dsd.model_number, dsd.lf_disable_idc,
  	 // dsd.lf_alpha_c0_offset, dsd.lf_beta_offset
	#define SLICE_INFO2(a,b,c,d,e) ((((uint32_t)(a)& 0x1F)<<12) | (((uint32_t)(b)& 3)<<10) | (((uint32_t)(c)& 3)<<8) |(((uint32_t)(d)& 0xF)<<4) |((uint32_t)(e)& 0xF))

	//--- 8Ch ADDR_BSM_CTL
	#define m_sys_parser_ctl(v) ((uint32_t)(v)<<8)
	#define SYS_BSM_IDLE 0
	#define SYS_BSM_SFT 1
	#define SYS_BSM_BTAL 2
	#define SYS_BSM_UVLD 3
	#define SYS_BSM_NSC 4
	#define SYS_BSM_RBSPTB 5
	#define SYS_CONT_DEC 6

	//--- 90h ADDR_STATUS0
	#define m_isParserBusy(v) (((uint32_t)(v)&0x0001)==0) // bit[0], paser_idle!=0

	//--- 94h STREAM_STATUS
	#define m_isEndOfDec(v) ((uint32_t)(v)&0x0020) // bit[5], bsb_le16==1

	//--- a0h ADDR_DECODE_CTL0
	#define START_DEC_SLICE 1
	
//	#define	NULL	0
	
//	#define mdelay	sysDelay
	
#endif	//_H264DEC_H_

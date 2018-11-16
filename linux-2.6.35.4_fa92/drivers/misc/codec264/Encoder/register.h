#ifndef _REGISTER_H_
#define _REGISTER_H_

#define INLINE inline

// A macro that defines enumeration values for a bitset
// just supply the start and end bit positions
#define REG_BIT_DEFN(start, end) ((start<<16)|(end-start+1))
 // The bitfield definition for each register
 
enum PARM0_bitfield
{
  PARM0_PIC_WIDTH        = REG_BIT_DEFN( 0, 6),
  PARM0_PIC_HEIGHT       = REG_BIT_DEFN( 8,14),
  PARM0_PIC_MB_COUNT     = REG_BIT_DEFN(16,28)
};
enum PARM1_bitfield
{
  PARM1_CHROMA_QP_OFFSET = REG_BIT_DEFN( 0, 6),
  PARM1_ALPHA_OFFSET     = REG_BIT_DEFN( 8,11),
  PARM1_BETA_OFFSET      = REG_BIT_DEFN(12,15),
  PARM1_LCT              = REG_BIT_DEFN(16,18),
  PARM1_CCT              = REG_BIT_DEFN(20,22),
  PARM1_TD               = REG_BIT_DEFN(23,23),
  PARM1_IMODE            = REG_BIT_DEFN(24,24)   //< 1: intra 9 modes   0: intra 5 modes 
};
enum PARM2_bitfield
{
  PARM2_WATERMARK_PATTERN = REG_BIT_DEFN( 0, 31)  //< Watermark initial pattern 
};
enum PARM3_bitfield
{
  PARM3_MASK_0 = REG_BIT_DEFN( 0, 21)  //< Mask 0 fileds 
};
enum PARM4_bitfield
{
  PARM4_ID               = REG_BIT_DEFN( 2, 3),
  PARM4_QP               = REG_BIT_DEFN( 4, 9)
};
enum PARM5_bitfield
{
  PARM5_MASK_1 = REG_BIT_DEFN( 0, 21)  //< Mask 1 fileds 
};

enum PARM6_bitfield
{
  PARM6_MPEG4_2D         = REG_BIT_DEFN( 0, 0)
};
enum PARM7_bitfield
{
  PARM7_EXT_PK_CODE      = REG_BIT_DEFN( 0,31)
};
enum CMD0_bitfield
{
  CMD0_ENCODE            = REG_BIT_DEFN( 0, 0),
  CMD0_SLICE_TYPE        = REG_BIT_DEFN( 1, 1),
  CMD0_WATERMARK_ENABLE  = REG_BIT_DEFN( 4, 5),   //< bit4 : intra mb watermark enable, bit5 : inter mb watermark enable 
  CMD0_RESET	= REG_BIT_DEFN( 31, 31)
};

enum CMD1_bitfield
{
  CMD1_EXT_PK_LEN           = REG_BIT_DEFN( 0, 5),
  CMD1_EXT_PK_GO            = REG_BIT_DEFN( 6, 6),
  CMD1_EXP_GOLOMB_CODE      = REG_BIT_DEFN( 7, 7),
  CMD1_PREVENT_0x3          = REG_BIT_DEFN( 8, 8),
  CMD1_ADD_RBSP_TRAILING    = REG_BIT_DEFN( 9, 9),
  CMD1_CLOSE_BITSTREAM      = REG_BIT_DEFN(10,10),
  CMD1_CLEAR_BITSTREAM_LEN  = REG_BIT_DEFN(11,11)
};
enum STS0_bitfield
{
  STS0_FRAME_DONE        = REG_BIT_DEFN( 0, 0),
  STS0_VLC_DONE          = REG_BIT_DEFN( 1, 1)
};
enum STS1_bitfield
{
  STS1_BITSTREAM_LEN     = REG_BIT_DEFN( 0,31)
};
enum STS2_bitfield
{
  STS2_FD_INT_MASK = REG_BIT_DEFN( 16,16),
  STS2_FD_INT_STS = REG_BIT_DEFN( 0 , 0 )	
};
enum STS3_bitfield
{
  STS3_FRAME_COST        = REG_BIT_DEFN( 0,31)
};
enum DMATH_bitfield
{
  DMATH_ISSUE_HALF_EMPTY = REG_BIT_DEFN( 0, 0),
  DMATH_ISSUE_HALF_FULL  = REG_BIT_DEFN( 1, 1)
};
enum DMAA0_bitfield
{
  DMAA0_SMBA             = REG_BIT_DEFN( 2,31)
};
enum DMAC0_bitfield
{
  DMAC0_TRANSFER_LEN     = REG_BIT_DEFN( 0,11),
  DMAC0_TRANSFER_DIR     = REG_BIT_DEFN(15,15)
};
enum DMAA1_bitfield
{
  DMAA1_INCREMENT        = REG_BIT_DEFN( 0, 1),
  DMAA1_SMBA             = REG_BIT_DEFN( 2,31)
};
enum DMAC1_bitfield
{
  DMAC1_LENGTH           = REG_BIT_DEFN( 0,11),
  DMAC1_TRANSFER_TYPE    = REG_BIT_DEFN(12,13),
  DMAC1_TRANSFER_DIR     = REG_BIT_DEFN(15,15),
  DMAC1_SLOT_COUNTER     = REG_BIT_DEFN(16,24)
};
enum DMAW1_bitfield
{
  DMAW1_SLOT_HALF_AMOUNT = REG_BIT_DEFN(24,31),
  DMAB1_CBBA             = REG_BIT_DEFN( 2,31)
};

enum GSMBA_bitfield
{
  GSMBA_INCREMENT        = REG_BIT_DEFN( 0, 1),  //< increment index 
  GSMBA_SMBA             = REG_BIT_DEFN( 2, 31)   ///< system memory base address (byte) 
};

enum GCTRL_bitfield
{
  GCTRL_SLOT_TRANSFER_LEN  = REG_BIT_DEFN( 0,11),  //< slot transfer length (word) 
  GCTRL_STYPE              = REG_BIT_DEFN(12,13),  //< system memory data type 
  GCTRL_ROI_STYPE          = REG_BIT_DEFN(14,14),  //< ROI realted system memory data type 
  GCTRL_DIR                = REG_BIT_DEFN(15,15),  //< transfer direction 
  GCTRL_SLOT_COUNTER       = REG_BIT_DEFN(16,24)   //< slot counter 
};

enum GBKWI_bitfield
{
  GBKWI_SYSMEM_BLOCK_WIDTH = REG_BIT_DEFN( 0, 7),  //< system memory block width (2-words) 
  GBKWI_SYSMEM_LINE_OFFSET = REG_BIT_DEFN( 8,23),  //< system memory line offset (word) 
  GBKWI_HALF_SLOT          = REG_BIT_DEFN(24,31)   //< half of slot amount 
};

enum GCBB_bitfield
{
  GCBB_CIRCULAR_BUF_BASEADR = REG_BIT_DEFN( 2,31)  //< circular buffer base address 
};

// DMAC control registers group 2 releated fields definition
enum DMATH_2_bitfield
{
  DMATH_2_ISSUE_HALF_EMPTY = REG_BIT_DEFN( 0, 0),
  DMATH_2_ISSUE_HALF_FULL  = REG_BIT_DEFN( 1, 1)
};
enum DMAA0_2_bitfield
{
  DMAA0_2_SMBA             = REG_BIT_DEFN( 2,31)
};
enum DMAC0_2_bitfield
{
  DMAC0_2_TRANSFER_LEN     = REG_BIT_DEFN( 0,11),
  DMAC0_2_TRANSFER_DIR     = REG_BIT_DEFN(15,15)
};
enum DMAA1_2_bitfield
{
  DMAA1_2_INCREMENT        = REG_BIT_DEFN( 0, 1),
  DMAA1_2_SMBA             = REG_BIT_DEFN( 2,31)
};
enum DMAC1_2_bitfield
{
  DMAC1_2_LENGTH           = REG_BIT_DEFN( 0,11),
  DMAC1_2_TRANSFER_TYPE    = REG_BIT_DEFN(12,13),
  DMAC1_2_TRANSFER_DIR     = REG_BIT_DEFN(15,15),
  DMAC1_2_SLOT_COUNTER     = REG_BIT_DEFN(16,24)
};
enum DMAW1_2_bitfield
{
  DMAW1_2_SLOT_HALF_AMOUNT = REG_BIT_DEFN(24,31),
  DMAB1_2_CBBA             = REG_BIT_DEFN( 2,31)
};
enum GSMBA_2_bitfield
{
  GSMBA_2_INCREMENT        = REG_BIT_DEFN( 0, 1), 
  GSMBA_2_SMBA             = REG_BIT_DEFN( 2, 31)  
};
enum GCTRL_2_bitfield
{
  GCTRL_2_SLOT_TRANSFER_LEN  = REG_BIT_DEFN( 0,11),  
  GCTRL_2_STYPE              = REG_BIT_DEFN(12,13),  
  GCTRL_2_ROI_STYPE          = REG_BIT_DEFN(14,14), 
  GCTRL_2_DIR                = REG_BIT_DEFN(15,15),  
  GCTRL_2_SLOT_COUNTER       = REG_BIT_DEFN(16,24)   
};
enum GBKWI_2_bitfield
{
  GBKWI_2_SYSMEM_BLOCK_WIDTH = REG_BIT_DEFN( 0, 7),  
  GBKWI_2_SYSMEM_LINE_OFFSET = REG_BIT_DEFN( 8,23),  
  GBKWI_2_HALF_SLOT          = REG_BIT_DEFN(24,31)   
};
enum GCBB_2_bitfield
{
  GCBB_2_CIRCULAR_BUF_BASEADR = REG_BIT_DEFN( 2,31)  
};

#undef REG_BIT_DEFN


static __inline unsigned int regbitRead(unsigned int reg, unsigned int bits)
{
  volatile unsigned int       regval = inp32(reg);  
  const unsigned int width  = bits & 0xff;
  const unsigned int bitno  = bits >> 16;
  regval >>= bitno;
  regval  &= ((1<<width)-1);
  return regval;
}

static __inline void regbitWrite(unsigned int reg, unsigned int bits, unsigned int value)
{
  volatile unsigned int regval;
  const unsigned int     width  = bits & 0xff;
  const unsigned int     bitno  = bits >> 16;
  
  regval = inp32(reg);  
  regval &= ~(((1<<width)-1) << bitno);
  regval |=  value << bitno;
  outp32(reg, regval);
}

// the macros exported for user to do register access and form register value
#define REG_READ(reg)                     inp32(reg)
#define REG_WRITE(reg,value)              outp32(reg,value)
#define REG_BITREAD(reg,bitfield)         regbitRead((unsigned int)reg,bitfield)
#define REG_BITWRITE(reg,bitfield,value)  regbitWrite((unsigned int)reg,bitfield,value)
// A macro that forms each field value according to its bitfield name and value
#define REG_BITVAL(bitfield_name,value) ((unsigned int)value<<(bitfield_name>>16))


// the macros for user to do bitstream packing
#define BITSTREAM_PUTBITS(value,size,exp_golomb,prevent_0x3) \
          outp32(REG_264_PARM7,(value)); \
          outp32(REG_264_CMD1 ,  REG_BITVAL(CMD1_PREVENT_0x3,prevent_0x3) \
                        | REG_BITVAL(CMD1_EXP_GOLOMB_CODE,exp_golomb) \
                        | REG_BITVAL(CMD1_EXT_PK_GO,1) \
                        | REG_BITVAL(CMD1_EXT_PK_LEN,(size)) \
                 )
/** to pack unsigned integer using 'size' bits with no 'emulation_prevention_three_byte' enabled */
#define U(value,size)            BITSTREAM_PUTBITS((value),(size),0,0)

// In RBSP, we need to add 'emulation_prevention_three_byte'. So we add an '_RBSP' suffix
// to denote these RBSP properties.
#define U_RBSP(value,size)       BITSTREAM_PUTBITS((value),(size),0,1)
#define UE_RBSP(value)           BITSTREAM_PUTBITS((value),0,1,1)  // for Exp-Golomb Coding, we don't care the size field
// for Exp-Golomb Coding, we don't care the size field and we need to convert the value to 'code_num' in the first place
// code_num = 2|k|   for (k<=0)
// code_num = 2|k|-1 for (k>0)
#define SE_RBSP(value)           BITSTREAM_PUTBITS(((value)>0)? (((value)<<1)-1):((-(value))<<1),0,1,1) 

#define RBSP_TRAILING_BITS()     REG_BITWRITE(REG_264_CMD1,CMD1_ADD_RBSP_TRAILING,1)

// define various kind of macros to perform specific operation
#define CHECK_FRAME_DONE()       REG_BITREAD(REG_264_STS0,STS0_FRAME_DONE)
#define CHECK_VLC_DONE()         REG_BITREAD(REG_264_STS0,STS0_VLC_DONE)
#define CLEAR_BITSTREAM_LENGTH() REG_BITWRITE(REG_264_CMD1,CMD1_CLEAR_BITSTREAM_LEN,1)
#define CLOSE_BITSTREAM()        REG_BITWRITE(REG_264_CMD1,CMD1_CLOSE_BITSTREAM,1)

#endif


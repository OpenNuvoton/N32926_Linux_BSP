/*****************************************************************************
 * bs.h :
 *****************************************************************************/
#ifndef _BS_HEADER_
#define _BS_HEADER_

#define boolean int
#define int8_t   char
#define uint8_t  unsigned char
#define int16_t  short
#define uint16_t unsigned short
#define int32_t  int
#define uint32_t unsigned int
#define int64_t  __int64
#define uint64_t unsigned __int64
#define bool int

typedef struct bs_s
{
    uint8_t *p_start;
    uint8_t *p;
    uint8_t *p_end;

    int     i_left;    /* i_count number of available bits */
    int     i_bits_encoded; /* RD only */
} bs_t;

typedef struct bs_out_s
{
   uint8_t *pt;
   int32_t idx;
} bs_out_t;

void bs_init( bs_t *s, void *p_data, int i_data );
uint32_t bs_read( bs_t *s, int i_count );
uint32_t bs_read1( bs_t *s );

void bs_write( bs_t *s, int i_count, uint32_t i_bits );
void bs_write1( bs_t *s, uint32_t i_bit );
void bs_align_1( bs_t *s );
void bs_align( bs_t *s );
void bs_write_ue( bs_t *s, unsigned int val );
void bs_write_se( bs_t *s, int val );
void bs_write_te( bs_t *s, int x, int val );
void bs_rbsp_trailing( bs_t *s );
int bs_size_ue( unsigned int val );
int bs_size_se( int val );
int bs_size_te( int x, int val );

int zero_M( unsigned int val );

#endif //_BS_HEADER_


//--------------------------------------------------------------
//
// Copyright (c) Nuvoton Technology Corp. All rights reserved.
//
//--------------------------------------------------------------

#ifndef __W55FA92_AAC_H__
#define __W55FA92_AAC_H__

#include <linux/ioctl.h>
#include <linux/types.h>

#ifdef	__cplusplus
extern "C"
{
#endif

// TO BE CONTINUED
typedef struct {
   int32_t *inbuf;
   int32_t *outbuf;
   int32_t i32Size;
} aac_enc_ctx_s;
// For AAC encoder, pass the parameters

// TO BE CONTINUED
typedef struct {
   int32_t i32Size;
   int32_t *inbuf;
   int32_t *outbuf;
} aac_dec_ctx_s;
// For AAC decoder, pass the parameters

#define AAC_IOC_MAGIC 'u'
#define AAC_IOC_MAXNR 200

/* ===========================================================
	S means "Set" through a ptr
	T means Tell" directly with the argument value
	G means "Get": reply by setting through a pointer
	Q means "Query": response is on the return value
	X means "eXchange": switch G and S atomically
	H means "sHift": switch T and Q atomically
  ============================================================ */

#define AAC_IOCSENC			_IOW(AAC_IOC_MAGIC, 1, aac_enc_ctx_s)
#define AAC_IOCSDEC			_IOW(AAC_IOC_MAGIC, 2, aac_dec_ctx_s)
#define AAC_IOCTRIGGER		_IO(AAC_IOC_MAGIC, 3)
#define AAC_IOCWAIT			_IO(AAC_IOC_MAGIC, 4)

#ifdef	__cplusplus
}
#endif

#endif	// __W55FA92_AAC_H__


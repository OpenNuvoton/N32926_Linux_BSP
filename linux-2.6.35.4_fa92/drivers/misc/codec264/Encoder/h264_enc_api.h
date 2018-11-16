/**
 *  @file h264_enc_api.h
 *  @brief The header file for Gm H264 encoder API.
 *
 */
#ifndef _H264_ENC_API_H_
#define _H264_ENC_API_H_

//#include "favc_avcodec.h"
#include <mach/favc_avcodec.h>

void h264_encoder_reinit(void *handle,FAVC_ENC_PARAM *pParam,unsigned int mp4yuv);
void* h264_encoder_init(FAVC_ENC_PARAM *pParam);
void h264_toggle_DMA_buffer(void *handle,int idx);
unsigned int encoder_decide_time_slot(unsigned int width,unsigned int height);
int h264_encoder_encode(void *handle,FAVC_ENC_PARAM *mEncParam,int first_mb, int last, int idx);
void h264_encoder_setyuv_addr(void *handle,unsigned char *yaddr,unsigned char *uaddr,unsigned char *vaddr);
void h264_encoder_destroy(void *handle);
unsigned char *h264_get_sysinfo(void *handle);
void h264_encoder_init_vui(void *handle, FAVC_VUI_PARAM *pVUI);
void h264_encoder_init_crop(void *handle, FAVC_CROP_PARAM *pcrop);
int h264_fmt(void * handle);

#ifdef EVALUATION_PERFORMANCE
void h264_performance_report(void);
#endif

#define IGNORE_FRAME_NO
//#define LARGER_LEVEL
//#define ENABLE_9_INTRA_PREDICTION_MODE
#define MODIFIED_GBKWI_HALF_SLOT
//#define WATER_MARK_ENABLE
//#define OUTPUT_SIZED_BITSTREAM // to output sized byte stream for RTP payload , if not defined, then output annex B byte stream

#define SYSBUF_SIZE(width, height) ( ((width+15)/16)*((height+15)/16)*64)
#define DMA_BUF_WORD 78 //(4*19+2)
#define DMA_BUF_SIZE (DMA_BUF_WORD*sizeof(unsigned int))

#endif

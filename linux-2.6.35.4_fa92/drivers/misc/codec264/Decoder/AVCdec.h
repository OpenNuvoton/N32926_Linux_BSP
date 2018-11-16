#ifndef _AVCDEC_H_
#define _AVCDEC_H_

	#include "port.h"
	#include <mach/favc_avcodec.h>
	
	#define TRUE	1
	#define FALSE	0

	#define FRAME_DONE 0x1
	#define HW_BS_EMPTY 0x2
	#define PREPARE_DECODE 0x4
	#define SLICE_TRIGGER 0x8	
	#define CLR_HW_BS_EMPTY 0xFFFFFFFD
	
	#define AVCDEC_EXT extern

	AVCDEC_EXT int32_t
	AVCDec_Init(FAVC_DEC_PARAM * ptParam, void ** pptDecHandle, unsigned char ndev);
	
    void AVCDec_ReInit(void * ptDecHandle);

	AVCDEC_EXT uint32_t
	AVCDec_QueryEmptyBuffer (void * ptDecHandle);

	AVCDEC_EXT uint32_t
	AVCDec_QueryFilledBuffer (void * ptDecHandle);

	AVCDEC_EXT void
	AVCDec_InvalidBS (void * ptDecHandle);

	AVCDEC_EXT AVC_RET
	AVCDec_FillBuffer(void * ptDecHandle, uint8_t * ptBuf, uint32_t u32BufSize, bool bfitFrameSize);

	AVCDEC_EXT void
	AVCDec_EndOfData (void * ptDecHandle);

	AVCDEC_EXT void
	AVCDec_SetOutputAddr (void * ptDecHandle,
		uint8_t * pu8output_phy, uint8_t * pu8output_u_phy, uint8_t * pu8output_v_phy);

	AVCDEC_EXT AVC_RET
	AVCDec_OneFrame(void * ptDecHandle, FAVC_DEC_RESULT * ptResult, int dev);

	AVCDEC_EXT void
	AVCDec_SetCrop(void * ptDecHandle, int x, int y);

	AVCDEC_EXT int32_t
	AVCDec_Sync_OneFrame(void * ptDecHandle);
	
	AVCDEC_EXT void
	AVCDec_Release(void * ptDecHandle);

	void* nv_malloc(int size, int alignment);
	int nv_free(void* ptr);
	
	AVC_RET AVCDec_UpdateInform(void * ptDecHandle, uint32_t u32BufSize, bool bfitFrameSize);
	
	void decoder_fill_bs_reg(void * ptDecHandle, int size);
	
	void* nv_malloc(int size, int alignment);
	int nv_free(void* ptr);
	
#define DIV_16(n)	((n+0x0000F)/0x00010)*0x00010	

#endif

#ifndef _SLICE_H_
#define _SLICE_H_

#include <mach/favc_avcodec.h>
#include "common.h"
#include "port.h"

void start_nalu_header(h264_encoder *pEnc);
unsigned int slice_type(h264_encoder *pEnc);
void start_slice(h264_encoder *pEnc, int last);
void terminate_slice(h264_encoder *pEnc);

#endif


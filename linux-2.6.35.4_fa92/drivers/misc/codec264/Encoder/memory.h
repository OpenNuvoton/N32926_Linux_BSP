#ifndef _MEMORY_H_
#define _MEMORY_H_
#include "port.h"

void *h264_malloc(unsigned int size,int alignment);
void h264_free(void *mem_ptr);

#endif

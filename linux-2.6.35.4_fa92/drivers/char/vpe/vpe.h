
#ifndef __ASM_ARM_W55FA92_VPE_H
#define __ASM_ARM_W55FA92_VPE_H

#include <linux/semaphore.h>

#define VPE_MAJOR	196
#define VPE_MINOR	0


#if 0
#define FB_SIZE		(320*240*4)
#define MAP_SIZE	(FB_SIZE+1024*1024)
#define MAP_SIZE_ORDER	9
#endif
#define ERRCODE		__s32
#define UINT8			__u8
#define UINT16		__u16
#define UINT32		__u32
#define INT8			__s8
#define INT16			__s16
#define INT32			__s32

//typedef enum
//{	
//	FALSE =0,
//	TRUE =1
//}BOOL;

#define BOOL	 		UINT32 
#define FALSE		0
#define TRUE			1


#define PBOOL		BOOL*
#define PUINT8		UINT8*
#define PUINT16		UINT16*
#define PUINT32		UINT32*
#define Successful	0
#define outp32(addr, value)		outl(value, addr)
#define inp32(addr)			inl(addr)


#define VPE_IOCTL_SET_SRCBUF_ADDR			0
#define VPE_IOCTL_SET_DSTBUF_ADDR			(VPE_IOCTL_SET_SRCBUF_ADDR+2)
#define VPE_IOCTL_SET_FMT						(VPE_IOCTL_SET_DSTBUF_ADDR+2)
#define VPE_IOCTL_SET_SRC_OFFSET				(VPE_IOCTL_SET_FMT+2)
#define VPE_IOCTL_SET_DST_OFFSET				(VPE_IOCTL_SET_SRC_OFFSET+2)
#define VPE_IOCTL_SET_SRC_DIMENSION			(VPE_IOCTL_SET_DST_OFFSET+2)
#define VPE_IOCTL_SET_DST_DIMENSION			(VPE_IOCTL_SET_SRC_DIMENSION+2)
#define VPE_IOCTL_SET_MACRO_BLOCK			(VPE_IOCTL_SET_DST_DIMENSION+2)
#define VPE_IOCTL_SET_COLOR_RANGE			(VPE_IOCTL_SET_MACRO_BLOCK+2)
#define VPE_IOCTL_SET_FILTER					(VPE_IOCTL_SET_COLOR_RANGE+2)
#define VPE_IOCTL_SET_3X3_COEF				(VPE_IOCTL_SET_FILTER+2)
#define VPE_IOCTL_HOST_OP					(VPE_IOCTL_SET_3X3_COEF+2)
#define VPE_IOCTL_TRIGGER					(VPE_IOCTL_HOST_OP+2)
#define VPE_IOCTL_CHECK_TRIGGER				(VPE_IOCTL_TRIGGER+2)
#define VPE_IOCTL_SET_MMU_ENTRY				(VPE_IOCTL_CHECK_TRIGGER+2)
#define VPE_IOCTL_SET_TLB_ENTRY				(VPE_IOCTL_SET_MMU_ENTRY+2)

typedef enum
{
	VPE_INT_COMP=0,
	VPE_INT_PAGE_FAULT,		/* Reserved */
	VPE_INT_PAGE_MISS,		/* Reserved */
	VPE_INT_MB_COMP,
	VPE_INT_MB_ERR,
	VPE_INT_DMA_ERR
}E_VPE_INT_TYPE;

typedef enum vpe_state {
	VPE_CLOSE	= 0,
	VPE_IDLE	= 1,
	VPE_RUNNING	= 2,
	VPE_FINISH	= 3,
	VPE_ERROR	= 9,
} vpe_state_t;

#define IS_FINISH(state)			(state == VPE_FINISH)
#define IS_ERROR(state)		(state == VPE_ERROR)
#define IS_DONE(state)			(IS_FINISH(state)|IS_ERROR(state))

typedef enum vpe_func {
	VPE_NONE	= 0,
	VPE_BLIT		= 1,
	VPE_FILL		= 2,
} vpe_func_t;

typedef struct vpe_priv {
	vpe_state_t			state;
	unsigned int			intr_stat;
	unsigned int 		wait_intr_stat;
	
	unsigned int 		pgd;			/* A pointer for the process */
} vpe_priv_t;

ERRCODE vpeOpen(void);
ERRCODE vpeEnableInt(E_VPE_INT_TYPE eIntType);
ERRCODE vpeIoctl(UINT32 u32Cmd, UINT32 u32Arg0, UINT32 u32Arg1, UINT32 u32Arg2);
ERRCODE vpeClose(void);

#endif//__ASM_ARM_W55FA92_VPE_H



#ifndef __ASM_ARM_W55FA92_ROT_H
#define __ASM_ARM_W55FA92_ROT_H

#include <linux/semaphore.h>
#include <mach/w55fa92_rot.h>

#define ROT_MAJOR	236
#define ROT_MINOR	0

#if 1
#define ERRCODE			__s32
#define UINT8			__u8
#define UINT16			__u16
#define UINT32			__u32
#define INT8			__s8
#define INT16			__s16
#define INT32			__s32
#define BOOL	 		__u32
#define FALSE			0
#define TRUE			1
#define VOID			void

#define PBOOL			BOOL*
#define PUINT8			UINT8*
#define PUINT16			UINT16*
#define PUINT32			UINT32*
#define Successful		0
#define outp32(addr, value)		__raw_writel(value, addr)
#define inp32(addr)				__raw_readl(addr)
#endif


#define ERR_PRINTF		printk
#define DBG_PRINTF(...)		//printk


typedef enum rot_state {
	ROT_CLOSE	= 0,
	ROT_IDLE	= 1,
	ROT_RUNNING	= 2,
	ROT_FINISH	= 3,
} rot_state_t;

#define IS_FINISH(state)	(state == ROT_FINISH)
#define IS_IDLE(state)		(state == ROT_IDLE)
#define IS_RUNNING(state)	(state == ROT_RUNNING)
#define IS_DONE(state)		(IS_FINISH(state)|IS_IDLE(state))

typedef struct rot_priv {
	unsigned int 		u32MemPhyAddr;
	unsigned int 		u32MemVirAddr;
	
	S_ROT*				psRotConf;	
	rot_state_t			state;		
	volatile BOOL 		bIsROTComplete;	
}S_ROT_PRIV;

#define ROT_ERR_ID				0xFFFF2000	/* ROT library ID */
#define ERR_ROT_BUSY       		(ROT_ERR_ID + 1) 	

VOID rotOpen(VOID);
VOID rotClose(VOID);
INT32 rotImageConfig(S_ROT* psRotConf);
INT32 rotTrigger(void);
void rotIntHandler(void);
unsigned int dmamalloc_phy(unsigned long);

#endif//__ASM_ARM_W55FA92_ROT_H


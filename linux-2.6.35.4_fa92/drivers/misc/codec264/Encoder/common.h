#ifndef _COMMON_H_
#define _COMMON_H_

#define SWAP(A,B)    { void * tmp = A; A = B; B = tmp; }

#define _DEBUG



typedef unsigned char byte;    //!< byte type definition

#define MIN(a,b)  (((a)<(b)) ? (a) : (b))//LIZG 28/10/2002
#define MAX(a,b)  (((a)<(b)) ? (b) : (a))//LIZG 28/10/2002

/** 
 * Boolean Type
 */
 
typedef enum {
  bFALSE,
  bTRUE
} Boolean;


unsigned CeilLog2( unsigned uiVal);

#define PIXEL_Y	16
#define PIXEL_U	8
#define PIXEL_V	8
#define SIZE_Y	(PIXEL_Y * PIXEL_Y)
#define SIZE_U	(PIXEL_U * PIXEL_U)
#define SIZE_V	(PIXEL_V * PIXEL_V)

/** Multi slice definitions
 *    LIMIT_MAX_WIDHT and LIMIT_MAX_HEIGHT is depended on skip_run num.
 *    the max skip_run num is 4096.
 */
// user definition start 
#define MAX_WIDTH 2048
#define MAX_HEIGHT 2048
// user definition end

#define LIMIT_MAX_MB 4095		// never bigger than or equal to 4096, hardware limitation
#define SUPPORT_MAX_MSLIC_NUM (((MAX_WIDTH*MAX_HEIGHT/SIZE_Y)+LIMIT_MAX_MB-1)/LIMIT_MAX_MB)

	typedef void (* SEM_WAIT_PTR)(void * pSem);
	typedef void (* SEM_SIGNAL_PTR)(void * pSem);
	typedef void *(* FAVC_DMA_MALLOC_PTR)(uint32_t size, uint16_t align_size,	uint8_t reserved_size, void ** phy_ptr);
	typedef void (* FAVC_DMA_FREE_PTR)(void * virt_ptr, void * phy_ptr);
    typedef void *(* FAVC_MALLOC_PTR)(unsigned int size, unsigned char align_size, unsigned char reserved_size);
    typedef void (* FAVC_FREE_PTR)(void * virt_ptr);


unsigned int phys_to_virt_92(unsigned int phy_addr);
	
#endif

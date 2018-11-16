/****************************************************************
 *                                                              *
 * Copyright (c) Nuvoton Technology Corp. All rights reserved.  *
 *                                                              *
 ****************************************************************/
#include <asm/io.h>
#include <linux/module.h>

#include <mach/w55fa92_reg.h>
#include <mach/videoin.h>	


#define ERR_PRINTF			printk
#define DBG_PRINTF(...)		

unsigned int get_vin_dev1_buffer_size(void)
{
	unsigned int PlanarWidth, 	PlanarHeight, PlanarFormat;
	unsigned int PacketWidth, 	PacketHeight;
	
	unsigned int TotalMem=0;

	PlanarWidth = CONFIG_VIN_DEV1_ENCODE_WIDTH;
	PlanarHeight = CONFIG_VIN_DEV1_ENCODE_HEIGHT;
#ifdef CONFIG_PLANAR_YUV422_YUV420_DEV1
	PlanarFormat = 2;	//Planar YUV422/YUV420.
#else
	PlanarFormat = 1;	//Planar YUV420.
#endif

	if(PlanarFormat==2)
		TotalMem = (PlanarWidth*PlanarHeight*2+4096)*CONFIG_VIN_DEV1_ENCODE_BUF_NO;
	else
		TotalMem = (PlanarWidth*PlanarHeight*3/2+4096)*CONFIG_VIN_DEV1_ENCODE_BUF_NO;		

	PacketWidth = CONFIG_VIN_DEV1_PREVIEW_WIDTH;
	PacketHeight = CONFIG_VIN_DEV1_PREVIEW_HEIGHT;

	TotalMem = TotalMem + (PacketWidth*PacketHeight*2+4096)*CONFIG_VIN_DEV1_PREVIEW_BUF_NO;
#ifdef CONFIG_MOTION_DETECTION_DEV1	
	{
		unsigned int DiffSize = 0;
		if( ((PacketWidth/8)%4) ==0)
			DiffSize = PacketWidth/8*PacketHeight/8;
		else
			DiffSize = (PacketWidth/8 + 4-((PacketWidth/8)%4))*PacketHeight/8;
		DiffSize = (DiffSize+4096)*2;
		TotalMem = TotalMem+DiffSize;
	}	
#endif
	DBG_PRINTF("Total buffer size(Dec) = %d\n", TotalMem);
	DBG_PRINTF("Total buffer size(Hex) = 0x%x\n", TotalMem);
	return TotalMem;
}

EXPORT_SYMBOL(get_vin_dev1_buffer_size);

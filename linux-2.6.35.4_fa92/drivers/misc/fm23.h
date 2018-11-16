#ifndef __FN23_H__

#include <linux/delay.h>

#define IOCTL_SET_FM_STANDBY		_IO('v', 0)		
#define SET_FM_BT_BYPASS		_IO('v', 1)		
#define SET_FM_HANDSET_BYPASS		_IO('v', 2)		
#define SET_FM_HANDSET_NOR		_IO('v', 3)		
#define SET_FM_HANDFREE_NOR		_IO('v', 4)		
#define IOCTL_GET_FM_STATE		_IOR('v', 5, unsigned int*)		


struct ard_denoise_mic_gpios {
	s32 clk;
	s32 en;
	s32 bypass;
	s32 reset;
	s32 pwd;
};

extern int fm23_set_procedure(unsigned commad);
extern int fm23_set_pwd(int com);

#endif

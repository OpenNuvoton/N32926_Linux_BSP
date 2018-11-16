/*
 * Copyright (c) 2009-2010 Nuvoton technology corporation.
 *
 * Wan ZongShun <mcuos.com@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation;version 2 of the License.
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/suspend.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/initval.h>
#include <sound/soc.h>
#include <linux/device.h>
#include <linux/clk.h>

//#include <mach/mfp.h>
//#include <mach/map.h>
//#include <mach/regs-clock.h>

//#include <arch/arm/mach-w55fa92>

#include <mach/w55fa92_audio.h>
#include <mach/w55fa92_i2s.h>
#include <mach/w55fa92_spu.h>
#include <mach/w55fa92_reg.h>


//#include "nuc900-audio.h"


//#define SPU_DEBUG
#define SPU_DEBUG_ENTER_LEAVE
#define SPU_DEBUG_MSG
#define SPU_DEBUG_MSG2

#ifdef SPU_DEBUG
#define DBG(fmt, arg...)			printk(fmt, ##arg)
#else
#define DBG(fmt, arg...)
#endif


#ifdef SPU_DEBUG_ENTER_LEAVE
#define ENTER()					DBG("[%-10s] : Enter\n", __FUNCTION__)
#define LEAVE()					DBG("[%-10s] : Leave\n", __FUNCTION__)
#else
#define ENTER()
#define LEAVE()
#endif

#ifdef SPU_DEBUG_MSG
#define MSG(fmt)					DBG("[%-10s] : "fmt, __FUNCTION__)
#else
#define MSG(fmt)
#endif

#ifdef SPU_DEBUG_MSG2
#define MSG2(fmt, arg...)			DBG("[%-10s] : "fmt, __FUNCTION__, ##arg)
#else
#define MSG2(fmt, arg...)
#endif

//static int s_rightVol = 0, s_leftVol = 0;
static int s_spuInit = 0;

int _bSpuActive = 0;
int _u8Channel0 = 0, _u8Channel1 = 1;
int _bApuVolumeActive = 0;
	
static volatile int _bPlayDmaToggle, _bRecDmaToggle;

int DrvSPU_SetBaseAddress(u32 u32Channel, u32 u32Address);
int DrvSPU_GetBaseAddress(u32 u32Channel);
int DrvSPU_SetThresholdAddress(u32 u32Channel, u32 u32Address);
int DrvSPU_GetThresholdAddress(u32 u32Channel);
int DrvSPU_SetEndAddress(u32 u32Channel, u32 u32Address);
int DrvSPU_GetEndAddress(u32 u32Channel);
int DrvSPU_GetCurrentAddress(u32 u32Channel);


int DrvSPU_SetDFA(u32 u32Channel, u16 u16DFA);
int DrvSPU_GetDFA(u32 u32Channel);
int DrvSPU_SetPAN(u32 u32Channel, u16 u16PAN);	// MSB 8-bit = left channel; LSB 8-bit = right channel
int DrvSPU_SetPauseAddress(u32 u32Channel, u32 u32Address);
int DrvSPU_GetPAN(u32 u32Channel);

static int DrvSPU_EnableInt(u32 u32Channel, u32 u32InterruptFlag);
static int DrvSPU_ChannelOpen(u32 u32Channel);
static int DrvSPU_ChannelClose(u32 u32Channel);
static int DrvSPU_DisableInt(u32 u32Channel, u32 u32InterruptFlag);
static int DrvSPU_ClearInt(u32 u32Channel, u32 u32InterruptFlag);

static int spuInit(void);
static int spuStartPlay(int nChannels);
static void spuStopPlay(void);
static void spuSetPlaySampleRate(int nSamplingRate);
void spuDacOn(void);

// PLL clock settings
extern unsigned int w55fa92_apll_clock;
extern unsigned int w55fa92_upll_clock;
extern int w55fa92_set_apll_clock(unsigned int clock);

// Linux 2.6.35
static DEFINE_MUTEX(spu_mutex);
struct w55fa92_audio *w55fa92_spu_data;

static const int N_table[32] = {128,114,101,90,80,71,64,57,50,45,40,36,32,28,25,22,20,18,16,14,12,11,10,9,8,7,6,5,4,3,2,0};
extern int spuNormalizeVolume(int inputVolume);

//struct clk *clk_hclk4 = NULL;
static int DrvAUR_AudioI2cRead(int u32Addr, char* p8Data);
static int DrvAUR_AudioI2cWrite(int u32Addr, int u32Data);
static struct clk *clk_hclk4 = NULL, *clk_adc = NULL;
extern void nvt_lock(void);
extern void nvt_unlock(void);
extern void update_dac_register_cache(void);
int DrvSPU_ReadDACReg (int DACRegIndex);
void DrvSPU_WriteDACReg (int DACRegIndex, int DACRegData);

#ifdef CONFIG_HEADSET_ENABLED		// for headset_detect
	#define HEADSET_IRQ_NUM 		W55FA92_IRQ(4)  // nIRQ_GPIO2
	#define Enable_IRQ(n)     		outl(1 << (n),REG_AIC_MECR)
	#define Disable_IRQ(n)    		outl(1 << (n),REG_AIC_MDCR)
#endif


#define OPT_SPU_INIT_20140311
#ifdef OPT_SPU_INIT_20140311
	#define SPU_DACINIT_DELAY	100 			// 1000 ms
	static struct timer_list spuinit_timer;		// handle depop
	static void spu_1Sec_TimeOut(unsigned long data);	
#endif	

#ifdef OPT_SPU_INIT_20140311
	void spuDacEnable(void)
	{
		int reg_00, reg_01;
		
		// backup reg_0x00, reg_0x01 contents
		reg_00 = DrvSPU_ReadDACReg(0x00);
		reg_01 = DrvSPU_ReadDACReg(0x01);		

	    DrvSPU_WriteDACReg(0x0, 0x1F);	
	    DrvSPU_WriteDACReg(0x0, 0x1F);                                                        
		
	    // open power amplifer 
	    DrvSPU_WriteDACReg(0x03, 0x05);	
	    DrvSPU_WriteDACReg(0x05, 0x10);                                                        
	
	    // given analog and digital volume
		DrvSPU_WriteDACReg(0x00, 0x05);	
		DrvSPU_WriteDACReg(0x01, 0x05);	
//		DrvSPU_WriteDACReg(0x09, 0x80);	
//		DrvSPU_WriteDACReg(0x0A, 0x80);	
		  
	    // software reset DAC is I2S slave mode 
	    DrvSPU_WriteDACReg(0x08, 0x01);	                   
	    DrvSPU_WriteDACReg(0x08, 0x03);

		mdelay(10);	    
	    DrvSPU_WriteDACReg(0x00, reg_00);	
	    DrvSPU_WriteDACReg(0x01, reg_01);                                                        
	}

	void spuADCVmidEnable(void)
	{
	    // enable ADC VMID
	    AUDIO_WRITE(REG_APBCLK, AUDIO_READ(REG_APBCLK) | ADC_CKE);
	    AUDIO_WRITE(REG_CLKDIV3, (AUDIO_READ(REG_CLKDIV3) & ~(ADC_N1 | ADC_S| ADC_N0)) ); // Fed to ADC clock need 12MHz=External clock 
	    AUDIO_WRITE(REG_SDADC_CTL, 0x308121bf);      // enable ADC VMID
	   	while(AUDIO_READ(REG_SDADC_CTL) & AR_BUSY);	
	   	
	   	// enable SPU DAC digital part
	    DrvSPU_WriteDACReg(0x07, 0x01);	   	
	}

	static void spu_1Sec_TimeOut(unsigned long data)
	{
//		printk("setup SPU time OUT !!!\n");
		spuDacEnable();
		update_dac_register_cache();
		printk("SPU DAC ON !!! \n");		
	}
			
	static void setup_spuInit_TimeOut(unsigned long data)
	{
		
		init_timer(&spuinit_timer);
//		spuinit_timer.function = spu_1Sec_TimeOut;		// timer handler for touch 
//		mod_timer(&spuinit_timer, jiffies + TOUCH_INTERVAL_TIME);
//      del_timer(&spuinit_timer);	        
        spuinit_timer.data = 0UL;
        spuinit_timer.expires = jiffies +  SPU_DACINIT_DELAY;
        spuinit_timer.function = spu_1Sec_TimeOut;
        add_timer(&spuinit_timer);
        return;
	}
#endif

int DrvSPU_ReadDACReg (int DACRegIndex)
{
	int u32Reg = 0x30800000;		// clock divider = 0x30, ID = 0x80
//	int u32Reg = 0x40800000;		// clock divider = 0x40, ID = 0x80	
	int u8Ret;
	
#ifdef CONFIG_HEADSET_ENABLED		// for headset_detect
	Disable_IRQ(HEADSET_IRQ_NUM);	
#endif	

	u32Reg |= DACRegIndex << 8;
	while(AUDIO_READ(REG_SPU_DAC_CTRL) & V_I2C_BUSY);
	AUDIO_WRITE(REG_SPU_DAC_CTRL, u32Reg);	
	udelay(200);
	while(AUDIO_READ(REG_SPU_DAC_CTRL) & V_I2C_BUSY);
	udelay(200);
	u8Ret = AUDIO_READ(REG_SPU_DAC_CTRL) & 0xFF;	
	
#ifdef CONFIG_HEADSET_ENABLED		// for headset_detect
	Enable_IRQ(HEADSET_IRQ_NUM);	
#endif	
	
	return u8Ret;
}

void DrvSPU_WriteDACReg (int DACRegIndex, int DACRegData)
{
	int u32Reg = 0x30810000;		// clock divider = 0x30, ID = 0x80
//	int u32Reg = 0x40810000;		// clock divider = 0x40, ID = 0x81		

	ENTER();
	
#ifdef CONFIG_HEADSET_ENABLED		// for headset_detect
	Disable_IRQ(HEADSET_IRQ_NUM);	
#endif	

	DBG("DACRegIndex = 0x%x !!! \n", DACRegIndex);
	DBG("DACRegData = 0x%x !!! \n", DACRegData);	


#ifdef OPT_FPGA_DEBUG
	if ( (AUDIO_READ(REG_CLKDIV1) & 0xFF000000) == 0x00)
		AUDIO_WRITE(REG_CLKDIV1, AUDIO_READ(REG_CLKDIV1) | 0x10000000);	// given ADO clock divider at least 1	
#endif
						
	u32Reg |= DACRegIndex << 8;
	u32Reg |= DACRegData;
	while(AUDIO_READ(REG_SPU_DAC_CTRL) & V_I2C_BUSY);
	AUDIO_WRITE(REG_SPU_DAC_CTRL, u32Reg);	
	udelay(200);
	while(AUDIO_READ(REG_SPU_DAC_CTRL) & V_I2C_BUSY);	
	udelay(200);	

#ifdef CONFIG_HEADSET_ENABLED		// for headset_detect
	Enable_IRQ(HEADSET_IRQ_NUM);	
#endif	
	
}

//===========================================================
static int w55fa92_spu_set_fmt(struct snd_soc_dai *cpu_dai,
                              unsigned int fmt)
{
		ENTER();

	#ifdef CONFIG_SPU_WITH_I2S_OUTPUT 	
        AUDIO_WRITE(REG_I2S_ACTL_I2SCON, AUDIO_READ(REG_I2S_ACTL_I2SCON) & ~MSB_Justified);
	#endif        
	
//		struct w55fa92_audio *w55fa92_audio = w55fa92_spu_data;
//     	unsigned long val = 0;
        return 0;
}

static int w55fa92_spu_set_sysclk(struct snd_soc_dai *cpu_dai,
                                 int clk_id, unsigned int freq, int dir)
{
		ENTER();

//        unsigned int val;
//        struct w55fa92_audio *w55fa92_audio = w55fa92_spu_data;

		spuSetPlaySampleRate(freq);
		LEAVE();		
        return 0;
}

static int w55fa92_spu_trigger(struct snd_pcm_substream *substream,
                              int cmd, struct snd_soc_dai *dai)
{
        int ret = 0;
		ENTER();
		
        switch (cmd) {
        case SNDRV_PCM_TRIGGER_START:
        	DBG("SPU trigger start !!! \n");
				if (spuStartPlay(substream->runtime->channels))
                	ret = -EINVAL;					

//#define OPT_DAC_POWERDOWN_TEST				        	
#ifdef OPT_DAC_POWERDOWN_TEST				
			DrvSPU_WriteDACReg (0x05, DrvSPU_ReadDACReg(0x05)& ~0x03);	// enable headphone driver
//	printk("enable headphone driver !!!\n");						
#endif        	
        
        case SNDRV_PCM_TRIGGER_RESUME:
        	DBG("SPU trigger resume !!! \n");        
			AUDIO_WRITE(REG_SPU_CTRL, AUDIO_READ(REG_SPU_CTRL) | SPU_I2S_EN);        	
			AUDIO_WRITE(REG_SPU_CTRL, AUDIO_READ(REG_SPU_CTRL) | SPU_EN);
//			DBG("REG_SPU_DAC_VOL = 0x%x  !!!\n", AUDIO_READ(REG_SPU_DAC_VOL));						
			
	#ifdef CONFIG_SPU_WITH_I2S_OUTPUT 
            AUDIO_WRITE(REG_I2S_ACTL_CON, AUDIO_READ(REG_I2S_ACTL_CON) | I2S_EN);
            AUDIO_WRITE(REG_I2S_ACTL_RESET, AUDIO_READ(REG_I2S_ACTL_RESET) | I2S_PLAY);            
	#endif            
                break;
                
        case SNDRV_PCM_TRIGGER_STOP:
        	DBG("SPU trigger stop !!! \n");                
				spuStopPlay(); 
				

#ifdef OPT_DAC_POWERDOWN_TEST				
			DrvSPU_WriteDACReg (0x05, DrvSPU_ReadDACReg(0x05)| 0x03);	// enable headphone driver
//	printk("disable headphone driver !!!\n");			
#endif

        case SNDRV_PCM_TRIGGER_SUSPEND:
        	DBG("SPU trigger suspend !!! \n");                
			AUDIO_WRITE(REG_SPU_CTRL, AUDIO_READ(REG_SPU_CTRL) & ~SPU_EN);        

	#ifdef CONFIG_SPU_WITH_I2S_OUTPUT 
            AUDIO_WRITE(REG_I2S_ACTL_RESET, AUDIO_READ(REG_I2S_ACTL_RESET) & ~I2S_PLAY);                        
            AUDIO_WRITE(REG_I2S_ACTL_CON, AUDIO_READ(REG_I2S_ACTL_CON) & ~I2S_EN);            
	#endif            
	            break;
        default:
                ret = -EINVAL;
        }

		LEAVE();        
        return ret;
}

static int w55fa92_spu_probe(struct platform_device *pdev,
                            struct snd_soc_dai *dai)
{
//        struct w55fa92_audio *w55fa92_audio = w55fa92_spu_data;
        struct w55fa92_audio *w55fa92_audio = w55fa92_spu_data;

		ENTER();
        mutex_lock(&spu_mutex);

        /* enable unit clock */
//        clk_enable(w55fa92_audio->clk);        

        clk_enable(clk_hclk4);        
        clk_enable(w55fa92_audio->eng_clk);        
        clk_enable(w55fa92_audio->spu_clk);        

		if (!s_spuInit)
		{
			s_spuInit = 1;	
			spuInit();
			
			if (DrvSPU_ReadDACReg(0x05) != 0x10)
			{
				spuADCVmidEnable();		// enable ADC VMID			
				setup_spuInit_TimeOut(0);
			}
		}					
		else
			update_dac_register_cache();

	#ifdef CONFIG_SPU_WITH_I2S_OUTPUT 
		// enable I2S pins
		AUDIO_WRITE(REG_GPBFUN0, (AUDIO_READ(REG_GPBFUN0)&~(MF_GPB6+MF_GPB5+MF_GPB4+MF_GPB3+MF_GPB2))|0x01111100);	// GPB[6:2] to be I2S signals
		
		AUDIO_WRITE(REG_MISFUN, AUDIO_READ(REG_MISFUN) & (~0x01));			// I2S interface for I2S, but not SPU

		// enable I2S engine clock		
        clk_enable(w55fa92_audio->i2s_clk);        		
		// set Play & Record interrupt encountered in half of DMA buffer length
		AUDIO_WRITE(REG_I2S_ACTL_CON, (AUDIO_READ(REG_I2S_ACTL_CON) & (~R_DMA_IRQ_SEL)) | (0x01 << 14)); 	
		AUDIO_WRITE(REG_I2S_ACTL_CON, (AUDIO_READ(REG_I2S_ACTL_CON) & (~P_DMA_IRQ_SEL)) | (0x01 << 12)); 		
	#endif

        mutex_unlock(&spu_mutex);
		LEAVE();        
        return 0;
}

static void w55fa92_spu_remove(struct platform_device *pdev,
                              struct snd_soc_dai *dai)
{
//        struct w55fa92_audio *w55fa92_audio = w55fa92_spu_data;
        struct w55fa92_audio *w55fa92_audio = w55fa92_spu_data;

	ENTER();
	
		spuStopPlay();         
	LEAVE();		
//        clk_disable(w55fa92_audio->clk);
#ifdef CONFIG_SPU_WITH_I2S_OUTPUT 
        clk_disable(w55fa92_audio->i2s_clk);
        clk_put(w55fa92_audio->i2s_clk);        
#endif
        clk_disable(w55fa92_audio->spu_clk);
        clk_disable(w55fa92_audio->eng_clk);
	    clk_disable(clk_hclk4);                
        clk_put(w55fa92_audio->spu_clk);
        clk_put(w55fa92_audio->eng_clk);
	    clk_put(clk_adc);                	    
	    clk_put(clk_hclk4);                
}

static struct snd_soc_dai_ops w55fa92_spu_dai_ops = {
        .trigger	= w55fa92_spu_trigger,
        .set_fmt	= w55fa92_spu_set_fmt,
        .set_sysclk	= w55fa92_spu_set_sysclk,
};

struct snd_soc_dai w55fa92_spu_dai = {
        .name			= "w55fa92-spu",
        .id 			= 0,
        .probe			= w55fa92_spu_probe,
        .remove			= w55fa92_spu_remove,
        .playback = {
                .rates		= SNDRV_PCM_RATE_8000_48000,
                .formats	= SNDRV_PCM_FMTBIT_S16_LE,
                .channels_min	= 1,
                .channels_max	= 2,
        },
        .capture = {
                .rates		= SNDRV_PCM_RATE_8000_48000,
                .formats	= SNDRV_PCM_FMTBIT_S16_LE,
                .channels_min	= 1,
                .channels_max	= 2,
        },
        .ops = &w55fa92_spu_dai_ops,
};
EXPORT_SYMBOL_GPL(w55fa92_spu_dai);

static int __devinit w55fa92_spu_drvprobe(struct platform_device *pdev)
{
        struct w55fa92_audio *w55fa92_audio;
        int ret;

	ENTER();
	
        if (w55fa92_spu_data)
                return -EBUSY;

        w55fa92_audio = kzalloc(sizeof(struct w55fa92_audio), GFP_KERNEL);
        if (!w55fa92_audio)
                return -ENOMEM;

        spin_lock_init(&w55fa92_audio->lock);
		spin_lock_init(&w55fa92_audio->irqlock);		
        
        w55fa92_audio->res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
        if (!w55fa92_audio->res) {
                ret = -ENODEV;
                goto out0;
        }

        if (!request_mem_region(w55fa92_audio->res->start,
                                resource_size(w55fa92_audio->res), pdev->name)) {
                ret = -EBUSY;
                goto out0;
        }

        w55fa92_audio->mmio = ioremap(w55fa92_audio->res->start,
                                     resource_size(w55fa92_audio->res));
        if (!w55fa92_audio->mmio) {
                ret = -ENOMEM;
                goto out1;
        }

//        w55fa92_audio->clk = clk_get(&pdev->dev, NULL);
//        if (IS_ERR(w55fa92_audio->clk)) {
//                ret = PTR_ERR(w55fa92_audio->clk);
//                goto out2;
//        }

       if ( clk_hclk4 == NULL )
       {
			clk_hclk4 = clk_get(NULL, "hclk4");
	        if (IS_ERR(clk_hclk4)) {
                ret = PTR_ERR(clk_hclk4);
                goto out2;       	
			}                
       }
       if ( clk_adc == NULL )
       {
			clk_adc = clk_get(NULL, "adc");
	        if (IS_ERR(clk_adc)) {
                ret = PTR_ERR(clk_adc);
                goto out2;       	
			}                
       }
       
		printk("ADC clock get OK !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");       
		
        w55fa92_audio->spu_clk = clk_get(NULL, "spu");
        if (IS_ERR(w55fa92_audio->spu_clk)) {
                ret = PTR_ERR(w55fa92_audio->spu_clk);
                goto out2;
        }

        w55fa92_audio->eng_clk = clk_get(NULL, "ado");
        if (IS_ERR(w55fa92_audio->eng_clk)) {
                ret = PTR_ERR(w55fa92_audio->eng_clk);
                goto out2;
        }

	#ifdef CONFIG_SPU_WITH_I2S_OUTPUT 
        w55fa92_audio->i2s_clk = clk_get(NULL, "i2s");
        if (IS_ERR(w55fa92_audio->i2s_clk)) {
                ret = PTR_ERR(w55fa92_audio->i2s_clk);
                goto out2;
        }
	#endif
//        w55fa92_audio->irq_num = platform_get_irq(pdev, 0);
        w55fa92_audio->irq_num = IRQ_SPU;
        if (!w55fa92_audio->irq_num) {
                ret = -EBUSY;
                goto out2;
        }
		
		ret = w55fa92_dma_create(w55fa92_audio);
		if (ret != 0)
			return ret;
		
        w55fa92_spu_data = w55fa92_audio;

        w55fa92_audio->dev = w55fa92_spu_dai.dev =  &pdev->dev;

        ret = snd_soc_register_dai(&w55fa92_spu_dai);
        
        if (ret)
                goto out3;

 //       mfp_set_groupg(w55fa92_audio->dev); /* enbale spu multifunction pin*/

	LEAVE();
        return 0;

out3:
//        clk_put(w55fa92_audio->clk);
out2:
        iounmap(w55fa92_audio->mmio);
out1:
        release_mem_region(w55fa92_audio->res->start,
                           resource_size(w55fa92_audio->res));
out0:
        kfree(w55fa92_audio);
	LEAVE();        
        return ret;
}

static int __devexit w55fa92_spu_drvremove(struct platform_device *pdev)
{
	ENTER();	
	
		w55fa92_dma_destroy(w55fa92_spu_data);
		
        snd_soc_unregister_dai(&w55fa92_spu_dai);

//        clk_put(w55fa92_spu_data->clk);
        iounmap(w55fa92_spu_data->mmio);
        release_mem_region(w55fa92_spu_data->res->start,
                           resource_size(w55fa92_spu_data->res));

        w55fa92_spu_data = NULL;
	LEAVE();
        return 0;
}

static struct platform_driver w55fa92_spu_driver = {
        .driver	= {
                .name	= "w55fa92-audio-spu",
                .owner	= THIS_MODULE,
        },
        .probe		= w55fa92_spu_drvprobe,
        .remove		= __devexit_p(w55fa92_spu_drvremove),
                     };

static int __init w55fa92_spu_init(void)
{
        return platform_driver_register(&w55fa92_spu_driver);
}

static void __exit w55fa92_spu_exit(void)
{
	ENTER();
		
        platform_driver_unregister(&w55fa92_spu_driver);
	LEAVE();        
}


int DrvSPU_SetBaseAddress(
	u32 u32Channel, 
	u32 u32Address
)
{
	if ( (u32Channel >=eDRVSPU_CHANNEL_0) && (u32Channel <=eDRVSPU_CHANNEL_31) )
	{
		// wait to finish previous channel settings
		while(AUDIO_READ(REG_SPU_CH_CTRL) & CH_FN);
		
		// load previous channel settings		
		AUDIO_WRITE(REG_SPU_CH_CTRL, (AUDIO_READ(REG_SPU_CH_CTRL) & ~CH_NO) | (u32Channel << 24));		
		AUDIO_WRITE(REG_SPU_CH_CTRL, AUDIO_READ(REG_SPU_CH_CTRL) | DRVSPU_LOAD_SELECTED_CHANNEL);
		while(AUDIO_READ(REG_SPU_CH_CTRL) & CH_FN);
		
		AUDIO_WRITE(REG_SPU_S_ADDR, u32Address);		

		AUDIO_WRITE(REG_SPU_CH_CTRL, AUDIO_READ(REG_SPU_CH_CTRL) | DRVSPU_UPDATE_ALL_SETTINGS);				
		while(AUDIO_READ(REG_SPU_CH_CTRL) & CH_FN);		
		
		return E_SUCCESS;
	}
	else
		return E_DRVSPU_WRONG_CHANNEL;	   
}

int DrvSPU_GetCurrentAddress(
	u32 u32Channel
)
{
//	ENTER();	
	if ( (u32Channel >=eDRVSPU_CHANNEL_0) && (u32Channel <=eDRVSPU_CHANNEL_31) )
	{
		// wait to finish previous channel settings
		while(AUDIO_READ(REG_SPU_CH_CTRL) & CH_FN);
		
		// load previous channel settings		
		AUDIO_WRITE(REG_SPU_CH_CTRL, (AUDIO_READ(REG_SPU_CH_CTRL) & ~CH_NO) | (u32Channel << 24));		
		AUDIO_WRITE(REG_SPU_CH_CTRL, AUDIO_READ(REG_SPU_CH_CTRL) | DRVSPU_LOAD_SELECTED_CHANNEL);
		while(AUDIO_READ(REG_SPU_CH_CTRL) & CH_FN);
		
//	LEAVE();		
		return AUDIO_READ(REG_SPU_CUR_ADDR);
	}

	else
	{
		LEAVE();					
		return 0;	   
	}		
		 
}

int DrvSPU_GetBaseAddress(
	u32 u32Channel
)
{
	if ( (u32Channel >=eDRVSPU_CHANNEL_0) && (u32Channel <=eDRVSPU_CHANNEL_31) )
	{
		// wait to finish previous channel settings
		while(AUDIO_READ(REG_SPU_CH_CTRL) & CH_FN);
		
		// load previous channel settings		
		AUDIO_WRITE(REG_SPU_CH_CTRL, (AUDIO_READ(REG_SPU_CH_CTRL) & ~CH_NO) | (u32Channel << 24));		
		AUDIO_WRITE(REG_SPU_CH_CTRL, AUDIO_READ(REG_SPU_CH_CTRL) | DRVSPU_LOAD_SELECTED_CHANNEL);
		while(AUDIO_READ(REG_SPU_CH_CTRL) & CH_FN);
		
		return (AUDIO_READ(REG_SPU_S_ADDR));
	}
	else
		return 0;
}

int DrvSPU_SetThresholdAddress(
	u32 u32Channel, 
	u32 u32Address
)
{
	if ( (u32Channel >=eDRVSPU_CHANNEL_0) && (u32Channel <=eDRVSPU_CHANNEL_31) )
	{
		// wait to finish previous channel settings
		while(AUDIO_READ(REG_SPU_CH_CTRL) & CH_FN);
		
		// load previous channel settings		
		AUDIO_WRITE(REG_SPU_CH_CTRL, (AUDIO_READ(REG_SPU_CH_CTRL) & ~CH_NO) | (u32Channel << 24));		
		AUDIO_WRITE(REG_SPU_CH_CTRL, AUDIO_READ(REG_SPU_CH_CTRL) | DRVSPU_LOAD_SELECTED_CHANNEL);
		while(AUDIO_READ(REG_SPU_CH_CTRL) & CH_FN);
		
		AUDIO_WRITE(REG_SPU_M_ADDR, u32Address);		

		AUDIO_WRITE(REG_SPU_CH_CTRL, AUDIO_READ(REG_SPU_CH_CTRL) | DRVSPU_UPDATE_ALL_SETTINGS);				
		while(AUDIO_READ(REG_SPU_CH_CTRL) & CH_FN);		
		
		return E_SUCCESS;
	}
	 
	else
		return E_DRVSPU_WRONG_CHANNEL;	   
}

int DrvSPU_GetThresholdAddress(
	u32 u32Channel
)
{
	if ( (u32Channel >=eDRVSPU_CHANNEL_0) && (u32Channel <=eDRVSPU_CHANNEL_31) )
	{
		// wait to finish previous channel settings
		while(AUDIO_READ(REG_SPU_CH_CTRL) & CH_FN);
		
		// load previous channel settings		
		AUDIO_WRITE(REG_SPU_CH_CTRL, (AUDIO_READ(REG_SPU_CH_CTRL) & ~CH_NO) | (u32Channel << 24));		
		AUDIO_WRITE(REG_SPU_CH_CTRL, AUDIO_READ(REG_SPU_CH_CTRL) | DRVSPU_LOAD_SELECTED_CHANNEL);
		while(AUDIO_READ(REG_SPU_CH_CTRL) & CH_FN);
		
		return AUDIO_READ(REG_SPU_M_ADDR);
	}
	else
		return 0;
}

int DrvSPU_SetEndAddress(
	u32 u32Channel, 
	u32 u32Address
)
{
	if ( (u32Channel >=eDRVSPU_CHANNEL_0) && (u32Channel <=eDRVSPU_CHANNEL_31) )
	{
		// wait to finish previous channel settings
		while(AUDIO_READ(REG_SPU_CH_CTRL) & CH_FN);
		
		// load previous channel settings		
		AUDIO_WRITE(REG_SPU_CH_CTRL, (AUDIO_READ(REG_SPU_CH_CTRL) & ~CH_NO) | (u32Channel << 24));		
		AUDIO_WRITE(REG_SPU_CH_CTRL, AUDIO_READ(REG_SPU_CH_CTRL) | DRVSPU_LOAD_SELECTED_CHANNEL);
		while(AUDIO_READ(REG_SPU_CH_CTRL) & CH_FN);
		
		AUDIO_WRITE(REG_SPU_E_ADDR, u32Address);		

		AUDIO_WRITE(REG_SPU_CH_CTRL, AUDIO_READ(REG_SPU_CH_CTRL) | DRVSPU_UPDATE_ALL_SETTINGS);				
		while(AUDIO_READ(REG_SPU_CH_CTRL) & CH_FN);		
		
		return E_SUCCESS;
	}
	 
	else
		return E_DRVSPU_WRONG_CHANNEL;	   
}

int DrvSPU_GetEndAddress(
	u32 u32Channel
)
{
	if ( (u32Channel >=eDRVSPU_CHANNEL_0) && (u32Channel <=eDRVSPU_CHANNEL_31) )
	{
		// wait to finish previous channel settings
		while(AUDIO_READ(REG_SPU_CH_CTRL) & CH_FN);
		
		// load previous channel settings		
		AUDIO_WRITE(REG_SPU_CH_CTRL, (AUDIO_READ(REG_SPU_CH_CTRL) & ~CH_NO) | (u32Channel << 24));		
		AUDIO_WRITE(REG_SPU_CH_CTRL, AUDIO_READ(REG_SPU_CH_CTRL) | DRVSPU_LOAD_SELECTED_CHANNEL);
		while(AUDIO_READ(REG_SPU_CH_CTRL) & CH_FN);
		
		return AUDIO_READ(REG_SPU_E_ADDR);
	}
	else
		return 0;
}



int DrvSPU_SetPauseAddress(
	u32 u32Channel, 
	u32 u32Address
)
{
	
	if ( (u32Channel >=eDRVSPU_CHANNEL_0) && (u32Channel <=eDRVSPU_CHANNEL_31) )
	{
		// wait to finish previous channel settings
		while(AUDIO_READ(REG_SPU_CH_CTRL) & CH_FN);
		
		// load previous channel settings		
		AUDIO_WRITE(REG_SPU_CH_CTRL, (AUDIO_READ(REG_SPU_CH_CTRL) & ~CH_NO) | (u32Channel << 24));		
//		AUDIO_WRITE(REG_SPU_CH_CTRL, AUDIO_READ(REG_SPU_CH_CTRL) | DRVSPU_LOAD_SELECTED_CHANNEL);
//		while(AUDIO_READ(REG_SPU_CH_CTRL) & CH_FN);
		
		AUDIO_WRITE(REG_SPU_PA_ADDR, u32Address);		

		AUDIO_WRITE(REG_SPU_CH_CTRL, (AUDIO_READ(REG_SPU_CH_CTRL) & ~0xFF) | DRVSPU_UPDATE_PAUSE_PARTIAL);				
		while(AUDIO_READ(REG_SPU_CH_CTRL) & CH_FN);		
		
#if 1
		AUDIO_WRITE(REG_SPU_CH_CTRL, (AUDIO_READ(REG_SPU_CH_CTRL) & ~CH_NO) | ((u32Channel+1) << 24));		
		AUDIO_WRITE(REG_SPU_CH_CTRL, (AUDIO_READ(REG_SPU_CH_CTRL) & ~0xFF) | DRVSPU_UPDATE_PAUSE_PARTIAL);				
		while(AUDIO_READ(REG_SPU_CH_CTRL) & CH_FN);		
		
#endif 		
		
		return E_SUCCESS;
	}
	else
		return E_DRVSPU_WRONG_CHANNEL;	   
}


int DrvSPU_GetPauseAddress(
	u32 u32Channel
)
{
	
	if ( (u32Channel >=eDRVSPU_CHANNEL_0) && (u32Channel <=eDRVSPU_CHANNEL_31) )
	{
		// wait to finish previous channel settings
		while(AUDIO_READ(REG_SPU_CH_CTRL) & CH_FN);
		
		// load previous channel settings		
		AUDIO_WRITE(REG_SPU_CH_CTRL, (AUDIO_READ(REG_SPU_CH_CTRL) & ~CH_NO) | (u32Channel << 24));		
		AUDIO_WRITE(REG_SPU_CH_CTRL, AUDIO_READ(REG_SPU_CH_CTRL) | DRVSPU_LOAD_SELECTED_CHANNEL);
		while(AUDIO_READ(REG_SPU_CH_CTRL) & CH_FN);
		
		return AUDIO_READ(REG_SPU_PA_ADDR);		
	}
	else
		return E_DRVSPU_WRONG_CHANNEL;	   
}

#if 0 
static int DrvSPU_GetLoopStartAddress(
	u32 u32Channel, 
	u32 u32Address
)
{
	if ( (u32Channel >=eDRVSPU_CHANNEL_0) && (u32Channel <=eDRVSPU_CHANNEL_31) )
	{
		// wait to finish previous channel settings
		while(AUDIO_READ(REG_SPU_CH_CTRL) & CH_FN);
		
		// load previous channel settings		
		AUDIO_WRITE(REG_SPU_CH_CTRL, (AUDIO_READ(REG_SPU_CH_CTRL) & ~CH_NO) | (u32Channel << 24));		
		AUDIO_WRITE(REG_SPU_CH_CTRL, AUDIO_READ(REG_SPU_CH_CTRL) | DRVSPU_LOAD_SELECTED_CHANNEL);
		while(AUDIO_READ(REG_SPU_CH_CTRL) & CH_FN);
		
		return AUDIO_READ(REG_SPU_LP_ADDR);
	}
	else return 0;	   
}
#endif

int DrvSPU_SetDFA(u32 u32Channel, u16 u16DFA)
{
	if ( (u32Channel >=eDRVSPU_CHANNEL_0) && (u32Channel <=eDRVSPU_CHANNEL_31) )
	{
		// wait to finish previous channel settings
		while(AUDIO_READ(REG_SPU_CH_CTRL) & CH_FN);
		
		// load previous channel settings		
		AUDIO_WRITE(REG_SPU_CH_CTRL, (AUDIO_READ(REG_SPU_CH_CTRL) & ~CH_NO) | (u32Channel << 24));		
		AUDIO_WRITE(REG_SPU_CH_CTRL, AUDIO_READ(REG_SPU_CH_CTRL) | DRVSPU_LOAD_SELECTED_CHANNEL);
		while(AUDIO_READ(REG_SPU_CH_CTRL) & CH_FN);
		
		AUDIO_WRITE(REG_SPU_CH_PAR_2, u16DFA);		

		AUDIO_WRITE(REG_SPU_CH_CTRL, AUDIO_READ(REG_SPU_CH_CTRL) & ~DRVSPU_UPDATE_ALL_PARTIALS);		
		AUDIO_WRITE(REG_SPU_CH_CTRL, AUDIO_READ(REG_SPU_CH_CTRL) | (DRVSPU_UPDATE_DFA_PARTIAL + DRVSPU_UPDATE_PARTIAL_SETTINGS));				

		while(AUDIO_READ(REG_SPU_CH_CTRL) & CH_FN);		
		
		return E_SUCCESS;
	}
	 
	else
		return E_DRVSPU_WRONG_CHANNEL;	   
}


int DrvSPU_GetDFA(u32 u32Channel)
{
	if ( (u32Channel >=eDRVSPU_CHANNEL_0) && (u32Channel <=eDRVSPU_CHANNEL_31) )
	{
		// wait to finish previous channel settings
		while(AUDIO_READ(REG_SPU_CH_CTRL) & CH_FN);
		
		// load previous channel settings		
		AUDIO_WRITE(REG_SPU_CH_CTRL, (AUDIO_READ(REG_SPU_CH_CTRL) & ~CH_NO) | (u32Channel << 24));		
		AUDIO_WRITE(REG_SPU_CH_CTRL, AUDIO_READ(REG_SPU_CH_CTRL) | DRVSPU_LOAD_SELECTED_CHANNEL);
		while(AUDIO_READ(REG_SPU_CH_CTRL) & CH_FN);
		
		return AUDIO_READ(REG_SPU_CH_PAR_2) & 0x1FFF;
	}
	else
		return 0;
}

// MSB 8-bit = left channel; LSB 8-bit = right channel
//static int DrvSPU_SetPAN(
int DrvSPU_SetPAN(u32 u32Channel, u16 u16PAN)
{
	u32 u32PAN;
	
	if ( (u32Channel >=eDRVSPU_CHANNEL_0) && (u32Channel <=eDRVSPU_CHANNEL_31) )
	{
		// wait to finish previous channel settings
		while(AUDIO_READ(REG_SPU_CH_CTRL) & CH_FN);
		
		// load previous channel settings		
		AUDIO_WRITE(REG_SPU_CH_CTRL, (AUDIO_READ(REG_SPU_CH_CTRL) & ~CH_NO) | (u32Channel << 24));		
		AUDIO_WRITE(REG_SPU_CH_CTRL, AUDIO_READ(REG_SPU_CH_CTRL) | DRVSPU_LOAD_SELECTED_CHANNEL);
		while(AUDIO_READ(REG_SPU_CH_CTRL) & CH_FN);
		
		u32PAN = u16PAN;
		u32PAN <<= 8;			
		u32PAN &= (PAN_L + PAN_R);
		AUDIO_WRITE(REG_SPU_CH_PAR_1, (AUDIO_READ(REG_SPU_CH_PAR_1) & (~(PAN_L+PAN_R))) | u32PAN);		

		AUDIO_WRITE(REG_SPU_CH_CTRL, AUDIO_READ(REG_SPU_CH_CTRL) & ~DRVSPU_UPDATE_ALL_PARTIALS);		
		AUDIO_WRITE(REG_SPU_CH_CTRL, AUDIO_READ(REG_SPU_CH_CTRL) | (DRVSPU_UPDATE_PAN_PARTIAL + DRVSPU_UPDATE_PARTIAL_SETTINGS));				

		while(AUDIO_READ(REG_SPU_CH_CTRL) & CH_FN);		
		
		return E_SUCCESS;
	}
	 
	else
		return E_DRVSPU_WRONG_CHANNEL;	   
}

int DrvSPU_GetPAN(u32 u32Channel)
{
	u32 u32PAN;
	
	if ( (u32Channel >=eDRVSPU_CHANNEL_0) && (u32Channel <=eDRVSPU_CHANNEL_31) )
	{
		// wait to finish previous channel settings
		while(AUDIO_READ(REG_SPU_CH_CTRL) & CH_FN);
		
		// load previous channel settings		
		AUDIO_WRITE(REG_SPU_CH_CTRL, (AUDIO_READ(REG_SPU_CH_CTRL) & ~CH_NO) | (u32Channel << 24));		
		AUDIO_WRITE(REG_SPU_CH_CTRL, AUDIO_READ(REG_SPU_CH_CTRL) | DRVSPU_LOAD_SELECTED_CHANNEL);
		while(AUDIO_READ(REG_SPU_CH_CTRL) & CH_FN);
		
		u32PAN = AUDIO_READ(REG_SPU_CH_PAR_1);
		u32PAN >>= 8;
		return (u32PAN & 0xFFFF);
	}
	else
		return 0;
}


int DrvSPU_SetSrcType(
	u32 u32Channel, 
	u8  u8DataFormat
)
{

	if ( (u32Channel >=eDRVSPU_CHANNEL_0) && (u32Channel <=eDRVSPU_CHANNEL_31) )
	{
		// wait to finish previous channel settings
		while(AUDIO_READ(REG_SPU_CH_CTRL) & CH_FN);
		
		// load previous channel settings		
		AUDIO_WRITE(REG_SPU_CH_CTRL, (AUDIO_READ(REG_SPU_CH_CTRL) & ~CH_NO) | (u32Channel << 24));		
		AUDIO_WRITE(REG_SPU_CH_CTRL, AUDIO_READ(REG_SPU_CH_CTRL) | DRVSPU_LOAD_SELECTED_CHANNEL);
		while(AUDIO_READ(REG_SPU_CH_CTRL) & CH_FN);
		
		AUDIO_WRITE(REG_SPU_CH_PAR_1, (AUDIO_READ(REG_SPU_CH_PAR_1) & ~SRC_TYPE) | u8DataFormat);		
		AUDIO_WRITE(REG_SPU_CH_CTRL, AUDIO_READ(REG_SPU_CH_CTRL) | DRVSPU_UPDATE_ALL_SETTINGS );				

		while(AUDIO_READ(REG_SPU_CH_CTRL) & CH_FN);		
		
		return E_SUCCESS;
	}
	 
	else
		return E_DRVSPU_WRONG_CHANNEL;	   
}

#if 0
static int DrvSPU_GetSrcType(
	u32 u32Channel
)
{
	u8 u8DataFormat;
	
	if ( (u32Channel >=eDRVSPU_CHANNEL_0) && (u32Channel <=eDRVSPU_CHANNEL_31) )
	{
		// wait to finish previous channel settings
		while(AUDIO_READ(REG_SPU_CH_CTRL) & CH_FN);
		
		// load previous channel settings		
		AUDIO_WRITE(REG_SPU_CH_CTRL, (AUDIO_READ(REG_SPU_CH_CTRL) & ~CH_NO) | (u32Channel << 24));		
		AUDIO_WRITE(REG_SPU_CH_CTRL, AUDIO_READ(REG_SPU_CH_CTRL) | DRVSPU_LOAD_SELECTED_CHANNEL);
		while(AUDIO_READ(REG_SPU_CH_CTRL) & CH_FN);
		
		u8DataFormat = AUDIO_READ(REG_SPU_CH_PAR_1);
		return (u8DataFormat & 0x07);
	}
	else
		return 0;
}
#endif	

int DrvSPU_SetChannelVolume(
	u32 u32Channel, 
	u8 	u8Volume
)
{
	u32 u32PAN;
	
	if ( (u32Channel >=eDRVSPU_CHANNEL_0) && (u32Channel <=eDRVSPU_CHANNEL_31) )
	{
		// wait to finish previous channel settings
		while(AUDIO_READ(REG_SPU_CH_CTRL) & CH_FN);
		
		// load previous channel settings		
		AUDIO_WRITE(REG_SPU_CH_CTRL, (AUDIO_READ(REG_SPU_CH_CTRL) & ~CH_NO) | (u32Channel << 24));		
		AUDIO_WRITE(REG_SPU_CH_CTRL, AUDIO_READ(REG_SPU_CH_CTRL) | DRVSPU_LOAD_SELECTED_CHANNEL);
		while(AUDIO_READ(REG_SPU_CH_CTRL) & CH_FN);
		
		u32PAN = u8Volume;
		u32PAN <<= 24;
		AUDIO_WRITE(REG_SPU_CH_PAR_1, (AUDIO_READ(REG_SPU_CH_PAR_1) & 0x00FFFFFF) | u32PAN);		

		AUDIO_WRITE(REG_SPU_CH_CTRL, AUDIO_READ(REG_SPU_CH_CTRL) & ~DRVSPU_UPDATE_ALL_PARTIALS);		
		AUDIO_WRITE(REG_SPU_CH_CTRL, AUDIO_READ(REG_SPU_CH_CTRL) | (DRVSPU_UPDATE_VOL_PARTIAL + DRVSPU_UPDATE_PARTIAL_SETTINGS));				

		while(AUDIO_READ(REG_SPU_CH_CTRL) & CH_FN);		
		
		return E_SUCCESS;
	}
	 
	else
		return E_DRVSPU_WRONG_CHANNEL;	   
}

static int DrvSPU_EnableInt(
	u32 u32Channel, 
	u32 u32InterruptFlag 
)
{
	
	if ( (u32Channel >=eDRVSPU_CHANNEL_0) && (u32Channel <=eDRVSPU_CHANNEL_31) )
	{
		// wait to finish previous channel settings
		while(AUDIO_READ(REG_SPU_CH_CTRL) & CH_FN);
		
		// load previous channel settings		
		AUDIO_WRITE(REG_SPU_CH_CTRL, (AUDIO_READ(REG_SPU_CH_CTRL) & ~CH_NO) | (u32Channel << 24));		
		AUDIO_WRITE(REG_SPU_CH_CTRL, AUDIO_READ(REG_SPU_CH_CTRL) | DRVSPU_LOAD_SELECTED_CHANNEL);
		while(AUDIO_READ(REG_SPU_CH_CTRL) & CH_FN);

		MSG2("*** DrvSPU_EnableInt *** \n");
		MSG2("*** download channel data *** \n");	
		MSG2("==>REG_SPU_CH_CTRL=0x%x\n", AUDIO_READ(REG_SPU_CH_CTRL));
		MSG2("==>REG_SPU_CH_EVENT=0x%x\n", AUDIO_READ(REG_SPU_CH_EVENT));		
		
		// set new channel settings for previous channel settings						
		if (u32InterruptFlag & DRVSPU_USER_INT)
		{
			AUDIO_WRITE(REG_SPU_CH_EVENT, AUDIO_READ(REG_SPU_CH_EVENT) | EV_USR_EN);		
		}
		if (u32InterruptFlag & DRVSPU_SILENT_INT)
		{
			AUDIO_WRITE(REG_SPU_CH_EVENT, AUDIO_READ(REG_SPU_CH_EVENT) | EV_SLN_EN);				
		}
		if (u32InterruptFlag & DRVSPU_LOOPSTART_INT)
		{
			AUDIO_WRITE(REG_SPU_CH_EVENT, AUDIO_READ(REG_SPU_CH_EVENT) | EV_LP_EN);						
		}
		if (u32InterruptFlag & DRVSPU_END_INT)
		{
			AUDIO_WRITE(REG_SPU_CH_EVENT, AUDIO_READ(REG_SPU_CH_EVENT) | EV_END_EN);						
		}

		if (u32InterruptFlag & DRVSPU_ENDADDRESS_INT)
		{
			AUDIO_WRITE(REG_SPU_CH_EVENT, AUDIO_READ(REG_SPU_CH_EVENT) | END_EN);						
		}

		if (u32InterruptFlag & DRVSPU_THADDRESS_INT)
		{
			AUDIO_WRITE(REG_SPU_CH_EVENT, AUDIO_READ(REG_SPU_CH_EVENT) | TH_EN);														
		}
		AUDIO_WRITE(REG_SPU_CH_EVENT, AUDIO_READ(REG_SPU_CH_EVENT) & ~AT_CLR_EN);
		AUDIO_WRITE(REG_SPU_CH_CTRL, AUDIO_READ(REG_SPU_CH_CTRL) & ~DRVSPU_UPDATE_ALL_PARTIALS);		
		AUDIO_WRITE(REG_SPU_CH_CTRL, AUDIO_READ(REG_SPU_CH_CTRL) | (DRVSPU_UPDATE_IRQ_PARTIAL + DRVSPU_UPDATE_PARTIAL_SETTINGS));				
		while(AUDIO_READ(REG_SPU_CH_CTRL) & CH_FN);		
		
	MSG2("*** upload channel data *** \n");	
	MSG2("==>REG_SPU_CH_CTRL=0x%x\n", AUDIO_READ(REG_SPU_CH_CTRL));
	MSG2("==>REG_SPU_CH_EVENT=0x%x\n", AUDIO_READ(REG_SPU_CH_EVENT));		
		
		
		return E_SUCCESS;
	}
	 
	else
		return E_DRVSPU_WRONG_CHANNEL;	   
}

static int DrvSPU_DisableInt(
	u32 u32Channel, 
	u32 u32InterruptFlag
)
{
	if ( (u32Channel >=eDRVSPU_CHANNEL_0) && (u32Channel <=eDRVSPU_CHANNEL_31) )
	{
		// wait to finish previous channel settings
		MSG2("wait to finish previous channel settings  11\n"); 						
		while(AUDIO_READ(REG_SPU_CH_CTRL) & CH_FN);
		
		// load previous channel settings		
		MSG2("load previous channel settings  11\n"); 						
		AUDIO_WRITE(REG_SPU_CH_CTRL, (AUDIO_READ(REG_SPU_CH_CTRL) & ~CH_NO) | (u32Channel << 24));		
		AUDIO_WRITE(REG_SPU_CH_CTRL, AUDIO_READ(REG_SPU_CH_CTRL) | DRVSPU_LOAD_SELECTED_CHANNEL);
		while(AUDIO_READ(REG_SPU_CH_CTRL) & CH_FN);
		
		// set new channel settings for previous channel settings						
		MSG2("set new channel settings for previous channel settings 11\n");		
		if (u32InterruptFlag & DRVSPU_USER_INT)
		{
			AUDIO_WRITE(REG_SPU_CH_EVENT, AUDIO_READ(REG_SPU_CH_EVENT) & ~EV_USR_EN);		
		}
		if (u32InterruptFlag & DRVSPU_SILENT_INT)
		{
			AUDIO_WRITE(REG_SPU_CH_EVENT, AUDIO_READ(REG_SPU_CH_EVENT) & ~EV_SLN_EN);				
		}
		if (u32InterruptFlag & DRVSPU_LOOPSTART_INT)
		{
			AUDIO_WRITE(REG_SPU_CH_EVENT, AUDIO_READ(REG_SPU_CH_EVENT) & ~EV_LP_EN);						
		}
		if (u32InterruptFlag & DRVSPU_END_INT)
		{
			AUDIO_WRITE(REG_SPU_CH_EVENT, AUDIO_READ(REG_SPU_CH_EVENT) & ~EV_END_EN);						
		}
		if (u32InterruptFlag & DRVSPU_ENDADDRESS_INT)
		{
			AUDIO_WRITE(REG_SPU_CH_EVENT, AUDIO_READ(REG_SPU_CH_EVENT) & ~END_EN);						
		}
		if (u32InterruptFlag & DRVSPU_THADDRESS_INT)
		{
			AUDIO_WRITE(REG_SPU_CH_EVENT, AUDIO_READ(REG_SPU_CH_EVENT) & ~TH_EN);
		}
		AUDIO_WRITE(REG_SPU_CH_CTRL, AUDIO_READ(REG_SPU_CH_CTRL) & ~DRVSPU_UPDATE_ALL_PARTIALS);		
		AUDIO_WRITE(REG_SPU_CH_CTRL, AUDIO_READ(REG_SPU_CH_CTRL) | (DRVSPU_UPDATE_IRQ_PARTIAL + DRVSPU_UPDATE_PARTIAL_SETTINGS));				
		
		while(AUDIO_READ(REG_SPU_CH_CTRL) & CH_FN);
	
		return E_SUCCESS;
	}
	 
	else
		return E_DRVSPU_WRONG_CHANNEL;	   
}


static int DrvSPU_ClearInt(
	u32 u32Channel, 
	u32 u32InterruptFlag
)
{
	if ( (u32Channel >=eDRVSPU_CHANNEL_0) && (u32Channel <=eDRVSPU_CHANNEL_31) )
	{
		// wait to finish previous channel settings
		MSG2("wait to finish previous channel settings\n"); 				
		while(AUDIO_READ(REG_SPU_CH_CTRL) & CH_FN);
		
		// load previous channel settings		
		MSG2("load previous channel settings\n"); 				
		AUDIO_WRITE(REG_SPU_CH_CTRL, (AUDIO_READ(REG_SPU_CH_CTRL) & ~CH_NO) | (u32Channel << 24));		
		AUDIO_WRITE(REG_SPU_CH_CTRL, AUDIO_READ(REG_SPU_CH_CTRL) | DRVSPU_LOAD_SELECTED_CHANNEL);
		while(AUDIO_READ(REG_SPU_CH_CTRL) & CH_FN);

		MSG2("*** DrvSPU_ClearInt *** \n");
		MSG2("*** download channel data *** \n");	
		MSG2("==>REG_SPU_CH_CTRL=0x%x\n", AUDIO_READ(REG_SPU_CH_CTRL));
		MSG2("==>REG_SPU_CH_EVENT=0x%x\n", AUDIO_READ(REG_SPU_CH_EVENT));		
		
		// set new channel settings for previous channel settings
		MSG2("set new channel settings for previous channel settings\n");
		if (u32InterruptFlag & DRVSPU_USER_INT)
		{
			AUDIO_WRITE(REG_SPU_CH_EVENT, (AUDIO_READ(REG_SPU_CH_EVENT) & ~0x3F00) | EV_USR_FG);		
		}
		if (u32InterruptFlag & DRVSPU_SILENT_INT)
		{
			AUDIO_WRITE(REG_SPU_CH_EVENT, (AUDIO_READ(REG_SPU_CH_EVENT) & ~0x3F00) | EV_SLN_FG);				
		}
		if (u32InterruptFlag & DRVSPU_LOOPSTART_INT)
		{
			AUDIO_WRITE(REG_SPU_CH_EVENT, (AUDIO_READ(REG_SPU_CH_EVENT) & ~0x3F00) | EV_LP_FG);						
		}
		if (u32InterruptFlag & DRVSPU_END_INT)
		{
			AUDIO_WRITE(REG_SPU_CH_EVENT, (AUDIO_READ(REG_SPU_CH_EVENT) & ~0x3F00) | EV_END_FG);						
		}
		if (u32InterruptFlag & DRVSPU_ENDADDRESS_INT)
		{
			AUDIO_WRITE(REG_SPU_CH_EVENT, (AUDIO_READ(REG_SPU_CH_EVENT) & ~0x3F00) | END_FG);						
		}
		if (u32InterruptFlag & DRVSPU_THADDRESS_INT)
		{
			AUDIO_WRITE(REG_SPU_CH_EVENT, (AUDIO_READ(REG_SPU_CH_EVENT) & ~0x3F00) | TH_FG);														
		}
		AUDIO_WRITE(REG_SPU_CH_CTRL, AUDIO_READ(REG_SPU_CH_CTRL) & ~DRVSPU_UPDATE_ALL_PARTIALS);		
		AUDIO_WRITE(REG_SPU_CH_CTRL, AUDIO_READ(REG_SPU_CH_CTRL) | (DRVSPU_UPDATE_IRQ_PARTIAL + DRVSPU_UPDATE_PARTIAL_SETTINGS));				
		
		MSG2("wait to finish previous channel settings 00\n"); 						
		while(AUDIO_READ(REG_SPU_CH_CTRL) & CH_FN);		
		
		MSG2("wait to finish previous channel settings OK\n"); 								
	
		MSG2("*** upload channel data *** \n");	
		MSG2("==>REG_SPU_CH_CTRL=0x%x\n", AUDIO_READ(REG_SPU_CH_CTRL));
		MSG2("==>REG_SPU_CH_EVENT=0x%x\n", AUDIO_READ(REG_SPU_CH_EVENT));		
	
		return E_SUCCESS;
	}
	else
	{
		MSG2("WORNG CHANNEL\n"); 									
		return E_DRVSPU_WRONG_CHANNEL;	   
	}		
		
}

static int DrvSPU_ChannelOpen(
	u32 u32Channel
)
{
	if ( (u32Channel >=eDRVSPU_CHANNEL_0) && (u32Channel <=eDRVSPU_CHANNEL_31) )
	{
		AUDIO_WRITE(REG_SPU_CH_EN, AUDIO_READ(REG_SPU_CH_EN) | (0x0001 << u32Channel));
		return E_SUCCESS;
	}
	else		
		return E_DRVSPU_WRONG_CHANNEL;	   	
}

static int DrvSPU_ChannelClose(
	u32 u32Channel
)
{
	if ( (u32Channel >=eDRVSPU_CHANNEL_0) && (u32Channel <=eDRVSPU_CHANNEL_31) )
	{
		AUDIO_WRITE(REG_SPU_CH_EN, AUDIO_READ(REG_SPU_CH_EN) & ~(0x0001 << u32Channel));
		return E_SUCCESS;
	}		
	else		
		return E_DRVSPU_WRONG_CHANNEL;	   	
}

static int spuInit(void)
{
	int ii;

		ENTER();

		MSG2("init SPU register BEGIN !!\n");
			
		// disable SPU engine 
		AUDIO_WRITE(REG_SPU_CTRL, 0x00);
		
		// given FIFO size = 4
		AUDIO_WRITE(REG_SPU_CTRL, 0x04000000);		
	
		// reset SPU engine 
	//	AUDIO_WRITE(REG_SPU_CTRL, AUDIO_READ(REG_SPU_CTRL) | SPU_EN);
		AUDIO_WRITE(REG_SPU_CTRL, AUDIO_READ(REG_SPU_CTRL) & ~SPU_SWRST);	
		
		AUDIO_WRITE(REG_SPU_CTRL, AUDIO_READ(REG_SPU_CTRL) | SPU_SWRST);
		AUDIO_WRITE(REG_SPU_CTRL, AUDIO_READ(REG_SPU_CTRL) & ~SPU_SWRST);	
		
		// disable all channels
		AUDIO_WRITE(REG_SPU_CH_EN, 0x00);		

		for (ii=0; ii<32; ii++)
		{
			DrvSPU_ClearInt(ii, DRVSPU_ALL_INT);
			DrvSPU_DisableInt(ii, DRVSPU_ALL_INT);
		}
		
		LEAVE();
	
	return 0;	
}

#if 0
static int spuSetPcmVolume(int ucLeftVol, int ucRightVol)
{
	ENTER();
	MSG2("Set PCM volume to : %d-%d\n", ucLeftVol, ucRightVol);
	
	//save the flag that ap already sets the volume
	_bApuVolumeActive = 1;
	
	ucLeftVol = spuNormalizeVolume(ucLeftVol);
	ucRightVol = spuNormalizeVolume(ucRightVol);
	
	DrvSPU_WriteDACReg(0x09, ucLeftVol);		// left digital volume control
	DrvSPU_WriteDACReg(0x0A, ucRightVol);		// right digital volume control
	
	LEAVE();
	return 0;
}
#endif

static int spuStartPlay(int nChannels)
{
	ENTER();

	if (_bSpuActive & SPU_PLAY_ACTIVE)
		return -1;		

	
	/* set default pcm volume */
//	if(!_bApuVolumeActive)
//		spuSetPcmVolume(15,15);

//#define OPT_SPU_TEST_20151023
#ifdef OPT_SPU_TEST_20151023
	printk("Software reset I2S Interface");
	DrvSPU_WriteDACReg(0x08, 0x01);	                   
    DrvSPU_WriteDACReg(0x08, 0x03);
#endif


	if(!_bApuVolumeActive)
	{
		_bApuVolumeActive = 1;
		DrvSPU_WriteDACReg(0x09, 0x80);		// left digital volume control
		DrvSPU_WriteDACReg(0x0A, 0x80);		// right digital volume control
	}		

	/* limit digital gain to 0x7A to avoid DAC becoming unstable */
	DrvSPU_WriteDACReg(0x09, 0x7A);
	DrvSPU_WriteDACReg(0x0A, 0x7A);		
	
	/* start playing */
	MSG("SPU start playing...\n");
		
	_bPlayDmaToggle = 0;
	
	if (nChannels ==1)
	{
		DrvSPU_ChannelOpen(_u8Channel0);		
		DrvSPU_ChannelOpen(_u8Channel1);
	}
	else
	{	
		DrvSPU_ChannelOpen(_u8Channel0);	//left channel 
		DrvSPU_ChannelOpen(_u8Channel1);	// right channel
	}				
	DrvSPU_SetChannelVolume(_u8Channel0, 0x7f);
	DrvSPU_SetChannelVolume(_u8Channel1, 0x7f);		

	/* set DFA */
	DrvSPU_SetDFA(_u8Channel0, 0x400);		// FA92 can be fixed to 0x400			
	DrvSPU_SetDFA(_u8Channel1, 0x400);			

	/* enable interrupt */
	DrvSPU_ClearInt(_u8Channel0, DRVSPU_ALL_INT);
	DrvSPU_DisableInt(_u8Channel0, DRVSPU_ALL_INT);		
	DrvSPU_ClearInt(_u8Channel1, DRVSPU_ALL_INT);
	DrvSPU_DisableInt(_u8Channel1, DRVSPU_ALL_INT);		
	
	
	if (nChannels ==1)
	{
		DrvSPU_EnableInt(_u8Channel0, DRVSPU_ENDADDRESS_INT);
		DrvSPU_EnableInt(_u8Channel0, DRVSPU_THADDRESS_INT);		
	}
	else
	{	/* just open one channel interrupt */
		DrvSPU_EnableInt(_u8Channel0, DRVSPU_ENDADDRESS_INT);
		DrvSPU_EnableInt(_u8Channel0, DRVSPU_THADDRESS_INT);		
	}				

	AUDIO_WRITE(REG_SPU_CH_IRQ,AUDIO_READ(REG_SPU_CH_IRQ));		
	
	_bSpuActive |= SPU_PLAY_ACTIVE;
	
	AUDIO_WRITE(REG_AIC_MECR,(1<<IRQ_SPU));
	

	LEAVE();
	return 0;
}

static void spuStopPlay(void)
{
	
	ENTER();
	
	if (!(_bSpuActive & SPU_PLAY_ACTIVE))
		return;

#if 0		
	if(_bApuVolumeActive)	//save the volume before reset audio engine
		volume = AUDIO_READ(REG_SPU_DAC_VOL) & (LHPVL | RHPVL);		
#endif		


	/* channel close (before SPU disabled) */
	DrvSPU_ChannelClose(_u8Channel0);	//left channel 
	DrvSPU_ChannelClose(_u8Channel1);	// right channel
			
	/* disable audio play interrupt */
	if (!_bSpuActive)
 	{		
		AUDIO_WRITE(REG_AIC_MDCR,(1<<IRQ_SPU));
		AUDIO_WRITE(REG_AIC_MECR,(1<<IRQ_SPU));		
	}
	
	/* disable SPU (after channel being closed) */
	AUDIO_WRITE(REG_SPU_CTRL, AUDIO_READ(REG_SPU_CTRL) & ~SPU_EN);	/*disable spu*/	
	
	/* reset SPU engine */
	AUDIO_WRITE(REG_SPU_CTRL, AUDIO_READ(REG_SPU_CTRL) & ~SPU_SWRST);	
	AUDIO_WRITE(REG_SPU_CTRL, AUDIO_READ(REG_SPU_CTRL) | SPU_SWRST);
	AUDIO_WRITE(REG_SPU_CTRL, AUDIO_READ(REG_SPU_CTRL) & ~SPU_SWRST);	

	//restore volume
#if 0	
	if(_bApuVolumeActive)	// if ever set the volume then restore it 
		AUDIO_WRITE(REG_SPU_DAC_VOL, (AUDIO_READ(REG_SPU_DAC_VOL) | volume));
#endif		

	_bSpuActive &= ~SPU_PLAY_ACTIVE;       
	
	LEAVE();
}

static void  spuSetPlaySampleRate(int nSamplingRate)
{
	
	ENTER();
	
#ifndef CONFIG_SPU_WITH_I2S_OUTPUT 
	
		AUDIO_WRITE(REG_SPU_DAC_PAR, AUDIO_READ(REG_SPU_DAC_PAR) | PLLSELMOD);
	
		nvt_lock();
		AUDIO_WRITE(REG_CLKDIV1, AUDIO_READ(REG_CLKDIV1) & ~(ADO_N1 | ADO_S | ADO_N0));	// engine clcok fixed to 12 MHz
		nvt_unlock();
	
		switch(nSamplingRate)
		{
			case  eDRVSPU_FREQ_48000: 
				DrvSPU_WriteDACReg(0x02, 0x00);			
				DrvSPU_WriteDACReg(0x0B, 0x14);
				DrvSPU_WriteDACReg(0x0C, 0x55);	
		        break;
		        
			case  eDRVSPU_FREQ_44100: 
		  		DrvSPU_WriteDACReg(0x02, 0x00);		       
				DrvSPU_WriteDACReg(0x0B, 0x10);
				DrvSPU_WriteDACReg(0x0C, 0x3F);	
		        break;	    

			case  eDRVSPU_FREQ_32000: 
				DrvSPU_WriteDACReg(0x02, 0x04);			
				DrvSPU_WriteDACReg(0x0B, 0x14);			
				DrvSPU_WriteDACReg(0x0C, 0x55);	
		        break;	
		        
			case  eDRVSPU_FREQ_24000: 
				DrvSPU_WriteDACReg(0x02, 0x01);			
				DrvSPU_WriteDACReg(0x0B, 0x14);			
				DrvSPU_WriteDACReg(0x0C, 0x55);	
		        break;	
		        
			case  eDRVSPU_FREQ_22050: 
				DrvSPU_WriteDACReg(0x02, 0x01);			
				DrvSPU_WriteDACReg(0x0B, 0x10);			
				DrvSPU_WriteDACReg(0x0C, 0x3F);	
		        break;	
		        
			case  eDRVSPU_FREQ_16000: 
				DrvSPU_WriteDACReg(0x02, 0x05);			
				DrvSPU_WriteDACReg(0x0B, 0x14);			
				DrvSPU_WriteDACReg(0x0C, 0x55);	
		        break;	
		        
			case  eDRVSPU_FREQ_12000: 
				DrvSPU_WriteDACReg(0x02, 0x02);			
				DrvSPU_WriteDACReg(0x0B, 0x14);			
				DrvSPU_WriteDACReg(0x0C, 0x55);	
		        break;	
		        
			case  eDRVSPU_FREQ_11025: 
				DrvSPU_WriteDACReg(0x02, 0x02);			
				DrvSPU_WriteDACReg(0x0B, 0x10);			
				DrvSPU_WriteDACReg(0x0C, 0x3F);	
		        break;
		        
			case  eDRVSPU_FREQ_8000: 
			default:
				DrvSPU_WriteDACReg(0x02, 0x06);			
				DrvSPU_WriteDACReg(0x0B, 0x14);			
				DrvSPU_WriteDACReg(0x0C, 0x55);	
		        break;
		}

#else	// CONFIG_SPU_WITH_I2S_OUTPUT 

		int PllFreq, u32ClockDivider, u32Divider0;
		int ii;

		switch(nSamplingRate)
		{
		//	case  eDRVSPU_FREQ_192000: 			
			case  eDRVSPU_FREQ_96000: 
			case  eDRVSPU_FREQ_64000: 			
			case  eDRVSPU_FREQ_48000: 			
			case  eDRVSPU_FREQ_32000: 			
			case  eDRVSPU_FREQ_24000: 
			case  eDRVSPU_FREQ_12000:
				w55fa92_set_apll_clock(344064);			
		        break;
		        
			case  eDRVSPU_FREQ_16000: 				
			case  eDRVSPU_FREQ_8000: 			 						
				w55fa92_set_apll_clock(344064);			
		        break;		        
		        
			case  eDRVSPU_FREQ_88200: 
			case  eDRVSPU_FREQ_44100: 			
				w55fa92_set_apll_clock(316108);
				break;

			case  eDRVSPU_FREQ_22050: 
				w55fa92_set_apll_clock(254016);
				break;

			case  eDRVSPU_FREQ_11025: 						
				w55fa92_set_apll_clock(254016);
				break;
	
			default:		
				printk("SampleRate not support\n");
				break;		        
		}
	
		PllFreq = w55fa92_apll_clock;
		PllFreq *= 1000;
		u32ClockDivider = (PllFreq / (256*nSamplingRate));
		
	//#define OPT_FPGA_DEBUG
	#ifdef OPT_FPGA_DEBUG
		PllFreq = 27000000;
		u32ClockDivider = PllFreq / (256*nSamplingRate);
	#endif

		// clock_in of CLKDIV1_N1 divider must be less than 60 MHz
		for (ii=8; ii>1; ii--)
		{
			if (!(u32ClockDivider%ii))
				break;
		}			

		if (ii==1)
			printk("SampleRate not support\n");		
		else
		{
			u32Divider0 = ii;
			u32ClockDivider = u32ClockDivider / ii;
		}			
		
		nvt_lock();
			
		AUDIO_WRITE(REG_CLKDIV1, (AUDIO_READ(REG_CLKDIV1) & (~ADO_S)) | (0x02 << 19) );	// SPU clock from APLL	

		u32Divider0 --;
		AUDIO_WRITE(REG_CLKDIV1, (AUDIO_READ(REG_CLKDIV1) & (~ADO_N0)) | (u32Divider0<<16));	
		u32ClockDivider &= 0xFF;
		u32ClockDivider	--;
		AUDIO_WRITE(REG_CLKDIV1, (AUDIO_READ(REG_CLKDIV1) & (~ADO_N1)) | (u32ClockDivider<<24));	
	
		nvt_unlock();	
		
//		DrvSPU_WriteDACReg(0x05, DrvSPU_ReadDACReg(0x05)|0x80);		// bypass DAC PLL
		DrvSPU_WriteDACReg(0x05, DrvSPU_ReadDACReg(0x05)|0xC0);		// bypass DAC PLL, PLL power down
		AUDIO_WRITE(REG_SPU_DAC_PAR, AUDIO_READ(REG_SPU_DAC_PAR) & ~PLLSELMOD);

//		printk("SampleRate = %d !!!\n", nSamplingRate);		
//		printk("w55fa92_apll_clock = %d !!!\n", w55fa92_apll_clock);			
//		printk("REG_CLKDIV1 = 0x%x !!!\n", AUDIO_READ(REG_CLKDIV1));		
		
#endif	// CONFIG_SPU_WITH_I2S_OUTPUT 	
		
	#ifdef CONFIG_SPU_WITH_I2S_OUTPUT 
		nvt_lock();
		AUDIO_WRITE(REG_CLKDIV7, (AUDIO_READ(REG_CLKDIV7) & (~I2S_S)) | (0x02 << 11) );	// SPU clock from APLL	
		AUDIO_WRITE(REG_CLKDIV7, (AUDIO_READ(REG_CLKDIV7) & (~I2S_N0)) | (u32Divider0<<8));			
		AUDIO_WRITE(REG_CLKDIV7, (AUDIO_READ(REG_CLKDIV7) & (~I2S_N1)) | (u32ClockDivider<<16));		
		nvt_unlock();		
	#endif
			
	MSG2("==>REG_CLKDIV1=0x%x\n", AUDIO_READ(REG_CLKDIV1));			

	LEAVE();
}

void spuDacOn(void)
{
	volatile int s_clkdiv3, buf;
	
		// enable ADC VMID bias
		s_clkdiv3 = AUDIO_READ(REG_CLKDIV3);
	    clk_enable(clk_adc);        	
		nvt_lock();	    
		AUDIO_WRITE(REG_CLKDIV3, (AUDIO_READ(REG_CLKDIV3) & ~(ADC_N1 | ADC_S| ADC_N0)) );	// ADC in 12 mHz clock in
	    DrvAUR_AudioI2cRead(0x21, (char*) &buf);
		DrvAUR_AudioI2cWrite(0x21, buf & ~0x40); // enable ADC VMID bias       
		AUDIO_WRITE(REG_CLKDIV3, s_clkdiv3);	// restore CLKDIV3 
		nvt_unlock();		
	    clk_disable(clk_adc);        		
    
		// enable SPU DAC 
		DrvSPU_WriteDACReg(0x07, 0x01);		// bit-1 is the digital interface of DAC
		DrvSPU_WriteDACReg(0x05, 0xDF);
		mdelay(700);
		DrvSPU_WriteDACReg(0x05, 0x13);
		DrvSPU_WriteDACReg(0x05, 0x10);
		DrvSPU_WriteDACReg(0x08, 0x01);		// bit-1 of reg. 0x08 is the reset bit	
		DrvSPU_WriteDACReg(0x08, 0x03);		// bit-1 of reg. 0x08 is the reset bit	
}


void spuDacOff(void)
{		
	volatile int buf;
		
		DrvSPU_WriteDACReg(0x07, 0x00);		// digital DAC disabled
		DrvSPU_WriteDACReg(0x05, 0x13);		// L/R headphone power OFF (analog)
		mdelay(200);		
		DrvSPU_WriteDACReg(0x05, 0x3F);		// reference voltage, DAC L/R channl power OFF (analog), pre-charge/PLL/Bypass mode power OFF
		mdelay(200);		
	    DrvAUR_AudioI2cRead(0x21, (char*) &buf);		
		DrvAUR_AudioI2cWrite(0x21, buf | 0x40); // disable ADC VMID bias       		
		DrvSPU_WriteDACReg(0x05, 0xFF);		// reference voltage power OFF
		mdelay(1000);			
}
static int DrvAUR_AudioI2cRead(int u32Addr, char* p8Data)
{
	int u32AGCEnable = 0;
	int u32Ctrl = 0;	
	if(AUDIO_READ(REG_AR_AGC1)&AGC_EN){	
		u32AGCEnable = 1;
		AUDIO_WRITE(REG_AR_AGC1, (AUDIO_READ(REG_AR_AGC1) & ~AGC_EN)); /* Disable AGC */
	}
	/* if i2C is working, the i2c keep working, so program still need to check the busy bit */
	do
	{		
	
	}while((AUDIO_READ(REG_SDADC_CTL)&AR_BUSY) != 0);		
	u32Addr = u32Addr & 0xFF;
	u32Ctrl = (u32Ctrl | (0x800000 | (u32Addr<<8)));
	u32Ctrl = u32Ctrl | 0x20000000;
	AUDIO_WRITE(REG_SDADC_CTL, u32Ctrl);
	do
	{		
	}while((AUDIO_READ(REG_SDADC_CTL)&AR_BUSY) != 0);
	*p8Data = AUDIO_READ(REG_SDADC_CTL)&0xFF;
	if(u32AGCEnable == 1)	
		AUDIO_WRITE(REG_AR_AGC1, (AUDIO_READ(REG_AR_AGC1) | AGC_EN)); /* Enable AGC */	
	return 0;
}

static int DrvAUR_AudioI2cWrite(int u32Addr, int u32Data)
{
	int u32AGCEnable = 0;
	int u32Ctrl = 0;
	if(AUDIO_READ(REG_AR_AGC1)&AGC_EN){	
		u32AGCEnable = 1;
		AUDIO_WRITE(REG_AR_AGC1, (AUDIO_READ(REG_AR_AGC1) & ~AGC_EN)); /* Disable AGC */
	}

	/* if i2C keeps working, so program still need to check the busy bit */
	do
	{		
	}while((AUDIO_READ(REG_SDADC_CTL)&AR_BUSY) != 0);		 
	
	if( u32Addr == 0x22 ){//Left //Need to keep pre-gain if H22
		if( (u32Data&0x0F) ==0 )
			u32Data = u32Data | 0xF; //Mute
		else
			u32Data = (0x10) | ((u32Data&0x0F) -1);
	}
	if(u32Addr==0x23){//Right
		if(u32Data&BIT4)
			AUDIO_WRITE(REG_SDADC_AGBIT, AUDIO_READ(REG_SDADC_AGBIT) | 0x1);
		else
			AUDIO_WRITE(REG_SDADC_AGBIT, AUDIO_READ(REG_SDADC_AGBIT) & ~0x1);	
	}	
	u32Addr = u32Addr & 0xFF;
	u32Data = u32Data & 0xFF;

	u32Ctrl = u32Ctrl | (0x810000 | (u32Addr<<8)  | u32Data) ;
	u32Ctrl = u32Ctrl | 0x20000000;
	AUDIO_WRITE(REG_SDADC_CTL, u32Ctrl);		
	
	do
	{		
	}while((AUDIO_READ(REG_SDADC_CTL)&AR_BUSY) != 0);

	//Update the cache 
//	if(u32Addr == 0x22)
//		w55fa92adc->reg_cache[0] = u32Data;
//	else if(u32Addr == 0x23)
//		w55fa92adc->reg_cache[1] = u32Data;

	if(u32AGCEnable == 1)
		AUDIO_WRITE(REG_AR_AGC1, (AUDIO_READ(REG_AR_AGC1) | AGC_EN)); /* Enable AGC */	
	return 0;
}

EXPORT_SYMBOL_GPL(spuDacOn);
EXPORT_SYMBOL_GPL(spuDacOff);


module_init(w55fa92_spu_init);
module_exit(w55fa92_spu_exit);

MODULE_AUTHOR("Wan ZongShun <mcuos.com@gmail.com>");
MODULE_DESCRIPTION("W55FA92 IIS SoC driver!");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:w55fa92-spu");

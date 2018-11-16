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
#include <mach/map.h>
#include <mach/regs-clock.h>

#include <sound/pcm_params.h>
#include <mach/hardware.h>

#include <mach/w55fa92_audio.h>
#include <mach/w55fa92_i2s.h>
#include <mach/w55fa92_reg.h>


//#define OPT_FPGA_DEBUG

//#define I2S_DEBUG
#define I2S_DEBUG_ENTER_LEAVE
#define I2S_DEBUG_MSG
#define I2S_DEBUG_MSG2

#ifdef I2S_DEBUG
#define DBG(fmt, arg...)			printk(fmt, ##arg)
#else
#define DBG(fmt, arg...)
#endif


#ifdef I2S_DEBUG_ENTER_LEAVE
#define ENTER()					DBG("[%-10s] : Enter\n", __FUNCTION__)
#define LEAVE()					DBG("[%-10s] : Leave\n", __FUNCTION__)
#else
#define ENTER()
#define LEAVE()
#endif


static DEFINE_MUTEX(i2s_mutex);
struct w55fa92_audio *w55fa92_i2s_data;
extern unsigned int w55fa92_apll_clock;
extern unsigned int w55fa92_upll_clock;
extern int w55fa92_set_apll_clock(unsigned int clock);
#if defined(CONFIG_SND_SOC_W55FA92_I2S_MODULE)
EXPORT_SYMBOL(w55fa92_i2s_data);
#endif

static struct clk *clk_hclk4 = NULL;

extern void nvt_lock(void);
extern void nvt_unlock(void);

static int w55fa92_i2s_set_fmt(struct snd_soc_dai *cpu_dai,
                              unsigned int fmt)
{
 //       struct w55fa92_audio *w55fa92_audio = w55fa92_i2s_data;
        unsigned long val = 0;

		ENTER();

		val = AUDIO_READ(REG_I2S_ACTL_I2SCON);
		
        switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
        case SND_SOC_DAIFMT_MSB:
                val |= MSB_Justified;
                break;
        case SND_SOC_DAIFMT_I2S:
                val &= ~MSB_Justified;
                break;
        default:
                return -EINVAL;
        }

        AUDIO_WRITE(REG_I2S_ACTL_I2SCON, val);

		LEAVE();
		
        return 0;
}

static int w55fa92_i2s_set_sysclk(struct snd_soc_dai *cpu_dai,
                                 int clk_id, unsigned int freq, int dir)
{
        unsigned int val, ii, u32Divider0;
		unsigned int PllFreq;
		unsigned int u32MCLK, u32ClockDivider;

		ENTER();

        if (clk_id == W55FA92_AUDIO_SAMPLECLK) 
        {
			printk("freq = %d !!!\n", freq);
			switch (freq)	//all 16bit, 256fs
			{
				case AU_SAMPLE_RATE_8000:						//8KHz
					w55fa92_set_apll_clock(344064);
					u32MCLK = 12288/6;									
					break;
				case AU_SAMPLE_RATE_11025:						//11.025KHz
					w55fa92_set_apll_clock(254016);		
					u32MCLK = 16934/6;							
					break;
				case AU_SAMPLE_RATE_12000:						//12KHz
					w55fa92_set_apll_clock(344064);
					u32MCLK = 12288/4;
					break;
				case AU_SAMPLE_RATE_16000:						//16KHz
					w55fa92_set_apll_clock(344064);
					u32MCLK = 12288/3;
					break;
				case AU_SAMPLE_RATE_22050:						//22.05KHz
					w55fa92_set_apll_clock(254016);		
					u32MCLK = 16934/3;
					break;
				case AU_SAMPLE_RATE_24000:						//24KHz
					w55fa92_set_apll_clock(344064);
					u32MCLK = 12288/2;
					break;
				case AU_SAMPLE_RATE_32000:						//32KHz
					w55fa92_set_apll_clock(344064);
					u32MCLK = 16384/2;									
					break;
				case AU_SAMPLE_RATE_44100:						//44.1KHz
					w55fa92_set_apll_clock(316108);		
					u32MCLK = 16934*2/3;
					break;
				case AU_SAMPLE_RATE_48000:						//48KHz
					w55fa92_set_apll_clock(344064);
					u32MCLK = 12288;
					break;
				case AU_SAMPLE_RATE_64000:						//64KHz
					w55fa92_set_apll_clock(344064);
					u32MCLK = 16384;
					break;
				case AU_SAMPLE_RATE_88200:						//88.2KHz
					w55fa92_set_apll_clock(316108);		
					u32MCLK = 16934*4/3;
					break;
				case AU_SAMPLE_RATE_96000:						//96KHz
					w55fa92_set_apll_clock(344064);
					u32MCLK = 12288*2;
					break;
			//	case AU_SAMPLE_RATE_19200:						//192KHz
				default:		
					w55fa92_set_apll_clock(344064);
					u32MCLK = 12288;
					break;
					
			}
			val = AUDIO_READ(REG_I2S_ACTL_I2SCON) & 0x08;	
			AUDIO_WRITE(REG_I2S_ACTL_I2SCON, val);			
			
			PllFreq = w55fa92_apll_clock;
			u32ClockDivider = PllFreq / u32MCLK;
			
			for (ii=8; ii>1; ii--)
			{
				if (!(u32ClockDivider%ii))
					break;
			}			
			{
				u32Divider0 = ii;
				u32ClockDivider = u32ClockDivider / ii;
			}			
				
			u32Divider0 --;				
			u32ClockDivider	--;			

			nvt_lock();
			AUDIO_WRITE(REG_CLKDIV7, (AUDIO_READ(REG_CLKDIV7) & (~I2S_S)) | (0x02 << 11) );	// SPU clock from APLL	
			AUDIO_WRITE(REG_CLKDIV7, (AUDIO_READ(REG_CLKDIV7) & (~I2S_N0)) | (u32Divider0<<8));			
			AUDIO_WRITE(REG_CLKDIV7, (AUDIO_READ(REG_CLKDIV7) & (~I2S_N1)) | (u32ClockDivider<<16));		
			nvt_unlock();		
		}	// clk_id == W55FA92_AUDIO_SAMPLECLK
		
		LEAVE();
        return 0;
}

static int w55fa92_i2s_trigger(struct snd_pcm_substream *substream,
                              int cmd, struct snd_soc_dai *dai)
{
   //     struct w55fa92_audio *w55fa92_audio = w55fa92_i2s_data;
        int ret = 0;
        unsigned long val, con;

		ENTER();
		
        con = AUDIO_READ(REG_I2S_ACTL_CON);

        switch (cmd) {
        case SNDRV_PCM_TRIGGER_START:
        case SNDRV_PCM_TRIGGER_RESUME:

                val = AUDIO_READ(REG_I2S_ACTL_RESET);
                con |= I2S_EN;
                if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
     				val |= I2S_PLAY;         
                	con |= P_DMA_IRQ_EN; 
                } 
                else
                {
                    val |= I2S_RECORD;
               		con |= R_DMA_IRQ_EN;
                }
                AUDIO_WRITE(REG_I2S_ACTL_RESET, val);
                
                AUDIO_WRITE(REG_I2S_ACTL_RSR, AUDIO_READ(REG_I2S_ACTL_RSR));                
                AUDIO_WRITE(REG_I2S_ACTL_PSR, AUDIO_READ(REG_I2S_ACTL_PSR));                                
                AUDIO_WRITE(REG_I2S_ACTL_CON, con);
				AUDIO_WRITE(REG_AIC_MECR,(1<<IRQ_I2S));                
               break;
               
        case SNDRV_PCM_TRIGGER_STOP:
        case SNDRV_PCM_TRIGGER_SUSPEND:
                val = AUDIO_READ(REG_I2S_ACTL_RESET);
			//	con &= ~I2S_EN;
                if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
                		AUDIO_WRITE(REG_I2S_ACTL_PSR, AUDIO_READ(REG_I2S_ACTL_PSR));                                	                	
                        val &= ~I2S_PLAY;
                } else {
                		AUDIO_WRITE(REG_I2S_ACTL_RSR, AUDIO_READ(REG_I2S_ACTL_RSR));                                	
                        val &= ~I2S_RECORD;
                }
				if (!(AUDIO_READ(REG_I2S_ACTL_RESET)&(I2S_PLAY|I2S_RECORD)))
					con &= ~I2S_EN;

				AUDIO_WRITE(REG_I2S_ACTL_RESET, val);
                AUDIO_WRITE(REG_I2S_ACTL_CON, con);
                break;
                
        default:
                ret = -EINVAL;
        }

		LEAVE();
        return ret;
}

static int w55fa92_i2s_probe(struct platform_device *pdev,
                            struct snd_soc_dai *dai)
{
        struct w55fa92_audio *w55fa92_audio = w55fa92_i2s_data;
//        unsigned long val;

		ENTER();
		
        mutex_lock(&i2s_mutex);

        /* enable unit clock */
        clk_enable(clk_hclk4);        
        clk_enable(w55fa92_audio->eng_clk);        
        clk_enable(w55fa92_audio->i2s_clk);        

#if defined(CONFIG_SND_SOC_W55FA92_I2S_PIN2GPG)
		AUDIO_WRITE(REG_GPGFUN0, (AUDIO_READ(REG_GPGFUN0)&~(MF_GPG5+MF_GPG4+MF_GPG3+MF_GPG2))|0x00333300);	// GPG[5:2] to be I2S signals
		AUDIO_WRITE(REG_GPGFUN1, (AUDIO_READ(REG_GPGFUN1)&(~MF_GPG9))|0x20);	// GPG[9] to be I2S signals
		AUDIO_WRITE(REG_SHRPIN_TOUCH, AUDIO_READ(REG_SHRPIN_TOUCH)&(~SAR_AHS_AEN));
		AUDIO_WRITE(REG_SHRPIN_TVDAC, AUDIO_READ(REG_SHRPIN_TVDAC)&(~SMTVDACAEN));		
#else
		// enable I2S pins
		AUDIO_WRITE(REG_GPBFUN0, (AUDIO_READ(REG_GPBFUN0)&~(MF_GPB6+MF_GPB5+MF_GPB4+MF_GPB3+MF_GPB2))|0x01111100);	// GPB[6:2] to be I2S signals
#endif		
		AUDIO_WRITE(REG_MISFUN, AUDIO_READ(REG_MISFUN) & (~0x01));					// I2S interface for I2S, but not SPU

		// set Play & Record interrupt encountered in half of DMA buffer length
		AUDIO_WRITE(REG_I2S_ACTL_CON, (AUDIO_READ(REG_I2S_ACTL_CON) & (~R_DMA_IRQ_SEL)) | (0x01 << 14)); 	
		AUDIO_WRITE(REG_I2S_ACTL_CON, (AUDIO_READ(REG_I2S_ACTL_CON) & (~P_DMA_IRQ_SEL)) | (0x01 << 12)); 		

		// clear I2S interrupt flags
		AUDIO_WRITE(REG_I2S_ACTL_RSR, R_FIFO_FULL | R_FIFO_EMPTY | R_DMA_RIA_IRQ);	
		AUDIO_WRITE(REG_I2S_ACTL_PSR, 0x1F);	
		AUDIO_WRITE(REG_I2S_ACTL_CON, AUDIO_READ(REG_I2S_ACTL_CON) | I2S_EN | P_DMA_IRQ | R_DMA_IRQ); 		
//		AUDIO_WRITE(REG_I2S_ACTL_CON, AUDIO_READ(REG_I2S_ACTL_CON) | P_DMA_IRQ | R_DMA_IRQ); 		

        mutex_unlock(&i2s_mutex);

		LEAVE();
        return 0;
}

static void w55fa92_i2s_remove(struct platform_device *pdev,
                              struct snd_soc_dai *dai)
{
        struct w55fa92_audio *w55fa92_audio = w55fa92_i2s_data;

		ENTER();
		
		/* disable audio enigne clock */
        clk_disable(w55fa92_audio->i2s_clk);
        clk_disable(w55fa92_audio->eng_clk);
	    clk_disable(clk_hclk4);                
        clk_put(w55fa92_audio->i2s_clk);
        clk_put(w55fa92_audio->eng_clk);
	    clk_put(clk_hclk4);                
			        
		LEAVE();		
}

static struct snd_soc_dai_ops w55fa92_i2s_dai_ops = {
        .trigger	= w55fa92_i2s_trigger,
        .set_fmt	= w55fa92_i2s_set_fmt,
        .set_sysclk	= w55fa92_i2s_set_sysclk,
};

struct snd_soc_dai w55fa92_i2s_dai = {
        .name			= "w55fa92-audio-i2s",
        .id 			= 0,
        .probe			= w55fa92_i2s_probe,
        .remove			= w55fa92_i2s_remove,
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
        .ops = &w55fa92_i2s_dai_ops,
};
EXPORT_SYMBOL_GPL(w55fa92_i2s_dai);

#define OPT_AUDIO_MODULE
#ifdef OPT_AUDIO_MODULE
	static irqreturn_t w55fa92_dma_interrupt(int irq, void *dev_id)
	{
	        struct w55fa92_audio *w55fa92_audio = dev_id;
			unsigned long flags;
			int stream;
			u32 val;		

		ENTER();
	        spin_lock_irqsave(&w55fa92_audio->irqlock, flags);
	
	        val = AUDIO_READ(REG_I2S_ACTL_CON);
	        if (val & R_DMA_IRQ) {
	    		stream = SNDRV_PCM_STREAM_CAPTURE;    		
	
	            AUDIO_WRITE(REG_I2S_ACTL_RSR, AUDIO_READ(REG_I2S_ACTL_RSR));    		
	            AUDIO_WRITE(REG_I2S_ACTL_CON, val | R_DMA_IRQ);
	
	        } else if (val & P_DMA_IRQ) {
	        	stream = SNDRV_PCM_STREAM_PLAYBACK;        	
	        	
	            AUDIO_WRITE(REG_I2S_ACTL_PSR, AUDIO_READ(REG_I2S_ACTL_PSR));
	            AUDIO_WRITE(REG_I2S_ACTL_CON, val | P_DMA_IRQ);
	
	        } else {
	                dev_err(w55fa92_audio->dev, "Wrong DMA interrupt status!\n");                
	                spin_unlock_irqrestore(&w55fa92_audio->irqlock, flags);
	                return IRQ_HANDLED;
	        }

	        snd_pcm_period_elapsed(w55fa92_audio->substream[stream]);        
	        spin_unlock_irqrestore(&w55fa92_audio->irqlock, flags);
			LEAVE();
		
	        return IRQ_HANDLED;
	}
#endif

static int __devinit w55fa92_i2s_drvprobe(struct platform_device *pdev)
{
        struct w55fa92_audio *w55fa92_audio;
        int ret;

		ENTER();
		
        if (w55fa92_i2s_data)
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

        w55fa92_audio->i2s_clk = clk_get(NULL, "i2s");
        if (IS_ERR(w55fa92_audio->i2s_clk)) {
                ret = PTR_ERR(w55fa92_audio->i2s_clk);
                goto out2;
        }

        w55fa92_audio->eng_clk = clk_get(NULL, "ado");
        if (IS_ERR(w55fa92_audio->eng_clk)) {
                ret = PTR_ERR(w55fa92_audio->eng_clk);
                goto out2;
        }

//        w55fa92_audio->irq_num = platform_get_irq(pdev, 0);

        w55fa92_audio->irq_num = IRQ_I2S;        
        if (!w55fa92_audio->irq_num) {
                ret = -EBUSY;
                goto out2;
        }

#ifdef OPT_AUDIO_MODULE
		ret = request_irq(w55fa92_audio->irq_num, w55fa92_dma_interrupt, IRQF_DISABLED, "w55fa92-dma", w55fa92_audio);
	    if(ret)          
		    return ret;
#else
		ret = w55fa92_dma_create(w55fa92_audio);
		if (ret != 0)
			return ret;
#endif			
		
        w55fa92_i2s_data = w55fa92_audio;

        w55fa92_audio->dev = w55fa92_i2s_dai.dev =  &pdev->dev;

        ret = snd_soc_register_dai(&w55fa92_i2s_dai);
        if (ret)
                goto out3;

//        mfp_set_groupg(w55fa92_audio->dev); /* enbale i2s multifunction pin*/

		DBG("w55fa92_i2s_drvprobe OK \n");
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
        
        
		DBG("w55fa92_i2s_drvprobe FAIL \n");        

		LEAVE();        
        return ret;
}

static int __devexit w55fa92_i2s_drvremove(struct platform_device *pdev)
{
		ENTER();

#ifdef OPT_AUDIO_MODULE
		free_irq(IRQ_I2S, w55fa92_i2s_data);			
#else		
		w55fa92_dma_destroy(w55fa92_i2s_data);
#endif		
        snd_soc_unregister_dai(&w55fa92_i2s_dai);

//        clk_put(w55fa92_i2s_data->clk);
        iounmap(w55fa92_i2s_data->mmio);
        release_mem_region(w55fa92_i2s_data->res->start,
                           resource_size(w55fa92_i2s_data->res));

        w55fa92_i2s_data = NULL;

        return 0;
}

static struct platform_driver w55fa92_i2s_driver = {
        .driver	= {
                .name	= "w55fa92-audio-i2s",
                .owner	= THIS_MODULE,
        },
        .probe		= w55fa92_i2s_drvprobe,
        .remove		= __devexit_p(w55fa92_i2s_drvremove),
                     };

static int __init w55fa92_i2s_init(void)
{
		ENTER();
			
        return platform_driver_register(&w55fa92_i2s_driver);
}

static void __exit w55fa92_i2s_exit(void)
{
		ENTER();
			
        platform_driver_unregister(&w55fa92_i2s_driver);
}

module_init(w55fa92_i2s_init);
module_exit(w55fa92_i2s_exit);

MODULE_AUTHOR("Wan ZongShun <mcuos.com@gmail.com>");
MODULE_DESCRIPTION("W55FA92 IIS SoC driver!");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:w55fa92-i2s");

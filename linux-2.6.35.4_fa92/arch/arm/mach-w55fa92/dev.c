/*
 * linux/arch/arm/mach-w55fa92/dev.c
 *
 * Copyright (C) 2009 Nuvoton corporation.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation;version 2 of the License.
 *
 */

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/timer.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include <linux/mtd/physmap.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>

#include <linux/spi/spi.h>
#include <linux/spi/flash.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>
#include <asm/mach-types.h>

#include <mach/fb.h>
#include <mach/w55fa92_spi.h>
//#include <mach/w55fa92_keypad.h>
#include <mach/w55fa92_reg.h>

#include "cpu.h"
#include "dev.h"

/* W55FA92 SPI flash driver data */
#define W55FA92_FLASH_BASE	0xA0000000
#define W55FA92_FLASH_SIZE	0x400000
#define SPIOFFSET		0x200
#define SPIOREG_SIZE		0x100

/* Serial port registrations */
struct platform_device *w55fa92_uart_devs[2];

static struct mtd_partition w55fa92_flash_partitions[] = {
        {
                .name	=	"MTD reserved for user app",
                .size	=	0x100000,
                .offset	=	0x300000,
        }
};

static struct physmap_flash_data w55fa92_flash_data = {
        .width		=	2,
        .parts		=	w55fa92_flash_partitions,
        .nr_parts	=	ARRAY_SIZE(w55fa92_flash_partitions),
};

static struct resource w55fa92_flash_resources[] = {
        {
                .start	=	W55FA92_FLASH_BASE,
                .end	=	W55FA92_FLASH_BASE + W55FA92_FLASH_SIZE - 1,
                .flags	=	IORESOURCE_MEM,
        }
};

static struct platform_device w55fa92_flash_device = {
        .name		=	"physmap-flash",
        .id		=	0,
        .dev		= {
                .platform_data = &w55fa92_flash_data,
        },
        .resource	=	w55fa92_flash_resources,
        .num_resources	=	ARRAY_SIZE(w55fa92_flash_resources),
};


/* USB EHCI Host Controller */

static struct resource w55fa92_ehci_resource[] = {
        [0] = {
                .start = W55FA92_PA_UEHCI20,
                .end   = W55FA92_PA_UEHCI20 + W55FA92_SZ_UEHCI20 - 1,
                .flags = IORESOURCE_MEM,
        },
        [1] = {
                .start = IRQ_EHCI,
                .end   = IRQ_OHCI,
                .flags = IORESOURCE_IRQ,
        }
};

static u64 w55fa92_device_usb_ehci_dmamask = 0xffffffffUL;

static struct platform_device w55fa92_device_ehci = {
        .name		  = "w55fa92-ehci",
        .id		  = -1,
        .num_resources	  = ARRAY_SIZE(w55fa92_ehci_resource),
        .resource	  = w55fa92_ehci_resource,
        .dev              = {
                .dma_mask = &w55fa92_device_usb_ehci_dmamask,
                .coherent_dma_mask = 0xffffffffUL
        }
};

/* USB OHCI Host Controller */

static struct resource w55fa92_ohci_resource[] = {
        [0] = {
                .start = W55FA92_PA_UOHCI20,
                .end   = W55FA92_PA_UOHCI20 + W55FA92_SZ_UOHCI20 - 1,
                .flags = IORESOURCE_MEM,
        },
        [1] = {
                .start = IRQ_OHCI,
                .end   = IRQ_OHCI,
                .flags = IORESOURCE_IRQ,
        }
};

static u64 w55fa92_device_usb_ohci_dmamask = 0xffffffffUL;
static struct platform_device w55fa92_device_ohci = {
        .name		  = "w55fa92-ohci",
        .id		  = -1,
        .num_resources	  = ARRAY_SIZE(w55fa92_ohci_resource),
        .resource	  = w55fa92_ohci_resource,
        .dev              = {
                .dma_mask = &w55fa92_device_usb_ohci_dmamask,
                .coherent_dma_mask = 0xffffffffUL
        }
};
/* USB OHCI Host Like Controller */

static struct resource w55fa92_ohci_like_resource[] = {
        [0] = {
                .start = W55FA92_PA_USBH,
                .end   = W55FA92_PA_USBH + W55FA92_SZ_USBH - 1,
                .flags = IORESOURCE_MEM,
        },
        [1] = {
                .start = IRQ_USBH,
                .end   = IRQ_USBH,
                .flags = IORESOURCE_IRQ,
        }
};

static u64 w55fa92_device_usb_ohci_like_dmamask = 0xffffffffUL;
static struct platform_device w55fa92_device_ohci_like = {
        .name		  = "w55fa92-ohci-like",
        .id		  = -1,
        .num_resources	  = ARRAY_SIZE(w55fa92_ohci_like_resource),
        .resource	  = w55fa92_ohci_like_resource,
        .dev              = {
                .dma_mask = &w55fa92_device_usb_ohci_like_dmamask,
                .coherent_dma_mask = 0xffffffffUL
        }
};
/* USB Device (Gadget)*/

static struct resource w55fa92_usbgadget_resource[] = {
        [0] = {
                .start = W55FA92_PA_USBD,
                .end   = W55FA92_PA_USBD + W55FA92_SZ_USBD - 1,
                .flags = IORESOURCE_MEM,
        },
        [1] = {
                .start = IRQ_USBD,
                .end   = IRQ_USBD,
                .flags = IORESOURCE_IRQ,
        }
};

static u64 w55fa92_device_udc_dmamask = 0xffffffffUL;
static struct platform_device w55fa92_device_usbgadget = {
	.name		= "w55fa92-usbgadget",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(w55fa92_usbgadget_resource),
	.resource	= w55fa92_usbgadget_resource,
	.dev              = {
		.dma_mask = &w55fa92_device_udc_dmamask,
		.coherent_dma_mask = 0xffffffffUL
	}
};


/* MAC device */

static struct resource w55fa92_emc_resource[] = {
        [0] = {
                .start = W55FA92_PA_EMC,
                .end   = W55FA92_PA_EMC + W55FA92_SZ_EMC - 1,
                .flags = IORESOURCE_MEM,
        },
        [1] = {
                .start = IRQ_EMCTX,
                .end   = IRQ_EMCTX,
                .flags = IORESOURCE_IRQ,
        },
        [2] = {
                .start = IRQ_EMCRX,
                .end   = IRQ_EMCRX,
                .flags = IORESOURCE_IRQ,
        }
};


static u64 w55fa92_device_emc_dmamask = 0xffffffffUL;
static struct platform_device w55fa92_device_emc = {
        .name           = "w55fa92-emc",
        .id             = -1,
        .num_resources  = ARRAY_SIZE(w55fa92_emc_resource),
        .resource       = w55fa92_emc_resource,
        .dev              = {
                .dma_mask = &w55fa92_device_emc_dmamask,
                .coherent_dma_mask = 0xffffffffUL
        }
};


/* spi device, spi flash info */

#ifdef CONFIG_MTD_PARTITIONS
#ifdef CONFIG_MTD_M25P80
static struct mtd_partition w55fa92_spi_flash_partitions[] = {
        [0] = {
                .name = "UserData",
                .size = CONFIG_SPIFLASH_PARTITION_SIZE,
                .offset = CONFIG_SPIFLASH_PARTITION_OFFSET,
        },
};
#endif
#endif

#if defined (CONFIG_FA92_SPI0_ENABLE) || defined (CONFIG_FA92_SPI1_ENABLE)
static struct flash_platform_data w55fa92_spi_flash_data = {        
#ifdef CONFIG_MTD_PARTITIONS
#ifdef CONFIG_MTD_M25P80	
        .name = "m25p80",
        .parts =  w55fa92_spi_flash_partitions,
        .nr_parts = ARRAY_SIZE(w55fa92_spi_flash_partitions),    
        .type = "w25q32",
#endif
#endif                
};
#endif

/* SPI device */

#ifdef CONFIG_FA92_SPI0_ENABLE
static struct w55fa92_spi_info w55fa92_spi0flash_data = {
        .num_cs		= 2,
        .lsb		= 0,
        .txneg		= 1,
        .rxneg		= 0,
#ifdef CONFIG_OPT_FPGA     
        .divider	= 5,
#else
	.divider	= 0,
#endif
        .sleep		= 0,
        .txnum		= 0,
        .txbitlen	= 8,
        .byte_endin = 0,
        .bus_num	= 0,
};

static struct resource w55fa92_spi0_resource[] = {
        [0] = {
                .start = W55FA92_PA_SPI0,
                .end   = W55FA92_PA_SPI0 + 0x400 - 1,
                .flags = IORESOURCE_MEM,
        },
        [1] = {
                .start = IRQ_SPI0,
                .end   = IRQ_SPI0,
                .flags = IORESOURCE_IRQ,
        }
};

static struct platform_device w55fa92_device_spi0 = {
        .name		= "w55fa92-spi",
        .id		= 0,
        .num_resources	= ARRAY_SIZE(w55fa92_spi0_resource),
        .resource	= w55fa92_spi0_resource,
        .dev		= {
                .platform_data = &w55fa92_spi0flash_data,
        }
};

static struct spi_board_info w55fa92_spi0_board_info[] __initdata = {
#ifdef CONFIG_FA92_SPI0_CS0_ENABLE
        {
#ifdef CONFIG_FA92_SPI0_CS0_MTD			
                .modalias = "m25p80",
#endif
#ifdef CONFIG_FA92_SPI0_CS0_SPIDEV
		.modalias = "spidev",
#endif
                .max_speed_hz = 50000000,
                .bus_num = 0,
                .chip_select = 0,                
                .platform_data = &w55fa92_spi_flash_data,
                .mode = SPI_MODE_0,
        },
#endif     

#ifdef CONFIG_FA92_SPI0_CS1_ENABLE
	{
#ifdef CONFIG_FA92_SPI0_CS1_MTD			
                .modalias = "m25p80",
#endif
#ifdef CONFIG_FA92_SPI0_CS1_SPIDEV
		.modalias = "spidev",
#endif
                .max_speed_hz = 50000000,
                .bus_num = 0,                
                .chip_select = 1,
                .platform_data = &w55fa92_spi_flash_data,
                .mode = SPI_MODE_0,
        },
#endif        
};
#endif

#ifdef CONFIG_FA92_SPI1_ENABLE
static struct w55fa92_spi_info w55fa92_spi1flash_data = {
        .num_cs		= 2,
        .lsb		= 0,
        .txneg		= 1,
        .rxneg		= 0,
#ifdef CONFIG_OPT_FPGA     
        .divider	= 5,
#else
	.divider	= 0,
#endif
        .sleep		= 0,
        .txnum		= 0,
        .txbitlen	= 8,
        .byte_endin = 0,
        .bus_num	= 1,
};

static struct resource w55fa92_spi1_resource[] = {
        [0] = {
                .start = W55FA92_PA_SPI0 + 0x400,
                .end   = W55FA92_PA_SPI0 + 0x800 - 1,
                .flags = IORESOURCE_MEM,
        },
        [1] = {
                .start = IRQ_SPI1,
                .end   = IRQ_SPI1,
                .flags = IORESOURCE_IRQ,
        }
};

static struct platform_device w55fa92_device_spi1 = {
        .name		= "w55fa92-spi",
        .id		= 1,
        .num_resources	= ARRAY_SIZE(w55fa92_spi1_resource),
        .resource	= w55fa92_spi1_resource,
        .dev		= {
                .platform_data = &w55fa92_spi1flash_data,
        }
};

static struct spi_board_info w55fa92_spi1_board_info[] __initdata = {
#ifdef CONFIG_FA92_SPI1_CS0_ENABLE
	{ 
#ifdef CONFIG_FA92_SPI1_CS0_MTD			
                .modalias = "m25p80",
#endif
#ifdef CONFIG_FA92_SPI1_CS0_SPIDEV
		.modalias = "spidev",
#endif
                .max_speed_hz = 50000000,
                .bus_num = 1,                
                .chip_select = 0,
                .platform_data = &w55fa92_spi_flash_data,
                .mode = SPI_MODE_0,
        },
#endif       

#ifdef CONFIG_FA92_SPI1_CS1_ENABLE
	{ 
#ifdef CONFIG_FA92_SPI1_CS1_MTD			
                .modalias = "m25p80",
#endif
#ifdef CONFIG_FA92_SPI1_CS1_SPIDEV
		.modalias = "spidev",
#endif
                .max_speed_hz = 50000000,
                .bus_num = 1,                
                .chip_select = 1,
                .platform_data = &w55fa92_spi_flash_data,
                .mode = SPI_MODE_0,
        },
#endif        
};
#endif


/* WDT Device */

static struct resource w55fa92_wdt_resource[] = {
        [0] = {
                .start = W55FA92_PA_TIMER,
                .end   = W55FA92_PA_TIMER + W55FA92_SZ_TIMER - 1,
                .flags = IORESOURCE_MEM,
        },
        [1] = {
                .start = IRQ_WDT,
                .end   = IRQ_WDT,
                .flags = IORESOURCE_IRQ,
        }
};

static struct platform_device w55fa92_device_wdt = {
        .name		= "w55fa92-wdt",
        .id		= -1,
        .num_resources	= ARRAY_SIZE(w55fa92_wdt_resource),
        .resource	= w55fa92_wdt_resource,
};

/*
 * public device definition between 910 and 920, or 910
 * and 950 or 950 and 960...,their dev platform register
 * should be in specific file such as nuc950, nuc960 c
 * files rather than the public dev.c file here. so the
 * corresponding platform_device definition should not be
 * static.
*/

/* RTC controller*/

static struct resource w55fa92_rtc_resource[] = {
        [0] = {
                .start = W55FA92_PA_RTC,
                .end   = W55FA92_PA_RTC + 0xff,
                .flags = IORESOURCE_MEM,
        },
        [1] = {
                .start = IRQ_RTC,
                .end   = IRQ_RTC,
                .flags = IORESOURCE_IRQ,
        },
};

struct platform_device w55fa92_device_rtc = {
        .name		= "w55fa92-rtc",
        .id		= -1,
        .num_resources	= ARRAY_SIZE(w55fa92_rtc_resource),
        .resource	= w55fa92_rtc_resource,
};

/*TouchScreen controller*/

static struct resource w55fa92_ts_resource[] = {
        [0] = {
                .start = W55FA92_PA_ADC,
                .end   = W55FA92_PA_ADC + W55FA92_SZ_ADC-1,
                .flags = IORESOURCE_MEM,
        },
        [1] = {
                .start = IRQ_ADC,
                .end   = IRQ_ADC,
                .flags = IORESOURCE_IRQ,
        },
};

struct platform_device w55fa92_device_ts = {
        .name		= "w55fa92-ts",
        .id		= -1,
        .resource	= w55fa92_ts_resource,
        .num_resources	= ARRAY_SIZE(w55fa92_ts_resource),
};

/* FMI Device */

static struct resource w55fa92_fmi_resource[] = {
        [0] = {
                .start = W55FA92_PA_SIC,
                .end   = W55FA92_PA_SIC + W55FA92_SZ_SIC - 1,
                .flags = IORESOURCE_MEM,
        },
        [1] = {
                .start = IRQ_SIC,
                .end   = IRQ_SIC,
                .flags = IORESOURCE_IRQ,
        }
};

static u64 w55fa92_device_fmi_dmamask = 0xffffffffUL;
struct platform_device w55fa92_device_fmi = {
	.name		= "w55fa92-fmi",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(w55fa92_fmi_resource),
	.resource	= w55fa92_fmi_resource,
	.dev              = {
		.dma_mask = &w55fa92_device_fmi_dmamask,
		.coherent_dma_mask = 0xffffffffUL
	}
};


/* SDIO Device */
static struct resource w55fa92_sdio_resource[] = {
    [0] = {
        .start = W55FA92_PA_SDIO,
        .end   = W55FA92_PA_SDIO + W55FA92_SZ_SDIO - 1,
        .flags = IORESOURCE_MEM,
    },
    [1] = {
        .start = IRQ_SDIO,
        .end   = IRQ_SDIO,
        .flags = IORESOURCE_IRQ,
    }
};

static u64 w55fa92_device_sdio_dmamask = 0xffffffffUL;
struct platform_device w55fa92_device_sdio = {
    .name   = "w55fa92-sdio",
    .id     = -1,
    .num_resources  = ARRAY_SIZE(w55fa92_sdio_resource),
    .resource   = w55fa92_sdio_resource,
    .dev        = {
        .dma_mask = &w55fa92_device_sdio_dmamask,
        .coherent_dma_mask = 0xffffffffUL
    }
};


/* KPI controller */

static struct resource w55fa92_kpi_resource[] = {
        [0] = {
                .start = W55FA92_PA_KPI,
                .end   = W55FA92_PA_KPI + W55FA92_SZ_KPI - 1,
                .flags = IORESOURCE_MEM,
        },
        [1] = {
                .start = IRQ_KPI,
                .end   = IRQ_KPI,
                .flags = IORESOURCE_IRQ,
        }

};

struct platform_device w55fa92_device_kpi = {
        .name		= "w55fa92-kpi",
        .id		= -1,
        .num_resources	= ARRAY_SIZE(w55fa92_kpi_resource),
        .resource	= w55fa92_kpi_resource,
};

#if 0
static struct w55fa92fb_display  w55fa92_lcd_info[] = {
	/* Giantplus Technology GWMTF9360A 320x240 Color TFT LCD */
	[0] = {
		.type		= LCM_DCCS_VA_SRC_RGB565,
		.width		= 320,
		.height		= 240,
		.xres		= 320,
		.yres		= 240,
		.bpp		= 16,
		.pixclock	= 200000,
		.left_margin	= 34,
		.right_margin   = 54,
		.hsync_len	= 10,
		.upper_margin	= 18,
		.lower_margin	= 4,
		.vsync_len	= 1,
		.dccs		= 0x0e00041a,
		.devctl		= 0x060800c0,
		.fbctrl		= 0x00a000a0,
		.scale		= 0x04000400,
	},
};

static struct w55fa92fb_mach_info w55fa92_fb_info = {
#if defined(CONFIG_GOWORLD_GWMTF9360A_320x240)
	.displays		= &w55fa92_lcd_info[0],
#else
	.displays		= w55fa92_lcd_info,
#endif
	.num_displays		= ARRAY_SIZE(w55fa92_lcd_info),
	.default_display	= 0,
	.gpio_dir		= 0x00000004,
	.gpio_dir_mask		= 0xFFFFFFFD,
	.gpio_data		= 0x00000004,
	.gpio_data_mask		= 0xFFFFFFFD,
};

#endif

/* VPOST controller*/
static struct resource w55fa92_lcd_resource[] = {
	[0] = {
		.start = W55FA92_PA_VPOST,
		.end   = W55FA92_PA_VPOST + W55FA92_SZ_VPOST - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_VPOST,
		.end   = IRQ_VPOST,
		.flags = IORESOURCE_IRQ,
	}
};

static u64 w55fa92_device_lcd_dmamask = -1;

struct platform_device w55fa92_device_lcd = {
	.name             = "w55fa92-lcd",
	.id               = -1,
	.num_resources    = ARRAY_SIZE(w55fa92_lcd_resource),
	.resource         = w55fa92_lcd_resource,
	.dev              = {
		.dma_mask               = &w55fa92_device_lcd_dmamask,
		.coherent_dma_mask      = -1,
//		.platform_data = &w55fa92_fb_info,
	}
};


//W55FA92_RECS(VPOST);
//W55FA92_DEVICE(lcddevice,VPOST,0,"w55fa92-lcd");

EXPORT_SYMBOL(w55fa92_device_lcd);

void w55fa92_fb_set_platdata(struct w55fa92fb_mach_info *pd)
{
        struct w55fa92fb_mach_info *npd;

        npd = kmalloc(sizeof(*npd), GFP_KERNEL);
        if (npd) {
                memcpy(npd, pd, sizeof(*npd));
                w55fa92_device_lcd.dev.platform_data = npd;
        } else {
                printk(KERN_ERR "no memory for W55FA92 LCD platform data\n");
        }
}
EXPORT_SYMBOL(w55fa92_fb_set_platdata);



/* AUDIO controller */

static u64 w55fa92_device_audio_dmamask = -1;
#if 0
static struct resource w55fa92_ac97_resource[] = {
        [0] = {
                .start = W55FA92_PA_SPU,
                .end   = W55FA92_PA_SPU + W55FA92_SZ_SPU - 1,
                .flags = IORESOURCE_MEM,
        },
        [1] = {
                .start = IRQ_SPU,
                .end   = IRQ_SPU,
                .flags = IORESOURCE_IRQ,
        }
};

struct platform_device w55fa92_device_audio_ac97 = {
        .name		= "w55fa92-audio-ac97",
        .id		= -1,
        .num_resources	= ARRAY_SIZE(w55fa92_ac97_resource),
        .resource	= w55fa92_ac97_resource,
        .dev              = {
                .dma_mask               = &w55fa92_device_audio_dmamask,
                .coherent_dma_mask      = -1,
        }
};
#endif

static struct resource w55fa92_spu_resource[] = {
        [0] = {
                .start = W55FA92_PA_SPU,
                .end   = W55FA92_PA_SPU + W55FA92_SZ_SPU - 1,
                .flags = IORESOURCE_MEM,
        },
        [1] = {
                .start = IRQ_SPU,
                .end   = IRQ_SPU,
                .flags = IORESOURCE_IRQ,
        }
};

struct platform_device w55fa92_device_audio_spu = {
        .name		= "w55fa92-audio-spu",
        .id		= -1,
        .num_resources	= ARRAY_SIZE(w55fa92_spu_resource),
        .resource	= w55fa92_spu_resource,
        .dev              = {
                .dma_mask               = &w55fa92_device_audio_dmamask,
                .coherent_dma_mask      = -1,
        }
};

static struct resource w55fa92_i2s_resource[] = {
        [0] = {
                .start = W55FA92_PA_I2SM,
                .end   = W55FA92_PA_I2SM + W55FA92_SZ_I2SM - 1,
                .flags = IORESOURCE_MEM,
        },
        [1] = {
                .start = IRQ_I2S,
                .end   = IRQ_I2S,
                .flags = IORESOURCE_IRQ,
        }
};

struct platform_device w55fa92_device_audio_i2s = {
        .name		= "w55fa92-audio-i2s",
        .id		= -1,
        .num_resources	= ARRAY_SIZE(w55fa92_i2s_resource),
        .resource	= w55fa92_i2s_resource,
        .dev              = {
                .dma_mask               = &w55fa92_device_audio_dmamask,
                .coherent_dma_mask      = -1,
        }
};



static struct resource w55fa92_adc_resource[] = {
        [0] = {
                .start = W55FA92_PA_ADC,
                .end   = W55FA92_PA_ADC + W55FA92_SZ_ADC - 1,
                .flags = IORESOURCE_MEM,
        },
        [1] = {
                .start = IRQ_EDMA,
                .end   = IRQ_EDMA,
                .flags = IORESOURCE_IRQ,
        }
};

struct platform_device w55fa92_device_audio_adc = {
        .name		= "w55fa92-audio-adc",
        .id		= -1,
        .num_resources	= ARRAY_SIZE(w55fa92_adc_resource),
        .resource	= w55fa92_adc_resource,
        .dev              = {
                .dma_mask               = &w55fa92_device_audio_dmamask,
                .coherent_dma_mask      = -1,
        }
};

/* I2C */

static struct resource w55fa92_i2c_resource[] = {
        [0] = {
                .start = W55FA92_PA_I2C,
                .end   = W55FA92_PA_I2C + W55FA92_SZ_I2C - 1,
                .flags = IORESOURCE_MEM,
        },
        [1] = {
                .start = IRQ_I2C,
                .end   = IRQ_I2C,
                .flags = IORESOURCE_IRQ,
        }

};

struct platform_device w55fa92_device_i2c = {
        .name		  = "w55fa92-i2c",
        .id		  = -1,
        .num_resources	  = ARRAY_SIZE(w55fa92_i2c_resource),
        .resource	  = w55fa92_i2c_resource,
};

EXPORT_SYMBOL(w55fa92_device_i2c);

static struct platform_device w55fa92_device_gpioi2c = {
	.name		= "w55fa92-gpioi2c",
	.id		= 0,
	.dev		= {
		.platform_data = NULL,
	},
	.num_resources	= 0
};

/* PWM */

#if defined (CONFIG_W55FA92_PWM0_PD0) || defined (CONFIG_W55FA92_PWM0_PD12)
static struct resource w55fa92_pwm0_resource[] = {
        [0] = {
                .start = W55FA92_PA_PWM + 0x0C,
                .end   = W55FA92_PA_PWM + 0x14 - 1,
                .flags = IORESOURCE_MEM,
        },
        [1] = {
                .start = IRQ_PWM,
                .end   = IRQ_PWM,
                .flags = IORESOURCE_IRQ,
        }

};

struct platform_device w55fa92_device_pwm0 = {
        .name		  = "w55fa92-pwm",
        .id		  = 0,
        .num_resources	  = ARRAY_SIZE(w55fa92_pwm0_resource),
        .resource	  = w55fa92_pwm0_resource,
};

EXPORT_SYMBOL(w55fa92_device_pwm0);
#endif

#if defined (CONFIG_W55FA92_PWM1_PD1) || defined (CONFIG_W55FA92_PWM1_PD13)
static struct resource w55fa92_pwm1_resource[] = {
        [0] = {
                .start = W55FA92_PA_PWM + 0x18,
                .end   = W55FA92_PA_PWM + 0x20 - 1,
                .flags = IORESOURCE_MEM,
        },
        [1] = {
                .start = IRQ_PWM,
                .end   = IRQ_PWM,
                .flags = IORESOURCE_IRQ,
        }

};

struct platform_device w55fa92_device_pwm1 = {
        .name		  = "w55fa92-pwm",
        .id		  = 1,
        .num_resources	  = ARRAY_SIZE(w55fa92_pwm1_resource),
        .resource	  = w55fa92_pwm1_resource,
};

EXPORT_SYMBOL(w55fa92_device_pwm1);
#endif

#ifdef CONFIG_W55FA92_PWM2_PD2
static struct resource w55fa92_pwm2_resource[] = {
        [0] = {
                .start = W55FA92_PA_PWM + 0x24,
                .end   = W55FA92_PA_PWM + 0x2C - 1,
                .flags = IORESOURCE_MEM,
        },
        [1] = {
                .start = IRQ_PWM,
                .end   = IRQ_PWM,
                .flags = IORESOURCE_IRQ,
        }

};

struct platform_device w55fa92_device_pwm2 = {
        .name		  = "w55fa92-pwm",
        .id		  = 2,
        .num_resources	  = ARRAY_SIZE(w55fa92_pwm2_resource),
        .resource	  = w55fa92_pwm2_resource,
};

EXPORT_SYMBOL(w55fa92_device_pwm2);
#endif

#ifdef CONFIG_W55FA92_PWM3_PD3
static struct resource w55fa92_pwm3_resource[] = {
        [0] = {
                .start = W55FA92_PA_PWM + 0x30,
                .end   = W55FA92_PA_PWM + 0x38 - 1,
                .flags = IORESOURCE_MEM,
        },
        [1] = {
                .start = IRQ_PWM,
                .end   = IRQ_PWM,
                .flags = IORESOURCE_IRQ,
        }

};

struct platform_device w55fa92_device_pwm3 = {
        .name		  = "w55fa92-pwm",
        .id		  = 3,
        .num_resources	  = ARRAY_SIZE(w55fa92_pwm3_resource),
        .resource	  = w55fa92_pwm3_resource,
};

EXPORT_SYMBOL(w55fa92_device_pwm3);
#endif

static struct platform_device *w55fa92_public_dev[] __initdata = {
//	&w55fa92_serial_device,
	&w55fa92_flash_device,
	&w55fa92_device_ohci_like,
	&w55fa92_device_ohci,
	&w55fa92_device_ehci,
	&w55fa92_device_usbgadget,
	&w55fa92_device_emc,
#ifdef CONFIG_FA92_SPI0_ENABLE	
	&w55fa92_device_spi0,
#endif
#ifdef CONFIG_FA92_SPI1_ENABLE	
	&w55fa92_device_spi1,
#endif	
	&w55fa92_device_wdt,
	&w55fa92_device_rtc,
	&w55fa92_device_ts,
	&w55fa92_device_fmi,
	&w55fa92_device_sdio,
	&w55fa92_device_kpi,
	&w55fa92_device_lcd,
//	&w55fa92_device_audio_ac97,
	&w55fa92_device_audio_i2s,
	&w55fa92_device_audio_adc,
	&w55fa92_device_audio_spu,
	&w55fa92_device_i2c,
	&w55fa92_device_gpioi2c,
#if defined (CONFIG_W55FA92_PWM0_PD0) || defined (CONFIG_W55FA92_PWM0_PD12)	
	&w55fa92_device_pwm0,
#endif	
#if defined (CONFIG_W55FA92_PWM1_PD1) || defined (CONFIG_W55FA92_PWM1_PD13)
	&w55fa92_device_pwm1,
#endif
#ifdef CONFIG_W55FA92_PWM2_PD2
	&w55fa92_device_pwm2,
#endif	
#ifdef CONFIG_W55FA92_PWM3_PD3
	&w55fa92_device_pwm3,
#endif	
};

/* Provide adding specific CPU platform devices API */

void __init w55fa92_dev_init()
{
        platform_add_devices(w55fa92_public_dev, ARRAY_SIZE(w55fa92_public_dev));
#ifdef CONFIG_FA92_SPI0_ENABLE
        spi_register_board_info(w55fa92_spi0_board_info, ARRAY_SIZE(w55fa92_spi0_board_info));
#endif
#ifdef CONFIG_FA92_SPI1_ENABLE
		spi_register_board_info(w55fa92_spi1_board_info, ARRAY_SIZE(w55fa92_spi1_board_info));
#endif
}

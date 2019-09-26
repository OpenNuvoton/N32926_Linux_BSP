/*
 * linux/arch/arm/mach-w55fa92/cpu.c
 *
 * Copyright (c) 2014 Nuvoton corporation.
 *
 * W55FA92 CPU Support
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation;version 2 of the License.
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/rtc.h>

#include <asm/irq.h>
#include <asm/io.h>
#include <asm/delay.h>
#include <asm/tlbflush.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <mach/w55fa92_reg.h>
#include <mach/w55fa92_sysmgr.h>
#include <mach/w55fa92_rtc.h>
#include "cpu.h"
#include "dev.h"

#define CPU_DEBUG	0

#if defined(CONFIG_W55FA92_SYSMGR) || defined(CONFIG_W55FA92_SYSMGR_MODULE)
extern void sysmgr_report(unsigned);
#endif
extern unsigned int w55fa92_get_pllcon(char, unsigned int);
extern int w55fa92_edma_isbusy(int);
#if defined(CONFIG_RTC_DRV_W55FA92)
#define USE_REL_ALARM
extern int w55fa92_rtc_read_time_wrap(struct rtc_time *);
extern int w55fa92_rtc_set_time_wrap(struct rtc_time *);
extern int w55fa92_rtc_read_alarm_wrap(struct rtc_wkalrm *);
extern int w55fa92_rtc_set_alarm_wrap(struct rtc_wkalrm *);
extern int w55fa92_rtc_set_rel_alarm_wrap(int second);
extern void rtc_wait_ready(void);
#endif
#if defined(CONFIG_SND_SOC_W55FA92_SPU)
extern void spuDacOn(void);
extern void spuDacOff(void);
#endif
#if defined(CONFIG_W55FA92_ETH)
extern void w55fa92_ether_power_down(void);
extern void w55fa92_ether_wakeup(void);
#endif


/* Initial serial platform data */
/*
struct plat_serial8250_port w55fa92_uart_data[] = {
	[0] = W55FA92_8250PORT(UART0),
	// MFSEL and CLKEN must set accordingly
	//[1] = W55FA92_8250PORT(UART1),
	{},
};

struct platform_device w55fa92_serial_device = {
	.name			= "serial8250",
	.id			= PLAT8250_DEV_PLATFORM,
	.dev			= {
		.platform_data	= w55fa92_uart_data,
	},
};
*/
/* Init W55FA92 io */

void __init cpu_map_io(struct map_desc *mach_desc, int mach_size)
{
	unsigned long idcode = 0x0;

	iotable_init(mach_desc, mach_size);

	idcode = __raw_readl(REG_CHIPID);
	if (idcode == W55FA92_CPUID)
		printk(KERN_INFO "CPU type 0x%08lx is W55FA92\n", idcode);
}


static struct platform_device *sys_clk;
extern unsigned int w55fa92_upll_clock;
extern unsigned int w55fa92_mpll_clock;
extern unsigned int w55fa92_system_clock;
extern unsigned int w55fa92_cpu_clock;
extern unsigned int w55fa92_ahb_clock;
extern unsigned int w55fa92_apb_clock;
#if defined(CONFIG_W55FA92_KEYPAD)
extern u32 w55fa92_key_pressing;
#endif
#if defined(CONFIG_TOUCHSCREEN_W55FA92)
extern u32 w55fa92_ts_pressing;
#endif
static u32 *sram_vaddr;

void enter_clock_setting(u32, u8, u8, u8, u8) __attribute__ ((section ("enter_cs")));
void enter_clock_setting(u32 pllcon, u8 sys_div_N0, u8 sys_div_N1, u8 cpu_div, u8 apb_div)
{
	unsigned int volatile i;

	// push register pages to TLB cache entry
	__raw_readl(REG_CLKDIV0);
	__raw_readl(REG_SDEMR);

	if (pllcon) {
		__raw_writel(pllcon, REG_UPLLCON);
		for (i = 0; i < 1000; i++) ;
	}
	__raw_writel((__raw_readl(REG_CLKDIV0)&~(SYSTEM_N0|SYSTEM_N1))|(sys_div_N0)|(sys_div_N1<<8), REG_CLKDIV0);
	__raw_writel((__raw_readl(REG_CLKDIV4)&~(APB_N|CPU_N))|(((apb_div+8)<<8)|cpu_div), REG_CLKDIV4);
	for (i = 0; i < 1000; i++) ;
}

int set_system_clocks(u32 tmp_upll_clock, u32 sys_clock, u32 cpu_clock, u32 apb_clock)
{
	void (*cs_func)(u32, u8, u8, u8, u8);
	unsigned long volatile flags;
	u32 pllcon, int_mask, int_maskh, ahbclk;
	u32 tmp_system_clock, tmp_cpu_clock, tmp_hclk1_clock, tmp_hclk234_clock, tmp_apb_clock, tmp_ddr_clock;
	u8 sys_div, sys_div_N0, sys_div_N1, cpu_div, hclk234_div, apb_div, ddr_div;

	pllcon = 0;
	if (tmp_upll_clock != w55fa92_upll_clock) {
		pllcon = w55fa92_get_pllcon(1, tmp_upll_clock);
		if (pllcon == 0) {
			printk("Cannot calculate UPLL %d KHz!!\n", tmp_upll_clock);
			return -1;
		}
	}

	sys_div = tmp_upll_clock / sys_clock + (tmp_upll_clock%sys_clock ? 1:0);
	for (sys_div_N1 = 1; sys_div_N1 <= 16; sys_div_N1++) {
		for (sys_div_N0 = 1; sys_div_N0 <= 8; sys_div_N0++) {
			if (sys_div == sys_div_N1*sys_div_N0)
				break;
		}
		if (sys_div_N0 <= 8)
			break;
	}
	if (sys_div_N1 > 16) {
		printk("Cannot set system clock divider %d !!\n", sys_div);
		return -1;
	}
	sys_div_N0 = sys_div_N0 - 1;
	sys_div_N1 = sys_div_N1 - 1;

	tmp_system_clock = tmp_upll_clock / sys_div;
	cpu_div = tmp_system_clock / cpu_clock - 1 + (tmp_system_clock%cpu_clock ? 1:0);
	tmp_cpu_clock = tmp_system_clock / (cpu_div + 1);
	tmp_hclk1_clock = (cpu_div == 0) ? tmp_cpu_clock/2:tmp_cpu_clock;
	hclk234_div = ((__raw_readl(REG_CLKDIV4) & HCLK234_N) >> 4) + 1;
	tmp_hclk234_clock = (tmp_system_clock / 2) / hclk234_div;
	apb_div = tmp_hclk1_clock / apb_clock - 1 + (tmp_hclk1_clock%apb_clock ? 1:0);
	tmp_apb_clock = tmp_hclk1_clock / (apb_div + 1);
	ddr_div = ((__raw_readl(REG_CLKDIV7) & DRAM_N0) + 1) * (((__raw_readl(REG_CLKDIV7) & DRAM_N1) >> 5) + 1);
	tmp_ddr_clock = w55fa92_mpll_clock / ddr_div;
#if CPU_DEBUG
	printk("sys_div_N0=%d, sys_div_N1=%d, cpu_div=%d, apb_div=%d, ddr_div=%d\n", 
		sys_div_N0, sys_div_N1, cpu_div, apb_div, ddr_div);
	printk("tmp_hclk1_clock = %d\n", tmp_hclk1_clock);
	printk("tmp_hclk234_clock = %d\n", tmp_hclk234_clock);
	printk("tmp_ddr_clock = %d\n", tmp_ddr_clock);
#endif

	if ((tmp_ddr_clock/2) <= tmp_hclk1_clock) {
		printk("MCLK must large than HCLK1 !!\n");
		return -1;
	}
	if ((tmp_ddr_clock/2) <= tmp_hclk234_clock) {
		printk("MCLK must large than HCLK234 !!\n");
		return -1;
	}

	w55fa92_upll_clock = tmp_upll_clock;
	w55fa92_system_clock = tmp_system_clock;
	w55fa92_cpu_clock = tmp_cpu_clock;
	w55fa92_ahb_clock = tmp_hclk1_clock;
	w55fa92_apb_clock = tmp_apb_clock;


#if CPU_DEBUG
	printk("PLL clock = %d\n", w55fa92_upll_clock);
	printk("SYS clock = %d\n", w55fa92_system_clock);
	printk("CPU clock = %d\n", w55fa92_cpu_clock);
	printk("AHB clock = %d\n", w55fa92_ahb_clock);
	printk("APB clock = %d\n", w55fa92_apb_clock);
	printk("REG_CHIPCFG = 0x%x\n", __raw_readl(REG_CHIPCFG));
	printk("REG_UPLLCON = 0x%x\n", __raw_readl(REG_UPLLCON));
	printk("REG_CLKDIV0 = 0x%x\n", __raw_readl(REG_CLKDIV0));
	printk("REG_CLKDIV4 = 0x%x\n", __raw_readl(REG_CLKDIV4));
#endif

	local_irq_save(flags);
	int_mask = __raw_readl(REG_AIC_IMR);
	int_maskh = __raw_readl(REG_AIC_IMRH);
	// disable all interrupts
	__raw_writel(0xFFFFFFFF, REG_AIC_MDCR);
	__raw_writel(0xFFFFFFFF, REG_AIC_MDCRH);

	// save AHB clock registers
	ahbclk = __raw_readl(REG_AHBCLK);
	// open SRAM engine clock
	__raw_writel(ahbclk | SRAM_CKE, REG_AHBCLK);

	// put enter_clock_setting into SRAM
	memcpy(sram_vaddr, enter_clock_setting, 512);
	cs_func = (void(*)(u32, u8, u8, u8, u8)) (sram_vaddr);

	// flush all TLB cache entries
	local_flush_tlb_all();
	// change the system clocks
	cs_func(pllcon, sys_div_N0, sys_div_N1, cpu_div, apb_div);

	// restore AHB registers
	__raw_writel(ahbclk, REG_AHBCLK);

	// restore interrupt mask
	__raw_writel(int_mask, REG_AIC_MECR);
	__raw_writel(int_maskh, REG_AIC_MECRH);
	local_irq_restore(flags);

	return 0;
}

void enter_power_saving(u8) __attribute__ ((section ("enter_ps")));
// power saving mode be 0:standby, 1:power down
void enter_power_saving(u8 stop_xtal)
{
	unsigned int volatile i;

	// push register pages to TLB cache entry
	__raw_readl(REG_CLKDIV0);

	__raw_writel(__raw_readl(REG_SDOPM) & ~AUTOPDN, REG_SDOPM);			            
	// disable DLL of FA92
	__raw_writel((__raw_readl(SDRAM_BA+0x58) & ~BIT3) | BIT4, SDRAM_BA+0x54);
	for (i = 0; i < 0x100; i++) ;

	// enable SDRAM self refresh mode
	__raw_writel((__raw_readl(REG_SDCMD)|SELF_REF) & ~AUTOEXSELFREF, REG_SDCMD);
	for (i = 0; i < 0x100; i++) ;
	// disable DLL of FA92
//	__raw_writel((__raw_readl(SDRAM_BA+0x58) & ~BIT3) | BIT4, SDRAM_BA+0x54);
//	for (i = 0; i < 0x100; i++) ;

	if (stop_xtal == 1) {
		// change system clock source to external crystal
		__raw_writel(__raw_readl(REG_CLKDIV0) & ~SYSTEM_S, REG_CLKDIV0);
		for (i = 0; i < 0x100; i++) ;
		// stop APLL clock
		__raw_writel(__raw_readl(REG_APLLCON) | PD, REG_APLLCON);
		// stop UPLL clock
		__raw_writel(__raw_readl(REG_UPLLCON) | PD, REG_UPLLCON);
		// stop MPLL clock
		//__raw_writel(__raw_readl(REG_MPLLCON) | PD, REG_MPLLCON);
		for (i = 0; i < 0x300; i++) ;

#if 0
		// stop both of external and CPU clock
		__asm__ __volatile__ \
		( \
			"mov	r2, %0			@ read clock register \n\
			ldmia	r2, {r0, r1}		@ load registers to r0 and r1 \n\
			bic	r0, r0, #0x01		@ \n\
			bic	r1, r1, #0x01		@ \n\
			stmia	r2, {r0, r1}		@ " \
				: /* no output registers */ \
				: "r" (REG_PWRCON) \
				: "r0", "r1", "r2" \
		);
#else
		// stop only external crystal
		__raw_writel((__raw_readl(REG_PWRCON) & ~(PRE_SCALAR|XTAL_EN)) | (0xFF << 8), REG_PWRCON);
#endif
	}
	else {
		// stop APLL clock
		__raw_writel(__raw_readl(REG_APLLCON) | PD, REG_APLLCON);
		for (i = 0; i < 0x300; i++) ;
		// stop CPU clock
		__raw_writel(__raw_readl(REG_AHBCLK) & ~CPU_CKE, REG_AHBCLK);
	}
	for (i = 0; i < 0x300; i++) ;

	if (stop_xtal == 1) {
		// enable APLL clock
		__raw_writel(__raw_readl(REG_APLLCON) & ~PD, REG_APLLCON);
		// enable UPLL clock
		__raw_writel(__raw_readl(REG_UPLLCON) & ~PD, REG_UPLLCON);
		// enable MPLL clock
		//__raw_writel(__raw_readl(REG_MPLLCON) & ~PD, REG_MPLLCON);
		//for (i = 0; i < 0x3000; i++) ;
		// Wait PLL lock bit.
		while ((__raw_readl(REG_POR_LVRD) & (UPLL_LKDT|APLL_LKDT)) != (UPLL_LKDT|APLL_LKDT)) ;

		// restore system clock source to UPLL
		__raw_writel(__raw_readl(REG_CLKDIV0) | SYSTEM_S, REG_CLKDIV0);
		//for (i = 0; i < 0x500; i++) ;
	}
	else {
		// enable APLL clock
		__raw_writel(__raw_readl(REG_APLLCON) & ~PD, REG_APLLCON);
		for (i = 0; i < 0x300; i++) ;
	}

	// exit SDRAM self refresh mode
	__raw_writel(__raw_readl(REG_SDCMD) & ~SELF_REF, REG_SDCMD);
	for (i = 0; i < 0x100; i++) ;
	// enable DLL of DDR2
	__raw_writel(__raw_readl(REG_SDEMR) & ~DLLEN, REG_SDEMR);
	// RESET DLL(bit[8]) of DDR2
	__raw_writel(0x532, REG_SDMR);
	for (i = 0; i < 0x100; i++) ;
	__raw_writel(0x432, REG_SDMR);

	// enable DLL of FA92
	__raw_writel(__raw_readl(SDRAM_BA+0x58) | BIT3 | BIT4, SDRAM_BA+0x54);
	for (i = 0; i < 10000; i++) ;
}

#if defined(CONFIG_RTC_DRV_W55FA92) && !defined(USE_REL_ALARM)
int rtc_add_day(struct rtc_wkalrm *alrm)
{
	unsigned char mdays[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
	unsigned char leap_year_add_day;

	leap_year_add_day = ((__raw_readl(REG_RTC_LIR) & LEAPYEAR) && ((alrm->time).tm_mon == 1)) ? 1 : 0;
	(alrm->time).tm_mday += 1;
	if ((alrm->time).tm_mday > (mdays[(alrm->time).tm_mon] + leap_year_add_day)) {
		(alrm->time).tm_mday = 1;
		(alrm->time).tm_mon += 1;
	}
	if ((alrm->time).tm_mon > 11) {
		(alrm->time).tm_mon = 0;
		(alrm->time).tm_year += 1;
	}

	return 0;
}
#endif

#define SHUTDOWN_TIME	30					// seconds
#define SHUTDOWN_COUNT	CLOCK_TICK_RATE/HZ*SHUTDOWN_TIME	// ticks

static ssize_t
read_clk(struct device *dev, struct device_attribute *attr, char *buffer)
{
	return snprintf(buffer, PAGE_SIZE, "[UPLL]%d:[SYS]%d:[CPU]%d:[HCLK]%d:[APB]%d\n",
			w55fa92_upll_clock, w55fa92_system_clock, w55fa92_cpu_clock,
			w55fa92_ahb_clock, w55fa92_apb_clock);
}

// nvt_mode: 0 - standard mode (control by drivers), 1 - nuvoton mode (control by this function)
void w55fa92_pm_suspend(int nvt_mode)
{
	void (*ps_func)(u8);
	unsigned long volatile flags;
	u32 int_mask, int_maskh, ahbclk, ahbclk2, apbclk;
	u32 lcm_tvctl, tp_ctl1, usbd_phy_ctl, misc_ctrl, gpiog_mod, gpiog_dout;
	//u32 adc_con, audio_con;
	u8 shutdown_flag;
#if defined(CONFIG_RTC_DRV_W55FA92)
#if defined(USE_REL_ALARM)
	u32 rtc_pwron;
#else
	struct rtc_wkalrm alrm, alrm_bak;
#endif
#endif

	local_irq_save(flags);
	int_mask = __raw_readl(REG_AIC_IMR);
	int_maskh = __raw_readl(REG_AIC_IMRH);
	// disable all interrupts
	__raw_writel(0xFFFFFFFF, REG_AIC_MDCR);
	__raw_writel(0xFFFFFFFF, REG_AIC_MDCRH);
	//restore_flags(flags);

	// save clock registers
	ahbclk = __raw_readl(REG_AHBCLK);
	ahbclk2 = __raw_readl(REG_AHBCLK2);
	apbclk = __raw_readl(REG_APBCLK);

	//if (nvt_mode == 1) {
	if (1) {
#if 0
		// turn off back light
		__raw_writel(__raw_readl(REG_GPDFUN) & ~(MF_GPD1), REG_GPDFUN);
		__raw_writel(__raw_readl(REG_GPIOD_OMD) | (1<<1), REG_GPIOD_OMD);
		__raw_writel(__raw_readl(REG_GPIOD_DOUT) & ~(1<<1), REG_GPIOD_DOUT);
#endif

		__raw_writel(__raw_readl(REG_AHBCLK) | (HCLK4_CKE|VPOST_CKE), REG_AHBCLK);
		lcm_tvctl = __raw_readl(REG_LCM_TVCtl);
		// set VPOST to grab build in color instead of SDRAM
		__raw_writel((__raw_readl(REG_LCM_TVCtl) & ~(TVCtl_LCDSrc)) | 0x800, REG_LCM_TVCtl);
		// disable TV DAC
		__raw_writel(__raw_readl(REG_LCM_TVCtl) | TVCtl_Tvdac, REG_LCM_TVCtl);
#if defined(CONFIG_FB_W55FA92)
		// wait vsync
		while (!(__raw_readl(REG_LCM_LCDCInt) & LCDCInt_VINT));
		__raw_writel((__raw_readl(REG_LCM_LCDCInt) & ~ LCDCInt_VINT), REG_LCM_LCDCInt);
		while (!(__raw_readl(REG_LCM_LCDCInt) & LCDCInt_VINT));		
		__raw_writel((__raw_readl(REG_LCM_LCDCInt) & ~ LCDCInt_VINT), REG_LCM_LCDCInt);		
#endif		
		// close VPOST engine clock
		__raw_writel(__raw_readl(REG_AHBCLK) & ~(VPOST_CKE), REG_AHBCLK);
	}

	// make sure no VDMA transaction in progress
	while (w55fa92_edma_isbusy(0));
	while (w55fa92_edma_isbusy(5));
	while (w55fa92_edma_isbusy(8));

#if 1
	// turn off speaker
	gpiog_mod = __raw_readl(REG_GPIOG_OMD);
	gpiog_dout = __raw_readl(REG_GPIOG_DOUT);
	__raw_writel(__raw_readl(REG_GPIOG_OMD) | (1 << 5), REG_GPIOG_OMD);
	__raw_writel(__raw_readl(REG_GPIOG_DOUT) & ~(1 << 5), REG_GPIOG_DOUT);
#endif
#if defined(CONFIG_SND_SOC_W55FA92_SPU)
	// set all SPU channels to be pause state
	__raw_writel(0xFFFFFFFF, REG_SPU_CH_PAUSE);
	// turn off SPU DAC
	spuDacOff();
#endif

	// clear wake-up status
	__raw_writel(__raw_readl(REG_MISSR) | 0xFF00C000, REG_MISSR);
	// enable wake-up source
	__raw_writel((__raw_readl(REG_MISSR) & ~0x00FF00C0) | (ADC_WE|RTC_WE|GPIO_WE), REG_MISSR);
#if defined(CONFIG_W55FA92_WAKE_ON_LAN)
	__raw_writel(__raw_readl(REG_MISSR) | EMAC_WE, REG_MISSR);
#endif

	//if (nvt_mode == 1) {
	if (1) {
		// for GPIO wake up
		// enable IRQ0 wake up and latch
		__raw_writel(0x11, REG_IRQLHSEL);
		// clear IRQ0 interrupt status
		__raw_writel(__raw_readl(REG_IRQTGSRC0) | 0x1C, REG_IRQTGSRC0);
	}

	//if (nvt_mode == 1) {
	if (1) {
#if defined(CONFIG_W55FA92_ETH)
		w55fa92_ether_power_down();
#endif
	}

	//if (nvt_mode == 1) {
	if (1) {
		// for H264 decode
		if (__raw_readl(REG_AHBCLK2) & VDEC_CKE) {
			do {
				if ((__raw_readl(REG_264_ADDR_STATUS0) & 0xFFFF0000) == 0)
					break;
				mdelay(10);
			} while(1);
		}
	}

	//if (nvt_mode == 1) {
	if (1) {
#if defined(CONFIG_RTC_DRV_W55FA92)
		// for RTC wake up
#if defined(USE_REL_ALARM)
		// store relative time alarm setting
		__raw_writel(apbclk | RTC_CKE, REG_APBCLK);
		rtc_pwron = __raw_readl(REG_RTC_PWRON);
		w55fa92_rtc_set_rel_alarm_wrap(SHUTDOWN_TIME);
#else
		// set RTC alarm time
		memset(&alrm, 0, sizeof(struct rtc_wkalrm));
		memset(&alrm_bak, 0, sizeof(struct rtc_wkalrm));
		w55fa92_rtc_read_alarm_wrap(&alrm_bak);
		w55fa92_rtc_read_time_wrap(&alrm.time);
		alrm.enabled = 1;
		alrm.pending = 0;
#if CPU_DEBUG
		printk("RTC = %d/%d/%d %d:%d:%d\n", alrm.time.tm_year, alrm.time.tm_mon, alrm.time.tm_mday, 
						alrm.time.tm_hour, alrm.time.tm_min, alrm.time.tm_sec);
#endif

		alrm.time.tm_sec += SHUTDOWN_TIME;
		while (alrm.time.tm_sec > 59) {
			alrm.time.tm_sec -= 60;
			alrm.time.tm_min += 1;
		}
		while (alrm.time.tm_min > 59) {
			alrm.time.tm_min -= 60;
			alrm.time.tm_hour += 1;
		}
		while (alrm.time.tm_hour > 23) {
			alrm.time.tm_hour -= 24;
			rtc_add_day(&alrm);
		}
		w55fa92_rtc_set_alarm_wrap(&alrm);
#endif
		// open RTC engine clock
		__raw_writel(apbclk | RTC_CKE, REG_APBCLK);
#endif
	}
	shutdown_flag = 0;

#if 0
	// disable Audio ADC, Touch ADC and LVR
	__raw_writel(__raw_readl(REG_APBCLK) | (ADC_CKE|TOUCH_CKE), REG_APBCLK);
	adc_con = __raw_readl(REG_ADC_CON);
	audio_con = __raw_readl(REG_AUDIO_CON);
	__raw_writel(0x0, REG_ADC_CON);
	__raw_writel(0x0, REG_AUDIO_CON);
	// enable LVR will keep about 70uA, but disable it may cause a side effect
	//__raw_writel(__raw_readl(REG_MISCR) & ~LVR_EN, REG_MISCR);
	//__raw_writel(__raw_readl(REG_POR_LVRD) & ~0xF, REG_POR_LVRD);
#endif

	// disable SADC
	tp_ctl1 = __raw_readl(REG_TP_CTL1);
	//__raw_writel(__raw_readl(REG_TP_CTL1) | (PD_Power|PD_BUF|LOW_SPEED), REG_TP_CTL1);											
	__raw_writel(__raw_readl(REG_TP_CTL1) | (PD_BUF|ADC_SLEEP), REG_TP_CTL1);
	__raw_writel((__raw_readl(REG_TP_CTL1) & ~(XP_EN|XM_EN|YP_EN)) | YM_EN, REG_TP_CTL1);
	// clear interrupt in advance
	__raw_writel(__raw_readl(REG_TP_INTST), REG_TP_INTST);
#if 0
	// for ADC touch wake up only, user can enable either ADC keypad or ADC touch
	// enable internal pull up
	__raw_writel(__raw_readl(REG_TP_CTL1) | (PLLUP|YM_EN), REG_TP_CTL1);
#endif
	__raw_writel(__raw_readl(REG_APBCLK) & ~(ADC_CKE|TOUCH_CKE), REG_APBCLK);

	// disable USB phy
	__raw_writel(__raw_readl(REG_AHBCLK) | USBD_CKE, REG_AHBCLK);
	usbd_phy_ctl = __raw_readl(REG_USBD_PHY_CTL);
	__raw_writel(__raw_readl(REG_USBD_PHY_CTL) & ~Phy_suspend, REG_USBD_PHY_CTL);
	__raw_writel(__raw_readl(REG_AHBCLK) & ~USBD_CKE, REG_AHBCLK);

	// disable USB Host Transceiver
	__raw_writel(__raw_readl(REG_AHBCLK) | USBH_CKE, REG_AHBCLK);
	misc_ctrl = __raw_readl(USBH_BA+0x200);
	__raw_writel(BIT27, USBH_BA+0x200);
	__raw_writel(__raw_readl(REG_USBPCR0) & ~BIT8, REG_USBPCR0);
	__raw_writel(__raw_readl(REG_AHBCLK) & ~USBH_CKE, REG_AHBCLK);

#if 0
	// close audio engine clocks, usb11h (bit 17) is for fa92
	__raw_writel(__raw_readl(REG_AHBCLK) & ~(ADO_CKE|I2S_CKE|SPU_CKE|BIT17), REG_AHBCLK);
	// close all edma engine clocks
	__raw_writel(__raw_readl(REG_AHBCLK) & ~(EDMA0_CKE|EDMA1_CKE|EDMA2_CKE|EDMA3_CKE|EDMA4_CKE), REG_AHBCLK);
	__raw_writel(__raw_readl(REG_AHBCLK2) & ~(EDMA5_CKE|EDMA6_CKE|EDMA20_CKE|EDMA21_CKE|EDMA22_CKE|EDMA23_CKE|EDMA24_CKE|EDMA25_CKE|EDMA26_CKE), REG_AHBCLK2);
#else
	// close all engine clocks
	__raw_writel(0x0000811F , REG_AHBCLK);	// cannot disable VDE engine
	__raw_writel(0x0 , REG_AHBCLK2);
	__raw_writel(__raw_readl(REG_APBCLK) & (UART1_CKE|UART0_CKE|RTC_CKE), REG_APBCLK);
#endif
#if CPU_DEBUG
	printk("REG_AHBCLK = 0x%x\n", __raw_readl(REG_AHBCLK));
	printk("REG_AHBCLK2 = 0x%x\n", __raw_readl(REG_AHBCLK2));
	printk("REG_APBCLK = 0x%x\n", __raw_readl(REG_APBCLK));
#endif

#if 0
	// for IO power
	// to disable the GPIO pins may affect some features with share pin, 
	// users needs to do it according the board setting

	// change SD card pin function
	__raw_writel(0x0, REG_GPAFUN0);
	__raw_writel(0x0, REG_GPAFUN1);
	__raw_writel(0x0, REG_GPBFUN0);
	__raw_writel(0x0, REG_GPBFUN1);
	__raw_writel(0x0, REG_GPCFUN0);
	__raw_writel(0x0, REG_GPCFUN1);
	__raw_writel(0x0, REG_GPCFUN0);
	__raw_writel(0x0, REG_GPCFUN1);
	__raw_writel(0x0, REG_GPEFUN0);
	__raw_writel(0x0, REG_GPEFUN1);

	printk("GPIOA STATUS = 0x%x\n", __raw_readl(REG_GPIOA_PIN));
	printk("GPIOB STATUS = 0x%x\n", __raw_readl(REG_GPIOB_PIN));
	printk("GPIOC STATUS = 0x%x\n", __raw_readl(REG_GPIOC_PIN));
	printk("GPIOD STATUS = 0x%x\n", __raw_readl(REG_GPIOD_PIN));
	printk("GPIOE STATUS = 0x%x\n", __raw_readl(REG_GPIOE_PIN));
	printk("GPIOG STATUS = 0x%x\n", __raw_readl(REG_GPIOG_PIN));
	printk("GPIOH STATUS = 0x%x\n", __raw_readl(REG_GPIOH_PIN));
	__raw_writel(0x0, REG_GPIOA_OMD);
	__raw_writel(0x0, REG_GPIOB_OMD);
	__raw_writel(0x0, REG_GPIOC_OMD);
	__raw_writel(0x0, REG_GPIOD_OMD);
	__raw_writel(0x0, REG_GPIOE_OMD);
	__raw_writel(0x0, REG_GPIOG_OMD);
	__raw_writel(0x0, REG_GPIOH_OMD);
	__raw_writel(0x3FF, REG_GPIOA_PUEN);
	__raw_writel(0xFFFF, REG_GPIOB_PUEN);
	__raw_writel(0xFFFF, REG_GPIOC_PUEN);
	__raw_writel(0xFFFF, REG_GPIOD_PUEN);
	__raw_writel(0x0FFF, REG_GPIOE_PUEN);
	// Bon suggest
	//__raw_writel(__raw_readl(REG_GPIOG_PUEN)& ~(BIT11|BIT12|BIT13|BIT14|BIT15), REG_GPIOG_PUEN);
	// pull up is inverse !	1.634~1.682mA
	__raw_writel(__raw_readl(REG_GPIOG_PUEN)& ~0xF85C, REG_GPIOG_PUEN);
	// Don't set GPIOH. R_FB will consume some power.
	//__raw_writel(0x0, REG_GPIOH_PUEN);

	//__raw_writel(0x0, REG_SHRPIN_AUDIO);
	__raw_writel(0x0, REG_SHRPIN_TOUCH);
#endif

	// kick into power down mode, make SDRAM enter self refresh mode
	// put enter_power_saving into SRAM
	memcpy(sram_vaddr, enter_power_saving, 1024);
	ps_func = (void(*)(u8)) sram_vaddr;
	// flush all TLB cache entries
	local_flush_tlb_all();
	// enter to power down mode
	ps_func(0x1);

	printk("REG_MISSR=0x%x\n", __raw_readl(REG_MISSR));
#if defined(CONFIG_W55FA92_SYSMGR) || defined(CONFIG_W55FA92_SYSMGR_MODULE)
	if (__raw_readl(REG_MISSR) & RTC_WS) {
		shutdown_flag = 1;
		sysmgr_report(SYSMGR_STATUS_RTC_POWER_OFF);
		printk("sysmgr_report(SYSMGR_STATUS_RTC_POWER_OFF)\n");
	}
#endif
	// clear wake-up status
	__raw_writel(__raw_readl(REG_MISSR) | 0xFF00C000, REG_MISSR);

	// restore Audio ADC, Touch ADC and LVR
	__raw_writel(__raw_readl(REG_APBCLK) | (ADC_CKE|TOUCH_CKE), REG_APBCLK);
#if 0
	//printk("adc_con=0x%x\n", adc_con);
	__raw_writel(adc_con, REG_ADC_CON);
	//printk("REG_ADC_CON=0x%x\n", __raw_readl(REG_ADC_CON));
	//printk("audio_con=0x%x\n", audio_con);
	__raw_writel(audio_con, REG_AUDIO_CON);
	//printk("REG_AUDIO_CON=0x%x\n", __raw_readl(REG_AUDIO_CON));
	//__raw_writel(__raw_readl(REG_MISCR) & ~LVR_EN, REG_MISCR);
#endif
	__raw_writel(tp_ctl1, REG_TP_CTL1);

	// restore TV DAC
	__raw_writel(__raw_readl(REG_AHBCLK) | (HCLK4_CKE|VPOST_CKE), REG_AHBCLK);
	//printk("lcm_tvctl=0x%x\n", lcm_tvctl);
	__raw_writel(lcm_tvctl, REG_LCM_TVCtl);
	//printk("REG_LCM_TVCtl=0x%x\n", __raw_readl(REG_LCM_TVCtl));

	// restore USB phy
	__raw_writel(__raw_readl(REG_AHBCLK) | (HCLK3_CKE|USBD_CKE), REG_AHBCLK);
	//printk("usbd_phy_ctl=0x%x\n", usbd_phy_ctl);
	__raw_writel(usbd_phy_ctl, REG_USBD_PHY_CTL);
	//printk("REG_USBD_PHY_CTL=0x%x\n", __raw_readl(REG_USBD_PHY_CTL));

	// restore USB Host Transceiver
	__raw_writel(__raw_readl(REG_AHBCLK) | USBH_CKE, REG_AHBCLK);
	__raw_writel(__raw_readl(REG_USBPCR0) | BIT8, REG_USBPCR0);
	//printk("misc_ctrl=0x%x\n", misc_ctrl);
	__raw_writel(misc_ctrl, USBH_BA+0x200);
	//printk("REG_MISC_CTRL=0x%x\n", __raw_readl(USBH_BA+0x200));

	//if (nvt_mode == 1) {
	if (1) {
#if defined(CONFIG_RTC_DRV_W55FA92)
#if defined(USE_REL_ALARM)
		// restore relative time alarm setting
		__raw_writel(apbclk | RTC_CKE, REG_APBCLK);
		__raw_writel(rtc_pwron, REG_RTC_PWRON);
		rtc_wait_ready();
#else
		// restore RTC alarm time
		w55fa92_rtc_set_alarm_wrap(&alrm_bak);
#endif
#endif
	}

	//if (nvt_mode == 1) {
	if (1) {
		if (ahbclk & VPOST_CKE) {
			__raw_writel(__raw_readl(REG_AHBCLK) | VPOST_CKE, REG_AHBCLK);
#if defined(CONFIG_GIANTPLUS_GPM1006D0_320X240)
//			mdelay(200);
#endif
		}
	}

	// restore registers
	__raw_writel(ahbclk, REG_AHBCLK);
	__raw_writel(ahbclk2, REG_AHBCLK2);
	__raw_writel(apbclk, REG_APBCLK);

#if defined(CONFIG_SND_SOC_W55FA92_SPU)
	// restore DAC SPU HPVDD33 and ADO
	//printk("spu_dac_val=0x%x\n", spu_dac_val);
	//__raw_writel(spu_dac_val, REG_SPU_DAC_VOL);
	//printk("REG_SPU_DAC_VOL=0x%x\n", __raw_readl(REG_SPU_DAC_VOL));
	// turn on SPU DAC
	spuDacOn();
	// set all SPU channels to be normal state
	__raw_writel(0x00000000, REG_SPU_CH_PAUSE);
#endif
#if 1
	// restore speaker setting
	__raw_writel(gpiog_mod, REG_GPIOG_OMD);
	__raw_writel(gpiog_dout, REG_GPIOG_DOUT);
#endif

	//if (nvt_mode == 1) {
	if (1) {
#if defined(CONFIG_W55FA92_ETH)
		w55fa92_ether_wakeup();
#endif
	}

	//save_flags(flags);
	//cli();
	// restore interrupt mask
	__raw_writel(int_mask, REG_AIC_MECR);
	__raw_writel(int_maskh, REG_AIC_MECRH);
	local_irq_restore(flags);

	//if (nvt_mode == 1) {
	if (1) {
		if (shutdown_flag == 0) {
			// VPOST get SDRAM data
			__raw_writel((__raw_readl(REG_LCM_TVCtl) & ~TVCtl_LCDSrc) | 0x400, REG_LCM_TVCtl);

			// turn on back light
			__raw_writel(__raw_readl(REG_GPIOD_DOUT) | (1<<1), REG_GPIOD_DOUT);
		}
	}
}

// you can decide what clocks are disabled in w55fa92_pm_idle()
void w55fa92_pm_idle(void)
{
#if 0
	u32 int_mask, ahbclk, apbclk, tcsr0, ticr0;
	u8 shutdown_flag, skip_check_shutdown, do_half_clock;

	if (buffer[0] == 'm' && buffer[1] == 'i') {
		// turn off back light
		__raw_writel(__raw_readl(REG_GPDFUN) & ~(MF_GPD1), REG_GPDFUN);
		__raw_writel(__raw_readl(REG_GPIOD_OMD) | (1<<1), REG_GPIOD_OMD);
		__raw_writel(__raw_readl(REG_GPIOD_DOUT) & ~(1<<1), REG_GPIOD_DOUT);

		// set VPOST to grab build in color instead of SDRAM
		__raw_writel((__raw_readl(REG_LCM_TVCtl) & ~TVCtl_LCDSrc) | 0x800, REG_LCM_TVCtl);
	}

	local_irq_save(flags);
	int_mask = __raw_readl(REG_AIC_IMR);
	// disable all interrupts
	__raw_writel(0xFFFFFFFF, REG_AIC_MDCR);
	//restore_flags(flags);

	// set all SPU channels to be pause state
	__raw_writel(0xFFFFFFFF, REG_SPU_CH_PAUSE);
	// close unused engine clocks
	ahbclk = __raw_readl(REG_AHBCLK);
	apbclk = __raw_readl(REG_APBCLK);
	if (buffer[0] == 'i' && buffer[1] == 'd') {
		// make sure no DMA transaction in progress
		mdelay(100);

		__raw_writel(ahbclk & ~(SEN_CKE|CAP_CKE|SD_CKE|NAND_CKE|SIC_CKE|JPG_CKE), REG_AHBCLK);
		__raw_writel(apbclk & ~(KPI_CKE|WDCLK_CKE|SPIMS1_CKE|SPIMS0_CKE), REG_APBCLK);
	}
	else if (buffer[0] == 'm' && buffer[1] == 'i') {
		// make sure no DMA transaction in progress
		mdelay(100);
		for (i = 0; i < 5; i++)
			while (w55fa92_edma_isbusy(i));

		__raw_writel(ahbclk & ~(SEN_CKE|CAP_CKE|VPOST_CKE|SD_CKE|NAND_CKE|SIC_CKE|EDMA4_CKE|EDMA3_CKE|EDMA2_CKE|EDMA1_CKE|EDMA0_CKE|JPG_CKE|BLT_CKE), REG_AHBCLK);
		__raw_writel(apbclk & ~(KPI_CKE|WDCLK_CKE|SPIMS1_CKE|SPIMS0_CKE|PWM_CKE), REG_APBCLK);
	}
	// divide clocks by 2, ex: drop system clock from 240MHz to 120MHz
	do_half_clock = 0;
	if ((w55fa92_system_clock == 240000) || (w55fa92_system_clock == 192000)) {
		set_system_clocks(w55fa92_system_clock/2, w55fa92_cpu_clock/2, w55fa92_apb_clock/2);
		do_half_clock = 1;
	}

	// store timer register
	tcsr0 = __raw_readl(REG_TCSR0);
	ticr0 = __raw_readl(REG_TICR0);
	// reset timer
	__raw_writel(__raw_readl(REG_APBIPRST) | TMR0RST, REG_APBIPRST);
	__raw_writel(__raw_readl(REG_APBIPRST) & ~TMR0RST, REG_APBIPRST);

	// wait reset complete
	for (i = 0; i < 0x1000; i++) ;
	__raw_writel(0x01, REG_TISR);
	__raw_writel(SHUTDOWN_COUNT, REG_TICR0);
	__raw_writel(0x60010063, REG_TCSR0);

	// enable wake up interrupts
	__raw_writel((1<<IRQ_ADC)|(1<<IRQ_GPIO0)|(1<<IRQ_TIMER0), REG_AIC_MECR);

	shutdown_flag = 0;
	skip_check_shutdown = 0;

	// close audio engine clocks
	__raw_writel(ahbclk & ~(ADO_CKE|I2S_CKE|SPU_CKE), REG_AHBCLK);
	// avoid key and touch pressing
#if defined(CONFIG_W55FA92_KEYPAD) && defined(CONFIG_TOUCHSCREEN_W55FA92)
	if ((w55fa92_key_pressing == 0) && (w55fa92_ts_pressing == 0)) {
#elif defined(CONFIG_W55FA92_KEYPAD)
	if (w55fa92_key_pressing == 0) {
#elif defined(CONFIG_TOUCHSCREEN_W55FA92)
	if (w55fa92_ts_pressing == 0) {
#else
	if (1) {
#endif
		if (buffer[0] == 'i' && buffer[1] == 'd') {
			// stop APLL clock
			__raw_writel(__raw_readl(REG_APLLCON) | PD, REG_APLLCON);
			for (i = 0; i < 0x300; i++) ;
			// stop CPU clock
			__raw_writel(__raw_readl(REG_AHBCLK) & ~CPU_CKE, REG_AHBCLK);
			for (i = 0; i < 0x300; i++) ;
			// enable APLL clock
			__raw_writel(__raw_readl(REG_APLLCON) & ~PD, REG_APLLCON);
			for (i = 0; i < 0x3000; i++) ;
		}
		else if (buffer[0] == 'm' && buffer[1] == 'i') {
			// kick into memory idle mode, make SDRAM enter self refresh mode
			// put enter_power_saving into SRAM
			memcpy(sram_vaddr, enter_power_saving, 1024);
			ps_func = (void(*)(u8)) sram_vaddr;

			// flush all TLB cache entries
			local_flush_tlb_all();
			// enter to memory idle mode
			ps_func(0x0);
		}
	}
	else
		skip_check_shutdown = 1;

	__raw_writel(0x20000063, REG_TCSR0);
	if ((__raw_readl(REG_TDR0) == 0x1) && (!skip_check_shutdown))
		shutdown_flag = 1;

	if ((ahbclk & VPOST_CKE) && !(__raw_readl(REG_AHBCLK) & VPOST_CKE)) {
		__raw_writel(__raw_readl(REG_AHBCLK) | VPOST_CKE, REG_AHBCLK);
#if defined(CONFIG_GIANTPLUS_GPM1006D0_320X240)
//			mdelay(200);
#endif
	}

	// restore clocks to full speed
	if (do_half_clock)
		set_system_clocks(w55fa92_system_clock*2, w55fa92_cpu_clock*2, w55fa92_apb_clock*2);
	// restore registers
	__raw_writel(ahbclk, REG_AHBCLK);
	__raw_writel(apbclk, REG_APBCLK);
	// set all SPU channels to be normal state
	__raw_writel(0x00000000, REG_SPU_CH_PAUSE);

	if (shutdown_flag == 1) {
#if defined(CONFIG_W55FA92_SYSMGR) || defined(CONFIG_W55FA92_SYSMGR_MODULE)
		sysmgr_report(SYSMGR_STATUS_RTC_POWER_OFF);
		printk("sysmgr_report(SYSMGR_STATUS_RTC_POWER_OFF)\n");
#endif
	}

	// reset timer
	__raw_writel(__raw_readl(REG_APBIPRST) | TMR0RST, REG_APBIPRST);
	__raw_writel(__raw_readl(REG_APBIPRST) & ~TMR0RST, REG_APBIPRST);
	// wait reset complete
	for (i = 0; i < 0x1000; i++) ;
	// store timer register
	__raw_writel(0x01, REG_TISR);
	__raw_writel(tcsr0, REG_TCSR0);
	__raw_writel(ticr0, REG_TICR0);

	//save_flags(flags);
	//cli();
	__raw_writel(0xFFFFFFFF, REG_AIC_MDCR);
	// restore interrupt mask
	__raw_writel(int_mask, REG_AIC_MECR);
	local_irq_restore(flags);

	if (shutdown_flag == 0) {
		// VPOST get SDRAM data
		__raw_writel((__raw_readl(REG_LCM_TVCtl) & ~TVCtl_LCDSrc) | 0x400, REG_LCM_TVCtl);

		// turn on back light
		__raw_writel(__raw_readl(REG_GPIOD_DOUT) | (1<<1), REG_GPIOD_DOUT);
	}
#endif
}

void w55fa92_poweroff(void)
{
	unsigned long volatile flags;
#if defined(CONFIG_RTC_DRV_W55FA92)
	int rtc_time_out;
#endif

	printk("enter to w55fa92_poweroff()\n");
	msleep(10);

	// disable LVR
	__raw_writel(__raw_readl(REG_MISCR) & ~(LVR_RDY | LVR_EN), REG_MISCR);

	// turn off speaker
#if defined(CONFIG_GIANTPLUS_GPM1006D0_320X240)
	__raw_writel(__raw_readl(REG_GPIOE_OMD) | (1 << 1), REG_GPIOE_OMD);
	__raw_writel(__raw_readl(REG_GPIOE_DOUT) & ~(1 << 1), REG_GPIOE_DOUT);
#endif

	// turn off video out
	__raw_writel((__raw_readl(REG_LCM_TVCtl) & ~TVCtl_LCDSrc) | 0x800, REG_LCM_TVCtl);

	// disable system interrupts
	local_irq_save(flags);

#if defined(CONFIG_RTC_DRV_W55FA92)
	__raw_writel(__raw_readl(REG_APBCLK) | RTC_CKE, REG_APBCLK);
	while (1) {
		rtc_time_out = 0;
		// enable long time press power disable
		if ((__raw_readl(REG_RTC_AER) & 0x10000) == 0x0) {
			// set RTC register access enable password
			__raw_writel(0xA965, REG_RTC_AER);
			// make sure RTC register read/write enable
			while ((__raw_readl(REG_RTC_AER) & 0x10000) == 0x0) {
				rtc_time_out++;
				if (rtc_time_out > 0xFFFFFF) {
					printk("RTC Access Eanble Fail\n");
					break;
				}
			}

			rtc_wait_ready();

			if ((__raw_readl(REG_RTC_AER) & 0x10000) == 0x10000)
				break;
		}
		else
			break;
	}

	// RTC will power off
	__raw_writel((__raw_readl(REG_RTC_PWRON) & ~0x5) | 0x2, REG_RTC_PWRON);
#else
	// turn off power
	__raw_writel(__raw_readl(REG_GPIOE_OMD) | (1<<9), REG_GPIOE_OMD);
	__raw_writel(__raw_readl(REG_GPIOE_DOUT) & ~(1<<9), REG_GPIOE_DOUT);
#endif

	// enable system interrupts
	local_irq_restore(flags);

	// stop CPU clock
	//__raw_writel(__raw_readl(REG_AHBCLK) & ~CPU_CKE, REG_AHBCLK);
	// fix RTC may wakeup fail issue
	__raw_writel(0x0, REG_AHBCLK);

	// wait system enter power off
	while (1) ;
}

void w55fa92_reboot(void)
{
	unsigned long volatile flags;

	local_irq_save(flags);
	printk("enter to w55fa92_reboot()\n");

	// turn off speaker
#if defined(CONFIG_GIANTPLUS_GPM1006D0_320X240)
	__raw_writel(__raw_readl(REG_GPIOE_OMD) | (1 << 1), REG_GPIOE_OMD);
	__raw_writel(__raw_readl(REG_GPIOE_DOUT) & ~(1 << 1), REG_GPIOE_DOUT);
#endif

	// turn off video out
	__raw_writel((__raw_readl(REG_LCM_TVCtl) & ~TVCtl_LCDSrc) | 0x800, REG_LCM_TVCtl);

	// close NAND and SIC engine clock
	__raw_writel(__raw_readl(REG_AHBCLK) & ~(NAND_CKE|SIC_CKE), REG_AHBCLK);

	// watchdog reset
	__raw_writel((__raw_readl(REG_WTCR) & ~(3<<4|1<<10))|0x2C2, REG_WTCR);

	// wait system enter power off
	while (1) ;
	local_irq_restore(flags);
}

static ssize_t
write_clk(struct device *dev, struct device_attribute *attr,
	  const char *buffer, size_t count)
{
	int i;

	// power down mode
	if (buffer[0] == 'p' && buffer[1] == 'd') {
		w55fa92_pm_suspend(1);
	}

#if 0
	// idle mode or memory idle mode
	else if ((buffer[0] == 'i' && buffer[1] == 'd') || (buffer[0] == 'm' && buffer[1] == 'i')) {
		w55fa92_pm_idle();
	}
#endif

#if defined(CONFIG_RTC_DRV_W55FA92)
	// RTC power off mode
	else if (buffer[0] == 'r' && buffer[1] == 'p' && buffer[2] == 'o') {
		w55fa92_poweroff();
	}
#else
	// power off mode
	else if (buffer[0] == 'p' && buffer[1] == 'o') {
		w55fa92_poweroff();
	}
#endif

	// power reset mode
	else if (buffer[0] == 'p' && buffer[1] == 'r') {
		w55fa92_reboot();
	}

	// CPU:PLL clock change
	else {
		u32 pll_clock, sys_clock, cpu_clock, apb_clock, vpost_clock;
		u8 vpost_div_N0, vpost_div_N1;
		char clock_buf[64];
		char *clock1, *clock2, *next;

		strncpy(clock_buf, buffer, count);
		next = &clock_buf[0];
		pll_clock = w55fa92_upll_clock;
		clock1 = strsep(&next, ":");
		//printk("clock1 = %s\n", clock1);
		cpu_clock = simple_strtol(clock1, NULL, 10) * 1000;
		if (cpu_clock == 0) {
			printk("Command \"%s\" does not support !!\n", clock1);
			return -1;
		}
		if (next) {
			clock2 = strsep(&next, ":");
			//printk("clock2 = %s\n", clock2);
			pll_clock = simple_strtol(clock2, NULL, 10) * 1000;
			if (pll_clock == 0) {
				printk("Command \"%s\" does not support !!\n", clock2);
				return -1;
			}
		}

		if (pll_clock % cpu_clock) {
			printk("UPLL clock(%d) is not a multiple of CPU clock(%d) !!\n", 
				pll_clock, cpu_clock);
			return -1;
		}

#if defined(CONFIG_FB_W55FA92)
		vpost_div_N0 = (__raw_readl(REG_CLKDIV1) & VPOST_N0) + 1;
		vpost_div_N1 = ((__raw_readl(REG_CLKDIV1) & VPOST_N1) >> 8) + 1;
		vpost_clock = pll_clock / (vpost_div_N0 * vpost_div_N1);
		if (cpu_clock > vpost_clock*2) {
			sys_clock = cpu_clock;
		} else {
			for (i = 1; ; i++) {
				sys_clock = cpu_clock * i * 2;
				if ((i > 8) || (sys_clock > pll_clock)) {
					printk("Cannot get valid System clock !!\n"); 
					return -1;
				}
				if ((sys_clock>(vpost_clock*2)) && ((pll_clock%sys_clock)==0))
					break;
			}
		}
#else
		sys_clock = cpu_clock;
#endif
		apb_clock = (cpu_clock == sys_clock) ? cpu_clock/4 : cpu_clock/2;
#if CPU_DEBUG
		printk("vpost_clock = %d\n", vpost_clock);
		printk("pll_clock = %d\n", pll_clock);
		printk("sys_clock = %d\n", sys_clock);
		printk("cpu_clock = %d\n", cpu_clock);
		printk("apb_clock = %d\n", apb_clock);
#endif

		// PLL:SYS:CPU:AHB:APB = pll_clock:sys_clock:cpu_clock:sys_clock/2:apb_clock
		set_system_clocks(pll_clock, sys_clock, cpu_clock, apb_clock);
	}

	return count;
}

/* Attach the sysfs write method */
DEVICE_ATTR(clock, 0644, read_clk, write_clk);

/* Attribute Descriptor */
static struct attribute *clk_attrs[] = {
	&dev_attr_clock.attr,
	NULL
};

/* Attribute group */
static struct attribute_group clk_attr_group = {
	.attrs = clk_attrs,
};

static int __init w55fa92_system_clock_init(void)
{
	/* Register a platform device */
	printk("register clock device\n");

	sys_clk = platform_device_register_simple("w55fa92-clk", -1, NULL, 0);
	if (sys_clk == NULL)
		printk("register failed\n");
	sysfs_create_group(&sys_clk->dev.kobj, &clk_attr_group);
	sram_vaddr = ioremap(0xFF000000, 4*1024);

	return 0;
}

module_init(w55fa92_system_clock_init);

#if 0
static int __init w55fa92_arch_init(void)
{
	int ret;
//	struct platform_device **ptr = w55fa92_board.devices;
	struct platform_device *ptr;
	ptr= &w55fa92_device_lcd;
	ret = platform_device_register(&w55fa92_device_lcd);
	printk("### Call platform_device_register in %s \n", __FUNCTION__);
	if (ret) {
		printk(KERN_ERR "w55fa92: failed to add board device %s (%d) @%p\n", (ptr)->name, ret, ptr);
	}

	return 0;
}

arch_initcall(w55fa92_arch_init);
#endif

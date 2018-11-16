/* linux/driver/rtc/rtc-w55fa92.c
 *
 * Copyright (c) 2008 Nuvoton technology corporation
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * Changelog:
 *
 *   2008/09/02     vincen.zswan add this file for nuvoton RTC.
 */

#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/clk.h>
#include <linux/rtc.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/string.h>
#include <linux/pm.h>
#include <linux/delay.h>
#include <asm/bitops.h>
#include <asm/irq.h>
#include <linux/rtc.h>
#include <asm/io.h>
#include <linux/math64.h>


#include <mach/w55fa92_reg.h>
#include <mach/w55fa92_rtc.h>

#define TIMER_FREQ		CLOCK_TICK_RATE
#define RTC_DEF_DIVIDER		32768 - 1
#define RTC_DEF_TRIM		0
static const unsigned char days_in_mo[] = 
{0, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

//static spinlock_t w55fa92_rtc_lock = SPIN_LOCK_UNLOCKED;
extern unsigned int w55fa92_apb_clock;
#ifdef CONFIG_W55FA92_RTC_USE_COMPENSATION
static struct timer_list rtc_timer;
#endif
int rtc_time_out;
struct clk  *rtc_clk;

static struct timer_list rtc_int_timer;

int volatile TAR_ALARM_MASK = 0;

int volatile CAR_ALARM_MASK = CAR_Mask_WD_Alarm;

#define RTC_DELAY	1000000 
#define INTERVAL_TIME  HZ*60*60	
#define RTC_INTERVAL_TIME  HZ	

void rtc_wait_ready(void)
{
	int volatile Wait,i;
	i =0;
	Wait = readl(REG_FLAG) & RTC_REG_FLAG;
    
	while(Wait!=RTC_REG_FLAG)
	{
    		Wait = readl(REG_FLAG) & RTC_REG_FLAG;
    		i++;
    		if(i > RTC_DELAY)
    		{
    			printk("RTC Wait Time out\n");
    			break;
 		}
  	}      
}
EXPORT_SYMBOL(rtc_wait_ready);

#ifdef CONFIG_W55FA92_RTC_USE_COMPENSATION
static void timer_check_rtc(unsigned long dummy)
{
	unsigned int count,integer, fraction; 
	long long rtc_tmp,rtc_value;
	unsigned int rtc_result;
	clk_enable(rtc_clk);
	
	writel((readl(REG_RTC_FCR) | FC_EN), REG_RTC_FCR);
	rtc_wait_ready();
	while(readl(REG_RTC_FCR) & FC_EN);	

	count = readl(REG_1Hz_CNT);
	
	if(count > 0)
	{
		rtc_value = (long long)w55fa92_apb_clock * 1000  * 32768 * 100;
			
		rtc_tmp = rtc_value;
			
		do_div(rtc_tmp, count);
	
		rtc_result = (unsigned int)rtc_tmp;		
			
		integer = ((unsigned int) (rtc_result / 100)) * 100;

		fraction = ((unsigned int)(rtc_result - integer) * 60 / 100) - 1;

		integer = integer / 100 - 1;

		writel((readl(REG_RTC_FCR) & ~(INTEGER |FRACTION)) | (((integer & 0xFFFF) << 8) | (fraction & 0x3F)), REG_RTC_FCR);
		rtc_wait_ready();			
	}
  	else
		goto end_check;	     			
	writel((readl(REG_RTC_FCR) & ~(INTEGER |FRACTION)) | (((integer & 0xFFFF) << 8) | (fraction & 0x3F)), REG_RTC_FCR);
	rtc_wait_ready();
	printk("RTC compensation value is 0x%X\n",readl(REG_RTC_FCR) );
end_check:
	clk_disable(rtc_clk);	
	mod_timer(&rtc_timer, jiffies + INTERVAL_TIME); 
}
#endif
static irqreturn_t w55fa92_rtc_interrupt(int irq, void *dev_id,
		struct pt_regs *regs)
{
	struct platform_device *pdev = to_platform_device(dev_id);
	struct rtc_device *rtc = platform_get_drvdata(pdev);
	unsigned long events = 0;
	static unsigned int rtc_irq = 0;
	unsigned int irq_clear=0;
	clk_enable(rtc_clk);
	rtc_irq = readl(REG_RTC_RIIR);
	irq_clear=rtc_irq;
	
	if(rtc_irq&ALARMINTENB){
		irq_clear = ALARMINTENB;
		writel(irq_clear ,REG_RTC_RIIR);
		rtc_wait_ready();
		events |= RTC_AF | RTC_IRQF;
	}
	
	if(rtc_irq&TICKINTENB){
		irq_clear = TICKINTENB;
		writel(irq_clear ,REG_RTC_RIIR);
		rtc_wait_ready();
		events |= RTC_PF | RTC_IRQF;
	}

	if(rtc_irq&PSWINTENB){
		irq_clear = PSWINTENB;
		writel(irq_clear ,REG_RTC_RIIR);
		rtc_wait_ready();
	}

	if(rtc_irq&RALARMINTENB){
		irq_clear = RALARMINTENB;
		writel(irq_clear ,REG_RTC_RIIR);
		rtc_wait_ready();
		events |= RTC_AF | RTC_IRQF;
	}
		
	//rtc_update_irq(&rtc->class_dev, 1, events);
	rtc_update_irq(rtc, 1, events);
	clk_disable(rtc_clk);
	return IRQ_HANDLED;
}

static int w55fa92_rtc_open(struct device *dev)
{
	int ret;
	//ret = request_irq(IRQ_RTC, w55fa92_rtc_interrupt, SA_INTERRUPT, "rtc", dev);
	ret = request_irq(IRQ_RTC, (irq_handler_t) w55fa92_rtc_interrupt, IRQF_DISABLED | IRQF_IRQPOLL, "rtc", dev);

	if (ret) {
		printk("RTC IRQ %d already in use.\n", IRQ_RTC);
        return -1;
	}
	return 0;

}

static void w55fa92_rtc_release(struct device *dev)
{
	free_irq (IRQ_RTC, dev);
}


static int w55fa92_rtc_ioctl(struct device *dev, unsigned int cmd,
		unsigned long arg)
{
	int scale=0;
	int tickount=0;
	int tickcount=0;
	clk_enable(rtc_clk);
	switch(cmd) {
		case RTC_AIE_OFF:
		{
			u32 u32Tmp;
			u32Tmp = readl(REG_RTC_PWRON) & ~ALARM_EN;	
	       		writel(u32Tmp, REG_RTC_PWRON);
			rtc_wait_ready();	
			writel(readl(REG_RTC_RIER)&(~ALARMINTENB),REG_RTC_RIER);
			rtc_wait_ready();
			clk_disable(rtc_clk);
			return 0;		
		}
		case RTC_AIE_ON:			
		{
			u32 u32Tmp;
			u32Tmp = readl(REG_RTC_PWRON)| ALARM_EN;	
	       		writel(u32Tmp, REG_RTC_PWRON);
			rtc_wait_ready();
			writel(readl(REG_RTC_RIER)|(ALARMINTENB),REG_RTC_RIER);
			rtc_wait_ready();
			clk_disable(rtc_clk);
			return 0;			
		}
		case RTC_TICK_OFF:
			writel(readl(REG_RTC_RIER)&(~TICKINTENB),REG_RTC_RIER);
			rtc_wait_ready();
			clk_disable(rtc_clk);
			return 0;		
		case RTC_TICK_ON:
			writel(readl(REG_RTC_RIER)|(TICKINTENB),REG_RTC_RIER);
			rtc_wait_ready();
			clk_disable(rtc_clk);
			return 0;
			
		case RTC_TIME_SCALE:		
			scale=	*(int*)arg; 
			if(scale==TIME24)
				writel(readl(REG_RTC_TSSR)|(HR24),REG_RTC_TSSR);
			if(scale==TIME12)
				writel(readl(REG_RTC_TSSR)|(HR12),REG_RTC_TSSR);
			rtc_wait_ready();	
			clk_disable(rtc_clk);	
			return 0;
			
		case RTC_TICK_READ:
			tickount=(readl(REG_RTC_TTR)&0x7F);
			*(int *)arg=tickount;
			clk_disable(rtc_clk);
			return 0;			
		case RTC_TICK_SET:	
			tickcount=(*(int*)arg);
			
			if(tickcount>7||tickcount<0){	
				printk("bad tickcount is %d\n",tickcount);	
				clk_disable(rtc_clk);
				return -EINVAL;	
			}	
	
			writel(0, REG_RTC_TTR);		
			rtc_wait_ready();	
			writel(tickcount, REG_RTC_TTR);
			rtc_wait_ready();	
			clk_disable(rtc_clk);	
			return 0;
		case RTC_RAIE_OFF:
			writel(readl(REG_RTC_RIER)&(~RALARMINTENB),REG_RTC_RIER);
			rtc_wait_ready();
			clk_disable(rtc_clk);
			return 0;		
		case RTC_RAIE_ON:
			writel(readl(REG_RTC_RIER)|(RALARMINTENB),REG_RTC_RIER);
			rtc_wait_ready();	
			clk_disable(rtc_clk);
			return 0;		

		case RTC_SET_RWALARM:
		case RTC_SET_RALARM:
		{
			u32 u32Tmp;
		       	writel((readl(REG_RTC_PWRON) & ~0xFFF00010),REG_RTC_PWRON);
	            	rtc_wait_ready();         
	              
			u32Tmp = (readl(REG_RTC_PWRON) & ~0xFFF00000)| ((arg & 0xFFF) <<20) | REL_ALARM_EN;
	
	       		writel(u32Tmp, REG_RTC_PWRON);
	           	rtc_wait_ready();
	                               
	       	    	u32Tmp = readl(REG_RTC_RIER) | RALARMINTENB;
	             	writel(u32Tmp, REG_RTC_RIER);          	
			rtc_wait_ready();	
			if(1)//cmd == RTC_SET_RWALARM)
			{
				enable_irq_wake(IRQ_RTC);
			}
			else
			{
				enable_irq(IRQ_RTC);	
			}
			clk_disable(rtc_clk);
			return 0;	
		}
		case RTC_GET_ACCESS_STATUS:
		{
			u32 temp;
			temp = (readl(REG_RTC_AER) & 0x10000) >> 16;
			copy_to_user((u32 *)arg, (u32 *)&temp , sizeof(temp));
			clk_disable(rtc_clk);
			return 0;
		}
		case RTC_GET_SW_STATUS:
		{
			u32 temp;
			temp = (readl(REG_RTC_PWRON) & 0xFF00) >> 8;
			copy_to_user((u32 *)arg, (u32 *)&temp , sizeof(temp));
			clk_disable(rtc_clk);
			return 0;
		}
		case RTC_SET_SW_STATUS:
		{
			u32 temp;
			temp = ((arg & 0xFF) << 8) | (readl(REG_RTC_PWRON) & ~0xFF00);
			writel(temp, REG_RTC_PWRON);
			rtc_wait_ready();
			clk_disable(rtc_clk);
			return 0;
		}
		case RTC_GET_SW_REG0_DATA:
		{
			u32 temp;
			temp = readl(REG_RTC_DUMMY0);
			copy_to_user((u32 *)arg, (u32 *)&temp , sizeof(temp));
			clk_disable(rtc_clk);
			return 0;
		}
		case RTC_SET_SW_REG0_DATA:
		{
			writel(arg, REG_RTC_DUMMY0);
			rtc_wait_ready();
			clk_disable(rtc_clk);
			return 0;
		}
		case RTC_GET_SW_REG1_DATA:
		{
			u32 temp;
			temp = readl(REG_RTC_DUMMY1);
			copy_to_user((u32 *)arg, (u32 *)&temp , sizeof(temp));
			clk_disable(rtc_clk);
			return 0;
		}
		case RTC_SET_SW_REG1_DATA:
		{
			writel(arg, REG_RTC_DUMMY1);
			rtc_wait_ready();
			clk_disable(rtc_clk);
			return 0;
		}
		case RTC_GET_ALARM_MASK:
		{
			u32 temp = 0;
			u32 temp_car, temp_tar;
			temp_car = readl(REG_RTC_CAR) & (CAR_Mask_WD_Alarm | CAR_Mask_Yr_Alarm | CAR_Mon_Alarm | CAR_Mask_Day_Alarm);
			temp_tar = readl(REG_RTC_TAR) & (TAR_Mask_HR_Alarm | TAR_Mask_Min_Alarm | TAR_Mask_Sec_Alarm);
			TAR_ALARM_MASK = temp_tar;
			CAR_ALARM_MASK = temp_car;
			if(temp_car & CAR_Mask_WD_Alarm)
				temp |= RTC_ALARM_MASK_DAYOFWEEK;
			if(temp_car & CAR_Mask_Yr_Alarm)
				temp |= RTC_ALARM_MASK_YEAR;
			if(temp_car & CAR_Mon_Alarm)
				temp |= RTC_ALARM_MASK_MONTH;
			if(temp_car & CAR_Mask_Day_Alarm)
				temp |= RTC_ALARM_MASK_DAY;
			if(temp_tar & TAR_Mask_HR_Alarm)
				temp |= RTC_ALARM_MASK_HOUR;
			if(temp_tar & TAR_Mask_Min_Alarm)
				temp |= RTC_ALARM_MASK_MINUTE;
			if(temp_tar & TAR_Mask_Sec_Alarm)
				temp |= RTC_ALARM_MASK_SECOND;
			copy_to_user((u32 *)arg, (u32 *)&temp , sizeof(temp));
			clk_disable(rtc_clk);
			return 0;
		}
		case RTC_SET_ALARM_MASK:
		{
			u32 u32Tmp;
			u32 temp = 0;
			u32 temp_car = 0, temp_tar = 0;
			u32Tmp = readl(REG_RTC_PWRON);	
			writel((u32Tmp & ~ALARM_EN), REG_RTC_PWRON);
			rtc_wait_ready();

			temp = arg;
			if(temp & RTC_ALARM_MASK_DAYOFWEEK)
				temp_car |= CAR_Mask_WD_Alarm;
			if(temp & RTC_ALARM_MASK_YEAR)
				temp_car |= CAR_Mask_Yr_Alarm;
			if(temp & RTC_ALARM_MASK_MONTH)
				temp_car |= CAR_Mon_Alarm;
			if(temp & RTC_ALARM_MASK_DAY)
				temp_car |= CAR_Mask_Day_Alarm;

			if(temp & RTC_ALARM_MASK_HOUR)
				temp_tar  |= TAR_Mask_HR_Alarm;
			if(temp & RTC_ALARM_MASK_MINUTE)
				temp_tar  |= TAR_Mask_Min_Alarm;
			if(temp & RTC_ALARM_MASK_SECOND )
				temp_tar  |= TAR_Mask_Sec_Alarm;
			TAR_ALARM_MASK = temp_tar;
			CAR_ALARM_MASK = temp_car;
			writel((readl(REG_RTC_CAR) & ~(CAR_Mask_WD_Alarm | CAR_Mask_Yr_Alarm | CAR_Mon_Alarm | CAR_Mask_Day_Alarm)) | temp_car,REG_RTC_CAR);
			rtc_wait_ready();

			writel((readl(REG_RTC_TAR) & ~(TAR_Mask_HR_Alarm | TAR_Mask_Min_Alarm | TAR_Mask_Sec_Alarm)) | temp_tar,REG_RTC_TAR);
			rtc_wait_ready();
			writel(u32Tmp, REG_RTC_PWRON);
			rtc_wait_ready();
			clk_disable(rtc_clk);
			return 0;
		}
		case RTC_ACCESS_ENABLE:
		{
			while(1)
			{	
				rtc_time_out = 0;
				// enable long time press power disable
				if ((readl(REG_RTC_AER) & 0x10000) == 0x0) {
					// set RTC register access enable password
					writel(0xA965, REG_RTC_AER);
					// make sure RTC register read/write enable
					while ((readl(REG_RTC_AER) & 0x10000) == 0x0)
					{
						rtc_time_out++;
						if(rtc_time_out > 0xFFFFFF)
						{
							printk("RTC Access Eanble Fail\n");
							break;
						}
					}
						
					rtc_wait_ready();

					if ((readl(REG_RTC_AER) & 0x10000) == 0x10000) 
						break;			
				}
				else
					break;
			}
			clk_disable(rtc_clk);
			return 0;
		}
		case RTC_ACCESS_DISABLE:
		{
			int i32i;
			writel(0x0, REG_RTC_AER);
	
			for (i32i = 0 ; i32i < RTC_DELAY ; i32i++)
			{
				if (readl(REG_RTC_AER) == 0)
					break;
		        }
			rtc_wait_ready();
			clk_disable(rtc_clk);
			return 0;		
		}
	}
	clk_disable(rtc_clk);
	return -ENOIOCTLCMD;
}

static int w55fa92_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	//printk("Get time\n");
	clk_enable(rtc_clk);
	tm->tm_sec =  RTC_SECONDS;
	tm->tm_min =  RTC_MINUTES;
	tm->tm_hour = RTC_HOURS;
	tm->tm_mday = RTC_DAY_OF_MONTH;
	tm->tm_mon =  RTC_MONTH - 1;
	tm->tm_year = RTC_YEAR + 100;
	tm->tm_wday = RTC_DAYOFWEEK;
	clk_disable(rtc_clk);
	//printk("To Lunix %d/%d/%d %d:%d:%d\n", tm->tm_year + 1900,tm->tm_mon,tm->tm_mday ,tm->tm_hour ,tm->tm_min ,tm->tm_sec);
	return 0;
}
int w55fa92_rtc_read_time_wrap(struct rtc_time *tm)
{
	return w55fa92_rtc_read_time(NULL, tm);
}
EXPORT_SYMBOL(w55fa92_rtc_read_time_wrap);

static int w55fa92_rtc_set_time(struct device *dev, struct rtc_time *tm)
{

	unsigned char mon, day, hrs, min, sec, leap_yr,wday;
	unsigned int yrs;
	clk_enable(rtc_clk);
//	printk("Set time\n");
	//printk("From Lunix %d/%d/%d %d:%d:%d\n", tm->tm_year + 1900,tm->tm_mon,tm->tm_mday,tm->tm_hour,tm->tm_min,tm->tm_sec);
	yrs = 1900 + tm->tm_year;
	if(((yrs % 4) == 0) )
	{	
	    	if(((yrs % 100) == 0) && ((yrs % 400) != 0) )
	    		leap_yr = 0;	
	    	else
	    		leap_yr = 1;
	}
	else
		leap_yr = 0;

	yrs = tm->tm_year - 100;
	mon = tm->tm_mon + 1;
	day = tm->tm_mday;
	hrs = tm->tm_hour;
	min = tm->tm_min;
	sec = tm->tm_sec;
	wday = tm->tm_wday;

	if ((mon > 12) || (day == 0))		
		return -EINVAL;
	if (day > (days_in_mo[mon] + ((mon == 2) && leap_yr)))
		return -EINVAL;
	if ((hrs >= 24) || (min >= 60) || (sec >= 60))
		return -EINVAL;
	
	writel((yrs/10)<<20|(yrs%10)<<16|(mon/10)<<12|(mon%10)<<8|(day/10)<<4|(day%10)<<0, REG_RTC_CLR);
	rtc_wait_ready();
	writel((hrs/10)<<20|(hrs%10)<<16|(min/10)<<12|(min%10)<<8|(sec/10)<<4|(sec%10)<<0, REG_RTC_TLR);
	rtc_wait_ready();
	writel(wday, REG_RTC_DWR);	
	rtc_wait_ready();
	clk_disable(rtc_clk);
	
	return 0;
}
int w55fa92_rtc_set_time_wrap(struct rtc_time *tm)
{
	return w55fa92_rtc_set_time(NULL, tm);
}
EXPORT_SYMBOL(w55fa92_rtc_set_time_wrap);

static int w55fa92_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	clk_enable(rtc_clk);
	(alrm->time).tm_sec = RTC_SECONDS_ALARM;
	(alrm->time).tm_min = RTC_MINUTES_ALARM;
	(alrm->time).tm_hour = RTC_HOURS_ALARM;
	(alrm->time).tm_mday = RTC_DAY_OF_MONTH_ALARM;
	(alrm->time).tm_mon = RTC_MONTH_ALARM - 1;
	(alrm->time).tm_year = RTC_YEAR_ALARM + 100;
	(alrm->time).tm_year = (alrm->time).tm_year;
	(alrm->time).tm_wday = (readl(REG_RTC_CAR) >> 28) & 0x07; 
	clk_disable(rtc_clk);	
	return 0;
}
int w55fa92_rtc_read_alarm_wrap(struct rtc_wkalrm *alrm)
{
	return w55fa92_rtc_read_alarm(NULL, alrm);
}
EXPORT_SYMBOL(w55fa92_rtc_read_alarm_wrap);


static int w55fa92_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	unsigned char mon, day, hrs, min, sec,yrs,week ;
	u32 u32Tmp;
	clk_enable(rtc_clk);
	u32Tmp = readl(REG_RTC_PWRON) & ~ALARM_EN;	
	writel(u32Tmp, REG_RTC_PWRON);
	rtc_wait_ready();
	yrs = (alrm->time).tm_year - 100;
	mon = (alrm->time).tm_mon + 1;
	day = (alrm->time).tm_mday;
	hrs = (alrm->time).tm_hour;
	min = (alrm->time).tm_min;
	sec = (alrm->time).tm_sec;
	week = (alrm->time).tm_wday; 
	
	/*if(alrm->enabled == 0){
		yrs = RTC_YEAR;
		mon = RTC_MONTH;
		day = RTC_DAY_OF_MONTH;
	}*/
	writel(((yrs/10)<<20|(yrs%10)<<16|(mon/10)<<12|(mon%10)<<8|(day/10)<<4|(day%10)<<0) | ((week & 0x07) << 28) | CAR_ALARM_MASK, REG_RTC_CAR);
	rtc_wait_ready();
	writel(((hrs/10)<<20|(hrs%10)<<16|(min/10)<<12|(min%10)<<8|(sec/10)<<4|(sec%10)<<0) |TAR_ALARM_MASK, REG_RTC_TAR);
	rtc_wait_ready();

	printk("(%d/%d/%d %d:%d:%d)\n", yrs,mon,day,hrs,min,sec);
	if (1)//alrm->enabled)
	{
		writel(readl(REG_RTC_RIER) | ALARMINTENB, REG_RTC_RIER);          	
		rtc_wait_ready();

		writel((readl(REG_RTC_PWRON) | ALARM_EN), REG_RTC_PWRON);
		rtc_wait_ready();	
		enable_irq_wake(IRQ_RTC);
	}
	else
	{
		writel(readl(REG_RTC_RIER) & ~ALARMINTENB, REG_RTC_RIER);          	
		rtc_wait_ready();
		writel((readl(REG_RTC_PWRON) & ~ALARM_EN), REG_RTC_PWRON);
		rtc_wait_ready();	
		disable_irq_wake(IRQ_RTC);		
	}
	clk_disable(rtc_clk);
	return 0;
}
int w55fa92_rtc_set_alarm_wrap(struct rtc_wkalrm *alrm)
{
	return w55fa92_rtc_set_alarm(NULL, alrm);
}
EXPORT_SYMBOL(w55fa92_rtc_set_alarm_wrap);

int w55fa92_rtc_set_rel_alarm_wrap(int second)
{

	u32 u32Tmp;
	clk_enable(rtc_clk);
	writel((readl(REG_RTC_PWRON) & ~0xFFF00010),REG_RTC_PWRON);
	rtc_wait_ready();         
	              
	u32Tmp = (readl(REG_RTC_PWRON) & ~0xFFF00000)| ((second & 0xFFF) <<20) | REL_ALARM_EN;
	
	writel(u32Tmp, REG_RTC_PWRON);
	rtc_wait_ready();
	                               
	u32Tmp = readl(REG_RTC_RIER) | RALARMINTENB;
	writel(u32Tmp, REG_RTC_RIER);          	
	rtc_wait_ready();	
	enable_irq_wake(IRQ_RTC);
	clk_disable(rtc_clk);
	return 0;
}
EXPORT_SYMBOL(w55fa92_rtc_set_rel_alarm_wrap);



static struct rtc_class_ops w55fa92_rtc_ops = {
	.open = w55fa92_rtc_open,
	.release = w55fa92_rtc_release,
	.ioctl = w55fa92_rtc_ioctl,
	.read_time = w55fa92_rtc_read_time,
	.set_time = w55fa92_rtc_set_time,
	.read_alarm = w55fa92_rtc_read_alarm,
	.set_alarm = w55fa92_rtc_set_alarm,
	
};
static void timer_check_rtc_interrupt(unsigned long dummy)
{	
	clk_enable(rtc_clk);
	mod_timer(&rtc_int_timer, jiffies + RTC_INTERVAL_TIME); 	
	clk_disable(rtc_clk);
	return;
}

static int w55fa92_rtc_probe(struct platform_device *pdev)
{
	struct rtc_device *rtc;
	
	if (!request_region((unsigned long)REG_RTC_INIR, (unsigned long)RTC_IO_EXTENT, "rtc"))
	{
		printk(KERN_ERR "rtc: I/O port 0x%x is not free.\n", (unsigned int)REG_RTC_INIR);
		return -EIO;
	}


	rtc = rtc_device_register(pdev->name, &pdev->dev, &w55fa92_rtc_ops,
				THIS_MODULE);

	if (IS_ERR(rtc))
		return PTR_ERR(rtc);

	platform_set_drvdata(pdev, rtc);

        rtc_clk = clk_get(NULL,"rtc");
        if (IS_ERR(rtc_clk)) {
              
		printk("failed to get rtc clock\n");
        }
        clk_enable(rtc_clk);
	printk("Check RTC Level Shift Status\n");
	rtc_time_out = 0;
	while(1)
	{
		if((readl(REG_RTC_INIR) & BIT1))
		{
			printk("-> Level Shift is enabled!\n");
			break;
		}
		else
		{
			rtc_time_out++;
			if(rtc_time_out > 1000000)
			{
				printk("-> Level Shift is enabled!\n");
				break;
			}
		}
	}
	
	printk("RTC Access Enable Check\n");
	while(1)
	{	
		rtc_time_out = 0;
		// enable long time press power disable
		if ((readl(REG_RTC_AER) & 0x10000) == 0x0) {
			// set RTC register access enable password
			writel(0xA965, REG_RTC_AER);
			// make sure RTC register read/write enable
			while ((readl(REG_RTC_AER) & 0x10000) == 0x0)
			{
				rtc_time_out++;
				if(rtc_time_out > 0xFFFFFF)
				{
					printk("RTC Access Eanble Fail\n");
					break;
				}
			}
			rtc_wait_ready();

			if ((readl(REG_RTC_AER) & 0x10000) == 0x10000) 
				break;			
		}
		else
			break;
	}
#ifdef CONFIG_W55FA92_RTC_ENABLE_HW_POWER_OFF
	// check rtc power off is set or not
	if ((readl(REG_RTC_PWRON) & 0x5) != 0x5) {
		// press power key during 6 sec to power off (0x'6'0005)
		u32 u32Tmp;
		u32Tmp = readl(REG_RTC_PWRON) & ~(0xF0000);	
	       	writel((u32Tmp | 0x60005), REG_RTC_PWRON);
		rtc_wait_ready();	
		writel(readl(REG_RTC_RIER) & ~0x4, REG_RTC_RIER);
		rtc_wait_ready();
		writel(0x4, REG_RTC_RIIR);
		rtc_wait_ready();
	}
	printk("Enable RTC H/W Power Off Function - 0x%X\n", readl(REG_RTC_PWRON));
#else
	if ((readl(REG_RTC_PWRON) & 0x5) == 0x5) {
		// press power key during 6 sec to power off (0x'6'0005)
		writel(readl(REG_RTC_RIER) & ~0x4, REG_RTC_RIER);
		rtc_wait_ready();
		writel(0x4, REG_RTC_RIIR);
		rtc_wait_ready();
	}
	printk("Not Change RTC H/W Power Off Function setting - 0x%X\n", readl(REG_RTC_PWRON));	
#endif
	printk("Init Nuvoton RTC!\n");
#ifdef CONFIG_W55FA92_RTC_USE_INERNAL_CRYSTAL
	writel(0x1, REG_OSC_32K);
#else
	writel(0x0, REG_OSC_32K);
#endif
	rtc_wait_ready();

	printk("<RTC Clock Source is from ");
	if(readl(REG_OSC_32K))
		printk("Internal Oscillator>\n");
	else
		printk("External Crystal>\n");

#ifdef CONFIG_W55FA92_RTC_USE_COMPENSATION
	printk("Use RTC Compensation\n"); 	
	init_timer(&rtc_timer);
  	rtc_timer.function = timer_check_rtc;	/* timer handler */	

	{
		unsigned int count,integer, fraction; 
		long long rtc_tmp,rtc_value;
		unsigned int rtc_result;
	
		writel((readl(REG_RTC_FCR) | FC_EN), REG_RTC_FCR);
		rtc_wait_ready();
		while(readl(REG_RTC_FCR) & FC_EN);	

		count = readl(REG_1Hz_CNT);
		
		if(count > 0)
		{
			rtc_value = (long long)w55fa92_apb_clock * 1000  * 32768 * 100;
			
			rtc_tmp = rtc_value;
			
			do_div(rtc_tmp, count);
	
			rtc_result = (unsigned int)rtc_tmp;		
			
			integer = ((unsigned int) (rtc_result / 100)) * 100;

			fraction = ((unsigned int)(rtc_result - integer) * 60 / 100) - 1;

			integer = integer / 100 - 1;

			writel((readl(REG_RTC_FCR) & ~(INTEGER |FRACTION)) | (((integer & 0xFFFF) << 8) | (fraction & 0x3F)), REG_RTC_FCR);
			rtc_wait_ready();			
			printk("RTC compensation value is 0x%X\n",readl(REG_RTC_FCR) );
		}
		else
			goto end;	

	}
	mod_timer(&rtc_timer, jiffies + INTERVAL_TIME); 
end:
#else
	writel((readl(REG_RTC_FCR) & ~0xFFFF3F) | 0x007FFF00, REG_RTC_FCR);
	rtc_wait_ready();
#endif
	clk_disable(rtc_clk);
	init_timer(&rtc_int_timer);
	rtc_int_timer.function = timer_check_rtc_interrupt;	/* timer handler */
	mod_timer(&rtc_int_timer, jiffies + RTC_INTERVAL_TIME); 

	

	return 0;
}

static int w55fa92_rtc_remove(struct platform_device *pdev)
{
	struct rtc_device *rtc = platform_get_drvdata(pdev);
		
 	if (rtc)
		rtc_device_unregister(rtc);
	release_region ((unsigned long)REG_RTC_INIR, (unsigned long)RTC_IO_EXTENT);
	return 0;
}

static struct platform_driver w55fa92_rtc_driver = {
	.probe		= w55fa92_rtc_probe,
	.remove		= w55fa92_rtc_remove,
	.driver		= {
		.name		= "w55fa92-rtc",
	},
};

static int __init w55fa92_rtc_init(void)
{
	return platform_driver_register(&w55fa92_rtc_driver);
}

static void __exit w55fa92_rtc_exit(void)
{
	platform_driver_unregister(&w55fa92_rtc_driver);
}

module_init(w55fa92_rtc_init);
module_exit(w55fa92_rtc_exit);

MODULE_DESCRIPTION("Nuvoton Realtime Clock Driver (RTC)");
MODULE_LICENSE("GPL");

/* linux/include/linux/w55fa92.h
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
 *   2008/11/10     First version
 */

#include <mach/w55fa92_reg.h>


/* GPIO group definition */
#define GPIO_GROUP_A 0
#define GPIO_GROUP_B 1
#define GPIO_GROUP_C 2
#define GPIO_GROUP_D 3
#define GPIO_GROUP_E 4
#define GPIO_GROUP_G 5
#define GPIO_GROUP_H 6

/* GPIO register offset definition */
static int __iomem * gpio_reg_dir[7] = { REG_GPIOA_OMD, REG_GPIOB_OMD, REG_GPIOC_OMD, REG_GPIOD_OMD, REG_GPIOE_OMD, REG_GPIOG_OMD, REG_GPIOH_OMD};
static int __iomem * gpio_reg_out[7] = { REG_GPIOA_DOUT, REG_GPIOB_DOUT, REG_GPIOC_DOUT, REG_GPIOD_DOUT, REG_GPIOE_DOUT, REG_GPIOG_DOUT, REG_GPIOH_DOUT};
static int __iomem * gpio_reg_in[7] = { REG_GPIOA_PIN, REG_GPIOB_PIN, REG_GPIOC_PIN, REG_GPIOD_PIN, REG_GPIOE_PIN, REG_GPIOG_PIN, REG_GPIOH_PIN};

/* returns the value of the GPIO pin */
static inline int w55fa92_gpio_get(int group, int num) 
{
	return readl(gpio_reg_in[group]) & (1 << num) ? 1:0;
}

/* set direction of pin to input mode */
static inline void w55fa92_gpio_set_input(int group, int num)
{
	writel (readl(gpio_reg_dir[group]) & ~(1 << num), gpio_reg_dir[group]); 
}

/* set direction of pin to output mode */
static inline void w55fa92_gpio_set_output(int group, int num)
{
	writel (readl(gpio_reg_dir[group]) | (1 << num), gpio_reg_dir[group]); 
}

/* drive the GPIO signal to state */
static inline void w55fa92_gpio_set(int group, int num, int state) 
{	
	if(state)
		writel (readl(gpio_reg_out[group]) | (1 << num), gpio_reg_out[group]); 		//set high			
	else
		writel (readl(gpio_reg_out[group]) & ~(1 << num), gpio_reg_out[group]); 	//set low
}

static inline void gpio_set_portg2digital(unsigned short num)
{
	switch (num) {
	
		case 15:
		case 14:
		case 13:
		case 12:
			writel(readl(REG_SHRPIN_TOUCH) &~ TP_AEN, REG_SHRPIN_TOUCH);
			break;		
		case 9:
			writel(readl(REG_SHRPIN_TOUCH) &~ SAR_AHS_AEN, REG_SHRPIN_TOUCH);
			break;
		case 8:
			writel(readl(REG_SHRPIN_AUDIO) &~ AIN2_AEN, REG_SHRPIN_AUDIO);
			break;
		case 7:
			writel(readl(REG_SHRPIN_AUDIO) &~ AIN3_AEN, REG_SHRPIN_AUDIO);
			break;		
		case 5:
		case 4:
		case 3:
		case 2:
			writel(readl(REG_SHRPIN_TVDAC) &~ SMTVDACAEN, REG_SHRPIN_TVDAC);
			break;
					
		
		default:
			;

	}
}

/* set share pin and direction of gpios */
static inline int w55fa92_gpio_configure(int group, int num)
{
	printk("w55fa92_gpio_configure()-%d,%d\n", group, num);
	if(num > 16)
		goto err;
		
	switch(group)
	{
		case GPIO_GROUP_A:	
			if(num <= 11)
			{
				if(num <= 7)
					writel(readl(REG_GPAFUN0) &~ (0xF << (num<<2)), REG_GPAFUN0);
				else
					writel(readl(REG_GPAFUN1) &~ (0xF << ((num%8)<<2)), REG_GPAFUN1);				
			}
			else
				writel((readl(REG_GPAFUN1) &~ (0xF << ((num%8)<<2))) | (0x2 << ((num%8)<<2)), REG_GPAFUN1);								
			break;
		
		case GPIO_GROUP_B:
			if(num <= 7)
				writel(readl(REG_GPBFUN0) &~ (0xF << (num<<2)), REG_GPBFUN0);
			else
				writel(readl(REG_GPBFUN1) &~ (0xF << ((num%8)<<2)), REG_GPBFUN1);								
			break;

		case GPIO_GROUP_C:
			if(num <= 7)
				writel(readl(REG_GPCFUN0) &~ (0xF << (num<<2)), REG_GPCFUN0);
			else
				writel(readl(REG_GPCFUN1) &~ (0xF << ((num%8)<<2)), REG_GPCFUN1);								
			break;
				
		case GPIO_GROUP_D:		
			if(num <= 7)
				writel(readl(REG_GPDFUN0) &~ (0xF << (num<<2)), REG_GPDFUN0);
			else
				writel(readl(REG_GPDFUN1) &~ (0xF << ((num%8)<<2)), REG_GPDFUN1);						
			break;
		
		case GPIO_GROUP_E:
			if(num>11)
				goto err;
			if(num <= 7)
				writel(readl(REG_GPEFUN0) &~ (0xF << (num<<2)), REG_GPEFUN0);
			else
				writel(readl(REG_GPEFUN1) &~ (0xF << ((num%8)<<2)), REG_GPEFUN1);									
			break;

		case GPIO_GROUP_G:
			if(num>=2 && num<=5)
			{
				gpio_set_portg2digital(num);
				writel(readl(REG_GPGFUN0) &~ (0xF << (num<<2)), REG_GPGFUN0);			
			}
			else if(num>=7 && num<=9)
			{
				gpio_set_portg2digital(num);
				if(num == 7)
					writel(readl(REG_GPGFUN0) &~ (0xF << (num<<2)), REG_GPGFUN0);
				else
					writel(readl(REG_GPGFUN1) &~ (0xF << ((num%8)<<2)), REG_GPGFUN1);							
			}
			else if(num>=12 && num<=15)
			{
				gpio_set_portg2digital(num);
				writel(readl(REG_GPGFUN1) &~ (0xF << ((num%8)<<2)), REG_GPGFUN1);			
			}
			else
				goto err;
			break;
		case GPIO_GROUP_H:
			if(num<=7)				
				writel(readl(REG_GPHFUN) &~ (0x3 << (num<<1)), REG_GPHFUN);			
			else
				goto err;
			break;
		default:
			break;
	}
	w55fa92_gpio_set_output(group, num);
	return 1;
	
err:
	return 0;
}

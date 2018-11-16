/* 
 * nau8822.h            Based on nau8822.h
 * Copyright (c) 2010 Nuvoton technology corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * Changelog:
 *
 */
/*
 * wm8978.h		--  codec driver for WM8978
 *
 * Copyright 2009 Guennadi Liakhovetski <g.liakhovetski@gmx.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __NAU8822_H__
#define __NAU8822_H__

/*
 * Register values.
 */
#define NAU8822_RESET				0x00
#define NAU8822_POWER_MANAGEMENT_1		0x01
#define NAU8822_POWER_MANAGEMENT_2		0x02
#define NAU8822_POWER_MANAGEMENT_3		0x03
#define NAU8822_AUDIO_INTERFACE			0x04
#define NAU8822_COMPANDING_CONTROL		0x05
#define NAU8822_CLOCKING				0x06
#define NAU8822_ADDITIONAL_CONTROL		0x07
#define NAU8822_GPIO_CONTROL			0x08
#define NAU8822_JACK_DETECT_CONTROL_1		0x09
#define NAU8822_DAC_CONTROL			0x0A
#define NAU8822_LEFT_DAC_DIGITAL_VOLUME		0x0B
#define NAU8822_RIGHT_DAC_DIGITAL_VOLUME		0x0C
#define NAU8822_JACK_DETECT_CONTROL_2		0x0D
#define NAU8822_ADC_CONTROL			0x0E
#define NAU8822_LEFT_ADC_DIGITAL_VOLUME		0x0F
#define NAU8822_RIGHT_ADC_DIGITAL_VOLUME		0x10
#define NAU8822_EQ1				0x12
#define NAU8822_EQ2				0x13
#define NAU8822_EQ3				0x14
#define NAU8822_EQ4				0x15
#define NAU8822_EQ5				0x16
#define NAU8822_DAC_LIMITER_1			0x18
#define NAU8822_DAC_LIMITER_2			0x19
#define NAU8822_NOTCH_FILTER_1			0x1b
#define NAU8822_NOTCH_FILTER_2			0x1c
#define NAU8822_NOTCH_FILTER_3			0x1d
#define NAU8822_NOTCH_FILTER_4			0x1e
#define NAU8822_ALC_CONTROL_1			0x20
#define NAU8822_ALC_CONTROL_2			0x21
#define NAU8822_ALC_CONTROL_3			0x22
#define NAU8822_NOISE_GATE			0x23
#define NAU8822_PLL_N				0x24
#define NAU8822_PLL_K1				0x25
#define NAU8822_PLL_K2				0x26
#define NAU8822_PLL_K3				0x27
#define NAU8822_3D_CONTROL			0x29
#define NAU8822_BEEP_CONTROL			0x2b
#define NAU8822_INPUT_CONTROL			0x2c
#define NAU8822_LEFT_INP_PGA_CONTROL		0x2d
#define NAU8822_RIGHT_INP_PGA_CONTROL		0x2e
#define NAU8822_LEFT_ADC_BOOST_CONTROL		0x2f
#define NAU8822_RIGHT_ADC_BOOST_CONTROL		0x30
#define NAU8822_OUTPUT_CONTROL			0x31
#define NAU8822_LEFT_MIXER_CONTROL		0x32
#define NAU8822_RIGHT_MIXER_CONTROL		0x33
#define NAU8822_LOUT1_HP_CONTROL			0x34
#define NAU8822_ROUT1_HP_CONTROL			0x35
#define NAU8822_LOUT2_SPK_CONTROL		0x36
#define NAU8822_ROUT2_SPK_CONTROL		0x37
#define NAU8822_OUT3_MIXER_CONTROL		0x38
#define NAU8822_OUT4_MIXER_CONTROL		0x39

#define NAU8822_CACHEREGNUM			58

/* Clock divider Id's */
enum nau8822_clk_id {
	NAU8822_OPCLKRATE,
	NAU8822_BCLKDIV,
};

enum nau8822_sysclk_src {
	NAU8822_PLL,
	NAU8822_MCLK
};

extern struct snd_soc_dai nau8822_dai;
extern struct snd_soc_codec_device soc_codec_dev_nau8822;

#endif	/* __NAU8822_H__ */

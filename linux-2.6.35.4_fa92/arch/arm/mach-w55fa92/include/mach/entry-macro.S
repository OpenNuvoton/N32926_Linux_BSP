/*
 * arch/arm/mach-w55fa92/include/mach/entry-macro.S
 *
 * Low-level IRQ helper macros for W55FA92 platforms
 *
 * This file is licensed under  the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 *
 */

#include <mach/hardware.h>
#include <mach/w55fa92_reg.h>

	.macro  get_irqnr_preamble, base, tmp
	.endm

	.macro  arch_ret_to_user, tmp1, tmp2
	.endm

	.macro	get_irqnr_and_base, irqnr, irqstat, base, tmp

		mov	\base, #AIC_BA

		ldr	\irqnr, [ \base, #AIC_IPER]
		ldr	\irqnr, [ \base, #AIC_ISNR]
		cmp	\irqnr, #0
		streq	\tmp, [\base, #AIC_EOSCR]	@ fix the fake interrupt issue

	.endm

	/* currently don't need an disable_fiq macro */

	.macro	disable_fiq
	.endm

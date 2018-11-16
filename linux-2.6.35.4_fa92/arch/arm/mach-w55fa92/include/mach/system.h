/*
 * arch/arm/mach-w55fa92/include/mach/system.h
 *
 * Copyright (c) 2013 Nuvoton technology corporation
 * All rights reserved.
 *
 * Based on arch/arm/mach-s3c2410/include/mach/system.h
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#include <asm/proc-fns.h>
#include <asm/io.h>
#include <asm/uaccess.h>

static void arch_idle(void)
{
}

extern void w55fa92_reboot(void);
static void arch_reset(char mode, const char *cmd)
{
	if (mode == 's') {
		/* Jump into ROM at address 0 */
		cpu_reset(0);
	} else {
		w55fa92_reboot();
	}
}


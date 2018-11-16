
#
# Makefile for the linux kernel.
#

# Object file lists.

obj-y			:= cpu.o irq.o time.o dev.o clock.o dmac.o fmi.o edma.o DrvEDMA.o
obj-m			:=
obj-n			:=
obj-			:=

obj-$(CONFIG_CPU_W55FA92)  += w55fa92.o

# machine specific support
obj-$(CONFIG_ARCH_W55FA92)	+= mach-w55fa92.o


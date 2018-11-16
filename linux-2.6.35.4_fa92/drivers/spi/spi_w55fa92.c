/* linux/drivers/spi/spi_w55fa92.c
 *
 * Copyright (c) 2009 Nuvoton technology.
 * Wan ZongShun <mcuos.com@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
*/

#include <linux/init.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/io.h>
#include <linux/slab.h>

#include <linux/spi/spi.h>
#include <linux/spi/spi_bitbang.h>

#include <asm/cacheflush.h>

#include <mach/w55fa92_spi.h>
#include <mach/w55fa92_reg.h>
#include <mach/DrvEDMA.h>

/* usi registers offset */
#define USI_CNT		0x00
#define USI_DIV		0x04
#define USI_SSR		0x08
#define USI_RX0		0x20
#define USI_TX0		0x20

/* usi register bit */
#define TXNUM		(0x07 << 21)
#define BYTEENDIN	(0x01 << 20)
#define ENINT		(0x01 << 17)
#define ENFLG		(0x01 << 16)
#define TXBIT		(0x1F << 3)
#define TXNEG		(0x01 << 2)
#define RXNEG		(0x01 << 1)
//#define LSB		(0x01 << 10)
#define SELECTLEV	(0x01 << 2)
#define SELECTPOL	(0x01 << 11)
#define SELECTSLAVE0	0x01
#define SELECTSLAVE1	(0x01 << 1)
#define GOBUSY		0x01

extern unsigned int w55fa92_apb_clock;
struct w55fa92_spi {
	struct spi_bitbang	 bitbang;
	struct completion	 done;
	void __iomem		*regs;
	int			 irq;
	int			 len;
	int			 count;
	int			 tx_num;
	const unsigned char	*tx;
	unsigned char		*rx;
	struct clk		*clk;
	struct resource		*ioarea;
	struct spi_master	*master;
	struct spi_device	*curdev;
	struct device		*dev;
	struct w55fa92_spi_info *pdata;
	spinlock_t		lock;
	struct resource		*res;
};

static inline struct w55fa92_spi *to_hw(struct spi_device *sdev)
{
	return spi_master_get_devdata(sdev->master);
}

static void w55fa92_slave_select(struct spi_device *spi, unsigned int ssr)
{
	struct w55fa92_spi *hw = to_hw(spi);
	unsigned int val;
	unsigned int cs = spi->mode & SPI_CS_HIGH ? 1 : 0;	
	unsigned long flags;

	spin_lock_irqsave(&hw->lock, flags);

	val = __raw_readl(hw->regs + USI_SSR);

	if (!cs)
		val &= ~SELECTLEV;
	else
		val |= SELECTLEV;

	if(spi->chip_select == 0)
	{
		if (!ssr)
			val &= ~SELECTSLAVE0;
		else
			val |= SELECTSLAVE0;
	}
	else
	{
		if (!ssr)
			val &= ~SELECTSLAVE1;
		else
			val |= SELECTSLAVE1;
	}

	__raw_writel(val, hw->regs + USI_SSR);	

	spin_unlock_irqrestore(&hw->lock, flags);
}

static void w55fa92_spi_chipsel(struct spi_device *spi, int value)
{
	switch (value) {
	case BITBANG_CS_INACTIVE:
		w55fa92_slave_select(spi, 0);
		break;

	case BITBANG_CS_ACTIVE:
		w55fa92_slave_select(spi, 1);
		break;
	}
}

static void w55fa92_spi_setup_txnum(struct w55fa92_spi *hw,
							unsigned int txnum)
{
	unsigned int val;
	unsigned long flags;

	spin_lock_irqsave(&hw->lock, flags);

	hw->tx_num = txnum;

	val = __raw_readl(hw->regs + USI_CNT) & ~TXNUM;

	val |= txnum << 21;	

	__raw_writel(val, hw->regs + USI_CNT);

	spin_unlock_irqrestore(&hw->lock, flags);

}

static void w55fa92_spi_setup_txbitlen(struct w55fa92_spi *hw,
							unsigned int txbitlen)
{
	unsigned int val;
	unsigned long flags;

	spin_lock_irqsave(&hw->lock, flags);

	val = __raw_readl(hw->regs + USI_CNT) & ~TXBIT;

	if(txbitlen == 32)
		txbitlen = 0;

	val |= (txbitlen << 0x03);

	__raw_writel(val, hw->regs + USI_CNT);

	spin_unlock_irqrestore(&hw->lock, flags);
}

static void w55fa92_spi_setup_byte_endin(struct w55fa92_spi *hw,
							unsigned int endin)
{
	unsigned int val;
	unsigned long flags;

	spin_lock_irqsave(&hw->lock, flags);

	val = __raw_readl(hw->regs + USI_CNT) & ~BYTEENDIN;

	val |= (endin << 20);

	__raw_writel(val, hw->regs + USI_CNT);

	spin_unlock_irqrestore(&hw->lock, flags);
}

static void w55fa92_spi_gobusy(struct w55fa92_spi *hw)
{
	unsigned int val;
	unsigned long flags;

	spin_lock_irqsave(&hw->lock, flags);

	val = __raw_readl(hw->regs + USI_CNT);

	val |= GOBUSY;

	__raw_writel(val, hw->regs + USI_CNT);

	spin_unlock_irqrestore(&hw->lock, flags);
}


#define	DRVSPI_SINGLE	0
#define	DRVSPI_DUAL	0x100
#define	DRVSPI_QUAD	0x200
static int w55fa92_spi_setBitMode(struct spi_device *spi,
				 struct spi_transfer *t)
{
	struct w55fa92_spi *hw = to_hw(spi);

	u32 status = __raw_readl(hw->regs + USI_CNT);
	status &= (~BIT_MODE);
	status &= (~SIO_DIR);
	
	switch (t->bit_mode) {

		case SPI_RX_DUAL:	//DRVSPI_DUAL
			status |= SIO_DIR;
		case SPI_TX_DUAL:	
			status |= DRVSPI_DUAL ;
		break;
			
		case SPI_RX_QUAD:	//DRVSPI_QUAD
			status |= SIO_DIR;
		case SPI_TX_QUAD:	
			status |= DRVSPI_QUAD ;
#ifdef CONFIG_FA92_D2_D3_GPE8_GPE9			
			writel( (readl(REG_GPEFUN1) & ~(0x000000FF)) | 0x00000044, REG_GPEFUN1);
#else
			writel( (readl(REG_GPEFUN0) & ~(0x000000FF)) | 0x00000033, REG_GPEFUN0);
#endif
		break;
	}
	__raw_writel(  status , hw->regs+USI_CNT );
	return 0;
}

static unsigned int spi_speed_hz =50000000;
static int w55fa92_spi_setupxfer(struct spi_device *spi,
				 struct spi_transfer *t)
{
	struct w55fa92_spi *hw = to_hw(spi);
	unsigned int val;	
	unsigned int cpol = spi->mode & SPI_CPOL ? 1 : 0;
	unsigned int cpha = spi->mode & SPI_CPHA ? 1 : 0;
	unsigned int lsb = spi->mode & SPI_LSB_FIRST ? 1 : 0;
	unsigned long flags;
	int divider;
		
	spin_lock_irqsave(&hw->lock, flags);

	val = __raw_readl(hw->regs + USI_CNT);

	if (!cpol)
	{
		val &= ~SELECTPOL;
		if (!cpha)	
			val = (val & ~(TXNEG | RXNEG)) | TXNEG;
		else
			val = (val & ~(TXNEG | RXNEG)) | RXNEG;		
	}
	else
	{
		val |= SELECTPOL;
		if (!cpha)	
			val = (val & ~(TXNEG | RXNEG)) | RXNEG;
		else
			val = (val & ~(TXNEG | RXNEG)) | TXNEG;
	}

	if (!lsb)
		val &= ~LSB;
	else
		val |= LSB;

	__raw_writel(val, hw->regs + USI_CNT);	

	if(spi_speed_hz != spi->max_speed_hz)
	{	
		spi_speed_hz = spi->max_speed_hz;
		divider = (w55fa92_apb_clock * 1000) / (2 * spi->max_speed_hz) - 1;
		if(divider < 0)
			divider = 0;
		//printk("spi divider %d %d %d\n", w55fa92_apb_clock * 1000, spi->max_speed_hz, divider);
				
		__raw_writel(divider, hw->regs + USI_DIV);
		
	}
	
	spin_unlock_irqrestore(&hw->lock, flags);

	return 0;
}

static int w55fa92_spi_setup(struct spi_device *spi)
{	
	return 0;
}

static inline unsigned int hw_txbyte(struct w55fa92_spi *hw, int count)
{
	return hw->tx ? hw->tx[count] : 0;
}

static inline unsigned int hw_txword(struct w55fa92_spi *hw, int count)
{
	unsigned int * p32tmp;	

	if(hw->tx == 0)
		return 0;
	else
	{
		p32tmp = (unsigned int *)((unsigned int )(hw->tx) + count);		
		return *p32tmp;
	}		
}

static int w55fa92_spi_txrx_transfer ( struct w55fa92_spi *hw )
{
	u32 	i, val, transfer_len, is_write;
	u32* 	pU32;

	// Busy-waiting method. The performence is more than original interrupt-mode. (~7%)
	// Disable IE
	__raw_writel ( __raw_readl(hw->regs + USI_CNT) & (~ENINT), hw->regs + USI_CNT );

	if ( hw->rx ) {
		pU32 = (u32*)hw->rx;
		is_write = 0;
	} else {
		pU32 = (u32*)hw->tx;
		is_write = 1;
	}

	transfer_len = hw->len / 4;
	if ( transfer_len )	// for word-transfer
	{
		u32 u32TxNum ;

		val = __raw_readl(hw->regs + USI_CNT) & ~BYTEENDIN;
		val |= (1 << 20);
		__raw_writel(val, hw->regs + USI_CNT);

		while ( transfer_len > 0 )
		{
			u32TxNum = transfer_len%8;
			if ( !u32TxNum )
				u32TxNum = 8;

			// word-transfer
			val = __raw_readl(hw->regs + USI_CNT) & ~TXBIT;
			__raw_writel(val, hw->regs + USI_CNT);

			// multi-transfer
			val = __raw_readl(hw->regs + USI_CNT) & ~TXNUM ;
			val |= ((u32TxNum-1) << 21);
			__raw_writel( val , hw->regs + USI_CNT);

			for ( i=0; i<u32TxNum  ; i++ )
			{
				if ( is_write )
					__raw_writel ( *pU32++, hw->regs + (USI_TX0 + 4*i));
				else
					__raw_writel ( 0xffffffff, hw->regs + (USI_TX0 + 4*i));
			}

			val = __raw_readl(hw->regs + USI_CNT);
			val |= GOBUSY;
			__raw_writel(val, hw->regs + USI_CNT);
			while ( __raw_readl(hw->regs + USI_CNT)&GOBUSY );
	
			if ( !is_write ) // for read
				for ( i=0; i< u32TxNum ; i++ )
					*pU32++ = __raw_readl(hw->regs + (USI_RX0  + 4*i) ) ;

			transfer_len -= u32TxNum;
		}

		val = __raw_readl(hw->regs + USI_CNT) & ~BYTEENDIN;
		val |= (0 << 20);
		__raw_writel(val, hw->regs + USI_CNT);
	}

	transfer_len = hw->len % 4;
	if ( transfer_len )	// for byte-transfer
	{
		u8*	pU8 = (u8*)pU32;

		for ( i=0; i<transfer_len; i++ )
		{
			/* How many bits recieve/send in this transfer, always 8bite */
			val = __raw_readl(hw->regs + USI_CNT) & ~TXBIT;
			val |= ( 8 << 0x03);
			__raw_writel(val, hw->regs + USI_CNT);

			// one-shot
			__raw_writel( (__raw_readl(hw->regs + USI_CNT) & ~TXNUM) , hw->regs + USI_CNT);

			if ( is_write ) 
				__raw_writel ( pU8[i], 	hw->regs + USI_TX0);
			else
				__raw_writel ( 0xff, 	hw->regs + USI_TX0);

			val = __raw_readl(hw->regs + USI_CNT);
			val |= GOBUSY;
			__raw_writel(val, hw->regs + USI_CNT);
			while ( __raw_readl(hw->regs + USI_CNT)&GOBUSY );

			if ( !is_write ) // for read
				pU8[i] = __raw_readl(hw->regs + USI_RX0)&0xFF ;
		}

	}

	return hw->len;

}

#ifdef CONFIG_SPI_USE_PDMA
static void w55fa92_spi_pdma_transfer_remain_data(struct w55fa92_spi *hw, unsigned char * addr, unsigned int len, unsigned int is_write)
{
	u32 	i, val;
	u8*	pU8;

	pU8 = addr;
	
	w55fa92_spi_setup_byte_endin(hw, 0);
	w55fa92_spi_setup_txbitlen(hw, 8);
	w55fa92_spi_setup_txnum(hw, 0);	

	for ( i=0; i<len; i++ )
	{
		if ( is_write ) 
			__raw_writel ( pU8[i], 	hw->regs + USI_TX0);
		else
			__raw_writel ( 0xff, 	hw->regs + USI_TX0);

		val = __raw_readl(hw->regs + USI_CNT);
		val |= GOBUSY;
		__raw_writel(val, hw->regs + USI_CNT);
		while ( __raw_readl(hw->regs + USI_CNT)&GOBUSY );

		if ( !is_write ) // for read
			pU8[i] = __raw_readl(hw->regs + USI_RX0)&0xFF ;
	}
	
}

struct w55fa92_spi * spi0_phw, * spi1_phw;

void spi0_edma_irq_handler(void)
{	  	
	complete(&spi0_phw->done);
}

void spi1_edma_irq_handler(void)
{	  	
	complete(&spi1_phw->done);
}

#define CLIENT_SPI_NAME "w55fa93-spi"
static int w55fa92_spi_pdma_transfer ( struct w55fa92_spi *hw )
{		
	unsigned char	* buf_addr;
	int edma_channel, ret;	
	unsigned int is_write, phys_addr, transfer_len, tmp_len0=0, tmp_len;

	__raw_writel ( __raw_readl(hw->regs + USI_CNT) & (~ENINT), hw->regs + USI_CNT );	

	if(hw->pdata->bus_num == 0)		//SPI0
		spi0_phw = hw;
	else
		spi1_phw = hw;
	
	transfer_len = hw->len;

	if ( hw->rx ) {
		buf_addr = hw->rx;
		is_write = 0;
	} else {
		buf_addr = (unsigned char *)hw->tx;
		is_write = 1;
	}

	phys_addr = virt_to_phys(buf_addr);		

	if(phys_addr %4 != 0)
	{
		//printk("phys_addr is not 4 align\n");
		tmp_len0 = 4 -(phys_addr %4);
		w55fa92_spi_pdma_transfer_remain_data(hw, buf_addr, tmp_len0, is_write);
		phys_addr += tmp_len0; 
		transfer_len -= tmp_len0;
	}

	if((transfer_len%4) != 0)	
		tmp_len = (transfer_len/4)*4;	
	else
		tmp_len = transfer_len;

	w55fa92_spi_setup_byte_endin(hw, 1);
	w55fa92_spi_setup_txbitlen(hw, 32);
	w55fa92_spi_setup_txnum(hw, 0);

	edma_channel = w55fa92_pdma_find_and_request(CLIENT_SPI_NAME); /* request EDMA channel */
	if(edma_channel == -ENODEV){
		printk("Run out of EDMA channel\n");
		return -ENODEV;
	}

	if(hw->pdata->bus_num == 0)		//SPI0
	{	
		if(is_write == 1)
		{
			w55fa92_edma_setAPB(edma_channel,					//int channel, 
					eDRVEDMA_SPIMS0,			//E_DRVEDMA_APB_DEVICE eDevice, 
					eDRVEDMA_WRITE_APB,		//E_DRVEDMA_APB_RW eRWAPB, 
					eDRVEDMA_WIDTH_32BITS);		//E_DRVEDMA_TRANSFER_WIDTH eTransferWidth
		}
		else
		{
			w55fa92_edma_setAPB(edma_channel,					//int channel, 
					eDRVEDMA_SPIMS0,			//E_DRVEDMA_APB_DEVICE eDevice, 
					eDRVEDMA_READ_APB,		//E_DRVEDMA_APB_RW eRWAPB, 
					eDRVEDMA_WIDTH_32BITS);		//E_DRVEDMA_TRANSFER_WIDTH eTransferWidth
		}		

		w55fa92_edma_setup_handlers(edma_channel, 				//int channel
						eDRVEDMA_BLKD, 				//int interrupt,						
						(void*)spi0_edma_irq_handler,
						0);					//void *data	

		w55fa92_edma_set_wrapINTtype(edma_channel , 0);	
			
		if(is_write == 1)
		{						
			w55fa92_edma_set_direction(edma_channel , eDRVEDMA_DIRECTION_INCREMENTED, eDRVEDMA_DIRECTION_FIXED);

			ret = w55fa92_edma_setup_single(edma_channel, phys_addr, (u32)0xB800c020, tmp_len);
				if (ret < 0) {
					printk("w55fa93_edma_setup_single failed and returns %d\n", ret);
					goto out_unmap_single;
			}
		}
		else
		{						
			w55fa92_edma_set_direction(edma_channel , eDRVEDMA_DIRECTION_FIXED, eDRVEDMA_DIRECTION_INCREMENTED);

			ret = w55fa92_edma_setup_single(edma_channel, (u32)0xB800c020, phys_addr, tmp_len);
				if (ret < 0) {
					printk("w55fa93_edma_setup_single failed and returns %d\n", ret);
					goto out_unmap_single;
			}		
		}
			
		flush_cache_all();
		w55fa92_edma_trigger(edma_channel);

		if(is_write == 1)		
			__raw_writel((__raw_readl((u32)REG_SPI0_EDMA) & ~0x03) | EDMA_GO , (u32)REG_SPI0_EDMA);
		else
		{
			__raw_writel((__raw_readl((u32)REG_SPI0_EDMA) & ~0x03) | (EDMA_RW | EDMA_GO) , (u32)REG_SPI0_EDMA);
			__raw_writel((__raw_readl((u32)REG_SPI0_CNTRL) |GO_BUSY) , (u32)REG_SPI0_CNTRL);
		}
		
		wait_for_completion(&hw->done);

		if(is_write == 1)
			while ( __raw_readl(hw->regs + USI_CNT)&GOBUSY );

		w55fa92_edma_free(edma_channel);

		__raw_writel((__raw_readl((u32)REG_SPI0_EDMA) & ~EDMA_GO) , (u32)REG_SPI0_EDMA);

	}
	else		//SPI1
	{
		if(is_write == 1)
		{
			w55fa92_edma_setAPB(edma_channel,					//int channel, 
					eDRVEDMA_SPIMS1,			//E_DRVEDMA_APB_DEVICE eDevice, 
					eDRVEDMA_WRITE_APB,		//E_DRVEDMA_APB_RW eRWAPB, 
					eDRVEDMA_WIDTH_32BITS);		//E_DRVEDMA_TRANSFER_WIDTH eTransferWidth
		}
		else
		{
			w55fa92_edma_setAPB(edma_channel,					//int channel, 
					eDRVEDMA_SPIMS1,			//E_DRVEDMA_APB_DEVICE eDevice, 
					eDRVEDMA_READ_APB,		//E_DRVEDMA_APB_RW eRWAPB, 
					eDRVEDMA_WIDTH_32BITS);		//E_DRVEDMA_TRANSFER_WIDTH eTransferWidth
		}		

		w55fa92_edma_setup_handlers(edma_channel, 				//int channel
						eDRVEDMA_BLKD, 				//int interrupt,						
						(void*)spi1_edma_irq_handler,
						0);					//void *data	

		w55fa92_edma_set_wrapINTtype(edma_channel , 0);	
			
		if(is_write == 1)
		{						
			w55fa92_edma_set_direction(edma_channel , eDRVEDMA_DIRECTION_INCREMENTED, eDRVEDMA_DIRECTION_FIXED);

			ret = w55fa92_edma_setup_single(edma_channel, phys_addr, (u32)0xB800c420, tmp_len);
				if (ret < 0) {
					printk("w55fa93_edma_setup_single failed and returns %d\n", ret);
					goto out_unmap_single;
			}
		}
		else
		{						
			w55fa92_edma_set_direction(edma_channel , eDRVEDMA_DIRECTION_FIXED, eDRVEDMA_DIRECTION_INCREMENTED);

			ret = w55fa92_edma_setup_single(edma_channel, (u32)0xB800c420, phys_addr, tmp_len);
				if (ret < 0) {
					printk("w55fa93_edma_setup_single failed and returns %d\n", ret);
					goto out_unmap_single;
			}		
		}
			
		flush_cache_all();
		w55fa92_edma_trigger(edma_channel);

		if(is_write == 1)		
			__raw_writel((__raw_readl((u32)REG_SPI0_EDMA + 0x400) & ~0x03) | EDMA_GO , (u32)REG_SPI0_EDMA + 0x400);
		else
		{
			__raw_writel((__raw_readl((u32)REG_SPI0_EDMA + 0x400) & ~0x03) | (EDMA_RW | EDMA_GO) , (u32)REG_SPI0_EDMA + 0x400);
			__raw_writel((__raw_readl((u32)REG_SPI0_CNTRL + 0x400) |GO_BUSY) , (u32)REG_SPI0_CNTRL + 0x400);
		}
		
		wait_for_completion(&hw->done);

		if(is_write == 1)
			while ( __raw_readl(hw->regs + USI_CNT)&GOBUSY );

		w55fa92_edma_free(edma_channel);

		__raw_writel((__raw_readl((u32)REG_SPI0_EDMA + 0x400) & ~EDMA_GO) , (u32)REG_SPI0_EDMA + 0x400);
	}

	transfer_len = hw->len - tmp_len0 - tmp_len;
	if(transfer_len)
	{
		buf_addr = buf_addr + tmp_len0 + tmp_len;
		w55fa92_spi_pdma_transfer_remain_data(hw, buf_addr, transfer_len, is_write);
	}
	else
	{
		w55fa92_spi_setup_byte_endin(hw, 0);
		w55fa92_spi_setup_txbitlen(hw, 8);
		w55fa92_spi_setup_txnum(hw, 0);	
	}
	
out_unmap_single:

	if (ret < 0)
		return ret;
	else
		return hw->len;
	
}	
#endif

static int w55fa92_spi_txrx(struct spi_device *spi, struct spi_transfer *t)
{
	int i;
	struct w55fa92_spi *hw = to_hw(spi);

	hw->tx = t->tx_buf;
	hw->rx = t->rx_buf;
	hw->len = t->len;
	hw->count = 0;

	if(hw->pdata->bus_num == 0)
		w55fa92_spi_setBitMode(	spi, t );
	
#ifdef CONFIG_SPI_USE_PDMA
	if(hw->len > 32)
	{
		i = w55fa92_spi_pdma_transfer(hw);
		if(i>0)
			hw->count = i;
		else
			hw->count = 0;
	}
	else
		hw->count = w55fa92_spi_txrx_transfer (hw);
#else	

#if 1
	//Interrupt-mode.

	if ( hw->len < 4 )
	{
		w55fa92_spi_setup_byte_endin(hw, 0);
		w55fa92_spi_setup_txbitlen(hw, 8);
		w55fa92_spi_setup_txnum(hw, 0);
		__raw_writel(hw_txbyte(hw, 0x0), hw->regs + USI_TX0);
	
	}
	else
	{
		w55fa92_spi_setup_byte_endin(hw, 1);
		w55fa92_spi_setup_txbitlen(hw, 32);

		if(hw->len >= 32)
		{
			w55fa92_spi_setup_txnum(hw, 7);
			for(i=0;i<(hw->tx_num+1);i++)
				__raw_writel(hw_txword(hw, i*4), hw->regs + USI_TX0 + i*4);
		}
		else
		{
			w55fa92_spi_setup_txnum(hw, 0);
			__raw_writel(hw_txword(hw, 0x0), hw->regs + USI_TX0);	
		}
	}

	w55fa92_spi_gobusy(hw);

	wait_for_completion(&hw->done);

#else
	hw->count = w55fa92_spi_txrx_transfer (hw);
#endif

#endif
	
	return hw->count;
}

static irqreturn_t w55fa92_spi_irq(int irq, void *dev)
{
	struct w55fa92_spi *hw = dev;
	unsigned int status;
	unsigned int count = hw->count;
	unsigned int val,i;
	unsigned int * p32tmp;

	status = __raw_readl(hw->regs + USI_CNT);
	__raw_writel(status, hw->regs + USI_CNT);

	if (status & ENFLG) {

		val = __raw_readl(hw->regs + USI_CNT) & BYTEENDIN;

		if(val)
		{
			hw->count = hw->count + (hw->tx_num + 1)*4;						

			if (hw->rx)
			{
				p32tmp = (unsigned int *)((unsigned int )(hw->rx) + count);

				for(i=0;i<(hw->tx_num+1);i++)
				{
					*p32tmp = __raw_readl(hw->regs + USI_RX0 + i*4);
					p32tmp++;  
				}								
				//printk("p32tmp=%x, count=%d, hw->tx_num=%d\n", p32tmp, hw->count, hw->tx_num);
			}

			count = count + (hw->tx_num + 1)*4;			

			if (count < hw->len)
			{
				if((count+32) <= hw->len)
				{
					w55fa92_spi_setup_txnum(hw, 7);
					for(i=0;i<(hw->tx_num+1);i++)
						__raw_writel(hw_txword(hw, (count+i*4)), hw->regs + USI_TX0 + i*4);
				}
				else if((count+24) <= hw->len)
				{
					w55fa92_spi_setup_txnum(hw, 5);
					for(i=0;i<(hw->tx_num+1);i++)
						__raw_writel(hw_txword(hw, (count+i*4)), hw->regs + USI_TX0 + i*4);
				}
				else if((count+16) <= hw->len)
				{
					w55fa92_spi_setup_txnum(hw, 3);
					for(i=0;i<(hw->tx_num+1);i++)
						__raw_writel(hw_txword(hw, (count+i*4)), hw->regs + USI_TX0 + i*4);
				}
				else if((count+8) <= hw->len)
				{
					w55fa92_spi_setup_txnum(hw, 1);
					for(i=0;i<(hw->tx_num+1);i++)
						__raw_writel(hw_txword(hw, (count+i*4)), hw->regs + USI_TX0 + i*4);
				}
				else if((count+4) <= hw->len)
				{
					w55fa92_spi_setup_txnum(hw, 0);
					__raw_writel(hw_txword(hw, count), hw->regs + USI_TX0);
				}
				else
				{
					w55fa92_spi_setup_byte_endin(hw, 0);
					w55fa92_spi_setup_txbitlen(hw, 8);
					w55fa92_spi_setup_txnum(hw, 0);
					__raw_writel(hw_txbyte(hw, count), hw->regs + USI_TX0);
				}
				w55fa92_spi_gobusy(hw);
			}
			else
			{
				complete(&hw->done);
			}
		}
		else
		{		
			hw->count++;

			if (hw->rx)
				hw->rx[count] = __raw_readl(hw->regs + USI_RX0);
			count++;

			if (count < hw->len) {
				__raw_writel(hw_txbyte(hw, count), hw->regs + USI_TX0);
				w55fa92_spi_gobusy(hw);
			} else {
				complete(&hw->done);
			}
		}

		return IRQ_HANDLED;
	}

	complete(&hw->done);
	return IRQ_HANDLED;
}

static void w55fa92_tx_edge(struct w55fa92_spi *hw, unsigned int edge)
{
	unsigned int val;
	unsigned long flags;

	spin_lock_irqsave(&hw->lock, flags);

	val = __raw_readl(hw->regs + USI_CNT);

	if (edge)
		val |= TXNEG;
	else
		val &= ~TXNEG;
	__raw_writel(val, hw->regs + USI_CNT);

	spin_unlock_irqrestore(&hw->lock, flags);
}

static void w55fa92_rx_edge(struct w55fa92_spi *hw, unsigned int edge)
{
	unsigned int val;
	unsigned long flags;

	spin_lock_irqsave(&hw->lock, flags);

	val = __raw_readl(hw->regs + USI_CNT);

	if (edge)
		val |= RXNEG;
	else
		val &= ~RXNEG;
	__raw_writel(val, hw->regs + USI_CNT);

	spin_unlock_irqrestore(&hw->lock, flags);
}

static void w55fa92_send_first(struct w55fa92_spi *hw, unsigned int lsb)
{
	unsigned int val;
	unsigned long flags;

	spin_lock_irqsave(&hw->lock, flags);

	val = __raw_readl(hw->regs + USI_CNT);

	if (lsb)
		val |= LSB;
	else
		val &= ~LSB;
	__raw_writel(val, hw->regs + USI_CNT);

	spin_unlock_irqrestore(&hw->lock, flags);
}

static void w55fa92_set_sleep(struct w55fa92_spi *hw, unsigned int sleep)
{
	unsigned int val;
	unsigned long flags;

	spin_lock_irqsave(&hw->lock, flags);

	val = __raw_readl(hw->regs + USI_CNT);

	if (sleep)
		val |= (sleep << 12);
	else
		val &= ~(0x0f << 12);
	__raw_writel(val, hw->regs + USI_CNT);

	spin_unlock_irqrestore(&hw->lock, flags);
}

static void w55fa92_enable_int(struct w55fa92_spi *hw)
{
	unsigned int val;
	unsigned long flags;

	spin_lock_irqsave(&hw->lock, flags);

	val = __raw_readl(hw->regs + USI_CNT);

	val |= ENINT;

	__raw_writel(val, hw->regs + USI_CNT);

	spin_unlock_irqrestore(&hw->lock, flags);
}

static void w55fa92_set_divider(struct w55fa92_spi *hw)
{
	__raw_writel(hw->pdata->divider, hw->regs + USI_DIV);
}

static void w55fa92_init_spi(struct w55fa92_spi *hw)
{
	clk_enable(hw->clk);

	if(hw->pdata->bus_num == 0)
	{
	       writel(readl(REG_APBIPRST) | SPI0RST, REG_APBIPRST);	//reset spi0
		writel(readl(REG_APBIPRST) & ~SPI0RST, REG_APBIPRST);
	}
	else
	{
		 writel(readl(REG_APBIPRST) | SPI1RST, REG_APBIPRST);	//reset spi1
		writel(readl(REG_APBIPRST) & ~SPI1RST, REG_APBIPRST);
	}

	spin_lock_init(&hw->lock);

	w55fa92_tx_edge(hw, hw->pdata->txneg);
	w55fa92_rx_edge(hw, hw->pdata->rxneg);
	w55fa92_send_first(hw, hw->pdata->lsb);
	w55fa92_set_sleep(hw, hw->pdata->sleep);
	w55fa92_spi_setup_txbitlen(hw, hw->pdata->txbitlen);
	w55fa92_spi_setup_txnum(hw, hw->pdata->txnum);
	if(w55fa92_apb_clock>=45000)
		hw->pdata->divider = 1;
	w55fa92_set_divider(hw);
	w55fa92_enable_int(hw);
}

static int __devinit w55fa92_spi_probe(struct platform_device *pdev)
{
	struct w55fa92_spi *hw;
	struct spi_master *master;
	int err = 0;	

	master = spi_alloc_master(&pdev->dev, sizeof(struct w55fa92_spi));
	if (master == NULL) {
		dev_err(&pdev->dev, "No memory for spi_master\n");
		err = -ENOMEM;
		goto err_nomem;
	}

	hw = spi_master_get_devdata(master);
	memset(hw, 0, sizeof(struct w55fa92_spi));

	hw->master = spi_master_get(master);
	hw->pdata  = pdev->dev.platform_data;
	hw->dev = &pdev->dev;

	if (hw->pdata == NULL) {
		dev_err(&pdev->dev, "No platform data supplied\n");
		err = -ENOENT;
		goto err_pdata;
	}

	platform_set_drvdata(pdev, hw);
	init_completion(&hw->done);

	master->mode_bits          = SPI_MODE_0;
	master->num_chipselect     = hw->pdata->num_cs;
	master->bus_num            = hw->pdata->bus_num;
	hw->bitbang.master         = hw->master;
	hw->bitbang.setup_transfer = w55fa92_spi_setupxfer;
	hw->bitbang.chipselect     = w55fa92_spi_chipsel;
	hw->bitbang.txrx_bufs      = w55fa92_spi_txrx;
	hw->bitbang.master->setup  = w55fa92_spi_setup;

	hw->res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (hw->res == NULL) {
		dev_err(&pdev->dev, "Cannot get IORESOURCE_MEM\n");
		err = -ENOENT;
		goto err_pdata;
	}

	hw->ioarea = request_mem_region(hw->res->start,
					resource_size(hw->res), pdev->name);

	if (hw->ioarea == NULL) {
		dev_err(&pdev->dev, "Cannot reserve region\n");
		err = -ENXIO;
		goto err_pdata;
	}

	hw->regs = ioremap(hw->res->start, resource_size(hw->res));
	if (hw->regs == NULL) {
		dev_err(&pdev->dev, "Cannot map IO\n");
		err = -ENXIO;
		goto err_iomap;
	}

	hw->irq = platform_get_irq(pdev, 0);
	if (hw->irq < 0) {
		dev_err(&pdev->dev, "No IRQ specified\n");
		err = -ENOENT;
		goto err_irq;
	}

	err = request_irq(hw->irq, w55fa92_spi_irq, 0, pdev->name, hw);
	if (err) {
		dev_err(&pdev->dev, "Cannot claim IRQ\n");
		goto err_irq;
	}

	if(hw->pdata->bus_num == 0)
		hw->clk = clk_get(&pdev->dev, "ms0");
	else
		hw->clk = clk_get(&pdev->dev, "ms1");	
	
	if (IS_ERR(hw->clk)) {
		dev_err(&pdev->dev, "No clock for device\n");
		err = PTR_ERR(hw->clk);
		goto err_clk;
	}

	if(hw->pdata->bus_num == 0)
	{
		writel((readl(REG_GPDFUN1) & ~0xFF0F0000) | 0x22020000, REG_GPDFUN1);  // configuer pin function
#ifdef CONFIG_FA92_SPI0_CS0_ENABLE
		writel((readl(REG_GPDFUN1) & ~(0x00F00000)) |0x00200000, REG_GPDFUN1);
#endif
#ifdef CONFIG_FA92_SPI0_CS1_ENABLE
		writel((readl(REG_GPDFUN0) & ~(0x000F0000)) |0x00020000, REG_GPDFUN0);
#endif
	}
	else
	{
		writel((readl(REG_SHRPIN_TOUCH) & ~TP_AEN), REG_SHRPIN_TOUCH);	//enable digital pin
		writel((readl(REG_GPGFUN1) & ~0xFF0F0000) | 0x22020000, REG_GPGFUN1);  // configuer pin function
#ifdef CONFIG_FA92_SPI1_CS0_ENABLE
		writel((readl(REG_GPGFUN1) & ~0x00F00000) | 0x200000, REG_GPGFUN1);
#endif
#ifdef CONFIG_FA92_SPI1_CS1_ENABLE
		writel((readl(REG_GPDFUN0) & ~(0x0000000F)) |0x00000001, REG_GPDFUN0);
#endif
	}

	w55fa92_init_spi(hw);

	err = spi_bitbang_start(&hw->bitbang);
	if (err) {
		dev_err(&pdev->dev, "Failed to register SPI master\n");
		goto err_register;
	}

	return 0;

err_register:	
	clk_disable(hw->clk);
	clk_put(hw->clk);
err_clk:
	free_irq(hw->irq, hw);
err_irq:
	iounmap(hw->regs);
err_iomap:
	release_mem_region(hw->res->start, resource_size(hw->res));
	kfree(hw->ioarea);
err_pdata:
	spi_master_put(hw->master);;

err_nomem:
	return err;
}

static int __devexit w55fa92_spi_remove(struct platform_device *dev)
{
	struct w55fa92_spi *hw = platform_get_drvdata(dev);	

	free_irq(hw->irq, hw);

	platform_set_drvdata(dev, NULL);

	spi_unregister_master(hw->master);
	
	clk_disable(hw->clk);
	clk_put(hw->clk);

	iounmap(hw->regs);

	release_mem_region(hw->res->start, resource_size(hw->res));
	kfree(hw->ioarea);

	spi_master_put(hw->master);
	return 0;
}

static struct platform_driver w55fa92_spi_driver = {
	.probe		= w55fa92_spi_probe,
	.remove		= __devexit_p(w55fa92_spi_remove),
	.driver		= {
		.name	= "w55fa92-spi",
		.owner	= THIS_MODULE,
	},
};

static int __init w55fa92_spi_init(void)
{
	return platform_driver_register(&w55fa92_spi_driver);
}

static void __exit w55fa92_spi_exit(void)
{
	platform_driver_unregister(&w55fa92_spi_driver);
}

module_init(w55fa92_spi_init);
module_exit(w55fa92_spi_exit);

MODULE_DESCRIPTION("w55fa92 spi driver!");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:w55fa92-spi");

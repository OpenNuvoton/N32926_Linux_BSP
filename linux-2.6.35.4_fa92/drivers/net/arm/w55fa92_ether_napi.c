/*
 * Copyright (c) 2013-2015 Nuvoton technology corporation.
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation;version 2 of the License.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/mii.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/ethtool.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/gfp.h>

#include <mach/w55fa92_reg.h>

#define DRV_MODULE_NAME		"w55fa92-emc"
#define DRV_MODULE_VERSION	"1.1"

/* Ethernet MAC Registers */
#define REG_CAMM_BASE		0x08
#define REG_CAML_BASE		0x0c
#if 0
#define REG_CAMCMR		0x00
#define REG_CAMEN		0x04
#define REG_TXDLSA		0x88
#define REG_RXDLSA		0x8C
#define REG_MCMDR		0x90
#define REG_MIID		0x94
#define REG_MIIDA		0x98
#define REG_FFTCR		0x9C
#define REG_TSDR		0xa0
#define REG_RSDR		0xa4
#define REG_DMARFC		0xa8
#define REG_MIEN		0xac
#define REG_MISTA		0xb0
#define REG_CTXDSA		0xcc
#define REG_CTXBSA		0xd0
#define REG_CRXDSA		0xd4
#define REG_CRXBSA		0xd8
#endif

/* mac controller bit */
#define MCMDR_RXON		0x01
#define MCMDR_ACP		(0x01 << 3)
#define MCMDR_SPCRC		(0x01 << 5)
#define MCMDR_AMGP		(0x01 << 6)		// Enable to receive magice packet for WLAN
#define MCMDR_TXON		(0x01 << 8)
#define MCMDR_FDUP		(0x01 << 18)
#define MCMDR_ENMDC		(0x01 << 19)
#define MCMDR_OPMOD		(0x01 << 20)
#define MCMDR_ENRMII	(0x01 << 22)
#define SWR			(0x01 << 24)

/* cam command regiser */
#define CAMCMR_AUP		0x01
#define CAMCMR_AMP		(0x01 << 1)
#define CAMCMR_ABP		(0x01 << 2)
#define CAMCMR_CCAM		(0x01 << 3)
#define CAMCMR_ECMP		(0x01 << 4)
#define CAM0EN			0x01

/* mac mii controller bit */
#define MDCCR			(0x0a << 20)
#define PHYAD			(0x01 << 8)
#define PHYWR			(0x01 << 16)
#define PHYBUSY			(0x01 << 17)
#define PHYPRESP		(0x01 << 18)
#define CAM_ENTRY_SIZE		0x08

/* rx and tx status */
#define TXDS_TXCP		(0x01 << 19)
#define RXDS_CRCE		(0x01 << 17)
#define RXDS_PTLE		(0x01 << 19)
#define RXDS_RXGD		(0x01 << 20)
#define RXDS_ALIE		(0x01 << 21)
#define RXDS_RP			(0x01 << 22)

/* mac interrupt status*/
#define MISTA_EXDEF		(0x01 << 19)
#define MISTA_TXBERR		(0x01 << 24)
#define MISTA_TDU		(0x01 << 23)
#define MISTA_RDU		(0x01 << 10)
#define MISTA_RXBERR		(0x01 << 11)

#define ENSTART			0x01
#define ENRXINTR		0x01
#define ENRXGD			(0x01 << 4)
#define ENRXBERR		(0x01 << 11)
#define ENTXINTR		(0x01 << 16)
#define ENTXCP			(0x01 << 18)
#define ENTXABT			(0x01 << 21)
#define ENTXBERR		(0x01 << 24)
#define ENMDC			(0x01 << 19)
#define PHYBUSY			(0x01 << 17)
#define MDCCR_VAL		0xa00000

/* rx and tx owner bit */
#define RX_OWEN_DMA		(0x01 << 31)
#define RX_OWEN_CPU		(~(0x03 << 30))
#define TX_OWEN_DMA		(0x01 << 31)
#define TX_OWEN_CPU		(~(0x01 << 31))

/* tx frame desc controller bit */
#define MACTXINTEN		0x04
#define CRCMODE			0x02
#define PADDINGMODE		0x01

/* fftcr controller bit */
#define TXTHD 			(0x03 << 8)
#define BLENGTH			(0x01 << 20)

/* global setting for driver */
#define RX_DESC_SIZE		32
#define TX_DESC_SIZE		16
#define MAX_RBUFF_SZ		0x600
#define MAX_TBUFF_SZ		0x600
#define TX_TIMEOUT		50
#define DELAY			1000
#define CAM0			0x0

#define ETH_TRIGGER_RX	do{__raw_writel(ENSTART, REG_RSDR);}while(0)
#define ETH_TRIGGER_TX	do{__raw_writel(ENSTART, REG_TSDR);}while(0)
#define ETH_ENABLE_TX	do{__raw_writel(__raw_readl( REG_MCMDR) | MCMDR_TXON, REG_MCMDR);}while(0)
#define ETH_ENABLE_RX	do{__raw_writel(__raw_readl( REG_MCMDR) | MCMDR_RXON, REG_MCMDR);}while(0)
#define ETH_DISABLE_TX	do{__raw_writel(__raw_readl( REG_MCMDR) & ~MCMDR_TXON, REG_MCMDR);}while(0)
#define ETH_DISABLE_RX	do{__raw_writel(__raw_readl( REG_MCMDR) & ~MCMDR_RXON, REG_MCMDR);}while(0)

static int w55fa92_mdio_read(struct net_device *dev, int phy_id, int reg);
static void w55fa92_mdio_write(struct net_device *dev,
                                        int phy_id, int reg, int data);
//extern void mfp_set_groupf(struct device *dev);

struct w55fa92_rxbd {
	unsigned int sl;
	unsigned int buffer;
	unsigned int reserved;
	unsigned int next;
};

struct w55fa92_txbd {
	unsigned int mode;
	unsigned int buffer;
	unsigned int sl;
	unsigned int next;
};

struct recv_pdesc {
	struct w55fa92_rxbd desclist[RX_DESC_SIZE];
	char recv_buf[RX_DESC_SIZE][MAX_RBUFF_SZ];
};


struct sk_buff *tx_skb[TX_DESC_SIZE];


struct  w55fa92_ether {
	struct recv_pdesc  *rdesc;
	struct w55fa92_txbd *tdesc;
	dma_addr_t rdesc_phys;
	dma_addr_t tdesc_phys;
	struct net_device_stats stats;
	struct platform_device *pdev;
	struct resource *res;
	struct clk *clk;
	struct clk *hclk3;
	struct mii_if_info mii;
	struct timer_list check_timer;
	struct napi_struct napi;
	struct net_device *ndev;
	void __iomem *reg;
	int rxirq;
	int txirq;
	unsigned int cur_tx;
	unsigned int cur_rx;
	unsigned int finish_tx;
	unsigned int start_tx_ptr;
	unsigned int start_rx_ptr;
	unsigned int linkflag;
};

extern void nvt_lock(void);
extern void nvt_unlock(void);

static void update_linkspeed_register(struct net_device *dev,
				unsigned int speed, unsigned int duplex)
{
	unsigned int val;

	val = __raw_readl( REG_MCMDR);

	if (speed == SPEED_100) {
		/* 100 full/half duplex */
		nvt_lock();
		__raw_writel(__raw_readl(REG_CLKDIV7) & (~0x13000000), REG_CLKDIV7); // clean divider of 10Mbps
		__raw_writel(__raw_readl(REG_CLKDIV7) | 0x01000000, REG_CLKDIV7); // ref-clk for TX/RX 100Mbps		
		nvt_unlock();
		if (duplex == DUPLEX_FULL) {
			val |= (MCMDR_OPMOD | MCMDR_FDUP);
		} else {
			val |= MCMDR_OPMOD;
			val &= ~MCMDR_FDUP;
		}
	} else {
		/* 10 full/half duplex */
		nvt_lock();
		__raw_writel(__raw_readl(REG_CLKDIV7) | 0x13000000, REG_CLKDIV7); // ref-clk for TX/RX 10Mbps		
		nvt_unlock();
		if (duplex == DUPLEX_FULL) {
			val |= MCMDR_FDUP;
			val &= ~MCMDR_OPMOD;
		} else {
			val &= ~(MCMDR_FDUP | MCMDR_OPMOD);
		}
	}

	__raw_writel(val,  REG_MCMDR);
}

static void update_linkspeed(struct net_device *dev)
{
	struct w55fa92_ether *ether = netdev_priv(dev);
	struct platform_device *pdev;
	unsigned int bmsr, bmcr, lpa, speed, duplex;

	pdev = ether->pdev;

	if (!mii_link_ok(&ether->mii)) {
		if( ether->linkflag != 0x0 )
			dev_dbg(&pdev->dev, "%s: Link down.\n", dev->name);
		ether->linkflag = 0x0;
		netif_carrier_off(dev);
		return;
	}

	if (ether->linkflag == 1)
		return;

	bmsr = w55fa92_mdio_read(dev, ether->mii.phy_id, MII_BMSR);
	bmcr = w55fa92_mdio_read(dev, ether->mii.phy_id, MII_BMCR);

	if (bmcr & BMCR_ANENABLE) {
		if (!(bmsr & BMSR_ANEGCOMPLETE))
			return;
#ifdef CONFIG_OPT_FPGA  // ## For FPGA stage speed 10M limit
		w55fa92_mdio_write(dev, ether->mii.phy_id, MII_BMCR, BMCR_FULLDPLX);	// SPEED_10 FULL
		//w55fa92_mdio_write(dev, ether->mii.phy_id, MII_BMCR, 0);	// SPEED_10 Half
		//w55fa92_mdio_write(dev, ether->mii.phy_id, MII_BMCR, BMCR_FULLDPLX | BMCR_SPEED100);	// SPEED_100 FULL
		while( !( w55fa92_mdio_read(dev, ether->mii.phy_id, MII_BMSR)&BMSR_LSTATUS) );
#endif

#ifdef CONFIG_W55FA92_ETH_PHY_RTL8201
		w55fa92_mdio_write(dev, ether->mii.phy_id,31,7);
		w55fa92_mdio_write(dev, ether->mii.phy_id,17,0x12);
		if( w55fa92_mdio_read(dev, ether->mii.phy_id,17) != 0x12 ) 
			printk("RTL8201 Setting Fail..");
		w55fa92_mdio_write(dev, ether->mii.phy_id,31,7);
		w55fa92_mdio_write(dev, ether->mii.phy_id,19,0x38);
		if( w55fa92_mdio_read(dev, ether->mii.phy_id,19) != 0x38 ) 
			printk("RTL8201 Setting Fail..");
		w55fa92_mdio_write(dev, ether->mii.phy_id,31,7);
		w55fa92_mdio_write(dev, ether->mii.phy_id,16,0x79A);
		if( w55fa92_mdio_read(dev, ether->mii.phy_id,16) != 0x79A ) 
			printk("RTL8201 Setting Fail..");
#endif

		lpa = w55fa92_mdio_read(dev, ether->mii.phy_id, MII_LPA);

		if ((lpa & LPA_100FULL) || (lpa & LPA_100HALF))
			speed = SPEED_100;
		else
			speed = SPEED_10;

		if ((lpa & LPA_100FULL) || (lpa & LPA_10FULL))
			duplex = DUPLEX_FULL;
		else
			duplex = DUPLEX_HALF;

	} else {
		speed = (bmcr & BMCR_SPEED100) ? SPEED_100 : SPEED_10;
		duplex = (bmcr & BMCR_FULLDPLX) ? DUPLEX_FULL : DUPLEX_HALF;
	}
	printk("Phy speed:%d M\n",speed);
	
	update_linkspeed_register(dev, speed, duplex);

	dev_info(&pdev->dev, "%s: Link now %i-%s\n", dev->name, speed,
			(duplex == DUPLEX_FULL) ? "FullDuplex" : "HalfDuplex");
	ether->linkflag = 0x01;

	netif_carrier_on(dev);
}

static void w55fa92_check_link(unsigned long dev_id)
{
	struct net_device *dev = (struct net_device *) dev_id;
	struct w55fa92_ether *ether = netdev_priv(dev);

	update_linkspeed(dev);
	mod_timer(&ether->check_timer, jiffies + msecs_to_jiffies(1000));
}

static void w55fa92_write_cam(struct net_device *dev,
				unsigned int x, unsigned char *pval)
{
	struct w55fa92_ether *ether = netdev_priv(dev);
	unsigned int msw, lsw;

	msw = (pval[0] << 24) | (pval[1] << 16) | (pval[2] << 8) | pval[3];

	lsw = (pval[4] << 24) | (pval[5] << 16);

	__raw_writel(lsw, ether->reg + REG_CAML_BASE + x * CAM_ENTRY_SIZE);
	__raw_writel(msw, ether->reg + REG_CAMM_BASE + x * CAM_ENTRY_SIZE);
//	__raw_writel(lsw,  REG_CAML_BASE + x * CAM_ENTRY_SIZE);
//	__raw_writel(msw,  REG_CAMM_BASE + x * CAM_ENTRY_SIZE);
}

static int w55fa92_init_desc(struct net_device *dev)
{
	struct w55fa92_ether *ether;
	struct w55fa92_txbd  *tdesc;
	struct w55fa92_rxbd  *rdesc;
	struct platform_device *pdev;
	u32 i;

	ether = netdev_priv(dev);
	pdev = ether->pdev;

	ether->tdesc = (struct w55fa92_txbd *)
			dma_alloc_coherent(&pdev->dev, sizeof(struct w55fa92_txbd) * TX_DESC_SIZE,
						&ether->tdesc_phys, GFP_KERNEL);

	if (!ether->tdesc) {
		dev_err(&pdev->dev, "Failed to allocate memory for tx desc\n");
		return -ENOMEM;
	}

	ether->rdesc = (struct recv_pdesc *)
		dma_alloc_coherent(&pdev->dev, sizeof(struct recv_pdesc),
					&ether->rdesc_phys, GFP_KERNEL);

	if (!ether->rdesc) {
		dev_err(&pdev->dev, "Failed to allocate memory for rx desc\n");
		dma_free_coherent(&pdev->dev, sizeof(struct w55fa92_txbd) * TX_DESC_SIZE,
						ether->tdesc, ether->tdesc_phys);
		return -ENOMEM;
	}

	for (i = 0; i < TX_DESC_SIZE; i++) {
		unsigned int offset;

		tdesc = (ether->tdesc + i);

		if (i == TX_DESC_SIZE - 1)
			offset = 0;
		else
			offset = sizeof(struct w55fa92_txbd) * (i + 1);

		tdesc->next = ether->tdesc_phys + offset;
		tdesc->buffer = (unsigned int)NULL;
		tdesc->sl = 0;
		tdesc->mode = 0;
	}

	ether->start_tx_ptr = ether->tdesc_phys;

	for (i = 0; i < RX_DESC_SIZE; i++) {
		unsigned int offset;

		rdesc = &(ether->rdesc->desclist[i]);

		if (i == RX_DESC_SIZE - 1)
			offset = offsetof(struct recv_pdesc, desclist[0]);
		else
			offset = offsetof(struct recv_pdesc, desclist[i + 1]);

		rdesc->next = ether->rdesc_phys + offset;
		rdesc->sl = RX_OWEN_DMA;
		rdesc->buffer = ether->rdesc_phys +
			offsetof(struct recv_pdesc, recv_buf[i]) + 2;  //==============>
	}


	ether->start_rx_ptr = ether->rdesc_phys;

	return 0;
}

// This API must call with Tx/Rx stopped
static void w55fa92_free_desc(struct net_device *dev)
{
	struct sk_buff *skb;
	u32 i;
	struct w55fa92_ether *ether = netdev_priv(dev);
	struct platform_device *pdev = ether->pdev;

	for (i = 0; i < TX_DESC_SIZE; i++) {
		skb = tx_skb[i];
		if(skb != NULL) {
			dma_unmap_single(&dev->dev, (dma_addr_t)((ether->tdesc + i)->buffer), skb->len, DMA_TO_DEVICE);
			dev_kfree_skb_any(skb);
		}
	}


	dma_free_coherent(&pdev->dev, sizeof(struct w55fa92_txbd) * TX_DESC_SIZE,
				ether->tdesc, ether->tdesc_phys);
	dma_free_coherent(&pdev->dev, sizeof(struct recv_pdesc) * RX_DESC_SIZE,
				ether->rdesc, ether->rdesc_phys);

}

static void w55fa92_set_fifo_threshold(struct net_device *dev)
{
	unsigned int val;

	val = TXTHD | BLENGTH;
	__raw_writel(val,  REG_FFTCR);
}

static void w55fa92_return_default_idle(struct net_device *dev)
{
	unsigned int val;

	val = __raw_readl( REG_MCMDR);
	val |= SWR;
	__raw_writel(val,  REG_MCMDR);
}

static void w55fa92_enable_mac_interrupt(struct net_device *dev)
{
	unsigned int val;

	val = ENTXINTR | ENRXINTR | ENRXGD | ENTXCP;
	val |= ENTXBERR | ENRXBERR | ENTXABT;

	__raw_writel(val,  REG_MIEN);
}

static void w55fa92_get_and_clear_int(struct net_device *dev,
					unsigned int *val, unsigned int mask)
{
	*val = __raw_readl( REG_MISTA) & mask;
	__raw_writel(*val,  REG_MISTA);
}

static void w55fa92_set_global_maccmd(struct net_device *dev)
{
	unsigned int val;

	val = __raw_readl( REG_MCMDR);
	val |= MCMDR_SPCRC | MCMDR_ENMDC | MCMDR_ACP | MCMDR_ENRMII | ENMDC;
	__raw_writel(val,  REG_MCMDR);
}

static void w55fa92_enable_cam(struct net_device *dev)
{
	unsigned int val;

	w55fa92_write_cam(dev, CAM0, dev->dev_addr);

	val = __raw_readl( REG_CAMEN);
	val |= CAM0EN;
	__raw_writel(val,  REG_CAMEN);
}

static void w55fa92_enable_cam_command(struct net_device *dev)
{
	unsigned int val;

	val = CAMCMR_ECMP | CAMCMR_ABP /*| CAMCMR_AMP*/;
	__raw_writel(val,  REG_CAMCMR);
}

static void w55fa92_set_curdest(struct net_device *dev)
{
	struct w55fa92_ether *ether = netdev_priv(dev);

	__raw_writel(ether->start_rx_ptr,  REG_RXDLSA);
	__raw_writel(ether->start_tx_ptr,  REG_TXDLSA);
}

static void w55fa92_reset_mac(struct net_device *dev, int need_free)
{
	struct w55fa92_ether *ether = netdev_priv(dev);

	ETH_DISABLE_TX;
	ETH_DISABLE_RX;

	w55fa92_return_default_idle(dev);
	w55fa92_set_fifo_threshold(dev);

	if (!netif_queue_stopped(dev))
		netif_stop_queue(dev);

	if(need_free)
		w55fa92_free_desc(dev);
	w55fa92_init_desc(dev);

	dev->trans_start = jiffies; /* prevent tx timeout */
	ether->cur_tx = 0x0;
	ether->finish_tx = 0x0;
	ether->cur_rx = 0x0;

	w55fa92_set_curdest(dev);
	w55fa92_enable_cam(dev);
	w55fa92_enable_cam_command(dev);
	w55fa92_enable_mac_interrupt(dev);
	ETH_ENABLE_TX;
	ETH_ENABLE_RX;

	ETH_TRIGGER_RX;

	dev->trans_start = jiffies; /* prevent tx timeout */

	if (netif_queue_stopped(dev))
		netif_wake_queue(dev);
}

static void w55fa92_mdio_write(struct net_device *dev,
					int phy_id, int reg, int data)
{
	struct w55fa92_ether *ether = netdev_priv(dev);
	struct platform_device *pdev;
	unsigned int val, i;

	pdev = ether->pdev;

	__raw_writel(data,  REG_MIID);

	val = (phy_id << 0x08) | reg;
	val |= PHYBUSY | PHYWR | MDCCR_VAL;
	__raw_writel(val,  REG_MIIDA);

	for (i = 0; i < DELAY; i++) {
		if ((__raw_readl( REG_MIIDA) & PHYBUSY) == 0)
			break;
	}

	if (i == DELAY)
		dev_warn(&pdev->dev, "mdio write timed out\n");
}

static int w55fa92_mdio_read(struct net_device *dev, int phy_id, int reg)
{
	struct w55fa92_ether *ether = netdev_priv(dev);
	struct platform_device *pdev;
	unsigned int val, i, data;

	pdev = ether->pdev;

	val = (phy_id << 0x08) | reg;
	val |= PHYBUSY | MDCCR_VAL;
	__raw_writel(val,  REG_MIIDA);

	for (i = 0; i < DELAY; i++) {
		if ((__raw_readl( REG_MIIDA) & PHYBUSY) == 0)
			break;
	}

	if (i == DELAY) {
		dev_warn(&pdev->dev, "mdio read timed out\n");
		data = 0xffff;
	} else {
		data = __raw_readl( REG_MIID);
	}

	return data;
}

static int w55fa92_set_mac_address(struct net_device *dev, void *addr)
{
	struct sockaddr *address = addr;

	if (!is_valid_ether_addr(address->sa_data))
		return -EADDRNOTAVAIL;

	memcpy(dev->dev_addr, address->sa_data, dev->addr_len);
	w55fa92_write_cam(dev, CAM0, dev->dev_addr);

	return 0;
}

static int w55fa92_ether_close(struct net_device *dev)
{
	struct w55fa92_ether *ether = netdev_priv(dev);
	struct platform_device *pdev;

	pdev = ether->pdev;

	netif_stop_queue(dev);
	napi_disable(&ether->napi);
	free_irq(ether->txirq, dev);
	free_irq(ether->rxirq, dev);

	w55fa92_free_desc(dev);

	del_timer_sync(&ether->check_timer);
#if 1
	clk_disable(ether->clk);
	clk_disable(ether->hclk3);
#else
	//__raw_writel(__raw_readl(REG_AHBCLK2) & (~0x80), REG_AHBCLK2); // Disable EMAC CLK
#endif



	return 0;
}

static struct net_device_stats *w55fa92_ether_stats(struct net_device *dev)
{
	struct w55fa92_ether *ether;

	ether = netdev_priv(dev);

	return &ether->stats;
}



static int w55fa92_ether_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct w55fa92_ether *ether = netdev_priv(dev);
	struct w55fa92_txbd *txbd = (ether->tdesc + ether->cur_tx);
	

	txbd->buffer = dma_map_single(&dev->dev, skb->data,
					skb->len, DMA_TO_DEVICE);

	tx_skb[ether->cur_tx]  = skb;
	txbd->sl = skb->len > 1514 ? 1514 : skb->len;
	txbd->mode = TX_OWEN_DMA | PADDINGMODE | CRCMODE | MACTXINTEN;

	ETH_TRIGGER_TX;

	if (++ether->cur_tx >= TX_DESC_SIZE)
		ether->cur_tx = 0;

	txbd = (ether->tdesc + ether->cur_tx);

	if (txbd->mode & TX_OWEN_DMA)
		netif_stop_queue(dev);

	return(0);
}

static irqreturn_t w55fa92_tx_interrupt(int irq, void *dev_id)
{
	struct w55fa92_ether *ether;
	struct w55fa92_txbd  *txbd;
	struct platform_device *pdev;
	struct net_device *dev;
	struct sk_buff *s;
	unsigned int cur_entry, entry, status;

	dev = dev_id;
	ether = netdev_priv(dev);
	pdev = ether->pdev;

	w55fa92_get_and_clear_int(dev, &status, 0xFFFF0000);

	cur_entry = __raw_readl( REG_CTXDSA);

	entry = ether->tdesc_phys + sizeof(struct w55fa92_txbd) * (ether->finish_tx);

	while (entry != cur_entry) {
		txbd = (ether->tdesc + ether->finish_tx);
		s = tx_skb[ether->finish_tx];
		dma_unmap_single(&dev->dev, txbd->buffer, s->len, DMA_TO_DEVICE);
		dev_kfree_skb_irq(s);
		tx_skb[ether->finish_tx] = NULL;

		if (++ether->finish_tx >= TX_DESC_SIZE)
			ether->finish_tx = 0;

		if (txbd->sl & TXDS_TXCP) {
			ether->stats.tx_packets++;
			ether->stats.tx_bytes += txbd->sl;
		} else {
			ether->stats.tx_errors++;
		}

		txbd->sl = 0x0;
		txbd->mode = 0x0;
		txbd->buffer = (unsigned int)NULL;

		netif_wake_queue(dev);

		entry = ether->tdesc_phys + sizeof(struct w55fa92_txbd) * (ether->finish_tx);
	}

	if (status & MISTA_EXDEF) {
		dev_err(&pdev->dev, "emc defer exceed interrupt\n");
	} else if (status & MISTA_TXBERR) {
		dev_err(&pdev->dev, "emc bus error interrupt\n");
		w55fa92_reset_mac(dev, 1);
	} else if (status & MISTA_TDU) {
		netif_wake_queue(dev);
	}

	return IRQ_HANDLED;
}

static int w55fa92_poll(struct napi_struct *napi, int budget)
{
	struct w55fa92_ether *ether = container_of(napi, struct w55fa92_ether, napi);
	struct w55fa92_rxbd *rxbd;
	struct net_device *dev = ether->ndev;
	struct sk_buff *skb;
	unsigned char *data;
	unsigned int length, status, val, entry;
	int rx_cnt = 0;
	
	//printk("enter\n");

	rxbd = &ether->rdesc->desclist[ether->cur_rx];

	while(rx_cnt < budget) {

		val = __raw_readl(REG_CRXDSA);
		entry = ether->rdesc_phys +
			offsetof(struct recv_pdesc, desclist[ether->cur_rx]);

		if (val == entry)
			break;

		rx_cnt++;
		status = rxbd->sl;
		length = status & 0xFFFF;

		if (likely(status & RXDS_RXGD)) {

			data = ether->rdesc->recv_buf[ether->cur_rx] + 2;//====================>
			skb = dev_alloc_skb(length+2);
			if (!skb) {
				ether->stats.rx_dropped++;
				goto rx_out;
			}

			skb_reserve(skb, 2);
			skb_put(skb, length);
			skb_copy_to_linear_data(skb, data, length);
#if 0

	if((skb->data[5] != 0x03) && (skb->data[5] != 0xff)) {
		int i,j;
		printk("***********\n");
		printk("%x %x\n", rxbd->buffer, skb->data);
		printk("*\n");
		printk("===================================================\n");
		for(i = 0; i < 2; i++) {
			for(j = 0; j < 16; j++) {
				printk("%02x ", skb->data[(i * 16) + j]);
			}
			printk("\n");
		}
	}
#endif

			skb->protocol = eth_type_trans(skb, dev);
			ether->stats.rx_packets++;
			ether->stats.rx_bytes += length;
			netif_receive_skb(skb);

		} else {
			ether->stats.rx_errors++;

			if (status & RXDS_RP) {
				ether->stats.rx_length_errors++;
			} else if (status & RXDS_CRCE) {
				ether->stats.rx_crc_errors++;
			} else if (status & RXDS_ALIE) {
				ether->stats.rx_frame_errors++;
			} else if (status & RXDS_PTLE) {
				ether->stats.rx_over_errors++;
			}
		}

		rxbd->sl = RX_OWEN_DMA;
		rxbd->reserved = 0x0;

		if (++ether->cur_rx >= RX_DESC_SIZE)
			ether->cur_rx = 0;

		rxbd = &ether->rdesc->desclist[ether->cur_rx];

	}
	
	if(rx_cnt < budget)
		__napi_complete(napi);

rx_out:

	ETH_TRIGGER_RX;
	__raw_writel(__raw_readl(REG_MIEN) | ENRXINTR,  REG_MIEN);	
	return(rx_cnt);
}

static irqreturn_t w55fa92_rx_interrupt(int irq, void *dev_id)
{
	struct net_device *dev = (struct net_device *)dev_id;
	struct w55fa92_ether *ether = netdev_priv(dev);
	unsigned int status;

	w55fa92_get_and_clear_int(dev, &status, 0xFFFF);

	if (unlikely(status & MISTA_RXBERR)) {
		struct platform_device *pdev = ether->pdev;

		dev_err(&pdev->dev, "emc rx bus error\n");
		w55fa92_reset_mac(dev, 1);

	} else {
		__raw_writel(__raw_readl(REG_MIEN) & ~ENRXINTR,  REG_MIEN);	
		napi_schedule(&ether->napi);
	}
	return IRQ_HANDLED;

}

static int w55fa92_ether_open(struct net_device *dev)
{
	struct w55fa92_ether *ether;
	struct platform_device *pdev;

	ether = netdev_priv(dev);
	pdev = ether->pdev;
#if 1
	clk_enable(ether->hclk3);
	clk_enable(ether->clk);
#else
	//__raw_writel(__raw_readl(REG_AHBCLK2) | 0x80, REG_AHBCLK2); // Enable EMAC CLK	
#endif
	w55fa92_reset_mac(dev, 0);
	w55fa92_set_fifo_threshold(dev);
	w55fa92_set_curdest(dev);
	w55fa92_enable_cam(dev);
	w55fa92_enable_cam_command(dev);
	w55fa92_enable_mac_interrupt(dev);
	w55fa92_set_global_maccmd(dev);
	ETH_ENABLE_RX;


	if (request_irq(ether->txirq, w55fa92_tx_interrupt,
						0x0, pdev->name, dev)) {
		dev_err(&pdev->dev, "register irq tx failed\n");
		return -EAGAIN;
	}

	if (request_irq(ether->rxirq, w55fa92_rx_interrupt,
						0x0, pdev->name, dev)) {
		dev_err(&pdev->dev, "register irq rx failed\n");
		free_irq(ether->txirq, dev);
		return -EAGAIN;
	}

	mod_timer(&ether->check_timer, jiffies + msecs_to_jiffies(1000));
	netif_start_queue(dev);
	napi_enable(&ether->napi);
	
	ETH_TRIGGER_RX;

	dev_info(&pdev->dev, "%s is OPENED\n", dev->name);

	return 0;
}

static void w55fa92_ether_set_multicast_list(struct net_device *dev)
{
	struct w55fa92_ether *ether;
	unsigned int rx_mode;

	ether = netdev_priv(dev);
#if 0
	if (dev->flags & IFF_PROMISC)
		rx_mode = CAMCMR_AUP | CAMCMR_AMP | CAMCMR_ABP | CAMCMR_ECMP;
	else if ((dev->flags & IFF_ALLMULTI) || !netdev_mc_empty(dev))
		rx_mode = CAMCMR_AMP | CAMCMR_ABP | CAMCMR_ECMP;
	else
#endif
		rx_mode = CAMCMR_ECMP | CAMCMR_ABP;

	__raw_writel(rx_mode,  REG_CAMCMR);
}

static int w55fa92_ether_ioctl(struct net_device *dev,
						struct ifreq *ifr, int cmd)
{
	struct w55fa92_ether *ether = netdev_priv(dev);
	struct mii_ioctl_data *data = if_mii(ifr);

	return generic_mii_ioctl(&ether->mii, data, cmd, NULL);
}

static void w55fa92_get_drvinfo(struct net_device *dev,
					struct ethtool_drvinfo *info)
{
	strcpy(info->driver, DRV_MODULE_NAME);
	strcpy(info->version, DRV_MODULE_VERSION);
}

static int w55fa92_get_settings(struct net_device *dev, struct ethtool_cmd *cmd)
{
	struct w55fa92_ether *ether = netdev_priv(dev);
	return mii_ethtool_gset(&ether->mii, cmd);
}

static int w55fa92_set_settings(struct net_device *dev, struct ethtool_cmd *cmd)
{
	struct w55fa92_ether *ether = netdev_priv(dev);
	return mii_ethtool_sset(&ether->mii, cmd);
}

static int w55fa92_nway_reset(struct net_device *dev)
{
	struct w55fa92_ether *ether = netdev_priv(dev);
	return mii_nway_restart(&ether->mii);
}

static u32 w55fa92_get_link(struct net_device *dev)
{
	struct w55fa92_ether *ether = netdev_priv(dev);
	return mii_link_ok(&ether->mii);
}

static const struct ethtool_ops w55fa92_ether_ethtool_ops = {
	.get_settings	= w55fa92_get_settings,
	.set_settings	= w55fa92_set_settings,
	.get_drvinfo	= w55fa92_get_drvinfo,
	.nway_reset	= w55fa92_nway_reset,
	.get_link	= w55fa92_get_link,
};

static const struct net_device_ops w55fa92_ether_netdev_ops = {
	.ndo_open		= w55fa92_ether_open,
	.ndo_stop		= w55fa92_ether_close,
	.ndo_start_xmit		= w55fa92_ether_start_xmit,
	.ndo_get_stats		= w55fa92_ether_stats,
	.ndo_set_multicast_list	= w55fa92_ether_set_multicast_list,
	.ndo_set_mac_address	= w55fa92_set_mac_address,
	.ndo_do_ioctl		= w55fa92_ether_ioctl,
	.ndo_validate_addr	= eth_validate_addr,
	.ndo_change_mtu		= eth_change_mtu,
};

static void __init get_mac_address(struct net_device *dev)
{
	struct w55fa92_ether *ether = netdev_priv(dev);
	struct platform_device *pdev;
	char addr[6];
	
	pdev = ether->pdev;

	addr[0] = 0x00;
	addr[1] = 0x02;
	addr[2] = 0xac;
	addr[3] = 0x55;
	addr[4] = 0x88;
	addr[5] = 0xa8;

	if (is_valid_ether_addr(addr))
		memcpy(dev->dev_addr, &addr[0], 0x06);
	else
		dev_err(&pdev->dev, "invalid mac address\n");
}

static int w55fa92_ether_setup(struct net_device *dev)
{
	struct w55fa92_ether *ether = netdev_priv(dev);

	ether_setup(dev);
	dev->netdev_ops = &w55fa92_ether_netdev_ops;
	dev->ethtool_ops = &w55fa92_ether_ethtool_ops;

	dev->tx_queue_len = 16;
	dev->dma = 0x0;
	dev->watchdog_timeo = TX_TIMEOUT;

	get_mac_address(dev);

	ether->cur_tx = 0x0;
	ether->cur_rx = 0x0;
	ether->finish_tx = 0x0;
	ether->linkflag = 0x0;
#ifdef CONFIG_W55FA92_ETH_PHY_RTL8201
	ether->mii.phy_id = 0x00;
#else
	ether->mii.phy_id = 0x01;
#endif
	ether->mii.phy_id_mask = 0x1f;
	ether->mii.reg_num_mask = 0x1f;
	ether->mii.dev = dev;
	ether->mii.mdio_read = w55fa92_mdio_read;
	ether->mii.mdio_write = w55fa92_mdio_write;

	setup_timer(&ether->check_timer, w55fa92_check_link,
						(unsigned long)dev);

	return 0;
}

static int __devinit w55fa92_ether_probe(struct platform_device *pdev)
{
	struct w55fa92_ether *ether;
	struct net_device *dev;
	int error;

	dev = alloc_etherdev(sizeof(struct w55fa92_ether));
	if (!dev)
		return -ENOMEM;

	ether = netdev_priv(dev);

	ether->res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (ether->res == NULL) {
		dev_err(&pdev->dev, "failed to get I/O memory\n");
		error = -ENXIO;
		goto failed_free;
	}

	if (!request_mem_region(ether->res->start,
				resource_size(ether->res), pdev->name)) {
		dev_err(&pdev->dev, "failed to request I/O memory\n");
		error = -EBUSY;
		goto failed_free;
	}

	ether->reg = ioremap(ether->res->start, resource_size(ether->res));
	if (ether->reg == NULL) {
		dev_err(&pdev->dev, "failed to remap I/O memory\n");
		error = -ENXIO;
		goto failed_free_mem;
	}

	ether->txirq = platform_get_irq(pdev, 0);
	if (ether->txirq < 0) {
		dev_err(&pdev->dev, "failed to get ether tx irq\n");
		error = -ENXIO;
		goto failed_free_io;
	}

	ether->rxirq = platform_get_irq(pdev, 1);
	if (ether->rxirq < 0) {
		dev_err(&pdev->dev, "failed to get ether rx irq\n");
		error = -ENXIO;
		goto failed_free_txirq;
	}

	platform_set_drvdata(pdev, dev);
	ether->ndev = dev;
#if 1	
//	mfp_set_groupf(&pdev->dev);

	ether->clk = clk_get(&pdev->dev, NULL);
	if (IS_ERR(ether->clk)) {
		dev_err(&pdev->dev, "failed to get ether clock\n");
		error = PTR_ERR(ether->clk);
		goto failed_free_rxirq;
	}

	ether->hclk3 = clk_get(NULL, "hclk3");
	if (IS_ERR(ether->hclk3)) {
		dev_err(&pdev->dev, "failed to get ether clock\n");
		error = PTR_ERR(ether->hclk3);
		goto failed_put_clk;
	}
#endif
	ether->pdev = pdev;
	netif_napi_add(dev, &ether->napi, w55fa92_poll, 16);
	w55fa92_ether_setup(dev);

	error = register_netdev(dev);
	if (error != 0) {
		dev_err(&pdev->dev, "Regiter EMC w55fa92 FAILED\n");
		error = -ENODEV;
		goto failed_put_hclk3;
	}

	return 0;
#if 1	
failed_put_hclk3:
	clk_put(ether->hclk3);
failed_put_clk:
	clk_put(ether->clk);
#endif	
failed_free_rxirq:
	free_irq(ether->rxirq, pdev);
	platform_set_drvdata(pdev, NULL);
failed_free_txirq:
	free_irq(ether->txirq, pdev);
failed_free_io:
	iounmap(ether->reg);
failed_free_mem:
	release_mem_region(ether->res->start, resource_size(ether->res));
failed_free:
	free_netdev(dev);
	return error;
}

static int __devexit w55fa92_ether_remove(struct platform_device *pdev)
{
	struct net_device *dev = platform_get_drvdata(pdev);
	struct w55fa92_ether *ether = netdev_priv(dev);

	unregister_netdev(dev);
#if 1
	clk_put(ether->hclk3);
	clk_put(ether->clk);
#endif
	iounmap(ether->reg);
	release_mem_region(ether->res->start, resource_size(ether->res));

	free_irq(ether->txirq, dev);
	free_irq(ether->rxirq, dev);

	del_timer_sync(&ether->check_timer);
	platform_set_drvdata(pdev, NULL);

	free_netdev(dev);
	return 0;
}

static struct platform_driver w55fa92_ether_driver = {
	.probe		= w55fa92_ether_probe,
	.remove		= __devexit_p(w55fa92_ether_remove),
	.driver		= {
		.name	= "w55fa92-emc",
		.owner	= THIS_MODULE,
	},
};


static int __init w55fa92_ether_init(void)
{
	printk("### W55FA92 ether driver v%s has been initialized successfully!\n", DRV_MODULE_VERSION);
#ifdef CONFIG_W55FA92_ETH_PORT0	// LCM Data Bus pins
	*((unsigned volatile int *)(REG_GPCFUN1)) = 0x44444444; 	// GPCFUN1: GPC15 ~GPC8[31:0] 

	*((unsigned volatile int *)(REG_GPEFUN0)) &= ~0xFF; 	// GPEFUN0: GPE1~0[7:0]
	*((unsigned volatile int *)(REG_GPEFUN0)) |= 0x44;

#else // JTAG1 + TVOut + LVSYNC + LVDE pins

	*((unsigned volatile int *)(REG_GPDFUN0)) &= ~0xFFFF; 	// GPDFUN0: MF_GPD0~3[15:0]
	*((unsigned volatile int *)(REG_GPDFUN0)) |= 0xEEEE;
	*((unsigned volatile int *)(REG_GPDFUN1)) &= ~0xFF00; 	// GPDFUN1: MF_GPD11[15:12] MF_GPD10[11:8]
	*((unsigned volatile int *)(REG_GPDFUN1)) |= 0xEE00;	

	*((unsigned volatile int *)(REG_SHRPIN_TVDAC)) &= ~0x80000000;	// Switch to digitial pin
	*((unsigned volatile int *)(REG_GPGFUN0)) &= ~0xFFFF00; 	// GPGFUN0: GPG5~2[23:7]
	*((unsigned volatile int *)(REG_GPGFUN0)) |= 0xEEEE00;
	*((unsigned volatile int *)(REG_GPIOD_PUEN)) &= ~0x0C0F;        //GPIOD_PUEN: disable PUEN GPIOD_PUEN[0~3,10~11]=0
#endif	
	return platform_driver_register(&w55fa92_ether_driver);
}

static void __exit w55fa92_ether_exit(void)
{
	platform_driver_unregister(&w55fa92_ether_driver);
}

module_init(w55fa92_ether_init);
module_exit(w55fa92_ether_exit);

MODULE_DESCRIPTION("w55fa92 MAC driver!");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:w55fa92-emc");


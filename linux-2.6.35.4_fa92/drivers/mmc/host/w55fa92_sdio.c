/*
 *  linux/drivers/mmc/host/w55fa92_sdio.c - Nuvoton W55FA92 SDIO Driver
 *
 *  Copyright (C) 2005 Cougar Creek Computing Devices Ltd, All Rights Reserved
 *
 *  Copyright (C) 2006 Malcolm Noyes
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/blkdev.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/dma-mapping.h>
#include <linux/clk.h>
#include <linux/atmel_pdc.h>
#include <linux/gfp.h>
#include <linux/mmc/host.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <mach/map.h>
#include <mach/regs-clock.h>

#include <mach/w55fa92_reg.h>

// define DATE CODE and show it when running to make maintaining easy.
#define DATE_CODE   "20161207"

//--- Define compile flags depend on Kconfig
#ifdef CONFIG_FA92_SDIO_SDIO0_SDIO1
    #define NVT_SD_SD0
    #define NVT_SD_SD1
#elif defined (CONFIG_FA92_SDIO_SDIO0_ONLY)
    #define NVT_SD_SD0
#elif defined (CONFIG_FA92_SDIO_SDIO1_ONLY)
    #define NVT_SD_SD1
#else
    #error "CONFIG ERROR: You MUST choose one from SDIO0 ONLY, SDIO1 ONLY, or SDIO0+SDIO1 !!"
#endif

#define nvt_sd_debug printk
//#define ENTER()     nvt_sd_debug("[%-20s #%d] Enter ...\n", __FUNCTION__, __LINE__)
//#define LEAVE()     nvt_sd_debug("[%-20s #%d] Leave ...\n", __FUNCTION__, __LINE__)
#define ENTER()
#define LEAVE()

#define DRIVER_NAME "w55fa92-sdio"  // driver name MUST exact same to device name that defined at dev.c

#define FL_SENT_COMMAND (1 << 0)
#define FL_SENT_STOP    (1 << 1)

#define nvt_sd_read(reg)         __raw_readl(reg)
#define nvt_sd_write(reg, val)   __raw_writel((val), (reg))

#define Enable_AIC_IRQ(n)       __raw_writel((1 << (n)), REG_AIC_MECR)
#define Disable_AIC_IRQ(n)      __raw_writel((1 << (n)), REG_AIC_MDCR)

#define MCI_BLKSIZE         512
#define MCI_MAXBLKSIZE      4095
#define MCI_BLKATONCE       255     // SIC support max 255 blocks in one SD CMD
#define MCI_BUFSIZE         (MCI_BLKSIZE * MCI_BLKATONCE)

/* Driver thread command */
#define SD_EVENT_NONE       0x00000000
#define SD_EVENT_CMD_OUT    0x00000001
#define SD_EVENT_RSP_IN     0x00000010
#define SD_EVENT_RSP2_IN    0x00000100
#define SD_EVENT_CLK_KEEP0  0x00001000
#define SD_EVENT_CLK_KEEP1  0x00010000

#ifdef NVT_SD_SD0
    // for SD port 0
    static volatile int sd_event=0, sd_state=0, sd_state_xfer=0, sd_ri_timeout=0, sd_send_cmd=0;
    static DECLARE_WAIT_QUEUE_HEAD(sdio_event_wq);
    static DECLARE_WAIT_QUEUE_HEAD(sdio_wq);
    static DECLARE_WAIT_QUEUE_HEAD(sdio_wq_xfer);
#endif

#ifdef NVT_SD_SD1
    // for SD port 1
    static volatile int sd1_event=0, sd1_state=0, sd1_state_xfer=0, sd1_ri_timeout=0, sd1_send_cmd=0;
    static DECLARE_WAIT_QUEUE_HEAD(sdio1_event_wq);
    static DECLARE_WAIT_QUEUE_HEAD(sdio1_wq);
    static DECLARE_WAIT_QUEUE_HEAD(sdio1_wq_xfer);
#endif

//--- Define semaphore for SDIO controller
static DECLARE_MUTEX(sdio_fmi_sem);
static DECLARE_MUTEX(sdio_dmac_sem);

static void nvt_sdio_set_clock(u32 sd_clock_khz);
static void nvt_sdio_change_driver_strength(int card_no, u32 sd_clock_khz);

static int sdio_init_completed = 0;     // indicate nvt_sdio_init() not yet completed

/*
 * Low level type for this driver
 */
struct nvt_sd_host {
    struct mmc_host *mmc;
    struct mmc_command *cmd;
    struct mmc_request *request;

    void __iomem *sd_base;
    int irq;        // IRQ number of system for SD

    int present;    // 0 for card inserted; 1 for card removed

    struct clk *fmi_clk, *sd_clk, *dmac_clk;

    /*
     * Flag indicating when the command has been sent. This is used to
     * work out whether or not to send the stop
     */
    unsigned int flags;
    /* flag for current port */
    u32 bus_mode;   // MMC_BUS_WIDTH_1 / MMC_BUS_WIDTH_4
    u32 port;       // SDIO port 0 / 1

    /* DMA buffer used for transmitting */
    unsigned int* buffer;
    dma_addr_t physical_address;
    unsigned int total_length;

    /* Latest in the scatterlist that has been enabled for transfer, but not freed */
    int in_use_index;

    /* Latest in the scatterlist that has been enabled for transfer */
    int transfer_index;

    /* Timer for timeouts */
    struct timer_list timer;
};

#ifdef NVT_SD_SD0
    static struct nvt_sd_host *sd_host;
#endif

#ifdef NVT_SD_SD1
    static struct nvt_sd_host *sd1_host;
#endif

#if 0
// add by CJChen1 for debugging.
static void nvt_sdio_show_reg(char *title)
{
    nvt_sd_debug("    ---- %s ---------------------\n", title);
    nvt_sd_debug("    REG_SDIOCR   =0x%08X\n", nvt_sd_read(REG_SDIOCR));
    nvt_sd_debug("    REG_SDIOARG  =0x%08X\n", nvt_sd_read(REG_SDIOARG));
    nvt_sd_debug("    REG_SDIOIER  =0x%08X\n", nvt_sd_read(REG_SDIOIER));
    nvt_sd_debug("    REG_SDIOISR  =0x%08X\n", nvt_sd_read(REG_SDIOISR));
    nvt_sd_debug("    REG_SDIOBLEN =0x%08X\n", nvt_sd_read(REG_SDIOBLEN));
    nvt_sd_debug("    REG_SDIOTMOUT=0x%08X\n", nvt_sd_read(REG_SDIOTMOUT));
    nvt_sd_debug("    REG_SDIORSP0 =0x%08X\n", nvt_sd_read(REG_SDIORSP0));
    nvt_sd_debug("    REG_SDIORSP1 =0x%08X\n", nvt_sd_read(REG_SDIORSP1));
}
#endif


/*
 * Detect the SD card status. Present or Absent?
 */
static int nvt_sdio_card_detect(struct mmc_host *mmc)
{
    struct nvt_sd_host *host = mmc_priv(mmc);
    int ret;

    if(nvt_sd_read(REG_SDIOFMICR) != FMI_SD_EN)
        nvt_sd_write(REG_SDIOFMICR, FMI_SD_EN);

    //--- always return card removed if nvt_sdio_init() not yet complete
    if (sdio_init_completed == 0)
    {
        host->present = 1;  // card removed
        return 0;
    }

#ifdef NVT_SD_SD0
    if (host->port == 0)
    {
  #ifdef CONFIG_SDIO0_CD_GPC14
        host->present = nvt_sd_read(REG_SDIOISR) & SDISR_CD_Card;   // LOW means card inserted
  #elif defined (CONFIG_SDIO0_CD_NO)
        host->present = 0;                                          // card inserted always
  #elif defined (CONFIG_SDIO0_CD_GPC14_HIGH)
        host->present = (nvt_sd_read(REG_SDIOISR) & SDISR_CD_Card) ? 0 : 1; // HIGH means card inserted
  #endif
    }
#endif

#ifdef NVT_SD_SD1
    if (host->port == 1)
    {
  #ifdef CONFIG_SDIO1_CD_GPG4
        host->present = nvt_sd_read(REG_SDIOISR) & SDISR_CD1_Card;
  #elif defined (CONFIG_SDIO1_CD_NO)
        host->present = 0;                                          // card inserted always
  #elif defined (CONFIG_SDIO1_CD_GPG5_SW)
        if (nvt_sd_read(REG_GPIOG_PIN) & BIT5)
            host->present = 0;  // GPG5 high means card inserted.
        else
            host->present = 1;  // GPG5 low means card removed.
  #endif
    }
#endif

    // Return values for the get_cd() callback should be:
    //      0 for a absent card
    //      1 for a present card
    ret = host->present ? 0 : 1;

    //nvt_sd_debug("--> nvt_sdio_card_detect(): SDIO port %d %s.\n", host->port, ret ? "inserted" : "removed");
    return ret;
}


//-------------------------------------------------
// Waiting SD card become to READY status.
//      Check DATA0 pin. High means READY; LOW means BUSY for write operation.
//-------------------------------------------------
static void nvt_sdio_wait_card_ready(struct mmc_host *mmc)
{
    while (!(nvt_sd_read(REG_SDIOISR) & SDISR_SD_DATA0))
    {
        //--- SD card is busy. Keep waiting or exit if SD card removed.
        if (nvt_sdio_card_detect(mmc) == 0)
            break;  // don't wait if SD card removed.
        nvt_sd_write(REG_SDIOCR, nvt_sd_read(REG_SDIOCR) | SDCR_8CLK_OE);   // generate 8 clocks
        while (nvt_sd_read(REG_SDIOCR) & SDCR_8CLK_OE)    // wait for 8 clocks completed.
        {
            schedule();
        }
    }
}


/*
 * Reset the controller and restore most of the state
 */
static void nvt_sdio_reset_host(struct nvt_sd_host *host)
{
    unsigned long flags;

    local_irq_save(flags);
    nvt_sd_write(REG_SDIODMACCSR, nvt_sd_read(REG_SDIODMACCSR) | DMAC_EN | DMAC_SWRST); // enable DMAC for FMI
    nvt_sd_write(REG_SDIOFMICR, FMI_SD_EN);     // Enable SD functionality of FMI
    local_irq_restore(flags);
}


static void nvt_sdio_timeout_timer(unsigned long data)
{
    struct nvt_sd_host *host;

    host = (struct nvt_sd_host *)data;

    if (host->request) {
        dev_err(host->mmc->parent, "Timeout waiting end of packet\n");

        if (host->cmd && host->cmd->data) {
            host->cmd->data->error = -ETIMEDOUT;
        } else {
            if (host->cmd)
                host->cmd->error = -ETIMEDOUT;
            else
                host->request->cmd->error = -ETIMEDOUT;
        }

        nvt_sdio_reset_host(host);
        mmc_request_done(host->mmc, host->request);
    }
}


/*
 * Copy from sg to a dma block - used for transfers
 */
static inline void nvt_sdio_sg_to_dma(struct nvt_sd_host *host, struct mmc_data *data)
{
    unsigned int len, i, size;
    unsigned *dmabuf = host->buffer;

    size = data->blksz * data->blocks;
    len = data->sg_len;

    /*
     * Just loop through all entries. Size might not
     * be the entire list though so make sure that
     * we do not transfer too much.
     */
    for (i = 0; i < len; i++) {
        struct scatterlist *sg;
        int amount;
        unsigned int *sgbuffer;

        sg = &data->sg[i];

        sgbuffer = kmap_atomic(sg_page(sg), KM_BIO_SRC_IRQ) + sg->offset;
        amount = min(size, sg->length);
        size -= amount;

        {
            char *tmpv = (char *)dmabuf;
            memcpy(tmpv, sgbuffer, amount);
            tmpv += amount;
            dmabuf = (unsigned *)tmpv;
        }

        kunmap_atomic(sgbuffer, KM_BIO_SRC_IRQ);
        data->bytes_xfered += amount;

        if (size == 0)
            break;
    }

    /*
     * Check that we didn't get a request to transfer
     * more data than can fit into the SG list.
     */
    BUG_ON(size != 0);
}


/*
 * Handle after a dma read
 */
static void nvt_sdio_post_dma_read(struct nvt_sd_host *host)
{
    struct mmc_command *cmd;
    struct mmc_data *data;
    unsigned int len, i, size;
    unsigned *dmabuf = host->buffer;

    cmd = host->cmd;
    if (!cmd) {
        nvt_sd_debug("no command\n");
        return;
    }

    data = cmd->data;
    if (!data) {
        nvt_sd_debug("no data\n");
        return;
    }

    size = data->blksz * data->blocks;
    len = data->sg_len;

    for (i = 0; i < len; i++) {
        struct scatterlist *sg;
        int amount;
        unsigned int *sgbuffer;

        sg = &data->sg[i];

        sgbuffer = kmap_atomic(sg_page(sg), KM_BIO_SRC_IRQ) + sg->offset;
        amount = min(size, sg->length);
        size -= amount;

        {
            char *tmpv = (char *)dmabuf;
            memcpy(sgbuffer, tmpv, amount);
            tmpv += amount;
            dmabuf = (unsigned *)tmpv;
        }

        flush_kernel_dcache_page(sg_page(sg));
        kunmap_atomic(sgbuffer, KM_BIO_SRC_IRQ);
        data->bytes_xfered += amount;
        if (size == 0)
            break;
    }
}


/*
 * Handle transmitted data
 */
static void nvt_sdio_handle_transmitted(struct nvt_sd_host *host)
{
    //nvt_sd_debug("Handling the transmit\n");
    if (nvt_sd_read(REG_SDIOISR) & SDISR_CRC_IF)
    {
        //nvt_sd_write(REG_SDIOISR, SDISR_CRC_IF);

        host->cmd->error = -EIO;

        if (((nvt_sd_read(REG_SDIOISR) & SDISR_CRC) >> 4) != 0x2)   // bit 010 for Positive CRC STATUS; bit 101 or others for Negative CRC STATUS
            printk("CRC STATUS error detected and set to %d (cmd = %d, retries = %d, SDIOISR=0x%08X)\n",
                          host->cmd->error, host->cmd->opcode, host->cmd->retries, nvt_sd_read(REG_SDIOISR));

        nvt_sd_write(REG_SDIOISR, SDISR_CRC_IF);  // clear CRC interrupt flag
        // When CRC error is occurred, software should reset SDIO engine.
        nvt_sd_write(REG_SDIOCR, nvt_sd_read(REG_SDIOCR) | SDCR_SWRST); // software reset SDIO engine
        while (nvt_sd_read(REG_SDIOCR) & SDCR_SWRST); // waiting for reset completed
    }

    /* check read/busy */
#ifdef NVT_SD_SD0
    if (host->port == 0)
        nvt_sd_write(REG_SDIOCR, nvt_sd_read(REG_SDIOCR) | SDCR_CLK_KEEP);
#endif

#ifdef NVT_SD_SD1
    if (host->port == 1)
        nvt_sd_write(REG_SDIOCR, nvt_sd_read(REG_SDIOCR) | SDCR_CLK_KEEP1);
#endif

    if ((host->port != 0) && (host->port != 1))
        printk("ERROR: Don't support SD port %d to transmitted data !\n", host->port);
}


/*
 * Update bytes tranfered count during a write operation
 */
static void nvt_sdio_update_bytes_xfered(struct nvt_sd_host *host)
{
    struct mmc_data *data;

    /* always deal with the effective request (and not the current cmd) */
    if (host->request->cmd && host->request->cmd->error != 0)
        return;

    if (host->request->data) {
        data = host->request->data;
        if (data->flags & MMC_DATA_WRITE) {
            /* card is in IDLE mode now */
            data->bytes_xfered = data->blksz * data->blocks;
            //nvt_sd_debug("-> bytes_xfered %d, total_length = %d\n",
            //  data->bytes_xfered, host->total_length);
        }
    }
}


/*-----------------------------------------------------------------------------
 * Config SIC register to select SD port.
 *---------------------------------------------------------------------------*/
static int nvt_sdio_select_port(u32 port)
{
#ifdef NVT_SD_SD0
    if (port == 0)
        nvt_sd_write(REG_SDIOCR, (nvt_sd_read(REG_SDIOCR) & (~SDCR_SDPORT)) | SDCR_SDPORT_0);   // SDIO Port 0 is selected
#endif

#ifdef NVT_SD_SD1
    if (port == 1)
        nvt_sd_write(REG_SDIOCR, (nvt_sd_read(REG_SDIOCR) & (~SDCR_SDPORT)) | SDCR_SDPORT_1);   // SDIO Port 1 is selected
#endif

    if ((port != 0) && (port != 1))
    {
        printk("ERROR: Don't support SD port %d !\n", port);
        return -1;
    }

    //--- 2014/2/26, Reset SDIO controller and DMAC to keep clean status for next access.
    // Reset DMAC engine and interrupt satus
    nvt_sd_write(REG_SDIODMACCSR, nvt_sd_read(REG_SDIODMACCSR) | DMAC_SWRST | DMAC_EN);
    while(nvt_sd_read(REG_SDIODMACCSR) & DMAC_SWRST);
    nvt_sd_write(REG_SDIODMACCSR, nvt_sd_read(REG_SDIODMACCSR) | DMAC_EN);
    nvt_sd_write(REG_SDIODMACISR, WEOT_IF | TABORT_IF);     // clear all interrupt flag

    // Reset FMI engine and interrupt status
    nvt_sd_write(REG_SDIOFMICR, FMI_SWRST);
    while(nvt_sd_read(REG_SDIOFMICR) & FMI_SWRST);
    nvt_sd_write(REG_SDIOFMIISR, FMI_DAT_IF);               // clear all interrupt flag

    // Reset SDIO engine and interrupt status
    nvt_sd_write(REG_SDIOFMICR, FMI_SD_EN);
    nvt_sd_write(REG_SDIOCR, nvt_sd_read(REG_SDIOCR) | SDCR_SWRST);
    while(nvt_sd_read(REG_SDIOCR) & SDCR_SWRST);
    nvt_sd_write(REG_SDIOISR, 0xFFFFFFFF);                  // clear all interrupt flag

    return 0;
}


/*
 * Enable the controller
 */
static void nvt_sdio_enable(struct nvt_sd_host *host)
{
#ifdef NVT_SD_SD0
  #if defined (CONFIG_SDIO0_CD_GPC14) || defined (CONFIG_SDIO0_CD_GPC14_HIGH)
    //--- Set GPIO to SDIO card mode
    // SDIO 0 GPIO select: set GPC8~13 to SDIO card mode (SD0_CLK/SD0_CMD/DAT0[0~3]).
    // Set GPC14 to SDIO mode for SDIO port 0 card detection.
    nvt_sd_write(REG_GPCFUN1, (nvt_sd_read(REG_GPCFUN1) & (~0x0FFFFFFF)) | 0x05555555);

    // Set SDIO0 card detect source to GPIO pin.
    nvt_sd_write(REG_SDIOIER, nvt_sd_read(REG_SDIOIER) | SDIER_CDSRC);

  #elif defined (CONFIG_SDIO0_CD_NO)
    //--- Set GPIO to SDIO card mode
    // SDIO 0 GPIO select: set GPC8~13 to SDIO card mode (SD0_CLK/SD0_CMD/DAT0[0~3]).
    // Don't set GPC14 since has no card detect pin in this configuration.
    nvt_sd_write(REG_GPCFUN1, (nvt_sd_read(REG_GPCFUN1) & (~0x00FFFFFF)) | 0x00555555);

    // Set SDIO0 card detect source to NOT DAT3 pin.
    nvt_sd_write(REG_SDIOIER, nvt_sd_read(REG_SDIOIER) | SDIER_CDSRC);
  #endif
#endif

#ifdef NVT_SD_SD1
    // Set GPGx pins to Digital mode for SDIO1.
    nvt_sd_write(REG_SHRPIN_TVDAC, (nvt_sd_read(REG_SHRPIN_TVDAC) & (~SMTVDACAEN)));

  #ifdef CONFIG_SDIO1_CD_GPG4
    // SDIO 1 GPIO select: set GPG2~3, 12~15 to SDIO card mode (SD1_CLK/SD1_CMD/DAT1[0~3]).
    // Set GPG4 to SDIO mode for SDIO port 1 card detection.
    nvt_sd_write(REG_GPGFUN0, (nvt_sd_read(REG_GPGFUN0) & (~0x000FFF00)) | 0x00011100);
    nvt_sd_write(REG_GPGFUN1, (nvt_sd_read(REG_GPGFUN1) & (~0xFFFF0000)) | 0x11110000);

    nvt_sd_write(REG_SHRPIN_TVDAC, nvt_sd_read(REG_SHRPIN_TVDAC) & (~SMTVDACAEN));  // set GPG2~5 to digital mode
    nvt_sd_write(REG_SHRPIN_TOUCH, nvt_sd_read(REG_SHRPIN_TOUCH) & (~TP_AEN));      // set GPG12~15 to digital mode

    // Set SDIO1 card detect source to GPIO pin.
    nvt_sd_write(REG_SDIOIER, nvt_sd_read(REG_SDIOIER) | SDIER_CD1SRC);

  #elif defined (CONFIG_SDIO1_CD_NO)
    // SDIO 1 GPIO select: set GPG2~3, 12~15 to SDIO card mode (SD1_CLK/SD1_CMD/DAT1[0~3]).
    // Don't set GPG4 since has no card detect pin in this configuration.
    nvt_sd_write(REG_GPGFUN0, (nvt_sd_read(REG_GPGFUN0) & (~0x0000FF00)) | 0x00001100);
    nvt_sd_write(REG_GPGFUN1, (nvt_sd_read(REG_GPGFUN1) & (~0xFFFF0000)) | 0x11110000);

    nvt_sd_write(REG_SHRPIN_TVDAC, nvt_sd_read(REG_SHRPIN_TVDAC) & (~SMTVDACAEN));  // set GPG2~5 to digital mode
    nvt_sd_write(REG_SHRPIN_TOUCH, nvt_sd_read(REG_SHRPIN_TOUCH) & (~TP_AEN));      // set GPG12~15 to digital mode

    // Disable Card detect interrupt since we don't use GPG4 as card detect pin now.
    nvt_sd_write(REG_SDIOIER, nvt_sd_read(REG_SDIOIER) & ~SDIER_CD1_IEN);

  #elif defined (CONFIG_SDIO1_CD_GPG5_SW)
    // SDIO 1 GPIO select: set GPG2~3, 12~15 to SDIO card mode (SD1_CLK/SD1_CMD/DAT1[0~3]).
    // Don't set GPG4 since has no card detect pin in this configuration.
    nvt_sd_write(REG_GPGFUN0, (nvt_sd_read(REG_GPGFUN0) & (~0x0000FF00)) | 0x00001100);
    nvt_sd_write(REG_GPGFUN1, (nvt_sd_read(REG_GPGFUN1) & (~0xFFFF0000)) | 0x11110000);

    nvt_sd_write(REG_SHRPIN_TVDAC, nvt_sd_read(REG_SHRPIN_TVDAC) & (~SMTVDACAEN));  // set GPG2~5 to digital mode
    nvt_sd_write(REG_SHRPIN_TOUCH, nvt_sd_read(REG_SHRPIN_TOUCH) & (~TP_AEN));      // set GPG12~15 to digital mode

    // Disable Card detect interrupt since we don't use GPG4 as card detect pin now.
    nvt_sd_write(REG_SDIOIER, nvt_sd_read(REG_SDIOIER) & ~SDIER_CD1_IEN);

    //--- Select GPG5 OUTPUT mode as SDIO port 1 card detection. LOW means card removed, HIGH means inserted.
    // Set GPG5 to GPIO mode for SDIO port 1 card detection.
    nvt_sd_write(REG_GPGFUN0, nvt_sd_read(REG_GPGFUN0) & (~MF_GPG5));  // set GPG5 to GPIO mode
    nvt_sd_write(REG_GPIOG_PUEN, nvt_sd_read(REG_GPIOG_PUEN) | BIT5);  // set GPG5 internal pull high
    nvt_sd_write(REG_GPIOG_DOUT, nvt_sd_read(REG_GPIOG_DOUT) | BIT5);  // set GPG5 to output high
    nvt_sd_write(REG_GPIOG_OMD,  nvt_sd_read(REG_GPIOG_OMD) | BIT5);   // set GPG5 to output mode

    // Set GPG5 as interrupt pin and bind to GPIO1 interrupt
    nvt_sd_write(REG_IRQTGSRC2, BIT21); // clear GPG5 interrupt status
    nvt_sd_write(REG_IRQSRCGPG, (nvt_sd_read(REG_IRQSRCGPG) & (~0x00000C00)) | 0x00000400); // set GPG5 as GPIO1 interrupt trigger source
    nvt_sd_write(REG_IRQENGPG, nvt_sd_read(REG_IRQENGPG) | BIT5 | BIT21);  // set GPG5 trigger by both falling and rising edge
  #endif
#endif

    //--- Initial SD engine
    nvt_sd_write(REG_SDIODMACCSR, nvt_sd_read(REG_SDIODMACCSR) | DMAC_EN);    // enable DMAC for FMI
    nvt_sd_write(REG_SDIOFMICR, FMI_SD_EN);     // enable SD
    nvt_sd_write(REG_SDIOISR, 0xFFFFFFFF);    // write bit 1 to clear all SDISR
#ifdef CONFIG_OPT_FPGA
    nvt_sdio_set_clock(300);      // default SD clock 300KHz
#endif

    //--- Select SD port
    if (nvt_sdio_select_port(host->port) != 0)
        return;

    // SDNWR = 9+1 clock
    nvt_sd_write(REG_SDIOCR, (nvt_sd_read(REG_SDIOCR) & (~SDCR_SDNWR)) | 0x09000000);

    // SDCR_BLKCNT = 1
    nvt_sd_write(REG_SDIOCR, (nvt_sd_read(REG_SDIOCR) & (~SDCR_BLKCNT)) | 0x00010000);
}

/*
 * Disable the controller
 */
static void nvt_sdio_disable(struct nvt_sd_host *host)
{
    nvt_sd_write(REG_SDIOISR, 0xFFFFFFFF);     // write bit 1 to clear all SDISR
    nvt_sd_write(REG_SDIOFMICR, nvt_sd_read(REG_SDIOFMICR) & (~FMI_SD_EN));   // disable SDIO engine
}

/*
 * Send a command
 */
static void nvt_sdio_send_command(struct nvt_sd_host *host, struct mmc_command *cmd)
{
    unsigned int csr;
    unsigned int volatile block_length;
    struct mmc_data *data = cmd->data;
    unsigned int volatile blocks;
    int clock_free_run_status = 0;

    host->cmd = cmd;
#ifdef NVT_SD_SD0
    if (host->port == 0)
    {
        sd_host = host;
        sd_state = 0;
        sd_state_xfer = 0;
    }
#endif

#ifdef NVT_SD_SD1
    if (host->port == 1)
    {
        sd1_host = host;
        sd1_state = 0;
        sd1_state_xfer = 0;
    }
#endif

    if (down_interruptible(&sdio_fmi_sem))   // get sdio_fmi_sem for whole SD command, include data read/write.
        return;

    if(nvt_sd_read(REG_SDIOFMICR) != FMI_SD_EN)
        nvt_sd_write(REG_SDIOFMICR, FMI_SD_EN);

    if (nvt_sdio_select_port(host->port) != 0)
        return;

    //--- prepare initial value for SDCR register
    csr = nvt_sd_read(REG_SDIOCR) & 0xff00c080;   // clear BLK_CNT, CMD_CODE, and all xx_EN fields.

    //--- 2013/7/23, always disable SD clock free run to support SDIO card interrupt mode.
#ifdef NVT_SD_SD0
    if (host->port == 0)
    {
        clock_free_run_status = csr | SDCR_CLK_KEEP;
        csr = csr & (~SDCR_CLK_KEEP);
    }
#endif
#ifdef NVT_SD_SD1
    if (host->port == 1)
    {
        clock_free_run_status = csr | SDCR_CLK_KEEP1;
        csr = csr & (~SDCR_CLK_KEEP1);
    }
#endif

    csr = csr | (cmd->opcode << 8) | SDCR_CO_EN;    // set command code and enable command out
#ifdef NVT_SD_SD0
    if (host->port == 0)
        sd_event |= SD_EVENT_CMD_OUT;
#endif

#ifdef NVT_SD_SD1
    if (host->port == 1)
        sd1_event |= SD_EVENT_CMD_OUT;
#endif

    if (host->bus_mode == MMC_BUS_WIDTH_4)
        csr |= SDCR_DBW;

    if (mmc_resp_type(cmd) != MMC_RSP_NONE) {
        /* if a response is expected then allow maximum response latancy */

        /* set 136 bit response for R2, 48 bit response otherwise */
        if (mmc_resp_type(cmd) == MMC_RSP_R2) {
            csr |= SDCR_R2_EN;
#ifdef NVT_SD_SD0
            if (host->port == 0)
                sd_event |= SD_EVENT_RSP2_IN;
#endif

#ifdef NVT_SD_SD1
            if (host->port == 1)
                sd1_event |= SD_EVENT_RSP2_IN;
#endif
        } else {
            csr |= SDCR_RI_EN;
#ifdef NVT_SD_SD0
            if (host->port == 0)
                sd_event |= SD_EVENT_RSP_IN;
#endif

#ifdef NVT_SD_SD1
            if (host->port == 1)
                sd1_event |= SD_EVENT_RSP_IN;
#endif
        }
        nvt_sd_write(REG_SDIOISR, SDISR_RITO_IF);

#ifdef NVT_SD_SD0
        if (host->port == 0)
            sd_ri_timeout = 0;
#endif

#ifdef NVT_SD_SD1
        if (host->port == 1)
            sd1_ri_timeout = 0;
#endif

        // CONFIG_SD_DISK_MOUNT_DELAY defined in Kconfig.
        // The valid value range between 0x1FFF and 0xFFFFFF.
        // The longer delay makes SD disk mount slower but more stable.
        nvt_sd_write(REG_SDIOTMOUT, CONFIG_SD_DISK_MOUNT_DELAY);    // timeout for CMD
    }

    if (data) {
        nvt_sd_write(REG_SDIOIER, nvt_sd_read(REG_SDIOIER) | SDIER_BLKD_IEN); // Enable SD interrupt & select GPIO detect
        block_length = data->blksz;     // data block size, seem <= 512 for SD CMD
        blocks = data->blocks;          // number of block

        nvt_sd_write(REG_SDIOBLEN, block_length-1);
        if ((block_length > 512) || (blocks >= 256))
            printk("ERROR: SIC don't support read/write 256 blocks in one SD CMD !\n");
        else
            csr = (csr & (~SDCR_BLKCNT)) | (blocks << 16);
    } else {
        nvt_sd_write(REG_SDIOIER, nvt_sd_read(REG_SDIOIER) & (~SDIER_BLKD_IEN)); // Disable SD interrupt & select GPIO detect
        block_length = 0;
        blocks = 0;
    }

    /*
     * Set the arguments and send the command
     */
    if (data) {
        data->bytes_xfered = 0;
        host->transfer_index = 0;
        host->in_use_index = 0;
        if (data->flags & MMC_DATA_READ) {
            /*
             * Handle a read
             */
            host->total_length = 0;
            nvt_sd_write(REG_SDIODMACSAR, host->physical_address);
        } else if (data->flags & MMC_DATA_WRITE) {
            /*
             * Handle a write
             */
            if (down_interruptible(&sdio_dmac_sem))  // get sdio_dmac_sem for data writing.
                return;
            host->total_length = block_length * blocks;
            nvt_sdio_sg_to_dma(host, data);
            //nvt_sd_debug("Transmitting %d bytes\n", host->total_length);
            nvt_sd_write(REG_SDIODMACSAR, host->physical_address);
            csr = csr | SDCR_DO_EN;
        }
    }

    /*
     * Send the command and then enable the PDC - not the other way round as
     * the data sheet says
     */
    nvt_sdio_change_driver_strength(host->port, host->mmc->ios.clock / 1000);
    nvt_sdio_set_clock(host->mmc->ios.clock / 1000);  // set SD clock for working SD port.

    nvt_sd_write(REG_SDIOARG, cmd->arg);
    //nvt_sd_debug("SDIO%d send cmd %d as 0x%08X, arg=0x%08X, blocks=%d, length=%d\n", host->port, cmd->opcode, csr, cmd->arg, blocks, block_length);
    nvt_sd_write(REG_SDIOCR, csr);

#ifdef NVT_SD_SD0
    if (host->port == 0)
    {
        sd_send_cmd = 1;
        wake_up_interruptible(&sdio_event_wq);
        wait_event_interruptible(sdio_wq, (sd_state != 0));
    }
#endif

#ifdef NVT_SD_SD1
    if (host->port == 1)
    {
        sd1_send_cmd = 1;
        wake_up_interruptible(&sdio1_event_wq);
        wait_event_interruptible(sdio1_wq, (sd1_state != 0));
    }
#endif

    if (data) {
        if (data->flags & MMC_DATA_WRITE) {
            // waiting for SD card write completed and become ready.
            nvt_sdio_wait_card_ready(host->mmc);
#if 0
            // SD clock don't free run any more
#ifdef NVT_SD_SD0
            if (host->port == 0)
                nvt_sd_write(REG_SDIOCR, nvt_sd_read(REG_SDIOCR) & ~SDCR_CLK_KEEP);
#endif

#ifdef NVT_SD_SD1
            if (host->port == 1)
                nvt_sd_write(REG_SDIOCR, nvt_sd_read(REG_SDIOCR) & (~SDCR_CLK_KEEP1));
#endif
            if ((host->port != 0) && (host->port != 1))
                printk("ERROR: Don't support SD port %d to stop free run SD clock !\n", host->port);
#endif
            up(&sdio_dmac_sem);  // release sdio_dmac_sem for data writing.
            nvt_sdio_update_bytes_xfered(host);
        }
    }

    //--- 2013/7/23, restore SD clock free run status.
    if (clock_free_run_status)
    {
    #ifdef NVT_SD_SD0
        if (host->port == 0)
            nvt_sd_write(REG_SDIOCR, nvt_sd_read(REG_SDIOCR) | SDCR_CLK_KEEP);
    #endif
    #ifdef NVT_SD_SD1
        if (host->port == 1)
            nvt_sd_write(REG_SDIOCR, nvt_sd_read(REG_SDIOCR) | SDCR_CLK_KEEP1);
    #endif
    }

    up(&sdio_fmi_sem);   // release sdio_fmi_sem for whole SD command, include data readd/write.
    mmc_request_done(host->mmc, host->request);
}


/*
 * Send stop command
 */
static void nvt_sdio_send_stop(struct nvt_sd_host *host, struct mmc_command *cmd)
{
    unsigned int csr;
    unsigned int volatile block_length;
    unsigned int volatile blocks;

    host->cmd = cmd;

#ifdef NVT_SD_SD0
    if (host->port == 0)
    {
        sd_host = host;
        sd_state = 0;
        sd_state_xfer = 0;
    }
#endif

#ifdef NVT_SD_SD1
    if (host->port == 1)
    {
        sd1_host = host;
        sd1_state = 0;
        sd1_state_xfer = 0;
    }
#endif

    if(nvt_sd_read(REG_SDIOFMICR) != FMI_SD_EN)
        nvt_sd_write(REG_SDIOFMICR, FMI_SD_EN);

    if (nvt_sdio_select_port(host->port) != 0)
        return;

    //--- prepare initial value for SDCR register
    csr = nvt_sd_read(REG_SDIOCR) & 0xff00c080;   // clear BLK_CNT, CMD_CODE, and all xx_EN fields.

    csr = csr | (cmd->opcode << 8) | SDCR_CO_EN;   // set command code and enable command out
#ifdef NVT_SD_SD0
    if (host->port == 0)
        sd_event |= SD_EVENT_CMD_OUT;
#endif

#ifdef NVT_SD_SD1
    if (host->port == 1)
        sd1_event |= SD_EVENT_CMD_OUT;
#endif

    if (host->bus_mode == MMC_BUS_WIDTH_4)
        csr |= SDCR_DBW;

    if (mmc_resp_type(cmd) != MMC_RSP_NONE) {
        /* if a response is expected then allow maximum response latancy */

        /* set 136 bit response for R2, 48 bit response otherwise */
        if (mmc_resp_type(cmd) == MMC_RSP_R2) {
            csr |= SDCR_R2_EN;
#ifdef NVT_SD_SD0
            if (host->port == 0)
                sd_event |= SD_EVENT_RSP2_IN;
#endif

#ifdef NVT_SD_SD1
            if (host->port == 1)
                sd1_event |= SD_EVENT_RSP2_IN;
#endif
        } else {
            csr |= SDCR_RI_EN;
#ifdef NVT_SD_SD0
            if (host->port == 0)
                sd_event |= SD_EVENT_RSP_IN;
#endif

#ifdef NVT_SD_SD1
            if (host->port == 1)
                sd1_event |= SD_EVENT_RSP_IN;
#endif
        }
        nvt_sd_write(REG_SDIOISR, SDISR_RITO_IF);

#ifdef NVT_SD_SD0
        if (host->port == 0)
            sd_ri_timeout = 0;
#endif

#ifdef NVT_SD_SD1
        if (host->port == 1)
            sd1_ri_timeout = 0;
#endif

        nvt_sd_write(REG_SDIOTMOUT, CONFIG_SD_DISK_MOUNT_DELAY);    // timeout for STOP CMD
    }

    nvt_sd_write(REG_SDIOIER, nvt_sd_read(REG_SDIOIER) & (~SDIER_BLKD_IEN)); // Disable SD interrupt & select GPIO detect
    block_length = 0;
    blocks = 0;

    /*
     * Set the arguments and send the command
     */
    nvt_sd_write(REG_SDIOARG, cmd->arg);
    nvt_sd_write(REG_SDIOCR, csr);

#ifdef NVT_SD_SD0
    if (host->port == 0)
    {
        sd_send_cmd = 1;
        wake_up_interruptible(&sdio_event_wq);
    }
#endif

#ifdef NVT_SD_SD1
    if (host->port == 1)
    {
        sd1_send_cmd = 1;
        wake_up_interruptible(&sdio1_event_wq);
    }
#endif
    mmc_request_done(host->mmc, host->request);
    // nvt_sd_debug("--> nvt_sdio_send_stop(): SD %d, cmd %d, END !!\n", host->port, cmd->opcode);
}


/*
 * Process the request
 */
static void nvt_sdio_send_request(struct nvt_sd_host *host)
{
    if (!(host->flags & FL_SENT_COMMAND)) {
        host->flags |= FL_SENT_COMMAND;
        nvt_sdio_send_command(host, host->request->cmd);
    } else if ((!(host->flags & FL_SENT_STOP)) && host->request->stop) {
        host->flags |= FL_SENT_STOP;
        nvt_sdio_send_stop(host, host->request->stop);
    } else {
#ifdef NVT_SD_SD0
        if (host->port == 0)
        {
            sd_state = 1;
            wake_up_interruptible(&sdio_wq);
        }
#endif

#ifdef NVT_SD_SD1
        if (host->port == 1)
        {
            sd1_state = 1;
            wake_up_interruptible(&sdio1_wq);
        }
#endif
        del_timer(&host->timer);
    }
}

/*
 * Handle a command that has been completed
 */
static void nvt_sdio_completed_command(struct nvt_sd_host *host, unsigned int status)
{
    struct mmc_command *cmd = host->cmd;
    struct mmc_data *data = cmd->data;
    unsigned int i, j, tmp[5], err;
    unsigned char *ptr;

    err = nvt_sd_read(REG_SDIOISR);

    if ((err & SDISR_RITO_IF) || (cmd->error)) {
        // nvt_sd_debug("got SDISR_RITO_IF, response timeout, SDISR=0x%08X, error=0x%08X\n", err, cmd->error);
        nvt_sd_write(REG_SDIOTMOUT, 0x0);
        nvt_sd_write(REG_SDIOISR, SDISR_RITO_IF);
        cmd->error = -ETIMEDOUT;
        cmd->resp[0] = cmd->resp[1] = cmd->resp[2] = cmd->resp[3] = 0;
    } else {
        if (status & SD_EVENT_RSP_IN) {
            // if not R2
            // nvt_sd_debug("got response, SDISR=0x%08X\n", err);
            cmd->resp[0] = (nvt_sd_read(REG_SDIORSP0) << 8)|(nvt_sd_read(REG_SDIORSP1) & 0xff);
            cmd->resp[1] = cmd->resp[2] = cmd->resp[3] = 0;
        } else if (status & SD_EVENT_RSP2_IN) {
            // if R2
            // nvt_sd_debug("got response R2, SDISR=0x%08X\n", err);
            ptr = (unsigned char *)REG_SDIOFB_0;    // pointer to DMA buffer
            for (i=0, j=0; j<5; i+=4, j++)
                tmp[j] = (*(ptr+i)<<24)|(*(ptr+i+1)<<16)|(*(ptr+i+2)<<8)|(*(ptr+i+3));
            for (i=0; i<4; i++)
                cmd->resp[i] = ((tmp[i] & 0x00ffffff)<<8)|((tmp[i+1] & 0xff000000)>>24);
        }
    }
    //nvt_sd_debug("    Event = 0x%0X, Resp = [0x%08X 0x%08X 0x%08X 0x%08X]\n", status, cmd->resp[0], cmd->resp[1], cmd->resp[2], cmd->resp[3]);

    if (!cmd->error) {
        if ((err & SDISR_CRC_7) == 0) {
            if (!(mmc_resp_type(cmd) & MMC_RSP_CRC)) {
                // some response don't contain CRC-7 info (ex. R3), then software should ignore SDISR_CRC_7 bit.
                cmd->error = 0;
                nvt_sd_write(REG_SDIOISR, SDISR_CRC_IF);
            } else {
                // really CRC-7 error
                cmd->error = -EIO;
                nvt_sd_debug("CRC-7 error detected and set to %d/%d (cmd = %d, retries = %d)\n",
                                cmd->error, data ? data->error : 0,
                                cmd->opcode, cmd->retries);
                nvt_sd_write(REG_SDIOISR, SDISR_CRC_IF);  // clear CRC interrupt flag
                // When CRC error is occurred, software should reset SDIO engine.
                nvt_sd_write(REG_SDIOCR, nvt_sd_read(REG_SDIOCR) | SDCR_SWRST); // software reset SDIO engine
                while (nvt_sd_read(REG_SDIOCR) & SDCR_SWRST); // waiting for reset completed
            }
        } else
            cmd->error = 0;

        if (data) {
            data->bytes_xfered = 0;
            host->transfer_index = 0;
            host->in_use_index = 0;
            if (data->flags & MMC_DATA_READ) {
                if (down_interruptible(&sdio_dmac_sem))  // get sdio_dmac_sem for data reading.
                    return;

                nvt_sd_write(REG_SDIOTMOUT, 0xffffff);    // longer timeout to read more data
                nvt_sd_write(REG_SDIOCR, nvt_sd_read(REG_SDIOCR) | SDCR_DI_EN);
            }

#ifdef NVT_SD_SD0
            if (host->port == 0)
                wait_event_interruptible(sdio_wq_xfer, (sd_state_xfer != 0));
#endif

#ifdef NVT_SD_SD1
            if (host->port == 1)
                wait_event_interruptible(sdio1_wq_xfer, (sd1_state_xfer != 0));
#endif
            if (data->flags & MMC_DATA_READ)
                up(&sdio_dmac_sem);  // release sdio_dmac_sem for data reading.
        }
    }
    nvt_sdio_send_request(host);
}


/*
 * Handle an MMC request
 */
static void nvt_sdio_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
    struct nvt_sd_host *host = mmc_priv(mmc);
    int card_present;

    host->request = mrq;
    host->flags = 0;

    /* more than 1s timeout needed with slow SD cards */
    //mod_timer(&host->timer, jiffies +  msecs_to_jiffies(2000));
    if (down_interruptible(&sdio_fmi_sem))
        return;
    card_present = nvt_sdio_card_detect(mmc);
    up(&sdio_fmi_sem);

    if (!card_present) {
        //nvt_sd_debug("no medium present\n");
        host->request->cmd->error = -ENOMEDIUM;
        mmc_request_done(host->mmc, host->request);
    } else
        nvt_sdio_send_request(host);
}


 /*-----------------------------------------------------------------------------
 * 2013/9/6, change SDIO pins driver strength according to the SD clock.
 *      For SD clock >  25MHz, set SDCLK driver strength to 8mA;
 *      For SD clock <= 25MHz, set SDCLK driver strength to 4mA;
 *---------------------------------------------------------------------------*/
static void nvt_sdio_change_driver_strength(int card_no, u32 sd_clock_khz)
{
    if (sd_clock_khz <= 25000)
    {
        // change driver strength to 4mA (set REG_MISC_DS_GPx bit to 00)
        if (card_no == 0)       // GPC8~GPC13 for SDIO0 pins
        {
            nvt_sd_write(REG_MISC_DS_GPC, nvt_sd_read(REG_MISC_DS_GPC) & (~(0x0FFF0000)));    // clear for 4mA
        }
        // SDIO port 1 use GPGx pin and cannot config driver strength since chip design limitation.
    }
    else
    {
        // change driver strength to 8mA (set REG_MISC_DS_GPx bit to 01)
        if (card_no == 0)       // GPC8~GPC13 for SDIO0 pins
        {
            nvt_sd_write(REG_MISC_DS_GPC, nvt_sd_read(REG_MISC_DS_GPC) & (~(0x0FFF0000)));    // clear for 4mA
            nvt_sd_write(REG_MISC_DS_GPC, nvt_sd_read(REG_MISC_DS_GPC) | 0x05550000);         // set to bits 01 for 8mA
        }
    }
}


extern unsigned int w55fa92_upll_clock;
/*-----------------------------------------------------------------------------
 * 2013/4/30 by CJChen1, To set up the clock for SDIO_CLK
 *      SDIO_CLK = UPLL / ((CLKDIV8[SDIO_N0] + 1) * (CLKDIV8[SDIO_N1] + 1))
 * INPUT: sd_clock_khz: the SDIO clock you wanted with unit KHz.
 *---------------------------------------------------------------------------*/
// there are 3 bits for divider N0, maximum is 8
#define SD_CLK_DIV0_MAX     8
// there are 8 bits for divider N1, maximum is 256
#define SD_CLK_DIV1_MAX     256
u32 current_sdio_clock_khz = 0;

static void nvt_sdio_set_clock(u32 sd_clock_khz)
{
    u32 rate, div0, div1;
    unsigned int upll_clock = w55fa92_upll_clock;
    u32 sd_clk_div0_min;

#ifdef CONFIG_OPT_FPGA
    u32 upll_clock = 27000;     // 27MHz
#endif

    //nvt_sd_debug("Set SDIO clock to %d KHz.\n", sd_clock_khz);

    //--- calculate the rate that 2 divider have to divide
    // upll_clock is the UPLL input clock with unit KHz
    if (sd_clock_khz > upll_clock)
    {
        printk("ERROR: wrong SDIO clock %dKHz setting since it is faster than input clock %dKHz !\n",
            sd_clock_khz, upll_clock);
        return;
    }

    if (sd_clock_khz == 0)
    {
        printk("WARNING: cannot set SDIO clock to 0Hz. Ignore it !\n");
        return;
    }

    rate = upll_clock / sd_clock_khz;
    // choose slower clock if system clock cannot divisible by wanted clock
    if (upll_clock % sd_clock_khz != 0)
        rate++;
    if (rate > (SD_CLK_DIV0_MAX * SD_CLK_DIV1_MAX)) // the maximum divider for SD_CLK is (SD_CLK_DIV0_MAX * SD_CLK_DIV1_MAX)
    {
        printk("ERROR: wrong SDIO clock %dKHz setting since it is slower than input clock %dKHz/%d !\n",
            sd_clock_khz, upll_clock, SD_CLK_DIV0_MAX * SD_CLK_DIV1_MAX);
        return;
    }

    //--- Ignore the request to set SD clock to same frequency in order to improve SD card access performance.
    if (sd_clock_khz == current_sdio_clock_khz)
    {
        //nvt_sd_debug("nvt_sdio_set_clock(): ignore SDIO clock %dKHz setting since it is same frequency.\n", sd_clock_khz);
        return;
    }
    else
        current_sdio_clock_khz = sd_clock_khz;

    //--- choose a suitable value for first divider CLKDIV2[SD_N0]
#ifdef CONFIG_OPT_FPGA
    // div0 always is 1 for FPGA board since FPGA board don't support CLKDIV2[SD_N0]
    div0 = 1;
#else
    // 2014/12/29, the frequency after first divider MUST <= 177MHz because of FA92 hardware limitation.
    sd_clk_div0_min = upll_clock / 177000;
    if (upll_clock % 177000 != 0)
        sd_clk_div0_min++;
    if (sd_clk_div0_min > SD_CLK_DIV0_MAX)
        sd_clk_div0_min = SD_CLK_DIV0_MAX;
    if (sd_clk_div0_min == 0)
        sd_clk_div0_min = 1;

    for (div0 = SD_CLK_DIV0_MAX; div0 >= sd_clk_div0_min; div0--)    // choose the maximum value if can exact division
    {
        if (rate % div0 == 0)
            break;
    }
    if (div0 < sd_clk_div0_min) // cannot exact division
    {
        // keep div0 as small as possible to improve the accuracy of final SDIO clock
        div0 = rate / SD_CLK_DIV1_MAX;
        if (rate % SD_CLK_DIV1_MAX != 0)
            div0++;
        if (div0 < sd_clk_div0_min)
            div0 = sd_clk_div0_min;
    }
#endif

    //--- calculate the second divider CLKDIV2[SD_N1]
    div1 = rate / div0;
    if (rate % div0 != 0)
        div1++;
    div1 &= 0xFF;
    //nvt_sd_debug("UPLL=%dKHz (rate=%d, div0=%d, div1=%d), SDIO clock=%dKHz\n", upll_clock, rate, div0, div1, upll_clock/(div0*div1));

    //--- setup register
    nvt_sd_write(REG_CLKDIV8, (nvt_sd_read(REG_CLKDIV8) & ~SDIO_S) | (0x03 << 3));      // SDIO clock from UPLL
    nvt_sd_write(REG_CLKDIV8, (nvt_sd_read(REG_CLKDIV8) & ~SDIO_N0) | (div0-1) );       // SDIO clock divided by CLKDIV8[SDIO_N0]
    nvt_sd_write(REG_CLKDIV8, (nvt_sd_read(REG_CLKDIV8) & ~SDIO_N1) | ((div1-1) << 5)); // SDIO clock divider by CLKDIV8[SDIO_N1]
    udelay(10);     // delay to wait SD clock become stable.
    return;
}


/*
 * Handle an MMC request
 */
static void nvt_sdio_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
    struct nvt_sd_host *host = mmc_priv(mmc);

    host->bus_mode = ios->bus_width;

    if (down_interruptible(&sdio_fmi_sem))
        return;

    /* maybe switch power to the card */
    switch (ios->power_mode) {
        case MMC_POWER_OFF:
            // Disable the SD function in FMI controller
            nvt_sd_write(REG_SDIOFMICR, nvt_sd_read(REG_SDIOFMICR) & (~FMI_SD_EN));
            //nvt_sd_debug("SDIO power OFF\n");
            break;
        case MMC_POWER_UP:
        case MMC_POWER_ON:
            //if (ios->power_mode == MMC_POWER_UP)
            //    nvt_sd_debug("SDIO power UP.\n");
            //else
            //    nvt_sd_debug("SDIO power ON\n");

            // Enable the SDIO function in FMI controller
            nvt_sd_write(REG_SDIOFMICR, FMI_SD_EN);
            nvt_sdio_change_driver_strength(host->port, ios->clock / 1000);
            if (ios->clock == 0)
                nvt_sdio_set_clock(300);      // default SD clock 300KHz
            else
                nvt_sdio_set_clock(ios->clock / 1000);    // ios->clock unit is Hz

            nvt_sd_write(REG_SDIOCR, nvt_sd_read(REG_SDIOCR) | SDCR_74CLK_OE);
            while (nvt_sd_read(REG_SDIOCR) & SDCR_74CLK_OE);  // waiting for 74 clock completed.
            break;
        default:
            WARN_ON(1);
    }

    if (ios->bus_width == MMC_BUS_WIDTH_4) {
        // nvt_sd_debug("MMC: Setting controller bus width to 4\n");
        nvt_sd_write(REG_SDIOCR, nvt_sd_read(REG_SDIOCR) | SDCR_DBW);
    } else {
        // nvt_sd_debug("MMC: Setting controller bus width to 1\n");
        nvt_sd_write(REG_SDIOCR, nvt_sd_read(REG_SDIOCR) & (~SDCR_DBW));
    }

    up(&sdio_fmi_sem);
}


#ifdef NVT_SD_SD0
/*
 * Handle CO, RI, and R2 event
 */
static int sdio_event_thread(void *unused)
{
    int event = 0;
    int completed = 0;

    daemonize("sdioeventd");

    //-----------------------------------------------------------------------------------
    // 2014/11/24, the SD card could not be mount as a disk if it inserted but
    //  system not yet ready to mount disk. It could happen when system booting.
    //  To fix this problem, we have to postpone the SD card insert event till
    //  the system ready. The process are
    //  1. Don't report and handle any SD event before nvt_sdio_init() completed.
    //  2. Detect SD card status in thread sdio_event_thread() to trigger SD card event
    //     after nvt_sdio_init() completed.
    //-----------------------------------------------------------------------------------

    //--- Waiting for nvt_sdio_init() completed
    while (sdio_init_completed == 0)
        schedule();

    // 2014/9/19, the parameter of msecs_to_jiffies() in SDIO driver MUST larger than
    // same parameter in SD driver. Else, the disk in SDIO port could be mounted before
    // mount bootable disk in SD port, and then reuslt in system hang up when booting.
    // The parameter of msecs_to_jiffies() in SD driver is 500ms, so we set 1000ms here.
    mmc_detect_change(sd_host->mmc, msecs_to_jiffies(1000));     // detect SD card status

    for (;;) {
        wait_event_interruptible(sdio_event_wq, (sd_event != SD_EVENT_NONE) && (sd_send_cmd));

        completed = 0;
        event = sd_event;
        sd_event = SD_EVENT_NONE;
        sd_send_cmd = 0;
        if (event & SD_EVENT_CMD_OUT) {
            while (1) {
                if (!(nvt_sd_read(REG_SDIOCR) & SDCR_CO_EN)) {
                    completed = 1;
                    break;
                }
            }
        }

        if (event & SD_EVENT_RSP_IN) {
            while (1) {
                if (!(nvt_sd_read(REG_SDIOCR) & SDCR_RI_EN)) {
                    completed = 1;
                    break;
                }

                if (nvt_sd_read(REG_SDIOISR) & SDISR_RITO_IF) {
                    nvt_sd_write(REG_SDIOTMOUT, 0x0);
                    nvt_sd_write(REG_SDIOISR, SDISR_RITO_IF);

                    completed = 1;
                    sd_host->cmd->error = -ETIMEDOUT;
                    break;
                }
            }
        }

        if (event & SD_EVENT_RSP2_IN) {
            while (1) {
                if (!(nvt_sd_read(REG_SDIOCR) & SDCR_R2_EN)) {
                    completed = 1;
                    break;
                }
            }
        }

        if (completed) {
            //nvt_sd_debug("Completed command\n");
            nvt_sdio_completed_command(sd_host, event);
        }
    }
    nvt_sd_debug("event quit\n");
    return 0;
}
#endif  // end of NVT_SD_SD0


#ifdef NVT_SD_SD1
/*
 * Handle CO, RI, and R2 event for SD port 1
 */
static int sdio1_event_thread(void *unused)
{
    int event = 0;
    int completed = 0;

    daemonize("sdio1eventd");

    //--- Waiting for nvt_sdio_init() completed
    while (sdio_init_completed == 0)
        schedule();

    mmc_detect_change(sd1_host->mmc, msecs_to_jiffies(1000));    // detect SD card status

    for (;;) {
        wait_event_interruptible(sdio1_event_wq, (sd1_event != SD_EVENT_NONE) && (sd1_send_cmd));

        completed = 0;
        event = sd1_event;
        sd1_event = SD_EVENT_NONE;
        sd1_send_cmd = 0;
        if (event & SD_EVENT_CMD_OUT) {
            while (1) {
                if (!(nvt_sd_read(REG_SDIOCR) & SDCR_CO_EN)) {
                    completed = 1;
                    break;
                }
            }
        }

        if (event & SD_EVENT_RSP_IN) {
            while (1) {
                if (!(nvt_sd_read(REG_SDIOCR) & SDCR_RI_EN)) {
                    completed = 1;
                    break;
                }

                if (nvt_sd_read(REG_SDIOISR) & SDISR_RITO_IF) {
                    nvt_sd_write(REG_SDIOTMOUT, 0x0);
                    nvt_sd_write(REG_SDIOISR, SDISR_RITO_IF);

                    completed = 1;
                    sd1_host->cmd->error = -ETIMEDOUT;
                    break;
                }
            }
        }

        if (event & SD_EVENT_RSP2_IN) {
            while (1) {
                if (!(nvt_sd_read(REG_SDIOCR) & SDCR_R2_EN)) {
                    completed = 1;
                    break;
                }
            }
        }

        if (completed) {
            //nvt_sd_debug("Completed command\n");
            nvt_sdio_completed_command(sd1_host, event);
        }
    }
    nvt_sd_debug("SD1 event quit\n");
    return 0;
}
#endif  // end of NVT_SD_SD1


/*
 * Handle an interrupt
 */


static irqreturn_t nvt_sdio_irq(int irq, void *devid)
{
    struct nvt_sd_host *host = devid;
    unsigned int int_status;
    unsigned int reg_port_select;

    int_status = nvt_sd_read(REG_SDIOISR);
    reg_port_select = (nvt_sd_read(REG_SDIOCR) & SDCR_SDPORT) >> 29;

    //nvt_sd_debug("Check SD %d irq: SDISR status = 0x%08X, port select = %d\n", host->port, int_status, reg_port_select);

#ifdef NVT_SD_SD0
    // SD card port 0 for SDIO interrupt
    if (host->port == 0)
    {
        if (int_status & SDISR_SDIO_IF) {
            //nvt_sd_debug("--> MMC/SDIO: Clear SDIO interrupt flag. DAT1=0x%x\n", int_status & SDISR_SD_DATA1);
            nvt_sd_write(REG_SDIOIER, nvt_sd_read(REG_SDIOIER) & (~SDIER_SDIO_IEN));    // Disable SDIO0 interrupt before clear interrupt flag
            nvt_sd_write(REG_SDIOISR, SDISR_SDIO_IF);    // write 1 to clear interrupt flag
            mmc_signal_sdio_irq(host->mmc);     // clear IRQ flag and wake up IRQ thread
        }
    }
#endif

#ifdef NVT_SD_SD1
    if (host->port == 1)
    {
        if (int_status & SDISR_SDIO1_IF) {
            //nvt_sd_debug("--> MMC/SDIO1: Clear SDIO interrupt flag. DAT1=0x%x\n", int_status & SDISR_SD_DATA1);
            nvt_sd_write(REG_SDIOIER, nvt_sd_read(REG_SDIOIER) & (~SDIER_SDIO1_IEN));    // Disable SDIO1 interrupt before clear interrupt flag
            nvt_sd_write(REG_SDIOISR, SDISR_SDIO1_IF);  // write 1 to clear interrupt flag
            mmc_signal_sdio_irq(host->mmc);     // clear IRQ flag and wake up IRQ thread
        }
    }
#endif

    if (int_status & SDISR_BLKD_IF) {
        //nvt_sd_debug("Block transfer has ended\n");
        //nvt_sd_debug("SDIO %d irq: port select = %d\n", host->port, reg_port_select);
        if (host->port != reg_port_select)
        {
            // This shared interrupt is not come from this device host. Ignore it.
            //nvt_sd_debug("SDIO %d irq: return IRQ_NONE.\n", host->port);
            return IRQ_NONE;    // interrupt was not from this device
        }

        if ((host->cmd == 0) || (host->cmd->data == 0))
        {
            nvt_sd_debug("SDIO %d irq: port select = %d, found NULL pointer!!\n", host->port, reg_port_select);
            return IRQ_NONE;
        }

        if (host->cmd->data->flags & MMC_DATA_WRITE) {
            nvt_sdio_handle_transmitted(host);
        } else if (host->cmd->data->flags & MMC_DATA_READ) {
            nvt_sdio_post_dma_read(host);

            //-- - check CRC-16 error for data-in transfer
            if (int_status & SDISR_CRC_IF)
            {
                // 2014/5/16, according to comment in mmc_sd_init_card() in core/sd.c, it said
                //      "This CRC enable is located AFTER the reading of the
	            //       card registers because some SDHC cards are not able
	            //       to provide valid CRCs for non-512-byte blocks."
	            // The ACMD51 will read 8 bytes SCR register and could trigger invalid CRC-16 error.
	            // The CMD6 will read 64 bytes SSR register and could trigger invalid CRC-16 error.
	            // So, SD driver ignore CRC-16 error check for command code 51 and 6 here.
                if ((host->cmd->opcode != 51) && (host->cmd->opcode != 6))
                {
                    host->cmd->error = -EIO;
                    if ((nvt_sd_read(REG_SDIOISR) & SDISR_CRC_16) == 0)
                        printk("READ CRC-16 error detected and set to %d (cmd = %d, retries = %d, SDIOISR=0x%08X)\n",
                                      host->cmd->error, host->cmd->opcode, host->cmd->retries, int_status);
                    nvt_sd_write(REG_SDIOISR, SDISR_CRC_IF);  // clear CRC interrupt flag
                    // When CRC error is occurred, software should reset SDIO engine.
                    nvt_sd_write(REG_SDIOCR, nvt_sd_read(REG_SDIOCR) | SDCR_SWRST); // software reset SDIO engine
                    while (nvt_sd_read(REG_SDIOCR) & SDCR_SWRST); // waiting for reset completed
                }
            }
        }
        nvt_sd_write(REG_SDIOISR, SDISR_BLKD_IF);    // write 1 to clear interrupt flag

#ifdef NVT_SD_SD0
        if (host->port == 0)
        {
            sd_state_xfer = 1;
            wake_up_interruptible(&sdio_wq_xfer);
        }
#endif

#ifdef NVT_SD_SD1
        if (host->port == 1)
        {
            sd1_state_xfer = 1;
            wake_up_interruptible(&sdio1_wq_xfer);
        }
#endif
    }

    /*
     * we expect this irq on both insert and remove,
     * and use a short delay to debounce.
     */

#ifdef NVT_SD_SD0
    /* SD card port 0 detect */
    if (host->port == 0)
    {
        if (int_status & SDISR_CD_IF) {
            //nvt_sd_debug("--> nvt_sdio_irq() GPC14 interrupt happened !\n");
      #ifdef CONFIG_SDIO0_CD_GPC14
            host->present = int_status & SDISR_CD_Card;             // LOW means card inserted
      #elif defined (CONFIG_SDIO0_CD_NO)
            host->present = 0;                                      // card inserted always
      #elif defined (CONFIG_SDIO0_CD_GPC14_HIGH)
            host->present = (int_status & SDISR_CD_Card) ? 0 : 1;   // HIGH means card inserted
      #endif

            /* 0.5s needed because of early card detect switch firing */
            mmc_detect_change(host->mmc, msecs_to_jiffies(1000));
            nvt_sd_write(REG_SDIOISR, SDISR_CD_IF);  // write 1 to clear interrupt flag
        }
    }
#endif

#ifdef NVT_SD_SD1
    if (host->port == 1)
    {
        if (int_status & SDISR_CD1_IF) {
      #ifdef CONFIG_SDIO1_CD_GPG4
            //nvt_sd_debug("--> nvt_sdio_irq() GPG4 interrupt happened !\n");
            host->present = int_status & SDISR_CD1_Card;
      #elif defined (CONFIG_SDIO1_CD_NO)
            host->present = 0;                                      // card inserted always
      #endif

            /* 0.5s needed because of early card detect switch firing */
            mmc_detect_change(host->mmc, msecs_to_jiffies(1000));
            nvt_sd_write(REG_SDIOISR, SDISR_CD1_IF);  // write 1 to clear interrupt flag
        }
    }
#endif

    return IRQ_HANDLED;
}


#if (defined(NVT_SD_SD1) && defined(CONFIG_SDIO1_CD_GPG5_SW))
//-------------------------------------------------
// Handle the GPIO1 interrupt for SDIO1 card detection.
//      SDIO1 card detect interrupt bind to GPIO1 interrupt.
//-------------------------------------------------
static irqreturn_t nvt_sdio1_card_detect_irq(int irq, void *devid)
{
    struct nvt_sd_host *host = devid;
    u32 src;

#ifdef CONFIG_SDIO1_CD_GPG5_SW
    src = nvt_sd_read(REG_IRQTGSRC2);
    if (src & BIT21)    // This interrupt is trigger by GPG5
    {
        //nvt_sd_debug("--> nvt_sdio1_card_detect_irq() GPG5 interrupt happened !\n");
        if (nvt_sd_read(REG_GPIOG_DOUT) & BIT5)
            host->present = 0;  // card inserted
        else
            host->present = 1;  // card removed
        /* 0.5s needed because of early card detect switch firing */
        mmc_detect_change(host->mmc, msecs_to_jiffies(1000));

        nvt_sd_write(REG_IRQTGSRC2, BIT21); // clear GPG5 interrupt status
        return IRQ_HANDLED;
    }
    else
        return IRQ_NONE;    // interrupt was not from this device
#else
    return IRQ_NONE;    // interrupt was not from this device
#endif
}
#endif  // end of NVT_SD_SD1


/*
 * Handle an MMC request
 */
//-------------------------------------------------
// TODO: check write protect pin.
// if write protect, it should return >0 value.
//-------------------------------------------------
static int nvt_sdio_get_ro(struct mmc_host *mmc)
{
    return 0;   // no write protect

    /*
     * Board doesn't support read only detection; let the mmc core
     * decide what to do.
     */
    //return -ENOSYS;
}


/*
 * Handle an MMC request
 */
static void nvt_sdio_enable_sdio_irq(struct mmc_host *mmc, int enable)
{
    struct nvt_sd_host *host = mmc_priv(mmc);

#ifdef NVT_SD_SD0
    if (host->port == 0)
    {
        // disable SDIO interrupt first, and then clear interrupt flag
        nvt_sd_write(REG_SDIOIER, nvt_sd_read(REG_SDIOIER) & (~SDIER_SDIO_IEN));    // Disable SDIO interrupt
        nvt_sd_write(REG_SDIOISR, SDISR_SDIO_IF);    // write 1 to clear SDIO interrupt flag
        if (enable)
        {
            // enable SDIO interrupt finally
            nvt_sd_write(REG_SDIOIER, nvt_sd_read(REG_SDIOIER) | SDIER_WKUP_EN);    // Enable Wake-Up interrupt
            nvt_sd_write(REG_SDIOIER, nvt_sd_read(REG_SDIOIER) | SDIER_SDIO_IEN);   // Enable SDIO interrupt
            //nvt_sd_debug("--> MMC/SDIO: Enable SDIO interrupt for SDIO0.\n");

            // MUST enable SD clock free run to latch SDIO interrupt signel.
            nvt_sd_write(REG_SDIOCR, nvt_sd_read(REG_SDIOCR) | SDCR_CLK_KEEP);
        }
        else
        {
            nvt_sd_write(REG_SDIOIER, nvt_sd_read(REG_SDIOIER) & (~SDIER_WKUP_EN));     // Disable Wake-Up interrupt
            //nvt_sd_debug("--> MMC/SDIO: Disable SDIO interrupt for SDIO0.\n");

            nvt_sd_write(REG_SDIOCR, nvt_sd_read(REG_SDIOCR) & (~SDCR_CLK_KEEP));
        }
    }
#endif

#ifdef NVT_SD_SD1
    if (host->port == 1)
    {
        // disable SDIO interrupt first, and then clear interrupt flag
        nvt_sd_write(REG_SDIOIER, nvt_sd_read(REG_SDIOIER) & (~SDIER_SDIO1_IEN));    // Disable SDIO interrupt
        nvt_sd_write(REG_SDIOISR, SDISR_SDIO1_IF);  // write 1 to clear SDIO interrupt flag
        if (enable)
        {
            // enable SDIO interrupt finally
            nvt_sd_write(REG_SDIOIER, nvt_sd_read(REG_SDIOIER) | SDIER_WKUP_EN);    // Enable Wake-Up interrupt
            nvt_sd_write(REG_SDIOIER, nvt_sd_read(REG_SDIOIER) | SDIER_SDIO1_IEN);  // Enable SDIO interrupt
            //nvt_sd_debug("--> MMC/SDIO: Enable SDIO interrupt for SDIO1.\n");

            // MUST enable SD clock free run to latch SDIO interrupt signel.
            nvt_sd_write(REG_SDIOCR, nvt_sd_read(REG_SDIOCR) | SDCR_CLK_KEEP1);
        }
        else
        {
            nvt_sd_write(REG_SDIOIER, nvt_sd_read(REG_SDIOIER) & (~SDIER_WKUP_EN));     // Disable Wake-Up interrupt
            //nvt_sd_debug("--> MMC/SDIO: Disable SDIO interrupt for SDIO1.\n");

            nvt_sd_write(REG_SDIOCR, nvt_sd_read(REG_SDIOCR) & (~SDCR_CLK_KEEP1));
        }
    }
#endif
}


/*
 * Handle an MMC request
 */
static int nvt_sdio_get_cd(struct mmc_host *mmc)
{
    int ret;

    if (down_interruptible(&sdio_fmi_sem))
        return -ENOSYS;
    ret = nvt_sdio_card_detect(mmc);
    up(&sdio_fmi_sem);
    return ret;
}

static const struct mmc_host_ops nvt_sdio_ops = {
    .request    = nvt_sdio_request,
    .set_ios    = nvt_sdio_set_ios,
    .get_ro     = nvt_sdio_get_ro,
    .get_cd     = nvt_sdio_get_cd,
    .enable_sdio_irq = nvt_sdio_enable_sdio_irq,
};

/*
 * Probe for the device
 */
static int __init nvt_sdio_probe(struct platform_device *pdev)
{
    struct resource *res;
    int ret;

#ifdef NVT_SD_SD0
    struct mmc_host *mmc;
    struct nvt_sd_host *host;
#endif

#ifdef NVT_SD_SD1
    struct mmc_host *mmc1;
    struct nvt_sd_host *host1;
#endif

    res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    if (!res)
        return -ENXIO;

    if (!request_mem_region(res->start, res->end - res->start + 1, DRIVER_NAME))
        return -EBUSY;

#ifdef NVT_SD_SD0
    mmc = mmc_alloc_host(sizeof(struct nvt_sd_host), &pdev->dev);
    if (!mmc) {
        ret = -ENOMEM;
        dev_dbg(&pdev->dev, "couldn't allocate mmc host\n");
        goto fail6;
    }

    mmc->ops = &nvt_sdio_ops;
    mmc->f_min = 300000;

  #if defined (CONFIG_W55FA92_SDHC_24MHZ)
    mmc->f_max = 24000000;
  #elif defined (CONFIG_W55FA92_SDHC_48MHZ)
    mmc->f_max = 48000000;
  #else
    mmc->f_max = 24000000;
  #endif

    //--- 2014/11/21, use mmc->unused to indicate physical SD/SDIO port number.
    //    Value 0xAB: A=1 means SD port; A=2 means SDIO port; B=port number.
    //    sdio_io_rw_ext_helper() will use it to identify CMD53 comes from SD port or SDIO port.
    mmc->unused = 0x20;

    mmc->ocr_avail = MMC_VDD_27_28|MMC_VDD_28_29|MMC_VDD_29_30|MMC_VDD_30_31|MMC_VDD_31_32|MMC_VDD_32_33 | MMC_VDD_33_34;
    mmc->caps = 0;

    mmc->max_blk_size  = MCI_MAXBLKSIZE;    // maximum size of one mmc block
    mmc->max_blk_count = MCI_BLKATONCE;     // maximum number of blocks in one req
    mmc->max_req_size  = MCI_BUFSIZE;       // maximum number of bytes in one req
    mmc->max_phys_segs = MCI_BLKATONCE;
    mmc->max_hw_segs   = MCI_BLKATONCE;
    mmc->max_seg_size  = MCI_BUFSIZE;

    host = mmc_priv(mmc);
    sd_host = host;
    host->mmc = mmc;
    host->bus_mode = MMC_BUS_WIDTH_1;
    host->port = 0;     // default SD port to check

    mmc->caps |= (MMC_CAP_4_BIT_DATA | MMC_CAP_SD_HIGHSPEED);

    host->buffer = dma_alloc_coherent(&pdev->dev, MCI_BUFSIZE, &host->physical_address, GFP_KERNEL);
    if (!host->buffer) {
        ret = -ENOMEM;
        dev_err(&pdev->dev, "Can't allocate transmit buffer\n");
        goto fail5;
    }

    /*
     * Get Clock
     */
    host->fmi_clk = clk_get(NULL, "sic");
    if (IS_ERR(host->fmi_clk)) {
        ret = -ENODEV;
        dev_err(&pdev->dev, "Get clock fail. No SIC clock for SDIO !\n");
        goto fail2;
    }

    host->sd_clk = clk_get(NULL, "sdio");
    if (IS_ERR(host->sd_clk)) {
        ret = -ENODEV;
        dev_err(&pdev->dev, "Get clock fail. No SDIO clock for SDIO !\n");
        goto fail2;
    }

    /*
     * Reset hardware
     */
    clk_enable(host->fmi_clk);
    clk_enable(host->sd_clk);
    nvt_sdio_disable(host);
    nvt_sdio_enable(host);

    /*
     * Allocate the MCI interrupt
     */
    host->irq = platform_get_irq(pdev, 0);
    ret = request_irq(host->irq, nvt_sdio_irq, IRQF_SHARED, mmc_hostname(mmc), host);
    if (ret) {
        dev_dbg(&pdev->dev, "request MCI interrupt failed\n");
        goto fail0;
    }
    Enable_AIC_IRQ(host->irq);

    /* add a thread to check CO, RI, and R2 */
    kernel_thread(sdio_event_thread, NULL, 0);

    setup_timer(&host->timer, nvt_sdio_timeout_timer, (unsigned long)host);

    platform_set_drvdata(pdev, mmc);

    nvt_sdio_card_detect(mmc);

    /*
     * Add host to MMC layer
     */
    ret = mmc_add_host(mmc);
    if (ret != 0)
    {
        printk("mmc_add_host() fail. Return = 0x%x\n", ret);
        goto fail0;
    }
#endif


#ifdef NVT_SD_SD1
    //------ for SD1
    mmc1 = mmc_alloc_host(sizeof(struct nvt_sd_host), &pdev->dev);
    if (!mmc1) {
        ret = -ENOMEM;
        dev_dbg(&pdev->dev, "couldn't allocate mmc1 host\n");
        goto fail16;
    }

    mmc1->ops = &nvt_sdio_ops;
    mmc1->f_min = 300000;

  #if defined (CONFIG_W55FA92_SDHC_24MHZ)
    mmc1->f_max = 24000000;
  #elif defined (CONFIG_W55FA92_SDHC_48MHZ)
    // FA92 SDIO1 has a limitation that the SD clock up to only 25MHz because of analog/digital IO pad.
    mmc1->f_max = 24000000;
  #else
    mmc1->f_max = 24000000;
  #endif

    mmc1->unused = 0x21;

    mmc1->ocr_avail = MMC_VDD_27_28|MMC_VDD_28_29|MMC_VDD_29_30|MMC_VDD_30_31|MMC_VDD_31_32|MMC_VDD_32_33 | MMC_VDD_33_34;
    mmc1->caps = 0;

    mmc1->max_blk_size  = MCI_MAXBLKSIZE;    // maximum size of one mmc block
    mmc1->max_blk_count = MCI_BLKATONCE;     // maximum number of blocks in one req
    mmc1->max_req_size  = MCI_BUFSIZE;       // maximum number of bytes in one req
    mmc1->max_phys_segs = MCI_BLKATONCE;
    mmc1->max_hw_segs   = MCI_BLKATONCE;
    mmc1->max_seg_size  = MCI_BUFSIZE;

    host1 = mmc_priv(mmc1);
    sd1_host = host1;
    host1->mmc = mmc1;
    host1->bus_mode = MMC_BUS_WIDTH_1;
    host1->port = 1;

    mmc1->caps |= (MMC_CAP_4_BIT_DATA);  // FA92 SDIO1 don't support High Speed mode.

    host1->buffer = dma_alloc_coherent(&pdev->dev, MCI_BUFSIZE, &host1->physical_address, GFP_KERNEL);
    if (!host1->buffer) {
        ret = -ENOMEM;
        dev_err(&pdev->dev, "Can't allocate transmit buffer 1\n");
        goto fail15;
    }

#ifndef NVT_SD_SD0
    // Only get clock here for SDIO1 only config.
    /*
     * Get Clock
     */
    host1->fmi_clk = clk_get(NULL, "sic");
    if (IS_ERR(host1->fmi_clk)) {
        ret = -ENODEV;
        dev_err(&pdev->dev, "Get clock fail. No SIC clock for SDIO !\n");
        goto fail12;
    }

    host1->sd_clk = clk_get(NULL, "sdio");
    if (IS_ERR(host1->sd_clk)) {
        ret = -ENODEV;
        dev_err(&pdev->dev, "Get clock fail. No SDIO clock for SDIO !\n");
        goto fail12;
    }

    clk_enable(host1->fmi_clk);
    clk_enable(host1->sd_clk);
#endif

    /*
     * Reset hardware
     */
    nvt_sdio_disable(host1);
    nvt_sdio_enable(host1);

    /*
     * Allocate the MCI interrupt
     */
    // interrupt handler for SD1 BLKD_IF and SDIO0_IF
    host1->irq = platform_get_irq(pdev, 0);
    ret = request_irq(host1->irq, nvt_sdio_irq, IRQF_SHARED, mmc_hostname(mmc1), host1);
    if (ret) {
        dev_dbg(&pdev->dev, "request MCI interrupt failed\n");
        goto fail10;
    }
    Enable_AIC_IRQ(host1->irq);

  #if defined (CONFIG_SDIO1_CD_GPG5_SW)
    // Allocate the GPIO1 interrupt handler. SDIO1 card detect GPG5 bind to GPIO1 interrupt.
    ret = request_irq(IRQ_GPIO1, nvt_sdio1_card_detect_irq, IRQF_SHARED, mmc_hostname(mmc1), host1);
    if (ret) {
        dev_dbg(&pdev->dev, "Error: request GPIO1 interrupt failed!\n");
        goto fail10;
    }
  #endif

    /* add a thread to check CO, RI, and R2 */
    kernel_thread(sdio1_event_thread, NULL, 0);

    setup_timer(&host1->timer, nvt_sdio_timeout_timer, (unsigned long)host1);

    platform_set_drvdata(pdev, mmc1);

    nvt_sdio_card_detect(mmc1);

    /*
     * Add host to MMC layer
     */
    ret = mmc_add_host(mmc1);
    if (ret != 0)
    {
        printk("mmc_add_host(mmc1) fail. Return = 0x%x\n", ret);
        goto fail10;
    }
#endif  // end of NVT_SD_SD1

    nvt_sd_debug("W55FA92 MMC/SDIO driver (%s) has been initialized successfully!\n", DATE_CODE);

#if defined (CONFIG_W55FA92_SDHC_24MHZ)
    nvt_sd_debug("SDHC card will run under 24MHz clock on SDIO port.\n");
#elif defined (CONFIG_W55FA92_SDHC_48MHZ)
    nvt_sd_debug("SDHC card will run under 48MHz clock on SDIO port 0 and 24MHz on SDIO port 1.\n");
#else
    nvt_sd_debug("SDHC card will run under 24MHz clock on SDIO port.\n");
#endif
    return 0;

#ifdef NVT_SD_SD0
fail0:
    clk_disable(host->sd_clk);
    clk_put(host->sd_clk);
fail2:
    if (host->buffer)
        dma_free_coherent(&pdev->dev, MCI_BUFSIZE, host->buffer, host->physical_address);
fail5:
    mmc_free_host(mmc);
fail6:
    release_mem_region(res->start, res->end - res->start + 1);
    dev_err(&pdev->dev, "probe failed, err %d\n", ret);
    return ret;
#endif

#ifdef NVT_SD_SD1
fail10:
#ifndef NVT_SD_SD0
    // Only disable clock here for SDIO1 only config.
    clk_disable(host1->sd_clk);
    clk_put(host1->sd_clk);
fail12:
#endif
    if (host1->buffer)
        dma_free_coherent(&pdev->dev, MCI_BUFSIZE, host1->buffer, host1->physical_address);
fail15:
    mmc_free_host(mmc1);
fail16:
    release_mem_region(res->start, res->end - res->start + 1);
    dev_err(&pdev->dev, "probe 1 failed, err %d\n", ret);
    return ret;
#endif  // end of NVT_SD_SD1
}

/*
 * Remove a device
 */
static int __exit nvt_sdio_remove(struct platform_device *pdev)
{
    struct mmc_host *mmc = platform_get_drvdata(pdev);
    struct nvt_sd_host *host;

    if (!mmc)
        return -1;

    host = mmc_priv(mmc);
    if (host->buffer)
        dma_free_coherent(&pdev->dev, MCI_BUFSIZE, host->buffer, host->physical_address);

    nvt_sdio_disable(host);
    del_timer_sync(&host->timer);
    mmc_remove_host(mmc);
    free_irq(host->irq, host);

    clk_disable(host->sd_clk);
    clk_put(host->sd_clk);

    mmc_free_host(mmc);
    platform_set_drvdata(pdev, NULL);
    nvt_sd_debug("MMC/SDIO device Removed!\n");

    return 0;
}


#ifdef CONFIG_PM

static int nvt_sdio_suspend(struct platform_device *pdev, pm_message_t state)
{
    struct mmc_host *mmc = platform_get_drvdata(pdev);
    struct nvt_sd_host *host = mmc_priv(mmc);
    int ret = 0;

    if (mmc)
        ret = mmc_suspend_host(mmc);

    return ret;
}

static int nvt_sdio_resume(struct platform_device *pdev)
{
    struct mmc_host *mmc = platform_get_drvdata(pdev);
    struct nvt_sd_host *host = mmc_priv(mmc);
    int ret = 0;

    if (mmc)
        ret = mmc_resume_host(mmc);

    return ret;
}

#else

#define nvt_sdio_suspend   NULL
#define nvt_sdio_resume    NULL

#endif  // end of CONFIG_PM

static struct platform_driver nvt_sdio_driver = {
    .remove     = __exit_p(nvt_sdio_remove),
    .suspend    = nvt_sdio_suspend,
    .resume     = nvt_sdio_resume,
    .driver     = {
        .name   = DRIVER_NAME,
        .owner  = THIS_MODULE,
    },
};

static int __init nvt_sdio_init(void)
{
    int ret;

    ret = platform_driver_probe(&nvt_sdio_driver, nvt_sdio_probe);     // for non-hotplug device
#ifdef NVT_SD_SD0
    nvt_sd_write(REG_SDIOIER, nvt_sd_read(REG_SDIOIER) | SDIER_CD_IEN);     // enable Interrupt for card detect
#endif

#ifdef NVT_SD_SD1
  #ifdef CONFIG_SDIO1_CD_GPG4
    nvt_sd_write(REG_SDIOIER, nvt_sd_read(REG_SDIOIER) | SDIER_CD1_IEN);    // enable Interrupt for card detect
  #endif
#endif

    sdio_init_completed = 1;  // nvt_sdio_init() completed
    return ret;
}

static void __exit nvt_sdio_exit(void)
{
    platform_driver_unregister(&nvt_sdio_driver);
}

module_init(nvt_sdio_init);
module_exit(nvt_sdio_exit);

MODULE_DESCRIPTION("W55FA92 SDIO Card Interface driver");
MODULE_AUTHOR("HPChen / CJChen");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:w55fa92_sdio");

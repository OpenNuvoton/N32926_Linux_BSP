/* linux/driver/scsi/nuvoton_nand/w55fa92_NandDrv.c
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
 *   2008/08/19     jcao add this file for nuvoton all nand driver.
 *   2011/03/04     C.J.Chen, to support NAND ROM chip "Infinite IM90A001GT"
 *                  Problem: This chip use different command to read redundancy data
 *                     and result in chip response "busy" always for
 *                     fmiSM_Read_RA_512() and fmiSM2BufferM_RA().
 *                  Solution: Since ROM chip don't need to check ECC, so we response "ready"
 *                     for ROM chip always.
 *   2011/03/22     C.J.Chen, to support NAND ROM chip "Infinite IM90A001GT" on both MTD and RS.
 *                  Problem: fmiNormalCheckBlock() and fmiCheckInvalidBlock() will check the
 *                      redundancy data that fmiSM2BufferM_RA() read back.
 *                      At 2011/03/04 code, we just return 0 but don't prepare the data for them to check.
 *                  Solution: Force to assign 0xFF to read back data for ROM chip
 *                      within fmiNormalCheckBlock() and fmiCheckInvalidBlock().
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/highmem.h>
#include <linux/blkdev.h>
#include <linux/string.h>
#include <linux/slab.h>

#include <mach/w55fa92_reg.h>
#include <mach/gnand/GNAND.h>
#include <mach/gnand/GNAND_global.h>
#include <mach/NandDrv.h>
#include <mach/w55fa92_nand.h>

#define FALSE   0
#define TRUE    1

/* Return value of bad block checking */
#define NAND_GOOD_BLOCK     (0)
#define NAND_BAD_BLOCK      (1)

/* Return value of dirty page checking */
#define NAND_CLEAN_PAGE     (0)
#define NAND_DIRTY_PAGE     (1)

static u8 _fmi_pNANDBuffer2[1024*8] __attribute__((aligned (32)));

#undef  outl
#undef  inl
#define outl    writel
#define inl     readl

#define OPT_FIRST_4BLOCKS_ECC4
#define OPT_MARK_BAD_BLOCK_WHILE_ERASE_FAIL
#define OPT_SUPPORT_H27UAG8T2A

u8 volatile _fmi_bIsNandFirstAccess = 1;
extern char volatile _fmi_bIsSMDataReady;
static int _nand_init0 = 0, _nand_init1 = 0;
extern char volatile _fmi_bIsSMPRegionDetect;

//---2013/11/29, write "check marker" to spare area if write_nandloader is TRUE
static u8 write_nandloader = 0;

extern u32 _fmi_ucNANDBuffer;
extern u32 _fmi_pNANDBuffer;
extern u8 *_fmi_gptr1;


#if 0
#define DEBUG
#define SD_DEBUG
#define DUMP
#define SD_DEBUG_ENABLE_ENTER_LEAVE
#define SD_DEBUG_ENABLE_MSG
#define SD_DEBUG_ENABLE_MSG2
#define SD_DEBUG_PRINT_LINE
#endif

#ifdef SD_DEBUG
#define PDEBUG(fmt, arg...)     printk(fmt, ##arg)
#else
#define PDEBUG(fmt, arg...)
#endif

#ifdef SD_DEBUG_PRINT_LINE
#define PRN_LINE()              PDEBUG("[%-20s] : %d\n", __FUNCTION__, __LINE__)
#else
#define PRN_LINE()
#endif

#ifdef SD_DEBUG_ENABLE_ENTER_LEAVE
#define ENTER()                 PDEBUG("[%-20s] : Enter...\n", __FUNCTION__)
#define LEAVE()                 PDEBUG("[%-20s] : Leave...\n", __FUNCTION__)
#else
#define ENTER()
#define LEAVE()
#endif

#ifdef SD_DEBUG_ENABLE_MSG
#define MSG(msg)                PDEBUG("[%-20s] : %s\n", __FUNCTION__, msg)
#else
#define MSG(msg)
#endif

#ifdef SD_DEBUG_ENABLE_MSG2
#define MSG2(fmt, arg...)       PDEBUG("[%-20s] : "fmt, __FUNCTION__, ##arg)
#define PRNBUF(buf, count)      {int i;MSG2("CID Data: ");for(i=0;i<count;i++)\
                                 PDEBUG("%02x ", buf[i]);PDEBUG("\n");}
#else
#define MSG2(fmt, arg...)
#define PRNBUF(buf, count)
#endif

extern struct semaphore fmi_sem;
extern struct semaphore dmac_sem;
/* function pointer */
FMI_SM_INFO_T *pSM0 = NULL, *pSM1 = NULL;
#define NAND_RESERVED_BLOCK     10

/*-----------------------------------------------------------------------------
 * Define some constants for BCH
 *---------------------------------------------------------------------------*/
// define the total padding bytes for 512/1024 data segment
#define BCH_PADDING_LEN_512     32
#define BCH_PADDING_LEN_1024    64
// define the BCH parity code lenght for 512 bytes data pattern
#define BCH_PARITY_LEN_T4  8
#define BCH_PARITY_LEN_T8  15
#define BCH_PARITY_LEN_T12 23
#define BCH_PARITY_LEN_T15 29
// define the BCH parity code lenght for 1024 bytes data pattern
#define BCH_PARITY_LEN_T24 45

/*-----------------------------------------------------------------------------
 * Declaration of  local function prototype.
 *---------------------------------------------------------------------------*/
static void sicSMselect(int chipSel);
void fmiSM_Initial(FMI_SM_INFO_T *pSM);

#ifdef OPT_SUPPORT_H27UAG8T2A
// 2011/12/13, support Hynix H27UAG8T2A that with larger redundancy area size 224.
//      fmiSM_Initial() will set Redundancy Area size to 224 for 4K page NAND if ra_224_4k_page is TRUE.
//      Else, the default Redundancy Area size for other 4K page NAND is 216.
int ra_224_4k_page = 0;
#endif

#if defined(CONFIG_W55FA92_NAND_BOTH) || defined(CONFIG_W55FA92_NAND1)
    extern int nand_card_status(void);
#endif

#define OPT_NAND_ENABLED_PROTECT

/*---------------------------------- functions ----------------------------------*/

/*-----------------------------------------------------------------------------
 * Set BCH according to the input parametere inIBR.
 *      The BCH of IBR area is different from others.
 * INPUT:
 *      pSM: pointer to data stucture of CS0 or CS1
 *      inIBR: TRUE  to use BCH rule in IBR area
 *             FALSE to sue BCH rule in others
 * OUTPUT:
 *      None
 * RETURN:
 *      None.
 * NOTE:
 *      Set registers that depend on page size. According to the sepc, the correct order is
 *      1. SMCR_BCH_TSEL  : to support T24, MUST set SMCR_BCH_TSEL before SMCR_PSIZE.
 *      2. SMCR_PSIZE     : set SMCR_PSIZE will auto change SMRE_REA128_EXT to default value.
 *      3. SMRE_REA128_EXT: to use non-default value, MUST set SMRE_REA128_EXT after SMCR_PSIZE.
 *---------------------------------------------------------------------------*/
void sicSMsetBCH(FMI_SM_INFO_T *pSM, int inIBR)
{
    volatile UINT32 u32PowerOn, powerOnPageSize, powerOnEcc;
    volatile UINT32 bch, oob;

    u32PowerOn = inpw(REG_CHIPCFG);
    // CHIPCFG[9:8]  : 0=2KB,   1=4KB,   2=8KB,   3=Ignore power-on setting
    powerOnPageSize = (u32PowerOn & (NPAGE)) >> 8;
    // CHIPCFG[11,1] : 0=BCH12, 1=BCH15, 2=BCH24, 3=Ignore BCH
    powerOnEcc = ((u32PowerOn & (BIT11)) >> 10) | ((u32PowerOn & (BIT1)) >> 1);

    if ((powerOnPageSize == 3)||(powerOnEcc == 3))  // Ignore power-on setting
    {
        //--- Without Power-on setting. Set BCH by rule of SIC/NAND dirver.
        if (inIBR)
        {
            //DBG_PRINTF("sicSMsetBCH() set BCH to IBR rule\n");
            // page size 512B: BCH T4, Spare area size 16 bytes
            // page size 2KB:  BCH T4, Spare area size 64 bytes
            // page size 4KB:  BCH T8, Spare area size 128 bytes
            // page size 8K:   BCH T12, Spare area size 376 bytes
            if (pSM->nPageSize == NAND_PAGE_512B)
            {
                outpw(REG_SMCSR, inpw(REG_SMCSR) &  ~SMCR_BCH_TSEL);
                outpw(REG_SMCSR, inpw(REG_SMCSR) | BCH_T4);
                outpw(REG_SMCSR, (inpw(REG_SMCSR)&(~SMCR_PSIZE)) | (PSIZE_512));
                outpw(REG_SMREAREA_CTL, (inpw(REG_SMREAREA_CTL) & ~SMRE_REA128_EXT) | 16);
            }
            else if (pSM->nPageSize == NAND_PAGE_2KB)
            {
                outpw(REG_SMCSR, inpw(REG_SMCSR) &  ~SMCR_BCH_TSEL);
                outpw(REG_SMCSR, inpw(REG_SMCSR) | BCH_T4);
                outpw(REG_SMCSR, (inpw(REG_SMCSR)&(~SMCR_PSIZE)) | (PSIZE_2K));
                outpw(REG_SMREAREA_CTL, (inpw(REG_SMREAREA_CTL) & ~SMRE_REA128_EXT) | 64);
            }
            else if (pSM->nPageSize == NAND_PAGE_4KB)
            {
                outpw(REG_SMCSR, inpw(REG_SMCSR) &  ~SMCR_BCH_TSEL);
                outpw(REG_SMCSR, inpw(REG_SMCSR) | BCH_T8);
                outpw(REG_SMCSR, (inpw(REG_SMCSR)&(~SMCR_PSIZE)) | (PSIZE_4K));
                outpw(REG_SMREAREA_CTL, (inpw(REG_SMREAREA_CTL) & ~SMRE_REA128_EXT) | 128);
            }
            else if (pSM->nPageSize == NAND_PAGE_8KB)
            {
                outpw(REG_SMCSR, inpw(REG_SMCSR) &  ~SMCR_BCH_TSEL);
                outpw(REG_SMCSR, inpw(REG_SMCSR) | BCH_T12);
                outpw(REG_SMCSR, (inpw(REG_SMCSR)&(~SMCR_PSIZE)) | (PSIZE_8K));
                outpw(REG_SMREAREA_CTL, (inpw(REG_SMREAREA_CTL) & ~SMRE_REA128_EXT) | 376);
            }
        }
        else    // not in IBR area
        {
            //DBG_PRINTF("sicSMsetBCH() set BCH to Normal rule\n");
            // page size 512B: BCH T4, Spare area size 16 bytes
            // page size 2KB:  BCH T8, Spare area size 64 bytes
            // page size 4KB:  BCH T8, Spare area size 128 bytes or
            //                 BCH T12, Spare area size 216 bytes (224 for special case) or
            //                 BCH T24, Spare area size 216 bytes
            // page size 8K:   BCH T24, Spare area size 376 bytes
            if (pSM->nPageSize == NAND_PAGE_512B)
            {
                outpw(REG_SMCSR, inpw(REG_SMCSR) &  ~SMCR_BCH_TSEL);
                outpw(REG_SMCSR, inpw(REG_SMCSR) | BCH_T4);
                outpw(REG_SMCSR, (inpw(REG_SMCSR)&(~SMCR_PSIZE)) | (PSIZE_512));
                outpw(REG_SMREAREA_CTL, (inpw(REG_SMREAREA_CTL) & ~SMRE_REA128_EXT) | 16);
            }
            else if (pSM->nPageSize == NAND_PAGE_2KB)
            {
                outpw(REG_SMCSR, inpw(REG_SMCSR) &  ~SMCR_BCH_TSEL);
                outpw(REG_SMCSR, inpw(REG_SMCSR) | BCH_T8);
                outpw(REG_SMCSR, (inpw(REG_SMCSR)&(~SMCR_PSIZE)) | (PSIZE_2K));
                outpw(REG_SMREAREA_CTL, (inpw(REG_SMREAREA_CTL) & ~SMRE_REA128_EXT) | 64);
            }
            else if (pSM->nPageSize == NAND_PAGE_4KB)
            {
                if (pSM->bIsNandECC8 == TRUE)
                {
                    outpw(REG_SMCSR, inpw(REG_SMCSR) &  ~SMCR_BCH_TSEL);
                    outpw(REG_SMCSR, inpw(REG_SMCSR) | BCH_T8);
                    outpw(REG_SMCSR, (inpw(REG_SMCSR)&(~SMCR_PSIZE)) | (PSIZE_4K));
                    outpw(REG_SMREAREA_CTL, (inpw(REG_SMREAREA_CTL) & ~SMRE_REA128_EXT) | 128);
                }
                else if (pSM->bIsNandECC12 == TRUE)
                {
                    outpw(REG_SMCSR, inpw(REG_SMCSR) &  ~SMCR_BCH_TSEL);
                    outpw(REG_SMCSR, inpw(REG_SMCSR) | BCH_T12);
                    outpw(REG_SMCSR, (inpw(REG_SMCSR)&(~SMCR_PSIZE)) | (PSIZE_4K));
                    if (pSM->bIsRA224)  // for Hynix H27UAG8T2A
                        outpw(REG_SMREAREA_CTL, (inpw(REG_SMREAREA_CTL) & ~SMRE_REA128_EXT) | 224);  // Redundant area size
                    else
                        outpw(REG_SMREAREA_CTL, (inpw(REG_SMREAREA_CTL) & ~SMRE_REA128_EXT) | 216);  // Redundant area size
                }
                else if (pSM->bIsNandECC24 == TRUE) // for Micron MT29F16G08CBACA and MT29F32G08CBACA
                {
                    outpw(REG_SMCSR, inpw(REG_SMCSR) &  ~SMCR_BCH_TSEL);
                    outpw(REG_SMCSR, inpw(REG_SMCSR) | BCH_T24);
                    outpw(REG_SMCSR, (inpw(REG_SMCSR)&(~SMCR_PSIZE)) | (PSIZE_4K));
                    outpw(REG_SMREAREA_CTL, (inpw(REG_SMREAREA_CTL) & ~SMRE_REA128_EXT) | 216);
                }
            }
            else if (pSM->nPageSize == NAND_PAGE_8KB)
            {
                outpw(REG_SMCSR, inpw(REG_SMCSR) &  ~SMCR_BCH_TSEL);
                outpw(REG_SMCSR, inpw(REG_SMCSR) | BCH_T24);
                outpw(REG_SMCSR, (inpw(REG_SMCSR)&(~SMCR_PSIZE)) | (PSIZE_8K));
                outpw(REG_SMREAREA_CTL, (inpw(REG_SMREAREA_CTL) & ~SMRE_REA128_EXT) | 376);
            }
        }
    }

    //--- With Power-on setting. Set BCH by Power-on setting rule of IBR for ALL blocks.
    // CHIPCFG[9:8]  : 0=2KB,   1=4KB,   2=8KB,   3=Ignore power-on setting
    // CHIPCFG[11,1] : 0=BCH12, 1=BCH15, 2=BCH24, 3=Ignore BCH
    // BCH T12 : 2K+100 / 4K+192 / 8K+376
    // BCH T15 : 2K+124 / 4K+240 / 8K+472
    // BCH T24 : 2K+100 / 4K+188 / 8K+368  ** IBR cannot support 2K+BCH T24 since it use 2K+98.
    else if (powerOnPageSize == 0)  // 2KB
    {
        outpw(REG_SMCSR, inpw(REG_SMCSR) & ~SMCR_BCH_TSEL);
        if (powerOnEcc == 0)        // BCH T12
        {
            bch = BCH_T12;
            oob = 100;
        }
        else if (powerOnEcc == 1)   // BCH T15
        {
            bch = BCH_T15;
            oob = 124;
        }
        else if (powerOnEcc == 2)   // BCH T24
        {
            bch = BCH_T24;
            oob = 100;
        }
        outpw(REG_SMCSR, inpw(REG_SMCSR) | bch);
        outpw(REG_SMCSR, (inpw(REG_SMCSR)&(~SMCR_PSIZE)) | (PSIZE_2K));
        outpw(REG_SMREAREA_CTL, (inpw(REG_SMREAREA_CTL) & ~SMRE_REA128_EXT) | oob);
    }
    else if (powerOnPageSize == 1)  // 4KB
    {
        outpw(REG_SMCSR, inpw(REG_SMCSR) & ~SMCR_BCH_TSEL);
        if (powerOnEcc == 0)        // BCH T12
        {
            bch = BCH_T12;
            oob = 192;
        }
        else if (powerOnEcc == 1)   // BCH T15
        {
            bch = BCH_T15;
            oob = 240;
        }
        else if (powerOnEcc == 2)   // BCH T24
        {
            bch = BCH_T24;
            oob = 188;
        }
        outpw(REG_SMCSR, inpw(REG_SMCSR) | bch);
        outpw(REG_SMCSR, (inpw(REG_SMCSR)&(~SMCR_PSIZE)) | (PSIZE_4K));
        outpw(REG_SMREAREA_CTL, (inpw(REG_SMREAREA_CTL) & ~SMRE_REA128_EXT) | oob);
    }
    else if (powerOnPageSize == 2)  // 8KB
    {
        outpw(REG_SMCSR, inpw(REG_SMCSR) & ~SMCR_BCH_TSEL);
        if (powerOnEcc == 0)        // BCH T12
        {
            bch = BCH_T12;
            oob = 376;
        }
        else if (powerOnEcc == 1)   // BCH T15
        {
            bch = BCH_T15;
            oob = 472;
        }
        else if (powerOnEcc == 2)   // BCH T24
        {
            bch = BCH_T24;
            oob = 368;
        }
        outpw(REG_SMCSR, inpw(REG_SMCSR) | bch);
        outpw(REG_SMCSR, (inpw(REG_SMCSR)&(~SMCR_PSIZE)) | (PSIZE_8K));
        outpw(REG_SMREAREA_CTL, (inpw(REG_SMREAREA_CTL) & ~SMRE_REA128_EXT) | oob);
    }
}


/*-----------------------------------------------------------------------------
 * Config pSM and register about Region Protect feature.
 * INPUT:
 *      PBA/Page: 0 to disable feature; others to define Region Protect end address
 * OUTPUT:
 *      Assign Region Protect end address to pSM->uRegionProtect
 * RETURN:
 *      0 : OK
 *      -1: Fail since invalid input
 *---------------------------------------------------------------------------*/
static int sicSMRegionProtect(int chipSel, int PBA, int page)
{
    FMI_SM_INFO_T *pSM;
    int pageNo;

    ENTER();
    sicSMselect(chipSel);
    if (chipSel == 0)
        pSM = pSM0;
    else
        pSM = pSM1;

    if ((PBA < 0) || (PBA > pSM0->uBlockPerFlash) || (page < 0) || (page > pSM->uPagePerBlock))
    {
        printk("NAND ERROR: %s(): Invalid block %d and page %d for Region Protect End Address!!\n", __FUNCTION__, PBA, page);
        LEAVE();
        return -1;  // fail
    }

    if ((PBA==0) && (page==0))
        pSM->uRegionProtect = 0;    // disable Region Protect
    else
    {
        PBA += pSM->uLibStartBlock;
        pageNo = PBA * pSM->uPagePerBlock + page;
        pSM->uRegionProtect = pageNo;
    }
    fmiSM_Initial(pSM);
    LEAVE();
    return 0;
}


int nandRegionProtect0(int PBA, int page)
{
    return (sicSMRegionProtect(0, PBA, page));
}


int nandRegionProtect1(int PBA, int page)
{
    return (sicSMRegionProtect(1, PBA, page));
}


int fmiSMCheckRB(FMI_SM_INFO_T *pSM)
{
        unsigned long volatile count1, count2;

        count1 = jiffies + NAND_SHORT_DELAY;

        ENTER();
        while (1) {

    #if defined(CONFIG_W55FA92_TWO_RB_PINS) || defined(CONFIG_W55FA92_NAND1)
                if(pSM == pSM0)
                {
                    if (inpw(REG_SMISR) & SMISR_RB0_IF)
                    {
                        outpw(REG_SMISR, SMISR_RB0_IF);
                        outpw(REG_SMCMD, 0x70);     // status read command
                        if (inpw(REG_SMDATA) & 0x40) {  // 1:ready; 0:busy
                                outpw(REG_SMCMD, 0x00);
                                return 1;
                        } else {
                                count2 = jiffies + NAND_SHORT_DELAY;
                                while (1) {
                                        outpw(REG_SMCMD, 0x70);     // status read command
                                        if (inpw(REG_SMDATA) & 0x40) {  // 1:ready; 0:busy
                                                outpw(REG_SMISR, SMISR_RB0_IF);
                                                outpw(REG_SMCMD, 0x00);
                                                 return 1;
                                        }

                                        if (time_after(jiffies, count2)) {
                                                printk("NAND Warning: R/B# timeout 1\n");
                                                return 0;   // timeout
                                        }
                                        schedule();
                                #ifdef OPT_NAND_ENABLED_PROTECT
                                        outpw(REG_FMICR, FMI_SM_EN);    // 20121205 mhkuo
                                #endif
                                }
                        }
                    }
                }
                else
                {
                    if (inpw(REG_SMISR) & SMISR_RB1_IF)
                    {
                        outpw(REG_SMISR, SMISR_RB1_IF);
                        outpw(REG_SMCMD, 0x70);     // status read command
                        if (inpw(REG_SMDATA) & 0x40) {  // 1:ready; 0:busy
                                outpw(REG_SMCMD, 0x00);
                                return 1;
                        } else {
                                count2 = jiffies + NAND_SHORT_DELAY;
                                while (1) {
                                        outpw(REG_SMCMD, 0x70);     // status read command
                                        if (inpw(REG_SMDATA) & 0x40) {  // 1:ready; 0:busy
                                                outpw(REG_SMISR, SMISR_RB1_IF);
                                                outpw(REG_SMCMD, 0x00);
                                                return 1;
                                        }

                                        if (time_after(jiffies, count2)) {
                                                printk("NAND Warning: R/B# timeout 1\n");
                                                return 0;   // timeout
                                        }
                                        schedule();
                                #ifdef OPT_NAND_ENABLED_PROTECT
                                        outpw(REG_FMICR, FMI_SM_EN);    // 20121205 mhkuo
                                #endif
                                }
                        }
                    }
                }
    #else
                {
                    if (inpw(REG_SMISR) & SMISR_RB0_IF)
                    {
                        outpw(REG_SMISR, SMISR_RB0_IF);
                        outpw(REG_SMCMD, 0x70);     // status read command
                        if (inpw(REG_SMDATA) & 0x40) {  // 1:ready; 0:busy
                                outpw(REG_SMCMD, 0x00);
                                return 1;
                        } else {
                                count2 = jiffies + NAND_SHORT_DELAY;
                                while (1) {
                                        outpw(REG_SMCMD, 0x70);     // status read command
                                        if (inpw(REG_SMDATA) & 0x40) {  // 1:ready; 0:busy
                                                outpw(REG_SMISR, SMISR_RB0_IF);
                                                outpw(REG_SMCMD, 0x00);
                                                return 1;
                                        }

                                        if (time_after(jiffies, count2)) {
                                                printk("NAND Warning: R/B# timeout 1\n");
                                                return 0;   // timeout
                                        }
                                        schedule();
                                #ifdef OPT_NAND_ENABLED_PROTECT
                                        outpw(REG_FMICR, FMI_SM_EN);    // 20121205 mhkuo
                                #endif
                                }
                        }
                    }
                }
    #endif

                if (time_after(jiffies, count1)) {
                        printk("NAND Warning: R/B# timeout 2 <0x%x>, jiffies %x, count1 %x\n", inpw(REG_SMISR), (u32)jiffies, (u32)count1);
                        return 0;   // timeout
                }
                schedule();
        #ifdef OPT_NAND_ENABLED_PROTECT
                outpw(REG_FMICR, FMI_SM_EN);    // 20121205 mhkuo
        #endif
        }
        return 0;
}


int fmiSMCheckStatus(FMI_SM_INFO_T *pSM)
{
    u32 status, ret;

    ret = 0;
    outpw(REG_SMCMD, 0x70);     // Status Read command for NAND flash
    status = inpw(REG_SMDATA);

    if (status & BIT0)          // BIT0: Chip status: 1:fail; 0:pass
    {
        printk("ERROR: NAND device status: FAIL!!\n");
        ret = FMI_SM_STATE_ERROR;
    }

    if ((status & BIT7) == 0)   // BIT7: Write Protect: 1:unprotected; 0:protected
    {
        printk("WARNING: NAND device status: Write Protected!!\n");
        ret = FMI_SM_STATE_ERROR;
    }

    return ret;
}


// SM functions
int fmiSM_Reset(FMI_SM_INFO_T *pSM)
{
#if defined(CONFIG_W55FA92_TWO_RB_PINS) || defined(CONFIG_W55FA92_NAND1)
        u32 volatile i;

        ENTER();

        if(pSM == pSM0)
            outpw(REG_SMISR, SMISR_RB0_IF);
        else
            outpw(REG_SMISR, SMISR_RB1_IF);

        outpw(REG_SMCMD, 0xff);
        for (i=100; i>0; i--);

        if (!fmiSMCheckRB(pSM))
            return FMI_SM_RB_ERR;
        return 0;

#else
        u32 volatile i;

        ENTER();

        outpw(REG_SMISR, SMISR_RB0_IF);
        outpw(REG_SMCMD, 0xff);
        for (i=100; i>0; i--);

        if (!fmiSMCheckRB(pSM))
                return FMI_SM_RB_ERR;
        return 0;

#endif  // CONFIG_W55FA92_TWO_RB_PINS
}


void fmiSM_Initial(FMI_SM_INFO_T *pSM)
{
    ENTER();

    //--- Set registers for ECC enable/disable
    outpw(REG_SMCSR,  inpw(REG_SMCSR) | SMCR_ECC_EN);   // enable ECC

    if (pSM->bIsCheckECC)
        outpw(REG_SMCSR,  inpw(REG_SMCSR) | SMCR_ECC_CHK);  // enable ECC check
    else
        outpw(REG_SMCSR, inpw(REG_SMCSR) & ~SMCR_ECC_CHK);  // disable ECC check

    //--- Set register to disable Mask ECC feature
    outpw(REG_SMREAREA_CTL, inpw(REG_SMREAREA_CTL) & ~SMRE_MECC);

    //--- Set BCH
    sicSMsetBCH(pSM, FALSE);

    if ((_nand_init0 == 0) && (_nand_init1 == 0))
        outpw(REG_SMIER, inpw(REG_SMIER) | SMIER_DMA_IE);    // enable interrupt for DMA complete

    //--- config register for Region Protect
    if (pSM->uRegionProtect == 0)
        outpw(REG_SMCSR, inpw(REG_SMCSR) & ~SMCR_PROT_REGION_EN);   // disable Region Protect feature
    else
    {
        if (pSM->nPageSize == NAND_PAGE_512B)
        {
            // just has 8 bits for column address
            outpw(REG_SM_PROT_ADDR0, (pSM->uRegionProtect & 0x00FFFFFF) << 8);  // low  address (cycle 1~4) for protect region end address
            outpw(REG_SM_PROT_ADDR1, (pSM->uRegionProtect & 0xFF000000) >> 24); // high address (cycle 5) for protect region end address
        }
        else
        {
            // has 16 bits for column address
            outpw(REG_SM_PROT_ADDR0, (pSM->uRegionProtect & 0x0000FFFF) << 16); // low  address (cycle 1~4) for protect region end address
            outpw(REG_SM_PROT_ADDR1, (pSM->uRegionProtect & 0xFFFF0000) >> 16); // high address (cycle 5) for protect region end address
        }
        outpw(REG_SMISR, SMISR_PROT_REGION_WR_IF);                      // clear protect region detection flag
        _fmi_bIsSMPRegionDetect = FALSE;                                // clear protect region detection flag for ISR
        outpw(REG_SMIER, inpw(REG_SMIER) | SMIER_PROT_REGION_WR_IE);    // enable interrupt for Region Protect
        outpw(REG_SMCSR, inpw(REG_SMCSR) | SMCR_PROT_REGION_EN);        // enable Region Protect feature
    }

    LEAVE();
}


int fmiSM_ReadID(FMI_SM_INFO_T *pSM, NDISK_T *NDISK_info)
{
        int volatile tempID[5];
        volatile UINT32 u32PowerOn, powerOnPageSize, powerOnEcc;

        ENTER();
        fmiSM_Reset(pSM);
        outpw(REG_SMCMD, 0x90);     // read ID command
        outpw(REG_SMADDR, 0x80000000);  // address 0x00

#ifdef OPT_SUPPORT_H27UAG8T2A
        ra_224_4k_page = 0;
#endif

        tempID[0] = inpw(REG_SMDATA);
        tempID[1] = inpw(REG_SMDATA);
        tempID[2] = inpw(REG_SMDATA);
        tempID[3] = inpw(REG_SMDATA);
        tempID[4] = inpw(REG_SMDATA);

        NDISK_info->vendor_ID = tempID[0];
        NDISK_info->device_ID = tempID[1];

        pSM->bIsCheckECC = TRUE;
        if (tempID[0] == 0xC2)
        {
            if ((tempID[1] == 0x79) || (tempID[1] == 0x76))
            {
                // Don't support ECC for NAND Interface ROM
                pSM->bIsCheckECC = FALSE;
            }
            else
            {
                // The first ID of this NAND is 0xC2 BUT it is NOT NAND ROM (read only)
                // So, we MUST modify the configuration of it
                //      1. change pSM->bIsCheckECC to TRUE to enable ECC feature;
                //      2. assign a fake vendor_ID to make NVTFAT can write data to this NAND disk.
                //         (GNAND will check vendor_ID and set disk to DISK_TYPE_READ_ONLY if it is 0xC2)
                pSM->bIsCheckECC = TRUE;
                NDISK_info->vendor_ID = 0xFF;   // fake vendor_ID
            }
        }
        else
        {
            pSM->bIsCheckECC = TRUE;
        }

        pSM->bIsNandECC4 = FALSE;
        pSM->bIsNandECC8 = FALSE;
        pSM->bIsNandECC12 = FALSE;
        pSM->bIsNandECC15 = FALSE;
        pSM->bIsNandECC24 = FALSE;

        switch (tempID[1]) {
                /* page size 512B */
        case 0x79:  // 128M
                pSM->uSectorPerFlash = 255744;
                pSM->uBlockPerFlash = 8191;
                pSM->uPagePerBlock = 32;
                pSM->uSectorPerBlock = 32;
                pSM->bIsMulticycle = TRUE;
                pSM->nPageSize = NAND_PAGE_512B;
                pSM->bIsNandECC4 = TRUE;
                pSM->bIsMLCNand = FALSE;

                NDISK_info->NAND_type = NAND_TYPE_SLC;
                NDISK_info->write_page_in_seq = NAND_TYPE_PAGE_OUT_SEQ;
                NDISK_info->nZone = 1;              /* number of zones */
                NDISK_info->nBlockPerZone = 8192;   /* blocks per zone */
                NDISK_info->nPagePerBlock = 32;     /* pages per block */
                NDISK_info->nLBPerZone = 8000;      /* logical blocks per zone */
                NDISK_info->nPageSize = 512;
                break;

        case 0x76:  // 64M
        case 0x5A:  // 64M XtraROM
                pSM->uSectorPerFlash = 127872;
                pSM->uBlockPerFlash = 4095;
                pSM->uPagePerBlock = 32;
                pSM->uSectorPerBlock = 32;
                pSM->bIsMulticycle = TRUE;
                pSM->nPageSize = NAND_PAGE_512B;
                pSM->bIsNandECC4 = TRUE;
                pSM->bIsMLCNand = FALSE;

                NDISK_info->NAND_type = NAND_TYPE_SLC;
                NDISK_info->write_page_in_seq = NAND_TYPE_PAGE_OUT_SEQ;
                NDISK_info->nZone = 1;              /* number of zones */
                NDISK_info->nBlockPerZone = 4096;   /* blocks per zone */
                NDISK_info->nPagePerBlock = 32;     /* pages per block */
                NDISK_info->nLBPerZone = 4000;      /* logical blocks per zone */
                NDISK_info->nPageSize = 512;
                break;

        case 0x75:  // 32M
                pSM->uSectorPerFlash = 63936;
                pSM->uBlockPerFlash = 2047;
                pSM->uPagePerBlock = 32;
                pSM->uSectorPerBlock = 32;
                pSM->bIsMulticycle = FALSE;
                pSM->nPageSize = NAND_PAGE_512B;
                pSM->bIsNandECC4 = TRUE;
                pSM->bIsMLCNand = FALSE;

                NDISK_info->NAND_type = NAND_TYPE_SLC;
                NDISK_info->write_page_in_seq = NAND_TYPE_PAGE_OUT_SEQ;
                NDISK_info->nZone = 1;              /* number of zones */
                NDISK_info->nBlockPerZone = 2048;   /* blocks per zone */
                NDISK_info->nPagePerBlock = 32;     /* pages per block */
                NDISK_info->nLBPerZone = 2000;      /* logical blocks per zone */
                NDISK_info->nPageSize = 512;
                break;

        case 0x73:  // 16M
                pSM->uSectorPerFlash = 31968;   // max. sector no. = 999 * 32
                pSM->uBlockPerFlash = 1023;
                pSM->uPagePerBlock = 32;
                pSM->uSectorPerBlock = 32;
                pSM->bIsMulticycle = FALSE;
                pSM->nPageSize = NAND_PAGE_512B;
                pSM->bIsNandECC4 = TRUE;
                pSM->bIsMLCNand = FALSE;

                NDISK_info->NAND_type = NAND_TYPE_SLC;
                NDISK_info->write_page_in_seq = NAND_TYPE_PAGE_OUT_SEQ;
                NDISK_info->nZone = 1;              /* number of zones */
                NDISK_info->nBlockPerZone = 1024;   /* blocks per zone */
                NDISK_info->nPagePerBlock = 32;     /* pages per block */
                NDISK_info->nLBPerZone = 1000;      /* logical blocks per zone */
                NDISK_info->nPageSize = 512;
                break;

        /* page size 2KB */
        case 0xf1:  // 128M
        case 0xd1:
                pSM->uBlockPerFlash = 1023;
                pSM->uPagePerBlock = 64;
                pSM->uSectorPerBlock = 256;
                pSM->uSectorPerFlash = 255744;
                pSM->bIsMulticycle = FALSE;
                pSM->nPageSize = NAND_PAGE_2KB;
                pSM->bIsNandECC8 = TRUE;
                pSM->bIsMLCNand = FALSE;

                NDISK_info->NAND_type = NAND_TYPE_SLC;
                NDISK_info->write_page_in_seq = NAND_TYPE_PAGE_OUT_SEQ;
                NDISK_info->nZone = 1;              /* number of zones */
                NDISK_info->nBlockPerZone = 1024;   /* blocks per zone */
                NDISK_info->nPagePerBlock = 64;     /* pages per block */
                NDISK_info->nLBPerZone = 1000;      /* logical blocks per zone */
                NDISK_info->nPageSize = 2048;

                // 2013/10/22, support MXIC MX30LF1G08AA NAND flash
                // 2015/06/22, support MXIC MX30LF1G18AC NAND flash
                if ( ((tempID[0]==0xC2)&&(tempID[1]==0xF1)&&(tempID[2]==0x80)&&(tempID[3]==0x1D)) ||
                     ((tempID[0]==0xC2)&&(tempID[1]==0xF1)&&(tempID[2]==0x80)&&(tempID[3]==0x95)&&(tempID[4]==0x02)) )
                {
                    // The first ID of this NAND is 0xC2 BUT it is NOT NAND ROM (read only)
                    // So, we MUST modify the configuration of it
                    //      1. change pSM->bIsCheckECC to TRUE to enable ECC feature;
                    //      2. assign a fake vendor_ID to make NVTFAT can write data to this NAND disk.
                    //         (GNAND will check vendor_ID and set disk to DISK_TYPE_READ_ONLY if it is 0xC2)
                    pSM->bIsCheckECC = TRUE;
                    NDISK_info->vendor_ID = 0xFF;   // fake vendor_ID
                }

                // 2014/10/16, support Winbond W29N01GV NAND flash
                // 2017/09/14, support Samsung K9F1G08U0B NAND flash
                // 2017/09/19, support Winbond W29N01HV NAND flash
                if (   ((tempID[0]==0xEF)&&(tempID[1]==0xF1)&&(tempID[2]==0x80)&&(tempID[3]==0x95))
                    || ((tempID[0]==0xEC)&&(tempID[1]==0xF1)&&(tempID[2]==0x00)&&(tempID[3]==0x95))
                    || ((tempID[0]==0xEF)&&(tempID[1]==0xF1)&&(tempID[2]==0x00)&&(tempID[3]==0x95))
                   )
                {
                    NDISK_info->write_page_in_seq = NAND_TYPE_PAGE_IN_SEQ;
                }
                break;

        case 0xda:  // 256M
                if ((tempID[3] & 0x33) == 0x11)
                {
                        pSM->uBlockPerFlash = 2047;
                        pSM->uPagePerBlock = 64;
                        pSM->uSectorPerBlock = 256;
                        pSM->bIsMLCNand = FALSE;

                        NDISK_info->NAND_type = NAND_TYPE_SLC;
                        NDISK_info->write_page_in_seq = NAND_TYPE_PAGE_OUT_SEQ;
                        NDISK_info->nZone = 1;              /* number of zones */
                        NDISK_info->nPagePerBlock = 64;     /* pages per block */
                        NDISK_info->nBlockPerZone = 2048;   /* blocks per zone */
                        NDISK_info->nLBPerZone = 2000;      /* logical blocks per zone */
                }
                else if ((tempID[3] & 0x33) == 0x21)
                {
                        pSM->uBlockPerFlash = 1023;
                        pSM->uPagePerBlock = 128;
                        pSM->uSectorPerBlock = 512;
                        pSM->bIsMLCNand = TRUE;

                        NDISK_info->NAND_type = NAND_TYPE_MLC;
                        NDISK_info->write_page_in_seq = NAND_TYPE_PAGE_IN_SEQ;
                        NDISK_info->nZone = 1;              /* number of zones */
                        NDISK_info->nPagePerBlock = 128;    /* pages per block */
                        NDISK_info->nBlockPerZone = 1024;   /* blocks per zone */
                        NDISK_info->nLBPerZone = 1000;      /* logical blocks per zone */
                }
                pSM->uSectorPerFlash = 511488;
                pSM->bIsMulticycle = TRUE;
                pSM->nPageSize = NAND_PAGE_2KB;
                pSM->bIsNandECC8 = TRUE;

                // 2018/10/29, support MXIC MX30LF2G18AC NAND flash
                if ((tempID[0]==0xC2)&&(tempID[1]==0xDA)&&(tempID[2]==0x90)&&(tempID[3]==0x95)&&(tempID[4]==0x06))
                {
                    // The first ID of this NAND is 0xC2 BUT it is NOT NAND ROM (read only)
                    // So, we MUST modify the configuration of it
                    //      1. change pSM->bIsCheckECC to TRUE to enable ECC feature;
                    //      2. assign a fake vendor_ID to make NVTFAT can write data to this NAND disk.
                    //         (GNAND will check vendor_ID and set disk to DISK_TYPE_READ_ONLY if it is 0xC2)
                    pSM->bIsCheckECC = TRUE;
                    NDISK_info->vendor_ID = 0xFF;   // fake vendor_ID
                }

                NDISK_info->nPageSize = 2048;
                break;

        case 0xdc:  // 512M
            // 2020/10/08, support Micron MT29F4G08ABAEA 512MB NAND flash
            if ((tempID[0]==0x2C)&&(tempID[2]==0x90)&&(tempID[3]==0xA6)&&(tempID[4]==0x54))
            {
                pSM->uBlockPerFlash  = 2047;        // block index with 0-base. = physical blocks - 1
                pSM->uPagePerBlock   = 64;
                pSM->nPageSize       = NAND_PAGE_4KB;
                pSM->uSectorPerBlock = pSM->nPageSize / 512 * pSM->uPagePerBlock;
                pSM->bIsMLCNand      = FALSE;
                pSM->bIsMulticycle   = TRUE;
                pSM->bIsNandECC24    = TRUE;

                NDISK_info->NAND_type     = (pSM->bIsMLCNand ? NAND_TYPE_MLC : NAND_TYPE_SLC);
                NDISK_info->write_page_in_seq = NAND_TYPE_PAGE_IN_SEQ;
                NDISK_info->nZone         = 1;      // number of zones
                NDISK_info->nBlockPerZone = pSM->uBlockPerFlash + 1;   // blocks per zone
                NDISK_info->nPagePerBlock = pSM->uPagePerBlock;
                NDISK_info->nPageSize     = pSM->nPageSize;
                NDISK_info->nLBPerZone    = 2000;   // logical blocks per zone

                pSM->uSectorPerFlash = pSM->uSectorPerBlock * NDISK_info->nLBPerZone / 1000 * 999;
                break;
            }

                // 2017/9/19, To support both Maker Founder MP4G08JAA
                //                        and Toshiba TC58NVG2S0HTA00 512MB NAND flash
                if ((tempID[0]==0x98)&&(tempID[2]==0x90)&&(tempID[3]==0x26)&&(tempID[4]==0x76))
                {
                    pSM->uBlockPerFlash  = 2047;        // block index with 0-base. = physical blocks - 1
                    pSM->uPagePerBlock   = 64;
                    pSM->nPageSize       = NAND_PAGE_4KB;
                    pSM->uSectorPerBlock = pSM->nPageSize / 512 * pSM->uPagePerBlock;
                    pSM->bIsMLCNand      = FALSE;
                    pSM->bIsMulticycle   = TRUE;
                    pSM->bIsNandECC8     = TRUE;

                    NDISK_info->NAND_type     = (pSM->bIsMLCNand ? NAND_TYPE_MLC : NAND_TYPE_SLC);
                    NDISK_info->write_page_in_seq = NAND_TYPE_PAGE_IN_SEQ;
                    NDISK_info->nZone         = 1;      // number of zones
                    NDISK_info->nBlockPerZone = pSM->uBlockPerFlash + 1;   // blocks per zone
                    NDISK_info->nPagePerBlock = pSM->uPagePerBlock;
                    NDISK_info->nPageSize     = pSM->nPageSize;
                    NDISK_info->nLBPerZone    = 2000;   // logical blocks per zone

                    pSM->uSectorPerFlash = pSM->uSectorPerBlock * NDISK_info->nLBPerZone / 1000 * 999;
                    break;
                }

                if ((tempID[3] & 0x33) == 0x11)
                {
                        pSM->uBlockPerFlash = 4095;
                        pSM->uPagePerBlock = 64;
                        pSM->uSectorPerBlock = 256;
                        pSM->bIsMLCNand = FALSE;

                        NDISK_info->NAND_type = NAND_TYPE_SLC;
                        NDISK_info->write_page_in_seq = NAND_TYPE_PAGE_OUT_SEQ;
                        NDISK_info->nZone = 1;              /* number of zones */
                        NDISK_info->nPagePerBlock = 64;     /* pages per block */
                        NDISK_info->nBlockPerZone = 4096;   /* blocks per zone */
                        NDISK_info->nLBPerZone = 4000;      /* logical blocks per zone */
                }
                else if ((tempID[3] & 0x33) == 0x21)
                {
                        pSM->uBlockPerFlash = 2047;
                        pSM->uPagePerBlock = 128;
                        pSM->uSectorPerBlock = 512;
                        pSM->bIsMLCNand = TRUE;

                        NDISK_info->NAND_type = NAND_TYPE_MLC;
                        NDISK_info->write_page_in_seq = NAND_TYPE_PAGE_IN_SEQ;
                        NDISK_info->nZone = 1;              /* number of zones */
                        NDISK_info->nPagePerBlock = 128;    /* pages per block */
                        NDISK_info->nBlockPerZone = 2048;   /* blocks per zone */
                        NDISK_info->nLBPerZone = 2000;      /* logical blocks per zone */
                }
                pSM->uSectorPerFlash = 1022976;
                pSM->bIsMulticycle = TRUE;
                pSM->nPageSize = NAND_PAGE_2KB;
                pSM->bIsNandECC8 = TRUE;

                NDISK_info->nPageSize = 2048;

                // 2018/10/29, support MXIC MX30LF4G18AC NAND flash
                if ((tempID[0]==0xC2)&&(tempID[1]==0xDC)&&(tempID[2]==0x90)&&(tempID[3]==0x95)&&(tempID[4]==0x56))
                {
                    // The first ID of this NAND is 0xC2 BUT it is NOT NAND ROM (read only)
                    // So, we MUST modify the configuration of it
                    //      1. change pSM->bIsCheckECC to TRUE to enable ECC feature;
                    //      2. assign a fake vendor_ID to make NVTFAT can write data to this NAND disk.
                    //         (GNAND will check vendor_ID and set disk to DISK_TYPE_READ_ONLY if it is 0xC2)
                    pSM->bIsCheckECC = TRUE;
                    NDISK_info->vendor_ID = 0xFF;   // fake vendor_ID
                }
                break;

        case 0xd3:  // 1024M
                // 2014/4/2, To support Samsung K9WAG08U1D 512MB NAND flash
                if ((tempID[0]==0xEC)&&(tempID[2]==0x51)&&(tempID[3]==0x95)&&(tempID[4]==0x58))
                {
                    pSM->uBlockPerFlash  = 4095;        // block index with 0-base. = physical blocks - 1
                    pSM->uPagePerBlock   = 64;
                    pSM->nPageSize       = NAND_PAGE_2KB;
                    pSM->uSectorPerBlock = pSM->nPageSize / 512 * pSM->uPagePerBlock;
                    pSM->bIsMLCNand      = FALSE;
                    pSM->bIsMulticycle   = TRUE;
                    pSM->bIsNandECC8     = TRUE;
                    pSM->uSectorPerFlash = 1022976;

                    NDISK_info->NAND_type     = NAND_TYPE_MLC;
                    NDISK_info->write_page_in_seq = NAND_TYPE_PAGE_IN_SEQ;
                    NDISK_info->nZone         = 1;      // number of zones
                    NDISK_info->nBlockPerZone = pSM->uBlockPerFlash + 1;   // blocks per zone
                    NDISK_info->nPagePerBlock = pSM->uPagePerBlock;
                    NDISK_info->nPageSize     = pSM->nPageSize;
                    NDISK_info->nLBPerZone    = 4000;   // logical blocks per zone
                    break;
                }

                // 2016/9/29, support MXIC MX60LF8G18AC NAND flash
                if ((tempID[0]==0xC2)&&(tempID[1]==0xD3)&&(tempID[2]==0xD1)&&(tempID[3]==0x95)&&(tempID[4]==0x5A))
                {
                    // The first ID of this NAND is 0xC2 BUT it is NOT NAND ROM (read only)
                    // So, we MUST modify the configuration of it
                    //      1. change pSM->bIsCheckECC to TRUE to enable ECC feature;
                    //      2. assign a fake vendor_ID to make NVTFAT can write data to this NAND disk.
                    //         (GNAND will check vendor_ID and set disk to DISK_TYPE_READ_ONLY if it is 0xC2)
                    pSM->bIsCheckECC = TRUE;
                    NDISK_info->vendor_ID = 0xFF;   // fake vendor_ID
                }

                if ((tempID[3] & 0x33) == 0x32)
                {
                        pSM->uBlockPerFlash = 2047;
                        pSM->uPagePerBlock = 128;
                        pSM->uSectorPerBlock = 1024;    /* 128x8 */
                        pSM->nPageSize = NAND_PAGE_4KB;
                        pSM->bIsMLCNand = TRUE;

                        NDISK_info->NAND_type = NAND_TYPE_MLC;
                        NDISK_info->write_page_in_seq = NAND_TYPE_PAGE_IN_SEQ;
                        NDISK_info->nZone = 1;              /* number of zones */
                        NDISK_info->nPagePerBlock = 128;    /* pages per block */
                        NDISK_info->nPageSize = 4096;
                        NDISK_info->nBlockPerZone = 2048;   /* blocks per zone */
                        NDISK_info->nLBPerZone = 2000;      /* logical blocks per zone */
                }
                else if ((tempID[3] & 0x33) == 0x11)
                {
                        pSM->uBlockPerFlash = 8191;
                        pSM->uPagePerBlock = 64;
                        pSM->uSectorPerBlock = 256;
                        pSM->nPageSize = NAND_PAGE_2KB;
                        pSM->bIsMLCNand = FALSE;

                        NDISK_info->NAND_type = NAND_TYPE_SLC;
                        NDISK_info->write_page_in_seq = NAND_TYPE_PAGE_OUT_SEQ;
                        NDISK_info->nZone = 1;              /* number of zones */
                        NDISK_info->nPagePerBlock = 64;     /* pages per block */
                        NDISK_info->nPageSize = 2048;
                        NDISK_info->nBlockPerZone = 8192;   /* blocks per zone */
                        NDISK_info->nLBPerZone = 8000;      /* logical blocks per zone */
                }
                else if ((tempID[3] & 0x33) == 0x21)
                {
                        pSM->uBlockPerFlash = 4095;
                        pSM->uPagePerBlock = 128;
                        pSM->uSectorPerBlock = 512;
                        pSM->nPageSize = NAND_PAGE_2KB;
                        pSM->bIsMLCNand = TRUE;

                        NDISK_info->NAND_type = NAND_TYPE_MLC;
                        NDISK_info->write_page_in_seq = NAND_TYPE_PAGE_IN_SEQ;
                        NDISK_info->nZone = 1;              /* number of zones */
                        NDISK_info->nPagePerBlock = 128;    /* pages per block */
                        NDISK_info->nPageSize = 2048;
                        NDISK_info->nBlockPerZone = 4096;   /* blocks per zone */
                        NDISK_info->nLBPerZone = 4000;      /* logical blocks per zone */
                }
                else if ((tempID[3] & 0x33) == 0x22)
                {
                        pSM->uBlockPerFlash = 4095;
                        pSM->uPagePerBlock = 64;
                        pSM->uSectorPerBlock = 512;
                        pSM->nPageSize = NAND_PAGE_4KB;
                        pSM->bIsMLCNand = FALSE;

                        NDISK_info->NAND_type = NAND_TYPE_SLC;
                        NDISK_info->write_page_in_seq = NAND_TYPE_PAGE_OUT_SEQ;
                        NDISK_info->nZone = 1;              /* number of zones */
                        NDISK_info->nPagePerBlock = 64;     /* pages per block */
                        NDISK_info->nPageSize = 4096;
                        NDISK_info->nBlockPerZone = 4096;   /* blocks per zone */
                        NDISK_info->nLBPerZone = 4000;      /* logical blocks per zone */
                }

                pSM->uSectorPerFlash = 2045952;
                pSM->bIsMulticycle = TRUE;
                pSM->bIsNandECC8 = TRUE;
                break;

        case 0xd5:  // 2048M

            // 2011/7/28, To support Hynix H27UAG8T2B NAND flash
            if ((tempID[0]==0xAD)&&(tempID[2]==0x94)&&(tempID[3]==0x9A))
            {
                pSM->uBlockPerFlash  = 1023;        // block index with 0-base. = physical blocks - 1
                pSM->uPagePerBlock   = 256;
                pSM->nPageSize       = NAND_PAGE_8KB;
                pSM->uSectorPerBlock = pSM->nPageSize / 512 * pSM->uPagePerBlock;
                pSM->bIsMLCNand      = TRUE;
                pSM->bIsMulticycle   = TRUE;
                pSM->bIsNandECC24    = TRUE;

                NDISK_info->NAND_type     = NAND_TYPE_MLC;
                NDISK_info->write_page_in_seq = NAND_TYPE_PAGE_IN_SEQ;
                NDISK_info->nZone         = 1;      // number of zones
                NDISK_info->nBlockPerZone = pSM->uBlockPerFlash + 1;   // blocks per zone
                NDISK_info->nPagePerBlock = pSM->uPagePerBlock;
                NDISK_info->nPageSize     = pSM->nPageSize;
                NDISK_info->nLBPerZone    = 1000;   // logical blocks per zone

                pSM->uSectorPerFlash = 4091904;
                break;
            }

            // 2011/7/28, To support Toshiba TC58NVG4D2FTA00 NAND flash
            if ((tempID[0]==0x98)&&(tempID[2]==0x94)&&(tempID[3]==0x32))
            {
                pSM->uBlockPerFlash  = 2075;        // block index with 0-base. = physical blocks - 1
                pSM->uPagePerBlock   = 128;
                pSM->nPageSize       = NAND_PAGE_8KB;
                pSM->uSectorPerBlock = pSM->nPageSize / 512 * pSM->uPagePerBlock;
                pSM->bIsMLCNand      = TRUE;
                pSM->bIsMulticycle   = TRUE;
                pSM->bIsNandECC24    = TRUE;

                NDISK_info->NAND_type     = NAND_TYPE_MLC;
                NDISK_info->write_page_in_seq = NAND_TYPE_PAGE_IN_SEQ;
                NDISK_info->nZone         = 1;      // number of zones
                NDISK_info->nBlockPerZone = pSM->uBlockPerFlash + 1;   // blocks per zone
                NDISK_info->nPagePerBlock = pSM->uPagePerBlock;
                NDISK_info->nPageSize     = pSM->nPageSize;
                NDISK_info->nLBPerZone    = 2000;   // logical blocks per zone

                pSM->uSectorPerFlash = 4091904;
                break;
            }

    #ifdef OPT_SUPPORT_H27UAG8T2A
            if ((tempID[0]==0xAD)&&(tempID[2] == 0x94)&&(tempID[3] == 0x25))
            {
                pSM->uBlockPerFlash = 4095;
                pSM->uPagePerBlock = 128;
                pSM->uSectorPerBlock = 1024;
                pSM->nPageSize = NAND_PAGE_4KB;
                pSM->bIsMLCNand = TRUE;

                NDISK_info->NAND_type = NAND_TYPE_MLC;
                NDISK_info->write_page_in_seq = NAND_TYPE_PAGE_IN_SEQ;
                NDISK_info->nZone = 1;              /* number of zones */
                NDISK_info->nPagePerBlock = 128;    /* pages per block */
                NDISK_info->nPageSize = 4096;
                NDISK_info->nBlockPerZone = 4096;   /* blocks per zone */
                NDISK_info->nLBPerZone = 4000;      /* logical blocks per zone */

                pSM->uSectorPerFlash = 4091904;
                pSM->bIsMulticycle = TRUE;
                pSM->bIsNandECC12 = TRUE;

                pSM->bIsRA224 = TRUE;
                break;
            }
            else
            {
                if ((tempID[3] & 0x33) == 0x32)
                {
                    pSM->uBlockPerFlash = 4095;
                    pSM->uPagePerBlock = 128;
                    pSM->uSectorPerBlock = 1024;
                    pSM->nPageSize = NAND_PAGE_4KB;
                    pSM->bIsMLCNand = TRUE;

                    NDISK_info->NAND_type = NAND_TYPE_MLC;
                    NDISK_info->write_page_in_seq = NAND_TYPE_PAGE_IN_SEQ;
                    NDISK_info->nZone = 1;              /* number of zones */
                    NDISK_info->nPagePerBlock = 128;    /* pages per block */
                    NDISK_info->nPageSize = 4096;
                    NDISK_info->nBlockPerZone = 4096;   /* blocks per zone */
                    NDISK_info->nLBPerZone = 4000;      /* logical blocks per zone */
                }
                else if ((tempID[3] & 0x33) == 0x11)
                {
                    pSM->uBlockPerFlash = 16383;
                    pSM->uPagePerBlock = 64;
                    pSM->uSectorPerBlock = 256;
                    pSM->nPageSize = NAND_PAGE_2KB;
                    pSM->bIsMLCNand = FALSE;

                    NDISK_info->NAND_type = NAND_TYPE_SLC;
                    NDISK_info->write_page_in_seq = NAND_TYPE_PAGE_OUT_SEQ;
                    NDISK_info->nZone = 1;              /* number of zones */
                    NDISK_info->nPagePerBlock = 64;     /* pages per block */
                    NDISK_info->nPageSize = 2048;
                    NDISK_info->nBlockPerZone = 16384;  /* blocks per zone */
                    NDISK_info->nLBPerZone = 16000;     /* logical blocks per zone */
                }
                else if ((tempID[3] & 0x33) == 0x21)
                {
                    pSM->uBlockPerFlash = 8191;
                    pSM->uPagePerBlock = 128;
                    pSM->uSectorPerBlock = 512;
                    pSM->nPageSize = NAND_PAGE_2KB;
                    pSM->bIsMLCNand = TRUE;

                    NDISK_info->NAND_type = NAND_TYPE_MLC;
                    NDISK_info->write_page_in_seq = NAND_TYPE_PAGE_IN_SEQ;
                    NDISK_info->nZone = 1;              /* number of zones */
                    NDISK_info->nPagePerBlock = 128;    /* pages per block */
                    NDISK_info->nPageSize = 2048;
                    NDISK_info->nBlockPerZone = 8192;   /* blocks per zone */
                    NDISK_info->nLBPerZone = 8000;      /* logical blocks per zone */
                }

                pSM->uSectorPerFlash = 4091904;
                pSM->bIsMulticycle = TRUE;
                pSM->bIsNandECC8 = TRUE;
                break;
            }
    #else
            if ((tempID[3] & 0x33) == 0x32)
            {
                pSM->uBlockPerFlash = 4095;
                pSM->uPagePerBlock = 128;
                pSM->uSectorPerBlock = 1024;
                pSM->nPageSize = NAND_PAGE_4KB;
                pSM->bIsMLCNand = TRUE;

                NDISK_info->NAND_type = NAND_TYPE_MLC;
                NDISK_info->write_page_in_seq = NAND_TYPE_PAGE_IN_SEQ;
                NDISK_info->nZone = 1;              /* number of zones */
                NDISK_info->nPagePerBlock = 128;    /* pages per block */
                NDISK_info->nPageSize = 4096;
                NDISK_info->nBlockPerZone = 4096;   /* blocks per zone */
                NDISK_info->nLBPerZone = 4000;      /* logical blocks per zone */
            }
            else if ((tempID[3] & 0x33) == 0x11)
            {
                pSM->uBlockPerFlash = 16383;
                pSM->uPagePerBlock = 64;
                pSM->uSectorPerBlock = 256;
                pSM->nPageSize = NAND_PAGE_2KB;
                pSM->bIsMLCNand = FALSE;

                NDISK_info->NAND_type = NAND_TYPE_SLC;
                NDISK_info->write_page_in_seq = NAND_TYPE_PAGE_OUT_SEQ;
                NDISK_info->nZone = 1;              /* number of zones */
                NDISK_info->nPagePerBlock = 64;     /* pages per block */
                NDISK_info->nPageSize = 2048;
                NDISK_info->nBlockPerZone = 16384;  /* blocks per zone */
                NDISK_info->nLBPerZone = 16000;     /* logical blocks per zone */
            }
            else if ((tempID[3] & 0x33) == 0x21)
            {
                pSM->uBlockPerFlash = 8191;
                pSM->uPagePerBlock = 128;
                pSM->uSectorPerBlock = 512;
                pSM->nPageSize = NAND_PAGE_2KB;
                pSM->bIsMLCNand = TRUE;

                NDISK_info->NAND_type = NAND_TYPE_MLC;
                NDISK_info->write_page_in_seq = NAND_TYPE_PAGE_IN_SEQ;
                NDISK_info->nZone = 1;              /* number of zones */
                NDISK_info->nPagePerBlock = 128;    /* pages per block */
                NDISK_info->nPageSize = 2048;
                NDISK_info->nBlockPerZone = 8192;   /* blocks per zone */
                NDISK_info->nLBPerZone = 8000;      /* logical blocks per zone */
            }

            pSM->uSectorPerFlash = 4091904;
            pSM->bIsMulticycle = TRUE;
            pSM->bIsNandECC8 = TRUE;
            break;
    #endif

        default:
            // 2012/3/8, support Micron MT29F16G08CBACA NAND flash
            if ((tempID[0]==0x2C)&&(tempID[1]==0x48)&&(tempID[2]==0x04)&&(tempID[3]==0x4A)&&(tempID[4]==0xA5))
            {
                pSM->uBlockPerFlash  = 2047;        // block index with 0-base. = physical blocks - 1
                pSM->uPagePerBlock   = 256;
                pSM->nPageSize       = NAND_PAGE_4KB;
                pSM->uSectorPerBlock = pSM->nPageSize / 512 * pSM->uPagePerBlock;
                pSM->bIsMLCNand      = TRUE;
                pSM->bIsMulticycle   = TRUE;
                pSM->bIsNandECC24    = TRUE;

                NDISK_info->NAND_type     = NAND_TYPE_MLC;
                NDISK_info->write_page_in_seq = NAND_TYPE_PAGE_IN_SEQ;
                NDISK_info->nZone         = 1;      // number of zones
                NDISK_info->nBlockPerZone = pSM->uBlockPerFlash + 1;   // blocks per zone
                NDISK_info->nPagePerBlock = pSM->uPagePerBlock;
                NDISK_info->nPageSize     = pSM->nPageSize;
                NDISK_info->nLBPerZone    = 2000;   // logical blocks per zone

                pSM->uSectorPerFlash = 4091904;
                break;
            }
           // 2012/3/27, support Micron MT29F32G08CBACA NAND flash
            else if ((tempID[0]==0x2C)&&(tempID[1]==0x68)&&(tempID[2]==0x04)&&(tempID[3]==0x4A)&&(tempID[4]==0xA9))
            {
                pSM->uBlockPerFlash  = 4095;        // block index with 0-base. = physical blocks - 1
                pSM->uPagePerBlock   = 256;
                pSM->nPageSize       = NAND_PAGE_4KB;
                pSM->uSectorPerBlock = pSM->nPageSize / 512 * pSM->uPagePerBlock;
                pSM->bIsMLCNand      = TRUE;
                pSM->bIsMulticycle   = TRUE;
                pSM->bIsNandECC24    = TRUE;

                NDISK_info->NAND_type     = NAND_TYPE_MLC;
                NDISK_info->write_page_in_seq = NAND_TYPE_PAGE_IN_SEQ;
                NDISK_info->nZone         = 1;      // number of zones
                NDISK_info->nBlockPerZone = pSM->uBlockPerFlash + 1;   // blocks per zone
                NDISK_info->nPagePerBlock = pSM->uPagePerBlock;
                NDISK_info->nPageSize     = pSM->nPageSize;
                NDISK_info->nLBPerZone    = 4000;   // logical blocks per zone

                pSM->uSectorPerFlash = 8183808; // = pSM->uSectorPerBlock * NDISK_info->nLBPerZone / 1000 * 999
                break;
            }
            // 2013/9/25, support MXIC MX30LF1208AA NAND flash
            else if ((tempID[0]==0xC2)&&(tempID[1]==0xF0)&&(tempID[2]==0x80)&&(tempID[3]==0x1D))
            {
                pSM->uBlockPerFlash  = 511;         // block index with 0-base. = physical blocks - 1
                pSM->uPagePerBlock   = 64;
                pSM->nPageSize       = NAND_PAGE_2KB;
                pSM->uSectorPerBlock = pSM->nPageSize / 512 * pSM->uPagePerBlock;
                pSM->bIsMLCNand      = FALSE;
                pSM->bIsMulticycle   = FALSE;
                pSM->bIsNandECC8     = TRUE;

                NDISK_info->NAND_type     = NAND_TYPE_SLC;
                NDISK_info->write_page_in_seq = NAND_TYPE_PAGE_OUT_SEQ;
                NDISK_info->nZone         = 1;      // number of zones
                NDISK_info->nBlockPerZone = pSM->uBlockPerFlash + 1;    // blocks per zone
                NDISK_info->nPagePerBlock = pSM->uPagePerBlock;
                NDISK_info->nPageSize     = pSM->nPageSize;
                NDISK_info->nLBPerZone    = 500;    // logical blocks per zone

                pSM->uSectorPerFlash = pSM->uSectorPerBlock * NDISK_info->nLBPerZone / 1000 * 999;

                // The first ID of this NAND is 0xC2 BUT it is NOT NAND ROM (read only)
                // So, we MUST modify the configuration of it
                //      1. change pSM->bIsCheckECC to TRUE to enable ECC feature;
                //      2. assign a fake vendor_ID to make NVTFAT can write data to this NAND disk.
                //         (GNAND will check vendor_ID and set disk to DISK_TYPE_READ_ONLY if it is 0xC2)
                pSM->bIsCheckECC = TRUE;
                NDISK_info->vendor_ID = 0xFF;   // fake vendor_ID

                break;
            }

            printk("NAND ERROR: SM ID not support!! %02X-%02X-%02X-%02X\n", tempID[0], tempID[1], tempID[2], tempID[3]);
            LEAVE();
            return FMI_SM_ID_ERR;
        }

        u32PowerOn = inpw(REG_CHIPCFG);
        // CHIPCFG[9:8]  : 0=2KB,   1=4KB,   2=8KB,   3=Ignore power-on setting
        powerOnPageSize = (u32PowerOn & (NPAGE)) >> 8;
        // CHIPCFG[11,1] : 0=BCH12, 1=BCH15, 2=BCH24, 3=Ignore BCH and power-on setting
        powerOnEcc = ((u32PowerOn & (BIT11)) >> 10) | ((u32PowerOn & (BIT1)) >> 1);

        if ((powerOnPageSize == 3)||(powerOnEcc == 3))
        {
            printk("NAND: Found %s NAND, ID %02X-%02X-%02X-%02X-%02X, page size %d, BCH T%d\n",
                pSM->bIsMLCNand ? "MLC" : "SLC",
                tempID[0], tempID[1], tempID[2], tempID[3], tempID[4],
                pSM->nPageSize,
                pSM->bIsNandECC4*4 + pSM->bIsNandECC8*8 + pSM->bIsNandECC12*12 + pSM->bIsNandECC15*15 + pSM->bIsNandECC24*24
                );
        }
        else
        {
            printk("NAND: Found %s NAND, ID %02X-%02X-%02X-%02X-%02X\n",
                pSM->bIsMLCNand ? "MLC" : "SLC",
                tempID[0], tempID[1], tempID[2], tempID[3], tempID[4]);
            printk("      By Power-on setting, NAND page size %d, BCH T%d\n",
                1024 << (powerOnPageSize+1),
                ( powerOnEcc == 0) ? 12 :
                ((powerOnEcc == 1) ? 15 :
                ((powerOnEcc == 2) ? 24 : 0)));
        }

        LEAVE();
        return 0;
}


int fmiSM2BufferM(FMI_SM_INFO_T *pSM, u32 uSector, u8 ucColAddr)
{
        ENTER();
        /* clear R/B flag */
#if defined(CONFIG_W55FA92_TWO_RB_PINS) || defined(CONFIG_W55FA92_NAND1)
        if(pSM == pSM0)
        {
            while (!(inpw(REG_SMISR) & SMISR_RB0));
            outpw(REG_SMISR, SMISR_RB0_IF);
        }
        else
        {
            while (!(inpw(REG_SMISR) & SMISR_RB1));
            outpw(REG_SMISR, SMISR_RB1_IF);
        }
#else
        while (!(inpw(REG_SMISR) & SMISR_RB0));
        outpw(REG_SMISR, SMISR_RB0_IF);
#endif

        outpw(REG_SMCMD, 0x00);             // read command
        outpw(REG_SMADDR, ucColAddr);       // CA0 - CA7
        outpw(REG_SMADDR, uSector & 0xff);  // PA0 - PA7
        if (!pSM->bIsMulticycle)
            outpw(REG_SMADDR, ((uSector >> 8) & 0xff)|EOA_SM);      // PA8 - PA15
        else {
            outpw(REG_SMADDR, (uSector >> 8) & 0xff);               // PA8 - PA15
            outpw(REG_SMADDR, ((uSector >> 16) & 0xff)|EOA_SM);     // PA16 - PA17
        }

        if (!fmiSMCheckRB(pSM)) {
            LEAVE();
            return FMI_SM_RB_ERR;
        }
        LEAVE();
        return 0;
}


int fmiSM2BufferM_RA(FMI_SM_INFO_T *pSM, u32 uSector, u8 ucColAddr)
{
        ENTER();
        /* 2011/03/04, to support NAND ROM chip "Infinite IM90A001GT" */
        if (! pSM->bIsCheckECC)
        {
            LEAVE();
            return 0;
        }

        /* clear R/B flag */
#if defined(CONFIG_W55FA92_TWO_RB_PINS) || defined(CONFIG_W55FA92_NAND1)
        if(pSM == pSM0)
        {
            while (!(inpw(REG_SMISR) & SMISR_RB0));
            outpw(REG_SMISR, SMISR_RB0_IF);
        }
        else
        {
            while (!(inpw(REG_SMISR) & SMISR_RB1));
            outpw(REG_SMISR, SMISR_RB1_IF);
        }
#else

        while (!(inpw(REG_SMISR) & SMISR_RB0));
        outpw(REG_SMISR, SMISR_RB0_IF);
#endif

        outpw(REG_SMCMD, 0x50);             // read command
        outpw(REG_SMADDR, ucColAddr);       // CA0 - CA7
        outpw(REG_SMADDR, uSector & 0xff);  // PA0 - PA7
        if (!pSM->bIsMulticycle)
                outpw(REG_SMADDR, ((uSector >> 8) & 0xff)|EOA_SM);  // PA8 - PA15
        else {
                outpw(REG_SMADDR, (uSector >> 8) & 0xff);           // PA8 - PA15
                outpw(REG_SMADDR, ((uSector >> 16) & 0xff)|EOA_SM); // PA16 - PA17
        }

        if (!fmiSMCheckRB(pSM))
        {
            LEAVE();
            return FMI_SM_RB_ERR;
        }

        LEAVE();
        return 0;
}


/*-----------------------------------------------------------------------------
 * fmiSM_CorrectData_BCH() correct data by BCH alrogithm.
 *      Support 8K page size NAND and BCH T4/8/12/15/24.
 *---------------------------------------------------------------------------*/
void fmiSM_CorrectData_BCH(u8 ucFieidIndex, u8 ucErrorCnt, u8* pDAddr)
{
    u32 uaData[24], uaAddr[24];

    u32 uaErrorData[4];
    u8  ii, jj;
    u32 uPageSize;
    u32 field_len, padding_len, parity_len;
    u32 total_field_num;
    u8  *smra_index;

    ENTER();

    //--- assign some parameters for different BCH and page size
    switch (inpw(REG_SMCSR) & SMCR_BCH_TSEL)
    {
        case BCH_T24:
            field_len   = 1024;
            padding_len = BCH_PADDING_LEN_1024;
            parity_len  = BCH_PARITY_LEN_T24;
            break;
        case BCH_T15:
            field_len   = 512;
            padding_len = BCH_PADDING_LEN_512;
            parity_len  = BCH_PARITY_LEN_T15;
            break;
        case BCH_T12:
            field_len   = 512;
            padding_len = BCH_PADDING_LEN_512;
            parity_len  = BCH_PARITY_LEN_T12;
            break;
        case BCH_T8:
            field_len   = 512;
            padding_len = BCH_PADDING_LEN_512;
            parity_len  = BCH_PARITY_LEN_T8;
            break;
        case BCH_T4:
            field_len   = 512;
            padding_len = BCH_PADDING_LEN_512;
            parity_len  = BCH_PARITY_LEN_T4;
            break;
        default:
            printk("NAND ERROR: %s(): invalid SMCR_BCH_TSEL = 0x%08X\n", __FUNCTION__, (u32)(inpw(REG_SMCSR) & SMCR_BCH_TSEL));
            LEAVE();
            return;
    }

    uPageSize = inpw(REG_SMCSR) & SMCR_PSIZE;
    switch (uPageSize)
    {
        case PSIZE_8K:  total_field_num = 8192 / field_len; break;
        case PSIZE_4K:  total_field_num = 4096 / field_len; break;
        case PSIZE_2K:  total_field_num = 2048 / field_len; break;
        case PSIZE_512: total_field_num =  512 / field_len; break;
        default:
            printk("NAND ERROR: %s(): invalid SMCR_PSIZE = 0x%08X\n", __FUNCTION__, uPageSize);
            LEAVE();
            return;
    }

    //--- got valid BCH_ECC_DATAx and parse them to uaData[]
    // got the valid register number of BCH_ECC_DATAx since one register include 4 error bytes
    jj = ucErrorCnt/4;
    jj ++;
    if (jj > 6)
        jj = 6;     // there are 6 BCH_ECC_DATAx registers to support BCH T24

    for(ii=0; ii<jj; ii++)
    {
        uaErrorData[ii] = inpw(REG_BCH_ECC_DATA0 + ii*4);
    }


    for(ii=0; ii<jj; ii++)
    {
        uaData[ii*4+0] = uaErrorData[ii] & 0xff;
        uaData[ii*4+1] = (uaErrorData[ii]>>8) & 0xff;
        uaData[ii*4+2] = (uaErrorData[ii]>>16) & 0xff;
        uaData[ii*4+3] = (uaErrorData[ii]>>24) & 0xff;
    }

    //--- got valid REG_BCH_ECC_ADDRx and parse them to uaAddr[]
    // got the valid register number of REG_BCH_ECC_ADDRx since one register include 2 error addresses
    jj = ucErrorCnt/2;
    jj ++;
    if (jj > 12)
        jj = 12;    // there are 12 REG_BCH_ECC_ADDRx registers to support BCH T24

    for(ii=0; ii<jj; ii++)
    {
        uaAddr[ii*2+0] = inpw(REG_BCH_ECC_ADDR0 + ii*4) & 0x07ff;   // 11 bits for error address
        uaAddr[ii*2+1] = (inpw(REG_BCH_ECC_ADDR0 + ii*4)>>16) & 0x07ff;
    }

    //--- pointer to begin address of field that with data error
    pDAddr += (ucFieidIndex-1) * field_len;

    //--- correct each error bytes
    for(ii=0; ii<ucErrorCnt; ii++)
    {
        // for wrong data in field
        if (uaAddr[ii] < field_len)
        {
            PDEBUG("BCH error corrected for data: address 0x%08X, data [0x%02X] --> ",
                (unsigned int)(pDAddr+uaAddr[ii]), (unsigned int)(*(pDAddr+uaAddr[ii])));

            *(pDAddr+uaAddr[ii]) ^= uaData[ii];

            PDEBUG("[0x%02X]\n", *(pDAddr+uaAddr[ii]));
        }
        // for wrong first-3-bytes in redundancy area
        else if (uaAddr[ii] < (field_len+3))
        {
            uaAddr[ii] -= field_len;
            uaAddr[ii] += (parity_len*(ucFieidIndex-1));    // field offset

            PDEBUG("BCH error corrected for 3 bytes: address 0x%08X, data [0x%02X] --> ",
                (unsigned int)((u8 *)REG_SMRA_0+uaAddr[ii]), (unsigned int)(*((u8 *)REG_SMRA_0+uaAddr[ii])));

            *((u8 *)REG_SMRA_0+uaAddr[ii]) ^= uaData[ii];

            PDEBUG("[0x%02X]\n", *((u8 *)REG_SMRA_0+uaAddr[ii]));
        }
        // for wrong parity code in redundancy area
        else
        {
            // BCH_ERR_ADDRx = [data in field] + [3 bytes] + [xx] + [parity code]
            //                                   |<--     padding bytes      -->|
            // The BCH_ERR_ADDRx for last parity code always = field size + padding size.
            // So, the first parity code = field size + padding size - parity code length.
            // For example, for BCH T12, the first parity code = 512 + 32 - 23 = 521.
            // That is, error byte address offset within field is
            uaAddr[ii] = uaAddr[ii] - (field_len + padding_len - parity_len);

            // smra_index point to the first parity code of first field in register SMRA0~n
            smra_index = (u8 *)
                         (REG_SMRA_0 + (inpw(REG_SMREAREA_CTL) & SMRE_REA128_EXT) - // bottom of all parity code -
                          (parity_len * total_field_num)                            // byte count of all parity code
                         );

            // final address = first parity code of first field +
            //                 offset of fields +
            //                 offset within field
            PDEBUG("BCH error corrected for parity: address 0x%08X, data [0x%02X] --> ",
                (unsigned int)(smra_index + (parity_len * (ucFieidIndex-1)) + uaAddr[ii]),
                (unsigned int)(*(smra_index + (parity_len * (ucFieidIndex-1)) + uaAddr[ii])));
            *((u8 *)smra_index + (parity_len * (ucFieidIndex-1)) + uaAddr[ii]) ^= uaData[ii];
            PDEBUG("[0x%02X]\n",
                *((u8 *)smra_index + (parity_len * (ucFieidIndex-1)) + uaAddr[ii]));
        }
    }   // end of for (ii<ucErrorCnt)

    LEAVE();
}


int fmiSM_Read_512(FMI_SM_INFO_T *pSM, u32 uSector, u32 uDAddr)
{
        int volatile ret=0;
        u32 uStatus;
        u32 uErrorCnt;

        ENTER();

        if (down_interruptible(&dmac_sem)) {
                LEAVE();
                //printk("io err\n");
                return(GNERR_IO_ERR);
        }
        while (inpw(REG_DMACCSR)&FMI_BUSY); //Wait IP finished... for safe

        outpw(REG_DMACSAR, _fmi_ucNANDBuffer);

        _fmi_bIsSMDataReady = FALSE;
        ret = fmiSM2BufferM(pSM, uSector, 0);
        if (ret < 0) {
                up(&dmac_sem);
                printk("NAND ERROR: read error 0x%x\n", ret);
                LEAVE();
                return ret;
        }

        outpw(REG_SMISR, SMISR_DMA_IF);                 // clear DMA flag
        outpw(REG_SMISR, SMISR_ECC_FIELD_IF);           // clear ECC_FIELD flag

        outpw(REG_SMCSR, inpw(REG_SMCSR) | SMCR_DRD_EN);

        while (!_fmi_bIsSMDataReady);

        if (pSM->bIsCheckECC)
        {
            while(1)
            {
                if (inpw(REG_SMISR) & SMISR_ECC_FIELD_IF)
                {
                    if (inpw(REG_SMCSR) & BCH_T4)   // BCH_ECC selected
                    {
                        uStatus = inpw(REG_SM_ECC_ST0);
                        uStatus &= 0x3f;

                        if ((uStatus & 0x03)==0x01)         // correctable error in 1st field
                        {
                            // 2011/8/17, mask uErrorCnt since Fx_ECNT just has 5 valid bits
                            uErrorCnt = (uStatus >> 2) & 0x1F;
                            fmiSM_CorrectData_BCH(1, uErrorCnt, (u8*)uDAddr);
                            PDEBUG("NAND Warning: Field 1 have %d BCH error. Corrected!!\n", uErrorCnt);
                        }
                        else if (((uStatus & 0x03)==0x02) ||
                                 ((uStatus & 0x03)==0x03))  // uncorrectable error or ECC error
                        {
                            outpw(REG_SMISR, SMISR_ECC_FIELD_IF);
                            outpw(REG_DMACCSR, DMAC_EN+DMAC_SWRST);         // reset DMAC
                            while(inpw(REG_DMACCSR) & DMAC_SWRST);
                            outpw(REG_SMCSR, inpw(REG_SMCSR)|SMCR_SM_SWRST);// reset SM controller
                            while(inpw(REG_SMCSR) & SMCR_SM_SWRST);
                            up(&dmac_sem);
                            printk("NAND ERROR: SM uncorrectable error is encountered, uStatus=0x%4x !!\n", uStatus);
                            LEAVE();
                            return FMI_SM_ECC_ERROR;
                        }
                    }
                    else
                    {
                        printk("NAND ERROR: Wrong BCH setting for page-512 NAND !!\n");
                    }

                    outpw(REG_SMISR, SMISR_ECC_FIELD_IF);       // clear ECC_FLD_Error
                }
            #if 1
                if ( !(inpw(REG_SMISR) & SMISR_ECC_FIELD_IF) )
                    break;
            #else
                if (inpw(REG_SMISR) & SMISR_DMA_IF)      // wait to finish DMAC transfer.
                {
                    printk("NAND: regiater SM_ISR = 0x%08X \n ", inpw(REG_SMISR));
                    if ( !(inpw(REG_SMISR) & SMISR_ECC_FIELD_IF) )
                    {
                        break;
                    }
                }
            #endif
            }
        }
        else
            outpw(REG_SMISR, SMISR_ECC_FIELD_IF);

        memcpy((char *)uDAddr, (char *)_fmi_pNANDBuffer, 512);
        outpw(REG_SMISR, SMISR_DMA_IF);     // clear DMA flag
        up(&dmac_sem);
        LEAVE();
        return 0;
}


void fmiBuffer2SMM(FMI_SM_INFO_T *pSM, u32 uSector, u8 ucColAddr)
{
        ENTER();

        if (write_nandloader)
            // add "check marker" (0xFF5Axx00, xx is page number) in spare area for NandLoader
            outpw(REG_SMRA_0, 0x00005AFF | (((uSector % pSM->uPagePerBlock) & 0xFF) << 16));
        else
            // set the spare area configuration
            /* write byte 514, 515 as used page */
            outpw(REG_SMRA_0, 0x0000FFFF);

        outpw(REG_SMRA_1, 0xFFFFFFFF);

        // send command
        outpw(REG_SMCMD, 0x80);             // serial data input command
        outpw(REG_SMADDR, ucColAddr);       // CA0 - CA7
        outpw(REG_SMADDR, uSector & 0xff);  // PA0 - PA7
        if (!pSM->bIsMulticycle)
                outpw(REG_SMADDR, ((uSector >> 8) & 0xff)|EOA_SM);      // PA8 - PA15
        else {
                outpw(REG_SMADDR, (uSector >> 8) & 0xff);               // PA8 - PA15
                outpw(REG_SMADDR, ((uSector >> 16) & 0xff)|EOA_SM);     // PA16 - PA17
        }
}


int fmiSM_Write_512(FMI_SM_INFO_T *pSM, u32 uSector, u32 uSAddr)
{
        ENTER();
        /* clear R/B flag */
#if defined(CONFIG_W55FA92_TWO_RB_PINS) || defined(CONFIG_W55FA92_NAND1)
        if(pSM == pSM0)
        {
            while (!(inpw(REG_SMISR) & SMISR_RB0));
            outpw(REG_SMISR, SMISR_RB0_IF);
        }
        else
        {
            while (!(inpw(REG_SMISR) & SMISR_RB1));
            outpw(REG_SMISR, SMISR_RB1_IF);
        }
#else

        while (!(inpw(REG_SMISR) & SMISR_RB0));
        outpw(REG_SMISR, SMISR_RB0_IF);
#endif

        outpw(REG_SMCSR, inpw(REG_SMCSR) | SMCR_REDUN_AUTO_WEN);

        memcpy((char *)_fmi_pNANDBuffer, (char *)uSAddr, 512);

        if (down_interruptible(&dmac_sem))
        {
            LEAVE();
            return(GNERR_IO_ERR);
        }
        while (inpw(REG_DMACCSR)&FMI_BUSY); // Wait IP finished... for safe

        _fmi_bIsSMDataReady = FALSE;
        _fmi_bIsSMPRegionDetect = FALSE;

        outpw(REG_DMACSAR, _fmi_ucNANDBuffer);

        fmiBuffer2SMM(pSM, uSector, 0);

        outpw(REG_SMISR, SMISR_DMA_IF);                 // clear DMA flag
        outpw(REG_SMISR, SMISR_ECC_FIELD_IF);           // clear ECC_FIELD flag
        outpw(REG_SMISR, SMISR_PROT_REGION_WR_IF);      // clear Region Protect flag
        outpw(REG_SMCSR, inpw(REG_SMCSR) | SMCR_DWR_EN);
        while (!_fmi_bIsSMDataReady);

        outpw(REG_SMISR, SMISR_DMA_IF); // clear DMA flag
        outpw(REG_SMCMD, 0x10);         // auto program command

        if (!fmiSMCheckRB(pSM)) {
            up(&dmac_sem);
            LEAVE();
            return FMI_SM_RB_ERR;
        }

        //--- check Region Protect result
        if (_fmi_bIsSMPRegionDetect)
        {
            outpw(REG_SMISR, SMISR_PROT_REGION_WR_IF);      // clear Region Protect flag
            up(&dmac_sem);
            printk("NAND ERROR: %s(): region write protect detected!!\n", __FUNCTION__);
            LEAVE();
            return FMI_SM_REGION_PROTECT_ERR;
        }

        if (fmiSMCheckStatus(pSM) != 0) {
            up(&dmac_sem);
            LEAVE();
            return FMI_SM_STATE_ERROR;
        }
        up(&dmac_sem);
        LEAVE();
        return 0;
}

int fmiSM_Read_2K(FMI_SM_INFO_T *pSM, u32 uPage, u32 uDAddr)
{
        u32 uStatus;
        u32 uErrorCnt, ii;

        ENTER();
        if (down_interruptible(&dmac_sem))
        {
            LEAVE();
            return(GNERR_IO_ERR);
        }

        while (inpw(REG_DMACCSR)&FMI_BUSY); // Wait IP finished... for safe

        outpw(REG_DMACCSR, inpw(REG_DMACCSR) | DMAC_EN);
        outpw(REG_DMACSAR, _fmi_ucNANDBuffer);

        /* clear R/B flag */
#if defined(CONFIG_W55FA92_TWO_RB_PINS) || defined(CONFIG_W55FA92_NAND1)
        if(pSM == pSM0)
        {
            while (!(inpw(REG_SMISR) & SMISR_RB0));
            outpw(REG_SMISR, SMISR_RB0_IF);
        }
        else
        {
            while (!(inpw(REG_SMISR) & SMISR_RB1));
            outpw(REG_SMISR, SMISR_RB1_IF);
        }
#else

        while (!(inpw(REG_SMISR) & SMISR_RB0));
        outpw(REG_SMISR, SMISR_RB0_IF);
#endif

        if (inpw(REG_SMISR) & SMISR_ECC_FIELD_IF)   /* ECC_FLD_IF */
        {
            printk("NAND ERROR: read ECC error!!\n");
            outpw(REG_SMISR, SMISR_ECC_FIELD_IF);
        }

        outpw(REG_SMCMD, 0x00);             // read command
        outpw(REG_SMADDR, 0);               // CA0 - CA7
        outpw(REG_SMADDR, 0);               // CA8 - CA11
        outpw(REG_SMADDR, uPage & 0xff);    // PA0 - PA7
        if (!pSM->bIsMulticycle)
            outpw(REG_SMADDR, ((uPage >> 8) & 0xff)|EOA_SM);    // PA8 - PA15
        else {
            outpw(REG_SMADDR, (uPage >> 8) & 0xff);             // PA8 - PA15
            outpw(REG_SMADDR, ((uPage >> 16) & 0xff)|EOA_SM);   // PA16 - PA17
        }
        outpw(REG_SMCMD, 0x30);     // read command

        if (!fmiSMCheckRB(pSM)) {
            up(&dmac_sem);
            LEAVE();
            return FMI_SM_RB_ERR;
        }
        _fmi_bIsSMDataReady = FALSE;

        outpw(REG_SMISR, SMISR_DMA_IF);                 // clear DMA flag
        outpw(REG_SMISR, SMISR_ECC_FIELD_IF);           // clear ECC_FIELD flag
        outpw(REG_SMCSR, inpw(REG_SMCSR) | SMCR_DRD_EN);

        if (pSM->bIsCheckECC)
        {
            while(1)
            {
                if (inpw(REG_SMISR) & SMISR_ECC_FIELD_IF)
                {
                    uStatus = inpw(REG_SM_ECC_ST0);
                    for (ii=1; ii<5; ii++)
                    {
                        if ((uStatus & 0x03)==0x01)  // correctable error in 1st field
                        {
                            // 2011/8/17, mask uErrorCnt since Fx_ECNT just has 5 valid bits
                            uErrorCnt = (uStatus >> 2) & 0x1F;
                            fmiSM_CorrectData_BCH(ii, uErrorCnt, (u8*)_fmi_pNANDBuffer);
                            PDEBUG("NAND Warning: Field %d have %d BCH error. Corrected!!\n", ii, uErrorCnt);
                            break;
                        }
                        else if (((uStatus & 0x03)==0x02) ||
                                 ((uStatus & 0x03)==0x03)) // uncorrectable error or ECC error in 1st field
                        {
                            #if 0
                                printk("register REG_DMACCSR = %4x \n ", inpw(REG_DMACCSR));
                                printk("register REG_DMACSAR = %4x \n ", inpw(REG_DMACSAR));
                                printk("register REG_DMACBCR = %4x \n ", inpw(REG_DMACBCR));
                                printk("register REG_DMACIER = %4x \n ", inpw(REG_DMACIER));
                                printk("register REG_DMACISR = %4x \n ", inpw(REG_DMACISR));
                                printk("register REG_FMICR = %4x \n ", inpw(REG_FMICR));
                                printk("register REG_FMIIER = %4x \n ", inpw(REG_FMIIER));
                                printk("register REG_FMIISR = %4x \n ", inpw(REG_FMIISR));
                                printk("register REG_SMCSR = %4x \n ", inpw(REG_SMCSR));
                                printk("register REG_SMTCR = %4x \n ", inpw(REG_SMTCR));
                                printk("register REG_SMIER = %4x \n ", inpw(REG_SMIER));
                                printk("register REG_SMISR = %4x \n ", inpw(REG_SMISR));

                                printk("physical PageNo = 0x%4x \n ", uPage);
                                printk("SMRA contents: \n ");
                                for(ii=0; ii<4; ii+=0x10)
                                {
                                    for(jj=0; jj<0x10; jj++)
                                    {
                                        printk("0x%2x ", __raw_readl(REG_SMRA_0+ii*0x10+jj));
                                    }
                                        printk("\n");
                                }
                                printk("SMRA contents: \n ");
                            #endif

                            outpw(REG_SMISR, SMISR_ECC_FIELD_IF);
                            outpw(REG_DMACCSR, DMAC_EN+DMAC_SWRST);         // reset DMAC
                            while(inpw(REG_DMACCSR) & DMAC_SWRST);
                            outpw(REG_SMCSR, inpw(REG_SMCSR)|SMCR_SM_SWRST);// reset SM controller
                            while(inpw(REG_SMCSR) & SMCR_SM_SWRST);
                            memcpy((char *)uDAddr, (char *)_fmi_pNANDBuffer, 2048);
                            up(&dmac_sem);
                            printk("NAND ERROR: SM uncorrectable error is encountered, 0x%08X !!\n", uStatus);
                            LEAVE();
                            return FMI_SM_ECC_ERROR;
                        }
                        uStatus >>= 8;
                    }

                    outpw(REG_SMISR, SMISR_ECC_FIELD_IF);       // clear ECC_FLD_Error
                }

                if (_fmi_bIsSMDataReady)
                {
                    if ( !(inpw(REG_SMISR) & SMISR_ECC_FIELD_IF) )
                        break;
                }
            }
        }
        else
        {
            while(1)
            {
                if (_fmi_bIsSMDataReady)
                {
                    outpw(REG_SMISR, SMISR_DMA_IF);                 // clear DMA flag
                    outpw(REG_SMISR, SMISR_ECC_FIELD_IF);
                    break;
                }
            }
        }

        memcpy((char *)uDAddr, (char *)_fmi_pNANDBuffer, 2048);
        up(&dmac_sem);
        LEAVE();
        return 0;
}


int fmiSM_Read_RA(FMI_SM_INFO_T *pSM, u32 uPage, u32 ucColAddr)
{
        ENTER();

        /* clear R/B flag */
#if defined(CONFIG_W55FA92_TWO_RB_PINS) || defined(CONFIG_W55FA92_NAND1)
        if(pSM == pSM0)
        {
            while (!(inpw(REG_SMISR) & SMISR_RB0));
            outpw(REG_SMISR, SMISR_RB0_IF);
        }
        else
        {
            while (!(inpw(REG_SMISR) & SMISR_RB1));
            outpw(REG_SMISR, SMISR_RB1_IF);
        }
#else

        while (!(inpw(REG_SMISR) & SMISR_RB0));
        outpw(REG_SMISR, SMISR_RB0_IF);
#endif

        outpw(REG_SMCMD, 0x00);                     // read command
        outpw(REG_SMADDR, ucColAddr);               // CA0 - CA7
        outpw(REG_SMADDR, (ucColAddr >> 8) & 0xff); // CA8 - CA11
        outpw(REG_SMADDR, uPage & 0xff);            // PA0 - PA7
        if (!pSM->bIsMulticycle)
            outpw(REG_SMADDR, ((uPage >> 8) & 0xff)|EOA_SM);    // PA8 - PA15
        else {
            outpw(REG_SMADDR, (uPage >> 8) & 0xff);             // PA8 - PA15
            outpw(REG_SMADDR, ((uPage >> 16) & 0xff)|EOA_SM);   // PA16 - PA17
        }
        outpw(REG_SMCMD, 0x30);     // read command

        if (!fmiSMCheckRB(pSM)) {
            printk("NAND ERROR: *** R/B error !!! ***\n");
            LEAVE();
            return FMI_SM_RB_ERR;
        }
        LEAVE();
        return 0;
}

int fmiSM_Read_RA_512(FMI_SM_INFO_T *pSM, u32 uPage, u32 uColumm)
{
    return fmiSM2BufferM_RA(pSM, uPage, uColumm);
}

int fmiSM_Write_2K(FMI_SM_INFO_T *pSM, u32 uSector, u32 ucColAddr, u32 uSAddr)
{
        ENTER();

        if (down_interruptible(&dmac_sem))
        {
            LEAVE();
            return(GNERR_IO_ERR);
        }
        while (inpw(REG_DMACCSR)&FMI_BUSY); // Wait IP finished... for safe

        memcpy((char *)_fmi_pNANDBuffer, (char *)uSAddr, 2048);

        outpw(REG_DMACSAR, _fmi_ucNANDBuffer);

        if (write_nandloader)
            // add "check marker" (0xFF5Axx00, xx is page number) in spare area for NandLoader
            outpw(REG_SMRA_0, 0x00005AFF | (((uSector % pSM->uPagePerBlock) & 0xFF) << 16));
        else
            // set the spare area configuration
            /* write byte 2050, 2051 as used page */
            outpw(REG_SMRA_0, 0x0000FFFF);

        outpw(REG_SMCSR, inpw(REG_SMCSR) | SMCR_REDUN_AUTO_WEN);

        /* clear R/B flag */
#if defined(CONFIG_W55FA92_TWO_RB_PINS) || defined(CONFIG_W55FA92_NAND1)
        if(pSM == pSM0)
        {
            while (!(inpw(REG_SMISR) & SMISR_RB0));
            outpw(REG_SMISR, SMISR_RB0_IF);
        }
        else
        {
            while (!(inpw(REG_SMISR) & SMISR_RB1));
            outpw(REG_SMISR, SMISR_RB1_IF);
        }
#else

        while (!(inpw(REG_SMISR) & SMISR_RB0));
        outpw(REG_SMISR, SMISR_RB0_IF);
#endif

        if (inpw(REG_SMISR) & SMISR_ECC_FIELD_IF) { /* ECC_FLD_IF */
                // printk("error sector !!\n");
                outpw(REG_SMISR, SMISR_ECC_FIELD_IF);
        }

        // send command
        outpw(REG_SMCMD, 0x80);                     // serial data input command
        outpw(REG_SMADDR, ucColAddr);               // CA0 - CA7
        outpw(REG_SMADDR, (ucColAddr >> 8) & 0xff); // CA8 - CA11
        outpw(REG_SMADDR, uSector & 0xff);          // PA0 - PA7
        if (!pSM->bIsMulticycle)
                outpw(REG_SMADDR, ((uSector >> 8) & 0xff)|EOA_SM);  // PA8 - PA15
        else {
                outpw(REG_SMADDR, (uSector >> 8) & 0xff);           // PA8 - PA15
                outpw(REG_SMADDR, ((uSector >> 16) & 0xff)|EOA_SM); // PA16 - PA17
        }

        _fmi_bIsSMDataReady = FALSE;
        _fmi_bIsSMPRegionDetect = FALSE;

        outpw(REG_SMISR, SMISR_DMA_IF);                 // clear DMA flag
        outpw(REG_SMISR, SMISR_ECC_FIELD_IF);           // clear ECC_FIELD flag
        outpw(REG_SMISR, SMISR_PROT_REGION_WR_IF);      // clear Region Protect flag
        outpw(REG_SMCSR, inpw(REG_SMCSR) | SMCR_DWR_EN);

        while (!_fmi_bIsSMDataReady) {
//                if (inpw(REG_SMISR) & SMISR_ECC_FIELD_IF) { /* ECC_FLD_IF */
//                      printk("write: error sector !!\n");
//                      outpw(REG_SMISR, SMISR_ECC_FIELD_IF);
//                }
        }

        outpw(REG_SMCMD, 0x10);     // auto program command
        if (!fmiSMCheckRB(pSM)) {
            up(&dmac_sem);
            LEAVE();
            return FMI_SM_RB_ERR;
        }

        //--- check Region Protect result
        if (_fmi_bIsSMPRegionDetect)
        {
            outpw(REG_SMISR, SMISR_PROT_REGION_WR_IF);      // clear Region Protect flag
            up(&dmac_sem);
            printk("NAND ERROR: %s(): region write protect detected!!\n", __FUNCTION__);
            LEAVE();
            return FMI_SM_REGION_PROTECT_ERR;
        }

        if (fmiSMCheckStatus(pSM) != 0) {
            up(&dmac_sem);
            LEAVE();
            return FMI_SM_STATE_ERROR;
        }
        up(&dmac_sem);
        LEAVE();
        return 0;
}


int fmiSM_Read_4K(FMI_SM_INFO_T *pSM, u32 uPage, u32 uDAddr)
{
        u32 uStatus;
        u32 uErrorCnt, ii, jj;

        ENTER();

        if (down_interruptible(&dmac_sem))
        {
            LEAVE();
            return(GNERR_IO_ERR);
        }

        while (inpw(REG_DMACCSR)&FMI_BUSY);     // Wait IP finished... for safe

        outpw(REG_DMACSAR, _fmi_ucNANDBuffer);  // set DMA transfer starting address

        /* clear R/B flag */
#if defined(CONFIG_W55FA92_TWO_RB_PINS) || defined(CONFIG_W55FA92_NAND1)
        if(pSM == pSM0)
        {
            while (!(inpw(REG_SMISR) & SMISR_RB0));
            outpw(REG_SMISR, SMISR_RB0_IF);
        }
        else
        {
            while (!(inpw(REG_SMISR) & SMISR_RB1));
            outpw(REG_SMISR, SMISR_RB1_IF);
        }
#else

        while (!(inpw(REG_SMISR) & SMISR_RB0));
        outpw(REG_SMISR, SMISR_RB0_IF);
#endif

        outpw(REG_SMCMD, 0x00);             // read command
        outpw(REG_SMADDR, 0);               // CA0 - CA7
        outpw(REG_SMADDR, 0);               // CA8 - CA11
        outpw(REG_SMADDR, uPage & 0xff);    // PA0 - PA7
        if (!pSM->bIsMulticycle)
            outpw(REG_SMADDR, ((uPage >> 8) & 0xff)|EOA_SM);    // PA8 - PA15
        else {
            outpw(REG_SMADDR, (uPage >> 8) & 0xff);             // PA8 - PA15
            outpw(REG_SMADDR, ((uPage >> 16) & 0xff)|EOA_SM);   // PA16 - PA17
        }
        outpw(REG_SMCMD, 0x30);     // read command

        if (!fmiSMCheckRB(pSM)) {
            up(&dmac_sem);
            LEAVE();
            return FMI_SM_RB_ERR;
        }
        _fmi_bIsSMDataReady = FALSE;

        outpw(REG_SMISR, SMISR_DMA_IF);                 // clear DMA flag
        outpw(REG_SMISR, SMISR_ECC_FIELD_IF);           // clear ECC_FIELD flag
        outpw(REG_SMCSR, inpw(REG_SMCSR) | SMCR_DRD_EN);

        if ((pSM->bIsCheckECC) || (inpw(REG_SMCSR)&SMCR_ECC_CHK) )
        {
            while(1)
            {
                if (inpw(REG_SMISR) & SMISR_ECC_FIELD_IF)
                {
                    for (jj=0; jj<2; jj++)
                    {
                        uStatus = inpw(REG_SM_ECC_ST0+jj*4);
                        if (!uStatus)
                            continue;

                        for (ii=1; ii<5; ii++)
                        {
                            if (!(uStatus & 0x03))
                            {
                                uStatus >>= 8;
                                continue;
                            }

                            if ((uStatus & 0x03)==0x01)  // correctable error in 1st field
                            {
                                // 2011/8/17, mask uErrorCnt since Fx_ECNT just has 5 valid bits
                                uErrorCnt = (uStatus >> 2) & 0x1F;
                                fmiSM_CorrectData_BCH(ii, uErrorCnt, (u8*)_fmi_pNANDBuffer);
                                PDEBUG("NAND Warning: Field %d have %d BCH error. Corrected!!\n", ii, uErrorCnt);
                                break;
                            }
                            else if (((uStatus & 0x03)==0x02) ||
                                     ((uStatus & 0x03)==0x03)) // uncorrectable error or ECC error in 1st field
                            {
                                outpw(REG_SMISR, SMISR_ECC_FIELD_IF);
                                outpw(REG_DMACCSR, DMAC_EN+DMAC_SWRST);         // reset DMAC
                                while(inpw(REG_DMACCSR) & DMAC_SWRST);
                                outpw(REG_SMCSR, inpw(REG_SMCSR)|SMCR_SM_SWRST);// reset SM controller
                                while(inpw(REG_SMCSR) & SMCR_SM_SWRST);
                                memcpy((char *)uDAddr, (char *)_fmi_pNANDBuffer, 4096);
                                up(&dmac_sem);
                                printk("NAND ERROR: SM uncorrectable BCH error is encountered !!\n");
                                LEAVE();
                                return FMI_SM_ECC_ERROR;
                            }
                            uStatus >>= 8;
                        }
                    }
                    outpw(REG_SMISR, SMISR_ECC_FIELD_IF);       // clear ECC_FLD_Error
                }

                if (_fmi_bIsSMDataReady)
                {
                    if ( !(inpw(REG_SMISR) & SMISR_ECC_FIELD_IF) )
                        break;
                }
            }
        }
        else
        {
            while(1)
            {
                if (_fmi_bIsSMDataReady)
                {
                    outpw(REG_SMISR, SMISR_DMA_IF);                 // clear DMA flag
                    outpw(REG_SMISR, SMISR_ECC_FIELD_IF);
                    break;
                }
            }
        }
        memcpy((char *)uDAddr, (char *)_fmi_pNANDBuffer, 4096);
        up(&dmac_sem);
        LEAVE();
        return 0;
}


int fmiSM_Read_8K(FMI_SM_INFO_T *pSM, u32 uPage, u32 uDAddr)
{
        u32 uStatus;
        u32 uErrorCnt, ii, jj;

        ENTER();
        if (down_interruptible(&dmac_sem))
        {
            LEAVE();
            return(GNERR_IO_ERR);
        }

        while (inpw(REG_DMACCSR)&FMI_BUSY); // Wait IP finished... for safe

        outpw(REG_DMACSAR, _fmi_ucNANDBuffer);

        /* clear R/B flag */
#if defined(CONFIG_W55FA92_TWO_RB_PINS) || defined(CONFIG_W55FA92_NAND1)
        if(pSM == pSM0)
        {
            while (!(inpw(REG_SMISR) & SMISR_RB0));
            outpw(REG_SMISR, SMISR_RB0_IF);
        }
        else
        {
            while (!(inpw(REG_SMISR) & SMISR_RB1));
            outpw(REG_SMISR, SMISR_RB1_IF);
        }
#else
        while (!(inpw(REG_SMISR) & SMISR_RB0));
        outpw(REG_SMISR, SMISR_RB0_IF);
#endif

        outpw(REG_SMCMD, 0x00);             // read command
        outpw(REG_SMADDR, 0);               // CA0 - CA7
        outpw(REG_SMADDR, 0);               // CA8 - CA11
        outpw(REG_SMADDR, uPage & 0xff);    // PA0 - PA7
        if (!pSM->bIsMulticycle)
                outpw(REG_SMADDR, ((uPage >> 8) & 0xff)|EOA_SM);    // PA8 - PA15
        else {
                outpw(REG_SMADDR, (uPage >> 8) & 0xff);             // PA8 - PA15
                outpw(REG_SMADDR, ((uPage >> 16) & 0xff)|EOA_SM);   // PA16 - PA17
        }
        outpw(REG_SMCMD, 0x30);     // read command

        if (!fmiSMCheckRB(pSM)) {
                up(&dmac_sem);
                LEAVE();
                return FMI_SM_RB_ERR;
        }
        _fmi_bIsSMDataReady = FALSE;

        outpw(REG_SMISR, SMISR_DMA_IF);                 // clear DMA flag
        outpw(REG_SMISR, SMISR_ECC_FIELD_IF);           // clear ECC_FIELD flag
        outpw(REG_SMCSR, inpw(REG_SMCSR) | SMCR_DRD_EN);

        if ((pSM->bIsCheckECC) || (inpw(REG_SMCSR)&SMCR_ECC_CHK) )
        {
            while(1)
            {
                if (inpw(REG_SMISR) & SMISR_ECC_FIELD_IF)
                {
                    for (jj=0; jj<2; jj++)
                    {
                        uStatus = inpw(REG_SM_ECC_ST0+jj*4);
                        if (!uStatus)
                            continue;

                        for (ii=1; ii<5; ii++)
                        {
                            if (!(uStatus & 0x03))
                            {
                                uStatus >>= 8;
                                continue;
                            }

                            if ((uStatus & 0x03)==0x01)  // correctable error in 1st field
                            {
                                // 2011/8/17, mask uErrorCnt since Fx_ECNT just has 5 valid bits
                                uErrorCnt = (uStatus >> 2) & 0x1F;
                                fmiSM_CorrectData_BCH(ii, uErrorCnt, (u8*)_fmi_pNANDBuffer);
                                PDEBUG("NAND Warning: Field %d have %d BCH error. Corrected!!\n", ii, uErrorCnt);
                                break;
                            }
                            else if (((uStatus & 0x03)==0x02) ||
                                     ((uStatus & 0x03)==0x03)) // uncorrectable error or ECC error in 1st field
                            {
                                outpw(REG_SMISR, SMISR_ECC_FIELD_IF);
                                outpw(REG_DMACCSR, DMAC_EN+DMAC_SWRST);         // reset DMAC
                                while(inpw(REG_DMACCSR) & DMAC_SWRST);
                                outpw(REG_SMCSR, inpw(REG_SMCSR)|SMCR_SM_SWRST);// reset SM controller
                                while(inpw(REG_SMCSR) & SMCR_SM_SWRST);
                                memcpy((char *)uDAddr, (char *)_fmi_pNANDBuffer, 8192);
                                up(&dmac_sem);
                                printk("NAND ERROR: SM uncorrectable BCH error is encountered !!\n");
                                LEAVE();
                                return FMI_SM_ECC_ERROR;
                            }
                            uStatus >>= 8;
                        }
                    }
                    outpw(REG_SMISR, SMISR_ECC_FIELD_IF);       // clear ECC_FLD_Error
                }

                if (_fmi_bIsSMDataReady)
                {
                    if ( !(inpw(REG_SMISR) & SMISR_ECC_FIELD_IF) )
                        break;
                }
            }
        }
        else
        {
            while(1)
            {
                if (_fmi_bIsSMDataReady)
                {
                    outpw(REG_SMISR, SMISR_DMA_IF);                 // clear DMA flag
                    outpw(REG_SMISR, SMISR_ECC_FIELD_IF);
                    break;
                }
            }
        }
        memcpy((char *)uDAddr, (char *)_fmi_pNANDBuffer, 8192);
        up(&dmac_sem);
        LEAVE();
        return 0;
}


static void sicSMselect(int chipSel)
{
    ENTER();
    if (chipSel == 0)
    {
        outpw(REG_GPDFUN0, (inpw(REG_GPDFUN0) & (~0xF0F00000)) | 0x20200000);   // enable NRE/RB0 pins
        outpw(REG_GPDFUN1, (inpw(REG_GPDFUN1) & (~0x0000000F)) | 0x00000002);   // enable NWR pins
        outpw(REG_GPEFUN1, (inpw(REG_GPEFUN1) & (~0x000FFF0F)) | 0x00022202);   // enable CS0/ALE/CLE/ND3 pins
        outpw(REG_SMCSR, inpw(REG_SMCSR) & ~SMCR_CS0);
        outpw(REG_SMCSR, inpw(REG_SMCSR) |  SMCR_CS1);
    }
    else
    {
        outpw(REG_GPDFUN0, (inpw(REG_GPDFUN0) & (~0xFF000000)) | 0x22000000);   // enable NRE/RB1 pins
        outpw(REG_GPDFUN1, (inpw(REG_GPDFUN1) & (~0x0000000F)) | 0x00000002);   // enable NWR pins
        outpw(REG_GPEFUN1, (inpw(REG_GPEFUN1) & (~0x000FFFF0)) | 0x00022220);   // enable CS1/ALE/CLE/ND3 pins
        outpw(REG_SMCSR, inpw(REG_SMCSR) & ~SMCR_CS1);
        outpw(REG_SMCSR, inpw(REG_SMCSR) |  SMCR_CS0);
    }

    //--- 2014/2/26, Reset NAND controller and DMAC to keep clean status for next access.
    // Reset DMAC engine and interrupt satus
    outpw(REG_DMACCSR, inpw(REG_DMACCSR) | DMAC_SWRST | DMAC_EN);
    while(inpw(REG_DMACCSR) & DMAC_SWRST);
    outpw(REG_DMACCSR, inpw(REG_DMACCSR) | DMAC_EN);
    outpw(REG_DMACISR, WEOT_IF | TABORT_IF);    // clear all interrupt flag

    // Reset FMI engine and interrupt status
    outpw(REG_FMICR, FMI_SWRST);
    while(inpw(REG_FMICR) & FMI_SWRST);
    outpw(REG_FMIISR, FMI_DAT_IF);              // clear all interrupt flag

    // Reset NAND engine and interrupt status
    outpw(REG_FMICR, FMI_SM_EN);
    outpw(REG_SMCSR, inpw(REG_SMCSR) | SMCR_SM_SWRST);
    while(inpw(REG_SMCSR) & SMCR_SM_SWRST);
    outpw(REG_SMISR, 0xFFFFFFFF);               // clear all interrupt flag

    LEAVE();
}


int fmiSM_Write_4K(FMI_SM_INFO_T *pSM, u32 uSector, u32 ucColAddr, u32 uSAddr)
{
        ENTER();
        if (down_interruptible(&dmac_sem))
        {
            LEAVE();
            return(GNERR_IO_ERR);
        }
        while (inpw(REG_DMACCSR)&FMI_BUSY); // Wait IP finished... for safe

        memcpy((char *)_fmi_pNANDBuffer, (char *)uSAddr, 4096);
        outpw(REG_DMACSAR, _fmi_ucNANDBuffer);

        if (write_nandloader)
            // add "check marker" (0xFF5Axx00, xx is page number) in spare area for NandLoader
            outpw(REG_SMRA_0, 0x00005AFF | (((uSector % pSM->uPagePerBlock) & 0xFF) << 16));
        else
            // set the spare area configuration
            /* write byte 4098, 4099 as used page */
            outpw(REG_SMRA_0, 0x0000FFFF);

        outpw(REG_SMCSR, inpw(REG_SMCSR) | SMCR_REDUN_AUTO_WEN);

        /* clear R/B flag */
#if defined(CONFIG_W55FA92_TWO_RB_PINS) || defined(CONFIG_W55FA92_NAND1)
        if(pSM == pSM0)
        {
            while (!(inpw(REG_SMISR) & SMISR_RB0));
            outpw(REG_SMISR, SMISR_RB0_IF);
        }
        else
        {
            while (!(inpw(REG_SMISR) & SMISR_RB1));
            outpw(REG_SMISR, SMISR_RB1_IF);
        }
#else

        while (!(inpw(REG_SMISR) & SMISR_RB0));
        outpw(REG_SMISR, SMISR_RB0_IF);
#endif

        // send command
        outpw(REG_SMCMD, 0x80);                     // serial data input command
        outpw(REG_SMADDR, ucColAddr);               // CA0 - CA7
        outpw(REG_SMADDR, (ucColAddr >> 8) & 0xff); // CA8 - CA11
        outpw(REG_SMADDR, uSector & 0xff);          // PA0 - PA7
        if (!pSM->bIsMulticycle)
            outpw(REG_SMADDR, ((uSector >> 8) & 0xff)|EOA_SM);  // PA8 - PA15
        else {
            outpw(REG_SMADDR, (uSector >> 8) & 0xff);           // PA8 - PA15
            outpw(REG_SMADDR, ((uSector >> 16) & 0xff)|EOA_SM); // PA16 - PA17
        }

        _fmi_bIsSMDataReady = FALSE;
        _fmi_bIsSMPRegionDetect = FALSE;

        outpw(REG_SMISR, SMISR_DMA_IF);                 // clear DMA flag
        outpw(REG_SMISR, SMISR_ECC_FIELD_IF);           // clear ECC_FIELD flag
        outpw(REG_SMISR, SMISR_PROT_REGION_WR_IF);      // clear Region Protect flag
        outpw(REG_SMCSR, inpw(REG_SMCSR) | SMCR_DWR_EN);

        while (!_fmi_bIsSMDataReady) {
            //printk("--> fmiSM_Write_4K(): wait _fmi_bIsSMDataReady, uSector=%d\n", uSector);
            //    if (inpw(REG_SMISR) & SMISR_ECC_FIELD_IF) { /* ECC_FLD_IF */
            //            printk("write: error sector !!\n");
            //           outpw(REG_SMISR, SMISR_ECC_FIELD_IF);
            //    }
        }

        outpw(REG_SMCMD, 0x10);     // auto program command
        if (!fmiSMCheckRB(pSM)) {
            up(&dmac_sem);
            LEAVE();
            return FMI_SM_RB_ERR;
        }

        //--- check Region Protect result
        if (_fmi_bIsSMPRegionDetect)
        {
            outpw(REG_SMISR, SMISR_PROT_REGION_WR_IF);      // clear Region Protect flag
            up(&dmac_sem);
            printk("NAND ERROR: %s(): region write protect detected!!\n", __FUNCTION__);
            LEAVE();
            return FMI_SM_REGION_PROTECT_ERR;
        }

        if (fmiSMCheckStatus(pSM) != 0) {
            up(&dmac_sem);
            LEAVE();
            return FMI_SM_STATE_ERROR;
        }
        up(&dmac_sem);
        LEAVE();
        return 0;
}


/*-----------------------------------------------------------------------------
 * 2011/8/1, To clear Ready/Busy flag in registers.
 *---------------------------------------------------------------------------*/
void fmiSMClearRBflag(FMI_SM_INFO_T *pSM)
{
#if defined(CONFIG_W55FA92_TWO_RB_PINS) || defined(CONFIG_W55FA92_NAND1)
    if (pSM == pSM0)
    {
        while(!(inpw(REG_SMISR) & SMISR_RB0));
        outpw(REG_SMISR, SMISR_RB0_IF);
    }
    else
    {
        while(!(inpw(REG_SMISR) & SMISR_RB1));
        outpw(REG_SMISR, SMISR_RB1_IF);
    }
#else
    while(!(inpw(REG_SMISR) & SMISR_RB0));
    outpw(REG_SMISR, SMISR_RB0_IF);
#endif
}


static int sicSMpread(int NandPort, int PBA, int page, u8 *buff)
{
    FMI_SM_INFO_T *pSM;
    int pageNo;
    int status;
    int i;
    char *ptr;
    int spareSize;

    ENTER();

    // enable SM
    outpw(REG_FMICR, FMI_SM_EN);

    sicSMselect(NandPort);
    if (NandPort == 0)
        pSM = pSM0;
    else
        pSM = pSM1;

    fmiSM_Initial(pSM);

    PBA += pSM->uLibStartBlock;
    pageNo = PBA * pSM->uPagePerBlock + page;

#ifdef OPT_FIRST_4BLOCKS_ECC4
    if (PBA <= 3)
        sicSMsetBCH(pSM, TRUE);
#endif

        spareSize = inpw(REG_SMREAREA_CTL) & SMRE_REA128_EXT;
        ptr = (char *)REG_SMRA_0;
        if (pSM->nPageSize == NAND_PAGE_2KB)    /* 2KB */
        {
            LEAVE();
            fmiSM_Read_RA(pSM, pageNo, 2048);
            for (i=0; i<64; i++)
            {
                *ptr++ = inpw(REG_SMDATA) & 0xff;
            }

            status = fmiSM_Read_2K(pSM, pageNo, (u32)buff);
        }
        else if (pSM->nPageSize == NAND_PAGE_4KB)   /* 4KB */
        {
            LEAVE();
            fmiSM_Read_RA(pSM, pageNo, 4096);
#ifdef OPT_SUPPORT_H27UAG8T2A
            for (i=0; i<spareSize; i++)
                *ptr++ = inpw(REG_SMDATA) & 0xff;
#else
            for (i=0; i<128; i++)
                *ptr++ = inpw(REG_SMDATA) & 0xff;
#endif
            status = fmiSM_Read_4K(pSM, pageNo, (u32)buff);
        }
        else if (pSM->nPageSize == NAND_PAGE_8KB)   /* 8KB */
        {
            LEAVE();
            fmiSM_Read_RA(pSM, pageNo, pSM->nPageSize);
            for (i=0; i<spareSize; i++)
                *ptr++ = inpw(REG_SMDATA) & 0xff;
            status = fmiSM_Read_8K(pSM, pageNo, (u32)buff);
        }
        else {  /* 512B */
            LEAVE();
            fmiSM_Read_RA_512(pSM, pageNo, 0);
            for (i=0; i<16; i++)
                *ptr++ = inpw(REG_SMDATA) & 0xff;
            status = fmiSM_Read_512(pSM, pageNo, (u32)buff);
        }
        if (status < 0)
        {
            printk("NAND ERROR: %s(): read Nand%d fail for PBA %d page %d ! Return = 0x%x\n", __FUNCTION__, NandPort, PBA, page, status);
        }

#ifdef OPT_FIRST_4BLOCKS_ECC4
    if (PBA <= 3)
        sicSMsetBCH(pSM, FALSE);
#endif
    LEAVE();
    return (status);
}


/*-----------------------------------------------------------------------------
 * Really write data and parity code to 2K/4K/8K page size NAND flash by NAND commands.
 *---------------------------------------------------------------------------*/
static int fmiSM_Write_large_page(FMI_SM_INFO_T *pSM, u32 uSector, u32 ucColAddr, u32 uSAddr)
{
    ENTER();
    if (down_interruptible(&dmac_sem))
    {
        LEAVE();
        return(GNERR_IO_ERR);
    }
    while (inpw(REG_DMACCSR)&FMI_BUSY); // wait IP finished... for safe

    memcpy((char *)_fmi_pNANDBuffer, (char *)uSAddr, pSM->nPageSize);
    outpw(REG_DMACSAR, _fmi_ucNANDBuffer);  // set DMA transfer starting address

    // set the spare area configuration
    /* write 0x00 to both 3rd byte and 4th byte of spare area as used page */
    outpw(REG_SMRA_0, 0x0000FFFF);

    fmiSMClearRBflag(pSM);  // clear R/B flag

    if (inpw(REG_SMISR) & SMISR_ECC_FIELD_IF) { /* ECC_FLD_IF */
        // printk("error sector !!\n");
        outpw(REG_SMISR, SMISR_ECC_FIELD_IF);
    }

    // send command
    outpw(REG_SMCMD, 0x80);                     // serial data input command
    outpw(REG_SMADDR, ucColAddr);               // CA0 - CA7
    outpw(REG_SMADDR, (ucColAddr >> 8) & 0xff); // CA8 - CA12
    outpw(REG_SMADDR, uSector & 0xff);          // PA0 - PA7
    if (!pSM->bIsMulticycle)
        outpw(REG_SMADDR, ((uSector >> 8) & 0xff)|EOA_SM);  // PA8 - PA15
    else
    {
        outpw(REG_SMADDR, (uSector >> 8) & 0xff);           // PA8 - PA15
        outpw(REG_SMADDR, ((uSector >> 16) & 0xff)|EOA_SM); // PA16 - PA17
    }

    _fmi_bIsSMDataReady = FALSE;
    _fmi_bIsSMPRegionDetect = FALSE;

    outpw(REG_SMISR, SMISR_DMA_IF);                     // clear DMA flag
    outpw(REG_SMISR, SMISR_ECC_FIELD_IF);               // clear ECC_FIELD flag
    outpw(REG_SMISR, SMISR_PROT_REGION_WR_IF);          // clear Region Protect flag
    outpw(REG_SMCSR, inpw(REG_SMCSR) | SMCR_REDUN_AUTO_WEN);    // auto write redundancy data to NAND after page data written
    outpw(REG_SMCSR, inpw(REG_SMCSR) | SMCR_DWR_EN);            // begin to write one page data to NAND flash

    while(1)
    {
        if (_fmi_bIsSMDataReady)
            break;
    }

    outpw(REG_SMCMD, 0x10);         // auto program command

    if (!fmiSMCheckRB(pSM))
    {
        up(&dmac_sem);
        LEAVE();
        return FMI_SM_RB_ERR;
    }

    //--- check Region Protect result
    if (_fmi_bIsSMPRegionDetect)
    {
        outpw(REG_SMISR, SMISR_PROT_REGION_WR_IF);      // clear Region Protect flag
        up(&dmac_sem);
        printk("NAND ERROR: %s(): region write protect detected!!\n", __FUNCTION__);
        LEAVE();
        return FMI_SM_REGION_PROTECT_ERR;
    }

    if (fmiSMCheckStatus(pSM) != 0) {
        up(&dmac_sem);
        printk("NAND ERROR: %s(): data error!!\n", __FUNCTION__);
        LEAVE();
        return FMI_SM_STATE_ERROR;
    }
    up(&dmac_sem);
    LEAVE();
    return 0;
}


static int sicSMpwrite(int NandPort, int PBA, int page, u8 *buff)
{
    FMI_SM_INFO_T *pSM;
    int pageNo;
    int status;

    ENTER();

    // enable SM
    outpw(REG_FMICR, FMI_SM_EN);

    sicSMselect(NandPort);
    if (NandPort == 0)
        pSM = pSM0;
    else
        pSM = pSM1;

    fmiSM_Initial(pSM);

    PBA += pSM->uLibStartBlock;
    pageNo = PBA * pSM->uPagePerBlock + page;

#ifdef OPT_FIRST_4BLOCKS_ECC4
    if (PBA <= 3)
        sicSMsetBCH(pSM, TRUE);
#endif  // end of OPT_FIRST_4BLOCKS_ECC4

    if (pSM->nPageSize == NAND_PAGE_2KB)
        status = fmiSM_Write_2K(pSM, pageNo, 0, (u32)buff);
    else if (pSM->nPageSize == NAND_PAGE_4KB)
        status = fmiSM_Write_4K(pSM, pageNo, 0, (u32)buff);
    else if (pSM->nPageSize == NAND_PAGE_8KB)
        status = fmiSM_Write_large_page(pSM, pageNo, 0, (u32)buff);
    else    // 512B
        status = fmiSM_Write_512(pSM, pageNo, (u32)buff);

    if (status < 0)
    {
        printk("NAND ERROR: %s(): page write fail for block %d page %d!! Status=0x%x\n", __FUNCTION__, PBA, page, status);
        LEAVE();
        return (status);
    }

#ifdef OPT_FIRST_4BLOCKS_ECC4
    if (PBA <= 3)
        sicSMsetBCH(pSM, FALSE);
#endif  // end of OPT_FIRST_4BLOCKS_ECC4

    LEAVE();
    return (status);
}


int sicSMChangeBadBlockMark(FMI_SM_INFO_T *pSM)
{
    int status=0;
    int chipPort = 0;

    // enable SM
    outpw(REG_FMICR, FMI_SM_EN);
    fmiSM_Initial(pSM);

    /* read physical block 0 - image information */
    if (pSM == pSM0)
        chipPort = 0;
    else
        chipPort = 1;

    status = sicSMpread(chipPort, 0, pSM->uPagePerBlock-1, (u8*)_fmi_gptr1);
    if (status < 0)
        return status;

    /* write specific mark */
    *(_fmi_gptr1+pSM->nPageSize-6) = '5';
    *(_fmi_gptr1+pSM->nPageSize-5) = '5';
    *(_fmi_gptr1+pSM->nPageSize-4) = '0';
    *(_fmi_gptr1+pSM->nPageSize-3) = '0';
    *(_fmi_gptr1+pSM->nPageSize-2) = '9';
    *(_fmi_gptr1+pSM->nPageSize-1) = '1';

#if 1
    // doesn't write "550091" to block-0
    // status = sicSMpwrite(chipPort, 0, pSM->uPagePerBlock-1, (u8*)_fmi_gptr1);
#else
        if (pSM->nPageSize == NAND_PAGE_2KB) {  /* 2KB */
                status = fmiSM_Write_2K(pSM, pSM->uPagePerBlock-1, 0, (u32)_fmi_gptr1);
        } else if (pSM->nPageSize == NAND_PAGE_4KB) {   /* 4KB */
                if (pSM->bIsNandECC4)
                        status = fmiSM_Write_4K_ECC4(pSM, pSM->uPagePerBlock-1, 0, (u32)_fmi_gptr1);
                else
                        status = fmiSM_Write_4K(pSM, pSM->uPagePerBlock-1, 0, (u32)_fmi_gptr1);
    }
#endif
    return status;
}


int fmiSMCheckBootHeader(FMI_SM_INFO_T *pSM)
{
    int chipPort;
    int volatile status, imageCount, i, infoPage, block;
    unsigned int *pImageList = (unsigned int *)_fmi_gptr1;
    volatile int ii;

#define OPT_FOUR_BOOT_IMAGE

    int  fmiNandSysArea = 0;

    memset(_fmi_gptr1, 0xff, 4096);
    infoPage = pSM->uPagePerBlock-1;

    /* read physical block 0 - image information */
    if (pSM == pSM0)
        chipPort = 0;
    else
        chipPort = 1;

#ifdef OPT_FOUR_BOOT_IMAGE
    for (ii=0; ii<4; ii++)
    {
        status = sicSMpread(chipPort, ii, pSM->uPagePerBlock-1, (u8*)_fmi_gptr1);
        if (!status)
        {
            if (((*(pImageList+0)) == 0x574255aa) && ((*(pImageList+3)) == 0x57425963))
                break;
        }
    }
#else
    status = sicSMpread(chipPort, 0, pSM->uPagePerBlock-1, (u8*)_fmi_gptr1);
#endif

    if (status < 0)
        return status;

    /* check specific mark */
    if (pSM->nPageSize != NAND_PAGE_512B)
    {
        if ((*(_fmi_gptr1+pSM->nPageSize-6) == '5') && (*(_fmi_gptr1+pSM->nPageSize-5) == '5') &&
            (*(_fmi_gptr1+pSM->nPageSize-4) == '0') && (*(_fmi_gptr1+pSM->nPageSize-3) == '0') &&
            (*(_fmi_gptr1+pSM->nPageSize-2) == '9') && (*(_fmi_gptr1+pSM->nPageSize-1) == '1'))
        {
            _fmi_bIsNandFirstAccess = 0;
            printk("NAND: Boot ID is found !!!\n");
        }
        else
        {
            //printk("Boot ID NOT found !!!\n");
            sicSMChangeBadBlockMark(pSM);
        }
    }
    printk("NAND: fmiSMCheckBootHeader %d\n", _fmi_bIsNandFirstAccess);

    if (((*(pImageList+0)) == 0x574255aa) && ((*(pImageList+3)) == 0x57425963)) {
        fmiNandSysArea = *(pImageList+1);
    }

    if (fmiNandSysArea != 0xFFFFFFFF && fmiNandSysArea != 0) {
        pSM->uLibStartBlock = (fmiNandSysArea / pSM->uSectorPerBlock) + 1;
    }
    else
    {
        /* read physical block 0 - image information */

#ifdef OPT_FOUR_BOOT_IMAGE
        for (ii=0; ii<4; ii++)
        {
            status = sicSMpread(chipPort, ii, pSM->uPagePerBlock-2, (u8*)_fmi_gptr1);
            if (!status)
            {
                if (((*(pImageList+0)) == 0x574255aa) && ((*(pImageList+3)) == 0x57425963))
                    break;
            }
        }

#else
        status = sicSMpread(chipPort, 0, pSM->uPagePerBlock-2, (u8*)_fmi_gptr1);
#endif

        if (status < 0)
            return status;

        if (((*(pImageList+0)) == 0x574255aa) && ((*(pImageList+3)) == 0x57425963))
        {
            imageCount = *(pImageList+1);

            // pointer to image information
            pImageList = pImageList+4;
            for (i=0; i<imageCount; i++)
            {
                block = (*(pImageList + 1) & 0xFFFF0000) >> 16;
                if (block > pSM->uLibStartBlock)
                    pSM->uLibStartBlock = block;

                // pointer to next image
                pImageList = pImageList+12; //### 2010.10.01 KC Modified
            }
            pSM->uLibStartBlock++;
        }
    }
    LEAVE();
    return 0;
}


int fmiCheckInvalidBlock(FMI_SM_INFO_T *pSM, u32 BlockNo)
{
    int chipPort;
    unsigned int volatile logical_block, physical_block;
    unsigned char volatile byte0=0xFF, byte5=0xFF;

    physical_block = BlockNo;
    logical_block = physical_block - pSM->uLibStartBlock;

    if (pSM == pSM0)
        chipPort = 0;
    else
        chipPort = 1;

    // NAND flash guarantee block 0 is good block.
    if (physical_block == 0)
        return NAND_GOOD_BLOCK;

    sicSMpread(chipPort, logical_block, 0, _fmi_pNANDBuffer2);
    byte0 = (inpw(REG_SMRA_0) & 0x000000FF);
    byte5 = (inpw(REG_SMRA_1) & 0x0000FF00) >> 8;

    if (pSM->nPageSize == NAND_PAGE_512B)
    {
        if ((byte0 == 0xFF) && (byte5 == 0xFF))
        {
            fmiSM_Reset(pSM);
            sicSMpread(chipPort, logical_block, 1, _fmi_pNANDBuffer2);
            byte0 = inpw(REG_SMRA_0) & 0x000000FF;
            byte5 = (inpw(REG_SMRA_1) & 0x0000FF00) >> 8;
            if ((byte0 != 0xFF) || (byte5 != 0xFF))
            {
                fmiSM_Reset(pSM);
                return NAND_BAD_BLOCK;
            }
        }
        else
        {
            fmiSM_Reset(pSM);
            return NAND_BAD_BLOCK;
        }
    }
    else
    {
        if (byte0 == 0xFF)
        {
            fmiSM_Reset(pSM);
            if (pSM->bIsMLCNand == TRUE)
                // read last page with ECC for MLC NAND flash
                sicSMpread(chipPort, logical_block, pSM->uPagePerBlock - 1, _fmi_pNANDBuffer2);
            else
                // read page 1 with ECC for SLC NAND flash
                sicSMpread(chipPort, logical_block, 1, _fmi_pNANDBuffer2);
            byte0 = inpw(REG_SMRA_0) & 0x000000FF;
            if (byte0 != 0xFF)
            {
                fmiSM_Reset(pSM);
                return NAND_BAD_BLOCK;
            }
        }
        else
        {
            fmiSM_Reset(pSM);
            return NAND_BAD_BLOCK;
        }
    }

    fmiSM_Reset(pSM);
    return NAND_GOOD_BLOCK;
}


int fmiNormalCheckBlock(FMI_SM_INFO_T *pSM, u32 BlockNo)
{
    return fmiCheckInvalidBlock(pSM, BlockNo);
}

#ifdef OPT_MARK_BAD_BLOCK_WHILE_ERASE_FAIL

    static int sicSMMarkBadBlock_WhileEraseFail(FMI_SM_INFO_T *pSM, u32 BlockNo)
    {
        u32 uSector, ucColAddr = 0;

        /* check if MLC NAND */
        if (pSM->bIsMLCNand == TRUE)
        {
            uSector = (BlockNo+1) * pSM->uPagePerBlock - 1; // write last page
            ucColAddr = pSM->nPageSize;     // point to first byte of redundancy area

            // send command
            outpw(REG_SMCMD, 0x80);                     // serial data input command
            outpw(REG_SMADDR, ucColAddr);               // CA0 - CA7
            outpw(REG_SMADDR, (ucColAddr >> 8) & 0xff); // CA8 - CA11
            outpw(REG_SMADDR, uSector & 0xff);          // PA0 - PA7
            if (!pSM->bIsMulticycle)
                outpw(REG_SMADDR, ((uSector >> 8) & 0xff)|EOA_SM);  // PA8 - PA15
            else
            {
                outpw(REG_SMADDR, (uSector >> 8) & 0xff);           // PA8 - PA15
                outpw(REG_SMADDR, ((uSector >> 16) & 0xff)|EOA_SM); // PA16 - PA17
            }
            outpw(REG_SMDATA, 0xf0);    // mark bad block (use 0xf0 instead of 0x00 to differ from Old (Factory) Bad Blcok Mark)
            outpw(REG_SMCMD, 0x10);

            if (! fmiSMCheckRB(pSM))
                return FMI_SM_RB_ERR;

            fmiSM_Reset(pSM);
            return 0;
        }
        /* SLC check the 2048 byte of 1st or 2nd page per block */
        else    // SLC
        {
            uSector = BlockNo * pSM->uPagePerBlock;     // write lst page
            if (pSM->nPageSize == NAND_PAGE_512B)
            {
                ucColAddr = 0;                  // keep 0 for 512B page NAND to access redundancy area
                goto _mark_512;
            }
            else
                ucColAddr = pSM->nPageSize;     // point to first byte of redundancy area

            // send command
            outpw(REG_SMCMD, 0x80);                     // serial data input command
            outpw(REG_SMADDR, ucColAddr);               // CA0 - CA7
            outpw(REG_SMADDR, (ucColAddr >> 8) & 0xff); // CA8 - CA11
            outpw(REG_SMADDR, uSector & 0xff);          // PA0 - PA7
            if (!pSM->bIsMulticycle)
                outpw(REG_SMADDR, ((uSector >> 8) & 0xff)|EOA_SM);  // PA8 - PA15
            else
            {
                outpw(REG_SMADDR, (uSector >> 8) & 0xff);           // PA8 - PA15
                outpw(REG_SMADDR, ((uSector >> 16) & 0xff)|EOA_SM); // PA16 - PA17
            }
            outpw(REG_SMDATA, 0xf0);    // mark bad block (use 0xf0 instead of 0x00 to differ from Old (Factory) Bad Blcok Mark)
            outpw(REG_SMCMD, 0x10);

            if (! fmiSMCheckRB(pSM))
                return FMI_SM_RB_ERR;

            fmiSM_Reset(pSM);
            return 0;

    _mark_512:

            outpw(REG_SMCMD, 0x50);             // point to redundant area
            outpw(REG_SMCMD, 0x80);             // serial data input command
            outpw(REG_SMADDR, ucColAddr);       // CA0 - CA7
            outpw(REG_SMADDR, uSector & 0xff);  // PA0 - PA7
            if (!pSM->bIsMulticycle)
                outpw(REG_SMADDR, ((uSector >> 8) & 0xff)|EOA_SM);  // PA8 - PA15
            else
            {
                outpw(REG_SMADDR, (uSector >> 8) & 0xff);           // PA8 - PA15
                outpw(REG_SMADDR, ((uSector >> 16) & 0xff)|EOA_SM); // PA16 - PA17
            }

            outpw(REG_SMDATA, 0xf0);    // 512
            outpw(REG_SMDATA, 0xff);
            outpw(REG_SMDATA, 0xff);
            outpw(REG_SMDATA, 0xff);
            outpw(REG_SMDATA, 0xf0);    // 516
            outpw(REG_SMDATA, 0xf0);    // 517
            outpw(REG_SMCMD, 0x10);
            if (! fmiSMCheckRB(pSM))
                return FMI_SM_RB_ERR;

            fmiSM_Reset(pSM);
            return 0;
        }
    }

#endif      // OPT_MARK_BAD_BLOCK_WHILE_ERASE_FAIL

#if 0
    int sicSMMarkBadBlock(FMI_SM_INFO_T *pSM, u32 BlockNo)
    {
        u32 sector, column;

        /* page 0 */
        sector = BlockNo * pSM->uPagePerBlock;
        column = 512;

        // send command
        outpw(REG_SMCMD, 0x80);                     // serial data input command
        outpw(REG_SMADDR, column);                  // CA0 - CA7
        outpw(REG_SMADDR, (column >> 8) & 0xff);    // CA8 - CA12
        outpw(REG_SMADDR, sector & 0xff);           // PA0 - PA7
        if (!pSM->bIsMulticycle)
            outpw(REG_SMADDR, ((sector >> 8) & 0xff)|EOA_SM);   // PA8 - PA15
        else
        {
            outpw(REG_SMADDR, (sector >> 8) & 0xff);            // PA8 - PA15
            outpw(REG_SMADDR, ((sector >> 16) & 0xff)|EOA_SM);  // PA16 - PA17
        }

        outpw(REG_SMDATA, 0xf0);    // 512
        outpw(REG_SMDATA, 0xff);
        outpw(REG_SMDATA, 0xff);
        outpw(REG_SMDATA, 0xff);
        outpw(REG_SMDATA, 0xf0);    // 516
        outpw(REG_SMDATA, 0xf0);    // 517
        outpw(REG_SMCMD, 0x10);
        if (! fmiSMCheckRB(pSM))
            return FMI_SM_RB_ERR;
        fmiSM_Reset(pSM);

        /* page 1 */
        sector++;
        // send command
        outpw(REG_SMCMD, 0x80);                     // serial data input command
        outpw(REG_SMADDR, column);                  // CA0 - CA7
        outpw(REG_SMADDR, (column >> 8) & 0xff);    // CA8 - CA12
        outpw(REG_SMADDR, sector & 0xff);           // PA0 - PA7
        if (!pSM->bIsMulticycle)
            outpw(REG_SMADDR, ((sector >> 8) & 0xff)|EOA_SM);   // PA8 - PA15
        else
        {
            outpw(REG_SMADDR, (sector >> 8) & 0xff);            // PA8 - PA15
            outpw(REG_SMADDR, ((sector >> 16) & 0xff)|EOA_SM);  // PA16 - PA17
        }

        outpw(REG_SMDATA, 0xf0);    // 512
        outpw(REG_SMDATA, 0xff);
        outpw(REG_SMDATA, 0xff);
        outpw(REG_SMDATA, 0xff);
        outpw(REG_SMDATA, 0xf0);    // 516
        outpw(REG_SMDATA, 0xf0);    // 517
        outpw(REG_SMCMD, 0x10);
        if (! fmiSMCheckRB(pSM))
            return FMI_SM_RB_ERR;
        fmiSM_Reset(pSM);

        return 0;
    }
#endif


static int sicSMInit(int NandPort, NDISK_T *NDISK_info)
{
    int status=0, count;

    ENTER();

    outpw(REG_DMACCSR, inpw(REG_DMACCSR) | DMAC_EN);
    outpw(REG_DMACCSR, inpw(REG_DMACCSR) | DMAC_SWRST);
    while(inpw(REG_DMACCSR) & DMAC_SWRST);
    outpw(REG_FMICR, FMI_SM_EN);

    // if (down_interruptible(&fmi_sem))
    //        return GNERR_IO_ERR;

    if ((_nand_init0 == 0) && (_nand_init1 == 0))
    {
        // enable SM
        /* select NAND control pin used */
        outpw(REG_SMTCR, 0x20305);
        outpw(REG_SMCSR, (inpw(REG_SMCSR) & ~SMCR_PSIZE) | PSIZE_512);
        outpw(REG_SMCSR, inpw(REG_SMCSR) |  SMCR_ECC_3B_PROTECT);
        outpw(REG_SMCSR, inpw(REG_SMCSR) | SMCR_ECC_CHK);

        /* init SM interface */
        outpw(REG_SMCSR, inpw(REG_SMCSR) | SMCR_REDUN_AUTO_WEN);
    }

    sicSMselect(NandPort);
    mdelay(1);

    if (NandPort == 0) {
        if (_nand_init0)
            return 0;

        pSM0 = kmalloc(sizeof(FMI_SM_INFO_T),GFP_KERNEL);
        if (pSM0 == NULL)
            return FMI_SM_NO_MEMORY;

        memset((char *)pSM0, 0, sizeof(FMI_SM_INFO_T));

        if ((status = fmiSM_ReadID(pSM0, NDISK_info)) < 0) {
            if (pSM0 != NULL) {
                kfree(pSM0);
                pSM0 = 0;
            }
            return status;
        }
        fmiSM_Initial(pSM0);

        // check NAND boot header
        fmiSMCheckBootHeader(pSM0);

        while (1) {
            if (!fmiNormalCheckBlock(pSM0, pSM0->uLibStartBlock))
                break;
            else
                pSM0->uLibStartBlock++;
        }
        if (pSM0->bIsCheckECC) {
            if (pSM0->uLibStartBlock == 0)
                pSM0->uLibStartBlock++;
        }
        printk("Nand0: uLibStartBlock=%d\n",pSM0->uLibStartBlock);
        NDISK_info->nStartBlock = pSM0->uLibStartBlock;     /* available start block */
        pSM0->uBlockPerFlash -= pSM0->uLibStartBlock;
        count = NDISK_info->nBlockPerZone * 2 / 100 + NAND_RESERVED_BLOCK;

        NDISK_info->nBlockPerZone = (NDISK_info->nBlockPerZone * NDISK_info->nZone - NDISK_info->nStartBlock) / NDISK_info->nZone;
        NDISK_info->nLBPerZone = NDISK_info->nBlockPerZone - count;
        NDISK_info->nNandNo = NandPort;

        // default to disable Region Protect by set to block 0 and page 0
        sicSMRegionProtect(NandPort, 0, 0);

        _nand_init0 = 1;
    }
    else if (NandPort == 1) {
        if (_nand_init1)
            return 0;

        pSM1 = kmalloc(sizeof(FMI_SM_INFO_T),GFP_KERNEL);
        memset((char *)pSM1, 0, sizeof(FMI_SM_INFO_T));

        if ((status = fmiSM_ReadID(pSM1, NDISK_info)) < 0) {
            if (pSM1 != NULL) {
                kfree(pSM1);
                pSM1 = 0;
            }
            return status;
        }
        fmiSM_Initial(pSM1);

        // check NAND boot header
        fmiSMCheckBootHeader(pSM1);

        while (1) {
            if (!fmiNormalCheckBlock(pSM1, pSM1->uLibStartBlock))
                break;
            else
                pSM1->uLibStartBlock++;
    #if defined(CONFIG_W55FA92_NAND_BOTH) || defined(CONFIG_W55FA92_NAND1)
            if (nand_card_status() == 1)    // nand card removed now
            {
                printk("NAND Warning: Nand Card initial fail since card had beed removed.\n");
                if (pSM1 != NULL)
                {
                    kfree(pSM1);
                    pSM1 = 0;
                }
                LEAVE();
                return FMI_SM_INIT_ERROR;
            }
    #endif
        }
        if (pSM1->bIsCheckECC) {
            if (pSM1->uLibStartBlock == 0)
                pSM1->uLibStartBlock++;
        }
        printk("Nand1: uLibStartBlock=%d\n",pSM1->uLibStartBlock);
        NDISK_info->nStartBlock = pSM1->uLibStartBlock;     /* available start block */
        pSM1->uBlockPerFlash -= pSM1->uLibStartBlock;
        count = NDISK_info->nBlockPerZone * 2 / 100 + NAND_RESERVED_BLOCK;

        //up(&fmi_sem);
        NDISK_info->nBlockPerZone = (NDISK_info->nBlockPerZone * NDISK_info->nZone - NDISK_info->nStartBlock) / NDISK_info->nZone;
        NDISK_info->nLBPerZone = NDISK_info->nBlockPerZone - count;
        NDISK_info->nNandNo = NandPort;

        // default to disable Region Protect by set to block 0 and page 0
        sicSMRegionProtect(NandPort, 0, 0);

        _nand_init1 = 1;
    }
    LEAVE();
    return 0;
}


/* Return the number of bit 1 within integer n */
static int countBit1(int n)
{
    int count = 0;
    while(n)
    {
        count += (n & 1);
        n >>= 1;
    }
    return count;
}

static int sicSM_is_page_dirty(int NandPort, int PBA, int page)
{
    unsigned char volatile byte2=0xFF, byte3=0xFF;

    ENTER();

    /* read page with ECC */
    sicSMpread(NandPort, PBA, page, _fmi_pNANDBuffer2);
    byte2 = (inpw(REG_SMRA_0) & 0x00FF0000) >> 16;
    byte3 = (inpw(REG_SMRA_0) & 0xFF000000) >> 24;

    /* If bit 1 count value of byte 2 and byte 3 is greater than 8,
       NAND controller will treat this page as none used page (clean page);
       otherwise, it¡¦s used (dirty page). */
    if (countBit1(byte2) + countBit1(byte3) > 8)
        return NAND_CLEAN_PAGE;
    else
        return NAND_DIRTY_PAGE;
}


static int sicSM_is_valid_block(int NandPort, int PBA)
{
    FMI_SM_INFO_T *pSM;

    ENTER();

    // enable SM
    outpw(REG_FMICR, FMI_SM_EN);

    sicSMselect(NandPort);
    if (NandPort == 0)
        pSM = pSM0;
    else
        pSM = pSM1;

    PBA += pSM->uLibStartBlock;

    // enable SM
    fmiSM_Initial(pSM);

    //  PBA += pSM->uLibStartBlock;
    if (fmiCheckInvalidBlock(pSM, PBA) == 1)    // invalid block
        return 0;
    else
        return 1;   // valid block
}


static int sicSMblock_erase(int NandPort, int PBA)
{
    FMI_SM_INFO_T *pSM;
    u32 page_no;

    ENTER();

    // enable SM
    outpw(REG_FMICR, FMI_SM_EN);

    sicSMselect(NandPort);
    if (NandPort == 0)
        pSM = pSM0;
    else
        pSM = pSM1;

    fmiSM_Initial(pSM);

    PBA += pSM->uLibStartBlock;
    if (fmiCheckInvalidBlock(pSM, PBA) != 1) {
            page_no = PBA * pSM->uPagePerBlock;     // get page address

            //fmiSM_Reset(pSM);

            /* clear R/B flag */
    #if defined(CONFIG_W55FA92_TWO_RB_PINS) || defined(CONFIG_W55FA92_NAND1)
            if(pSM == pSM0)
            {
                while (!(inpw(REG_SMISR) & SMISR_RB0));
                outpw(REG_SMISR, SMISR_RB0_IF);
            }
            else
            {
                while (!(inpw(REG_SMISR) & SMISR_RB1));
                outpw(REG_SMISR, SMISR_RB1_IF);
            }
    #else

            while (!(inpw(REG_SMISR) & SMISR_RB0));
            outpw(REG_SMISR, SMISR_RB0_IF);
    #endif

            if (inpw(REG_SMISR) & SMISR_ECC_FIELD_IF)   /* ECC_FLD_IF */
            {
                // PDEBUG("erase: error sector !!\n");
                outpw(REG_SMISR, SMISR_ECC_FIELD_IF);
            }

            outpw(REG_SMCMD, 0x60);     // erase setup command

            outpw(REG_SMADDR, (page_no & 0xff));        // PA0 - PA7
            if (!pSM->bIsMulticycle)
                    outpw(REG_SMADDR, ((page_no >> 8) & 0xff)|EOA_SM);  // PA8 - PA15
            else {
                    outpw(REG_SMADDR, ((page_no >> 8) & 0xff));         // PA8 - PA15
                    outpw(REG_SMADDR, ((page_no >> 16) & 0xff)|EOA_SM); // PA16 - PA17
            }

            outpw(REG_SMCMD, 0xd0);     // erase command
            if (!fmiSMCheckRB(pSM))
                    return FMI_SM_RB_ERR;

            if (fmiSMCheckStatus(pSM) != 0) {
    #ifdef OPT_MARK_BAD_BLOCK_WHILE_ERASE_FAIL
                    sicSMMarkBadBlock_WhileEraseFail(pSM,PBA);
    #endif
                    return FMI_SM_STATUS_ERR;
            }
    } else
            return FMI_SM_INVALID_BLOCK;

    LEAVE();
    return 0;
}


static int sicSMchip_erase(int NandPort)
{
    int i, status=0;
    FMI_SM_INFO_T *pSM;

    ENTER();

    // enable SM
    outpw(REG_FMICR, FMI_SM_EN);

    sicSMselect(NandPort);
    if (NandPort == 0)
        pSM = pSM0;
    else
        pSM = pSM1;

    fmiSM_Initial(pSM);

    // erase all chip
    for (i=0; i<=pSM->uBlockPerFlash; i++) {
        status = sicSMblock_erase(NandPort, i);
        if (status < 0)
            printk("NAND ERROR: SM block erase fail <%d>!! Status = 0x%x\n", i, status);
    }

    LEAVE();
    return 0;
}

/* driver function */
int nandInit0(NDISK_T *NDISK_info)
{
    ENTER();
    return (sicSMInit(0, NDISK_info));
}

int nandpread0(int PBA, int page, u8 *buff)
{
    ENTER();
    return (sicSMpread(0, PBA, page, buff));
}

int nandpwrite0(int PBA, int page, u8 *buff)
{
    ENTER();
    return (sicSMpwrite(0, PBA, page, buff));
}

int nand_is_page_dirty0(int PBA, int page)
{
    ENTER();
    return (sicSM_is_page_dirty(0, PBA, page));
}

int nand_is_valid_block0(int PBA)
{
    ENTER();
    return (sicSM_is_valid_block(0, PBA));
}

int nand_block_erase0(int PBA)
{
    ENTER();
    return (sicSMblock_erase(0, PBA));
}

int nand_chip_erase0(void)
{
    ENTER();
    return (sicSMchip_erase(0));
}


//u8 ioctl_buf[8192];     // The data buffer for IOCTL operation.
u8 ioctl_buf[8192] __attribute__((aligned (32)));
/*-----------------------------------------------------------------------------
 * 2013/11/26, support IOCTL for NAND port 0.
 * Input:
 *      cmd: the command for ioctl. Define at NandDrv.h
 *      arg: the pointer to FMI_IOCTL_INFO_T.
 *---------------------------------------------------------------------------*/
int nand_ioctl_0(int cmd, int arg, int param3, int param4)
{
    FMI_IOCTL_INFO_T *pArg = (FMI_IOCTL_INFO_T *)arg;
    int result;

    switch (cmd)
    {
        case NAND_IOC_GET_CHIP_INFO:
            copy_to_user(pArg->pSM, pSM0, sizeof(FMI_SM_INFO_T));
            break;
        case NAND_IOC_IO_BLOCK_ERASE:
            nand_block_erase0(pArg->uBlock - pSM0->uLibStartBlock);
            break;
        case NAND_IOC_IO_PAGE_READ:
            nandpread0(pArg->uBlock - pSM0->uLibStartBlock, pArg->uPage, ioctl_buf);
            copy_to_user(pArg->pData, ioctl_buf, pSM0->nPageSize);
            break;
        case NAND_IOC_IO_PAGE_WRITE:
            write_nandloader = pArg->uWrite_nandloader; // write "check marker" or not
            if (access_ok(VERIFY_READ, pArg->pData, pSM0->nPageSize))
            {
                result = copy_from_user(ioctl_buf, pArg->pData, pSM0->nPageSize);
                if (result != 0)
                    printk("ERROR: nand_ioctl_0: copy_from_user() fail, return %d, block %d page %d\n", result, pArg->uBlock, pArg->uPage);
            }
            else
                printk("ERROR: nand_ioctl_0: access_ok() fail, block %d page %d\n", pArg->uBlock, pArg->uPage);
            result = nandpwrite0(pArg->uBlock - pSM0->uLibStartBlock, pArg->uPage, ioctl_buf);
            write_nandloader = 0;   // MUST set back to 0 for normal page writing (no "check marker")
            break;
        default:
            printk("Warning SCSI/NAND: Unknown command to SCSI/NAND IOCTL 0x%X.\n", cmd);
            break;
    }
    return 0;
}

int nandInit1(NDISK_T *NDISK_info)
{
    ENTER();
    return (sicSMInit(1, NDISK_info));
}

int nandpread1(int PBA, int page, u8 *buff)
{
    ENTER();
    return (sicSMpread(1, PBA, page, buff));
}

int nandpwrite1(int PBA, int page, u8 *buff)
{
    ENTER();
    return (sicSMpwrite(1, PBA, page, buff));
}

int nand_is_page_dirty1(int PBA, int page)
{
    ENTER();
    return (sicSM_is_page_dirty(1, PBA, page));
}

int nand_is_valid_block1(int PBA)
{
    ENTER();
    return (sicSM_is_valid_block(1, PBA));
}

int nand_block_erase1(int PBA)
{
    ENTER();
    return (sicSMblock_erase(1, PBA));
}

int nand_chip_erase1(void)
{
    ENTER();
    return (sicSMchip_erase(1));
}

int nand_ioctl_1(int param1, int param2, int param3, int param4)
{
    ENTER();
    return 0;
}

void fmiSMClose(int NandPort)
{
    ENTER();
    if (NandPort == 0)
    {
        _nand_init0 = 0;
        if (pSM0 != 0) {
            kfree(pSM0);
            pSM0 = 0;
        }
    }
    else
    {
        _nand_init1 = 0;
        if (pSM1 != 0) {
            kfree(pSM1);
            pSM1 = 0;
        }
    }

    if ((_nand_init0 == 0) && (_nand_init1 == 0)) {
        printk("NAND: Nand close !!!\n");
        outpw(REG_FMICR, FMI_SM_EN);
        outpw(REG_SMCSR, inpw(REG_SMCSR)|0x06000000);
        outpw(REG_SMISR, 0xfff);
        outpw(REG_FMICR, 0x00);
        outpw(REG_GPDFUN0, inpw(REG_GPDFUN0) & (~0xFFF00000));  // disable NRE/RB0/RB1 pins
        outpw(REG_GPDFUN1, inpw(REG_GPDFUN1) & (~0x0000000F));  // disable NWR pins
        outpw(REG_GPEFUN1, inpw(REG_GPEFUN1) & (~0x000FFFFF));  // disable CS0/ALE/CLE/ND3/CS1 pins
    }
}

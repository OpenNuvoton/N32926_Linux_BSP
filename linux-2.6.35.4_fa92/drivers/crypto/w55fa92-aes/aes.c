/***************************************************************************
 * Copyright (c) 2011 Nuvoton Technology. All rights reserved.
 *
 * FILENAME
 *     aes.c
 * DESCRIPTION
 *     The library for AES.
 * FUNCTIONS
 *     None
 **************************************************************************/
#include "aes.h"
#include <linux/string.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <mach/w55fa92_reg.h>

//--- Define AES Opearation mode
#define AES_ENCRYPT     0
#define AES_DECRYPT     1

/*
//--- Define for Endian Conversion
#define AES_BIG_ENDIAN      1
#define AES_LITTLE_ENDIAN   0
#define BYTE_ORDER          AES_LITTLE_ENDIAN

#if (BYTE_ORDER == AES_BIG_ENDIAN)
    #define  htonl(l)  l
    #define  htons(s)  s
    #define  ntohl(l)  l
    #define  ntohs(s)  s
#endif

#if (BYTE_ORDER == AES_LITTLE_ENDIAN)
    #define  htonl(l)  (unsigned long)((((unsigned long)(l) >> 24) & 0x000000FF) | \
                                       (((unsigned long)(l) >>  8) & 0x0000FF00) | \
                                       (((unsigned long)(l) <<  8) & 0x00FF0000) | \
                                       (((unsigned long)(l) << 24) & 0xFF000000) )
    #define  htons(s)  ((unsigned short)(((unsigned short)(s) >> 8) | ((unsigned short)(s) << 8)))
    #define  ntohl(l)  (unsigned long)((((unsigned long)(l) >> 24) & 0x000000FF) | \
                                       (((unsigned long)(l) >>  8) & 0x0000FF00) | \
                                       (((unsigned long)(l) <<  8) & 0x00FF0000) | \
                                       (((unsigned long)(l) << 24) & 0xFF000000) )
    #define  ntohs(s)  htons(s)
#endif
*/

/*-----------------------------------------------------------------------------
 * Define Global Variables
 *---------------------------------------------------------------------------*/
static BOOL volatile _aes_bIsAESOK=FALSE, _aes_bIsBusError=FALSE;      // for interrupt flag in interrupt handler
static struct clk *clk_hclk3 = NULL;
static struct clk *clk_ipsec = NULL;

/*-----------------------------------------------------------------------------
 * Interrupt handler for AES interrupt.
 * OUTPUT:
 *      _aes_bIsAESOK:    TRUE indicate AESOK interrupt happen.
 *      _aes_bIsBusError: TRUE indicate BERR  interrupt happen.
 *---------------------------------------------------------------------------*/
VOID AES_Int_Handler()
{
    UINT32 volatile isr;
    UINT32 volatile ier;

    ier = inpw(REG_AESIER);
    isr = inpw(REG_AESISR);

    if ((ier & ENAESOK) && (isr & AESOK))
    {
        _aes_bIsAESOK = TRUE;
        outpw(REG_AESISR, AESOK);   // clear interrupt flag
    }

    if ((ier & ENBERR) && (isr & BERR))
    {
        _aes_bIsBusError = TRUE;
        outpw(REG_AESISR, BERR);    // clear interrupt flag
    }
}


/*-----------------------------------------------------------------------------
 * Clear interrupt flag in AES register.
 *---------------------------------------------------------------------------*/
VOID AES_Clear_Interrupt_Flag(VOID)
{
    _aes_bIsAESOK = FALSE;
    outpw(REG_AESISR, AESOK);   // clear interrupt flag

    _aes_bIsBusError = FALSE;
    outpw(REG_AESISR, BERR);    // clear interrupt flag
}


/*-----------------------------------------------------------------------------
 * Check the Bus Error status
 * RETURN:
 *      0:  without bus error now
 *      AES_ERR_BUS_ERROR: with bus error
 *---------------------------------------------------------------------------*/
int AES_Check_Bus_Error(VOID)
{
    int result = 0;

    //--- check Bus Error status
    if (inpw(REG_AESIER) & ENBERR)
    {
        // interrupt enable, check bus error by _aes_bIsBusError that update by interrupt handler
        if (_aes_bIsBusError)
        {
            printk(KERN_ERR "ERROR in AES_Check_Bus_Error(): Interrupt Handler inform that Bus Error !!\n");
            _aes_bIsBusError = FALSE;
            result = AES_ERR_BUS_ERROR;
        }
    }
    else
    {
        // interrupt disable, check bus error by polling REG_AESISR
        if (inpw(REG_AESISR) & BERR)
        {
            printk(KERN_ERR "ERROR in AES_Check_Bus_Error(): Polling REG_AESISR and found Bus Error !!\n");
            outpw(REG_AESISR, BERR);    // clear interrupt flag
            result = AES_ERR_BUS_ERROR;
        }
    }

    if (result == AES_ERR_BUS_ERROR)
    {
        printk(KERN_ERR "    AESCR = 0x%08x\n", inpw(REG_AESCR));
        printk(KERN_ERR "    AESSAR = 0x%08x\n", inpw(REG_AESSAR));
        printk(KERN_ERR "    AESDAR = 0x%08x\n", inpw(REG_AESDAR));
        printk(KERN_ERR "    AESBCR = 0x%08x\n", inpw(REG_AESBCR));
        printk(KERN_ERR "    ACSAR = 0x%08x\n", inpw(REG_ACSAR));
        printk(KERN_ERR "    ACDAR = 0x%08x\n", inpw(REG_ACDAR));
        printk(KERN_ERR "    ACBCR = 0x%08x\n", inpw(REG_ACBCR));
    }
    return result;
}


/*-----------------------------------------------------------------------------
 * Encrypt or Decrypt text by AES algorithm that support by FA95 with length limitation.
 * INPUT:
 *      UINT8*  input_buf,      // pointer to source address
 *      UINT8*  output_buf,     // pointer to destination address
 *      UINT32  input_len,      // length of source data. MUST be divisible by 16 and <= AES_MAX_BCNT (65520)
 *      UINT8*  iv,             // pointer to initial vector
 *      UINT8*  key,            // pointer to cipher key
 *      KEYSIZE key_size        // lenght of cipher key
 *      INT     isDecrypt       // AES mode: encrypt or decrypt
 * OUTPUT:
 *      Put the result of AES operation to buffer that output_buf pointed.
 * RETURN:
 *      0:  OK
 *      error code: FAIL
 *---------------------------------------------------------------------------*/
int Do_AES_Async(UINT8* input_buf, UINT8* output_buf, UINT32 input_len, UINT8* iv,
           UINT8* key, KEYSIZE key_size, INT isDecrypt)
{
    //--- check input_len
    if((input_len % 16 != 0) || (input_len > AES_MAX_BCNT))
    {
        printk(KERN_ERR "ERROR in Do_AES(): input length MUST be divisible by 16 and <= %d. Wrong length %d !!\n", AES_MAX_BCNT, input_len);
        return AES_ERR_DATA_LEN;
    }

    //--- check input buffer
    if (input_buf == NULL)
    {
        printk(KERN_ERR "ERROR in Do_AES(): must specify input buffer !!\n");
        return AES_ERR_DATA_BUF;
    }

    //--- check where is the destination buffer
    // FA95 AES engine allow output and input share same buffer for one AES operation with length <= 65520
    if (output_buf == NULL)
    {
        output_buf = input_buf;     // output and input share same buffer
    }

    //--- fill AES Source Address register
    outpw(REG_AESSAR, (UINT32)input_buf);
    //--- fill AES Destination Address register
    outpw(REG_AESDAR, (UINT32)output_buf);
    //--- fill AES Byte Count register
    outpw(REG_AESBCR, input_len);

    //--- fill AES KEY register
    if (key == NULL)
    {
        printk(KERN_ERR "ERROR in Do_AES(): key not found !!\n");
        return AES_ERR_CIPHER_KEY;
    }

    //--- fill register for cipher key length
    outpw(REG_AESCR, inpw(REG_AESCR) & ~KSIZE);    // clear to 0 for 2 bits field
    switch (key_size)
    {
        case KEY_128:
            outpw(REG_AESKW0R, htonl(*(UINT32*)key));
            outpw(REG_AESKW1R, htonl(*(UINT32*)&key[4]));
            outpw(REG_AESKW2R, htonl(*(UINT32*)&key[8]));
            outpw(REG_AESKW3R, htonl(*(UINT32*)&key[12]));
            outpw(REG_AESCR, inpw(REG_AESCR) | KSIZE_128);
            break;
        case KEY_192:
            outpw(REG_AESKW0R, htonl(*(UINT32*)key));
            outpw(REG_AESKW1R, htonl(*(UINT32*)&key[4]));
            outpw(REG_AESKW2R, htonl(*(UINT32*)&key[8]));
            outpw(REG_AESKW3R, htonl(*(UINT32*)&key[12]));
            outpw(REG_AESKW4R, htonl(*(UINT32*)&key[16]));
            outpw(REG_AESKW5R, htonl(*(UINT32*)&key[20]));
            outpw(REG_AESCR, inpw(REG_AESCR) | KSIZE_192);
            break;
        case KEY_256:
            outpw(REG_AESKW0R, htonl(*(UINT32*)key));
            outpw(REG_AESKW1R, htonl(*(UINT32*)&key[4]));
            outpw(REG_AESKW2R, htonl(*(UINT32*)&key[8]));
            outpw(REG_AESKW3R, htonl(*(UINT32*)&key[12]));
            outpw(REG_AESKW4R, htonl(*(UINT32*)&key[16]));
            outpw(REG_AESKW5R, htonl(*(UINT32*)&key[20]));
            outpw(REG_AESKW6R, htonl(*(UINT32*)&key[24]));
            outpw(REG_AESKW7R, htonl(*(UINT32*)&key[28]));
            outpw(REG_AESCR, inpw(REG_AESCR) | KSIZE_256);
            break;
        default:
            printk(KERN_ERR "ERROR in Do_AES(): wrong key size !!\n");
            return AES_ERR_CIPHER_KEY;
    }

    //--- fill AES Initial Vector register
    if (iv != NULL)
    {
        outpw(REG_AESIV0R, htonl(*(UINT32*)iv));
        outpw(REG_AESIV1R, htonl(*(UINT32*)&iv[4]));
        outpw(REG_AESIV2R, htonl(*(UINT32*)&iv[8]));
        outpw(REG_AESIV3R, htonl(*(UINT32*)&iv[12]));
    }
    else
    {
        printk(KERN_ERR "ERROR in Do_AES(): inital vector not found\n");
        return AES_ERR_IV;
    }

    //--- fill register for AES operation mode
    if (isDecrypt == AES_ENCRYPT)
        outpw(REG_AESCR, inpw(REG_AESCR) | ENCRPT);
    else if (isDecrypt == AES_DECRYPT)
        outpw(REG_AESCR, inpw(REG_AESCR) & ~ENCRPT);
    else
    {
        printk(KERN_ERR "ERROR in Do_AES(): wrong AES Operation mode !!\n");
        return AES_ERR_MODE;  // wrong AES operation mode
    }

    //--- begin to do AES operation
    AES_Clear_Interrupt_Flag();
    outpw(REG_AESCR, inpw(REG_AESCR) | AESON);

    // AES is running now, call flush to wait for its finish.
    return AES_ERR_RUNNING;
}
int Do_AES(UINT8* input_buf, UINT8* output_buf, UINT32 input_len, UINT8* iv,
           UINT8* key, KEYSIZE key_size, INT isDecrypt)
{
	int errcode = Do_AES_Async (input_buf, output_buf, input_len, iv, key, key_size, isDecrypt);
	if (errcode == AES_ERR_RUNNING) {
		errcode = AES_Flush ();
	}
	
	return errcode;
}
	
/*-----------------------------------------------------------------------------
 * Encrypt plain text by AES algorithm that support by FA95 with any length.
 * INPUT:
 *      UINT8*  plain_buf,      // pointer to plain text buffer
 *      UINT8*  cipher_buf,     // pointer to cipher text buffer.
 *      UINT32  data_len,       // length of data. MUST be divisible by 16
 *      UINT8*  iv,             // pointer to initial vector
 *      UINT8*  key,            // pointer to cipher key
 *      KEYSIZE key_size        // lenght of cipher key
 * OUTPUT:
 *      Put the cipher text to buffer that output_buf pointed.
 * RETURN:
 *      same to Do_AES()
 *---------------------------------------------------------------------------*/
int AES_Encrypt_Async(UINT8* plain_buf, UINT8* cipher_buf, UINT32 data_len, UINT8* iv,
                UINT8* key, KEYSIZE key_size)
{
    int result;
    UINT8 next_iv[16];

    //--- check data_len
    if(data_len % 16 != 0)
    {
        printk(KERN_ERR "ERROR in AES_Encrypt(): input length MUST be divisible by 16. Wrong length %d !!\n", data_len);
        return AES_ERR_DATA_LEN;
    }

    // Don't accept null output buffer in order to support any length
    if (cipher_buf == NULL)
    {
        printk(KERN_ERR "ERROR in AES_Encrypt(): must specify output buffer for cipher text !!\n");
        return AES_ERR_DATA_BUF;
    }

    memcpy(next_iv, iv, 16);
    while (1)
    {
		// Due to division into several sub-text, just the last sub-text can go async mode.
        if (data_len > AES_MAX_BCNT)
        {
            result = Do_AES(plain_buf, cipher_buf, AES_MAX_BCNT, next_iv, key, key_size, AES_ENCRYPT);
            if (result != 0)
                return result;  // return since AES fail
            // prepare for next round
            data_len -= AES_MAX_BCNT;
            plain_buf += AES_MAX_BCNT;
            cipher_buf += AES_MAX_BCNT;
            memcpy(next_iv, cipher_buf - 16, 16);   // support CBC mode, next IV = last cipher text
        }
        else    // final round
        {
            result = Do_AES_Async(plain_buf, cipher_buf, data_len, next_iv, key, key_size, AES_ENCRYPT);
            return result;
        }
    }
}
int AES_Encrypt(UINT8* plain_buf, UINT8* cipher_buf, UINT32 data_len, UINT8* iv,
                UINT8* key, KEYSIZE key_size)
{
	int errcode = AES_Encrypt_Async (plain_buf, cipher_buf, data_len, iv, key, key_size);
	if (errcode == AES_ERR_RUNNING) {
		errcode = AES_Flush ();
	}
	
	return errcode;
}

/*-----------------------------------------------------------------------------
 * Decrypt cipher text by AES algorithm that support by FA95 with any length.
 * INPUT:
 *      UINT8*  cipher_buf,     // pointer to cipher text buffer
 *      UINT8*  plain_buf,      // pointer to plain text buffer
 *      UINT32  data_len,       // length of data. MUST be divisible by 16
 *      UINT8*  iv,             // pointer to initial vector
 *      UINT8*  key,            // pointer to cipher key
 *      KEYSIZE key_size        // lenght of cipher key
 * OUTPUT:
 *      Put the plain text to buffer that output_buf pointed.
 * RETURN:
 *      same to Do_AES()
 *---------------------------------------------------------------------------*/
int AES_Decrypt_Async(UINT8* cipher_buf, UINT8* plain_buf, UINT32 data_len, UINT8* iv,
                UINT8* key, KEYSIZE key_size)
{
    int result;
    UINT8 next_iv[16];

    //--- check data_len
    if(data_len % 16 != 0)
    {
        printk(KERN_ERR "ERROR in AES_Decrypt(): input length MUST be divisible by 16. Wrong length %d !!\n", data_len);
        return AES_ERR_DATA_LEN;
    }

    // Don't accept null output buffer in order to support any length
    if (plain_buf == NULL)
    {
        printk(KERN_ERR "ERROR in AES_Decrypt(): must specify output buffer for plain text !!\n");
        return AES_ERR_DATA_BUF;
    }

    memcpy(next_iv, iv, 16);
    while (1)
    {
		// Due to division into several sub-text, just the last sub-text can go async mode.
        if (data_len > AES_MAX_BCNT)
        {
            result = Do_AES(cipher_buf, plain_buf, AES_MAX_BCNT, next_iv, key, key_size, AES_DECRYPT);
            if (result != 0)
                return result;  // return since AES fail
            // prepare for next round
            data_len -= AES_MAX_BCNT;
            plain_buf += AES_MAX_BCNT;
            cipher_buf += AES_MAX_BCNT;
            memcpy(next_iv, cipher_buf - 16, 16);   // support CBC mode, next IV = last cipher text
        }
        else    // final round
        {
            result = Do_AES_Async(cipher_buf, plain_buf, data_len, next_iv, key, key_size, AES_DECRYPT);
            return result;
        }
    }

//    return Do_AES(cipher_buf, plain_buf, data_len, iv, key, key_size, AES_DECRYPT);
}
int AES_Decrypt(UINT8* cipher_buf, UINT8* plain_buf, UINT32 data_len, UINT8* iv,
                UINT8* key, KEYSIZE key_size)
{
	int errcode = AES_Decrypt_Async (cipher_buf, plain_buf, data_len, iv, key, key_size);
	if (errcode == AES_ERR_RUNNING) {
		errcode = AES_Flush ();
	}
	
	return errcode;
}
				
int AES_Flush(VOID)
{
	int result;
	
	while ((result = AES_Check_Status ()) == AES_ERR_BUSY);
	
	return result;
}

/*-----------------------------------------------------------------------------
 * Check task status
 * RETURN:
 * 	0:					task done
 *	AES_ERR_BUSY:		task on-going
 * 	AES_ERR_BUS_ERROR:	bus error
 *---------------------------------------------------------------------------*/
int AES_Check_Status()
{
    //--- check Bus Error status
    if (inpw(REG_AESIER) & ENBERR)
    {
        // interrupt enable, check bus error by _aes_bIsBusError that update by interrupt handler
        if (_aes_bIsBusError)
        {
            printk(KERN_ERR "ERROR in AES_Check_Bus_Error(): Interrupt Handler inform that Bus Error !!\n");
            _aes_bIsBusError = FALSE;
			return AES_ERR_BUS_ERROR;
        }
    }
    else
    {
        // interrupt disable, check bus error by polling REG_AESISR
        if (inpw(REG_AESISR) & BERR)
        {
            printk(KERN_ERR "ERROR in AES_Check_Bus_Error(): Polling REG_AESISR and found Bus Error !!\n");
            outpw(REG_AESISR, BERR);    // clear interrupt flag
            return AES_ERR_BUS_ERROR;
        }
    }

   
   //--- check AES done
    if (inpw(REG_AESIER) & ENAESOK)
    {
        // interrupt enable, wait AES done by _aes_bIsAESOK that update by interrupt handler
        if (_aes_bIsAESOK)
        {
			_aes_bIsAESOK = FALSE;
			return 0;
		}
    }
    else
    {
        // interrupt disable, wait AES done by polling REG_AESISR
        if (inpw(REG_AESISR) & AESOK)
        {
			outpw(REG_AESISR, AESOK);   // clear interrupt flag
			return 0;
		}
    }
	
    return AES_ERR_BUSY;
}

/*-----------------------------------------------------------------------------
 * Enable AES interrupt feature in AES.
 *---------------------------------------------------------------------------*/
VOID AES_Enable_Interrupt(VOID)
{
    AES_Clear_Interrupt_Flag();             // clear all AES interrupt flags
    outpw(REG_AESIER, ENAESOK | ENBERR);    // enable all AES interrupts
}


/*-----------------------------------------------------------------------------
 * Disable AES interrupt feature in AES.
 *---------------------------------------------------------------------------*/
VOID AES_Disable_Interrupt(VOID)
{
    AES_Clear_Interrupt_Flag();             // clear all AES interrupt flags
    outpw(REG_AESIER, 0x00);                // disable all AES interrupts
}


/*-----------------------------------------------------------------------------
 * Initial AES.
 *---------------------------------------------------------------------------*/
VOID AES_Initial(VOID)
{
	// 1.Check I/O pins. If I/O pins are used by other IPs, return error code.
	// 2.Enable IP¡¦s clock
	//outpw(REG_AHBCLK, inpw(REG_AHBCLK) | IPSEC_CKE | HCLK3_CKE);
	// Enable HCLK3 clock.
	clk_hclk3 = clk_get(NULL, "hclk3");
	BUG_ON(IS_ERR(clk_hclk3));
	clk_enable(clk_hclk3);
	// Enable IPSEC clock.
	clk_ipsec = clk_get(NULL, "ipsec");
	BUG_ON(IS_ERR(clk_ipsec));
	clk_enable(clk_ipsec);
	// 3.Reset IP
	{
		unsigned long flags;
		local_irq_save(flags);
		outp32 (REG_AHBIPRST, IPSEC_RST);
		outp32 (REG_AHBIPRST, 0);
		local_irq_restore(flags);
	}
	// 4.Configure IP according to inputted arguments.
	// 5.Enable IP I/O pins
	// 6.Register ISR.
    //--- initial AES interrupt: hook up ISR to IRQ5 for AES and enable it.
	//sysInstallISR(IRQ_LEVEL_1, IRQ_IPSEC, (PVOID)AES_Int_Handler);    // hook up ISR
	//sysEnableInterrupt(IRQ_IPSEC);        // enable AES interrupt in AIC
}

/*-----------------------------------------------------------------------------
 * Final AES.
 *---------------------------------------------------------------------------*/
VOID AES_Final(VOID)
{
	// 1.Disable IP I/O pins
	// 2.Disable IP¡¦s clock
	//outp32 (REG_AHBCLK, inp32 (REG_AHBCLK) & ~IPSEC_CKE);
	// Disable IPSEC clock.
	BUG_ON(IS_ERR(clk_ipsec));
	clk_disable(clk_ipsec);
	clk_put(clk_ipsec);
	// Disable HCLK3 clock.
	BUG_ON(IS_ERR(clk_hclk3));
	clk_disable(clk_hclk3);
	clk_put(clk_hclk3);
}

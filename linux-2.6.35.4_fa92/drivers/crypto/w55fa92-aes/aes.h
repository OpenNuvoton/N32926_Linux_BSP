/***************************************************************************
 * Copyright (c) 2011 Nuvoton Technology. All rights reserved.
 *
 * FILENAME
 *     aes.h
 * DESCRIPTION
 *     The header file of AES library.
 * FUNCTIONS
 *     None
 **************************************************************************/
#ifndef _AES_H
#define _AES_H

#include <linux/kernel.h>
#include <asm/io.h>

#define VOID						void
#define BOOL						int
#define INT8						int8_t
#define UINT8						uint8_t
#define INT16						int16_t
#define UINT16						uint16_t
#define INT32						int32_t
#define UINT32						uint32_t
#define INT							int

enum {
	TRUE	=	1,
	FALSE	=	0
};

#define AES_ERR_ID	        0xFFFF1400  /* AES library ID */

//--- Define Error Code for AES
#define AES_ERR_FAIL        (AES_ERR_ID|0x01)
#define AES_ERR_DATA_LEN    (AES_ERR_ID|0x02)
#define AES_ERR_DATA_BUF    (AES_ERR_ID|0x03)
#define AES_ERR_CIPHER_KEY  (AES_ERR_ID|0x04)
#define AES_ERR_IV          (AES_ERR_ID|0x05)
#define AES_ERR_MODE        (AES_ERR_ID|0x06)
#define AES_ERR_BUS_ERROR   (AES_ERR_ID|0x07)
#define AES_ERR_RUNNING		(AES_ERR_ID|0x08)
#define AES_ERR_BUSY		(AES_ERR_ID|0x09)
#define AES_ERR_CMPDAT		(AES_ERR_ID|0x0A)

#define outp32(addr,value)		iowrite32((u32) value, (void *) addr)
#define inp32(addr)				ioread32((void *) addr)

#define outpw(addr,value)		iowrite32((u32) value, (void *) addr)
#define inpw(addr)				ioread32((void *) addr)

#define Successful				0
#define ERRCODE					int32_t

//--- Define valid macro for AES cipher key length
typedef enum
{
    KEY_128 = 0,
    KEY_192,
    KEY_256
} KEYSIZE;

//--- Define constant for FA95 AES engine
// The maximum byte count in one AES operation for AES engine. It MUST be divisible by 16 and <= 65535.
#define AES_MAX_BCNT    65520

//--- Declare the API prototype for AES driver
// Initial AES
VOID AES_Initial(VOID);
// Final AES
VOID AES_Final(VOID);

// Encrypt input_buf by AES CBC mode.
int AES_Encrypt(
    UINT8*  input_buf,      // pointer to plain text buffer
    UINT8*  output_buf,     // pointer to cipher text buffer
    UINT32  input_len,      // length of plain text
    UINT8*  iv,             // pointer to initial vector
    UINT8*  key,            // pointer to cipher key
    KEYSIZE key_size        // length of cipher key
    );

// Encrypt input_buf by AES CBC mode without waiting flush.
int AES_Encrypt_Async(
    UINT8*  input_buf,      // pointer to plain text buffer
    UINT8*  output_buf,     // pointer to cipher text buffer
    UINT32  input_len,      // length of plain text
    UINT8*  iv,             // pointer to initial vector
    UINT8*  key,            // pointer to cipher key
    KEYSIZE key_size        // length of cipher key
    );
	
// Decrypt input_buf by AES CBC mode.
int AES_Decrypt(
    UINT8*  input_buf,      // pointer to cipher text buffer
    UINT8*  output_buf,     // pointer to plain text buffer
    UINT32  input_len,      // length of cipher text
    UINT8*  iv,             // pointer to initial vector
    UINT8*  key,            // pointer to cipher key
    KEYSIZE key_size        // length of cipher key
    );

// Decrypt input_buf by AES CBC mode without flush.
int AES_Decrypt_Async(
    UINT8*  input_buf,      // pointer to cipher text buffer
    UINT8*  output_buf,     // pointer to plain text buffer
    UINT32  input_len,      // length of cipher text
    UINT8*  iv,             // pointer to initial vector
    UINT8*  key,            // pointer to cipher key
    KEYSIZE key_size        // length of cipher key
    );

// Wait for finish of pending job.
int AES_Flush(VOID);

// Check task status
int AES_Check_Status (VOID);

// Enable AES interrupt feature
VOID AES_Enable_Interrupt(VOID);

// Disable AES interrupt feature
VOID AES_Disable_Interrupt(VOID);

VOID AES_Int_Handler(VOID);

#endif  // end of _AES_H

//--------------------------------------------------------------
//
// Copyright (c) Nuvoton Technology Corp. All rights reserved.
//
//--------------------------------------------------------------
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/pci_ids.h>
#include <linux/crypto.h>
#include <linux/spinlock.h>
#include <linux/wait.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <crypto/algapi.h>
#include <crypto/aes.h>

#include <linux/io.h>
#include <linux/delay.h>

#include <asm/cacheflush.h>

#include <mach/w55fa92_reg.h>
#include "aes.h"

// Don't export H/W AES ECB in formal release due to its bad performance.
// H/W AES ECB is simulated by H/W AES CBC with only 1-block crypt size.
//#define HW_AES_ECB_TEST

#define DRIVER_NAME			"w55fa92-aes"

#define LOG_PREFIX			"[" DRIVER_NAME "] "

enum {
	AES_STAT_BY			=	0,
	AES_STAT_ERR		=	1,
};

struct w55fa92_aes_glob_ctx {
	int 					irq_line;
	int						irq_reqed;
	struct crypto_queue 	crpt_q;
	struct task_struct *	run_aes_crypt_tsk;
	
	int						ecb_aes_alg_async_reged;
	int						cbc_aes_alg_async_reged;
	
	int						ecb_aes_alg_sync_reged;
	int						cbc_aes_alg_sync_reged;
	
	unsigned long			stat;		// Use bit operations to ensure atomicity.
	//atomic_t open_nr;
	spinlock_t				lock;
	wait_queue_head_t		wq;
} glob_ctx;

struct w55fa92_aes_loc_ctx {
	uint8_t				key[AES_MAX_KEY_SIZE];
	unsigned long		keysize;
	int					keysize2;
	uint8_t				iv[16];
};

enum crypto_op {
	COP_AES_ECB,
	COP_AES_CBC,
};

struct w55fa92_aes_req_ctx {
	enum crypto_op 		op;
	int					is_enc;
};

static int w55fa92_aes_ecb_open_sync(struct crypto_tfm *tfm);
static void w55fa92_aes_ecb_close_sync(struct crypto_tfm *tfm);
static int w55fa92_aes_ecb_setkey_sync(struct crypto_tfm *tfm, const u8 *key, unsigned int len);
static int w55fa92_aes_ecb_encrypt_sync(struct blkcipher_desc *desc, struct scatterlist *dst, struct scatterlist *src, unsigned int nbytes);
static int w55fa92_aes_ecb_decrypt_sync(struct blkcipher_desc *desc, struct scatterlist *dst, struct scatterlist *src, unsigned int nbytes);

static int w55fa92_aes_cbc_open_sync(struct crypto_tfm *tfm);
static void w55fa92_aes_cbc_close_sync(struct crypto_tfm *tfm);
static int w55fa92_aes_cbc_setkey_sync(struct crypto_tfm *tfm, const u8 *key, unsigned int len);
static int w55fa92_aes_cbc_encrypt_sync(struct blkcipher_desc *desc, struct scatterlist *dst, struct scatterlist *src, unsigned int nbytes);
static int w55fa92_aes_cbc_decrypt_sync(struct blkcipher_desc *desc, struct scatterlist *dst, struct scatterlist *src, unsigned int nbytes);

static int aes_setkey_sync(struct crypto_tfm *tfm, const u8 *key, unsigned int len);

static int aes_ecb_crypt_sync(struct blkcipher_desc *desc, struct scatterlist *dst, struct scatterlist *src, unsigned int nbytes, int is_enc);
static int aes_cbc_crypt_sync(struct blkcipher_desc *desc, struct scatterlist *dst, struct scatterlist *src, unsigned int nbytes, int is_enc);

static struct crypto_alg w55fa92_aes_ecb_alg_sync = {
	.cra_name			=	"ecb(aes)",
	.cra_driver_name	=	DRIVER_NAME "-ecb-sync",
	.cra_priority		=	300,
	.cra_flags			=	CRYPTO_ALG_TYPE_BLKCIPHER,
	.cra_init			=	w55fa92_aes_ecb_open_sync,
	.cra_exit			=	w55fa92_aes_ecb_close_sync,
	.cra_blocksize		=	AES_BLOCK_SIZE,
	.cra_ctxsize		=	sizeof(struct w55fa92_aes_loc_ctx),
	.cra_alignmask		=	3,
	.cra_type			=	&crypto_blkcipher_type,
	.cra_module			=	THIS_MODULE,
	.cra_list			=	LIST_HEAD_INIT(w55fa92_aes_ecb_alg_sync.cra_list),
	.cra_u				=	{
		.blkcipher	=	{
			.min_keysize	=	AES_MIN_KEY_SIZE,
			.max_keysize	=	AES_MAX_KEY_SIZE,
			.setkey			=	w55fa92_aes_ecb_setkey_sync,
			.encrypt		=	w55fa92_aes_ecb_encrypt_sync,
			.decrypt		=	w55fa92_aes_ecb_decrypt_sync,
		}
	}
};

static struct crypto_alg w55fa92_aes_cbc_alg_sync = {
	.cra_name			=	"cbc(aes)",
	.cra_driver_name	=	DRIVER_NAME "-cbc-sync",
	.cra_priority		=	300,
	.cra_flags			=	CRYPTO_ALG_TYPE_BLKCIPHER,
	.cra_init			=	w55fa92_aes_cbc_open_sync,
	.cra_exit			=	w55fa92_aes_cbc_close_sync,
	.cra_blocksize		=	AES_BLOCK_SIZE,
	.cra_ctxsize		=	sizeof(struct w55fa92_aes_loc_ctx),
	.cra_alignmask		=	3,
	.cra_type			=	&crypto_blkcipher_type,
	.cra_module			=	THIS_MODULE,
	.cra_list			=	LIST_HEAD_INIT(w55fa92_aes_cbc_alg_sync.cra_list),
	.cra_u				=	{
		.blkcipher	=	{
			.min_keysize	=	AES_MIN_KEY_SIZE,
			.max_keysize	=	AES_MAX_KEY_SIZE,
			.setkey			=	w55fa92_aes_cbc_setkey_sync,
			.encrypt		=	w55fa92_aes_cbc_encrypt_sync,
			.decrypt		=	w55fa92_aes_cbc_decrypt_sync,
			.ivsize			=	16,
		}
	}
};

static int w55fa92_aes_ecb_open_async(struct crypto_tfm *tfm);
static void w55fa92_aes_ecb_close_async(struct crypto_tfm *tfm);
static int w55fa92_aes_ecb_setkey_async(struct crypto_ablkcipher *cipher, const u8 *key, unsigned int len);
static int w55fa92_aes_ecb_encrypt_async(struct ablkcipher_request *req);
static int w55fa92_aes_ecb_decrypt_async(struct ablkcipher_request *req);

static int w55fa92_aes_cbc_open_async(struct crypto_tfm *tfm);
static void w55fa92_aes_cbc_close_async(struct crypto_tfm *tfm);
static int w55fa92_aes_cbc_setkey_async(struct crypto_ablkcipher *cipher, const u8 *key, unsigned int len);
static int w55fa92_aes_cbc_encrypt_async(struct ablkcipher_request *req);
static int w55fa92_aes_cbc_decrypt_async(struct ablkcipher_request *req);

static int aes_setkey_async(struct crypto_ablkcipher *cipher, const u8 *key, unsigned int len);
static int aes_submit_req_async(struct crypto_async_request *req);

static int aes_ecb_crypt_async(struct ablkcipher_request *req);
static int aes_cbc_crypt_async(struct ablkcipher_request *req);

static int aes_crypt_hw(unsigned long src_pa, unsigned long dst_pa, unsigned long crypt_size, const uint8_t *iv, const uint8_t *key, KEYSIZE keysize2, int is_enc, int is_async);

static struct crypto_alg w55fa92_aes_ecb_alg_async = {
	.cra_name			=	"ecb(aes)",
	.cra_driver_name	=	DRIVER_NAME "-ecb-async",
	.cra_priority		=	400,	// Higher than sync version to be chosen to reduce CPU loading.
	.cra_flags			=	CRYPTO_ALG_TYPE_ABLKCIPHER | CRYPTO_ALG_ASYNC,
	.cra_init			=	w55fa92_aes_ecb_open_async,
	.cra_exit			=	w55fa92_aes_ecb_close_async,
	.cra_blocksize		=	AES_BLOCK_SIZE,
	.cra_ctxsize		=	sizeof(struct w55fa92_aes_loc_ctx),
	.cra_alignmask		=	3,
	.cra_type			=	&crypto_ablkcipher_type,
	.cra_module			=	THIS_MODULE,
	.cra_list			=	LIST_HEAD_INIT(w55fa92_aes_ecb_alg_async.cra_list),
	.cra_u				=	{
		.ablkcipher	=	{
			.min_keysize	=	AES_MIN_KEY_SIZE,
			.max_keysize	=	AES_MAX_KEY_SIZE,
			.setkey			=	w55fa92_aes_ecb_setkey_async,
			.encrypt		=	w55fa92_aes_ecb_encrypt_async,
			.decrypt		=	w55fa92_aes_ecb_decrypt_async,
		}
	}
};


static struct crypto_alg w55fa92_aes_cbc_alg_async = {
	.cra_name			=	"cbc(aes)",
	.cra_driver_name	=	DRIVER_NAME "-cbc-async",
	.cra_priority		=	400,	// Higher than sync version to be chosen to reduce CPU loading.
	.cra_flags			=	CRYPTO_ALG_TYPE_ABLKCIPHER | CRYPTO_ALG_ASYNC,
	.cra_init			=	w55fa92_aes_cbc_open_async,
	.cra_exit			=	w55fa92_aes_cbc_close_async,
	.cra_blocksize		=	AES_BLOCK_SIZE,
	.cra_ctxsize		=	sizeof(struct w55fa92_aes_loc_ctx),
	.cra_alignmask		=	3,
	.cra_type			=	&crypto_ablkcipher_type,
	.cra_module			=	THIS_MODULE,
	.cra_list			=	LIST_HEAD_INIT(w55fa92_aes_cbc_alg_async.cra_list),
	.cra_u				=	{
		.ablkcipher	=	{
			.min_keysize	=	AES_MIN_KEY_SIZE,
			.max_keysize	=	AES_MAX_KEY_SIZE,
			.setkey			=	w55fa92_aes_cbc_setkey_async,
			.encrypt		=	w55fa92_aes_cbc_encrypt_async,
			.decrypt		=	w55fa92_aes_cbc_decrypt_async,
			.ivsize			=	16,
		}
	}
};

static irqreturn_t aes_irq_handler(int irq, void *dev_id)
{
	struct w55fa92_aes_glob_ctx *glob_ctx = (struct w55fa92_aes_glob_ctx *) dev_id;
	
	AES_Int_Handler();
	do {
		int result = AES_Check_Status();
		switch (result) {
		case AES_ERR_BUS_ERROR:
			set_bit(AES_STAT_ERR, &glob_ctx->stat);
			clear_bit(AES_STAT_BY, &glob_ctx->stat);
			wake_up(&glob_ctx->wq);
			break;

		case AES_ERR_BUSY:
			break;
			
		case Successful:
			clear_bit(AES_STAT_BY, &glob_ctx->stat);
			wake_up(&glob_ctx->wq);
			break;
		}
	}
	while (0);
	
	return IRQ_HANDLED;
}


int w55fa92_aes_ecb_open_sync(struct crypto_tfm *tfm)
{	
	struct w55fa92_aes_loc_ctx *ctx = crypto_tfm_ctx(tfm);
	memset(ctx->iv, 0x00, 16);
	
	return 0;
}

void w55fa92_aes_ecb_close_sync(struct crypto_tfm *tfm)
{
}

int w55fa92_aes_ecb_setkey_sync(struct crypto_tfm *tfm, const u8 *key, unsigned int len)
{
	return aes_setkey_sync(tfm, key, len);
}

int w55fa92_aes_ecb_encrypt_sync(struct blkcipher_desc *desc, struct scatterlist *dst, struct scatterlist *src, unsigned int nbytes)
{
	return aes_ecb_crypt_sync(desc, dst, src, nbytes, 1);
}

int w55fa92_aes_ecb_decrypt_sync(struct blkcipher_desc *desc, struct scatterlist *dst, struct scatterlist *src, unsigned int nbytes)
{
	return aes_ecb_crypt_sync(desc, dst, src, nbytes, 0);
}

int w55fa92_aes_cbc_open_sync(struct crypto_tfm *tfm)
{
	return 0;
}

void w55fa92_aes_cbc_close_sync(struct crypto_tfm *tfm)
{
}

int w55fa92_aes_cbc_setkey_sync(struct crypto_tfm *tfm, const u8 *key, unsigned int len)
{
	return aes_setkey_sync(tfm, key, len);
}

int w55fa92_aes_cbc_encrypt_sync(struct blkcipher_desc *desc, struct scatterlist *dst, struct scatterlist *src, unsigned int nbytes)
{
	return aes_cbc_crypt_sync(desc, dst, src, nbytes, 1);
}

int w55fa92_aes_cbc_decrypt_sync(struct blkcipher_desc *desc, struct scatterlist *dst, struct scatterlist *src, unsigned int nbytes)
{
	return aes_cbc_crypt_sync(desc, dst, src, nbytes, 0);
}

int aes_setkey_sync(struct crypto_tfm *tfm, const u8 *key, unsigned int len)
{
	struct w55fa92_aes_loc_ctx *loc_ctx = crypto_tfm_ctx(tfm);
	
	switch (len) {
	case AES_KEYSIZE_128:
		loc_ctx->keysize2 = KEY_128;
		break;
		
	case AES_KEYSIZE_192:
		loc_ctx->keysize2 = KEY_192;
		break;
		
	case AES_KEYSIZE_256:
		loc_ctx->keysize2 = KEY_256;
		break;
		
	default:
		printk(LOG_PREFIX "Invalid key size: %d\n", len);
		return -EINVAL;
	}
	
	memcpy(loc_ctx->key, key, len);
	loc_ctx->keysize = len;
		
	return 0;
}


int aes_ecb_crypt_sync(struct blkcipher_desc *desc, struct scatterlist *dst, struct scatterlist *src, unsigned int nbytes, int is_enc)
{
	struct w55fa92_aes_loc_ctx *loc_ctx = (struct w55fa92_aes_loc_ctx *) crypto_blkcipher_ctx(desc->tfm);
	struct blkcipher_walk walk;
	int err = 0;
	
	blkcipher_walk_init(&walk, dst, src, nbytes);
	err = blkcipher_walk_phys(desc, &walk);
	if (err) {
		printk(KERN_ERR LOG_PREFIX "blkcipher_walk_phys failed\n");
		return err;
	}
	
	while ((nbytes = walk.nbytes)) {
		unsigned long src_pa = page_to_phys(walk.src.phys.page) + walk.src.phys.offset;
		unsigned long dst_pa = page_to_phys(walk.dst.phys.page) + walk.dst.phys.offset;
		unsigned long one_crypt_size = AES_BLOCK_SIZE;
		
		if (one_crypt_size > nbytes) {
			one_crypt_size = 0;
		}
		
		if (one_crypt_size) {
			err = aes_crypt_hw(src_pa, dst_pa, one_crypt_size, loc_ctx->iv, loc_ctx->key, loc_ctx->keysize2, is_enc, 0);
			if (err) {
				break;
			}
		}
		
		nbytes -= one_crypt_size;
		err = blkcipher_walk_done(desc, &walk, nbytes);
		if (err) {
			printk(KERN_ERR LOG_PREFIX "blkcipher_walk_done failed\n");
			break;
		}
	}

	return err;
}

int aes_cbc_crypt_sync(struct blkcipher_desc *desc, struct scatterlist *dst, struct scatterlist *src, unsigned int nbytes, int is_enc)
{
	struct w55fa92_aes_loc_ctx *loc_ctx = (struct w55fa92_aes_loc_ctx *) crypto_blkcipher_ctx(desc->tfm);
	struct blkcipher_walk walk;
	int err = 0;
	
	blkcipher_walk_init(&walk, dst, src, nbytes);
	err = blkcipher_walk_phys(desc, &walk);
	if (err) {
		printk(KERN_ERR LOG_PREFIX "blkcipher_walk_phys failed\n");
		return err;
	}
	
	memcpy(loc_ctx->iv, walk.iv, 16);
	
	while ((nbytes = walk.nbytes)) {
		unsigned long src_pa = page_to_phys(walk.src.phys.page) + walk.src.phys.offset;
		unsigned long dst_pa = page_to_phys(walk.dst.phys.page) + walk.dst.phys.offset;
		unsigned long one_crypt_size;
		uint8_t iv_tmp[16];
	
		one_crypt_size = nbytes;
		if (one_crypt_size > AES_MAX_BCNT) {
			one_crypt_size = AES_MAX_BCNT;
		}
		one_crypt_size -= one_crypt_size % AES_BLOCK_SIZE;
		
		if (one_crypt_size) {
			if (! is_enc) {// Keep last cipher text for next IV in decryption case in advance, which may overwrite when src/dst buffers overlap.
				memcpy(iv_tmp, phys_to_virt(src_pa + one_crypt_size - 16), 16);
			}
		
			err = aes_crypt_hw(src_pa, dst_pa, one_crypt_size, loc_ctx->iv, loc_ctx->key, loc_ctx->keysize2, is_enc, 0);
			if (err) {
				break;
			}
			
			// InCBC mode, next IV = last cipher text.
			if (is_enc) {
				memcpy(loc_ctx->iv, phys_to_virt(dst_pa + one_crypt_size - 16), 16);
			}
			else {
				memcpy(loc_ctx->iv, iv_tmp, 16);
			}
		}
		
		nbytes -= one_crypt_size;
		err = blkcipher_walk_done(desc, &walk, nbytes);
		if (err) {
			printk(KERN_ERR LOG_PREFIX "blkcipher_walk_done failed\n");
			break;
		}
	}

	return err;
}

int w55fa92_aes_ecb_open_async(struct crypto_tfm *tfm)
{	
	struct w55fa92_aes_loc_ctx *ctx = crypto_tfm_ctx(tfm);
	memset(ctx->iv, 0x00, 16);
	
	// Our context structure is allocated along with ablkcipher request by ablkcipher routine, so ablkcipher routine needs to know this size.
	tfm->crt_ablkcipher.reqsize = sizeof (struct w55fa92_aes_req_ctx);
	
	return 0;
}

void w55fa92_aes_ecb_close_async(struct crypto_tfm *tfm)
{
}

int w55fa92_aes_ecb_setkey_async(struct crypto_ablkcipher *cipher, const u8 *key, unsigned int len)
{
	return aes_setkey_async(cipher, key, len);
}

int w55fa92_aes_ecb_encrypt_async(struct ablkcipher_request *req)
{
	struct w55fa92_aes_req_ctx *req_ctx = ablkcipher_request_ctx(req);
	
	req_ctx->op = COP_AES_ECB;
	req_ctx->is_enc = 1;
	
	return aes_submit_req_async(&req->base);
}

int w55fa92_aes_ecb_decrypt_async(struct ablkcipher_request *req)
{	
	struct w55fa92_aes_req_ctx *req_ctx = ablkcipher_request_ctx(req);
	
	req_ctx->op = COP_AES_ECB;
	req_ctx->is_enc = 0;
	
	return aes_submit_req_async(&req->base);
}

int w55fa92_aes_cbc_open_async(struct crypto_tfm *tfm)
{
	tfm->crt_ablkcipher.reqsize = sizeof (struct w55fa92_aes_req_ctx);
	
	return 0;
}

void w55fa92_aes_cbc_close_async(struct crypto_tfm *tfm)
{
}

int w55fa92_aes_cbc_setkey_async(struct crypto_ablkcipher *cipher, const u8 *key, unsigned int len)
{
	return aes_setkey_async(cipher, key, len);
}

int w55fa92_aes_cbc_encrypt_async(struct ablkcipher_request *req)
{
	struct w55fa92_aes_req_ctx *req_ctx = ablkcipher_request_ctx(req);

	req_ctx->op = COP_AES_CBC;
	req_ctx->is_enc = 1;
	
	return aes_submit_req_async(&req->base);
}

int w55fa92_aes_cbc_decrypt_async(struct ablkcipher_request *req)
{
	struct w55fa92_aes_req_ctx *req_ctx = ablkcipher_request_ctx(req);

	req_ctx->op = COP_AES_CBC;
	req_ctx->is_enc = 0;

	return aes_submit_req_async(&req->base);
}

int aes_setkey_async(struct crypto_ablkcipher *cipher, const u8 *key, unsigned int len)
{
	struct crypto_tfm *tfm = crypto_ablkcipher_tfm(cipher);
	struct w55fa92_aes_loc_ctx *loc_ctx = crypto_tfm_ctx(tfm);
	
	switch (len) {
	case AES_KEYSIZE_128:
		loc_ctx->keysize2 = KEY_128;
		break;
		
	case AES_KEYSIZE_192:
		loc_ctx->keysize2 = KEY_192;
		break;
		
	case AES_KEYSIZE_256:
		loc_ctx->keysize2 = KEY_256;
		break;
		
	default:
		printk(LOG_PREFIX "Invalid key size: %d\n", len);
		return -EINVAL;
	}
	
	memcpy(loc_ctx->key, key, len);
	loc_ctx->keysize = len;
		
	return 0;
}

int aes_submit_req_async(struct crypto_async_request *req)
{
	unsigned long flags_intr;
	int err = 0;

	spin_lock_irqsave(&glob_ctx.lock, flags_intr);
	err = crypto_enqueue_request(&glob_ctx.crpt_q, req);
	spin_unlock_irqrestore(&glob_ctx.lock, flags_intr);
	wake_up_process(glob_ctx.run_aes_crypt_tsk);
	
	return err;
}

int aes_ecb_crypt_async(struct ablkcipher_request *req)
{
	int err = 0;
	unsigned long nbytes = 0;
	struct ablkcipher_walk walk;
	struct w55fa92_aes_loc_ctx *loc_ctx = crypto_tfm_ctx(req->base.tfm);
	struct w55fa92_aes_req_ctx *req_ctx = ablkcipher_request_ctx(req);
	
	ablkcipher_walk_init(&walk, req->dst, req->src, req->nbytes);
	err = ablkcipher_walk_phys(req, &walk);
	if (err) {
		printk(KERN_ERR LOG_PREFIX "ablkcipher_walk_phys failed\n");
		return err;
	}
	
	while ((nbytes = walk.nbytes)) {
		unsigned long src_pa = page_to_phys(walk.src.page) + walk.src.offset;
		unsigned long dst_pa = page_to_phys(walk.dst.page) + walk.dst.offset;
		unsigned long one_crypt_size = AES_BLOCK_SIZE;
						  
		if (one_crypt_size > nbytes) {
			one_crypt_size = 0;
		}
		
		if (one_crypt_size) {
			err = aes_crypt_hw(src_pa, dst_pa, one_crypt_size, loc_ctx->iv, loc_ctx->key, loc_ctx->keysize2, req_ctx->is_enc, 1);
			if (err) {
				break;
			}
		}
		
		nbytes -= one_crypt_size;
		err = ablkcipher_walk_done(req, &walk, nbytes);
		if (err) {
			printk(KERN_ERR LOG_PREFIX "ablkcipher_walk_done failed\n");
			break;
		}
	}
	
	ablkcipher_walk_complete(&walk);

	return err;
}

int aes_cbc_crypt_async(struct ablkcipher_request *req)
{
	int err = 0;
	unsigned long nbytes = 0;
	struct ablkcipher_walk walk;
	struct w55fa92_aes_loc_ctx *loc_ctx = crypto_tfm_ctx(req->base.tfm);
	struct w55fa92_aes_req_ctx *req_ctx = ablkcipher_request_ctx(req);
	
	ablkcipher_walk_init(&walk, req->dst, req->src, req->nbytes);
	err = ablkcipher_walk_phys(req, &walk);
	if (err) {
		printk(KERN_ERR LOG_PREFIX "ablkcipher_walk_phys failed\n");
		return err;
	}
	
	memcpy(loc_ctx->iv, walk.iv, 16);
	
	while ((nbytes = walk.nbytes)) {
		unsigned long src_pa = page_to_phys(walk.src.page) + walk.src.offset;
		unsigned long dst_pa = page_to_phys(walk.dst.page) + walk.dst.offset;
		unsigned long one_crypt_size;
		uint8_t iv_tmp[16];
	
		one_crypt_size = nbytes;
		if (one_crypt_size > AES_MAX_BCNT) {
			one_crypt_size = AES_MAX_BCNT;
		}
		one_crypt_size -= one_crypt_size % AES_BLOCK_SIZE;
		
		if (one_crypt_size) {
			if (! req_ctx->is_enc) {// Keep last cipher text for next IV in decryption case in advance, which may overwrite when src/dst buffers overlap.
				memcpy(iv_tmp, phys_to_virt(src_pa + one_crypt_size - 16), 16);
			}
			
			err = aes_crypt_hw(src_pa, dst_pa, one_crypt_size, loc_ctx->iv, loc_ctx->key, loc_ctx->keysize2, req_ctx->is_enc, 1);
			if (err) {
				break;
			}
			
			// InCBC mode, next IV = last cipher text.
			if (req_ctx->is_enc) {
				memcpy(loc_ctx->iv, phys_to_virt(dst_pa + one_crypt_size - 16), 16);
			}
			else {
				memcpy(loc_ctx->iv, iv_tmp, 16);
			}
		}
		
		nbytes -= one_crypt_size;
		err = ablkcipher_walk_done(req, &walk, nbytes);
		if (err) {
			printk(KERN_ERR LOG_PREFIX "ablkcipher_walk_done failed\n");
			break;
		}
	}

	ablkcipher_walk_complete(&walk);
	
	return err;
}

static int aes_crypt_hw(unsigned long src_pa, unsigned long dst_pa, unsigned long crypt_size, const uint8_t *iv, const uint8_t *key, KEYSIZE keysize2, int is_enc, int is_async)
{
	int err = 0;
	unsigned long flags_intr;
	int result = Successful;
	
	// In sync mode, this function cannot sleep because it may run in atomic context.
	if (is_async) {
		wait_event(glob_ctx.wq, ! test_and_set_bit(AES_STAT_BY, &glob_ctx.stat));
	}
	else {
		while (test_and_set_bit(AES_STAT_BY, &glob_ctx.stat)) {
			cpu_relax();
		}
	}
	
	flush_cache_all();
	
	spin_lock_irqsave(&glob_ctx.lock, flags_intr);	// Lock critical section.
	if (is_async) {
		AES_Enable_Interrupt();
	}
	else {
		AES_Disable_Interrupt();
	}
	spin_unlock_irqrestore(&glob_ctx.lock, flags_intr);	// Unlock critical section.
	
	spin_lock_irqsave(&glob_ctx.lock, flags_intr);	// Lock critical section.
	if (is_enc) {
		result = AES_Encrypt_Async((void *) src_pa, (void *) dst_pa, crypt_size, (uint8_t *) iv, (uint8_t *) key, keysize2);
	}
	else {
		result = AES_Decrypt_Async((void *) src_pa, (void *) dst_pa, crypt_size, (uint8_t *) iv, (uint8_t *) key, keysize2);
	}
	spin_unlock_irqrestore(&glob_ctx.lock, flags_intr);	// Unlock critical section.
	
	if (result == Successful) {
		clear_bit(AES_STAT_BY, &glob_ctx.stat);
	}
	else if (result == AES_ERR_RUNNING) {
		// In sync mode, this function cannot sleep because it may run in atomic context.
		if (is_async) {
			wait_event(glob_ctx.wq, ! test_bit(AES_STAT_BY, &glob_ctx.stat));
		}
		else {
			result = AES_Flush();
			if (result != Successful) {
				printk(LOG_PREFIX "AES_Flush failed: 0x%08x\n", result);
				err = -EFAULT;
		
				set_bit(AES_STAT_ERR, &glob_ctx.stat);
			}
			clear_bit(AES_STAT_BY, &glob_ctx.stat);
		}
	}
	else {
		printk(LOG_PREFIX "%s failed: 0x%08x\n", is_enc ? "AES_Encrypt_Async" : "AES_Decrypt_Async", result);
		err = -EFAULT;
		
		set_bit(AES_STAT_ERR, &glob_ctx.stat);
		clear_bit(AES_STAT_BY, &glob_ctx.stat);	
	}
	
	wake_up(&glob_ctx.wq);
	
	return err;
}

static int run_aes_crypt_hdlr(void *data)
{
	struct w55fa92_aes_glob_ctx *glob_ctx = (struct w55fa92_aes_glob_ctx *) data;
	
	do {
		unsigned long flags_intr;
		struct crypto_async_request *async_req = NULL;
		struct crypto_async_request *backlog_req = NULL;

		spin_lock_irqsave(&glob_ctx->lock, flags_intr);
		backlog_req = crypto_get_backlog(&glob_ctx->crpt_q);
		async_req = crypto_dequeue_request(&glob_ctx->crpt_q);
		spin_unlock_irqrestore(&glob_ctx->lock, flags_intr);
		
		if (backlog_req) {
			backlog_req->complete(backlog_req, -EINPROGRESS);
			backlog_req = NULL;
		}

		if (async_req) {
			int err = 0;
			struct ablkcipher_request *ablkcipher_req = container_of(async_req, struct ablkcipher_request, base);
			struct w55fa92_aes_req_ctx *req_ctx = ablkcipher_request_ctx(ablkcipher_req);

			if (req_ctx->op == COP_AES_ECB) {
				err = aes_ecb_crypt_async(ablkcipher_req);
			}
			else if (req_ctx->op == COP_AES_CBC) {
				err = aes_cbc_crypt_async(ablkcipher_req);
			}
			else {
				printk(LOG_PREFIX "Internal error\n");
				err = -EFAULT;
			}
			async_req->complete(async_req, err);
			ablkcipher_req = NULL;
			async_req = NULL;
		}
		else {
			// Relinguish CPU if no request pending. But if it is module unloading time, don't.
			// Via PREEMPT_ACTIVE, kernel guarantees non-TASK_RUNNING process won't remove from running queue by preemption until the call to schedule().
			// Sequence below is significant to avoid race condition.
			// 1. Change process state to sleep.
			// 2. Check condition.
			// 3. Go to sleep if need be.
			// 4. Change process stage back to running.
			set_current_state(TASK_INTERRUPTIBLE);
			spin_lock_irqsave(&glob_ctx->lock, flags_intr);
			if (list_empty(&glob_ctx->crpt_q.list) && ! kthread_should_stop()) {
				spin_unlock_irqrestore(&glob_ctx->lock, flags_intr);
				schedule();
				spin_lock_irqsave(&glob_ctx->lock, flags_intr);
			}
			spin_unlock_irqrestore(&glob_ctx->lock, flags_intr);
			set_current_state(TASK_RUNNING);	// Process state may still be in TASK_INTERRUPTIBLE due to condition not met. Change back to TASK_RUNNING.
		}
	}
	while (! kthread_should_stop());
	
	return 0;
}

static void aes_cleanup(void)
{
	if (glob_ctx.run_aes_crypt_tsk) {
		kthread_stop(glob_ctx.run_aes_crypt_tsk);
		glob_ctx.run_aes_crypt_tsk = NULL;
	}
	
	if (glob_ctx.ecb_aes_alg_sync_reged) {
		crypto_unregister_alg(&w55fa92_aes_ecb_alg_sync);
		glob_ctx.ecb_aes_alg_sync_reged = 0;
	}
	
	if (glob_ctx.cbc_aes_alg_sync_reged) {
		crypto_unregister_alg(&w55fa92_aes_cbc_alg_sync);
		glob_ctx.cbc_aes_alg_sync_reged = 0;
	}
	
	if (glob_ctx.ecb_aes_alg_async_reged) {
		crypto_unregister_alg(&w55fa92_aes_ecb_alg_async);
		glob_ctx.ecb_aes_alg_async_reged = 0;
	}
	
	if (glob_ctx.cbc_aes_alg_async_reged) {
		crypto_unregister_alg(&w55fa92_aes_cbc_alg_async);
		glob_ctx.cbc_aes_alg_async_reged = 0;
	}
	
	{
		unsigned long flags_intr;
		spin_lock_irqsave(&glob_ctx.lock, flags_intr);	// Lock critical section.
		AES_Disable_Interrupt();
		spin_unlock_irqrestore(&glob_ctx.lock, flags_intr);	// Unlock critical section.
	}
		
	if (glob_ctx.irq_reqed) {
		free_irq(glob_ctx.irq_line, &glob_ctx);
		glob_ctx.irq_reqed = 0;
	}
	
	//{	// GCR may access by other drivers, so disable all interrupts for sync.
	//	unsigned long flags_intr;
	//	local_irq_save(flags_intr);
	//	AES_Final();
	//	local_irq_restore(flags_intr);
	//}
	AES_Final();	// Take care of race condition in itself.
}

static int __init
w55fa92_aes_init(void)
{
	int err = 0;
	do {
		memset(&glob_ctx, 0x00, sizeof (glob_ctx));
		
		spin_lock_init(&glob_ctx.lock);
		
		crypto_init_queue(&glob_ctx.crpt_q, 100);
		
		init_waitqueue_head(&glob_ctx.wq);
		
		//{	// GCR may access by other drivers, so disable all interrupts for sync.
		//	unsigned long flags_intr;
		//	local_irq_save(flags_intr);
		//	AES_Initial();
		//	local_irq_restore(flags_intr);
		//}
		AES_Initial();	// Take care of race condition in itself.
		
		glob_ctx.irq_line = IRQ_IPSEC;
		err = request_irq(glob_ctx.irq_line, aes_irq_handler, IRQF_DISABLED, DRIVER_NAME, &glob_ctx);
		if (err) {
			printk(KERN_ERR LOG_PREFIX "request_irq failed, irq line = %d, err= %d\n", glob_ctx.irq_line, err);
			break;
		}
		glob_ctx.irq_reqed = 1;
		
		{
			unsigned long flags_intr;
			spin_lock_irqsave(&glob_ctx.lock, flags_intr);	// Lock critical section.
			AES_Enable_Interrupt();
			spin_unlock_irqrestore(&glob_ctx.lock, flags_intr);	// Unlock critical section.
		}
		
		glob_ctx.run_aes_crypt_tsk = kthread_run(run_aes_crypt_hdlr, &glob_ctx, DRIVER_NAME);
		if (IS_ERR(glob_ctx.run_aes_crypt_tsk)) {
			err = PTR_ERR(glob_ctx.run_aes_crypt_tsk);
			break;
		}
		
		// NOTE: Crypto self-test code may start immediately (if enabled) after algorithms are registered, so get everything ready before registration.
		
#ifdef HW_AES_ECB_TEST
		err = crypto_register_alg(&w55fa92_aes_ecb_alg_sync);
		if (err) {
			printk(KERN_ERR LOG_PREFIX "crypto_register_alg %s failed: %d\n", w55fa92_aes_ecb_alg_sync.cra_driver_name, err);
			break;
		}
		glob_ctx.ecb_aes_alg_sync_reged = 1;
#endif
		
		err = crypto_register_alg(&w55fa92_aes_cbc_alg_sync);
		if (err) {
			printk(KERN_ERR LOG_PREFIX "crypto_register_alg %s failed: %d\n", w55fa92_aes_cbc_alg_sync.cra_driver_name, err);
			break;
		}
		glob_ctx.cbc_aes_alg_sync_reged = 1;
		
#ifdef HW_AES_ECB_TEST
		err = crypto_register_alg(&w55fa92_aes_ecb_alg_async);
		if (err) {
			printk(KERN_ERR LOG_PREFIX "crypto_register_alg %s failed: %d\n", w55fa92_aes_ecb_alg_async.cra_driver_name, err);
			break;
		}
		glob_ctx.ecb_aes_alg_async_reged = 1;
#endif		
	
		err = crypto_register_alg(&w55fa92_aes_cbc_alg_async);
		if (err) {
			printk(KERN_ERR LOG_PREFIX "crypto_register_alg %s failed: %d\n", w55fa92_aes_cbc_alg_async.cra_driver_name, err);
			break;
		}
		glob_ctx.cbc_aes_alg_async_reged = 1;
	}
	while (0);
	
	if (err) {
		aes_cleanup();
	}
	else {
		printk(KERN_NOTICE LOG_PREFIX "Init OK\n");
	}
	
	return err;
}

static void __exit
w55fa92_aes_exit(void)
{
	aes_cleanup();
	
	printk(KERN_NOTICE LOG_PREFIX "Exit OK\n");
}

MODULE_AUTHOR("Nuvoton, Inc.");
MODULE_DESCRIPTION("W55FA92 AES driver");
MODULE_LICENSE("GPL");

module_init(w55fa92_aes_init);
module_exit(w55fa92_aes_exit);

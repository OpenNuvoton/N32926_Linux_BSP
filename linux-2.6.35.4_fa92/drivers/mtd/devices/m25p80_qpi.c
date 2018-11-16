#ifdef CONFIG_M25PXX_USE_QPI
#define OPCODE_QPP              0x32    /* Page program (up to 256 bytes) */


/*For EON/MXIC spi flash*/
static inline int EnableQuad_EON(struct m25p *flash, int bEnable)
{
	char bFlag=0x0;
        u8 code[4], code1[4];

	code[0] = 0x06;
	spi_write(flash->spi, code, 1);

	if (bEnable)
		bFlag = 0x40 ;

        code1[0] = 0x1;
        code1[1] = bFlag;
	spi_write(flash->spi, code1, 2);

	return 1;
}
/* For MXIC */
static int QPIRead_MXIC ( struct m25p *flash, struct spi_transfer* pT, struct spi_message* pM, u_char *buf, size_t len )
{
        /* NOTE:
         * OPCODE_FAST_READ (if available) is faster.
         * Should add 3 byte DUMMY_BYTE.
         */
	flash->m_i32DummyByte = 3;

	// command
        pT[0].tx_buf = flash->command;
        pT[0].len = 1;
	pT[0].bit_mode = 0;
        spi_message_add_tail(pT, pM);

	// address+dummy
        pT[1].tx_buf = flash->command+pT[0].len;
	memset ( (void*)(flash->command+m25p_cmdsz(flash)), 0xFF, 8-m25p_cmdsz(flash) );
        pT[1].len = m25p_cmdsz(flash)-pT[0].len + flash->m_i32DummyByte;
	pT[1].bit_mode = SPI_TX_QUAD;
        spi_message_add_tail(pT+1, pM);

	// read data
        pT[2].rx_buf = buf;
        pT[2].len = len;
	pT[2].bit_mode = SPI_RX_QUAD;
        spi_message_add_tail(pT+2, pM);

        /* Set up the write data buffer. */

	flash->command[0] = 0xEB;

	return 2;
}

static int QPIWrite_MXIC  ( struct m25p *flash, struct spi_transfer* pT, struct spi_message* pM, const u_char * buf, size_t len )
{
        pT[0].tx_buf = flash->command;
	pT[0].len = 1;
	pT[0].bit_mode = 0;
        spi_message_add_tail(pT, pM);

        pT[1].tx_buf = flash->command+pT[0].len;
	pT[1].len = m25p_cmdsz(flash)-pT[0].len;
	pT[1].bit_mode = SPI_TX_QUAD;
        spi_message_add_tail(pT+1, pM);

        pT[2].tx_buf = buf;
	pT[2].len = len;
	pT[2].bit_mode = SPI_TX_QUAD;
        spi_message_add_tail(pT+2, pM);

        /* Set up the opcode in the write buffer. */
        flash->command[0] = 0x38; 	// Quad page program
	return 2;
}

/* For EON */
static int QPIRead_EON ( struct m25p *flash, struct spi_transfer* pT, struct spi_message* pM, u_char *buf, size_t len )
{
        /* NOTE:
         * OPCODE_FAST_READ (if available) is faster.
         * Should add 3 byte DUMMY_BYTE.
         */
	flash->m_i32DummyByte = 3;

#if 0
	// command
        pT[0].tx_buf = flash->command;
        pT[0].len = 1;
	pT[0].bit_mode = 0;
        spi_message_add_tail(pT, pM);

	// address+dummy
        pT[1].tx_buf = flash->command+pT[0].len;
	memset ( (void*)(flash->command+m25p_cmdsz(flash)), 0xFF, 8-m25p_cmdsz(flash) );
        pT[1].len = m25p_cmdsz(flash)-pT[0].len + flash->m_i32DummyByte;
	pT[1].bit_mode = SPI_TX_QUAD;
        spi_message_add_tail(pT+1, pM);

	// read data
        pT[2].rx_buf = buf;
        pT[2].len = len;
	pT[2].bit_mode = SPI_RX_QUAD;
        spi_message_add_tail(pT+2, pM);

        /* Set up the write data buffer. */

	flash->command[0] = 0xEB;
	return 2;
#else
	memset ( (void*)(flash->command+m25p_cmdsz(flash)), 0xFF, 8-m25p_cmdsz(flash) );
	// command+address+dummy
        pT[0].tx_buf = flash->command;
        pT[0].len =  m25p_cmdsz(flash) + flash->m_i32DummyByte;
	pT[0].bit_mode = SPI_TX_QUAD;
        spi_message_add_tail(pT, pM);

	// read data
        pT[1].rx_buf = buf;
        pT[1].len = len;
	pT[1].bit_mode = SPI_RX_QUAD;
        spi_message_add_tail(pT+1, pM);

        /* Set up the write data buffer. */

	flash->command[0] = 0xEB;
	return 1;
#endif

}

static int QPIWrite_EON  ( struct m25p *flash, struct spi_transfer* pT, struct spi_message* pM, const u_char * buf, size_t len )
{

// This is wrong code.

        pT[0].tx_buf = flash->command;
	pT[0].len = m25p_cmdsz(flash);
	pT[0].bit_mode = SPI_TX_QUAD;
        spi_message_add_tail(pT, pM);

        pT[1].tx_buf = buf;
	pT[1].len = len;
	pT[1].bit_mode = SPI_TX_QUAD;
        spi_message_add_tail(pT+1, pM);

        /* Set up the opcode in the write buffer. */
        flash->command[0] = 0x02; 	// Quad page program

	return 1;
}

/*For winbond spi flash*/
static inline int EnableQuad_WB(struct m25p *flash, int bEnable)
{
	char bFlag=0x0;
        u8 code[4], code1[4];

	code[0] = 0x06;
	spi_write(flash->spi, code, 1);

	if (bEnable)
		bFlag = SR_WEL ;
        code1[0] = 0x1;
        code1[1] = 0;
        code1[2] = bFlag;
	spi_write(flash->spi, code1, 3);

	return 1;
}

static inline int m25p80_EnableQuad(struct m25p *flash, int bEnable)
{
	if ( flash->m_i32InQPIMode )
		return 0;
	flash->m_i32InQPIMode=1;

        return flash->m_pfnEnableQuad(flash, bEnable);
}

/* For winbond */
static int QPIRead_WB ( struct m25p *flash, struct spi_transfer* pT, struct spi_message* pM, u_char *buf, size_t len )
{
        /* NOTE:
         * OPCODE_FAST_READ (if available) is faster.
         * Should add 1 byte DUMMY_BYTE.
         */
	flash->m_i32DummyByte = 1;

        pT[0].tx_buf = flash->command;
        pT[0].len = m25p_cmdsz(flash) + flash->m_i32DummyByte;
	pT[0].bit_mode = 0;
        spi_message_add_tail(pT, pM);

        pT[1].rx_buf = buf;
        pT[1].len = len;
	pT[1].bit_mode = SPI_RX_QUAD;
        spi_message_add_tail(pT+1, pM);

        /* Set up the write data buffer. */
        flash->command[0] = OPCODE_FAST_READ_QPI;
	return 1;
}

static int QPIWrite_WB  ( struct m25p *flash, struct spi_transfer* pT, struct spi_message* pM, const u_char *buf, size_t len )
{

        pT[0].tx_buf = flash->command;
	pT[0].len = m25p_cmdsz(flash);
	pT[0].bit_mode = 0;
        spi_message_add_tail(pT, pM);

        pT[1].tx_buf = buf;
	pT[1].bit_mode = SPI_TX_QUAD;

        spi_message_add_tail(pT+1, pM);

        /* Set up the opcode in the write buffer. */
        flash->command[0] = OPCODE_QPP; 	// Quad page program
	return 1;
}


/*
 * Read an address range from the flash chip.  The address range
 * may be any size provided it is within the physical boundaries.
 */
static int m25p80_qpi_read(struct mtd_info *mtd, loff_t from, size_t len,
        size_t *retlen, u_char *buf)
{
        struct m25p *flash = mtd_to_m25p(mtd);
        struct spi_transfer t[4];
        struct spi_message m;
	int i32LastMsgIdx=0;

        DEBUG(MTD_DEBUG_LEVEL2, "%s: %s %s 0x%08x, len %zd\n",
                        dev_name(&flash->spi->dev), __func__, "from",
                        (u32)from, len);

        /* sanity checks */
        if (!len)
                return 0;

        if (from + len > flash->mtd.size)
                return -EINVAL;

        spi_message_init(&m);
        memset(t, 0, (sizeof t));

        mutex_lock(&flash->lock);

#ifdef CONFIG_M25PXX_AUTO_RESET_FLASH
	m25p_chkaddrmode(flash, from);
#endif

	m25p80_EnableQuad(flash, 1);

#if 0
        /* NOTE:
         * OPCODE_FAST_READ (if available) is faster.
         * Should add 1 byte DUMMY_BYTE.
         */
        t[0].tx_buf = flash->command;
        t[0].len = m25p_cmdsz(flash) + FAST_READ_DUMMY_BYTE;
	t[0].bit_mode = 0;
        spi_message_add_tail(&t[0], &m);

        t[1].rx_buf = buf;
        t[1].len = len;
	t[1].bit_mode = SPI_RX_QUAD;

        spi_message_add_tail(&t[1], &m);
#else
	i32LastMsgIdx = flash->m_pfnQPIRead ( flash, &t[0], &m, buf, len );
#endif

        /* Byte count starts at zero. */
        *retlen = 0;

	if ( flash->m_isLastWrite )
	{
	        /* Wait till previous write/erase is done. */
	        if (wait_till_ready(flash)) {
	#ifdef CONFIG_M25PXX_AUTO_RESET_FLASH			
		m25p_chkaddrmode(flash, 0);
	#endif			
	                /* REVISIT status return?? */
	                mutex_unlock(&flash->lock);
	                return 1;
	        }
	}

        /* FIXME switch to OPCODE_FAST_READ.  It's required for higher
         * clocks; and at this writing, every chip this driver handles
         * supports that opcode.
         */

        m25p_addr2cmd(flash, from, flash->command);

        spi_sync(flash->spi, &m);

        *retlen = m.actual_length - m25p_cmdsz(flash) - flash->m_i32DummyByte;

#ifdef CONFIG_M25PXX_AUTO_RESET_FLASH
	m25p_chkaddrmode(flash, 0);
#endif		

	flash->m_isLastWrite = 0;

        mutex_unlock(&flash->lock);

        return 0;
}

/*
 * Write an address range to the flash chip.  Data must be written in
 * FLASH_PAGESIZE chunks.  The address range may be any size provided
 * it is within the physical boundaries.
 */
static int m25p80_qpi_write(struct mtd_info *mtd, loff_t to, size_t len,
        size_t *retlen, const u_char *buf)
{
        struct m25p *flash = mtd_to_m25p(mtd);
        u32 page_offset, page_size;
        struct spi_transfer t[4];
        struct spi_message m;
	int i32LastMsgIdx=0;

        DEBUG(MTD_DEBUG_LEVEL2, "%s: %s %s 0x%08x, len %zd\n",
                        dev_name(&flash->spi->dev), __func__, "to",
                        (u32)to, len);

        *retlen = 0;

        /* sanity checks */
        if (!len)
                return(0);

        if (to + len > flash->mtd.size)
                return -EINVAL;

        spi_message_init(&m);
        memset(t, 0, (sizeof t));

        mutex_lock(&flash->lock);
#ifdef CONFIG_M25PXX_AUTO_RESET_FLASH
	m25p_chkaddrmode(flash, to);
#endif

	m25p80_EnableQuad(flash, 1);

#if 0
        t[0].tx_buf = flash->command;
	t[0].len = m25p_cmdsz(flash);
	t[0].bit_mode = 0;
        spi_message_add_tail(&t[0], &m);

        t[1].tx_buf = buf;
	t[1].bit_mode = SPI_TX_QUAD;

        spi_message_add_tail(&t[1], &m);
#else
	i32LastMsgIdx = flash->m_pfnQPIWrite ( flash, &t[0], &m, buf, len );
#endif
        /* Wait until finished previous write command. */
        if (wait_till_ready(flash)) {
#ifdef CONFIG_M25PXX_AUTO_RESET_FLASH			
	m25p_chkaddrmode(flash, 0);
#endif			
                mutex_unlock(&flash->lock);
                return 1;
        }

        write_enable(flash);

        m25p_addr2cmd(flash, to, flash->command);

        page_offset = to & (flash->page_size - 1);

        /* do all the bytes fit onto one page? */
        if (page_offset + len <= flash->page_size) {
                t[i32LastMsgIdx].len = len;

                spi_sync(flash->spi, &m);

                *retlen = m.actual_length - m25p_cmdsz(flash);
        } else {
                u32 i;

                /* the size of data remaining on the first page */
                page_size = flash->page_size - page_offset;

                t[i32LastMsgIdx].len = page_size;
                spi_sync(flash->spi, &m);

                *retlen = m.actual_length - m25p_cmdsz(flash);

                /* write everything in flash->page_size chunks */
                for (i = page_size; i < len; i += page_size) {
                        page_size = len - i;
                        if (page_size > flash->page_size)
                                page_size = flash->page_size;

                        /* write the next page to flash */
                        m25p_addr2cmd(flash, to + i, flash->command);

                        t[i32LastMsgIdx].tx_buf = buf + i;
                        t[i32LastMsgIdx].len = page_size;

                        wait_till_ready(flash);

                        write_enable(flash);

                        spi_sync(flash->spi, &m);

                        *retlen += m.actual_length - m25p_cmdsz(flash);
                }
        }

#ifdef CONFIG_M25PXX_AUTO_RESET_FLASH
	m25p_chkaddrmode(flash, 0);
#endif	

	flash->m_isLastWrite = 1;

        mutex_unlock(&flash->lock);

        return 0;
}
#endif

/* First release: 2009.05.10
 * Foremedia's FM23-380 is an echo canceller and noise suppressor.
 */
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/ioctl.h>
#include <linux/interrupt.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/platform_device.h>
#include <linux/clk.h>

#include <mach/w55fa92_reg.h>
#include <asm/io.h>

#include "fm23.h"

#undef log
#define log printk
//#define log(...)

#define IOC_MAXNR	2

#define FM23_DRIVER_NAME "fm23"

static int __init fm23_init(void);
static int fm23_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int fm23_remove(struct i2c_client *client);
static int fm23_open(struct inode *inode, struct file *file);
static int fm23_close(struct inode *inode, struct file *file);
static int fm23_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg);
static int i2c_write(struct i2c_client *client, char *buf, int count);
static int i2c_read(struct i2c_client *client, char *buf, int count);
static int fm23_write(int regH, int regL, int dataH, int dataL);
static int fm23_read(int regH, int regL);
static void fm23_power_seq(void);

struct ard_denoise_mic_gpios dm_gpios;

static const struct i2c_device_id fm23_id[] = {
    { FM23_DRIVER_NAME, 0 },
    { }
};

static struct fm23_data {
    struct i2c_client *client;
    wait_queue_head_t wait;
} fm23_data;

static const struct file_operations fm23_fops = {
    .owner      = THIS_MODULE,
    .open       = fm23_open,
    .release    = fm23_close,
    .ioctl      = fm23_ioctl,
};

static struct i2c_driver fm23_driver = {
    .probe		= fm23_probe,
    .remove		= fm23_remove,
    .id_table	= fm23_id,
    .driver		= {
        .name = FM23_DRIVER_NAME,
    },
};

static struct miscdevice fm23_dev = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = FM23_DRIVER_NAME,
    .fops = &fm23_fops,
};

typedef struct S_REG_TABLE
{
	unsigned short reg;
	unsigned short data;
}REG_TABLE;

//手持模式BYPASS
static REG_TABLE s_FmHandsetBypass[]= 
{          
	{0x1E6F,0x0003},
	{0x1E72,0x8002},
	{0x1E82,0x0001},
	{0x1E84,0x001E},
	{0x1E85,0x0000},
	{0x1E87,0x0003},
	//{0x1E50,0x0019},//26M
	//{0x1E50,0x0017},//24M	
	{0x1E50,0x000B},	//12M
	{0x1E45,0x0000},
	{0x1E75,0x0000},
};

//手持模式 
static REG_TABLE s_FmHandsetNormal[]= 
{  
	//{0x1E50,0x0019},//26M
	{0x1E50,0x000B},	//12M
	{0x1E83, 0x2D89},
	{0x1E84, 0x031E},
	{0x1E85, 0x006C},
	{0x1E87, 0x0033},
	{0x1E88, 0x0800},
	{0x1E8B, 0x0200},
	{0x1EAE, 0x0040},
	{0x1EB1, 0x7FFF},
	{0x1EB2, 0x0004},
	{0x1EC7, 0x0E00},
	{0x1ED9, 0x0600},
	{0x1EDA, 0x1200},
	{0x1EED, 0x1000},
	{0x1EF6, 0x7000},
	{0x1F02, 0x0440},
	{0x1F03, 0x0007},
	{0x1F04, 0x0005},
	{0x1F0B, 0x4000},
	{0x1F5D, 0x4000},
	{0x1EB8, 0x0001},
	{0x1ECF, 0x2800},
	{0x1ED0, 0x2800},
	{0x1E72, 0x8000},
	{0x1E6F, 0x0003},
	{0x1EEE, 0x0B05},
	{0x1EEF, 0x0800},
	{0x1EC8, 0x0E00},
	{0x1F5C, 0x0600},
	{0x1F53, 0x2000},
	{0x1F33, 0x0001},
	{0x1F32, 0x0004},
	{0x1E75, 0x0000},
};

//免提模式
static REG_TABLE s_FmHandFreeNormal[]= 
{
	{0x1E6F, 0x0003},
	{0x1E72, 0x8002},
	{0x1E82, 0x0101},
	{0x1E83, 0x2981},
	{0x1E84, 0x030F},
	{0x1E85, 0x0005},
	{0x1E87, 0x0033},
	{0x1E88, 0x0400},
	{0x1E89, 0x0000},
	{0x1E8B, 0x0200},
	{0x1E8C, 0x0100},
	{0x1E8F, 0x0080},
	{0x1EA7, 0x7FFF},
	{0x1EA9, 0x0000},
	{0x1EAE, 0x0060},
	{0x1EB1, 0x0200},
	{0x1EB2, 0x0040},
	{0x1EB8, 0x0001},
	{0x1EC7, 0x0E00},
	{0x1EC8, 0x0800},
	{0x1EED, 0x1000},
	{0x1EEE, 0x0B05},
	{0x1EEF, 0x0800},
	{0x1F03, 0x0007},
	{0x1F0B, 0x4000},
	{0x1F24, 0x0000},
	{0x1F32, 0x0002},
	{0x1F33, 0x0001},
	{0x1F34, 0x6000},
	{0x1F36, 0x0003},
	{0x1F37, 0x7800},
	{0x1F38, 0x2400},
	{0x1F39, 0x0600},
	{0x1F3A, 0x1800},
	{0x1F4D, 0x0780},
	{0x1F4E, 0x0680},
	{0x1F4F, 0x0400},
	{0x1F50, 0x0200},
	{0x1F53, 0x3000},
	{0x1F5C, 0x0A00},
	{0x1F5D, 0x4000},
	{0x1F5F, 0x6A00},
	//{0x1E50,0x0019},//26M
	{0x1E50,0x000B},	//12M
	{0x1ECF, 0x1000},
	{0x1ED0, 0x1000},
	{0x1ED9, 0x0600},
	{0x1EDA, 0x1200},
	{0x1EF6, 0x7000},
	{0x1F02, 0x0440},
	{0x1F04, 0x0005},
	{0x1EF1, 0x2C00},
	{0x1E75, 0x0000},	
};
	
static int i2c_read(struct i2c_client *client, char *buf, int count) {
    if(count != i2c_master_send(client, buf, count)) {
        printk("[FM23] i2c_read --> Send reg. info error\n");
        return -1;
    }

    if(1 != i2c_master_recv(client, buf, 1)) {
        printk("[FM23] i2c_read --> get response error\n");
        return -1;
    }
    return 0;
}

static int i2c_write(struct i2c_client *client, char *buf, int count) {
    if(count != i2c_master_send(client, buf, count)) {
        printk("[FM23] i2c_write --> Send reg. info error\n");
        return -1;
    }
    return 0;
}

static int fm23_read(int regH, int regL)
{
    uint8_t fm_wBuf[5];
    uint8_t fm_rBuf[4];
    int dataH, dataL, dataA;
    struct i2c_client *client = fm23_data.client;

    fm_wBuf[0]=0xFC;
    fm_wBuf[1]=0xF3;
    fm_wBuf[2]=0x37;
    fm_wBuf[3]=regH;
    fm_wBuf[4]=regL;
    i2c_write(client, fm_wBuf, 5);
    msleep(1);

    /* Get high byte */
    fm_rBuf[0]=0xfc;
    fm_rBuf[1]=0xf3;
    fm_rBuf[2]=0x60;
    fm_rBuf[3]=0x26;
    i2c_read(client, fm_rBuf, 4);
    dataH = fm_rBuf[0];

    /* Get low byte */
    fm_rBuf[0]=0xfc;
    fm_rBuf[1]=0xf3;
    fm_rBuf[2]=0x60;
    fm_rBuf[3]=0x25;
    i2c_read(client, fm_rBuf, 4);
    dataL = fm_rBuf[0];

    dataA = dataH;
    dataA = dataA << 8;
    dataA = dataA | dataL;

    return dataA;
}

static int fm23_write(int regH, int regL, int dataH, int dataL)
{
    uint8_t fm_wBuf[7];
    struct i2c_client *client = fm23_data.client;

    fm_wBuf[0]=0xFC;
    fm_wBuf[1]=0xF3;
    fm_wBuf[2]=0x3B;
    fm_wBuf[3]=regH;
    fm_wBuf[4]=regL;
    fm_wBuf[5]=dataH;
    fm_wBuf[6]=dataL;
    i2c_write(client, fm_wBuf, 7);
    msleep(1);

    return 0;
}

static int fm23_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int err = 0;

    fm23_data.client = client;

    log("[FM23] Probe!!\n");

    fm23_power_seq();

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        printk("[FM23] i2c_check_functionality error!\n");
        return -ENOTSUPP;
    }
    strlcpy(client->name, FM23_DRIVER_NAME, I2C_NAME_SIZE);
    i2c_set_clientdata(client, &fm23_data);

    init_waitqueue_head(&fm23_data.wait);

    err = misc_register(&fm23_dev);
    if (err) {
        printk("fm23_probe: fm23_dev register failed\n");
        goto error_fm23_dev;
    }

    fm23_set_procedure(SET_FM_HANDFREE_NOR);
    mdelay(115);

    printk("[FM23] probe done\n");
    return 0;

error_fm23_dev:
    printk("[FM23] probe error\n");
    return err;
}

static void fm23_write_table(REG_TABLE *table,int size)
{
	int i=0;
	unsigned short reg=0,data=0;
	for(i=0;i<size;i++)
	{
		reg=table[i].reg;
		data=table[i].data;

		fm23_write((reg&0xff00)>>8, reg&0xff, (data&0xff00)>>8, data&0xff); //1
	}
}

static void fm23_start_mclk()
{
	extern void nvt_lock(void);
	extern void nvt_unlock(void);
	extern unsigned int w55fa92_external_clock;

	unsigned int sclk=0;
	//
	//PWM CLK OUT GPD13-PWM1
	//
	/* Enable PWM clock */
	struct clk *clk = clk_get(NULL, "pwm");
	int freq=24000000;

	BUG_ON(IS_ERR(clk));
	clk_enable(clk);

	nvt_lock();	
	//12MHZ
	//__raw_writel(__raw_readl(REG_CLKDIV5) & ~0x1FFF, REG_CLKDIV5);
	//sclk=12000000;
	
	//48MHZ
	__raw_writel((__raw_readl(REG_CLKDIV5) & ~0x1FFF)|0x001C, REG_CLKDIV5);
	sclk=48000000;
	nvt_unlock();		

	/* Set divider to 1 */
	__raw_writel(0x4444, REG_PWM_CSR);
	
	/* Set all to Toggle mode  */
	__raw_writel(0x0C0C0C0C, REG_PCR);	

	// set PWM clock to 6MHz
	__raw_writel((((sclk/freq - 1) << 8)|(sclk/freq - 1)), REG_PPR);	

	//
	//Enable PWM1
	//
	/* Enable Channel 1 */	
	__raw_writel(0x08080D08, REG_PCR);	
	/* Set Channel 1 Pin function */	
	__raw_writel((((__raw_readl(REG_GPDFUN1) & ~MF_GPD13)) | (1<<20)), REG_GPDFUN1);

	//__raw_writel(1, REG_CNR1);  // duty 1/2
	//__raw_writel(0, REG_CMR1);  // 

	__raw_writel(1, REG_CNR1);  // duty 1/2
	__raw_writel(0, REG_CMR1);  // 

	__raw_writel((__raw_readl(REG_POE) | (0x01 << 1)), REG_POE);		

	/* Enable Channel 1 */	
	__raw_writel(0x08080D08, REG_PCR);

}

static void fm23_stop_mclk()
{
	__raw_writel(0x08080808, REG_PCR);	
}

static void fm23_power_seq(void)
{
//reset GPD15
#if 0
    gpio_set_value(dm_gpios.en, 1);
    mdelay(10);
    gpio_set_value(dm_gpios.pwd, 1);
    gpio_set_value(dm_gpios.clk, 1);
    mdelay(5);
    gpio_set_value(dm_gpios.reset, 1);
    mdelay(30);
#else
    __raw_writel(readl(REG_GPDFUN1) & (~ MF_GPG15), REG_GPDFUN1);
    __raw_writel(readl(REG_GPIOD_OMD) | BIT15, REG_GPIOD_OMD);
    __raw_writel(readl(REG_GPIOD_DOUT) & ~BIT15, REG_GPIOD_DOUT);
    mdelay(10);

	fm23_start_mclk();

    mdelay(10);
	
    __raw_writel(readl(REG_GPIOD_DOUT) | BIT15, REG_GPIOD_DOUT);
    mdelay(30);
#endif

}

static int fm23_remove(struct i2c_client *client)
{
    misc_deregister(&fm23_dev);
    return 0;
}

/*	open command for fm23 device file	*/
static int fm23_open(struct inode *inode, struct file *file)
{
    log("[FM23] has been opened\n");
    return 0;
}

static int fm23_close(struct inode *inode, struct file *file)
{
    log("[FM23] has been closed\n");
    return 0;
}

int fm23_set_pwd(int com)
{
    return 0;
}

int fm23_set_procedure(unsigned int commad)
{
    uint16_t fmdata;

    switch(commad) {
    case IOCTL_GET_FM_STATE:
        log("[FM23] IOCTL_GET_FM_STATE \n");
        fmdata = fm23_read(0x1E, 0x3A);
        return fmdata;

    case IOCTL_SET_FM_STANDBY:
        log("[FM23] IOCTL_SET_FM_STANDBY procedure!!!!!\n");
        fm23_write(0x1E, 0x4D, 0x03, 0xF0); /* no need to reload parameter */
        fm23_write(0x1E, 0x79, 0x00, 0x01);
		mdelay(10);
		fm23_stop_mclk();
        return 0;

    case SET_FM_HANDSET_BYPASS:
        log("[FM23] SET_FM_HANDSET_BYPASS procedure!!!!!\n");
		fm23_write_table(s_FmHandsetBypass,sizeof(s_FmHandsetBypass)/sizeof(REG_TABLE));
        return 0;

    case SET_FM_HANDSET_NOR:
        log("[FM23] SET_FM_HANDSET_NOR procedure!!!!!\n");
		fm23_write_table(s_FmHandsetNormal,sizeof(s_FmHandsetNormal)/sizeof(REG_TABLE));
        return 0;

    case SET_FM_HANDFREE_NOR:
		log("[FM23] SET_FM_HANDFREE_NOR procedure!!!!!\n");
		fm23_write_table(s_FmHandFreeNormal,sizeof(s_FmHandFreeNormal)/sizeof(REG_TABLE));
        return 0;		
		
    default:
        log("[FM23]IOCTL: Command not found!\n");
        return -1;
    }

}

static int fm23_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
    int result = 0;
    log("[FM23] fm23 ioctl \n");

	result=fm23_set_procedure(cmd);
	
    return result;
}

static void __exit fm23_exit(void)
{
    i2c_del_driver(&fm23_driver);
}

static int __init fm23_init(void)
{
    int res=0;
    res = i2c_add_driver(&fm23_driver);
    if (res) {
        log("[FM23]i2c_add_driver failed! \n");
        return res;
    }

    printk("[FM23] fm23-380 device init ok!\n");
    return 0;
}

module_init(fm23_init);
module_exit(fm23_exit);

MODULE_AUTHOR("Wayne Lin");
MODULE_DESCRIPTION("FM23 driver");
MODULE_LICENSE("GPL");

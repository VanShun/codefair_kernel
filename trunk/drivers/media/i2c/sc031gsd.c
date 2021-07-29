/* SPDX-License-Identifier: GPL-2.0
 *
 * Copyright (c) 2020 Codefair Semiconductor Technology Co., Ltd.
 *		http://www.codefairsemi.com
 *
 */
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/sysfs.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <dt-bindings/gpio/gpio.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>


/* 45Mhz * 4 Binning */
#define SC031D_PIXEL_RATE		(45 * 1000 * 1000 * 4)
#define SC031D_XVCLK_FREQ		24000000

#define CHIP_ID				0x5648
#define SC031D_CHIP_ID_H	(0x56)
#define SC031D_CHIP_ID_L	(0x48)

#define SC031D_REG_CHIP_ID		0x300a

#define SC031D_REG_CTRL_MODE		0x0100
#define SC031D_MODE_SW_STANDBY		0x0
#define SC031D_MODE_STREAMING		BIT(0)

#define SC031D_REG_EXPOSURE		0x3500
#define	SC031D_EXPOSURE_MIN		4
#define	SC031D_EXPOSURE_STEP		1
#define SC031D_VTS_MAX			0x7fff

#define SC031D_REG_ANALOG_GAIN		0x3509
#define	ANALOG_GAIN_MIN			0x10
#define	ANALOG_GAIN_MAX			0xf8
#define	ANALOG_GAIN_STEP		1
#define	ANALOG_GAIN_DEFAULT		0xf8

#define SC031D_REG_DIGI_GAIN_H		0x350a
#define SC031D_REG_DIGI_GAIN_L		0x350b
#define SC031D_DIGI_GAIN_L_MASK		0x3f
#define SC031D_DIGI_GAIN_H_SHIFT	6
#define SC031D_DIGI_GAIN_MIN		0
#define SC031D_DIGI_GAIN_MAX		(0x4000 - 1)
#define SC031D_DIGI_GAIN_STEP		1
#define SC031D_DIGI_GAIN_DEFAULT	1024

#define SC031D_REG_TEST_PATTERN		0x4503
#define	SC031D_TEST_PATTERN_ENABLE	0x80
#define	SC031D_TEST_PATTERN_DISABLE	0x0

#define SC031D_REG_VTS			0x380e

#define REG_NULL			0xFFFF

#define SC031D_REG_VALUE_08BIT		1
#define SC031D_REG_VALUE_16BIT		2
#define SC031D_REG_VALUE_24BIT		3

#define SC031D_LANES			2
#define SC031D_BITS_PER_SAMPLE		10

#define SC031D_CNT   1
#define DEV_NAME	"sc031d"

//#define DVP_FLAG
 #define MIPI_FLAG

#define SEN_MAGIC 'N'
#define IOCTL_SENINIT _IOW (SEN_MAGIC, 1, int)
#define IOCTL_SENREGW _IOW (SEN_MAGIC, 2, int)
#define IOCTL_SENREGR _IOW (SEN_MAGIC, 3, int)
#define IOCTL_SENREGWB _IOW (SEN_MAGIC, 4, int)

static const char * const SC031D_supply_names[] = {
	"avdd",		/* Analog power */
	"dovdd",	/* Digital I/O power */
	"dvdd",		/* Digital core power */
};

#define SC031D_NUM_SUPPLIES ARRAY_SIZE(SC031D_supply_names)

struct regval {
	u16 addr;
	u8 val;
};
struct ioctl_data {		
	unsigned int reg;
	unsigned char value;
	unsigned char bit;
};

struct SC031D_mode {
	u32 width;
	u32 height;
	u32 max_fps;
	u32 hts_def;
	u32 vts_def;
	u32 exp_def;
	const struct regval *reg_list;
};
#if 0
struct SC031D {
	struct i2c_client	*client;
	struct clk		*xvclk;
	struct gpio_desc	*pwdn_gpio;
	struct regulator_bulk_data supplies[SC031D_NUM_SUPPLIES];

	struct v4l2_subdev	subdev;
	struct media_pad	pad;
	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl	*exposure;
	struct v4l2_ctrl	*anal_gain;
	struct v4l2_ctrl	*digi_gain;
	struct v4l2_ctrl	*hblank;
	struct v4l2_ctrl	*vblank;
	struct v4l2_ctrl	*test_pattern;
	struct mutex		mutex;
	bool			streaming;
	const struct SC031D_mode *cur_mode;
};
#endif
struct SC031D {
	dev_t devid;			/*  	 */
	struct cdev cdev;		/* cdev 	*/
	struct class *class;	/*  		*/
	struct device *device;	/* 	 */
	struct device_node	*nd; /*  */
	int major;			/*  */
	void *private_data;	/*  */
	int chipid, als, ps;		/*  */
	struct i2c_client *client;
	// struct gpio_desc	*pwdn_gpio;
};
static struct SC031D SC031Ddevd;

// #define to_SC031D(sd) container_of(sd, struct SC031D, subdev)

/*
 * Xclk 24Mhz
 * Pclk 45Mhz
 * linelength 672(0x2a0)
 * framelength 2232(0x8b8)
 * grabwindow_width 1296
 * grabwindow_height 972
 * max_framerate 30fps
 * mipi_datarate per lane 840Mbps
 */
static const struct regval SC031D_global_regs[] = {
// {0x0103,0x01},//1:reset
#if 0
{0x0103,0x01},//1:reset
{0x0100,0x00},
{0x300f,0x0f},
{0x3018,0x1f},
{0x3019,0xff},
{0x301c,0xb4},
{0x3028,0x82},
{0x320c,0x03},
{0x320d,0x6e},
{0x320e,0x0a},
{0x320f,0xac},
{0x3220,0x10},
{0x3250,0xf0},
{0x3251,0x02},
{0x3252,0x0a},
{0x3253,0xa7},
{0x3254,0x02},
{0x3255,0x07},
{0x3304,0x48},
{0x3306,0x38},
{0x3309,0x68},
{0x330b,0xe0},
{0x330c,0x18},
{0x330f,0x20},
{0x3310,0x10},
{0x3314,0x42},
{0x3315,0x38},
{0x3316,0x48},
{0x3317,0x20},
{0x3329,0x3c},
{0x332d,0x3c},
{0x332f,0x40},
{0x3335,0x44},
{0x3344,0x44},
{0x335b,0x80},
{0x335f,0x80},
{0x3366,0x06},
{0x3385,0x31},
{0x3387,0x51},
{0x3389,0x01},
{0x33b1,0x03},
{0x33b2,0x06},
{0x3621,0xa4},
{0x3622,0x05},
{0x3624,0x47},
{0x3630,0x46},
{0x3631,0x48},
{0x3633,0x52},
{0x3635,0x18},
{0x3636,0x25},
{0x3637,0x89},
{0x3638,0x0f},
{0x3639,0x08},
{0x363a,0x00},
{0x363b,0x48},
{0x363c,0x06},
{0x363d,0x00},
{0x363e,0xf8},
{0x3640,0x00},
{0x3641,0x01},

{0x36e9,0x00},
{0x36ea,0x3b},
{0x36eb,0x1a},
{0x36ec,0x0a},
{0x36ed,0x33},
{0x36f9,0x00},
{0x36fa,0x3a},
{0x36fc,0x01},
{0x3908,0x91},//add
{0x3d08,0x02},		//01  polarity
{0x3e01,0x14},
{0x3e02,0x80},
{0x3e06,0x0c},
{0x4500,0x59},
// {0x4501,0xc1},
////test pattern
{0x5011,0x00},
{0x3e08,0x04},
{0x3e09,0x10},
{0x3e06,0x00},
{0x3e07,0x80},
//{0x3221,0x66},
//{0x3221,0x06},
{0x0100,0x01},
{0x4418,0x08},
{0x4419,0x8e},
#endif
#if 1
{0x0100,0x00},
{0x300f,0x0f},
{0x3018,0x1f},
{0x3019,0xff},
{0x301c,0xb4},
{0x3028,0x82},
{0x320c,0x03},
{0x320d,0x6e},
{0x320e,0x02},
{0x320f,0xab},
{0x3220,0x10},
{0x3250,0xf0},
{0x3251,0x02},
{0x3252,0x02},
{0x3253,0xa6},
{0x3254,0x02},
{0x3255,0x07},
{0x3304,0x48},
{0x3306,0x38},
{0x3309,0x68},
{0x330b,0xe0},
{0x330c,0x18},
{0x330f,0x20},
{0x3310,0x10},
{0x3314,0x42},
{0x3315,0x38},
{0x3316,0x48},
{0x3317,0x20},
{0x3329,0x3c},
{0x332d,0x3c},
{0x332f,0x40},
{0x3335,0x44},
{0x3344,0x44},
{0x335b,0x80},
{0x335f,0x80},
{0x3366,0x06},
{0x3385,0x31},
{0x3387,0x51},
{0x3389,0x01},
{0x33b1,0x03},
{0x33b2,0x06},
{0x3621,0xa4},
{0x3622,0x05},
{0x3624,0x47},
{0x3630,0x46},
{0x3631,0x48},
{0x3633,0x52},
{0x3635,0x18},
{0x3636,0x25},
{0x3637,0x89},
{0x3638,0x0f},
{0x3639,0x08},
{0x363a,0x00},
{0x363b,0x48},
{0x363c,0x06},
{0x363d,0x00},
{0x363e,0xf8},
{0x3640,0x01},//0  PCLK DLY modify zhangxa
{0x3641,0x02},// 1,2
{0x36e9,0x00},
{0x36ea,0x3b},
{0x36eb,0x1a},
{0x36ec,0x0a},
{0x36ed,0x33},
{0x36f9,0x00},
{0x36fa,0x3a},
{0x36fc,0x01},

//{0x3d01,0x01},
{0x3d08,0x02},//2

{0x3e01,0x1f}, //line count
{0x3e02,0x40},
{0x3e06,0x0c},
{0x4500,0x59},
{0x4501,0xc4},
{0x5011,0x00},

{0x3900,0x01},
{0x3902,0x40},
{0x3908,0x10},  // target blc

{0x3e08,0x04},  //Agains   1X:0   X2:4   X4:C   8X  1C
{0x3e09,0x10},
{0x3e06,0x00},  //Dgains   1X:0   X2:1   X4:3
{0x3e07,0x80},

#ifdef TEST_PATTERN
{0x4501,0x08},
{0x3902,0x00},
{0x3e06,0x07},
#endif
{0x0100,0x01},
{0x4418,0x08},
{0x4419,0x8e},
#if 0
{0x3900,0x01},
{0x3902,0x40},
{0x3908,0x10},  // target blc

{0x3e08,0x0C},  //Agains   1X:0   X2:4   X4:C   8X  1C
{0x3e09,0x10},
{0x3e06,0x01},  //Dgains   1X:0   X2:1   X4:3
{0x3e07,0x80},

#ifdef TEST_PATTERN
{0x4501,0x08},
{0x3902,0x00},
{0x3e06,0x07},
#endif
#endif

#endif
{REG_NULL, 0x00},
};



#if 0
static const struct SC031D_mode supported_modes[] = {
	{
		.width = 2592,
		.height = 1944,
		.max_fps = 30,
		.exp_def = 0x0450,
		.hts_def = 0x02e4 * 4,
		.vts_def = 0x07e8,
		.reg_list = SC031D_2592x1944_regs,
	},
	{
		.width = 1920,
		.height = 1080,
		.max_fps = 30,
		.exp_def = 0x0450,
		.hts_def = 0x02a0 * 4,
		.vts_def = 0x08b8,
		.reg_list = SC031D_1920x1080_regs,
	},
	{
		.width = 1296,
		.height = 972,
		.max_fps = 60,
		.exp_def = 0x03e0,
		.hts_def = 0x02e4 * 4,
		.vts_def = 0x03f4,
		.reg_list = SC031D_1296x972_regs,
	},
	{
		.width = 1280,
		.height = 720,
		.max_fps = 30,
		.exp_def = 0x0450,
		.hts_def = 0x02a0 * 4,
		.vts_def = 0x08b8,
		.reg_list = SC031D_1280x720_regs,
	},
	{
		.width = 640,
		.height = 480,
		.max_fps = 120,
		.exp_def = 0x0450,
		.hts_def = 0x02a0 * 4,
		.vts_def = 0x022e,
		.reg_list = SC031D_640x480_regs,
	},
};
#endif
#define SC031D_LINK_FREQ_420MHZ		420000000
static const s64 link_freq_menu_items[] = {
	SC031D_LINK_FREQ_420MHZ
};

static const char * const SC031D_test_pattern_menu[] = {
	"Disabled",
	"Vertical Color Bar Type 1",
	"Vertical Color Bar Type 2",
	"Vertical Color Bar Type 3",
	"Vertical Color Bar Type 4"
};
static void __SC031D_power_off(struct i2c_client *client)
{
	int pwdnnum = 0;
	enum of_gpio_flags  flags;
	int ret;
	// struct i2c_client *client = to_i2c_client(dev);
	pwdnnum= of_get_named_gpio_flags(client->dev.of_node, "pwdn-d031", 0, &flags);
	if (gpio_is_valid(pwdnnum)) {
		printk("pmoff-gpio = %d\n", pwdnnum);
		ret = gpio_request(pwdnnum, "pwdn-d031");
		// printk("pwdn-gpio request = %d\n", ret);
		gpio_direction_output(pwdnnum, 1);
		gpio_set_value(pwdnnum, 1);
		mdelay(20);
		gpio_set_value(pwdnnum, 0);
		gpio_free(pwdnnum);
	} 
	else{
    printk("gpio PWDN fail\n");
	}
	return ;
}

static int __SC031D_power_on(struct i2c_client *client)
{   
    int pwdnnum = 0;
    int ret;
    enum of_gpio_flags  flags;
    // struct i2c_client *client = to_i2c_client(dev);
	pwdnnum= of_get_named_gpio_flags(client->dev.of_node, "pwdn-d031", 0, &flags);
	if (gpio_is_valid(pwdnnum)) {
		printk("pmon pwdn-gpio = %d\n", pwdnnum);
		ret = gpio_request(pwdnnum, "pwdn-d031");
		// printk("pwdn-gpio request = %d\n", ret);
		gpio_direction_output(pwdnnum, 1);
		gpio_set_value(pwdnnum, 0);
		mdelay(20);
		gpio_set_value(pwdnnum, 1);
		gpio_free(pwdnnum);
	} 
	else{
    printk("gpio PWDN fail\n");
	}
	return 0;
}
static int SC031D_read_reg(struct SC031D *dev,  unsigned short reg,
		void *val)
{
	// struct i2c_client *client = v4l2_get_subdevdata(sd);
	// struct i2c_client *client = (struct i2c_client *)SC031Ddevdd->private_data;
	struct i2c_client *client = (struct i2c_client *)dev->private_data;
	// unsigned char buf= reg;
	// struct i2c_msg msg[2];
	unsigned char buf[2] = {reg >> 8, reg & 0xff};
	struct i2c_msg msg[2] = {
		[0] = {
			.addr	= client->addr,
			.flags	= 0,
			.len	= 2,
			.buf	= buf,
		},
		[1] = {
			.addr	= client->addr,
			.flags	= I2C_M_RD,
			.len	= 1,
			.buf	= val,
		}
	};
	int ret;
	 // printk("read\n");
	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret > 0)
		ret = 0;

	return ret;
}
static int SC031D_read_array(struct SC031D *dev,const struct regval *regs)
{
	u32 i;
	int ret = 0;
	// unsigned char buf[200]={0};
	u8 data = 0;
	// unsigned char *value;
     // struct i2c_client *client = (struct i2c_client *)dev->private_data;
	for (i = 0; ret == 0 && regs[i].addr != REG_NULL; i++)
	{
		// printk("i2c read\n");
        ret = SC031D_read_reg(dev, regs[i].addr, &data);
        printk("SC031D : %x = %x\n",regs[i].addr,data);

	}
	return ret;
}

static int SC031D_write_reg(struct SC031D *dev, unsigned short reg,unsigned char value)
{
	struct i2c_client *client = (struct i2c_client *)dev->private_data;
	unsigned char buf[3] = {reg >> 8, reg & 0xff, value};
	struct i2c_msg msg = {
		.addr	= client->addr,
		.flags	= 0,
		.len	= 3,
		.buf	= buf,
	};
	int ret;

	ret = i2c_transfer(client->adapter, &msg, 1);
	// if (ret > 0)
	// 	ret = 0;
	if(ret == 1) {
	ret = 0;
	} else {
	printk("i2c rd failed=%d reg=0x%06x value=%d\n",ret, reg, value);
	ret = -EREMOTEIO;
	}
	return ret;
}
#if 0
static int SC031D_write_array(struct SC031D *dev,const struct regval *regs)
{
	u32 i;
	int ret = 0;
     // struct i2c_client *client = (struct i2c_client *)dev->private_data;
	for (i = 0; ret == 0 && regs[i].addr != REG_NULL; i++)
	{
		printk("SC031D : %x = %x\n",regs[i].addr,regs[i].val );
        ret = SC031D_write_reg(dev, regs[i].addr, regs[i].val);
	}
	return ret;
}
#endif

static int SC031D_write_array(struct SC031D *dev,const struct regval *regs)
{
	u32 i;
	int ret = 0;
	u8 data = 0;
     // struct i2c_client *client = (struct i2c_client *)dev->private_data;
	for (i = 0; ret == 0 && regs[i].addr != REG_NULL; i++)
	{
        ret = SC031D_write_reg(dev, regs[i].addr, regs[i].val);
        printk("SC031D : %x = %x\n",regs[i].addr,regs[i].val );
        ret = SC031D_read_reg(dev, regs[i].addr, &data);
        printk("SC031D : %x = %x\n",regs[i].addr,data);
        if(data==regs[i].val)
        {
         printk("rw reg ok\n");
        }
	}
	return ret;
}



static int __SC031D_start_stream(struct SC031D *dev)
{
	int ret=0;
	ret = SC031D_write_array(dev,SC031D_global_regs);//yuan
	printk("SC031D init dvp \n");
	return ret;
}

static int __SC031D_stop_stream(struct SC031D *dev)
{
	int ret;
	struct i2c_client *client = (struct i2c_client *)dev->private_data;
	__SC031D_power_off(client);
	// ret = SC031D_write_reg(dev, SC031D_REG_CTRL_MODE,SC031D_MODE_SW_STANDBY);
	// if(ret)
	// printk("SC031D stop error\n");	

	return ret;
}


static inline u32 SC031D_cal_delay(u32 cycles)
{
	return DIV_ROUND_UP(cycles, SC031D_XVCLK_FREQ / 1000 / 1000);
}


static int SC031D_detect(struct SC031D *dev)
{
	unsigned char v;
	int ret;
	int id=0;

	ret = SC031D_read_reg(dev, 0x320a, &v);
	if (ret < 0)
		return ret;
	id|=v;
	if (v != SC031D_CHIP_ID_H)
		return -ENODEV;
	ret = SC031D_read_reg(dev, 0x320b, &v);
	if (ret < 0)
		return ret;
	id=(id<<8)|v;
	// if (v != SC031D_CHIP_ID_L)
	// 	return -ENODEV;
	dev->chipid=id;
	printk("discover 0X%x\n", id);
	return 0;
}

/*
 * @description		: 
 * @param - inode 	: 
 * @param - filp 	: 
 * 					  
 * @return 			:
 */
static int SC031D_open(struct inode *inode, struct file *filp)
{
	struct i2c_client *client = (struct i2c_client *)SC031Ddevd.private_data;
	filp->private_data = &SC031Ddevd;
	__SC031D_power_on(client);
	__SC031D_start_stream(&SC031Ddevd);
	return 0;
}

/*
 * @description		: 
 * @param - filp 	: 
 * @param - buf 	: 
 * @param - cnt 	: 
 * @param - offt 	: 
 * @return 			: 
 */
static ssize_t SC031D_read(struct file *filp, char __user *buf, size_t cnt, loff_t *off)
{
	int data[1]={0};
	long err = 0;
	// int chipidnum;

	struct SC031D *dev = (struct SC031D *)filp->private_data;
	
	// SC031D_readdata(dev);
	// SC031D_detect(dev);

	data[0] = dev->chipid;
	// data[1] = dev->als;
	// data[2] = dev->ps;
	err = copy_to_user(buf, data, sizeof(data[1]));
	printk("Readid OV%x sensor\n", data[0]);
	return 0;
}
// ssize_t (*write) (struct file *, const char __user *, size_t, loff_t *);
static ssize_t SC031D_write(struct file *filp, const char __user *buf, size_t cnt, loff_t *off)
{
	struct regval data[]={0};
	long err = 0;
	int ret;
    static struct regval SC031D_data_regs[100]={0};
	struct SC031D *dev = (struct SC031D *)filp->private_data;
	

	err = copy_from_user(data, buf, sizeof(buf));
	if(err < 0) {
	printk("kernel write failed!\r\n");
	return -EFAULT;
    }
    // memcpy(&SC031D_data_regs,(struct regval *)data,sizeof(data));
    memcpy(SC031D_data_regs,&data,sizeof(data[100]));
    ret = SC031D_write_array(dev,SC031D_data_regs);
	if(ret)
		printk("SC031D write error\n");	
	return 0;
}

/*
 * @description		: 
 * @param - filp 	: 
 * @return 			: 
 */
static int SC031D_release(struct inode *inode, struct file *filp)
{
	__SC031D_stop_stream(&SC031Ddevd);
	return 0;
}


static long senioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int ret,err;
	struct ioctl_data sendata;
	unsigned char kvalue=0;
    struct SC031D *dev = (struct SC031D *)filp->private_data;
    
    switch(cmd) {
		case IOCTL_SENINIT:
           break;
			
        case IOCTL_SENREGW:
            err = copy_from_user(&sendata, (struct ioctl_data *)arg, sizeof(struct ioctl_data));
            	if(err < 0) {
	        printk("senser write failed!\r\n");
	        return -EFAULT;
            }
            SC031D_write_reg(dev, sendata.reg, sendata.value);
           	printk("sen write reg %x=%x\n",sendata.reg,sendata.value);
		 // printk("REGW AFTER %x=%x\n", 0xa0180000+sendata.reg,readl(global_csi->regs + sendata.reg) );
			break;

        case IOCTL_SENREGWB:
			// if(sendata.value == 1)
			// {
			// 	set_bit_w(sendata.bit,sendata.reg);
			// 	printk("sen write reg bit %x=bit%x\n",sendata.reg,sendata.value);
			// }
			// else
			// {
			// 	clear_bit_w(sendata.bit,sendata.reg);
			// 	printk("sen write reg bit %x=bit%x\n",sendata.reg,sendata.value);
			// }
           	break;
            
        case IOCTL_SENREGR:			 
			// sendata.value = readl(sendata.reg)
            // unsigned char kvalue=0;
			err = copy_from_user(&sendata, (struct ioctl_data *)arg, sizeof(struct ioctl_data));
            	if(err < 0) {
	        printk("senser read reg failed!\r\n");
	        return -EFAULT;
            }
            ret = SC031D_read_reg(dev, sendata.reg, &kvalue);
            if (ret < 0)
            {
            	return ret;
            }
		    sendata.value=kvalue;
			if(copy_to_user( (struct ioctl_data *)arg, &sendata, sizeof(struct ioctl_data))){
				ret = -1;
			 	printk("sendata to user error\n");
			}
			// ret = SC031D_read_reg(dev, sendata.reg, sendata.value);	
			printk("sen read out reg  %x=%x\n",sendata.reg,sendata.value);
			break;
        default:
            printk("SENSER in the default\n");
            return -EINVAL;
    }
    return 0;
}


static int __maybe_unused SC031D_runtime_resume(struct device *dev)
{  
	
	struct i2c_client *client = to_i2c_client(dev);
	printk("pmon-gpio\n");
	__SC031D_power_on(client);
	return 0;
}

static int __maybe_unused SC031D_runtime_suspend(struct device *dev)
{
   
   struct i2c_client *client = to_i2c_client(dev);
   printk("pmoff-gpio\n");
	__SC031D_power_off(client);

	return 0;
}
static const struct dev_pm_ops SC031D_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(SC031D_runtime_suspend,SC031D_runtime_resume)
	SET_RUNTIME_PM_OPS(SC031D_runtime_suspend,SC031D_runtime_resume, NULL)
};

/* SC031D */
static const struct file_operations SC031D_ops = {
	.owner = THIS_MODULE,
	.open = SC031D_open,
	.read = SC031D_read,
	.write = SC031D_write,
	.unlocked_ioctl  = senioctl,
	.release = SC031D_release,
};

 /*
  * @description     : 
  *                    
  * @param - client  : 
  * @param - id      : 
  * @return          : 
  */
static int SC031D_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	enum of_gpio_flags  flags;
	int pwdnnum = 0;
	int ret=0;
	// struct device *dev = &client->dev;
	/* major */
	if (SC031Ddevd.major) {
		SC031Ddevd.devid = MKDEV(SC031Ddevd.major, 0);
		register_chrdev_region(SC031Ddevd.devid, SC031D_CNT, DEV_NAME);
	} else {
		alloc_chrdev_region(&SC031Ddevd.devid, 0, SC031D_CNT, DEV_NAME);
		SC031Ddevd.major = MAJOR(SC031Ddevd.devid);
	}

	/* cdev */
	cdev_init(&SC031Ddevd.cdev, &SC031D_ops);
	cdev_add(&SC031Ddevd.cdev, SC031Ddevd.devid, SC031D_CNT);

	/* class */
	SC031Ddevd.class = class_create(THIS_MODULE, DEV_NAME);
	if (IS_ERR(SC031Ddevd.class)) {
		return PTR_ERR(SC031Ddevd.class);
	}

	/* device */
	SC031Ddevd.device = device_create(SC031Ddevd.class, NULL, SC031Ddevd.devid, NULL, DEV_NAME);
	if (IS_ERR(SC031Ddevd.device)) {
		return PTR_ERR(SC031Ddevd.device);
	}
 
	SC031Ddevd.private_data = client;
    pwdnnum= of_get_named_gpio_flags(client->dev.of_node, "sen-d031", 0, &flags);
	if (gpio_is_valid(pwdnnum)) {
		ret = gpio_request(pwdnnum, "sen-d031");
		gpio_direction_output(pwdnnum, 1);
		gpio_set_value(pwdnnum, 0);
		mdelay(20);
		// msleep(5);
		gpio_set_value(pwdnnum, 1);
		printk("SC031D reset\n");	
		gpio_free(pwdnnum);
	}
	pwdnnum= of_get_named_gpio_flags(client->dev.of_node, "pwdn-d031", 0, &flags);
	if (gpio_is_valid(pwdnnum)) {
		ret = gpio_request(pwdnnum, "pwdn-d031");
		gpio_direction_output(pwdnnum, 1);
		gpio_set_value(pwdnnum, 1);
		mdelay(20);
		// msleep(5);
		gpio_set_value(pwdnnum, 0);
		printk("SC031D pwdn\n");	
		gpio_free(pwdnnum);
	} 
	// else
	// 	pr_err("pwdn-d031 request fail\n");
	// SC031D_detect(&SC031Ddevd);	
	return 0;
}

/*
 * @description     : 
 * @param - client 	: 
 * @return          : 
 */
static int SC031D_remove(struct i2c_client *client)
{
	/*  */
	cdev_del(&SC031Ddevd.cdev);
	unregister_chrdev_region(SC031Ddevd.devid, SC031D_CNT);

	/* device class */
	device_destroy(SC031Ddevd.class, SC031Ddevd.devid);
	class_destroy(SC031Ddevd.class);
	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev)){
		__SC031D_power_off(client);
	}
	
	pm_runtime_set_suspended(&client->dev);
	return 0;
}

/* i2c_device_id */
static const struct i2c_device_id SC031D_id[] = {
	{"ov,SC031D", 0},  
	{}
};

/* device_id */
static const struct of_device_id SC031D_of_match[] = {
	{ .compatible = "sm,sc031dvp" },
	{ /* Sentinel */ }
};

/* i2c_driver */	
static struct i2c_driver SC031D_driver = {
	.probe = SC031D_probe,
	.remove = SC031D_remove,
	.driver = {
			.owner = THIS_MODULE,
		   	.name = "SC031D",
		   	.pm = &SC031D_pm_ops,
		   	.of_match_table = SC031D_of_match, 
		   },
	.id_table = SC031D_id,
};
module_i2c_driver(SC031D_driver);		   
MODULE_DESCRIPTION("OmniVision SC031D sensor driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("linronghua,Codefair");



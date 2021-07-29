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
#define SC031_PIXEL_RATE		(45 * 1000 * 1000 * 4)
#define SC031_XVCLK_FREQ		24000000

#define CHIP_ID				0x5648
#define SC031_CHIP_ID_H	(0x56)
#define SC031_CHIP_ID_L	(0x48)

#define SC031_REG_CHIP_ID		0x300a

#define SC031_REG_CTRL_MODE		0x0100
#define SC031_MODE_SW_STANDBY		0x0
#define SC031_MODE_STREAMING		BIT(0)

#define SC031_REG_EXPOSURE		0x3500
#define	SC031_EXPOSURE_MIN		4
#define	SC031_EXPOSURE_STEP		1
#define SC031_VTS_MAX			0x7fff

#define SC031_REG_ANALOG_GAIN		0x3509
#define	ANALOG_GAIN_MIN			0x10
#define	ANALOG_GAIN_MAX			0xf8
#define	ANALOG_GAIN_STEP		1
#define	ANALOG_GAIN_DEFAULT		0xf8

#define SC031_REG_DIGI_GAIN_H		0x350a
#define SC031_REG_DIGI_GAIN_L		0x350b
#define SC031_DIGI_GAIN_L_MASK		0x3f
#define SC031_DIGI_GAIN_H_SHIFT	6
#define SC031_DIGI_GAIN_MIN		0
#define SC031_DIGI_GAIN_MAX		(0x4000 - 1)
#define SC031_DIGI_GAIN_STEP		1
#define SC031_DIGI_GAIN_DEFAULT	1024

#define SC031_REG_TEST_PATTERN		0x4503
#define	SC031_TEST_PATTERN_ENABLE	0x80
#define	SC031_TEST_PATTERN_DISABLE	0x0

#define SC031_REG_VTS			0x380e

#define REG_NULL			0xFFFF

#define SC031_REG_VALUE_08BIT		1
#define SC031_REG_VALUE_16BIT		2
#define SC031_REG_VALUE_24BIT		3

#define SC031_LANES			2
#define SC031_BITS_PER_SAMPLE		10

#define SC031_CNT   1
#define DEV_NAME	"SC031M"


#define MIPI_FLAG

#define SEN_MAGIC 'N'
#define IOCTL_SENINIT _IOW (SEN_MAGIC, 1, int)
#define IOCTL_SENREGW _IOW (SEN_MAGIC, 2, int)
#define IOCTL_SENREGR _IOW (SEN_MAGIC, 3, int)
#define IOCTL_SENREGWB _IOW (SEN_MAGIC, 4, int)

static const char * const SC031_supply_names[] = {
	"avdd",		/* Analog power */
	"dovdd",	/* Digital I/O power */
	"dvdd",		/* Digital core power */
};

#define SC031_NUM_SUPPLIES ARRAY_SIZE(SC031_supply_names)

struct regval {
	u16 addr;
	u8 val;
};
struct ioctl_data {		
	unsigned int reg;
	unsigned char value;
	unsigned char bit;
};

struct SC031_mode {
	u32 width;
	u32 height;
	u32 max_fps;
	u32 hts_def;
	u32 vts_def;
	u32 exp_def;
	const struct regval *reg_list;
};
#if 0
struct SC031 {
	struct i2c_client	*client;
	struct clk		*xvclk;
	struct gpio_desc	*pwdn_gpio;
	struct regulator_bulk_data supplies[SC031_NUM_SUPPLIES];

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
	const struct SC031_mode *cur_mode;
};
#endif
struct SC031 {
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
static struct SC031 SC031dev;

// #define to_SC031(sd) container_of(sd, struct SC031, subdev)

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
static const struct regval SC031_global_regs[] = {
// {0x0103,0x01},//1:reset
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
{0x3e08,0x00},
{0x3e09,0x18},
{0x3e06,0x00},
{0x3e07,0x80},
//{0x3221,0x66},
//{0x3221,0x06},
{0x0100,0x01},
{0x4418,0x08},
{0x4419,0x8e},
{REG_NULL, 0x00},
};


static const struct regval SC031_globalm_regs[] = {
{0x0103,0x01},//1:reset
{0x0100,0x00},// 0 :sleep
{0x4603,0x00},//[0] mipi read from fifo enable
{0x3000,0x00},//0:MIPI pad 1:DVP pad
{0x3001,0x00},//hff:DVP pad out ,00:MIPI pad
{0x303f,0x01},//[7] pll pclk dis
{0x3018,0x13},//[7:5] lane num = [7:5] + 1 , [3:2] pd_mipi
{0x3031,0x0a},//[3:0] mipi bit num  raw10 mode
// {0x3031,0x08},
{0x3037,0x20},//[6:5] phy bit num 00~8bit 01~10bit 10~12bit
{0x301c,0x78},//[7] pclk_dvp_en [6] pclk_mipi_en [3] rst_dvp [2] rst_mipi//?
{0x3019,0xfe},//[3:0] lane_disable
{0x363c,0x08},
{0x3630,0x82},
{0x3638,0x0f}, //ramp offset
{0x3639,0x08}, //ramp_vref
{0x335b,0x80}, //pix_samp all high
{0x3636,0x25},
{0x3640,0x02},
{0x3306,0x38},
{0x3304,0x48},
{0x3389,0x01}, //offset en
{0x3385,0x31}, //integ en
{0x330c,0x18}, //0109
{0x3315,0x38},
{0x3306,0x28},
{0x3309,0x68},
{0x3387,0x51}, //integ_en
{0x3306,0x48},
{0x3366,0x04}, //sb over count up
{0x335f,0x80}, //avdd samp all high
{0x363a,0x00}, // psr
{0x3622,0x01},
{0x3633,0x62},
//bsi
{0x36f9,0x20},
//0x3250,0x}f0,
{0x3637,0x80},
{0x363d,0x04},
{0x3e06,0x00},
{0x363c,0x48},
{0x320c,0x03},//hang data number ? h
{0x320e,0x0e},//A frame hang number ?h
{0x320f,0xa8},//A frame hang number ?l
{0x3306,0x38},
{0x330b,0xb6},
//0x3621,0x}90,
{0x36f9,0x24},
//0104
{0x363b,0x4a},
{0x3366,0x02},
//precharge}
{0x3316,0x78},
{0x3344,0x74},
{0x3335,0x74},
{0x332f,0x70},
{0x332d,0x6c},
{0x3329,0x6c},
//0105
//0x3250,0x}ff,
//1.24
{0x363c,0x08},
{0x3630,0x81},
//0x3314,0x}28, //for offset cancel
//0x3317,0x}23,
//0x3366,0x}04,
{0x3366,0x06},
{0x3314,0x3a},
{0x3317,0x28},
//0130
{0x3622,0x05}, //blksun
{0x363d,0x00},
{0x3637,0x86},
{0x3e01,0x62},
{0x3633,0x52},
{0x3630,0x86},
{0x3306,0x4c},
{0x330b,0xa0},
//0131
{0x3631,0x48},
//0201
{0x33b1,0x03}, // chis position
{0x33b2,0x06}, //[3:2]=chis_length [1:0] +1=trans_length
{0x320c,0x02},//?hang
{0x320e,0x02},//frame h
{0x320f,0x0d},//frame l
{0x3e01,0x20},//frame time h?
{0x3e02,0x20},//frame time l?
//precharge}
{0x3316,0x48},
{0x3344,0x44},
{0x3335,0x44},
{0x332f,0x40},
{0x332d,0x3c},
{0x3329,0x3c},
{0x3310,0x10}, //tx width
{0x3637,0x87},
{0x363e,0xf8},
//power sav}e mode  0201B
{0x3254,0x02},
{0x3255,0x07},  //{3204,3205}={3260,3261}+1
{0x3252,0x02},
{0x3253,0xa6},  // {3252,3253}={320e,320f}-5
{0x3250,0xc0},  //[5:4]sa1 [3:2] opbuf  [1:0] ramp 0724
{0x3251,0x02},  //[1] opbuf  [0] ramp
//0322
{0x330f,0x50},//
{0x3630,0x46},
//0324
{0x3621,0xa2}, //aa?
//0329
{0x3621,0xa0},
{0x4500,0x59}, //fifo delay
//0404
{0x3637,0x88}, //blksun margin
{0x3908,0x81}, //for digital gain//BLC target?
{0x3640,0x00},//
{0x3641,0x02},//
{0x363c,0x05}, //only for vt++ com1 dvp sample
{0x363b,0x4c},
//72MPCLK 183fps
{0x36e9,0x40},//
{0x36ea,0x36},
{0x36ed,0x13},
{0x36fa,0x38},
{0x330b,0x80},
//0x3304,0x40,
//0x3389,0x01, //offset en
//0x3385,0x31, //integ en
//0x330c,0x10, //0109
//0x3315,0x30,
//0x3309,0x60,
//0x3387,0x51, //integ_en
//0x3306,0x50,
{0x3640,0x00},
{0x3641,0x01},
//0502
{0x3d08,0x00},
{0x3306,0x48},
{0x3621,0xa4},
{0x300f,0x0f},
//0612
{0x4837, 0x1b},  //0x4837=2000/PCLK
{0x4809, 0x01}, //clock lane lp initialized to 11
//0622  24M} input 72MPCLK 120fps
{0x363b,0x48},
{0x363c,0x06},
{0x36e9,0x00},
{0x36ea,0x3b},
{0x36eb,0x0e},
{0x36ec,0x0e},
{0x36ed,0x33},
{0x36f9,0x00},
{0x36fa,0x3a},
{0x36fc,0x01},
{0x320c,0x03}, //for high temp more margin hou
{0x320d,0x6e},//hang
{0x320e,0x03},//frame
{0x320f,0x00},//ame hou
{0x330b,0x80},
{0x330f,0x50},
{0x3637,0x89}, //blksun margin
{0x3641,0x01}, //0x01 for 2.8V dovdd 0x02 for 1.8V DOVDD
//0625 for 70C 4ms 16X gain
// {0x4501,0xc4},//
{0x3e03,0x03},
// {0x3e03,0x0b},
//{0x4501,0xc0},
{0x5011,0x01},
{0x3908,0x21},//lin
{0x3e01,0x10},//s 1480
{0x3e02,0x00},
{0x3306,0x38},
{0x330b,0xe0}, //high temp [a0,f0]
{0x330f,0x20},
//0626
{0x3d08,0x01},
//0724
{0x5011,0x00},
//{0x3e06,0x0c},
// {0x3900,0x01},
{0x3902,0x40},
//{0x3902,0x00},
//{0x3928,0x01},
{0x3908,0x20}, //blc 91
//0x3314,0x}3a,
//0x3317,0x}20,
{0x3314,0x1e},//
{0x3317,0x10},
//{0x3e08,0x00},
//{0x3e09,0x80},
{0x3e08,0x00},
{0x3e09,0x10},
{0x3e06,0x00},
{0x3e07,0x80},
//st light
{0x3635,0x18},
//  reduce minimum precharge
{0x3316,0x40},
{0x3344,0x3c},
{0x3335,0x3c},
{0x332f,0x38},
{0x332d,0x34},
{0x3329,0x34},
//20180829
{0x3624,0x47},
//20181101
{0x3220,0x10},//[1:0] vflip blc调整     1~HDR 模式打开lin
//20181106
{0x3635,0x18},//
//mipi
{0x4837,0x1b},
{0x4800,0x64},
{0x5988,0x02},
{0x598e,0x03},
{0x598f,0x3c},
{0x0100,0x01}, //1：sleep disable
{0x4418,0x08},//
{0x4419,0x8e},//20190222 cover avdd 2.6V-3.0V
{REG_NULL, 0x00},
};
#if 0
static const struct SC031_mode supported_modes[] = {
	{
		.width = 2592,
		.height = 1944,
		.max_fps = 30,
		.exp_def = 0x0450,
		.hts_def = 0x02e4 * 4,
		.vts_def = 0x07e8,
		.reg_list = SC031_2592x1944_regs,
	},
	{
		.width = 1920,
		.height = 1080,
		.max_fps = 30,
		.exp_def = 0x0450,
		.hts_def = 0x02a0 * 4,
		.vts_def = 0x08b8,
		.reg_list = SC031_1920x1080_regs,
	},
	{
		.width = 1296,
		.height = 972,
		.max_fps = 60,
		.exp_def = 0x03e0,
		.hts_def = 0x02e4 * 4,
		.vts_def = 0x03f4,
		.reg_list = SC031_1296x972_regs,
	},
	{
		.width = 1280,
		.height = 720,
		.max_fps = 30,
		.exp_def = 0x0450,
		.hts_def = 0x02a0 * 4,
		.vts_def = 0x08b8,
		.reg_list = SC031_1280x720_regs,
	},
	{
		.width = 640,
		.height = 480,
		.max_fps = 120,
		.exp_def = 0x0450,
		.hts_def = 0x02a0 * 4,
		.vts_def = 0x022e,
		.reg_list = SC031_640x480_regs,
	},
};
#endif
#define SC031_LINK_FREQ_420MHZ		420000000
static const s64 link_freq_menu_items[] = {
	SC031_LINK_FREQ_420MHZ
};

static const char * const SC031_test_pattern_menu[] = {
	"Disabled",
	"Vertical Color Bar Type 1",
	"Vertical Color Bar Type 2",
	"Vertical Color Bar Type 3",
	"Vertical Color Bar Type 4"
};
static int __SC031_power_on(struct i2c_client *client)
{   
    int pwdnnum = 0;
    int ret;
    enum of_gpio_flags  flags;
    // struct i2c_client *client = to_i2c_client(dev);
	pwdnnum= of_get_named_gpio_flags(client->dev.of_node, "pwdn-gpio", 0, &flags);
	if (gpio_is_valid(pwdnnum)) {
		printk("pmon pwdn-gpio = %d\n", pwdnnum);
		ret = gpio_request(pwdnnum, "pwdn-gpio");
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
static void __SC031_power_off(struct i2c_client *client)
{
	int pwdnnum = 0;
	enum of_gpio_flags  flags;
	int ret;
	// struct i2c_client *client = to_i2c_client(dev);
	pwdnnum= of_get_named_gpio_flags(client->dev.of_node, "pwdn-gpio", 0, &flags);
	if (gpio_is_valid(pwdnnum)) {
		printk("pmoff-gpio = %d\n", pwdnnum);
		ret = gpio_request(pwdnnum, "pwdn-gpio");
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

static int SC031_write_reg(struct SC031 *dev, unsigned short reg,unsigned char value)
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

static int SC031_write_array(struct SC031 *dev,const struct regval *regs)
{
	u32 i;
	int ret = 0;
     // struct i2c_client *client = (struct i2c_client *)dev->private_data;
	for (i = 0; ret == 0 && regs[i].addr != REG_NULL; i++)
	{
		printk("SC031 : %x = %x\n",regs[i].addr,regs[i].val );
        ret = SC031_write_reg(dev, regs[i].addr, regs[i].val);
	}
	return ret;
}



static int SC031_read_reg(struct SC031 *dev,  unsigned short reg,
		unsigned char *value)
{
	// struct i2c_client *client = v4l2_get_subdevdata(sd);
	// struct i2c_client *client = (struct i2c_client *)SC031dev->private_data;
	struct i2c_client *client = (struct i2c_client *)dev->private_data;
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
			.buf	= value,
		}
	};
	int ret;

	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret > 0)
		ret = 0;

	return ret;
}


static int __SC031_start_stream(struct SC031 *dev)
{
	int ret=0;
	#ifdef 	MIPI_FLAG
	ret = SC031_write_array(dev,SC031_globalm_regs);
	// ret = SC031_write_array(dev,SC031_640x480_regs);
		printk("SC031 init mipi \n");
	#endif
	return ret;
}

static int __SC031_stop_stream(struct SC031 *dev)
{
	int ret;
	ret = SC031_write_reg(dev, SC031_REG_CTRL_MODE,SC031_MODE_SW_STANDBY);
	if(ret)
	printk("SC031 stop error\n");	
	return ret;
}


static inline u32 SC031_cal_delay(u32 cycles)
{
	return DIV_ROUND_UP(cycles, SC031_XVCLK_FREQ / 1000 / 1000);
}


static int SC031_detect(struct SC031 *dev)
{
	unsigned char v;
	int ret;
	int id=0;

	ret = SC031_read_reg(dev, 0x320a, &v);
	if (ret < 0)
		return ret;
	id|=v;
	if (v != SC031_CHIP_ID_H)
		return -ENODEV;
	ret = SC031_read_reg(dev, 0x320b, &v);
	if (ret < 0)
		return ret;
	id=(id<<8)|v;
	// if (v != SC031_CHIP_ID_L)
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
static int SC031_open(struct inode *inode, struct file *filp)
{
	filp->private_data = &SC031dev;
	struct i2c_client *client = (struct i2c_client *)SC031dev.private_data;
	__SC031_power_on(client);
	__SC031_start_stream(&SC031dev);
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
static ssize_t SC031_read(struct file *filp, char __user *buf, size_t cnt, loff_t *off)
{
	int data[1]={0};
	long err = 0;
	// int chipidnum;

	struct SC031 *dev = (struct SC031 *)filp->private_data;
	
	// SC031_readdata(dev);
	// SC031_detect(dev);

	data[0] = dev->chipid;
	// data[1] = dev->als;
	// data[2] = dev->ps;
	err = copy_to_user(buf, data, sizeof(data[1]));
	printk("Readid OV%x sensor\n", data[0]);
	return 0;
}
// ssize_t (*write) (struct file *, const char __user *, size_t, loff_t *);
static ssize_t SC031_write(struct file *filp, const char __user *buf, size_t cnt, loff_t *off)
{
	struct regval data[]={0};
	long err = 0;
	int ret;
    static const struct regval SC031_data_regs[100]={0};
	struct SC031 *dev = (struct SC031 *)filp->private_data;
	

	err = copy_from_user(data, buf, sizeof(buf));
	if(err < 0) {
	printk("kernel write failed!\r\n");
	return -EFAULT;
    }
    // memcpy(&SC031_data_regs,(struct regval *)data,sizeof(data));
    memcpy(&SC031_data_regs,&data,sizeof(data[100]));
    ret = SC031_write_array(dev,SC031_data_regs);
	if(ret)
		printk("SC031 write error\n");	
	return 0;
}

/*
 * @description		: 
 * @param - filp 	: 
 * @return 			: 
 */
static int SC031_release(struct inode *inode, struct file *filp)
{
	// __SC031_stop_stream(&SC031dev);
	struct i2c_client *client = (struct i2c_client *)SC031dev.private_data;
	__SC031_power_off(client);
	return 0;
}


static long senioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int ret,err;
	struct ioctl_data sendata;
	unsigned char kvalue=0;
    struct SC031 *dev = (struct SC031 *)filp->private_data;
    
    switch(cmd) {
		case IOCTL_SENINIT:
           break;
			
        case IOCTL_SENREGW:
            err = copy_from_user(&sendata, (struct ioctl_data *)arg, sizeof(struct ioctl_data));
            	if(err < 0) {
	        printk("senser write failed!\r\n");
	        return -EFAULT;
            }
            SC031_write_reg(dev, sendata.reg, sendata.value);
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
            ret = SC031_read_reg(dev, sendata.reg, &kvalue);
            if (ret < 0)
            {
            	return ret;
            }
		    sendata.value=kvalue;
			if(copy_to_user( (struct ioctl_data *)arg, &sendata, sizeof(struct ioctl_data))){
				ret = -1;
			 	printk("sendata to user error\n");
			}
			// ret = SC031_read_reg(dev, sendata.reg, sendata.value);	
			printk("sen read out reg  %x=%x\n",sendata.reg,sendata.value);
			break;
        default:
            printk("SENSER in the default\n");
            return -EINVAL;
    }
    return 0;
}


static int __maybe_unused SC031_runtime_resume(struct device *dev)
{  
	
	struct i2c_client *client = to_i2c_client(dev);
	printk("pmon-gpio\n");
	__SC031_power_on(client);
	return 0;
}

static int __maybe_unused SC031_runtime_suspend(struct device *dev)
{
   
   struct i2c_client *client = to_i2c_client(dev);
   printk("pmoff-gpio\n");
	__SC031_power_off(client);

	return 0;
}
static const struct dev_pm_ops SC031_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(SC031_runtime_suspend,SC031_runtime_resume)
	SET_RUNTIME_PM_OPS(SC031_runtime_suspend,SC031_runtime_resume, NULL)
};

/* SC031 */
static const struct file_operations SC031_ops = {
	.owner = THIS_MODULE,
	.open = SC031_open,
	.read = SC031_read,
	.write = SC031_write,
	.unlocked_ioctl  = senioctl,
	.release = SC031_release,
};

 /*
  * @description     : 
  *                    
  * @param - client  : 
  * @param - id      : 
  * @return          : 
  */
static int SC031_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	enum of_gpio_flags  flags;
	int pwdnnum = 0;
	int ret=0;
	// struct device *dev = &client->dev;
	/* major */
	if (SC031dev.major) {
		SC031dev.devid = MKDEV(SC031dev.major, 0);
		register_chrdev_region(SC031dev.devid, SC031_CNT, DEV_NAME);
	} else {
		alloc_chrdev_region(&SC031dev.devid, 0, SC031_CNT, DEV_NAME);
		SC031dev.major = MAJOR(SC031dev.devid);
	}

	/* cdev */
	cdev_init(&SC031dev.cdev, &SC031_ops);
	cdev_add(&SC031dev.cdev, SC031dev.devid, SC031_CNT);

	/* class */
	SC031dev.class = class_create(THIS_MODULE, DEV_NAME);
	if (IS_ERR(SC031dev.class)) {
		return PTR_ERR(SC031dev.class);
	}

	/* device */
	SC031dev.device = device_create(SC031dev.class, NULL, SC031dev.devid, NULL, DEV_NAME);
	if (IS_ERR(SC031dev.device)) {
		return PTR_ERR(SC031dev.device);
	}
 
	SC031dev.private_data = client;
    pwdnnum= of_get_named_gpio_flags(client->dev.of_node, "sen-reset", 0, &flags);
	if (gpio_is_valid(pwdnnum)) {
		ret = gpio_request(pwdnnum, "sen-reset");
		gpio_direction_output(pwdnnum, 1);
		gpio_set_value(pwdnnum, 0);
		mdelay(20);
		// msleep(5);
		gpio_set_value(pwdnnum, 1);
		printk("SC031 reset\n");	
		gpio_free(pwdnnum);
	}
	pwdnnum= of_get_named_gpio_flags(client->dev.of_node, "pwdn-gpio", 0, &flags);
	if (gpio_is_valid(pwdnnum)) {
		ret = gpio_request(pwdnnum, "pwdn-gpio");
		gpio_direction_output(pwdnnum, 1);
		gpio_set_value(pwdnnum, 1);
		mdelay(20);
		// msleep(5);
		gpio_set_value(pwdnnum, 0);
		printk("SC031 pwdn\n");	
		gpio_free(pwdnnum);
	} 
	// else
	// 	pr_err("pwdn-d031 request fail\n");
	// SC031_detect(&SC031dev);	
	return 0;
}

/*
 * @description     : 
 * @param - client 	: 
 * @return          : 
 */
static int SC031_remove(struct i2c_client *client)
{
	/*  */
	cdev_del(&SC031dev.cdev);
	unregister_chrdev_region(SC031dev.devid, SC031_CNT);

	/* device class */
	device_destroy(SC031dev.class, SC031dev.devid);
	class_destroy(SC031dev.class);
	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev)){
		__SC031_power_off(client);
	}
	
	pm_runtime_set_suspended(&client->dev);
	return 0;
}

/* i2c_device_id */
static const struct i2c_device_id SC031_id[] = {
	{"ov,sc031mipi", 0},  
	{}
};

/* device_id */
static const struct of_device_id SC031_of_match[] = {
	{ .compatible = "sm,sc031mipi" },
	{ /* Sentinel */ }
};

/* i2c_driver */	
static struct i2c_driver SC031_driver = {
	.probe = SC031_probe,
	.remove = SC031_remove,
	.driver = {
			.owner = THIS_MODULE,
		   	.name = "sc031mipi",
		   	.pm = &SC031_pm_ops,
		   	.of_match_table = SC031_of_match, 
		   },
	.id_table = SC031_id,
};
module_i2c_driver(SC031_driver);		   

MODULE_DESCRIPTION("OmniVision SC031 sensor driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("linronghua,Codefair");



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
#define OV5648_PIXEL_RATE		(45 * 1000 * 1000 * 4)
#define OV5648_XVCLK_FREQ		24000000

#define CHIP_ID				0x5648
#define OV5648_CHIP_ID_H	(0x56)
#define OV5648_CHIP_ID_L	(0x48)

#define OV5648_REG_CHIP_ID		0x300a

#define OV5648_REG_CTRL_MODE		0x0100
#define OV5648_MODE_SW_STANDBY		0x0
#define OV5648_MODE_STREAMING		BIT(0)

#define OV5648_REG_EXPOSURE		0x3500
#define	OV5648_EXPOSURE_MIN		4
#define	OV5648_EXPOSURE_STEP		1
#define OV5648_VTS_MAX			0x7fff

#define OV5648_REG_ANALOG_GAIN		0x3509
#define	ANALOG_GAIN_MIN			0x10
#define	ANALOG_GAIN_MAX			0xf8
#define	ANALOG_GAIN_STEP		1
#define	ANALOG_GAIN_DEFAULT		0xf8

#define OV5648_REG_DIGI_GAIN_H		0x350a
#define OV5648_REG_DIGI_GAIN_L		0x350b
#define OV5648_DIGI_GAIN_L_MASK		0x3f
#define OV5648_DIGI_GAIN_H_SHIFT	6
#define OV5648_DIGI_GAIN_MIN		0
#define OV5648_DIGI_GAIN_MAX		(0x4000 - 1)
#define OV5648_DIGI_GAIN_STEP		1
#define OV5648_DIGI_GAIN_DEFAULT	1024

#define OV5648_REG_TEST_PATTERN		0x4503
#define	OV5648_TEST_PATTERN_ENABLE	0x80
#define	OV5648_TEST_PATTERN_DISABLE	0x0

#define OV5648_REG_VTS			0x380e

#define REG_NULL			0xFFFF

#define OV5648_REG_VALUE_08BIT		1
#define OV5648_REG_VALUE_16BIT		2
#define OV5648_REG_VALUE_24BIT		3

#define OV5648_LANES			2
#define OV5648_BITS_PER_SAMPLE		10

#define OV5648_CNT   1
#define DEV_NAME	"ov5648"

#define SEN_MAGIC 'N'
#define IOCTL_SENINIT _IOW (SEN_MAGIC, 1, int)
#define IOCTL_SENREGW _IOW (SEN_MAGIC, 2, int)
#define IOCTL_SENREGR _IOW (SEN_MAGIC, 3, int)
#define IOCTL_SENREGWB _IOW (SEN_MAGIC, 4, int)

static const char * const ov5648_supply_names[] = {
	"avdd",		/* Analog power */
	"dovdd",	/* Digital I/O power */
	"dvdd",		/* Digital core power */
};

#define OV5648_NUM_SUPPLIES ARRAY_SIZE(ov5648_supply_names)

struct regval {
	u16 addr;
	u8 val;
};
struct ioctl_data {		
	unsigned int reg;
	unsigned char value;
	unsigned char bit;
};

struct ov5648_mode {
	u32 width;
	u32 height;
	u32 max_fps;
	u32 hts_def;
	u32 vts_def;
	u32 exp_def;
	const struct regval *reg_list;
};
#if 0
struct ov5648 {
	struct i2c_client	*client;
	struct clk		*xvclk;
	struct gpio_desc	*pwdn_gpio;
	struct regulator_bulk_data supplies[OV5648_NUM_SUPPLIES];

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
	const struct ov5648_mode *cur_mode;
};
#endif
struct ov5648 {
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
static struct ov5648 ov5648dev;

// #define to_ov5648(sd) container_of(sd, struct ov5648, subdev)

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
static const struct regval ov5648_global_regs[] = {
	#if 1			/////////640*480
		{0x0103,0x01},
		{0x3001,0x00},
		{0x3002,0x00},
		{0x3011,0x02},
		{0x3017,0x05},
		{0x3018,0x4c},
		{0x301c,0xd2},
		{0x3022,0x00},
		{0x3034,0x1a},
		{0x3035,0x21},
		{0x3036,0x69},
		{0x3037,0x03},
		{0x3038,0x00},
		{0x3039,0x00},
		{0x303a,0x00},
		{0x303b,0x19},
		{0x303c,0x11},
		{0x303d,0x30},
		{0x3105,0x11},
		{0x3106,0x05},
		{0x3304,0x28},
		{0x3305,0x41},
		{0x3306,0x30},
		{0x3308,0x00},
		{0x3309,0xc8},
		{0x330a,0x01},
		{0x330b,0x90},
		{0x330c,0x02},
		{0x330d,0x58},
		{0x330e,0x03},
		{0x330f,0x20},
		{0x3300,0x00},
		{0x3500,0x00},
		{0x3501,0x1e},
		{0x3502,0xc0},
		{0x3503,0x07},	//auto expo
		//{0x3503,0x06},

		{0x350a,0x00},
		{0x350b,0x40},
		{0x3601,0x33},
		{0x3602,0x00},
		{0x3611,0x0e},
		{0x3612,0x2b},
		{0x3614,0x50},
		{0x3620,0x33},
		{0x3622,0x00},
		{0x3630,0xad},
		{0x3631,0x00},
		{0x3632,0x94},
		{0x3633,0x17},
		{0x3634,0x14},
		{0x3704,0xc0},
		{0x3705,0x2a},
		{0x3708,0x69},
		{0x3709,0x92},
		{0x370b,0x23},
		{0x370c,0xc3},
		{0x370d,0x00},
		{0x370e,0x00},
		{0x371c,0x07},
		{0x3739,0xd2},
		{0x373c,0x00},
		{0x3800,0x00},
		{0x3801,0x00},
		{0x3802,0x00},
		{0x3803,0x02},
		{0x3804,0x0a},
		{0x3805,0x3f},
		{0x3806,0x07},
		{0x3807,0xa1},
		{0x3808,0x02},
		{0x3809,0x80},
		{0x380a,0x01},
		{0x380b,0xe0},
		{0x380c,0x07},
		{0x380d,0x28},
		{0x380e,0x01},
		{0x380f,0xfc},
		{0x3810,0x00},
		{0x3811,0x08},
		{0x3812,0x00},
		{0x3813,0x04},
		{0x3814,0x71},
		{0x3815,0x53},
		{0x3817,0x00},
		{0x3820,0x08},	/////flip
		//{0x3820,0x0a},

		{0x3821,0x07},
		{0x3826,0x03},
		{0x3829,0x00},
		{0x382b,0x0b},
		{0x3830,0x00},
		{0x3836,0x00},
		{0x3837,0x00},
		{0x3838,0x00},
		{0x3839,0x04},
		{0x383a,0x00},
		{0x383b,0x01},
		{0x3b00,0x00},
		{0x3b02,0x08},
		{0x3b03,0x00},
		{0x3b04,0x04},
		{0x3b05,0x00},
		{0x3b06,0x04},
		{0x3b07,0x08},
		{0x3b08,0x00},
		{0x3b09,0x02},
		{0x3b0a,0x04},
		{0x3b0b,0x00},
		{0x3b0c,0x3d},
		{0x3f01,0x0d},
		{0x3f0f,0xf5},
		{0x4000,0x89},
		{0x4001,0x02},
		{0x4002,0x45},
		{0x4004,0x02},
		{0x4005,0x18},
		{0x4006,0x08},
		{0x4007,0x10},
		{0x4008,0x00},
		{0x4300,0xf8},
		{0x4303,0xff},
		{0x4304,0x00},
		{0x4307,0xff},
		{0x4520,0x00},
		{0x4521,0x00},
		{0x4511,0x22},
		{0x4801,0x0f},
		{0x4814,0x2a},
		{0x481f,0x3c},
		{0x4823,0x3c},
		{0x4826,0x00},
		{0x481b,0x3c},
		{0x4827,0x32},
		{0x4837,0x18},
		{0x4b00,0x06},
		{0x4b01,0x0a},
		{0x4b04,0x10},
		{0x5000,0xff},
		{0x5001,0x00},
		{0x5002,0x41},
		{0x5003,0x0a},
		{0x5004,0x00},
		{0x5043,0x00},
		{0x5013,0x00},
		{0x501f,0x03},
		{0x503d,0x00},
		{0x5a00,0x08},
		{0x5b00,0x01},
		{0x5b01,0x40},
		{0x5b02,0x00},
		{0x5b03,0xf0},
		{0x0100,0x01},

		{0x350b,0xF0},

		{REG_NULL, 0x00},


#endif


};

/*
 * Xclk 24Mhz
 * Pclk 45Mhz
 * linelength 740(0x2e4)
 * framelength 2024(0x7e8)
 * grabwindow_width 2592
 * grabwindow_height 1944
 * max_framerate 30fps
 * mipi_datarate per lane 840Mbps
 */
static const struct regval ov5648_2592x1944_regs[] = {
	{0x3501, 0x7e},
	{0x366e, 0x18},
	{0x3800, 0x00},
	{0x3801, 0x00},
	{0x3802, 0x00},
	{0x3803, 0x04},
	{0x3804, 0x0a},
	{0x3805, 0x3f},
	{0x3806, 0x07},
	{0x3807, 0xab},
	{0x3808, 0x0a},
	{0x3809, 0x20},
	{0x380a, 0x07},
	{0x380b, 0x98},
	{0x380c, 0x02},
	{0x380d, 0xe4},
	{0x380e, 0x07},
	{0x380f, 0xe8},
	{0x3811, 0x06},
	{0x3813, 0x08},
	{0x3814, 0x01},
	{0x3816, 0x01},
	{0x3817, 0x01},
	{0x3820, 0x88},
	{0x3821, 0x00},
	{0x4501, 0x00},
	{0x4008, 0x04},
	{0x4009, 0x13},
	{REG_NULL, 0x00},
};

/*
 * Xclk 24Mhz
 * Pclk 45Mhz
 * linelength 672(0x2a0)
 * framelength 2232(0x8b8)
 * grabwindow_width 1920
 * grabwindow_height 1080
 * max_framerate 30fps
 * mipi_datarate per lane 840Mbps
 */
static const struct regval ov5648_1920x1080_regs[] = {
	{0x3501, 0x45},
	{0x366e, 0x18},
	{0x3800, 0x01},
	{0x3801, 0x50},
	{0x3802, 0x01},
	{0x3803, 0xb8},
	{0x3804, 0x08},
	{0x3805, 0xef},
	{0x3806, 0x05},
	{0x3807, 0xf7},
	{0x3808, 0x07},
	{0x3809, 0x80},
	{0x380a, 0x04},
	{0x380b, 0x38},
	{0x380c, 0x02},
	{0x380d, 0xa0},
	{0x380e, 0x08},
	{0x380f, 0xb8},
	{0x3811, 0x06},
	{0x3813, 0x04},
	{0x3814, 0x01},
	{0x3816, 0x01},
	{0x3817, 0x01},
	{0x3820, 0x88},
	{0x3821, 0x00},
	{0x4501, 0x00},
	{0x4008, 0x04},
	{0x4009, 0x13},
	{REG_NULL, 0x00}
};

/*
 * Xclk 24Mhz
 * Pclk 45Mhz
 * linelength 740(0x02e4)
 * framelength 1012(0x03f4)
 * grabwindow_width 1296
 * grabwindow_height 972
 * max_framerate 60fps
 * mipi_datarate per lane 840Mbps
 */
static const struct regval ov5648_1296x972_regs[] = {
	{0x0103, 0x01},
	{0x0100, 0x00},
	{0x0300, 0x04},
	{0x0301, 0x00},
	{0x0302, 0x69},
	{0x0303, 0x00},
	{0x0304, 0x00},
	{0x0305, 0x01},
	{0x0307, 0x00},
	{0x030b, 0x00},
	{0x030c, 0x00},
	{0x030d, 0x1e},
	{0x030e, 0x04},
	{0x030f, 0x03},
	{0x0312, 0x01},
	{0x3000, 0x00},
	{0x3002, 0xa1},
	{0x3008, 0x00},
	{0x3010, 0x00},
	{0x3016, 0x32},
	{0x3022, 0x51},
	{0x3106, 0x15},
	{0x3107, 0x01},
	{0x3108, 0x05},
	{0x3500, 0x00},
	{0x3501, 0x3e},
	{0x3502, 0x00},
	{0x3503, 0x08},
	{0x3504, 0x03},
	{0x3505, 0x8c},
	{0x3507, 0x03},
	{0x3508, 0x00},
	{0x3509, 0x10},
	{0x350c, 0x00},
	{0x350d, 0x80},
	{0x3510, 0x00},
	{0x3511, 0x02},
	{0x3512, 0x00},
	{0x3601, 0x55},
	{0x3602, 0x58},
	{0x3611, 0x58},
	{0x3614, 0x30},
	{0x3615, 0x77},
	{0x3621, 0x08},
	{0x3624, 0x40},
	{0x3633, 0x0c},
	{0x3634, 0x0c},
	{0x3635, 0x0c},
	{0x3636, 0x0c},
	{0x3638, 0x00},
	{0x3639, 0x00},
	{0x363a, 0x00},
	{0x363b, 0x00},
	{0x363c, 0xff},
	{0x363d, 0xfa},
	{0x3650, 0x44},
	{0x3651, 0x44},
	{0x3652, 0x44},
	{0x3653, 0x44},
	{0x3654, 0x44},
	{0x3655, 0x44},
	{0x3656, 0x44},
	{0x3657, 0x44},
	{0x3660, 0x00},
	{0x3661, 0x00},
	{0x3662, 0x00},
	{0x366a, 0x00},
	{0x366e, 0x0c},
	{0x3673, 0x04},
	{0x3700, 0x14},
	{0x3703, 0x0c},
	{0x3706, 0x24},
	{0x3714, 0x27},
	{0x3715, 0x01},
	{0x3716, 0x00},
	{0x3717, 0x02},
	{0x3733, 0x10},
	{0x3734, 0x40},
	{0x373f, 0xa0},
	{0x3765, 0x20},
	{0x37a1, 0x1d},
	{0x37a8, 0x26},
	{0x37ab, 0x14},
	{0x37c2, 0x04},
	{0x37c3, 0xf0},
	{0x37cb, 0x09},
	{0x37cc, 0x13},
	{0x37cd, 0x1f},
	{0x37ce, 0x1f},
	{0x3800, 0x00},
	{0x3801, 0x00},
	{0x3802, 0x00},
	{0x3803, 0x00},
	{0x3804, 0x0a},
	{0x3805, 0x3f},
	{0x3806, 0x07},
	{0x3807, 0xaf},
	{0x3808, 0x05},
	{0x3809, 0x10},
	{0x380a, 0x03},
	{0x380b, 0xcc},
	{0x380c, 0x02},
	{0x380d, 0xe4},
	{0x380e, 0x03},
	{0x380f, 0xf4},
	{0x3810, 0x00},
	{0x3811, 0x00},
	{0x3812, 0x00},
	{0x3813, 0x06},
	{0x3814, 0x03},
	{0x3815, 0x01},
	{0x3816, 0x03},
	{0x3817, 0x01},
	{0x3818, 0x00},
	{0x3819, 0x00},
	{0x381a, 0x00},
	{0x381b, 0x01},
	{0x3820, 0x8b},
	{0x3821, 0x01},
	{0x3c80, 0x08},
	{0x3c82, 0x00},
	{0x3c83, 0x00},
	{0x3c88, 0x00},
	{0x3d85, 0x14},
	{0x3f02, 0x08},
	{0x3f03, 0x10},
	{0x4008, 0x02},
	{0x4009, 0x09},
	{0x404e, 0x20},
	{0x4501, 0x00},
	{0x4502, 0x10},
	{0x4800, 0x00},
	{0x481f, 0x2a},
	{0x4837, 0x13},
	{0x5000, 0x13},
	{0x5780, 0x3e},
	{0x5781, 0x0f},
	{0x5782, 0x44},
	{0x5783, 0x02},
	{0x5784, 0x01},
	{0x5785, 0x01},
	{0x5786, 0x00},
	{0x5787, 0x04},
	{0x5788, 0x02},
	{0x5789, 0x0f},
	{0x578a, 0xfd},
	{0x578b, 0xf5},
	{0x578c, 0xf5},
	{0x578d, 0x03},
	{0x578e, 0x08},
	{0x578f, 0x0c},
	{0x5790, 0x08},
	{0x5791, 0x06},
	{0x5792, 0x00},
	{0x5793, 0x52},
	{0x5794, 0xa3},
	{0x5b00, 0x00},
	{0x5b01, 0x1c},
	{0x5b02, 0x00},
	{0x5b03, 0x7f},
	{0x5b05, 0x6c},
	{0x5e10, 0xfc},
	{0x4010, 0xf1},
	{0x3503, 0x08},
	{0x3505, 0x8c},
	{0x3507, 0x03},
	{0x3508, 0x00},
	{0x3509, 0xf8},
	{0x0100, 0x01},
	{REG_NULL, 0x00}
};

/*
 * Xclk 24Mhz
 * Pclk 45Mhz
 * linelength 672(0x2a0)
 * framelength 2232(0x8b8)
 * grabwindow_width 1280
 * grabwindow_height 720
 * max_framerate 30fps
 * mipi_datarate per lane 840Mbps
 */
static const struct regval ov5648_1280x720_regs[] = {
	{0x3501, 0x45},
	{0x366e, 0x0c},
	{0x3800, 0x00},
	{0x3801, 0x00},
	{0x3802, 0x01},
	{0x3803, 0x00},
	{0x3804, 0x0a},
	{0x3805, 0x3f},
	{0x3806, 0x06},
	{0x3807, 0xaf},
	{0x3808, 0x05},
	{0x3809, 0x00},
	{0x380a, 0x02},
	{0x380b, 0xd0},
	{0x380c, 0x02},
	{0x380d, 0xa0},
	{0x380e, 0x08},
	{0x380f, 0xb8},
	{0x3811, 0x06},
	{0x3813, 0x02},
	{0x3814, 0x03},
	{0x3816, 0x03},
	{0x3817, 0x01},
	{0x3820, 0x8b},
	{0x3821, 0x01},
	{0x4501, 0x00},
	{0x4008, 0x02},
	{0x4009, 0x09},
	{REG_NULL, 0x00}
};

/*
 * Xclk 24Mhz
 * Pclk 45Mhz
 * linelength 672(0x2a0)
 * framelength 558(0x22e)
 * grabwindow_width 640
 * grabwindow_height 480
 * max_framerate 120fps
 * mipi_datarate per lane 840Mbps
 */
static const struct regval ov5648_640x480_regs[] = {
	{0x0103,0x01},
	{0x3501, 0x22},
	{0x366e, 0x0c},
	{0x3800, 0x00},
	{0x3801, 0x00},
	{0x3802, 0x00},
	{0x3803, 0x08},
	{0x3804, 0x0a},
	{0x3805, 0x3f},
	{0x3806, 0x07},
	{0x3807, 0xa7},
	{0x3808, 0x02},
	{0x3809, 0x80},
	{0x380a, 0x01},
	{0x380b, 0xe0},
	{0x380c, 0x02},
	{0x380d, 0xa0},
	{0x380e, 0x02},
	{0x380f, 0x2e},
	{0x3811, 0x06},
	{0x3813, 0x04},
	{0x3814, 0x07},
	{0x3816, 0x05},
	{0x3817, 0x03},
	{0x3820, 0x8d},
	{0x3821, 0x01},
	{0x4501, 0x00},
	{0x4008, 0x02},
	{0x4009, 0x09},
	{REG_NULL, 0x00}
};

static const struct ov5648_mode supported_modes[] = {
	{
		.width = 2592,
		.height = 1944,
		.max_fps = 30,
		.exp_def = 0x0450,
		.hts_def = 0x02e4 * 4,
		.vts_def = 0x07e8,
		.reg_list = ov5648_2592x1944_regs,
	},
	{
		.width = 1920,
		.height = 1080,
		.max_fps = 30,
		.exp_def = 0x0450,
		.hts_def = 0x02a0 * 4,
		.vts_def = 0x08b8,
		.reg_list = ov5648_1920x1080_regs,
	},
	{
		.width = 1296,
		.height = 972,
		.max_fps = 60,
		.exp_def = 0x03e0,
		.hts_def = 0x02e4 * 4,
		.vts_def = 0x03f4,
		.reg_list = ov5648_1296x972_regs,
	},
	{
		.width = 1280,
		.height = 720,
		.max_fps = 30,
		.exp_def = 0x0450,
		.hts_def = 0x02a0 * 4,
		.vts_def = 0x08b8,
		.reg_list = ov5648_1280x720_regs,
	},
	{
		.width = 640,
		.height = 480,
		.max_fps = 120,
		.exp_def = 0x0450,
		.hts_def = 0x02a0 * 4,
		.vts_def = 0x022e,
		.reg_list = ov5648_640x480_regs,
	},
};

#define OV5648_LINK_FREQ_420MHZ		420000000
static const s64 link_freq_menu_items[] = {
	OV5648_LINK_FREQ_420MHZ
};

static const char * const ov5648_test_pattern_menu[] = {
	"Disabled",
	"Vertical Color Bar Type 1",
	"Vertical Color Bar Type 2",
	"Vertical Color Bar Type 3",
	"Vertical Color Bar Type 4"
};



static int ov5648_write_reg(struct ov5648 *dev, unsigned short reg,unsigned char value)
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

static int ov5648_write_array(struct ov5648 *dev,const struct regval *regs)
{
	u32 i;
	int ret = 0;
     // struct i2c_client *client = (struct i2c_client *)dev->private_data;
	for (i = 0; ret == 0 && regs[i].addr != REG_NULL; i++)
	{
		printk("ov5648 : %x = %x\n",regs[i].addr,regs[i].val );
        ret = ov5648_write_reg(dev, regs[i].addr, regs[i].val);
	}
	return ret;
}



static int ov5648_read_reg(struct ov5648 *dev,  unsigned short reg,
		unsigned char *value)
{
	// struct i2c_client *client = v4l2_get_subdevdata(sd);
	// struct i2c_client *client = (struct i2c_client *)ov5648dev->private_data;
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
static void __ov5648_power_off(struct i2c_client *client)
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
		msleep(20);
		gpio_set_value(pwdnnum, 0);
		gpio_free(pwdnnum);
	} 
	else{
    printk("gpio PWDN fail\n");
	}
	return ;
}
static int __ov5648_power_on(struct i2c_client *client)
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
		msleep(20);
		gpio_set_value(pwdnnum, 1);
		gpio_free(pwdnnum);
	} 
	else{
    printk("gpio PWDN fail\n");
	}
	return 0;
}
static int __ov5648_start_stream(struct ov5648 *dev)
// static int __ov5648_start_stream(struct i2c_client *client)
{
	int ret;
	struct i2c_client *client = (struct i2c_client *)dev->private_data;
   	// struct i2c_client *client = to_i2c_client(dev);
	__ov5648_power_on(client);
	ret = ov5648_write_array(dev,ov5648_global_regs);//yuan
	// ret = ov5648_write_array(dev,ov5648_640x480_regs);
	if(ret)
		printk("ov5648 init error\n");	
	return ret;
}

static int __ov5648_stop_stream(struct ov5648 *dev)
// static int __ov5648_stop_stream(struct device *dev)
{
	int ret;
	struct i2c_client *client = (struct i2c_client *)dev->private_data;
	__ov5648_power_off(client);
	return ret;
}


static inline u32 ov5648_cal_delay(u32 cycles)
{
	return DIV_ROUND_UP(cycles, OV5648_XVCLK_FREQ / 1000 / 1000);
}


static int ov5648_detect(struct ov5648 *dev)
{
	unsigned char v;
	int ret;
	int id=0;

	ret = ov5648_read_reg(dev, 0x300a, &v);
	if (ret < 0)
		return ret;
	id|=v;
	if (v != OV5648_CHIP_ID_H)
		return -ENODEV;
	ret = ov5648_read_reg(dev, 0x300b, &v);
	if (ret < 0)
		return ret;
	id=(id<<8)|v;
	if (v != OV5648_CHIP_ID_L)
		return -ENODEV;
	dev->chipid=id;
	printk("discover ov%x\n", id);
	return 0;
}

/*
 * @description		: 
 * @param - inode 	: 
 * @param - filp 	: 
 * 					  
 * @return 			:
 */
static int OV5648_open(struct inode *inode, struct file *filp)
{
	filp->private_data = &ov5648dev;
	__ov5648_start_stream(&ov5648dev);
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
static ssize_t OV5648_read(struct file *filp, char __user *buf, size_t cnt, loff_t *off)
{
	int data[1]={0};
	long err = 0;
	// int chipidnum;

	struct ov5648 *dev = (struct ov5648 *)filp->private_data;
	
	// OV5648_readdata(dev);
	ov5648_detect(dev);

	data[0] = dev->chipid;
	// data[1] = dev->als;
	// data[2] = dev->ps;
	err = copy_to_user(buf, data, sizeof(data[1]));
	printk("Readid OV%x sensor\n", data[0]);
	return 0;
}
// ssize_t (*write) (struct file *, const char __user *, size_t, loff_t *);
static ssize_t OV5648_write(struct file *filp, const char __user *buf, size_t cnt, loff_t *off)
{
	struct regval data[]={0};
	long err = 0;
	int ret;
    static const struct regval ov5648_data_regs[100]={0};
	struct ov5648 *dev = (struct ov5648 *)filp->private_data;
	

	err = copy_from_user(data, buf, sizeof(buf));
	if(err < 0) {
	printk("kernel write failed!\r\n");
	return -EFAULT;
    }
    // memcpy(&ov5648_data_regs,(struct regval *)data,sizeof(data));
    memcpy(&ov5648_data_regs,&data,sizeof(data[100]));
    ret = ov5648_write_array(dev,ov5648_data_regs);
	if(ret)
		printk("ov5648 write error\n");	
	return 0;
}


static int OV5648_release(struct inode *inode, struct file *filp)
{
	filp->private_data = &ov5648dev;
	__ov5648_stop_stream(&ov5648dev);
	return 0;
}

static long senioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int ret,err;
	struct ioctl_data sendata;
	unsigned char kvalue=0;
    struct ov5648 *dev = (struct ov5648 *)filp->private_data;
    
    switch(cmd) {
		case IOCTL_SENINIT:
           break;
			
        case IOCTL_SENREGW:
            err = copy_from_user(&sendata, (struct ioctl_data *)arg, sizeof(struct ioctl_data));
            	if(err < 0) {
	        printk("senser write failed!\r\n");
	        return -EFAULT;
            }
            ov5648_write_reg(dev, sendata.reg, sendata.value);
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
            ret = ov5648_read_reg(dev, sendata.reg, &kvalue);
            if (ret < 0)
            {
            	return ret;
            }
		    sendata.value=kvalue;
			if(copy_to_user( (struct ioctl_data *)arg, &sendata, sizeof(struct ioctl_data))){
				ret = -1;
			 	printk("sendata to user error\n");
			}
			// ret = ov5648_read_reg(dev, sendata.reg, sendata.value);	
			printk("sen read out reg  %x=%x\n",sendata.reg,sendata.value);
			break;
        default:
            printk("SENSER in the default\n");
            return -EINVAL;
    }
    return 0;
}

static int __maybe_unused ov5648_runtime_resume(struct device *dev)
{  
	
	struct i2c_client *client = to_i2c_client(dev);
	printk("pmon-gpio\n");
	__ov5648_power_on(client);
	return 0;
}

static int __maybe_unused ov5648_runtime_suspend(struct device *dev)
{
   
   struct i2c_client *client = to_i2c_client(dev);
   printk("pmoff-gpio\n");
	__ov5648_power_off(client);

	return 0;
}
static const struct dev_pm_ops ov5648_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(ov5648_runtime_suspend,ov5648_runtime_resume)
	SET_RUNTIME_PM_OPS(ov5648_runtime_suspend,ov5648_runtime_resume, NULL)
};

/* OV5648 */
static const struct file_operations OV5648_ops = {
	.owner = THIS_MODULE,
	.open = OV5648_open,
	.read = OV5648_read,
	.write = OV5648_write,
	.unlocked_ioctl  = senioctl,
	.release = OV5648_release,
};

 /*
  * @description     : 
  *                    
  * @param - client  : 
  * @param - id      : 
  * @return          : 
  */
static int OV5648_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	// enum of_gpio_flags  flags;
	// int pwdnnum = 0;
	// int ret=0;
	// struct device *dev = &client->dev;
	/* major */
	if (ov5648dev.major) {
		ov5648dev.devid = MKDEV(ov5648dev.major, 0);
		register_chrdev_region(ov5648dev.devid, OV5648_CNT, DEV_NAME);
	} else {
		alloc_chrdev_region(&ov5648dev.devid, 0, OV5648_CNT, DEV_NAME);
		ov5648dev.major = MAJOR(ov5648dev.devid);
	}

	/* cdev */
	cdev_init(&ov5648dev.cdev, &OV5648_ops);
	cdev_add(&ov5648dev.cdev, ov5648dev.devid, OV5648_CNT);

	/* class */
	ov5648dev.class = class_create(THIS_MODULE, DEV_NAME);
	if (IS_ERR(ov5648dev.class)) {
		return PTR_ERR(ov5648dev.class);
	}

	/* device */
	ov5648dev.device = device_create(ov5648dev.class, NULL, ov5648dev.devid, NULL, DEV_NAME);
	if (IS_ERR(ov5648dev.device)) {
		return PTR_ERR(ov5648dev.device);
	}
 
	ov5648dev.private_data = client;
	// ov5648_detect(&ov5648dev);	
	return 0;
}

/*
 * @description     : 
 * @param - client 	: 
 * @return          : 
 */
static int OV5648_remove(struct i2c_client *client)
{
	/*  */
	cdev_del(&ov5648dev.cdev);
	unregister_chrdev_region(ov5648dev.devid, OV5648_CNT);

	/* device class */
	device_destroy(ov5648dev.class, ov5648dev.devid);
	class_destroy(ov5648dev.class);
	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev)){
		__ov5648_power_off(client);
	}
	
	pm_runtime_set_suspended(&client->dev);
	return 0;
}

/* i2c_device_id */
static const struct i2c_device_id OV5648_id[] = {
	{"ov,ov5648", 0},  
	{}
};

/* device_id */
static const struct of_device_id OV5648_of_match[] = {
	{ .compatible = "ov,ov5648" },
	{ /* Sentinel */ }
};

/* i2c_driver */	
static struct i2c_driver OV5648_driver = {
	.probe = OV5648_probe,
	.remove = OV5648_remove,
	.driver = {
			.owner = THIS_MODULE,
		   	.name = "ov5648",
		   	.pm = &ov5648_pm_ops,
		   	.of_match_table = OV5648_of_match, 
		   },
	.id_table = OV5648_id,
};
module_i2c_driver(OV5648_driver);		   

MODULE_DESCRIPTION("OmniVision ov5648 sensor driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("linronghua,Codefair");



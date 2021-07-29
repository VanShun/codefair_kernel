// SPDX-License-Identifier: GPL-2.0
/*
 * gc0308 driver
 *
 * Copyright (C) 2017 Fuzhou Rockchip Electronics Co., Ltd.
 */

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/sysfs.h>
#include <media/media-entity.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-subdev.h>

#ifndef V4L2_CID_DIGITAL_GAIN
#define V4L2_CID_DIGITAL_GAIN		V4L2_CID_GAIN
#endif

/* 45Mhz * 4 Binning */
#define GC0308_PIXEL_RATE		(45 * 1000 * 1000 * 4)
#define GC0308_XVCLK_FREQ		24000000

//#define CHIP_ID				0x005695
#define CHIP_ID				0x56486c		//////added by xieyh 

#define GC0308_REG_CHIP_ID		0x300a

#define GC0308_REG_CTRL_MODE		0x0100
#define GC0308_MODE_SW_STANDBY		0x0
#define GC0308_MODE_STREAMING		BIT(0)

#define GC0308_REG_EXPOSURE		0x3500
#define	GC0308_EXPOSURE_MIN		4
#define	GC0308_EXPOSURE_STEP		1
#define GC0308_VTS_MAX			0x7fff

#define GC0308_REG_ANALOG_GAIN		0x3509
#define	ANALOG_GAIN_MIN			0x10
#define	ANALOG_GAIN_MAX			0xf8
#define	ANALOG_GAIN_STEP		1
#define	ANALOG_GAIN_DEFAULT		0xf8

#define GC0308_REG_DIGI_GAIN_H		0x350a
#define GC0308_REG_DIGI_GAIN_L		0x350b
#define GC0308_DIGI_GAIN_L_MASK		0x3f
#define GC0308_DIGI_GAIN_H_SHIFT	6
#define GC0308_DIGI_GAIN_MIN		0
#define GC0308_DIGI_GAIN_MAX		(0x4000 - 1)
#define GC0308_DIGI_GAIN_STEP		1
#define GC0308_DIGI_GAIN_DEFAULT	1024

#define GC0308_REG_TEST_PATTERN		0x4503
#define	GC0308_TEST_PATTERN_ENABLE	0x80
#define	GC0308_TEST_PATTERN_DISABLE	0x0

#define GC0308_REG_VTS			0x380e

#define REG_NULL			0xFF

#define GC0308_REG_VALUE_08BIT		1
#define GC0308_REG_VALUE_16BIT		2
#define GC0308_REG_VALUE_24BIT		3

#define GC0308_LANES			2
#define GC0308_BITS_PER_SAMPLE		10

static const char * const gc0308_supply_names[] = {
	"avdd",		/* Analog power */
	"dovdd",	/* Digital I/O power */
	"dvdd",		/* Digital core power */
};

#define GC0308_NUM_SUPPLIES ARRAY_SIZE(gc0308_supply_names)

struct regval {
	u8 addr;
	u8 val;
};

struct gc0308_mode {
	u32 width;
	u32 height;
	u32 max_fps;
	u32 hts_def;
	u32 vts_def;
	u32 exp_def;
	const struct regval *reg_list;
};

struct gc0308 {
	struct i2c_client	*client;
	struct clk		*xvclk;
	struct gpio_desc	*reset_gpio;
	struct regulator_bulk_data supplies[GC0308_NUM_SUPPLIES];

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
	const struct gc0308_mode *cur_mode;
};

#define to_gc0308(sd) container_of(sd, struct gc0308, subdev)

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
static const struct regval gc0308_global_regs[] = {
#if 1			/////////640*480
			{0xfe , 0x80},

			{0xfe , 0x00},   // set page0

			{0xd2 , 0x10},   // close AEC
			{0x22 , 0x55},   // close AWB

			{0x03 , 0x01},
			{0x04 , 0x2c},
			{0x5a , 0x56},
			{0x5b , 0x40},
			{0x5c , 0x4a},

			{0x22 , 0x57},   // Open AWB

			{0x01 , 0xfa},
			{0x02 , 0x70},
			{0x0f , 0x01},


			{0xe2 , 0x00},   //anti-flicker step [11:8]
			{0xe3 , 0x64},   //anti-flicker step [7:0]

			{0xe4 , 0x02},   //exp level 1  16.67fps
			{0xe5 , 0x58},
			{0xe6 , 0x03},   //exp level 2  12.5fps
			{0xe7 , 0x20},
			{0xe8 , 0x04},   //exp level 3  8.33fps
			{0xe9 , 0xb0},
			{0xea , 0x09},   //exp level 4  4.00fps
			{0xeb , 0xc4},

			//{0xec , 0x20},

			{0x05 , 0x00},
			{0x06 , 0x00},
			{0x07 , 0x00},
			{0x08 , 0x00},
			{0x09 , 0x01},
			{0x0a , 0xe8},
			{0x0b , 0x02},
			{0x0c , 0x88},
			{0x0d , 0x02},
			{0x0e , 0x02},
			{0x10 , 0x22},
			{0x11 , 0xfd},
			{0x12 , 0x2a},
			{0x13 , 0x00},
			//{0x14 , 0x10},
			{0x15 , 0x0a},
			{0x16 , 0x05},
			{0x17 , 0x01},
			{0x18 , 0x44},
			{0x19 , 0x44},
			{0x1a , 0x1e},
			{0x1b , 0x00},
			{0x1c , 0xc1},
			{0x1d , 0x08},
			{0x1e , 0x60},
			{0x1f , 0x16},


			{0x20 , 0x00},
			{0x21 , 0xf8},
			{0x22 , 0x57},
			{0x24 , 0xb7},    //mode
			{0x25 , 0x0f},

			//output sync_mode
			{0x26 , 0x03},
			{0x2f , 0x01},
			{0x30 , 0xf7},
			{0x31 , 0x50},
			{0x32 , 0x00},
			{0x39 , 0x04},
			{0x3a , 0x18},
			{0x3b , 0x20},
			{0x3c , 0x00},
			{0x3d , 0x00},
			{0x3e , 0x00},
			{0x3f , 0x00},
			{0x50 , 0x10},
			{0x53 , 0xa2},    //82
			{0x54 , 0x80},
			{0x55 , 0x80},
			{0x56 , 0x82},
			{0x8b , 0x40},
			{0x8c , 0x40},
			{0x8d , 0x40},
			{0x8e , 0x2e},
			{0x8f , 0x2e},
			{0x90 , 0x2e},
			{0x91 , 0x3c},
			{0x92 , 0x50},
			{0x5d , 0x12},
			{0x5e , 0x1a},
			{0x5f , 0x24},
			{0x60 , 0x07},
			{0x61 , 0x15},
			{0x62 , 0x08},
			{0x64 , 0x03},
			{0x66 , 0xe8},
			{0x67 , 0x86},
			{0x68 , 0xa2},
			{0x69 , 0x18},
			{0x6a , 0x0f},
			{0x6b , 0x00},
			{0x6c , 0x5f},
			{0x6d , 0x8f},
			{0x6e , 0x55},
			{0x6f , 0x38},
			{0x70 , 0x15},
			{0x71 , 0x33},
			{0x72 , 0xdc},
			{0x73 , 0x80},
			{0x74 , 0x02},
			{0x75 , 0x3f},
			{0x76 , 0x02},
			{0x77 , 0x36},
			{0x78 , 0x88},
			{0x79 , 0x81},
			{0x7a , 0x81},
			{0x7b , 0x22},
			{0x7c , 0xff},
			{0x93 , 0x48},
			{0x94 , 0x00},
			{0x95 , 0x05},
			{0x96 , 0xe8},
			{0x97 , 0x40},
			{0x98 , 0xf0},
			{0xb1 , 0x38},
			{0xb2 , 0x38},
			{0xbd , 0x38},
			{0xbe , 0x36},
			{0xd0 , 0xc9},
			{0xd1 , 0x10},
			//{0xd2 , 0x90},
			{0xd3 , 0x80},
			{0xd5 , 0xf2},
			{0xd6 , 0x16},
			{0xdb , 0x92},
			{0xdc , 0xa5},
			{0xdf , 0x23},
			{0xd9 , 0x00},
			{0xda , 0x00},
			{0xe0 , 0x09},

			{0xed , 0x04},
			{0xee , 0xa0},
			{0xef , 0x40},
			{0x80 , 0x03},
			{0x80 , 0x03},
			{0x9F , 0x10},
			{0xA0 , 0x20},
			{0xA1 , 0x38},
			{0xA2 , 0x4E},
			{0xA3 , 0x63},
			{0xA4 , 0x76},
			{0xA5 , 0x87},
			{0xA6 , 0xA2},
			{0xA7 , 0xB8},
			{0xA8 , 0xCA},
			{0xA9 , 0xD8},
			{0xAA , 0xE3},
			{0xAB , 0xEB},
			{0xAC , 0xF0},
			{0xAD , 0xF8},
			{0xAE , 0xFD},
			{0xAF , 0xFF},
			{0xc0 , 0x00},
			{0xc1 , 0x10},
			{0xc2 , 0x1C},
			{0xc3 , 0x30},
			{0xc4 , 0x43},
			{0xc5 , 0x54},
			{0xc6 , 0x65},
			{0xc7 , 0x75},
			{0xc8 , 0x93},
			{0xc9 , 0xB0},
			{0xca , 0xCB},
			{0xcb , 0xE6},
			{0xcc , 0xFF},
			{0xf0 , 0x02},
			{0xf1 , 0x01},
			{0xf2 , 0x01},
			{0xf3 , 0x30},
			{0xf9 , 0x9f},
			{0xfa , 0x78},

			//---------------------------------------------------------------
			{0xfe , 0x01},// set page1

			{0x00 , 0xf5},
			{0x02 , 0x1a},
			{0x0a , 0xa0},
			{0x0b , 0x60},
			{0x0c , 0x08},
			{0x0e , 0x4c},
			{0x0f , 0x39},
			{0x11 , 0x3f},
			{0x12 , 0x72},
			{0x13 , 0x13},
			{0x14 , 0x42},
			{0x15 , 0x43},
			{0x16 , 0xc2},
			{0x17 , 0xa8},
			{0x18 , 0x18},
			{0x19 , 0x40},
			{0x1a , 0xd0},
			{0x1b , 0xf5},
			{0x70 , 0x40},
			{0x71 , 0x58},
			{0x72 , 0x30},
			{0x73 , 0x48},
			{0x74 , 0x20},
			{0x75 , 0x60},
			{0x77 , 0x20},
			{0x78 , 0x32},
			{0x30 , 0x03},
			{0x31 , 0x40},
			{0x32 , 0xe0},
			{0x33 , 0xe0},
			{0x34 , 0xe0},
			{0x35 , 0xb0},
			{0x36 , 0xc0},
			{0x37 , 0xc0},
			{0x38 , 0x04},
			{0x39 , 0x09},
			{0x3a , 0x12},
			{0x3b , 0x1C},
			{0x3c , 0x28},
			{0x3d , 0x31},
			{0x3e , 0x44},
			{0x3f , 0x57},
			{0x40 , 0x6C},
			{0x41 , 0x81},
			{0x42 , 0x94},
			{0x43 , 0xA7},
			{0x44 , 0xB8},
			{0x45 , 0xD6},
			{0x46 , 0xEE},
			{0x47 , 0x0d},

			{0xfe , 0x00}, // set page0
			{0xd2 , 0x90},


			//-----------Update the registers 2010/07/06-------------//
			//Registers of Page0
			{0xfe , 0x00}, // set page0
			{0x10 , 0x26},
			{0x11 , 0x0d},  // fd,modified by mormo 2010/07/06
			{0x1a , 0x2a},  // 1e,modified by mormo 2010/07/06

			{0x1c , 0x49}, // c1,modified by mormo 2010/07/06
			{0x1d , 0x9a}, // 08,modified by mormo 2010/07/06
			{0x1e , 0x61}, // 60,modified by mormo 2010/07/06

			{0x3a , 0x20},

			{0x50 , 0x14},  // 10,modified by mormo 2010/07/06
			{0x53 , 0xa0},
			{0x56 , 0x80},

			{0x8b , 0x20}, //LSC
			{0x8c , 0x20},
			{0x8d , 0x20},
			{0x8e , 0x14},
			{0x8f , 0x10},
			{0x90 , 0x14},

			{0x94 , 0x02},
			{0x95 , 0x07},
			{0x96 , 0xe0},

			{0xb1 , 0x40}, // YCPT
			{0xb2 , 0x40},
			{0xb3 , 0x40},
			{0xb6 , 0xe0},

			{0xd0 , 0xcb}, // AECT  c9,modifed by mormo 2010/07/06
			{0xd3 , 0x48}, // 80,modified by mormor 2010/07/06

			{0xf2 , 0x02},
			{0xf7 , 0x12},
			{0xf8 , 0x0a},

			//Registers of Page1
			{0xfe , 0x01},// set page1
			{0x02 , 0x20},
			{0x04 , 0x10},
			{0x05 , 0x08},
			{0x06 , 0x20},
			{0x08 , 0x0a},

			{0x0e , 0x44},
			{0x0f , 0x32},
			{0x10 , 0x41},
			{0x11 , 0x37},
			{0x12 , 0x22},
			{0x13 , 0x19},
			{0x14 , 0x44},
			{0x15 , 0x44},

			{0x19 , 0x50},
			{0x1a , 0xd8},

			{0x32 , 0x10},

			{0x35 , 0x00},
			{0x36 , 0x80},
			{0x37 , 0x00},
			//-----------Update the registers end---------//
			{0xfe , 0x00},// set page0

			//-----------GAMMA Select(2)---------------//
					{0x9F , 0x0E},
					{0xA0 , 0x1C},
					{0xA1 , 0x34},
					{0xA2 , 0x48},
					{0xA3 , 0x5A},
					{0xA4 , 0x6B},
					{0xA5 , 0x7B},
					{0xA6 , 0x95},
					{0xA7 , 0xAB},
					{0xA8 , 0xBF},
					{0xA9 , 0xCE},
					{0xAA , 0xD9},
					{0xAB , 0xE4},
					{0xAC , 0xEC},
					{0xAD , 0xF7},
					{0xAE , 0xFD},
					{0xAF , 0xFF},
	
#endif

			{REG_NULL, 0x00},
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
static const struct regval gc0308_2592x1944_regs[] = {
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
static const struct regval gc0308_1920x1080_regs[] = {
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
static const struct regval gc0308_1296x972_regs[] = {
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
static const struct regval gc0308_1280x720_regs[] = {
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
static const struct regval gc0308_640x480_regs[] = {
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

static const struct gc0308_mode supported_modes[] = {
	{
		.width = 2592,
		.height = 1944,
		.max_fps = 30,
		.exp_def = 0x0450,
		.hts_def = 0x02e4 * 4,
		.vts_def = 0x07e8,
		.reg_list = gc0308_2592x1944_regs,
	},
	{
		.width = 1920,
		.height = 1080,
		.max_fps = 30,
		.exp_def = 0x0450,
		.hts_def = 0x02a0 * 4,
		.vts_def = 0x08b8,
		.reg_list = gc0308_1920x1080_regs,
	},
	{
		.width = 1296,
		.height = 972,
		.max_fps = 60,
		.exp_def = 0x03e0,
		.hts_def = 0x02e4 * 4,
		.vts_def = 0x03f4,
		.reg_list = gc0308_1296x972_regs,
	},
	{
		.width = 1280,
		.height = 720,
		.max_fps = 30,
		.exp_def = 0x0450,
		.hts_def = 0x02a0 * 4,
		.vts_def = 0x08b8,
		.reg_list = gc0308_1280x720_regs,
	},
	{
		.width = 640,
		.height = 480,
		.max_fps = 120,
		.exp_def = 0x0450,
		.hts_def = 0x02a0 * 4,
		.vts_def = 0x022e,
		.reg_list = gc0308_640x480_regs,
	},
};

#define GC0308_LINK_FREQ_420MHZ		420000000
static const s64 link_freq_menu_items[] = {
	GC0308_LINK_FREQ_420MHZ
};

static const char * const gc0308_test_pattern_menu[] = {
	"Disabled",
	"Vertical Color Bar Type 1",
	"Vertical Color Bar Type 2",
	"Vertical Color Bar Type 3",
	"Vertical Color Bar Type 4"
};

/* Write registers up to 4 at a time */
static int gc0308_write_reg(struct i2c_client *client, u16 reg,
			    u32 len, u32 val)
{
	u32 buf_i, val_i;
	u8 buf[6];
	u8 *val_p;
	__be32 val_be;

	if (len > 4)
		return -EINVAL;

	buf[0] = reg >> 8;
	buf[1] = reg & 0xff;

	val_be = cpu_to_be32(val);
	val_p = (u8 *)&val_be;
	buf_i = 2;
	val_i = 4 - len;

	while (val_i < 4)
		buf[buf_i++] = val_p[val_i++];

	if (i2c_master_send(client, buf, len + 2) != len + 2)
		return -EIO;

	return 0;
}

static int gc0308_write_array(struct i2c_client *client,
			      const struct regval *regs)
{
	u32 i;
	int ret = 0;

	for (i = 0; ret == 0 && regs[i].addr != REG_NULL; i++)
	{
		printk("gc0308 : %x = %x\n",regs[i].addr,regs[i].val );
		ret = gc0308_write_reg(client, regs[i].addr,
				       GC0308_REG_VALUE_08BIT, regs[i].val);
	}
	return ret;
}

/* Read registers up to 4 at a time */
static int gc0308_read_reg(struct i2c_client *client, u16 reg, unsigned int len,
			   u32 *val)
{
	struct i2c_msg msgs[2];
	u8 *data_be_p;
	__be32 data_be = 0;
	__be16 reg_addr_be = cpu_to_be16(reg);
	int ret;

	if (len > 4)
		return -EINVAL;

	data_be_p = (u8 *)&data_be;
	/* Write register address */
	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = 2;
	msgs[0].buf = (u8 *)&reg_addr_be;

	/* Read data from register */
	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = len;
	msgs[1].buf = &data_be_p[4 - len];

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret != ARRAY_SIZE(msgs))
		return -EIO;

	*val = be32_to_cpu(data_be);

	return 0;
}

static int gc0308_get_reso_dist(const struct gc0308_mode *mode,
				struct v4l2_mbus_framefmt *framefmt)
{
	return abs(mode->width - framefmt->width) +
	       abs(mode->height - framefmt->height);
}

static const struct gc0308_mode *
gc0308_find_best_fit(struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *framefmt = &fmt->format;
	int dist;
	int cur_best_fit = 0;
	int cur_best_fit_dist = -1;
	int i;

	for (i = 0; i < ARRAY_SIZE(supported_modes); i++) {
		dist = gc0308_get_reso_dist(&supported_modes[i], framefmt);
		if (cur_best_fit_dist == -1 || dist < cur_best_fit_dist) {
			cur_best_fit_dist = dist;
			cur_best_fit = i;
		}
	}

	return &supported_modes[cur_best_fit];
}

static int gc0308_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct gc0308 *gc0308 = to_gc0308(sd);
	const struct gc0308_mode *mode;
	s64 h_blank, vblank_def;

	mutex_lock(&gc0308->mutex);

	mode = gc0308_find_best_fit(fmt);
	fmt->format.code = MEDIA_BUS_FMT_SBGGR10_1X10;
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		*v4l2_subdev_get_try_format(sd, cfg, fmt->pad) = fmt->format;
#endif
	} else {
		gc0308->cur_mode = mode;
		h_blank = mode->hts_def - mode->width;
		__v4l2_ctrl_modify_range(gc0308->hblank, h_blank,
					 h_blank, 1, h_blank);
		vblank_def = mode->vts_def - mode->height;
		__v4l2_ctrl_modify_range(gc0308->vblank, vblank_def,
					 GC0308_VTS_MAX - mode->height,
					 1, vblank_def);
	}

	mutex_unlock(&gc0308->mutex);

	return 0;
}

static int gc0308_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct gc0308 *gc0308 = to_gc0308(sd);
	const struct gc0308_mode *mode = gc0308->cur_mode;

	mutex_lock(&gc0308->mutex);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		fmt->format = *v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
#else
		mutex_unlock(&gc0308->mutex);
		return -EINVAL;
#endif
	} else {
		fmt->format.width = mode->width;
		fmt->format.height = mode->height;
		fmt->format.code = MEDIA_BUS_FMT_SBGGR10_1X10;
		fmt->format.field = V4L2_FIELD_NONE;
	}
	mutex_unlock(&gc0308->mutex);

	return 0;
}

static int gc0308_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index != 0)
		return -EINVAL;
	code->code = MEDIA_BUS_FMT_SBGGR10_1X10;

	return 0;
}

static int gc0308_enum_frame_sizes(struct v4l2_subdev *sd,
				   struct v4l2_subdev_pad_config *cfg,
				   struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index >= ARRAY_SIZE(supported_modes))
		return -EINVAL;

	if (fse->code != MEDIA_BUS_FMT_SBGGR10_1X10)
		return -EINVAL;

	fse->min_width  = supported_modes[fse->index].width;
	fse->max_width  = supported_modes[fse->index].width;
	fse->max_height = supported_modes[fse->index].height;
	fse->min_height = supported_modes[fse->index].height;

	return 0;
}

static int gc0308_enable_test_pattern(struct gc0308 *gc0308, u32 pattern)
{
	u32 val;

	if (pattern)
		val = (pattern - 1) | GC0308_TEST_PATTERN_ENABLE;
	else
		val = GC0308_TEST_PATTERN_DISABLE;

	return gc0308_write_reg(gc0308->client, GC0308_REG_TEST_PATTERN,
				GC0308_REG_VALUE_08BIT, val);
}


static int gc0308_read_reg2(struct i2c_client *client, u8 reg, u8 *val)
{
	int ret;
	u8 data = reg;
	struct i2c_msg msg = {
		.addr	= client->addr,
		.flags	= 0,
		.len	= 1,
		.buf	= &data,
	};

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0)
		goto err;

	msg.flags = I2C_M_RD;
	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0)
		goto err;

	*val = data;
	return 0;

err:
	dev_err(&client->dev, "Failed reading register 0x%02x!\n", reg);
	return ret;
}


/* write a register */
static int gc0308_write_reg2(struct i2c_client *client, u8 reg, u8 val)
{
	int ret;
	unsigned char data[2] = { reg, val };
	struct i2c_msg msg = {
		.addr	= client->addr,
		.flags	= 0,
		.len	= 2,
		.buf	= data,
	};

	ret = i2c_transfer(client->adapter, &msg, 1);
	udelay(100);

	if (ret < 0) {
		dev_err(&client->dev, "Failed writing register 0x%02x!\n", reg);
		return ret;
	}
	return 0;
}

static int gc0308_write_array2(struct i2c_client *client,
			      const struct regval *regs)
{
	u32 i;
	int ret = 0;

	for (i = 0; ret == 0 && regs[i].addr != REG_NULL; i++)
	{
		printk("gc0308 : %x = %x\n",regs[i].addr,regs[i].val );
		//ret = gc0308_write_reg(client, regs[i].addr,
				       //GC0308_REG_VALUE_08BIT, regs[i].val);
		 ret = gc0308_write_reg2(client, regs[i].addr, regs[i].val);
	}
	return ret;
}


static int __gc0308_start_stream(struct gc0308 *gc0308)
{
	int ret;

	//ret = gc0308_write_array(gc0308->client, gc0308_global_regs);
	ret = gc0308_write_array2(gc0308->client, gc0308_global_regs);
	if(ret)
		printk("gc0308 init error\n");
	#if 0
	if (ret)
		return ret;
	
	ret = gc0308_write_array(gc0308->client, gc0308->cur_mode->reg_list);
	if (ret)
		return ret;

	/* In case these controls are set before streaming */
	ret = __v4l2_ctrl_handler_setup(&gc0308->ctrl_handler);
	if (ret)
		return ret;

	return gc0308_write_reg(gc0308->client, GC0308_REG_CTRL_MODE,
				GC0308_REG_VALUE_08BIT, GC0308_MODE_STREAMING);
	#endif
	
}

static int __gc0308_stop_stream(struct gc0308 *gc0308)
{
	return gc0308_write_reg(gc0308->client, GC0308_REG_CTRL_MODE,
				GC0308_REG_VALUE_08BIT, GC0308_MODE_SW_STANDBY);
}

static int gc0308_s_stream(struct v4l2_subdev *sd, int on)
{
	struct gc0308 *gc0308 = to_gc0308(sd);
	struct i2c_client *client = gc0308->client;
	int ret = 0;

	mutex_lock(&gc0308->mutex);
	on = !!on;
	if (on == gc0308->streaming)
		goto unlock_and_return;

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		ret = __gc0308_start_stream(gc0308);
		if (ret) {
			v4l2_err(sd, "start stream failed while write regs\n");
			pm_runtime_put(&client->dev);
			goto unlock_and_return;
		}
	} else {
		__gc0308_stop_stream(gc0308);
		pm_runtime_put(&client->dev);
	}

	gc0308->streaming = on;

unlock_and_return:
	mutex_unlock(&gc0308->mutex);

	return ret;
}

/* Calculate the delay in us by clock rate and clock cycles */
static inline u32 gc0308_cal_delay(u32 cycles)
{
	return DIV_ROUND_UP(cycles, GC0308_XVCLK_FREQ / 1000 / 1000);
}

static int __gc0308_power_on(struct gc0308 *gc0308)
{
	int ret;
	u32 delay_us;
	struct device *dev = &gc0308->client->dev;

	ret = clk_prepare_enable(gc0308->xvclk);
	if (ret < 0) {
		dev_err(dev, "Failed to enable xvclk\n");
		return ret;
	}

	gpiod_set_value_cansleep(gc0308->reset_gpio, 1);

	ret = regulator_bulk_enable(GC0308_NUM_SUPPLIES, gc0308->supplies);
	if (ret < 0) {
		dev_err(dev, "Failed to enable regulators\n");
		goto disable_clk;
	}

	gpiod_set_value_cansleep(gc0308->reset_gpio, 0);

	/* 8192 cycles prior to first SCCB transaction */
	delay_us = gc0308_cal_delay(8192);
	usleep_range(delay_us, delay_us * 2);

	return 0;

disable_clk:
	clk_disable_unprepare(gc0308->xvclk);

	return ret;
}

static void __gc0308_power_off(struct gc0308 *gc0308)
{
	clk_disable_unprepare(gc0308->xvclk);
	gpiod_set_value_cansleep(gc0308->reset_gpio, 1);
	regulator_bulk_disable(GC0308_NUM_SUPPLIES, gc0308->supplies);
}

static int __maybe_unused gc0308_runtime_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct gc0308 *gc0308 = to_gc0308(sd);

	return __gc0308_power_on(gc0308);
}

static int __maybe_unused gc0308_runtime_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct gc0308 *gc0308 = to_gc0308(sd);

	__gc0308_power_off(gc0308);

	return 0;
}

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static int gc0308_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct gc0308 *gc0308 = to_gc0308(sd);
	struct v4l2_mbus_framefmt *try_fmt =
				v4l2_subdev_get_try_format(sd, fh->pad, 0);
	const struct gc0308_mode *def_mode = &supported_modes[0];

	mutex_lock(&gc0308->mutex);
	/* Initialize try_fmt */
	try_fmt->width = def_mode->width;
	try_fmt->height = def_mode->height;
	try_fmt->code = MEDIA_BUS_FMT_SBGGR10_1X10;
	try_fmt->field = V4L2_FIELD_NONE;

	mutex_unlock(&gc0308->mutex);
	/* No crop or compose */

	return 0;
}
#endif

static const struct dev_pm_ops gc0308_pm_ops = {
	SET_RUNTIME_PM_OPS(gc0308_runtime_suspend,
			   gc0308_runtime_resume, NULL)
};

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static const struct v4l2_subdev_internal_ops gc0308_internal_ops = {
	.open = gc0308_open,
};
#endif

static const struct v4l2_subdev_video_ops gc0308_video_ops = {
	.s_stream = gc0308_s_stream,
};

static const struct v4l2_subdev_pad_ops gc0308_pad_ops = {
	.enum_mbus_code = gc0308_enum_mbus_code,
	.enum_frame_size = gc0308_enum_frame_sizes,
	.get_fmt = gc0308_get_fmt,
	.set_fmt = gc0308_set_fmt,
};

static const struct v4l2_subdev_ops gc0308_subdev_ops = {
	.video	= &gc0308_video_ops,
	.pad	= &gc0308_pad_ops,
};

static int gc0308_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct gc0308 *gc0308 = container_of(ctrl->handler,
					     struct gc0308, ctrl_handler);
	struct i2c_client *client = gc0308->client;
	s64 max;
	int ret = 0;

	/* Propagate change of current control to all related controls */
	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		/* Update max exposure while meeting expected vblanking */
		max = gc0308->cur_mode->height + ctrl->val - 4;
		__v4l2_ctrl_modify_range(gc0308->exposure,
					 gc0308->exposure->minimum, max,
					 gc0308->exposure->step,
					 gc0308->exposure->default_value);
		break;
	}

	if (!pm_runtime_get_if_in_use(&client->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		/* 4 least significant bits of expsoure are fractional part */
		ret = gc0308_write_reg(gc0308->client, GC0308_REG_EXPOSURE,
				       GC0308_REG_VALUE_24BIT, ctrl->val << 4);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		ret = gc0308_write_reg(gc0308->client, GC0308_REG_ANALOG_GAIN,
				       GC0308_REG_VALUE_08BIT, ctrl->val);
		break;
	case V4L2_CID_DIGITAL_GAIN:
		ret = gc0308_write_reg(gc0308->client, GC0308_REG_DIGI_GAIN_L,
				       GC0308_REG_VALUE_08BIT,
				       ctrl->val & GC0308_DIGI_GAIN_L_MASK);
		ret = gc0308_write_reg(gc0308->client, GC0308_REG_DIGI_GAIN_H,
				       GC0308_REG_VALUE_08BIT,
				       ctrl->val >> GC0308_DIGI_GAIN_H_SHIFT);
		break;
	case V4L2_CID_VBLANK:
		ret = gc0308_write_reg(gc0308->client, GC0308_REG_VTS,
				       GC0308_REG_VALUE_16BIT,
				       ctrl->val + gc0308->cur_mode->height);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = gc0308_enable_test_pattern(gc0308, ctrl->val);
		break;
	default:
		dev_warn(&client->dev, "%s Unhandled id:0x%x, val:0x%x\n",
			 __func__, ctrl->id, ctrl->val);
		break;
	}

	pm_runtime_put(&client->dev);

	return ret;
}

static const struct v4l2_ctrl_ops gc0308_ctrl_ops = {
	.s_ctrl = gc0308_set_ctrl,
};

static int gc0308_initialize_controls(struct gc0308 *gc0308)
{
	const struct gc0308_mode *mode;
	struct v4l2_ctrl_handler *handler;
	struct v4l2_ctrl *ctrl;
	s64 exposure_max, vblank_def;
	u32 h_blank;
	int ret;

	handler = &gc0308->ctrl_handler;
	mode = gc0308->cur_mode;
	ret = v4l2_ctrl_handler_init(handler, 8);
	if (ret)
		return ret;
	handler->lock = &gc0308->mutex;

	ctrl = v4l2_ctrl_new_int_menu(handler, NULL, V4L2_CID_LINK_FREQ,
				      0, 0, link_freq_menu_items);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	v4l2_ctrl_new_std(handler, NULL, V4L2_CID_PIXEL_RATE,
			  0, GC0308_PIXEL_RATE, 1, GC0308_PIXEL_RATE);

	h_blank = mode->hts_def - mode->width;
	gc0308->hblank = v4l2_ctrl_new_std(handler, NULL, V4L2_CID_HBLANK,
				h_blank, h_blank, 1, h_blank);
	if (gc0308->hblank)
		gc0308->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	vblank_def = mode->vts_def - mode->height;
	gc0308->vblank = v4l2_ctrl_new_std(handler, &gc0308_ctrl_ops,
				V4L2_CID_VBLANK, vblank_def,
				GC0308_VTS_MAX - mode->height,
				1, vblank_def);

	exposure_max = mode->vts_def - 4;
	gc0308->exposure = v4l2_ctrl_new_std(handler, &gc0308_ctrl_ops,
				V4L2_CID_EXPOSURE, GC0308_EXPOSURE_MIN,
				exposure_max, GC0308_EXPOSURE_STEP,
				mode->exp_def);

	gc0308->anal_gain = v4l2_ctrl_new_std(handler, &gc0308_ctrl_ops,
				V4L2_CID_ANALOGUE_GAIN, ANALOG_GAIN_MIN,
				ANALOG_GAIN_MAX, ANALOG_GAIN_STEP,
				ANALOG_GAIN_DEFAULT);

	/* Digital gain */
	gc0308->digi_gain = v4l2_ctrl_new_std(handler, &gc0308_ctrl_ops,
				V4L2_CID_DIGITAL_GAIN, GC0308_DIGI_GAIN_MIN,
				GC0308_DIGI_GAIN_MAX, GC0308_DIGI_GAIN_STEP,
				GC0308_DIGI_GAIN_DEFAULT);

	gc0308->test_pattern = v4l2_ctrl_new_std_menu_items(handler,
				&gc0308_ctrl_ops, V4L2_CID_TEST_PATTERN,
				ARRAY_SIZE(gc0308_test_pattern_menu) - 1,
				0, 0, gc0308_test_pattern_menu);

	if (handler->error) {
		ret = handler->error;
		dev_err(&gc0308->client->dev,
			"Failed to init controls(%d)\n", ret);
		goto err_free_handler;
	}

	gc0308->subdev.ctrl_handler = handler;

	return 0;

err_free_handler:
	v4l2_ctrl_handler_free(handler);

	return ret;
}

static int gc0308_check_sensor_id(struct gc0308 *gc0308,
				  struct i2c_client *client)
{
	struct device *dev = &gc0308->client->dev;
	u32 id = 0;
	int ret;

#if 1
	ret = gc0308_read_reg(client, GC0308_REG_CHIP_ID,
			      GC0308_REG_VALUE_24BIT, &id);
	if (id != CHIP_ID) {
		dev_err(dev, "Unexpected sensor id(%06x), ret(%d), CHIP_ID(%x)\n", id, ret, CHIP_ID);
		//return ret;
	}
#endif

	dev_info(dev, "Detected OV%06x sensor\n", CHIP_ID);

	return 0;
}

static int gc0308_configure_regulators(struct gc0308 *gc0308)
{
	int i;

	for (i = 0; i < GC0308_NUM_SUPPLIES; i++)
		gc0308->supplies[i].supply = gc0308_supply_names[i];

	return devm_regulator_bulk_get(&gc0308->client->dev,
				       GC0308_NUM_SUPPLIES,
				       gc0308->supplies);
}

static int gc0308_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct gc0308 *gc0308;
	struct v4l2_subdev *sd;
	int ret;

	printk("xieyh gc0308_probe\n");

	gc0308 = devm_kzalloc(dev, sizeof(*gc0308), GFP_KERNEL);
	if (!gc0308)
		return -ENOMEM;

	gc0308->client = client;
	gc0308->cur_mode = &supported_modes[0];

#if 0
	gc0308->xvclk = devm_clk_get(dev, "xvclk");
	if (IS_ERR(gc0308->xvclk)) {
		dev_err(dev, "Failed to get xvclk\n");
		//return -EINVAL;
	}
	ret = clk_set_rate(gc0308->xvclk, GC0308_XVCLK_FREQ);
	if (ret < 0) {
		dev_err(dev, "Failed to set xvclk rate (24MHz)\n");
		//return ret;
	}
	if (clk_get_rate(gc0308->xvclk) != GC0308_XVCLK_FREQ)
		dev_warn(dev, "xvclk mismatched, modes are based on 24MHz\n");

	gc0308->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(gc0308->reset_gpio)) {
		dev_err(dev, "Failed to get reset-gpios\n");
		//return -EINVAL;
	}

	ret = gc0308_configure_regulators(gc0308);
	if (ret) {
 		dev_err(dev, "Failed to get power regulators\n");
		//return ret;
	}
#endif
	mutex_init(&gc0308->mutex);

	sd = &gc0308->subdev;
	v4l2_i2c_subdev_init(sd, client, &gc0308_subdev_ops);
	ret = gc0308_initialize_controls(gc0308);
	if (ret)
		goto err_destroy_mutex;

	#if 0
	ret = __gc0308_power_on(gc0308);
	if (ret)
		goto err_free_handler;
	#endif
	
	printk("xieyh check sensor id\n");
	ret = gc0308_check_sensor_id(gc0308, client);
	if (ret)
		goto err_power_off;

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
	sd->internal_ops = &gc0308_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
#endif
#if defined(CONFIG_MEDIA_CONTROLLER)
	gc0308->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sd->entity, 1, &gc0308->pad);
	if (ret < 0)
		goto err_power_off;
#endif

	ret = v4l2_async_register_subdev(sd);
	if (ret) {
		dev_err(dev, "v4l2 async register subdev failed\n");
		goto err_clean_entity;
	}

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_idle(dev);

	printk("xieyh gc0308 probed end\n");

	return 0;

err_clean_entity:
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
err_power_off:
	__gc0308_power_off(gc0308);
err_free_handler:
	v4l2_ctrl_handler_free(&gc0308->ctrl_handler);
err_destroy_mutex:
	mutex_destroy(&gc0308->mutex);

	return ret;
}

static int gc0308_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct gc0308 *gc0308 = to_gc0308(sd);

	v4l2_async_unregister_subdev(sd);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
	v4l2_ctrl_handler_free(&gc0308->ctrl_handler);
	mutex_destroy(&gc0308->mutex);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		__gc0308_power_off(gc0308);
	pm_runtime_set_suspended(&client->dev);

	return 0;
}

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id gc0308_of_match[] = {
	{ .compatible = "ovti,gc0308" },
	{},
};
MODULE_DEVICE_TABLE(of, gc0308_of_match);
#endif

static struct i2c_driver gc0308_i2c_driver = {
	.driver = {
		.name = "gc0308",
		.pm = &gc0308_pm_ops,
		.of_match_table = of_match_ptr(gc0308_of_match),
	},
	.probe		= &gc0308_probe,
	.remove		= &gc0308_remove,
};

module_i2c_driver(gc0308_i2c_driver);

MODULE_DESCRIPTION("OmniVision gc0308 sensor driver");
MODULE_LICENSE("GPL v2");


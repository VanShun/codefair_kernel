/* Copyright (c) 2020 Codefair Semiconductor Technology Co., Ltd.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * This file contains WOKOO IOMUX driver that supports group
 * based PINMUX configuration.
 */

#include <linux/err.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/clk.h>

#include "../core.h"
#include "../pinctrl-utils.h"




#define WOKOO_NUM_IOMUX			32
#define WOKOO_NUM_PWM_MUX		4

#define WOKOO_PIN_MUX_BASE0		0x00
#define WOKOO_PIN_INPUT_BASE	0x110
#define WOKOO_PIN_PULL_BASE		0x70

#define WOKOO_PIN_PULL_MASK		0x01
#define WOKOO_PIN_PULL_UP		0x01
#define WOKOO_PIN_PULL_DOWN		0x02
#define WOKOO_PIN_INPUT_EN_MASK	0x01




/*
 * WOKOO IOMUX register description
 *
 * @base: base address number
 * @offset: register offset for mux configuration of a group
 * @shift: bit shift for mux configuration of a group
 * @mask: mask bits
 * @alt: alternate function to set to
 */
struct wokoo_mux {
	unsigned int base;
	unsigned int offset;
	unsigned int shift;
	unsigned int mask;
	unsigned int alt;
};

/*
 * Keep IOMUX configuration and prevent double
 * configuration
 *
 * @wokoo_mux: IOMUX register description
 * @is_configured: flag to indicate whether a mux setting has already
 * been configured
 */
struct wokoo_mux_log {
	struct wokoo_mux mux;
	bool is_configured;
};

/*
 * Group based IOMUX configuration
 *
 * @name: name of the group
 * @pins: array of pins used by this group
 * @num_pins: total number of pins used by this group
 * @mux: IOMUX configuration
 */
struct wokoo_pin_group {
	const char *name;
	const unsigned int *pins;
	const unsigned int num_pins;
	const struct wokoo_mux mux;
};

/*
 * mux function and supported pin groups
 *
 * @name: name of the function
 * @groups: array of groups that can be supported by this function
 * @num_groups: total number of groups that can be supported by function
 */
struct wokoo_pin_function {
	const char *name;
	const char * const *groups;
	const unsigned int num_groups;
};

/*
 * IOMUX pinctrl core
 *
 * @pctl: pointer to pinctrl_dev
 * @dev: pointer to device
 * @base: first IOMUX register base
 * @base_pin_pull: pull pin register base
 * @base_pin_input: input pin register base
 * @groups: pointer to array of groups
 * @num_groups: total number of groups
 * @functions: pointer to array of functions
 * @num_functions: total number of functions
 * @mux_log: pointer to the array of mux logs
 * @clk: gate clock
 * @lock: lock to protect register access
 */
struct wokoo_pinctrl {
	struct pinctrl_dev *pctl;
	struct device *dev;
	void __iomem *base;
	void __iomem *base_pin_pull;
	void __iomem *base_pin_input;

	const struct wokoo_pin_group *groups;
	unsigned int num_groups;

	const struct wokoo_pin_function *functions;
	unsigned int num_functions;

	struct wokoo_mux_log *mux_log;
	struct clk *pclk;

	spinlock_t lock;
};

/*
 * Pin configuration info
 *
 * @off_pull: pullregister offset from base
 * @off_input: input register offset from base
 * @input_en: input enable control bit shift
 * @pull_shift: pull-up/pull-down control bit shift in the register
 * @pull_mode: pull mode
 */
struct wokoo_pinconf {
	unsigned int off_pull;
	unsigned int off_input;
	unsigned int input_en;
	unsigned int pull_shift;
	unsigned int pull_mode;
};

/*
 * Description of a pin in wokoo
 *
 * @pin: pin number
 * @name: pin name
 * @pin_conf: pin configuration structure
 */
struct wokoo_pin {
	unsigned int pin;
	char *name;
	struct wokoo_pinconf pin_conf;
};

#define WOKOO_PIN_DESC(p, n, op, oi, i, pu, m)	\
{						\
	.pin = p,				\
	.name = n,				\
	.pin_conf = {				\
		.off_pull = op,			\
		.off_input = oi,			\
		.input_en = i,			\
		.pull_shift = pu,		\
		.pull_mode = m,		\
	}					\
}

/*
 * List of pins in wokoo
 */
static struct wokoo_pin wokoo_pins[] = {
	WOKOO_PIN_DESC(0, "gpio0", 0x0, 0x0, 0, 0, WOKOO_PIN_PULL_DOWN),
	WOKOO_PIN_DESC(1, "gpio1", 0x0, 0x0, 1, 1, WOKOO_PIN_PULL_DOWN),
	WOKOO_PIN_DESC(2, "gpio2", 0x0, 0x0, 2, 2, WOKOO_PIN_PULL_DOWN),
	WOKOO_PIN_DESC(3, "gpio3", 0x0, 0x0, 3, 3, WOKOO_PIN_PULL_DOWN),
	WOKOO_PIN_DESC(4, "gpio4", 0x0, 0x0, 4, 4, WOKOO_PIN_PULL_DOWN),
	WOKOO_PIN_DESC(5, "gpio5", 0x0, 0x0, 5, 5, WOKOO_PIN_PULL_DOWN),
	WOKOO_PIN_DESC(6, "gpio6", 0x0, 0x0, 6, 6, WOKOO_PIN_PULL_DOWN),
	WOKOO_PIN_DESC(7, "gpio7", 0x0, 0x0, 7, 7, WOKOO_PIN_PULL_DOWN),
	WOKOO_PIN_DESC(8, "gpio8", 0x0, 0x0, 8, 8, WOKOO_PIN_PULL_DOWN),
	WOKOO_PIN_DESC(9, "gpio9", 0x0, 0x0, 9, 9, WOKOO_PIN_PULL_DOWN),
	WOKOO_PIN_DESC(10, "gpio10", 0x0, 0x0, 10, 10, WOKOO_PIN_PULL_DOWN),
	WOKOO_PIN_DESC(11, "gpio11", 0x0, 0x0, 11, 11, WOKOO_PIN_PULL_DOWN),
	WOKOO_PIN_DESC(12, "gpio12", 0x0, 0x0, 12, 12, WOKOO_PIN_PULL_DOWN),
	WOKOO_PIN_DESC(13, "gpio13", 0x0, 0x0, 13, 13, WOKOO_PIN_PULL_DOWN),
	WOKOO_PIN_DESC(14, "gpio14", 0x0, 0x0, 14, 14, WOKOO_PIN_PULL_DOWN),
	WOKOO_PIN_DESC(15, "gpio15", 0x0, 0x0, 15, 15, WOKOO_PIN_PULL_DOWN),
	WOKOO_PIN_DESC(16, "gpio16", 0x0, 0x0, 16, 16, WOKOO_PIN_PULL_DOWN),
	WOKOO_PIN_DESC(17, "gpio17", 0x0, 0x0, 17, 17, WOKOO_PIN_PULL_DOWN),
	WOKOO_PIN_DESC(18, "gpio18", 0x0, 0x0, 18, 18, WOKOO_PIN_PULL_DOWN),
	WOKOO_PIN_DESC(19, "gpio19", 0x0, 0x0, 19, 19, WOKOO_PIN_PULL_DOWN),
	WOKOO_PIN_DESC(20, "gpio20", 0x0, 0x0, 20, 20, WOKOO_PIN_PULL_DOWN),
	WOKOO_PIN_DESC(21, "gpio21", 0x0, 0x0, 21, 21, WOKOO_PIN_PULL_DOWN),
	WOKOO_PIN_DESC(22, "gpio22", 0x0, 0x0, 22, 22, WOKOO_PIN_PULL_DOWN),
	WOKOO_PIN_DESC(23, "gpio23", 0x0, 0x0, 23, 23, WOKOO_PIN_PULL_DOWN),
	WOKOO_PIN_DESC(24, "gpio24", 0x0, 0x0, 24, 24, WOKOO_PIN_PULL_DOWN),
	WOKOO_PIN_DESC(25, "gpio25", 0x0, 0x0, 25, 25, WOKOO_PIN_PULL_DOWN),
	WOKOO_PIN_DESC(26, "gpio26", 0x0, 0x0, 26, 26, WOKOO_PIN_PULL_DOWN),
	WOKOO_PIN_DESC(27, "gpio27", 0x0, 0x0, 27, 27, WOKOO_PIN_PULL_DOWN),
	WOKOO_PIN_DESC(28, "mfio_28", 0x0, 0x0, 28, 28, WOKOO_PIN_PULL_DOWN),
	WOKOO_PIN_DESC(29, "mfio_29", 0x0, 0x0, 29, 29, WOKOO_PIN_PULL_DOWN),
	WOKOO_PIN_DESC(30, "mfio_30", 0x0, 0x0, 30, 30, WOKOO_PIN_PULL_DOWN),
	WOKOO_PIN_DESC(31, "mfio_31", 0x0, 0x0, 31, 31, WOKOO_PIN_PULL_DOWN),
	WOKOO_PIN_DESC(32, "mfio_32", 0x4, 0x4, 0, 0, WOKOO_PIN_PULL_DOWN),
	WOKOO_PIN_DESC(33, "mfio_33", 0x4, 0x4, 1, 1, WOKOO_PIN_PULL_DOWN),
	WOKOO_PIN_DESC(34, "mfio_34", 0x4, 0x4, 2, 2, WOKOO_PIN_PULL_DOWN),
	WOKOO_PIN_DESC(35, "mfio_35", 0x4, 0x4, 3, 3, WOKOO_PIN_PULL_DOWN),
	WOKOO_PIN_DESC(36, "mfio_36", 0x4, 0x4, 4, 4, WOKOO_PIN_PULL_DOWN),
	WOKOO_PIN_DESC(37, "mfio_37", 0x4, 0x4, 5, 5, WOKOO_PIN_PULL_DOWN),
	WOKOO_PIN_DESC(38, "mfio_38", 0x4, 0x4, 6, 6, WOKOO_PIN_PULL_DOWN),
	WOKOO_PIN_DESC(39, "gpio39", 0x4, 0x4, 7, 7, WOKOO_PIN_PULL_UP),
	WOKOO_PIN_DESC(40, "gpio40", 0x4, 0x4, 8, 8, WOKOO_PIN_PULL_UP),
	WOKOO_PIN_DESC(41, "mfio_41", 0x4, 0x4, 9, 9, WOKOO_PIN_PULL_UP),
	WOKOO_PIN_DESC(42, "mfio_42", 0x4, 0x4, 10, 10, WOKOO_PIN_PULL_DOWN),
	WOKOO_PIN_DESC(43, "mfio_43", 0x4, 0x4, 11, 11, WOKOO_PIN_PULL_DOWN),
	WOKOO_PIN_DESC(44, "mfio_44", 0x4, 0x4, 12, 12, WOKOO_PIN_PULL_DOWN),
	WOKOO_PIN_DESC(45, "mfio_45", 0x4, 0x4, 13, 13, WOKOO_PIN_PULL_UP),
	WOKOO_PIN_DESC(46, "mfio_46", 0x4, 0x4, 14, 14, WOKOO_PIN_PULL_UP),
	WOKOO_PIN_DESC(47, "mfio_47", 0x4, 0x4, 15, 15, WOKOO_PIN_PULL_UP),
	WOKOO_PIN_DESC(48, "mfio_48", 0x4, 0x4, 16, 16, WOKOO_PIN_PULL_DOWN),
	WOKOO_PIN_DESC(49, "mfio_49", 0x4, 0x4, 17, 17, WOKOO_PIN_PULL_DOWN),
	WOKOO_PIN_DESC(50, "mfio_50", 0x4, 0x4, 18, 18, WOKOO_PIN_PULL_DOWN),
	WOKOO_PIN_DESC(51, "mfio_51", 0x4, 0x4, 19, 19, WOKOO_PIN_PULL_DOWN),
	WOKOO_PIN_DESC(52, "mfio_52", 0x4, 0x4, 20, 20, WOKOO_PIN_PULL_DOWN),
	WOKOO_PIN_DESC(53, "mfio_53", 0x4, 0x4, 21, 21, WOKOO_PIN_PULL_UP),
	WOKOO_PIN_DESC(54, "mfio_54", 0x4, 0x4, 22, 22, WOKOO_PIN_PULL_UP),
	WOKOO_PIN_DESC(55, "mfio_55", 0x4, 0x4, 23, 23, WOKOO_PIN_PULL_UP),
	WOKOO_PIN_DESC(56, "mfio_56", 0x4, 0x4, 24, 24, WOKOO_PIN_PULL_DOWN),
	WOKOO_PIN_DESC(57, "mfio_57", 0x4, 0x4, 25, 25, WOKOO_PIN_PULL_UP), 
	WOKOO_PIN_DESC(58, "mfio_58", 0x4, 0x4, 26, 26, WOKOO_PIN_PULL_UP), 
	WOKOO_PIN_DESC(59, "mfio_59", 0x4, 0x4, 27, 27, WOKOO_PIN_PULL_UP), 
	WOKOO_PIN_DESC(60, "mfio_60", 0x4, 0x4, 28, 28, WOKOO_PIN_PULL_DOWN),
	WOKOO_PIN_DESC(61, "mfio_61", 0x4, 0x4, 29, 29, WOKOO_PIN_PULL_UP),
	WOKOO_PIN_DESC(62, "mfio_62", 0x4, 0x4, 30, 30, WOKOO_PIN_PULL_UP),
	WOKOO_PIN_DESC(63, "mfio_63", 0x4, 0x4, 31, 31, WOKOO_PIN_PULL_UP),
	WOKOO_PIN_DESC(64, "mfio_64", 0x8, 0x8, 0, 0, WOKOO_PIN_PULL_DOWN),
	WOKOO_PIN_DESC(65, "mfio_65", 0x8, 0x8, 1, 1, WOKOO_PIN_PULL_DOWN),
	WOKOO_PIN_DESC(66, "mfio_66", 0x8, 0x8, 2, 2, WOKOO_PIN_PULL_DOWN),
	WOKOO_PIN_DESC(67, "mfio_67", 0x8, 0x8, 3, 3, WOKOO_PIN_PULL_DOWN),
	WOKOO_PIN_DESC(68, "mfio_68", 0x8, 0x8, 4, 4, WOKOO_PIN_PULL_UP),
	WOKOO_PIN_DESC(69, "mfio_69", 0x8, 0x8, 5, 5, WOKOO_PIN_PULL_UP),
	WOKOO_PIN_DESC(70, "mfio_70", 0x8, 0x8, 6, 6, WOKOO_PIN_PULL_UP),
	WOKOO_PIN_DESC(71, "mfio_71", 0x8, 0x8, 7, 7, WOKOO_PIN_PULL_DOWN),
	WOKOO_PIN_DESC(72, "mfio_72", 0x8, 0x8, 8, 8, WOKOO_PIN_PULL_DOWN),
	WOKOO_PIN_DESC(73, "mfio_73", 0x8, 0x8, 9, 9, WOKOO_PIN_PULL_DOWN),
	WOKOO_PIN_DESC(74, "mfio_74", 0x8, 0x8, 10, 10, WOKOO_PIN_PULL_UP),
	WOKOO_PIN_DESC(75, "mfio_75", 0x8, 0x8, 11, 11, WOKOO_PIN_PULL_UP),
	WOKOO_PIN_DESC(76, "mfio_76", 0x8, 0x8, 12, 12, WOKOO_PIN_PULL_UP),
	WOKOO_PIN_DESC(77, "mfio_77", 0x8, 0x8, 13, 13, WOKOO_PIN_PULL_UP),
	WOKOO_PIN_DESC(78, "mfio_78", 0x8, 0x8, 14, 14, WOKOO_PIN_PULL_DOWN),
	WOKOO_PIN_DESC(79, "mfio_79", 0x8, 0x8, 15, 15, WOKOO_PIN_PULL_DOWN),
	WOKOO_PIN_DESC(80, "mfio_80", 0x8, 0x8, 16, 16, WOKOO_PIN_PULL_UP),
	WOKOO_PIN_DESC(81, "mfio_81", 0x8, 0x8, 17, 17, WOKOO_PIN_PULL_DOWN),
	WOKOO_PIN_DESC(82, "mfio_82", 0x8, 0x8, 18, 18, WOKOO_PIN_PULL_UP),
	WOKOO_PIN_DESC(83, "mfio_83", 0x8, 0x8, 19, 19, WOKOO_PIN_PULL_UP),
	WOKOO_PIN_DESC(84, "mfio_84", 0x8, 0x8, 20, 20, WOKOO_PIN_PULL_UP),
	WOKOO_PIN_DESC(85, "mfio_85", 0x8, 0x8, 21, 21, WOKOO_PIN_PULL_DOWN),
	WOKOO_PIN_DESC(86, "mfio_86", 0x8, 0x8, 22, 22, WOKOO_PIN_PULL_DOWN),
	WOKOO_PIN_DESC(87, "mfio_87", 0x8, 0x8, 23, 23, WOKOO_PIN_PULL_DOWN),
	WOKOO_PIN_DESC(88, "mfio_88", 0x8, 0x8, 24, 24, WOKOO_PIN_PULL_DOWN),
	WOKOO_PIN_DESC(89, "mfio_89", 0x8, 0x8, 25, 25, WOKOO_PIN_PULL_DOWN),
	WOKOO_PIN_DESC(90, "mfio_90", 0x8, 0x8, 26, 26, WOKOO_PIN_PULL_DOWN),
	WOKOO_PIN_DESC(91, "mfio_91", 0x8, 0x8, 27, 27, WOKOO_PIN_PULL_DOWN),
	WOKOO_PIN_DESC(92, "mfio_92", 0x8, 0x8, 28, 28, WOKOO_PIN_PULL_DOWN),
	WOKOO_PIN_DESC(93, "mfio_93", 0x8, 0x8, 29, 29, WOKOO_PIN_PULL_DOWN),
	WOKOO_PIN_DESC(94, "mfio_94", 0x8, 0x8, 30, 30, WOKOO_PIN_PULL_DOWN),
	WOKOO_PIN_DESC(95, "mfio_95", 0x8, 0x8, 31, 31, WOKOO_PIN_PULL_DOWN),
	WOKOO_PIN_DESC(96, "mfio_96", 0xc, 0xc, 0, 0, WOKOO_PIN_PULL_DOWN),
	WOKOO_PIN_DESC(97, "mfio_97", 0xc, 0xc, 1, 1, WOKOO_PIN_PULL_DOWN),
	WOKOO_PIN_DESC(98, "mfio_98", 0xc, 0xc, 2, 2, WOKOO_PIN_PULL_DOWN),
	WOKOO_PIN_DESC(99, "mfio_99", 0xc, 0xc, 3, 3, WOKOO_PIN_PULL_DOWN),
	WOKOO_PIN_DESC(100, "mfio_100", 0xc, 0xc, 4, 4, WOKOO_PIN_PULL_DOWN),
	WOKOO_PIN_DESC(101, "mfio_101", 0xc, 0xc, 5, 5, WOKOO_PIN_PULL_DOWN),
	WOKOO_PIN_DESC(102, "mfio_102", 0xc, 0xc, 6, 6, WOKOO_PIN_PULL_DOWN),
	WOKOO_PIN_DESC(103, "mfio_103", 0xc, 0xc, 7, 7, WOKOO_PIN_PULL_DOWN),
	WOKOO_PIN_DESC(104, "mfio_104", 0xc, 0xc, 8, 8, WOKOO_PIN_PULL_DOWN),
	WOKOO_PIN_DESC(105, "mfio_105", 0xc, 0xc, 9, 9, WOKOO_PIN_PULL_DOWN),
	WOKOO_PIN_DESC(106, "mfio_106", 0xc, 0xc, 10, 10, WOKOO_PIN_PULL_DOWN),
	WOKOO_PIN_DESC(107, "mfio_107", 0xc, 0xc, 11, 11, WOKOO_PIN_PULL_DOWN),
	WOKOO_PIN_DESC(108, "mfio_108", 0xc, 0xc, 12, 12, WOKOO_PIN_PULL_DOWN),
	WOKOO_PIN_DESC(109, "mfio_109", 0xc, 0xc, 13, 13, WOKOO_PIN_PULL_DOWN),
	WOKOO_PIN_DESC(110, "mfio_110", 0xc, 0xc, 14, 14, WOKOO_PIN_PULL_DOWN),
	WOKOO_PIN_DESC(111, "mfio_111", 0xc, 0xc, 15, 15, WOKOO_PIN_PULL_DOWN),
	WOKOO_PIN_DESC(112, "mfio_112", 0xc, 0xc, 16, 16, WOKOO_PIN_PULL_DOWN),
	WOKOO_PIN_DESC(113, "mfio_113", 0xc, 0xc, 17, 17, WOKOO_PIN_PULL_DOWN),
	WOKOO_PIN_DESC(114, "mfio_114", 0xc, 0xc, 18, 18, WOKOO_PIN_PULL_DOWN),
	WOKOO_PIN_DESC(115, "mfio_115", 0xc, 0xc, 19, 19, WOKOO_PIN_PULL_DOWN),
	WOKOO_PIN_DESC(116, "mfio_116", 0xc, 0xc, 20, 20, WOKOO_PIN_PULL_DOWN),
	WOKOO_PIN_DESC(117, "mfio_117", 0xc, 0xc, 21, 21, WOKOO_PIN_PULL_DOWN),
	WOKOO_PIN_DESC(118, "mfio_118", 0xc, 0xc, 22, 22, WOKOO_PIN_PULL_DOWN),
	WOKOO_PIN_DESC(119, "mfio_119", 0xc, 0xc, 23, 23, WOKOO_PIN_PULL_DOWN),
	WOKOO_PIN_DESC(120, "mfio_120", 0xc, 0xc, 24, 24, WOKOO_PIN_PULL_DOWN),
	WOKOO_PIN_DESC(121, "mfio_121", 0xc, 0xc, 25, 25, WOKOO_PIN_PULL_DOWN),
	WOKOO_PIN_DESC(122, "mfio_122", 0xc, 0xc, 26, 26, WOKOO_PIN_PULL_DOWN),
	WOKOO_PIN_DESC(123, "mfio_123", 0xc, 0xc, 27, 27, WOKOO_PIN_PULL_DOWN),
	WOKOO_PIN_DESC(124, "mfio_124", 0xc, 0xc, 28, 28, WOKOO_PIN_PULL_DOWN),
	WOKOO_PIN_DESC(125, "mfio_125", 0xc, 0xc, 29, 29, WOKOO_PIN_PULL_DOWN),
	WOKOO_PIN_DESC(126, "mfio_126", 0xc, 0xc, 30, 30, WOKOO_PIN_PULL_DOWN),
	WOKOO_PIN_DESC(127, "mfio_127", 0xc, 0xc, 31, 31, WOKOO_PIN_PULL_DOWN),
	WOKOO_PIN_DESC(128, "lcdc_d23", -1, -1, 0, 0, 0),
};

/*
 * List of groups of pins
 */

static const unsigned int gpio_0_27_pins[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10,
											11, 12, 13, 14, 15, 16, 17, 18, 19, 20,
											21, 22, 23, 24, 25, 26, 27};

static const unsigned int i2s1_pins[] = {28, 29, 30, 31};
static const unsigned int gpio_28_31_pins[] = {28, 29, 30, 31};

static const unsigned int isp_clk_pins[] = {32};
static const unsigned int gpio_32_pins[] = {32};

static const unsigned int isp_rst_pins[] = {33, 34, 35};
static const unsigned int gpio_33_35_pins[] = {33, 34, 35};

static const unsigned int isp_thv_pins[] = { 37, 38};
static const unsigned int gpio_37_38_pins[] = { 37, 38};
static const unsigned int gpio_36_pins[] = { 36};

static const unsigned int isp_fun_pins[] = { 39, 40, 41, 42};
static const unsigned int gpio_39_42_pins[] = {39, 40, 41, 42};

// static const unsigned int isp_fun_pins[] = {32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42};
// static const unsigned int gpio_32_42_pins[] = {32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42};

static const unsigned int isp_d0_d9_pins[] = {43, 44, 45, 46, 47, 48, 49, 50, 51, 52};
static const unsigned int gpio_43_52_pins[] = {43, 44, 45, 46, 47, 48, 49, 50, 51, 52};

static const unsigned int gpio_53_54_pins[] = {53, 54};
static const unsigned int uart0_tx_rx_pins[] = {53, 54};

static const unsigned int gpio_55_56_pins[] = {55, 56};
static const unsigned int uart0_cts_rts_pins[] = {55, 56};

static const unsigned int gpio_57_58_pins[] = {57, 58};
static const unsigned int uart1_tx_rx_pins[] = {57, 58};

static const unsigned int gpio_59_60_pins[] = {59, 60};
static const unsigned int uart1_cts_rts_pins[] = {59, 60};

static const unsigned int gpio_61_62_pins[] = {61, 62};
static const unsigned int uart2_tx_rx_pins[] = {61, 62};

static const unsigned int gpio_63_64_pins[] = {63, 64};
static const unsigned int uart2_cts_rts_pins[] = {63, 64};

static const unsigned int gpio_65_69_pins[] = {65, 66, 67, 68, 69};
static const unsigned int ssi0_pins[] = {65, 66, 67, 68, 69};

static const unsigned int gpio_70_73_pins[] = {70, 71, 72, 73};
static const unsigned int i2s0_pins[] = {70, 71, 72, 73};

static const unsigned int gpio_74_75_pins[] = {74, 75};
static const unsigned int i2c0_pins[] = {74, 75};

static const unsigned int gpio_76_77_pins[] = {76, 77};
static const unsigned int i2c1_pins[] = {76, 77};

static const unsigned int gpio_78_82_pins[] = {78, 79, 80, 81, 82};
static const unsigned int jtag_pins[] = {78, 79, 80, 81, 82};

static const unsigned int gpio_83_84_pins[] = {83, 84};
static const unsigned int pmu_pins[] = {83, 84};

static const unsigned int gpio_85_pins[] = {85};
static const unsigned int osc_en_3v_pins[] = {85};

static const unsigned int gpio_86_pins[] = {86};
static const unsigned int gpio_87_pins[] = {87};
static const unsigned int gpio_88_pins[] = {88};
static const unsigned int gpio_89_pins[] = {89};
static const unsigned int clkout0_pins[] = {86};
static const unsigned int clkout1_pins[] = {87};
static const unsigned int clkout2_pins[] = {88};
static const unsigned int clkout3_pins[] = {89};

static const unsigned int gpio_90_91_pins[] = {90, 91};
static const unsigned int pmu_dvfs_pins[] = {90, 91};

static const unsigned int gpio_92_pins[] = {92};
static const unsigned int usb_vbus_pins[] = {92};

static const unsigned int gpio_93_pins[] = {93};
static const unsigned int gpio_94_pins[] = {94};
static const unsigned int gpio_95_pins[] = {95};
static const unsigned int gpio_96_pins[] = {96};
static const unsigned int pwm0_pins[] = {93};
static const unsigned int pwm1_pins[] = {94};
static const unsigned int pwm2_pins[] = {95};
static const unsigned int pwm3_pins[] = {96};

static const unsigned int gpio_97_98_pins[] = {97, 98};
static const unsigned int isp_d10_d11_pins[] = {97, 98};

static const unsigned int gpio_99_104_pins[] = {99, 100, 101, 102, 103, 104};
static const unsigned int lcdc_fun_pins[] = {99, 100, 101, 102, 103, 104};

static const unsigned int gpio_105_107_pins[] = {105, 106, 107};
static const unsigned int lcdc_d0_d2_pins[] = {105, 106, 107};

static const unsigned int gpio_108_123_pins[] = {108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123};
static const unsigned int lcdc_d3_d18_pins[] = {108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123};

static const unsigned int gpio_124_127_pins[] = {124, 125, 126, 127};
static const unsigned int lcdc_d19_d23_pins[] = {124, 125, 126, 127, 128};




#define WOKOO_PIN_GROUP(group_name, ba, off, sh, ma, al)	\
{							\
	.name = __stringify(group_name) "_grp",		\
	.pins = group_name ## _pins,			\
	.num_pins = ARRAY_SIZE(group_name ## _pins),	\
	.mux = {					\
		.base = ba,				\
		.offset = off,				\
		.shift = sh,				\
		.mask = ma,				\
		.alt = al,				\
	}						\
}

/*
 * List of wokoo pin groups
 */
static const struct wokoo_pin_group wokoo_pin_groups[] = {
	WOKOO_PIN_GROUP(gpio_0_27, 0, 0, 0, 0, 0),

	/* BIT[0~1] = 0x0, BIT[2~3] = 0x0, BIT[4~5] = 0x0 */
	WOKOO_PIN_GROUP(i2s1, 0, 0x1c, 10, 0xff, 0x0),
	/* BIT[0~1] = 0x2, BIT[2~3] = 0x2, BIT[4~5] = 0x2 */
	WOKOO_PIN_GROUP(gpio_28_31, 0, 0x1c, 10, 0xff, 0xaa),

	WOKOO_PIN_GROUP(isp_clk, 0, 0x04, 0, 0x3, 0x0),
	WOKOO_PIN_GROUP(gpio_32, 0, 0x04, 0, 0x3, 0x2),

	WOKOO_PIN_GROUP(isp_rst, 0, 0x04, 2, 0x3f, 0x0),
	WOKOO_PIN_GROUP(gpio_33_35, 0, 0x04, 2, 0x3f, 0x2a),

	WOKOO_PIN_GROUP(isp_thv, 0, 0x04, 10, 0xf, 0x0),
	WOKOO_PIN_GROUP(gpio_37_38, 0, 0x04, 10, 0xf, 0xa),
	WOKOO_PIN_GROUP(gpio_36, 0, 0x04, 8, 0x3, 0x2),

	WOKOO_PIN_GROUP(isp_fun, 0, 0x04, 14, 0xff, 0x0),
	WOKOO_PIN_GROUP(gpio_39_42, 0, 0x04, 14, 0xff, 0xaa),

	WOKOO_PIN_GROUP(isp_d0_d9,	0, 0x08, 0, 0xfffff, 0x0),
	WOKOO_PIN_GROUP(gpio_43_52, 0, 0x08, 0, 0xfffff, 0xaaaaa),

	WOKOO_PIN_GROUP(uart0_tx_rx, 0, 0x0c, 0, 0xf, 0x0),
	WOKOO_PIN_GROUP(gpio_53_54, 0, 0x0c, 0, 0xf, 0xa),

	WOKOO_PIN_GROUP(uart0_cts_rts, 0, 0x0c, 4, 0xf, 0x0),
	WOKOO_PIN_GROUP(gpio_55_56, 0, 0x0c, 4, 0xf, 0xa),

	WOKOO_PIN_GROUP(uart1_tx_rx, 0, 0x0c, 8, 0xf, 0x0),
	WOKOO_PIN_GROUP(gpio_57_58, 0, 0x0c, 8, 0xf, 0xa),

	WOKOO_PIN_GROUP(uart1_cts_rts, 0, 0x0c, 12, 0xf, 0x0),
	WOKOO_PIN_GROUP(gpio_59_60, 0, 0x0c, 12, 0xf, 0xa),

	WOKOO_PIN_GROUP(uart2_tx_rx, 0, 0x0c, 16, 0xf, 0x0),
	WOKOO_PIN_GROUP(gpio_61_62, 0, 0x0c, 16, 0xf, 0xa),

	WOKOO_PIN_GROUP(uart2_cts_rts, 0, 0x0c, 20, 0xf, 0x0),
	WOKOO_PIN_GROUP(gpio_63_64, 0, 0x0c, 20, 0xf, 0xa),

	WOKOO_PIN_GROUP(ssi0, 0, 0x10, 0, 0x3ff, 0x0),
	WOKOO_PIN_GROUP(gpio_65_69, 0, 0x10, 0, 0x3ff, 0x2aa),

	WOKOO_PIN_GROUP(i2s0, 0, 0x10, 10, 0xff, 0x0),
	WOKOO_PIN_GROUP(gpio_70_73, 0, 0x10, 10, 0xff, 0xaa),

	WOKOO_PIN_GROUP(i2c0, 0, 0x10, 18, 0xf, 0x0),
	WOKOO_PIN_GROUP(gpio_74_75, 0, 0x10, 18, 0xf, 0xa),

	WOKOO_PIN_GROUP(i2c1, 0, 0x10, 22, 0xf, 0x0),
	WOKOO_PIN_GROUP(gpio_76_77, 0, 0x10, 22, 0xf, 0xa),

	WOKOO_PIN_GROUP(jtag, 0, 0x13, 0, 0x3ff, 0x0),
	WOKOO_PIN_GROUP(gpio_78_82, 0, 0x13, 0, 0x3ff, 0x2aa),

	WOKOO_PIN_GROUP(pmu, 0, 0x00, 0, 0xf, 0x0),
	WOKOO_PIN_GROUP(gpio_83_84, 0, 0x00, 0, 0xf, 0xa),

	WOKOO_PIN_GROUP(osc_en_3v, 0, 0x00, 4, 0x3, 0x0),
	WOKOO_PIN_GROUP(gpio_85, 0, 0x00, 4, 0x3, 0x2),

	WOKOO_PIN_GROUP(clkout0, 0, 0x00, 6, 0x3, 0x0),
	WOKOO_PIN_GROUP(gpio_86, 0, 0x00, 6, 0x3, 0x2),

	WOKOO_PIN_GROUP(clkout1, 0, 0x00, 8, 0x3, 0x0),
	WOKOO_PIN_GROUP(gpio_87, 0, 0x00, 8, 0x3, 0x2),

	WOKOO_PIN_GROUP(clkout2, 0, 0x00, 10, 0x3, 0x0),
	WOKOO_PIN_GROUP(gpio_88, 0, 0x00, 10, 0x3, 0x2),

	WOKOO_PIN_GROUP(clkout3, 0, 0x00, 12, 0x3, 0x0),
	WOKOO_PIN_GROUP(gpio_89, 0, 0x00, 12, 0x3, 0x2),

	WOKOO_PIN_GROUP(pmu_dvfs, 0, 0x00, 14, 0xf, 0x0),
	WOKOO_PIN_GROUP(gpio_90_91, 0, 0x00, 14, 0xf, 0xa),

	WOKOO_PIN_GROUP(usb_vbus, 0, 0x00, 18, 0x3, 0x0),
	WOKOO_PIN_GROUP(gpio_92, 0, 0x00, 18, 0x3, 0x2),

	WOKOO_PIN_GROUP(pwm0, 0, 0x00, 20, 0x3, 0x0),
	WOKOO_PIN_GROUP(gpio_93, 0, 0x00, 20, 0x3, 0x2),

	WOKOO_PIN_GROUP(pwm1, 0, 0x00, 22, 0x3, 0x0),
	WOKOO_PIN_GROUP(gpio_94, 0, 0x00, 22, 0x3, 0x2),

	WOKOO_PIN_GROUP(pwm2, 0, 0x00, 24, 0x3, 0x0),
	WOKOO_PIN_GROUP(gpio_95, 0, 0x00, 24, 0x3, 0x2),

	WOKOO_PIN_GROUP(pwm3, 0, 0x00, 26, 0x3, 0x0),
	WOKOO_PIN_GROUP(gpio_96, 0, 0x00, 26, 0x3, 0x2),

	WOKOO_PIN_GROUP(isp_d10_d11, 0, 0x14, 10, 0xf, 0x0),
	WOKOO_PIN_GROUP(gpio_97_98, 0, 0x14, 10, 0xf, 0xa),

	WOKOO_PIN_GROUP(lcdc_fun, 0, 0x14, 14, 0xfff, 0x0),
	WOKOO_PIN_GROUP(gpio_99_104, 0, 0x14, 14, 0xfff, 0xaaa),

	WOKOO_PIN_GROUP(lcdc_d0_d2, 0, 0x14, 26, 0x3f, 0x0),
	WOKOO_PIN_GROUP(gpio_105_107, 0, 0x14, 26, 0x3f, 0x2a),

	WOKOO_PIN_GROUP(lcdc_d3_d18, 0, 0x18, 0, 0xffffffff, 0x0),
	WOKOO_PIN_GROUP(gpio_108_123, 0, 0x18, 0, 0xffffffff, 0xaaaaaaaa),

	WOKOO_PIN_GROUP(lcdc_d19_d23, 0, 0x1c, 0, 0x3ff, 0x0),
	WOKOO_PIN_GROUP(gpio_124_127, 0, 0x1c, 0, 0xff, 0xaa),
};

/*
 * List of groups supported by functions
 */

static const char * const i2s0_grps[] = {"i2s0_grp"};
static const char * const i2s1_grps[] = {"i2s1_grp"};
static const char * const i2c0_grps[] = {"i2c0_grp"};
static const char * const i2c1_grps[] = {"i2c1_grp"};
static const char * const ssi0_grps[] = {"ssi0_grp"};
static const char * const isp_grps[] = {"isp_clk_grp", "isp_rst_grp", "isp_thv_grp","isp_fun_grp", "isp_d0_d9_grp", "isp_d10_d11_grp"};
static const char * const uart0_grps[] = {"uart0_cts_rts_grp", "uart0_tx_rx_grp"};
static const char * const uart1_grps[] = {"uart1_cts_rts_grp", "uart1_tx_rx_grp"};
static const char * const uart2_grps[] = {"uart2_cts_rts_grp", "uart2_tx_rx_grp"};
static const char * const jtag_grps[] = {"jtag_grp"};
static const char * const pmu_grps[] = {"pmu_grp", "pmu_dvfs_grp"};
static const char * const pwm_grps[] = {"pwm0_grp", "pwm1_grp", "pwm2_grp", "pwm3_grp"};
static const char * const clkout_grps[] = {"clkout0_grp", "clkout1_grp", "clkout2_grp", "clkout3_grp"};
static const char * const lcdc_grps[] = {"lcdc_fun_grp", "lcdc_d0_d2_grp", "lcdc_d3_d18_grp", "lcdc_d19_d23_grp"};

static const char * const gpio_grps[] = {"gpio_0_27_grp", "gpio_28_31_grp",
    "gpio_32_grp", "gpio_33_35_grp",	"gpio_37_38_grp","gpio_36_grp",
	"gpio_39_42_grp", "gpio_43_52_grp",	"gpio_53_54_grp", "gpio_55_56_grp",
	"gpio_57_58_grp", "gpio_59_60_grp", "gpio_61_62_grp", "gpio_63_64_grp",
	"gpio_65_69_grp", "gpio_70_73_grp", "gpio_74_75_grp", "gpio_76_77_grp",
	"gpio_78_82_grp", "gpio_83_84_grp", "gpio_85_grp",
	"gpio_86_grp",    "gpio_87_grp",    "gpio_88_grp",    "gpio_89_grp",
	"gpio_90_91_grp", "gpio_92_grp",    "gpio_93_grp",    "gpio_94_grp",
	"gpio_95_grp",    "gpio_96_grp",    "gpio_97_98_grp", "gpio_99_104_grp",
	"gpio_105_107_grp", "gpio_108_123_grp", "gpio_124_127_grp",
};

#define WOKOO_PIN_FUNCTION(func)				\
{							\
	.name = #func,					\
	.groups = func ## _grps,			\
	.num_groups = ARRAY_SIZE(func ## _grps),	\
}

/*
 * List of supported functions
 */
static const struct wokoo_pin_function wokoo_pin_functions[] = {
	WOKOO_PIN_FUNCTION(i2s0),
	WOKOO_PIN_FUNCTION(i2s1),
	WOKOO_PIN_FUNCTION(gpio),
	WOKOO_PIN_FUNCTION(uart0),
	WOKOO_PIN_FUNCTION(uart1),
	WOKOO_PIN_FUNCTION(uart2),
	WOKOO_PIN_FUNCTION(pwm),
	WOKOO_PIN_FUNCTION(clkout),
	WOKOO_PIN_FUNCTION(isp),
	WOKOO_PIN_FUNCTION(jtag),
	WOKOO_PIN_FUNCTION(pmu),
	WOKOO_PIN_FUNCTION(lcdc),
};

static int wokoo_get_groups_count(struct pinctrl_dev *pctrl_dev)
{
	struct wokoo_pinctrl *pinctrl = pinctrl_dev_get_drvdata(pctrl_dev);

	return pinctrl->num_groups;
}

static const char *wokoo_get_group_name(struct pinctrl_dev *pctrl_dev,
						unsigned int selector)
{
	struct wokoo_pinctrl *pinctrl = pinctrl_dev_get_drvdata(pctrl_dev);

	return pinctrl->groups[selector].name;
}

static int wokoo_get_group_pins(struct pinctrl_dev *pctrl_dev,
					unsigned int selector, const unsigned int **pins,
					unsigned int *num_pins)
{
	struct wokoo_pinctrl *pinctrl = pinctrl_dev_get_drvdata(pctrl_dev);

	*pins = pinctrl->groups[selector].pins;
	*num_pins = pinctrl->groups[selector].num_pins;

	return 0;
}

static void wokoo_pin_dbg_show(struct pinctrl_dev *pctrl_dev,
				struct seq_file *s, unsigned int offset)
{
	seq_printf(s, " %s", dev_name(pctrl_dev->dev));
}

static const struct pinctrl_ops wokoo_pinctrl_ops = {
	.get_groups_count = wokoo_get_groups_count,
	.get_group_name = wokoo_get_group_name,
	.get_group_pins = wokoo_get_group_pins,
	.pin_dbg_show = wokoo_pin_dbg_show,
	.dt_node_to_map = pinconf_generic_dt_node_to_map_pin,
	.dt_free_map = pinctrl_utils_free_map,
};

static int wokoo_get_functions_count(struct pinctrl_dev *pctrl_dev)
{
	struct wokoo_pinctrl *pinctrl = pinctrl_dev_get_drvdata(pctrl_dev);

	return pinctrl->num_functions;
}

static const char *wokoo_get_function_name(struct pinctrl_dev *pctrl_dev,
					 unsigned int selector)
{
	struct wokoo_pinctrl *pinctrl = pinctrl_dev_get_drvdata(pctrl_dev);

	return pinctrl->functions[selector].name;
}

static int wokoo_get_function_groups(struct pinctrl_dev *pctrl_dev,
					unsigned int selector,
					const char * const **groups,
					unsigned int * const num_groups)
{
	struct wokoo_pinctrl *pinctrl = pinctrl_dev_get_drvdata(pctrl_dev);

	*groups = pinctrl->functions[selector].groups;
	*num_groups = pinctrl->functions[selector].num_groups;

	return 0;
}

/*
 *  wokoo_pinmux_set - pin mux setting
 *
 */
static int wokoo_pinmux_set(struct wokoo_pinctrl *pinctrl,
				const struct wokoo_pin_function *func,
				const struct wokoo_pin_group *grp,
				struct wokoo_mux_log *mux_log)
{
	const struct wokoo_mux *mux = &grp->mux;
	int i;
	u32 val, mask;
	unsigned long flags;
	void __iomem *base_address;

	for (i = 0; i < WOKOO_NUM_IOMUX; i++) {
		if ((mux->shift  != mux_log[i].mux.shift) ||
			(mux->base   != mux_log[i].mux.base) ||
			(mux->offset != mux_log[i].mux.offset))
			continue;

		/* if this is a new configuration, just do it! */
		if (!mux_log[i].is_configured)
			break;

		/*
		 * IOMUX has been configured previously and one is trying to
		 * configure it to a different function
		 */
		if (mux_log[i].mux.alt != mux->alt) {
			dev_err(pinctrl->dev,
				"double configuration error detected!\n");
			dev_err(pinctrl->dev, "func:%s grp:%s\n",
				func->name, grp->name);
			return -EINVAL;
		}

		return 0;
	}

	if (i == WOKOO_NUM_IOMUX)
		return -EINVAL;

	mask = mux->mask;
	mux_log[i].mux.alt = mux->alt;
	mux_log[i].is_configured = true;

	base_address = pinctrl->base;

	spin_lock_irqsave(&pinctrl->lock, flags);
	val = readl(base_address + grp->mux.offset);
	val &= ~(mask << grp->mux.shift);
	val |= grp->mux.alt << grp->mux.shift;
	writel(val, (base_address + grp->mux.offset));
	spin_unlock_irqrestore(&pinctrl->lock, flags);

	return 0;
}

/*
 *  wokoo_pinmux_enable - pin mux enable
 *
 */
static int wokoo_pinmux_enable(struct pinctrl_dev *pctrl_dev,
				unsigned int func_select, unsigned int grp_select)
{
	struct wokoo_pinctrl *pinctrl = pinctrl_dev_get_drvdata(pctrl_dev);
	const struct wokoo_pin_function *func;
	const struct wokoo_pin_group *grp;

	if (grp_select >= pinctrl->num_groups ||
		func_select >= pinctrl->num_functions)
		return -EINVAL;

	func = &pinctrl->functions[func_select];
	grp = &pinctrl->groups[grp_select];

	dev_dbg(pctrl_dev->dev, "func:%u name:%s grp:%u name:%s\n",
		func_select, func->name, grp_select, grp->name);

	dev_dbg(pctrl_dev->dev, "offset:0x%08x shift:%u alt:%u\n",
		grp->mux.offset, grp->mux.shift, grp->mux.alt);

	return wokoo_pinmux_set(pinctrl, func, grp, pinctrl->mux_log);
}

/*
 *  wokoo_pin_set_enable - pin input enable
 *
 */
static int wokoo_pin_set_enable(struct pinctrl_dev *pctrldev, unsigned int pin,
				u16 enable)
{
	struct wokoo_pinctrl *pinctrl = pinctrl_dev_get_drvdata(pctrldev);
	struct wokoo_pin *pin_data = pctrldev->desc->pins[pin].drv_data;
	unsigned long flags;
	u32 val;
	void __iomem *base_address;

	base_address = pinctrl->base_pin_input;
	spin_lock_irqsave(&pinctrl->lock, flags);
	val = readl(base_address + pin_data->pin_conf.off_input);
	val &= ~(WOKOO_PIN_INPUT_EN_MASK << pin_data->pin_conf.input_en);

	if (enable)
		val |= WOKOO_PIN_INPUT_EN_MASK << pin_data->pin_conf.input_en;

	writel(val, (base_address + pin_data->pin_conf.off_input));
	spin_unlock_irqrestore(&pinctrl->lock, flags);

	dev_dbg(pctrldev->dev, "pin:%u set enable:%d\n", pin, enable);
	return 0;
}

/*
 *  wokoo_pin_get_enable - get pin input enable
 *
 */
static int wokoo_pin_get_enable(struct pinctrl_dev *pctrldev, unsigned int pin)
{
	struct wokoo_pinctrl *pinctrl = pinctrl_dev_get_drvdata(pctrldev);
	struct wokoo_pin *pin_data = pctrldev->desc->pins[pin].drv_data;
	unsigned long flags;
	int enable;

	spin_lock_irqsave(&pinctrl->lock, flags);
	enable = readl(pinctrl->base_pin_input + pin_data->pin_conf.off_input);
	enable = (enable >> pin_data->pin_conf.input_en) &
			WOKOO_PIN_INPUT_EN_MASK;
	spin_unlock_irqrestore(&pinctrl->lock, flags);

	if (enable)
		enable = WOKOO_PIN_INPUT_EN_MASK;
	else
		enable = 0;

	dev_dbg(pctrldev->dev, "pin:%u get disable:%d\n", pin, enable);
	return enable;
}

/*
 *  wokoo_pin_set_pull - pin set pull up/down
 *
 */
static int wokoo_pin_set_pull(struct pinctrl_dev *pctrldev, unsigned int pin,
				bool pull_up, bool pull_down)
{
	struct wokoo_pinctrl *pinctrl = pinctrl_dev_get_drvdata(pctrldev);
	struct wokoo_pin *pin_data = pctrldev->desc->pins[pin].drv_data;
	unsigned long flags;
	u32 val;
	void __iomem *base_address;

	base_address = pinctrl->base_pin_pull;
	spin_lock_irqsave(&pinctrl->lock, flags);
	val = readl(base_address + pin_data->pin_conf.off_pull);
	val |= 0x1 << pin_data->pin_conf.pull_shift;

	if (pull_up == true && pin_data->pin_conf.pull_mode == WOKOO_PIN_PULL_UP)
		val &= ~(WOKOO_PIN_PULL_MASK << pin_data->pin_conf.pull_shift);
	if (pull_down == true && pin_data->pin_conf.pull_mode == WOKOO_PIN_PULL_DOWN)
		val &= ~(WOKOO_PIN_PULL_MASK << pin_data->pin_conf.pull_shift);
	writel(val, (base_address + pin_data->pin_conf.off_pull));
	spin_unlock_irqrestore(&pinctrl->lock, flags);

	dev_dbg(pctrldev->dev, "pin:%u set pullup:%d pulldown: %d\n",
		pin, pull_up, pull_down);
	return 0;
}

/*
 *  wokoo_pin_get_pull - get pull up/down
 *
 */
static void wokoo_pin_get_pull(struct pinctrl_dev *pctrldev,
				unsigned int pin, bool *pull_up,
				bool *pull_down)
{
	struct wokoo_pinctrl *pinctrl = pinctrl_dev_get_drvdata(pctrldev);
	struct wokoo_pin *pin_data = pctrldev->desc->pins[pin].drv_data;
	unsigned long flags;
	u32 val;

	spin_lock_irqsave(&pinctrl->lock, flags);
	val = readl(pinctrl->base_pin_pull + pin_data->pin_conf.off_pull);
	val = (val >> pin_data->pin_conf.pull_shift) & WOKOO_PIN_PULL_MASK;
	*pull_up = false;
	*pull_down = false;

	if (val == 0x1 ) {
		if (pin_data->pin_conf.pull_mode == WOKOO_PIN_PULL_UP)
			*pull_up = true;

		if (pin_data->pin_conf.pull_mode == WOKOO_PIN_PULL_DOWN)
			*pull_down = true;
	}
	spin_unlock_irqrestore(&pinctrl->lock, flags);
}

/*
 *  wokoo_pin_config_get - pin config get
 *
 */
static int wokoo_pin_config_get(struct pinctrl_dev *pctldev, unsigned int pin,
				unsigned long *config)
{
	struct wokoo_pin *pin_data = pctldev->desc->pins[pin].drv_data;
	enum pin_config_param param = pinconf_to_config_param(*config);
	bool pull_up, pull_down;
	int ret;

	if (pin_data->pin_conf.off_input == -1 || pin_data->pin_conf.off_pull == -1)
		return -ENOTSUPP;

	switch (param) {
	case PIN_CONFIG_BIAS_DISABLE:
		wokoo_pin_get_pull(pctldev, pin, &pull_up, &pull_down);
		if ((pull_up == false) && (pull_down == false))
			return 0;
		else
			return -EINVAL;

	case PIN_CONFIG_BIAS_PULL_UP:
		wokoo_pin_get_pull(pctldev, pin, &pull_up, &pull_down);
		if (pull_up)
			return 0;
		else
			return -EINVAL;

	case PIN_CONFIG_BIAS_PULL_DOWN:
		wokoo_pin_get_pull(pctldev, pin, &pull_up, &pull_down);
		if (pull_down)
			return 0;
		else
			return -EINVAL;

	case PIN_CONFIG_INPUT_ENABLE:
		ret = wokoo_pin_get_enable(pctldev, pin);
		if (ret)
			return 0;
		else
			return -EINVAL;

	default:
		return -ENOTSUPP;
	}
}

/*
 *  wokoo_pin_config_set - pin config set
 *
 */
static int wokoo_pin_config_set(struct pinctrl_dev *pctrldev, unsigned int pin,
				unsigned long *configs, unsigned int num_configs)
{
	struct wokoo_pin *pin_data = pctrldev->desc->pins[pin].drv_data;
	enum pin_config_param param;
	unsigned int i;
	u32 arg;
	int ret = -ENOTSUPP;

	if (pin_data->pin_conf.off_input == -1 || pin_data->pin_conf.off_pull == -1)
		return -ENOTSUPP;

	for (i = 0; i < num_configs; i++) {
		param = pinconf_to_config_param(configs[i]);
		arg = pinconf_to_config_argument(configs[i]);

		switch (param) {
		case PIN_CONFIG_BIAS_DISABLE:
			ret = wokoo_pin_set_pull(pctrldev, pin, false, false);
			if (ret < 0)
				goto out;
			break;

		case PIN_CONFIG_BIAS_PULL_UP:
			ret = wokoo_pin_set_pull(pctrldev, pin, true, false);
			if (ret < 0)
				goto out;
			break;

		case PIN_CONFIG_BIAS_PULL_DOWN:
			ret = wokoo_pin_set_pull(pctrldev, pin, false, true);
			if (ret < 0)
				goto out;
			break;

		case PIN_CONFIG_INPUT_ENABLE:
			ret = wokoo_pin_set_enable(pctrldev, pin, arg);
			if (ret < 0)
				goto out;
			break;

		default:
			dev_err(pctrldev->dev, "invalid configuration\n");
			return -ENOTSUPP;
		}
	}
out:
	return ret;
}

static const struct pinmux_ops wokoo_pinmux_ops = {
	.get_functions_count = wokoo_get_functions_count,
	.get_function_name = wokoo_get_function_name,
	.get_function_groups = wokoo_get_function_groups,
	.set_mux = wokoo_pinmux_enable,
};

static const struct pinconf_ops wokoo_pinconf_ops = {
	.is_generic = true,
	.pin_config_get = wokoo_pin_config_get,
	.pin_config_set = wokoo_pin_config_set,
};

static struct pinctrl_desc wokoo_pinctrl_desc = {
	.name = "wokoo-pinmux",
	.pctlops = &wokoo_pinctrl_ops,
	.pmxops = &wokoo_pinmux_ops,
	.confops = &wokoo_pinconf_ops,
};

static int wokoo_mux_log_init(struct wokoo_pinctrl *pinctrl)
{
	struct wokoo_mux_log *log;
	unsigned int i;

	pinctrl->mux_log = devm_kcalloc(pinctrl->dev, WOKOO_NUM_IOMUX,
					sizeof(struct wokoo_mux_log),
					GFP_KERNEL);
	if (!pinctrl->mux_log)
		return -ENOMEM;

	for (i = 0; i < WOKOO_NUM_IOMUX; i++)
		pinctrl->mux_log[i].is_configured = false;
	/* Group 0 uses not need iomux */
	log = &pinctrl->mux_log[0];
	log->mux.base = WOKOO_PIN_MUX_BASE0;
	log->mux.offset = 0;
	log->mux.shift = 0;
	log->mux.alt = 0;

	/* Init to default values */
	for (i = 1; i < WOKOO_NUM_IOMUX; i++) {
		log = &pinctrl->mux_log[i];
		log->mux.base = wokoo_pin_groups[2 * i - 1].mux.base;
		log->mux.offset = wokoo_pin_groups[2 * i - 1].mux.offset;
		log->mux.shift = wokoo_pin_groups[2 * i - 1].mux.shift;
		log->mux.alt = wokoo_pin_groups[2 * i - 1].mux.alt;
	}
	return 0;
}

static int wokoo_pinmux_probe(struct platform_device *pdev)
{
	struct wokoo_pinctrl *pinctrl;
	struct resource *res;
	int i, ret;
	struct pinctrl_pin_desc *pins;
	unsigned int num_pins = ARRAY_SIZE(wokoo_pins);

	pinctrl = devm_kzalloc(&pdev->dev, sizeof(*pinctrl), GFP_KERNEL);
	if (!pinctrl)
		return -ENOMEM;

	pinctrl->dev = &pdev->dev;
	platform_set_drvdata(pdev, pinctrl);
	spin_lock_init(&pinctrl->lock);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	pinctrl->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(pinctrl->base))
		return PTR_ERR(pinctrl->base);

	pinctrl->pclk = devm_clk_get(&pdev->dev, "pclk");
	if (IS_ERR(pinctrl->pclk)) {
		ret = PTR_ERR(pinctrl->pclk);
		dev_err(&pdev->dev, "failed to get ipg clk: %d\n", ret);
		return ret;
	}

	/* For register access, we only need to enable the pclk clock. */
	ret = clk_prepare_enable(pinctrl->pclk);
	if (ret) {
		dev_err(&pdev->dev, "failed to enable pclk clk: %d\n", ret);
		return ret;
	}

	pinctrl->base_pin_input = pinctrl->base + WOKOO_PIN_INPUT_BASE;
	pinctrl->base_pin_pull  = pinctrl->base + WOKOO_PIN_PULL_BASE;

	ret = wokoo_mux_log_init(pinctrl);
	if (ret) {
		dev_err(&pdev->dev, "unable to initialize IOMUX log\n");
		clk_disable_unprepare(pinctrl->pclk);
		return ret;
	}

	pins = devm_kcalloc(&pdev->dev, num_pins, sizeof(*pins), GFP_KERNEL);
	if (!pins) {
		clk_disable_unprepare(pinctrl->pclk);
		return -ENOMEM;
	}

	for (i = 0; i < num_pins; i++) {
		pins[i].number = wokoo_pins[i].pin;
		pins[i].name = wokoo_pins[i].name;
		pins[i].drv_data = &wokoo_pins[i];
	}

	pinctrl->groups = wokoo_pin_groups;
	pinctrl->num_groups = ARRAY_SIZE(wokoo_pin_groups);
	pinctrl->functions = wokoo_pin_functions;
	pinctrl->num_functions = ARRAY_SIZE(wokoo_pin_functions);
	wokoo_pinctrl_desc.pins = pins;
	wokoo_pinctrl_desc.npins = num_pins;

	pinctrl->pctl = pinctrl_register(&wokoo_pinctrl_desc, &pdev->dev,
			pinctrl);
	if (IS_ERR(pinctrl->pctl)) {
		dev_err(&pdev->dev, "unable to register IOMUX pinctrl\n");
		clk_disable_unprepare(pinctrl->pclk);
		return PTR_ERR(pinctrl->pctl);
	}

	return 0;
}

static const struct of_device_id wokoo_pinmux_of_match[] = {
	{.compatible = "wokoo,wokoo-pinmux"},
	{ }
};

static struct platform_driver wokoo_pinmux_driver = {
	.driver = {
		.name = "wokoo-pinmux",
		.of_match_table = wokoo_pinmux_of_match,
	},
	.probe = wokoo_pinmux_probe,
};

static int __init wokoo_pinmux_init(void)
{
	return platform_driver_register(&wokoo_pinmux_driver);
}
arch_initcall(wokoo_pinmux_init);

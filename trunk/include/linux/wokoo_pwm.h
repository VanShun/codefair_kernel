
#ifndef _LINUX_WOKOO_PWM_H_
#define _LINUX_WOKOO_PWM_H_

#include <linux/clk.h>
#include <linux/regmap.h>

#define WOKOO_PWM_EN		0x00
#define WOKOO_PWM_UP		0x04
#define WOKOO_PWM_RST		0x08
#define WOKOO_PWM_P		    0x0C
#define WOKOO_PWM_OCPY		0x10
#define WOKOO_PWM1_EN		0x40
#define WOKOO_PWM1_UP		0x44
#define WOKOO_PWM1_RST		0x48
#define WOKOO_PWM1_P		0x4C
#define WOKOO_PWM1_OCPY		0x50
#define WOKOO_PWM2_EN		0x80
#define WOKOO_PWM2_UP		0x84
#define WOKOO_PWM2_RST		0x88
#define WOKOO_PWM2_P		0x8C
#define WOKOO_PWM2_OCPY		0x90
#define WOKOO_PWM3_EN		0xC0
#define WOKOO_PWM3_UP		0xC4
#define WOKOO_PWM3_RST		0xC8
#define WOKOO_PWM3_P		0xCC
#define WOKOO_PWM3_OCPY		0xD0

// struct wokoo_pwmtimer {
	// struct clk *clk;
	// struct regmap *regmap;
// };

#endif
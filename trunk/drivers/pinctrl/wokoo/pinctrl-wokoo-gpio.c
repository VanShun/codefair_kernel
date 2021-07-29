/*
 * Copyright (c) 2020 Codefair Semiconductor Technology Co., Ltd.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

/*
 * This file contains the WOKOO GPIO driver thatsupports the chip0
 * GPIO controller. Pins from the chip0  GPIO can be individually 
 * muxed to GPIO function, through the interaction with the WOKOO 
 * IOMUX controller.
 */

#include <linux/gpio/driver.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/kernel.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/slab.h>
#include <linux/clk.h>

#include "../pinctrl-utils.h"




#define WOKOO_GPIO_DATA_IN			0x40
#define WOKOO_GPIO_DATA_OUT			0x00
#define WOKOO_GPIO_OUT_EN			0x20
#define WOKOO_GPIO_INT_CTRL			0x100
#define WOKOO_GPIO_DEBOUNCE			0x180
#define WOKOO_GPIO_INT_MASK			0x200
#define WOKOO_INT_STATUS			0x220
#define WOKOO_GPIO_EVENT			0x1b0
#define WOKOO_GPIO_EVENT_INT_MASK	0x200

/*
 * wokoo GPIO core
 *
 * @dev: pointer to device
 * @base: I/O register base for wokoo GPIO controller
 * @gc: GPIO chip
 * @pctl: pointer to pinctrl_dev
 * @pctldesc: pinctrl descriptor
 * @irq_domain: pointer to irq domain
 * @clk_pclk: clock gate for gpio
 * @lock: lock to protect access to I/O registers
 */
struct wokoo_gpio {
	struct device *dev;
	void __iomem *base;
	struct gpio_chip gc;
	struct pinctrl_dev *pctl;
	struct pinctrl_desc pctldesc;
	struct irq_domain *irq_domain;
	int			pioc_virq;
	struct clk *clk_pclk;
	raw_spinlock_t lock;
};

/*
 * Mapping from PINCONF pins to GPIO pins is 1-to-1
 */
static inline unsigned wokoo_pin_to_gpio(unsigned pin)
{
	return pin;
}

/*
 *  wokoo_set_bit - set or clear one bit (corresponding to the GPIO pin) in a
 *  wokoo GPIO register
 *
 *  @wokoo_gpio: wokoo GPIO device
 *  @reg: register offset
 *  @gpio: GPIO pin
 *  @set: set or clear
 */
static inline void wokoo_set_bit(struct wokoo_gpio *chip,
					unsigned int reg, unsigned gpio, bool set)
{
	u32 val;

	val = readl(chip->base + reg);
	if (set)
		val |= BIT(gpio);
	else
		val &= ~BIT(gpio);

	writel(val, chip->base + reg);
}

/*
 *  wokoo_set_bit - set or clear one bit (corresponding to the GPIO pin) in a
 *  wokoo GPIO register
 *
 *  @wokoo_gpio: wokoo GPIO device
 *  @reg: register offset
 *  @gpio: GPIO pin
 *  @set: set or clear
 */
static inline void wokoo_set_bit_w_protect(struct wokoo_gpio *chip,
					unsigned int reg, unsigned gpio, bool set, unsigned int protect)
{
	u32 val;

	val = readl(chip->base + reg);
	if (set)
		val |= BIT(gpio);
	else
		val &= ~BIT(gpio);

	val |= BIT(protect);
	writel(val, chip->base + reg);
}

/*
 *  wokoo_set_bits - set or clear one bit (corresponding to the GPIO pin) in a
 *  wokoo GPIO register
 *
 *  @wokoo_gpio: wokoo GPIO device
 *  @reg: register offset
 *  @bits: bits value
 *  @set: set or clear
 */
static void wokoo_set_bits(struct wokoo_gpio *chip, unsigned int reg, unsigned int bits, bool set)
{
	u32 val;

	val = readl(chip->base + reg);
	if (set)
		val |= bits;
	else
		val &= ~bits;

	writel(val, chip->base + reg);
}

/*
 *  wokoo_get_bit - get one bit (corresponding to the GPIO pin) in a
 *  wokoo GPIO register
 */
static inline bool wokoo_get_bit(struct wokoo_gpio *chip,
					unsigned int reg, unsigned gpio)
{
	return !!(readl(chip->base + reg) & BIT(gpio));
}

/*
 *  wokoo_gpio_irq_clear - clear gpio irq
 *  wokoo GPIO register
 */
static void wokoo_gpio_irq_clear(struct wokoo_gpio *chip, u32 gpio)
{
	u32 off_irq_reg;
	u32 gpio_irq_bit;

	off_irq_reg  = gpio / 32;
	gpio_irq_bit = gpio % 32;
	off_irq_reg  = off_irq_reg  << 2;
	wokoo_set_bit(chip, WOKOO_GPIO_EVENT + off_irq_reg, gpio_irq_bit, true);
}

/*
 *  wokoo_gpio_irq_handler - gpio irq handler
 */
static irqreturn_t wokoo_gpio_irq_handler(int irq, void *data)
{
	struct wokoo_gpio *chip = (struct wokoo_gpio *)data;
	u32 i, j, handle = 0;
	u32 reg, pin, virq;

	for (i = 0; i < 4; i++) {
		/* Read irq status(x) */
		reg = readl(chip->base + WOKOO_INT_STATUS + (i << 2));
		for (j = 0; j < 32; j++) {
			/* get gpio irq status */
			if (reg & (1 << j)) {
				/* get gpio */
				pin  = i * 32 + j;
				virq = irq_linear_revmap(chip->irq_domain, pin);
				if (!virq)
					continue;
				generic_handle_irq(virq);

				/* clear gpio irq */
				wokoo_gpio_irq_clear(chip, pin);
				handle = 1;
			}
		}
	}

	return  handle ? IRQ_HANDLED : IRQ_NONE;
}

/*
 *  wokoo_gpio_irq_ack - clear gpio irq
 */
static void wokoo_gpio_irq_ack(struct irq_data *d)
{
	struct wokoo_gpio *chip = irq_data_get_irq_chip_data(d);
	unsigned gpio = d->hwirq;
	u32 trigger_type;

	trigger_type = irq_get_trigger_type(d->irq);
	if (trigger_type & (IRQ_TYPE_EDGE_FALLING | IRQ_TYPE_EDGE_RISING)) {
		wokoo_gpio_irq_clear(chip, gpio);
	}
}

/*
 *  wokoo_gpio_irq_set_mask - mask/unmask a GPIO interrupt
 *
 *  @d: IRQ chip data
 *  @unmask: mask/unmask GPIO interrupt
 */
static void wokoo_gpio_irq_set_mask(struct irq_data *d, bool unmask)
{
	struct wokoo_gpio *chip = irq_data_get_irq_chip_data(d);
	unsigned gpio = d->hwirq;
	u32 trigger_type;
	unsigned int  off_irq_reg;
	unsigned int  gpio_irq_bit;
	unsigned int  w_protect_bit;

	/* Mask interupt */
	off_irq_reg  = gpio / 16;
	gpio_irq_bit = gpio % 16;
	off_irq_reg  = off_irq_reg  << 2;
	trigger_type = irq_get_trigger_type(d->irq);
	wokoo_set_bit_w_protect(chip, WOKOO_GPIO_EVENT_INT_MASK + off_irq_reg,
							gpio_irq_bit, !unmask, gpio_irq_bit + 16);

	/* Enable interupt */
	off_irq_reg   = gpio / 4;
	gpio_irq_bit  = gpio % 4;
	w_protect_bit = gpio_irq_bit + 16;
	gpio_irq_bit  = gpio_irq_bit << 2;
	off_irq_reg   = off_irq_reg  << 2;
	wokoo_set_bits(chip, WOKOO_GPIO_INT_CTRL + off_irq_reg,
					(1 << gpio_irq_bit) | (1 << w_protect_bit), unmask);
}

/*
 *  wokoo_gpio_irq_mask - mask a GPIO interrupt
 *
 *  @d: IRQ chip dat
 */
static void wokoo_gpio_irq_mask(struct irq_data *d)
{
	struct wokoo_gpio *chip = irq_data_get_irq_chip_data(d);
	unsigned long flags;

	raw_spin_lock_irqsave(&chip->lock, flags);
	wokoo_gpio_irq_set_mask(d, false);
	raw_spin_unlock_irqrestore(&chip->lock, flags);
}

/*
 *  wokoo_gpio_irq_unmask - unmask a GPIO interrupt
 *
 *  @d: IRQ chip dat
 */
static void wokoo_gpio_irq_unmask(struct irq_data *d)
{
	struct wokoo_gpio *chip = irq_data_get_irq_chip_data(d);
	unsigned long flags;

	raw_spin_lock_irqsave(&chip->lock, flags);
	wokoo_gpio_irq_set_mask(d, true);
	raw_spin_unlock_irqrestore(&chip->lock, flags);
}

/*
 *  wokoo_gpio_irq_set_type - set interupt type
 *
 *  @d: IRQ chip dat
 *  @type: type
 */
static int wokoo_gpio_irq_set_type(struct irq_data *d, unsigned int type)
{
	struct wokoo_gpio *chip = irq_data_get_irq_chip_data(d);
	unsigned gpio = d->hwirq;
	unsigned long flags;
	unsigned int  off_irq_reg;
	unsigned int  gpio_irq_bit;
	unsigned int  w_protect_bit;

	off_irq_reg  = gpio / 4;
	gpio_irq_bit = gpio % 4;
	off_irq_reg <<= 2;
	raw_spin_lock_irqsave(&chip->lock, flags);

	switch (type & IRQ_TYPE_SENSE_MASK) {
	case IRQ_TYPE_EDGE_RISING:
		type = 2;
		break;

	case IRQ_TYPE_EDGE_FALLING:
		type = 3;
		break;

	case IRQ_TYPE_LEVEL_HIGH:
		type = 0;
		break;

	case IRQ_TYPE_LEVEL_LOW:
		type = 1;
		break;

	case IRQ_TYPE_EDGE_BOTH:
		type = 4;
		break;

	default:
		dev_err(chip->dev, "invalid GPIO IRQ type 0x%x\n",
			type);
		raw_spin_unlock_irqrestore(&chip->lock, flags);
		return -EINVAL;
	}

	w_protect_bit = gpio_irq_bit + 16;
	/* MODE[0~3] * 4 group, BIT(16~19) is write protec bit */
	gpio_irq_bit  = gpio_irq_bit << 2;
	wokoo_set_bits(chip, WOKOO_GPIO_INT_CTRL + off_irq_reg,
					(type << (gpio_irq_bit + 1)) | (1 << w_protect_bit), true);
	raw_spin_unlock_irqrestore(&chip->lock, flags);

	dev_dbg(chip->dev, "gpio:%u level_low:%s falling:%s\n", gpio,
		type == 1 ? "true" : "false", type == 3 ? "true" : "false");
	return 0;
}

/*
 *  wokoo_gpio_irq_set_wake - set interupt can wake
 *
 *  @d: IRQ chip dat
 *  @type: type
 */
static int wokoo_gpio_irq_set_wake(struct irq_data *d, unsigned state)
{
	struct wokoo_gpio *chip = irq_data_get_irq_chip_data(d);

	irq_set_irq_wake(chip->pioc_virq, state);
	return 0;
}

static struct irq_chip wokoo_gpio_irq_chip = {
	.name = "gpio0",
	.irq_enable = wokoo_gpio_irq_unmask,
	.irq_disable = wokoo_gpio_irq_mask,
	.irq_ack = wokoo_gpio_irq_ack,
	.irq_mask = wokoo_gpio_irq_mask,
	.irq_unmask = wokoo_gpio_irq_unmask,
	.irq_set_type = wokoo_gpio_irq_set_type,
	.irq_set_wake = wokoo_gpio_irq_set_wake,
};

/*
 *  wokoo_gpio_direction_input - set direction input
 *
 *  @gc: gpio chip
 *  @gpio: gpio
 */
static int wokoo_gpio_direction_input(struct gpio_chip *gc, unsigned gpio)
{
	struct wokoo_gpio *chip = gpiochip_get_data(gc);
	unsigned long flags;
	unsigned int  off_reg_out_en;
	unsigned int  gpio_out_en_bit;

	off_reg_out_en   = gpio / 16;
	gpio_out_en_bit  = gpio % 16;
	off_reg_out_en <<= 2;
	raw_spin_lock_irqsave(&chip->lock, flags);
	wokoo_set_bit_w_protect(chip, WOKOO_GPIO_OUT_EN + off_reg_out_en,
							gpio_out_en_bit, false, gpio_out_en_bit + 16);
	raw_spin_unlock_irqrestore(&chip->lock, flags);

	dev_dbg(chip->dev, "gpio:%u set input\n", gpio);
	return 0;
}

/*
 *  wokoo_gpio_direction_output - set direction output
 *
 *  @gc: gpio chip
 *  @gpio: gpio
 *  @val: value of output
 */
static int wokoo_gpio_direction_output(struct gpio_chip *gc, unsigned gpio,
				     int val)
{
	struct wokoo_gpio *chip = gpiochip_get_data(gc);
	unsigned long flags;
	unsigned int  off_reg_out_en;
	unsigned int  off_reg_out_data;
	unsigned int  gpio_out_en_bit;
	unsigned int  gpio_out_data_bit;


	raw_spin_lock_irqsave(&chip->lock, flags);
	off_reg_out_en  = gpio / 16;
	gpio_out_en_bit = gpio % 16;
	off_reg_out_data  = gpio / 32;
	gpio_out_data_bit = gpio % 32;
	off_reg_out_en   <<= 2;
	off_reg_out_data <<= 2;
	wokoo_set_bit_w_protect(chip, WOKOO_GPIO_OUT_EN + off_reg_out_en,
							gpio_out_en_bit, true, gpio_out_en_bit + 16);
	wokoo_set_bit_w_protect(chip, WOKOO_GPIO_DATA_OUT + off_reg_out_data,
							gpio_out_data_bit, !!(val), gpio_out_data_bit + 16);
	raw_spin_unlock_irqrestore(&chip->lock, flags);

	dev_dbg(chip->dev, "gpio:%u set output, value:%d\n", gpio, val);
	return 0;
}

/*
 *  wokoo_gpio_set - set gpio value
 *
 *  @gc: gpio chip
 *  @gpio: gpio
 *  @val: value of output
 */
static void wokoo_gpio_set(struct gpio_chip *gc, unsigned gpio, int val)
{
	struct wokoo_gpio *chip = gpiochip_get_data(gc);
	unsigned long flags;
	unsigned int  off_reg_out_data;
	unsigned int  gpio_out_data_bit;

	/* x = n / 16��y = n % 16 */
	off_reg_out_data  = gpio / 16;
	gpio_out_data_bit = gpio % 16;
	off_reg_out_data <<= 2;
	raw_spin_lock_irqsave(&chip->lock, flags);
	wokoo_set_bit_w_protect(chip, WOKOO_GPIO_DATA_OUT + off_reg_out_data,
							gpio_out_data_bit, !!(val), gpio_out_data_bit + 16);
	raw_spin_unlock_irqrestore(&chip->lock, flags);

	dev_dbg(chip->dev, "gpio:%u set, value:%d\n", gpio, val);
}

/*
 *  wokoo_gpio_get - get gpio value
 *
 *  @gc: gpio chip
 *  @gpio: gpio
 *  @return: value of gpio input
 */
static int wokoo_gpio_get(struct gpio_chip *gc, unsigned gpio)
{
	struct wokoo_gpio *chip = gpiochip_get_data(gc);
	unsigned int  off_reg_in_data;
	unsigned int  gpio_in_data_bit;

	/* x = n / 32��y = n % 32 */
	off_reg_in_data  = gpio / 32;
	gpio_in_data_bit = gpio % 32;
	off_reg_in_data <<= 2;

	return !!(readl(chip->base + WOKOO_GPIO_DATA_IN + off_reg_in_data) & BIT(gpio_in_data_bit));
}

static int wokoo_gpio_to_irq(struct gpio_chip *gc, unsigned offset)
{
	struct wokoo_gpio *chip = gpiochip_get_data(gc);

	return irq_linear_revmap(chip->irq_domain, offset);
}

static int wokoo_get_groups_count(struct pinctrl_dev *pctldev)
{
	return 1;
}

/*
 * Only one group: "gpio_grp", since this local pinctrl device only performs
 * GPIO specific PINCONF configurations
 */
static const char *wokoo_get_group_name(struct pinctrl_dev *pctldev,
						unsigned selector)
{
	return "gpio_grp";
}

static const struct pinctrl_ops wokoo_pctrl_ops = {
	.get_groups_count = wokoo_get_groups_count,
	.get_group_name = wokoo_get_group_name,
	.dt_node_to_map = pinconf_generic_dt_node_to_map_pin,
	.dt_free_map = pinctrl_utils_free_map,
};

static int wokoo_pin_config_group_get(struct pinctrl_dev *pctldev,
					unsigned selector, unsigned long *config)
{
	return 0;
}

static int wokoo_pin_config_group_set(struct pinctrl_dev *pctldev,
					unsigned selector,
					unsigned long *configs, unsigned num_configs)
{
	return 0;
}

static int wokoo_pin_config_get(struct pinctrl_dev *pctldev, unsigned pin,
					unsigned long *config)
{
	return -ENOTSUPP;
}

/*
 *  wokoo_gpio_set_debounce - gpio set input debounce
 *
 */
static int wokoo_gpio_set_debounce(struct gpio_chip *gc, unsigned gpio,
						unsigned debounce)
{
	struct wokoo_gpio *chip = gpiochip_get_data(gc);
	unsigned long flags;
	unsigned int  gpio_reg;
	unsigned int  gpio_bit;

	gpio_reg = gpio / 16;
	gpio_bit = gpio % 16;
	gpio_reg <<= 2;

	raw_spin_lock_irqsave(&chip->lock, flags);
	wokoo_set_bit_w_protect(chip, WOKOO_GPIO_DEBOUNCE + gpio_reg, gpio_bit, !!debounce, gpio_bit + 16);
	raw_spin_unlock_irqrestore(&chip->lock, flags);

	dev_dbg(chip->dev, "gpio:%u set debounce:%d reg = %x\n",
			gpio, !!debounce, readl(chip->base + WOKOO_GPIO_DEBOUNCE + gpio_reg));
	return 0;
}

static int wokoo_pin_config_set(struct pinctrl_dev *pctldev, unsigned pin,
					unsigned long *configs, unsigned num_configs)
{
	return -ENOTSUPP;
}

static const struct pinconf_ops wokoo_pconf_ops = {
	.is_generic = true,
	.pin_config_get = wokoo_pin_config_get,
	.pin_config_set = wokoo_pin_config_set,
	.pin_config_group_get = wokoo_pin_config_group_get,
	.pin_config_group_set = wokoo_pin_config_group_set,
};

/*
 * WOKOO GPIO controller supports some PINCONF related configurations such as
 * pull up, pull down, slew and drive strength, when the pin is configured
 * to GPIO.
 *
 * Here a local pinctrl device is created with simple 1-to-1 pin mapping to the
 * local GPIO pins
 */
static int wokoo_gpio_register_pinconf(struct wokoo_gpio *chip)
{
	struct pinctrl_desc *pctldesc = &chip->pctldesc;
	struct pinctrl_pin_desc *pins;
	struct gpio_chip *gc = &chip->gc;
	int i;

	pins = devm_kcalloc(chip->dev, gc->ngpio, sizeof(*pins), GFP_KERNEL);
	if (!pins)
		return -ENOMEM;
	for (i = 0; i < gc->ngpio; i++) {
		pins[i].number = i;
		pins[i].name = devm_kasprintf(chip->dev, GFP_KERNEL,
							"gpio-%d", i);
		if (!pins[i].name)
			return -ENOMEM;
	}
	pctldesc->name = dev_name(chip->dev);
	pctldesc->pctlops = &wokoo_pctrl_ops;
	pctldesc->pins = pins;
	pctldesc->npins = gc->ngpio;
	pctldesc->confops = &wokoo_pconf_ops;

	chip->pctl = devm_pinctrl_register(chip->dev, pctldesc, chip);
	if (IS_ERR(chip->pctl)) {
		dev_err(chip->dev, "unable to register pinctrl device\n");
		return PTR_ERR(chip->pctl);
	}

	return 0;
}

static int wokoo_gpio_set_config(struct gpio_chip *chip, unsigned gpio,
					unsigned long config)
{
	u32 debounce;

	if (pinconf_to_config_param(config) != PIN_CONFIG_INPUT_DEBOUNCE)
		return -ENOTSUPP;

	debounce = pinconf_to_config_argument(config);
	return wokoo_gpio_set_debounce(chip, gpio, debounce);
}

static const struct of_device_id wokoo_gpio_of_match[] = {
	{.compatible = "wokoo,wokoo-gpio0",},
	{}
};

static int wokoo_gpio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct resource *res;
	struct wokoo_gpio *chip;
	struct gpio_chip *gc;
	u32 val, count;
	int irqno, ret;

	if (of_property_read_u32(pdev->dev.of_node, "ngpios", &val)) {
		dev_err(&pdev->dev, "Missing ngpios OF property\n");
		return -ENODEV;
	}

	chip = devm_kzalloc(dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->dev = dev;
	platform_set_drvdata(pdev, chip);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	chip->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(chip->base)) {
		dev_err(dev, "unable to map I/O memory\n");
		return PTR_ERR(chip->base);
	}

	raw_spin_lock_init(&chip->lock);
	gc = &chip->gc;
	gc->base = -1;
	gc->can_sleep = false;
	gc->ngpio = val;
	gc->label = dev_name(dev);
	gc->parent = dev;
	gc->of_node = dev->of_node;
	gc->request = gpiochip_generic_request;
	gc->free = gpiochip_generic_free;
	gc->direction_input = wokoo_gpio_direction_input;
	gc->direction_output = wokoo_gpio_direction_output;
	gc->set = wokoo_gpio_set;
	gc->get = wokoo_gpio_get;
	gc->to_irq = wokoo_gpio_to_irq;
	gc->set_config = wokoo_gpio_set_config;

	chip->clk_pclk = devm_clk_get(&pdev->dev, "pclk");
	if (IS_ERR(chip->clk_pclk)) {
		ret = PTR_ERR(chip->clk_pclk);
		dev_err(&pdev->dev, "failed to get ipg clk: %d\n", ret);
		return ret;
	}

	/* For register access, we only need to enable the pclk clock. */
	ret = clk_prepare_enable(chip->clk_pclk);
	if (ret) {
		dev_err(&pdev->dev, "failed to enable pclk clk: %d\n", ret);
		return ret;
	}

	/* optional GPIO interrupt support */
	irqno = platform_get_irq(pdev, 0);
	if (irqno > 0) {
		/* Create irq domain so that each pin can be assigned an IRQ.*/
		chip->irq_domain = irq_domain_add_linear(gc->of_node, gc->ngpio,
							 &irq_domain_simple_ops,
							 chip);
		if (!chip->irq_domain) {
			dev_err(&pdev->dev, "Couldn't allocate IRQ domain\n");
			return -ENXIO;
		}

		/* Map each gpio to an IRQ and set the handler for gpiolib. */
		for (count = 0; count < gc->ngpio; count++) {
			int irq = irq_create_mapping(chip->irq_domain, count);

			irq_set_chip_and_handler(irq, &wokoo_gpio_irq_chip,
						 handle_simple_irq);
			irq_set_chip_data(irq, chip);
		}

		/* Install ISR for this GPIO controller. */
		ret = devm_request_irq(&pdev->dev, irqno, wokoo_gpio_irq_handler,
						IRQF_SHARED, "gpio0", chip);
		if (ret) {
			dev_err(&pdev->dev, "Unable to request IRQ%d: %d\n",
				irqno, ret);
			goto err_rm_gpiochip;
		}
	}

	chip->pioc_virq = irqno;

	ret = gpiochip_add_data(gc, chip);
	if (ret < 0) {
		dev_err(dev, "unable to add GPIO chip\n");
		return ret;
	}

	ret = wokoo_gpio_register_pinconf(chip);
	if (ret) {
		dev_err(dev, "unable to register pinconf\n");
		goto err_rm_gpiochip;
	}

	return 0;

err_rm_gpiochip:
	clk_disable_unprepare(chip->clk_pclk);
	gpiochip_remove(gc);

	return ret;
}

static struct platform_driver wokoo_gpio_driver = {
	.driver = {
		.name = "wokoo-gpio0",
		.of_match_table = wokoo_gpio_of_match,
	},
	.probe = wokoo_gpio_probe,
};

static int __init wokoo_gpio_init(void)
{
	return platform_driver_register(&wokoo_gpio_driver);
}
arch_initcall_sync(wokoo_gpio_init);

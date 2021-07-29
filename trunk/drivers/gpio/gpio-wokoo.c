/* SPDX-License-Identifier: GPL-2.0
 *
 * Copyright (c) 2020 Codefair Semiconductor Technology Co., Ltd.
 *		http://www.codefairsemi.com 
 *
 */


#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/rfkill.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/regulator/consumer.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <asm/irq.h>
#include <linux/suspend.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <dt-bindings/gpio/gpio.h>
#include <linux/skbuff.h>
#include <linux/fb.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>
#include <linux/workqueue.h>
#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#endif

#define DOC_DECTECT_REPEAT_TIME     (20)

static int docirqno  = 0;
static int doc_usb_det_gpio = 0;
static int doc_usb_oel_gpio = 0;
static int doc_usb_pwr_gpio = 0;
static int doc_usb_pwr_en   = 0;
static int doc_usb_det_en   = 0;
static int doc_usb_oel_en   = 0;
static int doc_is_in = 0;
static spinlock_t         doc_irqlock;
static struct work_struct workDocking;

static void docking_schedule_work_func(struct work_struct *work)
{
	//unsigned long flags;
	int   i         = 0;
	int   doc_in_count = 0;


	while ( 1 ) {
		//printk(KERN_ERR "%s: enter\n", __func__);
		for (i = 0; i < DOC_DECTECT_REPEAT_TIME; i++) {
			if ( gpio_get_value(doc_usb_det_gpio) == doc_usb_det_en )
			{
				doc_in_count++;
			}

			msleep(20);
		}

		//printk(KERN_ERR "%s: doc_in_count = %d\n", __func__, doc_in_count);
		//spin_lock_irqsave(&doc_irqlock, flags);
		if ( (doc_in_count > DOC_DECTECT_REPEAT_TIME / 3)
		  && (gpio_get_value(doc_usb_det_gpio) == doc_usb_det_en) )
			doc_in_count = 1;
		else
			doc_in_count = 0;
		//spin_unlock_irqrestore(&doc_irqlock, flags);

		if ( doc_in_count == 1 && doc_is_in != 1 ) {
			printk(KERN_ERR "%s: det a doc\n", __func__);
			gpio_direction_output(doc_usb_oel_gpio, doc_usb_oel_en);
			gpio_set_value(doc_usb_oel_gpio, doc_usb_oel_en);

	#if 0
			/* Rest usb hub for maybe hub is disable */
			gpio_direction_output(usb_hut_rst_gpio, usb_hut_rst_en);
			gpio_set_value(usb_hut_rst_gpio, usb_hut_rst_en);
			mdelay(10);
			gpio_direction_output(usb_hut_rst_gpio, !usb_hut_rst_en);
			gpio_set_value(usb_hut_rst_gpio, !usb_hut_rst_en);
	#endif

			/* Enable doc power */
			gpio_direction_output(doc_usb_pwr_gpio, doc_usb_pwr_en);
			gpio_set_value(doc_usb_pwr_gpio, doc_usb_pwr_en);
			doc_is_in = 1;
		} else if ( doc_in_count == 0 && doc_is_in != 0 ) {
			printk(KERN_ERR "%s: remove a doc\n", __func__);
			gpio_direction_output(doc_usb_oel_gpio, !doc_usb_oel_en);
			gpio_set_value(doc_usb_oel_gpio, !doc_usb_oel_en);

			gpio_direction_output(doc_usb_pwr_gpio, !doc_usb_pwr_en);
			gpio_set_value(doc_usb_pwr_gpio, !doc_usb_pwr_en);
			doc_is_in = 0;
		}

		msleep(2000);
	}
}

static irqreturn_t irqDocHander(int irq, void *dev_id)
{
	//printk(KERN_ERR "%s: Get a irq = %d\n", __func__, irq);
	//schedule_work(&workDocking);
	return IRQ_HANDLED;
}

static int wokoo_gpios_probe(struct platform_device *pdev)
{
	int                 ret = -1;
	int                 gpio;
	enum of_gpio_flags  flags;
	struct device_node *gpio_node = pdev->dev.of_node;

	/* Docking gpio */
	gpio = of_get_named_gpio_flags(gpio_node, "doc_5ven", 0, &flags);
	if ( gpio_is_valid(gpio) ) {
		//printk(KERN_ERR "doc_5ven = %d\n", gpio);
		ret = gpio_request(gpio, "doc_5ven");
		//printk(KERN_ERR "doc_5ven request = %d\n", ret);
		doc_usb_pwr_gpio = gpio;
		doc_usb_pwr_en   = (flags == GPIO_ACTIVE_HIGH)? 1:0;
		//printk(KERN_ERR "%s: get property: doc_5ven = %d, flags = %d.\n", __func__, gpio, flags);
	}

	gpio = of_get_named_gpio_flags(gpio_node, "doc_usboel", 0, &flags);
	if ( gpio_is_valid(gpio) ) {
		//printk(KERN_ERR "doc_usboel = %d\n", gpio);
		ret = gpio_request(gpio, "doc_usboel");
		//printk(KERN_ERR "doc_usboel request = %d\n", ret);
		doc_usb_oel_gpio = gpio;
		doc_usb_oel_en   = (flags == GPIO_ACTIVE_HIGH) ? 1:0;
		//printk(KERN_ERR "%s: get property: doc_usboel = %d, flags = %d.\n", __func__, gpio, flags);
	}

	gpio = of_get_named_gpio_flags(gpio_node, "doc_det", 0, &flags);
	if ( gpio_is_valid(gpio) ) {
		printk(KERN_ERR "doc_det = %d\n", gpio);
		ret = gpio_request(gpio, "doc_det");
		//printk(KERN_ERR "doc_det request = %d\n", ret);
		if (ret == 0) {
			docirqno = gpio_to_irq(gpio);
			ret   = request_irq(docirqno, irqDocHander, IRQF_TRIGGER_RISING, "Doc interrupt", NULL);
			if ( ret != 0 ) {
				printk(KERN_ERR "doc_det irq request fail\n");
			}
		}

		doc_usb_det_gpio  = gpio;
		doc_usb_det_en    = (flags == GPIO_ACTIVE_HIGH)? 1:0;

		/* Is doc in when system uping */
		if ( gpio_get_value(doc_usb_det_gpio) == doc_usb_det_en ) {
			//printk(KERN_ERR "%s: doc is on\n", __func__);
			gpio_direction_output(doc_usb_oel_gpio, doc_usb_oel_en);
			gpio_set_value(doc_usb_oel_gpio, doc_usb_oel_en);

			gpio_direction_output(doc_usb_pwr_gpio, doc_usb_pwr_en);
			gpio_set_value(doc_usb_pwr_gpio, doc_usb_pwr_en);
			doc_is_in = 1;
		}

		/* Enable irq wake */
		ret = enable_irq_wake(docirqno);
		if(ret) {
			printk(KERN_ERR "failed to set mcu gpio irq %d wake up ret = %d!\n",
			docirqno, ret);
		}

		spin_lock_init(&doc_irqlock);
		INIT_WORK(&workDocking, docking_schedule_work_func);
		schedule_work(&workDocking);
		//printk(KERN_ERR "%s: get property: doc_5ven = %d, flags = %d.\n", __func__, gpio, flags);
	}

	return 0;
}

static int wokoo_gpios_remove(struct platform_device *pdev)
{
	if ( doc_usb_oel_gpio ) {
		gpio_free(doc_usb_oel_gpio);
		doc_usb_oel_gpio = 0;
	}

	if ( doc_usb_det_gpio ) {
		gpio_free(doc_usb_det_gpio);
		doc_usb_det_gpio= 0;
	}

	if ( doc_usb_pwr_gpio ) {
		gpio_free(doc_usb_pwr_gpio);
		doc_usb_pwr_gpio= 0;
	}

	if ( docirqno ) {
		free_irq(docirqno, NULL);
		docirqno = 0;
	}

	return 0;
}

#ifdef CONFIG_PM
static int wokoo_gpios_suspend(struct platform_device *pdev, pm_message_t state)
{
#if 0
	printk(KERN_ERR "%s: suspend\n", __func__);
	if ( gpio_get_value(doc_usb_det_gpio) == doc_usb_det_en ) {
		printk(KERN_ERR "%s: remove a doc\n", __func__);
		gpio_direction_output(doc_usb_oel_gpio, !doc_usb_oel_en);
		gpio_set_value(doc_usb_oel_gpio, !doc_usb_oel_en);

		gpio_direction_output(doc_usb_pwr_gpio, !doc_usb_pwr_en);
		gpio_set_value(doc_usb_pwr_gpio, !doc_usb_pwr_en);
	}
#endif

	
	return 0;
}

static int wokoo_gpios_resume(struct platform_device *pdev)
{
#if 0
	printk(KERN_ERR "%s: resume\n", __func__);
	if ( gpio_get_value(doc_usb_det_gpio) == doc_usb_det_en ) {
		printk(KERN_ERR "%s: det a doc\n", __func__);
		gpio_direction_output(doc_usb_oel_gpio, doc_usb_oel_en);
		gpio_set_value(doc_usb_oel_gpio, doc_usb_oel_en);

		/* Rest usb hub for maybe hub is disable */
		gpio_direction_output(usb_hut_rst_gpio, usb_hut_rst_en);
		gpio_set_value(usb_hut_rst_gpio, usb_hut_rst_en);
		mdelay(10);
		gpio_direction_output(usb_hut_rst_gpio, !usb_hut_rst_en);
		gpio_set_value(usb_hut_rst_gpio, !usb_hut_rst_en);

		/* Enable doc power */
		gpio_direction_output(doc_usb_pwr_gpio, doc_usb_pwr_en);
		gpio_set_value(doc_usb_pwr_gpio, doc_usb_pwr_en);
	}
#endif

	return 0;
}
#endif

static struct of_device_id wokoo_gpios_of_match[] = {
		{ .compatible = "wokoo,wokoo-gpios" },
		{ }
};

MODULE_DEVICE_TABLE(of, wokoo_gpios_of_match);
static struct platform_driver wokoo_gpios_driver = {
		.driver         = {
			.name           = "wokoo-gpios",
			.owner          = THIS_MODULE,
			.of_match_table = of_match_ptr(wokoo_gpios_of_match),
		},
		.probe          = wokoo_gpios_probe,
		.remove         = wokoo_gpios_remove,
#ifdef CONFIG_PM
	.suspend        = wokoo_gpios_suspend,
	.resume         = wokoo_gpios_resume,
#endif
};

module_platform_driver(wokoo_gpios_driver);
MODULE_DESCRIPTION("WOKOO gpios Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:WOKOO-gpios-drivers");

/* SPDX-License-Identifier: GPL-2.0
 *
 * Copyright (c) 2020 Codefair Semiconductor Technology Co., Ltd.
 *		http://www.codefairsemi.com 
 *
 */

#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/rtc.h>




#define DRIVER_NAME "wokoo-rtc"

#define WOKOO_RTC_CCVR      0x00 /* Current count register */
#define WOKOO_RTC_CLR       0x04 /* Counter initial value setting register */
#define WOKOO_RTC_CMR_1     0x08 /* Timing match register 1 */
#define WOKOO_RTC_CMR_2     0x0c /* Timing match register 2 */
#define WOKOO_RTC_CMR_3     0x10 /* Timing match register 3 */
#define WOKOO_RTC_ICR       0x14 /* Timing control register */
#define WOKOO_RTC_ISR       0x18 /* Timing interput status register */
#define WOKOO_RTC_EOI       0x18 /* Timing interput clear register */
#define WOKOO_RTC_WVR       0x1c /* Current week counter value register */
#define WOKOO_RTC_WLR       0x20 /* Week initial value setting register */
#define WOKOO_RTC_RAW_LIMIT 0x24 /* Count frequency setting register */
#define WOKOO_RTC_SEC_LIMIT 0x28 /* Second count upper limit control register */
#define WOKOO_RTC_MIN_LIMIT 0x2c /* Minute count upper limit control register */
#define WOKOO_RTC_HOR_LIMIT 0x30 /* Hour count upper limit control register */
#define WOKOO_RTC_ISR_RAW   0x34 /* Interrupt raw status register */
#define WOKOO_RTC_RVR       0x38 /* Count value register in the current second */
#define WOKOO_RTC_PWEN_CTL  0x3c /* Off-chip PMU control register */

#define WOKOO_RTC_CNT_EN    BIT(8) /* Counter enable */
#define WOKOO_RTC_INT_MASK  BIT(7) /* Interupt mask */
#define WOKOO_RTC_INT_T3E   BIT(6) /* Timer interrupt 3 enable */
#define WOKOO_RTC_INT_T1E   BIT(5) /* Timer interrupt 1 enable */
#define WOKOO_RTC_INT_T2E   BIT(4) /* Timer interrupt 2 enable */
#define WOKOO_RTC_INT_SEE   BIT(3) /* Second interrupt enable */
#define WOKOO_RTC_INT_MIE   BIT(2) /* Minute interrupt enable */
#define WOKOO_RTC_INT_HOE   BIT(1) /* Hour interrupt enable */
#define WOKOO_RTC_INT_DAE   BIT(0) /* Day interrupt enable */

#define WOKOO_RTC_MAX_YEAR  88
#define WOKOO_RTC_RS_FLAG   0xaaaaaa




/*
 * wokoo_rtc
 *
 * @rtc: device of rtc
 * @base: register base address
 * @irq: interupt nubmer
 * @clk: rtc gate clock
 */
struct wokoo_rtc {
	struct rtc_device *rtc;
	void __iomem *base;
	int irq;
	struct clk *clk;
};

static void wokoo_w32(struct wokoo_rtc *rtc, u32 reg, u32 val)
{
	writel_relaxed(val, rtc->base + reg);
}

static u32 wokoo_r32(struct wokoo_rtc *rtc, u32 reg)
{
	return readl_relaxed(rtc->base + reg);
}

static void wokoo_rmw(struct wokoo_rtc *rtc, u32 reg, u32 mask, u32 set)
{
	u32 val;

	val = wokoo_r32(rtc, reg);
	val &= ~mask;
	val |= set;
	wokoo_w32(rtc, reg, val);
}

static void wokoo_set(struct wokoo_rtc *rtc, u32 reg, u32 val)
{
	wokoo_rmw(rtc, reg, 0, val);
}

static void wokoo_clr(struct wokoo_rtc *rtc, u32 reg, u32 val)
{
	wokoo_rmw(rtc, reg, val, 0);
}

static void wokoo_rtc_hw_init(struct wokoo_rtc *hw)
{
	/* RTC restart or not (RTC power down then on), use WOKOO_RTC_CMR_3 for judge it */
	if (!(wokoo_r32(hw, WOKOO_RTC_CMR_3) & WOKOO_RTC_RS_FLAG)) {
		/* The setup of the init sequence is for allowing RTC got to work */
		wokoo_w32(hw, WOKOO_RTC_RAW_LIMIT, 0x8000);
		wokoo_w32(hw, WOKOO_RTC_SEC_LIMIT, 0x3c);
		wokoo_w32(hw, WOKOO_RTC_MIN_LIMIT, 0x3c);
		wokoo_w32(hw, WOKOO_RTC_HOR_LIMIT, 0x18);

		/* Reset regs values */
		wokoo_w32(hw, WOKOO_RTC_CLR, 0x0);
		wokoo_w32(hw, WOKOO_RTC_CMR_1, 0x0);
		wokoo_w32(hw, WOKOO_RTC_CMR_2, 0x0);
		wokoo_w32(hw, WOKOO_RTC_WLR, 0x0);
		wokoo_w32(hw, WOKOO_RTC_PWEN_CTL, 0x3);

		/* Clear all interrupt */
		wokoo_w32(hw, WOKOO_RTC_EOI, 0x3f);

		/* Unmask all interrupt and enable RTC */
		wokoo_w32(hw, WOKOO_RTC_ICR, WOKOO_RTC_CNT_EN);
		/* Set restart flag */
		wokoo_w32(hw, WOKOO_RTC_CMR_3, WOKOO_RTC_RS_FLAG);
	}
}

static irqreturn_t wokoo_rtc_alarmirq(int irq, void *id)
{
	struct wokoo_rtc *hw = (struct wokoo_rtc *)id;
	u32 irq_sta;

	irq_sta = wokoo_r32(hw, WOKOO_RTC_ISR);
	if (irq_sta & WOKOO_RTC_INT_T1E) {
		/* Stop alarm also implicitly disables the alarm interrupt */
		wokoo_set(hw, WOKOO_RTC_EOI, WOKOO_RTC_INT_T1E);
		wokoo_clr(hw, WOKOO_RTC_ICR, WOKOO_RTC_INT_T1E);
		rtc_update_irq(hw->rtc, 1, RTC_IRQF | RTC_AF);
		return IRQ_HANDLED;
	} else if (irq_sta & WOKOO_RTC_INT_T2E) {
		/* Stop alarm also implicitly disables the alarm interrupt */
		wokoo_set(hw, WOKOO_RTC_EOI, WOKOO_RTC_INT_T2E);
		wokoo_clr(hw, WOKOO_RTC_ICR, WOKOO_RTC_INT_T2E);
		rtc_update_irq(hw->rtc, 1, RTC_IRQF | RTC_AF);
		return IRQ_HANDLED;
	} else if (irq_sta & WOKOO_RTC_INT_T3E) {
		/* Stop alarm also implicitly disables the alarm interrupt */
		wokoo_set(hw, WOKOO_RTC_EOI, WOKOO_RTC_INT_T3E);
		wokoo_clr(hw, WOKOO_RTC_ICR, WOKOO_RTC_INT_T3E);
		rtc_update_irq(hw->rtc, 1, RTC_IRQF | RTC_AF);
		return IRQ_HANDLED;
	}

	return IRQ_NONE;
}

u8 const table_week[12] = {0, 3, 3, 6, 1, 4, 6, 2, 5, 0, 3, 5};
u8 const table_mon[12]  = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

static void get_year_month_bydays(int days, int *year, int *mon, int *day)
{
	int  yearbase = 1970;
	int  monbase  = 0;

	while (days > 365) {
		if (is_leap_year(yearbase)) {
			if (days > 366) {
				days -= 366;
			} else {
				break;
			}
		} else {
			days -= 365;
		}

		yearbase++;
	}

	*year = yearbase - 1900;

	while(days > 28) {
		if (is_leap_year(yearbase) && monbase == 1) {
			if (days > 29) {
				days -= 29;
			} else {
				break;
			}
		} else {
			if (days > table_mon[monbase]) {
				days -= table_mon[monbase];
			} else {
				break;
			}
		}

		monbase++;
	}

	*mon = monbase;
	*day = days;
}

unsigned int get_days_by_year_month(int year, int mon, int day)
{
	unsigned int  days = 0;
	unsigned int  i    = 0;

	for (i = 1970; i < year; i++) {
		if (is_leap_year(i)) {
			days += 366;
		} else {
			days += 365;
		}
	}

	for (i = 0; i < mon; i++) {
		days += table_mon[i];
		if (is_leap_year(year) && i == 1) {
			days += 1;
		}
	}

	return days + day;
}

static int wokoo_rtc_gettime(struct device *dev, struct rtc_time *tm)
{
	struct wokoo_rtc *hw   = dev_get_drvdata(dev);
	unsigned int      ccvr = 0;
	unsigned int      wlr  = 0;
	unsigned int      days = 0;


	ccvr = wokoo_r32(hw, WOKOO_RTC_CCVR);
	wlr  = wokoo_r32(hw, WOKOO_RTC_WLR) & 0x7;
	tm->tm_sec  =  ccvr & 0x3f;
	tm->tm_min  = (ccvr & (0x3f << 6 )) >> 6;
	tm->tm_hour = (ccvr & (0x1f << 12)) >> 12;
	days        = (ccvr & (0x7fff << 17)) >> 17;

	get_year_month_bydays(days, &tm->tm_year, &tm->tm_mon, &tm->tm_mday);
	//tm->tm_wday = wlr;
	if (tm->tm_mday == 0) {
		tm->tm_mday = 1;
	}

	//printk("wokoo_rtc_gettime = %x, date = %d-%d-%d %d:%d:%d, days = %d\n", ccvr, tm->tm_year + 1900, tm->tm_mon + 1, tm->tm_mday, tm->tm_hour, tm->tm_min, tm->tm_sec, days);
	return 0;
}

static int wokoo_rtc_settime(struct device *dev, struct rtc_time *tm)
{
	struct wokoo_rtc *hw = dev_get_drvdata(dev);
	unsigned int      days = 0;

	//printk("wokoo_rtc_settime date = %d-%d-%d %d:%d:%d\n", tm->tm_year + 1900, tm->tm_mon + 1, tm->tm_mday, tm->tm_hour, tm->tm_min, tm->tm_sec);
	if (tm->tm_year > 70 + WOKOO_RTC_MAX_YEAR)
		return -EINVAL;

	/* Stop time counter before setting a new one*/
	wokoo_clr(hw, WOKOO_RTC_ICR, WOKOO_RTC_CNT_EN);

	days = get_days_by_year_month(tm->tm_year + 1900, tm->tm_mon, tm->tm_mday);
	wokoo_w32(hw, WOKOO_RTC_CLR, tm->tm_sec | (tm->tm_min << 6) |
						(tm->tm_hour << 12) | (days << 17));
	wokoo_w32(hw, WOKOO_RTC_WLR, tm->tm_wday);
	//printk("wokoo_rtc_settime days = %d, CLR = %x, CCVR = %x\n", days, wokoo_r32(hw, WOKOO_RTC_CLR), wokoo_r32(hw, WOKOO_RTC_CCVR));

	/* Restart the time counter */
	wokoo_set(hw, WOKOO_RTC_ICR, WOKOO_RTC_CNT_EN);

	return 0;
}

static int wokoo_rtc_getalarm(struct device *dev, struct rtc_wkalrm *wkalrm)
{
	struct wokoo_rtc  *hw      = dev_get_drvdata(dev);
	struct rtc_time   *tm      = &wkalrm->time;
	unsigned int       cmr     = 0, i;
	unsigned int       weekbit = 0;

	cmr = wokoo_r32(hw, WOKOO_RTC_CMR_1);
	tm->tm_sec  =  cmr & 0x3f;
	tm->tm_min  = (cmr & (0x3f << 6 )) >> 6;
	tm->tm_hour = (cmr & (0x1f << 12)) >> 12;
	weekbit     = (cmr & (0x7f << 17)) >> 17;

	tm->tm_wday = 0;
	for (i = 0; i < 7; i++) {
		if (weekbit & (1 << i)) {
			tm->tm_wday = i;
			break;
		}
	}

	wkalrm->enabled = !!(wokoo_r32(hw, WOKOO_RTC_ICR) & WOKOO_RTC_CNT_EN);
	wkalrm->pending = !!(wokoo_r32(hw, WOKOO_RTC_ICR) & WOKOO_RTC_INT_T1E);

	return 0;
}
static int wokoo_rtc_irq(struct device *dev, unsigned int enabled)
{
	struct wokoo_rtc *hw = dev_get_drvdata(dev);

	 wokoo_rmw(hw,WOKOO_RTC_ICR,WOKOO_RTC_INT_SEE,WOKOO_RTC_INT_SEE);


     return 0;

}

static int wokoo_rtc_setalarm(struct device *dev, struct rtc_wkalrm *wkalrm)
{
	struct wokoo_rtc *hw = dev_get_drvdata(dev);
	struct rtc_time  *tm = &wkalrm->time;

	if (tm->tm_year + 1900 > (1970 + WOKOO_RTC_MAX_YEAR))
		return -EINVAL;

	/*
	 * Stop the alarm also implicitly including disables interrupt before
	 * setting a new one.
	 */
	wokoo_clr(hw, WOKOO_RTC_ICR, WOKOO_RTC_INT_T1E | WOKOO_RTC_INT_MASK);

	/*
	 * Avoid contention between mtk_rtc_setalarm and IRQ handler so that
	 * disabling the interrupt and awaiting for pending IRQ handler to
	 * complete.
	 */
	synchronize_irq(hw->irq);
	pr_info("tm->tm_sec:%d,tm->tm_min:%d,tm->tm_hour:%d,tm->tm_wday:%d\n",tm->tm_sec,tm->tm_min,tm->tm_hour,tm->tm_wday);

	wokoo_w32(hw, WOKOO_RTC_CMR_1, tm->tm_sec | (tm->tm_min << 6) |
						(tm->tm_hour << 12) | (1 << (17 + tm->tm_wday)));

	/* Restart the alarm with the new setup */
	wokoo_set(hw, WOKOO_RTC_ICR, WOKOO_RTC_INT_T1E);

	return 0;
}

static const struct rtc_class_ops wokoo_rtc_ops = {
	.read_time		= wokoo_rtc_gettime,
	.set_time		= wokoo_rtc_settime,
	.read_alarm		= wokoo_rtc_getalarm,
	.set_alarm		= wokoo_rtc_setalarm,
	.alarm_irq_enable = wokoo_rtc_irq,
};

static const struct of_device_id wokoo_rtc_match[] = {
	{ .compatible = "wokoo,wokoo-rtc" },
	{},
};
MODULE_DEVICE_TABLE(of, wokoo_rtc_match);

static int wokoo_rtc_probe(struct platform_device *pdev)
{
	struct wokoo_rtc *hw;
	struct resource *res;
	int ret;

	hw = devm_kzalloc(&pdev->dev, sizeof(*hw), GFP_KERNEL);
	if (!hw)
		return -ENOMEM;

	platform_set_drvdata(pdev, hw);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	hw->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(hw->base))
		return PTR_ERR(hw->base);

	hw->clk = devm_clk_get(&pdev->dev, "rtc");
	if (IS_ERR(hw->clk)) {
		dev_err(&pdev->dev, "No clock\n");
		return PTR_ERR(hw->clk);
	}

	ret = clk_prepare_enable(hw->clk);
	if (ret)
		return ret;

	hw->irq = platform_get_irq(pdev, 0);
	if (hw->irq < 0) {
		ret = hw->irq;
		goto err;
	}

	wokoo_rtc_hw_init(hw);

	ret = devm_request_irq(&pdev->dev, hw->irq, wokoo_rtc_alarmirq,
					0, dev_name(&pdev->dev), hw);
	if (ret) {
		dev_err(&pdev->dev, "Can't request IRQ\n");
		goto err;
	}

	device_init_wakeup(&pdev->dev, true);

	hw->rtc = devm_rtc_device_register(&pdev->dev, pdev->name,
						&wokoo_rtc_ops, THIS_MODULE);
	if (IS_ERR(hw->rtc)) {
		ret = PTR_ERR(hw->rtc);
		dev_err(&pdev->dev, "Unable to register device\n");
		goto err;
	}

	return 0;
err:
	clk_disable_unprepare(hw->clk);

	return ret;
}

static int wokoo_rtc_remove(struct platform_device *pdev)
{
	struct wokoo_rtc *hw = platform_get_drvdata(pdev);

	clk_disable_unprepare(hw->clk);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int wokoo_rtc_suspend(struct device *dev)
{
	struct wokoo_rtc *hw = dev_get_drvdata(dev);

	if (device_may_wakeup(dev))
		enable_irq_wake(hw->irq);

	return 0;
}

static int wokoo_rtc_resume(struct device *dev)
{
	struct wokoo_rtc *hw = dev_get_drvdata(dev);

	if (device_may_wakeup(dev))
		disable_irq_wake(hw->irq);

	return 0;
}

static SIMPLE_DEV_PM_OPS(wokoo_rtc_pm_ops, wokoo_rtc_suspend, wokoo_rtc_resume);

#define WOKOO_RTC_PM_OPS (&wokoo_rtc_pm_ops)
#else	/* CONFIG_PM */
#define WOKOO_RTC_PM_OPS NULL
#endif	/* CONFIG_PM */

static struct platform_driver wokoo_rtc_driver = {
	.probe	= wokoo_rtc_probe,
	.remove	= wokoo_rtc_remove,
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = wokoo_rtc_match,
		.pm = WOKOO_RTC_PM_OPS,
	},
};

module_platform_driver(wokoo_rtc_driver);

MODULE_ALIAS("platform:wokoo-rtc");
MODULE_DESCRIPTION("Wokoo SoC based RTC driver");
MODULE_AUTHOR("GuangHuang Lin<linguanghuang@codefairsemi.com >");
MODULE_LICENSE("GPL v2");

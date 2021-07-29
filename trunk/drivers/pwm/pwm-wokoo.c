// SPDX-License-Identifier: GPL-2.0-only
/*
 *  pwm-wokoo.c Wokoo pwm device driver
 *
 */
#include <linux/bitfield.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <linux/wokoo_pwm.h>

struct wokoo_pwm {
	struct pwm_chip chip;
	struct mutex lock; 		/* protect pwm config/enable */
	struct clk *pclk;
	struct clk *mclk;
	struct clk *sclk;
	void __iomem *regmap;
};

static inline struct wokoo_pwm *to_wokoo_pwm_dev(struct pwm_chip *chip)
{
	return container_of(chip, struct wokoo_pwm, chip);
}

static void wokoo_pwm_disable(struct wokoo_pwm *priv, int ch)
{
	writel_relaxed(0, priv->regmap + WOKOO_PWM_EN);
}

static int wokoo_pwm_enable(struct wokoo_pwm *priv, int ch)
{

	writel_relaxed(1, priv->regmap + WOKOO_PWM_EN);

	return 0;
}

static void wokoo_pwm_config(struct wokoo_pwm *priv, int ch, int duty_ns, int period_ns)
{
	
	writel_relaxed(period_ns, priv->regmap + WOKOO_PWM_P);
	
	writel_relaxed(duty_ns, priv->regmap + WOKOO_PWM_OCPY);
}
		
static void wokoo_pwm_update(struct wokoo_pwm *priv, int ch)
{
	
	writel_relaxed(1, priv->regmap + WOKOO_PWM_UP);
}	
	
static int wokoo_pwm_apply(struct pwm_chip *chip, struct pwm_device *pwm, const struct pwm_state *state)
{
	struct wokoo_pwm *priv = to_wokoo_pwm_dev(chip);
	int ret;
	unsigned long period, duty;
	unsigned long clk_rate;
	unsigned long scaler;
	
	clk_rate = clk_get_rate(priv->sclk);
	if (!clk_rate) 
    {		
        dev_err(chip->dev, "failed to get clock rate\n"); 
	    return -EINVAL; 
    }

	/*
	 * Since period and duty cycle registers have a width of 32
	 * bits, every possible input period can be obtained using the
	 * default prescaler value for all practical clock rate values.
	 */
	scaler = DIV_ROUND_CLOSEST(NSEC_PER_SEC, clk_rate); 
	period = DIV_ROUND_CLOSEST(state->period, scaler);
	period = DIV_ROUND_CLOSEST(period, 100);
	if (period > 255)
		period = 255;


	if (period >=1)
		period -= 1;
	duty = DIV_ROUND_CLOSEST( state->duty_cycle *100,state->period);
	
	wokoo_pwm_disable(priv, pwm->hwpwm);
    wokoo_pwm_config(priv, pwm->hwpwm, duty, period);
	//wokoo_pwm_config(priv, pwm->hwpwm, state->duty_cycle, state->period);

	wokoo_pwm_update(priv, pwm->hwpwm);
	
	ret = wokoo_pwm_enable(priv, pwm->hwpwm);

#if 0	
	printk("*********************************************************\n");
	u32 val = 0;
	val = readl_relaxed(priv->regmap + WOKOO_PWM_EN);
	printk( "enable %d reg is %x\n", pwm->hwpwm, val);
	val = readl_relaxed(priv->regmap + WOKOO_PWM_P);
	printk( "period %d reg is %x\n", pwm->hwpwm, val);
	val = readl_relaxed(priv->regmap + WOKOO_PWM_OCPY);
	printk( "duty_cycle %d reg is %x\n", pwm->hwpwm, val);
#endif

	return ret;
}

static int wokooo_pwm_apply_locked(struct pwm_chip *chip, struct pwm_device *pwm, const struct pwm_state *state)
{
	struct wokoo_pwm *priv = to_wokoo_pwm_dev(chip);
	int ret = 0;

	/* protect common prescaler for all active channels */
	mutex_lock(&priv->lock);
	ret = wokoo_pwm_apply(chip, pwm, state);
	mutex_unlock(&priv->lock);

	return ret;
}

static  const struct pwm_ops wokoo_pwm_ops = {
	.owner = THIS_MODULE,
	.apply = wokooo_pwm_apply_locked,
};

static int wokoo_pwm_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct wokoo_pwm *priv;
	int ret = 0;
		
	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM; 
	
	/* 获取设备树的寄存器配置 */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	
	/* 内存地址映射，将物理地址转换成虚拟地址 */
	priv->regmap = devm_ioremap_resource(&pdev->dev, res);
//	dev_err(&pdev->dev, "mapped PA %08lx to VA %p\n",
//		(unsigned long)res->start, priv->regmap);
	
	priv->pclk = devm_clk_get(&pdev->dev, "pclk");
    if (IS_ERR(priv->pclk)) {
        ret = PTR_ERR(priv->pclk);
        dev_err(&pdev->dev, "failed to get pwm clk: %d\n", ret);
        return ret;
    }
	
	ret = clk_prepare_enable(priv->pclk);
	if (ret) {
		dev_err(&pdev->dev, "failed to prepare PWM clock\n");
		return ret;
	}
	
	priv->mclk = devm_clk_get(&pdev->dev, "mclk");
    if (IS_ERR(priv->mclk)) {
        ret = PTR_ERR(priv->mclk);
        dev_err(&pdev->dev, "failed to get pwm clk: %d\n", ret);
        return ret;
    }
	
	ret = clk_prepare_enable(priv->mclk);
	if (ret) {
		dev_err(&pdev->dev, "failed to prepare PWM clock\n");
		return ret;
	}
	
	priv->sclk = devm_clk_get(&pdev->dev, "sclk");
    if (IS_ERR(priv->sclk)) {
        ret = PTR_ERR(priv->sclk);
        dev_err(&pdev->dev, "failed to get pwm clk: %d\n", ret);
        return ret;
    }
	clk_set_rate(priv->sclk,2000000);
	ret = clk_prepare_enable(priv->sclk);
	mutex_init(&priv->lock);
	priv->chip.base = -1;
	priv->chip.dev = &pdev->dev;
	priv->chip.ops = &wokoo_pwm_ops;
	priv->chip.npwm = 1;
	priv->chip.of_xlate = of_pwm_xlate_with_flags;
	priv->chip.of_pwm_n_cells = 3;
	
	ret = pwmchip_add(&priv->chip);
	if (ret < 0)
		return ret;

	platform_set_drvdata(pdev, priv);

	return 0;
}

static int wokoo_pwm_remove(struct platform_device *pdev)
{
	struct wokoo_pwm *priv = platform_get_drvdata(pdev);
	unsigned int i;

	for (i = 0; i < priv->chip.npwm; i++)
		pwm_disable(&priv->chip.pwms[i]);

	pwmchip_remove(&priv->chip);

	return 0;
}

static const struct of_device_id wokoo_pwm_dt_ids[] = {
	{ .compatible = "wokoo,wokoo-pwm",},
	{ /* end node */ },
};
MODULE_DEVICE_TABLE(of, wokoo_pwm_dt_ids);

static struct platform_driver wokoo_pwm_driver = {
	.probe	= wokoo_pwm_probe,
	.remove	= wokoo_pwm_remove,
	.driver	= {
		.name = "wokoo-pwm",
		.of_match_table = wokoo_pwm_dt_ids,
	},
};

module_platform_driver(wokoo_pwm_driver);

MODULE_ALIAS("platform:wokoo-pwm");
MODULE_DESCRIPTION("WOKOO PWM driver");
MODULE_LICENSE("GPL v2");

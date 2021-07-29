// SPDX-License-Identifier: GPL-2.0-only
/*
 * es8311.c -- es8311 ALSA SoC audio driver
 * Copyright Everest Semiconductor Co.,Ltd
 *
 */

#include <linux/module.h>
#include <linux/acpi.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/mod_devicetable.h>
#include <linux/mutex.h>
#include <linux/regmap.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/tlv.h>
#include <sound/jack.h>
#include "es8311.h"




struct es8311_priv {
	struct mutex lock;
	struct clk *mclk;
	struct regmap *regmap;
	struct snd_soc_component *component;
	struct snd_soc_jack *jack;
	int irq;
	unsigned int sysclk;
	struct snd_pcm_hw_constraint_list sysclk_constraints;
	bool jd_inverted;
};

static const struct snd_kcontrol_new es8311_snd_controls[] = {
	SOC_SINGLE_RANGE("DAC Playback Volume", ES8311_DAC_VOLUME, 0, 0x4f, 0xbf, 0),
	SOC_SINGLE_RANGE("ADC Capture Volume",  ES8311_ADC_VOLUME, 0, 0x4f, 0xcf, 0),
	SOC_SINGLE("DAC Playback Double Fs Switch", ES8311_CLKMGR_CLKSEL, 2, 1, 0),
	SOC_SINGLE("ADC Capture Double Fs Switch", ES8311_CLKMGR_ADCOSR, 6, 1, 0),
};

static const struct snd_soc_dapm_widget es8311_dapm_widgets[] = {
	SND_SOC_DAPM_SUPPLY("ANA ADC Clock", ES8311_CLKMGR_CLKSW, 1, 1, NULL, 0),
	SND_SOC_DAPM_SUPPLY("ANA DAC Clock", ES8311_CLKMGR_CLKSW, 0, 1, NULL, 0),
	SND_SOC_DAPM_SUPPLY("ADC Power Down", ES8311_SYS_LP2, 5, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("PGA Power Down", ES8311_SYS_LP2, 6, 0, NULL, 0),

	SND_SOC_DAPM_SUPPLY("ADC Clock", ES8311_CLKMGR_CLKSW, 3, 1, NULL, 0),
	SND_SOC_DAPM_ADC("Mono ADC", NULL, ES8311_SYS_LP2, 5, 0),
	SND_SOC_DAPM_ADC("Mono DAC", NULL, ES8311_SYS_REF, 1, 0),

	SND_SOC_DAPM_SUPPLY("DAC Clock", ES8311_CLKMGR_CLKSW, 2, 1, NULL, 0),
	SND_SOC_DAPM_SUPPLY("Headphone In", ES8311_HPMIX_SEL, 4, 1, NULL, 0),
};

static const struct snd_soc_dapm_route es8311_dapm_routes[] = {
	{"Mono ADC", NULL, "ANA ADC Clock"},
	{"Mono DAC", NULL, "ANA DAC Clock"},

	{"Mono ADC", NULL, "ADC Clock"},
	{"Mono DAC", NULL, "DAC Clock"},
	{"Mono ADC", NULL, "ADC Power Down"},
	{"Mono ADC", NULL, "PGA Power Down"},
};

static int es8311_set_dai_sysclk(struct snd_soc_dai *codec_dai,
				 int clk_id, unsigned int freq, int dir)
{
	return 0;
}

static int es8311_set_dai_fmt(struct snd_soc_dai *codec_dai,
			      unsigned int fmt)
{
	struct snd_soc_component *component = codec_dai->component;
	u8 serdata1 = 0;
	u8 serdata2 = 0;
	u8 clksw;
	u8 mask;

	if ((fmt & SND_SOC_DAIFMT_MASTER_MASK) != SND_SOC_DAIFMT_CBS_CFS) {
		dev_err(component->dev, "Codec driver only supports slave mode\n");
		return -EINVAL;
	}

	if ((fmt & SND_SOC_DAIFMT_FORMAT_MASK) != SND_SOC_DAIFMT_I2S) {
		dev_err(component->dev, "Codec driver only supports I2S format\n");
		return -EINVAL;
	}

	/* Clock inversion */
	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		break;
	case SND_SOC_DAIFMT_IB_IF:
		serdata1 |= ES8311_SERDATA1_BCLK_INV;
		serdata2 |= ES8311_SERDATA2_ADCLRP;
		break;
	case SND_SOC_DAIFMT_IB_NF:
		serdata1 |= ES8311_SERDATA1_BCLK_INV;
		break;
	case SND_SOC_DAIFMT_NB_IF:
		serdata2 |= ES8311_SERDATA2_ADCLRP;
		break;
	default:
		dev_err(component->dev, "es8311_set_dai_fmt fmt = %x error\n", fmt);
		return -EINVAL;
	}

	mask = ES8311_SERDATA1_BCLK_INV;
	snd_soc_component_update_bits(component, ES8311_SERDATA_DAC, mask, serdata1);

	mask = ES8311_SERDATA2_FMT_MASK | ES8311_SERDATA2_ADCLRP;
	snd_soc_component_update_bits(component, ES8311_SERDATA_ADC, mask, serdata2);

	/* Enable BCLK and MCLK inputs in slave mode */
	clksw = ES8311_CLKMGR_CLKSW_MCLK_ON | ES8311_CLKMGR_CLKSW_BCLK_ON;
	snd_soc_component_update_bits(component, ES8311_CLKMGR_CLKSW, clksw, clksw);

	return 0;
}

static int es8311_pcm_startup(struct snd_pcm_substream *substream,
			      struct snd_soc_dai *dai)
{
	struct snd_soc_component *component = dai->component;
	struct es8311_priv *es8311 = snd_soc_component_get_drvdata(component);

	if (es8311->sysclk_constraints.list)
		snd_pcm_hw_constraint_list(substream->runtime, 0,
					   SNDRV_PCM_HW_PARAM_RATE,
					   &es8311->sysclk_constraints);

	return 0;
}

static int es8311_pcm_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params,
				struct snd_soc_dai *dai)
{
	struct snd_soc_component *component = dai->component;
	u8 wordlen = 0;
	u8 pcm = 0;

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		wordlen = ES8311_SERDATA2_LEN_16;
		pcm = ES8311_SERDATA2_FMT_I2S;
		break;
	case SNDRV_PCM_FORMAT_S20_LE:
		wordlen = ES8311_SERDATA2_LEN_20;
		pcm = ES8311_SERDATA2_FMT_PCM;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		wordlen = ES8311_SERDATA2_LEN_24;
		pcm = ES8311_SERDATA2_FMT_PCM;
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		wordlen = ES8311_SERDATA2_LEN_32;
		pcm = ES8311_SERDATA2_FMT_PCM;
		break;
	default:
		return -EINVAL;
	}

	/* Set serial audio data word length and audio data format */
	snd_soc_component_update_bits(component, ES8311_SERDATA_ADC,
			    ES8311_SERDATA2_LEN_MASK, wordlen);
	snd_soc_component_update_bits(component, ES8311_SERDATA_DAC,
			    ES8311_SERDATA2_LEN_MASK, wordlen);
	snd_soc_component_update_bits(component, ES8311_SERDATA_ADC,
			    ES8311_SERDATA2_FMT_MASK, pcm);
	snd_soc_component_update_bits(component, ES8311_SERDATA_DAC,
			    ES8311_SERDATA2_FMT_MASK, pcm);
	return 0;
}

static int es8311_mute(struct snd_soc_dai *dai, int mute)
{
	/*snd_soc_component_update_bits(dai->component, ES8311_SERDATA_DAC, 0x40,
			    mute ? 0x40 : 0);*/
	return 0;
}

#define ES8311_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_LE | \
			SNDRV_PCM_FMTBIT_S24_LE)

static const struct snd_soc_dai_ops es8311_ops = {
	.startup = es8311_pcm_startup,
	.hw_params = es8311_pcm_hw_params,
	.set_fmt = es8311_set_dai_fmt,
	.set_sysclk = es8311_set_dai_sysclk,
	.digital_mute = es8311_mute,
};

static struct snd_soc_dai_driver es8311_dai = {
	.name = "ES8311 HiFi",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000_96000,
		.formats = ES8311_FORMATS,
	},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000_96000,
		.formats = ES8311_FORMATS,
	},
	.ops = &es8311_ops,
	.symmetric_rates = 1,
};

#ifdef CONFIG_PM
static int es8311_suspend(struct snd_soc_component *component)
{
	snd_soc_component_write(component, ES8311_HPMIX_SEL, 0x40);
	snd_soc_component_write(component, ES8311_SYS_REF, 0x02);
	snd_soc_component_write(component, ES8311_SYS_LP2, 0x6a);
	snd_soc_component_write(component, ES8311_CLKMGR_CLKSW, 0x00);
	msleep(5);

	return 0;
}

static int es8311_resume(struct snd_soc_component *component)
{
	snd_soc_component_write(component, ES8311_CLKMGR_CLKSW, 0x3f);
	snd_soc_component_write(component, ES8311_SYS_LP2, 0x0a);
	snd_soc_component_write(component, ES8311_SYS_REF, 0x00);
	snd_soc_component_write(component, ES8311_HPMIX_SEL, 0x50);

	return 0;
}
#endif

static int es8311_probe(struct snd_soc_component *component)
{
	struct es8311_priv *es8311 = snd_soc_component_get_drvdata(component);

	es8311->component = component;

	/* Reset codec and enable current state machine */
	snd_soc_component_write(component, ES8311_RESET, 0x3f);
	/* Clock doubler path select for playback */
	snd_soc_component_write(component, ES8311_CLKMGR_CLKSEL, 0x04);
	usleep_range(5000, 5500);
	snd_soc_component_write(component, ES8311_RESET, ES8311_RESET_CSM_ON);
	msleep(30);

	snd_soc_component_write(component, 0xc, 0xff);
	/* Power up analog */
	snd_soc_component_write(component, ES8311_SYS_LP1, 0x01);

	/*
	 * Documentation for this register is unclear and incomplete,
	 * but here is a vendor-provided value that improves volume
	 * and quality for Intel CHT platforms.
	 */
	/*
	 * Set 0x72 to enable clock doubler path for capture
	 */
	snd_soc_component_write(component, ES8311_CLKMGR_ADCOSR, 0x32);

	return 0;
}

static void es8311_remove(struct snd_soc_component *component)
{
	struct es8311_priv *es8311 = snd_soc_component_get_drvdata(component);

	clk_disable_unprepare(es8311->mclk);
}

static const struct snd_soc_component_driver soc_component_dev_es8311 = {
	.probe			= es8311_probe,
	.remove			= es8311_remove,
#ifdef CONFIG_PM
	.suspend		= es8311_suspend,
	.resume			= es8311_resume,
#endif
	.controls		= es8311_snd_controls,
	.num_controls		= ARRAY_SIZE(es8311_snd_controls),
	.dapm_widgets		= es8311_dapm_widgets,
	.num_dapm_widgets	= ARRAY_SIZE(es8311_dapm_widgets),
	.dapm_routes		= es8311_dapm_routes,
	.num_dapm_routes	= ARRAY_SIZE(es8311_dapm_routes),
	.use_pmdown_time	= 1,
	.endianness		= 1,
	.non_legacy_dai_naming	= 1,
};

static const struct regmap_range es8311_volatile_ranges[] = {
	regmap_reg_range(ES8311_GPIO_FLAG, ES8311_GPIO_FLAG),
};

static const struct regmap_access_table es8311_volatile_table = {
	.yes_ranges	= es8311_volatile_ranges,
	.n_yes_ranges	= ARRAY_SIZE(es8311_volatile_ranges),
};

static const struct regmap_config es8311_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0x53,
	.volatile_table	= &es8311_volatile_table,
	.cache_type = REGCACHE_RBTREE,
};

static int es8311_i2c_probe(struct i2c_client *i2c_client,
			    const struct i2c_device_id *id)
{
	struct device *dev = &i2c_client->dev;
	struct es8311_priv *es8311;

	es8311 = devm_kzalloc(dev, sizeof(struct es8311_priv),
			      GFP_KERNEL);
	if (es8311 == NULL)
		return -ENOMEM;

	i2c_set_clientdata(i2c_client, es8311);

	es8311->regmap = devm_regmap_init_i2c(i2c_client, &es8311_regmap);
	if (IS_ERR(es8311->regmap))
		return PTR_ERR(es8311->regmap);

	mutex_init(&es8311->lock);

	return devm_snd_soc_register_component(dev,
				      &soc_component_dev_es8311,
				      &es8311_dai, 1);
}

static const struct i2c_device_id es8311_i2c_id[] = {
	{"es8311", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, es8311_i2c_id);

static const struct of_device_id es8311_of_match[] = {
	{ .compatible = "everest,es8311", },
	{},
};
MODULE_DEVICE_TABLE(of, es8311_of_match);

static const struct acpi_device_id es8311_acpi_match[] = {
	{"ESSX8311", 0},
	{},
};
MODULE_DEVICE_TABLE(acpi, es8311_acpi_match);

static struct i2c_driver es8311_i2c_driver = {
	.driver = {
		.name			= "es8311",
		.acpi_match_table	= ACPI_PTR(es8311_acpi_match),
		.of_match_table		= of_match_ptr(es8311_of_match),
	},
	.probe		= es8311_i2c_probe,
	.id_table	= es8311_i2c_id,
};
module_i2c_driver(es8311_i2c_driver);


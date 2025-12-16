/*
 * ASoC Driver for Raspberry Pi add on soundcard
 *
 * Copyright 2018 Nuvoton Technology Corp.
 * Author: John Hsu <KCHSU0@nuvoton.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */

#include <linux/module.h>
#include <linux/types.h>
#include <linux/input.h>
#include <linux/clk.h>
#include <sound/core.h>
#include <sound/soc.h>
#include <sound/pcm_params.h>
#include <sound/control.h>
#include <sound/jack.h>
#include "../codecs/nau8810.h"

#define NOT_4_CUST 1
#define BCM2835_CLK_SRC_GPCLK1 12288000

struct pisound_nau8810_dev {
	struct snd_soc_card *card;
	struct clk *mclk_gpclk;
	unsigned long mclk_rate;
	bool pll_enable;
	bool clk_enable;
};

static const unsigned int bcm2835_rates_12000000[] = {
	8000, 16000, 24000, 32000, 48000, 96000, 176400, 192000
};

static struct snd_pcm_hw_constraint_list bcm2835_constraints_12000000 = {
	.list = bcm2835_rates_12000000,
	.count = ARRAY_SIZE(bcm2835_rates_12000000),
};

static int pisound_nau8810_hw_params(struct snd_pcm_substream *substream,
				       struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = asoc_substream_to_rtd(substream);
	struct snd_soc_dai *cpu_dai = asoc_rtd_to_cpu(rtd, 0);
	struct snd_soc_dai *codec_dai = asoc_rtd_to_codec(rtd, 0);
	struct snd_soc_card *card = rtd->card;
	struct pisound_nau8810_dev *dev_nau8810 =
		snd_soc_card_get_drvdata(card);
	int ret;

#ifdef NOT_4_CUST
	ret = snd_soc_dai_set_bclk_ratio(cpu_dai, 256); // BCLK 12.288MHz
	if (ret)
		dev_err(rtd->dev, "can't set BCLK ratio: %d", ret);
#endif

	if (dev_nau8810->pll_enable) {
		ret = snd_soc_dai_set_sysclk(codec_dai, NAU8810_SCLK_PLL,
			dev_nau8810->mclk_rate, SND_SOC_CLOCK_IN);
		if (ret) {
			dev_err(card->dev, "set sysclk fail (%d)", ret);
			goto err;
		}
		ret = snd_soc_dai_set_pll(codec_dai, 0, 0, dev_nau8810->mclk_rate,
			params_rate(params) * 256);
		if (ret) {
			dev_err(card->dev, "set PLL fail (%d)", ret);
			goto err;
		}
	} else {
		if (dev_nau8810->mclk_gpclk) {
			unsigned long mclk_gpclk_rate;

			dev_nau8810->mclk_rate = params_rate(params) * 256;
			mclk_gpclk_rate =
				clk_round_rate(dev_nau8810->mclk_gpclk,
					dev_nau8810->mclk_rate);
			dev_info(card->dev, "Set mclk_gpclk %luHz",
				mclk_gpclk_rate);
			ret = clk_set_rate(dev_nau8810->mclk_gpclk,
				mclk_gpclk_rate);
			if (ret) {
				dev_err(card->dev, "Unable to set mclk_gpclk rate (%d)",
					ret);
				goto err;
			}
		}
		ret = snd_soc_dai_set_sysclk(codec_dai, NAU8810_SCLK_MCLK,
			dev_nau8810->mclk_rate, SND_SOC_CLOCK_IN);
		if (ret) {
			dev_err(card->dev, "set sysclk fail (%d)", ret);
			goto err;
		}
	}
	dev_info(card->dev, "MCLK %luHz (codec needs 8MHz - 32MHz)",
		dev_nau8810->mclk_rate);

err:
	return ret;
}

static int pisound_nau8810_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = asoc_substream_to_rtd(substream);
	struct snd_soc_card *card = rtd->card;
	struct pisound_nau8810_dev *dev_nau8810 = snd_soc_card_get_drvdata(card);
	int ret;

	/* Setup constraints, because there is a 12 MHz XTAL on the board */
	snd_pcm_hw_constraint_list(substream->runtime, 0,
		SNDRV_PCM_HW_PARAM_RATE, &bcm2835_constraints_12000000);

	if (dev_nau8810->mclk_gpclk && !dev_nau8810->clk_enable) {
		ret = clk_prepare_enable(dev_nau8810->mclk_gpclk);
		if (ret) {
			dev_err(rtd->dev, "Unable to prepare mclk_gpclk (%d)", ret);
			goto err;
		}
		dev_nau8810->clk_enable = 1;
		dev_dbg(rtd->dev, "MCLK enable; Updated mclk_gpclk %luHz",
			clk_get_rate(dev_nau8810->mclk_gpclk));
	}

err:
	return 0;
}

static int platform_clock_control(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *k, int event)
{
	struct snd_soc_dapm_context *dapm = w->dapm;
	struct snd_soc_card *card = dapm->card;
	struct pisound_nau8810_dev *dev_nau8810 = snd_soc_card_get_drvdata(card);

	if (!dev_nau8810->mclk_gpclk || !dev_nau8810->clk_enable)
		return 0;

	if (SND_SOC_DAPM_EVENT_OFF(event)) {
		clk_disable_unprepare(dev_nau8810->mclk_gpclk);
		dev_dbg(card->dev, "MCLK disable");
		dev_nau8810->clk_enable = 0;
	}

	return 0;
}

/* machine stream operations */
static struct snd_soc_ops pisound_nau8810_ops = {
	.startup = pisound_nau8810_startup,
	.hw_params = pisound_nau8810_hw_params,
};

SND_SOC_DAILINK_DEFS(nau8810_link,
	DAILINK_COMP_ARRAY(COMP_CPU("bcm2835-i2s.0")),
	DAILINK_COMP_ARRAY(COMP_CODEC("nau8810.1-001a", "nau8810-hifi")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("bcm2835-i2s.0")));

static struct snd_soc_dai_link pisound_nau8810_dai[] = {
	{
		.name = "pisound nau8810",
		.stream_name = "pisound nau8810",
		.ops = &pisound_nau8810_ops,
#if defined(NAU8810_MASTER)
		.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBM_CFM,
#else
		.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS,
#endif
		SND_SOC_DAILINK_REG(nau8810_link),
	},
};

static const struct snd_kcontrol_new pisound_nau8810_controls[] = {
	SOC_DAPM_PIN_SWITCH("Ext Spk"),
	SOC_DAPM_PIN_SWITCH("Mono Out"),
	SOC_DAPM_PIN_SWITCH("Int Mic"),
	SOC_DAPM_PIN_SWITCH("Aux In"),
};

static const struct snd_soc_dapm_widget pisound_nau8810_dapm_widgets[] = {
	SND_SOC_DAPM_SPK("Ext Spk", NULL),
	SND_SOC_DAPM_LINE("Mono Out", NULL),
	SND_SOC_DAPM_MIC("Int Mic", NULL),
	SND_SOC_DAPM_LINE("Aux In", NULL),
	SND_SOC_DAPM_SUPPLY("Platform Clock", SND_SOC_NOPM, 0, 0,
		platform_clock_control, SND_SOC_DAPM_POST_PMD),
};

static const struct snd_soc_dapm_route pisound_nau8810_audio_map[] = {
	{"Ext Spk", NULL, "SPKOUTP"},
	{"Ext Spk", NULL, "SPKOUTN"},
	{"Mono Out", NULL, "MONOOUT"},

	{"MICP", NULL, "Int Mic"},
	{"MICN", NULL, "Int Mic"},
	{"AUX", NULL, "Aux In"},

	{"Int Mic", NULL, "Platform Clock"},
	{"Ext Spk", NULL, "Platform Clock"},
	{"Mono Out", NULL, "Platform Clock"},
	{"Aux In", NULL, "Platform Clock"},
};

static struct snd_soc_card snd_soc_pisound_nau8810 = {
	.name = "pisoundnau8810",
	.owner = THIS_MODULE,
	.dai_link = pisound_nau8810_dai,
	.num_links = ARRAY_SIZE(pisound_nau8810_dai),

	.controls = pisound_nau8810_controls,
	.num_controls = ARRAY_SIZE(pisound_nau8810_controls),
	.dapm_widgets = pisound_nau8810_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(pisound_nau8810_dapm_widgets),
	.dapm_routes = pisound_nau8810_audio_map,
	.num_dapm_routes = ARRAY_SIZE(pisound_nau8810_audio_map),
};

static int pisound_nau8810_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = &snd_soc_pisound_nau8810;
	struct pisound_nau8810_dev *dev_nau8810;
	unsigned long mclk_gpclk_rate;
	int ret;

	card->dev = &pdev->dev;
	dev_nau8810 = devm_kzalloc(&pdev->dev, sizeof(*dev_nau8810),
		GFP_KERNEL);
	if (!dev_nau8810)
		return -ENOMEM;
	dev_nau8810->card = card;

	if (pdev->dev.of_node) {
		struct snd_soc_dai_link *dai = &pisound_nau8810_dai[0];
		struct device_node *i2s_node = of_parse_phandle(pdev->dev.of_node,
								"i2s-controller", 0);
		if (i2s_node) {
			dai->cpus->dai_name = NULL;
			dai->cpus->of_node = i2s_node;
			dai->platforms->name = NULL;
			dai->platforms->of_node = i2s_node;
		} else
			if (!dai->cpus->of_node) {
				dev_err(&pdev->dev, "Property 'i2s-controller' missing or invalid");
				return -EINVAL;
			}
	}

	snd_soc_card_set_drvdata(card, dev_nau8810);
	ret = devm_snd_soc_register_card(&pdev->dev, &snd_soc_pisound_nau8810);
	if (ret && ret != -EPROBE_DEFER) {
		dev_err(&pdev->dev, "snd_soc_register_card failed (%d)", ret);
	} else {
		int ret, clock_rate = 0;

		dev_nau8810->mclk_gpclk = devm_clk_get(card->dev, NULL);
		if (IS_ERR(dev_nau8810->mclk_gpclk)) {
			dev_info(card->dev, "No 'mclk_gpclk' clock found");
			dev_nau8810->mclk_gpclk = NULL;
		}
		dev_nau8810->pll_enable = device_property_read_bool(card->dev,
			"nuvoton,pll-enable");
		ret = device_property_read_u32(card->dev, "nuvoton,clock-rates", &clock_rate);
		if (ret)
			dev_nau8810->mclk_rate = BCM2835_CLK_SRC_GPCLK1;
		else
			dev_nau8810->mclk_rate = clock_rate;
		dev_info(card->dev, "assign MCLK rate: %luHz", dev_nau8810->mclk_rate);

		if (dev_nau8810->mclk_gpclk) {
			mclk_gpclk_rate = clk_round_rate(dev_nau8810->mclk_gpclk,
				dev_nau8810->mclk_rate);
			ret = clk_set_rate(dev_nau8810->mclk_gpclk, mclk_gpclk_rate);
			if (ret) {
				dev_err(card->dev, "Unable to set mclk_gpclk rate (%d)",
					ret);
				goto clk_err;
			}
		}
		snd_soc_card_set_drvdata(card, dev_nau8810);
	}

clk_err:
	return ret;
}

static int pisound_nau8810_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);
	struct pisound_nau8810_dev *dev_nau8810 =
		snd_soc_card_get_drvdata(card);

	if (dev_nau8810->mclk_gpclk)
		clk_disable_unprepare(dev_nau8810->mclk_gpclk);

	snd_soc_unregister_card(card);

	return 0;
}

static const struct of_device_id pisound_nau8810_of_match[] = {
	{ .compatible = "nuvoton,pisound-nau8810", },
	{},
};
MODULE_DEVICE_TABLE(of, pisound_nau8810_of_match);

static struct platform_driver pisound_nau8810_driver = {
	.driver = {
		.name   = "snd-pisound-nau8810",
		.owner  = THIS_MODULE,
		.of_match_table = pisound_nau8810_of_match,
	},
	.probe = pisound_nau8810_probe,
	.remove = pisound_nau8810_remove,
};
module_platform_driver(pisound_nau8810_driver);

MODULE_DESCRIPTION("NAU88C10 Pi Soundcard");
MODULE_AUTHOR("John Hsu <KCHSU0@nuvoton.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:pisound-nau8810");
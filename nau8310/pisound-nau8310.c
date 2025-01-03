/*
 * ASoC Driver for Raspberry Pi add on soundcard
 *
 * Copyright 2021 Nuvoton Technology Corp.
 * Author: David Lin <CTLIN0@nuvoton.com>
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
#define DEBUG
#include <linux/module.h>
#include <linux/types.h>
#include <linux/input.h>
#include <linux/clk.h>
#include <sound/core.h>
#include <sound/soc.h>
#include <sound/pcm_params.h>
#include <sound/control.h>
#include <sound/jack.h>

#include "../codecs/nau8310.h"

#define BCM2835_CLK_SRC_GPCLK1 25000000
#define NAU8310_CLK_SRC_12 12288000
#define NAU8310_CLK_SRC_19 19200000
#define NAU8310_CLK_SRC_24 24000000

#define RPI_TDM_I2S

#ifdef CONFIG_OF
#define	COMPONENT_NAME_LEFT "nau8310.1-0010"
#define COMPONENT_NAME_RIGHT "nau8310.1-0011"
#else
#define COMPONENT_NAME_LEFT "i2c-NVTN2000:00"
#define COMPONENT_NAME_RIGHT "i2c-NVTN2000:01"
#endif

struct pisound_nau8310_dev {
	struct snd_soc_card *card;
	struct clk *mclk_gpclk;
	unsigned long mclk_rate;
	bool clk_enable;
};

static const unsigned int nau8310_rates[] = {
	48000,
};

static struct snd_pcm_hw_constraint_list nau8310_constraints = {
	.list = nau8310_rates,
	.count = ARRAY_SIZE(nau8310_rates),
};

static int pisound_nau8310_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = asoc_substream_to_rtd(substream);
	struct snd_soc_card *card = rtd->card;
	struct pisound_nau8310_dev *dev_nau8310 = snd_soc_card_get_drvdata(card);
	int ret;
	dev_dbg(rtd->dev, "%s\n", __func__);
	dev_dbg(rtd->dev, "constraint to 48k\n");
	/* Setup constraints, because when nau8310 enable DSP, it just support 48K */
	snd_pcm_hw_constraint_list(substream->runtime, 0,
	                           SNDRV_PCM_HW_PARAM_RATE, &nau8310_constraints);

	if (dev_nau8310->mclk_gpclk && !dev_nau8310->clk_enable) {
		ret = clk_prepare_enable(dev_nau8310->mclk_gpclk);
		if (ret) {
			dev_err(rtd->dev, "Unable to prepare mclk_gpclk (%d)\n", ret);
			goto err;
		}
		dev_nau8310->clk_enable = 1;
		dev_dbg(rtd->dev, "Updated mclk_gpclk %luHz\n", clk_get_rate(dev_nau8310->mclk_gpclk));
		dev_dbg(rtd->dev, "MCLK enable");
	}

err:
	return 0;
}

static int pisound_nau8310_hw_params(struct snd_pcm_substream *substream,
                                     struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = asoc_substream_to_rtd(substream);
	struct snd_soc_card *card = rtd->card;
	struct snd_soc_dai *codec_dai = asoc_rtd_to_codec(rtd, 0);
	struct snd_soc_dai *cpu_dai = asoc_rtd_to_cpu(rtd, 0);
	struct pisound_nau8310_dev *dev_nau8310 = snd_soc_card_get_drvdata(card);
	unsigned int sample_bits = snd_pcm_format_physical_width(params_format(params));
	int i, ret;
	dev_dbg(rtd->dev, "%s\n", __func__);
#ifdef RPI_TDM_I2S //if set bclk_ratio, RPI3 will be TDM I2S mode.
	ret = snd_soc_dai_set_bclk_ratio(cpu_dai, 2 * sample_bits);
	if (ret < 0) {
		dev_err(rtd->dev, "can't set BCLK ratio: %d\n", ret);
	}
#endif
	for_each_rtd_codec_dais(rtd, i, codec_dai) {
#ifdef RPI_TDM_I2S
		if (!strcmp(codec_dai->component->name, COMPONENT_NAME_LEFT)) {
			/* DEV0 tdm slot configuration */
			snd_soc_dai_set_tdm_slot(codec_dai, 0x11, 0x1, 8, 16);
		}
		if (!strcmp(codec_dai->component->name, COMPONENT_NAME_RIGHT)) {
			/* DEV1 tdm slot configuration */
			snd_soc_dai_set_tdm_slot(codec_dai, 0x22, 0x2, 8, 16);
		}
#endif
		/* Configure sysclk for codec */
		ret = snd_soc_dai_set_sysclk(codec_dai, 0, dev_nau8310->mclk_rate, SND_SOC_CLOCK_IN);
		if (ret < 0) {
			dev_err(rtd->dev, "failed to set sysclk\n");
			return ret;
		}
	}

	return ret;
}

static void pisound_nau8310_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = asoc_substream_to_rtd(substream);
	struct snd_soc_card *card = rtd->card;
	struct pisound_nau8310_dev *dev_nau8310 = snd_soc_card_get_drvdata(card);

	dev_dbg(rtd->dev, "%s\n", __func__);

	if (dev_nau8310->mclk_gpclk && dev_nau8310->clk_enable) {
		dev_dbg(rtd->dev, "MCLK disable\n");
		clk_disable_unprepare(dev_nau8310->mclk_gpclk);
		dev_nau8310->clk_enable = 0;
	}
}

/* machine stream operations */
static struct snd_soc_ops pisound_nau8310_ops = {
	.hw_params = pisound_nau8310_hw_params,
	.startup = pisound_nau8310_startup,
	.shutdown = pisound_nau8310_shutdown,
};

SND_SOC_DAILINK_DEF(pisnd,
                    DAILINK_COMP_ARRAY(COMP_CPU("bcm2835-i2s")));

SND_SOC_DAILINK_DEF(platform, /* subject to be overridden during probe */
                    DAILINK_COMP_ARRAY(COMP_PLATFORM("bcm2835-i2s")));


SND_SOC_DAILINK_DEF(amps,
                    DAILINK_COMP_ARRAY(COMP_CODEC(COMPONENT_NAME_LEFT, NAU8310_CODEC_DAI)),
                    COMP_CODEC(COMPONENT_NAME_RIGHT, NAU8310_CODEC_DAI));

static struct snd_soc_dai_link pisound_nau8310_dai[] = {
	{
		.name = "pisound-nau8310",
		.platforms = platform,
		.num_platforms = ARRAY_SIZE(platform),
		.cpus = pisnd,
		.num_cpus = 1,
		.codecs = amps,
		.num_codecs = ARRAY_SIZE(amps),
		.ignore_pmdown_time = 1,
		.ops = &pisound_nau8310_ops,
		.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS,
	},

};

static const struct snd_kcontrol_new pisound_nau8310_controls[] = {
	SOC_DAPM_PIN_SWITCH("Left Spk"),
	SOC_DAPM_PIN_SWITCH("Right Spk"),
};

static const struct snd_soc_dapm_widget pisound_nau8310_dapm_widgets[] = {
	SND_SOC_DAPM_SPK("Left Spk", NULL),
	SND_SOC_DAPM_SPK("Right Spk", NULL),
};

static const struct snd_soc_dapm_route pisound_nau8310_audio_map[] = {
	/* speaker */
	{ "Left Spk", NULL, "Left Speaker" },
	{ "Right Spk", NULL, "Right Speaker" },
};

static struct snd_soc_codec_conf nau8310_codec_conf[] = {
	{
		.dlc = COMP_CODEC_CONF(COMPONENT_NAME_LEFT),
		.name_prefix = "Left",
	},
	{
		.dlc = COMP_CODEC_CONF(COMPONENT_NAME_RIGHT),
		.name_prefix = "Right",
	},
};

static struct snd_soc_card snd_soc_pisound_nau8310 = {
	.name = "pisoundnau8310",
	.owner = THIS_MODULE,
	.dai_link = pisound_nau8310_dai,
	.num_links = ARRAY_SIZE(pisound_nau8310_dai),

	.controls = pisound_nau8310_controls,
	.num_controls = ARRAY_SIZE(pisound_nau8310_controls),
	.dapm_widgets = pisound_nau8310_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(pisound_nau8310_dapm_widgets),
	.dapm_routes = pisound_nau8310_audio_map,
	.num_dapm_routes = ARRAY_SIZE(pisound_nau8310_audio_map),

	.codec_conf = nau8310_codec_conf,
	.num_configs = ARRAY_SIZE(nau8310_codec_conf),

};

static int pisound_nau8310_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = &snd_soc_pisound_nau8310;
	struct pisound_nau8310_dev *dev_nau8310;
	unsigned long mclk_gpclk_rate;
	int ret;

	card->dev = &pdev->dev;
	dev_nau8310 = devm_kzalloc(&pdev->dev, sizeof(*dev_nau8310),
	                           GFP_KERNEL);
	if (!dev_nau8310)
		return -ENOMEM;
	dev_nau8310->card = card;

	if (pdev->dev.of_node) {
		struct snd_soc_dai_link *dai = &pisound_nau8310_dai[0];
		struct device_node *i2s_node = of_parse_phandle(pdev->dev.of_node,
		                               "i2s-controller", 0);
		if (i2s_node) {
			dai->cpus->dai_name = NULL;
			dai->cpus->of_node = i2s_node;
			dai->platforms->name = NULL;
			dai->platforms->of_node = i2s_node;
		} else if (!dai->cpus->of_node) {
			dev_err(&pdev->dev, "Property 'i2s-controller' missing or invalid\n");
			return -EINVAL;
		}
	}

	ret = devm_snd_soc_register_card(&pdev->dev, &snd_soc_pisound_nau8310);
	if (ret && ret != -EPROBE_DEFER)
		dev_err(&pdev->dev, "Failed to register card %d\n", ret);
	else {
		int ret, clock_rate = 0;

		dev_nau8310->mclk_gpclk = devm_clk_get(card->dev, NULL);
		if (IS_ERR(dev_nau8310->mclk_gpclk)) {
			dev_info(card->dev, "No 'mclk_gpclk' clock found");
			dev_nau8310->mclk_gpclk = NULL;
		}
		ret = device_property_read_u32(card->dev, "nuvoton,clock-rates", &clock_rate);
		if (ret)
			dev_nau8310->mclk_rate = BCM2835_CLK_SRC_GPCLK1;
		else
			dev_nau8310->mclk_rate = NAU8310_CLK_SRC_12;
		dev_info(card->dev, "assign MCLK rate: %luHz\n", dev_nau8310->mclk_rate);

		if (dev_nau8310->mclk_gpclk) {
			mclk_gpclk_rate = clk_round_rate(dev_nau8310->mclk_gpclk,
			                                 dev_nau8310->mclk_rate);
			ret = clk_set_rate(dev_nau8310->mclk_gpclk,
			                   mclk_gpclk_rate);
			if (ret) {
				dev_err(card->dev, "Unable to set mclk_gpclk rate (%d)\n", ret);
				goto clk_err;
			}
#if 0//defined(DEBUG)
			ret = clk_prepare_enable(dev_nau8310->mclk_gpclk);
			if (ret) {
				dev_err(card->dev, "Unable to prepare mclk_gpclk (%d)\n", ret);
				goto clk_err;
			}
			mclk_gpclk_rate = clk_get_rate(dev_nau8310->mclk_gpclk);
			dev_dbg(card->dev, "Updated mclk_gpclk %luHz\n", mclk_gpclk_rate);
#endif
		}
		snd_soc_card_set_drvdata(card, dev_nau8310);
	}

clk_err:
	return ret;
}

static int pisound_nau8310_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);
	snd_soc_unregister_card(card);

	return 0;
}

static const struct of_device_id pisound_nau8310_of_match[] = {
	{ .compatible = "nuvoton,pisound-nau8310", },
	{},
};
MODULE_DEVICE_TABLE(of, pisound_nau8310_of_match);

static struct platform_driver pisound_nau8310_driver = {
	.driver = {
		.name   = "snd-pisound-nau8310",
		.owner  = THIS_MODULE,
		.of_match_table = pisound_nau8310_of_match,
	},
	.probe = pisound_nau8310_probe,
	.remove = pisound_nau8310_remove,
};
module_platform_driver(pisound_nau8310_driver);

MODULE_DESCRIPTION("nau8310 Pi Soundcard");
MODULE_AUTHOR("David Lin <ctlin0@nuvoton.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:pisound-nau8310");

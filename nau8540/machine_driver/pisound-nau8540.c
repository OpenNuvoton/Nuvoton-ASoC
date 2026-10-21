/*
 * ASoC Driver for AudioInjector Pi add on soundcard
 *
 *  Created on: 13-May-2016
 *      Author: flatmax@flatmax.org
 *              based on code by  Cliff Cai <Cliff.Cai@analog.com> for the ssm2602 machine blackfin.
 *              with help from Lars-Peter Clausen for simplifying the original code to use the dai_fmt field.
 *		i2s_node code taken from the other sound/soc/bcm machine drivers.
 *
 * Copyright (C) 2016 Flatmax Pty. Ltd.
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

#include <sound/core.h>
#include <sound/soc.h>
#include <sound/pcm_params.h>
#include <sound/control.h>
#include "../codecs/nau8542.h"

static const unsigned int bcm2835_rates_12000000[] = {
	8000, 16000, 32000, 44100, 48000, 96000, 88200,
};

static struct snd_pcm_hw_constraint_list bcm2835_constraints_12000000 = {
	.list = bcm2835_rates_12000000,
	.count = ARRAY_SIZE(bcm2835_rates_12000000),
};

static int pisound_nau8540_startup(struct snd_pcm_substream *substream)
{
	/* Setup constraints, because there is a 12 MHz XTAL on the board */
	snd_pcm_hw_constraint_list(substream->runtime, 0,
				SNDRV_PCM_HW_PARAM_RATE,
				&bcm2835_constraints_12000000);
	return 0;
}

static int pisound_nau8540_hw_params(struct snd_pcm_substream *substream,
				       struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	int channels, width, rate;
	int ret = 0;

	channels = params_channels(params);
	width = snd_pcm_format_physical_width(params_format(params));
	rate = params_rate(params);

	dev_info(rtd->dev, "%s:%d channels %d width %d rate %d\n",
		__func__, __LINE__, channels, width, rate);

	/* set codec DAI configuration */
	/* TDM 4 slot 24 bit set the Rx and Tx bitmask to
	* 4 active slots as 0xF
	*/
	ret = snd_soc_dai_set_tdm_slot(codec_dai, 0xf, 0, channels, width);
	if (ret < 0) {
		pr_err("can't set codec TDM slot %d\n", ret);
		return ret;
	}
	ret = snd_soc_dai_set_tdm_slot(codec_dai, 0xf0, 0, channels, width);
	if (ret < 0) {
		pr_err("can't set codec TDM slot %d\n", ret);
		return ret;
	}

	switch (params_rate(params)){
		case 8000:
			return snd_soc_dai_set_bclk_ratio(cpu_dai, 1);
		case 16000:
			return snd_soc_dai_set_bclk_ratio(cpu_dai, 750);
		case 32000:
			return snd_soc_dai_set_bclk_ratio(cpu_dai, 375);
		case 44100:
			return snd_soc_dai_set_bclk_ratio(cpu_dai, 272);
		case 48000:
			return snd_soc_dai_set_bclk_ratio(cpu_dai, 250);
		case 88200:
			return snd_soc_dai_set_bclk_ratio(cpu_dai, 136);
		case 96000:
			return snd_soc_dai_set_bclk_ratio(cpu_dai, 125);
		default:
			return snd_soc_dai_set_bclk_ratio(cpu_dai, 125);
	}
}

/* machine stream operations */
static struct snd_soc_ops pisound_nau8540_ops = {
	.startup = pisound_nau8540_startup,
	.hw_params = pisound_nau8540_hw_params,
};

static int pisound_nau8540_dai_init(struct snd_soc_pcm_runtime *rtd)
{
	//return snd_soc_dai_set_sysclk(rtd->codec_dai, WM8731_SYSCLK_XTAL, 12000000, SND_SOC_CLOCK_IN);
	return 0;
}

static struct snd_soc_dai_link pisound_nau8540_dai[] = {
	{
		.name = "pisound-nau8540",
		.stream_name = "pisound-nau8540",
		.cpu_dai_name	= "bcm2708-i2s.0",
		.codec_dai_name = "nau8540-hifi",
		.platform_name	= "bcm2835-i2s.0",
		.codec_name = "nau8540.1-001c",
		.ops = &pisound_nau8540_ops,
		.init = pisound_nau8540_dai_init,
		//.dai_fmt = SND_SOC_DAIFMT_CBM_CFM|SND_SOC_DAIFMT_I2S|SND_SOC_DAIFMT_NB_NF,
		.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
			SND_SOC_DAIFMT_CBS_CFS,
	},
};

static const struct snd_soc_dapm_widget pisound_nau8540_dapm_widgets[] = {
	SND_SOC_DAPM_MIC("Microphone", NULL),
};

static const struct snd_soc_dapm_route pisound_nau8540_audio_map[] = {
	{"Microphone", NULL, "MIC1"},
	{"Microphone", NULL, "MIC2"},
	{"Microphone", NULL, "MIC3"},
	{"Microphone", NULL, "MIC4"},
};

static struct snd_soc_card snd_soc_pisound_nau8540 = {
	.name = "pisound_nau8540",
	.dai_link = pisound_nau8540_dai,
	.num_links = ARRAY_SIZE(pisound_nau8540_dai),

	.dapm_widgets = pisound_nau8540_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(pisound_nau8540_dapm_widgets),
	.dapm_routes = pisound_nau8540_audio_map,
	.num_dapm_routes = ARRAY_SIZE(pisound_nau8540_audio_map),
};

static int pisound_nau8540_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = &snd_soc_pisound_nau8540;
	int ret;

	dev_info(&pdev->dev, "%s:%d\n", __func__, __LINE__);

	card->dev = &pdev->dev;

	if (pdev->dev.of_node) {
		struct snd_soc_dai_link *dai = &pisound_nau8540_dai[0];
		struct device_node *i2s_node = of_parse_phandle(pdev->dev.of_node,
								"i2s-controller", 0);

		if (i2s_node) {
			dai->cpu_dai_name = NULL;
			dai->cpu_of_node = i2s_node;
			dai->platform_name = NULL;
			dai->platform_of_node = i2s_node;
		} else
			if (!dai->cpu_of_node) {
				dev_err(&pdev->dev, "Property 'i2s-controller' missing or invalid\n");
				return -EINVAL;
			}
	}

	if ((ret = snd_soc_register_card(card))) {
		dev_err(&pdev->dev, "snd_soc_register_card failed (%d)\n", ret);
	}
	return ret;
}

static int pisound_nau8542_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);

	snd_soc_unregister_card(card);

	return 0;
}

static const struct of_device_id pisound_nau8542_of_match[] = {
	{ .compatible = "nuvoton,pisound-nau8542", },
	{},
};
MODULE_DEVICE_TABLE(of, pisound_nau8542_of_match);

static struct platform_driver pisound_nau8542_driver = {
	.driver = {
		.name   = "snd-pisound-nau8542",
		.owner  = THIS_MODULE,
		.of_match_table = pisound_nau8542_of_match,
	},
	.probe = pisound_nau8542_probe,
	.remove = pisound_nau8542_remove,
};
module_platform_driver(pisound_nau8542_driver);

MODULE_DESCRIPTION("NAU8542 Pi Soundcard");
MODULE_AUTHOR("John Hsu <KCHSU0@nuvoton.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:pisound-nau8542");
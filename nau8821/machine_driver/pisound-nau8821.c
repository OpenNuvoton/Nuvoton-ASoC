/*
 * ASoC Driver for Raspberry Pi add on soundcard
 *
 * Copyright 2020 Nuvoton Technology Corp.
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
#include <sound/core.h>
#include <sound/soc.h>
#include <sound/pcm_params.h>
#include <sound/control.h>
#include <sound/jack.h>
#include "../codecs/nau8821.h"

static struct snd_soc_jack pisound_headset;
static struct snd_soc_card snd_soc_pisound_nau8821;

static struct snd_soc_jack_pin pisound_jack_pins[] = {
	{
		.pin	= "Headphone",
		.mask	= SND_JACK_HEADPHONE,
	},
	{
		.pin	= "Headset Mic",
		.mask	= SND_JACK_MICROPHONE,
	},
};

static const unsigned int bcm2835_rates_12000000[] = {
	8000, 16000, 32000, 44100, 48000, 96000, 88200,
};

static struct snd_pcm_hw_constraint_list bcm2835_constraints_12000000 = {
	.list = bcm2835_rates_12000000,
	.count = ARRAY_SIZE(bcm2835_rates_12000000),
};

static int pisound_nau8821_startup(struct snd_pcm_substream *substream)
{
	/* Setup constraints, because there is a 12 MHz XTAL on the board */
	snd_pcm_hw_constraint_list(substream->runtime, 0,
		SNDRV_PCM_HW_PARAM_RATE, &bcm2835_constraints_12000000);

	return 0;
}

static int pisound_nau8821_hw_params(struct snd_pcm_substream *substream,
				       struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_card *card = rtd->card;
	struct snd_soc_dai *cpu_dai = asoc_rtd_to_cpu(rtd, 0);
	struct snd_soc_dai *codec_dai = asoc_rtd_to_codec(rtd, 0);
	unsigned int sample_bits = snd_pcm_format_physical_width(params_format(params));
	int ret;

#ifdef NOT_4_CUST
	ret = snd_soc_dai_set_bclk_ratio(cpu_dai, 256); // BCLK 12.288MHz
#else
	ret = snd_soc_dai_set_bclk_ratio(cpu_dai, sample_bits * 2);
#endif
	if (ret < 0)
		dev_err(card->dev, "can't set BCLK ratio: %d\n", ret);

#if 0 // FLL source from BCLK
	ret = snd_soc_dai_set_sysclk(codec_dai, NAU8821_CLK_FLL_BLK, 0,
		SND_SOC_CLOCK_IN);
	if (ret < 0)
		dev_err(card->dev, "can't set BCLK clock %d\n", ret);
	ret = snd_soc_dai_set_pll(codec_dai, 0, 0,
		snd_soc_params_to_bclk(params),
		params_rate(params) * 256);
	if (ret < 0)
		dev_err(card->dev, "can't set FLL: %d\n", ret);
#else // FLL source from FS
	ret = snd_soc_dai_set_sysclk(codec_dai, NAU8821_CLK_FLL_FS, 0,
		SND_SOC_CLOCK_IN);
	if (ret < 0)
		dev_err(card->dev, "can't set FS clock %d\n", ret);
	ret = snd_soc_dai_set_pll(codec_dai, 0, 0, params_rate(params),
		params_rate(params) * 256);
	if (ret < 0)
		dev_err(card->dev, "can't set FLL: %d\n", ret);
#endif

	return ret;
}

/* machine stream operations */
static struct snd_soc_ops pisound_nau8821_ops = {
	.startup = pisound_nau8821_startup,
	.hw_params = pisound_nau8821_hw_params,
};

static int pisound_nau8821_dai_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_component *component = asoc_rtd_to_codec(rtd, 0)->component;
	struct snd_soc_jack *jack = &pisound_headset;
	int ret;

	ret = snd_soc_card_jack_new_pins(&snd_soc_pisound_nau8821, "Headset Jack",
		SND_JACK_HEADSET | SND_JACK_BTN_0, &pisound_headset,
		pisound_jack_pins, ARRAY_SIZE(pisound_jack_pins));
	if (ret) {
		dev_err(rtd->dev, "Headset Jack creation failed %d\n", ret);
		return ret;
	}

	snd_soc_component_set_jack(component, jack, NULL);

	return ret;
}

SND_SOC_DAILINK_DEFS(nau8821_link,
	DAILINK_COMP_ARRAY(COMP_CPU("bcm2835-i2s.0")),
	DAILINK_COMP_ARRAY(COMP_CODEC("nau8821.1-001b", "nau8821-hifi")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("bcm2835-i2s.0")));

static struct snd_soc_dai_link pisound_nau8821_dai[] = {
	{
		.name = "pisound nau8821",
		.stream_name = "pisound nau8821",
		.ops = &pisound_nau8821_ops,
		.init = pisound_nau8821_dai_init,
#if 1 // slave mode
		.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS,
#else // master mode
		.dai_fmt = SND_SOC_DAIFMT_CBM_CFM|SND_SOC_DAIFMT_I2S|SND_SOC_DAIFMT_NB_NF,
#endif
		SND_SOC_DAILINK_REG(nau8821_link),
	},
};

static const struct snd_kcontrol_new pisound_nau8821_controls[] = {
	SOC_DAPM_PIN_SWITCH("Headphone"),
	SOC_DAPM_PIN_SWITCH("Headset Mic"),
};

#define PI_NUVOTON_CODEC_DAI	"nau8821-hifi"

static inline struct snd_soc_dai *pi_get_codec_dai(struct snd_soc_card *card)
{
	struct snd_soc_pcm_runtime *rtd;
	struct snd_soc_dai *codec_dai;
	int i;

	list_for_each_entry(rtd, &card->rtd_list, list) {
		for_each_rtd_codec_dais(rtd, i, codec_dai) {
			if (!strncmp(codec_dai->name, PI_NUVOTON_CODEC_DAI,
				strlen(PI_NUVOTON_CODEC_DAI)))
				return codec_dai;
		}
	}

	return NULL;
}

static int platform_clock_control(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *k, int  event)
{
	struct snd_soc_dapm_context *dapm = w->dapm;
	struct snd_soc_card *card = dapm->card;
	struct snd_soc_dai *codec_dai;
	int ret;

	codec_dai = pi_get_codec_dai(card);
	if (!codec_dai) {
		dev_err(card->dev, "Codec dai not found\n");
		return -EIO;
	}

	if (SND_SOC_DAPM_EVENT_OFF(event)) {
		ret = snd_soc_dai_set_sysclk(codec_dai,
			NAU8821_CLK_INTERNAL, 0, SND_SOC_CLOCK_IN);
		if (ret < 0) {
			dev_err(card->dev, "set sysclk err = %d\n", ret);
			return -EIO;
		}
	}
	return ret;
}

static const struct snd_soc_dapm_widget pisound_nau8821_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Headphone", NULL),
	SND_SOC_DAPM_MIC("Headset Mic", NULL),

	SND_SOC_DAPM_SUPPLY("Platform Clock", SND_SOC_NOPM, 0, 0,
			platform_clock_control, SND_SOC_DAPM_PRE_PMU |
			SND_SOC_DAPM_POST_PMD),
};

static const struct snd_soc_dapm_route pisound_nau8821_audio_map[] = {
	{ "Headphone", NULL, "HPOL" },
	{ "Headphone", NULL, "HPOR" },
	{ "Headset Mic", NULL, "MICL" },
	{ "Headset Mic", NULL, "MICR" },
	{ "Headphone", NULL, "Platform Clock" },
	{ "Headset Mic", NULL, "Platform Clock" },
};

static struct snd_soc_card snd_soc_pisound_nau8821 = {
	.name = "pisoundnau8821",
	.owner = THIS_MODULE,
	.dai_link = pisound_nau8821_dai,
	.num_links = ARRAY_SIZE(pisound_nau8821_dai),
	.controls = pisound_nau8821_controls,
	.num_controls = ARRAY_SIZE(pisound_nau8821_controls),
	.dapm_widgets = pisound_nau8821_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(pisound_nau8821_dapm_widgets),
	.dapm_routes = pisound_nau8821_audio_map,
	.num_dapm_routes = ARRAY_SIZE(pisound_nau8821_audio_map),
};

static int pisound_nau8821_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = &snd_soc_pisound_nau8821;
	int ret;

	card->dev = &pdev->dev;

	if (pdev->dev.of_node) {
		struct snd_soc_dai_link *dai = &pisound_nau8821_dai[0];
		struct device_node *i2s_node = of_parse_phandle(pdev->dev.of_node,
								"i2s-controller", 0);
		if (i2s_node) {
			dai->cpus->dai_name = NULL;
			dai->cpus->of_node = i2s_node;
			dai->platforms->name = NULL;
			dai->platforms->of_node = i2s_node;
		} else
			if (!dai->cpus->of_node) {
				dev_err(&pdev->dev, "Property 'i2s-controller' missing or invalid\n");
				return -EINVAL;
			}
	}

	ret = devm_snd_soc_register_card(&pdev->dev, &snd_soc_pisound_nau8821);
	if (ret && ret != -EPROBE_DEFER)
		dev_err(&pdev->dev, "Failed to register card %d\n", ret);

	return ret;
}

static int pisound_nau8821_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);

	snd_soc_unregister_card(card);

	return 0;
}

static const struct of_device_id pisound_nau8821_of_match[] = {
	{ .compatible = "nuvoton,pisound-nau8821", },
	{},
};
MODULE_DEVICE_TABLE(of, pisound_nau8821_of_match);

static struct platform_driver pisound_nau8821_driver = {
	.driver = {
		.name   = "snd-pisound-nau8821",
		.owner  = THIS_MODULE,
		.of_match_table = pisound_nau8821_of_match,
	},
	.probe = pisound_nau8821_probe,
	.remove = pisound_nau8821_remove,
};
module_platform_driver(pisound_nau8821_driver);

MODULE_DESCRIPTION("NAU88L21 Pi Soundcard");
MODULE_AUTHOR("John Hsu <KCHSU0@nuvoton.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:pisound-nau8821");
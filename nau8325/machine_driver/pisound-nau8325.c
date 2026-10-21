 // SPDX-License-Identifier: GPL-2.0
 /*
  * ASoC Driver for Raspberry Pi add on soundcard
  *
  * Copyright (c) 2022 Nuvoton Technology Corp.
  * Author: Seven Lee <wtli@nuvoton.com>
  *
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
#include "../codecs/nau8325.h"

#define BCM2835_CLK_SRC_GPCLK1 25000000

struct pisound_nau8325_dev {
	struct snd_soc_card *card;
	struct clk *mclk_gpclk;
	unsigned long mclk_rate;
};

static struct snd_soc_card snd_soc_pisound_nau8325;

static const unsigned int bcm2835_rates_12000000[] = {
	8000, 16000, 32000, 44100, 48000, 96000, 83250,
};

static struct snd_pcm_hw_constraint_list bcm2835_constraints_12000000 = {
	.list = bcm2835_rates_12000000,
	.count = ARRAY_SIZE(bcm2835_rates_12000000),
};

static int pisound_nau8325_startup(struct snd_pcm_substream *substream)
{
	/* Setup constraints, because there is a 12 MHz XTAL on the board */
	snd_pcm_hw_constraint_list(substream->runtime, 0,
		SNDRV_PCM_HW_PARAM_RATE, &bcm2835_constraints_12000000);

	return 0;
}

static int pisound_nau8325_hw_params(struct snd_pcm_substream *substream,
				       struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = asoc_substream_to_rtd(substream);
	struct snd_soc_dai *cpu_dai = asoc_rtd_to_cpu(rtd, 0);
	struct snd_soc_dai *codec_dai = asoc_rtd_to_codec(rtd, 0);
	unsigned int sample_bits = snd_pcm_format_physical_width(params_format(params));
	struct snd_soc_card *card = rtd->card;
	struct pisound_nau8325_dev *dev_nau8325 = snd_soc_card_get_drvdata(card);
	int ret;

	ret = snd_soc_dai_set_bclk_ratio(cpu_dai, sample_bits * 2);
	if (ret < 0)
		dev_err(rtd->dev, "can't set BCLK ratio: %d\n", ret);

	if (dev_nau8325->mclk_gpclk) {
		unsigned long mclk_gpclk_rate;
		
		dev_nau8325->mclk_rate = params_rate(params) * 256;
		mclk_gpclk_rate = clk_round_rate(dev_nau8325->mclk_gpclk, dev_nau8325->mclk_rate);
		
		ret = clk_set_rate(dev_nau8325->mclk_gpclk, mclk_gpclk_rate);
		if (ret) {
			goto err;
		}
	}

	ret = snd_soc_dai_set_sysclk(codec_dai, 0,
		dev_nau8325->mclk_rate, SND_SOC_CLOCK_IN);
	if (ret < 0)
		dev_err(rtd->dev, "can't set FS clock %d\n", ret);

err:
	return ret;
}

/* machine stream operations */
static struct snd_soc_ops pisound_nau8325_ops = {
	.startup = pisound_nau8325_startup,
	.hw_params = pisound_nau8325_hw_params,
};

SND_SOC_DAILINK_DEFS(nau8325_link,
	DAILINK_COMP_ARRAY(COMP_CPU("bcm2835-i2s.0")),
	DAILINK_COMP_ARRAY(COMP_CODEC("nau8325.1-0021", "nau8325-hifi")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("bcm2835-i2s.0")));

static struct snd_soc_dai_link pisound_nau8325_dai[] = {
	{
		.name = "pisound nau8325",
		.stream_name = "pisound nau8325",
		.ops = &pisound_nau8325_ops,
#if 1 // slave mode
		.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS,
#else // master mode
		.dai_fmt = SND_SOC_DAIFMT_CBM_CFM|SND_SOC_DAIFMT_I2S|SND_SOC_DAIFMT_NB_NF,
#endif
		SND_SOC_DAILINK_REG(nau8325_link),
	},
};

static const struct snd_kcontrol_new pisound_nau8325_controls[] = {
	SOC_DAPM_PIN_SWITCH("Ext Spk"),
};

static const struct snd_soc_dapm_widget pisound_nau8325_dapm_widgets[] = {
	SND_SOC_DAPM_SPK("Ext Spk", NULL),
};

static const struct snd_soc_dapm_route pisound_nau8325_audio_map[] = {
	{"Ext Spk", NULL, "SPKL"},
	{"Ext Spk", NULL, "SPKR"},
};


static struct snd_soc_card snd_soc_pisound_nau8325 = {
	.name = "pisoundnau8325",
	.owner = THIS_MODULE,

	.dai_link = pisound_nau8325_dai,
	.num_links = ARRAY_SIZE(pisound_nau8325_dai),

	.controls = pisound_nau8325_controls,
	.num_controls = ARRAY_SIZE(pisound_nau8325_controls),
	.dapm_widgets = pisound_nau8325_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(pisound_nau8325_dapm_widgets),
	.dapm_routes = pisound_nau8325_audio_map,
	.num_dapm_routes = ARRAY_SIZE(pisound_nau8325_audio_map),
};

static int pisound_nau8325_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = &snd_soc_pisound_nau8325;
	struct pisound_nau8325_dev *dev_nau8325;
	unsigned long mclk_gpclk_rate;
	int ret;

	card->dev = &pdev->dev;
	dev_nau8325 = devm_kzalloc(&pdev->dev, sizeof(*dev_nau8325), GFP_KERNEL);
	if (!dev_nau8325)
		return -ENOMEM;
	dev_nau8325->card = card;

	if (pdev->dev.of_node) {
		struct snd_soc_dai_link *dai = &pisound_nau8325_dai[0];
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

	if ((ret = snd_soc_register_card(card))) {
		dev_err(&pdev->dev, "snd_soc_register_card failed (%d)\n", ret);
	} else {
		int ret, clock_rate = 0;

		dev_nau8325->mclk_gpclk =
			devm_clk_get(card->dev, NULL);
		if (IS_ERR(dev_nau8325->mclk_gpclk)) {
			dev_info(card->dev, "No 'mclk_gpclk' clock found");
			dev_nau8325->mclk_gpclk = NULL;
		}

		ret = device_property_read_u32(card->dev, "nuvoton,clock-rates", &clock_rate);
		if (ret)
			dev_nau8325->mclk_rate = BCM2835_CLK_SRC_GPCLK1;
		else
			dev_nau8325->mclk_rate = clock_rate;

		dev_info(card->dev, "assign MCLK rate: %luHz\n", dev_nau8325->mclk_rate);
				
		if (dev_nau8325->mclk_gpclk) {
			mclk_gpclk_rate = clk_round_rate(dev_nau8325->mclk_gpclk,
				dev_nau8325->mclk_rate);
			ret = clk_set_rate(dev_nau8325->mclk_gpclk,
				mclk_gpclk_rate);
			if (ret) {
				dev_err(card->dev, "Unable to set mclk_gpclk rate (%d)\n",
					ret);
				goto clk_err;
			}
#if 1
			ret = clk_prepare_enable(dev_nau8325->mclk_gpclk);
			if (ret) {
				dev_err(card->dev, "Unable to prepare mclk_gpclk (%d)\n",
					ret);
				goto clk_err;
			}
			mclk_gpclk_rate = clk_get_rate(dev_nau8325->mclk_gpclk);
			dev_dbg(card->dev, "Updated mclk_gpclk %luHz\n",
				mclk_gpclk_rate);
#endif
		}
		snd_soc_card_set_drvdata(card, dev_nau8325);
	}

clk_err:

	return ret;
}

static int pisound_nau8325_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);
	snd_soc_unregister_card(card);
	return 0;
}

static const struct of_device_id pisound_nau8325_of_match[] = {
	{ .compatible = "nuvoton,pisound-nau8325", },
	{},
};
MODULE_DEVICE_TABLE(of, pisound_nau8325_of_match);

static struct platform_driver pisound_nau8325_driver = {
	.driver = {
		.name   = "snd-pisound-nau8325",
		.owner  = THIS_MODULE,
		.of_match_table = pisound_nau8325_of_match,
	},
	.probe = pisound_nau8325_probe,
	.remove = pisound_nau8325_remove,
};
module_platform_driver(pisound_nau8325_driver);

MODULE_DESCRIPTION("NAU8325 Pi Sound Card");
MODULE_AUTHOR("Seven Lee <wtli@nuvoton.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:pisound-nau8325");

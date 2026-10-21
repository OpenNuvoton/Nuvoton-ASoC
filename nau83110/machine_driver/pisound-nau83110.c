 // SPDX-License-Identifier: GPL-2.0
 /*
  * ASoC machine driver for soundcard NAU83110 in Raspberry Pi 4
  *
  * Copyright (c) 2025 Nuvoton Technology Corp.
  * Author: Neo Chang <YLCHANG2@nuvoton.com>
  */
#include <linux/module.h>
#include <linux/types.h>
#include <linux/input.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <sound/core.h>
#include <sound/soc.h>
#include <sound/pcm_params.h>
#include <sound/control.h>
#include "../codecs/nau83110.h"

#define BCM2835_CLK_SRC_12288K_GPCLK1 12288000
#define BCM2835_CLK_SRC_GPCLK2 50000000

#define NOT_4_CUST 1

struct pisound_nau83110_dev {
	struct snd_soc_card *card;
	struct clk *mclk_gpclk;
	unsigned long mclk_rate;
	bool clk_enable;
};

static int pisound_nau83110_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);
	struct snd_soc_card *card = rtd->card;
	struct pisound_nau83110_dev *snd_nau83110 = snd_soc_card_get_drvdata(card);
	int ret;

#ifdef NOT_4_CUST
	ret = snd_soc_dai_set_bclk_ratio(cpu_dai, 256); // BCLK 12.288MHz
	if (ret)
		dev_err(rtd->dev, "can't set BCLK ratio: %d", ret);
#endif
	dev_dbg(card->dev, "assign mclk %luHz", snd_nau83110->mclk_rate );

	return ret;
}

/* machine stream operations */
static struct snd_soc_ops pisound_nau83110_ops = {
	.hw_params = pisound_nau83110_hw_params,
};

SND_SOC_DAILINK_DEFS(nau83110_link,
	DAILINK_COMP_ARRAY(COMP_CPU("bcm2835-i2s.0")),
	DAILINK_COMP_ARRAY(COMP_CODEC("nau83110.1-005b", "nau83110-hifi")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("bcm2835-i2s.0")));

static struct snd_soc_dai_link pisound_nau83110_dai[] = {
	{
		.name = "pisound nau83110",
		.stream_name = "pisound nau83110",
		.ops = &pisound_nau83110_ops,
		.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS,
		SND_SOC_DAILINK_REG(nau83110_link),
	},
};

static const struct snd_soc_dapm_widget pisound_nau83110_dapm_widgets[] = {
	SND_SOC_DAPM_SPK("Speaker", NULL),
};

static const struct snd_soc_dapm_route pisound_nau83110_audio_map[] = {
	{ "Speaker", NULL, "SPKP"},
	{ "Speaker", NULL, "SPKN"},
};

static struct snd_soc_card snd_soc_pisound_nau83110 = {
	.name = "pisoundnau83110",
	.owner = THIS_MODULE,
	.dai_link = pisound_nau83110_dai,
	.num_links = ARRAY_SIZE(pisound_nau83110_dai),
	.dapm_widgets = pisound_nau83110_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(pisound_nau83110_dapm_widgets),
	.dapm_routes = pisound_nau83110_audio_map,
	.num_dapm_routes = ARRAY_SIZE(pisound_nau83110_audio_map),
};

static int pisound_nau83110_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = &snd_soc_pisound_nau83110;
	struct pisound_nau83110_dev *snd_nau83110;
	unsigned long mclk_gpclk_rate;
	int ret;

	card->dev = &pdev->dev;
	snd_nau83110 = devm_kzalloc(&pdev->dev, sizeof(*snd_nau83110), GFP_KERNEL);
	if (!snd_nau83110)
		return -ENOMEM;
	snd_nau83110->card = card;

	if (pdev->dev.of_node) {
		struct snd_soc_dai_link *dai = &pisound_nau83110_dai[0];
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

	snd_soc_card_set_drvdata(card, snd_nau83110);
	ret = devm_snd_soc_register_card(&pdev->dev, &snd_soc_pisound_nau83110);
	if (ret && ret != -EPROBE_DEFER) {
		dev_err(&pdev->dev, "Failed to register card %d\n", ret);
	} else {
		int ret, clock_rate = 0;

		snd_nau83110->mclk_gpclk = devm_clk_get(card->dev, NULL);
		if (IS_ERR(snd_nau83110->mclk_gpclk)) {
			dev_err(card->dev, "Unable to get mclk_gpclk");
			snd_nau83110->mclk_gpclk = NULL;
			goto clk_err;
		}

		ret = device_property_read_u32(card->dev, "nuvoton,clock-rates", &clock_rate);
		if (ret) {
			snd_nau83110->mclk_rate = BCM2835_CLK_SRC_12288K_GPCLK1;
		} else {
			snd_nau83110->mclk_rate = clock_rate;
		}

		if (snd_nau83110->mclk_gpclk) {
			mclk_gpclk_rate = clk_round_rate(snd_nau83110->mclk_gpclk,
				snd_nau83110->mclk_rate);
			ret = clk_set_rate(snd_nau83110->mclk_gpclk, mclk_gpclk_rate);
			if (ret) {
				dev_err(card->dev, "Unable to set mclk_gpclk rate (%d)",
					ret);
				goto clk_err;
			}
		}
		snd_soc_card_set_drvdata(card, snd_nau83110);
	}

clk_err:

	return ret;
}

static int pisound_nau83110_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);

	snd_soc_unregister_card(card);

	return 0;
}

static const struct of_device_id pisound_nau83110_of_match[] = {
	{ .compatible = "nuvoton,pisound-nau83110", },
	{},
};
MODULE_DEVICE_TABLE(of, pisound_nau83110_of_match);

static struct platform_driver pisound_nau83110_driver = {
	.driver = {
		.name   = "snd-pisound-nau83110",
		.owner  = THIS_MODULE,
		.of_match_table = pisound_nau83110_of_match,
	},
	.probe = pisound_nau83110_probe,
	.remove = pisound_nau83110_remove,
};
module_platform_driver(pisound_nau83110_driver);

MODULE_DESCRIPTION("NAU83110 Pi Soundcard");
MODULE_AUTHOR("Neo Chang <YLCHANG2@nuvoton.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:pisound-nau83110");
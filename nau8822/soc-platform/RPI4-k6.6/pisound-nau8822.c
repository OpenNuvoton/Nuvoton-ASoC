 // SPDX-License-Identifier: GPL-2.0
 /*
  * ASoC machine driver for soundcard NAU88C22 in Raspberry Pi 4
  *
  * Copyright (c) 2025 Nuvoton Technology Corp.
  * Author: John Hsu <kchsu0@nuvoton.com>
  */
#define DEBUG 1

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
#include "../codecs/nau8822.h"

#define NAU8822_DEF_MCLK_SRC 12000000

struct pisound_nau8822_dev {
	struct snd_soc_card *card;
	unsigned long mclk_rate;
};

static int pisound_nau8822_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = asoc_substream_to_rtd(substream);
	struct snd_soc_dai *cpu_dai = asoc_rtd_to_cpu(rtd, 0);
	struct snd_soc_dai *codec_dai = asoc_rtd_to_codec(rtd, 0);
	struct snd_soc_card *card = rtd->card;
	struct pisound_nau8822_dev *snd_nau8822 = snd_soc_card_get_drvdata(card);
	int ret;

	dev_dbg(card->dev, "assign mclk %luHz", snd_nau8822->mclk_rate );

	ret = snd_soc_dai_set_sysclk(codec_dai, NAU8822_CLK_PLL,
		snd_nau8822->mclk_rate, SND_SOC_CLOCK_IN);
	if (ret)
		dev_err(rtd->dev, "set codec sysclk fail %d", ret);

	ret = snd_soc_dai_set_pll(codec_dai, 0, 0, snd_nau8822->mclk_rate,
		params_rate(params) * 256);
	if (ret)
		dev_err(rtd->dev, "set codec pll fail %d", ret);

	return ret;
}

/* machine stream operations */
static struct snd_soc_ops pisound_nau8822_ops = {
	.hw_params = pisound_nau8822_hw_params,
};

static int pisound_nau8822_dai_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_card *card = rtd->card;
	struct snd_soc_dapm_context *dapm = &card->dapm;
	struct pisound_nau8822_dev *snd_nau8822 = snd_soc_card_get_drvdata(card);
	int ret, clock_rate = 0;

	snd_soc_dapm_disable_pin(dapm, "RMICN");
	snd_soc_dapm_disable_pin(dapm, "RMICP"),
	snd_soc_dapm_disable_pin(dapm, "LAUX"),
	snd_soc_dapm_disable_pin(dapm, "RAUX"),
	snd_soc_dapm_disable_pin(dapm, "L2"),
	snd_soc_dapm_disable_pin(dapm, "R2"),
	snd_soc_dapm_disable_pin(dapm, "LSPK"),
	snd_soc_dapm_disable_pin(dapm, "RSPK"),
	snd_soc_dapm_disable_pin(dapm, "AUXOUT1"),
	snd_soc_dapm_disable_pin(dapm, "AUXOUT2"),

	ret = device_property_read_u32(card->dev, "nuvoton,clock-rates", &clock_rate);
	if (ret)
		snd_nau8822->mclk_rate = NAU8822_DEF_MCLK_SRC;
	else
		snd_nau8822->mclk_rate = clock_rate;

	dev_dbg(card->dev, "updated mclk %luHz\n", snd_nau8822->mclk_rate);

	return 0;
}

SND_SOC_DAILINK_DEFS(nau8822_link,
	DAILINK_COMP_ARRAY(COMP_CPU("bcm2835-i2s.0")),
	DAILINK_COMP_ARRAY(COMP_CODEC("nau8822.1-001a", "nau8822-hifi")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("bcm2835-i2s.0")));

static struct snd_soc_dai_link pisound_nau8822_dai[] = {
	{
		.name = "pisound nau8822",
		.stream_name = "pisound nau8822",
		.ops = &pisound_nau8822_ops,
		.init = pisound_nau8822_dai_init,
		.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS,
		SND_SOC_DAILINK_REG(nau8822_link),
	},
};

static const struct snd_soc_dapm_widget pisound_nau8822_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Headphones", NULL),
	SND_SOC_DAPM_MIC("Microphone", NULL),
};

static const struct snd_soc_dapm_route pisound_nau8822_audio_map[] = {
	{ "Headphones", NULL, "LHP" },
	{ "Headphones", NULL, "RHP" },
	{ "LMICN", NULL, "Microphone" },
	{ "LMICP", NULL, "Microphone" },
};

static struct snd_soc_card snd_soc_pisound_nau8822 = {
	.name = "pisoundnau8822",
	.owner = THIS_MODULE,
	.dai_link = pisound_nau8822_dai,
	.num_links = ARRAY_SIZE(pisound_nau8822_dai),

	.dapm_widgets = pisound_nau8822_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(pisound_nau8822_dapm_widgets),
	.dapm_routes = pisound_nau8822_audio_map,
	.num_dapm_routes = ARRAY_SIZE(pisound_nau8822_audio_map),
};

static int pisound_nau8822_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = &snd_soc_pisound_nau8822;
	struct pisound_nau8822_dev *snd_nau8822;
	int ret;

	dev_dbg(&pdev->dev, "%s...", __func__);

	card->dev = &pdev->dev;
	snd_nau8822 = devm_kzalloc(&pdev->dev, sizeof(*snd_nau8822), GFP_KERNEL);
	if (!snd_nau8822)
		return -ENOMEM;
	snd_nau8822->card = card;

	if (pdev->dev.of_node) {
		struct snd_soc_dai_link *dai = &pisound_nau8822_dai[0];
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
	snd_soc_card_set_drvdata(card, snd_nau8822);
	ret = devm_snd_soc_register_card(&pdev->dev, &snd_soc_pisound_nau8822);
	if (ret && ret != -EPROBE_DEFER)
		dev_err(&pdev->dev, "Failed to register card %d\n", ret);

	return ret;
}

static int pisound_nau8822_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);

	snd_soc_unregister_card(card);

	return 0;
}

static const struct of_device_id pisound_nau8822_of_match[] = {
	{ .compatible = "nuvoton,pisound-nau8822", },
	{},
};
MODULE_DEVICE_TABLE(of, pisound_nau8822_of_match);

static struct platform_driver pisound_nau8822_driver = {
	.driver = {
		.name   = "snd-pisound-nau8822",
		.owner  = THIS_MODULE,
		.of_match_table = pisound_nau8822_of_match,
	},
	.probe = pisound_nau8822_probe,
	.remove = pisound_nau8822_remove,
};
module_platform_driver(pisound_nau8822_driver);

MODULE_DESCRIPTION("NAU88C22 Pi Soundcard");
MODULE_AUTHOR("John Hsu <kchsu0@nuvoton.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:pisound-nau8822");

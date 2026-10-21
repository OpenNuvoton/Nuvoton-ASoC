 // SPDX-License-Identifier: GPL-2.0
 /*
  * ASoC machine driver for soundcard NAU8421 in Raspberry Pi 3
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
#include "../codecs/nau8421.h"

#define BCM2835_CLK_SRC_12288K_GPCLK1 12288000
#define BCM2835_CLK_SRC_GPCLK2 50000000

//#define NOT_4_CUST 1

struct pisound_nau8421_dev {
	struct snd_soc_card *card;
	struct clk *mclk_gpclk;
	unsigned long mclk_rate;
	bool clk_enable;
};

static const unsigned int bcm2835_rates_12000000[] = {
	8000, 16000, 24000, 32000, 48000, 96000, 176400, 192000
};

static struct snd_pcm_hw_constraint_list bcm2835_constraints_12000000 = {
	.list = bcm2835_rates_12000000,
	.count = ARRAY_SIZE(bcm2835_rates_12000000),
};

static int pisound_nau8421_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = asoc_substream_to_rtd(substream);
	struct snd_soc_card *card = rtd->card;
	struct pisound_nau8421_dev *snd_nau8421 = snd_soc_card_get_drvdata(card);
	int ret;

	/* Setup constraints, because there is a 12 MHz XTAL on the board */
	snd_pcm_hw_constraint_list(substream->runtime, 0,
		SNDRV_PCM_HW_PARAM_RATE, &bcm2835_constraints_12000000);

	if (snd_nau8421->mclk_gpclk && !snd_nau8421->clk_enable) {
                ret = clk_prepare_enable(snd_nau8421->mclk_gpclk);
                if (ret) {
                        dev_err(rtd->dev, "Unable to prepare mclk_gpclk (%d)\n", ret);
                        goto err;
                }
                snd_nau8421->clk_enable = 1;
                dev_dbg(rtd->dev, "MCLK enable; Updated mclk_gpclk %luHz\n",
			clk_get_rate(snd_nau8421->mclk_gpclk));
        }

err:
        return 0;
}

static int pisound_nau8421_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = asoc_substream_to_rtd(substream);
	struct snd_soc_dai *cpu_dai = asoc_rtd_to_cpu(rtd, 0);
	struct snd_soc_dai *codec_dai = asoc_rtd_to_codec(rtd, 0);
	struct snd_soc_card *card = rtd->card;
	struct pisound_nau8421_dev *snd_nau8421 = snd_soc_card_get_drvdata(card);
	int ret;

#ifdef NOT_4_CUST
	ret = snd_soc_dai_set_bclk_ratio(cpu_dai, 250); // BCLK 12MHz
	if (ret)
		dev_err(rtd->dev, "can't set BCLK ratio: %d", ret);
#endif
	dev_dbg(card->dev, "assign mclk %luHz", snd_nau8421->mclk_rate );

	ret = snd_soc_dai_set_sysclk(codec_dai, 0, snd_nau8421->mclk_rate,
		SND_SOC_CLOCK_IN);
	if (ret)
		dev_err(rtd->dev, "set codec sysclk fail %d", ret);

	return ret;
}

static int platform_clock_control(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *k, int event)
{
	struct snd_soc_dapm_context *dapm = w->dapm;
	struct snd_soc_card *card = dapm->card;
	struct pisound_nau8421_dev *snd_nau8421 = snd_soc_card_get_drvdata(card);

	if (!snd_nau8421->mclk_gpclk || !snd_nau8421->clk_enable)
		return 0;

	if (SND_SOC_DAPM_EVENT_OFF(event)) {
		clk_disable_unprepare(snd_nau8421->mclk_gpclk);
		dev_dbg(card->dev, "MCLK disable");
		snd_nau8421->clk_enable = 0;
	}

	return 0;
}

static int pisound_nau8421_mclk_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct pisound_nau8421_dev *snd_nau8421 = snd_soc_card_get_drvdata(card);

	ucontrol->value.integer.value[0] = snd_nau8421->mclk_rate;

	return 0;
}

static int pisound_nau8421_mclk_put(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct pisound_nau8421_dev *snd_nau8421 = snd_soc_card_get_drvdata(card);
	unsigned long mclk_gpclk_rate = ucontrol->value.integer.value[0];
	int ret;

	if (!mclk_gpclk_rate || snd_nau8421->mclk_rate == mclk_gpclk_rate)
		return 0;
	snd_nau8421->mclk_rate = mclk_gpclk_rate;
	if (snd_nau8421->mclk_gpclk) {
		mclk_gpclk_rate = clk_round_rate(snd_nau8421->mclk_gpclk,
			snd_nau8421->mclk_rate);
		ret = clk_set_rate(snd_nau8421->mclk_gpclk, mclk_gpclk_rate);
		if (ret) {
			dev_err(card->dev, "Unable to set mclk_gpclk rate (%d)",
				ret);
			return -EINVAL;;
		}
	}
	dev_dbg(card->dev, "%s: maclk_rate = %d\n", __func__, snd_nau8421->mclk_rate);

	return 1;
}

/* machine stream operations */
static struct snd_soc_ops pisound_nau8421_ops = {
	.startup = pisound_nau8421_startup,
	.hw_params = pisound_nau8421_hw_params,
};

SND_SOC_DAILINK_DEFS(nau8421_link,
	DAILINK_COMP_ARRAY(COMP_CPU("bcm2835-i2s.0")),
	DAILINK_COMP_ARRAY(COMP_CODEC("nau8421.1-004c", "nau8421-hifi")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("bcm2835-i2s.0")));

static int pisound_nau8421_init(struct snd_soc_pcm_runtime *rtd)
{
	dev_dbg(rtd->dev, "pisoundnau8421 nau8421-hifi is initialized");
	return 0;
}

static struct snd_soc_dai_link pisound_nau8421_dai[] = {
	{
		.name = "pisound nau8421",
		.stream_name = "pisound nau8421",
		.ops = &pisound_nau8421_ops,
		.init = pisound_nau8421_init,
		.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS,
		SND_SOC_DAILINK_REG(nau8421_link),
	},
};

static const struct snd_kcontrol_new pisound_nau8421_controls[] = {
	SOC_DAPM_PIN_SWITCH("Left Spk"),
	SOC_DAPM_PIN_SWITCH("Right Spk"),
	SOC_SINGLE_EXT("MCLK rate control", SND_SOC_NOPM, 0, BCM2835_CLK_SRC_GPCLK2, 0,
		pisound_nau8421_mclk_get, pisound_nau8421_mclk_put),
};

static const struct snd_soc_dapm_widget pisound_nau8421_dapm_widgets[] = {
	SND_SOC_DAPM_SPK("Left Spk", NULL),
	SND_SOC_DAPM_SPK("Right Spk", NULL),
	SND_SOC_DAPM_SUPPLY("Platform Clock", SND_SOC_NOPM, 0, 0,
		platform_clock_control, SND_SOC_DAPM_POST_PMD),
};

static const struct snd_soc_dapm_route pisound_nau8421_audio_map[] = {
	{ "Left Spk", NULL, "LOUT" },
	{ "Right Spk", NULL, "ROUT" },
	{ "Left Spk", NULL, "Platform Clock"},
	{ "Right Spk", NULL, "Platform Clock"},
};

static struct snd_soc_card snd_soc_pisound_nau8421 = {
	.name = "pisoundnau8421",
	.owner = THIS_MODULE,
	.dai_link = pisound_nau8421_dai,
	.num_links = ARRAY_SIZE(pisound_nau8421_dai),

	.controls = pisound_nau8421_controls,
	.num_controls = ARRAY_SIZE(pisound_nau8421_controls),
	.dapm_widgets = pisound_nau8421_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(pisound_nau8421_dapm_widgets),
	.dapm_routes = pisound_nau8421_audio_map,
	.num_dapm_routes = ARRAY_SIZE(pisound_nau8421_audio_map),
};

static int pisound_nau8421_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = &snd_soc_pisound_nau8421;
	struct pisound_nau8421_dev *snd_nau8421;
	unsigned long mclk_gpclk_rate;
	int ret;

	//dev_dbg(&pdev->dev, "%s...", __func__);

	card->dev = &pdev->dev;
	snd_nau8421 = devm_kzalloc(&pdev->dev, sizeof(*snd_nau8421), GFP_KERNEL);
	if (!snd_nau8421)
		return -ENOMEM;
	snd_nau8421->card = card;

	if (pdev->dev.of_node) {
		struct snd_soc_dai_link *dai = &pisound_nau8421_dai[0];
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

	snd_soc_card_set_drvdata(card, snd_nau8421);
	ret = devm_snd_soc_register_card(&pdev->dev, &snd_soc_pisound_nau8421);
	if (ret && ret != -EPROBE_DEFER) {
		dev_err(&pdev->dev, "Failed to register card %d\n", ret);
	} else {
		int ret, clock_rate = 0;

		snd_nau8421->mclk_gpclk = devm_clk_get(card->dev, NULL);
		if (IS_ERR(snd_nau8421->mclk_gpclk)) {
			dev_err(card->dev, "Unable to get mclk_gpclk");
			snd_nau8421->mclk_gpclk = NULL;
			goto clk_err;
		}

		ret = device_property_read_u32(card->dev, "nuvoton,clock-rates", &clock_rate);
		if (ret) {
			snd_nau8421->mclk_rate = BCM2835_CLK_SRC_12288K_GPCLK1;
		} else {
			snd_nau8421->mclk_rate = clock_rate;
		}
		//dev_info(card->dev, "assign MCLK rate: %luHz\n", snd_nau8421->mclk_rate);

		if (snd_nau8421->mclk_gpclk) {
			mclk_gpclk_rate = clk_round_rate(snd_nau8421->mclk_gpclk,
				snd_nau8421->mclk_rate);
			ret = clk_set_rate(snd_nau8421->mclk_gpclk, mclk_gpclk_rate);
			if (ret) {
				dev_err(card->dev, "Unable to set mclk_gpclk rate (%d)",
					ret);
				goto clk_err;
			}
		}
		snd_soc_card_set_drvdata(card, snd_nau8421);
	}

clk_err:

	return ret;
}

static int pisound_nau8421_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);

	snd_soc_unregister_card(card);

	return 0;
}

static const struct of_device_id pisound_nau8421_of_match[] = {
	{ .compatible = "nuvoton,pisound-nau8421", },
	{},
};
MODULE_DEVICE_TABLE(of, pisound_nau8421_of_match);

static struct platform_driver pisound_nau8421_driver = {
	.driver = {
		.name   = "snd-pisound-nau8421",
		.owner  = THIS_MODULE,
		.of_match_table = pisound_nau8421_of_match,
	},
	.probe = pisound_nau8421_probe,
	.remove = pisound_nau8421_remove,
};
module_platform_driver(pisound_nau8421_driver);

MODULE_DESCRIPTION("NAU8421 Pi Soundcard");
MODULE_AUTHOR("John Hsu <kchsu0@nuvoton.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:pisound-nau8421");

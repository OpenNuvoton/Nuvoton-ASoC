 // SPDX-License-Identifier: GPL-2.0
 /*
  * ASoC machine driver for soundcard NAU85L42 in Raspberry Pi 4
  *
  * Copyright (c) 2026 Nuvoton Technology Corp.
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
#include "../codecs/nau8542.h"

static struct snd_soc_card snd_soc_pisound_nau8542;

static const unsigned int bcm2835_rates_12000000[] = {
	8000, 16000, 32000, 44100, 48000, 96000, 88200,
};

static struct snd_pcm_hw_constraint_list bcm2835_constraints_12000000 = {
	.list = bcm2835_rates_12000000,
	.count = ARRAY_SIZE(bcm2835_rates_12000000),
};

#define BCM2835_CLK_SRC_12288K_GPCLK1 12288000
#define BCM2835_CLK_SRC_11289K_GPCLK1 11289600
#define TDM_CHAN 4
#define TDM_CLEN 32

enum pisound_nau8542_dai_fmt {
	PISOUND_NAU8542_DAI_I2S,
	PISOUND_NAU8542_DAI_TDM,
};

struct pisound_nau8542_dev {
	struct snd_soc_card *card;
	struct clk *mclk_gpclk;
	unsigned long mclk_rate;
	bool clk_enable;
	unsigned int tdm_slots;
	unsigned int tdm_slot_width;
	unsigned int rx_mask;
	unsigned int tx_mask;
	enum pisound_nau8542_dai_fmt dai_format;
};

static int pisound_nau8540_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = asoc_substream_to_rtd(substream);
	struct snd_soc_card *card = rtd->card;
	struct pisound_nau8542_dev *dev_nau8542 = snd_soc_card_get_drvdata(card);
	int ret;

	/* Setup constraints, because there is a 12 MHz XTAL on the board */
	snd_pcm_hw_constraint_list(substream->runtime, 0,
				SNDRV_PCM_HW_PARAM_RATE,
				&bcm2835_constraints_12000000);
	if (dev_nau8542->mclk_gpclk && !dev_nau8542->clk_enable) {
		ret = clk_prepare_enable(dev_nau8542->mclk_gpclk);
		if (ret) {
			dev_err(rtd->dev, "Unable to prepare mclk_gpclk (%d)\n", ret);
			goto err;
		}
		dev_nau8542->clk_enable = 1;
		dev_dbg(rtd->dev, "MCLK enable; Updated mclk_gpclk %luHz\n", clk_get_rate(dev_nau8542->mclk_gpclk));
	}

err:
	return 0;
}

static int pisound_nau8542_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = asoc_rtd_to_cpu(rtd, 0);
	struct snd_soc_dai *codec_dai = asoc_rtd_to_codec(rtd, 0);
 	struct snd_soc_card *card = rtd->card;
	struct pisound_nau8542_dev *dev_nau8542 = snd_soc_card_get_drvdata(card);
	unsigned int base_mclk = BCM2835_CLK_SRC_12288K_GPCLK1;
	unsigned int ratio;
	int ret = 0;

	if (dev_nau8542->dai_format == PISOUND_NAU8542_DAI_TDM) {
		/* set codec DAI configuration */
		/* TDM 4 slot 24 bit set the Rx and Tx bitmask to
		* 4 active slots as 0xF
		*/
		ret = snd_soc_dai_set_tdm_slot(cpu_dai, 0x0f, 0x0f, dev_nau8542->tdm_slots, dev_nau8542->tdm_slot_width);
		if (ret < 0) {
			dev_err(rtd->dev, "can't set cpu TDM slot %d\n", ret);
			return ret;
		}

		ret = snd_soc_dai_set_tdm_slot(codec_dai, 0x0f, 0, dev_nau8542->tdm_slots, dev_nau8542->tdm_slot_width);
		if (ret < 0) {
			pr_err("can't set codec TDM slot %d\n", ret);
			return ret;
		}
	}

	if (params_rate(params) % 11025 == 0) {
		/* 44.1kHz (44100, 88200...) */
		base_mclk = BCM2835_CLK_SRC_11289K_GPCLK1; 
	}
	dev_nau8542->mclk_rate = base_mclk;

	ratio = base_mclk / params_rate(params);
	ret = snd_soc_dai_set_bclk_ratio(cpu_dai, ratio);
	if (ret < 0) {
		dev_err(rtd->dev, "Failed to set BCLK ratio\n");
	}

	ret = snd_soc_dai_set_sysclk(codec_dai, NAU8542_CLK_MCLK, dev_nau8542->mclk_rate,
		SND_SOC_CLOCK_IN);
	if (ret)
		dev_err(rtd->dev, "set sysclk output %ld, err %d", dev_nau8542->mclk_rate, ret);

	return ret;
}

static int platform_clock_control(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *k, int event)
{
	struct snd_soc_dapm_context *dapm = w->dapm;
	struct snd_soc_card *card = dapm->card;
	struct snd_soc_dai *codec_dai;
	struct pisound_nau8542_dev *dev_nau8542 = snd_soc_card_get_drvdata(card);

	if (!dev_nau8542->mclk_gpclk || !dev_nau8542->clk_enable)
		return 0;

	if (SND_SOC_DAPM_EVENT_OFF(event)) {
		codec_dai = snd_soc_card_get_codec_dai(card, "nau8542-hifi");
		if (!codec_dai) {
			dev_err(card->dev, "codec DAI not found");
			return -EIO;
		}

		clk_disable_unprepare(dev_nau8542->mclk_gpclk);
		dev_dbg(card->dev, "MCLK disable");
		dev_nau8542->clk_enable = 0;
	}

	return 0;
}

/* machine stream operations */
static struct snd_soc_ops pisound_nau8542_ops = {
	.startup = pisound_nau8540_startup,
	.hw_params = pisound_nau8542_hw_params,
};

SND_SOC_DAILINK_DEFS(nau8542_link,
	DAILINK_COMP_ARRAY(COMP_CPU("bcm2835-i2s.0")),
	DAILINK_COMP_ARRAY(COMP_CODEC("nau8542.1-001c", "nau8542-hifi")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("bcm2835-i2s.0")));

static struct snd_soc_dai_link pisound_nau8542_dai[] = {
	{
		.name = "pisound nau8542",
		.stream_name = "pisound nau8542",
		.ops = &pisound_nau8542_ops,
		.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS,
		SND_SOC_DAILINK_REG(nau8542_link),
	},
};

static const struct snd_soc_dapm_widget pisound_nau8542_dapm_widgets[] = {
	SND_SOC_DAPM_MIC("Microphone", NULL),
	SND_SOC_DAPM_SUPPLY("Platform Clock", SND_SOC_NOPM, 0, 0,
		platform_clock_control, SND_SOC_DAPM_POST_PMD),
};

static const struct snd_soc_dapm_route pisound_nau8542_audio_map[] = {
	{"Microphone", NULL, "MIC1"},
	{"Microphone", NULL, "MIC2"},
	{"Microphone", NULL, "MIC3"},
	{"Microphone", NULL, "MIC4"},
	{"Microphone", NULL, "Platform Clock"},
};

static struct snd_soc_card snd_soc_pisound_nau8542 = {
	.name = "pisoundnau8542",
	.owner = THIS_MODULE,
	.dai_link = pisound_nau8542_dai,
	.num_links = ARRAY_SIZE(pisound_nau8542_dai),
	.dapm_widgets = pisound_nau8542_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(pisound_nau8542_dapm_widgets),
	.dapm_routes = pisound_nau8542_audio_map,
	.num_dapm_routes = ARRAY_SIZE(pisound_nau8542_audio_map),
};


static int pisound_nau8542_parse_dai_mode(struct device *dev,
					struct pisound_nau8542_dev *priv)
{
	const char *dai_format;
	int ret;

	ret = device_property_read_string(dev, "dai-format", &dai_format);
	if (ret)
		dai_format = "i2s";

	if (!strcmp(dai_format, "i2s")) {
		priv->dai_format = PISOUND_NAU8542_DAI_I2S;
	} else if (!strcmp(dai_format, "tdm")) {
		priv->dai_format = PISOUND_NAU8542_DAI_TDM;
		pisound_nau8542_dai[0].dai_fmt &= ~SND_SOC_DAIFMT_FORMAT_MASK;
		pisound_nau8542_dai[0].dai_fmt |= SND_SOC_DAIFMT_DSP_B;
	} else {
		return dev_err_probe(dev, -EINVAL,
			"unsupported audio dai_format: %s\n",
			dai_format);
	}

	dev_info(dev, "audio dai_format: %s\n", dai_format);

	return 0;
}

static int pisound_nau8542_parse_tdm_slots(struct device *dev,
					   struct pisound_nau8542_dev *priv)
{
	u32 val;
	int ret;

	ret = device_property_read_u32(dev, "dai-tdm-slot-num", &val);
	if (ret)
		val = TDM_CHAN;

	if (!val || val > NAU8542_TDM_MAX_CHAN)
		return dev_err_probe(dev, -EINVAL,
				     "invalid dai-tdm-slot-num: %u\n", val);

	priv->tdm_slots = val;

	ret = device_property_read_u32(dev, "dai-tdm-slot-width", &val);
	if (ret)
		val = TDM_CLEN;

	switch (val) {
	case 16:
	case 24:
	case 32:
		break;
	default:
		return dev_err_probe(dev, -EINVAL,
				     "invalid dai-tdm-slot-width: %u\n", val);
	}

	priv->tdm_slot_width = val;

	ret = device_property_read_u32(dev, "dai-tdm-slot-tx-mask", &val);
	if (ret)
		priv->tx_mask = 0;//Default Disable TX
	else
		priv->tx_mask = val;

	ret = device_property_read_u32(dev, "dai-tdm-slot-rx-mask", &val);
	if (ret)
		priv->rx_mask = 0;//Default Disable RX
	else
		priv->rx_mask = val;

	dev_info(dev,
		 "TDM slots: %u, slot width: %u, tx_mask: 0x%x, rx_mask: 0x%x\n",
		 priv->tdm_slots,
		 priv->tdm_slot_width,
		 priv->tx_mask,
		 priv->rx_mask);

	return 0;
}

static int pisound_nau8542_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = &snd_soc_pisound_nau8542;
	struct pisound_nau8542_dev *snd_nau8542;
	unsigned long mclk_gpclk_rate;
	int ret;

	card->dev = &pdev->dev;
	snd_nau8542 = devm_kzalloc(&pdev->dev, sizeof(*snd_nau8542), GFP_KERNEL);
	if (!snd_nau8542)
		return -ENOMEM;
	snd_nau8542->card = card;

	if (pdev->dev.of_node) {
		struct snd_soc_dai_link *dai = &pisound_nau8542_dai[0];
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

	ret = pisound_nau8542_parse_dai_mode(&pdev->dev, snd_nau8542);
	if (ret)
		return ret;

	ret = pisound_nau8542_parse_tdm_slots(&pdev->dev, snd_nau8542);
	if (ret)
		return ret;

	if ((ret = snd_soc_register_card(card))) {
		dev_err(&pdev->dev, "snd_soc_register_card failed (%d)\n", ret);
	} else {
		int ret, clock_rate = 0;

		snd_nau8542->mclk_gpclk = devm_clk_get(card->dev, NULL);
		if (IS_ERR(snd_nau8542->mclk_gpclk)) {
			dev_err(card->dev, "Unable to get mclk_gpclk");
			snd_nau8542->mclk_gpclk = NULL;
			goto clk_err;
		}

		ret = device_property_read_u32(card->dev, "nuvoton,clock-rates", &clock_rate);
		if (ret) {
			snd_nau8542->mclk_rate = BCM2835_CLK_SRC_12288K_GPCLK1;
		} else {
			snd_nau8542->mclk_rate = clock_rate;
		}

		if (snd_nau8542->mclk_gpclk) {
			mclk_gpclk_rate = clk_round_rate(snd_nau8542->mclk_gpclk,
				snd_nau8542->mclk_rate);
			ret = clk_set_rate(snd_nau8542->mclk_gpclk, mclk_gpclk_rate);
			if (ret) {
				dev_err(card->dev, "Unable to set mclk_gpclk rate (%d)",
					ret);
				goto clk_err;
			}
		}
		snd_soc_card_set_drvdata(card, snd_nau8542);
	}

clk_err:

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
MODULE_AUTHOR("Neo Chang <YLCHANG2@nuvoton.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:pisound-nau8542");
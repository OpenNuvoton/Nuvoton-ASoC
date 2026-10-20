 // SPDX-License-Identifier: GPL-2.0
 /*
  * ASoC Driver for Raspberry Pi add on soundcard
  *
  * Copyright (c) 2022 Nuvoton Technology Corp.
  * Author: Seven Lee <wtli@nuvoton.com>
  *        David Lin <ctlin0@nuvoton.com>
  *
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
#include "../codecs/nau8360.h"

#define BCM2835_CLK_SRC_12288K_GPCLK1 12288000
#define NAU8360_DSP_OP_CLK 122880000
#define COMPONENT_NAME	"nau8360.1-001a"
#undef PLL_BCLK_64FS
#define BCLK_64FS 3072000
#ifdef BCLK_256FS
	#define TDM_CHAN 8 //use 2*n, n: 1~4
#else
	#define TDM_CHAN 4 //use 2*n, n: 1~2
#endif
#define TDM_CLEN 32

enum pisound_nau8360_dai_fmt {
	PISOUND_NAU8360_DAI_I2S,
	PISOUND_NAU8360_DAI_TDM,
};

struct pisound_nau8360_dev {
	struct snd_soc_card *card;
	struct clk *mclk_gpclk;
	unsigned long mclk_rate;
	bool clk_enable;
	unsigned int tdm_slots;
	unsigned int tdm_slot_width;
	unsigned int rx_mask;
	unsigned int tx_mask;
	enum pisound_nau8360_dai_fmt dai_format;
};

static struct snd_soc_card snd_soc_pisound_nau8360;

static const unsigned int bcm2835_rates_12000000[] = {
	44100, 48000,
};

static struct snd_pcm_hw_constraint_list bcm2835_constraints_12000000 = {
	.list = bcm2835_rates_12000000,
	.count = ARRAY_SIZE(bcm2835_rates_12000000),
};

static int pisound_nau8360_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = asoc_substream_to_rtd(substream);
	struct snd_soc_card *card = rtd->card;
	struct pisound_nau8360_dev *dev_nau8360 = snd_soc_card_get_drvdata(card);
	int ret;

	/* Setup constraints, because there is a 12 MHz XTAL on the board */
	snd_pcm_hw_constraint_list(substream->runtime, 0,
		SNDRV_PCM_HW_PARAM_RATE, &bcm2835_constraints_12000000);

	if (dev_nau8360->mclk_gpclk && !dev_nau8360->clk_enable) {
                ret = clk_prepare_enable(dev_nau8360->mclk_gpclk);
                if (ret) {
                        dev_err(rtd->dev, "Unable to prepare mclk_gpclk (%d)\n", ret);
                        goto err;
                }
                dev_nau8360->clk_enable = 1;
                dev_dbg(rtd->dev, "MCLK enable; Updated mclk_gpclk %luHz\n", clk_get_rate(dev_nau8360->mclk_gpclk));
        }

err:
        return 0;
}

static int pisound_nau8360_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	#define NAU8360_FS 48000 //sysclk of G60 locked as 48kHz

	struct snd_soc_pcm_runtime *rtd = asoc_substream_to_rtd(substream);
	struct snd_soc_dai *cpu_dai = asoc_rtd_to_cpu(rtd, 0);
	struct snd_soc_dai *codec_dai = asoc_rtd_to_codec(rtd, 0);
 	struct snd_soc_card *card = rtd->card;
	struct pisound_nau8360_dev *dev_nau8360 = snd_soc_card_get_drvdata(card);
	int ret, sys_clk, bclk;

	if (dev_nau8360->dai_format == PISOUND_NAU8360_DAI_I2S) {
		/* If format is TDM I2S, the max channel length is 32 bits, and max BCLK is 64fs (3.072MHz).
		 * If BCLK is beyond 64fs, ex. 128fs, format should change to TDM.
		 */
		ret = snd_soc_dai_set_bclk_ratio(cpu_dai, TDM_CLEN * 2);//64fs
		if (ret)
			dev_err(rtd->dev, "set BCLK ratio err %d", ret);
		bclk = params_rate(params) * TDM_CLEN * 2;
	} else {
		/* If format is TDM, 8 slots, the max channel length is 32 bits,
		 * and max BCLK is 256fs (12.288MHz).
		 */
		ret = snd_soc_dai_set_bclk_ratio(cpu_dai, dev_nau8360->tdm_slot_width * dev_nau8360->tdm_slots);
		if (ret)
			dev_err(rtd->dev, "set BCLK ratio err %d", ret);
		ret = snd_soc_dai_set_tdm_slot(cpu_dai, 0x3, 0x3, dev_nau8360->tdm_slots, dev_nau8360->tdm_slot_width);
		if (ret) {
			dev_err(rtd->dev, "set cpu tdm slot err %d", ret);
			return ret;
		}

		if (dev_nau8360->tx_mask == 0 && dev_nau8360->rx_mask == 0) {
			dev_err(rtd->dev, "Invalid TDM mask from DTS: TX and RX cannot both be 0\n");
			return -EINVAL;
		}

		ret = snd_soc_dai_set_tdm_slot(codec_dai, dev_nau8360->tx_mask, dev_nau8360->rx_mask,
			dev_nau8360->tdm_slots, dev_nau8360->tdm_slot_width);
		if (ret) {
			dev_err(rtd->dev, "set codec tdm slot err %d", ret);
			return ret;
		}
		bclk = params_rate(params) * dev_nau8360->tdm_slot_width * dev_nau8360->tdm_slots;
	}

	/* set Pll source */
#ifdef PLL_BCLK_64FS
	ret = snd_soc_dai_set_pll(codec_dai, 0, NAU8360_PLL_BCLK, BCLK_64FS, NAU8360_DSP_OP_CLK);
#else
	ret = snd_soc_dai_set_pll(codec_dai, 0, NAU8360_PLL_MCLK, dev_nau8360->mclk_rate, NAU8360_DSP_OP_CLK);
#endif
	if (ret)
		dev_err(rtd->dev, "snd_soc_dai_set_pll err %d", ret);
	/* set system clock to meet sysclk >= 2 * bclk */
	sys_clk = BCM2835_CLK_SRC_12288K_GPCLK1;
	if (sys_clk < (2 *  bclk))
		sys_clk = 2 * bclk;

	ret = snd_soc_component_set_sysclk(codec_dai->component, 0, 0, sys_clk,
		SND_SOC_CLOCK_OUT);
	if (ret)
		dev_err(rtd->dev, "set sysclk output %d, err %d", sys_clk, ret);
	ret = snd_soc_component_set_sysclk(codec_dai->component, NAU8360_CLK_ID_DIG,
		NAU8360_CLK_SRC_PLL, NAU8360_DSP_OP_CLK, SND_SOC_CLOCK_IN);
	if (ret)
		dev_err(rtd->dev, "set digital sysclk err %d", ret);
	ret = snd_soc_component_set_sysclk(codec_dai->component, NAU8360_CLK_ID_ANA,
		NAU8360_CLK_SRC_PLL, NAU8360_DSP_OP_CLK, SND_SOC_CLOCK_IN);
	if (ret)
		dev_err(rtd->dev, "set analog sysclk err %d", ret);

	return ret;
}

static int platform_clock_control(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *k, int event)
{
	struct snd_soc_dapm_context *dapm = w->dapm;
	struct snd_soc_card *card = dapm->card;
	struct snd_soc_dai *codec_dai;
	struct pisound_nau8360_dev *dev_nau8360 = snd_soc_card_get_drvdata(card);

	if (!dev_nau8360->mclk_gpclk || !dev_nau8360->clk_enable)
		return 0;

	if (SND_SOC_DAPM_EVENT_OFF(event)) {
		codec_dai = snd_soc_card_get_codec_dai(card, "nau8360-hifi");
		if (!codec_dai) {
			dev_err(card->dev, "codec DAI not found");
			return -EIO;
		}
		snd_soc_component_set_sysclk(codec_dai->component, NAU8360_CLK_ID_INT,
			0, 0, SND_SOC_CLOCK_IN);

		clk_disable_unprepare(dev_nau8360->mclk_gpclk);
		dev_dbg(card->dev, "MCLK disable");
		dev_nau8360->clk_enable = 0;
	}

	return 0;
}

/* machine stream operations */
static struct snd_soc_ops pisound_nau8360_ops = {
	.startup = pisound_nau8360_startup,
	.hw_params = pisound_nau8360_hw_params,
};

SND_SOC_DAILINK_DEFS(nau8360_link,
	DAILINK_COMP_ARRAY(COMP_CPU("bcm2835-i2s.0")),
	DAILINK_COMP_ARRAY(COMP_CODEC("nau8360.1-001a", "nau8360-hifi")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("bcm2835-i2s.0")));

static struct snd_soc_dai_link pisound_nau8360_dai[] = {
	{
		.name = "pisound nau8360",
		.stream_name = "pisound nau8360",
		.ops = &pisound_nau8360_ops,
		// slave mode
		.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBC_CFC | SND_SOC_DAIFMT_CONT,
		SND_SOC_DAILINK_REG(nau8360_link),
	},
};

static const struct snd_kcontrol_new pisound_nau8360_controls[] = {
	SOC_DAPM_PIN_SWITCH("Left Spk"),
	SOC_DAPM_PIN_SWITCH("Right Spk"),
};

static const struct snd_soc_dapm_widget pisound_nau8360_dapm_widgets[] = {
	SND_SOC_DAPM_SPK("Left Spk", NULL),
	SND_SOC_DAPM_SPK("Right Spk", NULL),
	SND_SOC_DAPM_SUPPLY("Platform Clock", SND_SOC_NOPM, 0, 0,
		platform_clock_control, SND_SOC_DAPM_POST_PMD),
};

static const struct snd_soc_dapm_route pisound_nau8360_audio_map[] = {
	{ "Left Spk", NULL, "OUTL" },
	{ "Right Spk", NULL, "OUTR" },
	{ "Left Spk", NULL, "Platform Clock"},
	{ "Right Spk", NULL, "Platform Clock"},
};

static struct snd_soc_card snd_soc_pisound_nau8360 = {
	.name = "pisoundnau8360",
	.owner = THIS_MODULE,

	.dai_link = pisound_nau8360_dai,
	.num_links = ARRAY_SIZE(pisound_nau8360_dai),

	.controls = pisound_nau8360_controls,
	.num_controls = ARRAY_SIZE(pisound_nau8360_controls),
	.dapm_widgets = pisound_nau8360_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(pisound_nau8360_dapm_widgets),
	.dapm_routes = pisound_nau8360_audio_map,
	.num_dapm_routes = ARRAY_SIZE(pisound_nau8360_audio_map),
};

static int pisound_nau8360_parse_dai_mode(struct device *dev,
					struct pisound_nau8360_dev *priv)
{
	const char *dai_format;
	int ret;

	ret = device_property_read_string(dev, "dai-format", &dai_format);
	if (ret)
		dai_format = "i2s";

	if (!strcmp(dai_format, "i2s")) {
		priv->dai_format = PISOUND_NAU8360_DAI_I2S;
	} else if (!strcmp(dai_format, "tdm")) {
		priv->dai_format = PISOUND_NAU8360_DAI_TDM;
		pisound_nau8360_dai[0].dai_fmt &= ~SND_SOC_DAIFMT_FORMAT_MASK;
		pisound_nau8360_dai[0].dai_fmt |= SND_SOC_DAIFMT_DSP_A;
	} else {
		return dev_err_probe(dev, -EINVAL,
				     "unsupported audio dai_format: %s\n",
				     dai_format);
	}

	dev_info(dev, "audio dai_format: %s\n", dai_format);

	return 0;
}

static int pisound_nau8360_parse_tdm_slots(struct device *dev,
					   struct pisound_nau8360_dev *priv)
{
	u32 val;
	int ret;

	ret = device_property_read_u32(dev, "dai-tdm-slot-num", &val);
	if (ret)
		val = TDM_CHAN;

	if (!val || val > NAU8360_TDM_MAX_CHAN)
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
		priv->rx_mask = 0x3;//Default Enable RX DAC: Slot 0 & 1
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

static int pisound_nau8360_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = &snd_soc_pisound_nau8360;
	struct pisound_nau8360_dev *dev_nau8360;
	unsigned long mclk_gpclk_rate;
	int ret;

	card->dev = &pdev->dev;
	dev_nau8360 = devm_kzalloc(&pdev->dev, sizeof(*dev_nau8360), GFP_KERNEL);
	if (!dev_nau8360)
		return -ENOMEM;
	dev_nau8360->card = card;

	if (pdev->dev.of_node) {
		struct snd_soc_dai_link *dai = &pisound_nau8360_dai[0];
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

	ret = pisound_nau8360_parse_dai_mode(&pdev->dev, dev_nau8360);
	if (ret)
		return ret;

	ret = pisound_nau8360_parse_tdm_slots(&pdev->dev, dev_nau8360);
	if (ret)
		return ret;

	if ((ret = snd_soc_register_card(card))) {
		dev_err(&pdev->dev, "snd_soc_register_card failed (%d)\n", ret);
	} else {
		int ret, clock_rate = 0;

		dev_nau8360->mclk_gpclk = devm_clk_get(card->dev, NULL);
		if (IS_ERR(dev_nau8360->mclk_gpclk)) {
			dev_err(card->dev, "No 'mclk_gpclk' clock found");
			dev_nau8360->mclk_gpclk = NULL;
		}

		ret = device_property_read_u32(card->dev, "nuvoton,clock-rates", &clock_rate);
		if (ret) {
			dev_nau8360->mclk_rate = BCM2835_CLK_SRC_12288K_GPCLK1;
		} else {
			dev_nau8360->mclk_rate = clock_rate;
		}
		dev_info(card->dev, "assign MCLK rate: %luHz\n", dev_nau8360->mclk_rate);

		if (dev_nau8360->mclk_gpclk) {
			mclk_gpclk_rate = clk_round_rate(dev_nau8360->mclk_gpclk, dev_nau8360->mclk_rate);
			ret = clk_set_rate(dev_nau8360->mclk_gpclk,
				mclk_gpclk_rate);
			if (ret) {
				dev_err(card->dev, "Unable to set mclk_gpclk rate (%d)\n",
					ret);
				goto clk_err;
			}
		}
		snd_soc_card_set_drvdata(card, dev_nau8360);
	}

clk_err:

	return ret;
}

static int pisound_nau8360_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);

	snd_soc_unregister_card(card);

	return 0;
}

static const struct of_device_id pisound_nau8360_of_match[] = {
	{ .compatible = "nuvoton,pisound-nau8360", },
	{},
};
MODULE_DEVICE_TABLE(of, pisound_nau8360_of_match);

static struct platform_driver pisound_nau8360_driver = {
	.driver = {
		.name   = "snd-pisound-nau8360",
		.owner  = THIS_MODULE,
		.of_match_table = pisound_nau8360_of_match,
	},
	.probe = pisound_nau8360_probe,
	.remove = pisound_nau8360_remove,
};
module_platform_driver(pisound_nau8360_driver);

MODULE_DESCRIPTION("NAU8360 Pi Sound Card");
MODULE_AUTHOR("Seven Lee <wtli@nuvoton.com>");
MODULE_AUTHOR("David Lin <ctlin0@nuvoton.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:pisound-nau8360");

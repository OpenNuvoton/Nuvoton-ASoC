 // SPDX-License-Identifier: GPL-2.0
 /*
  * ASoC Driver for Raspberry Pi add on soundcard
  *
  * Copyright (c) 2022 Nuvoton Technology Corp.
  * Author: Seven Lee <wtli@nuvoton.com>
  *        David Lin <ctlin0@nuvoton.com>
  *
  */
#include <linux/module.h>
#include <sound/soc.h>

struct pisound_nau8315_dev {
	struct snd_soc_card *card;
};

static struct snd_soc_card snd_soc_pisound_nau8315;

SND_SOC_DAILINK_DEFS(nau8315_link,
	DAILINK_COMP_ARRAY(COMP_CPU("bcm2835-i2s.0")),
	DAILINK_COMP_ARRAY(COMP_CODEC(NULL, "nau8315-hifi")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("bcm2835-i2s.0")));

static struct snd_soc_dai_link pisound_nau8315_dai[] = {
	{
		.name = "pisound nau8315",
		.stream_name = "pisound nau8315",
		// slave mode
		.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBC_CFC | SND_SOC_DAIFMT_CONT,
		SND_SOC_DAILINK_REG(nau8315_link),
	},
};

static struct snd_soc_card snd_soc_pisound_nau8315 = {
	.name = "pisoundnau8315",
	.owner = THIS_MODULE,

	.dai_link = pisound_nau8315_dai,
	.num_links = ARRAY_SIZE(pisound_nau8315_dai),
};

static int pisound_nau8315_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct device_node *codec_np;
	struct snd_soc_card *card = &snd_soc_pisound_nau8315;
	struct pisound_nau8315_dev *dev_nau8315;
	int ret;

	card->dev = &pdev->dev;
	dev_nau8315 = devm_kzalloc(&pdev->dev, sizeof(*dev_nau8315), GFP_KERNEL);
	if (!dev_nau8315)
		return -ENOMEM;
	dev_nau8315->card = card;

	if (pdev->dev.of_node) {
		struct snd_soc_dai_link *dai = &pisound_nau8315_dai[0];
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

	codec_np = of_parse_phandle(np, "nuvoton,codec", 0);
	if (!codec_np) {
		dev_err(&pdev->dev, "Property 'nuvoton,codec' missing or invalid\n");
		return -EINVAL;
	}

	pisound_nau8315_dai[0].codecs->of_node = codec_np;

	ret = devm_snd_soc_register_card(&pdev->dev, card);
	if (ret)
		dev_err(&pdev->dev, "snd_soc_register_card() failed: %d\n", ret);
	
	snd_soc_card_set_drvdata(card, dev_nau8315);

	return ret;
}

static int pisound_nau8315_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);

	snd_soc_unregister_card(card);

	return 0;
}

static const struct of_device_id pisound_nau8315_of_match[] = {
	{ .compatible = "nuvoton,pisound-nau8315", },
	{},
};
MODULE_DEVICE_TABLE(of, pisound_nau8315_of_match);

static struct platform_driver pisound_nau8315_driver = {
	.driver = {
		.name   = "snd-pisound-nau8315",
		.owner  = THIS_MODULE,
		.of_match_table = pisound_nau8315_of_match,
	},
	.probe = pisound_nau8315_probe,
	.remove = pisound_nau8315_remove,
};
module_platform_driver(pisound_nau8315_driver);

MODULE_DESCRIPTION("NAU8315 Pi Sound Card");
MODULE_AUTHOR("Seven Lee <wtli@nuvoton.com>");
MODULE_AUTHOR("David Lin <ctlin0@nuvoton.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:pisound-nau8315");
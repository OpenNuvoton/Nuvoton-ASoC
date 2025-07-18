// SPDX-License-Identifier: GPL-2.0-only
//
// The NAU8421 ALSA SoC audio driver. The codec is a high quality 24-bit stereo DAC
// with 8Vpp differential analog output capability.
//
// Copyright 2025 Nuvoton Technology Crop.
// Author: John Hsu <kchsu0@nuvoton.com>

#define DEBUG

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/initval.h>
#include <sound/tlv.h>
#include "nau8421.h"

/* Range of Master Clock MCLK (Hz) */
#define MASTER_CLK_MIN	2048000
#define MASTER_CLK_MAX	36864000

struct nau8421_clk_attr {
	unsigned int param;
	unsigned int value;
};

static const struct nau8421_clk_attr mclksrc_sel[] = {
	{ 1, 0x0 },
	{ 2, 0x2 },
	{ 3, 0x3 },
	{ 4, 0x4 },
	{ 6, 0x6 },
};

static const struct nau8421_clk_attr mclksps_sel[] = {
	{ 128, 0x1 },
	{ 192, 0x2 },
	{ 256, 0x3 },
	{ 384, 0x4 },
	{ 512, 0x5 },
	{ 768, 0x6 },
	{ 1152, 0x7 },
};

/* fs: upper threshold of frame sync range.
 * vmask:
 *	[1 | 32 fs]
 *	[2 | 64 fs]
 *	[3 | 128 fs]
 *	[4 | 192 fs]
 *	[5 | 256 fs]
 *	[6 | 384 fs]
 */
struct nau8421_osr_attr {
	unsigned int fs;
	unsigned int ratio;
	unsigned int vamask;
};

#define OSR_384FS BIT(6)
#define OSR_256FS BIT(5)
#define OSR_192FS BIT(4)
#define OSR_128FS BIT(3)
#define OSR_64FS BIT(2)
#define OSR_32FS BIT(1)

static const struct nau8421_osr_attr osrate_table[] = {
	{ 32000, 128, OSR_64FS | OSR_128FS },
	{ 32000, 192, OSR_64FS | OSR_192FS },
	{ 32000, 256, OSR_64FS | OSR_128FS | OSR_256FS },
	{ 32000, 384, OSR_64FS | OSR_128FS | OSR_192FS | OSR_384FS },
	{ 32000, 512, OSR_64FS | OSR_128FS | OSR_256FS },
	{ 32000, 768, OSR_64FS | OSR_128FS | OSR_192FS | OSR_256FS | OSR_384FS },
	{ 32000, 1152, OSR_64FS | OSR_128FS | OSR_192FS | OSR_384FS },
	{ 48000, 128, OSR_64FS | OSR_128FS },
	{ 48000, 192, OSR_64FS | OSR_192FS },
	{ 48000, 256, OSR_64FS | OSR_128FS | OSR_256FS },
	{ 48000, 384, OSR_64FS | OSR_128FS | OSR_192FS },
	{ 48000, 512, OSR_64FS | OSR_128FS | OSR_256FS },
	{ 48000, 768, OSR_64FS | OSR_128FS | OSR_192FS | OSR_256FS },
	{ 96000, 128, OSR_32FS | OSR_64FS | OSR_128FS },
	{ 96000, 192, OSR_32FS | OSR_64FS | OSR_192FS },
	{ 96000, 256, OSR_32FS | OSR_64FS | OSR_128FS },
	{ 96000, 384, OSR_32FS | OSR_64FS | OSR_128FS | OSR_192FS },
	{ 192000, 128, OSR_32FS | OSR_64FS },
	{ 192000, 192, OSR_32FS | OSR_64FS },
};

static const struct reg_default nau8421_reg_defaults[] = {
	{ NAU8421_R00_HRESET, 0x00 },
	{ NAU8421_R01_SRESET, 0x00 },
	{ NAU8421_R04_SYSCTL, 0x0c },
	{ NAU8421_R05_MCLKDETDEM, 0x83 },
	{ NAU8421_R06_MUTEFUNC, 0xff },
	{ NAU8421_R07_MUTECTL, 0x20 },
	{ NAU8421_R08_RCHDGAIN, 0xcf },
	{ NAU8421_R09_LCHDGAIN, 0xcf },
	{ NAU8421_R0A_I2SCTL, 0x04 },
	{ NAU8421_R0B_I2SPTLST, 0x00 },
	{ NAU8421_R0C_I2SPTRST, 0x00 },
	{ NAU8421_R0D_DEMPA1L, 0xa4 },
	{ NAU8421_R0E_DEMPA1M, 0x5e },
	{ NAU8421_R0F_DEMPA1H, 0x07 },
	{ NAU8421_R13_DEMPB0L, 0xed },
	{ NAU8421_R14_DEMPB0M, 0x6d },
	{ NAU8421_R15_DEMPB0H, 0x00 },
	{ NAU8421_R16_DEMPB1L, 0xb7 },
	{ NAU8421_R17_DEMPB1M, 0xf0 },
	{ NAU8421_R18_DEMPB1H, 0x07 },
	{ NAU8421_R1B_DEMCTL, 0x00 },
	{ NAU8421_R1D_SDMDTR, 0x00 },
	{ NAU8421_R1E_DEMASWCTL, 0x40 },
	{ NAU8421_R1F_DEMASWTH, 0x33 },
	{ NAU8421_R20_DEMASWTL, 0x31 },
	{ NAU8421_R28_ANACTL7, 0x00 },
	{ NAU8421_R29_ENVREFCTL, 0x00 },
};

static bool nau8421_readable_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case NAU8421_R00_HRESET ... NAU8421_R0F_DEMPA1H:
	case NAU8421_R13_DEMPB0L ... NAU8421_R18_DEMPB1H:
	case NAU8421_R1B_DEMCTL:
	case NAU8421_R1D_SDMDTR ... NAU8421_R20_DEMASWTL:
	case NAU8421_R28_ANACTL7 ... NAU8421_R29_ENVREFCTL:
	case NAU8421_R32_STATUS ... NAU8421_R33_STATUS:
		return true;
	default:
		return false;
	}
}

static bool nau8421_writeable_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case NAU8421_R00_HRESET ... NAU8421_R01_SRESET:
	case NAU8421_R04_SYSCTL ... NAU8421_R0F_DEMPA1H:
	case NAU8421_R13_DEMPB0L ... NAU8421_R18_DEMPB1H:
	case NAU8421_R1B_DEMCTL:
	case NAU8421_R1D_SDMDTR ... NAU8421_R20_DEMASWTL:
	case NAU8421_R28_ANACTL7 ... NAU8421_R29_ENVREFCTL:
		return true;
	default:
		return false;
	}
}

static bool nau8421_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case NAU8421_R00_HRESET ... NAU8421_R03_SIREV:
	case NAU8421_R32_STATUS ... NAU8421_R33_STATUS:
		return true;
	default:
		return false;
	}
}

static int nau8421_mute_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct nau8421 *nau8421 = snd_soc_component_get_drvdata(component);

	ucontrol->value.integer.value[0] = !nau8421->mutei_status;

	return 0;
}

static int nau8421_mute_put(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct nau8421 *nau8421 = snd_soc_component_get_drvdata(component);
	unsigned long mute = ucontrol->value.integer.value[0];

	/* The MUTEI input mutes the left and right DAC output when set to low. */
	if (nau8421->mute_pin) {
		gpiod_set_value(nau8421->mute_pin, mute);
		nau8421->mutei_status = !mute;
	}

	return 0;
}

static const char *const nau8421_mute_enum[] = { "unmute", "mute" };

static const struct soc_enum nau8421_mute_pin_enum =
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(nau8421_mute_enum), nau8421_mute_enum);

static const char *const nau8421_dacl_phase[] = { "Left", "Right" };

static const struct soc_enum nau8421_dacl_chsel_enum =
	SOC_ENUM_SINGLE(NAU8421_R0A_I2SCTL, NAU8421_I2SDOPL_SFT,
		ARRAY_SIZE(nau8421_dacl_phase), nau8421_dacl_phase);

static const char *const nau8421_dacr_phase[] = { "Right", "Left" };

static const struct soc_enum nau8421_dacr_chsel_enum =
	SOC_ENUM_SINGLE(NAU8421_R0A_I2SCTL, NAU8421_I2SDOPR_SFT,
		ARRAY_SIZE(nau8421_dacr_phase), nau8421_dacr_phase);

static const char *const nau8421_osr_list[] = {
	"", "32", "64", "128", "192", "256", "384"
};

static const struct soc_enum nau8421_dac_osr_enum =
	SOC_ENUM_SINGLE(NAU8421_R04_SYSCTL, NAU8421_OVSRATE_SFT,
		ARRAY_SIZE(nau8421_osr_list), nau8421_osr_list);

static const DECLARE_TLV_DB_SCALE(digital_vol_tlv, -9600, 50, 1);

static const struct snd_kcontrol_new nau8421_snd_controls[] = {
	SOC_DOUBLE_R_TLV("Digital Playback Volume", NAU8421_R09_LCHDGAIN,
		NAU8421_R08_RCHDGAIN, 0, 0xf3, 0, digital_vol_tlv),
	SOC_ENUM("DACL Channel Source", nau8421_dacl_chsel_enum),
	SOC_ENUM("DACR Channel Source", nau8421_dacr_chsel_enum),
	SOC_ENUM("DAC Oversampling Rate", nau8421_dac_osr_enum),
	SOC_ENUM_EXT("Mute pin control", nau8421_mute_pin_enum,
		nau8421_mute_get, nau8421_mute_put),
};

static const struct snd_soc_dapm_widget nau8421_dapm_widgets[] = {
	SND_SOC_DAPM_AIF_IN("AIFRX", "Playback", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_DAC("DACL", NULL, NAU8421_R04_SYSCTL, NAU8421_DACLEN_SFT, 0),
	SND_SOC_DAPM_DAC("DACR", NULL, NAU8421_R04_SYSCTL, NAU8421_DACREN_SFT, 0),
	SND_SOC_DAPM_OUTPUT("LOUT"),
	SND_SOC_DAPM_OUTPUT("ROUT"),
};

static const struct snd_soc_dapm_route nau8421_dapm_routes[] = {
	{"DACL", NULL, "AIFRX"},
	{"DACR", NULL, "AIFRX"},
	{"LOUT", NULL, "DACL"},
	{"ROUT", NULL, "DACR"},
};

static int nau8421_set_sysclk(struct snd_soc_dai *dai, int clk_id,
	unsigned int freq, int dir)
{
	struct snd_soc_component *component = dai->component;
	struct nau8421 *nau8421 = snd_soc_component_get_drvdata(component);

	if (freq < MASTER_CLK_MIN || freq > MASTER_CLK_MAX) {
		dev_err(nau8421->dev, "Exceed the clocks range, sysclk %dHz.", freq);
		return -EINVAL;
	}

	nau8421->sysclk = freq;
	dev_dbg(nau8421->dev, "master sysclk %dHz", freq);

	return 0;
}

static int nau8421_set_dai_fmt(struct snd_soc_dai *codec_dai, unsigned int fmt)
{
	struct snd_soc_component *component = codec_dai->component;
	struct nau8421 *nau8421 = snd_soc_component_get_drvdata(component);
	u16 ctrl_val = 0;

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBS_CFS:
		break;
	default:
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		break;
	default:
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		ctrl_val |= NAU8421_I2SFMT_I2S;
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		ctrl_val |= NAU8421_I2SFMT_RJ;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		ctrl_val |= NAU8421_I2SFMT_LJ;
		break;
	case SND_SOC_DAIFMT_DSP_B:
		ctrl_val |= NAU8421_I2SFMT_PCMB;
		break;
	case SND_SOC_DAIFMT_DSP_A:
		ctrl_val |= NAU8421_I2SFMT_PCMA;
		break;
	default:
		return -EINVAL;
	}

	regmap_update_bits(nau8421->regmap, NAU8421_R0A_I2SCTL,
		NAU8421_I2SFMT_MASK, ctrl_val);

	return 0;
}

static int inline nau8421_osr_check(int fs, int ratio, int osrate)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(osrate_table); i++)
		if (osrate_table[i].fs >= fs && osrate_table[i].ratio == ratio) {
			if (osrate_table[i].vamask & osrate)
				return 0;
			else
				return -EINVAL;
		}
	return -EINVAL;
}

static int nau8421_mclk_select(struct nau8421 *nau8421)
{
	int ret, src_i, sps_i, mclk_ratios, mclksrc;

	for (sps_i = ARRAY_SIZE(mclksps_sel) - 1; sps_i >= 0; sps_i--) {
		mclk_ratios = nau8421->fs * mclksps_sel[sps_i].param;
		for (src_i = 0; src_i < ARRAY_SIZE(mclksrc_sel); src_i++) {
			mclksrc = nau8421->sysclk / mclksrc_sel[src_i].param;
			if (mclk_ratios == mclksrc)
				break;
		}
		if (mclk_ratios == mclksrc)
			break;
	}
	if (sps_i < 0) {
		dev_err(nau8421->dev, "MCLK speed check fail");
		ret = -EINVAL;
		goto err;
	}

	ret = nau8421_osr_check(nau8421->fs, mclksps_sel[sps_i].param,
		BIT(nau8421->osr_id));
	if (ret) {
		dev_err(nau8421->dev, "OSR %sFS check fail",
			nau8421_osr_list[nau8421->osr_id]);
		goto err;
	}

	dev_dbg(nau8421->dev, "FS %d MCLK %dFS (MCLKSRC 1/%d); OSR %sFS;", nau8421->fs,
		mclksps_sel[sps_i].param, mclksrc_sel[src_i].param,
		nau8421_osr_list[nau8421->osr_id]);
	regmap_update_bits(nau8421->regmap, NAU8421_R04_SYSCTL, NAU8421_MCLKSRC_MASK,
		(mclksrc_sel[src_i].value << NAU8421_MCLKSRC_SFT));
	regmap_update_bits(nau8421->regmap, NAU8421_R05_MCLKDETDEM, NAU8421_MCLKSPS_MASK,
		mclksps_sel[sps_i].value);

	return 0;
err:
	return ret;
}

static int nau8421_pcm_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	struct snd_soc_component *component = dai->component;
	struct nau8421 *nau8421 = snd_soc_component_get_drvdata(component);
	int ret, ctrl_val = 0, value;

	regmap_read(nau8421->regmap, NAU8421_R04_SYSCTL, &value);
	nau8421->osr_id = (value & NAU8421_OVSRATE_MASK) >> NAU8421_OVSRATE_SFT;
	nau8421->fs = params_rate(params);
	ret = nau8421_mclk_select(nau8421);
	if (ret) {
		dev_err(nau8421->dev, "MCLK setup fail (FS %d, MCLK %d, OSR %s)",
			nau8421->fs, nau8421->sysclk, nau8421_osr_list[nau8421->osr_id]);
		goto err;
	}

	switch (params_width(params)) {
	case 16:
		break;
	case 20:
		ctrl_val |= NAU8421_I2SWD_20;
		break;
	case 24:
		ctrl_val |= NAU8421_I2SWD_24;
		break;
	case 32:
		ctrl_val |= NAU8421_I2SWD_32;
		break;
	default:
		ret = -EINVAL;
		dev_err(nau8421->dev, "PCM data width setup fail");
		goto err;
	}

	regmap_update_bits(nau8421->regmap, NAU8421_R0A_I2SCTL,
		NAU8421_I2SWD_MASK, ctrl_val);

	return 0;
err:
	return ret;
}

#define NAU8421_RATES (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |\
	SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000 |\
	SNDRV_PCM_RATE_88200 | SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_176400 |\
	SNDRV_PCM_RATE_192000)

#define NAU8421_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE | \
	SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE)

static const struct snd_soc_dai_ops nau8421_ops = {
	.set_sysclk = nau8421_set_sysclk,
	.set_fmt = nau8421_set_dai_fmt,
	.hw_params = nau8421_pcm_hw_params,
};

static struct snd_soc_dai_driver nau8421_dai = {
	.name = "nau8421-hifi",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates = NAU8421_RATES,
		.formats = NAU8421_FORMATS,
	},
	.ops = &nau8421_ops,
};

static int __maybe_unused nau8421_suspend(struct snd_soc_component *component)
{
	struct nau8421 *nau8421 = snd_soc_component_get_drvdata(component);

	/* enable clock detection during suspend for power saving */
	if (!nau8421->clock_detection) {
		regmap_update_bits(nau8421->regmap, NAU8421_R29_ENVREFCTL,
			NAU8421_ENVREFEN, 0);
		mdelay(1);
	}
	regcache_cache_only(nau8421->regmap, true);
	regcache_mark_dirty(nau8421->regmap);
	/* set enable pin to low to enter reset state */
	if (nau8421->enable_pin)
		gpiod_set_value(nau8421->enable_pin, 0);

	return 0;
}

static int __maybe_unused nau8421_resume(struct snd_soc_component *component)
{
	struct nau8421 *nau8421 = snd_soc_component_get_drvdata(component);

	/* set enable pin to high to leave reset state */
	if (nau8421->enable_pin) {
		gpiod_set_value(nau8421->enable_pin, 1);
		mdelay(1);
	}
	regcache_cache_only(nau8421->regmap, false);
	regcache_sync(nau8421->regmap);
	/* restore manual mode */
	if (!nau8421->clock_detection)
		regmap_update_bits(nau8421->regmap, NAU8421_R29_ENVREFCTL,
			NAU8421_ENVREFEN, NAU8421_ENVREFEN);

	return 0;
}

static int nau8421_reg_write(void *context, unsigned int reg, unsigned int value)
{
	struct i2c_client *client = context;
	u8 buf[2];
	int ret, count = 0;

	buf[count++] = reg & 0xff;
	buf[count++] = value & 0xff;
	dev_dbg(&client->dev, " %x <= %x", reg, value);

	ret = i2c_master_send(client, buf, count);
	if (ret == count)
		return 0;
	else if (ret < 0)
		return ret;
	else
		return -EIO;
}

static int nau8421_reg_read(void *context, unsigned int reg, unsigned int *value)
{
	struct i2c_client *client = context;
	struct i2c_msg xfer[2];
	u8 val_buf;
	u8 reg_buf;
	int ret;

	reg_buf = (u8)reg;
	xfer[0].addr = client->addr;
	xfer[0].len = sizeof(reg_buf);
	xfer[0].buf = &reg_buf;
	xfer[0].flags = 0;

	xfer[1].addr = client->addr;
	xfer[1].len = sizeof(val_buf);
	xfer[1].buf = &val_buf;
	xfer[1].flags = I2C_M_RD;

	ret = i2c_transfer(client->adapter, xfer, ARRAY_SIZE(xfer));
	if (ret < 0)
		return ret;
	else if (ret != ARRAY_SIZE(xfer))
		return -EIO;

	*value = val_buf;

	return 0;
}

static const struct regmap_config nau8421_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,

	.max_register = NAU8421_REG_MAX,
	.readable_reg = nau8421_readable_reg,
	.writeable_reg = nau8421_writeable_reg,
	.volatile_reg = nau8421_volatile_reg,
	.reg_read = nau8421_reg_read,
	.reg_write = nau8421_reg_write,

	.cache_type = REGCACHE_RBTREE,
	.reg_defaults = nau8421_reg_defaults,
	.num_reg_defaults = ARRAY_SIZE(nau8421_reg_defaults),
};

static const struct snd_soc_component_driver nau8421_component_driver = {
	.suspend		= nau8421_suspend,
	.resume			= nau8421_resume,
	.controls		= nau8421_snd_controls,
	.num_controls		= ARRAY_SIZE(nau8421_snd_controls),
	.dapm_widgets		= nau8421_dapm_widgets,
	.num_dapm_widgets	= ARRAY_SIZE(nau8421_dapm_widgets),
	.dapm_routes		= nau8421_dapm_routes,
	.num_dapm_routes	= ARRAY_SIZE(nau8421_dapm_routes),
	.suspend_bias_off	= 1,
	.idle_bias_on		= 1,
	.use_pmdown_time	= 1,
	.endianness		= 1,
};

#ifdef DEBUG
static ssize_t reg_ctrl_store(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct nau8421 *nau8421 = dev_get_drvdata(dev);
	char reg_char[10], val_char[10];
	long reg, val;

	if (sscanf(buf, "%s %s", reg_char, val_char) == 2) {
		if (!kstrtol(reg_char, 8, &reg) && !kstrtol(val_char, 8, &val)) {
			if (nau8421_writeable_reg(dev, reg)) {
				regmap_write(nau8421->regmap, reg, val);
				return count;
			}
		}
	}

	dev_err(dev, "Format Error!");
	dev_err(dev, "echo [register] [value] > reg_debug");
	dev_err(dev, "register and value are all hexadecimal");
	return -EINVAL;
}

static ssize_t reg_ctrl_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	#define LINE_NUM 16
	struct nau8421 *nau8421 = dev_get_drvdata(dev);
	unsigned int val[NAU8421_REG_MAX + 1] = { 0 }, reg;
	int i, count = 0;

	for (reg = 0; reg <= NAU8421_REG_MAX; reg++) {
		if (!nau8421_readable_reg(dev, reg)) {
			val[reg] = 0;
			continue;
		}
		regmap_read(nau8421->regmap, reg, &val[reg]);
	}

	count += sprintf(buf + count,
			"addr   00   01   02   03   04   05   06   07   08   09   0A   0B   0C   0D   0E   0F\n");
	for (reg = 0; reg <= NAU8421_REG_MAX; reg += LINE_NUM) {
		count += sprintf(buf + count, "0x%02X:  ", reg);
		for (i = 0; i < LINE_NUM; i++) {
			if ((reg + i) > NAU8421_REG_MAX)
				break;
			count += sprintf(buf + count, "%02X   ", val[reg + i]);
		}
		count += sprintf(buf + count, "\n");
	}

	if (count >= PAGE_SIZE)
		count = PAGE_SIZE - 1;

	return count;
}

static DEVICE_ATTR(reg_control, S_IRUGO | S_IWUSR, reg_ctrl_show,
	reg_ctrl_store);

static struct attribute *nau8421_attrs[] = {
	&dev_attr_reg_control.attr,
	NULL,
};

static const struct attribute_group nau8421_attr_group = {
	.name = "reg_debug",
	.attrs = nau8421_attrs,
};

static void remove_sysfs_debug(struct device *dev)
{
	sysfs_remove_group(&dev->kobj, &nau8421_attr_group);
}

static int create_sysfs_debug(struct device *dev)
{
	int ret;

	ret = sysfs_create_group(&dev->kobj, &nau8421_attr_group);
	if (ret)
		remove_sysfs_debug(dev);
	return ret;
}
#endif

static void nau8421_hardreset(struct regmap *regmap)
{
	regmap_write(regmap, NAU8421_R00_HRESET, 0x5A);
}

static void nau8421_softreset(struct regmap *regmap)
{
	regmap_write(regmap, NAU8421_R01_SRESET, 0x5A);
	regmap_write(regmap, NAU8421_R01_SRESET, 0xA5);
}

static void nau8421_init_regs(struct nau8421 *nau8421)
{
	struct regmap *regmap = nau8421->regmap;

	/* OSR64 for common case. Adjust later by user. */
	regmap_update_bits(regmap, NAU8421_R04_SYSCTL, NAU8421_OVSRATE_MASK,
		NAU8421_OVSRATE_64);
	/* DEM DWA enable */
	regmap_update_bits(regmap, NAU8421_R05_MCLKDETDEM, NAU8421_DEMSEL_MASK,
		NAU8421_DEMSEL_DWA);
	/* DEM function auto-switching enable */
	regmap_update_bits(regmap, NAU8421_R1E_DEMASWCTL, NAU8421_DEMASW_EN,
		NAU8421_DEMASW_EN);
	/* cotrol power up references */
	if (nau8421->clock_detection) {
		regmap_write(regmap, NAU8421_R28_ANACTL7, NAU8421_ENOPARCTL |
			NAU8421_ENOPAREN | NAU8421_ENOPALCTL | NAU8421_ENOPALEN |
			NAU8421_RNRCTL | NAU8421_RNREN | NAU8421_RNLCTL | NAU8421_RNLEN);
		regmap_update_bits(regmap, NAU8421_R29_ENVREFCTL, NAU8421_ENVREFCTL |
			NAU8421_ENVREFEN, NAU8421_ENVREFCTL | NAU8421_ENVREFEN);
	} else {
		regmap_update_bits(regmap, NAU8421_R28_ANACTL7, NAU8421_ENOPARCTL |
			NAU8421_ENOPAREN | NAU8421_ENOPALCTL | NAU8421_ENOPALEN |
			NAU8421_RNRCTL | NAU8421_RNREN | NAU8421_RNLCTL | NAU8421_RNLEN,
			NAU8421_ENOPAREN | NAU8421_ENOPALEN | NAU8421_RNRCTL |
			NAU8421_RNREN | NAU8421_RNLCTL | NAU8421_RNLEN);
		regmap_update_bits(regmap, NAU8421_R29_ENVREFCTL, NAU8421_ENVREFCTL |
			NAU8421_ENVREFEN, NAU8421_ENVREFEN);
	}
}

static void nau8421_print_device_properties(struct nau8421 *nau8421)
{
	struct device *dev = nau8421->dev;

	dev_dbg(dev, "enable-gpios:            %d\n", nau8421->enable_pin ? 1 : 0);
	dev_dbg(dev, "mute-gpios:              %d\n", nau8421->mute_pin ? 1 : 0);
	dev_dbg(dev, "clock-det-disable:       %d\n", !nau8421->clock_detection);
}

static int nau8421_read_device_properties(struct device *dev, struct nau8421 *nau8421)
{
	nau8421->enable_pin = devm_gpiod_get_optional(dev, "nuvoton,enable", GPIOD_OUT_HIGH);
	if (IS_ERR(nau8421->enable_pin))
		nau8421->enable_pin = NULL;
	nau8421->mute_pin = devm_gpiod_get_optional(dev, "nuvoton,mute", GPIOD_OUT_HIGH);
	if (IS_ERR(nau8421->mute_pin))
		nau8421->mute_pin = NULL;
	nau8421->clock_detection =
		!device_property_read_bool(dev,	"nuvoton,clock-det-disable");

	return 0;
}

static int nau8421_i2c_probe(struct i2c_client *i2c)
{
	struct device *dev = &i2c->dev;
	struct nau8421 *nau8421 = dev_get_platdata(dev);
	int ret, value;

	if (!nau8421) {
		nau8421 = devm_kzalloc(dev, sizeof(*nau8421), GFP_KERNEL);
		if (!nau8421)
			return -ENOMEM;
	}
	i2c_set_clientdata(i2c, nau8421);

	nau8421->regmap = devm_regmap_init(dev, NULL, i2c, &nau8421_regmap_config);
	if (IS_ERR(nau8421->regmap))
		return PTR_ERR(nau8421->regmap);
	nau8421->dev = dev;

	nau8421_read_device_properties(dev, nau8421);
	nau8421_print_device_properties(nau8421);
	/* Set enable pin to high to leave reset state. The MUTEI input mutes the
	 * left and right DAC output when set to low. The mute_pin is configured as
	 * active low. Initially keep MUTEI high to make DAC output normal.
	 */
	if (nau8421->enable_pin) {
		gpiod_set_value(nau8421->enable_pin, 1);
		mdelay(1);
	}
	if (nau8421->mute_pin)
		gpiod_set_value(nau8421->mute_pin, 0);
	nau8421->mutei_status = 1;

	nau8421_hardreset(nau8421->regmap);
	nau8421_softreset(nau8421->regmap);
	ret = regmap_read(nau8421->regmap, NAU8421_R03_SIREV, &value);
	if (ret) {
		dev_err(dev, "can't read chip id from NAU8421 (%d)", ret);
		return ret;
	}

	nau8421_init_regs(nau8421);
#ifdef DEBUG
	create_sysfs_debug(dev);
#endif

	return devm_snd_soc_register_component(dev,
		&nau8421_component_driver, &nau8421_dai, 1);
}

static const struct i2c_device_id nau8421_i2c_id[] = {
	{ "nau8421", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, nau8421_i2c_id);

#ifdef CONFIG_OF
static const struct of_device_id nau8421_of_match[] = {
	{ .compatible = "nuvoton,nau8421", },
	{ }
};
MODULE_DEVICE_TABLE(of, nau8421_of_match);
#endif

static struct i2c_driver nau8421_i2c_driver = {
	.driver = {
		.name = "nau8421",
		.of_match_table = of_match_ptr(nau8421_of_match),
	},
	.probe_new = nau8421_i2c_probe,
	.id_table = nau8421_i2c_id,
};

module_i2c_driver(nau8421_i2c_driver);

MODULE_DESCRIPTION("ASoC NAU8421 driver");
MODULE_AUTHOR("John Hsu <kchsu0@nuvoton.com>");
MODULE_LICENSE("GPL v2");

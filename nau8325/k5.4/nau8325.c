/* SPDX-License-Identifier: GPL-2.0
 *
 * The NAU8325 is a stereo high efficiency filter-free Class-D audio
 * amplifier driver.
 *
 * Copyright 2020 Nuvoton Technology Crop.
 *  Author: Seven Lee <wtli@nuvoton.com>
 * Copyright 2020 Nuvoton Technology Crop.
 *  Author: David Lin <CTLIN0@nuvoton.com>
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <sound/initval.h>
#include <sound/tlv.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/jack.h>
#include "nau8325.h"

static void nau8325_software_reset(struct regmap *regmap);
static int nau8325_set_sysclk(struct snd_soc_component *component, int clk_id,
	int source, unsigned int freq, int dir);
	
/* Range of Master Clock MCLK (Hz) */
#define MASTER_CLK_MIN 2048000
#define MASTER_CLK_MAX 49152000
/* DSP Clock (Hz) */
#define DSP_CLK_MIN 96000000
#define DSP_CLK_MAX 102400000
#define DSP_MCLK_HI_SPEED 12288000

/* scaling for MCLK source */
#define CLK_PROC_BYPASS (-1)

/* the maximum frequency of CLK_ADC and CLK_DAC */
#define CLK_DA_AD_MAX 6144000

static const struct nau8325_src_attr dsp_src_mult[] = {
	/* param : power of 2 */
	{ 0, 0x3 },
	{ 1, 0x2 },
	{ 2, 0x1 },
	{ 3, 0x0 },
};

static const struct nau8325_src_attr mclk_n1_div[] = {
	{ 3, 0x2 },
	{ 1, 0x0 },
	{ 2, 0x1 },
};

/* over sampling rate */
static const struct nau8325_osr_attr osr_dac_sel[] = {
	{ 64, 2 },	/* OSR 64, SRC 1/4 */
	{ 256, 0 },	/* OSR 256, SRC 1 */
	{ 128, 1 },	/* OSR 128, SRC 1/2 */
	{ 0, 0 },
	{ 32, 3 },	/* OSR 32, SRC 1/8 */
};

static const struct nau8325_src_attr mclk_n2_div[] = {
	/* param : power of 2 */
	{ 0, 0x0 },
	{ 1, 0x1 },
	{ 2, 0x2 },
};

static const struct nau8325_src_attr mclk_src_mult[] = {
	/* param : power of 2 */
	{ 0, 0x1 },
	{ 1, 0x2 },
	{ 2, 0x3 },
};

/* Sample Rate and MCLK_SRC selections */
static const struct nau8325_srate_attr target_srate_table[] = {
	/* { FS, range, max, { MCLK source }, adc_div} */
	{ 48000, 2, true, { 12288000, 19200000, 24000000 }, 0x1 },
	{ 16000, 1, false, { 4096000, 6400000, 8000000 }, 0x0 },
	{ 8000, 0, false, { 2048000, 3200000, 4000000 }, 0x0 },
	{ 64000, 3, false, { 16384000, 25600000, 32000000 }, 0x1 },
	{ 96000, 3, true, { 24576000, 38400000, 48000000 }, (-1) },
	{ 12000, 0, true, { 3072000, 4800000, 6000000 }, 0x0 },
	{ 24000, 1, true, { 6144000, 9600000, 12000000 }, 0x2 },
	{ 32000, 2, false, { 8192000, 12800000, 16000000 }, 0x2 },
};

static const struct reg_default nau8325_reg_defaults[] = {
	{ NAU8325_REG_HARDWARE_RST, 0x0000 },
	{ NAU8325_REG_SOFTWARE_RST, 0x0000 },
	{ NAU8325_REG_DEVICE_ID, 0x21f0 },
	{ NAU8325_REG_CLK_CTRL, 0x0000 },
	{ NAU8325_REG_ENA_CTRL, 0x0000 },
	{ NAU8325_REG_INTERRUPT_CTRL, 0x007f },
	{ NAU8325_REG_INT_CLR_STATUS, 0x0000 },
	{ NAU8325_REG_IRQOUT, 0x0000 },
	{ NAU8325_REG_IO_CTRL, 0x0000 },
	{ NAU8325_REG_I2S_PCM_CTRL0, 0x0000 },
	{ NAU8325_REG_TDM_CTRL, 0x0000 },
	{ NAU8325_REG_I2S_PCM_CTRL1, 0x000a },
	{ NAU8325_REG_I2S_PCM_CTRL2, 0x0000 },
	{ NAU8325_REG_TIME_SLOT, 0x0000 },
	{ NAU8325_REG_HPF_CTRL, 0x0000 },
	{ NAU8325_REG_MUTE_CTRL, 0x0000 },
	{ NAU8325_REG_DAC_VOLUME, 0xf3f3 },
	{ NAU8325_REG_DEBUG_READ1, 0x0000 },
	{ NAU8325_REG_DEBUG_READ2, 0x0000 },
	{ NAU8325_REG_DEBUG_READ3, 0x1f80 },
	{ NAU8325_REG_DAC_CTRL1, 0x0081 },
	{ NAU8325_REG_DAC_CTRL2, 0x0000 },
	{ NAU8325_REG_ALC_CTRL1, 0x000e },
	{ NAU8325_REG_ALC_CTRL2, 0x8400 },
	{ NAU8325_REG_ALC_CTRL3, 0x0000 },
	{ NAU8325_REG_ALC_CTRL4, 0x003f },
	{ NAU8325_REG_CLK_DET_CTRL, 0xa801 },
	{ NAU8325_REG_TEST_STATUS, 0x0000 },
	{ NAU8325_REG_ANALOG_READ, 0x051a },
	{ NAU8325_REG_MIXER_CTRL, 0x0000 },
	{ NAU8325_REG_MISC_CTRL, 0x0000 },
	{ NAU8325_REG_BIAS_ADJ, 0x0000 },
	{ NAU8325_REG_ANALOG_CONTROL_1, 0x0000 },
	{ NAU8325_REG_ANALOG_CONTROL_2, 0x0000 },
	{ NAU8325_REG_ANALOG_CONTROL_3, 0x0000 },
	{ NAU8325_REG_ANALOG_CONTROL_4, 0x0000 },
	{ NAU8325_REG_ANALOG_CONTROL_5, 0x0000 },
	{ NAU8325_REG_ANALOG_CONTROL_6, 0x0000 },
	{ NAU8325_REG_CLIP_CTRL, 0x0000 },
	{ NAU8325_REG_RDAC, 0x0008 },
};

static bool nau8325_readable_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case NAU8325_REG_CLK_CTRL ... NAU8325_REG_INT_CLR_STATUS:
	case NAU8325_REG_IRQOUT:
	case NAU8325_REG_IO_CTRL ... NAU8325_REG_TIME_SLOT:
	case NAU8325_REG_HPF_CTRL ... NAU8325_REG_DAC_VOLUME:
	case NAU8325_REG_DEBUG_READ1:
	case NAU8325_REG_DEBUG_READ2:
	case NAU8325_REG_DEBUG_READ3:
	case NAU8325_REG_DAC_CTRL1:
	case NAU8325_REG_DAC_CTRL2:
	case NAU8325_REG_ALC_CTRL1 ... NAU8325_REG_ALC_CTRL4:
	case NAU8325_REG_CLK_DET_CTRL:
	case NAU8325_REG_TEST_STATUS:
	case NAU8325_REG_ANALOG_READ:
	case NAU8325_REG_MIXER_CTRL:
	case NAU8325_REG_MISC_CTRL:
	case NAU8325_REG_BIAS_ADJ ... NAU8325_REG_ANALOG_CONTROL_6:
	case NAU8325_REG_CLIP_CTRL:
	case NAU8325_REG_RDAC:
		return true;
	default:
		return false;
	}

}

static bool nau8325_writeable_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case NAU8325_REG_CLK_CTRL ... NAU8325_REG_INT_CLR_STATUS:
	case NAU8325_REG_IRQOUT:
	case NAU8325_REG_IO_CTRL ... NAU8325_REG_TIME_SLOT:
	case NAU8325_REG_HPF_CTRL ... NAU8325_REG_DAC_VOLUME:
	case NAU8325_REG_DAC_CTRL1:
	case NAU8325_REG_DAC_CTRL2:
	case NAU8325_REG_ALC_CTRL1 ... NAU8325_REG_ALC_CTRL4:
	case NAU8325_REG_CLK_DET_CTRL:
	case NAU8325_REG_TEST_STATUS:
	case NAU8325_REG_MIXER_CTRL:
	case NAU8325_REG_MISC_CTRL:
	case NAU8325_REG_BIAS_ADJ ... NAU8325_REG_ANALOG_CONTROL_6:
	case NAU8325_REG_CLIP_CTRL:
	case NAU8325_REG_RDAC:
		return true;
	default:
		return false;
	}
}

static bool nau8325_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case NAU8325_REG_HARDWARE_RST ... NAU8325_REG_SOFTWARE_RST:
	case NAU8325_REG_DEVICE_ID:
	case NAU8325_REG_INT_CLR_STATUS:
	case NAU8325_REG_DEBUG_READ1 ... NAU8325_REG_DEBUG_READ3:
	case NAU8325_REG_TEST_STATUS ... NAU8325_REG_ANALOG_READ:
		return true;
	default:
		return false;
	}
}

static int nau8325_clkdet_put(struct snd_kcontrol *kcontrol,
        struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	//struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct nau8325 *nau8325 = snd_soc_component_get_drvdata(component);
	unsigned int max = mc->max, min = mc->min, val;
	unsigned int mask = (1 << fls(max)) - 1;

	val = ((ucontrol->value.integer.value[0] + min) & mask);
	nau8325->clock_detection = val;

	if (nau8325->clock_detection)
		regmap_update_bits(nau8325->regmap, NAU8325_REG_CLK_DET_CTRL,
			NAU8325_CLKPWRUP_EN, 0);
	else
		regmap_update_bits(nau8325->regmap, NAU8325_REG_CLK_DET_CTRL,
			NAU8325_CLKPWRUP_EN, NAU8325_CLKPWRUP_EN);

	return nau8325->clock_detection;
}

int nau8325_clkdet_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct nau8325 *nau8325 = snd_soc_component_get_drvdata(component);

	ucontrol->value.integer.value[0] = nau8325->clock_detection;

	return 0;
}

static const char * const nau8325_adc_decimation[] =
	{ "32", "64", "128" };

static const char * const nau8325_dac_oversampl[] =
	{ "64", "256", "128", "", "32" };

static const struct soc_enum nau8325_dac_oversampl_enum =
	SOC_ENUM_SINGLE(NAU8325_REG_DAC_CTRL1, NAU8325_DAC_OVERSAMPLE_SFT,
	ARRAY_SIZE(nau8325_dac_oversampl), nau8325_dac_oversampl);

static const char * const nau8325_tdm_slot[] = { "Slot 0", "Slot 1", "Slot 2",
	"Slot 3", "Slot 4", "Slot 5", "Slot 6", "Slot 7" };

static const struct soc_enum nau8325_dac_alcgain_sel_enum =
	SOC_ENUM_SINGLE(NAU8325_REG_I2S_PCM_CTRL0, NAU8325_DAC_SEL_AGC_SFT,
	ARRAY_SIZE(nau8325_tdm_slot), nau8325_tdm_slot);

static const struct soc_enum nau8325_dac_sel_enum =
	SOC_ENUM_SINGLE(NAU8325_REG_I2S_PCM_CTRL0, NAU8325_DAC_SEL_SFT,
	ARRAY_SIZE(nau8325_tdm_slot), nau8325_tdm_slot);

static const char * const nau8325_adc_alcgain_slot[] = { "Disable", "Slot 0",
	"Slot 1", "Slot 2", "Slot 3", "Slot 4", "Slot 5", "Slot 6", "Slot 7" };

static const unsigned int nau8325_adc_alcgain_val[] =
	{ 0x0, 0x8, 0x9, 0xa, 0xb, 0xc, 0xd, 0xe, 0xf };

static const char * const nau8325_adc_isense_slot[] =
	{ "Disable", "Slot 1", "Slot 3", "Slot 5", "Slot 7" };

static const unsigned int nau8325_adc_isense_val[] =
	{ 0x0, 0x4, 0x5, 0x6, 0x7 };

static const char * const nau8325_adc_vsense_slot[] =
	{ "Disable", "Slot 0", "Slot 2", "Slot 4", "Slot 6" };

static const char * const nau8325_limiter_mode[] = {
	"Limiter Mode with Clip Detection",
	"Low Battery Limiter Mode with Clip Detection",
	"Limiter Mode with Pre-Programmed Limit Level",
	"Low Battery Limiter Mode with Pre-Programmed Limit Level",
	"Low Battery Limiter Mode with Pre-Programmed VBAT Ratio Limit Level" };

static const struct soc_enum nau8325_limiter_mode_enum =
	SOC_ENUM_SINGLE(NAU8325_REG_ALC_CTRL3, NAU8325_LIM_MDE_SFT,
	ARRAY_SIZE(nau8325_limiter_mode), nau8325_limiter_mode);

static const DECLARE_TLV_DB_MINMAX_MUTE(dac_vol_tlv, -8000, 600);
static const DECLARE_TLV_DB_MINMAX(adc_vol_tlv, 0, 24125);
static const DECLARE_TLV_DB_MINMAX(pga_gain_tlv, 0, 1600);

static const struct snd_kcontrol_new nau8325_snd_controls[] = {
	SOC_ENUM("DAC Oversampling Rate", nau8325_dac_oversampl_enum),
	SOC_DOUBLE_TLV("Speaker Volume",NAU8325_REG_DAC_VOLUME,
		NAU8325_DAC_VOLUME_L_SFT, NAU8325_DAC_VOLUME_R_SFT,
		NAU8325_DAC_VOL_MAX,0, dac_vol_tlv),
	
	SOC_ENUM("DAC ALC Gain Channel Source",
		nau8325_dac_alcgain_sel_enum),
	SOC_ENUM("DAC Channel Source", nau8325_dac_sel_enum),

	SOC_SINGLE("PingPong Mode", NAU8325_REG_TDM_CTRL,
		NAU8325_PINGPONG_SFT, 1, 0),

	SOC_SINGLE_EXT("Clock Detection", SND_SOC_NOPM, 0, 1, 0,
		nau8325_clkdet_get, nau8325_clkdet_put),

	SOC_SINGLE("ALC Max Gain", NAU8325_REG_ALC_CTRL1,
		NAU8325_ALC_MAXGAIN_SFT, NAU8325_ALC_MAXGAIN_MAX, 0),
	SOC_SINGLE("ALC Min Gain", NAU8325_REG_ALC_CTRL1,
		NAU8325_ALC_MINGAIN_SFT, NAU8325_ALC_MINGAIN_MAX, 0),
	SOC_SINGLE("ALC Decay Timer", NAU8325_REG_ALC_CTRL2,
		NAU8325_ALC_DCY_SFT, NAU8325_ALC_DCY_MAX, 0),
	SOC_SINGLE("ALC Attack Timer", NAU8325_REG_ALC_CTRL2,
		NAU8325_ALC_ATK_SFT, NAU8325_ALC_ATK_MAX, 0),
	SOC_SINGLE("ALC Hold Time", NAU8325_REG_ALC_CTRL2,
		NAU8325_ALC_HLD_SFT, NAU8325_ALC_HLD_MAX, 0),
	SOC_SINGLE("ALC Target Level", NAU8325_REG_ALC_CTRL2,
		NAU8325_ALC_LVL_SFT, NAU8325_ALC_LVL_MAX, 0),
	SOC_SINGLE("ALC Enable", NAU8325_REG_ALC_CTRL3,
		NAU8325_ALC_EN_SFT, 1, 0),
	SOC_ENUM("Limiter Mode Selection", nau8325_limiter_mode_enum),
	SOC_SINGLE("Low Battery Limiter Threshold", NAU8325_REG_ALC_CTRL3,
		NAU8325_VBAT_THLD_SFT, NAU8325_VBAT_THLD_MAX, 0),
	SOC_SINGLE("Auto Attenuation Enable", NAU8325_REG_ALC_CTRL3,
		NAU8325_AUTOATT_EN_SFT, 1, 0),

	SOC_SINGLE("BCLK IO Driver Stronger", NAU8325_REG_IO_CTRL,
		NAU8325_BCLK_DS_SFT, 1, 0),
	SOC_SINGLE("LRC IO Driver Stronger", NAU8325_REG_IO_CTRL,
		NAU8325_LRC_DS_SFT, 1, 0),
	SOC_SINGLE("ADCDAT IO Driver Stronger", NAU8325_REG_IO_CTRL,
		NAU8325_ADCDAT_DS_SFT, 1, 0),

	SOC_SINGLE("Low Output Noise in Receiver Mode",
		NAU8325_REG_ANALOG_CONTROL_4,
		NAU8325_RECV_MODE_SFT, 1, 0),
};

static int nau8325_dac_event(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component =
		snd_soc_dapm_to_component(w->dapm);
	struct nau8325 *nau8325 = snd_soc_component_get_drvdata(component);

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		regmap_update_bits(nau8325->regmap,
			NAU8325_REG_MUTE_CTRL,
			NAU8325_SOFT_MUTE, 0);
		msleep(30);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		/* Soft mute the output to prevent the pop noise */
		regmap_update_bits(nau8325->regmap,
			NAU8325_REG_MUTE_CTRL,
			NAU8325_SOFT_MUTE, NAU8325_SOFT_MUTE);
		msleep(30);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int nau8325_powerup_event(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component =
		snd_soc_dapm_to_component(w->dapm);
	struct nau8325 *nau8325 = snd_soc_component_get_drvdata(component);

	if (nau8325->clock_detection)
		return 0;

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		regmap_update_bits(nau8325->regmap, NAU8325_REG_CLK_DET_CTRL,
			NAU8325_PWRUP_DFT, NAU8325_PWRUP_DFT);
		break;
	case SND_SOC_DAPM_POST_PMD:
		regmap_update_bits(nau8325->regmap,
			NAU8325_REG_CLK_DET_CTRL,
			NAU8325_PWRUP_DFT, 0);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static const struct snd_soc_dapm_widget nau8325_dapm_widgets[] = {
	SND_SOC_DAPM_SUPPLY("PowerUp", SND_SOC_NOPM, 0, 0,
		nau8325_powerup_event, SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_IN("AIFRX", "Playback", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_DAC_E("DAC_L", NULL, NAU8325_REG_ENA_CTRL,
		NAU8325_DAC_LEFT_CH_EN_SFT, 0, nau8325_dac_event,
		SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_DAC_E("DAC_R", NULL, NAU8325_REG_ENA_CTRL,
		NAU8325_DAC_RIGHT_CH_EN_SFT, 0, nau8325_dac_event,
		SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),

	SND_SOC_DAPM_OUTPUT("SPK"),
};

static const struct snd_soc_dapm_route nau8325_dapm_routes[] = {
	{ "DAC_L", NULL, "PowerUp" },
	{ "DAC_R", NULL, "PowerUp" },
	{ "DAC_L", NULL, "I2S Normal Mode" },
	{ "DAC_R", NULL, "I2S Normal Mode" },
	{ "DAC_L", NULL, "AIFRX" },
	{ "DAC_R", NULL, "AIFRX" },
	{ "SPK", NULL, "DAC_L" },
	{ "SPK", NULL, "DAC_R" },
};

static int nau8325_srate_clk_apply(struct nau8325 *nau8325,
	const struct nau8325_srate_attr *srate_table,
	int n1_sel, int mclk_mult_sel, int n2_sel, int dsp_mult_sel)
{
	int dsp_clk, mult_sel;
	
	if (!srate_table || srate_table->adc_div < 0 ||
		dsp_mult_sel < 0 || dsp_mult_sel >= ARRAY_SIZE(dsp_src_mult) ||
		n2_sel < 0 || n2_sel >= ARRAY_SIZE(mclk_n2_div) ||
		n1_sel < 0 || n1_sel >= ARRAY_SIZE(mclk_n1_div)) {
		dev_err(nau8325->dev, "The fs %d isn't support in DSP.\n",
			nau8325->fs);
		
		return -EINVAL;
	}
	
	regmap_update_bits(nau8325->regmap, NAU8325_REG_CLK_DET_CTRL,
		NAU8325_REG_SRATE_MASK | NAU8325_REG_DIV_MAX,
		(srate_table->range << NAU8325_REG_SRATE_SFT) |
		(srate_table->max ? NAU8325_REG_DIV_MAX : 0));
		
	if (srate_table->fs == 44100){
		regmap_update_bits(nau8325->regmap, NAU8325_REG_CLK_DET_CTRL,
			NAU8325_REG_ALT_SRATE_EN, NAU8325_REG_ALT_SRATE_EN);
	}
	else{
		regmap_update_bits(nau8325->regmap, NAU8325_REG_CLK_DET_CTRL,
			NAU8325_REG_ALT_SRATE_EN, 0);
	}

	dsp_clk = nau8325->mclk << dsp_src_mult[dsp_mult_sel].param;
	dsp_clk = dsp_clk / mclk_n1_div[n1_sel].param;

	regmap_update_bits(nau8325->regmap, NAU8325_REG_CLK_CTRL,
		NAU8325_MCLK_SRC_MASK, mclk_n2_div[n2_sel].val);
	regmap_update_bits(nau8325->regmap, NAU8325_REG_ENA_CTRL,
		NAU8325_CLK_MUL_SRC_MASK,
		mclk_n1_div[n1_sel].val << NAU8325_CLK_MUL_SRC_SFT);
	
	if (mclk_mult_sel != CLK_PROC_BYPASS) {
		regmap_update_bits(nau8325->regmap, NAU8325_REG_ENA_CTRL,
			NAU8325_MCLK_SEL_MASK,
			mclk_src_mult[mclk_mult_sel].val << NAU8325_MCLK_SEL_SFT);
		mult_sel = (mclk_mult_sel > dsp_mult_sel ?
			mclk_mult_sel : dsp_mult_sel);
	} else {
		mult_sel = dsp_mult_sel;
	}
	
	switch (mult_sel) {
	case 3:	/* multiplier 8x, i.e. 2^3 */
		regmap_update_bits(nau8325->regmap,
			NAU8325_REG_ANALOG_CONTROL_5,
			NAU8325_MCLK8XEN_EN, NAU8325_MCLK8XEN_EN);
	case 2:	/* multiplier 4x, i.e. 2^2 */
		regmap_update_bits(nau8325->regmap,
			NAU8325_REG_ANALOG_CONTROL_5,
			NAU8325_MCLK4XEN_EN, NAU8325_MCLK4XEN_EN);
		break;
	default :
		return -EINVAL;
	}

	return 0;
}

int nau8325_clkdsp_choose(struct nau8325 *nau8325, int *n1_sel, bool n1_try,
	int *dsp_n1_sel, int *dsp_mult_sel)
{
	int i, j, dsp_n1, mult_sel, dsp_clk, dsp_clk_max = 0;

	if (!nau8325->fs || !nau8325->mclk)
		goto proc_err;

	if (n1_sel) {
		dsp_n1 = *n1_sel;
		for (j = 0; j < ARRAY_SIZE(dsp_src_mult); j++) {
			dsp_clk = nau8325->mclk << dsp_src_mult[j].param;
			dsp_clk = dsp_clk / mclk_n1_div[dsp_n1].param;
			if (dsp_clk < DSP_CLK_MIN || dsp_clk > DSP_CLK_MAX)
				continue;
			if (dsp_clk_max < dsp_clk) {
				dsp_clk_max = dsp_clk;
				mult_sel = j;
			}
		}
		if (!dsp_clk_max)
			goto proc_err;
		if (!n1_try) {
			*dsp_n1_sel = dsp_n1;
			*dsp_mult_sel = mult_sel;
		}
	} else {
		dsp_n1 = -1;
		if (nau8325->mclk >= DSP_MCLK_HI_SPEED) {
			/* get max N1 when MCLK over 12.288MHz */
			for (i = 0; i < ARRAY_SIZE(mclk_n1_div); i++)
				for (j = 0; j < ARRAY_SIZE(dsp_src_mult); j++) {
					dsp_clk = nau8325->mclk <<
						dsp_src_mult[j].param;
					dsp_clk = dsp_clk /
						mclk_n1_div[i].param;
					if (dsp_clk < DSP_CLK_MIN ||
						dsp_clk > DSP_CLK_MAX)
						continue;
					if ((dsp_clk_max < dsp_clk ||
						i > dsp_n1)) {
						dsp_clk_max = dsp_clk;
						dsp_n1 = i;
						mult_sel = j;
					}
				}
			if (!dsp_clk_max)
				goto proc_err;
		} else {
			for (i = 0; i < ARRAY_SIZE(mclk_n1_div); i++) {
				for (j = 0; j < ARRAY_SIZE(dsp_src_mult); j++) {
					dsp_clk = nau8325->mclk <<
						dsp_src_mult[j].param;
					dsp_clk = dsp_clk /
						mclk_n1_div[i].param;
					if (dsp_clk >= DSP_CLK_MIN &&
						dsp_clk <= DSP_CLK_MAX) {
						dsp_n1 = i;
						mult_sel = j;
						break;
					}
				}
				if (dsp_n1 >= 0)
					break;
			}
			if (dsp_n1 < 0)
				goto proc_err;
		}
		*dsp_n1_sel = dsp_n1;
		*dsp_mult_sel = mult_sel;
	}

	return 0;

proc_err:
	return -EINVAL;
}

static int nau8325_osr_apply(struct nau8325 *nau8325)
{
	unsigned int osr;

	/* CLK_DAC or CLK_ADC = OSR * FS
	 * DAC or ADC clock frequency is defined as Over Sampling Rate (OSR)
	 * multiplied by the audio sample rate (Fs). Note that the OSR and Fs
	 * values must be selected such that the maximum frequency is less
	 * than 6.144 MHz.
	 */
	regmap_read(nau8325->regmap, NAU8325_REG_DAC_CTRL1, &osr);

	osr &= NAU8325_DAC_OVERSAMPLE_MASK;

	regmap_update_bits(nau8325->regmap, NAU8325_REG_CLK_CTRL,
		NAU8325_CLK_DAC_SRC_MASK,
		osr_dac_sel[osr].clk_src << NAU8325_CLK_DAC_SRC_SFT);

	return 0;
}

static int nau8325_clksrc_n2(struct nau8325 *nau8325,
	const struct nau8325_srate_attr *srate_table, int mclk, int *n2_sel)
{
	int i, mclk_src, ratio;
	
	ratio = NAU8325_MCLK_FS_RATIO_NUM;
	for (i = 0; i < ARRAY_SIZE(mclk_n2_div); i++) {
		mclk_src = mclk >> mclk_n2_div[i].param;
		if (srate_table->mclk_src[NAU8325_MCLK_FS_RATIO_256]
			== mclk_src) {
			ratio = NAU8325_MCLK_FS_RATIO_256;
			break;
		} else if (srate_table->mclk_src[NAU8325_MCLK_FS_RATIO_400]
			== mclk_src) {
				ratio = NAU8325_MCLK_FS_RATIO_400;
			break;
		} else if (srate_table->mclk_src[NAU8325_MCLK_FS_RATIO_500]
			== mclk_src) {
				ratio = NAU8325_MCLK_FS_RATIO_500;
			break;
		}
	}
	if (ratio != NAU8325_MCLK_FS_RATIO_NUM)
		*n2_sel = i;

	return ratio;
}

static const struct nau8325_srate_attr *target_srate_attribute(int srate)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(target_srate_table); i++)
		if (target_srate_table[i].fs == srate)
			break;
	if (i == ARRAY_SIZE(target_srate_table))
		goto proc_err;

	return &target_srate_table[i];

proc_err:
	return NULL;
}


static int nau8325_clksrc_choose(struct nau8325 *nau8325,
	const struct nau8325_srate_attr **srate_table,
	int *n1_sel, int *mult_sel, int *n2_sel, int *dsp_mult_sel)
{
	int i, j, mclk, mclk_max, ratio, ratio_sel, n2_max, ret;

	if (!nau8325->mclk || !nau8325->fs)
		goto proc_err;

	/* select sampling rate and MCLK_SRC */
	*srate_table = target_srate_attribute(nau8325->fs);
	if (*srate_table == NULL)
		goto proc_err;

	/* First check clock from MCLK directly, decide N2 for MCLK_SRC.
	 * If not good, consider 1/N1 and Multiplier.
	 */
	ratio = nau8325_clksrc_n2(nau8325, *srate_table, nau8325->mclk, n2_sel);

	if (ratio != NAU8325_MCLK_FS_RATIO_NUM) {
		*n1_sel = CLK_PROC_BYPASS;
		*mult_sel = CLK_PROC_BYPASS;
		ret = nau8325_clkdsp_choose(nau8325, NULL, false,
			n1_sel, dsp_mult_sel);
		if (ret)
			goto proc_err;
		goto proc_done;
	}
	/* Get MCLK_SRC through 1/N, Multiplier, and then 1/N2. */
	mclk_max = 0;
	for (i = 0; i < ARRAY_SIZE(mclk_n1_div); i++) {
		for (j = 0; j < ARRAY_SIZE(mclk_src_mult); j++) {
			mclk = nau8325->mclk << mclk_src_mult[j].param;
			mclk = mclk / mclk_n1_div[i].param;
			ratio = nau8325_clksrc_n2(nau8325,
				*srate_table, mclk, n2_sel);
			if (ratio != NAU8325_MCLK_FS_RATIO_NUM &&
				(mclk_max < mclk || i > *n1_sel)) {
				ret = nau8325_clkdsp_choose(nau8325, &i, true,
					n1_sel, dsp_mult_sel);
				if (ret)
					break;
				mclk_max = mclk;
				n2_max = *n2_sel;
				*n1_sel = i;
				*mult_sel = j;
				ratio_sel = ratio;
			}
		}
	}
	if (mclk_max) {
		*n2_sel = n2_max;
		ratio = ratio_sel;
		nau8325_clkdsp_choose(nau8325, n1_sel, false,
			n1_sel, dsp_mult_sel);
		goto proc_done;
	}

proc_err:
	dev_err(nau8325->dev, "The MCLK %d is invalid. It can't get MCLK_SRC of 256/400/500 FS (%d).\n",
		nau8325->mclk, nau8325->fs);
	return -EINVAL;
proc_done:
	dev_dbg(nau8325->dev, "FS %dHz, range 0x%x, %s, (n1,mu,n2,dmu):(%d,%d,%d,%d), MCLK_SRC %uHz (%d)\n",
		nau8325->fs, (*srate_table)->range,
		(*srate_table)->max ? "MAX" : "MIN",
		*n1_sel == CLK_PROC_BYPASS ?
		CLK_PROC_BYPASS : mclk_n1_div[*n1_sel].param,
		*mult_sel == CLK_PROC_BYPASS ?
		CLK_PROC_BYPASS : 1 << mclk_src_mult[*mult_sel].param,
		1 << mclk_n2_div[*n2_sel].param,
		1 << dsp_src_mult[*dsp_mult_sel].param,
		(*srate_table)->mclk_src[ratio],
		(*srate_table)->mclk_src[ratio] / nau8325->fs);
	return 0;
}


static int nau8325_clock_config(struct nau8325 *nau8325)
{
	const struct nau8325_srate_attr *srate_table;
	int ret, n1_sel, mult_sel, n2_sel, dsp_mult_sel;

	ret = nau8325_clksrc_choose(nau8325, &srate_table,
		&n1_sel, &mult_sel, &n2_sel, &dsp_mult_sel);
	if (ret)
		goto err;

	ret = nau8325_srate_clk_apply(nau8325, srate_table,
		n1_sel, mult_sel, n2_sel, dsp_mult_sel);
	if (ret)
		goto err;

	ret = nau8325_osr_apply(nau8325);
	if (ret)
		goto err;

	return 0;
err:
	return ret;
}

static int nau8325_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	struct snd_soc_component *component = dai->component;
	struct nau8325 *nau8325 = snd_soc_component_get_drvdata(component);
	unsigned int val_len = 0;
	int ret;

	nau8325->fs = params_rate(params);
	ret = nau8325_clock_config(nau8325);
	if (ret)
		goto err;

	switch (params_width(params)) {
	case 16:
		val_len |= NAU8325_I2S_DL_16;
		break;
	case 20:
		val_len |= NAU8325_I2S_DL_20;
		break;
	case 24:
		val_len |= NAU8325_I2S_DL_24;
		break;
	case 32:
		val_len |= NAU8325_I2S_DL_32;
		break;
	default:
		ret = -EINVAL;
		goto err;
	}

	regmap_update_bits(nau8325->regmap, NAU8325_REG_I2S_PCM_CTRL1,
		NAU8325_I2S_DL_MASK, val_len);

	return 0;
err:
	return ret;
}

static int nau8325_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct snd_soc_component *component = dai->component;
	struct nau8325 *nau8325 = snd_soc_component_get_drvdata(component);
	unsigned int ctrl1_val = 0, ctrl2_val = 0;

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		ctrl2_val |= NAU8325_I2S_MS_MASTER;
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
		break;
	default:
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		break;
	case SND_SOC_DAIFMT_IB_NF:
		ctrl1_val |= NAU8325_I2S_BP_INV;
		break;
	default:
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		ctrl1_val |= NAU8325_I2S_DF_I2S;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		ctrl1_val |= NAU8325_I2S_DF_LEFT;
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		ctrl1_val |= NAU8325_I2S_DF_RIGTH;
		break;
	case SND_SOC_DAIFMT_DSP_A:
		ctrl1_val |= NAU8325_I2S_DF_PCM_AB;
		break;
	case SND_SOC_DAIFMT_DSP_B:
		ctrl1_val |= NAU8325_I2S_DF_PCM_AB;
		ctrl1_val |= NAU8325_I2S_PCMB_EN;
		break;
	default:
		return -EINVAL;
	}

	regmap_update_bits(nau8325->regmap, NAU8325_REG_I2S_PCM_CTRL1,
		NAU8325_I2S_DF_MASK | NAU8325_I2S_BP_MASK |
		NAU8325_I2S_PCMB_EN, ctrl1_val);
	regmap_update_bits(nau8325->regmap, NAU8325_REG_I2S_PCM_CTRL2,
		NAU8325_I2S_MS_MASK, ctrl2_val);

	return 0;
}

static int nau8325_set_sysclk(struct snd_soc_component *component, int clk_id,
	int source, unsigned int freq, int dir)
{
	struct nau8325 *nau8325 = snd_soc_component_get_drvdata(component);

	if (freq < MASTER_CLK_MIN || freq > MASTER_CLK_MAX) {
		dev_err(nau8325->dev,
			"Exceed the range of input clocks, MCLK %dHz\n", freq);
		return -EINVAL;
	}
	nau8325->mclk = freq;
	dev_dbg(nau8325->dev, "MCLK %dHz\n", nau8325->mclk);

	return 0;
}

static int nau8325_codec_probe(struct snd_soc_component *component)
{
	struct nau8325 *nau8325 = snd_soc_component_get_drvdata(component);
	struct snd_soc_dapm_context *dapm = snd_soc_component_get_dapm(component);

	nau8325->dapm = dapm;

	return 0;
}

static struct snd_soc_component_driver nau8325_component_driver = {
	.probe = nau8325_codec_probe,
	.set_sysclk = nau8325_set_sysclk,
	.suspend_bias_off = true,
	.controls = nau8325_snd_controls,
	.num_controls = ARRAY_SIZE(nau8325_snd_controls),
	.dapm_widgets = nau8325_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(nau8325_dapm_widgets),
	.dapm_routes = nau8325_dapm_routes,
	.num_dapm_routes = ARRAY_SIZE(nau8325_dapm_routes),
	
};

static const struct snd_soc_dai_ops nau8325_dai_ops = {
	.hw_params = nau8325_hw_params,
	.set_fmt = nau8325_set_fmt,
};

#define NAU8325_RATES SNDRV_PCM_RATE_8000_96000
#define NAU8325_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE \
	 | SNDRV_PCM_FMTBIT_S24_3LE)

static struct snd_soc_dai_driver nau8325_dai = {
	.name = NAU8325_CODEC_DAI,
	.playback = {
		.stream_name = "Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates = NAU8325_RATES,
		.formats = NAU8325_FORMATS,
	},
	.ops = &nau8325_dai_ops,
};

static const struct regmap_config nau8325_regmap_config = {
	.reg_bits = NAU8325_REG_ADDR_LEN,
	.val_bits = NAU8325_REG_DATA_LEN,

	.max_register = NAU8325_REG_MAX,
	.readable_reg = nau8325_readable_reg,
	.writeable_reg = nau8325_writeable_reg,
	.volatile_reg = nau8325_volatile_reg,
	
	.cache_type = REGCACHE_RBTREE,
	.reg_defaults = nau8325_reg_defaults,
	.num_reg_defaults = ARRAY_SIZE(nau8325_reg_defaults),
};

static void nau8325_reset_chip(struct regmap *regmap)
{
	regmap_write(regmap, NAU8325_REG_HARDWARE_RST, 0x0001);
	regmap_write(regmap, NAU8325_REG_HARDWARE_RST, 0x0000);
}

static void nau8325_software_reset(struct regmap *regmap)
{
	regmap_write(regmap, NAU8325_REG_SOFTWARE_RST, 0x0000);
	regmap_write(regmap, NAU8325_REG_SOFTWARE_RST, 0x0000);
}

static void nau8325_init_regs(struct nau8325 *nau8325)
{
	struct regmap *regmap = nau8325->regmap;

	/* Block MCLK */
	regmap_update_bits(regmap, NAU8325_REG_CLK_CTRL,
		NAU8325_MCLK_SRC_MASK, 0x4);
	
	regmap_update_bits(regmap, NAU8325_REG_DAC_CTRL1,
		NAU8325_DAC_OVERSAMPLE_MASK, NAU8325_DAC_OVERSAMPLE_64);
	/* Enable auto attenuation for better output signal */
	regmap_update_bits(regmap, NAU8325_REG_ALC_CTRL3,
		NAU8325_AUTOATT_EN, NAU8325_AUTOATT_EN);
	/* Set ALC parameters */
	regmap_update_bits(regmap, NAU8325_REG_ALC_CTRL1,
		NAU8325_ALC_MAXGAIN_MASK,
		0x7 << NAU8325_ALC_MAXGAIN_SFT);
	regmap_update_bits(regmap, NAU8325_REG_ALC_CTRL2,
		NAU8325_ALC_DCY_MASK | NAU8325_ALC_ATK_MASK |
		NAU8325_ALC_HLD_MASK, (0x5 << NAU8325_ALC_DCY_SFT) |
		(0x3 << NAU8325_ALC_ATK_SFT) |
		(0x5 << NAU8325_ALC_HLD_SFT));
	regmap_update_bits(regmap, NAU8325_REG_ALC_CTRL3,
		NAU8325_LIM_MDE_MASK |	NAU8325_VBAT_THLD_MASK,
		(0x3 << NAU8325_LIM_MDE_SFT) |
		(0x18 << NAU8325_VBAT_THLD_SFT));
	/* Enable ALC to avoid signal distortion when battery low. */
	if (nau8325->alc_enable)
		regmap_update_bits(regmap, NAU8325_REG_ALC_CTRL3,
			NAU8325_ALC_EN, NAU8325_ALC_EN);
	if (nau8325->clock_detection)
		regmap_update_bits(regmap, NAU8325_REG_CLK_DET_CTRL,
			NAU8325_CLKPWRUP_EN | NAU8325_PWRUP_DFT, 0);
	else
		regmap_update_bits(regmap, NAU8325_REG_CLK_DET_CTRL,
			NAU8325_CLKPWRUP_EN | NAU8325_PWRUP_DFT,
			NAU8325_CLKPWRUP_EN);
	if (nau8325->clock_det_data)
		regmap_update_bits(regmap, NAU8325_REG_CLK_DET_CTRL,
			NAU8325_APWRUP_EN, NAU8325_APWRUP_EN);
	else
		regmap_update_bits(regmap, NAU8325_REG_CLK_DET_CTRL,
			NAU8325_APWRUP_EN, 0);

	/* DAC Reference Voltage Setting */
	regmap_update_bits(regmap, NAU8325_REG_RDAC,
		NAU8325_DACVREFSEL_MASK,
		nau8325->dac_vref << NAU8325_DACVREFSEL_SFT);
	/* DAC Reference Voltage Decoupling Capacitors. */
	regmap_update_bits(regmap, NAU8325_REG_ANALOG_CONTROL_3,
		NAU8325_DACREFCAP_MASK, 0x3 << NAU8325_DACREFCAP_SFT);
	/* Auto-Att Min Gain 0dB, Class-D N Driver Slew Rate -25%. */
	regmap_update_bits(regmap, NAU8325_REG_ANALOG_CONTROL_4,
		NAU8325_AUTOATTMIN_MASK | NAU8325_CLASSD_SLEWN_MASK,
		NAU8325_AUTOATTMIN_0DB | 0x7);
	
	/* Slew Rate Adjust, Bias Current. */
	regmap_update_bits(regmap, NAU8325_REG_ANALOG_CONTROL_6,
		NAU8325_BSTSLEWPOFF_MASK | NAU8325_BSTSLEWPON_MASK |
		NAU8325_BSTSLEWNON_MASK |NAU8325_BSTSLEWNOFF_MASK |
		NAU8325_BSTIPDR_MASK, (0x3 << NAU8325_BSTSLEWPOFF_SFT) |
		(0x2 << NAU8325_BSTSLEWPON_SFT) |
		(0x2 << NAU8325_BSTSLEWNON_SFT) |
		(0x1 << NAU8325_BSTSLEWNOFF_SFT) | 0x4);
	
	/* VMID Tieoff (VMID Resistor Selection) */
	regmap_update_bits(regmap, NAU8325_REG_BIAS_ADJ,
		NAU8325_BIAS_VMID_SEL_MASK,
		nau8325->vref_impedance << NAU8325_BIAS_VMID_SEL_SFT);
	
	nau8325_software_reset(regmap);
	/* Enable VMID, BIAS, DAC, DCA CLOCK, ADC, Voltage/Current Amps,
	 * ADC Resetb gated by 'PowerUp' Signal.
	 */
	regmap_update_bits(regmap, NAU8325_REG_ANALOG_CONTROL_1,
		NAU8325_ISEN_MASK | NAU8325_ADCRSTEN_MASK |
		NAU8325_VSEN_MASK | NAU8325_ADCEN_MASK |
		NAU8325_DACCLKEN_MASK | NAU8325_DACEN_MASK |
		NAU8325_BIASEN_MASK | NAU8325_VMIDEN_MASK,
		(0x1 << NAU8325_ISEN_SFT) | (0x1 << NAU8325_ADCRSTEN_SFT) |
		(0x1 << NAU8325_VSEN_SFT) | (0x1 << NAU8325_ADCEN_SFT) |
		(0x1 << NAU8325_DACCLKEN_SFT) | (0x1 << NAU8325_DACEN_SFT) |
		(0x1 << NAU8325_BIASEN_SFT) | 0x1);
	/* Enable Boost Convertor, VMID Fast, Class-D gated by
	 * 'PowerUp' Signal
	 */
	regmap_update_bits(regmap, NAU8325_REG_ANALOG_CONTROL_2,
		NAU8325_CLASSDEN_MASK | NAU8325_PDVMDFST_MASK |
		NAU8325_BSTEN_MASK, (0x1 << NAU8325_CLASSDEN_SFT) |
		(0x1 << NAU8325_PDVMDFST_SFT) | nau8325->boost_convert_enable);
		
}

static int nau8325_i2c_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
{
	struct device *dev = &i2c->dev;
	struct nau8325 *nau8325 = dev_get_platdata(dev);
	int ret, value;

	if (!nau8325) {
		nau8325 = devm_kzalloc(dev, sizeof(*nau8325), GFP_KERNEL);
		if (!nau8325)
			return -ENOMEM;
		if (ret)
			return ret;
	}
	i2c_set_clientdata(i2c, nau8325);

	nau8325->regmap = devm_regmap_init_i2c(i2c, &nau8325_regmap_config);
	if (IS_ERR(nau8325->regmap))
		return PTR_ERR(nau8325->regmap);
	nau8325->dev = dev;
	
	ret = regmap_read(nau8325->regmap, NAU8325_REG_DEVICE_ID, &value);
	if (ret) {
		dev_err(dev, "Failed to read device id from the NAU8325: %d\n",
			ret);
		return ret;
	}
	nau8325_reset_chip(nau8325->regmap);
	nau8325_init_regs(nau8325);

	return devm_snd_soc_register_component(dev, &nau8325_component_driver, &nau8325_dai, 1);
}

static int nau8325_i2c_remove(struct i2c_client *client)
{
	snd_soc_unregister_component(&client->dev);
	return 0;
}

static const struct i2c_device_id nau8325_i2c_ids[] = {
	{ "nau8325", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, nau8325_i2c_ids);

#ifdef CONFIG_OF
static const struct of_device_id nau8325_of_ids[] = {
	{ .compatible = "nuvoton,nau8325", },
	{}
};
MODULE_DEVICE_TABLE(of, nau8325_of_ids);
#endif

static struct i2c_driver nau8325_i2c_driver = {
	.driver = {
		.name = "nau8325",
		.of_match_table = of_match_ptr(nau8325_of_ids),
	},
	.probe = nau8325_i2c_probe,
	.remove = nau8325_i2c_remove,
	.id_table = nau8325_i2c_ids,
};
module_i2c_driver(nau8325_i2c_driver);

MODULE_DESCRIPTION("ASoC NAU8325 is a stereo high efficiency filter-free Class-D audio amplifier driver.");
MODULE_AUTHOR("Seven Lee <wtli@nuvoton.com>");
MODULE_AUTHOR("David Lin <CTLIN0@nuvoton.com>");
MODULE_LICENSE("GPL v2");

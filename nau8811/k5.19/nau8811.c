// SPDX-License-Identifier: GPL-2.0-only
//
// nau8811.c -- Nuvoton NAU88L11 audio codec driver
//
// Copyright 2021 Nuvoton Technology Corp.
// Author: David Lin <ctlin0@nuvoton.com>
//
#include <linux/acpi.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/math64.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/initval.h>
#include <sound/jack.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/tlv.h>
#include "nau8811.h"

/* Range of Master Clock MCLK (Hz) */
#define MASTER_CLK_MIN 2048000
#define MCLK_RNG_15 15360000
#define MCLK_RNG_21 21600000
#define MASTER_CLK_MAX 24576000

/* the maximum frequency of CLK_ADC and CLK_DAC */
#define CLK_DA_AD_MAX 6144000

/* the default DMIC clock frequency */
#define DMIC_CLK 3072000

static const struct nau8811_osr_attr osr_dac_sel[] = {
	{ 64, 2 },	/* OSR 64, SRC 1/4 */
	{ 256, 0 },	/* OSR 256, SRC 1 */
	{ 128, 1 },	/* OSR 128, SRC 1/2 */
	{ 0, 0 },
	{ 32, 3 },	/* OSR 32, SRC 1/8 */
};

static const struct nau8811_osr_attr osr_adc_sel[] = {
	{ 32, 3 },	/* OSR 32, SRC 1/8 */
	{ 64, 2 },	/* OSR 64, SRC 1/4 */
	{ 128, 1 },	/* OSR 128, SRC 1/2 */
	{ 256, 0 },	/* OSR 256, SRC 1 */
};

static const struct nau8811_src_attr mclk_src_div[] = {
	{ 1, 0x0 },
	{ 2, 0x2 },
	{ 3, 0x3 },
	{ 4, 0x4 },
	{ 6, 0x6 },
};

static const struct nau8811_src_attr mclk_src_mult[] = {
	{ 2, 0x1 },
};

/* Sample Rate and MCLK_SRC selections */
static const struct nau8811_srate_attr target_srate_table[] = {
	/* { FS, { MCLK source } } */
	{ 48000, { 12288000, 19200000, 24000000 } },
	{ 44100, { 11289600, 17640000, 22050000 } },
	{  8000, {  2048000,  3200000,  4000000 } },
	{ 16000, {  4096000,  6400000,  8000000 } },
	{ 24000, {  6144000,  9600000, 12000000 } },
	{ 32000, {  8192000, 12800000, 16000000 } },
	{ 88200, { 22579200,        0,        0 } },
	{ 96000, { 24576000,        0,        0 } },
};

static const struct nau8811_dmic_speed dmic_speed_sel[] = {
	{ 0, 0x0 },	/* SPEED 1, SRC 1 */
	{ 1, 0x1 },	/* SPEED 2, SRC 1/2 */
	{ 2, 0x2 },	/* SPEED 4, SRC 1/4 */
	{ 3, 0x3 },	/* SPEED 8, SRC 1/8 */
};

static const struct reg_default nau8811_reg_defaults[] = {
	{ NAU8811_R01_ENA_CTRL, 0x03fe },
	{ NAU8811_R03_CLK_DIVIDER, 0x0050 },
	{ NAU8811_R08_PDB_CTL, 0x4000 },
	{ NAU8811_R0F_INTERRUPT_MASK, 0x0 },
	{ NAU8811_R12_INTERRUPT_DIS_CTRL, 0xffff },
	{ NAU8811_R13_DMIC_CTRL, 0x0f00 },
	{ NAU8811_R1B_TDM_CTRL, 0x0 },
	{ NAU8811_R1C_I2S_PCM_CTRL1, 0x000a },
	{ NAU8811_R1D_I2S_PCM_CTRL2, 0x8010 },
	{ NAU8811_R1E_LEFT_TIME_SLOT, 0x0 },
	{ NAU8811_R21_BIQ0_COF1, 0x0 },
	{ NAU8811_R22_BIQ0_COF2, 0x0 },
	{ NAU8811_R23_BIQ0_COF3, 0x0 },
	{ NAU8811_R24_BIQ0_COF4, 0x0 },
	{ NAU8811_R25_BIQ0_COF5, 0x0 },
	{ NAU8811_R26_BIQ0_COF6, 0x0 },
	{ NAU8811_R27_BIQ0_COF7, 0x0 },
	{ NAU8811_R28_BIQ0_COF8, 0x0 },
	{ NAU8811_R29_BIQ0_COF9, 0x0 },
	{ NAU8811_R2A_BIQ0_COF10, 0x0 },
	{ NAU8811_R2B_ADC_RATE, 0x0002 },
	{ NAU8811_R2C_DAC_CTRL1, 0x0082 },
	{ NAU8811_R2D_DAC_CTRL2, 0x0 },
	{ NAU8811_R30_ADC_DGAIN_CTRL, 0x0 },
	{ NAU8811_R31_MUTE_CTRL, 0x0 },
	{ NAU8811_R34_DAC_DGAIN_CTRL, 0xcfcf },
	{ NAU8811_R35_ADC_DGAIN_CTRL1, 0xcfcf },
	{ NAU8811_R36_ADC_DRC_KNEE_IP12, 0x1486 },
	{ NAU8811_R37_ADC_DRC_KNEE_IP34, 0x0f12 },
	{ NAU8811_R38_ADC_DRC_SLOPES, 0x25ff },
	{ NAU8811_R39_ADC_DRC_ATKDCY, 0x3457 },
	{ NAU8811_R3A_DAC_DRC_KNEE_IP12, 0x1486 },
	{ NAU8811_R3B_DAC_DRC_KNEE_IP34, 0x0f12 },
	{ NAU8811_R3C_DAC_DRC_SLOPES, 0x25f9 },
	{ NAU8811_R3D_DAC_DRC_ATKDCY, 0x3457 },
	{ NAU8811_R41_BIQ1_COF1, 0x0 },
	{ NAU8811_R42_BIQ1_COF2, 0x0 },
	{ NAU8811_R43_BIQ1_COF3, 0x0 },
	{ NAU8811_R44_BIQ1_COF4, 0x0 },
	{ NAU8811_R45_BIQ1_COF5, 0x0 },
	{ NAU8811_R46_BIQ1_COF6, 0x0 },
	{ NAU8811_R47_BIQ1_COF7, 0x0 },
	{ NAU8811_R48_BIQ1_COF8, 0x0 },
	{ NAU8811_R49_BIQ1_COF9, 0x0 },
	{ NAU8811_R4A_BIQ1_COF10, 0x0 },
	{ NAU8811_R4C_IMM_MODE_CTRL, 0x0 },
	{ NAU8811_R51_VCM_BUF, 0x0210 },
	{ NAU8811_R52_SPK_DRV, 0x0080 },
	{ NAU8811_R53_SPG_AMP_OFFSETDEC, 0x0 },
	{ NAU8811_R55_MISC_CTRL, 0x0 },
	{ NAU8811_R66_BIAS_ADJ, 0x0 },
	{ NAU8811_R69_SPARE_ANALOG, 0x0 },
	{ NAU8811_R6B_MUTE_CTL, 0x0 },
	{ NAU8811_R71_ANALOG_ADC_1, 0x0 },
	{ NAU8811_R72_ANALOG_ADC_2, 0x0100 },
	{ NAU8811_R73_DAC_CTRL, 0x0008 },
	{ NAU8811_R74_MIC_BIAS, 0x0004 },
	{ NAU8811_R76_BOOST, 0x0040 },
	{ NAU8811_R77_FEPGA, 0x0 },
	{ NAU8811_R7E_PGA_GAIN, 0x0 },
	{ NAU8811_R7F_POWER_UP_CONTROL, 0x0 },
};

static bool nau8811_readable_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case NAU8811_R01_ENA_CTRL:
	case NAU8811_R03_CLK_DIVIDER:
	case NAU8811_R08_PDB_CTL:
	case NAU8811_R0F_INTERRUPT_MASK ... NAU8811_R13_DMIC_CTRL:
	case NAU8811_R1B_TDM_CTRL ... NAU8811_R1E_LEFT_TIME_SLOT:
	case NAU8811_R21_BIQ0_COF1 ... NAU8811_R2D_DAC_CTRL2:
	case NAU8811_R30_ADC_DGAIN_CTRL ... NAU8811_R31_MUTE_CTRL:
	case NAU8811_R34_DAC_DGAIN_CTRL ... NAU8811_R3D_DAC_DRC_ATKDCY:
	case NAU8811_R41_BIQ1_COF1 ... NAU8811_R4A_BIQ1_COF10:
	case NAU8811_R4C_IMM_MODE_CTRL:
	case NAU8811_R51_VCM_BUF ... NAU8811_R53_SPG_AMP_OFFSETDEC:
	case NAU8811_R55_MISC_CTRL:
	case NAU8811_R58_I2C_DEVICE_ID ... NAU8811_R59_SARDOUT_RAM_STATUS:
	case NAU8811_R66_BIAS_ADJ:
	case NAU8811_R69_SPARE_ANALOG:
	case NAU8811_R6B_MUTE_CTL:
	case NAU8811_R71_ANALOG_ADC_1 ... NAU8811_R74_MIC_BIAS:
	case NAU8811_R76_BOOST ... NAU8811_R77_FEPGA:
	case NAU8811_R7E_PGA_GAIN ... NAU8811_R7F_POWER_UP_CONTROL:
	case NAU8811_R82_GENERAL_STATUS:
		return true;
	default:
		return false;
	}
}

static bool nau8811_writeable_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case NAU8811_R00_RESET ... NAU8811_R01_ENA_CTRL:
	case NAU8811_R03_CLK_DIVIDER:
	case NAU8811_R08_PDB_CTL:
	case NAU8811_R0F_INTERRUPT_MASK:
	case NAU8811_R11_INT_CLR_KEY_STATUS ... NAU8811_R13_DMIC_CTRL:
	case NAU8811_R1B_TDM_CTRL ... NAU8811_R1E_LEFT_TIME_SLOT:
	case NAU8811_R21_BIQ0_COF1 ... NAU8811_R2D_DAC_CTRL2:
	case NAU8811_R30_ADC_DGAIN_CTRL ... NAU8811_R31_MUTE_CTRL:
	case NAU8811_R34_DAC_DGAIN_CTRL ... NAU8811_R3D_DAC_DRC_ATKDCY:
	case NAU8811_R41_BIQ1_COF1 ... NAU8811_R4A_BIQ1_COF10:
	case NAU8811_R4C_IMM_MODE_CTRL:
	case NAU8811_R51_VCM_BUF ... NAU8811_R53_SPG_AMP_OFFSETDEC:
	case NAU8811_R55_MISC_CTRL:
	case NAU8811_R66_BIAS_ADJ:
	case NAU8811_R69_SPARE_ANALOG:
	case NAU8811_R6B_MUTE_CTL:
	case NAU8811_R71_ANALOG_ADC_1 ... NAU8811_R74_MIC_BIAS:
	case NAU8811_R76_BOOST ... NAU8811_R77_FEPGA:
	case NAU8811_R7E_PGA_GAIN ... NAU8811_R7F_POWER_UP_CONTROL:
		return true;
	default:
		return false;
	}
}

static bool nau8811_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case NAU8811_R00_RESET:
	case NAU8811_R10_IRQ_STATUS ... NAU8811_R11_INT_CLR_KEY_STATUS:
	case NAU8811_R21_BIQ0_COF1 ... NAU8811_R2A_BIQ0_COF10:
	case NAU8811_R41_BIQ1_COF1 ... NAU8811_R4A_BIQ1_COF10:
	case NAU8811_R58_I2C_DEVICE_ID ... NAU8811_R59_SARDOUT_RAM_STATUS:
	case NAU8811_R82_GENERAL_STATUS:
		return true;
	default:
		return false;
	}
}

static int nau8811_dac_biq_coeff_get(struct snd_kcontrol *kcontrol,
				     struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct nau8811 *nau8811 = snd_soc_component_get_drvdata(component);
	struct soc_bytes_ext *params = (void *)kcontrol->private_value;
	int i, reg, reg_val;
	u16 *val;

	val = (u16 *)ucontrol->value.bytes.data;
	reg = NAU8811_R41_BIQ1_COF1;
	for (i = 0; i < params->max / sizeof(u16); i++) {
		regmap_read(nau8811->regmap, reg + i, &reg_val);
		/* conversion of 16-bit integers between native CPU format and big endian format */
		reg_val = cpu_to_be16(reg_val);
		*(val + i) = reg_val;
	}

	return 0;
}

static int nau8811_dac_biq_coeff_put(struct snd_kcontrol *kcontrol,
				     struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct nau8811 *nau8811 = snd_soc_component_get_drvdata(component);
	struct soc_bytes_ext *params = (void *)kcontrol->private_value;
	void *data;
	u16 *val, value;
	int i, reg, ret;

	data = kmemdup(ucontrol->value.bytes.data, params->max, GFP_KERNEL | GFP_DMA);
	if (!data)
		return -ENOMEM;

	val = (u16 *)data;
	reg = NAU8811_R41_BIQ1_COF1;
	for (i = 0; i < params->max / sizeof(u16); i++) {
		/* conversion of 16-bit integers between native CPU format and big endian format */
		value = be16_to_cpu(*(val + i));
		ret = regmap_write(nau8811->regmap, reg + i, value);
		if (ret) {
			dev_err(component->dev,
				"DAC BIQ configuration fail, register: %x ret: %d\n",
				reg + i, ret);
			kfree(data);
			return ret;
		}
	}
	kfree(data);

	return 0;
}

static int nau8811_adc_biq_coeff_get(struct snd_kcontrol *kcontrol,
				     struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct nau8811 *nau8811 = snd_soc_component_get_drvdata(component);
	struct soc_bytes_ext *params = (void *)kcontrol->private_value;
	int i, reg, reg_val;
	u16 *val;

	val = (u16 *)ucontrol->value.bytes.data;
	reg = NAU8811_R21_BIQ0_COF1;
	for (i = 0; i < params->max / sizeof(u16); i++) {
		regmap_read(nau8811->regmap, reg + i, &reg_val);
		/* conversion of 16-bit integers between native CPU format and big endian format */
		reg_val = cpu_to_be16(reg_val);
		*(val + i) = reg_val;
	}

	return 0;
}

static int nau8811_adc_biq_coeff_put(struct snd_kcontrol *kcontrol,
				     struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct nau8811 *nau8811 = snd_soc_component_get_drvdata(component);
	struct soc_bytes_ext *params = (void *)kcontrol->private_value;
	void *data;
	u16 *val, value;
	int i, reg, ret;

	data = kmemdup(ucontrol->value.bytes.data, params->max, GFP_KERNEL | GFP_DMA);
	if (!data)
		return -ENOMEM;

	val = (u16 *)data;
	reg = NAU8811_R21_BIQ0_COF1;
	for (i = 0; i < params->max / sizeof(u16); i++) {
		/* conversion of 16-bit integers between native CPU format and big endian format */
		value = be16_to_cpu(*(val + i));
		ret = regmap_write(nau8811->regmap, reg + i, value);
		if (ret) {
			dev_err(component->dev,
				"ADC BIQ configuration fail, register: %x ret: %d\n",
				reg + i, ret);
			kfree(data);
			return ret;
		}
	}
	kfree(data);

	return 0;
}

static const char * const nau8811_tdm_slot[] = {"Slot 0", "Slot 1", "Slot 2", "Slot 3",
						"Slot 4", "Slot 5", "Slot 6", "Slot 7" };

static const struct soc_enum nau8811_dac_sel_enum =
	SOC_ENUM_SINGLE(NAU8811_R1B_TDM_CTRL, NAU8811_DAC_SEL_SFT,
			ARRAY_SIZE(nau8811_tdm_slot), nau8811_tdm_slot);

static const struct soc_enum nau8811_adc_sel_enum =
	SOC_ENUM_SINGLE(NAU8811_R1B_TDM_CTRL, NAU8811_ADC_TX_SEL_SFT,
			ARRAY_SIZE(nau8811_tdm_slot), nau8811_tdm_slot);

static const char * const nau8811_adc_decimation[] = {
	"32", "64", "128", "256" };

static const struct soc_enum nau8811_adc_decimation_enum =
	SOC_ENUM_SINGLE(NAU8811_R2B_ADC_RATE, NAU8811_OSR_ADC_RATE_SFT,
			ARRAY_SIZE(nau8811_adc_decimation), nau8811_adc_decimation);

static const char * const nau8811_dac_oversampl[] = {
	"64", "256", "128", "", "32" };

static const struct soc_enum nau8811_dac_oversampl_enum =
	SOC_ENUM_SINGLE(NAU8811_R2C_DAC_CTRL1, NAU8811_OSR_DAC_RATE_SFT,
			ARRAY_SIZE(nau8811_dac_oversampl), nau8811_dac_oversampl);

static const DECLARE_TLV_DB_MINMAX_MUTE(sidetone_vol_tlv, -4200, 0);
static const DECLARE_TLV_DB_MINMAX(dac_vol_tlv, -6600, 2400);
static const DECLARE_TLV_DB_MINMAX(adc_vol_tlv, -6600, 2400);
static const DECLARE_TLV_DB_MINMAX(spk_vol_tlv, 0, 560);
static const DECLARE_TLV_DB_MINMAX(fepga_gain_tlv, -100, 3600);

static const struct snd_kcontrol_new nau8811_controls[] = {
	SOC_SINGLE_TLV("Sidetone Volume", NAU8811_R30_ADC_DGAIN_CTRL,
		       NAU8811_ADC_TO_DAC_ST0_SFT, 0x0f, 0, sidetone_vol_tlv),
	SOC_SINGLE_TLV("Digital Playback Volume", NAU8811_R34_DAC_DGAIN_CTRL,
		       NAU8811_DGAIN_DAC_SFT, 0xff, 0, dac_vol_tlv),
	SOC_SINGLE_TLV("Mic Volume", NAU8811_R35_ADC_DGAIN_CTRL1,
		       NAU8811_DGAIN_ADC_SFT, 0xff, 0, adc_vol_tlv),
	SOC_SINGLE_TLV("Speaker Volume", NAU8811_R52_SPK_DRV,
		       NAU8811_SPK_GAIN_CNTRL_SFT, 0xf, 0, spk_vol_tlv),
	SOC_SINGLE_TLV("Frontend PGA Volume", NAU8811_R7E_PGA_GAIN,
		       NAU8811_SPK_GAIN_CNTRL_SFT, 37, 0, fepga_gain_tlv),

	SOC_ENUM("ADC Decimation Rate", nau8811_adc_decimation_enum),
	SOC_ENUM("DAC Oversampling Rate", nau8811_dac_oversampl_enum),
	SOC_ENUM("DAC Channel Source", nau8811_dac_sel_enum),
	SOC_ENUM("ADC Channel Source", nau8811_adc_sel_enum),

	SND_SOC_BYTES_EXT("DAC BIQ Coefficients", 20,
			  nau8811_dac_biq_coeff_get, nau8811_dac_biq_coeff_put),
	SND_SOC_BYTES_EXT("ADC BIQ Coefficients", 20,
			  nau8811_adc_biq_coeff_get, nau8811_adc_biq_coeff_put),
	SOC_SINGLE("ADC Phase Switch", NAU8811_R1B_TDM_CTRL,
		   NAU8811_ADCPHS0_SFT, 1, 0),
	SOC_SINGLE("DAC Phase Switch", NAU8811_R1B_TDM_CTRL,
		   NAU8811_DACPHS0_SFT, 1, 0),
};

static const struct snd_kcontrol_new nau8811_dmic_mode_switch =
	SOC_DAPM_SINGLE("Switch", NAU8811_R13_DMIC_CTRL,
			NAU8811_DMICEN_SFT, 1, 0);

static int dmic_clock_control(struct snd_soc_dapm_widget *w,
			      struct snd_kcontrol *k, int  event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);
	struct nau8811 *nau8811 = snd_soc_component_get_drvdata(component);
	int i, speed_selection = -1, clk_adc_src, clk_adc;
	unsigned int clk_divider_r03;

	/* The DMIC clock is gotten from adc clock divided by CLK_DMIC_SRC (1, 2, 4, 8).
	 * The clock has to be equal or less than nau8811->dmic_clk_threshold.
	 */
	regmap_read(nau8811->regmap, NAU8811_R03_CLK_DIVIDER, &clk_divider_r03);
	clk_adc_src = (clk_divider_r03 & NAU8811_CLK_ADC_SRC_MASK) >> NAU8811_CLK_ADC_SRC_SFT;
	clk_adc = (nau8811->fs * 256) >> clk_adc_src;

	for (i = 0 ; i < ARRAY_SIZE(dmic_speed_sel) ; i++)
		if ((clk_adc >> dmic_speed_sel[i].param) <= nau8811->dmic_clk_threshold) {
			speed_selection = dmic_speed_sel[i].val;
			break;
		}

	if (i == ARRAY_SIZE(dmic_speed_sel))
		return -EINVAL;

	dev_dbg(component->dev, "clk_adc=%d, dmic_clk_threshold = %d, param=%d, val = %d\n",
		clk_adc, nau8811->dmic_clk_threshold, dmic_speed_sel[i].param,
		dmic_speed_sel[i].val);

	regmap_update_bits(nau8811->regmap, NAU8811_R13_DMIC_CTRL,
			   NAU8811_CLK_DMIC_SRC_MASK,
			   (speed_selection << NAU8811_CLK_DMIC_SRC_SFT));

	return 0;
}

static int nau8811_input_adc_event(struct snd_soc_dapm_widget *w,
				   struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);
	struct nau8811 *nau8811 = snd_soc_component_get_drvdata(component);

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		/* Avoid related noise from possible DC offset */
		msleep(300);
		regmap_update_bits(nau8811->regmap, NAU8811_R31_MUTE_CTRL,
				   NAU8811_ADC_SMUTE_EN, 0);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		regmap_update_bits(nau8811->regmap, NAU8811_R31_MUTE_CTRL,
				   NAU8811_ADC_SMUTE_EN, NAU8811_ADC_SMUTE_EN);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int nau8811_dac_event(struct snd_soc_dapm_widget *w,
			     struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);
	struct nau8811 *nau8811 = snd_soc_component_get_drvdata(component);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		regmap_update_bits(nau8811->regmap, NAU8811_R01_ENA_CTRL,
				   NAU8811_DCLK_DAC_EN, NAU8811_DCLK_DAC_EN);
		break;
	case SND_SOC_DAPM_POST_PMD:
		regmap_update_bits(nau8811->regmap, NAU8811_R01_ENA_CTRL,
				   NAU8811_DCLK_DAC_EN, NAU8811_DCLK_DAC_EN);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int nau8811_vcm_event(struct snd_soc_dapm_widget *w,
			     struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);
	struct nau8811 *nau8811 = snd_soc_component_get_drvdata(component);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		regmap_update_bits(nau8811->regmap, NAU8811_R51_VCM_BUF,
				   NAU8811_VOUT_PRECHG_DISABLE_MASK,
				   NAU8811_VOUT_PRECHG_DISABLE_OFF);
		msleep(600);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int nau8811_output_dac_event(struct snd_soc_dapm_widget *w,
				    struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);
	struct nau8811 *nau8811 = snd_soc_component_get_drvdata(component);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		regmap_update_bits(nau8811->regmap, NAU8811_R73_DAC_CTRL,
				   NAU8811_CLK_DAC_EN, NAU8811_CLK_DAC_EN);
		break;
	case SND_SOC_DAPM_POST_PMD:
		regmap_update_bits(nau8811->regmap, NAU8811_R73_DAC_CTRL,
				   NAU8811_CLK_DAC_EN, 0);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int nau8811_i2s_output_event(struct snd_soc_dapm_widget *w,
				    struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);
	struct nau8811 *nau8811 = snd_soc_component_get_drvdata(component);

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		regmap_update_bits(nau8811->regmap, NAU8811_R51_VCM_BUF,
				   NAU8811_VOUT_PRECHG_DISABLE_MASK,
				   NAU8811_VOUT_PRECHG_DISABLE_ON);
		regmap_update_bits(nau8811->regmap, NAU8811_R1D_I2S_PCM_CTRL2,
				   NAU8811_I2S_TRI, 0);
		/* Disables the TESTDAC to let DAC signal pass through. */
		regmap_update_bits(nau8811->regmap, NAU8811_R66_BIAS_ADJ,
				   NAU8811_TESTDAC_EN, 0);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		/* Disables the TESTDAC to let DAC signal pass through. */
		regmap_update_bits(nau8811->regmap, NAU8811_R66_BIAS_ADJ,
				   NAU8811_TESTDAC_EN, NAU8811_TESTDAC_EN);
		regmap_update_bits(nau8811->regmap, NAU8811_R1D_I2S_PCM_CTRL2,
				   NAU8811_I2S_TRI, NAU8811_I2S_TRI);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static const struct snd_soc_dapm_widget nau8811_dapm_widgets[] = {
	SND_SOC_DAPM_SUPPLY("DMIC Clock", SND_SOC_NOPM, 0, 0,
			    dmic_clock_control, SND_SOC_DAPM_POST_PMU),

	SND_SOC_DAPM_SWITCH("DMIC Mode", SND_SOC_NOPM,
			    0, 0, &nau8811_dmic_mode_switch),

	SND_SOC_DAPM_SUPPLY("MICBIAS", NAU8811_R74_MIC_BIAS,
			    NAU8811_POWERUP_SFT, 0, NULL, 0),

	SND_SOC_DAPM_ADC_E("ADC Power", NULL, NAU8811_R72_ANALOG_ADC_2,
			   NAU8811_PDNOT_SFT, 0, nau8811_input_adc_event,
			   SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),

	SND_SOC_DAPM_PGA("Frontend PGA", NAU8811_R7F_POWER_UP_CONTROL,
			 NAU8811_PUPL_SFT, 0, NULL, 0),

	SND_SOC_DAPM_ADC("ADC", NULL, NAU8811_R01_ENA_CTRL,
			 NAU8811_ADCEN_SFT, 0),

	SND_SOC_DAPM_SUPPLY("ADC Clock", NAU8811_R01_ENA_CTRL,
			    NAU8811_DCLK_ADC_EN_SFT, 0, NULL, 0),

	SND_SOC_DAPM_AIF_OUT("AIFTX", "Capture", 0, NAU8811_R1D_I2S_PCM_CTRL2,
			     NAU8811_I2S_TRI_SFT, 1),

	SND_SOC_DAPM_PGA_S("DAC", 4, NAU8811_R01_ENA_CTRL,
			   NAU8811_DACEN_SFT, 0, nau8811_dac_event,
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_AIF_IN("AIFRX", "Playback", 0, SND_SOC_NOPM, 0, 0),

	SND_SOC_DAPM_PGA_S("VCM Buffer", 0, NAU8811_R51_VCM_BUF,
			   NAU8811_PDB_VCMBUF_SFT, 0, nau8811_vcm_event,
			   SND_SOC_DAPM_PRE_PMU),

	SND_SOC_DAPM_PGA_S("VREF Buffer", 1, NAU8811_R08_PDB_CTL,
			   NAU8811_PDB_DAC_SFT, 0, NULL, 0),

	SND_SOC_DAPM_PGA_S("Main Driver", 2, NAU8811_R52_SPK_DRV,
			   NAU8811_PUP_MAIN_DRV_SFT, 0, NULL, 0),

	SND_SOC_DAPM_PGA_S("Output DAC", 3, NAU8811_R73_DAC_CTRL,
			   NAU8811_DAC_SFT, 0, nau8811_output_dac_event,
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_PGA_S("I2S Output", 5, SND_SOC_NOPM,
			   0, 0, nau8811_i2s_output_event,
			   SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),

	SND_SOC_DAPM_INPUT("MIC"),
	SND_SOC_DAPM_INPUT("DMIC"),
	SND_SOC_DAPM_OUTPUT("SPK"),
};

static const struct snd_soc_dapm_route nau8811_dapm_routes[] = {
	{"DMIC Mode", "Switch", "DMIC"},
	{"DMIC Mode", NULL, "DMIC Clock"},
	{"ADC", NULL, "DMIC Mode"},

	{"Frontend PGA", NULL, "MICBIAS"},
	{"Frontend PGA", NULL, "MIC"},
	{"ADC Power", NULL, "Frontend PGA"},
	{"ADC", NULL, "ADC Power"},
	{"ADC", NULL, "ADC Clock"},
	{"AIFTX", NULL, "ADC"},

	{"VCM Buffer", NULL, "AIFRX"},
	{"VREF Buffer", NULL, "VCM Buffer"},
	{"Main Driver", NULL, "VREF Buffer"},
	{"Output DAC", NULL, "Main Driver"},
	{"DAC", NULL, "Output DAC"},
	{"I2S Output", NULL, "DAC"},
	{"SPK", NULL, "I2S Output"},
};

static int nau8811_clksrc_div(struct nau8811 *nau8811,
			      const struct nau8811_srate_attr *srate_table, unsigned int mclk)
{
	int i, ratio;
	unsigned int mclk_int;

	ratio = NAU8811_MCLK_FS_RATIO_NUM;
	for (i = 0; i < ARRAY_SIZE(mclk_src_div); i++) {
		if (mclk % mclk_src_div[i].param != 0)
			continue;
		else {
			mclk_int = mclk / mclk_src_div[i].param;
			if (srate_table->mclk_src[NAU8811_MCLK_FS_RATIO_256] == mclk_int) {
				ratio = NAU8811_MCLK_FS_RATIO_256;
				break;
			} else if (srate_table->mclk_src[NAU8811_MCLK_FS_RATIO_400] == mclk_int) {
				ratio = NAU8811_MCLK_FS_RATIO_400;
				break;
			} else if (srate_table->mclk_src[NAU8811_MCLK_FS_RATIO_500] == mclk_int) {
				ratio = NAU8811_MCLK_FS_RATIO_500;
				break;
			}
		}
	}
	if (i == ARRAY_SIZE(mclk_src_div))
		return ratio;

	if (ratio != NAU8811_MCLK_FS_RATIO_NUM) {
		regmap_update_bits(nau8811->regmap, NAU8811_R03_CLK_DIVIDER,
				   NAU8811_CLK_MCLK_SRC_MASK, mclk_src_div[i].val);
		dev_dbg(nau8811->dev, "mclk should be divided %d!\n", mclk_src_div[i].param);
	}
	return ratio;
}

static const struct nau8811_srate_attr *target_srate_attribute(unsigned int srate)
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

static void nau8811_set_freq_range(struct nau8811 *nau8811)
{
	if (nau8811->mclk >= MASTER_CLK_MIN && nau8811->mclk <= MCLK_RNG_15)
		regmap_update_bits(nau8811->regmap, NAU8811_R01_ENA_CTRL,
				   NAU8811_MCLK_RNG_SEL_MASK, NAU8811_MCLK_RNG_SEL_0);
	else if  (nau8811->mclk > MCLK_RNG_15 && nau8811->mclk <= MCLK_RNG_21)
		regmap_update_bits(nau8811->regmap, NAU8811_R01_ENA_CTRL,
				   NAU8811_MCLK_RNG_SEL_MASK, NAU8811_MCLK_RNG_SEL_4);
	else if (nau8811->mclk > MCLK_RNG_21 && nau8811->mclk <= MASTER_CLK_MAX)
		regmap_update_bits(nau8811->regmap, NAU8811_R01_ENA_CTRL,
				   NAU8811_MCLK_RNG_SEL_MASK, NAU8811_MCLK_RNG_SEL_7);
}

static int nau8811_clock_config(struct nau8811 *nau8811)
{
	struct nau8811_srate_attr tmp_table, *srate_table = &tmp_table;
	int i = 0, mult_sel, ratio;
	unsigned int mult_mclk = 0;

	if (!nau8811->mclk || !nau8811->fs) {
		dev_err(nau8811->dev, "mclk or fs is empty!\n");
		goto proc_err;
	}
	/* select sampling rate and MCLK_SRC */
	srate_table = (struct nau8811_srate_attr *)target_srate_attribute(nau8811->fs);
	if (srate_table == NULL) {
		dev_err(nau8811->dev, "srate table is empty!\n");
		goto proc_err;
	}
	/* First check clock from MCLK directly, decide the div for MCLK_INT.
	 * If not good, consider Multiplier.
	 */
	ratio = nau8811_clksrc_div(nau8811, srate_table, nau8811->mclk);
	dev_dbg(nau8811->dev, "ratio = %d!\n", ratio);
	if (ratio != NAU8811_MCLK_FS_RATIO_NUM) {
		mult_sel = NAU8811_SYSCLK_SEL_MCLK;
		regmap_update_bits(nau8811->regmap, NAU8811_R01_ENA_CTRL,
				   NAU8811_SYSCLK_SEL_MASK, mult_sel);
		dev_dbg(nau8811->dev, "MCLK X1 case!\n");
		goto proc_done;
	} else {
		mult_sel = NAU8811_SYSCLK_SEL_2MCLK;
		regmap_update_bits(nau8811->regmap, NAU8811_R01_ENA_CTRL,
				   NAU8811_SYSCLK_SEL_MASK, mult_sel);
		for (i = 0; i < ARRAY_SIZE(mclk_src_mult); i++) {
			mult_mclk = mclk_src_mult[i].param * nau8811->mclk;
			ratio = nau8811_clksrc_div(nau8811, srate_table, mult_mclk);
			if (ratio != NAU8811_MCLK_FS_RATIO_NUM) {
				nau8811_set_freq_range(nau8811);
				dev_dbg(nau8811->dev, "MCLK X2 case!\n");
				goto proc_done;
			} else {
				goto proc_err;
			}
		}
	}

proc_err:
	return -EINVAL;
proc_done:
	dev_dbg(nau8811->dev, "Clock settings configure successfully!\n");
	return 0;
}

static const struct nau8811_osr_attr *
nau8811_get_osr(struct nau8811 *nau8811, int stream)
{
	unsigned int osr;

	if (stream == SNDRV_PCM_STREAM_PLAYBACK) {
		regmap_read(nau8811->regmap, NAU8811_R2C_DAC_CTRL1, &osr);
		osr &= NAU8811_OSR_DAC_RATE_MASK;
		if (osr >= ARRAY_SIZE(osr_dac_sel))
			return NULL;
		return &osr_dac_sel[osr];
	} else {
		regmap_read(nau8811->regmap, NAU8811_R2B_ADC_RATE, &osr);
		osr &= NAU8811_OSR_ADC_RATE_MASK;
		if (osr >= ARRAY_SIZE(osr_adc_sel))
			return NULL;
		return &osr_adc_sel[osr];
	}
}

static int nau8811_dai_startup(struct snd_pcm_substream *substream,
			       struct snd_soc_dai *dai)
{
	struct snd_soc_component *component = dai->component;
	struct nau8811 *nau8811 = snd_soc_component_get_drvdata(component);
	const struct nau8811_osr_attr *osr;

	osr = nau8811_get_osr(nau8811, substream->stream);
	if (!osr || !osr->osr)
		return -EINVAL;

	return snd_pcm_hw_constraint_minmax(substream->runtime,
					    SNDRV_PCM_HW_PARAM_RATE,
					    0, CLK_DA_AD_MAX / osr->osr);
}

static int nau8811_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	struct snd_soc_component *component = dai->component;
	struct nau8811 *nau8811 = snd_soc_component_get_drvdata(component);
	unsigned int val_len = 0, ctrl_val, bclk_fs, clk_div;
	const struct nau8811_osr_attr *osr;
	int ret=-EINVAL;

	nau8811->fs = params_rate(params);
	if (nau8811->clk_src_sel == INTERNAL_BUF_MCLK) {
		ret = nau8811_clock_config(nau8811);
		if (ret)
			goto err;
	}

	/* CLK_DAC or CLK_ADC = OSR * FS
	 * DAC or ADC clock frequency is defined as Over Sampling Rate (OSR)
	 * multiplied by the audio sample rate (Fs). Note that the OSR and Fs
	 * values must be selected such that the maximum frequency is less
	 * than 6.144 MHz.
	 */
	osr = nau8811_get_osr(nau8811, substream->stream);
	if (!osr || !osr->osr)
		goto err;
	if (params_rate(params) * osr->osr > CLK_DA_AD_MAX)
		goto err;
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		regmap_update_bits(nau8811->regmap, NAU8811_R03_CLK_DIVIDER,
				   NAU8811_CLK_DAC_SRC_MASK,
				   osr->clk_src << NAU8811_CLK_DAC_SRC_SFT);
	else
		regmap_update_bits(nau8811->regmap, NAU8811_R03_CLK_DIVIDER,
				   NAU8811_CLK_ADC_SRC_MASK,
				   osr->clk_src << NAU8811_CLK_ADC_SRC_SFT);

	/* make BCLK and LRC divde configuration if the codec as master. */
	regmap_read(nau8811->regmap, NAU8811_R1D_I2S_PCM_CTRL2, &ctrl_val);
	if (ctrl_val & NAU8811_MS_MASTER) {
		/* get the bclk and fs ratio */
		bclk_fs = snd_soc_params_to_bclk(params) / nau8811->fs;
		if (bclk_fs <= 32)
			clk_div = NAU8811_BCLKDIV_8;
		else if (bclk_fs <= 64)
			clk_div = NAU8811_BCLKDIV_4;
		else if (bclk_fs <= 128)
			clk_div = NAU8811_BCLKDIV_2;
		else
			return -EINVAL;
		regmap_update_bits(nau8811->regmap, NAU8811_R1D_I2S_PCM_CTRL2,
				   NAU8811_LRC_DIV_MASK | NAU8811_BCLKDIV_MASK,
				   (clk_div << NAU8811_LRC_DIV_SFT) | clk_div);
	}

	switch (params_width(params)) {
	case 16:
		val_len |= NAU8811_WLEN0_16;
		break;
	case 20:
		val_len |= NAU8811_WLEN0_20;
		break;
	case 24:
		val_len |= NAU8811_WLEN0_24;
		break;
	case 32:
		val_len |= NAU8811_WLEN0_32;
		break;
	default:
		return -EINVAL;
	}

	regmap_update_bits(nau8811->regmap, NAU8811_R1C_I2S_PCM_CTRL1,
			   NAU8811_WLEN0_MASK, val_len);

	return 0;
err:
	return ret;
}

static int nau8811_set_dai_fmt(struct snd_soc_dai *codec_dai, unsigned int fmt)
{
	struct snd_soc_component *component = codec_dai->component;
	struct nau8811 *nau8811 = snd_soc_component_get_drvdata(component);
	unsigned int ctrl1_val = 0, ctrl2_val = 0;

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBP_CFP:
		ctrl2_val |= NAU8811_MS_MASTER;
		break;
	case SND_SOC_DAIFMT_CBC_CFC:
		break;
	default:
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		break;
	case SND_SOC_DAIFMT_IB_NF:
		ctrl1_val |= NAU8811_BCP0_INV;
		break;
	default:
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		ctrl1_val |= NAU8811_AIFMT0_I2S;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		ctrl1_val |= NAU8811_AIFMT0_LEFT;
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		ctrl1_val |= NAU8811_AIFMT0_RIGHT;
		break;
	case SND_SOC_DAIFMT_DSP_A:
		ctrl1_val |= NAU8811_AIFMT0_PCM_AB;
		break;
	case SND_SOC_DAIFMT_DSP_B:
		ctrl1_val |= NAU8811_AIFMT0_PCM_AB;
		ctrl1_val |= NAU8811_LRP0_PCMB_EN;
		break;
	default:
		return -EINVAL;
	}

	regmap_update_bits(nau8811->regmap, NAU8811_R1C_I2S_PCM_CTRL1,
			   NAU8811_AIFMT0_MASK | NAU8811_BCP0_MASK |
			   NAU8811_LRP0_MASK, ctrl1_val);
	regmap_update_bits(nau8811->regmap, NAU8811_R1D_I2S_PCM_CTRL2,
			   NAU8811_MS_MASK, ctrl2_val);

	return 0;
}

static int nau8811_digital_mute(struct snd_soc_dai *dai, int mute, int direction)
{
	struct snd_soc_component *component = dai->component;
	struct nau8811 *nau8811 = snd_soc_component_get_drvdata(component);

	dev_dbg(component->dev, "%s: %d\n", __func__, mute);

	regmap_update_bits(nau8811->regmap, NAU8811_R31_MUTE_CTRL,
			   NAU8811_SMUTE_EN, mute ? NAU8811_SMUTE_EN : 0);

	return 0;
}

static const struct snd_soc_dai_ops nau8811_dai_ops = {
	.startup		= nau8811_dai_startup,
	.hw_params		= nau8811_hw_params,
	.set_fmt		= nau8811_set_dai_fmt,
	.mute_stream		= nau8811_digital_mute,
	.no_capture_mute	= 1,
};

#define NAU8811_RATES SNDRV_PCM_RATE_8000_96000
#define NAU8811_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE \
			 | SNDRV_PCM_FMTBIT_S24_3LE | SNDRV_PCM_FMTBIT_S32_LE)

static struct snd_soc_dai_driver nau8811_dai = {
	.name = NUVOTON_CODEC_DAI,
	.playback = {
		.stream_name = "Playback",
		.channels_min = 1,
		.channels_max = 2,   /* Only 1 channel of data */
		.rates = NAU8811_RATES,
		.formats = NAU8811_FORMATS,
	},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = 2,   /* Only 1 channel of data */
		.rates = NAU8811_RATES,
		.formats = NAU8811_FORMATS,
	},
	.ops = &nau8811_dai_ops,
	.symmetric_rate = 1,
};

static const struct regmap_config nau8811_regmap_config = {
	.val_bits = NAU8811_REG_DATA_LEN,
	.reg_bits = NAU8811_REG_ADDR_LEN,

	.max_register = NAU8811_REG_MAX,
	.readable_reg = nau8811_readable_reg,
	.writeable_reg = nau8811_writeable_reg,
	.volatile_reg = nau8811_volatile_reg,

	.cache_type = REGCACHE_RBTREE,
	.reg_defaults = nau8811_reg_defaults,
	.num_reg_defaults = ARRAY_SIZE(nau8811_reg_defaults),
};

static int nau8811_set_sysclk(struct snd_soc_component *component, int clk_id,
			      int source, unsigned int freq, int dir)
{
	struct nau8811 *nau8811 = snd_soc_component_get_drvdata(component);

	nau8811->clk_src_sel = clk_id;
	switch (clk_id) {
	case INTERNAL_BUF_MCLK:
		if (freq < MASTER_CLK_MIN || freq > MASTER_CLK_MAX) {
			dev_err(nau8811->dev,
				"Exceed the range of input clocks, MCLK %dHz\n", freq);
			return -EINVAL;
		}
		break;
	case EXTERNAL_MCLK:
		break;
	default:
		return -EINVAL;
	}

	regmap_update_bits(nau8811->regmap, NAU8811_R03_CLK_DIVIDER,
			   NAU8811_CLK_CODEC_SRC_MASK, clk_id << NAU8811_CLK_CODEC_SRC_SFT);
	nau8811->mclk = freq;
	dev_dbg(nau8811->dev, "%s, MCLK %dHz\n", __func__, nau8811->mclk);

	return  0;
}

static const struct snd_soc_component_driver nau8811_component_driver = {
	.set_sysclk		= nau8811_set_sysclk,
	.controls		= nau8811_controls,
	.num_controls		= ARRAY_SIZE(nau8811_controls),
	.dapm_widgets		= nau8811_dapm_widgets,
	.num_dapm_widgets	= ARRAY_SIZE(nau8811_dapm_widgets),
	.dapm_routes		= nau8811_dapm_routes,
	.num_dapm_routes	= ARRAY_SIZE(nau8811_dapm_routes),
	.suspend_bias_off	= 1,
	.idle_bias_on		= 1,
	.use_pmdown_time	= 1,
	.endianness		= 1,
};

static void nau8811_reset_chip(struct regmap *regmap)
{
	regmap_write(regmap, NAU8811_R00_RESET, 0xffff);
	regmap_write(regmap, NAU8811_R00_RESET, 0xffff);
}

static void nau8811_print_device_properties(struct nau8811 *nau8811)
{
	struct device *dev = nau8811->dev;

	dev_dbg(dev, "micbias-voltage:      %d\n", nau8811->micbias_voltage);
	dev_dbg(dev, "vref-impedance:       %d\n", nau8811->vref_impedance);
	dev_dbg(dev, "dmic-clk-threshold:   %d\n", nau8811->dmic_clk_threshold);
}

static int nau8811_read_device_properties(struct device *dev, struct nau8811 *nau8811)
{
	int ret;

	ret = device_property_read_u32(dev, "nuvoton,micbias-voltage",
		&nau8811->micbias_voltage);
	if (ret)
		nau8811->micbias_voltage = 5;
	ret = device_property_read_u32(dev, "nuvoton,vref-impedance",
		&nau8811->vref_impedance);
	if (ret)
		nau8811->vref_impedance = 2;
	ret = device_property_read_u32(dev, "nuvoton,dmic-clk-threshold",
		&nau8811->dmic_clk_threshold);
	if (ret)
		nau8811->dmic_clk_threshold = DMIC_CLK;

	return 0;
}

static void nau8811_init_regs(struct nau8811 *nau8811)
{
	struct regmap *regmap = nau8811->regmap;

	/* Keep slow rising VREF */
	regmap_update_bits(regmap, NAU8811_R76_BOOST,
			   NAU8811_PDVMDFST_DIS, NAU8811_PDVMDFST_DIS);
	msleep(600);
	regmap_update_bits(regmap, NAU8811_R66_BIAS_ADJ,
			   NAU8811_VMIDEN_EN, NAU8811_VMIDEN_EN);
	regmap_update_bits(regmap, NAU8811_R76_BOOST,
			   NAU8811_BIASEN_EN, NAU8811_BIASEN_EN);

	/* VMID Tieoff setting and enable TESTDAC.
	 * This sets the analog DAC inputs to a '0' input signal to avoid
	 * any glitches due to power up transients in both the analog and
	 * digital DAC circuit.
	 */
	regmap_update_bits(regmap, NAU8811_R66_BIAS_ADJ,
			   NAU8811_VMIDSEL_MASK | NAU8811_TESTDAC_EN,
			   (nau8811->vref_impedance << NAU8811_VMIDSEL_SFT) |
			   NAU8811_TESTDAC_EN);
	/* Disable short Frame Sync detection logic */
	regmap_update_bits(regmap, NAU8811_R1E_LEFT_TIME_SLOT,
			   NAU8811_DIS_FS_SHORT_DET_DIS, NAU8811_DIS_FS_SHORT_DET_DIS);
	/* Automatic Short circuit protection enable */
	regmap_update_bits(regmap, NAU8811_R76_BOOST,
			   NAU8811_DISABLE_SHRT_DET_EN, NAU8811_DISABLE_SHRT_DET_EN);
	/* DAC clock delay 0ns, VREF 1.61V */
	regmap_update_bits(regmap, NAU8811_R73_DAC_CTRL,
			   NAU8811_CLK_DAC_DELAY_MASK | NAU8811_DACVREFSEL_MASK,
			   (0x0 << NAU8811_CLK_DAC_DELAY_SFT) |
			   (0x2 << NAU8811_DACVREFSEL_SFT));
	/* Fine tuning DAC out glitch */
	regmap_update_bits(regmap, NAU8811_R2C_DAC_CTRL1,
			   NAU8811_CICCLP_OFF_MASK, NAU8811_CICCLP_OFF_DIS);
	regmap_update_bits(regmap, NAU8811_R2C_DAC_CTRL1,
			   NAU8811_CIC_GAIN_ADJ_MASK, 0x7);
	/* THD_BOOST enable */
	regmap_update_bits(regmap, NAU8811_R69_SPARE_ANALOG,
			   NAU8811_THD_BOOST_EN, NAU8811_THD_BOOST_EN);
	/* Set default MICBIAS voltage */
	regmap_update_bits(regmap, NAU8811_R74_MIC_BIAS,
			   NAU8811_MICBIASLVL1_MASK, nau8811->micbias_voltage);
	/* Mute ADC */
	regmap_update_bits(nau8811->regmap, NAU8811_R31_MUTE_CTRL,
			   NAU8811_ADC_SMUTE_EN, NAU8811_ADC_SMUTE_EN);

	/* Default oversampling/decimations settings are unusable
	 * (audible hiss). Set it to something better.
	 */
	regmap_update_bits(regmap, NAU8811_R2B_ADC_RATE,
			   NAU8811_OSR_ADC_RATE_MASK, NAU8811_OSR_ADC_RATE_DOWN_128);
	regmap_update_bits(regmap, NAU8811_R2C_DAC_CTRL1,
			   NAU8811_OSR_DAC_RATE_MASK, NAU8811_OSR_DAC_RATE_DOWN_128);
}

static int nau8811_i2c_probe(struct i2c_client *i2c)
{
	struct device *dev = &i2c->dev;
	struct nau8811 *nau8811 = dev_get_platdata(&i2c->dev);
	int ret, value;

	if (!nau8811) {
		nau8811 = devm_kzalloc(dev, sizeof(*nau8811), GFP_KERNEL);
		if (!nau8811)
			return -ENOMEM;
		nau8811_read_device_properties(dev, nau8811);
	}

	i2c_set_clientdata(i2c, nau8811);
	nau8811->regmap = devm_regmap_init_i2c(i2c, &nau8811_regmap_config);
	if (IS_ERR(nau8811->regmap))
		return PTR_ERR(nau8811->regmap);

	nau8811->dev = dev;
	nau8811_print_device_properties(nau8811);

	nau8811_reset_chip(nau8811->regmap);
	ret = regmap_read(nau8811->regmap, NAU8811_R58_I2C_DEVICE_ID, &value);
	if (ret) {
		dev_err(dev, "Failed to read device id (%d)\n", ret);
		return ret;
	}
	nau8811_init_regs(nau8811);

	ret = devm_snd_soc_register_component(&i2c->dev,
					      &nau8811_component_driver, &nau8811_dai, 1);

	return ret;
}

static const struct i2c_device_id nau8811_i2c_ids[] = {
	{ "nau8811", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, nau8811_i2c_ids);

#ifdef CONFIG_OF
static const struct of_device_id nau8811_of_ids[] = {
	{ .compatible = "nuvoton,nau8811", },
	{}
};
MODULE_DEVICE_TABLE(of, nau8811_of_ids);
#endif

#ifdef CONFIG_ACPI
static const struct acpi_device_id nau8811_acpi_match[] = {
	{ "NVTN2021", 0 },
	{},
};
MODULE_DEVICE_TABLE(acpi, nau8811_acpi_match);
#endif

static struct i2c_driver nau8811_driver = {
	.driver = {
		.name = "nau8811",
		.of_match_table = of_match_ptr(nau8811_of_ids),
		.acpi_match_table = ACPI_PTR(nau8811_acpi_match),
	},
	.probe_new = nau8811_i2c_probe,
	.id_table = nau8811_i2c_ids,
};
module_i2c_driver(nau8811_driver);

MODULE_DESCRIPTION("ASoC NAU8811 driver");
MODULE_AUTHOR("David Lin <ctlinu0@nuvoton.com>");
MODULE_LICENSE("GPL");

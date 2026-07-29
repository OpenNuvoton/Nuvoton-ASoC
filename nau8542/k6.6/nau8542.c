// SPDX-License-Identifier: GPL-2.0-only
//
// NAU85L42 ALSA SoC audio ADC driver
//
// Copyright (C) 2026 Nuvoton Technology Corp.
// Author: John Hsu <KCHSU0@nuvoton.com>
//         Neo Chang <ylchang2@nuvoton.com>

#define DEBUG
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/tlv.h>
#include "nau8542.h"

#define NAU_FREF_MAX 13500000
#define NAU_FVCO_MAX 100000000
#define NAU_FVCO_MIN 90000000

/* the maximum frequency of CLK_ADC */
#define CLK_ADC_MAX 6144000

/* scaling for mclk from sysclk_src output */
static const struct nau8542_fll_attr mclk_src_scaling[] = {
	{ 1, 0x0 },
	{ 2, 0x2 },
	{ 4, 0x3 },
	{ 8, 0x4 },
	{ 16, 0x5 },
	{ 32, 0x6 },
	{ 3, 0x7 },
	{ 6, 0xa },
	{ 12, 0xb },
	{ 24, 0xc },
};

/* ratio for input clk freq */
static const struct nau8542_fll_attr fll_ratio[] = {
	{ 512000, 0x01 },
	{ 256000, 0x02 },
	{ 128000, 0x04 },
	{ 64000, 0x08 },
	{ 32000, 0x10 },
	{ 8000, 0x20 },
	{ 4000, 0x40 },
};

static const struct nau8542_fll_attr fll_pre_scalar[] = {
	{ 1, 0x0 },
	{ 2, 0x1 },
	{ 4, 0x2 },
	{ 8, 0x3 },
};

/* over sampling rate */
static const struct nau8542_osr_attr osr_adc_sel[] = {
	{ 256, 0 },	/* OSR 256, SRC 1 */
	{ 128, 1 },	/* OSR 128, SRC 1/2 */
	{ 64, 2 },	/* OSR 64, SRC 1/4 */
	{ 32, 3 },	/* OSR 32, SRC 1/8 */
};

static const struct reg_default nau8542_reg_defaults[] = {
	{ NAU8542_R01_POWER_MANAGEMENT, 0x0000 },
	{ NAU8542_R02_CLOCK_CTRL, 0x0000 },
	{ NAU8542_R03_CLOCK_SRC, 0x0010 },
	{ NAU8542_R04_FLL1, 0x0001 },
	{ NAU8542_R05_FLL2, 0x26E9 },
	{ NAU8542_R06_FLL3, 0x0231 },
	{ NAU8542_R07_FLL4, 0x0010 },
	{ NAU8542_R08_FLL5, 0xD008 },
	{ NAU8542_R09_FLL6, 0x6000 },
	{ NAU8542_R0A_FLL_VCO_RSV, 0x00BC },
	{ NAU8542_R10_TDM_CTRL0, 0x000E },
	{ NAU8542_R11_TDM_CTRL1, 0x2010 },
	{ NAU8542_R12_TDM_CTRL2, 0x0800 },
	{ NAU8542_R13_TDM_CTRL3, 0x3C00 },
	{ NAU8542_R14_PWRST_CTRL, 0x3E7F },
	{ NAU8542_R20_ALC_CONTROL_1, 0x0009 },
	{ NAU8542_R21_ALC_CONTROL_2, 0x700B },
	{ NAU8542_R22_ALC_CONTROL_3, 0x0222 },
	{ NAU8542_R23_ALC_CONTROL_4, 0xD0D0 },
	{ NAU8542_R24_ALC_CONTROL_5, 0xD0D0 },
	{ NAU8542_R25_ALC_CONTROL_6, 0xD000 },
	{ NAU8542_R26_MIC_DIAG_CTRL_1, 04000 },
	{ NAU8542_R30_HPF_FILTER_CH12, 0x0000 },
	{ NAU8542_R31_HPF_FILTER_CH34, 0x0000 },
	{ NAU8542_R32_ADC_FILTER_CTRL, 0x11A0 },
	{ NAU8542_R33_BQ_CHANNEL_SEL, 0x0000 },
	{ NAU8542_R34_BQ_COEFF_B2L, 0x0000 },
	{ NAU8542_R35_BQ_COEFF_B2H, 0x0000 },
	{ NAU8542_R36_BQ_COEFF_A2L, 0x0000 },
	{ NAU8542_R37_BQ_COEFF_A2H, 0x0000 },
	{ NAU8542_R38_BQ_COEFF_B1L, 0x0000 },
	{ NAU8542_R39_BQ_COEFF_B1H, 0x0000 },
	{ NAU8542_R3A_BQ_COEFF_A1L, 0x0000 },
	{ NAU8542_R3B_BQ_COEFF_A1H, 0x0000 },
	{ NAU8542_R3C_BQ_COEFF_B0L, 0x0000 },
	{ NAU8542_R3D_BQ_COEFF_B0H, 0x0000 },
	{ NAU8542_R40_DIGITAL_GAIN_CH1, 0x0400 },
	{ NAU8542_R41_DIGITAL_GAIN_CH2, 0x0400 },
	{ NAU8542_R42_DIGITAL_GAIN_CH3, 0x0400 },
	{ NAU8542_R43_DIGITAL_GAIN_CH4, 0x0400 },
	{ NAU8542_R44_DIGITAL_MUX, 0x00E4 },
	{ NAU8542_R50_GPIO_CTRL, 0x0002 },
	{ NAU8542_R51_MISC_CTRL, 0x0000 },
	{ NAU8542_R52_I2C_CTRL, 0xEFFF },
	{ NAU8542_R5A_SW_RST, 0x0000 },
	{ NAU8542_R5B_HW_RST, 0x0000 },
	{ NAU8542_R60_VMID_CTRL, 0x0000 },
	{ NAU8542_R61_MUTE, 0x0000 },
	{ NAU8542_R62_ANALOG_SPARE1, 0x0000 },
	{ NAU8542_R64_ANALOG_ADC1, 0x0000 },
	{ NAU8542_R65_ANALOG_ADC2, 0x0020 },
	{ NAU8542_R66_ANALOG_PWR, 0x0000 },
	{ NAU8542_R67_MIC_BIAS, 0x0004 },
	{ NAU8542_R68_REFERENCE, 0x0000 },
	{ NAU8542_R69_FEPGA1, 0x0000 },
	{ NAU8542_R6A_FEPGA2, 0x0000 },
	{ NAU8542_R6B_FEPGA3, 0x0101 },
	{ NAU8542_R6C_FEPGA4, 0x0101 },
	{ NAU8542_R6D_PWR, 0x0000 },
};

static bool nau8542_readable_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case NAU8542_R01_POWER_MANAGEMENT ... NAU8542_R0A_FLL_VCO_RSV:
	case NAU8542_R10_TDM_CTRL0 ... NAU8542_R14_PWRST_CTRL:
	case NAU8542_R20_ALC_CONTROL_1 ... NAU8542_R25_ALC_CONTROL_6:
	case NAU8542_R26_MIC_DIAG_CTRL_1:
	case NAU8542_R28_MIC_DIAG_CTRL_2:
	case NAU8542_R2D_ALC_STATUS_0 ... NAU8542_R3D_BQ_COEFF_B0H:
	case NAU8542_R40_DIGITAL_GAIN_CH1 ... NAU8542_R44_DIGITAL_MUX:
	case NAU8542_R48_P2P_CH1 ... NAU8542_R52_I2C_CTRL:
	case NAU8542_R58_I2C_DEVICE_ID:
	case NAU8542_R60_VMID_CTRL ... NAU8542_R61_MUTE:
	case NAU8542_R62_ANALOG_SPARE1:
	case NAU8542_R64_ANALOG_ADC1 ... NAU8542_R6E_ANALOG_STATUS:
		return true;
	default:
		return false;
	}
}

static bool nau8542_writeable_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case NAU8542_R01_POWER_MANAGEMENT ... NAU8542_R0A_FLL_VCO_RSV:
	case NAU8542_R10_TDM_CTRL0 ... NAU8542_R14_PWRST_CTRL:
	case NAU8542_R20_ALC_CONTROL_1 ... NAU8542_R25_ALC_CONTROL_6:
	case NAU8542_R26_MIC_DIAG_CTRL_1:
	case NAU8542_R28_MIC_DIAG_CTRL_2:
	case NAU8542_R2D_ALC_STATUS_0 ... NAU8542_R3D_BQ_COEFF_B0H:
	case NAU8542_R40_DIGITAL_GAIN_CH1 ... NAU8542_R44_DIGITAL_MUX:
	case NAU8542_R50_GPIO_CTRL ... NAU8542_R52_I2C_CTRL:
	case NAU8542_R5A_SW_RST ... NAU8542_R5B_HW_RST:
	case NAU8542_R60_VMID_CTRL ... NAU8542_R61_MUTE:
	case NAU8542_R64_ANALOG_ADC1 ... NAU8542_R6D_PWR:
		return true;
	default:
		return false;
	}
}

static bool nau8542_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case NAU8542_R26_MIC_DIAG_CTRL_1 ... NAU8542_R2F_ALC_STATUS_2:
	case NAU8542_R48_P2P_CH1 ... NAU8542_R4F_PEAK_CH4:
	case NAU8542_R58_I2C_DEVICE_ID:
	case NAU8542_R5A_SW_RST ... NAU8542_R5B_HW_RST:
		return true;
	default:
		return false;
	}
}

static const DECLARE_TLV_DB_MINMAX(adc_vol_tlv, -12800, 3600);
static const DECLARE_TLV_DB_MINMAX(fepga_gain_tlv, -100, 3600);

static const struct snd_kcontrol_new nau8542_snd_controls[] = {
	SOC_SINGLE_TLV("Mic1 Volume", NAU8542_R40_DIGITAL_GAIN_CH1,
		0, 0x520, 0, adc_vol_tlv),
	SOC_SINGLE_TLV("Mic2 Volume", NAU8542_R41_DIGITAL_GAIN_CH2,
		0, 0x520, 0, adc_vol_tlv),
	SOC_SINGLE_TLV("Mic3 Volume", NAU8542_R42_DIGITAL_GAIN_CH3,
		0, 0x520, 0, adc_vol_tlv),
	SOC_SINGLE_TLV("Mic4 Volume", NAU8542_R43_DIGITAL_GAIN_CH4,
		0, 0x520, 0, adc_vol_tlv),

	SOC_SINGLE_TLV("Frontend PGA1 Volume", NAU8542_R6B_FEPGA3,
		0, 0x25, 0, fepga_gain_tlv),
	SOC_SINGLE_TLV("Frontend PGA2 Volume", NAU8542_R6B_FEPGA3,
		8, 0x25, 0, fepga_gain_tlv),
	SOC_SINGLE_TLV("Frontend PGA3 Volume", NAU8542_R6C_FEPGA4,
		0, 0x25, 0, fepga_gain_tlv),
	SOC_SINGLE_TLV("Frontend PGA4 Volume", NAU8542_R6C_FEPGA4,
		8, 0x25, 0, fepga_gain_tlv),
};

static const struct snd_kcontrol_new nau8542_dmic_mode_switch =
	SOC_DAPM_SINGLE("Switch", NAU8542_R01_POWER_MANAGEMENT,
		NAU8542_DMIC_EN_SFT, 1, 0);

static const char * const adc_channel[] = {
	"ADC channel 1", "ADC channel 2", "ADC channel 3", "ADC channel 4"
};

static SOC_ENUM_SINGLE_DECL(
	digital_ch4_enum, NAU8542_R44_DIGITAL_MUX, 6, adc_channel);

static const struct snd_kcontrol_new digital_ch4_mux =
	SOC_DAPM_ENUM("Digital CH4 Select", digital_ch4_enum);

static SOC_ENUM_SINGLE_DECL(
	digital_ch3_enum, NAU8542_R44_DIGITAL_MUX, 4, adc_channel);

static const struct snd_kcontrol_new digital_ch3_mux =
	SOC_DAPM_ENUM("Digital CH3 Select", digital_ch3_enum);

static SOC_ENUM_SINGLE_DECL(
	digital_ch2_enum, NAU8542_R44_DIGITAL_MUX, 2, adc_channel);

static const struct snd_kcontrol_new digital_ch2_mux =
	SOC_DAPM_ENUM("Digital CH2 Select", digital_ch2_enum);

static SOC_ENUM_SINGLE_DECL(
	digital_ch1_enum, NAU8542_R44_DIGITAL_MUX, 0, adc_channel);

static const struct snd_kcontrol_new digital_ch1_mux =
	SOC_DAPM_ENUM("Digital CH1 Select", digital_ch1_enum);

static int nau8542_fepga_event(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *k, int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);
	struct nau8542 *nau8542 = snd_soc_component_get_drvdata(component);

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		regmap_update_bits(nau8542->regmap, NAU8542_R6A_FEPGA2,
			NAU8542_ACDC_CTL_MASK, NAU8542_ACDC_CTL_MIC1P_VREF |
			NAU8542_ACDC_CTL_MIC1N_VREF | NAU8542_ACDC_CTL_MIC2P_VREF |
			NAU8542_ACDC_CTL_MIC2N_VREF | NAU8542_ACDC_CTL_MIC3P_VREF |
			NAU8542_ACDC_CTL_MIC3N_VREF | NAU8542_ACDC_CTL_MIC4P_VREF |
			NAU8542_ACDC_CTL_MIC4N_VREF);
		break;
	default:
		break;
	}
	return 0;
}

static int nau8542_precharge_event(struct snd_soc_dapm_widget *w,
				   struct snd_kcontrol *k, int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);
	struct nau8542 *nau8542 = snd_soc_component_get_drvdata(component);

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		regmap_update_bits(nau8542->regmap, NAU8542_R68_REFERENCE,
				   NAU8542_DISCHRG_EN, NAU8542_DISCHRG_EN);
		msleep(40);
		regmap_update_bits(nau8542->regmap, NAU8542_R68_REFERENCE,
			NAU8542_DISCHRG_EN, 0);
		regmap_update_bits(nau8542->regmap, NAU8542_R6A_FEPGA2,
			NAU8542_ACDC_CTL_MASK, 0);
		break;
	default:
		break;
	}
	return 0;
}

static int adc_power_control(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *k, int  event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);
	struct nau8542 *nau8542 = snd_soc_component_get_drvdata(component);

	if (SND_SOC_DAPM_EVENT_ON(event)) {
		msleep(160);
		/* DO12 and DO34 pad output enable */
		regmap_update_bits(nau8542->regmap, NAU8542_R01_POWER_MANAGEMENT,
			NAU8542_ADC_ALL_EN, NAU8542_ADC_ALL_EN);
		regmap_update_bits(nau8542->regmap, NAU8542_R11_TDM_CTRL1,
			NAU8542_TDM_DO12_TRI, 0);
		regmap_update_bits(nau8542->regmap, NAU8542_R12_TDM_CTRL2,
			NAU8542_TDM_DO34_TRI, 0);
	} else if (SND_SOC_DAPM_EVENT_OFF(event)) {
		regmap_update_bits(nau8542->regmap, NAU8542_R11_TDM_CTRL1,
			NAU8542_TDM_DO12_TRI, NAU8542_TDM_DO12_TRI);
		regmap_update_bits(nau8542->regmap, NAU8542_R12_TDM_CTRL2,
			NAU8542_TDM_DO34_TRI, NAU8542_TDM_DO34_TRI);
		regmap_update_bits(nau8542->regmap, NAU8542_R01_POWER_MANAGEMENT,
			NAU8542_ADC_ALL_EN, 0);
	}

	return 0;
}

static int aiftx_power_control(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *k, int  event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);
	struct nau8542 *nau8542 = snd_soc_component_get_drvdata(component);

	if (SND_SOC_DAPM_EVENT_OFF(event)) {
		regmap_write(nau8542->regmap, NAU8542_R5A_SW_RST, 0x0001);
		regmap_write(nau8542->regmap, NAU8542_R5A_SW_RST, 0x0000);
	}
	return 0;
}

static const struct snd_soc_dapm_widget nau8542_dapm_widgets[] = {
	SND_SOC_DAPM_SUPPLY("MICBIAS2", NAU8542_R67_MIC_BIAS, 11, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("MICBIAS1", NAU8542_R67_MIC_BIAS, 10, 0, NULL, 0),

	SND_SOC_DAPM_INPUT("MIC1"),
	SND_SOC_DAPM_INPUT("MIC2"),
	SND_SOC_DAPM_INPUT("MIC3"),
	SND_SOC_DAPM_INPUT("MIC4"),
	SND_SOC_DAPM_INPUT("DMIC2"),
	SND_SOC_DAPM_INPUT("DMIC3"),

	SND_SOC_DAPM_PGA_S("Frontend PGA1", 0, NAU8542_R6D_PWR, 12, 0,
		nau8542_fepga_event, SND_SOC_DAPM_POST_PMU),
	SND_SOC_DAPM_PGA_S("Frontend PGA2", 0, NAU8542_R6D_PWR, 13, 0,
		nau8542_fepga_event, SND_SOC_DAPM_POST_PMU),
	SND_SOC_DAPM_PGA_S("Frontend PGA3", 0, NAU8542_R6D_PWR, 14, 0,
		nau8542_fepga_event, SND_SOC_DAPM_POST_PMU),
	SND_SOC_DAPM_PGA_S("Frontend PGA4", 0, NAU8542_R6D_PWR, 15, 0,
		nau8542_fepga_event, SND_SOC_DAPM_POST_PMU),

	SND_SOC_DAPM_PGA_S("Precharge", 1, SND_SOC_NOPM, 0, 0,
		nau8542_precharge_event, SND_SOC_DAPM_POST_PMU),

	SND_SOC_DAPM_PGA_S("ADC CH1", 2, NAU8542_R66_ANALOG_PWR, 0, 0,
		adc_power_control, SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_PGA_S("ADC CH2", 2, NAU8542_R66_ANALOG_PWR, 1, 0,
		adc_power_control, SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_PGA_S("ADC CH3", 2, NAU8542_R66_ANALOG_PWR, 2, 0,
		adc_power_control, SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_PGA_S("ADC CH4", 2, NAU8542_R66_ANALOG_PWR, 3, 0,
		adc_power_control, SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),

	SND_SOC_DAPM_MUX("Digital CH4 Mux",
		SND_SOC_NOPM, 0, 0, &digital_ch4_mux),
	SND_SOC_DAPM_MUX("Digital CH3 Mux",
		SND_SOC_NOPM, 0, 0, &digital_ch3_mux),
	SND_SOC_DAPM_MUX("Digital CH2 Mux",
		SND_SOC_NOPM, 0, 0, &digital_ch2_mux),
	SND_SOC_DAPM_MUX("Digital CH1 Mux",
		SND_SOC_NOPM, 0, 0, &digital_ch1_mux),

	SND_SOC_DAPM_SWITCH("DMIC Enable", SND_SOC_NOPM,
		0, 0, &nau8542_dmic_mode_switch),
	SND_SOC_DAPM_AIF_OUT_E("AIFTX", "Capture", 0, SND_SOC_NOPM, 0, 0,
		aiftx_power_control, SND_SOC_DAPM_POST_PMD),
};

static const struct snd_soc_dapm_route nau8542_dapm_routes[] = {
	{ "Frontend PGA1", NULL, "MIC1" },
	{ "Frontend PGA2", NULL, "MIC2" },
	{ "Frontend PGA3", NULL, "MIC3" },
	{ "Frontend PGA4", NULL, "MIC4" },

	{ "Precharge", NULL, "Frontend PGA1" },
	{ "Precharge", NULL, "Frontend PGA2" },
	{ "Precharge", NULL, "Frontend PGA3" },
	{ "Precharge", NULL, "Frontend PGA4" },

	{ "ADC CH1", NULL, "Precharge" },
	{ "ADC CH2", NULL, "Precharge" },
	{ "ADC CH3", NULL, "Precharge" },
	{ "ADC CH4", NULL, "Precharge" },

	{ "DMIC Enable", "Switch", "DMIC2" },
	{ "DMIC Enable", "Switch", "DMIC3" },

	{ "ADC CH2", NULL, "DMIC Enable" },
	{ "ADC CH3", NULL, "DMIC Enable" },

	{ "ADC CH1", NULL, "MICBIAS1" },
	{ "ADC CH2", NULL, "MICBIAS1" },
	{ "ADC CH3", NULL, "MICBIAS2" },
	{ "ADC CH4", NULL, "MICBIAS2" },

	{ "Digital CH1 Mux", "ADC channel 1", "ADC CH1" },
	{ "Digital CH1 Mux", "ADC channel 2", "ADC CH2" },
	{ "Digital CH1 Mux", "ADC channel 3", "ADC CH3" },
	{ "Digital CH1 Mux", "ADC channel 4", "ADC CH4" },

	{ "Digital CH2 Mux", "ADC channel 1", "ADC CH1" },
	{ "Digital CH2 Mux", "ADC channel 2", "ADC CH2" },
	{ "Digital CH2 Mux", "ADC channel 3", "ADC CH3" },
	{ "Digital CH2 Mux", "ADC channel 4", "ADC CH4" },

	{ "Digital CH3 Mux", "ADC channel 1", "ADC CH1" },
	{ "Digital CH3 Mux", "ADC channel 2", "ADC CH2" },
	{ "Digital CH3 Mux", "ADC channel 3", "ADC CH3" },
	 {"Digital CH3 Mux", "ADC channel 4", "ADC CH4" },

	{ "Digital CH4 Mux", "ADC channel 1", "ADC CH1" },
	{ "Digital CH4 Mux", "ADC channel 2", "ADC CH2" },
	{ "Digital CH4 Mux", "ADC channel 3", "ADC CH3" },
	{ "Digital CH4 Mux", "ADC channel 4", "ADC CH4" },

	{ "AIFTX", NULL, "Digital CH1 Mux" },
	{ "AIFTX", NULL, "Digital CH2 Mux" },
	{ "AIFTX", NULL, "Digital CH3 Mux" },
	{ "AIFTX", NULL, "Digital CH4 Mux" },
};

static const struct nau8542_osr_attr *
nau8542_get_osr(struct nau8542 *nau8542)
{
	unsigned int osr;

	regmap_read(nau8542->regmap, NAU8542_R03_CLOCK_SRC, &osr);
	osr = (osr & NAU8542_ADC_OSR_MASK) >> NAU8542_ADC_OSR_SFT;

	if (osr >= ARRAY_SIZE(osr_adc_sel))
		return NULL;
	return &osr_adc_sel[osr];
}

static int nau8542_dai_startup(struct snd_pcm_substream *substream,
	struct snd_soc_dai *dai)
{
	struct snd_soc_component *component = dai->component;
	struct nau8542 *nau8542 = snd_soc_component_get_drvdata(component);
	const struct nau8542_osr_attr *osr;

	osr = nau8542_get_osr(nau8542);

	if (!osr || !osr->osr)
		return -EINVAL;

	return snd_pcm_hw_constraint_minmax(substream->runtime,SNDRV_PCM_HW_PARAM_RATE, 0,
		CLK_ADC_MAX / osr->osr);
}

/**
 * nau8542_set_sysclk_ratio - Calculate and set the SYSCLK to Fs ratio.
 * @nau8542: codec private data.
 * @fs: audio sample rate.
 *
 * Returns 0 for success or negative error code.
 */
static int nau8542_set_sysclk_ratio(struct nau8542 *nau8542, unsigned int fs)
{
	unsigned int ratio, ratio_val;

	if (!nau8542->sysclk) {
		dev_err(nau8542->dev, "sysclk is not configured\n");
		return -EINVAL;
	}

	ratio = nau8542->sysclk / fs;

	switch (ratio) {
	case 256:
		ratio_val = NAU8542_SYSCLK_256FS;
		break;
	case 128:
		ratio_val = NAU8542_SYSCLK_128FS;
		break;
	case 384:
		ratio_val = NAU8542_SYSCLK_384FS;
		break;
	default:
		dev_err(nau8542->dev, "Unsupported sysclk ratio: %d\n", ratio);
		return -EINVAL;
	}

	regmap_update_bits(nau8542->regmap, NAU8542_R03_CLOCK_SRC,
		NAU8542_SYSCLK_RATIO_MASK, ratio_val);

	return 0;
}

static int nau8542_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	struct snd_soc_component *component = dai->component;
	struct nau8542 *nau8542 = snd_soc_component_get_drvdata(component);
	unsigned int val_len = 0;
	unsigned int fs = params_rate(params);
	const struct nau8542_osr_attr *osr;
	int ret;

	ret = nau8542_set_sysclk_ratio(nau8542, fs);
	if (ret)
		return ret;

	/* CLK_ADC = OSR * FS
	 * ADC clock frequency is defined as Over Sampling Rate (OSR)
	 * multiplied by the audio sample rate (Fs). Note that the OSR and Fs
	 * values must be selected such that the maximum frequency is less
	 * than 6.144 MHz.
	 */
	osr = nau8542_get_osr(nau8542);
	if (!osr || !osr->osr)
		return -EINVAL;
	if (params_rate(params) * osr->osr > CLK_ADC_MAX)
		return -EINVAL;

	regmap_update_bits(nau8542->regmap, NAU8542_R03_CLOCK_SRC,
		NAU8542_CLK_DMIC_SRC_MASK,
		osr->clk_src << NAU8542_CLK_DMIC_SRC_SFT);
	switch (params_width(params)) {
	case 16:
		val_len |= NAU8542_TDM_DL_16;
		break;
	case 20:
		val_len |= NAU8542_TDM_DL_20;
		break;
	case 24:
		val_len |= NAU8542_TDM_DL_24;
		break;
	case 32:
		val_len |= NAU8542_TDM_DL_32;
		break;
	default:
		return -EINVAL;
	}

	regmap_update_bits(nau8542->regmap, NAU8542_R10_TDM_CTRL0,
		NAU8542_TDM_DL_MASK, val_len);

	return 0;
}

static int nau8542_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct snd_soc_component *component = dai->component;
	struct nau8542 *nau8542 = snd_soc_component_get_drvdata(component);
	unsigned int ctrl1_val = 0;

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBP_CFP:
		ctrl1_val |= NAU8542_TDM_MS_MASTER;
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
		ctrl1_val |= NAU8542_TDM_BP_INV;
		break;
	default:
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		ctrl1_val |= NAU8542_TDM_DF_I2S;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		ctrl1_val |= NAU8542_TDM_DF_LEFT;
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		ctrl1_val |= NAU8542_TDM_DF_RIGTH;
		break;
	case SND_SOC_DAIFMT_DSP_A:
		ctrl1_val |= NAU8542_TDM_DF_PCM_AB;
		break;
	case SND_SOC_DAIFMT_DSP_B:
		ctrl1_val |= NAU8542_TDM_DF_PCM_AB;
		ctrl1_val |= NAU8542_TDM_PCMB_EN;
		break;
	default:
		return -EINVAL;
	}

	regmap_update_bits(nau8542->regmap, NAU8542_R10_TDM_CTRL0,
		NAU8542_TDM_DL_MASK | NAU8542_TDM_DF_MASK | NAU8542_TDM_MS_MASK |
		NAU8542_TDM_BP_INV | NAU8542_TDM_PCMB_EN, ctrl1_val);

	return 0;
}

/**
 * nau8542_set_tdm_slot - configure DAI TX TDM.
 * @dai: DAI
 * @tx_mask: bitmask representing active TX slots. Ex.
 *                 0xf for normal 4 channel TDM.
 *                 0xf0 for shifted 4 channel TDM
 * @rx_mask: no used.
 * @slots: Number of slots in use.
 * @slot_width: Width in bits for each slot.
 *
 * Configures a DAI for TDM operation. Only support 4 slots TDM.
 */
static int nau8542_set_tdm_slot(struct snd_soc_dai *dai,
	unsigned int tx_mask, unsigned int rx_mask, int slots, int slot_width)
{

	struct snd_soc_component *component = dai->component;
	struct nau8542 *nau8542 = snd_soc_component_get_drvdata(component);
	unsigned int ctrl2_val = 0, ctrl4_val = 0;

	if (slots > 4 || ((tx_mask & 0xf0) && (tx_mask & 0xf)))
		return -EINVAL;

	if (tx_mask & 0xf0) {
		ctrl2_val = 4 * slot_width;
		ctrl4_val |= (tx_mask >> 4);
	} else {
		ctrl4_val |= tx_mask;
	}

	regmap_update_bits(nau8542->regmap, NAU8542_R10_TDM_CTRL0,
		NAU8542_TDM_MODE_MASK | NAU8542_TDM_PCM_TS_EN,
		NAU8542_TDM_MODE | NAU8542_TDM_PCM_TS_EN);
	regmap_update_bits(nau8542->regmap, NAU8542_R14_PWRST_CTRL,
		NAU8542_TDM_TX_EN_MASK, ctrl4_val);
	regmap_update_bits(nau8542->regmap, NAU8542_R11_TDM_CTRL1,
		NAU8542_TDM_DO12_TRI, NAU8542_TDM_DO12_TRI);
	regmap_update_bits(nau8542->regmap, NAU8542_R12_TDM_CTRL2,
		NAU8542_TDM_DO34_TRI | NAU8542_TDM_TSLOT_L_MASK,
		NAU8542_TDM_DO34_TRI | ctrl2_val);

	return 0;
}

static const struct snd_soc_dai_ops nau8542_dai_ops = {
	.startup = nau8542_dai_startup,
	.hw_params = nau8542_hw_params,
	.set_fmt = nau8542_set_fmt,
	.set_tdm_slot = nau8542_set_tdm_slot,
};

#define NAU8542_RATES SNDRV_PCM_RATE_8000_48000
#define NAU8542_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE \
	 | SNDRV_PCM_FMTBIT_S24_3LE | SNDRV_PCM_FMTBIT_S32_LE)

static struct snd_soc_dai_driver nau8542_dai = {
	.name = NAU8542_CODEC_DAI,
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = 4,
		.rates = NAU8542_RATES,
		.formats = NAU8542_FORMATS,
	},
	.ops = &nau8542_dai_ops,
};

/**
 * nau8542_calc_fll_param - Calculate FLL parameters.
 * @fll_in: external clock provided to codec.
 * @fs: sampling rate.
 * @fll_param: Pointer to structure of FLL parameters.
 *
 * Calculate FLL parameters to configure codec.
 *
 * Returns 0 for success or negative error code.
 */
static int nau8542_calc_fll_param(unsigned int fll_in,
	unsigned int fs, struct nau8542_fll *fll_param)
{
	u64 fvco, fvco_max;
	unsigned int fref, i, fvco_sel;

	/* Ensure the reference clock frequency (FREF) is <= 13.5MHz by dividing
	 * freq_in by 1, 2, 4, or 8 using FLL pre-scalar.
	 * FREF = freq_in / NAU8542_FLL_REF_DIV_MASK
	 */
	for (i = 0; i < ARRAY_SIZE(fll_pre_scalar); i++) {
		fref = fll_in / fll_pre_scalar[i].param;
		if (fref <= NAU_FREF_MAX)
			break;
	}
	if (i == ARRAY_SIZE(fll_pre_scalar))
		return -EINVAL;
	fll_param->clk_ref_div = fll_pre_scalar[i].val;

	/* Choose the FLL ratio based on FREF */
	for (i = 0; i < ARRAY_SIZE(fll_ratio); i++) {
		if (fref >= fll_ratio[i].param)
			break;
	}
	if (i == ARRAY_SIZE(fll_ratio))
		return -EINVAL;
	fll_param->ratio = fll_ratio[i].val;

	/* Calculate the frequency of DCO (FDCO) given freq_out = 256 * Fs.
	 * FDCO must be within the 90MHz - 100MHz or the FFL cannot be
	 * guaranteed across the full range of operation.
	 * FDCO = freq_out * 2 * mclk_src_scaling
	 */
	fvco_max = 0;
	fvco_sel = ARRAY_SIZE(mclk_src_scaling);
	for (i = 0; i < ARRAY_SIZE(mclk_src_scaling); i++) {
		fvco = 256ULL * fs * 2 * mclk_src_scaling[i].param;
		if (fvco > NAU_FVCO_MIN && fvco < NAU_FVCO_MAX &&
			fvco_max < fvco) {
			fvco_max = fvco;
			fvco_sel = i;
		}
	}
	if (ARRAY_SIZE(mclk_src_scaling) == fvco_sel)
		return -EINVAL;

	fll_param->mclk_src = mclk_src_scaling[fvco_sel].val;

	/* Calculate the FLL 10-bit integer input and the FLL 24-bit fractional
	 * input based on FDCO, FREF and FLL ratio.
	 */
	fvco = div_u64(fvco_max << 24, fref * fll_param->ratio);
	fll_param->fll_int = (fvco >> 24) & 0x3FF;
	fll_param->fll_frac = fvco & 0xFFFFFF;

	return 0;
}

static void nau8542_fll_apply(struct regmap *regmap,
	struct nau8542_fll *fll_param)
{
	regmap_update_bits(regmap, NAU8542_R03_CLOCK_SRC,
		NAU8542_CLK_SRC_MASK | NAU8542_CLK_MCLK_SRC_MASK,
		NAU8542_CLK_SRC_MCLK | fll_param->mclk_src);
	regmap_update_bits(regmap, NAU8542_R04_FLL1,
		NAU8542_FLL_RATIO_MASK, fll_param->ratio);

	/* FLL 24-bit fractional input: MSB [23:16] in FLL3, LSB [15:0] in FLL2 */
	regmap_write(regmap, NAU8542_R05_FLL2, (fll_param->fll_frac & 0xFFFF));
	regmap_update_bits(regmap, NAU8542_R06_FLL3, NAU8542_FLL_FRAC_H_MASK,
		(fll_param->fll_frac >> 16) & NAU8542_FLL_FRAC_H_MASK);
	/* FLL 10-bit integer input */
	regmap_update_bits(regmap, NAU8542_R08_FLL5,
		NAU8542_FLL_INTEGER_MASK, fll_param->fll_int);
	/* FLL pre-scaler */
	regmap_update_bits(regmap, NAU8542_R07_FLL4,
		NAU8542_FLL_REF_DIV_MASK,
		fll_param->clk_ref_div << NAU8542_FLL_REF_DIV_SFT);
	regmap_update_bits(regmap,
		NAU8542_R09_FLL6, NAU8542_DCO_EN, 0);
	if (fll_param->fll_frac) {
		regmap_update_bits(regmap, NAU8542_R08_FLL5,
			NAU8542_FLL_PDB_DAC_EN | NAU8542_FLL_LOOP_FTR_EN,
			NAU8542_FLL_PDB_DAC_EN | NAU8542_FLL_LOOP_FTR_EN);
		regmap_update_bits(regmap, NAU8542_R09_FLL6,
			NAU8542_SDM_EN | NAU8542_CUTOFF500,
			NAU8542_SDM_EN | NAU8542_CUTOFF500);
	} else {
		regmap_update_bits(regmap, NAU8542_R08_FLL5,
			NAU8542_FLL_PDB_DAC_EN | NAU8542_FLL_LOOP_FTR_EN, 0);
		regmap_update_bits(regmap, NAU8542_R09_FLL6,
			NAU8542_SDM_EN | NAU8542_CUTOFF500, 0);
	}
}

/* freq_out must be 256*Fs in order to achieve the best performance */
static int nau8542_set_pll(struct snd_soc_component *component, int pll_id, int source,
		unsigned int freq_in, unsigned int freq_out)
{
	struct nau8542 *nau8542 = snd_soc_component_get_drvdata(component);
	struct nau8542_fll fll_param;
	int ret, fs;

	switch (pll_id) {
	case NAU8542_CLK_FLL_MCLK:
		printk("====>NAU8542_CLK_FLL_MCLK freq_in:%d ", freq_in);

		regmap_update_bits(nau8542->regmap, NAU8542_R06_FLL3,
			NAU8542_FLL_CLK_SRC_MASK | NAU8542_GAIN_ERR_MASK,
			NAU8542_FLL_CLK_SRC_MCLK | 0);
		break;

	case NAU8542_CLK_FLL_BLK:
		printk("====>NAU8542_CLK_FLL_BLK freq_in:%d", freq_in);
		regmap_update_bits(nau8542->regmap, NAU8542_R06_FLL3,
			NAU8542_FLL_CLK_SRC_MASK | NAU8542_GAIN_ERR_MASK,
			NAU8542_FLL_CLK_SRC_BLK |
			(0xf << NAU8542_GAIN_ERR_SFT));
		regmap_update_bits(nau8542->regmap, NAU8542_R0A_FLL_VCO_RSV,
			NAU8542_BCLKDET_CTRLFLL_MASK, NAU8542_BCLKDET_CTRLFLL_EN);
		break;

	default:
		dev_err(nau8542->dev, "Invalid clock id (%d)\n", pll_id);
		return -EINVAL;
	}
	dev_dbg(nau8542->dev, "Sysclk is %dHz and clock id is %d\n",
		freq_out, pll_id);

	fs = freq_out / 256;
	printk("===> fs:%d", fs);
	ret = nau8542_calc_fll_param(freq_in, fs, &fll_param);
	if (ret < 0) {
		dev_err(nau8542->dev, "Unsupported input clock %d\n", freq_in);
		return ret;
	}
	dev_dbg(nau8542->dev, "mclk_src=%x ratio=%x fll_frac=%x fll_int=%x clk_ref_div=%x\n",
		fll_param.mclk_src, fll_param.ratio, fll_param.fll_frac,
		fll_param.fll_int, fll_param.clk_ref_div);

	nau8542_fll_apply(nau8542->regmap, &fll_param);
	mdelay(2);
	regmap_update_bits(nau8542->regmap, NAU8542_R03_CLOCK_SRC,
		NAU8542_CLK_SRC_MASK, NAU8540_CLK_SRC_VCO);
	regmap_update_bits(nau8542->regmap, NAU8542_R0A_FLL_VCO_RSV,
		NAU8542_MCLKDET_CTRLFLL_MASK, NAU8542_MCLKDET_CTRLFLL_DIS);
	return 0;
}

static int nau8542_set_sysclk(struct snd_soc_component *component,
	int clk_id, int source, unsigned int freq, int dir)
{
	struct nau8542 *nau8542 = snd_soc_component_get_drvdata(component);
	nau8542->sysclk = freq;

	switch (clk_id) {
	case NAU8542_CLK_MCLK:
		regmap_update_bits(nau8542->regmap, NAU8542_R03_CLOCK_SRC,
			NAU8542_CLK_SRC_MASK, NAU8542_CLK_SRC_MCLK);
		regmap_update_bits(nau8542->regmap, NAU8542_R09_FLL6,
			NAU8542_DCO_EN, 0);
		break;

	case NAU8542_CLK_INTERNAL:
		regmap_update_bits(nau8542->regmap, NAU8542_R09_FLL6,
			NAU8542_DCO_EN, NAU8542_DCO_EN);
		regmap_update_bits(nau8542->regmap, NAU8542_R03_CLOCK_SRC,
			NAU8542_CLK_SRC_MASK, NAU8540_CLK_SRC_VCO);
		break;

	default:
		dev_err(nau8542->dev, "Invalid clock id (%d)\n", clk_id);
		return -EINVAL;
	}

	dev_dbg(nau8542->dev, "Sysclk is %dHz and clock id is %d\n",
		freq, clk_id);

	return 0;
}

static void nau8542_reset_chip(struct regmap *regmap)
{
	regmap_write(regmap, NAU8542_R5B_HW_RST, 0x00);
}

static void nau8542_init_regs(struct nau8542 *nau8542)
{
	struct regmap *regmap = nau8542->regmap;

	/* Enable Bias/VMID/VMID Tieoff */
	regmap_update_bits(regmap, NAU8542_R60_VMID_CTRL,
		NAU8542_VMID_EN | NAU8542_VMID_SEL_MASK,
		NAU8542_VMID_EN | (0x2 << NAU8542_VMID_SEL_SFT));
	regmap_update_bits(regmap, NAU8542_R68_REFERENCE,
		NAU8542_PRECHARGE_DIS | NAU8542_GLOBAL_BIAS_EN,
		NAU8542_PRECHARGE_DIS | NAU8542_GLOBAL_BIAS_EN);
	mdelay(2);
	regmap_update_bits(regmap, NAU8542_R67_MIC_BIAS,
		NAU8542_PU_PRE, NAU8542_PU_PRE);
	regmap_update_bits(regmap, NAU8542_R02_CLOCK_CTRL,
		NAU8542_CLK_I2S_EN, NAU8542_CLK_I2S_EN);
	/* ADC OSR selection, CLK_ADC = Fs * OSR;
	 * Channel time alignment enable.
	 */
	regmap_update_bits(regmap, NAU8542_R03_CLOCK_SRC,
		NAU8542_ADC_OSR_MASK, NAU8542_ADC_OSR_64);
	/* PGA input mode selection */
	regmap_update_bits(regmap, NAU8542_R69_FEPGA1,
		NAU8542_FEPGA1_MODCH2_SHT | NAU8542_FEPGA1_MODCH1_SHT,
		NAU8542_FEPGA1_MODCH2_SHT | NAU8542_FEPGA1_MODCH1_SHT);
	regmap_update_bits(regmap, NAU8542_R6A_FEPGA2,
		NAU8542_FEPGA2_MODCH4_SHT | NAU8542_FEPGA2_MODCH3_SHT,
		NAU8542_FEPGA2_MODCH4_SHT | NAU8542_FEPGA2_MODCH3_SHT);
	/* DO12 and DO34 pad output disable */
	regmap_update_bits(regmap, NAU8542_R11_TDM_CTRL1,
		NAU8542_TDM_DO12_TRI, NAU8542_TDM_DO12_TRI);
	regmap_update_bits(regmap, NAU8542_R12_TDM_CTRL2,
		NAU8542_TDM_DO34_TRI, NAU8542_TDM_DO34_TRI);

}

static int __maybe_unused nau8542_suspend(struct snd_soc_component *component)
{
	struct nau8542 *nau8542 = snd_soc_component_get_drvdata(component);

	regcache_cache_only(nau8542->regmap, true);
	regcache_mark_dirty(nau8542->regmap);

	return 0;
}

static int __maybe_unused nau8542_resume(struct snd_soc_component *component)
{
	struct nau8542 *nau8542 = snd_soc_component_get_drvdata(component);

	regcache_cache_only(nau8542->regmap, false);
	regcache_sync(nau8542->regmap);

	return 0;
}

static const struct snd_soc_component_driver nau8542_component_driver = {
	.set_sysclk		= nau8542_set_sysclk,
	.set_pll		= nau8542_set_pll,
	.suspend		= nau8542_suspend,
	.resume			= nau8542_resume,
	.controls		= nau8542_snd_controls,
	.num_controls		= ARRAY_SIZE(nau8542_snd_controls),
	.dapm_widgets		= nau8542_dapm_widgets,
	.num_dapm_widgets	= ARRAY_SIZE(nau8542_dapm_widgets),
	.dapm_routes		= nau8542_dapm_routes,
	.num_dapm_routes	= ARRAY_SIZE(nau8542_dapm_routes),
	.suspend_bias_off	= 1,
	.idle_bias_on		= 1,
	.use_pmdown_time	= 1,
	.endianness		= 1,
};

static const struct regmap_config nau8542_regmap_config = {
	.val_bits = 16,
	.reg_bits = 16,

	.max_register = NAU8542_REG_MAX,
	.readable_reg = nau8542_readable_reg,
	.writeable_reg = nau8542_writeable_reg,
	.volatile_reg = nau8542_volatile_reg,

	.cache_type = REGCACHE_MAPLE,
	.reg_defaults = nau8542_reg_defaults,
	.num_reg_defaults = ARRAY_SIZE(nau8542_reg_defaults),
};

static int nau8542_i2c_probe(struct i2c_client *i2c)
{
	struct device *dev = &i2c->dev;
	struct nau8542 *nau8542;
	int ret, value;

	nau8542 = devm_kzalloc(dev, sizeof(*nau8542), GFP_KERNEL);
	if (!nau8542)
		return -ENOMEM;

	i2c_set_clientdata(i2c, nau8542);
	nau8542->regmap = devm_regmap_init_i2c(i2c, &nau8542_regmap_config);
	if (IS_ERR(nau8542->regmap))
		return PTR_ERR(nau8542->regmap);
	ret = regmap_read(nau8542->regmap, NAU8542_R58_I2C_DEVICE_ID, &value);
	if (ret < 0) {
		dev_err(dev, "Failed to read device id from the NAU85L42: %d\n",
			ret);
		return ret;
	}

	nau8542->dev = dev;
	nau8542_reset_chip(nau8542->regmap);
	nau8542_init_regs(nau8542);

	return devm_snd_soc_register_component(dev,
		&nau8542_component_driver, &nau8542_dai, 1);
}

static const struct i2c_device_id nau8542_i2c_ids[] = {
	{ .name = "nau8542" },
	{}
};
MODULE_DEVICE_TABLE(i2c, nau8542_i2c_ids);

static const struct of_device_id nau8542_of_ids[] = {
	{ .compatible = "nuvoton,nau8542" },
	{}
};
MODULE_DEVICE_TABLE(of, nau8542_of_ids);

static struct i2c_driver nau8542_i2c_driver = {
	.driver = {
		.name = "nau8542",
		.of_match_table = of_match_ptr(nau8542_of_ids),
	},
	.probe = nau8542_i2c_probe,
	.id_table = nau8542_i2c_ids,
};
module_i2c_driver(nau8542_i2c_driver);

MODULE_DESCRIPTION("ASoC NAU85L42 ADC driver");
MODULE_AUTHOR("John Hsu <KCHSU0@nuvoton.com>");
MODULE_AUTHOR("Neo Chang <ylchang2@nuvoton.com>");
MODULE_LICENSE("GPL v2");
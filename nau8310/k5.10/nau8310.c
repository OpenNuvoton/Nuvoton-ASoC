// SPDX-License-Identifier: GPL-2.0
//
// The NAU83G10 Boosted Mono Class-D Amplifier with DSP and I/V-sense driver.
//
// Copyright 2021 Nuvoton Technology Crop.
// Author: John Hsu <kchsu0@nuvoton.com>
//         David Lin <ctlin0@nuvoton.com>

#include <linux/acpi.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/initval.h>
#include <sound/jack.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/tlv.h>

#include "nau8310.h"
#include "nau8310-dsp.h"
static void nau8310_software_reset(struct regmap *regmap);
static int nau8310_set_sysclk(struct snd_soc_component *component, int clk_id,
			      int source, unsigned int freq, int dir);

/* Range of Master Clock MCLK (Hz) */
#define MASTER_CLK_MIN 2048000
#define MASTER_CLK_MAX 49152000
/* DSP Clock (Hz) */
#define DSP_CLK_MIN_MULTI_VALUE 2000
#define DSP_CLK_MAX 102400000
#define DSP_MCLK_HI_SPEED 12288000
#define WIDGET_NAME_MAX_SIZE 80

static const struct nau8310_src_attr dsp_src_mult[] = {
	/* param : power of 2 */
	{ 0, 0x3 },
	{ 1, 0x2 },
	{ 2, 0x1 },
	{ 3, 0x0 },
	{ 4, 0x4 },
};

/* scaling for MCLK source */
#define CLK_PROC_BYPASS (-1)

static const struct nau8310_src_attr mclk_n1_div[] = {
	{ 3, 0x2 },
	{ 1, 0x0 },
	{ 2, 0x1 },
};

static const struct nau8310_src_attr mclk_n2_div[] = {
	/* param : power of 2 */
	{ 0, 0x0 },
	{ 1, 0x1 },
	{ 2, 0x2 },
	{ 3, 0x3 },
};

static const struct nau8310_src_attr mclk_src_mult[] = {
	/* param : power of 2 */
	{ 0, 0x1 },
	{ 1, 0x2 },
	{ 2, 0x3 },
	{ 3, 0x4 },
	{ 4, 0x5 },
};

/* Sample Rate and MCLK_SRC selections */
static const struct nau8310_srate_attr target_srate_table[] = {
	/* { FS, range, max, { MCLK source }, adc_div} */
	{ 48000, 2, true, { 12288000, 19200000, 24000000 }, 0x1 },
	{ 16000, 1, false, { 4096000, 6400000, 8000000 }, 0x0 },
	{ 8000, 0, false, { 2048000, 3200000, 4000000 }, 0x0 },
	{ 44100, 2, true, { 11289600, 17640000, 22050000 }, 0x1 },
	{ 64000, 3, false, { 16384000, 25600000, 32000000 }, 0x1 },
	{ 96000, 3, true, { 24576000, 38400000, 48000000 }, (-1) },
	{ 12000, 0, true, { 3072000, 4800000, 6000000 }, 0x0 },
	{ 24000, 1, true, { 6144000, 9600000, 12000000 }, 0x2 },
	{ 32000, 2, false, { 8192000, 12800000, 16000000 }, 0x2 },
};

/* the maximum frequency of CLK_ADC and CLK_DAC */
#define CLK_DA_AD_MAX 6144000

/* over sampling rate */
static const struct nau8310_osr_attr osr_dac_sel[] = {
	{ 64, 2 },		/* OSR 64, SRC 1/4 */
	{ 256, 0 },		/* OSR 256, SRC 1 */
	{ 128, 1 },		/* OSR 128, SRC 1/2 */
	{ 0, 0 },
	{ 32, 3 },		/* OSR 32, SRC 1/8 */
};

static const struct nau8310_osr_attr osr_adc_sel[] = {
	{ 32, 2 },		/* OSR 32, SRC 1/8 */
	{ 64, 1 },		/* OSR 64, SRC 1/4 */
	{ 128, 0 },		/* OSR 128, SRC 1/2 */
};

/* For a typical 48kHz frame sync I2S signal on DSP case, the ADC sampling rate
 * must be 12kHz. So the specfied OSR to SRC divider is need to divide by 4.
 */
static const struct nau8310_osr_attr osr_adc_dsp_sel[] = {
	{ 0, 0 },		/* no such case */
	{ 64, 3 },		/* OSR 64, SRC 1/16 */
	{ 128, 2 },		/* OSR 128, SRC 1/8 */
};

static const struct reg_default nau8310_reg_defaults[] = {
	{ NAU8310_R02_I2C_ADDR, 0x0000 },
	{ NAU8310_R03_CLK_CTRL, 0x0000 },
	{ NAU8310_R04_ENA_CTRL, 0x0000 },
	{ NAU8310_R05_INTERRUPT_CTRL, 0x007f },
	{ NAU8310_R06_INT_CLR_STATUS, 0x0000 },
	{ NAU8310_R07_SAR_CTRL1, 0x00b4 },
	{ NAU8310_R08_GPIO124_CTRL, 0x0102 },
	{ NAU8310_R09_GPIOOUT, 0x0000 },
	{ NAU8310_R0A_IO_CTRL, 0x0000 },
	{ NAU8310_R0B_I2S_PCM_CTRL0, 0x0000 },
	{ NAU8310_R0C_TDM_CTRL, 0x0c00 },
	{ NAU8310_R0D_I2S_PCM_CTRL1, 0x000a },
	{ NAU8310_R0E_I2S_PCM_CTRL2, 0x8010 },
	{ NAU8310_R0F_LEFT_TIME_SLOT, 0x0000 },
	{ NAU8310_R10_RIGHT_TIME_SLOT, 0x0000 },
	{ NAU8310_R12_HPF_CTRL, 0x0010 },
	{ NAU8310_R13_MUTE_CTRL, 0x00cf },
	{ NAU8310_R14_ADC_VOL_CTRL, 0x0000 },
	{ NAU8310_R15_INT_COE, 0xa41f },
	{ NAU8310_R16_DEC_COE, 0xb7cd },
	{ NAU8310_R17_BOOST_CTRL1, 0x3f00 },
	{ NAU8310_R18_BOOST_CTRL2, 0x0000 },
	{ NAU8310_R19_DSP_CORE_CTRL1, 0x0000 },
	{ NAU8310_R1A_DSP_CORE_CTRL2, 0x0000 },
	{ NAU8310_R28_ADC_RATE, 0x0002 },
	{ NAU8310_R29_DAC_CTRL1, 0x0081 },
	{ NAU8310_R2A_DAC_CTRL2, 0x0000 },
	{ NAU8310_R2C_ALC_CTRL1, 0x2000 },
	{ NAU8310_R2D_ALC_CTRL2, 0x8400 },
	{ NAU8310_R2E_ALC_CTRL3, 0x0000 },
	{ NAU8310_R2F_ALC_CTRL4, 0x00bf },
	{ NAU8310_R30_TEMP_COMP_CTRL, 0x7df4 },
	{ NAU8310_R31_UVLO_CTRL0, 0x0000 },
	{ NAU8310_R32_UVLO_CTRL1, 0x0000 },
	{ NAU8310_R33_LPF_CTRL, 0x7380 },
	{ NAU8310_R40_CLK_DET_CTRL, 0xa801 },
	{ NAU8310_R55_MISC_CTRL, 0x0000 },
	{ NAU8310_R60_BIAS_ADJ, 0x0000 },
	{ NAU8310_R61_ANALOG_CONTROL_1, 0x0000 },
	{ NAU8310_R62_ANALOG_CONTROL_2, 0x0000 },
	{ NAU8310_R63_ANALOG_CONTROL_3, 0x0000 },
	{ NAU8310_R64_ANALOG_CONTROL_4, 0x0000 },
	{ NAU8310_R65_ANALOG_CONTROL_5, 0x0000 },
	{ NAU8310_R66_ANALOG_CONTROL_6, 0x0000 },
	{ NAU8310_R68_ANALOG_CONTROL_7, 0x0000 },
	{ NAU8310_R69_CLIP_CTRL, 0x0000 },
	{ NAU8310_R6B_ANALOG_CONTROL_8, 0x0000 },
	{ NAU8310_R6C_ANALOG_CONTROL_9, 0x0000 },
	{ NAU8310_R71_ANALOG_ADC_1, 0x0011 },
	{ NAU8310_R72_ANALOG_ADC_2, 0x0020 },
	{ NAU8310_R73_RDAC, 0x0008 },
	{ NAU8310_R76_BOOST, 0x0000 },
	{ NAU8310_R77_FEPGA, 0x0000 },
	{ NAU8310_R7F_POWER_UP_CONTROL, 0x0000 },
	{ NAU8310_R80_BIQ0_COE_1, 0x0800 },
	{ NAU8310_R81_BIQ0_COE_2, 0x0000 },
	{ NAU8310_R82_BIQ0_COE_3, 0x0000 },
	{ NAU8310_R83_BIQ0_COE_4, 0x0000 },
	{ NAU8310_R84_BIQ0_COE_5, 0x0000 },
	{ NAU8310_R85_BIQ0_COE_6, 0x0000 },
	{ NAU8310_R86_BIQ0_COE_7, 0x0000 },
	{ NAU8310_R87_BIQ0_COE_8, 0x0000 },
	{ NAU8310_R88_BIQ0_COE_9, 0x0000 },
	{ NAU8310_R89_BIQ0_COE_10, 0x0000 },
	{ NAU8310_R8A_BIQ1_COE_1, 0x0000 },
	{ NAU8310_R8B_BIQ1_COE_2, 0x0000 },
	{ NAU8310_R8C_BIQ1_COE_3, 0x0000 },
	{ NAU8310_R8D_BIQ1_COE_4, 0x0000 },
	{ NAU8310_R8E_BIQ1_COE_5, 0x0000 },
	{ NAU8310_R8F_BIQ1_COE_6, 0x0000 },
	{ NAU8310_R90_BIQ1_COE_7, 0x0000 },
	{ NAU8310_R91_BIQ1_COE_8, 0x0000 },
	{ NAU8310_R92_BIQ1_COE_9, 0x0000 },
	{ NAU8310_R93_BIQ1_COE_10, 0x0000 },
	{ NAU8310_R94_BIQ2_COE_1, 0x0000 },
	{ NAU8310_R95_BIQ2_COE_2, 0x0000 },
	{ NAU8310_R96_BIQ2_COE_3, 0x0000 },
	{ NAU8310_R97_BIQ2_COE_4, 0x0000 },
	{ NAU8310_R98_BIQ2_COE_5, 0x0000 },
	{ NAU8310_R99_BIQ2_COE_6, 0x0000 },
	{ NAU8310_R9A_BIQ2_COE_7, 0x0000 },
	{ NAU8310_R9B_BIQ2_COE_8, 0x0000 },
	{ NAU8310_R9C_BIQ2_COE_9, 0x0000 },
	{ NAU8310_R9D_BIQ2_COE_10, 0x0000 },
	{ NAU8310_RF000_DSP_COMM, 0x0000 },
};

static bool nau8310_readable_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case NAU8310_R03_CLK_CTRL ... NAU8310_R10_RIGHT_TIME_SLOT:
	case NAU8310_R12_HPF_CTRL ... NAU8310_R2A_DAC_CTRL2:
	case NAU8310_R2C_ALC_CTRL1 ... NAU8310_R33_LPF_CTRL:
	case NAU8310_R40_CLK_DET_CTRL:
	case NAU8310_R46_I2C_DEVICE_ID:
	case NAU8310_R49_SARDOUT_RAM_STATUS ... NAU8310_R4A_ANALOG_READ:
	case NAU8310_R55_MISC_CTRL:
	case NAU8310_R60_BIAS_ADJ ... NAU8310_R66_ANALOG_CONTROL_6:
	case NAU8310_R68_ANALOG_CONTROL_7 ... NAU8310_R69_CLIP_CTRL:
	case NAU8310_R6B_ANALOG_CONTROL_8 ... NAU8310_R6C_ANALOG_CONTROL_9:
	case NAU8310_R71_ANALOG_ADC_1 ... NAU8310_R73_RDAC:
	case NAU8310_R76_BOOST ... NAU8310_R77_FEPGA:
	case NAU8310_R7F_POWER_UP_CONTROL ... NAU8310_R9D_BIQ2_COE_10:
	case NAU8310_RF000_DSP_COMM:
		return true;
	default:
		return false;
	}

}

static bool nau8310_writeable_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case NAU8310_R00_HARDWARE_RST ... NAU8310_R10_RIGHT_TIME_SLOT:
	case NAU8310_R12_HPF_CTRL ... NAU8310_R1A_DSP_CORE_CTRL2:
	case NAU8310_R28_ADC_RATE ... NAU8310_R2A_DAC_CTRL2:
	case NAU8310_R2C_ALC_CTRL1 ... NAU8310_R33_LPF_CTRL:
	case NAU8310_R40_CLK_DET_CTRL:
	case NAU8310_R55_MISC_CTRL:
	case NAU8310_R60_BIAS_ADJ ... NAU8310_R66_ANALOG_CONTROL_6:
	case NAU8310_R68_ANALOG_CONTROL_7 ... NAU8310_R69_CLIP_CTRL:
	case NAU8310_R6B_ANALOG_CONTROL_8 ... NAU8310_R6C_ANALOG_CONTROL_9:
	case NAU8310_R71_ANALOG_ADC_1 ... NAU8310_R73_RDAC:
	case NAU8310_R76_BOOST ... NAU8310_R77_FEPGA:
	case NAU8310_R7F_POWER_UP_CONTROL ... NAU8310_R9D_BIQ2_COE_10:
	case NAU8310_RF000_DSP_COMM:
		return true;
	default:
		return false;
	}
}

static bool nau8310_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case NAU8310_R00_HARDWARE_RST ... NAU8310_R01_SOFTWARE_RST:
	case NAU8310_R06_INT_CLR_STATUS:
	case NAU8310_R1B_CLK_DOUBLER_O ... NAU8310_R27_DSP_STATUS_2:
	case NAU8310_R46_I2C_DEVICE_ID:
	case NAU8310_R49_SARDOUT_RAM_STATUS ... NAU8310_R4A_ANALOG_READ:
	case NAU8310_RF000_DSP_COMM:
		return true;
	default:
		return false;
	}
}

static int nau8310_clkdet_put(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
			(struct soc_mixer_control *)kcontrol->private_value;
	struct snd_soc_component *component =
			snd_soc_kcontrol_component(kcontrol);
	struct nau8310 *nau8310 = snd_soc_component_get_drvdata(component);
	unsigned int max = mc->max, min = mc->min, val;
	unsigned int mask = (1 << fls(max)) - 1;

	val = ((ucontrol->value.integer.value[0] + min) & mask);
	nau8310->clock_detection = val;

	if (nau8310->clock_detection)
		regmap_update_bits(nau8310->regmap, NAU8310_R40_CLK_DET_CTRL,
				   NAU8310_CLKPWRUP_DIS, 0);
	else
		regmap_update_bits(nau8310->regmap, NAU8310_R40_CLK_DET_CTRL,
				   NAU8310_CLKPWRUP_DIS, NAU8310_CLKPWRUP_DIS);

	return 0;
}

int nau8310_clkdet_get(struct snd_kcontrol *kcontrol,
		       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
			snd_soc_kcontrol_component(kcontrol);
	struct nau8310 *nau8310 = snd_soc_component_get_drvdata(component);

	ucontrol->value.integer.value[0] = nau8310->clock_detection;

	return 0;
}

static int nau8310_biq_coeff_get(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
			snd_soc_kcontrol_component(kcontrol);
	struct nau8310 *nau8310 = snd_soc_component_get_drvdata(component);
	struct soc_bytes_ext *params = (void *)kcontrol->private_value;
	int i, reg, reg_val;
	u16 *val;

	val = (u16 *)ucontrol->value.bytes.data;
	reg = NAU8310_R80_BIQ0_COE_1;
	for (i = 0; i < params->max / sizeof(u16); i++) {
		regmap_read(nau8310->regmap, reg + i, &reg_val);
		/* conversion of 16-bit integers between native CPU format
		 * and big endian format
		 */
		reg_val = cpu_to_be16(reg_val);
		*(val + i) = reg_val;
	}

	return 0;
}

static int nau8310_biq_coeff_put(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
			snd_soc_kcontrol_component(kcontrol);
	struct nau8310 *nau8310 = snd_soc_component_get_drvdata(component);
	struct soc_bytes_ext *params = (void *)kcontrol->private_value;
	void *data;
	u16 *val, value;
	int i, reg, ret;

	data = kmemdup(ucontrol->value.bytes.data,
		       params->max, GFP_KERNEL | GFP_DMA);
	if (!data)
		return -ENOMEM;

	val = (u16 *)data;
	reg = NAU8310_R80_BIQ0_COE_1;
	for (i = 0; i < params->max / sizeof(u16); i++) {
		/* conversion of 16-bit integers between native CPU format
		 * and big endian format
		 */
		value = be16_to_cpu(*(val + i));
		ret = regmap_write(nau8310->regmap, reg + i, value);
		if (ret) {
			dev_err(nau8310->dev, "BIQ configuration fail, register: %x ret: %d\n",
				reg + i, ret);
			kfree(data);
			return ret;
		}
	}
	kfree(data);

	return 0;
}

static const char * const nau8310_adc_decimation[] = { "32", "64", "128" };

static const struct soc_enum nau8310_adc_decimation_enum =
	SOC_ENUM_SINGLE(NAU8310_R28_ADC_RATE, NAU8310_ADC_SYNC_DOWN_SFT,
			ARRAY_SIZE(nau8310_adc_decimation), nau8310_adc_decimation);

static const char * const nau8310_dac_oversampl[] = {
	"64", "256", "128", "", "32" };

static const struct soc_enum nau8310_dac_oversampl_enum =
	SOC_ENUM_SINGLE(NAU8310_R29_DAC_CTRL1, NAU8310_DAC_OVERSAMPLE_SFT,
			ARRAY_SIZE(nau8310_dac_oversampl), nau8310_dac_oversampl);

static const char * const nau8310_i2s_channel[] = { "Right", "Left"};

static const char * const nau8310_source[] = { "DSP", "Raw Data"};

static const char * const nau8310_tdm_slot[] = { "Slot 0", "Slot 1", "Slot 2",
	"Slot 3", "Slot 4", "Slot 5", "Slot 6", "Slot 7" };

static const struct soc_enum nau8310_aec_channel_sel_enum =
	SOC_ENUM_SINGLE(NAU8310_R0B_I2S_PCM_CTRL0, NAU8310_AEC_CH_SEL_SFT,
			ARRAY_SIZE(nau8310_i2s_channel), nau8310_i2s_channel);

static const struct soc_enum nau8310_aec_source_sel_enum =
	SOC_ENUM_SINGLE(NAU8310_R0B_I2S_PCM_CTRL0, NAU8310_AEC_SRC_SEL_SFT,
			ARRAY_SIZE(nau8310_source), nau8310_source);

static const struct soc_enum nau8310_dac_sel_enum =
	SOC_ENUM_SINGLE(NAU8310_R0B_I2S_PCM_CTRL0, NAU8310_DAC_SEL_SFT,
			ARRAY_SIZE(nau8310_tdm_slot), nau8310_tdm_slot);

static const char * const nau8310_adc_alcgain_slot[] = { "Disable", "Slot 0",
	"Slot 1", "Slot 2", "Slot 3", "Slot 4", "Slot 5", "Slot 6", "Slot 7" };

static const unsigned int nau8310_adc_alcgain_val[] = {
	0x0, 0x8, 0x9, 0xa, 0xb, 0xc, 0xd, 0xe, 0xf };

static const struct soc_enum nau8310_adc_alcgain_sel_enum =
	SOC_VALUE_ENUM_SINGLE(NAU8310_R0C_TDM_CTRL, NAU8310_ADC_ALC_SEL_SFT,
			      0xf, ARRAY_SIZE(nau8310_adc_alcgain_slot),
			      nau8310_adc_alcgain_slot, nau8310_adc_alcgain_val);

static const char * const nau8310_adc_isense_slot[] = { "Disable", "Slot 1",
	"Slot 3", "Slot 5", "Slot 7" };

static const unsigned int nau8310_adc_isense_val[] = { 0x0, 0x4, 0x5, 0x6, 0x7 };

static const struct soc_enum nau8310_adc_sel_i_enum =
	SOC_VALUE_ENUM_SINGLE(NAU8310_R0C_TDM_CTRL, NAU8310_ADC_I_SEL_SFT,
			      0x7, ARRAY_SIZE(nau8310_adc_isense_slot),
			      nau8310_adc_isense_slot, nau8310_adc_isense_val);

static const char * const nau8310_adc_vsense_slot[] = { "Disable", "Slot 0",
	"Slot 2", "Slot 4", "Slot 6" };

static const unsigned int nau8310_adc_vsense_val[] = { 0x0, 0x4, 0x5, 0x6, 0x7 };

static const struct soc_enum nau8310_adc_sel_v_enum =
	SOC_VALUE_ENUM_SINGLE(NAU8310_R0C_TDM_CTRL, NAU8310_ADC_V_SEL_SFT,
			      0x7, ARRAY_SIZE(nau8310_adc_vsense_slot),
			      nau8310_adc_vsense_slot, nau8310_adc_vsense_val);

static const char * const nau8310_ivsense_format[] = { "signed", "unsigned" };

static const struct soc_enum nau8310_ivsense_format_enum =
	SOC_ENUM_SINGLE(NAU8310_R28_ADC_RATE, NAU8310_UNSIGN_IV_SFT,
			ARRAY_SIZE(nau8310_ivsense_format), nau8310_ivsense_format);

/* LB/LM is referred to low battery/limiter mode. */
static const char * const nau8310_limiter_mode[] = {
	"LM with Clip Detection",
	"LBLM with Clip Detection",
	"LM with Pre-Programmed Limit Level",
	"LBLM with Pre-Programmed Limit Level",
	"LBLM with Pre-Programmed VBAT Ratio Limit Level" };

static const struct soc_enum nau8310_limiter_mode_enum =
	SOC_ENUM_SINGLE(NAU8310_R2E_ALC_CTRL3, NAU8310_LIM_MDE_SFT,
			ARRAY_SIZE(nau8310_limiter_mode), nau8310_limiter_mode);

static const DECLARE_TLV_DB_MINMAX(adc_vol_tlv, 0, 24125);
static const DECLARE_TLV_DB_MINMAX(pga_gain_tlv, 0, 1600);

static const struct snd_kcontrol_new nau8310_snd_controls[] = {
	SOC_ENUM("ADC Decimation Rate", nau8310_adc_decimation_enum),
	SOC_ENUM("DAC Oversampling Rate", nau8310_dac_oversampl_enum),
	SND_SOC_BYTES_EXT("BIQ Coefficients", 60,
			  nau8310_biq_coeff_get, nau8310_biq_coeff_put),

	SOC_SINGLE_TLV("ADC Left Channel Volume",
		       NAU8310_R14_ADC_VOL_CTRL, NAU8310_ADC_GAIN_L_SFT,
		       NAU8310_ADC_GAIN_L_MAX, 0, adc_vol_tlv),
	SOC_SINGLE_TLV("ADC Right Channel Volume",
		       NAU8310_R14_ADC_VOL_CTRL, NAU8310_ADC_GAIN_R_SFT,
		       NAU8310_ADC_GAIN_R_MAX, 0, adc_vol_tlv),
	SOC_SINGLE_TLV("PGA Volume", NAU8310_R7F_POWER_UP_CONTROL,
		       NAU8310_PGA_GAIN_SFT, NAU8310_PGA_GAIN_MAX, 0, pga_gain_tlv),

	/* Boost Control Target Value Limit. 0.16 V per step. */
	SOC_SINGLE("Boost Target Limit", NAU8310_R17_BOOST_CTRL1,
		   NAU8310_BSTLIMIT_SFT, NAU8310_BSTLIMIT_MAX, 0),
	/* Boost Margin Value between Boost Target Voltage and
	 * Peak Output Level. 0.16 V per step.
	 */
	SOC_SINGLE("Boost Margin", NAU8310_R17_BOOST_CTRL1,
		   NAU8310_BSTMARGIN_SFT, NAU8310_BSTMARGIN_MAX, 0),

	SOC_ENUM("AEC Channel", nau8310_aec_channel_sel_enum),
	SOC_ENUM("AEC Source", nau8310_aec_source_sel_enum),
	SOC_ENUM("DAC Channel Source", nau8310_dac_sel_enum),
	SOC_ENUM("ADC ALC Gain Channel Source", nau8310_adc_alcgain_sel_enum),
	SOC_ENUM("ADC I Sense Channel Source", nau8310_adc_sel_i_enum),
	SOC_ENUM("ADC V Sense Channel Source", nau8310_adc_sel_v_enum),

	SOC_SINGLE("PingPong Mode", NAU8310_R0C_TDM_CTRL,
		   NAU8310_PINGPONG_SFT, 1, 0),
	SOC_ENUM("IV Sense Format", nau8310_ivsense_format_enum),

	SOC_SINGLE_EXT("Clock Detection", SND_SOC_NOPM, 0, 1, 0,
		       nau8310_clkdet_get, nau8310_clkdet_put),

	SOC_SINGLE("ALC Max Gain", NAU8310_R2C_ALC_CTRL1,
		   NAU8310_ALC_MAXGAIN_SFT, NAU8310_ALC_MAXGAIN_MAX, 0),
	SOC_SINGLE("ALC Min Gain", NAU8310_R2C_ALC_CTRL1,
		   NAU8310_ALC_MINGAIN_SFT, NAU8310_ALC_MINGAIN_MAX, 0),
	SOC_SINGLE("ALC Decay Timer", NAU8310_R2D_ALC_CTRL2,
		   NAU8310_ALC_DCY_SFT, NAU8310_ALC_DCY_MAX, 0),
	SOC_SINGLE("ALC Attack Timer", NAU8310_R2D_ALC_CTRL2,
		   NAU8310_ALC_ATK_SFT, NAU8310_ALC_ATK_MAX, 0),
	SOC_SINGLE("ALC Hold Time", NAU8310_R2D_ALC_CTRL2,
		   NAU8310_ALC_HLD_SFT, NAU8310_ALC_HLD_MAX, 0),
	SOC_SINGLE("ALC Target Level", NAU8310_R2D_ALC_CTRL2,
		   NAU8310_ALC_LVL_SFT, NAU8310_ALC_LVL_MAX, 0),
	SOC_SINGLE("ALC Enable", NAU8310_R2E_ALC_CTRL3,
		   NAU8310_ALC_EN_SFT, 1, 0),
	SOC_ENUM("Limiter Mode Selection", nau8310_limiter_mode_enum),
	SOC_SINGLE("Low Battery Limiter Threshold", NAU8310_R2E_ALC_CTRL3,
		   NAU8310_VBAT_THLD_SFT, NAU8310_VBAT_THLD_MAX, 0),
	SOC_SINGLE("Auto Attenuation Enable", NAU8310_R2E_ALC_CTRL3,
		   NAU8310_AUTOATT_EN_SFT, 1, 0),

	SOC_SINGLE("BCLK IO Driver Stronger", NAU8310_R0A_IO_CTRL,
		   NAU8310_BCLK_DS_SFT, 1, 0),
	SOC_SINGLE("LRC IO Driver Stronger", NAU8310_R0A_IO_CTRL,
		   NAU8310_LRC_DS_SFT, 1, 0),
	SOC_SINGLE("ADCDAT IO Driver Stronger", NAU8310_R0A_IO_CTRL,
		   NAU8310_ADCDAT_DS_SFT, 1, 0),

	SOC_SINGLE("Low Output Noise in Receiver Mode",
		   NAU8310_R64_ANALOG_CONTROL_4, NAU8310_RECV_MODE_SFT, 1, 0),

	SOC_SINGLE("VBAT Peak Current Limit",
		   NAU8310_R6B_ANALOG_CONTROL_8,
		   NAU8310_VBAT_PCL_SFT, NAU8310_VBAT_PCL_MAX, 0),
	SOC_SINGLE("VBAT Comparator Threshold",
		   NAU8310_R6B_ANALOG_CONTROL_8,
		   NAU8310_VBAT_THD_SFT, NAU8310_VBAT_THD_MAX, 0),

	SOC_SINGLE("ISense PGA Supply Current Trim",
		   NAU8310_R77_FEPGA,
		   NAU8310_CURR_TRIM_SFT, NAU8310_CURR_TRIM_MAX, 0),
};

static const struct snd_kcontrol_new nau8310_alc_gain_select =
	SOC_DAPM_SINGLE("Switch", NAU8310_R2C_ALC_CTRL1,
			NAU8310_ALC_GAIN_SEL_SFT, 1, 0);

static int nau8310_clock_event(struct snd_soc_dapm_widget *w,
			       struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component =
			snd_soc_dapm_to_component(w->dapm);
	struct nau8310 *nau8310 = snd_soc_component_get_drvdata(component);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		/* Switch the clock source of DSP to MCLK. */
		regmap_update_bits(nau8310->regmap, NAU8310_R04_ENA_CTRL,
				   NAU8310_DSP_SEL_OSC, 0);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int nau8310_powerup_event(struct snd_soc_dapm_widget *w,
				 struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component =
			snd_soc_dapm_to_component(w->dapm);
	struct nau8310 *nau8310 = snd_soc_component_get_drvdata(component);

	if (nau8310->clock_detection)
		return 0;

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		regmap_update_bits(nau8310->regmap, NAU8310_R40_CLK_DET_CTRL,
				   NAU8310_PWRUP_DFT, NAU8310_PWRUP_DFT);
		break;
	case SND_SOC_DAPM_POST_PMD:
		regmap_update_bits(nau8310->regmap, NAU8310_R40_CLK_DET_CTRL,
				   NAU8310_PWRUP_DFT, 0);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int nau8310_adc_event(struct snd_soc_dapm_widget *w,
			     struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component =
			snd_soc_dapm_to_component(w->dapm);
	struct nau8310 *nau8310 = snd_soc_component_get_drvdata(component);

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		regmap_update_bits(nau8310->regmap, NAU8310_R0E_I2S_PCM_CTRL2,
				   NAU8310_I2S_ADCDAT_OE_MASK, NAU8310_I2S_ADCDAT_OE_EN);
		break;
	case SND_SOC_DAPM_POST_PMD:
		regmap_update_bits(nau8310->regmap, NAU8310_R0E_I2S_PCM_CTRL2,
				   NAU8310_I2S_ADCDAT_OE_MASK, NAU8310_I2S_ADCDAT_OE_DIS);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int nau8310_dac_event(struct snd_soc_dapm_widget *w,
			     struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component =
			snd_soc_dapm_to_component(w->dapm);
	struct nau8310 *nau8310 = snd_soc_component_get_drvdata(component);

	/* Soft mute to prevent the pop noise */
	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		regmap_update_bits(nau8310->regmap, NAU8310_R13_MUTE_CTRL,
				   NAU8310_SOFT_MUTE, 0);
		msleep(30);
		regmap_update_bits(nau8310->regmap, NAU8310_R0E_I2S_PCM_CTRL2,
				   NAU8310_I2S_TRISTATE, 0);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		regmap_update_bits(nau8310->regmap, NAU8310_R0E_I2S_PCM_CTRL2,
				   NAU8310_I2S_TRISTATE, NAU8310_I2S_TRISTATE);
		regmap_update_bits(nau8310->regmap, NAU8310_R13_MUTE_CTRL,
				   NAU8310_SOFT_MUTE, NAU8310_SOFT_MUTE);
		msleep(30);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int check_dsp_enabled(struct snd_soc_dapm_widget *source,
			     struct snd_soc_dapm_widget *sink)
{
	struct snd_soc_component *component =
			snd_soc_dapm_to_component(source->dapm);
	struct nau8310 *nau8310 = snd_soc_component_get_drvdata(component);

	return nau8310->dsp_enable;
}

static const struct snd_soc_dapm_widget nau8310_dapm_widgets[] = {
	SND_SOC_DAPM_SUPPLY("Clock", SND_SOC_NOPM, 0, 0,
			    nau8310_clock_event, SND_SOC_DAPM_PRE_PMU),
	SND_SOC_DAPM_SUPPLY("PowerUp", SND_SOC_NOPM, 0, 0,
			    nau8310_powerup_event, SND_SOC_DAPM_POST_PMU |
			    SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_SWITCH_E("ALC gain selection mode", SND_SOC_NOPM,
			      0, 0, &nau8310_alc_gain_select, nau8310_adc_event,
			      SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_SIGGEN("Sense"),

	SND_SOC_DAPM_PGA("SAR", NAU8310_R07_SAR_CTRL1, NAU8310_SAR_ENA_SFT,
			 0, NULL, 0),
	SND_SOC_DAPM_PGA("ADC_I", NAU8310_R04_ENA_CTRL,
			 NAU8310_ADCEN_I_SFT, 0, NULL, 0),
	SND_SOC_DAPM_PGA("ADC_V", NAU8310_R04_ENA_CTRL,
			 NAU8310_ADCEN_V_SFT, 0, NULL, 0),
	SND_SOC_DAPM_PGA("Capture Sense", SND_SOC_NOPM, 0, 0, NULL, 0),

	SND_SOC_DAPM_ADC_E("ADC_OUT", "Capture", SND_SOC_NOPM, 0, 0,
			   nau8310_adc_event, SND_SOC_DAPM_POST_PMU |
			   SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_AIF_IN("AIFRX", "Playback", 0, SND_SOC_NOPM, 0, 0),

	SND_SOC_DAPM_DAC_E("DAC", NULL, NAU8310_R04_ENA_CTRL,
			   NAU8310_DAC_CH_EN_SFT, 0,
			   nau8310_dac_event, SND_SOC_DAPM_POST_PMU |
			   SND_SOC_DAPM_PRE_PMD),

	SND_SOC_DAPM_OUTPUT("Speaker"),
};

static const struct snd_soc_dapm_route nau8310_dapm_routes[] = {
	{ "SAR", NULL, "Sense" },
	{ "ADC_I", NULL, "Sense" },
	{ "ADC_V", NULL, "Sense" },
	{ "Capture Sense", NULL, "SAR" },
	{ "Capture Sense", NULL, "ADC_I" },
	{ "Capture Sense", NULL, "ADC_V" },

	{ "ALC gain selection mode", "Switch", "Capture Sense" },
	{ "ADC_OUT", NULL, "Capture Sense" },

	{ "AIFRX", NULL, "Clock", check_dsp_enabled },
	{ "DAC", NULL, "PowerUp" },
	{ "DAC", NULL, "ALC gain selection mode" },
	{ "DAC", NULL, "AIFRX" },
	{ "Speaker", NULL, "DAC"},
};

static int nau8310_srate_clk_apply(struct nau8310 *nau8310,
				   const struct nau8310_srate_attr *srate_table,
				   int n1_sel, int mclk_mult_sel, int n2_sel, int dsp_mult_sel)
{
	int dsp_clk, mult_sel;

	if (!srate_table || srate_table->adc_div < 0 ||
		dsp_mult_sel < 0 || dsp_mult_sel >= ARRAY_SIZE(dsp_src_mult) ||
		n2_sel < 0 || n2_sel >= ARRAY_SIZE(mclk_n2_div) ||
		n1_sel < 0 || n1_sel >= ARRAY_SIZE(mclk_n1_div)) {
		dev_err(nau8310->dev, "The fs %d isn't support in DSP.\n",
			nau8310->fs);
		return -EINVAL;
	}
	regmap_update_bits(nau8310->regmap, NAU8310_R03_CLK_CTRL,
			   NAU8310_CLK_ADC_DIV2 | NAU8310_CLK_ADC_DIV4,
			   srate_table->adc_div << NAU8310_CLK_ADC_DIV4_SFT);
	regmap_update_bits(nau8310->regmap, NAU8310_R40_CLK_DET_CTRL,
			   NAU8310_SRATE_MASK | NAU8310_DIV_MAX,
			   (srate_table->range << NAU8310_SRATE_SFT) |
			   (srate_table->max ? NAU8310_DIV_MAX : 0));
	if (srate_table->fs == 44100)
		regmap_update_bits(nau8310->regmap, NAU8310_R40_CLK_DET_CTRL,
				   NAU8310_ALT_SRATE_EN, NAU8310_ALT_SRATE_EN);
	else
		regmap_update_bits(nau8310->regmap, NAU8310_R40_CLK_DET_CTRL,
				   NAU8310_ALT_SRATE_EN, 0);

	dsp_clk = nau8310->mclk << dsp_src_mult[dsp_mult_sel].param;
	dsp_clk = dsp_clk / mclk_n1_div[n1_sel].param;

	dev_dbg(nau8310->dev, "FS %dHz, DSP_CLK %uHz, n1_sel (%d), mclk_mult_sel (%d), n2_sel (%d), dsp_mult_sel (%d).\n",
		nau8310->fs, dsp_clk, n1_sel, mclk_mult_sel, n2_sel, dsp_mult_sel);

	regmap_update_bits(nau8310->regmap, NAU8310_R04_ENA_CTRL,
			   NAU8310_CLK_DSP_SRC_MASK,
			   dsp_src_mult[dsp_mult_sel].val << NAU8310_CLK_DSP_SRC_SFT);
	regmap_update_bits(nau8310->regmap, NAU8310_R03_CLK_CTRL,
			   NAU8310_MCLK_SRC_MASK, mclk_n2_div[n2_sel].val);
	regmap_update_bits(nau8310->regmap, NAU8310_R04_ENA_CTRL,
			   NAU8310_CLK_MUL_SRC_MASK,
			   mclk_n1_div[n1_sel].val << NAU8310_CLK_MUL_SRC_SFT);
	if (mclk_mult_sel != CLK_PROC_BYPASS) {
		regmap_update_bits(nau8310->regmap, NAU8310_R04_ENA_CTRL,
				   NAU8310_MCLK_SEL_MASK,
				   mclk_src_mult[mclk_mult_sel].val <<
				   NAU8310_MCLK_SEL_SFT);
		mult_sel = (mclk_mult_sel > dsp_mult_sel ? mclk_mult_sel : dsp_mult_sel);
	} else {
		mult_sel = dsp_mult_sel;
		regmap_update_bits(nau8310->regmap, NAU8310_R04_ENA_CTRL,
						   NAU8310_MCLK_SEL_MASK, 0);
	}
	switch (mult_sel) {
	case 4:	/* multiplier 16x, i.e. 2^4 */
		regmap_update_bits(nau8310->regmap, NAU8310_R68_ANALOG_CONTROL_7,
				   NAU8310_MCLKX_MASK, NAU8310_MCLK16XEN |
				   NAU8310_MCLK8XEN | NAU8310_MCLK4XEN);
		break;
	case 3:	/* multiplier 8x, i.e. 2^3 */
		regmap_update_bits(nau8310->regmap, NAU8310_R68_ANALOG_CONTROL_7,
				   NAU8310_MCLKX_MASK, NAU8310_MCLK8XEN |
				   NAU8310_MCLK4XEN);
		break;
	case 2:	/* multiplier 4x, i.e. 2^2 */
		regmap_update_bits(nau8310->regmap, NAU8310_R68_ANALOG_CONTROL_7,
				   NAU8310_MCLKX_MASK, NAU8310_MCLK4XEN);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

int nau8310_clkdsp_choose(struct nau8310 *nau8310, int *n1_sel, bool n1_try,
			  int *dsp_n1_sel, int *dsp_mult_sel)
{
	int i, j, dsp_n1, mult_sel, dsp_clk, dsp_clk_max = 0;

	if (!nau8310->fs || !nau8310->mclk)
		goto proc_err;

	if (n1_sel) {
		dsp_n1 = *n1_sel;
		for (j = 0; j < ARRAY_SIZE(dsp_src_mult); j++) {
			dsp_clk = nau8310->mclk << dsp_src_mult[j].param;
			dsp_clk = dsp_clk / mclk_n1_div[dsp_n1].param;
			if (dsp_clk < (nau8310->fs * DSP_CLK_MIN_MULTI_VALUE) ||
				dsp_clk > DSP_CLK_MAX)
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
		if (nau8310->mclk >= DSP_MCLK_HI_SPEED) {
			/* get max N1 when MCLK over 12.288MHz */
			for (i = 0; i < ARRAY_SIZE(mclk_n1_div); i++)
				for (j = 0; j < ARRAY_SIZE(dsp_src_mult); j++) {
					dsp_clk = nau8310->mclk << dsp_src_mult[j].param;
					dsp_clk = dsp_clk / mclk_n1_div[i].param;
					if (dsp_clk < (nau8310->fs * DSP_CLK_MIN_MULTI_VALUE) ||
						dsp_clk > DSP_CLK_MAX)
						continue;
					if ((dsp_clk_max < dsp_clk || i > dsp_n1)) {
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
					dsp_clk = nau8310->mclk << dsp_src_mult[j].param;
					dsp_clk = dsp_clk / mclk_n1_div[i].param;
					if (dsp_clk >= (nau8310->fs * DSP_CLK_MIN_MULTI_VALUE) &&
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

static int nau8310_clock_check(struct nau8310 *nau8310, int stream, int rate,
			       int osr)
{
	int osrate = 0;

	if (stream == SNDRV_PCM_STREAM_PLAYBACK) {
		if (osr >= ARRAY_SIZE(osr_dac_sel))
			goto err;
		osrate = osr_dac_sel[osr].osr;
	} else {
		if (nau8310->dsp_enable)
			if (osr >= ARRAY_SIZE(osr_adc_dsp_sel))
				goto err;
			else
				osrate = osr_adc_dsp_sel[osr].osr;
		else
			if (osr >= ARRAY_SIZE(osr_adc_sel))
				goto err;
			else
				osrate = osr_adc_sel[osr].osr;
	}

	if (!osrate || rate * osrate > CLK_DA_AD_MAX) {
		dev_err(nau8310->dev, "exceed the maximum frequency of CLK_ADC or CLK_DAC\n");
		goto err;
	}

	return 0;
err:
	return -EINVAL;
}

static int nau8310_osr_apply(struct nau8310 *nau8310)
{
	unsigned int osr, stream;

	/* CLK_DAC or CLK_ADC = OSR * FS
	 * DAC or ADC clock frequency is defined as Over Sampling Rate (OSR)
	 * multiplied by the audio sample rate (Fs). Note that the OSR and Fs
	 * values must be selected such that the maximum frequency is less
	 * than 6.144 MHz.
	 */
	regmap_read(nau8310->regmap, NAU8310_R29_DAC_CTRL1, &osr);
	osr &= NAU8310_DAC_OVERSAMPLE_MASK;
	stream = SNDRV_PCM_STREAM_PLAYBACK;
	if (nau8310_clock_check(nau8310, stream, nau8310->fs, osr))
		return -EINVAL;
	regmap_update_bits(nau8310->regmap, NAU8310_R03_CLK_CTRL,
			   NAU8310_CLK_DAC_SRC_MASK,
			   osr_dac_sel[osr].clk_src << NAU8310_CLK_DAC_SRC_SFT);

	regmap_read(nau8310->regmap, NAU8310_R28_ADC_RATE, &osr);
	osr &= NAU8310_ADC_SYNC_DOWN_MASK;
	stream = SNDRV_PCM_STREAM_CAPTURE;
	if (nau8310_clock_check(nau8310, stream, nau8310->fs, osr))
		return -EINVAL;

	if (nau8310->dsp_enable) {
		regmap_update_bits(nau8310->regmap, NAU8310_R03_CLK_CTRL,
				   NAU8310_CLK_ADC_SRC_MASK,
				   osr_adc_dsp_sel[osr].clk_src << NAU8310_CLK_ADC_SRC_SFT);
	} else {
		regmap_update_bits(nau8310->regmap, NAU8310_R03_CLK_CTRL,
				   NAU8310_CLK_ADC_SRC_MASK,
				   osr_adc_sel[osr].clk_src << NAU8310_CLK_ADC_SRC_SFT);
	}

	return 0;
}

static int nau8310_clksrc_n2(struct nau8310 *nau8310, const struct nau8310_srate_attr *srate_table,
			     int mclk, int *n2_sel)
{
	int i, mclk_src, ratio;

	ratio = NAU8310_MCLK_FS_RATIO_NUM;
	for (i = 0; i < ARRAY_SIZE(mclk_n2_div); i++) {
		mclk_src = mclk >> mclk_n2_div[i].param;
		if (srate_table->mclk_src[NAU8310_MCLK_FS_RATIO_256] == mclk_src) {
			ratio = NAU8310_MCLK_FS_RATIO_256;
			break;
		} else if (srate_table->mclk_src[NAU8310_MCLK_FS_RATIO_400] == mclk_src) {
			ratio = NAU8310_MCLK_FS_RATIO_400;
			break;
		} else if (srate_table->mclk_src[NAU8310_MCLK_FS_RATIO_500] == mclk_src) {
			ratio = NAU8310_MCLK_FS_RATIO_500;
			break;
		}
	}
	if (ratio != NAU8310_MCLK_FS_RATIO_NUM)
		*n2_sel = i;

	return ratio;
}

static const struct nau8310_srate_attr *target_srate_attribute(int srate)
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

static int nau8310_clksrc_choose(struct nau8310 *nau8310,
				 const struct nau8310_srate_attr **srate_table,
				 int *n1_sel, int *mult_sel, int *n2_sel, int *dsp_mult_sel)
{
	int i, j, mclk, mclk_max, ratio, ratio_sel, n2_max, ret;

	if (!nau8310->mclk || !nau8310->fs)
		goto proc_err;

	/* select sampling rate and MCLK_SRC */
	*srate_table = target_srate_attribute(nau8310->fs);
	if (*srate_table == NULL)
		goto proc_err;

	/* First check clock from MCLK directly, decide N2 for MCLK_SRC.
	 * If not good, consider 1/N1 and Multiplier.
	 */
	ratio = nau8310_clksrc_n2(nau8310, *srate_table, nau8310->mclk, n2_sel);
	if (ratio != NAU8310_MCLK_FS_RATIO_NUM) {
		*n1_sel = CLK_PROC_BYPASS;
		*mult_sel = CLK_PROC_BYPASS;
		ret = nau8310_clkdsp_choose(nau8310, NULL, false,
					    n1_sel, dsp_mult_sel);
		if (ret)
			goto proc_err;
		goto proc_done;
	}
	/* Get MCLK_SRC through 1/N, Multiplier, and then 1/N2. */
	mclk_max = 0;
	for (i = 0; i < ARRAY_SIZE(mclk_n1_div); i++) {
		for (j = 0; j < ARRAY_SIZE(mclk_src_mult); j++) {
			mclk = nau8310->mclk << mclk_src_mult[j].param;
			mclk = mclk / mclk_n1_div[i].param;
			ratio = nau8310_clksrc_n2(nau8310, *srate_table, mclk, n2_sel);
			if (ratio != NAU8310_MCLK_FS_RATIO_NUM &&
				(mclk_max < mclk || i > *n1_sel)) {
				ret = nau8310_clkdsp_choose(nau8310, &i, true,
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
		nau8310_clkdsp_choose(nau8310, n1_sel, false, n1_sel, dsp_mult_sel);
		goto proc_done;
	}

proc_err:
	dev_err(nau8310->dev, "The MCLK %d is invalid. It can't get MCLK_SRC of 256/400/500 FS (%d).\n",
		nau8310->mclk, nau8310->fs);
	return -EINVAL;
proc_done:
	dev_dbg(nau8310->dev, "FS %dHz, range 0x%x, %s, (n1,mu,n2,dmu):(%d,%d,%d,%d), MCLK_SRC %uHz (%d)\n",
		nau8310->fs, (*srate_table)->range,
		(*srate_table)->max ? "MAX" : "MIN",
		*n1_sel == CLK_PROC_BYPASS ?
		CLK_PROC_BYPASS : mclk_n1_div[*n1_sel].param,
		*mult_sel == CLK_PROC_BYPASS ?
		CLK_PROC_BYPASS : 1 << mclk_src_mult[*mult_sel].param,
		1 << mclk_n2_div[*n2_sel].param,
		1 << dsp_src_mult[*dsp_mult_sel].param,
		(*srate_table)->mclk_src[ratio],
		(*srate_table)->mclk_src[ratio] / nau8310->fs);
	return 0;
}

static int nau8310_clock_config(struct nau8310 *nau8310)
{
	const struct nau8310_srate_attr *srate_table;
	int ret, n1_sel, mult_sel, n2_sel, dsp_mult_sel;

	ret = nau8310_clksrc_choose(nau8310, &srate_table, &n1_sel, &mult_sel,
				    &n2_sel, &dsp_mult_sel);
	if (ret)
		goto err;
	ret = nau8310_srate_clk_apply(nau8310, srate_table, n1_sel, mult_sel,
				      n2_sel, dsp_mult_sel);
	if (ret)
		goto err;

	ret = nau8310_osr_apply(nau8310);
	if (ret)
		goto err;

	return 0;
err:
	return ret;
}

static int nau8310_trigger(struct snd_pcm_substream *substream,
		int cmd, struct snd_soc_dai *dai)
{
	struct snd_soc_component *component = dai->component;
	struct nau8310 *nau8310 = snd_soc_component_get_drvdata(component);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
		/* Switch the clock source of DSP to OSC. */
		regmap_update_bits(nau8310->regmap, NAU8310_R04_ENA_CTRL,
				   NAU8310_DSP_SEL_OSC, NAU8310_DSP_SEL_OSC);
		dev_dbg(component->dev, "Switch the clock source of DSP to OSC");
		break;
	}

	return 0;
}

static int nau8310_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	struct snd_soc_component *component = dai->component;
	struct nau8310 *nau8310 = snd_soc_component_get_drvdata(component);
	unsigned int val_len = 0;
	int ret;

	nau8310->fs = params_rate(params);
	if (nau8310->dsp_enable && nau8310->fs != 48000) {
		dev_err(nau8310->dev, "For ROM Code revision 00, DSP only run at 48kHz.\n");
		return -EINVAL;
	}
	ret = nau8310_clock_config(nau8310);
	if (ret)
		goto err;

	switch (params_width(params)) {
	case 16:
		val_len |= NAU8310_I2S_DL_16;
		break;
	case 20:
		val_len |= NAU8310_I2S_DL_20;
		break;
	case 24:
		val_len |= NAU8310_I2S_DL_24;
		break;
	case 32:
		val_len |= NAU8310_I2S_DL_32;
		break;
	default:
		ret = -EINVAL;
		goto err;
	}

	regmap_update_bits(nau8310->regmap, NAU8310_R0D_I2S_PCM_CTRL1,
			   NAU8310_I2S_DL_MASK, val_len);

	return 0;
err:
	return ret;
}

static int nau8310_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct snd_soc_component *component = dai->component;
	struct nau8310 *nau8310 = snd_soc_component_get_drvdata(component);
	unsigned int ctrl1_val = 0, ctrl2_val = 0;

	dev_dbg(component->dev, "%s: fmt 0x%08X\n", __func__, fmt);

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		ctrl2_val |= NAU8310_I2S_MS_MASTER;
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
		ctrl1_val |= NAU8310_I2S_BP_INV;
		break;
	default:
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		ctrl1_val |= NAU8310_I2S_DF_I2S;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		ctrl1_val |= NAU8310_I2S_DF_LEFT;
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		ctrl1_val |= NAU8310_I2S_DF_RIGHT;
		break;
	case SND_SOC_DAIFMT_DSP_A:
		ctrl1_val |= NAU8310_I2S_DF_PCM_AB;
		break;
	case SND_SOC_DAIFMT_DSP_B:
		ctrl1_val |= NAU8310_I2S_DF_PCM_AB;
		ctrl1_val |= NAU8310_I2S_PCMB_EN;
		break;
	default:
		return -EINVAL;
	}

	regmap_update_bits(nau8310->regmap, NAU8310_R0D_I2S_PCM_CTRL1,
			   NAU8310_I2S_DF_MASK | NAU8310_I2S_BP_MASK |
			   NAU8310_I2S_PCMB_EN, ctrl1_val);
	regmap_update_bits(nau8310->regmap, NAU8310_R0E_I2S_PCM_CTRL2,
			   NAU8310_I2S_MS_MASK, ctrl2_val);

	return 0;
}

/**
 * nau8310_set_tdm_slot - configure DAI TDM.
 * @dai: DAI
 * @tx_mask: Bitmask representing active TX slots. Ex.
 *           0xf for ADC voltage channel source selection(slots 0, 2, 4 and 6)
 *           0xf0 for ADC current channel source selection(slots 1, 3, 5 and 7)
 * @rx_mask: Bitmask representing active RX slots. Ex.
 *             0xff for DAC channel source selection(slots 0~7)
 * @slots: Number of slots in use.
 * @slot_width: Width in bits for each slot.
 *
 * Configures a DAI for TDM operation. Only support 8 slots TDM.
 */
static int nau8310_set_tdm_slot(struct snd_soc_dai *dai, unsigned int tx_mask,
				unsigned int rx_mask, int slots, int slot_width)
{
	struct snd_soc_component *component = dai->component;
	struct nau8310 *nau8310 = snd_soc_component_get_drvdata(component);
	unsigned int ctrl0_val = 0, ctrl_val = 0;

	if (slots > 8)
		return -EINVAL;

	if (tx_mask || rx_mask)
		ctrl_val |= NAU8310_TDM_EN;

	/* For ADC Voltage Channel Source Selection */
	if (tx_mask & 0xf) {
		switch (tx_mask & 0xf) {
		case 0x1:
			ctrl_val |= NAU8310_ADC_V_SEL_SLOT0;
			break;
		case 0x2:
			ctrl_val |= NAU8310_ADC_V_SEL_SLOT2;
			break;
		case 0x4:
			ctrl_val |= NAU8310_ADC_V_SEL_SLOT4;
			break;
		case 0x8:
			ctrl_val |= NAU8310_ADC_V_SEL_SLOT6;
			break;
		default:
			return -EINVAL;
		}
	}

	/* For ADC Current Channel Source Selection */
	if ((tx_mask >> 4) & 0xf) {
		switch ((tx_mask >> 4) & 0xf) {
		case 0x1:
			ctrl_val |= NAU8310_ADC_I_SEL_SLOT1;
			break;
		case 0x2:
			ctrl_val |= NAU8310_ADC_I_SEL_SLOT3;
			break;
		case 0x4:
			ctrl_val |= NAU8310_ADC_I_SEL_SLOT5;
			break;
		case 0x8:
			ctrl_val |= NAU8310_ADC_I_SEL_SLOT7;
			break;
		default:
			return -EINVAL;
		}
	}

	if (rx_mask & 0xff) {
		switch (rx_mask) {
		case 0x1:
			break;
		case 0x2:
			ctrl0_val |= NAU8310_DAC_SEL_SLOT1;
			break;
		case 0x4:
			ctrl0_val |= NAU8310_DAC_SEL_SLOT2;
			break;
		case 0x8:
			ctrl0_val |= NAU8310_DAC_SEL_SLOT3;
			break;
		case 0x10:
			ctrl0_val |= NAU8310_DAC_SEL_SLOT4;
			break;
		case 0x20:
			ctrl0_val |= NAU8310_DAC_SEL_SLOT5;
			break;
		case 0x40:
			ctrl0_val |= NAU8310_DAC_SEL_SLOT6;
			break;
		case 0x80:
			ctrl0_val |= NAU8310_DAC_SEL_SLOT7;
			break;
		default:
			return -EINVAL;
		}
	}

	dev_dbg(nau8310->dev, "%s: tx_mask: 0x%08X, rx_mask: 0x%08X, ADC slots: 0x%08X, DAC slots: 0x%08X\n",
		 __func__, tx_mask, rx_mask, ctrl_val, ctrl0_val);

	regmap_update_bits(nau8310->regmap, NAU8310_R0C_TDM_CTRL,
			   NAU8310_TDM_EN | NAU8310_TDM_OFFSET_EN |
			   NAU8310_ADC_I_SEL_MASK | NAU8310_ADC_V_SEL_MASK, ctrl_val);
	regmap_update_bits(nau8310->regmap, NAU8310_R0B_I2S_PCM_CTRL0,
			   NAU8310_DAC_SEL_MASK, ctrl0_val);

	return 0;
}

int nau8310_startup(struct snd_pcm_substream *substream, struct snd_soc_dai *dai)
{
	struct snd_soc_component *component = dai->component;
	struct nau8310 *nau8310 = snd_soc_component_get_drvdata(component);

	if (nau8310->dsp_enable)
		snd_soc_dapm_enable_pin(nau8310->dapm, "Sense");

	return 0;
}

static void nau8310_shutdown(struct snd_pcm_substream *substream,
			     struct snd_soc_dai *dai)
{
	struct snd_soc_component *component = dai->component;
	struct nau8310 *nau8310 = snd_soc_component_get_drvdata(component);

	if (nau8310->dsp_enable) {
		int ret;

		snd_soc_dapm_disable_pin(nau8310->dapm, "Sense");

		/* For internal Ring OSC, the default fs apply to 48kHz */
		nau8310->fs = 48000;
		ret = nau8310_set_sysclk(component, 0, 0,
			nau8310->fs * 256, SND_SOC_CLOCK_IN);
		if (ret)
			goto err;
		ret = nau8310_clock_config(nau8310);
		if (ret)
			goto err;

		/* Switch clock source to OSC before MCLK off if DSP is enabled. */
		regmap_update_bits(nau8310->regmap, NAU8310_R04_ENA_CTRL,
			NAU8310_DSP_SEL_OSC, NAU8310_DSP_SEL_OSC);
	}

err:
	return;
}

static int nau8310_set_sysclk(struct snd_soc_component *component,
			      int clk_id, int source, unsigned int freq, int dir)
{
	struct nau8310 *nau8310 = snd_soc_component_get_drvdata(component);

	if (freq < MASTER_CLK_MIN || freq > MASTER_CLK_MAX) {
		dev_err(nau8310->dev, "Exceed the range of input clocks, MCLK %dHz\n",
			freq);
		return -EINVAL;
	}
	nau8310->mclk = freq;
	dev_dbg(nau8310->dev, "%s, MCLK %dHz\n", __func__, nau8310->mclk);

	return 0;
}

static int nau8310_codec_probe(struct snd_soc_component *component)
{
	struct nau8310 *nau8310 = snd_soc_component_get_drvdata(component);
	struct snd_soc_dapm_context *dapm =
			snd_soc_component_get_dapm(component);

	nau8310->dapm = dapm;

	/* For internal Ring OSC, the default fs apply to 48kHz */
	nau8310->fs = 48000;
	ret = nau8310_set_sysclk(component, 0, 0,
				 nau8310->fs * 256, SND_SOC_CLOCK_IN);
	if (ret)
		goto err;
	ret = nau8310_clock_config(nau8310);
	if (ret)
		goto err;
	regmap_update_bits(nau8310->regmap, NAU8310_R68_ANALOG_CONTROL_7,
			   NAU8310_MU_HALF_RANGE_EN, NAU8310_MU_HALF_RANGE_EN);
	regmap_update_bits(nau8310->regmap, NAU8310_R04_ENA_CTRL,
			   NAU8310_DSP_OSC_EN | NAU8310_DSP_SEL_OSC,
			   NAU8310_DSP_OSC_EN | NAU8310_DSP_SEL_OSC);
	nau8310_software_reset(nau8310->regmap);
	/* wait for the power ready */
	msleep(120);
	regmap_update_bits(nau8310->regmap, NAU8310_R1A_DSP_CORE_CTRL2,
			   NAU8310_DSP_RUNSTALL | NAU8310_DAC_SEL_DSP_OUT,
			   NAU8310_DAC_SEL_DSP_OUT);
	/* Loading DSP firmware */
	ret = nau8310_enable_dsp(component);
	if(ret)
		dev_warn("Can't enable DSP, so enable bypass mode\n");

	return 0;
}

int nau8310_enable_dsp(struct snd_soc_component *component)
{
	struct nau8310 *nau8310 = snd_soc_component_get_drvdata(component);
	int ret;

	snd_soc_dapm_disable_pin(nau8310->dapm, "Sense");

	ret = nau8310_dsp_init(component);
	if (ret)
		goto err;
	nau8310->dsp_enable = true;

	return 0;

err:
	regmap_update_bits(nau8310->regmap,
			   NAU8310_R1A_DSP_CORE_CTRL2,
			   NAU8310_DSP_RUNSTALL | NAU8310_DAC_SEL_DSP_OUT,
			   NAU8310_DSP_RUNSTALL);
	return ret;
}
EXPORT_SYMBOL_GPL(nau8310_enable_dsp);

static int __maybe_unused nau8310_suspend(struct snd_soc_component *component)
{
	struct nau8310 *nau8310 = snd_soc_component_get_drvdata(component);

	if (nau8310->dsp_enable)
		regmap_update_bits(nau8310->regmap, NAU8310_R04_ENA_CTRL,
				   NAU8310_DSP_SEL_OSC, NAU8310_DSP_SEL_OSC);

	regcache_cache_only(nau8310->regmap, true);
	regcache_mark_dirty(nau8310->regmap);

	return 0;
}

static int __maybe_unused nau8310_resume(struct snd_soc_component *component)
{
	struct nau8310 *nau8310 = snd_soc_component_get_drvdata(component);
	int ret, value;

	regcache_cache_only(nau8310->regmap, false);
	regcache_sync(nau8310->regmap);

	ret = regmap_read(nau8310->regmap, NAU8310_R1A_DSP_CORE_CTRL2, &value);
	if (ret) {
		dev_err(nau8310->dev, "Failed to read register of DSP core control: %d\n",
			ret);
		goto err;
	}
	if (nau8310->dsp_enable && (value & NAU8310_DAC_SEL_DSP_OUT)) {
		/* For internal Ring OSC, the default fs apply to 48kHz */
		nau8310->fs = 48000;
		ret = nau8310_set_sysclk(component, 0, 0,
					 nau8310->fs * 256, SND_SOC_CLOCK_IN);
		if (ret)
			goto err;
		ret = nau8310_clock_config(nau8310);
		if (ret)
			goto err;
		regmap_update_bits(nau8310->regmap, NAU8310_R04_ENA_CTRL,
				   NAU8310_DSP_OSC_EN | NAU8310_DSP_SEL_OSC,
				   NAU8310_DSP_OSC_EN | NAU8310_DSP_SEL_OSC);
		nau8310_software_reset(nau8310->regmap);
		/* wait for the power ready */
		msleep(120);
		regmap_update_bits(nau8310->regmap, NAU8310_R1A_DSP_CORE_CTRL2,
				   NAU8310_DSP_RUNSTALL, 0);
		ret = nau8310_dsp_resume(component);
		if (ret) {
			dev_err(nau8310->dev, "Failed to resume DSP: %d\n", ret);
			goto err;
		}
	}

	return 0;

err:
	regmap_update_bits(nau8310->regmap, NAU8310_R1A_DSP_CORE_CTRL2,
			   NAU8310_DSP_RUNSTALL | NAU8310_DAC_SEL_DSP_OUT,
			   NAU8310_DSP_RUNSTALL);
	return ret;
}

static const struct snd_soc_component_driver soc_component_dev_nau8310 = {
	.probe			= nau8310_codec_probe,
	.set_sysclk		= nau8310_set_sysclk,
	.suspend		= nau8310_suspend,
	.resume			= nau8310_resume,
	.controls		= nau8310_snd_controls,
	.num_controls		= ARRAY_SIZE(nau8310_snd_controls),
	.dapm_widgets		= nau8310_dapm_widgets,
	.num_dapm_widgets	= ARRAY_SIZE(nau8310_dapm_widgets),
	.dapm_routes		= nau8310_dapm_routes,
	.num_dapm_routes	= ARRAY_SIZE(nau8310_dapm_routes),
	.suspend_bias_off	= 1,
	.idle_bias_on		= 1,
	.use_pmdown_time	= 1,
	.endianness		= 1,
	.non_legacy_dai_naming	= 1,
};

static const struct snd_soc_dai_ops nau8310_dai_ops = {
	.startup	= nau8310_startup,
	.hw_params	= nau8310_hw_params,
	.trigger	= nau8310_trigger,
	.shutdown	= nau8310_shutdown,
	.set_fmt	= nau8310_set_fmt,
	.set_tdm_slot	= nau8310_set_tdm_slot,
};

#define NAU8310_RATES SNDRV_PCM_RATE_8000_192000
#define NAU8310_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE \
	 | SNDRV_PCM_FMTBIT_S24_3LE | SNDRV_PCM_FMTBIT_S32_LE)

static struct snd_soc_dai_driver nau8310_dai = {
	.name = NAU8310_CODEC_DAI,
	.playback = {
		.stream_name = "Playback",
		.channels_min = 1,
		.channels_max = 1,
		.rates = NAU8310_RATES,
		.formats = NAU8310_FORMATS,
	},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = 2,
		.rates = NAU8310_RATES,
		.formats = NAU8310_FORMATS,
	},
	.ops = &nau8310_dai_ops,
};

static int nau8310_reg_write(void *context, unsigned int reg, unsigned int value)
{
	struct i2c_client *client = context;
	u8 buf[6];
	int ret, count = 0;

	buf[count++] = reg >> 8;
	buf[count++] = reg;
	if (reg != NAU8310_RF000_DSP_COMM) {
		/* format for G10, 2 bytes value and endian big */
		buf[count++] = value >> 8;
		buf[count++] = value;
		dev_dbg(&client->dev, "Reg:0x%x Value:0x%x\n", reg, value);
	} else {
		/* format for DSP, 4 bytes value and native */
		*(u32 *)&buf[count] = value;
		count += sizeof(u32);
	}
	ret = i2c_master_send(client, buf, count);
	if (ret == count)
		return 0;
	else if (ret < 0)
		return ret;
	else
		return -EIO;
}

static int nau8310_reg_read(void *context, unsigned int reg, unsigned int *value)
{
	struct i2c_client *client = context;
	struct i2c_msg xfer[2];
	u8 buf[4];
	const u8 *b = buf;
	u16 reg_buf;
	int ret;

	reg_buf = cpu_to_be16(reg);
	xfer[0].addr = client->addr;
	xfer[0].len = sizeof(reg_buf);
	xfer[0].buf = (u8 *)&reg_buf;
	xfer[0].flags = 0;

	xfer[1].addr = client->addr;
	xfer[1].len = (reg != NAU8310_RF000_DSP_COMM ? 2 : 4);
	xfer[1].buf = buf;
	xfer[1].flags = I2C_M_RD;

	ret = i2c_transfer(client->adapter, xfer, ARRAY_SIZE(xfer));
	if (ret < 0)
		return ret;
	else if (ret != ARRAY_SIZE(xfer))
		return -EIO;

	if (reg != NAU8310_RF000_DSP_COMM) {
		/* parse for G10, 2 bytes value and endian big */
		*value = b[1];
		*value |= ((unsigned int)b[0]) << 8;
	} else {
		/* parse for DSP, 4 bytes value and native */
		*value = *(u32 *)b;
	}
	return 0;
}

static const struct regmap_config nau8310_regmap_config = {
	.reg_bits = NAU8310_REG_ADDR_LEN,
	.val_bits = NAU8310_REG_DATA_LEN,

	.max_register = NAU8310_REG_MAX,
	.readable_reg = nau8310_readable_reg,
	.writeable_reg = nau8310_writeable_reg,
	.volatile_reg = nau8310_volatile_reg,
	.reg_read = nau8310_reg_read,
	.reg_write = nau8310_reg_write,

	.cache_type = REGCACHE_RBTREE,
	.reg_defaults = nau8310_reg_defaults,
	.num_reg_defaults = ARRAY_SIZE(nau8310_reg_defaults),
};

static void nau8310_reset_chip(struct regmap *regmap)
{
	regmap_write(regmap, NAU8310_R00_HARDWARE_RST, 0x0001);
	regmap_write(regmap, NAU8310_R00_HARDWARE_RST, 0x0000);
}

static void nau8310_software_reset(struct regmap *regmap)
{
	regmap_write(regmap, NAU8310_R01_SOFTWARE_RST, 0x0000);
	regmap_write(regmap, NAU8310_R01_SOFTWARE_RST, 0x0000);
}

static void nau8310_init_regs(struct nau8310 *nau8310)
{
	struct regmap *regmap = nau8310->regmap;

	/* Latch IIC LSB value */
	regmap_write(regmap, NAU8310_R02_I2C_ADDR, 0x0001);
	/* Block MCLK */
	regmap_update_bits(regmap, NAU8310_R04_ENA_CTRL,
			   NAU8310_MCLK_SEL_MASK, 0x7 << NAU8310_MCLK_SEL_SFT);
	/* stall DSP */
	regmap_update_bits(regmap, NAU8310_R1A_DSP_CORE_CTRL2,
			   NAU8310_DSP_RUNSTALL, NAU8310_DSP_RUNSTALL);
	/* Default ADCI from slot1, ADCV from slot 0. */
	regmap_update_bits(regmap, NAU8310_R0C_TDM_CTRL,
			   NAU8310_ADC_I_SEL_MASK | NAU8310_ADC_V_SEL_MASK,
			   (0x4 << NAU8310_ADC_I_SEL_SFT) | 0x4);
	/* Set I-sense digital gain to +10.25dB */
	regmap_update_bits(regmap, NAU8310_R14_ADC_VOL_CTRL,
			   NAU8310_ADC_GAIN_L_MASK | NAU8310_ADC_GAIN_R_MASK,
			   (0x1 << NAU8310_ADC_GAIN_L_SFT) | 0x62);
	/* PGA in Class A Mode */
	regmap_update_bits(regmap, NAU8310_R76_BOOST,
			   NAU8310_STG2_SEL_CLASSA, NAU8310_STG2_SEL_CLASSA);
	/* Set I-sense PGA Bias Current to minimum,
	 * Disable PGA Common Mode Lock.
	 */
	regmap_update_bits(regmap, NAU8310_R77_FEPGA,
			   NAU8310_CURR_TRIM_MASK | NAU8310_CMLCK_ENB,
			   (0x7 << NAU8310_CURR_TRIM_SFT) | NAU8310_CMLCK_ENB);
	/* Set I-sense PGA to +12dB gain */
	regmap_update_bits(regmap, NAU8310_R7F_POWER_UP_CONTROL,
			   NAU8310_PGA_GAIN_MASK, 0x3 << NAU8310_PGA_GAIN_SFT);
	/* Normal I2S Audio data w/o SAR ADC data.
	 * Default oversampling/decimations settings are unusable
	 * (audible hiss). Set it to something better.
	 */
	regmap_update_bits(regmap, NAU8310_R28_ADC_RATE,
			   NAU8310_I2S_MODE | NAU8310_ADC_SYNC_DOWN_MASK,
			   (nau8310->normal_iis_data << NAU8310_I2S_MODE_SFT) |
			   NAU8310_ADC_SYNC_DOWN_64);
	regmap_update_bits(regmap, NAU8310_R29_DAC_CTRL1,
			   NAU8310_DAC_OVERSAMPLE_MASK, NAU8310_DAC_OVERSAMPLE_64);
	/* Enable auto attenuation for better output signal */
	regmap_update_bits(regmap, NAU8310_R2E_ALC_CTRL3,
			   NAU8310_AUTOATT_EN, NAU8310_AUTOATT_EN);
	/* Set ALC parameters */
	regmap_update_bits(regmap, NAU8310_R2C_ALC_CTRL1,
			   NAU8310_ALC_MAXGAIN_MASK,
			   0x7 << NAU8310_ALC_MAXGAIN_SFT);
	regmap_update_bits(regmap, NAU8310_R2D_ALC_CTRL2,
			   NAU8310_ALC_DCY_MASK | NAU8310_ALC_ATK_MASK |
			   NAU8310_ALC_HLD_MASK, (0x5 << NAU8310_ALC_DCY_SFT) |
			   (0x3 << NAU8310_ALC_ATK_SFT) |
			   (0x5 << NAU8310_ALC_HLD_SFT));
	regmap_update_bits(regmap, NAU8310_R2E_ALC_CTRL3,
			   NAU8310_LIM_MDE_MASK | NAU8310_VBAT_THLD_MASK,
			   (0x3 << NAU8310_LIM_MDE_SFT) |
			   (0x18 << NAU8310_VBAT_THLD_SFT));

	/* Enable ALC to avoid signal distortion when battery low. */
	if (nau8310->alc_enable)
		regmap_update_bits(regmap, NAU8310_R2E_ALC_CTRL3,
				   NAU8310_ALC_EN, NAU8310_ALC_EN);
	/* Enable AEC for plarform used */
	if (nau8310->aec_enable)
		regmap_update_bits(regmap, NAU8310_R0B_I2S_PCM_CTRL0,
				   NAU8310_AEC_MODE_AEC, NAU8310_AEC_MODE_AEC);

	if (nau8310->clock_detection)
		regmap_update_bits(regmap, NAU8310_R40_CLK_DET_CTRL,
				   NAU8310_CLKPWRUP_DIS | NAU8310_PWRUP_DFT, 0);
	else
		regmap_update_bits(regmap, NAU8310_R40_CLK_DET_CTRL,
				   NAU8310_CLKPWRUP_DIS | NAU8310_PWRUP_DFT,
				   NAU8310_CLKPWRUP_DIS);
	if (nau8310->clock_det_data)
		regmap_update_bits(regmap, NAU8310_R40_CLK_DET_CTRL,
				   NAU8310_APWRUP_EN, NAU8310_APWRUP_EN);
	else
		regmap_update_bits(regmap, NAU8310_R40_CLK_DET_CTRL,
				   NAU8310_APWRUP_EN, 0);
	/* Boost Target Limit and Margin */
	regmap_update_bits(regmap, NAU8310_R17_BOOST_CTRL1,
			   NAU8310_BSTLIMIT_MASK | NAU8310_BSTMARGIN_MASK,
			   (nau8310->boost_target_limit << NAU8310_BSTLIMIT_SFT) |
			   nau8310->boost_target_margin);
	/* Temperature compensation */
	regmap_update_bits(regmap, NAU8310_R30_TEMP_COMP_CTRL,
			   NAU8310_TEMP_COMP_ACT2_MASK, 0);
	/* Low pass filter control */
	regmap_update_bits(regmap, NAU8310_R33_LPF_CTRL,
			   NAU8310_LPF_IN1_EN | NAU8310_LPF_IN1_TC_MASK |
			   NAU8310_LPF_IN2_EN | NAU8310_LPF_IN2_TC_MASK,
			   NAU8310_LPF_IN1_EN | (0x7 << NAU8310_LPF_IN1_TC_SFT) |
			   NAU8310_LPF_IN2_EN | (0x7 << NAU8310_LPF_IN2_TC_SFT));
	/* Set Boost Attack/Decay, Hold Time and TC EN */
	regmap_update_bits(regmap, NAU8310_R18_BOOST_CTRL2, NAU8310_TC_EN |
			   NAU8310_BSTHOLD_MASK | NAU8310_BSTSTEPTIME_MASK |
			   NAU8310_BSTDELAY_MASK,
			   (nau8310->temp_compensation << NAU8310_TC_SFT) |
			   (0xf << NAU8310_BSTHOLD_SFT) |
			   (0x7 << NAU8310_BSTSTEPTIME_SFT) | nau8310->boost_delay);
	/* DAC Reference Voltage Setting */
	regmap_update_bits(regmap, NAU8310_R73_RDAC,
			   NAU8310_DACVREFSEL_MASK,
			   nau8310->dac_vref << NAU8310_DACVREFSEL_SFT);
	/* DAC Reference Voltage Decoupling Capacitors. */
	regmap_update_bits(regmap, NAU8310_R63_ANALOG_CONTROL_3,
			   NAU8310_DACREFCAP_MASK, 0x3 << NAU8310_DACREFCAP_SFT);
	/* Auto-Att Min Gain 12dB, Class-D P&N Driver Slew Rate +75%. */
	regmap_update_bits(regmap, NAU8310_R64_ANALOG_CONTROL_4,
			   NAU8310_AUTOATTMIN_MASK | NAU8310_CLASSD_SLEWN_MASK |
			   NAU8310_CLASSD_SLEWP_MASK | NAU8310_CLASSD_SHORTP_MASK |
			   NAU8310_CLASSD_SHORTN_MASK,
			   NAU8310_CLASSD_SHORTP_100UP | NAU8310_AUTOATTMIN_12DB |
			   NAU8310_CLASSD_SLEWP_75UP | NAU8310_CLASSD_SLEWN_75UP);
	/* Bias Current, Monitor Vboost drops below VBAT,
	 * Non-Overlapping-Time Control.
	 */
	regmap_update_bits(regmap, NAU8310_R65_ANALOG_CONTROL_5,
			   NAU8310_BSTINDR_MASK | NAU8310_BSTCURGEN_MASK |
			   NAU8310_BSTMONEN | NAU8310_BSTCLKPULSE_MASK,
			   (0x4 << NAU8310_BSTINDR_SFT) | NAU8310_BSTMONEN |
			   (0xd << NAU8310_BSTCURGEN_SFT) | 0x2);
	/* Slew Rate Adjust, Bias Current. */
	regmap_update_bits(regmap, NAU8310_R66_ANALOG_CONTROL_6,
			   NAU8310_BSTSLEWPOFF_MASK | NAU8310_BSTSLEWPON_MASK |
			   NAU8310_BSTSLEWNON_MASK | NAU8310_BSTSLEWNOFF_MASK |
			   NAU8310_BSTIPDR_MASK, (0x3 << NAU8310_BSTSLEWPOFF_SFT) |
			   (0x2 << NAU8310_BSTSLEWPON_SFT) |
			   (0x2 << NAU8310_BSTSLEWNON_SFT) |
			   (0x1 << NAU8310_BSTSLEWNOFF_SFT) | 0x4);
	if (nau8310->silicon_id == NAU8310_REG_SI_REV_G10) {
		/* Peak Current 4A, Low Voltage 2.8V */
		regmap_update_bits(regmap, NAU8310_R6B_ANALOG_CONTROL_8,
				   NAU8310_VBAT_PCL_MASK | NAU8310_VBAT_THD_MASK,
				   (0x11 << NAU8310_VBAT_PCL_SFT) |
				   (0xf << NAU8310_VBAT_THD_SFT));
	} else if (nau8310->silicon_id == NAU8310_REG_SI_REV_G20) {
		/* Peak Current 1A, Low Voltage 3.6V */
		regmap_update_bits(regmap, NAU8310_R6B_ANALOG_CONTROL_8,
				   NAU8310_VBAT_PCL_MASK | NAU8310_VBAT_THD_MASK,
				   (0x7 << NAU8310_VBAT_THD_SFT));
	}
	/* current limiting for low VBAT level */
	regmap_update_bits(regmap, NAU8310_R6C_ANALOG_CONTROL_9,
			   NAU8310_VBAT_CURLMT_MASK, 0xf);
	/* VMID Tieoff (VMID Resistor Selection) */
	regmap_update_bits(regmap, NAU8310_R60_BIAS_ADJ,
			   NAU8310_BIAS_VMID_SEL_MASK,
			   nau8310->vref_impedance << NAU8310_BIAS_VMID_SEL_SFT);
	/* VREF Bandgap buffer ON, I/V Sense Ref Buffer Setting -1.5dB. */
	regmap_update_bits(regmap, NAU8310_R68_ANALOG_CONTROL_7,
			   NAU8310_ADCGAIN_MASK | NAU8310_VREFBG_EN,
			   (0x3 << NAU8310_ADCGAIN_SFT) | NAU8310_VREFBG_EN);
	nau8310_software_reset(regmap);
	/* Enable VMID, BIAS, DAC, DCA CLOCK, ADC, Voltage/Current Amps,
	 * ADC Resetb gated by 'PowerUp' Signal.
	 */
	regmap_update_bits(regmap, NAU8310_R61_ANALOG_CONTROL_1,
			   NAU8310_ISEN_MASK | NAU8310_ADCRSTEN_MASK |
			   NAU8310_VSEN_MASK | NAU8310_ADCEN_MASK |
			   NAU8310_DACCLKEN_MASK | NAU8310_DACEN_MASK |
			   NAU8310_BIASEN_MASK | NAU8310_VMIDEN_MASK,
			   (0x1 << NAU8310_ISEN_SFT) | (0x1 << NAU8310_ADCRSTEN_SFT) |
			   (0x1 << NAU8310_VSEN_SFT) | (0x1 << NAU8310_ADCEN_SFT) |
			   (0x1 << NAU8310_DACCLKEN_SFT) | (0x1 << NAU8310_DACEN_SFT) |
			   (0x1 << NAU8310_BIASEN_SFT) | 0x1);
	/* Enable Boost Converter, VMID Fast, Class-D gated by
	 * 'PowerUp' Signal
	 */
	regmap_update_bits(regmap, NAU8310_R62_ANALOG_CONTROL_2,
			   NAU8310_CLASSDEN_MASK | NAU8310_PDVMDFST_MASK |
			   NAU8310_BSTEN_MASK, (0x1 << NAU8310_CLASSDEN_SFT) |
			   (0x1 << NAU8310_PDVMDFST_SFT) | nau8310->boost_convert_enable);
	regmap_update_bits(regmap, NAU8310_R07_SAR_CTRL1,
			   NAU8310_SAR_TRACKING_GAIN_MASK | NAU8310_SAR_COMPARE_TIME_MASK |
			   NAU8310_SAR_SAMPLING_TIME_MASK,
			   nau8310->sar_voltage << NAU8310_SAR_TRACKING_GAIN_SFT |
			   nau8310->sar_compare_time << NAU8310_SAR_COMPARE_TIME_SFT |
			   nau8310->sar_sampling_time << NAU8310_SAR_SAMPLING_TIME_SFT);
}

static void nau8310_print_device_properties(struct nau8310 *nau8310)
{
	struct device *dev = nau8310->dev;

	dev_dbg(dev, "vref-impedance:          %d\n", nau8310->vref_impedance);
	dev_dbg(dev, "dac-vref:                %d\n", nau8310->dac_vref);
	dev_dbg(dev, "sar-voltage:             %d\n", nau8310->sar_voltage);
	dev_dbg(dev, "sar-compare-time:        %d\n", nau8310->sar_compare_time);
	dev_dbg(dev, "sar-sampling-time:       %d\n", nau8310->sar_sampling_time);
	dev_dbg(dev, "clock-det-disable:       %d\n", nau8310->clock_detection);
	dev_dbg(dev, "clock-det-data:          %d\n", nau8310->clock_det_data);
	dev_dbg(dev, "temp-compensation:       %d\n", nau8310->temp_compensation);
	dev_dbg(dev, "boost-delay:             %d\n", nau8310->boost_delay);
	dev_dbg(dev, "boost-convert-enable:    %d\n", nau8310->boost_convert_enable);
	dev_dbg(dev, "boost-target-limit:      %x\n", nau8310->boost_target_limit);
	dev_dbg(dev, "boost-target-margin:     %x\n", nau8310->boost_target_margin);
	dev_dbg(dev, "normal-iis-data:         %d\n", nau8310->normal_iis_data);
	dev_dbg(dev, "alc-enable:              %d\n", nau8310->alc_enable);
	dev_dbg(dev, "aec-enable:              %d\n", nau8310->aec_enable);
}

static int nau8310_read_device_properties(struct device *dev,
					  struct nau8310 *nau8310)
{
	int ret;

	ret = device_property_read_u32(dev, "nuvoton,vref-impedance",
				       &nau8310->vref_impedance);
	if (ret)
		nau8310->vref_impedance = 2;
	ret = device_property_read_u32(dev, "nuvoton,dac-vref",
				       &nau8310->dac_vref);
	if (ret)
		nau8310->dac_vref = 1;
	ret = device_property_read_u32(dev, "nuvoton,sar-voltage",
				       &nau8310->sar_voltage);
	if (ret)
		nau8310->sar_voltage = 0;
	ret = device_property_read_u32(dev, "nuvoton,sar-compare-time",
				       &nau8310->sar_compare_time);
	if (ret)
		nau8310->sar_compare_time = 1;
	ret = device_property_read_u32(dev, "nuvoton,sar-sampling-time",
				       &nau8310->sar_sampling_time);
	if (ret)
		nau8310->sar_sampling_time = 1;
	nau8310->clock_detection =
		!device_property_read_bool(dev,	"nuvoton,clock-det-disable");
	nau8310->clock_det_data =
		device_property_read_bool(dev, "nuvoton,clock-det-data");
	nau8310->temp_compensation =
		device_property_read_bool(dev, "nuvoton,temp-compensation");
	if (nau8310->silicon_id == NAU8310_REG_SI_REV_G10) {
		ret = device_property_read_u32(dev, "nuvoton,boost-delay",
					       &nau8310->boost_delay);
		if (ret)
			nau8310->boost_delay = 0x8;
		ret = device_property_read_u32(dev, "nuvoton,boost-convert-enable",
					       &nau8310->boost_convert_enable);
		if (ret)
			nau8310->boost_convert_enable = 0x0;
		ret = device_property_read_u32(dev, "nuvoton,boost-target-limit",
					       &nau8310->boost_target_limit);
		if (ret)
			nau8310->boost_target_limit = 0x32;
		ret = device_property_read_u32(dev, "nuvoton,boost-target-margin",
					       &nau8310->boost_target_margin);
		if (ret)
			nau8310->boost_target_margin = 0x2;
	}
	nau8310->normal_iis_data =
		device_property_read_bool(dev, "nuvoton,normal-iis-data");
	nau8310->alc_enable =
		device_property_read_bool(dev, "nuvoton,alc-enable");
	nau8310->aec_enable =
		device_property_read_bool(dev, "nuvoton,aec-enable");

	return 0;
}

static int nau8310_i2c_probe(struct i2c_client *i2c,
			     const struct i2c_device_id *id)
{
	struct device *dev = &i2c->dev;
	struct nau8310 *nau8310 = dev_get_platdata(dev);
	int ret, value;

	if (!nau8310) {
		nau8310 = devm_kzalloc(dev, sizeof(*nau8310), GFP_KERNEL);
		if (!nau8310)
			return -ENOMEM;
	}
	i2c_set_clientdata(i2c, nau8310);

	nau8310->regmap = devm_regmap_init(dev, NULL, i2c,
					   &nau8310_regmap_config);
	if (IS_ERR(nau8310->regmap))
		return PTR_ERR(nau8310->regmap);

	nau8310->dev = dev;
	nau8310->dsp_enable = false;

	nau8310_reset_chip(nau8310->regmap);
	ret = regmap_read(nau8310->regmap, NAU8310_R46_I2C_DEVICE_ID, &value);
	if (ret) {
		dev_err(dev, "Failed to read device id from the NAU8310: %d\n",
			ret);
		return ret;
	}
	nau8310->silicon_id = value & NAU8310_REG_SI_REV_MASK;
	ret = nau8310_read_device_properties(dev, nau8310);
	if (ret)
		return ret;
	nau8310_print_device_properties(nau8310);
	nau8310_init_regs(nau8310);

	return snd_soc_register_component(dev, &soc_component_dev_nau8310,
					  &nau8310_dai, 1);
}


static int nau8310_i2c_remove(struct i2c_client *client)
{
	snd_soc_unregister_component(&client->dev);
	return 0;
}

static const struct i2c_device_id nau8310_i2c_ids[] = {
	{ "nau8310", 0 },
	{ "nau8320", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, nau8310_i2c_ids);

#ifdef CONFIG_OF
static const struct of_device_id nau8310_of_ids[] = {
	{ .compatible = "nuvoton,nau8310", },
	{ .compatible = "nuvoton,nau8320", },
	{}
};
MODULE_DEVICE_TABLE(of, nau8310_of_ids);
#endif

#ifdef CONFIG_ACPI
static const struct acpi_device_id nau8310_acpi_match[] = {
	{"NVTN2000", 0,},
	{"NVTN2001", 0,},
	{},
};
MODULE_DEVICE_TABLE(acpi, nau8310_acpi_match);
#endif

static struct i2c_driver nau8310_i2c_driver = {
	.driver = {
		.name = "nau8310",
		.of_match_table = of_match_ptr(nau8310_of_ids),
		.acpi_match_table = ACPI_PTR(nau8310_acpi_match),
	},
	.probe = nau8310_i2c_probe,
	.remove = nau8310_i2c_remove,
	.id_table = nau8310_i2c_ids,
};
module_i2c_driver(nau8310_i2c_driver);

MODULE_DESCRIPTION("ASoC NAU83G10/20 Boosted Mono Class-D Amplifier with DSP and I/V-sense driver");
MODULE_AUTHOR("John Hsu <kchsu0@nuvoton.com>");
MODULE_AUTHOR("David Lin <ctlin0@nuvoton.com>");
MODULE_LICENSE("GPL");

/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * The NAU83110 ALSA SoC audio driver. The codec is a mono high efficiency
 * filter-free Class-D audio amplifier, which is capable of driving a 4Ω load
 * with up to 20.8W output power at 12.6V supply.
 *
 * Copyright (C) 2025 Nuvoton Technology Corp.
 * Author: Neo Chang <ylchang2@nuvoton.com>
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/clk.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <sound/tlv.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#include "nau83110.h"

/* Range of Master Clock MCLK (Hz) */
#define MASTER_CLK_MAX 38400000
#define MASTER_CLK_MIN 256000

static const struct reg_default nau83110_reg_defaults[] = {
	{ NAU83110_R00_SW_RST_EN, 0x0000 },
	{ NAU83110_R01_AUDIO_RST_EN, 0x0000 },
	{ NAU83110_R02_I2C_ADDRESS_LATCH, 0x0000 },
	{ NAU83110_R16_INTR_CTRL0, 0x8800 },
	{ NAU83110_R17_INTR_CTRL1, 0xffff },
	{ NAU83110_R18_INTR_CTRL2, 0x0000 },
	{ NAU83110_R58_DAC_VOLUME_CTRL, 0x00f3},
	{ NAU83110_R59_ALC_MODE_CTRL, 0x4000},
	{ NAU83110_R5A_ALCDCY_CLIP, 0x03e8},
	{ NAU83110_R5B_ALCATK_CLIP, 0x0010},
	{ NAU83110_R5C_ALCHLD_CLIP, 0x0000},
	{ NAU83110_R5D_ALC_MINGAIN_CLIP, 0x0000},
	{ NAU83110_R5F_ALCATK_OTW2, 0x1388},
	{ NAU83110_R60_ALCHLD_OTW2, 0x01f4},
	{ NAU83110_R61_ALC_MINGAIN_OTW2, 0x0000},
	{ NAU83110_R63_ALCATK_OTW1, 0x1388},
	{ NAU83110_R64_ALCHLD_OTW1, 0x01f4},
	{ NAU83110_R65_ALC_MINGAIN_OTW1, 0x0000},
	{ NAU83110_R67_ALCATK_UVLOP, 0x0000},
	{ NAU83110_R68_ALCHLD_UVLOP, 0x0064},
	{ NAU83110_R69_ALC_MINGAIN_STEP_UVLOP, 0x0000},
	{ NAU83110_R6D_DIAG_CTRL0,  0x0774},
	{ NAU83110_R6E_DIAG_CTRL1, 0x2929},
	{ NAU83110_R6F_DIAG_CTRL2, 0x0000},
	{ NAU83110_R74_CLIP_CTRL, 0x0000},
	{ NAU83110_R76_DAC_CTRL1, 0x0000},
	{ NAU83110_R77_HPF_CTRL, 0x0000},
	{ NAU83110_R78_I2S_PCM_CTRL2, 0x0000},
	{ NAU83110_R7A_MUTE_CTRL, 0x0070},
	{ NAU83110_R7B_MUTE_RAMP_SPD, 0x000f},
	{ NAU83110_R7C_TDM_RX_CTRL0, 0x0002},
	{ NAU83110_R7D_TDM_RX_CTRL1, 0x0000},
	{ NAU83110_R80_TDM_TX_CTRL0, 0x0000},
	{ NAU83110_R81_TDM_TX_CTRL1, 0x0000},
	{ NAU83110_RA8_STANDBY_CTRL, 0x0000},
	{ NAU83110_RB6_ANALOG_CTRL0, 0x0000},
	{ NAU83110_RB7_ANALOG_CTRL1, 0x0000},
	{ NAU83110_RBB_PWRSTAGE_CTRL0, 0x810B},
	{ NAU83110_RBC_PWRSTAGE_CTRL1, 0x0008},
	{ NAU83110_RBD_PWRSTAGE_CTRL2, 0xF001},
};

static bool nau83110_readable_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case NAU83110_R00_SW_RST_EN ... NAU83110_R02_I2C_ADDRESS_LATCH:
	case NAU83110_R16_INTR_CTRL0 ... NAU83110_R19_INTR_CTRL3:
	case NAU83110_R1B_INTR_CTRL4:
	case NAU83110_R58_DAC_VOLUME_CTRL ... NAU83110_R5D_ALC_MINGAIN_CLIP:
	case NAU83110_R5F_ALCATK_OTW2 ... NAU83110_R61_ALC_MINGAIN_OTW2:
	case NAU83110_R63_ALCATK_OTW1 ... NAU83110_R65_ALC_MINGAIN_OTW1:
	case NAU83110_R67_ALCATK_UVLOP ... NAU83110_R69_ALC_MINGAIN_STEP_UVLOP:
	case NAU83110_R6D_DIAG_CTRL0 ... NAU83110_R6F_DIAG_CTRL2:
	case NAU83110_R74_CLIP_CTRL:
	case NAU83110_R76_DAC_CTRL1 ... NAU83110_R78_I2S_PCM_CTRL2:
	case NAU83110_R7A_MUTE_CTRL ... NAU83110_R7D_TDM_RX_CTRL1:
	case NAU83110_R80_TDM_TX_CTRL0 ... NAU83110_R81_TDM_TX_CTRL1:
	case NAU83110_RA8_STANDBY_CTRL:
	case NAU83110_RB6_ANALOG_CTRL0 ... NAU83110_RB7_ANALOG_CTRL1:
	case NAU83110_RB9_ANALOG_READ0 ... NAU83110_RBD_PWRSTAGE_CTRL2:
	case NAU83110_RF0_DEV_INFO0 ... NAU83110_RF1_DEV_INFO1:
		return true;
	default:
		return false;
	}
}

static bool nau83110_writeable_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case NAU83110_R00_SW_RST_EN ... NAU83110_R02_I2C_ADDRESS_LATCH:
	case NAU83110_R16_INTR_CTRL0 ... NAU83110_R19_INTR_CTRL3:
	case NAU83110_R58_DAC_VOLUME_CTRL ... NAU83110_R5D_ALC_MINGAIN_CLIP:
	case NAU83110_R5F_ALCATK_OTW2 ... NAU83110_R61_ALC_MINGAIN_OTW2:
	case NAU83110_R63_ALCATK_OTW1 ... NAU83110_R65_ALC_MINGAIN_OTW1:
	case NAU83110_R67_ALCATK_UVLOP ... NAU83110_R69_ALC_MINGAIN_STEP_UVLOP:
	case NAU83110_R6D_DIAG_CTRL0 ... NAU83110_R6F_DIAG_CTRL2:
	case NAU83110_R74_CLIP_CTRL:
	case NAU83110_R76_DAC_CTRL1 ... NAU83110_R78_I2S_PCM_CTRL2:
	case NAU83110_R7A_MUTE_CTRL ... NAU83110_R7D_TDM_RX_CTRL1:
	case NAU83110_R80_TDM_TX_CTRL0 ... NAU83110_R81_TDM_TX_CTRL1:
	case NAU83110_RA8_STANDBY_CTRL:
	case NAU83110_RB6_ANALOG_CTRL0 ... NAU83110_RB7_ANALOG_CTRL1:
	case NAU83110_RBB_PWRSTAGE_CTRL0 ... NAU83110_RBD_PWRSTAGE_CTRL2:
		return true;
	default:
		return false;
	}
}

static bool nau83110_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case NAU83110_R00_SW_RST_EN ... NAU83110_R02_I2C_ADDRESS_LATCH:
	case NAU83110_R19_INTR_CTRL3:
	case NAU83110_R1B_INTR_CTRL4:
	case NAU83110_RB9_ANALOG_READ0 ... NAU83110_RBA_ANALOG_READ1:
	case NAU83110_RF0_DEV_INFO0 ... NAU83110_RF1_DEV_INFO1:
		return true;
	default:
		return false;
	}
}

static const DECLARE_TLV_DB_SCALE(dac_vol_tlv, -12150, 50, 0);

static const char *const nau83110_osr_list[] = { "64", "256", "128", "", "32" };

static const struct soc_enum nau83110_dac_osr_enum =
	SOC_ENUM_SINGLE(NAU83110_R76_DAC_CTRL1, 0,
		ARRAY_SIZE(nau83110_osr_list), nau83110_osr_list);

static const char *const nau83110_tdm_slot[] = { "", "", "", "Slot 0", "Slot 1",
	"Slot 2", "Slot 3", "Slot 4", "Slot 5", "Slot 6", "Slot 7", "Slot 0 + 1",
	"Slot 4 + 5" };

static const struct soc_enum nau83110_tdm_sel_enum =
	SOC_ENUM_SINGLE(NAU83110_R7D_TDM_RX_CTRL1, 0,
		ARRAY_SIZE(nau83110_tdm_slot), nau83110_tdm_slot);

static int nau83110_wait_for_bit(struct regmap *regmap, unsigned int reg,
	unsigned int mask, unsigned int retries, unsigned int delay_us)
{
	int status = 0;

	while (retries) {
		regmap_read(regmap, reg, &status);
		if (status & mask)
			break;

		usleep_range(delay_us, delay_us + 500);
		retries--;
	}

	if (!retries)
		goto timeout;

	return 0;

timeout:
	return -ETIMEDOUT;
}

static inline int wait_mute_done(struct regmap *regmap)
{
	return nau83110_wait_for_bit(regmap, NAU83110_R1B_INTR_CTRL4,
		BIT(1), 20, 2000);
}

static inline int wait_diag_done(struct regmap *regmap)
{
	return nau83110_wait_for_bit(regmap, NAU83110_R19_INTR_CTRL3,
		BIT(2), 20, 20000);
}

static int load_diagnostic(struct regmap *regmap)
{
	int bclk_status = 0;

	regmap_update_bits(regmap, NAU83110_R7A_MUTE_CTRL,
		NAU83110_SOFT_MUTE, NAU83110_SOFT_MUTE);
	if (wait_mute_done(regmap))
		goto timeout;

	/* Set HV(PWM) off */
	regmap_update_bits(regmap, NAU83110_RBD_PWRSTAGE_CTRL2,
		NAU83110_PWRSTAGE_CTRL2, 0);
	regmap_write(regmap, NAU83110_RBB_PWRSTAGE_CTRL0, 0xb90b);
	regmap_write(regmap, NAU83110_RBC_PWRSTAGE_CTRL1, 0x3818);

	/* OSC48M Enable */
	regmap_update_bits(regmap, NAU83110_RA8_STANDBY_CTRL,
		NAU83110_STANDBY_CTRL_MASK, NAU83110_STANDBY_CTRL);
	/* Check BCLK status */
	regmap_read(regmap, NAU83110_R1B_INTR_CTRL4, &bclk_status);
	if (bclk_status & BIT(15))
		msleep(100);

	regmap_write(regmap, NAU83110_R19_INTR_CTRL3, 0xffff);
	/* Trigger Load Diagnostic*/
	regmap_update_bits(regmap, NAU83110_R6D_DIAG_CTRL0, NAU83110_DIAG_EN,
		NAU83110_DIAG_EN);
	if (wait_diag_done(regmap))
		goto timeout;

	regmap_update_bits(regmap, NAU83110_R6D_DIAG_CTRL0, NAU83110_DIAG_EN, 0);
	/* OSC48M Disable */
	regmap_update_bits(regmap, NAU83110_RA8_STANDBY_CTRL,
		NAU83110_STANDBY_CTRL_MASK, 0);
	regmap_write(regmap, NAU83110_RBB_PWRSTAGE_CTRL0, 0x810b);
	regmap_write(regmap, NAU83110_RBC_PWRSTAGE_CTRL1, 0x0008);
	/* Set HV(PWM) on */
	regmap_update_bits(regmap, NAU83110_RBD_PWRSTAGE_CTRL2,
		NAU83110_PWRSTAGE_CTRL2, NAU83110_PWRSTAGE_CTRL2);
	regmap_update_bits(regmap, NAU83110_R7A_MUTE_CTRL, NAU83110_SOFT_MUTE, 0);

	return 0;

timeout:
	return -EINVAL;
}

static int nau83110_diag_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = 0;

	return 0;
}

static int nau83110_diag_put(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *comp = snd_soc_kcontrol_component(kcontrol);
	struct nau83110 *nau83110 = snd_soc_component_get_drvdata(comp);
	int ret;

	if (!ucontrol->value.integer.value[0])
		return 0;

	ret = load_diagnostic(nau83110->regmap);
	dev_info(nau83110->dev, "load diagnostic %s", ret ? "fail" : "success");

	return ret;
}

static const struct snd_kcontrol_new nau83110_snd_controls[] = {
	SOC_SINGLE_EXT("Run Diagnostic", SND_SOC_NOPM, 0, 1, 0,
		nau83110_diag_get, nau83110_diag_put),
	SOC_SINGLE_TLV("Digital Playback Volume",
		NAU83110_R58_DAC_VOLUME_CTRL, 0,
		NAU83110_DAC_VOL_MAX, 0, dac_vol_tlv),
	SOC_ENUM("TDM Channel Select", nau83110_tdm_sel_enum),
	SOC_ENUM("DAC Oversampling Rate", nau83110_dac_osr_enum),
	SOC_SINGLE("HPF Switch", NAU83110_R77_HPF_CTRL, NAU83110_HPF_EN_SFT, 1, 0),
	SOC_SINGLE("HPF App Switch", NAU83110_R77_HPF_CTRL, NAU83110_HPF_APP_SFT, 1, 0),
	SOC_SINGLE("DAC DEM Switch", NAU83110_R76_DAC_CTRL1, NAU83110_HPF_EN_SFT, 1, 1),
	SOC_SINGLE("DAC DEM Delay Switch", NAU83110_R76_DAC_CTRL1,
		NAU83110_HPF_APP_SFT, 1, 0),
 	SOC_SINGLE("DAC Zero Crossing Switch", NAU83110_R7A_MUTE_CTRL,
		NAU83110_DACZCEN_SFT, 1, 0),
	SOC_SINGLE("Slow Timer Clock Switch", NAU83110_R59_ALC_MODE_CTRL,
		NAU83110_SLOWCLKEN_SFT, 1, 0),
	SOC_SINGLE("Auto attenuation Switch", NAU83110_R59_ALC_MODE_CTRL,
		NAU83110_ENAUTOATT_SFT, 1, 0),
	SOC_SINGLE("ALC Zero Crossing Switch", NAU83110_R59_ALC_MODE_CTRL,
		NAU83110_ALCZC_SFT, 1, 0),
	SOC_SINGLE("ALC Switch", NAU83110_R59_ALC_MODE_CTRL,
		NAU83110_ALCEN_SFT, 1, 0),
	SOC_SINGLE("ALC Clip Detection Switch", NAU83110_R74_CLIP_CTRL,
		NAU83110_CLIP_EN_SFT, 1, 0),
	SOC_SINGLE("ALC Decay",	NAU83110_R5A_ALCDCY_CLIP, 0, 0x7fff, 0),
	SOC_SINGLE("ALC Attack", NAU83110_R5B_ALCATK_CLIP, 0, 0x7fff, 0),
	SOC_SINGLE("ALC Hold", NAU83110_R5C_ALCHLD_CLIP, 0, 0x7ff, 0),
	SOC_SINGLE("ALC Min Gain", NAU83110_R5D_ALC_MINGAIN_CLIP, 8, 31, 0),
	SOC_SINGLE("ALC Attack OTW2", NAU83110_R5F_ALCATK_OTW2, 0, 0x7fff, 0),
	SOC_SINGLE("ALC Hold OTW2", NAU83110_R60_ALCHLD_OTW2, 0, 0x7ff, 0),
	SOC_SINGLE("ALC Min Gain OTW2", NAU83110_R61_ALC_MINGAIN_OTW2, 8, 31, 0),
	SOC_SINGLE("ALC Attack OTW1", NAU83110_R63_ALCATK_OTW1, 0, 0x7fff, 0),
	SOC_SINGLE("ALC Hold OTW1", NAU83110_R64_ALCHLD_OTW1, 0, 0x7ff, 0),
	SOC_SINGLE("ALC Min Gain OTW1", NAU83110_R65_ALC_MINGAIN_OTW1, 8, 31, 0),
	SOC_SINGLE("ALC Attack UVLOP", NAU83110_R67_ALCATK_UVLOP, 0, 0x7fff, 0),
	SOC_SINGLE("ALC Hold UVLOP", NAU83110_R68_ALCHLD_UVLOP, 0, 0x7ff, 0),
	SOC_SINGLE("ALC Min Gain UVLOP", NAU83110_R69_ALC_MINGAIN_STEP_UVLOP, 8, 31, 0),
};

static const struct snd_soc_dapm_widget nau83110_dapm_widgets[] = {
	SND_SOC_DAPM_OUTPUT("SPKP"),
	SND_SOC_DAPM_OUTPUT("SPKN"),
};

static const struct snd_soc_dapm_route nau83110_dapm_routes[] = {
	{"SPKP", NULL, "Playback"},
	{"SPKN", NULL, "Playback"},
};

#ifdef ENABLE_ISR
static void nau83110_trigger_work(struct work_struct *work)
{
	struct nau83110 *nau83110 = container_of(work, struct nau83110, trigger_work);

	switch (nau83110->work_cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		regmap_update_bits(nau83110->regmap, NAU83110_R18_INTR_CTRL2,
			NAU83110_COMMON_INT_MASK, 0);
		regmap_write(nau83110->regmap, NAU83110_R19_INTR_CTRL3, 0xffff);
		enable_irq(nau83110->irq);
		break;

	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		disable_irq(nau83110->irq);
		regmap_write(nau83110->regmap, NAU83110_R18_INTR_CTRL2, 0xffff);
		regmap_write(nau83110->regmap, NAU83110_R19_INTR_CTRL3, 0xffff);
		break;
	}
}

static int nau83110_trigger(struct snd_pcm_substream *substream,
		int cmd, struct snd_soc_dai *dai)
{
	struct snd_soc_component *component = dai->component;
	struct nau83110 *nau83110 = snd_soc_component_get_drvdata(component);

	nau83110->work_cmd = cmd;
	schedule_work(&nau83110->trigger_work);

	return 0;
}
#endif

static int nau83110_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	struct snd_soc_component *component = dai->component;
	int ctrl_val = 0;

	switch (params_width(params)) {
	case 16:
		ctrl_val |= NAU83110_WLEN_16;
		break;
	case 20:
		ctrl_val |= NAU83110_WLEN_20;
		break;
	case 24:
		ctrl_val |= NAU83110_WLEN_24;
		break;
	case 32:
		ctrl_val |= NAU83110_WLEN_32;
		break;
	default:
		goto err;
	}

	snd_soc_component_update_bits(component, NAU83110_R7C_TDM_RX_CTRL0,
		NAU83110_WLEN_MASK, ctrl_val);

	return 0;
err:
	return -EINVAL;
}

static int nau83110_mute(struct snd_soc_dai *dai, int mute, int direction)
{
	struct snd_soc_component *component = dai->component;

	return snd_soc_component_update_bits(component, NAU83110_R7A_MUTE_CTRL,
		NAU83110_SOFT_MUTE, mute ? NAU83110_SOFT_MUTE : 0);
}

static int nau83110_set_dai_fmt(struct snd_soc_dai *codec_dai, unsigned int fmt)
{
	struct snd_soc_component *component = codec_dai->component;
	u16 ctrl0_val = 0, ctrl1_val = 0;

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBS_CFS:
		break;
	default:
		goto err;
	}

	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		break;
	case SND_SOC_DAIFMT_IB_NF:
		ctrl1_val |= NAU83110_BCLK_INV;
		break;
	default:
		goto err;
	}

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		ctrl0_val |= NAU83110_AIFMT_I2S;
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		ctrl0_val |= NAU83110_AIFMT_RJ;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		ctrl0_val |= NAU83110_AIFMT_LJ;
		break;
	case SND_SOC_DAIFMT_DSP_B:
		ctrl0_val |= (NAU83110_AIFMT_PCM | NAU83110_LRP0_PCMB);
		break;
	case SND_SOC_DAIFMT_DSP_A:
		ctrl0_val |= (NAU83110_AIFMT_PCM | NAU83110_LRP0_PCMA);
		break;
	default:
		goto err;
	}

	snd_soc_component_update_bits(component, NAU83110_R7C_TDM_RX_CTRL0,
		NAU83110_AIFMT_MASK | NAU83110_LRP0_MASK, ctrl0_val);
	snd_soc_component_update_bits(component, NAU83110_R7D_TDM_RX_CTRL1,
		NAU83110_BCLK_INV, ctrl1_val);
	return 0;
err:
	return -EINVAL;
}

/*
 * Configures a DAI for TDM operation. Only 8 RX slots are supported.
 *     0x3: TDM channel selection mixing slot 0 and slot 1.
 *     0x30: TDM channel selection mixing slot 4 and slot 5.
 */
static int nau83110_set_tdm_slot(struct snd_soc_dai *dai, unsigned int tx_mask,
	unsigned int rx_mask, int slots, int slot_width)
{
	struct snd_soc_component *component = dai->component;
	u16 ctrl0_val = 0;

	if (!(rx_mask & 0xff))
		goto err;

	switch (rx_mask) {
	case BIT(0):
		ctrl0_val |= NAU83110_TDM_CH_SEL_SLOT0;
		break;
	case BIT(1):
		ctrl0_val |= NAU83110_TDM_CH_SEL_SLOT1;
		break;
	case BIT(0) | BIT(1):
		ctrl0_val |= NAU83110_TDM_CH_SEL_SLOT01;
		break;
	case BIT(2):
		ctrl0_val |= NAU83110_TDM_CH_SEL_SLOT2;
		break;
	case BIT(3):
		ctrl0_val |= NAU83110_TDM_CH_SEL_SLOT3;
		break;
	case BIT(4):
		ctrl0_val |= NAU83110_TDM_CH_SEL_SLOT4;
		break;
	case BIT(5):
		ctrl0_val |= NAU83110_TDM_CH_SEL_SLOT5;
		break;
	case BIT(4) | BIT(5):
		ctrl0_val |= NAU83110_TDM_CH_SEL_SLOT45;
		break;
	case BIT(6):
		ctrl0_val |= NAU83110_TDM_CH_SEL_SLOT6;
		break;
	case BIT(7):
		ctrl0_val |= NAU83110_TDM_CH_SEL_SLOT7;
		break;
	default:
		goto err;
	}

	snd_soc_component_update_bits(component, NAU83110_R78_I2S_PCM_CTRL2,
		NAU83110_PCM_TS_EN, NAU83110_PCM_TS_EN);
	snd_soc_component_update_bits(component, NAU83110_R7D_TDM_RX_CTRL1,
		NAU83110_TDM_CH_SEL_MASK, ctrl0_val);

	return 0;
err:
	return -EINVAL;
}

static const struct snd_soc_dai_ops nau83110_ops = {
#ifdef ENABLE_ISR
	.trigger = nau83110_trigger,
#endif
	.hw_params = nau83110_hw_params,
	.mute_stream = nau83110_mute,
	.set_fmt = nau83110_set_dai_fmt,
	.set_tdm_slot = nau83110_set_tdm_slot,
};

#define NAU83110_RATES SNDRV_PCM_RATE_8000_96000

#define NAU83110_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE | \
	SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE)

static struct snd_soc_dai_driver nau83110_dai = {
	.name = "nau83110-hifi",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 1,
		.channels_max = 2,   /* Only 1 channel of data */
		.rates = NAU83110_RATES,
		.formats = NAU83110_FORMATS,
	},
	.ops = &nau83110_ops,
};

static const struct regmap_config nau83110_regmap_config = {
	.reg_bits = NAU83110_REG_ADDR_LEN,
	.val_bits = NAU83110_REG_DATA_LEN,
	.max_register = NAU83110_REG_MAX,
	.volatile_reg = nau83110_volatile_reg,
	.readable_reg = nau83110_readable_reg,
	.writeable_reg = nau83110_writeable_reg,
	.cache_type = REGCACHE_RBTREE,
	.reg_defaults = nau83110_reg_defaults,
	.num_reg_defaults = ARRAY_SIZE(nau83110_reg_defaults),
};

static int __maybe_unused nau83110_suspend(struct snd_soc_component *component)
{
	struct nau83110 *nau83110 = snd_soc_component_get_drvdata(component);

	regcache_mark_dirty(nau83110->regmap);
	/* set enable pin to low to enter reset state */
	if (nau83110->enable_pin)
		gpiod_set_value(nau83110->enable_pin, 0);

	return 0;
}

static int __maybe_unused nau83110_resume(struct snd_soc_component *component)
{
	struct nau83110 *nau83110 = snd_soc_component_get_drvdata(component);

	/* set enable pin to high to leave reset state */
	if (nau83110->enable_pin) {
		gpiod_set_value(nau83110->enable_pin, 1);
		mdelay(1);
	}
	regcache_sync(nau83110->regmap);

	return 0;
}

static const struct snd_soc_component_driver nau83110_component_driver = {
	.suspend		= nau83110_suspend,
	.resume			= nau83110_resume,
	.controls		= nau83110_snd_controls,
	.num_controls		= ARRAY_SIZE(nau83110_snd_controls),
	.dapm_widgets		= nau83110_dapm_widgets,
	.num_dapm_widgets	= ARRAY_SIZE(nau83110_dapm_widgets),
	.dapm_routes		= nau83110_dapm_routes,
	.num_dapm_routes	= ARRAY_SIZE(nau83110_dapm_routes),
	.suspend_bias_off	= 1,
	.idle_bias_on		= 1,
	.use_pmdown_time	= 1,
	.endianness		= 1,
};

static int nau83110_init_regs(struct regmap *regmap)
{
	/* Interrupt Mask */
	regmap_write(regmap, NAU83110_R18_INTR_CTRL2, 0xffff);
	/* Interrupt Clear */
	regmap_write(regmap, NAU83110_R19_INTR_CTRL3, 0x8002);
	/* Latch IIC ADDR value */
	regmap_write(regmap, NAU83110_R02_I2C_ADDRESS_LATCH, 0x0001);
	/* Analog Control */
	regmap_write(regmap, NAU83110_RB6_ANALOG_CTRL0, 0x3e8f);
	regmap_write(regmap, NAU83110_RB7_ANALOG_CTRL1, 0x0000);

	/* Load Diagnostic Rework */
	regmap_update_bits(regmap, NAU83110_R7A_MUTE_CTRL, NAU83110_SOFT_MUTE,
		NAU83110_SOFT_MUTE);
	if (wait_mute_done(regmap))
		goto timeout;

	/* OSC48M Enable */
	regmap_update_bits(regmap, NAU83110_RA8_STANDBY_CTRL, NAU83110_STANDBY_CTRL_MASK,
		NAU83110_STANDBY_CTRL);
	/* EN ATTU*/
	regmap_update_bits(regmap, NAU83110_R59_ALC_MODE_CTRL, NAU83110_ENAUTOATT,
		NAU83110_ENAUTOATT);
	/* Set HV(PWM) off */
	regmap_update_bits(regmap, NAU83110_RBD_PWRSTAGE_CTRL2,
		NAU83110_PWRSTAGE_CTRL2, 0);
	regmap_write(regmap, NAU83110_RBB_PWRSTAGE_CTRL0, 0xb90b);
	regmap_write(regmap, NAU83110_RBC_PWRSTAGE_CTRL1, 0x3818);
	/* Read Efuse */
	regmap_update_bits(regmap, NAU83110_RA8_STANDBY_CTRL, NAU83110_EFUASE_READ_MASK,
		NAU83110_EFUASE_READ);
	msleep(300);
	/* OSC48M Disable */
	regmap_update_bits(regmap, NAU83110_RA8_STANDBY_CTRL, NAU83110_STANDBY_CTRL_MASK,
		0);
	regmap_write(regmap, NAU83110_RBB_PWRSTAGE_CTRL0, 0x810b);
	regmap_write(regmap, NAU83110_RBC_PWRSTAGE_CTRL1, 0x0008);
	/* Set HV(PWM) on */
	regmap_update_bits(regmap, NAU83110_RBD_PWRSTAGE_CTRL2, NAU83110_PWRSTAGE_CTRL2,
		NAU83110_PWRSTAGE_CTRL2);
	regmap_update_bits(regmap, NAU83110_R7A_MUTE_CTRL, NAU83110_SOFT_MUTE, 0);
	/* TDM RX Control */
	regmap_write(regmap, NAU83110_R7C_TDM_RX_CTRL0, 0x000a);
	regmap_write(regmap, NAU83110_R7D_TDM_RX_CTRL1, 0x0000);
	/* TDM TX Control */
	regmap_write(regmap, NAU83110_R80_TDM_TX_CTRL0, 0x0005);
	regmap_write(regmap, NAU83110_R81_TDM_TX_CTRL1, 0x1801);
	/* Disable Interrupt */
	regmap_update_bits(regmap, NAU83110_R18_INTR_CTRL2, NAU83110_COMMON_INT_MASK, 0);

	return 0;

timeout:
	return -EINVAL;
}

#ifdef ENABLE_ISR
static irqreturn_t nau83110_interrupt(int irq, void *data)
{
	struct nau83110 *nau83110 = (struct nau83110 *)data;
	struct regmap *regmap = nau83110->regmap;
	int active_int, clear_int = 0;	

	if (regmap_read(regmap, NAU83110_R19_INTR_CTRL3, &active_int))
		return IRQ_NONE;

	if (active_int & NAU83110_INT_BCLKDETB) {
		dev_dbg(nau83110->dev, "BCLKDET occur");
		clear_int |= NAU83110_INT_BCLKDETB;
	}

	if (active_int & NAU83110_INT_TALARM) {
		dev_dbg(nau83110->dev, "TALARM occur");
		clear_int |= NAU83110_INT_TALARM;
	}

	if (active_int & NAU83110_INT_OWT2) {
		dev_dbg(nau83110->dev, "OWT2 occur");
		clear_int |= NAU83110_INT_OWT2;
	}

	if (active_int & NAU83110_INT_OWT1) {
		dev_dbg(nau83110->dev, "OWT1 occur");
		clear_int |= NAU83110_INT_OWT1;
	}

	if (active_int & NAU83110_INT_SCP) {
		dev_dbg(nau83110->dev, "SCP occur");
		clear_int |= NAU83110_INT_SCP;
	}

	if (active_int & NAU83110_INT_UVLOP) {
		dev_dbg(nau83110->dev, "UVLOP occur");
		clear_int |= NAU83110_INT_UVLOP;
	}

	if (active_int & NAU83110_INT_OVLOB) {
		dev_dbg(nau83110->dev, "OVLOB occur");
		clear_int |= NAU83110_INT_OVLOB;
	}

	regmap_write(regmap, NAU83110_R19_INTR_CTRL3, clear_int);

	return IRQ_HANDLED;
}

static int nau83110_setup_irq(struct nau83110 *nau83110)
{
	int ret;

	ret = devm_request_threaded_irq(nau83110->dev, nau83110->irq, NULL,
		nau83110_interrupt, IRQF_TRIGGER_HIGH | IRQF_ONESHOT, "nau83110",
		nau83110);
	if (ret) {
		dev_err(nau83110->dev, "Cannot request irq %d (%d)", nau83110->irq, ret);
		return ret;
	}

	disable_irq(nau83110->irq);

	return 0;
}
#endif

static inline void nau83110_software_reset(struct regmap *regmap)
{
	regmap_write(regmap, NAU83110_R00_SW_RST_EN, 0x5a5a);
	regmap_write(regmap, NAU83110_R00_SW_RST_EN, 0xa5a5);
}

static inline void nau83110_audio_reset(struct regmap *regmap)
{
	regmap_write(regmap, NAU83110_R01_AUDIO_RST_EN, 0x5a5a);
	regmap_write(regmap, NAU83110_R01_AUDIO_RST_EN, 0xa5a5);
}

static int nau83110_i2c_probe(struct i2c_client *i2c)
{
	struct device *dev = &i2c->dev;
	struct nau83110 *nau83110 = dev_get_platdata(dev);
	int ret;

	if (!nau83110) {
		nau83110 = devm_kzalloc(dev, sizeof(*nau83110), GFP_KERNEL);
		if (!nau83110)
			return -ENOMEM;
	}

	nau83110->enable_pin = devm_gpiod_get_optional(dev, "nuvoton,enable",
		GPIOD_OUT_LOW);
	if (IS_ERR(nau83110->enable_pin))
		return PTR_ERR(nau83110->enable_pin);
	gpiod_set_value(nau83110->enable_pin, 1);
	mdelay(1);

	i2c_set_clientdata(i2c, nau83110);

	nau83110->regmap = devm_regmap_init_i2c(i2c, &nau83110_regmap_config);
	if (IS_ERR(nau83110->regmap))
		return PTR_ERR(nau83110->regmap);
	nau83110->dev = dev;

	/* Reset the codec */
	nau83110_software_reset(nau83110->regmap);
	nau83110_audio_reset(nau83110->regmap);

	ret = nau83110_init_regs(nau83110->regmap);
	if (ret) {
		dev_err(dev, "NAU83110 init failed(%d)", ret);
		return -ENODEV;
	}

#ifdef ENABLE_ISR
	nau83110->irq = i2c->irq;
	if (i2c->irq) {
		INIT_WORK(&nau83110->trigger_work, nau83110_trigger_work);
		nau83110_setup_irq(nau83110);
	}
#endif

	return devm_snd_soc_register_component(dev,
		&nau83110_component_driver, &nau83110_dai, 1);
}

static const struct i2c_device_id nau83110_i2c_id[] = {
	{ "nau83110", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, nau83110_i2c_id);

#ifdef CONFIG_OF
static const struct of_device_id nau83110_of_match[] = {
	{ .compatible = "nuvoton,nau83110", },
	{ }
};
MODULE_DEVICE_TABLE(of, nau83110_of_match);
#endif

static struct i2c_driver nau83110_i2c_driver = {
	.driver = {
		.name = "nau83110",
		.of_match_table = of_match_ptr(nau83110_of_match),
	},
	.probe = nau83110_i2c_probe,
	.id_table = nau83110_i2c_id,
};
module_i2c_driver(nau83110_i2c_driver);

MODULE_DESCRIPTION("ASoC NAU3110 driver");
MODULE_AUTHOR("Neo Chang <ylchang2@nuvoton.com>");
MODULE_LICENSE("GPL v2");
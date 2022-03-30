/*
 * NAU88L24 ALSA SoC audio driver
 *
 * Copyright 2016 Nuvoton Technology Corp.
 * Author: John Hsu <KCHSU0@nuvoton.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __NAU8824_H__
#define __NAU8824_H__

#define NAU8824_REG_RESET			0x00
#define NAU8824_REG_ENA_CTRL			0x01
#define NAU8824_REG_CLK_GATING_ENA		0x02
#define NAU8824_REG_CLK_DIVIDER		0x03
#define NAU8824_REG_FLL1			0x04
#define NAU8824_REG_FLL2			0x05
#define NAU8824_REG_FLL3			0x06
#define NAU8824_REG_FLL4			0x07
#define NAU8824_REG_FLL5			0x08
#define NAU8824_REG_FLL6			0x09
#define NAU8824_REG_FLL_VCO_RSV		0x0A
#define NAU8824_REG_JACK_DET_CTRL		0x0D
#define NAU8824_REG_INTERRUPT_SETTING_1	0x0F
#define NAU8824_REG_IRQ			0x10
#define NAU8824_REG_CLEAR_INT_REG		0x11
#define NAU8824_REG_INTERRUPT_SETTING	0x12
#define NAU8824_REG_SAR_ADC			0x13
#define NAU8824_REG_VDET_COEFFICIENT		0x14
#define NAU8824_REG_VDET_THRESHOLD_1	0x15
#define NAU8824_REG_VDET_THRESHOLD_2	0x16
#define NAU8824_REG_VDET_THRESHOLD_3	0x17
#define NAU8824_REG_VDET_THRESHOLD_4	0x18
#define NAU8824_REG_GPIO_SEL			0x1A
#define NAU8824_REG_PORT0_I2S_PCM_CTRL_1	0x1C
#define NAU8824_REG_PORT0_I2S_PCM_CTRL_2	0x1D
#define NAU8824_REG_PORT0_LEFT_TIME_SLOT	0x1E
#define NAU8824_REG_PORT0_RIGHT_TIME_SLOT	0x1F
#define NAU8824_REG_TDM_CTRL			0x20
#define NAU8824_REG_ADC_HPF_FILTER		0x23
#define NAU8824_REG_ADC_FILTER_CTRL		0x24
#define NAU8824_REG_DAC_FILTER_CTRL_1	0x25
#define NAU8824_REG_DAC_FILTER_CTRL_2	0x26
#define NAU8824_REG_NOTCH_FILTER_1		0x27
#define NAU8824_REG_NOTCH_FILTER_2		0x28
#define NAU8824_REG_EQ1_LOW			0x29
#define NAU8824_REG_EQ2_EQ3			0x2A
#define NAU8824_REG_EQ4_EQ5			0x2B
#define NAU8824_REG_ADC_CH0_DGAIN_CTRL	0x2D
#define NAU8824_REG_ADC_CH1_DGAIN_CTRL	0x2E
#define NAU8824_REG_ADC_CH2_DGAIN_CTRL	0x2F
#define NAU8824_REG_ADC_CH3_DGAIN_CTRL	0x30
#define NAU8824_REG_DAC_MUTE_CTRL		0x31
#define NAU8824_REG_DAC_CH0_DGAIN_CTRL	0x32
#define NAU8824_REG_DAC_CH1_DGAIN_CTRL	0x33
#define NAU8824_REG_ADC_TO_DAC_ST		0x34
#define NAU8824_REG_DRC_KNEE_IP12_ADC_CH01	0x38
#define NAU8824_REG_DRC_KNEE_IP34_ADC_CH01	0x39
#define NAU8824_REG_DRC_SLOPE_ADC_CH01	0x3A
#define NAU8824_REG_DRC_ATKDCY_ADC_CH01	0x3B
#define NAU8824_REG_DRC_KNEE_IP12_ADC_CH23	0x3C
#define NAU8824_REG_DRC_KNEE_IP34_ADC_CH23	0x3D
#define NAU8824_REG_DRC_SLOPE_ADC_CH23	0x3E
#define NAU8824_REG_DRC_ATKDCY_ADC_CH23	0x3F
#define NAU8824_REG_DRC_GAINL_ADC0		0x40
#define NAU8824_REG_DRC_GAINL_ADC1		0x41
#define NAU8824_REG_DRC_GAINL_ADC2		0x42
#define NAU8824_REG_DRC_GAINL_ADC3		0x43
#define NAU8824_REG_DRC_KNEE_IP12_DAC	0x45
#define NAU8824_REG_DRC_KNEE_IP34_DAC	0x46
#define NAU8824_REG_DRC_SLOPE_DAC		0x47
#define NAU8824_REG_DRC_ATKDCY_DAC		0x48
#define NAU8824_REG_DRC_GAIN_DAC_CH0	0x49
#define NAU8824_REG_DRC_GAIN_DAC_CH1	0x4A
#define NAU8824_REG_MODE			0x4C
#define NAU8824_REG_MODE1			0x4D
#define NAU8824_REG_MODE2			0x4E
#define NAU8824_REG_CLASSG			0x50
#define NAU8824_REG_OTP_EFUSE			0x51
#define NAU8824_REG_OTPDOUT_1		0x53
#define NAU8824_REG_OTPDOUT_2		0x54
#define NAU8824_REG_MISC_CTRL			0x55
#define NAU8824_REG_I2C_TIMEOUT		0x56
#define NAU8824_REG_TEST_MODE		0x57
#define NAU8824_REG_I2C_DEVICE_ID		0x58
#define NAU8824_REG_SAR_ADC_DATA_OUT	0x59
#define NAU8824_REG_BIAS_ADJ			0x66
#define NAU8824_REG_PGA_GAIN			0x67
#define NAU8824_REG_TRIM_SETTINGS		0x68
#define NAU8824_REG_ANALOG_CONTROL_1	0x69
#define NAU8824_REG_ANALOG_CONTROL_2	0x6A
#define NAU8824_REG_ENABLE_LO			0x6B
#define NAU8824_REG_GAIN_LO			0x6C
#define NAU8824_REG_CLASSD_GAIN_1		0x6D
#define NAU8824_REG_CLASSD_GAIN_2		0x6E
#define NAU8824_REG_ANALOG_ADC_1		0x71
#define NAU8824_REG_ANALOG_ADC_2		0x72
#define NAU8824_REG_RDAC			0x73
#define NAU8824_REG_MIC_BIAS			0x74
#define NAU8824_REG_HS_VOLUME_CONTROL	0x75
#define NAU8824_REG_BOOST			0x76
#define NAU8824_REG_FEPGA			0x77
#define NAU8824_REG_FEPGA_II			0x78
#define NAU8824_REG_FEPGA_SE			0x79
#define NAU8824_REG_FEPGA_ATTENUATION	0x7A
#define NAU8824_REG_ATT_PORT0		0x7B
#define NAU8824_REG_ATT_PORT1		0x7C
#define NAU8824_REG_POWER_UP_CONTROL	0x7F
#define NAU8824_REG_CHARGE_PUMP_CONTROL	0x80
#define NAU8824_REG_CHARGE_PUMP_INPUT	0x81
#define NAU8824_REG_MAX			NAU8824_REG_CHARGE_PUMP_INPUT
/* 16-bit control register address, and 16-bits control register data */
#define NAU8824_REG_ADDR_LEN		16
#define NAU8824_REG_DATA_LEN		16


/* ENA_CTRL (0x1) */
#define NAU8824_DMIC_LCH_EDGE_CH23	(0x1 << 12)
#define NAU8824_DMIC_LCH_EDGE_CH01	(0x1 << 11)
#define NAU8824_JD_SLEEP_MODE		(0x1 << 10)
#define NAU8824_ADC_CH3_DMIC_SFT	9
#define NAU8824_ADC_CH3_DMIC_EN	(0x1 << NAU8824_ADC_CH3_DMIC_SFT)
#define NAU8824_ADC_CH2_DMIC_SFT	8
#define NAU8824_ADC_CH2_DMIC_EN	(0x1 << NAU8824_ADC_CH2_DMIC_SFT)
#define NAU8824_ADC_CH1_DMIC_SFT	7
#define NAU8824_ADC_CH1_DMIC_EN	(0x1 << NAU8824_ADC_CH1_DMIC_SFT)
#define NAU8824_ADC_CH0_DMIC_SFT	6
#define NAU8824_ADC_CH0_DMIC_EN	(0x1 << NAU8824_ADC_CH0_DMIC_SFT)
#define NAU8824_DAC_CH1_EN		(0x1 << 5)
#define NAU8824_DAC_CH0_EN		(0x1 << 4)
#define NAU8824_ADC_CH3_EN		(0x1 << 3)
#define NAU8824_ADC_CH2_EN		(0x1 << 2)
#define NAU8824_ADC_CH1_EN		(0x1 << 1)
#define NAU8824_ADC_CH0_EN		0x1

/* CLK_GATING_ENA (0x02) */
#define NAU8824_CLK_ADC_CH23_EN	(0x1 << 15)
#define NAU8824_CLK_ADC_CH01_EN	(0x1 << 14)
#define NAU8824_CLK_DAC_CH1_EN	(0x1 << 13)
#define NAU8824_CLK_DAC_CH0_EN	(0x1 << 12)
#define NAU8824_CLK_I2S_EN		(0x1 << 7)
#define NAU8824_CLK_GAIN_EN		(0x1 << 5)
#define NAU8824_CLK_SAR_EN		(0x1 << 3)
#define NAU8824_CLK_DMIC_CH23_EN	(0x1 << 1)

/* CLK_DIVIDER (0x3) */
#define NAU8824_CLK_SRC_SFT		15
#define NAU8824_CLK_SRC_MASK		(1 << NAU8824_CLK_SRC_SFT)
#define NAU8824_CLK_SRC_VCO		(1 << NAU8824_CLK_SRC_SFT)
#define NAU8824_CLK_SRC_MCLK		(0 << NAU8824_CLK_SRC_SFT)
#define NAU8824_CLK_MCLK_SRC_MASK	(0xf << 0)
#define NAU8824_CLK_DMIC_SRC_SFT	10
#define NAU8824_CLK_DMIC_SRC_MASK	(0x7 << NAU8824_CLK_DMIC_SRC_SFT)
#define NAU8824_CLK_ADC_SRC_SFT	6
#define NAU8824_CLK_ADC_SRC_MASK	(0x3 << NAU8824_CLK_ADC_SRC_SFT)
#define NAU8824_CLK_DAC_SRC_SFT	4
#define NAU8824_CLK_DAC_SRC_MASK	(0x3 << NAU8824_CLK_DAC_SRC_SFT)

/* FLL1 (0x04) */
#define NAU8824_FLL_RATIO_MASK	(0x7f << 0)

/* FLL3 (0x06) */
#define NAU8824_FLL_INTEGER_MASK	(0x3ff << 0)
#define NAU8824_FLL_CLK_SRC_SFT	10
#define NAU8824_FLL_CLK_SRC_MASK	(0x3 << NAU8824_FLL_CLK_SRC_SFT)
#define NAU8824_FLL_CLK_SRC_MCLK	(0 << NAU8824_FLL_CLK_SRC_SFT)
#define NAU8824_FLL_CLK_SRC_BLK	(0x2 << NAU8824_FLL_CLK_SRC_SFT)
#define NAU8824_FLL_CLK_SRC_FS		(0x3 << NAU8824_FLL_CLK_SRC_SFT)

/* FLL4 (0x07) */
#define NAU8824_FLL_REF_DIV_SFT	10
#define NAU8824_FLL_REF_DIV_MASK	(0x3 << NAU8824_FLL_REF_DIV_SFT)

/* FLL5 (0x08) */
#define NAU8824_FLL_PDB_DAC_EN	(0x1 << 15)
#define NAU8824_FLL_LOOP_FTR_EN	(0x1 << 14)
#define NAU8824_FLL_CLK_SW_MASK	(0x1 << 13)
#define NAU8824_FLL_CLK_SW_N2		(0x1 << 13)
#define NAU8824_FLL_CLK_SW_REF	(0x0 << 13)
#define NAU8824_FLL_FTR_SW_MASK	(0x1 << 12)
#define NAU8824_FLL_FTR_SW_ACCU	(0x1 << 12)
#define NAU8824_FLL_FTR_SW_FILTER	(0x0 << 12)

/* FLL6 (0x9) */
#define NAU8824_DCO_EN			(0x1 << 15)
#define NAU8824_SDM_EN			(0x1 << 14)

/* IRQ (0x10) */
#define NAU8824_SHORT_CIRCUIT_IRQ		(0x1 << 7)
#define NAU8824_IMPEDANCE_MEAS_IRQ		(0x1 << 6)
#define NAU8824_KEY_RELEASE_IRQ		(0x1 << 5)
#define NAU8824_KEY_LONG_PRESS_IRQ		(0x1 << 4)
#define NAU8824_KEY_SHORT_PRESS_IRQ		(0x1 << 3)
#define NAU8824_JACK_EJECTION_DETECTED	(0x1 << 1)
#define NAU8824_JACK_INSERTION_DETECTED	0x1

/* JACK_DET_CTRL (0x0D) */
#define NAU8824_JACK_EJECT_DT_SFT	2
#define NAU8824_JACK_EJECT_DT_MASK (0x3 << NAU8824_JACK_EJECT_DT_SFT)
#define NAU8824_JACK_LOGIC		(0x1 << 1)


/* INTERRUPT_SETTING_1 (0x0F) */
#define NAU8824_IRQ_EJECT_EN		(0x1 << 9)
#define NAU8824_IRQ_INSERT_EN		(0x1 << 8)

/* INTERRUPT_SETTING (0x12) */
#define NAU8824_IRQ_KEY_RELEASE_DIS		(0x1 << 5)
#define NAU8824_IRQ_KEY_SHORT_PRESS_DIS	(0x1 << 3)
#define NAU8824_IRQ_EJECT_DIS			(0x1 << 1)
#define NAU8824_IRQ_INSERT_DIS		0x1

/* SAR_ADC (0x13) */
#define NAU8824_SAR_ADC_EN_SFT		12
#define NAU8824_SAR_TRACKING_GAIN_SFT	8
#define NAU8824_SAR_TRACKING_GAIN_MASK	(0x7 << NAU8824_SAR_TRACKING_GAIN_SFT)
#define NAU8824_SAR_COMPARE_TIME_SFT	2
#define NAU8824_SAR_COMPARE_TIME_MASK	(3 << 2)
#define NAU8824_SAR_SAMPLING_TIME_SFT	0
#define NAU8824_SAR_SAMPLING_TIME_MASK	(3 << 0)

/* VDET_COEFFICIENT (0x14) */
#define NAU8824_SHORTKEY_DEBOUNCE_SFT	12
#define NAU8824_SHORTKEY_DEBOUNCE_MASK	(0x3 << NAU8824_SHORTKEY_DEBOUNCE_SFT)
#define NAU8824_LEVELS_NR_SFT			8
#define NAU8824_LEVELS_NR_MASK		(0x7 << 8)
#define NAU8824_HYSTERESIS_SFT		0
#define NAU8824_HYSTERESIS_MASK		0xf

/* PORT0_I2S_PCM_CTRL_1 (0x1C) */
#define NAU8824_I2S_BP_SFT		7
#define NAU8824_I2S_BP_MASK		(1 << NAU8824_I2S_BP_SFT)
#define NAU8824_I2S_BP_INV		(1 << NAU8824_I2S_BP_SFT)
#define NAU8824_I2S_PCMB_SFT		6
#define NAU8824_I2S_PCMB_EN		(1 << NAU8824_I2S_PCMB_SFT)
#define NAU8824_I2S_DL_SFT		2
#define NAU8824_I2S_DL_MASK		(0x3 << NAU8824_I2S_DL_SFT)
#define NAU8824_I2S_DL_16		(0 << NAU8824_I2S_DL_SFT)
#define NAU8824_I2S_DL_20		(1 << NAU8824_I2S_DL_SFT)
#define NAU8824_I2S_DL_24		(2 << NAU8824_I2S_DL_SFT)
#define NAU8824_I2S_DL_32		(3 << NAU8824_I2S_DL_SFT)
#define NAU8824_I2S_DF_MASK		0x3
#define NAU8824_I2S_DF_RIGTH		0
#define NAU8824_I2S_DF_LEFT		1
#define NAU8824_I2S_DF_I2S		2
#define NAU8824_I2S_DF_PCM_AB		3


/* PORT0_I2S_PCM_CTRL_2 (0x1D) */
#define NAU8824_I2S_LRC_DIV_SFT	12
#define NAU8824_I2S_LRC_DIV_MASK	(0x3 << NAU8824_I2S_LRC_DIV_SFT)
#define NAU8824_I2S_MS_SFT		3
#define NAU8824_I2S_MS_MASK		(1 << NAU8824_I2S_MS_SFT)
#define NAU8824_I2S_MS_MASTER		(1 << NAU8824_I2S_MS_SFT)
#define NAU8824_I2S_MS_SLAVE		(0 << NAU8824_I2S_MS_SFT)
#define NAU8824_I2S_BLK_DIV_MASK	0x7

/* PORT0_LEFT_TIME_SLOT (0x1E) */
#define NAU8824_TSLOT_L_MASK	0x3ff

/* TDM_CTRL (0x20) */
#define NAU8824_TDM_MODE		(0x1 << 15)
#define NAU8824_TDM_OFFSET_EN		(0x1 << 14)
#define NAU8824_TDM_DACL_RX_SFT	6
#define NAU8824_TDM_DACL_RX_MASK	(0x3 << NAU8824_TDM_DACL_RX_SFT)
#define NAU8824_TDM_DACR_RX_SFT	4
#define NAU8824_TDM_DACR_RX_MASK	(0x3 << NAU8824_TDM_DACR_RX_SFT)
#define NAU8824_TDM_TX_MASK		0xf

/* ADC_FILTER_CTRL (0x24) */
#define NAU8824_ADC_SYNC_DOWN_MASK	0x3
#define NAU8824_ADC_SYNC_DOWN_32	0
#define NAU8824_ADC_SYNC_DOWN_64	1
#define NAU8824_ADC_SYNC_DOWN_128	2
#define NAU8824_ADC_SYNC_DOWN_256	3

/* DAC_FILTER_CTRL_1 (0x25) */
#define NAU8824_DAC_CICCLP_OFF	(0x1 << 7)
#define NAU8824_DAC_OVERSAMPLE_MASK	0x7
#define NAU8824_DAC_OVERSAMPLE_64	0
#define NAU8824_DAC_OVERSAMPLE_256	1
#define NAU8824_DAC_OVERSAMPLE_128	2
#define NAU8824_DAC_OVERSAMPLE_32	4

/* DAC_MUTE_CTRL (0x31) */
#define NAU8824_DAC_CH01_MIX		0x3
#define NAU8824_DAC_ZC_EN		(0x1 << 11)

/* DAC_CH0_DGAIN_CTRL (0x32) */
#define NAU8824_DAC_CH0_SEL_SFT	9
#define NAU8824_DAC_CH0_SEL_MASK	(0x1 << NAU8824_DAC_CH0_SEL_SFT)
#define NAU8824_DAC_CH0_SEL_I2S0	(0x0 << NAU8824_DAC_CH0_SEL_SFT)
#define NAU8824_DAC_CH0_SEL_I2S1	(0x1 << NAU8824_DAC_CH0_SEL_SFT)
#define NAU8824_DAC_CH0_VOL_MASK	0x1ff

/* DAC_CH1_DGAIN_CTRL (0x33) */
#define NAU8824_DAC_CH1_SEL_SFT	9
#define NAU8824_DAC_CH1_SEL_MASK	(0x1 << NAU8824_DAC_CH1_SEL_SFT)
#define NAU8824_DAC_CH1_SEL_I2S0	(0x0 << NAU8824_DAC_CH1_SEL_SFT)
#define NAU8824_DAC_CH1_SEL_I2S1	(0x1 << NAU8824_DAC_CH1_SEL_SFT)
#define NAU8824_DAC_CH1_VOL_MASK	0x1ff

/* CLASSG (0x50) */
#define NAU8824_CLASSG_TIMER_SFT	8
#define NAU8824_CLASSG_TIMER_MASK	(0x3f << NAU8824_CLASSG_TIMER_SFT)
#define NAU8824_CLASSG_LDAC_EN_SFT	2
#define NAU8824_CLASSG_RDAC_EN_SFT	1
#define NAU8824_CLASSG_EN_SFT		0

/* SAR_ADC_DATA_OUT (0x59) */
#define NAU8824_SAR_ADC_DATA_MASK	0xff

/* BIAS_ADJ (0x66) */
#define NAU8824_VMID			(1 << 6)
#define NAU8824_VMID_SEL_SFT		4
#define NAU8824_VMID_SEL_MASK		(3 << NAU8824_VMID_SEL_SFT)
#define NAU8824_DMIC2_EN_SFT		3
#define NAU8824_DMIC1_EN_SFT		2

/* TRIM_SETTINGS (0x68) */
#define NAU8824_DRV_CURR_INC		(1 << 15)

/* ANALOG_CONTROL_1 (0x69) */
#define NAU8824_DMIC_CLK_DRV_STRG	(1 << 3)
#define NAU8824_DMIC_CLK_SLEW_FAST	(0x7)

/* ANALOG_CONTROL_2 (0x6A) */
#define NAU8824_CLASSD_CLAMP_DIS_SFT	3
#define NAU8824_CLASSD_CLAMP_DIS	(0x1 << NAU8824_CLASSD_CLAMP_DIS_SFT)

/* ENABLE_LO (0x6B) */
#define NAU8824_TEST_DAC_SFT		14
#define NAU8824_TEST_DAC_EN		(0x3 << NAU8824_TEST_DAC_SFT)
#define NAU8824_DACL_HPR_EN_SFT	3
#define NAU8824_DACL_HPR_EN		(0x1 << NAU8824_DACL_HPR_EN_SFT)
#define NAU8824_DACR_HPR_EN_SFT	2
#define NAU8824_DACR_HPR_EN		(0x1 << NAU8824_DACR_HPR_EN_SFT)
#define NAU8824_DACR_HPL_EN_SFT	1
#define NAU8824_DACR_HPL_EN		(0x1 << NAU8824_DACR_HPL_EN_SFT)
#define NAU8824_DACL_HPL_EN_SFT	0
#define NAU8824_DACL_HPL_EN		0x1

/* CLASSD_GAIN_1 (0x6D) */
#define NAU8824_CLASSD_GAIN_1R_SFT	8
#define NAU8824_CLASSD_GAIN_1R_MASK	(0x1f << NAU8824_CLASSD_GAIN_1R_SFT)
#define NAU8824_CLASSD_EN_SFT		7
#define NAU8824_CLASSD_EN		(0x1 << NAU8824_CLASSD_EN_SFT)
#define NAU8824_CLASSD_GAIN_1L_MASK	0x1f

/* CLASSD_GAIN_2 (0x6E) */
#define NAU8824_CLASSD_GAIN_2R_SFT	8
#define NAU8824_CLASSD_GAIN_2R_MASK	(0x1f << NAU8824_CLASSD_GAIN_1R_SFT)
#define NAU8824_CLASSD_EN_SFT		7
#define NAU8824_CLASSD_EN		(0x1 << NAU8824_CLASSD_EN_SFT)
#define NAU8824_CLASSD_GAIN_2L_MASK	0x1f

/* ANALOG_ADC_2 (0x72) */
#define NAU8824_ADCR_EN_SFT		7
#define NAU8824_ADCL_EN_SFT		6

/* RDAC (0x73) */
#define NAU8824_DACR_EN_SFT		13
#define NAU8824_DACL_EN_SFT		12
#define NAU8824_DACR_CLK_SFT		9
#define NAU8824_DACL_CLK_SFT		8
#define NAU8824_RDAC_CLK_DELAY_SFT	4
#define NAU8824_RDAC_CLK_DELAY_MASK	(0x7 << NAU8824_RDAC_CLK_DELAY_SFT)
#define NAU8824_RDAC_VREF_SFT		2
#define NAU8824_RDAC_VREF_MASK	(0x3 << NAU8824_RDAC_VREF_SFT)

/* MIC_BIAS (0x74) */
#define NAU8824_MICBIAS_JKSLV		(1 << 14)
#define NAU8824_MICBIAS_JKR2		(1 << 12)
#define NAU8824_MICBIAS_POWERUP_SFT	8
#define NAU8824_MICBIAS_VOLTAGE_SFT	0
#define NAU8824_MICBIAS_VOLTAGE_MASK	0x7

/* BOOST (0x76) */
#define NAU8824_PRECHARGE_DIS			(0x1 << 13)
#define NAU8824_GLOBAL_BIAS_EN		(0x1 << 12)
#define NAU8824_HP_BOOST_DIS_SFT		9
#define NAU8824_HP_BOOST_DIS		(0x1 << NAU8824_HP_BOOST_DIS_SFT)
#define NAU8824_HP_BOOST_G_DIS_SFT		8
#define NAU8824_HP_BOOST_G_DIS		(0x1 << NAU8824_HP_BOOST_G_DIS_SFT)
#define NAU8824_SHORT_SHUTDOWN_DIG_EN	(1 << 7)
#define NAU8824_SHORT_SHUTDOWN_EN		(1 << 6)

/* FEPGA (0x77) */
#define NAU8824_FEPGA_MODER_SHORT_SFT	7
#define NAU8824_FEPGA_MODER_SHORT_EN	(0x1 << NAU8824_FEPGA_MODER_SHORT_SFT)
#define NAU8824_FEPGA_MODER_MIC2_SFT		5
#define NAU8824_FEPGA_MODER_MIC2_EN	(0x1 << NAU8824_FEPGA_MODER_MIC2_SFT)
#define NAU8824_FEPGA_MODER_HSMIC_SFT	4
#define NAU8824_FEPGA_MODER_HSMIC_EN	(0x1 << NAU8824_FEPGA_MODER_HSMIC_SFT)
#define NAU8824_FEPGA_MODEL_SHORT_SFT	3
#define NAU8824_FEPGA_MODEL_SHORT_EN	(0x1 << NAU8824_FEPGA_MODEL_SHORT_SFT)
#define NAU8824_FEPGA_MODEL_MIC1_SFT		1
#define NAU8824_FEPGA_MODEL_MIC1_EN	(0x1 << NAU8824_FEPGA_MODEL_MIC1_SFT)
#define NAU8824_FEPGA_MODEL_HSMIC_SFT	0
#define NAU8824_FEPGA_MODEL_HSMIC_EN	(0x1 << NAU8824_FEPGA_MODEL_HSMIC_SFT)

/* FEPGA_II (0x78) */
#define NAU8824_FEPGA_GAINR_SFT	5
#define NAU8824_FEPGA_GAINR_MASK	(0x1f << NAU8824_FEPGA_GAINR_SFT)
#define NAU8824_FEPGA_GAINL_SFT	0
#define NAU8824_FEPGA_GAINL_MASK	0x1f

/* CHARGE_PUMP_CONTROL (0x80) */
#define NAU8824_JAMNODCLOW		(0x1 << 15)
#define NAU8824_SPKR_PULL_DOWN	(0x1 << 13)
#define NAU8824_SPKL_PULL_DOWN	(0x1 << 12)
#define NAU8824_POWER_DOWN_DACR	(0x1 << 9)
#define NAU8824_POWER_DOWN_DACL	(0x1 << 8)
#define NAU8824_CHARGE_PUMP_EN_SFT	5
#define NAU8824_CHARGE_PUMP_EN	(0x1 << NAU8824_CHARGE_PUMP_EN_SFT)


#define NAU8824_CODEC_DAI "nau8824-hifi"

/* System Clock Source */
enum {
	NAU8824_CLK_DIS,
	NAU8824_CLK_MCLK,
	NAU8824_CLK_INTERNAL,
	NAU8824_CLK_FLL_MCLK,
	NAU8824_CLK_FLL_BLK,
	NAU8824_CLK_FLL_FS,
};

struct nau8824 {
	struct device *dev;
	struct regmap *regmap;
	struct snd_soc_dapm_context *dapm;
	struct snd_soc_jack *jack;
	struct work_struct jdet_work;
	struct semaphore jd_sem;
	int fs;
	int irq;
	int micbias_voltage;
	int vref_impedance;
	int jkdet_polarity;
	int sar_threshold_num;
	int sar_threshold[8];
	int sar_hysteresis;
	int sar_voltage;
	int sar_compare_time;
	int sar_sampling_time;
	int key_debounce;
	int jack_eject_debounce;
};

struct nau8824_fll {
	int mclk_src;
	int ratio;
	int fll_frac;
	int fll_int;
	int clk_ref_div;
};

struct nau8824_fll_attr {
	unsigned int param;
	unsigned int val;
};

struct nau8824_osr_attr {
	unsigned int osr;
	unsigned int clk_src;
};


int nau8824_enable_jack_detect(struct snd_soc_component *component,
	struct snd_soc_jack *jack);
const char *nau8824_components(void);

#endif				/* _NAU8824_H */


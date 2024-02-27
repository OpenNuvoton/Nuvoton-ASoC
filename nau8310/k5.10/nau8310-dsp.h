/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * The NAU83G10/20 DSP driver.
 *
 * Copyright 2021 Nuvoton Technology Crop.
 * Author: John Hsu <KCHSU0@nuvoton.com>
 *         David Lin <ctlin0@nuvoton.com>
 */

#ifndef __NAU8310_DSP_H__
#define __NAU8310_DSP_H__

#define NAU8310_DSP_COMM_IDLE_WORD		0xf4f3f2f1
#define NAU8310_DSP_COMM_PREAMBLE		0xb2a1
#define NAU8310_DSP_DATA_BYTE			4
#define NAU8310_DSP_DATA_LEN			(NAU8310_DSP_DATA_BYTE << 3)
/* max bytes of data to transfer into DSP each time during the KCS setup */
#define NAU8310_DSP_KCS_TX_MAX			96
#define NAU8310_DSP_RETRY_MAX			3
#define NAU8310_DSP_KCS_DAT_LEN_BITS		10
#define NAU8310_DSP_KCS_DAT_LEN_MAX		((1 << NAU8310_DSP_KCS_DAT_LEN_BITS) - 1)
#define NAU8310_DSP_KCS_OFFSET_MAX		3072

/* FRAME_STATUS (0x9) */
#define NAU8310_DSP_SNS_OVF_SFT			31
#define NAU8310_DSP_SNS_OVF			(0x1 << NAU8310_DSP_SNS_OVF_SFT)
#define NAU8310_DSP_AUD_UVF_SFT			30
#define NAU8310_DSP_AUD_UVF			(0x1 << NAU8310_DSP_AUD_UVF_SFT)
#define NAU8310_DSP_AUD_OVF_SFT			29
#define NAU8310_DSP_AUD_OVF			(0x1 << NAU8310_DSP_AUD_OVF_SFT)
#define NAU8310_DSP_ALC_STS_SFT			28
#define NAU8310_DSP_ALC_STS			(0x1 << NAU8310_DSP_ALC_STS_SFT)
#define NAU8310_DSP_CLK_STOP_SFT		25
#define NAU8310_DSP_CLK_STOP			(0x1 << NAU8310_DSP_CLK_STOP_SFT)
#define NAU8310_DSP_OCP_OTP_SFT			24
#define NAU8310_DSP_OCP_OTP			(0x1 << NAU8310_DSP_OCP_OTP_SFT)
#define NAU8310_DSP_UVLO_SFT			22
#define NAU8310_DSP_UVLO			(0x1 << NAU8310_DSP_UVLO_SFT)
#define NAU8310_DSP_OVP_SFT			21
#define NAU8310_DSP_OVP				(0x1 << NAU8310_DSP_OVP_SFT)
#define NAU8310_DSP_APWR_DWN_SFT		20
#define NAU8310_DSP_APWR_DWN			(0x1 << NAU8310_DSP_APWR_DWN_SFT)
#define NAU8310_DSP_SNSR_RATE_SFT		12
#define NAU8310_DSP_SNSR_RATE_MASK		(0xff << NAU8310_DSP_SNSR_RATE_SFT)
#define NAU8310_DSP_AUD_RATE_SFT		4
#define NAU8310_DSP_AUD_RATE_MASK		(0xff << NAU8310_DSP_AUD_RATE_SFT)
#define NAU8310_DSP_FEED_TRU_SFT		1
#define NAU8310_DSP_FEED_TRU			(0x1 << NAU8310_DSP_FEED_TRU_SFT)
#define NAU8310_DSP_ALGO_OK			0x1

#define NAU8310_DSP_FIRMWARE			"Nuvoton/NAU83G10.kcs.bin"
#define NAU8310_DSP_FWHDR_SIZE			8
#define NAU8310_DSP_FWHDR_SIREV			4
#define NAU8310_DSP_FW_KCS_DATA			NAU8310_DSP_FWHDR_SIZE

enum {
	NAU8310_DSP_REPLY_OK,
	NAU8310_DSP_REPLY_MSG_INTEGRETY_ERR,
	NAU8310_DSP_REPLY_EXECUTION_ERR,
	NAU8310_DSP_REPLY_COMMAND_DOESNT_EXISTS_ERR,
	NAU8310_DSP_REPLY_UNKNOWN_ERR,
	NAU8310_DSP_REPLY_MSG_TOO_LONG,
};

enum {
	NAU8310_DSP_CMD_GET_COUNTER		= 0x1,
	NAU8310_DSP_CMD_GET_FRAME_STATUS	= 0x9,
	NAU8310_DSP_CMD_GET_REVISION		= 0xa,
	NAU8310_DSP_CMD_GET_KCS_RSLTS		= 0x4,
	NAU8310_DSP_CMD_GET_KCS_SETUP		= 0x6,
	NAU8310_DSP_CMD_SET_KCS_SETUP		= 0x7,
	NAU8310_DSP_CMD_CLK_STOP		= 0xb,
	NAU8310_DSP_CMD_CLK_RESTART		= 0xc,
};

struct nau8310_cmd_info {
	int cmd_id;
	/* parameters include offset, size, and data */
	bool msg_param;
	/* data for write only */
	bool setup_data;
	/* get data from dsp */
	bool reply_data;
};

/**
 * set_len: length bytes of data writen
 * set_kcs_offset: address offset relative to set KCS start
 * set_kcs_data: address of data writen to KCS
 * get_len: length bytes from reply
 * get_data: address for reply data
 */
struct nau8310_kcs_setup {
	int set_len;
	int set_kcs_offset;
	void *set_kcs_data;
	int get_len;
	void *get_data;
};

int nau8310_dsp_init(struct snd_soc_component *component);
int nau8310_dsp_resume(struct snd_soc_component *component);

#endif /* __NAU8310_DSP_H__ */

/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * The NAU83G60 DSP driver.
 *
 * Copyright (C) 2025 Nuvoton Technology Crop.
 * Author: David Lin <ctlin0@nuvoton.com>
 *        Seven Lee <wtli@nuvoton.com>
 *        John Hsu <kchsu0@nuvoton.com>
 */

#ifndef __NAU8360_DSP_H__
#define __NAU8360_DSP_H__

#define NAU8360_DSP_COMM_IDLE_WORD		0xf4f3f2f1
#define NAU8360_DSP_COMM_PREAMBLE		0xb2a1
#define NAU8360_DSP_DATA_BYTE			4
#define NAU8360_DSP_DATA_LEN			(NAU8360_DSP_DATA_BYTE << 3)
/* max bytes of data to transfer into DSP each time during the KCS setup */
#define NAU8360_DSP_KCS_TX_MAX			96
#define NAU8360_DSP_RETRY_MAX			3
#define NAU8360_DSP_KCS_DAT_LEN_MAX		1024
#define NAU8360_DSP_KCS_OFFSET_MAX		3072

/* FRAME_STATUS (0x9) */
#define NAU8360_DSP_SNS_OVF_SFT			31
#define NAU8360_DSP_SNS_OVF			(0x1 << NAU8360_DSP_SNS_OVF_SFT)
#define NAU8360_DSP_AUD_UVF_SFT			30
#define NAU8360_DSP_AUD_UVF			(0x1 << NAU8360_DSP_AUD_UVF_SFT)
#define NAU8360_DSP_AUD_OVF_SFT			29
#define NAU8360_DSP_AUD_OVF			(0x1 << NAU8360_DSP_AUD_OVF_SFT)
#define NAU8360_DSP_ALC_STS_SFT			28
#define NAU8360_DSP_ALC_STS			(0x1 << NAU8360_DSP_ALC_STS_SFT)
#define NAU8360_DSP_CLK_STOP_SFT		25
#define NAU8360_DSP_CLK_STOP			(0x1 << NAU8360_DSP_CLK_STOP_SFT)
#define NAU8360_DSP_OCP_OTP_SFT			24
#define NAU8360_DSP_OCP_OTP			(0x1 << NAU8360_DSP_OCP_OTP_SFT)
#define NAU8360_DSP_UVLO_SFT			22
#define NAU8360_DSP_UVLO			(0x1 << NAU8360_DSP_UVLO_SFT)
#define NAU8360_DSP_OVP_SFT			21
#define NAU8360_DSP_OVP				(0x1 << NAU8360_DSP_OVP_SFT)
#define NAU8360_DSP_APWR_DWN_SFT		20
#define NAU8360_DSP_APWR_DWN			(0x1 << NAU8360_DSP_APWR_DWN_SFT)
#define NAU8360_DSP_SNSR_RATE_SFT		12
#define NAU8360_DSP_SNSR_RATE_MASK		(0xff << NAU8360_DSP_SNSR_RATE_SFT)
#define NAU8360_DSP_AUD_RATE_SFT		4
#define NAU8360_DSP_AUD_RATE_MASK		(0xff << NAU8360_DSP_AUD_RATE_SFT)
#define NAU8360_DSP_FEED_TRU_SFT		1
#define NAU8360_DSP_FEED_TRU			(0x1 << NAU8360_DSP_FEED_TRU_SFT)
#define NAU8360_DSP_ALGO_OK			0x1

#define NAU8360_DSP_FIRMDIR			"Nuvoton/"
#define NAU8360_DSP_FIRMWARE			NAU8360_DSP_FIRMDIR"NAU83G60.kcs.bin"
#define NAU8360_DSP_FW_NUM			NAU8360_DSP_CORE_NUM
#define NAU8360_DSP_FW_NAMELEN			64

enum {
	NAU8360_DSP_REPLY_OK,
	NAU8360_DSP_REPLY_MSG_INTEGRETY_ERR,
	NAU8360_DSP_REPLY_EXECUTION_ERR,
	NAU8360_DSP_REPLY_COMMAND_DOESNT_EXISTS_ERR,
	NAU8360_DSP_REPLY_UNKNOWN_ERR,
	NAU8360_DSP_REPLY_MSG_TOO_LONG,
};

enum {
	NAU8360_DSP_CMD_GET_COUNTER	= 0x1,
	NAU8360_DSP_CMD_GET_FRAME_STATUS	= 0x9,
	NAU8360_DSP_CMD_GET_REVISION	= 0xa,
	NAU8360_DSP_CMD_GET_KCS_RSLTS	= 0x4,
	NAU8360_DSP_CMD_GET_KCS_SETUP	= 0x6,
	NAU8360_DSP_CMD_SET_KCS_SETUP	= 0x7,
	NAU8360_DSP_CMD_CLK_STOP	= 0xb,
	NAU8360_DSP_CMD_CLK_RESTART	= 0xc,
};

struct nau8360_cmd_info {
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
struct nau8360_kcs_setup {
	int set_len;
	int set_kcs_offset;
	void *set_kcs_data;
	int get_len;
	void *get_data;
};

int nau8360_dsp_init(struct snd_soc_component *component);
int nau8360_dsp_reinit(struct snd_soc_component *component);

#endif /* __NAU8360_DSP_H__ */

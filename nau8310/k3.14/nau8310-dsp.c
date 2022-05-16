/*
 * The NAU83G10/20 DSP driver.
 *
 * Copyright 2021 Nuvoton Technology Corp.
 *
 * Author: John Hsu <kchsu0@nuvoton.com>
 *         David Lin <ctlin0@nuvoton.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <sound/core.h>
#include <sound/initval.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/tlv.h>

#include "nau8310.h"
#include "nau8310-dsp.h"

#define NAU8310_DSP_IDLE_RETRY 10

static int nau8310_dsp_set_kcs_setup(struct snd_soc_codec *codec, bool nowait);

static const struct nau8310_cmd_info nau8310_dsp_cmd_table[] = {
	[NAU8310_DSP_CMD_GET_COUNTER] = {
		.cmd_id = NAU8310_DSP_CMD_GET_COUNTER,
		.reply_data = 1,
	},
	[NAU8310_DSP_CMD_GET_FRAME_STATUS] = {
		.cmd_id = NAU8310_DSP_CMD_GET_FRAME_STATUS,
		.reply_data = 1,
	},
	[NAU8310_DSP_CMD_GET_REVISION] = {
		.cmd_id = NAU8310_DSP_CMD_GET_REVISION,
		.reply_data = 1,
	},
	[NAU8310_DSP_CMD_GET_KCS_RSLTS] = {
		.cmd_id = NAU8310_DSP_CMD_GET_KCS_RSLTS,
		.msg_param = 1,
		.reply_data = 1,
	},
	[NAU8310_DSP_CMD_GET_KCS_SETUP] = {
		.cmd_id = NAU8310_DSP_CMD_GET_KCS_SETUP,
		.msg_param = 1,
		.reply_data = 1,
	},
	[NAU8310_DSP_CMD_SET_KCS_SETUP] = {
		.cmd_id = NAU8310_DSP_CMD_SET_KCS_SETUP,
		.msg_param = 1,
		.setup_data = 1,
	},
	[NAU8310_DSP_CMD_CLK_STOP] = {
		.cmd_id = NAU8310_DSP_CMD_CLK_STOP,
	},
	[NAU8310_DSP_CMD_CLK_RESTART] = {
		.cmd_id = NAU8310_DSP_CMD_CLK_RESTART,
	},
};

static bool nau8310_dsp_commands(int cmd_id)
{
	switch (cmd_id) {
	case NAU8310_DSP_CMD_GET_COUNTER:
	case NAU8310_DSP_CMD_GET_FRAME_STATUS:
	case NAU8310_DSP_CMD_GET_REVISION:
	case NAU8310_DSP_CMD_GET_KCS_RSLTS:
	case NAU8310_DSP_CMD_GET_KCS_SETUP:
	case NAU8310_DSP_CMD_SET_KCS_SETUP:
	case NAU8310_DSP_CMD_CLK_STOP:
	case NAU8310_DSP_CMD_CLK_RESTART:
		return true;
	default:
		return false;
	}
}

static const char * const dsp_cmd_table[] = {
	[NAU8310_DSP_CMD_GET_COUNTER] = "GET_COUNTER",
	[NAU8310_DSP_CMD_GET_FRAME_STATUS] = "GET_FRAME_STATUS",
	[NAU8310_DSP_CMD_GET_REVISION] = "GET_REVISION",
	[NAU8310_DSP_CMD_GET_KCS_RSLTS] = "GET_KCS_RSLTS",
	[NAU8310_DSP_CMD_GET_KCS_SETUP] = "GET_KCS_SETUP",
	[NAU8310_DSP_CMD_SET_KCS_SETUP] = "SET_KCS_SETUP",
	[NAU8310_DSP_CMD_CLK_STOP] = "CLK_STOP",
	[NAU8310_DSP_CMD_CLK_RESTART] = "CLK_RESTART",
};

static void nau8310_sw_reset_chip(struct regmap *regmap)
{
	regmap_write(regmap, NAU8310_R01_SOFTWARE_RST, 0x00);
	regmap_write(regmap, NAU8310_R01_SOFTWARE_RST, 0x00);
}

/* checking for DSP IDLE pattern */
static int nau8310_dsp_idle(struct snd_soc_codec *codec)
{
	struct nau8310 *nau8310 = snd_soc_codec_get_drvdata(codec);
	u8 buf[4];
	unsigned int idle_pattern;
	int ret, retries;

	for (retries = NAU8310_DSP_IDLE_RETRY; retries > 0; retries--) {
		ret = regmap_read(nau8310->regmap, NAU8310_RF000_DSP_COMM,
				  &idle_pattern);
		if (ret) {
			dev_err(codec->dev, "Failed to read dsp status\n");
			return ret;
		}
		if (idle_pattern == NAU8310_DSP_COMM_IDLE_WORD) {
			dev_dbg(codec->dev, "Idle pattern found\n");
			break;
		}
	}
	/* The driver can't establish a connection to DSP.
	 * Maybe it is not clocked, or previous synchronization issue.
	 */
	if (retries == 0) {
		dev_err(codec->dev, "Timeout for idle pattern\n");
		return -EIO;
	}

	*(unsigned int *)&buf[0] = idle_pattern;

	dev_dbg(codec->dev, "[R] %02x %02x %02x %02x\n",
		buf[0], buf[1], buf[2], buf[3]);

	return 0;
}

static int nau8310_massage_to_dsp(struct snd_soc_codec *codec,
				  const struct nau8310_cmd_info *cmd_info, int frag_len,
				  int param_offset, int param_size, void *param_data)
{
	struct nau8310 *nau8310 = snd_soc_codec_get_drvdata(codec);
	u8 data[4], *b_data;
	unsigned int *value = (unsigned int *)data;
	unsigned int preamble = NAU8310_DSP_COMM_PREAMBLE;
	int ret, i, data_size, padding = 0, frag_cnt = 0;

	ret = nau8310_dsp_idle(codec);
	if (ret)
		goto err;

	/* sending preamble fragment */
	data[0] = preamble;
	data[1] = preamble >> 8;
	data[2] = (cmd_info->cmd_id << 2) | (frag_len & 0x3);
	data[3] = frag_len >> 2;
	regmap_write(nau8310->regmap, NAU8310_RF000_DSP_COMM, *value);

	dev_dbg(codec->dev, "Sending preamble fragment (CMD_ID 0x%x, LEN 0x%x)\n",
		cmd_info->cmd_id, frag_len);
	dev_dbg(codec->dev, "[W] %02x %02x %02x %02x\n",
		data[0], data[1], data[2], data[3]);

	if (!cmd_info->msg_param)
		goto done;

	/* sending payload + padding */
	*value = 0;
	data[0] = param_offset;
	data[1] = param_offset >> 8;
	data[2] = param_size;
	data[3] = param_size >> 8;
	regmap_write(nau8310->regmap, NAU8310_RF000_DSP_COMM, *value);
	frag_cnt++;

	dev_dbg(codec->dev, "Sending parameters fragment (offset 0x%x, size 0x%x)\n",
		param_offset, param_size);
	dev_dbg(codec->dev, "[W] %02x %02x %02x %02x\n",
		data[0], data[1], data[2], data[3]);

	if (cmd_info->setup_data) {
		b_data = (u8 *)param_data;
		for (data_size = 0, *value = 0, i = 0; i < param_size; i++) {
			data[i % NAU8310_DSP_DATA_BYTE] = b_data[i];
			data_size++;
			if (data_size == NAU8310_DSP_DATA_BYTE) {
				regmap_write(nau8310->regmap,
					     NAU8310_RF000_DSP_COMM, *value);
				data_size = 0;
				*value = 0;
				frag_cnt++;

				dev_dbg(codec->dev, "[W] %02x %02x %02x %02x\n",
					data[0], data[1], data[2], data[3]);
			}
		}
		if (data_size > 0 && data_size < NAU8310_DSP_DATA_BYTE) {
			/* sending the data fragments with padding bytes */
			padding = NAU8310_DSP_DATA_BYTE - data_size;
			regmap_write(nau8310->regmap,
				     NAU8310_RF000_DSP_COMM, *value);
			*value = 0;
			frag_cnt++;

			dev_dbg(codec->dev, "[W] %02x %02x %02x %02x\n",
				data[0], data[1], data[2], data[3]);
		}
		dev_dbg(codec->dev, "\n");
	}

	/* sending trailing fragment */
	frag_cnt++;
	*value = 0;
	data[0] = frag_cnt;
	data[1] = ((frag_cnt >> 8) << 6) | (padding << 4);
	regmap_write(nau8310->regmap, NAU8310_RF000_DSP_COMM, *value);

	dev_dbg(codec->dev,	"Sending trailing fragment (LEN 0x%x, PAD 0x%x)\n",
		frag_cnt, padding);
	dev_dbg(codec->dev, "[W] %02x %02x %02x %02x\n",
		data[0], data[1], data[2], data[3]);

	if (frag_cnt != frag_len) {
		dev_err(codec->dev,	"Sending massage error (CMD_ID 0x%x, LEN 0x%x) !!!\n",
			cmd_info->cmd_id, frag_cnt);
		ret = -EPROTO;
		goto err;
	}

done:
	return 0;
err:
	return ret;
}

static int nau8310_dsp_replied(struct snd_soc_codec *codec, int *length)
{
	struct nau8310 *nau8310 = snd_soc_codec_get_drvdata(codec);
	u8 buf[4];
	const u8 *b = buf;
	unsigned int reply_preamble;
	int ret, retries, reply_id;

	for (retries = 10; retries > 0; retries--) {
		ret = regmap_read(nau8310->regmap, NAU8310_RF000_DSP_COMM,
				  &reply_preamble);
		if (ret) {
			dev_err(codec->dev, "Failed to read reply preamble of dsp\n");
			return ret;
		}
		/* check for preamble */
		*(unsigned int *)&buf[0] = reply_preamble;
		if (b[0] == (NAU8310_DSP_COMM_PREAMBLE & 0xff) &&
			b[1] == (NAU8310_DSP_COMM_PREAMBLE >> 8)) {
			*length = b[2] & 0x3;
			*length |= b[3] << 2;
			reply_id = b[2] >> 2;
			break;
		}
	}
	if (retries == 0) {
		dev_err(codec->dev, "Timeout for reply preamble\n");
		return -EIO;
	}

	dev_dbg(codec->dev,	"Receiving preamble fragment (REPLY_ID 0x%x, LEN 0x%x)\n",
		reply_id, *length);
	dev_dbg(codec->dev, "[R] %02x %02x %02x %02x\n",
		b[0], b[1], b[2], b[3]);

	if (reply_id == NAU8310_DSP_REPLY_OK)
		return 0;
	else
		return -reply_id;
}

static int nau8310_reply_from_dsp(struct snd_soc_codec *codec,
	const struct nau8310_cmd_info *cmd_info, int data_size, void *data)
{
	struct nau8310 *nau8310 = snd_soc_codec_get_drvdata(codec);
	u8 buf[4], *b_data;
	const u8 *b = buf;
	unsigned int payload, *data_buf;
	int i, j, ret, frag_len, frag_payload_len;
	int data_count, len_pos, pad_len, pad_len_exp;

	if (!data) {
		ret = -EINVAL;
		goto err;
	}
	data_buf = (unsigned int *)data;

	ret = nau8310_dsp_replied(codec, &frag_len);
	if (ret)
		goto err;
	else if (frag_len == 0)
		goto done;

	frag_payload_len = frag_len - 1;
	if (cmd_info->msg_param)
		data_count = data_size;
	for (i = 0; i < frag_payload_len; i++) {
		ret = regmap_read(nau8310->regmap, NAU8310_RF000_DSP_COMM, &payload);
		if (ret) {
			dev_err(codec->dev, "Failed to read payload of dsp\n");
			goto err;
		}
		if (cmd_info->msg_param) {
			if (data_count >= NAU8310_DSP_DATA_BYTE) {
				*data_buf++ = payload;
				data_count -= NAU8310_DSP_DATA_BYTE;
				*(unsigned int *)&buf[0] = payload;

				dev_dbg(codec->dev, "[R] %02x %02x %02x %02x\n",
					buf[0], buf[1], buf[2], buf[3]);
			} else {
				*(unsigned int *)&buf[0] = payload;
				b_data = (u8 *)data_buf;
				for (j = 0; j < NAU8310_DSP_DATA_BYTE; j++) {
					b_data[j] = b[j];
					data_count--;
					if (data_count <= 0)
						break;
				}

				dev_dbg(codec->dev, "[R] %02x %02x %02x %02x\n",
					buf[0], buf[1], buf[2], buf[3]);
				break;
			}
		} else {
			*(unsigned int *)&buf[0] = payload;
			*data_buf = b[0];
			*data_buf |= b[1] << 8;
			*data_buf |= b[2] << 16;
			*data_buf |= b[3] << 24;

			dev_dbg(codec->dev, "[R] %02x %02x %02x %02x\n",
				buf[0], buf[1], buf[2], buf[3]);
		}
	}

	/* check the reply length same as request */
	if (data_count && (cmd_info->cmd_id == NAU8310_DSP_CMD_GET_KCS_RSLTS ||
		cmd_info->cmd_id == NAU8310_DSP_CMD_GET_KCS_SETUP)) {
		dev_warn(codec->dev, "payload_len = %d, expected %d\n",
			 data_size - data_count, data_size);
	}

	dev_dbg(codec->dev, "Reading trailing fragment\n");

	ret = regmap_read(nau8310->regmap, NAU8310_RF000_DSP_COMM, &payload);
	if (ret) {
		dev_err(codec->dev, "Failed to read trailing fragment of dsp\n");
		goto err;
	}
	*(unsigned int *)&buf[0] = payload;
	len_pos = b[0];
	len_pos |= (b[1] & 0xc0) << 2;
	if (len_pos != frag_len) {
		dev_err(codec->dev, "LEN_POST = %02X, expected %02X\n",
			len_pos, frag_len);
		ret = -EPROTO;
		goto err;
	}
	pad_len = (b[1] & 0x30) >> 4;
	if (cmd_info->msg_param) {
		pad_len_exp = frag_payload_len * NAU8310_DSP_DATA_BYTE -
			(data_size - data_count);
	} else {
		pad_len_exp = 0;
	}
	if (pad_len != pad_len_exp) {
		dev_err(codec->dev, "PAD_LEN = %02X, expected %02X\n",
			pad_len, pad_len_exp);
		ret = -EPROTO;
		goto err;
	}

	dev_dbg(codec->dev, "LEN_POST = 0x%x, PAD_LEN 0x%x\n",
		len_pos, pad_len);
	dev_dbg(codec->dev, "[R] %02x %02x %02x %02x\n",
		buf[0], buf[1], buf[2], buf[3]);

done:
	return 0;
err:
	dev_err(codec->dev, "DSP reply error %d !!!\n", ret);
	return ret;
}

/**
 * nau8310_send_dsp_command - Send command to DSP
 *
 * @codec:  codec to register
 * @cmd_id:  DSP supported command ID
 * @kcs_setup: KCS setup structure
 *
 * The communication protocol is a Master-Slave type protocol
 * where the host processor is the master and DSP is the slave.
 * The Master initiates the communication and can either write or
 * read back from the slave.
 * Transactions from the Master are called "Messages",
 * and read-back data from the Slave is called a "Reply".
 *
 * The function sends command to DSP according to the command ID.
 * These commands include getting the information of DSP,
 * getting or setting KCS configuration, or making DSP control.
 */
int nau8310_send_dsp_command(struct snd_soc_codec *codec,
			     int cmd_id, struct nau8310_kcs_setup *kcs_setup)
{
	const struct nau8310_cmd_info *cmd_info;
	int ret, frag_len = 0;

	if (!codec)
		return -EINVAL;

	if (!kcs_setup) {
		ret = -EINVAL;
		goto msg_fail;
	}
	if (!nau8310_dsp_commands(cmd_id)) {
		dev_err(codec->dev, "Command not support!\n");
		ret = -EINVAL;
		goto msg_fail;
	}

	cmd_info = &nau8310_dsp_cmd_table[cmd_id];
	if ((cmd_info->msg_param && !kcs_setup->set_len) ||
		(cmd_info->setup_data && !kcs_setup->set_kcs_data) ||
		(cmd_info->reply_data && !kcs_setup->get_data)) {
		ret = -EINVAL;
		goto msg_fail;
	}
	/* Read up to 1kB data because the LEN field to request data is 10-bits
	 * long; and not beyond 3kB offset.
	 */
	if (cmd_id == NAU8310_DSP_CMD_GET_KCS_SETUP &&
		(kcs_setup->set_len > NAU8310_DSP_KCS_DAT_LEN_MAX ||
		kcs_setup->set_kcs_offset > NAU8310_DSP_KCS_OFFSET_MAX)) {
		ret = -EINVAL;
		goto msg_fail;
	}

	if (cmd_info->msg_param) {
		/* one fragment for offset and size parameters */
		frag_len++;
		/* one fragment for a postamble fragment */
		frag_len++;
	}

	/* fragments for KCS setup writen */
	if (cmd_info->setup_data)
		frag_len += (kcs_setup->set_len +
			NAU8310_DSP_DATA_BYTE - 1) / NAU8310_DSP_DATA_BYTE;
	ret = nau8310_massage_to_dsp(codec, cmd_info, frag_len,
		kcs_setup->set_kcs_offset, kcs_setup->set_len,
		kcs_setup->set_kcs_data);
	if (ret)
		goto msg_fail;

	ret = nau8310_reply_from_dsp(codec, cmd_info,
		kcs_setup->get_len, kcs_setup->get_data);
	if (ret)
		goto reply_fail;

	return 0;

msg_fail:
	dev_err(codec->dev, "Fail to send a message(%d) to dsp, ret %d.\n",
		cmd_id, ret);
	return ret;
reply_fail:
	dev_err(codec->dev, "Reply fail (%d) from dsp.\n", ret);
	return ret;
}

/**
 * nau8310_dsp_kcs_setup - Send KCS setup command to DSP
 *
 * @codec:  codec to register
 * @offset: address offset relative to KCS start
 * @size: size of data writen to KCS
 * @data: data writen to KCS setup
 *
 * The function sends KCS setup command to DSP for
 * setting KCS configuration. The maximum size that you can transfer into
 * the DSP is 96 bytes. Therefore, the driver has to split the data into
 * 96 bytes chucks, if the setup configuration over the threshold.
 */
int nau8310_dsp_kcs_setup(struct snd_soc_codec *codec,
			  int offset, int size, const void *data)
{
	struct nau8310_kcs_setup kcs_setup_comp, *kcs_setup = &kcs_setup_comp;
	u8 *data_buf;
	int cmd_id = NAU8310_DSP_CMD_SET_KCS_SETUP;
	int retries, ret, data_len, data_rem, addr_offset;
	unsigned int kcs_rst;

	/* Limit full load of KCS_SETUP data and not beyond 3kB offset. */
	if (!data || size > NAU8310_DSP_KCS_DAT_LEN_MAX ||
		offset > NAU8310_DSP_KCS_OFFSET_MAX) {
		ret = -EINVAL;
		goto msg_fail;
	}

	/* sending fragments for KCS setup */
	data_buf = (u8 *)data;
	addr_offset = offset;
	data_rem = size;
	retries = 0;
	kcs_setup->get_data = &kcs_rst;
	while (data_rem) {
		if (data_rem > NAU8310_DSP_KCS_TX_MAX)
			data_len = NAU8310_DSP_KCS_TX_MAX;
		else
			data_len = data_rem;
		cmd_id = NAU8310_DSP_CMD_SET_KCS_SETUP;
		kcs_setup->set_kcs_offset = addr_offset;
		kcs_setup->set_len = data_len;
		kcs_setup->set_kcs_data = data_buf;
		ret = nau8310_send_dsp_command(codec, cmd_id, kcs_setup);
		if (ret < 0) {
			if (retries++ < NAU8310_DSP_RETRY_MAX)
				continue;
			else
				goto msg_fail;
		} else {
			data_buf += (u8)data_len;
			addr_offset += data_len;
			data_rem -= data_len;
		}
		/* checking KCS result */
		cmd_id = NAU8310_DSP_CMD_GET_KCS_RSLTS;
		kcs_setup->set_kcs_offset = 0;
		kcs_setup->set_len = NAU8310_DSP_DATA_BYTE;
		kcs_setup->get_len = kcs_setup->set_len;
		ret = nau8310_send_dsp_command(codec, cmd_id, kcs_setup);
		if (ret < 0)
			goto msg_fail;
	}

	return 0;

msg_fail:
	dev_err(codec->dev, "Fail to send a kcs setup message(%d) to dsp, ret %d.\n",
		cmd_id, ret);
	return ret;
}

static int nau8310_dsp_get_counter_put(struct snd_kcontrol *kcontrol,
				       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct nau8310_kcs_setup kcs_setup_comp, *kcs_setup = &kcs_setup_comp;
	int ret, counter;

	dev_info(codec->dev, "Send DSP command %s\n",
		 dsp_cmd_table[NAU8310_DSP_CMD_GET_COUNTER]);

	kcs_setup->get_len = kcs_setup->set_len = sizeof(counter);
	kcs_setup->get_data = (void *)&counter;
	ret = nau8310_send_dsp_command(codec,
				       NAU8310_DSP_CMD_GET_COUNTER, kcs_setup);
	if (ret) {
		dev_err(codec->dev, "Send DSP command %s fail (%d)\n",
			dsp_cmd_table[NAU8310_DSP_CMD_GET_COUNTER], ret);
	} else {
		dev_info(codec->dev, "DSP counter %d\n", counter);
	}

	return 0;
}

static int nau8310_dsp_get_revision_put(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct nau8310_kcs_setup kcs_setup_comp, *kcs_setup = &kcs_setup_comp;
	int ret, version;

	dev_info(codec->dev, "Send DSP command %s\n",
		 dsp_cmd_table[NAU8310_DSP_CMD_GET_REVISION]);
	kcs_setup->get_len = kcs_setup->set_len = sizeof(version);
	kcs_setup->get_data = (void *)&version;
	ret = nau8310_send_dsp_command(codec,
				       NAU8310_DSP_CMD_GET_REVISION, kcs_setup);
	if (ret) {
		dev_err(codec->dev, "Send DSP command %s fail (%d)\n",
			dsp_cmd_table[NAU8310_DSP_CMD_GET_REVISION], ret);
	} else {
		dev_info(codec->dev, "DSP version %x\n", version);
	}

	return 0;
}

static int nau8310_dsp_get_frame_status_put(struct snd_kcontrol *kcontrol,
					    struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct nau8310_kcs_setup kcs_setup_comp, *kcs_setup = &kcs_setup_comp;
	int ret, status;

	dev_info(codec->dev, "Send DSP command %s\n",
		 dsp_cmd_table[NAU8310_DSP_CMD_GET_FRAME_STATUS]);
	kcs_setup->get_len = kcs_setup->set_len = sizeof(status);
	kcs_setup->get_data = (void *)&status;
	ret = nau8310_send_dsp_command(codec,
				       NAU8310_DSP_CMD_GET_FRAME_STATUS, kcs_setup);
	if (ret) {
		dev_err(codec->dev, "Send DSP command %s fail (%d)\n",
			dsp_cmd_table[NAU8310_DSP_CMD_GET_FRAME_STATUS], ret);
	} else {
		dev_info(codec->dev, "DSP frame status %x\n", status);
	}

	return 0;
}

static int nau8310_dsp_get_kcs_setup_put(struct snd_kcontrol *kcontrol,
					 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct nau8310 *nau8310 = snd_soc_codec_get_drvdata(codec);
	struct nau8310_kcs_setup kcs_setup_comp, *kcs_setup = &kcs_setup_comp;
	int ret, i, buf_off, buf_len, count;
	char *data, buf[100];

	data = kcalloc(nau8310->kcs_setup_size, sizeof(char), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	if (nau8310->kcs_setup_size == 0) {
		kfree(data);
		ret = -EINVAL;
		dev_err(codec->dev, "KCS of DSP not load yet (%d)\n", ret);
		return ret;
	}
	buf_off = 0;
	buf_len = nau8310->kcs_setup_size;
	dev_info(nau8310->dev, "Send DSP command %s (OFF %d, LEN %d)\n",
		 dsp_cmd_table[NAU8310_DSP_CMD_GET_KCS_SETUP], buf_off, buf_len);
	kcs_setup->set_kcs_offset = buf_off;
	kcs_setup->get_len = kcs_setup->set_len = buf_len;
	kcs_setup->get_data = data;
	ret = nau8310_send_dsp_command(codec,
				       NAU8310_DSP_CMD_GET_KCS_SETUP, kcs_setup);
	if (ret) {
		dev_err(codec->dev, "Send DSP command %s fail (%d)\n",
			dsp_cmd_table[NAU8310_DSP_CMD_GET_KCS_SETUP], ret);
	} else {
		dev_dbg(codec->dev, "DSP KCS result:\n");
		for (i = 0; i < kcs_setup->get_len; i += 16) {
			dev_dbg(codec->dev,	"%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x",
				data[i], data[i+1], data[i+2], data[i+3],
				data[i+4], data[i+5], data[i+6], data[i+7],
				data[i+8], data[i+9], data[i+10], data[i+11],
				data[i+12], data[i+13], data[i+14], data[i+15]);
			if (kcs_setup->get_len - i < 16)
				break;
		}
		count = 0;
		for (; i < kcs_setup->get_len; i++)
			count += sprintf(buf + count, "%02x ", data[i]);
		dev_dbg(codec->dev, "%s", buf);
		dev_info(codec->dev, "Get length %d of kcs_setup\n",
			 kcs_setup->get_len);
	}
	kfree(data);

	return 0;
}

static int nau8310_dsp_set_kcs_setup_put(struct snd_kcontrol *kcontrol,
					 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct nau8310 *nau8310 = snd_soc_codec_get_drvdata(codec);

	nau8310_sw_reset_chip(nau8310->regmap);
	/* wait for the power ready */
	msleep(200);
	regmap_update_bits(nau8310->regmap, NAU8310_R1A_DSP_CORE_CTRL2,
			   NAU8310_DSP_RUNSTALL, 0);

	return nau8310_dsp_set_kcs_setup(codec, false);
}

static int nau8310_dsp_clk_restart_put(struct snd_kcontrol *kcontrol,
				       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct nau8310_kcs_setup kcs_setup_comp, *kcs_setup = &kcs_setup_comp;
	int ret;

	dev_info(codec->dev, "Send DSP command %s\n",
		 dsp_cmd_table[NAU8310_DSP_CMD_CLK_RESTART]);

	ret = nau8310_send_dsp_command(codec,
				       NAU8310_DSP_CMD_CLK_RESTART, kcs_setup);
	if (ret)
		dev_err(codec->dev, "Send DSP command %s fail (%d)\n",
			dsp_cmd_table[NAU8310_DSP_CMD_CLK_RESTART], ret);

	return 0;
}

static int nau8310_dsp_clk_stop_put(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct nau8310_kcs_setup kcs_setup_comp, *kcs_setup = &kcs_setup_comp;
	int ret;

	dev_info(codec->dev, "Send DSP command %s\n",
		 dsp_cmd_table[NAU8310_DSP_CMD_CLK_STOP]);

	ret = nau8310_send_dsp_command(codec,
				       NAU8310_DSP_CMD_CLK_STOP, kcs_setup);
	if (ret)
		dev_err(codec->dev, "Send DSP command %s fail (%d)\n",
			dsp_cmd_table[NAU8310_DSP_CMD_CLK_STOP], ret);

	return 0;
}

int nau8310_dsp_cmd_get(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = 1;

	return 0;
}

static const struct snd_kcontrol_new nau8310_dsp_snd_controls[] = {
	SOC_SINGLE_EXT("DSP get counter command", SND_SOC_NOPM, 0, 1, 0,
		       nau8310_dsp_cmd_get, nau8310_dsp_get_counter_put),
	SOC_SINGLE_EXT("DSP get frame status command", SND_SOC_NOPM, 0, 1, 0,
		       nau8310_dsp_cmd_get, nau8310_dsp_get_frame_status_put),
	SOC_SINGLE_EXT("DSP get revision command", SND_SOC_NOPM, 0, 1, 0,
		       nau8310_dsp_cmd_get, nau8310_dsp_get_revision_put),
	SOC_SINGLE_EXT("DSP get KCS setup command", SND_SOC_NOPM, 0, 1, 0,
		       nau8310_dsp_cmd_get, nau8310_dsp_get_kcs_setup_put),
	SOC_SINGLE_EXT("DSP set KCS setup command", SND_SOC_NOPM, 0, 1, 0,
		       nau8310_dsp_cmd_get, nau8310_dsp_set_kcs_setup_put),
	SOC_SINGLE_EXT("DSP clock restart command", SND_SOC_NOPM, 0, 1, 0,
		       nau8310_dsp_cmd_get, nau8310_dsp_clk_restart_put),
	SOC_SINGLE_EXT("DSP clock stop command", SND_SOC_NOPM, 0, 1, 0,
		       nau8310_dsp_cmd_get, nau8310_dsp_clk_stop_put),
};

static int nau8310_dsp_snd_soc_dapm_put(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct nau8310 *nau8310 = snd_soc_codec_get_drvdata(codec);
	struct soc_mixer_control *mc =
			(struct soc_mixer_control *)kcontrol->private_value;
	int reg = mc->reg;
	unsigned int shift = mc->shift;
	int max = mc->max;
	unsigned int mask = (1 << fls(max)) - 1;
	unsigned int val, orig, tmp;
	int connect, ret;

	ret = regmap_read(nau8310->regmap, reg, &orig);
	if (ret) {
		dev_err(codec->dev, "Failed to read register: %d\n", ret);
		return ret;
	}

	val = (ucontrol->value.integer.value[0] & mask);
	connect = !!val;
	mask = mask << shift;
	val = val << shift;
	tmp = orig & ~mask;
	tmp |= val & mask;
	if (tmp != orig && connect) {
		nau8310_sw_reset_chip(nau8310->regmap);
		/* wait for the power ready */
		msleep(200);
		regmap_update_bits(nau8310->regmap, NAU8310_R1A_DSP_CORE_CTRL2,
				   NAU8310_DSP_RUNSTALL, 0);
		ret = nau8310_dsp_set_kcs_setup(codec, false);
		if (ret)
			return -EINVAL;
	}

	return snd_soc_dapm_put_volsw(kcontrol, ucontrol);
}

#define NAU8310_DSP_SOC_DAPM_SINGLE(xname, reg, shift, mask, invert) \
	SOC_SINGLE_EXT(xname, reg, shift, mask, invert, \
		       snd_soc_dapm_get_volsw, nau8310_dsp_snd_soc_dapm_put)

static const struct snd_kcontrol_new nau8310_dacdat_select_dsp =
	NAU8310_DSP_SOC_DAPM_SINGLE("Switch", NAU8310_R1A_DSP_CORE_CTRL2,
				    NAU8310_DAC_SEL_DSP_SFT, 1, 0);

static int nau8310_dsp_clock_event(struct snd_soc_dapm_widget *w,
				   struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct nau8310 *nau8310 = snd_soc_codec_get_drvdata(codec);
	struct nau8310_kcs_setup kcs_setup_comp, *kcs_setup = &kcs_setup_comp;
	int ret;

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		dev_dbg(codec->dev, "Send DSP command %s\n",
			dsp_cmd_table[NAU8310_DSP_CMD_CLK_RESTART]);
		ret = nau8310_send_dsp_command(codec,
					       NAU8310_DSP_CMD_CLK_RESTART, kcs_setup);
		if (ret)
			dev_err(codec->dev, "Send DSP command %s fail (%d)\n",
				dsp_cmd_table[NAU8310_DSP_CMD_CLK_RESTART],
				ret);
		/* Switch the clock source of DSP to MCLK. */
		regmap_update_bits(nau8310->regmap, NAU8310_R04_ENA_CTRL,
				   NAU8310_DSP_SEL_OSC, 0);
		break;
	case SND_SOC_DAPM_POST_PMD:
		dev_dbg(codec->dev, "Send DSP command %s\n",
			dsp_cmd_table[NAU8310_DSP_CMD_CLK_STOP]);
		ret = nau8310_send_dsp_command(codec,
					       NAU8310_DSP_CMD_CLK_STOP, kcs_setup);
		if (ret)
			dev_err(codec->dev, "Send DSP command %s fail (%d)\n",
				dsp_cmd_table[NAU8310_DSP_CMD_CLK_STOP], ret);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static const struct snd_soc_dapm_widget nau8310_dsp_dapm_widgets[] = {
	SND_SOC_DAPM_PGA("DSP", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_SWITCH("DAC data from DSP", SND_SOC_NOPM,
			    0, 0, &nau8310_dacdat_select_dsp),
	SND_SOC_DAPM_SUPPLY("DSP Clock", SND_SOC_NOPM, 0, 0,
			    nau8310_dsp_clock_event, SND_SOC_DAPM_PRE_PMU |
			    SND_SOC_DAPM_POST_PMD),
};

static const struct snd_soc_dapm_route nau8310_dsp_dapm_routes[] = {
	{ "DSP", NULL, "Capture Sense" },
	{ "DAC data from DSP", "Switch", "DSP" },
	{ "DAC", NULL, "DAC data from DSP" },
	{ "DSP", NULL, "DSP Clock" },
};

static void nau8310_dsp_fw_cb(const struct firmware *fw, void *context)
{
	struct snd_soc_codec *codec = context;
	struct nau8310 *nau8310 = snd_soc_codec_get_drvdata(codec);
	struct snd_soc_dapm_context *dapm = nau8310->dapm;
	int ret, buf_off, buf_len;

	if (!fw) {
		dev_err(codec->dev, "Cannot load firmware %s\n",
			NAU8310_DSP_FIRMWARE);
		goto err;
	}
	regmap_update_bits(nau8310->regmap, NAU8310_R13_MUTE_CTRL,
			   NAU8310_SOFT_MUTE, NAU8310_SOFT_MUTE);

	buf_off = 0;
	buf_len = nau8310->kcs_setup_size = fw->size;

	dev_dbg(codec->dev, "Send DSP command %s (OFF %d, LEN %d)\n",
		dsp_cmd_table[NAU8310_DSP_CMD_SET_KCS_SETUP],
		buf_off, buf_len);

	ret = nau8310_dsp_kcs_setup(codec, buf_off, buf_len, fw->data);
	if (ret) {
		dev_err(codec->dev, "Send DSP command %s fail (%d)\n",
			dsp_cmd_table[NAU8310_DSP_CMD_SET_KCS_SETUP], ret);
		goto err;
	}
	regmap_update_bits(nau8310->regmap, NAU8310_R13_MUTE_CTRL,
			   NAU8310_SOFT_MUTE, 0);

	release_firmware(fw);

	return;
err:
	release_firmware(fw);
	snd_soc_dapm_del_routes(dapm, nau8310_dsp_dapm_routes,
				ARRAY_SIZE(nau8310_dsp_dapm_routes));
	regmap_update_bits(nau8310->regmap, NAU8310_R1A_DSP_CORE_CTRL2,
			   NAU8310_DAC_SEL_DSP_OUT, 0);
	regmap_update_bits(nau8310->regmap, NAU8310_R13_MUTE_CTRL,
			   NAU8310_SOFT_MUTE, 0);
}

static int nau8310_dsp_set_kcs_setup(struct snd_soc_codec *codec, bool nowait)
{
	struct nau8310 *nau8310 = snd_soc_codec_get_drvdata(codec);
	struct nau8310_kcs_setup kcs_setup_comp, *kcs_setup = &kcs_setup_comp;
	const struct firmware *fw;
	int ret, status = 0, buf_off, buf_len;

	dev_dbg(codec->dev, "Send DSP command %s\n",
		dsp_cmd_table[NAU8310_DSP_CMD_GET_FRAME_STATUS]);

	kcs_setup->get_len = kcs_setup->set_len = sizeof(status);
	kcs_setup->get_data = (void *)&status;

	ret = nau8310_send_dsp_command(codec,
				       NAU8310_DSP_CMD_GET_FRAME_STATUS, kcs_setup);
	if (ret) {
		dev_err(codec->dev, "Send DSP command %s fail (%d)\n",
			dsp_cmd_table[NAU8310_DSP_CMD_GET_FRAME_STATUS], ret);
		goto err;
	}

	if (!(status & NAU8310_DSP_ALGO_OK)) {
		dev_err(codec->dev, "Algorithm of DSP is not ready, status %x\n",
			status);
		ret = -EIO;
		goto err;
	}

	dev_info(codec->dev, "Algorithm of DSP is ready, status %x\n", status);

	if (nowait) {
		dev_dbg(codec->dev, "Request firmware and no wait.\n");
		ret = request_firmware_nowait(THIS_MODULE, true,
					      NAU8310_DSP_FIRMWARE,
					      codec->dev, GFP_KERNEL,
					      codec, nau8310_dsp_fw_cb);
		if (ret) {
			dev_err(codec->dev, "Failed to load firmware (%d)\n", ret);
			goto err;
		}
	} else {
		dev_dbg(codec->dev, "Request firmware and wait until finish.\n");
		ret = request_firmware(&fw, NAU8310_DSP_FIRMWARE, codec->dev);
		if (ret) {
			dev_err(codec->dev, "Failed to load firmware (%d)\n", ret);
			goto err;
		}
		regmap_update_bits(nau8310->regmap, NAU8310_R13_MUTE_CTRL,
				   NAU8310_SOFT_MUTE, NAU8310_SOFT_MUTE);

		buf_off = 0;
		buf_len = nau8310->kcs_setup_size = fw->size;
		dev_dbg(codec->dev, "Send DSP command %s (OFF %d, LEN %d)\n",
			dsp_cmd_table[NAU8310_DSP_CMD_SET_KCS_SETUP],
			buf_off, buf_len);
		ret = nau8310_dsp_kcs_setup(codec, buf_off, buf_len, fw->data);
		if (ret) {
			dev_err(codec->dev, "Send DSP command %s fail (%d)\n",
				dsp_cmd_table[NAU8310_DSP_CMD_SET_KCS_SETUP],
				ret);
			goto err_loaded;
		}
		regmap_update_bits(nau8310->regmap, NAU8310_R13_MUTE_CTRL,
				   NAU8310_SOFT_MUTE, 0);
		release_firmware(fw);
	}

	return 0;

err_loaded:
	regmap_update_bits(nau8310->regmap, NAU8310_R13_MUTE_CTRL,
			   NAU8310_SOFT_MUTE, 0);
	if (fw)
		release_firmware(fw);
err:
	return ret;
}

int nau8310_dsp_init(struct snd_soc_codec *codec)
{
	struct nau8310 *nau8310 = snd_soc_codec_get_drvdata(codec);
	struct snd_soc_dapm_context *dapm = nau8310->dapm;
	int ret;

	dev_dbg(codec->dev, "DSP initializing...\n");

	ret = nau8310_dsp_set_kcs_setup(codec, true);
	if (ret)
		goto err;

	ret = snd_soc_add_codec_controls(codec, nau8310_dsp_snd_controls,
					     ARRAY_SIZE(nau8310_dsp_snd_controls));
	if (ret) {
		dev_err(codec->dev, "Add DSP control fail (%d)\n", ret);
		goto err;
	}
	ret = snd_soc_dapm_new_controls(dapm, nau8310_dsp_dapm_widgets,
					ARRAY_SIZE(nau8310_dsp_dapm_widgets));
	if (ret) {
		dev_err(codec->dev, "Add DSP widget fail (%d)\n", ret);
		goto err;
	}
	ret = snd_soc_dapm_add_routes(dapm, nau8310_dsp_dapm_routes,
				      ARRAY_SIZE(nau8310_dsp_dapm_routes));
	if (ret) {
		dev_err(codec->dev, "Add DSP route fail (%d)\n", ret);
		goto err;
	}

	return 0;
err:
	snd_soc_dapm_del_routes(dapm, nau8310_dsp_dapm_routes,
				ARRAY_SIZE(nau8310_dsp_dapm_routes));
	return ret;
}
EXPORT_SYMBOL_GPL(nau8310_dsp_init);

int nau8310_dsp_resume(struct snd_soc_codec *codec)
{
	return nau8310_dsp_set_kcs_setup(codec, false);
}
EXPORT_SYMBOL_GPL(nau8310_dsp_resume);

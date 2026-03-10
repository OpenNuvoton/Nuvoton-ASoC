// SPDX-License-Identifier: GPL-2.0-only
//
// The NAU83G60 DSP driver.
//
// Copyright (C) 2025 Nuvoton Technology Crop.
// Author: David Lin <ctlin0@nuvoton.com>
//         Seven Lee <wtli@nuvoton.com>
//         John Hsu <kchsu0@nuvoton.com>

#define DEBUG
#undef DSP_DBG

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

#include "nau8360.h"
#include "nau8360-dsp.h"

#define NAU8360_DSP_IDLE_RETRY 10
const char *nau8360_def_firmwares[NAU8360_DSP_FW_NUM] =
	{ NAU8360_DSP_FIRMWARE".l", NAU8360_DSP_FIRMWARE".r" };
const unsigned short nau8360_dsp_addr[NAU8360_DSP_FW_NUM] =
	{ NAU8360_RF000_DSP_COMM, NAU8360_RF002_DSP_COMM };

static int nau8360_dsp_chan_kcs_setup(struct snd_soc_component *component,
	const char *fw_name, int dsp_addr);

static const struct nau8360_cmd_info nau8360_dsp_cmd_table[] = {
	[NAU8360_DSP_CMD_GET_COUNTER] = {
		.cmd_id = NAU8360_DSP_CMD_GET_COUNTER,
		.msg_param = 0,
		.setup_data = 0,
		.reply_data = 1,
	},
	[NAU8360_DSP_CMD_GET_FRAME_STATUS] = {
		.cmd_id = NAU8360_DSP_CMD_GET_FRAME_STATUS,
		.msg_param = 0,
		.setup_data = 0,
		.reply_data = 1,
	},
	[NAU8360_DSP_CMD_GET_REVISION] = {
		.cmd_id = NAU8360_DSP_CMD_GET_REVISION,
		.msg_param = 0,
		.setup_data = 0,
		.reply_data = 1,
	},
	[NAU8360_DSP_CMD_GET_KCS_RSLTS] = {
		.cmd_id = NAU8360_DSP_CMD_GET_KCS_RSLTS,
		.msg_param = 1,
		.setup_data = 0,
		.reply_data = 1,
	},
	[NAU8360_DSP_CMD_GET_KCS_SETUP] = {
		.cmd_id = NAU8360_DSP_CMD_GET_KCS_SETUP,
		.msg_param = 1,
		.setup_data = 0,
		.reply_data = 1,
	},
	[NAU8360_DSP_CMD_SET_KCS_SETUP] = {
		.cmd_id = NAU8360_DSP_CMD_SET_KCS_SETUP,
		.msg_param = 1,
		.setup_data = 1,
		.reply_data = 1,
	},
	[NAU8360_DSP_CMD_CLK_STOP] = {
		.cmd_id = NAU8360_DSP_CMD_CLK_STOP,
	},
	[NAU8360_DSP_CMD_CLK_RESTART] = {
		.cmd_id = NAU8360_DSP_CMD_CLK_RESTART,
	},
};

static bool nau8360_dsp_commands(int cmd_id)
{
	switch (cmd_id) {
	case NAU8360_DSP_CMD_GET_COUNTER:
	case NAU8360_DSP_CMD_GET_FRAME_STATUS:
	case NAU8360_DSP_CMD_GET_REVISION:
	case NAU8360_DSP_CMD_GET_KCS_RSLTS:
	case NAU8360_DSP_CMD_GET_KCS_SETUP:
	case NAU8360_DSP_CMD_SET_KCS_SETUP:
	case NAU8360_DSP_CMD_CLK_STOP:
	case NAU8360_DSP_CMD_CLK_RESTART:
		return true;
	default:
		return false;
	}
}

static const char *const dsp_cmd_table[] = {
	[NAU8360_DSP_CMD_GET_COUNTER] = "GET_COUNTER",
	[NAU8360_DSP_CMD_GET_FRAME_STATUS] = "GET_FRAME_STATUS",
	[NAU8360_DSP_CMD_GET_REVISION] = "GET_REVISION",
	[NAU8360_DSP_CMD_GET_KCS_RSLTS] = "GET_KCS_RSLTS",
	[NAU8360_DSP_CMD_GET_KCS_SETUP] = "GET_KCS_SETUP",
	[NAU8360_DSP_CMD_SET_KCS_SETUP] = "SET_KCS_SETUP",
	[NAU8360_DSP_CMD_CLK_STOP] = "CLK_STOP",
	[NAU8360_DSP_CMD_CLK_RESTART] = "CLK_RESTART",
};

/* checking for DSP IDLE pattern */
static int nau8360_dsp_idle(struct snd_soc_component *component, unsigned short dsp_addr)
{
	struct nau8360 *nau8360 = snd_soc_component_get_drvdata(component);
#ifdef DSP_DBG
	u8 buf[4];
#endif
	unsigned int idle_pattern;
	int ret, retries;

	for (retries = NAU8360_DSP_IDLE_RETRY; retries > 0; retries--) {
		ret = regmap_read(nau8360->regmap, dsp_addr, &idle_pattern);
		if (ret) {
			dev_err(nau8360->dev, "failed to read dsp status");
			return ret;
		}
		if (idle_pattern == NAU8360_DSP_COMM_IDLE_WORD) {
#ifdef DSP_DBG
			dev_dbg(component->dev, "idle pattern found");
#endif
			break;
		}
		mdelay(1);
	}
	/* The driver can't establish a connection to DSP. Maybe it is not clocked,
	 * or previous synchronization issue.
	 */
	if (retries == 0) {
		dev_dbg(component->dev, "timeout for idle pattern");
		return -EIO;
	}

#ifdef DSP_DBG
	*(unsigned int *)&buf[0] = idle_pattern;
	dev_dbg(component->dev, "[R] %02x %02x %02x %02x", buf[0], buf[1], buf[2], buf[3]);
#endif
	return 0;
}

static int nau8360_massage_to_dsp(struct snd_soc_component *component,
	const struct nau8360_cmd_info *cmd_info, int frag_len, int param_offset,
	int param_size, void *param_data, unsigned short dsp_addr)
{
	struct device *dev = component->dev;
	u8 data[4], *b_data;
	unsigned int *value = (unsigned int *)data, preamble = NAU8360_DSP_COMM_PREAMBLE;
	int ret, i, data_size, padding = 0, frag_cnt = 0;

	ret = nau8360_dsp_idle(component, dsp_addr);
	if (ret)
		goto err;

	/* sending preamble fragment */
	data[0] = preamble;
	data[1] = preamble >> 8;
	data[2] = (cmd_info->cmd_id << 2) | (frag_len & 0x3);
	data[3] = frag_len >> 2;
	snd_soc_component_write(component, dsp_addr, *value);
#ifdef DSP_DBG
	dev_dbg(dev, "sending preamble fragment (CMD_ID 0x%x, LEN 0x%x)",
		cmd_info->cmd_id, frag_len);
	dev_dbg(dev, "[W] %02x %02x %02x %02x", data[0], data[1], data[2], data[3]);
#endif
	if (!cmd_info->msg_param)
		goto done;

	/* sending payload + padding */
	*value = 0;
	data[0] = param_offset;
	data[1] = param_offset >> 8;
	data[2] = param_size;
	data[3] = param_size >> 8;
	snd_soc_component_write(component, dsp_addr, *value);
	frag_cnt++;
#ifdef DSP_DBG
	dev_dbg(dev, "send fragment (offset 0x%x, size 0x%x)", param_offset, param_size);
	dev_dbg(dev, "[W] %02x %02x %02x %02x", data[0], data[1], data[2], data[3]);
#endif
	if (cmd_info->setup_data) {
		b_data = (u8 *)param_data;
		for (data_size = 0, *value = 0, i = 0; i < param_size; i++) {
			data[i % NAU8360_DSP_DATA_BYTE] = b_data[i];
			data_size++;
			if (data_size == NAU8360_DSP_DATA_BYTE) {
				snd_soc_component_write(component, dsp_addr, *value);
#ifdef DSP_DBG
				dev_dbg(dev, "[W] %02x %02x %02x %02x",
					data[0], data[1], data[2], data[3]);
#endif
				data_size = 0;
				*value = 0;
				frag_cnt++;
			}
		}

		if (data_size > 0 && data_size < NAU8360_DSP_DATA_BYTE) {
			/* sending the data fragments with padding bytes */
			padding = NAU8360_DSP_DATA_BYTE - data_size;
			snd_soc_component_write(component, dsp_addr, *value);
#ifdef DSP_DBG
			dev_dbg(dev, "[W] %02x %02x %02x %02x",
				data[0], data[1], data[2], data[3]);
#endif
			*value = 0;
			frag_cnt++;

		}
#ifdef DSP_DBG
		dev_dbg(dev, "\n");
#endif
	}

	/* sending trailing fragment */
	frag_cnt++;
	*value = 0;
	data[0] = frag_cnt;
	data[1] = ((frag_cnt >> 8) << 6) | (padding << 4);
	snd_soc_component_write(component, dsp_addr, *value);
#ifdef DSP_DBG
	dev_dbg(dev, "send trailing fragment (LEN 0x%x, PAD 0x%x)", frag_cnt, padding);
	dev_dbg(dev, "[W] %02x %02x %02x %02x", data[0], data[1], data[2], data[3]);
#endif
	if (frag_cnt != frag_len) {
		dev_err(dev, "massage error (CMD_ID 0x%x, LEN 0x%x) !!!",
			cmd_info->cmd_id, frag_cnt);
		ret = -EPROTO;
		goto err;
	}

done:
	return 0;
err:
	return ret;
}

static int nau8360_dsp_replied(struct nau8360 *nau8360, int *length,
	unsigned short dsp_addr)
{
	struct device *dev = nau8360->dev;
	u8 buf[4];
	const u8 *b = buf;
	unsigned int reply_preamble;
	int ret, retries, reply_id;

	for (retries = NAU8360_DSP_IDLE_RETRY; retries > 0; retries--) {
		ret = regmap_read(nau8360->regmap, dsp_addr, &reply_preamble);
		if (ret) {
			dev_err(dev, "failed to read reply preamble of dsp");
			return ret;
		}
		/* check for preamble */
		*(unsigned int *)&buf[0] = reply_preamble;
		if (b[0] == (NAU8360_DSP_COMM_PREAMBLE & 0xff) &&
			b[1] == (NAU8360_DSP_COMM_PREAMBLE >> 8)) {
			*length = b[2] & 0x3;
			*length |= b[3] << 2;
			reply_id = b[2] >> 2;
			break;
		}
	}
	if (retries == 0) {
		dev_err(dev, "timeout for reply preamble");
		return -EIO;
	}
#ifdef DSP_DBG
	dev_dbg(dev, "receive preamble fragment (REPLY_ID 0x%x, LEN 0x%x)",
		reply_id, *length);
	dev_dbg(dev, "[R] %02x %02x %02x %02x", b[0], b[1], b[2], b[3]);
#endif
	if (reply_id == NAU8360_DSP_REPLY_OK)
		return 0;
	else
		return -reply_id;
}

static int nau8360_reply_from_dsp(struct snd_soc_component *component,
	const struct nau8360_cmd_info *cmd_info, int data_size,
	void *data, unsigned short dsp_addr)
{
	struct nau8360 *nau8360 = snd_soc_component_get_drvdata(component);
	struct device *dev = component->dev;
	u8 buf[4], *b_data;
	const u8 *b = buf;
	unsigned int payload, *data_buf;
	int i, j, ret, frag_len, frag_payload_len, data_count, len_pos, pad_len,
		pad_len_exp;

	if (!cmd_info->reply_data) {
#ifdef DSP_DBG
		dev_dbg(dev, "The cmd without replay data!!");
#endif
		ret = nau8360_dsp_replied(nau8360, &frag_len, dsp_addr);
		if (ret)
			goto err;
		else if (frag_len == 0)
			goto done;
	}

	if (!data) {
		ret = -EINVAL;
		goto err;
	}
	data_buf = (unsigned int *)data;

	ret = nau8360_dsp_replied(nau8360, &frag_len, dsp_addr);
	if (ret)
		goto err;
	else if (frag_len == 0)
		goto done;

	frag_payload_len = frag_len - 1;
	if (cmd_info->msg_param)
		data_count = data_size;
	for (i = 0; i < frag_payload_len; i++) {
		ret = regmap_read(nau8360->regmap, dsp_addr, &payload);
		if (ret) {
			dev_err(dev, "failed to read payload of dsp");
			goto err;
		}
		if (cmd_info->msg_param) {
			if (data_count >= NAU8360_DSP_DATA_BYTE) {
				*data_buf++ = payload;
				data_count -= NAU8360_DSP_DATA_BYTE;
				*(unsigned int *)&buf[0] = payload;
#ifdef DSP_DBG
				dev_dbg(dev, "[R] %02x %02x %02x %02x",
					buf[0], buf[1], buf[2], buf[3]);
#endif
			} else {
				*(unsigned int *)&buf[0] = payload;
				b_data = (u8 *)data_buf;
				for (j = 0; j < NAU8360_DSP_DATA_BYTE; j++) {
					b_data[j] = b[j];
					data_count--;
					if (data_count <= 0)
						break;
				}
#ifdef DSP_DBG
				dev_dbg(dev, "[R] %02x %02x %02x %02x",
					buf[0], buf[1], buf[2], buf[3]);
#endif
				break;
			}
		} else {
			*(unsigned int *)&buf[0] = payload;
			*data_buf = b[0];
			*data_buf |= b[1] << 8;
			*data_buf |= b[2] << 16;
			*data_buf |= b[3] << 24;
#ifdef DSP_DBG
			dev_dbg(dev, "[R] %02x %02x %02x %02x",
				buf[0], buf[1], buf[2], buf[3]);
#endif
		}
	}

	/* check the reply length same as request */
	if (data_count && (cmd_info->cmd_id == NAU8360_DSP_CMD_GET_KCS_RSLTS ||
			cmd_info->cmd_id == NAU8360_DSP_CMD_GET_KCS_SETUP)) {
		dev_warn(dev, "payload_len %d, expected %d",
			data_size - data_count, data_size);
	}
#ifdef DSP_DBG
	dev_dbg(dev, "reading trailing fragment");
#endif
	ret = regmap_read(nau8360->regmap, dsp_addr, &payload);
	if (ret) {
		dev_err(dev, "failed to read trailing fragment of dsp");
		goto err;
	}
	*(unsigned int *)&buf[0] = payload;
	len_pos = b[0];
	len_pos |= (b[1] & 0xc0) << 2;
	if (len_pos != frag_len) {
		dev_err(dev, "LEN_POST %02X, expect %02X", len_pos, frag_len);
		ret = -EPROTO;
		goto err;
	}
	pad_len = (b[1] & 0x30) >> 4;
	if (cmd_info->msg_param)
		pad_len_exp = frag_payload_len * NAU8360_DSP_DATA_BYTE -
			(data_size - data_count);
	else
		pad_len_exp = 0;
	if (pad_len != pad_len_exp) {
		dev_err(dev, "PAD_LEN %02X, expect %02X", pad_len, pad_len_exp);
		ret = -EPROTO;
		goto err;
	}
#ifdef DSP_DBG
	dev_dbg(dev, "LEN_POST 0x%x, PAD_LEN 0x%x", len_pos, pad_len);
	dev_dbg(dev, "[R] %02x %02x %02x %02x", buf[0], buf[1], buf[2], buf[3]);
#endif
done:
	return 0;
err:
	dev_err(dev, "DSP reply error %d !!!", ret);
	return ret;
}

/**
 * nau8360_send_dsp_command - Send command to DSP
 *
 * @component:  component to register
 * @cmd_id:  DSP supported command ID
 * @kcs_setup: KCS setup structure
 * @dsp_addr: DSP address
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
int nau8360_send_dsp_command(struct snd_soc_component *component, int cmd_id,
	struct nau8360_kcs_setup *kcs_setup, unsigned short dsp_addr)
{
	const struct nau8360_cmd_info *cmd_info;
	int ret, frag_len = 0;

	if (!component || !kcs_setup) {
		ret = -EINVAL;
		goto msg_fail;
	}
	if (!nau8360_dsp_commands(cmd_id)) {
		dev_err(component->dev, "command not support!");
		ret = -EINVAL;
		goto msg_fail;
	}

	cmd_info = &nau8360_dsp_cmd_table[cmd_id];
	if ((cmd_info->msg_param && !kcs_setup->set_len) ||
		(cmd_info->setup_data && !kcs_setup->set_kcs_data) ||
		(cmd_info->reply_data && !kcs_setup->get_data)) {
		ret = -EFAULT;
		goto msg_fail;
	}

	/* Read up to 1kB data because the LEN field to request data is 10-bits
	 * long; and not beyond 3kB offset.
	 */
	if (cmd_id == NAU8360_DSP_CMD_GET_KCS_SETUP &&
		(kcs_setup->set_len > NAU8360_DSP_KCS_DAT_LEN_MAX ||
			kcs_setup->set_kcs_offset > NAU8360_DSP_KCS_OFFSET_MAX)) {
		ret = -ERANGE;
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
				NAU8360_DSP_DATA_BYTE - 1) / NAU8360_DSP_DATA_BYTE;

	ret = nau8360_massage_to_dsp(component, cmd_info, frag_len,
			kcs_setup->set_kcs_offset, kcs_setup->set_len,
			kcs_setup->set_kcs_data, dsp_addr);
	if (ret)
		goto msg_fail;

	ret = nau8360_reply_from_dsp(component, cmd_info, kcs_setup->get_len,
			kcs_setup->get_data, dsp_addr);
	if (ret)
		goto reply_fail;

	return 0;

msg_fail:
	dev_err(component->dev, "fail to send a message %d to dsp (%d)", cmd_id, ret);
	return ret;
reply_fail:
	dev_err(component->dev, "reply fail (%d) from dsp.", ret);
	return ret;
}

static inline int nau8360_send_dsp_broadcast(struct snd_soc_component *cp,
	int cmd_id, struct nau8360_kcs_setup *kcs_setup)
{
	int i, ret;

	for (i = 0; i < NAU8360_DSP_FW_NUM; i++) {
		ret = nau8360_send_dsp_command(cp, cmd_id, kcs_setup,
				nau8360_dsp_addr[i]);
		if (ret) {
			dev_err(cp->dev, "DSP %x fail (%d)", nau8360_dsp_addr[i], ret);
			return ret;
		}
	}

	return 0;
}

/**
 * nau8360_dsp_kcs_setup - Send KCS setup command to DSP
 *
 * @component:  component to register
 * @offset: address offset relative to KCS start
 * @size: size of data writen to KCS
 * @data: data writen to KCS setup
 * @dsp_addr : DSP address
 *
 * The function sends KCS setup command to DSP for
 * setting KCS configuration. The maximum size that you can transfer into
 * the DSP is 96 bytes. Therefore, the driver has to split the data into
 * 96 bytes chucks, if the setup configuration over the threshold.
 */
int nau8360_dsp_kcs_setup(struct snd_soc_component *component, int offset, int size,
	const void *data, unsigned short dsp_addr)
{
	struct nau8360_kcs_setup kcs_setup_comp, *kcs_setup = &kcs_setup_comp;
	u8 *data_buf;
	unsigned int kcs_rst;
	int cmd_id = NAU8360_DSP_CMD_SET_KCS_SETUP, retries, ret, data_len, data_rem,
		addr_offset;

	/* Limit full load of KCS_SETUP data and not beyond 3kB offset. */
	if (!data || size > NAU8360_DSP_KCS_DAT_LEN_MAX ||
		offset > NAU8360_DSP_KCS_OFFSET_MAX) {
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
		if (data_rem > NAU8360_DSP_KCS_TX_MAX)
			data_len = NAU8360_DSP_KCS_TX_MAX;
		else
			data_len = data_rem;
		cmd_id = NAU8360_DSP_CMD_SET_KCS_SETUP;
		kcs_setup->set_kcs_offset = addr_offset;
		kcs_setup->set_len = data_len;
		kcs_setup->set_kcs_data = data_buf;
		ret = nau8360_send_dsp_command(component, cmd_id, kcs_setup, dsp_addr);
		if (ret < 0) {
			if (retries++ < NAU8360_DSP_RETRY_MAX)
				continue;
			else
				goto msg_fail;
		} else {
			data_buf += (u8)data_len;
			addr_offset += data_len;
			data_rem -= data_len;
		}
		/* checking KCS result */
		cmd_id = NAU8360_DSP_CMD_GET_KCS_RSLTS;
		kcs_setup->set_kcs_offset = 0;
		kcs_setup->set_len = NAU8360_DSP_DATA_BYTE;
		kcs_setup->get_len = kcs_setup->set_len;
		ret = nau8360_send_dsp_command(component, cmd_id, kcs_setup, dsp_addr);
		if (ret < 0)
			goto msg_fail;
	}

	return 0;

msg_fail:
	dev_err(component->dev, "send a kcs setup message %d fail (%d)", cmd_id, ret);
	return ret;
}

static int nau8360_dsp_get_cmd_put(struct snd_soc_component *component,
	int dsp_addr, int cmd, int *value)
{
	struct nau8360_kcs_setup kcs_setup_comp, *kcs_setup = &kcs_setup_comp;
	int ret;

	dev_info(component->dev, "send DSP %x command %s", dsp_addr, dsp_cmd_table[cmd]);

	kcs_setup->get_len = kcs_setup->set_len = sizeof(int);
	kcs_setup->get_data = (void *)value;
	ret = nau8360_send_dsp_command(component, cmd, kcs_setup, dsp_addr);
	if (ret) {
		dev_err(component->dev, "do command fail (%d)", ret);
		return ret;
	}

	return 0;
}

static int nau8360_dsp_get_counter_put(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	int ret, counter, dsp_addr = NAU8360_DSP_ADDR_BYNAME(kcontrol->id.name);

	if (!ucontrol->value.integer.value[0])
		return 0;

	ret = nau8360_dsp_get_cmd_put(component, dsp_addr,
			NAU8360_DSP_CMD_GET_COUNTER, &counter);
	if (ret)
		return ret;

	dev_info(component->dev, "DSP %x counter %d", dsp_addr, counter);

	return 0;
}

static int nau8360_dsp_get_frame_status_put(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	int ret, status, dsp_addr = NAU8360_DSP_ADDR_BYNAME(kcontrol->id.name);

	if (!ucontrol->value.integer.value[0])
		return 0;

	ret = nau8360_dsp_get_cmd_put(component, dsp_addr,
			NAU8360_DSP_CMD_GET_FRAME_STATUS, &status);
	if (ret)
		return ret;

	dev_info(component->dev, "DSP %x frame status %x", dsp_addr, status);

	return 0;
}

static int nau8360_dsp_get_revision_put(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	int ret, version, dsp_addr = NAU8360_DSP_ADDR_BYNAME(kcontrol->id.name);

	if (!ucontrol->value.integer.value[0])
		return 0;

	ret = nau8360_dsp_get_cmd_put(component, dsp_addr,
			NAU8360_DSP_CMD_GET_REVISION, &version);
	if (ret)
		return ret;

	dev_info(component->dev, "DSP %x version %x", dsp_addr, version);

	return 0;
}

static int nau8360_dsp_get_kcs_setup_put(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct device *dev = component->dev;
	struct nau8360 *nau8360 = snd_soc_component_get_drvdata(component);
	struct nau8360_kcs_setup kcs_setup_comp, *kcs_setup = &kcs_setup_comp;
	int ret, i, buf_off, buf_len, count,
		dsp_addr = NAU8360_DSP_ADDR_BYNAME(kcontrol->id.name);
	char *data, buf[100];

	if (!ucontrol->value.integer.value[0])
		return 0;

	if (snd_soc_component_get_bias_level(component) > SND_SOC_BIAS_STANDBY) {
		dev_err(dev, "command is not allowed during playback");
		return -EINVAL;
	}

	data = kcalloc(nau8360->kcs_setup_size, sizeof(char), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	if (nau8360->kcs_setup_size == 0) {
		kfree(data);
		ret = -EINVAL;
		dev_err(dev, "KCS of DSP not load yet (%d)", ret);
		return ret;
	}
	buf_off = 0;
	buf_len = nau8360->kcs_setup_size;
	dev_info(dev, "send DSP command %s (OFF %d, LEN %d)",
		dsp_cmd_table[NAU8360_DSP_CMD_GET_KCS_SETUP], buf_off, buf_len);
	kcs_setup->set_kcs_offset = buf_off;
	kcs_setup->get_len = kcs_setup->set_len = buf_len;
	kcs_setup->get_data = data;
	ret = nau8360_send_dsp_command(component, NAU8360_DSP_CMD_GET_KCS_SETUP,
			kcs_setup, dsp_addr);
	if (ret) {
		dev_err(dev, "send DSP command %s fail (%d)",
			dsp_cmd_table[NAU8360_DSP_CMD_GET_KCS_SETUP], ret);
	} else {
		dev_dbg(dev, "DSP KCS result:");
		for (i = 0; i < kcs_setup->get_len; i += 16) {
			dev_dbg(dev, "%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x",
				data[i], data[i + 1], data[i + 2], data[i + 3],
				data[i + 4], data[i + 5], data[i + 6], data[i + 7],
				data[i + 8], data[i + 9], data[i + 10], data[i + 11],
				data[i + 12], data[i + 13], data[i + 14], data[i + 15]);
			if (kcs_setup->get_len - i < 16)
				break;
		}
		count = 0;
		for (; i < kcs_setup->get_len; i++)
			count += sprintf(buf + count, "%02x ", data[i]);
		if (count)
			dev_dbg(dev, "%s", buf);
		dev_info(dev, "get length %d of kcs_setup", kcs_setup->get_len);
	}
	kfree(data);

	return 0;
}

static int nau8360_dsp_set_kcs_setup_put(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct nau8360 *nau8360 = snd_soc_component_get_drvdata(component);
	int i, ret, dsp_addr = NAU8360_DSP_ADDR_BYNAME(kcontrol->id.name);
	char firmware[NAU8360_DSP_FW_NAMELEN];
	const char *fw_name;

	if (!ucontrol->value.integer.value[0])
		return 0;

	if (snd_soc_component_get_bias_level(component) > SND_SOC_BIAS_STANDBY) {
		dev_err(component->dev, "command is not allowed during playback");
		return -EINVAL;
	}

	for (i = 0; i < NAU8360_DSP_FW_NUM; i++) {
		if (nau8360_dsp_addr[i] != dsp_addr)
			continue;

		if (nau8360->dsp_fws_num) {
			snprintf(firmware, sizeof(firmware), NAU8360_DSP_FIRMDIR"%s",
				nau8360->dsp_firmware[i]);
			fw_name = firmware;
		} else
			fw_name = nau8360_def_firmwares[i];

		ret = nau8360_dsp_chan_kcs_setup(component, fw_name, dsp_addr);
		if (ret)
			return ret;
	}

	return 0;
}

static int nau8360_dsp_cmd_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = 0;
	return 0;
}

static const struct snd_kcontrol_new nau8360_dsp_snd_controls[] = {
	SOC_SINGLE_EXT("Left DSP get counter command", SND_SOC_NOPM, 0, 1, 0,
		nau8360_dsp_cmd_get, nau8360_dsp_get_counter_put),
	SOC_SINGLE_EXT("Right DSP get counter command", SND_SOC_NOPM, 0, 1, 0,
		nau8360_dsp_cmd_get, nau8360_dsp_get_counter_put),
	SOC_SINGLE_EXT("Left DSP get frame status command", SND_SOC_NOPM, 0, 1, 0,
		nau8360_dsp_cmd_get, nau8360_dsp_get_frame_status_put),
	SOC_SINGLE_EXT("Right DSP get frame status command", SND_SOC_NOPM, 0, 1, 0,
		nau8360_dsp_cmd_get, nau8360_dsp_get_frame_status_put),
	SOC_SINGLE_EXT("Left DSP get revision command", SND_SOC_NOPM, 0, 1, 0,
		nau8360_dsp_cmd_get, nau8360_dsp_get_revision_put),
	SOC_SINGLE_EXT("Right DSP get revision command", SND_SOC_NOPM, 0, 1, 0,
		nau8360_dsp_cmd_get, nau8360_dsp_get_revision_put),
	SOC_SINGLE_EXT("Left DSP get KCS setup command", SND_SOC_NOPM, 0, 1, 0,
		nau8360_dsp_cmd_get, nau8360_dsp_get_kcs_setup_put),
	SOC_SINGLE_EXT("Right DSP get KCS setup command", SND_SOC_NOPM, 0, 1, 0,
		nau8360_dsp_cmd_get, nau8360_dsp_get_kcs_setup_put),
	SOC_SINGLE_EXT("Left DSP set KCS setup command", SND_SOC_NOPM, 0, 1, 0,
		nau8360_dsp_cmd_get, nau8360_dsp_set_kcs_setup_put),
	SOC_SINGLE_EXT("Right DSP set KCS setup command", SND_SOC_NOPM, 0, 1, 0,
		nau8360_dsp_cmd_get, nau8360_dsp_set_kcs_setup_put),
};

static int nau8360_dsp_clock_event(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);
	struct nau8360_kcs_setup kcs_setup_comp, *kcs_setup = &kcs_setup_comp;
	int ret, cmd;

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		cmd = NAU8360_DSP_CMD_CLK_RESTART;
		break;
	case SND_SOC_DAPM_POST_PMD:
		cmd = NAU8360_DSP_CMD_CLK_STOP;
		break;
	default:
		ret = -EINVAL;
		goto err;
	}
	dev_dbg(component->dev, "send DSP command %s", dsp_cmd_table[cmd]);
	ret = nau8360_send_dsp_broadcast(component, cmd, kcs_setup);
	if (ret) {
		dev_err(component->dev, "send DSP command %s fail (%d)",
			dsp_cmd_table[cmd], ret);
		goto err;
	}

	return 0;
err:
	return ret;
}

static const struct snd_soc_dapm_widget nau8360_dsp_dapm_widgets[] = {
	SND_SOC_DAPM_SUPPLY("DSP Clock", SND_SOC_NOPM, 0, 0, nau8360_dsp_clock_event,
		SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
};

static const struct snd_soc_dapm_route nau8360_dsp_dapm_routes[] = {
	{ "DSP", NULL, "HW3 Engine" },
	{ "DSP", NULL, "DSP Clock" },
};

static int nau8360_dsp_chan_kcs_setup(struct snd_soc_component *cp,
	const char *fw_name, int dsp_addr)
{
	struct nau8360 *nau8360 = snd_soc_component_get_drvdata(cp);
	const struct firmware *fw;
	int ret, status, buf_off, buf_len;

	ret = nau8360_dsp_get_cmd_put(cp, dsp_addr,
			NAU8360_DSP_CMD_GET_FRAME_STATUS, &status);
	if (ret || !(status & NAU8360_DSP_ALGO_OK)) {
		dev_err(cp->dev, "DSP %x is not ready", dsp_addr);
		ret = -EIO;
		goto err;
	}

	dev_info(cp->dev, "DSP %x is ready to load firmware %s, status %x",
		dsp_addr, fw_name, status);

	ret = request_firmware(&fw, fw_name, cp->dev);
	if (ret) {
		dev_err(cp->dev, "failed to load firmware (%d)", ret);
		goto err;
	}

	buf_off = 0;
	buf_len = nau8360->kcs_setup_size = fw->size;
	ret = nau8360_dsp_kcs_setup(cp, buf_off, buf_len, fw->data, dsp_addr);
	if (ret) {
		dev_err(cp->dev, "send DSP command %s fail (%d)",
			dsp_cmd_table[NAU8360_DSP_CMD_SET_KCS_SETUP], ret);
		goto err_loaded;
	}
	release_firmware(fw);

	return 0;

err_loaded:
	if (fw)
		release_firmware(fw);
err:
	return ret;
}

static int nau8360_dsp_set_kcs_setup(struct snd_soc_component *cp)
{
	struct nau8360 *nau8360 = snd_soc_component_get_drvdata(cp);
	char firmware[NAU8360_DSP_FW_NAMELEN];
	const char *fw_name;
	int i, ret;

	for (i = 0; i < NAU8360_DSP_FW_NUM; i++) {
		if (nau8360->dsp_fws_num) {
			snprintf(firmware, sizeof(firmware), NAU8360_DSP_FIRMDIR"%s",
				nau8360->dsp_firmware[i]);
			fw_name = firmware;
		} else
			fw_name = nau8360_def_firmwares[i];

		ret = nau8360_dsp_chan_kcs_setup(cp, fw_name, nau8360_dsp_addr[i]);
		if (ret)
			return ret;

		msleep(100);
	}

	return 0;
}

int nau8360_dsp_init(struct snd_soc_component *component)
{
	struct nau8360 *nau8360 = snd_soc_component_get_drvdata(component);
	struct snd_soc_dapm_context *dapm = nau8360->dapm;
	int ret;

	dev_info(component->dev, "DSP initializing...");
	ret = nau8360_dsp_set_kcs_setup(component);
	if (ret)
		goto err;

	ret = snd_soc_add_component_controls(component, nau8360_dsp_snd_controls,
			ARRAY_SIZE(nau8360_dsp_snd_controls));
	if (ret) {
		dev_err(component->dev, "add DSP control fail (%d)", ret);
		goto err;
	}
	ret = snd_soc_dapm_new_controls(dapm, nau8360_dsp_dapm_widgets,
			ARRAY_SIZE(nau8360_dsp_dapm_widgets));
	if (ret) {
		dev_err(component->dev, "add DSP widget fail (%d)", ret);
		goto err;
	}
	ret = snd_soc_dapm_add_routes(dapm, nau8360_dsp_dapm_routes,
			ARRAY_SIZE(nau8360_dsp_dapm_routes));
	if (ret) {
		dev_err(component->dev, "add DSP route fail (%d)", ret);
		goto err;
	}
	nau8360->dsp_created = true;

	return 0;

err:
	return ret;
}
EXPORT_SYMBOL_GPL(nau8360_dsp_init);

int nau8360_dsp_reinit(struct snd_soc_component *component)
{
	dev_info(component->dev, "DSP initializing...");
	return nau8360_dsp_set_kcs_setup(component);
}
EXPORT_SYMBOL_GPL(nau8360_dsp_reinit);

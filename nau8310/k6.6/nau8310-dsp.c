// SPDX-License-Identifier: GPL-2.0-only
//
// The NAU83G10/20 Boosted Mono Class-D Amplifier with DSP and I/V-sense driver.
//
// Copyright 2026 Nuvoton Technology Crop.
// Author: John Hsu <KCHSU0@nuvoton.com>
//         David Lin <ctlin0@nuvoton.com>
//         Neo Chang <ylchang2@nuvoton.com>

#define DEBUG

#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/bitfield.h>
#include <sound/soc.h>

#include "nau8310-dsp.h"
#include "nau8310.h"

#define NAU8310_DSP_IDLE_RETRY 10

static int nau8310_dsp_set_kcs_setup(struct snd_soc_component *component);

#define NAU_DSP_CMD(_id, _msg, _setup, _reply) \
	[_id] = { \
		.cmd_id = _id, \
		.msg_param = _msg, \
		.setup_data = _setup, \
		.reply_data = _reply, \
	}

#define NAU_DSP_CMD_ID(_id) \
	[_id] = { \
		.cmd_id = _id, \
	}

/**
 * Preamble Fragment Masks:
 * FIELD           | BITS   | SHIFT | MASK | DETAILS
 * ----------------|--------|-------|------|---------------------------------
 * length high     | [31:24]| 24    | 0xff | Upper 8 bits of total length (LEN[9:2])
 * cmd_id          | [23:18]| 18    | 0x3f | Command identifier (6 bits)
 * length low      | [17:16]| 16    | 0x03 | Lower 2 bits of total length (LEN[1:0])
 * preamble magic  | [15:0] | 0     |0xffff| Preamble signature (0xB2A1)
 */
#define NAU8310_HOST_LEN_HIGH_MASK	GENMASK(31, 24)
#define NAU8310_HOST_CMD_ID_MASK	GENMASK(23, 18)
#define NAU8310_HOST_LEN_LOW_MASK	GENMASK(17, 16)
#define NAU8310_HOST_PREAMBLE_MASK	GENMASK(15, 0)

/**
 * Payload Fragment Masks:
 * FIELD           | BITS   | SHIFT | MASK | DETAILS
 * ----------------|--------|-------|------|---------------------------------
 * param_size      | [31:16]| 16    |0xffff| Size of parameter data (16 bits)
 * param_offset    | [15:0] | 0     |0xffff| Starting offset (16 bits)
 */
#define NAU8310_HOST_PARAM_SIZE_MASK	GENMASK(31, 16)
#define NAU8310_HOST_PARAM_OFFSET_MASK	GENMASK(15, 0)

/**
 * Trailing Fragment Masks:
 * FIELD           | BITS   | SHIFT | MASK | DETAILS
 * ----------------|--------|-------|------|---------------------------------
 * length high     | [15:14]| 14    | 0x03 | Upper 2 bits of total length
 * padding         | [13:12]| 12    | 0x03 | Padding bytes count
 * length low      | [7:0]  | 0     | 0xff | Lower 8 bits of total length
 * Note: Reassembled length = (length_high << 8) | length_low (Total 10 bits)
 */
#define NAU8310_TRAIL_LEN_LOW_MASK	GENMASK(7, 0)
#define NAU8310_TRAIL_PAD_MASK		GENMASK(13, 12)
#define NAU8310_TRAIL_LEN_HIGH_MASK	GENMASK(15, 14)

/**
 * Reply Preamble Masks:
 * FIELD           | BITS   | SHIFT | MASK | DETAILS
 * ----------------|--------|-------|------|---------------------------------
 * length high     | [31:24]| 24    | 0xff | Upper 8 bits of total length
 * reply_id        | [23:18]| 18    | 0x3f | Reply identifier (6 bits)
 * length low      | [17:16]| 16    | 0x03 | Lower 2 bits of total length
 * Note: Reassembled length = (length_high << 2) | length_low (Total 10 bits)
 */
#define NAU8310_REPLY_LEN_HIGH_MASK	GENMASK(31, 24)
#define NAU8310_REPLY_ID_MASK		GENMASK(23, 18)
#define NAU8310_REPLY_LEN_LOW_MASK	GENMASK(17, 16)

#ifdef DSP_DBG
#define __dsp_dbg_data(dev, prefix, val) \
	dsp_dbg(dev, prefix " %02x %02x %02x %02x", \
		(u8)((val) & 0xff), \
		(u8)(((val) >> 8) & 0xff), \
		(u8)(((val) >> 16) & 0xff), \
		(u8)(((val) >> 24) & 0xff))
#define payload_read(dev, val)  __dsp_dbg_data(dev, "[R]", val)
#define payload_write(dev, val) __dsp_dbg_data(dev, "[W]", val)
#endif

static const struct nau8310_cmd_info nau8310_dsp_cmd_table[] = {
	/* { cmd_id, msg_param, setup_data, reply_data} */
	NAU_DSP_CMD(NAU8310_DSP_CMD_GET_COUNTER,      0, 0, 1),
	NAU_DSP_CMD(NAU8310_DSP_CMD_GET_FRAME_STATUS, 0, 0, 1),
	NAU_DSP_CMD(NAU8310_DSP_CMD_GET_REVISION,     0, 0, 1),
	NAU_DSP_CMD(NAU8310_DSP_CMD_GET_KCS_RSLTS,    1, 0, 1),
	NAU_DSP_CMD(NAU8310_DSP_CMD_GET_KCS_SETUP,    1, 0, 1),
	NAU_DSP_CMD(NAU8310_DSP_CMD_SET_KCS_SETUP,    1, 1, 0),
	NAU_DSP_CMD_ID(NAU8310_DSP_CMD_CLK_STOP),
	NAU_DSP_CMD_ID(NAU8310_DSP_CMD_CLK_RESTART),
};

static const char *const dsp_cmd_table[] = {
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

static int nau8310_dsp_idle(struct snd_soc_component *cp)
{
	struct nau8310 *nau8310 = snd_soc_component_get_drvdata(cp);
	unsigned int idle_pattern, timeout = NAU8310_DSP_IDLE_RETRY * USEC_PER_MSEC;
	int ret = 0;

	ret = regmap_read_poll_timeout(nau8310->regmap,
		NAU8310_RF000_DSP_COMM, idle_pattern,
		idle_pattern == NAU8310_DSP_COMM_IDLE_WORD, USEC_PER_MSEC, timeout);
	if (ret) {
		dev_err(nau8310->dev, "Timeout waiting for DSP idle state: %d", ret);
		nau8310_sw_reset_chip(nau8310->regmap);
	}

#ifdef DSP_DBG
	dsp_dbg(cp->dev, "idle pattern found");
	payload_read(cp->dev, idle_pattern);
#endif

	return ret;
}

/**
 * nau8310_pack_preamble - Pack DSP preamble fragment
 * @cmd_id: Command ID for the DSP message
 * @frag_len: Total length of the message fragments
 *
 * Return: 32-bit packed payload in Little Endian format.
 */
static inline u32 nau8310_pack_preamble(u8 cmd_id, u16 frag_len)
{
	return FIELD_PREP(NAU8310_HOST_PREAMBLE_MASK, NAU8310_DSP_COMM_PREAMBLE) |
		FIELD_PREP(NAU8310_HOST_CMD_ID_MASK, cmd_id) |
		FIELD_PREP(NAU8310_HOST_LEN_LOW_MASK, frag_len) |
		FIELD_PREP(NAU8310_HOST_LEN_HIGH_MASK, frag_len >> 2);
}

/**
 * nau8310_pack_param - Pack DSP parameter fragment
 * @param_offset: Starting offset of the parameter data
 * @param_size: Size of the parameter data in bytes
 *
 * Return: 32-bit packed payload in Little Endian format.
 */
static inline u32 nau8310_pack_param(u16 param_offset, u16 param_size)
{
	return FIELD_PREP(NAU8310_HOST_PARAM_OFFSET_MASK, param_offset) |
		FIELD_PREP(NAU8310_HOST_PARAM_SIZE_MASK, param_size);
}

/**
 * nau8310_pack_trailing - Pack DSP trailing fragment
 * @frag_cnt: Current fragment count
 * @padding: Number of padding bytes added to the final data fragment
 *
 * Return: 32-bit packed payload in Little Endian format.
 */
static inline u32 nau8310_pack_trailing(u16 frag_cnt, u8 padding)
{
	return FIELD_PREP(NAU8310_TRAIL_LEN_LOW_MASK, frag_cnt) |
		FIELD_PREP(NAU8310_TRAIL_PAD_MASK, padding) |
		FIELD_PREP(NAU8310_TRAIL_LEN_HIGH_MASK, frag_cnt >> 8);
}

static void nau8310_send_data_payload(struct snd_soc_component *cp,
	unsigned short dsp_addr, const void *param_data, int param_size,
	int *frag_cnt, int *padding)
{
	const u8 *data = (const u8 *)param_data;
	u32 payload = 0;
	int i, data_size = 0;

	for (i = 0; i < param_size; i++) {
		payload |= data[i] << (data_size * 8);
		data_size++;

		if (data_size == NAU8310_DSP_DATA_BYTE) {
			snd_soc_component_write(cp, dsp_addr, payload);
#ifdef DSP_DBG
			payload_write(cp->dev, payload);
#endif
			data_size = 0;
			payload = 0;
			(*frag_cnt)++;
		}
	}

	if (data_size > 0) {
		*padding = NAU8310_DSP_DATA_BYTE - data_size;
		snd_soc_component_write(cp, dsp_addr, payload);
#ifdef DSP_DBG
		payload_write(cp->dev, payload);
#endif
		(*frag_cnt)++;
	}
}

static int nau8310_message_to_dsp(struct snd_soc_component *cp,
	const struct nau8310_cmd_info *cmd_info, int frag_len,
	int param_offset, int param_size, void *param_data)
{
	unsigned int payload;
	int ret, padding = 0, frag_cnt = 0;

	ret = nau8310_dsp_idle(cp);
	if (ret)
		return ret;

	/* sending preamble fragment */
	payload = nau8310_pack_preamble(cmd_info->cmd_id, frag_len);
	snd_soc_component_write(cp, NAU8310_RF000_DSP_COMM, payload);
#ifdef DSP_DBG
	dsp_dbg(cp->dev, "sending preamble fragment (CMD_ID 0x%x, LEN 0x%x)",
		cmd_info->cmd_id, frag_len);
	payload_write(cp->dev, payload);
#endif
	if (!cmd_info->msg_param)
		return ret;

	/* sending payload + padding */
	payload = nau8310_pack_param(param_offset, param_size);
	snd_soc_component_write(cp, NAU8310_RF000_DSP_COMM, payload);
	frag_cnt++;
#ifdef DSP_DBG
	dsp_dbg(cp->dev, "send fragment (offset 0x%x, size 0x%x)", param_offset, param_size);
	payload_write(cp->dev, payload);
#endif

	if (cmd_info->setup_data)
		nau8310_send_data_payload(cp, NAU8310_RF000_DSP_COMM, param_data, param_size,
			&frag_cnt, &padding);

	/* sending trailing fragment */
	frag_cnt++;
	payload = nau8310_pack_trailing(frag_cnt, padding);
	snd_soc_component_write(cp, NAU8310_RF000_DSP_COMM, payload);
#ifdef DSP_DBG
	dsp_dbg(cp->dev, "send trailing fragment (LEN 0x%x, PAD 0x%x)", frag_cnt, padding);
	payload_write(cp->dev, payload);
#endif
	if (frag_cnt != frag_len) {
		dev_err(cp->dev, "message error (CMD_ID 0x%x, LEN 0x%x) !!!",
			cmd_info->cmd_id, frag_cnt);
		return -EPROTO;
	}

	return 0;
}

static int nau8310_dsp_replied(struct nau8310 *nau8310, int *length)
{
	unsigned int reply_preamble;
	int ret, reply_id;

	ret = regmap_read_poll_timeout(nau8310->regmap, NAU8310_RF000_DSP_COMM, reply_preamble,
		(reply_preamble & 0xffff) == NAU8310_DSP_COMM_PREAMBLE,
		USEC_PER_MSEC, NAU8310_DSP_IDLE_RETRY * USEC_PER_MSEC);
	if (ret) {
		dev_err(nau8310->dev, "timeout for reply preamble: %d", ret);
		return ret;
	}

	*length = FIELD_GET(NAU8310_REPLY_LEN_LOW_MASK, reply_preamble);
	*length |= FIELD_GET(NAU8310_REPLY_LEN_HIGH_MASK, reply_preamble) << 2;
	reply_id = FIELD_GET(NAU8310_REPLY_ID_MASK, reply_preamble);

#ifdef DSP_DBG
	dsp_dbg(nau8310->dev, "receive preamble fragment (REPLY_ID 0x%x, LEN 0x%x)",
		reply_id, *length);
	payload_read(nau8310->dev, reply_preamble);
#endif

	return (reply_id != NAU8310_DSP_REPLY_OK) ? -reply_id : 0;
}

static int nau8310_validate_trailing(struct snd_soc_component *cp,
	int frag_len, int pad_len_exp)
{
	int ret, len_pos, pad_len;
	unsigned int payload;
	struct device *dev = cp->dev;
	struct nau8310 *nau8310 = snd_soc_component_get_drvdata(cp);

	/* reading trailing fragment */
	ret = regmap_read(nau8310->regmap, NAU8310_RF000_DSP_COMM, &payload);
	if (ret) {
		dev_err(dev, "failed to read trailing fragment");
		return ret;
	}

	len_pos = FIELD_GET(NAU8310_TRAIL_LEN_LOW_MASK, payload);
	len_pos |= FIELD_GET(NAU8310_TRAIL_LEN_HIGH_MASK, payload) << 8;

	if (len_pos != frag_len) {
		dev_err(dev, "LEN_POST %02X, expect %02X", len_pos, frag_len);
		return -EPROTO;
	}

	pad_len = FIELD_GET(NAU8310_TRAIL_PAD_MASK, payload);
	if (pad_len != pad_len_exp) {
		dev_err(dev, "PAD_LEN %02X, expect %02X", pad_len, pad_len_exp);
		return -EPROTO;
	}

#ifdef DSP_DBG
	dsp_dbg(dev, "LEN_POST 0x%x, PAD_LEN 0x%x", len_pos, pad_len);
	payload_read(dev, payload);
#endif

	return 0;
}

static int nau8310_read_data_payload(struct snd_soc_component *cp,
	int frag_len, bool msg_param,
	void *data, int data_size, int *data_count)
{
	struct nau8310 *nau8310 = snd_soc_component_get_drvdata(cp);
	struct device *dev = cp->dev;
	unsigned int payload;
	u32 *data_buf = data;
	int i, j, ret;

	*data_count = (msg_param) ? data_size : 0;
	for (i = 0; i < frag_len - 1; i++) {
		ret = regmap_read(nau8310->regmap, NAU8310_RF000_DSP_COMM, &payload);
		if (ret) {
			dev_err(dev, "failed to read payload of dsp");
			return ret;
		}
#ifdef DSP_DBG
		payload_read(dev, payload);
#endif
		if (!msg_param) {
			*data_buf++ = payload;
			break;
		}

		if (*data_count >= NAU8310_DSP_DATA_BYTE) {
			*data_buf++ = payload;
			*data_count -= NAU8310_DSP_DATA_BYTE;
		} else if (*data_count > 0) {
			for (j = 0; j < *data_count; j++)
				((u8 *)data_buf)[j] = (payload >> (j * 8)) & 0xff;

			*data_count = 0;
		}

		if (*data_count == 0)
			break;
	}

	return 0;
}

static int nau8310_reply_from_dsp(struct snd_soc_component *cp,
	const struct nau8310_cmd_info *cmd_info, int data_size,
	void *data)
{
	struct nau8310 *nau8310 = snd_soc_component_get_drvdata(cp);
	struct device *dev = cp->dev;
	int ret, frag_len, pad_len_exp, data_count = 0;

	ret = nau8310_dsp_replied(nau8310, &frag_len);
	if (ret)
		return ret;
	if (!frag_len || !cmd_info->reply_data)
		return 0;
	if (!data || frag_len == 1)
		return -EINVAL;

	ret = nau8310_read_data_payload(cp, frag_len,
		cmd_info->msg_param, data, data_size, &data_count);
	if (ret)
		return ret;

	/* check the reply length same as request */
	if (data_count && (cmd_info->cmd_id == NAU8310_DSP_CMD_GET_KCS_RSLTS ||
		cmd_info->cmd_id == NAU8310_DSP_CMD_GET_KCS_SETUP))
		dev_warn(dev, "payload_len %d, expected %d",
			data_size - data_count, data_size);

	pad_len_exp = cmd_info->msg_param ?
		(frag_len - 1) * NAU8310_DSP_DATA_BYTE - (data_size - data_count) : 0;
	return nau8310_validate_trailing(cp, frag_len, pad_len_exp);
}

/**
 * nau8310_send_dsp_command - Send command to DSP
 *
 * @component:  component to register
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
static int nau8310_send_dsp_command(struct snd_soc_component *cp, int cmd_id,
	struct nau8310_kcs_setup *kcs_setup)
{
	const struct nau8310_cmd_info *cmd_info;
	int ret = 0, frag_len = 0;

	cmd_info = &nau8310_dsp_cmd_table[cmd_id];
	if ((cmd_info->msg_param && !kcs_setup->set_len) ||
		(cmd_info->setup_data && !kcs_setup->set_kcs_data) ||
		(cmd_info->reply_data && !kcs_setup->get_data))
		return -EFAULT;

	/* Read up to 1kB data because the LEN field to request data is 10-bits
	 * long; and not beyond 3kB offset.
	 */
	if (cmd_id == NAU8310_DSP_CMD_GET_KCS_SETUP &&
		(kcs_setup->set_len > NAU8310_DSP_KCS_DAT_LEN_MAX ||
		kcs_setup->set_kcs_offset > NAU8310_DSP_KCS_OFFSET_MAX))
		return -ERANGE;

	/* one fragment for offset and size parameters
	 * one fragment for a postamble fragment
	 */
	if (cmd_info->msg_param)
		frag_len += 2;

	/* fragments for KCS setup writen */
	if (cmd_info->setup_data)
		frag_len += DIV_ROUND_UP(kcs_setup->set_len, NAU8310_DSP_DATA_BYTE);

	ret = nau8310_message_to_dsp(cp, cmd_info, frag_len,
		kcs_setup->set_kcs_offset, kcs_setup->set_len,
		kcs_setup->set_kcs_data);
	if (ret)
		return ret;

	return nau8310_reply_from_dsp(cp, cmd_info, kcs_setup->get_len,
		kcs_setup->get_data);
}

static inline int nau8310_dsp_exec_command(struct snd_soc_component *cp, int cmd_id,
	int offset, int set_len, void *set_data, int get_len, void *get_data)
{
	struct nau8310_kcs_setup kcs_setup = {
		.set_kcs_offset = offset,
		.set_len = set_len,
		.set_kcs_data = set_data,
		.get_len = get_len,
		.get_data = get_data,
	};

	return nau8310_send_dsp_command(cp, cmd_id, &kcs_setup);
}

/**
 * nau8310_dsp_kcs_setup - Send KCS setup command to DSP
 *
 * @component:  component to register
 * @offset: address offset relative to KCS start
 * @size: size of data writen to KCS
 * @data: data writen to KCS setup
 *
 * The function sends KCS setup command to DSP for
 * setting KCS configuration. The maximum size that you can transfer into
 * the DSP is 96 bytes. Therefore, the driver has to split the data into
 * 96 bytes chucks, if the setup configuration over the threshold.
 */
static int nau8310_dsp_kcs_setup(struct snd_soc_component *cp, int offset, int size,
	const void *data)
{
	u8 *data_buf = (u8 *)data;
	unsigned int kcs_rst;
	int retries = 0, ret, data_len, data_rem, addr_offset;

	/* Limit full load of KCS_SETUP data and not beyond 3kB offset. */
	if (!data || size > NAU8310_DSP_KCS_DAT_LEN_MAX ||
		offset > NAU8310_DSP_KCS_OFFSET_MAX)
		return -EINVAL;

	/* sending fragments for KCS setup */
	addr_offset = offset;
	data_rem = size;

	while (data_rem) {
		data_len = min(data_rem, NAU8310_DSP_KCS_TX_MAX);

		ret = nau8310_dsp_exec_command(cp, NAU8310_DSP_CMD_SET_KCS_SETUP,
			addr_offset, data_len, (void *)data_buf, 0, &kcs_rst);
		if (ret) {
			if (retries++ < NAU8310_DSP_RETRY_MAX)
				continue;
			return ret;
		}

		data_buf += (u8)data_len;
		addr_offset += data_len;
		data_rem -= data_len;

		/* checking KCS result */
		ret = nau8310_dsp_exec_command(cp, NAU8310_DSP_CMD_GET_KCS_RSLTS,
			0, NAU8310_DSP_DATA_BYTE, NULL,
			NAU8310_DSP_DATA_BYTE, &kcs_rst);
		if (ret)
			return ret;
		if (kcs_rst != NAU8310_DSP_KCS_RSLTS_SUCCESS)
			return -EINVAL;
	}

	return 0;
}

#ifdef DEBUG
static int nau8310_dsp_get_cmd_put(struct snd_soc_component *cp, int cmd, int *value)
{
	struct device *dev = cp->dev;
	int ret;

	dev_dbg(dev, "send DSP command %s", dsp_cmd_table[cmd]);

	ret = nau8310_dsp_exec_command(cp, cmd, 0, sizeof(int), NULL,
		sizeof(int), value);
	if (ret) {
		dev_err(dev, "do command fail (%d)", ret);
		return ret;
	}

	return 0;
}

static int nau8310_dsp_info_get_put(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *cp = snd_soc_kcontrol_component(kcontrol);
	struct device *dev = cp->dev;
	struct nau8310 *nau8310 = snd_soc_component_get_drvdata(cp);
	struct soc_mixer_control *mc = (struct soc_mixer_control *)kcontrol->private_value;
	unsigned int idx = mc->shift;
	int val = ucontrol->value.integer.value[0];
	int ret, cmd ,dsp_val;
	const char *name = kcontrol->id.name;

	if (mc->max == 1 && (unsigned int)val > 1)
		return -EINVAL; 

	if (nau8310->dsp_switch_state[idx] == val)
		return 0;

	if (val) {
		if (!nau8310->load_fw_done) {
			dev_warn(dev, "DSP firmware is not ready yet!");
			return -EBUSY;
		}

		if (strstr(name, "counter"))
			cmd = NAU8310_DSP_CMD_GET_COUNTER;
		else if (strstr(name, "frame status"))
			cmd = NAU8310_DSP_CMD_GET_FRAME_STATUS;
		else if (strstr(name, "revision"))
			cmd = NAU8310_DSP_CMD_GET_REVISION;
		else
			return -EINVAL;

		ret = nau8310_dsp_get_cmd_put(cp, cmd, &dsp_val);
		if (ret)
			return ret;

		dev_info(dev, "DSP name: %s, dsp_val: 0x%x",
			dsp_cmd_table[cmd], dsp_val);
	}
	nau8310->dsp_switch_state[idx] = val;

	return 1;
}

static int nau8310_dsp_get_kcs_setup_put(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *cp = snd_soc_kcontrol_component(kcontrol);
	struct nau8310 *nau8310 = snd_soc_component_get_drvdata(cp);
	struct nau8310_kcs_setup kcs_setup_comp, *kcs_setup = &kcs_setup_comp;
	int ret, i, buf_off, buf_len, count;
	char *data, buf[100];

	if (!nau8310->load_fw_done) {
		dev_warn(cp->dev, "DSP firmware is not ready yet!");
		return -EBUSY;
	}

	data = kcalloc(nau8310->kcs_setup_size, sizeof(char), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	if (nau8310->kcs_setup_size == 0) {
		kfree(data);
		ret = -EINVAL;
		dev_err(cp->dev, "KCS of DSP not load yet (%d)\n", ret);
		return ret;
	}
	buf_off = 0;
	buf_len = nau8310->kcs_setup_size;
	dev_info(nau8310->dev, "Send DSP command %s (OFF %d, LEN %d)\n",
		dsp_cmd_table[NAU8310_DSP_CMD_GET_KCS_SETUP], buf_off, buf_len);
	kcs_setup->set_kcs_offset = buf_off;
	kcs_setup->get_len = kcs_setup->set_len = buf_len;
	kcs_setup->get_data = data;
	ret = nau8310_send_dsp_command(cp,
		NAU8310_DSP_CMD_GET_KCS_SETUP, kcs_setup);
	if (ret) {
		dev_err(cp->dev, "Send DSP command %s fail (%d)\n",
			dsp_cmd_table[NAU8310_DSP_CMD_GET_KCS_SETUP], ret);
	} else {
		dev_dbg(cp->dev, "DSP KCS result:\n");
		for (i = 0; i < kcs_setup->get_len; i += 16) {
			dev_dbg(cp->dev,	"%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x",
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
		dev_dbg(cp->dev, "%s", buf);
		dev_info(cp->dev, "Get length %d of kcs_setup\n",
			 kcs_setup->get_len);
	}
	kfree(data);

	return 0;
}

static int nau8310_dsp_set_kcs_setup_put(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *cp = snd_soc_kcontrol_component(kcontrol);
	struct nau8310 *nau8310 = snd_soc_component_get_drvdata(cp);

	if (!nau8310->load_fw_done) {
		dev_warn(cp->dev, "DSP firmware is not ready yet!");
		return -EBUSY;
	}

	nau8310_sw_reset_chip(nau8310->regmap);
	/* wait for the power ready */
	msleep(200);
	regmap_update_bits(nau8310->regmap, NAU8310_R1A_DSP_CORE_CTRL2,
		NAU8310_DSP_RUNSTALL, 0);

	return nau8310_dsp_set_kcs_setup(cp);
}

static int nau8310_dsp_clk_restart_put(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *cp = snd_soc_kcontrol_component(kcontrol);
	struct nau8310 *nau8310 = snd_soc_component_get_drvdata(cp);
	struct nau8310_kcs_setup kcs_setup_comp, *kcs_setup = &kcs_setup_comp;
	int ret;

	dev_info(cp->dev, "Send DSP command %s\n",
		 dsp_cmd_table[NAU8310_DSP_CMD_CLK_RESTART]);
	if (!nau8310->load_fw_done) {
		dev_warn(cp->dev, "DSP firmware is not ready yet!");
		return -EBUSY;
	}

	ret = nau8310_send_dsp_command(cp,
		NAU8310_DSP_CMD_CLK_RESTART, kcs_setup);
	if (ret)
		dev_err(cp->dev, "Send DSP command %s fail (%d)\n",
			dsp_cmd_table[NAU8310_DSP_CMD_CLK_RESTART], ret);

	return 0;
}

static int nau8310_dsp_clk_stop_put(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *cp = snd_soc_kcontrol_component(kcontrol);
	struct nau8310 *nau8310 = snd_soc_component_get_drvdata(cp);
	struct nau8310_kcs_setup kcs_setup_comp, *kcs_setup = &kcs_setup_comp;
	int ret;

	dev_info(cp->dev, "Send DSP command %s\n",
		 dsp_cmd_table[NAU8310_DSP_CMD_CLK_STOP]);
	if (!nau8310->load_fw_done) {
		dev_warn(cp->dev, "DSP firmware is not ready yet!");
		return -EBUSY;
	}

	ret = nau8310_send_dsp_command(cp,
		NAU8310_DSP_CMD_CLK_STOP, kcs_setup);
	if (ret)
		dev_err(cp->dev, "Send DSP command %s fail (%d)\n",
			dsp_cmd_table[NAU8310_DSP_CMD_CLK_STOP], ret);

	return 0;
}

int nau8310_dsp_cmd_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *cp = snd_soc_kcontrol_component(kcontrol);
	struct nau8310 *nau8310 = snd_soc_component_get_drvdata(cp);
	struct soc_mixer_control *mc = (struct soc_mixer_control *)kcontrol->private_value;

	unsigned int idx = mc->shift; 
	ucontrol->value.integer.value[0] = nau8310->dsp_switch_state[idx];
	return 0;
}

static const struct snd_kcontrol_new nau8310_dsp_snd_controls[] = {
	SOC_SINGLE_EXT("DSP get counter command", SND_SOC_NOPM,
		NAU8310_DSP_CMD_COUNTER, 1, 0,
		nau8310_dsp_cmd_get, nau8310_dsp_info_get_put),
	SOC_SINGLE_EXT("DSP get frame status command", SND_SOC_NOPM,
		NAU8310_DSP_CMD_GET_FRAME, 1, 0,
		nau8310_dsp_cmd_get, nau8310_dsp_info_get_put),
	SOC_SINGLE_EXT("DSP get revision command", SND_SOC_NOPM,
		NAU8310_DSP_CMD_REVISION, 1, 0,
		nau8310_dsp_cmd_get, nau8310_dsp_info_get_put),
	SOC_SINGLE_EXT("DSP get KCS setup command", SND_SOC_NOPM,
		NAU8310_DSP_CMD_GET_KCS_DATA, 1, 0,
		nau8310_dsp_cmd_get, nau8310_dsp_get_kcs_setup_put),
	SOC_SINGLE_EXT("DSP set KCS setup command", SND_SOC_NOPM,
		NAU8310_DSP_CMD_SET_KCS_DATA, 1, 0,
		nau8310_dsp_cmd_get, nau8310_dsp_set_kcs_setup_put),
	SOC_SINGLE_EXT("DSP clock restart command", SND_SOC_NOPM,
		NAU8310_DSP_CMD_CLOCK_RESTART, 1, 0,
		nau8310_dsp_cmd_get, nau8310_dsp_clk_restart_put),
	SOC_SINGLE_EXT("DSP clock stop command", SND_SOC_NOPM,
		NAU8310_DSP_CMD_CLOCK_STOP, 1, 0,
		nau8310_dsp_cmd_get, nau8310_dsp_clk_stop_put),
};
#endif

static int nau8310_dsp_snd_soc_dapm_put(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dapm_context *dapm = snd_soc_dapm_kcontrol_dapm(kcontrol);
	struct snd_soc_component *component = snd_soc_dapm_to_component(dapm);
	struct nau8310 *nau8310 = snd_soc_component_get_drvdata(component);
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
		dev_err(component->dev, "Failed to read register: %d\n", ret);
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
		ret = nau8310_dsp_set_kcs_setup(component);
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
	struct snd_soc_component *cp = snd_soc_dapm_to_component(w->dapm);
	struct nau8310 *nau8310 = snd_soc_component_get_drvdata(cp);
	int ret = 0;

	if (SND_SOC_DAPM_EVENT_ON(event)) {
		ret = nau8310_dsp_exec_command(cp, NAU8310_DSP_CMD_CLK_RESTART, 0, 0, NULL, 0, NULL);
		/* Switch the clock source of DSP to MCLK. */
		regmap_update_bits(nau8310->regmap, NAU8310_R04_ENA_CTRL,
			NAU8310_DSP_SEL_OSC, 0);
	} else if (SND_SOC_DAPM_EVENT_OFF(event))
		ret = nau8310_dsp_exec_command(cp, NAU8310_DSP_CMD_CLK_STOP, 0, 0, NULL, 0, NULL);

	return ret;
}

static const struct snd_soc_dapm_widget nau8310_dsp_dapm_widgets[] = {
	SND_SOC_DAPM_PGA("DSP", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_SWITCH("DAC data from DSP", SND_SOC_NOPM,
		0, 0, &nau8310_dacdat_select_dsp),
	SND_SOC_DAPM_SUPPLY("DSP Clock", SND_SOC_NOPM, 0, 0,
		nau8310_dsp_clock_event, SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
};

static const struct snd_soc_dapm_route nau8310_dsp_dapm_routes[] = {
	{ "DSP", NULL, "Capture Sense" },
	{ "DAC data from DSP", "Switch", "DSP" },
	{ "DAC", NULL, "DAC data from DSP" },
	{ "DSP", NULL, "DSP Clock" },
};

static int nau8310_dsp_set_kcs_setup(struct snd_soc_component *cp)
{
	struct nau8310 *nau8310 = snd_soc_component_get_drvdata(cp);
	struct nau8310_kcs_setup kcs_setup_comp, *kcs_setup = &kcs_setup_comp;
	const struct firmware *fw;
	int ret, status = 0, buf_off, buf_len;

	dev_dbg(cp->dev, "Send DSP command %s\n",
		dsp_cmd_table[NAU8310_DSP_CMD_GET_FRAME_STATUS]);

	kcs_setup->get_len = kcs_setup->set_len = sizeof(status);
	kcs_setup->get_data = (void *)&status;

	ret = nau8310_send_dsp_command(cp, NAU8310_DSP_CMD_GET_FRAME_STATUS, kcs_setup);
	if (ret) {
		dev_err(cp->dev, "Send DSP command %s fail (%d)\n",
			dsp_cmd_table[NAU8310_DSP_CMD_GET_FRAME_STATUS], ret);
		goto err;
	}

	if (!(status & NAU8310_DSP_ALGO_OK)) {
		dev_err(cp->dev, "Algorithm of DSP is not ready, status %x\n",
			status);
		ret = -EIO;
		goto err;
	}

	dev_info(cp->dev, "Algorithm of DSP is ready, status %x\n", status);

	dev_dbg(cp->dev, "Request firmware and wait until finish.\n");
	ret = request_firmware(&fw, NAU8310_DSP_FIRMWARE, cp->dev);
	if (ret) {
		dev_err(cp->dev, "Failed to load firmware (%d)\n", ret);
		goto err;
	}
	regmap_update_bits(nau8310->regmap, NAU8310_R13_MUTE_CTRL,
		NAU8310_SOFT_MUTE, NAU8310_SOFT_MUTE);
	buf_off = 0;
	buf_len = nau8310->kcs_setup_size = fw->size;
	dev_dbg(cp->dev, "Send DSP command %s (OFF %d, LEN %d)\n",
		dsp_cmd_table[NAU8310_DSP_CMD_SET_KCS_SETUP],
		buf_off, buf_len);
	ret = nau8310_dsp_kcs_setup(cp, buf_off, buf_len, fw->data);
	if (ret) {
		dev_err(cp->dev, "Send DSP command %s fail (%d)\n",
			dsp_cmd_table[NAU8310_DSP_CMD_SET_KCS_SETUP],
			ret);
		goto err_loaded;
	}
	regmap_update_bits(nau8310->regmap, NAU8310_R13_MUTE_CTRL,
		NAU8310_SOFT_MUTE, 0);
	release_firmware(fw);
	nau8310->load_fw_done = true;

	return 0;

err_loaded:
	regmap_update_bits(nau8310->regmap, NAU8310_R13_MUTE_CTRL,
		NAU8310_SOFT_MUTE, 0);
	if (fw)
		release_firmware(fw);
err:
	return ret;
}

int nau8310_dsp_init(struct snd_soc_component *cp)
{
	return nau8310_dsp_set_kcs_setup(cp);
}

int nau8310_dsp_setup_controls(struct snd_soc_component *cp)
{
	struct nau8310 *nau8310 = snd_soc_component_get_drvdata(cp);
	struct snd_soc_dapm_context *dapm = nau8310->dapm;
	int ret;

#ifdef DEBUG
	ret = snd_soc_add_component_controls(cp, nau8310_dsp_snd_controls,
		ARRAY_SIZE(nau8310_dsp_snd_controls));
	if (ret) {
		dev_err(cp->dev, "Add DSP control fail (%d)\n", ret);
		goto err;
	}
#endif

	ret = snd_soc_dapm_new_controls(dapm, nau8310_dsp_dapm_widgets,
		ARRAY_SIZE(nau8310_dsp_dapm_widgets));
	if (ret) {
		dev_err(cp->dev, "Add DSP widget fail (%d)\n", ret);
		goto err;
	}

	ret = snd_soc_dapm_add_routes(dapm, nau8310_dsp_dapm_routes,
		ARRAY_SIZE(nau8310_dsp_dapm_routes));
	if (ret) {
		dev_err(cp->dev, "Add DSP route fail (%d)\n", ret);
		goto err;
	}

	return 0;
err:
	snd_soc_dapm_del_routes(dapm, nau8310_dsp_dapm_routes,
		ARRAY_SIZE(nau8310_dsp_dapm_routes));
	return ret;
}

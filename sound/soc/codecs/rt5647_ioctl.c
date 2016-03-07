/*
 * rt5647_ioctl.h  --  RT5647 ALSA SoC audio driver IO control
 *
 * Copyright 2012 Realtek Microelectronics
 * Author: Bard <bardliao@realtek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/spi/spi.h>
#include <sound/soc.h>
#include "rt_codec_ioctl.h"
#include "rt5647_ioctl.h"
#include "rt5647.h"

hweq_t hweq_param[] = {
	{/* NORMAL */
		{0},
		{0},
		0x0000,
	},
	{/* headset mic ADC */
		{0},
		{0x1ec4},
		0x00a0,
	},
	{/* Reset Input*/
		{0},
		{0x1c10},
		0x0000,
	},
};
#define RT5647_HWEQ_LEN ARRAY_SIZE(hweq_param)

/*
	EQ_CH_DACL = 0,
	EQ_CH_DACR,
	EQ_CH_HS_MIC_ADC,
	EQ_CH_RESET_INPUT,
*/
int eqreg[EQ_CH_NUM][EQ_REG_NUM] = {
	{0xc4, 0xc5, 0xc6},
	{0xc7, 0xc8, 0xc9},
	{0xdc},
	{0xdc},
};

int rt5647_update_eqmode(
	struct snd_soc_codec *codec, int channel, int mode)
{
	struct rt_codec_ops *ioctl_ops = rt_codec_get_ioctl_ops();
	int i, upd_reg, reg, mask;

	if (codec == NULL ||  mode >= RT5647_HWEQ_LEN)
		return -EINVAL;

	dev_dbg(codec->dev, "%s(): mode=%d\n", __func__, mode);

	if (mode != NORMAL) {
		for (i = 0; i <= EQ_REG_NUM; i++) {
			if (eqreg[channel][i]) {
				unsigned int reg = eqreg[channel][i];
				hweq_param[mode].reg[i] = reg;
			} else
				break;
		}

		for (i = 0; i <= EQ_REG_NUM; i++) {
			if (hweq_param[mode].reg[i])
				ioctl_ops->index_write(codec, hweq_param[mode].reg[i],
						hweq_param[mode].value[i]);
			else
				break;
		}
	}
	switch (channel) {
	case EQ_CH_DACL:
		reg = RT5647_EQ_CTRL2;
		mask = 0x11fe;
		upd_reg = RT5647_EQ_CTRL1;
		break;
	case EQ_CH_DACR:
		reg = RT5647_EQ_CTRL2;
		mask = 0x22fe;
		upd_reg = RT5647_EQ_CTRL1;
		break;
	case EQ_CH_HS_MIC_ADC:
	case EQ_CH_RESET_INPUT:
		reg = RT5647_ADC_EQ_CTRL2;
		mask = 0x01bf;
		upd_reg = RT5647_ADC_EQ_CTRL1;
		break;
	default:
		pr_err("Invalid EQ channel\n");
		return -EINVAL;
	}
	snd_soc_update_bits(codec, reg, mask, hweq_param[mode].ctrl);
	snd_soc_update_bits(codec, upd_reg,
		RT5647_EQ_UPD, RT5647_EQ_UPD);
	snd_soc_update_bits(codec, upd_reg, RT5647_EQ_UPD, 0);

	return 0;
}

int rt5647_ioctl_common(struct snd_hwdep *hw, struct file *file,
			unsigned int cmd, unsigned long arg)
{
	struct snd_soc_codec *codec = hw->private_data;
	struct rt_codec_cmd __user *_rt_codec = (struct rt_codec_cmd *)arg;
	struct rt_codec_cmd rt_codec;
	/* struct rt_codec_ops *ioctl_ops = rt_codec_get_ioctl_ops(); */
	int *buf;
	static int eq_mode[EQ_CH_NUM];

	if (copy_from_user(&rt_codec, _rt_codec, sizeof(rt_codec))) {
		dev_err(codec->dev, "copy_from_user faild\n");
		return -EFAULT;
	}
	dev_dbg(codec->dev, "%s(): rt_codec.number=%d, cmd=%d\n",
			__func__, rt_codec.number, cmd);
	buf = kmalloc(sizeof(*buf) * rt_codec.number, GFP_KERNEL);
	if (buf == NULL)
		return -ENOMEM;
	if (copy_from_user(buf, rt_codec.buf, sizeof(*buf) * rt_codec.number))
		goto err;


	switch (cmd) {
	case RT_SET_CODEC_HWEQ_IOCTL:
		if (eq_mode == *buf)
			break;
		eq_mode[*buf] = *(buf + 1);
		rt5647_update_eqmode(codec, eq_mode[*buf], *buf);
		break;

	case RT_GET_CODEC_ID:
		*buf = snd_soc_read(codec, RT5647_VENDOR_ID2);
		if (copy_to_user(rt_codec.buf, buf, sizeof(*buf) * rt_codec.number))
			goto err;
		break;
	default:
		break;
	}

	kfree(buf);
	return 0;

err:
	kfree(buf);
	return -EFAULT;
}
EXPORT_SYMBOL_GPL(rt5647_ioctl_common);
